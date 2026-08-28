#define _USE_MATH_DEFINES
#include "GeoreferencingWithMultiPosition.h"
#include <iostream>
#include <vector>
#include <string>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include "UtilTools.h"
#include <proj/proj.h>

GeoreferencingWithMultiPosition::GeoreferencingWithMultiPosition(const std::string& srccrs, const std::string& targetcrs)
    : IGeoreferencing(srccrs, targetcrs)
{
}

GeoreferencingWithMultiPosition::~GeoreferencingWithMultiPosition()
{
}

void GeoreferencingWithMultiPosition::SetFitMethod(FitMethod method)
{
    m_fitMethod = method;
}

void GeoreferencingWithMultiPosition::SetPolyOrder(int order)
{
    if (order < 1 || order > 3) {
        throw std::runtime_error("Polynomial order must be 1, 2, or 3");
    }
    m_polyOrder = order;
}

// ------ 2D polynomial basis (x, y) ------
int GeoreferencingWithMultiPosition::NumBasis2D(int order)
{
    switch (order) {
        case 1: return 3;  // 1, x, y
        case 2: return 6;  // 1, x, y, x², xy, y²
        case 3: return 10; // 1, x, y, x², xy, y², x³, x²y, xy², y³
        default: return 3;
    }
}

Eigen::VectorXd GeoreferencingWithMultiPosition::Basis2D(int order, double x, double y)
{
    int n = NumBasis2D(order);
    Eigen::VectorXd b(n);
    switch (order) {
        case 1:
            b << 1.0, x, y;
            break;
        case 2: {
            double x2 = x * x;
            double xy = x * y;
            double y2 = y * y;
            b << 1.0, x, y, x2, xy, y2;
            break;
        }
        case 3: {
            double x2 = x * x;
            double xy = x * y;
            double y2 = y * y;
            double x3 = x2 * x;
            double x2y = x2 * y;
            double xy2 = x * y2;
            double y3 = y2 * y;
            b << 1.0, x, y, x2, xy, y2, x3, x2y, xy2, y3;
            break;
        }
        default:
            b << 1.0, x, y;
            break;
    }
    return b;
}

Eigen::Vector4d GeoreferencingWithMultiPosition::BasisH(double x, double y, double z)
{
    return Eigen::Vector4d(1.0, x, y, z);
}

// ------ ECEF affine fitting (original method) ------
void GeoreferencingWithMultiPosition::SolveECEF()
{
    if (!m_pj_SourceToECEF && !InitPROJPipelines()) {
        throw std::runtime_error("ECEF_Affine requires PROJ pipelines");
    }
    const size_t numPoints = m_controlPositions.size();
    if (numPoints < 4) {
        throw std::runtime_error("At least 4 control points required for ECEF affine fitting");
    }

    Eigen::MatrixXd A(3 * numPoints, 12);
    Eigen::VectorXd b(3 * numPoints);

    for (size_t i = 0; i < numPoints; ++i) {
        const auto& cp = m_controlPositions[i];
        const auto& source = cp.ecef_source_position;
        const auto& target = cp.ecef_target_position;
        const double x = source.x(), y = source.y(), z = source.z();
        const double tx = target.x(), ty = target.y(), tz = target.z();

        // Build linear system AX = b for affine: [x y z 1] → [x' y' z']
        A.block<1, 12>(3 * i, 0) << x, y, z, 1, 0, 0, 0, 0, 0, 0, 0, 0;
        b[3 * i] = tx;

        A.block<1, 12>(3 * i + 1, 0) << 0, 0, 0, 0, x, y, z, 1, 0, 0, 0, 0;
        b[3 * i + 1] = ty;

        A.block<1, 12>(3 * i + 2, 0) << 0, 0, 0, 0, 0, 0, 0, 0, x, y, z, 1;
        b[3 * i + 2] = tz;
    }

    // Solve using SVD for robust least squares
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::VectorXd solution = svd.solve(b);

    // Map solution to 4x4 matrix
    m_transform = Eigen::Matrix4d::Identity();
    Eigen::Matrix<double, 3, 4, Eigen::RowMajor> affine_transform =
        Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>>(solution.data());
    m_transform.block<3, 4>(0, 0) = affine_transform;
}

// ------ Polynomial fitting (no source CRS needed) ------
void GeoreferencingWithMultiPosition::SolvePoly()
{
    const size_t numPoints = m_controlPositions.size();
    const int nBasis = NumBasis2D(m_polyOrder);

    // Minimum points needed: ceil((2*nBasis + 4) / 3)
    const size_t minPoints = static_cast<size_t>(std::ceil((2.0 * nBasis + 4.0) / 3.0));
    if (numPoints < minPoints) {
        throw std::runtime_error(
            "Not enough control points for order " + std::to_string(m_polyOrder) +
            " polynomial. Need at least " + std::to_string(minPoints) +
            ", got " + std::to_string(numPoints));
    }

    // Total parameters: lon(nBasis) + lat(nBasis) + h(4)
    const int nParams = 2 * nBasis + 4;
    Eigen::MatrixXd A(3 * numPoints, nParams);
    Eigen::VectorXd b(3 * numPoints);

    for (size_t i = 0; i < numPoints; ++i) {
        const auto& cp = m_controlPositions[i];
        double sx = cp.orign_source_position.x();
        double sy = cp.orign_source_position.y();
        double sz = cp.orign_source_position.z();
        double tx = cp.orign_target_position.x();   // target longitude
        double ty = cp.orign_target_position.y();   // target latitude
        double tz = cp.orign_target_position.z();   // target height

        Eigen::VectorXd basis = Basis2D(m_polyOrder, sx, sy);
        Eigen::Vector4d basisHVec = BasisH(sx, sy, sz);

        // Lon equation: B₂D · coeff_lon = tx
        A.block(3 * i, 0, 1, nBasis) = basis.transpose();
        A.block(3 * i, nBasis, 1, nBasis).setZero();
        A.block(3 * i, 2 * nBasis, 1, 4).setZero();
        b[3 * i] = tx;

        // Lat equation: B₂D · coeff_lat = ty
        A.block(3 * i + 1, 0, 1, nBasis).setZero();
        A.block(3 * i + 1, nBasis, 1, nBasis) = basis.transpose();
        A.block(3 * i + 1, 2 * nBasis, 1, 4).setZero();
        b[3 * i + 1] = ty;

        // Height equation: B_H · coeff_h = tz
        A.block(3 * i + 2, 0, 1, nBasis).setZero();
        A.block(3 * i + 2, nBasis, 1, nBasis).setZero();
        A.block(3 * i + 2, 2 * nBasis, 1, 4) = basisHVec.transpose();
        b[3 * i + 2] = tz;
    }

    // Solve using SVD for robust least squares
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::VectorXd solution = svd.solve(b);

    // Extract polynomial coefficients
    m_polyLon = solution.segment(0, nBasis);
    m_polyLat = solution.segment(nBasis, nBasis);
    m_polyH = solution.segment(2 * nBasis, 4);

    // Compute and print residuals for diagnostics
    std::cerr << std::fixed << std::setprecision(6);
    std::cerr << u8"多项式拟合完成 (阶数=" << m_polyOrder
              << u8", 控制点=" << numPoints
              << u8", 参数=" << nParams << ")" << std::endl;

    double rmsLon = 0, rmsLat = 0, rmsH = 0;
    for (size_t i = 0; i < numPoints; ++i) {
        const auto& cp = m_controlPositions[i];
        double sx = cp.orign_source_position.x();
        double sy = cp.orign_source_position.y();
        double sz = cp.orign_source_position.z();

        double predLon = Basis2D(m_polyOrder, sx, sy).dot(m_polyLon);
        double predLat = Basis2D(m_polyOrder, sx, sy).dot(m_polyLat);
        double predH = BasisH(sx, sy, sz).dot(m_polyH);

        double dLon = predLon - cp.orign_target_position.x();
        double dLat = predLat - cp.orign_target_position.y();
        double dH = predH - cp.orign_target_position.z();

        rmsLon += dLon * dLon;
        rmsLat += dLat * dLat;
        rmsH += dH * dH;

        std::cerr << "  CP[" << i << "]: pred=(" << predLon << ", " << predLat << ", " << predH
                  << ")  residual=(" << dLon << ", " << dLat << ", " << dH << ")" << std::endl;
    }
    rmsLon = std::sqrt(rmsLon / numPoints);
    rmsLat = std::sqrt(rmsLat / numPoints);
    rmsH = std::sqrt(rmsH / numPoints);
    std::cerr << u8"  RMS残差: lon=" << rmsLon << u8"°, lat=" << rmsLat
              << u8"°, h=" << rmsH << "m" << std::endl;
}

// ------ Solve dispatcher ------
void GeoreferencingWithMultiPosition::Solve()
{
    if (m_fitMethod == FitMethod::DirectPoly2D) {
        SolvePoly();
    } else {
        // ECEF_Affine: InitPROJPipelines is called in SolveECEF
        SolveECEF();
    }
}

// ------ Transform dispatcher ------
Eigen::Vector3d GeoreferencingWithMultiPosition::Transform(const Eigen::Vector3d& position)
{
    if (m_fitMethod == FitMethod::DirectPoly2D) {
        // Polynomial mode: directly evaluate source (x,y,z) → geographic (lon,lat,h)
        double lon = Basis2D(m_polyOrder, position.x(), position.y()).dot(m_polyLon);
        double lat = Basis2D(m_polyOrder, position.x(), position.y()).dot(m_polyLat);
        double h   = BasisH(position.x(), position.y(), position.z()).dot(m_polyH);
        return Eigen::Vector3d(lon, lat, h);
    }

    // ECEF affine mode: source CRS -> ECEF (PROJ) -> affine -> geographic (PROJ)
    PJ_COORD ecef_source = proj_trans(m_pj_SourceToECEF, PJ_FWD, ConvertTool::EigenToCoord(position));
    if (proj_errno(m_pj_SourceToECEF)) return position;

    // Apply affine transform in ECEF space
    Eigen::Vector4d ecef_target = m_transform * Eigen::Vector4d(
        ecef_source.xyz.x, ecef_source.xyz.y, ecef_source.xyz.z, 1.0);

    // ECEF -> Target CRS (geographic)
    PJ_COORD target = proj_trans(m_pj_ECEFToTarget, PJ_FWD,
        proj_coord(ecef_target.x(), ecef_target.y(), ecef_target.z(), 0));
    if (proj_errno(m_pj_ECEFToTarget)) return position;

    // EPSG:4979 has lat-first axis order; CoordToEigenGeo swaps to (lon,lat,h).
    // PROJ 9.x CRS-to-CRS pipeline returns geographic coordinates in degrees
    // (not radians).  No unit conversion needed.
    return ConvertTool::CoordToEigenGeo(target);
}


Eigen::Vector3d GeoreferencingWithMultiPosition::InverseTransform(const Eigen::Vector3d& target_position)
{
    double lon_deg = target_position.x();
    double lat_deg = target_position.y();
    double h = target_position.z();

    if (m_fitMethod == FitMethod::DirectPoly2D) {
        // Polynomial inversion is non-trivial for non-linear polynomials.
        // Return the input unchanged with a diagnostic — this path should not
        // be called in normal operation (TileConverter uses Transform only).
        static bool s_warned = false;
        if (!s_warned) {
            s_warned = true;
            std::cerr << "[MultiPos] InverseTransform not supported for DirectPoly2D, returning identity" << std::endl;
        }
        return target_position;
    }

    // ECEF_Affine: reverse the forward Transform() pipeline via PROJ.
    // Forward:  src_proj -> PJ_FWD -> ECEF -> affine -> PJ_FWD -> tgt_geo
    // Inverse:  tgt_geo -> PJ_INV -> ECEF -> affine^-1 -> PJ_INV -> src_proj

    // Target geographic (lon,lat,h in degrees) → PROJ (lat,lon,h) for EPSG:4979.
    // PROJ 9.x expects degrees, not radians.
    PJ_COORD tgt_coord = ConvertTool::EigenToCoordGeo(target_position);
    PJ_COORD tgt_ecef = proj_trans(m_pj_ECEFToTarget, PJ_INV, tgt_coord);
    if (proj_errno(m_pj_ECEFToTarget)) return target_position;

    // Inverse affine: src_ecef = affine^-1 * [tgt_ecef; 1]
    Eigen::Vector4d ecef_h(tgt_ecef.xyz.x, tgt_ecef.xyz.y, tgt_ecef.xyz.z, 1.0);
    Eigen::Vector4d src_ecef_h = m_transform.inverse() * ecef_h;

    // ECEF -> Source CRS (projected) via PROJ (PJ_INV on source->ECEF)
    PJ_COORD src_coord = proj_coord(src_ecef_h.x(), src_ecef_h.y(), src_ecef_h.z(), 0);
    PJ_COORD src_result = proj_trans(m_pj_SourceToECEF, PJ_INV, src_coord);
    if (proj_errno(m_pj_SourceToECEF)) return target_position;

    return ConvertTool::CoordToEigen(src_result);
}

// ------ SetParameter ------
void GeoreferencingWithMultiPosition::SetParameter(const std::vector<ControlPoint>& controlPositions)
{
    m_controlPositions = controlPositions;

    if (m_fitMethod == FitMethod::DirectPoly2D) {
        std::cerr << u8"多项式拟合模式: 不依赖源坐标系 PROJ 转换" << std::endl;
        return;
    }

    // ECEF affine mode: ensure PROJ pipelines are ready.
    if (!m_pj_SourceToECEF && !InitPROJPipelines()) {
        std::cerr << "[MultiPos] InitPROJPipelines failed" << std::endl;
        return;
    }
    for (size_t i = 0; i < m_controlPositions.size(); i++)
    {
        const auto& cp = m_controlPositions[i];
        ControlPoint& mutable_cp = m_controlPositions[i];

        // Source (E,N,H) -> ECEF via PROJ
        PJ_COORD src_coord = ConvertTool::EigenToCoord(cp.orign_source_position);
        PJ_COORD src_ecef = proj_trans(m_pj_SourceToECEF, PJ_FWD, src_coord);
        mutable_cp.ecef_source_position = Eigen::Vector3d(
            src_ecef.xyz.x, src_ecef.xyz.y, src_ecef.xyz.z);

        // Target (lon_deg, lat_deg, h) -> ECEF via PROJ (EPSG:4979 lat-first order).
        // PROJ 9.x expects degrees, not radians.
        PJ_COORD tgt_coord = ConvertTool::EigenToCoordGeo(cp.orign_target_position);
        PJ_COORD tgt_ecef = proj_trans(m_pj_ECEFToTarget, PJ_INV, tgt_coord);
        mutable_cp.ecef_target_position = Eigen::Vector3d(
            tgt_ecef.xyz.x, tgt_ecef.xyz.y, tgt_ecef.xyz.z);
    }
}

// ------ CRS Auto-Detection ------

/// Estimate UTM zone number from longitude
static int lon_to_utm_zone(double lon_deg)
{
    return static_cast<int>(std::floor((lon_deg + 180.0) / 6.0)) + 1;
}

/// Build a PROJ string for UTM
static std::string utm_proj_str(int zone, bool south)
{
    std::string s = "+proj=utm +zone=" + std::to_string(zone);
    if (south) s += " +south";
    s += " +datum=WGS84 +units=m +no_defs";
    return s;
}

/// Build a PROJ string for TM with custom central meridian
static std::string tm_proj_str(double lon0, double a, double rf)
{
    std::string s = "+proj=tmerc +lon_0=" + std::to_string(lon0)
                  + " +lat_0=0 +k=1 +x_0=500000 +y_0=0"
                  + " +a=" + std::to_string(a) + " +rf=" + std::to_string(rf)
                  + " +units=m +no_defs";
    return s;
}

/// Try to reverse-infer the source CRS by evaluating candidate projections
/// against the known control point pairs.
std::vector<CRSDetectionResult> GeoreferencingWithMultiPosition::DetectSourceCRS(int maxResults)
{
    std::vector<CRSDetectionResult> results;

    const size_t numPoints = m_controlPositions.size();
    if (numPoints < 3) {
        std::cerr << u8"CRS检测: 至少需要3个控制点" << std::endl;
        return results;
    }

    // Extract source and target positions
    std::vector<Eigen::Vector3d> srcPos, tgtPos;
    double minLon = 1e10, maxLon = -1e10;
    double minLat = 1e10, maxLat = -1e10;
    double minSrcX = 1e10, maxSrcX = -1e10;

    for (const auto& cp : m_controlPositions) {
        srcPos.push_back(cp.orign_source_position);
        tgtPos.push_back(cp.orign_target_position);

        double lon = cp.orign_target_position.x();
        double lat = cp.orign_target_position.y();
        if (lon < minLon) minLon = lon;
        if (lon > maxLon) maxLon = lon;
        if (lat < minLat) minLat = lat;
        if (lat > maxLat) maxLat = lat;

        double sx = cp.orign_source_position.x();
        if (sx < minSrcX) minSrcX = sx;
        if (sx > maxSrcX) maxSrcX = sx;
    }

    double centerLon = (minLon + maxLon) / 2.0;
    double centerLat = (minLat + maxLat) / 2.0;

    std::vector<std::pair<std::string, std::string>> candidates;
    candidates.emplace_back("WGS84 geographic (EPSG:4326)", "EPSG:4326");
    candidates.emplace_back("WGS84 ECEF (EPSG:4978)", "EPSG:4978");

    // NOTE: utm_source_crs is for PJ_INV direction: EPSG:4326 → source CRS
    // which means we transform from source → lon/lat using candidate CRS
    // The EvaluateCRS function proj_transforms from source CRS → geographic

    // UTM zones covering the target area (±1 zone)
    int centerZone = lon_to_utm_zone(centerLon);
    for (int dz = -2; dz <= 2; ++dz) {
        int zone = centerZone + dz;
        if (zone >= 1 && zone <= 60) {
            bool south = centerLat < 0;
            std::string projStr = utm_proj_str(zone, south);
            // Also try the opposite hemisphere variant
            std::string desc = "UTM zone " + std::to_string(zone) + (south ? "S" : "N");
            candidates.emplace_back(desc, projStr);
            if (centerLat >= -10 && centerLat <= 10) {
                // Near equator: try both hemispheres
                std::string desc2 = "UTM zone " + std::to_string(zone) + (south ? "N" : "S");
                std::string projStr2 = utm_proj_str(zone, !south);
                candidates.emplace_back(desc2, projStr2);
            }
        }
    }

    // TM projections: estimate central meridian from source easting values
    // Source X might be easting with false easting (500000), estimate true central meridian
    if (maxSrcX - minSrcX > 100 && maxSrcX - minSrcX < 1000000) {
        double estimatedFE = 500000.0; // standard false easting
        for (double dLon = -3; dLon <= 3; dLon += 1.0) {
            double lon0 = std::round((centerLon + dLon) * 4.0) / 4.0; // round to 0.25°
            std::string descTm = "TM lon_0=" + std::to_string(lon0) + " (WGS84)";
            candidates.emplace_back(descTm, tm_proj_str(lon0, 6378137.0, 298.257223563));
        }
    }

    std::cerr << u8"\n--- CRS 自动检测 ---" << std::endl;
    std::cerr << u8"目标范围: lon=[" << minLon << ", " << maxLon << "], lat=["
              << minLat << ", " << maxLat << "]" << std::endl;
    std::cerr << u8"候选数: " << candidates.size() << std::endl;

    // Evaluate each candidate
    for (const auto& [desc, crsStr] : candidates) {
        double rms = EvaluateCRS(crsStr, srcPos, tgtPos);
        if (rms >= 0) {
            CRSDetectionResult r;
            r.crs = crsStr;
            r.rms_degrees = rms;
            r.num_points = static_cast<int>(numPoints);
            r.description = desc + " (RMS=" + std::to_string(rms) + "°)";
            results.push_back(r);
        }
    }

    // Sort by RMS (best first)
    std::sort(results.begin(), results.end(),
        [](const CRSDetectionResult& a, const CRSDetectionResult& b) {
            return a.rms_degrees < b.rms_degrees;
        });

    // Report top results
    std::cerr << u8"\nCRS检测结果 (TOP " << std::min(maxResults, (int)results.size()) << "):" << std::endl;
    for (int i = 0; i < std::min(maxResults, (int)results.size()); ++i) {
        std::cerr << "  #" << (i+1) << ": " << results[i].description << std::endl;
    }

    // Trim to maxResults
    if ((int)results.size() > maxResults)
        results.resize(maxResults);

    return results;
}

/// Evaluate a single CRS candidate by projecting source → geographic and
/// computing RMS residual against target positions.
double GeoreferencingWithMultiPosition::EvaluateCRS(
    const std::string& candidateCrs,
    const std::vector<Eigen::Vector3d>& srcPos,
    const std::vector<Eigen::Vector3d>& tgtPos)
{
    const size_t n = srcPos.size();
    if (n == 0) return -1.0;

    // Create a PROJ transform from candidate CRS → geographic (lon/lat)
    // We use proj_create_crs_to_crs with candidate as source and geographic as target
    PJ_CONTEXT* ctx = proj_context_create();
    if (!ctx) return -1.0;

    // If candidate is already geographic (EPSG:4326), no transform needed
    PJ* transform = nullptr;
    bool isGeographic = false;

    if (candidateCrs == "EPSG:4326") {
        isGeographic = true;
    } else {
        // Candidate CRS → geographic using proj_create_crs_to_crs
        // EPSG:4979 is geographic 3D (lon, lat, h) — the target CRS for evaluation
        transform = proj_create_crs_to_crs(ctx, candidateCrs.c_str(), "EPSG:4979", nullptr);
        if (!transform) {
            // Try with simpler geographic CRS
            transform = proj_create_crs_to_crs(ctx, candidateCrs.c_str(), "EPSG:4326", nullptr);
            if (!transform) {
                proj_context_destroy(ctx);
                return -1.0;
            }
        }
    }

    double sumSq = 0.0;
    int validPoints = 0;

    for (size_t i = 0; i < n; ++i) {
        double lon_proj = 0, lat_proj = 0;

        if (isGeographic) {
            // Already geographic — use target coords directly (though this is trivial)
            lon_proj = srcPos[i].x();
            lat_proj = srcPos[i].y();
        } else {
            PJ_COORD src = proj_coord(srcPos[i].x(), srcPos[i].y(), srcPos[i].z(), 0);
            PJ_COORD dst = proj_trans(transform, PJ_FWD, src);
            if (proj_errno(transform) == 0) {
                lon_proj = dst.lpz.lam * 180.0 / M_PI;
                lat_proj = dst.lpz.phi * 180.0 / M_PI;
            } else {
                proj_errno_reset(transform);
                continue;
            }
        }

        // Compute great-circle distance in degrees
        double dLon = lon_proj - tgtPos[i].x();
        double dLat = lat_proj - tgtPos[i].y();

        // Handle longitude wrap
        if (dLon > 180.0) dLon -= 360.0;
        if (dLon < -180.0) dLon += 360.0;

        sumSq += dLon * dLon + dLat * dLat;
        validPoints++;
    }

    if (transform) proj_destroy(transform);
    proj_context_destroy(ctx);

    if (validPoints < 3) return -1.0;

    // Spherical approximation: 1 degree ≈ 111320m at equator
    // For the RMS, we use degrees as the unit (simpler, no cos(lat) issue)
    return std::sqrt(sumSq / validPoints);
}
