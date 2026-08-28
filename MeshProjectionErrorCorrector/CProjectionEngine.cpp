// Copyright Johnlyon
//
// CProjectionEngine — unified projection/coordinate engine for 3D Tiles pipeline

#include "CProjectionEngine.h"
#include "CoordinateTransform.hpp"
#include "GeodeticMath.h"
#include "Constants.h"
#include "IGeoreferencing.h"
#include "GeoreferencingFactory.h"
#include <assimp/scene.h>
#include <assimp/mesh.h>
#include <proj/proj.h>

#include <cctype>
#include <Eigen/Dense>
#include "Log.hpp"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <unordered_set>

// ===========================================================================
// CProjectionEngine
// ===========================================================================

CProjectionEngine::CProjectionEngine()
    : m_a(Geodetic::WGS84_SEMI_MAJOR_AXIS)
    , m_f_inv(Geodetic::CGCS2000_INV_FLATTENING)
    , m_lambda0(0)
    , m_falseE(0)
    , m_falseN(0)
    , m_k0(ProjectionDefaults::SCALE_FACTOR)
    , m_originX(0)
    , m_originY(0)
    , m_originZ(0)
    , m_hasProjection(false)
{
}

CProjectionEngine::~CProjectionEngine()
{
    // m_ownedGeoref auto-released by unique_ptr
}

void CProjectionEngine::Reset()
{
    m_a = Geodetic::WGS84_SEMI_MAJOR_AXIS;
    m_f_inv = Geodetic::CGCS2000_INV_FLATTENING;
    m_lambda0 = 0;
    m_falseE = 0;
    m_falseN = 0;
    m_k0 = ProjectionDefaults::SCALE_FACTOR;
    m_originX = m_originY = m_originZ = 0;
    m_hasProjection = false;
    m_ownedGeoref.reset();
    m_georef = nullptr;
}

// ---------------------------------------------------------------------------
// Initialization
// ---------------------------------------------------------------------------

// Expand an inline "EPSG:<code>" spec to WKT via PROJ so it can flow through
// the existing WKT parameter parser. Returns "" on failure. Geographic CRSs
// are rejected: the engine's math consumes projected (easting/northing)
// coordinates; geographic input belongs to the GeoJSON converter.
static std::string ExpandEPSGToWKT(const std::string& spec)
{
    PJ_CONTEXT* ctx = proj_context_create();
    if (!ctx) return "";

    std::string wkt;
    PJ* crs = proj_create(ctx, spec.c_str());
    if (crs && proj_is_crs(crs))
    {
        PJ_TYPE type = proj_get_type(crs);
        if (type == PJ_TYPE_GEOGRAPHIC_2D_CRS || type == PJ_TYPE_GEOGRAPHIC_3D_CRS)
        {
            std::cerr << "[CProjectionEngine] " << spec
                      << " is a geographic CRS; a projected CRS is required here"
                      << std::endl;
        }
        else
        {
            const char* text = proj_as_wkt(ctx, crs, PJ_WKT1_ESRI, nullptr);
            if (text) wkt = text;
        }
    }
    if (crs) proj_destroy(crs);
    proj_context_destroy(ctx);
    return wkt;
}

bool CProjectionEngine::LoadProjection(const std::string& prjFileOrSpec)
{
    // Existing file: read it (classic .prj path).
    std::ifstream f(prjFileOrSpec);
    if (f)
    {
        std::string content((std::istreambuf_iterator<char>(f)),
                             std::istreambuf_iterator<char>());
        return LoadProjectionFromString(content);
    }

    // Not a file - treat as an inline CRS spec: WKT, PROJ string, or EPSG code.
    if (prjFileOrSpec.empty())
        return false;

    // A .prj-looking path that could not be opened is a user error - keep
    // the classic message instead of confusing it with an inline spec.
    if (prjFileOrSpec.size() >= 4)
    {
        std::string ext = prjFileOrSpec.substr(prjFileOrSpec.size() - 4);
        for (auto& c : ext) c = static_cast<char>(std::tolower(c));
        if (ext == ".prj")
        {
            std::cerr << "[CProjectionEngine] Cannot open projection file: "
                      << prjFileOrSpec << std::endl;
            return false;
        }
    }

    std::string lower;
    for (size_t i = 0; i < prjFileOrSpec.size() && i < 4; ++i)
        lower += static_cast<char>(std::tolower(prjFileOrSpec[i]));
    if (lower == "enu:")
    {
        std::cerr << "[CProjectionEngine] ENU specs are not supported here -"
                     " use 'mgo osgb --enu <lat>,<lon>[,<h>]' or 'mgo geojson'"
                  << std::endl;
        return false;
    }
    if (lower == "epsg:")
    {
        std::string wkt = ExpandEPSGToWKT(prjFileOrSpec);
        if (wkt.empty())
        {
            std::cerr << "[CProjectionEngine] Cannot expand CRS spec: "
                      << prjFileOrSpec << std::endl;
            return false;
        }
        return LoadProjectionFromString(wkt);
    }

    std::cerr << "[CProjectionEngine] Projection file not found, treating "
                 "argument as inline CRS spec: " << prjFileOrSpec << std::endl;
    return LoadProjectionFromString(prjFileOrSpec);
}

// Normalize PROJ string for PROJ 9.x compatibility.
// Strips +no_defs, adds +type=crs, replaces +a/+rf with +ellps.
// Returns cleaned string (or original if already WKT or clean).
static std::string NormalizePROJString(const std::string& content)
{
    // WKT — PROJ 9.x handles natively, no normalization needed
    if (content.find("PROJCS[") != std::string::npos ||
        content.find("GEOGCS[") != std::string::npos)
        return content;

    // Old-style PROJ string (+proj=...) — apply PROJ 9.x fixes
    if (content.find("+proj=") == std::string::npos)
        return content;

    std::string result = content;

    // Remove deprecated +no_defs (PROJ 9.x rejects it)
    size_t pos;
    while ((pos = result.find("+no_defs")) != std::string::npos) {
        size_t end = pos + 7;
        // Eat trailing whitespace
        while (end < result.size() && result[end] == ' ') ++end;
        result.erase(pos, end - pos);
    }

    // Add +type=crs if not present (required by PROJ 9.x)
    if (result.find("+type=crs") == std::string::npos)
        result += " +type=crs";

    // Replace +a=6378137 +rf=298.257223563 with +ellps=WGS84 (deprecated syntax)
    {
        // Try both orderings
        std::string target1 = "+a=6378137 +rf=298.257223563";
        std::string target2 = "+rf=298.257223563 +a=6378137";
        size_t p1 = result.find(target1);
        size_t p2 = result.find(target2);
        if (p1 != std::string::npos) {
            result.replace(p1, target1.length(), "+ellps=WGS84");
        } else if (p2 != std::string::npos) {
            result.replace(p2, target2.length(), "+ellps=WGS84");
        }
    }

    return result;
}

bool CProjectionEngine::LoadProjectionFromString(const std::string& rawContent)
{
    // Normalize for PROJ 9.x compatibility (handles +no_defs, +type=crs, etc.)
    std::string content = NormalizePROJString(rawContent);

    // Helper: extract a WKT PARAMETER value, trying TitleCase first then lowercase.
    // Handles both WKT1 (e.g. "Central_Meridian") and WKT2 (e.g. "central_meridian").
    auto extractParam = [&content](const std::string& titleCase,
                                    const std::string& lowerCase,
                                    double defaultValue) -> double {
        for (const auto& name : {titleCase, lowerCase}) {
            auto pos = content.find(name);
            if (pos != std::string::npos) {
                pos = content.find(',', pos);
                if (pos != std::string::npos) {
                    try { return std::stod(content.substr(pos + 1)); }
                    catch (const std::exception&) {}
                }
            }
        }
        return defaultValue;
    };

    // Extract SPHEROID parameters: SPHEROID["...", a, 1/f]
    // Handle both "SPHEROID[" and "SPHEROID [" (with optional space).
    auto pos = content.find("SPHEROID[");
    if (pos == std::string::npos)
        pos = content.find("SPHEROID [");
    if (pos == std::string::npos)
        pos = content.find("spheroid[");
    if (pos == std::string::npos)
        pos = content.find("spheroid [");
    if (pos != std::string::npos)
    {
        auto comma1 = content.find(',', pos);
        if (comma1 != std::string::npos)
        {
            auto comma2 = content.find(',', comma1 + 1);
            if (comma2 != std::string::npos)
            {
                m_a = std::stod(content.substr(comma1 + 1));
                m_f_inv = std::stod(content.substr(comma2 + 1));
            }
        }
    }

    m_lambda0 = extractParam("Central_Meridian", "central_meridian", m_lambda0);
    m_falseE  = extractParam("False_Easting", "false_easting", m_falseE);
    m_falseN  = extractParam("False_Northing", "false_northing", m_falseN);
    m_k0      = extractParam("Scale_Factor", "scale_factor", m_k0);

    m_hasProjection = true;
    MGO_LOG(Info) << "[CProjectionEngine] Projection loaded: a=" << m_a
                  << " 1/f=" << m_f_inv << " lambda0=" << m_lambda0
                  << " E0=" << m_falseE << " N0=" << m_falseN
                  << " k0=" << m_k0;

    // Create default georef via factory (identity = plain GK projection).
    // Pass the file CONTENT (WKT/PROJ string), not the file path — the factory
    // expects a CRS definition string, which PROJ parses natively.
    GeoreferencingOptions gopts;
    m_ownedGeoref = GeoreferencingFactory::Create(GeoreferencingType::Identity, content, gopts);
    m_georef = m_ownedGeoref.get();

    return true;
}

void CProjectionEngine::SetOrigin(double easting, double northing, double height)
{
    m_originX = easting;
    m_originY = northing;
    m_originZ = height;
}

void CProjectionEngine::SetGeoreferencing(IGeoreferencing* georef)
{
    m_ownedGeoref.reset();  // release default georef
    m_georef = georef;
    if (georef)
        m_hasProjection = true;
}

bool CProjectionEngine::TransformPointToECEF(double x, double y, double z,
                                              double& ex, double& ey, double& ez)
{
    if (!m_georef) {
        MGO_LOG(Error) << "[CProjectionEngine] TransformPointToECEF: no georeferencing set";
        return false;
    }
    Eigen::Vector3d src(x, y, z);
    Eigen::Vector3d geo = m_georef->Transform(src);
    Eigen::Vector3d ecef = m_georef->TransformTargetToECEF(geo);
    ex = ecef.x(); ey = ecef.y(); ez = ecef.z();
    return true;
}

// ---------------------------------------------------------------------------
// Core geodetic conversions
// ---------------------------------------------------------------------------

bool CProjectionEngine::ProjectedToGeographic(double easting, double northing,
                                              double& lat, double& lon)
{
    if (!m_georef) return false;
    Eigen::Vector3d src(easting, northing, 0.0);
    Eigen::Vector3d geo = m_georef->Transform(src);
    lon = geo.x() * Angle::DEG_TO_RAD;
    lat = geo.y() * Angle::DEG_TO_RAD;
    return true;
}

void CProjectionEngine::GeographicToProjected(double lat, double lon,
                                               double& easting, double& northing) const
{
    if (!m_georef) { easting = 0; northing = 0; return; }
    Eigen::Vector3d geo(lon * Angle::RAD_TO_DEG, lat * Angle::RAD_TO_DEG, 0.0);
    Eigen::Vector3d src = m_georef->InverseTransform(geo);
    easting = src.x();
    northing = src.y();
}

void CProjectionEngine::GeographicToECEF(double lat, double lon, double height,
                                         double& x, double& y, double& z)
{
    double f = 1.0 / m_f_inv;
    double e2 = 2.0 * f - f * f;
    GeodeticMath::GeographicToECEF(lat, lon, height, x, y, z, m_a, e2);
}

Eigen::Matrix3d CProjectionEngine::ComputeENUToECEFRotation(double lat, double lon)
{
    return GeodeticMath::ENUToECEFRotation(lat, lon);
}

void CProjectionEngine::TransformNormalToECEF(double lat, double lon,
                                              double& nx, double& ny, double& nz)
{
    Eigen::Matrix3d R = ComputeENUToECEFRotation(lat, lon);
    Eigen::Vector3d r = R * Eigen::Vector3d(nx, ny, nz);
    nx = r.x(); ny = r.y(); nz = r.z();
}

// ---------------------------------------------------------------------------
// Composite conversions
// ---------------------------------------------------------------------------

void CProjectionEngine::ProjectedToECEF(double easting, double northing, double height,
                                        double& x, double& y, double& z)
{
    double lat, lon;
    if (!ProjectedToGeographic(easting, northing, lat, lon)) return;
    GeographicToECEF(lat, lon, height, x, y, z);
}

// ---------------------------------------------------------------------------
// Per-instance projection correction
// ---------------------------------------------------------------------------

double CProjectionEngine::ComputeProjectionError(double originE, double originN, double originZ,
                                                  double x, double y, double z,
                                                  double& dx, double& dy, double& dz,
                                                  Eigen::Matrix3d* R_correction)
{
    // x,y,z are in Assimp space (X=East, Y=Up, Z=South) relative to origin.
    // Convert to ENU (East, North, Up) for geodetic calculations.
    double enuE, enuN, enuU;
    double in_ayu[3] = {x, y, z};
    double enu[3];
    MGO::CoordinateTransform::Convert(in_ayu, MGO::CoordinateFrame::AssimpYUp,
                                 enu, MGO::CoordinateFrame::ENU);
    enuE = enu[0]; enuN = enu[1]; enuU = enu[2];

    // Origin -> geographic -> ECEF
    double lat0 = 0, lon0 = 0;
    double Tx, Ty, Tz;

    auto setIdentity = [&]() {
        dx = dy = dz = 0.0;
        if (R_correction) *R_correction = Eigen::Matrix3d::Identity();
    };

    // Unified: ProjectedToGeographic handles both georef and non-georef paths
    if (!ProjectedToGeographic(originE, originN, lat0, lon0))
    {
        setIdentity();
        return 0.0;
    }
    GeographicToECEF(lat0, lon0, originZ, Tx, Ty, Tz);

    // ENU->ECEF rotation at origin (columns = East, North, Up)
    Eigen::Matrix3d R = ComputeENUToECEFRotation(lat0, lon0);

    // True ECEF: point ENU offset + origin -> projected -> geographic -> ECEF
    // Unified: ProjectedToGeographic handles both geeref and non-geeref paths
    double trueX, trueY, trueZ;
    double lat_i, lon_i;
    if (!ProjectedToGeographic(enuE + originE, enuN + originN, lat_i, lon_i))
    {
        setIdentity();
        return 0.0;
    }
    GeographicToECEF(lat_i, lon_i, enuU + originZ, trueX, trueY, trueZ);

    // Approximate ECEF: single ENU->ECEF rotation at origin applied to ENU offset
    Eigen::Vector3d approx = R * Eigen::Vector3d(enuE, enuN, enuU)
                           + Eigen::Vector3d(Tx, Ty, Tz);

    // Delta in ECEF -> ENU local space via R^T: (dEast, dNorth, dUp)
    Eigen::Vector3d deltaECEF(trueX - approx.x(), trueY - approx.y(), trueZ - approx.z());
    Eigen::Vector3d deltaENU = R.transpose() * deltaECEF;

    // Convert ENU delta -> Assimp Y-up (East, Up, North)
    double enu_delta[3] = {deltaENU.x(), deltaENU.y(), deltaENU.z()};
    double ayu_delta[3];
    MGO::CoordinateTransform::Convert(enu_delta, MGO::CoordinateFrame::ENU,
                                 ayu_delta, MGO::CoordinateFrame::AssimpYUp);
    dx = ayu_delta[0]; dy = ayu_delta[1]; dz = ayu_delta[2];

    // Rotation correction: R_correction = R_origin^T x R_instance (optional)
    if (R_correction)
    {
        Eigen::Matrix3d Ri = ComputeENUToECEFRotation(lat_i, lon_i);
        *R_correction = R.transpose() * Ri;
    }

    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

double CProjectionEngine::ComputeInstanceProjectionDelta(double cx, double cy, double cz,
                                                         double& dx, double& dy, double& dz,
                                                         Eigen::Matrix3d& R_correction)
{
    // Thin wrapper: delegates to the decoupled ComputeProjectionError using
    // the engine's stored origin (m_originX/Y/Z).
    return ComputeProjectionError(m_originX, m_originY, m_originZ,
                                  cx, cy, cz, dx, dy, dz, &R_correction);
}

void CProjectionEngine::RebaseInstancesToCentroid(const aiScene* scene,
                                                    MeshInstance* instances,
                                                    int count)
{
    if (!scene || count == 0) return;

    int rebasedCount = 0;
    std::unordered_set<unsigned> processedMeshes;
    for (int i = 0; i < count; ++i)
    {
        auto& inst = instances[i];
        if (inst.meshIndex >= scene->mNumMeshes) continue;
        if (!processedMeshes.insert(inst.meshIndex).second) {
            // Shared mesh — already rebased by a prior instance.  Skipping
            // avoids applying the centroid shift twice (which would corrupt
            // the per-vertex deltas for the first instance).
            continue;
        }
        aiMesh* mesh = const_cast<aiMesh*>(scene->mMeshes[inst.meshIndex]);
        if (!mesh || mesh->mNumVertices == 0) continue;

        // Extract worldTransform rotation (R) and translation (T)
        // worldTransform is row-major 4x4:
        //   [R00 R01 R02 Tx]  = [0  1  2  3]
        //   [R10 R11 R12 Ty]    [4  5  6  7]
        //   [R20 R21 R22 Tz]    [8  9 10 11]
        //   [  0   0   0  1]    [12 13 14 15]
        const double R[9] = {
            inst.worldTransform[0], inst.worldTransform[1], inst.worldTransform[2],
            inst.worldTransform[4], inst.worldTransform[5], inst.worldTransform[6],
            inst.worldTransform[8], inst.worldTransform[9], inst.worldTransform[10]
        };
        const double T[3] = {
            inst.worldTransform[3], inst.worldTransform[7], inst.worldTransform[11]
        };

        // Convert world-space centroid to local-space: lc = R^T * (wc - T)
        // R is extracted row-major above, so Rm(i,j) = R[i*3+j].
        Eigen::Matrix3d Rm;
        Rm << R[0], R[1], R[2],
              R[3], R[4], R[5],
              R[6], R[7], R[8];
        Eigen::Vector3d wc(inst.vertexCentroid[0] - T[0],
                           inst.vertexCentroid[1] - T[1],
                           inst.vertexCentroid[2] - T[2]);
        Eigen::Vector3d lcv = Rm.transpose() * wc;
        const double lc[3] = { lcv.x(), lcv.y(), lcv.z() };

        // Shift vertices: v_local -= lc  (centroid moves to local origin)
        for (unsigned int vi = 0; vi < mesh->mNumVertices; ++vi)
        {
            mesh->mVertices[vi].x -= static_cast<float>(lc[0]);
            mesh->mVertices[vi].y -= static_cast<float>(lc[1]);
            mesh->mVertices[vi].z -= static_cast<float>(lc[2]);
        }

        // Update worldTransform translation: T_new = T + R * lc = world centroid
        // (R * lc = R * R^T * (wc - T) = wc - T, so T_new = T + wc - T = wc)
        inst.worldTransform[3]  = static_cast<float>(inst.vertexCentroid[0]);
        inst.worldTransform[7]  = static_cast<float>(inst.vertexCentroid[1]);
        inst.worldTransform[11] = static_cast<float>(inst.vertexCentroid[2]);

        // worldTransform translation = centroid world position (already set above).
        // bbox stays in WORLD space - vertex world positions are unchanged by
        // the rebase (local shift + worldTransform translation change cancel out).
        // Only vertexCentroid resets to 0 (centroid is now at local origin).
        for (int a = 0; a < 3; ++a)
        {
            inst.vertexCentroid[a] = 0.0;  // Now at local origin
        }
        rebasedCount++;
    }
    std::cout << "[CProjectionEngine] Rebased " << rebasedCount
              << " instances to centroid" << std::endl;
}

double CProjectionEngine::ApplyInstanceCorrection(MeshInstance& inst)
{
    // After RebaseInstancesToCentroid, local origin (0,0,0) = vertex centroid.
    // Delta is computed at the instance origin (worldTransform translation),
    // which is now the centroid's world position.
    const double cx = inst.worldTransform[3];
    const double cy = inst.worldTransform[7];
    const double cz = inst.worldTransform[11];

    double dx, dy, dz;
    Eigen::Matrix3d R_corr;
    double mag = ComputeInstanceProjectionDelta(cx, cy, cz, dx, dy, dz, R_corr);

    // 1. Apply translation correction
    inst.worldTransform[3] += static_cast<float>(dx);
    inst.worldTransform[7] += static_cast<float>(dy);
    inst.worldTransform[11] += static_cast<float>(dz);

    // 2. Convert R_corr from ENU space to Assimp space
    Eigen::Matrix3d Rc = MGO::CoordinateTransform::RotateMatrix(
        R_corr, MGO::CoordinateFrame::ENU, MGO::CoordinateFrame::AssimpYUp);

    // 3. Apply rotation correction: new_R = Rc * old_R
    // worldTransform is Assimp aiMatrix4x4 layout; kept as array math.
    float r0 = inst.worldTransform[0], r1 = inst.worldTransform[1], r2 = inst.worldTransform[2];
    float r3 = inst.worldTransform[4], r4 = inst.worldTransform[5], r5 = inst.worldTransform[6];
    float r6 = inst.worldTransform[8], r7 = inst.worldTransform[9], r8 = inst.worldTransform[10];

    inst.worldTransform[0]  = static_cast<float>(Rc(0,0)*r0 + Rc(0,1)*r3 + Rc(0,2)*r6);
    inst.worldTransform[1]  = static_cast<float>(Rc(0,0)*r1 + Rc(0,1)*r4 + Rc(0,2)*r7);
    inst.worldTransform[2]  = static_cast<float>(Rc(0,0)*r2 + Rc(0,1)*r5 + Rc(0,2)*r8);
    inst.worldTransform[4]  = static_cast<float>(Rc(1,0)*r0 + Rc(1,1)*r3 + Rc(1,2)*r6);
    inst.worldTransform[5]  = static_cast<float>(Rc(1,0)*r1 + Rc(1,1)*r4 + Rc(1,2)*r7);
    inst.worldTransform[6]  = static_cast<float>(Rc(1,0)*r2 + Rc(1,1)*r5 + Rc(1,2)*r8);
    inst.worldTransform[8]  = static_cast<float>(Rc(2,0)*r0 + Rc(2,1)*r3 + Rc(2,2)*r6);
    inst.worldTransform[9]  = static_cast<float>(Rc(2,0)*r1 + Rc(2,1)*r4 + Rc(2,2)*r7);
    inst.worldTransform[10] = static_cast<float>(Rc(2,0)*r2 + Rc(2,1)*r5 + Rc(2,2)*r8);

    // 4. Update bbox
    inst.bboxMin[0] += dx; inst.bboxMin[1] += dy; inst.bboxMin[2] += dz;
    inst.bboxMax[0] += dx; inst.bboxMax[1] += dy; inst.bboxMax[2] += dz;

    return mag;
}

void CProjectionEngine::ApplyPerInstanceProjectionCorrection(MeshInstance* instances,
                                                             int count)
{
    if (!m_hasProjection || count == 0) return;

    int correctedCount = 0;
    double maxDelta = 0.0;

    for (int i = 0; i < count; ++i)
    {
        double mag = ApplyInstanceCorrection(instances[i]);
        if (mag > maxDelta) maxDelta = mag;
        correctedCount++;
    }

    std::cout << "[CProjectionEngine] Per-instance projection correction: "
              << correctedCount << " instances, max delta = "
              << std::fixed << std::setprecision(3) << maxDelta
              << " m" << std::endl;
}

// ---------------------------------------------------------------------------
// Per-vertex projection correction (exact)
// ---------------------------------------------------------------------------

void CProjectionEngine::ApplyPerVertexProjectionCorrection(const aiScene* scene,
                                                            MeshInstance* instances,
                                                            int count)
{
    if (!m_hasProjection || count == 0 || !scene) return;

    int totalVertices = 0;
    double maxDelta = 0.0;
    std::unordered_set<unsigned> processedMeshes;

    for (int i = 0; i < count; ++i)
    {
        auto& inst = instances[i];
        if (inst.meshIndex >= scene->mNumMeshes) continue;
        if (!processedMeshes.insert(inst.meshIndex).second) {
            // Shared mesh — already corrected by a prior instance.
            // Applying per-vertex deltas twice would compound the correction.
            continue;
        }
        // Cast away const - the scene is modified in-place (consistent with
        // CMeshGroupOptimizer::SimplifyScene pattern).
        aiMesh* mesh = const_cast<aiMesh*>(scene->mMeshes[inst.meshIndex]);
        if (!mesh) continue;

        // Extract world transform
        aiMatrix4x4 world;
        world.a1 = inst.worldTransform[0];  world.a2 = inst.worldTransform[1];
        world.a3 = inst.worldTransform[2];  world.a4 = inst.worldTransform[3];
        world.b1 = inst.worldTransform[4];  world.b2 = inst.worldTransform[5];
        world.b3 = inst.worldTransform[6];  world.b4 = inst.worldTransform[7];
        world.c1 = inst.worldTransform[8];  world.c2 = inst.worldTransform[9];
        world.c3 = inst.worldTransform[10]; world.c4 = inst.worldTransform[11];
        world.d1 = inst.worldTransform[12]; world.d2 = inst.worldTransform[13];
        world.d3 = inst.worldTransform[14]; world.d4 = inst.worldTransform[15];

        // Build inverse rotation (R^T) for transforming world delta -> local delta.
        // world = [R | T; 0 | 1], so for pure rotation R, R^-1 = R^T.
        // local_new = local + R^T * delta_world
        double R[9] = {
            inst.worldTransform[0], inst.worldTransform[1], inst.worldTransform[2],
            inst.worldTransform[4], inst.worldTransform[5], inst.worldTransform[6],
            inst.worldTransform[8], inst.worldTransform[9], inst.worldTransform[10]
        };
        // R extracted row-major; Rm(i,j) = R[i*3+j]. R^T applied via transpose().
        Eigen::Matrix3d Rm;
        Rm << R[0], R[1], R[2],
              R[3], R[4], R[5],
              R[6], R[7], R[8];

        // Reset bbox/centroid for recompute after correction
        double newBboxMin[3] = { DBL_MAX, DBL_MAX, DBL_MAX };
        double newBboxMax[3] = { -DBL_MAX, -DBL_MAX, -DBL_MAX };
        double sumX = 0.0, sumY = 0.0, sumZ = 0.0;

        for (unsigned int vi = 0; vi < mesh->mNumVertices; ++vi)
        {
            // World-space position
            aiVector3D wp = world * mesh->mVertices[vi];

            // Compute per-vertex delta (curvature correction at this exact point)
            // Use decoupled ComputeProjectionError with engine's origin,
            // passing nullptr to skip R_correction (not needed for per-vertex).
            double dx, dy, dz;
            double mag = ComputeProjectionError(m_originX, m_originY, m_originZ,
                                                wp.x, wp.y, wp.z,
                                                dx, dy, dz, nullptr);
            if (mag > maxDelta) maxDelta = mag;

            // Transform delta from world space to local space: delta_local = R^T * delta_world
            Eigen::Vector3d dl = Rm.transpose() * Eigen::Vector3d(dx, dy, dz);

            // Apply delta to vertex in local space
            mesh->mVertices[vi].x += static_cast<float>(dl.x());
            mesh->mVertices[vi].y += static_cast<float>(dl.y());
            mesh->mVertices[vi].z += static_cast<float>(dl.z());

            // Track new bbox/centroid in world space (vertex + delta applied)
            double wx = wp.x + dx, wy = wp.y + dy, wz = wp.z + dz;
            if (wx < newBboxMin[0]) newBboxMin[0] = wx;
            if (wy < newBboxMin[1]) newBboxMin[1] = wy;
            if (wz < newBboxMin[2]) newBboxMin[2] = wz;
            if (wx > newBboxMax[0]) newBboxMax[0] = wx;
            if (wy > newBboxMax[1]) newBboxMax[1] = wy;
            if (wz > newBboxMax[2]) newBboxMax[2] = wz;
            sumX += wx; sumY += wy; sumZ += wz;

            totalVertices++;
        }

        // Update instance with corrected bbox/centroid
        for (int a = 0; a < 3; ++a)
        {
            inst.bboxMin[a] = newBboxMin[a];
            inst.bboxMax[a] = newBboxMax[a];
        }
        if (mesh->mNumVertices > 0)
        {
            double inv = 1.0 / static_cast<double>(mesh->mNumVertices);
            inst.vertexCentroid[0] = sumX * inv;
            inst.vertexCentroid[1] = sumY * inv;
            inst.vertexCentroid[2] = sumZ * inv;
            inst.vertexCount = mesh->mNumVertices;
        }
        // Note: worldTransform is NOT modified - the delta is baked into vertices.
    }

    std::cout << "[CProjectionEngine] Per-vertex projection correction: "
              << totalVertices << " vertices, max delta = "
              << std::fixed << std::setprecision(3) << maxDelta
              << " m" << std::endl;
}

// ---------------------------------------------------------------------------
// Root tileset transform
// ---------------------------------------------------------------------------

Eigen::Matrix4d CProjectionEngine::ComputeRootTransform()
{
    if (!m_hasProjection)
        return Eigen::Matrix4d::Identity();

    double Tx, Ty, Tz;
    double lat0 = 0, lon0 = 0;

    // Unified: ProjectedToGeographic handles both georef and non-georef paths
    if (!ProjectedToGeographic(m_originX, m_originY, lat0, lon0))
        return Eigen::Matrix4d::Identity();
    GeographicToECEF(lat0, lon0, m_originZ, Tx, Ty, Tz);

    Eigen::Matrix3d R = ComputeENUToECEFRotation(lat0, lon0);
    return MGO::CoordinateTransform::BuildRootTransform(
        R, Eigen::Vector3d(Tx, Ty, Tz));
}
