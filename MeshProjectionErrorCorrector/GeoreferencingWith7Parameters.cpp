#define _USE_MATH_DEFINES
#include "GeoreferencingWith7Parameters.h"
#include "GeodeticMath.h"
#include "Constants.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <cmath>
#include "UtilTools.h"
#include <proj/proj.h>

GeoreferencingWith7Parameters::GeoreferencingWith7Parameters(const std::string& srccrs, const std::string& targetcrs)
    : IGeoreferencing(srccrs, targetcrs)
{
    m_pj_transform = nullptr;
    m_useProj = false;
    m_tm_lon0 = 0.0; m_tm_lat0 = 0.0; m_tm_k0 = 1.0;
    m_tm_a = Geodetic::WGS84_SEMI_MAJOR_AXIS; m_tm_rf = Geodetic::WGS84_INV_FLATTENING;
    m_tm_fe = 0.0; m_tm_fn = 0.0;
}

GeoreferencingWith7Parameters::~GeoreferencingWith7Parameters()
{
    if (m_pj_transform) {
        proj_destroy(m_pj_transform);
    }
}

void GeoreferencingWith7Parameters::SetParameter(const SevenParameter& parameter)
{
    m_parameter = parameter;
}

// Parse a +key=value from a PROJ string
static double parse_proj_param(const std::string& proj_str, const std::string& key, double default_val) {
    std::string search = " +" + key + "=";
    auto pos = proj_str.find(search);
    if (pos == std::string::npos) {
        search = " " + key + "=";
        pos = proj_str.find(search);
    }
    if (pos != std::string::npos) {
        try {
            std::string val_str = proj_str.substr(pos + search.length());
            auto end = val_str.find(' ');
            if (end != std::string::npos) val_str = val_str.substr(0, end);
            return std::stod(val_str);
        } catch (const std::exception&) {}
    }
    return default_val;
}

// Parse a PARAMETER["name",value] from a WKT string
static double parse_wkt_param(const std::string& wkt, const std::string& name, double default_val) {
    std::string search = "PARAMETER[\"" + name + "\",";
    auto pos = wkt.find(search);
    if (pos != std::string::npos) {
        try {
            std::string val_str = wkt.substr(pos + search.length());
            auto end = val_str.find_first_of(",]");
            if (end != std::string::npos) val_str = val_str.substr(0, end);
            return std::stod(val_str);
        } catch (const std::exception&) {}
    }
    return default_val;
}

// Check if string is WKT format (not PROJ string)
static bool is_wkt(const std::string& s) {
    return s.find("PROJCS[") != std::string::npos ||
           s.find("GEOGCS[") != std::string::npos;
}

// Inverse Transverse Mercator (delegates to GeodeticMath)
static void inverse_tmerc(double easting, double northing,
    double lon0_rad, double /*lat0_rad*/,
    double k0, double fe, double fn,
    double a, double rf,
    double& lon_rad, double& lat_rad)
{
    GeodeticMath::TMParams tm;
    tm.lon0_deg = lon0_rad * 180.0 / M_PI;
    tm.lat0_deg = 0.0;
    tm.k0 = k0;
    tm.fe = fe;
    tm.fn = fn;
    tm.a  = a;
    tm.rf = rf;
    GeodeticMath::InverseTM(easting, northing, tm, lon_rad, lat_rad);
}

// Geographic → ECEF (delegates to GeodeticMath)
static Eigen::Vector3d geo_to_ecef(double lon_rad, double lat_rad, double h, double a, double rf)
{
    double X, Y, Z;
    GeodeticMath::GeoToECEF(lon_rad, lat_rad, h, X, Y, Z, a, rf);
    return Eigen::Vector3d(X, Y, Z);
}

// ECEF → Geographic (delegates to GeodeticMath)
static void ecef_to_geo(const Eigen::Vector3d& ecef, double a, double rf,
    double& lon_rad, double& lat_rad, double& h)
{
    GeodeticMath::ECEFToGeo(ecef.x(), ecef.y(), ecef.z(),
                            lon_rad, lat_rad, h, a, rf);
}

// Build the Helmert rotation matrix from arc-second rotation angles.
// Supports both Coordinate Frame and Position Vector conventions.
static Eigen::Matrix3d helmert_rotation(const SevenParameter& params)
{
    double rx_rad = params.rx * Angle::ARCSEC_TO_RAD;
    double ry_rad = params.ry * Angle::ARCSEC_TO_RAD;
    double rz_rad = params.rz * Angle::ARCSEC_TO_RAD;

    double crx = cos(rx_rad), srx = sin(rx_rad);
    double cry = cos(ry_rad), sry = sin(ry_rad);
    double crz = cos(rz_rad), srz = sin(rz_rad);

    Eigen::Matrix3d R;
    if (params.isCoordinateFrame) {
        // Coordinate Frame convention: R = Rz(rz) * Ry(ry) * Rx(rx)
        R(0,0) = crz * cry;               R(0,1) = crz * sry * srx - srz * crx;   R(0,2) = crz * sry * crx + srz * srx;
        R(1,0) = srz * cry;               R(1,1) = srz * sry * srx + crz * crx;   R(1,2) = srz * sry * crx - crz * srx;
        R(2,0) = -sry;                    R(2,1) = cry * srx;                      R(2,2) = cry * crx;
    } else {
        // Position Vector convention: R = Rx(rx) * Ry(ry) * Rz(rz)
        R(0,0) = cry * crz;               R(0,1) = -cry * srz;                     R(0,2) = sry;
        R(1,0) = crx * srz + srx * sry * crz;  R(1,1) = crx * crz - srx * sry * srz;  R(1,2) = -srx * cry;
        R(2,0) = srx * srz - crx * sry * crz;  R(2,1) = srx * crz + crx * sry * srz;  R(2,2) = crx * cry;
    }
    return R;
}

// Apply 7-parameter Helmert transformation in ECEF space
// X' = T + (1 + s/1e6) * R * X
static Eigen::Vector3d apply_helmert(const Eigen::Vector3d& ecef, const SevenParameter& params)
{
    double scale = params.scale * 1e-6;
    Eigen::Matrix3d R = helmert_rotation(params);
    return (1.0 + scale) * R * ecef + Eigen::Vector3d(params.mx, params.my, params.mz);
}

// Apply inverse 7-parameter Helmert: X = R^T * (X' - T) / (1 + s)
static Eigen::Vector3d apply_helmert_inverse(const Eigen::Vector3d& ecef, const SevenParameter& params)
{
    double scale = params.scale * 1e-6;
    Eigen::Matrix3d R = helmert_rotation(params);
    return R.transpose() * (ecef - Eigen::Vector3d(params.mx, params.my, params.mz)) / (1.0 + scale);
}

Eigen::Vector3d GeoreferencingWith7Parameters::Transform(const Eigen::Vector3d& position)
{
    // Both the PROJ path and the manual-TM fallback assign these, but the
    // compiler cannot prove it across the nested branches - default-init.
    double lon_deg = 0.0, lat_deg = 0.0, h = 0.0;

    // Step 1: TM projected (easting, northing) → geographic (lon, lat)
    bool didProj = false;
    if (m_useProj && m_pj_transform) {
        PJ_COORD src_coord = proj_coord(position.x(), position.y(), position.z(), 0);
        // The Conversion from proj_crs_get_coordoperation maps geographic→projected
        // (PJ_FWD). PJ_INV reverses it: projected→geographic.
        proj_errno_reset(m_pj_transform);
        PJ_COORD dst_coord = proj_trans(m_pj_transform, PJ_INV, src_coord);
        if (!proj_errno(m_pj_transform)) {
            // CRS-aware pipeline returns degrees; raw Conversion returns radians.
            if (m_pj_returns_degrees) {
                lon_deg = dst_coord.lpz.lam;
                lat_deg = dst_coord.lpz.phi;
            } else {
                lon_deg = dst_coord.lpz.lam * 180.0 / M_PI;
                lat_deg = dst_coord.lpz.phi * 180.0 / M_PI;
            }
            h = dst_coord.lpz.z;
            didProj = true;
        } else {
            // PROJ transform failed — log once per instance then fall through to manual TM.
            if (!m_proj_warned) {
                m_proj_warned = true;
                int err = proj_context_errno(m_pj_context);
                std::cerr << "PROJ transform failed, falling back to manual TM"
                          << "\n  Error (" << err << "): "
                          << (err ? proj_errno_string(err) : "unknown")
                          << "\n  Input: E=" << position.x() << " N=" << position.y()
                          << "\n  Source CRS (first 200 chars): "
                          << m_srccrs.substr(0, 200) << std::endl;
            }
        }
    }
    if (!didProj) {
        double lon_rad, lat_rad;
        inverse_tmerc(position.x(), position.y(),
            m_tm_lon0, m_tm_lat0, m_tm_k0, m_tm_fe, m_tm_fn,
            m_tm_a, m_tm_rf, lon_rad, lat_rad);
        lon_deg = lon_rad * 180.0 / M_PI;
        lat_deg = lat_rad * 180.0 / M_PI;
        h = position.z();
    }

    // Step 2: Apply 7-parameter Helmert datum shift (if any parameter is non-zero)
    bool has_shift = (m_parameter.mx != 0.0 || m_parameter.my != 0.0 || m_parameter.mz != 0.0 ||
        m_parameter.rx != 0.0 || m_parameter.ry != 0.0 || m_parameter.rz != 0.0 ||
        m_parameter.scale != 0.0);

    if (has_shift) {
        double lon_rad = lon_deg * M_PI / 180.0;
        double lat_rad = lat_deg * M_PI / 180.0;

        // Geographic (source datum) → ECEF using source ellipsoid
        Eigen::Vector3d ecef = geo_to_ecef(lon_rad, lat_rad, h, m_tm_a, m_tm_rf);


        // Apply Helmert
        Eigen::Vector3d ecef_out = apply_helmert(ecef, m_parameter);


        // ECEF → Geographic (target datum, WGS84 for EPSG:4979)
        double out_lon_rad, out_lat_rad, out_h;
        ecef_to_geo(ecef_out, Geodetic::WGS84_SEMI_MAJOR_AXIS, Geodetic::WGS84_INV_FLATTENING, out_lon_rad, out_lat_rad, out_h);
        lon_deg = out_lon_rad * 180.0 / M_PI;
        lat_deg = out_lat_rad * 180.0 / M_PI;
        h = out_h;
    }

    return Eigen::Vector3d(lon_deg, lat_deg, h);
}

Eigen::Vector3d GeoreferencingWith7Parameters::InverseTransform(const Eigen::Vector3d& target_position)
{
    // target_position = (lon_deg, lat_deg, h) in target CRS (WGS84)
    double lon_rad = target_position.x() * M_PI / 180.0;
    double lat_rad = target_position.y() * M_PI / 180.0;
    double h = target_position.z();

    // Check if Helmert shift is active
    bool has_shift = (m_parameter.mx != 0.0 || m_parameter.my != 0.0 || m_parameter.mz != 0.0 ||
        m_parameter.rx != 0.0 || m_parameter.ry != 0.0 || m_parameter.rz != 0.0 ||
        m_parameter.scale != 0.0);

    Eigen::Vector3d source_ecef;

    if (has_shift) {
        // target geographic -> target ECEF.
        // Default target CRS is EPSG:4979 (WGS84). If a different target CRS
        // with non-WGS84 ellipsoid is used, this should query PROJ instead.
        Eigen::Vector3d target_ecef = geo_to_ecef(lon_rad, lat_rad, h, Geodetic::WGS84_SEMI_MAJOR_AXIS, Geodetic::WGS84_INV_FLATTENING);
        // target ECEF -> source ECEF (Helmert inverse)
        source_ecef = apply_helmert_inverse(target_ecef, m_parameter);
    }
    else {
        // No Helmert: target ECEF = source ECEF (same datum)
        source_ecef = geo_to_ecef(lon_rad, lat_rad, h, m_tm_a, m_tm_rf);
    }

    // source ECEF -> source CRS (projected) via PROJ pipeline
    if (!m_pj_SourceToECEF) {
        std::cerr << "InverseTransform: m_pj_SourceToECEF is null" << std::endl;
        return target_position;
    }
    PJ_COORD coord = proj_coord(source_ecef.x(), source_ecef.y(), source_ecef.z(), 0);
    PJ_COORD result = proj_trans(m_pj_SourceToECEF, PJ_INV, coord);
    return ConvertTool::CoordToEigen(result);
}

void GeoreferencingWith7Parameters::Solve()
{
    if (m_pj_transform) {
        proj_destroy(m_pj_transform);
        m_pj_transform = nullptr;
    }

    // Parse TM parameters from both WKT and PROJ string formats.
    // These are used as a manual fallback when PROJ cannot handle the CRS.
    if (is_wkt(m_srccrs)) {
        // Support both WKT1 (TitleCase) and WKT2 (lowercase) parameter names
        m_tm_lon0 = parse_wkt_param(m_srccrs, "Central_Meridian", 0.0) * M_PI / 180.0;
        if (m_tm_lon0 == 0.0)
            m_tm_lon0 = parse_wkt_param(m_srccrs, "central_meridian", 0.0) * M_PI / 180.0;
        m_tm_lat0 = parse_wkt_param(m_srccrs, "Latitude_Of_Origin", 0.0) * M_PI / 180.0;
        if (m_tm_lat0 == 0.0)
            m_tm_lat0 = parse_wkt_param(m_srccrs, "latitude_of_origin", 0.0) * M_PI / 180.0;
        m_tm_k0 = parse_wkt_param(m_srccrs, "Scale_Factor", 1.0);
        if (m_tm_k0 == 1.0)
            m_tm_k0 = parse_wkt_param(m_srccrs, "scale_factor", 1.0);
        m_tm_fe = parse_wkt_param(m_srccrs, "False_Easting", 0.0);
        if (m_tm_fe == 0.0)
            m_tm_fe = parse_wkt_param(m_srccrs, "false_easting", 0.0);
        m_tm_fn = parse_wkt_param(m_srccrs, "False_Northing", 0.0);
        if (m_tm_fn == 0.0)
            m_tm_fn = parse_wkt_param(m_srccrs, "false_northing", 0.0);
        // Parse SPHEROID["name",a,rf] from WKT (tolerate optional space before bracket)
        auto spos = m_srccrs.find("SPHEROID[");
        if (spos == std::string::npos)
            spos = m_srccrs.find("SPHEROID [");
        if (spos != std::string::npos) {
            auto c1 = m_srccrs.find(',', spos);
            if (c1 != std::string::npos) {
                auto c2 = m_srccrs.find(',', c1 + 1);
                if (c2 != std::string::npos) {
                    try { m_tm_a = std::stod(m_srccrs.substr(c1 + 1)); } catch(...) {}
                    try { m_tm_rf = std::stod(m_srccrs.substr(c2 + 1)); } catch(...) {}
                }
            }
        }
    } else {
        m_tm_lon0 = parse_proj_param(m_srccrs, "lon_0", 0.0) * M_PI / 180.0;
        m_tm_lat0 = parse_proj_param(m_srccrs, "lat_0", 0.0) * M_PI / 180.0;
        m_tm_k0   = parse_proj_param(m_srccrs, "k", 1.0);
        if (m_tm_k0 == 1.0)
            m_tm_k0 = parse_proj_param(m_srccrs, "k_0", 1.0);
        m_tm_a    = parse_proj_param(m_srccrs, "a", Geodetic::WGS84_SEMI_MAJOR_AXIS);
        m_tm_rf   = parse_proj_param(m_srccrs, "rf", Geodetic::WGS84_INV_FLATTENING);
        m_tm_fe   = parse_proj_param(m_srccrs, "x_0", 0.0);
        m_tm_fn   = parse_proj_param(m_srccrs, "y_0", 0.0);
    }

    // Build a transform for projected→geographic.
    //
    // WKT definitions create a ProjectedCRS; use proj_crs_get_coordoperation
    // to extract the Conversion which supports proj_trans(PJ_INV).
    //
    // PROJ strings without +type=crs create a bare Conversion that already
    // supports proj_trans(PJ_INV) directly.
    //
    // m_srccrs must contain the CRS definition string (WKT or PROJ string),
    // NOT a file path.

    PJ* src_crs = proj_create(m_pj_context, m_srccrs.c_str());
    if (src_crs) {
        // Only call proj_crs_get_coordoperation for actual CRS objects.
        // Bare Conversions (plain PROJ string without +type=crs) already
        // support proj_trans(PJ_INV) and return radians.
        PJ_TYPE type = proj_get_type(src_crs);
        bool isCRS = (type == PJ_TYPE_PROJECTED_CRS ||
                      type == PJ_TYPE_GEOGRAPHIC_CRS ||
                      type == PJ_TYPE_COMPOUND_CRS ||
                      type == PJ_TYPE_BOUND_CRS ||
                      type == PJ_TYPE_DERIVED_PROJECTED_CRS);
        if (isCRS) {
            m_pj_transform = proj_crs_get_coordoperation(m_pj_context, src_crs);
            if (m_pj_transform) {
                m_pj_returns_degrees = true;
                proj_destroy(src_crs);
            }
        }
        if (!m_pj_transform) {
            // Raw Conversion or failed CRS extraction: use directly, radians
            m_pj_transform = src_crs;
            m_pj_returns_degrees = false;
        }
    }
    if (!m_pj_transform) {
        int err = proj_context_errno(m_pj_context);
        std::cerr << "[7param] Failed to create coordinate operation: "
                  << (err ? proj_errno_string(err) : "unknown")
                  << "\n  Input (first 200 chars): "
                  << m_srccrs.substr(0, 200) << std::endl;
    }

    if (m_pj_transform) {
        m_useProj = true;
        return;
    }

    // Manual TM inverse projection fallback
    m_useProj = false;
}
