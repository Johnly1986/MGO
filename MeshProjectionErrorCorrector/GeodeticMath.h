// Copyright Johnlyon
//
// GeodeticMath — shared geodetic constants and helper functions
//
// Pure math, no external dependencies (no PROJ, no Assimp).
// Matrix helpers use Eigen for the 3x3/4x4 rotation construction.
// Used by both CProjectionEngine (3D Tiles pipeline) and
// GeoreferencingWithMultiPosition (control-point georeferencing).
//

#pragma once

#include <cmath>
#include <cstring>
#include <Eigen/Dense>

namespace GeodeticMath
{
    // ===========================================================================
    // Ellipsoid constants (CGCS2000 ≈ WGS84)
    // ===========================================================================
    constexpr double A  = 6378137.0;                // semi-major axis (m)
    constexpr double FI = 298.257222101;             // inverse flattening
    constexpr double F  = 1.0 / FI;                  // flattening
    constexpr double E2 = 2.0 * F - F * F;           // first eccentricity squared
    constexpr double EP2 = E2 / (1.0 - E2);          // second eccentricity squared
    constexpr double DEG2RAD = 0.017453292519943295;  // π/180

    // ===========================================================================
    // Meridian arc length from equator to latitude phi (radians)
    // ===========================================================================
    inline double MeridianArcLength(double phi, double a = A, double e2 = E2)
    {
        double e4 = e2 * e2, e6 = e4 * e2;
        double m0 = a * (1.0 - e2 / 4.0 - 3.0 * e4 / 64.0 - 5.0 * e6 / 256.0);
        double m2 = a * (-3.0 * e2 / 8.0 - 3.0 * e4 / 32.0 - 45.0 * e6 / 1024.0);
        double m4 = a * (15.0 * e4 / 256.0 + 45.0 * e6 / 1024.0);
        double m6 = a * (-35.0 * e6 / 3072.0);
        return m0 * phi + m2 * std::sin(2.0 * phi) + m4 * std::sin(4.0 * phi) + m6 * std::sin(6.0 * phi);
    }

    // ===========================================================================
    // Foot-point latitude from meridian arc distance (Newton iteration)
    // ===========================================================================
    inline double FootPointLatitude(double M, double a = A, double e2 = E2)
    {
        double e4 = e2 * e2, e6 = e4 * e2;
        double m0 = a * (1.0 - e2 / 4.0 - 3.0 * e4 / 64.0 - 5.0 * e6 / 256.0);
        double phi = M / m0;
        for (int iter = 0; iter < 6; ++iter)
        {
            double arc = MeridianArcLength(phi, a, e2);
            double dphi = (M - arc);
            double sinphi = std::sin(phi);
            double Mprime = a * (1.0 - e2) / std::pow(1.0 - e2 * sinphi * sinphi, 1.5);
            phi += dphi / Mprime;
            if (std::abs(dphi) < 1e-12) break;
        }
        return phi;
    }

    // ===========================================================================
    // Gauss-Kruger inverse: (Easting, Northing) → (lat, lon) in radians
    // Ref: Snyder, "Map Projections — A Working Manual" (1987), pp. 63-65
    // ===========================================================================
    inline bool GKInverse(double E, double N, double lambda0_deg, double k0,
                          double falseE, double falseN,
                          double& lat, double& lon,
                          double a = A, double e2 = E2)
    {
        double ep2 = e2 / (1.0 - e2);
        double lambda0 = lambda0_deg * DEG2RAD;

        double x = E - falseE;
        double y = N - falseN;
        double M = y / k0;

        double phi_f = FootPointLatitude(M, a, e2);

        double sin_f  = std::sin(phi_f);
        double cos_f  = std::cos(phi_f);
        double tan_f  = sin_f / cos_f;
        double t_f    = tan_f;
        double t_f2   = t_f * t_f;
        double t_f4   = t_f2 * t_f2;
        double t_f6   = t_f4 * t_f2;
        double eta_f2 = ep2 * cos_f * cos_f;
        double eta_f4 = eta_f2 * eta_f2;

        double nu_f  = a / std::sqrt(1.0 - e2 * sin_f * sin_f);
        double rho_f = a * (1.0 - e2) / std::pow(1.0 - e2 * sin_f * sin_f, 1.5);

        // Lat — Taylor expansion to 8th order
        double D = x / (k0 * nu_f);
        double D2 = D * D, D3 = D2 * D, D4 = D3 * D, D5 = D4 * D, D6 = D5 * D;
        double D7 = D6 * D, D8 = D7 * D;

        double phi = phi_f;
        phi -= (t_f / (2.0 * rho_f)) * (x * D / k0);
        double term2 = (5.0 + 3.0 * t_f2 + eta_f2 - 4.0 * eta_f4 - 9.0 * eta_f2 * t_f2);
        phi += (t_f / (24.0 * rho_f)) * (x * D3 / k0) * term2;
        double term3 = (61.0 + 90.0 * t_f2 + 45.0 * t_f4 + 46.0 * eta_f2
                     - 252.0 * eta_f2 * t_f2 - 3.0 * eta_f4 + 100.0 * eta_f4 * t_f2
                     - 66.0 * eta_f2 * t_f4 - 90.0 * eta_f4 * t_f4);
        phi -= (t_f / (720.0 * rho_f)) * (x * D5 / k0) * term3;
        double term4 = (1385.0 + 3633.0 * t_f2 + 4095.0 * t_f4 + 1575.0 * t_f6);
        phi += (t_f / (40320.0 * rho_f)) * (x * D7 / k0) * term4;

        // Lon — Taylor expansion to 8th order
        double lambda = lambda0;
        lambda += D / cos_f;
        double lon2 = D3 * (1.0 + 2.0 * t_f2 + eta_f2);
        lambda -= lon2 / (6.0 * cos_f);
        double lon3 = D5 * (5.0 + 28.0 * t_f2 + 24.0 * t_f4 + 6.0 * eta_f2 + 8.0 * eta_f2 * t_f2);
        lambda += lon3 / (120.0 * cos_f);
        double lon4 = D7 * (61.0 + 662.0 * t_f2 + 1320.0 * t_f4 + 720.0 * t_f6);
        lambda -= lon4 / (5040.0 * cos_f);

        lat = phi;
        lon = lambda;
        return true;
    }

    // ===========================================================================
    // Transverse Mercator forward: (lat, lon in radians) → (Easting, Northing)
    // Ref: Snyder, "Map Projections — A Working Manual" (1987), pp. 61-63
    // ===========================================================================
    inline void TMForward(double lat, double lon, double lambda0_deg, double k0,
                          double falseE, double falseN, double lat0_deg,
                          double& E, double& N,
                          double a = A, double e2 = E2)
    {
        double ep2 = e2 / (1.0 - e2);
        double lambda0 = lambda0_deg * DEG2RAD;
        double phi0 = lat0_deg * DEG2RAD;

        double sinPhi = std::sin(lat);
        double cosPhi = std::cos(lat);
        double tanPhi = sinPhi / cosPhi;
        double T = tanPhi * tanPhi;
        double C = ep2 * cosPhi * cosPhi;
        double A_val = (lon - lambda0) * cosPhi;

        double nu = a / std::sqrt(1.0 - e2 * sinPhi * sinPhi);
        double M = MeridianArcLength(lat, a, e2);
        double M0 = MeridianArcLength(phi0, a, e2);

        double A2 = A_val * A_val;
        double A3 = A2 * A_val;
        double A4 = A3 * A_val;
        double A5 = A4 * A_val;
        double A6 = A5 * A_val;

        // Easting
        double x = k0 * nu * (A_val + (1.0 - T + C) * A3 / 6.0 +
                   (5.0 - 18.0 * T + T * T + 72.0 * C - 58.0 * ep2) * A5 / 120.0);

        // Northing
        double y = k0 * (M - M0 + nu * tanPhi *
                   (A2 / 2.0 + (5.0 - T + 9.0 * C + 4.0 * C * C) * A4 / 24.0 +
                    (61.0 - 58.0 * T + T * T + 600.0 * C - 330.0 * ep2) * A6 / 720.0));

        E = falseE + x;
        N = falseN + y;
    }

    // ===========================================================================
    // Geographic (lat, lon in radians, h in meters) → ECEF (X, Y, Z)
    // ===========================================================================
    inline void GeographicToECEF(double lat, double lon, double h,
                                 double& X, double& Y, double& Z,
                                 double a = A, double e2 = E2)
    {
        double sinLat = std::sin(lat);
        double cosLat = std::cos(lat);
        double N = a / std::sqrt(1.0 - e2 * sinLat * sinLat);
        X = (N + h) * cosLat * std::cos(lon);
        Y = (N + h) * cosLat * std::sin(lon);
        Z = (N * (1.0 - e2) + h) * sinLat;
    }

    // ===========================================================================
    // ECEF → Geographic (lon, lat in radians, h in meters)
    // Bowring's iterative method
    // ===========================================================================
    inline void ECEFToGeographic(double X, double Y, double Z,
                                 double& lat, double& lon, double& h,
                                 double a = A, double e2 = E2)
    {
        lon = std::atan2(Y, X);
        double p = std::sqrt(X * X + Y * Y);
        lat = std::atan2(Z, p * (1.0 - e2));
        for (int i = 0; i < 10; ++i)
        {
            double sl = std::sin(lat);
            double N = a / std::sqrt(1.0 - e2 * sl * sl);
            double new_lat = std::atan2(Z + e2 * N * sl, p);
            if (std::abs(new_lat - lat) < 1e-12) { lat = new_lat; break; }
            lat = new_lat;
        }
        double sl = std::sin(lat);
        double N = a / std::sqrt(1.0 - e2 * sl * sl);
        h = p / std::cos(lat) - N;
    }

    // ===========================================================================
    // ENU→ECEF rotation matrix at given geographic point.
    // R(i,j) = i-th ECEF component of the j-th ENU basis (E, N, U).
    // Matches the previous row-major R[9] layout R[i*3+j] element-for-element.
    // ===========================================================================
    inline Eigen::Matrix3d ENUToECEFRotation(double lat, double lon)
    {
        double sinLat = std::sin(lat);
        double cosLat = std::cos(lat);
        double sinLon = std::sin(lon);
        double cosLon = std::cos(lon);

        Eigen::Matrix3d R;
        // Column 0: East  = [-sinLon,  cosLon, 0]
        R(0, 0) = -sinLon;  R(1, 0) = cosLon;   R(2, 0) = 0.0;
        // Column 1: North = [-sinLat*cosLon, -sinLat*sinLon, cosLat]
        R(0, 1) = -sinLat * cosLon;  R(1, 1) = -sinLat * sinLon;  R(2, 1) = cosLat;
        // Column 2: Up    = [cosLat*cosLon,  cosLat*sinLon,  sinLat]
        R(0, 2) =  cosLat * cosLon;  R(1, 2) =  cosLat * sinLon;  R(2, 2) = sinLat;
        return R;
    }

    // ===========================================================================
    // Build ENU→ECEF 4×4 root transform (column-major, matches 3D Tiles spec).
    // Top-left 3×3 = R element-wise; column 3 = ECEF translation.
    // The returned .data() order is identical to the previous out[16] layout.
    // ===========================================================================
    inline Eigen::Matrix4d BuildRootTransform(const Eigen::Matrix3d& R_enu,
                                              const Eigen::Vector3d& translation)
    {
        Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
        out.topLeftCorner<3, 3>() = R_enu;
        out.block<3, 1>(0, 3) = translation;
        return out;
    }

    // ===========================================================================
    // Transverse Mercator inverse (Krüger series, 6th order)
    // Used by GeoreferencingWithMultiPosition for ECEF_Affine mode
    // ===========================================================================
    struct TMParams {
        double lon0_deg{100.35};
        double lat0_deg{0.0};
        double k0{1.0};
        double fe{500000.0};
        double fn{0.0};
        double a{A};
        double rf{FI};
    };

    inline void InverseTM(double easting, double northing, const TMParams& tm,
                          double& lon_rad, double& lat_rad)
    {
        double lon0_rad = tm.lon0_deg * DEG2RAD;
        double f = 1.0 / tm.rf;
        double e2 = 2.0 * f - f * f;
        double e4 = e2 * e2, e6 = e4 * e2, e8 = e6 * e2;

        double Ndist = northing - tm.fn;
        double a0 = tm.a * (1.0 - e2 / 4.0 - 3.0 * e4 / 64.0 - 5.0 * e6 / 256.0 - 175.0 * e8 / 16384.0);

        double latf = Ndist / a0;
        for (int i = 0; i < 20; ++i) {
            double sin2f = std::sin(2.0 * latf);
            double sin4f = std::sin(4.0 * latf);
            double sin6f = std::sin(6.0 * latf);
            double sin8f = std::sin(8.0 * latf);

            double M = tm.a * ((1.0 - e2 / 4.0 - 3.0 * e4 / 64.0 - 5.0 * e6 / 256.0 - 175.0 * e8 / 16384.0) * latf
                - (3.0 * e2 / 8.0 + 3.0 * e4 / 32.0 + 45.0 * e6 / 1024.0 + 105.0 * e8 / 4096.0) * sin2f
                + (15.0 * e4 / 256.0 + 45.0 * e6 / 1024.0 + 525.0 * e8 / 16384.0) * sin4f
                - (35.0 * e6 / 3072.0 + 175.0 * e8 / 12288.0) * sin6f
                + (315.0 * e8 / 131072.0) * sin8f);

            latf = latf + (Ndist - M) / a0;
            if (std::abs(Ndist - M) < 1e-10) break;
        }

        double sf = std::sin(latf), cf = std::cos(latf);
        double tf = sf / cf;
        double ef2 = e2 / (1.0 - e2) * cf * cf;
        double vf = tm.a / std::sqrt(1.0 - e2 * sf * sf);
        double rhof = vf / (1.0 + ef2);

        double x = easting - tm.fe;
        double eta = x / (tm.k0 * vf);
        double eta3 = eta * eta * eta;
        double eta5 = eta3 * eta * eta;

        lat_rad = latf
            - tf * x * x / (2.0 * tm.k0 * tm.k0 * rhof * vf)
            + tf * std::pow(x, 4) / (24.0 * std::pow(tm.k0, 4) * rhof * std::pow(vf, 3))
              * (5.0 + 3.0 * tf * tf + ef2 - 9.0 * ef2 * tf * tf - 4.0 * ef2 * ef2)
            - tf * std::pow(x, 6) / (720.0 * std::pow(tm.k0, 6) * rhof * std::pow(vf, 5))
              * (61.0 + 90.0 * tf * tf + 45.0 * tf * tf * tf * tf + 46.0 * ef2
                 + 252.0 * ef2 * tf * tf + 90.0 * ef2 * ef2 + 45.0 * ef2 * ef2 * tf * tf);

        lon_rad = lon0_rad
            + eta / cf
            - eta3 / (6.0 * cf) * (1.0 + 2.0 * tf * tf + ef2)
            + eta5 / (120.0 * cf) * (5.0 + 28.0 * tf * tf + 24.0 * tf * tf * tf * tf
                                     + 6.0 * ef2 + 8.0 * ef2 * tf * tf);
    }

    // ===========================================================================
    // Geographic → ECEF with explicit ellipsoid parameters
    // (for GeoreferencingWithMultiPosition which uses custom a/rf)
    // ===========================================================================
    inline void GeoToECEF(double lon_rad, double lat_rad, double h,
                          double& X, double& Y, double& Z,
                          double a, double rf)
    {
        double f = 1.0 / rf;
        double e2 = 2.0 * f - f * f;
        double sinlat = std::sin(lat_rad), coslat = std::cos(lat_rad);
        double N = a / std::sqrt(1.0 - e2 * sinlat * sinlat);
        X = (N + h) * coslat * std::cos(lon_rad);
        Y = (N + h) * coslat * std::sin(lon_rad);
        Z = (N * (1.0 - e2) + h) * sinlat;
    }

    // ===========================================================================
    // ECEF → Geographic with explicit ellipsoid parameters
    // ===========================================================================
    inline void ECEFToGeo(double X, double Y, double Z,
                          double& lon_rad, double& lat_rad, double& h,
                          double a, double rf)
    {
        double f = 1.0 / rf;
        double e2 = 2.0 * f - f * f;
        lon_rad = std::atan2(Y, X);
        double p = std::sqrt(X * X + Y * Y);
        double lat = std::atan2(Z, p * (1.0 - e2));
        for (int i = 0; i < 10; ++i)
        {
            double sl = std::sin(lat);
            double N = a / std::sqrt(1.0 - e2 * sl * sl);
            double new_lat = std::atan2(Z + e2 * N * sl, p);
            if (std::abs(new_lat - lat) < 1e-12) { lat = new_lat; break; }
            lat = new_lat;
        }
        lat_rad = lat;
        double sl = std::sin(lat);
        double N = a / std::sqrt(1.0 - e2 * sl * sl);
        h = p / std::cos(lat) - N;
    }

} // namespace GeodeticMath
