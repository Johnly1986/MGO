// Copyright Johnlyon
//
// Constants — unified shared constants for the entire MGO project.
//
// Replace hand-written magic numbers throughout the codebase with named
// constants defined in a single source of truth.  Includes geodetic
// ellipsoid parameters, EPSG CRS identifiers, projection defaults,
// tile/terrain constants, and epsilon thresholds.
//
// Usage: #include "Constants.h"  (header-only, no link dependency)
//

#pragma once

// =========================================================================
// 1. Geodetic ellipsoid parameters
// =========================================================================
// CGCS2000 ≈ WGS84 (identical semi-major axis, near-identical flattening)
namespace Geodetic {
    constexpr double WGS84_SEMI_MAJOR_AXIS = 6378137.0;
    constexpr double CGCS2000_INV_FLATTENING = 298.257222101;   // CGCS2000 / GRS80
    constexpr double WGS84_INV_FLATTENING = 298.257223563;      // WGS84

    // ---- Derived (computed from CGCS2000 by default) ----
    constexpr double CGCS2000_FLATTENING = 1.0 / CGCS2000_INV_FLATTENING;
    constexpr double CGCS2000_E2 = 2.0 * CGCS2000_FLATTENING - CGCS2000_FLATTENING * CGCS2000_FLATTENING;
} // namespace Geodetic

// =========================================================================
// 2. Angle conversion
// =========================================================================
// GeodeticMath.h already defines DEG2RAD = 0.017453292519943295;
// these are aliases for clarity in non-GeodeticMath contexts.
namespace Angle {
    constexpr double DEG_TO_RAD = 0.017453292519943295;
    constexpr double RAD_TO_DEG = 57.295779513082322865;
    constexpr double PI = 3.14159265358979323846;
    constexpr double PI_HALF = PI / 2.0;
    constexpr double TWO_PI = PI * 2.0;
    constexpr double ARCSEC_TO_RAD = 4.848136811095359935899141e-6;
} // namespace Angle

// =========================================================================
// 3. EPSG / CRS identifier strings
// =========================================================================
// Use these instead of writing "EPSG:xxxx" as string literals.
namespace CRS {
    constexpr const char* WGS84_GEOGRAPHIC_2D = "EPSG:4326";
    constexpr const char* WGS84_GEOGRAPHIC_3D = "EPSG:4979";
    constexpr const char* WGS84_ECEF = "EPSG:4978";
    constexpr const char* CGCS2000_GK_ZONE39 = "EPSG:4547";
} // namespace CRS

// =========================================================================
// 4. PROJ / projection default parameters
// =========================================================================
namespace ProjectionDefaults {
    constexpr double FALSE_EASTING = 500000.0;
    constexpr double FALSE_NORTHING = 0.0;
    constexpr double SCALE_FACTOR = 1.0;
    constexpr double LATITUDE_OF_ORIGIN = 0.0;
} // namespace ProjectionDefaults

// =========================================================================
// 5. Earth surface / meters-per-degree
// =========================================================================
namespace Earth {
    constexpr double METERS_PER_DEGREE_AT_EQUATOR = 111320.0;
    constexpr double SEMI_MAJOR_AXIS = Geodetic::WGS84_SEMI_MAJOR_AXIS;  // alias
} // namespace Earth

// =========================================================================
// 6. Tile / terrain constants
// =========================================================================
namespace TileConstants {
    constexpr float MAX_QUANTIZED_UV = 32767.0f;
    constexpr double MAX_QUANTIZED_UV_D = 32767.0;
    constexpr double DEFAULT_MIN_BLOCK_DISTANCE = 100.0;
    constexpr int MIN_CONTENT_FOR_EXTERNAL_TILESET = 6;
    constexpr int MIN_VERTICES_FOR_TILE_CONTENT = 50;
    constexpr int MAX_TERRAIN_LOD = 17;
    // Fractional scale factor - must be double; as int it would truncate
    // to 0 and collapse every geometricError to the floors (1.0 / 100.0).
    constexpr double GEOMETRIC_ERROR_COEFFICIENT = 0.25;
} // namespace TileConstants

// =========================================================================
// 7. General epsilon / tolerance values
// =========================================================================
namespace Epsilon {
    constexpr double DOUBLE_LOOSE = 1e-6;
    constexpr double DOUBLE_CONVERGENCE = 1e-10;
    constexpr double DOUBLE_ITERATION = 1e-12;
    constexpr float  FLOAT_DEGENERATE = 1e-10f;
    constexpr double BBOX_SENTINEL_MAX = 1e100;
    constexpr double BBOX_SENTINEL_LARGE = 1e10;
    constexpr double BBOX_SENTINEL_GEO = 1e9;
} // namespace Epsilon

// =========================================================================
// 8. Coordinate system axes
// =========================================================================
namespace Axis {
    constexpr double MAX_LONGITUDE = 180.0;
    constexpr double MAX_LATITUDE = 90.0;
    constexpr double MIN_LONGITUDE = -180.0;
    constexpr double MIN_LATITUDE = -90.0;
} // namespace Axis
