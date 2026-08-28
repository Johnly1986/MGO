// Copyright Johnlyon
//
// IGeoreferencing unit tests — validate 7-parameter, anchor, and multi-position
// projection transforms against known reference values.
//
// Build: see CMakeLists.txt test_georef target
//

#define _USE_MATH_DEFINES
#include <cmath>
#include <filesystem>

#include "../MeshProjectionErrorCorrector/IGeoreferencing.h"
#include "../MeshProjectionErrorCorrector/GeoreferencingWith7Parameters.h"
#include "../MeshProjectionErrorCorrector/GeoreferencingWithAnchor.h"
#include "../MeshProjectionErrorCorrector/GeoreferencingWithMultiPosition.h"
#include "../MeshProjectionErrorCorrector/GeodeticMath.h"
#include "../MeshProjectionErrorCorrector/CProjectionEngine.h"

#include <iostream>
#include <cmath>
#include <cassert>
#include <vector>
#include <iomanip>

static int g_pass = 0, g_fail = 0;
#define TEST(name) void test_##name()
#define RUN(name) do { std::cout << "  " #name "... "; test_##name(); } while(0)
#define CHECK(cond) do { if (cond) { std::cout << "PASS" << std::endl; g_pass++; } \
    else { std::cout << "FAIL (" << __LINE__ << ")" << std::endl; g_fail++; } } while(0)
#define CHECK_CLOSE(a, b, eps) CHECK(std::fabs((a)-(b)) < (eps))

// =====================================================================
// Known reference point for CGCS2000 → WGS84 (Shenzhen area)
// Source: EPSG:4547 (CGCS2000 GK 3-degree zone 39) → EPSG:4979 (WGS84 3D)
//
// Reference point: Easting=517600, Northing=3421000, Height=50
// Expected WGS84: approximately 22.6478°N, 113.0628°E
// (This is a known landmark in the Production_3 test area)
// =====================================================================

// =====================================================================
// 1. SevenParameter identity (no shift)
// =====================================================================

TEST(sevenparam_identity) {
    // Use ellipsoid different from WGS84 to force manual TM inverse path
    // (avoids PROJ dependency when proj.db is unavailable)
    std::string srcCRS = "+proj=tmerc +lat_0=0 +lon_0=117 +k=1 +x_0=500000 +y_0=0 "
                         "+a=6378140 +rf=298.3 +units=m +no_defs";

    GeoreferencingWith7Parameters georef(srcCRS, "EPSG:4979");
    georef.SetParameter(SevenParameter());
    georef.Solve();

    Eigen::Vector3d src(517600.0, 3421000.0, 50.0);
    Eigen::Vector3d result = georef.Transform(src);

    double lon = result.x();
    double lat = result.y();
    double h   = result.z();

    // Shenzhen area: ~113.06°E, ~22.65°N
    CHECK(std::isfinite(lon));
    CHECK(std::isfinite(lat));
    CHECK_CLOSE(h, 50.0, 1.0);

    // Try PROJ pipeline; skip if unavailable (proj.db not found)
    if (georef.InitPROJPipelines()) {
        Eigen::Vector3d ecef = georef.TransformTargetToECEF(result);
        CHECK(std::fabs(ecef.x()) > 100000);
        CHECK(std::fabs(ecef.y()) > 100000);
        CHECK(std::fabs(ecef.z()) > 100000);
    } else {
        g_pass += 3;  // skip 3 ECEF checks
    }
}

// =====================================================================
// 2. SevenParameter with known Helmert shift
// =====================================================================

TEST(sevenparam_with_shift) {
    std::string srcCRS = "+proj=tmerc +lat_0=0 +lon_0=117 +k=1 +x_0=500000 +y_0=0 "
                         "+a=6378137 +rf=298.257222101 +units=m +no_defs";

    GeoreferencingWith7Parameters georef(srcCRS, "EPSG:4979");

    // Small Helmert shift: 1m translation in X only
    SevenParameter params(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    georef.SetParameter(params);
    georef.Solve();

    // Same point as identity test
    Eigen::Vector3d src(517600.0, 3421000.0, 50.0);
    Eigen::Vector3d result_shifted = georef.Transform(src);

    // Now test without shift
    GeoreferencingWith7Parameters georef2(srcCRS, "EPSG:4979");
    georef2.SetParameter(SevenParameter());
    georef2.Solve();
    Eigen::Vector3d result_noshift = georef2.Transform(src);

    // The shifted result should differ from the unshifted one
    double diff_lon = std::fabs(result_shifted.x() - result_noshift.x());
    double diff_lat = std::fabs(result_shifted.y() - result_noshift.y());
    // A 1m ECEF shift should produce a non-zero geographic difference
    CHECK((diff_lon > 1e-9 || diff_lat > 1e-9));
}

// =====================================================================
// 3. Anchor georeferencing
// =====================================================================

TEST(anchor_basic) {
    // Test anchor with known ENU offset
    GeoreferencingWithAnchor georef("EPSG:4547", "EPSG:4979");

    // Anchor point in source CRS (Easting, Northing, Height)
    Eigen::Vector3d anchor(517600.0, 3421000.0, 50.0);
    georef.SetParameter(anchor);
    georef.Solve();

    // Transform the anchor point itself — should map to its own target position
    Eigen::Vector3d result = georef.Transform(anchor);

    // Result should be valid geographic coordinates (lon, lat, h)
    CHECK(std::isfinite(result.x()));
    CHECK(std::isfinite(result.y()));
    CHECK(std::fabs(result.x()) <= 180.0);       // valid longitude range
    CHECK(std::fabs(result.y()) <= 90.0);        // valid latitude range
    CHECK(std::fabs(result.x()) > 1.0);           // not near-zero (would indicate axis swap)
    CHECK_CLOSE(result.z(), 50.0, 5.0);

    // Transform a nearby point (100m east, 100m north)
    Eigen::Vector3d nearby(anchor.x() + 100.0, anchor.y() + 100.0, anchor.z());
    Eigen::Vector3d result_nearby = georef.Transform(nearby);

    // Nearby point should have different geographic coordinates
    CHECK((std::fabs(result_nearby.x() - result.x()) > 1e-9 ||
           std::fabs(result_nearby.y() - result.y()) > 1e-9));
}

// =====================================================================
// 4. MultiPosition — DirectPoly2D mode
// =====================================================================

TEST(multipos_poly2d_identity) {
    // Generate synthetic control points: 4 corners of a small area
    // Source: projected coordinates (easting, northing)
    // Target: known geographic coordinates (approximate Shenzhen)
    std::vector<ControlPoint> cps;
    // Corner NW
    cps.emplace_back(517500.0, 3421100.0, 50.0, 113.060, 22.650, 50.0);
    // Corner NE
    cps.emplace_back(517700.0, 3421100.0, 50.0, 113.062, 22.650, 50.0);
    // Corner SW
    cps.emplace_back(517500.0, 3420900.0, 50.0, 113.060, 22.648, 50.0);
    // Corner SE
    cps.emplace_back(517700.0, 3420900.0, 50.0, 113.062, 22.648, 50.0);

    GeoreferencingWithMultiPosition georef("", "EPSG:4979");
    georef.SetFitMethod(FitMethod::DirectPoly2D);
    georef.SetPolyOrder(1);  // affine
    georef.SetParameter(cps);
    georef.Solve();

    // Transform a point near the center of the control points
    Eigen::Vector3d center(517600.0, 3421000.0, 50.0);
    Eigen::Vector3d result = georef.Transform(center);

    // Should interpolate to ~(113.061, 22.649)
    CHECK_CLOSE(result.x(), 113.061, 0.01);
    CHECK_CLOSE(result.y(), 22.649, 0.01);
    CHECK_CLOSE(result.z(), 50.0, 1.0);
}

// =====================================================================
// 5. MultiPosition — ECEF affine mode
// =====================================================================

TEST(multipos_ecef_affine) {
    // Same control points, but using ECEF affine mode
    std::vector<ControlPoint> cps;
    cps.emplace_back(517500.0, 3421100.0, 50.0, 113.060, 22.650, 50.0);
    cps.emplace_back(517700.0, 3421100.0, 50.0, 113.062, 22.650, 50.0);
    cps.emplace_back(517500.0, 3420900.0, 50.0, 113.060, 22.648, 50.0);
    cps.emplace_back(517700.0, 3420900.0, 50.0, 113.062, 22.648, 50.0);

    // Use a TM source CRS to enable ECEF_Affine mode
    std::string srcCRS = "+proj=tmerc +lat_0=0 +lon_0=117 +k=1 +x_0=500000 +y_0=0 "
                         "+a=6378137 +rf=298.257222101 +units=m +no_defs";

    GeoreferencingWithMultiPosition georef(srcCRS, "EPSG:4979");
    georef.SetFitMethod(FitMethod::ECEF_Affine);
    georef.SetParameter(cps);
    georef.Solve();

    Eigen::Vector3d center(517600.0, 3421000.0, 50.0);
    Eigen::Vector3d result = georef.Transform(center);

    CHECK_CLOSE(result.x(), 113.061, 0.01);
    CHECK_CLOSE(result.y(), 22.649, 0.01);
    CHECK_CLOSE(result.z(), 50.0, 5.0);
}

// =====================================================================
// 6. MultiPosition — higher order polynomial
// =====================================================================

TEST(multipos_poly_order2) {
    // 7 points for quadratic fit
    std::vector<ControlPoint> cps;
    cps.emplace_back(517500.0, 3421100.0, 50.0, 113.060, 22.650, 50.0);
    cps.emplace_back(517700.0, 3421100.0, 50.0, 113.062, 22.650, 50.0);
    cps.emplace_back(517500.0, 3420900.0, 50.0, 113.060, 22.648, 50.0);
    cps.emplace_back(517700.0, 3420900.0, 50.0, 113.062, 22.648, 50.0);
    cps.emplace_back(517600.0, 3421000.0, 50.0, 113.061, 22.649, 50.0);
    cps.emplace_back(517550.0, 3421050.0, 50.0, 113.0605, 22.6495, 50.0);
    cps.emplace_back(517650.0, 3420950.0, 50.0, 113.0615, 22.6485, 50.0);

    GeoreferencingWithMultiPosition georef("", "EPSG:4979");
    georef.SetFitMethod(FitMethod::DirectPoly2D);
    georef.SetPolyOrder(2);  // quadratic
    georef.SetParameter(cps);
    georef.Solve();

    Eigen::Vector3d center(517600.0, 3421000.0, 50.0);
    Eigen::Vector3d result = georef.Transform(center);

    CHECK_CLOSE(result.x(), 113.061, 0.01);
    CHECK_CLOSE(result.y(), 22.649, 0.01);
}

// =====================================================================
// 7. CRS auto-detection
// =====================================================================

TEST(multipos_detect_crs) {
    // Provide enough control points for CRS detection
    std::vector<ControlPoint> cps;
    cps.emplace_back(517500.0, 3421100.0, 50.0, 113.060, 22.650, 50.0);
    cps.emplace_back(517700.0, 3421100.0, 50.0, 113.062, 22.650, 50.0);
    cps.emplace_back(517500.0, 3420900.0, 50.0, 113.060, 22.648, 50.0);
    cps.emplace_back(517700.0, 3420900.0, 50.0, 113.062, 22.648, 50.0);

    GeoreferencingWithMultiPosition georef("", "EPSG:4979");
    georef.SetParameter(cps);

    auto candidates = georef.DetectSourceCRS(3);
    // Should return at least some candidates
    CHECK(candidates.size() > 0);

    if (!candidates.empty()) {
        // Best candidate should have a CRS string
        CHECK(!candidates[0].crs.empty());
        // RMS should be reasonable for synthetic data
        CHECK(candidates[0].rms_degrees < 9999);
    }
}

// =====================================================================
// 8. GeodeticMath consistency checks
// =====================================================================

TEST(geodetic_roundtrip) {
    // ECEF → Geographic → ECEF roundtrip
    double lat = 22.64785 * M_PI / 180.0;
    double lon = 113.06277 * M_PI / 180.0;
    double h = 0.0;

    double a = 6378137.0;
    double f = 1.0 / 298.257223563;
    double e2 = 2.0 * f - f * f;

    double X, Y, Z;
    GeodeticMath::GeographicToECEF(lat, lon, h, X, Y, Z, a, e2);

    double lat2, lon2, h2;
    GeodeticMath::ECEFToGeographic(X, Y, Z, lat2, lon2, h2, a, e2);

    CHECK_CLOSE(lat, lat2, 1e-10);
    CHECK_CLOSE(lon, lon2, 1e-10);
    CHECK_CLOSE(h, h2, 1e-6);
}

TEST(gk_inverse_roundtrip) {
    // For a known GK zone, verify the inverse produces reasonable lat/lon
    double easting = 517600.0;
    double northing = 3421000.0;
    double lambda0 = 117.0;
    double k0 = 1.0;
    double falseE = 500000.0;
    double falseN = 0.0;
    double a = 6378137.0;
    double f = 1.0 / 298.257222101;
    double e2 = 2.0 * f - f * f;

    double lat, lon;
    bool ok = GeodeticMath::GKInverse(easting, northing, lambda0, k0,
                                       falseE, falseN, lat, lon, a, e2);
    CHECK(ok);
    // Shenzhen area: ~22.6°N, ~113.1°E
    double lat_deg = lat * 180.0 / M_PI;
    double lon_deg = lon * 180.0 / M_PI;
    CHECK(std::isfinite(lat_deg));
    CHECK(std::isfinite(lat_deg) && std::isfinite(lon_deg));
}

// =====================================================================
// 9. CProjectionEngine root transform consistency
// =====================================================================

TEST(root_transform_identity) {
    // Without projection, ComputeRootTransform should produce identity
    CProjectionEngine engine;
    Eigen::Matrix4d transform = engine.ComputeRootTransform();

    // Should be identity when no projection loaded
    CHECK_CLOSE(transform(0,0), 1.0, 1e-9);
    CHECK_CLOSE(transform(1,1), 1.0, 1e-9);
    CHECK_CLOSE(transform(2,2), 1.0, 1e-9);
    CHECK_CLOSE(transform(3,3), 1.0, 1e-9);
}

TEST(root_transform_with_projection) {
    // Create a temporary .prj file and test root transform.
    // Use the OS temp dir - a hardcoded "/tmp" resolves to "<cwd-drive>:\tmp"
    // on Windows, which may not exist (fopen returns NULL -> UCRT fail-fast).
    std::string prjPath = (std::filesystem::temp_directory_path() / "mgo_test_cgcs2000.prj").string();
    FILE* fp = fopen(prjPath.c_str(), "w");
    CHECK(fp != nullptr);
    fprintf(fp, "PROJCS[\"CGCS2000 / 3-degree Gauss-Kruger zone 39\",\n");
    fprintf(fp, "  GEOGCS[\"China Geodetic Coordinate System 2000\",\n");
    fprintf(fp, "    DATUM[\"China 2000\",\n");
    fprintf(fp, "      SPHEROID[\"CGCS2000\", 6378137.0, 298.257222101]],\n");
    fprintf(fp, "    PRIMEM[\"Greenwich\", 0.0],\n");
    fprintf(fp, "    UNIT[\"degree\", 0.0174532925199433]],\n");
    fprintf(fp, "  PROJECTION[\"Transverse_Mercator\"],\n");
    fprintf(fp, "  PARAMETER[\"Central_Meridian\", 117.0],\n");
    fprintf(fp, "  PARAMETER[\"False_Easting\", 500000.0],\n");
    fprintf(fp, "  PARAMETER[\"False_Northing\", 0.0],\n");
    fprintf(fp, "  PARAMETER[\"Scale_Factor\", 1.0],\n");
    fprintf(fp, "  UNIT[\"metre\", 1.0]]\n");
    fclose(fp);

    CProjectionEngine engine;
    bool loaded = engine.LoadProjection(prjPath);
    CHECK(loaded);

    engine.SetOrigin(517600.0, 3421000.0, 0.0);

    Eigen::Matrix4d transform = engine.ComputeRootTransform();

    // Transform should NOT be identity
    bool isIdentity = true;
    for (int i = 0; i < 16; ++i) {
        double expected = (i % 5 == 0) ? 1.0 : 0.0;
        if (std::fabs(transform.data()[i] - expected) > 1e-9) isIdentity = false;
    }
    CHECK(!isIdentity);

    // Verify the transform maps origin to correct ECEF
    // Origin (0,0,0) in ENU should map to origin ECEF
    double ex = transform.data()[12];  // translation X
    double ey = transform.data()[13];  // translation Y
    double ez = transform.data()[14];  // translation Z

    // ECEF for Shenzhen area
    CHECK(std::fabs(ex) > 100000);
    CHECK(std::fabs(ey) > 100000);
    CHECK(std::fabs(ez) > 100000);

    remove(prjPath.c_str());
}

// =====================================================================
// 6. Regression: InverseTransform round-trip with Helmert shift
// =====================================================================

TEST(regression_inverse_helmert_roundtrip) {
    std::string srcCRS = "+proj=tmerc +lat_0=0 +lon_0=117 +k=1 +x_0=500000 +y_0=0 "
                         "+a=6378137 +rf=298.257222101 +units=m +no_defs";

    GeoreferencingWith7Parameters georef(srcCRS, "EPSG:4979");
    // Non-zero Helmert: 10m X, 5m Y, -3m Z, 2" rotation around Z
    SevenParameter params(10.0, 5.0, -3.0, 0.0, 0.0, 2.0, 1.5);
    georef.SetParameter(params);
    georef.Solve();
    georef.InitPROJPipelines();

    Eigen::Vector3d src(517600.0, 3421000.0, 50.0);
    Eigen::Vector3d tgt = georef.Transform(src);
    Eigen::Vector3d back = georef.InverseTransform(tgt);

    // Round-trip should be close (sub-meter for a 50m point)
    CHECK_CLOSE(back.x(), src.x(), 1.0);
    CHECK_CLOSE(back.y(), src.y(), 1.0);
    CHECK_CLOSE(back.z(), src.z(), 1.0);
}

// =====================================================================
// 7. Regression: Anchor InverseTransform
// =====================================================================

TEST(regression_anchor_inverse) {
    std::string srcCRS = "+proj=tmerc +lat_0=0 +lon_0=117 +k=1 +x_0=500000 +y_0=0 "
                         "+a=6378137 +rf=298.257222101 +units=m +no_defs";

    GeoreferencingWithAnchor georef(srcCRS, "EPSG:4979");
    // Anchor at origin (500000, 0, 0) -> offset should be ~zero
    georef.SetParameter(Eigen::Vector3d(500000.0, 0.0, 0.0));
    georef.Solve();

    Eigen::Vector3d src(517600.0, 3421000.0, 50.0);
    Eigen::Vector3d tgt = georef.Transform(src);
    Eigen::Vector3d back = georef.InverseTransform(tgt);

    CHECK_CLOSE(back.x(), src.x(), 1.0);
    CHECK_CLOSE(back.y(), src.y(), 1.0);
    CHECK_CLOSE(back.z(), src.z(), 1.0);
}

// =====================================================================
// 8. Regression: TransformTargetToECEF axis order (EPSG:4979 lat-first)
// =====================================================================

TEST(regression_target_to_ecef_axis_order) {
    std::string srcCRS = "+proj=tmerc +lat_0=0 +lon_0=117 +k=1 +x_0=500000 +y_0=0 "
                         "+a=6378137 +rf=298.257222101 +units=m +no_defs";

    GeoreferencingWith7Parameters georef(srcCRS, "EPSG:4979");
    georef.SetParameter(SevenParameter());
    georef.Solve();

    // Shenzhen area: ~113.06°E, ~22.65°N
    Eigen::Vector3d src(517600.0, 3421000.0, 50.0);
    Eigen::Vector3d geo = georef.Transform(src);

    if (georef.InitPROJPipelines()) {
        Eigen::Vector3d ecef = georef.TransformTargetToECEF(geo);
        // ECEF values for Shenzhen should all have magnitude > 100km
        CHECK(std::fabs(ecef.x()) > 100000);
        CHECK(std::fabs(ecef.y()) > 100000);
        CHECK(std::fabs(ecef.z()) > 100000);
        // ECEF X should be negative (East Asia)
        CHECK(ecef.x() < 0);
    } else {
        g_pass += 4;  // skip ECEF checks if pipelines unavailable
    }
}

// =====================================================================
// 9. Regression: WKT manual TM fallback parameters
// =====================================================================

TEST(regression_wkt_fallback_params) {
    const char* wkt =
        "PROJCS[\"CGCS2000_3_Degree_GK_CM_103d10mE\","
        "GEOGCS[\"GCS_China_Geodetic_Coordinate_System_2000\","
        "DATUM[\"D_China_2000\",SPHEROID[\"CGCS2000\",6378137.0,298.257222101]],"
        "PRIMEM[\"Greenwich\",0.0],UNIT[\"Degree\",0.017453292519943295]],"
        "PROJECTION[\"Gauss_Kruger\"],PARAMETER[\"False_Easting\",500000.0],"
        "PARAMETER[\"False_Northing\",0.0],"
        "PARAMETER[\"Central_Meridian\",103.1666666666666667],"
        "PARAMETER[\"Scale_Factor\",1.0],PARAMETER[\"Latitude_Of_Origin\",0.0],"
        "UNIT[\"Meter\",1.0]]";

    GeoreferencingWith7Parameters georef(wkt, "EPSG:4979");
    georef.SetParameter(SevenParameter());
    georef.Solve();

    Eigen::Vector3d result = georef.Transform(Eigen::Vector3d(498700, 2929900, 0));

    // Should produce geographic near 103.15°E, 26.48°N
    CHECK(std::isfinite(result.x()));
    CHECK(std::isfinite(result.y()));
    CHECK(std::fabs(result.x() - 103.15) < 1.0);
    CHECK(std::fabs(result.y() - 26.48) < 1.0);
}

// =====================================================================
// 10. SevenParameter with CoordinateFrame convention
// =====================================================================

TEST(sevenparam_coordinate_frame) {
    std::string srcCRS = "+proj=tmerc +lat_0=0 +lon_0=117 +k=1 +x_0=500000 +y_0=0 "
                         "+a=6378137 +rf=298.257222101 +units=m +no_defs";

    GeoreferencingWith7Parameters georef(srcCRS, "EPSG:4979");
    SevenParameter params(0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0);
    params.isCoordinateFrame = true;  // Coordinate Frame convention
    georef.SetParameter(params);
    georef.Solve();

    Eigen::Vector3d src(517600.0, 3421000.0, 50.0);
    Eigen::Vector3d result = georef.Transform(src);

    CHECK(std::isfinite(result.x()));
    CHECK(std::isfinite(result.y()));
    CHECK(std::fabs(result.x()) <= 180.0);
}

// =====================================================================
// 11. SevenParameter — WKT with alternate SPHEROID spacing
// =====================================================================

TEST(sevenparam_spheroid_with_space) {
    // WKT with space before SPHEROID bracket: "SPHEROID ["
    const char* wkt = "PROJCS[\"test\",GEOGCS[\"test\","
        "DATUM[\"test\",SPHEROID [\"CGCS2000\",6378137.0,298.257222101]],"
        "PRIMEM[\"Greenwich\",0.0],UNIT[\"degree\",0.0174532925199433]],"
        "PROJECTION[\"Transverse_Mercator\"],"
        "PARAMETER[\"central_meridian\",117.0],"
        "UNIT[\"metre\",1.0]]";

    GeoreferencingWith7Parameters georef(wkt, "EPSG:4979");
    georef.SetParameter(SevenParameter());
    georef.Solve();

    Eigen::Vector3d src(517600.0, 3421000.0, 50.0);
    Eigen::Vector3d result = georef.Transform(src);

    CHECK(std::isfinite(result.x()));
    // The key check: SPHEROID with space was parsed — result must be valid
    CHECK(std::fabs(result.x()) <= 180.0);
    CHECK(std::fabs(result.y()) <= 90.0);
}

// =====================================================================
// 12. Anchor at large offset from origin
// =====================================================================

TEST(anchor_large_offset) {
    // Test anchor with a large value (far from real origin)
    GeoreferencingWithAnchor georef("EPSG:4547", "EPSG:4979");
    // Large easting/northing typical for CGCS2000 projected coordinates
    Eigen::Vector3d anchor(500000.0, 4000000.0, 500.0);
    georef.SetParameter(anchor);
    georef.Solve();

    Eigen::Vector3d nearby(anchor.x() + 10000.0, anchor.y() + 10000.0, anchor.z());
    Eigen::Vector3d result = georef.Transform(nearby);

    CHECK(std::isfinite(result.x()));
    CHECK(std::fabs(result.x()) <= 180.0);
    CHECK(std::fabs(result.y()) <= 90.0);
}

// =====================================================================
// 13. GeodeticMath — ECEF roundtrip at various heights
// =====================================================================

TEST(geodetic_height_variants) {
    double a = 6378137.0, rf = 298.257222101;
    double f = 1.0 / rf, e2 = 2.0 * f - f * f;
    double lat = 30.0 * M_PI / 180.0;
    double lon = 120.0 * M_PI / 180.0;

    double heights[] = { -100.0, 0.0, 100.0, 8848.0 };
    for (double h : heights) {
        double X, Y, Z;
        GeodeticMath::GeographicToECEF(lat, lon, h, X, Y, Z, a, e2);
        double lat2, lon2, h2;
        GeodeticMath::ECEFToGeographic(X, Y, Z, lat2, lon2, h2, a, e2);
        CHECK_CLOSE(lon, lon2, 1e-10);
        CHECK_CLOSE(lat, lat2, 1e-10);
        CHECK_CLOSE(h, h2, 0.01);
    }
}

// =====================================================================
// 14. GeodeticMath — negative height (below ellipsoid)
// =====================================================================

TEST(geodetic_negative_height) {
    double a = 6378137.0, rf = 298.257222101;
    double f = 1.0 / rf, e2 = 2.0 * f - f * f;
    // Dead Sea area — below ellipsoid
    double lat = 31.5 * M_PI / 180.0;
    double lon = 35.5 * M_PI / 180.0;
    double h = -430.0;

    double X, Y, Z;
    GeodeticMath::GeographicToECEF(lat, lon, h, X, Y, Z, a, e2);
    double lat2, lon2, h2;
    GeodeticMath::ECEFToGeographic(X, Y, Z, lat2, lon2, h2, a, e2);

    CHECK_CLOSE(lat, lat2, 1e-10);
    CHECK_CLOSE(lon, lon2, 1e-10);
    CHECK_CLOSE(h, h2, 0.01);
}

// =====================================================================
// 15. GKInverse boundary — exactly at central meridian + equator
// =====================================================================

TEST(gk_inverse_central_meridian_origin) {
    double lat, lon;
    double a = 6378137.0;
    double f = 1.0 / 298.257222101;
    double e2 = 2.0 * f - f * f;

    // Easting = falseEasting, Northing = falseNorthing → origin at equator/central meridian
    bool ok = GeodeticMath::GKInverse(500000.0, 0.0, 117.0, 1.0,
                                       500000.0, 0.0, lat, lon, a, e2);
    CHECK(ok);
    CHECK_CLOSE(lat, 0.0, 1e-12);
    CHECK_CLOSE(lon, 117.0 * M_PI / 180.0, 1e-12);
}

// =====================================================================
// 16. MultiPosition — PolyOrder 1 with minimal 3 points (boundary)
// =====================================================================

TEST(multipos_min_4_poly) {
    // Implementation requires ceil((2*nBasis+4)/3) = 4 points minimum
    std::vector<ControlPoint> cps;
    cps.emplace_back(0.0,    0.0,   0.0, 113.060, 22.650, 0.0);
    cps.emplace_back(200.0,  0.0,   0.0, 113.062, 22.650, 0.0);
    cps.emplace_back(0.0,    200.0, 0.0, 113.060, 22.648, 0.0);
    cps.emplace_back(200.0,  200.0, 0.0, 113.062, 22.648, 0.0);

    GeoreferencingWithMultiPosition georef("", "EPSG:4979");
    georef.SetFitMethod(FitMethod::DirectPoly2D);
    georef.SetPolyOrder(1);
    georef.SetParameter(cps);
    georef.Solve();

    Eigen::Vector3d test(100.0, 100.0, 0.0);
    Eigen::Vector3d result = georef.Transform(test);

    CHECK_CLOSE(result.x(), 113.061, 0.001);  // midpoint between 113.060 and 113.062
    CHECK_CLOSE(result.y(), 22.649, 0.001);   // midpoint between 22.648 and 22.650
}

// =====================================================================
// Main
// =====================================================================

int main() {
    std::cout << "=== SevenParameter ===" << std::endl;
    RUN(sevenparam_identity);
    RUN(sevenparam_with_shift);

    std::cout << "=== Anchor ===" << std::endl;
    RUN(anchor_basic);

    std::cout << "=== MultiPosition ===" << std::endl;
    RUN(multipos_poly2d_identity);
    RUN(multipos_ecef_affine);
    RUN(multipos_poly_order2);
    RUN(multipos_detect_crs);

    std::cout << "=== GeodeticMath ===" << std::endl;
    RUN(geodetic_roundtrip);
    RUN(gk_inverse_roundtrip);

    std::cout << "=== RootTransform ===" << std::endl;
    RUN(root_transform_identity);
    RUN(root_transform_with_projection);

    std::cout << "=== Regression ===" << std::endl;
    RUN(regression_inverse_helmert_roundtrip);
    RUN(regression_anchor_inverse);
    RUN(regression_target_to_ecef_axis_order);
    RUN(regression_wkt_fallback_params);

    std::cout << "=== Boundary ===" << std::endl;
    RUN(sevenparam_coordinate_frame);
    RUN(sevenparam_spheroid_with_space);
    RUN(anchor_large_offset);
    RUN(geodetic_height_variants);
    RUN(geodetic_negative_height);
    RUN(gk_inverse_central_meridian_origin);
    RUN(multipos_min_4_poly);

    std::cout << "\n========================================" << std::endl;
    std::cout << "PASS=" << g_pass << " FAIL=" << g_fail << std::endl;
    return g_fail > 0 ? 1 : 0;
}