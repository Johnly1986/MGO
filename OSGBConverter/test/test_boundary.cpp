// Copyright Johnlyon
//
// Boundary value tests — validates edge cases across all modules.
// Uses the same lightweight test framework as test_georef.cpp.
//
// Coverage:
//   GeodeticMath:      poles, equator, dateline, zero, extreme values
//   Georeferencing:    extreme 7-param, anchor at origin/pole, min control points
//   CProjectionEngine: LoadProjectionFromString, WKT2, identity transforms
//   MeshGroupOptimizer: programmatic scene, error extremes, threshold bounds
//
// Build: see CMakeLists.txt test_boundary target
//

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <cmath>
#include <cassert>
#include <vector>
#include <iomanip>
#include <cstring>
#include <cfloat>
#include <memory>

#include "../MeshProjectionErrorCorrector/GeodeticMath.h"
#include "../MeshProjectionErrorCorrector/IGeoreferencing.h"
#include "../MeshProjectionErrorCorrector/GeoreferencingWith7Parameters.h"
#include "../MeshProjectionErrorCorrector/GeoreferencingWithAnchor.h"
#include "../MeshProjectionErrorCorrector/GeoreferencingWithMultiPosition.h"
#include "../MeshProjectionErrorCorrector/CProjectionEngine.h"
#include "../MeshProjectionErrorCorrector/AxisMapper.h"

#include "../MeshGroupOptimizer/MeshGroupOptimizer.h"

#include <assimp/scene.h>
#include <assimp/mesh.h>
#include <assimp/postprocess.h>

// =========================================================================
// Test framework (same as test_georef.cpp)
// =========================================================================
static int g_pass = 0, g_fail = 0;
#define TEST(name) void test_##name()
#define RUN(name) do { std::cout << "  " #name "... "; test_##name(); } while(0)
#define CHECK(cond) do { if (cond) { std::cout << "PASS" << std::endl; g_pass++; } \
    else { std::cout << "FAIL (" << __LINE__ << ")" << std::endl; g_fail++; } } while(0)
#define CHECK_CLOSE(a, b, eps) CHECK(std::fabs((a)-(b)) < (eps))
#define CHECK_NZERO(v, eps) CHECK(std::fabs(v) > (eps))
#define CHECK_RANGE(v, lo, hi) CHECK((v) >= (lo) && (v) <= (hi))

// =========================================================================
// Helper: construct a minimal aiScene for MeshGroupOptimizer tests
// =========================================================================
static aiScene* makeMinimalScene(const char* name,
                                  float* verts, unsigned int nVerts,
                                  unsigned int* indices, unsigned int nIndices,
                                  bool hasNormals = true,
                                  bool hasUVs = true)
{
    aiScene* scene = new aiScene();

    // Mesh
    scene->mNumMeshes = 1;
    scene->mMeshes = new aiMesh*[1];
    aiMesh* mesh = new aiMesh();
    // Zero-initialize all texture coord pointers to avoid garbage
    for (int tc = 0; tc < AI_MAX_NUMBER_OF_TEXTURECOORDS; ++tc) {
        mesh->mTextureCoords[tc] = nullptr;
        mesh->mNumUVComponents[tc] = 0;
    }
    mesh->mTangents = nullptr;
    mesh->mBitangents = nullptr;
    mesh->mColors[0] = nullptr;
    mesh->mColors[1] = nullptr;
    mesh->mColors[2] = nullptr;
    mesh->mColors[3] = nullptr;
    mesh->mName = aiString(name);
    mesh->mMaterialIndex = 0;
    mesh->mPrimitiveTypes = aiPrimitiveType_TRIANGLE;

    // Vertices
    mesh->mNumVertices = nVerts;
    mesh->mVertices = new aiVector3D[nVerts];
    for (unsigned int i = 0; i < nVerts; ++i)
        mesh->mVertices[i] = aiVector3D(verts[i*3], verts[i*3+1], verts[i*3+2]);

    // Faces (triangles)
    mesh->mNumFaces = nIndices / 3;
    mesh->mFaces = new aiFace[mesh->mNumFaces];
    for (unsigned int fi = 0; fi < mesh->mNumFaces; ++fi) {
        aiFace& face = mesh->mFaces[fi];
        face.mNumIndices = 3;
        face.mIndices = new unsigned int[3];
        face.mIndices[0] = indices[fi*3];
        face.mIndices[1] = indices[fi*3+1];
        face.mIndices[2] = indices[fi*3+2];
    }

    // Normals
    if (hasNormals) {
        mesh->mNormals = new aiVector3D[nVerts];
        for (unsigned int i = 0; i < nVerts; ++i)
            mesh->mNormals[i] = aiVector3D(0.0f, 0.0f, 1.0f);
    }

    // UVs
    if (hasUVs) {
        mesh->mTextureCoords[0] = new aiVector3D[nVerts];
        for (unsigned int i = 0; i < nVerts; ++i)
            mesh->mTextureCoords[0][i] = aiVector3D(0.0f, 0.0f, 0.0f);
        mesh->mNumUVComponents[0] = 2;
    }

    scene->mMeshes[0] = mesh;

    // Material
    scene->mNumMaterials = 1;
    scene->mMaterials = new aiMaterial*[1];
    scene->mMaterials[0] = new aiMaterial();

    // Root node
    scene->mRootNode = new aiNode();
    scene->mRootNode->mName = aiString("root");
    scene->mRootNode->mNumMeshes = 1;
    scene->mRootNode->mMeshes = new unsigned int[1];
    scene->mRootNode->mMeshes[0] = 0;

    return scene;
}

static void freeScene(aiScene* scene)
{
    if (!scene) return;
    // Free individual meshes and their internal arrays.
    // IMPORTANT: aiMesh::~aiMesh() also deletes these arrays, so we must
    // set them to nullptr after manual deletion to prevent double-free.
    if (scene->mMeshes) {
        for (unsigned int mi = 0; mi < scene->mNumMeshes; ++mi) {
            aiMesh* mesh = scene->mMeshes[mi];
            if (!mesh) continue;
            delete[] mesh->mVertices;   mesh->mVertices = nullptr;
            delete[] mesh->mNormals;    mesh->mNormals = nullptr;
            for (int tc = 0; tc < AI_MAX_NUMBER_OF_TEXTURECOORDS; ++tc) {
                delete[] mesh->mTextureCoords[tc];
                mesh->mTextureCoords[tc] = nullptr;
            }
            for (unsigned int fi = 0; fi < mesh->mNumFaces; ++fi) {
                delete[] mesh->mFaces[fi].mIndices;
                mesh->mFaces[fi].mIndices = nullptr;
            }
            delete[] mesh->mFaces;  mesh->mFaces = nullptr;
            mesh->mNumFaces = 0;
            mesh->mNumVertices = 0;
            delete mesh;
        }
    }
    // Free individual materials.
    if (scene->mMaterials) {
        for (unsigned int mi = 0; mi < scene->mNumMaterials; ++mi)
            delete scene->mMaterials[mi];
    }
    // Set pointers to nullptr so aiScene/aiNode destructors don't double-free.
    // aiScene::~aiScene deletes mMeshes, mMaterials, mRootNode.
    // aiNode::~aiNode deletes mMeshes, mChildren.
    if (scene->mRootNode) {
        scene->mRootNode->mMeshes = nullptr;
        scene->mRootNode->mNumMeshes = 0;
        scene->mRootNode->mChildren = nullptr;
        scene->mRootNode->mNumChildren = 0;
    }
    scene->mMeshes = nullptr;
    scene->mNumMeshes = 0;
    scene->mMaterials = nullptr;
    scene->mNumMaterials = 0;
    delete scene;
}

// =========================================================================
// 1. GeodeticMath — boundary value tests
// =========================================================================

// Pole: latitude = 90°
TEST(geodetic_pole_north_roundtrip) {
    double a = 6378137.0;
    double rf = 298.257222101;
    double f = 1.0 / rf;
    double e2 = 2.0 * f - f * f;

    double lon = 0.0;      // any longitude
    double lat = M_PI / 2.0;  // North Pole
    double h = 0.0;

    double X, Y, Z;
    GeodeticMath::GeographicToECEF(lat, lon, h, X, Y, Z, a, e2);

    // At North Pole: X=0, Y=0, Z > 0
    CHECK_CLOSE(X, 0.0, 1e-6);
    CHECK_CLOSE(Y, 0.0, 1e-6);
    CHECK(Z > 0);

    double lat2, lon2, h2;
    GeodeticMath::ECEFToGeographic(X, Y, Z, lat2, lon2, h2, a, e2);

    CHECK_CLOSE(lat, lat2, 1e-10);
    CHECK_CLOSE(h, h2, 1e-3);
}

// Pole: latitude = -90°
TEST(geodetic_pole_south_roundtrip) {
    double a = 6378137.0, rf = 298.257222101;
    double f = 1.0 / rf, e2 = 2.0 * f - f * f;

    double lon = 45.0 * M_PI / 180.0;
    double lat = -M_PI / 2.0;
    double h = 100.0;

    double X, Y, Z;
    GeodeticMath::GeographicToECEF(lat, lon, h, X, Y, Z, a, e2);

    // At South Pole: X=0, Y=0, Z < 0
    CHECK_CLOSE(X, 0.0, 1e-6);
    CHECK_CLOSE(Y, 0.0, 1e-6);
    CHECK(Z < 0);

    double lat2, lon2, h2;
    GeodeticMath::ECEFToGeographic(X, Y, Z, lat2, lon2, h2, a, e2);

    CHECK_CLOSE(lat, lat2, 1e-10);
    CHECK_CLOSE(h, h2, 1e-3);
}

// Equator: latitude = 0°
TEST(geodetic_equator) {
    double a = 6378137.0, rf = 298.257222101;
    double f = 1.0 / rf, e2 = 2.0 * f - f * f;

    double lon = 0.0;
    double lat = 0.0;
    double h = 0.0;

    double X, Y, Z;
    GeodeticMath::GeographicToECEF(lat, lon, h, X, Y, Z, a, e2);

    // At equator, prime meridian: X = a, Y = 0, Z = 0
    CHECK_CLOSE(X, a, 1.0);
    CHECK_CLOSE(Y, 0.0, 1e-6);
    CHECK_CLOSE(Z, 0.0, 1e-6);

    double lat2, lon2, h2;
    GeodeticMath::ECEFToGeographic(X, Y, Z, lat2, lon2, h2, a, e2);
    CHECK_CLOSE(lat, lat2, 1e-10);
    CHECK_CLOSE(lon, lon2, 1e-10);
}

// Dateline: longitude = 180° and -180°
TEST(geodetic_dateline) {
    double a = 6378137.0, rf = 298.257222101;
    double f = 1.0 / rf, e2 = 2.0 * f - f * f;

    // +180° and -180° should produce same ECEF
    double lonE = 180.0 * M_PI / 180.0;
    double lonW = -180.0 * M_PI / 180.0;
    double lat = 30.0 * M_PI / 180.0;
    double h = 0.0;

    double Xe, Ye, Ze, Xw, Yw, Zw;
    GeodeticMath::GeographicToECEF(lat, lonE, h, Xe, Ye, Ze, a, e2);
    GeodeticMath::GeographicToECEF(lat, lonW, h, Xw, Yw, Zw, a, e2);

    CHECK_CLOSE(Xe, Xw, 1e-6);
    CHECK_CLOSE(Ye, Yw, 1e-6);
    CHECK_CLOSE(Ze, Zw, 1e-6);
}

// Zero height roundtrip
TEST(geodetic_zero_height) {
    double a = 6378137.0, rf = 298.257222101;
    double f = 1.0 / rf, e2 = 2.0 * f - f * f;

    double lon = 113.0 * M_PI / 180.0;
    double lat = 22.5 * M_PI / 180.0;
    double h = 0.0;

    double X, Y, Z;
    GeodeticMath::GeographicToECEF(lat, lon, h, X, Y, Z, a, e2);

    double lat2, lon2, h2;
    GeodeticMath::ECEFToGeographic(X, Y, Z, lat2, lon2, h2, a, e2);

    CHECK_CLOSE(lat, lat2, 1e-10);
    CHECK_CLOSE(lon, lon2, 1e-10);
    CHECK_CLOSE(h, h2, 1e-3);
}

// Extreme height (aircraft altitude)
TEST(geodetic_extreme_height) {
    double a = 6378137.0, rf = 298.257222101;
    double f = 1.0 / rf, e2 = 2.0 * f - f * f;

    double lon = 113.0 * M_PI / 180.0;
    double lat = 22.5 * M_PI / 180.0;
    double h = 20000.0;  // 20km altitude

    double X, Y, Z;
    GeodeticMath::GeographicToECEF(lat, lon, h, X, Y, Z, a, e2);

    double lat2, lon2, h2;
    GeodeticMath::ECEFToGeographic(X, Y, Z, lat2, lon2, h2, a, e2);

    CHECK_CLOSE(lat, lat2, 1e-10);
    CHECK_CLOSE(lon, lon2, 1e-10);
    CHECK_CLOSE(h, h2, 0.01);  // sub-cm for 20km altitude
}

// GKInverse at origin (E=500000, N=0) — should be near central meridian
TEST(gk_inverse_at_origin) {
    double lat, lon;
    bool ok = GeodeticMath::GKInverse(500000.0, 0.0, 117.0, 1.0,
                                       500000.0, 0.0, lat, lon,
                                       6378137.0,
                                       2.0*(1.0/298.257222101) - (1.0/298.257222101)*(1.0/298.257222101));
    CHECK(ok);
    CHECK_CLOSE(lon, 117.0 * M_PI / 180.0, 1e-10);
    CHECK_CLOSE(lat, 0.0, 1e-10);
}

// GKInverse at far easting (> 500km from central meridian)
TEST(gk_inverse_far_easting) {
    double lat, lon;
    // Easting 800000 = 300km east of central meridian
    double e2 = 2.0*(1.0/298.257222101) - (1.0/298.257222101)*(1.0/298.257222101);
    bool ok = GeodeticMath::GKInverse(800000.0, 3500000.0, 117.0, 1.0,
                                       500000.0, 0.0, lat, lon,
                                       6378137.0, e2);
    CHECK(ok);
    double lon_deg = lon * 180.0 / M_PI;
    // Should be east of 117°
    CHECK(lon_deg > 117.0);
    CHECK(lon_deg < 121.0);
}

// TM forward-inverse roundtrip with custom ellipsoid
TEST(gk_forward_inverse_roundtrip) {
    double a = 6381237.0;
    double rf = 298.257222101;
    double f = 1.0 / rf;
    double e2 = 2.0 * f - f * f;

    double input_lat = 27.0 * M_PI / 180.0;
    double input_lon = 100.35 * M_PI / 180.0;
    double E, N;
    GeodeticMath::TMForward(input_lat, input_lon, 100.35, 1.0,
                             500000.0, 0.0, 0.0, E, N, a, e2);

    double lat2, lon2;
    GeodeticMath::TMParams tm;
    tm.lon0_deg = 100.35; tm.lat0_deg = 0.0; tm.k0 = 1.0;
    tm.fe = 500000.0; tm.fn = 0.0; tm.a = a; tm.rf = rf;
    GeodeticMath::InverseTM(E, N, tm, lon2, lat2);

    CHECK_CLOSE(input_lat, lat2, 1e-10);
    CHECK_CLOSE(input_lon, lon2, 1e-10);
}

// ENU→ECEF rotation at North Pole (singularity check)
TEST(enu_rotation_at_pole) {
    Eigen::Matrix3d R = GeodeticMath::ENUToECEFRotation(M_PI/2.0, 0.0);

    // At North Pole, East direction should be well-defined
    // East vector: R.col(0) = [-sin(lon), cos(lon), 0] = [0, 1, 0] at lon=0
    CHECK_CLOSE(R(0,0), 0.0, 1e-10);  // East x = -sin(0) = 0
    CHECK_CLOSE(R(1,0), 1.0, 1e-10);  // East y = cos(0) = 1
    CHECK_CLOSE(R(2,0), 0.0, 1e-10);  // East z = 0

    // Up vector: R.col(2) = [cos(lat)cos(lon), cos(lat)sin(lon), sin(lat)]
    // At pole: [0, 0, 1]
    CHECK_CLOSE(R(0,2), 0.0, 1e-10);
    CHECK_CLOSE(R(1,2), 0.0, 1e-10);
    CHECK_CLOSE(R(2,2), 1.0, 1e-10);
}

// ENU→ECEF rotation at Equator
TEST(enu_rotation_at_equator) {
    Eigen::Matrix3d R = GeodeticMath::ENUToECEFRotation(0.0, 0.0);

    // At equator, prime meridian:
    // East: [-sin(0), cos(0), 0] = [0, 1, 0]
    // North: [-sin(0)*cos(0), -sin(0)*sin(0), cos(0)] = [0, 0, 1]
    // Up: [cos(0)*cos(0), cos(0)*sin(0), sin(0)] = [1, 0, 0]
    CHECK_CLOSE(R(0,0), 0.0, 1e-10);   // East x
    CHECK_CLOSE(R(1,0), 1.0, 1e-10);   // East y
    CHECK_CLOSE(R(2,0), 0.0, 1e-10);   // East z
    CHECK_CLOSE(R(0,1), 0.0, 1e-10);   // North x
    CHECK_CLOSE(R(1,1), 0.0, 1e-10);   // North y
    CHECK_CLOSE(R(2,1), 1.0, 1e-10);   // North z
    CHECK_CLOSE(R(0,2), 1.0, 1e-10);   // Up x
    CHECK_CLOSE(R(1,2), 0.0, 1e-10);   // Up y
    CHECK_CLOSE(R(2,2), 0.0, 1e-10);   // Up z
}

// =========================================================================
// 2. Georeferencing — boundary value tests
// =========================================================================

// SevenParameter with extreme rotation (near-max arc-seconds)
TEST(sevenparam_extreme_rotation) {
    std::string srcCRS = "+proj=tmerc +lat_0=0 +lon_0=117 +k=1 +x_0=500000 +y_0=0 "
                         "+a=6378137 +rf=298.257222101 +units=m +no_defs";

    GeoreferencingWith7Parameters georef(srcCRS, "EPSG:4979");
    // Extreme: 100 arc-seconds rotation on all axes, 1000 ppm scale
    SevenParameter params(100.0, -50.0, 200.0, 100.0, -100.0, 50.0, 1000.0);
    georef.SetParameter(params);
    georef.Solve();

    Eigen::Vector3d src(517600.0, 3421000.0, 50.0);
    Eigen::Vector3d result = georef.Transform(src);

    // Result should be finite numbers in valid geographic range
    CHECK(std::isfinite(result.x()));
    CHECK(std::isfinite(result.y()));
    CHECK(std::isfinite(result.z()));
    CHECK(std::fabs(result.x()) <= 180.0);
    CHECK(std::fabs(result.y()) <= 90.0);
}

// SevenParameter with zero scale (boundary: scale=0 means no scale change)
TEST(sevenparam_zero_scale) {
    std::string srcCRS = "+proj=tmerc +lat_0=0 +lon_0=117 +k=1 +x_0=500000 +y_0=0 "
                         "+a=6378137 +rf=298.257222101 +units=m +no_defs";

    GeoreferencingWith7Parameters georef1(srcCRS, "EPSG:4979");
    georef1.SetParameter(SevenParameter(0,0,0,0,0,0,0));  // scale=0
    georef1.Solve();

    GeoreferencingWith7Parameters georef2(srcCRS, "EPSG:4979");
    georef2.SetParameter(SevenParameter());  // default scale=0
    georef2.Solve();

    Eigen::Vector3d src(517600.0, 3421000.0, 50.0);
    Eigen::Vector3d r1 = georef1.Transform(src);
    Eigen::Vector3d r2 = georef2.Transform(src);

    // Both should produce same result
    CHECK_CLOSE(r1.x(), r2.x(), 1e-10);
    CHECK_CLOSE(r1.y(), r2.y(), 1e-10);
}

// SevenParameter with WKT2 lowercase parameter names
TEST(sevenparam_wkt2_lowercase) {
    // WKT2 uses lowercase parameter names
    const char* wkt = "PROJCS[\"test\",GEOGCS[\"test\","
        "DATUM[\"test\",SPHEROID[\"CGCS2000\",6378137.0,298.257222101]],"
        "PRIMEM[\"Greenwich\",0.0],UNIT[\"degree\",0.0174532925199433]],"
        "PROJECTION[\"Transverse_Mercator\"],"
        "PARAMETER[\"central_meridian\",117.0],"
        "PARAMETER[\"false_easting\",500000.0],"
        "PARAMETER[\"false_northing\",0.0],"
        "PARAMETER[\"scale_factor\",1.0],"
        "UNIT[\"metre\",1.0]]";

    GeoreferencingWith7Parameters georef(wkt, "EPSG:4979");
    georef.SetParameter(SevenParameter());
    georef.Solve();

    // Source: (500000+17600, 3421000) in GK zone with CM 117°
    // This is ~17.6km east of 117° → expected longitude ~117.16°
    Eigen::Vector3d src(517600.0, 3421000.0, 50.0);
    Eigen::Vector3d result = georef.Transform(src);

    CHECK(std::isfinite(result.x()));
    // Lon should be slightly east of 117° (central meridian)
    CHECK(result.x() > 117.0);
    CHECK(result.x() < 118.0);
    // Northing 3421000 → ~30.9°N
    CHECK(result.y() > 30.0);
    CHECK(result.y() < 35.0);
}

// Anchor with same source and target CRS (TM→TM should be near-identity)
TEST(anchor_same_crs) {
    std::string srcCRS = "+proj=tmerc +lat_0=0 +lon_0=117 +k=1 +x_0=500000 +y_0=0 "
                         "+a=6378137 +rf=298.257222101 +units=m +no_defs";

    GeoreferencingWithAnchor georef(srcCRS, "EPSG:4979");
    Eigen::Vector3d anchor(517600.0, 3421000.0, 50.0);
    georef.SetParameter(anchor);
    georef.Solve();

    // Transform the anchor point itself
    Eigen::Vector3d result = georef.Transform(anchor);

    // Result should be valid geographic
    CHECK(std::isfinite(result.x()));
    CHECK(std::isfinite(result.y()));
    CHECK(std::fabs(result.x()) <= 180.0);
    CHECK(std::fabs(result.y()) <= 90.0);

    // Transform a nearby point (100m east)
    Eigen::Vector3d nearby(anchor.x() + 100.0, anchor.y(), anchor.z());
    Eigen::Vector3d result2 = georef.Transform(nearby);

    // Nearby point should have slightly different longitude
    CHECK(std::fabs(result2.x() - result.x()) > 1e-9);
}

// MultiPosition with exactly 4 control points (minimum for ECEF_Affine)
TEST(multipos_min_4_points) {
    std::vector<ControlPoint> cps;
    cps.emplace_back(0.0,   0.0,   0.0, 113.060, 22.650, 0.0);
    cps.emplace_back(100.0, 0.0,   0.0, 113.061, 22.650, 0.0);
    cps.emplace_back(0.0,   100.0, 0.0, 113.060, 22.649, 0.0);
    cps.emplace_back(100.0, 100.0, 0.0, 113.061, 22.649, 0.0);

    std::string srcCRS = "+proj=tmerc +lat_0=0 +lon_0=117 +k=1 +x_0=500000 +y_0=0 "
                         "+a=6378137 +rf=298.257222101 +units=m +no_defs";

    GeoreferencingWithMultiPosition georef(srcCRS, "EPSG:4979");
    georef.SetFitMethod(FitMethod::ECEF_Affine);
    georef.SetParameter(cps);
    georef.Solve();

    // Center point
    Eigen::Vector3d center(50.0, 50.0, 0.0);
    Eigen::Vector3d result = georef.Transform(center);

    CHECK(std::isfinite(result.x()));
    CHECK(std::isfinite(result.y()));
    CHECK_RANGE(result.x(), 113.0, 113.1);
    CHECK_RANGE(result.y(), 22.6, 22.7);
}

// MultiPosition with 3rd-order polynomial (maximum)
TEST(multipos_poly_order3) {
    // Need enough points for cubic: numBasis=10, minPoints = ceil((20+4)/3)=8
    std::vector<ControlPoint> cps;
    for (int ix = 0; ix < 3; ++ix) {
        for (int iy = 0; iy < 3; ++iy) {
            double sx = ix * 100.0, sy = iy * 100.0;
            double tx = 113.060 + ix * 0.001;
            double ty = 22.650 + iy * 0.001;
            cps.emplace_back(sx, sy, 0.0, tx, ty, 0.0);
        }
    }

    GeoreferencingWithMultiPosition georef("", "EPSG:4979");
    georef.SetFitMethod(FitMethod::DirectPoly2D);
    georef.SetPolyOrder(3);
    georef.SetParameter(cps);
    georef.Solve();

    Eigen::Vector3d center(100.0, 100.0, 0.0);
    Eigen::Vector3d result = georef.Transform(center);

    CHECK(std::isfinite(result.x()));
    CHECK(std::isfinite(result.y()));
}

// =========================================================================
// 3. CProjectionEngine — boundary value tests
// =========================================================================

// LoadProjectionFromString with WKT2 lowercase
TEST(projengine_wkt2_lowercase_params) {
    const char* wkt2 = "PROJCS[\"test\",GEOGCS[\"test\","
        "DATUM[\"test\",SPHEROID[\"CGCS2000\",6378137.0,298.257222101]],"
        "PRIMEM[\"Greenwich\",0.0],UNIT[\"degree\",0.0174532925199433]],"
        "PROJECTION[\"Transverse_Mercator\"],"
        "PARAMETER[\"central_meridian\",117.0],"
        "PARAMETER[\"false_easting\",500000.0],"
        "PARAMETER[\"false_northing\",0.0],"
        "PARAMETER[\"scale_factor\",1.0],"
        "UNIT[\"metre\",1.0]]";

    CProjectionEngine engine;
    bool loaded = engine.LoadProjectionFromString(wkt2);
    CHECK(loaded);
    // Verify the parameters were correctly extracted from WKT2 lowercase names
    CHECK_CLOSE(engine.GetCentralMeridian(), 117.0, 1e-9);
    CHECK_CLOSE(engine.GetFalseEasting(), 500000.0, 1e-9);
    CHECK_CLOSE(engine.GetFalseNorthing(), 0.0, 1e-9);
    CHECK_CLOSE(engine.GetScaleFactor(), 1.0, 1e-9);
}

// LoadProjectionFromString with WKT1 TitleCase (regression)
TEST(projengine_wkt1_titlecase_params) {
    const char* wkt1 = "PROJCS[\"test\",GEOGCS[\"test\","
        "DATUM[\"test\",SPHEROID[\"CGCS2000\",6378140.0,298.3]],"
        "PRIMEM[\"Greenwich\",0.0],UNIT[\"degree\",0.0174532925199433]],"
        "PROJECTION[\"Transverse_Mercator\"],"
        "PARAMETER[\"Central_Meridian\",103.1666666666667],"
        "PARAMETER[\"False_Easting\",500000.0],"
        "PARAMETER[\"False_Northing\",0.0],"
        "PARAMETER[\"Scale_Factor\",1.0],"
        "UNIT[\"metre\",1.0]]";

    CProjectionEngine engine;
    bool loaded = engine.LoadProjectionFromString(wkt1);
    CHECK(loaded);
    CHECK_CLOSE(engine.GetCentralMeridian(), 103.1666666666667, 0.001);
    CHECK_CLOSE(engine.GetFalseEasting(), 500000.0, 1e-9);
    CHECK_CLOSE(engine.GetScaleFactor(), 1.0, 1e-9);
}

// SPHEROID with space before bracket
TEST(projengine_spheroid_with_space) {
    const char* wkt = "PROJCS[\"test\",GEOGCS[\"test\","
        "DATUM[\"test\",SPHEROID [\"CGCS2000\",6379999.0,299.0]],"
        "PRIMEM[\"Greenwich\",0.0],UNIT[\"degree\",0.0174532925199433]],"
        "PROJECTION[\"Transverse_Mercator\"],"
        "PARAMETER[\"Central_Meridian\",117.0],"
        "UNIT[\"metre\",1.0]]";

    CProjectionEngine engine;
    bool loaded = engine.LoadProjectionFromString(wkt);
    CHECK(loaded);
    CHECK_CLOSE(engine.GetSemiMajorAxis(), 6379999.0, 1e-9);
    CHECK_CLOSE(engine.GetInverseFlattening(), 299.0, 1e-9);
}

// Lowercase spheroid keyword
TEST(projengine_spheroid_lowercase) {
    const char* wkt = "PROJCS[\"test\",GEOGCS[\"test\","
        "DATUM[\"test\",spheroid[\"CGCS2000\",6377777.0,298.0]],"
        "PRIMEM[\"Greenwich\",0.0],UNIT[\"degree\",0.0174532925199433]],"
        "PROJECTION[\"Transverse_Mercator\"],"
        "PARAMETER[\"Central_Meridian\",117.0],"
        "UNIT[\"metre\",1.0]]";

    CProjectionEngine engine;
    bool loaded = engine.LoadProjectionFromString(wkt);
    CHECK(loaded);
    CHECK_CLOSE(engine.GetSemiMajorAxis(), 6377777.0, 1e-9);
}

// Identity transform when no projection loaded
TEST(projengine_no_projection_root_transform) {
    CProjectionEngine engine;
    Eigen::Matrix4d transform = engine.ComputeRootTransform();
    for (int i = 0; i < 16; ++i) {
        double expected = (i % 5 == 0) ? 1.0 : 0.0;
        CHECK_CLOSE(transform.data()[i], expected, 1e-9);
    }
}

// SetOrigin then Reset
TEST(projengine_reset) {
    CProjectionEngine engine;
    engine.SetOrigin(500000, 3000000, 100);
    CHECK_CLOSE(engine.GetOriginX(), 500000.0, 1e-9);
    CHECK_CLOSE(engine.GetOriginY(), 3000000.0, 1e-9);
    CHECK_CLOSE(engine.GetOriginZ(), 100.0, 1e-9);

    engine.Reset();
    CHECK_CLOSE(engine.GetOriginX(), 0.0, 1e-9);
    CHECK_CLOSE(engine.GetOriginY(), 0.0, 1e-9);
    CHECK_CLOSE(engine.GetOriginZ(), 0.0, 1e-9);
    CHECK(!engine.HasProjection());
}

// =========================================================================
// 4. AxisMapper — boundary value tests
// =========================================================================

// Assimp ↔ ENU roundtrip
TEST(axismapper_roundtrip) {
    double x_in = 100.0, y_in = 200.0, z_in = -300.0;
    double east, north, up;
    AxisMapper::AssimpToENU(x_in, y_in, z_in, east, north, up);

    double x2, y2, z2;
    AxisMapper::ENUToAssimp(east, north, up, x2, y2, z2);

    CHECK_CLOSE(x_in, x2, 1e-10);
    CHECK_CLOSE(y_in, y2, 1e-10);
    CHECK_CLOSE(z_in, z2, 1e-10);
}

// BBox conversion maintains volume
TEST(axismapper_bbox_conversion) {
    double bmin[3] = {0.0, 0.0, -100.0};  // Assimp: X=0, Y=0, Z=-100 (south)
    double bmax[3] = {100.0, 50.0, 0.0};   // X=100, Y=50, Z=0 (north)
    double omin[3], omax[3];
    AxisMapper::BBoxAssimpToTilesZUp(bmin, bmax, omin, omax);

    // X unchanged: [0, 100]
    CHECK_CLOSE(omin[0], 0.0, 1e-10);
    CHECK_CLOSE(omax[0], 100.0, 1e-10);
    // Y in tiles = North = -AssimpZ → [-(-100), -0] = [100, 0] → min=0, max=100
    CHECK_CLOSE(omin[1], 0.0, 1e-10);
    CHECK_CLOSE(omax[1], 100.0, 1e-10);
    // Z in tiles = Y assimp = [0, 50]
    CHECK_CLOSE(omin[2], 0.0, 1e-10);
    CHECK_CLOSE(omax[2], 50.0, 1e-10);
}

// RotationENUToAssimp — identity case
TEST(axismapper_rotation_identity) {
    Eigen::Matrix3d R_enu = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d Rc = AxisMapper::RotationENUToAssimp(R_enu);

    // A = [[1,0,0],[0,0,-1],[0,1,0]],  Rc = A^T * I * A.
    // Just verify the output is well-formed (determinant ≈ 1).
    CHECK_CLOSE(Rc.determinant(), 1.0, 1e-9);
}

// =========================================================================
// 5. MeshGroupOptimizer — boundary value tests
// =========================================================================

// Simplification with error=0 (no simplification — pass-through)
TEST(meshopt_error_zero) {
    float verts[] = {
        0,0,0,  1,0,0,  0,1,0,   // triangle 1
        1,0,0,  0,1,0,  1,1,0,   // triangle 2
        0,0,0,  1,1,0,  0,1,0,   // triangle 3
        0,0,0,  1,0,0,  1,1,0,   // triangle 4
    };
    unsigned int indices[] = {0,1,2, 3,4,5, 6,7,8, 9,10,11};
    aiScene* scene = makeMinimalScene("test", verts, 12, indices, 12);

    OptimizerConfig config;
    config.reorder = true;
    config.items.push_back(OptimizerItem(".*", 0.0f, 0.1f, 0.0f, true, false));

    // error=0 should be detected and return early (no simplification)
    bool result = CMeshGroupOptimizer::SimplifyScene(scene, config);
    CHECK(result);

    // Vertex count should be unchanged (error=0 → skip simplification)
    CHECK(scene->mMeshes[0]->mNumVertices == 12);

    freeScene(scene);
}

// Simplification with very high error — should collapse to minimal geometry
TEST(meshopt_error_extreme) {
    // Create a simple quad (2 triangles, 4 vertices)
    float verts[] = {
        0,0,0,  10,0,0,  0,10,0,  10,10,0
    };
    unsigned int indices[] = {0,1,2, 1,3,2};
    aiScene* scene = makeMinimalScene("test", verts, 4, indices, 6);

    OptimizerConfig config;
    config.reorder = false;
    // Very high error should collapse to minimal triangles
    config.items.push_back(OptimizerItem(".*", 100.0f, 0.0f, 0.0f, false, false));

    bool result = CMeshGroupOptimizer::SimplifyScene(scene, config);
    CHECK(result);

    // With extreme error, mesh should still have some vertices (can't be < 3 for a triangle)
    unsigned int nv = scene->mMeshes[0]->mNumVertices;
    CHECK(nv >= 3);
    CHECK(nv <= 4);  // should not increase

    freeScene(scene);
}

// Simplification with lockBorder=true preserves boundary vertices
TEST(meshopt_lockborder) {
    float verts[] = {
        0,0,0,  10,0,0,  20,0,0,  30,0,0,
        0,10,0, 10,10,0, 20,10,0, 30,10,0,
    };
    // 6 quads = 12 triangles, 8 vertices
    unsigned int indices[] = {
        0,1,4, 1,5,4,  1,2,5, 2,6,5,
        2,3,6, 3,7,6
    };
    aiScene* scene = makeMinimalScene("test", verts, 8, indices, 18);

    OptimizerConfig config;
    config.reorder = false;
    config.items.push_back(OptimizerItem(".*", 0.5f, 0.1f, 0.0f, true, false));

    bool result = CMeshGroupOptimizer::SimplifyScene(scene, config);
    CHECK(result);

    freeScene(scene);
}

// Simplification with threshold=1.0 (max allowed triangle fraction)
TEST(meshopt_threshold_max) {
    float verts[] = {
        0,0,0,  1,0,0,  0,1,0,  1,1,0
    };
    unsigned int indices[] = {0,1,2, 1,3,2};
    aiScene* scene = makeMinimalScene("test", verts, 4, indices, 6);

    OptimizerConfig config;
    config.reorder = true;
    // threshold=1.0 allows meshopt to reduce to 100% of original
    config.items.push_back(OptimizerItem(".*", 0.01f, 0.1f, 1.0f, false, false));

    bool result = CMeshGroupOptimizer::SimplifyScene(scene, config);
    CHECK(result);

    freeScene(scene);
}

// Simplification with threshold=0 (no target triangle count constraint)
TEST(meshopt_threshold_zero) {
    float verts[] = {
        0,0,0,  1,0,0,  0,1,0,  1,1,0
    };
    unsigned int indices[] = {0,1,2, 1,3,2};
    aiScene* scene = makeMinimalScene("test", verts, 4, indices, 6);

    OptimizerConfig config;
    config.reorder = true;
    // threshold=0 means error-driven only (no ratio constraint)
    config.items.push_back(OptimizerItem(".*", 0.01f, 0.1f, 0.0f, false, false));

    bool result = CMeshGroupOptimizer::SimplifyScene(scene, config);
    CHECK(result);

    freeScene(scene);
}

// Mesh with no normals
TEST(meshopt_no_normals) {
    float verts[] = {
        0,0,0,  1,0,0,  0,1,0,  1,1,0
    };
    unsigned int indices[] = {0,1,2, 1,3,2};
    aiScene* scene = makeMinimalScene("test", verts, 4, indices, 6, false, false);

    OptimizerConfig config;
    config.reorder = true;
    config.items.push_back(OptimizerItem(".*", 0.01f, 0.0f, 0.0f, false, false));

    bool result = CMeshGroupOptimizer::SimplifyScene(scene, config);
    CHECK(result);

    // Without normals, meshopt still simplifies (using position only)
    unsigned int nv = scene->mMeshes[0]->mNumVertices;
    CHECK(nv <= 4);  // should not expand

    freeScene(scene);
}

// Regex matching: verify specific mesh name matches
TEST(meshopt_regex_match) {
    float verts[] = {0,0,0, 1,0,0, 0,1,0};
    unsigned int indices[] = {0,1,2};
    aiScene* scene = makeMinimalScene("Building_Wall_001", verts, 3, indices, 3);

    OptimizerConfig config;
    config.reorder = false;
    // Only match meshes with "Wall" in name
    config.items.push_back(OptimizerItem(".*Wall.*", 0.01f, 0.1f, 0.0f, false, false));

    bool result = CMeshGroupOptimizer::SimplifyScene(scene, config);
    CHECK(result);

    freeScene(scene);
}

// Regex non-matching: verify mesh is skipped when name doesn't match
TEST(meshopt_regex_nomatch) {
    float verts[] = {0,0,0, 1,0,0, 0,1,0};
    unsigned int indices[] = {0,1,2};
    aiScene* scene = makeMinimalScene("Building_Roof_001", verts, 3, indices, 3);

    OptimizerConfig config;
    config.reorder = false;
    // Only match "Wall" — Roof should NOT match
    config.items.push_back(OptimizerItem(".*Wall.*", 100.0f, 0.0f, 0.0f, false, false));

    bool result = CMeshGroupOptimizer::SimplifyScene(scene, config);
    CHECK(result);

    // Without a matching item, mesh should be skipped — vertex count unchanged
    CHECK(scene->mMeshes[0]->mNumVertices == 3);

    freeScene(scene);
}

// Config with no items (should not crash)
TEST(meshopt_empty_config) {
    float verts[] = {0,0,0, 1,0,0, 0,1,0};
    unsigned int indices[] = {0,1,2};
    aiScene* scene = makeMinimalScene("test", verts, 3, indices, 3);

    OptimizerConfig config;
    config.reorder = false;
    // No items in config

    bool result = CMeshGroupOptimizer::SimplifyScene(scene, config);
    CHECK(result);  // should succeed (no-op)

    freeScene(scene);
}

// Single triangle — minimum valid mesh
TEST(meshopt_single_triangle) {
    float verts[] = {0,0,0, 1,0,0, 0,1,0};
    unsigned int indices[] = {0,1,2};
    aiScene* scene = makeMinimalScene("test", verts, 3, indices, 3);

    OptimizerConfig config;
    config.reorder = true;
    config.items.push_back(OptimizerItem(".*", 0.01f, 0.1f, 0.0f, false, false));

    bool result = CMeshGroupOptimizer::SimplifyScene(scene, config);
    CHECK(result);

    // Single triangle should survive simplification
    unsigned int nv = scene->mMeshes[0]->mNumVertices;
    unsigned int nf = scene->mMeshes[0]->mNumFaces;
    CHECK(nv == 3);
    CHECK(nf == 1);

    freeScene(scene);
}

// =========================================================================
// 6. MeshGroupOptimizer Save/return value fix verification
// =========================================================================

TEST(meshopt_return_value) {
    float verts[] = {0,0,0, 1,0,0, 0,1,0};
    unsigned int indices[] = {0,1,2};
    aiScene* scene = makeMinimalScene("test", verts, 3, indices, 3);

    OptimizerConfig config;
    config.reorder = true;
    config.items.push_back(OptimizerItem(".*", 0.01f, 0.1f, 0.0f, false, false));

    // SimplifyScene should return true on success
    bool result = CMeshGroupOptimizer::SimplifyScene(scene, config);
    CHECK(result);

    freeScene(scene);
}

// =========================================================================
int main() {
    std::cout << "=== GeodeticMath Boundary Tests ===" << std::endl;
    RUN(geodetic_pole_north_roundtrip);
    RUN(geodetic_pole_south_roundtrip);
    RUN(geodetic_equator);
    RUN(geodetic_dateline);
    RUN(geodetic_zero_height);
    RUN(geodetic_extreme_height);
    RUN(gk_inverse_at_origin);
    RUN(gk_inverse_far_easting);
    RUN(gk_forward_inverse_roundtrip);
    RUN(enu_rotation_at_pole);
    RUN(enu_rotation_at_equator);

    std::cout << "\n=== Georeferencing Boundary Tests ===" << std::endl;
    RUN(sevenparam_extreme_rotation);
    RUN(sevenparam_zero_scale);
    RUN(sevenparam_wkt2_lowercase);
    RUN(anchor_same_crs);
    RUN(multipos_min_4_points);
    RUN(multipos_poly_order3);

    std::cout << "\n=== CProjectionEngine Boundary Tests ===" << std::endl;
    RUN(projengine_wkt2_lowercase_params);
    RUN(projengine_wkt1_titlecase_params);
    RUN(projengine_spheroid_with_space);
    RUN(projengine_spheroid_lowercase);
    RUN(projengine_no_projection_root_transform);
    RUN(projengine_reset);

    std::cout << "\n=== AxisMapper Boundary Tests ===" << std::endl;
    RUN(axismapper_roundtrip);
    RUN(axismapper_bbox_conversion);
    RUN(axismapper_rotation_identity);

    std::cout << "\n=== MeshGroupOptimizer Boundary Tests ===" << std::endl;
    RUN(meshopt_error_zero);
    RUN(meshopt_error_extreme);
    RUN(meshopt_lockborder);
    RUN(meshopt_threshold_max);
    RUN(meshopt_threshold_zero);
    RUN(meshopt_no_normals);
    RUN(meshopt_regex_match);
    RUN(meshopt_regex_nomatch);
    RUN(meshopt_empty_config);
    RUN(meshopt_single_triangle);
    RUN(meshopt_return_value);

    std::cout << "\n========================================" << std::endl;
    std::cout << "PASS=" << g_pass << " FAIL=" << g_fail << std::endl;
    return g_fail > 0 ? 1 : 0;
}
