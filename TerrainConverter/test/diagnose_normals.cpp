// diagnose_normals — standalone diagnostic for terrain normal encoding
//
// NOTE: This test uses a simplified local ENU normal computation (UVH +
// scaleX/scaleY) that approximates the production pipeline's ECEF approach.
// The production code in QuantizedMeshEncoder::ComputeVertexNormals converts
// UVH→ECEF, computes normals in ECEF, and oct-encodes them directly (CesiumJS
// treats quantized-mesh normals as ECEF model coordinates — there is no ENU
// rotation). This test validates the OctEncode/OctDecode round-trip and basic
// normal direction properties; it does not exercise the ECEF path.
//
// Creates a synthetic heightmap to verify:
//   1. Flat terrain → normals point (0,0,1)
//   2. Sloped terrain → normals point in correct direction
//   3. Winding order yields normals pointing Up (not Down)
//   4. Oct encode/decode round-trip accuracy
//
// Build: g++ -std=c++17 -I.. -I../../MeshProjectionErrorCorrector \
//        diagnose_normals.cpp -o diagnose_normals && ./diagnose_normals

#include <cmath>
#include <cstdio>
#include <cassert>
#include <vector>
#include <cstdint>
#include <algorithm>

static const float MAX_UV = 32767.0f;
static int g_pass = 0, g_fail = 0;

#define CHECK(cond, msg) do { \
    if (!(cond)) { printf("  FAIL: %s\n", msg); ++g_fail; } \
    else { ++g_pass; } \
} while(0)

// OctEncode — exact copy from QuantizedMeshEncoder.cpp
static void OctEncode(float nx, float ny, float nz, uint8_t out[2])
{
    float l1norm = std::abs(nx) + std::abs(ny) + std::abs(nz);
    if (l1norm > 1e-10f) { nx /= l1norm; ny /= l1norm; nz /= l1norm; }
    float u, v;
    if (nz >= 0.0f) { u = nx; v = ny; }
    else {
        u = (1.0f - std::abs(ny)) * (nx >= 0 ? 1.0f : -1.0f);
        v = (1.0f - std::abs(nx)) * (ny >= 0 ? 1.0f : -1.0f);
    }
    out[0] = static_cast<uint8_t>(std::round((u * 0.5f + 0.5f) * 255.0f));
    out[1] = static_cast<uint8_t>(std::round((v * 0.5f + 0.5f) * 255.0f));
}

// OctDecode — inverse of OctEncode
static void OctDecode(uint8_t a, uint8_t b, float& nx, float& ny, float& nz)
{
    float u = (static_cast<float>(a) / 255.0f) * 2.0f - 1.0f;
    float v = (static_cast<float>(b) / 255.0f) * 2.0f - 1.0f;
    float au = std::abs(u), av = std::abs(v);
    if (au + av <= 1.0f) {
        nz = 1.0f - au - av;
        nx = u; ny = v;
    } else {
        // Lower hemisphere reflection
        nx = (1.0f - av) * (u >= 0 ? 1.0f : -1.0f);
        ny = (1.0f - au) * (v >= 0 ? 1.0f : -1.0f);
        nz = -(1.0f - std::abs(nx) - std::abs(ny));
    }
    // Normalize
    float len = std::sqrt(nx*nx + ny*ny + nz*nz);
    if (len > 1e-10f) { nx /= len; ny /= len; nz /= len; }
}

// ComputeVertexNormals — matches QuantizedMeshEncoder::ComputeVertexNormals
// (equal-weight voting: normalise each face normal before accumulating)
static void ComputeVertexNormals(const std::vector<float>& vertices,
                                  const std::vector<unsigned>& indices,
                                  double widthDeg, double heightDeg,
                                  double midLatDeg,
                                  std::vector<float>& outNormals)
{
    const size_t n = vertices.size() / 3;
    outNormals.assign(n * 3, 0.0f);

    const double midLatRad = midLatDeg * M_PI / 180.0;
    // Earth::METERS_PER_DEGREE_AT_EQUATOR = 111320.0
    const double metersPerDegLat = 111320.0;
    const double metersPerDegLon = 111320.0 * std::cos(midLatRad);
    const float scaleX = static_cast<float>((widthDeg  / MAX_UV) * metersPerDegLon);
    const float scaleY = static_cast<float>((heightDeg / MAX_UV) * metersPerDegLat);

    for (size_t t = 0; t + 2 < indices.size(); t += 3) {
        unsigned i0 = indices[t], i1 = indices[t+1], i2 = indices[t+2];
        float ax = (vertices[i1*3+0] - vertices[i0*3+0]) * scaleX;
        float ay = (vertices[i1*3+1] - vertices[i0*3+1]) * scaleY;
        float az =  vertices[i1*3+2] - vertices[i0*3+2];
        float bx = (vertices[i2*3+0] - vertices[i0*3+0]) * scaleX;
        float by = (vertices[i2*3+1] - vertices[i0*3+1]) * scaleY;
        float bz =  vertices[i2*3+2] - vertices[i0*3+2];
        float nx = ay*bz - az*by;
        float ny = az*bx - ax*bz;
        float nz = ax*by - ay*bx;
        // Normalise face normal then accumulate — equal-weight voting
        float flen = std::sqrt(nx*nx + ny*ny + nz*nz);
        if (flen > 1e-10f) {
            float inv = 1.0f / flen;
            outNormals[i0*3+0] += nx * inv; outNormals[i0*3+1] += ny * inv; outNormals[i0*3+2] += nz * inv;
            outNormals[i1*3+0] += nx * inv; outNormals[i1*3+1] += ny * inv; outNormals[i1*3+2] += nz * inv;
            outNormals[i2*3+0] += nx * inv; outNormals[i2*3+1] += ny * inv; outNormals[i2*3+2] += nz * inv;
        }
    }
    for (size_t i = 0; i < n; ++i) {
        float nx = outNormals[i*3+0], ny = outNormals[i*3+1], nz = outNormals[i*3+2];
        float len = std::sqrt(nx*nx + ny*ny + nz*nz);
        if (len > 1e-10f) { outNormals[i*3+0] = nx/len; outNormals[i*3+1] = ny/len; outNormals[i*3+2] = nz/len; }
        else { outNormals[i*3+0] = 0; outNormals[i*3+1] = 0; outNormals[i*3+2] = 1; }
    }
}

// Build a simple test tile with a known slope
static void BuildTestTile(std::vector<float>& verts, std::vector<unsigned>& indices,
                          int W, int H, float elevationScale)
{
    verts.resize(W * H * 3);
    for (int r = 0; r < H; ++r) {
        for (int c = 0; c < W; ++c) {
            size_t i = (r * W + c) * 3;
            verts[i+0] = static_cast<float>(c) / (W-1) * MAX_UV;
            verts[i+1] = static_cast<float>(H-1-r) / (H-1) * MAX_UV;
            // Slope: height increases eastward
            verts[i+2] = c * elevationScale;
        }
    }
    indices.clear();
    for (int r = 0; r < H-1; ++r) {
        for (int c = 0; c < W-1; ++c) {
            unsigned a = r*W+c, b = a+1, d = a+W, e = d+1;
            // CCW in uv-space: a(NW), d(SW), b(NE)
            indices.push_back(a); indices.push_back(d); indices.push_back(b);
            indices.push_back(b); indices.push_back(d); indices.push_back(e);
        }
    }
}

int main()
{
    printf("=== Terrain Normal Diagnostic ===\n\n");

    // Test 1: Flat terrain → all normals point (0,0,1)
    printf("[Test 1] Flat terrain (elevation=0)\n");
    {
        std::vector<float> verts;
        std::vector<unsigned> indices;
        BuildTestTile(verts, indices, 65, 65, 0.0f);
        std::vector<float> normals;
        ComputeVertexNormals(verts, indices, 0.1, 0.1, 30.0, normals);
        int wrong = 0;
        for (size_t i = 0; i < normals.size()/3; ++i) {
            float nz = normals[i*3+2];
            if (nz < 0.99f) ++wrong;
        }
        CHECK(wrong == 0, "flat terrain normals should all be (0,0,1)");
        if (wrong > 0) printf("    %d/%zu normals have nz < 0.99\n", wrong, normals.size()/3);
    }

    // Test 2: Eastward slope → normals have negative nx component
    printf("[Test 2] Eastward slope (height increases with u)\n");
    {
        std::vector<float> verts;
        std::vector<unsigned> indices;
        BuildTestTile(verts, indices, 65, 65, 10.0f);
        std::vector<float> normals;
        ComputeVertexNormals(verts, indices, 0.1, 0.1, 30.0, normals);
        // All normals should have nx < 0 (pointing west, away from the uphill slope)
        int wrong = 0;
        float avg_nx = 0;
        for (size_t i = 0; i < normals.size()/3; ++i) {
            avg_nx += normals[i*3+0];
            if (normals[i*3+2] < 0) ++wrong; // should never point down
        }
        avg_nx /= (normals.size()/3);
        CHECK(wrong == 0, "all normals should point Up (nz >= 0)");
        CHECK(avg_nx < -0.001f, "eastward slope normals should have negative nx (point west)");
    }

    // Test 3: Northward slope → normals have negative ny component
    printf("[Test 3] Northward slope (height increases with v/North)\n");
    {
        std::vector<float> verts;
        std::vector<unsigned> indices;
        int W = 65, H = 65;
        verts.resize(W*H*3);
        for (int r = 0; r < H; ++r) {
            for (int c = 0; c < W; ++c) {
                size_t i = (r * W + c) * 3;
                verts[i+0] = static_cast<float>(c) / (W-1) * MAX_UV;
                verts[i+1] = static_cast<float>(H-1-r) / (H-1) * MAX_UV;
                // Slope: height increases northward (smaller r = more north = higher)
                verts[i+2] = (H-1-r) * 10.0f;
            }
        }
        indices.clear();
        for (int r = 0; r < H-1; ++r)
            for (int c = 0; c < W-1; ++c) {
                unsigned a = r*W+c, b = a+1, d = a+W, e = d+1;
                indices.push_back(a); indices.push_back(d); indices.push_back(b);
                indices.push_back(b); indices.push_back(d); indices.push_back(e);
            }
        std::vector<float> normals;
        ComputeVertexNormals(verts, indices, 0.1, 0.1, 30.0, normals);
        int wrong = 0;
        float avg_ny = 0;
        for (size_t i = 0; i < normals.size()/3; ++i) {
            avg_ny += normals[i*3+1];
            if (normals[i*3+2] < 0) ++wrong;
        }
        avg_ny /= (normals.size()/3);
        CHECK(wrong == 0, "all normals should point Up");
        CHECK(avg_ny < -0.001f, "northward slope normals should have negative ny (point south/downhill)");
    }

    // Test 4: OctEncode round-trip
    printf("[Test 4] OctEncode/OctDecode round-trip\n");
    {
        float test_normals[][3] = {
            {0,0,1}, {0,1,0}, {1,0,0},
            {0.577f,0.577f,0.577f}, {0,0,-1},
            {-0.707f,0,0.707f}, {0,-0.707f,-0.707f}
        };
        int fail_count = 0;
        for (auto& n : test_normals) {
            uint8_t enc[2];
            OctEncode(n[0], n[1], n[2], enc);
            float dn[3];
            OctDecode(enc[0], enc[1], dn[0], dn[1], dn[2]);
            float dot = n[0]*dn[0] + n[1]*dn[1] + n[2]*dn[2];
            if (dot < 0.999f) {
                printf("    (%.3f,%.3f,%.3f) -> [%d,%d] -> (%.3f,%.3f,%.3f) dot=%.4f\n",
                       n[0],n[1],n[2], enc[0],enc[1], dn[0],dn[1],dn[2], dot);
                ++fail_count;
            }
        }
        CHECK(fail_count == 0, "oct round-trip accuracy");
    }

    // Test 5: Verify that CW triangles produce inverted normals
    printf("[Test 5] CW vs CCW winding effect on normals\n");
    {
        std::vector<float> verts;
        std::vector<unsigned> indices;
        BuildTestTile(verts, indices, 3, 3, 100.0f); // steep slope for clear normal
        // Compute normals with CCW winding (as built)
        std::vector<float> norms_ccw;
        ComputeVertexNormals(verts, indices, 0.1, 0.1, 30.0, norms_ccw);
        float nz_ccw = 0;
        for (size_t i = 0; i < norms_ccw.size()/3; ++i) nz_ccw += norms_ccw[i*3+2];
        nz_ccw /= (norms_ccw.size()/3);

        // Flip winding by swapping every triangle
        std::vector<unsigned> idx_cw = indices;
        for (size_t t = 0; t + 2 < idx_cw.size(); t += 3)
            std::swap(idx_cw[t+1], idx_cw[t+2]);

        std::vector<float> norms_cw;
        ComputeVertexNormals(verts, idx_cw, 0.1, 0.1, 30.0, norms_cw);
        float nz_cw = 0;
        for (size_t i = 0; i < norms_cw.size()/3; ++i) nz_cw += norms_cw[i*3+2];
        nz_cw /= (norms_cw.size()/3);

        CHECK(nz_ccw > 0.0f, "CCW winding → normals point Up");
        CHECK(nz_cw < 0.0f, "CW winding → normals point Down");
        printf("    CCW avg nz = %.4f, CW avg nz = %.4f\n", nz_ccw, nz_cw);
    }

    // Test 6: Vector (1,0,0) [East], (0,1,0) [North] → cross = (0,0,1) [Up]
    printf("[Test 6] Cross product coordinate system consistency\n");
    {
        // Simplified right-hand rule check
        float a[3] = {1,0,0}; // East
        float b[3] = {0,1,0}; // North
        float n[3] = { a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0] };
        CHECK(n[0] == 0 && n[1] == 0 && n[2] == 1, "East × North = Up in right-handed system");
    }

    printf("\n=== RESULTS: PASS=%d FAIL=%d ===\n", g_pass, g_fail);
    return g_fail > 0 ? 1 : 0;
}
