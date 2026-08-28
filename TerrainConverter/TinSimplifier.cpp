#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include <cstdio>
#include <iostream>

#include "TinSimplifier.h"
#include "../MeshGroupOptimizer/meshoptimizer/meshoptimizer.h"
#include "../MeshProjectionErrorCorrector/CProjectionEngine.h"
#include "../MeshProjectionErrorCorrector/GeodeticMath.h"
#include "../MeshProjectionErrorCorrector/Constants.h"

TinMesh TinSimplifier::Simplify(const HeightmapGrid& localGrid,
                                 CProjectionEngine& proj,
                                 const SimplifyOptions& simplify)
{
    TinMesh result;
    const int W = localGrid.width;
    const int H = localGrid.height;
    if (W < 2 || H < 2) return result;

    const size_t vertCount = static_cast<size_t>(W) * H;

    // — 1. Check for all-noData tile (early return) —
    // Build a stable 5×5 flat grid in UVH space directly — no ECEF conversion
    // needed since there is no valid elevation data.
    bool allNoData = true;
    for (int r = 0; r < H && allNoData; ++r)
        for (int c = 0; c < W; ++c)
            if (localGrid.IsValid(c, r)) { allNoData = false; break; }

    if (allNoData)
    {
        // Entire tile is noData — return empty mesh so ProcessTile skips
        // writing this tile.  CesiumJS will use the parent tile or the
        // default ellipsoid surface for this area.
        std::cerr << "[TinSimplifier] tile entirely noData — skipping"
                  << std::endl;
        return result;  // vertices.empty() == true, ProcessTile skips
    }

    // — 2. Compute height range —
    float minH = 1e30f, maxH = -1e30f;
    for (int r = 0; r < H; ++r)
        for (int c = 0; c < W; ++c)
            if (localGrid.IsValid(c, r))
            {
                float h = localGrid.HeightAt(c, r);
                if (h < minH) minH = h;
                if (h > maxH) maxH = h;
            }
    float hRange = maxH - minH;
    if (hRange < 1e-6f) hRange = 1.0f;

    // — 3. Compute tile center in ECEF —
    // Tile-local ECEF = global ECEF − tile center, so that positions fit in
    // float32 with sub-millimetre precision while preserving true 3D metric
    // distances and normal directions.
    double midEasting  = (localGrid.minEasting  + localGrid.maxEasting)  * 0.5;
    double midNorthing = (localGrid.minNorthing + localGrid.maxNorthing) * 0.5;
    double midHeight   = (minH + maxH) * 0.5;

    double midLat, midLon;  // radians
    if (localGrid.isGeographic)
    {
        midLat = midNorthing * Angle::DEG_TO_RAD;
        midLon = midEasting  * Angle::DEG_TO_RAD;
    }
    else if (proj.HasProjection())
    {
        proj.ProjectedToGeographic(midEasting, midNorthing, midLat, midLon);
    }
    else
    {
        // No projection and not geographic — fallback to identity.
        // Should not occur for valid terrain data.
        midLat = 0.0;
        midLon = 0.0;
    }

    double cx, cy, cz;
    proj.GeographicToECEF(midLat, midLon, midHeight, cx, cy, cz);

    // — 4. Build double-layer positions —
    // positionsECEF: tile-local ECEF (meters), for meshopt simplification + normals
    // positionsUV:   normalised [0,1] UV + metric height, for final output encoding
    // isNoData:      marks noData vertices for output height substitution
    std::vector<float> positionsECEF(vertCount * 3);
    std::vector<float> positionsUV(vertCount * 3);
    std::vector<bool>  isNoData(vertCount, false);

    for (int r = 0; r < H; ++r)
    {
        for (int c = 0; c < W; ++c)
        {
            size_t idx = (static_cast<size_t>(r) * W + c) * 3;
            double easting  = localGrid.EastingAt(c);
            double northing = localGrid.NorthingAt(r);

            float h;
            bool valid = localGrid.IsValid(c, r);
            if (valid)
                h = localGrid.HeightAt(c, r);
            else
            {
                isNoData[idx / 3] = true;
                h = 0.0f;  // noData at 0 m abs — matches noDataFill in ProcessTile
            }

            double gx, gy, gz;
            if (localGrid.isGeographic)
            {
                proj.GeographicToECEF(northing * Angle::DEG_TO_RAD,
                                       easting  * Angle::DEG_TO_RAD,
                                       h, gx, gy, gz);
            }
            else
            {
                proj.ProjectedToECEF(easting, northing, h, gx, gy, gz);
            }

            positionsECEF[idx + 0] = static_cast<float>(gx - cx);
            positionsECEF[idx + 1] = static_cast<float>(gy - cy);
            positionsECEF[idx + 2] = static_cast<float>(gz - cz);

            positionsUV[idx + 0] = static_cast<float>(c) / static_cast<float>(W - 1);
            positionsUV[idx + 1] = static_cast<float>(H - 1 - r) / static_cast<float>(H - 1);
            positionsUV[idx + 2] = h;
        }
    }

    // — 5. Build triangle indices — emit ALL quads (noData participates) —
    // Grid: a=(c,r)→NW, b=(c+1,r)→NE, d=(c,r+1)→SW, e=(c+1,r+1)→SE
    std::vector<unsigned> indices;
    indices.reserve(static_cast<size_t>(W - 1) * (H - 1) * 6);
    for (int r = 0; r < H - 1; ++r)
    {
        for (int c = 0; c < W - 1; ++c)
        {
            unsigned a = static_cast<unsigned>(r * W + c);
            unsigned b = a + 1;
            unsigned d = a + W;
            unsigned e = d + 1;
            indices.push_back(a); indices.push_back(d); indices.push_back(b);
            indices.push_back(b); indices.push_back(d); indices.push_back(e);
        }
    }

    // — 6. Lock border vertices for tile stitching —
    std::vector<unsigned char> vertexLock(vertCount, 0);
    if (simplify.lockBorder)
    {
        for (int c = 0; c < W; ++c)
        {
            vertexLock[c] = 1;                     // north edge (row 0)
            vertexLock[(H-1)*W + c] = 1;           // south edge (row H-1)
        }
        for (int r = 1; r < H-1; ++r)
        {
            vertexLock[r*W] = 1;                   // west edge (col 0)
            vertexLock[r*W + W-1] = 1;             // east edge (col W-1)
        }
    }

    // — 7. Compute per-vertex normals in ECEF space (equal-weight voting) —
    // Edge vectors are already in real meters — cross product gives the
    // physically correct normal directly. No scale-factor correction needed.
    //
    // Normals are used as meshopt simplification attributes: meshopt penalises
    // collapsing vertices whose normals differ, preserving terrain features.
    std::vector<float> normals;
    if (simplify.normalWeight > 0.0f && indices.size() >= 6)
    {
        normals.assign(vertCount * 3, 0.0f);
        for (size_t t = 0; t + 2 < indices.size(); t += 3)
        {
            unsigned i0 = indices[t], i1 = indices[t+1], i2 = indices[t+2];
            float ax = positionsECEF[i1*3+0] - positionsECEF[i0*3+0];
            float ay = positionsECEF[i1*3+1] - positionsECEF[i0*3+1];
            float az = positionsECEF[i1*3+2] - positionsECEF[i0*3+2];
            float bx = positionsECEF[i2*3+0] - positionsECEF[i0*3+0];
            float by = positionsECEF[i2*3+1] - positionsECEF[i0*3+1];
            float bz = positionsECEF[i2*3+2] - positionsECEF[i0*3+2];

            float nx = ay*bz - az*by;
            float ny = az*bx - ax*bz;
            float nz = ax*by - ay*bx;

            // Normalise face normal, then accumulate — equal-weight voting.
            float flen = std::sqrt(nx*nx + ny*ny + nz*nz);
            if (flen > 1e-10f) {
                float inv = 1.0f / flen;
                normals[i0*3+0] += nx * inv; normals[i0*3+1] += ny * inv; normals[i0*3+2] += nz * inv;
                normals[i1*3+0] += nx * inv; normals[i1*3+1] += ny * inv; normals[i1*3+2] += nz * inv;
                normals[i2*3+0] += nx * inv; normals[i2*3+1] += ny * inv; normals[i2*3+2] += nz * inv;
            }
        }
        for (size_t i = 0; i < vertCount; ++i)
        {
            float nx = normals[i*3+0], ny = normals[i*3+1], nz = normals[i*3+2];
            float len = std::sqrt(nx*nx + ny*ny + nz*nz);
            if (len > 1e-10f) { normals[i*3+0] = nx/len; normals[i*3+1] = ny/len; normals[i*3+2] = nz/len; }
            else { normals[i*3+0] = 0.0f; normals[i*3+1] = 0.0f; normals[i*3+2] = 1.0f; }
        }
    }

    // — 8. Simplify with error-driven meshopt in ECEF space —
    // `error` is a normalized relative error (0..1), matching meshopt's
    // target_error contract: meshopt internally rescales positions to a unit
    // cube and reads target_error as a fraction of the mesh extent (e.g.
    // 0.001 = 0.1% deformation).  Converting it to absolute meters here
    // (ratio × tileDiagonal) breaks that contract — the meter value is read
    // as a relative fraction (e.g. 1.7 → 170%), collapsing every tile down
    // to its locked border vertices and discarding all terrain detail.
    double ratio = simplify.error;
    if (ratio <= 0.0) ratio = 0.001;
    if (ratio > 1.0)  ratio = 1.0;

    const float errorF = static_cast<float>(ratio);
    const unsigned int kOptions = 0;

    size_t targetIndexCount = 0;
    if (simplify.threshold > 0.0f && simplify.threshold < 1.0f)
        targetIndexCount = static_cast<size_t>(simplify.threshold * indices.size());

    std::vector<unsigned> simpIndices(indices.size());
    size_t simpIndexCount;
    if (!normals.empty())
    {
        float nweights[] = { simplify.normalWeight, simplify.normalWeight, simplify.normalWeight };
        simpIndexCount = meshopt_simplifyWithAttributes(
            simpIndices.data(), indices.data(), indices.size(),
            positionsECEF.data(), vertCount, sizeof(float)*3,
            normals.data(), sizeof(float)*3, nweights, 3,
            vertexLock.data(),
            targetIndexCount, errorF,
            kOptions, nullptr);
    }
    else
    {
        simpIndexCount = meshopt_simplifyWithAttributes(
            simpIndices.data(), indices.data(), indices.size(),
            positionsECEF.data(), vertCount, sizeof(float)*3,
            nullptr, 0, nullptr, 0,
            vertexLock.data(),
            targetIndexCount, errorF,
            kOptions, nullptr);
    }

    if (simpIndexCount == 0)
    {
        std::cerr << "[TinSimplifier] simplification returned 0, using original" << std::endl;
        simpIndices = indices;
        simpIndexCount = indices.size();
    }
    else
    {
        simpIndices.resize(simpIndexCount);
    }

    // — 9. Filter degenerate triangles (real 3D area in ECEF m²) —
    // `area2` is |cross|² = (2·area)².  A threshold of 1e-4 m⁴ ≈ 50 cm²
    // triangle removes the sliver triangles that form at the poles, where
    // every longitude at lat=±90 collapses to (nearly) the same ECEF point
    // (cos(90°) ≈ 6.1e-17 in double → the pole vertices end up ~1e-10 m apart,
    // so their cross product is tiny but not exactly zero).  These slivers have
    // area² up to ~1.3e-6 m⁴; real terrain triangles are ≥ ~200 cm²
    // (area² ≥ 1.6e-3 m⁴), so 1e-4 sits safely between the two.
    {
        std::vector<unsigned> filtered;
        filtered.reserve(simpIndexCount);
        for (size_t i = 0; i + 2 < simpIndexCount; i += 3)
        {
            unsigned i0 = simpIndices[i];
            unsigned i1 = simpIndices[i + 1];
            unsigned i2 = simpIndices[i + 2];
            float ax = positionsECEF[i1 * 3 + 0] - positionsECEF[i0 * 3 + 0];
            float ay = positionsECEF[i1 * 3 + 1] - positionsECEF[i0 * 3 + 1];
            float az = positionsECEF[i1 * 3 + 2] - positionsECEF[i0 * 3 + 2];
            float bx = positionsECEF[i2 * 3 + 0] - positionsECEF[i0 * 3 + 0];
            float by = positionsECEF[i2 * 3 + 1] - positionsECEF[i0 * 3 + 1];
            float bz = positionsECEF[i2 * 3 + 2] - positionsECEF[i0 * 3 + 2];
            float nx = ay * bz - az * by;
            float ny = az * bx - ax * bz;
            float nz = ax * by - ay * bx;
            float area2 = nx * nx + ny * ny + nz * nz;
            if (area2 > 1e-4f)
            {
                filtered.push_back(i0);
                filtered.push_back(i1);
                filtered.push_back(i2);
            }
        }
        if (!filtered.empty())
        {
            simpIndices = std::move(filtered);
            simpIndexCount = simpIndices.size();
        }
    }

    // — 10. Fix winding order —
    // meshopt_simplifyWithAttributes can flip triangles during edge collapses.
    // "up" must be each triangle's OWN geocentric radial (at its centroid), not
    // the tile centre's radial.  For hemisphere-spanning placeholder tiles the
    // local up direction rotates by up to 180° across the tile, so a single
    // tile-centre up would wrongly flip triangles on the far side and fold the
    // surface.  dot(face_normal, local_up) < 0 → normal points inward → swap.
    {
        int flipped = 0, nearVertical = 0;
        for (size_t t = 0; t + 2 < simpIndexCount; t += 3)
        {
            unsigned i0 = simpIndices[t];
            unsigned i1 = simpIndices[t + 1];
            unsigned i2 = simpIndices[t + 2];
            float ax = positionsECEF[i1 * 3 + 0] - positionsECEF[i0 * 3 + 0];
            float ay = positionsECEF[i1 * 3 + 1] - positionsECEF[i0 * 3 + 1];
            float az = positionsECEF[i1 * 3 + 2] - positionsECEF[i0 * 3 + 2];
            float bx = positionsECEF[i2 * 3 + 0] - positionsECEF[i0 * 3 + 0];
            float by = positionsECEF[i2 * 3 + 1] - positionsECEF[i0 * 3 + 1];
            float bz = positionsECEF[i2 * 3 + 2] - positionsECEF[i0 * 3 + 2];
            float nx = ay*bz - az*by;
            float ny = az*bx - ax*bz;
            float nz = ax*by - ay*bx;

            // Local up = geocentric radial at the triangle centroid (global ECEF
            // = tile-local ECEF + tile centre).
            float cxg = (positionsECEF[i0*3+0] + positionsECEF[i1*3+0] + positionsECEF[i2*3+0]) * (1.0f/3.0f) + static_cast<float>(cx);
            float cyg = (positionsECEF[i0*3+1] + positionsECEF[i1*3+1] + positionsECEF[i2*3+1]) * (1.0f/3.0f) + static_cast<float>(cy);
            float czg = (positionsECEF[i0*3+2] + positionsECEF[i1*3+2] + positionsECEF[i2*3+2]) * (1.0f/3.0f) + static_cast<float>(cz);
            float upLen = std::sqrt(cxg*cxg + cyg*cyg + czg*czg);
            float upX = cxg / upLen, upY = cyg / upLen, upZ = czg / upLen;

            float dot = nx*upX + ny*upY + nz*upZ;
            if (dot < 0.0f)
            {
                std::swap(simpIndices[t + 1], simpIndices[t + 2]);
                ++flipped;
            }
            else
            {
                // Near-vertical face: skip winding processing for faces
                // within ~0.11° of horizontal — same 2e-3 threshold as
                // ComputeVertexNormals.  These faces contribute nothing
                // to the upward component and have unreliable lateral
                // direction due to float32 position quantisation.
                float flen = std::sqrt(nx*nx + ny*ny + nz*nz);
                if (flen > 1e-10f && std::abs(dot) / flen < 2e-3f)
                    ++nearVertical;
            }
        }
        // Post-fix verification: count triangles still pointing inward.
        {
            int postBad = 0;
            for (size_t t = 0; t + 2 < simpIndexCount; t += 3)
            {
                unsigned i0 = simpIndices[t], i1 = simpIndices[t+1], i2 = simpIndices[t+2];
                float ax = positionsECEF[i1*3+0] - positionsECEF[i0*3+0];
                float ay = positionsECEF[i1*3+1] - positionsECEF[i0*3+1];
                float az = positionsECEF[i1*3+2] - positionsECEF[i0*3+2];
                float bx = positionsECEF[i2*3+0] - positionsECEF[i0*3+0];
                float by = positionsECEF[i2*3+1] - positionsECEF[i0*3+1];
                float bz = positionsECEF[i2*3+2] - positionsECEF[i0*3+2];
                float nx = ay*bz - az*by;
                float ny = az*bx - ax*bz;
                float nz = ax*by - ay*bx;
                float cxg = (positionsECEF[i0*3+0] + positionsECEF[i1*3+0] + positionsECEF[i2*3+0]) * (1.0f/3.0f) + static_cast<float>(cx);
                float cyg = (positionsECEF[i0*3+1] + positionsECEF[i1*3+1] + positionsECEF[i2*3+1]) * (1.0f/3.0f) + static_cast<float>(cy);
                float czg = (positionsECEF[i0*3+2] + positionsECEF[i1*3+2] + positionsECEF[i2*3+2]) * (1.0f/3.0f) + static_cast<float>(cz);
                float ul = std::sqrt(cxg*cxg + cyg*cyg + czg*czg);
                float d = nx*(cxg/ul) + ny*(cyg/ul) + nz*(czg/ul);
                if (d < 0.0f) ++postBad;
            }
        }
    }

    // — 11. Remap to compact output —
    // Remap is based on ECEF positions (metric spatial coincidence).
    // Output uses positionsUV to produce the (u_quant, v_quant, h_meters)
    // TinMesh format expected by QuantizedMeshEncoder.
    const float MAX_UV = TileConstants::MAX_QUANTIZED_UV;
    std::vector<unsigned> remap(vertCount);
    size_t newVertCount = meshopt_generateVertexRemap(
        remap.data(),
        simpIndices.data(),
        simpIndexCount,
        positionsECEF.data(),
        vertCount,
        sizeof(float) * 3);

    result.vertices.resize(newVertCount * 3);
    std::vector<bool> outNoData(newVertCount, false);
    for (size_t i = 0; i < vertCount; ++i)
    {
        unsigned ni = remap[i];
        if (ni < newVertCount)
        {
            result.vertices[ni * 3 + 0] = positionsUV[i * 3 + 0] * MAX_UV;
            result.vertices[ni * 3 + 1] = positionsUV[i * 3 + 1] * MAX_UV;
            // noData vertices: use noDataFill for output height so adjacent
            // tiles stitch correctly at a common fill elevation
            result.vertices[ni * 3 + 2] = isNoData[i]
                ? localGrid.noDataFill
                : positionsUV[i * 3 + 2];
            if (isNoData[i]) outNoData[ni] = true;
        }
    }
    result.indices.resize(simpIndexCount);
    for (size_t i = 0; i < simpIndexCount; ++i)
    {
        result.indices[i] = remap[simpIndices[i]];
    }

    localGrid.ComputeMinMax(result.minHeight, result.maxHeight);

    return result;
}

void TinSimplifier::QuantizeToLocal(const HeightmapGrid& /*localGrid*/,
                                     CProjectionEngine& /*proj*/,
                                     const TinMesh& tin,
                                     TinMesh& outQuantized)
{
    // tin.vertices are already in quantized [0, 32767] format (u, v, h_meters)
    // from Simplify(). Simple copy — no conversion needed.
    const size_t n = tin.vertices.size() / 3;
    outQuantized.vertices.resize(n * 3);
    outQuantized.indices = tin.indices;
    outQuantized.minHeight = tin.minHeight;
    outQuantized.maxHeight = tin.maxHeight;

    for (size_t i = 0; i < n; ++i)
    {
        float u = tin.vertices[i * 3 + 0];
        float v = tin.vertices[i * 3 + 1];
        float h = tin.vertices[i * 3 + 2];
        // Clamp to valid range (should already be in range)
        if (u < 0.0f) u = 0.0f; else if (u > TileConstants::MAX_QUANTIZED_UV) u = TileConstants::MAX_QUANTIZED_UV;
        if (v < 0.0f) v = 0.0f; else if (v > TileConstants::MAX_QUANTIZED_UV) v = TileConstants::MAX_QUANTIZED_UV;
        outQuantized.vertices[i * 3 + 0] = static_cast<float>(std::round(u));
        outQuantized.vertices[i * 3 + 1] = static_cast<float>(std::round(v));
        outQuantized.vertices[i * 3 + 2] = h;
    }
}
