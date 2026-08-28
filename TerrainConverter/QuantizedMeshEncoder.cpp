#include "QuantizedMeshEncoder.h"
#include "../MeshProjectionErrorCorrector/CProjectionEngine.h"
#include "../MeshProjectionErrorCorrector/GeodeticMath.h"
#include "../MeshProjectionErrorCorrector/Constants.h"

#include <cmath>
#include <algorithm>
#include <cstring>
#include <cstdio>
#include <iostream>
#include <iomanip>
#include <array>

namespace {

// Vertex struct for sorting (u, v, height) — Cesium requires monotonic u/v
struct SortVertex
{
    uint16_t u;
    uint16_t v;
    float    h;
    unsigned originalIndex;
};

// Edge vertex collection: classify vertices by which edge(s) they lie on.
// V axis convention (Cesium quantized-mesh spec): v=0=south, v=max=north.
// Returns a bit mask so corner vertices (on two edges) appear in both lists:
//   bit 0 (1) = West  (u=0)
//   bit 1 (2) = South (v=0)
//   bit 2 (4) = East  (u=max)
//   bit 3 (8) = North (v=max)
// Cesium's skirt generation walks each edge independently and expects corner
// vertices to be present in both adjacent edges to avoid gaps at tile corners.
int ClassifyEdge(uint16_t u, uint16_t v, uint16_t maxUV)
{
    int mask = 0;
    if (u == 0)      mask |= 1;  // West
    if (v == 0)      mask |= 2;  // South
    if (u == maxUV)  mask |= 4;  // East
    if (v == maxUV)  mask |= 8;  // North
    return mask;
}

} // namespace

bool QuantizedMeshEncoder::Encode(const TinMesh& mesh,
                                   const TerrainTile& tile,
                                   CProjectionEngine& proj,
                                   const QuantizedMeshOptions& opts,
                                   std::vector<uint8_t>& outBytes)
{
    outBytes.clear();
    if (mesh.vertices.empty() || mesh.indices.empty()) return false;

    const size_t nVerts = mesh.vertices.size() / 3;

    // Guard: the encoder only supports uint16 indices.  CesiumJS infers
    // the index width from vertex count (≤ 64*1024 → uint16, else uint32),
    // and the current WriteTriangles/edge code always writes uint16.
    // nVerts == 65536 is also rejected because the HWM wrap trick used in
    // WriteTriangles breaks when the max index 65535 is followed by 0
    // (code wraps to 0, the decoder misreads it as a new-high-water marker).
    if (nVerts >= 65536)
    {
        std::cerr << "[QuantizedMeshEncoder] Tile has " << nVerts
                  << " vertices (max supported: 65535).  Reduce --samples "
                  << "or increase --error to produce fewer vertices."
                  << std::endl;
        return false;
    }
    const uint16_t MAX_UV = 32767;

    // — 1. Vertex order: keep meshopt's HWM-compatible order —
    // meshopt_generateVertexRemap (in TinSimplifier) numbers vertices by first-occurrence
    // in the simplified index stream, which is exactly the order Cesium's HWM (high-water-mark)
    // index decoder expects: indices appear in non-decreasing order with code=0 marking a
    // new highest index. Re-sorting by (v,u) here would renumber vertices and break that
    // invariant, producing negative indices after HWM decoding.
    //
    // Quantized-mesh spec also recommends (but does not require) sorted u/v for better
    // delta-encoding compression; HWM correctness is the hard requirement, so we preserve
    // meshopt's order and accept slightly worse u/v delta compression.
    std::vector<uint16_t> u_quant(nVerts), v_quant(nVerts);
    std::vector<float>    h_meters(nVerts);

    for (size_t i = 0; i < nVerts; ++i)
    {
        // mesh.vertices = (u_quant_f, v_quant_f, height_meters)
        u_quant[i] = static_cast<uint16_t>(std::round(mesh.vertices[i * 3 + 0]));
        v_quant[i] = static_cast<uint16_t>(std::round(mesh.vertices[i * 3 + 1]));
        h_meters[i] = mesh.vertices[i * 3 + 2];
    }

    const std::vector<unsigned>& sortedIndices = mesh.indices;
    const std::vector<uint16_t>& sortedU = u_quant;
    const std::vector<uint16_t>& sortedV = v_quant;
    const std::vector<float>&    sortedH = h_meters;

    // — 2. Tile center ECEF (computed once, reused by header + bounding sphere) —
    QuantizedMeshHeader hdr{};
    hdr.minimumHeight = mesh.minHeight;
    hdr.maximumHeight = mesh.maxHeight;
    {
        double midLat = (tile.south + tile.north) * 0.5 * GeodeticMath::DEG2RAD;
        double midLon = (tile.west  + tile.east)  * 0.5 * GeodeticMath::DEG2RAD;
        // Use actual vertex-content mid-height for the centre so the
        // bounding sphere is tight around the content, not inflated by
        // the global encoding range.  This prevents Cesium's "bounding
        // volume does not include all of its content" warning.
        float contentMinH = 1e30f, contentMaxH = -1e30f;
        for (size_t i = 0; i < nVerts; ++i) {
            float h = h_meters[i];
            if (h < contentMinH) contentMinH = h;
            if (h > contentMaxH) contentMaxH = h;
        }
        if (contentMaxH < contentMinH) { contentMinH = 0.0f; contentMaxH = 0.0f; }
        double midH = (contentMinH + contentMaxH) * 0.5;
        double cx, cy, cz;
        proj.GeographicToECEF(midLat, midLon, midH, cx, cy, cz);
        hdr.centerX = cx;
        hdr.centerY = cy;
        hdr.centerZ = cz;

        // Bounding-sphere centre = ECEF centroid of the decoded vertices
        // (u/v/height → lat/lon → ECEF, averaged).  The header `center` stays
        // at the tile's geographic midpoint for CesiumJS's precision reference,
        // while the bounding sphere uses the centroid so that hemisphere-wide
        // top-level tiles get a radius close to the Earth's radius (~6378 km)
        // instead of ~9000 km (the midpoint-to-pole distance, which broke
        // Cesium's updateFrustums with "RangeError: Invalid array length").
        const double kMaxUV   = static_cast<double>(TileConstants::MAX_QUANTIZED_UV);
        const double widthDeg = tile.east  - tile.west;
        const double heightDeg = tile.north - tile.south;
        const double degToRad = GeodeticMath::DEG2RAD;
        double bsX = 0.0, bsY = 0.0, bsZ = 0.0;
        for (size_t i = 0; i < nVerts; ++i)
        {
            const double lonDeg = tile.west  + (static_cast<double>(u_quant[i]) / kMaxUV) * widthDeg;
            const double latDeg = tile.south + (static_cast<double>(v_quant[i]) / kMaxUV) * heightDeg;
            double ex, ey, ez;
            proj.GeographicToECEF(latDeg * degToRad, lonDeg * degToRad,
                                  static_cast<double>(h_meters[i]), ex, ey, ez);
            bsX += ex; bsY += ey; bsZ += ez;
        }
        const double invN = 1.0 / static_cast<double>(nVerts);
        hdr.boundingSphereCenterX = bsX * invN;
        hdr.boundingSphereCenterY = bsY * invN;
        hdr.boundingSphereCenterZ = bsZ * invN;
    }
    hdr.boundingSphereRadius = ComputeBoundingSphereRadius(mesh, tile, proj,
                                  hdr.boundingSphereCenterX,
                                  hdr.boundingSphereCenterY,
                                  hdr.boundingSphereCenterZ);
    ComputeHorizonOcclusionPoint(mesh, tile, proj,
                                  hdr.boundingSphereCenterX,
                                  hdr.boundingSphereCenterY,
                                  hdr.boundingSphereCenterZ,
                                  hdr.boundingSphereRadius,
                                  hdr.horizonOcclusionPointX,
                                  hdr.horizonOcclusionPointY,
                                  hdr.horizonOcclusionPointZ);

    // — 3. Write header —
    WriteHeader(hdr, outBytes);

    // — 4. Write vertices (zigzag + delta encoded) —
    WriteU32LE(static_cast<uint32_t>(nVerts), outBytes);

    // u: zigzag delta encoded as FIXED uint16 LE (Cesium 1.111 reads via Uint16Array)
    // NOTE: Cesium spec mentions varint, but the actual Cesium 1.111 decoder (Fpt)
    // creates Uint16Array(t, a, P*3) and zigzag-delta decodes in place. Fixed uint16.
    {
        int32_t prev = 0;
        for (size_t i = 0; i < nVerts; ++i)
        {
            int32_t cur = static_cast<int32_t>(sortedU[i]);
            int32_t delta = cur - prev;
            uint32_t zz = ZigZagEncode(delta);
            WriteU16LE(static_cast<uint16_t>(zz & 0xFFFF), outBytes);
            prev = cur;
        }
    }
    // v: zigzag delta encoded as FIXED uint16 LE
    {
        int32_t prev = 0;
        for (size_t i = 0; i < nVerts; ++i)
        {
            int32_t cur = static_cast<int32_t>(sortedV[i]);
            int32_t delta = cur - prev;
            uint32_t zz = ZigZagEncode(delta);
            WriteU16LE(static_cast<uint16_t>(zz & 0xFFFF), outBytes);
            prev = cur;
        }
    }
    // height: uint16 zigzag-delta encoded as FIXED uint16 LE
    // (quantized to 16-bit using min/max per spec)
    {
        float minH = mesh.minHeight;
        float maxH = mesh.maxHeight;
        float range = maxH - minH;
        if (range < 1e-6f) range = 1.0f;  // avoid div-by-zero for flat tiles
        int32_t prev = 0;
        for (size_t i = 0; i < nVerts; ++i)
        {
            float normalized = (sortedH[i] - minH) / range;
            if (normalized < 0.0f) normalized = 0.0f;
            if (normalized > 1.0f) normalized = 1.0f;
            int32_t cur = static_cast<int32_t>(std::round(normalized * TileConstants::MAX_QUANTIZED_UV));
            int32_t delta = cur - prev;
            uint32_t zz = ZigZagEncode(delta);
            WriteU16LE(static_cast<uint16_t>(zz & 0xFFFF), outBytes);
            prev = cur;
        }
    }

    // — 5. Write triangle indices (high-water-mark encoded) —
    // Extension flag: write 1 byte = 1 (VertexData.TriangleIndex extension present)
    // Per spec: if extension flag present, triangle indices follow
    // Actually the format is: after vertex data, there is a uint32 triangle count,
    // then indices. No extension flag byte needed for the base TriangleIndices.
    // (The extension flag is only for optional metadata extension at the end.)
    WriteTriangles(sortedIndices, nVerts, outBytes);

    // — 6. Write edge indices —
    // Collect edge vertices by classification (bit mask: corners appear in both edges)
    std::array<std::vector<unsigned>, 4> edgeVerts;  // W, S, E, N
    for (size_t i = 0; i < nVerts; ++i)
    {
        int mask = ClassifyEdge(sortedU[i], sortedV[i], MAX_UV);
        if (mask & 1) edgeVerts[0].push_back(static_cast<unsigned>(i));  // West
        if (mask & 2) edgeVerts[1].push_back(static_cast<unsigned>(i));  // South
        if (mask & 4) edgeVerts[2].push_back(static_cast<unsigned>(i));  // East
        if (mask & 8) edgeVerts[3].push_back(static_cast<unsigned>(i));  // North
    }
    // Sort each edge for deterministic output.
    // V convention: v=0=south, v=max=north.
    // West (u=0): by v ascending (south → north)
    // South (v=0): by u ascending (west → east)
    // East (u=max): by v descending (north → south, continues CCW from West)
    // North (v=max): by u descending (east → west, completes CCW loop)
    std::sort(edgeVerts[0].begin(), edgeVerts[0].end(),
              [&](unsigned a, unsigned b) { return sortedV[a] < sortedV[b]; });
    std::sort(edgeVerts[1].begin(), edgeVerts[1].end(),
              [&](unsigned a, unsigned b) { return sortedU[a] < sortedU[b]; });
    std::sort(edgeVerts[2].begin(), edgeVerts[2].end(),
              [&](unsigned a, unsigned b) { return sortedV[a] > sortedV[b]; });
    std::sort(edgeVerts[3].begin(), edgeVerts[3].end(),
              [&](unsigned a, unsigned b) { return sortedU[a] > sortedU[b]; });

    // Write edge counts and indices
    // Per spec: counts are uint32, indices are FIXED uint16 LE
    // (Cesium 1.111 reads edge indices via Uint16Array, not varint)
    for (int e = 0; e < 4; ++e)
    {
        WriteU32LE(static_cast<uint32_t>(edgeVerts[e].size()), outBytes);
        for (unsigned idx : edgeVerts[e])
        {
            WriteU16LE(static_cast<uint16_t>(idx), outBytes);
        }
    }

    // — 7. Write OctVertexNormals extension (optional) —
    if (opts.writeOctVertexNormals)
    {
        // Extension header: 1 byte extension ID + 4 bytes length
        // OctVertexNormals extension ID = 1
        // Vertices are no longer re-sorted (preserving HWM order), so normals
        // computed from the original mesh.vertices/indices are already in the
        // same order as the vertices we wrote above.
        std::vector<float> normals;
        ComputeVertexNormals(mesh.vertices, mesh.indices, tile, proj, normals);
        WriteOctNormals(normals, nVerts, outBytes);
    }

    return true;
}

void QuantizedMeshEncoder::WriteHeader(const QuantizedMeshHeader& h, std::vector<uint8_t>& out)
{
    WriteF64LE(h.centerX, out);
    WriteF64LE(h.centerY, out);
    WriteF64LE(h.centerZ, out);
    WriteF32LE(h.minimumHeight, out);
    WriteF32LE(h.maximumHeight, out);
    WriteF64LE(h.boundingSphereCenterX, out);
    WriteF64LE(h.boundingSphereCenterY, out);
    WriteF64LE(h.boundingSphereCenterZ, out);
    WriteF64LE(h.boundingSphereRadius, out);
    WriteF64LE(h.horizonOcclusionPointX, out);
    WriteF64LE(h.horizonOcclusionPointY, out);
    WriteF64LE(h.horizonOcclusionPointZ, out);
}

void QuantizedMeshEncoder::WriteTriangles(const std::vector<unsigned>& indices,
                                           size_t vertexCount,
                                           std::vector<uint8_t>& out)
{
    const size_t nTris = indices.size() / 3;
    WriteU32LE(static_cast<uint32_t>(nTris), out);

    // Per quantized-mesh spec: indices use high-water-mark encoding.
    //   code = highest - idx
    //   if (code == 0): highest = idx + 1   (Cesium decoder uses ==, not >=)
    //   write code as FIXED uint16 LE (Cesium 1.111 reads via Uint16Array, NO zigzag)
    // Decoder: idx = highest - code; if (code == 0) highest++.
    // uint16 truncation handles idx > highest cases correctly.
    uint32_t hwm = 0;
    int hwm_violations = 0;
    for (size_t i = 0; i < indices.size(); ++i)
    {
        int32_t idx = static_cast<int32_t>(indices[i]);
        int32_t code = static_cast<int32_t>(hwm) - idx;  // code = highest - idx
        if (idx > static_cast<int32_t>(hwm)) ++hwm_violations;
        WriteU16LE(static_cast<uint16_t>(code & 0xFFFF), out);
        if (code == 0) hwm = static_cast<uint32_t>(idx + 1);
    }
    if (hwm_violations > 0)
        std::cerr << "[HWM] " << hwm_violations << " violations (idx > hwm) in "
                  << nTris << " triangles" << std::endl;
}

void QuantizedMeshEncoder::WriteOctNormals(const std::vector<float>& normals,
                                            size_t vertexCount,
                                            std::vector<uint8_t>& out)
{
    // Extension marker: 1 byte = 1 (OctVertexNormals extension ID)
    out.push_back(1);
    // Extension length: 4 bytes (uint32 LE) = 2 * vertexCount
    WriteU32LE(static_cast<uint32_t>(vertexCount * 2), out);
    // Oct-encoded normals: 2 bytes per vertex
    for (size_t i = 0; i < vertexCount; ++i)
    {
        uint8_t enc[2];
        OctEncode(normals[i * 3 + 0], normals[i * 3 + 1], normals[i * 3 + 2], enc);
        out.push_back(enc[0]);
        out.push_back(enc[1]);
    }
    // No extension terminator byte.  The Cesium quantized-mesh decoder
    // (CesiumTerrainProvider.js) uses a while (pos < view.byteLength) loop
    // without an extensionId==0 early-exit check.  A trailing zero byte
    // would be read as extensionId=0 at the last byte, then the subsequent
    // getUint32(pos) for the length would read 4 bytes past the buffer,
    // producing a RangeError.  Cesium's reference tiles also have no
    // terminator — the loop terminates naturally at EOF.
}

double QuantizedMeshEncoder::ComputeBoundingSphereRadius(const TinMesh& mesh,
                                                  const TerrainTile& tile,
                                                  CProjectionEngine& proj,
                                                  double cx, double cy, double cz)
{
    // Bounding sphere center is pre-computed by the caller (tile geographic
    // center at mid-height, shared with the header centerX/Y/Z).

    // Compute the bounding sphere from ALL mesh vertices, not just the 8
    // geographic corners.  The geographic→ECEF transform is non-linear
    // (N(φ) varies with latitude by ~21 km between the equator and poles),
    // so the maximum ECEF distance from the sphere centre is not guaranteed
    // to occur at a corner of the geographic box — an interior vertex at a
    // latitude with a larger prime-vertical radius can be farther away.
    // Iterating every vertex guarantees the sphere encloses the exact mesh
    // that CesiumJS will decode, eliminating "bounding volume does not
    // include all of its content" warnings.
    const size_t nVerts = mesh.vertices.size() / 3;
    if (nVerts == 0) return 0.0;

    const double kMaxUV     = static_cast<double>(TileConstants::MAX_QUANTIZED_UV);
    const double widthDeg   = tile.east  - tile.west;
    const double heightDeg  = tile.north - tile.south;
    const double degToRad   = GeodeticMath::DEG2RAD;

    double maxDistSq = 0.0;
    for (size_t i = 0; i < nVerts; ++i)
    {
        const double u   = static_cast<double>(mesh.vertices[i * 3 + 0]);
        const double v   = static_cast<double>(mesh.vertices[i * 3 + 1]);
        const double hM  = static_cast<double>(mesh.vertices[i * 3 + 2]);

        const double lonDeg = tile.west  + (u / kMaxUV) * widthDeg;
        const double latDeg = tile.south + (v / kMaxUV) * heightDeg;

        double ex, ey, ez;
        proj.GeographicToECEF(latDeg * degToRad, lonDeg * degToRad, hM, ex, ey, ez);
        double dx = ex - cx, dy = ey - cy, dz = ez - cz;
        double d2 = dx * dx + dy * dy + dz * dz;
        if (d2 > maxDistSq) maxDistSq = d2;
    }

    // Safety margins:
    //   1 mm — guards against float↔double round-trip differences between
    //          the C++ encoder and the JavaScript decoder.
    //   heightQuantizationError — the encoder quantizes heights to uint16
    //          against [minHeight, maxHeight] (QuantizedMeshEncoder.cpp
    //          Encode), so the decoded height can differ from the raw
    //          height used in this loop by up to range/65534.  Without
    //          this margin, the decoded mesh can poke outside the header
    //          bounding sphere, triggering Cesium's "bounding volume does
    //          not include all of its content" warning.
    const double heightRange = static_cast<double>(mesh.maxHeight - mesh.minHeight);
    const double heightQuantizationError = heightRange / 65534.0;
    return std::sqrt(maxDistSq) + std::max(1e-3, heightQuantizationError);
}

void QuantizedMeshEncoder::ComputeHorizonOcclusionPoint(const TinMesh& mesh,
                                                         const TerrainTile& tile,
                                                         CProjectionEngine& proj,
                                                         double bsCx, double bsCy, double bsCz, double bsR,
                                                         double& hx, double& hy, double& hz)
{
    // Simplified horizon occlusion point: extend the bounding sphere center
    // outward along the Earth-center direction by the sphere radius. This is a
    // conservative approximation; the full Cesium algorithm projects all vertices
    // onto a plane and finds the extremal point.

    // Direction from Earth center to bounding sphere center
    double len = std::sqrt(bsCx * bsCx + bsCy * bsCy + bsCz * bsCz);
    if (len < 1e-6) { hx = bsCx; hy = bsCy; hz = bsCz; return; }

    double dirX = bsCx / len, dirY = bsCy / len, dirZ = bsCz / len;
    // Extend by bounding sphere radius
    hx = bsCx + dirX * bsR;
    hy = bsCy + dirY * bsR;
    hz = bsCz + dirZ * bsR;
}

void QuantizedMeshEncoder::ComputeVertexNormals(const std::vector<float>& vertices,
                                                 const std::vector<unsigned>& indices,
                                                 const TerrainTile& tile,
                                                 CProjectionEngine& proj,
                                                 std::vector<float>& outNormals)
{
    const size_t n = vertices.size() / 3;
    outNormals.assign(n * 3, 0.0f);

    // — 1. Compute tile centre in ECEF (for tile-local offset) —
    const double midLatDeg = (tile.south + tile.north) * 0.5;
    const double midLonDeg = (tile.west  + tile.east)  * 0.5;

    static int callCount = 0;
    if (++callCount <= 3)
        fprintf(stderr, "[Normals-ECEF] tile %d/%d/%d center lat=%.4f lon=%.4f\n",
                tile.level, tile.x, tile.y, midLatDeg, midLonDeg);

    float minH = 1e30f, maxH = -1e30f;
    for (size_t i = 0; i < n; ++i) {
        float h = vertices[i * 3 + 2];
        if (h < minH) minH = h;
        if (h > maxH) maxH = h;
    }
    if (maxH < minH) { minH = 0.0f; maxH = 0.0f; }

    double cx, cy, cz;
    proj.GeographicToECEF(midLatDeg * Angle::DEG_TO_RAD,
                           midLonDeg * Angle::DEG_TO_RAD,
                           (minH + maxH) * 0.5, cx, cy, cz);

    // — 2. Convert all vertices from UVH → tile-local ECEF —
    const float    kMaxUV    = TileConstants::MAX_QUANTIZED_UV;
    const double   widthDeg  = tile.east  - tile.west;
    const double   heightDeg = tile.north - tile.south;
    std::vector<float> posECEF(n * 3);

    for (size_t i = 0; i < n; ++i)
    {
        double lonDeg = tile.west  + (static_cast<double>(vertices[i * 3 + 0]) / kMaxUV) * widthDeg;
        double latDeg = tile.south + (static_cast<double>(vertices[i * 3 + 1]) / kMaxUV) * heightDeg;
        double h      = static_cast<double>(vertices[i * 3 + 2]);

        double gx, gy, gz;
        proj.GeographicToECEF(latDeg * Angle::DEG_TO_RAD,
                               lonDeg * Angle::DEG_TO_RAD,
                               h, gx, gy, gz);
        posECEF[i * 3 + 0] = static_cast<float>(gx - cx);
        posECEF[i * 3 + 1] = static_cast<float>(gy - cy);
        posECEF[i * 3 + 2] = static_cast<float>(gz - cz);
    }

    // — 3. Compute geocentric Up for near-vertical face filtering —
    float upLen = static_cast<float>(std::sqrt(cx*cx + cy*cy + cz*cz));
    float Ux = static_cast<float>(cx) / upLen;
    float Uy = static_cast<float>(cy) / upLen;
    float Uz = static_cast<float>(cz) / upLen;

    // — 4. Compute face normals in ECEF, accumulate (equal-weight voting) —
    for (size_t t = 0; t + 2 < indices.size(); t += 3)
    {
        unsigned i0 = indices[t], i1 = indices[t + 1], i2 = indices[t + 2];

        float ax = posECEF[i1 * 3 + 0] - posECEF[i0 * 3 + 0];
        float ay = posECEF[i1 * 3 + 1] - posECEF[i0 * 3 + 1];
        float az = posECEF[i1 * 3 + 2] - posECEF[i0 * 3 + 2];
        float bx = posECEF[i2 * 3 + 0] - posECEF[i0 * 3 + 0];
        float by = posECEF[i2 * 3 + 1] - posECEF[i0 * 3 + 1];
        float bz = posECEF[i2 * 3 + 2] - posECEF[i0 * 3 + 2];

        float nx = ay * bz - az * by;
        float ny = az * bx - ax * bz;
        float nz = ax * by - ay * bx;

        float flen = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (flen > 1e-10f) {
            // Skip near-vertical faces: their horizontal normal direction
            // is unreliable because small floating-point errors in vertex
            // positions (tile-local ECEF stored as float32) can rotate a
            // near-vertical face's lateral normal by ~10°.  The threshold
            // 2e-3 corresponds to faces tilted less than ~0.11° from
            // horizontal — essentially vertical walls whose orientation is
            // dominated by grid-sampling noise rather than terrain shape.
            float dotUp = (nx*Ux + ny*Uy + nz*Uz) / flen;
            if (dotUp < 2e-3f) continue;

            float inv = 1.0f / flen;
            outNormals[i0 * 3 + 0] += nx * inv;
            outNormals[i0 * 3 + 1] += ny * inv;
            outNormals[i0 * 3 + 2] += nz * inv;
            outNormals[i1 * 3 + 0] += nx * inv;
            outNormals[i1 * 3 + 1] += ny * inv;
            outNormals[i1 * 3 + 2] += nz * inv;
            outNormals[i2 * 3 + 0] += nx * inv;
            outNormals[i2 * 3 + 1] += ny * inv;
            outNormals[i2 * 3 + 2] += nz * inv;
        }
    }

    // — 5. Normalize and output ECEF normals —
    // CesiumJS treats oct-encoded quantized-mesh vertex normals as
    // model-coordinate (ECEF) vectors — see the terrain vertex shader
    // (GlobeVS.glsl):
    //     vec3 normalMC = czm_octDecode(encodedNormal);
    //     v_normalEC   = czm_normal3D * v_normalMC;
    // The decoder applies no ENU transform to the normal, so the tile must
    // store the normal in ECEF.  Rotating it into a local ENU frame (as an
    // earlier version did) mis-orients lighting everywhere except where the
    // ENU axes coincidentally align with ECEF.
    //
    // Degenerate vertices (no contributing face) default to the geodetic
    // surface normal (Up) at the tile centre, expressed in ECEF.
    const double midLatRad = midLatDeg * Angle::DEG_TO_RAD;
    const double midLonRad = midLonDeg * Angle::DEG_TO_RAD;
    const double sinLat = std::sin(midLatRad), cosLat = std::cos(midLatRad);
    const double sinLon = std::sin(midLonRad), cosLon = std::cos(midLonRad);
    const float UpEcef[3] = {
        static_cast<float>(cosLat * cosLon),
        static_cast<float>(cosLat * sinLon),
        static_cast<float>(sinLat)
    };

    int ecefBad = 0;
    for (size_t i = 0; i < n; ++i)
    {
        float nx = outNormals[i * 3 + 0];
        float ny = outNormals[i * 3 + 1];
        float nz = outNormals[i * 3 + 2];
        float len = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (len > 1e-10f)
        {
            outNormals[i * 3 + 0] = nx / len;
            outNormals[i * 3 + 1] = ny / len;
            outNormals[i * 3 + 2] = nz / len;
        }
        else
        {
            outNormals[i * 3 + 0] = UpEcef[0];
            outNormals[i * 3 + 1] = UpEcef[1];
            outNormals[i * 3 + 2] = UpEcef[2];
        }

        // An ECEF normal must point generally away from Earth centre
        // (dot with the geocentric radial at the tile centre > 0).
        float d = outNormals[i * 3 + 0] * Ux
                + outNormals[i * 3 + 1] * Uy
                + outNormals[i * 3 + 2] * Uz;
        if (d <= 0.0f) ++ecefBad;
    }
    if (ecefBad > 0)
        fprintf(stderr, "[Normals] %zu verts: %d point away-from-Earth (tile %d/%d/%d)\n",
                n, ecefBad, tile.level, tile.x, tile.y);
}

void QuantizedMeshEncoder::OctEncode(float nx, float ny, float nz, uint8_t out[2])
{
    // Octahedral encoding: project unit sphere to octahedron, then to square
    // Reference: "A Survey of Efficient Representations for Independent Unit Vectors"
    float l1norm = std::abs(nx) + std::abs(ny) + std::abs(nz);
    if (l1norm > 1e-10f)
    {
        nx /= l1norm; ny /= l1norm; nz /= l1norm;
    }

    float u, v;
    if (nz >= 0.0f)
    {
        u = nx;
        v = ny;
    }
    else
    {
        // Reflect lower hemisphere
        u = (1.0f - std::abs(ny)) * (nx >= 0 ? 1.0f : -1.0f);
        v = (1.0f - std::abs(nx)) * (ny >= 0 ? 1.0f : -1.0f);
    }

    // Map [-1, 1] to [0, 255]
    out[0] = static_cast<uint8_t>(std::round((u * 0.5f + 0.5f) * 255.0f));
    out[1] = static_cast<uint8_t>(std::round((v * 0.5f + 0.5f) * 255.0f));
}

void QuantizedMeshEncoder::WriteU32LE(uint32_t v, std::vector<uint8_t>& out)
{
    out.push_back(static_cast<uint8_t>(v & 0xFF));
    out.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
    out.push_back(static_cast<uint8_t>((v >> 16) & 0xFF));
    out.push_back(static_cast<uint8_t>((v >> 24) & 0xFF));
}

void QuantizedMeshEncoder::WriteU16LE(uint16_t v, std::vector<uint8_t>& out)
{
    out.push_back(static_cast<uint8_t>(v & 0xFF));
    out.push_back(static_cast<uint8_t>((v >> 8) & 0xFF));
}

void QuantizedMeshEncoder::WriteF32LE(float v, std::vector<uint8_t>& out)
{
    uint32_t bits;
    std::memcpy(&bits, &v, 4);
    WriteU32LE(bits, out);
}

void QuantizedMeshEncoder::WriteF64LE(double v, std::vector<uint8_t>& out)
{
    uint64_t bits;
    std::memcpy(&bits, &v, 8);
    for (int i = 0; i < 8; ++i)
        out.push_back(static_cast<uint8_t>((bits >> (i * 8)) & 0xFF));
}
