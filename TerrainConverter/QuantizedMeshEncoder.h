#pragma once

#include "macro.h"
#include "HeightmapGrid.h"
#include <vector>
#include <cstdint>
#include <string>

// Forward declarations
class CProjectionEngine;

// TinMesh — simplified terrain mesh in tile-local coordinates
struct TinMesh
{
    std::vector<float>    vertices;   // (u_quant, v_quant, height_meters) × n
    std::vector<unsigned> indices;    // triangle indices
    float                 minHeight = 0.0f;
    float                 maxHeight = 0.0f;
};

// TerrainTile — geographic bounds + local heightmap subset
struct TerrainTile
{
    int    level = 0;
    int    x = 0, y = 0;
    double west = 0, south = 0, east = 0, north = 0;  // degrees

    // Local heightmap subset (already resampled)
    std::vector<float> localHeights;
    int    localWidth = 0, localHeight = 0;
    double localMinEasting = 0, localMaxEasting = 0;
    double localMinNorthing = 0, localMaxNorthing = 0;

    // No-data metadata (propagated from source grid so downstream stages can filter)
    bool   hasNoData   = false;
    float  noDataValue = -9999.0f;

    bool   hasContent = false;
};

// QuantizedMeshOptions — encoder options
struct QuantizedMeshOptions
{
    bool writeOctVertexNormals = true;
    bool writeWaterMask        = false;
    bool writeMetadata         = false;
};

// QuantizedMeshHeader — 88 bytes, little-endian
#pragma pack(push, 1)
struct QuantizedMeshHeader
{
    double centerX, centerY, centerZ;                                  // 24
    float  minimumHeight, maximumHeight;                               // 8
    double boundingSphereCenterX, boundingSphereCenterY, boundingSphereCenterZ; // 24
    double boundingSphereRadius;                                       // 8
    double horizonOcclusionPointX, horizonOcclusionPointY, horizonOcclusionPointZ; // 24
    // Total: 88
};
#pragma pack(pop)
static_assert(sizeof(QuantizedMeshHeader) == 88, "QuantizedMeshHeader must be 88 bytes");

// QuantizedMeshEncoder — encodes TinMesh to quantized-mesh 1.0 binary format
//
// Reference: https://github.com/CesiumGS/quantized-mesh
//
class TERRAIN_CONVERTER_API QuantizedMeshEncoder
{
public:
    // Encode a single tile to .terrain format
    static bool Encode(const TinMesh& mesh,
                       const TerrainTile& tile,
                       CProjectionEngine& proj,
                       const QuantizedMeshOptions& opts,
                       std::vector<uint8_t>& outBytes);

private:
    // — Encoding helpers —
    static void WriteHeader(const QuantizedMeshHeader& h, std::vector<uint8_t>& out);
    static void WriteTriangles(const std::vector<unsigned>& indices,
                                size_t vertexCount,
                                std::vector<uint8_t>& out);
    static void WriteOctNormals(const std::vector<float>& normals,
                                 size_t vertexCount,
                                 std::vector<uint8_t>& out);

    // — Geometry helpers —
    static double ComputeBoundingSphereRadius(const TinMesh& mesh,
                                               const TerrainTile& tile,
                                               CProjectionEngine& proj,
                                               double cx, double cy, double cz);
    static void ComputeHorizonOcclusionPoint(const TinMesh& mesh,
                                              const TerrainTile& tile,
                                              CProjectionEngine& proj,
                                              double bsCx, double bsCy, double bsCz, double bsR,
                                              double& hx, double& hy, double& hz);
    static void ComputeVertexNormals(const std::vector<float>& vertices,
                                      const std::vector<unsigned>& indices,
                                      const TerrainTile& tile,
                                      CProjectionEngine& proj,
                                      std::vector<float>& outNormals);
    static void OctEncode(float nx, float ny, float nz, uint8_t out[2]);

    // — Bit helpers —
    static uint32_t ZigZagEncode(int32_t v)
    {
        return static_cast<uint32_t>((v >> 31) ^ (v << 1));
    }

    static void WriteU32LE(uint32_t v, std::vector<uint8_t>& out);
    static void WriteF32LE(float v, std::vector<uint8_t>& out);
    static void WriteF64LE(double v, std::vector<uint8_t>& out);
    static void WriteU16LE(uint16_t v, std::vector<uint8_t>& out);
};
