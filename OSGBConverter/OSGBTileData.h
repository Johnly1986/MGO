// Copyright Johnlyon
//
// OSGBTileData — shared data structures for the OSGB conversion pipeline
//

#pragma once

#include "macro.h"

#include <vector>
#include <string>
#include <cstdint>
#include <cmath>

// ---------------------------------------------------------------------------
// Raw geometry extracted from a single OSGB tile
// ---------------------------------------------------------------------------

// Per-texture sub-group of geometry within a tile.
// An OSGB tile may contain multiple textures (e.g., different building faces).
// Each TextureGroup holds geometry that shares a single texture.
struct TextureGroup
{
    std::vector<float> positions;       // (x, y, z) × n, flat
    std::vector<float> normals;         // (nx, ny, nz) × n, flat
    std::vector<float> texcoords;       // (u, v) × n, flat
    std::vector<unsigned int> indices;  // triangle list

    std::string texturePath;            // relative path to texture image
    float baseColorFactor[4] = { 1.0f, 1.0f, 1.0f, 1.0f };

    // Embedded texture pixels (RGBA8) when the OSGB embeds its textures
    // inline rather than referencing external image files.
    std::vector<uint8_t> texturePixels;
    int textureWidth = 0;
    int textureHeight = 0;

    // Local bounding box for this group
    double bboxMin[3] = { 0, 0, 0 };
    double bboxMax[3] = { 0, 0, 0 };

    bool HasPositions() const { return !positions.empty(); }
    bool HasNormals()   const { return !normals.empty(); }
    bool HasTexCoords() const { return !texcoords.empty(); }
    bool HasIndices()   const { return !indices.empty(); }
    bool HasTexture()   const { return !texturePath.empty(); }
    size_t VertexCount() const { return positions.size() / 3; }
    size_t TriangleCount() const { return indices.size() / 3; }
    bool IsEmpty() const { return positions.empty() || indices.empty(); }

    void ComputeBBox()
    {
        if (positions.empty()) return;
        bboxMin[0] = bboxMin[1] = bboxMin[2] = 1e30;
        bboxMax[0] = bboxMax[1] = bboxMax[2] = -1e30;
        for (size_t i = 0; i < positions.size(); i += 3)
        {
            double x = positions[i];
            double y = positions[i + 1];
            double z = positions[i + 2];
            if (x < bboxMin[0]) bboxMin[0] = x;
            if (y < bboxMin[1]) bboxMin[1] = y;
            if (z < bboxMin[2]) bboxMin[2] = z;
            if (x > bboxMax[0]) bboxMax[0] = x;
            if (y > bboxMax[1]) bboxMax[1] = y;
            if (z > bboxMax[2]) bboxMax[2] = z;
        }
    }

    double Diagonal() const
    {
        double dx = bboxMax[0] - bboxMin[0];
        double dy = bboxMax[1] - bboxMin[1];
        double dz = bboxMax[2] - bboxMin[2];
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }
};

struct OSGBTileData
{
    // Per-texture geometry groups (preferred for multi-texture tiles)
    std::vector<TextureGroup> groups;

    // Legacy: single-group vertex attributes (for tiles with one texture)
    std::vector<float> positions;       // (x, y, z) × n, flat
    std::vector<float> normals;         // (nx, ny, nz) × n, flat
    std::vector<float> texcoords;       // (u, v) × n, flat
    std::vector<unsigned int> indices;  // triangle list

    // Bounding box in local tile coordinates (before any projection transform)
    double bboxMin[3] = { 0, 0, 0 };
    double bboxMax[3] = { 0, 0, 0 };

    // Tile transform (4×4 column-major, from OSG MatrixTransform)
    double localTransform[16] = {
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    };

    // Material properties
    float baseColorFactor[4] = { 1.0f, 1.0f, 1.0f, 1.0f };

    // Texture references (relative to OSGB tile directory)
    std::string texturePath;

    // Tile metadata
    std::string tilePath;         // relative path from root
    int         lodLevel = 0;     // LOD level (0 = coarsest)
    int         tileX = 0;        // quadtree x index
    int         tileY = 0;        // quadtree y index
    std::string subTileIndex;     // sub-tile index (e.g. "00002200"), empty for root tiles

    bool HasPositions() const { return !positions.empty(); }
    bool HasNormals()   const { return !normals.empty(); }
    bool HasTexCoords() const { return !texcoords.empty(); }
    bool HasIndices()   const { return !indices.empty(); }
    bool HasTexture()   const { return !texturePath.empty(); }

    size_t VertexCount() const { return positions.size() / 3; }
    size_t TriangleCount() const { return indices.size() / 3; }

    // Compute local bounding box from positions
    void ComputeBBox()
    {
        if (positions.empty()) return;
        bboxMin[0] = bboxMin[1] = bboxMin[2] = 1e30;
        bboxMax[0] = bboxMax[1] = bboxMax[2] = -1e30;
        for (size_t i = 0; i < positions.size(); i += 3)
        {
            double x = positions[i];
            double y = positions[i + 1];
            double z = positions[i + 2];
            if (x < bboxMin[0]) bboxMin[0] = x;
            if (y < bboxMin[1]) bboxMin[1] = y;
            if (z < bboxMin[2]) bboxMin[2] = z;
            if (x > bboxMax[0]) bboxMax[0] = x;
            if (y > bboxMax[1]) bboxMax[1] = y;
            if (z > bboxMax[2]) bboxMax[2] = z;
        }
    }

    double Diagonal() const
    {
        double dx = bboxMax[0] - bboxMin[0];
        double dy = bboxMax[1] - bboxMin[1];
        double dz = bboxMax[2] - bboxMin[2];
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    bool IsEmpty() const
    {
        if (!groups.empty())
        {
            for (auto& g : groups)
                if (!g.IsEmpty()) return false;
            return true;
        }
        return positions.empty() || indices.empty();
    }

    // Convert legacy single-group format to multi-group format
    void EnsureGroups()
    {
        if (!groups.empty()) return;
        if (positions.empty()) return;

        TextureGroup g;
        g.positions  = std::move(positions);
        g.normals    = std::move(normals);
        g.texcoords  = std::move(texcoords);
        g.indices    = std::move(indices);
        g.texturePath = texturePath;
        for (int i = 0; i < 4; ++i) g.baseColorFactor[i] = baseColorFactor[i];
        for (int i = 0; i < 3; ++i) { g.bboxMin[i] = bboxMin[i]; g.bboxMax[i] = bboxMax[i]; }
        groups.push_back(std::move(g));
    }
};

// ---------------------------------------------------------------------------
// Projection mode determined from metadata.xml
// ---------------------------------------------------------------------------
enum class ProjectionMode
{
    RootOnly,   // EPSG:454x GK — only root transform, no per-tile correction
    PerTile,    // ENU / unknown — per-tile delta correction
    None        // No projection information
};

// ---------------------------------------------------------------------------
// Data vendor detection
// ---------------------------------------------------------------------------
enum class DataVendor
{
    Unknown,
    ContextCapture,  // Data/Tile_+XXX_+YYY/ pattern
    DJITerra,        // terra_osgbs/Block_*/ pattern
};

// ---------------------------------------------------------------------------
// Parsed metadata.xml information
// ---------------------------------------------------------------------------
struct OSGBMetadata
{
    std::string  srs;              // e.g. "EPSG:4547" or "ENU:22.64785,113.06277"
    double       originX = 0;      // easting (or ENU east)
    double       originY = 0;      // northing (or ENU north)
    double       originZ = 0;      // height

    // ENU-specific: geographic origin (lat, lon in degrees)
    double       enuLat = 0;
    double       enuLon = 0;
    bool         isENU = false;    // true if SRS starts with "ENU:"

    double       bboxMin[3] = { 0, 0, 0 };
    double       bboxMax[3] = { 0, 0, 0 };

    ProjectionMode projectionMode = ProjectionMode::None;
    DataVendor    vendor = DataVendor::Unknown;

    // Tile inventory (relative paths from root)
    std::vector<std::string> tilePaths;

    // Max LOD level detected from tile paths
    int maxLOD = 0;

    bool HasProjection() const { return !srs.empty() || isENU; }
    bool HasOrigin()     const { return originX != 0 || originY != 0 || originZ != 0; }
};

// ---------------------------------------------------------------------------
// Internal tile node for quadtree construction
// ---------------------------------------------------------------------------
struct OSGBTileNode
{
    OSGBTileData data;
    int          level = 0;
    int          x = 0, y = 0;       // quadtree indices
    std::string  path;                // relative tile path

    // World-space bounding box (after projection correction)
    double worldBBoxMin[3] = { 0, 0, 0 };
    double worldBBoxMax[3] = { 0, 0, 0 };

    // 4 children: NW, NE, SW, SE
    OSGBTileNode* children[4] = { nullptr, nullptr, nullptr, nullptr };
    OSGBTileNode* parent = nullptr;

    bool hasContent = false;

    ~OSGBTileNode()
    {
        for (int i = 0; i < 4; ++i)
            delete children[i];
    }

    bool IsLeaf() const
    {
        return children[0] == nullptr
            && children[1] == nullptr
            && children[2] == nullptr
            && children[3] == nullptr;
    }

    bool HasChildren() const
    {
        return children[0] != nullptr
            || children[1] != nullptr
            || children[2] != nullptr
            || children[3] != nullptr;
    }
};