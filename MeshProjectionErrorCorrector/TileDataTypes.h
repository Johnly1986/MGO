// Copyright Johnlyon
//
// TileDataTypes — shared data structures for 3D Tiles pipeline
//
// These types are used by both CProjectionEngine (MeshProjectionErrorCorrector DLL)
// and TileBuilder / TilesConverter.  Defined here to avoid circular
// dependencies between modules.
//

#pragma once

#include <cfloat>
#include <cstdint>
#include <vector>
#include <array>
#include <string>
#include <memory>

// ---------------------------------------------------------------------------
// Per-instance data (one per scene node -> mesh reference)
// ---------------------------------------------------------------------------
struct MeshInstance
{
    unsigned int meshIndex = 0;
    float worldTransform[16] = {};  // identity requires diagonal set by caller; zero-init so default-constructed instances are not garbage
    double bboxMin[3] = { DBL_MAX, DBL_MAX, DBL_MAX };
    double bboxMax[3] = { -DBL_MAX, -DBL_MAX, -DBL_MAX };

    // Vertex average (true centroid) in world-space Assimp Y-up (X=East, Y=Up, Z=North).
    // Computed in CollectMeshInstances by averaging all vertex positions.
    // Used as the reference point for per-instance delta computation: the delta
    // is computed here and applied as a uniform translation to all vertices,
    // minimizing the maximum residual across the instance.
    double vertexCentroid[3] = { 0.0, 0.0, 0.0 };
    size_t vertexCount = 0;

    inline double centroid(int axis) const
    {
        return (bboxMin[axis] + bboxMax[axis]) * 0.5;
    }
};

// ---------------------------------------------------------------------------
// Merged mesh group — all meshes sharing the same material index merged
// ---------------------------------------------------------------------------
struct MergedMeshGroup
{
    int materialIndex = -1;

    // Vertex attributes (deinterleaved)
    std::vector<float> positions;   // 3 floats per vertex
    std::vector<float> normals;     // 3 floats per vertex
    std::vector<float> texcoords;   // 2 floats per vertex
    std::vector<uint32_t> indices;  // triangle list

    size_t vertexCount() const { return positions.size() / 3; }
    size_t indexCount()  const { return indices.size(); }

    // World-space bounding box (min/max)
    double bboxMin[3] = { DBL_MAX, DBL_MAX, DBL_MAX };
    double bboxMax[3] = { -DBL_MAX, -DBL_MAX, -DBL_MAX };

    // Base color from material (RGBA 0..1)
    std::array<float, 4> baseColorFactor = { 1.0f, 1.0f, 1.0f, 1.0f };

    // Diffuse texture path from FBX material (empty if no texture)
    std::string diffuseTexturePath;

    // Embedded texture pixels (RGBA8) - set when the source embeds its
    // textures inline (e.g. ContextCapture OSGB) rather than external files.
    std::vector<uint8_t> texturePixels;
    int textureWidth = 0;
    int textureHeight = 0;
};

// ---------------------------------------------------------------------------
// In-memory binary blob
// ---------------------------------------------------------------------------
struct BinaryBlob
{
    std::vector<uint8_t> data;

    const uint8_t* ptr() const { return data.data(); }
    size_t size() const { return data.size(); }
};

// ---------------------------------------------------------------------------
// Grid cell for bottom-up sparse-wrap aggregation
// ---------------------------------------------------------------------------
struct GridCell
{
    double bboxMin[3], bboxMax[3];       // cell spatial extent (ECEF after GroupCellByMaterial)
    double localBboxMin[3], localBboxMax[3]; // cell extent in local projected coords (pre-ECEF)
    int level = 0;                       // 0 = finest, N-1 = coarsest
    int ix = 0, iy = 0, iz = 0;        // grid indices at this level
    std::string cellKey;                // "L_ix_iy_iz" — hash map key & tile name

    std::vector<MeshInstance> instances; // assigned mesh instances
    std::vector<MergedMeshGroup> materialGroups;
    std::string tileFileName;          // b3dm filename (without directory)
    bool hasContent = false;            // true if instances assigned from candidate pool
    bool isOverflow = false;            // true = root overflow tile
    bool isExternal = false;            // cells under a subdir tileset (external layout)

    mutable double computedGeomError = 0.0;

    std::vector<std::unique_ptr<GridCell>> children;
    GridCell* parent = nullptr;

    static int childSparseIndex(int cix, int ciy, int ciz)
    {
        return (cix & 1) | ((ciy & 1) << 1) | ((ciz & 1) << 2);
    }

    void ensureChildSlots()
    {
        if (children.size() < 8)
            children.resize(8);
    }

    bool hasChildren() const
    {
        for (auto& c : children) if (c) return true;
        return false;
    }

    size_t totalVertexCount() const
    {
        size_t n = 0;
        for (auto& g : materialGroups) n += g.vertexCount();
        return n;
    }
};
