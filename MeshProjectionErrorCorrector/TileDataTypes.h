// Copyright Johnlyon
//
// TileDataTypes — shared data structures for 3D Tiles pipeline
//
// These types are used by both CProjectionEngine (MeshProjectionErrorCorrector DLL)
// and TileBuilder / TilesConverter.  Defined here to avoid circular
// dependencies between modules.
//

#pragma once

#include "macro.h"

#include <cfloat>
#include <cstdint>
#include <vector>
#include <array>
#include <string>
#include <memory>
#include <map>
#include <unordered_map>
#include <utility>

// ---------------------------------------------------------------------------
// BIM property binding — shared value/row/schema types (BIM_BINDING_ARCHITECTURE.md §4.2)
//
// Defined in this low-level header (not in TilesConverter) because
// MeshInstance/MergedMeshGroup/GridCell carry them across the
// TilesConverter -> TileBuilder -> TileDataTypes dependency chain.
// This module must NOT depend on assimp, hence the opaque node pointer
// (const void*) on MeshInstance — the binding strategy layer in
// TilesConverter owns the aiNode interpretation.
// ---------------------------------------------------------------------------

// Single property value. Widening order: Bool < Int32 < Int64 < Double < String.
struct BimValue
{
    enum class Type { Bool, Int32, Int64, Double, Vec3, String };

    Type type = Type::String;

    bool        b = false;
    int32_t     i32 = 0;
    int64_t     i64 = 0;
    double      d = 0.0;
    double      v3[3] = { 0.0, 0.0, 0.0 };
    std::string s;

    static BimValue MakeBool(bool x)    { BimValue r; r.type = Type::Bool;   r.b = x;   return r; }
    static BimValue MakeInt32(int32_t x){ BimValue r; r.type = Type::Int32;  r.i32 = x; return r; }
    static BimValue MakeInt64(int64_t x){ BimValue r; r.type = Type::Int64;  r.i64 = x; return r; }
    static BimValue MakeDouble(double x){ BimValue r; r.type = Type::Double; r.d = x;  return r; }
    static BimValue MakeVec3(double x, double y, double z)
                                            { BimValue r; r.type = Type::Vec3; r.v3[0]=x; r.v3[1]=y; r.v3[2]=z; return r; }
    static BimValue MakeString(std::string x){ BimValue r; r.type = Type::String; r.s = std::move(x); return r; }
};

// One property row: ordered (key, value) pairs, byte-order-stable.
using BimPropertyRow = std::vector<std::pair<std::string, BimValue>>;

// Column schema inferred from rows (or provided explicitly).
struct PropertySchemaField
{
    std::string  name;
    BimValue::Type type = BimValue::Type::String;
    bool         nullable = false;   // true if any row lacks this key
};
struct PropertySchema
{
    std::vector<PropertySchemaField> fields;
    const PropertySchemaField* Find(const std::string& name) const
    {
        for (auto& f : fields) if (f.name == name) return &f;
        return nullptr;
    }
};

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

    // ---- BIM feature identity (filled when binding enabled; see BIM doc §4.2) ----
    // Opaque owning-scene node pointer. Valid while the aiScene is alive during
    // Convert(); interpreted by the binding strategy layer (TilesConverter only).
    const void*   identityNode = nullptr;
    std::string   objectName;                          // "$AssimpFbx$" tail stripped
    std::string   meshName;                            // aiMesh::mName
    std::shared_ptr<const BimPropertyRow> properties; // deduplicated; nullptr = none

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

    // ---- BIM feature binding: per-vertex batch IDs (one per vertex,
    // parallel to positions; empty when binding disabled) ----
    std::vector<uint32_t> batchIds;
};

// ---------------------------------------------------------------------------
// Per-tile feature batch: deduplicated property rows for one GridCell.
// row index == batchId within the tile (BIM doc §4.2 FeatureBatchTable).
// ---------------------------------------------------------------------------
struct FeatureBatchTable
{
    std::vector<std::shared_ptr<const BimPropertyRow>> rows;
    std::shared_ptr<const BimPropertyRow> emptyRow;    // lazily created shared empty row

    MESH_PROJECTION_API uint32_t AssignBatchId(const std::shared_ptr<const BimPropertyRow>& row);
    size_t   size() const { return rows.size(); }

private:
    // hash -> candidate row indices. Dedup must be O(1) amortized (review
    // #12): the previous full linear scan re-hashed every stored row on each
    // call, making a 60k-instance cell quadratic (~50s). rows must only be
    // mutated through AssignBatchId (nothing else does).
    std::unordered_map<size_t, std::vector<uint32_t>> m_hashIndex;
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
    FeatureBatchTable featureBatch;      // populated when binding enabled (BIM doc)
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
