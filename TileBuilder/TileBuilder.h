// Copyright Johnlyon
//
// TileBuilder — 3D Tiles binary format builder + tileset.json generator
//
// Migrated from TilesConverter.cpp. Provides:
//   - GlbBuilder:      binary glTF (.glb) construction from MergedMeshGroup
//   - B3dmBuilder:     Batched 3D Model (.b3dm) wrapping
//   - MaterialGrouper: per-cell material grouping and merging
//   - TilesetWriter:   tileset.json generation with hierarchical bounding volumes
//   - BBoxUtils:       bounding box helpers (union, JSON, diagonal)
//

#pragma once

#include "macro.h"
#include "../MeshProjectionErrorCorrector/TileDataTypes.h"
#include "../MeshProjectionErrorCorrector/AxisMapper.h"
#include "../MeshProjectionErrorCorrector/Constants.h"

#include <string>
#include <vector>
#include <sstream>
#include <memory>
#include <cstdint>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>

struct aiScene;
struct aiMesh;
struct aiMaterial;

// GridCell is now defined in ../MeshProjectionErrorCorrector/TileDataTypes.h

// ---------------------------------------------------------------------------
// TileBuilder options
// ---------------------------------------------------------------------------
struct TileBuildOptions
{
    std::string outputDir = ".";
    std::string tileBaseName = "tile";
    std::string refine = "ADD";
    std::string fbxDirectory;
    double rootGeometricError = 500.0;
    double tileGeometricError = 50.0;
    bool inputIsZUp = false;
    bool doubleSided = false;
    double minBlockDistance = TileConstants::DEFAULT_MIN_BLOCK_DISTANCE;
    int maxLODLevels = 5;
    // Projection params (for GroupCellByMaterial ECEF conversion)
    bool hasProjection = false;
    double originX = 0, originY = 0, originZ = 0;
};

// ---------------------------------------------------------------------------
// PropertySchemaField/PropertySchema/BimValue/BimPropertyRow are defined in
// ../MeshProjectionErrorCorrector/TileDataTypes.h (imported above).
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// BatchTableWriter — PropertySchema + rows -> Batch Table JSON/binary
// (BIM_BINDING_ARCHITECTURE.md §4.3-⑥). Column three-state rule:
//   all rows non-null numeric/Vec3 -> binary column;
//   any null / string / bool      -> JSON array (null allowed);
//   all null                      -> column omitted.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// GlbBuilder — binary glTF construction
// ---------------------------------------------------------------------------
class TILE_BUILDER_API GlbBuilder
{
public:
    // Optional BIM batch-id context: when provided with batchLength > 0, a
    // _BATCHID vertex attribute is appended per primitive. The per-vertex
    // values themselves travel inside MergedMeshGroup::batchIds (populated by
    // GroupCellByMaterial from MeshInstance::properties); batchLength (the
    // tile's feature-row count) only drives the componentType ladder
    // 5121/5123/5126 — UNSIGNED_INT(5125) is forbidden for vertex attributes.
    struct BatchIdContext
    {
        uint32_t batchLength = 0;   // rows in the tile's feature batch (0 = disabled)
    };

    static bool Build(const std::vector<MergedMeshGroup>& groups,
                      BinaryBlob& outGlb,
                      const std::string& textureBaseDir,
                      bool doubleSided,
                      const BatchIdContext* bim = nullptr);
};

// ---------------------------------------------------------------------------
// B3dmBuilder — Batched 3D Model wrapping
// ---------------------------------------------------------------------------
class TILE_BUILDER_API B3dmBuilder
{
public:
    // Legacy signature: BATCH_LENGTH=0, no batch table (byte-identical when
    // binding disabled).
    static bool Build(const BinaryBlob& glb, BinaryBlob& outB3dm);

    // BIM signature: writes FeatureTable JSON with BATCH_LENGTH + optional
    // Batch Table. All sections end on 8-byte boundaries measured from the
    // tile start (absolute rule — see implementation note); header length
    // fields include chunk padding so consumers can sum them to find glb.
    static bool Build(const BinaryBlob& glb,
                      uint32_t batchLength,
                      const std::string& batchTableJson,
                      const std::vector<uint8_t>& batchTableBinary,
                      BinaryBlob& outB3dm);

    static void PadTo8(std::vector<uint8_t>& buf);
};

// ---------------------------------------------------------------------------
// BatchTableWriter — schema + rows -> Batch Table (JSON + binary)
// ---------------------------------------------------------------------------
class TILE_BUILDER_API BatchTableWriter
{
public:
    // Writes the Batch Table for one tile.
    //   rows         — deduplicated feature rows (row i == batchId i)
    //   outNullified — optional count of NaN/±Inf property values nulled out
    //     (columns containing them are demoted to JSON per BIM doc §4.3-⑧ /
    //     §4.8(3); legacy Batch Table has no INT64 binary type, so Int64
    //     values beyond int32 range stay JSON as exact digits).
    // Binary column mapping (doc §4.8(3)):
    //   Int32 / Int64-fits-int32 -> INT(5124); Double -> DOUBLE(5128);
    //   Vec3 -> VEC3/FLOAT(5126); Bool / String / any column with nulls or
    //   non-finite values -> JSON array.
    // Returns false on internal errors (empty schema with non-empty rows).
    static bool Build(const std::vector<std::shared_ptr<const BimPropertyRow>>& rows,
                      std::string& outJson,
                      std::vector<uint8_t>& outBinary,
                      size_t* outNullified = nullptr);
};

// ---------------------------------------------------------------------------
// MaterialGrouper — per-cell material grouping and merging
// ---------------------------------------------------------------------------
class TILE_BUILDER_API MaterialGrouper
{
public:
    // Group instances in a GridCell by material, convert to ECEF, merge groups
    static void GroupCellByMaterial(GridCell& cell, const aiScene* scene,
                                    const TileBuildOptions& opts);

    // Merge MergedMeshGroup entries with identical baseColorFactor + texturePath
    static void MergeGroupsByMaterial(std::vector<MergedMeshGroup>& groups);
};

// ---------------------------------------------------------------------------
// BBoxUtils — bounding box helpers
// ---------------------------------------------------------------------------
class TILE_BUILDER_API BBoxUtils
{
public:
    // Build 3D Tiles "boundingVolume": {"box": [...]} JSON value.
    // Always converts Y-up bbox -> 3D Tiles Z-up via CoordinateTransform::ConvertBBox.
    // GroupCellByMaterial normalizes Z-up input to Y-up first, so bbox is always Y-up here.
    static nlohmann::ordered_json WriteBoxJson(const double* bmin, const double* bmax);

    // Bounding box diagonal length
    static double Diagonal(const double* bmin, const double* bmax);

    // Bottom-up bbox union: propagate children's bboxes into parent cells
    static void UpdateGridCellBBoxes(GridCell& cell);
};

// ---------------------------------------------------------------------------
// TilesetWriter — tileset.json generation
// ---------------------------------------------------------------------------
class TILE_BUILDER_API TilesetWriter
{
public:
    // Generate root tileset.json + external sub-tilesets
    // rootTransform: nullable ENU->ECEF 4x4 (column-major, .data() order).
    static bool Generate(const GridCell& root,
                         const TileBuildOptions& opts,
                         const Eigen::Matrix4d* rootTransform,
                         std::string& outJson);

    // Write all .b3dm files to disk; returns false if any tile write failed
    static bool WriteTiles(GridCell& cell, const aiScene* scene,
                           const TileBuildOptions& opts,
                           const std::string& subdir = "");

    // Count content cells in a subtree
    static int CountDescendantContent(const GridCell& cell);

private:
    static bool WriteSubtreeTileset(const GridCell& cell,
                                    const TileBuildOptions& opts,
                                    const std::string& subdir);
    static nlohmann::ordered_json WriteNodeRecursive(const GridCell& cell,
                                                     double parentGeomErr,
                                                     const std::string& b3dmRelBase,
                                                     const std::string& refine);
    static double ComputeCellGeometricErrors(const GridCell& cell,
                                             double parentGeomErr);
    static void CollectContentSubtree(const GridCell& cell,
                                      std::vector<const GridCell*>& out);
    static void CollectContentCells(GridCell& cell,
                                    std::vector<GridCell*>& out);
};
