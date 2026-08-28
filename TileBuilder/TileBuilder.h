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
// GlbBuilder — binary glTF construction
// ---------------------------------------------------------------------------
class TILE_BUILDER_API GlbBuilder
{
public:
    static bool Build(const std::vector<MergedMeshGroup>& groups,
                      BinaryBlob& outGlb,
                      const std::string& textureBaseDir,
                      bool doubleSided);
};

// ---------------------------------------------------------------------------
// B3dmBuilder — Batched 3D Model wrapping
// ---------------------------------------------------------------------------
class TILE_BUILDER_API B3dmBuilder
{
public:
    static bool Build(const BinaryBlob& glb, BinaryBlob& outB3dm);
    static void PadTo8(std::vector<uint8_t>& buf);
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
