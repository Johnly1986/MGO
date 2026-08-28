// Copyright Johnlyon
//
// TilesConverter — Assimp scene → 3D Tiles (b3dm + tileset.json) converter
//
// Pipeline:
//   1. CollectMeshInstances() — flatten scene, bake world transforms + bboxes
//   2. BuildGridHierarchy()  — bottom-up sparse grid aggregation
//   3. TilesetWriter writes b3dm + tileset.json via TileBuilder module
//
// Coordinate system flow:
//
//   Input: Assimp scene (Y-up, right-handed: X=East, Y=Up, Z=South)
//     |
//     +-- CollectMeshInstances():
//     |    World-space bboxes computed in Assimp Y-up
//     |
//     +-- CProjectionEngine::ApplyPerInstanceProjectionCorrection():
//     |    Assimp centroid -> AxisMapper::AssimpToENU -> projected -> geographic
//     |    -> ECEF -> delta in ENU -> AxisMapper::ENUToAssimp -> Assimp translation
//     |    (delta applied to MeshInstance worldTransform + bbox)
//     |
//     +-- MaterialGrouper::GroupCellByMaterial():
//     |    Assimp Y-up vertices -> glTF buffer (if inputIsZUp: Z-up -> Y-up
//     |    via YZ-swap first)
//     |
//     +-- BBoxUtils::WriteBoxJson():
//     |    Assimp Y-up bbox (East,Up,South) -> BBoxAssimpToTilesZUp ->
//     |    3D Tiles Z-up bbox (East,-South,Up) = (East,North,Up) = ENU
//     |
//     +-- CProjectionEngine::ComputeRootTransform():
//          ENU->ECEF 4x4 column-major matrix. Does NOT negate the North column —
//          CesiumJS Y_UP_TO_Z_UP maps Assimp (East,Up,South) -> (East,North,Up),
//          which is standard ENU consumed directly by the root transform.
//
//   Output: 3D Tiles Z-up b3dm + tileset.json
//
// Modular dependencies:
//   - TileBuilder:     GlbBuilder, B3dmBuilder, MaterialGrouper, BBoxUtils, TilesetWriter
//   - CProjectionEngine: projection, ECEF, per-instance correction, root transform
//   - AxisMapper:      coordinate axis conversion utilities (single source of truth)
//   - TileDataTypes:   MeshInstance, MergedMeshGroup, BinaryBlob
//

#pragma once

#include "macro.h"
#include "../MeshProjectionErrorCorrector/SimplifyOptions.h"
#include "../MeshProjectionErrorCorrector/GeoreferencingFactory.h"
#include "../MeshProjectionErrorCorrector/GeoreferencingWithMultiPosition.h"
#include "../MeshProjectionErrorCorrector/Constants.h"
#include <string>
#include <vector>
#include <memory>

#include "../MeshProjectionErrorCorrector/TileDataTypes.h"
#include "../TileBuilder/TileBuilder.h"
#include "../MeshProjectionErrorCorrector/CProjectionEngine.h"

struct aiScene;

// ---------------------------------------------------------------------------
// Converter options
// ---------------------------------------------------------------------------
struct TilesConverterOptions
{
    double rootGeometricError = 500.0;
    double tileGeometricError = 50.0;
    std::string refine = "ADD";
    std::string outputDir = ".";
    std::string tileBaseName = "tile";
    bool inputIsZUp = false;
    double minBlockDistance = TileConstants::DEFAULT_MIN_BLOCK_DISTANCE;
    int    maxLODLevels = 5;
    std::string prjFile;
    double originX = 0;
    double originY = 0;
    double originZ = 0;
    std::string fbxDirectory;
    bool doubleSided = false;

    // Mesh simplification options (meshoptimizer)
    SimplifyOptions simplify;  // disabled by default (error=0)

    // Georeferencing (local→projected coordinate transform)
    GeoreferencingType georefType = GeoreferencingType::None;
    std::string cpsFile;              // control points CSV (for multipos)
    double helmert[7] = {0};          // 7-parameter: mx,my,mz,rx,ry,rz,scale
    std::vector<ControlPoint> controlPoints;  // parsed control points
    FitMethod fitMethod = FitMethod::ECEF_Affine;
    int  polyOrder = 1;              // polynomial order for DirectPoly2D

    // Projection correction mode:
    //   false (default): per-instance (centroid-based) - delta computed at vertex
    //     centroid, applied as uniform translation. Approximation with residual
    //     proportional to model extent.
    //   true: per-vertex - delta computed for EACH vertex independently,
    //     eliminating all curvature residual. More expensive (O(N) GK inverse
    //     calls). Modifies aiScene vertices in-place.
    bool perVertexProjectionCorrection = false;
};

// ---------------------------------------------------------------------------
// Main converter class
// ---------------------------------------------------------------------------
class TILES_CONVERTER_API TilesConverter
{
public:
    TilesConverter();
    ~TilesConverter();

    bool Convert(const aiScene* scene, const TilesConverterOptions& options);

    const std::string& GetTilesetJson() const { return m_tilesetJson; }
    int GetTileCount() const;

private:
    bool CollectMeshInstances(const aiScene* scene);
    bool BuildGridHierarchy(const aiScene* scene);

    TilesConverterOptions m_opts;
    CProjectionEngine m_projEngine;
    std::unique_ptr<GridCell> m_gridRoot;
    std::vector<MeshInstance> m_instances;
    std::string m_tilesetJson;
    int m_tileCount = 0;
};
