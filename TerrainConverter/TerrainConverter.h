#pragma once

#include "macro.h"
#include "../MeshProjectionErrorCorrector/SimplifyOptions.h"
#include "../MeshProjectionErrorCorrector/GeoreferencingFactory.h"
#include "../MeshProjectionErrorCorrector/GeoreferencingWithMultiPosition.h"
#include "HeightmapGrid.h"
#include "GeoTiffReader.h"
#include "TerrainQuadtree.h"
#include "TinSimplifier.h"
#include "QuantizedMeshEncoder.h"
#include "TerrainLayerJson.h"
#include <string>
#include <vector>

// TerrainConverterOptions — options for TIF → terrain tile conversion
struct TerrainConverterOptions
{
    std::string inputTif;
    std::string outputDir;
    std::string prjFile;            // optional: override TIF-embedded projection

    // Optional: override TIF tiepoint with this origin (projected coords in same CRS as prjFile)
    bool   hasOrigin = false;
    double originX = 0.0;
    double originY = 0.0;
    double originZ = 0.0;

    int    maxLODLevels      = -1;   // -1 = auto-compute from TIF resolution
    int    samplesPerTile    = 65;
    SimplifyOptions simplify = { 0.001f, 0.0f };

    bool   writeOctVertexNormals = true;
    bool   writeWaterMask        = false;

    // Debug
    bool   verbose = false;

    // Georeferencing (optional — defaults to None = plain projection)
    GeoreferencingType georefType = GeoreferencingType::None;
    double helmert[7] = {0};         // 7-parameter: mx,my,mz,rx,ry,rz,scale
    std::vector<ControlPoint> controlPoints;  // for MultiPosition
    FitMethod fitMethod = FitMethod::ECEF_Affine;
    int  polyOrder = 1;             // polynomial order for DirectPoly2D
};

// TerrainConverter — main orchestration class
//
// Pipeline:
//   1. Read GeoTIFF → HeightmapGrid
//   2. Build quadtree (geographic tiling)
//   3. For each leaf tile:
//      a. Extract local heightmap
//      b. Simplify to TIN (meshoptimizer)
//      c. Quantize vertices to (u, v, height)
//      d. Encode to quantized-mesh binary
//      e. Write {z}/{x}/{y}.terrain
//   4. Generate layer.json
//
class TERRAIN_CONVERTER_API TerrainConverter
{
public:
    TerrainConverter();
    ~TerrainConverter();

    bool Convert(const TerrainConverterOptions& opts);

private:
    bool ProcessTile(const TerrainTile& tile,
                     const TerrainConverterOptions& opts,
                     CProjectionEngine& proj,
                     bool hasProj,
                     float globalMinH,
                     float globalMaxH);

    bool WriteLayerJson(const TerrainConverterOptions& opts,
                        const TerrainQuadtree& quadtree);
};
