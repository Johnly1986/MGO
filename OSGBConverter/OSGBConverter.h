// Copyright Johnlyon
//
// OSGBConverter — main orchestrator for OSGB → 3D Tiles conversion
//
// Pipeline:
//   1. Parse metadata.xml → projection mode, tile inventory, origin
//   2. Initialize CProjectionEngine (LoadProjection, SetOrigin)
//   3. Read all OSGB tiles via OSGBReader
//   4. Apply projection correction (RootOnly or PerTile mode)
//   5. Build GridCell quadtree hierarchy
//   6. Top-level reconstruction + mesh simplification
//   7. Write b3dm tiles + tileset.json via TileBuilder
//

#pragma once

#include "macro.h"
#include "OSGBTileData.h"
#include "../MeshProjectionErrorCorrector/SimplifyOptions.h"
#include "../MeshProjectionErrorCorrector/GeoreferencingFactory.h"
#include "../MeshProjectionErrorCorrector/GeoreferencingWithMultiPosition.h"

#include <string>
#include <vector>
#include <map>
#include <Eigen/Dense>
#include <memory>
#include <atomic>

// Forward declarations
class CProjectionEngine;
class IVendorHandler;
struct GridCell;
struct TileBuildOptions;

// ---------------------------------------------------------------------------
// Conversion options (mirrors existing TilesConverterOptions pattern)
// ---------------------------------------------------------------------------
struct OSGBConverterOptions
{
    std::string inputDir;       // OSGB root directory (contains metadata.xml)
    std::string outputDir;      // Output directory for 3D Tiles

    // Projection
    std::string prjFile;        // Optional .prj file (WKT)
    double originX = 0;         // Override origin from metadata.xml
    double originY = 0;
    double originZ = 0;
    bool   hasOriginOverride = false;

    // ENU local tangent plane override (--enu lat,lon[,h]): forces ENU mode
    // regardless of metadata.xml, using the existing ENU pipeline
    // (ECEF root transform + per-tile correction).
    bool   hasENUOverride = false;
    double enuLat = 0;
    double enuLon = 0;
    double enuH = 0;

    // Georeferencing (optional — defaults to None = plain projection)
    GeoreferencingType georefType = GeoreferencingType::None;
    double helmert[7] = {0};         // 7-parameter: mx,my,mz,rx,ry,rz,scale
    std::vector<ControlPoint> controlPoints;  // for MultiPosition
    FitMethod fitMethod = FitMethod::ECEF_Affine;
    int  polyOrder = 1;             // polynomial order for DirectPoly2D

    // Tile options
    int    maxLOD = 0;          // 0 = auto-detect from data
    double rootGeometricError = 500.0;
    double tileGeometricError = 50.0;
    std::string refine = "REPLACE";
    std::string tileBaseName = "osgb_tile";

    // Simplification
    SimplifyOptions simplify;

    // Misc
    bool   verbose = false;
};

// ---------------------------------------------------------------------------
// OSGBConverter — main conversion class
// ---------------------------------------------------------------------------
class OSGB_CONVERTER_API OSGBConverter
{
public:
    OSGBConverter();
    ~OSGBConverter();

    // Main entry point: convert OSGB dataset to 3D Tiles
    bool Convert(const OSGBConverterOptions& opts);

    // Get the last error message
    const std::string& GetLastError() const { return m_lastError; }

private:
    // --- Pipeline stages ---

    // Stage 1: Parse metadata.xml and initialize projection
    bool InitializeProjection(const OSGBConverterOptions& opts,
                              OSGBMetadata& metadata);

    // Stage 2: Load all OSGB tiles
    bool LoadTiles(const OSGBConverterOptions& opts,
                   const OSGBMetadata& metadata,
                   std::vector<OSGBTileData>& tiles);

    // Stage 3: Apply projection correction to tiles
    void ApplyProjection(const OSGBConverterOptions& opts,
                         const OSGBMetadata& metadata,
                         std::vector<OSGBTileData>& tiles);

    // Stage 7: Write output
    bool WriteOutput(GridCell* root,
                     const OSGBConverterOptions& opts,
                     const Eigen::Matrix4d* rootTransform);

    // Simplify all GridCells in the tree using meshoptimizer
    void SimplifyGridCells(GridCell* root, const OSGBConverterOptions& opts);

    // Write b3dm tiles directly from pre-populated materialGroups
    void WriteOSGBTiles(GridCell& cell, const OSGBConverterOptions& opts,
                        const std::string& subdir, std::atomic<int>& tileCount);

    // --- Members ---
    std::unique_ptr<CProjectionEngine> m_projEngine;
    std::unique_ptr<IVendorHandler> m_vendorHandler;
    std::unique_ptr<IGeoreferencing> m_georefHolder;  // lifetime: georef→engine
    std::string m_lastError;
    std::string m_textureBaseDir;
};