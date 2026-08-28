// Copyright Johnlyon
//
// IVendorHandler — abstraction layer for vendor-specific OSGB processing
//
// ContextCapture and DJI Terra use different:
//   - Directory structures (Data/Tile_* vs terra_osgbs/Block_*)
//   - Tile naming conventions (_Lxx_xxxxxx vs level_xx/N.osgb)
//   - LOD hierarchy models (quadtree sub-index vs flat per-level)
//   - Metadata XML element names (SRS vs SpatialReferenceSystem)
//
// This interface isolates vendor differences so the conversion pipeline
// operates on a uniform OSGBTileData stream regardless of source.
//

#pragma once

#include "macro.h"
#include "OSGBTileData.h"

#include <string>
#include <vector>
#include <memory>

class CProjectionEngine;
struct OSGBConverterOptions;
struct GridCell;

// ---------------------------------------------------------------------------
// Abstract vendor handler
// ---------------------------------------------------------------------------
class OSGB_CONVERTER_API IVendorHandler
{
public:
    virtual ~IVendorHandler() = default;

    // --- Tile discovery ---
    // Scan the input directory and return all tile paths (relative to rootDir).
    virtual std::vector<std::string> DiscoverTiles(const std::string& rootDir) = 0;

    // --- Tile metadata extraction ---
    // Parse LOD level, grid coordinates, and sub-tile index from a tile path.
    // Returns true if successful.
    virtual bool ParseTilePath(const std::string& path,
                               int& lodLevel, int& gridX, int& gridY,
                               std::string& subTileIndex) = 0;

    // --- Hierarchy building ---
    // Build a GridCell tree from loaded tiles. The root is a virtual container;
    // its children are the top-level tiles with their own children attached.
    // Takes a mutable vector: BuildHierarchy may call EnsureGroups() to migrate
    // legacy single-group tiles into the multi-group representation.
    virtual std::unique_ptr<GridCell> BuildHierarchy(
        std::vector<OSGBTileData>& tiles,
        const OSGBConverterOptions& opts) = 0;

    // --- Metadata parsing ---
    // Parse vendor-specific metadata.xml elements.
    // Called for each XML element; returns true if handled.
    virtual bool ParseMetadataElement(const std::string& elementName,
                                      const std::string& elementText,
                                      OSGBMetadata& metadata) = 0;

    // --- Projection setup ---
    // Configure CProjectionEngine based on parsed metadata.
    // Returns true if projection was successfully configured.
    virtual bool ConfigureProjection(const OSGBMetadata& metadata,
                                     const std::string& prjFile,
                                     CProjectionEngine& engine,
                                     bool verbose) = 0;

    // --- Vendor identity ---
    virtual DataVendor GetVendor() const = 0;
    virtual const char* GetVendorName() const = 0;
};

// ---------------------------------------------------------------------------
// Factory: auto-detect vendor from directory structure
// ---------------------------------------------------------------------------
class OSGB_CONVERTER_API VendorHandlerFactory
{
public:
    // Detect vendor from rootDir and create the appropriate handler.
    // Returns nullptr if vendor cannot be determined.
    static std::unique_ptr<IVendorHandler> Create(const std::string& rootDir);

    // Create a handler for a known vendor type.
    static std::unique_ptr<IVendorHandler> Create(DataVendor vendor);
};