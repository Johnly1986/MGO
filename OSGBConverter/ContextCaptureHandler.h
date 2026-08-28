// Copyright Johnlyon
//
// ContextCaptureHandler — vendor-specific logic for ContextCapture OSGB data
//
// Directory:  Data/Tile_+XXX_+YYY/Tile_+XXX_+YYY[_Lxx_xxxxxx].osgb
// LOD model:  quadtree sub-tile index (accumulated path of 0-3 digits)
// Hierarchy:  parent subIdx = child subIdx without last character
//

#pragma once

#include "IVendorHandler.h"

class OSGB_CONVERTER_API ContextCaptureHandler : public IVendorHandler
{
public:
    std::vector<std::string> DiscoverTiles(const std::string& rootDir) override;
    bool ParseTilePath(const std::string& path, int& lodLevel, int& gridX, int& gridY,
                       std::string& subTileIndex) override;
    std::unique_ptr<GridCell> BuildHierarchy(std::vector<OSGBTileData>& tiles,
                                              const OSGBConverterOptions& opts) override;
    bool ParseMetadataElement(const std::string& elementName, const std::string& elementText,
                              OSGBMetadata& metadata) override;
    bool ConfigureProjection(const OSGBMetadata& metadata, const std::string& prjFile,
                              CProjectionEngine& engine, bool verbose) override;
    DataVendor GetVendor() const override { return DataVendor::ContextCapture; }
    const char* GetVendorName() const override { return "ContextCapture"; }
};