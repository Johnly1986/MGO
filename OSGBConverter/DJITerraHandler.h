// Copyright Johnlyon
//
// DJITerraHandler — vendor-specific logic for DJI Terra OSGB data
//
// Directory:  terra_osgbs/Block_*/Block_*.osgb + level_*/*.osgb
// LOD model:  level_XX directory encodes LOD; files are numbered (1.osgb, 2.osgb...)
// Hierarchy:  Block root → level_* tiles as direct children
//

#pragma once

#include "IVendorHandler.h"

class OSGB_CONVERTER_API DJITerraHandler : public IVendorHandler
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
    DataVendor GetVendor() const override { return DataVendor::DJITerra; }
    const char* GetVendorName() const override { return "DJI Terra"; }
};