// Copyright Johnlyon
//
// OSGBReader — read OSGB binary files via OpenSceneGraph
//
// Uses osgDB::readNodeFile() to load .osgb files, then traverses the scene
// graph to extract geometry (positions, normals, texcoords, indices),
// textures, and material properties.
//

#pragma once

#include "macro.h"
#include "OSGBTileData.h"

#include <string>
#include <vector>
#include <memory>

class OSGB_CONVERTER_API OSGBReader
{
public:
    OSGBReader() = default;

    // Read a single OSGB tile file.
    //   tilePath: path to the .osgb file
    //   rootDir:  root directory of the OSGB dataset (for resolving texture paths)
    // Returns true on success, populates outData.
    bool ReadTile(const std::string& tilePath, const std::string& rootDir,
                  OSGBTileData& outData);

    // Read all tiles from a list of paths.
    // Uses vendor handler to parse tile metadata (LOD, grid, sub-index).
    // maxLOD > 0 skips tiles whose LOD level exceeds it (before the OSG read).
    // Returns the number of successfully loaded tiles.
    int ReadAllTiles(const std::vector<std::string>& tilePaths,
                     const std::string& rootDir,
                     class IVendorHandler& handler,
                     int maxLOD,
                     std::vector<OSGBTileData>& outTiles);

    // Get the last error message.
    const std::string& GetLastError() const { return m_lastError; }

private:
    std::string m_lastError;
};