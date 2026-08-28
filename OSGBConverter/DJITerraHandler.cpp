// Copyright Johnlyon
//
// DJITerraHandler implementation
//

#include "DJITerraHandler.h"
#include "MetadataParser.h"
#include "PlatformCompat.h"
#include "OSGBCellBuilder.h"
#include "../MeshProjectionErrorCorrector/CProjectionEngine.h"
#include "../MeshProjectionErrorCorrector/TileDataTypes.h"
#include "OSGBConverter.h"

#include <cstring>
#include <cctype>
#include <algorithm>
#include <iostream>
#include <map>
#include <functional>

// ---------------------------------------------------------------------------
// Tile discovery
// ---------------------------------------------------------------------------

std::vector<std::string> DJITerraHandler::DiscoverTiles(const std::string& rootDir)
{
    std::vector<std::string> tiles;

    auto entries = DirectoryLister::List(rootDir);
    for (auto& entry : entries)
    {
        if (!entry.isDirectory) continue;
        if (entry.name.find("Block_") != 0) continue;

        std::string blockPath = rootDir + "/" + entry.name;
        auto blockEntries = DirectoryLister::List(blockPath);
        for (auto& be : blockEntries)
        {
            if (be.name == "." || be.name == "..") continue;
            std::string fullPath = blockPath + "/" + be.name;

            if (be.name.find("level_") == 0 && be.isDirectory)
            {
                auto levelEntries = DirectoryLister::List(fullPath);
                for (auto& le : levelEntries)
                {
                    if (le.isFile && le.name.size() >= 5 &&
                        le.name.compare(le.name.size() - 5, 5, ".osgb") == 0)
                    {
                        tiles.push_back(entry.name + "/" + be.name + "/" + le.name);
                    }
                }
            }
            else if (be.isFile && be.name.size() >= 5 &&
                     be.name.compare(be.name.size() - 5, 5, ".osgb") == 0)
            {
                tiles.push_back(entry.name + "/" + be.name);
            }
        }
    }

    return tiles;
}

// ---------------------------------------------------------------------------
// Tile path parsing (DJI: level_XX directory encodes LOD)
// ---------------------------------------------------------------------------

bool DJITerraHandler::ParseTilePath(const std::string& path,
                                     int& lodLevel, int& gridX, int& gridY,
                                     std::string& subTileIndex)
{
    lodLevel = 0; gridX = 0; gridY = 0;
    subTileIndex.clear();

    size_t lastSlash = path.find_last_of("/\\");
    std::string filename = (lastSlash != std::string::npos)
        ? path.substr(lastSlash + 1) : path;

    if (lastSlash != std::string::npos)
    {
        size_t prevSlash = path.find_last_of("/\\", lastSlash - 1);
        std::string parentDir = (prevSlash != std::string::npos)
            ? path.substr(prevSlash + 1, lastSlash - prevSlash - 1)
            : path.substr(0, lastSlash);

        if (parentDir.find("level_") == 0)
        {
            lodLevel = 0;
            size_t p = 6;
            while (p < parentDir.size() && std::isdigit(parentDir[p]))
                { lodLevel = lodLevel * 10 + (parentDir[p] - '0'); ++p; }

            // Use filename (without .osgb) as sub-tile index
            if (filename.size() > 5)
                subTileIndex = filename.substr(0, filename.size() - 5);
            return true;
        }

        if (parentDir.find("Block_") == 0)
        {
            lodLevel = 0;
            return true;
        }
    }

    return true;
}

// ---------------------------------------------------------------------------
// Metadata XML element parsing (DJI element names)
// ---------------------------------------------------------------------------

bool DJITerraHandler::ParseMetadataElement(const std::string& elementName,
                                            const std::string& elementText,
                                            OSGBMetadata& metadata)
{
    if (elementName == "SRS" || elementName == "srs" ||
        elementName == "SpatialReferenceSystem" || elementName == "SRSName" ||
        elementName == "srsname")
    {
        metadata.srs = elementText;
        metadata.projectionMode = MetadataParser::DetectProjectionMode(elementText);
        if (MetadataParser::ParseENUOrigin(elementText, metadata.enuLat, metadata.enuLon))
        {
            metadata.isENU = true;
            metadata.projectionMode = ProjectionMode::PerTile;
        }
        return true;
    }

    if (elementName == "SRSOrigin" || elementName == "srsorigin" ||
        elementName == "Origin" || elementName == "origin")
    {
        MetadataParser::ParseOrigin(elementText, metadata.originX,
                                     metadata.originY, metadata.originZ);
        return true;
    }

    if (elementName == "Path" || elementName == "path")
    {
        std::string path = elementText;
        size_t first = path.find_first_not_of(" \t\n\r");
        size_t last  = path.find_last_not_of(" \t\n\r");
        if (first != std::string::npos && last != std::string::npos)
            path = path.substr(first, last - first + 1);
        if (!path.empty())
            metadata.tilePaths.push_back(path);
        return true;
    }

    return false;
}

// ---------------------------------------------------------------------------
// Projection configuration
// ---------------------------------------------------------------------------

bool DJITerraHandler::ConfigureProjection(const OSGBMetadata& metadata,
                                           const std::string& prjFile,
                                           CProjectionEngine& engine,
                                           bool verbose)
{
    if (metadata.isENU)
    {
        engine.SetOrigin(metadata.originX, metadata.originY, metadata.originZ);
        return true;
    }

    if (metadata.HasProjection())
    {
        if (!prjFile.empty())
        {
            engine.LoadProjection(prjFile);
        }
        else
        {
            int epsgCode = MetadataParser::ParseEPSGCode(metadata.srs);
            if (epsgCode > 0 && MetadataParser::ConfigureGKProjection(engine, epsgCode))
            {
                if (verbose)
                    std::cout << "[OSGBConverter] Auto-configured GK from "
                              << metadata.srs << std::endl;
            }
        }

        if (metadata.HasOrigin())
            engine.SetOrigin(metadata.originX, metadata.originY, metadata.originZ);
        return true;
    }

    return false;
}

// ---------------------------------------------------------------------------
// Hierarchy building (DJI: Block root → level_* tiles as children)
// ---------------------------------------------------------------------------

namespace {

// Derive a unique, filesystem-safe cell key from the tile's relative path.
// DJI level files are numbered (1.osgb, 2.osgb...) and repeat across blocks,
// so the bare filename is NOT unique — it must be scoped by block/level.
std::string MakeDJICellKey(const std::string& tilePath)
{
    std::string key = tilePath;
    if (key.size() > 5 && key.compare(key.size() - 5, 5, ".osgb") == 0)
        key.resize(key.size() - 5);
    std::replace(key.begin(), key.end(), '/', '_');
    std::replace(key.begin(), key.end(), '\\', '_');
    return key;
}

std::unique_ptr<GridCell> BuildDJIGridCell(OSGBTileData& tile)
{
    return BuildGridCellFromTile(tile, MakeDJICellKey(tile.tilePath));
}

} // anonymous namespace

std::unique_ptr<GridCell> DJITerraHandler::BuildHierarchy(
    std::vector<OSGBTileData>& tiles,
    const OSGBConverterOptions& opts)
{
    // Phase 1: Build Block-level hierarchy (Block root → level_* tiles)
    // Each Block keeps its native LOD structure.
    struct BlockCell {
        std::unique_ptr<GridCell> cell;
        std::string name;
        double cx, cy;  // centroid XY for spatial grouping
        double bboxMin[3], bboxMax[3];
    };
    std::vector<BlockCell> blockCells;

    std::map<std::string, std::vector<OSGBTileData*>> blockMap;
    for (auto& tile : tiles)
    {
        if (tile.IsEmpty()) continue;
        size_t slash = tile.tilePath.find('/');
        std::string blockName = (slash != std::string::npos)
            ? tile.tilePath.substr(0, slash) : tile.tilePath;
        blockMap[blockName].push_back(&tile);
    }

    double gMin[3] = { 1e30, 1e30, 1e30 };
    double gMax[3] = { -1e30, -1e30, -1e30 };

    for (auto& [blockName, blockTiles] : blockMap)
    {
        OSGBTileData* rootTile = nullptr;
        std::vector<OSGBTileData*> levelTiles;
        for (auto* t : blockTiles)
        {
            if (t->lodLevel == 0 && t->subTileIndex.empty())
                rootTile = t;
            else
                levelTiles.push_back(t);
        }
        if (!rootTile) continue;

        auto blockRoot = BuildDJIGridCell(*rootTile);
        for (auto* t : levelTiles)
        {
            auto child = BuildDJIGridCell(*t);
            child->parent = blockRoot.get();
            blockRoot->children.push_back(std::move(child));
        }

        BlockCell bc;
        bc.name = blockName;
        for (int i = 0; i < 3; ++i)
        {
            bc.bboxMin[i] = blockRoot->bboxMin[i];
            bc.bboxMax[i] = blockRoot->bboxMax[i];
            if (bc.bboxMin[i] < gMin[i]) gMin[i] = bc.bboxMin[i];
            if (bc.bboxMax[i] > gMax[i]) gMax[i] = bc.bboxMax[i];
        }
        bc.cx = (bc.bboxMin[0] + bc.bboxMax[0]) * 0.5;
        bc.cy = (bc.bboxMin[1] + bc.bboxMax[1]) * 0.5;
        bc.cell = std::move(blockRoot);
        blockCells.push_back(std::move(bc));
    }

    if (blockCells.empty()) return nullptr;

    // Phase 2: Spatial quadtree grouping over Blocks
    // DJI Blocks are adaptively sized — group them spatially so the
    // tileset.json root has a manageable number of children.
    //
    // Use a 2D grid in the XY plane (projected coordinates).
    // Cell size = average Block extent × 2, so ~4 Blocks per cell.
    double avgExtentX = 0, avgExtentY = 0;
    for (auto& bc : blockCells)
    {
        avgExtentX += (bc.bboxMax[0] - bc.bboxMin[0]);
        avgExtentY += (bc.bboxMax[1] - bc.bboxMin[1]);
    }
    avgExtentX /= blockCells.size();
    avgExtentY /= blockCells.size();
    double cellSize = std::max(avgExtentX, avgExtentY) * 2.0;
    if (cellSize < 1.0) cellSize = 100.0;

    auto root = std::make_unique<GridCell>();
    root->level = -1;

    if (blockCells.size() <= 8)
    {
        // Few blocks — attach directly to root
        for (auto& bc : blockCells)
        {
            bc.cell->parent = root.get();
            root->children.push_back(std::move(bc.cell));
        }
    }
    else
    {
        // Group Blocks into spatial cells
        std::map<std::pair<int, int>, std::vector<BlockCell*>> spatialCells;
        for (auto& bc : blockCells)
        {
            int ix = static_cast<int>(std::floor((bc.cx - gMin[0]) / cellSize));
            int iy = static_cast<int>(std::floor((bc.cy - gMin[1]) / cellSize));
            spatialCells[{ix, iy}].push_back(&bc);
        }

        // Create a parent GridCell for each spatial cell with >1 Block
        int cellIdx = 0;
        for (auto& [key, cellBlocks] : spatialCells)
        {
            if (cellBlocks.size() == 1)
            {
                // Single Block — attach directly to root
                cellBlocks[0]->cell->parent = root.get();
                root->children.push_back(std::move(cellBlocks[0]->cell));
            }
            else
            {
                // Multiple Blocks — create a spatial parent cell
                auto parentCell = std::make_unique<GridCell>();
                parentCell->level = 0;
                parentCell->hasContent = false;  // container only

                char keyBuf[128];
                snprintf(keyBuf, sizeof(keyBuf), "spatial_%d_%d", key.first, key.second);
                parentCell->cellKey = keyBuf;

                // Union bounding box from children
                for (int i = 0; i < 3; ++i)
                {
                    parentCell->bboxMin[i] = 1e30;
                    parentCell->bboxMax[i] = -1e30;
                    parentCell->localBboxMin[i] = 1e30;
                    parentCell->localBboxMax[i] = -1e30;
                }
                for (auto* bc : cellBlocks)
                {
                    for (int i = 0; i < 3; ++i)
                    {
                        if (bc->bboxMin[i] < parentCell->bboxMin[i])
                            parentCell->bboxMin[i] = bc->bboxMin[i];
                        if (bc->bboxMax[i] > parentCell->bboxMax[i])
                            parentCell->bboxMax[i] = bc->bboxMax[i];
                    }
                    bc->cell->parent = parentCell.get();
                    parentCell->children.push_back(std::move(bc->cell));
                }
                for (int i = 0; i < 3; ++i)
                {
                    parentCell->localBboxMin[i] = parentCell->bboxMin[i];
                    parentCell->localBboxMax[i] = parentCell->bboxMax[i];
                }

                parentCell->parent = root.get();
                root->children.push_back(std::move(parentCell));
                ++cellIdx;
            }
        }
    }

    for (int i = 0; i < 3; ++i)
    {
        root->bboxMin[i] = gMin[i];
        root->bboxMax[i] = gMax[i];
        root->localBboxMin[i] = gMin[i];
        root->localBboxMax[i] = gMax[i];
    }

    return root;
}