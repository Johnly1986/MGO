// Copyright Johnlyon
//
// ContextCaptureHandler implementation
//

#include "ContextCaptureHandler.h"
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

std::vector<std::string> ContextCaptureHandler::DiscoverTiles(const std::string& rootDir)
{
    std::vector<std::string> tiles;

    std::string dataDir = rootDir + "/Data";
    if (!DirectoryLister::IsDirectory(dataDir))
        dataDir = rootDir;

    auto entries = DirectoryLister::List(dataDir);
    for (auto& entry : entries)
    {
        if (!entry.isDirectory) continue;
        if (entry.name.find("Tile_") != 0) continue;

        std::string tileDirPath = dataDir + "/" + entry.name;
        auto tileEntries = DirectoryLister::List(tileDirPath);
        for (auto& te : tileEntries)
        {
            if (!te.isFile) continue;
            if (te.name.size() < 5 ||
                te.name.compare(te.name.size() - 5, 5, ".osgb") != 0)
                continue;

            tiles.push_back("Data/" + entry.name + "/" + te.name);
        }
    }

    return tiles;
}

// ---------------------------------------------------------------------------
// Tile path parsing
// ---------------------------------------------------------------------------

bool ContextCaptureHandler::ParseTilePath(const std::string& path,
                                           int& lodLevel, int& gridX, int& gridY,
                                           std::string& subTileIndex)
{
    lodLevel = 0; gridX = 0; gridY = 0;
    subTileIndex.clear();

    size_t lastSlash = path.find_last_of("/\\");
    std::string filename = (lastSlash != std::string::npos)
        ? path.substr(lastSlash + 1) : path;

    // Parse grid coordinates from directory name: Tile_[+-]XXX_[+-]YYY
    std::string dirName;
    if (lastSlash != std::string::npos)
    {
        size_t prevSlash = path.find_last_of("/\\", lastSlash - 1);
        dirName = (prevSlash != std::string::npos)
            ? path.substr(prevSlash + 1, lastSlash - prevSlash - 1)
            : path.substr(0, lastSlash);
    }

    size_t tilePos = dirName.find("Tile_");
    if (tilePos != std::string::npos)
    {
        auto parseSignedInt = [](const std::string& s, size_t& pos, int& val) -> bool {
            val = 0;
            int sign = 1;
            if (pos < s.size() && s[pos] == '+') { ++pos; }
            else if (pos < s.size() && s[pos] == '-') { sign = -1; ++pos; }
            if (pos >= s.size() || !std::isdigit(s[pos])) return false;
            while (pos < s.size() && std::isdigit(s[pos]))
                { val = val * 10 + (s[pos] - '0'); ++pos; }
            val *= sign;
            return true;
        };

        size_t p = tilePos + 5;
        if (parseSignedInt(dirName, p, gridX) && p < dirName.size() && dirName[p] == '_')
        {
            ++p;
            parseSignedInt(dirName, p, gridY);
        }
    }

    // Parse LOD: _L<level>_<subIndex>
    size_t lPos = filename.find("_L");
    if (lPos == std::string::npos)
    {
        lodLevel = 0;
        return true;
    }

    lPos += 2;
    if (lPos >= filename.size() || !std::isdigit(filename[lPos]))
        return false;

    lodLevel = 0;
    while (lPos < filename.size() && std::isdigit(filename[lPos]))
    {
        lodLevel = lodLevel * 10 + (filename[lPos] - '0');
        ++lPos;
    }

    // Parse sub-tile index after _L<level>_
    if (lPos < filename.size() && filename[lPos] == '_')
    {
        ++lPos;
        size_t end = filename.find(".osgb", lPos);
        if (end == std::string::npos) end = filename.size();
        // Keep the FULL sub-tile index including any texture-variant suffix
        // (t1, t2, ...). ContextCapture splits a sub-tile whose texture atlas
        // exceeds the size limit into sibling files (base + tN variants) that
        // each carry a disjoint part of the geometry with its own texture.
        // Stripping the suffix here would make these sibling files share one
        // hierarchy key and silently drop all but the last one.
        subTileIndex = filename.substr(lPos, end - lPos);
    }

    return true;
}

// ---------------------------------------------------------------------------
// Metadata XML element parsing
// ---------------------------------------------------------------------------

bool ContextCaptureHandler::ParseMetadataElement(const std::string& elementName,
                                                  const std::string& elementText,
                                                  OSGBMetadata& metadata)
{
    if (elementName == "SRS" || elementName == "srs")
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

    if (elementName == "SRSOrigin" || elementName == "srsorigin")
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

bool ContextCaptureHandler::ConfigureProjection(const OSGBMetadata& metadata,
                                                 const std::string& prjFile,
                                                 CProjectionEngine& engine,
                                                 bool verbose)
{
    if (metadata.isENU)
    {
        engine.SetOrigin(metadata.originX, metadata.originY, metadata.originZ);
        return true;
    }

    if (metadata.HasProjection() && metadata.projectionMode == ProjectionMode::RootOnly)
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

    if (!prjFile.empty())
    {
        engine.LoadProjection(prjFile);
        if (metadata.HasOrigin())
            engine.SetOrigin(metadata.originX, metadata.originY, metadata.originZ);
        return true;
    }

    return false;
}

// ---------------------------------------------------------------------------
// Hierarchy building (CC: quadtree sub-tile index matching)
// ---------------------------------------------------------------------------

namespace {

std::unique_ptr<GridCell> BuildCCGridCell(OSGBTileData& tile)
{
    size_t lastSlash = tile.tilePath.find_last_of("/\\");
    std::string baseName = (lastSlash != std::string::npos)
        ? tile.tilePath.substr(lastSlash + 1) : tile.tilePath;
    if (baseName.size() > 5) baseName = baseName.substr(0, baseName.size() - 5);
    return BuildGridCellFromTile(tile, baseName);
}

// Strip a trailing texture-variant suffix (t1, t2, ...) from a sub-tile
// index: "000310t2" -> "000310". The result is the quadtree position used
// for parent/child matching; the full index (with suffix) is the unique
// content identity.
std::string StripVariantSuffix(const std::string& subIdx)
{
    size_t tPos = subIdx.find('t');
    return (tPos != std::string::npos) ? subIdx.substr(0, tPos) : subIdx;
}

} // anonymous namespace

std::unique_ptr<GridCell> ContextCaptureHandler::BuildHierarchy(
    std::vector<OSGBTileData>& tiles,
    const OSGBConverterOptions& opts)
{
    auto root = std::make_unique<GridCell>();
    root->level = -1;

    double gMin[3] = { 1e30, 1e30, 1e30 };
    double gMax[3] = { -1e30, -1e30, -1e30 };

    std::map<std::string, std::map<int, std::map<std::string, OSGBTileData*>>> gridCells;
    for (auto& tile : tiles)
    {
        if (tile.IsEmpty()) continue;
        char key[64];
        snprintf(key, sizeof(key), "%d_%d", tile.tileX, tile.tileY);
        gridCells[key][tile.lodLevel][tile.subTileIndex] = &tile;
    }

    for (auto& [gridKey, lodMap] : gridCells)
    {
        if (lodMap.empty()) continue;
        int minLOD = lodMap.begin()->first;
        auto& coarsestTiles = lodMap.begin()->second;
        if (coarsestTiles.empty()) continue;

        auto& coarsestTile = coarsestTiles.begin()->second;
        auto cellRoot = BuildCCGridCell(*coarsestTile);

        // Quadtree-position -> cell map for ancestor lookup. Variant siblings
        // (base + tN) register under the same stripped position; the first
        // (base) registration wins.
        std::map<std::pair<int, std::string>, GridCell*> cellByLODSubIdx;
        cellByLODSubIdx.insert({ { coarsestTile->lodLevel,
                                   StripVariantSuffix(coarsestTile->subTileIndex) },
                                 cellRoot.get() });

        // Remaining tiles at the coarsest LOD (e.g. texture variants of the
        // root, or a split root) hang directly off the cell root - they must
        // not be dropped just because they share the lowest level.
        for (auto it = std::next(coarsestTiles.begin()); it != coarsestTiles.end(); ++it)
        {
            auto& siblingTile = it->second;
            auto siblingCell = BuildCCGridCell(*siblingTile);
            siblingCell->parent = cellRoot.get();
            cellRoot->children.push_back(std::move(siblingCell));
            cellByLODSubIdx.insert({ { siblingTile->lodLevel,
                                       StripVariantSuffix(siblingTile->subTileIndex) },
                                     cellRoot->children.back().get() });
        }

        for (auto lodIt = std::next(lodMap.begin()); lodIt != lodMap.end(); ++lodIt)
        {
            int childLOD = lodIt->first;
            for (auto& [childSubIdx, childTile] : lodIt->second)
            {
                if (childSubIdx.empty()) continue;
                GridCell* ancestor = nullptr;
                std::string baseIdx = StripVariantSuffix(childSubIdx);
                std::string searchIdx = baseIdx;
                for (int searchLOD = childLOD - 1; searchLOD >= minLOD && !ancestor; --searchLOD)
                {
                    if (searchIdx.empty()) break;
                    searchIdx = searchIdx.substr(0, searchIdx.size() - 1);
                    auto it = cellByLODSubIdx.find({searchLOD, searchIdx});
                    if (it != cellByLODSubIdx.end() && it->second)
                        ancestor = it->second;
                }
                if (!ancestor) ancestor = cellRoot.get();

                auto childCell = BuildCCGridCell(*childTile);
                childCell->parent = ancestor;
                ancestor->children.push_back(std::move(childCell));
                cellByLODSubIdx.insert({ { childLOD, baseIdx },
                                         ancestor->children.back().get() });
            }
        }

        for (int i = 0; i < 3; ++i)
        {
            if (cellRoot->bboxMin[i] < gMin[i]) gMin[i] = cellRoot->bboxMin[i];
            if (cellRoot->bboxMax[i] > gMax[i]) gMax[i] = cellRoot->bboxMax[i];
        }

        cellRoot->parent = root.get();
        root->children.push_back(std::move(cellRoot));
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