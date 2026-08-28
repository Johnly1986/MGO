#include "TerrainLayerJson.h"
#include <nlohmann/json.hpp>

TerrainLayerJson::TerrainLayerJson() = default;
TerrainLayerJson::~TerrainLayerJson() = default;

void TerrainLayerJson::SetBounds(double west, double south, double east, double north)
{
    m_west = west; m_south = south; m_east = east; m_north = north;
}

void TerrainLayerJson::SetProjection(const std::string& epsg)
{
    m_projection = epsg;
}

void TerrainLayerJson::SetTileTemplate(const std::string& tmpl)
{
    m_tileTemplate = tmpl;
}

void TerrainLayerJson::AddExtension(const std::string& name)
{
    m_extensions.push_back(name);
}

void TerrainLayerJson::SetAvailableLevels(const std::vector<std::vector<TerrainQuadtree::TileRange>>& levels)
{
    m_available = levels;
}

std::string TerrainLayerJson::Generate() const
{
    nlohmann::ordered_json j;

    j["tilejsonVersion"] = "1.0.0";
    j["format"] = "quantized-mesh-1.0";
    j["version"] = "1.0.0";

    // Extensions
    if (!m_extensions.empty())
        j["extensions"] = m_extensions;

    // Tiles template
    j["tiles"] = nlohmann::ordered_json::array({ m_tileTemplate });

    // Bounds
    j["bounds"] = { m_west, m_south, m_east, m_north };

    // Max zoom — highest LOD level present (last index of the available
    // array).  CesiumJS uses this to size its TileAvailability tree.  Without
    // it, overallMaxZoom = Math.max(0, undefined) = NaN, which leaves
    // _maximumLevel = NaN: the availability tree is never built and the
    // computeChildMaskForTile max-level early-exit never triggers, so Cesium
    // performs an unbounded lazy descent per tile per frame (main-thread
    // freeze + OOM crash).
    j["maxzoom"] = m_available.empty() ? 0 : static_cast<int>(m_available.size()) - 1;

    // Available levels
    //
    // Y-axis convention: TerrainQuadtree stores tile (x, y) in Cesium-internal
    // convention (Y=0 at north, increasing southward).  Cesium 1.111's
    // TileAvailability uses TMS Y (Y=0 at south) for both the `available`
    // array and the `{y}` URL template.  Convert Cesium-internal Y → TMS Y
    // so that Cesium's availability checks and URL requests match the
    // on-disk file names.
    nlohmann::ordered_json available = nlohmann::ordered_json::array();
    for (size_t lvl = 0; lvl < m_available.size(); ++lvl)
    {
        const int rowsAtLevel = 1 << static_cast<int>(lvl);  // GeographicTilingScheme: 2^L Y tiles
        const auto& ranges = m_available[lvl];
        nlohmann::ordered_json levelRanges = nlohmann::ordered_json::array();
        for (size_t i = 0; i < ranges.size(); ++i)
        {
            // Cesium-internal Y → TMS Y: tms_y = rows - 1 - cesium_y
            const int tmsStartY = rowsAtLevel - 1 - ranges[i].endY;
            const int tmsEndY   = rowsAtLevel - 1 - ranges[i].startY;
            levelRanges.push_back({
                {"startX", ranges[i].startX},
                {"endX", ranges[i].endX},
                {"startY", tmsStartY},
                {"endY", tmsEndY}
            });
        }
        available.push_back(levelRanges);
    }
    j["available"] = available;

    // Projection
    j["projection"] = m_projection;

    // littleEndianExtensionSize — controls byte order of extension length fields
    // Cesium's quantized-mesh decoder reads extension length via:
    //   C.getUint32(a, r.littleEndianExtensionSize)
    // where r.littleEndianExtensionSize comes from this layer.json field.
    // When extensions include "octvertexnormals" (not the legacy "vertexnormals"),
    // Cesium's parser leaves `s` undefined → getUint32 uses big-endian by default.
    // Our QuantizedMeshEncoder writes extension lengths as little-endian uint32 LE,
    // so we must set this flag to true. Without it, Cesium reads a garbage length
    // (e.g. 16777216 instead of 512) and either skips the extension or reads past
    // the buffer, producing corrupted normals or missing terrain geometry.
    j["littleEndianExtensionSize"] = true;

    return j.dump(2) + "\n";
}
