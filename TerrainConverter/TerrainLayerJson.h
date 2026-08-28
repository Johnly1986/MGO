#pragma once

#include "macro.h"
#include "TerrainQuadtree.h"
#include <string>
#include <vector>

// TerrainLayerJson — generates layer.json for Cesium terrain provider
//
// layer.json describes the terrain layer metadata:
//   - tilejsonVersion: "1.0.0"
//   - format: "quantized-mesh-1.0"
//   - version: "1.0.0"
//   - extensions: ["octvertexnormals"]
//   - tiles: ["{z}/{x}/{y}.terrain"]
//   - available: array of level → tile ranges
//   - bounds: [west, south, east, north]
//   - projection: "EPSG:4326"
//
class TERRAIN_CONVERTER_API TerrainLayerJson
{
public:
    TerrainLayerJson();
    ~TerrainLayerJson();

    void SetBounds(double west, double south, double east, double north);
    void SetProjection(const std::string& epsg);
    void SetTileTemplate(const std::string& tmpl);  // e.g. "{z}/{x}/{y}.terrain"
    void AddExtension(const std::string& name);
    void SetAvailableLevels(const std::vector<std::vector<TerrainQuadtree::TileRange>>& levels);

    std::string Generate() const;

private:
    double m_west = -180, m_south = -90, m_east = 180, m_north = 90;
    std::string m_projection = "EPSG:4326";
    std::string m_tileTemplate = "{z}/{x}/{y}.terrain";
    std::vector<std::string> m_extensions;
    std::vector<std::vector<TerrainQuadtree::TileRange>> m_available;
};
