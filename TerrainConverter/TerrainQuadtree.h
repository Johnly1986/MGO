#pragma once

#include "macro.h"
#include "HeightmapGrid.h"
#include "QuantizedMeshEncoder.h"
#include "../MeshProjectionErrorCorrector/CProjectionEngine.h"
#include <vector>
#include <string>
#include <functional>

// TerrainQuadtree — Cesium terrain tiling scheme
//
// Tiling convention (Cesium terrain):
//   - Projection: EPSG:4326 (geographic WGS84)
//   - Root level (L=0): 2 tiles
//       (0,0,0): lon [-180, 0], lat [-90, 90]
//       (0,1,0): lon [0, 180],  lat [-90, 90]
//   - Level L: 2 * 2^L tiles in X, 2^L tiles in Y
//   - Each tile subdivides into 4 children (NW, NE, SW, SE)
//   - Tile naming: {z}/{x}/{y}.terrain
//
// For non-global TIF data, we compute the minimal level that contains the TIF bounds
// and generate only the tiles that intersect.
//
class TERRAIN_CONVERTER_API TerrainQuadtree
{
public:
    TerrainQuadtree();
    ~TerrainQuadtree();

    // Build quadtree from heightmap grid (bottom-up with empty propagation).
    // grid: input elevation grid (in projected CRS)
    // projWKT: projection WKT (parsed by CProjectionEngine)
    // maxLevel: maximum LOD level. If -1, auto-compute from TIF resolution.
    // samplesPerTile: target samples per tile (e.g. 65)
    //
    // Generation is bottom-up: leaf tiles (maxLevel) are sampled first; a leaf
    // is empty iff all samples are noData. Non-leaf tiles are empty iff all 4
    // children are empty. Non-empty tiles are always generated, including L0
    // (root), so the Cesium availability tree is always complete.
    void Build(const HeightmapGrid& grid,
               const std::string& projWKT,
               int maxLevel,
               int samplesPerTile = 65);

    // Iterate all non-empty tiles (leaves + ancestors that survived empty propagation)
    void ForEachLeaf(std::function<void(const TerrainTile&)> visitor) const;

    // Get available tile ranges per level (for layer.json)
    struct TileRange
    {
        int startX, endX, startY, endY;
    };
    void GetAvailableLevels(std::vector<std::vector<TileRange>>& outLevels) const;

    // Get bounds
    void GetBounds(double& west, double& south, double& east, double& north) const;

    bool Empty() const { return m_tiles.empty(); }
    size_t TileCount() const { return m_tiles.size(); }

private:
    // Compute the maximal Cesium level where tile span >= TIF pixel size.
    // At level L, tile X span = 360/(2*2^L) degrees. Pick the largest L such
    // that tileSpan >= pixelSize, so one terrain sample ≈ one tile pixel.
    int ComputeMaxLevelFromResolution(double pixelSizeDegrees) const;
    // Compute tile geographic bounds at (level, x, y)
    void ComputeTileBounds(int level, int x, int y,
                           double& west, double& south,
                           double& east, double& north) const;

    // Extract local heightmap for a tile from the global grid.
    // Returns valid sample count (0 = empty tile, caller should skip).
    // Caller treats tiles with too few valid samples as empty so meshopt
    // doesn't collapse sparse data into all-noData output.
    int ExtractLocalGrid(const HeightmapGrid& globalGrid,
                         double tileWest, double tileSouth,
                         double tileEast, double tileNorth,
                         int samplesPerTile,
                         HeightmapGrid& outLocal) const;

    std::vector<TerrainTile> m_tiles;  // non-empty tiles (leaves + ancestors)
    double m_west = 0, m_south = 0, m_east = 0, m_north = 0;
    int    m_maxLevel = 0;
    int    m_samplesPerTile = 65;

    // Projection engine for accurate geographic<->projected conversion.
    // Set by Build() from the TIFF's WKT or user-supplied .prj file.
    CProjectionEngine m_proj;
    bool m_hasProjection = false;
};
