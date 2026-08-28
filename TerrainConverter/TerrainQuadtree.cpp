#include "TerrainQuadtree.h"
#include "../MeshProjectionErrorCorrector/CProjectionEngine.h"
#include "../MeshProjectionErrorCorrector/GeodeticMath.h"
#include "../MeshProjectionErrorCorrector/Constants.h"

#include <cmath>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <set>
#include <utility>

TerrainQuadtree::TerrainQuadtree()
    : m_west(0), m_south(0), m_east(0), m_north(0)
{
}

TerrainQuadtree::~TerrainQuadtree() = default;

void TerrainQuadtree::Build(const HeightmapGrid& grid,
                             const std::string& projWKT,
                             int maxLevel,
                             int samplesPerTile)
{
    m_tiles.clear();
    m_samplesPerTile = samplesPerTile;

    // — 1. Convert grid bounds to geographic (lat/lon) —
    m_hasProjection = false;
    if (!projWKT.empty())
    {
        m_hasProjection = m_proj.LoadProjectionFromString(projWKT);
    }

    if (!m_hasProjection)
    {
        std::cerr << "[TerrainQuadtree] No projection — assuming EPSG:4326 (geographic)" << std::endl;
        m_west  = grid.minEasting;
        m_east  = grid.maxEasting;
        m_south = grid.minNorthing;
        m_north = grid.maxNorthing;
    }
    else
    {
        // Sample all 4 projected corners — GK/UTM rectangles are rotated
        // relative to the lat/lon grid, so min/max of SW+NE alone can miss
        // the true geographic extent at NW/SE corners.
        const double cE[4] = { grid.minEasting, grid.maxEasting,
                                grid.minEasting, grid.maxEasting };
        const double cN[4] = { grid.minNorthing, grid.minNorthing,
                                grid.maxNorthing, grid.maxNorthing };
        m_west = 180.0; m_east = -180.0; m_south = 90.0; m_north = -90.0;
        for (int i = 0; i < 4; ++i) {
            double lat, lon;
            m_proj.ProjectedToGeographic(cE[i], cN[i], lat, lon);
            lon *= Angle::RAD_TO_DEG; lat *= Angle::RAD_TO_DEG;
            if (lon < m_west)  m_west  = lon;
            if (lon > m_east)  m_east  = lon;
            if (lat < m_south) m_south = lat;
            if (lat > m_north) m_north = lat;
        }
    }

    if (m_west  < -180.0) m_west  = -180.0;
    if (m_east  >  180.0) m_east  =  180.0;
    if (m_south <  -90.0) m_south =  -90.0;
    if (m_north >   90.0) m_north =   90.0;

    std::cout << "[TerrainQuadtree] Geographic bounds: W=" << m_west
              << " S=" << m_south << " E=" << m_east << " N=" << m_north << std::endl;

    // — 2. Compute maxLevel from TIF resolution if not user-specified —
    if (maxLevel >= 0)
    {
        m_maxLevel = maxLevel;
        std::cout << "[TerrainQuadtree] Max level (user override): " << m_maxLevel << std::endl;
    }
    else
    {
        double pixelSizeDegrees;
        if (grid.isGeographic)
        {
            pixelSizeDegrees = grid.dx;
        }
        else
        {
            double midLat = (m_south + m_north) * 0.5;
            double metersPerDeg = Earth::METERS_PER_DEGREE_AT_EQUATOR * std::cos(midLat * Angle::DEG_TO_RAD);
            pixelSizeDegrees = grid.dx / metersPerDeg;
        }
        m_maxLevel = ComputeMaxLevelFromResolution(pixelSizeDegrees);
        std::cout << "[TerrainQuadtree] Max level (auto from TIF resolution "
                  << pixelSizeDegrees << " deg/px): " << m_maxLevel << std::endl;
    }

    // — 3. Bottom-up tile generation with empty propagation —
    // Leaf tiles (maxLevel) are sampled first. A leaf tile is empty if ALL its
    // samples are noData. Non-leaf tiles are empty if ALL 4 children are empty.
    // Empty tiles are NOT generated. Since TIF has real data, non-empty leaf
    // tiles exist, so at least one ancestor chain reaches L0 — guaranteeing a
    // root node for Cesium's availability tree.
    std::set<std::tuple<int, int, int>> emptyTiles;

    // Helper: compute Y range for a given level
    auto computeRange = [&](int level, int& xMin, int& xMax, int& yMin, int& yMax) {
        int tilesX = 2 * (1 << level);
        int tilesY = (1 << level);
        double tileW = 360.0 / tilesX;
        double tileH = 180.0 / tilesY;
        xMin = static_cast<int>(std::floor((m_west + 180.0) / tileW));
        xMax = static_cast<int>(std::floor((m_east + 180.0) / tileW));
        yMin = static_cast<int>(std::floor((90.0 - m_north) / tileH));
        yMax = static_cast<int>(std::floor((90.0 - m_south) / tileH));
        xMin = std::max(0, xMin);  xMax = std::min(tilesX - 1, xMax);
        yMin = std::max(0, yMin);  yMax = std::min(tilesY - 1, yMax);
    };

    // Helper: add non-empty tile to m_tiles
    auto addTile = [&](int level, int x, int y, HeightmapGrid& lg) {
        double tw, ts, te, tn;
        ComputeTileBounds(level, x, y, tw, ts, te, tn);
        TerrainTile tile;
        tile.level = level; tile.x = x; tile.y = y;
        tile.west = tw; tile.east = te; tile.south = ts; tile.north = tn;
        tile.localHeights = std::move(lg.heights);
        tile.localWidth  = lg.width;  tile.localHeight = lg.height;
        tile.localMinEasting = lg.minEasting; tile.localMaxEasting = lg.maxEasting;
        tile.localMinNorthing = lg.minNorthing; tile.localMaxNorthing = lg.maxNorthing;
        tile.hasNoData = lg.hasNoData; tile.noDataValue = lg.noDataValue;
        tile.hasContent = true;
        m_tiles.push_back(std::move(tile));
    };

    // Pass 1: leaf level (maxLevel) — sample every candidate tile.
    // A leaf tile is empty if validCount==0 (no TIF coverage) OR validCount is
    // too small to form a usable mesh (sparse hits that meshopt would collapse
    // into all-noData output). Threshold: at least 4 valid samples (enough for
    // one triangle).
    {
        int xMin, xMax, yMin, yMax;
        computeRange(m_maxLevel, xMin, xMax, yMin, yMax);
        std::cout << "[TerrainQuadtree] Level " << m_maxLevel
                  << " (leaf): X[" << xMin << "," << xMax << "] Y[" << yMin << "," << yMax << "]"
                  << " (" << (xMax - xMin + 1) * (yMax - yMin + 1) << " candidates)" << std::endl;
        int totalCandidates = (xMax - xMin + 1) * (yMax - yMin + 1);
        int leafProcessed = 0;
        int leafAdded = 0;
        int leafEmpty = 0;
        int reportStep = std::max(1, totalCandidates / 100);  // report ~every 1%
        int nextReport = 0;
        std::cout << std::unitbuf;  // disable buffering so progress shows up live
        for (int y = yMin; y <= yMax; ++y) {
            for (int x = xMin; x <= xMax; ++x) {
                double tw, ts, te, tn;
                ComputeTileBounds(m_maxLevel, x, y, tw, ts, te, tn);
                if (te <= m_west || tw >= m_east || tn <= m_south || ts >= m_north) {
                    emptyTiles.emplace(m_maxLevel, x, y);
                    ++leafEmpty;
                } else {
                    HeightmapGrid lg;
                    int vc = ExtractLocalGrid(grid, tw, ts, te, tn, m_samplesPerTile, lg);
                    if (vc < 4) {
                        emptyTiles.emplace(m_maxLevel, x, y);
                        ++leafEmpty;
                    } else {
                        addTile(m_maxLevel, x, y, lg);
                        ++leafAdded;
                    }
                }
                ++leafProcessed;
                if (leafProcessed >= nextReport) {
                    int pct = totalCandidates > 0 ? (leafProcessed * 100 / totalCandidates) : 100;
                    std::cout << "[TerrainQuadtree]   leaf sampling " << leafProcessed
                              << "/" << totalCandidates << " (" << pct << "%, added "
                              << leafAdded << ", empty " << leafEmpty << ")" << std::endl;
                    nextReport = leafProcessed + reportStep;
                }
            }
        }
        std::cout << "[TerrainQuadtree]   leaf sampling done: " << leafProcessed
                  << "/" << totalCandidates
                  << " (added " << leafAdded << ", empty " << leafEmpty << ")" << std::endl;
        std::cout << std::nounitbuf;
    }

    // Pass 2: propagate emptiness upward from maxLevel-1 to 0
    for (int lev = m_maxLevel - 1; lev >= 0; --lev) {
        // All tiles use the same samplesPerTile grid density (matching Cesium's
        // default 65×65 heightmap) so placeholder tiles are as dense as real
        // DEM tiles.
        static const int ANCESTOR_MAX = 4;
        int samples = m_samplesPerTile;
        int xMin, xMax, yMin, yMax;
        computeRange(lev, xMin, xMax, yMin, yMax);
        int added = 0, empty = 0;
        for (int y = yMin; y <= yMax; ++y) {
            for (int x = xMin; x <= xMax; ++x) {
                // Check 4 children for emptiness
                int cl = lev + 1;
                bool c00 = emptyTiles.count({cl, 2*x,   2*y  }) > 0;
                bool c10 = emptyTiles.count({cl, 2*x+1, 2*y  }) > 0;
                bool c01 = emptyTiles.count({cl, 2*x,   2*y+1}) > 0;
                bool c11 = emptyTiles.count({cl, 2*x+1, 2*y+1}) > 0;
                if (c00 && c10 && c01 && c11) {
                    emptyTiles.emplace(lev, x, y);
                    ++empty; continue;
                }
                double tw, ts, te, tn;
                ComputeTileBounds(lev, x, y, tw, ts, te, tn);
                if (te <= m_west || tw >= m_east || tn <= m_south || ts >= m_north) {
                    emptyTiles.emplace(lev, x, y); continue;
                }
                HeightmapGrid lg;
                int vc = ExtractLocalGrid(grid, tw, ts, te, tn, samples, lg);
                // Threshold for fallback: require enough valid samples to form
                // a meaningful mesh. 4 is the minimum for one quad (2 triangles).
                // Sparse tiles with 1-40 valid samples out of 4225 often produce
                // no valid triangles after noData filtering (isolated valid
                // samples don't form quads), so use a higher threshold.
                const int MIN_VALID_FOR_TRIANGULATION = 4;
                if (vc >= MIN_VALID_FOR_TRIANGULATION) {
                    addTile(lev, x, y, lg); ++added;
                } else if (grid.heights.size() > 0) {
                    // Sparse-coverage fallback: fill the tile with a flat 0 m
                    // grid at the same density as real tiles (samplesPerTile).
                    // Border locking keeps the dense perimeter so the mesh
                    // follows the ellipsoid's curvature instead of collapsing to
                    // a handful of huge triangles.
                    const int GW = m_samplesPerTile, GH = m_samplesPerTile;
                    lg.width  = GW;
                    lg.height = GH;
                    lg.dx = (te - tw) / (GW - 1);
                    lg.dy = (tn - ts) / (GH - 1);
                    lg.minEasting  = tw;
                    lg.maxEasting  = te;
                    lg.minNorthing = ts;
                    lg.maxNorthing = tn;
                    lg.heights.assign(GW * GH, 0.0f);
                    lg.isGeographic = true;
                    lg.epsg = 4326;
                    addTile(lev, x, y, lg); ++added;
                }
            }
        }
        std::cout << "[TerrainQuadtree] Level " << lev
                  << (lev <= ANCESTOR_MAX ? " (ancestor)" : "")
                  << ": added " << added << ", empty " << empty << std::endl;
    }

    // — 4. Ensure both level-0 root tiles always exist —
    // Cesium's terrain tiling scheme defines 2 root tiles at level 0:
    //   (0,0,0): lon [-180, 0], lat [-90, 90]  — western hemisphere
    //   (0,1,0): lon [  0,180], lat [-90, 90]  — eastern hemisphere
    // CesiumJS requests both root tiles unconditionally. If a TIF only covers
    // one hemisphere, the missing root tile never gets generated (computeRange
    // filters it out), CesiumJS gets a 404, and the terrain layer fails to load.
    // Write a minimal flat tile at height 0 for any missing root tile.
    {
        auto rootTileExists = [&](int rx, int ry) -> bool {
            for (const auto& t : m_tiles)
                if (t.level == 0 && t.x == rx && t.y == ry) return true;
            return false;
        };
        for (int rx = 0; rx <= 1; ++rx) {
            if (!rootTileExists(rx, 0)) {
                double tw, ts, te, tn;
                ComputeTileBounds(0, rx, 0, tw, ts, te, tn);
                const int GW = m_samplesPerTile, GH = m_samplesPerTile;
                HeightmapGrid lg;
                lg.width  = GW;
                lg.height = GH;
                lg.dx = (te - tw) / (GW - 1);
                lg.dy = (tn - ts) / (GH - 1);
                lg.minEasting  = tw;
                lg.maxEasting  = te;
                lg.minNorthing = ts;
                lg.maxNorthing = tn;
                lg.heights.assign(GW * GH, 0.0f);
                lg.isGeographic = true;
                lg.epsg = 4326;
                addTile(0, rx, 0, lg);
                std::cout << "[TerrainQuadtree] Level 0 (ancestor):"
                          << " added missing root tile (" << rx << ",0)"
                          << " as flat placeholder" << std::endl;
            }
        }
    }

    std::cout << "[TerrainQuadtree] Generated " << m_tiles.size() << " non-empty tiles across "
              << (m_maxLevel + 1) << " LOD levels (0 -> " << m_maxLevel << ")" << std::endl;
}

int TerrainQuadtree::ComputeMaxLevelFromResolution(double pixelSizeDegrees) const
{
    // At level L, tile X span = 360/(2*2^L) degrees.
    // We want each tile to cover ~m_samplesPerTile TIF pixels, so that one
    // terrain sample ≈ one TIF pixel after sampling.
    //   tile_span ≈ pixelSize * m_samplesPerTile
    //   360 / (2 * 2^L) ≈ pixelSize * samples
    //   2 * 2^L ≈ 360 / (pixelSize * samples)
    //   2^L ≈ 180 / (pixelSize * samples)
    //   L ≈ log2(180 / (pixelSize * samples))
    if (pixelSizeDegrees <= 0) return 0;
    int level = static_cast<int>(std::floor(std::log2(180.0 / (pixelSizeDegrees * m_samplesPerTile))));
    if (level < 0) level = 0;
    // — samplesPerTile=65 already captures all detail at L17 for a 0.3m DEM.
    if (level > TileConstants::MAX_TERRAIN_LOD) level = TileConstants::MAX_TERRAIN_LOD;
    return level;
}

void TerrainQuadtree::ComputeTileBounds(int level, int x, int y,
                                         double& west, double& south,
                                         double& east, double& north) const
{
    int tilesX = 2 * (1 << level);
    int tilesY = (1 << level);
    double tileW = 360.0 / tilesX;
    double tileH = 180.0 / tilesY;
    west  = -180.0 + x * tileW;
    east  = west + tileW;
    // Cesium convention: Y=0 at +90° (north), increasing southward.
    north = 90.0 - y * tileH;
    south = north - tileH;
}

int TerrainQuadtree::ExtractLocalGrid(const HeightmapGrid& globalGrid,
                                       double tileWest, double tileSouth,
                                       double tileEast, double tileNorth,
                                       int samplesPerTile,
                                       HeightmapGrid& outLocal) const
{
    // Sample global grid at regular intervals within tile bounds
    outLocal.width = samplesPerTile;
    outLocal.height = samplesPerTile;
    outLocal.heights.resize(static_cast<size_t>(samplesPerTile) * samplesPerTile);
    outLocal.hasNoData = globalGrid.hasNoData;
    outLocal.noDataValue = globalGrid.noDataValue;
    outLocal.dx = (tileEast - tileWest) / (samplesPerTile - 1);
    outLocal.dy = (tileNorth - tileSouth) / (samplesPerTile - 1);
    outLocal.minEasting = tileWest;
    outLocal.maxEasting = tileEast;
    outLocal.minNorthing = tileSouth;
    outLocal.maxNorthing = tileNorth;
    outLocal.isGeographic = true;
    outLocal.epsg = 4326;

    // Geographic (lon,lat) → projected (E,N) for sampling.
    // Three paths depending on what projection info is available:
    //   geographic grid:       sample directly with (lon, lat)
    //   projected grid + proj: use accurate forward projection GeographicToProjected
    //   projected grid − proj: linear map using grid bounds (fallback)
    bool sampleInGeographic = globalGrid.isGeographic;

    // noDataFill is set later by TerrainConverter::ProcessTile based on the
    // global height range computed once for all tiles.  Setting it here would
    // require an O(W×H) ComputeMinMax scan of the entire source grid — wasted
    // work since ProcessTile immediately overwrites it.
    outLocal.noDataFill = 0.0f;

    int validCount = 0;
    for (int r = 0; r < samplesPerTile; ++r)
    {
        for (int c = 0; c < samplesPerTile; ++c)
        {
            double lon = tileWest + c * outLocal.dx;
            double lat = tileNorth - r * outLocal.dy;  // row 0 = north

            float h = globalGrid.noDataValue;
            if (sampleInGeographic)
            {
                globalGrid.BilinearSample(lon, lat, h);
            }
            else if (m_hasProjection)
            {
                // Accurate: use forward projection (geographic -> projected)
                double latRad = lat * Angle::DEG_TO_RAD;
                double lonRad = lon * Angle::DEG_TO_RAD;
                double e = 0.0, n = 0.0;
                m_proj.GeographicToProjected(latRad, lonRad, e, n);
                globalGrid.BilinearSample(e, n, h);
            }
            else
            {
                // Fallback: linear map from geographic to projected using grid bounds
                double fracX = (lon - m_west) / (m_east - m_west);
                double fracY = (lat - m_south) / (m_north - m_south);
                double e = globalGrid.minEasting + fracX * (globalGrid.maxEasting - globalGrid.minEasting);
                double n = globalGrid.minNorthing + fracY * (globalGrid.maxNorthing - globalGrid.minNorthing);
                globalGrid.BilinearSample(e, n, h);
            }

            // Store sample. noData samples keep noDataValue so IsValid() returns
            // false; TinSimplifier will set their vertex position height to 0.
            // Valid samples store the real elevation.
            outLocal.heights[static_cast<size_t>(r) * samplesPerTile + c] = h;
            if (!globalGrid.hasNoData || std::abs(h - globalGrid.noDataValue) > 1e-6f)
                ++validCount;
        }
    }

    return validCount;
}

void TerrainQuadtree::ForEachLeaf(std::function<void(const TerrainTile&)> visitor) const
{
    for (const auto& t : m_tiles)
        visitor(t);
}

void TerrainQuadtree::GetAvailableLevels(std::vector<std::vector<TileRange>>& outLevels) const
{
    outLevels.clear();
    if (m_tiles.empty()) return;

    // Group tiles by level
    int maxL = 0;
    for (const auto& t : m_tiles) if (t.level > maxL) maxL = t.level;

    outLevels.resize(maxL + 1);

    // Pass 1: merge consecutive X tiles with the same Y into horizontal ranges.
    // Tiles are stored in (y ascending, x ascending) order from Build().
    for (const auto& t : m_tiles)
    {
        auto& level = outLevels[t.level];
        if (level.empty())
        {
            level.push_back({t.x, t.x, t.y, t.y});
        }
        else
        {
            auto& r = level.back();
            if (t.x == r.endX + 1 && t.y == r.startY)
            {
                r.endX = t.x;
            }
            else
            {
                level.push_back({t.x, t.x, t.y, t.y});
            }
        }
    }

    // Pass 2: merge consecutive Y ranges with the same X span into vertical blocks.
    // E.g. [{X:805-806, Y:330}, {X:805-806, Y:331}] → [{X:805-806, Y:330-331}]
    for (auto& level : outLevels)
    {
        if (level.size() < 2) continue;
        std::vector<TileRange> merged;
        merged.reserve(level.size());
        for (auto& r : level)
        {
            if (!merged.empty())
            {
                auto& prev = merged.back();
                if (r.startX == prev.startX && r.endX == prev.endX &&
                    r.startY == prev.endY + 1)
                {
                    prev.endY = r.endY;
                    continue;
                }
            }
            merged.push_back(r);
        }
        level = std::move(merged);
    }
}

void TerrainQuadtree::GetBounds(double& west, double& south, double& east, double& north) const
{
    west = m_west; south = m_south; east = m_east; north = m_north;
}
