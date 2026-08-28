#include "TerrainConverter.h"
#include "../MeshProjectionErrorCorrector/CProjectionEngine.h"
#include "../MeshProjectionErrorCorrector/GeoreferencingWithMultiPosition.h"

#include <algorithm>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <sstream>
#include <iomanip>
#include <thread>
#include <future>
#include <atomic>
#include <mutex>

namespace fs = std::filesystem;

TerrainConverter::TerrainConverter() = default;
TerrainConverter::~TerrainConverter() = default;

bool TerrainConverter::Convert(const TerrainConverterOptions& opts)
{
    if (opts.inputTif.empty() || opts.outputDir.empty())
    {
        std::cerr << "[TerrainConverter] Input TIF and output directory required" << std::endl;
        return false;
    }

    // — 1. Read GeoTIFF —
    GeoTiffReader reader;
    if (!reader.Open(opts.inputTif))
    {
        std::cerr << "[TerrainConverter] Failed to open GeoTIFF" << std::endl;
        return false;
    }

    // Apply user-specified origin BEFORE ReadElevationGrid so the
    // resampling step maps output grid positions using the corrected
    // GeoTransform.  Origin (X, Y) replaces GT[0] and GT[3] directly;
    // all other GT parameters (pixel size, rotation, GT[5] sign) are
    // preserved from the TIF.
    if (opts.hasOrigin)
    {
        reader.OverrideOrigin(opts.originX, opts.originY);
    }

    HeightmapGrid grid;
    if (!reader.ReadElevationGrid(grid))
    {
        std::cerr << "[TerrainConverter] Failed to read elevation grid" << std::endl;
        reader.Close();
        return false;
    }
    reader.Close();

    // — 2. Build projection WKT —
    std::string projWKT;
    if (!opts.prjFile.empty())
    {
        std::ifstream f(opts.prjFile);
        if (f)
        {
            std::stringstream ss;
            ss << f.rdbuf();
            projWKT = ss.str();
        }
    }
    if (projWKT.empty() && !grid.projectionWKT.empty())
    {
        projWKT = grid.projectionWKT;
    }

    // If TIF is already geographic (EPSG:4326), no projection WKT needed
    if (grid.isGeographic)
    {
        projWKT.clear();
    }

    // — 2b. Load projection engine once (shared across all tiles) —
    // Previously each tile reloaded the .prj from a temp file — wasteful
    // when there are hundreds of tiles all using the same projection.
    CProjectionEngine proj;
    bool hasProj = false;
    if (!projWKT.empty())
    {
        hasProj = proj.LoadProjectionFromString(projWKT);
    }

    // — 2c. Georeferencing (optional) —
    // When control points or 7-param Helmert parameters are provided, create
    // a georeferencing strategy and attach it to the projection engine.
    // This applies a datum shift (local → projected) before projection.
    std::unique_ptr<IGeoreferencing> georefHolder;
    if (opts.georefType != GeoreferencingType::None && hasProj)
    {
        GeoreferencingOptions gopts;
        for (int i = 0; i < 7; ++i) gopts.helmert[i] = opts.helmert[i];
        georefHolder = GeoreferencingFactory::Create(opts.georefType, projWKT, gopts);
        if (georefHolder)
        {
            if (opts.georefType == GeoreferencingType::MultiPosition && !opts.controlPoints.empty())
            {
                auto* multi = dynamic_cast<GeoreferencingWithMultiPosition*>(georefHolder.get());
                if (multi)
                {
                    multi->SetFitMethod(opts.fitMethod);
                    if (opts.fitMethod == FitMethod::DirectPoly2D)
                        multi->SetPolyOrder(opts.polyOrder);
                    multi->SetParameter(opts.controlPoints);
                    multi->Solve();
                }
            }
            proj.SetGeoreferencing(georefHolder.get());
        }
    }

    // — 3. Build quadtree —
    TerrainQuadtree quadtree;
    quadtree.Build(grid, projWKT, opts.maxLODLevels, opts.samplesPerTile);

    if (quadtree.Empty())
    {
        std::cerr << "[TerrainConverter] No tiles generated — check input bounds" << std::endl;
        return false;
    }

    // — 3b. Compute global min/max height for consistent edge alignment —
    // All tiles share the same quantization range so that the same absolute
    // height encodes to the same h_raw in every tile, eliminating seams at
    // tile boundaries.  The range is extended to include 0 m (sea level) so
    // that noData vertices at 0 m absolute encode correctly without clamping.
    // For datasets entirely above sea level the floor stays at 0; for datasets
    // with below-sea-level terrain the floor extends downward to globalMinH.
    float globalMinH = 0, globalMaxH = 0;
    grid.ComputeMinMax(globalMinH, globalMaxH);
    if (globalMinH > globalMaxH) { globalMinH = 0; globalMaxH = 0; }

    // — 4. Process each tile (parallel) —
    int totalTiles = quadtree.TileCount();
    std::cout << "[TerrainConverter] 处理进度: 0/" << totalTiles << std::endl;

    // Collect tile pointers (ForEachLeaf provides const refs; ProcessTile takes const ref)
    std::vector<const TerrainTile*> tilePtrs;
    tilePtrs.reserve(static_cast<size_t>(totalTiles));
    quadtree.ForEachLeaf([&](const TerrainTile& tile) {
        tilePtrs.push_back(&tile);
    });

    std::atomic<int> processed{0};
    std::atomic<int> failed{0};
    std::mutex progressMutex;

    auto reportProgress = [&]() {
        int done = processed.load(std::memory_order_relaxed)
                 + failed.load(std::memory_order_relaxed);
        if (done % 10 == 0)
        {
            std::lock_guard<std::mutex> lk(progressMutex);
            std::cout << "[TerrainConverter] 处理进度: " << done << "/"
                      << totalTiles << std::endl;
        }
    };

    unsigned nThreads = std::thread::hardware_concurrency();
    if (nThreads < 2) nThreads = 1;
    // Cap at 8 to avoid oversubscription on high-core machines for small tile counts
    if (nThreads > 8) nThreads = 8;
    if (static_cast<int>(tilePtrs.size()) < static_cast<int>(nThreads))
        nThreads = static_cast<unsigned>(tilePtrs.size());
    if (nThreads < 1) nThreads = 1;

    auto processChunk = [&](size_t start, size_t end) {
        for (size_t i = start; i < end; ++i) {
            if (ProcessTile(*tilePtrs[i], opts, proj, hasProj, globalMinH, globalMaxH))
                processed.fetch_add(1, std::memory_order_relaxed);
            else
                failed.fetch_add(1, std::memory_order_relaxed);
            reportProgress();
        }
    };

    std::vector<std::future<void>> futures;
    size_t chunkSize = (tilePtrs.size() + nThreads - 1) / nThreads;
    for (unsigned t = 0; t < nThreads; ++t) {
        size_t start = t * chunkSize;
        size_t end = std::min(start + chunkSize, tilePtrs.size());
        if (start >= end) break;
        futures.push_back(std::async(std::launch::async, processChunk, start, end));
    }

    // Wait for all threads to complete
    for (auto& f : futures) f.get();

    int done = processed.load() + failed.load();
    if (failed.load() == 0)
        std::cout << "[TerrainConverter] 处理完成: " << done << "/"
                  << totalTiles << std::endl;
    else
        std::cerr << "[TerrainConverter] 处理完成: " << done << "/" << totalTiles
                  << " (" << failed.load() << " tiles failed)" << std::endl;

    // — 5. Generate layer.json —
    WriteLayerJson(opts, quadtree);

    return failed == 0;
}

bool TerrainConverter::ProcessTile(const TerrainTile& tile,
                                    const TerrainConverterOptions& opts,
                                    CProjectionEngine& proj,
                                    bool hasProj,
                                    float globalMinH,
                                    float globalMaxH)
{
    // — a. Build local heightmap from tile.localHeights —
    HeightmapGrid localGrid;
    localGrid.width = tile.localWidth;
    localGrid.height = tile.localHeight;
    localGrid.heights = tile.localHeights;
    localGrid.minEasting = tile.localMinEasting;
    localGrid.maxEasting = tile.localMaxEasting;
    localGrid.minNorthing = tile.localMinNorthing;
    localGrid.maxNorthing = tile.localMaxNorthing;
    if (localGrid.width > 1)
        localGrid.dx = (localGrid.maxEasting - localGrid.minEasting) / (localGrid.width - 1);
    if (localGrid.height > 1)
        localGrid.dy = (localGrid.maxNorthing - localGrid.minNorthing) / (localGrid.height - 1);
    localGrid.hasNoData   = tile.hasNoData;
    localGrid.noDataValue = tile.noDataValue;
    localGrid.noDataFill  = 0.0f;     // noData → absolute 0 metres
    localGrid.isGeographic = true;

    // — b. Simplify to TIN —
    TinMesh tin = TinSimplifier::Simplify(localGrid, proj, opts.simplify);
    if (tin.vertices.empty())
    {
        // Entire tile is noData or simplification failed — skip it.
        // Cesium will use the default ellipsoid surface for this area.
        std::cerr << "[TerrainConverter] Skipping empty tile "
                  << tile.level << "/" << tile.x << "/" << tile.y << std::endl;
        return true;
    }

    // — c. Quantize to (u, v, height) —
    TinMesh quantized;
    TinSimplifier::QuantizeToLocal(localGrid, proj, tin, quantized);

    // — d. Encode to quantized-mesh —
    // Encoding range is [encodingMinH, encodingMaxH] shared by all tiles.
    // The floor is min(0, globalMinH) so that: (a) noData vertices at 0 m
    // absolute always fall within the range and encode linearly to h_raw
    // without clamping, and (b) below-sea-level terrain (negative heights)
    // is preserved.  The ceiling is max(0, globalMaxH) to handle datasets
    // that are entirely below sea level (where globalMaxH < 0).
    float encodingMinH = std::min(0.0f, globalMinH);
    float encodingMaxH = std::max(0.0f, globalMaxH);
    if (encodingMinH >= encodingMaxH)
        encodingMaxH = encodingMinH + 1.0f;  // flat or degenerate guard
    quantized.minHeight = encodingMinH;
    quantized.maxHeight = encodingMaxH;
    // CProjectionEngine is shared across all tiles (loaded once in Convert()).
    // For geographic TIF (no projection), CProjectionEngine::GeographicToECEF
    // still works using default WGS84 ellipsoid parameters.

    QuantizedMeshOptions qopts;
    qopts.writeOctVertexNormals = opts.writeOctVertexNormals;
    qopts.writeWaterMask = opts.writeWaterMask;

    std::vector<uint8_t> bytes;
    if (!QuantizedMeshEncoder::Encode(quantized, tile, proj, qopts, bytes))
    {
        std::cerr << "[TerrainConverter] Encode failed for tile "
                  << tile.level << "/" << tile.x << "/" << tile.y << std::endl;
        return false;
    }

    // — e. Write file —
    fs::path tileDir = fs::path(opts.outputDir) / std::to_string(tile.level)
                       / std::to_string(tile.x);
    std::error_code ec;
    fs::create_directories(tileDir, ec);

    // Cesium's `{y}` URL template uses TMS Y (Y=0 at south) — verified
    // against Cesium 1.111's TileAvailability which returns true only
    // for TMS Y coordinates.  tile.y is stored in Cesium-internal
    // convention (Y=0 at north); convert to TMS Y for the file path.
    const int rowsAtLevel = 1 << tile.level;
    const int tmsY = rowsAtLevel - 1 - tile.y;
    std::string filePath = (tileDir / (std::to_string(tmsY) + ".terrain")).string();
    std::ofstream outFile(filePath, std::ios::binary);
    if (!outFile)
    {
        std::cerr << "[TerrainConverter] Cannot write: " << filePath << std::endl;
        return false;
    }
    outFile.write(reinterpret_cast<const char*>(bytes.data()),
                  static_cast<std::streamsize>(bytes.size()));
    outFile.close();

    if (opts.verbose)
    {
        std::cout << "[TerrainConverter] Wrote " << filePath
                  << " (" << bytes.size() << " bytes, "
                  << quantized.vertices.size() / 3 << " verts, "
                  << quantized.indices.size() / 3 << " tris)" << std::endl;
    }

    return true;
}

bool TerrainConverter::WriteLayerJson(const TerrainConverterOptions& opts,
                                       const TerrainQuadtree& quadtree)
{
    TerrainLayerJson lj;

    double w, s, e, n;
    quadtree.GetBounds(w, s, e, n);
    lj.SetBounds(w, s, e, n);
    lj.SetProjection("EPSG:4326");

    if (opts.writeOctVertexNormals)
        lj.AddExtension("octvertexnormals");
    if (opts.writeWaterMask)
        lj.AddExtension("watermask");

    std::vector<std::vector<TerrainQuadtree::TileRange>> levels;
    quadtree.GetAvailableLevels(levels);
    lj.SetAvailableLevels(levels);

    std::string json = lj.Generate();
    fs::path jsonPath = fs::path(opts.outputDir) / "layer.json";
    std::ofstream f(jsonPath.string());
    if (!f)
    {
        std::cerr << "[TerrainConverter] Cannot write layer.json" << std::endl;
        return false;
    }
    f << json;
    f.close();

    return true;
}
