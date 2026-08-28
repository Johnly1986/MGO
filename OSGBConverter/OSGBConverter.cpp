// Copyright Johnlyon
//
// OSGBConverter implementation — OSGB → 3D Tiles conversion pipeline
//

#include "OSGBConverter.h"
#include "MetadataParser.h"
#include "OSGBReader.h"
#include "IVendorHandler.h"
#include "PlatformCompat.h"
#include "OSGBCellBuilder.h"

#include "../MeshProjectionErrorCorrector/CProjectionEngine.h"
#include "../MeshProjectionErrorCorrector/TileDataTypes.h"
#include "../MeshProjectionErrorCorrector/AxisMapper.h"
#include "../MeshProjectionErrorCorrector/CoordinateTransform.hpp"
#include "../MeshProjectionErrorCorrector/GeodeticMath.h"
#include "../TileBuilder/TileBuilder.h"
#include "../MeshGroupOptimizer/meshoptimizer/meshoptimizer.h"

#include <iostream>
#include <fstream>
#include <filesystem>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <cerrno>
#include <map>
#include <set>
#include <climits>
#include <functional>
#include <future>
#include <mutex>
#include <thread>
#include <atomic>

// ---------------------------------------------------------------------------
// Construction / Destruction
// ---------------------------------------------------------------------------

OSGBConverter::OSGBConverter() = default;
OSGBConverter::~OSGBConverter() = default;

// ---------------------------------------------------------------------------
// Main entry point
// ---------------------------------------------------------------------------

bool OSGBConverter::Convert(const OSGBConverterOptions& opts)
{
    // Reset state for re-entrant calls
    m_lastError.clear();
    m_textureBaseDir.clear();
    m_projEngine.reset(new CProjectionEngine());

    // Validate inputs first so an empty directory reports the right error
    // rather than a misleading "cannot detect vendor".
    if (opts.inputDir.empty())
    {
        m_lastError = "Input directory is required";
        return false;
    }
    if (opts.outputDir.empty())
    {
        m_lastError = "Output directory is required";
        return false;
    }

    // Auto-detect vendor and create handler
    m_vendorHandler = VendorHandlerFactory::Create(opts.inputDir);
    if (!m_vendorHandler)
    {
        m_lastError = "Cannot detect OSGB vendor (try ContextCapture Data/ or DJI Block_* layout)";
        return false;
    }

    if (opts.verbose)
        std::cout << "[OSGBConverter] Vendor: " << m_vendorHandler->GetVendorName()
                  << std::endl;

    // Create output directory
    DirectoryLister::MakePath(opts.outputDir);

    // === Stage 1: Parse metadata.xml and initialize projection ===
    OSGBMetadata metadata;
    metadata.vendor = m_vendorHandler->GetVendor();
    if (!InitializeProjection(opts, metadata))
    {
        m_lastError = "Failed to initialize projection";
        return false;
    }

    if (opts.verbose)
    {
        std::cout << "[OSGBConverter] SRS: " << metadata.srs << std::endl;
        std::cout << "[OSGBConverter] Origin: (" << metadata.originX << ", "
                  << metadata.originY << ", " << metadata.originZ << ")" << std::endl;
        std::cout << "[OSGBConverter] Projection mode: "
                  << (metadata.projectionMode == ProjectionMode::RootOnly
                      ? "RootOnly" : "PerTile") << std::endl;
        std::cout << "[OSGBConverter] Tile count: " << metadata.tilePaths.size() << std::endl;
        std::cout << "[OSGBConverter] Max LOD: " << metadata.maxLOD << std::endl;
    }

    // === Stage 2: Load all OSGB tiles ===
    std::vector<OSGBTileData> tiles;
    if (!LoadTiles(opts, metadata, tiles))
    {
        m_lastError = "Failed to load tiles";
        return false;
    }

    if (tiles.empty())
    {
        m_lastError = "No tiles loaded";
        return false;
    }

    // === Stage 3: Apply projection correction ===
    if (m_projEngine->HasProjection())
    {
        ApplyProjection(opts, metadata, tiles);
    }

    // === Stage 4: Build GridCell hierarchy (delegated to vendor handler) ===
    auto gridRoot = m_vendorHandler->BuildHierarchy(tiles, opts);
    if (!gridRoot)
    {
        m_lastError = "Failed to build grid cells";
        return false;
    }

    // === Stage 5: Optional mesh simplification ===
    if (opts.simplify.enabled())
    {
        if (opts.verbose)
            std::cout << "[OSGBConverter] Simplifying: error=" << opts.simplify.error
                      << " nweight=" << opts.simplify.normalWeight << std::endl;
        SimplifyGridCells(gridRoot.get(), opts);
    }

    // === Stage 7: Compute root transform and write output ===
    Eigen::Matrix4d rootTransform = Eigen::Matrix4d::Identity();
    const Eigen::Matrix4d* xform = nullptr;

    if (metadata.isENU)
    {
        // ENU mode: compute root transform directly from geographic lat/lon
        double latRad = metadata.enuLat * GeodeticMath::DEG2RAD;
        double lonRad = metadata.enuLon * GeodeticMath::DEG2RAD;
        double ecefX, ecefY, ecefZ;
        GeodeticMath::GeographicToECEF(latRad, lonRad, metadata.originZ,
                                        ecefX, ecefY, ecefZ);
        // ENU→ECEF 4×4 column-major. Does NOT negate the North column —
        // CesiumJS Y_UP_TO_Z_UP maps (East,Up,South)→(East,North,Up),
        // which is standard ENU consumed directly by the root transform.
        // (Same convention as CProjectionEngine::ComputeRootTransform)
        rootTransform = GeodeticMath::BuildRootTransform(
            GeodeticMath::ENUToECEFRotation(latRad, lonRad),
            Eigen::Vector3d(ecefX, ecefY, ecefZ));
        xform = &rootTransform;

        if (opts.verbose)
        {
            std::cout << "[OSGBConverter] ENU root transform: origin=("
                      << metadata.enuLat << ", " << metadata.enuLon
                      << "), ECEF=(" << ecefX << ", " << ecefY << ", " << ecefZ << ")"
                      << std::endl;
        }
    }
    else if (m_projEngine->HasProjection())
    {
        rootTransform = m_projEngine->ComputeRootTransform();
        xform = &rootTransform;
    }

    if (!WriteOutput(gridRoot.get(), opts, xform))
    {
        m_lastError = "Failed to write output";
        return false;
    }

    if (opts.verbose)
        std::cout << "[OSGBConverter] Conversion complete. Output: "
                  << opts.outputDir << std::endl;

    return true;
}

// ---------------------------------------------------------------------------
// Stage 1: Initialize projection from metadata.xml
// ---------------------------------------------------------------------------

bool OSGBConverter::InitializeProjection(const OSGBConverterOptions& opts,
                                          OSGBMetadata& metadata)
{
    // Parse metadata.xml via vendor-aware parser
    std::string xmlPath = opts.inputDir + "/metadata.xml";
    MetadataParser parser;
    bool parsed = parser.ParseWithHandler(xmlPath, metadata, *m_vendorHandler);

    if (!parsed)
    {
        // Try Data/ subdirectory (CC convention)
        xmlPath = opts.inputDir + "/Data/metadata.xml";
        parsed = parser.ParseWithHandler(xmlPath, metadata, *m_vendorHandler);
    }

    if (!parsed)
    {
        std::cerr << "[OSGBConverter] Warning: Could not parse metadata.xml, "
                  << "proceeding without projection info" << std::endl;
        metadata.projectionMode = ProjectionMode::PerTile;
    }

    // If no tiles found via XML, discover via directory scan
    if (metadata.tilePaths.empty())
    {
        metadata.tilePaths = m_vendorHandler->DiscoverTiles(opts.inputDir);
    }

    // Override origin from CLI if provided
    if (opts.hasOriginOverride)
    {
        metadata.originX = opts.originX;
        metadata.originY = opts.originY;
        metadata.originZ = opts.originZ;
    }

    // Override ENU origin from CLI (--enu lat,lon[,h]): route the dataset
    // through the existing ENU pipeline instead of a .prj projection.
    if (opts.hasENUOverride)
    {
        metadata.isENU = true;
        metadata.enuLat = opts.enuLat;
        metadata.enuLon = opts.enuLon;
        metadata.originZ = opts.enuH;
        metadata.projectionMode = ProjectionMode::PerTile;
    }

    // Delegate projection configuration to vendor handler
    m_vendorHandler->ConfigureProjection(metadata, opts.prjFile, *m_projEngine, opts.verbose);

    // Georeferencing (optional — datum shift between local and projected coords)
    if (opts.georefType != GeoreferencingType::None && m_projEngine->HasProjection())
    {
        if (!opts.prjFile.empty())
        {
            std::string prjContent;
            {
                std::ifstream f(opts.prjFile);
                if (f) prjContent.assign(std::istreambuf_iterator<char>(f),
                                         std::istreambuf_iterator<char>());
            }
            if (!prjContent.empty())
            {
                GeoreferencingOptions gopts;
                for (int i = 0; i < 7; ++i) gopts.helmert[i] = opts.helmert[i];
                auto georef = GeoreferencingFactory::Create(opts.georefType, prjContent, gopts);
                if (georef)
                {
                    if (opts.georefType == GeoreferencingType::MultiPosition &&
                        !opts.controlPoints.empty())
                    {
                        auto* multi = dynamic_cast<GeoreferencingWithMultiPosition*>(georef.get());
                        if (multi)
                        {
                            multi->SetFitMethod(opts.fitMethod);
                            if (opts.fitMethod == FitMethod::DirectPoly2D)
                                multi->SetPolyOrder(opts.polyOrder);
                            multi->SetParameter(opts.controlPoints);
                            multi->Solve();
                        }
                    }
                    m_georefHolder = std::move(georef);
                    m_projEngine->SetGeoreferencing(m_georefHolder.get());
                    if (opts.verbose)
                        std::cout << "[OSGBConverter] Georeferencing: type="
                                  << static_cast<int>(opts.georefType) << std::endl;
                }
            }
        }
    }

    // Determine max LOD from CLI override or auto-detect
    if (opts.maxLOD > 0)
    {
        metadata.maxLOD = opts.maxLOD;
    }

    return true;
}

// ---------------------------------------------------------------------------
// Stage 2: Load all OSGB tiles
// ---------------------------------------------------------------------------

bool OSGBConverter::LoadTiles(const OSGBConverterOptions& opts,
                               const OSGBMetadata& metadata,
                               std::vector<OSGBTileData>& tiles)
{
    OSGBReader reader;
    m_textureBaseDir = opts.inputDir;

    int loaded = reader.ReadAllTiles(metadata.tilePaths, opts.inputDir,
                                      *m_vendorHandler, metadata.maxLOD, tiles);

    return loaded > 0;
}

// ---------------------------------------------------------------------------
// Stage 3: Apply projection correction
// ---------------------------------------------------------------------------

void OSGBConverter::ApplyProjection(const OSGBConverterOptions& opts,
                                     const OSGBMetadata& metadata,
                                     std::vector<OSGBTileData>& tiles)
{
    if (metadata.projectionMode == ProjectionMode::RootOnly)
    {
        // RootOnly mode: no per-tile correction needed.
        // Tile vertices remain in projected coordinates (Easting, Northing, Height).
        // The root transform in tileset.json handles the ENU→ECEF conversion.
        if (opts.verbose)
        {
            std::cout << "[OSGBConverter] RootOnly mode: skipping per-tile correction"
                      << std::endl;
        }
        return;
    }

    // PerTile mode: apply per-tile delta correction (translation + rotation)
    if (opts.verbose)
    {
        std::cout << "[OSGBConverter] PerTile mode: applying per-tile delta correction"
                  << std::endl;
    }

    double maxDelta = 0;
    for (auto& tile : tiles)
    {
        // Compute tile centroid in local coordinates
        double cx = (tile.bboxMin[0] + tile.bboxMax[0]) * 0.5;
        double cy = (tile.bboxMin[1] + tile.bboxMax[1]) * 0.5;
        double cz = (tile.bboxMin[2] + tile.bboxMax[2]) * 0.5;

        // Compute projection delta + rotation correction at centroid
        double dx, dy, dz;
        Eigen::Matrix3d R_corr;
        double delta = m_projEngine->ComputeInstanceProjectionDelta(cx, cy, cz, dx, dy, dz, R_corr);

        if (delta > maxDelta) maxDelta = delta;

        // 1. Translation correction
        tile.localTransform[12] += dx;
        tile.localTransform[13] += dy;
        tile.localTransform[14] += dz;

        // 2. Rotation correction: convert ENU → model space, left-multiply
        //    Same pattern as CProjectionEngine::ApplyInstanceCorrection
        Eigen::Matrix3d Rc = MGO::CoordinateTransform::RotateMatrix(
            R_corr, MGO::CoordinateFrame::ENU, MGO::CoordinateFrame::AssimpYUp);

        float r0 = tile.localTransform[0],  r1 = tile.localTransform[1],  r2 = tile.localTransform[2];
        float r3 = tile.localTransform[4],  r4 = tile.localTransform[5],  r5 = tile.localTransform[6];
        float r6 = tile.localTransform[8],  r7 = tile.localTransform[9],  r8 = tile.localTransform[10];

        tile.localTransform[0]  = static_cast<float>(Rc(0,0)*r0 + Rc(0,1)*r3 + Rc(0,2)*r6);
        tile.localTransform[1]  = static_cast<float>(Rc(0,0)*r1 + Rc(0,1)*r4 + Rc(0,2)*r7);
        tile.localTransform[2]  = static_cast<float>(Rc(0,0)*r2 + Rc(0,1)*r5 + Rc(0,2)*r8);
        tile.localTransform[4]  = static_cast<float>(Rc(1,0)*r0 + Rc(1,1)*r3 + Rc(1,2)*r6);
        tile.localTransform[5]  = static_cast<float>(Rc(1,0)*r1 + Rc(1,1)*r4 + Rc(1,2)*r7);
        tile.localTransform[6]  = static_cast<float>(Rc(1,0)*r2 + Rc(1,1)*r5 + Rc(1,2)*r8);
        tile.localTransform[8]  = static_cast<float>(Rc(2,0)*r0 + Rc(2,1)*r3 + Rc(2,2)*r6);
        tile.localTransform[9]  = static_cast<float>(Rc(2,0)*r1 + Rc(2,1)*r4 + Rc(2,2)*r7);
        tile.localTransform[10] = static_cast<float>(Rc(2,0)*r2 + Rc(2,1)*r5 + Rc(2,2)*r8);

        // 3. Update bbox
        tile.bboxMin[0] += dx;
        tile.bboxMin[1] += dy;
        tile.bboxMin[2] += dz;
        tile.bboxMax[0] += dx;
        tile.bboxMax[1] += dy;
        tile.bboxMax[2] += dz;
    }

    if (opts.verbose)
    {
        std::cout << "[OSGBConverter] Max per-tile delta: " << maxDelta << " m" << std::endl;
    }
}

// Stage 7: Write output
// ---------------------------------------------------------------------------

bool OSGBConverter::WriteOutput(GridCell* root,
                                 const OSGBConverterOptions& opts,
                                 const Eigen::Matrix4d* rootTransform)
{
    TileBuildOptions buildOpts;
    buildOpts.outputDir = opts.outputDir;
    buildOpts.tileBaseName = opts.tileBaseName;
    buildOpts.refine = opts.refine;
    buildOpts.rootGeometricError = opts.rootGeometricError;
    buildOpts.tileGeometricError = opts.tileGeometricError;
    // ContextCapture/oblique-photography meshes have inconsistent triangle
    // winding, so single-sided rendering would cull ~half the faces (holes /
    // inside-out appearance). Render double-sided to show every face.
    buildOpts.doubleSided = true;
    buildOpts.hasProjection = m_projEngine->HasProjection();
    buildOpts.originX = m_projEngine->GetOriginX();
    buildOpts.originY = m_projEngine->GetOriginY();
    buildOpts.originZ = m_projEngine->GetOriginZ();
    buildOpts.fbxDirectory = m_textureBaseDir;

    // Collect all content cells (depth-first), tagging each with the external
    // tileset subdir of its top-level tile, then write in parallel.
    std::vector<std::pair<GridCell*, std::string>> contentCells;
    {
        std::function<void(GridCell*, const std::string&)> collect =
            [&](GridCell* cell, const std::string& subdir) {
                if (!cell) return;
                if (cell->hasContent && !cell->materialGroups.empty())
                    contentCells.push_back({cell, subdir});
                for (auto& c : cell->children)
                    if (c) collect(c.get(), subdir);
            };
        for (auto& c : root->children)
        {
            if (!c) continue;
            std::string subdir = c->isOverflow
                ? "overflow"
                : (opts.tileBaseName + "_" + c->cellKey);
            // OSGB always lays out b3dm under the grid subdir; mark it so
            // Generate emits an external subtree reference (consistency, else
            // small grids get referenced inline and 404).
            c->isExternal = true;
            collect(c.get(), subdir);
        }
    }

    std::atomic<int> tileCount{0};
    if (contentCells.size() <= 4)
    {
        // Small dataset: sequential
        int done = 0;
        for (auto& [cell, subdir] : contentCells)
        {
            WriteOSGBTiles(*cell, opts, subdir, tileCount);
            ++done;
            std::cout << "[OSGBConverter] 处理进度: " << done << "/"
                      << contentCells.size() << std::endl;
        }
    }
    else
    {
        // Parallel: independent GLB construction per tile.
        // Each content cell writes only its own tile (no recursion), so
        // there is no overlap between concurrent writes. Process in batches
        // bounded by the core count — spawning one thread per tile for a large
        // dataset (thousands) can exhaust thread stacks/handles and make fopen
        // fail intermittently.
        const size_t batchSize = std::max<size_t>(1, std::thread::hardware_concurrency());
        for (size_t start = 0; start < contentCells.size(); start += batchSize)
        {
            size_t end = std::min(start + batchSize, contentCells.size());
            std::vector<std::future<void>> futures;
            futures.reserve(end - start);
            for (size_t i = start; i < end; ++i)
            {
                auto [cell, subdir] = contentCells[i];
                futures.push_back(std::async(std::launch::async, [&, cell, subdir]() {
                    WriteOSGBTiles(*cell, opts, subdir, tileCount);
                }));
            }
            for (auto& f : futures) f.get();
            std::cout << "[OSGBConverter] 处理进度: " << end << "/"
                      << contentCells.size() << std::endl;
        }
    }
    int tileCountVal = tileCount.load();

    if (tileCountVal == 0)
    {
        m_lastError = "No tiles generated";
        return false;
    }

    // Propagate bboxes from content cells to internal nodes (bottom-up)
    BBoxUtils::UpdateGridCellBBoxes(*root);

    // Generate tileset.json (TilesetWriter::Generate writes it to disk)
    std::string outJson;
    if (!TilesetWriter::Generate(*root, buildOpts, rootTransform, outJson))
    {
        m_lastError = "Failed to generate tileset.json";
        return false;
    }

    std::cout << "[OSGBConverter] 处理完成: " << tileCountVal << " tile(s) → "
              << opts.outputDir << "/tileset.json" << std::endl;

    return true;
}

void OSGBConverter::WriteOSGBTiles(GridCell& cell, const OSGBConverterOptions& opts,
                                    const std::string& subdir, std::atomic<int>& tileCount)
{
    if (!cell.hasContent || cell.materialGroups.empty()) return;

    // Validate groups before writing
    bool anyValid = false;
    for (auto& g : cell.materialGroups)
    {
        if (!g.positions.empty() && !g.indices.empty())
        {
            anyValid = true;
            break;
        }
    }
    if (!anyValid) return;

    // Build GLB directly from pre-populated materialGroups
    BinaryBlob glb;
    if (!GlbBuilder::Build(cell.materialGroups, glb, m_textureBaseDir, true))
    {
        // GlbBuilder::Build returns false for empty groups
        return;
    }

    BinaryBlob b3dm;
    if (!B3dmBuilder::Build(glb, b3dm)) return;

    // Determine output path. b3dm files live inside the external tileset's
    // subdir (matching TilesetWriter::WriteSubtreeTileset's uri convention),
    // e.g. <outputDir>/<tileBaseName>_<cellKey>/L<level>/<tileName>.b3dm.
    std::string ld = "L" + std::to_string(cell.level);
    std::string prefix = subdir.empty() ? "" : (subdir + "/");
    std::string tilePath = opts.outputDir + "/" + prefix + ld + "/"
                         + opts.tileBaseName + "_" + cell.cellKey + ".b3dm";

    // Create directory (thread-safe: parallel tile writes may create the same
    // parent dirs concurrently, and create_directories handles that atomically).
    std::string dirPath = opts.outputDir + "/" + prefix + ld;
    std::error_code ec;
    std::filesystem::create_directories(dirPath, ec);

    FILE* fp = fopen(tilePath.c_str(), "wb");
    if (!fp)
    {
        std::cerr << "[OSGBConverter] Cannot write: " << tilePath
                  << " (errno=" << errno << ": " << std::strerror(errno) << ")"
                  << (ec ? " [mkdir: " + ec.message() + "]" : "")
                  << std::endl;
        return;
    }
    fwrite(b3dm.ptr(), 1, b3dm.size(), fp);
    fclose(fp);

    // Set tileFileName (name only — WriteNodeRecursive prepends level dir).
    // Must match TilesetWriter::WriteTiles convention (line 1082).
    cell.tileFileName = opts.tileBaseName + "_" + cell.cellKey + ".b3dm";

    ++tileCount;
}

void OSGBConverter::SimplifyGridCells(GridCell* root, const OSGBConverterOptions& opts)
{
    if (!root) return;

    for (auto& c : root->children)
        if (c) SimplifyGridCells(c.get(), opts);

    if (!root->hasContent || root->materialGroups.empty()) return;

    // Tile extent used for border locking. Vertices lying on one of the tile's
    // bounding-box faces are shared with adjacent tiles, so they must not be
    // moved or removed — otherwise simplification opens cracks along tile seams.
    const double* bmin = root->bboxMin;
    const double* bmax = root->bboxMax;
    double bdiag = std::sqrt(
        (bmax[0] - bmin[0]) * (bmax[0] - bmin[0]) +
        (bmax[1] - bmin[1]) * (bmax[1] - bmin[1]) +
        (bmax[2] - bmin[2]) * (bmax[2] - bmin[2]));
    double borderEps = bdiag * 1e-4;
    if (borderEps < 1e-6) borderEps = 1e-6;

    for (auto& group : root->materialGroups)
    {
        if (group.positions.empty() || group.indices.empty()) continue;

        size_t vertexCount = group.vertexCount();
        size_t indexCount  = group.indexCount();

        double cx = 0, cy = 0, cz = 0;
        for (size_t i = 0; i < vertexCount; ++i)
        {
            cx += group.positions[i * 3];
            cy += group.positions[i * 3 + 1];
            cz += group.positions[i * 3 + 2];
        }
        cx /= vertexCount; cy /= vertexCount; cz /= vertexCount;

        double diag = 0;
        for (size_t i = 0; i < vertexCount; ++i)
        {
            double dx = group.positions[i * 3]     - cx;
            double dy = group.positions[i * 3 + 1] - cy;
            double dz = group.positions[i * 3 + 2] - cz;
            double d = std::sqrt(dx*dx + dy*dy + dz*dz);
            if (d > diag) diag = d;
        }
        if (diag < 1e-6) diag = 1.0;
        double invScale = 1.0 / diag;

        std::vector<float> normPos(vertexCount * 3);
        for (size_t i = 0; i < vertexCount; ++i)
        {
            normPos[i * 3]     = static_cast<float>((group.positions[i * 3]     - cx) * invScale);
            normPos[i * 3 + 1] = static_cast<float>((group.positions[i * 3 + 1] - cy) * invScale);
            normPos[i * 3 + 2] = static_cast<float>((group.positions[i * 3 + 2] - cz) * invScale);
        }

        std::vector<unsigned char> vertexLock(vertexCount, 0);
        if (opts.simplify.lockBorder)
        {
            for (size_t i = 0; i < vertexCount; ++i)
            {
                float x = group.positions[i * 3];
                float y = group.positions[i * 3 + 1];
                float z = group.positions[i * 3 + 2];
                if (x < bmin[0] + borderEps || x > bmax[0] - borderEps ||
                    y < bmin[1] + borderEps || y > bmax[1] - borderEps ||
                    z < bmin[2] + borderEps || z > bmax[2] - borderEps)
                    vertexLock[i] = 1;
            }
        }

        unsigned int options = 0;
        if (opts.simplify.lockBorder) options |= meshopt_SimplifyLockBorder;
        if (opts.simplify.localError) options |= meshopt_SimplifyErrorAbsolute;

        size_t targetCount = opts.simplify.threshold > 0
            ? static_cast<size_t>(opts.simplify.threshold * indexCount) : 0;

        std::vector<unsigned int> simpIndices(indexCount);
        float resultError = 0;

        // Pass normals as a simplification attribute weighted by
        // normalWeight so collapses that bend sharp edges (roofs, walls)
        // are penalized. Mirrors the TinSimplifier convention.
        const bool useNormalAttr =
            opts.simplify.normalWeight > 0.0f &&
            group.normals.size() == vertexCount * 3;
        float nweights[3] = {
            opts.simplify.normalWeight,
            opts.simplify.normalWeight,
            opts.simplify.normalWeight
        };

        size_t newIdxCount = meshopt_simplifyWithAttributes(
            simpIndices.data(), group.indices.data(), indexCount,
            normPos.data(), vertexCount, sizeof(float) * 3,
            useNormalAttr ? group.normals.data() : nullptr,
            useNormalAttr ? sizeof(float) * 3 : 0,
            useNormalAttr ? nweights : nullptr,
            useNormalAttr ? 3 : 0,
            vertexLock.data(), targetCount, opts.simplify.error,
            options, &resultError);

        if (newIdxCount == 0) continue;

        // Optimize index order for GPU vertex cache and overdraw.
        // These steps reorder triangle indices to maximize post-T&L cache
        // reuse and minimize fragment shader overdraw. Vertex-fetch
        // reordering is intentionally NOT applied per-stream - it would
        // permute one attribute buffer and desynchronize the others.
        simpIndices.resize(newIdxCount);
        meshopt_optimizeVertexCache(simpIndices.data(), simpIndices.data(),
                                     newIdxCount, vertexCount);
        meshopt_optimizeOverdraw(simpIndices.data(), simpIndices.data(),
                                  newIdxCount, group.positions.data(),
                                  vertexCount, sizeof(float) * 3, 1.05f);
        // The remap must consider ALL attribute streams: matching positions
        // alone would merge UV-seam vertices (same position, different UV),
        // smearing the texture along seams. Note meshopt_Stream::size is the
        // size of ONE vertex element in bytes (must be <= 256), not the total
        // stream size - passing totals makes the hasher read out of bounds.
        meshopt_Stream streams[3];
        size_t streamCount = 0;
        streams[streamCount++] = { group.positions.data(), sizeof(float) * 3, sizeof(float) * 3 };
        if (!group.normals.empty())
            streams[streamCount++] = { group.normals.data(), sizeof(float) * 3, sizeof(float) * 3 };
        if (!group.texcoords.empty())
            streams[streamCount++] = { group.texcoords.data(), sizeof(float) * 2, sizeof(float) * 2 };

        std::vector<unsigned int> remap(vertexCount);
        size_t newVtxCount = meshopt_generateVertexRemapMulti(
            remap.data(), simpIndices.data(), newIdxCount,
            vertexCount, streams, streamCount);

        std::vector<unsigned int> remappedIdx(newIdxCount);
        meshopt_remapIndexBuffer(remappedIdx.data(), simpIndices.data(), newIdxCount, remap.data());

        std::vector<float> remappedPos(newVtxCount * 3);
        meshopt_remapVertexBuffer(remappedPos.data(), group.positions.data(), vertexCount,
                                  sizeof(float) * 3, remap.data());
        group.positions = std::move(remappedPos);
        group.indices   = std::move(remappedIdx);

        if (!group.normals.empty())
        {
            std::vector<float> remappedNorm(newVtxCount * 3);
            meshopt_remapVertexBuffer(remappedNorm.data(), group.normals.data(), vertexCount,
                                      sizeof(float) * 3, remap.data());
            group.normals = std::move(remappedNorm);
        }
        if (!group.texcoords.empty())
        {
            std::vector<float> remappedUV(newVtxCount * 2);
            meshopt_remapVertexBuffer(remappedUV.data(), group.texcoords.data(), vertexCount,
                                      sizeof(float) * 2, remap.data());
            group.texcoords = std::move(remappedUV);
        }

        // Re-run topological winding unification after simplification: edge
        // collapses can stitch previously disconnected patches, and the
        // remap/reorder steps do not alter orientation, so this only has to
        // repair what the collapse introduced. No up/down heuristic - shared
        // edges must simply be traversed in opposite directions.
        MakeWindingConsistent(group.indices);
    }
}

// ---------------------------------------------------------------------------
