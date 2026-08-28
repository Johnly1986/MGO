// Copyright Johnlyon
//
// TilesConverter — Assimp scene → 3D Tiles (b3dm + tileset.json)
//
// Orchestration layer. Delegates to:
//   - TileBuilder (GlbBuilder, B3dmBuilder, MaterialGrouper, TilesetWriter)
//   - CProjectionEngine (projection, ECEF, root transform)
//

#include "TilesConverter.h"
#include "../MeshGroupOptimizer/MeshGroupOptimizer.h"
#include "../MeshProjectionErrorCorrector/GeoreferencingFactory.h"

#include <assimp/scene.h>
#include <assimp/mesh.h>

#include <algorithm>
#include <cmath>
#include <cfloat>
#include <fstream>
#include <functional>
#include <iostream>
#include <unordered_map>
#include <Eigen/Dense>

// ===========================================================================
// Helpers
// ===========================================================================
namespace
{
    TileBuildOptions ToBuildOpts(const TilesConverterOptions& o)
    {
        TileBuildOptions b;
        b.outputDir        = o.outputDir;
        b.tileBaseName     = o.tileBaseName;
        b.refine           = o.refine;
        b.fbxDirectory     = o.fbxDirectory;
        b.rootGeometricError = o.rootGeometricError;
        b.tileGeometricError = o.tileGeometricError;
        b.inputIsZUp       = o.inputIsZUp;
        b.doubleSided      = o.doubleSided;
        b.minBlockDistance  = o.minBlockDistance;
        b.maxLODLevels     = o.maxLODLevels;
        b.hasProjection    = !o.prjFile.empty();
        b.originX          = o.originX;
        b.originY          = o.originY;
        b.originZ          = o.originZ;
        return b;
    }

    void fromMatrix4x4(const aiMatrix4x4& mat, float* out)
    {
        out[0]  = mat.a1; out[1]  = mat.a2; out[2]  = mat.a3; out[3]  = mat.a4;
        out[4]  = mat.b1; out[5]  = mat.b2; out[6]  = mat.b3; out[7]  = mat.b4;
        out[8]  = mat.c1; out[9]  = mat.c2; out[10] = mat.c3; out[11] = mat.c4;
        out[12] = mat.d1; out[13] = mat.d2; out[14] = mat.d3; out[15] = mat.d4;
    }
}

// ===========================================================================
// TilesConverter
// ===========================================================================

TilesConverter::TilesConverter()  {}
TilesConverter::~TilesConverter() {}

int TilesConverter::GetTileCount() const
{
    if (!m_gridRoot) return 0;
    return TilesetWriter::CountDescendantContent(*m_gridRoot);
}

// ===========================================================================
// CollectMeshInstances — flatten scene, bake world transforms + bboxes
// ===========================================================================
bool TilesConverter::CollectMeshInstances(const aiScene* scene)
{
    if (!scene || !scene->HasMeshes())
        return false;

    m_instances.clear();

    std::function<void(aiNode*, const aiMatrix4x4&)> traverse =
        [&](aiNode* node, const aiMatrix4x4& parentXform)
    {
        aiMatrix4x4 world = parentXform * node->mTransformation;

        for (unsigned int i = 0; i < node->mNumMeshes; ++i)
        {
            unsigned int mi = node->mMeshes[i];
            if (mi >= scene->mNumMeshes) continue;

            MeshInstance inst;
            inst.meshIndex = mi;
            fromMatrix4x4(world, inst.worldTransform);

            const aiMesh* mesh = scene->mMeshes[mi];
            double sumX = 0.0, sumY = 0.0, sumZ = 0.0;
            for (unsigned int vi = 0; vi < mesh->mNumVertices; ++vi)
            {
                aiVector3D wp = world * mesh->mVertices[vi];
                double wx = wp.x, wy = wp.y, wz = wp.z;
                if (wx < inst.bboxMin[0]) inst.bboxMin[0] = wx;
                if (wy < inst.bboxMin[1]) inst.bboxMin[1] = wy;
                if (wz < inst.bboxMin[2]) inst.bboxMin[2] = wz;
                if (wx > inst.bboxMax[0]) inst.bboxMax[0] = wx;
                if (wy > inst.bboxMax[1]) inst.bboxMax[1] = wy;
                if (wz > inst.bboxMax[2]) inst.bboxMax[2] = wz;
                sumX += wx; sumY += wy; sumZ += wz;
            }
            // Vertex centroid (average position) - used as reference point for
            // per-instance delta computation. This is NOT the bbox center nor the
            // worldTransform translation; it's the true mean of all vertices.
            if (mesh->mNumVertices > 0)
            {
                double inv = 1.0 / static_cast<double>(mesh->mNumVertices);
                inst.vertexCentroid[0] = sumX * inv;
                inst.vertexCentroid[1] = sumY * inv;
                inst.vertexCentroid[2] = sumZ * inv;
                inst.vertexCount = mesh->mNumVertices;
            }

            m_instances.push_back(inst);
        }

        for (unsigned int i = 0; i < node->mNumChildren; ++i)
            traverse(node->mChildren[i], world);
    };

    traverse(scene->mRootNode, aiMatrix4x4());
    return !m_instances.empty();
}

// ===========================================================================
// BuildGridHierarchy — bottom-up sparse grid aggregation
// ===========================================================================
bool TilesConverter::BuildGridHierarchy(const aiScene* scene)
{
    if (m_instances.empty()) return false;

    double cellSize = m_opts.minBlockDistance;
    int maxLOD = m_opts.maxLODLevels;
    if (maxLOD < 1) maxLOD = 1;
    if (cellSize <= 0) cellSize = 100.0;

    // 1. Compute global bounding box with median-based outlier rejection
    std::vector<double> centersX, centersY, centersZ;
    centersX.reserve(m_instances.size());
    centersY.reserve(m_instances.size());
    centersZ.reserve(m_instances.size());
    for (auto& inst : m_instances)
    {
        if (inst.bboxMin[0] > 1e100 || inst.bboxMax[0] < -1e100)
            continue;
        centersX.push_back((inst.bboxMin[0] + inst.bboxMax[0]) * 0.5);
        centersY.push_back((inst.bboxMin[1] + inst.bboxMax[1]) * 0.5);
        centersZ.push_back((inst.bboxMin[2] + inst.bboxMax[2]) * 0.5);
    }
    if (centersX.empty()) return false;

    std::sort(centersX.begin(), centersX.end());
    std::sort(centersY.begin(), centersY.end());
    std::sort(centersZ.begin(), centersZ.end());
    double medianX = centersX[centersX.size() / 2];
    double medianY = centersY[centersY.size() / 2];
    double medianZ = centersZ[centersZ.size() / 2];

    double outlierThreshold = cellSize * (1 << maxLOD) * 10.0;
    if (outlierThreshold < 100000.0) outlierThreshold = 100000.0;

    double gMin[3] = { DBL_MAX, DBL_MAX, DBL_MAX };
    double gMax[3] = { -DBL_MAX, -DBL_MAX, -DBL_MAX };
    for (auto& inst : m_instances)
    {
        if (inst.bboxMin[0] > 1e100 || inst.bboxMax[0] < -1e100)
            continue;
        double cx = (inst.bboxMin[0] + inst.bboxMax[0]) * 0.5;
        double cy = (inst.bboxMin[1] + inst.bboxMax[1]) * 0.5;
        double cz = (inst.bboxMin[2] + inst.bboxMax[2]) * 0.5;
        if (std::abs(cx - medianX) > outlierThreshold ||
            std::abs(cy - medianY) > outlierThreshold ||
            std::abs(cz - medianZ) > outlierThreshold)
            continue;
        for (int a = 0; a < 3; ++a)
        {
            if (inst.bboxMin[a] < gMin[a]) gMin[a] = inst.bboxMin[a];
            if (inst.bboxMax[a] > gMax[a]) gMax[a] = inst.bboxMax[a];
        }
    }

    for (int a = 0; a < 3; ++a)
    {
        double pad = (gMax[a] - gMin[a]) * 0.001 + 1.0;
        gMin[a] -= pad;
        gMax[a] += pad;
    }

    // 2. Sparse cell map — packed 64-bit key (no allocation per lookup)
    // Layout: [level:5][ix:16][iy:16][iz:16], 53 bits used, 11 spare.
    // On MSVC, int is 32-bit regardless of platform.
    std::unordered_map<uint64_t, std::unique_ptr<GridCell>> cellMap;

    auto makeKey = [](int level, int ix, int iy, int iz) -> uint64_t {
        uint64_t k = 0;
        k |= (static_cast<uint64_t>(level) & 0x1F) << 48;   // 5 bits: level 0–31
        k |= (static_cast<uint64_t>(static_cast<int64_t>(ix)) & 0xFFFF) << 32;
        k |= (static_cast<uint64_t>(static_cast<int64_t>(iy)) & 0xFFFF) << 16;
        k |= (static_cast<uint64_t>(static_cast<int64_t>(iz)) & 0xFFFF);
        return k;
    };

    auto getOrCreateCell = [&](int level, int ix, int iy, int iz) -> GridCell* {
        uint64_t key = makeKey(level, ix, iy, iz);
        auto it = cellMap.find(key);
        if (it != cellMap.end()) return it->second.get();

        auto c = std::make_unique<GridCell>();
        c->level = level;
        c->ix = ix; c->iy = iy; c->iz = iz;
        // Store human-readable key for debugging and TilesetWriter use
        c->cellKey = std::to_string(level) + "_" + std::to_string(ix) + "_"
                   + std::to_string(iy) + "_" + std::to_string(iz);

        double s = cellSize * (1 << level);
        c->bboxMin[0] = gMin[0] + ix * s;
        c->bboxMin[1] = gMin[1] + iy * s;
        c->bboxMin[2] = gMin[2] + iz * s;
        c->bboxMax[0] = c->bboxMin[0] + s;
        c->bboxMax[1] = c->bboxMin[1] + s;
        c->bboxMax[2] = c->bboxMin[2] + s;
        c->localBboxMin[0] = c->bboxMin[0];
        c->localBboxMin[1] = c->bboxMin[1];
        c->localBboxMin[2] = c->bboxMin[2];
        c->localBboxMax[0] = c->bboxMax[0];
        c->localBboxMax[1] = c->bboxMax[1];
        c->localBboxMax[2] = c->bboxMax[2];

        GridCell* ptr = c.get();
        cellMap[key] = std::move(c);
        return ptr;
    };

    // 3. Candidate pool
    std::vector<size_t> pool;
    pool.reserve(m_instances.size());
    for (size_t i = 0; i < m_instances.size(); ++i)
    {
        auto& inst = m_instances[i];
        if (inst.bboxMin[0] > 1e100 || inst.bboxMax[0] < -1e100)
            continue;
        double cx = (inst.bboxMin[0] + inst.bboxMax[0]) * 0.5;
        double cy = (inst.bboxMin[1] + inst.bboxMax[1]) * 0.5;
        double cz = (inst.bboxMin[2] + inst.bboxMax[2]) * 0.5;
        if (std::abs(cx - medianX) > outlierThreshold ||
            std::abs(cy - medianY) > outlierThreshold ||
            std::abs(cz - medianZ) > outlierThreshold)
            continue;
        pool.push_back(i);
    }

    // 4. Bottom-up assignment
    for (int level = 0; level < maxLOD && !pool.empty(); ++level)
    {
        double s = cellSize * (1 << level);
        std::vector<size_t> nextPool;

        for (size_t idx : pool)
        {
            auto& inst = m_instances[idx];

            if (inst.bboxMin[0] > 1e100 || inst.bboxMax[0] < -1e100)
            {
                nextPool.push_back(idx);
                continue;
            }
            int ixMin = static_cast<int>(std::floor((inst.bboxMin[0] - gMin[0]) / s));
            int iyMin = static_cast<int>(std::floor((inst.bboxMin[1] - gMin[1]) / s));
            int izMin = static_cast<int>(std::floor((inst.bboxMin[2] - gMin[2]) / s));
            int ixMax = static_cast<int>(std::floor((inst.bboxMax[0] - gMin[0]) / s));
            int iyMax = static_cast<int>(std::floor((inst.bboxMax[1] - gMin[1]) / s));
            int izMax = static_cast<int>(std::floor((inst.bboxMax[2] - gMin[2]) / s));

            if (ixMin == ixMax && iyMin == iyMax && izMin == izMax)
            {
                GridCell* cell = getOrCreateCell(level, ixMin, iyMin, izMin);
                cell->instances.push_back(std::move(inst));
                cell->hasContent = true;
            }
            else
            {
                nextPool.push_back(idx);
            }
        }

        pool = std::move(nextPool);

        if (pool.empty()) break;
    }

    // 5. Overflow
    if (!pool.empty())
    {
        auto overflow = std::make_unique<GridCell>();
        overflow->level = maxLOD;
        overflow->ix = 0; overflow->iy = 0; overflow->iz = 0;
        overflow->cellKey = "overflow";
        overflow->isOverflow = true;
        for (int a = 0; a < 3; ++a)
        {
            overflow->bboxMin[a] = gMin[a];
            overflow->bboxMax[a] = gMax[a];
            overflow->localBboxMin[a] = gMin[a];
            overflow->localBboxMax[a] = gMax[a];
        }

        for (size_t idx : pool)
            overflow->instances.push_back(std::move(m_instances[idx]));
        overflow->hasContent = true;

        constexpr uint64_t OVERFLOW_KEY = UINT64_MAX;
        cellMap[OVERFLOW_KEY] = std::move(overflow);
    }

    // 6. Build root node
    m_gridRoot = std::make_unique<GridCell>();
    m_gridRoot->level = maxLOD + 1;
    m_gridRoot->ix = 0; m_gridRoot->iy = 0; m_gridRoot->iz = 0;
    m_gridRoot->cellKey = "root";
    for (int a = 0; a < 3; ++a)
    {
        m_gridRoot->bboxMin[a] = gMin[a];
        m_gridRoot->bboxMax[a] = gMax[a];
        m_gridRoot->localBboxMin[a] = gMin[a];
        m_gridRoot->localBboxMax[a] = gMax[a];
    }

    // 7. Link parent-child relationships
    //    Phase A: pre-create all parent cells, bottom-up by level
    {
        for (int lvl = 0; lvl < maxLOD; ++lvl)
        {
            struct ParentKey { int level, ix, iy, iz; };
            std::vector<ParentKey> parentsNeeded;
            for (auto& kv : cellMap)
            {
                GridCell* cell = kv.second.get();
                if (!cell || cell->isOverflow) continue;
                if (cell->level != lvl) continue;
                parentsNeeded.push_back({ lvl + 1,
                                          cell->ix >> 1,
                                          cell->iy >> 1,
                                          cell->iz >> 1 });
            }
            for (auto& pk : parentsNeeded)
                getOrCreateCell(pk.level, pk.ix, pk.iy, pk.iz);
        }
    }

    //    Phase B: link children to parents
    {
        std::vector<uint64_t> keysToRemove;
        for (int lvl = 0; lvl < maxLOD; ++lvl)
        {
            for (auto& kv : cellMap)
            {
                GridCell* cell = kv.second.get();
                if (!cell || cell->isOverflow) continue;
                if (cell->level != lvl) continue;

                int pIx = cell->ix >> 1;
                int pIy = cell->iy >> 1;
                int pIz = cell->iz >> 1;
                int parentLevel = cell->level + 1;

                uint64_t pkey = makeKey(parentLevel, pIx, pIy, pIz);
                auto it = cellMap.find(pkey);
                if (it == cellMap.end()) continue;
                if (!it->second) continue;

                GridCell* parent = it->second.get();
                parent->ensureChildSlots();
                int ci = GridCell::childSparseIndex(cell->ix & 1, cell->iy & 1, cell->iz & 1);
                parent->children[ci] = std::move(kv.second);
                cell->parent = parent;
                keysToRemove.push_back(kv.first);
            }
        }
        for (auto& k : keysToRemove)
            cellMap.erase(k);
    }

    // Link remaining cells to root
    {
        for (auto& kv : cellMap)
        {
            GridCell* cell = kv.second.get();
            if (!cell) continue;

            m_gridRoot->children.push_back(std::move(kv.second));
            cell->parent = m_gridRoot.get();
        }
    }

    return true;
}

// ===========================================================================
// Convert — main entry point
// ===========================================================================
bool TilesConverter::Convert(const aiScene* scene,
                              const TilesConverterOptions& options)
{
    m_opts = options;
    m_instances.clear();
    m_gridRoot.reset();
    m_tilesetJson.clear();
    m_tileCount = 0;

    // 0. Initialize projection engine
    m_projEngine.Reset();
    if (!m_opts.prjFile.empty())
    {
        if (!m_projEngine.LoadProjection(m_opts.prjFile))
        {
            std::cerr << "[TilesConverter] Warning: failed to parse projection file,"
                      << " coordinates will not be transformed" << std::endl;
        }
        m_projEngine.SetOrigin(m_opts.originX, m_opts.originY, m_opts.originZ);
    }

    // 0a. Georeferencing (local coordinate -> projected coordinate transform)
    std::unique_ptr<IGeoreferencing> georefHolder;
    if (m_opts.georefType != GeoreferencingType::None && !m_opts.prjFile.empty())
    {
        // Read file content — the factory expects a CRS definition string
        // (WKT or PROJ string), not a file path.
        std::string prjContent;
        {
            std::ifstream f(m_opts.prjFile);
            if (f)
            {
                prjContent.assign(std::istreambuf_iterator<char>(f),
                                  std::istreambuf_iterator<char>());
            }
        }
        if (prjContent.empty())
        {
            std::cerr << "[TilesConverter] Warning: empty projection file "
                      << m_opts.prjFile << std::endl;
        }
        else
        {
            GeoreferencingOptions gopts;
            gopts.helmert[0] = m_opts.helmert[0]; gopts.helmert[1] = m_opts.helmert[1];
            gopts.helmert[2] = m_opts.helmert[2]; gopts.helmert[3] = m_opts.helmert[3];
            gopts.helmert[4] = m_opts.helmert[4]; gopts.helmert[5] = m_opts.helmert[5];
            gopts.helmert[6] = m_opts.helmert[6];
            georefHolder = GeoreferencingFactory::Create(m_opts.georefType, prjContent, gopts);
            if (georefHolder)
            {
                if (m_opts.georefType == GeoreferencingType::MultiPosition &&
                    !m_opts.controlPoints.empty())
                {
                    auto* multi = dynamic_cast<GeoreferencingWithMultiPosition*>(georefHolder.get());
                    if (multi)
                    {
                        multi->SetFitMethod(m_opts.fitMethod);
                        if (m_opts.fitMethod == FitMethod::DirectPoly2D)
                            multi->SetPolyOrder(m_opts.polyOrder);
                        multi->SetParameter(m_opts.controlPoints);
                        multi->Solve();
                    }
                }
                m_projEngine.SetGeoreferencing(georefHolder.get());
            }
        }
    }

    // 0b. Mesh simplification (optional, in-place on aiScene)
    if (m_opts.simplify.enabled())
    {
        OptimizerConfig simpCfg;
        simpCfg.reorder = false;
        simpCfg.items.push_back(OptimizerItem(".*",
            m_opts.simplify.error, m_opts.simplify.normalWeight,
            m_opts.simplify.threshold, m_opts.simplify.lockBorder,
            m_opts.simplify.localError));
        CMeshGroupOptimizer::SimplifyScene(scene, simpCfg);
    }

    // 1. Collect mesh instances
    if (!CollectMeshInstances(scene))
    {
        std::cerr << "[TilesConverter] No mesh instances found" << std::endl;
        return false;
    }
    // 1b. Auto-detect per-vertex mode: if the model span exceeds 3 km,
    // per-vertex correction eliminates the tangent-plane residual that would
    // otherwise reach ~60 cm at the model edges (d²/R at 3 km ≈ 70 cm).
    if (!m_opts.perVertexProjectionCorrection && !m_instances.empty())
    {
        double minE = DBL_MAX, maxE = -DBL_MAX, minN = DBL_MAX, maxN = -DBL_MAX;
        for (auto& inst : m_instances)
        {
            if (inst.bboxMin[0] < minE) minE = inst.bboxMin[0];
            if (inst.bboxMax[0] > maxE) maxE = inst.bboxMax[0];
            if (inst.bboxMin[2] < minN) minN = inst.bboxMin[2];  // Z=South, span ≈ North extent
            if (inst.bboxMax[2] > maxN) maxN = inst.bboxMax[2];
        }
        double span = std::sqrt(
            (maxE - minE) * (maxE - minE) + (maxN - minN) * (maxN - minN));
        if (span > 3000.0)
        {
            m_opts.perVertexProjectionCorrection = true;
        }
    }

    // 1c. Projection correction (delegate to CProjectionEngine)
    if (m_opts.perVertexProjectionCorrection)
    {
        // Per-vertex: compute delta for each vertex, bake into aiScene vertices
        m_projEngine.ApplyPerVertexProjectionCorrection(
            scene, m_instances.data(), static_cast<int>(m_instances.size()));
    }
    else
    {
        // Per-instance: rebase centroid to origin, then apply delta + rotation
        m_projEngine.RebaseInstancesToCentroid(
            scene, m_instances.data(), static_cast<int>(m_instances.size()));
        m_projEngine.ApplyPerInstanceProjectionCorrection(
            m_instances.data(), static_cast<int>(m_instances.size()));
    }

    // 2. Build spatial hierarchy
    if (!BuildGridHierarchy(scene))
    {
        std::cerr << "[TilesConverter] Failed to build grid hierarchy" << std::endl;
        return false;
    }

    // 3. Write tiles via TilesetWriter
    TileBuildOptions buildOpts = ToBuildOpts(m_opts);
    const int MIN_CONTENT_FOR_EXTERNAL = TileConstants::MIN_CONTENT_FOR_EXTERNAL_TILESET;

    int totalTiles = TilesetWriter::CountDescendantContent(*m_gridRoot);
    if (totalTiles == 0)
    {
        std::cerr << "[TilesConverter] No tiles generated" << std::endl;
        return false;
    }
    m_tileCount = totalTiles;

    int writtenTiles = 0;
    for (auto& c : m_gridRoot->children)
    {
        if (!c) continue;
        int n = TilesetWriter::CountDescendantContent(*c);
        std::string subdir;
        if (n >= MIN_CONTENT_FOR_EXTERNAL)
        {
            subdir = c->isOverflow
                ? "overflow"
                : (m_opts.tileBaseName + "_" + c->cellKey);
        }
        // Persist the external/inline decision on the cell: Generate must
        // route content to the SAME subdir layout WriteTiles used. Recomputing
        // the descendant count after WriteTiles would diverge (tiny cells are
        // dropped), stranding .b3dm under the subdir while the root tileset
        // references them inline -> 404.
        c->isExternal = !subdir.empty();
        if (!TilesetWriter::WriteTiles(*c, scene, buildOpts, subdir)) {
            std::cerr << "[TilesConverter] Failed to write tiles for " << subdir << std::endl;
            return false;
        }
        writtenTiles += n;
        std::cout << "[TilesConverter] 处理进度: "
                  << std::min(writtenTiles, totalTiles) << "/" << totalTiles << std::endl;
    }

    // 4. Propagate bboxes from content cells to internal nodes
    BBoxUtils::UpdateGridCellBBoxes(*m_gridRoot);

    // 5. Generate tileset.json using CProjectionEngine for root transform
    Eigen::Matrix4d rootTransform = Eigen::Matrix4d::Identity();
    if (m_projEngine.HasProjection())
    {
        rootTransform = m_projEngine.ComputeRootTransform();
    }

    std::string tilesetJson;
    if (!TilesetWriter::Generate(*m_gridRoot, buildOpts,
                                  m_projEngine.HasProjection() ? &rootTransform : nullptr,
                                  tilesetJson))
    {
        std::cerr << "[TilesConverter] Failed to generate tileset.json" << std::endl;
        return false;
    }

    m_tilesetJson = tilesetJson;

    return true;
}
