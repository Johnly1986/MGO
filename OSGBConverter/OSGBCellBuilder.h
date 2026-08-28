// Copyright Johnlyon
//
// OSGBCellBuilder — shared GridCell construction for OSGB → 3D Tiles
//
// Builds a GridCell from a single OSGB tile, converting geometry from the
// OSGB native ENU frame (X=East, Y=North, Z=Up) to the AssimpYUp frame
// (X=East, Y=Up, Z=South) that 3D Tiles glTF content expects.
//
// Why this conversion exists: b3dm embeds glTF, which is Y-up. CesiumJS applies
// Y_UP_TO_Z_UP (glTF +Y=up -> 3D Tiles +Z=up, +Z -> -Y), so the authored content
// must be in the (East, Up, South) frame — the same convention TilesConverter
// uses. OSGB data is natively (East, North, Up), so it must be rotated here.

#pragma once

#include "OSGBTileData.h"
#include "../MeshProjectionErrorCorrector/TileDataTypes.h"
#include "../MeshProjectionErrorCorrector/CoordinateTransform.hpp"

#include <memory>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

// Make triangle winding topologically consistent, with NO geometric
// up/down assumption: two manifold neighbors must traverse their shared
// edge in opposite directions. Consistency is propagated across shared
// edges within each connected component (DFS); each component's global
// sign is chosen to flip as few original triangles as possible, so
// legitimately down-facing geometry (overhangs, bridges, eaves) keeps its
// original orientation. Non-manifold edges (duplicate directed edges) are
// left untouched rather than guessed at.
inline void MakeWindingConsistent(std::vector<unsigned int>& indices)
{
    const size_t triCount = indices.size() / 3;
    if (triCount == 0) return;

    // Directed edge (a<<32 | b) -> owning triangle (first owner wins).
    std::unordered_map<uint64_t, uint32_t> owner;
    owner.reserve(triCount * 3);
    for (uint32_t t = 0; t < triCount; ++t)
        for (int e = 0; e < 3; ++e)
            owner.emplace((static_cast<uint64_t>(indices[t * 3 + e]) << 32)
                          | indices[t * 3 + (e + 1) % 3], t);

    std::vector<char> visited(triCount, 0);
    std::vector<char> flip(triCount, 0);
    std::vector<uint32_t> stack;
    std::vector<uint32_t> members;

    for (uint32_t seed = 0; seed < triCount; ++seed)
    {
        if (visited[seed]) continue;
        members.clear();
        size_t flips = 0;

        visited[seed] = 1;
        stack.push_back(seed);
        while (!stack.empty())
        {
            uint32_t t = stack.back();
            stack.pop_back();
            members.push_back(t);
            flips += static_cast<size_t>(flip[t]);

            for (int e = 0; e < 3; ++e)
            {
                unsigned int a = indices[t * 3 + e];
                unsigned int b = indices[t * 3 + (e + 1) % 3];
                if (flip[t]) std::swap(a, b);  // effective directed edge

                // Consistent neighbor carries the opposite directed edge
                // (b, a) effectively.
                auto it = owner.find((static_cast<uint64_t>(b) << 32) | a);
                if (it == owner.end() || it->second == t || visited[it->second])
                {
                    // Or it carries (a, b) originally and must flip. (When t
                    // itself is flipped it owns the (b,a) key, so the true
                    // neighbor is found here.)
                    it = owner.find((static_cast<uint64_t>(a) << 32) | b);
                    if (it == owner.end() || it->second == t || visited[it->second])
                        continue;
                    visited[it->second] = 1;
                    flip[it->second] = 1;
                    stack.push_back(it->second);
                }
                else
                {
                    // Neighbor originally owns (b, a): keep unflipped.
                    visited[it->second] = 1;
                    flip[it->second] = 0;
                    stack.push_back(it->second);
                }
            }
        }

        // Choose the component sign that flips fewer original triangles.
        if (flips * 2 > members.size())
            for (uint32_t t : members)
                flip[t] = !flip[t];
    }

    for (uint32_t t = 0; t < triCount; ++t)
        if (flip[t])
            std::swap(indices[t * 3 + 1], indices[t * 3 + 2]);
}

// Compute smooth vertex normals for a triangle-list mesh as area-weighted
// sums of the raw face cross products. Call MakeWindingConsistent first:
// mixed winding would otherwise cancel out at shared vertices. Result is
// unit length per glTF requirements.
inline void ComputeSmoothNormals(std::vector<float>& positions,
                                 const std::vector<unsigned int>& indices,
                                 std::vector<float>& normals)
{
    const size_t vc = positions.size() / 3;
    normals.assign(vc * 3, 0.0f);
    if (indices.size() < 3) return;

    for (size_t t = 0; t + 2 < indices.size(); t += 3)
    {
        unsigned int ia = indices[t], ib = indices[t + 1], ic = indices[t + 2];
        const float* pa = &positions[ia * 3];
        const float* pb = &positions[ib * 3];
        const float* pc = &positions[ic * 3];

        float e1[3] = { pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2] };
        float e2[3] = { pc[0] - pa[0], pc[1] - pa[1], pc[2] - pa[2] };
        // Cross product magnitude doubles the triangle area -> area-weighted.
        float n[3] = {
            e1[1] * e2[2] - e1[2] * e2[1],
            e1[2] * e2[0] - e1[0] * e2[2],
            e1[0] * e2[1] - e1[1] * e2[0]
        };

        for (unsigned int vi : { ia, ib, ic })
        {
            normals[vi * 3 + 0] += n[0];
            normals[vi * 3 + 1] += n[1];
            normals[vi * 3 + 2] += n[2];
        }
    }

    for (size_t i = 0; i < vc; ++i)
    {
        float* n = &normals[i * 3];
        float len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        if (len < 1e-12f)
        {
            // Conflicting faces cancelled out; fall back to up.
            n[0] = 0.0f; n[1] = 1.0f; n[2] = 0.0f;
        }
        else
        {
            n[0] /= len; n[1] /= len; n[2] /= len;
        }
    }
}

inline std::unique_ptr<GridCell> BuildGridCellFromTile(OSGBTileData& tile,
                                                       const std::string& cellKey)
{
    auto cell = std::make_unique<GridCell>();
    cell->level = tile.lodLevel;
    cell->hasContent = true;
    cell->cellKey = cellKey;

    tile.EnsureGroups();

    for (auto& g : tile.groups)
    {
        if (g.IsEmpty()) continue;
        size_t vc = g.VertexCount();

        MergedMeshGroup mg;
        mg.positions.resize(vc * 3);
        for (size_t i = 0; i < vc; ++i)
        {
            double in[3] = { g.positions[i * 3], g.positions[i * 3 + 1], g.positions[i * 3 + 2] };
            double out[3];
            MGO::CoordinateTransform::Convert(in, MGO::CoordinateFrame::ENU,
                                              out, MGO::CoordinateFrame::AssimpYUp);
            mg.positions[i * 3]     = static_cast<float>(out[0]);
            mg.positions[i * 3 + 1] = static_cast<float>(out[1]);
            mg.positions[i * 3 + 2] = static_cast<float>(out[2]);
        }

        mg.normals.resize(vc * 3);
        // Length check (not just HasNormals): a group may accumulate several
        // geometries whose normal availability differs, leaving the array short.
        if (g.normals.size() == vc * 3)
        {
            for (size_t i = 0; i < vc; ++i)
            {
                double in[3] = { g.normals[i * 3], g.normals[i * 3 + 1], g.normals[i * 3 + 2] };
                double out[3];
                MGO::CoordinateTransform::ConvertNormal(in, MGO::CoordinateFrame::ENU,
                                                        out, MGO::CoordinateFrame::AssimpYUp);
                mg.normals[i * 3]     = static_cast<float>(out[0]);
                mg.normals[i * 3 + 1] = static_cast<float>(out[1]);
                mg.normals[i * 3 + 2] = static_cast<float>(out[2]);
            }
        }

        mg.texcoords = g.HasTexCoords() ? g.texcoords : std::vector<float>(vc * 2, 0.0f);
        mg.indices.assign(g.indices.begin(), g.indices.end());

        // Unify winding topologically (no up/down assumption). This both
        // gives the glTF output a consistent CCW front face and makes the
        // smooth-normal accumulation below valid (mixed winding would
        // cancel at shared vertices).
        MakeWindingConsistent(mg.indices);

        if (g.normals.size() != vc * 3)
        {
            // Source tile has no normals: compute smooth ones directly in the
            // AssimpYUp frame (Y = up) so the glTF NORMAL accessor is valid
            // (unit length, same frame as POSITION). Must run AFTER indices
            // are assigned - the computation iterates the triangle list.
            ComputeSmoothNormals(mg.positions, mg.indices, mg.normals);
        }

        MGO::CoordinateTransform::ConvertBBox(g.bboxMin, g.bboxMax,
                                              MGO::CoordinateFrame::ENU,
                                              mg.bboxMin, mg.bboxMax,
                                              MGO::CoordinateFrame::AssimpYUp);
        for (int i = 0; i < 4; ++i)
            mg.baseColorFactor[i] = g.baseColorFactor[i];
        mg.diffuseTexturePath = g.texturePath;
        mg.texturePixels = g.texturePixels;
        mg.textureWidth = g.textureWidth;
        mg.textureHeight = g.textureHeight;
        mg.materialIndex = 0;
        cell->materialGroups.push_back(std::move(mg));
    }

    // Convert the tile extent to AssimpYUp as well, so TilesetWriter's
    // WriteBoxJson (which assumes AssimpYUp → TilesZUp) is consistent.
    MGO::CoordinateTransform::ConvertBBox(tile.bboxMin, tile.bboxMax,
                                          MGO::CoordinateFrame::ENU,
                                          cell->bboxMin, cell->bboxMax,
                                          MGO::CoordinateFrame::AssimpYUp);
    for (int i = 0; i < 3; ++i)
    {
        cell->localBboxMin[i] = cell->bboxMin[i];
        cell->localBboxMax[i] = cell->bboxMax[i];
    }

    return cell;
}
