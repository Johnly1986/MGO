// Regression test: empty primitives (vertices but no triangles) must NOT be
// exported. Covers two drops:
//   1. GroupCellByMaterial: a cell whose faces are all degenerate/non-
//      triangular is dropped entirely (hasContent=false, no materialGroups).
//   2. GlbBuilder::Build: a MergedMeshGroup with zero indices is filtered;
//      a valid group in the same cell still exports, with no count:0
//      accessors in the GLB JSON chunk.
#include "../TileBuilder/TileBuilder.h"
#include <assimp/scene.h>
#include <assimp/mesh.h>
#include <assimp/material.h>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>

using std::cout;
using std::endl;

static int g_fail = 0;
#define CHECK(cond) do { if (!(cond)) { std::cout << "FAIL " << __FILE__ << ":" << __LINE__ << "\n"; g_fail++; } } while (0)

static void fillGridMesh(aiMesh* m, int G)
{
    m->mNumVertices = static_cast<unsigned int>(G * G);
    m->mVertices = new aiVector3D[G * G];
    for (int iy = 0; iy < G; ++iy)
        for (int ix = 0; ix < G; ++ix)
            m->mVertices[iy * G + ix] = aiVector3D(ix * 0.1f, iy * 0.1f, 0);
    m->mMaterialIndex = 0;
}

static void addFace(aiMesh* m, unsigned int i0, unsigned int i1, unsigned int i2)
{
    aiFace& f = m->mFaces[m->mNumFaces++];
    f.mNumIndices = 3;
    f.mIndices = new unsigned int[3]{i0, i1, i2};
}

static void addInstance(GridCell& cell, unsigned int meshIndex)
{
    MeshInstance inst;
    inst.meshIndex = meshIndex;
    inst.worldTransform[0] = 1; inst.worldTransform[5] = 1;
    inst.worldTransform[10] = 1; inst.worldTransform[15] = 1;
    cell.instances.push_back(inst);
}

int main()
{
    auto* scene = new aiScene;
    scene->mNumMeshes = 1;
    scene->mMeshes = new aiMesh*[1];
    scene->mNumMaterials = 1;
    scene->mMaterials = new aiMaterial*[1];
    scene->mMaterials[0] = new aiMaterial;

    TileBuildOptions opts;

    // ---- Case 1: 60 vertices, all faces degenerate (i0 == i1) ----
    {
        aiMesh* m = new aiMesh;
        fillGridMesh(m, 8 /*64 verts > 50 threshold*/);
        m->mNumFaces = 0;
        m->mFaces = new aiFace[10];
        for (int i = 0; i < 10; ++i)
            addFace(m, 0, 0, i % m->mNumVertices);  // degenerate
        scene->mMeshes[0] = m;

        GridCell cell;
        cell.hasContent = true;
        cell.level = 0;
        addInstance(cell, 0);

        MaterialGrouper::GroupCellByMaterial(cell, scene, opts);
        CHECK(cell.materialGroups.empty());
        CHECK(!cell.hasContent);
    }

    // ---- Case 2: valid mesh, direct GlbBuilder call with an empty group ----
    {
        aiMesh* m = new aiMesh;
        fillGridMesh(m, 8);
        m->mNumFaces = 0;
        m->mFaces = new aiFace[2];
        addFace(m, 0, 1, 8);   // one valid triangle
        addFace(m, 0, 0, 2);   // one degenerate
        scene->mMeshes[0] = m;

        GridCell cell;
        cell.hasContent = true;
        cell.level = 0;
        addInstance(cell, 0);
        MaterialGrouper::GroupCellByMaterial(cell, scene, opts);
        CHECK(cell.hasContent);
        CHECK(cell.materialGroups.size() == 1);

        // Inject an empty-primitive group alongside the valid one.
        MergedMeshGroup empty;
        empty.positions.assign(60 * 3, 1.0f);
        empty.normals.assign(60 * 3, 0.0f);
        empty.texcoords.assign(60 * 2, 0.0f);
        empty.materialIndex = 1;
        cell.materialGroups.push_back(empty);

        BinaryBlob glb;
        CHECK(GlbBuilder::Build(cell.materialGroups, glb, "", false));

        // Extract the GLB JSON chunk (12-byte header + 8-byte chunk header).
        CHECK(glb.size() > 28);
        uint32_t jsonLen;
        std::memcpy(&jsonLen, glb.ptr() + 12, 4);
        std::string json(reinterpret_cast<const char*>(glb.ptr() + 20), jsonLen);

        // Exactly one primitive must be exported; no zero-count accessors.
        size_t primCount = 0, pos = 0;
        while ((pos = json.find("\"primitives\"", pos)) != std::string::npos)
            { ++primCount; ++pos; }
        CHECK(primCount == 1);
        CHECK(json.find("\"count\":0") == std::string::npos);
    }

    // ---- Case 3: empty group alone must not build a GLB ----
    {
        std::vector<MergedMeshGroup> groups;
        MergedMeshGroup empty;
        empty.positions.assign(60 * 3, 1.0f);
        empty.indices.clear();
        groups.push_back(empty);
        BinaryBlob glb;
        CHECK(!GlbBuilder::Build(groups, glb, "", false));
    }

    delete scene;
    if (g_fail == 0)
        cout << "PASS: empty primitives are not exported\n";
    return g_fail == 0 ? 0 : 1;
}
