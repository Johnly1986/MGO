// Repro/regression test: an EXTERNAL overflow subtree must not produce 404.
// Builds a GridCell whose overflow root has >= MIN_CONTENT content so the
// converter routes it to an external subtree tileset.json, then asserts every
// content.uri in the written subtree json resolves to an existing file.
#include "../TileBuilder/TileBuilder.h"
#include <assimp/scene.h>
#include <assimp/mesh.h>
#include <assimp/material.h>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

using std::cout;
using std::endl;

static int g_fail = 0;
#define CHECK(cond) do { if (!(cond)) { std::cout << "FAIL " << __FILE__ << ":" << __LINE__ << "\n"; g_fail++; } } while (0)

int main()
{
    using namespace std;

    // ---- Minimal aiScene: 10x10 vertex grid mesh (100 verts >= the 50-vertex
    // content threshold so cells are not dropped by GroupCellByMaterial) ----
    auto* scene = new aiScene;
    scene->mNumMeshes = 1;
    scene->mMeshes = new aiMesh*[1];
    aiMesh* m = new aiMesh;
    const int G = 10;
    m->mNumVertices = static_cast<unsigned int>(G * G);
    m->mVertices = new aiVector3D[G * G];
    for (int iy = 0; iy < G; ++iy)
        for (int ix = 0; ix < G; ++ix)
            m->mVertices[iy * G + ix] = aiVector3D(ix * 0.1f, iy * 0.1f, 0);
    m->mNumFaces = static_cast<unsigned int>((G - 1) * (G - 1) * 2);
    m->mFaces = new aiFace[m->mNumFaces];
    unsigned int fi = 0;
    for (int iy = 0; iy < G - 1; ++iy)
        for (int ix = 0; ix < G - 1; ++ix)
        {
            unsigned int a = iy * G + ix;
            unsigned int b = a + 1;
            unsigned int c = a + G;
            unsigned int d = c + 1;
            m->mFaces[fi].mNumIndices = 3;
            m->mFaces[fi].mIndices = new unsigned int[3]{a, b, c}; ++fi;
            m->mFaces[fi].mNumIndices = 3;
            m->mFaces[fi].mIndices = new unsigned int[3]{b, d, c}; ++fi;
        }
    m->mMaterialIndex = 0;
    scene->mMeshes[0] = m;
    scene->mNumMaterials = 1;
    scene->mMaterials = new aiMaterial*[1];
    scene->mMaterials[0] = new aiMaterial;

    // ---- GridCell: overflow root with 7 content descendants (>= 6 -> external) ----
    GridCell root;   // virtual root
    auto ovf = std::make_unique<GridCell>();
    ovf->isOverflow = true;
    ovf->isExternal = true;   // converter marks cells written under a subdir
    ovf->level = 0;
    ovf->hasContent = true;
    ovf->cellKey = "cell_+000_+000";
    ovf->localBboxMin[0] = -1; ovf->localBboxMin[1] = -1; ovf->localBboxMin[2] = -1;
    ovf->localBboxMax[0] =  1; ovf->localBboxMax[1] =  1; ovf->localBboxMax[2] =  1;
    MeshInstance inst;
    inst.meshIndex = 0;
    inst.worldTransform[0] = 1; inst.worldTransform[5] = 1; inst.worldTransform[10] = 1;
    inst.worldTransform[15] = 1;
    ovf->instances.push_back(inst);
    // children below the overflow root to push descendant content count >= 6
    for (int k = 0; k < 6; ++k)
    {
        auto ch = std::make_unique<GridCell>();
        ch->level = 1;
        ch->hasContent = true;
        ch->localBboxMin[0] = -1; ch->localBboxMin[1] = -1; ch->localBboxMin[2] = -1;
        ch->localBboxMax[0] =  1; ch->localBboxMax[1] =  1; ch->localBboxMax[2] =  1;
        ch->instances.push_back(inst);
        ovf->children.push_back(std::move(ch));
    }
    GridCell* ovfPtr = ovf.get();
    root.children.push_back(std::move(ovf));

    // A second, INLINE grid cell (not external): its b3dm live directly under
    // outputDir and must be referenced inline in the root tileset.
    auto inlineCell = std::make_unique<GridCell>();
    inlineCell->isExternal = false;
    inlineCell->level = 0;
    inlineCell->hasContent = true;
    inlineCell->cellKey = "cell_+001_+000";
    inlineCell->localBboxMin[0] = -1; inlineCell->localBboxMin[1] = -1; inlineCell->localBboxMin[2] = -1;
    inlineCell->localBboxMax[0] =  1; inlineCell->localBboxMax[1] =  1; inlineCell->localBboxMax[2] =  1;
    inlineCell->instances.push_back(inst);
    for (int k = 0; k < 2; ++k)
    {
        auto ch = std::make_unique<GridCell>();
        ch->level = 1;
        ch->hasContent = true;
        ch->localBboxMin[0] = -1; ch->localBboxMin[1] = -1; ch->localBboxMin[2] = -1;
        ch->localBboxMax[0] =  1; ch->localBboxMax[1] =  1; ch->localBboxMax[2] =  1;
        ch->instances.push_back(inst);
        inlineCell->children.push_back(std::move(ch));
    }
    GridCell* inlinePtr = inlineCell.get();
    root.children.push_back(std::move(inlineCell));

    // ---- Build options ----
    TileBuildOptions opts;
    opts.outputDir = "overflow_test_out";
    std::filesystem::create_directories(opts.outputDir);
    opts.tileBaseName = "tile";
    opts.doubleSided = true;
    opts.minBlockDistance = 100.0;
    opts.maxLODLevels = 5;

    // ---- Write b3dm files: external subtree under "overflow", inline at root ----
    if (!TilesetWriter::WriteTiles(*ovfPtr, scene, opts, "overflow"))
    {
        cout << "WriteTiles failed\n";
        return 1;
    }
    if (!TilesetWriter::WriteTiles(*inlinePtr, scene, opts, ""))
    {
        cout << "WriteTiles(inline) failed\n";
        return 1;
    }
    // ---- Generate writes external subtree tilesets + root tileset.json ----
    std::string outJson;
    if (!TilesetWriter::Generate(root, opts, nullptr, outJson))
    {
        cout << "Generate failed\n";
        return 1;
    }

    // ---- Verify every content.uri resolves to an existing file ----
    int checked = 0;
    auto audit = [&checked](const std::string& jsonPath,
                            const std::string& baseDir) {
        std::ifstream f(jsonPath);
        std::string content((std::istreambuf_iterator<char>(f)),
                            std::istreambuf_iterator<char>());
        size_t pos = 0;
        while ((pos = content.find("\"uri\": \"", pos)) != std::string::npos)
        {
            pos += 8;
            size_t end = content.find('"', pos);
            std::string uri = content.substr(pos, end - pos);
            std::string full = baseDir + "/" + uri;
            std::ifstream t(full, std::ios::binary);
            if (!t.good())
                std::cout << "[404] " << full << std::endl;
            CHECK(t.good());
            ++checked;
            pos = end;
        }
    };
    audit("overflow_test_out/overflow/tileset.json", "overflow_test_out/overflow");
    audit("overflow_test_out/tileset.json", "overflow_test_out");
    CHECK(checked > 0);

    delete scene;
    if (g_fail == 0)
        std::cout << "PASS: all " << checked << " uris resolve\n";
    return g_fail == 0 ? 0 : 1;
}
