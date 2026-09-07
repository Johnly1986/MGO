// Unit tests for the BIM batch-table / _BATCHID writers (TileBuilder).
// Covers (BIM_BINDING_ARCHITECTURE.md §4.4, §4.8, §4.6):
//   1. BatchTableWriter column three-state + binary componentType mapping
//      (doc §4.8(3), legacy Batch Table has NO INT64/BOOL):
//      Int32/Int64-in-range -> INT(5124), Double -> DOUBLE(5128),
//      Vec3 -> VEC3/FLOAT(5126); string/bool/null/non-finite -> JSON array;
//      NaN/±Inf values are nulled and counted; all-null -> column omitted.
//   2. b3dm byte layout: sections END on 8-byte boundaries measured from the
//      TILE START (absolute rule, header lengths include padding), total
//      8-aligned, consumer sum-of-lengths finds the glb exactly.
//   3. GlbBuilder _BATCHID: componentType ladder (5121/5123/5126), never
//      5125; odd batchId section lengths must not desync the following
//      FLOAT accessors (all bufferView byteOffsets %4==0).
//   4. Disabled path: GlbBuilder+B3dmBuilder without BIM context produce
//      byte-identical output to the legacy call (master switch D3).

#include "../TileBuilder/TileBuilder.h"
#include <assimp/scene.h>
#include <assimp/mesh.h>
#include <assimp/material.h>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>

using std::cout;
using std::endl;

static int g_fail = 0;
#define CHECK(cond) do { if (!(cond)) { std::cout << "FAIL " << __FILE__ << ":" << __LINE__ << "  " << #cond << "\n"; g_fail++; } } while (0)

// b3dm header field readers (little-endian)
static uint32_t RD32(const std::vector<uint8_t>& b, size_t off)
{
    uint32_t v;
    std::memcpy(&v, b.data() + off, 4);
    return v;
}

// Every b3dm section must END on an 8-byte boundary from the tile start and
// a consumer computing glbOffset = 28 + ftJson + ftBin + btJson + btBin must
// land exactly on the glTF magic (b3dm spec, padding section).
static void CheckB3dmAbsoluteAlignment(const std::vector<uint8_t>& b, const char* where)
{
    size_t ftJson = RD32(b, 12), ftBin = RD32(b, 16);
    size_t btJson = RD32(b, 20), btBin = RD32(b, 24);
    size_t off = 28;
    off += ftJson; if (off % 8 != 0) { std::cout << "FAIL ftJson end not 8-aligned (" << where << ")\n"; g_fail++; }
    off += ftBin;  if (off % 8 != 0) { std::cout << "FAIL ftBin end not 8-aligned (" << where << ")\n"; g_fail++; }
    off += btJson; if (btJson && off % 8 != 0) { std::cout << "FAIL btJson end not 8-aligned (" << where << ")\n"; g_fail++; }
    off += btBin;  if (btBin && off % 8 != 0) { std::cout << "FAIL btBin end not 8-aligned (" << where << ")\n"; g_fail++; }
    if (RD32(b, off) != 0x46546C67 /* "glTF" */)
    { std::cout << "FAIL glb not at summed offset (" << where << ")\n"; g_fail++; }
    if (off % 8 != 0)
    { std::cout << "FAIL glb start not 8-aligned (" << where << ")\n"; g_fail++; }
    if (RD32(b, 8) % 8 != 0)
    { std::cout << "FAIL total byteLength not 8-aligned (" << where << ")\n"; g_fail++; }
}

// Build a small MergedMeshGroup: G*G grid mesh, 2 triangles.
static MergedMeshGroup MakeGroup(int G)
{
    MergedMeshGroup g;
    g.materialIndex = 0;
    for (int iy = 0; iy < G; ++iy)
        for (int ix = 0; ix < G; ++ix)
        {
            g.positions.push_back(static_cast<float>(ix) * 0.1f);
            g.positions.push_back(static_cast<float>(iy) * 0.1f);
            g.positions.push_back(0.0f);
            g.normals.push_back(0.0f); g.normals.push_back(1.0f); g.normals.push_back(0.0f);
            g.texcoords.push_back(0.0f); g.texcoords.push_back(0.0f);
        }
    for (int iy = 0; iy < G - 1; ++iy)
        for (int ix = 0; ix < G - 1; ++ix)
        {
            unsigned int a = iy * G + ix, b = a + 1, c = a + G, d = c + 1;
            g.indices.push_back(a); g.indices.push_back(b); g.indices.push_back(c);
            g.indices.push_back(b); g.indices.push_back(d); g.indices.push_back(c);
        }
    return g;
}

// ---------------------------------------------------------------------------
// 1. BatchTableWriter three-state columns + type mapping
// ---------------------------------------------------------------------------
static void TestBatchTableThreeState()
{
    // Mixed: "h" all-numeric -> binary DOUBLE(5128); "s" has null -> JSON;
    // "gone" all null -> omitted
    std::vector<std::shared_ptr<const BimPropertyRow>> rows;
    rows.push_back(std::make_shared<BimPropertyRow>(BimPropertyRow{
        { "h", BimValue::MakeDouble(1.5) }, { "s", BimValue::MakeString("x") }, { "gone", BimValue::MakeString("") } }));
    rows.push_back(std::make_shared<BimPropertyRow>(BimPropertyRow{
        { "h", BimValue::MakeDouble(2.5) }, { "s", BimValue::MakeString("") } }));
    rows.push_back(std::make_shared<BimPropertyRow>(BimPropertyRow{
        { "h", BimValue::MakeDouble(3.5) }, { "s", BimValue::MakeString("z") }, { "gone", BimValue::MakeString("") } }));

    std::string json;
    std::vector<uint8_t> bin;
    CHECK(BatchTableWriter::Build(rows, json, bin));

    // "h": binary DOUBLE column, 3 rows x 8B = 24B (no float32 downcast)
    CHECK(json.find("\"h\":{\"byteOffset\":0,\"componentType\":5128,\"type\":\"SCALAR\"}") != std::string::npos);
    CHECK(bin.size() == 24);
    double d0;
    std::memcpy(&d0, bin.data(), 8);
    CHECK(d0 == 1.5);

    // "s": JSON array ["x",null,"z"]
    CHECK(json.find("\"s\":[\"x\",null,\"z\"]") != std::string::npos);

    // "gone": omitted
    CHECK(json.find("gone") == std::string::npos);

    // Valid JSON object braces
    CHECK(json.front() == '{' && json.back() == '}');

    // Empty rows -> {}
    std::vector<std::shared_ptr<const BimPropertyRow>> empty;
    std::string j2; std::vector<uint8_t> b2;
    CHECK(BatchTableWriter::Build(empty, j2, b2));
    CHECK(j2 == "{}");
    CHECK(b2.empty());

    // Vec3 column -> binary VEC3/FLOAT (24 bytes for 2 rows)
    std::vector<std::shared_ptr<const BimPropertyRow>> vrows;
    vrows.push_back(std::make_shared<BimPropertyRow>(BimPropertyRow{
        { "v", BimValue::MakeVec3(1, 2, 3) } }));
    vrows.push_back(std::make_shared<BimPropertyRow>(BimPropertyRow{
        { "v", BimValue::MakeVec3(4, 5, 6) } }));
    std::string j3; std::vector<uint8_t> b3;
    CHECK(BatchTableWriter::Build(vrows, j3, b3));
    CHECK(j3.find("\"v\":{\"byteOffset\":0,\"componentType\":5126,\"type\":\"VEC3\"}") != std::string::npos);
    CHECK(b3.size() == 24);

    // Int32 column present in every row -> binary INT(5124) (doc §4.8(3):
    // legacy Batch Table has no INT64/BOOL binary type). "b" Bool stays JSON.
    std::vector<std::shared_ptr<const BimPropertyRow>> irows;
    irows.push_back(std::make_shared<BimPropertyRow>(BimPropertyRow{
        { "b", BimValue::MakeBool(true) }, { "n", BimValue::MakeInt32(7) } }));
    irows.push_back(std::make_shared<BimPropertyRow>(BimPropertyRow{
        { "b", BimValue::MakeBool(false) }, { "n", BimValue::MakeInt32(9) } }));
    std::string j4; std::vector<uint8_t> b4;
    size_t nullified4 = 12345;
    CHECK(BatchTableWriter::Build(irows, j4, b4, &nullified4));
    CHECK(j4.find("\"b\":[true,false]") != std::string::npos);
    CHECK(j4.find("\"n\":{\"byteOffset\":0,\"componentType\":5124,\"type\":\"SCALAR\"}") != std::string::npos);
    CHECK(b4.size() == 8);
    int32_t n0, n1;
    std::memcpy(&n0, b4.data(), 4);
    std::memcpy(&n1, b4.data() + 4, 4);
    CHECK(n0 == 7 && n1 == 9);
    CHECK(nullified4 == 0);

    // Int64 beyond int32 range -> JSON array with EXACT digits (no INT64 in
    // binary; no float64 rounding).
    std::vector<std::shared_ptr<const BimPropertyRow>> lrows;
    lrows.push_back(std::make_shared<BimPropertyRow>(BimPropertyRow{
        { "guid", BimValue::MakeInt64(1099511627776LL) } }));
    lrows.push_back(std::make_shared<BimPropertyRow>(BimPropertyRow{
        { "guid", BimValue::MakeInt64(1099511627777LL) } }));
    std::string j6; std::vector<uint8_t> b6;
    CHECK(BatchTableWriter::Build(lrows, j6, b6));
    CHECK(j6.find("\"guid\":[1099511627776,1099511627777]") != std::string::npos);
    CHECK(b6.empty());

    // NaN/±Inf -> null + demoted to JSON + counted (review #3)
    std::vector<std::shared_ptr<const BimPropertyRow>> nrows;
    nrows.push_back(std::make_shared<BimPropertyRow>(BimPropertyRow{
        { "vol", BimValue::MakeDouble(0.0 / 0.0) }, { "k", BimValue::MakeInt32(1) } }));
    nrows.push_back(std::make_shared<BimPropertyRow>(BimPropertyRow{
        { "vol", BimValue::MakeDouble(1.0 / 0.0) }, { "k", BimValue::MakeInt32(2) } }));
    std::string j7; std::vector<uint8_t> b7;
    size_t nullified7 = 0;
    CHECK(BatchTableWriter::Build(nrows, j7, b7, &nullified7));
    CHECK(j7.find("\"vol\":[null,null]") != std::string::npos);
    CHECK(j7.find("nan") == std::string::npos && j7.find("inf") == std::string::npos);
    CHECK(nullified7 == 2);
    // numeric "k" column unaffected -> INT binary column, 8 bytes
    CHECK(j7.find("\"k\":{\"byteOffset\":0,\"componentType\":5124,\"type\":\"SCALAR\"}") != std::string::npos);
    CHECK(b7.size() == 8);

    // String escaping incl. control chars; JSON double precision round-trips
    std::vector<std::shared_ptr<const BimPropertyRow>> erows;
    erows.push_back(std::make_shared<BimPropertyRow>(BimPropertyRow{
        { "quote\"key", BimValue::MakeString("quote\"back\\slash\n\x01") } }));
    std::string j5; std::vector<uint8_t> b5;
    CHECK(BatchTableWriter::Build(erows, j5, b5));
    CHECK(j5.find("quote\\\"key") != std::string::npos);          // key escaped (review #18-class)
    CHECK(j5.find("quote\\\"back\\\\slash\\n\\u0001") != std::string::npos);

    // JSON doubles keep 17-digit precision (demote-to-JSON path: the column
    // has a missing row, so the present value must not lose digits).
    std::vector<std::shared_ptr<const BimPropertyRow>> prows;
    prows.push_back(std::make_shared<BimPropertyRow>(BimPropertyRow{
        { "x", BimValue::MakeDouble(1234567.87654321) } }));
    prows.push_back(std::make_shared<BimPropertyRow>(BimPropertyRow{
        { "other", BimValue::MakeInt32(1) } }));
    std::string j8; std::vector<uint8_t> b8;
    CHECK(BatchTableWriter::Build(prows, j8, b8));
    CHECK(j8.find("[1234567.87654321,null]") != std::string::npos);   // no float32-style rounding in JSON
}

// ---------------------------------------------------------------------------
// 2. b3dm BIM layout (absolute 8-byte alignment)
// ---------------------------------------------------------------------------
static void TestB3dmLayout()
{
    // Minimal glb: 12 header + empty json chunk + empty bin chunk
    BinaryBlob glb;
    glb.data.resize(12 + 8 + 8);
    glb.data[0] = 'g'; glb.data[1] = 'l'; glb.data[2] = 'T'; glb.data[3] = 'F';

    std::string btJson = "{\"a\":[1,2]}";
    std::vector<uint8_t> btBin(5, 0xAB);   // intentionally NOT 8-aligned

    BinaryBlob out;
    CHECK(B3dmBuilder::Build(glb, 3, btJson, btBin, out));

    // magic "b3dm"
    CHECK(out.data[0] == 'b' && out.data[1] == '3' && out.data[2] == 'd' && out.data[3] == 'm');
    CHECK(RD32(out.data, 4) == 1);   // version

    CheckB3dmAbsoluteAlignment(out.data, "bt 11B + btb 5B");

    size_t totalLen   = RD32(out.data, 8);
    size_t ftJsonLen  = RD32(out.data, 12);
    size_t btJsonLen  = RD32(out.data, 20);
    size_t btBinLen   = RD32(out.data, 24);

    CHECK(RD32(out.data, 16) == 0);        // ftBinLen
    CHECK(totalLen == out.data.size());
    CHECK(totalLen % 8 == 0);

    // FT JSON: 28 + len must be 8-aligned -> 20B ({"BATCH_LENGTH":3} = 18B,
    // pad 2 spaces; 28+20 = 48).
    CHECK((28 + ftJsonLen) % 8 == 0);
    CHECK(ftJsonLen == 20);
    CHECK(std::memcmp(out.data.data() + 28, "{\"BATCH_LENGTH\":3}", 18) == 0);
    for (size_t i = 18; i < ftJsonLen; ++i)
        CHECK(out.data[28 + i] == 0x20);

    // BT JSON starts at 48 (8-aligned), content intact, space-padded
    size_t btJsonOff = 28 + ftJsonLen;
    CHECK(btJsonOff == 48);
    CHECK((btJsonOff + btJsonLen) % 8 == 0);
    CHECK(std::memcmp(out.data.data() + btJsonOff, btJson.data(), btJson.size()) == 0);
    for (size_t i = btJson.size(); i < btJsonLen; ++i)
        CHECK(out.data[btJsonOff + i] == 0x20);

    // BT binary: header reports the PADDED length (consumers sum lengths);
    // 5 payload bytes + 3 zeros.
    size_t btBinOff = btJsonOff + btJsonLen;
    CHECK(btBinLen == 8);
    CHECK(std::memcmp(out.data.data() + btBinOff, btBin.data(), 5) == 0);
    for (size_t i = 5; i < btBinLen; ++i)
        CHECK(out.data[btBinOff + i] == 0);

    size_t glbOff = btBinOff + btBinLen;
    CHECK(glbOff % 8 == 0);
    CHECK(std::memcmp(out.data.data() + glbOff, glb.ptr(), glb.size()) == 0);
    CHECK(glbOff + glb.size() <= totalLen);

    // Empty batch table JSON => both BT lengths zero (spec rule), glb right
    // after FT.
    BinaryBlob out2;
    CHECK(B3dmBuilder::Build(glb, 1, "", std::vector<uint8_t>(7, 1), out2));
    CHECK(RD32(out2.data, 20) == 0);
    CHECK(RD32(out2.data, 24) == 0);
    CheckB3dmAbsoluteAlignment(out2.data, "empty bt");
    CHECK(std::memcmp(out2.data.data() + 28 + RD32(out2.data, 12), glb.ptr(), 4) == 0);
}

// ---------------------------------------------------------------------------
// 3. GlbBuilder _BATCHID
// ---------------------------------------------------------------------------
static std::string ExtractGlbJson(const BinaryBlob& glb)
{
    uint32_t jsonLen;
    std::memcpy(&jsonLen, glb.data.data() + 12, 4);
    return std::string(reinterpret_cast<const char*>(glb.data.data()) + 20,
                       jsonLen - (jsonLen % 4));
}

// Every glTF bufferView byteOffset must be a multiple of 4 (all our
// components are FLOAT/UBYTE indices/USHORT — review #2: a UBYTE batchId
// section ending on an odd offset desyncs the NEXT primitive's FLOAT
// accessors and viewers reject the whole glb).
static void CheckAllBufferViewsAligned4(const std::string& json, const char* where)
{
    size_t pos = 0;
    int seen = 0;
    while ((pos = json.find("\"byteOffset\":", pos)) != std::string::npos)
    {
        pos += 13;
        long v = std::strtol(json.c_str() + pos, nullptr, 10);
        ++seen;
        if (v % 4 != 0)
        {
            std::cout << "FAIL " << where << ": bufferView byteOffset " << v
                      << " not 4-aligned\n";
            g_fail++;
        }
    }
    CHECK(seen > 0);
}

static void TestGlbBatchId()
{
    std::vector<MergedMeshGroup> groups;
    groups.push_back(MakeGroup(4));   // 16 verts
    groups.push_back(MakeGroup(5));  // 25 verts — odd UBYTE batchId length!
    groups.push_back(MakeGroup(4));  // 16 verts — starts after the odd section

    for (auto& g : groups)
    {
        g.batchIds.assign(g.vertexCount(), 0);
        g.batchIds[g.vertexCount() - 1] = 1;
    }

    GlbBuilder::BatchIdContext ctx;
    ctx.batchLength = 2;

    BinaryBlob glb;
    CHECK(GlbBuilder::Build(groups, glb, "", false, &ctx));
    std::string json = ExtractGlbJson(glb);

    // componentType ladder: 2 rows -> 5121, never 5125
    CHECK(json.find("\"componentType\":5121") != std::string::npos);
    CHECK(json.find("_BATCHID") != std::string::npos);

    // ALIGNMENT REGRESSION (review #2): with a 25-byte UBYTE batchId section
    // the following POSITION/NORMAL (FLOAT) bufferViews must stay 4-aligned.
    CheckAllBufferViewsAligned4(json, "3-group UBYTE ladder");

    // Accessor count: 6 per group (pos/norm/uv/idx/batchid + texcoord? — count
    // componentType occurrences)
    size_t nAcc = 0;
    for (size_t p = 0; p < json.size(); ++p)
        if (json.compare(p, 14, "\"componentType") == 0) nAcc++;
    CHECK(nAcc == 15);   // 5 per group x 3

    // Every primitive carries _BATCHID consistently
    size_t nBATCHID = 0;
    for (size_t p = 0; p + 8 <= json.size(); ++p)
        if (json.compare(p, 8, "_BATCHID") == 0) nBATCHID++;
    CHECK(nBATCHID == 3);

    // Ladder 5123 for 300 rows (UNSIGNED_SHORT)
    GlbBuilder::BatchIdContext ctx2;
    ctx2.batchLength = 300;
    BinaryBlob glb2;
    CHECK(GlbBuilder::Build(groups, glb2, "", false, &ctx2));
    std::string json2 = ExtractGlbJson(glb2);
    CHECK(json2.find("\"componentType\":5123") != std::string::npos);
    CheckAllBufferViewsAligned4(json2, "3-group USHORT ladder");

    // Ladder 5126 for huge batch counts (FLOAT)
    GlbBuilder::BatchIdContext ctx3;
    ctx3.batchLength = 100000;
    BinaryBlob glb3;
    CHECK(GlbBuilder::Build(groups, glb3, "", false, &ctx3));
    std::string json3 = ExtractGlbJson(glb3);
    CHECK(json3.find("\"componentType\":5126,\"count\":16,\"type\":\"SCALAR\"") != std::string::npos ||
          json3.find("\"componentType\":5126,\"count\":25,\"type\":\"SCALAR\"") != std::string::npos);
    CheckAllBufferViewsAligned4(json3, "3-group FLOAT ladder");
}

// ---------------------------------------------------------------------------
// 4. Disabled path: byte-identical (D3)
// ---------------------------------------------------------------------------
static void TestDisabledByteIdentical()
{
    std::vector<MergedMeshGroup> groups;
    groups.push_back(MakeGroup(4));
    groups.push_back(MakeGroup(6));

    BinaryBlob glbNew, b3dmNew;
    CHECK(GlbBuilder::Build(groups, glbNew, "", false, nullptr));
    CHECK(B3dmBuilder::Build(glbNew, b3dmNew));

    // Legacy call must equal the new call with bim=nullptr
    BinaryBlob glbOld, b3dmOld;
    CHECK(GlbBuilder::Build(groups, glbOld, "", false));
    CHECK(B3dmBuilder::Build(glbOld, b3dmOld));

    CHECK(glbNew.data == glbOld.data);
    CHECK(b3dmNew.data == b3dmOld.data);

    // Legacy b3dm shape: BATCH_LENGTH:0, 20-byte FT JSON, glb at 48.
    CHECK(RD32(b3dmOld.data, 12) == 20);    // ftJsonLen
    CHECK(RD32(b3dmOld.data, 20) == 0);     // btJsonLen
    CHECK(RD32(b3dmOld.data, 24) == 0);     // btBinLen
    CHECK(std::memcmp(b3dmOld.data.data() + 28, "{\"BATCH_LENGTH\":0}  ", 20) == 0);
    CHECK(std::memcmp(b3dmOld.data.data() + 48, "glTF", 4) == 0);

    // Group with batchIds set but batchLength=0 (binding disabled):
    // GlbBuilder must NOT emit _BATCHID (context disabled).
    std::vector<MergedMeshGroup> g2;
    g2.push_back(MakeGroup(4));
    g2[0].batchIds.assign(g2[0].vertexCount(), 7);
    GlbBuilder::BatchIdContext off;
    off.batchLength = 0;
    BinaryBlob glbOff;
    CHECK(GlbBuilder::Build(g2, glbOff, "", false, &off));
    std::string jsonOff = ExtractGlbJson(glbOff);
    CHECK(jsonOff.find("_BATCHID") == std::string::npos);
}

// ---------------------------------------------------------------------------
// 5. GroupCellByMaterial batch ids through the whole pipeline
// ---------------------------------------------------------------------------
static void TestGroupCellBatchIds()
{
    // Scene with 2 meshes, 2 nodes (with property rows), one material
    auto* scene = new aiScene;
    scene->mNumMeshes = 2;
    scene->mMeshes = new aiMesh*[2];
    scene->mNumMaterials = 1;
    scene->mMaterials = new aiMaterial*[1];
    scene->mMaterials[0] = new aiMaterial;

    for (int m = 0; m < 2; ++m)
    {
        aiMesh* mesh = new aiMesh;
        const int G = 5;
        mesh->mNumVertices = G * G;
        mesh->mVertices = new aiVector3D[G * G];
        for (int iy = 0; iy < G; ++iy)
            for (int ix = 0; ix < G; ++ix)
                mesh->mVertices[iy * G + ix] = aiVector3D(ix * 0.1f, iy * 0.1f, 0);
        mesh->mNumFaces = 2 * (G - 1) * (G - 1);
        mesh->mFaces = new aiFace[mesh->mNumFaces];
        mesh->mMaterialIndex = 0;
        unsigned int fi = 0;
        for (int iy = 0; iy < G - 1; ++iy)
            for (int ix = 0; ix < G - 1; ++ix)
            {
                unsigned int a = iy * G + ix, b = a + 1, c = a + G, d = c + 1;
                aiFace& f1 = mesh->mFaces[fi++];
                f1.mNumIndices = 3;
                f1.mIndices = new unsigned int[3]{a, b, c};
                aiFace& f2 = mesh->mFaces[fi++];
                f2.mNumIndices = 3;
                f2.mIndices = new unsigned int[3]{b, d, c};
            }
        scene->mMeshes[m] = mesh;
    }

    GridCell cell;
    cell.hasContent = true;

    auto rowA = std::make_shared<BimPropertyRow>(BimPropertyRow{
        { "objectId", BimValue::MakeString("a") }, { "h", BimValue::MakeDouble(1) } });
    auto rowB = std::make_shared<BimPropertyRow>(BimPropertyRow{
        { "objectId", BimValue::MakeString("b") }, { "h", BimValue::MakeDouble(2) } });

    for (int m = 0; m < 2; ++m)
    {
        MeshInstance inst;
        inst.meshIndex = m;
        inst.worldTransform[0] = 1; inst.worldTransform[5] = 1;
        inst.worldTransform[10] = 1; inst.worldTransform[15] = 1;
        inst.properties = (m == 0) ? rowA : rowB;
        cell.instances.push_back(inst);
    }

    TileBuildOptions opts;
    MaterialGrouper::GroupCellByMaterial(cell, scene, opts);

    CHECK(!cell.materialGroups.empty());

    // One material -> one group containing both instances' vertices
    size_t totalVerts = 0;
    uint32_t distinct = 0;
    bool sawA = false, sawB = false;
    for (auto& g : cell.materialGroups)
    {
        totalVerts += g.vertexCount();
        CHECK(g.batchIds.size() == g.vertexCount());
        // batch ids must be constant per instance span (25 verts each)
        for (size_t i = 0; i < g.batchIds.size(); ++i)
            CHECK(g.batchIds[i] == g.batchIds[i / 25 * 25]);
        for (auto id : g.batchIds)
        {
            if (id == 0) sawA = true;
            if (id == 1) sawB = true;
            if (id > distinct) distinct = id;
        }
    }
    CHECK(totalVerts == 50);
    CHECK(sawA && sawB);
    CHECK(distinct == 1);                  // ids 0 and 1 -> max 1
    CHECK(cell.featureBatch.size() == 2);  // two distinct rows

    // Full BIM path: GlbBuilder + B3dmBuilder with batch table
    GlbBuilder::BatchIdContext ctx;
    ctx.batchLength = static_cast<uint32_t>(cell.featureBatch.size());
    BinaryBlob glb;
    CHECK(GlbBuilder::Build(cell.materialGroups, glb, "", false, &ctx));
    std::string json = ExtractGlbJson(glb);
    CHECK(json.find("_BATCHID") != std::string::npos);
    CheckAllBufferViewsAligned4(json, "pipeline group");

    std::string btJson;
    std::vector<uint8_t> btBin;
    CHECK(BatchTableWriter::Build(cell.featureBatch.rows, btJson, btBin));
    BinaryBlob b3dm;
    CHECK(B3dmBuilder::Build(glb, ctx.batchLength, btJson, btBin, b3dm));
    CheckB3dmAbsoluteAlignment(b3dm.data, "pipeline tile");
    // absolute-alignment regression (the old check `RD32(...,12)%8==0` was
    // wrong: with a 28B header even lengths leave the glb at offset %8==4)
    CHECK(RD32(b3dm.data, 12) == 20);

    // cleanup: aiScene destructor owns and deletes meshes/materials/faces
    delete scene;
}

int main()
{
    TestBatchTableThreeState();
    TestB3dmLayout();
    TestGlbBatchId();
    TestDisabledByteIdentical();
    TestGroupCellBatchIds();

    if (g_fail == 0) { cout << "test_batchtable: ALL PASSED\n"; return 0; }
    cout << "test_batchtable: " << g_fail << " FAILURES\n";
    return 1;
}
