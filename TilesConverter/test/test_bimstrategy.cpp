// Unit tests for the BIM binding strategy layer (BIM_BINDING_ARCHITECTURE.md).
// Covers:
//   1. BindingStrategyRegistry::For() dispatch (format id normalization).
//   2. Per-strategy Describe() — the --bim-formats single source of truth
//      must match the §3.2.1 matrix (IFC guid / FBX UDP / glTF extras /
//      OBJ 3DS name-only / Generic).
//   3. IfcBindingStrategy::ResolveObjectId — "IfcWall_..._GUID" name tail,
//      negative cases (short tail, no underscore, numeric suffix).
//   4. Fbx/Gltf strategies — aiNode::mMetaData id keys, objectName fallback.
//   5. ThreeDs — "$$$DUMMY" falls back to meshName.
//   6. SidecarTableSource — CSV parse, first-row-wins dedup, typed cells,
//      no numeric promotion of quoted-lookalike strings.
//   7. BimBindingPipeline::BindAll — merge semantics (sidecar overrides
//      scene), reserved columns (objectId forced string), idSource counts.
//   8. FeatureBatchTable::AssignBatchId — dedup + shared empty row (D6).
//   9. --bim-formats output text.

#include "../TilesConverter/BimBindingPipeline.h"
#include "../TilesConverter/BimFormatStrategy.h"
#include "../MeshProjectionErrorCorrector/TileDataTypes.h"

#include <assimp/scene.h>
#include <assimp/metadata.h>

#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>

using std::cout;
using std::endl;

static int g_fail = 0;
#define CHECK(cond) do { if (!(cond)) { std::cout << "FAIL " << __FILE__ << ":" << __LINE__ << "  " << #cond << "\n"; g_fail++; } } while (0)

// ---------------------------------------------------------------------------
// aiNode metadata construction helpers
// ---------------------------------------------------------------------------

static aiNode* MakeNode(const std::string& name, const std::vector<std::pair<std::string, aiString>>& props = {})
{
    aiNode* n = new aiNode(name);
    if (!props.empty())
    {
        n->mMetaData = new aiMetadata();
        for (auto& kv : props)
            n->mMetaData->Add(kv.first.c_str(), kv.second);
    }
    return n;
}

static aiString S(const std::string& s) { aiString a; a.Set(s); return a; }

static MeshInstance MakeInstance(const std::string& objectName,
                                 const std::string& meshName,
                                 const void* node)
{
    MeshInstance inst;
    inst.meshIndex = 0;
    inst.objectName = objectName;
    inst.meshName = meshName;
    inst.identityNode = node;
    return inst;
}

// ---------------------------------------------------------------------------
// 1. Registry dispatch
// ---------------------------------------------------------------------------
static void TestRegistryDispatch()
{
    CHECK(&BindingStrategyRegistry::For(".ifc")    == &BindingStrategyRegistry::For("IFC"));
    CHECK(&BindingStrategyRegistry::For("IfcImporter") == &BindingStrategyRegistry::For("ifc"));
    CHECK(&BindingStrategyRegistry::For("model.FBX") == &BindingStrategyRegistry::For("fbx"));
    CHECK(&BindingStrategyRegistry::For(".glb")    == &BindingStrategyRegistry::For("glTF2"));
    CHECK(&BindingStrategyRegistry::For("obj")     != &BindingStrategyRegistry::For("3ds"));
    CHECK(&BindingStrategyRegistry::For("")        == &BindingStrategyRegistry::For("unknown-format"));

    // Extension/basename beats path substrings (review #14 regression:
    // "/data/ifc_archive/model.fbx" used to resolve to the IFC strategy).
    CHECK(&BindingStrategyRegistry::For("/data/ifc_archive/model.fbx") == &BindingStrategyRegistry::For("fbx"));
    CHECK(&BindingStrategyRegistry::For("C:\\jobs\\fbx_backup\\wall.obj") == &BindingStrategyRegistry::For("obj"));
    CHECK(&BindingStrategyRegistry::For("ifc_export.glb") == &BindingStrategyRegistry::For("gltf2"));

    auto all = BindingStrategyRegistry::All();
    CHECK(all.size() == 6);
    // Stable order, unique formats
    std::vector<std::string> fmts;
    for (auto* s : all) fmts.push_back(s->Describe().format);
    CHECK(fmts == std::vector<std::string>({ "IFC", "FBX", "glTF2", "OBJ", "3DS", "Generic" }));
}

// ---------------------------------------------------------------------------
// 2. Describe() matrix (§3.2.1)
// ---------------------------------------------------------------------------
static void TestDescribeMatrix()
{
    auto ifc = BindingStrategyRegistry::For("ifc").Describe();
    CHECK(std::string(ifc.format) == "IFC");
    CHECK(ifc.nativeGuid == true);
    CHECK(ifc.sceneMetadata == true);
    CHECK(std::string(ifc.idSource).find("name tail") != std::string::npos);

    auto fbx = BindingStrategyRegistry::For("fbx").Describe();
    CHECK(std::string(fbx.format) == "FBX");
    CHECK(fbx.nativeGuid == false);
    CHECK(fbx.sceneMetadata == true);

    auto gltf = BindingStrategyRegistry::For("gltf2").Describe();
    CHECK(std::string(gltf.format) == "glTF2");
    CHECK(gltf.nativeGuid == false);
    CHECK(gltf.sceneMetadata == true);

    auto obj = BindingStrategyRegistry::For("obj").Describe();
    CHECK(std::string(obj.format) == "OBJ");
    CHECK(obj.sceneMetadata == false);

    auto tds = BindingStrategyRegistry::For("3ds").Describe();
    CHECK(std::string(tds.format) == "3DS");
    CHECK(tds.sceneMetadata == false);

    auto gen = BindingStrategyRegistry::For("").Describe();
    CHECK(std::string(gen.format) == "Generic");
    CHECK(gen.sceneMetadata == true);
}

// ---------------------------------------------------------------------------
// 3. IFC id resolution
// ---------------------------------------------------------------------------
static void TestIfcIdResolution()
{
    IfcBindingStrategy ifc;

    // Positive: 22-char IFC base64 GUID tail
    MeshInstance good = MakeInstance("IfcWall_Basic Wall_0a3tXq2nTCwQxVEQOVJqDz", "mesh0", nullptr);
    auto r = ifc.ResolveObjectId(good);
    CHECK(r.objectId == "0a3tXq2nTCwQxVEQOVJqDz");
    CHECK(r.source == IBindingStrategy::IdResolution::Source::NameTail);
    CHECK(ifc.ObjectClass(good) == "IfcWall");

    // Negative: plain trailing number (not a GUID)
    MeshInstance num = MakeInstance("IfcWall_Basic Wall_42", "m", nullptr);
    CHECK(ifc.ResolveObjectId(num).objectId.empty());

    // Negative: no underscore
    MeshInstance plain = MakeInstance("JustAName", "m", nullptr);
    CHECK(ifc.ResolveObjectId(plain).objectId.empty());

    // Negative: empty
    MeshInstance empty = MakeInstance("", "", nullptr);
    CHECK(ifc.ResolveObjectId(empty).objectId.empty());
}

// ---------------------------------------------------------------------------
// 4. FBX / glTF id resolution via metadata
// ---------------------------------------------------------------------------
static void TestMetadataIdResolution()
{
    FbxBindingStrategy fbx;
    GltfBindingStrategy gltf;
    GenericBindingStrategy gen;

    // Node with GlobalId UDP
    aiNode* n1 = MakeNode("Wall", { { "GlobalId", S("fbx-guid-123") } });
    MeshInstance i1 = MakeInstance("Wall", "WallMesh", n1);
    auto r1 = fbx.ResolveObjectId(i1);
    CHECK(r1.objectId == "fbx-guid-123");
    CHECK(r1.source == IBindingStrategy::IdResolution::Source::MetadataKey);

    // No metadata: falls back to objectName
    aiNode* n2 = MakeNode("Wall001");
    MeshInstance i2 = MakeInstance("Wall001", "WallMesh", n2);
    auto r2 = fbx.ResolveObjectId(i2);
    CHECK(r2.objectId == "Wall001");
    CHECK(r2.source == IBindingStrategy::IdResolution::Source::ObjectName);

    // glTF extras id
    aiNode* n3 = MakeNode("node0", { { "ElementId", S("e-42") } });
    MeshInstance i3 = MakeInstance("node0", "mesh0", n3);
    CHECK(gltf.ResolveObjectId(i3).objectId == "e-42");

    // JoinKeys: resolved id first, then objectName, then meshName
    auto keys = fbx.JoinKeys(i1);
    CHECK(keys.size() >= 1 && keys[0] == "fbx-guid-123");
    auto keys2 = fbx.JoinKeys(i2);
    CHECK(keys2.size() >= 1 && keys2[0] == "Wall001");

    // Scene row: metadata present
    auto row = fbx.CollectSceneRow(i1, false);
    CHECK(row != nullptr);
    CHECK(row->size() == 1);
    CHECK((*row)[0].first == "GlobalId");
    CHECK((*row)[0].second.type == BimValue::Type::String);
    CHECK((*row)[0].second.s == "fbx-guid-123");

    // Inheritance: parent metadata inherited, child overrides
    aiNode* parent = MakeNode("Building", { { "Site", S("A") }, { "Floor", S("1F") } });
    aiNode* child = MakeNode("Wall", { { "Floor", S("2F") } });
    child->mParent = parent;
    MeshInstance i4 = MakeInstance("Wall", "m", child);
    auto row4 = fbx.CollectSceneRow(i4, /*inheritAncestors=*/true);
    CHECK(row4 != nullptr);
    CHECK(row4->size() == 2);
    bool siteFound = false, floorOverridden = false;
    for (auto& kv : *row4)
    {
        if (kv.first == "Site" && kv.second.s == "A") siteFound = true;
        if (kv.first == "Floor" && kv.second.s == "2F") floorOverridden = true;
    }
    CHECK(siteFound);
    CHECK(floorOverridden);

    // No inheritance: only own metadata
    auto row5 = fbx.CollectSceneRow(i4, false);
    CHECK(row5 != nullptr && row5->size() == 1);

    // No metadata anywhere: nullptr row
    aiNode* n6 = MakeNode("lonely");
    MeshInstance i6 = MakeInstance("lonely", "m", n6);
    CHECK(fbx.CollectSceneRow(i6, true) == nullptr);

    // NUMERIC UDP values serve as ids (review #5 regression: FBX exporters
    // bake ElementId/UniqueId as typed ints; a String-only lookup lost them
    // and silently fell back to objectName).
    aiNode* n7 = new aiNode("Beam");
    n7->mMetaData = new aiMetadata();
    n7->mMetaData->Add("ElementId", static_cast<int32_t>(9001));
    MeshInstance i7 = MakeInstance("Beam001", "bm", n7);
    auto r7 = fbx.ResolveObjectId(i7);
    CHECK(r7.objectId == "9001");
    CHECK(r7.source == IBindingStrategy::IdResolution::Source::MetadataKey);

    // Nested metadata dictionaries flatten to "Parent.Child" keys with the
    // value type preserved (review #16; AI_AIMETADATA used to be dropped).
    aiNode* n8 = new aiNode("Panel");
    n8->mMetaData = new aiMetadata();
    n8->mMetaData->Add("Top", S("t-value"));
    aiMetadata nested;
    nested.Add("ifcGUID", S("guid-nested-77"));
    n8->mMetaData->Add("Pset", nested);
    MeshInstance i8 = MakeInstance("Panel", "pn", n8);
    auto row8 = fbx.CollectSceneRow(i8, false);
    CHECK(row8 != nullptr);
    bool flatOk = false;
    for (auto& kv : *row8)
        if (kv.first == "Pset.ifcGUID" && kv.second.type == BimValue::Type::String &&
            kv.second.s == "guid-nested-77") flatOk = true;
    CHECK(flatOk);
    // the flattened nested GUID is found by the default id-key candidates
    CHECK(IBindingStrategy::FindKey(*row8, IBindingStrategy::DefaultIdKeys()) == "guid-nested-77");

    // AI_UINT32 must not wrap negatives into int32 (review #16)
    aiNode* n9 = new aiNode("Uint");
    n9->mMetaData = new aiMetadata();
    n9->mMetaData->Add("BigU", static_cast<uint32_t>(4000000000u));
    auto row9 = fbx.CollectSceneRow(MakeInstance("Uint", "u", n9), false);
    bool uintOk = false;
    if (row9)
        for (auto& kv : *row9)
            if (kv.first == "BigU" && kv.second.type == BimValue::Type::Int64 &&
                kv.second.i64 == 4000000000LL) uintOk = true;
    CHECK(uintOk);

    delete n1; delete n2; delete n3; delete parent; delete child; delete n6;
    delete n7; delete n8; delete n9;
}

// ---------------------------------------------------------------------------
// 5. ThreeDs $$$DUMMY
// ---------------------------------------------------------------------------
static void TestThreeDs()
{
    ThreeDsBindingStrategy tds;
    MeshInstance dummy = MakeInstance("$$$DUMMY", "RealMesh", nullptr);
    auto r = tds.ResolveObjectId(dummy);
    CHECK(r.objectId == "RealMesh");

    MeshInstance named = MakeInstance("Object01", "Mesh", nullptr);
    CHECK(tds.ResolveObjectId(named).objectId == "Object01");
}

// ---------------------------------------------------------------------------
// 6. SidecarTableSource
// ---------------------------------------------------------------------------
static void TestSidecar()
{
    const char* csv =
        "name,height,mass,note\n"
        "Wall001,3.5,120,first\n"
        "Wall002,4.25,80,\n"
        "Wall001,9.9,9,dup-ignored\n"
        "Wall003,,55,missing-height\n";
    std::ofstream f("/tmp/mgo_test_bim_sidecar.csv");
    f << csv;
    f.close();

    SidecarTableSource src;
    std::string err;
    CHECK(src.Load("/tmp/mgo_test_bim_sidecar.csv", err));
    CHECK(err.empty());
    CHECK(src.rowCount() == 3);           // Wall001 dedup keeps first
    CHECK(src.Stats().duplicateKeys == 1);

    // Typed cells
    auto r1 = src.RowFor({ "Wall001" });
    CHECK(r1 != nullptr);
    CHECK(r1->size() == 3);
    bool heightOk = false, massOk = false, noteOk = false;
    for (auto& kv : *r1)
    {
        if (kv.first == "height") { heightOk = (kv.second.type == BimValue::Type::Double && kv.second.d == 3.5); }
        if (kv.first == "mass")   { massOk = (kv.second.type == BimValue::Type::Int32 && kv.second.i32 == 120); }
        if (kv.first == "note")   { noteOk = (kv.second.type == BimValue::Type::String && kv.second.s == "first"); }
    }
    CHECK(heightOk);
    CHECK(massOk);
    CHECK(noteOk);

    // Missing height -> column present but empty-string value (skipped at load)
    auto r3 = src.RowFor({ "Wall003" });
    CHECK(r3 != nullptr);
    bool hasHeight = false;
    for (auto& kv : *r3) if (kv.first == "height") hasHeight = true;
    CHECK(!hasHeight);
    bool massOk3 = false;
    for (auto& kv : *r3) if (kv.first == "mass" && kv.second.i32 == 55) massOk3 = true;
    CHECK(massOk3);

    // Join by alternate key (second key matches)
    CHECK(src.RowFor({ "nope", "Wall002" }) != nullptr);

    // Miss
    CHECK(src.RowFor({ "missing" }) == nullptr);

    // Error case: missing file
    SidecarTableSource bad;
    CHECK(!bad.Load("/tmp/mgo_test_bim_does_not_exist_9x.csv", err));
    CHECK(!err.empty());

    // Error case: no header key
    std::ofstream f2("/tmp/mgo_test_bim_sidecar2.csv");
    f2 << ",a,b\nx,1,2\n";
    f2.close();
    SidecarTableSource bad2;
    CHECK(!bad2.Load("/tmp/mgo_test_bim_sidecar2.csv", err));

    std::remove("/tmp/mgo_test_bim_sidecar.csv");
    std::remove("/tmp/mgo_test_bim_sidecar2.csv");
}

// ---------------------------------------------------------------------------
// 7. Pipeline: merge + reserved columns
// ---------------------------------------------------------------------------
static void TestPipeline()
{
    // Scene: FBX node with UDP
    aiNode* n = MakeNode("Wall001", { { "GlobalId", S("g-001") }, { "mat", S("concrete") } });
    MeshInstance inst = MakeInstance("Wall001", "WallMesh", n);

    // Sidecar: height + mat override
    std::ofstream f("/tmp/mgo_test_bim_pipe.csv");
    f << "name,mat,height,storey\nWall001,steel,3.5,2F\n";
    f.close();

    SidecarTableSource sidecar;
    std::string err;
    CHECK(sidecar.Load("/tmp/mgo_test_bim_pipe.csv", err));

    BimBindingPipeline::Options opts;
    opts.collectSceneMetadata = true;
    opts.inheritAncestors = false;
    BimBindingPipeline pipe(BindingStrategyRegistry::For("fbx"), &sidecar, opts);

    std::vector<MeshInstance> instances = { inst };
    BimBindingPipeline::Summary summary;
    auto results = pipe.BindAll(instances, summary);

    CHECK(results.size() == 1);
    auto& res = results[0];
    CHECK(res.row != nullptr);
    CHECK(res.objectId == "g-001");
    CHECK(std::string(res.idSource) == "metadata");
    CHECK(std::string(res.matchSource) == "merged");

    // Sidecar value overrides scene value for same key
    bool matSteel = false, heightD = false, storey = false, objectIdStr = false;
    for (auto& kv : *res.row)
    {
        if (kv.first == "mat" && kv.second.s == "steel") matSteel = true;
        if (kv.first == "height" && kv.second.type == BimValue::Type::Double && kv.second.d == 3.5) heightD = true;
        if (kv.first == "storey" && kv.second.s == "2F") storey = true;
        if (kv.first == "objectId" && kv.second.type == BimValue::Type::String && kv.second.s == "g-001")
            objectIdStr = true;
    }
    CHECK(matSteel);
    CHECK(heightD);
    CHECK(storey);
    CHECK(objectIdStr);   // reserved column, forced string

    // objectName reserved column present
    bool nameCol = false;
    for (auto& kv : *res.row) if (kv.first == "objectName" && kv.second.s == "Wall001") nameCol = true;
    CHECK(nameCol);

    // Summary counters
    CHECK(summary.total == 1);
    CHECK(summary.withSceneRow == 1);
    CHECK(summary.withSidecarRow == 1);
    CHECK(summary.withRow == 1);
    CHECK(summary.withObjectId == 1);
    CHECK(summary.idSourceCounts["metadata"] == 1);

    // instance got the row attached
    CHECK(instances[0].properties == res.row);

    // No sidecar: scene-only row
    BimBindingPipeline pipe2(BindingStrategyRegistry::For("fbx"), nullptr, opts);
    std::vector<MeshInstance> instances2 = { MakeInstance("Wall001", "WallMesh", n) };
    BimBindingPipeline::Summary s2;
    auto res2 = pipe2.BindAll(instances2, s2);
    CHECK(std::string(res2[0].matchSource) == "scene");
    CHECK(res2[0].row != nullptr);

    // No metadata + no sidecar: row == nullptr, matchSource none
    aiNode* bare = MakeNode("Plain");
    std::vector<MeshInstance> instances3 = { MakeInstance("Plain", "m", bare) };
    BimBindingPipeline::Summary s3;
    auto res3 = pipe2.BindAll(instances3, s3);
    CHECK(res3[0].row == nullptr);
    CHECK(std::string(res3[0].matchSource) == "none");
    CHECK(std::string(res3[0].idSource) == "objectName");   // FBX falls back to name

    std::remove("/tmp/mgo_test_bim_pipe.csv");
    delete n; delete bare;
}

// ---------------------------------------------------------------------------
// 6b. Sidecar CSV robustness (review #13/#17/#5): RFC4180 quoting, BOM,
// leading-zero ids, explicit-string quoting, non-UTF-8 repair.
// ---------------------------------------------------------------------------
static void TestSidecarRobustness()
{
    // \xEF\xBB\xBF = UTF-8 BOM; \xB3\xE0 = invalid UTF-8 bytes (GBK-looking).
    std::string csv =
        "\xEF\xBB\xBF" "name,desc,code,note\n"
        "Wall001,\"has, comma and \"\"quotes\"\"\",0123,\"multi\nline\"\n"
        "Wall002,plain,\"9042\",X\xB3\xE0Y\n";
    {
        std::ofstream f("/tmp/mgo_test_bim_robust.csv", std::ios::binary);
        f << csv;
    }

    SidecarTableSource src;
    std::string err;
    CHECK(src.Load("/tmp/mgo_test_bim_robust.csv", err));
    CHECK(src.rowCount() == 2);

    auto r1 = src.RowFor({ "Wall001" });
    CHECK(r1 != nullptr);
    bool descOk = false, codeStrOk = false, multilineOk = false;
    if (r1)
        for (auto& kv : *r1)
        {
            if (kv.first == "desc" && kv.second.s == "has, comma and \"quotes\"") descOk = true;
            // leading-zero token stays a STRING (id-like), never 123 (review #5)
            if (kv.first == "code" && kv.second.type == BimValue::Type::String && kv.second.s == "0123") codeStrOk = true;
            if (kv.first == "note" && kv.second.s.find('\n') != std::string::npos) multilineOk = true;
        }
    CHECK(descOk);
    CHECK(codeStrOk);
    CHECK(multilineOk);

    auto r2 = src.RowFor({ "Wall002" });
    CHECK(r2 != nullptr);
    bool quotedNumStr = false;   // "9042" quoted -> explicit string, not Int32
    if (r2)
        for (auto& kv : *r2)
        {
            if (kv.first == "code" && kv.second.type == BimValue::Type::String && kv.second.s == "9042") quotedNumStr = true;
            // invalid bytes repaired (review #17): GB18030 conversion where
            // iconv is available (real GBK names survive), U+FFFD otherwise;
            // the INVARIANT is valid UTF-8 without the raw invalid bytes.
            if (kv.first == "note")
            {
                bool rawGone = kv.second.s.find('\xB3') == std::string::npos &&
                               kv.second.s.find('\xE0') == std::string::npos;
                bool validUtf8 = true;
                for (size_t i = 0; i < kv.second.s.size(); )
                {
                    unsigned char c = static_cast<unsigned char>(kv.second.s[i]);
                    size_t len = 0;
                    if (c < 0x80) len = 1;
                    else if ((c & 0xE0) == 0xC0 && i + 1 < kv.second.s.size()) len = 2;
                    else if ((c & 0xF0) == 0xE0 && i + 2 < kv.second.s.size()) len = 3;
                    else if ((c & 0xF8) == 0xF0 && i + 3 < kv.second.s.size()) len = 4;
                    if (!len) { validUtf8 = false; break; }
                    bool cont = true;
                    for (size_t k = 1; k < len; ++k)
                        if ((static_cast<unsigned char>(kv.second.s[i + k]) & 0xC0) != 0x80) cont = false;
                    if (!cont) { validUtf8 = false; break; }
                    i += len;
                }
                CHECK(rawGone);
                CHECK(validUtf8);
            }
        }
    CHECK(quotedNumStr);
    CHECK(src.Stats().utf8Replaced >= 1);

    // BOM must not leak into the first header key
    bool firstKeyClean = false;
    if (r1)
        for (auto& kv : *r1)
            if (kv.first == "desc") firstKeyClean = true;   // headers parsed w/o BOM prefix
    CHECK(firstKeyClean);

    std::remove("/tmp/mgo_test_bim_robust.csv");
}

// ---------------------------------------------------------------------------
// 7b. objectId priority chain + reserved-column collisions
// ---------------------------------------------------------------------------
static void TestIdPriorityAndReserved()
{
    // OBJ-style instance: no scene metadata, objectName only (weak ④),
    // sidecar carries a numeric GlobalId column -> priority ③ must win and
    // report idSource "sidecar" (review #6 regression: the old code resolved
    // the weak objectName BEFORE consulting the sidecar, so ③ never won).
    {
        std::ofstream f("/tmp/mgo_test_bim_prio.csv");
        f << "name,GlobalId,height\nPanel,12345,3.2\n";
        f.close();
        SidecarTableSource sidecar;
        std::string err;
        CHECK(sidecar.Load("/tmp/mgo_test_bim_prio.csv", err));

        BimBindingPipeline::Options opts;
        opts.collectSceneMetadata = false;
        BimBindingPipeline pipe(BindingStrategyRegistry::For("obj"), &sidecar, opts);
        std::vector<MeshInstance> insts = { MakeInstance("Panel", "PanelMesh", nullptr) };
        BimBindingPipeline::Summary s;
        auto res = pipe.BindAll(insts, s);
        CHECK(res[0].objectId == "12345");                 // numeric stringified (review #5)
        CHECK(std::string(res[0].idSource) == "sidecar");
        CHECK(s.idSourceCounts["sidecar"] == 1);
        std::remove("/tmp/mgo_test_bim_prio.csv");
    }

    // Reserved-column collision: sidecar column named objectName must be
    // DROPPED (reserved wins) and reported (review #8).
    {
        std::ofstream f("/tmp/mgo_test_bim_res.csv");
        f << "name,objectName,Height\nPanel,HACKED,3.2\n";
        f.close();
        SidecarTableSource sidecar;
        std::string err;
        CHECK(sidecar.Load("/tmp/mgo_test_bim_res.csv", err));

        BimBindingPipeline::Options opts;
        opts.collectSceneMetadata = false;
        BimBindingPipeline pipe(BindingStrategyRegistry::For("obj"), &sidecar, opts);
        std::vector<MeshInstance> insts = { MakeInstance("Panel", "m", nullptr) };
        BimBindingPipeline::Summary s;
        auto res = pipe.BindAll(insts, s);

        size_t nameEntries = 0;
        bool nameWins = false;
        if (res[0].row)
            for (auto& kv : *res[0].row)
            {
                if (kv.first == "objectName")
                {
                    ++nameEntries;
                    if (kv.second.s == "Panel") nameWins = true;
                }
            }
        CHECK(nameEntries == 1);          // no duplicate keys in the row
        CHECK(nameWins);                  // reserved value wins over user column
        CHECK(s.reservedCollisions == 1);
        CHECK(!s.droppedKeys.empty() && s.droppedKeys[0].find("objectName@") == 0);
        std::remove("/tmp/mgo_test_bim_res.csv");
    }

    // --bim-id-property with spaces: trimmed and honored from the sidecar
    // (review #7 regression: " Height , GlobalId " never matched before).
    {
        std::ofstream f("/tmp/mgo_test_bim_keys.csv");
        f << "name,GlobalId,Other\nPanel,G-9,1\n";
        f.close();
        SidecarTableSource sidecar;
        std::string err;
        CHECK(sidecar.Load("/tmp/mgo_test_bim_keys.csv", err));

        BimBindingPipeline::Options opts;
        opts.collectSceneMetadata = false;
        opts.idPropertyKeys = " Nothing , GlobalId ";
        BimBindingPipeline pipe(BindingStrategyRegistry::For("obj"), &sidecar, opts);
        std::vector<MeshInstance> insts = { MakeInstance("Panel", "m", nullptr) };
        BimBindingPipeline::Summary s;
        auto res = pipe.BindAll(insts, s);
        CHECK(res[0].objectId == "G-9");   // trimmed candidates, first hit wins
        CHECK(std::string(res[0].idSource) == "sidecar");
        std::remove("/tmp/mgo_test_bim_keys.csv");
    }

    // Scene-internal metadata (②b via default keys) beats sidecar ID column (③).
    {
        std::ofstream f("/tmp/mgo_test_bim_ovr.csv");
        f << "name,GlobalId\nWall001,sidecar-guid\n";
        f.close();
        SidecarTableSource sidecar;
        std::string err;
        CHECK(sidecar.Load("/tmp/mgo_test_bim_ovr.csv", err));

        aiNode* n = MakeNode("Wall001", { { "GlobalId", S("scene-guid") } });
        BimBindingPipeline::Options opts;
        opts.collectSceneMetadata = true;
        opts.inheritAncestors = false;
        BimBindingPipeline pipe(BindingStrategyRegistry::For("fbx"), &sidecar, opts);
        std::vector<MeshInstance> insts = { MakeInstance("Wall001", "wm", n) };
        BimBindingPipeline::Summary s;
        auto res = pipe.BindAll(insts, s);
        CHECK(res[0].objectId == "scene-guid");            // ② before ③
        CHECK(std::string(res[0].idSource) == "metadata");
        // merged row: sidecar overrides the same key (§4.8(2))
        bool overridden = false;
        if (res[0].row)
            for (auto& kv : *res[0].row)
                if (kv.first == "GlobalId" && kv.second.s == "sidecar-guid") overridden = true;
        CHECK(overridden);
        std::remove("/tmp/mgo_test_bim_ovr.csv");
        delete n;
    }
}

// ---------------------------------------------------------------------------
// 8. FeatureBatchTable dedup + empty row
// ---------------------------------------------------------------------------
static void TestFeatureBatch()
{
    FeatureBatchTable t;

    auto rowA = std::make_shared<BimPropertyRow>(
        BimPropertyRow{ { "objectId", BimValue::MakeString("a") } });
    auto rowA2 = std::make_shared<BimPropertyRow>(
        BimPropertyRow{ { "objectId", BimValue::MakeString("a") } });
    auto rowB = std::make_shared<BimPropertyRow>(
        BimPropertyRow{ { "objectId", BimValue::MakeString("b") } });

    uint32_t b0 = t.AssignBatchId(rowA);
    uint32_t b0again = t.AssignBatchId(rowA2);   // content-equal -> same id
    uint32_t b1 = t.AssignBatchId(rowB);
    uint32_t b0same = t.AssignBatchId(rowA);

    CHECK(b0 == 0);
    CHECK(b0again == 0);
    CHECK(b1 == 1);
    CHECK(b0same == 0);
    CHECK(t.size() == 2);

    // Empty-row path (D6): unmatched instances share ONE row
    uint32_t e1 = t.AssignBatchId(nullptr);
    uint32_t e2 = t.AssignBatchId(nullptr);
    CHECK(e1 == e2);
    CHECK(t.size() == 3);
    CHECK(t.rows[e1] != nullptr);
    CHECK(t.rows[e1]->empty());
}

// ---------------------------------------------------------------------------
// 9. PrintFormats output
// ---------------------------------------------------------------------------
static void TestPrintFormats()
{
    std::string out;
    BimBindingPipeline::PrintFormats(out);
    CHECK(out.find("IFC") != std::string::npos);
    CHECK(out.find("FBX") != std::string::npos);
    CHECK(out.find("glTF2") != std::string::npos);
    CHECK(out.find("OBJ") != std::string::npos);
    CHECK(out.find("3DS") != std::string::npos);
    CHECK(out.find("Generic") != std::string::npos);
}

int main()
{
    TestRegistryDispatch();
    TestDescribeMatrix();
    TestIfcIdResolution();
    TestMetadataIdResolution();
    TestThreeDs();
    TestSidecar();
    TestSidecarRobustness();
    TestPipeline();
    TestIdPriorityAndReserved();
    TestFeatureBatch();
    TestPrintFormats();

    if (g_fail == 0) { cout << "test_bimstrategy: ALL PASSED\n"; return 0; }
    cout << "test_bimstrategy: " << g_fail << " FAILURES\n";
    return 1;
}
