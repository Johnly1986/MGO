// Copyright Johnlyon
//
// OSGBConverter unit tests — parser logic, data structures, vendor detection
//
// Build: g++ -std=c++17 -I.. -I../.. -o test_unit test_unit.cpp \
//        ../MetadataParser.cpp ../PlatformCompat.cpp ../VendorHandlerFactory.cpp \
//        ../ContextCaptureHandler.cpp ../DJITerraHandler.cpp \
//        ../../MeshProjectionErrorCorrector/CProjectionEngine.cpp \
//        ../../MeshProjectionErrorCorrector/AxisMapper.cpp \
//        -DOSGB_UNIT_TEST
//

#include "../MetadataParser.h"
#include "../PlatformCompat.h"
#include "../IVendorHandler.h"
#include "../OSGBTileData.h"
#include "../ContextCaptureHandler.h"
#include "../DJITerraHandler.h"
#include "../OSGBConverter.h"
#include "../MeshProjectionErrorCorrector/TileDataTypes.h"
#include "../OSGBCellBuilder.h"

#include <iostream>
#include <cstring>
#include <cassert>
#include <cmath>
#include <vector>
#include <string>
#include <set>
#include <functional>

static int g_pass = 0, g_fail = 0;
#define TEST(name) void test_##name()
#define RUN(name) do { std::cout << "  " #name "... "; test_##name(); } while(0)
#define CHECK(cond) do { if (cond) { std::cout << "PASS" << std::endl; g_pass++; } \
    else { std::cout << "FAIL (" << __LINE__ << ")" << std::endl; g_fail++; } } while(0)
#define CHECK_EQ(a, b) CHECK((a) == (b))
#define CHECK_CLOSE(a, b, eps) CHECK(std::fabs((a)-(b)) < (eps))

// =====================================================================
// MetadataParser tests
// =====================================================================

TEST(parse_srs_epsg) {
    int code = MetadataParser::ParseEPSGCode("EPSG:4547");
    CHECK_EQ(code, 4547);
}

TEST(parse_srs_epsg_lower) {
    int code = MetadataParser::ParseEPSGCode("epsg:4548");
    CHECK_EQ(code, 4548);
}

TEST(parse_srs_epsg_bad) {
    int code = MetadataParser::ParseEPSGCode("ENU:22.6,113.0");
    CHECK_EQ(code, 0);
}

TEST(parse_srs_epsg_empty) {
    int code = MetadataParser::ParseEPSGCode("");
    CHECK_EQ(code, 0);
}

TEST(parse_enu_origin) {
    double lat, lon;
    bool ok = MetadataParser::ParseENUOrigin("ENU:22.64785,113.06277", lat, lon);
    CHECK(ok && std::fabs(lat - 22.64785) < 1e-5 && std::fabs(lon - 113.06277) < 1e-5);
}

TEST(parse_enu_origin_lower) {
    double lat, lon;
    bool ok = MetadataParser::ParseENUOrigin("enu:30.5,120.8", lat, lon);
    CHECK(ok && std::fabs(lat - 30.5) < 1e-5 && std::fabs(lon - 120.8) < 1e-5);
}

TEST(parse_enu_not_enu) {
    double lat, lon;
    bool ok = MetadataParser::ParseENUOrigin("EPSG:4547", lat, lon);
    CHECK(!ok);
}

TEST(parse_origin_3d) {
    double x, y, z;
    bool ok = MetadataParser::ParseOrigin("500000.0, 3400000.0, 100.5", x, y, z);
    CHECK(ok && std::fabs(x - 500000.0) < 1e-6 && std::fabs(y - 3400000.0) < 1e-6 && std::fabs(z - 100.5) < 1e-6);
}

TEST(parse_origin_2d) {
    double x, y, z;
    bool ok = MetadataParser::ParseOrigin("517600.0 3421000.0", x, y, z);
    CHECK(ok && std::fabs(x - 517600.0) < 1e-6 && std::fabs(y - 3421000.0) < 1e-6 && z == 0);
}

TEST(detect_projection_root_only) {
    auto mode = MetadataParser::DetectProjectionMode("EPSG:4547");
    CHECK(mode == ProjectionMode::RootOnly);
}

TEST(detect_projection_per_tile) {
    auto mode = MetadataParser::DetectProjectionMode("ENU:22.6,113.0");
    CHECK(mode == ProjectionMode::PerTile);
}

TEST(detect_projection_unknown) {
    auto mode = MetadataParser::DetectProjectionMode("LOCAL_CS");
    CHECK(mode == ProjectionMode::PerTile);
}

// =====================================================================
// Tile path parsing (ContextCapture format)
// =====================================================================

TEST(cc_parse_basic) {
    ContextCaptureHandler h;
    int level, x, y; std::string sub;
    bool ok = h.ParseTilePath("Data/Tile_+005_+002/Tile_+005_+002_L22_00002200.osgb", level, x, y, sub);
    CHECK(ok && level == 22 && x == 5 && y == 2 && sub == "00002200");
}

TEST(cc_parse_root) {
    ContextCaptureHandler h;
    int level, x, y; std::string sub;
    bool ok = h.ParseTilePath("Data/Tile_+000_+000/Tile_+000_+000.osgb", level, x, y, sub);
    CHECK(ok && level == 0 && x == 0 && y == 0);
}

TEST(cc_parse_neg_coords) {
    ContextCaptureHandler h;
    int level, x, y; std::string sub;
    bool ok = h.ParseTilePath("Data/Tile_-003_+005/Tile_-003_+005_L18_0000t1.osgb", level, x, y, sub);
    CHECK(ok && level == 18 && x == -3 && y == 5 && sub == "0000t1");
}

TEST(cc_parse_sub_index_texture_variant) {
    ContextCaptureHandler h;
    int level, x, y; std::string sub;
    bool ok = h.ParseTilePath("Data/Tile_+005_+002/Tile_+005_+002_L20_000030t1.osgb", level, x, y, sub);
    CHECK(ok && level == 20 && sub == "000030t1");
}

// =====================================================================
// Tile path parsing (DJI Terra format)
// =====================================================================

TEST(dji_parse_level) {
    DJITerraHandler h;
    int level, x, y; std::string sub;
    bool ok = h.ParseTilePath("Block_0/level_12/1.osgb", level, x, y, sub);
    CHECK(ok && level == 12 && x == 0 && y == 0);
}

TEST(dji_parse_root) {
    DJITerraHandler h;
    int level, x, y; std::string sub;
    bool ok = h.ParseTilePath("Block_0/Block_0.osgb", level, x, y, sub);
    CHECK(ok && level == 0);
}

TEST(dji_parse_level_sub_index) {
    DJITerraHandler h;
    int level, x, y; std::string sub;
    bool ok = h.ParseTilePath("Block_3/level_14/5.osgb", level, x, y, sub);
    CHECK(ok && level == 14 && sub == "5");
}

// =====================================================================
// Data structure tests
// =====================================================================

TEST(tile_data_bbox) {
    OSGBTileData data;
    data.positions = {0,0,0, 1,0,0, 0,1,0, 0,0,1, -1,-1,-1};
    data.ComputeBBox();
    CHECK_CLOSE(data.bboxMin[0], -1.0, 1e-9);
    CHECK_CLOSE(data.bboxMax[0], 1.0, 1e-9);
    CHECK_CLOSE(data.bboxMin[1], -1.0, 1e-9);
    CHECK_CLOSE(data.bboxMax[1], 1.0, 1e-9);
    CHECK_CLOSE(data.bboxMin[2], -1.0, 1e-9);
    CHECK_CLOSE(data.bboxMax[2], 1.0, 1e-9);
}

TEST(tile_data_empty) {
    OSGBTileData data;
    CHECK(data.IsEmpty());
    data.positions = {0,0,0, 1,0,0, 0,1,0};
    data.indices = {0,1,2};
    CHECK(!data.IsEmpty());
}

TEST(texture_group_bbox) {
    TextureGroup g;
    g.positions = {0,0,0, 10,0,0, 0,10,0};
    g.ComputeBBox();
    CHECK_CLOSE(g.bboxMax[0], 10.0, 1e-9);
    CHECK_CLOSE(g.bboxMax[1], 10.0, 1e-9);
}

TEST(ensure_groups) {
    OSGBTileData data;
    data.positions = {0,0,0, 1,0,0, 0,1,0};
    data.indices = {0,1,2};
    data.texturePath = "test.jpg";
    data.baseColorFactor[0] = 0.5f;
    data.EnsureGroups();
    CHECK(data.groups.size() == 1);
    CHECK(data.groups[0].positions.size() == 9);
    CHECK(data.groups[0].texturePath == "test.jpg");
    CHECK(data.groups[0].baseColorFactor[0] == 0.5f);
}

TEST(tile_data_multi_group) {
    OSGBTileData data;
    TextureGroup g1;
    g1.positions = {0,0,0, 1,0,0, 0,1,0};
    g1.indices = {0,1,2};
    g1.texturePath = "a.jpg";
    TextureGroup g2;
    g2.positions = {0,0,0, 1,0,0, 0,1,0};
    g2.indices = {0,1,2};
    g2.texturePath = "b.jpg";
    data.groups.push_back(std::move(g1));
    data.groups.push_back(std::move(g2));
    CHECK(!data.IsEmpty());
    CHECK(data.groups.size() == 2);
    CHECK(data.groups[0].texturePath == "a.jpg");
    CHECK(data.groups[1].texturePath == "b.jpg");
}

// ---------------------------------------------------------------------------
// DJI hierarchy: cellKey uniqueness across blocks
// ---------------------------------------------------------------------------

static OSGBTileData MakeTile(const std::string& path, int lod, const std::string& sub)
{
    OSGBTileData t;
    t.tilePath = path;
    t.lodLevel = lod;
    t.subTileIndex = sub;
    t.positions = {0,0,0, 1,0,0, 0,1,0};
    t.indices = {0,1,2};
    t.ComputeBBox();
    return t;
}

TEST(cell_builder_enu_to_assimp) {
    // OSGB content is ENU (East, North, Up). BuildGridCellFromTile must convert
    // it to AssimpYUp (East, Up, South) via (x, z, -y) for glTF output.
    OSGBTileData tile;
    tile.tilePath = "Data/Tile_+000_+000/Tile_+000_+000_L1_0.osgb";
    tile.lodLevel = 1;

    TextureGroup g;
    g.positions = {10, 20, 30, 100, 200, 300};  // ENU: (East,North,Up)
    g.indices = {0, 1};
    g.texturePath = "tex.jpg";
    g.ComputeBBox();
    tile.groups.push_back(std::move(g));

    tile.bboxMin[0] = 10; tile.bboxMin[1] = 20; tile.bboxMin[2] = 30;
    tile.bboxMax[0] = 100; tile.bboxMax[1] = 200; tile.bboxMax[2] = 300;

    auto cell = BuildGridCellFromTile(tile, "key");
    CHECK(cell != nullptr);
    CHECK(cell->cellKey == "key");
    CHECK(cell->level == 1);
    CHECK(cell->materialGroups.size() == 1);

    auto& mg = cell->materialGroups[0];
    // (10,20,30) -> (10, 30, -20)
    CHECK_CLOSE(mg.positions[0], 10.0, 1e-6);
    CHECK_CLOSE(mg.positions[1], 30.0, 1e-6);
    CHECK_CLOSE(mg.positions[2], -20.0, 1e-6);
    // (100,200,300) -> (100, 300, -200)
    CHECK_CLOSE(mg.positions[3], 100.0, 1e-6);
    CHECK_CLOSE(mg.positions[4], 300.0, 1e-6);
    CHECK_CLOSE(mg.positions[5], -200.0, 1e-6);

    // Cell bbox converted: ENU (10,20,30)-(100,200,300) -> AssimpYUp
    // X[10,100], Y[30,300], Z[-200,-20]
    CHECK_CLOSE(cell->bboxMin[0], 10.0, 1e-6);
    CHECK_CLOSE(cell->bboxMin[1], 30.0, 1e-6);
    CHECK_CLOSE(cell->bboxMin[2], -200.0, 1e-6);
    CHECK_CLOSE(cell->bboxMax[0], 100.0, 1e-6);
    CHECK_CLOSE(cell->bboxMax[1], 300.0, 1e-6);
    CHECK_CLOSE(cell->bboxMax[2], -20.0, 1e-6);
}

TEST(dji_cellkey_unique_across_blocks) {
    DJITerraHandler h;
    std::vector<OSGBTileData> tiles;
    // Two blocks each have a level_19/1.osgb tile — the bare filename "1" would
    // collide; the fix scopes the cellKey by block/level.
    tiles.push_back(MakeTile("Block_0/Block_0.osgb", 0, ""));
    tiles.push_back(MakeTile("Block_0/level_19/1.osgb", 19, "1"));
    tiles.push_back(MakeTile("Block_1/Block_1.osgb", 0, ""));
    tiles.push_back(MakeTile("Block_1/level_19/1.osgb", 19, "1"));

    OSGBConverterOptions opts;
    auto root = h.BuildHierarchy(tiles, opts);
    CHECK(root != nullptr);

    std::vector<std::string> keys;
    std::function<void(const GridCell*)> collect = [&](const GridCell* c) {
        if (!c) return;
        if (c->hasContent) keys.push_back(c->cellKey);
        for (auto& ch : c->children) if (ch) collect(ch.get());
    };
    for (auto& ch : root->children) if (ch) collect(ch.get());

    CHECK(keys.size() == 4);
    std::set<std::string> unique(keys.begin(), keys.end());
    CHECK(unique.size() == keys.size());
}

TEST(cc_variant_tiles_not_dropped) {
    ContextCaptureHandler h;
    std::vector<OSGBTileData> tiles;
    // ContextCapture splits a sub-tile's texture into sibling files:
    // base + t1/t2 variants, each carrying a disjoint part of the geometry.
    // All of them must become content cells - deduplication by the stripped
    // sub-tile index silently dropped every variant but the last one.
    tiles.push_back(MakeTile("Data/Tile_+005_+002/Tile_+005_+002_L20_000310.osgb",   20, "000310"));
    tiles.push_back(MakeTile("Data/Tile_+005_+002/Tile_+005_+002_L20_000310t1.osgb", 20, "000310t1"));
    tiles.push_back(MakeTile("Data/Tile_+005_+002/Tile_+005_+002_L20_000310t2.osgb", 20, "000310t2"));
    // A child of the base sub-tile at the next LOD - it must attach under the
    // base cell (ancestor lookup strips the variant suffix), not under the root.
    tiles.push_back(MakeTile("Data/Tile_+005_+002/Tile_+005_+002_L21_0003100.osgb",  21, "0003100"));

    OSGBConverterOptions opts;
    auto root = h.BuildHierarchy(tiles, opts);
    CHECK(root != nullptr);
    CHECK(root->children.size() == 1);

    GridCell* cellRoot = root->children[0].get();
    CHECK(cellRoot->cellKey == "Tile_+005_+002_L20_000310");

    // base's own 2 variant siblings + the L21 child, all under the base cell.
    CHECK(cellRoot->children.size() == 3);

    std::set<std::string> childKeys;
    for (auto& ch : cellRoot->children) childKeys.insert(ch->cellKey);
    CHECK(childKeys.count("Tile_+005_+002_L20_000310t1") == 1);
    CHECK(childKeys.count("Tile_+005_+002_L20_000310t2") == 1);
    CHECK(childKeys.count("Tile_+005_+002_L21_0003100") == 1);
}

// =====================================================================
// Platform compatibility tests
// =====================================================================

TEST(platform_dir_exists) {
    std::string srcDir = __FILE__;
    size_t slash = srcDir.find_last_of("/\\");
    std::string repoRoot = srcDir.substr(0, slash);  // .../OSGBConverter/test
    slash = repoRoot.find_last_of("/\\");
    repoRoot = repoRoot.substr(0, slash);            // .../OSGBConverter
    slash = repoRoot.find_last_of("/\\");
    repoRoot = repoRoot.substr(0, slash);            // repo root
    CHECK(DirectoryLister::IsDirectory(repoRoot));
    CHECK(!DirectoryLister::IsDirectory(repoRoot + "/nonexistent_path_12345"));
}

TEST(platform_dir_list) {
    std::string f = __FILE__;
    size_t slash = f.find_last_of("/\\");
    std::string osgbDir = f.substr(0, slash) + "/..";  // OSGBConverter dir
    auto entries = DirectoryLister::List(osgbDir);
    bool found_h = false, found_cpp = false;
    for (auto& e : entries) {
        if (e.name == "OSGBConverter.h") found_h = true;
        if (e.name == "OSGBConverter.cpp") found_cpp = true;
    }
    CHECK(found_h && found_cpp);
}

// =====================================================================
// Vendor detection tests
// =====================================================================

TEST(vendor_cc) {
    auto handler = VendorHandlerFactory::Create(DataVendor::ContextCapture);
    CHECK(handler != nullptr);
    CHECK(handler->GetVendor() == DataVendor::ContextCapture);
}

TEST(vendor_dji) {
    auto handler = VendorHandlerFactory::Create(DataVendor::DJITerra);
    CHECK(handler != nullptr);
    CHECK(handler->GetVendor() == DataVendor::DJITerra);
}

TEST(vendor_unknown_null) {
    auto handler = VendorHandlerFactory::Create(DataVendor::Unknown);
    CHECK(handler == nullptr);
}

// =====================================================================
// Main
// =====================================================================

int main() {
    std::cout << "=== MetadataParser ===" << std::endl;
    RUN(parse_srs_epsg);
    RUN(parse_srs_epsg_lower);
    RUN(parse_srs_epsg_bad);
    RUN(parse_srs_epsg_empty);
    RUN(parse_enu_origin);
    RUN(parse_enu_origin_lower);
    RUN(parse_enu_not_enu);
    RUN(parse_origin_3d);
    RUN(parse_origin_2d);
    RUN(detect_projection_root_only);
    RUN(detect_projection_per_tile);
    RUN(detect_projection_unknown);

    std::cout << "=== CC Tile Path Parsing ===" << std::endl;
    RUN(cc_parse_basic);
    RUN(cc_parse_root);
    RUN(cc_parse_neg_coords);
    RUN(cc_parse_sub_index_texture_variant);

    std::cout << "=== DJI Tile Path Parsing ===" << std::endl;
    RUN(dji_parse_level);
    RUN(dji_parse_root);
    RUN(dji_parse_level_sub_index);

    std::cout << "=== Data Structures ===" << std::endl;
    RUN(tile_data_bbox);
    RUN(tile_data_empty);
    RUN(texture_group_bbox);
    RUN(ensure_groups);
    RUN(tile_data_multi_group);
    RUN(cell_builder_enu_to_assimp);
    RUN(dji_cellkey_unique_across_blocks);
    RUN(cc_variant_tiles_not_dropped);

    std::cout << "=== PlatformCompat ===" << std::endl;
    RUN(platform_dir_exists);
    RUN(platform_dir_list);

    std::cout << "=== Vendor Detection ===" << std::endl;
    RUN(vendor_cc);
    RUN(vendor_dji);
    RUN(vendor_unknown_null);

    std::cout << "\n========================================" << std::endl;
    std::cout << "PASS=" << g_pass << " FAIL=" << g_fail << std::endl;
    return g_fail > 0 ? 1 : 0;
}