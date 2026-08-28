// Copyright Johnlyon
//
// GeoJSONConverter unit tests - JSON parser/serializer, ENU support,
// CRS transformation, end-to-end GeoJSON conversion
//

#include "../Json.h"
#include "../GeoJSONConverter.h"
#include "../../MeshProjectionErrorCorrector/CRSTransformer.h"

#include <iostream>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>

static int g_pass = 0, g_fail = 0;
#define TEST(name) void test_##name()
#define RUN(name) do { std::cout << "  " #name "... "; test_##name(); } while(0)
#define CHECK(cond) do { if (cond) { std::cout << "PASS" << std::endl; g_pass++; } \
    else { std::cout << "FAIL (" << __LINE__ << ")" << std::endl; g_fail++; } } while(0)
#define CHECK_EQ(a, b) CHECK((a) == (b))
#define CHECK_CLOSE(a, b, eps) CHECK(std::fabs((a)-(b)) < (eps))

using mgo::json::Value;

// =====================================================================
// JSON parser / serializer
// =====================================================================

TEST(json_scalars)
{
    Value v; std::string err;
    CHECK(mgo::json::parse("null", v, err) && v.isNull());
    CHECK(mgo::json::parse("true", v, err) && v.isBool() && v.boolean);
    CHECK(mgo::json::parse("false", v, err) && v.isBool() && !v.boolean);
    CHECK(mgo::json::parse("\"hi\"", v, err) && v.isString() && v.str == "hi");
    CHECK(mgo::json::parse("-12.5e2", v, err) && v.isNumber());
    CHECK_EQ(v.number, std::string("-12.5e2"));
}

TEST(json_escapes)
{
    Value v; std::string err;
    CHECK(mgo::json::parse("\"a\\nb\\t\\\"\\\\\\/c\"", v, err) && v.str == "a\nb\t\"\\/c");
    CHECK(mgo::json::parse("\"\\u00e9\"", v, err) && v.str == "\xc3\xa9");
    // Surrogate pair U+1F600
    CHECK(mgo::json::parse("\"\\ud83d\\ude00\"", v, err) && v.str == "\xf0\x9f\x98\x80");
    // Lone surrogate must be rejected
    CHECK(!mgo::json::parse("\"\\ud800\"", v, err));
}

TEST(json_structure)
{
    Value v; std::string err;
    CHECK(mgo::json::parse("{\"a\": [1, 2, {\"b\": null}], \"c\": {}}", v, err));
    CHECK(v.isObject() && v.object.size() == 2);
    const Value* a = v.find("a");
    CHECK(a && a->isArray() && a->array.size() == 3);
    CHECK(a->array[0].asDouble() == 1.0 && a->array[1].asDouble() == 2.0);
    CHECK(a->array[2].isObject() && a->array[2].find("b")->isNull());
    CHECK(v.find("c") && v.find("c")->object.empty());
    CHECK(v.find("missing") == nullptr);
    // member order preserved
    CHECK(v.object[0].first == "a" && v.object[1].first == "c");
}

TEST(json_errors)
{
    Value v; std::string err;
    CHECK(!mgo::json::parse("", v, err));
    CHECK(!mgo::json::parse("{", v, err));
    CHECK(!mgo::json::parse("[1,]", v, err));
    CHECK(!mgo::json::parse("\"unterminated", v, err));
    CHECK(!mgo::json::parse("01", v, err));          // leading zero
    CHECK(!mgo::json::parse("1 2", v, err));         // trailing content
    // BOM tolerated
    CHECK(mgo::json::parse("\xEF\xBB\xBF{\"k\":1}", v, err));
}

TEST(json_roundtrip)
{
    Value v; std::string err;
    const char* doc = "{\"n\":1.250,\"s\":\"caf\\u00e9\",\"arr\":[true,null,-0.5]}";
    CHECK(mgo::json::parse(doc, v, err));
    // Untouched numbers keep their raw token (trailing zero preserved).
    CHECK(mgo::json::serialize(v).find("\"n\":1.250") != std::string::npos);
    CHECK(mgo::json::serialize(v).find("-0.5") != std::string::npos);
    // Value re-parses to the same structure.
    Value w; CHECK(mgo::json::parse(mgo::json::serialize(v), w, err));
    CHECK(w.find("n")->number == "1.250");
}

TEST(json_make_number)
{
    CHECK_EQ(Value::makeNumber(113.06277).number, std::string("113.06277"));
    CHECK_EQ(Value::makeNumber(100.0).number, std::string("100"));
    CHECK_EQ(Value::makeNumber(-0.5).number, std::string("-0.5"));
}

// =====================================================================
// ENU support (CRSTransformer)
// =====================================================================

TEST(enu_spec_parsing)
{
    double lat, lon, h;
    CHECK(CRSTransformer::ParseENUSpec("ENU:22.64785,113.06277", lat, lon, h));
    CHECK_CLOSE(lat, 22.64785, 1e-12);
    CHECK_CLOSE(lon, 113.06277, 1e-12);
    CHECK_CLOSE(h, 0.0, 1e-12);
    CHECK(CRSTransformer::ParseENUSpec("enu:1.5,2.5,3.5", lat, lon, h));
    CHECK_CLOSE(h, 3.5, 1e-12);
    CHECK(!CRSTransformer::ParseENUSpec("EPSG:4326", lat, lon, h));
    CHECK(!CRSTransformer::ParseENUSpec("ENU:22.6", lat, lon, h));
    CHECK(!CRSTransformer::ParseENUSpec("ENU:a,b", lat, lon, h));
}

TEST(enu_origin_identity)
{
    CRSTransformer tr;
    CHECK(tr.SetSourceCRS("EPSG:4326"));
    CHECK(tr.SetTargetCRS("ENU:22.64785,113.06277"));
    double x = 113.06277, y = 22.64785, z = 10.0;
    CHECK(tr.Transform(x, y, z));
    CHECK_CLOSE(x, 0.0, 1e-6);
    CHECK_CLOSE(y, 0.0, 1e-6);
    CHECK_CLOSE(z, 10.0, 1e-6);
}

TEST(enu_east_offset)
{
    CRSTransformer tr;
    CHECK(tr.SetSourceCRS("ENU:22.64785,113.06277"));
    CHECK(tr.SetTargetCRS("EPSG:4326"));
    // 100 m due east -> small positive longitude delta, latitude unchanged.
    double x = 100.0, y = 0.0, z = 0.0;
    CHECK(tr.Transform(x, y, z));
    CHECK_CLOSE(x, 113.06277 + 100.0 / (111319.9 * std::cos(22.64785 * M_PI / 180.0)), 1e-5);
    CHECK_CLOSE(y, 22.64785, 1e-7);  // 2nd-order tangent-plane effect
    // IsIdentity must be false for 4326 -> ENU
    CHECK(!tr.IsIdentity());
}

TEST(enu_roundtrip)
{
    CRSTransformer tr;
    CHECK(tr.SetSourceCRS("EPSG:4326"));
    CHECK(tr.SetTargetCRS("EPSG:4326"));
    CHECK(tr.IsIdentity());
    CRSTransformer tr2;
    CHECK(tr2.SetSourceCRS("ENU:22.64785,113.06277"));
    CHECK(tr2.SetTargetCRS("ENU:22.64785,113.06277"));
    CHECK(tr2.IsIdentity());
}

TEST(epsg_roundtrip)
{
    CRSTransformer fwd, inv;
    CHECK(fwd.SetSourceCRS("EPSG:4547"));
    CHECK(fwd.SetTargetCRS("EPSG:4326"));
    CHECK(inv.SetSourceCRS("EPSG:4326"));
    CHECK(inv.SetTargetCRS("EPSG:4547"));
    double x = 510000.0, y = 2510000.0, z = 50.0;
    CHECK(fwd.Transform(x, y, z));
    CHECK(inv.Transform(x, y, z));
    CHECK_CLOSE(x, 510000.0, 1e-4);
    CHECK_CLOSE(y, 2510000.0, 1e-4);
    CHECK_CLOSE(z, 50.0, 1e-6);
}

TEST(invalid_crs_rejected)
{
    CRSTransformer tr;
    CHECK(!tr.SetSourceCRS("EPSG:999999"));
    CHECK(!tr.GetLastError().empty());
}

// =====================================================================
// GeoJSONConverter end-to-end (temp files)
// =====================================================================

static std::string writeTemp(const char* name, const std::string& content)
{
    std::string path = (std::string(name));
    std::ofstream f(path, std::ios::binary);
    f << content;
    return path;
}

TEST(geojson_end_to_end)
{
    std::string in = writeTemp("mgo_test_in.geojson",
        "{\"type\":\"FeatureCollection\",\"features\":[{"
        "\"type\":\"Feature\",\"properties\":{\"name\":\"p\",\"v\":1},"
        "\"geometry\":{\"type\":\"Point\",\"coordinates\":[113.06277,22.64785,10]}},"
        "{\"type\":\"Feature\",\"properties\":{},"
        "\"geometry\":{\"type\":\"LineString\",\"coordinates\":[[113.06277,22.64785],[113.07277,22.64785]]}}]}");

    GeoJSONConverterOptions opts;
    opts.inputFile = in;
    opts.outputFile = "mgo_test_out.geojson";
    opts.sourceCRS = "EPSG:4326";
    opts.targetCRS = "ENU:22.64785,113.06277";

    GeoJSONConverter conv;
    if (!conv.Convert(opts))
    {
        std::cout << "ERROR: " << conv.GetLastError() << std::endl;
        std::ifstream tf(opts.inputFile, std::ios::binary);
        std::string tc((std::istreambuf_iterator<char>(tf)),
                       std::istreambuf_iterator<char>());
        std::cout << "INPUT(" << tc.size() << "): " << tc << std::endl;
        std::cout << "around 163: " << tc.substr(150, 40) << std::endl;
    }
    CHECK(conv.Convert(opts));

    Value out; std::string err;
    std::ifstream f(opts.outputFile, std::ios::binary);
    std::string content((std::istreambuf_iterator<char>(f)),
                        std::istreambuf_iterator<char>());
    CHECK(mgo::json::parse(content, out, err));
    const Value* feats = out.find("features");
    CHECK(feats && feats->array.size() == 2);
    CHECK(feats != nullptr);
    const Value* c0 = feats->array[0].find("geometry")->find("coordinates");
    CHECK(c0 != nullptr);
    CHECK_CLOSE(c0->array[0].asDouble(), 0.0, 1e-6);
    CHECK_CLOSE(c0->array[1].asDouble(), 0.0, 1e-6);
    CHECK_CLOSE(c0->array[2].asDouble(), 10.0, 1e-6);
    // properties untouched
    const Value* props = feats->array[0].find("properties");
    CHECK(props && props->find("name")->str == "p" &&
          props->find("v")->number == "1");
    // line second point ~1027.9 m east
    const Value* c1 = feats->array[1].find("geometry")->find("coordinates");
    CHECK(c1 != nullptr);
    CHECK_CLOSE(c1->array[1].array[0].asDouble(), 1027.9, 1.0);
    CHECK_CLOSE(c1->array[1].array[1].asDouble(), 0.0, 1.0);

    std::remove("mgo_test_in.geojson");
    std::remove("mgo_test_out.geojson");
}

TEST(geojson_identity_passthrough)
{
    std::string in = writeTemp("mgo_test_id.geojson",
        "{\"type\":\"Point\",\"coordinates\":[113.06277,22.64785]}");
    GeoJSONConverterOptions opts;
    opts.inputFile = in;
    opts.outputFile = "mgo_test_id_out.geojson";
    opts.sourceCRS = "EPSG:4326";
    opts.targetCRS = "EPSG:4326";
    GeoJSONConverter conv;
    if (!conv.Convert(opts))
    {
        std::cout << "ERROR: " << conv.GetLastError() << std::endl;
        std::ifstream tf(opts.inputFile, std::ios::binary);
        std::string tc((std::istreambuf_iterator<char>(tf)),
                       std::istreambuf_iterator<char>());
        std::cout << "INPUT(" << tc.size() << "): " << tc << std::endl;
        std::cout << "around 163: " << tc.substr(150, 40) << std::endl;
    }
    CHECK(conv.Convert(opts));
    std::ifstream f(opts.outputFile);
    std::string content((std::istreambuf_iterator<char>(f)),
                        std::istreambuf_iterator<char>());
    CHECK(content.find("113.06277") != std::string::npos);
    std::remove("mgo_test_id.geojson");
    std::remove("mgo_test_id_out.geojson");
}

TEST(geojson_invalid_input)
{
    GeoJSONConverter conv;
    GeoJSONConverterOptions opts;
    opts.inputFile = "no_such_file.geojson";
    opts.outputFile = "no_out.geojson";
    CHECK(!conv.Convert(opts));
    CHECK(!conv.GetLastError().empty());
}

// =====================================================================

int main()
{
    std::cout << "=== JSON parser ===" << std::endl;
    RUN(json_scalars);
    RUN(json_escapes);
    RUN(json_structure);
    RUN(json_errors);
    RUN(json_roundtrip);
    RUN(json_make_number);
    std::cout << "=== ENU / CRS transformer ===" << std::endl;
    RUN(enu_spec_parsing);
    RUN(enu_origin_identity);
    RUN(enu_east_offset);
    RUN(enu_roundtrip);
    RUN(epsg_roundtrip);
    RUN(invalid_crs_rejected);
    std::cout << "=== GeoJSONConverter ===" << std::endl;
    RUN(geojson_end_to_end);
    RUN(geojson_identity_passthrough);
    RUN(geojson_invalid_input);

    std::cout << "\n========================================\n"
              << "PASS=" << g_pass << " FAIL=" << g_fail << std::endl;
    return g_fail == 0 ? 0 : 1;
}
