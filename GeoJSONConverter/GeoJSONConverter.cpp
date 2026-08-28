// Copyright Johnlyon
//
// GeoJSONConverter implementation
//

#include "GeoJSONConverter.h"
#include "Json.h"
#include "../MeshProjectionErrorCorrector/CRSTransformer.h"

#include <fstream>
#include <iostream>
#include <memory>
#include <vector>

namespace {

using mgo::json::Value;

// Legacy "crs" member name -> "EPSG:<code>" (empty when unsupported).
std::string crsNameToSpec(const Value* crsMember)
{
    if (!crsMember || !crsMember->isObject()) return "";
    const Value* props = crsMember->find("properties");
    if (!props || !props->isObject()) return "";
    const Value* name = props->find("name");
    if (!name || !name->isString()) return "";

    double lat, lon, h;
    if (CRSTransformer::ParseENUSpec(name->str, lat, lon, h))
        return name->str;                       // "ENU:..." name
    const std::string& s = name->str;
    std::string lower;
    for (char c : s) lower += static_cast<char>(::tolower(c));
    size_t pos = lower.find("epsg");
    if (pos == std::string::npos) return "";
    size_t i = pos + 4;
    while (i < lower.size() && !::isdigit(lower[i])) ++i;
    size_t start = i;
    while (i < lower.size() && ::isdigit(lower[i])) ++i;
    if (start == i) return "";
    return "EPSG:" + s.substr(start, i - start);
}

// Transform one position array [x, y(, z)]. Returns false on failure.
bool transformPosition(Value& pos, CRSTransformer& tr, std::string& err)
{
    if (!pos.isArray() || pos.array.size() < 2)
    {
        err = "Invalid position (expected [x, y(, z)])";
        return false;
    }
    double x = pos.array[0].asDouble();
    double y = pos.array[1].asDouble();
    double z = 0.0;
    bool hasZ = pos.array.size() >= 3 && pos.array[2].isNumber();
    if (hasZ) z = pos.array[2].asDouble();

    if (!tr.Transform(x, y, z))
    {
        err = tr.GetLastError();
        return false;
    }
    pos.array[0] = Value::makeNumber(x);
    pos.array[1] = Value::makeNumber(y);
    if (hasZ) pos.array[2] = Value::makeNumber(z);
    return true;
}

struct Stats
{
    size_t positions = 0;
    size_t features = 0;
};

bool transformMultiPointLike(Value& coords, CRSTransformer& tr,
                             std::string& err, Stats& st)
{
    // coords = [ [x,y], ... ]
    if (!coords.isArray()) { err = "Invalid coordinates"; return false; }
    for (auto& p : coords.array)
    {
        if (!transformPosition(p, tr, err)) return false;
        ++st.positions;
    }
    return true;
}

bool transformLineStringLike(Value& coords, CRSTransformer& tr,
                             std::string& err, Stats& st)
{
    return transformMultiPointLike(coords, tr, err, st);
}

bool transformPolygonRings(Value& coords, CRSTransformer& tr,
                           std::string& err, Stats& st)
{
    // coords = [ ring, ring, ... ], ring = [ [x,y], ... ]
    if (!coords.isArray()) { err = "Invalid coordinates"; return false; }
    for (auto& ring : coords.array)
        if (!transformMultiPointLike(ring, tr, err, st)) return false;
    return true;
}

bool transformMultiPolygon(Value& coords, CRSTransformer& tr,
                           std::string& err, Stats& st)
{
    // coords = [ polygon, ... ], polygon = [ ring, ... ]
    if (!coords.isArray()) { err = "Invalid coordinates"; return false; }
    for (auto& poly : coords.array)
        if (!transformPolygonRings(poly, tr, err, st)) return false;
    return true;
}

bool transformGeometry(Value& geom, CRSTransformer& tr,
                       std::string& err, Stats& st);

bool transformGeometryCollection(Value& geoms, CRSTransformer& tr,
                                 std::string& err, Stats& st)
{
    if (!geoms.isArray()) { err = "Invalid GeometryCollection"; return false; }
    for (auto& g : geoms.array)
        if (!transformGeometry(g, tr, err, st)) return false;
    return true;
}

bool transformGeometry(Value& geom, CRSTransformer& tr,
                       std::string& err, Stats& st)
{
    if (!geom.isObject()) { err = "Geometry is not an object"; return false; }
    const Value* type = geom.find("type");
    if (!type || !type->isString()) { err = "Geometry missing \"type\""; return false; }
    const std::string& t = type->str;

    if (t == "GeometryCollection")
    {
        Value* geoms = geom.find("geometries");
        if (!geoms) { err = "GeometryCollection missing \"geometries\""; return false; }
        return transformGeometryCollection(*geoms, tr, err, st);
    }

    Value* coords = geom.find("coordinates");
    if (!coords) { err = "Geometry missing \"coordinates\""; return false; }

    if (t == "Point")         { if (!transformPosition(*coords, tr, err)) return false; ++st.positions; return true; }
    if (t == "MultiPoint")    return transformMultiPointLike(*coords, tr, err, st);
    if (t == "LineString")    return transformLineStringLike(*coords, tr, err, st);
    if (t == "MultiLineString") return transformPolygonRings(*coords, tr, err, st);
    if (t == "Polygon")       return transformPolygonRings(*coords, tr, err, st);
    if (t == "MultiPolygon")  return transformMultiPolygon(*coords, tr, err, st);

    err = "Unknown geometry type: " + t;
    return false;
}

bool transformFeature(Value& feature, CRSTransformer& tr,
                      std::string& err, Stats& st)
{
    Value* geom = feature.find("geometry");
    if (!geom || geom->isNull()) return true;   // null geometry is legal
    if (!transformGeometry(*geom, tr, err, st)) return false;
    ++st.features;
    return true;
}

bool transformNode(Value& node, CRSTransformer& tr, std::string& err, Stats& st)
{
    if (!node.isObject()) { err = "Top-level GeoJSON must be an object"; return false; }
    const Value* type = node.find("type");
    if (!type || !type->isString()) { err = "Missing \"type\""; return false; }
    const std::string& t = type->str;

    if (t == "FeatureCollection")
    {
        Value* features = node.find("features");
        if (!features || !features->isArray())
        {
            err = "FeatureCollection missing \"features\" array";
            return false;
        }
        for (auto& f : features->array)
        {
            if (!f.isObject()) { err = "Feature is not an object"; return false; }
            if (!transformFeature(f, tr, err, st)) return false;
        }
        return true;
    }
    if (t == "Feature") return transformFeature(node, tr, err, st);
    return transformGeometry(node, tr, err, st);
}

} // anonymous namespace

// ---------------------------------------------------------------------------

GeoJSONConverter::GeoJSONConverter() = default;
GeoJSONConverter::~GeoJSONConverter() = default;

bool GeoJSONConverter::Convert(const GeoJSONConverterOptions& opts)
{
    m_lastError.clear();

    if (opts.inputFile.empty() || opts.outputFile.empty())
    {
        m_lastError = "inputFile and outputFile are required";
        return false;
    }

    // --- Read ---
    std::ifstream in(opts.inputFile, std::ios::binary);
    if (!in)
    {
        m_lastError = "Cannot open input file: " + opts.inputFile;
        return false;
    }
    std::string content((std::istreambuf_iterator<char>(in)),
                        std::istreambuf_iterator<char>());

    Value root;
    std::string jsonErr;
    if (!mgo::json::parse(content, root, jsonErr))
    {
        m_lastError = "JSON parse error: " + jsonErr;
        return false;
    }

    // --- Resolve CRS ---
    std::string sourceSpec = opts.sourceCRS;
    if (sourceSpec.empty())
    {
        sourceSpec = crsNameToSpec(root.find("crs"));
        if (sourceSpec.empty())
            sourceSpec = "EPSG:4326";   // RFC 7946 default
        else if (opts.verbose)
            std::cout << "[GeoJSONConverter] Source CRS from crs member: "
                      << sourceSpec << std::endl;
    }
    std::string targetSpec = opts.targetCRS.empty()
        ? std::string("EPSG:4326") : opts.targetCRS;

    auto transformer = std::make_unique<CRSTransformer>();
    if (!transformer->SetSourceCRS(sourceSpec))
    {
        m_lastError = "Invalid source CRS \"" + sourceSpec + "\": "
                      + transformer->GetLastError();
        return false;
    }
    if (!transformer->SetTargetCRS(targetSpec))
    {
        m_lastError = "Invalid target CRS \"" + targetSpec + "\": "
                      + transformer->GetLastError();
        return false;
    }

    // --- Transform ---
    Stats stats;
    if (!transformer->IsIdentity())
    {
        std::string err;
        if (!transformNode(root, *transformer, err, stats))
        {
            m_lastError = err;
            return false;
        }
    }
    else if (opts.verbose)
    {
        std::cout << "[GeoJSONConverter] Source == target CRS, passing through"
                  << std::endl;
    }

    // --- Write ---
    std::string out = mgo::json::serialize(root, opts.pretty);
    std::ofstream of(opts.outputFile, std::ios::binary);
    if (!of)
    {
        m_lastError = "Cannot open output file: " + opts.outputFile;
        return false;
    }
    of.write(out.data(), static_cast<std::streamsize>(out.size()));
    of << "\n";
    if (!of.good())
    {
        m_lastError = "Failed writing output file: " + opts.outputFile;
        return false;
    }

    if (opts.verbose)
    {
        std::cout << "[GeoJSONConverter] " << sourceSpec << " -> " << targetSpec
                  << ": " << stats.features << " feature(s), "
                  << stats.positions << " position(s) transformed" << std::endl;
    }
    return true;
}
