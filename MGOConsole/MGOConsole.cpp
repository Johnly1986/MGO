// MGOConsole.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <fstream>
#include <sstream>
#include <cstring>
#include <algorithm>
#ifdef _WIN32
#include <Windows.h>
#elif defined(__APPLE__)
#include <mach-o/dyld.h>
#include <unistd.h>
#elif defined(__linux__)
#include <unistd.h>
#endif

#include <boost/regex.hpp>
#include <filesystem>
#include <boost/algorithm/string.hpp>
namespace fs = std::filesystem;

#include "CSVReader.h"
#include "MeshProjectionErrorCorrector.h"
#include "MeshGroupOptimizer.h"

#include "GeoreferencingWithMultiPosition.h"
#include "GeoreferencingWith7Parameters.h"
#include "GeoreferencingWithAnchor.h"
#include <proj/proj.h>

#include "MGOVersion.h"

// Subcommand modules
#include "TilesConverter.h"
#include "TerrainConverter.h"
#include "ImageTiler.h"
#include "GeoJSONConverter.h"
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#ifdef HAS_OSGB_CONVERTER
#include "OSGBConverter.h"
#endif

// Simple argument parser to replace boost::program_options dependency
struct OptionSpec {
    std::string longName;
    std::string shortName;
    bool takesValue;
    std::string defaultValue;
    std::string description;
};

class SimpleArgParser {
    std::vector<OptionSpec> specs;
    std::map<std::string, std::string> values;
    std::map<std::string, bool> flags;
    std::string helpText;
public:
    void AddOption(const std::string& longName, const std::string& shortName,
        bool takesValue, const std::string& defaultValue, const std::string& description)
    {
        specs.push_back({ longName, shortName, takesValue, defaultValue, description });
    }

    bool Parse(int argc, char** argv)
    {
        // Set defaults
        for (auto& spec : specs) {
            if (spec.takesValue)
                values[spec.longName] = spec.defaultValue;
            else
                flags[spec.longName] = (spec.defaultValue == "true");
        }

        // Build help text
        helpText = u8"数字孪生与实景构建\n\n";
        for (auto& spec : specs) {
            helpText += "  --" + spec.longName;
            if (!spec.shortName.empty())
                helpText += ", -" + spec.shortName;
            if (spec.takesValue)
                helpText += " <value>";
            helpText += "\t" + spec.description + "\n";
        }

        for (int i = 1; i < argc; i++) {
            std::string arg = argv[i];
            if (arg == "--help" || arg == "-h") {
                std::cout << helpText << "\n";
                return false;
            }

            bool matched = false;
            for (auto& spec : specs) {
                std::string prefix = "--" + spec.longName;
                std::string shortPrefix = "-" + spec.shortName;
                if (arg == prefix || arg == shortPrefix) {
                    if (spec.takesValue) {
                        if (i + 1 < argc) {
                            values[spec.longName] = argv[++i];
                        }
                        else {
                            std::cerr << u8"选项 " << arg << u8" 需要参数值\n";
                            return false;
                        }
                    }
                    else {
                        flags[spec.longName] = true;
                    }
                    matched = true;
                    break;
                }
            }
            if (!matched) {
                // Treat as positional if it looks like a standalone value
                // but warn about unknown options
                if (arg.substr(0, 1) == "-") {
                    std::cerr << u8"未知选项: " << arg << "\n";
                    return false;
                }
            }
        }
        return true;
    }

    std::string GetString(const std::string& name) const {
        auto it = values.find(name);
        return it != values.end() ? it->second : "";
    }

    double GetDouble(const std::string& name) const {
        auto it = values.find(name);
        if (it == values.end()) return 0.0;
        try { return std::stod(it->second); }
        catch (...) {
            std::cerr << u8"非法数值 --" << name << ": "
                      << it->second << "\n";
            return 0.0;
        }
    }

    float GetFloat(const std::string& name) const {
        return static_cast<float>(GetDouble(name));
    }

    bool GetBool(const std::string& name) const {
        auto it = flags.find(name);
        if (it != flags.end()) return it->second;
        auto vit = values.find(name);
        if (vit != values.end()) return vit->second == "true" || vit->second == "1";
        return false;
    }

    bool Has(const std::string& name) const {
        auto vit = values.find(name);
        if (vit != values.end() && !vit->second.empty()) return true;
        auto fit = flags.find(name);
        if (fit != flags.end() && fit->second) return true;
        // Also check if it was explicitly set to non-default
        return false;
    }

    bool Empty(const std::string& name) const {
        auto it = values.find(name);
        return it == values.end() || it->second.empty();
    }
};

// GBK to UTF-8 conversion using Windows API (replacement for boost::locale::conv::to_utf)
std::string gbk_to_utf8(const std::string& gbk_str)
{
    if (gbk_str.empty()) return {};
#ifdef _WIN32
    int wideLen = MultiByteToWideChar(CP_ACP, 0, gbk_str.c_str(), (int)gbk_str.size(), nullptr, 0);
    if (wideLen <= 0) return gbk_str;
    std::wstring wide(wideLen, L'\0');
    MultiByteToWideChar(CP_ACP, 0, gbk_str.c_str(), (int)gbk_str.size(), &wide[0], wideLen);
    int utf8Len = WideCharToMultiByte(CP_UTF8, 0, wide.c_str(), wideLen, nullptr, 0, nullptr, nullptr);
    if (utf8Len <= 0) return gbk_str;
    std::string utf8(utf8Len, '\0');
    WideCharToMultiByte(CP_UTF8, 0, wide.c_str(), wideLen, &utf8[0], utf8Len, nullptr, nullptr);
    return utf8;
#else
    // Linux: paths from CLI args are already UTF-8; no conversion needed.
    return gbk_str;
#endif
}

struct OptimizerItemLoader : public OptimizerItem
{
    void fromkv(const std::map<std::string, std::string>& values)
    {
        if (values.count("name")) name = values.at("name");  // utf_to_utf<char> is a no-op for char->char
        if (values.count("error")) error = std::stof(values.at("error"));
        if (values.count("nweight")) nweight = std::stof(values.at("nweight"));
        if (values.count("threshold")) threshold = std::stof(values.at("threshold"));
        if (values.count("lockborder")) lockBorder = values.at("lockborder") == "true";
        if (values.count("localerror")) localError = values.at("localerror") == "true";
    }
};

struct ControlPointLoader : public ControlPoint
{
    void fromkv(const std::map<std::string, std::string>& values)
    {
        if (values.count("sx")) orign_source_position(0) = std::stof(values.at("sx"));
        if (values.count("sy")) orign_source_position(1) = std::stof(values.at("sy"));
        if (values.count("sz")) orign_source_position(2) = std::stof(values.at("sz"));
        if (values.count("tx")) orign_target_position(0) = std::stof(values.at("tx"));
        if (values.count("ty")) orign_target_position(1) = std::stof(values.at("ty"));
        if (values.count("tz")) orign_target_position(2) = std::stof(values.at("tz"));
    }
};

// WKT to PROJ string converter (bypasses PROJ's broken WKT parser for custom datums)
std::string wkt_to_proj_string(const std::string& wkt)
{
    // Handle empty/fallback case
    if (wkt.empty() || wkt[0] != 'P') {
        return wkt;
    }

    // Extract SPHEROID parameters
    boost::regex spheroid_regex(R"(SPHEROID\["[^"]*",(\d+\.?\d*),(\d+\.?\d*)\])");
    boost::smatch spheroid_match;
    std::string a_val = "6378137";
    std::string rf_val = "298.257223563";
    if (boost::regex_search(wkt, spheroid_match, spheroid_regex)) {
        a_val = spheroid_match[1];
        rf_val = spheroid_match[2];
    }

    // Extract projection type
    std::string proj = "tmerc";
    if (wkt.find("Gauss_Kruger") != std::string::npos || wkt.find("Transverse_Mercator") != std::string::npos) {
        proj = "tmerc";
    } else if (wkt.find("Lambert_Conformal_Conic") != std::string::npos) {
        proj = "lcc";
    } else if (wkt.find("Albers") != std::string::npos) {
        proj = "aea";
    } else if (wkt.find("Mercator") != std::string::npos) {
        proj = "merc";
    } else if (wkt.find("Polyconic") != std::string::npos) {
        proj = "poly";
    }

    // Extract PARAMETER values using case-insensitive regex
    // (WKT can use mixed case like "central_meridian" or "Central_Meridian")
    auto extract_param = [&](const std::string& name) -> std::string {
        std::string re_str = "PARAMETER\\[\\\"" + name + "\\\",([^]]+)\\]";
        boost::regex re(re_str, boost::regex::icase);
        boost::smatch m;
        if (boost::regex_search(wkt, m, re)) {
            std::string val = m[1];
            boost::trim(val);
            return val;
        }
        return "";
    };

    // Extract UNIT at top level (PROJCS level, not GEOGCS level)
    std::string units = "m";
    // Find the last UNIT[...] (the projected CRS unit)
    std::string::size_type last_unit = wkt.rfind("UNIT[");
    if (last_unit != std::string::npos) {
        std::string after = wkt.substr(last_unit);
        boost::regex unit_re(R"(UNIT\["[^"]*",(\d+\.?\d*)\])", boost::regex::icase);
        boost::smatch um;
        if (boost::regex_search(after, um, unit_re)) {
            double unit_val = std::stod(um[1]);
            if (std::abs(unit_val - 0.0174532925199433) < 1e-10) {
                units = "degree";
            }
        }
    }

    std::ostringstream proj_str;
    proj_str << "+proj=" << proj;

    std::string lon_0 = extract_param("central_meridian");
    if (!lon_0.empty()) proj_str << " +lon_0=" << lon_0;

    std::string lat_0 = extract_param("latitude_of_origin");
    if (!lat_0.empty()) proj_str << " +lat_0=" << lat_0;

    std::string k = extract_param("scale_factor");
    if (!k.empty()) proj_str << " +k=" << k;

    std::string x_0 = extract_param("false_easting");
    if (!x_0.empty()) proj_str << " +x_0=" << x_0;

    std::string y_0 = extract_param("false_northing");
    if (!y_0.empty()) proj_str << " +y_0=" << y_0;

    // Standard parallels for LCC/Albers
    std::string lat_1 = extract_param("standard_parallel_1");
    if (!lat_1.empty()) proj_str << " +lat_1=" << lat_1;
    std::string lat_2 = extract_param("standard_parallel_2");
    if (!lat_2.empty()) proj_str << " +lat_2=" << lat_2;

    proj_str << " +a=" << a_val << " +rf=" << rf_val;
    proj_str << " +units=" << units << " +no_defs +type=crs";

    return proj_str.str();
}

Vector3D parseVector3D(const std::string& input)
{
    Vector3D result;
    boost::smatch match;
    boost::regex paren_regex(R"(\(([^)]+)\))");

    if (boost::regex_search(input, match, paren_regex)) {
        std::string inner = match[1];

        // 正则表达式匹配所有数字（整数、负数、浮点数）
        boost::regex num_regex(R"(([+-]?\d+\.?\d*))");
        boost::sregex_iterator it(inner.begin(), inner.end(), num_regex);
        boost::sregex_iterator end;

        std::vector<std::string> numbers;
        for (; it != end; ++it) {
            numbers.push_back((*it)[1]);
        }
        if (numbers.size() >= 3) {
            result.x = std::stod(numbers[0]);
            result.y = std::stod(numbers[1]);
            result.z = std::stod(numbers[2]);
        }
    }
    return result;
}

// PROJ database discovery — shared implementation, single source of truth.
// See MeshProjectionErrorCorrector/PROJUtils.h
#include "../MeshProjectionErrorCorrector/PROJUtils.h"

static void TestPROJ()
{
    PJ_CONTEXT* ctx = proj_context_create();
    if (!ctx) {
        std::cerr << "[PROJ] ERROR: proj_context_create() failed!" << std::endl;
        return;
    }

    std::string dbDir = FindPROJDatabase();
    if (dbDir.empty()) {
        std::cerr << "[PROJ] ERROR: proj.db not found! "
                  << "Set PROJ_LIB or place proj.db next to the executable."
                  << std::endl;
        proj_context_destroy(ctx);
        return;
    }
    proj_context_set_database_path(ctx, (dbDir + "/proj.db").c_str(),
                                    nullptr, nullptr);

    PJ_INFO info = proj_info();
    std::cerr << "[PROJ] " << info.major << "." << info.minor << "." << info.patch;
    std::cerr << "  database: " << dbDir << std::endl;

    // Quick transform test
    PJ* transform = proj_create_crs_to_crs(ctx, "EPSG:4326", "EPSG:4979", nullptr);
    if (transform) {
        std::cerr << "[PROJ] EPSG:4326->EPSG:4979: OK" << std::endl;
        proj_destroy(transform);
    } else {
        std::cerr << "[PROJ] EPSG:4326->EPSG:4979: FAILED (errno="
                  << proj_context_errno(ctx) << ")" << std::endl;
    }

    proj_context_destroy(ctx);
    std::cerr << "[PROJ] Test complete." << std::endl;
}


// ---------------------------------------------------------------------------
// Shared CLI helpers (safe numeric parsing, common option groups)
// ---------------------------------------------------------------------------

// Exit code convention: 0 = success, 1 = conversion failure, 2 = usage error.
constexpr int kExitOk = 0;
constexpr int kExitFail = 1;
constexpr int kExitUsage = 2;

static bool ParseDoubleSafe(const std::string& sub, const std::string& opt,
                            const char* raw, double& out)
{
    try {
        size_t pos = 0;
        std::string s(raw);
        out = std::stod(s, &pos);
        if (pos != s.size()) throw std::invalid_argument("trailing chars");
        return true;
    } catch (...) {
        std::cerr << "mgo " << sub << ": " << opt << " expects a number, got '"
                  << raw << "'\n";
        return false;
    }
}

static bool ParseIntSafe(const std::string& sub, const std::string& opt,
                         const char* raw, int& out)
{
    double v;
    if (!ParseDoubleSafe(sub, opt, raw, v)) return false;
    out = static_cast<int>(v);
    return true;
}

// --prj value resolution: an existing .prj file is passed through as a path;
// anything else is an inline CRS spec (EPSG:<code> | WKT | +proj=...), which
// the converters read via ifstream(path), so materialize it to a temp file.
static std::string ResolvePrjArg(const std::string& value)
{
    if (value.empty()) return value;
    if (fs::is_regular_file(value)) return value;
    static int seq = 0;  // cross-platform unique suffix (no getpid on Windows)
    auto tmp = fs::temp_directory_path() /
        ("mgo_crs_" + std::to_string(++seq) + ".prj");
    std::ofstream f(tmp, std::ios::trunc);
    if (f) { f << value; f.close(); }
    return tmp.string();
}

// View over the georeferencing fields shared by tiles/terrain/osgb options.
struct GeorefOpts
{
    GeoreferencingType* type;
    double* helmert;                 // 7 values
    std::vector<ControlPoint>* cps;
    int* polyOrder;
};

// Parses one georeferencing argument. Returns 0 = not ours,
// 1 = consumed, 2 = consumed with error.
static int ParseGeorefArg(const std::string& sub, const std::string& arg,
                          int& i, int argc, char** argv, GeorefOpts& g)
{
    if (arg == "--7p" && i + 1 < argc)
    {
        *g.type = GeoreferencingType::SevenParam;
        if (sscanf(argv[++i], "%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                   &g.helmert[0], &g.helmert[1], &g.helmert[2],
                   &g.helmert[3], &g.helmert[4], &g.helmert[5],
                   &g.helmert[6]) != 7)
        {
            std::cerr << "mgo " << sub
                      << ": --7p expects mx,my,mz,rx,ry,rz,s\n";
            return kExitUsage;
        }
        return kExitOk + 1;
    }
    if (arg == "--cps" && i + 1 < argc)
    {
        *g.type = GeoreferencingType::MultiPosition;
        std::string cpsPath = gbk_to_utf8(argv[++i]);
        cpsPath = fs::absolute(cpsPath).string();
        auto cpList = CSVReader::Read<ControlPointLoader>(cpsPath);
        g.cps->assign(cpList.begin(), cpList.end());
        std::cerr << "Control points: " << g.cps->size() << std::endl;
        return kExitOk + 1;
    }
    if (arg == "--georef" && i + 1 < argc)
    {
        std::string mode = argv[++i];
        if (mode == "multipos")    *g.type = GeoreferencingType::MultiPosition;
        else if (mode == "anchor") *g.type = GeoreferencingType::Anchor;
        else if (mode == "7param") *g.type = GeoreferencingType::SevenParam;
        else {
            std::cerr << "mgo " << sub
                      << ": --georef expects 7param/multipos/anchor, got '"
                      << mode << "'\n";
            return kExitUsage;
        }
        return kExitOk + 1;
    }
    if (arg == "--fit-order" && i + 1 < argc)
    {
        int v;
        if (!ParseIntSafe(sub, "--fit-order", argv[++i], v)) return kExitUsage;
        if (v < 1 || v > 3) {
            std::cerr << "mgo " << sub << ": --fit-order must be 1..3\n";
            return kExitUsage;
        }
        *g.polyOrder = v;
        return kExitOk + 1;
    }
    return 0;
}

// View over the simplification fields shared by tiles/terrain/osgb options.
struct SimplifyOpts
{
    float* error;
    float* normalWeight;
    float* threshold;     // nullptr when the command has no --threshold
    bool* lockBorder;
    bool hasShortS;
};

static int ParseSimplifyArg(const std::string& sub, const std::string& arg,
                            int& i, int argc, char** argv, SimplifyOpts& s)
{
    if (arg == "--error" && i + 1 < argc) {
        double v;
        if (!ParseDoubleSafe(sub, "--error", argv[++i], v)) return kExitUsage;
        *s.error = static_cast<float>(v);
        return kExitOk + 1;
    }
    if (arg == "--nweight" && i + 1 < argc) {
        double v;
        if (!ParseDoubleSafe(sub, "--nweight", argv[++i], v)) return kExitUsage;
        *s.normalWeight = static_cast<float>(v);
        return kExitOk + 1;
    }
    if (arg == "--threshold" && i + 1 < argc) {
        double v;
        if (!ParseDoubleSafe(sub, "--threshold", argv[++i], v)) return kExitUsage;
        *s.threshold = static_cast<float>(v);
        return kExitOk + 1;
    }
    if (arg == "--lock-border") { *s.lockBorder = true; return kExitOk + 1; }
    return 0;
}

static void PrintTopHelp()
{
    std::cout << "MGO v" << MGO_VERSION_STRING
              << " - Computational software for Mesh Generation Optimizer & 3D Tiles Converter\n"
              << MGO_COPYRIGHT_STRING << "\n\n"
              << "Subcommands:\n"
              << "  mgo mesh     Mesh simplification + coordinate projection\n"
              << "  mgo tiles    FBX/OBJ -> 3D Tiles (b3dm + tileset.json)\n"
              << "  mgo terrain  GeoTIFF -> Quantized-Mesh terrain tiles\n"
              << "  mgo image    DOM orthophoto -> TMS image tiles\n"
              << "  mgo geojson  GeoJSON projection conversion\n"
#ifdef HAS_OSGB_CONVERTER
              << "  mgo osgb     OSGB oblique photography -> 3D Tiles\n"
#endif
              << "  mgo version  Version information\n"
              << "\nRun 'mgo <subcommand> --help' for detailed options.\n"
              << "Exit codes: 0 success, 1 conversion failure, 2 usage error.\n";
}

// mesh subcommand — simplification + coordinate projection. Also the default
// path when options are given without a subcommand (e.g. `mgo -i x -o y`).
static int RunMesh(int argc, char** argv);

int main(int argc, char** argv)
{
#ifdef _WIN32
    system("chcp 65001");
#endif

    if (argc <= 1)
    {
        PrintTopHelp();
        return kExitUsage;
    }

    // ---- Subcommand dispatch ----
    {
        std::string subcmd = argv[1];

        if (subcmd == "version" || subcmd == "--version" || subcmd == "-V")
        {
            std::cout << "MGO v" << MGO_VERSION_STRING << " (build "
                      << __DATE__ << " " << __TIME__ << ")\n";
            std::cout << MGO_COPYRIGHT_STRING << "\n";
            std::cout << "PROJ " << PROJ_VERSION_MAJOR << "." << PROJ_VERSION_MINOR
                      << "." << PROJ_VERSION_PATCH << "\n";
#ifdef HAS_OSGB_CONVERTER
            std::cout << "OSGB support: enabled\n";
#else
            std::cout << "OSGB support: disabled\n";
#endif
            return kExitOk;
        }

        if (subcmd == "tiles" || subcmd == "tilesconverter")
        {
            TilesConverter converter;
            TilesConverterOptions opts;
            std::string inputFile;
            GeorefOpts georef = { &opts.georefType, opts.helmert,
                                  &opts.controlPoints, &opts.polyOrder };
            SimplifyOpts simpl = { &opts.simplify.error, &opts.simplify.normalWeight,
                                   &opts.simplify.threshold, &opts.simplify.lockBorder, true };
            for (int i = 2; i < argc; ++i)
            {
                std::string arg = argv[i];
                if (arg == "-i" && i + 1 < argc)      inputFile = gbk_to_utf8(argv[++i]);
                else if (arg == "-o" && i + 1 < argc) opts.outputDir = gbk_to_utf8(argv[++i]);
                else if (arg == "-e" && i + 1 < argc) { if (!ParseDoubleSafe("tiles", "-e", argv[++i], opts.rootGeometricError)) return kExitUsage; }
                else if (arg == "-t" && i + 1 < argc) { if (!ParseDoubleSafe("tiles", "-t", argv[++i], opts.tileGeometricError)) return kExitUsage; }
                else if (arg == "-r" && i + 1 < argc)
                {
                    std::string r = argv[++i];
                    if (r != "ADD" && r != "REPLACE")
                    {
                        std::cerr << "mgo tiles: -r expects ADD or REPLACE\n";
                        return kExitUsage;
                    }
                    opts.refine = r;
                }
                else if (arg == "-Z")                 opts.inputIsZUp = true;
                else if (arg == "--prj" && i + 1 < argc) opts.prjFile = ResolvePrjArg(gbk_to_utf8(argv[++i]));
                else if (arg == "--origin" && i + 1 < argc)
                {
                    if (sscanf(argv[++i], "%lf,%lf,%lf", &opts.originX, &opts.originY, &opts.originZ) != 3)
                    {
                        std::cerr << "mgo tiles: --origin expects x,y,z\n";
                        return kExitUsage;
                    }
                }
                else if(arg == "--min-block" && i + 1 < argc) { if (!ParseDoubleSafe("tiles", "--min-block", argv[++i], opts.minBlockDistance)) return kExitUsage; }
                else if (arg == "--max-lod" && i + 1 < argc) { int v; if (!ParseIntSafe("tiles", "--max-lod", argv[++i], v)) return kExitUsage; opts.maxLODLevels = v; }
                else if (int sr = ParseSimplifyArg("tiles", arg, i, argc, argv, simpl)) { if (sr == kExitUsage) return sr; }
                else if (int gr = ParseGeorefArg("tiles", arg, i, argc, argv, georef)) { if (gr == kExitUsage) return gr; }
                else if (arg == "-h" || arg == "--help")
                {
                    std::cout << "mgo tiles — FBX/OBJ to 3D Tiles\n"
                              << "  -i <file>           Input model\n"
                              << "  -o <dir>            Output directory\n"
                              << "  -e <val>            Root geometric error (default: 500.0)\n"
                              << "  -t <val>            Tile geometric error (default: 50.0)\n"
                              << "  -r <ADD|REPLACE>    Refine mode (default: ADD)\n"
                              << "  -Z                  Input is Z-up\n"
                              << "  --prj <file|spec>   Projection: .prj file path, or inline\n"
                              << "                      CRS (EPSG:<code> | WKT | +proj=...)\n"
                              << "  --origin <x,y,z>    Coordinate origin (default: 0,0,0)\n"
                              << "  --min-block <val>   Minimum block distance (default: 100.0)\n"
                              << "  --max-lod <N>       Max LOD levels (default: 5)\n"
                              << "  --7p <mx,my,mz,rx,ry,rz,s>  7-param Helmert transform\n"
                              << "  --cps <f>           Control points CSV (for multipos)\n"
                              << "  --georef <mode>     Georef mode: 7param/multipos/anchor\n"
                              << "  --fit-order <N>     Polynomial fit order (1/2/3, multipos only)\n"
                              << "  --error <val>       Simplification error (default: 0.01)\n"
                              << "  --nweight <val>     Normal weight (default: 0.1)\n"
                              << "  --threshold <val>   Ratio threshold (default: 0.1, error-driven)\n"
                              << "  --lock-border       Enable border vertex locking\n";
                    return 0;
                }
                else if (!arg.empty() && arg[0] == '-')
                {
                    std::cerr << "mgo tiles: unknown option " << arg << "\n";
                    return kExitUsage;
                }
            }
            if (inputFile.empty() || opts.outputDir.empty())
            {
                std::cerr << "mgo tiles: -i <file> and -o <dir> are required\n";
                return kExitUsage;
            }
            Assimp::Importer importer;
            const aiScene* scene = importer.ReadFile(inputFile,
                aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_GenBoundingBoxes);
            if (!scene) { std::cerr << "Failed to load: " << inputFile << std::endl; return 1; }
            opts.fbxDirectory = inputFile.substr(0, inputFile.find_last_of("/\\"));
            return converter.Convert(scene, opts) ? 0 : 1;
        }

        if (subcmd == "terrain")
        {
            TerrainConverter converter;
            TerrainConverterOptions opts;
            GeorefOpts georef = { &opts.georefType, opts.helmert,
                                  &opts.controlPoints, &opts.polyOrder };
            SimplifyOpts simpl = { &opts.simplify.error, &opts.simplify.normalWeight,
                                   &opts.simplify.threshold, &opts.simplify.lockBorder, false };
            for (int i = 2; i < argc; ++i)
            {
                std::string arg = argv[i];
                if (arg == "-i" && i + 1 < argc)      opts.inputTif = gbk_to_utf8(argv[++i]);
                else if (arg == "-o" && i + 1 < argc) opts.outputDir = gbk_to_utf8(argv[++i]);
                else if (arg == "--prj" && i + 1 < argc) opts.prjFile = ResolvePrjArg(gbk_to_utf8(argv[++i]));
                else if (arg == "--origin" && i + 1 < argc) {
                    if (sscanf(argv[++i], "%lf,%lf,%lf", &opts.originX, &opts.originY, &opts.originZ) != 3)
                    {
                        std::cerr << "mgo terrain: --origin expects x,y,z\n";
                        return kExitUsage;
                    }
                    opts.hasOrigin = true;
                }
                else if (arg == "--max-lod" && i + 1 < argc) { int v; if (!ParseIntSafe("tiles", "--max-lod", argv[++i], v)) return kExitUsage; opts.maxLODLevels = v; }
                else if (arg == "--samples" && i + 1 < argc)  { int v; if (!ParseIntSafe("terrain", "--samples", argv[++i], v)) return kExitUsage; opts.samplesPerTile = v; }
                else if (int sr = ParseSimplifyArg("terrain", arg, i, argc, argv, simpl)) { if (sr == kExitUsage) return sr; }
                else if (arg == "--no-normals")                opts.writeOctVertexNormals = false;
                else if (arg == "-v")                          opts.verbose = true;
                else if (int gr = ParseGeorefArg("terrain", arg, i, argc, argv, georef)) { if (gr == kExitUsage) return gr; }
                else if (arg == "-h" || arg == "--help")
                {
                    std::cout << "mgo terrain — GeoTIFF to Quantized-Mesh terrain tiles\n"
                              << "  -i <file>           Input GeoTIFF\n"
                              << "  -o <dir>            Output directory\n"
                              << "  --prj <file|spec>   Override projection: .prj file path, or\n"
                              << "                      inline CRS (EPSG:<code> | WKT | +proj=...)\n"
                              << "  --origin <x,y,z>    Override TIF tiepoint (default: from TIF)\n"
                              << "  --max-lod <N>       Max LOD level\n"
                              << "  --samples <N>       Samples per tile (default: 65)\n"
                              << "  --error <val>       Simplification error (default: 0.001, normalized)\n"
                              << "  --nweight <val>     Normal weight (default: 0.0)\n"
                              << "  --threshold <val>   Ratio threshold (default: 0.1, error-driven)\n"
                              << "  --lock-border       Enable border vertex locking\n"
                              << "  --no-normals        Skip OctVertexNormals extension\n"
                              << "  --7p <mx,..,s>      7-param Helmert transform\n"
                              << "  --cps <f>           Control points CSV (for multipos)\n"
                              << "  --georef <mode>     Georef mode: 7param/multipos/anchor\n"
                              << "  --fit-order <N>     Polynomial fit order (1/2/3, multipos only)\n"
                              << "  -v                  Verbose output\n";
                    return 0;
                }
                else if (!arg.empty() && arg[0] == '-')
                {
                    std::cerr << "mgo terrain: unknown option " << arg << "\n";
                    return kExitUsage;
                }
            }
            if (opts.inputTif.empty() || opts.outputDir.empty())
            {
                std::cerr << "mgo terrain: -i <file> and -o <dir> are required\n";
                return kExitUsage;
            }
            if (opts.samplesPerTile < 2)
            {
                std::cerr << "mgo terrain: --samples must be >= 2 (got "
                          << opts.samplesPerTile << ")\n";
                return kExitUsage;
            }
            if (opts.samplesPerTile > 255)
            {
                std::cerr << "mgo terrain: --samples must be <= 255 to keep "
                          << "vertex count below the uint16 index limit "
                          << "(255²=" << 255*255 << " < 65535; got "
                          << opts.samplesPerTile << ", would produce up to "
                          << opts.samplesPerTile * opts.samplesPerTile
                          << " vertices)\n";
                return kExitUsage;
            }
            return converter.Convert(opts) ? 0 : 1;
        }

        if (subcmd == "image" || subcmd == "imagetiler")
        {
            ImageTiler tiler;
            ImageTilerOptions opts;
            for (int i = 2; i < argc; ++i)
            {
                std::string arg = argv[i];
                if (arg == "-i" && i + 1 < argc)      opts.inputTif = gbk_to_utf8(argv[++i]);
                else if (arg == "-o" && i + 1 < argc) opts.outputDir = gbk_to_utf8(argv[++i]);
                else if (arg == "--prj" && i + 1 < argc) opts.prjFile = ResolvePrjArg(gbk_to_utf8(argv[++i]));
                else if (arg == "-h" || arg == "--help")
                {
                    std::cout << "mgo image — DOM orthophoto to TMS image tiles\n"
                              << "  -i <file>      Input GeoTIFF\n"
                              << "  -o <dir>       Output directory\n"
                              << "  --prj <file|spec>  Projection: .prj file path, or inline\n"
                              << "                 CRS (EPSG:<code> | WKT | +proj=...)\n";
                    return 0;
                }
                else if (!arg.empty() && arg[0] == '-')
                {
                    std::cerr << "mgo image: unknown option " << arg << "\n";
                    return kExitUsage;
                }
            }
            if (opts.inputTif.empty() || opts.outputDir.empty())
            {
                std::cerr << "mgo image: -i <file> and -o <dir> are required\n";
                return kExitUsage;
            }
            return tiler.Convert(opts) ? 0 : 1;
        }

#ifdef HAS_OSGB_CONVERTER
        if (subcmd == "osgb")
        {
            OSGBConverter converter;
            OSGBConverterOptions opts;
            GeorefOpts georef = { &opts.georefType, opts.helmert,
                                  &opts.controlPoints, &opts.polyOrder };
            SimplifyOpts simpl = { &opts.simplify.error, &opts.simplify.normalWeight,
                                   &opts.simplify.threshold, &opts.simplify.lockBorder, true };
            for (int i = 2; i < argc; ++i)
            {
                std::string arg = argv[i];
                if (arg == "-i" && i + 1 < argc)      opts.inputDir = gbk_to_utf8(argv[++i]);
                else if (arg == "-o" && i + 1 < argc) opts.outputDir = gbk_to_utf8(argv[++i]);
                else if (arg == "--prj" && i + 1 < argc) opts.prjFile = ResolvePrjArg(gbk_to_utf8(argv[++i]));
                else if (arg == "--origin" && i + 1 < argc)
                {
                    if (sscanf(argv[++i], "%lf,%lf,%lf", &opts.originX, &opts.originY, &opts.originZ) != 3)
                    {
                        std::cerr << "mgo osgb: --origin expects x,y,z\n";
                        return kExitUsage;
                    }
                }
                else if (arg == "--enu" && i + 1 < argc)
                {
                    opts.hasENUOverride =
                        sscanf(argv[++i], "%lf,%lf,%lf", &opts.enuLat, &opts.enuLon, &opts.enuH) >= 2;
                    if (!opts.hasENUOverride)
                    {
                        std::cerr << "mgo osgb: --enu expects <lat>,<lon>[,<height>]" << std::endl;
                        return kExitUsage;
                    }
                }
                else if (arg == "--max-lod" && i + 1 < argc) { int v; if (!ParseIntSafe("osgb", "--max-lod", argv[++i], v)) return kExitUsage; opts.maxLOD = v; }
                else if (int sr = ParseSimplifyArg("osgb", arg, i, argc, argv, simpl)) { if (sr == kExitUsage) return sr; }
                else if (int gr = ParseGeorefArg("osgb", arg, i, argc, argv, georef)) { if (gr == kExitUsage) return gr; }
                else if (arg == "-v") opts.verbose = true;
                else if (arg == "-h" || arg == "--help")
                {
                    std::cout << "mgo osgb — OSGB oblique photography to 3D Tiles\n"
                              << "  -i <dir>            Input OSGB directory\n"
                              << "  -o <dir>            Output directory\n"
                              << "  --prj <file|spec>   Projection: .prj file path, or inline\n"
                              << "                      CRS (EPSG:<code> | WKT | +proj=...)\n"
                              << "  --enu <lat,lon[,h]> ENU tangent plane override (bypasses --prj)\n"
                              << "  --origin <x,y,z>    Coordinate origin override\n"
                              << "  --max-lod <N>       Max LOD level to convert\n"
                              << "  --7p <mx,..,s>      7-param Helmert transform\n"
                              << "  --cps <f>           Control points CSV (for multipos)\n"
                              << "  --georef <mode>     Georef mode: 7param/multipos/anchor\n"
                              << "  --fit-order <N>     Polynomial fit order (1/2/3, multipos only)\n"
                              << "  --error <val>       Simplification error (default: 0.01)\n"
                              << "  --nweight <val>     Normal weight (default: 0.1)\n"
                              << "  --threshold <val>   Ratio threshold (default: 0.1, error-driven)\n"
                              << "  --lock-border       Enable border vertex locking\n"
                              << "  -v                  Verbose\n";
                    return 0;
                }
                else if (!arg.empty() && arg[0] == '-')
                {
                    std::cerr << "mgo osgb: unknown option " << arg << "\n";
                    return kExitUsage;
                }
            }
            if (opts.inputDir.empty() || opts.outputDir.empty())
            {
                std::cerr << "mgo osgb: -i <dir> and -o <dir> are required\n";
                return kExitUsage;
            }
            if (!converter.Convert(opts))
            {
                std::cerr << "Error: " << converter.GetLastError() << std::endl;
                return 1;
            }
            return 0;
        }
#endif

        if (subcmd == "geojson")
        {
            GeoJSONConverter converter;
            GeoJSONConverterOptions opts;
            for (int i = 2; i < argc; ++i)
            {
                std::string arg = argv[i];
                if (arg == "-i" && i + 1 < argc)        opts.inputFile = gbk_to_utf8(argv[++i]);
                else if (arg == "-o" && i + 1 < argc)   opts.outputFile = gbk_to_utf8(argv[++i]);
                else if (arg == "--source-crs" && i + 1 < argc) opts.sourceCRS = argv[++i];
                else if (arg == "--target-crs" && i + 1 < argc) opts.targetCRS = argv[++i];
                else if (arg == "--pretty")             opts.pretty = true;
                else if (arg == "-v")                   opts.verbose = true;
                else if (arg == "-h" || arg == "--help")
                {
                    std::cout << "mgo geojson - GeoJSON projection conversion\n"
                              << "  -i <file>          Input GeoJSON file\n"
                              << "  -o <file>          Output GeoJSON file\n"
                              << "  --source-crs <crs> Source CRS (default: crs member or EPSG:4326)\n"
                              << "  --target-crs <crs> Target CRS (default: EPSG:4326)\n"
                              << "  CRS forms: EPSG:<code> | ENU:<lat>,<lon>[,<h>] | WKT | +proj=... | <file.prj>\n"
                              << "  --pretty           Pretty-print output\n";
                    return 0;
                }
                else if (!arg.empty() && arg[0] == '-')
                {
                    std::cerr << "mgo geojson: unknown option " << arg << "\n";
                    return kExitUsage;
                }
            }
            if (opts.inputFile.empty() || opts.outputFile.empty())
            {
                std::cerr << "mgo geojson: -i <file> and -o <file> are required\n";
                return kExitUsage;
            }
            // Accept a .prj file path as a CRS spec for convenience.
            auto loadPrj = [&opts](const std::string& spec) -> std::string {
                if (spec.size() >= 4)
                {
                    std::string ext = spec.substr(spec.size() - 4);
                    for (auto& c : ext) c = static_cast<char>(std::tolower(c));
                    if (ext == ".prj")
                    {
                        std::ifstream f(spec);
                        if (f)
                            return std::string((std::istreambuf_iterator<char>(f)),
                                               std::istreambuf_iterator<char>());
                    }
                }
                return spec;
            };
            opts.sourceCRS = loadPrj(opts.sourceCRS);
            opts.targetCRS = loadPrj(opts.targetCRS);
            if (!converter.Convert(opts))
            {
                std::cerr << "Error: " << converter.GetLastError() << std::endl;
                return 1;
            }
            return 0;
        }

        if (subcmd == "help" || subcmd == "--help" || subcmd == "-h")
        {
            PrintTopHelp();
            return kExitOk;
        }

        if (subcmd == "mesh")
        {
            return RunMesh(argc, argv);
        }

        // Unknown bare-word argument (option-style args such as "-i" fall
        // through to the mesh path below).
        {
            static const std::set<std::string> kSubcommands = {
                "mesh", "tiles", "tilesconverter", "terrain", "image",
                "imagetiler", "osgb", "geojson", "help", "--help", "-h",
                "version", "--version", "-V"
            };
            if (subcmd[0] != '-' && kSubcommands.count(subcmd) == 0)
            {
                std::cerr << "mgo: unknown subcommand '" << subcmd << "'\n"
                          << "Run 'mgo help' for the subcommand list.\n";
                return kExitUsage;
            }
        }
    }

    // ---- mesh: default path (bare option-style args) ----
    return RunMesh(argc, argv);
}

// ===========================================================================
// mesh subcommand — simplification + coordinate projection
// ===========================================================================
static int RunMesh(int argc, char** argv)
{
    OptimizerConfig config;
    CMeshGroupOptimizer mgo;
    std::string inputFileName, outputFileName, configFile;

    SimpleArgParser parser;
    parser.AddOption("input", "i", true, "", u8"输入文件名");
    parser.AddOption("output", "o", true, "", u8"输出文件名");
    parser.AddOption("error", "e", true, "0.01", u8"简化误差");
    parser.AddOption("nweight", "n", true, "0.1", u8"角向量简化权重");
    parser.AddOption("threshold", "t", true, "0.0", u8"简化阈值");
    parser.AddOption("reorder", "r", true, "false", u8"重建排序");
    parser.AddOption("rebuild", "R", true, "false", u8"重建优化");
    parser.AddOption("localerror", "l", true, "false", u8"是否使用局部简化误差");
    parser.AddOption("lockborder", "L", true, "true", u8"是否锁定边界");
    parser.AddOption("config", "c", true, "", u8"构件简化参数配置文件");
    parser.AddOption("prj", "p", true, "", u8"投影坐标系: *.prj 文件或内联 CRS 规范");
    parser.AddOption("cps", "", true, "", u8"控制点文件 *.csv(表头为 sx,sy,sz,tx,ty,tz)");
    parser.AddOption("coordinateSystem", "C", true, "original", u8"输出坐标系: original（保持原样）, left（转换为左手坐标系）");
    parser.AddOption("offset", "", true, "", u8"投影偏移 x,y,z (source CRS 内)");
    parser.AddOption("7p", "", true, "", u8"七参数 Helmert: mx,my,mz,rx,ry,rz,s (rx/ry/rz 角秒, s 为 ppm)");
    parser.AddOption("georef", "g", true, "7param", u8"坐标转换方法: 7param（七参数）, multipos（多控制点）, anchor（单锚点）");
    parser.AddOption("fit-order", "", true, "1", u8"多项式拟合阶数 (1/2/3, 仅 multipos 模式)");
    parser.AddOption("auto-crs", "", true, "false", u8"自动反推源坐标系 (仅 multipos 模式)");
    parser.AddOption("proj-test", "", false, "false", u8"验证 PROJ 库功能");

    if (!parser.Parse(argc, argv)) {
        return 1;
    }

    // PROJ quick validation mode
    if (parser.GetBool("proj-test")) {
        TestPROJ();
        return 0;
    }

    if (parser.Has("input"))
    inputFileName = parser.GetString("input");
    outputFileName = parser.GetString("output");

    // Required args are checked before any path normalization (fs::absolute
    // throws on an empty path).
    if (inputFileName.empty())
    {
        std::cerr << u8"mgo mesh: -i <file> 是必需的" << std::endl;
        return kExitUsage;
    }
    if (outputFileName.empty())
    {
        std::cerr << u8"mgo mesh: -o <file> 是必需的" << std::endl;
        return kExitUsage;
    }

    // Convert GBK paths to UTF-8 for Boost.Filesystem on Windows
    std::cout << u8"系统编码为：GBK" << std::endl;
    inputFileName = gbk_to_utf8(inputFileName);
    outputFileName = gbk_to_utf8(outputFileName);
    inputFileName = fs::absolute(inputFileName).string();
    outputFileName = fs::absolute(outputFileName).string();

    if (!fs::is_regular_file(fs::path(inputFileName)))
    {
        std::cout << u8"无效的输入文件 : " << inputFileName << std::endl;
        return kExitFail;
    }

    if (parser.Has("config"))
    {
        configFile = parser.GetString("config");
        std::cout << configFile << std::endl;
        configFile = gbk_to_utf8(configFile);
        configFile = fs::absolute(configFile).string();
        std::cout << u8"读取模型简化配置文件 !" << std::endl;
        std::vector<OptimizerItemLoader> oplist = CSVReader::Read<OptimizerItemLoader>(configFile);

        for (auto& item : oplist)
            config.items.push_back(item);
    }
    else
    {
        std::cout << u8"使用默认简化配置 !" << std::endl;

        float error = parser.GetFloat("error");
        float nweight = parser.GetFloat("nweight");
        float threshold = parser.GetFloat("threshold");
        bool lockborder = parser.GetBool("lockborder");
        bool localerror = parser.GetBool("localerror");

        config.items.push_back(OptimizerItem(".*", error, nweight, threshold, lockborder, localerror));
    }

    std::cout << u8"读取模型.." << std::endl;

    const aiScene* scene = mgo.Load(inputFileName, parser.GetBool("rebuild"));
    if (!scene)
    {
        std::cout << u8"无效的输入文件 !" << std::endl;
        return kExitFail;
    }

    // ResolvePrjArg: an existing .prj file passes through as a path; an inline
    // CRS spec (EPSG:<code> | WKT | +proj=...) is materialized to a temp file.
    std::string prj_path = ResolvePrjArg(parser.GetString("prj"));

    // 先执行简化优化（优化器工作在局部坐标系上）
    config.reorder = parser.GetBool("reorder");
    mgo.Optimize(config);

    // 再执行投影转换（如有 PRJ 文件）
    if (!prj_path.empty())
    {
        CMeshProjectionErrorCorrector mp;
        // 从 PRJ 文件读取投影定义
        std::ifstream prjFile(prj_path);
        std::stringstream prjBuffer;
        prjBuffer << prjFile.rdbuf();
        std::string strproj = prjBuffer.str();
        // 去除首尾空白
        boost::trim(strproj);

        // 如果 PRJ 文件内容为空或读取失败，使用默认 TM 投影
        if (strproj.empty())
        {
            strproj = "+proj=tmerc +lon_0=100.35 +lat_0=0 +k=1 +x_0=500000 +y_0=0 +ellps=GRS80 +units=m +type=crs";
        }

        // PROJ 字符串规范化已由 CProjectionEngine::LoadProjectionFromString /
        // IGeoreferencing::InitPROJPipelines 内部处理（+no_defs 剥离、
        // +type=crs 添加、+ellps 替换等）。直接传递原始字符串即可。


        // 读取命令行偏移量
        double offsetX = 0, offsetY = 0, offsetZ = 0;
        std::string off = parser.GetString("offset");
        if (!off.empty())
        {
            if (sscanf(off.c_str(), "%lf,%lf,%lf", &offsetX, &offsetY, &offsetZ) != 3)
            {
                std::cerr << u8"mgo mesh: --offset 需要 x,y,z" << std::endl;
                return kExitUsage;
            }
        }

        std::cerr << u8"投影参数: " << strproj << std::endl;
        std::cerr << u8"偏移: (" << offsetX << ", " << offsetY << ", " << offsetZ << ")" << std::endl;

        std::string georefType = parser.GetString("georef");

        if (georefType == "multipos")
        {
            // 多控制点坐标转换
            std::string cpsPath = parser.GetString("cps");
            if (cpsPath.empty())
            {
                std::cerr << u8"使用 multipos 方法时必须指定 --cps 控制点文件" << std::endl;
                return 1;
            }
            cpsPath = gbk_to_utf8(cpsPath);
            cpsPath = fs::absolute(cpsPath).string();
            std::vector<ControlPointLoader> cpList = CSVReader::Read<ControlPointLoader>(cpsPath);
            std::cerr << u8"控制点数量: " << cpList.size() << std::endl;
            if (cpList.size() < 4)
            {
                std::cerr << u8"至少需要 4 个控制点" << std::endl;
                return 1;
            }

            int fitOrder = (int)parser.GetDouble("fit-order");
            bool autoCrs = parser.GetBool("auto-crs");
            bool usePoly = strproj.empty() || fitOrder > 1 || autoCrs;

            // ECEF_Affine mode uses PROJ pipelines. For WKT with custom
            // datums that PROJ can't parse, convert to a clean PROJ string.
            // Normalization (+no_defs stripping, +type=crs, +ellps) is handled
            // by IGeoreferencing::InitPROJPipelines / NormalizePROJString.
            if (!usePoly) {
                if (strproj.find("PROJCS[") != std::string::npos || strproj.find("GEOGCS[") != std::string::npos) {
                    strproj = wkt_to_proj_string(strproj);
                }
            }

            GeoreferencingWithMultiPosition mpGeorefencing(
                usePoly ? "" : strproj, "EPSG:4979");
            std::vector<ControlPoint> cps(cpList.begin(), cpList.end());

            if (usePoly) {
                mpGeorefencing.SetFitMethod(FitMethod::DirectPoly2D);
                mpGeorefencing.SetPolyOrder(fitOrder);
                std::cerr << u8"多项式拟合模式: 阶数=" << fitOrder
                          << u8", 控制点=" << cpList.size() << std::endl;

                if (autoCrs) {
                    std::cerr << u8"\n自动反推源坐标系..." << std::endl;
                    auto crsResults = mpGeorefencing.DetectSourceCRS(5);
                    if (!crsResults.empty()) {
                        std::cerr << u8"最佳候选: " << crsResults[0].crs
                                  << u8" (RMS=" << crsResults[0].rms_degrees << u8"°)" << std::endl;
                    }
                }
            } else {
                mpGeorefencing.SetFitMethod(FitMethod::ECEF_Affine);
                std::cerr << u8"ECEF 仿射变换模式" << std::endl;
            }
            try {
                mpGeorefencing.SetParameter(cps);
                mpGeorefencing.Solve();
                mp.Transform(&mpGeorefencing, scene, offsetX, offsetY, offsetZ, false);
            } catch (const std::exception& e) {
                std::cerr << u8"坐标转换失败: " << e.what() << std::endl;
                return 1;
            }
        }
        else if (georefType == "anchor")
        {
            // 单锚点坐标转换
            Eigen::Vector3d anchor(offsetX, offsetY, offsetZ);
            GeoreferencingWithAnchor mpGeorefencing(strproj, "EPSG:4979");
            mpGeorefencing.SetParameter(anchor);
            mpGeorefencing.Solve();
            mp.Transform(&mpGeorefencing, scene, offsetX, offsetY, offsetZ, false);
        }
        else
        {
            // 默认: 七参数坐标转换
            std::string p7 = parser.GetString("7p");
            if (p7.empty())
            {
                std::cerr << u8"mgo mesh: 七参数模式需要 --7p mx,my,mz,rx,ry,rz,s" << std::endl;
                return kExitUsage;
            }
            SevenParameter helmert;
            if (sscanf(p7.c_str(), "%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                       &helmert.mx, &helmert.my, &helmert.mz,
                       &helmert.rx, &helmert.ry, &helmert.rz, &helmert.scale) != 7)
            {
                std::cerr << u8"mgo mesh: --7p 需要 mx,my,mz,rx,ry,rz,s" << std::endl;
                return kExitUsage;
            }
            std::cerr << u8"七参数 Helmert: (" << helmert.mx << ", " << helmert.my << ", " << helmert.mz
                << ") m  (" << helmert.rx << ", " << helmert.ry << ", " << helmert.rz
                << ") arcsec  scale=" << helmert.scale << " ppm" << std::endl;

            GeoreferencingWith7Parameters mpGeorefencing(strproj, "EPSG:4979");
            mpGeorefencing.SetParameter(helmert);
            mpGeorefencing.Solve();
            mp.Transform(&mpGeorefencing, scene, offsetX, offsetY, offsetZ, false);
        }
    }

    std::string coordSys = parser.GetString("coordinateSystem");
    if (coordSys != "original" && coordSys != "left")
    {
        std::cerr << u8"mgo mesh: -C 只接受 original 或 left, 得到 '" << coordSys << "'" << std::endl;
        return kExitUsage;
    }
    if (!outputFileName.empty()) mgo.Save(outputFileName, coordSys == "left");

    std::cout << u8"优化完成 !" << std::endl;
    return 0;
}
