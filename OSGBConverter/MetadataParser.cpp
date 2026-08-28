// Copyright Johnlyon
//
// MetadataParser implementation — parse metadata.xml from OSGB datasets
//

#include "MetadataParser.h"
#include "IVendorHandler.h"
#include "PlatformCompat.h"
#include "../MeshProjectionErrorCorrector/CProjectionEngine.h"
#include "../MeshProjectionErrorCorrector/Constants.h"

#include <fstream>
#include <sstream>
#include <cstring>
#include <cctype>
#include <cstdio>
#include <algorithm>
#include <iostream>

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

bool MetadataParser::Parse(const std::string& xmlPath, OSGBMetadata& outMetadata)
{
    std::ifstream file(xmlPath);
    if (!file.is_open())
    {
        std::cerr << "[MetadataParser] Cannot open: " << xmlPath << std::endl;
        return false;
    }

    std::ostringstream buf;
    buf << file.rdbuf();
    std::string xml = buf.str();

    size_t pos = xmlPath.find_last_of("/\\");
    std::string baseDir = (pos != std::string::npos) ? xmlPath.substr(0, pos) : ".";

    return ParseString(xml, baseDir, outMetadata);
}

bool MetadataParser::ParseWithHandler(const std::string& xmlPath,
                                       OSGBMetadata& outMetadata,
                                       IVendorHandler& handler)
{
    std::ifstream file(xmlPath);
    if (!file.is_open())
        return false;

    std::ostringstream buf;
    buf << file.rdbuf();
    std::string xml = buf.str();

    size_t pos = xmlPath.find_last_of("/\\");
    std::string baseDir = (pos != std::string::npos) ? xmlPath.substr(0, pos) : ".";

    m_baseDir   = baseDir;
    m_metadata  = &outMetadata;
    outMetadata = OSGBMetadata();

    ParseXML(xml, [&](const Element& el) {
        handler.ParseMetadataElement(el.name, el.text, outMetadata);
    });

    // Extract bounding box
    ExtractBBoxFromXML(xml, outMetadata.bboxMin, outMetadata.bboxMax);

    // Determine max LOD from tile paths
    outMetadata.maxLOD = 0;
    for (const auto& path : outMetadata.tilePaths)
    {
        int level, x, y;
        std::string subIdx;
        if (handler.ParseTilePath(path, level, x, y, subIdx))
        {
            if (level > outMetadata.maxLOD)
                outMetadata.maxLOD = level;
        }
    }

    return !outMetadata.tilePaths.empty() || !outMetadata.srs.empty();
}

bool MetadataParser::ParseString(const std::string& xml, const std::string& baseDir,
                                  OSGBMetadata& outMetadata)
{
    m_baseDir   = baseDir;
    m_metadata  = &outMetadata;
    outMetadata = OSGBMetadata();

    ParseXML(xml, [&](const Element& el) {
        // SRS: ContextCapture uses <SRS>, DJI uses <SpatialReferenceSystem> or <SRSName>
        if (el.name == "SRS" || el.name == "srs" ||
            el.name == "SpatialReferenceSystem" || el.name == "SRSName" || el.name == "srsname")
        {
            outMetadata.srs = el.text;
            outMetadata.projectionMode = DetectProjectionMode(el.text);
            if (ParseENUOrigin(el.text, outMetadata.enuLat, outMetadata.enuLon))
            {
                outMetadata.isENU = true;
                outMetadata.projectionMode = ProjectionMode::PerTile;
            }
        }
        // Origin: ContextCapture uses <SRSOrigin>, DJI uses <Origin>
        else if (el.name == "SRSOrigin" || el.name == "srsorigin" ||
                 el.name == "Origin" || el.name == "origin")
        {
            ParseOrigin(el.text, outMetadata.originX,
                        outMetadata.originY, outMetadata.originZ);
        }
        else if (el.name == "Path" || el.name == "path")
        {
            if (!el.text.empty())
            {
                // Trim whitespace
                std::string path = el.text;
                size_t first = path.find_first_not_of(" \t\n\r");
                size_t last  = path.find_last_not_of(" \t\n\r");
                if (first != std::string::npos && last != std::string::npos)
                    path = path.substr(first, last - first + 1);
                if (!path.empty())
                    outMetadata.tilePaths.push_back(path);
            }
        }
    });

    // Extract bounding box
    ExtractBBoxFromXML(xml, outMetadata.bboxMin, outMetadata.bboxMax);

    // If no tiles found via Path elements, try raw extraction and directory scan
    if (outMetadata.tilePaths.empty())
    {
        ExtractTilesFromXML(xml, outMetadata.tilePaths);
    }

    // Determine max LOD from tile paths
    outMetadata.maxLOD = 0;
    for (const auto& path : outMetadata.tilePaths)
    {
        int level, x, y;
        if (ParseTileLevelAndIndex(path, level, x, y))
        {
            if (level > outMetadata.maxLOD)
                outMetadata.maxLOD = level;
        }
    }

    // If no SRS but we have tiles, default to PerTile
    if (outMetadata.srs.empty() && !outMetadata.tilePaths.empty())
    {
        outMetadata.projectionMode = ProjectionMode::PerTile;
    }

    return !outMetadata.tilePaths.empty();
}

// ---------------------------------------------------------------------------
// XML Parser — simple recursive-descent, handles the subset needed for metadata.xml
// ---------------------------------------------------------------------------

bool MetadataParser::ParseXML(const std::string& xml,
                               std::function<void(const Element&)> onElement)
{
    size_t pos = 0;
    size_t len = xml.size();

    // Skip XML declaration
    if (xml.compare(0, 5, "<?xml") == 0)
    {
        pos = xml.find("?>", pos);
        if (pos != std::string::npos) pos += 2;
    }

    std::vector<std::string> stack;
    std::string currentText;

    while (pos < len)
    {
        while (pos < len && (xml[pos] == ' ' || xml[pos] == '\t' ||
                             xml[pos] == '\n' || xml[pos] == '\r'))
            ++pos;

        if (pos >= len) break;

        if (xml[pos] == '<')
        {
            ++pos;
            if (pos >= len) break;

            // Closing tag: </name>
            if (xml[pos] == '/')
            {
                ++pos;
                size_t end = xml.find('>', pos);
                if (end == std::string::npos) break;

                if (!stack.empty())
                {
                    Element el;
                    el.name = stack.back();
                    el.text = currentText;
                    onElement(el);
                    stack.pop_back();
                    currentText.clear();
                }

                pos = end + 1;
            }
            // Comment: <!-- -->
            else if (pos < len && xml[pos] == '!')
            {
                size_t end = xml.find("-->", pos);
                if (end == std::string::npos) break;
                pos = end + 3;
            }
            // Opening tag or self-closing: <name ...> or <name ... />
            else
            {
                size_t end = xml.find('>', pos);
                if (end == std::string::npos) break;

                std::string tagContent = xml.substr(pos, end - pos);
                bool selfClosing = (!tagContent.empty() && tagContent.back() == '/');
                if (selfClosing)
                    tagContent = tagContent.substr(0, tagContent.size() - 1);

                // Extract tag name
                size_t spacePos = tagContent.find_first_of(" \t\n\r");
                std::string name = (spacePos != std::string::npos)
                    ? tagContent.substr(0, spacePos)
                    : tagContent;

                if (!selfClosing)
                {
                    stack.push_back(name);
                    currentText.clear();
                }
                else
                {
                    Element el;
                    el.name = name;
                    onElement(el);
                }

                pos = end + 1;
            }
        }
        else
        {
            // Text content
            size_t tagStart = xml.find('<', pos);
            if (tagStart == std::string::npos) break;

            currentText += xml.substr(pos, tagStart - pos);
            pos = tagStart;
        }
    }

    return true;
}

// ---------------------------------------------------------------------------
// Helper: extract bounding box from raw XML
// ---------------------------------------------------------------------------

void MetadataParser::ExtractBBoxFromXML(const std::string& xml,
                                         double bboxMin[3], double bboxMax[3])
{
    bboxMin[0] = bboxMin[1] = bboxMin[2] = 0;
    bboxMax[0] = bboxMax[1] = bboxMax[2] = 0;

    auto extractTag = [&](const std::string& tagName, double out[3]) -> bool {
        std::string openTag = "<" + tagName + ">";
        size_t start = xml.find(openTag);
        if (start != std::string::npos)
        {
            start += openTag.size();
            std::string closeTag = "</" + tagName + ">";
            size_t end = xml.find(closeTag, start);
            if (end != std::string::npos)
            {
                std::string text = xml.substr(start, end - start);
                double x, y, z;
                if (ParseOrigin(text, x, y, z))
                {
                    out[0] = x; out[1] = y; out[2] = z;
                    return true;
                }
            }
        }
        return false;
    };

    extractTag("Min", bboxMin);
    extractTag("Max", bboxMax);
}

void MetadataParser::ExtractTilesFromXML(const std::string& xml,
                                          std::vector<std::string>& tiles)
{
    size_t pos = 0;
    while (true)
    {
        size_t start = xml.find("<Path>", pos);
        if (start == std::string::npos)
            start = xml.find("<path>", pos);
        if (start == std::string::npos) break;

        start = xml.find('>', start) + 1;
        size_t end = xml.find("</Path>", start);
        if (end == std::string::npos)
            end = xml.find("</path>", start);
        if (end == std::string::npos) break;

        std::string path = xml.substr(start, end - start);
        size_t first = path.find_first_not_of(" \t\n\r");
        size_t last  = path.find_last_not_of(" \t\n\r");
        if (first != std::string::npos && last != std::string::npos)
            path = path.substr(first, last - first + 1);

        if (!path.empty())
            tiles.push_back(path);

        pos = end + 1;
    }
}

// ---------------------------------------------------------------------------
// Projection mode detection
// ---------------------------------------------------------------------------

ProjectionMode MetadataParser::DetectProjectionMode(const std::string& srs)
{
    if (srs.empty()) return ProjectionMode::None;

    // EPSG:454x series — CGCS2000 / 3-degree Gauss-Kruger zones
    if (srs.find("EPSG:454") == 0 || srs.find("epsg:454") == 0)
        return ProjectionMode::RootOnly;

    // EPSG:452x series — CGCS2000 / 3-degree Gauss-Kruger zones (older)
    if (srs.find("EPSG:452") == 0 || srs.find("epsg:452") == 0)
        return ProjectionMode::RootOnly;

    // EPSG:449x — CGCS2000 geographic 3D
    if (srs.find("EPSG:449") == 0 || srs.find("epsg:449") == 0)
        return ProjectionMode::PerTile;

    // ENU / LOCAL
    if (srs.find("ENU") != std::string::npos ||
        srs.find("enu") != std::string::npos ||
        srs.find("LOCAL") != std::string::npos)
        return ProjectionMode::PerTile;

    // Unknown — default to per-tile
    return ProjectionMode::PerTile;
}

// ---------------------------------------------------------------------------
// Token parsing helpers
// ---------------------------------------------------------------------------

bool MetadataParser::ParseOrigin(const std::string& text, double& x, double& y, double& z)
{
    std::string cleaned = text;
    std::replace(cleaned.begin(), cleaned.end(), ',', ' ');

    std::istringstream iss(cleaned);
    double v[3] = { 0, 0, 0 };
    int count = 0;
    while (iss >> v[count] && count < 3) ++count;

    if (count >= 2)
    {
        x = v[0]; y = v[1]; z = (count >= 3) ? v[2] : 0;
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// LOD level extraction from tile path
// ---------------------------------------------------------------------------

bool MetadataParser::ParseTileLevelAndIndex(const std::string& path,
                                             int& level, int& x, int& y)
{
    level = 0; x = 0; y = 0;

    size_t lastSlash = path.find_last_of("/\\");
    std::string filename = (lastSlash != std::string::npos)
        ? path.substr(lastSlash + 1) : path;

    // --- DJI Terra: level_XX/ directory encodes LOD ---
    if (lastSlash != std::string::npos)
    {
        size_t prevSlash = path.find_last_of("/\\", lastSlash - 1);
        std::string parentDir = (prevSlash != std::string::npos)
            ? path.substr(prevSlash + 1, lastSlash - prevSlash - 1) : path.substr(0, lastSlash);

        if (parentDir.find("level_") == 0)
        {
            // Parse level number from "level_XX"
            level = 0;
            size_t p = 6;  // skip "level_"
            while (p < parentDir.size() && std::isdigit(parentDir[p]))
            {
                level = level * 10 + (parentDir[p] - '0');
                ++p;
            }
            // DJI: higher level number = coarser. Invert for 3D Tiles (0 = coarsest).
            // We'll keep the raw level and let the hierarchy builder handle ordering.
            x = 0; y = 0;
            return true;
        }

        // Check for Block_* root tile (no level_ directory)
        if (parentDir.find("Block_") == 0)
        {
            level = 0;  // root/coarsest
            x = 0; y = 0;
            return true;
        }
    }

    // --- ContextCapture: directory name Tile_+XXX_+YYY ---
    std::string dirName;
    if (lastSlash != std::string::npos)
    {
        size_t prevSlash = path.find_last_of("/\\", lastSlash - 1);
        dirName = (prevSlash != std::string::npos)
            ? path.substr(prevSlash + 1, lastSlash - prevSlash - 1) : path.substr(0, lastSlash);
    }

    {
        size_t tilePos = dirName.find("Tile_");
        if (tilePos != std::string::npos)
        {
            auto parseSignedInt = [](const std::string& s, size_t& pos, int& val) -> bool {
                val = 0;
                int sign = 1;
                if (pos < s.size() && s[pos] == '+') { ++pos; }
                else if (pos < s.size() && s[pos] == '-') { sign = -1; ++pos; }
                if (pos >= s.size() || !std::isdigit(s[pos])) return false;
                while (pos < s.size() && std::isdigit(s[pos]))
                {
                    val = val * 10 + (s[pos] - '0');
                    ++pos;
                }
                val *= sign;
                return true;
            };

            size_t p = tilePos + 5;
            if (parseSignedInt(dirName, p, x) && p < dirName.size() && dirName[p] == '_')
            {
                ++p;
                parseSignedInt(dirName, p, y);
            }
        }
    }

    // Parse LOD level from filename: ..._L<level>_...
    size_t lPos = filename.find("_L");
    if (lPos == std::string::npos)
    {
        level = 0;  // root tile (no LOD suffix)
        return true;
    }

    lPos += 2;
    if (lPos >= filename.size() || !std::isdigit(filename[lPos]))
        return false;

    level = 0;
    while (lPos < filename.size() && std::isdigit(filename[lPos]))
    {
        level = level * 10 + (filename[lPos] - '0');
        ++lPos;
    }

    return true;
}

// ---------------------------------------------------------------------------
// Sub-tile index parsing
// ---------------------------------------------------------------------------

std::string MetadataParser::ParseSubTileIndex(const std::string& path)
{
    size_t lastSlash = path.find_last_of("/\\");
    std::string filename = (lastSlash != std::string::npos)
        ? path.substr(lastSlash + 1) : path;

    size_t lPos = filename.find("_L");
    if (lPos == std::string::npos) return "";

    size_t p = lPos + 2;
    while (p < filename.size() && std::isdigit(filename[p])) ++p;
    if (p >= filename.size() || filename[p] != '_') return "";
    ++p;

    size_t end = filename.find(".osgb", p);
    if (end == std::string::npos) end = filename.size();

    std::string subIndex = filename.substr(p, end - p);
    size_t tPos = subIndex.find('t');
    if (tPos != std::string::npos)
        subIndex = subIndex.substr(0, tPos);

    return subIndex;
}

// ---------------------------------------------------------------------------
// ENU origin parsing
// ---------------------------------------------------------------------------

bool MetadataParser::ParseENUOrigin(const std::string& srs, double& lat, double& lon)
{
    if (srs.compare(0, 4, "ENU:") != 0 && srs.compare(0, 4, "enu:") != 0)
        return false;

    std::string coords = srs.substr(4);
    std::replace(coords.begin(), coords.end(), ',', ' ');

    std::istringstream iss(coords);
    double v[3] = { 0, 0, 0 };
    int count = 0;
    while (iss >> v[count] && count < 3) ++count;

    if (count >= 2)
    {
        lat = v[0];
        lon = v[1];
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// EPSG parsing
// ---------------------------------------------------------------------------

int MetadataParser::ParseEPSGCode(const std::string& srs)
{
    const char* p = srs.c_str();
    if (strncasecmp(p, "EPSG:", 5) == 0) p += 5;
    else if (strncasecmp(p, "epsg:", 5) == 0) p += 5;
    else return 0;

    int code = 0;
    while (*p && std::isdigit(*p))
    {
        code = code * 10 + (*p - '0');
        ++p;
    }
    return code;
}

// ---------------------------------------------------------------------------
// GK projection auto-configuration
// ---------------------------------------------------------------------------

bool MetadataParser::ConfigureGKProjection(CProjectionEngine& engine, int epsgCode)
{
    int zone = 0;

    if (epsgCode >= 4547 && epsgCode <= 4554)
        zone = epsgCode - 4547 + 39;
    else if (epsgCode >= 4525 && epsgCode <= 4534)
        zone = epsgCode - 4525 + 25;

    if (zone == 0) return false;

    double a = Geodetic::WGS84_SEMI_MAJOR_AXIS;
    double fInv = Geodetic::CGCS2000_INV_FLATTENING;
    double lambda0 = zone * 3.0;
    double falseE = ProjectionDefaults::FALSE_EASTING + (zone - 25) * 1000000.0;
    double falseN = ProjectionDefaults::FALSE_NORTHING;
    double k0 = ProjectionDefaults::SCALE_FACTOR;

    #ifdef _WIN32
    std::string prjPath = std::string(std::getenv("TEMP") ? std::getenv("TEMP") : ".") + "\\osgb_auto_gk.prj";
#else
    std::string prjPath = "/tmp/osgb_auto_gk.prj";
#endif
    FILE* fp = fopen(prjPath.c_str(), "w");
    if (!fp) return false;

    fprintf(fp, "PROJCS[\"CGCS2000 / 3-degree Gauss-Kruger zone %d\",\n", zone);
    fprintf(fp, "  GEOGCS[\"China Geodetic Coordinate System 2000\",\n");
    fprintf(fp, "    DATUM[\"China 2000\",\n");
    fprintf(fp, "      SPHEROID[\"CGCS2000\", %.1f, %.9f]],\n", a, fInv);
    fprintf(fp, "    PRIMEM[\"Greenwich\", 0.0],\n");
    fprintf(fp, "    UNIT[\"degree\", 0.0174532925199433]],\n");
    fprintf(fp, "  PROJECTION[\"Transverse_Mercator\"],\n");
    fprintf(fp, "  PARAMETER[\"Central_Meridian\", %.1f],\n", lambda0);
    fprintf(fp, "  PARAMETER[\"False_Easting\", %.1f],\n", falseE);
    fprintf(fp, "  PARAMETER[\"False_Northing\", %.1f],\n", falseN);
    fprintf(fp, "  PARAMETER[\"Scale_Factor\", %.1f],\n", k0);
    fprintf(fp, "  UNIT[\"metre\", 1.0]]\n");
    fclose(fp);

    bool ok = engine.LoadProjection(prjPath);
    remove(prjPath.c_str());
    return ok;
}