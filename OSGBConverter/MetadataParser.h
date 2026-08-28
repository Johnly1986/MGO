// Copyright Johnlyon
//
// MetadataParser — parse metadata.xml from DJI / ContextCapture OSGB datasets
//
// The metadata.xml contains:
//   - SRS: projection coordinate system (e.g. "EPSG:4547")
//   - SRSOrigin: origin coordinates (easting, northing, height)
//   - BoundingBox: dataset extent
//   - Tile inventory: list of tile paths and their bounding boxes
//
// Two projection modes are determined from the SRS:
//   - EPSG:454x  → RootOnly (GK projection, root transform only)
//   - Other/ENU  → PerTile  (per-tile true-error correction)
//

#pragma once

#include "macro.h"
#include "OSGBTileData.h"

#include <string>
#include <vector>
#include <functional>

class OSGB_CONVERTER_API MetadataParser
{
public:
    MetadataParser() = default;

    // Parse metadata.xml from file path.
    // Returns true on success, populates outMetadata.
    bool Parse(const std::string& xmlPath, OSGBMetadata& outMetadata);

    // Parse using a vendor handler for element interpretation.
    bool ParseWithHandler(const std::string& xmlPath, OSGBMetadata& outMetadata,
                          class IVendorHandler& handler);

    // Parse from in-memory string (for testing).
    bool ParseString(const std::string& xml, const std::string& baseDir,
                     OSGBMetadata& outMetadata);

    // --- Static helpers ---

    // Tile path LOD extraction
    // Extracts LOD level and (x,y) indices from paths like "Tile_L0_0_0/Tile_L0_0_0.osgb"
    static bool ParseTileLevelAndIndex(const std::string& path,
                                       int& level, int& x, int& y);

    // Parse ENU:lat,lon from SRS string. Returns true if SRS is ENU format.
    static bool ParseENUOrigin(const std::string& srs, double& lat, double& lon);

    // Parse EPSG code from string like "EPSG:4547". Returns 0 if not found.
    static int ParseEPSGCode(const std::string& srs);

    // Auto-configure CProjectionEngine for CGCS2000 GK zones (EPSG:4547-4554, 4525-4534).
    // Writes a temporary .prj file, loads it, then removes it.
    // Returns true if the EPSG code was recognized and configured.
    static bool ConfigureGKProjection(class CProjectionEngine& engine, int epsgCode);

    // Parse sub-tile index from ContextCapture filename.
    // e.g. "Tile_+005_+002_L23_00002200.osgb" → "00002200"
    // Strips trailing texture variant suffix (e.g. "t1", "t3").
    static std::string ParseSubTileIndex(const std::string& path);

    // Projection mode detection from SRS string
    static ProjectionMode DetectProjectionMode(const std::string& srs);

    // Parse origin coordinates from string like "500000,3400000,0"
    static bool ParseOrigin(const std::string& text, double& x, double& y, double& z);

private:
    // --- XML pull-parser helpers ---
    struct Element
    {
        std::string name;
        std::string text;
        std::vector<std::pair<std::string, std::string>> attrs;
    };

    // Simple recursive-descent XML parser (no external dependencies).
    // Handles the subset of XML needed for metadata.xml.
    bool ParseXML(const std::string& xml, std::function<void(const Element&)> onElement);

    private:
    // --- Raw XML extraction helpers ---
    void ExtractBBoxFromXML(const std::string& xml,
                            double bboxMin[3], double bboxMax[3]);
    void ExtractTilesFromXML(const std::string& xml,
                             std::vector<std::string>& tiles);

    std::string m_baseDir;
    OSGBMetadata* m_metadata = nullptr;
};