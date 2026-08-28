// Copyright Johnlyon
//
// GeoJSONConverter - GeoJSON projection conversion
//
// Reads a GeoJSON document (RFC 7946), transforms every geometry
// coordinate from a source CRS to a target CRS (via CRSTransformer /
// PROJ), and writes the document back with all non-coordinate content
// (properties, ids, foreign members) preserved.
//
// Source CRS resolution order:
//   1. options.sourceCRS (EPSG:xxxx / ENU:lat,lon[,h] / WKT / +proj=...)
//   2. legacy GeoJSON "crs" member (urn:ogc:def:crs:EPSG::xxxx or
//      "EPSG:xxxx" name form)
//   3. EPSG:4326 (RFC 7946 default)
//
// Target CRS: options.targetCRS, default EPSG:4326. Axis convention is
// "visualization" order on both ends: x = lon/easting, y = lat/northing.
//

#pragma once

#include "macro.h"

#include <string>

class CRSTransformer;

struct GeoJSONConverterOptions
{
    std::string inputFile;
    std::string outputFile;

    // "" = auto-detect (crs member, else EPSG:4326)
    std::string sourceCRS;
    std::string targetCRS = "EPSG:4326";

    bool pretty = false;   // pretty-print output
    bool verbose = false;
};

class GEOJSON_CONVERTER_API GeoJSONConverter
{
public:
    GeoJSONConverter();
    ~GeoJSONConverter();

    // Main entry point: convert a GeoJSON file's coordinates between CRS.
    bool Convert(const GeoJSONConverterOptions& opts);

    const std::string& GetLastError() const { return m_lastError; }

private:
    std::string m_lastError;
};
