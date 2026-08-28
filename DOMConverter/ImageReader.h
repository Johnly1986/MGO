#pragma once

#include <string>
#include <vector>
#include <cstdint>

// ImageReader — read multi-band (RGB) GeoTIFF images using libtiff
//
// Reads RGB pixels and geospatial metadata (ModelPixelScale, ModelTiepoint,
// GeoKeyDirectory, GeoAsciiParams) from a GeoTIFF file.
// Unlike GeoTiffReader (single-band float32 elevation only), this supports
// multi-band uint8 RGB images typical of DOM orthophotos.

class ImageReader
{
public:
    ImageReader();
    ~ImageReader();

    bool Open(const std::string& path);

    // Read RGB pixels into a flat array (row-major, top-down, R/G/B interleaved).
    // Returns (width * height * 3) bytes.
    bool ReadRGB(std::vector<uint8_t>& outPixels);

    int Width()  const { return m_width; }
    int Height() const { return m_height; }
    int Channels() const { return m_channels; }

    // Geospatial metadata
    double ScaleX()  const { return m_scaleX; }
    double ScaleY()  const { return m_scaleY; }
    double TiePointX() const { return m_tiePointX; }
    double TiePointY() const { return m_tiePointY; }
    const std::string& ProjectionWKT() const { return m_wkt; }
    int EPSG() const { return m_epsg; }
    bool IsGeographic() const { return m_isGeographic; }
    bool IsSouthUp()  const { return m_southUp; }
    bool IsWestUp()   const { return m_westUp; }

    // noData support
    bool HasNoData() const { return m_hasNoData; }
    uint8_t NoDataR() const { return m_noDataR; }
    uint8_t NoDataG() const { return m_noDataG; }
    uint8_t NoDataB() const { return m_noDataB; }

private:
    bool ReadModelTransform();
    bool ParseGeoKeys(const uint16_t* keyDir, size_t count);
    bool ReadGeoAsciiParams();

    void*  m_tif;
    int    m_width = 0, m_height = 0;
    int    m_bitsPerSample = 0;
    int    m_sampleFormat = 0;
    int    m_channels = 0;
    int    m_planarConfig = 0;

    double m_scaleX = 0, m_scaleY = 0, m_scaleZ = 0;
    double m_tiePointX = 0, m_tiePointY = 0, m_tiePointZ = 0;
    double m_tiePointCol = 0, m_tiePointRow = 0;  // raster coords of tie point
    int    m_rasterType = 0;                       // GTRasterTypeGeoKey: 1=Area, 2=Point
    bool   m_southUp = false;                      // true if row 0 = south
    bool   m_westUp  = false;                      // true if col 0 = west
    bool   m_hasScale = false, m_hasTiePoint = false;

    int    m_epsg = 0;
    bool   m_isGeographic = false;

    // noData
    bool    m_hasNoData = false;
    uint8_t m_noDataR = 0, m_noDataG = 0, m_noDataB = 0;

    std::string m_wkt;
};
