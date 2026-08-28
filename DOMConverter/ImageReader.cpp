#include "ImageReader.h"

#include <tiffio.h>
#include <cstring>
#include <iostream>
#include <algorithm>

// TIFF tag constants (not in all tiffio.h versions)
#ifndef TIFFTAG_GEOKEYDIRECTORY
#define TIFFTAG_GEOKEYDIRECTORY 34735
#endif
#ifndef TIFFTAG_GEOPIXELSCALE
#define TIFFTAG_GEOPIXELSCALE   33550
#endif
#ifndef TIFFTAG_GEOTIEPOINT
#define TIFFTAG_GEOTIEPOINT     33922
#endif
#ifndef TIFFTAG_GEOASCIIPARAMS
#define TIFFTAG_GEOASCIIPARAMS  34737
#endif
#ifndef TIFFTAG_GDAL_NODATA
#define TIFFTAG_GDAL_NODATA     42113
#endif

// GeoKey IDs
enum {
    GTModelTypeGeoKey       = 1024,
    GTRasterTypeGeoKey      = 1025,
    GeographicTypeGeoKey    = 2048,
    GeogGeodeticDatumGeoKey = 2050,
    GeogEllipsoidGeoKey     = 2056,
    GeogAngularUnitsGeoKey  = 2054,
    ProjectedCSTypeGeoKey   = 3072,
    ProjLinearUnitsGeoKey   = 3076,
};

ImageReader::ImageReader() : m_tif(nullptr) {}
ImageReader::~ImageReader() { if (m_tif) TIFFClose(static_cast<TIFF*>(m_tif)); }

bool ImageReader::Open(const std::string& path)
{
    TIFF* tif = TIFFOpen(path.c_str(), "r");
    if (!tif) {
        std::cerr << "[ImageReader] Cannot open: " << path << std::endl;
        return false;
    }
    m_tif = tif;

    TIFFGetField(tif, TIFFTAG_IMAGEWIDTH,  &m_width);
    TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &m_height);
    TIFFGetField(tif, TIFFTAG_BITSPERSAMPLE, &m_bitsPerSample);
    TIFFGetField(tif, TIFFTAG_SAMPLEFORMAT, &m_sampleFormat);
    TIFFGetField(tif, TIFFTAG_SAMPLESPERPIXEL, &m_channels);

    uint16_t planarConfig = 1;
    TIFFGetField(tif, TIFFTAG_PLANARCONFIG, &planarConfig);
    m_planarConfig = planarConfig;

    ReadModelTransform();
    ReadGeoAsciiParams();

    // GDAL noData tag (ASCII) — e.g. "0" or "0 0 0" for RGB
    char* noDataStr = nullptr;
    if (TIFFGetField(tif, TIFFTAG_GDAL_NODATA, &noDataStr) && noDataStr) {
        int r = 0, g = 0, b = 0;
        int n = sscanf(noDataStr, "%d %d %d", &r, &g, &b);
        if (n >= 1) {
            m_hasNoData = true;
            m_noDataR = static_cast<uint8_t>(r);
            m_noDataG = static_cast<uint8_t>(n >= 2 ? g : r);
            m_noDataB = static_cast<uint8_t>(n >= 3 ? b : r);
        }
    }

    if (m_hasNoData) {
        std::cout << "[ImageReader] noData=("
                  << (int)m_noDataR << "," << (int)m_noDataG << "," << (int)m_noDataB << ")"
                  << std::endl;
    }

    std::cout << "[ImageReader] " << m_width << "x" << m_height
              << " x" << m_channels << " channels"
              << " bps=" << m_bitsPerSample << std::endl;
    return true;
}

bool ImageReader::ReadRGB(std::vector<uint8_t>& outPixels)
{
    TIFF* tif = static_cast<TIFF*>(m_tif);
    if (!tif) return false;

    const size_t rowBytes = static_cast<size_t>(m_width) * m_channels;
    outPixels.resize(static_cast<size_t>(m_width) * m_height * m_channels);

    // TIFF stores rows top-down by default. Read scanlines directly.
    for (int row = 0; row < m_height; ++row) {
        uint8_t* dst = outPixels.data() + static_cast<size_t>(row) * rowBytes;
        if (TIFFReadScanline(tif, dst, row, 0) < 0) {
            std::cerr << "[ImageReader] Error reading row " << row << std::endl;
            return false;
        }
    }

    // Normalize to north-up, east-west convention (mirrors GeoTiffReader fix).
    // The downstream pipeline assumes row 0 = north, col 0 = west.
    if (m_southUp) {
        std::vector<uint8_t> flipped(outPixels.size());
        for (int r = 0; r < m_height; ++r) {
            int r2 = m_height - 1 - r;
            memcpy(flipped.data() + r2 * rowBytes,
                   outPixels.data() + r * rowBytes, rowBytes);
        }
        outPixels.swap(flipped);
    }
    if (m_westUp) {
        std::vector<uint8_t> flipped(outPixels.size());
        const int bpp = m_channels;
        for (int r = 0; r < m_height; ++r) {
            for (int c = 0; c < m_width; ++c) {
                int c2 = m_width - 1 - c;
                memcpy(flipped.data() + r * rowBytes + c2 * bpp,
                       outPixels.data() + r * rowBytes + c * bpp, bpp);
            }
        }
        outPixels.swap(flipped);
    }
    return true;
}

bool ImageReader::ReadModelTransform()
{
    TIFF* tif = static_cast<TIFF*>(m_tif);
    if (!tif) return false;

    // ModelPixelScale
    uint16_t count;
    double* scaleData = nullptr;
    if (TIFFGetField(tif, TIFFTAG_GEOPIXELSCALE, &count, &scaleData) && count >= 2) {
        m_scaleX = scaleData[0];
        m_scaleY = scaleData[1];
        m_scaleZ = (count >= 3) ? scaleData[2] : 0.0;
        m_hasScale = true;
    }

    // ModelTiepoint — read raster coords (I,J) + model coords (X,Y,Z)
    double* tpData = nullptr;
    if (TIFFGetField(tif, TIFFTAG_GEOTIEPOINT, &count, &tpData) && count >= 6) {
        // tpData = {I, J, K, X, Y, Z, ...} for multiple tiepoints
        m_tiePointCol  = tpData[0];   // raster column (I)
        m_tiePointRow  = tpData[1];   // raster row (J)
        m_tiePointX    = tpData[3];   // model X (Easting)
        m_tiePointY    = tpData[4];   // model Y (Northing)
        m_tiePointZ    = tpData[5];
        m_hasTiePoint  = true;
    }

    // Detect orientation.
    //
    // ModelPixelScale values are pixel dimensions in model units — always
    // POSITIVE per the GeoTIFF spec.  The old heuristic  m_southUp = (ScaleY > 0)
    // was therefore wrong for every standard north-up TIF: it always evaluated
    // to true, causing ReadRGB() to flip the pixel rows while the origin
    // (TiePointY) stayed at the original tiepoint, producing a row offset in
    // every output tile.
    //
    // Correct approach:
    //   1. Check TIFFTAG_GEOTRANSMATRIX (tag 34264) — a full 16-double 4×4
    //      affine matrix where element [5] (= row 1, col 1 = pixelHeight)
    //      carries the sign: negative = north-up, positive = south-up.
    //   2. If no matrix tag, default to north-up (the standard for DOM
    //      orthophotos; south-up rasters are virtually non-existent in this
    //      domain).
    if (m_hasScale) {
        m_southUp = false;   // default: north-up
        m_westUp  = false;   // default: col 0 = west

        // TIFFTAG_GEOTRANSMATRIX = 34264 (16 doubles, 4×4 row-major)
        uint16_t mtCount = 0;
        double*  mtData  = nullptr;
        if (TIFFGetField(tif, 34264, &mtCount, &mtData) && mtCount >= 16) {
            // matrix[5] = row 1 col 1 = pixel height in model units
            m_southUp = (mtData[5] > 0.0);
            // matrix[0] = row 0 col 0 = pixel width
            m_westUp  = (mtData[0] < 0.0);
        }
    }

    // GeoKeyDirectory
    uint16_t* keyDir = nullptr;
    if (TIFFGetField(tif, TIFFTAG_GEOKEYDIRECTORY, &count, &keyDir) && count >= 4) {
        ParseGeoKeys(keyDir, static_cast<size_t>(count));
    }

    std::cout << "[ImageReader] Scale=(" << m_scaleX << ", " << m_scaleY
              << ") TiePoint=(" << m_tiePointX << ", " << m_tiePointY
              << ") EPSG=" << m_epsg << std::endl;
    return true;
}

bool ImageReader::ParseGeoKeys(const uint16_t* keyDir, size_t count)
{
    if (count < 4) return false;
    size_t numKeys = keyDir[3];
    const uint16_t* keys = keyDir + 4;

    for (size_t i = 0; i < numKeys && (i + 1) * 4 <= count; ++i) {
        uint16_t keyId = keys[i * 4];
        uint16_t value = keys[i * 4 + 3];

        switch (keyId) {
            case GTModelTypeGeoKey:
                m_isGeographic = (value == 2);
                break;
            case GTRasterTypeGeoKey:
                m_rasterType = value;  // 1=RasterPixelIsArea, 2=RasterPixelIsPoint
                break;
            case GeographicTypeGeoKey:
                if (value == 4326 || value == 4269 || value == 4490)
                    m_isGeographic = true;
                break;
            case ProjectedCSTypeGeoKey:
                m_epsg = value;
                break;
            default:
                break;
        }
    }
    return true;
}

bool ImageReader::ReadGeoAsciiParams()
{
    TIFF* tif = static_cast<TIFF*>(m_tif);
    if (!tif) return false;

    // Use raw tag access pattern for unknown ASCII tags to avoid segfault.
    // libtiff requires &count, &data for unknown tags; direct char** access
    // crashes on libtiff 4.5.x when the tag type isn't registered.
    uint16_t count = 0;
    void* data = nullptr;
    if (TIFFGetField(tif, TIFFTAG_GEOASCIIPARAMS, &count, &data) && data && count > 0) {
        m_wkt = std::string(static_cast<const char*>(data), count);
        // Trim trailing '|' and null
        while (!m_wkt.empty() && (m_wkt.back() == '|' || m_wkt.back() == '\0'))
            m_wkt.pop_back();
        return true;
    }
    return false;
}
