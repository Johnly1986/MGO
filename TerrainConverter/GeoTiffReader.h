#pragma once

#include "macro.h"
#include "HeightmapGrid.h"
#include <string>

// GeoTiffReader — read GeoTIFF elevation rasters using GDAL
//
// Uses GDAL's C API for all metadata and data access:
//   - GDALGetGeoTransform() for the unified 6-element GeoTransform
//   - GDALGetProjectionRef() for CRS WKT
//   - GDALRasterIO() for type-converting data reads
//
// Supports single-band rasters of any GDAL-supported format
// (GeoTIFF, BigTIFF, IMG, HFA, etc.).
//
class TERRAIN_CONVERTER_API GeoTiffReader
{
public:
    GeoTiffReader();
    ~GeoTiffReader();

    bool Open(const std::string& path);
    void Close();

    // Override the GeoTransform origin before ReadElevationGrid.
    // Replaces GT[0] (originX) and GT[3] (originY) so the resampling
    // step maps output grid positions using the corrected origin.
    void OverrideOrigin(double originX, double originY);

    // Read elevation grid. GDAL handles all type conversion internally;
    // resampling to the axis-aligned north-up output grid is done here.
    bool ReadElevationGrid(HeightmapGrid& outGrid);

    // Accessors after Open()
    int    GetWidth()  const { return m_width; }
    int    GetHeight() const { return m_height; }
    int    GetBitsPerSample() const { return m_bitsPerSample; }
    int    GetSampleFormat()  const { return m_sampleFormat; }
    bool   IsGeographic()     const { return m_isGeographic; }
    // GDAL-style GeoTransform: [originX, pixelWidth, rowRot, originY, colRot, pixelHeight]
    const double* GetTransform() const { return m_transform; }
    bool HasTransform() const { return m_hasTransform; }
    const std::string& GetProjectionWKT() const { return m_wkt; }
    int    GetEPSG() const { return m_epsg; }

private:
    bool ReadModelTransform();

    void*  m_dataset;          // GDALDatasetH (opaque)
    int    m_width;
    int    m_height;
    int    m_bitsPerSample;
    int    m_sampleFormat;     // SAMPLEFORMAT_UINT=1, INT=2, IEEEFP=3

    bool   m_hasNoData;
    float  m_noDataValue;

    // Unified GDAL GeoTransform: 6 doubles
    //   [0]=originX, [1]=pixelWidth,  [2]=rowRotation
    //   [3]=originY, [4]=colRotation, [5]=pixelHeight
    // pixelHeight sign: negative=north-up, positive=south-up
    double m_transform[6];
    bool   m_hasTransform;

    int    m_epsg;
    bool   m_isGeographic;
    double m_linearUnits;      // cached during Open(), used for verticalScale

    std::string m_wkt;
};
