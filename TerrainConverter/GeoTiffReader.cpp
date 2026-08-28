#include "GeoTiffReader.h"
#include <gdal.h>
#include <ogr_srs_api.h>
#include <tiffio.h>
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <vector>

GeoTiffReader::GeoTiffReader()
    : m_dataset(nullptr)
    , m_width(0), m_height(0)
    , m_bitsPerSample(0), m_sampleFormat(0)
    , m_hasNoData(false), m_noDataValue(-9999.0f)
    , m_hasTransform(false)
    , m_epsg(0)
    , m_isGeographic(false)
    , m_linearUnits(1.0)
{
    static bool gdalRegistered = false;
    if (!gdalRegistered)
    {
        GDALAllRegister();
        gdalRegistered = true;
    }
}

GeoTiffReader::~GeoTiffReader()
{
    Close();
}

bool GeoTiffReader::Open(const std::string& path)
{
    GDALDatasetH ds = GDALOpen(path.c_str(), GA_ReadOnly);
    if (!ds)
    {
        std::cerr << "[GeoTiffReader] Cannot open: " << path << std::endl;
        return false;
    }
    m_dataset = ds;

    m_width  = GDALGetRasterXSize(ds);
    m_height = GDALGetRasterYSize(ds);

    if (GDALGetRasterCount(ds) < 1)
    {
        std::cerr << "[GeoTiffReader] No raster bands found" << std::endl;
        Close();
        return false;
    }

    GDALRasterBandH band = GDALGetRasterBand(ds, 1);
    GDALDataType dt = GDALGetRasterDataType(band);
    m_bitsPerSample = GDALGetDataTypeSize(dt);

    // Map GDAL data type to TIFF SAMPLEFORMAT constants
    switch (dt)
    {
        case GDT_Byte:
        case GDT_UInt16:
        case GDT_UInt32:
        case GDT_UInt64:
            m_sampleFormat = 1;  // SAMPLEFORMAT_UINT
            break;
        case GDT_Int8:
        case GDT_Int16:
        case GDT_Int32:
        case GDT_Int64:
            m_sampleFormat = 2;  // SAMPLEFORMAT_INT
            break;
        case GDT_Float32:
        case GDT_Float64:
            m_sampleFormat = 3;  // SAMPLEFORMAT_IEEEFP
            break;
        default:
            m_sampleFormat = 1;
            break;
    }

    // NoData value
    int hasNoData = 0;
    double nd = GDALGetRasterNoDataValue(band, &hasNoData);
    if (hasNoData)
    {
        m_hasNoData = true;
        m_noDataValue = static_cast<float>(nd);
    }

    // GeoTransform — GDALGetGeoTransform returns CE_None on success
    if (GDALGetGeoTransform(ds, m_transform) == CE_None)
    {
        m_hasTransform = true;
    }
    else
    {
        // Fallback identity (1px = 1m, north-up, origin at 0,0)
        m_transform[0] = 0; m_transform[1] =  1; m_transform[2] = 0;
        m_transform[3] = 0; m_transform[4] =  0; m_transform[5] = -1;
        m_hasTransform = true;
    }

    // Projection WKT
    const char* wkt = GDALGetProjectionRef(ds);
    if (wkt && wkt[0])
    {
        m_wkt = wkt;
    }

    // Determine geographic/projected and EPSG code from spatial reference
    if (!m_wkt.empty())
    {
        OGRSpatialReferenceH sr = OSRNewSpatialReference(nullptr);
        if (sr)
        {
            std::vector<char> wktBuf(m_wkt.begin(), m_wkt.end());
            wktBuf.push_back('\0');
            char* pszWkt = wktBuf.data();

            if (OSRImportFromWkt(sr, &pszWkt) == OGRERR_NONE)
            {
                m_isGeographic = (OSRIsGeographic(sr) != 0);
                const char* code = OSRGetAuthorityCode(sr, nullptr);
                if (code)
                {
                    m_epsg = static_cast<int>(std::strtol(code, nullptr, 10));
                }
                double u = OSRGetLinearUnits(sr, nullptr);
                if (u > 0) m_linearUnits = u;
            }
            OSRDestroySpatialReference(sr);
        }
    }

    // Fallback: if GDAL's projection ref is LOCAL_CS or empty,
    // read GeographicTypeGeoKey directly from the raw GeoTIFF tag.
    // Some files have minimal geokeys (no GTModelTypeGeoKey) that
    // GDAL cannot interpret into a proper CRS WKT.
    if (m_wkt.empty() || m_wkt.compare(0, 8, "LOCAL_CS") == 0)
    {
        TIFF* tif = static_cast<TIFF*>(GDALGetInternalHandle(ds, "TIFF"));
        if (tif)
        {
            uint16_t* keyDir = nullptr;
            uint16_t  kdCount = 0;
            if (TIFFGetField(tif, 34735, &kdCount, &keyDir) && kdCount >= 4)
            {
                size_t numKeys = keyDir[3];
                if (numKeys > 0 && static_cast<size_t>(kdCount) >= numKeys * 4 + 4)
                {
                    for (size_t i = 0; i < numKeys; ++i)
                    {
                        const uint16_t* entry = keyDir + 4 + i * 4;
                        uint16_t keyId = entry[0];
                        uint16_t value = entry[3];
                        // GeographicTypeGeoKey=4326/4269/4258
                        if (keyId == 2048 && (value == 4326 || value == 4269 || value == 4258))
                        {
                            m_isGeographic = true;
                            m_epsg = value;
                            break;
                        }
                    }
                }
            }
        }
    }

    return true;
}

void GeoTiffReader::Close()
{
    if (m_dataset)
    {
        GDALClose(static_cast<GDALDatasetH>(m_dataset));
        m_dataset = nullptr;
    }
}

void GeoTiffReader::OverrideOrigin(double originX, double originY)
{
    m_transform[0] = originX;
    m_transform[3] = originY;
    m_hasTransform = true;
}

bool GeoTiffReader::ReadModelTransform()
{
    // With GDAL, the GeoTransform is already read in Open().
    // This function is kept for API compatibility.
    return m_hasTransform;
}

bool GeoTiffReader::ReadElevationGrid(HeightmapGrid& outGrid)
{
    GDALDatasetH ds = static_cast<GDALDatasetH>(m_dataset);
    if (!ds) return false;

    int bandCount = GDALGetRasterCount(ds);
    if (bandCount < 1)
    {
        std::cerr << "[GeoTiffReader] No raster bands found" << std::endl;
        return false;
    }
    if (bandCount > 1)
    {
        std::cerr << "[GeoTiffReader] Only single-band rasters supported (got "
                  << bandCount << ")" << std::endl;
        return false;
    }

    const int W = m_width;
    const int H = m_height;
    outGrid.width = W;
    outGrid.height = H;
    outGrid.heights.resize(static_cast<size_t>(W) * H);
    outGrid.hasNoData = m_hasNoData;
    outGrid.noDataValue = m_noDataValue;

    // Read all pixels via GDAL — handles type conversion to float32 automatically
    GDALRasterBandH band = GDALGetRasterBand(ds, 1);
    std::vector<float> srcHeights(static_cast<size_t>(W) * H);
    CPLErr err = GDALRasterIO(band, GF_Read, 0, 0, W, H,
                              srcHeights.data(), W, H, GDT_Float32, 0, 0);
    if (err != CE_None)
    {
        std::cerr << "[GeoTiffReader] GDALRasterIO failed" << std::endl;
        return false;
    }

    // Convert raw pixel values to meters using the TIF's linear unit factor.
    // GDAL's GDT_Float32 RasterIO returns raw values without unit conversion.
    // For TIFs in feet: multiply by ~0.3048; for meters: multiply by 1.0 (no-op).
    if (std::abs(m_linearUnits - 1.0) > 1e-9)
    {
        for (auto& h : srcHeights) h *= static_cast<float>(m_linearUnits);
    }

    // Compute grid bounds from GeoTransform
    if (!m_hasTransform)
    {
        std::cerr << "[GeoTiffReader] No transform — cannot compute bounds" << std::endl;
        return false;
    }

    {
        const double* T = m_transform;

        outGrid.dx = std::abs(T[1]);
        outGrid.dy = std::abs(T[5]);
        if (outGrid.dx < 1e-30 || outGrid.dy < 1e-30)
        {
            std::cerr << "[GeoTiffReader] Degenerate pixel scale: dx="
                      << outGrid.dx << " dy=" << outGrid.dy << std::endl;
            return false;
        }

        const double rotTol = 1e-10 * std::max(outGrid.dx, outGrid.dy);
        if (std::abs(T[2]) > rotTol || std::abs(T[4]) > rotTol)
        {
            std::cerr << "[GeoTiffReader] Warning: raster has rotation/shear "
                      << "(rowRot=" << T[2] << " colRot=" << T[4] << "). "
                      << "Output grid is axis-aligned; rotated pixels may not "
                      << "cover all output cells." << std::endl;
        }

        // Copy full GeoTransform to output
        for (int i = 0; i < 6; ++i) outGrid.GeoTransform[i] = T[i];

        // Four corner pixel-center positions
        double xNW = T[0];
        double yNW = T[3];
        double xNE = T[0] + (W-1)*T[1];
        double yNE = T[3] + (W-1)*T[4];
        double xSW = T[0] + (H-1)*T[2];
        double ySW = T[3] + (H-1)*T[5];
        double xSE = T[0] + (W-1)*T[1] + (H-1)*T[2];
        double ySE = T[3] + (W-1)*T[4] + (H-1)*T[5];

        outGrid.minEasting  = std::min({xNW, xNE, xSW, xSE});
        outGrid.maxEasting  = std::max({xNW, xNE, xSW, xSE});
        outGrid.minNorthing = std::min({yNW, yNE, ySW, ySE});
        outGrid.maxNorthing = std::max({yNW, yNE, ySW, ySE});
    }

    // Resample: output grid → geographic → source pixel via inverse GeoTransform
    //
    // Output pixel (outRow, outCol) lies at:
    //   X_geo = minEasting + outCol * dx
    //   Y_geo = maxNorthing - outRow * dy
    //
    // Inverse 3×3 homogeneous affine transform:
    //   det = T[1]*T[5] - T[2]*T[4]
    //   srcCol = (T[5]*(X_geo - T[0]) - T[2]*(Y_geo - T[3])) / det
    //   srcRow = (T[1]*(Y_geo - T[3]) - T[4]*(X_geo - T[0])) / det
    {
        const double* T = m_transform;
        const double det = T[1] * T[5] - T[2] * T[4];
        if (std::abs(det) < 1e-30)
        {
            std::cerr << "[GeoTiffReader] Singular GeoTransform determinant" << std::endl;
            return false;
        }
        const double invDet = 1.0 / det;

        for (int outRow = 0; outRow < H; ++outRow)
        {
            double Y_geo = outGrid.maxNorthing - outRow * outGrid.dy;
            for (int outCol = 0; outCol < W; ++outCol)
            {
                double X_geo = outGrid.minEasting + outCol * outGrid.dx;

                double dX = X_geo - T[0];
                double dY = Y_geo - T[3];
                double srcCol = (T[5] * dX - T[2] * dY) * invDet;
                double srcRow = (T[1] * dY - T[4] * dX) * invDet;

                int sc = static_cast<int>(std::round(srcCol));
                int sr = static_cast<int>(std::round(srcRow));

                if (sc >= 0 && sc < W && sr >= 0 && sr < H)
                    outGrid.heights[static_cast<size_t>(outRow) * W + outCol] =
                        srcHeights[static_cast<size_t>(sr) * W + sc];
                else
                    outGrid.heights[static_cast<size_t>(outRow) * W + outCol] =
                        outGrid.noDataValue;
            }
        }
    }

    outGrid.isGeographic = m_isGeographic;
    outGrid.epsg = m_epsg;
    outGrid.projectionWKT = m_wkt;

    // Vertical scale: cached from Open() OGRSpatialReference parse
    outGrid.verticalScale = m_linearUnits;

    return true;
}
