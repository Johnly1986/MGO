#pragma once

#include "macro.h"
#include <string>
#include <vector>
#include <cstdint>
#include <proj/proj.h>

// ImageTilerOptions — CLI-configurable options
struct ImageTilerOptions
{
    std::string inputTif;
    std::string outputDir;
    std::string prjFile;
    bool   verbose = false;

    // Override TIF-embedded origin
    bool   hasOrigin = false;
    double originX = 0.0, originY = 0.0, originZ = 0.0;

    int    maxZoom = -1;   // -1 = auto
    int    minZoom = -1;   // -1 = auto

    // Output format
    int    pngCompression = 9;       // 0–9, 9=max (PNG only)
    bool   enableKtx2 = false;       // Use KTX2/BasisU GPU-compressed textures

    // noData override (R,G,B values that should be transparent)
    bool    hasNoDataOverride = false;
    uint8_t noDataR = 0, noDataG = 0, noDataB = 0;
};

// ImageTiler — main orchestration (mirrors TerrainConverter pattern)
//
// Pipeline:
//   1. Read RGB GeoTIFF via ImageReader
//   2. Setup PROJ transform (WGS84 → source CRS)
//   3. Compute geographic bounds
//   4. Generate 256×256 RGBA PNG tiles at each zoom level
//   5. Write layer.json + tilemapresource.xml

class IMAGE_TILER_API ImageTiler
{
public:
    ImageTiler();
    ~ImageTiler();

    bool Convert(const ImageTilerOptions& opts);

private:
    // Web Mercator tiling math (Cesium WebMercatorTilingScheme default)
    static void ComputeTileBounds(int level, int x, int y,
                                   double& west, double& south,
                                   double& east, double& north);
    static int ComputeMaxLevel(double pixelSizeDeg, int tileSize = 256);
    static int ComputeMinLevel(double geoW, double geoH);

    // Bilinear sample from source image at fractional pixel coords.
    // Clamped to image bounds. Returns RGBA with alpha=0 for out-of-bounds.
    void SamplePixel(double sx, double sy, uint8_t out[4]) const;

    // Render a single tile using affine approximation (fast path).
    // Transforms 3 tile corners via PROJ, computes source-pixel affine,
    // then bilinear-samples all 256×256 pixels.
    bool RenderTile(int level, int tx, int tyCesium,
                    std::vector<uint8_t>& outRgba) const;

    // Write metadata
    bool WriteLayerJson(const std::string& outputDir,
                        double west, double south, double east, double north,
                        int minZoom, int maxZoom, bool ktx2 = false) const;
    bool WriteTilemapXml(const std::string& outputDir,
                         double west, double south, double east, double north,
                         int minZoom, int maxZoom,
                         const std::vector<int>& tileCounts,
                         bool ktx2 = false) const;

    // Source image data
    std::vector<uint8_t> m_srcPixels;
    int m_srcW = 0, m_srcH = 0;
    int m_srcChannels = 0;

    // noData
    bool    m_hasNoData = false;
    uint8_t m_noDataR = 0, m_noDataG = 0, m_noDataB = 0;

    // Geospatial
    double m_originX = 0, m_originY = 0;
    double m_resolution = 0;       // meters/pixel (abs of ScaleX)

    // Geographic bounds
    double m_geoWest = 0, m_geoSouth = 0, m_geoEast = 0, m_geoNorth = 0;

    // PROJ context for WGS84 → source CRS
    PJ_CONTEXT* m_projCtx = nullptr;
    PJ* m_projTransform = nullptr;  // EPSG:4326 → source CRS, lon,lat order
};
