#define _USE_MATH_DEFINES
#include <cmath>

#include "ImageTiler.h"
#include "ImageReader.h"
#include "PngWriter.h"
#include "../MeshProjectionErrorCorrector/PROJUtils.h"

#ifdef MGO_WITH_KTX2
#include <basisu/encoder/basisu_comp.h>
#endif

#ifdef _WIN32
#include <direct.h>
#define mkdir_p(p) _mkdir(p)
#else
#include <sys/stat.h>
#define mkdir_p(p) mkdir(p, 0755)
#endif
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <filesystem>
#include <system_error>

namespace fs = std::filesystem;

// ============================================================================
// Web Mercator tiling math (Cesium WebMercatorTilingScheme, the default)
// ============================================================================

void ImageTiler::ComputeTileBounds(int level, int x, int y,
                                    double& west, double& south,
                                    double& east, double& north)
{
    int tiles = 1 << level;          // square grid: N×N
    double tileW = 360.0 / tiles;    // longitude: linear
    west = -180.0 + x * tileW;
    east = west + tileW;
    // Web Mercator TMS Y: y=0 at ~85.06°S, y=N-1 at ~85.06°N.
    // projectedY = R * ln(tan(π/4 + φ/2)),  y_tms = (1+projectedY/(Rπ)) * N/2
    // Inverse: projectedY = (2*y/N - 1)*Rπ → φ = 2*atan(e^{projectedY/R}) - π/2
    auto yToLat = [](double yNorm) {
        double merc = (2.0 * yNorm - 1.0) * M_PI;   // = projectedY / R
        return 2.0 * std::atan(std::exp(merc)) * 180.0 / M_PI - 90.0;
    };
    double invN = 1.0 / static_cast<double>(tiles);
    north = yToLat((y + 1.0) * invN);   // top edge = y+1 (nearer +1)
    south = yToLat(y * invN);            // bottom edge = y (nearer 0)
}

int ImageTiler::ComputeMaxLevel(double pixelSizeDeg, int tileSize)
{
    // Web Mercator: 2πR / (tileSize * 2^L) = resolution (meters/pixel)
    // → 2^L = 2πR / (tileSize * resolution)
    if (pixelSizeDeg <= 0) return 18;
    double resolutionM = pixelSizeDeg * 111320.0;  // meters/pixel at equator
    double levelF = std::log2(2.0 * M_PI * 6378137.0 / (tileSize * resolutionM));
    int level = static_cast<int>(std::floor(levelF));
    if (level < 0) level = 0;
    if (level > 22) level = 22;
    return level;
}

int ImageTiler::ComputeMinLevel(double geoW, double geoH)
{
    double minSpan = std::min(geoW, geoH);
    for (int L = 0; L < 23; ++L) {
        double pixelW = (360.0 / (1 << L)) / 256.0;   // mercator: N tiles, not 2N
        if (pixelW <= minSpan)
            return std::max(0, L - 1);
    }
    return 0;
}

// ============================================================================
// Lifecycle
// ============================================================================

ImageTiler::ImageTiler() {}
ImageTiler::~ImageTiler()
{
    if (m_projTransform) proj_destroy(m_projTransform);
    if (m_projCtx) proj_context_destroy(m_projCtx);
}

// ============================================================================
// SamplePixel — bilinear interpolation from source image
// ============================================================================

void ImageTiler::SamplePixel(double sx, double sy, uint8_t out[4]) const
{
    // Determine validity: coordinates must be within source image bounds
    bool valid = (sx >= 0 && sy >= 0 && sx < m_srcW - 1 && sy < m_srcH - 1);

    // Clamp to safe range for bilinear interpolation (compute RGB even for
    // invalid pixels — matches Python behavior of computing colors then
    // setting alpha separately).
    double cx = std::max(0.0, std::min(sx, static_cast<double>(m_srcW - 2)));
    double cy = std::max(0.0, std::min(sy, static_cast<double>(m_srcH - 2)));

    int sx0 = static_cast<int>(std::floor(cx));
    int sy0 = static_cast<int>(std::floor(cy));
    double fx = cx - sx0;
    double fy = cy - sy0;

    int sx1 = std::min(sx0 + 1, m_srcW - 1);
    int sy1 = std::min(sy0 + 1, m_srcH - 1);

    const int stride = m_srcW * m_srcChannels;

    for (int c = 0; c < 3; ++c) {
        double v00 = m_srcPixels[sy0 * stride + sx0 * m_srcChannels + c];
        double v10 = m_srcPixels[sy0 * stride + sx1 * m_srcChannels + c];
        double v01 = m_srcPixels[sy1 * stride + sx0 * m_srcChannels + c];
        double v11 = m_srcPixels[sy1 * stride + sx1 * m_srcChannels + c];

        double v = v00 * (1.0 - fx) * (1.0 - fy)
                 + v10 * fx * (1.0 - fy)
                 + v01 * (1.0 - fx) * fy
                 + v11 * fx * fy;

        out[c] = static_cast<uint8_t>(std::max(0.0, std::min(255.0, v)));
    }

    // Check noData: if sampled pixel matches noData value, make transparent
    if (valid && m_hasNoData) {
        if (out[0] == m_noDataR && out[1] == m_noDataG && out[2] == m_noDataB) {
            valid = false;
        }
    }

    out[3] = valid ? 255 : 0;
}

// ============================================================================
// RenderTile — generate a single 256×256 tile
//
// Rather than calling proj_trans() for each of the 65,536 pixels (3+ billion
// PROJ round-trips for a typical 50K-tile pyramid), we evaluate the full PROJ
// transform at the 4 corners of the tile only and use bilinear interpolation
// for all interior pixels.  Over a single tile's geographic extent (~150 m at
// zoom 18), the non-linear component of the transverse Mercator projection is
// sub-millimeter — well below pixel resolution.
// ============================================================================

bool ImageTiler::RenderTile(int level, int tx, int ty,
                             std::vector<uint8_t>& outRgba) const
{
    double tw, ts, te, tn;
    ComputeTileBounds(level, tx, ty, tw, ts, te, tn);

    // Quick rejection
    if (te <= m_geoWest || tw >= m_geoEast || tn <= m_geoSouth || ts >= m_geoNorth)
        return false;

    const int TS = 256;

    // — Compute PROJ at 4 tile corners only —
    // Corner geographic coords: (tw,tn)=NW, (te,tn)=NE, (tw,ts)=SW, (te,ts)=SE
    PJ_COORD cNW = proj_trans(m_projTransform, PJ_FWD, proj_coord(tw, tn, 0, 0));
    PJ_COORD cNE = proj_trans(m_projTransform, PJ_FWD, proj_coord(te, tn, 0, 0));
    PJ_COORD cSW = proj_trans(m_projTransform, PJ_FWD, proj_coord(tw, ts, 0, 0));
    PJ_COORD cSE = proj_trans(m_projTransform, PJ_FWD, proj_coord(te, ts, 0, 0));

    // Convert to source pixel coordinates at the 4 corners
    double sxNW = (cNW.xy.x - m_originX) / m_resolution;
    double syNW = (m_originY - cNW.xy.y) / m_resolution;
    double sxNE = (cNE.xy.x - m_originX) / m_resolution;
    double syNE = (m_originY - cNE.xy.y) / m_resolution;
    double sxSW = (cSW.xy.x - m_originX) / m_resolution;
    double sySW = (m_originY - cSW.xy.y) / m_resolution;
    double sxSE = (cSE.xy.x - m_originX) / m_resolution;
    double sySE = (m_originY - cSE.xy.y) / m_resolution;

    // Precompute the bilinear interpolation of source-pixel coordinates
    //   sx(u,v) = (1-u)(1-v)*sxNW + u(1-v)*sxNE + (1-u)v*sxSW + u*v*sxSE
    // where u = (px+0.5)/TS, v = (py+0.5)/TS
    const double invTS = 1.0 / TS;
    const double halfTS = 0.5 / TS;

    outRgba.resize(TS * TS * 4);

    bool hasContent = false;
    for (int py = 0; py < TS; ++py) {
        double v = (py + 0.5) * invTS;   // normalized Y: 0=top (north), 1=bottom (south)
        double u = halfTS;                // start at px=0's u
        for (int px = 0; px < TS; ++px, u += invTS) {
            // Bilinear interpolation: sx = lerp(lerp(sxNW, sxNE, u), lerp(sxSW, sxSE, u), v)
            double sx = (sxNW * (1.0 - u) + sxNE * u) * (1.0 - v)
                      + (sxSW * (1.0 - u) + sxSE * u) * v;
            double sy = (syNW * (1.0 - u) + syNE * u) * (1.0 - v)
                      + (sySW * (1.0 - u) + sySE * u) * v;

            int idx = (py * TS + px) * 4;
            SamplePixel(sx, sy, &outRgba[idx]);
            if (outRgba[idx + 3] > 0) hasContent = true;
        }
    }
    return hasContent;
}

// ============================================================================
// Convert — main pipeline
// ============================================================================

bool ImageTiler::Convert(const ImageTilerOptions& opts)
{
    // ---- 1. Read image ----
    ImageReader reader;
    if (!reader.Open(opts.inputTif)) return false;
    if (!reader.ReadRGB(m_srcPixels)) return false;

    m_srcW = reader.Width();
    m_srcH = reader.Height();
    m_srcChannels = reader.Channels();

    // Copy noData info (CLI override takes precedence over TIF-embedded)
    if (opts.hasNoDataOverride) {
        m_hasNoData = true;
        m_noDataR = opts.noDataR;
        m_noDataG = opts.noDataG;
        m_noDataB = opts.noDataB;
    } else {
        m_hasNoData = reader.HasNoData();
        m_noDataR = reader.NoDataR();
        m_noDataG = reader.NoDataG();
        m_noDataB = reader.NoDataB();
    }

    // ---- 2. Determine origin and resolution ----
    if (opts.hasOrigin) {
        m_originX = opts.originX;
        m_originY = opts.originY;
    } else if (reader.ScaleX() != 0) {
        m_originX = reader.TiePointX();
        m_originY = reader.TiePointY();
        // If ReadRGB flipped the image (south-up → north-up), the origin
        // must be adjusted from the original south-edge northing to the
        // new north-edge northing so the per-pixel formula
        //   sy = (m_originY - projY) / m_resolution
        // maps row 0 to the north edge.  See ReadModelTransform() for
        // orientation detection details.
        if (reader.IsSouthUp()) {
            double absScaleY = std::abs(reader.ScaleY());
            m_originY = m_originY + (m_srcH - 1) * absScaleY;
        }
    } else {
        std::cerr << "Error: No origin specified (use --origin or provide a georeferenced TIF)"
                  << std::endl;
        return false;
    }

    // Use both X and Y pixel scales.  For DOM images ScaleX and ScaleY
    // should be nearly equal; take the average for resolution.
    double scaleX = std::abs(reader.ScaleX());
    double scaleY = std::abs(reader.ScaleY());
    m_resolution = (scaleX + scaleY) * 0.5;
    if (m_resolution <= 0) {
        std::cerr << "Error: No pixel resolution found in TIF" << std::endl;
        return false;
    }

    // ---- 3. Load projection and setup PROJ ----
    std::string srcWkt;
    if (!opts.prjFile.empty()) {
        std::ifstream f(opts.prjFile);
        if (!f.is_open()) {
            std::cerr << "Error: Cannot open PRJ file: " << opts.prjFile << std::endl;
            return false;
        }
        std::stringstream ss;
        ss << f.rdbuf();
        srcWkt = ss.str();
    } else if (!reader.ProjectionWKT().empty()) {
        srcWkt = reader.ProjectionWKT();
    }

    if (srcWkt.empty()) {
        std::cerr << "Error: No projection specified. Provide --prj or use a TIF with embedded CRS."
                  << std::endl;
        return false;
    }

    // Setup PROJ: EPSG:4326 → source CRS with forced lon,lat axis order.
    // proj_normalize_for_visualization() ensures output always uses
    // longitude-first order regardless of the CRS's official axis definition,
    // eliminating the need for fragile heuristic detection.
    m_projCtx = proj_context_create();

    // PROJ database path resolution — shared implementation, see PROJUtils.h
    {
        std::string projDbDir = FindPROJDatabase();
        if (!projDbDir.empty()) {
            std::string dbPath = projDbDir + "/proj.db";
            proj_context_set_database_path(m_projCtx, dbPath.c_str(), nullptr, nullptr);
        }
    }

    PJ* rawTransform = proj_create_crs_to_crs(m_projCtx, "EPSG:4326",
                                               srcWkt.c_str(), nullptr);
    if (!rawTransform) {
        std::cerr << "Error: Failed to create PROJ transform EPSG:4326 → source CRS" << std::endl;
        return false;
    }
    m_projTransform = proj_normalize_for_visualization(m_projCtx, rawTransform);
    proj_destroy(rawTransform);
    if (!m_projTransform) {
        std::cerr << "Error: Failed to normalize PROJ transform" << std::endl;
        return false;
    }
    // proj_normalize_for_visualization ensures lon,lat axis order

    // ---- 4. Compute geographic bounds ----
    // Transform 4 corners of the image from GK → WGS84
    m_geoWest = 1e9; m_geoEast = -1e9;
    m_geoNorth = -1e9; m_geoSouth = 1e9;

    // Create reverse transform for bounds computation
    PJ* rawRev = proj_create_crs_to_crs(m_projCtx, srcWkt.c_str(),
                                         "EPSG:4326", nullptr);
    if (!rawRev) {
        std::cerr << "Error: Failed to create reverse PROJ transform" << std::endl;
        return false;
    }
    PJ* revTransform = proj_normalize_for_visualization(m_projCtx, rawRev);
    proj_destroy(rawRev);
    if (!revTransform) {
        std::cerr << "Error: Failed to normalize reverse PROJ transform" << std::endl;
        return false;
    }

    // Sample all 4 corners + mid-points.
    // Row direction: north-up → y decreases southward; south-up → y increases.
    double rowSign = reader.IsSouthUp() ? 1.0 : -1.0;
    struct { double e, n; } corners[] = {
        {m_originX, m_originY},                                             // NW
        {m_originX + (m_srcW-1)*m_resolution, m_originY},                   // NE
        {m_originX, m_originY + rowSign * (m_srcH-1)*m_resolution},         // SW
        {m_originX + (m_srcW-1)*m_resolution,
                     m_originY + rowSign * (m_srcH-1)*m_resolution},        // SE
    };

    // Normalized: lam=longitude, phi=latitude, no swap needed
    auto extractLonLat = [](const PJ_COORD& geo, double& lon, double& lat) {
        lon = geo.lp.lam;
        lat = geo.lp.phi;
    };

    for (int ci = 0; ci < 4; ++ci) {
        auto& c = corners[ci];
        PJ_COORD coord = proj_coord(c.e, c.n, 0, 0);
        PJ_COORD geo = proj_trans(revTransform, PJ_FWD, coord);
        double lon, lat;
        extractLonLat(geo, lon, lat);
        m_geoWest  = std::min(m_geoWest, lon);
        m_geoEast  = std::max(m_geoEast, lon);
        m_geoSouth = std::min(m_geoSouth, lat);
        m_geoNorth = std::max(m_geoNorth, lat);
    }

    // Also sample mid-edges (top and bottom edges at multiple X positions)
    double southN = m_originY + rowSign * (m_srcH - 1) * m_resolution;
    for (double f : {0.25, 0.5, 0.75}) {
        double midE = m_originX + f * (m_srcW - 1) * m_resolution;
        PJ_COORD c1 = proj_coord(midE, m_originY, 0, 0);
        PJ_COORD g1 = proj_trans(revTransform, PJ_FWD, c1);
        double lon, lat;
        extractLonLat(g1, lon, lat);
        m_geoWest = std::min(m_geoWest, lon);
        m_geoEast = std::max(m_geoEast, lon);

        PJ_COORD c2 = proj_coord(midE, southN, 0, 0);
        PJ_COORD g2 = proj_trans(revTransform, PJ_FWD, c2);
        extractLonLat(g2, lon, lat);
        m_geoWest = std::min(m_geoWest, lon);
        m_geoEast = std::max(m_geoEast, lon);
    }

    // Also sample mid-edges (left and right edges at multiple Y positions)
    for (double f : {0.25, 0.5, 0.75}) {
        double midN = m_originY + rowSign * f * (m_srcH - 1) * m_resolution;
        PJ_COORD c1 = proj_coord(m_originX, midN, 0, 0);
        PJ_COORD g1 = proj_trans(revTransform, PJ_FWD, c1);
        double lon, lat;
        extractLonLat(g1, lon, lat);
        m_geoSouth = std::min(m_geoSouth, lat);
        m_geoNorth = std::max(m_geoNorth, lat);

        PJ_COORD c2 = proj_coord(m_originX + (m_srcW-1)*m_resolution, midN, 0, 0);
        PJ_COORD g2 = proj_trans(revTransform, PJ_FWD, c2);
        extractLonLat(g2, lon, lat);
        m_geoSouth = std::min(m_geoSouth, lat);
        m_geoNorth = std::max(m_geoNorth, lat);
    }

    proj_destroy(revTransform);

    // ---- 5. Compute zoom levels ----
    double midLat = (m_geoNorth + m_geoSouth) / 2.0;
    double metersPerDeg = 111320.0 * std::cos(midLat * M_PI / 180.0);
    double pixelSizeDeg = m_resolution / metersPerDeg;
    double geoW = m_geoEast - m_geoWest;
    double geoH = m_geoNorth - m_geoSouth;

    int maxZoom = opts.maxZoom;
    if (maxZoom < 0) maxZoom = ComputeMaxLevel(pixelSizeDeg);
    int minZoom = opts.minZoom;
    if (minZoom < 0) minZoom = ComputeMinLevel(geoW, geoH);

    // ---- 6. Generate tiles ----
    mkdir_p(opts.outputDir.c_str());

    std::vector<int> tileCounts(maxZoom + 1, 0);
    int totalTiles = 0;
    int levelsTotal = maxZoom - minZoom + 1;
    int levelIdx = 0;

    for (int level = minZoom; level <= maxZoom; ++level) {
        int tilesTotal = 1 << level;   // Web Mercator: square N×N grid

        // X: linear in longitude
        int xStart = std::max(0, static_cast<int>(std::floor(
            (m_geoWest + 180.0) / 360.0 * tilesTotal)));
        int xEnd = std::min(tilesTotal - 1, static_cast<int>(std::ceil(
            (m_geoEast + 180.0) / 360.0 * tilesTotal)));

        // Y: Web Mercator TMS — y = floor((1+ln(tan(π/4+lat/2))/π) * N/2)
        auto mercTileY = [](double latDeg, int N) -> double {
            double latRad = latDeg * M_PI / 180.0;
            return (1.0 + std::log(std::tan(M_PI / 4.0 + latRad / 2.0)) / M_PI)
                   * 0.5 * N;
        };
        // North → larger TMS Y, South → smaller TMS Y
        int ySouth = static_cast<int>(std::floor(mercTileY(m_geoSouth, tilesTotal)));
        int yNorth = static_cast<int>(std::ceil(mercTileY(m_geoNorth, tilesTotal)));
        int yStart = std::max(0, ySouth);
        int yEnd = std::min(tilesTotal - 1, yNorth);

        int count = 0;
        for (int tx = xStart; tx <= xEnd; ++tx) {
            for (int ty = yStart; ty <= yEnd; ++ty) {
                std::vector<uint8_t> rgba;
                if (!RenderTile(level, tx, ty, rgba))
                    continue;

                // Skip fully transparent tiles (all noData)
                bool hasOpaque = false;
                for (size_t pi = 3; pi < rgba.size(); pi += 4) {
                    if (rgba[pi] > 0) { hasOpaque = true; break; }
                }
                if (!hasOpaque) continue;

                // Web Mercator TMS Y naming (Y=0 at south)
                int fileY = ty;

                // Build path: {outputDir}/{level}/{tx}/{fileY}.png
                fs::path dir = fs::path(opts.outputDir) / std::to_string(level)
                               / std::to_string(tx);
                std::error_code ec;
                fs::create_directories(dir, ec);
                std::string ext = opts.enableKtx2 ? ".ktx2" : ".png";
                fs::path filePath = dir / (std::to_string(fileY) + ext);

                if (opts.enableKtx2)
                {
#ifdef MGO_WITH_KTX2
                    basisu::basis_compressor_params params;
                    params.m_source_images.resize(1);
                    params.m_source_images[0].init(rgba.data(), 256, 256, 4);
                    params.m_uastc = false;
                    params.m_perceptual = true;
                    params.m_quality_level = 128;
                    params.m_mip_gen = true;
                    params.m_write_output_basis_or_ktx2 = true;
                    params.m_multithreading = true;
                    params.m_out_filename = filePath.string();

                    basisu::basis_compressor compressor;
                    if (compressor.init(params))
                        compressor.process();
                    else
                        std::cerr << "[ImageTiler] KTX2 compression init failed: "
                                  << filePath << std::endl;
#else
                    std::cerr << "[ImageTiler] KTX2 not built (MGO_WITH_KTX2 not defined), "
                              << "writing PNG: " << filePath << std::endl;
                    PngWriter::Write(filePath.string(), rgba.data(), 256, 256,
                                     opts.pngCompression);
#endif
                }
                else
                {
                    PngWriter::Write(filePath.string(), rgba.data(), 256, 256,
                                     opts.pngCompression);
                }
                ++count;
            }
        }

        tileCounts[level] = count;
        totalTiles += count;
        ++levelIdx;

        std::cout << "[ImageTiler] 处理进度: " << levelIdx << "/"
                  << levelsTotal << std::endl;
    }

    std::cout << "[ImageTiler] 处理完成: " << totalTiles << " 瓦片 ("
              << levelsTotal << " 层级)" << std::endl;

    // ---- 7. Write metadata ----
    WriteLayerJson(opts.outputDir, m_geoWest, m_geoSouth, m_geoEast, m_geoNorth,
                   minZoom, maxZoom, opts.enableKtx2);
    WriteTilemapXml(opts.outputDir, m_geoWest, m_geoSouth, m_geoEast, m_geoNorth,
                    minZoom, maxZoom, tileCounts, opts.enableKtx2);

    return totalTiles > 0;
}

// ============================================================================
// Metadata writers
// ============================================================================

bool ImageTiler::WriteLayerJson(const std::string& dir,
                                 double west, double south,
                                 double east, double north,
                                 int minZoom, int maxZoom, bool ktx2) const
{
    std::string path = dir + "/layer.json";
    std::ofstream f(path);
    if (!f) return false;

    std::string format = ktx2 ? "ktx2" : "png";
    std::string ext = ktx2 ? ".ktx2" : ".png";
    f << "{\n"
      << "  \"tilejsonVersion\": \"1.0.0\",\n"
      << "  \"format\": \"" << format << "\",\n"
      << "  \"version\": \"1.0.0\",\n"
      << "  \"tiles\": [\"{z}/{x}/{y}" << ext << "\"],\n"
      << "  \"bounds\": [" << west << ", " << south << ", " << east << ", " << north << "],\n"
      << "  \"projection\": \"EPSG:4326\",\n"
      << "  \"minzoom\": " << minZoom << ",\n"
      << "  \"maxzoom\": " << maxZoom << "\n"
      << "}\n";

    return true;
}

bool ImageTiler::WriteTilemapXml(const std::string& dir,
                                  double west, double south,
                                  double east, double north,
                                  int minZoom, int maxZoom,
                                  const std::vector<int>& tileCounts,
                                  bool ktx2) const
{
    std::string path = dir + "/tilemapresource.xml";
    std::ofstream f(path);
    if (!f) return false;

    // global-mercator: full world extent in Mercator meters
    constexpr double MERC_MAX = 20037508.342789244;

    f << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
      << "<TileMap version=\"1.0.0\" tilemapservice=\"http://tms.osgeo.org/1.0.0\""
      << " profile=\"global-mercator\">\n"
      << "  <Title>DOM Orthophoto Tiles</Title>\n"
      << "  <Abstract>Web Mercator TMS tiles from DOM orthophoto</Abstract>\n"
      << "  <SRS>EPSG:3857</SRS>\n"
      << "  <BoundingBox minx=\"" << -MERC_MAX << "\" miny=\"" << -MERC_MAX
      << "\" maxx=\"" << MERC_MAX << "\" maxy=\"" << MERC_MAX << "\"/>\n"
      << "  <Origin x=\"" << -MERC_MAX << "\" y=\"" << -MERC_MAX << "\"/>\n"
      << "  <TileFormat width=\"256\" height=\"256\""
      << " mime-type=\"" << (ktx2 ? "image/ktx2" : "image/png") << "\""
      << " extension=\"" << (ktx2 ? "ktx2" : "png") << "\"/>\n"
      << "  <TileSets>\n";

    for (int level = minZoom; level <= maxZoom; ++level) {
        if (level < static_cast<int>(tileCounts.size()) && tileCounts[level] > 0) {
            double res = 360.0 / (2 * (1 << level)) / 256.0;
            f << "    <TileSet href=\"" << level
              << "\" units-per-pixel=\"" << res
              << "\" order=\"" << level << "\"/>\n";
        }
    }

    f << "  </TileSets>\n"
      << "</TileMap>\n";

    return true;
}
