// Copyright Johnlyon
//
// TextureAtlas — build 2×2 texture atlas from 4 child tile textures
//
// Assumes children are arranged in a quadtree:
//   [0]=NW, [1]=NE, [2]=SW, [3]=SE
//
// If child textures have different sizes, the atlas is sized to
// 2 × maxWidth × 2 × maxHeight, and smaller textures are placed
// in their quadrant without scaling (centered in the quadrant).
//
// UV coordinates are remapped to atlas space.
//

#pragma once

#include "macro.h"

#include <string>
#include <vector>
#include <cstdint>
#include <memory>

// Forward declarations
namespace osg { class Image; }

// ---------------------------------------------------------------------------
// Result of building a texture atlas
// ---------------------------------------------------------------------------
struct AtlasResult
{
    std::vector<uint8_t> pixels;    // RGBA pixel data
    int width = 0;
    int height = 0;
    std::string outputPath;         // path to the written atlas file
    bool success = false;
};

// ---------------------------------------------------------------------------
// TextureAtlas — 2×2 quadtree texture atlas builder
// ---------------------------------------------------------------------------
class OSGB_CONVERTER_API TextureAtlas
{
public:
    TextureAtlas() = default;

    // Build a 2×2 atlas from 4 child texture paths.
    //   childPaths[4]: paths to child texture images (may be empty)
    //   rootDir:       base directory for resolving relative paths
    //   outputDir:     directory to write the atlas image
    //   outputName:    base filename for the atlas (e.g. "tile_L0_0_0_atlas")
    //   format:        "jpg" or "png"
    //   quality:       JPEG quality (0-100), ignored for PNG
    // Returns the atlas result with pixel data and output path.
    AtlasResult Build(const std::string childPaths[4],
                      const std::string& rootDir,
                      const std::string& outputDir,
                      const std::string& outputName,
                      const std::string& format,
                      int quality = 90);

    // Remap UV coordinates from child tile space to atlas space.
    //   childIdx: 0=NW, 1=NE, 2=SW, 3=SE
    //   u, v:     original UV coordinates [0, 1]
    // Returns true if the child has a valid texture.
    bool RemapUV(int childIdx, float& u, float& v) const;

    // Get the atlas dimensions after Build().
    int AtlasWidth()  const { return m_atlasWidth; }
    int AtlasHeight() const { return m_atlasHeight; }

private:
    // Read a texture image using OSG (supports JPEG, PNG, TIFF, etc.)
    bool ReadTexture(const std::string& path, std::vector<uint8_t>& rgba,
                     int& width, int& height);

    // Write atlas to file using JPEG
    bool WriteJPEG(const std::string& path, const uint8_t* rgb,
                   int width, int height, int quality);

    // Write atlas to file using PNG
    bool WritePNG(const std::string& path, const uint8_t* rgba,
                  int width, int height);

    // Child texture dimensions
    int m_childWidth[4]  = { 0, 0, 0, 0 };
    int m_childHeight[4] = { 0, 0, 0, 0 };
    bool m_childValid[4] = { false, false, false, false };

    // Maximum child dimensions
    int m_maxChildWidth  = 0;
    int m_maxChildHeight = 0;

    // Atlas dimensions
    int m_atlasWidth  = 0;
    int m_atlasHeight = 0;

    // Quadrant offsets in atlas pixels
    int m_quadrantX[4] = { 0, 0, 0, 0 };
    int m_quadrantY[4] = { 0, 0, 0, 0 };
};