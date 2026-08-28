// Copyright Johnlyon
//
// TextureAtlas implementation — 2×2 quadtree atlas builder
//

#include "TextureAtlas.h"

#include <osg/Image>
#include <osgDB/ReadFile>

#include <cstdio>
#include <cstring>
#include <algorithm>
#include <iostream>
#include <jpeglib.h>
#include <png.h>

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

AtlasResult TextureAtlas::Build(const std::string childPaths[4],
                                 const std::string& rootDir,
                                 const std::string& outputDir,
                                 const std::string& outputName,
                                 const std::string& format,
                                 int quality)
{
    AtlasResult result;

    // Reset state
    for (int i = 0; i < 4; ++i)
    {
        m_childValid[i]  = false;
        m_childWidth[i]  = 0;
        m_childHeight[i] = 0;
    }
    m_maxChildWidth  = 0;
    m_maxChildHeight = 0;

    // Phase 1: Read all child textures and determine dimensions
    std::vector<uint8_t> childPixels[4];
    for (int i = 0; i < 4; ++i)
    {
        if (childPaths[i].empty()) continue;

        std::string fullPath = childPaths[i];
        if (fullPath[0] != '/' && fullPath.find(':') == std::string::npos)
            fullPath = rootDir + "/" + fullPath;

        int w, h;
        if (!ReadTexture(fullPath, childPixels[i], w, h))
        {
            std::cerr << "[TextureAtlas] Warning: cannot read texture: "
                      << fullPath << std::endl;
            continue;
        }

        m_childValid[i]  = true;
        m_childWidth[i]  = w;
        m_childHeight[i] = h;

        if (w > m_maxChildWidth)  m_maxChildWidth  = w;
        if (h > m_maxChildHeight) m_maxChildHeight = h;
    }

    // Check if any valid child textures
    int validCount = 0;
    for (int i = 0; i < 4; ++i)
        if (m_childValid[i]) ++validCount;

    if (validCount == 0)
    {
        result.success = false;
        return result;
    }

    // Phase 2: Compute atlas dimensions
    // Atlas = 2×maxChildWidth × 2×maxChildHeight
    m_atlasWidth  = m_maxChildWidth * 2;
    m_atlasHeight = m_maxChildHeight * 2;

    // Compute quadrant offsets (in pixels)
    // Quadrant layout:  [0]=NW(0,0),  [1]=NE(maxW,0),  [2]=SW(0,maxH),  [3]=SE(maxW,maxH)
    m_quadrantX[0] = 0;                    m_quadrantY[0] = 0;
    m_quadrantX[1] = m_maxChildWidth;       m_quadrantY[1] = 0;
    m_quadrantX[2] = 0;                    m_quadrantY[2] = m_maxChildHeight;
    m_quadrantX[3] = m_maxChildWidth;       m_quadrantY[3] = m_maxChildHeight;

    // Phase 3: Build atlas
    std::vector<uint8_t> atlas(m_atlasWidth * m_atlasHeight * 4, 0);

    for (int i = 0; i < 4; ++i)
    {
        if (!m_childValid[i]) continue;

        int cw = m_childWidth[i];
        int ch = m_childHeight[i];
        int ox = m_quadrantX[i];
        int oy = m_quadrantY[i];

        // Center the child texture in its quadrant if smaller than max
        int offsetX = (m_maxChildWidth - cw) / 2;
        int offsetY = (m_maxChildHeight - ch) / 2;

        const uint8_t* src = childPixels[i].data();
        for (int row = 0; row < ch; ++row)
        {
            int dstRow = oy + offsetY + row;
            if (dstRow >= m_atlasHeight) break;
            for (int col = 0; col < cw; ++col)
            {
                int dstCol = ox + offsetX + col;
                if (dstCol >= m_atlasWidth) break;

                size_t srcIdx = (static_cast<size_t>(row) * cw + col) * 4;
                size_t dstIdx = (static_cast<size_t>(dstRow) * m_atlasWidth + dstCol) * 4;

                atlas[dstIdx + 0] = src[srcIdx + 0];
                atlas[dstIdx + 1] = src[srcIdx + 1];
                atlas[dstIdx + 2] = src[srcIdx + 2];
                atlas[dstIdx + 3] = src[srcIdx + 3];
            }
        }
    }

    // Phase 4: Write atlas to file
    std::string ext = (format == "png") ? ".png" : ".jpg";
    std::string outPath = outputDir + "/" + outputName + ext;

    bool ok = false;
    if (format == "png")
        ok = WritePNG(outPath, atlas.data(), m_atlasWidth, m_atlasHeight);
    else
        ok = WriteJPEG(outPath, atlas.data(), m_atlasWidth, m_atlasHeight, quality);

    if (!ok)
    {
        result.success = false;
        return result;
    }

    result.pixels     = std::move(atlas);
    result.width      = m_atlasWidth;
    result.height     = m_atlasHeight;
    result.outputPath = outPath;
    result.success    = true;

    return result;
}

bool TextureAtlas::RemapUV(int childIdx, float& u, float& v) const
{
    if (childIdx < 0 || childIdx >= 4 || !m_childValid[childIdx])
        return false;

    int cw = m_childWidth[childIdx];
    int ch = m_childHeight[childIdx];

    // Offset within quadrant (centered)
    float offsetU = static_cast<float>(m_maxChildWidth - cw) * 0.5f / m_atlasWidth;
    float offsetV = static_cast<float>(m_maxChildHeight - ch) * 0.5f / m_atlasHeight;

    // Scale from child texture space to atlas texture space
    float scaleU = static_cast<float>(cw) / m_atlasWidth;
    float scaleV = static_cast<float>(ch) / m_atlasHeight;

    // Quadrant base position
    float baseU = static_cast<float>(m_quadrantX[childIdx]) / m_atlasWidth;
    float baseV = static_cast<float>(m_quadrantY[childIdx]) / m_atlasHeight;

    u = baseU + offsetU + u * scaleU;
    v = baseV + offsetV + v * scaleV;

    return true;
}

// ---------------------------------------------------------------------------
// Image I/O
// ---------------------------------------------------------------------------

bool TextureAtlas::ReadTexture(const std::string& path,
                                std::vector<uint8_t>& rgba,
                                int& width, int& height)
{
    osg::ref_ptr<osg::Image> img = osgDB::readImageFile(path);
    if (!img || !img->data())
        return false;

    width  = img->s();
    height = img->t();

    size_t pixelCount = static_cast<size_t>(width) * height;
    rgba.resize(pixelCount * 4);

    int channels = img->getPixelSizeInBits() / 8;
    const uint8_t* src = img->data();

    // Convert to RGBA
    if (channels == 4)
    {
        std::memcpy(rgba.data(), src, pixelCount * 4);
    }
    else if (channels == 3)
    {
        for (size_t i = 0; i < pixelCount; ++i)
        {
            rgba[i * 4 + 0] = src[i * 3 + 0];
            rgba[i * 4 + 1] = src[i * 3 + 1];
            rgba[i * 4 + 2] = src[i * 3 + 2];
            rgba[i * 4 + 3] = 255;
        }
    }
    else if (channels == 1)
    {
        for (size_t i = 0; i < pixelCount; ++i)
        {
            rgba[i * 4 + 0] = src[i];
            rgba[i * 4 + 1] = src[i];
            rgba[i * 4 + 2] = src[i];
            rgba[i * 4 + 3] = 255;
        }
    }
    else
    {
        return false;
    }

    return true;
}

bool TextureAtlas::WriteJPEG(const std::string& path,
                              const uint8_t* rgba,
                              int width, int height, int quality)
{
    FILE* fp = fopen(path.c_str(), "wb");
    if (!fp)
    {
        std::cerr << "[TextureAtlas] Cannot open for writing: " << path << std::endl;
        return false;
    }

    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);
    jpeg_stdio_dest(&cinfo, fp);

    cinfo.image_width      = width;
    cinfo.image_height     = height;
    cinfo.input_components = 3;
    cinfo.in_color_space   = JCS_RGB;

    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, quality, TRUE);
    jpeg_start_compress(&cinfo, TRUE);

    std::vector<uint8_t> scanline(width * 3);
    while (cinfo.next_scanline < cinfo.image_height)
    {
        const uint8_t* src = rgba + static_cast<size_t>(cinfo.next_scanline) * width * 4;
        for (int x = 0; x < width; ++x)
        {
            scanline[x * 3 + 0] = src[x * 4 + 0];
            scanline[x * 3 + 1] = src[x * 4 + 1];
            scanline[x * 3 + 2] = src[x * 4 + 2];
        }
        JSAMPROW rowPtr = scanline.data();
        jpeg_write_scanlines(&cinfo, &rowPtr, 1);
    }

    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);
    fclose(fp);
    return true;
}

bool TextureAtlas::WritePNG(const std::string& path,
                             const uint8_t* rgba,
                             int width, int height)
{
    FILE* fp = fopen(path.c_str(), "wb");
    if (!fp) return false;

    png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (!png) { fclose(fp); return false; }

    png_infop info = png_create_info_struct(png);
    if (!info) { png_destroy_write_struct(&png, nullptr); fclose(fp); return false; }

    if (setjmp(png_jmpbuf(png)))
    {
        png_destroy_write_struct(&png, &info);
        fclose(fp);
        return false;
    }

    png_init_io(png, fp);
    png_set_IHDR(png, info, width, height, 8, PNG_COLOR_TYPE_RGBA,
                 PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT,
                 PNG_FILTER_TYPE_DEFAULT);
    png_write_info(png, info);

    std::vector<png_bytep> rows(height);
    for (int i = 0; i < height; ++i)
        rows[i] = const_cast<png_bytep>(rgba + static_cast<size_t>(i) * width * 4);

    png_write_image(png, rows.data());
    png_write_end(png, nullptr);
    png_destroy_write_struct(&png, &info);
    fclose(fp);
    return true;
}