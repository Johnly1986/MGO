// Copyright Johnlyon
//
// TextureEncoder implementation
//

#include "TextureEncoder.h"

#include <cstdio>
#include <cstring>
#include <iostream>
#include <jpeglib.h>
#include <png.h>

// ---------------------------------------------------------------------------
// Factory
// ---------------------------------------------------------------------------

std::unique_ptr<ITextureEncoder> ITextureEncoder::Create(TextureFormat format)
{
    switch (format)
    {
    case TextureFormat::JPEG: return std::make_unique<JPEGEncoder>();
    case TextureFormat::PNG:  return std::make_unique<PNGEncoder>();
    case TextureFormat::KTX2: return std::make_unique<KTX2Encoder>();
    }
    return std::make_unique<JPEGEncoder>();
}

// ---------------------------------------------------------------------------
// JPEG Encoder
// ---------------------------------------------------------------------------

bool JPEGEncoder::Encode(const uint8_t* rgba, int width, int height,
                          const std::string& path)
{
    FILE* fp = fopen(path.c_str(), "wb");
    if (!fp)
    {
        std::cerr << "[JPEGEncoder] Cannot open: " << path << std::endl;
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
    jpeg_set_quality(&cinfo, m_quality, TRUE);
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

// ---------------------------------------------------------------------------
// PNG Encoder
// ---------------------------------------------------------------------------

bool PNGEncoder::Encode(const uint8_t* rgba, int width, int height,
                         const std::string& path)
{
    FILE* fp = fopen(path.c_str(), "wb");
    if (!fp)
    {
        std::cerr << "[PNGEncoder] Cannot open: " << path << std::endl;
        return false;
    }

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
    png_set_compression_level(png, m_compressionLevel);
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

// ---------------------------------------------------------------------------
// KTX2 Encoder (Basis Universal ETC1S / UASTC)
//
// Dependencies: basisu (Basis Universal v1.16+)
//   CMake: find_package(basisu) or vcpkg install basisu
//   Build: -DMGO_WITH_KTX2 to enable; absent → PNG fallback
// ---------------------------------------------------------------------------

#ifdef MGO_WITH_KTX2
#include <basisu/encoder/basisu_comp.h>
#include <basisu/transcoder/basisu_transcoder.h>
#endif

bool KTX2Encoder::Encode(const uint8_t* rgba, int width, int height,
                          const std::string& path)
{
#ifdef MGO_WITH_KTX2
    if (width < 4 || height < 4 || !rgba) return false;

    // Setup basisu compression parameters
    basisu::basis_compressor_params params;
    params.m_source_images.resize(1);
    params.m_source_images[0].init(rgba, static_cast<uint32_t>(width),
                                    static_cast<uint32_t>(height), 4);
    // Use ETC1S for best compression ratio (alternative: UASTC for higher quality)
    params.m_uastc = false;
    params.m_perceptual = true;
    params.m_quality_level = 128;               // 0-255, 128=default
    params.m_mip_gen = true;                    // Generate mipmaps
    params.m_mip_srgb = false;
    params.m_write_output_basis_or_ktx2 = true; // Output KTX2 container
    params.m_ktx2_srgb_transfer = false;
    params.m_ktx2_etc1s_transfer = true;

    // Multithreading: use all available cores
    params.m_multithreading = true;

    // Output file path
    params.m_out_filename = path;

    basisu::basis_compressor compressor;
    if (!compressor.init(params))
    {
        std::cerr << "[KTX2Encoder] Failed to initialize basisu compressor" << std::endl;
        return false;
    }

    basisu::basis_compressor::error_code ec = compressor.process();
    if (ec != basisu::basis_compressor::cECSuccess)
    {
        std::cerr << "[KTX2Encoder] Compression failed" << std::endl;
        return false;
    }

    std::cout << "[KTX2Encoder] Written: " << path
              << " (" << width << "x" << height << ")" << std::endl;
    return true;
#else
    // KTX2 not enabled at build time — fall back to PNG.
    // To enable: install basisu (vcpkg install basisu) and rebuild with
    // -DMGO_WITH_KTX2 in CMake.
    std::cerr << "[KTX2Encoder] Not built with KTX2 support (MGO_WITH_KTX2 not defined). "
              << "Falling back to PNG." << std::endl;
    PNGEncoder fallback;
    fallback.SetQuality(6);  // PNG compression level via ITextureEncoder::SetQuality
    return fallback.Encode(rgba, width, height, path);
#endif
}