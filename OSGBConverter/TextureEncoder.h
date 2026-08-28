// Copyright Johnlyon
//
// TextureEncoder — abstract texture encoding interface
//
// Provides:
//   - JPEG encoder (default, uses libjpeg)
//   - PNG encoder (lossless, uses libpng)
//   - KTX2 encoder interface (placeholder for future GPU-compressed textures)
//
// Usage:
//   auto encoder = TextureEncoder::Create(TextureFormat::JPEG);
//   encoder->SetQuality(90);
//   encoder->Encode(rgba, width, height, outputPath);
//

#pragma once

#include "macro.h"

#include <string>
#include <vector>
#include <cstdint>
#include <memory>

// ---------------------------------------------------------------------------
// Supported texture formats
// ---------------------------------------------------------------------------
enum class TextureFormat
{
    JPEG,   // lossy, RGB, libjpeg
    PNG,    // lossless, RGBA, libpng
    KTX2,   // GPU-compressed (Basis Universal), requires -DMGO_WITH_KTX2
};

// ---------------------------------------------------------------------------
// Abstract texture encoder interface
// ---------------------------------------------------------------------------
class OSGB_CONVERTER_API ITextureEncoder
{
public:
    virtual ~ITextureEncoder() = default;

    // Set encoding quality (0-100). Meaning depends on format:
    //   JPEG: compression quality (default 90)
    //   PNG:  compression level (default 9)
    //   KTX2: quality level for Basis encoder (default 128 = max)
    virtual void SetQuality(int quality) = 0;

    // Encode RGBA pixel data to a file.
    //   rgba:  interleaved RGBA, width*height*4 bytes
    //   path:  output file path
    virtual bool Encode(const uint8_t* rgba, int width, int height,
                        const std::string& path) = 0;

    // Get the file extension for this format (including dot).
    virtual std::string Extension() const = 0;

    // Factory: create encoder for the given format.
    static std::unique_ptr<ITextureEncoder> Create(TextureFormat format);
};

// ---------------------------------------------------------------------------
// JPEG encoder (libjpeg)
// ---------------------------------------------------------------------------
class OSGB_CONVERTER_API JPEGEncoder : public ITextureEncoder
{
public:
    void SetQuality(int quality) override { m_quality = quality; }
    bool Encode(const uint8_t* rgba, int width, int height,
                const std::string& path) override;
    std::string Extension() const override { return ".jpg"; }

private:
    int m_quality = 90;
};

// ---------------------------------------------------------------------------
// PNG encoder (libpng)
// ---------------------------------------------------------------------------
class OSGB_CONVERTER_API PNGEncoder : public ITextureEncoder
{
public:
    void SetQuality(int quality) override { m_compressionLevel = quality; }
    bool Encode(const uint8_t* rgba, int width, int height,
                const std::string& path) override;
    std::string Extension() const override { return ".png"; }

private:
    int m_compressionLevel = 9;
};

// ---------------------------------------------------------------------------
// KTX2 encoder (Basis Universal ETC1S/UASTC)
//
// Requires: basisu (vcpkg install basisu) + CMake -DMGO_WITH_KTX2
// Without MGO_WITH_KTX2: falls back to PNG with a log message
// Output: KTX2 container with compressed mip-mapped texture
// Supported by: CesiumJS >= 1.85, WebGL 2.0
// ---------------------------------------------------------------------------
class OSGB_CONVERTER_API KTX2Encoder : public ITextureEncoder
{
public:
    void SetQuality(int quality) override { m_quality = quality; }
    bool Encode(const uint8_t* rgba, int width, int height,
                const std::string& path) override;
    std::string Extension() const override { return ".ktx2"; }

private:
    int m_quality = 128;  // Basis quality level
};