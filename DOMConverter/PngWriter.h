#pragma once

#include <string>
#include <vector>
#include <cstdint>

// PngWriter — write 256×256 RGBA PNG tiles using libpng

class PngWriter
{
public:
    // Write a 256×256 RGBA PNG file.
    // pixels: flat array of width*height*4 bytes (R,G,B,A per pixel, row-major).
    // compressionLevel: 0 (none) to 9 (max), default 9.
    static bool Write(const std::string& path,
                      const uint8_t* rgba,
                      int width = 256, int height = 256,
                      int compressionLevel = 9);

    // Write helper: create directories if needed
    static bool WriteWithDirs(const std::string& path,
                              const uint8_t* rgba,
                              int width = 256, int height = 256,
                              int compressionLevel = 9);
};
