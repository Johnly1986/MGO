#pragma once

#include <string>
#include <cstdint>

// JpgWriter — write 256×256 JPEG tiles using libjpeg-turbo

class JpgWriter
{
public:
    // Write a 256×256 RGB JPEG file.
    // pixels: flat array of width*height*3 bytes (R,G,B per pixel, row-major).
    // quality: 1–100 (default 85).
    static bool Write(const std::string& path,
                      const uint8_t* rgb,
                      int width = 256, int height = 256,
                      int quality = 85);

    // Create directories and write
    static bool WriteWithDirs(const std::string& path,
                              const uint8_t* rgb,
                              int width = 256, int height = 256,
                              int quality = 85);
};
