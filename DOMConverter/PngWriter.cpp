#include "PngWriter.h"

#include <png.h>
#include <cstring>
#include <iostream>

bool PngWriter::Write(const std::string& path,
                      const uint8_t* rgba,
                      int width, int height,
                      int compressionLevel)
{
    FILE* fp = fopen(path.c_str(), "wb");
    if (!fp) {
        std::cerr << "[PngWriter] Cannot open: " << path << std::endl;
        return false;
    }

    png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (!png) { fclose(fp); return false; }

    png_infop info = png_create_info_struct(png);
    if (!info) { png_destroy_write_struct(&png, nullptr); fclose(fp); return false; }

    if (setjmp(png_jmpbuf(png))) {
        png_destroy_write_struct(&png, &info);
        fclose(fp);
        return false;
    }

    png_init_io(png, fp);

    // Set compression level (0=none, 9=max)
    png_set_compression_level(png, compressionLevel);
    // Use filtered compression for better ratio with photo content
    png_set_filter(png, 0, PNG_FILTER_NONE | PNG_FILTER_SUB | PNG_FILTER_UP);

    png_set_IHDR(png, info, width, height, 8,
                 PNG_COLOR_TYPE_RGBA,
                 PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_DEFAULT,
                 PNG_FILTER_TYPE_DEFAULT);
    png_write_info(png, info);

    std::vector<png_bytep> rowPointers(height);
    for (int y = 0; y < height; ++y) {
        rowPointers[y] = const_cast<png_bytep>(rgba + static_cast<size_t>(y) * width * 4);
    }
    png_write_image(png, rowPointers.data());
    png_write_end(png, nullptr);

    png_destroy_write_struct(&png, &info);
    fclose(fp);
    return true;
}

bool PngWriter::WriteWithDirs(const std::string& path,
                              const uint8_t* rgba,
                              int width, int height,
                              int compressionLevel)
{
    size_t pos = path.find_last_of('/');
    if (pos != std::string::npos) {
        std::string dir = path.substr(0, pos);
        std::string cmd = "mkdir -p \"" + dir + "\"";
        if (system(cmd.c_str()) != 0) { /* dir may exist */ }
    }
    return Write(path, rgba, width, height, compressionLevel);
}
