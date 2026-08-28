#include "JpgWriter.h"

#include <cstdio>
#include <cstring>
#include <iostream>
#include <vector>
#include <jpeglib.h>

bool JpgWriter::Write(const std::string& path,
                      const uint8_t* rgb,
                      int width, int height, int quality)
{
    FILE* fp = fopen(path.c_str(), "wb");
    if (!fp) {
        std::cerr << "[JpgWriter] Cannot open: " << path << std::endl;
        return false;
    }

    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);
    jpeg_stdio_dest(&cinfo, fp);

    cinfo.image_width = width;
    cinfo.image_height = height;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_RGB;

    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, quality, TRUE);

    jpeg_start_compress(&cinfo, TRUE);

    // Write scanlines (JPEG expects top-down row order)
    // Our RGBA data: skip alpha channel, write RGB only
    std::vector<uint8_t> scanline(width * 3);
    while (cinfo.next_scanline < cinfo.image_height) {
        int row = cinfo.next_scanline;
        const uint8_t* src = rgb + static_cast<size_t>(row) * width * 4;
        for (int x = 0; x < width; ++x) {
            scanline[x * 3 + 0] = src[x * 4 + 0];  // R
            scanline[x * 3 + 1] = src[x * 4 + 1];  // G
            scanline[x * 3 + 2] = src[x * 4 + 2];  // B
        }
        JSAMPROW rowPtr = scanline.data();
        jpeg_write_scanlines(&cinfo, &rowPtr, 1);
    }

    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);
    fclose(fp);
    return true;
}

bool JpgWriter::WriteWithDirs(const std::string& path,
                              const uint8_t* rgb,
                              int width, int height, int quality)
{
    size_t pos = path.find_last_of('/');
    if (pos != std::string::npos) {
        std::string dir = path.substr(0, pos);
        std::string cmd = "mkdir -p \"" + dir + "\"";
        if (system(cmd.c_str()) != 0) { /* dir may exist */ }
    }
    return Write(path, rgb, width, height, quality);
}
