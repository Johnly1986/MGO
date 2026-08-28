#!/usr/bin/env python3
"""Generate a synthetic GeoTIFF to diagnose tile degeneration.

Creates a 256x256 GeoTIFF with a single Gaussian hill (100m peak)
in EPSG:4326 (geographic). The hill is smooth and symmetric,
ideal for testing TIN simplification behavior.

Usage: python3 debug_degenerate.py
Output: debug_hill.tif in the same directory
"""
import struct
import math
import os

OUT_PATH = os.path.join(os.path.dirname(__file__), "debug_hill.tif")

# Parameters
WIDTH = 256
HEIGHT = 256
WEST = 120.0
EAST = WEST + WIDTH * 0.0001  # ~1.1km at equator
SOUTH = 30.0
NORTH = SOUTH + HEIGHT * 0.0001

# Gaussian hill: peak 100m at center, sigma = 30% of grid
CX = (WEST + EAST) / 2.0
CY = (SOUTH + NORTH) / 2.0
SIGMA_X = (EAST - WEST) * 0.3
SIGMA_Y = (NORTH - SOUTH) * 0.3
PEAK = 100.0

def generate_elevation():
    """Gaussian hill: h = PEAK * exp(-0.5*((x-cx)^2/sx^2 + (y-cy)^2/sy^2))"""
    elev = []
    for r in range(HEIGHT):
        for c in range(WIDTH):
            lon = WEST + (c + 0.5) / WIDTH * (EAST - WEST)
            lat = NORTH - (r + 0.5) / HEIGHT * (NORTH - SOUTH)  # row 0 = north
            dx = (lon - CX) / SIGMA_X
            dy = (lat - CY) / SIGMA_Y
            h = PEAK * math.exp(-0.5 * (dx*dx + dy*dy))
            elev.append(h)
    return elev


def write_geotiff(path, width, height, elev, west, east, south, north):
    """Minimal GeoTIFF writer (uncompressed float32)."""
    # TIFF header + IFD
    import io
    buf = io.BytesIO()

    def u16(v): buf.write(struct.pack("<H", v))
    def u32(v): buf.write(struct.pack("<I", v))
    def i32(v): buf.write(struct.pack("<i", v))
    def f64(v): buf.write(struct.pack("<d", v))
    def f32(v): buf.write(struct.pack("<f", v))

    pixel_scale_x = (east - west) / width
    pixel_scale_y = (north - south) / height
    tie_point = [west, north, 0, 0, 0, 0]  # lon, lat, 0, col, row, 0

    # Strip offsets
    data_offset = 8 + 2 + 12 * 20 + 4  # header + ntags + tags + next_ifd
    strip_offsets = []
    for r in range(height):
        strip_offsets.append(data_offset + r * width * 4)

    # GeoTIFF tags
    # ModelPixelScaleTag (33550): 3 doubles
    # ModelTiepointTag (33922): 6 doubles
    # GeoASCIIParamsTag (34737): "WGS 84\0"
    # GeoKeyDirectoryTag (34735): 4 shorts per key
    # SampleFormat (33922): 3 = IEEE float

    # Calculate offsets
    ps_offset = data_offset + height * width * 4
    tp_offset = ps_offset + 3 * 8
    geo_key_offset = tp_offset + 6 * 8
    geo_ascii_offset = geo_key_offset + 4 * 4 * 2  # 2 keys * 4 shorts each
    # Actually simpler: just write tags pointing to data after strips

    # TIFF header
    buf.write(b'II')  # little-endian
    u16(42)  # magic
    u32(8)  # IFD offset

    # IFD
    n_tags = 14
    u16(n_tags)

    def tag(id, type, count, value):
        u16(id)
        u16(type)  # 3=short, 4=long, 5=rational, 12=double
        u32(count)
        if type == 3 and count == 1:
            u16(value); u16(0)
        elif type == 4 and count == 1:
            u32(value)
        elif type == 12 and count == 1:
            f64(value);  # stored inline if fits
        else:
            u32(value)  # offset to data

    # Tags (must be sorted by ID)
    tag(256, 4, 1, width)         # ImageWidth
    tag(257, 4, 1, height)        # ImageLength
    tag(258, 3, 1, 32)            # BitsPerSample = 32
    tag(259, 3, 1, 1)             # Compression = None
    tag(262, 3, 1, 1)             # PhotometricInterpretation = MinIsBlack
    tag(273, 4, height, 8 + 2 + n_tags * 12 + 4)  # StripOffsets
    tag(277, 3, 1, 1)             # SamplesPerPixel = 1
    tag(278, 4, 1, height)        # RowsPerStrip = height (one strip per row)
    tag(279, 4, height, 8 + 2 + n_tags * 12 + 4)  # StripByteCounts (same as offsets for uncompressed)
    tag(282, 5, 1, 0)             # XResolution (not used)
    tag(283, 5, 1, 0)             # YResolution (not used)
    tag(296, 3, 1, 1)             # ResolutionUnit = None
    tag(339, 3, 1, 3)             # SampleFormat = IEEE float

    # GeoTIFF tags
    ps_off = 8 + 2 + n_tags * 12 + 4 + height * width * 4
    tp_off = ps_off + 3 * 8
    tag(33550, 12, 3, ps_off)     # ModelPixelScaleTag
    tag(33922, 12, 6, tp_off)     # ModelTiepointTag

    u32(0)  # next IFD = 0

    # Pixel data (float32, row by row)
    for h in elev:
        f32(h)

    # ModelPixelScaleTag data (3 doubles)
    f64(pixel_scale_x)
    f64(pixel_scale_y)
    f64(0.0)

    # ModelTiepointTag data (6 doubles)
    for v in tie_point:
        f64(v)

    with open(path, 'wb') as f:
        f.write(buf.getvalue())
    print("Wrote %s (%dx%d, %d bytes)" % (path, width, height, len(buf.getvalue())))


def main():
    print("Generating debug hill TIF: %dx%d, bounds [%.4f-%.4f, %.4f-%.4f]" %
          (WIDTH, HEIGHT, WEST, EAST, SOUTH, NORTH))
    print("Gaussian hill: peak=%.1fm, sigma=[%.4f, %.4f] deg" % (PEAK, SIGMA_X, SIGMA_Y))
    elev = generate_elevation()
    write_geotiff(OUT_PATH, WIDTH, HEIGHT, elev, WEST, EAST, SOUTH, NORTH)
    print("Run: TerrainConverter -i %s -o output_debug -v" % OUT_PATH)
    print("Expected: smooth hill, vertices should concentrate at peak, NOT at edges")


if __name__ == "__main__":
    main()
