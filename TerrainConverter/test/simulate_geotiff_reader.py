#!/usr/bin/env python3
"""
Full pipeline simulation: synthetic GeoTIFF with rotation+scale →
GeoTiffReader processing → correctness verification.

Creates a byte-level TIFF with ModelTransformationTag (rotation + scale),
then simulates Open/ReadModelTransform/ReadElevationGrid exactly as the
C++ code does. Verifies every pixel's geographic position against the
known ground truth.
"""

import struct, math, sys

# ============================================================
# 1. SYNTHETIC DATA GENERATION
# ============================================================

W, H = 5, 4  # small grid for easy manual verification

# Affine transform parameters (3x4 matrix)
ANGLE_DEG = 30.0
SCALE_X   = 2.0
SCALE_Y   = 1.5
ORIGIN_X  = 100.0
ORIGIN_Y  = 200.0

ang = math.radians(ANGLE_DEG)
cos_a, sin_a = math.cos(ang), math.sin(ang)

# ModelTransformation 3x4 matrix (a..h):
# X = d + a*col + b*row
# Y = h + e*col + f*row
a =  SCALE_X * cos_a
b = -SCALE_Y * sin_a           # b = row contribution to X (south-up if sign causes Y decrease?)
c =  0.0                         # unused (Z)
d = ORIGIN_X
e =  SCALE_X * sin_a
f =  SCALE_Y * cos_a
g =  0.0                         # unused (Z)
h = ORIGIN_Y

# Elevation: ramp pattern for easy visual verification
def elevation(row, col):
    return float(row * 10 + col * 5)

elevations = [[elevation(r, c) for c in range(W)] for r in range(H)]

# Geographic position of each pixel center (ground truth)
def geo_pos(row, col):
    """Returns (X, Y) using the matrix with pixel CENTER convention.
    Pixel center at (col + 0.5, row + 0.5) in pixel coordinates."""
    pc = col + 0.5  # pixel-center column
    pr = row + 0.5  # pixel-center row
    x = d + a * pc + b * pr
    y = h + e * pc + f * pr
    return x, y

# Geographic corners of the grid (pixel-corner convention)
corners_px = [
    (0.0, 0.0),          # top-left corner
    (W, 0.0),             # top-right corner
    (0.0, H),             # bottom-left corner
    (W, H),               # bottom-right corner
]
geo_corners = [(d + a*cx + b*ry, h + e*cx + f*ry) for cx, ry in corners_px]

# ============================================================
# 2. BUILD SYNTHETIC TIFF (byte-level)
# ============================================================

def pack_u16(v): return struct.pack('<H', v)
def pack_u32(v): return struct.pack('<I', v)
def pack_f32(v): return struct.pack('<f', v)
def pack_f64(v): return struct.pack('<d', v)

num_ifd_entries = 14
ifd_size = 2 + num_ifd_entries * 12 + 4  # count + entries + next-IFD

# Layout
header_sz = 8
strip_sz = W * H * 4  # float32
matrix_sz = 16 * 8    # 16 doubles (full 4x4 for ModelTransformationTag)
geokey_sz = (1 + 4) * 4 * 2  # header + 4 keys, each 4*uint16

strip_off  = header_sz
matrix_off = strip_off + strip_sz
geokey_off = matrix_off + matrix_sz
ifd_off    = geokey_off + geokey_sz

buf = bytearray()

# TIFF header
buf += b'II'
buf += pack_u16(42)
buf += pack_u32(ifd_off)

# Image data (row-major float32, file order = row 0 is top)
for r in range(H):
    for c in range(W):
        buf += pack_f32(elevations[r][c])

# ModelTransformationTag (3x4 matrix as 16 doubles)
# [a,b,c,d,  e,f,g,h,  i,j,k,l,  m,n,o,p]
buf += pack_f64(a)
buf += pack_f64(b)
buf += pack_f64(c)
buf += pack_f64(d)
buf += pack_f64(e)
buf += pack_f64(f)
buf += pack_f64(g)
buf += pack_f64(h)
buf += pack_f64(0.0) * 8  # i..p = zeros

# GeoKeyDirectory: header(4) + 4 keys
# Keys: GTModelType=Projected, GTRasterType=PixelIsPoint,
#        ProjectedCSType=32650 (UTM zone 50N), ProjLinearUnits=metre
# NOTE: GTRasterTypeGeoKey=2 means RasterPixelIsPoint → tiepoint is at pixel center
num_keys = 4
buf += pack_u16(1)     # KeyDirVersion
buf += pack_u16(1)     # KeyRev
buf += pack_u16(0)     # MinorRev
buf += pack_u16(num_keys)
# Key 1: GTModelTypeGeoKey=Projected(1)
buf += pack_u16(1024); buf += pack_u16(0); buf += pack_u16(1); buf += pack_u16(1)
# Key 2: GTRasterTypeGeoKey=2 (RasterPixelIsPoint)
buf += pack_u16(1025); buf += pack_u16(0); buf += pack_u16(1); buf += pack_u16(2)
# Key 3: ProjectedCSTypeGeoKey=32650
buf += pack_u16(3072); buf += pack_u16(0); buf += pack_u16(1); buf += pack_u16(32650)
# Key 4: ProjLinearUnitsGeoKey=9001 (metre)
buf += pack_u16(3076); buf += pack_u16(0); buf += pack_u16(1); buf += pack_u16(9001)

# IFD
buf += pack_u16(num_ifd_entries)

def ifd_entry(tag, typ, count, value):
    return pack_u16(tag) + pack_u16(typ) + pack_u32(count) + pack_u32(value)

# Types: 3=SHORT, 4=LONG, 12=DOUBLE
entries = []
entries.append(ifd_entry(256, 3, 1, W))           # ImageWidth
entries.append(ifd_entry(257, 3, 1, H))           # ImageLength
entries.append(ifd_entry(258, 3, 1, 32))           # BitsPerSample
entries.append(ifd_entry(259, 3, 1, 1))            # Compression=None
entries.append(ifd_entry(262, 3, 1, 1))            # PhotometricInterpretation
entries.append(ifd_entry(273, 4, 1, strip_off))    # StripOffsets
entries.append(ifd_entry(277, 3, 1, 1))            # SamplesPerPixel
entries.append(ifd_entry(278, 3, 1, H))            # RowsPerStrip
entries.append(ifd_entry(279, 4, 1, strip_sz))     # StripByteCounts
entries.append(ifd_entry(339, 3, 1, 3))            # SampleFormat=IEEEFP
entries.append(ifd_entry(34735, 3, (1+num_keys)*4, geokey_off))  # GeoKeyDir
entries.append(ifd_entry(34264, 12, 16, matrix_off))              # ModelTransformationTag
# Also include ModelTiepoint and ModelPixelScale (should be overridden by matrix)
entries.append(ifd_entry(33550, 12, 3, matrix_off))  # same offset (won't be read)
entries.append(ifd_entry(33922, 12, 6, matrix_off))  # same offset (won't be read)

for ent in entries:
    buf += ent
buf += pack_u32(0)  # next IFD = none

tif_path = '/tmp/test_rotated.tif'
with open(tif_path, 'wb') as out_f:
    out_f.write(buf)

print(f"Generated: {tif_path} ({len(buf)} bytes, {W}x{H} float32)")
print(f"Matrix: a={a:.6f} b={b:.6f} d={d:.6f}")
print(f"        e={e:.6f} f={f:.6f} h={h:.6f}")
print(f"Scale: X={SCALE_X}, Y={SCALE_Y}, Rotation: {ANGLE_DEG}°")
print(f"GeoKey: PixelIsPoint, EPSG:32650 (UTM 50N)")
print()

# ============================================================
# 3. SIMULATE GeoTiffReader::Open
# ============================================================

with open(tif_path, 'rb') as tf:
    # Read header
    hdr = tf.read(8)
    bo = '<' if hdr[:2] == b'II' else '>'
    ifd_off_read = struct.unpack(f'{bo}I', hdr[4:8])[0]

    # Read IFD
    tf.seek(ifd_off_read)
    num_e = struct.unpack(f'{bo}H', tf.read(2))[0]

    tags = {}
    for i in range(num_e):
        tf.seek(ifd_off_read + 2 + i * 12)
        tag, typ, cnt, val = struct.unpack(f'{bo}HHII', tf.read(12))
        tags[tag] = (typ, cnt, val)

    width  = tags[256][2]
    height = tags[257][2]
    bps    = tags[258][2]
    sf     = tags[339][2]
    spp    = tags[277][2]

    print(f"[Open] W={width} H={height} bps={bps} sf={sf} spp={spp}")

    # ============================================================
    # 4. SIMULATE GeoTiffReader::ReadModelTransform
    # ============================================================

    # Read ModelTransformationTag (tag 34264)
    if 34264 in tags:
        _, _, mat_off = tags[34264]
        tf.seek(mat_off)
        mat = struct.unpack(f'{bo}16d', tf.read(128))
        # Extract a,b,d,e,f,h from 16-element matrix
        matrix_a = mat[0]   # a = dX/dCol
        matrix_b = mat[1]   # b = dX/dRow
        matrix_d = mat[3]   # d = translation X
        matrix_e = mat[4]   # e = dY/dCol
        matrix_f = mat[5]   # f = dY/dRow
        matrix_h = mat[7]   # h = translation Y
        has_matrix = True

        # GDAL GeoTransform
        GT = [
            matrix_d,   # [0] originX
            matrix_a,   # [1] pixelWidth
            matrix_b,   # [2] rowRotation
            matrix_h,   # [3] originY
            matrix_e,   # [4] colRotation
            matrix_f,   # [5] pixelHeight
        ]
        print(f"[Transform] Matrix→GT: origin=({GT[0]:.6f},{GT[3]:.6f}) "
              f"pixel=({GT[1]:.6f},{GT[5]:.6f}) "
              f"rotation=({GT[2]:.6f},{GT[4]:.6f})")
    else:
        has_matrix = False
        GT = [0, 1, 0, 0, 0, -1]

    # Parse GeoKeys
    if 34735 in tags:
        _, _, gk_off = tags[34735]
        tf.seek(gk_off)
        gk_hdr = struct.unpack(f'{bo}4H', tf.read(8))
        num_keys = gk_hdr[3]
        raster_type = 0
        epsg = 0
        is_geographic = False
        model_type = 0
        linear_units = 0

        for i in range(num_keys):
            key_id, loc, cnt, val = struct.unpack(f'{bo}4H', tf.read(8))
            if key_id == 1024:   # GTModelTypeGeoKey
                model_type = val
                is_geographic = (val == 2)
            elif key_id == 1025: # GTRasterTypeGeoKey
                raster_type = val  # 1=Area, 2=Point
            elif key_id == 3072: # ProjectedCSTypeGeoKey
                if val < 32767:
                    epsg = val
                projected_type = val
            elif key_id == 3076: # ProjLinearUnitsGeoKey
                linear_units = val

        print(f"[GeoKeys] ModelType={model_type} RasterType={raster_type} "
              f"EPSG={epsg} LinearUnits={linear_units}")
    else:
        raster_type = 0
        epsg = 0
        is_geographic = False
        linear_units = 0

    # Compute vertical scale from linear units
    units_map = {9001:1.0, 9002:0.3048, 9003:1200.0/3937.0, 9036:1000.0}
    vertical_scale = units_map.get(linear_units, 1.0)
    print(f"[Units] linearUnits={linear_units} → verticalScale={vertical_scale}")

    # ============================================================
    # 5. SIMULATE GeoTiffReader::ReadElevationGrid
    # ============================================================

    # Read pixel data (strip is at strip_off, row-major float32)
    tf.seek(strip_off)
    heights = []
    for rr in range(H):
        row_data = []
        for cc in range(W):
            pixel_val = struct.unpack(f'{bo}f', tf.read(4))[0]
            row_data.append(pixel_val)
        heights.append(row_data)

    # Flatten to 1D for C++ simulation
    heights_flat = [heights[r][c] for r in range(H) for c in range(W)]

    print(f"\n[Read] Pixel values (file order):")
    for r in range(H):
        print(f"  row {r}: {heights[r]}")

    # Compute bounds from GT at pixel-corner positions
    T = GT
    dx = abs(T[1])
    dy = abs(T[5])

    # Four pixel-corner points (not centers)
    xNW = T[0]                          # c=0, r=0 corner
    yNW = T[3]
    xNE = T[0] + W * T[1]               # c=W, r=0 corner (RIGHT EDGE)
    yNE = T[3] + W * T[4]
    xSW = T[0] + H * T[2]               # c=0, r=H corner (BOTTOM EDGE)
    ySW = T[3] + H * T[5]
    xSE = T[0] + W * T[1] + H * T[2]   # c=W, r=H corner
    ySE = T[3] + W * T[4] + H * T[5]

    # Use pixel-center corners (same as C++ code uses W-1, H-1)
    # BUT: the C++ code uses (W-1) which gives pixel-center extents
    xNW_c = T[0]                             # c=0, r=0
    yNW_c = T[3]
    xNE_c = T[0] + (W-1) * T[1]             # c=W-1, r=0
    yNE_c = T[3] + (W-1) * T[4]
    xSW_c = T[0] + (H-1) * T[2]             # c=0, r=H-1
    ySW_c = T[3] + (H-1) * T[5]
    xSE_c = T[0] + (W-1) * T[1] + (H-1) * T[2]
    ySE_c = T[3] + (W-1) * T[4] + (H-1) * T[5]

    # Min/max of all four pixel-center corners (matching C++ code)
    minE = min(xNW_c, xNE_c, xSW_c, xSE_c)
    maxE = max(xNW_c, xNE_c, xSW_c, xSE_c)
    minN = min(yNW_c, yNE_c, ySW_c, ySE_c)
    maxN = max(yNW_c, yNE_c, ySW_c, ySE_c)

    print(f"\n[Bounds] Pixel CORNER corners (GDAL convention, using W,H):")
    print(f"  xNW=({xNW:.4f},{yNW:.4f})  xNE=({xNE:.4f},{yNE:.4f})")
    print(f"  xSW=({xSW:.4f},{ySW:.4f})  xSE=({xSE:.4f},{ySE:.4f})")
    print(f"  GDAL-extent: E=[{min(xNW,xNE,xSW,xSE):.4f}, {max(xNW,xNE,xSW,xSE):.4f}]")
    print(f"                N=[{min(yNW,yNE,ySW,ySE):.4f}, {max(yNW,yNE,ySW,ySE):.4f}]")
    print(f"\n[Bounds] Pixel CENTER corners (current code, using W-1, H-1):")
    print(f"  NW_c=({xNW_c:.4f},{yNW_c:.4f})  NE_c=({xNE_c:.4f},{yNE_c:.4f})")
    print(f"  SW_c=({xSW_c:.4f},{ySW_c:.4f})  SE_c=({xSE_c:.4f},{ySE_c:.4f})")
    print(f"  Code-extent: E=[{minE:.4f}, {maxE:.4f}] N=[{minN:.4f}, {maxN:.4f}]")
    print(f"  dx={dx:.6f} dy={dy:.6f}")

    # Flip normalization
    # GT[5] > 0 → south-up → need vertical flip
    # GT[1] < 0 → west-up  → need horizontal flip
    need_vflip = T[5] > 0.0
    need_hflip = T[1] < 0.0

    if need_vflip:
        for r in range(H // 2):
            r2 = H - 1 - r
            for c in range(W):
                i1 = r * W + c
                i2 = r2 * W + c
                heights_flat[i1], heights_flat[i2] = heights_flat[i2], heights_flat[i1]
        print(f"\n[Flip] Vertical flip applied (T[5]={T[5]:.6f} > 0, south-up)")

    if need_hflip:
        for r in range(H):
            for c in range(W // 2):
                c2 = W - 1 - c
                i1 = r * W + c
                i2 = r * W + c2
                heights_flat[i1], heights_flat[i2] = heights_flat[i2], heights_flat[i1]
        print(f"[Flip] Horizontal flip applied (T[1]={T[1]:.6f} < 0, west-up)")

    if not need_vflip and not need_hflip:
        print(f"\n[Flip] No flip needed (north-up, east-west)")

    print(f"\n[Heights] After flip (row 0 = north/maxNorthing):")
    for r in range(H):
        row_vals = [heights_flat[r * W + c] for c in range(W)]
        print(f"  row {r}: {row_vals}")

    # ============================================================
    # 6. VERIFY: EastingAt / NorthingAt
    # ============================================================

    def easting_at(col):
        return minE + col * dx

    def northing_at(row):
        return maxN - row * dy

    print(f"\n[EastingAt/NorthingAt] Pixel-center positions:")
    print(f"  EastingAt(0)={easting_at(0):.4f}  EastingAt({W-1})={easting_at(W-1):.4f}")
    print(f"  NorthingAt(0)={northing_at(0):.4f}  NorthingAt({H-1})={northing_at(H-1):.4f}")

    # ============================================================
    # 7. VERIFY: BilinearSample
    # ============================================================

    def height_at(row, col):
        return heights_flat[row * W + col]

    def bilinear_sample(easting, northing):
        fc = (easting - minE) / dx
        fr = (maxN - northing) / dy
        if fc < 0 or fc > W - 1 or fr < 0 or fr > H - 1:
            return None
        c0 = int(math.floor(fc))
        r0 = int(math.floor(fr))
        c1 = min(c0 + 1, W - 1)
        r1 = min(r0 + 1, H - 1)
        tc = fc - c0
        tr = fr - r0
        h00 = height_at(r0, c0)
        h10 = height_at(r0, c1)
        h01 = height_at(r1, c0)
        h11 = height_at(r1, c1)
        return h00 * (1-tc) * (1-tr) + h10 * tc * (1-tr) + h01 * (1-tc) * tr + h11 * tc * tr

    print(f"\n[BilinearSample] Verification (sampling at pixel centers):")

    # Sample at each pixel center and compare with stored value
    max_err = 0.0
    for r in range(H):
        for gc in range(W):
            # Geographic position of this pixel center
            gx = T[0] + (gc + 0.5) * T[1] + (r + 0.5) * T[2]
            gy = T[3] + (gc + 0.5) * T[4] + (r + 0.5) * T[5]

            # Sample via BilinearSample
            sampled = bilinear_sample(gx, gy)
            expected = height_at(r, gc)

            if sampled is None:
                print(f"  Pixel ({r},{gc}) at geo ({gx:.4f},{gy:.4f}): OUT OF RANGE!")
            else:
                err = abs(sampled - expected)
                if err > max_err:
                    max_err = err
                status = "OK" if err < 0.01 else "FAIL"
                if err > 0.01:
                    print(f"  Pixel ({r},{gc}) at geo ({gx:.4f},{gy:.4f}): "
                          f"expected={expected:.4f} sampled={sampled:.4f} err={err:.4f} [{status}]")

    print(f"\n[Result] Max bilinear error: {max_err:.6f}")

    # ============================================================
    # 8. VERIFY: Corner positions vs ground truth
    # ============================================================

    print(f"\n[Corner Verification] Geographic corners:")
    for (cx, cy), label in zip(corners_px, ['NW','NE','SW','SE']):
        gx = d + a * cx + b * cy
        gy = h + e * cx + f * cy
        print(f"  {label} px({cx},{cy}) → geo({gx:.4f},{gy:.4f})")

    # ============================================================
    # 9. SUMMARY
    # ============================================================

    # Verify: EastingAt(0) should be the geographic X of pixel (0,0) center
    expected_e0 = d + a * 0.5 + b * 0.5
    actual_e0 = easting_at(0)
    e0_err = abs(expected_e0 - actual_e0)

    # Verify: NorthingAt(0) should be the geographic Y of pixel (0,0) center
    expected_n0 = h + e * 0.5 + f * 0.5
    actual_n0 = northing_at(0)
    n0_err = abs(expected_n0 - actual_n0)

    print(f"\n{'='*60}")
    print(f"FINAL VERDICT")
    print(f"{'='*60}")
    print(f"EastingAt(0) vs expected pixel-0 center: "
          f"{actual_e0:.6f} vs {expected_e0:.6f} err={e0_err:.6f} "
          f"{'OK' if e0_err < 0.01 else 'FAIL'}")
    print(f"NorthingAt(0) vs expected pixel-0 center: "
          f"{actual_n0:.6f} vs {expected_n0:.6f} err={n0_err:.6f} "
          f"{'OK' if n0_err < 0.01 else 'FAIL'}")
    print(f"Bilinear error: {max_err:.6f} {'OK' if max_err < 0.01 else 'FAIL'}")

    if e0_err < 0.01 and n0_err < 0.01 and max_err < 0.01:
        print(f"\nPASS: Pipeline is internally consistent for rotated+scaled data.")
    else:
        print(f"\nFAIL: Pipeline has coordinate misalignment for rotated+scaled data.")
        # Diagnose the failure
        print(f"\nDiagnosis:")
        print(f"  Expected pixel(0,0) center: ({expected_e0:.4f}, {expected_n0:.4f})")
        print(f"  EastingAt(0)={actual_e0:.4f} NorthingAt(0)={actual_n0:.4f}")
        print(f"  minEasting={minE:.4f} maxEasting={maxE:.4f}")
        print(f"  minNorthing={minN:.4f} maxNorthing={maxN:.4f}")
        print(f"  This error occurs because minEasting is the pixel-CORNER,")
        print(f"  but EastingAt(0) returns a value expected to be a pixel-CENTER.")
        print(f"  Fix: EastingAt(col) = minEasting + 0.5*dx + col*dx")
        print(f"       NorthingAt(row) = maxNorthing - 0.5*dy - row*dy")

    # Clean up
    import os
    os.unlink(tif_path)
