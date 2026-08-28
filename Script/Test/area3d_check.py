"""Detailed 3D area check: distinguish truly degenerate (zero 3D area) from vertical (zero 2D, non-zero 3D)."""
import struct, os, glob, math

def read_varint(data, off):
    result = 0; shift = 0
    while True:
        b = data[off]; off += 1
        result |= (b & 0x7F) << shift
        if (b & 0x80) == 0: break
        shift += 7
    return result, off

def read_u32_le(data, off):
    return struct.unpack("<I", data[off:off+4])[0], off + 4

def zigzag_decode(z):
    return (z >> 1) ^ -(z & 1)

def parse_tile(path):
    with open(path, "rb") as f: data = f.read()
    min_h, max_h = struct.unpack("<ff", data[24:32])
    off = 88
    n_verts, off = read_u32_le(data, off)
    u_vals = []; prev = 0
    for _ in range(n_verts):
        z = struct.unpack("<H", data[off:off+2])[0]; off += 2
        delta = zigzag_decode(z); cur = prev + delta; u_vals.append(cur); prev = cur
    v_vals = []; prev = 0
    for _ in range(n_verts):
        z = struct.unpack("<H", data[off:off+2])[0]; off += 2
        delta = zigzag_decode(z); cur = prev + delta; v_vals.append(cur); prev = cur
    h_vals = []; prev = 0
    for _ in range(n_verts):
        z = struct.unpack("<H", data[off:off+2])[0]; off += 2
        delta = zigzag_decode(z); cur = prev + delta; h_vals.append(cur); prev = cur
    range_h = max_h - min_h
    if range_h < 1e-6: range_h = 1.0
    h_meters = [min_h + (hq / 32767.0) * range_h for hq in h_vals]
    n_tris, off = read_u32_le(data, off)
    indices = []; hwm = 0
    for _ in range(n_tris * 3):
        z, off = read_varint(data, off)
        delta = zigzag_decode(z); idx = hwm + delta; indices.append(idx)
        if delta > 0: hwm = idx + 1
    return {"n_verts": n_verts, "n_tris": n_tris, "u": u_vals, "v": v_vals, "h": h_meters,
            "min_h": min_h, "max_h": max_h, "indices": indices}

base = sys.argv[1] if len(sys.argv) > 1 else "."
all_tiles = []
for path in glob.glob(os.path.join(base, "*", "*", "*.terrain")):
    parts = path.replace("\\", "/").split("/")
    level = int(parts[-3]); x = int(parts[-2]); y = int(parts[-1].replace(".terrain", ""))
    info = parse_tile(path)
    if info: all_tiles.append((level, x, y, path, info))

print("=" * 70)
print("3D area check: truly degenerate vs vertical triangles")
print("=" * 70)

truly_degenerate = 0   # zero 3D area (duplicate vertices or collinear in 3D)
vertical = 0           # zero 2D area but non-zero 3D area (vertical triangles)
valid = 0
total = 0

for level, x, y, path, info in all_tiles:
    u = info["u"]; v = info["v"]; h = info["h"]; idx = info["indices"]
    for t in range(0, len(idx), 3):
        i0, i1, i2 = idx[t], idx[t+1], idx[t+2]
        total += 1

        # 2D area (u-v plane)
        area_2d = abs((u[i1]-u[i0])*(v[i2]-v[i0]) - (u[i2]-u[i0])*(v[i1]-v[i0]))

        # 3D area using cross product
        e1x = u[i1]-u[i0]; e1y = v[i1]-v[i0]; e1z = h[i1]-h[i0]
        e2x = u[i2]-u[i0]; e2y = v[i2]-v[i0]; e2z = h[i2]-h[i0]
        cx = e1y*e2z - e1z*e2y
        cy = e1z*e2x - e1x*e2z
        cz = e1x*e2y - e1y*e2x
        area_3d = math.sqrt(cx*cx + cy*cy + cz*cz)

        if area_3d < 1e-6:
            truly_degenerate += 1
        elif area_2d == 0:
            vertical += 1
        else:
            valid += 1

print("Total triangles:           {}".format(total))
print("Truly degenerate (0 3D):   {} ({:.1f}%)".format(truly_degenerate, 100*truly_degenerate/total))
print("Vertical (0 2D, >0 3D):    {} ({:.1f}%)".format(vertical, 100*vertical/total))
print("Valid (non-zero 2D & 3D):  {} ({:.1f}%)".format(valid, 100*valid/total))

# Detailed look at L10/1610/655
print()
print("=" * 70)
print("Detailed: L10/1610/655")
print("=" * 70)
for level, x, y, path, info in all_tiles:
    if level == 10 and x == 1610 and y == 655:
        u = info["u"]; v = info["v"]; h = info["h"]; idx = info["indices"]
        print("Vertices ({} total):".format(info["n_verts"]))
        for i in range(min(info["n_verts"], 20)):
            print("  [{}] u={}, v={}, h={:.1f}m".format(i, u[i], v[i], h[i]))
        print()
        print("Triangles ({} total):".format(info["n_tris"]))
        for t in range(0, len(idx), 3):
            i0, i1, i2 = idx[t], idx[t+1], idx[t+2]
            area_2d = abs((u[i1]-u[i0])*(v[i2]-v[i0]) - (u[i2]-u[i0])*(v[i1]-v[i0]))
            e1x = u[i1]-u[i0]; e1y = v[i1]-v[i0]; e1z = h[i1]-h[i0]
            e2x = u[i2]-u[i0]; e2y = v[i2]-v[i0]; e2z = h[i2]-h[i0]
            cx = e1y*e2z - e1z*e2y; cy = e1z*e2x - e1x*e2z; cz = e1x*e2y - e1y*e2x
            area_3d = math.sqrt(cx*cx + cy*cy + cz*cz)
            status = "TRULY DEGEN" if area_3d < 1e-6 else ("VERTICAL" if area_2d == 0 else "OK")
            print("  tri {}: [{},{},{}] 2D={} 3D={:.4f} {}".format(
                t//3, i0, i1, i2, area_2d, area_3d, status))
        break
