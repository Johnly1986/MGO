"""Check for degenerate triangles (zero area / duplicate vertices) and investigate tiny tiles."""
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
    with open(path, "rb") as f:
        data = f.read()
    if len(data) < 88:
        return None
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
    if range_h < 1e-6:
        range_h = 1.0
    h_meters = [min_h + (hq / 32767.0) * range_h for hq in h_vals]
    n_tris, off = read_u32_le(data, off)
    indices = []; hwm = 0
    for _ in range(n_tris * 3):
        z, off = read_varint(data, off)
        delta = zigzag_decode(z); idx = hwm + delta; indices.append(idx)
        if delta > 0:
            hwm = idx + 1
    return {"n_verts": n_verts, "n_tris": n_tris, "u": u_vals, "v": v_vals, "h": h_meters,
            "min_h": min_h, "max_h": max_h, "indices": indices}

base = sys.argv[1] if len(sys.argv) > 1 else "."
all_tiles = []
for path in glob.glob(os.path.join(base, "*", "*", "*.terrain")):
    parts = path.replace("\\", "/").split("/")
    level = int(parts[-3]); x = int(parts[-2]); y = int(parts[-1].replace(".terrain", ""))
    info = parse_tile(path)
    if info:
        all_tiles.append((level, x, y, path, info))

print("=" * 70)
print("Degenerate triangle check (zero area / duplicate vertices)")
print("=" * 70)
degenerate_count = 0
total_tris = 0
for level, x, y, path, info in all_tiles:
    u = info["u"]; v = info["v"]; idx = info["indices"]
    for t in range(0, len(idx), 3):
        i0, i1, i2 = idx[t], idx[t+1], idx[t+2]
        total_tris += 1
        if i0 == i1 or i1 == i2 or i0 == i2:
            degenerate_count += 1
            if degenerate_count <= 5:
                print("  Degenerate (dup idx): L{}/{}/{} tri {}: {},{},{}".format(level, x, y, t//3, i0, i1, i2))
            continue
        area = abs((u[i1]-u[i0])*(v[i2]-v[i0]) - (u[i2]-u[i0])*(v[i1]-v[i0]))
        if area == 0:
            degenerate_count += 1
            if degenerate_count <= 5:
                print("  Degenerate (zero area): L{}/{}/{} tri {}: {},{},{}".format(level, x, y, t//3, i0, i1, i2))

print()
print("Total triangles: {}".format(total_tris))
print("Degenerate triangles: {}".format(degenerate_count))
print("Valid triangles: {}".format(total_tris - degenerate_count))

print()
print("=" * 70)
print("Tiny tiles investigation (<10 triangles)")
print("=" * 70)
for level, x, y, path, info in sorted(all_tiles):
    if info["n_tris"] < 10:
        tiles_x = 2 * (1 << level)
        tiles_y = (1 << level)
        tile_w = 360.0 / tiles_x
        tile_h = 180.0 / tiles_y
        west = -180.0 + x * tile_w
        east = west + tile_w
        south = -90.0 + y * tile_h
        north = south + tile_h
        print("  L{}/{}/{}: {} verts, {} tris".format(level, x, y, info["n_verts"], info["n_tris"]))
        print("    Bounds: lon[{:.4f}, {:.4f}] lat[{:.4f}, {:.4f}]".format(west, east, south, north))
        print("    Height: [{:.1f}, {:.1f}]m".format(info["min_h"], info["max_h"]))
        print("    u range: [{}, {}] (0-32767)".format(min(info["u"]), max(info["u"])))
        print("    v range: [{}, {}] (0-32767)".format(min(info["v"]), max(info["v"])))
