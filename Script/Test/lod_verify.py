"""LOD spec-compliance verification: scan all levels in output_lod/."""
import struct, os, glob

def read_varint(data, off):
    result = 0
    shift = 0
    while True:
        if off >= len(data):
            raise ValueError("varint overflow at " + str(off))
        b = data[off]
        off += 1
        result |= (b & 0x7F) << shift
        if (b & 0x80) == 0:
            break
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
        return None, "file too small"

    cx, cy, cz = struct.unpack("<ddd", data[0:24])
    min_h, max_h = struct.unpack("<ff", data[24:32])
    bsx, bsy, bsz, bsr = struct.unpack("<dddd", data[32:64])
    hopx, hopy, hopz = struct.unpack("<ddd", data[64:88])

    off = 88
    info = {
        "center": (cx, cy, cz),
        "min_h": min_h, "max_h": max_h,
        "bs_center": (bsx, bsy, bsz), "bs_radius": bsr,
        "hop": (hopx, hopy, hopz),
        "size": len(data),
    }

    try:
        n_verts, off = read_u32_le(data, off)
        info["n_verts"] = n_verts

        for field in ("u", "v", "h"):
            vals = []
            prev = 0
            for _ in range(n_verts):
                z = struct.unpack("<H", data[off:off+2])[0]
                off += 2
                delta = zigzag_decode(z)
                cur = prev + delta
                vals.append(cur)
                prev = cur
            info[field + "_range"] = (min(vals), max(vals)) if vals else (0, 0)

        n_tris, off = read_u32_le(data, off)
        info["n_tris"] = n_tris

        indices = []
        hwm = 0
        for _ in range(n_tris * 3):
            z, off = read_varint(data, off)
            delta = zigzag_decode(z)
            idx = hwm + delta
            indices.append(idx)
            if delta > 0:
                hwm = idx + 1

        info["bad_idx"] = sum(1 for i in indices if i >= n_verts)

        for _ in range(4):
            n_e, off = read_u32_le(data, off)
            for _ in range(n_e):
                _, off = read_varint(data, off)

        info["extensions"] = []
        while off < len(data):
            ext_id = data[off]
            ext_len, off = read_u32_le(data, off + 1)
            off += ext_len
            info["extensions"].append(ext_id)

        info["consumed"] = off
        info["parse_ok"] = (off == len(data))
        return info, None
    except Exception as e:
        return info, "parse error: " + str(e) + " at " + str(off)


base = sys.argv[1] if len(sys.argv) > 1 else "."
all_tiles = []
for path in glob.glob(os.path.join(base, "*", "*", "*.terrain")):
    parts = path.replace("\\", "/").split("/")
    level = int(parts[-3])
    x = int(parts[-2])
    y = int(parts[-1].replace(".terrain", ""))
    info, err = parse_tile(path)
    if not err:
        all_tiles.append((level, x, y, info))

print("=" * 70)
print("LOD spec compliance — " + str(len(all_tiles)) + " tiles across all levels")
print("=" * 70)

# Per-level stats
for lv in sorted(set(t[0] for t in all_tiles)):
    lv_tiles = [t for t in all_tiles if t[0] == lv]
    full_parse = sum(1 for _, _, _, i in lv_tiles if i["parse_ok"])
    valid_idx = sum(1 for _, _, _, i in lv_tiles if i["bad_idx"] == 0)
    valid_uv = sum(1 for _, _, _, i in lv_tiles
                   if 0 <= i["u_range"][0] and i["u_range"][1] <= 32767
                   and 0 <= i["v_range"][0] and i["v_range"][1] <= 32767)
    valid_h = sum(1 for _, _, _, i in lv_tiles
                  if 0 <= i["h_range"][0] and i["h_range"][1] <= 32767)
    has_ext = sum(1 for _, _, _, i in lv_tiles if i["extensions"])
    nodata_leak = sum(1 for _, _, _, i in lv_tiles if i["min_h"] <= -1000)

    verts = [i["n_verts"] for _, _, _, i in lv_tiles]
    tris = [i["n_tris"] for _, _, _, i in lv_tiles]
    print("\nLevel " + str(lv) + " (" + str(len(lv_tiles)) + " tiles):")
    print("  Parse OK:           " + str(full_parse) + "/" + str(len(lv_tiles)))
    print("  Valid indices:      " + str(valid_idx) + "/" + str(len(lv_tiles)))
    print("  u/v in [0,32767]:   " + str(valid_uv) + "/" + str(len(lv_tiles)))
    print("  h in [0,32767]:     " + str(valid_h) + "/" + str(len(lv_tiles)))
    print("  Has extension:      " + str(has_ext) + "/" + str(len(lv_tiles)))
    print("  noData leak in min: " + str(nodata_leak) + "/" + str(len(lv_tiles)))
    if verts:
        print("  Verts: min=" + str(min(verts)) + " max=" + str(max(verts))
              + " mean=" + format(sum(verts)/len(verts), ".1f"))
        print("  Tris:  min=" + str(min(tris)) + " max=" + str(max(tris))
              + " mean=" + format(sum(tris)/len(tris), ".1f"))

# Aggregate
print("\n" + "=" * 70)
print("Aggregate across all " + str(len(all_tiles)) + " tiles")
print("=" * 70)
full = sum(1 for _, _, _, i in all_tiles if i["parse_ok"])
valid = sum(1 for _, _, _, i in all_tiles if i["parse_ok"] and i["bad_idx"] == 0)
print("Full parse:      " + str(full) + "/" + str(len(all_tiles)))
print("Valid indices:   " + str(valid) + "/" + str(len(all_tiles)))

# Sample tiles from each level
print("\nSample tiles per level:")
for lv in sorted(set(t[0] for t in all_tiles)):
    sample = next(t for t in all_tiles if t[0] == lv)
    _, x, y, i = sample
    print("  L" + str(lv) + "/" + str(x) + "/" + str(y)
          + ": verts=" + str(i["n_verts"])
          + " tris=" + str(i["n_tris"])
          + " h=[" + format(i["min_h"], ".1f") + "," + format(i["max_h"], ".1f") + "]m"
          + " bs_r=" + format(i["bs_radius"], ".0f"))
