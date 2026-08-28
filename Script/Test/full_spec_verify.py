"""Final spec-compliance check: read extension data too."""
import struct, os

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

def parse_tile_full(path):
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

        # u
        u_vals = []
        prev = 0
        for _ in range(n_verts):
            z = struct.unpack("<H", data[off:off+2])[0]
            off += 2
            delta = zigzag_decode(z)
            cur = prev + delta
            u_vals.append(cur)
            prev = cur

        # v
        v_vals = []
        prev = 0
        for _ in range(n_verts):
            z = struct.unpack("<H", data[off:off+2])[0]
            off += 2
            delta = zigzag_decode(z)
            cur = prev + delta
            v_vals.append(cur)
            prev = cur

        # h
        h_vals = []
        prev = 0
        for _ in range(n_verts):
            z = struct.unpack("<H", data[off:off+2])[0]
            off += 2
            delta = zigzag_decode(z)
            cur = prev + delta
            h_vals.append(cur)
            prev = cur

        info["u_range"] = (min(u_vals), max(u_vals)) if u_vals else (0, 0)
        info["v_range"] = (min(v_vals), max(v_vals)) if v_vals else (0, 0)
        info["h_quant_range"] = (min(h_vals), max(h_vals)) if h_vals else (0, 0)

        # n_tris
        n_tris, off = read_u32_le(data, off)
        info["n_tris"] = n_tris

        # triangle indices
        indices = []
        hwm = 0
        for _ in range(n_tris * 3):
            z, off = read_varint(data, off)
            delta = zigzag_decode(z)
            idx = hwm + delta
            indices.append(idx)
            if delta > 0:
                hwm = idx + 1

        bad_idx = sum(1 for i in indices if i >= n_verts)
        info["bad_idx_count"] = bad_idx
        info["idx_max"] = max(indices) if indices else 0

        # edges
        edges = []
        for _ in range(4):
            n_e, off = read_u32_le(data, off)
            edge_list = []
            for _ in range(n_e):
                v, off = read_varint(data, off)
                edge_list.append(v)
            edges.append(edge_list)
        info["edges"] = [len(e) for e in edges]
        bad_edge = sum(1 for e in edges for v in e if v >= n_verts)
        info["bad_edge_count"] = bad_edge

        # Extension: 1 byte id + uint32 LE length + data
        info["extensions"] = []
        while off < len(data):
            ext_id = data[off]
            ext_len, off = read_u32_le(data, off + 1)
            ext_data = data[off:off + ext_len]
            off += ext_len
            info["extensions"].append({
                "id": ext_id,
                "len": ext_len,
                "data_consumed": len(ext_data),
            })

        info["consumed"] = off
        info["parse_ok"] = (off == len(data))
        return info, None
    except Exception as e:
        return info, "parse error: " + str(e) + " at " + str(off)


base = sys.argv[1] if len(sys.argv) > 1 else "."
all_tiles = []
for x in range(1610, 1621):
    for y in range(653, 663):
        p = os.path.join(base, str(x), str(y) + ".terrain")
        if os.path.exists(p):
            info, err = parse_tile_full(p)
            if not err:
                all_tiles.append((x, y, info))

print("=" * 80)
print("Final spec compliance (with extension data parsing)")
print("=" * 80)

# Detailed view of 3 sample tiles
for x, y, desc in [(1612, 655, "valid interior"), (1613, 659, "noData interior"), (1610, 655, "all-noData edge")]:
    for tx, ty, info in all_tiles:
        if tx == x and ty == y:
            print()
            print("Tile 10/" + str(x) + "/" + str(y) + " - " + desc)
            print("  Header (88 bytes):")
            print("    Center ECEF: (" + format(info["center"][0], ".1f") + ", " + format(info["center"][1], ".1f") + ", " + format(info["center"][2], ".1f") + ")")
            print("    Height: " + format(info["min_h"], ".2f") + " - " + format(info["max_h"], ".2f") + " m")
            print("    BS center: (" + format(info["bs_center"][0], ".1f") + ", " + format(info["bs_center"][1], ".1f") + ", " + format(info["bs_center"][2], ".1f") + ")")
            print("    BS radius: " + format(info["bs_radius"], ".1f") + " m")
            print("    HOP: (" + format(info["hop"][0], ".1f") + ", " + format(info["hop"][1], ".1f") + ", " + format(info["hop"][2], ".1f") + ")")
            print("  Vertex data:")
            print("    n_verts = " + str(info["n_verts"]))
            print("    u range: " + str(info["u_range"]) + " (spec: 0-32767)")
            print("    v range: " + str(info["v_range"]) + " (spec: 0-32767)")
            print("    h quant range: " + str(info["h_quant_range"]) + " (spec: 0-32767)")
            print("  Triangles: n_tris = " + str(info["n_tris"]) + ", max idx = " + str(info["idx_max"]) + " (n_verts=" + str(info["n_verts"]) + ")")
            print("  Edges (W,S,E,N): " + str(info["edges"]))
            print("  Extensions: " + str([(e["id"], e["len"]) for e in info["extensions"]]))
            print("  Consumed: " + str(info["consumed"]) + " / " + str(info["size"]) + " bytes")
            ok = (info["parse_ok"] and info["bad_idx_count"] == 0 and info["bad_edge_count"] == 0
                  and 0 <= info["u_range"][0] and info["u_range"][1] <= 32767
                  and 0 <= info["v_range"][0] and info["v_range"][1] <= 32767
                  and 0 <= info["h_quant_range"][0] and info["h_quant_range"][1] <= 32767)
            print("  Spec compliance: " + ("PASS" if ok else "FAIL"))
            break

# Aggregate stats
print()
print("=" * 80)
print("Aggregate stats across all 74 tiles")
print("=" * 80)

full_parse = sum(1 for _, _, i in all_tiles if i["parse_ok"])
valid_idx = sum(1 for _, _, i in all_tiles if i["bad_idx_count"] == 0)
valid_edge = sum(1 for _, _, i in all_tiles if i["bad_edge_count"] == 0)
valid_uv = sum(1 for _, _, i in all_tiles
               if 0 <= i["u_range"][0] and i["u_range"][1] <= 32767
               and 0 <= i["v_range"][0] and i["v_range"][1] <= 32767)
valid_h = sum(1 for _, _, i in all_tiles
              if 0 <= i["h_quant_range"][0] and i["h_quant_range"][1] <= 32767)
has_ext = sum(1 for _, _, i in all_tiles if i["extensions"])

print("Full parse (consumed == total):    " + str(full_parse) + "/74")
print("Valid triangle indices:            " + str(valid_idx) + "/74")
print("Valid edge indices:                " + str(valid_edge) + "/74")
print("u/v in valid range [0, 32767]:     " + str(valid_uv) + "/74")
print("h quantized in valid range:        " + str(valid_h) + "/74")
print("Has OctVertexNormals extension:    " + str(has_ext) + "/74")

print()
print("Height distribution:")
valid_h_tiles = [(x, y, i) for x, y, i in all_tiles if i["min_h"] > -1000]
nodata_tiles = [(x, y, i) for x, y, i in all_tiles if i["min_h"] <= -1000]
print("  Tiles with valid heights: " + str(len(valid_h_tiles)) + "/74")
print("  Tiles with noData leaked into min_h: " + str(len(nodata_tiles)) + "/74")

if valid_h_tiles:
    mins = [i["min_h"] for _, _, i in valid_h_tiles]
    maxs = [i["max_h"] for _, _, i in valid_h_tiles]
    print("  Valid tiles height range: " + format(min(mins), ".1f") + " - " + format(max(maxs), ".1f") + " m")
    print("  (Yunnan expected: ~100-3000 m)")

print()
print("Vertex/triangle stats:")
verts = [i["n_verts"] for _, _, i in all_tiles]
tris = [i["n_tris"] for _, _, i in all_tiles]
print("  Verts per tile: min=" + str(min(verts)) + " max=" + str(max(verts)) + " mean=" + format(sum(verts)/len(verts), ".1f"))
print("  Tris per tile:  min=" + str(min(tris)) + " max=" + str(max(tris)) + " mean=" + format(sum(tris)/len(tris), ".1f"))

# Edge consistency check: West + East should equal North + South for a closed tile
print()
print("Edge vertex counts (should be consistent across tiles):")
for x, y, i in all_tiles[:5]:
    print("  10/" + str(x) + "/" + str(y) + ": W=" + str(i["edges"][0]) + " S=" + str(i["edges"][1]) + " E=" + str(i["edges"][2]) + " N=" + str(i["edges"][3]))
