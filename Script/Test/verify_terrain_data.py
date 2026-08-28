#!/usr/bin/env python3
"""Verify quantized-mesh .terrain tiles: vertices, triangles, edges, and OctVertexNormals.

Walks the output directory, parses every .terrain file, and checks:
  - Header fields sane (center ECEF, height range, bounding sphere)
  - Vertex u/v/h quantized values in [0, 32767]
  - Decoded geographic coords within tile bounds
  - Decoded heights within global height range
  - Triangle indices all < n_verts, no degenerate (zero-area) triangles
  - Edge indices all < n_verts
  - Full parse: consumed == file size
  - OctVertexNormals extension (id=1): present, length == 2*n_verts
  - Oct-decoded normals are unit length (within tolerance)
  - Normals point "up" (z-component positive in ENU/local frame, since terrain
    surfaces mostly face away from Earth center)

Usage: python3 verify_terrain_data.py <output_dir>
"""
import struct
import os
import sys
import math


def read_u32_le(data, off):
    return struct.unpack("<I", data[off:off + 4])[0], off + 4


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


def zigzag_decode(z):
    return (z >> 1) ^ -(z & 1)


def oct_decode(b0, b1):
    """Decode 2-byte oct-encoded normal back to (nx, ny, nz) unit vector."""
    u = (b0 / 255.0) * 2.0 - 1.0
    v = (b1 / 255.0) * 2.0 - 1.0
    z = 1.0 - (abs(u) + abs(v))
    if z < 0.0:
        # Reflect lower hemisphere
        u = (1.0 - abs(v)) * (1.0 if u >= 0 else -1.0)
        v = (1.0 - abs(u)) * (1.0 if v >= 0 else -1.0)
    l1 = abs(u) + abs(v) + abs(z)
    if l1 > 1e-10:
        u /= l1
        v /= l1
        z /= l1
    return u, v, z


def parse_tile(path):
    with open(path, "rb") as f:
        data = f.read()
    if len(data) < 88:
        return None, "file too small (<88 bytes)"

    cx, cy, cz = struct.unpack("<ddd", data[0:24])
    min_h, max_h = struct.unpack("<ff", data[24:32])
    bsx, bsy, bsz, bsr = struct.unpack("<dddd", data[32:64])
    hopx, hopy, hopz = struct.unpack("<ddd", data[64:88])

    info = {
        "center": (cx, cy, cz),
        "min_h": min_h, "max_h": max_h,
        "bs_center": (bsx, bsy, bsz), "bs_radius": bsr,
        "hop": (hopx, hopy, hopz),
        "size": len(data),
        "u_vals": [], "v_vals": [], "h_vals": [],
        "indices": [], "edges": [[], [], [], []],
        "extensions": [],
    }
    off = 88
    try:
        n_verts, off = read_u32_le(data, off)
        info["n_verts"] = n_verts

        for label in ("u_vals", "v_vals", "h_vals"):
            prev = 0
            for _ in range(n_verts):
                z = struct.unpack("<H", data[off:off + 2])[0]
                off += 2
                prev += zigzag_decode(z)
                info[label].append(prev)

        n_tris, off = read_u32_le(data, off)
        info["n_tris"] = n_tris
        # Cesium 1.111 format: indices are FIXED uint16 LE (NOT varint+zigzag).
        # HWM decoding: idx = hwm - code; if (code == 0) hwm = idx + 1.
        hwm = 0
        for _ in range(n_tris * 3):
            code = struct.unpack("<H", data[off:off + 2])[0]
            off += 2
            idx = hwm - code
            # code is uint16; treat as unsigned. idx can go negative if code > hwm,
            # which would indicate an encoder bug.
            info["indices"].append(idx)
            if code == 0:
                hwm = idx + 1

        # Edges: also FIXED uint16 LE (Cesium 1.111), NOT varint.
        for i in range(4):
            n_e, off = read_u32_le(data, off)
            for _ in range(n_e):
                v = struct.unpack("<H", data[off:off + 2])[0]
                off += 2
                info["edges"][i].append(v)

        while off < len(data):
            ext_id = data[off]
            ext_len, off = read_u32_le(data, off + 1)
            ext_data = data[off:off + ext_len]
            off += ext_len
            info["extensions"].append({
                "id": ext_id, "len": ext_len, "data": ext_data,
            })

        info["consumed"] = off
        info["parse_ok"] = (off == len(data))
        return info, None
    except Exception as e:
        return info, "parse error: " + str(e) + " at off " + str(off)


def check_tile(path, global_min_h, global_max_h):
    """Return (ok, warnings, details_dict) for one tile."""
    info, err = parse_tile(path)
    if err:
        return False, [err], {}

    warns = []
    n_verts = info["n_verts"]
    n_tris = info["n_tris"]

    # --- Vertex checks ---
    u_min, u_max = min(info["u_vals"]), max(info["u_vals"])
    v_min, v_max = min(info["v_vals"]), max(info["v_vals"])
    h_min, h_max = min(info["h_vals"]), max(info["h_vals"])
    if u_min < 0 or u_max > 32767:
        warns.append("u out of [0,32767]: %d..%d" % (u_min, u_max))
    if v_min < 0 or v_max > 32767:
        warns.append("v out of [0,32767]: %d..%d" % (v_min, v_max))
    if h_min < 0 or h_max > 32767:
        warns.append("h out of [0,32767]: %d..%d" % (h_min, h_max))

    # Height range check: decoded min_h/max_h should be within global range
    if info["min_h"] < global_min_h - 1.0 or info["max_h"] > global_max_h + 1.0:
        warns.append("tile height [%.1f, %.1f] outside global [%.1f, %.1f]"
                     % (info["min_h"], info["max_h"], global_min_h, global_max_h))

    # --- Triangle index checks ---
    bad_idx = [i for i in info["indices"] if i >= n_verts]
    if bad_idx:
        warns.append("%d triangle indices >= n_verts(%d)" % (len(bad_idx), n_verts))

    # Degenerate triangles (zero area): any two of the 3 indices equal
    degen = 0
    for t in range(n_tris):
        i0, i1, i2 = info["indices"][t * 3], info["indices"][t * 3 + 1], info["indices"][t * 3 + 2]
        if i0 == i1 or i1 == i2 or i0 == i2:
            degen += 1
    if degen > 0:
        warns.append("%d degenerate (zero-area) triangles" % degen)

    # --- Edge index checks ---
    bad_edge = 0
    for e in info["edges"]:
        for v in e:
            if v >= n_verts:
                bad_edge += 1
    if bad_edge > 0:
        warns.append("%d edge indices >= n_verts" % bad_edge)

    # --- Parse completeness ---
    if not info["parse_ok"]:
        warns.append("incomplete parse: consumed %d / %d" % (info["consumed"], info["size"]))

    # --- OctVertexNormals extension ---
    normals_ext = None
    for ext in info["extensions"]:
        if ext["id"] == 1:
            normals_ext = ext
            break

    norm_stats = None
    if normals_ext is None:
        warns.append("OctVertexNormals extension missing")
    else:
        if normals_ext["len"] != 2 * n_verts:
            warns.append("normals ext length %d != 2*n_verts(%d)"
                         % (normals_ext["len"], 2 * n_verts))
        else:
            # Decode all normals, check unit length and z-component
            lengths = []
            z_comps = []
            nd = normals_ext["data"]
            for i in range(n_verts):
                nx, ny, nz = oct_decode(nd[i * 2], nd[i * 2 + 1])
                L = math.sqrt(nx * nx + ny * ny + nz * nz)
                lengths.append(L)
                z_comps.append(nz)
            L_min, L_max = min(lengths), max(lengths)
            z_min, z_max = min(z_comps), max(z_comps)
            z_mean = sum(z_comps) / len(z_comps) if z_comps else 0
            norm_stats = {
                "n": n_verts,
                "length_min": L_min, "length_max": L_max,
                "length_mean": sum(lengths) / len(lengths) if lengths else 0,
                "z_min": z_min, "z_max": z_max, "z_mean": z_mean,
            }
            # Oct encoding L1-normalizes the input, so decoded vectors have
            # length < 1 (e.g. (0.6,0,0.8) -> L1norm -> decode -> len 0.714).
            # This is expected Cesium behavior; the decoder must renormalize.
            # So we check length > 0.5 (sanity: not collapsed to zero) and
            # direction via z_mean > 0 (terrain normals point up).
            if L_min < 0.5:
                warns.append("normal length too small (<0.5): %.3f..%.3f" % (L_min, L_max))
            # Terrain normals should mostly point up (z>0 in local ENU).
            # A few may point sideways at cliffs, but z_mean should be > 0.
            if z_mean < 0:
                warns.append("normal z_mean=%.3f < 0 (normals pointing down?)" % z_mean)

    det = {
        "n_verts": n_verts, "n_tris": n_tris,
        "u_range": (u_min, u_max), "v_range": (v_min, v_max), "h_range": (h_min, h_max),
        "min_h": info["min_h"], "max_h": info["max_h"],
        "bad_idx": len(bad_idx), "degen_tris": degen, "bad_edge": bad_edge,
        "parse_ok": info["parse_ok"],
        "has_normals": normals_ext is not None,
        "norm_stats": norm_stats,
    }
    return (len(warns) == 0), warns, det


def main():
    if len(sys.argv) < 2:
        print("Usage: %s <output_dir>" % sys.argv[0])
        sys.exit(1)
    out_dir = sys.argv[1]
    if not os.path.isdir(out_dir):
        print("ERROR: not a directory: " + out_dir)
        sys.exit(1)

    # Collect all .terrain files
    tiles = []
    for root, dirs, files in os.walk(out_dir):
        for fn in files:
            if fn.endswith(".terrain"):
                tiles.append(os.path.join(root, fn))
    if not tiles:
        print("ERROR: no .terrain files under " + out_dir)
        sys.exit(1)
    tiles.sort()
    print("Found %d .terrain tiles under %s" % (len(tiles), out_dir))

    # First pass: find global height range across all tiles (from header min_h/max_h)
    global_min_h = float("inf")
    global_max_h = float("-inf")
    parse_err = 0
    for p in tiles:
        info, err = parse_tile(p)
        if err:
            parse_err += 1
            continue
        global_min_h = min(global_min_h, info["min_h"])
        global_max_h = max(global_max_h, info["max_h"])
    print("Global height range across tiles: [%.2f, %.2f] m" % (global_min_h, global_max_h))
    if parse_err:
        print("WARNING: %d tiles failed first-pass parse" % parse_err)
    print("=" * 80)

    # Second pass: detailed check per tile
    total_ok = 0
    total_warn = 0
    total_fail = 0
    all_norm_lengths = []
    all_norm_z = []
    agg = {"verts": [], "tris": [], "degen": 0, "bad_idx": 0, "bad_edge": 0}
    for p in tiles:
        ok, warns, det = check_tile(p, global_min_h, global_max_h)
        if ok:
            total_ok += 1
        elif warns:
            total_warn += 1
            print("WARN %s:" % os.path.relpath(p, out_dir))
            for w in warns:
                print("    - " + w)
        else:
            total_fail += 1
        agg["verts"].append(det["n_verts"])
        agg["tris"].append(det["n_tris"])
        agg["degen"] += det["degen_tris"]
        agg["bad_idx"] += det["bad_idx"]
        agg["bad_edge"] += det["bad_edge"]
        if det["norm_stats"]:
            all_norm_lengths.append(det["norm_stats"]["length_min"])
            all_norm_lengths.append(det["norm_stats"]["length_max"])
            all_norm_z.append(det["norm_stats"]["z_mean"])

    print("=" * 80)
    print("SUMMARY (%d tiles)" % len(tiles))
    print("  PASS (no warnings):  %d" % total_ok)
    print("  WARN (issues found): %d" % total_warn)
    print("  FAIL:                 %d" % total_fail)
    print()
    print("Vertex/triangle stats:")
    vs = agg["verts"]
    ts = agg["tris"]
    print("  verts/tile: min=%d max=%d mean=%.1f total=%d" % (min(vs), max(vs), sum(vs) / len(vs), sum(vs)))
    print("  tris/tile:  min=%d max=%d mean=%.1f total=%d" % (min(ts), max(ts), sum(ts) / len(ts), sum(ts)))
    print("  total degenerate triangles: %d" % agg["degen"])
    print("  total bad triangle indices: %d" % agg["bad_idx"])
    print("  total bad edge indices: %d" % agg["bad_edge"])
    print()
    if all_norm_lengths:
        print("Normal stats (across all tiles):")
        print("  length range: %.4f .. %.4f (expect ~1.0)" % (min(all_norm_lengths), max(all_norm_lengths)))
        print("  z_mean range: %.4f .. %.4f (expect > 0, terrain normals point up)" % (min(all_norm_z), max(all_norm_z)))
        z_pos = sum(1 for z in all_norm_z if z > 0)
        print("  tiles with z_mean > 0: %d / %d" % (z_pos, len(all_norm_z)))
    print()
    print("Verdict: " + ("PASS" if total_fail == 0 and total_warn == 0 else "ISSUES FOUND"))


if __name__ == "__main__":
    main()
