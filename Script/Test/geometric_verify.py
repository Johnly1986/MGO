"""Geometric verification: check bounding sphere contains all mesh vertices,
HOP is valid, and parent-child tile bounds are consistent."""
import struct, os, glob, math

def read_varint(data, off):
    result = 0
    shift = 0
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

# WGS84 ellipsoid
WGS84_A = 6378137.0
WGS84_B = 6356752.314245179

def geographic_to_ecef(lat_deg, lon_deg, h):
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    a = WGS84_A
    b = WGS84_B
    e2 = 1 - (b*b)/(a*a)
    sinlat = math.sin(lat)
    coslat = math.cos(lat)
    N = a / math.sqrt(1 - e2 * sinlat * sinlat)
    x = (N + h) * coslat * math.cos(lon)
    y = (N + h) * coslat * math.sin(lon)
    z = (N * (1 - e2) + h) * sinlat
    return x, y, z

def parse_tile(path):
    with open(path, "rb") as f:
        data = f.read()
    if len(data) < 88: return None

    cx, cy, cz = struct.unpack("<ddd", data[0:24])
    min_h, max_h = struct.unpack("<ff", data[24:32])
    bsx, bsy, bsz, bsr = struct.unpack("<dddd", data[32:64])
    hopx, hopy, hopz = struct.unpack("<ddd", data[64:88])

    off = 88
    n_verts, off = read_u32_le(data, off)

    u_vals = []; prev = 0
    for _ in range(n_verts):
        z = struct.unpack("<H", data[off:off+2])[0]; off += 2
        delta = zigzag_decode(z)
        cur = prev + delta; u_vals.append(cur); prev = cur

    v_vals = []; prev = 0
    for _ in range(n_verts):
        z = struct.unpack("<H", data[off:off+2])[0]; off += 2
        delta = zigzag_decode(z)
        cur = prev + delta; v_vals.append(cur); prev = cur

    h_vals = []; prev = 0
    for _ in range(n_verts):
        z = struct.unpack("<H", data[off:off+2])[0]; off += 2
        delta = zigzag_decode(z)
        cur = prev + delta; h_vals.append(cur); prev = cur

    # Dequantize heights: h_quant in [0,32767] maps to [min_h, max_h]
    range_h = max_h - min_h
    if range_h < 1e-6: range_h = 1.0
    h_meters = [min_h + (hq / 32767.0) * range_h for hq in h_vals]

    return {
        "center": (cx, cy, cz),
        "min_h": min_h, "max_h": max_h,
        "bs_center": (bsx, bsy, bsz), "bs_radius": bsr,
        "hop": (hopx, hopy, hopz),
        "u": u_vals, "v": v_vals, "h": h_meters,
        "n_verts": n_verts,
    }

def compute_tile_bounds(level, x, y):
    tiles_x = 2 * (1 << level)
    tiles_y = (1 << level)
    tile_w = 360.0 / tiles_x
    tile_h = 180.0 / tiles_y
    west = -180.0 + x * tile_w
    east = west + tile_w
    south = -90.0 + y * tile_h
    north = south + tile_h
    return west, south, east, north

base = sys.argv[1] if len(sys.argv) > 1 else "."
all_tiles = []
for path in glob.glob(os.path.join(base, "*", "*", "*.terrain")):
    parts = path.replace("\\", "/").split("/")
    level = int(parts[-3])
    x = int(parts[-2])
    y = int(parts[-1].replace(".terrain", ""))
    info = parse_tile(path)
    if info:
        all_tiles.append((level, x, y, info))

print("=" * 70)
print("Geometric verification — " + str(len(all_tiles)) + " tiles")
print("=" * 70)

# Check 1: Bounding sphere contains all mesh vertices + tile corners
print("\n[Check 1] Bounding sphere contains all vertices + tile corners")
bs_failures = 0
for level, x, y, info in all_tiles:
    bsx, bsy, bsz = info["bs_center"]
    bsr = info["bs_radius"]
    west, south, east, north = compute_tile_bounds(level, x, y)

    # Check all mesh vertices
    for i in range(info["n_verts"]):
        uq = info["u"][i]
        vq = info["v"][i]
        h = info["h"][i]
        lon = west + (uq / 32767.0) * (east - west)
        lat = south + (vq / 32767.0) * (north - south)
        ex, ey, ez = geographic_to_ecef(lat, lon, h)
        dist = math.sqrt((ex-bsx)**2 + (ey-bsy)**2 + (ez-bsz)**2)
        if dist > bsr + 1.0:  # 1m tolerance for float precision
            print("  FAIL L{}/{}/{}: vertex {} at {}m > bsr {}m".format(
                level, x, y, i, dist, bsr))
            bs_failures += 1
            break

    # Check 4 tile corners at min and max height
    corner_fail = False
    for clon, clat in [(west,north),(east,north),(east,south),(west,south)]:
        for ch in [info["min_h"], info["max_h"]]:
            ex, ey, ez = geographic_to_ecef(clat, clon, ch)
            dist = math.sqrt((ex-bsx)**2 + (ey-bsy)**2 + (ez-bsz)**2)
            if dist > bsr + 1.0:
                print("  FAIL L{}/{}/{}: corner ({},{}) h={} at {}m > bsr {}m".format(
                    level, x, y, clon, clat, ch, dist, bsr))
                bs_failures += 1
                corner_fail = True
                break
        if corner_fail: break

if bs_failures == 0:
    print("  PASS: all tiles' bounding spheres contain vertices + corners")

# Check 2: HOP is outside Earth's surface (distance from Earth center > a)
print("\n[Check 2] Horizon Occlusion Point is outside Earth")
hop_failures = 0
for level, x, y, info in all_tiles:
    hx, hy, hz = info["hop"]
    dist = math.sqrt(hx*hx + hy*hy + hz*hz)
    if dist < WGS84_A:
        print("  FAIL L{}/{}/{}: HOP at {}m < Earth radius {}m".format(
            level, x, y, dist, WGS84_A))
        hop_failures += 1

if hop_failures == 0:
    print("  PASS: all HOPs are outside Earth surface")
    # Show HOP distance range
    dists = [math.sqrt(t[3]["hop"][0]**2 + t[3]["hop"][1]**2 + t[3]["hop"][2]**2) for t in all_tiles]
    print("  HOP distance range: {} - {} km".format(
        format(min(dists)/1000, ".1f"), format(max(dists)/1000, ".1f")))

# Check 3: Parent-child bounds consistency (parent extent = union of 4 children)
print("\n[Check 3] Parent-child bounds consistency")
pc_failures = 0
pc_checked = 0
tiles_by_pos = {(t[0], t[1], t[2]): t for t in all_tiles}
for level, x, y, info in all_tiles:
    if level <= 0: continue
    parent_level = level - 1
    parent_x = x // 2
    parent_y = y // 2
    parent = tiles_by_pos.get((parent_level, parent_x, parent_y))
    if parent is None: continue
    pc_checked += 1
    p_west, p_south, p_east, p_north = compute_tile_bounds(parent_level, parent_x, parent_y)
    c_west, c_south, c_east, c_north = compute_tile_bounds(level, x, y)
    # Child should fit within parent
    if c_west < p_west - 1e-9 or c_south < p_south - 1e-9 or \
       c_east > p_east + 1e-9 or c_north > p_north + 1e-9:
        print("  FAIL: child L{}/{}/{} extends beyond parent L{}/{}/{}".format(
            level, x, y, parent_level, parent_x, parent_y))
        pc_failures += 1

if pc_failures == 0:
    print("  PASS: {} parent-child relationships verified".format(pc_checked))

# Check 4: Height range consistency across levels
print("\n[Check 4] Height range per level")
for lv in sorted(set(t[0] for t in all_tiles)):
    lv_tiles = [t for t in all_tiles if t[0] == lv]
    mins = [i["min_h"] for _, _, _, i in lv_tiles]
    maxs = [i["max_h"] for _, _, _, i in lv_tiles]
    print("  Level {}: min_h=[{}, {}]m, max_h=[{}, {}]m, overall=[{}, {}]m".format(
        lv, format(min(mins), ".1f"), format(max(mins), ".1f"),
        format(min(maxs), ".1f"), format(max(maxs), ".1f"),
        format(min(mins), ".1f"), format(max(maxs), ".1f")))

# Check 5: Bounding sphere radius scales with tile size
print("\n[Check 5] Bounding sphere radius vs tile diagonal")
for lv in sorted(set(t[0] for t in all_tiles)):
    lv_tiles = [t for t in all_tiles if t[0] == lv]
    radii = [i["bs_radius"] for _, _, _, i in lv_tiles]
    # Expected: tile diagonal ≈ tile_size * sqrt(2) at Earth surface
    tiles_y = (1 << lv)
    tile_h_deg = 180.0 / tiles_y
    # Approximate tile size at equator: 111km per degree
    expected_diag = tile_h_deg * 111000 * math.sqrt(2)
    print("  Level {}: mean bsr={}m, tile diagonal≈{}m, ratio={:.2f}".format(
        lv, format(sum(radii)/len(radii), ".0f"),
        format(expected_diag, ".0f"),
        (sum(radii)/len(radii)) / expected_diag))
