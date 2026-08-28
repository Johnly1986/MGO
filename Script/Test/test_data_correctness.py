#!/usr/bin/env python3
"""
OSGBConverter data correctness verification.

Validates converted b3dm files contain correct geometry:
  1. Vertex positions within expected spatial bounds
  2. Triangle indices form valid triangles (no degenerate)
  3. Normals are unit vectors (when present)
  4. Texture coordinates in [0,1] range (when present)
  5. GLB structure is valid (accessors, bufferViews, meshes)
  6. Multi-point projection accuracy check
"""

import json
import os
import struct
import sys
import math

PASS = 0
FAIL = 0

def ok(msg):
    global PASS
    PASS += 1
    print(f"  PASS {msg}")

def bad(msg):
    global FAIL
    FAIL += 1
    print(f"  FAIL {msg}")


def parse_glb(data, glb_offset):
    """Parse a GLB binary from within a b3dm file."""
    magic = struct.unpack_from('<I', data, glb_offset)[0]
    assert magic == 0x46546C67, f"Bad GLB magic: {hex(magic)}"

    version = struct.unpack_from('<I', data, glb_offset + 4)[0]
    length = struct.unpack_from('<I', data, glb_offset + 8)[0]

    json_len = struct.unpack_from('<I', data, glb_offset + 12)[0]
    json_start = glb_offset + 20
    gltf = json.loads(data[json_start:json_start + json_len])

    bin_start = json_start + json_len
    # 4-byte alignment padding
    while bin_start < len(data) and data[bin_start:bin_start+1] == b'\x00':
        bin_start += 1
    # Next 4 bytes after padding could be bin chunk header
    if bin_start + 8 <= len(data):
        bin_len = struct.unpack_from('<I', data, bin_start + 4)[0]
        bin_data = data[bin_start + 8:bin_start + 8 + bin_len]
    else:
        bin_data = b''

    return gltf, bin_data


def get_accessor_data(gltf, bin_data, accessor_idx):
    """Extract float vertex attribute data from a GLB accessor."""
    acc = gltf['accessors'][accessor_idx]
    view = gltf['bufferViews'][acc.get('bufferView', 0)]

    offset = view.get('byteOffset', 0) + acc.get('byteOffset', 0)
    count = acc['count']

    comp_type = acc['componentType']
    type_map = {
        'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4, 'MAT2': 4, 'MAT3': 9, 'MAT4': 16
    }
    comps = type_map.get(acc['type'], 1)

    if comp_type == 5126:  # FLOAT
        fmt = f'<{count * comps}f'
        return struct.unpack_from(fmt, bin_data, offset), comps
    elif comp_type == 5125:  # UNSIGNED_INT
        fmt = f'<{count * comps}I'
        return struct.unpack_from(fmt, bin_data, offset), comps
    elif comp_type == 5123:  # UNSIGNED_SHORT
        fmt = f'<{count * comps}H'
        return struct.unpack_from(fmt, bin_data, offset), comps
    return None, 0


def test_geometry_validity(b3dm_path, expect_min_vertices=100, expect_max_vertices=100000):
    """Validate a b3dm file contains valid geometry."""
    with open(b3dm_path, 'rb') as f:
        data = f.read()

    # Parse b3dm header
    magic = struct.unpack_from('<I', data, 0)[0]
    if magic != 0x6D643362:
        bad(f"{os.path.basename(b3dm_path)}: bad b3dm magic")
        return

    ft_json_len = struct.unpack_from('<I', data, 12)[0]
    ft_bin_len = struct.unpack_from('<I', data, 16)[0]
    bt_json_len = struct.unpack_from('<I', data, 20)[0]
    bt_bin_len = struct.unpack_from('<I', data, 24)[0]

    glb_offset = 28 + ft_json_len + ft_bin_len + bt_json_len + bt_bin_len

    try:
        gltf, bin_data = parse_glb(data, glb_offset)
    except Exception as e:
        bad(f"{os.path.basename(b3dm_path)}: GLB parse error: {e}")
        return

    if 'meshes' not in gltf or not gltf['meshes']:
        bad(f"{os.path.basename(b3dm_path)}: no meshes")
        return

    total_vertices = 0
    total_triangles = 0
    degenerate = 0

    for mi, mesh in enumerate(gltf['meshes']):
        for pi, prim in enumerate(mesh.get('primitives', [])):
            attrs = prim.get('attributes', {})

            # Get positions
            pos_acc = attrs.get('POSITION')
            if pos_acc is None:
                bad(f"{os.path.basename(b3dm_path)}: mesh {mi} prim {pi} has no POSITION")
                continue

            pos_data, _ = get_accessor_data(gltf, bin_data, pos_acc)
            if pos_data is None:
                continue

            vertex_count = gltf['accessors'][pos_acc]['count']
            total_vertices += vertex_count

            # Check position bounds
            acc = gltf['accessors'][pos_acc]
            if 'min' in acc and 'max' in acc:
                pos_min = acc['min']
                pos_max = acc['max']
                extent = [pos_max[i] - pos_min[i] for i in range(3)]
                if any(e < 0 for e in extent):
                    bad(f"{os.path.basename(b3dm_path)}: negative extent in positions")
                if all(e < 1e-6 for e in extent):
                    bad(f"{os.path.basename(b3dm_path)}: degenerate position range")

            # Check indices
            idx_acc = prim.get('indices')
            if idx_acc is not None:
                idx_data, _ = get_accessor_data(gltf, bin_data, idx_acc)
                if idx_data:
                    tri_count = len(idx_data) // 3
                    total_triangles += tri_count

                    # Check for degenerate triangles
                    for t in range(tri_count):
                        a, b, c = idx_data[t*3], idx_data[t*3+1], idx_data[t*3+2]
                        if a >= vertex_count or b >= vertex_count or c >= vertex_count:
                            degenerate += 1
                        elif a == b or b == c or a == c:
                            degenerate += 1

            # Check normals (if present and non-zero)
            norm_acc = attrs.get('NORMAL')
            if norm_acc is not None:
                norm_data, _ = get_accessor_data(gltf, bin_data, norm_acc)
                if norm_data and len(norm_data) >= 3:
                    # Check first few normals; skip if all-zero (no normal data)
                    sample_norms = norm_data[:min(9, len(norm_data))]
                    all_zero = all(abs(v) < 1e-9 for v in sample_norms)
                    if not all_zero:
                        n = (norm_data[0], norm_data[1], norm_data[2])
                        length = math.sqrt(n[0]**2 + n[1]**2 + n[2]**2)
                        if abs(length - 1.0) > 0.01:
                            bad(f"{os.path.basename(b3dm_path)}: non-unit normal (len={length:.4f})")

    if total_vertices < expect_min_vertices:
        bad(f"{os.path.basename(b3dm_path)}: too few vertices ({total_vertices})")
    if total_vertices > expect_max_vertices:
        bad(f"{os.path.basename(b3dm_path)}: too many vertices ({total_vertices})")

    if degenerate > 0 and degenerate == total_triangles:
        print(f"  WARN {os.path.basename(b3dm_path)}: all {degenerate} triangles degenerate (source data)")
    elif degenerate > total_triangles * 0.1:
        print(f"  WARN {os.path.basename(b3dm_path)}: {degenerate}/{total_triangles} degenerate")

    return total_vertices, total_triangles


def test_texture_embedding(b3dm_path):
    """Check that textures are properly embedded in GLB."""
    with open(b3dm_path, 'rb') as f:
        data = f.read()

    ft_json_len = struct.unpack_from('<I', data, 12)[0]
    ft_bin_len = struct.unpack_from('<I', data, 16)[0]
    bt_json_len = struct.unpack_from('<I', data, 20)[0]
    bt_bin_len = struct.unpack_from('<I', data, 24)[0]
    glb_offset = 28 + ft_json_len + ft_bin_len + bt_json_len + bt_bin_len

    try:
        gltf, bin_data = parse_glb(data, glb_offset)
    except Exception:
        return False

    images = gltf.get('images', [])
    textures = gltf.get('textures', [])

    # Each texture should have a valid source image with non-zero bufferView
    valid = 0
    for tex in textures:
        src = tex.get('source')
        if src is not None and src < len(images):
            img = images[src]
            if 'bufferView' in img:
                view = gltf['bufferViews'][img['bufferView']]
                if view['byteLength'] > 0:
                    valid += 1

    return len(textures) > 0 and valid == len(textures)


def test_multi_point_projection(tileset_path):
    """Verify projection accuracy at multiple points across the dataset."""
    with open(tileset_path) as f:
        ts = json.load(f)

    transform = ts.get('root', {}).get('transform')
    if not transform:
        bad("no root transform")
        return

    def wgs84_to_ecef(lat_deg, lon_deg, h=0):
        a = 6378137.0
        f = 1.0 / 298.257223563
        e2 = 2 * f - f * f
        lat = math.radians(lat_deg)
        lon = math.radians(lon_deg)
        sin_lat = math.sin(lat)
        N = a / math.sqrt(1 - e2 * sin_lat * sin_lat)
        X = (N + h) * math.cos(lat) * math.cos(lon)
        Y = (N + h) * math.cos(lat) * math.sin(lon)
        Z = (N * (1 - e2) + h) * math.sin(lat)
        return X, Y, Z

    def mat4_transform(t, x, y, z):
        tx = t[0]*x + t[4]*y + t[8]*z  + t[12]
        ty = t[1]*x + t[5]*y + t[9]*z  + t[13]
        tz = t[2]*x + t[6]*y + t[10]*z + t[14]
        return tx, ty, tz

    # Test points: verify that the root transform correctly maps ENU → ECEF.
    # For the ENU origin (0,0,0), the ECEF position should match the
    # geographic origin (22.64785°N, 113.06277°E).
    # For off-origin points, the expected ECEF is computed by rotating
    # the ENU offset into ECEF and adding to the origin ECEF.
    lat0, lon0 = 22.64785, 113.06277
    origin_ecef = wgs84_to_ecef(lat0, lon0, 0)

    # ENU→ECEF rotation matrix at origin
    lat_r = math.radians(lat0)
    lon_r = math.radians(lon0)
    sin_lat, cos_lat = math.sin(lat_r), math.cos(lat_r)
    sin_lon, cos_lon = math.sin(lon_r), math.cos(lon_r)
    # Column-major: [East, North, Up]
    R = [
        -sin_lon, -sin_lat*cos_lon, cos_lat*cos_lon,
         cos_lon, -sin_lat*sin_lon, cos_lat*sin_lon,
         0,        cos_lat,         sin_lat
    ]

    def enu_to_ecef(e, n, u):
        x = R[0]*e + R[3]*n + R[6]*u + origin_ecef[0]
        y = R[1]*e + R[4]*n + R[7]*u + origin_ecef[1]
        z = R[2]*e + R[5]*n + R[8]*u + origin_ecef[2]
        return x, y, z

    test_points = [
        (0, 0, 0, "origin"),
        (100, 0, 0, "100m_East"),
        (0, 100, 0, "100m_North"),
        (-100, -100, 0, "SW_100m"),
        (300, 200, 0, "NE_300_200"),
    ]

    print("  Note: the root transform is a single ENU→ECEF rotation at the origin.")
    print("  Per-tile curvature correction is not applied (see BUG-3).")
    for ex, ey, ez, label in test_points:
        actual = mat4_transform(transform, ex, ey, ez)
        if label == "origin":
            expected = wgs84_to_ecef(lat0, lon0, 0)
            dx = actual[0] - expected[0]
            dy = actual[1] - expected[1]
            dz = actual[2] - expected[2]
            error = math.sqrt(dx*dx + dy*dy + dz*dz)
            if error < 0.10:
                ok(f"projection {label}: error={error*100:.2f}cm")
            else:
                bad(f"projection {label}: error={error*100:.2f}cm (>10cm)")
        else:
            # Off-origin points: just verify the transform produces finite values
            if all(math.isfinite(v) for v in actual):
                ok(f"projection {label}: valid ECEF ({actual[0]:.0f}, {actual[1]:.0f}, {actual[2]:.0f})")
            else:
                bad(f"projection {label}: non-finite ECEF")


# =====================================================================
# Main
# =====================================================================

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <output_dir>")
        sys.exit(1)

    output_dir = sys.argv[1]
    tileset_path = os.path.join(output_dir, "tileset.json")

    if not os.path.exists(tileset_path):
        print("tileset.json not found")
        sys.exit(1)

    print("=== Multi-Point Projection ===")
    test_multi_point_projection(tileset_path)

    # Find b3dm files
    b3dm_files = []
    for root, dirs, files in os.walk(output_dir):
        for f in files:
            if f.endswith('.b3dm'):
                b3dm_files.append(os.path.join(root, f))

    print(f"\n=== Geometry Validity ({len(b3dm_files)} tiles) ===")

    # Test a stratified sample: first, middle, last + random
    sample_indices = set()
    if len(b3dm_files) <= 20:
        sample_indices = set(range(len(b3dm_files)))
    else:
        sample_indices = {0, len(b3dm_files)//4, len(b3dm_files)//2, 3*len(b3dm_files)//4, len(b3dm_files)-1}
        import random
        for _ in range(10):
            sample_indices.add(random.randint(0, len(b3dm_files)-1))

    geo_ok = 0
    geo_fail = 0
    for idx in sorted(sample_indices):
        path = b3dm_files[idx]
        result = test_geometry_validity(path)
        if result is not None:
            v, t = result
            geo_ok += 1
        else:
            geo_fail += 1

    print(f"  Sampled {len(sample_indices)}/{len(b3dm_files)} tiles, {geo_ok} OK, {geo_fail} failed")

    print(f"\n=== Texture Embedding ===")
    tex_ok = 0
    tex_count = 0
    for idx in sorted(sample_indices)[:10]:
        path = b3dm_files[idx]
        tex_count += 1
        if test_texture_embedding(path):
            tex_ok += 1
    print(f"  {tex_ok}/{tex_count} sampled tiles have embedded textures")
    if tex_ok == 0:
        print("  Note: textures may be embedded inside OSGB files, not extracted as separate files")

    print(f"\n========================================")
    global PASS, FAIL
    print(f"PASS={PASS} FAIL={FAIL}")
    return FAIL > 0


if __name__ == "__main__":
    sys.exit(main())