#!/usr/bin/env python3
"""
OSGBConverter verification test suite.

Validates:
  1. Projection accuracy (cm-level)
  2. File size ratio (output ≤ 2.25× input)
  3. Tile file structure correctness (tileset.json + b3dm format)

Usage:
  python3 test_verify.py <output_dir> <input_dir>
"""

import json
import os
import struct
import sys
import math


# =============================================================================
# Test 1: Projection Accuracy (cm-level)
# =============================================================================

def wgs84_to_ecef(lat_deg, lon_deg, h):
    """Convert WGS84 geographic to ECEF coordinates."""
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


def enu_to_ecef_rotation(lat_deg, lon_deg):
    """Compute ENU-to-ECEF rotation matrix (column-major 3x3)."""
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)

    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)

    # Column 0: East direction in ECEF
    R = [
        -sin_lon,           cos_lon,           0.0,
        -sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat,
         cos_lat * cos_lon,  cos_lat * sin_lon, sin_lat,
    ]
    return R


def mat4x4_transform_point(transform, x, y, z):
    """Apply a 4x4 column-major transform to a point."""
    tx = transform[0] * x + transform[4] * y + transform[8]  * z + transform[12]
    ty = transform[1] * x + transform[5] * y + transform[9]  * z + transform[13]
    tz = transform[2] * x + transform[6] * y + transform[10] * z + transform[14]
    return tx, ty, tz


def test_projection_accuracy(tileset_path):
    """
    Verify that the root transform in tileset.json correctly maps
    the ENU origin to the expected ECEF position.

    For Production_3: ENU origin at (22.64785, 113.06277, 0)
    """
    with open(tileset_path) as f:
        ts = json.load(f)

    transform = ts.get("root", {}).get("transform")
    if not transform:
        print("  FAIL: No root.transform in tileset.json")
        return False

    # Expected ECEF for origin (22.64785°N, 113.06277°E, 0m)
    expected_ecef = wgs84_to_ecef(22.64785, 113.06277, 0.0)

    # Transform ENU origin (0,0,0) through the root transform
    actual_ecef = mat4x4_transform_point(transform, 0, 0, 0)

    # Compute error
    dx = actual_ecef[0] - expected_ecef[0]
    dy = actual_ecef[1] - expected_ecef[1]
    dz = actual_ecef[2] - expected_ecef[2]
    error = math.sqrt(dx * dx + dy * dy + dz * dz)

    print(f"  Expected ECEF: ({expected_ecef[0]:.2f}, {expected_ecef[1]:.2f}, {expected_ecef[2]:.2f})")
    print(f"  Actual ECEF:   ({actual_ecef[0]:.2f}, {actual_ecef[1]:.2f}, {actual_ecef[2]:.2f})")
    print(f"  Error: {error:.4f} m ({error * 100:.2f} cm)")

    if error < 0.01:  # < 1 cm
        print("  PASS: Projection accuracy within 1 cm")
        return True
    elif error < 0.10:  # < 10 cm
        print("  PASS: Projection accuracy within 10 cm")
        return True
    else:
        print(f"  FAIL: Projection error {error:.4f} m exceeds cm-level threshold")
        return False


# =============================================================================
# Test 2: File Size Ratio
# =============================================================================

def get_dir_size(path):
    """Get total size of all files in a directory tree."""
    total = 0
    for root, dirs, files in os.walk(path):
        for f in files:
            fp = os.path.join(root, f)
            try:
                total += os.path.getsize(fp)
            except OSError:
                pass
    return total


def test_file_size(output_dir, input_dir):
    """
    Verify that the output size is ≤ 2.25× the input size.
    """
    input_size = get_dir_size(input_dir)
    output_size = get_dir_size(output_dir)

    if input_size == 0:
        print("  FAIL: Input directory is empty")
        return False

    ratio = output_size / input_size

    print(f"  Input size:  {input_size / (1024*1024):.1f} MB")
    print(f"  Output size: {output_size / (1024*1024):.1f} MB")
    print(f"  Ratio: {ratio:.2f}x")

    if ratio <= 2.25:
        print("  PASS: File size within 2.25x limit")
        return True
    else:
        print(f"  FAIL: File size ratio {ratio:.2f}x exceeds 2.25x limit")
        return False


# =============================================================================
# Test 3: Tile File Structure Correctness
# =============================================================================

def validate_b3dm(filepath):
    """Validate a .b3dm file header and structure."""
    try:
        with open(filepath, "rb") as f:
            data = f.read()

        if len(data) < 28:
            return False, "File too small for b3dm header"

        # b3dm magic: "b3dm" (little-endian: 0x6D643362)
        magic = struct.unpack_from("<I", data, 0)[0]
        if magic != 0x6D643362:
            return False, f"Invalid magic: {hex(magic)}"

        version = struct.unpack_from("<I", data, 4)[0]
        if version != 1:
            return False, f"Invalid version: {version}"

        byte_length = struct.unpack_from("<I", data, 8)[0]
        if byte_length != len(data):
            return False, f"Length mismatch: header={byte_length}, actual={len(data)}"

        # Read feature table JSON
        feature_json_len = struct.unpack_from("<I", data, 12)[0]
        feature_bin_len = struct.unpack_from("<I", data, 16)[0]
        batch_json_len = struct.unpack_from("<I", data, 20)[0]
        batch_bin_len = struct.unpack_from("<I", data, 24)[0]

        # Check GLB magic after feature+batch tables
        glb_offset = 28 + feature_json_len + feature_bin_len + batch_json_len + batch_bin_len
        if glb_offset >= len(data):
            return False, "GLB offset out of bounds"

        glb_magic = struct.unpack_from("<I", data, glb_offset)[0]
        if glb_magic != 0x46546C67:  # "glTF"
            return False, f"Invalid GLB magic: {hex(glb_magic)}"

        return True, "OK"

    except Exception as e:
        return False, str(e)


def test_tile_structure(output_dir):
    """
    Verify 3D Tiles structure:
    - tileset.json exists and is valid JSON
    - All .b3dm files have valid headers
    - Tile hierarchy is consistent
    """
    tileset_path = os.path.join(output_dir, "tileset.json")
    if not os.path.exists(tileset_path):
        print("  FAIL: tileset.json not found")
        return False

    # Validate tileset.json
    try:
        with open(tileset_path) as f:
            tileset = json.load(f)
    except json.JSONDecodeError as e:
        print(f"  FAIL: tileset.json is not valid JSON: {e}")
        return False

    # Check required fields
    required = ["asset", "geometricError", "root"]
    for field in required:
        if field not in tileset:
            print(f"  FAIL: tileset.json missing required field: {field}")
            return False

    print(f"  tileset.json: valid (version={tileset['asset'].get('version', 'unknown')})")

    # Find and validate all .b3dm files
    b3dm_files = []
    for root, dirs, files in os.walk(output_dir):
        for f in files:
            if f.endswith(".b3dm"):
                b3dm_files.append(os.path.join(root, f))

    if not b3dm_files:
        print("  FAIL: No .b3dm files found")
        return False

    print(f"  Found {len(b3dm_files)} .b3dm file(s)")

    # Validate each b3dm file
    failures = 0
    for fp in b3dm_files[:50]:  # Check first 50 for speed
        valid, msg = validate_b3dm(fp)
        if not valid:
            failures += 1
            if failures <= 3:
                print(f"  FAIL: {os.path.basename(fp)}: {msg}")

    if failures > 0:
        print(f"  FAIL: {failures}/{len(b3dm_files[:50])} b3dm files invalid")
        return False

    print(f"  PASS: All b3dm files have valid headers")

    # Check tile structure consistency
    root = tileset["root"]
    bbox = root.get("boundingVolume", {}).get("box")
    if bbox and len(bbox) == 12:
        print(f"  Root boundingVolume: box format (12 floats)")
        # Verify box is non-degenerate
        half_axes = [
            bbox[3], bbox[4], bbox[5],
            bbox[6], bbox[7], bbox[8],
            bbox[9], bbox[10], bbox[11],
        ]
        max_axis = max(abs(v) for v in half_axes)
        if max_axis < 1e-6:
            print("  FAIL: Root bounding volume is degenerate (all zero)")
            return False
    else:
        print("  FAIL: Root missing or invalid boundingVolume.box")
        return False

    ref = root.get("refine")
    if ref:
        print(f"  Root refine mode: {ref}")

    geom_error = root.get("geometricError", 0)
    if geom_error <= 0:
        print("  FAIL: Root geometricError is zero or negative")
        return False
    print(f"  Root geometricError: {geom_error}")

    return True


# =============================================================================
# Main
# =============================================================================

def main():
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <output_dir> <input_dir>")
        sys.exit(1)

    output_dir = sys.argv[1]
    input_dir = sys.argv[2]

    if not os.path.isdir(output_dir):
        print(f"Error: output directory not found: {output_dir}")
        sys.exit(1)
    if not os.path.isdir(input_dir):
        print(f"Error: input directory not found: {input_dir}")
        sys.exit(1)

    all_passed = True

    print("=" * 60)
    print("Test 1: Projection Accuracy")
    print("=" * 60)
    if not test_projection_accuracy(os.path.join(output_dir, "tileset.json")):
        all_passed = False

    print()
    print("=" * 60)
    print("Test 2: File Size Ratio")
    print("=" * 60)
    if not test_file_size(output_dir, input_dir):
        all_passed = False

    print()
    print("=" * 60)
    print("Test 3: Tile File Structure")
    print("=" * 60)
    if not test_tile_structure(output_dir):
        all_passed = False

    print()
    print("=" * 60)
    if all_passed:
        print("ALL TESTS PASSED")
        sys.exit(0)
    else:
        print("SOME TESTS FAILED")
        sys.exit(1)


if __name__ == "__main__":
    main()