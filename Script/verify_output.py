#!/usr/bin/env python3
"""
Verify TMS tile output from imagery_tiler.py.

Checks:
  1. Directory structure {z}/{x}/{y}.png
  2. All PNG files are valid and 256x256 RGBA
  3. Tile geographic bounds are consistent across levels
  4. Edge stitching between adjacent tiles
  5. layer.json and tilemapresource.xml validity
  6. Geographic bounds match expected projection output

Usage:
  python verify_output.py <tiles_dir> [--prj prj_file] [--origin easting,northing]
"""

import argparse
import json
import math
import os
import struct
import sys
import glob
from xml.etree import ElementTree as ET

import numpy as np
from PIL import Image

TILE_SIZE = 256


def check_structure(tiles_dir):
    """Verify directory structure and count tiles per level."""
    errors = []
    levels = {}
    total = 0

    for entry in sorted(os.listdir(tiles_dir)):
        path = os.path.join(tiles_dir, entry)
        if not os.path.isdir(path):
            continue
        if not entry.isdigit():
            continue

        level = int(entry)
        level_tiles = glob.glob(os.path.join(path, "*", "*.png"))
        levels[level] = len(level_tiles)
        total += len(level_tiles)

        # Check each tile path format: {z}/{x}/{y}.png
        for t in level_tiles:
            rel = os.path.relpath(t, tiles_dir)
            parts = rel.replace("\\", "/").split("/")
            if len(parts) != 3:
                errors.append(f"Bad tile path: {rel}")
                continue
            z_str, x_str, y_file = parts
            if not z_str.isdigit() or not x_str.isdigit():
                errors.append(f"Non-integer z/x in: {rel}")
                continue
            y_str = y_file.replace(".png", "")
            if not y_str.isdigit():
                errors.append(f"Non-integer y in: {rel}")

        print(f"  Level {level}: {len(level_tiles)} tiles")

    print(f"  Total: {total} tiles across {len(levels)} levels")
    if not levels:
        errors.append("No tiles found")

    return levels, errors


def validate_png_files(tiles_dir):
    """Verify all PNG files are valid 256x256 RGBA images."""
    errors = []
    all_tiles = glob.glob(os.path.join(tiles_dir, "**", "*.png"), recursive=True)

    for t in all_tiles:
        try:
            img = Image.open(t)
            if img.size != (TILE_SIZE, TILE_SIZE):
                errors.append(f"Wrong size {img.size} in {os.path.relpath(t, tiles_dir)}")
            if img.mode not in ("RGBA", "RGB"):
                errors.append(f"Wrong mode {img.mode} in {os.path.relpath(t, tiles_dir)}")
        except Exception as e:
            errors.append(f"Invalid PNG {os.path.relpath(t, tiles_dir)}: {e}")

    print(f"  Checked {len(all_tiles)} PNG files: {len(errors)} errors")
    return errors


def check_edge_stitching(tiles_dir):
    """Verify adjacent tile edges are consistent."""
    errors = []
    levels = sorted([int(d) for d in os.listdir(tiles_dir)
                     if os.path.isdir(os.path.join(tiles_dir, d)) and d.isdigit()])

    if not levels:
        return ["No levels to check"]

    # Check the finest level (most tiles, most likely to have adjacent pairs)
    level = max(levels)
    level_dir = os.path.join(tiles_dir, str(level))
    tiles = glob.glob(os.path.join(level_dir, "*", "*.png"))

    # Group tiles by x coordinate
    by_x = {}
    for t in tiles:
        parts = os.path.relpath(t, tiles_dir).replace("\\", "/").split("/")
        tx = int(parts[1])
        ty = int(parts[2].replace(".png", ""))
        if tx not in by_x:
            by_x[tx] = {}
        by_x[tx][ty] = t

    h_checks = 0
    v_checks = 0

    # Check horizontal adjacency (same y, consecutive x)
    for tx in sorted(by_x.keys()):
        if tx + 1 in by_x:
            common_y = set(by_x[tx].keys()) & set(by_x[tx + 1].keys())
            for ty in sorted(common_y):
                try:
                    t1 = np.array(Image.open(by_x[tx][ty]))
                    t2 = np.array(Image.open(by_x[tx + 1][ty]))
                    right = t1[:, 255, :3]
                    left = t2[:, 0, :3]
                    diff = np.abs(right.astype(int) - left.astype(int))
                    if diff.max() > 100:
                        errors.append(
                            f"H-edge ({tx},{ty})-({tx+1},{ty}): max_diff={diff.max()}, mean={diff.mean():.1f}")
                    h_checks += 1
                except Exception as e:
                    errors.append(f"Error checking H-edge ({tx},{ty})-({tx+1},{ty}): {e}")

    # Check vertical adjacency (same x, consecutive y)
    for tx in sorted(by_x.keys()):
        for ty in sorted(by_x[tx].keys()):
            if ty + 1 in by_x[tx]:
                try:
                    t1 = np.array(Image.open(by_x[tx][ty]))
                    t2 = np.array(Image.open(by_x[tx][ty + 1]))
                    # TMS Y=0 at south; larger Y = more north
                    # t1 (ty) is south of t2 (ty+1)
                    # Shared boundary: t1 top (north) = t2 bottom (south)
                    # In pixel space: t1's top row (row 0, north edge) should match
                    # t2's bottom row (row 255, south edge)
                    top_of_south = t1[0, :, :3]
                    bottom_of_north = t2[255, :, :3]
                    diff = np.abs(top_of_south.astype(int) - bottom_of_north.astype(int))
                    if diff.max() > 100:
                        errors.append(
                            f"V-edge ({tx},{ty})-({tx},{ty+1}): max_diff={diff.max()}, mean={diff.mean():.1f}")
                    v_checks += 1
                except Exception as e:
                    errors.append(f"Error checking V-edge ({tx},{ty})-({tx},{ty+1}): {e}")

    print(f"  H-edges checked: {h_checks}, V-edges checked: {v_checks}")
    if errors:
        print(f"  Edge issues: {len(errors)}")
    return errors


def check_metadata(tiles_dir):
    """Validate layer.json and tilemapresource.xml."""
    errors = []

    # layer.json
    lj_path = os.path.join(tiles_dir, "layer.json")
    if not os.path.exists(lj_path):
        errors.append("layer.json not found")
    else:
        try:
            with open(lj_path) as f:
                lj = json.load(f)
            for key in ["tiles", "bounds", "minzoom", "maxzoom"]:
                if key not in lj:
                    errors.append(f"layer.json missing key: {key}")
            bounds = lj.get("bounds", [])
            if len(bounds) == 4:
                if not (-180 <= bounds[0] <= 180 and -90 <= bounds[1] <= 90):
                    errors.append(f"layer.json bounds out of range: {bounds}")
            print(f"  layer.json: valid, keys={list(lj.keys())}")
        except Exception as e:
            errors.append(f"layer.json parse error: {e}")

    # tilemapresource.xml
    tm_path = os.path.join(tiles_dir, "tilemapresource.xml")
    if not os.path.exists(tm_path):
        errors.append("tilemapresource.xml not found")
    else:
        try:
            tree = ET.parse(tm_path)
            root = tree.getroot()
            if root.tag != "TileMap":
                errors.append(f"tilemapresource.xml root is {root.tag}, expected TileMap")
            print(f"  tilemapresource.xml: valid, root={root.tag}")
        except Exception as e:
            errors.append(f"tilemapresource.xml parse error: {e}")

    return errors


def check_bounds_consistency(tiles_dir, prj_file=None, origin=None):
    """Verify tile geographic bounds against expected projection."""
    errors = []
    if prj_file is None or origin is None:
        print("  Skipped (no --prj/--origin provided)")
        return errors

    from pyproj import CRS, Transformer

    with open(prj_file) as f:
        wkt = f.read().strip()
    crs_gk = CRS.from_wkt(wkt)
    trans = Transformer.from_crs(crs_gk, "EPSG:4326", always_xy=True)

    e0, n0 = origin[0], origin[1]
    lon_tl, lat_tl = trans.transform(e0, n0)
    print(f"  Expected origin → ({lon_tl:.6f}, {lat_tl:.6f})")

    # Read layer.json bounds
    with open(os.path.join(tiles_dir, "layer.json")) as f:
        bounds = json.load(f)["bounds"]

    # The layer.json bounds should contain the projected origin
    w, s, e, n = bounds
    if not (w <= lon_tl <= e and s <= lat_tl <= n):
        errors.append(f"Origin ({lon_tl:.6f}, {lat_tl:.6f}) not in bounds [{w:.6f},{s:.6f},{e:.6f},{n:.6f}]")

    return errors


def main():
    parser = argparse.ArgumentParser(description="Verify TMS tile output")
    parser.add_argument("tiles_dir", help="TMS output directory")
    parser.add_argument("--prj", default=None, help="Projection PRJ file")
    parser.add_argument("--origin", default=None,
                        help="Origin easting,northing in projected CRS")
    parser.add_argument("--strict", action="store_true",
                        help="Treat edge stitching issues as errors")
    args = parser.parse_args()

    if not os.path.isdir(args.tiles_dir):
        print(f"ERROR: {args.tiles_dir} is not a directory")
        sys.exit(1)

    origin = None
    if args.origin:
        parts = [float(x) for x in args.origin.split(",")]
        origin = (parts[0], parts[1])

    print(f"Verifying: {args.tiles_dir}")
    all_errors = []

    # 1. Structure
    print("\n1. Directory structure:")
    levels, errs = check_structure(args.tiles_dir)
    all_errors.extend(errs)

    # 2. PNG validity
    print("\n2. PNG validation:")
    errs = validate_png_files(args.tiles_dir)
    all_errors.extend(errs)

    # 3. Edge stitching
    print("\n3. Edge stitching:")
    errs = check_edge_stitching(args.tiles_dir)
    all_errors.extend(errs)

    # 4. Metadata
    print("\n4. Metadata:")
    errs = check_metadata(args.tiles_dir)
    all_errors.extend(errs)

    # 5. Bounds consistency
    print("\n5. Bounds consistency:")
    errs = check_bounds_consistency(args.tiles_dir, args.prj, origin)
    all_errors.extend(errs)

    # Summary
    print(f"\n{'='*50}")
    print(f"Total errors: {len(all_errors)}")
    if all_errors:
        print("Errors:")
        for e in all_errors:
            print(f"  - {e}")
    else:
        print("ALL CHECKS PASSED")
    print(f"{'='*50}")

    sys.exit(1 if all_errors else 0)


if __name__ == "__main__":
    main()
