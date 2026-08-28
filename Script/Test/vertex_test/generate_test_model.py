#!/usr/bin/env python3
"""
Generate a synthetic OBJ model for TilesConverter vertex verification.

Creates 100 vertices (10×10 grid) on a horizontal plane, spanning 50km
in the CGCS2000 GK CM 103d10mE projection zone.

OBJ vertices are in Assimp Y-up convention:
  X = East  (0 .. 50000 m)
  Y = Up    (0, constant)
  Z = North (0 .. 50000 m)

Origin: GK (498700, 2929900, 0)
"""

import os

ORIGIN_E = 498700.0
ORIGIN_N = 2929900.0
GRID_SIZE = 10          # 10×10 = 100 vertices
SPAN_METERS = 50000.0   # 50 km

out_dir = os.path.dirname(os.path.abspath(__file__))
obj_path = os.path.join(out_dir, "test_plane.obj")

with open(obj_path, "w") as f:
    f.write("# Test plane: 100 vertices, 50km span, GK projection\n")
    f.write(f"# Origin: ({ORIGIN_E}, {ORIGIN_N}, 0)\n")
    f.write(f"# Assimp Y-up: X=East, Y=Up, Z=North\n\n")

    # Vertices
    for i in range(GRID_SIZE):
        for j in range(GRID_SIZE):
            east  = (i / (GRID_SIZE - 1)) * SPAN_METERS
            up    = 0.0
            north = (j / (GRID_SIZE - 1)) * SPAN_METERS
            f.write(f"v {east:.4f} {up:.4f} {north:.4f}\n")

    f.write("\n")

    # Faces (triangles for each 2×2 quad)
    for i in range(GRID_SIZE - 1):
        for j in range(GRID_SIZE - 1):
            a = i * GRID_SIZE + j + 1       # 1-indexed
            b = (i + 1) * GRID_SIZE + j + 1
            c = (i + 1) * GRID_SIZE + (j + 1) + 1
            d = i * GRID_SIZE + (j + 1) + 1
            f.write(f"f {a} {d} {b}\n")
            f.write(f"f {b} {d} {c}\n")

print(f"Generated: {obj_path}")
print(f"  Vertices: {GRID_SIZE * GRID_SIZE}")
print(f"  Span: {SPAN_METERS/1000:.0f} km × {SPAN_METERS/1000:.0f} km")
print(f"  Origin: GK ({ORIGIN_E}, {ORIGIN_N}, 0)")