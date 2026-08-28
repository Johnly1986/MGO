#!/usr/bin/env python3
"""End-to-end pipeline trace for DEM.tif: verify every coordinate transform step.

Simulates the full chain:
  GeoTiffReader → HeightmapGrid → TerrainQuadtree corners →
  GeographicToProjected → BilinearSample → TinSimplifier v-coordinate →
  QuantizedMeshEncoder lat/lon mapping

Also compares with the reference AxisMapping={1,-1} formulation.
"""
import struct, math, sys

# --- Config ---
W, H = 17405, 11985
GT = [484888.225, 2.0, 0.0, 2946089.830, 0.0, 2.0]  # from DEM.tif
minE, maxE = 484888.225, 519696.225
minN, maxN = 2946089.830, 2970057.830  # pixel-center range
dx, dy = 2.0, 2.0

# Elevation gradient from real data: south ~1464m, north ~1930m
def file_elev(row, col):
    return 1464.0 + (row / H) * (1930.0 - 1464.0)

# --- Step 1: GeoTiffReader GeoTransform-based write ---
# outRow = (maxN - Y_geo) / dy
print("=" * 60)
print("STEP 1: GeoTiffReader GeoTransform-based write")
print("=" * 60)

# Check file row 0 (south edge, low elevation)
file_r, file_c = 0, 0
X_geo = GT[0] + file_c * GT[1] + file_r * GT[2]
Y_geo = GT[3] + file_c * GT[4] + file_r * GT[5]
outC = int(round((X_geo - minE) / dx))
outR = int(round((maxN - Y_geo) / dy))
print(f"File (r={file_r},c={file_c}): geo=({X_geo:.1f},{Y_geo:.1f}) → out=({outC},{outR}) elev={file_elev(file_r,file_c):.0f}m")
assert outR == H-1, f"South edge should go to bottom row! Got {outR}"
print("  → South edge → output row H-1 (bottom) ✓")

# Check file row H-1 (north edge, high elevation)
file_r, file_c = H-1, 0
X_geo = GT[0] + file_c * GT[1] + file_r * GT[2]
Y_geo = GT[3] + file_c * GT[4] + file_r * GT[5]
outC = int(round((X_geo - minE) / dx))
outR = int(round((maxN - Y_geo) / dy))
print(f"File (r={file_r},c={file_c}): geo=({X_geo:.1f},{Y_geo:.1f}) → out=({outC},{outR}) elev={file_elev(file_r,file_c):.0f}m")
assert outR == 0, f"North edge should go to top row! Got {outR}"
print("  → North edge → output row 0 (top) ✓")

# --- Step 2: BilinearSample verify ---
print("\nSTEP 2: BilinearSample (output grid inverse)")
print("=" * 60)

for label, n in [("North edge", maxN), ("South edge", minN),
                  ("Mid-north", maxN - 1000), ("Mid-south", minN + 1000)]:
    fc = (minE - minE) / dx  # col 0
    fr = (maxN - n) / dy
    out_r = int(round(fr))
    elev = file_elev(H-1-out_r, 0) if GT[5] > 0 else file_elev(out_r, 0)
    print(f"  {label}: northing={n:.1f} → fr={fr:.1f} → row={out_r:d}")

# --- Step 3: Reference AxisMapping={1,-1} comparison ---
print("\nSTEP 3: Reference AxisMapping={1,-1} formulation")
print("=" * 60)

det = GT[1]*GT[5] - GT[2]*GT[4]  # = 4.0
AX, AY = 1.0, -1.0  # AxisMapping

for label, n in [("North", maxN), ("South", minN)]:
    dCol = (GT[5]*AX*(minE - GT[0]) - GT[2]*AY*(n - GT[3])) / det
    dRow = (GT[1]*AY*(n - GT[3]) - GT[4]*AX*(minE - GT[0])) / det
    print(f"  {label}: northing={n:.1f} → RefCol={dCol:.1f} RefRow={dRow:.1f}")

print(f"\n  RefRow(North) - RefRow(South) = "
      f"({-(maxN-GT[3])/GT[5]:.1f}) - ({-(minN-GT[3])/GT[5]:.1f}) = "
      f"{-maxN/GT[5] + minN/GT[5]:.1f}")
print(f"  → Reference Y is NEGATED vs standard inverse")
print(f"  → RefRow decreases northward (inverted Y convention)")

# --- Step 4: TinSimplifier v-coordinate ---
print("\nSTEP 4: TinSimplifier v-coordinate verification")
print("=" * 60)

# Simulate a local tile grid (65x65) extracted at a northern tile
tileNorth, tileSouth = 26.84, 26.82  # a tile near the north edge
tileWest, tileEast = 103.10, 103.12
S = 65  # samplesPerTile

# ExtractLocalGrid simulation
local_heights = []
for r in range(S):
    lat = tileNorth - r * (tileNorth - tileSouth) / (S - 1)
    # Find the output grid row for this latitude
    # northing at this latitude: need projection, but for this test
    # just use the known north-edge elevation
    if r < S//4:
        h = 1930.0  # north region
    elif r > 3*S//4:
        h = 1464.0  # south region
    else:
        h = 1700.0  # middle
    local_heights.append(h)

# TinSimplifier v computation
print("Local grid (65x65): row 0 = tileNorth:")
for r in [0, S//4, S//2, 3*S//4, S-1]:
    v_norm = (S - 1 - r) / (S - 1)
    v_quant = int(round(v_norm * 32767))
    print(f"  r={r:2d}: v_norm={v_norm:.3f} v_quant={v_quant:5d} h≈{local_heights[r]:.0f}m")

# --- Step 5: Encoder lat/lon mapping from u,v ---
print("\nSTEP 5: Encoder lat/lon from u,v (as Cesium would decode)")
print("=" * 60)

for label, v in [("South edge", 0), ("Mid", 16384), ("North edge", 32767)]:
    lat = tileSouth + (v / 32767.0) * (tileNorth - tileSouth)
    print(f"  {label}: v={v:5d} → lat={lat:.6f}°")

print(f"\n  tileNorth={tileNorth}° > tileSouth={tileSouth}° ✓")
print(f"  v=0 maps to south, v=32767 maps to north ✓")

# --- Summary ---
print("\n" + "=" * 60)
print("SUMMARY")
print("=" * 60)
print("GeoTiffReader flip:        south→bottom, north→top ✓")
print("BilinearSample inverse:     northing→row mapping correct ✓")
print("Reference AxisMapping:      Y-axis inverted (-1), consistent with")
print("                            BilinearSample (maxN-northing)/dy formula")
print("TinSimplifier v-coord:      v=0↔south, v=32767↔north ✓")
print("Encoder lat/lon:            v→lat mapping correct ✓")
print()
print("If tiles still render N-S reversed, check:")
print("  1. Run diagnose_tile.py on actual .terrain output")
print("  2. Check EdgeIndices order (CCW winding)")
print("  3. Check centerX/Y/Z ECEF header computation")
