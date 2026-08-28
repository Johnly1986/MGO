#!/usr/bin/env python3
"""
Verify TilesConverter vertex transformation accuracy.

Simulates the full projection pipeline for 100 test vertices
spanning 50km, and computes the ECEF error introduced by:
  1. Per-instance delta correction (translation-only)
  2. CesiumJS Y_UP_TO_Z_UP + root transform chain
"""

import math
import numpy as np
from pyproj import CRS, Transformer

# ============================================================================
# Test parameters
# ============================================================================
ORIGIN_E = 498700.0
ORIGIN_N = 2929900.0
ORIGIN_Z = 0.0
GRID_SIZE = 10
SPAN = 50000.0  # meters

# ============================================================================
# Geodetic math (mirrors GeodeticMath / CProjectionEngine)
# ============================================================================
A = 6378137.0
F_INV = 298.257222101
F = 1.0 / F_INV
E2 = 2.0 * F - F * F

def geographic_to_ecef(lat_rad, lon_rad, h):
    """GeodeticMath::GeographicToECEF"""
    sin_lat = math.sin(lat_rad)
    cos_lat = math.cos(lat_rad)
    sin_lon = math.sin(lon_rad)
    cos_lon = math.cos(lon_rad)
    N = A / math.sqrt(1.0 - E2 * sin_lat * sin_lat)
    x = (N + h) * cos_lat * cos_lon
    y = (N + h) * cos_lat * sin_lon
    z = (N * (1.0 - E2) + h) * sin_lat
    return np.array([x, y, z])

def enu_to_ecef_rotation(lat_rad, lon_rad):
    """GeodeticMath::ENUToECEFRotation — returns 3x3 row-major"""
    sLon = math.sin(lon_rad)
    cLon = math.cos(lon_rad)
    sLat = math.sin(lat_rad)
    cLat = math.cos(lat_rad)
    R = np.zeros((3, 3))
    R[0, 0] = -sLon;      R[0, 1] = -sLat * cLon;  R[0, 2] = cLat * cLon  # East
    R[1, 0] =  cLon;      R[1, 1] = -sLat * sLon;  R[1, 2] = cLat * sLon  # North
    R[2, 0] =  0.0;       R[2, 1] =  cLat;         R[2, 2] = sLat         # Up
    return R

# ============================================================================
# Setup projection
# ============================================================================
with open("/root/coding/MGO/Data/103d10m.prj") as f:
    prj_wkt = f.read().strip()

crs_gk = CRS.from_wkt(prj_wkt)
to_wgs84 = Transformer.from_crs(crs_gk, "EPSG:4326", always_xy=True)
from_wgs84 = Transformer.from_crs("EPSG:4326", crs_gk, always_xy=True)

# Origin ECEF
origin_lon, origin_lat = to_wgs84.transform(ORIGIN_E, ORIGIN_N)
origin_lat_rad = math.radians(origin_lat)
origin_lon_rad = math.radians(origin_lon)
T_origin = geographic_to_ecef(origin_lat_rad, origin_lon_rad, ORIGIN_Z)
R_enu = enu_to_ecef_rotation(origin_lat_rad, origin_lon_rad)

print(f"Origin GK: ({ORIGIN_E}, {ORIGIN_N}, {ORIGIN_Z})")
print(f"Origin WGS84: ({origin_lon:.8f}, {origin_lat:.8f})")
print(f"Origin ECEF: ({T_origin[0]:.3f}, {T_origin[1]:.3f}, {T_origin[2]:.3f})")
print()

# ============================================================================
# Instance center (for delta computation)
# ============================================================================
cx = SPAN / 2.0   # East = 25000
cy = 0.0           # Up = 0
cz = SPAN / 2.0   # North = 25000

# True ECEF at center
center_lon, center_lat = to_wgs84.transform(ORIGIN_E + cx, ORIGIN_N + cz)
center_ecef = geographic_to_ecef(math.radians(center_lat), math.radians(center_lon), cy)

# Approximate ECEF at center (original ENU convention)
center_approx = R_enu @ np.array([cx, cz, cy]) + T_origin

# Delta in ECEF
delta_ecef = center_ecef - center_approx

# Delta in ENU (R_enu^T * delta_ecef)
delta_enu = R_enu.T @ delta_ecef

# Delta in Assimp (ENU→Assimp: East, Up, North)
dx = delta_enu[0]   # East
dy = delta_enu[2]   # Up (ENU Z)
dz = delta_enu[1]   # North (ENU Y)

# CesiumJS applies Y_UP_TO_Z_UP which negates North.
# Pre-negate the North delta so it cancels out:
#   Y_UP_TO_Z_UP * (east+dx, up+dy, north-dz)
#   = (east+dx, -(north-dz), up+dy)
#   = (east+dx, -north+dz, up+dy)  ← dz is now positive in ENU ✓

print(f"=== Delta at center ({cx:.0f}, {cy:.0f}, {cz:.0f}) ===")
print(f"Center ECEF (true):    ({center_ecef[0]:.3f}, {center_ecef[1]:.3f}, {center_ecef[2]:.3f})")
print(f"Center ECEF (approx):  ({center_approx[0]:.3f}, {center_approx[1]:.3f}, {center_approx[2]:.3f})")
print(f"Delta ECEF:            ({delta_ecef[0]:.3f}, {delta_ecef[1]:.3f}, {delta_ecef[2]:.3f})")
print(f"Delta ECEF magnitude:  {np.linalg.norm(delta_ecef):.3f} m")
print(f"Delta Assimp:          dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f}")
print()

# ============================================================================
# Per-vertex verification
# ============================================================================
min_err = float('inf')
max_err = 0.0
errors = []

# Root transform with negated North column (matching C++ fix)
R_fixed = R_enu.copy()
R_fixed[:, 1] = -R_fixed[:, 1]  # column 1 = North axis, negated for Y_UP_TO_Z_UP

print("=== Per-vertex ECEF error (CesiumJS rendering chain) ===")
print(f"{'Idx':>4} {'East(m)':>10} {'North(m)':>10} {'TrueECEF':>30} {'RenderedECEF':>30} {'Err(m)':>10}")
print("-" * 100)

for i in range(GRID_SIZE):
    for j in range(GRID_SIZE):
        idx = i * GRID_SIZE + j
        east  = (i / (GRID_SIZE - 1)) * SPAN
        up    = 0.0
        north = (j / (GRID_SIZE - 1)) * SPAN

        # True ECEF (full GK pipeline)
        lon, lat = to_wgs84.transform(ORIGIN_E + east, ORIGIN_N + north)
        true_ecef = geographic_to_ecef(math.radians(lat), math.radians(lon), up)

        # glTF output vertex (with delta applied as translation)
        gltf_east  = east + dx
        gltf_up    = up + dy
        gltf_north = north + dz   # delta applied normally; root transform handles negation

        # CesiumJS rendering chain:
        # Y_UP_TO_Z_UP * (east, up, north) = (east, -north, up)
        # R_enu * (east, -north, up) + T_origin
        rendered = R_fixed @ np.array([gltf_east, -gltf_north, gltf_up]) + T_origin

        # Error
        err = np.linalg.norm(true_ecef - rendered) * 100.0  # cm

        if idx < 5 or idx > 94 or idx % 10 == 0:
            print(f"{idx:4d} {east:10.1f} {north:10.1f} "
                  f"({true_ecef[0]:10.3f},{true_ecef[1]:10.3f},{true_ecef[2]:10.3f}) "
                  f"({rendered[0]:10.3f},{rendered[1]:10.3f},{rendered[2]:10.3f}) "
                  f"{err:8.1f}cm")

        errors.append(err)
        min_err = min(min_err, err)
        max_err = max(max_err, err)

errors = np.array(errors)
print(f"\n=== Error Statistics (100 vertices, 50km span) ===")
print(f"  Min:    {min_err:.2f} cm")
print(f"  Max:    {max_err:.2f} cm")
print(f"  Mean:   {errors.mean():.2f} cm")
print(f"  Median: {np.median(errors):.2f} cm")
print(f"  Std:    {errors.std():.2f} cm")
print(f"  RMSE:   {np.sqrt((errors**2).mean()):.2f} cm")

# ============================================================================
# Error decomposition
# ============================================================================
print(f"\n=== Error decomposition ===")

# Error at center (should be ~0 since delta is computed there)
center_err = errors[GRID_SIZE * GRID_SIZE // 2 + GRID_SIZE // 2]
print(f"  Error at center (delta reference): {center_err:.2f} cm")

# Error at corners (farthest from center)
corner_indices = [0, GRID_SIZE-1, GRID_SIZE*(GRID_SIZE-1), GRID_SIZE*GRID_SIZE-1]
corner_labels = ["SW", "SE", "NW", "NE"]
for idx, label in zip(corner_indices, corner_labels):
    print(f"  Error at {label} corner: {errors[idx]:.2f} cm")

# Error due to using center delta for all vertices (translation-only approximation)
# vs per-vertex full pipeline
print(f"\n  Translation-only delta error increases with distance from center.")
print(f"  Center: {GRID_SIZE//2},{GRID_SIZE//2} → error near zero")
print(f"  Corners: ~{np.mean([errors[i] for i in corner_indices]):.2f} cm")
print(f"  Error gradient: ~{max_err/SPAN*1000:.4f} cm/km from center")

# ============================================================================
# North-negation impact (BUG-4 from audit)
# ============================================================================
print(f"\n=== North-negation impact (BUG-4 audit) ===")
# Without north-negation: R_enu * (east+dx, north+dz, up+dy) + T_origin
# With north-negation:    R_enu * (east+dx, -north-dz, up+dy) + T_origin
# Difference: R_enu * (0, 2*(north+dz), 0)

for north_val in [0, SPAN/4, SPAN/2, 3*SPAN/4, SPAN]:
    center_test = SPAN / 2  # center easting
    # Delta at this north position
    gltf_e = center_test + dx
    gltf_n = north_val + dz
    gltf_u = up + dy

    with_north_neg = R_fixed @ np.array([gltf_e, -gltf_n, gltf_u]) + T_origin
    without_north_neg = R_enu @ np.array([gltf_e, gltf_n, gltf_u]) + T_origin

    diff = np.linalg.norm(with_north_neg - without_north_neg)
    lon, lat = to_wgs84.transform(ORIGIN_E + center_test, ORIGIN_N + north_val)
    true = geographic_to_ecef(math.radians(lat), math.radians(lon), up)
    err_with = np.linalg.norm(true - with_north_neg) * 100
    err_without = np.linalg.norm(true - without_north_neg) * 100

    print(f"  North={north_val:6.0f}m: neg-diff={diff:.2f}m, "
          f"err_with={err_with:.1f}cm, err_without={err_without:.1f}cm")