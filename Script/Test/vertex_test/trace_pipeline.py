#!/usr/bin/env python3
"""
End-to-end trace of the TilesConverter coordinate pipeline.

Mirrors the EXACT C++ code path:
  1. CollectMeshInstances: world-space bbox in Assimp Y-up
  2. CProjectionEngine::ComputeInstanceProjectionDelta
  3. ApplyInstanceCorrection: dx,dy,dz → worldTransform
  4. GroupCellByMaterial: worldTransform * vertex → glTF
  5. WriteBoxJson / AxisMapper::BBoxAssimpToTilesZUp: Y-up bbox → Z-up
  6. ComputeRootTransform: ENU→ECEF 4x4 matrix
  7. CesiumJS rendering chain: Y_UP_TO_Z_UP * glTF * rootTransform

Tests THREE configurations:
  A) BROKEN (current): No wz=-wz, No North-negation in root transform
  B) VERTEX-FIX: wz=-wz in GroupCellByMaterial, clean ENU→ECEF root
  C) TRANSFORM-FIX: North column negated in root transform, clean vertices
"""

import math
import numpy as np

# ============================================================================
# Parameters (matching Data/103d10m.prj and routeOriginPt.txt)
# ============================================================================
ORIGIN_E = 498700.0
ORIGIN_N = 2929900.0
ORIGIN_Z = 0.0

# PRJ parameters: CGCS2000_3_Degree_GK_CM_103d10mE
# SPHEROID["CGCS2000",6378137.0,298.257222101]
# Central_Meridian=103.1666666666666667
# False_Easting=500000.0
A = 6378137.0
F_INV = 298.257222101

# ============================================================================
# Geodetic math (exact replicas of C++ code)
# ============================================================================
DEG2RAD = math.pi / 180.0

def geographic_to_ecef(lat_rad, lon_rad, h):
    """GeodeticMath::GeographicToECEF"""
    sin_lat = math.sin(lat_rad)
    cos_lat = math.cos(lat_rad)
    cos_lon = math.cos(lon_rad)
    sin_lon = math.sin(lon_rad)
    f = 1.0 / F_INV
    e2 = 2.0 * f - f * f
    N = A / math.sqrt(1.0 - e2 * sin_lat * sin_lat)
    X = (N + h) * cos_lat * cos_lon
    Y = (N + h) * cos_lat * sin_lon
    Z = (N * (1.0 - e2) + h) * sin_lat
    return np.array([X, Y, Z])

def ecef_to_geographic(X, Y, Z):
    """GeodeticMath::ECEFToGeographic - Bowring iterative"""
    lon = math.atan2(Y, X)
    p = math.sqrt(X * X + Y * Y)
    f = 1.0 / F_INV
    e2 = 2.0 * f - f * f
    lat = math.atan2(Z, p * (1.0 - e2))
    for _ in range(10):
        sl = math.sin(lat)
        N = A / math.sqrt(1.0 - e2 * sl * sl)
        new_lat = math.atan2(Z + e2 * N * sl, p)
        if abs(new_lat - lat) < 1e-12:
            lat = new_lat
            break
        lat = new_lat
    sl = math.sin(lat)
    N = A / math.sqrt(1.0 - e2 * sl * sl)
    h = p / math.cos(lat) - N
    return lat, lon, h

def enu_to_ecef_rotation(lat_rad, lon_rad):
    """GeodeticMath::ENUToECEFRotation — row-major 3×3"""
    sLon = math.sin(lon_rad)
    cLon = math.cos(lon_rad)
    sLat = math.sin(lat_rad)
    cLat = math.cos(lat_rad)
    R = np.zeros((3, 3))
    R[0, 0] = -sLon;  R[0, 1] = -sLat * cLon;  R[0, 2] = cLat * cLon  # row 0
    R[1, 0] =  cLon;  R[1, 1] = -sLat * sLon;  R[1, 2] = cLat * sLon  # row 1
    R[2, 0] =  0.0;   R[2, 1] =  cLat;         R[2, 2] = sLat         # row 2
    return R

def gk_inverse(E, N, lambda0_deg=103.1666666666666667):
    """GeodeticMath::GKInverse — Gauss-Kruger inverse"""
    f = 1.0 / F_INV
    e2 = 2.0 * f - f * f
    ep2 = e2 / (1.0 - e2)
    lambda0 = lambda0_deg * DEG2RAD
    k0 = 1.0
    FE = 500000.0
    FN = 0.0

    x = E - FE
    y = N - FN
    M = y / k0

    # Foot-point latitude
    e4 = e2 * e2
    e6 = e4 * e2
    m0 = A * (1.0 - e2 / 4.0 - 3.0 * e4 / 64.0 - 5.0 * e6 / 256.0)
    phi = M / m0
    for _ in range(6):
        m2 = A * (-3.0 * e2 / 8.0 - 3.0 * e4 / 32.0 - 45.0 * e6 / 1024.0)
        m4 = A * (15.0 * e4 / 256.0 + 45.0 * e6 / 1024.0)
        m6 = A * (-35.0 * e6 / 3072.0)
        arc = m0 * phi + m2 * math.sin(2.0 * phi) + m4 * math.sin(4.0 * phi) + m6 * math.sin(6.0 * phi)
        dphi = M - arc
        sinphi = math.sin(phi)
        Mprime = A * (1.0 - e2) / math.pow(1.0 - e2 * sinphi * sinphi, 1.5)
        phi += dphi / Mprime
        if abs(dphi) < 1e-12:
            break

    sin_f = math.sin(phi)
    cos_f = math.cos(phi)
    tan_f = sin_f / cos_f
    t_f2 = tan_f * tan_f
    t_f4 = t_f2 * t_f2
    eta_f2 = ep2 * cos_f * cos_f
    eta_f4 = eta_f2 * eta_f2

    nu_f = A / math.sqrt(1.0 - e2 * sin_f * sin_f)
    rho_f = A * (1.0 - e2) / math.pow(1.0 - e2 * sin_f * sin_f, 1.5)

    D = x / (k0 * nu_f)
    D2 = D * D
    D3 = D2 * D
    D4 = D3 * D
    D5 = D4 * D
    D6 = D5 * D
    D7 = D6 * D

    # Lat
    lat = phi
    lat -= (tan_f / (2.0 * rho_f)) * (x * D / k0)
    lat += (tan_f / (24.0 * rho_f)) * (x * D3 / k0) * (5.0 + 3.0 * t_f2 + eta_f2 - 4.0 * eta_f4 - 9.0 * eta_f2 * t_f2)
    lat -= (tan_f / (720.0 * rho_f)) * (x * D5 / k0) * (61.0 + 90.0 * t_f2 + 45.0 * t_f4 + 46.0 * eta_f2 - 252.0 * eta_f2 * t_f2 - 3.0 * eta_f4 + 100.0 * eta_f4 * t_f2 - 66.0 * eta_f2 * t_f4 - 90.0 * eta_f4 * t_f4)

    # Lon
    lon = lambda0
    lon += D / cos_f
    lon -= D3 / (6.0 * cos_f) * (1.0 + 2.0 * t_f2 + eta_f2)
    lon += D5 / (120.0 * cos_f) * (5.0 + 28.0 * t_f2 + 24.0 * t_f4 + 6.0 * eta_f2 + 8.0 * eta_f2 * t_f2)
    lon -= D7 / (5040.0 * cos_f) * (61.0 + 662.0 * t_f2 + 1320.0 * t_f4 + 720.0 * math.pow(tan_f, 6))

    return lat, lon

# AxisMapper functions (exact replicas of C++ code)
def assimp_to_enu(x, y, z):
    """AxisMapper::AssimpToENU: (East, Up, North) → (East, North, Up)"""
    return (x, z, y)

def enu_to_assimp(east, north, up):
    """AxisMapper::ENUToAssimp: (East, North, Up) → (East, Up, North)"""
    return (east, up, north)

def bbox_assimp_to_tiles_zup(bmin, bmax):
    """AxisMapper::BBoxAssimpToTilesZUp"""
    outMin = np.array([bmin[0], -bmax[2], bmin[1]])
    outMax = np.array([bmax[0], -bmin[2], bmax[1]])
    return outMin, outMax

# ============================================================================
# Test points: 4 corners + center of a 1km × 1km model
# ============================================================================
# Model center in Assimp space (instance centroid)
CX = 500.0
CY = 0.0
CZ = 500.0

# Test vertices (model-local coords)
test_vertices_assimp = [
    np.array([-500.0, 0.0, -500.0]),  # SW corner
    np.array([ 500.0, 0.0, -500.0]),  # SE corner
    np.array([-500.0, 0.0,  500.0]),  # NW corner
    np.array([ 500.0, 0.0,  500.0]),  # NE corner
    np.array([   0.0, 0.0,    0.0]),  # center
]

# ============================================================================
# Step 1: Compute origin ECEF and ENU rotation
# ============================================================================
origin_lat, origin_lon = gk_inverse(ORIGIN_E, ORIGIN_N)
T_origin = geographic_to_ecef(origin_lat, origin_lon, ORIGIN_Z)
R_enu = enu_to_ecef_rotation(origin_lat, origin_lon)
Rt = R_enu.T  # transpose

print("=" * 80)
print("STEP 0: Origin")
print(f"  GK: ({ORIGIN_E}, {ORIGIN_N}, {ORIGIN_Z})")
print(f"  Geographic: lat={math.degrees(origin_lat):.8f} lon={math.degrees(origin_lon):.8f}")
print(f"  ECEF: ({T_origin[0]:.3f}, {T_origin[1]:.3f}, {T_origin[2]:.3f})")

# ============================================================================
# Step 2: ComputeInstanceProjectionDelta (exact C++ logic)
# ============================================================================
print("\n" + "=" * 80)
print("STEP 1: ComputeInstanceProjectionDelta at instance centroid")
print(f"  centroid Assimp: cx={CX}, cy={CY}, cz={CZ}")

# Convert Assimp → ENU
enu_e, enu_n, enu_u = assimp_to_enu(CX, CY, CZ)
print(f"  centroid ENU:    e={enu_e}, n={enu_n}, u={enu_u}")

# True ECEF at centroid (full GK pipeline)
ctr_lat, ctr_lon = gk_inverse(ORIGIN_E + enu_e, ORIGIN_N + enu_n)
ctr_ecef = geographic_to_ecef(ctr_lat, ctr_lon, enu_u + ORIGIN_Z)
print(f"  centroid geographic: lat={math.degrees(ctr_lat):.8f} lon={math.degrees(ctr_lon):.8f}")
print(f"  True ECEF:    ({ctr_ecef[0]:.3f}, {ctr_ecef[1]:.3f}, {ctr_ecef[2]:.3f})")

# Approximate ECEF (single ENU→ECEF rotation at origin)
approx_ecef = R_enu @ np.array([enu_e, enu_n, enu_u]) + T_origin
print(f"  Approx ECEF:  ({approx_ecef[0]:.3f}, {approx_ecef[1]:.3f}, {approx_ecef[2]:.3f})")

# Delta in ECEF
delta_ecef = ctr_ecef - approx_ecef
print(f"  Delta ECEF:   ({delta_ecef[0]:.3f}, {delta_ecef[1]:.3f}, {delta_ecef[2]:.3f})")
print(f"  Delta ECEF magnitude: {np.linalg.norm(delta_ecef):.4f} m")

# Delta in ENU (Rt * delta_ecef)
dEast  = Rt[0, 0]*delta_ecef[0] + Rt[0, 1]*delta_ecef[1] + Rt[0, 2]*delta_ecef[2]
dNorth = Rt[1, 0]*delta_ecef[0] + Rt[1, 1]*delta_ecef[1] + Rt[1, 2]*delta_ecef[2]
dUp    = Rt[2, 0]*delta_ecef[0] + Rt[2, 1]*delta_ecef[1] + Rt[2, 2]*delta_ecef[2]
print(f"  Delta ENU: dEast={dEast:.4f}, dNorth={dNorth:.4f}, dUp={dUp:.4f}")

# ENU→Assimp
dx, dy, dz = enu_to_assimp(dEast, dNorth, dUp)
print(f"  Delta Assimp (worldTransform offset): dx={dx:.4f}, dy={dy:.4f}, dz={dz:.4f}")

# ============================================================================
# Step 3: ApplyInstanceCorrection — delta → worldTransform translation
# ============================================================================
print("\n" + "=" * 80)
print("STEP 2: ApplyInstanceCorrection (delta → worldTransform[3,7,11])")
print(f"  worldTransform[3] += {dx:.4f} (East)")
print(f"  worldTransform[7] += {dy:.4f} (Up)")
print(f"  worldTransform[11] += {dz:.4f} (North)")

# ============================================================================
# Step 4: GroupCellByMaterial — world × vertex → glTF output
# ============================================================================
print("\n" + "=" * 80)
print("STEP 3: GroupCellByMaterial (world * meshVertex + delta → glTF)")
print("  Tests: (A) BROKEN (no wz=-wz), (B) VERTEX-FIX (wz=-wz)")
print()

for label_prefix, apply_wz_neg in [("A) BROKEN  ", False), ("B) FIXED   ", True)]:
    print(f"  === {label_prefix} (wz_neg={apply_wz_neg}) ===")
    for vi, v_local in enumerate(test_vertices_assimp):
        # worldTransform * vertex (identity rotation + delta translation)
        v_world = v_local + np.array([dx, dy, dz])

        # glTF output convention
        gltf = v_world.copy()
        if apply_wz_neg:
            gltf[2] = -gltf[2]  # wz = -wz: Assimp North → glTF South

        labels = ["SW", "SE", "NW", "NE", "CTR"]
        print(f"    {labels[vi]:>3} local{v_local} → world{v_world} → glTF{gltf}")

# ============================================================================
# Step 5: WriteBoxJson — Y-up bbox → Z-up for tileset.json
# ============================================================================
print("\n" + "=" * 80)
print("STEP 4: BBoxUtils::WriteBoxJson / AxisMapper::BBoxAssimpToTilesZUp")
print("  (bbox is always in Y-up by this point — GroupCellByMaterial normalized it)")

# Model bbox in Y-up (after delta applied)
bbox_min_yup = np.array([CX - 500 + dx, CY - 500 + dy, CZ - 500 + dz])
bbox_max_yup = np.array([CX + 500 + dx, CY + 500 + dy, CZ + 500 + dz])
print(f"  BBox Y-up (Assimp): min={bbox_min_yup}, max={bbox_max_yup}")

bbox_min_zup, bbox_max_zup = bbox_assimp_to_tiles_zup(bbox_min_yup, bbox_max_yup)
print(f"  BBox Z-up (Tiles):  min={bbox_min_zup}, max={bbox_max_zup}")

# ============================================================================
# Step 6: ComputeRootTransform — ENU→ECEF 4x4
# ============================================================================
print("\n" + "=" * 80)
print("STEP 5: ComputeRootTransform — ENU→ECEF 4x4 matrix")
print("  (no North-negation in current code)")

# BuildRootTransform: column-major 4x4
# Column 0 (East), Column 1 (North), Column 2 (Up), Column 3 (Translation)
root_4x4 = np.eye(4)
root_4x4[:3, 0] = R_enu[:, 0]  # East column
root_4x4[:3, 1] = R_enu[:, 1]  # North column
root_4x4[:3, 2] = R_enu[:, 2]  # Up column
root_4x4[:3, 3] = T_origin      # Translation

# ============================================================================
# Step 7: CesiumJS rendering chain
# ============================================================================
# Y_UP_TO_Z_UP = [1,0,0,0; 0,0,-1,0; 0,1,0,0; 0,0,0,1]
# Maps (East, Up, North)_yup → (East, -North, Up)_zup
def y_up_to_z_up(v):
    """CesiumJS Y_UP_TO_Z_UP: (x,y,z)_yup → (x,-z,y)_zup"""
    return np.array([v[0], -v[2], v[1]])

print("\n" + "=" * 80)
print("STEP 6: CesiumJS rendering chain")
print("  Compares 3 configurations for each vertex:")
print()
print(f"{'Vtx':>4} {'True ECEF':>45} {'Err A':>10} {'Err B':>10} {'Err C':>10}")
print(f"{'':>4} {'':>45} {'(broken)':>10} {'(vtx-fix)':>10} {'(xform-fix)':>10}")
print("-" * 95)

errors = {"A": [], "B": [], "C": []}

for vi, v_local in enumerate(test_vertices_assimp):
    # World-space position (after delta applied via worldTransform)
    v_world = v_local + np.array([dx, dy, dz])

    # True ECEF (full GK pipeline for this exact world-space position)
    ve, vn, vu = assimp_to_enu(v_world[0], v_world[1], v_world[2])
    v_lat, v_lon = gk_inverse(ORIGIN_E + ve, ORIGIN_N + vn)
    true_ecef = geographic_to_ecef(v_lat, v_lon, vu + ORIGIN_Z)

    # ---- Config A: BROKEN (current code) ----
    # glTF: (East, Up, North) — no wz=-wz
    # Y_UP_TO_Z_UP: (East, -North, Up)
    # Root transform: ENU→ECEF without North-negation
    gltf_A = v_world.copy()
    tiles_A = y_up_to_z_up(gltf_A)
    rendered_A = root_4x4 @ np.append(tiles_A, 1.0)
    err_A = np.linalg.norm(true_ecef - rendered_A[:3]) * 100  # cm

    # ---- Config B: VERTEX-FIX (wz=-wz in GroupCellByMaterial) ----
    # glTF: (East, Up, -North) — with wz=-wz
    # Y_UP_TO_Z_UP: (East, -(-North), Up) = (East, North, Up)
    # Root transform: ENU→ECEF without North-negation
    gltf_B = v_world.copy()
    gltf_B[2] = -gltf_B[2]  # wz = -wz
    tiles_B = y_up_to_z_up(gltf_B)
    rendered_B = root_4x4 @ np.append(tiles_B, 1.0)
    err_B = np.linalg.norm(true_ecef - rendered_B[:3]) * 100

    # ---- Config C: TRANSFORM-FIX (North-negated column in root) ----
    # glTF: (East, Up, North) — no wz=-wz (same as A)
    # Y_UP_TO_Z_UP: (East, -North, Up)
    # Root transform: ENU→ECEF with column 1 negated
    root_C = root_4x4.copy()
    root_C[:3, 1] = -root_C[:3, 1]  # negate North column
    tiles_C = y_up_to_z_up(gltf_A)  # same tiles as A
    rendered_C = root_C @ np.append(tiles_C, 1.0)
    err_C = np.linalg.norm(true_ecef - rendered_C[:3]) * 100

    labels = ["SW", "SE", "NW", "NE", "CTR"]
    print(f"{labels[vi]:>4} ({true_ecef[0]:.1f},{true_ecef[1]:.1f},{true_ecef[2]:.1f}) "
          f"{err_A:8.1f}cm {err_B:8.1f}cm {err_C:8.1f}cm")

    errors["A"].append(err_A)
    errors["B"].append(err_B)
    errors["C"].append(err_C)

print("-" * 95)
for cfg in ["A", "B", "C"]:
    labels = {"A": "BROKEN (current)", "B": "VERTEX-FIX (wz=-wz)", "C": "TRANSFORM-FIX (neg col1)"}
    errs = np.array(errors[cfg])
    print(f"  {labels[cfg]}: max={errs.max():.1f}cm, mean={errs.mean():.1f}cm, "
          f"center={errs[-1]:.1f}cm")

# ============================================================================
# Step 8: North-negation sign analysis
# ============================================================================
print("\n" + "=" * 80)
print("STEP 7: North-negation sign analysis")
print("  What happens to a vertex at Assimp (0, 0, +Z) = North?")

for z_meters in [1.0, 10.0, 100.0]:
    v = np.array([0.0, 0.0, z_meters])
    v_w = v + np.array([dx, dy, dz])

    # True ECEF of this point
    ve, vn, vu = assimp_to_enu(v_w[0], v_w[1], v_w[2])
    v_lat, v_lon = gk_inverse(ORIGIN_E + ve, ORIGIN_N + vn)
    true = geographic_to_ecef(v_lat, v_lon, vu + ORIGIN_Z)

    # Broken (Config A)
    tiles = y_up_to_z_up(v_w)
    broken_ecef = root_4x4 @ np.append(tiles, 1.0)

    # Fixed (Config B)
    v_w_fixed = v_w.copy()
    v_w_fixed[2] = -v_w_fixed[2]
    tiles_fixed = y_up_to_z_up(v_w_fixed)
    fixed_ecef = root_4x4 @ np.append(tiles_fixed, 1.0)

    err_broken = np.linalg.norm(true - broken_ecef[:3])
    err_fixed = np.linalg.norm(true - fixed_ecef[:3])

    print(f"  North={z_meters:6.1f}m: broken_err={err_broken:.4f}m, fixed_err={err_fixed:.4f}m")

# ============================================================================
# Step 9: Summary of root cause
# ============================================================================
print("\n" + "=" * 80)
print("ROOT CAUSE ANALYSIS")
print("=" * 80)
print()
print("Standard glTF convention:  +X=East, +Y=Up, +Z=South (forward)")
print("Assimp convention in code: +X=East, +Y=Up, +Z=North")
print()
print("CesiumJS Y_UP_TO_Z_UP = [1,0,0; 0,0,-1; 0,1,0]")
print("Maps glTF (East, Up, Z) → Cesium (East, -Z, Up)")
print()
print("With Assimp convention (Z=North):")
print("  Y_UP_TO_Z_UP × (E, U, N) = (E, -N, U)  ← North becomes negative!")
print()
print("The root transform (ENU→ECEF) expects (East, North, Up),")
print("but receives (East, -North, Up) from CesiumJS.")
print("This negates all North components in the final ECEF position.")
print()
print("Fix options (mathematically equivalent):")
print("  A) wz=-wz in GroupCellByMaterial: converts Assimp→glTF convention")
print("     glTF (E, U, -N) → Y_UP_TO_Z_UP → (E, N, U) ✓")
print("  B) Negate North column of root transform")
print("     root_neg × (E, -N, U) = E*East + (-N)*(-North) + U*Up ✓")
print()
print("Neither fix is currently active in the code (commit 6a7f9528).")
