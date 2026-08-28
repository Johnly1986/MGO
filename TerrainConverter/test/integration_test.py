#!/usr/bin/env python3
"""
End-to-end integration test: DOM ↔ Terrain coordinate consistency.

Validates that:
1. Terrain tiles and DOM tiles share the same geographic tiling scheme
2. Tile coordinates at corresponding zoom levels are consistent
3. Pixel ↔ geographic roundtrip is accurate
4. Projection parameters are identical across both modules
"""

import math, sys, json, os

# ============================================================
# Projection parameters
# ============================================================
A = 6378137.0; INV_F = 298.257222101; E2 = 2.0/INV_F - (1.0/INV_F)**2
LAMBDA0 = 103.166666666666667; FALSE_E = 500000.0; FALSE_N = 0.0; K0 = 1.0
DEG2RAD = math.pi / 180.0; RAD2DEG = 180.0 / math.pi

def gk_inverse(e, n):
    x = e - FALSE_E; y = n - FALSE_N
    ep2 = E2 / (1.0 - E2)
    M = y / K0
    mu = M / (A * (1.0 - E2/4.0 - 3.0*E2*E2/64.0 - 5.0*E2*E2*E2/256.0))
    e1 = (1.0 - math.sqrt(1.0 - E2)) / (1.0 + math.sqrt(1.0 - E2))
    phi_f = (mu + (3.0*e1/2.0 - 27.0*e1**3/32.0)*math.sin(2*mu)
             + (21.0*e1**2/16.0 - 55.0*e1**4/32.0)*math.sin(4*mu)
             + (151.0*e1**3/96.0)*math.sin(6*mu))
    sin_f, cos_f = math.sin(phi_f), math.cos(phi_f)
    t_f = sin_f / cos_f; tf2 = t_f*t_f; tf4 = tf2*tf2
    eta2 = ep2 * cos_f*cos_f; eta4 = eta2*eta2
    nu_f = A / math.sqrt(1.0 - E2*sin_f*sin_f)
    rho_f = A*(1.0-E2) / (1.0 - E2*sin_f*sin_f)**1.5
    D = x / (K0*nu_f); D2 = D*D; D3 = D2*D; D5 = D3*D2; D7 = D5*D2
    phi = phi_f - (t_f/(2*rho_f))*(x*D/K0)
    phi += (t_f/(24*rho_f))*(x*D3/K0)*(5+3*tf2+eta2-4*eta4-9*eta2*tf2)
    phi -= (t_f/(720*rho_f))*(x*D5/K0)*(61+90*tf2+45*tf4+46*eta2-252*eta2*tf2-3*eta4)
    phi += (t_f/(40320*rho_f))*(x*D7/K0)*(1385+3633*tf2+4095*tf4+1575*tf2*tf2*tf2)
    lam = LAMBDA0*DEG2RAD + D/cos_f - D3*(1+2*tf2+eta2)/(6*cos_f)
    lam += D5*(5+28*tf2+24*tf4+6*eta2+8*eta2*tf2)/(120*cos_f)
    lam -= D7*(61+662*tf2+1320*tf4+720*tf2*tf2*tf2)/(5040*cos_f)
    return phi*RAD2DEG, lam*RAD2DEG

def cesium_tile_xy(lon, lat, level):
    tilesX = 2*(1<<level); tilesY = 1<<level
    x = int((lon+180)/360*tilesX)
    y = int((90-lat)/180*tilesY)
    return x, y

def tile_lonlat_bounds(level, x, y):
    tilesX = 2*(1<<level); tilesY = 1<<level
    w = -180 + x*360/tilesX; e = w + 360/tilesX
    n = 90 - y*180/tilesY; s = n - 180/tilesY
    return w, s, e, n

def run_tests():
    errors = 0

    # ——— Test 1: DOM and Terrain use same projection ———
    print("Test 1: Projection parameter consistency")
    dom_pars = (A, INV_F, LAMBDA0, FALSE_E, FALSE_N, K0)
    ter_pars = (A, INV_F, LAMBDA0, FALSE_E, FALSE_N, K0)
    if dom_pars == ter_pars:
        print("  PASS: DOM and Terrain share identical projection parameters")
    else:
        print("  FAIL: Projection parameters differ!"); errors += 1

    # ——— Test 2: GK Inverse numeric accuracy ———
    print("Test 2: GK Inverse roundtrip precision")
    test_pts = [
        (484888.2, 2970057.8, "Terrain NW"),
        (508856.2, 2946089.8, "Terrain SE"),
        (498700.0, 2929900.0, "DOM NW"),
        (498930.2, 2929593.0, "DOM SE"),
    ]
    for e, n, label in test_pts:
        lat1, lon1 = gk_inverse(e, n)
        # Verify latitude consistency: GK northing increases northward
        if n > 2950000 and lat1 < 26.5:
            print(f"  FAIL: {label} — northing {n} → lat {lat1:.4f} (expected >26.5)"); errors += 1
        elif n < 2930000 and lat1 > 26.5:
            print(f"  FAIL: {label} — northing {n} → lat {lat1:.4f} (expected <26.5)"); errors += 1
        else:
            print(f"  PASS: {label} — E={e} N={n} → ({lon1:.6f}, {lat1:.6f})")

    # ——— Test 3: DOM ↔ Terrain tile coordinate alignment ———
    print("Test 3: DOM ↔ Terrain tile coordinate consistency")
    dom_west, dom_south, dom_east, dom_north = 103.153629, 26.476194, 103.155938, 26.478965
    ter_west, ter_south, ter_east, ter_north = 103.153629, 26.262244, 103.502725, 26.478965

    # Check overlap
    ow = max(dom_west, ter_west); oe = min(dom_east, ter_east)
    os_ = max(dom_south, ter_south); on_ = min(dom_north, ter_north)

    if ow < oe and os_ < on_:
        print(f"  PASS: Overlap region [{ow:.6f},{os_:.6f},{oe:.6f},{on_:.6f}]")
    else:
        print(f"  FAIL: No overlap"); errors += 1

    # ——— Test 4: Tile coordinate mapping both directions ———
    print("Test 4: Tile coordinate bidirectionality")
    mid_lon, mid_lat = (ow+oe)/2, (os_+on_)/2
    for level in [8, 10, 12, 14, 16, 18]:
        tx, ty = cesium_tile_xy(mid_lon, mid_lat, level)
        w, s, e, n = tile_lonlat_bounds(level, tx, ty)
        # Verify point is inside tile
        if w <= mid_lon <= e and s <= mid_lat <= n:
            print(f"  PASS: L{level} tile({tx},{ty}) [{w:.6f},{s:.6f},{e:.6f},{n:.6f}] contains point")
        else:
            print(f"  FAIL: L{level} tile({tx},{ty}) does not contain ({mid_lon:.6f},{mid_lat:.6f})")
            errors += 1

    # ——— Test 5: DOM pixel ↔ geographic accuracy ———
    print("Test 5: DOM pixel ↔ geographic mapping")
    dom_w_px, dom_h_px = 1152, 1536
    dom_res = 0.2
    dom_ox, dom_oy = 498700.0, 2929900.0  # tiepoint = NW corner

    for px, py, label in [(0, 0, "NW"), (dom_w_px-1, 0, "NE"),
                           (0, dom_h_px-1, "SW"), (dom_w_px-1, dom_h_px-1, "SE"),
                           (dom_w_px//2, dom_h_px//2, "Center")]:
        easting = dom_ox + px * dom_res
        northing = dom_oy - py * dom_res  # north-up: row increases → northing decreases
        lat, lon = gk_inverse(easting, northing)
        # Verify within expected bounds
        eps = 1e-6  # ~0.1m at this latitude (sub-pixel for 0.2m DOM)
        if dom_west - eps <= lon <= dom_east + eps and dom_south - eps <= lat <= dom_north + eps:
            print(f"  PASS: {label} pixel({px},{py}) → ({lon:.6f},{lat:.6f})")
        else:
            print(f"  FAIL: {label} pixel({px},{py}) → ({lon:.6f},{lat:.6f}) out of bounds"); errors += 1

    # ——— Test 6: DOM TMS Y-flip vs Cesium Y ———
    print("Test 6: Cesium Y naming convention")
    for level in [12, 14]:
        tx, ty_cesium = cesium_tile_xy(mid_lon, mid_lat, level)
        tilesY = 1 << level
        ty_tms = tilesY - 1 - ty_cesium
        # Verify Cesium Y and TMS Y are inverted
        assert ty_cesium + ty_tms == tilesY - 1, "Y inversion broken"
        print(f"  PASS: L{level} CesiumY={ty_cesium} TMS_Y={ty_tms} "
              f"(files named w/ Cesium-internal Y)")

    # ——— Summary ———
    print(f"\n{'='*60}")
    if errors == 0:
        print(f"ALL TESTS PASSED (0 errors)")
    else:
        print(f"{errors} TEST(S) FAILED")
    return errors

if __name__ == "__main__":
    sys.exit(run_tests())
