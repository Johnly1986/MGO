#!/usr/bin/env python3
"""Debug: trace the center point through the full pipeline."""
import math
import numpy as np

A = 6378137.0
F_INV = 298.257222101
DEG2RAD = math.pi / 180.0

ORIGIN_E = 498700.0
ORIGIN_N = 2929900.0
ORIGIN_Z = 0.0
FE = 500000.0
LAMBDA0_DEG = 103.1666666666666667

def gk_inverse(E, N):
    f = 1.0 / F_INV
    e2 = 2*f - f*f
    ep2 = e2 / (1-e2)
    l0 = LAMBDA0_DEG * DEG2RAD
    x, y = E - FE, N
    M = y
    e4 = e2*e2
    e6 = e4*e2
    m0 = A*(1 - e2/4 - 3*e4/64 - 5*e6/256)
    phi = M / m0
    m2 = A*(-3*e2/8 - 3*e4/32 - 45*e6/1024)
    m4 = A*(15*e4/256 + 45*e6/1024)
    m6 = A*(-35*e6/3072)
    for _ in range(6):
        arc = m0*phi + m2*math.sin(2*phi) + m4*math.sin(4*phi) + m6*math.sin(6*phi)
        if abs(M - arc) < 1e-12: break
        sp = math.sin(phi)
        Mprime = A*(1-e2) / (1 - e2*sp*sp)**1.5
        phi += (M - arc) / Mprime
    sf, cf = math.sin(phi), math.cos(phi)
    tf = sf/cf
    tf2 = tf*tf
    tf4 = tf2*tf2
    ef2 = ep2*cf*cf
    ef4 = ef2*ef2
    nf = A / math.sqrt(1 - e2*sf*sf)
    rf = A*(1-e2) / (1 - e2*sf*sf)**1.5
    D = x / nf
    D2 = D*D
    D3 = D2*D
    D4 = D3*D
    D5 = D4*D
    D6 = D5*D
    D7 = D6*D
    lat = phi - (tf/(2*rf))*x*D + (tf/(24*rf))*x*D3*(5+3*tf2+ef2-4*ef4-9*ef2*tf2) - (tf/(720*rf))*x*D5*(61+90*tf2+45*tf4+46*ef2-252*ef2*tf2-3*ef4+100*ef4*tf2-66*ef2*tf4-90*ef4*tf4)
    lon = l0 + D/cf - D3/(6*cf)*(1+2*tf2+ef2) + D5/(120*cf)*(5+28*tf2+24*tf4+6*ef2+8*ef2*tf2) - D7/(5040*cf)*(61+662*tf2+1320*tf4+720*tf2*tf2*tf2)
    return lat, lon

def geo_to_ecef(lat, lon, h):
    f = 1.0/F_INV; e2 = 2*f - f*f
    sl, cl = math.sin(lat), math.cos(lat)
    N = A / math.sqrt(1 - e2*sl*sl)
    return np.array([(N+h)*cl*math.cos(lon), (N+h)*cl*math.sin(lon), (N*(1-e2)+h)*sl])

def enu_rot(lat, lon):
    sl, cl = math.sin(lat), math.cos(lat)
    sL, cL = math.sin(lon), math.cos(lon)
    R = np.array([[-sL, -sl*cL, cl*cL],
                  [ cL, -sl*sL, cl*sL],
                  [0.0,    cl,    sl]])
    return R

# Setup
lat0, lon0 = gk_inverse(ORIGIN_E, ORIGIN_N)
T0 = geo_to_ecef(lat0, lon0, ORIGIN_Z)
R = enu_rot(lat0, lon0)

print(f"Origin: GK({ORIGIN_E},{ORIGIN_N}), lat={math.degrees(lat0):.8f}, lon={math.degrees(lon0):.8f}")
print(f"T0: {T0}")
print(f"R orthonormality check: R@R.T = \n{R @ R.T}")

# Center point
cx, cy, cz = 500.0, 0.0, 500.0
enu_e, enu_n, enu_u = cx, cz, cy  # AssimpToENU: (East,North,Up) = (cx,cz,cy)
print(f"\nCenter Assimp({cx},{cy},{cz}) → ENU({enu_e},{enu_n},{enu_u})")

# True ECEF
clat, clon = gk_inverse(ORIGIN_E + enu_e, ORIGIN_N + enu_n)
ctrue = geo_to_ecef(clat, clon, enu_u + ORIGIN_Z)
print(f"True ECEF: {ctrue}")

# Approximate
c_approx = R @ np.array([enu_e, enu_n, enu_u]) + T0
print(f"Approx ECEF: {c_approx}")

# Delta
c_delta = ctrue - c_approx
print(f"Delta ECEF: {c_delta}, mag={np.linalg.norm(c_delta):.6f}")

# Correct ENU back-proj (R^T * delta)
cdE = R[0,0]*c_delta[0] + R[1,0]*c_delta[1] + R[2,0]*c_delta[2]
cdN = R[0,1]*c_delta[0] + R[1,1]*c_delta[1] + R[2,1]*c_delta[2]
cdU = R[0,2]*c_delta[0] + R[1,2]*c_delta[1] + R[2,2]*c_delta[2]
print(f"Delta ENU: dE={cdE:.6f}, dN={cdN:.6f}, dU={cdU:.6f}")

# ENU→Assimp
dx, dy, dz = cdE, cdU, cdN  # EnuToAssimp(E,N,U) = (E,U,N)
print(f"Delta Assimp: dx={dx:.6f}, dy={dy:.6f}, dz={dz:.6f}")

# Verify orthonormality: R * [cdE,cdN,cdU] should equal c_delta
R_delta = R @ np.array([cdE, cdN, cdU])
print(f"R * delta_ENU: {R_delta}")
print(f"c_delta:       {c_delta}")
print(f"Diff: {np.linalg.norm(R_delta - c_delta):.10f}")

# Render the center point
wx, wy, wz = cx + dx, cy + dy, cz + dz
print(f"\nWorld (after delta): ({wx:.6f}, {wy:.6f}, {wz:.6f})")

# glTF: wz = -wz
gx, gy, gz = wx, wy, -wz
print(f"glTF: ({gx:.6f}, {gy:.6f}, {gz:.6f})")

# Y_UP_TO_Z_UP: (gx, -gz, gy)
tz = np.array([gx, -gz, gy, 1.0])
print(f"Cesium Z-up: ({tz[0]:.6f}, {tz[1]:.6f}, {tz[2]:.6f})")

# Root transform
root = np.eye(4)
root[:3, 0] = R[:, 0]
root[:3, 1] = R[:, 1]
root[:3, 2] = R[:, 2]
root[:3, 3] = T0

rendered = root @ tz
print(f"Rendered ECEF:  ({rendered[0]:.3f}, {rendered[1]:.3f}, {rendered[2]:.3f})")

# True ECEF at center+delta
ve, vn, vu = wx, wz, wy  # AssimpToENU of (wx,wy,wz) = (wx,wz,wy)... wait
# Actually: Assimp(E,U,N) → ENU(E,N,U): e=wx, n=wz, u=wy
ve_c, vn_c, vu_c = wx, wz, wy
print(f"\nTrue ECEF input: ENU({ve_c:.6f},{vn_c:.6f},{vu_c:.6f})")
vlat, vlon = gk_inverse(ORIGIN_E + ve_c, ORIGIN_N + vn_c)
vtrue = geo_to_ecef(vlat, vlon, vu_c + ORIGIN_Z)
print(f"True ECEF (recomputed):  ({vtrue[0]:.3f}, {vtrue[1]:.3f}, {vtrue[2]:.3f})")

# Alternative: use the delta reference point
vtrue2 = c_approx + c_delta  # should equal ctrue
print(f"True ECEF (ctrue):       ({ctrue[0]:.3f}, {ctrue[1]:.3f}, {ctrue[2]:.3f})")

err1 = np.linalg.norm(vtrue - rendered[:3])
err2 = np.linalg.norm(ctrue - rendered[:3])
print(f"\nError vs recomputed true: {err1*100:.2f}cm")
print(f"Error vs delta-ref ctrue: {err2*100:.2f}cm")
