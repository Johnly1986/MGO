#!/usr/bin/env python3
"""
精确量化 R_corr 旋转修正的效果。
分离曲率残差和旋转误差的贡献。
"""
import math
import numpy as np

A = 6378137.0; F_INV = 298.257222101; DEG2RAD = math.pi/180.0
F = 1.0/F_INV; E2 = 2*F - F*F; EP2 = E2/(1-E2)
ORIGIN_E = 498700.0; ORIGIN_N = 2929900.0; ORIGIN_Z = 0.0
LAMBDA0 = 103.1666666666666667

def gk_inverse(E, N):
    x, y = E-500000.0, N; M = y
    e4 = E2*E2; e6 = e4*E2
    m0 = A*(1 - E2/4 - 3*e4/64 - 5*e6/256)
    phi = M/m0
    for _ in range(6):
        m2 = A*(-3*E2/8 - 3*e4/32 - 45*e6/1024)
        m4 = A*(15*e4/256 + 45*e6/1024)
        m6 = A*(-35*e6/3072)
        arc = m0*phi + m2*math.sin(2*phi) + m4*math.sin(4*phi) + m6*math.sin(6*phi)
        if abs(M-arc) < 1e-12: break
        sp = math.sin(phi)
        Mprime = A*(1-E2)/(1 - E2*sp*sp)**1.5
        phi += (M-arc)/Mprime
    sf, cf = math.sin(phi), math.cos(phi)
    tf = sf/cf; tf2 = tf*tf; tf4 = tf2*tf2
    ef2 = EP2*cf*cf; ef4 = ef2*ef2
    nf = A/math.sqrt(1 - E2*sf*sf)
    rf = A*(1-E2)/(1 - E2*sf*sf)**1.5
    D = x/nf
    D2 = D*D; D3 = D2*D; D4 = D3*D; D5 = D4*D; D7 = D5*D*D
    lat = phi - (tf/(2*rf))*x*D + (tf/(24*rf))*x*D3*(5+3*tf2+ef2-4*ef4-9*ef2*tf2) \
          - (tf/(720*rf))*x*D5*(61+90*tf2+45*tf4+46*ef2-252*ef2*tf2-3*ef4+100*ef4*tf2-66*ef2*tf4-90*ef4*tf4) \
          + (tf/(40320*rf))*x*D7*(1385+3633*tf2+4095*tf4+1575*tf2*tf2*tf2)
    lon = LAMBDA0*DEG2RAD + D/cf - D3/(6*cf)*(1+2*tf2+ef2) \
          + D5/(120*cf)*(5+28*tf2+24*tf4+6*ef2+8*ef2*tf2) \
          - D7/(5040*cf)*(61+662*tf2+1320*tf4+720*tf2*tf2*tf2)
    return lat, lon

def geo_to_ecef(lat, lon, h):
    sl, cl = math.sin(lat), math.cos(lat)
    N = A/math.sqrt(1 - E2*sl*sl)
    return np.array([(N+h)*cl*math.cos(lon), (N+h)*cl*math.sin(lon), (N*(1-E2)+h)*sl])

def enu_rot(lat, lon):
    sl, cl = math.sin(lat), math.cos(lat)
    sL, cL = math.sin(lon), math.cos(lon)
    return np.array([[-sL, -sl*cL, cl*cL], [cL, -sl*sL, cl*sL], [0.0, cl, sl]])

lat0, lon0 = gk_inverse(ORIGIN_E, ORIGIN_N)
T0 = geo_to_ecef(lat0, lon0, ORIGIN_Z)
R0 = enu_rot(lat0, lon0)

print("=" * 70)
print("R_corr 旋转修正效果 — 精确量化")
print("=" * 70)

for dist_km in [1, 3, 5, 10, 20]:
    d = dist_km * 1000 / math.sqrt(2.0)
    center_assimp = np.array([d, 0.0, d])

    # Delta at center
    clat, clon = gk_inverse(ORIGIN_E + d, ORIGIN_N + d)
    ctrue = geo_to_ecef(clat, clon, ORIGIN_Z)
    capprox = R0 @ np.array([d, d, 0.0]) + T0
    cd = ctrue - capprox
    cdE = R0[0,0]*cd[0] + R0[1,0]*cd[1] + R0[2,0]*cd[2]
    cdN = R0[0,1]*cd[0] + R0[1,1]*cd[1] + R0[2,1]*cd[2]
    cdU = R0[0,2]*cd[0] + R0[1,2]*cd[1] + R0[2,2]*cd[2]
    dx, dy, dz = cdE, cdU, cdN

    # R_corr
    R_inst = enu_rot(clat, clon)
    R_corr = R0.T @ R_inst

    # 对于模型上的 4 个角点 (500m 半宽)，分别计算：
    #   1) 仅平移修正 → 顶点 ECEF vs 真值 ECEF
    #   2) 平移+旋转修正 → 顶点 ECEF vs 真值 ECEF
    #   3) 误差差 = (1) − (2) = R_corr 的净贡献

    offsets = [
        np.array([500, 0, 500]),    # NE
        np.array([500, 0, -500]),   # SE
        np.array([-500, 0, 500]),   # NW
        np.array([-500, 0, -500]),  # SW
    ]

    errs_no_corr = []
    errs_with_corr = []

    for off in offsets:
        # 仅平移
        world_no = off + center_assimp + np.array([dx, dy, dz])
        ve, vn, vu = world_no[0], world_no[2], world_no[1]
        vlat, vlon = gk_inverse(ORIGIN_E + ve, ORIGIN_N + vn)
        true_no = geo_to_ecef(vlat, vlon, vu + ORIGIN_Z)
        approx_no = R0 @ np.array([ve, vn, vu]) + T0
        err_no = np.linalg.norm(true_no - approx_no)

        # 平移+旋转
        world_with = R_corr @ off + center_assimp + np.array([dx, dy, dz])
        ve_w, vn_w, vu_w = world_with[0], world_with[2], world_with[1]
        vlat_w, vlon_w = gk_inverse(ORIGIN_E + ve_w, ORIGIN_N + vn_w)
        true_with = geo_to_ecef(vlat_w, vlon_w, vu_w + ORIGIN_Z)
        approx_with = R0 @ np.array([ve_w, vn_w, vu_w]) + T0
        err_with = np.linalg.norm(true_with - approx_with)

        errs_no_corr.append(err_no)
        errs_with_corr.append(err_with)

    avg_no = np.mean(errs_no_corr)
    avg_with = np.mean(errs_with_corr)
    improvement = avg_no - avg_with

    # 理论预测: 旋转误差 ≈ model_half * sin(rot_angle)
    rot_angle = np.linalg.norm(R_corr - np.eye(3)) / math.sqrt(2)
    theory = 707.0 * rot_angle  # 707m = sqrt(500² + 500²)
    theory_alt = 500.0 * rot_angle * math.sqrt(2)

    print(f"\n  dist={dist_km:2d}km | 平均残留: 仅平移={avg_no*100:.1f}cm "
          f"平移+旋转={avg_with*100:.1f}cm | 改善={improvement*100:.1f}cm "
          f"(理论={theory*100:.1f}cm)")
