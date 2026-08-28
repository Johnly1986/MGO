#!/usr/bin/env python3
"""
验证修复：CProjectionEngine::ComputeRootTransform 添加 North 列取反后，
程序链路输出与直接世界坐标->ECEF 转换一致。
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
        arc = m0*phi+m2*math.sin(2*phi)+m4*math.sin(4*phi)+m6*math.sin(6*phi)
        if abs(M-arc)<1e-12: break
        sp = math.sin(phi)
        Mprime = A*(1-E2)/(1-E2*sp*sp)**1.5
        phi += (M-arc)/Mprime
    sf,cf = math.sin(phi),math.cos(phi)
    tf=sf/cf; tf2=tf*tf; tf4=tf2*tf2
    ef2=EP2*cf*cf; ef4=ef2*ef2
    nf=A/math.sqrt(1-E2*sf*sf); rf=A*(1-E2)/(1-E2*sf*sf)**1.5
    D=x/nf; D2=D*D; D3=D2*D; D5=D3*D*D; D7=D5*D*D
    lat=phi-(tf/(2*rf))*x*D+(tf/(24*rf))*x*D3*(5+3*tf2+ef2-4*ef4-9*ef2*tf2)-(tf/(720*rf))*x*D5*(61+90*tf2+45*tf4+46*ef2-252*ef2*tf2-3*ef4+100*ef4*tf2-66*ef2*tf4-90*ef4*tf4)+(tf/(40320*rf))*x*D7*(1385+3633*tf2+4095*tf4+1575*tf2*tf2*tf2)
    lon=LAMBDA0*DEG2RAD + D/cf - D3/(6*cf)*(1+2*tf2+ef2) + D5/(120*cf)*(5+28*tf2+24*tf4+6*ef2+8*ef2*tf2) - D7/(5040*cf)*(61+662*tf2+1320*tf4+720*tf2*tf2*tf2)
    return lat,lon

def geo_to_ecef(lat,lon,h):
    sl,cl=math.sin(lat),math.cos(lat)
    N=A/math.sqrt(1-E2*sl*sl)
    return np.array([(N+h)*cl*math.cos(lon),(N+h)*cl*math.sin(lon),(N*(1-E2)+h)*sl])

def enu_rot(lat,lon):
    sl,cl=math.sin(lat),math.cos(lat); sL,cL=math.sin(lon),math.cos(lon)
    return np.array([[-sL,-sl*cL,cl*cL],[cL,-sl*sL,cl*sL],[0.0,cl,sl]])

lat0,lon0=gk_inverse(ORIGIN_E,ORIGIN_N)
T0=geo_to_ecef(lat0,lon0,ORIGIN_Z); R0=enu_rot(lat0,lon0)

def compute_delta(cx, cy, cz):
    enu_e, enu_n, enu_u = cx, cz, cy
    lat_i, lon_i = gk_inverse(ORIGIN_E+enu_e, ORIGIN_N+enu_n)
    true_ecef = geo_to_ecef(lat_i, lon_i, enu_u+ORIGIN_Z)
    approx_ecef = R0 @ np.array([enu_e, enu_n, enu_u]) + T0
    delta_ecef = true_ecef - approx_ecef
    Rt = R0.T
    dEast = Rt[0,0]*delta_ecef[0]+Rt[0,1]*delta_ecef[1]+Rt[0,2]*delta_ecef[2]
    dNorth = Rt[1,0]*delta_ecef[0]+Rt[1,1]*delta_ecef[1]+Rt[1,2]*delta_ecef[2]
    dUp = Rt[2,0]*delta_ecef[0]+Rt[2,1]*delta_ecef[1]+Rt[2,2]*delta_ecef[2]
    return dEast, dUp, dNorth  # Assimp order

def build_root_transform_with_north_neg(R, T):
    """复现修复后的 ComputeRootTransform (含 North 列取反)"""
    root = np.eye(4)
    root[:3, 0] = R[:, 0]  # East col
    root[:3, 1] = R[:, 1]  # North col
    root[:3, 2] = R[:, 2]  # Up col
    root[:3, 3] = T
    # North-negation (修复)
    root[0, 1] = -root[0, 1]
    root[1, 1] = -root[1, 1]
    root[2, 1] = -root[2, 1]
    return root

def build_root_transform_no_neg(R, T):
    """旧版本 (无 North 列取反) - 用于对比"""
    root = np.eye(4)
    root[:3, 0] = R[:, 0]
    root[:3, 1] = R[:, 1]
    root[:3, 2] = R[:, 2]
    root[:3, 3] = T
    return root

# 测试
print("="*70)
print("验证：North 列取反修复 vs 直接 ECEF 转换")
print("="*70)
print()
print("CesiumJS 行为 (来自官方源码 ModelSceneGraph.js):")
print("  ECEF = tileTransform * Y_UP_TO_Z_UP * gltfVertex")
print("  Y_UP_TO_Z_UP: (x,y,z) -> (x,-z,y)")
print("  Assimp(East,Up,North) -> Y_UP -> (East,-North,Up)")
print()

for label, root_builder in [
    ("旧版本 (无 North 取反)", build_root_transform_no_neg),
    ("修复后 (North 列取反)", build_root_transform_with_north_neg),
]:
    print(f"\n--- {label} ---")
    root_4x4 = root_builder(R0, T0)

    for dist_km in [1, 5, 10]:
        d = dist_km * 1000 / math.sqrt(2)
        centroid = np.array([d, 0.0, d])  # Assimp

        # Delta
        dx, dy, dz = compute_delta(*centroid)

        # 测试顶点: 模型局部偏移
        for offset_name, offset in [
            ("中心", np.array([0,0,0])),
            ("东500m", np.array([500,0,0])),
            ("北500m", np.array([0,0,500])),
            ("东北500m", np.array([500,0,500])),
        ]:
            world = offset + centroid + np.array([dx, dy, dz])
            wx, wy, wz = world[0], world[1], world[2]

            # 程序链路
            # glTF: (wx, wy, wz) - Assimp Y-up, 不取反
            # Y_UP_TO_Z_UP: (wx, -wz, wy)
            # root transform
            chain = root_4x4 @ np.array([wx, -wz, wy, 1.0])

            # 直接转换 (真值)
            ve, vn, vu = wx, wz, wy  # Assimp->ENU
            vlat, vlon = gk_inverse(ORIGIN_E+ve, ORIGIN_N+vn)
            true_ecef = geo_to_ecef(vlat, vlon, vu+ORIGIN_Z)

            err = np.linalg.norm(true_ecef - chain[:3])

            if offset_name == "中心":
                print(f"  dist={dist_km:2d}km {offset_name}: err = {err*100:.2f} cm")
            else:
                print(f"         {offset_name}: err = {err*100:.2f} cm")
