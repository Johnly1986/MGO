#!/usr/bin/env python3
"""
追踪瓦片 E/N 方位错误：用已知顶点走完整链路，对比预期 ECEF。
"""
import math, numpy as np

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
        sp = math.sin(phi); Mprime = A*(1-E2)/(1-E2*sp*sp)**1.5
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

# 模拟一个已知顶点：在原点东 1000m, 北 2000m, 高 50m
# Assimp (East=1000, Up=50, North=2000)
test_vertex = (1000.0, 50.0, 2000.0)

print("=" * 70)
print(f"测试顶点: Assimp(East={test_vertex[0]}, Up={test_vertex[1]}, North={test_vertex[2]})")
print("=" * 70)

# 真值 ECEF (直接世界坐标 -> GK逆算 -> ECEF)
enu_e, enu_n, enu_u = test_vertex[0], test_vertex[2], test_vertex[1]  # Assimp->ENU
true_lat, true_lon = gk_inverse(ORIGIN_E + enu_e, ORIGIN_N + enu_n)
true_ecef = geo_to_ecef(true_lat, true_lon, enu_u + ORIGIN_Z)
print(f"\n真值 ECEF: ({true_ecef[0]:.3f}, {true_ecef[1]:.3f}, {true_ecef[2]:.3f})")

# 模拟 bbox: 假设模型 East[0,2000], Up[0,100], North[1000,3000]
bbox_min = (0.0, 0.0, 1000.0)   # (East, Up, North)
bbox_max = (2000.0, 100.0, 3000.0)

print(f"\nbbox (Assimp Y-up): East[{bbox_min[0]},{bbox_max[0]}] Up[{bbox_min[1]},{bbox_max[1]}] North[{bbox_min[2]},{bbox_max[2]}]")

# WriteBoxJson: BBoxAssimpToTilesZUp
# X=East, Y=-North, Z=Up
tile_min = (bbox_min[0], -bbox_max[2], bbox_min[1])
tile_max = (bbox_max[0], -bbox_min[2], bbox_max[1])
cx = (tile_min[0] + tile_max[0]) / 2
cy = (tile_min[1] + tile_max[1]) / 2
cz = (tile_min[2] + tile_max[2]) / 2
hx = (tile_max[0] - tile_min[0]) / 2
hy = (tile_max[1] - tile_min[1]) / 2
hz = (tile_max[2] - tile_min[2]) / 2

print(f"\nWriteBoxJson 输出 (tile-local Z-up):")
print(f"  center: ({cx:.1f}, {cy:.1f}, {cz:.1f})  <- cy = -North_center = {-(-1000+3000)/2:.1f}")
print(f"  half:   ({hx:.1f}, {hy:.1f}, {hz:.1f})")
print(f"  box: [{cx:.1f},{cy:.1f},{cz:.1f}, {hx:.1f},0,0, 0,{hy:.1f},0, 0,0,{hz:.1f}]")

# 测试 3 种 root transform 配置
print(f"\n{'='*70}")
print("3 种 root transform 配置对比")
print(f"{'='*70}")

for label, neg_north in [
    ("A: 纯 ENU->ECEF (无 North 取反)", False),
    ("B: North 列取反 (我的修复)", True),
]:
    root = np.eye(4)
    root[:3, 0] = R0[:, 0]   # East col
    root[:3, 1] = -R0[:, 1] if neg_north else R0[:, 1]  # North col (取反?)
    root[:3, 2] = R0[:, 2]   # Up col
    root[:3, 3] = T0

    # 1) bbox center 经 root transform -> ECEF
    bbox_ecef = root @ np.array([cx, cy, cz, 1.0])

    # 2) 顶点经 Y_UP_TO_Z_UP + root transform -> ECEF
    # Y_UP_TO_Z_UP: (x,y,z) -> (x,-z,y)
    yup_vertex = np.array([test_vertex[0], test_vertex[1], test_vertex[2]])
    zup_vertex = np.array([yup_vertex[0], -yup_vertex[2], yup_vertex[1]])
    vertex_ecef = root @ np.array([zup_vertex[0], zup_vertex[1], zup_vertex[2], 1.0])

    # 3) bbox 中心的真实 ECEF (直接计算)
    bbox_east_c = (bbox_min[0] + bbox_max[0]) / 2  # East center
    bbox_north_c = (bbox_min[2] + bbox_max[2]) / 2  # North center
    bbox_up_c = (bbox_min[1] + bbox_max[1]) / 2  # Up center
    bclat, bclon = gk_inverse(ORIGIN_E + bbox_east_c, ORIGIN_N + bbox_north_c)
    bbox_true_ecef = geo_to_ecef(bclat, bclon, bbox_up_c + ORIGIN_Z)

    # 4) 顶点的真实 ECEF (直接计算)
    vlat, vlon = gk_inverse(ORIGIN_E + test_vertex[0], ORIGIN_N + test_vertex[2])
    vertex_true_ecef = geo_to_ecef(vlat, vlon, test_vertex[1] + ORIGIN_Z)

    bbox_err = np.linalg.norm(bbox_true_ecef - bbox_ecef[:3])
    vertex_err = np.linalg.norm(vertex_true_ecef - vertex_ecef[:3])

    print(f"\n  --- {label} ---")
    print(f"  bbox center 真值 ECEF:  ({bbox_true_ecef[0]:.3f}, {bbox_true_ecef[1]:.3f}, {bbox_true_ecef[2]:.3f})")
    print(f"  bbox center 链路 ECEF:  ({bbox_ecef[0]:.3f}, {bbox_ecef[1]:.3f}, {bbox_ecef[2]:.3f})")
    print(f"  bbox 误差: {bbox_err*100:.1f} cm {'✓' if bbox_err < 0.01 else '✗'}")
    print(f"  vertex 真值 ECEF:       ({vertex_true_ecef[0]:.3f}, {vertex_true_ecef[1]:.3f}, {vertex_true_ecef[2]:.3f})")
    print(f"  vertex 链路 ECEF:       ({vertex_ecef[0]:.3f}, {vertex_ecef[1]:.3f}, {vertex_ecef[2]:.3f})")
    print(f"  vertex 误差: {vertex_err*100:.1f} cm {'✓' if vertex_err < 0.01 else '✗'}")

    # 分解 E/N 误差
    # ECEF 误差 -> ENU 误差
    err_ecef_bbox = bbox_true_ecef - bbox_ecef[:3]
    err_enu = R0.T @ err_ecef_bbox
    print(f"  bbox ENU 误差分解: East={err_enu[0]*100:.1f}cm, North={err_enu[1]*100:.1f}cm, Up={err_enu[2]*100:.1f}cm")
