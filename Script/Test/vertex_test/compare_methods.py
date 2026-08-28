#!/usr/bin/env python3
"""
验证：质心 vs 原点 vs 逐顶点 修正方法的精度对比。
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

# 构建含 North 列取反的 root transform
root = np.eye(4)
root[:3, 0] = R0[:, 0]
root[:3, 1] = -R0[:, 1]  # North 列取反
root[:3, 2] = R0[:, 2]
root[:3, 3] = T0

def compute_delta_at_point(wx, wy, wz):
    """计算 Assimp(wx,wy,wz) 处的 delta (ENU->ECEF 误差)"""
    enu_e, enu_n, enu_u = wx, wz, wy
    lat_i, lon_i = gk_inverse(ORIGIN_E+enu_e, ORIGIN_N+enu_n)
    true_ecef = geo_to_ecef(lat_i, lon_i, enu_u+ORIGIN_Z)
    approx_ecef = R0 @ np.array([enu_e, enu_n, enu_u]) + T0
    delta_ecef = true_ecef - approx_ecef
    Rt = R0.T
    dE = Rt[0,0]*delta_ecef[0]+Rt[0,1]*delta_ecef[1]+Rt[0,2]*delta_ecef[2]
    dN = Rt[1,0]*delta_ecef[0]+Rt[1,1]*delta_ecef[1]+Rt[1,2]*delta_ecef[2]
    dU = Rt[2,0]*delta_ecef[0]+Rt[2,1]*delta_ecef[1]+Rt[2,2]*delta_ecef[2]
    return dE, dU, dN  # Assimp order

def render_ecef(wx, wy, wz):
    """程序链路：Assimp(wx,wy,wz) -> Y_UP_TO_Z_UP -> root transform -> ECEF"""
    # Y_UP_TO_Z_UP: (x,y,z) -> (x,-z,y)
    tz = np.array([wx, -wz, wy, 1.0])
    return (root @ tz)[:3]

def true_ecef_at_world(wx, wy, wz):
    """直接转换：world -> GK逆算 -> ECEF"""
    enu_e, enu_n, enu_u = wx, wz, wy
    lat, lon = gk_inverse(ORIGIN_E+enu_e, ORIGIN_N+enu_n)
    return geo_to_ecef(lat, lon, enu_u+ORIGIN_Z)

# 模拟一个非对称模型（原点不在质心）
# 模型局部坐标：顶点在 [0, 1000] 范围，原点在 (0,0,0)
# 质心 = (500, 0, 500), bbox 中心 = (500, 0, 500)
# 但如果顶点分布不均匀，质心 ≠ bbox 中心
# 模拟：大部分顶点在 +X+Z 方向，少量在原点附近

print("=" * 80)
print("精度对比：3 种修正方法")
print("=" * 80)
print()
print("模型场景：")
print("  实例原点 (worldTransform 平移): (10000, 0, 10000)  -- 距投影原点 ~14km")
print("  模型局部顶点：非均匀分布（模拟道路模型）")
print()

# 测试场景：实例在 (10000, 0, 10000)，模型局部顶点偏移
instance_origin = np.array([10000.0, 0.0, 10000.0])  # Assimp

# 模型局部顶点（模拟道路：沿 X 方向延伸，大部分在远端）
local_vertices = []
for i in range(10):
    t = i / 9.0
    local_vertices.append(np.array([t * 1000, 0, t * 500]))  # 0..1000m X, 0..500m Z
# 加几个在原点附近的
local_vertices.append(np.array([10, 0, 5]))
local_vertices.append(np.array([20, 0, 10]))

local_vertices = np.array(local_vertices)

# 世界空间顶点（实例原点 + 局部偏移，假设旋转=I）
world_vertices = local_vertices + instance_origin

# 顶点质心（平均值）
vertex_centroid = world_vertices.mean(axis=0)
# bbox 中心
bbox_min = world_vertices.min(axis=0)
bbox_max = world_vertices.max(axis=0)
bbox_center = (bbox_min + bbox_max) / 2

print(f"  实例原点 (worldTransform): {instance_origin}")
print(f"  bbox 中心:                {bbox_center}")
print(f"  顶点质心 (平均):          {vertex_centroid}")
print(f"  质心 vs 原点距离:         {np.linalg.norm(vertex_centroid - instance_origin):.1f}m")
print()

# 三种修正方法
methods = [
    ("方法 A: 原点处计算 delta", instance_origin),
    ("方法 B: bbox 中心计算 delta", bbox_center),
    ("方法 C: 顶点质心计算 delta", vertex_centroid),
]

for method_name, ref_point in methods:
    print(f"\n--- {method_name} ---")
    print(f"  参考点: {ref_point}")

    # 在参考点计算 delta
    dx, dy, dz = compute_delta_at_point(*ref_point)
    print(f"  Delta: ({dx:.4f}, {dy:.4f}, {dz:.4f}), |delta|={np.linalg.norm([dx,dy,dz]):.4f}m")

    # 应用 delta 到所有顶点（均匀平移）
    errors = []
    for v_world in world_vertices:
        corrected = v_world + np.array([dx, dy, dz])
        rendered = render_ecef(*corrected)
        # 真值是 ORIGINAL 顶点位置的 ECEF（不是 corrected 的）
        true_ecef = true_ecef_at_world(*v_world)
        err = np.linalg.norm(true_ecef - rendered)
        errors.append(err)

    errors = np.array(errors)
    print(f"  顶点误差统计 ({len(errors)} 顶点):")
    print(f"    最大误差: {errors.max()*100:.2f} cm")
    print(f"    平均误差: {errors.mean()*100:.2f} cm")
    print(f"    RMSE:     {math.sqrt((errors**2).mean())*100:.2f} cm")

# 方法 D: 逐顶点修正
print(f"\n--- 方法 D: 逐顶点修正 (理想) ---")
errors = []
for v_world in world_vertices:
    # 每个顶点单独计算 delta
    dx, dy, dz = compute_delta_at_point(*v_world)
    corrected = v_world + np.array([dx, dy, dz])
    rendered = render_ecef(*corrected)
    # 真值是 ORIGINAL 顶点位置的 ECEF（不是 corrected 的）
    true_ecef = true_ecef_at_world(*v_world)
    err = np.linalg.norm(true_ecef - rendered)
    errors.append(err)

errors = np.array(errors)
print(f"  顶点误差统计 ({len(errors)} 顶点):")
print(f"    最大误差: {errors.max()*100:.4f} cm")
print(f"    平均误差: {errors.mean()*100:.4f} cm")
print(f"    RMSE:     {math.sqrt((errors**2).mean())*100:.4f} cm")
