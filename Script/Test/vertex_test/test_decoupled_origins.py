#!/usr/bin/env python3
"""
测试不同原点位置的网格实例修正结果是否正确。

验证方法：
  对同一组顶点，使用不同的投影原点 (originE, originN, originZ) 计算 delta，
  验证每个 (origin, vertex) 对的修正结果都正确（误差为 0）。

这验证了 ComputeProjectionError 解耦后的正确性：
  delta 只依赖 (origin, point)，与引擎内部状态无关。
"""
import math
import numpy as np

A = 6378137.0; F_INV = 298.257222101; DEG2RAD = math.pi/180.0
F = 1.0/F_INV; E2 = 2*F - F*F; EP2 = E2/(1-E2)
LAMBDA0 = 103.1666666666666667

def gk_inverse(E, N, lambda0_deg=LAMBDA0):
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
    lon=lambda0_deg*DEG2RAD + D/cf - D3/(6*cf)*(1+2*tf2+ef2) + D5/(120*cf)*(5+28*tf2+24*tf4+6*ef2+8*ef2*tf2) - D7/(5040*cf)*(61+662*tf2+1320*tf4+720*tf2*tf2*tf2)
    return lat,lon

def geo_to_ecef(lat,lon,h):
    sl,cl=math.sin(lat),math.cos(lat)
    N=A/math.sqrt(1-E2*sl*sl)
    return np.array([(N+h)*cl*math.cos(lon),(N+h)*cl*math.sin(lon),(N*(1-E2)+h)*sl])

def enu_rot(lat,lon):
    sl,cl=math.sin(lat),math.cos(lat); sL,cL=math.sin(lon),math.cos(lon)
    return np.array([[-sL,-sl*cL,cl*cL],[cL,-sl*sL,cl*sL],[0.0,cl,sl]])

# ============================================================================
# 解耦的 ComputeProjectionError (Python 复现 C++ 逻辑)
# ============================================================================
def compute_projection_error(originE, originN, originZ, x, y, z):
    """
    C++ ComputeProjectionError 的精确复现。
    输入: (originE, originN, originZ) 投影坐标, (x,y,z) Assimp (East,Up,North)
    输出: (dx, dy, dz) Assimp delta, 返回 delta 量级
    """
    # Assimp -> ENU
    enuE, enuN, enuU = x, z, y

    # Origin -> geographic -> ECEF
    lat0, lon0 = gk_inverse(originE, originN)
    T0 = geo_to_ecef(lat0, lon0, originZ)
    R0 = enu_rot(lat0, lon0)

    # True ECEF at point
    lat_i, lon_i = gk_inverse(originE + enuE, originN + enuN)
    true_ecef = geo_to_ecef(lat_i, lon_i, enuU + originZ)

    # Approximate ECEF (tangent plane at origin)
    approx_ecef = R0 @ np.array([enuE, enuN, enuU]) + T0

    # Delta in ECEF -> ENU via R0^T
    delta_ecef = true_ecef - approx_ecef
    delta_enu = R0.T @ delta_ecef

    # ENU -> Assimp: (East, North, Up) -> (East, Up, North)
    dx = delta_enu[0]  # East
    dy = delta_enu[2]   # Up
    dz = delta_enu[1]   # North

    mag = math.sqrt(dx*dx + dy*dy + dz*dz)
    return dx, dy, dz, mag

# ============================================================================
# 渲染链 (含 North 列取反的 root transform)
# ============================================================================
def build_root_transform(originE, originN, originZ):
    """构建含 North 列取反的 root transform (复现 C++ ComputeRootTransform)"""
    lat0, lon0 = gk_inverse(originE, originN)
    T0 = geo_to_ecef(lat0, lon0, originZ)
    R0 = enu_rot(lat0, lon0)

    root = np.eye(4)
    root[:3, 0] = R0[:, 0]   # East col
    root[:3, 1] = -R0[:, 1]  # North col (negated for CesiumJS Y_UP_TO_Z_UP)
    root[:3, 2] = R0[:, 2]   # Up col
    root[:3, 3] = T0
    return root

def render_ecef(root, wx, wy, wz):
    """CesiumJS 渲染链: Y_UP_TO_Z_UP -> root transform"""
    # Y_UP_TO_Z_UP: (x,y,z) -> (x,-z,y)
    tz = np.array([wx, -wz, wy, 1.0])
    return (root @ tz)[:3]

def true_ecef_at_world(originE, originN, originZ, wx, wy, wz):
    """直接转换: world -> GK逆算 -> ECEF (真值)"""
    enu_e, enu_n, enu_u = wx, wz, wy  # Assimp->ENU
    lat, lon = gk_inverse(originE + enu_e, originN + enu_n)
    return geo_to_ecef(lat, lon, enu_u + originZ)

# ============================================================================
# 测试 1: 不同原点位置，同一组顶点
# ============================================================================
print("=" * 80)
print("测试 1: 不同原点位置 + 逐顶点修正 -> 每个顶点误差应为 0")
print("=" * 80)

# 测试顶点: 模拟一个 1km x 1km 模型在局部空间的不同位置
test_cases = [
    # (name, originE, originN, originZ, vertex_offsets)
    ("原点 (500000, 3000000)", 500000.0, 3000000.0, 0.0,
     [(-500, 0, -500), (500, 0, -500), (-500, 0, 500), (500, 0, 500), (0, 0, 0)]),

    ("偏移 5km (505000, 3005000)", 505000.0, 3005000.0, 0.0,
     [(-500, 0, -500), (500, 0, -500), (-500, 0, 500), (500, 0, 500), (0, 0, 0)]),

    ("偏移 20km (520000, 3020000)", 520000.0, 3020000.0, 100.0,
     [(-500, 0, -500), (500, 0, -500), (-500, 0, 500), (500, 0, 500), (0, 0, 0)]),

    ("高纬度原点 (500000, 4500000)", 500000.0, 4500000.0, 0.0,
     [(-500, 0, -500), (500, 0, -500), (-500, 0, 500), (500, 0, 500), (0, 0, 0)]),

    ("极端偏移 (500000, 1000000)", 500000.0, 1000000.0, 500.0,
     [(-500, 0, -500), (500, 0, -500), (-500, 0, 500), (500, 0, 500), (0, 0, 0)]),
]

all_pass = True
for name, oE, oN, oZ, offsets in test_cases:
    root = build_root_transform(oE, oN, oZ)

    max_err = 0.0
    for offset in offsets:
        # 顶点世界位置 = 原点偏移 + 局部偏移 (在 Assimp 空间)
        # 这里 offset 就是相对于原点的 Assimp 坐标 (East, Up, North)
        wx, wy, wz = offset

        # 逐顶点修正: 在顶点位置计算 delta
        dx, dy, dz, mag = compute_projection_error(oE, oN, oZ, wx, wy, wz)

        # 修正后的世界位置
        corrected = np.array([wx + dx, wy + dy, wz + dz])

        # 渲染链输出
        rendered = render_ecef(root, *corrected)

        # 真值 (原始顶点位置的 ECEF)
        true_ecef = true_ecef_at_world(oE, oN, oZ, wx, wy, wz)

        err = np.linalg.norm(true_ecef - rendered)
        max_err = max(max_err, err)

    status = "PASS" if max_err < 1e-6 else "FAIL"
    if max_err >= 1e-6:
        all_pass = False
    print(f"  [{status}] {name}: max_err = {max_err*1e6:.2f} μm")

# ============================================================================
# 测试 2: 同一顶点，不同原点 -> delta 不同但修正后都应正确
# ============================================================================
print(f"\n{'=' * 80}")
print("测试 2: 同一顶点 (1000, 0, 1000) 在不同原点下的 delta 和修正结果")
print("=" * 80)

vertex = (1000.0, 0.0, 1000.0)  # Assimp (East=1km, Up=0, North=1km)

origins = [
    (500000.0, 3000000.0, 0.0),
    (505000.0, 3005000.0, 0.0),
    (510000.0, 3010000.0, 0.0),
    (520000.0, 3020000.0, 100.0),
    (500000.0, 4500000.0, 0.0),   # 不同纬度
    (500000.0, 1000000.0, 0.0),   # 低纬度
]

print(f"\n{'Origin (E,N,H)':>30} {'|delta| (m)':>12} {'修正后误差 (μm)':>16}")
print("-" * 62)

for oE, oN, oZ in origins:
    dx, dy, dz, mag = compute_projection_error(oE, oN, oZ, *vertex)
    root = build_root_transform(oE, oN, oZ)

    corrected = np.array([vertex[0]+dx, vertex[1]+dy, vertex[2]+dz])
    rendered = render_ecef(root, *corrected)
    true_ecef = true_ecef_at_world(oE, oN, oZ, *vertex)

    err = np.linalg.norm(true_ecef - rendered)
    status = "PASS" if err < 1e-6 else "FAIL"
    print(f"  ({oE:.0f}, {oN:.0f}, {oZ:.0f})    {mag:10.4f}    {err*1e6:10.2f}  [{status}]")

    if err >= 1e-6:
        all_pass = False

# ============================================================================
# 测试 3: 解耦验证 - 同一引擎实例，不同原点调用
# ============================================================================
print(f"\n{'=' * 80}")
print("测试 3: 解耦验证 - 同一引擎，不同原点调用 ComputeProjectionError")
print("=" * 80)
print("  (验证方法不依赖引擎 m_originX/Y/Z 状态)")

# 模拟: 引擎加载了原点 A，但用原点 B 调用解耦方法
origin_A = (500000.0, 3000000.0, 0.0)  # 引擎加载的原点
origin_B = (520000.0, 3020000.0, 100.0)  # 解耦调用用的原点

# 用引擎原点 (A) 计算
dxA, dyA, dzA, magA = compute_projection_error(*origin_A, *vertex)
# 用不同原点 (B) 计算 (解耦调用)
dxB, dyB, dzB, magB = compute_projection_error(*origin_B, *vertex)

# 分别验证
root_A = build_root_transform(*origin_A)
root_B = build_root_transform(*origin_B)

corrected_A = np.array([vertex[0]+dxA, vertex[1]+dyA, vertex[2]+dzA])
corrected_B = np.array([vertex[0]+dxB, vertex[1]+dyB, vertex[2]+dzB])

rendered_A = render_ecef(root_A, *corrected_A)
rendered_B = render_ecef(root_B, *corrected_B)

true_A = true_ecef_at_world(*origin_A, *vertex)
true_B = true_ecef_at_world(*origin_B, *vertex)

err_A = np.linalg.norm(true_A - rendered_A)
err_B = np.linalg.norm(true_B - rendered_B)

print(f"  引擎原点 A = {origin_A}")
print(f"  解耦原点 B = {origin_B}")
print(f"  测试顶点   = {vertex}")
print()
print(f"  用 A 计算: |delta|={magA:.4f}m, 修正后误差={err_A*1e6:.2f}μm")
print(f"  用 B 计算: |delta|={magB:.4f}m, 修正后误差={err_B*1e6:.2f}μm")
print()
status = "PASS" if err_A < 1e-6 and err_B < 1e-6 else "FAIL"
if err_A >= 1e-6 or err_B >= 1e-6:
    all_pass = False
print(f"  结论: [{status}] 解耦方法在不同原点下都正确")

# ============================================================================
# 总结
# ============================================================================
print(f"\n{'=' * 80}")
print("总结")
print("=" * 80)
print(f"  {'全部通过' if all_pass else '存在失败'}")
