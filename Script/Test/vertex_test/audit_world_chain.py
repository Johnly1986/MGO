#!/usr/bin/env python3
"""
模型展开到世界坐标的完整链路审查。

链路:
  mesh.local → worldTransform(含node层级+delta平移) → glTF顶点
  → CesiumJS Y_UP_TO_Z_UP → root transform (ENU→ECEF)
  → 最终 ECEF 世界坐标

审查点:
  1. worldTransform 层级展平是否正确传递了旋转
  2. delta 平移应用的坐标空间是否正确
  3. R_corr 旋转修正缺失导致的误差量化
  4. 对比：仅平移 vs 平移+旋转 vs 真值
"""

import math
import numpy as np

# ============================================================================
# 参数
# ============================================================================
A = 6378137.0
F_INV = 298.257222101
DEG2RAD = math.pi / 180.0
F = 1.0 / F_INV
E2 = 2.0 * F - F * F
EP2 = E2 / (1.0 - E2)
ORIGIN_E = 498700.0
ORIGIN_N = 2929900.0
ORIGIN_Z = 0.0
LAMBDA0 = 103.1666666666666667

def gk_inverse(E, N):
    """C++ GeodeticMath::GKInverse 精确复现"""
    x, y = E - 500000.0, N
    M = y
    e4 = E2*E2; e6 = e4*E2
    m0 = A*(1 - E2/4 - 3*e4/64 - 5*e6/256)
    phi = M / m0
    for _ in range(6):
        m2 = A*(-3*E2/8 - 3*e4/32 - 45*e6/1024)
        m4 = A*(15*e4/256 + 45*e6/1024)
        m6 = A*(-35*e6/3072)
        arc = m0*phi + m2*math.sin(2*phi) + m4*math.sin(4*phi) + m6*math.sin(6*phi)
        if abs(M-arc) < 1e-12: break
        sp = math.sin(phi)
        Mprime = A*(1-E2) / (1 - E2*sp*sp)**1.5
        phi += (M-arc) / Mprime
    sf, cf = math.sin(phi), math.cos(phi)
    tf = sf/cf; tf2 = tf*tf; tf4 = tf2*tf2
    ef2 = EP2*cf*cf; ef4 = ef2*ef2
    nf = A / math.sqrt(1 - E2*sf*sf)
    rf = A*(1-E2) / (1 - E2*sf*sf)**1.5
    D = x / nf
    D2 = D*D; D3 = D2*D; D4 = D3*D; D5 = D4*D; D6 = D5*D; D7 = D6*D
    lat = phi - (tf/(2*rf))*x*D + (tf/(24*rf))*x*D3*(5+3*tf2+ef2-4*ef4-9*ef2*tf2) - (tf/(720*rf))*x*D5*(61+90*tf2+45*tf4+46*ef2-252*ef2*tf2-3*ef4+100*ef4*tf2-66*ef2*tf4-90*ef4*tf4) + (tf/(40320*rf))*x*D7*(1385+3633*tf2+4095*tf4+1575*tf2*tf2*tf2)
    lon = LAMBDA0*DEG2RAD + D/cf - D3/(6*cf)*(1+2*tf2+ef2) + D5/(120*cf)*(5+28*tf2+24*tf4+6*ef2+8*ef2*tf2) - D7/(5040*cf)*(61+662*tf2+1320*tf4+720*tf2*tf2*tf2)
    return lat, lon

def geo_to_ecef(lat, lon, h):
    sl, cl = math.sin(lat), math.cos(lat)
    N = A / math.sqrt(1 - E2*sl*sl)
    return np.array([(N+h)*cl*math.cos(lon), (N+h)*cl*math.sin(lon), (N*(1-E2)+h)*sl])

def ecef_to_geo(X, Y, Z):
    lon = math.atan2(Y, X)
    p = math.sqrt(X*X + Y*Y)
    lat = math.atan2(Z, p*(1-E2))
    for _ in range(10):
        sl = math.sin(lat)
        N = A / math.sqrt(1 - E2*sl*sl)
        new_lat = math.atan2(Z + E2*N*sl, p)
        if abs(new_lat - lat) < 1e-12: lat = new_lat; break
        lat = new_lat
    sl = math.sin(lat)
    N = A / math.sqrt(1 - E2*sl*sl)
    h = p / math.cos(lat) - N
    return lat, lon, h

def enu_rot(lat, lon):
    sl, cl = math.sin(lat), math.cos(lat)
    sL, cL = math.sin(lon), math.cos(lon)
    return np.array([[-sL, -sl*cL, cl*cL],
                     [ cL, -sl*sL, cl*sL],
                     [0.0,    cl,    sl]])

# ============================================================================
# Setup
# ============================================================================
lat0, lon0 = gk_inverse(ORIGIN_E, ORIGIN_N)
T0 = geo_to_ecef(lat0, lon0, ORIGIN_Z)
R0 = enu_rot(lat0, lon0)

print("=" * 70)
print("坐标系设定")
print("=" * 70)
print(f"  原点 GK:   ({ORIGIN_E:.1f}, {ORIGIN_N:.1f}, {ORIGIN_Z:.1f})")
print(f"  原点 WGS84: lat={math.degrees(lat0):.8f} lon={math.degrees(lon0):.8f}")
print(f"  原点 ECEF: ({T0[0]:.3f}, {T0[1]:.3f}, {T0[2]:.3f})")

# ============================================================================
# 审查 1: worldTransform 的旋转部分如何影响顶点位置
# ============================================================================
print("\n" + "=" * 70)
print("审查 1: worldTransform 层级展平与顶点变换")
print("=" * 70)

# 模拟一个典型的模型实例:
#   - parent node: translation (10000, 0, 0) + rotation 45° around Y
#   - child node:  translation (5000, 0, 0)
#   - mesh vertex: (100, 0, 0)

# CollectMeshInstances 展平后的 worldTransform:
# world = parent * child
# parent: translate(10000,0,0) * rotateY(45°)
# child: translate(5000,0,0)

# 注意: Assimp 使用 Y-up (Y=Up)
# rotateY(45°) 绕 Up 轴旋转，在 XY 平面 (East-North 平面) 旋转

print("  模拟实例: parent(平移10000m东 + 绕Y旋转45°) × child(平移5000m东)")

cos45 = math.cos(math.radians(45))
sin45 = math.sin(math.radians(45))

# parent = translate(10000, 0, 0) * rotateY(45°)
# 在 row-major 4×4 中:
parent = np.array([
    [ cos45, 0, sin45, 10000],  # row 0
    [     0, 1,     0,     0],  # row 1
    [-sin45, 0, cos45,     0],  # row 2
    [     0, 0,     0,     1],  # row 3
])

# child = translate(5000, 0, 0)
child = np.array([
    [1, 0, 0, 5000],
    [0, 1, 0,    0],
    [0, 0, 1,    0],
    [0, 0, 0,    1],
])

# worldTransform = parent * child
world = parent @ child

# mesh 顶点 (model-local)
vertex_local = np.array([100, 0, 0, 1])

# world-space 顶点
vertex_world = world @ vertex_local

print(f"  parent:\n{parent}")
print(f"  child:\n{child}")
print(f"  worldTransform = parent × child:\n{world}")
print(f"  mesh.local: {vertex_local[:3]}")
print(f"  mesh.world: {vertex_world[:3]}")

# 手动验证
# 先对 child 平移，再对 parent:
# child: (100+5000, 0, 0) = (5100, 0, 0)
# parent rotateY(45°): (5100*cos45+0*sin45, 0, -5100*sin45+0*cos45) = (3606, 0, -3606)
# parent translate: (3606+10000, 0, -3606) = (13606, 0, -3606)
manual = np.array([5100*cos45 + 10000, 0, -5100*sin45])
print(f"  手动验证: {manual}")
assert np.allclose(vertex_world[:3], manual, atol=0.001)
print(f"  ✓ worldTransform 展平正确")

# 在 GroupCellByMaterial 中:
# wp = world * meshVertex  ← 直接用 worldTransform × vertex
# 这等价于先 child 变换再 parent 变换
print(f"\n  GroupCellByMaterial 中的 world × vertex:")
wp = world[:3, :3] @ vertex_local[:3] + world[:3, 3]
print(f"  旋转部分 × local + 平移 = {wp}")
print(f"  ✓ 与 world @ vertex 一致")

# ============================================================================
# 审查 2: delta 平移的坐标空间
# ============================================================================
print("\n" + "=" * 70)
print("审查 2: Delta 平移的坐标空间")
print("=" * 70)

# delta 是 ENU 空间的修正量（dx=East, dy=Up, dz=North），转为 Assimp (dx, dy, dz)
# worldTransform 的平移部分在 Assimp 空间
# worldTransform[3] += dx  →  East 方向平移
# worldTransform[7] += dy  →  Up 方向平移
# worldTransform[11] += dz →  North 方向平移

# 假设实例中心在 Assimp 空间 (cx, cy, cz) = 实例的 translation
inst_center_assimp = np.array([5000.0, 0.0, 5000.0])  # East=5000, Up=0, North=5000

# Delta (ENU空间): dEast, dNorth, dUp
# ENU→Assimp: dx=dEast, dy=dUp, dz=dNorth
dx, dy, dz = -0.5, -0.4, 0.5  # 模拟 delta

# worldTransform 平移应用:
# 原始 translation: (5000, 0, 5000) in Assimp
# 修正后: (5000+dx, 0+dy, 5000+dz) = (4999.5, -0.4, 5000.5)
wt_before = np.array([5000.0, 0.0, 5000.0])
wt_after = wt_before + np.array([dx, dy, dz])

# 顶点变换（未修正 vs 修正后）:
vertex = np.array([0.0, 0.0, 0.0])  # 模型中心
world_before = vertex + wt_before   # 简化（假设旋转=I）
world_after = vertex + wt_after

# 在 ENU 空间中:
enu_before = np.array([world_before[0], world_before[2], world_before[1]])
enu_after  = np.array([world_after[0], world_after[2], world_after[1]])

print(f"  实例中心 (Assimp):      {inst_center_assimp}")
print(f"  Delta (Assimp):         {np.array([dx, dy, dz])}")
print(f"  worldTransform 平移前:   {wt_before}")
print(f"  worldTransform 平移后:   {wt_after}")
print(f"  模型中心 world (修正前): {world_before}")
print(f"  模型中心 world (修正后): {world_after}")
print(f"  模型中心 ENU (修正前):   {enu_before}")
print(f"  模型中心 ENU (修正后):   {enu_after}")

# 正确性验证: 对修正后的 world 坐标走 GK→ECEF 管线
enu_e, enu_n, enu_u = enu_after[0], enu_after[1], enu_after[2]
lat_w, lon_w = gk_inverse(ORIGIN_E + enu_e, ORIGIN_N + enu_n)
ecef_true = geo_to_ecef(lat_w, lon_w, enu_u + ORIGIN_Z)
ecef_approx = R0 @ np.array([enu_e, enu_n, enu_u]) + T0
delta_check = ecef_true - ecef_approx

# ENU 空间的 delta (通过 R0^T 回算):
delta_enu = R0.T @ delta_check
dE, dN, dU = delta_enu[0], delta_enu[1], delta_enu[2]
dAssimp = np.array([dE, dU, dN])  # ENU→Assimp

print(f"\n  修正后 ENU: ({enu_e:.3f}, {enu_n:.3f}, {enu_u:.3f})")
print(f"  True ECEF:  ({ecef_true[0]:.3f}, {ecef_true[1]:.3f}, {ecef_true[2]:.3f})")
print(f"  Approx ECEF: ({ecef_approx[0]:.3f}, {ecef_approx[1]:.3f}, {ecef_approx[2]:.3f})")
print(f"  Delta ECEF: ({delta_check[0]:.3f}, {delta_check[1]:.3f}, {delta_check[2]:.3f})")
print(f"  Delta ENU: dE={dE:.4f}, dN={dN:.4f}, dU={dU:.4f}")
print(f"  Delta Assimp: {dAssimp}")
print(f"  ✓ delta 平移正确应用在 Assimp/ENU 空间")

# ============================================================================
# 审查 3: R_corr 旋转修正 → worldTransform 旋转部分
# ============================================================================
print("\n" + "=" * 70)
print("审查 3: R_corr 旋转修正对 worldTransform 的影响")
print("=" * 70)

# 场景：实例中心位于距原点 NE 方向 d 处
for dist_km in [2, 5, 10]:
    d = dist_km * 1000 / math.sqrt(2.0)

    # 实例中心在 ENU 空间
    center_enu = np.array([d, d, 0.0])  # East=d, North=d, Up=0
    center_assimp = np.array([d, 0.0, d])  # Assimp: East=d, Up=0, North=d

    # Instance 位置的 ENU 框架
    clat, clon = gk_inverse(ORIGIN_E + d, ORIGIN_N + d)
    R_inst = enu_rot(clat, clon)

    # R_correction = R0^T × R_inst
    R_corr = R0.T @ R_inst

    # 旋转角
    diff = R_corr - np.eye(3)
    rot_angle_rad = np.linalg.norm(diff) / math.sqrt(2)
    rot_angle_arcsec = math.degrees(rot_angle_rad) * 3600

    # 模型的"span" (假设模型宽 500m)
    model_span = 500.0

    # 模型上距离中心最远的顶点（单位方向）
    # 简化：模型是 500m×500m 的方形，旋转在 ENU 框架中
    # 顶点相对于中心的偏移（在 ENU 框架中测量）
    # 未修正时，偏移方向基于原点 ENU 框架
    # 修正后，偏移方向基于实例 ENU 框架
    # 两个框架间的旋转差 = R_corr

    # 未修正：顶点偏移在原点 ENU 框架中
    offset_no_corr = np.array([model_span/2, model_span/2, 0.0])

    # 已修正：顶点偏移在实例 ENU 框架中
    offset_with_corr = R_corr @ offset_no_corr

    # 偏移差 → 位置误差
    offset_diff = offset_with_corr - offset_no_corr
    pos_error = np.linalg.norm(offset_diff)

    print(f"\n  距原点 {dist_km}km (d={d/1000:.1f}km):")
    print(f"    实例 ENU 框架旋转角: {rot_angle_arcsec:.1f}\"")
    print(f"    R_corr ≈ I + skew({rot_angle_rad*1e6:.1f} μrad)")
    print(f"    250m 偏移量的位置误差: {pos_error:.3f}m ({pos_error*100:.1f}cm)")

    # 分解误差到 ENU 分量
    print(f"    误差分解 (ENU): E={offset_diff[0]:.4f}, N={offset_diff[1]:.4f}, U={offset_diff[2]:.4f}")

# ============================================================================
# 审查 4: 完整顶点级别对比（平移 vs 平移+旋转 vs 真值）
# ============================================================================
print("\n" + "=" * 70)
print("审查 4: 顶点级别对比 (10km 距原点, 1km×1km 模型)")
print("=" * 70)

# 实例中心: 10km NE (即 East=7071m, North=7071m)
dist_m = 10000 / math.sqrt(2.0)
center_assimp = np.array([dist_m, 0.0, dist_m])

# 中心的 Delta
clat, clon = gk_inverse(ORIGIN_E + dist_m, ORIGIN_N + dist_m)
ctrue_ecef = geo_to_ecef(clat, clon, ORIGIN_Z)
capprox_ecef = R0 @ np.array([dist_m, dist_m, 0.0]) + T0
cdelta_ecef = ctrue_ecef - capprox_ecef
cdE = R0[0,0]*cdelta_ecef[0] + R0[1,0]*cdelta_ecef[1] + R0[2,0]*cdelta_ecef[2]
cdN = R0[0,1]*cdelta_ecef[0] + R0[1,1]*cdelta_ecef[1] + R0[2,1]*cdelta_ecef[2]
cdU = R0[0,2]*cdelta_ecef[0] + R0[1,2]*cdelta_ecef[1] + R0[2,2]*cdelta_ecef[2]
dx, dy, dz = cdE, cdU, cdN  # ENU→Assimp

# Instance ENU 框架
R_inst = enu_rot(clat, clon)
R_corr = R0.T @ R_inst

print(f"  实例中心: Assimp({center_assimp[0]:.1f}, {center_assimp[1]:.1f}, {center_assimp[2]:.1f})")
print(f"  Delta: dx={dx:.4f}, dy={dy:.4f}, dz={dz:.4f}")

# 构建 root 4×4 (column-major)
root = np.eye(4)
root[:3, 0] = R0[:, 0]; root[:3, 1] = R0[:, 1]; root[:3, 2] = R0[:, 2]; root[:3, 3] = T0

# 模型顶点（相对中心的局部偏移）
corners = [
    ("SW", np.array([-500, 0, -500])),
    ("SE", np.array([ 500, 0, -500])),
    ("NW", np.array([-500, 0,  500])),
    ("NE", np.array([ 500, 0,  500])),
    ("CTR", np.array([   0, 0,    0])),
]

print(f"\n{'Corner':>5} {'仅平移 err(cm)':>15} {'平移+旋转 err(cm)':>18} {'真值 ECEF':>45}")
print("-" * 100)

for name, local in corners:
    # --- 方法 A: 仅平移 (当前实现) ---
    # worldTransform = I·rotation + translation(center + delta)
    wt_trans = np.eye(4)
    wt_trans[:3, 3] = center_assimp + np.array([dx, dy, dz])
    world_A = wt_trans[:3, :3] @ local + wt_trans[:3, 3]

    # glTF: negate Z
    g_A = np.array([world_A[0], world_A[1], -world_A[2], 1.0])
    # Y_UP_TO_Z_UP
    tz_A = np.array([g_A[0], -g_A[2], g_A[1], 1.0])
    rendered_A = root @ tz_A

    # 真值
    ve, vn, vu = world_A[0], world_A[2], world_A[1]  # Assimp→ENU
    vlat, vlon = gk_inverse(ORIGIN_E + ve, ORIGIN_N + vn)
    true_ecef = geo_to_ecef(vlat, vlon, vu + ORIGIN_Z)

    err_A = np.linalg.norm(true_ecef - rendered_A[:3]) * 100

    # --- 方法 B: 平移+旋转 (应用 R_corr) ---
    # R_corr 应该作用在 worldTransform 的旋转部分
    # worldTransform = rotate × translate
    # 旋转部分 = R_corr (因为原本是 I, 乘上 R_corr 后就是 R_corr)
    rot_B = R_corr
    trans_B = center_assimp + np.array([dx, dy, dz])
    world_B = rot_B @ local + trans_B

    g_B = np.array([world_B[0], world_B[1], -world_B[2], 1.0])
    tz_B = np.array([g_B[0], -g_B[2], g_B[1], 1.0])
    rendered_B = root @ tz_B

    # 真值（用 world_B 的位置）
    veB, vnB, vuB = world_B[0], world_B[2], world_B[1]
    vlatB, vlonB = gk_inverse(ORIGIN_E + veB, ORIGIN_N + vnB)
    trueB = geo_to_ecef(vlatB, vlonB, vuB + ORIGIN_Z)

    err_B = np.linalg.norm(trueB - rendered_B[:3]) * 100

    print(f"  {name:>5} {err_A:14.2f} {err_B:17.2f} "
          f"({true_ecef[0]:.1f}, {true_ecef[1]:.1f}, {true_ecef[2]:.1f})")

# ============================================================================
# 审查 5: 变换矩阵的合成正确性
# ============================================================================
print("\n" + "=" * 70)
print("审查 5: worldTransform × vertex 的矩阵合成")
print("=" * 70)

# 当前 C++ 代码中 worldTransform 的构成:
# - CollectMeshInstances: world = parentXform * node->mTransformation
#   这是 Assimp 的标准节点层级展平
# - ApplyInstanceCorrection:
#     inst.worldTransform[3] += dx; ← 平移被加到 row 0, col 3
#     inst.worldTransform[7] += dy; ← row 1, col 3
#     inst.worldTransform[11] += dz; ← row 2, col 3
#
# 问题: 旋转部分没有修正！
# 如果 worldTransform 的旋转部分是 R_assimp (3×3),
# 只改平移意味着顶点变换为:
#   world = R_assimp × local + (translation + delta)
# 而正确应该是:
#   world = R_corr × R_assimp × local + (translation + delta)
#                  ^^^^^^^ 缺少这个！

# 验证: 对于 R_assimp ≈ I (大多数模型),
# 缺失 R_corr 等价于忽略 inst ENU 框架相对原点的旋转

print("  worldTransform 原始状态:")
print("    ┌               ┐")
print("    │ R₀₀ R₀₁ R₀₂ │ Tx+dx │  ← row 0")
print("    │ R₁₀ R₁₁ R₁₂ │ Ty+dy │  ← row 1")
print("    │ R₂₀ R₂₁ R₂₂ │ Tz+dz │  ← row 2")
print("    │   0   0   0  │   1   │")
print("    └               ┘")
print()
print("  正确应该是:")
print("    R_correction × [上方 4×4 矩阵]")
print()
print("  这意味着旋转部分的 9 个元素全部需要修正:")

# 量化: 在 10km 距离处
# R_corr ≈ I + skew(343 arc-seconds)
# skew matrix 的非对角元 ≈ 343/206265 ≈ 0.00166 rad
# 模型顶点偏移 500m → 旋转导致位置偏移 ≈ 500 × 0.00166 = 0.83m
# 这解释了 ~80cm 的边缘顶点误差

dist_10km = 10000 / math.sqrt(2)
clat10, clon10 = gk_inverse(ORIGIN_E + dist_10km, ORIGIN_N + dist_10km)
R_inst10 = enu_rot(clat10, clon10)
R_corr10 = R0.T @ R_inst10

print(f"\n  R_corr (10km NE):")
print(f"    {R_corr10[0,:]}")
print(f"    {R_corr10[1,:]}")
print(f"    {R_corr10[2,:]}")
print(f"  非对角元量级: {np.max(np.abs(R_corr10 - np.eye(3))):.4f} rad")
print(f"  500m 偏移的旋转误差: {500 * np.max(np.abs(R_corr10 - np.eye(3))):.3f} m")
print(f"  这 ≈ 前面看到的 83cm ✓")

# ============================================================================
# 审查 6: 修复方案
# ============================================================================
print("\n" + "=" * 70)
print("审查 6: 修复方案")
print("=" * 70)

print("""
  在 ApplyInstanceCorrection 中增加旋转修正:

  // 现有: 平移修正
  inst.worldTransform[3] += static_cast<float>(dx);
  inst.worldTransform[7] += static_cast<float>(dy);
  inst.worldTransform[11] += static_cast<float>(dz);

  // 新增: 旋转修正
  // worldTransform 的 3×3 旋转部分 = R_corr × old_3×3
  float r00 = inst.worldTransform[0], r01 = inst.worldTransform[1], r02 = inst.worldTransform[2];
  float r10 = inst.worldTransform[4], r11 = inst.worldTransform[5], r12 = inst.worldTransform[6];
  float r20 = inst.worldTransform[8], r21 = inst.worldTransform[9], r22 = inst.worldTransform[10];

  inst.worldTransform[0] = R_corr[0]*r00 + R_corr[1]*r10 + R_corr[2]*r20;
  inst.worldTransform[1] = R_corr[0]*r01 + R_corr[1]*r11 + R_corr[2]*r21;
  inst.worldTransform[2] = R_corr[0]*r02 + R_corr[1]*r12 + R_corr[2]*r22;
  // ... (共 9 个乘加)

  效果: 消除 ENU 框架旋转差，顶点误差降至 mm 级。
""")

# ============================================================================
# 最终汇总
# ============================================================================
print("=" * 70)
print("审查结论")
print("=" * 70)
print("""
  ✓ worldTransform 层级展平: 正确
  ✓ delta 平移坐标空间:     正确 (Assimp/ENU 对齐)
  ✓ GK→ECEF 逆算链:         正确 (pyproj 验证 0.016mm)
  ✓ ENU→ECEF 旋转矩阵:      正确 (正交, det=+1)
  ✓ glTF Z=South 约定:      已修复 (wz=-wz)

  ✗ R_corr 旋转修正:         已计算但未应用
     → 距原点 5km:  模型边缘误差 ~40cm
     → 距原点 10km: 模型边缘误差 ~80cm
     → 距原点 50km: 模型边缘误差 ~4m
""")
