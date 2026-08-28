#!/usr/bin/env python3
"""
链路一致性验证器：对比程序链路输出 vs 直接世界坐标->ECEF 转换。

验证方法：
  对每个模型顶点，分别计算：
    A) 程序链路：local -> worldTransform -> glTF -> Y_UP_TO_Z_UP -> root transform -> ECEF
    B) 直接转换：local -> worldTransform -> world(E,N,U) -> GK逆算 -> ECEF

  两者应该一致。如果不一致，则链路存在 bug。

支持三种输入模式：
  1. 合成测试：用脚本生成的几何体
  2. OBJ 文件：读取 .obj 顶点
  3. C++ 输出：读取 C++ 程序导出的中间数据 (JSON/CSV)

使用方法：
  python3 chain_vs_direct.py                          # 合成测试
  python3 chain_vs_direct.py --obj path/to/model.obj # OBJ 文件
"""

import math
import json
import argparse
import numpy as np

# ============================================================================
# 投影参数（与 C++ CProjectionEngine::LoadProjection 一致）
# ============================================================================
A = 6378137.0
F_INV = 298.257222101
DEG2RAD = math.pi / 180.0
F = 1.0 / F_INV
E2 = 2.0 * F - F * F
EP2 = E2 / (1.0 - E2)

# 默认原点（与 Data/routeOriginPt.txt 一致）
ORIGIN_E = 498700.0
ORIGIN_N = 2929900.0
ORIGIN_Z = 0.0
LAMBDA0 = 103.1666666666666667

# ============================================================================
# C++ GeodeticMath 精确复现
# ============================================================================
def gk_inverse(E, N, lambda0_deg=LAMBDA0, k0=1.0, falseE=500000.0, falseN=0.0):
    lambda0 = lambda0_deg * DEG2RAD
    x, y = E - falseE, N - falseN
    M = y / k0
    e4 = E2*E2
    e6 = e4*E2
    m0 = A*(1 - E2/4 - 3*e4/64 - 5*e6/256)
    phi = M / m0
    for _ in range(6):
        m2 = A*(-3*E2/8 - 3*e4/32 - 45*e6/1024)
        m4 = A*(15*e4/256 + 45*e6/1024)
        m6 = A*(-35*e6/3072)
        arc = m0*phi + m2*math.sin(2*phi) + m4*math.sin(4*phi) + m6*math.sin(6*phi)
        if abs(M - arc) < 1e-12: break
        sp = math.sin(phi)
        Mprime = A*(1-E2) / (1 - E2*sp*sp)**1.5
        phi += (M - arc) / Mprime
    sf, cf = math.sin(phi), math.cos(phi)
    tf = sf/cf; tf2 = tf*tf; tf4 = tf2*tf2
    ef2 = EP2*cf*cf; ef4 = ef2*ef2
    nf = A/math.sqrt(1 - E2*sf*sf)
    rf = A*(1-E2) / (1 - E2*sf*sf)**1.5
    D = x / nf
    D2 = D*D; D3 = D2*D; D4 = D3*D; D5 = D4*D; D7 = D5*D*D
    lat = phi - (tf/(2*rf))*x*D + (tf/(24*rf))*x*D3*(5+3*tf2+ef2-4*ef4-9*ef2*tf2) \
          - (tf/(720*rf))*x*D5*(61+90*tf2+45*tf4+46*ef2-252*ef2*tf2-3*ef4+100*ef4*tf2-66*ef2*tf4-90*ef4*tf4) \
          + (tf/(40320*rf))*x*D7*(1385+3633*tf2+4095*tf4+1575*tf2*tf2*tf2)
    lon = lambda0 + D/cf - D3/(6*cf)*(1+2*tf2+ef2) + D5/(120*cf)*(5+28*tf2+24*tf4+6*ef2+8*ef2*tf2) \
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

# AxisMapper (与 C++ 一致)
def assimp_to_enu(x, y, z):
    """Assimp(E,U,N) -> ENU(E,N,U)"""
    return x, z, y

def enu_to_assimp(e, n, u):
    """ENU(E,N,U) -> Assimp(E,U,N)"""
    return e, u, n

# ============================================================================
# 程序链路（精确复现 C++ 代码）
# ============================================================================
def program_chain(vertex_local, world_transform_4x4, delta_assimp,
                   R0, T0, apply_z_negation=False):
    """
    程序链路：local -> worldTransform -> glTF -> Y_UP_TO_Z_UP -> root -> ECEF

    参数：
      vertex_local: 模型局部空间顶点 (3,)
      world_transform_4x4: 4x4 worldTransform (row-major, Assimp 约定)
      delta_assimp: (dx, dy, dz) 在 Assimp 空间的平移修正
      R0: 3x3 ENU->ECEF 旋转矩阵 (在原点)
      T0: 3D 原点 ECEF 位置
      apply_z_negation: 是否应用 wz=-wz (测试两种配置)

    返回：
      chain_ecef: 链路输出的 ECEF 坐标 (3,)
      debug: 中间值字典 (用于调试)
    """
    # Step 1: worldTransform * vertex (含 delta 平移)
    # C++: aiVector3D wp = world * mesh->mVertices[vi];
    # delta 已经被加到 worldTransform[3,7,11]
    v4 = np.append(vertex_local, 1.0)
    wp = world_transform_4x4 @ v4
    wx, wy, wz = wp[0], wp[1], wp[2]

    debug = {
        'world_pos': np.array([wx, wy, wz]),
    }

    # Step 2: GroupCellByMaterial 中的 wz=-wz (可选)
    if apply_z_negation:
        wz = -wz
    debug['gltf_vertex'] = np.array([wx, wy, wz])

    # Step 3: CesiumJS Y_UP_TO_Z_UP
    # 矩阵: [1,0,0,0; 0,0,-1,0; 0,1,0,0; 0,0,0,1]
    # 映射: (x, y, z) -> (x, -z, y)
    cesium_zup = np.array([wx, -wz, wy])
    debug['cesium_zup'] = cesium_zup

    # Step 4: root transform (ENU->ECEF 4x4, column-major)
    # C++: AxisMapper::BuildRootTransform(R0, T0, transform)
    # transform 是 column-major: transform[col*4+row]
    # 列 0 = East, 列 1 = North, 列 2 = Up, 列 3 = Translation
    root_4x4 = np.eye(4)
    root_4x4[:3, 0] = R0[:, 0]  # East 列
    root_4x4[:3, 1] = R0[:, 1]  # North 列
    root_4x4[:3, 2] = R0[:, 2]  # Up 列
    root_4x4[:3, 3] = T0         # Translation

    v4_zup = np.append(cesium_zup, 1.0)
    chain_ecef = root_4x4 @ v4_zup
    debug['chain_ecef'] = chain_ecef[:3]
    debug['root_4x4'] = root_4x4

    return chain_ecef[:3], debug

# ============================================================================
# 直接转换 (ground truth)
# ============================================================================
def direct_to_ecef(vertex_local, world_transform_4x4, delta_assimp,
                   origin_e, origin_n, origin_z):
    """
    直接转换：local -> worldTransform -> world(E,N,U) -> GK逆算 -> ECEF

    这是"真值"，不依赖任何 CesiumJS 假设。
    """
    # worldTransform * vertex (含 delta)
    v4 = np.append(vertex_local, 1.0)
    wp = world_transform_4x4 @ v4
    wx, wy, wz = wp[0], wp[1], wp[2]

    # Assimp(E,U,N) -> ENU(E,N,U)
    enu_e, enu_n, enu_u = assimp_to_enu(wx, wy, wz)

    # 加上原点偏移得到投影坐标
    proj_e = origin_e + enu_e
    proj_n = origin_n + enu_n
    proj_h = origin_z + enu_u

    # GK 逆算 -> geographic
    lat, lon = gk_inverse(proj_e, proj_n)

    # Geographic -> ECEF
    ecef = geo_to_ecef(lat, lon, proj_h)

    return ecef, {
        'world_pos': np.array([wx, wy, wz]),
        'enu': np.array([enu_e, enu_n, enu_u]),
        'proj': np.array([proj_e, proj_n, proj_h]),
        'lat': lat, 'lon': lon,
        'direct_ecef': ecef,
    }

# ============================================================================
# Delta 计算 (复现 C++ ComputeInstanceProjectionDelta)
# ============================================================================
def compute_delta(centroid_assimp, origin_e, origin_n, origin_z, R0, T0):
    """计算 per-instance delta (复现 C++ 代码)"""
    cx, cy, cz = centroid_assimp
    enu_e, enu_n, enu_u = assimp_to_enu(cx, cy, cz)

    # 真值 ECEF
    lat_i, lon_i = gk_inverse(origin_e + enu_e, origin_n + enu_n)
    true_ecef = geo_to_ecef(lat_i, lon_i, enu_u + origin_z)

    # 近似 ECEF (single ENU->ECEF at origin)
    approx_ecef = R0 @ np.array([enu_e, enu_n, enu_u]) + T0

    # Delta
    delta_ecef = true_ecef - approx_ecef
    Rt = R0.T
    dEast  = Rt[0,0]*delta_ecef[0] + Rt[0,1]*delta_ecef[1] + Rt[0,2]*delta_ecef[2]
    dNorth = Rt[1,0]*delta_ecef[0] + Rt[1,1]*delta_ecef[1] + Rt[1,2]*delta_ecef[2]
    dUp    = Rt[2,0]*delta_ecef[0] + Rt[2,1]*delta_ecef[1] + Rt[2,2]*delta_ecef[2]

    # ENU -> Assimp
    dx, dy, dz = enu_to_assimp(dEast, dNorth, dUp)
    return np.array([dx, dy, dz])

# ============================================================================
# 验证主函数
# ============================================================================
def verify_consistency(vertices_local, world_transforms, centroids,
                       origin_e, origin_n, origin_z, label="test"):
    """
    对一组顶点验证链路一致性。
    """
    # Setup
    lat0, lon0 = gk_inverse(origin_e, origin_n)
    T0 = geo_to_ecef(lat0, lon0, origin_z)
    R0 = enu_rot(lat0, lon0)

    print(f"\n{'='*70}")
    print(f"验证: {label}")
    print(f"{'='*70}")
    print(f"  原点 GK:   ({origin_e:.1f}, {origin_n:.1f}, {origin_z:.1f})")
    print(f"  原点 WGS84: lat={math.degrees(lat0):.8f}, lon={math.degrees(lon0):.8f}")
    print(f"  原点 ECEF: ({T0[0]:.3f}, {T0[1]:.3f}, {T0[2]:.3f})")
    print(f"  顶点数: {len(vertices_local)}")

    # 测试两种配置：有/无 wz=-wz
    for z_neg in [False, True]:
        label_z = "wz=-wz (我之前的错误修复)" if z_neg else "无 wz=-wz (原始)"
        print(f"\n  --- 配置: {label_z} ---")

        errors = []
        for i, (v_local, wt, centroid) in enumerate(zip(vertices_local,
                                                          world_transforms,
                                                          centroids)):
            # 计算 delta (per-instance)
            delta = compute_delta(centroid, origin_e, origin_n, origin_z, R0, T0)

            # 应用 delta 到 worldTransform (C++ ApplyInstanceCorrection)
            wt_with_delta = wt.copy()
            wt_with_delta[0, 3] += delta[0]
            wt_with_delta[1, 3] += delta[1]
            wt_with_delta[2, 3] += delta[2]

            # 链路输出
            chain_ecef, chain_debug = program_chain(
                v_local, wt_with_delta, delta, R0, T0, apply_z_negation=z_neg)

            # 直接转换 (真值)
            direct_ecef, direct_debug = direct_to_ecef(
                v_local, wt_with_delta, delta, origin_e, origin_n, origin_z)

            err = np.linalg.norm(direct_ecef - chain_ecef)
            errors.append(err)

            if i < 3 or i == len(vertices_local) - 1:
                print(f"    顶点 {i}: local={v_local}")
                print(f"      world: {chain_debug['world_pos']}")
                print(f"      direct ECEF: ({direct_ecef[0]:.3f}, {direct_ecef[1]:.3f}, {direct_ecef[2]:.3f})")
                print(f"      chain  ECEF: ({chain_ecef[0]:.3f}, {chain_ecef[1]:.3f}, {chain_ecef[2]:.3f})")
                print(f"      差异: {err*100:.2f} cm")

        errors = np.array(errors)
        print(f"\n    汇总 ({len(errors)} 顶点):")
        print(f"      最大差异: {errors.max()*100:.2f} cm")
        print(f"      平均差异: {errors.mean()*100:.2f} cm")
        print(f"      RMSE:     {math.sqrt((errors**2).mean())*100:.2f} cm")
        verdict = "✓ 一致 (链路正确)" if errors.max() < 0.01 else "✗ 不一致 (链路有 bug)"
        print(f"      结论: {verdict}")

# ============================================================================
# 合成测试数据
# ============================================================================
def synthetic_test():
    """合成测试：模拟 TilesConverter 的典型场景"""
    # 实例 1: 位于原点附近，无旋转
    vertices_local_1 = [
        np.array([0.0, 0.0, 0.0]),    # 中心
        np.array([100.0, 0.0, 0.0]),   # 东 100m
        np.array([0.0, 0.0, 100.0]),   # 北 100m
        np.array([100.0, 0.0, 100.0]), # 东北
        np.array([-100.0, 0.0, -100.0]), # 西南
    ]
    # worldTransform: translation(5000, 0, 5000) in Assimp space
    # 即实例中心在 (East=5000, Up=0, North=5000)
    wt_1 = np.eye(4)
    wt_1[:3, 3] = [5000.0, 0.0, 5000.0]
    centroids_1 = [np.array([5000.0, 0.0, 5000.0])] * len(vertices_local_1)

    verify_consistency(vertices_local_1,
                       [wt_1] * len(vertices_local_1),
                       centroids_1,
                       ORIGIN_E, ORIGIN_N, ORIGIN_Z,
                       "合成测试 1: 实例在 (5km E, 5km N)")

    # 实例 2: 距原点更远 (10km NE)
    vertices_local_2 = [
        np.array([0.0, 0.0, 0.0]),
        np.array([500.0, 0.0, 500.0]),
        np.array([-500.0, 0.0, -500.0]),
    ]
    wt_2 = np.eye(4)
    wt_2[:3, 3] = [10000.0, 0.0, 10000.0]
    centroids_2 = [np.array([10000.0, 0.0, 10000.0])] * len(vertices_local_2)

    verify_consistency(vertices_local_2,
                       [wt_2] * len(vertices_local_2),
                       centroids_2,
                       ORIGIN_E, ORIGIN_N, ORIGIN_Z,
                       "合成测试 2: 实例在 (10km E, 10km N)")

# ============================================================================
# OBJ 文件测试
# ============================================================================
def obj_test(obj_path):
    """读取 OBJ 文件顶点，假设所有顶点共享一个 worldTransform"""
    vertices = []
    with open(obj_path, 'r') as f:
        for line in f:
            if line.startswith('v '):
                parts = line.split()
                vertices.append(np.array([float(parts[1]), float(parts[2]), float(parts[3])]))

    print(f"\n读取 {len(vertices)} 顶点 from {obj_path}")

    # 假设 OBJ 顶点已在 world space (无 worldTransform)
    # 实例中心 = 顶点 bbox 中心
    vertices_arr = np.array(vertices)
    bbox_min = vertices_arr.min(axis=0)
    bbox_max = vertices_arr.max(axis=0)
    centroid = (bbox_min + bbox_max) / 2

    # worldTransform = identity (顶点已在 world space)
    wt = np.eye(4)

    verify_consistency(vertices,
                       [wt] * len(vertices),
                       [centroid] * len(vertices),
                       ORIGIN_E, ORIGIN_N, ORIGIN_Z,
                       f"OBJ 测试: {obj_path}")

# ============================================================================
# Main
# ============================================================================
def main():
    parser = argparse.ArgumentParser(description="链路一致性验证器")
    parser.add_argument('--obj', help="OBJ 模型文件路径")
    parser.add_argument('--origin-e', type=float, default=ORIGIN_E, help="原点东向")
    parser.add_argument('--origin-n', type=float, default=ORIGIN_N, help="原点北向")
    parser.add_argument('--origin-z', type=float, default=ORIGIN_Z, help="原点高程")
    args = parser.parse_args()

    if args.obj:
        obj_test(args.obj)
    else:
        synthetic_test()

if __name__ == "__main__":
    main()
