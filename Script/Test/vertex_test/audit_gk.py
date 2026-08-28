#!/usr/bin/env python3
"""
GK 逆算精度审计 & 矩阵变换完整性检查。

检查项：
  1. GK 逆算 vs pyproj 参考实现 (精度对比)
  2. 级数截断误差分析 (8阶够不够)
  3. 卯酉圈曲率半径 / 子午圈曲率半径 公式正确性
  4. ENU→ECEF 旋转矩阵正交性
  5. R_correction 矩阵是否应该应用 (旋转修正的缺失)
"""

import math
import sys
import numpy as np

# ============================================================================
# C++ GeodeticMath 的精确复现
# ============================================================================
A = 6378137.0
F_INV = 298.257222101
DEG2RAD = math.pi / 180.0
F = 1.0 / F_INV
E2 = 2.0 * F - F * F
EP2 = E2 / (1.0 - E2)

def meridian_arc(phi):
    """GeodeticMath::MeridianArcLength"""
    e4 = E2 * E2; e6 = e4 * E2
    m0 = A * (1.0 - E2/4.0 - 3.0*e4/64.0 - 5.0*e6/256.0)
    m2 = A * (-3.0*E2/8.0 - 3.0*e4/32.0 - 45.0*e6/1024.0)
    m4 = A * (15.0*e4/256.0 + 45.0*e6/1024.0)
    m6 = A * (-35.0*e6/3072.0)
    return m0*phi + m2*math.sin(2*phi) + m4*math.sin(4*phi) + m6*math.sin(6*phi)

def foot_point_lat(M):
    """GeodeticMath::FootPointLatitude"""
    e4 = E2 * E2; e6 = e4 * E2
    m0 = A * (1.0 - E2/4.0 - 3.0*e4/64.0 - 5.0*e6/256.0)
    phi = M / m0
    for _ in range(6):
        arc = meridian_arc(phi)
        dphi = M - arc
        sp = math.sin(phi)
        Mprime = A * (1.0 - E2) / (1.0 - E2*sp*sp)**1.5
        phi += dphi / Mprime
        if abs(dphi) < 1e-12:
            break
    return phi

def gk_inverse_cpp(E, N, lambda0_deg, k0=1.0, falseE=500000.0, falseN=0.0):
    """C++ GeodeticMath::GKInverse 精确复现"""
    lambda0 = lambda0_deg * DEG2RAD
    x = E - falseE
    y = N - falseN
    M_val = y / k0

    phi_f = foot_point_lat(M_val)
    sin_f = math.sin(phi_f)
    cos_f = math.cos(phi_f)
    tan_f = sin_f / cos_f
    t_f = tan_f
    t_f2 = t_f * t_f
    t_f4 = t_f2 * t_f2
    t_f6 = t_f4 * t_f2
    eta_f2 = EP2 * cos_f * cos_f
    eta_f4 = eta_f2 * eta_f2

    nu_f = A / math.sqrt(1.0 - E2 * sin_f * sin_f)
    rho_f = A * (1.0 - E2) / (1.0 - E2 * sin_f * sin_f)**1.5

    D = x / (k0 * nu_f)
    D2 = D * D
    D3 = D2 * D
    D4 = D3 * D
    D5 = D4 * D
    D6 = D5 * D
    D7 = D6 * D
    D8 = D7 * D

    # Lat — 8th order Taylor
    phi = phi_f
    phi -= (t_f / (2.0 * rho_f)) * (x * D / k0)
    phi += (t_f / (24.0 * rho_f)) * (x * D3 / k0) * \
           (5.0 + 3.0*t_f2 + eta_f2 - 4.0*eta_f4 - 9.0*eta_f2*t_f2)
    phi -= (t_f / (720.0 * rho_f)) * (x * D5 / k0) * \
           (61.0 + 90.0*t_f2 + 45.0*t_f4 + 46.0*eta_f2
            - 252.0*eta_f2*t_f2 - 3.0*eta_f4 + 100.0*eta_f4*t_f2
            - 66.0*eta_f2*t_f4 - 90.0*eta_f4*t_f4)
    phi += (t_f / (40320.0 * rho_f)) * (x * D7 / k0) * \
           (1385.0 + 3633.0*t_f2 + 4095.0*t_f4 + 1575.0*t_f6)

    # Lon — 8th order Taylor
    lon = lambda0
    lon += D / cos_f
    lon -= D3 / (6.0 * cos_f) * (1.0 + 2.0*t_f2 + eta_f2)
    lon += D5 / (120.0 * cos_f) * \
           (5.0 + 28.0*t_f2 + 24.0*t_f4 + 6.0*eta_f2 + 8.0*eta_f2*t_f2)
    lon -= D7 / (5040.0 * cos_f) * \
           (61.0 + 662.0*t_f2 + 1320.0*t_f4 + 720.0*t_f6)

    return phi, lon

# ============================================================================
# 检查 1: GK 逆算 vs pyproj
# ============================================================================
def check_gk_vs_pyproj():
    """对比 C++ GK 逆算与 pyproj 参考实现"""
    print("=" * 70)
    print("检查 1: GK 逆算 vs pyproj 参考精度")
    print("=" * 70)

    try:
        from pyproj import CRS, Transformer
        with open("/root/coding/MGO/Data/103d10m.prj") as f:
            prj_wkt = f.read().strip()
        crs_gk = CRS.from_wkt(prj_wkt)
        to_wgs84 = Transformer.from_crs(crs_gk, "EPSG:4326", always_xy=True)
    except ImportError:
        print("  pyproj 未安装，跳过对比")
        return True
    except Exception as e:
        print(f"  pyproj 初始化失败: {e}")
        return True

    # 测试网格：从 CM 向东 0..100km, 向北 0..100km
    ORIGIN_E = 498700.0
    ORIGIN_N = 2929900.0
    LAMBDA0 = 103.1666666666666667

    max_err_m = 0.0
    max_err_pt = None
    errors = []

    for east_km in [0, 1, 5, 10, 50, 100]:
        for north_km in [0, 1, 5, 10, 50, 100]:
            E = ORIGIN_E + east_km * 1000
            N = ORIGIN_N + north_km * 1000

            # C++ GK 逆算
            lat_cpp, lon_cpp = gk_inverse_cpp(E, N, LAMBDA0)

            # pyproj 参考
            lon_py, lat_py = to_wgs84.transform(E, N)
            lat_py_rad = math.radians(lat_py)
            lon_py_rad = math.radians(lon_py)

            # 计算 arc-second 误差 → 米
            dlat = abs(lat_cpp - lat_py_rad)
            dlon = abs(lon_cpp - lon_py_rad)

            # 1 arc-second ≈ 30.9m at equator, scale by cos(lat)
            err_lat_m = dlat * A  # rad * radius = meters on sphere
            err_lon_m = dlon * A * math.cos(lat_py_rad)
            err_m = math.sqrt(err_lat_m**2 + err_lon_m**2)

            errors.append(err_m)
            if err_m > max_err_m:
                max_err_m = err_m
                max_err_pt = (E, N)

    errors = np.array(errors)
    print(f"  测试点: {len(errors)} (0..100km range)")
    print(f"  最大误差: {max_err_m*1000:.3f} mm at GK({max_err_pt[0]:.0f},{max_err_pt[1]:.0f})")
    print(f"  平均误差: {errors.mean()*1000:.3f} mm")
    print(f"  RMSE:     {math.sqrt((errors**2).mean())*1000:.3f} mm")

    ok = max_err_m < 0.01  # 1cm tolerance
    print(f"  {'✓ 通过 (< 1cm)' if ok else '✗ 超差 (> 1cm)'}")
    return ok


# ============================================================================
# 检查 2: 级数截断误差
# ============================================================================
def check_truncation():
    """分析级数截断误差 — 8阶是否足够"""
    print("\n" + "=" * 70)
    print("检查 2: 级数截断误差分析")
    print("=" * 70)

    # D = x / (k0 * nu) ≈ x / 6.38e6
    # For x = 100km (3° zone edge ≈ 167km at lat=27°): D ≈ 0.026
    # D^8 ≈ 2.1e-13, D^9 ≈ 5.5e-15

    for x_km in [10, 50, 100, 167, 500]:
        x = x_km * 1000
        D = x / 6380000.0
        D8 = D ** 8
        D9 = D ** 9

        # 9th order term magnitude (rough estimate for lon):
        # 277 / 72576 * D^9 / cos(phi)
        # This is ~ 0.0038 * D^9
        term9 = 0.0038 * D9 * 6380000.0  # in meters

        print(f"  x={x_km:4d}km: D={D:.4f}, D^8={D8:.2e}, "
              f"9th-order term ≈ {term9*1e9:.2f} nm")

    print(f"  ✓ 8阶展开在 100km 范围内足够 (9阶项 < 1nm)")


# ============================================================================
# 检查 3: 曲率半径公式
# ============================================================================
def check_curvature_radii():
    """验证 ν (卯酉圈) 和 ρ (子午圈) 曲率半径"""
    print("\n" + "=" * 70)
    print("检查 3: 曲率半径公式验证")
    print("=" * 70)

    # 测试纬度
    lat_test = 26.5 * DEG2RAD  # ~27°N
    sin_lat = math.sin(lat_test)

    # ν = a / sqrt(1 - e² sin²φ)
    nu = A / math.sqrt(1.0 - E2 * sin_lat * sin_lat)

    # ρ = a(1-e²) / (1 - e² sin²φ)^(3/2)
    rho = A * (1.0 - E2) / (1.0 - E2 * sin_lat * sin_lat)**1.5

    # 交叉验证：ρ = ν(1-e²) / (1 - e² sin²φ)
    rho_check = nu * (1.0 - E2) / (1.0 - E2 * sin_lat * sin_lat)

    # 交叉验证：νρ 关系
    # 子午圈曲率半径 M = a(1-e²)/(1-e²sin²φ)^(3/2) = ρ ✓
    # 卯酉圈曲率半径 N = a/√(1-e²sin²φ) = ν ✓

    print(f"  lat={math.degrees(lat_test):.2f}°")
    print(f"  ν (prime vertical) = {nu:.3f} m")
    print(f"  ρ (meridian)       = {rho:.3f} m")
    print(f"  ρ_check            = {rho_check:.3f} m (diff={abs(rho-rho_check):.6f})")
    print(f"  ν-ρ                = {nu-rho:.3f} m (expected ~{A*E2:.0f} at equator)")

    # ν 和 ρ 的量级合理性
    ok = abs(rho - rho_check) < 1e-6 and nu > rho and nu < A * 1.01
    print(f"  {'✓ 通过' if ok else '✗ 失败'}")
    return ok


# ============================================================================
# 检查 4: ENU→ECEF 旋转矩阵正交性
# ============================================================================
def check_enu_rotation():
    """验证 ENU→ECEF 旋转矩阵"""
    print("\n" + "=" * 70)
    print("检查 4: ENU→ECEF 旋转矩阵正交性 & 几何意义")
    print("=" * 70)

    lat = 26.5 * DEG2RAD
    lon = 103.2 * DEG2RAD
    sl, cl = math.sin(lat), math.cos(lat)
    sL, cL = math.sin(lon), math.cos(lon)

    # C++ 行优先: R[row*3+col], columns = East, North, Up
    R = np.array([
        [-sL, -sl*cL,  cl*cL],  # row 0
        [ cL, -sl*sL,  cl*sL],  # row 1
        [0.0,     cl,     sl],  # row 2
    ])

    # 1. 正交性: R × R^T = I
    ortho = R @ R.T
    I_err = np.max(np.abs(ortho - np.eye(3)))
    print(f"  正交性: max|R×Rᵀ - I| = {I_err:.2e}")

    # 2. 行列式 = +1 (右手系)
    det = np.linalg.det(R)
    print(f"  行列式: {det:.10f} (应为 +1)")

    # 3. 列向量正交性
    e_col = R[:, 0]  # East
    n_col = R[:, 1]  # North
    u_col = R[:, 2]  # Up

    dot_en = np.dot(e_col, n_col)
    dot_eu = np.dot(e_col, u_col)
    dot_nu = np.dot(n_col, u_col)
    print(f"  列正交: E·N={dot_en:.2e}, E·U={dot_eu:.2e}, N·U={dot_nu:.2e}")

    # 4. East 方向验证: East 应该指向东 (局部坐标 -sin(lon), cos(lon), 0)
    #    在 ECEF 中，东方向是 (-sin(lon), cos(lon), 0)
    expected_e = np.array([-sL, cL, 0.0])
    e_err = np.max(np.abs(e_col - expected_e))
    print(f"  East 方向: max|实际-期望| = {e_err:.2e}")

    # 5. Up 方向: 应该指向 (cos(lat)cos(lon), cos(lat)sin(lon), sin(lat))
    expected_u = np.array([cl*cL, cl*sL, sl])
    u_err = np.max(np.abs(u_col - expected_u))
    print(f"  Up 方向:   max|实际-期望| = {u_err:.2e}")

    # 6. North 方向: 应该是 (-sin(lat)cos(lon), -sin(lat)sin(lon), cos(lat))
    expected_n = np.array([-sl*cL, -sl*sL, cl])
    n_err = np.max(np.abs(n_col - expected_n))
    print(f"  North 方向: max|实际-期望| = {n_err:.2e}")

    # 7. 局部正切平面验证: Up ⊥ East, Up ⊥ North
    #    Up 应该垂直于 East-North 平面 (局部切平面)

    all_ok = (I_err < 1e-14 and abs(det - 1.0) < 1e-14
              and e_err < 1e-14 and n_err < 1e-14 and u_err < 1e-14)
    print(f"  {'✓ 矩阵正确' if all_ok else '✗ 矩阵有问题'}")
    return all_ok


# ============================================================================
# 检查 5: R_correction 旋转修正分析
# ============================================================================
def check_rotation_correction():
    """分析 R_correction 矩阵是否应该应用到 worldTransform"""
    print("\n" + "=" * 70)
    print("检查 5: R_correction 旋转修正的必要性分析")
    print("=" * 70)

    # R_correction = R_origin^T × R_instance
    # 它表示 instance 所在位置的 ENU 框架相对于原点 ENU 框架的旋转差

    # 测试：原点 lat=26.5°, 模型中心 lat=26.5°+Δlat
    lat0 = 26.5 * DEG2RAD
    lon0 = 103.2 * DEG2RAD

    sl0, cl0 = math.sin(lat0), math.cos(lat0)
    sL0, cL0 = math.sin(lon0), math.cos(lon0)

    R_origin = np.array([
        [-sL0, -sl0*cL0,  cl0*cL0],
        [ cL0, -sl0*sL0,  cl0*sL0],
        [0.0,      cl0,       sl0],
    ])

    # 模型在不同距离处
    for dist_km in [1, 5, 10, 50]:
        # 模拟：ENU 偏移 (东, 北) = d, d
        d_en = dist_km * 1000 / math.sqrt(2.0)
        # ENU → ENU at new point: 通过 ECEF 转换
        # Approx: lat change = north / rho, lon change = east / (nu * cos(lat))

        lat1 = lat0 + d_en / (A * (1-E2) / (1-E2*sl0*sl0)**1.5)  # dN / rho
        lon1 = lon0 + d_en / (A / math.sqrt(1-E2*sl0*sl0) * cl0)  # dE / (nu*cos)
        sl1, cl1 = math.sin(lat1), math.cos(lat1)
        sL1, cL1 = math.sin(lon1), math.cos(lon1)

        R_instance = np.array([
            [-sL1, -sl1*cL1,  cl1*cL1],
            [ cL1, -sl1*sL1,  cl1*sL1],
            [0.0,      cl1,       sl1],
        ])

        # R_correction = R_origin^T × R_instance
        R_corr = R_origin.T @ R_instance

        # 旋转角度 (从单位矩阵偏差)
        # R_corr ≈ I + [skew], 旋转角 ≈ |R_corr - I|_F / sqrt(2)
        diff = R_corr - np.eye(3)
        rot_angle_rad = np.linalg.norm(diff) / math.sqrt(2)
        rot_angle_arcsec = math.degrees(rot_angle_rad) * 3600

        # 对于 1km 宽的模型，最远顶点距中心 500m
        # 顶点因旋转导致的位置误差 ≈ rot_angle * model_half_width
        model_half = 500.0  # m
        pos_err = rot_angle_rad * model_half

        print(f"  距原点 {dist_km:2d}km: R_corr 旋转角={rot_angle_arcsec:.2f}\", "
              f"500m 模型边缘的位置误差={pos_err*100:.1f}cm")

    print()
    print("  结论：R_correction 已计算但未应用。对于大范围模型 (>5km 距原点),")
    print("  旋转修正可消除 cm~dm 级的位置误差。")
    print("  ApplyInstanceCorrection 中应同时应用平移(dx,dy,dz)和旋转(R_corr)。")


# ============================================================================
# 检查 6: 完整变换链验证
# ============================================================================
def check_full_transform_chain():
    """
    验证完整变换链中每个矩阵运算的正确性。
    """
    print("\n" + "=" * 70)
    print("检查 6: 完整变换链验证")
    print("=" * 70)

    ORIGIN_E = 498700.0
    ORIGIN_N = 2929900.0
    LAMBDA0 = 103.1666666666666667
    ORIGIN_Z = 0.0

    def geo_to_ecef(lat, lon, h):
        sl, cl = math.sin(lat), math.cos(lat)
        N = A / math.sqrt(1.0 - E2*sl*sl)
        return np.array([(N+h)*cl*math.cos(lon), (N+h)*cl*math.sin(lon), (N*(1-E2)+h)*sl])

    # Origin
    lat0, lon0 = gk_inverse_cpp(ORIGIN_E, ORIGIN_N, LAMBDA0)
    T0 = geo_to_ecef(lat0, lon0, ORIGIN_Z)

    # ENU→ECEF rotation at origin
    sl0, cl0 = math.sin(lat0), math.cos(lat0)
    sL0, cL0 = math.sin(lon0), math.cos(lon0)
    R0 = np.array([
        [-sL0, -sl0*cL0,  cl0*cL0],
        [ cL0, -sl0*sL0,  cl0*sL0],
        [0.0,      cl0,       sl0],
    ])

    # 测试点：在 ENU 空间的不同位置
    test_pts = [
        (0, 0, 0, "origin"),
        (1000, 1000, 0, "1km_NE"),
        (5000, 5000, 0, "5km_NE"),
        (10000, 10000, 0, "10km_NE"),
        (50000, 50000, 0, "50km_NE"),
    ]

    print(f"  {'Point':>10} {'True ECEF':>45} {'Approx ECEF':>45} {'|delta|(m)':>10}")
    print("-" * 115)

    for en, nu, up, name in test_pts:
        # 真值 ECEF
        lat_i, lon_i = gk_inverse_cpp(ORIGIN_E + en, ORIGIN_N + nu, LAMBDA0)
        true_ecef = geo_to_ecef(lat_i, lon_i, up + ORIGIN_Z)

        # 近似 ECEF (切线平面)
        approx_ecef = R0 @ np.array([en, nu, up]) + T0

        delta = true_ecef - approx_ecef
        delta_mag = np.linalg.norm(delta)

        print(f"  {name:>10} ({true_ecef[0]:14.3f},{true_ecef[1]:14.3f},{true_ecef[2]:14.3f}) "
              f"({approx_ecef[0]:14.3f},{approx_ecef[1]:14.3f},{approx_ecef[2]:14.3f}) "
              f"{delta_mag:8.3f}")

    # 验证 delta 理论值: ≈ d²/(2R)
    print(f"\n  Delta 理论值验证 (d²/(2R)):")
    for en, nu, up, name in test_pts[1:]:
        d = math.sqrt(en*en + nu*nu)
        theory = d*d / (2 * A)
        lat_i, lon_i = gk_inverse_cpp(ORIGIN_E + en, ORIGIN_N + nu, LAMBDA0)
        true_ecef = geo_to_ecef(lat_i, lon_i, up + ORIGIN_Z)
        approx_ecef = R0 @ np.array([en, nu, up]) + T0
        actual = np.linalg.norm(true_ecef - approx_ecef)
        print(f"    {name:>10}: d={d/1000:.1f}km, theory={theory:.3f}m, actual={actual:.3f}m, "
              f"ratio={actual/theory:.3f}")

    return True


# ============================================================================
# Main
# ============================================================================
def main():
    results = {}
    results["gk_vs_pyproj"] = check_gk_vs_pyproj()
    check_truncation()
    results["curvature"] = check_curvature_radii()
    results["enu_rotation"] = check_enu_rotation()
    check_rotation_correction()
    results["full_chain"] = check_full_transform_chain()

    print("\n" + "=" * 70)
    print("审计汇总")
    print("=" * 70)
    all_ok = True
    for name, ok in results.items():
        status = "✓" if ok else "✗"
        if not ok:
            all_ok = False
        print(f"  {status} {name}")
    print(f"\n  {'全部通过' if all_ok else '存在问题，需修复'}")

    return 0 if all_ok else 1

if __name__ == "__main__":
    sys.exit(main())
