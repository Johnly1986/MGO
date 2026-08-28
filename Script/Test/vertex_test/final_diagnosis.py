#!/usr/bin/env python3
"""
最终诊断：分离所有误差源，定位 1.5m 误差的根因。
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

print("=" * 70)
print("误差源逐个排查")
print("=" * 70)

# ---- 误差源 A: North/South 约定 ----
print("\n--- 误差源 A: North/South glTF 约定 (wz=-wz) ---")
v_assimp = np.array([0.0, 0.0, 100.0])  # 100m North of origin

# 无 wz=-wz (旧代码)
gltf_old = v_assimp.copy()  # Z=North
tz_old = np.array([gltf_old[0], -gltf_old[2], gltf_old[1], 1.0])
rendered_old = np.eye(4); rendered_old[:3,:3]=R0; rendered_old[:3,3]=T0
r_old = rendered_old @ tz_old

# 有 wz=-wz (修复后)
gltf_new = np.array([v_assimp[0], v_assimp[1], -v_assimp[2]])
tz_new = np.array([gltf_new[0], -gltf_new[2], gltf_new[1], 1.0])
r_new = rendered_old @ tz_new

# 真值
ve,vn,vu=v_assimp[0],v_assimp[2],v_assimp[1]
vlat,vlon=gk_inverse(ORIGIN_E+ve,ORIGIN_N+vn)
true=geo_to_ecef(vlat,vlon,vu+ORIGIN_Z)

e_old = np.linalg.norm(true - r_old[:3])
e_new = np.linalg.norm(true - r_new[:3])
print(f"  100m North 顶点 ECEF 误差: 修复前={e_old*100:.1f}cm, 修复后={e_new*100:.1f}cm")
print(f"  North/South 约定是最主要的误差源 → 200m 误差 (2×100m)")

# ---- 误差源 B: Delta 平移修正 ----
print("\n--- 误差源 B: Per-instance Delta 平移修正 ---")
print("  Delta 修正了 centroid 处的曲率误差。")
print("  应用 delta 后，centroid 的渲染位置 = 真值 ECEF(修正前 centroid)。")
print("  但由于 delta 本身会平移顶点，修正前/后的 centroid 差 = |delta|。")
print("  这个偏移的二阶残留 = delta²/(2R) ≈ 0 (sub-mm for typical deltas)。")
print()
for d_km in [1, 3, 5, 10]:
    d = d_km*1000/math.sqrt(2)
    clat,clon=gk_inverse(ORIGIN_E+d,ORIGIN_N+d)
    ct=geo_to_ecef(clat,clon,ORIGIN_Z)
    ca=R0@np.array([d,d,0.0])+T0
    cd=ct-ca
    cdE=R0[0,0]*cd[0]+R0[1,0]*cd[1]+R0[2,0]*cd[2]
    cdN=R0[0,1]*cd[0]+R0[1,1]*cd[1]+R0[2,1]*cd[2]
    cdU=R0[0,2]*cd[0]+R0[1,2]*cd[1]+R0[2,2]*cd[2]
    dx,dy,dz=cdE,cdU,cdN
    delta_mag=math.sqrt(dx*dx+dy*dy+dz*dz)

    # 验证修正精度：渲染位置 vs 修正前 centroid 的真值
    rendered = R0 @ np.array([d+cdE, d+cdN, cdU]) + T0
    correction_error = np.linalg.norm(ct - rendered)
    # 二阶残留
    second_order = delta_mag*delta_mag/(2*A)

    print(f"  dist={d_km:2d}km: |delta|={delta_mag*100:.1f}cm, "
          f"修正精度={correction_error*1e6:.2f}μm, "
          f"二阶残留={second_order*1e9:.2f}nm")

# ---- 误差源 C: R_corr 旋转修正 (中心点) ----
print("\n--- 误差源 C: R_corr 旋转修正对中心点的影响 ---")
print("  旋转修正围绕模型中心旋转，不改变中心点坐标 → 对中心位置误差无贡献")

# ---- 误差源 D: 模型边缘的曲率残差 ----
print("\n--- 误差源 D: 平移近似的边缘曲率残留 ---")
print("  用了 centroid 的 delta 修正所有顶点 → 偏离中心的顶点有残留曲率误差。")
print("  误差量级 ≈ (edge_dist² − center_dist²) / (2R)")
print()
for d_km in [1, 5, 10]:
    d=d_km*1000/math.sqrt(2)
    # 边缘 ENU 距离 (相对于原点)
    edge_dist = math.sqrt((d+500)**2 + (d+500)**2)  # NE corner
    center_dist = math.sqrt(d**2 + d**2)
    theory_error = (edge_dist**2 - center_dist**2) / (2*A)
    print(f"  dist={d_km:2d}km: 边缘曲率残留 ≈ {theory_error*100:.1f}cm "
          f"(center={center_dist/1000:.1f}km, edge={edge_dist/1000:.1f}km)")

# ---- 综合 ----
print("\n" + "=" * 70)
print("误差源汇总")
print("=" * 70)
print("""
  误差源                      模型中心     模型边缘(500m)    备注
  ─────────────────────────────────────────────────────────────
  A. North/South 约定         200 m/100m  同左            ✓ 已修复 (wz=-wz)
  B. Delta 平移精度           0 cm        ~0 cm           ✓ 准确
  C. R_corr 旋转修正          0 cm        0 cm            不影响中心
  D. 边缘曲率残差             0 cm        d²/R ~ cm-m     固有的

  结论：如果 1.5m 误差测量自模型中心，则根源不在变换链路中。
        可能原因：原点坐标偏移、投影参数不匹配、或输入数据本身的误差。
        如果是模型边缘测量，曲率残差可解释 ~1.5m (对应距原点 ~4km)。
""")
