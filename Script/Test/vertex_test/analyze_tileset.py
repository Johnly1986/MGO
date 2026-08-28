#!/usr/bin/env python3
"""
分析真实 tileset.json 输出，推断 CesiumJS 的实际坐标约定。
"""
import json
import math
import numpy as np

A = 6378137.0
F_INV = 298.257222101
DEG2RAD = math.pi / 180.0

# 加载真实 tileset.json
with open('/root/coding/MGO/build/test_final/tileset.json') as f:
    tileset = json.load(f)

root = tileset['root']
transform = np.array(root['transform']).reshape(4, 4, order='F')  # column-major -> row-major numpy

print("=" * 70)
print("真实 tileset.json 分析")
print("=" * 70)

# 分解 transform
print(f"\nRoot transform (4x4, column-major in JSON):")
print(f"  Col 0 (East_dir):  {transform[:3, 0]}")
print(f"  Col 1 (North_dir): {transform[:3, 1]}")
print(f"  Col 2 (Up_dir):    {transform[:3, 2]}")
print(f"  Col 3 (Translation): {transform[:3, 3]}")

# 验证: 这应该是 ENU->ECEF 矩阵
# 原点 ECEF = Translation
T = transform[:3, 3]
print(f"\n原点 ECEF: ({T[0]:.3f}, {T[1]:.3f}, {T[2]:.3f})")

# 从 ECEF 反算 geographic
def ecef_to_geo(X, Y, Z):
    f = 1.0/F_INV; e2 = 2*f - f*f
    lon = math.atan2(Y, X)
    p = math.sqrt(X*X + Y*Y)
    lat = math.atan2(Z, p*(1-e2))
    for _ in range(10):
        sl = math.sin(lat)
        N = A/math.sqrt(1-e2*sl*sl)
        new_lat = math.atan2(Z + e2*N*sl, p)
        if abs(new_lat - lat) < 1e-12: lat = new_lat; break
        lat = new_lat
    return lat, lon

lat0, lon0 = ecef_to_geo(*T)
print(f"原点 geographic: lat={math.degrees(lat0):.8f} lon={math.degrees(lon0):.8f}")

# 从 ENU 旋转矩阵验证
sinLon = -transform[0, 0]  # East.x = -sinLon
cosLon = transform[1, 0]   # East.y = cosLon
sinLat = transform[2, 1]   # North.z = cosLat -> sinLat = sin(transform[2,2])
cosLat = transform[2, 1]   # North.z = cosLat

print(f"\n从 East 列推断: sinLon={sinLon:.6f}, cosLon={cosLon:.6f}")
print(f"  -> lon = {math.degrees(math.atan2(sinLon, cosLon)):.6f}°")
print(f"从 Up 列推断: sinLat={transform[2, 2]:.6f}, cosLat={cosLat:.6f}")
print(f"  -> lat = {math.degrees(math.atan2(transform[2, 2], cosLat)):.6f}°")

# 分析 boundingVolume box
box = root['boundingVolume']['box']
print(f"\nRoot boundingVolume box:")
print(f"  原始: {box}")
# [cx, cy, cz, hx_x, hx_y, hx_z, hy_x, hy_y, hy_z, hz_x, hz_y, hz_z]
cx, cy, cz = box[0], box[1], box[2]
hx = np.array(box[3:6])   # X half-axis
hy = np.array(box[6:9])   # Y half-axis
hz = np.array(box[9:12])  # Z half-axis

print(f"  Center: ({cx:.2f}, {cy:.2f}, {cz:.2f})")
print(f"  X half-axis: {hx} (len={np.linalg.norm(hx):.2f})")
print(f"  Y half-axis: {hy} (len={np.linalg.norm(hy):.2f})")
print(f"  Z half-axis: {hz} (len={np.linalg.norm(hz):.2f})")

# box 的范围 (在 tile local space)
# X: [cx - |hx|, cx + |hx|] if hx aligned with X
# 实际范围是: center ± each half-axis
corners = []
for sx in [-1, 1]:
    for sy in [-1, 1]:
        for sz in [-1, 1]:
            corner = np.array([cx, cy, cz]) + sx*hx + sy*hy + sz*hz
            corners.append(corner)

corners = np.array(corners)
bbox_min = corners.min(axis=0)
bbox_max = corners.max(axis=0)
print(f"\n  Box 范围 (tile local):")
print(f"    X: [{bbox_min[0]:.2f}, {bbox_max[0]:.2f}]")
print(f"    Y: [{bbox_min[1]:.2f}, {bbox_max[1]:.2f}]")
print(f"    Z: [{bbox_min[2]:.2f}, {bbox_max[2]:.2f}]")

# 推断 tile local 坐标系约定
print(f"\n  坐标系推断:")
print(f"    X 范围 [{bbox_min[0]:.0f}, {bbox_max[0]:.0f}] - {'East' if abs(bbox_max[0]) > abs(bbox_min[0]) else 'East (negative)'}")
print(f"    Y 范围 [{bbox_min[1]:.0f}, {bbox_max[1]:.0f}] - {'North (positive)' if bbox_max[1] > abs(bbox_min[1]) else '-North (South positive)' if bbox_min[1] < -100 else 'mixed'}")
print(f"    Z 范围 [{bbox_min[2]:.0f}, {bbox_max[2]:.0f}] - {'Up' if bbox_max[2] > 0 else 'other'}")

# 检查 Y 是 -North 还是 +North
# 如果 Y = -North: 模型在 Assimp 中 North 范围 [-bbox_max[1], -bbox_min[1]]
# 如果 Y = +North: 模型 North 范围 [bbox_min[1], bbox_max[1]]
print(f"\n  如果 Y = -North (WriteBoxJson 约定):")
print(f"    模型 Assimp North 范围: [{-bbox_max[1]:.2f}, {-bbox_min[1]:.2f}]")
print(f"  如果 Y = +North:")
print(f"    模型 Assimp North 范围: [{bbox_min[1]:.2f}, {bbox_max[1]:.2f}]")

# 模拟两种渲染链，看哪个匹配
print(f"\n{'='*70}")
print(f"渲染链模拟")
print(f"{'='*70}")

# 取 box 的一个角点作为测试
test_local = np.array([cx + hx[0], cy + hy[1], cz + hz[2]])  # 远角
print(f"\n测试顶点 (tile local): {test_local}")

# 情况 A: CesiumJS 应用 Y_UP_TO_Z_UP (我的假设)
# glTF vertex = (East, Up, North) in Assimp Y-up
# Y_UP_TO_Z_UP: (x, y, z) -> (x, -z, y) = (East, -North, Up)
# 然后 root transform 应用
# But wait - tile local IS the post-Y_UP_TO_Z_UP space
# So the box is in (East, -North, Up) space
# And glTF vertices are in (East, Up, North) space
# After Y_UP_TO_Z_UP, glTF vertices become (East, -North, Up) = tile local
# Then root transform maps tile local -> ECEF

# 情况 B: CesiumJS 不应用 Y_UP_TO_Z_UP
# glTF vertex = (East, Up, North) directly used as tile local
# But the box is in (East, -North, Up) space (from WriteBoxJson)
# This would mean box and content use DIFFERENT conventions - inconsistent!

# 关键洞察: box 的 Y 范围可以告诉我们 tile local 的 Y 含义
# 如果 Y 主要是负值 (South positive), 则 Y = -North (WriteBoxJson output)
# 如果 Y 主要是正值, 则 Y = +North

print(f"\n  Box Y 范围: [{bbox_min[1]:.2f}, {bbox_max[1]:.2f}]")
if bbox_min[1] < 0 and bbox_max[1] > 0:
    print(f"  Y 跨越 0 - 难以确定方向")
elif bbox_max[1] < 0:
    print(f"  Y 全负 - 可能 Y = -North (North 全正)")
elif bbox_min[1] > 0:
    print(f"  Y 全正 - 可能 Y = +North 或 Y = South (North 全负)")
else:
    print(f"  Y 主要为 {'正' if abs(bbox_max[1]) > abs(bbox_min[1]) else '负'}")

# 验证: 用 transform 把 box 中心变换到 ECEF
box_center_local = np.array([cx, cy, cz, 1.0])
box_center_ecef = transform @ box_center_local
print(f"\n  Box 中心 (local): ({cx:.2f}, {cy:.2f}, {cz:.2f})")
print(f"  Box 中心 (ECEF):  ({box_center_ecef[0]:.3f}, {box_center_ecef[1]:.3f}, {box_center_ecef[2]:.3f})")
print(f"  原点 ECEF:        ({T[0]:.3f}, {T[1]:.3f}, {T[2]:.3f})")

# Box 中心相对原点的 ECEF 偏移
offset_ecef = box_center_ecef[:3] - T
print(f"\n  Box 中心相对原点的 ECEF 偏移: ({offset_ecef[0]:.3f}, {offset_ecef[1]:.3f}, {offset_ecef[2]:.3f})")
print(f"  偏移量级: {np.linalg.norm(offset_ecef):.3f} m")

# 如果 box center local = (East, -North, Up) (即 Y = -North)
# 则 ECEF 偏移应该 = East * East_col + (-North) * North_col + Up * Up_col
# 即: transform * (East, -North, Up, 1) - T
# 但 transform 的列是 (East, North, Up), 所以:
# transform * (East, -North, Up, 1) = East*East_col + (-North)*North_col + Up*Up_col + T

# 如果 box center local = (East, North, Up) (即 Y = +North)
# 则 ECEF 偏移 = East*East_col + North*North_col + Up*Up_col + T

# 用 East, North, Up 分别计算
east_val = cx
north_if_positive = cy  # 如果 Y=+North
north_if_negative = -cy  # 如果 Y=-North
up_val = cz

# ECEF 偏移如果 Y=+North
offset_pos = east_val * transform[:3, 0] + north_if_positive * transform[:3, 1] + up_val * transform[:3, 2]
# ECEF 偏移如果 Y=-North
offset_neg = east_val * transform[:3, 0] + north_if_negative * transform[:3, 1] + up_val * transform[:3, 2]

print(f"\n  假设 Y=+North: 预测 ECEF 偏移 = {offset_pos}")
print(f"  假设 Y=-North: 预测 ECEF 偏移 = {offset_neg}")
print(f"  实际 ECEF 偏移 = {offset_ecef}")

err_pos = np.linalg.norm(offset_pos - offset_ecef)
err_neg = np.linalg.norm(offset_neg - offset_ecef)
print(f"\n  Y=+North 误差: {err_pos:.3f} m")
print(f"  Y=-North 误差: {err_neg:.3f} m")
print(f"\n  结论: Box 的 Y 是 {'+North' if err_pos < err_neg else '-North'}")
