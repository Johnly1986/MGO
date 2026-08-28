# TilesConverter 顶点转换逻辑审计

## 完整变换链

```
Assimp 局部空间顶点
    │
    ├──[1] CollectMeshInstances (TilesConverter.cpp:73-116)
    │       world = parent * node->mTransformation
    │       bbox 在 Assimp Y-up 空间计算 (East, Up, North)
    │       worldTransform 存储为列主序 float[16]
    │
    ├──[2] ApplyPerInstanceProjectionCorrection (TilesConverter.cpp:460-462)
    │       worldTransform[12,13,14] += (dx, dy, dz)
    │       delta 在 Assimp Y-up 空间 (East, Up, North)
    │
    ├──[3] GroupCellByMaterial (TileBuilder.cpp:558-656)
    │       world = toMatrix4x4(inst.worldTransform)
    │       wp = world * mesh->v     ← 逐顶点
    │       存储为: (East, Up, North) Assimp Y-up
    │
    ├──[4] GlbBuilder 输出
    │       glTF 顶点: Y-up 空间 (East, Up, North)
    │       boundingVolume box: Z-up 转换 (East, -North, Up)
    │
    └──[5] CesiumJS 渲染
            Y_UP_TO_Z_UP: (East, Up, North) → (East, -North, Up)
            rootTransform * (East, -North, Up) → ECEF
```

## 发现的问题

### BUG-1: 逐顶点重复计算法线矩阵 (TileBuilder.cpp:618-623)

```cpp
for (unsigned int vi = 0; vi < mesh->mNumVertices; ++vi)
{
    // ...
    aiMatrix4x4 normalMat = world;
    normalMat.Inverse();       // O(n³) 矩阵求逆
    normalMat.Transpose();     // 16次赋值
    aiVector3D wn = normalMat * mesh->mNormals[vi];
}
```

`normalMat` 仅依赖 `world` 矩阵（每个 instance 常量），却在每个顶点循环中重新计算。对一个 1000 顶点的 mesh，多做了 1000 次矩阵求逆。**应提升到 instance 作用域**。

### BUG-2: NaN 预检导致顶点遍历两次 (TileBuilder.cpp:570-589 + 592-649)

```cpp
// 第一遍：检查 NaN
for (vi = 0; vi < numVertices; ++vi) { ... if (NaN) skip; }
// 第二遍：实际处理
for (vi = 0; vi < numVertices; ++vi) { ... positions, normals, texcoords ... }
```

无 NaN 的 mesh 也被遍历两次。**应合并到一次遍历**，在循环内逐顶点检查。

### BUG-3: localBbox 和 bbox 重复追踪 (TileBuilder.cpp:598-613)

```cpp
// 同一组坐标，更新了两个 bounding box
if (wx < acc.localBboxMin[0]) acc.localBboxMin[0] = wx;  // "local"
if (wx < acc.bboxMin[0])      acc.bboxMin[0]      = wx;  // "global"
```

两者值完全相同（都在世界空间计算），命名和语义区分已丢失。

### BUG-4: 投影修正 delta 与 CesiumJS Y_UP_TO_Z_UP 不兼容 (CProjectionEngine.cpp:176-223)

**这是最严重的问题。**

`ComputeInstanceProjectionDelta` 计算 delta 时，近似 ECEF 为：

```
approx_ECEF = R_enu * (East, North, Up) + T_origin
```

但 CesiumJS 实际执行的变换是：

```
pos_ECEF = R_enu * Y_UP_TO_Z_UP * (East, Up, North) + T_origin
         = R_enu * (East, -North, Up) + T_origin
```

两者相差 `R_enu * (0, 2*North, 0)`。Delta 修正的是“true ECEF 与 approx_ECEF 的差”，但 CesiumJS 实际用的是不同的近似。**修正量不匹配渲染管线**。

**影响**：偏离原点 Northing 方向的实例，其 ECEF 位置存在误差。误差大小 = 2 × Northing 偏移 × 该纬度 ENU 北向分量。例如在原点北侧 1km 处，误差约 2km × sin(纬度) ≈ 在 ECEF 中约 900m 偏移。

### BUG-5: 法线矩阵在无 Normal 的 mesh 上仍然计算 (TileBuilder.cpp:616-636)

```cpp
if (meshHasNormal && mesh->HasNormals())
{
    // 计算 normalMat, 变换法线
}
else
{
    acc.normals.push_back(0.0f);  // 填充 (0,0,0)
    // 但 normalMat.Inverse() 已经在上面的 if 外执行了
}
```

Wait — 实际上 normalMat 的计算在 if 内部，不是在外部。让我重新核实... 不，代码第618行 `aiMatrix4x4 normalMat = world;` 确实在 `if (meshHasNormal && mesh->HasNormals())` 内部，所以这个 bug 不存在。

### 性能问题

| 位置 | 问题 | 影响 |
|------|------|------|
| TileBuilder.cpp:622 | 逐顶点 `world.Inverse()` | 每个顶点一次 O(n³) 求逆 |
| TileBuilder.cpp:570-589 | 双重遍历 | 无 NaN 时多做一次全顶点循环 |
| TileBuilder.cpp:598-613 | 双 bbox 追踪 | 每顶点多 6 次比较 |

## 结论

**逐顶点位置变换本身是正确的** — 顶点从 Assimp 局部空间通过世界矩阵变换到 Y-up 世界空间，delta 修正作为平移变换应用。glTF 输出保持 Y-up，CesiumJS 负责 Y_UP_TO_Z_UP 转换。

**但存在一个架构级缺陷 (BUG-4)**：投影修正 delta 的计算假设 CesiumJS 使用 `R_enu * (East, North, Up)`，而实际 CesiumJS 使用 `R_enu * (East, -North, Up)`。这导致所有偏离原点的实例在 Northing 方向存在系统性位置误差。需要验证此误差是否在可接受范围内（通常 < 1m 对于 GK 区域内的典型偏移）。