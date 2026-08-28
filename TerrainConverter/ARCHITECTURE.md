# TIF → 地形瓦片（Quantized-Mesh）架构设计

> 基于 [CesiumGS/quantized-mesh](https://github.com/CesiumGS/quantized-mesh) 1.0 规范，结合 MGO 现有 TilesConverter / CProjectionEngine / meshoptimizer 模块制定。

---

## 1. Quantized-Mesh 规范要点

### 1.1 文件结构

```
┌─────────────────────────────┐
│ Header (88 bytes)           │
├─────────────────────────────┤
│ VertexData                  │
│  ├─ n (uint32)              │
│  ├─ u[n] (uint16, zigzag δ) │
│  ├─ v[n] (uint16, zigzag δ) │
│  └─ height[n] (uint16, zigzag δ) │  (quantized to [0,32767] against header min/max)
├─────────────────────────────┤
│ TriangleIndices (extension) │
│  ├─ n (uint32)              │
│  └─ indices[3n] (uint16 LE, HWM) │  (high-water-mark, no zigzag)
├─────────────────────────────┤
│ EdgeIndices                 │
│  ├─ nW/nS/nE/nN (uint32×4)  │
│  └─ indices (uint16 LE)     │  (plain indices, no HWM/zigzag)
├─────────────────────────────┤
│ Extensions (variable)       │
│  ├─ OctVertexNormals        │
│  ├─ WaterMask               │
│  └─ Metadata                │
└─────────────────────────────┘
```

### 1.2 Header 字段（88 字节）

| 字段 | 类型 | 字节 | 说明 |
|------|------|------|------|
| centerX/Y/Z | double×3 | 24 | 瓦片中心 ECEF 坐标 |
| minimumHeight | float | 4 | 瓦片内最低高程 |
| maximumHeight | float | 4 | 瓦片内最高高程 |
| boundingSphereCenterX/Y/Z | double×3 | 24 | 包围球中心 ECEF |
| boundingSphereRadius | double | 8 | 包围球半径（米） |
| horizonOcclusionPointX/Y/Z | double×3 | 24 | 地平遮挡点 ECEF |

### 1.3 顶点编码

- **u, v**：uint16 LE，zigzag δ 编码（Cesium 1.111 解码器通过 Uint16Array 读取，非 varint）
- **zigzag 编码**：`zigzag(n) = (n >> 31) ^ (n << 1)`（32-bit），截断低 16-bit 写入
- **height**：uint16 LE，zigzag δ 编码，量化至 [0, 32767] 对应 header 的 [minimumHeight, maximumHeight]
  （上游规范使用 float32 δ，但 Cesium 1.111 解码器实际通过 Uint16Array 读取）
- **u/v 必须单调递增**（Cesium 解码器依赖此性质；为满足 HWM 不变式，当前保留 meshopt 输出顺序）

### 1.4 三角形/边索引

- **High-water-mark 编码**：复用已出现顶点的索引，减小数值范围；`code = highest - idx`，`code == 0` 时 `highest = idx + 1`
- **三角形索引**：uint16 LE，HWM 编码（无 zigzag）。Cesium 1.111 解码器通过 Uint16Array 读取，实现为固定宽度
- **边索引**：四条边（W/S/E/N）的边界顶点列表，用于邻接瓦片缝合。编码为 plain uint16 LE（无 HWM，无 zigzag），同样匹配 Cesium 1.111 解码器行为
- **索引类型**：顶点数 < 65536 用 uint16，否则 uint32（当前实现仅支持 uint16）

### 1.5 扩展

| 扩展 ID | 名称 | 内容 |
|---------|------|------|
| 1 | OctVertexNormals | 每顶点法线，oct-encoded（2 字节/顶点） |
| 2 | WaterMask | 水域掩膜（1 字节/像素，uint8 网格） |
| 4 | Metadata | JSON 元数据 + 二进制 blob |

### 1.6 地形瓦片金字塔

- **投影**：EPSG:4326（geographic WGS84）
- **根级**：2 个瓦片（东/西半球，各 180°×180°）
- **层级 L**：X 方向 2×2^L 块，Y 方向 2^L 块
- **细分**：每个瓦片分裂为 4 子瓦片（NW/NE/SW/SE）
- **采样网格**：标准 65×65（或 64×64 + 边界共享）

### 1.7 layer.json 结构

```json
{
  "tilejsonVersion": "1.0.0",
  "format": "quantized-mesh-1.0",
  "version": "1.0.0",
  "extensions": ["octvertexnormals"],
  "tiles": ["{z}/{x}/{y}.terrain"],
  "available": [
    [{"startX":0,"endX":1,"startY":0,"endY":0}],
    [{"startX":0,"endX":3,"startY":0,"endY":1}]
  ],
  "bounds": [west, south, east, north],
  "projection": "EPSG:4326"
}
```

---

## 2. 现有项目资产评估

### 2.1 可复用模块

| 模块 | 路径 | 用途 |
|------|------|------|
| CProjectionEngine | `MeshProjectionErrorCorrector/CProjectionEngine.cpp` | GKInverse、GeographicToECEF、ENUToECEFRotation、ComputeRootTransform |
| GeodeticMath | `MeshProjectionErrorCorrector/GeodeticMath.h` | 纯数学函数：GKInverse、GeographicToECEF、ECEFToGeographic、ENUToECEFRotation |
| AxisMapper | `MeshProjectionErrorCorrector/AxisMapper.cpp` | Assimp↔ENU↔Tiles 轴转换 |
| meshoptimizer | `MeshGroupOptimizer/meshoptimizer/` | simplifier、quantization、indexcodec（已集成） |
| TilesConverter 模式 | `TilesConverter/` | Options + Convert() + main.cpp CLI 模式可照搬 |

### 2.2 缺失依赖

| 依赖 | 用途 | 推荐版本 |
|------|------|----------|
| **GDAL** | 读取 GeoTIFF 栅格数据及元数据（GeoKeys、tie points、pixel scale、投影） | 3.8+ |

> **采用 GDAL**：GDAL 提供完善的 GeoTIFF 读写支持，一行 `GDALGetGeoTransform()` 替代约 200 行手动标签解析，`GDALRasterIO()` 自动处理所有数据类型转换。开发效率和代码可维护性优先于库体积。

### 2.3 集成切入点

- 新建 `TerrainConverter/` 工程目录，与 `TilesConverter/` 并列
- 复用 `RouteAnalysisAdpter.sln` 添加新 vcxproj
- 共享 `MeshProjectionErrorCorrector.lib`（CProjectionEngine、GeodeticMath、AxisMapper）
- 共享 `MeshGroupOptimizer/meshoptimizer/` 源码（与 TilesConverter 相同的源码直编模式）

---

## 3. 模块架构

### 3.1 目录结构

```
TerrainConverter/
├── TerrainConverter.h          # 主入口：TerrainConverterOptions + Convert()
├── TerrainConverter.cpp        # 管线编排
├── GeoTiffReader.h             # GeoTIFF 读取封装
├── GeoTiffReader.cpp
├── HeightmapGrid.h             # 高程网格数据结构
├── HeightmapGrid.cpp
├── TerrainQuadtree.h           # 地理四叉树分块
├── TerrainQuadtree.cpp
├── TinSimplifier.h             # 高程网格 → TIN 简化
├── TinSimplifier.cpp
├── QuantizedMeshEncoder.h      # TIN → quantized-mesh 二进制
├── QuantizedMeshEncoder.cpp
├── HorizonOcclusionPoint.h     # 地平遮挡点计算
├── HorizonOcclusionPoint.cpp
├── TerrainLayerJson.h          # layer.json 生成
├── TerrainLayerJson.cpp
├── macro.h                     # TERRAIN_CONVERTER_API 导出宏
├── TerrainConverter.vcxproj
└── main.cpp                    # CLI 入口
```

### 3.2 类设计

#### 3.2.1 GeoTiffReader

```cpp
class TERRAIN_CONVERTER_API GeoTiffReader
{
public:
    bool Open(const std::string& path);
    void Close();

    // 读取高程数据为 float 网格
    bool ReadElevationGrid(std::vector<float>& heights,
                           int& width, int& height,
                           double& geoLeft, double& geoTop,
                           double& geoRight, double& geoBottom);

    // 获取 GeoKeys
    bool GetProjectionWKT(std::string& wkt);           // PROJCS[GEOGCS[...]]
    bool GetPixelIsPoint(bool& isPoint);                // GTRasterTypeGeoKey
    bool GetNoDataValue(float& value);                  // 无效像素值
    bool GetVerticalUnits(bool& isMeter);               // 垂直单位

private:
    GDALDatasetH m_dataset;
    int          m_width, m_height;
    int          m_epsg;                                 // 投影 EPSG
    std::string  m_wkt;
};
```

#### 3.2.2 HeightmapGrid

```cpp
struct HeightmapGrid
{
    std::vector<float> heights;   // 行主序，row*width + col
    int    width  = 0;
    int    height = 0;

    // 地理范围（投影坐标，通常 Gauss-Kruger easting/northing）
    double minEasting = 0, maxEasting = 0;
    double minNorthing = 0, maxNorthing = 0;

    // 采样间隔
    double dx = 0, dy = 0;

    // 无效值
    float  noDataValue = -9999.0f;
    bool   hasNoData   = false;

    // 投影信息（来自 GeoTIFF）
    std::string projectionWKT;
    int         epsg = 0;

    float HeightAt(int col, int row) const;
    bool IsValid(int col, int row) const;
    void BilinearSample(double easting, double northing, float& h) const;
};
```

#### 3.2.3 TerrainQuadtree

```cpp
struct TerrainTile
{
    int    level;                  // LOD 层级
    int    x, y;                   // 瓦片索引
    double west, south, east, north;  // 地理范围（度）
    double minEasting, maxEasting;
    double minNorthing, maxNorthing;

    // 子集高程数据（裁剪 + 重采样到本瓦片）
    std::vector<float> localHeights;
    int    localWidth = 0, localHeight = 0;

    bool   hasContent = false;
    std::vector<TerrainTile> children;  // 4 子瓦片，空则无
};

class TerrainQuadtree
{
public:
    // 从 HeightmapGrid 构建四叉树
    void Build(const HeightmapGrid& grid,
               const CProjectionEngine& proj,
               int maxLevel,
               int minSamplesPerTile = 65);

    // 遍历所有瓦片
    void ForEach(std::function<void(const TerrainTile&)> visitor) const;

    // 获取可用层级（用于 layer.json）
    void GetAvailableLevels(std::vector<std::vector<TileRange>>& out) const;

private:
    std::vector<TerrainTile> m_roots;  // 1 或 2 个根（视经度范围）
    int                      m_maxLevel;
};
```

#### 3.2.4 TinSimplifier

```cpp
struct TinMesh
{
    std::vector<float>    vertices;   // (x, y, z) × n, 行主序
    std::vector<unsigned> indices;    // 三角形索引
    float                 minError;
};

class TinSimplifier
{
public:
    // 从规则高程网格构建 TIN
    // 1. 生成规则网格三角形（2×(w-1)×(h-1) 个三角形）
    // 2. meshopt_simplifyWithAttributes（height 作为属性）
    // 3. 锁定四条边界的顶点（保证邻接瓦片缝合）
    TinMesh Simplify(const HeightmapGrid& localGrid,
                     double targetError,        // 简化误差（米）
                     float  normalWeight = 0.0f,
                     bool   lockBorder = true);

    // 可选：基于地形粗糙度的自适应简化
    TinMesh SimplifyAdaptive(const HeightmapGrid& localGrid,
                              double maxError,
                              int    minTriangles = 200,
                              int    maxTriangles = 5000);
};
```

#### 3.2.5 QuantizedMeshEncoder

```cpp
struct QuantizedMeshHeader
{
    double centerX, centerY, centerZ;
    float  minimumHeight, maximumHeight;
    double boundingSphereCenterX, Y, Z;
    double boundingSphereRadius;
    double horizonOcclusionPointX, Y, Z;
};

struct QuantizedMeshOptions
{
    bool writeOctVertexNormals = true;   // 写入顶点法线
    bool writeWaterMask        = false;  // 写入水掩膜
    bool writeMetadata         = false;  // 写入元数据
};

class QuantizedMeshEncoder
{
public:
    // 将 TinMesh 编码为 quantized-mesh 二进制
    bool Encode(const TinMesh& mesh,
                const TerrainTile& tile,
                const CProjectionEngine& proj,
                const QuantizedMeshOptions& opts,
                std::vector<uint8_t>& outBytes);

private:
    // —— 编码辅助 ——
    void EncodeHeader(const QuantizedMeshHeader&, std::vector<uint8_t>&);
    void EncodeVertices(const TinMesh&, const TerrainTile&,
                        const CProjectionEngine&,
                        std::vector<uint8_t>&);
    void EncodeTriangles(const std::vector<unsigned>& indices,
                         size_t vertexCount,
                         std::vector<uint8_t>&);
    void EncodeEdges(const TinMesh&, const TerrainTile&,
                     std::vector<uint8_t>&);
    void EncodeOctNormals(const TinMesh&, std::vector<uint8_t>&);

    // —— 几何计算 ——
    void ComputeBoundingSphere(const TinMesh&,
                               const CProjectionEngine&,
                               double& cx, double& cy, double& cz,
                               double& radius);
    void ComputeHorizonOcclusionPoint(const TinMesh&,
                                      const CProjectionEngine&,
                                      double& hx, double& hy, double& hz);
    void ComputeVertexNormals(const TinMesh&, std::vector<float>& outNormals);
    void OctEncodeNormal(float nx, float ny, float nz, uint8_t out[2]);
};
```

#### 3.2.6 TerrainLayerJson

```cpp
class TerrainLayerJson
{
public:
    void SetBounds(double west, double south, double east, double north);
    void SetProjection(const std::string& epsg);
    void AddAvailableLevel(int level, const std::vector<TileRange>& ranges);
    void AddExtension(const std::string& name);

    std::string Generate() const;
};
```

#### 3.2.7 TerrainConverter（主编排）

```cpp
struct TerrainConverterOptions
{
    std::string inputTif;
    std::string outputDir;
    std::string prjFile;            // 可选：覆盖 TIF 内嵌投影

    int   maxLODLevels      = 12;   // 最大 LOD 层级
    int   samplesPerTile    = 65;   // 每瓦片采样数
    double simplifyError    = 1.0;  // TIN 简化误差（米）
    float  normalWeight     = 0.1f; // 法线权重
    bool   lockBorder       = true;
    bool   adaptiveSimplify = true;

    bool   writeOctVertexNormals = true;
    bool   writeWaterMask        = false;

    double originEasting  = 0;     // 投影原点偏移（可省略，自动用 TIF 范围）
    double originNorthing = 0;
};

class TerrainConverter
{
public:
    bool Convert(const TerrainConverterOptions& opts);

private:
    bool ReadGeoTiff(HeightmapGrid& out);
    bool BuildQuadtree(const HeightmapGrid& grid);
    bool ProcessTile(const TerrainTile& tile);
    bool WriteLayerJson();

    GeoTiffReader         m_reader;
    TerrainQuadtree       m_quadtree;
    CProjectionEngine     m_proj;
    TinSimplifier         m_simplifier;
    QuantizedMeshEncoder  m_encoder;
    TerrainLayerJson      m_layerJson;
    TerrainConverterOptions m_opts;
};
```

---

## 4. 数据流与管线

### 4.1 主流程

```
┌──────────────┐
│  Input .tif  │
└──────┬───────┘
       │ GeoTiffReader::Open + ReadElevationGrid
       ▼
┌──────────────┐
│ HeightmapGrid│  (heights[], width, height, easting/northing bounds, WKT)
└──────┬───────┘
       │ CProjectionEngine::LoadProjection(WKT) + SetOrigin
       ▼
┌──────────────┐
│ Quadtree     │  按 Cesium 地形规范递归切分
│ Build        │  (root: 2 tiles, level L: 2×2^L × 2^L)
└──────┬───────┘
       │ for each TerrainTile
       ▼
┌──────────────┐
│ TinSimplifier│  规则网格 → TIN (meshopt_simplifyWithAttributes)
│              │  锁定四边界顶点（邻接缝合）
└──────┬───────┘
       ▼
┌──────────────┐
│ QuantizedMesh│  1. 顶点 (u, v, height) → zigzag/δ 编码
│ Encoder      │  2. 三角形索引 → HWM 编码
│              │  3. 边索引（W/S/E/N）
│              │  4. 计算 boundingSphere + horizonOcclusionPoint
│              │  5. 可选扩展：OctVertexNormals
└──────┬───────┘
       │ write {z}/{x}/{y}.terrain
       ▼
┌──────────────┐
│ layer.json   │  tiles URL template + available levels + bounds
└──────────────┘
```

### 4.2 坐标流转

```
TIF pixel (col, row)
  ↓ GeoKeys: tiePoint + pixelScale
投影坐标 (Easting, Northing)  [Gauss-Kruger / UTM]
  ↓ CProjectionEngine::ProjectedToGeographic (GKInverse)
地理坐标 (lat, lon)  [EPSG:4326]
  ↓ 按四叉树分块，每瓦片 (west, south, east, north) 确定
瓦片内插值采样 (u, v) ∈ [0, 32767]
  ↓ GeographicToECEF
ECEF (X, Y, Z)
  ↓ 用于 Header.centerX/Y/Z, boundingSphere, horizonOcclusionPoint
Quantized-Mesh 文件头
```

### 4.3 顶点编码细节

```cpp
// 1. 顶点排序：按 v 升序，v 相同按 u 升序（Cesium 解码器要求）
std::sort(vertices.begin(), vertices.end(),
          [](auto& a, auto& b) {
              return a.v != b.v ? a.v < b.v : a.u < b.u;
          });

// 2. u/v 量化到 [0, 32767]
uint16_t u_quant = static_cast<uint16_t>(std::round(
    (vertex.easting - tile.minEasting) / tile.widthMeters * 32767));
uint16_t v_quant = static_cast<uint16_t>(std::round(
    (vertex.northing - tile.minNorthing) / tile.heightMeters * 32767));

// 3. zigzag + δ 编码
auto zigzag = [](int16_t v) -> uint16_t {
    return static_cast<uint16_t>((v >> 15) ^ (v << 1));
};
int16_t prev_u = 0, prev_v = 0;
for (auto& vtx : vertices) {
    uint16_t du = zigzag(vtx.u_quant - prev_u);
    uint16_t dv = zigzag(vtx.v_quant - prev_v);
    outU.push_back(du);
    outV.push_back(dv);
    prev_u = vtx.u_quant;
    prev_v = vtx.v_quant;
}

// 4. height δ 编码（float32）
float prev_h = 0.0f;
for (auto& vtx : vertices) {
    float dh = vtx.height - prev_h;
    write_float32_le(outBytes, dh);
    prev_h = vtx.height;
}
```

### 4.4 三角形索引 HWM 编码

```cpp
// High-water-mark: 索引值不超过 (已见顶点数 - 1)
// 实际就是直接写入索引值，但利用单调性让后续值不超过 high water
// 解码时 high_water_mark = max(high_water_mark, index) + 1
// 编码端无需变换，直接写 uint16/uint32
// 但索引值必须保证 ≤ vertexCount-1（自然满足）
```

### 4.5 边索引计算

```cpp
// 四条边的顶点列表（用于邻接瓦片缝合）
// 每条边按顺时针顺序排列：
//   West:  v=0,  u 从 0 到 max
//   South: u=max, v 从 0 到 max
//   East:  v=max, u 从 max 到 0
//   North: u=0,   v 从 max 到 0
//
// 简化后边界顶点必须保留（lockBorder=true），因此可直接从
// 简化后的顶点列表中筛选出边界顶点
```

### 4.6 地平遮挡点（Horizon Occlusion Point）

```cpp
// 计算瓦片所有顶点 ECEF 在某个方向上的最大投影
// 若相机沿该方向看，所有顶点都在地平线以下 → 整瓦片被遮挡
//
// 算法：选取瓦片中心向地心方向的反方向作为参考轴
//       horizonOcclusionPoint = max(v · axis) for all vertices
//
// 简化实现（参考 Cesium 官方实现）：
// 1. 计算瓦片中心 ECEF（C）
// 2. 计算瓦片所有顶点 ECEF（V_i）
// 3. 对每个 V_i，计算相对 C 的向量 dV = V_i - C
// 4. 取 |dV| 最大的顶点方向作为遮挡点
// 5. horizonOcclusionPoint = V_max + normalized(dV_max) * |dV_max|
```

---

## 5. 四叉树分块策略

### 5.1 根级处理

```
TIF 经度范围与 Cesium 地形瓦片网格对齐：

  Case A: 跨东西半球（lon 跨越 0° 或 ±180°）
    → 生成 2 个根瓦片 (0,0,0) 和 (0,1,0)

  Case B: 仅在东半球或西半球内
    → 生成 1 个根瓦片，但从对应层级开始
    → 例：lon ∈ [100°, 105°]，对应 level 5 (2^5=32 块，每块 360°/32=11.25°)
       但实际需要从能容纳该范围的最小层级开始
```

### 5.2 自底向上构建

```
1. 从 TIF 高程网格出发，确定最大可分层级 L_max
   L_max = floor(log2(min(widthMeters, heightMeters) / targetTileMeters))
   targetTileMeters ≈ 5000（5km 一块，约 level 12-14）

2. 从 L_max 层开始，每个瓦片：
   a. 从 HeightmapGrid 中裁剪出本瓦片范围的采样点
   b. 双线性重采样到 samplesPerTile × samplesPerTile（如 65×65）
   c. TinSimplifier::Simplify()
   d. QuantizedMeshEncoder::Encode()
   e. 写入 {z}/{x}/{y}.terrain

3. 自底向上聚合父级：
   - 父级从 4 个子瓦片中各采样一部分，重采样到 65×65
   - 同样走 Simplify → Encode 流程
   - 直到根级

4. 记录每层的 available ranges（用于 layer.json）
```

### 5.3 边界对齐

- TIF 边界不一定与 Cesium 瓦片网格对齐
- 采用"最近 Cesium 瓦片边界对齐"策略：
  - 计算每个 Cesium 瓦片的 (west, south, east, north)
  - 与 TIF 实际范围求交
  - 无交集 → 跳过
  - 部分交集 → 在交集内重采样，外部用 noData 填充
- layer.json 的 `available` 只包含有内容的瓦片

---

## 6. TIN 简化策略

### 6.1 meshoptimizer 集成

```cpp
TinMesh TinSimplifier::Simplify(const HeightmapGrid& localGrid,
                                 double targetError,
                                 float  normalWeight,
                                 bool   lockBorder)
{
    // 1. 构建规则网格
    int W = localGrid.width, H = localGrid.height;
    std::vector<float> positions(W * H * 3);
    for (int r = 0; r < H; ++r) {
        for (int c = 0; c < W; ++c) {
            int idx = (r * W + c) * 3;
            positions[idx + 0] = (float)c / (W - 1);  // u ∈ [0,1]
            positions[idx + 1] = (float)r / (H - 1);  // v ∈ [0,1]
            positions[idx + 2] = localGrid.HeightAt(c, r);
        }
    }

    // 2. 构建三角形索引（每格 2 三角形）
    std::vector<unsigned> indices((W - 1) * (H - 1) * 6);
    size_t ii = 0;
    for (int r = 0; r < H - 1; ++r) {
        for (int c = 0; c < W - 1; ++c) {
            unsigned a = r * W + c;
            unsigned b = a + 1;
            unsigned d = a + W;
            unsigned e = d + 1;
            indices[ii++] = a; indices[ii++] = d; indices[ii++] = b;
            indices[ii++] = b; indices[ii++] = d; indices[ii++] = e;
        }
    }

    // 3. 高程作为顶点属性（用于误差度量）
    std::vector<float> heights(W * H);
    for (int i = 0; i < W * H; ++i) heights[i] = positions[i * 3 + 2];

    // 4. 锁定边界顶点
    std::vector<unsigned> lockIndices;
    if (lockBorder) {
        for (int c = 0; c < W; ++c) {
            lockIndices.push_back(c);              // top
            lockIndices.push_back((H-1) * W + c);  // bottom
        }
        for (int r = 1; r < H - 1; ++r) {
            lockIndices.push_back(r * W);          // left
            lockIndices.push_back(r * W + W - 1);  // right
        }
    }

    // 5. 调用 meshopt_simplifyWithAttributes
    std::vector<unsigned> simpIndices(indices.size());
    float simpError = 0.0f;
    size_t simpCount = meshopt_simplifyWithAttributes(
        simpIndices.data(), indices.data(), indices.size() / 3,
        positions.data(), W * H, sizeof(float) * 3,
        heights.data(), sizeof(float),
        normalWeight,
        lockIndices.data(), lockIndices.size(),
        (float)targetError,
        nullptr  // optional output error
    );
    simpIndices.resize(simpCount * 3);

    // 6. 顶点重映射（去除未引用顶点）
    std::vector<unsigned> remap(W * H);
    size_t newVertexCount = meshopt_generateVertexRemapMulti(
        remap.data(), simpIndices.data(), simpCount,
        ... // 多流
    );
    // ... 应用 remap

    return TinMesh{...};
}
```

### 6.2 自适应简化

- 平坦区域：目标三角形数少（500-1000）
- 陡峭区域：目标三角形数多（3000-5000）
- 基于 slope = |dh/dx| + |dh/dy| 的局部梯度估计
- 简化误差 `targetError` 按 slope 自适应：
  - slope < 5°  →  error = 5.0m
  - slope < 15° →  error = 2.0m
  - slope > 15° →  error = 0.5m

### 6.3 边界缝合保证

- **`lockBorder = true`** 是硬约束
- 边界顶点必须保留，且 (u, v) 量化值必须与邻接瓦片一致
- 量化解：
  - 同一边界上的顶点，其垂直于边的坐标必须相同
  - 例：西边所有顶点 u=0，北边所有顶点 v=max
  - 简化后边界顶点数可能不同，但邻接瓦片共享同一角点

---

## 7. CLI 设计

```
TerrainConverter - GeoTIFF to quantized-mesh terrain tile converter

Usage:
  TerrainConverter -i <input.tif> -o <outputDir> [options]

Options:
  -i <file>             Input GeoTIFF file
  -o <dir>              Output directory
  --prj <file>          Override projection (.prj WKT)
  --max-lod <N>         Max LOD level (default: 12)
  --samples <N>         Samples per tile (default: 65)
  --error <m>           Simplification error in meters (default: 1.0)
  --nweight <val>       Normal weight (default: 0.1)
  --no-lock-border      Disable border locking (NOT recommended)
  --adaptive            Enable adaptive simplification
  --no-normals          Skip OctVertexNormals extension
  --watermask           Enable WaterMask extension
  --origin <E,N>        Projection origin (default: auto from TIF)
  -h                    Show this help
```

---

## 8. 实现路线图

### Phase 1 — 基础管线（MVP）

| 步骤 | 内容 | 依赖 |
|------|------|------|
| 1.1 | 集成 GDAL 到 CMakeLists.txt | GDAL 3.8+ |
| 1.2 | 实现 GeoTiffReader（单波段浮点高程，GDAL RasterIO） | GDAL |
| 1.3 | 实现 HeightmapGrid 数据结构 + 双线性采样 | Eigen |
| 1.4 | 实现 QuantizedMeshEncoder（header + vertex + triangle） | CProjectionEngine, GeodeticMath |
| 1.5 | 实现 TerrainQuadtree（单根，固定层级） | CProjectionEngine |
| 1.6 | 实现 TinSimplifier（meshopt_simplifyWithAttributes） | meshoptimizer |
| 1.7 | 实现 TerrainLayerJson | - |
| 1.8 | TerrainConverter 编排 + main.cpp CLI | 上述全部 |
| 1.9 | 测试：单 TIF → 单层级地形瓦片 | M1 测试数据 |

### Phase 2 — 完整功能

| 步骤 | 内容 |
|------|------|
| 2.1 | 多层级 LOD（自底向上聚合 + 重采样） |
| 2.2 | EdgeIndices 编码（四边界顶点） |
| 2.3 | HorizonOcclusionPoint 计算 |
| 2.4 | OctVertexNormals 扩展 |
| 2.5 | 自适应简化（基于坡度） |
| 2.6 | 跨东西半球处理（双根瓦片） |
| 2.7 | noData 像素处理（边界裁剪） |

### Phase 3 — 优化与扩展

| 步骤 | 内容 |
|------|------|
| 3.1 | WaterMask 扩展（从栅格掩膜读取） |
| 3.2 | Metadata 扩展（每瓦片 metadata.json） |
| 3.3 | 多线程并行处理瓦片 |
| 3.4 | 大文件流式读取（分块 TIFF） |
| 3.5 | 进度报告 + 断点续传 |
| 3.6 | 集成到 MGOConsole（统一 CLI） |

---

## 9. 测试计划

### 9.1 单元测试

| 模块 | 测试用例 |
|------|---------|
| GeoTiffReader | 打开已知 TIF，校验 width/height/scale/tiepoint |
| HeightmapGrid | 双线性采样精度、noData 处理 |
| QuantizedMeshEncoder | 88 字节 header 字节序、zigzag 编码、δ 编码 |
| TinSimplifier | 边界锁定、目标误差、三角形数 |
| HorizonOcclusionPoint | 已知输入的数值验证 |
| TerrainLayerJson | JSON 结构校验 |

### 9.2 集成测试

- M1 测试数据（已知 GeoTIFF）→ 生成地形瓦片 → CesiumJS 加载验证
- 跨东西半球 TIF → 双根瓦片
- 高纬度 TIF（极地区域）→ 瓦片范围处理
- 大尺寸 TIF（>4GB）→ BigTIFF 支持

### 9.3 验证工具

- `quantized-mesh-tile` Python 包解码验证
- Cesium Sandcastle 在线加载验证
- 自制 hex dump 工具校验 header

---

## 10. 风险与缓解

| 风险 | 影响 | 缓解 |
|------|------|------|
| GDAL 跨平台编译 | 阻塞 Phase 1 | CMake find_package(GDAL) 统一处理；Linux apt / macOS brew / Windows vcpkg |
| 大尺寸 TIF 内存占用高 | OOM | 分块读取（TIFFReadScanline / TIFFReadTile） |
| 边界缝合错位 | 视觉裂缝 | lockBorder + 量化对齐；测试用例覆盖 |
| horizonOcclusionPoint 计算不准 | 渲染剔除异常 | 参考 Cesium 官方算法；与 cesium-native 实现对比 |
| 四叉树层级过深 | 性能下降 | maxLODLevels 限制 12-14；大区域分批处理 |
| PROJ 与 GeoTIFF 投影不一致 | 坐标偏移 | 优先用 GeoTIFF 内嵌 WKT；PRJ 文件覆盖选项 |

---

## 11. 依赖清单

### 11.1 新增第三方库

| 库 | 版本 | 许可证 | 用途 |
|----|------|--------|------|
| GDAL | 3.8+ | MIT/X | GeoTIFF 文件读取、元数据解析、数据类型转换 |

### 11.2 复用现有库

| 库 | 路径 | 用途 |
|----|------|------|
| Eigen | ThirdParty/include/Eigen | 矩阵运算 |
| PROJ | find_package(PROJ) — system/vcpkg | 投影变换 |
| meshoptimizer | MeshGroupOptimizer/meshoptimizer/ | TIN 简化 |
| Boost | ThirdParty/include/boost | regex、geometry（如需要） |

---

## 12. 与现有 TilesConverter 的对比

| 维度 | TilesConverter（3D Tiles） | TerrainConverter（Quantized-Mesh） |
|------|---------------------------|-----------------------------------|
| 输入 | Assimp 场景（FBX/OBJ） | GeoTIFF 高程栅格 |
| 输出 | b3dm + tileset.json | .terrain + layer.json |
| 分块 | 稀疏网格（八叉树） | 地理四叉树（Cesium 规范） |
| 几何 | 三角网模型 | 地形 TIN |
| 简化 | meshopt_simplify（可选） | meshopt_simplifyWithAttributes（强制，height 作为属性） |
| 投影 | 投影 CRS → ECEF | 投影 CRS → geographic → ECEF |
| 顶点编码 | glTF float32 | quantized-mesh uint16 + float32 δ |
| 索引编码 | glTF uint16/uint32 | HWM + uint16/uint32 |
| 扩展 | glTF extensions | octvertexnormals / watermask / metadata |
| LOD | 几何误差驱动 | 地理四叉树层级 |

---

## 13. 关键设计决策

1. **GDAL 3.8+ 替代 libtiff/libgeotiff** — `GDALGetGeoTransform()` 一行替代 200 行手动标签解析，`GDALRasterIO()` 自动类型转换，大幅简化代码
2. **meshoptimizer 而非自研 Delaunay** — 已有依赖、性能可靠、API 稳定
3. **lockBorder = true 默认** — 地形瓦片缝合是硬性要求，不能为简化而牺牲
4. **自底向上构建四叉树** — 从最高 LOD 开始，逐级聚合，保证父子一致性
5. **OctVertexNormals 默认开启** — CesiumJS 默认期望，光照渲染必需
6. **WaterMask 默认关闭** — 大多数场景不需要，按需启用
7. **不引入 Cesium terrain server** — 仅生成静态瓦片文件，由 CesiumJS 直接加载
8. **复用 CProjectionEngine** — 已验证的 GKInverse + ECEF 计算，避免重复造轮子

---

## 14. 后续扩展点

- **多波段 TIF 支持**：读取 RGB 波段生成 terrain overlay texture
- **矢量数据叠加**：道路、河流作为地形修饰
- **在线切片服务**：动态生成瓦片（替代静态文件）
- **3D Tiles 集成**：地形 + 建筑模型联合渲染
- **Draco 压缩**：替代 quantized-mesh 内置编码（需 Cesium 扩展支持）

---

**文档版本**：1.0  
**规范参考**：[CesiumGS/quantized-mesh](https://github.com/CesiumGS/quantized-mesh)  
**生成日期**：2026-07-05
