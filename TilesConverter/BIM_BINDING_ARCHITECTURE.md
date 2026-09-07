# TilesConverter BIM 属性绑定 — 架构设计

| | |
|---|---|
| 状态 | 设计稿 **v2.5**（v2.4 P0+P1 实现后经全量代码评审，v2.5 修正 19 项实现级缺陷并使代码与本文契约对齐——b3dm 绝对 8B 对齐/头长含填充、glTF %4 补齐、批表 DOUBLE/INT 类型映射与 NaN 净化、objectId 优先级链按 §4.8(1) 落地、保留列冲突去重上报、CSV RFC4180 引号/BOM、UTF-8 修复、sidecar fail-fast、去重哈希索引 O(1)。验证状态见 §7 实施状态注记） |
| 范围 | TilesConverter / TileBuilder / TileDataTypes / MGOConsole(tiles) |
| 关联消费者 | MGOServer（自带 CesiumJS 1.111 viewer，实测支持 BatchTable + EXT_structural_metadata 两套编码） |
| 先例文档 | `TerrainConverter/ARCHITECTURE.md` |

---

## 1. 背景与目标

MGO 的 TilesConverter 将 FBX/OBJ 等模型转换为 3D Tiles（b3dm + tileset.json），服务于数字孪生可视化。当前输出**只有几何 + 材质**：BIM 对象的语义信息（构件类型、编码、材料、楼层、业务台账属性）在转换过程中全部丢失，前端只能"看"不能"查"，无法支撑设备台账、巡检工单、构件级监测等数字孪生典型场景。

**目标**

1. 将 BIM 对象属性绑定到 3D Tiles 的每个构件（feature），CesiumJS 中可拾取（pick）查询；
2. 属性来源可插拔：模型内置元数据（FBX / IFC / glTF extras）+ 外部属性表（CSV/JSON sidecar）；
3. 不破坏现有输出与性能：功能关闭时输出与现状**逐字节一致**；
4. 保持模块边界与依赖方向不变（TileBuilder 不感知属性来源）；
5. 为 3D Tiles 1.1 元数据路径（`EXT_structural_metadata`）预留可切换的演进空间。

**非目标（本期）**：IFC 专业级解析（IfcOpenShell 级别的 Pset/Quantity 结构化）、属性编辑/回写、`3DTILES_batch_table_hierarchy` 父子结构扩展、预生成 Cesium styling。

---

## 2. TilesConverter 模块解读（现状）

### 2.1 模块定位与依赖

TilesConverter 是**编排层**：输入 Assimp 场景与选项，产出 3D Tiles；二进制格式构造全部委托 TileBuilder，投影/根变换委托 CProjectionEngine。

```
MGOConsole (CLI "mgo tiles")                    MGOConsole.cpp:586-667
   │  ReadFile(Triangulate|GenSmoothNormals|GenBoundingBoxes)   :662-663
   │  + TilesConverterOptions                                   TilesConverter.h:64-100
   ▼
TilesConverter (Convert, TilesConverter.cpp:410-602)
   ├──► TileBuilder                                             TileBuilder.cpp
   │      GlbBuilder::Build        glb 二进制 glTF              :125-504
   │      B3dmBuilder::Build       b3dm 封装                    :510-540
   │      MaterialGrouper          材质分组/合并                 :552-917
   │      BBoxUtils                包围盒                        :923-994
   │      TilesetWriter            写 tile + tileset.json       :1000-1356
   ├──► MeshProjectionErrorCorrector
   │      CProjectionEngine        投影修正/根变换
   │      TileDataTypes.h          MeshInstance/MergedMeshGroup/GridCell 共享类型
   └──► MeshGroupOptimizer         （可选）SimplifyScene         TilesConverter.cpp:479-489
```

依赖方向单一：`TilesConverter → TileBuilder → TileDataTypes`，无循环。**本设计必须保持该方向**。

### 2.2 转换管线全流程

```
aiScene (Assimp, Y-up)
 │
 │ 1. CollectMeshInstances()                                  TilesConverter.cpp:76-132
 │      深度遍历节点树，烘焙 worldTransform + 包围盒 + 顶点质心
 │      → std::vector<MeshInstance>
 │      ⚠ 只保留 meshIndex 与几何量，节点名/元数据不采集           ← 断点 ①
 │
 │ 2. 投影修正（per-instance 质心平移 或 per-vertex 就地改顶点）  TilesConverter.cpp:518-532
 │
 │ 3. BuildGridHierarchy()                                    TilesConverter.cpp:137-405
 │      稀疏网格自底向上：每个 instance 恰好落入一个 GridCell
 │      （0 级放不下 → 更粗级别；最终落 overflow 兜底 cell）
 │
 │ 4. 逐 cell 写 tile — TilesetWriter::WriteTiles              TileBuilder.cpp:1143-1208
 │      ├ MaterialGrouper::GroupCellByMaterial(cell, scene)    TileBuilder.cpp:643-917
 │      │    按 materialIndex 聚合 cell 内所有 instance 的顶点
 │      │    → std::vector<MergedMeshGroup>
 │      │    ⚠ 实例边界（= BIM 对象边界）在此被抹掉               ← 断点 ②
 │      ├ MergeGroupsByMaterial(groups)                        TileBuilder.cpp:552-641
 │      │    同基色 + 同贴图的组再合并（顶点数组拼接、索引重定基）
 │      ├ GlbBuilder::Build(groups) → glb                      TileBuilder.cpp:125-504
 │      │    每 group 一个 mesh/primitive，属性仅 POSITION/NORMAL/
 │      │    TEXCOORD_0 + indices                              ← 断点 ③（无 _BATCHID）
 │      └ B3dmBuilder::Build(glb) → b3dm                       TileBuilder.cpp:510-540
 │           FeatureTable 硬编码 {"BATCH_LENGTH":0}，           ← 断点 ④（BatchTable 恒空）
 │           btJsonLen = btBinLen = 0
 │
 │ 5. TilesetWriter::Generate → tileset.json（asset 1.1 + 外部子树 tileset）
 │                                                             TileBuilder.cpp:1210-1356
 ▼
输出：<out>/L{n}/tile_{cellKey}.b3dm ... + tileset.json
```

### 2.3 关键数据结构

| 结构 | 位置 | 字段要点 | 说明 |
|---|---|---|---|
| `MeshInstance` | TileDataTypes.h:22-41 | `meshIndex` `worldTransform[16]` `bboxMin/Max[3]` `vertexCentroid[3]` `vertexCount` | 一个节点×网格引用；**无任何身份字段** |
| `MergedMeshGroup` | TileDataTypes.h:46-74 | `materialIndex` `positions/normals/texcoords/indices` `baseColorFactor` `diffuseTexturePath` `texturePixels` | 材质聚合后的顶点容器；无特征 ID |
| `GridCell` | TileDataTypes.h:90-133 | `instances[]` `materialGroups[]` `cellKey` `tileFileName` `hasContent/isOverflow/isExternal` `children/parent` | 空间划分单元 = 一个 b3dm |
| `TilesConverterOptions` | TilesConverter.h:64-100 | 分块/LOD/投影/地理参考/简化 | CLI 直填 |
| `TileBuildOptions` | TileBuilder.h:37-52 | 上述选项的 TileBuilder 视图 | 由 `ToBuildOpts` 转换（TilesConverter.cpp:31-49） |

两个对本设计有利的关键性质：

- **实例不跨瓦片**：`BuildGridHierarchy` 自底向上把每个 instance 分配进恰好一个 cell（TilesConverter.cpp:262-298），溢出进 overflow cell（:300-322）。因此"每瓦片一张属性表"是天然边界，不存在同一对象的几何跨 b3dm 分裂。
- **合并是纯拼接**：`MergeGroupsByMaterial` 只做顶点数组拼接与索引重定基（TileBuilder.cpp:609-634），若给顶点附加一个**随顶点走的特征 ID 值**，合并时无需任何特殊处理即可存活。

### 2.4 结论：特征身份的四个断点

增加 BIM 属性绑定 = 打通 **"实例身份 → 顶点级特征 ID → 每瓦片属性表 → b3dm BatchTable"** 四层：

1. `CollectMeshInstances` 不采集节点名/网格名/元数据（MeshInstance 无字段）；
2. `GroupCellByMaterial` 按材质聚合顶点，实例边界被抹掉；
3. `GlbBuilder` 不输出 `_BATCHID` 顶点属性；
4. `B3dmBuilder` 不写 BatchTable（`BATCH_LENGTH:0`，bt 长度恒 0）。

---

## 3. 需求与方案分析

### 3.1 属性从哪里来

**来源 A — 模型内置元数据（零额外文件）**。Assimp v6.0.5（本项目 FetchContent 固定版本）已在导入时把主流 BIM 格式的属性填入 `aiScene`：

- **FBX**：`FBXConverter::SetupNodeMetadata`（assimp FBXConverter.cpp:926-976）把节点 PropertyTable 中**所有未被变换系统消费的属性**以类型化形式（bool/int/uint32/uint64/int64/float/string/vec3）挂到 `aiNode::mMetaData`，另固定写入 `UserProperties`（UDP3DSMAX）与 `IsNull`。Revit / 3ds Max 导出的 FBX 元素属性、构件编码等**直接可用**。场景级 `scene->mMetaData` 携带 UpAxis / UnitScaleFactor 等（FBXConverter.cpp:3697-3698）。
- **IFC**：节点命名 `IfcClassName_Name_GlobalId`（IFCLoader.cpp:654-655）；PropertySets 经 `ProcessMetadata` 转为字符串 metadata 挂节点（IFCLoader.cpp:671-678），且空间层级（IfcProject → IfcBuilding → IfcBuildingStorey → 构件）保留为节点树 → **父链继承可直接下传楼层/专业信息**。⚠ 两处已核实的精度损失：① 多个 Pset **合并存储且不带 set 名前缀**（跨 set 同名属性互相覆盖，IFCLoader.cpp:661-668）；② **数值也被字符串化**（IFCLoader.cpp:561-616）→ 类型恢复策略见 §4.8。
- **glTF2**：`extras` → `aiMetadata`。
- `aiNode::mMetaData` 定义于 assimp scene.h:132；`aiMetadata` 类型集见 include/assimp/metadata.h（AI_BOOL…AI_META_MAX）。

**FBX preservePivots 陷阱**：复杂变换的 Model 会被拆成变换链节点，节点名 `name + "$AssimpFbx$_" + 分量名`（`NameTransformationChainNode`，FBXConverter.cpp:743-745），**元数据与几何都挂在链尾节点**（FBXConverter.cpp:296-316）。因此采集身份时必须按 `$AssimpFbx$` 截断，还原原始对象名。

**来源 B — 外部属性表 sidecar**（业务系统 / Revit 明细表 / 运维台账导出）：CSV 或 JSON，一行一个对象。符合项目既有"外部配置驱动"习惯（`.prj`、`--cps` 控制点 CSV、`config_info.txt`）；MGOConsole 已有模板化 `CSVReader`（MGOConsole/CSVReader.h，Boost tokenizer 转义解析）；TileBuilder 以 PUBLIC 链接 `nlohmann_json`（TileBuilder/CMakeLists.txt），TilesConverter 传递可用 → JSON 解析零新增依赖。

**来源 C — 未来插件**：IfcOpenShell（Pset/Quantity 结构化）、RVM/IML 等 → 通过 `IPropertySource` 扩展点接入，不进入本期。

### 3.2 Feature 定义与 ID 分层（以什么作为 id 绑定）

**Feature = `MeshInstance` = aiNode 对 aiMesh 的一次引用**（实例粒度而非网格粒度：同一门窗 mesh 被 100 个节点引用 → 100 个 Feature；一个节点引用多 mesh 的多材质构件 → 多个实例共享同一节点属性行，哈希去重后同 batchId，拾取语义仍为一个整体对象）。

"ID"必须拆成三层，混淆这三层是本领域最常见的错误来源：

| 层 | 是什么 | 生命周期 | 用途与规则 |
|---|---|---|---|
| **L1 绑定载体**（join 结构） | **aiNode 场景图引用本身**（node×mesh 实例） | 单次转换内 | 模型内置元数据（FBX `SetupNodeMetadata`、IFC Psets、glTF2 `ParseExtras`）都挂在 aiNode 上（已核验）→ **完全不需要按名字匹配**，`CollectMeshInstances` 遍历时按引用取 `node->mMetaData`，天然精确、重名免疫 |
| **L2 匹配键**（仅外部属性表需要） | 优先级：① **内嵌业务 GUID**（IFC：节点名尾段 `IfcClass_Name_<GlobalId>` 切分；FBX：metadata 指定键，`--bim-id-property`）→ ② `objectName`（剥 `$AssimpFbx$` 链后缀的节点名）→ ③ `meshName` 兜底；P2 增强：节点路径（`楼层/墙001`）消歧 | 需跨转换稳定 | sidecar 行与实例的连接方式。两个已核实的不稳定因素见下 |
| **L3 运行时 ID** | `batchId`（0..BATCH_LENGTH-1，即 `_BATCHID` / `Cesium3DTileFeature.featureId`） | **仅单个 b3dm 瓦片内有效** | 拾取索引；**禁止当业务 ID 用**——重切分块后编号即变 |
| **L3′ 业务主键** | 属性表保留列 `objectId`（GlobalId/ElementId/资产编号，强制 string） | 全生命周期稳定 | 前端接台账/工单/REST 的唯一正确键（表规格见 §4.8） |

L2 可靠性的两个已核实行为（assimp v6.0.5 源码实测）：

- **FBX 重名节点会被自动去重**：`FBXConverter::GetUniqueName`（FBXConverter.cpp:2228-2242）对同名节点追加 3 位零填充数字（第 2 个 `Wall` → `Wall001`…；空名回退祖先名 `MakeUniqueNodeName`）。→ sidecar 若按原始建模软件名匹配会落空；转换器必须**导出最终 objectName 清单**（日志/`--bim-report`），或优先走 L2-①。
- **glTF2 `extras` 确认落到 `aiNode::mMetaData`**（glTF2Importer.cpp:1130-1155 `ParseExtras`），L1 路径在 FBX/IFC/glTF2 三格式均成立。

IFC 特例：GlobalId 已内嵌在 assimp 生成的节点名尾段 → **IFC 模型即使无 Pset 需求，也能零配置获得精确的 L2-① 匹配键**。

### 3.2.1 objectId 按来源格式的可得性矩阵（v2.2 补；v2.3 起为本设计 `IBindingStrategy` 策略层的行为规范——运行时经 `--bim-formats` 打印同一矩阵，防文档漂移）

objectId 不是格式天然给的，而是"**格式自带 ID × assimp 暴露程度 × 导出烘焙策略**"的交集（三条均已对 assimp v6.0.5 源码核实）：

| 格式 | 本管线可读 | 格式自带稳定 ID | assimp 暴露情况 | objectId 实际来源（按优先级） |
|---|---|---|---|---|
| **FBX** | ✔ | 文件内部有对象 UniqueId（64 位） | **不暴露**：`SetupNodeMetadata` 只写 UserProperties/IsNull/未消费属性（FBXConverter.cpp:926-976 全函数已核，无 UniqueId） | ① 导出器烘焙**用户自定义属性**（Properties70 'U' 项 → 类型化进 `aiNode::mMetaData` → `--bim-id-property` 命中）② 烘焙进节点名（`Wall_1836242`，sidecar 按名 join）③ 兜底节点名（重导出不稳定，仅对账用） |
| **3DS** | ✔ | **无 ID 概念**（1990s DOS 格式） | **零 metadata**（code/AssetLib/3DS/ 无任何 aiMetadata 代码）；节点名可为占位 `$$$DUMMY`（真名在动画轨道 chunk，还原不可靠） | 只能**外部 sidecar 按名 join**（`--bim-key mesh` 通常优于节点名）；或上游转 FBX/glTF 时补烘焙 ID |
| **RVT** | ✘ **assimp 无 Revit 导入器**（AssetLib 清单已核） | Revit 内部有 UniqueId(GUID)/ElementId(int)，但封在 RVT 内 | 读不到 | **必须先导出**，ID 由导出通道决定：→ **IFC（推荐）**：IfcGloballyUniqueId 进节点名尾段，本管线零配置直取；→ FBX/glTF：需定制导出器烘焙 UniqueId/ElementId 进 UDP/名称；→ Revit API 导 sidecar 明细表（配任意保名几何格式） |
| IFC | ✔ | IfcGloballyUniqueId（构件级 GUID） | 节点名尾段直取（§3.1） | 零配置：名字切分尾段 |
| glTF2 | ✔ | 无强制 ID（extras 自定义） | extras → `aiNode::mMetaData`（glTF2Importer.cpp:1130-1155 已核） | 导出方约定 extras 键名 |

选键建议：Revit 侧长期主键用 **UniqueId（GUID）**——跨版本升级、工作集、拆分合并都稳定；**ElementId（int）** 仅同一文档内稳定（Revit UI"按 ID 选择"所用），适合同档台账 join，不宜作长期主键。Revit 官方 FBX 导出链路（含 3ds Max 中转）**默认不写稳定 ID**——RVT 源模型优先走 IFC 导出路线。

### 3.3 输出承载格式对比

| 维度 | A. b3dm + BatchTable（3D Tiles 1.0 经典） | B. glb + `EXT_mesh_features` / `EXT_structural_metadata`（3D Tiles 1.1） |
|---|---|---|
| CesiumJS 支持 | 全版本；`scene.pick` → `Cesium3DTileFeature.getProperty` | ≥1.92（3D Tiles Next）；**已实测** MGOServer 自带 Cesium.js 1.111 含 `EXT_structural_metadata`/`EXT_mesh_features` 解析代码 ✔ |
| 对现有代码的改动面 | 小：GlbBuilder/B3dmBuilder 增量扩展 | 大：需新扩展编码器、per-primitive featureIds、schema/propertyTables |
| 生态工具链 | 3d-tiles-tools / Cesium 完整支持 | 较新，逐步完善 |
| 长期演进 | **b3dm/Feature Table/Batch Table 已被 3D Tiles 1.1 标注 deprecated**（规范提供 b3dm→glb 迁移指南；CesiumJS 持续支持读取） | 官方指定演进方向（迁移指南明确用 `EXT_mesh_features`+`EXT_structural_metadata` 承接 Batch ID/Batch Table） |
| 属性存储位置 | BatchTable 自包含于每个 b3dm | propertyTables 在 glb 内（跨瓦片共享需另行设计） |

**决策**：阶段一以 **方案 A 为主路径**（改动小、兼容性最好、即刻服务 MGOServer 1.111）；阶段二在同一中间表示之上增加 **方案 B 编码器**，`--bim-encoding` 可切换。两者共享 Schema 与行数据，编码器互不影响。

### 3.4 关键设计决策

| # | 决策 | 理由 |
|---|---|---|
| D1 | 特征 ID 作为**顶点属性**（`_BATCHID`）随顶点走 | 材质合并（MergeGroupsByMaterial）是纯拼接，ID 值不依赖顶点序号，天然存活；不必放弃现有合并优化 |
| D2 | 属性行按**整行内容哈希去重**（瓦片内，含保留列），行相同则共享 batchId | 去重键 = `hash(objectId+objectName+objectClass+用户列)`。⚠ 注意（v2.1 修正）：身份列几乎总是逐对象唯一 → **object 粒度下去重实际退化为"按对象一行"**（这正是拾取/高亮语义所需的正确行为）；"同型号 100 樞门 1 行"只在 `--bim-feature-granularity type`（身份列不入行、去重生效，拾取=选中间型全部实例）时成立，见 §4.8(2) |
| D3 | 功能关闭时输出**逐字节一致**（完全旁路） | 现有回归（test_emptyprimitive / test_overflow404、 golden fixture）零影响 |
| D4 | Schema 自动推断 + 可选显式覆盖；数值列走二进制 BatchTable 段、字符串列走 JSON | 兼顾体积与实现简单；b3dm BatchTable 规范原生支持两种形式 |
| D5 | 属性源 `IPropertySource` 插件化；TileBuilder 只消费中间表示 | 依赖方向不变，TileBuilder 不感知 CSV/FBX/IFC |
| D6 | 匹配失败的实例落入共享"空行"（row 0） | 保证同一 primitive 内 `_BATCHID` 连续合法，避免混合有/无批次的 primitive |

### 3.5 业界方案调研（SuperMap / CesiumLab / GISBox，2026-09 检索）

| 维度 | SuperMap（S3M 生态） | CesiumLab 4 / CIMRTS | GISBox |
|---|---|---|---|
| 目标格式 | **S3M**（自有规范，开源 s3m-spec；非 3D Tiles） | 标准 3D Tiles（OGC） | 标准 3D Tiles |
| RVT 路径 | Revit 导出插件 → 模型数据集 → iDesktop 生成缓存 | **导出插件 → CLM 中间格式** →"CLM&IFC模型切片"；CIMRTS 支持 RVT/DGN 直接入库 | **RVT 直接切片**（内置解析，无需 Revit）；IFC 直接切片 |
| 属性承载 | **几何/属性分离**：瓦片旁挂属性文件，三模式可选——**S3MD**（JSON 行式："快速提取单对象全字段"）、**ATTRIBUTE**（二进制**列式**："快速提取多对象指定字段"）、**DB**（大文件，仅 S3M 3.01，配 iServer SQL 查询） | BIM 切片"**保留完整构件属性**"并"**生成完整构件结构树**"；手工模型（FBX/OBJ）走"**属性外挂**"（sidecar）；CIMRTS 支持"属性字段自定义、要素类/属性结构自定义、属性批量导入导出、3dtiles 内置属性完全自定义" | **不写入属性**（官方 FAQ 三连确认："还没做属性的写入，暂时无法获取"/"目前还不支持"/BIM 构件绑数据"不支持"，2025-10 截图）；"3DTiles再处理"仅做几何再处理（重建顶层/顶点压缩），非属性再绑定 |
| 绑定/查询机制 | 拾取得模型对象 ID → `layer.getAttributesById(ID)`；前端可 `indexedDBSetting.isAttributesSave=true` 存 IndexedDB；或走 iServer 服务端属性表查询 | 属性随 3D Tiles 内置（批表）；构件树另存（scenetree.json，GISBox 文档侧证） | ——（无属性可绑定） |

**对本设计的四点印证与两点借鉴**：

1. **双编码路线同构**：SuperMap S3MD（行式 JSON，利于"单对象全字段"）与 ATTRIBUTE（列式二进制，利于"多对象单字段"）并存，与本设计 P1 批表 JSON（行式）→ P2 `EXT_structural_metadata` 列存 propertyTable 的分野**完全同构**——行式/列式按查询模式分化是行业共识，不是过度设计；
2. **属性外挂被验证**：CesiumLab 手工模型切片的"属性外挂"= 本设计 `--bim-props` sidecar 的行业先例；
3. **属性与几何分离存储**是重属性场景（Revit 数百参数）的成熟路线——印证 §4.8 "瘦瓦片 + 胖台账"默认立场（SuperMap 干脆把属性全放瓦片外）；
4. **GISBox 反例证明空白**：能直读 RVT 却不绑属性，用户 FAQ 高频追问——"直读格式"与"属性绑定"是两个独立能力，本设计补的正是后者的空白。

**借鉴（P3 候选，不改变当前范围）**：

- **构件结构树输出**（CesiumLab 核心卖点之一）：转换期把树形结构（节点层级 + objectId）导出为独立 JSON，支撑前端树面板/批量选择/楼层筛选——与瓦片内属性绑定正交，可后置；MGOServer 面板设计时预留；
- **属性再处理工具**（GISBox "3DTiles再处理"的产品形态）：对已切好的瓦片离线重写批表（不动几何）——依赖 D2 行去重映射的持久化，本设计不承诺，记录为潜在工具方向。

#### 3.5.1 分格式的绑定机制矩阵（核心调研结论，2026-09 追加）

**问题本质**：绑定 = "身份（ID）从哪来 × 属性从哪来"。行业的绑定机制只有四种模式：

| 模式 | 机制 | 典型代表 | 损耗 |
|---|---|---|---|
| ① 插件直采 | 在宿主软件进程内（Revit API）采集身份+属性 | SuperMap Revit 导出插件、CesiumLab CLM 导出插件、BimAngle Engine | **零损耗**，但绑定宿主软件 |
| ② 中间格式 | 自有 schema 承载身份+属性（几何/属性/结构树内聚） | **CLM**（SQLite 单文件 6 表；Models 表：`id`（构件节点必须与构件 id 一致）、`name`、`type`、`props` JSON 附加属性）；SuperMap UDB 模型数据集 + 属性表（SmID） | 取决于导出端 |
| ③ 外挂关联 | 事后按 join 键挂属性表 | CesiumLab"属性外挂"：**CSV UTF-8，第一列=唯一名称，与场景名称匹配**；SuperMap 外挂属性表 | 依赖名称质量 |
| ④ 格式原生 | 文件自带身份/属性，直读即得 | IFC GlobalId+Pset、glTF extras/metadata、FBX SDK UniqueId | 零（但依赖导入器暴露程度） |

**分格式 × 分厂商对照**（均已核实，来源见 §9/§10）：

| 格式 | CesiumLab | SuperMap | GISBox | BimAngle | 本设计（MGO） |
|---|---|---|---|---|---|
| **RVT** | 插件→CLM（**id=Revit 构件 uuid**） | ① Revit 导出插件→模型数据集；② "打开 BIM 数据"——**需本机安装 Revit**，可选导出 Revit 明细表为属性表 | **直读（无 Revit）但丢全部属性** | 插件直采 | **不直读**（§3.2.1：走 Revit→IFC/FBX 导出） |
| **IFC** | 直切，GlobalId 身份 | 导入模型数据集 | 直读**丢属性** | — | L1+L2①（名字尾段 GlobalId 零配置） |
| **FBX** | name=节点名；**id=FBX SDK UniqueId 的 MD5**（官方 SDK 能取内部 UniqueId） | 导入数据集 + 外挂表 | 直读**丢属性** | — | L1（UDP→mMetaData）+L2② |
| **OBJ** | name=节点名；**id=随机生成** | 外挂表 | 直读**丢属性** | — | L2②③（名+sidecar） |
| **glTF** | — | — | — | — | L1（extras→mMetaData） |

**四点结论（对本设计的意义）**：

1. **RVT 是全行业共同痛点**：四家厂商四种解法（插件直采 / CLM 中间格式 / 本机 Revit 引擎 / 逆向直读），除 GISBox 丢属性外全部要求"Revit 在场"——**印证 §3.2.1 的 Revit→IFC 导出路线是免装 Revit 场景下的唯一理性选择**；
2. **工具链决定身份可见性**：CesiumLab 用商业 FBX SDK 能取内部 UniqueId → MD5 稳定 id；我们用 assimp 拿不到（§3.2.1 已核）→ 只能靠 UDP 烘焙/名称——这不是设计缺陷而是开源工具链的客观边界，文档如实承认即可；且 FBX UniqueId 是**文件内稳定**而非**跨导出稳定**（Revit 重导出 FBX 会重新分配），其定位与我们 L3′ objectId 的"长期主键"不同；
3. **"默认 id+name 双字段"与保留列同构**：CesiumLab 为每个对象默认生成 id/name 两个基础属性（OBJ 随机 id / FBX MD5 id），与本设计 objectId/objectName 保留列设计互为印证——行业已收敛于此模式；
4. **OBJ 的"随机 id"警示**：身份缺失时厂商选择随机生成（保证唯一性、牺牲稳定性）——本设计反其道（保留 objectName 真值 + 共享空行），因为随机 id 会骗过消费端"看起来有主键"，而我们的空行+报告模式更显式。

---

## 4. 目标架构

### 4.1 组件总览（新增 / 修改清单）

| 组件 | 位置 | 类型 | 职责 |
|---|---|---|---|
| `BimFormatStrategy.h/.cpp` | TilesConverter/ | **新增（核心抽象层）** | `IBindingStrategy` 抽象 + **每格式一个策略**：`IfcBindingStrategy` / `FbxBindingStrategy` / `GltfBindingStrategy` / `ObjBindingStrategy` / `ThreeDsBindingStrategy` / `GenericBindingStrategy`（兜底）；各自封装身份解析、场景属性行采集、继承规则、**能力自描述 `Describe()`**；`BindingStrategyRegistry` 按格式选择（§3.5.1 矩阵 = 策略层行为规范） |
| `BimBindingPipeline.h/.cpp` | TilesConverter/ | **新增** | 绑定管线：策略选行 → sidecar join → 合并 → 保留列注入 → **报告记录**（每对象的 idSource/matchResult 落盘，透明可查） |
| `BimPropertySource.h/.cpp` | TilesConverter/ | **新增** | `IPropertySource`（外部属性轴：`SidecarTableSource` CSV/JSON 匹配；`PropertySchema` 推断） |
| `MeshInstance` 扩展 | TileDataTypes.h | 修改 | `objectName` `meshName` `properties` |
| `MergedMeshGroup` 扩展 | TileDataTypes.h | 修改 | `batchIds`（per-vertex） |
| `GridCell` 扩展 | TileDataTypes.h | 修改 | `featureBatch.rows` |
| `GlbBuilder::Build` | TileBuilder.cpp | 修改 | 输出 `_BATCHID` accessor |
| `B3dmBuilder::Build` 重载 | TileBuilder.cpp | 修改 | 写 BATCH_LENGTH + BatchTable |
| `BatchTableWriter` | TileBuilder/（并入 TileBuilder.cpp 或独立文件） | **新增** | Schema + rows → BatchTable JSON + 二进制段 |
| `GroupCellByMaterial` | TileBuilder.cpp | 修改 | per-instance batchId 分配 |
| `TilesConverter::Convert` | TilesConverter.cpp | 修改 | 策略选择、绑定管线装配、Schema 推断 |
| `TilesConverterOptions` | TilesConverter.h | 修改 | `bim*` 选项组 |
| MGOConsole `tiles` | MGOConsole.cpp | 修改 | `--bim-*` 参数（含 `--bim-formats` 能力清单 / `--bim-report` 绑定清单） |
| `ThreeDNextEncoder` | TileBuilder/ | 阶段二新增 | glb + EXT_mesh_features / EXT_structural_metadata |

依赖方向保持：`TilesConverter → TileBuilder → TileDataTypes`；**策略层与绑定管线仅被 TilesConverter 依赖**（不泄漏进 TileBuilder）。

### 4.2 中间数据结构

新增类型放 **TileDataTypes.h**（模块共享层，避免 TileBuilder 反向依赖 TilesConverter）。约定：所有字符串 **UTF-8**（CLI 路径沿用 `gbk_to_utf8`；sidecar 文件内容约定 UTF-8；FBX 元数据按原始字节读入，写入前做 UTF-8 校验/转换——见 §8 R1）。

```cpp
// ===== TileDataTypes.h（扩展） =====

// 单个属性值（与 assimp aiMetadata 类型一一对应；编码两端均有官方落点）
struct BimValue {
    // 映射：AI_BOOL→Bool, AI_INT32→Int32, AI_UINT32/AI_INT64/AI_UINT64→Int64,
    //       AI_FLOAT/AI_DOUBLE→Double, AI_AISTRING→String, AI_AIVECTOR3D→Vec3,
    //       AI_AIMETADATA（嵌套字典）→ 展平为前缀键（Parent.Child），避免信息丢失
    enum class Type : uint8_t { Bool, Int32, Int64, Double, Vec3, String };
    Type      type = Type::String;
    bool      b = false;
    int64_t   i = 0;      // Int32/Int64 共用
    double    d = 0.0;
    double    v[3] = {};  // Vec3 → Batch Table 二进制 VEC3/FLOAT；3dnext 走 VEC3 属性
    std::string s;        // UTF-8
};

// 一条特征属性行（一个 BIM 对象）：有序键值对（顺序 = aiMetadata 采集顺序 / sidecar 列序）
using BimPropertyRow = std::vector<std::pair<std::string, BimValue>>;

// 属性 Schema（整个 tileset 统一、字段顺序稳定 → 输出可复现）
struct PropertyFieldSpec { std::string name; BimValue::Type type; };
struct PropertySchema { std::vector<PropertyFieldSpec> fields; };

// 每瓦片批表：batchId → 属性行（行内容已按哈希去重、跨实例共享）
struct FeatureBatchTable {
    std::vector<std::shared_ptr<const BimPropertyRow>> rows;
    bool IsEmpty() const;
};

struct MeshInstance {
    // ...现有字段不动...
    // ---- BIM 特征身份（属性绑定开启时填充）----
    const aiNode* node = nullptr;   // 归属节点（Convert 期间 aiScene 存活，指针安全；1a 阶段解析用，不序列化）
    std::string objectName;    // "$AssimpFbx$" 截断后的节点名
    std::string meshName;      // aiMesh::mName
    std::shared_ptr<const BimPropertyRow> properties;  // 内容去重后共享；nullptr = 无属性
};

struct MergedMeshGroup {
    // ...现有字段不动...
    std::vector<uint32_t> batchIds;   // per-vertex，与 positions 平行；空 = 未启用绑定
};

struct GridCell {
    // ...现有字段不动...
    FeatureBatchTable featureBatch;   // 本 tile 的 BatchTable 内容
};
```

```cpp
// ===== TilesConverter/BimPropertySource.h（新增） =====

// 属性源扩展点：CollectMeshInstances 遍历完成后逐实例调用
// ===== 抽象层：每格式一个绑定策略（v2.3）=====
// 设计原则：分格式的差异（身份解析/元数据解释/继承规则）收敛到一个策略接口后面，
//          管线其余部分格式无关；行为规范 = §3.2.1/§3.5.1 矩阵（每策略一一对应）。
class IBindingStrategy {
public:
    // —— 能力自描述（单一事实源：--bim-formats 清单由它打印，防文档与二进制漂移）——
    struct Capability {
        const char* format;        // "IFC" / "FBX" / "glTF2" / "OBJ" / "3DS" / "…"
        bool   nativeGuid;         // 格式是否自带稳定 GUID（矩阵"格式自带稳定 ID"列）
        const char* idSource;      // objectId 解析路径（人读，进 --bim-formats 与报告）
        bool   sceneMetadata;      // 是否有场景内属性（mMetaData）
        const char* caveats;       // 已知限制（IFC 字符串化 / FBX 重名后缀 / $$$DUMMY…）
    };
    virtual Capability Describe() const = 0;

    // —— 身份解析：返回 objectId 值 + 来源标签（供报告透明化）——
    struct IdResolution {
        enum class Source { NameTail, MetadataKey, SidecarColumn, ObjectName, None };
        std::string objectId;                 // 空 = 未解析到
        Source      source = Source::None;
    };
    virtual IdResolution ResolveObjectId(const MeshInstance& inst) const = 0;

    // —— join 键序列（sidecar 匹配用；L2 优先级由策略给出，不散落在管线里）——
    virtual std::vector<std::string> JoinKeys(const MeshInstance& inst) const = 0;

    // —— 场景属性行采集（含继承规则；OBJ/3DS 等无属性格式返回 nullptr）——
    virtual std::shared_ptr<const BimPropertyRow> CollectSceneRow(const MeshInstance& inst) const = 0;
};

// 每格式一个实现（§3.2.1 矩阵的代码化）：
class IfcBindingStrategy     : public IBindingStrategy { /* 名字尾段 GlobalId；objectClass=名字首段；Pset 全量+字符串化警示 */ };
class FbxBindingStrategy     : public IBindingStrategy { /* $AssimpFbx$ 剥离；UDP 键；无原生 GUID 需烘焙 */ };
class GltfBindingStrategy    : public IBindingStrategy { /* extras 键约定 */ };
class ObjBindingStrategy     : public IBindingStrategy { /* 仅名称；无 GUID 无 metadata */ };
class ThreeDsBindingStrategy : public IBindingStrategy { /* 同 OBJ + $$$DUMMY 警示 */ };
class GenericBindingStrategy: public IBindingStrategy { /* 兜底：通用 mMetaData + 名称；未识别格式带警告 */ };

// 注册表：Convert 按 assimp 检测的 importerId 选择；未注册 → Generic + Warning
class BindingStrategyRegistry {
public:
    static const IBindingStrategy& For(const std::string& assimpFormatId);
    static std::vector<const IBindingStrategy*> All();   // --bim-formats 遍历打印
};

// ===== 绑定管线（BimBindingPipeline）：策略 × 外部源的正交组合 =====
// 外部属性轴仍是 IPropertySource（sidecar：CSV 现实现，JSON/DB 将来——与格式策略无关）
class SidecarTableSource : public IPropertySource { /* 按策略给出的 JoinKeys 匹配 */ };

class BimBindingPipeline {
public:
    // 每实例产出：属性行 + 透明化记录（落盘 --bim-report）
    struct BindingResult {
        std::shared_ptr<const BimPropertyRow> row;  // 合并行（场景→sidecar，含保留列注入）
        // —— 透明可查三件套 ——
        std::string objectId;     // 最终 objectId（空 = 无）
        const char* idSource;     // "name-tail"/"metadata"/"sidecar"/"objectName"/"none"
        const char* matchSource;  // "scene"/"sidecar"/"merged"/"none"（属性行从哪来）
    };
    std::vector<BindingResult> BindAll(std::vector<MeshInstance>& instances);
};
```

```cpp
// ===== TilesConverter.h（选项组扩展） =====
struct TilesConverterOptions {
    // ...现有字段不动...

    // ---- BIM 属性绑定 ----
    bool        bimBind = false;                  // 总开关（--bim-props 或 --bim-metadata 触发）
    bool        bimCollectSceneMetadata = true;   // 采集 aiNode::mMetaData（FBX/IFC/glTF extras）
    std::string bimPropsFile;                     // 外部属性表 CSV/JSON（UTF-8）
    std::string bimKeyMode = "auto";              // auto | node | mesh —— sidecar 匹配键
    std::string bimIdColumn;                      // sidecar 匹配列；默认 auto（"GlobalId"/"ObjectId"/首列）
    std::string bimIdPropertyKeys = "GlobalId,ElementId,ifcGUID";  // objectId 候选键（依序找于 metadata/sidecar，见 §4.8 保留列；裸 "ID" 不入默认，防误命中无关属性列）
    std::string bimFields;                        // 属性输出白名单（逗号列表或文件）；空 = 全列
    std::string bimFeatureGranularity = "object"; // object | type —— 去重粒度（type：身份列不入行，同型构件共享 feature）
    std::string bimReportFile;                    // 绑定清单 JSON 导出（每对象 idSource/matchSource，§4.5）
    std::string bimStrategy;                      // 强制策略名（默认按 assimp importerId 自动选；排障用）
    bool        bimListFormats = false;            // --bim-formats：打印策略能力矩阵后退出
    bool        bimInheritParents = true;         // 祖先节点属性下传（子键覆盖父键）
    std::string bimSchemaFile;                    // 显式 Schema JSON（默认自动推断）
    std::string bimEncoding = "batchtable";       // batchtable | 3dnext（阶段二）
};
```

### 4.3 管线改造点（逐函数）

**① `CollectMeshInstances`（TilesConverter.cpp:76）— 身份采集**

- `traverse` 内：`objectName = StripAssimpFbxTag(node->mName)`（截断 `$AssimpFbx$` 及其后缀）；`meshName = scene->mMeshes[mi]->mName`；同时保存 `const aiNode* node`（v2.1：**节点指针入 MeshInstance**——`Convert` 执行期间 aiScene 存活，指针安全；属性解析统一放到 1a 阶段，遍历保持零逻辑分支）。
- 采集本身零成本旁路：`bimBind == false` 时不填新字段。
- ~~父链继承在遍历时顺带完成~~（v2.1 修正：**继承改为 1a 阶段按 `node->mParent` 链上溯解析**——单阶段、可单测，避免 traverse 携带 inherited 参数与 ② 的 RowFor 流程矛盾；继承只作用于场景元数据，sidecar 行不参与继承）。

**② `Convert` 步骤 1a（新增）— 属性源装配与行分配**

`CollectMeshInstances` 之后（简化与投影修正均不触碰身份字段，先后无约束）：

```
1a. // —— 策略选择与绑定管线（v2.3：分格式逻辑全部收敛在 IBindingStrategy 实现里）——
    strategy = BindingStrategyRegistry::For(scene->mImporterName)   // IFC/FBX/glTF2/OBJ/3DS/…；未识别 → Generic+Warning
    pipeline = BimBindingPipeline(strategy, SidecarTableSource?(bimPropsFile), opts)
    results = pipeline.BindAll(instances)   // 每实例：
        //   sceneRow = strategy->CollectSceneRow(inst)      // 继承（沿 inst.node->mParent 上溯，子键覆盖父键）
        //   sidecarRow = sidecar?->RowFor(JoinKeys(inst))   // join 键由策略按 L2 优先级给出
        //   row = merge(sceneRow, sidecarRow) + 保留列注入（objectId 强制 string；objectClass 由策略给出）
        //   记录 BindingResult{row, objectId, idSource, matchSource}   // 透明化三件套
    for each (inst, r): inst.properties = r.row
    日志：策略名 + 各 idSource/matchSource 计数分布（"IFC: GlobalId from name-tail 1234/1300,
          metadata-only 62, none 4"）；sidecar 匹配率 < 100% 时 Warning 列出前 N 个未命中名
    --bim-report 给出时：落盘绑定清单 JSON（见 §4.5 格式），供 sidecar 制作方核对 FBX 去重名与 ID 来源审计
```

**②′ 透明化机制（v2.3，与 ① 抽象层配套）**

- **`--bim-formats`**：运行时打印全策略 `Describe()` 能力矩阵（格式/原生 GUID/ID 来源/场景属性/限制）——**二进制即文档**，§3.2.1/§3.5.1 的静态矩阵由同一 `Describe()` 生成，杜绝文档漂移；
- **`--bim-report <file>`**：每对象一行 `{objectName, meshName, objectId, idSource, matchSource, propertyKeys[]}` + 汇总统计 + 丢弃键清单（类型推断失败、GBK 替换等）——用户可逐对象核对"我的 ID 从哪来、属性从哪来"；
- 转换日志首行输出 `[TilesConverter] BIM binding: strategy=IfcBindingStrategy format=IFC`（stdout 仅此一行新增，且仅在 bimBind 时输出——不破坏 MGOServer 进度协议的行匹配，其匹配规则为 `[TilesConverter] Progress: X/Y` 前缀式，非全量白名单）。

**③ `GroupCellByMaterial`（TileBuilder.cpp:643）— batchId 分配**

- `Accumulator` 增加 `std::vector<uint32_t> batchIds;`（与 positions 平行 push，天然规避 NaN 跳过（:674-695）与退化面跳过（:794-803）造成的错位）。
- 每个实例处理开始处：

```cpp
uint32_t bid = 0;
if (opts.propertySchema) {                       // 绑定开启（Schema 指针非空即总开关，见 ⑥）
    bid = AssignBatchId(cell.featureBatch, inst.properties);  // 行内容哈希 → 去重
}
// 顶点循环内与 positions 同步 push(bid)
```

- `AssignBatchId`：对 `BimPropertyRow` **整行（含保留列）**序列化取哈希 → cell 局部 `unordered_map<hash, batchId>`；新行 `rows.push_back(properties)`。`inst.properties == nullptr` → 惰性创建一个**共享空行**（首个未命中实例时追加，不承诺是 row 0——布局确定性由"追加序=遍历序"保证并由测试锁定，v2.1 修正）。
- `MergedMeshGroup.batchIds = std::move(a.batchIds)`；`MergeGroupsByMaterial` 增加一行：`combined.batchIds.insert(..., g.batchIds...)`（拼接，值语义，无需重定基）。
- 近空瓦片丢弃（<50 顶点，:876-886）连带 `featureBatch` 一并作废——已是 cell 级状态，无需额外处理。

**④ `GlbBuilder::Build`（TileBuilder.cpp:125）— `_BATCHID` accessor**

- `!g.batchIds.empty()` 时：BIN 段追加 `L.vertexCount * sizeof(component)`；accessor `type` 必须为 `"SCALAR"`；componentType 按行数降档：≤255 → `UNSIGNED_BYTE(5121)`、≤65535 → `UNSIGNED_SHORT(5123)`、更多 → `FLOAT(5126)`。
  ⚠ **不得使用 `UNSIGNED_INT(5125)`**：glTF 2.0 禁止其作为顶点属性访问器；`EXT_mesh_features` 官方实现说明同样要求用 `FLOAT` 承载 >2^16 的整数 ID（精确到 2^24，远超瓦片内特征数）。CPU 端 `batchIds` 存储仍用 `uint32_t`。
- primitive `attributes` 增加 `"_BATCHID": <accessorIndex>`；accessor 数与 bufferView 布局同步扩展（已实测确认：现布局每 primitive 4 个 bufferView（pos/norm/uv/idx），`bvBase = pi*4` 且 accessor `accBase = pi*4`，TileBuilder.cpp:378-431；⚠ **images 的 bufferView 追加在全部 primitive 段之后**（`imageBVStart` 基址）——加第 5 段后 `pi*4 → pi*5`，`imageBVStart` 同步移位，勿遗漏）。
- b3dm 规范：**同一 glb 内所有 primitive 必须一致携带 `_BATCHID`** —— 因绑定开启时每个实例（含空行实例）都写 ID，天然满足（D6）。

**⑤ `B3dmBuilder::Build` 重载（TileBuilder.cpp:510）— BatchTable**

```cpp
// 新重载；旧签名保留并转发（batchTableJson 为空 → 现行为，逐字节一致）
static bool Build(const BinaryBlob& glb,
                  const std::string& batchTableJson,        // "" = 无批表
                  const std::vector<uint8_t>& batchTableBinary,
                  uint32_t batchLength,
                  BinaryBlob& outB3dm);
```

- FeatureTable JSON：`{"BATCH_LENGTH":N}`（语义 `uint32`、**必填**；glTF 无 `batchId` 属性时必须为 0——现有硬编码 `0` 本身即规范合规，无需动）。
- header 的 `btJsonLen / btBinLen` 填实际值（`btJsonLen==0` 时 `btBinLen` **必须为 0**）；body 顺序：FT JSON → FT bin → BT JSON → BT bin → glb。
- **对齐硬性规则**（官方 Padding 节，逐条实现并单测）：
  - FT/BT 的 JSON 段以空格（`0x20`）补齐，使其**结束于 tile 起点的 8 字节边界**（现有 FT JSON 20B：28+20=48 ✔；加批表后 BT JSON 同样需补齐至 8）；
  - BT 二进制体**起止均在 8 字节边界**（补齐字节值任意，取 `0x00`）；
  - **glb 必须起始于 8 字节边界**——通过填充 BT（无 BT 时填 FT）实现；
  - tile 总 `byteLength` 8 字节对齐（尾部补 `0x00`）；消费者须以 GLB 头部的 length 为准，忽略尾部 padding（b3dm README 明示）。

**⑥ `BatchTableWriter`（新增，TileBuilder 内）**

```cpp
class TILE_BUILDER_API BatchTableWriter {
public:
    // rows → BatchTable（官方 Batch Table 规范，两种表示法）：
    //  1) JSON 内联数组："prop" : [v0, v1, …]，数组长度 == BATCH_LENGTH；
    //     元素可为任意 JSON 类型（对象/数组/null 均合法）；字符串与布尔只能走此形式。
    //  2) 二进制引用对象："prop" : {"byteOffset":N,"componentType":"DOUBLE","type":"SCALAR"}
    //     指向 BT binary body；byteOffset 须为该 componentType 尺寸（1/2/4/8B）的整数倍；
    //     componentType ∈ {BYTE…UNSIGNED_INT, FLOAT, DOUBLE}；type ∈ {SCALAR, VEC2, VEC3, VEC4}。
    // 策略（列三态，§4.8）：全行非空的数值/Vec3 列 → 二进制列（每列 batchLength 个紧凑值，
    //   列起点按分量大小对齐）；含 null / 字符串 / 布尔列 → JSON 数组（二进制定长列无法表达
    //   缺失值）；全 null 列 → 不输出。JSON 数组缺失值 = null（规范允许，Cesium 读取 undefined）。
    // 小表优化：rows ≤ 32 且字段 ≤ 8 时全部走 JSON 数组（实现最简，体积可忽略）。
    static bool Build(const PropertySchema& schema,
                      const std::vector<std::shared_ptr<const BimPropertyRow>>& rows,
                      std::string& outJson, std::vector<uint8_t>& outBinary);
};
```

**⑦ `TilesetWriter::WriteTiles`（TileBuilder.cpp:1143）**

glb 构建后调用 `BatchTableWriter`，再走 `B3dmBuilder` 新重载。Schema 通过 `TileBuildOptions` 传入：增加 `const PropertySchema* propertySchema = nullptr;`（**非空即绑定总开关**，TileBuilder 不需要 bool 标志位）。

**⑧ Schema 推断（TilesConverter 新私有方法）**

`Convert` 在步骤 3（写 tile）之前执行：

```
ComputePropertySchema(*m_gridRoot)：
    遍历全部 cell.featureBatch.rows → 每 key 的类型并集 → 取最宽：
        String > Double > Int64 > Int32 > Bool
    键集 = 所有行键的并集，按首次出现排序（稳定输出）
    bimSchemaFile 提供时以其为准，冲突告警
    Schema 挂入 buildOpts.propertySchema
```

补充规则：

- 浮点字段编码前必须做 **NaN/±Inf 过滤**（3D Metadata 规范对 FLOAT32/64 明令禁止，b3dm Batch Table 二进制列同理）→ 违规值置 `null` 并计数告警；
- 属性键在 legacy BatchTable 中原样使用（JSON 键无字符集限制）；阶段二 structural metadata 的 class/property **ID 必须匹配 `^[a-zA-Z_][a-zA-Z0-9_]*$`** → Schema 推断时为每个键生成合法 `id`（非法字符→`_`，数字开头→前缀 `p_`，冲突→序号），原始键（含中文/空格）保存为 class property 的 `name` 显示字段。两套编码共用同一 `PropertySchema`，仅 3dnext 输出净化 ID；

显式 Schema JSON 示例（`--bim-schema schema.json`）：

```json
{
  "class": "BuildingElement",
  "properties": {
    "GlobalId":  "string",
    "ElementType": "string",
    "Level":     "string",
    "FireRating": "int32",
    "Volume":    "double"
  }
}
```

### 4.4 输出示例（启用绑定后的 b3dm）

```
b3dm header (28B)
  magic="b3dm" version=1 byteLength=… 
  featureTableJsonByteLength=20      featureTableBinaryByteLength=0
  batchTableJsonByteLength=B         batchTableBinaryByteLength=B2   ← 新
feature table JSON  {"BATCH_LENGTH":42}
batch table JSON    { "ElementType": ["Basic Wall","Door",…],
                      "Level": ["1F","1F",…],
                      "Volume": {"byteOffset":0,"componentType":"DOUBLE","type":"SCALAR"} }
batch table binary  [ DOUBLE×42 紧凑列数组；各列起点仅按分量大小(8B)对齐，体结束补至 8B 边界 ]
glb                 { … primitives.attributes 增 "_BATCHID": a（UBYTE/USHORT/FLOAT，禁 UINT）… }
```

CesiumJS 拾取（MGOServer viewer，public/viewer.html:310 处已加载 tileset；API 已按自带 1.111 构建产物核验）：

```js
const picked = viewer.scene.pick(movement.position);
if (picked instanceof Cesium.Cesium3DTileFeature) {
    console.log('batchId =', picked.featureId);            // 1.111+ 可读
    for (const id of picked.getPropertyIds())              // 旧名 getPropertyNames 已弃用
        console.log(id, picked.getProperty(id));
}
```

### 4.5 CLI 设计（MGOConsole `tiles`）

```
--bim-metadata            采集模型内置元数据（FBX/IFC/glTF extras）为属性
--bim-props <file>        外部属性表（CSV/JSON，UTF-8）；给出即开启绑定
--bim-key <auto|node|mesh> sidecar 匹配键（auto = §3.2 L2 全序：内嵌 GUID → 节点名 → 网格名；P2 增加 path 消歧）
--bim-id-column <name>    sidecar 匹配列（默认 auto：GlobalId/ObjectId/首列）—— 这是"行怎么连上对象"的连接键
--bim-id-property <k1,k2> objectId 取值候选键（默认 GlobalId,ElementId,ifcGUID）—— 这是"行内保留列的值从哪来"，与上者语义不同
--bim-fields <k1,k2|file> 属性输出白名单（列多/含敏感台账字段时使用，§4.8）
--bim-feature-granularity <object|type> 去重粒度：object=按对象一行（默认，拾取/高亮单对象）；type=身份列不入行、同型构件共享 feature（拾取=选中全部同型，行数最小）
--bim-report <file>       导出绑定清单 JSON（每对象 objectName/objectId/idSource/matchSource + 统计 + 丢弃键）
--bim-formats            打印各格式绑定策略能力矩阵（格式/原生GUID/ID来源/场景属性/限制），即时退出
--bim-strategy <name>    强制指定绑定策略（默认按格式自动选择；排障用）
--bim-no-inherit          关闭父节点属性继承（默认继承）
--bim-no-scene-metadata   显式关闭内置元数据采集
--bim-schema <file>       显式字段类型 Schema JSON
--bim-encoding <batchtable|3dnext>   输出编码（3dnext 为阶段二）
```

示例：

```bash
mgo tiles -i model.fbx -o out --prj epsg:4547 \
  --bim-metadata --bim-props elements.csv --bim-id-column GlobalId
```

sidecar CSV 示例（首行为表头，UTF-8）：

```csv
GlobalId,ElementType,Level,FireRating,Volume
3f2a…c1,Basic Wall,1F,120,3.42
```

### 4.6 兼容性与回归保障

- `bimBind == false`：`MeshInstance` 新字段为空、`MergedMeshGroup.batchIds` 为空、`buildOpts.propertySchema == nullptr` → `_BATCHID` accessor 不生成、B3dmBuilder 走旧签名、FT JSON 逐字节保持 `{"BATCH_LENGTH":0}`；
- 绑定开启但命中行数 ≤ 1 且行内容为空 → 告警并回退旧路径（等价关闭）；
- 现有 `TileBuilder/test/test_emptyprimitive.cpp`、`test_overflow404.cpp` 无需修改；
- `SimplifyScene`（步骤 0b，作用于 aiMesh 内部、实例收集之前）与 batchId（实例级、合并期分配）无耦合；per-vertex 投影修正只改顶点坐标，不触碰身份字段；
- **硬性不变量（评审必查）**：特征边界依赖"顶点按实例独立复制"。任何后续优化都**不得对合并后的瓦片顶点缓冲做跨实例焊接/去重**（例如对 MergedMeshGroup 跑 meshopt 顶点合并、融合相邻构件的共享顶点）——那会把两个 feature 的顶点焊成一个，batch 边界被静默破坏（拾取跨界、无告警）。简化必须停留在 aiMesh 级（现行 `SimplifyScene` 位于实例收集之前、网格内部，安全）；
- 外部子树 tileset / overflow 布局不变（批表自包含于每个 b3dm，`tileset.json` 无需任何改动）。

### 4.7 阶段二要点：3D Tiles Next 编码器（`--bim-encoding 3dnext`，按官方规范核对）

内容载体从 b3dm 换为**裸 glb**（tileset `content.uri` 直接指向 `.glb`，asset.version 已是 1.1），批表换为两个 glTF Vendor 扩展：

| 规范要点（已核对原文） | 对本设计的影响 |
|---|---|
| 顶点特征 ID 属性语义为 `_FEATURE_ID_n`（`EXT_mesh_features.featureIds[].attribute = n`），替代 `_BATCHID`；accessor `SCALAR`、`normalized:false`、同样禁 `UNSIGNED_INT` | `GlbBuilder` 中 batch 属性名做成可配置语义（阶段一 `_BATCHID`，阶段二 `_FEATURE_ID_0`），其余数据流不变 |
| `featureIds[]` 支持 `featureCount`、`nullFeatureId`、`propertyTable`（指向根级 propertyTables 下标）、`label`；一个 primitive 可挂多套 ID | 共享空行可声明为 `nullFeatureId`，拾取时返回"无 feature"，语义优于 legacy 批表 |
| propertyTable：`{class, count, properties{id → {values, stringOffsets?, arrayOffsets?, …}}}`；**`values` 是 bufferView 下标而非 accessor**（列式紧凑存储）；STRING 存 UTF-8 字节流 + `stringOffsets`（N+1 个字节偏移，默认 `UINT32`）；BOOLEAN 按位打包 `ceil(N/8)` 字节 | `BatchTableWriter` 抽象为"行 → 列存储"接口，legacy 批表与 propertyTable 两种落盘后端复用同一列化逻辑 |
| Schema 类/属性 ID 净化规则见 §4.3-⑧；属性可标 `required`，缺列时 required 不得省略 | 推断出的全 tileset 统一 `PropertySchema` 直接序列化进 schema 对象 |
| 采用该扩展的 asset 需把 `EXT_mesh_features`/`EXT_structural_metadata` 写入 `extensionsUsed`（**不写 `extensionsRequired`**，规范建议 optional） | GlbBuilder 增加 `extensionsUsed` 输出分支 |
| 该扩展把 GLB 对齐提升到 8 字节（JSON chunk 补 `0x20` 至 8、BIN chunk 补 `0x00` 至 8；核心 glTF 仅要求 4） | `GlbBuilder` 现有 4 字节 JSON padding 需参数化（3dnext 模式传 8） |
| Content 的 schema 承载（v2.1 修正）：`EXT_structural_metadata` 自带 `schemaUri` 外链能力（README 原文："Multiple glTF assets may refer to the same external schema to avoid duplication"），**propertyTable 的 class 必须在其所属 schema（glb 内嵌或 glb 级 schemaUri 外链）中声明** | Schema 外置为**单一** `schema.json`，每个 glb 的扩展对象写 `schemaUri: "schema.json"`（相对 URI，同级文件）——零重复且规范自洽。⚠ 注意：**tileset 级 `schema`/`schemaUri` 服务的是 tileset/tile/group 实体元数据**，不是 content propertyTables 的类定义来源（v2 曾误用，已在 §10#22 记录）；tileset 级 metadata 本设计 P2 不输出 |
| `statistics`（class 级 min/max/occurrences 等）为官方对象 | P3 可顺带产出，支撑 Cesium 声明式样式（色带/直方图免全量解析） |

消费端一致性：CesiumJS 拾取返回 `Cesium3DTileFeature`，`getProperty/getPropertyIds/hasProperty` 对批表与 structural metadata 统一工作；`Cesium3DTileFeature.getPropertyInherited` 的查找顺序（批表 → content → tile → group → tileset）即两套编码混用时行为可预期的依据；三角形内部拾取归属"最近顶点"的 feature（EXT_mesh_features 实现说明）。

### 4.8 属性表规格（表结构设计核心）

一张表 = 每瓦片的"特征 × 属性"表：**行 = 去重后的属性行（batchId = 行号），列 = 全 tileset 统一的 Schema**。四段规格：

**(1) 列分层：系统保留列 + 用户列**

| 层 | 列 | 类型 | 值来源（按优先级） | 说明 |
|---|---|---|---|---|
| 保留 | `objectId` | String（**强制字符串**） | ① IFC 名尾段 GlobalId → ② 内置 metadata 指定键（`--bim-id-property`，默认 `GlobalId,ElementId,ifcGUID`——裸 `ID` 不入默认防误命中）→ ③ sidecar ID 列 → ④ 兜底 objectName | 前端接台账/工单/REST 的**唯一稳定键**（§3.2 L3′）；字符串化防止 GUID/64 位 ElementId 在 JS number 精度丢失 |
| 保留 | `objectName` | String | `$AssimpFbx$` 剥除后的节点名（含 assimp 去重 `NNN` 后缀） | 反查建模软件对应物；配合 `--bim-report` 导出清单 |
| 保留 | `objectClass` | String | IFC：名字首段（`IfcWall`…，可靠）；FBX：metadata 的 `Family`/`EntityLevel_*` 等键——**启发式（best-effort）**，键名因导出器而异，未命中置空；否则空 | 支撑声明式样式按类着色（`${objectClass}`） |
| 用户 | 原键直传 | per-Schema | 内置 metadata 键 + sidecar 列名 | 冲突策略：**sidecar 覆盖内置**（台账为权威），覆盖计数告警；用户键若撞保留列名 → 保留列胜出并告警 |

**(2) 行生成与去重**

- 每 MeshInstance 恰好一行合并结果，merge 顺序：继承祖先键 → 本节点 metadata → sidecar 行（后者覆盖前者的同名键）；
- 行整行序列化哈希（含保留列）→ 瓦片内去重共享（D2）；未命中实例 → 共享空行（惰性创建，§4.3-③）；
- **去重粒度**（v2.1 补，`--bim-feature-granularity`）：`object`（默认）= 身份列参与哈希 → 每对象一行，拾取/高亮作用于单对象（正确语义的代价：D2 的"同型 1 行"不成立）；`type` = objectId/objectName 不入行 → 同型构件共享 feature，行数最小化，拾取语义变为"选中全部同型实例"。两种模式都不影响管线结构，仅改行内容与哈希键；
- sidecar 同键多行 → 取首行 + Warning（R2）；表有行、模型无对象 → 计入未使用统计，不报错。

**(3) 类型系统与两后端编码映射**

| BimValue | legacy BatchTable | 3dnext propertyTable |
|---|---|---|
| Bool | JSON 数组（二进制无 BOOL 类型） | BOOLEAN 位打包 |
| Int32 | binary `INT`(4B) | INT32 |
| Int64 | binary `INT`(4B)——legacy Batch Table **无 INT64 二进制类型**，仅当全列落入 int32 时降格写入；超范围则整列走 JSON 数组（保留精确整数字面量，不经浮点） | INT64/INT32 |
| Double | binary `DOUBLE`；含 NaN/Inf 的列 → 整列降级 JSON（null 化） | FLOAT64（NaN/Inf 仍禁止） |
| Vec3 | binary `VEC3/FLOAT` | VEC3/FLOAT32 |
| String | JSON 数组（UTF-8 转义） | UTF-8 字节流 + `stringOffsets`(UINT32) |

- **可空性硬规则**：Batch Table 二进制列是定长数组，**不能表达缺失值** → 某列仅当**所有行都有值**才允许二进制化；出现任何空值即整列走 JSON 数组（规范允许 null）。BIM 属性（Pset 天然稀疏、sidecar 参差列）普遍触发此规则，是字符串列多数走 JSON 的根本原因，实现按"列三态"处理：全有值→binary；有 null→JSON；全 null→不输出列；
- **IFC 字符串化对策**：内置数值已丢失类型（§3.1）→ 自动推断**不对字符串值做数值提升**（防编号类数字串误判），IFC 模型数值列类型恢复以 `--bim-schema` 显式声明为准；FBX 类型化 metadata 与 sidecar 数值列正常参与推断；
- 类型加宽顺序：`Bool < Int32 < Int64 < Double < String`；Vec3 独立通道（FBX vec3 → VEC3，不与 Double 混列）。

**(4) 规模与治理**

- **列数控制**：Revit 族参数可达数百 → `--bim-fields` 白名单只输出业务需要列（体积 + 敏感字段隔离），其余属性留在业务库以 `objectId` 外键回查——**"3D Tiles 只带瘦属性、胖台账走外部系统"是本设计的默认立场**；
- 最坏体积估算：10k 行 × 20 列，JSON 字符串列 ≈ 3.2 MB/瓦片 vs 二进制数值列 ≈ 0.8 MB——相对几何体（数十 MB）可忽略，列裁剪是主要可控项；
- 稳定性：列序 = 首次出现序（cell 按 cellKey 遍历序），行序 = 去重追加序 → 同输入逐字节可复现；
- 保留列名与 `3DTILES_batch_table_hierarchy` / 未来 ENUM 化（枚举列字典编码）共用前缀槽位，不引入用户列冲突。

**(5) 失败模式（v2.1 补）**

| 情形 | 策略 |
|---|---|
| sidecar 文件不存在 / 无法解析 / 非 UTF-8 编码错误 | **fail-fast**：转换失败退出（exit 1），明确 stderr 指明文件与行号——属性绑定是用户显式意图，静默降级会造成"看似成功但属性全无"的难排查状态 |
| CSV 行列数不齐 / JSON 字段类型异常 | 行级容错：Warning + 跳过该行（与 CSVReader 现行为一致），计入报告 |
| sidecar 键与模型无一命中 | 正常完成 + 显式统计（"0/N matched"）+ exit 0（几何转换本身成功）；报告给出前 N 个模型侧 objectName 供排查 |
| 显式 Schema 文件类型与数据冲突 | Warning + 以数据实际类型为准（Schema 是声明不是断言） |

---

## 5. 消费端（CesiumJS / MGOServer）

- MGOServer viewer 固定自托管 CesiumJS 1.111（public/viewer.html:185-190）；已 grep 自带构建产物核验：`getPropertyIds`/`hasProperty` 存在（旧名 `getPropertyNames` 仅剩 1 处，视为弃用）、`EXT_structural_metadata`/`EXT_mesh_features` 解析代码均存在 → **两种编码都能拾取**；
- 阶段一输出对 viewer **零改动即可用**（属性随 b3dm 自包含）；建议 MGOServer 后续增加"构件属性面板"（pick → `Cesium3DTileFeature.getPropertyIds()/getProperty()`，并用 `featureId`/对象 ID 属性关联业务台账）；
- **绑定只发生一次（转换期），前端不存在"绑定"步骤**：几何侧 `_BATCHID` 顶点属性与属性侧每瓦片 BatchTable 均已烘焙进 b3dm。运行期关联链路 = `scene.pick` 命中三角面 → 读取该三角面顶点携带的 `_BATCHID`（CesiumJS 拾取通道）→ 得到 `Cesium3DTileFeature` → `getProperty` 按 batchId 取批表行；业务侧再以 `objectId` 对台账 REST/DB 做运行期 join（"瘦瓦片 + 胖台账"，§4.8）。声明式样式 `Cesium3DTileStyle ${PropertyId}` 同样读批表，零胶水代码；
- UI 注意事项（官方文档明确）：`feature.show/feature.color/setProperty` 的修改**只存活于瓦片内容生命周期**，出视野卸载即丢失——面板若做选中高亮需监听 `tileset.tileUnload/tileVisible` 重放，持久状态应存对象 GlobalId；
- 属性 ID **大小写敏感**（`getProperty` 文档明示 case-sensitive）：BatchTable 键原样取自 FBX 元数据/CSV 表头，MGOServer 面板与声明式样式（`Cesium3DTileStyle` 的 `${PropertyId}` 表达式，b3dm+批表可直接使用）必须精确匹配键名；建议输出侧在 Schema 日志中打印最终属性 ID 清单供前端对齐；
- 跨仓约束不变：MGOServer 只通过 `MGOConsole` 二进制与本仓交互，本设计不新增/不修改 stdout 进度协议行（`[TilesConverter] Progress: X/Y`），无需同步 `MGOServer/src/jobs/progress.js`。

---

## 6. 性能与内存

| 项 | 估算 | 缓解 |
|---|---|---|
| `_BATCHID` 顶点属性 | 1–4 B/顶点：≤255 行 → 1B、≤65535 行 → 2B、更多 → FLOAT 4B（glTF 禁顶点属性用 UINT，见 §10） | 按行数自动降档 componentType |
| 属性行内存 | 唯一对象数 × 行大小（shared_ptr 共享 + 瓦片内哈希去重） | 与实例数解耦 |
| BatchTable 体积 | 数值列：行数 × 分量大小（4/8B）；字符串列：JSON 数组（含重复值，行去重后已最小化） | 相对几何体积通常 <1–2% |
| 转换期开销 | 匹配 O(N)（名称 → 行的哈希查找）；哈希去重 O(实例数) | 无逐顶点成本（顶点循环内仅一次 push） |
| 运行期 | CesiumJS 对 BatchTable 惰性解析，无渲染帧开销 | — |

---

## 7. 实施计划

> **实施状态（v2.4）**：P0 + P1 已落地 —— `BimFormatStrategy.h/.cpp`（IBindingStrategy + 六策略 + Registry）、`BimBindingPipeline.h/.cpp`（含 SidecarTableSource）、TileDataTypes 扩展（BimValue/BimPropertyRow/FeatureBatchTable + MeshInstance/MergedMeshGroup/GridCell 扩展）、TileBuilder 侧 `_BATCHID`（GlbBuilder）/ 批表（BatchTableWriter）/ B3dmBuilder 重载、MGOConsole `--bim-*` CLI、单测 `test_bimstrategy` + `test_batchtable` 全绿、E2E 验证完成（OBJ sidecar 命中 / glTF2 extras / 关闭开关与基线逐字节一致）。P2/P3 未动。注意：`--bim-schema` 显式 schema 与 `--bim-key`/`--bim-feature-granularity`（见 §4.5）本轮未实现，留待后续；⑩⑪⑫ 号测试用例随其补齐。
>
> **v2.5 评审修复注记**：v2.4 落地代码经全量评审后修复 19 项（编号见评审报告），全部有独立解码器/探针回归佐证：
> ① B3dmBuilder 各段改为**自瓦片起点**的绝对 8B 对齐且头部长字段包含填充（消费端按长度求和定位 glb）；
> ② GlbBuilder `_BATCHID` 段后按 %4 补齐，杜绝 UBYTE/USHORT 奇数长度拖偏后续 FLOAT bufferView（仅绑定路径生效，legacy 字节不变）；
> ③ BatchTableWriter 对 NaN/±Inf 整列降级 JSON + 值置 null + `outNullified` 计数告警，键名全量 JSON 转义，JSON 双精度输出最短往返表示；
> ④ 二进制列映射对齐 §4.8(3)：Int32/Int64(≤int32)→`INT`5124、Double→`DOUBLE`5128（不再 float32 降采样）、Vec3→`VEC3/FLOAT`5126；
> ⑤⑥⑦⑧⑨ 语义层：metadata/sidecar 数值 ID 字符串化（LookupKeys 兼容嵌套展平后的 `Pset.ifcGUID` 叶名后缀匹配）、objectId 优先级链按 §4.8(1) 重排（①name-tail → ②显式键/内置键（metadata 后 sidecar）→ ③sidecar ID 列 → ④弱 objectName）、`--bim-id-property` 逗号串逐项 Trim、保留列冲突改"去重+胜出+计数上报"（`Summary.reservedCollisions/droppedKeys` + `--bim-report` 同名字段）、sidecar 读入失败改 **fail-fast 退出 1**（§4.8(5)）；
> ⑪⑫ batchId 分配移至 NaN 预检之后防孤儿行；`FeatureBatchTable::AssignBatchId` 改 RowHash 分桶索引（4000 行 451ms→1ms，60k 实例可行）；
> ⑬~⑲ CSV RFC4180 引号/转义引号/引号内换行/BOM、格式嗅探 basename+扩展名优先（路径含 "ifc" 不再误判）、`DefaultIdKeys()`/`SidecarIdKeys()` 单一来源（移除裸 "ID"）、AI_UINT32→Int64/AI_UINT64 溢出→精确十进制字符串、AI_AIMETADATA 嵌套字典展平（深度≤3）、`FixTextEncoding()`（UTF-8 校验→Windows CP_ACP / POSIX GB18030-iconv 转码→U+FFFD 有损兜底）贯通节点名/元数据/sidecar 三处摄入点并计数告警、`--bim-report` 字符串全转义 + 非有限值置 null、Generic 回退显式告警、删除死的 `bimListFormats` 字段、`§4.6` 单空行瓦片回退 legacy 并告警、withRow==0 总告警、`--bim-props` 隐含 `--bim-bind`（CLI）+ `--bim-metadata` 别名。
>
> **v2.5 真实数据验证（Data/roadbed/root.fbx，180MB/32087 实例/1340 瓦片）**：FBX 基线 29.4s；`--bim-bind` 同耗时（哈希去重索引生效），模型无 UDP GUID → `idSourceCounts={objectName:32087}` 正确回退并透明上报；实测 64174 个 GBK 节点名——#17 修复链路按设计工作，Windows CP_ACP / POSIX `iconv(GB18030)`（CMake `check_include_file_cxx(iconv.h)` 探测，缺失时退化 FFFD）转码后中文完整恢复（`SideSlope_会巧高速 施工图@Z2_左_ZK48+…`，抽样 10000 features 0 U+FFFD），批表/报告 JSON 合法可解析。**#20（真实数据新发现）**：image bufferView 此前不做 %4 对齐（PNG/JPEG 任意长度，993/1340 瓦片违反 glTF §5.1.2，先于 BIM 改造存在于 legacy 路径，合成单纹理数据未触发）——GlbBuilder 三处图像追加点统一补零，修复后 legacy 与 BIM 两路径 1340+1340 瓦片独立解码 0 失败。注：#20 使 legacy 字节流相对 v2.4 增加每图 0-3 字节填充（格式正确性修复优先于 D3 基线，D3"绑定开关不改变非绑定行为"语义不变）。

| 阶段 | 内容 | 验收 |
|---|---|---|
| **P0 打通链路（含抽象层）** | TileDataTypes 扩展；**`IBindingStrategy` + `BindingStrategyRegistry` + 首批策略（Ifc/Fbx/Gltf/Obj/ThreeDs/Generic，v2.3）**；`BimBindingPipeline`；身份采集（含 `$AssimpFbx$` 还原、父链继承）；GroupCellByMaterial batchId；GlbBuilder `_BATCHID`；B3dmBuilder 重载 + BatchTableWriter；单测 | 含元数据的 FBX 输出在 CesiumJS 中可拾取出属性；`--bim-formats` 输出与 §3.2.1 矩阵一致 |
| **P1 外部表 + CLI + 透明化** | SidecarTableSource（CSV/JSON，join 键由策略给出）、Schema 推断 + 显式 Schema、MGOConsole `--bim-*`、**`--bim-report` 绑定清单 JSON**、`Script/Test/bim_verify.py` | CSV 全量字段与拾取值一致；关闭开关输出与基线逐字节一致；报告含每对象 idSource/matchSource |
| **P2 3D Tiles Next + 面板** | `ThreeDNextEncoder`：裸 glb 内容（**TilesetWriter 增 `.glb` content 输出分支**，WriteTiles 3dnext 模式直写 glb、跳过 B3dmBuilder）+ `_FEATURE_ID_0` + `EXT_mesh_features.featureIds` → `EXT_structural_metadata.propertyTables`（bufferView 列存 + STRING 用 stringOffsets）；Schema 外置单一 `schema.json`，每 glb 经自身 `schemaUri` 引用（§4.7）；`--bim-encoding 3dnext`、MGOServer 属性面板 | 3d-tiles-tools `validate` 通过；同一 viewer 拾取语义不变（getPropertyInherited） |
| **P3 优化/扩展** | 跨瓦片共享属性表（external batch table / metadata store 方案择一）、FBX 属性键名映射配置、新格式策略（每加一个格式 = 新增一个 `IBindingStrategy` 实现 + 注册，管线零改动）、IPropertySource 接 IfcOpenShell | 重复属性体积显著下降 |

### 7.1 测试设计

| 层 | 测试 | 覆盖点 |
|---|---|---|
| 单元（TileBuilder/test） | `test_batchtable.cpp`（新增） | ① glb 含 `_BATCHID` accessor（type=SCALAR、componentType 三档降档：255/65535/256 行三个 fixture）；② b3dm 头 `btJsonLen/btBinLen/BATCH_LENGTH` 与实际内容一致；③ **全部 8B 对齐断言**：FT JSON 结束位、BT JSON 结束位、BT binary 起止、glb 起始、总长（§10#6）；④ 二进制引用对象含 `type` 字段且 byteOffset 满足分量大小倍数；⑤ 行内容哈希去重：同属性双实例共享 batchId；⑥ 无属性实例 → 共享 row 0；⑦ NaN/Inf 数值 → null 落表 |
| 单元（TileBuilder/test） | 既有 `test_emptyprimitive.cpp`/`test_overflow404.cpp` + 新增字节级基线比对 | 关闭绑定时输出与基线**逐字节一致**（§4.6 承诺的回归护栏） |
| 单元（TilesConverter） | `test_bim_source.cpp`（新增） | FBX 链名 `$AssimpFbx$` 截断还原；父链继承（子同名覆盖父）；sidecar CSV/JSON 解析与匹配统计；ID 净化正则（中文/空格/数字开头键） |
| 脚本（Script/Test/） | `bim_verify.py`（新增，风格同 `verify_output.py`） | 独立解码输出目录全部 b3dm：解析头/FT/BT → 与输入 sidecar CSV 逐行逐字段比对（含二进制列 little-endian 解码）、校验 `_BATCHID` 值域 `[0, BATCH_LENGTH)` |
| 工具校验 | 3d-tiles-tools `npx 3d-tiles-tools validate`（CI 步骤） | b3dm+批表结构合规；P2 起对 3dnext 输出同跑 |
| 手工验收 | MGOServer viewer 加载私有回归数据（Data/Test，FBX 含 Revit 属性） | 拾取弹窗字段与 Revit 侧一致；声明式样式 `${ElementType}` 生效 |
| 单元（策略层，v2.3 补） | `test_bimstrategy.cpp` | ⑬ `--bim-formats` 输出与 §3.2.1 矩阵逐行一致（Describe() 快照测试）；⑭ Registry 按 importerId 正确分发（IFC→Ifc…，未识别→Generic+Warning）；⑮ 各策略身份解析：IFC 名字尾段 GlobalId 提取、FBX `$AssimpFbx$` 剥离、OBJ 返回 None；⑯ `--bim-report` JSON 含全部 idSource/matchSource 字段且统计正确 |
| 单元（回归，v2.1 补） | `test_batchtable.cpp` 追加用例 | ⑧ **去重语义**：object 粒度（含身份列）两同型对象不共享 batchId；type 粒度共享；⑨ 属性列序/行序稳定性（同输入两次转换逐字节一致）；⑩ `--bim-schema` 对 IFC 字符串化数值列的类型恢复（"3.42"→DOUBLE）；⑪ GBK 属性值 → UTF-8 转换与失败替换告警；⑫ sidecar 缺列/多列/编码错 → fail-fast 与行级跳过的退出码行为 |
| **已落地（v2.4 实况）** | `test_bimstrategy`（9 组）+ `test_batchtable`（5 组） | 实际覆盖：Registry 分发与格式归一（`.ifc`/`IfcImporter`/`model.obj` 等）；Describe() 矩阵六策略断言；IFC 名字尾段提取（正/负例含纯数字尾段拒绝）；FBX/glTF 元数据 id 键 + objectName 回退 + JoinKeys 优先级 + 父链继承覆盖语义；`$$$DUMMY` 回退 meshName；sidecar 解析（首行胜出/类型化/缺值跳过/错误文件 fail-fast）；管线合并语义（sidecar 覆盖同键 scene 值、objectId 强制 string、idSourceCounts）；FeatureBatchTable 内容哈希去重 + 共享空行；批表三态列（全数值→二进制引用含 type/对齐、含 null→JSON 数组、全空→列剔除；Vec3 二进制列；bool/int32→JSON）；b3dm 头与 8B 对齐全断言（FT/BT JSON 0x20 填充、BT 二进制 0 填充且头字段记录未填充长度）；`_BATCHID` 组件三档（5121/5123/5126，禁 5125）与 count==顶点数；全管线 GroupCellByMaterial→GlbBuilder→B3dmBuilder 的 batchId/批表贯通；**关闭开关与基线逐字节一致**（b3dm+tileset.json 双比对，E2E） |

---

## 8. 风险与开放问题

| # | 风险/问题 | 对策 |
|---|---|---|
| R1 | FBX 导出器写入的属性值为 GBK/本地编码，非 UTF-8 | 写入 BatchTable 前做 UTF-8 校验，失败按项目 GBK→UTF-8 惯例转换，仍失败则替换并 Warning |
| R2 | sidecar 同名对象匹配歧义（两堵墙同名） | 文档要求唯一键（GlobalId/ElementId）；匹配计数 >1 时告警并列出；`--bim-key mesh` 可换键 |
| R3 | 属性行跨瓦片重复（b3dm 自包含） | 量级小（P2 前），P3 共享属性表解决；先出体积数据再决策 |
| R4 | BatchTable / `_BATCHID` 编码细节被 Cesium 严格校验 | 对齐规则已在 §10 对照表逐条锚定官方原文（FT/BT JSON 结束于 8B 边界、BT 二进制起止 8B、二进制属性按分量大小对齐、glb 起始 8B）；全部在 BatchTableWriter/B3dmBuilder 单点实现；CI 以 3d-tiles-tools `validate` 校验样例 |
| R5 | 全部 primitive 必须一致携带 `_BATCHID` | 规范确认："When a Batch Table is present or BATCH_LENGTH > 0, the `_BATCHID` attribute is required"。D6：空属性实例统一映射共享空行（惰性创建） |
| R6 | `_BATCHID` 值语义 | 规范确认 batchId 为 `[0, BATCH_LENGTH-1]` 整数、可乱序、一顶点仅属一模型（跨模型顶点须复制——本管线实例天然不重叠，满足）。注意 glTF 顶点属性禁用 `UNSIGNED_INT`（§10），>65535 行走 FLOAT |
| O1 | ~~tileset.json 是否可附 Schema~~ **已解决（v2.1 修正结论）** | Content 属性的 Schema 共享机制 = 每个 glb 的 `EXT_structural_metadata.schemaUri → schema.json`（规范原文支持多资产引用同一外链 schema；1.111 已实测含该解析路径）。tileset 级 `schema`/`schemaUri` 仍存在但服务 tileset/tile/group 实体元数据，本设计不使用。阶段一 legacy 批表不需要 schema |
| O2 | OSGBConverter（倾斜摄影）无 BIM 语义 | 明确不在范围；IPropertySource 架构上不阻碍未来接入矢量投射类属性 |
| O3 | `3DTILES_batch_table_hierarchy`（父子属性继承，经典批表路线） | 阶段二 structural metadata 的 class/统计能力覆盖同类需求，暂不实现 legacy 层级扩展 |

---

## 9. 参考资料（正文均已按下列原文核对，见 §10 对照表；检索日期 2026-09）

- 3D Tiles — Batched 3D Model (b3dm) 规范：https://github.com/CesiumGS/3d-tiles/blob/main/specification/TileFormats/Batched3DModel/README.adoc
- 3D Tiles — Feature Table：https://github.com/CesiumGS/3d-tiles/blob/main/specification/TileFormats/FeatureTable/README.adoc ；JSON Schema：specification/schema/TileFormats/b3dm.featureTable.schema.json
- 3D Tiles — Batch Table：https://github.com/CesiumGS/3d-tiles/blob/main/specification/TileFormats/BatchTable/README.adoc
- 3D Tiles 1.1 — 规范总览与 Legacy 迁移指南：https://github.com/CesiumGS/3d-tiles/blob/main/specification/TileFormats/glTF/MIGRATION.adoc
- 3D Tiles 1.1 — tileset 根对象 Schema（schema/schemaUri/statistics/groups/metadata，`properties` 已废弃）：https://github.com/CesiumGS/3d-tiles/blob/main/specification/schema/tileset.schema.json
- 3D Metadata Specification（概念/类型/二进制表格式）：https://github.com/CesiumGS/3d-tiles/blob/main/specification/Metadata/README.adoc
- glTF — `EXT_mesh_features`：https://github.com/CesiumGS/glTF/blob/3d-tiles-next/extensions/2.0/Vendor/EXT_mesh_features/README.md
- glTF — `EXT_structural_metadata`：https://github.com/CesiumGS/glTF/blob/3d-tiles-next/extensions/2.0/Vendor/EXT_structural_metadata/README.md ；propertyTable schema：同目录 `schema/propertyTable.schema.json`、`schema/propertyTable.property.schema.json`
- CesiumJS — `Cesium3DTileFeature` API：https://cesium.com/learn/cesiumjs/ref-doc/Cesium3DTileFeature.html
- glTF 2.0 — accessor componentType（顶点属性禁用 UNSIGNED_INT 的约束）：https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.html#accessors
- Assimp（本地 ThirdParty/package/assimp v6.0.5）— `include/assimp/metadata.h`、`code/AssetLib/FBX/FBXConverter.cpp::SetupNodeMetadata`、`code/AssetLib/IFC/IFCLoader.cpp` 属性集转换
- 本仓先例 — `TerrainConverter/ARCHITECTURE.md`；模块说明见根 `CLAUDE.md`

**业界调研（§3.5 依据，2026-09 检索）**：

- SuperMap S3M 属性存储三模式 — 技术问答：https://ask.supermap.com/150221 （S3MD=JSON 行式 / ATTRIBUTE=二进制列式 / DB=S3M 3.01 大文件）；帮助文档：https://help.supermap.com/iDesktopX/zh/tutorial/Optimization/Cache/CADModelCache.html
- SuperMap S3MB 属性查询机制 — https://ask.supermap.com/135693 （attribute 模式写入缓存；`S3MTilesLayer.indexedDBSetting.isAttributesSave` + `layer.getAttributesById(ID)`）
- SuperMap S3M 开放规范 — https://github.com/SuperMap/s3m-spec （含 S3MD.md 属性文件规范）
- CesiumLab 4 功能页 — http://www.cesiumlab.com/cesiumlab.html （BIM 切片"支持 Rvt/Dgn 导出插件、生成完整构件结构树、保留完整构件属性"；手工模型"支持属性外挂"）
- CesiumLab CIMRTS — http://www.cesiumlab.com/cimrts.html （RVT/DGN 直接入库或 CLM；属性字段/结构自定义、批量导入导出、3dtiles 内置属性完全自定义）；开源引擎：https://gitee.com/cesiumlab/cimrts-3dtiles （MIT）
- CesiumLab Revit→CLM 工作流教程 — https://www.jianshu.com/p/dd3f84c90534
- GISBox 官方文档/FAQ — https://www.gisbox.com.cn/docs/v1/x7orn31x80jl/ （IFC 切片）、https://www.gisbox.com.cn/docs/v1/xug2z9hshum0i47y/ （RVT 切片）、https://www.gisbox.com.cn/docs/v1/k5xt6gxw8ahn/ （"还没做属性的写入"）、https://www.gisbox.com.cn/docs/v1/me5wxtw8oiz7/ （BIM 构件绑数据"不支持"）、https://www.gisbox.com.cn/docs/v1/rvcfpsnzlbha/ （3DTiles再处理=几何再处理）

**分格式绑定机制调研（§3.5.1 依据，2026-09 追加）**：

- CesiumLab 系列教程（官方"西部世界"发布）— 人工模型/矢量面/BIM 处理参数：https://zhuanlan.zhihu.com/p/597292900 （分格式 id 来源：OBJ 随机、FBX=SDK UniqueId 的 MD5、CLM=Revit uuid；属性外挂 CSV **第一列唯一名称**与场景名称匹配）；通用模型处理：https://zhuanlan.zhihu.com/p/597316971
- CesiumLab 使用手册（CLM 中间格式 schema）— https://blog.csdn.net/q237895772/article/details/145450217 （CLM=SQLite 单文件 6 表；Models 表 id/name/type/props 字段）；https://www.scribd.com/document/758567333/ （同手册镜像）
- SuperMap "打开 BIM 数据"（RVT/3DXML）— https://support.supermap.com/DataWarehouse/WebDocHelp/iDesktop/Features/SceneOperation/DataProcessing/OpenBIM.html （**需本机安装 Revit**；"导出明细表"选项将 Revit 明细表导为属性表数据集）
- SuperMap Revit 导出插件使用说明 — https://blog.csdn.net/supermapsupport/article/details/107064455 ；GIS+BIM 对接流程：https://blog.csdn.net/THEDEAMON/article/details/104959909
- BimAngle（Revit 进程内插件直出 3D Tiles）— https://www.bimzyw.com/4013.html ；使用流程：https://jishuzhan.net/article/2002622298558365698

---

## 10. 附录：规范核对记录（v2.1）

| # | 设计中的声明 | 官方原文依据 | 核对结果 |
|---|---|---|---|
| 1 | b3dm 28B 头；`btJsonLen==0` 表示无批表且 `btBinLen` 必为 0 | Batched3DModel §Header | ✔ 与设计一致（现有实现保持旧布局合法） |
| 2 | `BATCH_LENGTH` 必填（uint32）；glTF 无 `batchId` 属性时**必须为 0** | Batched3DModel §Global semantics；b3dm.featureTable.schema.json `required:["BATCH_LENGTH"]` | ✔ 现有硬编码 `{"BATCH_LENGTH":0}` 合法，兼容旁路成立 |
| 3 | `RTC_CENTER` 可选 `float32[3]`（本管线未使用，root transform 方案） | 同上 | ✔ 不引入 |
| 4 | 批表存在或 BATCH_LENGTH>0 时**所有** primitive 需 `_BATCHID`；accessor `type:"SCALAR"` | Batched3DModel §Binary glTF | ✔ 印证 D6/R5；v1 未提 type 约束，已补 |
| 5 | batchId ∈ [0, BATCH_LENGTH-1]；顶点可不按 batchId 排序；一顶点仅属一模型 | 同上 | ✔ 实例级分配天然满足（R6） |
| 6 | FT/BT 的 JSON 段须以 `0x20` 补齐至**结束于 8 字节边界**；二进制体起止均在 8B 边界；glb 起始 8B；tile 总长 8B 对齐 | FeatureTable §Padding；BatchTable §Padding；Batched3DModel §Padding | ⚠ **v1 有误**（写成 JSON 4B 补齐）→ §4.3-⑤ 已改正。现有 FT JSON=20B 恰满足（28+20=48） |
| 7 | BT 二进制引用对象为 `{byteOffset, componentType, type}` 三元组；`byteOffset` 按分量类型大小（1/2/4/8B）对齐即可（不必统一 8B）；componentType 8 种、type SCALAR/VEC2/3/4 | BatchTable §JSON header / §Padding | ⚠ v1 示例缺 `type` 字段、列对齐过严 → §4.3-⑥、§4.4 已改正 |
| 8 | BT JSON 数组元素允许任意 JSON 类型（含 null/对象/数组），长度 == BATCH_LENGTH；字符串只能 JSON 表示（二进制无 STRING 类型） | BatchTable §JSON header | ✔ 缺失值由 `""` 改为 `null`（§4.3-⑥） |
| 9 | glTF 顶点属性 accessor 禁用 `UNSIGNED_INT(5125)`；>2^16 特征用 `FLOAT`（整数精度至 2^24） | EXT_mesh_features §Feature ID by Vertex（Implementation note，引用 glTF 2.0 accessor 约束） | ⚠ **v1 有误**（降档表含 5125）→ §4.3-④ 已改正 |
| 10 | b3dm / Feature Table / Batch Table 在 3D Tiles 1.1 标注 deprecated；官方迁移指南指定 `_BATCHID`/批表 → `EXT_mesh_features`+`EXT_structural_metadata` 承接 | 三个 README 顶部 WARNING；MIGRATION.adoc §b3dm | ✔ §3.3 对比表已更新（P2 路线即官方路线） |
| 11 | propertyTable 的 `values`/`arrayOffsets`/`stringOffsets` 为 **bufferView 索引（非 accessor）**；STRING = UTF-8 字节流 + N+1 个 stringOffset（默认 `UINT32`）；BOOLEAN 按位打包 `ceil(N/8)` 字节 | propertyTable.schema.json；propertyTable.property.schema.json；3D Metadata §Binary Table Format | ✔ 已写入 §4.7；与 §4.3 批表实现共享"行→列"抽象 |
| 12 | schema/class/property/enum **ID 正则** `^[a-zA-Z_][a-zA-Z0-9_]*$`；中文名放 `name`；FLOAT 值禁 NaN/±Inf | 3D Metadata §Identifiers/§Property；EXT_structural_metadata §Property Tables | ✔ §4.3-⑧ 新增净化与过滤规则 |
| 13 | 两扩展仅登记 `extensionsUsed`（规范建议 optional，不写 extensionsRequired）；采用后 GLB JSON/BIN chunk 补齐提升到 8B | EXT_mesh_features §Optional vs. Required；EXT_structural_metadata §Binary Data Storage | ✔ §4.7 |
| 14 | tileset.json 原生 `schema`/`schemaUri`（互斥）、`statistics`、`groups`、顶层 `metadata`；1.0 时代 `properties` 已废弃 | specification/schema/tileset.schema.json | ✔ **O1 从"开放问题"改为"已解决"**；P2 用单一外链 schema.json |
| 15 | 拾取统一 API：`getProperty/getPropertyIds/hasProperty/setProperty`、`featureId`；`show/color` 随瓦片卸载失效；`getPropertyInherited` 查找顺序（批表→content→tile→group→tileset） | Cesium3DTileFeature ref-doc | ✔ §4.4/§5 示例与 UI 注意事项 |
| 16 | 本地自带 Cesium.js 1.111 支持两套编码；旧 API `getPropertyNames` 弃用 | MGOServer/public/cesium/Cesium.js grep（getPropertyIds×8、getPropertyNames×1、EXT_structural_metadata×3、EXT_mesh_features×2、B3dmParser 存在） | ✔ 本地实测 |
| 17 | FBX 元数据/链名、IFC Psets→节点 metadata、`$AssimpFbx$` 链尾承载 metadata+几何 | 本地 assimp v6.0.5 源码（FBXConverter.cpp:296-316/743-745/926-976；IFCLoader.cpp:654-678） | ✔ v1 已核，v2 无改动 |
| 18 | **FBX 重名节点自动去重**：追加 3 位零填充数字（`Wall001`…），空名回退祖先基础名 | 本地 assimp `FBXConverter::GetUniqueName/MakeUniqueNodeName`（FBXConverter.cpp:226-243/2228-2242） | ⚠ v1 未考虑 → §3.2 L2 已补（sidecar 用导入后名或 `--bim-report` 对账） |
| 19 | **IFC Pset 合并无 set 名前缀**（跨 set 同名属性覆盖）、**数值全部字符串化** | 本地 assimp `ProcessMetadata`（IFCLoader.cpp:561-616，调用点 661-668 空前缀） | ⚠ v1 "Psets 直接可用"过强 → §3.1/§4.8 已补类型恢复与 P3 对策 |
| 20 | glTF2 `extras` 落到 `aiNode::mMetaData` | 本地 assimp `glTF2Importer.cpp:1130-1155 ParseExtras` | ✔ L1 三格式成立 |
| 21 | Batch Table 二进制列为定长数组 → **无法表达缺失值**（无 null 语义），JSON 数组形式允许 null | BatchTable §JSON header 两形式对比 | ✔ 列三态规则入 §4.8/§4.3-⑥（v1 未明说） |
| 22 | **Content schema 共享机制 = glb 级 `EXT_structural_metadata.schemaUri`**（README 原文："Multiple glTF assets may refer to the same external schema"）；propertyTable 的 class 须在 glb 自带/外链 schema 中声明；tileset 级 `schema`/`schemaUri` 是 tileset/tile/group 实体元数据的机制 | EXT_structural_metadata README §Schema + propertyTable.schema.json（"declared in the classes dictionary"）；本地实测 1.111 index.js 含两套 schemaUri 解析路径（`_baseResource` 与 content-derived） | ⚠ **v2 曾误将 tileset.schemaUri 当作 content propertyTables 的类定义来源** → §4.7/O1/P2 已修正（v2.1） |
| 23 | `Cesium3DTileFeature.featureId` 在 1.111 双实现（批表内容映射 `_batchId`；EXT_mesh_features 内容映射 `_featureId`） | 本地 index.js grep：`featureId:{get:function(){return this._batchId}}` 等 | ✔ §4.4 示例注释成立（1.111 可读） |
| 24 | GlbBuilder 现布局：每 primitive 4 bufferView（pos/norm/uv/idx），`bvBase/accBase = pi*4`；**images bufferView 后置**（`imageBVStart`） | TileBuilder.cpp:378-431 本地核实（v2 原引 ":388" 为近似行号） | ✔ §4.3-④ 已按实际布局改写并补 images 移位警示 |
| 25 | SuperMap S3M 属性存储三模式（S3MD=JSON 行式 / ATTRIBUTE=二进制列式 / DB=S3M 3.01）及各自查询优势 | ask.supermap.com/150221（官方回答）+ help.supermap.com CADModelCache 帮助页 | ✔ §3.5 印证双编码路线 |
| 26 | CesiumLab BIM 切片"保留完整构件属性 + 生成完整构件结构树"；手工模型"属性外挂"；CIMRTS 属性完全自定义 | cesiumlab.com/cesiumlab.html + cimrts.html 官方功能页 | ✔ §3.5；结构树输出记为 P3 候选 |
| 27 | GISBox（RVT/IFC 直接切片）**不写入任何属性**；"3DTiles再处理"仅几何再处理 | gisbox.com.cn 官方 FAQ 三条（k5xt6gxw8ahn / me5wxtw8oiz7 / rvcfpsnzlbha，2025-08/10 截图佐证） | ✔ §3.5 反例：直读格式 ≠ 属性绑定 |
| 28 | CesiumLab 分格式 id 来源：OBJ=随机生成；FBX=FBX SDK UniqueId 的 MD5；CLM=Revit 构件 uuid；SHP=name 字段；默认双字段 id+name | CesiumLab 官方教程（知乎 p/597292900，"地球可视化实验室"发布） | ✔ §3.5.1 矩阵；assimp 拿不到 FBX UniqueId 的差异已如实标注 |
| 29 | CesiumLab 属性外挂连接键 = **名称**："CSV 第一列必须是唯一名称，第一列的值和模型场景里的名称关联匹配"（UTF-8、英文逗号） | 同上教程（xfyun.csdn.net 镜像摘要交叉验证） | ✔ §3.5.1 模式③；与本设计 L2② 名称键 + sidecar 同构，但本设计多一层 GUID 优先 |
| 30 | CLM 中间格式 = SQLite 单文件 6 表，Models 表含 id/name/type/props(JSON)；构件节点 id 必须与构件 id 一致（结构树与属性以 id 内聚） | CesiumLab 使用手册（CSDN q237895772/145450217 + Scribd 镜像） | ✔ §3.5.1 模式②；印证"结构树+属性需同源 id"的设计要点 |
| 31 | SuperMap "打开 BIM 数据"（RVT）**需要本机安装 Revit**（另提供"导出明细表"→属性表数据集选项） | SuperMap iDesktop 帮助（OpenBIM 页，GBK 原文解析） | ✔ §3.5.1 RVT 行：SuperMap 全线依赖"Revit 在场"，无直读 |
| 32 | **实现层验证（v2.4）**：`_BATCHID` 组件档与 count；批表三态列；b3dm 全段 8B 对齐 + 头字段一致性；关闭开关与基线逐字节一致（b3dm+tileset.json）；OBJ sidecar 12/12 命中、glTF2 extras 12/12 命中、per-tile 空行回退与旧路径逐字节一致 | 本轮实现：`test_batchtable`/`test_bimstrategy` 单测 + E2E（HEAD 基线二进制对比、Python 独立解码 b3dm/glb 断言、报告 JSON 解析） | ✔ §7 实施状态注记；E2E 解码器独立于 C++ 写入器（防同源盲区） |
| 33 | **实现层偏差记录（v2.4）**：① `--bim-schema`/`--bim-key`/`--bim-feature-granularity` 未实现（§4.5 列出但属后续轮次）；② sidecar 数值单元按保守规则类型化（纯整→Int32/64、含 `.`/`e`→Double、true/false→Bool），**字符串外观的数字不做提升**（与 §4.8(3) 一致）；③ 空行/全空列产生 `{}` 批表而非省略段（btJsonLen>0 但内容为 `{}`，合法且消费端无害）；④ FT JSON 填充至 8B 的位置在 FT 段内（BATCH_LENGTH:N + 0x20），与基线 20B 版本（BATCH_LENGTH:0 + 2 空格）格式相同仅长度不同 | 本轮实现代码（BatchTableWriter/B3dmBuilder/SidecarTableSource::ParseCell） | ✔ §4.3 实现实况；②④ 为有意决策 |

**未验证项（诚实声明）**：① 3d-tiles-tools `validate` 对"带批表 b3dm"与"EXT_structural_metadata glb"的具体检查深度未实测（P0/P2 验收时本地跑一次即可）；② CesiumJS 1.111 对 `featureIds.nullFeatureId` 的拾取行为细节未实测（仅影响 §4.7 的 nullFeatureId 优化，P2 前补测）。
