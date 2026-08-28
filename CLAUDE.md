# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

MGO (Mesh Generation Optimizer) is a cross-platform C++ toolkit for 3D mesh optimization, coordinate projection, and 3D-tile conversion — targeting digital twin / 3D reality modeling workflows. It loads 3D models via Assimp, simplifies meshes using meshoptimizer, transforms coordinates via PROJ, and exports to 3D Tiles (b3dm + tileset.json) and Cesium Quantized-Mesh terrain tiles.

## Build System

Cross-platform CMake build (≥ 3.18, C++17). Single root `CMakeLists.txt` + `vcpkg.json` for dependency management.

```bash
make release        # Build Release
make debug          # Build Debug
make test           # Build + run unit tests
make clean          # Remove build directory
```

## Project Modules

| Module | Type | Purpose |
|--------|------|---------|
| **MGOConsole** | Console EXE | Unified CLI — `mgo mesh|tiles|terrain|image|osgb` subcommands |
| **MeshGroupOptimizer** | Shared lib | Mesh simplification engine using vendored meshoptimizer v1.2 |
| **MeshProjectionErrorCorrector** | Shared lib | Coordinate system transformation using PROJ + Eigen; 7-param / anchor / multi-control-point georeferencing, plus CProjectionEngine / Octree / TileDataTypes |
| **TileBuilder** | Shared lib | 3D Tiles binary format: GlbBuilder, B3dmBuilder, MaterialGrouper, BBoxUtils, TilesetWriter |
| **TilesConverter** | Shared lib | Assimp scene → 3D Tiles (b3dm + tileset.json) using sparse grid Octree partitioning |
| **TerrainConverter** | Shared lib | GeoTIFF → Cesium Quantized-Mesh terrain tiles |
| **DOMConverter** | Shared lib | DOM orthophoto → TMS image tiles (ImageTiler) |
| **OSGBConverter** | Shared lib | OSGB oblique photography (DJI Terra / ContextCapture) → 3D Tiles; optional, requires OpenSceneGraph |

> **Note**: MeshProjection was renamed to **MeshProjectionErrorCorrector**. CProjectionEngine, TileDataTypes, Octree, AxisMapper, and GeodeticMath all live in `MeshProjectionErrorCorrector/`. RouteAnalysisAdpter and RouteAnalysisAdpterMain were deprecated and removed.

## Key Dependencies

- **Assimp** v6.0.5 — Built from source via CMake FetchContent; 3D model import/export
- **meshoptimizer** v1.2 — Vendored in `MeshGroupOptimizer/meshoptimizer/`
- **Boost** (regex) — Regex matching for per-mesh config
- **PROJ** — Coordinate system transformations and ECEF conversion
- **Eigen3** — Linear algebra (transform matrices, Octree spatial queries)
- **GDAL** + **libtiff** + **libgeotiff** + **zlib** — GeoTIFF reading (TerrainConverter)
- **libjpeg** + **libpng** — Texture encoding (OSGBConverter, ImageTiler)
- **OpenSceneGraph** ≥ 3.6.5 — OSGB file reading (OSGBConverter, optional)

All dependencies (except vendored meshoptimizer and FetchContent assimp) are managed via vcpkg (`vcpkg.json`) with a pinned `builtin-baseline`.

## Architecture

### Layer 1 — Core Libraries

**MeshGroupOptimizer** (`CMeshGroupOptimizer`):
- Pipeline: `Load(filename)` → `Optimize(config)` → `Save(filename)`
- Load uses Assimp with triangulation + smooth normals + bounding boxes
- Optimize matches mesh names against regex patterns in `OptimizerConfig.items`, calls `meshopt_simplifyWithAttributes`
- Internal: `MergeVertices` (meshopt_generateVertexRemapMulti), `KeepEffectiveVertices` (meshopt_optimizeVertexFetchRemap)
- Save exports via Assimp with optional left-hand coordinate conversion

**MeshProjectionErrorCorrector** (`CMeshProjectionErrorCorrector`):
- Entry: `Transform(georeferencing, scene, offsetX, offsetY, offsetZ)`
- Pipeline: flatten scene hierarchy → per-vertex: Assimp world space + offset → ZY-swap → source CRS → target CRS (EPSG:4979) → ECEF (EPSG:4978) → tangent-plane transform back to local → ZY-swap − offset
- Georeferencing strategies (all implement `IGeoreferencing`):
  - `GeoreferencingWith7Parameters` — Helmert 7-parameter (mx, my, mz, rx, ry, rz, scale)
  - `GeoreferencingWithAnchor` — Single anchor-point ECEF offset
  - `GeoreferencingWithMultiPosition` — Multi-control-point least-squares solve
- Shared utilities used by TilesConverter/TerrainConverter:
  - `CProjectionEngine` — GKInverse, GeographicToECEF, ENUToECEFRotation, ComputeRootTransform, ComputeInstanceProjectionDelta
  - `GeodeticMath` — Pure math: GKInverse, GeographicToECEF, ECEFToGeographic, ENUToECEFRotation
  - `AxisMapper` — Assimp↔ENU↔Tiles axis conversions
  - `Octree` — Spatial indexing with AABB queries
  - `TileDataTypes` — MeshInstance, MergedMeshGroup, BinaryBlob, GridCell

### Layer 2 — 3D Tiles & Terrain Pipeline

**TilesConverter** (Assimp → 3D Tiles):
- Pipeline: `CollectMeshInstances()` (flatten scene, bake world transforms + bboxes) → `BuildGridHierarchy()` (bottom-up sparse grid Octree aggregation, LOD-aware) → `TilesetWriter` writes b3dm + tileset.json
- Uses `CProjectionEngine` for per-instance projection correction + root transform
- Uses `TileBuilder` for binary format construction (GlbBuilder, B3dmBuilder, MaterialGrouper)
- Simplification via meshoptimizer (optional, per TilesConverterOptions flags)
- Output: b3dm files organized in LOD directories + `tileset.json`

**TerrainConverter** (GeoTIFF → Quantized-Mesh terrain tiles):
- Pipeline: `GeoTiffReader` → `HeightmapGrid` → `TerrainQuadtree` (Cesium geographic quadtree) → `TinSimplifier` (meshopt_simplifyWithAttributes, border-locked) → `QuantizedMeshEncoder` → `.terrain` files + `layer.json`
- Full architecture design doc: `TerrainConverter/ARCHITECTURE.md`
- CLI: `mgo terrain -i <input.tif> -o <outputDir> [--max-lod N] [--error m] [--samples N]`

### Coordinate System Conventions

This is a critical, error-prone area of the codebase:

- **Assimp imports as Y-up** by default (X = east, Y = up, Z = south — standard right-handed glTF convention)
- **Some FBX files are Z-up** (e.g., M1 test data has "文件朝向：Z轴" in config_info.txt — `config_info.txt` signals from the asset author that the file was pre-rotated to Z-up, so Y↔Z swap should NOT be applied)
- **MGOConsole projection**: Per-vertex transforms go through Assimp Y-up → ZY-swap → source CRS → ECEF → tangent plane → ZY-swap back → Assimp Y-up
- **TilesConverter**: Assimp Y-up input → CProjectionEngine → 3D Tiles Z-up output (CesiumJS applies `Y_UP_TO_Z_UP` automatically — North-negation is NOT encoded)
- **TerrainConverter**: TIF projected coords → GKInverse to geographic → GeographicToECEF → quantized-mesh encoding
- **Custom types**: `Vector3`/`Vector3D` (float/double), `Vector2`/`Vector2D`
- **Encoding**: GBK used for file paths on Windows; Boost locale converts between GBK and UTF-8

Coordinate-system bugs here are subtle and have regressed before (Inverse matrix, YZ-swap ordering, anchor transform). When changing projection code, run the `test_georef` regression checks.

## Testing

**Unit tests** run via `make test`:
- `test_georef` - IGeoreferencing: 7-param, anchor, multi-position, regression
- `test_osgb_unit` - Parser, vendor handler, data structures
- `test_boundary` - Boundary values across all modules

**TerrainConverter verification scripts** live in `Script/Test/`:
- `generate_test_tif.py` - creates synthetic test GeoTIFF
- `full_spec_verify.py`, `geometric_verify.py`, `lod_verify.py`, `degenerate_check.py`, `area3d_check.py` - output verification (take the output dir as first argument)

Private regression test data (models, `.prj` files) is NOT in the repository - it lives in the maintainer's local `Data/Test/` folder (gitignored).

## Code Conventions

- C++17
- DLL API exported via `__declspec(dllexport)` / `__attribute__((visibility("default")))` macros in each module's `macro.h`
- Each module has its own `macro.h` with API export macro and (on MSVC) `#pragma comment(lib, ...)` for auto-linking
- `-fvisibility=hidden` used on Linux/macOS; only `XXX_API`-marked symbols are exported
- Object files and build outputs go under `build/` (gitignored)
