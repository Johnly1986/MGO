# MGO — Mesh Generation Optimizer

[![Build & Test](https://github.com/Johnly1986/MGO/actions/workflows/build.yml/badge.svg)](https://github.com/Johnly1986/MGO/actions/workflows/build.yml)
[![License: Apache-2.0](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)

MGO is a cross-platform C++17 toolkit for 3D mesh optimization, coordinate projection, and 3D Tiles conversion — targeting digital twin, GIS, and reality-modeling workflows.

It loads 3D models via Assimp, simplifies meshes with meshoptimizer, transforms coordinates via PROJ, and converts them into web-friendly streaming formats:

```
mgo mesh     - mesh simplification + coordinate projection
mgo tiles    - FBX/OBJ -> 3D Tiles (b3dm + tileset.json)
mgo terrain  - GeoTIFF DEM -> Cesium Quantized-Mesh terrain tiles
mgo image    - DOM orthophoto -> TMS image tiles
mgo geojson  - GeoJSON projection conversion
mgo osgb     - OSGB oblique photography (DJI Terra / ContextCapture) -> 3D Tiles
```

Everything is driven by one CLI, `MGOConsole`, and built as a set of reusable shared libraries.

## Requirements

- CMake ≥ 3.18, C++17 compiler (MSVC 2022 / GCC 9+ / Clang 10+)
- [vcpkg](https://github.com/microsoft/vcpkg) for dependency management (versions pinned in `vcpkg.json`)
- Assimp v6.0.5 is fetched and built from source automatically; meshoptimizer v1.2 is vendored in-tree

## Quick Start

### 1. Install vcpkg (once)

```bash
# Linux / macOS
git clone https://github.com/microsoft/vcpkg.git ~/vcpkg
cd ~/vcpkg && ./bootstrap-vcpkg.sh
```

```powershell
# Windows
git clone https://github.com/microsoft/vcpkg.git C:\vcpkg
cd C:\vcpkg && .\bootstrap-vcpkg.bat
```

### 2. Configure & build

With the vcpkg toolchain, CMake installs missing dependencies automatically:

```bash
# Linux / macOS
cmake -B build \
    -DCMAKE_TOOLCHAIN_FILE=~/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build build
```

```powershell
# Windows
cmake -B build -A x64 `
    -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build build --config Release
```

The OSGBConverter module needs OpenSceneGraph, which takes a while to build. To skip it: `-DMGO_WITH_OSG=OFF`.

Or simply use the bundled Makefile on Linux/macOS:

```bash
make release    # Release build
make debug      # Debug build
make test       # Build + run unit tests
```

### 3. Run

```bash
./build/bin/MGOConsole help
```

Generate a synthetic test GeoTIFF and convert it to terrain tiles:

```bash
python3 Script/Test/generate_test_tif.py test_terrain.tif
./build/bin/MGOConsole terrain -i test_terrain.tif -o terrain_output -v
```

## Modules

| Library | Purpose |
|---------|---------|
| `libMeshGroupOptimizer` | Mesh simplification via vendored meshoptimizer v1.2 |
| `libMeshProjectionErrorCorrector` | PROJ + Eigen coordinate transforms, georeferencing (7-parameter Helmert, anchor, multi-control-point least squares), octree spatial indexing |
| `libTileBuilder` | 3D Tiles binary format builders: GlbBuilder, B3dmBuilder, MaterialGrouper, TilesetWriter |
| `libTilesConverter` | Assimp scene -> 3D Tiles (b3dm + tileset.json), sparse-grid LOD hierarchy |
| `libTerrainConverter` | GeoTIFF -> Quantized-Mesh terrain tiles, parallel tile processing |
| `libImageTiler` | DOM orthophoto -> Web Mercator TMS image tiles |
| `libGeoJSONConverter` | GeoJSON projection conversion (round-trip-safe JSON parser) |
| `libOSGBConverter` | OSGB oblique photography (DJI Terra / ContextCapture) -> 3D Tiles |

## CLI Reference

### mgo mesh — Mesh Optimization

```
mgo mesh -i model.fbx -o output.obj -e 0.01 [options]
```

| Flag | Description | Default |
|------|-------------|---------|
| `-i <file>` | Input 3D model (FBX, OBJ, etc.) | required |
| `-o <file>` | Output file | required |
| `-e <val>` | Simplification error (0 = disabled) | 0.01 |
| `-n <val>` | Normal weight | 0.1 |
| `-t <val>` | Threshold (0 = error-driven, >0 = ratio of original triangles) | 0 |
| `-L` | Lock border vertices | off |
| `-l` | Local/absolute error mode | off |
| `-c <csv>` | Per-mesh config CSV | - |
| `-p <file\|spec>` | Projection: `.prj` file, or inline CRS (`EPSG:<code>` / WKT / `+proj=...`) | - |
| `--cps <csv>` | Control points CSV (sx,sy,sz,tx,ty,tz) | - |
| `-C <mode>` | Output coordinate system (original / left) | original |
| `-g <mode>` | Georeferencing: 7param / multipos / anchor | 7param |
| `--7p <mx,my,mz,rx,ry,rz,s>` | 7-parameter Helmert (translation m; rotation arc-sec; scale ppm) | required (7param) |
| `--offset x,y,z` | Projection offset in source CRS (meters) | 0,0,0 |
| `--fit-order` | Polynomial fit order for multipos (1/2/3) | 1 |
| `--auto-crs` | Auto-detect source CRS (multipos only) | off |
| `--proj-test` | Validate PROJ library and exit | - |

### mgo tiles — 3D Tiles Conversion

```
mgo tiles -i model.fbx -o output_dir [options]
```

| Flag | Description | Default |
|------|-------------|---------|
| `-i <file>` | Input 3D model | required |
| `-o <dir>` | Output directory | required |
| `-Z` | Input is Z-up (skip Y↔Z swap) | off |
| `-e <val>` | Root geometric error (meters) | 500 |
| `-t <val>` | Tile geometric error (meters) | 50 |
| `-r <mode>` | Refine mode (ADD / REPLACE) | ADD |
| `--max-lod <N>` | Max LOD levels | 5 |
| `--prj <file\|spec>` | Projection: `.prj` file, or inline CRS (`EPSG:<code>` / WKT / `+proj=...`) | - |
| `--origin <x,y,z>` | Coordinate origin (projected CRS) | 0,0,0 |
| `--7p <mx,my,mz,rx,ry,rz,s>` | 7-parameter Helmert transform | - |
| `-s` | Enable mesh simplification | off |
| `--error <val>` | Simplification error | 0.01 |
| `--nweight <val>` | Normal weight | 0.1 |
| `--threshold <val>` | Threshold (0 = error-driven, >0 = ratio cap) | 0 |
| `--no-lock-border` | Disable border vertex locking | off |

Models spanning more than 3 km automatically enable per-vertex projection correction to eliminate tangent-plane curvature residual.

### mgo terrain — Terrain Tiles

```
mgo terrain -i dem.tif -o output_dir [options]
```

| Flag | Description | Default |
|------|-------------|---------|
| `-i <file>` | Input GeoTIFF (single-band float32) | required |
| `-o <dir>` | Output directory | required |
| `--prj <file\|spec>` | Override projection: `.prj` file, or inline CRS (`EPSG:<code>` / WKT / `+proj=...`) | from TIF |
| `--origin <x,y,z>` | Override TIF tiepoint (projected CRS) | from TIF |
| `--max-lod <N>` | Max LOD level | auto from pixel size |
| `--samples <N>` | Samples per tile edge (must be odd) | 65 |
| `--error <val>` | Simplification error (normalized [0,1]) | 0.001 |
| `--nweight <val>` | Normal weight | 0.0 |
| `--no-lock-border` | Disable border vertex locking | off |
| `--no-normals` | Skip OctVertexNormals extension | off |
| `-v` | Verbose output | off |

Output format: Cesium quantized-mesh-1.0 (`{z}/{x}/{y}.terrain` + `layer.json`). Tiles are processed in parallel (up to 8 threads).

### mgo image — DOM Tiling

```
mgo image -i ortho.tif -o output_dir [options]
```

| Flag | Description | Default |
|------|-------------|---------|
| `-i <file>` | Input GeoTIFF (RGB) | required |
| `-o <dir>` | Output directory | required |
| `--prj <file\|spec>` | Override projection: `.prj` file, or inline CRS (`EPSG:<code>` / WKT / `+proj=...`) | from TIF |
| `--min-zoom <N>` | Min zoom level | auto |
| `--max-zoom <N>` | Max zoom level | auto |

Output: Web Mercator TMS tile pyramid (`{z}/{x}/{y}.png` + `layer.json` + `tilemapresource.xml`). Uses an affine approximation for tile rendering — 4 PROJ calls per tile instead of 65,536.

### mgo geojson — GeoJSON Projection Conversion

```
mgo geojson -i input.geojson -o output.geojson [options]
```

| Flag | Description | Default |
|------|-------------|---------|
| `-i <file>` | Input GeoJSON file | required |
| `-o <file>` | Output GeoJSON file | required |
| `--source-crs <crs>` | Source CRS | `crs` member or EPSG:4326 |
| `--target-crs <crs>` | Target CRS | EPSG:4326 |
| `--pretty` | Pretty-print output | off |

CRS forms: `EPSG:<code>` | `ENU:<lat>,<lon>[,<h>]` | WKT | `+proj=...` | `<file.prj>`. Per-vertex coordinates are projected; untouched numbers round-trip byte-exact.

### mgo osgb — OSGB to 3D Tiles

```
mgo osgb -i osgb_root_dir -o output_dir [options]
```

| Flag | Description | Default |
|------|-------------|---------|
| `-i <dir>` | OSGB root directory (contains metadata.xml) | required |
| `-o <dir>` | Output directory | required |
| `--prj <file\|spec>` | Projection: `.prj` file, or inline CRS (`EPSG:<code>` / WKT / `+proj=...`) | from metadata.xml |
| `--enu <lat,lon[,h]>` | ENU tangent plane override (bypasses `--prj`) | from metadata.xml |
| `--origin <x,y,z>` | Override coordinate origin | from metadata.xml |
| `--max-lod <N>` | Max LOD level | auto |
| `--refine <mode>` | ADD or REPLACE | REPLACE |
| `-s` | Enable mesh simplification | off |
| `--error <val>` | Simplification error | 0.01 |
| `--nweight <val>` | Normal weight | 0.1 |
| `--threshold <val>` | Threshold (0 = error-driven, >0 = ratio cap) | 0 |
| `--no-lock-border` | Disable border vertex locking | off |
| `-v` | Verbose output | off |

Requires OpenSceneGraph (`MGO_WITH_OSG=ON`, on by default). Supports DJI Terra (`Block_*` directories) and ContextCapture (`Data/Tile_*` directories).

### Exit Codes

All subcommands return a uniform exit code:

| Code | Meaning |
|------|---------|
| `0` | Success |
| `1` | Conversion failure (I/O, projection, PROJ error, ...) |
| `2` | Usage error (missing/invalid argument, unknown option) |

## Mesh Simplification

All geometry-processing modules use the unified `SimplifyOptions` struct (vendored meshoptimizer v1.2):

```cpp
struct SimplifyOptions {
    float error        = 0.0f;   // 0 = disabled, >0 = enabled
    float normalWeight = 0.1f;   // Normal attribute weight
    float threshold    = 0.0f;   // 0 = error-driven, >0 = ratio cap on triangle count
    bool  lockBorder   = true;   // Lock border vertices to prevent tile-edge cracks
    bool  localError   = false;  // Use absolute/local error metric
};
```

meshoptimizer v1.2 additions in this tree: `meshopt_SimplifySparse` (sparse heightmaps), `meshopt_SimplifyRegularize` (reduced sliver triangles), `meshopt_SimplifyVertex_Protect` (attribute discontinuity protection).

## Georeferencing

All modules support three projection strategies through a unified API:

```cpp
CProjectionEngine engine;

// 7-parameter Helmert datum shift
GeoreferencingWith7Parameters georef(srcCRS, "EPSG:4979");
georef.SetParameter(SevenParameter(mx, my, mz, rx, ry, rz, scale));
georef.Solve();
engine.SetGeoreferencing(&georef);

// Multi-control-point least squares
GeoreferencingWithMultiPosition georef(srcCRS, "EPSG:4979");
georef.SetFitMethod(FitMethod::DirectPoly2D);   // or FitMethod::ECEF_Affine
georef.SetParameter(controlPoints);
georef.Solve();
engine.SetGeoreferencing(&georef);

// GK projection (no georeferencing needed)
engine.LoadProjection("cgcs2000.prj");
engine.SetOrigin(easting, northing, height);
Eigen::Matrix4d transform = engine.ComputeRootTransform();  // column-major ENU->ECEF
```

## Testing

```bash
# Unit tests (executables are under build/bin/)
./build/bin/test_georef       # IGeoreferencing: 7-param, anchor, multi-position
./build/bin/test_osgb_unit    # Parser, vendor handler, data structures
./build/bin/test_boundary     # Boundary values across all modules

# Synthetic terrain data + verification
python3 Script/Test/generate_test_tif.py test_terrain.tif
python3 Script/Test/full_spec_verify.py <output_dir>

# PROJ validation
./build/bin/MGOConsole --proj-test
```

## Project Structure

```
MGO/
├── MGOConsole/           Unified CLI (all subcommands)
├── MeshGroupOptimizer/   Mesh simplification (vendored meshoptimizer v1.2)
├── MeshProjectionErrorCorrector/  PROJ transforms, georeferencing, octree, shared utilities
├── TileBuilder/          3D Tiles binary builders (GlbBuilder, B3dmBuilder, TilesetWriter)
├── TilesConverter/       Assimp -> 3D Tiles pipeline
├── TerrainConverter/     GeoTIFF -> Quantized-Mesh terrain pipeline
├── DOMConverter/         DOM orthophoto -> TMS image tiles (ImageTiler)
├── OSGBConverter/        OSGB oblique photography -> 3D Tiles
├── Script/               Verification and utility scripts
└── CMakeLists.txt        Unified CMake build (cross-platform)
```

## Build Options

| Option | Default | Description |
|--------|---------|-------------|
| `MGO_WITH_OSG` | ON | Build OSGBConverter (requires OpenSceneGraph) |
| `MGO_ASSIMP_SOURCE_DIR` | (auto) | Custom assimp source directory |
| `CMAKE_BUILD_TYPE` | Release | Debug / Release |

## Dependencies

All dependencies are managed via vcpkg + `vcpkg.json` with a pinned builtin-baseline, so all platforms get identical versions.

| Library | Minimum version | Notes |
|---------|-----------------|-------|
| CMake | 3.18 | `cmake_minimum_required` |
| Boost (regex) | 1.83.0 | `find_package(Boost 1.83.0)` |
| PROJ | 9.0 | runtime `proj_info()` check |
| Eigen3 | 3.4.0 | `find_package(Eigen3 3.4.0)` |
| Assimp | 6.0.5 | `FetchContent` source build |
| GDAL | 3.8.0 | `find_package(GDAL 3.8.0)` |
| libtiff | - | `find_package(TIFF)` |
| libjpeg-turbo | - | `find_package(JPEG)` |
| libpng | - | `find_package(PNG)` |
| zlib | 1.3.0 | `find_package(ZLIB 1.3.0)` |
| OpenSceneGraph | 3.6.5 | `find_package(OpenSceneGraph 3.6.5)` |
| meshoptimizer | v1.2 vendored | `add_library(meshoptimizer STATIC)` |

## License

Copyright Johnlyon. Licensed under the [Apache License 2.0](LICENSE).

Third-party component licenses are acknowledged in [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
