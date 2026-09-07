# MGO - Mesh Generation Optimizer

[![License: Apache-2.0](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/platform-Windows%20%7C%20Linux-lightgrey.svg)](#)
[![Build & Test](https://github.com/Johnly1986/MGO/actions/workflows/build.yml/badge.svg)](https://github.com/Johnly1986/MGO/actions/workflows/build.yml)

MGO is a cross-platform (Windows / Linux) C++17 command-line toolkit that turns engineering-grade 3D models and geospatial raster data into web-streamable 3D Tiles - built for digital twin, BIM+GIS integration, and reality-modeling workflows.

## Why MGO

If you have tried to put design-stage engineering models or survey data on a Cesium / 3D Tiles globe, you have probably hit the same walls:

- **Format silos.** Design tools export FBX/OBJ with no georeference; oblique-photography vendors ship OSGB that only their own viewer reads. Getting any of this into an open streaming format usually means expensive closed-source converters or hand-written scripts.
- **Coordinate systems are the real problem.** Engineering models live in a local projected CRS (e.g. CGCS2000 Gauss-Krüger); the globe needs ECEF. Along the way you must handle datum shifts (7-parameter Helmert), control-point fitting, Y-up vs Z-up conventions, and tangent-plane distortions on large sites - each one a silent-visual-offset bug waiting to happen.
- **DEM / DOM data doesn't stream.** GeoTIFF heightmaps and orthophotos must become Quantized-Mesh terrain and TMS tile pyramids before a browser can render them efficiently.
- **Models are too heavy.** Survey and BIM meshes routinely carry 10-100x more triangles than a web client can draw; simplification must be georeference-aware and crack-free across tile borders.

MGO solves all the aforementioned problems with a command-line interface (CLI). Hereinafter, MGO (MGOConsole) will be used as an alias for the main program:

```
mgo mesh     - mesh simplification + coordinate projection
mgo tiles    - FBX/OBJ -> 3D Tiles (b3dm + tileset.json)
mgo terrain  - GeoTIFF DEM -> Cesium Quantized-Mesh terrain tiles
mgo image    - DOM orthophoto -> TMS image tiles
mgo geojson  - GeoJSON projection conversion
mgo osgb     - OSGB oblique photography (DJI Terra / ContextCapture) -> 3D Tiles
```

Highlights:

- Three georeferencing strategies: 7-parameter Helmert, single anchor point, multi-control-point least-squares fitting (with automatic source-CRS detection).
- Per-vertex projection correction for large sites (>3 km) to eliminate tangent-plane curvature residual.
- Correct Y-up/Z-up handling per input format (glTF-style Y-up FBX vs pre-rotated Z-up FBX).
- Crack-free border-locked mesh simplification powered by a vendored, extended meshoptimizer.
- BIM property binding: per-format ID/attribute resolution and external ledger CSV join into the 3D Tiles Batch Table (`feature.properties`), with a per-instance transparency report and legacy encoding repair.
- Parallel terrain tile generation; Cesium-compatible `layer.json` / tileset output out of the box.

## Requirements

- Windows (MSVC 2022, tested) or Linux (GCC 9+, CI-tested on ubuntu-latest); macOS should work with the same vcpkg setup but is not regularly tested.
- CMake ≥ 3.18, C++17 compiler
- [vcpkg](https://github.com/microsoft/vcpkg) for dependency management (versions pinned in `vcpkg.json`)
- Assimp v6.0.5 is fetched and built from source automatically; meshoptimizer v1.2 is vendored in-tree

## Build

### 1. Install vcpkg (once)

```bash
# Linux
git clone https://github.com/microsoft/vcpkg.git ~/vcpkg
cd ~/vcpkg && ./bootstrap-vcpkg.sh
```

```powershell
# Windows
git clone https://github.com/microsoft/vcpkg.git C:\vcpkg
cd C:\vcpkg && .\bootstrap-vcpkg.bat
```

### 2. Configure & build

With the vcpkg toolchain, CMake installs missing dependencies automatically (Boost, PROJ, Eigen3, GDAL, libtiff, libjpeg-turbo, libpng, zlib, nlohmann-json, OpenSceneGraph):

```bash
# Linux
cmake -B build \
    -DCMAKE_TOOLCHAIN_FILE=~/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build build --config Release
```

```powershell
# Windows (Visual Studio generator)
cmake -B build -A x64 `
    -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build build --config Release
```

On Linux/macOS you can also use the bundled Makefile:

```bash
make release    # Release build
make debug      # Debug build
make test       # Build + run unit tests
```

Build options:

| Option | Default | Description |
|--------|---------|-------------|
| `MGO_WITH_OSG` | ON | Build OSGBConverter (requires OpenSceneGraph; slow to build - set `OFF` to skip) |
| `MGO_ASSIMP_SOURCE_DIR` | (auto) | Custom assimp source directory |

The `mgo` executable is `MGOConsole`; depending on platform and generator it lands in `build/bin/`, `build/bin/Release/` (VS generator), or `build/bin/Debug/`:

```bash
# Linux
./build/bin/MGOConsole help

# Windows (VS generator)
./build/bin/Release/MGOConsole.exe help
```

## Quick Tour (verified examples)

The commands below were tested end-to-end.

### Terrain from a synthetic GeoTIFF

```bash
python3 Script/Test/generate_test_tif.py   # writes Script/Test/test_terrain.tif (EPSG:4326 DEM)
./build/bin/MGOConsole terrain -i Script/Test/test_terrain.tif -o terrain_out -v
# -> terrain_out/{z}/{x}/{y}.terrain + layer.json (Cesium quantized-mesh-1.0)
```

### Model to 3D Tiles (with georeference)

```bash
./build/bin/MGOConsole tiles -i roadbed.fbx -o tiles_out -Z \
    --prj cgcs2000_gk.prj
# -> tiles_out/tileset.json + L0/L1/.../tile_*.b3dm
```

`-Z` marks the input as Z-up (skips the Y↔Z swap). Drop it for standard Y-up FBX/OBJ exports.

### Simplify + reproject a mesh

```bash
./build/bin/MGOConsole mesh -i model.fbx -o model_out.obj -e 0.01
```

### GeoJSON reprojection

```bash
./build/bin/MGOConsole geojson -i input.geojson -o output.geojson \
    --source-crs EPSG:4547 --target-crs EPSG:4326
```

## CLI Reference

All subcommands share uniform exit codes: `0` success, `1` conversion failure (I/O, projection, PROJ error), `2` usage error. Run `mgo <subcommand> --help` for the authoritative option list; the tables below mirror it.

### mgo mesh - Mesh Optimization

```
mgo mesh -i model.fbx -o output.obj -e 0.01 [options]
```

| Flag | Description | Default |
|------|-------------|---------|
| `-i <file>` | Input 3D model (FBX, OBJ, etc.) | required |
| `-o <file>` | Output file | required |
| `-e <val>` | Simplification error (0 = disabled) | 0.01 |
| `-n <val>` | Normal (angular) weight for simplification | 0.1 |
| `-t <val>` | Threshold (0 = error-driven, >0 = ratio of original triangles) | 0.1 |
| `-r <val>` | Reorder optimization | - |
| `-R <val>` | Rebuild optimization | - |
| `-l <val>` | Use local/absolute error metric | off |
| `-L <val>` | Lock border vertices (prevents edge cracks) | off |
| `-c <csv>` | Per-mesh config CSV (regex -> error/threshold) | - |
| `-p <file\|spec>` | Projection: `.prj` file, or inline CRS (`EPSG:<code>` / WKT / `+proj=...`) | - |
| `--cps <csv>` | Control points CSV (header `sx,sy,sz,tx,ty,tz`) | - |
| `-C <mode>` | Output coordinate system: `original` / `left` | original |
| `-g <mode>` | Georeferencing: `7param` / `multipos` / `anchor` | 7param |
| `--7p <mx,my,mz,rx,ry,rz,s>` | 7-parameter Helmert (translation m; rotation arc-sec; scale ppm) | required (7param) |
| `--offset x,y,z` | Projection offset in source CRS (meters) | 0,0,0 |
| `--fit-order` | Polynomial fit order for multipos (1/2/3) | 1 |
| `--auto-crs` | Auto-detect source CRS (multipos only) | off |
| `--proj-test` | Validate the PROJ library and exit | - |

### mgo tiles - 3D Tiles Conversion

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
| `-r <mode>` | Refine mode: `ADD` / `REPLACE` | ADD |
| `--prj <file\|spec>` | Projection: `.prj` file, or inline CRS (`EPSG:<code>` / WKT / `+proj=...`) | - |
| `--origin <x,y,z>` | Coordinate origin (projected CRS) | 0,0,0 |
| `--min-block <val>` | Minimum block distance | 100 |
| `--max-lod <N>` | Max LOD levels | 5 |
| `--7p <mx,my,mz,rx,ry,rz,s>` | 7-parameter Helmert transform | - |
| `--cps <csv>` | Control points CSV (for multipos) | - |
| `--georef <mode>` | Georef mode: `7param` / `multipos` / `anchor` | 7param |
| `--fit-order <N>` | Polynomial fit order (1/2/3, multipos only) | 1 |
| `--error <val>` | Simplification error | 0.01 |
| `--nweight <val>` | Normal weight | 0.1 |
| `--threshold <val>` | Ratio threshold (0 = error-driven) | 0.1 |
| `--lock-border` | Enable border vertex locking | off |
| `--bim-bind` | Enable BIM property binding (alias `--bim-metadata`; implied by `--bim-props`) | off |
| `--bim-props <csv>` | Sidecar property table (first column = join key, RFC4180 quoting) | - |
| `--bim-id-property <k1,k2>` | ID key override - searched in scene metadata, then sidecar columns | per-strategy defaults |
| `--bim-strategy <s>` | Force binding strategy: `ifc` / `fbx` / `gltf2` / `obj` / `3ds` / `generic` | by importer |
| `--bim-report <f>` | Write per-instance transparency manifest (JSON) | - |
| `--bim-formats` | List per-format ID/metadata strategies and exit | - |
| `--bim-no-scene-meta` | Sidecar-only mode (skip scene-internal metadata) | off |
| `--bim-no-inherit` | Skip ancestor-chain metadata inheritance | off |

Models spanning more than 3 km automatically enable per-vertex projection correction to eliminate tangent-plane curvature residual.

### BIM Property Binding (`mgo tiles --bim-*`)

Per-feature business attributes are embedded into each b3dm Batch Table (readable in Cesium as `feature.properties`), resolved through per-format strategies (IFC / FBX / glTF2 / OBJ / 3DS / Generic) plus an optional external property table:

```bash
# Model-internal IDs/properties (IFC GlobalId, FBX/glTF UDP metadata):
./build/bin/MGOConsole tiles -i model.ifc -o out --bim-bind

# Join an external ledger CSV (first column = join key, e.g. node names or GUIDs):
./build/bin/MGOConsole tiles -i model.fbx -o out --bim-props ledger.csv \
    --bim-report out/bim_report.json
```

- **ID resolution chain** (each instance's winning source is reported transparently): IFC name-tail GUID → explicit `--bim-id-property` keys (scene metadata, then sidecar) → per-format default keys (`GlobalId`, `ElementId`, `ifcGUID`, `UniqueId`) → sidecar ID columns → full node name as weak fallback.
- **Sidecar CSV**: RFC4180 quoting (commas, doubled `""`, embedded newlines), UTF-8 BOM tolerated; unquoted numeric cells promote to typed binary columns (Int32/Int64/Double), everything else stays a string - quoted `"0042"` never becomes `42`.
- **Fail-fast**: an unreadable sidecar aborts the run with exit 1 instead of silently dropping all attributes.
- **Legacy encoding repair**: GBK/GB18030 node names baked by Windows exporters are converted to valid UTF-8 (Win32 CP_ACP / POSIX iconv; lossy U+FFFD only as last resort), so Chinese object names survive into the Batch Table.
- **Zero impact when off**: without `--bim-*` flags the output is byte-identical to the classic geometry-only pipeline; binding never alters georeferencing, tiling or simplification. A sidecar that matches no instance warns and falls back to the legacy path.

Full design contract and verification matrix: [`TilesConverter/BIM_BINDING_ARCHITECTURE.md`](TilesConverter/BIM_BINDING_ARCHITECTURE.md).

### mgo terrain - Terrain Tiles

```
mgo terrain -i dem.tif -o output_dir [options]
```

| Flag | Description | Default |
|------|-------------|---------|
| `-i <file>` | Input GeoTIFF DEM (single-band float32, striped layout) | required |
| `-o <dir>` | Output directory | required |
| `--prj <file\|spec>` | Override projection: `.prj` file, or inline CRS (`EPSG:<code>` / WKT / `+proj=...`) | from TIF |
| `--origin <x,y,z>` | Override TIF tiepoint (projected CRS) | from TIF |
| `--max-lod <N>` | Max LOD level | auto from pixel size |
| `--samples <N>` | Samples per tile edge (must be odd) | 65 |
| `--error <val>` | Simplification error (normalized [0,1]) | 0.001 |
| `--nweight <val>` | Normal weight | 0.0 |
| `--threshold <val>` | Ratio threshold (0 = error-driven) | 0.1 |
| `--lock-border` | Enable border vertex locking | off |
| `--no-normals` | Skip OctVertexNormals extension | off |
| `--7p <mx,..,s>` | 7-parameter Helmert transform | - |
| `--cps <f>` | Control points CSV (for multipos) | - |
| `--georef <mode>` | Georef mode: `7param` / `multipos` / `anchor` | - |
| `--fit-order <N>` | Polynomial fit order (1/2/3, multipos only) | 1 |
| `-v` | Verbose output | off |

Output format: Cesium quantized-mesh-1.0 (`{z}/{x}/{y}.terrain` + `layer.json`, with the OctVertexNormals extension by default). Tiles are processed in parallel.

### mgo image - DOM Tiling

```
mgo image -i ortho.tif -o output_dir [--prj <file|spec>]
```

| Flag | Description | Default |
|------|-------------|---------|
| `-i <file>` | Input GeoTIFF orthophoto (RGB, striped layout) | required |
| `-o <dir>` | Output directory | required |
| `--prj <file\|spec>` | Override projection | from TIF |

Output: Web Mercator TMS tile pyramid (`{z}/{x}/{y}.png` + `layer.json` + `tilemapresource.xml`). Uses an affine approximation for tile rendering - 4 PROJ calls per tile instead of 65,536.

> Note: tiled (blocked) GeoTIFF layout is not currently supported; strip-interleaved files (the typical output of most processing tools) work.

### mgo geojson - GeoJSON Projection Conversion

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

### mgo osgb - OSGB to 3D Tiles

```
mgo osgb -i osgb_root_dir -o output_dir [options]
```

| Flag | Description | Default |
|------|-------------|---------|
| `-i <dir>` | OSGB root directory (contains metadata.xml) | required |
| `-o <dir>` | Output directory | required |
| `--prj <file\|spec>` | Projection: `.prj` file, or inline CRS | from metadata.xml |
| `--enu <lat,lon[,h]>` | ENU tangent plane override (bypasses `--prj`) | from metadata.xml |
| `--origin <x,y,z>` | Coordinate origin override | from metadata.xml |
| `--max-lod <N>` | Max LOD level to convert | auto |
| `--7p <mx,..,s>` | 7-parameter Helmert transform | - |
| `--cps <f>` | Control points CSV (for multipos) | - |
| `--georef <mode>` | Georef mode: `7param` / `multipos` / `anchor` | - |
| `--fit-order <N>` | Polynomial fit order (1/2/3, multipos only) | 1 |
| `--error <val>` | Simplification error | 0.01 |
| `--nweight <val>` | Normal weight | 0.1 |
| `--threshold <val>` | Ratio threshold (0 = error-driven) | 0.1 |
| `--lock-border` | Enable border vertex locking | off |
| `-v` | Verbose output | off |

Requires OpenSceneGraph (`MGO_WITH_OSG=ON`, on by default). Supports DJI Terra (`Block_*` directories) and ContextCapture (`Data/Tile_*` directories).

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

## Coordinate System Conventions

| Space | X | Y | Z |
|-------|---|---|---|
| Assimp Y-up | East | Up | South |
| ENU | East | North | Up |
| 3D Tiles Z-up | East | North | Up |
| ECEF | Earth-centered X | Y | Z |

CesiumJS applies `Y_UP_TO_Z_UP` at runtime to convert the glTF content from Y-up to Z-up. The tileset `transform` (root ENU->ECEF matrix) and per-tile `boundingVolume.box` are written in the Cesium Z-up frame. Coordinate transforms flow through `AxisMapper` (single source of truth for all axis conversions).

Terrain vertex normals (Quantized-Mesh OctVertexNormals extension) are ECEF-space normals: CesiumJS decodes and uses them directly as model-coordinate normals (`normalMC = czm_octDecode(...)`), so no ENU rotation is applied at encode time.

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
| iconv | POSIX auto-detected | GBK/GB18030 → UTF-8 name repair (`check_include_file_cxx`; absent → lossy-safe U+FFFD fallback) |

## License

Copyright Johnlyon. Licensed under the [Apache License 2.0](LICENSE).

Third-party component licenses are acknowledged in [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
