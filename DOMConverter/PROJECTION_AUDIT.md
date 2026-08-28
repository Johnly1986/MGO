# Projection Audit — TerrainConverter / TilesConverter / DOMConverter

## Executive Summary

Found **42 issues** across three modules: 8 critical, 12 high, 14 medium, 8 minor.

One critical compile error (**simpIndices undeclared** in TinSimplifier.cpp) was found and fixed during this audit.

---

## Critical Issues (8)

### C1. [TILES] GKInverse — D-power doubled in latitude Taylor series
**File:** `MeshProjectionErrorCorrector/GeodeticMath.h:98-105`

`term2`, `term3`, `term4` include redundant `D4`, `D6`, `D8` factors. Combined with `(x*D3/k0)`, this produces `D^8` instead of `D^4` (D^12 instead of D^6, D^16 instead of D^8). At the GK zone edge (D≈0.026), D^4≈4.6e-7 vs D^8≈2.1e-13 — higher-order corrections are effectively zeroed. Practical impact is **~0.6mm at zone edge** (the 4th-order term was already tiny), but mathematically incorrect per Snyder p.64 eq 8-79.

**Fix**: Remove `D4`, `D6`, `D8` from `term2`/`term3`/`term4`.

### C2. [TERRAIN] ExtractLocalGrid — linear approximation for inverse projection
**File:** `TerrainConverter/TerrainQuadtree.cpp:346-352`

When source is in projected CRS, lat/lon → easting/northing uses simple linear fraction of bounds. GK/UTM are conformal, not affine — positional error ~137m at 0.5° from central meridian. Every tile from a projected GeoTIFF samples elevation from the **wrong location**. The code acknowledges: "This is a rough approximation — for production, use proper inverse projection."

**Fix**: Implement `GeographicToProjected()` in GeodeticMath, pass CProjectionEngine to ExtractLocalGrid.

### C3. [TERRAIN] GeoTiffReader — m_wkt never populated
**File:** `TerrainConverter/GeoTiffReader.cpp` (all)

`m_wkt` is declared but never assigned. `ReadElevationGrid` copies it to `outGrid.projectionWKT`. For projected TIFs without `--prj`, the pipeline silently treats meters as degrees.

**Fix**: Build WKT string from parsed GeoKeys, or read GeoAsciiParams tag.

### C4. [TERRAIN] TinSimplifier — simpIndices undeclared (FIXED)
**File:** `TerrainConverter/TinSimplifier.cpp:144`

`simpIndices` used at line 144 without declaration. TerrainConverter cannot compile. **Fixed during this audit** — `std::vector<unsigned> simpIndices;` added before line 144.

### C5. [DOM] PROJ axis swap detection — geographically biased
**File:** `DOMConverter/ImageTiler.cpp:237-252`

Heuristic tests `proj_coord(103, 0)` and checks finiteness. Only works for GK zones near 103°E. For UTM zone 32N (CM 9°E), both `(103,0)` and `(0,103)` produce finite or infinite results depending on PROJ internals, making the test meaningless.

**Fix**: Use `proj_crs_get_coordinate_system()` to query axis order, or `proj_normalize_for_visualization()`.

### C6. [DOM] extractLonLat — fails when all longitudes in [-90, 90]
**File:** `DOMConverter/ImageTiler.cpp:279-290`

Checks `abs(a) <= 90 && abs(b) > 90` to detect swapped axes. When longitude < 90° (Africa, Europe, most of Asia west of 90°E), BOTH values are in [-90,90] after swap — heuristic cannot distinguish. German scene (lon=8.5°) would silently produce lat=8.5°, lon=52°.

### C7. [DOM] JPEG output destroys transparency
**File:** `DOMConverter/ImageTiler.cpp:384 + JpgWriter.cpp:38-49`

JPEG has no alpha channel. noData/out-of-bounds pixels become visible black/white artifacts. Combined with noData boundary blending (M3 below), creates visible fringes.

**Fix**: Either switch back to PNG (preserves RGBA), or render noData pixels as a uniform border color, or implement PNG+JPEG hybrid (PNG for edge tiles, JPEG for interior).

### C8. [DOM] Axis swap logic semantically inverted
**File:** `DOMConverter/ImageTiler.cpp:140-141`

`m_axisSwapped ? proj_coord(lat, lon) : proj_coord(lon, lat)` — the variable name is wrong. When `m_axisSwapped=true` (PROJ uses lat,lon), the code sends `(lat, lon)` which IS correct. When `false`, it sends `(lon, lat)` which is WRONG for default PROJ EPSG:4326 axis order. The detection heuristic happens to produce the right result by accident for Chinese GK zones.

---

## High-Severity Issues (12)

### H1. [SHARED] CProjectionEngine — uninitialized member variables
**File:** `MeshProjectionErrorCorrector/CProjectionEngine.cpp:40-111`

`m_a`, `m_f_inv`, `m_lambda0`, `m_falseE`, `m_falseN`, `m_k0` are never initialized. Missing PRJ params → garbage values → silently wrong projection.

### H2. [SHARED] CProjectionEngine — unchecked origin projection
**File:** `MeshProjectionErrorCorrector/CProjectionEngine.cpp:183`

`ComputeInstanceProjectionDelta()` calls `ProjectedToGeographic(origin)` without checking return value. Bad origin silently corrupts all instances' ECEF transforms.

### H3. [TERRAIN] GeoTiffReader — ModelTransformation scaleY assumes north-up
**File:** `TerrainConverter/GeoTiffReader.cpp:171`

`m_scaleY = -mat[5]` unconditionally negates. For south-up rasters, `mat[5]` is positive and negating gives a negative (wrong) value.

### H4. [TERRAIN] GeoTiffReader — user-defined projected CRS drops all params
**File:** `TerrainConverter/GeoTiffReader.cpp:232-236`

When `ProjectedCSTypeGeoKey >= 32767`, the code does nothing — no WKT, no projection parameters. All projection metadata silently lost.

### H5. [TERRAIN] TerrainQuadtree — antimeridian clamping
**File:** `TerrainConverter/TerrainQuadtree.cpp:74-77`

Longitudes clamped to [-180, 180] instead of wrapped. Scene at [179°, -179°] produces 358° span instead of 2°.

### H6. [TERRAIN] TerrainQuadtree — division by zero
**File:** `TerrainConverter/TerrainQuadtree.cpp:349-350`

`fracX = (lon - m_west) / (m_east - m_west)` — no guard for degenerate bounds (single-pixel TIF).

### H7. [TERRAIN] TerrainConverter — origin override assumes north-up
**File:** `TerrainConverter/TerrainConverter.cpp:57-58`

User origin always treated as top-left (max northing). No option for bottom-left origin.

### H8. [TERRAIN] Duplicate projection loading via tmpnam
**File:** `TerrainConverter/TerrainConverter.cpp:96`, `TerrainQuadtree.cpp:33`

WKT written to temp file and re-parsed independently in both classes. `std::tmpnam` deprecated (C++17), TOCTOU race condition.

### H9. [TILES] Bbox validity check only tests axis 0
**File:** `TilesConverter/TilesConverter.cpp:137,160,236,260`

`if (inst.bboxMin[0] > 1e100 || inst.bboxMax[0] < -1e100)` only checks X. Degenerate Y/Z silently passes.

### H10. [TILES] PRJ parser doesn't validate projection type
**File:** `MeshProjectionErrorCorrector/CProjectionEngine.cpp:40-111`

GKInverse-based math applied to any WKT. Lambert/Albers/Stereographic PRJ files silently produce wrong coordinates.

### H11. [TILES] Root transform ENU→ECEF vs CesiumJS Y_UP_TO_Z_UP
**File:** `MeshProjectionErrorCorrector/CProjectionEngine.cpp:303`

Root transform is ENU→ECEF (East,North,Up). CesiumJS applies Y_UP_TO_Z_UP which negates North. The root transform maps (-North) through ENU→ECEF, introducing east-west error for off-origin tiles.

### H12. [TERRAIN] GeoTiffReader — noDataFill not set
**File:** `TerrainConverter/GeoTiffReader.cpp:420-425`

`outGrid.noDataFill` never assigned in ReadElevationGrid. Remains at default 0.0f.

---

## Medium-Severity Issues (14)

M1. `GeoTiffReader:214,252` — DetectProjectionFromKeys whitelist incomplete (only 3 geographic EPSG codes)
M2. `TerrainQuadtree:58` — geographic bounds from 4 corners only, no mid-edge sampling
M3. `TerrainQuadtree:98` — spherical Earth 111320 m/deg, no meridian convergence
M4. `TerrainQuadtree:228` — empty-tile threshold (4 samples) doesn't check contiguity
M5. `HeightmapGrid:42` — bilinear rejects if ANY corner is noData
M6. `QuantizedMeshEncoder:352` — horizon occlusion overly conservative for large tiles
M7. DOM vs Shared — two projection paths (pure-math GKInverse vs PROJ library); numerical inconsistency
M8. DOM mid-edge sampling skips left/right edges, `g2` doesn't update latitude (ImageTiler.cpp:304-318)
M9. DOM `abs(ScaleX)` ignores ScaleY, non-square pixel handling broken (ImageTiler.cpp:196)
M10. DOM noData exact match misses boundary blend pixels after bilinear interp (ImageTiler.cpp:104-109)
M11. DOM `metersPerDeg` uses only longitude-cosine, ignores latitude variation (ImageTiler.cpp:328)
M12. TILES per-instance origin computation O(N) redundant (CProjectionEngine.cpp:182-196)
M13. TILES normal matrix inverse-transpose recomputed per vertex vs per instance (TileBuilder.cpp:617)
M14. TINSimplifier — docs describe nonexistent ECEF pipeline, dead code (TinSimplifier.h:14)

---

## Minor Issues (8)

N1. GKInverse 8th-order terms unnecessary (10⁻¹³ rad magnitude at zone edge)
N2. DOM PROJ handles not RAII, leak on exception (ImageTiler.h:95)
N3. PRJ WKT parser finds first `SPHEROID[` — wrong in nested WKT
N4. v-quant maps south→north, reversed from Cesium convention (QuantizedMeshEncoder.cpp:324)
N5. Geographic degrees in fields named `minEasting`/`maxEasting` (TerrainQuadtree.cpp:301)
N6. DOM duplicate `#include` directives (ImageTiler.h:3-8)
N7. DOM `system("mkdir -p")` shell injection via output path (ImageTiler.cpp:380)
N8. TILES `1 << maxLOD` UB if maxLOD ≥ 31 (TilesConverter.cpp:152)

---

## Optimization Recommendations

| Priority | Change | Impact |
|----------|--------|--------|
| **High** | Add `LoadProjectionFromString()` to skip temp files | Eliminate tmpnam + race |
| **High** | Cache origin ECEF/R in ComputeInstanceProjectionDelta | Remove O(N) redundant GKInverse |
| **High** | Hoist normal matrix from per-vertex to per-instance | ~100k fewer matrix inversions |
| **Medium** | Use `proj_trans_array()` for batch PROJ in RenderTile | 10-100x faster tile generation |
| **Medium** | Add `GeographicToProjected()` to GeodeticMath | Unify projection code paths |
| **Low** | Remove dead ECEF pipeline in TinSimplifier | Cleaner code |

---

## Risk Matrix

| Issue | Module | Severity | Detectability | Regression Risk |
|-------|--------|----------|---------------|-----------------|
| C1: GKInverse D-power | TilesConverter | Critical | Silent | Low |
| C2: Linear approx inverse | Terrain | Critical | Wrong coords | High |
| C3: m_wkt not populated | Terrain | Critical | Visible | Low |
| C4: simpIndices undeclared | Terrain | Critical | Compile error | N/A (fixed) |
| C5: Axis heuristic biased | DOM | Critical | Wrong coords | Medium |
| C6: extractLonLat fails <90° | DOM | Critical | Silent/region | Medium |
| C7: JPEG no alpha | DOM | Critical | Visible artifacts | Low |
| C8: Axis logic inverted | DOM | Critical | Wrong coords | Medium |
| H1: Uninitialized params | Shared | High | Silent | High |
| H2: Unchecked origin | Shared | High | Silent | Medium |
| H11: Root transform × Y_UP_TO_Z_UP | Tiles | High | Subtle | High |
