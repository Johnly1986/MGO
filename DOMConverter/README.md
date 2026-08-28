# DOMConverter — DOM Orthophoto to TMS Image Tiles

Converts a georeferenced DOM (Digital Orthophoto Map) image into a TMS image tile pyramid for web map display.

## Architecture

The pipeline mirrors the **TerrainConverter** module's design, adapted for RGB image data:

```
DOM image (PNG/JPEG/TIF)
    │
    ├──[1] GeoTIFF writer ─── georeferenced DOM_geo.tif
    │
    ├──[2] ProjectionEngine ── GK projected → WGS84 geographic (pyproj / PROJ)
    │         Replaces: CProjectionEngine::ProjectedToGeographic()
    │
    ├──[3] TileRenderer ───── for each zoom level, render 256×256 RGBA tiles
    │         Replaces: TerrainQuadtree::Build() + TinSimplifier + QuantizedMeshEncoder
    │
    ├──[4] Metadata ───────── layer.json + tilemapresource.xml
    │         Replaces: TerrainLayerJson + Cesium metadata
    │
    └──► Output: {z}/{x}/{y}.png tiles
```

## Relationship to Existing Modules

| This module | Existing C++ module | How it reuses |
|---|---|---|
| GK→WGS84 projection | `CProjectionEngine` / `GeodeticMath::GKInverse` | Same PROJ library via pyproj; identical math verified |
| Quadtree tiling | `TerrainQuadtree::ComputeTileBounds()` | Same Cesium GeographicTilingScheme (EPSG:4326) |
| Max zoom level | `TerrainQuadtree::ComputeMaxLevelFromResolution()` | Same formula: `L = log2(180 / (pixelSize * tileSize))` |
| TMS Y-flip | `TerrainConverter::ProcessTile()` L237 | Same: `tmsY = rowsAtLevel - 1 - cesiumY` |
| GeoTIFF writer | `test/generate_test_tif.py` | Same manual TIFF binary construction pattern |
| layer.json | `TerrainLayerJson` | Matching Cesium tile metadata format |

## Quick Start

```bash
# 1. Convert DOM image to georeferenced TIF + TMS tiles
python TerrainConverter/imagery_tiler.py \
    -i Data/TIF/DOM.jpeg \
    -p Data/103d10m.prj \
    --origin 498700,2929900,0 \
    -r 0.2 \
    -o tiles/

# 2. Verify output
python DOMConverter/verify_output.py tiles/ \
    --prj Data/103d10m.prj \
    --origin 498700,2929900
```

## CLI Reference

```
usage: imagery_tiler.py [-h] -i INPUT -p PRJ --origin ORIGIN
                        -r RESOLUTION [-o OUTPUT]
                        [--min-zoom MIN_ZOOM] [--max-zoom MAX_ZOOM]
                        [--no-tif] [-v]

Required:
  -i, --input          Input DOM image (PNG/JPEG/TIF)
  -p, --prj            Projection PRJ file (WKT format)
  --origin             Origin coordinates: easting,northing[,height]
  -r, --resolution     Ground pixel resolution in meters/pixel (GSD)

Optional:
  -o, --output         Output directory (default: tiles/)
  --min-zoom           Minimum zoom level (auto-computed)
  --max-zoom           Maximum zoom level (auto-computed)
  --no-tif             Skip intermediate GeoTIFF generation
  -v, --verbose        Verbose output
```

## Tiling Scheme

- **Projection**: EPSG:4326 (WGS84 geographic)
- **Tile size**: 256×256 pixels, RGBA PNG
- **Root level (0)**: 2 tiles × 1 tile
- **Level L**: 2·2^L tiles × 2^L tiles
- **Y convention**: TMS on disk (Y=0 at south), Cesium internal for math
- **Tile path**: `{z}/{x}/{y}.png`

## Output Files

```
output_dir/
├── layer.json              # Cesium-compatible metadata
├── tilemapresource.xml     # TMS standard metadata
└── {z}/
    └── {x}/
        └── {y}.png         # 256×256 RGBA tile
```

## Verification

The `verify_output.py` script checks:
1. Directory structure and tile counts per level
2. All PNG files are valid 256×256 images
3. Edge stitching between adjacent tiles
4. layer.json and tilemapresource.xml validity
5. Geographic bounds consistency with projection

## Limitations

- **No border overlap**: Adjacent tiles share a boundary but don't overlap; edge pixels may differ slightly (~1-2 source pixels). This matches Cesium terrain tile behavior.
- **Single-threaded**: Tile rendering is sequential. For large images, consider parallel processing.
- **Memory**: Entire source image is loaded into memory. For very large images (>100MP), consider tiled reading.

## Pixel Resolution (GSD)

The pixel resolution (`-r`) is a **required parameter** and must match the physical ground sample distance of the source image. Common values:

| Data Type | Typical GSD |
|-----------|-------------|
| Drone survey | 0.02–0.10 m |
| Aerial imagery (1:2000) | 0.20 m |
| Aerial imagery (1:5000) | 0.50 m |
| Satellite imagery | 0.50–10.0 m |
