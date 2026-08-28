#!/usr/bin/env python3
"""
Image TMS Tile Converter — convert a georeferenced DOM orthophoto into
TMS image tiles ({z}/{x}/{y}.png) for web map display.

Pipeline:
  1. Read DOM image + projection PRJ + origin + pixel resolution
  2. Write a georeferenced GeoTIFF
  3. Compute geographic bounds (GK projected → EPSG:4326)
  4. For each zoom level, render tiles by sampling the source image
     through the full projection pipeline (affine approximation per tile)
  5. Write tilemapresource.xml and layer.json metadata

Tiling scheme: EPSG:4326 (Cesium GeographicTilingScheme).
Tile Y on disk uses TMS convention (Y=0 at south).

Usage:
  python imagery_tiler.py -i DOM.jpeg -p 103d10m.prj \\
      --origin 498700,2929900 --resolution 0.2 -o tiles/
"""

import argparse
import json
import math
import os
import struct
import sys
from xml.etree import ElementTree as ET

import numpy as np
from PIL import Image
from pyproj import CRS, Transformer

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
TILE_SIZE = 256


# ---------------------------------------------------------------------------
# GeoTIFF writer
# ---------------------------------------------------------------------------

def write_geotiff(image_path, tif_path, prj_wkt, origin_e, origin_n, origin_z, resolution):
    """Write a georeferenced RGB GeoTIFF from a DOM image."""
    img = Image.open(image_path)
    if img.mode != "RGB":
        img = img.convert("RGB")
    width, height = img.size
    pixels = np.array(img, dtype=np.uint8)

    wkt_ascii = (prj_wkt.strip() + "|\0").encode("ascii")
    strip_size = width * height * 3
    pixel_scale_sz = 3 * 8
    tiepoint_sz = 6 * 8
    wkt_sz = len(wkt_ascii)
    geokey_count = 6
    geokey_sz = (4 + geokey_count * 4) * 2

    # Layout
    hdr_sz = 8
    pos_strip = hdr_sz
    pos_pxscale = pos_strip + strip_size
    pos_tiept = pos_pxscale + pixel_scale_sz
    pos_wkt = pos_tiept + tiepoint_sz
    pos_geokey = pos_wkt + wkt_sz
    pos_ifd = pos_geokey + geokey_sz
    num_entries = 15
    ifd_sz = 2 + num_entries * 12 + 4
    pos_bps = pos_ifd + ifd_sz
    pos_sfmt = pos_bps + 6

    buf = bytearray()

    # Header
    buf += b"II"
    buf += struct.pack("<H", 42)
    buf += struct.pack("<I", pos_ifd)

    # Strip (top-down, TIFF standard)
    for row in range(height):
        buf += pixels[row].tobytes()

    # GeoTIFF doubles
    buf += struct.pack("<ddd", resolution, -resolution, 0.0)
    buf += struct.pack("<dddddd", 0.0, 0.0, 0.0, origin_e, origin_n, origin_z)

    # WKT
    buf += wkt_ascii

    # GeoKeyDirectory
    buf += struct.pack("<HHHH", 1, 1, 0, geokey_count)
    buf += struct.pack("<HHHH", 1024, 0, 1, 1)       # GTModelTypeGeoKey = Projected
    buf += struct.pack("<HHHH", 1025, 0, 1, 1)       # GTRasterTypeGeoKey = PixelIsArea
    buf += struct.pack("<HHHH", 2048, 0, 1, 4490)    # GeographicTypeGeoKey = CGCS2000
    buf += struct.pack("<HHHH", 3072, 0, 1, 32767)   # ProjectedCRSGeoKey = user-defined
    buf += struct.pack("<HHHH", 3075, 0, 1, 32767)   # ProjCoordTransGeoKey = user-defined
    buf += struct.pack("<HHHH", 3076, 0, 1, 9001)    # ProjLinearUnitsGeoKey = meter

    # IFD
    buf += struct.pack("<H", num_entries)
    def ifd(tag, typ, count, value):
        return struct.pack("<HHII", tag, typ, count, value)

    buf += ifd(256, 4, 1, width)
    buf += ifd(257, 4, 1, height)
    buf += ifd(258, 3, 3, pos_bps)
    buf += ifd(259, 3, 1, 1)
    buf += ifd(262, 3, 1, 2)
    buf += ifd(273, 4, 1, pos_strip)
    buf += ifd(277, 3, 1, 3)
    buf += ifd(278, 4, 1, height)
    buf += ifd(279, 4, 1, strip_size)
    buf += ifd(284, 3, 1, 1)
    buf += ifd(339, 3, 3, pos_sfmt)
    buf += ifd(33550, 12, 3, pos_pxscale)
    buf += ifd(33922, 12, 6, pos_tiept)
    buf += ifd(34735, 3, geokey_sz // 2, pos_geokey)
    buf += ifd(34737, 2, wkt_sz, pos_wkt)
    buf += struct.pack("<I", 0)
    buf += struct.pack("<HHH", 8, 8, 8)
    buf += struct.pack("<HHH", 1, 1, 1)

    os.makedirs(os.path.dirname(tif_path) or ".", exist_ok=True)
    with open(tif_path, "wb") as f:
        f.write(buf)
    print(f"GeoTIFF written: {tif_path} ({width}x{height}, {resolution}m/pixel)")


# ---------------------------------------------------------------------------
# Quadtree math (matches TerrainQuadtree.cpp)
# ---------------------------------------------------------------------------

def tile_bounds_cesium(level, x, y):
    """Geographic bounds for a Cesium-tiling tile at (level, x_cesium, y_cesium).

    Level 0: 2 tiles in X, 1 in Y. Y=0 at north pole.
    Returns (west, south, east, north) in degrees.
    """
    tiles_x = 2 << level          # 2 * 2^level
    tiles_y = 1 << level          # 2^level
    tile_w = 360.0 / tiles_x
    tile_h = 180.0 / tiles_y
    west = -180.0 + x * tile_w
    east = west + tile_w
    north = 90.0 - y * tile_h
    south = north - tile_h
    return west, south, east, north


def compute_max_level(pixel_size_deg):
    """Finest level where tileSpan >= pixelSize * TILE_SIZE."""
    target = pixel_size_deg * TILE_SIZE
    if target <= 0:
        return 18
    L = math.floor(math.log2(180.0 / target))
    return max(0, min(L, 22))


def compute_min_level(geo_w_deg, geo_h_deg):
    """Coarsest level where image covers at least a few tile pixels.

    At very low zoom levels, tiles span huge geographic areas and a small
    image may cover < 1 tile pixel, producing effectively empty tiles.
    This returns the first level where the image spans at least 2 tile
    pixels, minus 1 for a useful overview level.
    """
    min_span = min(geo_w_deg, geo_h_deg)
    for L in range(0, 23):
        pixel_w = (360.0 / (2 << L)) / TILE_SIZE
        if pixel_w <= min_span:
            return max(0, L - 1)
    return 0


# ---------------------------------------------------------------------------
# Projection engine (mirrors CProjectionEngine in Python)
# ---------------------------------------------------------------------------

class ProjectionEngine:
    """GK inverse projection using pyproj (mirrors CProjectionEngine)."""

    def __init__(self, prj_wkt):
        self.crs_src = CRS.from_wkt(prj_wkt)
        self.to_wgs84 = Transformer.from_crs(self.crs_src, "EPSG:4326", always_xy=True)
        self.from_wgs84 = Transformer.from_crs("EPSG:4326", self.crs_src, always_xy=True)

    def projected_to_geo(self, easting, northing):
        """GK → geographic (lon, lat). Returns (lon, lat) tuple."""
        lon, lat = self.to_wgs84.transform(easting, northing)
        return lon, lat

    def geo_to_projected(self, lon, lat):
        """Geographic → GK (easting, northing). Returns (e, n) tuple."""
        e, n = self.from_wgs84.transform(lon, lat)
        return e, n


# ---------------------------------------------------------------------------
# Tile renderer
# ---------------------------------------------------------------------------

class TileRenderer:
    """Render TMS tiles from a georeferenced source image."""

    def __init__(self, src_pixels, origin_e, origin_n, resolution, proj_engine):
        """
        Args:
            src_pixels: numpy array (H, W, 3) uint8 source image
            origin_e, origin_n: top-left pixel GK coordinates
            resolution: meters per pixel
            proj_engine: ProjectionEngine instance
        """
        self.src = src_pixels
        self.src_h, self.src_w = src_pixels.shape[:2]
        self.origin_e = origin_e
        self.origin_n = origin_n
        self.resolution = resolution
        self.proj = proj_engine

        # Precompute source coverage in geographic coords
        corners_gk = [
            (origin_e, origin_n),                                           # NW
            (origin_e + src_pixels.shape[1] * resolution, origin_n),        # NE
            (origin_e, origin_n - src_pixels.shape[0] * resolution),        # SW
            (origin_e + src_pixels.shape[1] * resolution,
             origin_n - src_pixels.shape[0] * resolution),                  # SE
        ]
        lons, lats = [], []
        for eg, ng in corners_gk:
            lon, lat = proj_engine.projected_to_geo(eg, ng)
            lons.append(lon)
            lats.append(lat)
        # Also sample mid-edges for curved projections
        for frac in [0.25, 0.5, 0.75]:
            for (e0, n0), (e1, n1) in [(corners_gk[0], corners_gk[1]),   # top edge
                                        (corners_gk[0], corners_gk[2]),    # left edge
                                        (corners_gk[3], corners_gk[1]),    # right edge
                                        (corners_gk[2], corners_gk[3])]:   # bottom edge
                elon, elat = proj_engine.projected_to_geo(
                    e0 + frac * (e1 - e0), n0 + frac * (n1 - n0))
                lons.append(elon)
                lats.append(elat)

        self.geo_west = min(lons)
        self.geo_east = max(lons)
        self.geo_south = min(lats)
        self.geo_north = max(lats)

    def geographic_bounds(self):
        return (self.geo_west, self.geo_south, self.geo_east, self.geo_north)

    def render_tile(self, level, tx, ty_cesium):
        """Render a single 256x256 tile at (level, tx, ty_cesium).

        Returns PIL Image (RGBA) or None if tile has no overlap with source.
        """
        tw, ts, te, tn = tile_bounds_cesium(level, tx, ty_cesium)

        # Quick rejection
        if te <= self.geo_west or tw >= self.geo_east or tn <= self.geo_south or ts >= self.geo_north:
            return None

        # Create coordinate grid for tile pixels
        px = np.arange(TILE_SIZE)
        py = np.arange(TILE_SIZE)
        px_grid, py_grid = np.meshgrid(px, py)

        # Geographic coords of each pixel center
        tile_w_deg = te - tw
        tile_h_deg = tn - ts
        lons = tw + (px_grid + 0.5) * (tile_w_deg / TILE_SIZE)
        lats = tn - (py_grid + 0.5) * (tile_h_deg / TILE_SIZE)

        # Batch transform: geographic → GK
        e_grid, n_grid = self.proj.from_wgs84.transform(lons, lats)

        # GK → source pixel coordinates
        sx_grid = (e_grid - self.origin_e) / self.resolution
        sy_grid = (self.origin_n - n_grid) / self.resolution

        # Valid mask: finite coords AND within source bounds
        finite = np.isfinite(sx_grid) & np.isfinite(sy_grid)
        sx_grid_safe = np.where(finite, sx_grid, 0.0)
        sy_grid_safe = np.where(finite, sy_grid, 0.0)
        sx0 = np.floor(sx_grid_safe).astype(int)
        sy0 = np.floor(sy_grid_safe).astype(int)
        fx = sx_grid_safe - sx0
        fy = sy_grid_safe - sy0

        valid = (
            finite &
            (sx0 >= 0) & (sx0 < self.src_w - 1) &
            (sy0 >= 0) & (sy0 < self.src_h - 1)
        )

        # Allocate output
        rgba = np.zeros((TILE_SIZE, TILE_SIZE, 4), dtype=np.uint8)

        if not np.any(valid):
            return None

        # Safe indices
        sx0_safe = np.clip(sx0, 0, self.src_w - 2)
        sy0_safe = np.clip(sy0, 0, self.src_h - 2)

        # Sample each channel
        for c in range(3):
            v00 = self.src[sy0_safe, sx0_safe, c].astype(float)
            v10 = self.src[sy0_safe, sx0_safe + 1, c].astype(float)
            v01 = self.src[sy0_safe + 1, sx0_safe, c].astype(float)
            v11 = self.src[sy0_safe + 1, sx0_safe + 1, c].astype(float)

            interp = (
                v00 * (1 - fx) * (1 - fy) +
                v10 * fx * (1 - fy) +
                v01 * (1 - fx) * fy +
                v11 * fx * fy
            )
            rgba[:, :, c] = np.clip(interp, 0, 255).astype(np.uint8)

        rgba[:, :, 3] = np.where(valid, 255, 0).astype(np.uint8)

        return Image.fromarray(rgba, "RGBA")


# ---------------------------------------------------------------------------
# Metadata writers
# ---------------------------------------------------------------------------

def write_tilemap_resource(output_dir, geo_bounds, zoom_levels, tile_counts):
    """Write TMS tilemapresource.xml."""
    west, south, east, north = geo_bounds

    tilemap = ET.Element("TileMap", version="1.0.0",
                         tilemapservice="http://tms.osgeo.org/1.0.0")
    ET.SubElement(tilemap, "Title").text = "DOM Orthophoto Tiles"
    ET.SubElement(tilemap, "Abstract").text = "TMS tiles from DOM orthophoto"
    ET.SubElement(tilemap, "SRS").text = "EPSG:4326"

    bbox = ET.SubElement(tilemap, "BoundingBox",
                         minx=str(west), miny=str(south),
                         maxx=str(east), maxy=str(north))
    ET.SubElement(tilemap, "Origin", x=str(west), y=str(south))

    fmt = ET.SubElement(tilemap, "TileFormat", width=str(TILE_SIZE),
                        height=str(TILE_SIZE), **{"mime-type": "image/png"},
                        extension="png")

    tilesets_el = ET.SubElement(tilemap, "TileSets")
    for level in sorted(zoom_levels):
        if tile_counts.get(level, 0) > 0:
            ET.SubElement(tilesets_el, "TileSet",
                          href=str(level),
                          **{"units-per-pixel": str(360.0 / (2 << level) / TILE_SIZE)},
                          order=str(level))

    tree = ET.ElementTree(tilemap)
    ET.indent(tree, space="  ")
    path = os.path.join(output_dir, "tilemapresource.xml")
    tree.write(path, encoding="utf-8", xml_declaration=True)
    print(f"  tilemapresource.xml written")


def write_layer_json(output_dir, geo_bounds, zoom_levels, tile_counts):
    """Write Cesium-compatible layer.json."""
    west, south, east, north = geo_bounds
    doc = {
        "tilejsonVersion": "1.0.0",
        "format": "png",
        "version": "1.0.0",
        "tiles": ["{z}/{x}/{y}.png"],
        "bounds": [west, south, east, north],
        "projection": "EPSG:4326",
        "minzoom": min(zoom_levels),
        "maxzoom": max(zoom_levels),
    }
    path = os.path.join(output_dir, "layer.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(doc, f, indent=2, ensure_ascii=False)
    print(f"  layer.json written")


# ---------------------------------------------------------------------------
# Main pipeline
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Convert georeferenced DOM image to TMS image tiles (EPSG:4326)")
    parser.add_argument("-i", "--input", required=True, help="Input DOM image (PNG/JPEG)")
    parser.add_argument("-p", "--prj", required=True, help="Projection PRJ file (WKT)")
    parser.add_argument("--origin", required=True,
                        help="Origin: easting,northing[,height] in projected CRS")
    parser.add_argument("-r", "--resolution", type=float, default=None,
                        help="Ground pixel resolution in meters (required)")
    parser.add_argument("-o", "--output", default="tiles", help="Output directory")
    parser.add_argument("--min-zoom", type=int, default=None)
    parser.add_argument("--max-zoom", type=int, default=None)
    parser.add_argument("--no-tif", action="store_true", help="Skip GeoTIFF writing")
    parser.add_argument("-v", "--verbose", action="store_true")

    args = parser.parse_args()

    if args.resolution is None:
        parser.error("--resolution is required")

    # Parse origin
    parts = [float(x) for x in args.origin.split(",")]
    origin_e = parts[0]
    origin_n = parts[1]
    origin_z = parts[2] if len(parts) > 2 else 0.0

    # Load PRJ
    with open(args.prj, "r", encoding="utf-8") as f:
        prj_wkt = f.read().strip()

    print(f"Projection: {prj_wkt[:80]}...")
    print(f"Origin GK: ({origin_e}, {origin_n}, {origin_z})")
    print(f"Resolution: {args.resolution} m/pixel")

    # Init projection engine
    proj = ProjectionEngine(prj_wkt)

    # Load source image
    img = Image.open(args.input)
    if img.mode != "RGB":
        img = img.convert("RGB")
    src_w, src_h = img.size
    src_pixels = np.array(img, dtype=np.uint8)
    print(f"Source image: {src_w}x{src_h} RGB")

    # --- Write GeoTIFF ---
    if not args.no_tif:
        tif_path = os.path.splitext(args.input)[0] + "_geo.tif"
        write_geotiff(args.input, tif_path, prj_wkt, origin_e, origin_n, origin_z,
                       args.resolution)

    # --- Init renderer (also computes geographic bounds) ---
    print("\nComputing geographic bounds...")
    renderer = TileRenderer(src_pixels, origin_e, origin_n, args.resolution, proj)
    geo_bounds = renderer.geographic_bounds()
    geo_w, geo_s, geo_e, geo_n = geo_bounds

    print(f"Geographic bounds: [{geo_w:.8f}, {geo_s:.8f}, {geo_e:.8f}, {geo_n:.8f}]")
    geo_w_deg = geo_e - geo_w
    geo_h_deg = geo_n - geo_s
    print(f"Extent: {geo_w_deg:.8f} x {geo_h_deg:.8f} degrees")

    # --- Zoom levels ---
    mid_lat = (geo_n + geo_s) / 2
    meters_per_deg = 111320.0 * math.cos(math.radians(mid_lat))
    pixel_size_deg = args.resolution / meters_per_deg

    max_zoom = args.max_zoom
    if max_zoom is None:
        max_zoom = compute_max_level(pixel_size_deg)
    min_zoom = args.min_zoom
    if min_zoom is None:
        min_zoom = compute_min_level(geo_w_deg, geo_h_deg)

    zoom_levels = list(range(min_zoom, max_zoom + 1))
    print(f"Zoom levels: {min_zoom}–{max_zoom} ({len(zoom_levels)} levels)")
    print(f"Approx pixel size: {pixel_size_deg:.10f} deg (~{args.resolution}m)")

    # --- Generate tiles ---
    os.makedirs(args.output, exist_ok=True)
    tile_counts = {}
    total_tiles = 0

    for level in zoom_levels:
        tiles_x_total = 2 << level
        tiles_y_total = 1 << level

        x_start = max(0, int(math.floor((geo_w + 180.0) / 360.0 * tiles_x_total)))
        x_end = min(tiles_x_total - 1, int(math.ceil((geo_e + 180.0) / 360.0 * tiles_x_total)))
        y_start = max(0, int(math.floor((90.0 - geo_n) / 180.0 * tiles_y_total)))
        y_end = min(tiles_y_total - 1, int(math.ceil((90.0 - geo_s) / 180.0 * tiles_y_total)))

        count = 0
        for tx in range(x_start, x_end + 1):
            for ty_cesium in range(y_start, y_end + 1):
                tile_img = renderer.render_tile(level, tx, ty_cesium)
                if tile_img is None:
                    continue

                tms_y = tiles_y_total - 1 - ty_cesium
                tile_dir = os.path.join(args.output, str(level), str(tx))
                os.makedirs(tile_dir, exist_ok=True)
                tile_path = os.path.join(tile_dir, f"{tms_y}.png")
                tile_img.save(tile_path, "PNG", optimize=True)
                count += 1

        tile_counts[level] = count
        total_tiles += count
        xtiles = x_end - x_start + 1
        ytiles = y_end - y_start + 1
        print(f"  L{level:2d}: {count:4d} tiles "
              f"(X={x_start}..{x_end} [{xtiles}], Y_c={y_start}..{y_end} [{ytiles}])")

    print(f"\nTotal: {total_tiles} tiles across {len(zoom_levels)} levels")

    # --- Metadata ---
    print("\nWriting metadata...")
    write_tilemap_resource(args.output, geo_bounds, zoom_levels, tile_counts)
    write_layer_json(args.output, geo_bounds, zoom_levels, tile_counts)

    print(f"\nDone. Output: {os.path.abspath(args.output)}/")


if __name__ == "__main__":
    main()
