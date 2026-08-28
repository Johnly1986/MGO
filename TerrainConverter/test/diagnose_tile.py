#!/usr/bin/env python3
"""Decode a terrain tile and verify v-coordinate vs height orientation.

Usage: python3 diagnose_tile.py <path/to/tile.terrain>

Checks:
  1. Are v=0 vertices at SOUTH edge (lower northing)?
  2. Are v=32767 vertices at NORTH edge (higher northing)?
  3. Does height increase with northing (correct) or decrease (inverted)?
  4. Are the minH/maxH in the header reasonable for this tile?
"""

import struct, sys, math, os

def zigzag_decode(v):
    return (v >> 1) ^ (-(v & 1))

def main(path):
    with open(path, 'rb') as f:
        fsize = os.fstat(f.fileno()).st_size
        # Header (88 bytes)
        hdr = f.read(88)
        cx, cy, cz = struct.unpack('<ddd', hdr[0:24])
        minH, maxH = struct.unpack('<ff', hdr[24:32])
        bsX, bsY, bsZ = struct.unpack('<ddd', hdr[32:56])
        bsR = struct.unpack('<d', hdr[56:64])[0]

        print(f"File: {path} ({fsize} bytes)")
        print(f"Header: center=({cx:.3f},{cy:.3f},{cz:.3f})")
        print(f"        minH={minH:.4f} maxH={maxH:.4f} range={maxH-minH:.4f}")
        print(f"        bsRadius={bsR:.3f}")
        print()

        # Vertex count
        vc = struct.unpack('<I', f.read(4))[0]
        if vc == 0:
            print("Empty tile (no vertices)")
            return

        # Decode u
        u_vals = []
        prev = 0
        for _ in range(vc):
            z = struct.unpack('<H', f.read(2))[0]
            d = zigzag_decode(z)
            prev += d
            u_vals.append(prev)

        # Decode v
        v_vals = []
        prev = 0
        for _ in range(vc):
            z = struct.unpack('<H', f.read(2))[0]
            d = zigzag_decode(z)
            prev += d
            v_vals.append(prev)

        # Decode height (uint16 quantized)
        h_vals = []
        rng = maxH - minH
        if rng < 1e-6: rng = 1.0
        prev = 0
        for _ in range(vc):
            z = struct.unpack('<H', f.read(2))[0]
            d = zigzag_decode(z)
            cur = prev + d
            prev = cur
            h = minH + (cur / 32767.0) * rng
            h_vals.append(h)

        # Analysis
        print(f"Vertex count: {vc}")
        print(f"u range: [{min(u_vals)}, {max(u_vals)}]")
        print(f"v range: [{min(v_vals)}, {max(v_vals)}]")
        print(f"h range: [{min(h_vals):.4f}, {max(h_vals):.4f}]")
        print()

        # Split vertices into north (v > 16384) and south (v < 16383) halves
        north_vs = [i for i in range(vc) if v_vals[i] > 24575]   # top 25%
        south_vs = [i for i in range(vc) if v_vals[i] < 8191]    # bottom 25%
        west_us  = [i for i in range(vc) if u_vals[i] < 8191]
        east_us  = [i for i in range(vc) if u_vals[i] > 24575]

        def mean_h(indices):
            if not indices: return float('nan')
            return sum(h_vals[i] for i in indices) / len(indices)

        nh = mean_h(north_vs)
        sh = mean_h(south_vs)
        wh = mean_h(west_us)
        eh = mean_h(east_us)

        print("Elevation by region:")
        print(f"  North (v>24575): {len(north_vs)} verts, mean elevation = {nh:.2f}m")
        print(f"  South (v<8191):  {len(south_vs)} verts, mean elevation = {sh:.2f}m")
        print(f"  West  (u<8191):  {len(west_us)} verts, mean elevation = {wh:.2f}m")
        print(f"  East  (u>24575): {len(east_us)} verts, mean elevation = {eh:.2f}m")

        print()
        if not math.isnan(nh) and not math.isnan(sh):
            if nh > sh:
                print(f"  North > South by {nh-sh:.2f}m ✓ (north-up terrain)")
            elif nh < sh:
                print(f"  South > North by {sh-nh:.2f}m ✗ (NORTH-SOUTH INVERTED!)")
            else:
                print(f"  North ≈ South (flat tile)")

        # Check edge vertices
        edge_north = [i for i in range(vc) if v_vals[i] == 32767]
        edge_south = [i for i in range(vc) if v_vals[i] == 0]
        print(f"\nEdge vertices: north={len(edge_north)}, south={len(edge_south)}")
        if edge_north:
            nh_edge = mean_h(edge_north)
            print(f"  North edge mean elevation: {nh_edge:.2f}m")
        if edge_south:
            sh_edge = mean_h(edge_south)
            print(f"  South edge mean elevation: {sh_edge:.2f}m")

        # Show first 8 vertices
        print(f"\nFirst 8 vertices:")
        for i in range(min(8, vc)):
            print(f"  v[{i}] u={u_vals[i]:5d} v={v_vals[i]:5d} h={h_vals[i]:8.2f}")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 diagnose_tile.py <path/to/tile.terrain>")
        sys.exit(1)
    main(sys.argv[1])
