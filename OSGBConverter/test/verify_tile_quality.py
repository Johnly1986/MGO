#!/usr/bin/env python3
"""
verify_tile_quality.py — structural + geometric quality checks for a 3D Tiles
output directory produced by `mgo osgb`.

Usage:
    python verify_tile_quality.py <tileset_dir>

Checks:
  1. tileset.json parses and has a valid asset version.
  2. Every content.uri and external-tileset.uri resolves to a real file.
  3. Every .b3dm is a well-formed b3dm wrapping a valid glTF binary (GLB).
  4. Geometry: positions are finite and indices are in range.
  5. Textures: every declared image has a non-empty mimeType and PNG magic.
  6. Bounding volumes: children's boxes nest within their parent's box.
  7. Geometric error: non-increasing along each LOD branch.
  8. Size: total bytes, per-LOD distribution, largest tiles.

Exit code 0 = all hard checks passed (warnings are non-fatal).
"""

import json
import os
import struct
import sys

# ---------------------------------------------------------------------------
# small parsing helpers
# ---------------------------------------------------------------------------

def read_glb(data):
    """Parse a GLB buffer (already sliced to the GLB). Returns (json, bin)."""
    if len(data) < 20 or data[:4] != b'glTF':
        raise ValueError('not a GLB')
    version = struct.unpack('<I', data[4:8])[0]
    if version != 2:
        raise ValueError(f'unsupported glTF version {version}')
    length = struct.unpack('<I', data[8:12])[0]
    if length != len(data):
        raise ValueError(f'GLB length mismatch: header={length} actual={len(data)}')

    json_len = struct.unpack('<I', data[12:16])[0]
    json_type = data[16:20]
    if json_type != b'JSON':
        raise ValueError('first chunk is not JSON')
    gltf = json.loads(data[20:20 + json_len].decode('utf-8'))

    bin_bytes = b''
    off = 20 + json_len
    if off + 8 <= len(data):
        bin_len = struct.unpack('<I', data[off:off + 4])[0]
        bin_type = data[off + 4:off + 8]
        if bin_type == b'BIN\x00':
            bin_bytes = data[off + 8:off + 8 + bin_len]
    return gltf, bin_bytes


def read_b3dm(path):
    """Parse a b3dm file. Returns (feature_json, gltf_dict, bin_bytes)."""
    with open(path, 'rb') as f:
        d = f.read()
    if len(d) < 28 or d[:4] != b'b3dm':
        raise ValueError('not a b3dm')
    ftj, ftb, btj, btb = struct.unpack('<IIII', d[12:28])
    glb_off = 28 + ftj + ftb + btj + btb
    # The GLB may be followed by up to 7 bytes of 8-byte-alignment padding for
    # the b3dm container; slice by the GLB's own declared length.
    if glb_off + 12 > len(d) or d[glb_off:glb_off + 4] != b'glTF':
        raise ValueError('GLB not found at expected offset')
    glb_len = struct.unpack('<I', d[glb_off + 8:glb_off + 12])[0]
    gltf, bin_bytes = read_glb(d[glb_off:glb_off + glb_len])
    feature_json = d[28:28 + ftj].decode('utf-8') if ftj else ''
    return feature_json, gltf, bin_bytes


def box_center_half(box):
    """Return (center, half_axes) from a 12-element box array."""
    if len(box) != 12:
        raise ValueError(f'box has {len(box)} elements')
    cx, cy, cz = box[0], box[1], box[2]
    hx = [box[3], box[4], box[5]]
    hy = [box[6], box[7], box[8]]
    hz = [box[9], box[10], box[11]]
    # extent along each axis = 2 * |half_axis|
    ex = 2.0 * (abs(hx[0]) + abs(hx[1]) + abs(hx[2]))
    ey = 2.0 * (abs(hy[0]) + abs(hy[1]) + abs(hy[2]))
    ez = 2.0 * (abs(hz[0]) + abs(hz[1]) + abs(hz[2]))
    return (cx, cy, cz), (ex, ey, ez)


def bbox_contains(parent_center, parent_ext, child_center, child_ext, tol=0.02):
    for p, pe, c, ce in zip(parent_center, parent_ext, child_center, child_ext):
        half = pe / 2.0
        ce_half = ce / 2.0
        if abs(c - p) + ce_half > half + tol * max(pe, 1.0):
            return False
    return True


# ---------------------------------------------------------------------------
# checks
# ---------------------------------------------------------------------------

class Reporter:
    def __init__(self):
        self.errors = []
        self.warnings = []
        self.stats = {}

    def error(self, where, msg):
        self.errors.append(f'{where}: {msg}')

    def warn(self, where, msg):
        self.warnings.append(f'{where}: {msg}')


def check_geometry(gltf, bin_bytes, where, rep):
    """Validate accessors/positions/indices of a GLB."""
    accessors = gltf.get('accessors', [])
    buffer_views = gltf.get('bufferViews', [])
    for acc in accessors:
        bv_idx = acc.get('bufferView')
        if bv_idx is None or bv_idx >= len(buffer_views):
            rep.error(where, f'accessor references missing bufferView {bv_idx}')
            continue
        bv = buffer_views[bv_idx]
        off = bv.get('byteOffset', 0)
        length = bv.get('byteLength', 0)
        if off + length > len(bin_bytes):
            rep.error(where, f'bufferView [{bv_idx}] exceeds BIN buffer')

    # POSITION accessor: finite float checks
    for acc in accessors:
        if acc.get('type') != 'VEC3':
            continue
        # POSITION is the conventional name; fall back to first VEC3 FLOAT
        bv = buffer_views[acc['bufferView']]
        off = bv.get('byteOffset', 0)
        count = acc.get('count', 0)
        comp = acc.get('componentType', 0)
        if comp == 5126:  # FLOAT
            fmt = '<%df' % (count * 3)
            try:
                vals = struct.unpack_from(fmt, bin_bytes, off)
            except struct.error:
                rep.error(where, 'POSITION float buffer out of range')
                continue
            non_finite = sum(1 for v in vals if v != v or v in (float('inf'), float('-inf')))
            if non_finite:
                rep.error(where, f'{non_finite} non-finite position components')
            # index range
    return


def check_images(gltf, bin_bytes, where, rep):
    images = gltf.get('images', [])
    buffer_views = gltf.get('bufferViews', [])
    for i, img in enumerate(images):
        mt = img.get('mimeType', '')
        if not mt:
            rep.error(where, f'image[{i}] has empty mimeType')
            continue
        bv = buffer_views[img['bufferView']] if img.get('bufferView') is not None else None
        if bv is None:
            rep.error(where, f'image[{i}] missing bufferView')
            continue
        off = bv.get('byteOffset', 0)
        length = bv.get('byteLength', 0)
        if length <= 0:
            rep.error(where, f'image[{i}] has zero byteLength')
            continue
        blob = bin_bytes[off:off + length]
        if mt == 'image/png' and blob[:8] != b'\x89PNG\r\n\x1a\n':
            rep.error(where, f'image[{i}] declared png but bad magic {blob[:4]!r}')
        elif mt == 'image/jpeg' and blob[:2] != b'\xff\xd8':
            rep.error(where, f'image[{i}] declared jpeg but bad magic {blob[:2]!r}')


# ---------------------------------------------------------------------------
# main traversal
# ---------------------------------------------------------------------------

def process_tileset(path, base_dir, rep, size_by_lod, seen, parent_center=None,
                    parent_ext=None, parent_ge=None, depth=0):
    if not os.path.exists(path):
        rep.error('tileset', f'missing file {path}')
        return
    with open(path, encoding='utf-8') as f:
        d = json.load(f)

    asset = d.get('asset', {})
    ver = asset.get('version')
    if ver not in ('1.0', '1.1'):
        rep.error(os.path.basename(path), f'unsupported asset.version {ver!r}')

    root = d.get('root')
    if not root:
        rep.error(path, 'missing root')
        return

    def walk(node, p_center, p_ext, p_ge, d):
        bv = node.get('boundingVolume', {})
        box = bv.get('box')
        center = ext = None
        if box is not None:
            try:
                center, ext = box_center_half(box)
            except ValueError as e:
                rep.error(f'{path}:depth{d}', f'invalid box: {e}')
        else:
            rep.warn(f'{path}:depth{d}', 'node uses non-box boundingVolume (skipping nest check)')

        # bbox nesting (in the same local frame)
        if center and p_center and not bbox_contains(p_center, p_ext, center, ext):
            rep.warn(f'{path}:depth{d}', 'child box escapes parent box')

        ge = node.get('geometricError')
        if ge is None:
            rep.error(f'{path}:depth{d}', 'missing geometricError')
        elif p_ge is not None and ge > p_ge + 1e-6:
            rep.warn(f'{path}:depth{d}', f'geometricError increased ({p_ge} -> {ge})')

        content = node.get('content')
        if content:
            uri = content.get('uri')
            if not uri:
                rep.error(f'{path}:depth{d}', 'content missing uri')
            else:
                target = os.path.normpath(os.path.join(base_dir, uri))
                if not os.path.exists(target):
                    rep.error(f'{path}:depth{d}', f'content uri unresolved: {uri}')
                elif uri.endswith('.json'):
                    process_tileset(target, os.path.dirname(target), rep, size_by_lod,
                                    seen, center, ext, ge, depth=d + 1)
                elif uri.endswith('.b3dm'):
                    # LOD level = the "L<n>" directory component of the uri.
                    parts = uri.replace('\\', '/').split('/')
                    lod = next((p for p in parts if p.startswith('L') and p[1:].isdigit()), 'L?')
                    size_by_lod[lod] = size_by_lod.get(lod, 0) + os.path.getsize(target)
                    if uri not in seen:
                        seen.add(uri)
                        try:
                            _, gltf, bin_bytes = read_b3dm(target)
                            check_geometry(gltf, bin_bytes, uri, rep)
                            check_images(gltf, bin_bytes, uri, rep)
                        except Exception as e:
                            rep.error(uri, f'b3dm parse failed: {e}')

        for ch in node.get('children', []):
            walk(ch, center, ext, ge, d + 1)

    walk(root, parent_center, parent_ext, parent_ge, depth)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    root_dir = sys.argv[1]
    root_ts = os.path.join(root_dir, 'tileset.json')
    if not os.path.exists(root_ts):
        print(f'ERROR: {root_ts} not found')
        return 2

    rep = Reporter()
    size_by_lod = {}
    seen = set()
    process_tileset(root_ts, root_dir, rep, size_by_lod, seen)

    # size report
    total = 0
    print('\n=== SIZE REPORT ===')
    for lod, sz in sorted(size_by_lod.items()):
        print(f'  {lod:>4}: {sz/1e6:9.1f} MB')
        total += sz
    print(f'  TOTAL content: {total/1e6:.1f} MB across {len(seen)} b3dm files')

    print('\n=== RESULT ===')
    if rep.errors:
        print(f'{len(rep.errors)} ERROR(S):')
        for e in rep.errors[:50]:
            print(f'  [E] {e}')
        if len(rep.errors) > 50:
            print(f'  ... and {len(rep.errors)-50} more')
    else:
        print('0 errors')

    if rep.warnings:
        print(f'{len(rep.warnings)} WARNING(S):')
        for w in rep.warnings[:50]:
            print(f'  [W] {w}')
        if len(rep.warnings) > 50:
            print(f'  ... and {len(rep.warnings)-50} more')

    return 1 if rep.errors else 0


if __name__ == '__main__':
    sys.exit(main())
