#pragma once

#include "macro.h"
#include "../MeshProjectionErrorCorrector/SimplifyOptions.h"
#include "HeightmapGrid.h"
#include "QuantizedMeshEncoder.h"

// Forward declaration
class CProjectionEngine;

// TinSimplifier — simplify regular heightmap grid to TIN using meshoptimizer
//
// Pipeline (ECEF Cartesian-space simplification):
//   1. Build regular grid mesh (W×H vertices, 2×(W-1)×(H-1) triangles)
//   2. Convert grid points to tile-local ECEF (meters) via CProjectionEngine
//   3. Compute per-vertex normals directly in ECEF (equal-weight voting)
//   4. meshopt_simplifyWithAttributes on ECEF positions + normals (metric error)
//   5. Degenerate triangle filter (real 3D area) + winding fix (dot with up)
//   6. Vertex remap on ECEF, output mapped to UVH: (u*32767, v*32767, h_meters)
//
// Using ECEF ensures all geometric operations (distances, cross products,
// normals) are physically correct — no scale-factor corrections needed.
// Positions are stored tile-local (ECEF − tile centre) for float32 precision.
//
// QuantizeToLocal is a no-op passthrough (Simplify already produces quantized
// vertices). It exists for API consistency.
//
class TERRAIN_CONVERTER_API TinSimplifier
{
public:
    // Simplify a heightmap grid to TIN in ECEF Cartesian space.
    // localGrid: subset heightmap for this tile (localWidth × localHeight samples)
    // proj: projection engine for geographic → ECEF conversion
    // targetError: max error relative to mesh extents, range [0..1].
    //   In normalized [0,1] space, this is the fraction of the mesh diagonal.
    //   0.01 ≈ 1% of diagonal ≈ grid-resolution-level error — good default
    //   for preserving terrain detail while removing redundant vertices.
    // normalWeight: weight for normal attributes in simplification (default: 0.1)
    // lockBorder: if true, border vertices are locked (recommended for tile stitching)
    static TinMesh Simplify(const HeightmapGrid& localGrid,
                            CProjectionEngine& proj,
                            const SimplifyOptions& simplify);

    // Copy/clamp vertices to output. Since Simplify() already produces
    // quantized (u,v) ∈ [0,32767] and height in meters, this is a passthrough.
    // Exists for API consistency — a future ECEF-space pipeline would do the
    // inverse ECEF→geographic→quantize conversion here.
    static void QuantizeToLocal(const HeightmapGrid& localGrid,
                                CProjectionEngine& proj,
                                const TinMesh& tin,
                                TinMesh& outQuantized);
};
