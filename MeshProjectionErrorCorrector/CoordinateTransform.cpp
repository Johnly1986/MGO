// Copyright Johnlyon
//
// CoordinateTransform — centralized coordinate frame conversion implementation

#include "CoordinateTransform.hpp"
#include "GeodeticMath.h"
#include <cmath>
#include <algorithm>

namespace MGO {

// =========================================================================
// Internal helpers — one per frame pair, preserving exact semantics from
// the original AxisMapper / ConvertTool / TileBuilder code.
// =========================================================================

namespace {

// ---- AssimpYUp ↔ ENU (AxisMapper convention: Z=South) ----
inline void assimpYUp_to_enu(const double in[3], double out[3]) {
    // AxisMapper::AssimpToENU: east=x, north=-z, up=y
    out[0] =  in[0];   // East = X
    out[1] = -in[2];   // North = -Z (Z=South → northward)
    out[2] =  in[1];   // Up = Y
}
inline void enu_to_assimpYUp(const double in[3], double out[3]) {
    // AxisMapper::ENUToAssimp: x=east, y=up, z=-north
    out[0] =  in[0];   // X = East
    out[1] =  in[2];   // Y = Up
    out[2] = -in[1];   // Z = -North = South
}

// ---- ZUp ↔ AssimpYUp (TileBuilder convention: pure YZ swap) ----
inline void zUp_to_assimpYUp(const double in[3], double out[3]) {
    // TileBuilder: swap(y, z) — Z-up (East,North,Up) → Y-up (East,Up,North)
    out[0] = in[0];    // East
    out[1] = in[2];    // Up = Z
    out[2] = in[1];    // Z(Y-up) = North
}
inline void assimpYUp_to_zUp(const double in[3], double out[3]) {
    // Inverse of above: Y-up (East,Up,North) → Z-up (East,North,Up)
    out[0] = in[0];    // East
    out[1] = in[2];    // North(Y-up Z) → Y(Z-up)
    out[2] = in[1];    // Up → Z
}

// ---- AssimpYUp ↔ Projected (ConvertTool convention: (x, z, y), Z=North) ----
// Preserved exactly from ConvertTool::AssimpVectorTransformZY
inline void assimpYUp_to_projected(const double in[3], double out[3]) {
    // ConvertTool::AssimpVectorTransformZY: (in.x, in.z, in.y)
    out[0] = in[0];    // East(Easting)
    out[1] = in[2];    // Z → Northing
    out[2] = in[1];    // Y(Up) → Height
}
inline void projected_to_assimpYUp(const double in[3], double out[3]) {
    // Inverse: (East, Northing, Height) → (East, Up, North)
    out[0] = in[0];    // East
    out[1] = in[2];    // Height → Y(Up)
    out[2] = in[1];    // Northing → Z
}

// ---- TilesZUp ↔ ENU (identity — they're the same frame) ----
inline void enu_to_tilesZUp(const double in[3], double out[3]) {
    out[0] = in[0]; out[1] = in[1]; out[2] = in[2];
}
inline void tilesZUp_to_enu(const double in[3], double out[3]) {
    out[0] = in[0]; out[1] = in[1]; out[2] = in[2];
}

} // anonymous namespace

// =========================================================================
// Public API
// =========================================================================

void CoordinateTransform::Convert(const double in[3], CoordinateFrame from,
                                   double out[3], CoordinateFrame to)
{
    if (from == to) {
        out[0] = in[0]; out[1] = in[1]; out[2] = in[2];
        return;
    }

    // Route through ENU as intermediate hub where needed.
    // Direct pairs handled first for efficiency.

    // --- AssimpYUp ↔ ENU ---
    if (from == CoordinateFrame::AssimpYUp && to == CoordinateFrame::ENU) {
        assimpYUp_to_enu(in, out); return;
    }
    if (from == CoordinateFrame::ENU && to == CoordinateFrame::AssimpYUp) {
        enu_to_assimpYUp(in, out); return;
    }

    // --- ZUp ↔ AssimpYUp ---
    if (from == CoordinateFrame::ZUp && to == CoordinateFrame::AssimpYUp) {
        zUp_to_assimpYUp(in, out); return;
    }
    if (from == CoordinateFrame::AssimpYUp && to == CoordinateFrame::ZUp) {
        assimpYUp_to_zUp(in, out); return;
    }

    // --- AssimpYUp ↔ Projected (ConvertTool convention) ---
    if (from == CoordinateFrame::AssimpYUp && to == CoordinateFrame::Projected) {
        assimpYUp_to_projected(in, out); return;
    }
    if (from == CoordinateFrame::Projected && to == CoordinateFrame::AssimpYUp) {
        projected_to_assimpYUp(in, out); return;
    }

    // --- TilesZUp ↔ ENU (identity) ---
    if (from == CoordinateFrame::TilesZUp && to == CoordinateFrame::ENU) {
        enu_to_tilesZUp(in, out); return;
    }
    if (from == CoordinateFrame::ENU && to == CoordinateFrame::TilesZUp) {
        tilesZUp_to_enu(in, out); return;
    }

    // --- ZUp ↔ ENU (via AssimpYUp) ---
    if (from == CoordinateFrame::ZUp && to == CoordinateFrame::ENU) {
        double tmp[3];
        zUp_to_assimpYUp(in, tmp);
        assimpYUp_to_enu(tmp, out);
        return;
    }
    if (from == CoordinateFrame::ENU && to == CoordinateFrame::ZUp) {
        double tmp[3];
        enu_to_assimpYUp(in, tmp);
        assimpYUp_to_zUp(tmp, out);
        return;
    }

    // --- TilesZUp → AssimpYUp (via ENU) ---
    if (from == CoordinateFrame::TilesZUp && to == CoordinateFrame::AssimpYUp) {
        double tmp[3]; tmp[0]=in[0]; tmp[1]=in[1]; tmp[2]=in[2];
        enu_to_assimpYUp(tmp, out);
        return;
    }
    if (from == CoordinateFrame::AssimpYUp && to == CoordinateFrame::TilesZUp) {
        double tmp[3];
        assimpYUp_to_enu(in, tmp);
        enu_to_tilesZUp(tmp, out);
        return;
    }

    // --- Projected, Geographic, ECEF are identity in pure-frame conversion ---
    // They require geodetic math (PROJ/GeodeticMath) which is handled by
    // CProjectionEngine, not here.  For pure frame conversion they're pass-through.
    out[0] = in[0]; out[1] = in[1]; out[2] = in[2];
}

void CoordinateTransform::ConvertBBox(const double bmin[3], const double bmax[3],
                                       CoordinateFrame from,
                                       double omin[3], double omax[3],
                                       CoordinateFrame to)
{
    if (from == to) {
        for (int i = 0; i < 3; ++i) { omin[i] = bmin[i]; omax[i] = bmax[i]; }
        return;
    }

    // Convert all 8 corners and take min/max envelope
    double corners[8][3];
    for (int c = 0; c < 8; ++c) {
        double p[3] = {
            (c & 1) ? bmax[0] : bmin[0],
            (c & 2) ? bmax[1] : bmin[1],
            (c & 4) ? bmax[2] : bmin[2]
        };
        Convert(p, from, corners[c], to);
    }

    for (int i = 0; i < 3; ++i) {
        omin[i] = corners[0][i];
        omax[i] = corners[0][i];
        for (int c = 1; c < 8; ++c) {
            if (corners[c][i] < omin[i]) omin[i] = corners[c][i];
            if (corners[c][i] > omax[i]) omax[i] = corners[c][i];
        }
    }
}

void CoordinateTransform::ConvertNormal(const double n[3],
                                         CoordinateFrame from,
                                         double out[3],
                                         CoordinateFrame to)
{
    // Normals use the same axis mapping as points but zero translation
    Convert(n, from, out, to);
}

Eigen::Matrix3d CoordinateTransform::RotateMatrix(const Eigen::Matrix3d& R_in,
                                                   CoordinateFrame from,
                                                   CoordinateFrame to)
{
    // For now, only ENU ↔ AssimpYUp is implemented (matches AxisMapper).
    // Other frame pairs can be added as needed.
    if (from == CoordinateFrame::ENU && to == CoordinateFrame::AssimpYUp) {
        // AxisMapper::RotationENUToAssimp: Rc = A^T * R * A
        // A = [[1,0,0],[0,0,-1],[0,1,0]] maps Assimp(E,U,S) -> ENU(E,N,U)
        Eigen::Matrix3d A;
        A << 1.0, 0.0, 0.0,
             0.0, 0.0, -1.0,
             0.0, 1.0, 0.0;
        return A.transpose() * R_in * A;
    }
    return R_in;
}

Eigen::Matrix4d CoordinateTransform::BuildRootTransform(const Eigen::Matrix3d& R_enu,
                                                        const Eigen::Vector3d& translation)
{
    // Column-major 4×4 ENU→ECEF matrix, no North negation.
    // CesiumJS applies Y_UP_TO_Z_UP on top of this transform.
    // Exact copy of AxisMapper::BuildRootTransform + GeodeticMath::BuildRootTransform.
    Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
    out.topLeftCorner<3, 3>() = R_enu;
    out.block<3, 1>(0, 3) = translation;
    return out;
}

} // namespace MGO
