// Copyright Johnlyon
//
// CoordinateTransform — centralized coordinate frame conversion
//
// Replaces the three inconsistent YZ-swap implementations spread across
// AxisMapper, ConvertTool, and TileBuilder with a single class.
//
// Every conversion is parameterized by source and target CoordinateFrame.
// Adding a new frame only requires adding entries to the conversion table.

#pragma once

#include "macro.h"
#include "CoordinateFrame.hpp"
#include <Eigen/Dense>

namespace MGO {

class MESH_PROJECTION_API CoordinateTransform {
public:
    // ---- Point conversion (3 doubles in → 3 doubles out) ----
    static void Convert(const double in[3], CoordinateFrame from,
                        double out[3], CoordinateFrame to);

    // ---- Bounding box conversion (handles min/max swap on negation) ----
    static void ConvertBBox(const double bmin[3], const double bmax[3],
                            CoordinateFrame from,
                            double omin[3], double omax[3],
                            CoordinateFrame to);

    // ---- Normal/vector conversion (no translation, W=0 semantics) ----
    static void ConvertNormal(const double n[3], CoordinateFrame from,
                              double out[3], CoordinateFrame to);

    // ---- 3×3 rotation matrix between frames ----
    static Eigen::Matrix3d RotateMatrix(const Eigen::Matrix3d& R_in,
                                        CoordinateFrame from, CoordinateFrame to);

    // ---- Build 4×4 column-major ENU→ECEF root transform at a point ----
    // R_enu = ENU→ECEF rotation (columns = East, North, Up in ECEF)
    // translation = ECEF origin
    // Returns 4×4 column-major transform matrix (.data() matches the old out[16])
    static Eigen::Matrix4d BuildRootTransform(const Eigen::Matrix3d& R_enu,
                                              const Eigen::Vector3d& translation);
};

} // namespace MGO
