// Copyright Johnlyon
//
// AxisMapper — unified coordinate axis conversion utilities
//
// Coordinate systems:
//   Assimp Y-up (right-handed): X=East, Y=Up, Z=South (toward viewer)
//   ENU:                        X=East, Y=North, Z=Up
//   3D Tiles Z-up:              X=East, Y=North, Z=Up (after CesiumJS Y_UP_TO_Z_UP)
//   ECEF:                       Earth-Centered Earth-Fixed
//
// Assimp uses the standard right-handed glTF convention:
//   +X = right (East), +Y = up (Up), +Z = toward viewer (South)
// This matches the 3D Tiles / CesiumJS glTF convention exactly.
//
// CesiumJS applies Y_UP_TO_Z_UP at runtime (mandated by 3D Tiles spec):
//   (x, y, z) -> (x, -z, y)
//   Assimp (East, Up, South) -> (East, -South, Up) = (East, North, Up)
// This produces standard ENU, which the root transform (ENU->ECEF) maps
// directly to correct ECEF. No North-negation is needed.
//

#pragma once

#include "macro.h"
#include <Eigen/Dense>

class MESH_PROJECTION_API AxisMapper
{
public:
    // --- Assimp Y-up <-> ENU ---
    // Assimp (right-handed): X=East, Y=Up, Z=South (toward viewer)
    // ENU:                   X=East, Y=North, Z=Up
    // North = -South, so Z requires sign flip.
    static void AssimpToENU(double x, double y, double z,
                            double& east, double& north, double& up);
    static void ENUToAssimp(double east, double north, double up,
                            double& x, double& y, double& z);

    // Normal vector variant (same axis swap, no translation)
    static void NormalAssimpToENU(double nx, double ny, double nz,
                                  double& ne, double& nn, double& nu);
    static void NormalENUToAssimp(double ne, double nn, double nu,
                                  double& nx, double& ny, double& nz);

    // --- Assimp Y-up <-> 3D Tiles Z-up ---
    // CesiumJS Y_UP_TO_Z_UP matrix: [1,0,0,0; 0,0,-1,0; 0,1,0,0; 0,0,0,1]
    // Maps (X,Y,Z)_assimp -> (X, -Z, Y)_tiles
    static void AssimpToTilesZUp(double x, double y, double z,
                                 double& tx, double& ty, double& tz);

    // Bounding box conversion: Assimp Y-up bbox -> 3D Tiles Z-up bbox
    // Since -max(North) <= -min(North), we negate and swap Y/Z:
    //   out X = in X (East), out Y = -in Z (negated North), out Z = in Y (Up)
    static void BBoxAssimpToTilesZUp(const double bmin[3], const double bmax[3],
                                     double outMin[3], double outMax[3]);

    // --- ENU->ECEF 4x4 root transform ---
    // Builds a column-major 4x4 matrix from an ENU->ECEF rotation and translation.
    // Does NOT encode North-negation — CesiumJS applies Y_UP_TO_Z_UP on top.
    //
    // R_enu: 3x3 rotation matrix, maps (East, North, Up) -> ECEF
    // translation: ECEF position of the origin
    static Eigen::Matrix4d BuildRootTransform(const Eigen::Matrix3d& R_enu,
                                              const Eigen::Vector3d& translation);

    // --- Rotation matrix ENU -> Assimp ---
    // Converts a 3x3 rotation matrix from ENU space to Assimp space.
    // R_enu: columns = East, North, Up directions in ECEF.
    // Output Rc: columns = East, Up, South directions.
    // Formula: Rc = A^T * R_enu * A, where A maps Assimp(E,U,S) -> ENU(E,N,U).
    static Eigen::Matrix3d RotationENUToAssimp(const Eigen::Matrix3d& R_enu);
};
