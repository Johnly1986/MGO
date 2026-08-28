// Copyright Johnlyon
//
// AxisMapper — unified coordinate axis conversion utilities

#include "AxisMapper.h"
#include "GeodeticMath.h"

// ---------------------------------------------------------------------------
// Assimp Y-up <-> ENU
// ---------------------------------------------------------------------------
// Assimp (right-handed, standard glTF): X=East, Y=Up, Z=South (toward viewer)
// ENU:                                X=East, Y=North, Z=Up
// North = -South, so Assimp Z needs sign flip when converting to/from ENU.

void AxisMapper::AssimpToENU(double x, double y, double z,
                             double& east, double& north, double& up)
{
    east  = x;       // East  = East
    north = -z;      // North = -South = -Assimp Z
    up    = y;       // Up    = Assimp Y
}

void AxisMapper::ENUToAssimp(double east, double north, double up,
                             double& x, double& y, double& z)
{
    x = east;        // East  = East
    y = up;          // Up    = ENU Z
    z = -north;      // South = -North = -ENU Y
}

void AxisMapper::NormalAssimpToENU(double nx, double ny, double nz,
                                   double& ne, double& nn, double& nu)
{
    ne = nx;
    nn = -nz;        // North normal = -South normal
    nu = ny;
}

void AxisMapper::NormalENUToAssimp(double ne, double nn, double nu,
                                   double& nx, double& ny, double& nz)
{
    nx = ne;
    ny = nu;
    nz = -nn;        // South normal = -North normal
}

// ---------------------------------------------------------------------------
// Assimp Y-up <-> 3D Tiles Z-up (CesiumJS Y_UP_TO_Z_UP)
// ---------------------------------------------------------------------------
// Y_UP_TO_Z_UP = [1,0,0,0; 0,0,-1,0; 0,1,0,0; 0,0,0,1]
// (X, Y, Z)_assimp -> (X, -Z, Y)_tiles

void AxisMapper::AssimpToTilesZUp(double x, double y, double z,
                                  double& tx, double& ty, double& tz)
{
    tx = x;         // East  = East
    ty = -z;        // North = -South = -Assimp Z
    tz = y;         // Up    = Assimp Y
}

void AxisMapper::BBoxAssimpToTilesZUp(const double bmin[3], const double bmax[3],
                                      double outMin[3], double outMax[3])
{
    // X (East): unchanged
    outMin[0] = bmin[0];
    outMax[0] = bmax[0];
    // Y (Tiles) = North = -South = -Assimp Z: negate and swap min/max
    outMin[1] = -bmax[2];
    outMax[1] = -bmin[2];
    // Z (Tiles) = Y (Assimp Up): unchanged
    outMin[2] = bmin[1];
    outMax[2] = bmax[1];
}

// ---------------------------------------------------------------------------
// ENU->ECEF 4x4 root transform
// ---------------------------------------------------------------------------
// R_enu is row-major 3x3: R[row*3+col]
// Column-major 4x4: out[col*4+row]
//
// Column 0 = R * [1,0,0]^T = East  in ECEF
// Column 1 = R * [0,1,0]^T = North in ECEF
// Column 2 = R * [0,0,1]^T = Up    in ECEF
// Column 3 = Translation (ECEF position of origin)

Eigen::Matrix4d AxisMapper::BuildRootTransform(const Eigen::Matrix3d& R_enu,
                                               const Eigen::Vector3d& translation)
{
    return GeodeticMath::BuildRootTransform(R_enu, translation);
}

Eigen::Matrix3d AxisMapper::RotationENUToAssimp(const Eigen::Matrix3d& R_enu)
{
    // A maps Assimp(E,U,S) -> ENU(E,N,U):
    //   ENU_E = Assimp_E,  ENU_N = -Assimp_S,  ENU_U = Assimp_U
    // A = [[1,0,0],[0,0,-1],[0,1,0]]
    // Rc = A^T * R_enu * A
    Eigen::Matrix3d A;
    A << 1.0, 0.0, 0.0,
         0.0, 0.0, -1.0,
         0.0, 1.0, 0.0;
    return A.transpose() * R_enu * A;
}
