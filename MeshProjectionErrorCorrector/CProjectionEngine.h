// Copyright Johnlyon
//
// CProjectionEngine — unified projection/coordinate engine for 3D Tiles pipeline
//
// Encapsulates all geodetic calculations: .prj parsing, Gauss-Kruger inverse,
// geographic<->ECEF, ENU rotation, per-instance correction, root transform.
// Replaces the inline projection code previously scattered in TilesConverter.
//

#pragma once

#include "macro.h"
#include "TileDataTypes.h"
#include "GeodeticMath.h"
#include <memory>
#include <string>
#include <cmath>
#include <cstring>

class IGeoreferencing;
struct aiScene;

class MESH_PROJECTION_API CProjectionEngine
{
public:
    CProjectionEngine();
    ~CProjectionEngine();

    // Reset to default state (releases georef, clears origin/projection)
    void Reset();

    // --- Initialization ---
    bool LoadProjection(const std::string& prjFile);
    bool LoadProjectionFromString(const std::string& wktOrProjString);
    void SetOrigin(double easting, double northing, double height);
    bool HasProjection() const { return m_hasProjection; }

    // --- External georeferencing bridge ---
    // Set an IGeoreferencing strategy for point→ECEF transformation.
    // When set, ComputeRootTransform and ComputeInstanceProjectionDelta
    // use the georeferencing instead of the built-in GK inverse.
    // The caller must keep the IGeoreferencing alive during the conversion.
    void SetGeoreferencing(IGeoreferencing* georef);
    bool HasGeoreferencing() const { return m_georef != nullptr; }

    // Transform a point to ECEF using the current strategy
    // (IGeoreferencing if set, otherwise built-in GK inverse).
    bool TransformPointToECEF(double x, double y, double z,
                              double& ex, double& ey, double& ez);

    // --- Core geodetic conversions ---
    bool ProjectedToGeographic(double easting, double northing,
                               double& lat, double& lon);
    // TM Forward: (lat, lon in radians) → (Easting, Northing in meters)
    void GeographicToProjected(double lat, double lon,
                               double& easting, double& northing) const;
    void GeographicToECEF(double lat, double lon, double height,
                          double& x, double& y, double& z);
    Eigen::Matrix3d ComputeENUToECEFRotation(double lat, double lon);
    void TransformNormalToECEF(double lat, double lon,
                               double& nx, double& ny, double& nz);

    // --- Composite conversions ---
    void ProjectedToECEF(double easting, double northing, double height,
                         double& x, double& y, double& z);

    // --- Decoupled projection error computation ---
    // Computes the delta (error) at a point relative to an origin, using the
    // engine's projection parameters (ellipsoid, central meridian) and
    // georeferencing strategy (if set). Fully decoupled from engine state:
    // the origin and point are passed as explicit parameters.
    //
    // originE, originN, originZ: origin in projected coordinates (GK easting,
    //   northing, height). Can differ from the engine's m_originX/Y/Z.
    // x, y, z: point in Assimp space (East, Up, North) relative to origin.
    // dx, dy, dz: output delta in Assimp space (East, Up, North).
    // R_correction: optional output, 3×3 rotation matrix
    //   (R_correction = R_origin^T × R_instance). Pass nullptr to skip.
    // Returns: delta magnitude in meters.
    double ComputeProjectionError(double originE, double originN, double originZ,
                                  double x, double y, double z,
                                  double& dx, double& dy, double& dz,
                                  Eigen::Matrix3d* R_correction = nullptr);

    // --- Per-instance projection correction (legacy wrapper) ---
    // Convenience wrapper around ComputeProjectionError using the engine's
    // stored origin (m_originX/Y/Z). Kept for backward compatibility.
    // Input (cx,cy,cz): VERTEX CENTROID in Assimp space (X=East, Y=Up, Z=North).
    double ComputeInstanceProjectionDelta(double cx, double cy, double cz,
                                          double& dx, double& dy, double& dz,
                                          Eigen::Matrix3d& R_correction);

    // Apply correction to a single MeshInstance's worldTransform and bbox.
    // Returns the delta magnitude in meters.
    double ApplyInstanceCorrection(MeshInstance& inst);

    // Batch-apply to all instances, prints summary
    // (origin-based: delta + rotation computed at worldTransform translation,
    //  applied as translation + rotation correction)
    void ApplyPerInstanceProjectionCorrection(MeshInstance* instances,
                                              int count);

    // --- Rebase instances to centroid ---
    // Shifts each mesh's vertices so the vertex centroid is at local (0,0,0).
    // Updates worldTransform translation to the centroid's world position.
    // Must be called BEFORE ApplyPerInstanceProjectionCorrection so that:
    //   1. Delta is computed at the centroid (= local origin after rebase)
    //   2. Rotation correction is centered on the centroid (minimizes lever arm)
    void RebaseInstancesToCentroid(const aiScene* scene,
                                    MeshInstance* instances,
                                    int count);

    // --- Per-vertex projection correction (exact) ---
    // Computes and applies delta for EACH vertex independently, eliminating
    // all curvature residual. Modifies the aiScene's mesh vertices in-place
    // (bakes the per-vertex delta into local-space vertices). Does NOT modify
    // worldTransform - the delta is in the vertices, so worldTransform stays
    // as-is and GroupCellByMaterial applies it normally.
    // Use this instead of ApplyPerInstanceProjectionCorrection for exact
    // positioning. More expensive (O(total_vertices) GK inverse calls).
    void ApplyPerVertexProjectionCorrection(const aiScene* scene,
                                            MeshInstance* instances,
                                            int count);

    // --- Root tileset transform ---
    // Computes 4x4 column-major ENU->ECEF matrix at the origin.
    // Does NOT encode North-negation (CesiumJS handles Y_UP_TO_Z_UP).
    // Returns identity when no projection is configured.
    Eigen::Matrix4d ComputeRootTransform();

    // --- Projection parameters (read-only access) ---
    double GetSemiMajorAxis() const { return m_a; }
    double GetInverseFlattening() const { return m_f_inv; }
    double GetCentralMeridian() const { return m_lambda0; }
    double GetFalseEasting() const { return m_falseE; }
    double GetFalseNorthing() const { return m_falseN; }
    double GetScaleFactor() const { return m_k0; }
    double GetOriginX() const { return m_originX; }
    double GetOriginY() const { return m_originY; }
    double GetOriginZ() const { return m_originZ; }

private:
    // State
    double m_a, m_f_inv, m_lambda0, m_falseE, m_falseN, m_k0;
    double m_originX, m_originY, m_originZ;
    bool m_hasProjection;
    IGeoreferencing* m_georef = nullptr;
    std::unique_ptr<IGeoreferencing> m_ownedGeoref;  // default georef (identity Helmert)
};
