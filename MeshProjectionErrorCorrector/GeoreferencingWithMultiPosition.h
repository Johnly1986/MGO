#pragma once
#include "IGeoreferencing.h"
#include <vector>
#include <string>

struct MESH_PROJECTION_API ControlPoint {

    ControlPoint()
    {
        orign_source_position << 0, 0, 0;
        orign_target_position << 0, 0, 0;
    }

    ControlPoint(double sx, double sy, double sz, double tx, double ty, double tz)
    {
        orign_source_position << sx, sy, sz;
        orign_target_position << tx, ty, tz;
    }

    Eigen::Vector3d orign_source_position;
    Eigen::Vector3d orign_target_position;

    Eigen::Vector3d ecef_source_position;
    Eigen::Vector3d ecef_target_position;
};

/// Fit method for multi-position georeferencing
enum class FitMethod {
    /// PROJ pipelines → ECEF → 3×4 affine → ECEF → geographic.
    /// Requires source CRS so PROJ can resolve the projection.
    ECEF_Affine,

    /// Source (x,y,z) → geographic (lon,lat,h) via 2D polynomial.
    /// No source CRS or PROJ required — fits the mapping directly.
    DirectPoly2D
};

/// Result of automatic CRS detection
struct MESH_PROJECTION_API CRSDetectionResult {
    std::string crs;         ///< PROJ CRS string
    double rms_degrees{0.0}; ///< RMS residual in degrees (lower is better)
    int num_points{0};       ///< Number of control points used for evaluation
    std::string description; ///< Human-readable description
};

class MESH_PROJECTION_API GeoreferencingWithMultiPosition : public IGeoreferencing
{
public:
    GeoreferencingWithMultiPosition(const std::string& srccrs = "", const std::string& targetcrs = "");

    ~GeoreferencingWithMultiPosition();

public:
    void SetParameter(const std::vector<ControlPoint>& controlPositions);

    /// Set fitting method. Use DirectPoly2D when source CRS is unknown.
    void SetFitMethod(FitMethod method);

    /// Set polynomial order (1, 2, or 3). Only used with DirectPoly2D.
    /// Order 1: affine (3 params/dim, needs ≥4 points)
    /// Order 2: quadratic (6 params/dim, needs ≥7 points)
    /// Order 3: cubic (10 params/dim, needs ≥11 points)
    void SetPolyOrder(int order);

    /// Get current fit method
    FitMethod GetFitMethod() const { return m_fitMethod; }

public:
    void Solve() override;

    Eigen::Vector3d Transform(const Eigen::Vector3d& position) override;
    Eigen::Vector3d InverseTransform(const Eigen::Vector3d& target_position) override;

    /// --- Source CRS Auto-Detection ---

    /// Try to detect the source CRS from control points.
    /// Candidates are generated based on the geographic extent of target points.
    std::vector<CRSDetectionResult> DetectSourceCRS(int maxResults = 5);

    /// Get the polynomial coefficients (for inspection/debugging).
    const Eigen::VectorXd& GetPolyLon() const { return m_polyLon; }
    const Eigen::VectorXd& GetPolyLat() const { return m_polyLat; }
    const Eigen::VectorXd& GetPolyH() const { return m_polyH; }

protected:
    /// Build the 2D polynomial basis vector for (x, y) at the given order
    static Eigen::VectorXd Basis2D(int order, double x, double y);

    /// Height basis: [1, x, y, z]
    static Eigen::Vector4d BasisH(double x, double y, double z);

    /// Number of 2D polynomial terms for a given order
    static int NumBasis2D(int order);

    /// Fit polynomial from source → target using current control points
    void SolvePoly();

    /// Fit ECEF affine using current control points
    void SolveECEF();

    /// Try a single CRS candidate and return RMS residual in degrees
    double EvaluateCRS(const std::string& candidateCrs,
                       const std::vector<Eigen::Vector3d>& srcPos,
                       const std::vector<Eigen::Vector3d>& tgtPos);

protected:
    std::vector<ControlPoint>   m_controlPositions;
    Eigen::Matrix4d             m_transform;        // ECEF affine matrix (3x4)

    // Polynomial mode members
    FitMethod                   m_fitMethod{FitMethod::ECEF_Affine};
    int                         m_polyOrder{1};
    Eigen::VectorXd             m_polyLon;          // 2D poly coeffs for longitude
    Eigen::VectorXd             m_polyLat;          // 2D poly coeffs for latitude
    Eigen::VectorXd             m_polyH;            // height coeffs [1, x, y, z]
};
