#pragma once
#include "macro.h"
#include "IGeoreferencing.h"
#include "Constants.h"
#include <memory>
#include <string>

// ---------------------------------------------------------------------------
// GeoreferencingType - georeferencing strategy enumeration
// ---------------------------------------------------------------------------
enum class GeoreferencingType
{
    /// No georeferencing (use plain projection from .prj file alone).
    None,

    /// Identity (plain projection, no datum shift).
    /// Use case: source CRS and target CRS use the same datum (e.g. CGCS2000
    /// projected -> CGCS2000 geographic). The .prj file alone is sufficient.
    Identity,

    /// 7-parameter Helmert datum shift.
    /// Use case: source datum differs from target datum (e.g. CGCS2000 ->
    /// WGS84). Requires 3 translations (mx,my,mz in meters), 3 rotations
    /// (rx,ry,rz in arc-seconds), and 1 scale factor (ppm).
    /// Set via GeoreferencingOptions::helmert[7].
    SevenParam,

    /// Single anchor point ECEF offset.
    /// Use case: one known correspondence between source and target ECEF
    /// positions. Computes a constant ECEF translation vector.
    /// Set via GeoreferencingOptions::anchorX/Y/Z (source projected coords).
    Anchor,

    /// Multi-control-point least-squares fit.
    /// Use case: multiple known correspondences (>= 3 pairs) between source
    /// and target positions. Fits an ECEF affine or 2D polynomial transform.
    /// Control points are set externally after creation via
    /// GeoreferencingWithMultiPosition::SetParameter().
    MultiPosition,
};

// ---------------------------------------------------------------------------
// GeoreferencingOptions - parameters for georeferencing creation
// ---------------------------------------------------------------------------
struct GeoreferencingOptions
{
    /// Georeferencing strategy to use.
    /// Also mirrored in TilesConverterOptions/TerrainConverterOptions/
    /// OSGBConverterOptions for CLI compatibility; prefer using the shared
    /// GeoreferencingOptions struct in new code.
    GeoreferencingType georefType = GeoreferencingType::None;

    /// 7-parameter Helmert values. Only used when type == SevenParam.
    /// [0..2] = mx,my,mz translations (meters)
    /// [3..5] = rx,ry,rz rotations (arc-seconds)
    /// [6]    = scale factor (ppm, e.g. 1.0 = 1ppm)
    double helmert[7] = {0, 0, 0, 0, 0, 0, 0};

    /// Anchor source position in projected coordinates.
    /// Only used when type == Anchor.
    double anchorX = 0, anchorY = 0, anchorZ = 0;

    /// Control points CSV file path.
    std::string cpsFile;

    /// Projection .prj file path or WKT/PROJ string content.
    std::string prjFile;

    /// Target CRS identifier (PROJ string or EPSG code).
    /// Default: "EPSG:4979" (WGS84 geographic 3D).
    std::string targetCrs = CRS::WGS84_GEOGRAPHIC_3D;
};

// ---------------------------------------------------------------------------
// GeoreferencingFactory - creates IGeoreferencing instances
// ---------------------------------------------------------------------------
class MESH_PROJECTION_API GeoreferencingFactory
{
public:
    /// Create a georeferencing instance.
    /// \param type  Strategy to use (see GeoreferencingType enum).
    /// \param prjFile  .prj WKT file path defining the source CRS.
    /// \param opts  Parameters for the selected strategy.
    /// \return Owned IGeoreferencing, or nullptr on failure.
    static std::unique_ptr<IGeoreferencing> Create(
        GeoreferencingType type,
        const std::string& prjFile,
        const GeoreferencingOptions& opts = {});
};
