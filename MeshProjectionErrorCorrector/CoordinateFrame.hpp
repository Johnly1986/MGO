// Copyright Johnlyon
//
// CoordinateFrame — explicit coordinate system labels for MGO
//
// Every point, vector, and bounding box in the codebase lives in one of these
// frames.  Passing a frame enum alongside coordinates makes the axis convention
// explicit and enables a single CoordinateTransform class to replace the three
// inconsistent YZ-swap implementations currently scattered across the codebase.
//
// Axis conventions for each frame:
//
//   Frame            X       Y       Z
//   ─────────────────────────────────────
//   AssimpYUp        East    Up      South   (default Assimp import)
//   ZUp              East    North   Up      (pre-rotated models)
//   Projected        East    North   Height  (source CRS, meters)
//   Geographic       Lon     Lat     Height  (degrees, meters)
//   ECEF             X       Y       Z       (EPSG:4978)
//   ENU              East    North   Up      (local tangent plane)
//   TilesZUp         East    North   Up      (= ENU; Cesium applies Y_UP_TO_Z_UP)
//

#pragma once

namespace MGO {

enum class CoordinateFrame {
    // ---- Model input frames ----
    AssimpYUp,      // X=East, Y=Up, Z=South  (default Assimp import convention)
    ZUp,            // X=East, Y=North, Z=Up   (pre-rotated e.g. M1 roadbed model)

    // ---- Geodetic frames ----
    Projected,      // Easting, Northing, Height (source CRS, meters)
    Geographic,     // Longitude, Latitude, Height (degrees, meters; EPSG:4979)
    ECEF,           // Earth-Centered Earth-Fixed (EPSG:4978)

    // ---- Local / output frames ----
    ENU,            // East, North, Up (local tangent plane)
    TilesZUp,       // 3D Tiles Z-up (= ENU; CesiumJS applies Y_UP_TO_Z_UP at load)
};

} // namespace MGO
