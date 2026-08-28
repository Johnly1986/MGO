// Copyright Johnlyon
//
// CRSTransformer - general coordinate reference system transformer
//
// Transforms coordinates between arbitrary CRS pairs (EPSG codes, WKT,
// PROJ strings) routed through ECEF (EPSG:4978) as a hub, with axis order
// normalized to "visualization" order on both ends:
//   x = longitude / easting, y = latitude / northing, z = height (meters)
//
// ENU local tangent planes are supported as first-class "CRS" via the
// syntax "ENU:lat,lon[,height]" (WGS84 ellipsoid, PROJ topocentric):
//   x = east (m), y = north (m), z = up (m) relative to (lat, lon, h).
// The same syntax is accepted by SetSourceCRS / SetTargetCRS, plus the
// explicit SetSourceENU / SetTargetENU overloads.
//

#pragma once

#include "macro.h"

#include <string>

// Opaque PROJ handles (PJ / PJ_CONTEXT) - kept as void* so this public
// header does not need proj.h (the shim conflicts with forward decls).

class MESH_PROJECTION_API CRSTransformer
{
public:
    CRSTransformer();
    ~CRSTransformer();

    CRSTransformer(const CRSTransformer&) = delete;
    CRSTransformer& operator=(const CRSTransformer&) = delete;

    // Accepted CRS spec forms (empty string = EPSG:4326):
    //   "EPSG:<code>"           e.g. "EPSG:4547"
    //   "ENU:<lat>,<lon>[,<h>]" e.g. "ENU:22.64785,113.06277" (degrees)
    //   WKT string              (PROJCS/GEOGCS/...)
    //   PROJ string             "+proj=..." (+type=crs appended if missing)
    bool SetSourceCRS(const std::string& crs);
    bool SetTargetCRS(const std::string& crs);

    bool SetSourceENU(double latDeg, double lonDeg, double hMeters = 0.0);
    bool SetTargetENU(double latDeg, double lonDeg, double hMeters = 0.0);

    bool Ready() const;

    // True when source and target specs resolve to the same CRS: callers
    // may skip transformation entirely (values pass through unchanged).
    bool IsIdentity() const;

    // Transform in place. Input/output in visualization order:
    //   geographic CRS:  x = lon (deg), y = lat (deg)
    //   projected CRS:   x = easting (m), y = northing (m)
    //   ENU:             x = east (m), y = north (m), z = up (m)
    // Returns false on error (details in GetLastError; coordinates may be
    // left partially modified).
    bool Transform(double& x, double& y, double& z);

    const std::string& GetLastError() const { return m_lastError; }

    // "ENU:lat,lon[,h]" -> parts. Returns false if not an ENU spec.
    static bool ParseENUSpec(const std::string& crs,
                             double& latDeg, double& lonDeg, double& hMeters);

private:
    // Build a PJ that maps crs -> ECEF (fromSrc=true) or ECEF -> crs.
    void* buildToECEF(const std::string& crsSpec);
    void* buildFromECEF(const std::string& crsSpec);
    bool  build(const std::string& srcSpec, const std::string& dstSpec);

    void* m_ctx = nullptr;       // PJ_CONTEXT*
    void* m_srcToECEF = nullptr; // PJ*
    void* m_ecefToDst = nullptr; // PJ*
    std::string m_srcSpec;
    std::string m_dstSpec;
    std::string m_lastError;
};
