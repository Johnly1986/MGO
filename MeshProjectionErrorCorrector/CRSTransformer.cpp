// Copyright Johnlyon
//
// CRSTransformer implementation
//

#include "CRSTransformer.h"
#include "PROJUtils.h"

#include <proj/proj.h>

#include <cctype>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <sstream>

// ---------------------------------------------------------------------------

namespace {

std::string trimmedLower(std::string s)
{
    size_t b = s.find_first_not_of(" \t\r\n");
    size_t e = s.find_last_not_of(" \t\r\n");
    if (b == std::string::npos) return "";
    std::string t = s.substr(b, e - b + 1);
    for (auto& c : t) c = static_cast<char>(std::tolower(c));
    return t;
}

// Append +type=crs to a PROJ string so it can act as a CRS definition.
std::string normalizeProjString(const std::string& crs)
{
    std::string t = trimmedLower(crs);
    if (t.rfind("+proj=", 0) != 0) return crs;
    if (t.find("+type=crs") != std::string::npos) return crs;
    return crs + " +type=crs";
}

// Extract an EPSG code from a legacy GeoJSON "crs" name, e.g.
// "urn:ogc:def:crs:EPSG::4547", "urn:ogc:def:crs:EPSG:6.12:4547",
// "EPSG:4547". Returns "EPSG:<code>" or "".
std::string epsgFromCrsName(const std::string& name)
{
    std::string lower;
    for (char c : name) lower += static_cast<char>(std::tolower(c));

    size_t pos = lower.find("epsg");
    if (pos == std::string::npos) return "";

    // Walk forward to the first digit run after the "epsg" marker.
    size_t i = pos + 4;
    while (i < lower.size() && !std::isdigit(lower[i])) ++i;
    size_t start = i;
    while (i < lower.size() && std::isdigit(lower[i])) ++i;
    if (start == i) return "";
    return "EPSG:" + name.substr(start, i - start);
}

} // anonymous namespace

// ---------------------------------------------------------------------------

CRSTransformer::CRSTransformer()
{
    m_ctx = proj_context_create();
    std::string dbDir = FindPROJDatabase();
    if (!dbDir.empty() && m_ctx)
    {
        const char* paths[1] = { dbDir.c_str() };
        proj_context_set_search_paths(reinterpret_cast<PJ_CONTEXT*>(m_ctx), 1, paths);
    }
}

CRSTransformer::~CRSTransformer()
{
    if (m_srcToECEF) proj_destroy(reinterpret_cast<PJ*>(m_srcToECEF));
    if (m_ecefToDst) proj_destroy(reinterpret_cast<PJ*>(m_ecefToDst));
    if (m_ctx) proj_context_destroy(reinterpret_cast<PJ_CONTEXT*>(m_ctx));
}

bool CRSTransformer::ParseENUSpec(const std::string& crs,
                                  double& latDeg, double& lonDeg, double& hMeters)
{
    if (crs.size() <= 4) return false;
    for (int i = 0; i < 4; ++i)
        if (std::tolower(static_cast<unsigned char>(crs[i])) != "enu:"[i])
            return false;

    std::string body = crs.substr(4);
    double v[3] = { 0, 0, 0 };
    int n = 0;
    std::istringstream ss(body);
    for (std::string tok; std::getline(ss, tok, ',') && n < 3;)
    {
        try { v[n] = std::stod(tok); }
        catch (...) { return false; }
        ++n;
    }
    if (n < 2) return false;
    latDeg = v[0];
    lonDeg = v[1];
    hMeters = v[2];
    return true;
}

// ENU -> ECEF: single inverse topocentric step (input E,N,U in meters).
static std::string enuToEceFPipeline(double lat, double lon, double h)
{
    std::ostringstream os;
    os.precision(12);
    os << "+proj=pipeline"
       << " +step +proj=topocentric +inv +ellps=WGS84"
       << " +lat_0=" << lat << " +lon_0=" << lon << " +h_0=" << h;
    return os.str();
}

// ECEF -> ENU: single topocentric step (output E,N,U in meters).
static std::string ecefToEnuPipeline(double lat, double lon, double h)
{
    std::ostringstream os;
    os.precision(12);
    os << "+proj=pipeline"
       << " +step +proj=topocentric +ellps=WGS84"
       << " +lat_0=" << lat << " +lon_0=" << lon << " +h_0=" << h;
    return os.str();
}

void* CRSTransformer::buildToECEF(const std::string& crsSpec)
{
    double lat, lon, h;
    if (ParseENUSpec(crsSpec, lat, lon, h))
        return proj_create(reinterpret_cast<PJ_CONTEXT*>(m_ctx), enuToEceFPipeline(lat, lon, h).c_str());

    std::string spec = (crsSpec.empty()) ? std::string("EPSG:4326") : crsSpec;
    std::string epsg = epsgFromCrsName(spec);
    if (!epsg.empty()) spec = epsg;
    spec = normalizeProjString(spec);

    PJ* pj = proj_create_crs_to_crs(reinterpret_cast<PJ_CONTEXT*>(m_ctx), spec.c_str(), "EPSG:4978", nullptr);
    if (!pj)
    {
        m_lastError = "Cannot create CRS->ECEF transform for: " + crsSpec;
        return nullptr;
    }
    PJ* norm = proj_normalize_for_visualization(reinterpret_cast<PJ_CONTEXT*>(m_ctx), pj);
    proj_destroy(pj);
    if (!norm)
    {
        m_lastError = "Cannot normalize axis order for: " + crsSpec;
        return nullptr;
    }
    return norm;
}

void* CRSTransformer::buildFromECEF(const std::string& crsSpec)
{
    double lat, lon, h;
    if (ParseENUSpec(crsSpec, lat, lon, h))
        return proj_create(reinterpret_cast<PJ_CONTEXT*>(m_ctx), ecefToEnuPipeline(lat, lon, h).c_str());

    std::string spec = (crsSpec.empty()) ? std::string("EPSG:4326") : crsSpec;
    std::string epsg = epsgFromCrsName(spec);
    if (!epsg.empty()) spec = epsg;
    spec = normalizeProjString(spec);

    PJ* pj = proj_create_crs_to_crs(reinterpret_cast<PJ_CONTEXT*>(m_ctx), "EPSG:4978", spec.c_str(), nullptr);
    if (!pj)
    {
        m_lastError = "Cannot create ECEF->CRS transform for: " + crsSpec;
        return nullptr;
    }
    PJ* norm = proj_normalize_for_visualization(reinterpret_cast<PJ_CONTEXT*>(m_ctx), pj);
    proj_destroy(pj);
    if (!norm)
    {
        m_lastError = "Cannot normalize axis order for: " + crsSpec;
        return nullptr;
    }
    return norm;
}

bool CRSTransformer::build(const std::string& srcSpec, const std::string& dstSpec)
{
    if (m_srcToECEF) { proj_destroy(reinterpret_cast<PJ*>(m_srcToECEF)); m_srcToECEF = nullptr; }
    if (m_ecefToDst) { proj_destroy(reinterpret_cast<PJ*>(m_ecefToDst)); m_ecefToDst = nullptr; }

    m_srcToECEF = buildToECEF(srcSpec);
    if (!m_srcToECEF) return false;
    m_ecefToDst = buildFromECEF(dstSpec);
    if (!m_ecefToDst) return false;
    return true;
}

bool CRSTransformer::SetSourceCRS(const std::string& crs)
{
    m_srcSpec = crs;
    return build(m_srcSpec, m_dstSpec.empty() ? std::string("EPSG:4326") : m_dstSpec);
}

bool CRSTransformer::SetTargetCRS(const std::string& crs)
{
    m_dstSpec = crs;
    return build(m_srcSpec.empty() ? std::string("EPSG:4326") : m_srcSpec, m_dstSpec);
}

bool CRSTransformer::SetSourceENU(double latDeg, double lonDeg, double hMeters)
{
    std::ostringstream os;
    os.precision(12);
    os << "ENU:" << latDeg << "," << lonDeg << "," << hMeters;
    return SetSourceCRS(os.str());
}

bool CRSTransformer::SetTargetENU(double latDeg, double lonDeg, double hMeters)
{
    std::ostringstream os;
    os.precision(12);
    os << "ENU:" << latDeg << "," << lonDeg << "," << hMeters;
    return SetTargetCRS(os.str());
}

bool CRSTransformer::Ready() const
{
    return m_srcToECEF != nullptr && m_ecefToDst != nullptr;
}

bool CRSTransformer::IsIdentity() const
{
    if (!Ready()) return false;
    std::string src = trimmedLower(m_srcSpec.empty() ? "epsg:4326" : m_srcSpec);
    std::string dst = trimmedLower(m_dstSpec.empty() ? "epsg:4326" : m_dstSpec);
    if (src == dst) return true;
    // Normalize "EPSG:4326" vs "epsg:4326" vs "EPSG: 4326"
    auto strip = [](std::string s) {
        std::string out;
        for (char c : s)
            if (!std::isspace(static_cast<unsigned char>(c))) out += c;
        return out;
    };
    return strip(src) == strip(dst);
}

bool CRSTransformer::Transform(double& x, double& y, double& z)
{
    if (!Ready())
    {
        m_lastError = "CRSTransformer not initialized";
        return false;
    }

    PJ_COORD c = proj_coord(x, y, z, 0);
    PJ_COORD ecef = proj_trans(reinterpret_cast<PJ*>(m_srcToECEF), PJ_FWD, c);
    if (proj_errno(reinterpret_cast<PJ*>(m_srcToECEF)))
    {
        m_lastError = "Source CRS transform failed";
        proj_errno_reset(reinterpret_cast<PJ*>(m_srcToECEF));
        return false;
    }
    PJ_COORD out = proj_trans(reinterpret_cast<PJ*>(m_ecefToDst), PJ_FWD, ecef);
    if (proj_errno(reinterpret_cast<PJ*>(m_ecefToDst)))
    {
        m_lastError = "Target CRS transform failed";
        proj_errno_reset(reinterpret_cast<PJ*>(m_ecefToDst));
        return false;
    }
    x = out.xyz.x;
    y = out.xyz.y;
    z = out.xyz.z;
    return true;
}
