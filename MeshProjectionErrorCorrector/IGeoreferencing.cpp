#include "IGeoreferencing.h"
#include "Constants.h"
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <cstdlib>
#include <fstream>
#ifdef _WIN32
#include <Windows.h>
#elif defined(__APPLE__)
#include <mach-o/dyld.h>
#include <unistd.h>
#elif defined(__linux__)
#include <unistd.h>
#endif
#include <proj/proj.h>
#include "UtilTools.h"

// ---------------------------------------------------------------------------
// Find the PROJ database directory (containing proj.db) at runtime.
//
// Search order (cross-platform):
//   1) MGO_PROJ_DB_DIR compiled in at CMake time (the PROJ install we built
//      against — most reliable when the build machine matches the target)
//   2) Executable-relative:   <exeDir>/proj.db
//   3) PROJ_LIB env var
//   4) System common paths:   /usr/share/proj, vcpkg, homebrew, etc.
//
// Returns the directory path (without trailing separator), or empty string.
// ---------------------------------------------------------------------------
static std::string FindPROJDatabase()
{
    // Helper: check if a directory contains proj.db
    auto hasProjDb = [](const std::string& dir) -> bool {
        if (dir.empty()) return false;
        std::string path = dir + "/proj.db";
#ifdef _WIN32
        WIN32_FIND_DATAA ffd;
        HANDLE hf = FindFirstFileA((dir + "\\proj.db").c_str(), &ffd);
        if (hf != INVALID_HANDLE_VALUE) { FindClose(hf); return true; }
        // also try forward slash (some Windows paths use mixed separators)
        hf = FindFirstFileA(path.c_str(), &ffd);
        if (hf != INVALID_HANDLE_VALUE) { FindClose(hf); return true; }
        return false;
#else
        std::ifstream test(path);
        return test.good();
#endif
    };

    // —— 1) Compile-time path ——
#ifdef MGO_PROJ_DB_DIR
    if (hasProjDb(MGO_PROJ_DB_DIR))
        return MGO_PROJ_DB_DIR;
#endif

    // —— 2) Executable-relative ——
    {
        std::string exeDir;
#ifdef _WIN32
        char modPath[MAX_PATH];
        DWORD len = GetModuleFileNameA(NULL, modPath, MAX_PATH);
        if (len > 0 && len < MAX_PATH) {
            exeDir = modPath;
            auto pos = exeDir.find_last_of("\\/");
            if (pos != std::string::npos) exeDir = exeDir.substr(0, pos);
        }
#elif defined(__linux__)
        char buf[4096];
        ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
        if (n > 0) {
            buf[n] = '\0';
            exeDir = buf;
            auto pos = exeDir.find_last_of('/');
            if (pos != std::string::npos) exeDir = exeDir.substr(0, pos);
        }
#elif defined(__APPLE__)
        // macOS: use _NSGetExecutablePath or fallback to known paths
        char buf[4096];
        uint32_t size = sizeof(buf);
        if (_NSGetExecutablePath(buf, &size) == 0) {
            exeDir = buf;
            auto pos = exeDir.find_last_of('/');
            if (pos != std::string::npos) exeDir = exeDir.substr(0, pos);
        }
#endif
        // Also try relative to the library directory (for DLL builds)
        if (hasProjDb(exeDir))
            return exeDir;
    }

    // —— 3) PROJ_LIB environment variable ——
    {
#ifdef _WIN32
        char buf[4096];
        DWORD len = GetEnvironmentVariableA("PROJ_LIB", buf, (DWORD)sizeof(buf));
        if (len > 0 && len < sizeof(buf) && hasProjDb(buf))
            return buf;
#else
        const char* env = getenv("PROJ_LIB");
        if (env && hasProjDb(env))
            return env;
#endif
    }

    // —— 4) System common paths ——
    {
#ifdef _WIN32
        // vcpkg user-wide and system-wide
        const char* vcpkgRoots[3] = {nullptr, nullptr, "C:\\vcpkg"};
        char env[4096];
        if (GetEnvironmentVariableA("VCPKG_ROOT", env, sizeof(env)) > 0)
            vcpkgRoots[0] = env;
        if (GetEnvironmentVariableA("USERPROFILE", env, sizeof(env)) > 0) {
            static std::string s = std::string(env) + "\\vcpkg";
            vcpkgRoots[1] = s.c_str();
        }
        for (auto root : vcpkgRoots) {
            if (!root) continue;
            std::string dir = std::string(root) + "\\installed\\x64-windows\\share\\proj";
            if (hasProjDb(dir)) return dir;
        }
#else
        const char* candidates[] = {
            "/usr/share/proj",
            "/usr/local/share/proj",
            "/opt/share/proj",
            "/opt/homebrew/share/proj",
        };
        for (auto dir : candidates) {
            if (hasProjDb(dir)) return dir;
        }
#endif
    }

    return {};  // not found
}

IGeoreferencing::IGeoreferencing()
    : IGeoreferencing("")
{
}

IGeoreferencing::IGeoreferencing(const std::string& srccrs)
    : IGeoreferencing(srccrs, "")
{
}

IGeoreferencing::IGeoreferencing(const std::string& srccrs, const std::string& targetsrc)
    : m_srccrs(srccrs)
    , m_targetcrs(targetsrc)
    , m_ecefcrs(CRS::WGS84_ECEF)
{
    if(m_srccrs.empty()) m_srccrs = m_ecefcrs;
    if(m_targetcrs.empty()) m_targetcrs = m_ecefcrs;

    m_pj_context = proj_context_create();
    m_pj_SourceToECEF = nullptr;
    m_pj_ECEFToTarget = nullptr;

    // Explicitly set the PROJ database path on this context.
    // Bypasses the PROJ_LIB environment-variable dance — more robust
    // on Windows where PROJ_LIB is often not set at install time.
    std::string dbDir = FindPROJDatabase();
    if (!dbDir.empty()) {
        std::string dbPath = dbDir + "/proj.db";
        proj_context_set_database_path(m_pj_context, dbPath.c_str(),
                                        nullptr, nullptr);
    }
}

IGeoreferencing::~IGeoreferencing()
{
    if (m_pj_SourceToECEF) proj_destroy(m_pj_SourceToECEF);
    if (m_pj_ECEFToTarget) proj_destroy(m_pj_ECEFToTarget);
    if (m_pj_context) proj_context_destroy(m_pj_context);
}

bool IGeoreferencing::InitPROJPipelines()
{
    // Clean up any existing transforms
    if (m_pj_SourceToECEF) { proj_destroy(m_pj_SourceToECEF); m_pj_SourceToECEF = nullptr; }
    if (m_pj_ECEFToTarget) { proj_destroy(m_pj_ECEFToTarget); m_pj_ECEFToTarget = nullptr; }

    // Source CRS → ECEF (EPSG:4978)
    m_pj_SourceToECEF = proj_create_crs_to_crs(m_pj_context, m_srccrs.c_str(), m_ecefcrs.c_str(), nullptr);
    if (!m_pj_SourceToECEF) {
        int err = proj_context_errno(m_pj_context);
        std::cerr << "Warning: Failed to create PROJ transform: Source CRS → ECEF"
                  << "\n  Source: " << m_srccrs.substr(0, 200)
                  << "\n  Target: " << m_ecefcrs
                  << "\n  Error (" << err << "): "
                  << (err ? proj_errno_string(err) : "unknown") << std::endl;
        return false;
    }

    // ECEF → Target CRS
    m_pj_ECEFToTarget = proj_create_crs_to_crs(m_pj_context, m_ecefcrs.c_str(), m_targetcrs.c_str(), nullptr);
    if (!m_pj_ECEFToTarget) {
        std::cerr << "Warning: Failed to create PROJ transform: ECEF → Target CRS" << std::endl;
        std::cerr << "  Target CRS: " << m_targetcrs << std::endl;
        proj_destroy(m_pj_SourceToECEF); m_pj_SourceToECEF = nullptr;
        return false;
    }

    return true;
}

Eigen::Vector3d IGeoreferencing::TransformTargetToECEF(const Eigen::Vector3d& target_position)
{
    if (m_targetcrs == m_ecefcrs)
        return target_position;
    if (!m_pj_ECEFToTarget) {
        std::cerr << "TransformTargetToECEF: m_pj_ECEFToTarget is null" << std::endl;
        return target_position;
    }
    // EPSG:4979 uses latitude-first axis order (lat, lon, h).
    // Use EigenToCoordGeo to swap (lon,lat,h) → (lat,lon,h) for PROJ.
    PJ_COORD coord = ConvertTool::EigenToCoordGeo(target_position);
    // m_pj_ECEFToTarget goes ECEF→target, so PJ_INV goes target→ECEF
    PJ_COORD result = proj_trans(m_pj_ECEFToTarget, PJ_INV, coord);
    if (proj_errno(m_pj_ECEFToTarget))
        return target_position;
    return ConvertTool::CoordToEigen(result);
}

Eigen::Vector3d IGeoreferencing::InverseTransform(const Eigen::Vector3d& target_position)
{
    // target geographic -> ECEF
    Eigen::Vector3d ecef = TransformTargetToECEF(target_position);
    if (!m_pj_SourceToECEF) {
        std::cerr << "InverseTransform: m_pj_SourceToECEF is null" << std::endl;
        return target_position;
    }
    // ECEF -> source CRS (PJ_INV of source->ECEF pipeline)
    PJ_COORD coord = proj_coord(ecef.x(), ecef.y(), ecef.z(), 0);
    PJ_COORD result = proj_trans(m_pj_SourceToECEF, PJ_INV, coord);
    if (proj_errno(m_pj_SourceToECEF))
        return target_position;
    return ConvertTool::CoordToEigen(result);
}

Eigen::Matrix4d IGeoreferencing::GCSNormal(const Eigen::Vector3d& source_position) {
    const double dv = 0.5;

    Eigen::Vector3d px = TransformTargetToECEF(Transform(Eigen::Vector3d(source_position.x() - dv, source_position.y(), source_position.z())));
    Eigen::Vector3d nx = TransformTargetToECEF(Transform(Eigen::Vector3d(source_position.x() + dv, source_position.y(), source_position.z())));
    Eigen::Vector3d ax = nx - px;
    ax.stableNormalize();

    Eigen::Vector3d py = TransformTargetToECEF(Transform(Eigen::Vector3d(source_position.x(), source_position.y() - dv, source_position.z())));
    Eigen::Vector3d ny = TransformTargetToECEF(Transform(Eigen::Vector3d(source_position.x(), source_position.y() + dv, source_position.z())));
    Eigen::Vector3d ay = ny - py;
    ay.stableNormalize();


    Eigen::Vector3d origin = TransformTargetToECEF(Transform(source_position));

    Eigen::Vector3d az = ax.cross(ay);

    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3, 3>(0, 0) << ax, ay, az;
    transform.block<3, 1>(0, 3) = origin;

    return transform;
}