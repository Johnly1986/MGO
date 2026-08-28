#include "PROJUtils.h"

#include <fstream>
#include <iostream>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#elif defined(__linux__)
#include <unistd.h>
#elif defined(__APPLE__)
#include <unistd.h>
#include <mach-o/dyld.h>
#endif

static bool hasProjDb(const std::string& dir)
{
    if (dir.empty()) return false;
#ifdef _WIN32
    WIN32_FIND_DATAA ffd;
    HANDLE hf = FindFirstFileA((dir + "\\proj.db").c_str(), &ffd);
    if (hf != INVALID_HANDLE_VALUE) { FindClose(hf); return true; }
    hf = FindFirstFileA((dir + "/proj.db").c_str(), &ffd);
    if (hf != INVALID_HANDLE_VALUE) { FindClose(hf); return true; }
    return false;
#else
    std::ifstream test(dir + "/proj.db");
    return test.good();
#endif
}

std::string FindPROJDatabase()
{
    // 1) Executable-relative (highest priority)
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
        char buf[4096];
        uint32_t size = sizeof(buf);
        if (_NSGetExecutablePath(buf, &size) == 0) {
            exeDir = buf;
            auto pos = exeDir.find_last_of('/');
            if (pos != std::string::npos) exeDir = exeDir.substr(0, pos);
        }
#endif
        if (hasProjDb(exeDir))
            return exeDir;
    }

    // 2) PROJ_LIB env var (allows runtime override of compile-time path)
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

    // 3) Compile-time path
#ifdef MGO_PROJ_DB_DIR
    if (hasProjDb(MGO_PROJ_DB_DIR))
        return MGO_PROJ_DB_DIR;
#endif

    // 4) System / vcpkg common paths
    {
#ifdef _WIN32
        const char* roots[3] = {nullptr, nullptr, "C:\\vcpkg"};
        char env[4096];
        if (GetEnvironmentVariableA("VCPKG_ROOT", env, sizeof(env)) > 0)
            roots[0] = env;
        if (GetEnvironmentVariableA("USERPROFILE", env, sizeof(env)) > 0) {
            static std::string s = std::string(env) + "\\vcpkg";
            roots[1] = s.c_str();
        }
        for (auto root : roots) {
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

    return {};
}
