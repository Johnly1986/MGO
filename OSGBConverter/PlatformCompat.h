// Copyright Johnlyon
//
// PlatformCompat — cross-platform filesystem utilities
//
// Abstracts POSIX/Win32 differences for directory scanning and filesystem ops.
// All OSGBConverter modules should use these instead of <dirent.h> or <sys/stat.h>.
//

#pragma once

#include <string>
#include <vector>
#include <cstdint>

#ifdef _WIN32
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#  include <windows.h>
#  include <direct.h>
#  define strcasecmp _stricmp
#  define strncasecmp _strnicmp
#  define mkdir_p(path) _mkdir(path)
#else
#  include <sys/stat.h>
#  include <cstring>
#  define mkdir_p(path) mkdir(path, 0755)
#endif

// ---------------------------------------------------------------------------
// Directory entry (platform-independent)
// ---------------------------------------------------------------------------
struct DirEntry
{
    std::string name;
    bool        isDirectory = false;
    bool        isFile      = false;
    uint64_t    fileSize    = 0;
};

// ---------------------------------------------------------------------------
// Directory listing
// ---------------------------------------------------------------------------
class DirectoryLister
{
public:
    // List entries in a directory. Returns empty vector on failure.
    static std::vector<DirEntry> List(const std::string& path);

    // Check if a path exists and is a directory.
    static bool IsDirectory(const std::string& path);

    // Check if a path exists (file or directory).
    static bool Exists(const std::string& path);

    // Create a directory (including parents if needed).
    static bool MakePath(const std::string& path);
};