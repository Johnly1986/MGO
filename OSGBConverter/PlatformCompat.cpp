// Copyright Johnlyon
//
// PlatformCompat implementation
//

#include "PlatformCompat.h"

#include <algorithm>
#include <cstring>

#ifdef _WIN32
// Windows implementation using FindFirstFile/FindNextFile

std::vector<DirEntry> DirectoryLister::List(const std::string& path)
{
    std::vector<DirEntry> entries;
    std::string searchPath = path + "\\*";

    WIN32_FIND_DATAA fd;
    HANDLE hFind = FindFirstFileA(searchPath.c_str(), &fd);
    if (hFind == INVALID_HANDLE_VALUE) return entries;

    do {
        if (strcmp(fd.cFileName, ".") == 0 || strcmp(fd.cFileName, "..") == 0)
            continue;

        DirEntry e;
        e.name = fd.cFileName;
        e.isDirectory = (fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != 0;
        e.isFile = !e.isDirectory;
        if (e.isFile) {
            e.fileSize = (static_cast<uint64_t>(fd.nFileSizeHigh) << 32) | fd.nFileSizeLow;
        }
        entries.push_back(e);
    } while (FindNextFileA(hFind, &fd));

    FindClose(hFind);
    return entries;
}

bool DirectoryLister::IsDirectory(const std::string& path)
{
    DWORD attr = GetFileAttributesA(path.c_str());
    return (attr != INVALID_FILE_ATTRIBUTES) && (attr & FILE_ATTRIBUTE_DIRECTORY);
}

bool DirectoryLister::Exists(const std::string& path)
{
    DWORD attr = GetFileAttributesA(path.c_str());
    return attr != INVALID_FILE_ATTRIBUTES;
}

bool DirectoryLister::MakePath(const std::string& path)
{
    // Recursively create parent directories
    for (size_t i = 0; i < path.size(); ++i) {
        if (path[i] == '\\' || path[i] == '/') {
            if (i == 0) continue;
            std::string parent = path.substr(0, i);
            if (!IsDirectory(parent)) {
                _mkdir(parent.c_str());
            }
        }
    }
    return _mkdir(path.c_str()) == 0 || IsDirectory(path);
}

#else
// POSIX implementation using opendir/readdir

#include <dirent.h>
#include <sys/stat.h>

std::vector<DirEntry> DirectoryLister::List(const std::string& path)
{
    std::vector<DirEntry> entries;
    DIR* dir = opendir(path.c_str());
    if (!dir) return entries;

    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr)
    {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
            continue;

        DirEntry e;
        e.name = entry->d_name;

        std::string fullPath = path + "/" + entry->d_name;
        struct stat st;
        if (stat(fullPath.c_str(), &st) == 0)
        {
            e.isDirectory = S_ISDIR(st.st_mode);
            e.isFile = S_ISREG(st.st_mode);
            e.fileSize = static_cast<uint64_t>(st.st_size);
        }
        entries.push_back(e);
    }
    closedir(dir);
    return entries;
}

bool DirectoryLister::IsDirectory(const std::string& path)
{
    struct stat st;
    return stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode);
}

bool DirectoryLister::Exists(const std::string& path)
{
    struct stat st;
    return stat(path.c_str(), &st) == 0;
}

bool DirectoryLister::MakePath(const std::string& path)
{
    std::string cmd = "mkdir -p \"" + path + "\"";
    return system(cmd.c_str()) == 0;
}

#endif