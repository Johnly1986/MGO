// Copyright Johnlyon
//
// MGO software version + copyright — single source of truth. The same macros
// are consumed by:
//   - MGOConsole (startup banner / `mgo version`)
//   - MGOVersion.rc (Windows VERSIONINFO embedded in exported DLLs)
//   - CMake VERSION/SOVERSION (Linux/macOS SO SONAME, see CMakeLists.txt)

#ifndef MGO_VERSION_H
#define MGO_VERSION_H

#define MGO_VERSION_MAJOR 1
#define MGO_VERSION_MINOR 0
#define MGO_VERSION_PATCH 0

// Dot-separated string, e.g. "1.0.0" (console banner + VERSIONINFO strings).
#define MGO_VERSION_STRING "1.0.0"

// Comma-separated numeric form for Windows VERSIONINFO FILEVERSION /
// PRODUCTVERSION (consumed by rc.exe / windres only).
#define MGO_VERSION_FILE 1,0,0,0

#define MGO_COPYRIGHT_STRING "Copyright Johnlyon"

#endif
