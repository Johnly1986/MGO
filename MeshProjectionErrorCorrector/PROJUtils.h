#pragma once

#include <string>

// PROJUtils — shared PROJ database discovery, used by MGOConsole, ImageTiler,
// TerrainConverter, TilesConverter, and any other module that needs to locate
// proj.db at runtime.
//
// Single source of truth — previously duplicated verbatim in two modules.
//
// This is a utility function compiled directly into each consuming library
// (no cross-library export).  Add PROJUtils.cpp to each target's sources.

// Find the PROJ database directory (containing proj.db) at runtime.
//
// Search order (cross-platform):
//   1) Executable-relative:   <exeDir>/proj.db
//   2) PROJ_LIB environment variable (allows runtime override)
//   3) MGO_PROJ_DB_DIR compiled in at CMake time
//   4) System/vcpkg common paths
//
// Returns the directory path (without trailing separator), or empty string.
std::string FindPROJDatabase();
