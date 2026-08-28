cmake_minimum_required(VERSION 3.18...3.31)
project(MGO LANGUAGES C CXX)

# ---- Cross-platform build ----
# Linux:   cmake -B build && cmake --build build
# macOS:   brew install assimp boost proj eigen libtiff libjpeg libpng && cmake -B build && cmake --build build
# Windows: vcpkg install <deps> && cmake -B build -DCMAKE_TOOLCHAIN_FILE=%VCPKG_ROOT%/scripts/buildsystems/vcpkg.cmake && cmake --build build

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ---- Unified output directory (all binaries in one place) ----
# On Windows, CMake scatters DLLs across module subdirectories; the EXE
# can't find them at runtime.  Executables go under build/bin/ (DLLs too
# on Windows); shared/static libraries stay under build/.  The bin/ prefix
# also avoids the MGOConsole executable colliding with its CMake target
# directory build/MGOConsole/ on single-config (Linux) builds.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# ---- Build optimization ----
# ccache support (automatic if installed)
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    set(CMAKE_C_COMPILER_LAUNCHER   "${CCACHE_PROGRAM}")
    message(STATUS "ccache: enabled")
endif()

# ---- Testing ----
enable_testing()

# ---- Dependencies (all REQUIRED — install via apt/brew/vcpkg) ----
# Where the CMake module supports version checking, minimums are enforced at
# configure time.  Versions without enforcement are documented in README.
find_package(Boost 1.83.0 REQUIRED COMPONENTS regex)
find_package(PROJ REQUIRED)               # version checked at runtime via proj_info()
find_package(Eigen3 3.4.0 REQUIRED)
find_package(TIFF REQUIRED)               # CMake FindTIFF lacks version support
find_package(GDAL REQUIRED)
find_package(JPEG REQUIRED)               # CMake FindJPEG lacks version support
find_package(PNG REQUIRED)                # CMake FindPNG lacks version support
find_package(ZLIB 1.3.0 REQUIRED)

# ---- nlohmann::json (header-only) ----
# Prefer the vcpkg CMake package; fall back to the header installed by
# `vcpkg install nlohmann-json` (manifest mode) so non-toolchain builds work.
find_package(nlohmann_json CONFIG QUIET)
if(TARGET nlohmann_json::nlohmann_json)
    set(NLOHMANN_JSON_TARGET nlohmann_json::nlohmann_json)
else()
    find_path(NLOHMANN_JSON_INCLUDE_DIR nlohmann/json.hpp)
    if(NLOHMANN_JSON_INCLUDE_DIR)
        add_library(nlohmann_json INTERFACE)
        target_include_directories(nlohmann_json INTERFACE ${NLOHMANN_JSON_INCLUDE_DIR})
        set(NLOHMANN_JSON_TARGET nlohmann_json)
    else()
        message(FATAL_ERROR "nlohmann/json.hpp not found. Install nlohmann-json via vcpkg.json or nlohmann-json3-dev (apt/brew).")
    endif()
endif()

# ---- PROJ header shim ----
# PROJ installs its C API header at <prefix>/include/proj.h (flat), but the
# MGO source uses <proj/proj.h> (nested include path).  Create a shim so
# existing source is unchanged and find_package(PROJ) is the single source
# of PROJ headers on every platform.
set(PROJ_SHIM_DIR ${CMAKE_BINARY_DIR}/proj-shim)
file(MAKE_DIRECTORY ${PROJ_SHIM_DIR}/proj)
if(NOT EXISTS ${PROJ_SHIM_DIR}/proj/proj.h)
    find_path(_PROJ_H_DIR proj.h)
    if(_PROJ_H_DIR)
        if(WIN32)
            configure_file("${_PROJ_H_DIR}/proj.h" "${PROJ_SHIM_DIR}/proj/proj.h" COPYONLY)
        else()
            file(CREATE_LINK ${_PROJ_H_DIR}/proj.h ${PROJ_SHIM_DIR}/proj/proj.h SYMBOLIC)
        endif()
        message(STATUS "PROJ shim: ${_PROJ_H_DIR}/proj.h -> ${PROJ_SHIM_DIR}/proj/")
    else()
        message(FATAL_ERROR "proj.h not found (PROJ REQUIRED)")
    endif()
endif()

# ---- PROJ database path ----
# PROJ 9.x requires proj.db at runtime to resolve EPSG authority codes
# (e.g. EPSG:4978).  On Windows, PROJ_LIB is often not set by the
# package manager and the default compiled-in search path may point to
# a build-machine location that doesn't exist on the target machine.
#
# Fix: locate proj.db at configure time from the same PROJ installation
# that provides proj.h, embed its directory as MGO_PROJ_DB_DIR, and use
# proj_context_set_database_path() at runtime to set it explicitly —
# bypassing the PROJ_LIB environment-variable dance entirely.
get_filename_component(_PROJ_PREFIX ${_PROJ_H_DIR} DIRECTORY)
find_file(PROJ_DB_PATH proj.db
    HINTS
        ${_PROJ_PREFIX}/share/proj
        ${_PROJ_PREFIX}/../share/proj
        /usr/share/proj
        /usr/local/share/proj
        /opt/share/proj
        /opt/homebrew/share/proj
    DOC "Absolute path to the PROJ database file (proj.db)"
)
if(PROJ_DB_PATH)
    get_filename_component(PROJ_DB_DIR ${PROJ_DB_PATH} DIRECTORY)
    message(STATUS "PROJ database: ${PROJ_DB_PATH}")
else()
    message(WARNING "proj.db not found — PROJ EPSG code resolution may fail at runtime")
endif()

# ---- assimp: always build from source (unified across platforms) ----
# All platforms (Linux/macOS/Windows) build Assimp v6.0.5 from source.
# This ensures consistent behavior including the aiVector3d typedef
# (aiVector3t<double>) which may be absent from system-installed Assimp.
#
# Source resolution order:
#   1. MGO_ASSIMP_SOURCE_DIR (user override, if set and valid)
#   2. ThirdParty/package/assimp/ (project-local cache, auto-populated)
#   3. Download from GitHub (first build only, then cached to #2)
#
# For fully offline use, manually clone before first build:
#   git clone --depth 1 --branch v6.0.5 https://github.com/assimp/assimp.git ThirdParty/package/assimp
include(FetchContent)

set(MGO_ASSIMP_SOURCE_DIR "" CACHE PATH "Custom assimp source dir (empty = auto)")
set(MGO_ASSIMP_LOCAL_DIR ${CMAKE_SOURCE_DIR}/ThirdParty/package/assimp)

# Determine source location
if(MGO_ASSIMP_SOURCE_DIR AND EXISTS "${MGO_ASSIMP_SOURCE_DIR}/CMakeLists.txt")
    set(_assimp_source_dir ${MGO_ASSIMP_SOURCE_DIR})
    set(_assimp_from_git FALSE)
    message(STATUS "assimp: using user-specified source at ${_assimp_source_dir}")
elseif(EXISTS "${MGO_ASSIMP_LOCAL_DIR}/CMakeLists.txt")
    set(_assimp_source_dir ${MGO_ASSIMP_LOCAL_DIR})
    set(_assimp_from_git FALSE)
    message(STATUS "assimp: using project-local source at ${_assimp_source_dir}")
else()
    set(_assimp_source_dir "")
    set(_assimp_from_git TRUE)
    message(STATUS "assimp: will download v6.0.5 from GitHub (will cache to ThirdParty/assimp-src/)")
endif()

if(_assimp_source_dir)
    FetchContent_Declare(assimp SOURCE_DIR ${_assimp_source_dir})
else()
    FetchContent_Declare(assimp
        GIT_REPOSITORY https://github.com/assimp/assimp.git
        GIT_TAG v6.0.5
        GIT_SHALLOW TRUE
    )
endif()

set(ASSIMP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(ASSIMP_INSTALL OFF CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_ASSIMP_TOOLS OFF CACHE BOOL "" FORCE)
set(ASSIMP_NO_EXPORT OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(assimp)

# Cache downloaded source to project directory for offline reuse
if(_assimp_from_git AND EXISTS "${assimp_SOURCE_DIR}/CMakeLists.txt")
    message(STATUS "assimp: caching source to ${MGO_ASSIMP_LOCAL_DIR}")
    file(COPY "${assimp_SOURCE_DIR}/" DESTINATION "${MGO_ASSIMP_LOCAL_DIR}"
         REGEX ".git" EXCLUDE)
endif()

message(STATUS "assimp: built from source (v6.0.5, unified)")

# Common compile flags
if(MSVC)
    add_compile_options(/W4 /wd4100 /wd4101 /wd4244 /wd4267 /wd4996 /utf-8)
    add_compile_definitions(_USE_MATH_DEFINES _CRT_SECURE_NO_WARNINGS)
else()
    add_compile_options(-Wall -Wno-unused-parameter -Wno-unused-variable)
endif()

# ---- MGO software version (v1.0.0) ----
set(MGO_VERSION_MAJOR 1)
set(MGO_VERSION_MINOR 0)
set(MGO_VERSION_PATCH 0)
set(MGO_VERSION "${MGO_VERSION_MAJOR}.${MGO_VERSION_MINOR}.${MGO_VERSION_PATCH}")
set(MGO_SOVERSION ${MGO_VERSION_MAJOR})
# Single source of truth mirrored in MGOVersion.h. Version + copyright are
# stamped onto every exported shared library:
#   Windows   : MGOVersion.rc is compiled into the DLL (VERSIONINFO incl.
#               Legal Copyright) — VERSION is NOT set here because MSVC would
#               then auto-generate a second, copyright-free version resource.
#   Linux/mac : VERSION/SOVERSION produce the SONAME lib<mod>.so.1.0.0.
function(mgo_set_library_version target)
    if(WIN32)
        target_sources(${target} PRIVATE ${CMAKE_SOURCE_DIR}/MGOVersion.rc)
    else()
        set_target_properties(${target} PROPERTIES
            VERSION ${MGO_VERSION}
            SOVERSION ${MGO_SOVERSION})
    endif()
endfunction()

# ---- meshoptimizer static library (vendored v1.2, pure C, portable) ----
add_library(meshoptimizer STATIC
    MeshGroupOptimizer/meshoptimizer/allocator.cpp
    MeshGroupOptimizer/meshoptimizer/clusterizer.cpp
    MeshGroupOptimizer/meshoptimizer/indexanalyzer.cpp
    MeshGroupOptimizer/meshoptimizer/indexcodec.cpp
    MeshGroupOptimizer/meshoptimizer/indexgenerator.cpp
    MeshGroupOptimizer/meshoptimizer/meshletcodec.cpp
    MeshGroupOptimizer/meshoptimizer/meshletutils.cpp
    MeshGroupOptimizer/meshoptimizer/opacitymap.cpp
    MeshGroupOptimizer/meshoptimizer/overdrawoptimizer.cpp
    MeshGroupOptimizer/meshoptimizer/partition.cpp
    MeshGroupOptimizer/meshoptimizer/quantization.cpp
    MeshGroupOptimizer/meshoptimizer/rasterizer.cpp
    MeshGroupOptimizer/meshoptimizer/simplifier.cpp
    MeshGroupOptimizer/meshoptimizer/spatialorder.cpp
    MeshGroupOptimizer/meshoptimizer/stripifier.cpp
    MeshGroupOptimizer/meshoptimizer/tangentspace.cpp
    MeshGroupOptimizer/meshoptimizer/vcacheoptimizer.cpp
    MeshGroupOptimizer/meshoptimizer/vertexcodec.cpp
    MeshGroupOptimizer/meshoptimizer/vertexfilter.cpp
    MeshGroupOptimizer/meshoptimizer/vfetchoptimizer.cpp
)
target_include_directories(meshoptimizer PUBLIC
    MeshGroupOptimizer
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# =====================================================================
# Module libraries (each in its own CMakeLists.txt)
# =====================================================================
add_subdirectory(MeshGroupOptimizer)
add_subdirectory(MeshProjectionErrorCorrector)
add_subdirectory(TileBuilder)
add_subdirectory(TilesConverter)
add_subdirectory(TerrainConverter)
add_subdirectory(DOMConverter)
add_subdirectory(GeoJSONConverter)

# ---- OSGBConverter (optional, requires OpenSceneGraph) ----
# OSG detection must run in root scope before add_subdirectory
option(MGO_WITH_OSG "Build OSGBConverter (requires OpenSceneGraph)" ON)
set(_osg_thirdparty "${CMAKE_SOURCE_DIR}/ThirdParty/package/osg")
set(_osg_search_hints "${_osg_thirdparty}/lib")

if(MGO_WITH_OSG)
    find_package(OpenSceneGraph 3.6.5 QUIET COMPONENTS osgDB osgUtil)
endif()
if(MGO_WITH_OSG AND OpenSceneGraph_FOUND)
    set(OSG_INCLUDE_DIRS ${OPENSCENEGRAPH_INCLUDE_DIRS})
    set(OSG_LIBRARIES ${OPENSCENEGRAPH_LIBRARIES})
elseif(EXISTS "${_osg_thirdparty}")
    find_library(OSG_LIB osg QUIET PATHS ${_osg_search_hints} NO_DEFAULT_PATH)
    find_library(OSGDB_LIB osgDB QUIET PATHS ${_osg_search_hints} NO_DEFAULT_PATH)
    find_library(OSGUTIL_LIB osgUtil QUIET PATHS ${_osg_search_hints} NO_DEFAULT_PATH)
    find_library(OSG_OPENTHREADS_LIB OpenThreads QUIET PATHS ${_osg_search_hints} NO_DEFAULT_PATH)
    find_path(OSG_INCLUDE_DIRS osg/Node PATHS "${_osg_thirdparty}/include" NO_DEFAULT_PATH)
    set(OSG_LIBRARIES ${OSG_OPENTHREADS_LIB} ${OSG_LIB} ${OSGDB_LIB} ${OSGUTIL_LIB})
    if(OSG_LIBRARIES AND OSG_INCLUDE_DIRS)
        set(OpenSceneGraph_FOUND TRUE)
    endif()
else()
    find_library(OSG_LIB osg QUIET)
    find_library(OSGDB_LIB osgDB QUIET)
    find_library(OSGUTIL_LIB osgUtil QUIET)
    find_path(OSG_INCLUDE_DIRS osg/Node)
    set(OSG_LIBRARIES ${OSG_LIB} ${OSGDB_LIB} ${OSGUTIL_LIB})
endif()

if(OSG_LIBRARIES AND OSG_INCLUDE_DIRS)
    message(STATUS "OSGBConverter: enabled")
    add_subdirectory(OSGBConverter)
    set(OSGB_AVAILABLE TRUE)
elseif(MGO_WITH_OSG)
    message(WARNING "OSGBConverter: OpenSceneGraph not found. Install libopenscenegraph-dev or set MGO_WITH_OSG=OFF")
    set(OSGB_AVAILABLE FALSE)
else()
    message(STATUS "OSGBConverter: disabled (MGO_WITH_OSG=OFF)")
    set(OSGB_AVAILABLE FALSE)
endif()

# TileBuilder regression: external overflow subtree must not 404
add_executable(test_overflow404
    TileBuilder/test/test_overflow404.cpp
)
target_include_directories(test_overflow404 PRIVATE
    TileBuilder
    MeshProjectionErrorCorrector
    MeshGroupOptimizer
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJ_SHIM_DIR}
)
target_link_libraries(test_overflow404 PRIVATE
    TileBuilder
    MeshProjectionErrorCorrector
    MeshGroupOptimizer
    meshoptimizer
    assimp::assimp
    PROJ::proj
    Eigen3::Eigen
)
add_test(NAME test_overflow404 COMMAND test_overflow404)

# TileBuilder regression: empty primitives (no triangles) must not be exported
add_executable(test_emptyprimitive
    TileBuilder/test/test_emptyprimitive.cpp
)
target_include_directories(test_emptyprimitive PRIVATE
    TileBuilder
    MeshProjectionErrorCorrector
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJ_SHIM_DIR}
)
target_link_libraries(test_emptyprimitive PRIVATE
    TileBuilder
    MeshProjectionErrorCorrector
    assimp::assimp
    PROJ::proj
)
add_test(NAME test_emptyprimitive COMMAND test_emptyprimitive)

# GeoJSONConverter unit tests
add_executable(test_geojson
    GeoJSONConverter/test/test_geojson.cpp
    GeoJSONConverter/Json.cpp
    GeoJSONConverter/GeoJSONConverter.cpp
    MeshProjectionErrorCorrector/CRSTransformer.cpp
    MeshProjectionErrorCorrector/PROJUtils.cpp
)
target_include_directories(test_geojson PRIVATE
    GeoJSONConverter
    MeshProjectionErrorCorrector
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJ_SHIM_DIR}
)
target_compile_definitions(test_geojson PRIVATE
    _GEOJSON_CONVERTER_DLL _MESH_PROJECTION_DLL BOOST_ALL_NO_LIB
)
target_link_libraries(test_geojson PRIVATE PROJ::proj assimp::assimp)
add_test(NAME test_geojson COMMAND test_geojson)

add_subdirectory(MGOConsole)

# =====================================================================
# OSGBConverter unit tests
# =====================================================================
add_executable(test_osgb_unit
    OSGBConverter/test/test_unit.cpp
    OSGBConverter/MetadataParser.cpp
    OSGBConverter/PlatformCompat.cpp
    OSGBConverter/VendorHandlerFactory.cpp
    OSGBConverter/ContextCaptureHandler.cpp
    OSGBConverter/DJITerraHandler.cpp
    MeshProjectionErrorCorrector/CProjectionEngine.cpp
    MeshProjectionErrorCorrector/AxisMapper.cpp
    MeshProjectionErrorCorrector/IGeoreferencing.cpp
    MeshProjectionErrorCorrector/GeoreferencingWith7Parameters.cpp
    MeshProjectionErrorCorrector/GeoreferencingWithAnchor.cpp
    MeshProjectionErrorCorrector/GeoreferencingWithMultiPosition.cpp
    MeshProjectionErrorCorrector/GeoreferencingFactory.cpp
    MeshProjectionErrorCorrector/UtilTools.cpp
    MeshProjectionErrorCorrector/CoordinateTransform.cpp
)
target_include_directories(test_osgb_unit PRIVATE
    OSGBConverter
    MeshProjectionErrorCorrector
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJ_SHIM_DIR}
)
target_compile_definitions(test_osgb_unit PRIVATE
    _MESH_PROJECTION_DLL OSGB_CONVERTER_DLL BOOST_ALL_NO_LIB OSGB_UNIT_TEST
)
target_link_libraries(test_osgb_unit PRIVATE assimp::assimp PROJ::proj Eigen3::Eigen)
add_test(NAME test_osgb_unit COMMAND test_osgb_unit)

# ---- OSGB → 3D Tiles end-to-end quality test (optional) ----
# Validates generated tiles for structural + geometric quality. Skips unless
# MGO_OSGB_TEST_INPUT points at a ContextCapture input directory.
find_package(Python3 COMPONENTS Interpreter QUIET)
set(MGO_OSGB_TEST_INPUT "" CACHE PATH "ContextCapture OSGB input dir for the tile-quality integration test")
if(Python3_Interpreter_FOUND)
    add_test(NAME test_osgb_quality
        COMMAND ${Python3_EXECUTABLE}
                ${CMAKE_SOURCE_DIR}/OSGBConverter/test/integration_test.py
                $<TARGET_FILE:MGOConsole>
                ${MGO_OSGB_TEST_INPUT}
                ${CMAKE_BINARY_DIR}/osgb-quality-output)
    set_tests_properties(test_osgb_quality PROPERTIES SKIP_RETURN_CODE 77)
endif()

# IGeoreferencing unit tests
add_executable(test_georef
    OSGBConverter/test/test_georef.cpp
    MeshProjectionErrorCorrector/IGeoreferencing.cpp
    MeshProjectionErrorCorrector/GeoreferencingWith7Parameters.cpp
    MeshProjectionErrorCorrector/GeoreferencingWithAnchor.cpp
    MeshProjectionErrorCorrector/GeoreferencingWithMultiPosition.cpp
    MeshProjectionErrorCorrector/GeoreferencingFactory.cpp
    MeshProjectionErrorCorrector/CProjectionEngine.cpp
    MeshProjectionErrorCorrector/AxisMapper.cpp
    MeshProjectionErrorCorrector/UtilTools.cpp
    MeshProjectionErrorCorrector/CoordinateTransform.cpp
)
target_include_directories(test_georef PRIVATE
    MeshProjectionErrorCorrector
    OSGBConverter
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJ_SHIM_DIR}
)
target_compile_definitions(test_georef PRIVATE
    _MESH_PROJECTION_DLL BOOST_ALL_NO_LIB
)
target_link_libraries(test_georef PRIVATE
    assimp::assimp
    PROJ::proj
    Eigen3::Eigen
)
add_test(NAME test_georef COMMAND test_georef)

# Boundary value tests — covers all modules
add_executable(test_boundary
    OSGBConverter/test/test_boundary.cpp
    MeshProjectionErrorCorrector/IGeoreferencing.cpp
    MeshProjectionErrorCorrector/GeoreferencingWith7Parameters.cpp
    MeshProjectionErrorCorrector/GeoreferencingWithAnchor.cpp
    MeshProjectionErrorCorrector/GeoreferencingWithMultiPosition.cpp
    MeshProjectionErrorCorrector/GeoreferencingFactory.cpp
    MeshProjectionErrorCorrector/CProjectionEngine.cpp
    MeshProjectionErrorCorrector/AxisMapper.cpp
    MeshProjectionErrorCorrector/UtilTools.cpp
    MeshProjectionErrorCorrector/CoordinateTransform.cpp
)
target_include_directories(test_boundary PRIVATE
    MeshProjectionErrorCorrector
    MeshGroupOptimizer
    OSGBConverter
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJ_SHIM_DIR}
)
target_compile_definitions(test_boundary PRIVATE
    _MESH_PROJECTION_DLL MESH_GROUP_OPTIMIZER_DLL BOOST_ALL_NO_LIB
)
target_link_libraries(test_boundary PRIVATE
    MeshGroupOptimizer
    assimp::assimp
    PROJ::proj
    Eigen3::Eigen
    Boost::regex
)
add_test(NAME test_boundary COMMAND test_boundary)

# Error handling regression tests
add_executable(test_error
    OSGBConverter/test/test_error.cpp
    MeshProjectionErrorCorrector/IGeoreferencing.cpp
    MeshProjectionErrorCorrector/GeoreferencingWith7Parameters.cpp
    MeshProjectionErrorCorrector/GeoreferencingWithAnchor.cpp
    MeshProjectionErrorCorrector/GeoreferencingWithMultiPosition.cpp
    MeshProjectionErrorCorrector/GeoreferencingFactory.cpp
    MeshProjectionErrorCorrector/CProjectionEngine.cpp
    MeshProjectionErrorCorrector/AxisMapper.cpp
    MeshProjectionErrorCorrector/UtilTools.cpp
    MeshProjectionErrorCorrector/CoordinateTransform.cpp
)
target_include_directories(test_error PRIVATE
    MeshProjectionErrorCorrector
    MeshGroupOptimizer
    OSGBConverter
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJ_SHIM_DIR}
)
target_compile_definitions(test_error PRIVATE
    _MESH_PROJECTION_DLL MESH_GROUP_OPTIMIZER_DLL BOOST_ALL_NO_LIB
)
target_link_libraries(test_error PRIVATE
    MeshGroupOptimizer
    assimp::assimp
    PROJ::proj
    Eigen3::Eigen
    Boost::regex
)
add_test(NAME test_error COMMAND test_error)

# Coordinate frame verification tests
add_executable(test_coordframe
    OSGBConverter/test/test_coordframe.cpp
    MeshProjectionErrorCorrector/CoordinateTransform.cpp
    MeshProjectionErrorCorrector/AxisMapper.cpp
    MeshProjectionErrorCorrector/UtilTools.cpp
)
target_compile_definitions(test_coordframe PRIVATE
    _MESH_PROJECTION_DLL BOOST_ALL_NO_LIB
)
target_include_directories(test_coordframe PRIVATE
    MeshProjectionErrorCorrector
    MeshGroupOptimizer
    OSGBConverter
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJ_SHIM_DIR}
)
target_link_libraries(test_coordframe PRIVATE
    assimp::assimp
    Eigen3::Eigen
    PROJ::proj
)
add_test(NAME test_coordframe COMMAND test_coordframe)
