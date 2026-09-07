# TilesConverter — Assimp scene → 3D Tiles (b3dm + tileset.json)
add_library(TilesConverter SHARED
    TilesConverter.cpp
    BimFormatStrategy.cpp
    BimBindingPipeline.cpp
)
mgo_set_library_version(TilesConverter)
target_include_directories(TilesConverter PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/TileBuilder
    ${CMAKE_SOURCE_DIR}/MeshGroupOptimizer
    ${CMAKE_SOURCE_DIR}/MeshProjectionErrorCorrector
    ${CMAKE_SOURCE_DIR}
    ${PROJ_SHIM_DIR}
)
target_compile_definitions(TilesConverter PRIVATE _TILES_CONVERTER_DLL BOOST_ALL_NO_LIB)
target_compile_options(TilesConverter PRIVATE $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-fvisibility=hidden>)

# iconv (POSIX): FixTextEncoding's GB18030 -> UTF-8 tier for legacy GBK
# node names baked into Windows-exported FBX/OBJ assets. glibc keeps iconv
# in libc (no extra lib); macOS/vcpkg builds may provide an explicit libiconv.
if(NOT WIN32)
    include(CheckIncludeFileCXX)
    check_include_file_cxx(iconv.h MGO_HAVE_ICONV_H)
    if(MGO_HAVE_ICONV_H)
        target_compile_definitions(TilesConverter PRIVATE MGO_HAVE_ICONV)
        find_library(MGO_ICONV_LIBRARY NAMES iconv libiconv)
        if(MGO_ICONV_LIBRARY)
            target_link_libraries(TilesConverter PRIVATE ${MGO_ICONV_LIBRARY})
        endif()
    endif()
endif()
target_link_libraries(TilesConverter PUBLIC
    TileBuilder
    MeshProjectionErrorCorrector
    MeshGroupOptimizer
    assimp::assimp
)
