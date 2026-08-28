# TilesConverter — Assimp scene → 3D Tiles (b3dm + tileset.json)
add_library(TilesConverter SHARED
    TilesConverter.cpp
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
target_link_libraries(TilesConverter PUBLIC
    TileBuilder
    MeshProjectionErrorCorrector
    MeshGroupOptimizer
    assimp::assimp
)
