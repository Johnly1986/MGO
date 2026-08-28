# TileBuilder — 3D Tiles binary format builder (GlbBuilder, B3dmBuilder, TilesetWriter)
add_library(TileBuilder SHARED
    TileBuilder.cpp
)
mgo_set_library_version(TileBuilder)
target_include_directories(TileBuilder PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/MeshProjectionErrorCorrector
    ${CMAKE_SOURCE_DIR}
)
target_compile_definitions(TileBuilder PRIVATE
    _TILE_BUILDER_DLL BOOST_ALL_NO_LIB
)
if(NOT MSVC)
    target_compile_options(TileBuilder PRIVATE -fvisibility=hidden)
endif()
target_link_libraries(TileBuilder PUBLIC assimp::assimp MeshProjectionErrorCorrector JPEG::JPEG ${NLOHMANN_JSON_TARGET})
