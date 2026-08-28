# TerrainConverter — GeoTIFF → Cesium Quantized-Mesh terrain tiles
add_library(TerrainConverter SHARED
    HeightmapGrid.cpp
    GeoTiffReader.cpp
    TinSimplifier.cpp
    QuantizedMeshEncoder.cpp
    TerrainQuadtree.cpp
    TerrainLayerJson.cpp
    TerrainConverter.cpp
)
mgo_set_library_version(TerrainConverter)
target_include_directories(TerrainConverter PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/MeshGroupOptimizer
    ${CMAKE_SOURCE_DIR}/MeshProjectionErrorCorrector
    ${CMAKE_SOURCE_DIR}
    ${PROJ_SHIM_DIR}
)
target_compile_definitions(TerrainConverter PRIVATE _TERRAIN_CONVERTER_DLL BOOST_ALL_NO_LIB)
target_compile_options(TerrainConverter PRIVATE $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-fvisibility=hidden>)
target_link_libraries(TerrainConverter PUBLIC
    MeshProjectionErrorCorrector
    MeshGroupOptimizer
    meshoptimizer
    assimp::assimp
    GDAL::GDAL
    TIFF::TIFF
    ${NLOHMANN_JSON_TARGET}
)
