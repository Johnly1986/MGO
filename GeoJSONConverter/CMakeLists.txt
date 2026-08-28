# GeoJSONConverter - GeoJSON projection conversion
add_library(GeoJSONConverter SHARED
    Json.cpp
    GeoJSONConverter.cpp
    ${CMAKE_SOURCE_DIR}/MeshProjectionErrorCorrector/PROJUtils.cpp
)
mgo_set_library_version(GeoJSONConverter)
target_include_directories(GeoJSONConverter PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/MeshProjectionErrorCorrector
    ${CMAKE_SOURCE_DIR}
    ${PROJ_SHIM_DIR}
)
target_compile_definitions(GeoJSONConverter PRIVATE _GEOJSON_CONVERTER_DLL BOOST_ALL_NO_LIB)
if(NOT MSVC)
    target_compile_options(GeoJSONConverter PRIVATE -fvisibility=hidden)
endif()
target_link_libraries(GeoJSONConverter PUBLIC
    MeshProjectionErrorCorrector
    PROJ::proj
)
