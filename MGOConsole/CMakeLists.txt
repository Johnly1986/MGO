# MGOConsole — Unified CLI for all MGO subcommands
add_executable(MGOConsole
    MGOConsole.cpp
    CSVReader.cpp
    ${CMAKE_SOURCE_DIR}/MeshProjectionErrorCorrector/PROJUtils.cpp
)
target_include_directories(MGOConsole PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/TilesConverter
    ${CMAKE_SOURCE_DIR}/TerrainConverter
    ${CMAKE_SOURCE_DIR}/DOMConverter
    ${CMAKE_SOURCE_DIR}/MeshGroupOptimizer
    ${CMAKE_SOURCE_DIR}/MeshProjectionErrorCorrector
    ${CMAKE_SOURCE_DIR}
    ${PROJ_SHIM_DIR}
)
target_compile_definitions(MGOConsole PRIVATE BOOST_ALL_NO_LIB
    $<$<BOOL:${OSGB_AVAILABLE}>:HAS_OSGB_CONVERTER>
)
target_link_libraries(MGOConsole PRIVATE
    MeshGroupOptimizer
    MeshProjectionErrorCorrector
    TilesConverter
    TerrainConverter
    ImageTiler
    GeoJSONConverter
    Boost::regex
    PROJ::proj
)
if(OSGB_AVAILABLE)
    target_include_directories(MGOConsole PRIVATE ${CMAKE_SOURCE_DIR}/OSGBConverter ${OSG_INCLUDE_DIRS})
    target_link_libraries(MGOConsole PRIVATE OSGBConverter)
endif()
