# OSGBConverter — OSGB oblique photography → 3D Tiles (optional, requires OpenSceneGraph)

if(OSG_LIBRARIES AND OSG_INCLUDE_DIRS)
    add_library(OSGBConverter SHARED
        OSGBConverter.cpp
        OSGBReader.cpp
        MetadataParser.cpp
        TextureAtlas.cpp
        TextureEncoder.cpp
        ContextCaptureHandler.cpp
        DJITerraHandler.cpp
        VendorHandlerFactory.cpp
        PlatformCompat.cpp
    )
    mgo_set_library_version(OSGBConverter)
    target_include_directories(OSGBConverter PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/TileBuilder
        ${CMAKE_SOURCE_DIR}/MeshGroupOptimizer
        ${CMAKE_SOURCE_DIR}/MeshProjectionErrorCorrector
        ${CMAKE_SOURCE_DIR}
        ${PROJ_SHIM_DIR}
        ${OSG_INCLUDE_DIRS}
    )
    target_compile_definitions(OSGBConverter PRIVATE OSGB_CONVERTER_DLL BOOST_ALL_NO_LIB)
    target_compile_options(OSGBConverter PRIVATE $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-fvisibility=hidden>)
    target_link_libraries(OSGBConverter PUBLIC
        TileBuilder
        MeshProjectionErrorCorrector
        MeshGroupOptimizer
        meshoptimizer
        assimp::assimp
        ${OSG_LIBRARIES}
        JPEG::JPEG
        PNG::PNG
    )
    set(OSGB_AVAILABLE TRUE)
else()
    set(OSGB_AVAILABLE FALSE)
endif()
