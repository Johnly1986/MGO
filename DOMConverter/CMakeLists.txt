# ImageTiler — DOM orthophoto → TMS image tiles
add_library(ImageTiler SHARED
    ImageReader.cpp
    PngWriter.cpp
    ImageTiler.cpp
    ${CMAKE_SOURCE_DIR}/MeshProjectionErrorCorrector/PROJUtils.cpp
)
mgo_set_library_version(ImageTiler)
target_include_directories(ImageTiler PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/MeshProjectionErrorCorrector
    ${CMAKE_SOURCE_DIR}
    ${PROJ_SHIM_DIR}
)
target_compile_definitions(ImageTiler PRIVATE _IMAGE_TILER_DLL BOOST_ALL_NO_LIB)
target_compile_options(ImageTiler PRIVATE $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-fvisibility=hidden>)
target_link_libraries(ImageTiler PUBLIC
    TIFF::TIFF
    PNG::PNG
    PROJ::proj
)
