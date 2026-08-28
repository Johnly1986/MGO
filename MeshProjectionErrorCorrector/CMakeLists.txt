# MeshProjectionErrorCorrector — PROJ transforms, georeferencing, coordinate systems
add_library(MeshProjectionErrorCorrector SHARED
    GeoreferencingWith7Parameters.cpp
    GeoreferencingWithAnchor.cpp
    GeoreferencingWithMultiPosition.cpp
    GeoreferencingFactory.cpp
    IGeoreferencing.cpp
    Octree.cpp
    CoordinateTransform.cpp
    UtilTools.cpp
    AxisMapper.cpp
    CProjectionEngine.cpp
    CRSTransformer.cpp
    MeshProjectionErrorCorrector.cpp
    PROJUtils.cpp
)
mgo_set_library_version(MeshProjectionErrorCorrector)
target_include_directories(MeshProjectionErrorCorrector PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}
    ${PROJ_SHIM_DIR}
)
target_compile_definitions(MeshProjectionErrorCorrector PRIVATE
    _MESH_PROJECTION_DLL BOOST_ALL_NO_LIB
)
if(PROJ_DB_DIR)
    target_compile_definitions(MeshProjectionErrorCorrector PUBLIC
        MGO_PROJ_DB_DIR="${PROJ_DB_DIR}"
    )
endif()
if(NOT MSVC)
    target_compile_options(MeshProjectionErrorCorrector PRIVATE -fvisibility=hidden)
endif()
target_link_libraries(MeshProjectionErrorCorrector
    PUBLIC assimp::assimp PROJ::proj Eigen3::Eigen
    PRIVATE Boost::regex
)
