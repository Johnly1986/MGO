# MeshGroupOptimizer — Mesh simplification engine (vendored meshoptimizer v1.2)
add_library(MeshGroupOptimizer SHARED
    MeshGroupOptimizer.cpp
)
mgo_set_library_version(MeshGroupOptimizer)
target_include_directories(MeshGroupOptimizer PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}
)
target_compile_definitions(MeshGroupOptimizer PRIVATE
    _MESH_GROUP_OPTIMIZER_DLL BOOST_ALL_NO_LIB
)
if(NOT MSVC)
    target_compile_options(MeshGroupOptimizer PRIVATE -fvisibility=hidden)
endif()
target_link_libraries(MeshGroupOptimizer
    PUBLIC assimp::assimp meshoptimizer
    PRIVATE Boost::regex
)
