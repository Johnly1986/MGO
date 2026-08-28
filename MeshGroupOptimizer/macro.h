#pragma once

#if defined(_MSC_VER)
# ifdef _MESH_GROUP_OPTIMIZER_DLL
#	define MESH_GROUP_OPTIMIZER_API __declspec(dllexport)
#	ifdef _DEBUG
#		pragma comment(lib, "assimp-vc143-mtd.lib")
#		pragma message("auto linking to assimp-vc143-mtd.lib")
#	else
#		pragma comment(lib, "assimp-vc143-mt.lib")
#		pragma message("auto linking to assimp-vc143-mt.lib")
#	endif
# else
#	define MESH_GROUP_OPTIMIZER_API __declspec(dllimport)
#	ifdef _DEBUG
#		pragma comment(lib, "MeshGroupOptimizer_d.lib")
#		pragma message("auto linking to MeshGroupOptimizer_d.lib")
#	else
#		pragma comment(lib, "MeshGroupOptimizer.lib")
#		pragma message("auto linking to MeshGroupOptimizer.lib")
#	endif
# endif
#else
/* Linux / GCC / Clang */
# ifdef _MESH_GROUP_OPTIMIZER_DLL
#	define MESH_GROUP_OPTIMIZER_API __attribute__((visibility("default")))
# else
#	define MESH_GROUP_OPTIMIZER_API
# endif
#endif
