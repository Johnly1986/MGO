#pragma once

#if defined(_MSC_VER)
# ifdef _MESH_PROJECTION_DLL
#	define MESH_PROJECTION_API __declspec(dllexport)

#	ifdef _DEBUG
#		pragma comment(lib, "assimp-vc143-mtd.lib")
#		pragma message("auto linking to assimp-vc143-mtd.lib")
#	else
#		pragma comment(lib, "assimp-vc143-mt.lib")
#		pragma message("auto linking to assimp-vc143-mt.lib")
#	endif
#	pragma comment(lib, "proj.lib")
#	pragma message("auto linking to proj.lib")
# else
#	define MESH_PROJECTION_API __declspec(dllimport)
#	ifdef _DEBUG
#		pragma comment(lib, "MeshProjectionErrorCorrector_d.lib")
#		pragma message("auto linking to MeshProjectionErrorCorrector_d.lib")
#	else
#		pragma comment(lib, "MeshProjectionErrorCorrector.lib")
#		pragma message("auto linking to MeshProjectionErrorCorrector.lib")
#	endif
# endif
#else
/* Linux / GCC / Clang */
# ifdef _MESH_PROJECTION_DLL
#	define MESH_PROJECTION_API __attribute__((visibility("default")))
# else
#	define MESH_PROJECTION_API
# endif
#endif
