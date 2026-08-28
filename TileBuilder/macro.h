#pragma once

#if defined(_MSC_VER)
# ifdef _TILE_BUILDER_DLL
#	define TILE_BUILDER_API __declspec(dllexport)
# else
#	define TILE_BUILDER_API __declspec(dllimport)
#	ifdef _DEBUG
#		pragma comment(lib, "TileBuilder_d.lib")
#	else
#		pragma comment(lib, "TileBuilder.lib")
#	endif
# endif
#else
/* Linux / GCC / Clang */
# ifdef _TILE_BUILDER_DLL
#	define TILE_BUILDER_API __attribute__((visibility("default")))
# else
#	define TILE_BUILDER_API
# endif
#endif
