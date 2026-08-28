#pragma once

#if defined(_MSC_VER)
# ifdef _TERRAIN_CONVERTER_DLL
#   define TERRAIN_CONVERTER_API __declspec(dllexport)
# else
#   define TERRAIN_CONVERTER_API __declspec(dllimport)
# endif
#else
# ifdef _TERRAIN_CONVERTER_DLL
#   define TERRAIN_CONVERTER_API __attribute__((visibility("default")))
# else
#   define TERRAIN_CONVERTER_API
# endif
#endif