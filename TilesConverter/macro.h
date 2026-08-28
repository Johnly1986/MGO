#pragma once

#if defined(_MSC_VER)
# ifdef _TILES_CONVERTER_DLL
#   define TILES_CONVERTER_API __declspec(dllexport)
# else
#   define TILES_CONVERTER_API __declspec(dllimport)
# endif
#else
# ifdef _TILES_CONVERTER_DLL
#   define TILES_CONVERTER_API __attribute__((visibility("default")))
# else
#   define TILES_CONVERTER_API
# endif
#endif