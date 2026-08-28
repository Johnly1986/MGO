#pragma once

#if defined(_MSC_VER)
# ifdef _GEOJSON_CONVERTER_DLL
#   define GEOJSON_CONVERTER_API __declspec(dllexport)
# else
#   define GEOJSON_CONVERTER_API __declspec(dllimport)
# endif
#else
# ifdef _GEOJSON_CONVERTER_DLL
#   define GEOJSON_CONVERTER_API __attribute__((visibility("default")))
# else
#   define GEOJSON_CONVERTER_API
# endif
#endif
