#pragma once

#if defined(_MSC_VER)
# ifdef _IMAGE_TILER_DLL
#   define IMAGE_TILER_API __declspec(dllexport)
# else
#   define IMAGE_TILER_API __declspec(dllimport)
# endif
#else
# ifdef _IMAGE_TILER_DLL
#   define IMAGE_TILER_API __attribute__((visibility("default")))
# else
#   define IMAGE_TILER_API
# endif
#endif