// Copyright Johnlyon
//
// OSGBConverter — OSGB oblique photography model → 3D Tiles converter
//

#pragma once

#if defined(_MSC_VER)
# ifdef OSGB_CONVERTER_DLL
#   define OSGB_CONVERTER_API __declspec(dllexport)
# else
#   define OSGB_CONVERTER_API __declspec(dllimport)
# endif
#else
# ifdef OSGB_CONVERTER_DLL
#   define OSGB_CONVERTER_API __attribute__((visibility("default")))
# else
#   define OSGB_CONVERTER_API
# endif
#endif