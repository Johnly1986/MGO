# Third-Party Notices

This file acknowledges the third-party software distributed with or used by MGO.
MGO itself is licensed under the Apache License 2.0 (see [LICENSE](LICENSE)).

## Vendored in this repository

| Component | Location | License | Notes |
|-----------|----------|---------|-------|
| [meshoptimizer](https://github.com/zeux/meshoptimizer) v1.2 | `MeshGroupOptimizer/meshoptimizer/` | MIT | Copyright (c) 2016-2024 Stanislav Voronyi / Arseny Kapoulkine. Includes local modifications (sparse simplification, regularization, vertex protection). MIT license text is retained in the source files. |
| OpenSceneGraph headers | `ThirdParty/package/osg/` | OSGPL (LGPL 2.1 with exceptions) | Copyright (c) Don Burns and Robert Osfield. Distributed for build convenience; users may instead link against a system/vcpkg OpenSceneGraph. See https://github.com/openscenegraph/OpenSceneGraph/blob/master/LICENSE.txt |

## Dependencies managed via vcpkg / system packages

These libraries are **not** distributed in this repository; they are resolved at build time via [vcpkg](vcpkg.json) or the CI system packages.

| Library | License |
|---------|---------|
| [Assimp](https://github.com/assimp/assimp) (v6.0.5, built from source via CMake FetchContent) | BSD 3-Clause |
| [Boost](https://www.boost.org) | Boost Software License 1.0 |
| [PROJ](https://proj.org) | X/MIT style |
| [Eigen3](https://eigen.tuxfamily.org) | MPL 2.0 |
| [GDAL](https://gdal.org) | MIT |
| [libtiff](https://libtiff.org) | TIFF/HPND style |
| [libjpeg-turbo](https://libjpeg-turbo.org) | IJG / BSD 3-Clause / zlib |
| [libpng](http://www.libpng.org) | libpng license (PNG Reference Library License) |
| [zlib](https://zlib.net) | zlib license |
| [nlohmann-json](https://json.nlohmann.me) | MIT |
| [OpenSceneGraph](https://openscenegraph.com) (optional, `MGO_WITH_OSG=ON`) | OSGPL |

Product names and trademarks are the property of their respective owners and are used here for identification purposes only.
