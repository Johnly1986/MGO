# Makefile for MGO — thin CMake wrapper, works on Linux / macOS / Windows
#
# Prerequisites:
#   CMake >= 3.18, Ninja or Make
#   Linux:   apt install cmake ninja-build libboost-regex-dev libboost-locale-dev libproj-dev libeigen3-dev libtiff-dev libjpeg-dev libpng-dev zlib1g-dev
#   macOS:   brew install cmake ninja boost proj eigen libtiff libjpeg libpng zlib
#   Windows: vcpkg install boost-regex boost-locale proj eigen3 tiff libjpeg-turbo libpng zlib
#
# Usage:
#   make release        # Build Release
#   make debug          # Build Debug
#   make test           # Build Release + run tests
#   make clean          # Remove build directory
#
# Note (Windows): MSVC is a multi-config generator.  cmake --build outputs
#   to build/Release/ (or build/Debug/).  The Makefile targets handle this
#   automatically via --config flags.

BUILD_DIR := build
CONFIG    := Release

.PHONY: all release debug test clean configure

all: release

configure:
	@if [ ! -f "$(BUILD_DIR)/CMakeCache.txt" ]; then \
		cmake -B $(BUILD_DIR) -S . -DCMAKE_BUILD_TYPE=$(CONFIG); \
	fi

release:
	$(MAKE) configure CONFIG=Release
	cmake --build $(BUILD_DIR) --config Release

debug:
	$(MAKE) configure CONFIG=Debug
	cmake --build $(BUILD_DIR) --config Debug

test: release
	cd $(BUILD_DIR) && ctest --output-on-failure -C $(CONFIG)

clean:
	rm -rf $(BUILD_DIR)
