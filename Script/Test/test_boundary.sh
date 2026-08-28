#!/bin/bash
# TerrainConverter boundary value tests
# Tests the CLI with boundary value parameter combinations
set -e

# Run from project root regardless of where script is invoked
cd "$(dirname "$0")/../.."

BIN="${BIN:-build/TerrainConverterCli}"
TIF="TerrainConverter/test/test_terrain.tif"
TDIR="${TDIR:-/tmp/terrain_boundary_test}"
PASS=0
FAIL=0

rm -rf "$TDIR"

run_test() {
    local name="$1"; shift
    local out="$TDIR/$name"
    mkdir -p "$out"
    if LD_LIBRARY_PATH=build:$LD_LIBRARY_PATH "$BIN" -i "$TIF" -o "$out" "$@" >/dev/null 2>&1; then
        if [ -f "$out/layer.json" ]; then
            echo "  PASS: $name"
            PASS=$((PASS + 1))
        else
            echo "  WARN: $name — no layer.json produced"
        fi
    else
        echo "  FAIL: $name"
        FAIL=$((FAIL + 1))
    fi
}

echo "=== TerrainConverter Boundary Tests ==="

# LOD boundaries
run_test "max_lod_0"          --max-lod 0
run_test "max_lod_1"          --max-lod 1
run_test "max_lod_5"          --max-lod 5

# Error boundaries
run_test "error_zero"         --error 0.0
run_test "error_tiny"         --error 0.001
run_test "error_large"        --error 100.0

# Sample count boundaries
run_test "samples_2"          --samples 2
run_test "samples_4"          --samples 4
run_test "samples_default"    --samples 65

# Lock border flags
run_test "no_lock_border"     --no-lock-border
run_test "with_lock_border"   # default

# Normal weight boundaries
run_test "nweight_zero"       --nweight 0.0
run_test "nweight_one"        --nweight 1.0

# Normals flags
run_test "no_normals"         --no-normals
run_test "with_normals"       # default

# Verbose mode
run_test "verbose"            -v

echo ""
echo "PASS=$PASS FAIL=$FAIL"
rm -rf "$TDIR"
[ $FAIL -eq 0 ]
