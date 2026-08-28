#!/usr/bin/env python3
"""
TerrainConverter boundary value tests.

Tests edge cases by running TerrainConverterCli with boundary value
parameter combinations on the existing test_terrain.tif.
"""

import os, sys, subprocess, tempfile, shutil, json

BINARY = os.path.join(os.path.dirname(__file__), "..", "..", "build", "TerrainConverterCli")
TIF = os.path.join(os.path.dirname(__file__), "..", "..", "TerrainConverter", "test", "test_terrain.tif")
OUT_DIR = tempfile.mkdtemp(prefix="terrain_boundary_test_")
g_pass = 0
g_fail = 0

def check(name, cond):
    global g_pass, g_fail
    if cond:
        g_pass += 1
        print(f"  PASS: {name}")
    else:
        g_fail += 1
        print(f"  FAIL: {name}")

def run_test(name, **kwargs):
    """Run TerrainConverterCli with boundary parameters, verify output."""
    out = os.path.join(OUT_DIR, name)
    os.makedirs(out, exist_ok=True)
    args = [BINARY, "-i", TIF, "-o", out]
    for k, v in kwargs.items():
        args.append(f"--{k.replace('_','-')}")
        args.append(str(v))
    try:
        result = subprocess.run(args, capture_output=True, text=True, timeout=60)
        ok = result.returncode == 0
        has_output = os.path.isfile(os.path.join(out, "layer.json"))
        if ok and has_output:
            # Verify layer.json is valid
            with open(os.path.join(out, "layer.json")) as f:
                lj = json.load(f)
            tiles = sum(len(level) for level in lj.get("available", []))
            check(name, tiles > 0)
        else:
            check(name, False)
    except Exception as e:
        check(name, False)
        print(f"    exception: {e}")

def run_flag_test(name, **kwargs):
    """Run with boolean flags only (no value)."""
    out = os.path.join(OUT_DIR, name)
    os.makedirs(out, exist_ok=True)
    args = [BINARY, "-i", TIF, "-o", out]
    for k, v in kwargs.items():
        if v is True:
            args.append(f"--{k.replace('_','-')}")
        else:
            args.append(f"--{k.replace('_','-')}")
            args.append(str(v))
    try:
        result = subprocess.run(args, capture_output=True, text=True, timeout=60)
        ok = result.returncode == 0
        has_output = os.path.isfile(os.path.join(out, "layer.json"))
        if ok and has_output:
            with open(os.path.join(out, "layer.json")) as f:
                json.load(f)
            check(name, True)
        else:
            check(name, False)
    except Exception as e:
        check(name, False)
        print(f"    exception: {e}")

def main():
    if not os.path.isfile(BINARY):
        print(f"SKIP: {BINARY} not built")
        return 1

    print("=== TerrainConverter Boundary Tests ===\n")

    # LOD boundaries
    run_test("max_lod_boundary_0", max_lod=0)
    run_test("max_lod_boundary_1", max_lod=1)
    run_test("max_lod_boundary_5", max_lod=5)

    # Error boundaries
    run_test("error_boundary_zero", error=0.0)
    run_test("error_boundary_tiny", error=0.0001)
    run_test("error_boundary_large", error=100.0)

    # Sample count boundaries
    run_test("samples_boundary_2", samples=2)
    run_test("samples_boundary_4", samples=4)
    run_test("samples_boundary_65", samples=65)

    # Normal weight boundaries
    run_test("nweight_boundary_zero", nweight=0.0)
    run_test("nweight_boundary_one", nweight=1.0)

    # Flags
    run_flag_test("nolock_flag", no_lock_border=True)
    run_flag_test("nonormals_flag", no_normals=True)
    # Verbose mode: use -v (not --verbose)
    out = os.path.join(OUT_DIR, "verbose_flag")
    os.makedirs(out, exist_ok=True)
    args = [BINARY, "-i", TIF, "-o", out, "-v"]
    result = subprocess.run(args, capture_output=True, text=True, timeout=60)
    ok = result.returncode == 0 and os.path.isfile(os.path.join(out, "layer.json"))
    if ok:
        with open(os.path.join(out, "layer.json")) as f:
            json.load(f)
    check("verbose_flag", ok)

    print(f"\n========================================")
    print(f"PASS={g_pass} FAIL={g_fail}")

    shutil.rmtree(OUT_DIR, ignore_errors=True)
    return 0 if g_fail == 0 else 1

if __name__ == "__main__":
    sys.exit(main())
