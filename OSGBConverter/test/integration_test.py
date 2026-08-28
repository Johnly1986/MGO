#!/usr/bin/env python3
"""
integration_test.py — end-to-end OSGB → 3D Tiles quality regression test.

Runs `mgo osgb` on an input ContextCapture directory, then validates the
generated tiles with verify_tile_quality.py.

Exit codes:
  0  = PASS
  1  = FAIL (conversion or validation error)
  77 = SKIP (input directory not provided / not found)

Usage:
  python integration_test.py <mgo_exe> <input_dir> <output_dir>
"""

import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
VALIDATOR = os.path.join(HERE, 'verify_tile_quality.py')


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 2

    mgo_exe = sys.argv[1]
    # CMake collapses an empty MGO_OSGB_TEST_INPUT cache variable, so the input
    # argument may be absent. Treat a missing/empty input as "skip".
    input_dir = sys.argv[2] if len(sys.argv) >= 4 else ""
    output_dir = sys.argv[3] if len(sys.argv) >= 4 else sys.argv[2]

    # Skip when no test data is configured.
    if not input_dir or not os.path.isdir(input_dir):
        print(f'SKIP: input directory not found: {input_dir!r}')
        return 77

    if not os.path.exists(mgo_exe):
        print(f'FAIL: mgo executable not found: {mgo_exe}')
        return 1

    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)

    print(f'[integration] running: {mgo_exe} osgb -i {input_dir} -o {output_dir}')
    conv = subprocess.run(
        [mgo_exe, 'osgb', '-i', input_dir, '-o', output_dir],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    if conv.returncode != 0:
        print('FAIL: converter returned', conv.returncode)
        print(conv.stdout[-4000:])
        return 1

    print('[integration] validating generated tiles')
    val = subprocess.run(
        [sys.executable, VALIDATOR, output_dir],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    print(val.stdout)
    if val.returncode != 0:
        print('FAIL: quality validation found errors')
        return 1

    print('PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
