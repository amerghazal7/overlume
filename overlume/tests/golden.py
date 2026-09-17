#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

"""golden.py — Epic 1 Task 2 Step 11. Dev-only helper for inspecting a
rendered PNG before promoting it to a committed golden under tests/goldens/
(this project's stated working style: "user judges by visuals" — see
docs/plans/2026-08-18-visual-mode-epic1.md Task 2 Step 11).

Not on the ctest path -- run by hand:
    python3 tests/golden.py --show /tmp/empty_world_dark_adas_actual.png
    python3 tests/golden.py --promote /tmp/empty_world_dark_adas_actual.png \
        tests/goldens/empty_world_dark_adas.png
"""
import argparse
import shutil
import sys


def show(path: str) -> None:
    try:
        from PIL import Image  # ponytail: optional dep, only needed for --show
    except ImportError:
        print(f"(Pillow not installed; can't display {path} -- "
              f"pip install pillow, or just open the file yourself)")
        return
    Image.open(path).show()


def promote(src: str, dest: str) -> None:
    shutil.copyfile(src, dest)
    print(f"copied {src} -> {dest}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--show", metavar="PNG", help="open PNG for a visual sanity check")
    group.add_argument("--promote", nargs=2, metavar=("SRC_PNG", "DEST_PNG"),
                        help="copy SRC_PNG to DEST_PNG (e.g. into tests/goldens/) "
                             "after a human has looked at it")
    args = parser.parse_args()

    if args.show:
        show(args.show)
    else:
        promote(*args.promote)
    return 0


if __name__ == "__main__":
    sys.exit(main())
