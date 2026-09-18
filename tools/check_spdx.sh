#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
# SPDX header gate for Overlume's first-party source (Task 6b).
#
# File set (via `git ls-files`, so nothing untracked/generated/build ever
# enters it): C/C++ (.c .cc .cpp .cxx .h .hpp .hxx) and Filament .mat files
# under overlume/{src,include,tests,tools,smoke}, examples/, and
# ros/src/overlume_ros/**; Python (.py) and shell (.sh) under those same
# roots plus top-level tools/*.py, tools/*.sh, and ros/colcon_build.sh
# (the colcon workspace root's own build script — a sibling of
# ros/src/overlume_ros, not a descendant of it, so it needs its own entry);
# CMake helper files (overlume/cmake/**/*.cmake) and examples/CMakeLists.txt;
# and the "themes / profiles / params" YAML config under
# ros/src/overlume_ros/config/*.yaml.
#
# Deliberately excluded (never in the set above, stated for the record):
# - Test/golden fixtures (any path segment "fixtures/", e.g.
#   overlume/tests/fixtures/**, ros/src/overlume_ros/test/fixtures/**,
#   overlume/tests/goldens/**) and binary/data assets (.png .glb .b3dm
#   .rviz .csv .ttf .json) — data, not source.
# - PROVENANCE/ATTRIBUTION files and all .md docs.
# - Generated files, and the downloaded stb headers (not tracked in-tree).
# - Small non-code config carrying no copyrightable logic: package.xml,
#   Doxyfile.in, ros/config_colcon.yaml, .github/**/*.yml, *.srv.
# Covered (in addition to the roots above): overlume/assets/materials/*.mat,
# overlume/assets/themes/*.yaml, overlume/CMakeLists.txt,
# ros/src/overlume_ros/CMakeLists.txt, examples/CMakeLists.txt.
#
# Untracked-but-not-ignored files are included too (--others), so a new file
# is checked before it is ever staged.
# Usage: tools/check_spdx.sh
# Exits non-zero (and lists every offender) if any file in the set above is
# missing "SPDX-License-Identifier" from its first 3 lines.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

mapfile -t CPP_FILES < <(git ls-files --cached --others --exclude-standard \
    'overlume/src' 'overlume/include' 'overlume/tests' 'overlume/tools' 'overlume/smoke' 'examples' \
    'ros/src/overlume_ros' \
    | grep -E '\.(c|cc|cpp|cxx|h|hpp|hxx|mat)$' \
    | grep -v '/fixtures/')

mapfile -t SCRIPT_FILES < <(git ls-files \
    'overlume/src' 'overlume/include' 'overlume/tests' 'overlume/tools' 'overlume/scripts' 'examples' \
    'ros/src/overlume_ros' \
    | grep -E '\.(py|sh)$' \
    | grep -v '/fixtures/')

mapfile -t ROOT_TOOLS_FILES < <(git ls-files 'tools/*.py' 'tools/*.sh' 'ros/colcon_build.sh')

mapfile -t CMAKE_FILES < <(git ls-files 'overlume/cmake' | grep -E '\.cmake$')
CMAKE_FILES+=("examples/CMakeLists.txt" "overlume/CMakeLists.txt" "ros/src/overlume_ros/CMakeLists.txt")
mapfile -t MAT_FILES < <(git ls-files 'overlume/assets/materials' | grep -E '\.mat$')

mapfile -t YAML_FILES < <(git ls-files 'ros/src/overlume_ros/config/*.yaml')

FILES=("${CPP_FILES[@]}" "${SCRIPT_FILES[@]}" "${ROOT_TOOLS_FILES[@]}" "${CMAKE_FILES[@]}" "${MAT_FILES[@]}" "${YAML_FILES[@]}")

if [[ ${#FILES[@]} -eq 0 ]]; then
    echo "error: no first-party files found — check the path list above" >&2
    exit 1
fi

missing=()
for f in "${FILES[@]}"; do
    if ! head -3 "${f}" | grep -q "SPDX-License-Identifier"; then
        missing+=("${f}")
    fi
done

if [[ ${#missing[@]} -gt 0 ]]; then
    echo "check_spdx.sh: FAIL — ${#missing[@]} file(s) missing SPDX header in their first 3 lines:" >&2
    printf '  %s\n' "${missing[@]}" >&2
    exit 1
fi

echo "check_spdx.sh: PASS (${#FILES[@]} files carry the SPDX identifier)"
