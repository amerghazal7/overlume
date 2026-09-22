#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

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
