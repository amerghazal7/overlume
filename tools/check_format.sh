#!/usr/bin/env bash
# clang-format gate for Overlume's first-party C/C++ (Task 6b).
#
# Pinned formatter: clang-format 23.1.1 (newest on PyPI as of 2026-09-17,
# `python3 -m pip index versions clang-format`), installed with:
#   pip install --user clang-format==23.1.1
# Using the pinned wheel (not this box's system clang-format 22.1.8) is the
# whole point: a different clang-format minor version can reformat
# identical source differently, so CI and local runs must use the same
# binary. This script tries the pinned install path first and falls back to
# PATH only if that's absent; either way the version check below is what
# actually enforces the pin, refusing to run against any other version.
#
# File set: every first-party C/C++ source under
# overlume/{src,include,tests,smoke,tools}, examples/, and
# ros/src/overlume_ros/{src,include,test} (via `git ls-files`, so
# untracked/build/fixture directories never enter the set) filtered to C/C++
# extensions (.c .cc .cpp .cxx .h .hpp .hxx). This already excludes
# fixtures/goldens (they're .yaml/.json/.png, not C/C++), and generated
# files (never tracked by git).
#
# Usage:
#   tools/check_format.sh          # dry-run, fails on any formatting diff
#   tools/check_format.sh --fix    # reformat in place

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

PINNED_VERSION="23.1.1"
CLANG_FORMAT="${HOME}/.local/bin/clang-format"

# Fall back to PATH when the pinned wheel landed somewhere other than
# ~/.local/bin (a venv, pipx, or a system-wide install of the same pinned
# version) -- the version check below is what actually enforces the pin,
# this is just where we look for a candidate binary first.
if [[ ! -x "${CLANG_FORMAT}" ]] && command -v clang-format >/dev/null 2>&1; then
    CLANG_FORMAT="$(command -v clang-format)"
fi

if [[ ! -x "${CLANG_FORMAT}" ]]; then
    echo "error: pinned clang-format not found at ${HOME}/.local/bin/clang-format or on PATH" >&2
    echo "  install it with: pip install --user clang-format==${PINNED_VERSION}" >&2
    exit 1
fi

FOUND_VERSION="$("${CLANG_FORMAT}" --version | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -1)"
if [[ "${FOUND_VERSION}" != "${PINNED_VERSION}" ]]; then
    echo "error: ${CLANG_FORMAT} is version ${FOUND_VERSION}, expected ${PINNED_VERSION}" >&2
    echo "  reinstall with: pip install --user clang-format==${PINNED_VERSION}" >&2
    exit 1
fi

mapfile -t FILES < <(git ls-files \
    'overlume/include' 'overlume/src' 'overlume/tests' 'overlume/smoke' 'overlume/tools' \
    'examples' \
    'ros/src/overlume_ros/src' 'ros/src/overlume_ros/include' 'ros/src/overlume_ros/test' \
    | grep -E '\.(c|cc|cpp|cxx|h|hpp|hxx)$')

if [[ ${#FILES[@]} -eq 0 ]]; then
    echo "error: no first-party C/C++ files found — check the path list above" >&2
    exit 1
fi

if [[ "${1:-}" == "--fix" ]]; then
    "${CLANG_FORMAT}" -i "${FILES[@]}"
    echo "check_format.sh: reformatted ${#FILES[@]} files with clang-format ${PINNED_VERSION}"
    exit 0
fi

if "${CLANG_FORMAT}" --dry-run -Werror "${FILES[@]}"; then
    echo "check_format.sh: PASS (${#FILES[@]} files, clang-format ${PINNED_VERSION})"
    exit 0
else
    echo "check_format.sh: FAIL — run 'tools/check_format.sh --fix'" >&2
    exit 1
fi
