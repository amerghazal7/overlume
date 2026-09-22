#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO_ROOT}"

PINNED_VERSION="23.1.1"
CLANG_FORMAT="${HOME}/.local/bin/clang-format"

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

FIX=0
if [[ "${1:-}" == "--fix" ]]; then FIX=1; shift; fi
if [[ $# -gt 0 ]]; then
    FILES=()
    for f in "$@"; do
        case "$f" in
            *.c|*.cc|*.cpp|*.cxx|*.h|*.hh|*.hpp|*.hxx) FILES+=("$f") ;;
            *) echo "check_format.sh: not a C/C++ source, refusing to format: $f" >&2; exit 2 ;;
        esac
    done
fi
if [[ "${FIX}" == "1" ]]; then
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
