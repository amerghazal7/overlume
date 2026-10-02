#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# Usage: make_test_bundle.sh <build-dir> <out.tar.gz>   (run from the repo root)
# Packs what `ctest` needs from a cross-built tree to run natively on another
# machine: the ctest metadata, the test/probe executables and liboverlume. Paths
# are repo-root relative; unpack at the repo root of a checkout mounted at the
# same absolute path as in the build container (/src), because the metadata and
# the compiled-in test data directories are absolute.
set -euo pipefail
build="${1:?usage: $0 <build-dir> <out.tar.gz>}"
out="${2:?usage: $0 <build-dir> <out.tar.gz>}"
list="$(mktemp)"; trap 'rm -f "$list"' EXIT
{
    find "$build" -name CTestTestfile.cmake -o -name '*_tests.cmake' -o -name '*_include.cmake'
    # Top-level executables (tests, probes, tools) and the shared library.
    find "$build" -maxdepth 1 -type f \( -perm -u+x -o -name 'liboverlume.so*' \)
} | LC_ALL=C sort -u > "$list"
[ -s "$list" ] || { echo "FAIL: nothing to bundle under $build" >&2; exit 1; }
tar -czf "$out" -T "$list"
echo "PASS: $(wc -l < "$list") files -> $out ($(du -h "$out" | cut -f1))"
