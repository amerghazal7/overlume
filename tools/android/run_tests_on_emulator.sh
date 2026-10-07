#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# run_tests_on_emulator.sh BUILD_DIR -- run the Android build's tests on the attached emulator or
# device: every `cpu` test plus ReadbackOrientation.* (label gpu). BUILD_DIR must have been
# configured by build_all_abis.sh (CMAKE_CROSSCOMPILING_EMULATOR=adb_run.sh, test data dir
# /data/local/tmp/ov/data). Exits 1 on any failure; prints PASS/FAIL only.
# --print-plan BUILD_DIR: no device needed; lists the adb steps and the tests ctest would run
# (local check; emulators are CI-only on the dev host, see test_emulator_script.sh).
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd "$here/../.." && pwd)"
plan=0; [ "${1:-}" = --print-plan ] && { plan=1; shift; }
build="${1:?usage: $0 [--print-plan] BUILD_DIR}"
adb="${ADB:-adb}"
if [ $plan = 1 ]; then
    # ctest cannot list a cross build's cases without a device (it runs the binaries to list them),
    # so the plan names the test executables the device run will cover.
    echo "adb push overlume/tests overlume/assets -> /data/local/tmp/ov/data/; themes -> /data/local/tmp/ov/share/overlume/themes"
    echo "ctest -L cpu -E FiftyObjectsSceneUpdateUnderTwoMilliseconds, each case run via adb_run.sh; executables:"
    n=0
    for t in "$build"/test_*; do
        [ -x "$t" ] && [ -f "$t" ] || continue
        echo "  $(basename "$t")"; n=$((n + 1))
    done
    [ $n -gt 0 ] || { echo "FAIL: no test executables in $build"; exit 1; }
    echo "ctest -R ^ReadbackOrientation\\."
    echo "PASS: plan ($build, $n test executables)"; exit 0
fi
# Cross builds use PRE_TEST gtest discovery: each test_*_include.cmake hard-codes the configuring
# CMake's GoogleTestAddTests.cmake path, so the running ctest needs that same CMake install.
for f in "$build"/*_include.cmake; do
    inc="$(sed -n 's/^ *include("\(.*GoogleTestAddTests.cmake\)")$/\1/p' "$f" | head -1)"
    [ -z "$inc" ] || [ -f "$inc" ] || { echo "FAIL: $inc missing (CMake that configured $build is not installed here)"; exit 1; }
done
"$adb" wait-for-device
until [ "$("$adb" shell getprop sys.boot_completed | tr -d '\r')" = 1 ]; do sleep 2; done
"$adb" shell 'rm -rf /data/local/tmp/ov; mkdir -p /data/local/tmp/ov/bin /data/local/tmp/ov/tmp /data/local/tmp/ov/data'
"$adb" push "$repo/overlume/tests" "$repo/overlume/assets" /data/local/tmp/ov/data/ >/dev/null
# Installed-layout lookup for tests that pass no theme dir: <exe>/../share/overlume/themes.
"$adb" shell 'mkdir -p /data/local/tmp/ov/share/overlume; cp -r /data/local/tmp/ov/data/assets/themes /data/local/tmp/ov/share/overlume/themes'
rc=0
# FiftyObjectsSceneUpdateUnderTwoMilliseconds is a wall-clock budget a software GL emulator cannot meet.
ctest --test-dir "$build" -L cpu -E FiftyObjectsSceneUpdateUnderTwoMilliseconds --no-tests=error --output-on-failure || rc=1
ctest --test-dir "$build" -R '^ReadbackOrientation\.' --no-tests=error --output-on-failure || rc=1
if [ $rc -eq 0 ]; then echo "PASS: android tests ($build)"; else echo "FAIL: android tests ($build)"; fi
exit $rc
