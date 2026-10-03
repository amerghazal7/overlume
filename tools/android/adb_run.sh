#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# adb_run.sh EXE [ARGS...] -- CMAKE_CROSSCOMPILING_EMULATOR for Android builds: push EXE to the
# attached device/emulator and run it there, returning its exit status (so ctest and gtest
# discovery work unchanged). Device layout: /data/local/tmp/ov/{bin,data,tmp}; data/ is
# pushed by run_tests_on_emulator.sh. ADB / ANDROID_SERIAL are honoured.
set -euo pipefail
adb="${ADB:-adb}"
exe="$1"; shift
name="$(basename "$exe")"
"$adb" push "$exe" "/data/local/tmp/ov/bin/$name" >/dev/null
args=""
for a in "$@"; do args+=" $(printf '%q' "$a")"; done
"$adb" shell "mkdir -p /data/local/tmp/ov/tmp; cd /data/local/tmp/ov; chmod +x bin/$name; \
  TMPDIR=/data/local/tmp/ov/tmp ./bin/$name$args"
