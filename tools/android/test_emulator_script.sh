#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# test_emulator_script.sh BUILD_DIR -- local check of run_tests_on_emulator.sh without a device or
# emulator (never launch one on the dev host): a fake adb and a fake ctest prove the script pushes
# what it should and exits 1 when any ctest invocation fails, 0 when all pass.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
build="${1:?usage: $0 BUILD_DIR}"
"$here/run_tests_on_emulator.sh" --print-plan "$build" | tail -1 | grep -q '^PASS: plan'
w="$(mktemp -d)"; trap 'rm -rf "$w"' EXIT
cat > "$w/adb" <<'F'
#!/usr/bin/env bash
echo "adb $*" >> "$FAKE_LOG"
[ "$1 $2" = "shell getprop" ] && echo 1
exit 0
F
cat > "$w/ctest" <<'F'
#!/usr/bin/env bash
echo "ctest $*" >> "$FAKE_LOG"
case "$*" in *"${FAKE_FAIL:-@@none@@}"*) exit 8;; esac
exit 0
F
chmod +x "$w/adb" "$w/ctest"
export FAKE_LOG="$w/log" ADB="$w/adb" PATH="$w:$PATH"
: > "$FAKE_LOG"
"$here/run_tests_on_emulator.sh" "$build" | grep -q '^PASS: android tests' || { echo "FAIL: green run"; exit 1; }
grep -q 'adb push .*/overlume/tests .*/overlume/assets /data/local/tmp/ov/data/' "$FAKE_LOG" || { echo "FAIL: fixtures not pushed"; exit 1; }
grep -q -- '-L cpu' "$FAKE_LOG" && grep -q 'ReadbackOrientation' "$FAKE_LOG" || { echo "FAIL: ctest selections"; exit 1; }
for sel in "-L cpu" "ReadbackOrientation"; do
    if FAKE_FAIL="$sel" "$here/run_tests_on_emulator.sh" "$build" >"$w/out" 2>&1; then echo "FAIL: '$sel' failure swallowed"; exit 1; fi
    grep -q '^FAIL: android tests' "$w/out" || { echo "FAIL: no FAIL line for '$sel'"; exit 1; }
done
# A CMake-version mismatch (PRE_TEST include path absent on this machine) must fail loudly up front.
mkdir "$w/bad"; printf '    include("/nonexistent/GoogleTestAddTests.cmake")\n' > "$w/bad/test_x[1]_include.cmake"
if "$here/run_tests_on_emulator.sh" "$w/bad" >"$w/out" 2>&1; then echo "FAIL: missing CMake include not detected"; exit 1; fi
grep -q 'nonexistent.*missing' "$w/out" || { echo "FAIL: guard message"; exit 1; }
echo "PASS: emulator script (plan, pushes, ctest selections, failure => exit 1)"
