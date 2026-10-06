#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# test_pick_releases.sh -- pick_releases.sh exit codes: fits 0, truncated 0, too big 1, no packages 3.
set -uo pipefail
p="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/pick_releases.sh"
bad() { echo "FAIL: $*"; exit 1; }
run() { printf '%s' "$1" | "$p" "$2" 2>/dev/null; }

out="$(run '[{"tag":"v2","bytes":10},{"tag":"v1","bytes":20}]' 100)"; rc=$?
[ $rc -eq 0 ] && [ "$out" = $'N=2\nv2\nv1' ] || bad "fits: rc=$rc out=$out"
out="$(run '[{"tag":"v2","bytes":10},{"tag":"v1","bytes":20}]' 15)"; rc=$?
[ $rc -eq 0 ] && [ "$out" = $'N=1\nv2' ] || bad "truncated: rc=$rc out=$out"
run '[{"tag":"v2","bytes":950},{"tag":"v1","bytes":0}]' 899 >/dev/null; rc=$?
[ $rc -eq 1 ] || bad "too big: rc=$rc (want 1)"
run '[{"tag":"v1","bytes":0}]' 899 >/dev/null; rc=$?
[ $rc -eq 3 ] || bad "no packages: rc=$rc (want 3)"
run '[]' 899 >/dev/null; rc=$?
[ $rc -eq 3 ] || bad "empty: rc=$rc (want 3)"
echo PASS
