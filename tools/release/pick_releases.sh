#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# pick_releases.sh BUDGET_BYTES < releases.json -- stdin is newest-first JSON
# [{"tag":"v1","bytes":N}, ...] (bytes = size of that release's .deb/.rpm
# assets). Prints "N=<count>" then one tag per line: the newest releases that
# carry packages and fit the budget. Releases without packages are skipped.
# Exit 1 if packages exist but none fit the budget, 3 if no release carries packages at all.
set -euo pipefail
[ $# -eq 1 ] || { echo "usage: $0 BUDGET_BYTES < releases.json" >&2; exit 2; }
in="$(cat)"
out="$(jq -r --argjson budget "$1" '
  [.[] | select(.bytes > 0)]
  | reduce .[] as $r ({sum: 0, stop: false, tags: []};
      if .stop or (.sum + $r.bytes > $budget) then .stop = true
      else .sum += $r.bytes | .tags += [$r.tag] end)
  | .tags | "N=\(length)", .[]' <<< "$in")"
echo "$out"
jq -e 'any(.[]; .bytes > 0)' <<< "$in" > /dev/null \
  || { echo "FAIL: no release carries packages" >&2; exit 3; }
case "$out" in N=0*) echo "FAIL: no release with packages fits the budget" >&2; exit 1;; esac
