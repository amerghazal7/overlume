#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
set -euo pipefail
nm_tool="$1"; readelf_tool="$2"; lib="$3"
bad=$("$nm_tool" -D --defined-only -C "$lib" | awk '$2 ~ /^[TDBRVW]$/ {sub(/^[^ ]+ [^ ]+ /,""); print}' \
      | grep -vE '^overlume::|^(_init|_fini|_edata|_end|__bss_start)$' || true)
if [ -n "$bad" ]; then echo "FAIL: non-overlume exports:"; echo "$bad" | head -20; exit 1; fi
needed=$("$readelf_tool" -d "$lib" 2>/dev/null | awk '/NEEDED/ {print $NF}' | tr -d '[]')
echo "NEEDED: ${needed//$'\n'/ }"
if echo "$needed" | grep -qE 'libc\+\+|libc\+\+abi|libunwind|libyaml-cpp|libstdc\+\+'; then
  echo "FAIL: runtime leaked into NEEDED"; exit 1; fi
echo PASS
