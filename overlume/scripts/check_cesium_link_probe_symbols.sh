#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

set -euo pipefail

nm_tool="$1"; probe_path="$2"
system_libspdlog="${3:-/lib/x86_64-linux-gnu/libspdlog.so.1}"

if [ ! -x "$probe_path" ] && [ ! -f "$probe_path" ]; then
  echo "check_cesium_link_probe_symbols.sh: no such file: $probe_path" >&2
  exit 1
fi

if [ ! -e "$system_libspdlog" ]; then
  echo "check_cesium_link_probe_symbols.sh: $system_libspdlog not present on this box -- skipping the overlap check (nothing to overlap against here); this is a PASS-by-absence, not a proof of safety on a box where the node's system libspdlog lives elsewhere." >&2
  exit 0
fi

probe_defined=$("$nm_tool" --defined-only "$probe_path" 2>/dev/null | awk '{print $3}' | grep -E '6spdlog|N3fmt[0-9]' | grep -v '^overlume_vendored_' | sort -u || true)
system_defined=$("$nm_tool" -D --defined-only "$system_libspdlog" 2>/dev/null | awk '{print $3}' | sort -u || true)

overlap=$(comm -12 <(printf '%s\n' "$probe_defined") <(printf '%s\n' "$system_defined") | sed '/^$/d')
if [ -n "$overlap" ]; then
  echo "check_cesium_link_probe_symbols.sh: $probe_path defines symbol(s) also defined by $system_libspdlog -- the exact silent-ABI-corruption shape (a clang/libc++ definition here shadowing/shadowed-by a gcc/libstdc++ one there):" >&2
  echo "$overlap" >&2
  exit 1
fi
echo "check_cesium_link_probe_symbols.sh: OK -- no spdlog/fmt symbol overlap between $probe_path and $system_libspdlog"
