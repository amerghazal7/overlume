#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# check_shared_exports.sh NM TOOL LIB -- TOOL is readelf for an ELF LIB, otool for a Mach-O one.
# Fails if LIB exports anything but overlume:: or depends on a C++ runtime / system library
# beyond the platform floor.
set -euo pipefail
nm_tool="$1"; dep_tool="$2"; lib="$3"
if [ "$(head -c4 "$lib" | od -An -tx1 | tr -d ' \n')" = 7f454c46 ]; then macho=0; else macho=1; fi

if [ $macho = 0 ]; then
  bad=$("$nm_tool" -D --defined-only -C "$lib" | awk '$2 ~ /^[TDBRVW]$/ {sub(/^[^ ]+ [^ ]+ /,""); print}' \
        | grep -vE '^overlume::|^(_init|_fini|_edata|_end|__bss_start)$' || true)
  if [ -n "$bad" ]; then echo "FAIL: non-overlume exports:"; echo "$bad" | head -20; exit 1; fi
  needed=$("$dep_tool" -d "$lib" 2>/dev/null | awk '/NEEDED/ {print $NF}' | tr -d '[]')
  echo "NEEDED: ${needed//$'\n'/ }"
  if echo "$needed" | grep -qE 'libc\+\+|libc\+\+abi|libunwind|libyaml-cpp|libstdc\+\+'; then
    echo "FAIL: runtime leaked into NEEDED"; exit 1; fi
  echo PASS; exit 0
fi

# Mach-O (macOS dylib, iOS framework binary): external defined symbols, demangled.
bad=$("$nm_tool" -gU "$lib" | awk 'NF >= 3 {print substr($NF, 2)}' | c++filt | grep -vE '^overlume::' || true)
if [ -n "$bad" ]; then echo "FAIL: non-overlume exports:"; echo "$bad" | head -20; exit 1; fi
if [ -z "$("$nm_tool" -gU "$lib" | awk 'NF >= 3 {print substr($NF, 2)}' | c++filt | grep -E '^overlume::' | head -1)" ]; then
  echo "FAIL: no overlume:: symbol exported"; exit 1; fi
# Dependencies: the dylib's own id line is the first entry; everything else must be a system
# library (libc++ stays dynamic on Apple: it is the OS's, not ours to bake in).
deps=$("$dep_tool" -L "$lib" | tail -n +2 | awk '{print $1}')
echo "DEPS: ${deps//$'\n'/ }"
leaked=$(echo "$deps" | grep -vE '^(/usr/lib/lib(c\+\+\.1|System\.B|z\.1|objc\.A|resolv\.9)\.dylib|/System/Library/Frameworks/.*|@rpath/(liboverlume[^ ]*\.dylib|Overlume\.framework/Overlume))$' || true)
if [ -n "$leaked" ]; then echo "FAIL: unexpected dependencies:"; echo "$leaked"; exit 1; fi
echo PASS
