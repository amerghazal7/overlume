#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

set -euo pipefail

if [ "$#" -ne 1 ]; then
  echo "usage: $0 <path-to-liboverlume_node_lib.so>" >&2
  exit 1
fi

lib="$1"
if [ ! -f "$lib" ]; then
  echo "test_exported_symbols.sh: no such file: $lib" >&2
  exit 1
fi

pattern='^(sqlite3_|curl_|SSL_|EVP_|inflate|deflate|uriParse|zstd)'

defined_dynsyms="$(nm -D --defined-only "$lib" | awk '{print $3}')"

offenders="$(grep -E "$pattern" <<< "$defined_dynsyms" || true)"

if [ -n "$offenders" ]; then
  count="$(printf '%s\n' "$offenders" | sed '/^$/d' | wc -l)"
  echo "test_exported_symbols.sh: FAIL -- $lib dynamically exports $count vendored C-library symbol(s)." >&2
  echo "This means the version script's 'local:' pattern set stopped hiding" >&2
  echo "them (a dependency bump changed how a static archive's symbols reach" >&2
  echo "the link, or the version-script link option was dropped from" >&2
  echo "CMakeLists.txt). Offending symbols:" >&2
  printf '%s\n' "$offenders" >&2
  exit 1
fi

abi_missing=""
for sym in __cxa_throw __gxx_personality_v0; do
  if ! grep -qx "$sym" <<< "$defined_dynsyms"; then
    abi_missing="${abi_missing}${sym} "
  fi
done

if [ -n "$abi_missing" ]; then
  echo "test_exported_symbols.sh: FAIL -- $lib no longer exports the C++ ABI runtime" \
       "symbol(s): $abi_missing" >&2
  echo "This library must share ONE C++ exception runtime with the rest of the" >&2
  echo "process (it dynamically links libstdc++.so.6) -- hiding these turns them" >&2
  echo "local, giving this .so a private copy that can't catch an exception" >&2
  echo "thrown by another libstdc++-linked library (e.g. libtf2.so)." >&2
  exit 1
fi

echo "test_exported_symbols.sh: OK -- 0 vendored C-library symbols exported by $lib," \
     "C++ ABI runtime still shared."
