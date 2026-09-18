#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# FOLLOW-UP 5 regression check: liboverlume_node_lib.so must not dynamically
# EXPORT symbols from cesium-native's vendored C libraries (sqlite3/curl/
# openssl/zlib/zstd/uriparser -- Decision 4.3's "recorded, not renamed" set,
# merged into liboverlume.a as ordinary static archives -- see
# CMakeLists.txt's comment on this package's OVERLUME_ENABLE_CESIUM glob).
# The SAME process also maps the SYSTEM copies of these libraries (pulled in
# by ROS/rmw/python), so two exported copies of e.g. `sqlite3_bind_text` is
# runtime symbol interposition -- which one a caller binds to depends on
# load order, and a version mismatch can misbehave or crash silently.
#
# FIX ROUND 1 (2026-09-18): the original fix here was
# target_link_options(overlume_node_lib PRIVATE "LINKER:--exclude-libs,ALL"),
# which also hid the vendored libc++abi.a's C++ ABI runtime (__cxa_throw,
# __gxx_personality_v0, ...) right along with these C libraries -- ld's
# --exclude-libs marks EVERY symbol from a listed static archive STV_HIDDEN,
# with no way to carve out an exception once applied (confirmed empirically:
# a version script's `global:` rule cannot un-hide a symbol --exclude-libs
# already marked hidden). That split this .so's exception runtime from the
# rest of the process (it still dynamically links libstdc++, `ldd` shows it
# -- only the symbol's visibility, and therefore whether the two copies
# unify, changed), so an exception thrown by libtf2.so's libstdc++-linked
# code aborted instead of being caught by this library's own
# `catch (const tf2::TransformException&)` (TfAdapter::update()).
#
# CMakeLists.txt's fix: drop --exclude-libs,ALL and reach the same "hide the
# vendored C libraries" outcome with an explicit `--version-script` that
# lists only the sqlite3_*/curl_*/SSL_*/EVP_*/inflate*/deflate*/uriParse*/
# zstd* pattern set below under `local:` -- everything else (the C++ ABI
# runtime included) keeps normal default (global) visibility. This script
# checks BOTH halves: the vendored C libraries stay hidden, AND the ABI
# runtime stays exported -- so a future regression back to --exclude-libs,ALL
# (or an over-broad version script) fails loudly here instead of only
# showing up as a live TF-exception abort.
#
# Usage: test_exported_symbols.sh <path-to-liboverlume_node_lib.so>
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

# MUST STAY IN SYNC with the pattern set the orchestrator verified by hand
# (FOLLOW-UP 5's measured baseline: 286 sqlite3_*, 18 curl_easy_*, 536 SSL_*,
# 913 EVP_*, 19 inflate*, 18 deflate* symbols before this fix).
pattern='^(sqlite3_|curl_|SSL_|EVP_|inflate|deflate|uriParse|zstd)'

# Captured once, then grepped via here-strings below (not piped straight from
# nm again) -- `grep -q` stops reading after its first match and SIGPIPEs the
# upstream process, which `pipefail` turns into a nonzero pipeline status
# even though the grep itself matched. A here-string has no producer process
# to SIGPIPE.
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

# The other half (FIX ROUND 1): the C++ ABI runtime must stay GLOBAL/exported
# (defined 'T' in .dynsym, not local 't') -- if this regresses back to
# --exclude-libs,ALL, or a version script wide enough to catch these too, a
# TF/tf2 exception crossing this .so's boundary aborts the process instead of
# being caught (see the header comment above).
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
