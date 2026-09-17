#!/usr/bin/env bash
# VM-061 gate round 2, Finding 1 fix_instruction point 3: merge_yamlcpp.sh's
# own post-merge audit (see its header comment) can only see what
# liboverlume.a itself defines/references -- it cannot see what a
# CONSUMER's own link line additionally pulls in. cesium_link_probe links
# cesium CMake targets (CesiumGeospatial, CesiumGltfReader) directly,
# alongside overlume, for header access (CMakeLists.txt's own
# comment on that target explains why) -- so it's the one place in this
# tree that can independently prove the un-renamed vcpkg spdlog/fmt
# archives contribute nothing dangerous to a REAL final executable, not
# just to the library archive.
#
# The actual hazard (Decision 4, gate round 2 finding 1): a raw, un-renamed
# spdlog:: symbol landing in a final binary can resolve against the node
# process's own SEPARATELY-LOADED gcc/libstdc++ libspdlog.so.1 instead of
# this project's own clang/libc++ copy -- silent ABI corruption. So this
# checks for OVERLAP with that system library's defined symbols, the same
# check merge_yamlcpp.sh's header comment documents for the archive, not a
# bare "zero raw spdlog symbols" count (cesium_link_probe legitimately CAN
# carry some -- e.g. cesium's own vendored spdlog init code that never
# crosses the node boundary -- as long as none of it is a symbol the
# system's libspdlog.so.1 also defines).
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

# Unanchored, length-prefixed tokens (NOT _ZN-anchored): vtables (_ZTVN6spdlog...),
# typeinfo (_ZTI/_ZTSN6spdlog...) and const methods (_ZNK6spdlog...) don't start
# with _ZN, and those COMDAT-foldable symbols are exactly Decision 4's hazard.
# MUST STAY IN SYNC with merge_yamlcpp.sh's rename-map token set (its twin).
# Already-renamed overlume_vendored_* symbols are ours, not overlap candidates.
probe_defined=$("$nm_tool" --defined-only "$probe_path" 2>/dev/null | awk '{print $3}' | grep -E '6spdlog|N3fmt[0-9]' | grep -v '^overlume_vendored_' | sort -u || true)
system_defined=$("$nm_tool" -D --defined-only "$system_libspdlog" 2>/dev/null | awk '{print $3}' | sort -u || true)

overlap=$(comm -12 <(printf '%s\n' "$probe_defined") <(printf '%s\n' "$system_defined") | sed '/^$/d')
if [ -n "$overlap" ]; then
  echo "check_cesium_link_probe_symbols.sh: $probe_path defines symbol(s) also defined by $system_libspdlog -- the exact silent-ABI-corruption shape (a clang/libc++ definition here shadowing/shadowed-by a gcc/libstdc++ one there):" >&2
  echo "$overlap" >&2
  exit 1
fi
echo "check_cesium_link_probe_symbols.sh: OK -- no spdlog/fmt symbol overlap between $probe_path and $system_libspdlog"
