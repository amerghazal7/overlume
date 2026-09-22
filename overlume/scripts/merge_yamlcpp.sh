#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

set -euo pipefail

ar_tool="$1"; ld_tool="$2"; objcopy_tool="$3"; nm_tool="$4"
work_dir="$5"; merged_o="$6"; target_archive="$7"
shift 7
own_objs=()
archives=()
_parsing_objs=1
for arg in "$@"; do
  if [ "$_parsing_objs" -eq 1 ] && [ "$arg" = "--" ]; then
    _parsing_objs=0
    continue
  fi
  if [ "$_parsing_objs" -eq 1 ]; then
    own_objs+=("$arg")
  else
    archives+=("$arg")
  fi
done
if [ ${#archives[@]} -eq 0 ]; then
  echo "merge_yamlcpp.sh: no archives given (expected yaml-cpp + cesium + spdlog after --)" >&2
  exit 1
fi

rm -rf "$work_dir"
mkdir -p "$work_dir"

shopt -s nullglob
extracted_objs=()
idx=0
for archive in "${archives[@]}"; do
  idx=$((idx + 1))
  sub="$work_dir/a$idx"
  mkdir -p "$sub"
  ( cd "$sub" && "$ar_tool" x "$archive" )
  for o in "$sub"/*.o; do
    extracted_objs+=("$o")
  done
done
if [ ${#extracted_objs[@]} -eq 0 ]; then
  echo "merge_yamlcpp.sh: no .o files extracted from any of: ${archives[*]}" >&2
  exit 1
fi

"$ld_tool" -r -o "$merged_o" "${own_objs[@]}" "${extracted_objs[@]}"

rename_map="${merged_o}.vendor_rename.txt"
"$nm_tool" --defined-only "$merged_o" | awk '{print $3}' \
  | grep -E '4YAML|6spdlog|N3fmt[0-9]' | sort -u \
  | awk '{print $1, "overlume_vendored_" $1}' > "$rename_map"
if [ ! -s "$rename_map" ]; then
  echo "merge_yamlcpp.sh: found zero YAML::/spdlog::/fmt:: symbols in $merged_o -- something upstream broke" >&2
  exit 1
fi
"$objcopy_tool" --redefine-syms="$rename_map" "$merged_o"

rm -f "$target_archive"
"$ar_tool" crs "$target_archive" "$merged_o"

undefined_survivors=$("$nm_tool" --undefined-only "$target_archive" 2>/dev/null \
  | awk '{print $2}' | grep -E '4YAML|6spdlog|N3fmt[0-9]' | grep -v '^overlume_vendored_' | sort -u || true)
defined_survivors=$("$nm_tool" --defined-only "$target_archive" 2>/dev/null \
  | awk '{print $3}' | grep -E '^_Z.*(4YAML|6spdlog|N3fmt[0-9])' | sort -u || true)
if [ -n "$undefined_survivors" ] || [ -n "$defined_survivors" ]; then
  echo "merge_yamlcpp.sh: un-renamed YAML::/spdlog::/fmt:: symbols survive in $target_archive after the merge -- ABI hazard, not proceeding." >&2
  [ -n "$undefined_survivors" ] && { echo "  undefined survivors:" >&2; echo "$undefined_survivors" >&2; }
  [ -n "$defined_survivors" ] && { echo "  defined survivors:" >&2; echo "$defined_survivors" >&2; }
  exit 1
fi
