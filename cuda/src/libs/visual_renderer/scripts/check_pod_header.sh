#!/usr/bin/env bash
# Fails if any public API header leaks std:: types across the ABI boundary.
set -euo pipefail
include_dir="$(dirname "$0")/../include/visual_renderer"
shopt -s nullglob
headers=("$include_dir"/*.h)
if [ ${#headers[@]} -eq 0 ]; then
  echo "no headers found under $include_dir"
  exit 1
fi
for hdr in "${headers[@]}"; do
  if grep -nE '#include <(string|vector|memory|functional|optional|map|span)>|std::' "$hdr"; then
    echo "POD violation: std:: in public API ($hdr)"
    exit 1
  fi
done
