#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

set -euo pipefail
include_dir="$(dirname "$0")/../include/overlume"
shopt -s nullglob
headers=("$include_dir"/*.h*)
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
