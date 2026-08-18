#!/usr/bin/env bash
# Fails if the public API header leaks std:: types across the ABI boundary.
set -euo pipefail
hdr="$(dirname "$0")/../include/visual_renderer/api.h"
[ -f "$hdr" ] || { echo "api.h missing"; exit 1; }
if grep -nE '#include <(string|vector|memory|functional|optional|map|span)>|std::' "$hdr"; then
  echo "POD violation: std:: in public API"; exit 1
fi
