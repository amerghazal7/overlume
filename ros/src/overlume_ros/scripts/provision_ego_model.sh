#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$HERE")"
OBJ_SRC="${1:-${M02P_OBJ_SRC:-$HOME/Downloads/M02P.obj}}"
OUT="$PKG_DIR/assets/ego/M02P.glb"
CONVERTER="$PKG_DIR/../../../overlume/scripts/obj2gltf_m02p.py"

if [[ ! -f "$OBJ_SRC" ]]; then
  echo "provision_ego_model.sh: source OBJ not found at '$OBJ_SRC'" >&2
  echo "  Pass a path, or set M02P_OBJ_SRC, to point at the real M02P.obj." >&2
  echo "  Without it, ego_model_path resolves empty and the node's existing" >&2
  echo "  clay-box fallback is what ships -- an honest gap, not an error." >&2
  exit 1
fi
if [[ ! -f "$CONVERTER" ]]; then
  echo "provision_ego_model.sh: converter not found at '$CONVERTER'" >&2
  exit 1
fi

mkdir -p "$(dirname "$OUT")"
python3 "$CONVERTER" "$OBJ_SRC" "$OUT"
echo "provisioned $OUT -- re-run this package's cmake configure to pick it up."
