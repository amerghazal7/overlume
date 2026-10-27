#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
#
# Usage: provision_ego_model.sh <model.glb>
# Copies a glTF binary to assets/ego/ego.glb, which the node installs as its default ego model.
# An OBJ source: convert it first with overlume/scripts/obj2gltf.py <input.obj> <output.glb>.

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$HERE")"
OUT="$PKG_DIR/assets/ego/ego.glb"

if [[ $# -ne 1 ]]; then
  echo "usage: provision_ego_model.sh <model.glb>" >&2
  echo "  (an OBJ model: overlume/scripts/obj2gltf.py <input.obj> <output.glb> first)" >&2
  exit 2
fi
if [[ ! -f "$1" ]]; then
  echo "provision_ego_model.sh: model not found at '$1'" >&2
  echo "  Without one, ego_model_path resolves empty and the node's clay-box" >&2
  echo "  fallback is what ships -- an honest gap, not an error." >&2
  exit 1
fi

mkdir -p "$(dirname "$OUT")"
cp -- "$1" "$OUT"
echo "provisioned $OUT -- re-run this package's cmake configure to pick it up."
