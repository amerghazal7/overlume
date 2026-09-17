#!/usr/bin/env bash
# provision_ego_model.sh — VM-044: converts the real robot.obj (the deployed
# robot's ego proxy) into the glTF this package installs at
# share/overlume_visualization_node/assets/ego/ego.glb.
#
# NOT run by colcon build / CMakeLists.txt automatically: the source OBJ is
# ~143 MB and its converted glb ~72 MB -- past this repo's plain-git
# convention for binary assets (the already-committed car/pedestrian/
# truck_van glbs are all under 100 KB), and there is no Git LFS remote
# configured here. This script is the backlog's own documented alternative
# for an asset this size: a fetch/provision script, run once per box that
# actually has the source OBJ, before building this package.
#
# Usage: scripts/provision_ego_model.sh [path/to/robot.obj]
#   (defaults to $EGO_OBJ_SRC, then ~/Downloads/robot.obj — this box's own
#   convention, per robot_params.yaml's robot_model_transform history)
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$HERE")"
OBJ_SRC="${1:-${EGO_OBJ_SRC:-$HOME/Downloads/robot.obj}}"
OUT="$PKG_DIR/assets/ego/ego.glb"
# Same relative path CMakeLists.txt uses to locate visual_renderer from this
# package's own directory (ros/src/overlume_visualization_node
# -> overlume).
CONVERTER="$PKG_DIR/../../../overlume/scripts/obj2gltf.py"

if [[ ! -f "$OBJ_SRC" ]]; then
  echo "provision_ego_model.sh: source OBJ not found at '$OBJ_SRC'" >&2
  echo "  Pass a path, or set EGO_OBJ_SRC, to point at the real robot.obj." >&2
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
