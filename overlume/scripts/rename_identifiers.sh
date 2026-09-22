#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${REPO_ROOT}"

PATHSPEC_EXCLUDES=(
    ':!docs/plans' ':!docs/superpowers' ':!docs/adr' ':!docs/evidence'
    ':!*.png' ':!*.b3dm' ':!*.glb'
    ':!overlume/scripts/rename_identifiers.sh'
)

mapfile -t FILES < <(git grep -l -E \
    'mpviz|MPVIZ_|visual_renderer|micropilot_visualization_node|visualization_node|visualization_app|VisualizationNode' \
    -- . "${PATHSPEC_EXCLUDES[@]}" || true)

if [[ "${#FILES[@]}" -eq 0 ]]; then
    echo "rename_identifiers.sh: no matching files -- already renamed (or scope is empty)."
    exit 0
fi

echo "rename_identifiers.sh: rewriting ${#FILES[@]} file(s)."

for f in "${FILES[@]}"; do
    sed -i \
        -e 's/micropilot_visualization::visual_renderer/overlume::overlume/g' \
        -e 's/micropilot::visualization_app/overlume::ros/g' \
        -e 's/micropilot_visualization_node/overlume_ros/g' \
        -e 's/visualization_node/overlume_node/g' \
        -e 's/VISUAL_RENDERER/OVERLUME/g' \
        -e 's/visual_renderer/overlume/g' \
        -e 's/MPVIZ_/OVERLUME_/g' \
        -e 's/mpviz/overlume/g' \
        -e 's/\bVisualizationNode\b/OverlumeNode/g' \
        "${f}"
done

echo "rename_identifiers.sh: done."
