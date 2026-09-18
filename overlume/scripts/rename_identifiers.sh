#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
# rename_identifiers.sh — Task 2 of the Overlume restructure (2026-09-17).
#
# ONE-SHOT. Kept for the record, not meant to run again on this repo (it is
# idempotent by construction -- a second run is a no-op once the patterns
# below no longer match anything; the selector deliberately omits 'micropilot::'
# so scene.h's preserved provenance comment is never re-selected -- but it exists to document exactly what
# Task 2 changed, not as a recurring tool).
#
# What this does NOT do: `git mv` -- sed cannot rename paths. Six `git mv`s
# were run separately, before this script (so the sed passes below rewrite
# the post-move tree in one go):
#   overlume/include/visual_renderer                       -> overlume/include/overlume
#   ros/src/micropilot_visualization_node                  -> ros/src/overlume_ros
#   ros/src/overlume_ros/include/micropilot_visualization_node -> .../include/overlume_ros
#   ros/src/overlume_ros/src/visualization_node.cpp         -> .../src/overlume_node.cpp
#   ros/src/overlume_ros/include/overlume_ros/visualization_node.hpp -> .../overlume_node.hpp
#   ros/src/overlume_ros/launch/visualization_node.launch.py -> .../overlume_node.launch.py
# The last three are not strictly required by the plan's rename table (only
# the include dir and package dir are named there), but every one of the
# ~65 comments/#includes that name these three files reads correctly only if
# the files are renamed too -- left as `visualization_node.*` they would
# have gone stale the moment the sed passes below rewrote every reference
# to them into `overlume_node.*`.
#
# Scope: every tracked, non-binary file except the frozen historical trees
# (docs/plans, docs/superpowers, docs/adr, docs/evidence) and golden/fixture
# binaries (*.png *.b3dm *.glb) -- per the plan's Global constraints
# ("data contracts are not identifiers", "history is not rewritten").
#
# Rules, applied in this exact order (order matters: each later rule's
# pattern is disjoint from, or a strict subset that must come AFTER, the
# rule(s) before it -- see the plan's Task 2 table and the survey in the
# Task 2 report for why):
#
#   1. `micropilot_visualization::visual_renderer` (the CMake ALIAS target,
#      one line in overlume/CMakeLists.txt) -> `overlume::overlume`
#      Must run before rule 6 (`visual_renderer` -> `overlume`), which would
#      otherwise leave the ALIAS namespace half as the stale
#      `micropilot_visualization::overlume`.
#   2. `micropilot::visualization_app` (the node's C++ namespace)
#      -> `overlume::ros`
#   3. `micropilot_visualization_node` (ROS package name / include-dir
#      prefix / python module) -> `overlume_ros`
#      Must run before rule 4: this string CONTAINS `visualization_node` as
#      a substring ("micropilot_" + "visualization_node"), so rule 4 first
#      would produce the wrong `micropilot_overlume_node`.
#   4. `visualization_node` (bare -- the CMake executable/library target,
#      the rclcpp node NAME, and every fully-qualified `/visualization_node/
#      ...` ROS-graph topic/service derived from that node name)
#      -> `overlume_node`
#   5. `VISUAL_RENDERER` (uppercase CMake vars: VISUAL_RENDERER_DIR/_LIB/
#      _SOURCES) -> `OVERLUME`
#   6. `visual_renderer` (lowercase: CMake project/target names, the
#      `libvisual_renderer.a` archive name, `visual_renderer_prebuilt`/
#      `_stream`/`_merged`, the `include/visual_renderer` path segment,
#      `#include <visual_renderer/...>`) -> `overlume`
#   7. `MPVIZ_` (20 env vars + CMake cache options, e.g.
#      MPVIZ_ENABLE_CESIUM, MPVIZ_TEST_DATA_DIR, MPVIZ_NODE_*, MPVIZ_CLANGXX,
#      MPVIZ_SHOWCASE*, MPVIZ_CAPTURE_*, MPVIZ_EMIT_GEOM, MPVIZ_LIVE_ION_PERF,
#      MPVIZ_THEME_DIR, MPVIZ_DEFAULT_THEME_DIR) -> `OVERLUME_`
#   8. `mpviz` (lowercase: the library's C++ namespace `mpviz`/`mpviz::`,
#      CMake-internal `_mpviz_*` vars, the `mpviz-toolchain(-cesium)` and
#      `mpviz-tile-cache` cache-dir names, the spdlog logger "mpviz.cesium",
#      the `mpviz_node_test_paths`/`mpviz_vendored_*` internal names)
#      -> `overlume`
#
# Rules 5-8 are plain substring substitutions (no word-boundary anchoring):
# every one of the compound names above (e.g. `visual_renderer_prebuilt`,
# `_mpviz_vr_cache`) is a single token that CONTAINS the pattern, not a
# standalone word bounded by non-identifier characters, so a `\b`-anchored
# regex would silently miss the trailing-`_`-attached forms. Verified safe:
# none of these eight patterns is a substring of unrelated, real-word text
# anywhere in this repo (confirmed by the Task 2 survey).
#
# --- 2026-09-17: open follow-up 11 (C++ namespace merge, not run through --
# this script, recorded here only so the one-shot rename record stays
# complete) ---
#
# ros/src/overlume_ros carried two C++ namespaces left over from the Task 2
# rename above: `overlume_node` (31 files -- the former `visualization_node`
# namespace) and `overlume::ros` (the former `micropilot::visualization_app`).
# Merged into one: `overlume::ros` (tests: `overlume::ros::testing`).
#
# Survey: every symbol declared directly in each namespace (classes, structs,
# enums, free functions, constants, using-decls; anonymous-namespace helpers
# excluded) was enumerated and cross-checked -- ZERO name collisions between
# the two sets. No rename-on-merge was needed as a result; the merge is a
# pure mechanical namespace-token rewrite:
#   `namespace overlume_node {`            -> `namespace overlume::ros {`
#   `}  // namespace overlume_node`        -> `}  // namespace overlume::ros`
#   `overlume_node::` (incl. `overlume_node::testing`, `using overlume_node::X`)
#                                           -> `overlume::ros::` (`overlume::ros::testing`, ...)
# Left untouched (not C++ namespace tokens): the ROS node name string
# "overlume_node" (rclcpp_lifecycle::LifecycleNode ctor arg, log lines), the
# `overlume_node`/`overlume_node_lib`/`overlume_node_test_paths` CMake target
# names, the `OVERLUME_NODE_FIXTURES_DIR`/`OVERLUME_NODE_STB_DIR` macros, the
# `overlume_node.{hpp,cpp}` file names (in `#include`s and comments), and
# every ROS-graph reference in tools/vcam_ws_bridge.py / launch files / params
# YAML (topics, services, the node name) -- those name the ROS graph, not a
# C++ namespace.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${REPO_ROOT}"

# Exclusions match the plan's Global constraints exactly: historical docs are
# not rewritten; golden/fixture binaries are LFS and must not change; this
# script excludes itself so re-running it never rewrites its own comments.
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
