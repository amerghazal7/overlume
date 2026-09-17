# Agent instructions — Overlume

Canonical instructions for AI coding agents working in this repository.
`CLAUDE.md` imports this file; keep everything here, nothing there.

## What this repo is

Overlume: a real-time rendering library (Filament, clang/libc++) behind a
POD-only public header pair, plus a ROS 2 lifecycle node that feeds it live
robot data. Layout and the 2026-09-17 restructure plan:
`docs/plans/2026-09-17-overlume-restructure.md` (its path map covers older
documents that still cite pre-restructure paths).

## Hard rules

- **Tokens by name only.** `CESIUM_ION_TOKEN` and `MAPBOX_TOKEN` are never
  committed, echoed, logged, or pasted into any file or transcript. Scripts
  print PASS/FAIL and HTTP codes only. No test may need live network or a token.
- **Public headers are POD-only and append-only** (ADR-0003/0004). Struct
  changes bump `kSceneVersion`; free functions bump nothing.
  `scripts/check_pod_header.sh` must pass.
- **Goldens are promoted by a human.** A failing golden is a finding, not a
  file to overwrite. Whole-frame SSIM passing is not proof a golden is current;
  use the per-pixel drift audit after any palette-wide change.
- **Behaviour changes ship with a runnable check** that fails when the change
  is reverted.

## How work is done

- Implementation and reviews run as dynamic workflows: orchestrator on the
  session model, implementers on Sonnet, review gates on Opus, at most two fix
  rounds per task; leftover minors are applied by the orchestrator.
- Every task ends green on `tools/ci_visual_mode.sh` (run in the foreground; a
  background run can be killed by a spurious low-memory guard) and is
  committed on its own with a message that says what changed and why.
- Status truth lives in `docs/status.md` and the active plan's ledger, not in
  chat and not in memory.

## Codebase questions

A knowledge graph exists at `graphify-out/` (ignored, regenerate with
`graphify update .`). Prefer `graphify query "<question>"`, `graphify path`,
`graphify explain` before grepping raw source; they return a scoped subgraph.
The graph is AST-only: for structure, docs, or tooling questions, read the
tree directly.

## Build and test

Library: `overlume/scripts/setup_toolchain_cesium.sh`, then `cmake -S overlume
-B overlume/build && cmake --build overlume/build -j`. Node: `ros/colcon_build.sh`.
Gate: `tools/ci_visual_mode.sh`. Live rig: `tools/validate_visual_mode.sh --live`.
(Paths are the post-restructure ones; until Task 1 of the plan lands, the
library is at `cuda/src/libs/visual_renderer` and the node at
`cuda/src/ros_apps/src/micropilot_visualization_node`.)
