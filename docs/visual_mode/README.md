# Visual mode

Docs for the C++/Filament rendering path (`cuda/src/libs/visual_renderer` +
`micropilot_visualization_node`), as distinct from this repo's root
`README.md` (the NumPy/pygame bowl-method prototype). See
`docs/superpowers/specs/2026-08-18-visual-mode-backlog.md` for the epic/task
backlog and `docs/superpowers/plans/` for the per-epic plans.

## Pre-merge gate (VM-041)

This repo has no hosted CI (no `.github/workflows`, no `.gitlab-ci.yml`).
`tools/ci_visual_mode.sh` is the whole of "CI wiring" until a hosted platform
exists — run it before merging any visual-mode change:

```bash
tools/ci_visual_mode.sh
```

It runs, in order, and labels each stage PASS/FAIL:

1. **POD header check** — `visual_renderer/scripts/check_pod_header.sh`
   (the public `include/visual_renderer/*.h` boundary stays `std::`-free).
2. **Library ctest suite** — configures + builds `visual_renderer`
   incrementally into `cuda/src/libs/visual_renderer/build` (clang/libc++
   toolchain) and runs its full `ctest` suite. Set `CI_VISUAL_MODE_CLEAN=1`
   to wipe that build dir first, so a stale library build can't mask a
   broken clean build — the node stage (3) still builds incrementally.
3. **Node gtests** — `colcon build` + `colcon test` for
   `micropilot_visualization_node`. Needs a ROS install and
   `micropilot_rendering_node` already built+installed somewhere sourceable
   read-only (default: this checkout's own `cuda/install/ros_apps`; override
   with `CI_VISUAL_MODE_ROS_APPS_INSTALL=/path/to/setup.bash` when borrowing
   another checkout's install space, e.g. from a worktree). Missing ROS/colcon
   infra **fails this stage loudly** — a pre-merge gate never silently skips
   the node.
4. **WS bridge pytest suite** — `tools/test_vcam_ws_bridge.py` (53 tests).
   Two of them (`test_bridge_e2e_mode3_orbit_and_frames`,
   `test_bridge_e2e_set_layers_hides_and_shows`) start real
   `rendering_node` / `visualization_node` / `vcam_ws_bridge.py` processes
   and drive them by node name over ROS 2 — this stage pins them to an
   isolated `ROS_DOMAIN_ID` (default `77`, override with
   `CI_VISUAL_MODE_DOMAIN_ID`) so they can never resolve onto a live rig's
   domain.
5. **Golden suite (GPU-skip)** — some renderer gtests (goldens included)
   `GTEST_SKIP()` with no GPU/EGL and exit 0 either way, so a plain `ctest`
   summary can't tell a skip from a real pass. This stage re-derives OK vs.
   SKIPPED counts from stage 2's own gtest output and reports them as their
   own line — skipped is never folded into "passed". "Golden" here is a
   naming convention: some pixel-comparison tests sit outside it and aren't
   counted in this line (e.g. `MapElements.*`, `Fog.ColorAffectsRenderedOutput*`,
   three of the `ThemeTransition.*` tests — an illustrative list, not a
   complete one) — stage 2's suite-wide ok/skipped line is the complete
   skip check.

It never plays a bag, and it never touches a rig it didn't itself start —
stage 4's two E2E tests start their own rendering_node / visualization_node
/ bridge processes on an isolated ROS_DOMAIN_ID and tear down only the
process group they themselves created (`validate_visual_mode.sh`'s own
`--live` lesson: never a process the script didn't start).

**This gate requires a GPU/EGL-capable box.** It is not GPU-optional: without
a GPU/EGL, the gate FAILS at stage 2 (`visual_renderer`'s
`ProjectToScreen.*` and `RendererQuality.*` tests) and stage 3 (the node's
`test_callouts` / `test_hud_overlay` / `test_point_cloud_adapter` golden
tests) — those deliberately assert `create_renderer()` succeeds on this box
(see the header comment of `tests/test_renderer_quality_presets.cpp`) and do
not skip. Stage 5's OK-vs-SKIPPED breakdown exists so a partially-skipping
run on a GPU box can't misread as a full pass; it does not mean a GPU-less
run is expected to reach green.

## What green does not cover

- The node package's six Python integration tests (`test/smoke_test.py`,
  `test_vcam_contract.py`, `test_theme_ws.py`, `test_tf_adapter.py`,
  `test_ego_anchored_vcam.py`, `test_extra_topic_parity.py`) are not
  registered in `micropilot_visualization_node/CMakeLists.txt`, so stage 3's
  `colcon test` never runs them. Run them by hand against a live node.
- Stage 4's WS bridge suite has 2 tests that skip whenever
  `cuda/install/ros_apps` isn't built — the normal state in a worktree — and
  the stage still reports PASS.

## Benchmark (VM-041)

`cuda/src/libs/visual_renderer/tools/viz_benchmark.cpp` (built alongside the
library's other example/tool targets) renders one representative scene
(ribbons + a trajectory carpet + map elements + a point cloud — the same
shapes `tests/test_ribbon.cpp` / `test_trajectory_carpet.cpp` /
`test_map_elements.cpp` / `test_point_cloud.cpp` build) for 120 frames at
each quality preset and prints `render_ms` p50/p99 per preset:

```bash
cd cuda/src/libs/visual_renderer/build && ./viz_benchmark
```

These are the numbers VM-040's quality governor is tuned against; this tool
only measures, it does not tune anything itself. `create_renderer()`
returning null (no GPU/EGL) prints `SKIP (no GPU/EGL)` for that preset
instead of fabricating a number.
