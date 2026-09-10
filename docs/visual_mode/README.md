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
2. **Library ctest suite** — configures + builds `visual_renderer` fresh
   (clang/libc++ toolchain) and runs its full `ctest` suite.
3. **Node gtests** — `colcon build` + `colcon test` for
   `micropilot_visualization_node`. Needs a ROS install and
   `micropilot_rendering_node` already built+installed somewhere sourceable
   read-only (default: this checkout's own `cuda/install/ros_apps`; override
   with `CI_VISUAL_MODE_ROS_APPS_INSTALL=/path/to/setup.bash` when borrowing
   another checkout's install space, e.g. from a worktree). Missing ROS/colcon
   infra **fails this stage loudly** — a pre-merge gate never silently skips
   the node.
4. **WS bridge pytest suite** — `tools/test_vcam_ws_bridge.py` (53 tests).
5. **Golden suite (GPU-skip)** — most renderer gtests (goldens included)
   `GTEST_SKIP()` with no GPU/EGL and exit 0 either way, so a plain `ctest`
   summary can't tell a skip from a real pass. This stage re-derives OK vs.
   SKIPPED counts from stage 2's own gtest output and reports them as their
   own line — skipped is never folded into "passed". A GPU-less box passing
   this stage means every golden skipped, not that it was silently let
   through.

It never plays a bag or touches the live rig (`validate_visual_mode.sh`'s own
`--live` lesson) — every stage above is a build+test invocation only.

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
