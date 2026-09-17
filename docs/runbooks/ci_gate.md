# CI gate (VM-041)

`tools/ci_visual_mode.sh` is the local, GPU-required gate this section
mostly describes — run it before merging any visual-mode change. Hosted CI
(GitHub Actions, added by the open-source restructure's Task 6) is a
separate, narrower set of checks described in "Hosted CI" below; it does
**not** replace this gate.

Run the local gate:

```bash
tools/ci_visual_mode.sh
```

It runs, in order, and labels each stage PASS/FAIL:

1. **POD header check** — `overlume/scripts/check_pod_header.sh`
   (the public `include/overlume/*.h` boundary stays `std::`-free).
2. **Library ctest suite** — configures + builds `overlume`
   incrementally into `overlume/build` (clang/libc++
   toolchain) and runs its full `ctest` suite. Set `CI_VISUAL_MODE_CLEAN=1`
   to wipe that build dir first, so a stale library build can't mask a
   broken clean build — the node stage (3) still builds incrementally.
3. **Node gtests** — `colcon build` + `colcon test` for
   `overlume_ros`. Needs a ROS install and
   `overlume_ros` already built+installed somewhere sourceable
   read-only (default: this checkout's own `ros/install`; override
   with `CI_VISUAL_MODE_ROS_APPS_INSTALL=/path/to/setup.bash` when borrowing
   another checkout's install space, e.g. from a worktree). Missing ROS/colcon
   infra **fails this stage loudly** — a pre-merge gate never silently skips
   the node.
4. **WS bridge pytest suite** — `tools/test_vcam_ws_bridge.py`.
   Some of them (`test_bridge_e2e_mode3_orbit_and_frames`,
   `test_bridge_e2e_set_layers_hides_and_shows`) start real
   `overlume_node` / `vcam_ws_bridge.py` processes
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
stage 4's E2E tests start their own `overlume_node`
/ bridge processes on an isolated ROS_DOMAIN_ID and tear down only the
process group they themselves created (`validate_visual_mode.sh`'s own
`--live` lesson: never a process the script didn't start).

**This gate requires a GPU/EGL-capable box.** It is not GPU-optional: without
a GPU/EGL, the gate FAILS at stage 2 (`overlume`'s
`ProjectToScreen.*` and `RendererQuality.*` tests) and stage 3 (the node's
`test_callouts` / `test_hud_overlay` / `test_point_cloud_adapter` golden
tests) — those deliberately assert `create_renderer()` succeeds on this box
(see the header comment of `tests/test_renderer_quality_presets.cpp`) and do
not skip. Stage 5's OK-vs-SKIPPED breakdown exists so a partially-skipping
run on a GPU box can't misread as a full pass; it does not mean a GPU-less
run is expected to reach green.

## Hosted CI

`.github/workflows/` runs on GitHub, on every push/PR (plus release tags
and pushes to `main`):

- **`lint.yml`** — clang-format check (`tools/check_format.sh`), SPDX header
  check (`tools/check_spdx.sh`), shellcheck over `tools/*.sh`,
  `overlume/scripts/*.sh`, `overlume/tools/*.sh`, `ros/colcon_build.sh` and
  `ros/src/overlume_ros/scripts/*.sh`, a Python syntax check (`py_compile`)
  over `tools/*.py` and the node's Python tests, `pytest tools/` (the
  ROS/rclpy-, websockets- and `gi`-dependent cases skip themselves via
  `importorskip`/`skipif` when those aren't present on the runner), and the
  docs-link checker (`tools/check_docs_links.py`).
- **`build.yml`** — installs the `libegl1-mesa-dev`/`libgles2-mesa-dev` headers every
  test binary needs at link time (`-lEGL`, pulled in transitively via
  Filament), bootstraps the toolchain, configures with
  `-DOVERLUME_ENABLE_CESIUM=OFF` (no token needed), builds the library and
  examples, and runs `ctest` restricted to non-GPU tests. `cpu`/`gpu`
  ctest LABELs now exist in `overlume/CMakeLists.txt` (`check_pod_header`,
  `filament_link_probe`, `cesium_link_probe`/`cesium_link_probe_symbol_hygiene`,
  and every `gtest_discover_tests` binary except `test_renderer_projection`
  and `test_renderer_quality_presets`, which are labeled `gpu`), so the step
  selects `ctest -L cpu`; the `-LE gpu` branch is kept only as a safety
  fallback in case the labels ever go missing, not the path taken today. The
  two `gpu`-labeled binaries are excluded because they deliberately assert
  `create_renderer()` succeeds and **hard-fail** without a GPU/EGL — they do
  not self-skip. The step separately verifies via `ctest -N`'s test count
  (not its exit code, which is 0 either way) that whichever selector runs
  actually matched at least one test, so a labeling mismatch fails loudly
  instead of reporting a silent green.
- **`docs.yml`** — builds the Doxygen API reference on push and PR (the
  Pages deploy step itself only runs on push to `main`) and publishes it to
  GitHub Pages.
- **`release.yml`** — on a `v*` tag push, verifies the tag matches
  `overlume/include/overlume/version.h` and creates a GitHub release from
  the matching `CHANGELOG.md` section.

All four workflows pin `runs-on: ubuntu-22.04`; this is unverified against
GitHub's runner-image lifecycle until the first hosted run, and it's not a
one-word bump if that image is retired — `overlume/scripts/setup_toolchain_cesium.sh`
hardcodes an `apt.llvm.org/jammy` pool URL, so moving to `ubuntu-24.04`
would need a matching `noble` pool in that script too.

`build.yml`'s `libegl1-mesa-dev`/`libgles2-mesa-dev` install step exists because every
ctest binary links `-lEGL` explicitly and pulls `libGLESv2`/`libGLdispatch`
transitively via Filament — confirmed locally via `ldd` on
`overlume/build/test_theme` and `overlume/build/examples/01_hello_frame`.
A bare `ubuntu-22.04` runner image is not guaranteed to carry those dev
symlinks; without them the "Build library + examples" step would fail with
`cannot find -lEGL`. Like the rest of hosted CI, this is unverified until
the first push actually runs it.

**What hosted CI does *not* cover** — the reason this local gate still
exists and still gates every merge:

- No GPU/EGL runner. `build.yml` configures with Cesium off and skips every
  GPU-labeled test — that's most of what actually renders a frame. A green
  `build.yml` proves the library *compiles*, not that it *renders
  correctly*.
- No golden/pixel-comparison coverage at all (those tests need a GPU).
- No ROS 2 node build or test (`colcon build`/`colcon test`) — hosted CI
  only builds `overlume` on its own. `lint.yml`'s `pytest tools/` step
  exercises the operator-tool Python unit tests, not the ROS node.
- No WS bridge end-to-end tests (stage 4 of the local gate).
- No live-token, live-network checks (Cesium ion, Mapbox) — by design, per
  this repo's token rules; nothing in hosted CI needs a token.

A green hosted-CI run is a fast, cheap sanity check (does it compile, is it
formatted, are the docs/links intact) — it is not a substitute for running
`tools/ci_visual_mode.sh` on a real GPU box before merging a rendering
change.


**Repository settings the hosted CI relies on (set once, user-side):** GitHub
Pages source must be *GitHub Actions* (enabled 2026-09-17; the site is
<https://amerghazal7.github.io/overlume/>) — `actions/deploy-pages` has no configure step and fails otherwise;
Private Vulnerability Reporting must be on for `SECURITY.md`'s route to exist
(enabled 2026-09-17). The toolchain bootstrap in `build.yml`/`docs.yml` fetches
exact pinned `.deb`s from the apt.llvm.org jammy pool, which upstream prunes as
versions supersede; when that happens both workflows fail at *Bootstrap
toolchain* and the fix is re-pinning `LLVM_PKG_VERSION` in
`overlume/scripts/setup_toolchain_cesium.sh` (the `actions/cache` steps carry
`restore-keys` so a warm cache survives a prune where possible).

## What green does not cover

- The node package's Python integration tests not registered in
  `overlume_ros/CMakeLists.txt` (e.g. `test/smoke_test.py`,
  `test_vcam_contract.py`, `test_theme_ws.py`, `test_tf_adapter.py`,
  `test_ego_anchored_vcam.py`, `test_extra_topic_parity.py`) never run under
  stage 3's `colcon test`. Run them by hand against a live node.
- Stage 4's WS bridge suite has tests that skip whenever
  `ros/install` isn't built — the normal state in a worktree — and
  the stage still reports PASS.

## Benchmark (VM-041)

`overlume/tools/viz_benchmark.cpp` (built alongside the
library's other example/tool targets) renders one representative scene
(ribbons + a trajectory carpet + map elements + a point cloud — the same
shapes `tests/test_ribbon.cpp` / `test_trajectory_carpet.cpp` /
`test_map_elements.cpp` / `test_point_cloud.cpp` build) for 120 frames at
each quality preset and prints `render_ms` p50/p99 per preset:

```bash
cd overlume/build && ./viz_benchmark
```

These are the numbers VM-040's quality governor is tuned against; this tool
only measures, it does not tune anything itself. `create_renderer()`
returning null (no GPU/EGL) prints `SKIP (no GPU/EGL)` for that preset
instead of fabricating a number.
