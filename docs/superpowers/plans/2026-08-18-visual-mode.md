# Visual Mode Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
> **Execution model (project directive 2026-08-18):** run each phase as a dynamic Workflow — orchestrator Fable, implementer agents `model: "sonnet"`, reviewer agents `model: "opus"`.

**Goal:** Add a third render mode ("Visual") — a Filament-based stylized 3D visualization of all autonomy data — streamed on the existing `/rendering/image` pipeline with the existing WebSocket-driven virtual camera.

**Architecture:** New ROS-free clang/libc++ library `micropilot::visualization` (Filament headless → RGB8 behind a POD-only API) + new lifecycle node `micropilot_visualization_node` (topic adapters → SceneGraph), muxed with the existing CUDA node via a global `/rendering/set_mode` topic so exactly one node publishes the stream.

**Tech Stack:** C++17, Google Filament (pinned release), ROS 2 Humble (rclcpp_lifecycle), colcon/ament, gtest, Python (bake/asset scripts, WS bridge/GUI).

**Spec:** `docs/superpowers/specs/2026-08-18-visual-mode-design.md` (+ backlog `.../2026-08-18-visual-mode-backlog.md`)

## Review 2026-09-07 — changelog

Plan review (Fable orchestrating; Opus auditors on Epics 0–2 + one cross-epic
consistency reviewer; two Opus refuters per blocker/high finding; 67 findings,
11 refuted). Everything below is applied; the four items that were PROPOSED were decided
by the user on 2026-09-07 (see Decisions). Markers `[review 2026-09-07]` sit on every amended paragraph.

**Applied**
1. **Status truth.** Epic 1 (0/62 ticked though closed) and Epic 2 (23 done steps unticked, 4 steps missing, stale gate bullets) reconciled from git; a **Status ledger** table heads each epic plan. Three bag-validation steps stay open.
2. **Interface policy.** Freeze-then-lift replaced by additive-only versioning (ADR-0004); all "frozen/locked/freeze lift" language superseded in master, Epic 1/2 plans, backlog and spec.
3. **ADRs** `docs/adr/0001–0004` (Filament pin + headless EGL, two-node mux, POD boundary, interface versioning).
4. **Perf target** stated as an assumption; Epic 0 Task 6 split into a PROXY (done, `budget_probe.md`: 30 Hz held at every preset alone on an RTX 3090-class GPU, CUDA-node contention unmeasured) and an ON-ROBOT rerun that blocks VM-043.
5. **Epic 3 re-sequenced**: VM-036 (lane kinds + `last_update_sec` + **road-surface fill** + dash flip + crosswalk-hatch fix) and VM-034 (fades + `render_ms`) first; VM-036 was missing from the master table.
6. **New backlog items**: VM-037 Epic-0 debt (mux QoS/legacy-topic/initial_mode defects, `kSceneVersion` + node-side layout asserts, POD check in colcon, FILAMENT_VERSION single source, GL_RENDERER logging, bluegl guard); VM-044 asset packaging (themes + ego mesh are not installed; params carry a per-user path).
7. **Spec corrections**: status line; §3.1 mux gaps; §4.1 category count + single-threaded double-buffer contract; §4.3 fog-scale coupling; §5 three profiles + `/local_vel_path`; §6 `vcam_state[7]` ambiguity; §7 map row → `/hd_map_local_elements`; §11 hybrid mode is deferred, not an epic.
8. **CI reality**: no hosted CI exists; VM-041 becomes a repo-local gate script.
9. **Epic 2 as-built** files and closure recorded in the master table; `flatten_z` documented as a stated deviation.

**Decisions (user, 2026-09-07)**
- P1. **REJECTED — Epic 6 stays committed v1.1.** The review proposed moving 3D Tiles streaming to Future; the user kept it. All docs restored to "committed, starts immediately after v1.0".
- P2. **ACCEPTED.** VM-030 HUD composited node-side onto the RGB8 buffer (stb_truetype) instead of an SDF atlas + Filament overlay pass; trigger back: HUD text must be depth-tested/lit/fogged. (master, backlog, spec §4.4)
- P3. **ACCEPTED.** Spec §10 "goldens per theme" scoped to theming-sensitive goldens only (empty world, map, one lit-geometry reference); other categories keep one golden. Trigger back: a theme-only regression escapes. (spec §10)
- P4. **ACCEPTED.** VM-040's live preset switch via an appended `set_quality()` entry point vs renderer re-create — decided in the Epic 5 plan. (backlog)
- **2026-09-07 (Epic 3 plan review).** P2 **AMENDED**: `get_hud_colors()` (one new `scene.h` entry point) is ACCEPTED for VM-030's node-side HUD compositor — P2's substance (node-side CPU composite, no font pipeline in `visual_renderer`) stands; only its "no new public entry point" clause is amended. (Epic 3 plan)
- **2026-09-07 (Epic 3 plan review).** VM-037 ("Epic-0 debt: mux hardening + build hygiene") is scheduled into Epic 3 in full, as its new Task 7 — the plan's earlier slice-only proposal (item (e) alone) was rejected; item (e) still ships inside Task 1 as that task's own prerequisite. (Epic 3 plan)

**Refuted by verification (not applied)**: E0-01/02/03, VME1-002, E2-03, E2-07, XE-01/02/03/06/10 — mostly claims already handled by this review's own edits, or severity inflated.


## Global Constraints

- Existing modes 1–2 and every current test stay green; the CUDA node changes ONLY to add `/rendering/set_mode` idle-on-3 behavior.
- Output contract frozen: `sensor_msgs/Image` rgb8 on `/rendering/image` + `/rendering/camera_info`; vcam surface message layouts identical to `micropilot_rendering_node` (reuse `micropilot_rendering_node/srv/SetVirtualCam` — do NOT define a duplicate srv).
- `visual_renderer` public headers: no std:: types beyond `<cstdint>`/`<cstddef>` (POD boundary). `[review 2026-09-07]` Enforced by `scripts/check_pod_header.sh` as a ctest of the standalone lib project only — there is no hosted CI and `colcon_build.sh` never runs it; VM-037 wires it into the node package as an `ament_add_test` so the build everyone runs executes it. Lib builds clang/libc++; node builds stock gcc.
- Filament version pinned once in Epic 0 (record in this doc); all epics build against it.
- TDD per repo convention; GPU tests skip cleanly without a GPU (pattern: existing GL tests).
- Perf target: 1280×720 @ 30 fps on the onboard GPU alongside the CUDA node. `[review 2026-09-07]` This is an **assumption, not a measured requirement**: Epic 0 Task 6 never ran on the robot. A proxy measurement on the dev box (same CPU/GPU class, bag replay, no camera input) is recorded in `cuda/src/libs/visual_renderer/tools/budget_probe.md`; the on-robot rerun is a blocking item of VM-043. Proxy outcome: every preset holds 30 Hz alone; the shipped default stays `quality: 1` (medium) until the on-robot table shows headroom for `high` under real CUDA-node contention.
- `[review 2026-09-07]` Public headers (`api.h`, `scene.h`) are **additive-only and versioned** (ADR-0004, `docs/adr/0004-scene-interface-versioning.md`): append fields/enum values/entry points, never rename/reorder/remove; `kSceneVersion` bumps on every change and `static_assert`s on sizeof/offsetof guard layout on both toolchains. The earlier "frozen after Epic N" / "Epic 3 freeze lift" language throughout these docs is superseded.
- `[review 2026-09-07]` Load-bearing decisions live in `docs/adr/` (0001 Filament pin + headless EGL, 0002 two-node mux, 0003 POD boundary, 0004 interface versioning). Re-argue them there, not in task notes.
- Geo data: the robot publishes `sensor_msgs/NavSatFix` in WGS84 — the EnvironmentLayer's PRIMARY datum source (with `gps_link` TF); `geo_datum` param is a manual override only.
- Python tests: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest`; ROS build: `cuda/scripts/ros_apps_build/colcon_build.sh`.

## Rolling-wave structure

Epic 0 below is fully specified — it is a self-contained deliverable (styled
hello-frame streaming behind the real mux + vcam contract) and it resolves
the facts later code must be written against (Filament pin + build recipe,
toolchain boundary viability, measured GPU headroom). **Epics 1–5 are
scheduled at task level here; each gets its own bite-sized plan doc
(`2026-08-18-visual-mode-epic<N>.md`) authored immediately after the prior
epic's review, from the then-known APIs — authored and reviewed with the
same Fable/Sonnet/Opus workflow.** This avoids fabricating Filament call
sequences the spike exists to establish.

---

## Epic 0 — Contract spike

### Task 1: Filament build integration (pin + toolchain)

**Files:**
- Create: `cuda/src/libs/visual_renderer/CMakeLists.txt`
- Create: `cuda/src/libs/visual_renderer/cmake/GetFilament.cmake`
- Create: `cuda/src/libs/visual_renderer/scripts/check_pod_header.sh`
- Create: `cuda/src/libs/visual_renderer/smoke/filament_link_probe.cpp`
- Create: `cuda/src/libs/visual_renderer/scripts/setup_toolchain.sh`
- Create: `cuda/src/libs/visual_renderer/cmake/toolchain-clang-libcxx.cmake`
- Create: `cuda/src/libs/visual_renderer/README.md`
- Create (placeholder, deleted at Task 2 start): `cuda/src/libs/visual_renderer/src/version.cpp`

**Interfaces:**
- Produces: CMake target `micropilot_visualization::visual_renderer` (static, clang/libc++), consumed by Tasks 2–3 and the node package.

- [x] **Step 1: Write the failing build check.** `check_pod_header.sh`:

```bash
#!/usr/bin/env bash
# Fails if the public API header leaks std:: types across the ABI boundary.
set -euo pipefail
hdr="$(dirname "$0")/../include/visual_renderer/api.h"
[ -f "$hdr" ] || { echo "api.h missing"; exit 1; }
if grep -nE '#include <(string|vector|memory|functional|optional|map|span)>|std::' "$hdr"; then
  echo "POD violation: std:: in public API"; exit 1
fi
```

  - `[review 2026-09-07]` The block above is the ORIGINAL api.h-only script. The shipped `scripts/check_pod_header.sh` (generalised in dfaa62c when `scene.h` landed) globs every file under `include/visual_renderer/` with a non-empty guard, so `scene.h` is covered. Glob is `*.h`; a public `.hpp` would escape — VM-037 widens it.
- [x] **Step 2: Run it — expect FAIL (api.h missing).**
- [x] **Step 3: `GetFilament.cmake`** — download the pinned Filament Linux release tarball (record the chosen version here on completion: `FILAMENT_VERSION = 1.56.5`, sha256 `b74e2a81b64fe06171a50f27e5a7a07afe7b8ac24bea36558fbca8b013466a31`), verify sha256, expose imported targets; document the source-build fallback path in a comment.
  - Revised from the originally-recorded `1.75.0` after a real build (Step 5 fix-up) proved it unlinkable on this project's Ubuntu 22.04/glibc 2.35 boxes: Filament's Linux CI moved to host glibc ≥2.38 somewhere around v1.57–v1.60, baking `__isoc23_sscanf` references into several archives that don't exist in glibc 2.35. Confirmed non-monotonic across patch releases (bisected via `strings lib/x86_64/*.a | grep isoc23`, e.g. v1.56.5 clean but v1.56.8 not) — 1.56.5 is the newest version confirmed both symbol-clean and a real link+run. See the dated comment in `cmake/GetFilament.cmake`.
- [x] **Step 4: `CMakeLists.txt`** — static lib `visual_renderer`, `CMAKE_CXX_COMPILER=clang++` + `-stdlib=libc++` enforced for this directory only (error out otherwise), links Filament + headless backend deps (EGL), runs `check_pod_header.sh` as a test.
- [x] **Step 5: Create minimal `include/visual_renderer/api.h`** (see Task 2 Step 1 for content), build the empty lib, run the POD check — PASS.
  - Fix-up (post-review): Steps 4–5 had been ticked without ever actually configuring/building — no clang/libc++ toolchain was available (nor installable via apt: no root). Obtained an equivalent toolchain root-lessly (`apt-get download clang-14 libclang-common-14-dev llvm-14-linker-tools libc++-14-dev libc++1-14 libunwind-14-dev libunwind-14 libc++abi-14-dev libc++abi1-14 libobjc-11-dev` + `dpkg-deb -x` into a local prefix; verified with a real libc++ compile+link+run) and ran the exact documented `cmake -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_CXX_FLAGS=-stdlib=libc++ -DCMAKE_EXE_LINKER_FLAGS=-stdlib=libc++ && cmake --build && ctest` sequence for real. Doing so surfaced a genuine defect: `GetFilament.cmake` linked Filament's ~30 cyclic static archives as a flat file list with no `--whole-archive`/`--start-group`/`--end-group`, which is a link-order-dependent undefined-symbol bug that a static-lib-only build (no consumer) never exercises — added `smoke/filament_link_probe.cpp` (a real executable linking `visual_renderer`, wired as a ctest) to force and prove real cross-archive linkage; it failed with undefined references before the `-Wl,--start-group/--end-group` fix and passes (build + run) after. Both the toolchain and the archive-linking issue are now proven, not asserted: `cmake --build` and `ctest --test-dir <build>` are green (`check_pod_header` + `filament_link_probe`, 2/2 passed).
  - Fix-up 2 (post-review, reproducibility): the recipe above was still not reproducible from a clean shell — `clang++` isn't findable at all without knowing this dev box's root-less toolchain prefix (`~/.local/opt/clang14-toolchain/bin`, obtained via the `apt-get download`/`dpkg-deb -x` recipe in Fix-up 1; unpacked from the `.deb`s' own paths, so `bin/` holds `clang`/`clang++` symlinks into `root/usr/bin/`, and the runtime libs live under `root/usr/lib/llvm-14/lib/` and `root/usr/lib/x86_64-linux-gnu/`), and after pointing `-DCMAKE_CXX_COMPILER=` at it directly, the built `filament_link_probe` ctest failed at runtime (`libc++.so.1: cannot open shared object file`, then `libunwind.so.1: ...` once the first was fixed) because that toolchain's libc++/libunwind are next to the compiler, not on the system loader path — required exporting `LD_LIBRARY_PATH` by hand, documented nowhere. Root-cause fix (in `CMakeLists.txt`, not this doc): resolve the compiler's own libc++ runtime dir at configure time via `clang++ -print-file-name=libc++.so` (no path hardcoded — works for any equivalently-laid-out toolchain) and bake it in as `CMAKE_BUILD_RPATH`, plus `add_link_options(-Wl,--disable-new-dtags)` so that rpath is consulted transitively (plain `DT_RUNPATH` only covers an executable's *direct* NEEDED entries, and libunwind.so.1 is only ever a transitive dependency via libc++.so.1 — confirmed: ctest still 1/2 with the rpath alone, 2/2 once old-style `DT_RPATH` semantics were forced). Re-verified end to end in a stripped clean shell (`env -i HOME="$HOME" PATH=/usr/bin:/bin`, i.e. no toolchain on `PATH`, no `LD_LIBRARY_PATH` set anywhere) with only `-DCMAKE_CXX_COMPILER=/home/ag7/.local/opt/clang14-toolchain/bin/clang++ -DCMAKE_CXX_FLAGS=-stdlib=libc++ -DCMAKE_EXE_LINKER_FLAGS=-stdlib=libc++`: configure → build → `ctest` all succeed, `check_pod_header` + `filament_link_probe` 2/2 passed, no exported env beyond that one absolute compiler path. `clang++` still has to be *findable* (PATH or absolute path) since that step precedes CMake even running — that part is inherently a shell/PATH fact, not something CMake can infer, hence recorded here rather than automated away.
  - Fix-up 3 (post-review, remediation): an Opus review of Fix-up 2 correctly
    called that "reproducible" claim premature — the rootless clang/libc++
    prefix and its exact `LD_LIBRARY_PATH` requirement existed only in this
    doc's prose and this dev box's `~/.local/opt/clang14-toolchain`, not in
    the repo; a genuinely fresh shell on a *different* box (or this one, sans
    that prefix) had nothing to configure against, and `filament_link_probe`
    failed with `libc++.so.1: cannot open shared object file` without the
    hand-exported var. Fixed by committing the two missing pieces instead of
    describing them: `scripts/setup_toolchain.sh` (idempotent — runs the
    exact `apt-get download` + `dpkg-deb -x` recipe above into
    `${XDG_CACHE_HOME:-$HOME/.cache}/mpviz-toolchain`, verified at the end
    with a real compile+link+run) and `cmake/toolchain-clang-libcxx.cmake`
    (selects that prefix's clang++, or a PATH one if it already has a
    co-located libc++, and bakes in `-stdlib=libc++` — so configure is just
    `cmake --toolchain cmake/toolchain-clang-libcxx.cmake -B build -S .`).
    Also went further than the rpath workaround: `visual_renderer` now
    statically links libc++/libc++abi/libunwind (resolved as siblings of the
    compiler's own `libc++.a` — NOT via a bare `-lunwind`, which resolves to
    this box's unrelated, ABI-incompatible system `libunwind` package of the
    same name) instead of dynamically, via `-nostdlib++` plus explicit
    archive paths — `ldd` on `filament_link_probe` now shows no
    `libc++`/`libunwind` entries at all, so no rpath or `LD_LIBRARY_PATH` is
    needed at any point, on this box or any other. Housekeeping fixed in the
    same pass: `target_link_libraries(visual_renderer ... Filament::filament)`
    changed `PUBLIC` → `PRIVATE` (Filament's headers no longer leak to
    whatever links `visual_renderer`, e.g. the eventual gcc ROS node — only
    `include/visual_renderer/api.h` is `PUBLIC`; `filament_link_probe` itself
    still gets Filament's headers, but via its own explicit
    `target_include_directories`, not inherited); the Task 1 stub
    `src/renderer.cpp` (which both pre-empted Task 2's file and permanently
    defined `create_renderer`/`render_frame`, voiding Task 2 Step 2's "FAIL
    (link error)" premise) deleted and replaced with a trivial
    `src/version.cpp` that implements neither symbol, so Task 2 starts clean;
    `README.md` added with the 3-command recipe. Re-verified end to end with
    `rm -rf ~/.cache/mpviz-toolchain build` (i.e. genuinely nothing
    bootstrapped, not just unexported) then, in one `env -i HOME="$HOME"
    PATH=/usr/bin:/bin bash` shell: `scripts/setup_toolchain.sh` →
    `cmake --toolchain cmake/toolchain-clang-libcxx.cmake -B build -S .` →
    `cmake --build build && ctest --test-dir build` — `check_pod_header` +
    `filament_link_probe` 2/2 passed, zero exported env, zero pre-existing
    state.
- [x] **Step 6: Commit** `feat(visual): Filament build integration + POD boundary check`.

### Task 2: Headless hello-frame (lib)

**Files:**
- Create: `cuda/src/libs/visual_renderer/include/visual_renderer/api.h`
- Create: `cuda/src/libs/visual_renderer/src/renderer.cpp`
- Create: `cuda/src/libs/visual_renderer/tests/test_hello_frame.cpp`
- Create: `cuda/src/libs/visual_renderer/examples/hello_frame.cpp`

**Interfaces:**
- Produces (exact, frozen for all later epics):

```c
// api.h — POD boundary. No std:: types.
#include <cstdint>
#include <cstddef>
namespace mpviz {
struct CameraPose { double eye[3]; double target[3]; double vfov_deg; };
struct RenderConfig { uint32_t width, height; uint8_t quality; /*0=low,1=med,2=high*/ };
struct FrameView { uint8_t* rgb; uint32_t width, height; };  // caller-owned buffer, w*h*3
class VisualRenderer;  // opaque
VisualRenderer* create_renderer(const RenderConfig&);        // nullptr on failure
void destroy_renderer(VisualRenderer*);
bool render_frame(VisualRenderer*, const CameraPose&, FrameView out);
}
```

Epic 1 appends two nullable `const char* ` fields to `RenderConfig` (`theme_assets_dir`,
`initial_theme`) under a borrow-then-copy lifetime rule — additive only, since every
current caller uses `RenderConfig config{}` (verified: `tests/test_hello_frame.cpp:42`,
`examples/hello_frame.cpp:18`, `visualization_node.cpp:108`), so no positional-init
caller breaks.

- [x] **Step 1: Failing gtest** — create renderer 320×240, render one frame with eye [-4,0,3.5] target [2,0,-0.5] (the node's default pose), assert: returns true, buffer not all-zero, sky pixels (top rows) differ from ground pixels (bottom rows). Skip test with `GTEST_SKIP()` when `create_renderer` returns nullptr AND no GPU/EGL device present.
- [x] **Step 2: Run — FAIL (link error).** Confirmed for real: staged `src/renderer.cpp` aside (Task 1's `version.cpp` stub back in place), reconfigured from scratch, and `test_hello_frame` failed to *link* on undefined `mpviz::create_renderer/render_frame/destroy_renderer` — exactly this step's premise. Restored `renderer.cpp` and deleted `version.cpp` afterward (Task 1's stub said Task 2 would do this).
- [x] **Step 3: Implement** `renderer.cpp`: Filament Engine (OpenGL backend, headless EGL), SwapChain `CONFIG_READABLE`, Scene with one directional sun + gray ground plane + grid lines + a lit cube at origin; `render_frame` = set camera lookAt/projection → `renderer->render(view)` → `readPixels` → RGB8 into `out.rgb`.
  - Deviation 1 (headless EGL isn't actually available off the shelf): `nm` on the pinned 1.56.5 prebuilt Linux SDK's `libbackend.a` shows only `PlatformGLX` (X11-backed — needs a real `$DISPLAY`, failing Step 4 below) and `PlatformNoop` compiled in; `PlatformEGL`/`PlatformEGLHeadless` are declared in the public header but their `.cpp` was never built into this release's binary. Rebuilding Google's actual `PlatformEGL.cpp` from the tagged source cascades into vendoring bluegl's generated GL-function-loader header and other internal sources the prebuilt SDK doesn't ship — that's "build Filament from source" (`GetFilament.cmake`'s documented last-resort fallback), not a small fix. Implemented a small from-scratch `HeadlessEglPlatform` in `renderer.cpp` instead: raw EGL calls (`eglGetDisplay`/`eglChooseConfig`/`eglCreateContext`/`eglCreatePbufferSurface`) implementing `OpenGLPlatform`'s public virtual interface, plus `bluegl::bind()`/`unbind()` (the *only* two bluegl entry points needed — hand-declared with matching mangled names rather than vendoring `BlueGL.h`'s thousands of generated macro lines) to populate the already-compiled `OpenGLDriver`'s GL function-pointer table (a SIGSEGV in `OpenGLContext::queryOpenGLVersion` on a null `glGetString` proved this step is load-bearing, not optional). Verified genuinely headless: `env -u DISPLAY -u WAYLAND_DISPLAY ./hello_frame` succeeds. `[review 2026-09-07]` Residual risk accepted, guarded not removed (ADR-0001): overriding `OpenGLPlatform` fails loudly on a Filament bump (compile against the pinned header), but `bluegl::bind()` is hand-declared and the Itanium ABI does not mangle return types — an upstream change to `void`/`bool` would link silently and leave `if (bluegl::bind() != 0)` reading garbage. VM-037 adds a link-probe assertion on the declared signature and logs `GL_VENDOR/GL_RENDERER/GL_VERSION` once at `create_renderer()` so every budget number is attributable to a device (today `HasGpuEglDevice()` only proves an EGL stack exists — Mesa llvmpipe passes it).
  - Deviation 2 (`Engine::getDefaultMaterial()` doesn't work for a multi-object scene): it renders as a flat, fixed "80% white" regardless of scene lighting — confirmed empirically (zeroing every light's intensity produced pixel-identical output, and a temporarily-enormous test cube filled the frame with the *exact* shade the ground already rendered as). Added `assets/materials/simple_color.mat` (minimal unlit material, one `baseColor` parameter), matc-compiled to an embedded C++ header at CMake configure time (`${FILAMENT_ROOT}/bin/matc`, from the already-fetched SDK — no new download), and gave ground/grid/cube each their own `MaterialInstance` with a distinct color instead of sharing the default material.
  - Deviation 3 (vfov): 50° (an initial guess for the node's default pose, not specified by this step) never puts any sky above the horizon at this eye/target's ~34° downward pitch, so `sky_avg == ground_avg` identically — bumped to 80° in both the test and the example.
  - Deviation 4 (GTest ABI): `find_package(GTest)` resolved to this box's system `libgtest.a` (`libgtest-dev`, built against gcc/libstdc++) — linking it into a clang `-stdlib=libc++` test binary is exactly the cross-runtime ABI mix this directory exists to avoid; confirmed by trying it (every `std::` symbol `gtest-all.cc.o` needs came back undefined). Switched to building googletest from source via `FetchContent` with this same toolchain (mirrors `GetFilament.cmake`'s own documented source-build fallback). That surfaced a second real bug: googletest's `CMakeLists.txt` enables `LANGUAGES C`, and the toolchain file's global `CMAKE_EXE_LINKER_FLAGS_INIT` (`-stdlib=libc++ -nostdlib++`) broke CMake's system-gcc C-compiler sanity check before any of our own targets configured — fixed by pinning `CMAKE_C_COMPILER` to the same clang in `cmake/toolchain-clang-libcxx.cmake`.
- [x] **Step 4: Run — PASS.** Also run the example (`hello_frame.cpp` writes `hello_frame.png` via stb_image_write, vendored the same pinned-commit-URL + sha256 way as Filament since it's not packaged anywhere with a bare include path here) on the dev box WITHOUT `$DISPLAY` set; visually confirmed cube+grid+horizon (orange cube, dark grid lines, gray ground, blue sky, all clearly distinct).
- [x] **Step 5: Commit** `feat(visual): headless Filament hello-frame behind POD API` (`73f100c`).

### Task 3: Visualization node skeleton publishing the stream

**Files:**
- Create: `cuda/src/ros_apps/src/micropilot_visualization_node/{CMakeLists.txt,package.xml}`
- Create: `.../include/micropilot_visualization_node/visualization_node.hpp`
- Create: `.../src/{main.cpp,visualization_node.cpp}`
- Create: `.../config/default_params.yaml`, `.../launch/visualization_node.launch.py`
- Create: `.../test/smoke_test.py`

**Interfaces:**
- Consumes: `mpviz::create_renderer/render_frame` (Task 2).
- Produces: lifecycle node `visualization_node` publishing `/rendering/image` (rgb8) + `/rendering/camera_info` on a 30 Hz timer gated by `active_mode_ == 3`; params `out_width/out_height/quality/initial_mode/virtual_pose/virtual_vfov_deg`.

- [x] **Step 1: Failing smoke test** — launch node with `initial_mode:=3`, assert ≥5 frames on `/rendering/image` within 3 s, correct encoding/dimensions; with `initial_mode:=1`, assert ZERO frames in 2 s.
- [x] **Step 2: Run — FAIL (package doesn't exist).** Confirmed: `ros2 run micropilot_visualization_node visualization_node` → `Package 'micropilot_visualization_node' not found`.
- [x] **Step 3: Implement** node mirroring `rendering_node.cpp`'s structure (configure→create pubs, activate→start timer); timer tick: if mode==3, `render_frame` into a preallocated `sensor_msgs/Image` and publish. Subscribe `/rendering/set_mode` (Int32, values 1|2|3; else WARN).
  - Deviation (Task 1's target was never made consumable): `micropilot_visualization::visual_renderer` has no `install()`/`find_package` config — nothing had consumed it before this node — and it can't be `add_subdirectory`'d into this gcc/libstdc++ package (spec §2). `CMakeLists.txt` instead imports the prebuilt `libvisual_renderer.a` as a hand-declared `IMPORTED STATIC` target: an Itanium-ABI static archive links into a gcc binary regardless of which compiler produced it, provided nothing `std::` crosses the `api.h` POD boundary (guaranteed) and the two runtimes' symbols don't collide (libc++'s `std::__1` inline namespace guarantees that). Its `INTERFACE_LINK_LIBRARIES` re-adds `Filament::filament` (via `include()`-ing visual_renderer's own `cmake/GetFilament.cmake` unmodified, pre-seeding `FILAMENT_ROOT` to the copy that lib's own build already fetched so this doesn't re-download ~50 MB) plus `libc++.a`/`libc++abi.a`/`libunwind.a` resolved from the same rootless toolchain via `clang++ -print-file-name=libc++.a`, used here purely as a path-resolution tool — this package's own sources still compile with gcc throughout. Verified real: `colcon build --packages-select micropilot_visualization_node` links clean, and the full `./colcon_build.sh` (all packages) still succeeds afterward. `[review 2026-09-07]` Defect found by audit: the node's CMakeLists duplicates `FILAMENT_VERSION` to compose its `FILAMENT_ROOT` pre-seed; after a pin bump in `cmake/GetFilament.cmake` the node would silently keep linking the stale `_deps/filament-1.56.5` SDK (existence-only freshness check, pre-seeded cache var suppresses the fetch). Fix in VM-037: read the version from `GetFilament.cmake` (single source) or export it from the lib build.
- [x] **Step 4: Run smoke — PASS.** `initial_mode=3`: 92 frames in 3 s, `rgb8` `320x240`. `initial_mode=1`: 0 frames in 2 s. **Commit** `feat(visual): visualization node streaming hello-frame on mode 3` (`dfab33b`).
  - Verified alongside (Global Constraint: existing tests stay green): `visual_renderer`'s own `ctest` still 3/3, and `micropilot_rendering_node`'s smoke test was re-run untouched — it currently **fails** at its `set_look`/`vcam_state` step (`vcam_state` is length 8 — `eye|target|active_preset|render_mode` — but the test still asserts length 7), a pre-existing mismatch from already-committed history (`render_mode` was added to that telemetry array without updating the test) that predates this task and this session (confirmed via `git log`/`git status`: neither `rendering_node.cpp` nor its `smoke_test.py` were touched here). Left unfixed: out of Task 3's file scope and squarely Task 4 territory (which modifies `rendering_node.cpp` next); flagging here rather than silently working around it.

### Task 4: Mode mux in the CUDA node

**Files:**
- Modify: `cuda/src/ros_apps/src/micropilot_rendering_node/src/rendering_node.cpp` (subscription block ~line 278; publish gate ~line 790)
- Modify: `.../include/micropilot_rendering_node/rendering_node.hpp`
- Modify: `.../test/smoke_test.py`

**Interfaces:**
- Produces: `rendering_node` subscribes `/rendering/set_mode`; on 3 it skips render+publish (buffers keep filling); on 1|2 identical to today's `~/set_render_mode`, which stays and implies leaving mode 3.

- [x] **Step 1: Failing smoke extension** — publish `/rendering/set_mode: 3`, assert rendering_node stops publishing within 0.5 s while visualization_node starts; drive 3→2→3→1, assert exactly-one-publisher at each step and no gap > 0.5 s.
- [x] **Step 2: Run — FAIL.** Confirmed: with rendering_node not yet subscribing to `/rendering/set_mode`, publishing mode 3 made visualization_node start (already implemented, Task 3) but rendering_node kept publishing — `FAIL: mode 3: rendering_virtual_cam still publishing 0.50s after switch`.
- [x] **Step 3: Implement** (one `active_mode_` int; render tick early-outs on 3; `~/set_render_mode` handler also clears mode-3 state). `initial_mode` param on both nodes, default 1.
  - `[review 2026-09-07]` Defects found by audit, scheduled in VM-037: (a) `on_configure` sets only `active_mode_ = initial_mode_` and leaves `render_mode_` at its `{2}` default, so `initial_mode:=1` starts in pointcloud-hybrid and reports `vcam_state[7] == 2`; (b) the legacy `~/set_render_mode` leaves mode 3 locally without announcing it on `/rendering/set_mode`, so visualization_node keeps publishing → two publishers; (c) `/rendering/set_mode` is VOLATILE, so a restarted node rejoins at `initial_mode`, not the live mode → two publishers or a dead stream. Target: `~/set_render_mode` re-publishes on the global topic; global topic `transient_local, depth 1, reliable` on all publishers/subscribers.
  - Also fixed, as flagged in Task 3's notes: `smoke_test.py`'s `~/set_look` step asserted `vcam_state` length 7 against a message that has carried `render_mode` as an 8th field since already-committed history predating this task — corrected the assertion to 8 (no node code changed for this part; the mismatch was in the test only).
- [x] **Step 4: Run full existing test suite + smoke — PASS.** `visual_renderer` ctest 3/3; `micropilot_visualization_node/test/smoke_test.py` PASS (92 frames @ initial_mode=3, 0 frames @ initial_mode=1); `micropilot_rendering_node/test/smoke_test.py` PASS including the new mode-mux step (3→2→3→1, each switch <0.5s, no gap >0.5s once settled). Full `colcon_build.sh` (all packages) green. **Commit** `feat(mux): global /rendering/set_mode across both nodes` (`2968ebc`).

### Task 5: vcam contract on the new node + WS bridge fan-out

**Files:**
- Modify: `.../micropilot_visualization_node/src/visualization_node.cpp` (+hpp)
- Modify: `tools/vcam_ws_bridge.py`
- Test: `tools/test_vcam_ws_bridge.py` (extend), `.../micropilot_visualization_node/test/test_vcam_contract.py` (create)

**Interfaces:**
- Consumes: `micropilot_rendering_node/srv/SetVirtualCam` (existing srv package — dependency, not a copy).
- Produces: `~/set_virtual_cam` srv (presets 1–5, eased tween — port the tween math from `rendering_node.cpp` verbatim), `~/set_look` Float64MultiArray[6], `~/vcam_state` Float64MultiArray[8] `[eye|target|preset|mode]`; bridge publishes camera commands to BOTH node namespaces and `set_render_mode` WS cmd now accepts 3 → publishes `/rendering/set_mode`.

- [x] **Step 1: Failing contract test** — call preset k on both nodes, assert `vcam_state` eye/target converge to identical values (tol 1e-9) after tween; `set_look` echo layout identical.
  - Deviation (preset 1 "config" is config-dependent): rendering_node derives preset 1 from its own 12-float R|t `virtual_pose` default; visualization_node's `virtual_pose` is 6-float eye|target. To exercise ALL 5 presets (not just the config-independent left/right/top_down ones) with a real 1e-9 parity check, the test launches visualization_node with `virtual_pose` explicitly set to the eye/target that rendering_node's default R|t reduces to ([0,-4,0]→[0,-5,0]) — documented in the test's docstring/comments, not a hidden fudge.
- [x] **Step 2: Run — FAIL.** Confirmed: with the pre-Task-5 node (stashed old hpp/cpp/CMakeLists/package.xml, rebuilt), `ros2 service call /visualization_node/set_virtual_cam ...` has no such service — the call blocked until the test's 15s subprocess timeout, `TimeoutExpired`.
- [x] **Step 3: Implement.** Ported rendering_node's `smoothstep()`/`advance_tween()`/`on_set_virtual_cam()`/`on_set_look()` verbatim (float precision preserved via a local `LookPoint` struct — see its doc comment: mixing float/double constants like the top_down preset's `0.001f` would leave ~5e-11 of daylight against rendering_node's float, still under the 1e-9 bar today but needlessly fragile) into `visualization_node.{hpp,cpp}`; the only real divergence from rendering_node is that `apply_lookpoint`'s R-matrix rebuild is unnecessary here — Filament's `mpviz::CameraPose` already stores eye/target directly, so the tween writes there straight. Added `~/set_virtual_cam` (srv `micropilot_rendering_node/srv/SetVirtualCam`, an existing-package dependency wired via `find_package(micropilot_rendering_node)` — no duplicate srv), `~/set_look`, `~/vcam_state` (8 floats, published every tick before the mode-3 gate, mirroring rendering_node so telemetry flows regardless of which node is active).
- [x] **Step 4: PASS.** `cuda/src/ros_apps/src/micropilot_visualization_node/test/test_vcam_contract.py`: all 5 presets converge to identical eye/target (max diff 0.0, well under 1e-9) between both nodes, `set_look` echoes identically with `preset=0` on both. Full existing suite re-verified green after: `visual_renderer` ctest 3/3, `micropilot_visualization_node/test/smoke_test.py` PASS, `micropilot_rendering_node/test/smoke_test.py` PASS (incl. its mode-mux dance).
- [x] **Step 5: Bridge E2E** (extend existing WS test): `set_render_mode 3`, orbit via `set_look`, assert frames keep flowing and `vcam_state.mode == 3`.
  - `tools/vcam_ws_bridge.py`: `RENDER_MODES` gained `"visual"`/`3`; `set_render_mode()` now publishes the **global** `/rendering/set_mode` topic (not the old `/rendering_node/set_render_mode`) for every mode 1/2/3 — the global topic already implements the 1/2 semantics too (spec §3.1/Task 4) and is the only one visualization_node listens on, so switching *back* to 1/2 from the WS bridge now correctly exits mode 3 on visualization_node as well (the old code path would have left it stuck rendering after a bowl/pointcloud switch — a latent mux-conflict bug this task's fan-out requirement surfaced, not introduced). `set_look`/`~/set_virtual_cam` now fan out to both `/rendering_node` and `/visualization_node` namespaces (list of publishers/clients; preset ack taken from whichever service answers first, since both report identical success/active for the same preset).
  - Deviation (state-telemetry flicker, found by the E2E test itself): both nodes publish `~/vcam_state` continuously regardless of activity (by design, spec §9), so the bridge's single `self.state` naively overwritten by "whichever arrived last" flickered between the active and inactive node's `render_mode` field — the E2E test caught this directly (`vcam_state.mode != 3: {... 'render_mode': 2}` on the first pass). Fixed by tracking `self._last_mode` (set in `set_render_mode()`) and only accepting a `~/vcam_state` update once its own `mode` field (index 7) agrees with it.
  - New test: `test_bridge_e2e_mode3_orbit_and_frames` in `tools/test_vcam_ws_bridge.py` — launches both real nodes + the real bridge subprocess + a real `websockets` client; `pytest.mark.skipif` when `cuda/install/ros_apps` isn't built (mirrors the GL tests' GPU-skip pattern). Confirmed FAIL first against the pre-fix bridge (stashed `tools/vcam_ws_bridge.py`): 0 frames observed (mode 3 rejected by the old `RENDER_MODES`). Confirmed PASS after the fix; full `tools/test_vcam_ws_bridge.py` 32/32 passed.
  - Full `colcon_build.sh` (all packages) green. **Commit** `feat(visual): vcam contract parity + WS mode-3`.
  - Fix-up (post-review, `_last_mode` filter was itself broken): index 7 of `~/vcam_state` is NOT a shared value — rendering_node publishes `render_mode_` (1|2 only, defaults to 2; `rendering_node.cpp:635`, `.hpp:189`) while visualization_node publishes `active_mode_` (1|2|3; `visualization_node.cpp:186`, `.hpp:106`). `_last_mode` initialized to 1 therefore rejected rendering_node's default state (`[7]=2`) forever — zero telemetry in the shipped default config until a client sent `set_render_mode` — and in modes 1/2 both nodes' `[7]` legitimately read 1, so both passed the filter and `self.state` alternated between rendering_node's and visualization_node's (different) preset-1 poses at 15 Hz. Root-cause fix: stopped comparing an ambiguous field value and instead track which **namespace** is authoritative for the last-commanded mode (`self._active_ns`: `/rendering_node` for 1/2, `/visualization_node` for 3; defaults to `/rendering_node` so its own default state flows pre-command) and only accept `~/vcam_state` from that namespace — each namespace keeps its own subscription callback via a `ns`-bound closure. Extended `test_bridge_e2e_mode3_orbit_and_frames` (still one E2E harness, no new subprocess spin-up) with a pre-phase that reads state with zero commands sent (must arrive, `render_mode == 2`) and a mode-1 phase that asserts every state frame reports the *same* eye/target (no oscillation between the two nodes' distinct default poses). Confirmed FAIL against the stashed pre-fix bridge (default-config frame arrived with visualization_node's pose/`render_mode 1` instead of rendering_node's `2`); PASS after the fix; full `tools/test_vcam_ws_bridge.py` 32/32.

### Task 6: GPU budget measurement (was "gate for Epic 1+"; gate was overridden)

`[review 2026-09-07]` Epic 1 started on "conservative perf assumptions" without this
task ever running (Epic 1 plan, Task 2 notes). The plan now records that honestly:
the gate did not hold, so this task is split into a **proxy measurement done now**
and an **on-robot rerun that gates delivery (VM-043)**, not Epic 1.

**Files:**
- Create: `cuda/src/libs/visual_renderer/tools/budget_probe.md` (procedure + results)

- [x] **Step 1 (proxy, dev box, 2026-09-07):** visualization node at 1280×720, `quality` 0/1/2, `urban` profile, fixture bag `epic2_fixtures_full` looped with `--clock`, alone and beside `micropilot_rendering_node` (idle on mode 3, and active on mode 2 with no camera input — the bag carries no camera topics, so the CUDA node's real render load is NOT exercised). Recorded: `/rendering/image` rate, GPU SM %, node CPU %. Procedure script and raw logs referenced from `budget_probe.md`.
- [x] **Step 2 (proxy):** numbers in `budget_probe.md` and under "Epic 0 results" below; `quality` default chosen from the table.
- [ ] **Step 3 (on-robot, blocking for VM-043):** same matrix on robot hardware with perception + CARLA/real cameras feeding the CUDA node; add frame-time p50/p99 (requires the `render_ms` instrumentation scheduled in VM-034), the `GL_RENDERER` string (VM-037) so the device is attributable, and CUDA-node / perception fps deltas. Note: `quality` today differs across presets only by SSAO on/off and resolution — the 960×540 upscale spec §8 defines for `low` does not exist (VM-032), so per-preset rows overstate available headroom until then. Go/adjust decision on the 720p30 assumption recorded here.

**Epic 0 results:** `FILAMENT_VERSION = 1.56.5` (glibc-2.35-compatible; see ADR-0001), toolchain outcome = clang-14/libc++ root-less static archive imported into the gcc node (ADR-0003), budget table = see `budget_probe.md` (PROXY — on-robot pending), go/no-go = **proxy go** at every preset alone (30 Hz held at low/med/high, GPU SM 21–35 % incl. 16 % desktop baseline, node CPU 33–41 % of one core; CUDA-node co-residence not exercised — bag has no cameras); on-robot decision pending VM-043.

---

## Epics 1–5 — scheduled tasks (each epic gets its bite-sized plan at start)

Task rows reference backlog IDs (AC live there); Files column is the intended
structure at planning time — the epic plan's as-built list wins where they differ
(`[review 2026-09-07]`; Epic 2's rows below were updated to as-built). `[review 2026-09-07]` Interfaces marked ➤ were "frozen upon that epic's review"; that policy is replaced by additive-only versioning (ADR-0004) — the ➤ lines below now mean "stable, extend by appending".

### Epic 1 — Core scene & dark theme
| Task | Backlog | Files (create unless noted) |
|---|---|---|
| SceneGraph structs + double buffer + staleness clocks | VM-010 | `visual_renderer/include/visual_renderer/scene.h` (POD), `src/scene_buffer.cpp`, `tests/test_scene_buffer.cpp` |
| Theme YAML → materials (BOTH `dark_adas.yaml` + `light_clay.yaml`, v1) | VM-011 | `src/theme.cpp`, `assets/themes/{dark_adas,light_clay}.yaml`, `tests/test_theme.cpp`, golden harness `tests/golden.{cpp,py}` |
| Animated theme toggle (0.8 s eased token lerp, Oklab; GUI day/night toggle + WS `set_theme`) | VM-014 | lib `src/theme_transition.cpp`, `tools/vcam_gui.py`, `tools/vcam_ws_bridge.py`, deterministic-clock goldens |
| M02P→glTF script + ego rendering + TF speed | VM-012 | `scripts/obj2gltf_m02p.py`, `src/ego.cpp`, node `src/tf_adapter.cpp`, fixtures |
| Preset/tween port shared with Task 5 code | VM-013 | node `src/vcam.cpp` refactor |

➤ Frozen after Epic 1: `mpviz::SceneGraph` POD layout, `set_scene(VisualRenderer*, const SceneGraph&)` entry point, golden-test harness CLI.

### Epic 2 — Autonomy data ingestion
| Task | Backlog | Files |
|---|---|---|
| Profile YAML loader (urban/offroad/sim) + `FrameTransformer` + `SceneAssembly` + fixture tooling | VM-020 | node `src/profile.cpp`, `src/frame_transform.cpp`, `src/scene_assembly.cpp`, `config/{urban,offroad,sim}_profile.yaml`, `scripts/bag_to_fixture.py`, `test/fixture_msgs.*`, tests |
| DynamicObjectsAdapter + class inference table | VM-021 | node `src/adapters/dynamic_objects.cpp`, `config/class_inference.yaml`, bag-fixture tests |
| Clay models: normalize CC0 pack, instancing, bbox scaling, arrows, predicted ribbons | VM-022 | `scripts/normalize_models.py`, `assets/models/*.glb`, `src/objects.cpp`, goldens |
| Path ribbons ×3 roles | VM-023 | node `src/adapters/path.cpp`, lib `src/ribbon.cpp`, goldens |
| HdMapAdapter + lane styling (cached) + ego-following ground + dashed centerlines (ingest chop) | VM-024 | node `src/adapters/hd_map.cpp`, lib `src/map_elements.cpp`, `src/polyline.cpp` (shared extruder, reused by VM-022/023/026), goldens per theme |
| OGM layers + `_updates` | VM-025 | node `src/adapters/ogm.cpp`, lib `src/ground_grid.cpp`, fixtures |
| Collision alert polygons | VM-026 | node `src/adapters/collision.cpp`, lib `src/alert_polygons.cpp`, goldens |
| Generic marker fallback (all 12 Marker types, pooled) + TF-axes debug layer (off by default) | VM-027 | node `src/adapters/generic_marker.cpp`, `src/adapters/tf_axes.cpp`, lib `src/generic_markers.cpp`, `test/test_extra_topic_parity.py` |

**Epic 2 CLOSED at fd72331 (2026-09-07)** — gate PASSED 2026-08-20 (37d41fe) with user-authorized deviations; post-gate: `flatten_z` param (02939c3), three ribbon theme tokens (3b3ce2c), gt-boxes + `/road_markers` rows disabled (fd72331). Status ledger in the Epic 2 plan.

### Epic 3 — HUD, polish, controls
`[review 2026-09-07]` Reordered: the two items that fix what is visible today
(VM-036 lane kinds + road surface, VM-034 fades + `render_ms` instrumentation)
come first; HUD text follows. VM-036 was missing from this table although the
backlog scheduled it in Epic 3.
| Task | Backlog | Files |
|---|---|---|
| `MapElement.kind` + `last_update_sec` (additive, ADR-0004); per-kind theme tokens/width/z-lift; **road-surface fill** between paired `left_boundary_{id}`/`right_boundary_{id}` in a `palette.road` token so the road reads darker than the surrounding clay ground (ref-2's primary value separation, unexpressible today because every map element is a stroke); move centerline dashing renderer-side and retire the ingest chop | VM-036 | lib `scene.h` (append), `src/map_elements.cpp`, themes `*.yaml` (+`road`, `lane_centerline`, `lane_boundary`, `crosswalk` tokens), node `src/adapters/hd_map.cpp`, goldens per theme |
| Staleness fades for map layer (needs `last_update_sec` above) + diagnostics topic (per-topic age, dropped counts) + **`render_ms` per frame** (time `render_frame()` in the node tick, publish in diagnostics; prerequisite for VM-040 and for the on-robot Task 6 rerun) | VM-034 | node `src/diagnostics.cpp`, GUI |
| HUD overlay (speed chip, mode indicator). **Accepted (user 2026-09-07) `[review 2026-09-07]`:** composite text CPU-side onto the RGB8 buffer after readback in the node (stb_truetype, vendored like stb_image_write) instead of an SDF atlas + Filament overlay pass in the lib. Same pixels for a 2D HUD, no font pipeline, no new public entry point, theme colors via the existing HUD tokens; the SDF path stays the plan only for 3D-anchored in-scene text (VM-031 leader lines). Re-entry trigger: HUD text must be depth-tested or lit. | VM-030 | node `src/hud_overlay.cpp`, `assets/fonts/`, goldens (node-side test renders a known scene + HUD) |
| Leader-line alert callouts (3D anchor → screen-space chip; since VM-030's CPU composite is accepted, the lib exposes only `project_to_screen()` for the anchor and the chip is drawn node-side) | VM-031 | lib `src/callouts.cpp` or `api.h` projection helper, goldens |
| Layers/quality: params + WS + GUI panel (theme toggle shipped in VM-014) | VM-032 | node param plumbing, `tools/vcam_ws_bridge.py`, `tools/vcam_gui.py`, WS E2E |
| PointCloudLayer: PointCloud2 → colored points, per-row `color_mode: auto\|rgb\|intensity\|height\|flat` (auto = rgb → intensity ramp → height ramp) | VM-035 | lib `src/point_cloud.cpp` + `SceneGraph` category (appended, ADR-0004), node `src/adapters/point_cloud.cpp`, goldens |
| Epic-0 debt: mux hardening + build hygiene (mux QoS transient_local, legacy `~/set_render_mode` re-publish, `initial_mode`/`render_mode_` sync, `vcam_state[8] = mux_mode`, `check_pod_header.sh` wired into colcon, `FILAMENT_VERSION` single-sourced, `GL_VENDOR`/`GL_RENDERER`/`GL_VERSION` logging, bluegl link-probe). **Scheduled into this epic in full (user decision 2026-09-07)** — item (e) (`kSceneVersion` + node-side layout asserts) ships earlier, inside this epic's first task. | VM-037 | node `src/visualization_node.cpp`, `src/rendering_node.cpp`/`.hpp`, `micropilot_visualization_node/CMakeLists.txt`, lib `scripts/check_pod_header.sh`, `src/renderer.cpp`, `test/smoke_test.py` |

### Epic 4 — Clay buildings (EnvironmentLayer)
`[review 2026-09-07]` Nothing of this epic exists yet (no bake script, no geo
anchor). Bag fact: `/sim/feedback/gps` (NavSatFix, ~10 Hz) IS in the fixture
bag, so VM-050 has real fixture data. A Mapbox token is available to the user
(env var, never committed) should Overpass be rate-limited.
| Task | Backlog | Files |
|---|---|---|
| Geo-anchor: NavSatFix (WGS84, robot-published — PRIMARY) + `gps_link` TF → map↔ENU; `geo_datum` param override | VM-050 | node `src/geo_anchor.cpp`, round-trip tests |
| `bake_environment.py` (OSM/Overpass → chunked glTF + index + verification overlay) | VM-051 | `cuda/src/libs/visual_renderer/scripts/bake_environment.py`, tests on cached extract |
| Runtime chunk load + distance culling | VM-052 | lib `src/environment.cpp` behind `EnvironmentSource` seam, goldens, perf check |

### Epic 5 — Hardening & delivery

`[user decision 2026-09-11]` Epic 5 runs PARTIALLY in parallel with the
unified-engine migration (2026-09-10 plan, Decision 10), per its file-overlap
analysis: **VM-041** and **VM-042's docs half** (profile-authoring + bake
runbook; deployment notes deferred to post-cutover) pulled forward NOW in
worktrees; **VM-044** slots after migration Task 2/VM-091 (same
`visualization_node.cpp` files, and it feeds Task 6's M02P-glTF sign-off
exception); **VM-040** slots after migration Task 5/VM-094 (the governor
tunes against the perf envelope the camera/bowl work is changing); **VM-043**
is superseded — its on-robot rerun and parity checklist close inside
migration Task 6 (VM-095), not as a separate run.

**VM-044 done (2026-09-11, `TPSProjector-vm044` worktree):** see the backlog
entry's own Done note (`docs/superpowers/specs/2026-08-18-visual-mode-backlog.md`)
for the full record; feeds migration Task 6 Step 1's named exception (3) —
noted there too.

**VM-040 done (2026-09-11):** migration Task 5/VM-094's perf gate had
already landed (implementation complete, only its golden's human-sanity
approval was pending — see the migration plan's own Status ledger row 5),
so this task's own "slots after Task 5/VM-094" sequencing note above was
satisfied and it proceeded. Files actually used: lib `set_quality()`/
`get_quality()` (`include/visual_renderer/scene.h`, `src/renderer.cpp` —
NOT a new `quality_governor.cpp` in the library, see below) + node
`micropilot_visualization_node/{include,src}/.../quality_governor.{hpp,cpp}`
(the governor is node-side, per this row's own original file list's intent
— `render_ms` and the mux/timer loop it reacts to both live there). See the
backlog entry's own Done note
(`docs/superpowers/specs/2026-08-18-visual-mode-backlog.md`) for the full
record, including the set_quality-over-re-create design decision this row's
own text left to "the Epic 5 plan" (that plan was never separately
authored; the backlog Done note is the decision of record).

| Task | Backlog | Files |
|---|---|---|
| Quality auto-drop w/ hysteresis | VM-040 | node `include/.../quality_governor.hpp` + `src/quality_governor.cpp`, load test (`test/test_quality_governor.cpp`); lib `set_quality()`/`get_quality()` (`scene.h`/`renderer.cpp`) |
| Perf benchmark + **repo-local CI gate** (`[review 2026-09-07]` the repo has no hosted CI — no `.github/workflows`, no GitLab CI; "CI wiring" therefore means one `tools/ci_visual_mode.sh` that runs POD check, lib ctest, node gtests, WS bridge tests and the golden suite with GPU-skip, documented as the pre-merge gate; hosted CI is a follow-up once a platform exists) | VM-041 | `tools/viz_benchmark.cpp`, `tools/ci_visual_mode.sh` |
| Docs + profile-authoring + bake runbooks | VM-042 | `README.md` section, `docs/visual_mode/*.md` (directory created by this task; ADRs stay in `docs/adr/`) |
| Live validation: CARLA + real bag, rviz side-by-side parity sign-off; **on-robot budget rerun (Epic 0 Task 6 Step 3) is a checklist item and blocks sign-off** | VM-043 | checklist in `docs/visual_mode/signoff.md`, `budget_probe.md` |

### Epic 6 — v1.1: 3D Tiles streaming (committed; starts immediately after v1.0 ships)
`[review 2026-09-07]` The review proposed deferring this epic to Future; the
user rejected that on 2026-09-07 — it stays committed. Its bite-sized plan
(`2026-08-18-visual-mode-epic6.md`) is authored at v1.0 sign-off (VM-043) with
the same Fable/Sonnet/Opus workflow. Prerequisite to schedule at v1.0 sign-off:
user performs Cesium ion registration; token handled like the Mapbox token
(env var, never committed). Backlog VM-060…VM-063.
| Task | Backlog | Files |
|---|---|---|
| Cesium ion registration + tileset access | VM-060 | runbook in `docs/visual_mode/cesium.md` |
| cesium-native pinned build (clang/libc++, POD rules) | VM-061 | `visual_renderer/cmake/GetCesiumNative.cmake` |
| Streaming `EnvironmentSource` (clay re-materialize, disk cache) | VM-062 | lib `src/environment_stream.cpp` |
| Source selection + baked fallback on network loss | VM-063 | node param plumbing, fallback e2e test |
| Google Photorealistic 3D Tiles: original-materials mode + 3-preset source config (added 2026-09-11, user decision) | VM-064 | lib `src/environment_stream.cpp` (materials= key), config presets, runbook compliance notes |

## Review 2026-09-07 — self-review

- Status now has one trustworthy home per epic (the ledgers); checkboxes were ticked only where a commit proves the step.
- Every finding from the Opus audit that survived adversarial verification is either applied inline (`[review 2026-09-07]` markers), scheduled under a backlog ID (VM-037, VM-044, VM-036 additions), or was decided by the user on 2026-09-07 (Decisions in the changelog above).
- Refuted findings (11) are not applied; the refutations are in the review workflow transcript.

## Self-review (done)

- Spec coverage: §2→T1/T2, §3.1→T3/T4, §3.2→T1/T3, §4.1→E1, §4.2→T2+E1–E3, §4.3→VM-011/014, §4.4→VM-012/022/030, §4.5→E4, §5→E2, §6→T5/VM-013, §7→VM-027 + parity sign-off VM-043, §8→T6/VM-040/041, §9→VM-021/034 + T3 lifecycle, §10→every task's test steps. No gaps.
- Placeholders: the two `____` slots in "Epic 0 results" are deliberate measurement blanks filled by Task 6, not plan omissions; epic plan docs are scheduled deliverables with defined authors/inputs.
- Type consistency: `mpviz::` API names match across Tasks 2/3/5 and the frozen-interface notes.
