# Visual Mode Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
> **Execution model (project directive 2026-08-18):** run each phase as a dynamic Workflow — orchestrator Fable, implementer agents `model: "sonnet"`, reviewer agents `model: "opus"`.

**Goal:** Add a third render mode ("Visual") — a Filament-based stylized 3D visualization of all autonomy data — streamed on the existing `/rendering/image` pipeline with the existing WebSocket-driven virtual camera.

**Architecture:** New ROS-free clang/libc++ library `micropilot::visualization` (Filament headless → RGB8 behind a POD-only API) + new lifecycle node `micropilot_visualization_node` (topic adapters → SceneGraph), muxed with the existing CUDA node via a global `/rendering/set_mode` topic so exactly one node publishes the stream.

**Tech Stack:** C++17, Google Filament (pinned release), ROS 2 Humble (rclcpp_lifecycle), colcon/ament, gtest, Python (bake/asset scripts, WS bridge/GUI).

**Spec:** `docs/superpowers/specs/2026-08-18-visual-mode-design.md` (+ backlog `.../2026-08-18-visual-mode-backlog.md`)

## Global Constraints

- Existing modes 1–2 and every current test stay green; the CUDA node changes ONLY to add `/rendering/set_mode` idle-on-3 behavior.
- Output contract frozen: `sensor_msgs/Image` rgb8 on `/rendering/image` + `/rendering/camera_info`; vcam surface message layouts identical to `micropilot_rendering_node` (reuse `micropilot_rendering_node/srv/SetVirtualCam` — do NOT define a duplicate srv).
- `visual_renderer` public header: no std:: types beyond `<cstdint>`/`<cstddef>` (POD boundary; CI-checked). Lib builds clang/libc++; node builds stock gcc.
- Filament version pinned once in Epic 0 (record in this doc); all epics build against it.
- TDD per repo convention; GPU tests skip cleanly without a GPU (pattern: existing GL tests).
- Perf target: 1280×720 @ 30 fps on the onboard GPU alongside the CUDA node (validated Epic 0, enforced Epic 5).
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

**Interfaces:**
- Produces: CMake target `micropilot_visualization::visual_renderer` (static, clang/libc++), consumed by Tasks 2–3 and the node package.

- [ ] **Step 1: Write the failing build check.** `check_pod_header.sh`:

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

- [ ] **Step 2: Run it — expect FAIL (api.h missing).**
- [ ] **Step 3: `GetFilament.cmake`** — download the pinned Filament Linux release tarball (record the chosen version here on completion: `FILAMENT_VERSION = ____`), verify sha256, expose imported targets; document the source-build fallback path in a comment.
- [ ] **Step 4: `CMakeLists.txt`** — static lib `visual_renderer`, `CMAKE_CXX_COMPILER=clang++` + `-stdlib=libc++` enforced for this directory only (error out otherwise), links Filament + headless backend deps (EGL), runs `check_pod_header.sh` as a test.
- [ ] **Step 5: Create minimal `include/visual_renderer/api.h`** (see Task 2 Step 1 for content), build the empty lib, run the POD check — PASS.
- [ ] **Step 6: Commit** `feat(visual): Filament build integration + POD boundary check`.

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

- [ ] **Step 1: Failing gtest** — create renderer 320×240, render one frame with eye [-4,0,3.5] target [2,0,-0.5] (the node's default pose), assert: returns true, buffer not all-zero, sky pixels (top rows) differ from ground pixels (bottom rows). Skip test with `GTEST_SKIP()` when `create_renderer` returns nullptr AND no GPU/EGL device present.
- [ ] **Step 2: Run — FAIL (link error).**
- [ ] **Step 3: Implement** `renderer.cpp`: Filament Engine (OpenGL backend, headless EGL), SwapChain `CONFIG_READABLE`, Scene with one directional sun + gray ground plane + grid lines + a lit cube at origin; `render_frame` = set camera lookAt/projection → `renderer->render(view)` → `readPixels` → flip to top-down RGB into `out.rgb`.
- [ ] **Step 4: Run — PASS.** Also run the example (`hello_frame.cpp` writes `hello_frame.png` via stb_image_write) on the dev box WITHOUT `$DISPLAY` set; visually confirm cube+grid+horizon.
- [ ] **Step 5: Commit** `feat(visual): headless Filament hello-frame behind POD API`.

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

- [ ] **Step 1: Failing smoke test** — launch node with `initial_mode:=3`, assert ≥5 frames on `/rendering/image` within 3 s, correct encoding/dimensions; with `initial_mode:=1`, assert ZERO frames in 2 s.
- [ ] **Step 2: Run — FAIL (package doesn't exist).**
- [ ] **Step 3: Implement** node mirroring `rendering_node.cpp`'s structure (configure→create pubs, activate→start timer); timer tick: if mode==3, `render_frame` into a preallocated `sensor_msgs/Image` and publish. Subscribe `/rendering/set_mode` (Int32, values 1|2|3; else WARN).
- [ ] **Step 4: Run smoke — PASS. Commit** `feat(visual): visualization node streaming hello-frame on mode 3`.

### Task 4: Mode mux in the CUDA node

**Files:**
- Modify: `cuda/src/ros_apps/src/micropilot_rendering_node/src/rendering_node.cpp` (subscription block ~line 278; publish gate ~line 790)
- Modify: `.../include/micropilot_rendering_node/rendering_node.hpp`
- Modify: `.../test/smoke_test.py`

**Interfaces:**
- Produces: `rendering_node` subscribes `/rendering/set_mode`; on 3 it skips render+publish (buffers keep filling); on 1|2 identical to today's `~/set_render_mode`, which stays and implies leaving mode 3.

- [ ] **Step 1: Failing smoke extension** — publish `/rendering/set_mode: 3`, assert rendering_node stops publishing within 0.5 s while visualization_node starts; drive 3→2→3→1, assert exactly-one-publisher at each step and no gap > 0.5 s.
- [ ] **Step 2: Run — FAIL.**
- [ ] **Step 3: Implement** (one `active_mode_` int; render tick early-outs on 3; `~/set_render_mode` handler also clears mode-3 state). `initial_mode` param on both nodes, default 1.
- [ ] **Step 4: Run full existing test suite + smoke — PASS. Commit** `feat(mux): global /rendering/set_mode across both nodes`.

### Task 5: vcam contract on the new node + WS bridge fan-out

**Files:**
- Modify: `.../micropilot_visualization_node/src/visualization_node.cpp` (+hpp)
- Modify: `tools/vcam_ws_bridge.py`
- Test: `tools/test_vcam_ws_bridge.py` (extend), `.../micropilot_visualization_node/test/test_vcam_contract.py` (create)

**Interfaces:**
- Consumes: `micropilot_rendering_node/srv/SetVirtualCam` (existing srv package — dependency, not a copy).
- Produces: `~/set_virtual_cam` srv (presets 1–5, eased tween — port the tween math from `rendering_node.cpp` verbatim), `~/set_look` Float64MultiArray[6], `~/vcam_state` Float64MultiArray[8] `[eye|target|preset|mode]`; bridge publishes camera commands to BOTH node namespaces and `set_render_mode` WS cmd now accepts 3 → publishes `/rendering/set_mode`.

- [ ] **Step 1: Failing contract test** — call preset k on both nodes, assert `vcam_state` eye/target converge to identical values (tol 1e-9) after tween; `set_look` echo layout identical.
- [ ] **Step 2: Run — FAIL. Step 3: Implement. Step 4: PASS.**
- [ ] **Step 5: Bridge E2E** (extend existing WS test): `set_render_mode 3`, orbit via `set_look`, assert frames keep flowing and `vcam_state.mode == 3`. Commit `feat(visual): vcam contract parity + WS mode-3`.

### Task 6: On-robot GPU budget measurement (gate for Epic 1+)

**Files:**
- Create: `cuda/src/libs/visual_renderer/tools/budget_probe.md` (procedure + results)

- [ ] **Step 1:** Run visualization node (mode 3, 720p, each quality preset) beside the CUDA node (mode idle + mode active) + perception on robot hardware; record: frame time p50/p99, readback time, CUDA-node fps delta, perception fps delta.
- [ ] **Step 2:** Record numbers in `budget_probe.md` AND in this plan under "Epic 0 results"; decide go/adjust (resolution or preset defaults) for the 720p30 target.
- [ ] **Step 3: Commit.** **Epic 0 review gate:** Opus reviewer signs off spike vs spec §§2–3, 6, 8.

**Epic 0 results (fill on completion):** `FILAMENT_VERSION = ____`, toolchain outcome = ____, budget table = see budget_probe.md, go/no-go = ____.

---

## Epics 1–5 — scheduled tasks (each epic gets its bite-sized plan at start)

Task rows reference backlog IDs (AC live there); Files column locks the file
structure now. Interfaces marked ➤ are frozen upon that epic's review.

### Epic 1 — Core scene & dark theme
| Task | Backlog | Files (create unless noted) |
|---|---|---|
| SceneGraph structs + double buffer + staleness clocks | VM-010 | `visual_renderer/include/visual_renderer/scene.h` (POD), `src/scene_buffer.cpp`, `tests/test_scene_buffer.cpp` |
| Theme YAML → materials (BOTH `dark_adas.yaml` + `light_clay.yaml`, v1) | VM-011 | `src/theme.cpp`, `assets/themes/{dark_adas,light_clay}.yaml`, `tests/test_theme.cpp`, golden harness `tests/golden.{cpp,py}` |
| Animated theme toggle (0.8 s eased token lerp, Oklab; GUI day/night toggle + WS `set_theme`) | VM-014 | lib `src/theme_transition.cpp`, `tools/vcam_gui.py`, `tools/vcam_ws_bridge.py`, deterministic-clock goldens |
| M02P→glTF script + ego rendering + TF speed | VM-012 | `scripts/obj2gltf_m02p.py`, `src/ego.cpp`, node `src/tf_adapter.cpp`, fixtures |
| Preset/tween port shared with Task 5 code | VM-013 | node `src/vcam.cpp` refactor |

➤ Frozen after Epic 1: `mpviz::SceneGraph` POD layout, `set_scene(VisualRenderer*, const SceneGraph*)` entry point, golden-test harness CLI.

### Epic 2 — Autonomy data ingestion
| Task | Backlog | Files |
|---|---|---|
| Profile YAML loader (urban/offroad) | VM-020 | node `src/profile.cpp`, `config/{urban,offroad}_profile.yaml`, tests |
| DynamicObjectsAdapter + class inference table | VM-021 | node `src/adapters/dynamic_objects.cpp`, `config/class_inference.yaml`, bag-fixture tests |
| Clay models: normalize CC0 pack, instancing, bbox scaling, arrows, predicted ribbons | VM-022 | `scripts/normalize_models.py`, `assets/models/*.glb`, `src/objects.cpp`, goldens |
| Path ribbons ×3 roles | VM-023 | node `src/adapters/path.cpp`, lib `src/ribbon.cpp`, goldens |
| HdMapAdapter + lane styling (cached) | VM-024 | node `src/adapters/hd_map.cpp`, lib `src/map_elements.cpp`, goldens |
| OGM layers + `_updates` | VM-025 | node `src/adapters/ogm.cpp`, lib `src/ground_grid.cpp`, fixtures |
| Collision alert polygons | VM-026 | node `src/adapters/collision.cpp`, lib `src/alert_polygons.cpp`, goldens |
| Generic marker fallback (all primitive types, pooled) | VM-027 | node `src/adapters/generic_marker.cpp`, lib `src/generic_markers.cpp`, parity test |

### Epic 3 — HUD, polish, controls
| Task | Backlog | Files |
|---|---|---|
| SDF font + HUD overlay (speed, mode) | VM-030 | lib `src/{text,hud}.cpp`, `assets/fonts/`, goldens |
| Leader-line alert callouts | VM-031 | lib `src/callouts.cpp`, goldens |
| Layers/quality: params + WS + GUI panel (theme toggle shipped in VM-014) | VM-032 | node param plumbing, `tools/vcam_ws_bridge.py`, `tools/vcam_gui.py`, WS E2E |
| Staleness fades + diagnostics topic in GUI | VM-034 | node `src/diagnostics.cpp`, GUI |

### Epic 4 — Clay buildings (EnvironmentLayer)
| Task | Backlog | Files |
|---|---|---|
| Geo-anchor: NavSatFix (WGS84, robot-published — PRIMARY) + `gps_link` TF → map↔ENU; `geo_datum` param override | VM-050 | node `src/geo_anchor.cpp`, round-trip tests |
| `bake_environment.py` (OSM/Overpass → chunked glTF + index + verification overlay) | VM-051 | `cuda/src/libs/visual_renderer/scripts/bake_environment.py`, tests on cached extract |
| Runtime chunk load + distance culling | VM-052 | lib `src/environment.cpp` behind `EnvironmentSource` seam, goldens, perf check |

### Epic 5 — Hardening & delivery
| Task | Backlog | Files |
|---|---|---|
| Quality auto-drop w/ hysteresis | VM-040 | lib `src/quality_governor.cpp`, load test |
| Perf benchmark + CI wiring (GPU-skip, Filament cache) | VM-041 | `tools/viz_benchmark.cpp`, CI scripts |
| Docs + profile-authoring + bake runbooks | VM-042 | `README.md` section, `docs/visual_mode/*.md` |
| Live validation: CARLA + real bag, rviz side-by-side parity sign-off | VM-043 | checklist in `docs/visual_mode/signoff.md` |

## Self-review (done)

- Spec coverage: §2→T1/T2, §3.1→T3/T4, §3.2→T1/T3, §4.1→E1, §4.2→T2+E1–E3, §4.3→VM-011/014, §4.4→VM-012/022/030, §4.5→E4, §5→E2, §6→T5/VM-013, §7→VM-027 + parity sign-off VM-043, §8→T6/VM-040/041, §9→VM-021/034 + T3 lifecycle, §10→every task's test steps. No gaps.
- Placeholders: the two `____` slots in "Epic 0 results" are deliberate measurement blanks filled by Task 6, not plan omissions; epic plan docs are scheduled deliverables with defined authors/inputs.
- Type consistency: `mpviz::` API names match across Tasks 2/3/5 and the frozen-interface notes.
