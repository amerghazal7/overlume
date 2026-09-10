# Unified Rendering Engine Migration — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
> **Execution model (project directive 2026-08-18, same as every Visual Mode epic):** run this epic as a dynamic Workflow — orchestrator Fable, implementer agents `model: "sonnet"`, reviewer agents `model: "opus"`.

**Status: NOT STARTED.** Authored 2026-09-10 from the exhaustive research pass reproduced/cited throughout (all facts re-verified against the live repo during authoring — path:line citations are live, not carried over unchecked).

## USER DECISION (verbatim — this is the epic's charter)

> "a new decision to migrate everything implemented in the rendering node to the new engine so we have on node one view and we can swith the mode between all modes and everything use the same rendering engine, if the plan doesn't account for that, let's consider this and make it our next big thing and most important to get done"

Every task below exists to satisfy this literally: **one node, one Filament-backed engine, a local switch between all three modes.** Where a cheaper option would satisfy "one node" but not "same engine" (Research §4 Option C — keep CUDA as a Filament-fed texture), it is named and rejected, not silently taken.

**Goal:** Retire `micropilot_rendering_node`'s CUDA reprojector entirely. `micropilot_visualization_node` (already Filament/mode-3-native) grows modes 1 (bowl) and 2 (pointcloud-hybrid), rendered through the exact same `visual_renderer` engine mode 3 already uses, then becomes the only rendering process; the two-node mux (ADR-0002) is deleted once parity is signed off.

**Architecture:** Bowl reprojection — today a hand-written CUDA rasterizer (`reproject.cu`) — moves to a Filament mesh + material: camera projection/distortion math is ported to portable host C++ once (TDD'd standalone), used to bake per-vertex, per-camera UV+blend-weight vertex attributes on the CPU whenever bowl config changes, and a small multi-texture-sample fragment shader does the per-pixel blend GPU-side — the same "rasterizer does the projection" idea the research inventory names as CUDA's whole reason for existing (§1.12) minus the reimplementation. Camera pixels cross the clang/libc++ POD boundary via a dedicated `set_camera_frame()` entry point (Research §4 Option B), not a `SceneGraph`/`set_scene()` field, because they are a new *size class* the existing deep-copy staging buffer was never built for. Robot self-view/proxy compositing stops being a bespoke z-buffer kernel and becomes "add the already-Filament-native ego mesh to the same scene as the bowl" — Filament's own depth test does the compositing for free. Mode 2 becomes "mode 1's bowl, plus the already-shipped `PointCloud` layer (VM-035), colorized by a new node-side adapter" — no new library rendering code at all.

**Tech Stack:** Filament 1.56.5 (pinned, ADR-0001), the existing `visual_renderer` clang/libc++ static archive + POD boundary (ADR-0003), `micropilot_visualization_node` (gcc/libstdc++), CycloneDDS with the shared-memory transport fix already in production for 6-camera ingest.

**Spec:** No prior spec document covers this — the research inventory reproduced in full context above (2026-09-10 research pass) *is* this plan's spec; every Decision below cites it by section (`§N`) alongside live path:line re-verification.

## Global Constraints (bind every task below)

- **ADR-0004 (additive-only, versioned `scene.h`):** every task that appends a POD struct with a `sizeof`/`offsetof` layout bumps `kSceneVersion` (currently **4** — Epic 4 Task 1, `scene.h:33`) and updates BOTH `tests/test_scene_buffer.cpp` and the node-side `test_scene_layout.cpp` mirror. A free function addition alone (no new struct) does not bump it, per Epic 4's own precedent (`set_environment_source` added no second bump).
- **POD boundary** (ADR-0003, `api.h:1-8`, `scene.h:1-12`, `scripts/check_pod_header.sh`): nothing but `<cstdint>`/`<cstddef>` types crosses `api.h`/`scene.h`. Camera pixel buffers cross as a raw `const uint8_t*` + dimensions, exactly like every other array in this codebase — no `std::vector`, no smart pointer.
- **Two-yaml-cpp rule** (Epic 2 Task 1 Step 0, restated in every epic since): the library's bundled yaml-cpp and the node's `yaml_cpp_vendor` (`micropilot_visualization_node/CMakeLists.txt:24`) are two independently-built copies on two toolchains that must never `#include` each other's headers. **Nothing in this epic needs a third YAML surface** — camera extrinsics/intrinsics/bowl config stay ROS parameters (`declare_parameter`, exactly as `rendering_node.cpp:181` already does it), not a YAML file parsed by either copy. Stated for completeness, not because a task here touches it.
- **CycloneDDS SHM camera-transport constraint** (`rendering_node.launch.py:36-45`, fix commit `69b5a3a`): 6× ~4 MB CARLA frames over default loopback-UDP collapsed sim FPS 32→8 Hz. This is a transport-layer fact, not a rendering-engine one — it survives node consolidation unchanged (still 6 raw-image subscriptions, just in one process) and the merged node's launch file must carry the same SHM-forcing config forward. Named again in Task 4.
- **STANDING directive 2026-09-09** (`visual-mode-epic3.md:724-732`, verbatim: "add config entry to customize style or disable any element we are adding to the rendering now and in the future"): the camera bowl is a newly-added rendered element under this rule — it ships a disable knob (Task 2) exactly like `hud_enabled`/`environment_enabled` before it. "Style" for the bowl is its `sky_color`/`feather_margin`/exposure behavior, already config-driven params carried over from the CUDA node — no new theme YAML token is invented where a param already owns the value (the bowl isn't clay-material themed the way buildings/ego are; it's camera pixels).
- **Golden scoping (P3):** the 2026-09-07 user decision scopes "goldens per theme" to goldens whose subject is theming. The bowl/mode-1/mode-2 goldens introduced here are visual-fidelity references against the CUDA node's own output, not theme goldens — they ship **one** comparison capture each, human-sanity-approved, not a per-theme set (the bowl doesn't participate in the clay theme system at all).
- **Hand-written CMake source lists** (`micropilot_visualization_node/CMakeLists.txt:215`, "Hand-written list, NOT a glob"): every new node-side `.cpp` this epic adds must be appended to that list **and** every `ament_add_gtest` target that needs it, or it silently no-ops. The library side globs (`CONFIGURE_DEPENDS`, `CMakeLists.txt:157`) except the materials glob (`CMakeLists.txt:127`, **no** `CONFIGURE_DEPENDS`) — Task 2 **does** add a new `.mat` file (`bowl.mat`, unlike Epic 4's buildings), so that trap applies here and is named again at the task.
- Existing modes 1–2's shipped behavior and every current test stay green on `micropilot_rendering_node` **until this epic's own parity sign-off (Task 6)** — this epic does not touch that node's CUDA code at all except at final decommission.
- TDD per repo convention; GPU tests skip cleanly without a GPU (`HasGpuEglDevice()`/`GTEST_SKIP()` convention, `test_hello_frame.cpp`). Perf numbers stay proxy-only (dev-box RTX 3090-class) until Task 6's on-robot rerun, same honesty `budget_probe.md` already keeps.
- ROS build: `cuda/scripts/ros_apps_build/colcon_build.sh`.
- **Backlog IDs:** this is a brand-new epic with no prior backlog entry. **VM-090–VM-095** are assigned here for the first time, not yet mirrored into `docs/superpowers/specs/2026-08-18-visual-mode-backlog.md` — Task 6's close reconciles that doc, same as every epic's results block does for its own IDs.

---

## Status ledger

| Task | Backlog | Status | Notes |
|---|---|---|---|
| 1 POD-boundary camera-texture mechanism (ADR-0005) | VM-090 | Not started | Foundation for every later task; also the one place this plan asks for an explicit user go/no-go (see Decision 2). |
| 2 Mode 1 (bowl) migration into `micropilot_visualization_node` | VM-091 | Not started | Built and perf-gated behind the still-live mux — not yet authoritative in production. |
| 3 Self-view masks + robot-proxy compositing (Filament-native) | VM-092 | Not started | Rides Task 2's bowl scene; reuses the already-Filament ego mesh instead of a new kernel. |
| 4 Node consolidation — local mode switch, camera ingest moved, mux still live | VM-093 | Not started | Old node + mux stay running throughout; this only gives the new node the *ability* to answer modes 1/2. |
| 5 Mode 2 (pointcloud-hybrid) migration — camera-colorized lidar | VM-094 | Not started | Gated on Task 2; reuses the already-shipped `PointCloud` layer (VM-035) — no new library rendering surface. |
| 6 Cutover — parity sign-off, mux deleted LAST, old node decommissioned | VM-095 | Not started | ADR-0006 supersedes ADR-0002. The on-robot perf rerun (Epic 5's VM-043 blocker) closes here, for real, because there is finally only one process to measure. |

---

## Decisions (grounded in the research inventory + re-verified live, 2026-09-10)

### 1. Migration order is risk-ranked: bowl (mode 1) → self-view/robot-proxy → node consolidation → hybrid (mode 2) → cutover

Per the research's own §8: mode 1 is the whole existing product surface (mode 2 degrades to it with nothing else), has the clearest Filament-equivalent shape (project N textured views onto one mesh — a GL rasterizer's native job), and its perf risk is boundable with dirty-tracking precedent that already exists (`img_dirty_`, `rendering_node.hpp:220`, `.cpp:769-786`). Self-view/robot-proxy compositing reuses mode 1's texture path and is lower-stakes (cosmetic, already optional/off-by-default, `rendering_node.cpp:80-84`). Node consolidation (the mux-facing plumbing) is scheduled *before* mode 2 deliberately: it lets Task 5's camera-colorization adapter reuse the same live camera images Task 2 already ingested, instead of a second ingest path. Mode 2 is scheduled last of the rendering tasks because its camera-colorization gap (research §2, §7.3) has no current Filament precedent and should not block "one node, modes 1+3 working" from shipping first — **do not attempt bowl and hybrid simultaneously**, restated verbatim from the research's own recommendation.

### 2. The POD-boundary camera-texture mechanism: Option B (`set_camera_frame()`), not A or C — and one sub-question stays a real user decision

Research §4 laid out three options. Re-argued here with the numbers re-derived, not just cited:

- **Option A (a `CameraImagePlane[]` SceneGraph category)** would make `set_scene()`'s deep-copy staging buffer (`scene_buffer.cpp:7-103`) `std::vector::assign()` ≈1280×720×3×6 ≈ 16.6 MB into **two** double-buffered slots every tick the images change — at 30 Hz, ~1 GB/s of CPU memcpy for staging *alone*, before any GPU upload, against a buffer class whose largest current occupant (OGM cells) is a couple of small single-channel grids (`ground_grid.cpp:157-161`). Rejected: wrong entry point for this size class.
- **Option C (keep the CUDA reprojector, feed Filament one composited RGBA image)** sidesteps the whole texture-upload question but does not migrate anything — it keeps CUDA in the loop under a different label. Rejected outright: it fails the user's literal charter ("everything use the same rendering engine").
- **Option B (`set_camera_frame(VisualRenderer*, uint32_t cam_idx, const uint8_t* rgb, uint32_t width, uint32_t height)`)** is the same shape as `set_ego_model()` (`ego.cpp:106-186`) and `set_environment_source()` (Epic 4 Decision 2) — a POD entry point outside `SceneGraph`, for content that doesn't belong in the per-tick deep-copied buffer. It pushes straight to a persistent `filament::Texture` via `Texture::Builder()...setImage()` — the exact call `ground_grid.cpp:138-161` already uses for OGM textures — with **per-camera dirty tracking** (skip `setImage()` when a camera's frame hasn't changed since last upload) ported directly from `img_dirty_` (`rendering_node.cpp:769-786`). **Decision: Option B.**

**The one thing genuinely left open (flagged, not silently resolved, per the task instructions):** the historical GL renderer's own regression (`docs/superpowers/specs/2026-06-23-gl-optimization-design.md:11-24`) shows a ~35% fps hit from *naive per-frame reupload of all 6 camera textures*, on the same GPU class this project targets — before any bowl-projection cost is even added. Dirty-tracking bounds the *reupload* cost (a camera whose image hasn't changed this tick costs nothing), but says nothing about the *projection* cost once 6 live textures ARE dirty every tick during normal driving. **Task 2's Step 5 perf gate is the first real measurement of this** — nobody has run it, on either box (research §5). If that gate misses budget, the fallback is coarser per-vertex UV baking (larger dirty-skip windows, lower bowl tessellation) before falling back to Option C's "cheat," which stays available but unused unless Task 2's own review gate calls for it explicitly. **This tradeoff — vertex-baked UV/blend (this plan's design, §3 below) vs. true per-pixel shader-side backward sampling closer to CUDA's exact fidelity — is the one open item this plan marks as a USER DECISION at the Task 2 review gate**, because the research's own honest-unknown #5 (distortion handling) was never resolved and a visual-fidelity-vs-complexity call belongs to the person who judges the bowl by its visuals (per this project's documented working style).

### 3. Bowl rendering mechanism: vertex-baked per-camera UV + blend weight, GPU does the multi-texture blend

The CUDA kernel (`reproject.cu`) is a **backward** sampler: for every output pixel on the bowl surface, compute which camera(s) see it and sample their (distorted) pixel. A Filament rasterizer naturally wants the **forward** direction instead: for every bowl-mesh **vertex**, compute its UV in each camera's (distorted) image once, on the CPU, whenever bowl config changes (extrinsics/intrinsics/radius params — live-tunable via the GUI bridge, `tools/vcam_ws_bridge.py:62-67`, but not changing every tick), bake those UVs + a per-camera blend weight (feather-margin-based, matching `blend.cuh`'s existing feather math) as vertex attributes, and let the fragment shader sample up to N camera textures per vertex-interpolated UV and blend by weight. This is coarser than CUDA's per-pixel backward sample (fidelity bounded by bowl-mesh tessellation density, an implementer's call tuned against the golden comparison), but it is the shape every other GPU-rasterized texture-projection problem takes, and it entirely avoids doing lens-distortion math inside a fragment shader (an unresearched, higher-risk path per research §7.5). The camera projection + plumb_bob distortion formula itself is ported to portable host C++ once (Task 2 Step 0) — reusable by the vertex baker (Task 2) and the lidar colorization adapter (Task 5), one implementation, two call sites, matching this codebase's own "two small copies, one per toolchain" precedent (Epic 4 Decision 5) — except here it is the **same** toolchain (host C++) on both sides, so it is one copy, not two.

### 4. Self-view masks + robot-proxy compositing: delete the kernels, don't port them

Research §1.11 already names the finding: robot-proxy compositing (`robot_raster.cu`, a hand-written z-buffer triangle rasterizer) does exactly what Filament's own scene renderer already does for free. The ego glTF mesh is **already** a Filament-native renderable (`set_ego_model()`, mode 3, shipped since Epic 1) — adding it to the **same** Filament `Scene` as the bowl mesh (Task 2) means Filament's own depth test composites robot-over-bowl correctly with zero new code. Self-view masking (hiding the robot's own body from smearing onto the bowl surface as seen by each real camera) is not a rasterizer problem either — it is "for each camera, don't trust bowl-vertex UV samples that the ego mesh would occlude from that camera's vantage." Decision: extend Task 2's per-vertex UV/weight bake with an analytic occlusion test (ego bounding box vs. the camera-to-vertex ray) rather than a full per-camera depth-render pass — a deliberate fidelity/complexity simplification, named as such, with the golden comparison as the check that catches it if the box-test proves visibly wrong (Task 3's own review gate item).

### 5. Mode 2 needs **zero** new library rendering code

Research §2's finding, restated as the actual design: mode 2's only genuinely new behavior versus mode 1 is colorizing lidar points from the synchronized camera images. That colorization is host-side math (project each lidar point into whichever camera's FOV covers it, using the **same** portable projection function Decision 3 ports in Task 2 Step 0, sample that camera's already-ingested raw image) producing exactly `mpviz::PointCloudPoint{position, rgba}` — the type **already in `scene.h`**, already rendered by the **already-shipped** `PointCloud` layer (VM-035, Epic 3 Task 6), and already reused verbatim once before for an unrelated producer (`TrajectoryCarpet` reusing `PointCloudPoint` per VM-077, `scene.h:184-196`). Mode 2 = mode 1's bowl (fallback, unchanged) + a new node-side adapter feeding an existing `SceneGraph::point_clouds` row. `splat_radius` maps to `point_cloud.mat`'s existing per-point size uniform. No new `SceneGraph` category, no new entry point beyond what Task 2 already added.

### 6. Node consolidation target: `micropilot_visualization_node`, unchanged name

Rung 2 of the standard ladder — an existing thing already does most of this job. `micropilot_visualization_node` already hosts the Filament engine, mode 3, the vcam surface (`~/set_virtual_cam`/`~/set_look`/`~/vcam_state`, re-implemented from `micropilot_rendering_node` per ADR-0002 with identical semantics), and the `layer_*`/quality-preset plumbing (VM-032) that mode 1/2's "which overlays show in which mode" question can reuse directly instead of inventing a second gating mechanism. `micropilot_rendering_node` is decommissioned at Task 6, not renamed or repurposed.

### 7. The mux dies LAST — compatibility/cutover strategy, stated as a mechanism, not a slogan

Both processes and the **entire current mux protocol** (ADR-0002, `/rendering/set_mode`, VM-037's now-fully-shipped hardening) stay **exactly as they are today**, unmodified, through Tasks 1–5. Task 4 gives the new node the local *capability* to answer modes 1/2/3 from one process, but does **not** make it authoritative for 1/2 in the deployed mux — the OLD node keeps being selected for modes 1/2 in production, and the mux keeps arbitrating, until parity sign-off. This means the entire migration is judged with a real side-by-side flip using the **existing, already-battle-tested** mux mechanism as the safety net — genuinely lower risk than cutting over blind, and it costs nothing extra to build since ADR-0002's mechanism already exists and already works. Only Task 6 — gated on Tasks 2/3/5's parity checklists all passing — deletes `pub_mode_mux_`, `mux_mode_sub_`, the legacy `~/set_render_mode` republish shim, `initial_mode` race handling, and VM-037's transient_local QoS hardening, and decommissions `micropilot_rendering_node` outright. **The mux is deleted last, by construction of the task order, not by a note asking the implementer to remember that.**

### 8. `~/vcam_state`'s 9th element (`mux_mode`, VM-037 Step (d)) is kept, not unwound

Shrinking the wire format back to 8 elements would ripple every consumer that now expects `len(vcam_state) >= 9` (`tools/vcam_ws_bridge.py`, `test_vcam_contract.py`, `test_ego_anchored_vcam.py`) for a value that becomes meaningless the instant there is only one process. Decision: index 8 stays, permanently mirrors index 7 (`active_mode_`) once Task 6 lands (documented at the publish site, same "vestigial, named, not silently left" honesty Epic 4 gave `odom`) — reuse over removal, matching the ladder's "deletion over addition" *only when deletion doesn't cost more than it saves*.

### 9. Epic 4 (buildings) interaction — recommendation: **proceed**, not park, with one named file-conflict risk

EnvironmentLayer (Epic 4) is renderer-internal, not a `SceneGraph` category (Epic 4 Decision 1) — it renders whenever `environment_enabled` is true and a valid ego position exists, entirely independent of which "mode" is selected or which node hosts `visual_renderer`. Epic 4 Tasks 2–3 (VM-051 bake script, VM-052 runtime load) touch no file this epic's Tasks 1–3 touch. **Recommendation: Epic 4 Tasks 2–3 proceed on their own schedule, unblocked by this epic.** The one real risk is not architectural, it's a merge hazard: Epic 4 Task 3 and this epic's Task 4 **both** modify `micropilot_visualization_node/src/visualization_node.cpp`'s `on_activate()`/param wiring. Whichever lands first, the other rebases onto it (same "sequential tasks against a green tree" convention Epic 4's own status ledger used for the vm077 redirect) — this is a scheduling call the user makes at approval, not a technical blocker either way. Once both land, the merged node's bowl view and its buildings coexist in the same `render_frame()` the same way mode 3's buildings and autonomy overlays already do — no new interaction to design.

### 10. Epic 5/6 interaction

Epic 5's VM-043 ("on-robot budget rerun") and the CUDA-node-co-residence half of its own sign-off checklist (`budget_probe.md`'s `q1_rnode_idle`/`q1_rnode_mode2` rows) become **moot, not harder** once this epic's Task 6 lands — there is no second process to co-reside with. Recommendation: Epic 5's VM-043 on-robot rerun is superseded by **this epic's Task 6 perf gate**, which measures the same thing (one process, on-robot, real load) without a now-nonexistent second axis; VM-041 ("repo-local CI gate") and VM-042 ("docs/runbooks") are unaffected and proceed independently. Epic 6 (3D Tiles streaming, `EnvironmentSource` seam) is renderer-internal exactly like Epic 4's baked backend (Epic 4 Decision 2: "renderer code unchanged" between backends) and has zero interaction with this epic's camera/bowl work — proceeds on its own schedule, unaffected.

---

## Named fixture gaps (this epic)

1. **No on-robot GPU model/class is stated anywhere in the repo** (research §7.1) — every perf number in Tasks 2/3/4/5 stays dev-box-proxy-only; Task 6 is this epic's first and only on-robot measurement, and it is also the first on-robot measurement Epic 0/5 ever got (closing VM-043, not just this epic's own gate).
2. **CUDA-node/Filament co-residence was never measured even for today's two-node design** (`budget_probe.md:12-13`) — Task 4's perf gate is the first real number for "old CUDA node + new merged node running together," which is the actual state production is in for the whole rollout window between Task 4 and Task 6.
3. **The old GL renderer's ~35% reupload-cost regression** (`2026-06-23-gl-optimization-design.md:11-24`) is the closest existing data point to "what does 6 camera textures cost Filament" and is explicitly not a Filament-specific measurement — Task 2 Step 5 is the first one.
4. **Mode 2's camera-colorization accuracy against the CUDA reference has no prior art** — Task 5's own golden is the first time this gets checked at all, not merely re-checked.

---

## Task 1 (VM-090): POD-boundary camera-texture mechanism (ADR-0005)

**Files:**
- Modify: `cuda/src/libs/visual_renderer/include/visual_renderer/scene.h` (append `CameraExtrinsics`, `CameraIntrinsics`, `BowlConfig` structs; append `set_camera_frame()`, `set_bowl_config()` free functions; bump `kSceneVersion` 4 → 5)
- Modify: `cuda/src/libs/visual_renderer/tests/test_scene_buffer.cpp` (bump `static_assert(kSceneVersion == 4, ...)` to `== 5`; add `sizeof`/`offsetof` asserts for the three new structs)
- Modify: `cuda/src/ros_apps/src/micropilot_visualization_node/test/test_scene_layout.cpp` (same bump + mirrored asserts)
- Create: `cuda/src/libs/visual_renderer/src/camera_textures.hpp` / `.cpp` (library-internal — owns the 6 persistent `filament::Texture*` + per-camera dirty flags, mirrors `renderer_internal.hpp`'s "never `#include`d outside the library" status)
- Create: `cuda/src/libs/visual_renderer/tests/test_camera_textures.cpp`
- Create: `docs/adr/0005-camera-frame-pod-boundary.md`

**Interfaces:**
```c
// scene.h, additive per ADR-0004.
struct CameraExtrinsics { double R[9]; double t[3]; };            // row-major, rig frame
struct CameraIntrinsics { double fx, fy, cx, cy; double dist[5]; };// plumb_bob, ported verbatim
                                                                     // from micropilot::rendering::CameraParams
                                                                     // (types.hpp:8-16)

// Non-SceneGraph config, same class as RenderConfig -- rarely changes
// (live-tunable via the GUI bridge, not per-tick), so it is NOT deep-copied
// by set_scene()/SceneBuffer. Arrays are caller-owned for the duration of
// the call only; set_bowl_config() copies what it needs into the renderer's
// own storage before returning (same contract as RenderConfig's
// theme_assets_dir/initial_theme, api.h:24-33), then (re)bakes the bowl
// mesh + per-vertex UV/weight attributes (Decision 3) -- an expensive,
// rare operation, never called per-tick.
struct BowlConfig {
    uint32_t camera_count;                 // <= kMaxBowlCameras (6)
    const CameraExtrinsics* extrinsics;     // camera_count entries
    const CameraIntrinsics* intrinsics;     // camera_count entries
    const uint32_t* cam_width;              // camera_count entries, pixels
    const uint32_t* cam_height;             // camera_count entries, pixels
    double bowl_R0, bowl_k, bowl_Rmax;      // bowl surface shape (types.hpp:19-38 parity)
    double feather_margin;
    uint8_t fill_blind_zone;
    uint8_t exposure_match;
    float sky_color[3];
};
// Rebuilds the bowl mesh + camera textures + per-vertex UV/weight bake.
// false (no-op, WARN once) if r is null, camera_count == 0, or
// camera_count > kMaxBowlCameras -- same "missing/invalid config renders
// nothing" convention as set_environment_source's null-path (spec Sec.9
// precedent).
bool set_bowl_config(VisualRenderer*, const BowlConfig&);

// Per-tick (or per-camera-frame-arrival) call. Dirty-tracked internally
// (Decision 2): a call with byte-identical `rgb` content to the last
// uploaded frame for this cam_idx is a no-op past a cheap size/pointer
// check -- callers are NOT required to pre-filter; the node calls this
// every tick a camera image arrived, mirroring rendering_node.cpp's own
// img_dirty_ gate (rendering_node.cpp:769-786), now inside the library
// instead of the node. false if r is null, cam_idx >= the camera_count
// set_bowl_config() configured, or width/height mismatch the configured
// camera's dims.
bool set_camera_frame(VisualRenderer*, uint32_t cam_idx,
                       const uint8_t* rgb, uint32_t width, uint32_t height);
```
- Consumes: nothing from earlier tasks (this is the foundation task).
- Produces: `CameraExtrinsics`, `CameraIntrinsics`, `BowlConfig`, `set_bowl_config()`, `set_camera_frame()` — Task 2 is the first real consumer.

- [ ] **Step 0: Failing test — `set_camera_frame()` on an unconfigured renderer is a safe no-op.**
```cpp
TEST(CameraTextures, SetCameraFrameBeforeBowlConfigIsNonFatal) {
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP();
    std::vector<uint8_t> pixels(64 * 48 * 3, 128);
    EXPECT_FALSE(mpviz::set_camera_frame(r, 0, pixels.data(), 64, 48));  // no BowlConfig yet
    mpviz::destroy_renderer(r);
}
```
  Run — FAIL (`set_camera_frame` doesn't exist). Implement the free function + `camera_textures.hpp`'s `CameraTextureSet` skeleton (holds `camera_count = 0` until `set_bowl_config()` runs; `set_camera_frame` checks `cam_idx < camera_count` and returns `false` otherwise). Run — PASS.

- [ ] **Step 1: Failing test — `set_bowl_config()` allocates persistent textures; `set_camera_frame()` uploads into them.**
```cpp
TEST(CameraTextures, SetCameraFrameUploadsAfterBowlConfig) {
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP();
    mpviz::CameraExtrinsics ext[1] = {{{1,0,0, 0,1,0, 0,0,1}, {0,0,0.5}}};
    mpviz::CameraIntrinsics in[1] = {{400,400,160,120,{0,0,0,0,0}}};
    uint32_t w[1] = {320}, h[1] = {240};
    mpviz::BowlConfig bc{1, ext, in, w, h, 0.5, 1.2, 8.0, 0.15, 1, 1, {0.5f,0.7f,0.9f}};
    ASSERT_TRUE(mpviz::set_bowl_config(r, bc));
    std::vector<uint8_t> pixels(320 * 240 * 3, 200);
    EXPECT_TRUE(mpviz::set_camera_frame(r, 0, pixels.data(), 320, 240));
    EXPECT_FALSE(mpviz::set_camera_frame(r, 1, pixels.data(), 320, 240));  // cam_idx out of range
    mpviz::destroy_renderer(r);
}
```
  Run — FAIL then PASS. Implement `set_bowl_config()`: allocate `camera_count` persistent RGB8 `filament::Texture`s (`Texture::Builder().width(w).height(h).format(RGB8).sampler(SAMPLER_2D).build(engine)`, same builder shape as `ground_grid.cpp:138-145`'s R8 occupancy texture, three channels instead of one), store per-camera extrinsics/intrinsics/dims copied into owned storage. `set_camera_frame()` calls `tex->setImage()` (same `PixelBufferDescriptor` heap-copy pattern `ground_grid.cpp:149-161` already uses) only when `cam_idx` is in range and dims match.

- [ ] **Step 2: Failing test — dirty tracking skips a byte-identical re-upload.** Expose a test-only upload counter via a hooks header (`camera_textures_test_hooks.hpp`, same shape as `environment_test_hooks.hpp`/`map_elements_test_hooks.hpp` — internal-only, not installed, not POD, declares `uint64_t camera_frame_upload_count(mpviz::VisualRenderer*, uint32_t cam_idx)`).
```cpp
TEST(CameraTextures, ByteIdenticalFrameSkipsReupload) {
    // ... same setup as Step 1 ...
    mpviz::set_camera_frame(r, 0, pixels.data(), 320, 240);
    auto count_after_first = mpviz::testing::camera_frame_upload_count(r, 0);
    mpviz::set_camera_frame(r, 0, pixels.data(), 320, 240);  // identical content
    EXPECT_EQ(mpviz::testing::camera_frame_upload_count(r, 0), count_after_first);  // no re-upload
    std::vector<uint8_t> changed(pixels); changed[0] = 201;
    mpviz::set_camera_frame(r, 0, changed.data(), 320, 240);
    EXPECT_EQ(mpviz::testing::camera_frame_upload_count(r, 0), count_after_first + 1);  // real change -> uploads
}
```
  Run — FAIL then PASS. Implement: a cheap `memcmp` against the last-uploaded buffer (not a hash — at 320×240×3 this is a bounded, already-in-cache comparison, cheaper than composing a hash) gates the `setImage()` call. **Named simplification (ponytail):** this compares full pixel content rather than trusting an upstream "this camera published a new message" signal the way `rendering_node.cpp`'s `img_dirty_` does (set on message arrival, not content) — that upstream signal doesn't exist yet at the library boundary (the node hasn't been wired, Task 2), so content-comparison is the correct check *here*; Task 2's node-side wiring is expected to ALSO gate on message-arrival (cheaper, no memcmp) before ever calling `set_camera_frame()`, making this library-side check a defense-in-depth backstop, not the only gate — stated explicitly so a future reader doesn't think one gate is redundant with the other.

- [ ] **Step 3: `kSceneVersion` bump + layout asserts.** Append `CameraExtrinsics`/`CameraIntrinsics`/`BowlConfig` to `scene.h` per the Interfaces block above; bump `kSceneVersion` 4 → 5; add `sizeof`/`offsetof` `static_assert`s to `tests/test_scene_buffer.cpp` for all three new structs; mirror in `test_scene_layout.cpp`. Run both suites — PASS.

- [ ] **Step 4: Write ADR-0005.** `docs/adr/0005-camera-frame-pod-boundary.md`, recording Decision 2 verbatim (Option B chosen over A/C, with the reupload-cost open item named as the one thing Task 2's review gate must resolve, not this ADR).

- [ ] **Step 5: Commit** `feat(visual): set_camera_frame/set_bowl_config POD-boundary mechanism (VM-090, ADR-0005)`.

---

## Task 2 (VM-091): Mode 1 (bowl) migration into `micropilot_visualization_node`

**Files:**
- Create: `cuda/src/libs/visual_renderer/src/bowl_projection.hpp` / `.cpp` (portable — no Filament/GL types; pinhole + plumb_bob project/unproject, ported from `micropilot::rendering::CameraParams`/`reproject.cu`'s own formula, read in full before porting)
- Create: `cuda/src/libs/visual_renderer/tests/test_bowl_projection.cpp`
- Create: `cuda/src/libs/visual_renderer/src/bowl.hpp` / `.cpp` (library-internal — bowl mesh generator from `BowlConfig`'s `bowl_R0/k/Rmax`, per-vertex per-camera UV+weight bake using `bowl_projection`, Filament entity/material wiring)
- Create: `cuda/src/libs/visual_renderer/tests/test_bowl.cpp`
- Create: `cuda/src/libs/visual_renderer/assets/materials/bowl.mat` (new material — up to `kMaxBowlCameras` texture samplers + per-vertex UV/weight attributes; **this IS a new `.mat` file, unlike Epic 4's buildings — the materials glob has no `CONFIGURE_DEPENDS` (`CMakeLists.txt:127`), so this task's CMakeLists.txt touch below is required, not optional**)
- Modify: `cuda/src/libs/visual_renderer/CMakeLists.txt` (the materials glob is evaluated at configure time only — a fresh `cmake` re-run picks up the new `.mat`; note this explicitly in the PR so a reviewer doesn't assume the CONFIGURE_DEPENDS trap was silently worked around)
- Modify: `cuda/src/libs/visual_renderer/src/renderer_internal.hpp` (`VisualRenderer` gains `std::unique_ptr<BowlState> bowl;` — null until `set_bowl_config()` runs)
- Modify: `cuda/src/libs/visual_renderer/src/renderer.cpp` (`render_frame()` gains one call, `update_bowl(*r)`, gated on `r->bowl != nullptr`, slotted after the existing category updates per the file's own `r->scene_buffer.active()` re-derive-every-call pattern)
- Create: `cuda/src/ros_apps/src/micropilot_visualization_node/include/micropilot_visualization_node/camera_ingest.hpp`
- Create: `cuda/src/ros_apps/src/micropilot_visualization_node/src/camera_ingest.cpp` (6× `SensorDataQoS` image/info subs, frame-sync gate, ego-motion time compensation — ported from `rendering_node.hpp:152-176`/`.cpp:392-446,562-703`, all already-portable per research §1.1/1.3/1.9, re-read in full before porting)
- Create: `cuda/src/ros_apps/src/micropilot_visualization_node/test/test_camera_ingest.cpp`
- Modify: `cuda/src/ros_apps/src/micropilot_visualization_node/CMakeLists.txt` (hand-written source list + relevant `ament_add_gtest` targets — the trap named in Global Constraints)
- Modify: `cuda/src/ros_apps/src/micropilot_visualization_node/config/default_params.yaml` (append `bowl_enabled: false` — default OFF until parity sign-off, per the STANDING directive's disable-knob rule; `bowl_R0`/`bowl_k`/`bowl_Rmax`/`feather_margin`/`fill_blind_zone`/`exposure_match`/`sky_color`/`camera_extrinsics`/`camera_intrinsics`, same param shapes `rendering_node.cpp:158-205` already declares, ported not reinvented)
- Modify: `cuda/src/ros_apps/src/micropilot_visualization_node/src/visualization_node.cpp` (own a `CameraIngest`; on `bowl_enabled` + all params valid, call `set_bowl_config()` once at `on_activate()`, then `set_camera_frame()` per arriving, dirty camera image)

**Interfaces:**
```cpp
// bowl_projection.hpp -- pure math, no ROS/Filament, GPU-free-testable.
namespace mpviz::bowl {
// World (map-frame) point -> this camera's pixel UV [0,1]^2, applying
// plumb_bob distortion forward. Returns false if the point is behind the
// camera or outside its FOV (caller then applies zero blend weight for
// that vertex/camera pair -- Decision 3/4).
bool ProjectToCameraUv(const mpviz::CameraExtrinsics&, const mpviz::CameraIntrinsics&,
                       uint32_t width, uint32_t height,
                       mpviz::Vec3 world_point, float* out_u, float* out_v);
// Bowl surface point at (theta, r) per bowl_R0/k/Rmax -- ported verbatim
// from reproject.cu's own analytic surface (types.hpp:19-38 parity).
mpviz::Vec3 BowlSurfacePoint(double bowl_R0, double bowl_k, double bowl_Rmax,
                             double theta, double r);
}
```
- Consumes: `CameraExtrinsics`/`CameraIntrinsics`/`BowlConfig`/`set_camera_frame`/`set_bowl_config` (Task 1).
- Produces: `bowl::ProjectToCameraUv`/`BowlSurfacePoint` — Task 5's lidar-colorization adapter reuses `ProjectToCameraUv` directly (Decision 3/5).

- [ ] **Step 0: Failing test — `ProjectToCameraUv` round-trips a known point through a plain pinhole (zero distortion) camera.**
```cpp
TEST(BowlProjection, PinholeCenterPointProjectsToImageCenter) {
    CameraExtrinsics ext{{1,0,0, 0,1,0, 0,0,1}, {0,0,0}};  // identity, camera at origin
    CameraIntrinsics in{400,400,160,120,{0,0,0,0,0}};       // zero distortion
    float u, v;
    // A point straight ahead on the camera's optical axis, at some depth,
    // must project to the principal point (cx,cy) normalized -> (0.5,0.5).
    ASSERT_TRUE(bowl::ProjectToCameraUv(ext, in, 320, 240, {0, 0, 5}, &u, &v));
    EXPECT_NEAR(u, 0.5f, 1e-3f);
    EXPECT_NEAR(v, 0.5f, 1e-3f);
}
TEST(BowlProjection, PointBehindCameraFailsProjection) {
    CameraExtrinsics ext{{1,0,0, 0,1,0, 0,0,1}, {0,0,0}};
    CameraIntrinsics in{400,400,160,120,{0,0,0,0,0}};
    float u, v;
    EXPECT_FALSE(bowl::ProjectToCameraUv(ext, in, 320, 240, {0, 0, -5}, &u, &v));
}
```
  Run — FAIL (`bowl_projection.hpp` doesn't exist). Port the pinhole projection (world → camera frame via `R`/`t` → normalized image plane → `plumb_bob` forward distortion → pixel → UV) from `reproject.cu`'s own math, read in full first — **do not re-derive the distortion formula from a generic reference; use this codebase's own formula so Task 5's colorization stays bit-consistent with what the bowl itself does.** Run — PASS.

- [ ] **Step 1: Failing test — nonzero plumb_bob distortion pins a known off-axis case.** Pick one real `dist` row from `default_params.yaml` (or `m2o1_params.yaml`) and one hand-computed expected UV at a known world point (computed independently via the standard plumb_bob formula, not by calling the function under test and asserting it agrees with itself). Run — FAIL then PASS.

- [ ] **Step 2: Failing test — `BowlSurfacePoint` matches `reproject.cu`'s surface at `r=bowl_R0` and `r=bowl_Rmax`.** Two boundary-condition assertions (e.g., at `r=bowl_R0` the surface point's height matches the flat-floor case; at `r=bowl_Rmax` it matches the wall-rise case) derived directly from `types.hpp:19-38`'s own documented parameters, not invented. Run — FAIL then PASS.

- [ ] **Step 3: Failing test — bowl mesh generation + per-vertex UV/weight bake, single camera, no GPU needed.** A Filament-free unit test on `bowl.hpp`'s mesh-builder function in isolation (extract the CPU-only bake step as a separately-callable function from the Filament-entity-creation step, same "separate the math from the GPU wiring" shape Step 0 already established): build a small bowl mesh (low tessellation, test-sized), bake UV/weight for one configured camera, assert a vertex known to be in that camera's FOV gets a nonzero weight and a UV inside `[0,1]^2`, and a vertex known to be behind the bowl (opposite side) gets zero weight. Run — FAIL then PASS.

- [ ] **Step 4: Failing test — `set_bowl_config()` + `set_camera_frame()` + `render_frame()` produces a non-crashing, non-black frame.**
```cpp
TEST(Bowl, RenderFrameWithBowlConfiguredProducesNonBlackPixels) {
    mpviz::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    mpviz::CameraPose pose{{0, 0, 8}, {0, 0, 0}, 90};  // looking straight down at the bowl
    auto* r = mpviz::create_renderer(cfg);
    if (!r) GTEST_SKIP();
    /* configure a 1-camera BowlConfig as in Task 1 Step 1 */
    std::vector<uint8_t> cam_pixels(320 * 240 * 3, 200);  // flat gray test pattern
    mpviz::set_camera_frame(r, 0, cam_pixels.data(), 320, 240);
    std::vector<uint8_t> buf(320 * 240 * 3);
    ASSERT_TRUE(mpviz::render_frame(r, pose, {buf.data(), 320, 240}));
    // At least some pixels should be near the test-pattern gray (200), not
    // all zero -- proves the bowl actually rasterized and sampled the
    // uploaded texture, not just "didn't crash."
    size_t near_gray = 0;
    for (size_t i = 0; i < buf.size(); i += 3) if (std::abs(int(buf[i]) - 200) < 40) ++near_gray;
    EXPECT_GT(near_gray, buf.size() / 3 / 10);  // >10% of pixels
    mpviz::destroy_renderer(r);
}
```
  Run — FAIL then PASS. Implement `bowl.cpp`'s Filament wiring: build the `bowl.mat` `MaterialInstance`, create the mesh entity with UV/weight vertex attributes (Decision 3), bind the `camera_textures.hpp` textures as samplers, add the entity to `r->scene`. Run existing full library suite — everything untouched stays green (`bowl_enabled`-gated code path, no other category touched).

- [ ] **Step 5: Perf gate.** Measure `render_ms` (VM-034 instrumentation, already exists) with the bowl configured at real 6-camera dims/tessellation vs. bowl disabled, dev-box proxy, same `budget_probe.sh` procedure `budget_probe.md` already documents (extend it with a `bowl_on`/`bowl_off` case pair). Record the delta against the ~35%-reupload-regression ceiling named in Decision 2 as the honest reference point (not a hard bar this task invented) — if the delta is inside a reasonable margin of that ceiling, proceed; if it blows past it, this is the review-gate finding that triggers the vertex-baking-vs-shader-sampling USER DECISION named in Decision 2, not a silent tessellation tweak.

- [ ] **Step 6: Node wiring — 6-camera ingest, dirty-gated `set_camera_frame()` calls, `bowl_enabled` disable knob.** Port `camera_ingest.cpp` from `rendering_node.hpp:152-176`/`.cpp:392-446,562-703` (subs, frame-sync gate, ego-motion compensation) — read those exact line ranges in full before porting, do not re-derive from memory. Wire `visualization_node.cpp`: `on_activate()` calls `set_bowl_config()` once from params iff `bowl_enabled_`; the per-camera-image callback calls `set_camera_frame()` gated on "this callback fired" (the cheap upstream dirty signal named in Task 1 Step 2's note) — not merely relying on Task 1's internal memcmp backstop. `bowl_enabled: false` (the shipped default here) means `set_bowl_config()` is never called and the bowl step in `render_frame()` stays a no-op by construction (Decision 2/Task 1's null-config path) — **this task ships the capability, not the cutover**; the old `micropilot_rendering_node` remains the one actually serving modes 1/2 in the live mux (Decision 7).

- [ ] **Step 7: Golden — visual sanity check against the CUDA node's own output.** One capture (`bowl_test_town_dark_adas.png` or similar), same fixture bag frame, human-sanity-approved (not a pixel diff — this samples through a genuinely different mechanism than the CUDA backward sampler, per Decision 3), per the Golden scoping rule in Global Constraints.

- [ ] **Step 8: Commit** `feat(visual): mode-1 bowl migrated to Filament, camera ingest ported, bowl_enabled default off (VM-091)`.

---

## Task 3 (VM-092): Self-view masks + robot-proxy compositing (Filament-native)

**Files:**
- Modify: `cuda/src/libs/visual_renderer/src/bowl.cpp` (extend the per-vertex UV/weight bake with the ego-occlusion test, Decision 4)
- Modify: `cuda/src/libs/visual_renderer/src/renderer.cpp` (ensure the ego entity — already created by `set_ego_model()` — is added to the SAME `filament::Scene` the bowl entity lives in; no new entity-creation code, only confirming/wiring shared scene membership)
- Modify: `cuda/src/libs/visual_renderer/tests/test_bowl.cpp` (new cases below)
- Modify: `cuda/src/ros_apps/src/micropilot_visualization_node/config/default_params.yaml` (append `self_view_masks_enabled: true` — matches the CUDA node's own default, `rendering_node.cpp:80-84`)

**Interfaces:** none new — consumes `set_ego_model` (existing), `bowl.hpp`'s internal per-vertex bake (Task 2).

- [ ] **Step 0: Failing test — robot-proxy compositing needs no new code, only shared scene membership.**
```cpp
TEST(Bowl, EgoMeshOccludesBowlSurfaceBehindIt) {
    // Configure a 1-camera bowl (as Task 2 Step 4) AND call set_ego_model()
    // with a known-size fallback box (build_ego_fallback path, no glTF
    // file needed -- ego.cpp:111). Render from a pose that looks through
    // where the ego box sits toward the bowl surface beyond it. Assert the
    // rendered pixel at the ego's known screen position is the ego's own
    // fallback color/material, NOT the bowl's camera-texture color --
    // proving Filament's own depth test composited them correctly with
    // zero bowl-specific code.
}
```
  Run — this should PASS immediately if Task 2's bowl entity and the existing ego entity already share `r->scene` and both have real depth-tested geometry (no culling override on either) — if it FAILS, the fix is removing whatever accidentally excludes one entity from the other's depth test (e.g., a stray `culling(false)` interaction), not writing a new compositing pass. Document whichever outcome occurred.

- [ ] **Step 1: Failing test — self-view mask lowers a vertex's per-camera weight when the ego mesh would occlude that camera's view of it.**
```cpp
TEST(Bowl, VertexOccludedByEgoBoxGetsZeroWeightForThatCamera) {
    // A bowl vertex directly behind a known ego bounding box, as seen from
    // camera 0's known pose, should get weight 0 for camera 0 specifically
    // (other cameras, if configured, unaffected) after the bake -- proving
    // the analytic occlusion test (Decision 4) actually suppresses the
    // self-view smear the CUDA node's mask pass existed to prevent.
}
```
  Run — FAIL then PASS. Implement the analytic bbox-vs-ray occlusion test in `bowl.cpp`'s bake step, using the ego's known fallback/glTF bounding dimensions (already available wherever `set_ego_model()`/`build_ego_fallback()` stores them).

- [ ] **Step 2: `self_view_masks_enabled` disable knob.** Failing test: with the knob false, the occlusion test from Step 1 is skipped (weight stays whatever Decision 3's camera-visibility bake alone produced) — a real, checkable behavior difference, not a no-op flag. Run — FAIL then PASS.

- [ ] **Step 3: Golden — visual check that the box-test approximation doesn't produce a visibly wrong seam** around the ego's silhouette, one capture, human-sanity-approved (Decision 4 names this as the fallback trigger if it looks wrong: swap to a real per-camera depth-render pass instead of the bbox approximation — a review-gate finding, not silently absorbed).

- [ ] **Step 4: Commit** `feat(visual): self-view masks + robot-proxy compositing via shared Filament scene, no new rasterizer (VM-092)`.

---

## Task 4 (VM-093): Node consolidation — local mode switch, camera ingest live, mux UNCHANGED and still authoritative

**Files:**
- Modify: `cuda/src/ros_apps/src/micropilot_visualization_node/src/visualization_node.cpp` (own a local `render_mode_` enum {BOWL=1, HYBRID=2, FREE_LOOK=3}; the per-tick render call dispatches on it — bowl/hybrid paths call into Task 2/5's code, `FREE_LOOK` is the existing mode-3 path, untouched)
- Modify: `cuda/src/ros_apps/src/micropilot_visualization_node/launch/*.launch.py` (or create `visualization_node_unified.launch.py` — carries forward the CycloneDDS SHM-forcing QoS config from `rendering_node.launch.py:36-45`, since this node now also subscribes to 6 raw camera topics)
- Modify: `cuda/src/ros_apps/src/micropilot_visualization_node/test/smoke_test.py` (this package already has one, separate from `micropilot_rendering_node/test/smoke_test.py`) — new cases for the LOCAL mode switch (no topic involved): launch the merged node, drive its local `render_mode_` through 1→2→3→1 via whatever param/service surface Step 0 below defines, assert frames of the right shape and no crash at each mode
- **Explicitly NOT modified in this task:** `/rendering/set_mode` mux code on EITHER node, `pub_mode_mux_`, `mux_mode_sub_`, the legacy `~/set_render_mode` shim, VM-037's transient_local QoS hardening — Decision 7 requires all of it to keep working exactly as today through this task

**Interfaces:** none new at the `scene.h`/POD level — this task is node-internal wiring only.

- [ ] **Step 0: Failing test — the merged node's local mode enum switches which render path executes, independent of the global mux.** A node-internal test (or smoke-test case, whichever harness this package already supports per the precedent named in Epic 4 Task 3 Step 2/3's own honesty about node-test-harness gaps — checked at implementation time) that sets the local mode to BOWL and asserts the bowl path renders (reusing Task 2's non-black-pixel check), then to FREE_LOOK and asserts the existing mode-3 autonomy scene renders, with NO ROS message exchanged for either switch.
  Run — FAIL then PASS. Implement the local enum + a dispatch in the node's per-tick render call. **Do not touch the existing `/rendering/set_mode` handling at all** — this local enum is a SEPARATE piece of state, for now, that nothing in production reads yet.

- [ ] **Step 1: Parity checklist opened.** Create/append to `docs/visual_mode/signoff.md` (VM-043's existing file, per the master plan's Epic 5 row) a new section: "Unified-engine parity sign-off (blocks Task 6)" — bowl-vs-CUDA visual parity (owner: Task 2's golden + a live side-by-side), hybrid-vs-CUDA visual parity (owner: Task 5's golden), self-view/robot-proxy parity (owner: Task 3's golden), perf parity at each milestone (owner: this epic's own perf-gate steps). This is the gate Task 6 checks before deleting anything.

- [ ] **Step 2: CycloneDDS SHM launch config carried forward.** Verify (by actually launching the merged node with 6 camera topics against a CARLA/bag source, same procedure the original SHM fix commit `69b5a3a` used to reproduce the FPS collapse) that the merged node's launch file forces the shared-memory transport identically to `rendering_node.launch.py:36-45` — this is a transport fact, unrelated to which process subscribes, and it is trivial to silently lose when copying launch files. Record the measured sim FPS with vs. without the SHM config in this task's own notes.

- [ ] **Step 3: Perf gate — old node + new (still-camera-ingesting) merged node running together.** This is the actual state production is in for the whole rollout window (Decision 7) — the first real measurement of CUDA-node/Filament-node co-residence (research §7.2, previously unmeasured even for today's two-node design). Dev-box proxy, `budget_probe.sh`-style, both processes live, old node authoritative for modes 1/2 via the mux, new node idling its bowl/hybrid paths (built but not selected). Record GPU SM %, both processes' CPU %.

- [ ] **Step 4: Commit** `feat(visual): local mode-switch enum in the merged node; global mux untouched and still authoritative (VM-093)`.

---

## Task 5 (VM-094): Mode 2 (pointcloud-hybrid) migration — camera-colorized lidar

**Files:**
- Create: `cuda/src/ros_apps/src/micropilot_visualization_node/include/micropilot_visualization_node/lidar_colorize.hpp`
- Create: `cuda/src/ros_apps/src/micropilot_visualization_node/src/lidar_colorize.cpp` (subscribes `pointcloud_topic`, calls `bowl::ProjectToCameraUv` per point per configured camera, samples that camera's last-ingested raw image — reusing Task 2's `camera_ingest.cpp` buffers, not a second ingest path — builds `mpviz::PointCloudPoint[]`)
- Create: `cuda/src/ros_apps/src/micropilot_visualization_node/test/test_lidar_colorize.cpp`
- Modify: `cuda/src/ros_apps/src/micropilot_visualization_node/CMakeLists.txt` (hand-written list + gtest target, same trap as every prior task)
- Modify: `cuda/src/ros_apps/src/micropilot_visualization_node/config/default_params.yaml` (append `hybrid_enabled: false` — default off until sign-off, same as `bowl_enabled`; `splat_radius_px` mapped onto `point_cloud.mat`'s existing point-size uniform, no new material)
- Modify: `cuda/src/ros_apps/src/micropilot_visualization_node/src/visualization_node.cpp` (`render_mode_ == HYBRID` path: bowl renders as the fallback exactly as mode 1 — Decision 5 — plus feeds `lidar_colorize.cpp`'s output into `SceneGraph::point_clouds` via the EXISTING `set_scene()` path, no new entry point)

**Interfaces:**
```cpp
// lidar_colorize.hpp -- node-internal, ROS-facing.
namespace micropilot::visualization_app {
// For each input point, tries every configured camera's ProjectToCameraUv
// (Task 2) in turn; the first camera whose UV falls inside [0,1]^2 AND
// whose depth-from-camera is positive wins (first-match, not
// blended -- unlike the bowl's per-vertex weighted blend, a lidar point
// gets ONE camera's color, matching the CUDA node's own per-point
// colorization choice, reprojector.hpp:30-36). alpha byte is set to 255
// (packing convention, scene.h's PointCloudPoint comment) when a camera
// matched, else 0 (the existing "no real color, use theme neutral" sentinel
// PointCloud already defines -- reused verbatim, not reinvented).
std::vector<mpviz::PointCloudPoint> ColorizeFromCameras(
    const std::vector<mpviz::Vec3>& lidar_points_map_frame,
    const mpviz::BowlConfig& cameras,                 // extrinsics/intrinsics/dims
    const std::vector<const uint8_t*>& camera_rgb_buffers);  // last-ingested frame per camera
}
```
- Consumes: `bowl::ProjectToCameraUv` (Task 2), `mpviz::PointCloudPoint`/`SceneGraph::point_clouds` (already shipped, VM-035).
- Produces: `ColorizeFromCameras` — this task's own adapter's only new function; everything downstream is the already-existing `PointCloud` rendering path.

- [ ] **Step 0: Failing test — a lidar point visible to exactly one configured camera gets that camera's pixel color.**
```cpp
TEST(LidarColorize, PointInSingleCameraFovGetsThatCamerasColor) {
    // One camera, identity extrinsics, zero distortion (Task 2 Step 0's
    // fixture). A lidar point straight ahead at depth 5, and a solid-color
    // test-pattern buffer for that camera (all pixels = {10,20,30}).
    // ColorizeFromCameras() on a 1-point cloud should return one
    // PointCloudPoint with rgba unpacking to (10,20,30,255).
}
```
  Run — FAIL (`lidar_colorize.hpp` doesn't exist). Implement. Run — PASS.

- [ ] **Step 1: Failing test — a point outside every camera's FOV gets the "no color" sentinel (`a == 0`).** Run — FAIL then PASS.

- [ ] **Step 2: Failing test — a point visible to two cameras picks the first configured match, deterministically** (documents the tie-break rule explicitly, since it's a real design choice, not an accident). Run — FAIL then PASS.

- [ ] **Step 3: Wire the node adapter — `hybrid_enabled`, bowl fallback, `set_scene()` feed.** `render_mode_ == HYBRID`: bowl renders exactly as in Task 2 (fallback view, Decision 5), and each tick `ColorizeFromCameras()`'s output is packed into one `SceneGraph::point_clouds` row, fed through the node's EXISTING `set_scene()` call (already made every tick for mode 3's data — this task adds one populated array to the SAME call, not a new call). `hybrid_enabled: false` (shipped default) means the adapter never runs and `point_clouds` stays whatever mode 3's own point-cloud adapter (VM-035) would otherwise populate — **named interaction, not silently overwritten:** if both mode 3's own PointCloud adapter and this task's hybrid adapter could populate the same `SceneGraph::point_clouds` row, that is a real conflict; resolve it by having HYBRID mode be the only mode whose tick populates that row from `lidar_colorize`, exactly the same "one mode is active, others don't touch shared state" discipline the OLD mux already enforced between processes — now enforced by the local enum instead. State this explicitly in the implementation, don't leave it implicit.
  Run existing point-cloud tests (VM-035's) — unaffected, still green.

- [ ] **Step 4: Perf gate.** `render_ms` with hybrid active (bowl + colorization + PointCloud render) vs. bowl-only (mode 1), dev-box proxy. Record the colorization CPU cost specifically (per-point camera-match loop) — this is the honest-unknown research §7.3/§7.4 flagged as never measured; it gets measured here for the first time.

- [ ] **Step 5: Golden — visual check against the CUDA hybrid mode's own output**, one capture, human-sanity-approved, same convention as Task 2 Step 7.

- [ ] **Step 6: Commit** `feat(visual): mode-2 hybrid migrated — camera-colorized PointCloud, hybrid_enabled default off (VM-094)`.

---

## Task 6 (VM-095): Cutover — parity sign-off, mux deleted LAST, old node decommissioned

**Files:**
- Modify: `cuda/src/ros_apps/src/micropilot_visualization_node/src/visualization_node.cpp` (flip `bowl_enabled`/`hybrid_enabled` shipped defaults to `true`; `render_mode_` now driven by the SAME `/rendering/set_mode` topic value, mapped directly 1:1 instead of a mux decision — but read on, this task removes the mux entirely, so restate: `render_mode_` becomes driven by whatever the post-cutover control surface is, decided in Step 2 below)
- Delete: every mux-only code path — `pub_mode_mux_`, `mux_mode_sub_`, the legacy `~/set_render_mode` republish shim, `initial_mode` two-process race handling — on BOTH `rendering_node.cpp`/`.hpp` (about to be deleted wholesale) and `visualization_node.cpp` (only the mux-arbitration half; the local render-mode enum from Task 4 stays, it just becomes the ONLY thing selecting the mode now)
- Delete: `cuda/src/ros_apps/src/micropilot_rendering_node/` — the whole package, once parity holds (CUDA reprojector kernels, `rendering_reprojector` library, launch files, smoke tests — all of it)
- Modify: `cuda/src/ros_apps/src/micropilot_visualization_node/package.xml` (drop `<depend>micropilot_rendering_node</depend>` — this cross-dependency existed only for `SetVirtualCam.srv`)
- Move: `cuda/src/ros_apps/src/micropilot_rendering_node/srv/SetVirtualCam.srv` → `cuda/src/ros_apps/src/micropilot_visualization_node/srv/SetVirtualCam.srv` (the merged node now owns the type it already implements identically, per ADR-0002's own "srv type is reused" clause — this move is what lets the cross-package dependency actually drop)
- Modify: `cuda/src/ros_apps/src/micropilot_visualization_node/src/visualization_node.cpp` (`vcam_state[8]` becomes a pure mirror of `vcam_state[7]`, documented inline per Decision 8 — not removed)
- Modify: `cuda/src/ros_apps/src/micropilot_visualization_node/test/smoke_test.py` (retire `test_mode_mux`, `test_restart_rejoins_live_mode`, `test_legacy_topic_exits_mode_3_cleanly`, `test_initial_mode_one_starts_bowl_not_hybrid`'s two-process framing — replace with the single-process equivalents Task 4 Step 0 already introduced, extended to cover the now-real mode-switch entry point from Step 2 below)
- Create: `docs/adr/0006-one-node-unified-engine.md` (supersedes ADR-0002 explicitly)
- Modify: `docs/adr/0002-two-node-mode-mux.md` (add a **Status: Superseded by ADR-0006 (2026-XX-XX)** line at the top — per this repo's own convention, ADRs are not deleted, they're marked superseded, same as ADR-0004's own header does for the "freeze-then-lift" language it replaced)
- Modify: `cuda/src/libs/visual_renderer/tools/budget_probe.md` (Results (b) — ON-ROBOT section: record the real numbers here, finally, closing VM-043)

**Interfaces:** none new. This task only removes interfaces (the mux protocol) and moves one existing type's package ownership (`SetVirtualCam.srv`).

- [ ] **Step 0: Parity sign-off checklist closed.** Every item Task 4 Step 1 opened in `docs/visual_mode/signoff.md` is checked: bowl-vs-CUDA, hybrid-vs-CUDA, self-view/robot-proxy — each judged by a human against the goldens Tasks 2/3/5 produced plus a live side-by-side run (CARLA or a real bag) with both nodes active and the operator flipping between them via the still-live mux, exactly as Epic 5's VM-043 checklist already does for mode 3. **This step is a hard gate — nothing else in this task proceeds until it is checked off.**

- [ ] **Step 1: On-robot perf rerun — the real VM-043 blocker, closed for real.** Same `budget_probe.sh` procedure, run on actual robot hardware (or the most robot-representative box available), with the merged node as the ONLY rendering process, all three modes exercised, real camera+lidar input. Record `render_ms` p50/p99 (VM-034 instrumentation), GPU/CPU numbers, per Decision 10 — this closes both this epic's own perf gate and Epic 5's VM-043, since there is no longer a second-process axis to measure.

- [ ] **Step 2: Decide and implement the post-cutover mode-selection surface.** With only one process, `/rendering/set_mode` becomes a normal single-subscriber topic — no mux semantics needed. Keep the topic name and message type (`Int32`, 1|2|3) unchanged (every existing external consumer, including `tools/vcam_ws_bridge.py`, already publishes to it) but simplify the handler to a direct `render_mode_ = msg->data` assignment (no `active_mode_`/`render_mode_` distinction, no legacy-topic republish, no transient_local durability requirement — a single process has nothing to late-join). Failing test: publish `1`→`2`→`3` on `/rendering/set_mode` against the merged (now-sole) node; assert `render_mode_` follows directly, no mux-arbitration delay or race. Run — FAIL then PASS.

- [ ] **Step 3: Delete the mux code.** Remove `pub_mode_mux_`, `mux_mode_sub_`, the legacy shim, `initial_mode` race handling, VM-037's transient_local QoS hardening from `visualization_node.cpp` (the mode topic is now a plain default-QoS subscription per Step 2). Delete `micropilot_rendering_node/` wholesale. Run the full merged-node test suite — everything from Tasks 1–5 stays green; the retired smoke-test cases (Task 4/this task's own smoke_test.py edit) are gone, replaced by Step 2's single-process case.

- [ ] **Step 4: Move `SetVirtualCam.srv`, drop the cross-package dependency.** `package.xml`'s `<depend>micropilot_rendering_node</depend>` line removed; the `.srv` now lives in the surviving package. Rebuild clean (`colcon build --packages-select micropilot_visualization_node` with no `micropilot_rendering_node` in the workspace at all) to prove the dependency is genuinely gone, not just unused.

- [ ] **Step 5: Flip `bowl_enabled`/`hybrid_enabled` defaults to `true`.** The migration is complete — modes 1/2 are now live by default in the one surviving node.

- [ ] **Step 6: Write ADR-0006, mark ADR-0002 superseded.** ADR-0006 records: one process, one Filament engine, local mode-select replacing the two-node mux; explicitly notes it gives up the consequences ADR-0002 named (crash isolation between renderer and autonomy pipeline, independent deployment, two GPU contexts) as a deliberate, user-directed tradeoff (the charter quoted at this document's top), not an oversight.

- [ ] **Step 7: Commit** `feat(visual): cutover complete — mux deleted, micropilot_rendering_node decommissioned, ADR-0006 supersedes ADR-0002 (VM-095)`.

---

## Golden scoping (P3, restated for this epic)

Per the 2026-09-07 user decision, "goldens per theme" is scoped to goldens whose subject is theming. None of this epic's goldens (bowl-vs-CUDA, self-view/robot-proxy, hybrid-vs-CUDA) are theme goldens — the bowl doesn't participate in the clay theme system at all (it's camera pixels, not themed geometry). Each ships **one** comparison capture, human-sanity-approved, matching the "visual reference, not a theme sweep" treatment Epic 4 gave its own environment golden.

---

## Review gate

Opus reviewer signs off against:

- **The charter is actually satisfied** — after Task 6, exactly one process renders all three modes through `visual_renderer`/Filament; `git grep -rn "reproject.cu\|robot_raster.cu\|splat.cu" -- cuda/` returns nothing (the package is deleted, not merely unreferenced).
- **ADR-0004 held; each `kSceneVersion` bump is exactly one and justified.** Task 1's bump (4→5) is the only one this epic makes; `sizeof`/`offsetof` asserts exist for `CameraExtrinsics`/`CameraIntrinsics`/`BowlConfig` in both `test_scene_buffer.cpp` and `test_scene_layout.cpp`, and they agree.
- **The POD boundary held** — `check_pod_header.sh` (now colcon-wired per VM-037) stays green throughout; `git grep -n "std::" -- cuda/src/libs/visual_renderer/include/visual_renderer/{api,scene}.h` shows nothing beyond `<cstdint>`/`<cstddef>` typedefs.
- **Camera pixels never went through `set_scene()`/`SceneBuffer`** — `git grep -n "camera\|rgb" -- cuda/src/libs/visual_renderer/src/scene_buffer.cpp` shows no camera-related code; `set_camera_frame()` is the only path, per Decision 2.
- **Dirty tracking is real, not decorative** — Task 1 Step 2's upload-counter test is re-run by the reviewer, not taken on faith.
- **The old node kept working, unmodified, through Tasks 1–5** — `git diff <epic-start>..<task-5-end> -- cuda/src/ros_apps/src/micropilot_rendering_node/` is empty until Task 6.
- **The mux was deleted LAST, and only after the parity checklist was actually checked off** — `docs/visual_mode/signoff.md`'s new section shows every item checked, dated, with a named human sign-off, before Task 6 Step 3's deletion commit.
- **Self-view/robot-proxy compositing added no new rasterizer** — `git grep -n "robot_raster\|z-buffer\|zbuffer" -- cuda/src/libs/visual_renderer/src/bowl.cpp` shows only the analytic occlusion test (Decision 4), no per-pixel depth-render pass, unless Task 3's own golden explicitly triggered the named fallback.
- **Mode 2 added no new library rendering surface** — `git diff <epic-start>..<task-6-end> -- cuda/src/libs/visual_renderer/src/` shows no new point-rendering code; `lidar_colorize.cpp` only produces `PointCloudPoint[]` fed through the existing `set_scene()`/`PointCloud` path.
- **`vcam_state`'s wire shape did not shrink** — still 9 elements post-cutover, index 8 documented as a mirror of index 7, per Decision 8; every consumer (`vcam_ws_bridge.py`, contract tests) still parses correctly.
- **Every STANDING-directive disable knob is real** — `bowl_enabled`/`hybrid_enabled`/`self_view_masks_enabled` each independently proven to gate their respective code path to a true no-op when false (grep the node source, don't take this document's word for it).
- **Epic 4/5/6 interaction recommendations were followed or the user's override is recorded** — if Epic 4 Tasks 2-3 landed mid-epic, confirm the `visualization_node.cpp` merge happened cleanly (Decision 9); confirm this epic's Task 6 perf rerun is cited as VM-043's closure in the master plan, not a duplicate open item (Decision 10).
- **No task exceeded scope** — no Epic 6 streaming-source code, no Epic 4 bake/environment code, touched by any task here.
- **Every named fixture gap and every USER DECISION flagged in this document (Decision 2's vertex-bake-vs-shader-sampling tradeoff, in particular) was either resolved with the user's explicit input or explicitly deferred with a reason, not silently decided by an implementer.**

## Results (fill at close)

- Task 2 perf gate: `render_ms` delta, bowl on vs. off, dev-box proxy: ____ ms (reference ceiling: the ~35% reupload-regression analogy, Decision 2)
- Task 3 perf gate: delta with self-view masks + robot proxy active: ____ ms
- Task 4 perf gate: old node + new (idle-capable) merged node co-residence, dev-box proxy — first-ever measurement of this axis: GPU SM % ____, old-node CPU % ____, new-node CPU % ____
- Task 5 perf gate: hybrid colorization CPU cost specifically: ____ ms/frame at ____ points
- Task 6 on-robot rerun (closes VM-043): GPU model/class actually used: ____; `render_ms` p50/p99 at each mode: ____
- Parity sign-off: bowl ____ (who/when), hybrid ____ (who/when), self-view/robot-proxy ____ (who/when)
- Vertex-bake-vs-shader-sampling decision (Decision 2): which was shipped, and why: ____
- Epic 4 Tasks 2-3 scheduling outcome: parked / proceeded in parallel / landed before this epic — actual outcome recorded: ____
- Any finding the review gate surfaced that this document did not anticipate: ____
