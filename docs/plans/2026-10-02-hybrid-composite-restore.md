# Hybrid Composite Restore: Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to work through this plan task by task. Steps use checkbox (`- [ ]`) syntax for tracking.
> **Execution model (AGENTS.md):** run this as a dynamic workflow. The orchestrator runs on the session model, implementers on Sonnet, review gates on Opus, with at most two fix rounds per task. The orchestrator applies any leftover minor fixes. Each task ends green on `tools/ci_visual_mode.sh`, run in the foreground, and is committed on its own.

**Status: DONE 2026-10-02** (tasks A, B1, B2, D each Opus-gated; see the ledger). Authored 2026-10-02 on branch `fix/hybrid-composite` (from `main` @ `c2e1feb`). Every path:line below was re-checked against that tree while writing. The Filament mechanism was chosen by running a spike and inspecting its rendered output (Decision 1). A second spike, after review, measured the splat colour pipeline and the ego-occlusion revert (Decision 1, *Re-spike*).

## User decision (the charter)

Fix = **A + B**. **Ground lidar points are KEPT.** This matches CUDA: ground splats overwrite the bowl too.

**Goal:** mode 2 (and FREE_LOOK + Surround Stitching with profile `hybrid`) draws camera-colourised lidar the way the CUDA node did. That means opaque square splats **composited over the bowl**, nearest point winning among splats, ego-motion compensated. It must also fail loudly when it is configured to consume a cloud it has no subscription for.

**Architecture:** the library gets a new **hybrid splat layer**. It is a rig-frame point set, anchored by the ego transform exactly like the bowl, and drawn with an opaque unlit `POINTS` material at Filament priority 0. It writes a **stencil** bit. The *backdrop* material instances (bowl, ground plane, ground-grid lines) stencil-test `NOT_EQUAL` against that bit. Splats therefore always beat the backdrop, whatever its depth. They still depth-test normally against each other (nearest wins) and against the ego and objects (mutual occlusion). The node feeds the layer through one new free function per tick, compensates the cloud to the image reference time, warns and diagnoses when hybrid is starved, and gives `splat_radius` its CUDA meaning (size = 2r+1 px).

**Tech stack:** Filament 1.56.5 (stencil API: `MaterialInstance::setStencil*`, `MaterialInstance.h:427-518`; `View::setStencilBufferEnabled`, `View.h:695-717`; `RenderableManager::Builder::priority`, `RenderableManager.h:276-297`), the POD boundary (ADR-0003/0004), and `overlume_ros`.

## Established root cause (input to this plan, adversarially checked)

- **CUDA hybrid was a screen-space composite** (`git show 5247be6^:cuda/src/libs/rendering_reprojector/src/reprojector.cpp`, `render_hybrid` ~L496-554; `src/kernels/splat.cu`). It ran an analytic bowl pass, then a lidar splat pass of `(2r+1)^2` opaque squares (shipped `splat_radius` 3 = 7x7 px, nearest-wins via `atomicMin`, colourised on device from all cameras with feather blending). The composite was "splat where valid, else bowl", with **no depth test against the bowl**. The cloud was ego-motion compensated (`a381b57`: `p' = Rd^T (p - pd)` over `[cloud stamp, t_max]`).
- **The Filament port (VM-094) drew hybrid as an ordinary `SceneGraph::point_clouds` row.** Those rows are 2 px GL points (`theme.cpp:85`), `blending: fade` (`point_cloud.mat:12`), and **depth-tested against the opaque bowl**. Every row shares one material instance (`point_cloud.cpp:121,156-158`). Ground points sit under the lifted bowl floor (`bowl_mesh.cpp:69`, `kBowlZLiftM`), and points outside the bowl wall sit behind it, so both are hidden. That is why `docs/runbooks/hybrid-golden-vm094.md:51-52` measured hybrid-vs-bowl at 0.493 mean abs delta, "visually indistinguishable". **That number recorded the bug.**
- `splat_radius` is declared and then discarded (`overlume_node.cpp:394`), although `tools/vcam_gui.py:96,444` and `tools/vcam_ws_bridge.py:97` send it.
- `default_params.yaml:168-169` sets `hybrid_enabled: true` with `pointcloud_topic: ""`, so `overlume_node.cpp:395` never creates `cloud_sub_`. Hybrid is a silent no-op.
- In FREE_LOOK, the shipped profiles already draw `/iv_points_fusion` as an ordinary row (`urban_profile.yaml:54-56`, `min_z_m: 0.35`). Only `RenderMode::HYBRID` clears the rows (`overlume_node.cpp:1114`), so FREE_LOOK + `hybrid` profile shows the cloud twice (the node even warns about the double subscription at `:623-630`).
- No ego-motion compensation is applied in the node's hybrid path (`overlume_node.cpp:1116-1166`).
- No test reads hybrid output. `ros/src/overlume_ros/test/test_mode_dispatch_pixels.py` has no HYBRID case.

## Decisions (with evidence)

### 1. Filament mechanism: splats write stencil first; the backdrop is stencil-rejected

**Spike.** The spike was a scratch copy of `overlume/` built out-of-tree, never touching this worktree. Its source is `/tmp/claude-1000/.../scratchpad/hybrid/overlume/{src/hybrid_spike.cpp,assets/materials/hybrid_splat.mat,tests/test_zz_hybrid_spike.cpp}`; it is ephemeral, and the essentials are reproduced in Task B1. It used the existing `Bowl` test rig: an overhead magenta camera with `R0=0.5, k=0.3, Rmax=4`, a 1.5x1.5x1.2 fallback ego, a vcam at `(0,-6,6)` looking at the origin, and 320x240 output. It rendered four variants of one point set:

| colour | point set | where |
|---|---|---|
| green | ground grid | z = 0 |
| blue | wall | y = 6, **beyond the rim** |
| yellow | column | directly behind the ego |
| cyan | column | in front of the ego |

Pixel counts for each variant:

| variant | green | blue | yellow | cyan | what it shows |
|---|---|---|---|---|---|
| 0: bowl only | 0 | 0 | 0 | 0 | baseline |
| 1: today's path (ordinary `point_clouds` row) | 0 | 0 | 0 | 0 | **reproduces the bug**: ground and beyond-rim points all hidden by the bowl; only sparse 2 px dots on the bare ground outside the bowl |
| 2: opaque 7 px splats, normal depth (no stencil) | 4120 | 36 | 46 | 1 | size alone does not fix it; the bowl still eats ground and beyond-rim points |
| 3: opaque 7 px splats, priority 0, stencil REPLACE 1; bowl + ground-plane instances `NE 1` | 17045 | 4712 | 59 | 133 | **CUDA composite**: ground carpet over the bowl, beyond-rim wall visible, column in front of the ego visible, ego still occludes points behind it |

I inspected all four images. In variant 3 the ground-grid lines (`gridMaterial`, z = 0.001) still striped the splats. Adding `r.gridMaterial` to the backdrop group removed the stripes (re-inspected).

**Re-spike (after review).** Same rig and scratch tree, with a second test file (`tests/test_zz_hybrid_spike2.cpp`) and a second material (`hybrid_splat_lin.mat`, the B1 material below).

- **Colour.** The camera is uniform `(200,120,60)`. A 13x13 patch of splats with that same `rgba` sits on the bowl floor at `(1.6, 0, 0)`. Each frame is diffed against the bowl-only frame, per pixel and per channel:

  | splat material | pixels differing > 3 | max channel diff | splat pixel vs bowl pixel |
  |---|---|---|---|
  | raw `rgba` as linear `baseColor`, no exposure (the first draft) | 178 | 56 | `(191,159,118)` vs `(201,127,62)`: visibly lighter and washed out |
  | sRGB-decoded × `exposureCompensation` 1.56 | 0 | 1 | the patch is invisible against the bowl |

  The cause: camera textures are `SRGB8`/`SRGB8_A8` (`camera_textures.cpp:22-26`), so the GPU linearises them, and `bowl.mat:109,160` multiplies by `exposureCompensation`. The vertex `COLOR` is a normalized `UBYTE4` with no decode (`point_cloud.cpp:36-50`). Both then go through ACES (`renderer.cpp:758-759`). CUDA wrote the camera bytes straight to the output, so its splats matched its bowl exactly. The decode plus the exposure term restores that match.
- **Ego occlusion revert.** A yellow column at `(0, 1.0, 0.2..0.95)` stands behind the ego. Shipped splat state (priority 0, depth-tested): 0 yellow px. `depthCulling` off **and** priority 7 (the alternative rejected below): 115 yellow px, painted over the ego. `depthCulling` off at priority 0 is **not** a valid revert: opaque priority 0 draws before the ego (priority 4, `RenderableManager.h:276-287`, lower number first), so the depth-tested ego overwrites the splats anyway.

**Alternatives rejected:**

- **Bowl stops writing depth / bowl at far depth (skybox trick).** The ground plane (`renderer.cpp:248-258`, z = 0, present in every mode) and every FREE_LOOK layer would then paint over the bowl. That changes the `bowl` profile and mode 1.
- **Splat material with `depthCulling: false` drawn last.** Splats would paint over the ego and objects, and nearest-wins among splats would be lost (last-drawn wins).
- **A second View with its own depth.** Ego and objects would be drawn twice, with per-view post-processing and colour-matching risk. Too many moving parts.
- **Polygon offset.** It does not apply to `POINTS`, and it cannot fix points behind the wall anyway.

**Blended-queue trap:** the splat material is `blending: opaque`. It sits in the opaque pass, sorted by priority, with no fade co-located against the fade-blended profile rows. The profile `point_clouds` rows keep `point_cloud.mat` and their shared instance untouched. That gives byte-identical behaviour for every non-hybrid frame.

**Byte-identity guard:** the stencil buffer is enabled, and the backdrop instances get `NE 1`, **only while the hybrid layer holds at least one point and the bowl is visible**. Otherwise the View stencil is off and the backdrop compare is reset to `A` (always). The two halves cannot be checked by pixels alone. With the View stencil off there is no stencil attachment (`View.h:691-711`), so the test always passes and a stale `NE` is invisible. With the toggle removed instead, the stencil clears to 0 and `NE 1` passes everywhere. Task B1 therefore asserts the applied state through a test hook (test (d)). The unchanged golden suite is the byte-identity proof for non-hybrid frames.

### 2. Public API: one free function, no `kSceneVersion` bump

```cpp
// scene.h, appended after set_camera_frame():
bool set_hybrid_splats(VisualRenderer*, const PointCloudPoint* rig_points, uint32_t count,
                       float size_px);
```

- **Points are in RIG frame**, the same frame as `BowlConfig` and `ColorizeFromCameras`. The library anchors them with the bowl's ego transform each frame (`bowl.cpp:236-246`), so the node's world-transform loop (`overlume_node.cpp:1148-1158`) goes away. With it goes the "ego invalid, so drop every point" behaviour: hybrid now works whenever the bowl works, including the camera-only pixel-test bag with no `/tf`.
- **Colours** come from the per-point `rgba` (the alpha byte is ignored, and the splat is opaque). The material decodes the bytes from sRGB to linear (the exact piecewise curve, the one the GPU applies to the `SRGB8` camera textures) and multiplies by `exposureCompensation`. That value is taken from the same `BowlConfig::exposure_compensation` the bowl uses. Without this, splats come out lighter than the bowl they sit on (re-spike: max channel diff 56 without it, 1 with it).
- **`size_px <= 0`** selects the theme token. **`count == 0` or `rig_points == nullptr`** clears the layer. The layer is drawn only while the bowl is visible, because it is a bowl composite. The library copies the points. The call returns false for a null renderer.
- **Threading:** call it on the same thread as `render_frame`, like `set_camera_frame` and `set_camera_motion_delta`.
- **ADR-0003/0004 justification:**
  - It adds a free function with no new struct, using the existing POD `PointCloudPoint`. That bumps nothing, per the precedent of `set_environment_source`, `set_camera_frame` and `set_quality`.
  - Hybrid splats are bowl content (rig-frame, bound to the bowl's visibility and ego anchor), not map-frame scene content. Putting them in `SceneGraph` would add a field (bump to 9 and edit both layout tests) and a staging deep copy in `scene_buffer.cpp` for data that already gets one copy here.
  - `test_scene_layout.cpp:10` (`kSceneVersion == 8`) stays valid.
- **Per tick, the node** calls `set_hybrid_splats(renderer_, pts.data(), n, 2*splat_radius_+1)` when `hybrid_cloud_consumed()`, and `set_hybrid_splats(renderer_, nullptr, 0, 0)` otherwise.
- **Style token** (element-config directive), in both shipped themes plus the built-in fallback theme (`theme.cpp:~237`):

  ```yaml
  hybrid_splat:
    size_px: 7.0   # CUDA parity: splat_radius 3 -> 7x7
  ```

  It is parsed like `point_cloud.point_size_px` (`theme.cpp:83-85`, default 7.0 when the key is absent).
- **Disable knob:** the existing node param `hybrid_enabled` (it now actually disables the layer, and is no longer the half-wired flag it was). The library-level "off" is `count = 0`. No new knob.

### 3. Motion-compensation source

The node already has exactly CUDA's inputs, inside `CameraIngest` (`camera_ingest.hpp:84-125`):

- odometry twists `twists_` from `odom_topic`;
- `rig_delta(twists, t_from, t_ref, th, px, py)` (`camera_ingest.cpp:~38`), which is the same function CUDA extracted in `a381b57`;
- the image reference time `t_max` from `IngestState::newest_stamp` (used by `update_motion_deltas`, `camera_ingest.cpp:310-338`).

What B2 adds:

- `bool CameraIngest::cloud_motion_delta(double t_cloud, double& th, double& px, double& py)`: snapshot `twists_` under `odom_mtx_`, call `newest_stamp`, then call `rig_delta(t_cloud, t_max, ...)`. **The span is bounded first.** If `|t_max - t_cloud| > kMaxCloudCompSpanS` (0.5 s), it returns false with th/px/py zero, and logs `RCLCPP_WARN_THROTTLE(5000, "hybrid: cloud stamp %.3fs from image ref, compensation skipped")`.
  - Why: `rig_delta` runs `n = ceil(|span| / 0.005)` steps (`camera_ingest.cpp:39-55`) with no clamp. A lidar clock outside the cameras' time domain (unsynced device clock, a mixed-epoch replay, a zero stamp) gives `|span|` of about 1e9 s, which is about 2e11 iterations inside `timer_callback` on every tick, and the render loop hangs. CUDA `a381b57` only ever saw synced data.
  - Why 0.5 s rather than `max_sync_latency_` (0.12 s): a 10 Hz lidar cloud is routinely up to ~0.1 s older than the newest image, plus transport latency, so 0.12 s would skip legitimate compensation. 0.5 s is far above that. Anything beyond it is not motion to compensate but a clock problem. It is also well inside the 2 s twist history (`camera_ingest.cpp:276`), beyond which `twist_at` only extrapolates `twists.back()`.
- **Shared guard in `rig_delta`** (root cause; it has two callers, `cloud_motion_delta` and `compensation_delta_4x4` for the cameras): return false when `|span| > kTwistHistoryS` (2.0 s, the same constant that trims `twists_` at `:276`). Today a camera whose clock is in another epoch hangs the camera path the same way. With the guard, such a camera gets the identity delta, the existing `rig_delta`-false behaviour of `compensation_delta_4x4` (`:61`). The existing tests use spans ≤ 0.5 s and are unaffected.
- A pure function `void CompensateCloud(std::vector<Vec3>&, double th, double px, double py)`, which applies `p' = Rd^T (p - pd)`.
- The cloud callback records `msg->header.stamp`.

When odometry is absent (`default_params.yaml:81` sets `odom_topic: ""`), compensation is inert, which is the same as the bowl's own camera compensation, so the two stay consistent. Colourisation must use the **same** per-camera motion deltas the bowl shader applies (`bowl.cpp:211-226`). Otherwise compensated points are projected through uncompensated cameras. B2 adds `ApplyMotionDelta(CameraExtrinsics, const double delta[16])`, which mirrors `update_bowl`'s math, and feeds the compensated extrinsics into `ColorizeFromCameras`.

### 4. Pixel-test fixture

- `test_mode_dispatch_pixels.py:59` defaults to `~/TPSProjector-fixtures/stack_v3_full_sensors_2026-09-11`, **which does not exist on the dev box**. The bag lives at `~/overlume-fixtures/stack_v3_full_sensors_2026-09-11`. **The test currently SKIPs (exit 0) by default.** B2 changes the default to `~/overlume-fixtures/...` and keeps `--bag`.
- That bag **does carry `/iv_points_fusion`**: `sensor_msgs/msg/PointCloud2`, 633 msgs over 67.4 s (about 9.4 Hz), frame `seyond`, about 164k points per message (`metadata.yaml:143-147`, first message inspected via sqlite). It has **no `nav_msgs/Odometry`**, so motion compensation is not exercised by the pixel test; that is covered by gtest instead.
- The test plays camera topics only (`:244-248`). B2 adds `/iv_points_fusion` to `--topics`. No synthetic publisher is needed.
- Because the library now anchors splats with the bowl's transform, no `/tf` is needed. Ego is invalid, so the transform is identity, the same as the bowl.

## Global constraints

- Public headers are POD-only and append-only. `overlume/scripts/check_pod_header.sh` must pass. `kSceneVersion` stays **8**.
- **Goldens are promoted by a human.** Do not touch `overlume/tests/goldens/hybrid_*.png`. Candidates go to `docs/evidence/2026-10-02-hybrid-restore/`.
- Every behaviour change ships with a check that fails when the change is reverted. Each task names its revert check.
- **Element-config directive:** the hybrid splat ships the `hybrid_splat.size_px` token plus the `hybrid_enabled` knob.
- **Tokens:** `CESIUM_ION_TOKEN`/`MAPBOX_TOKEN` are referenced by name only. Nothing here needs network access.
- **Gate:** run `tools/ci_visual_mode.sh` in the foreground (Bash timeout 600000; if it runs longer, use `run_in_background` and wait). Re-run once after a spurious low-memory kill. Node build: `ros/colcon_build.sh`.
- Library sources and materials are globbed with `CONFIGURE_DEPENDS` (`CMakeLists.txt:115,143,328`), so a new `.mat`/`.cpp`/test needs no list edit. Node-side new `.cpp` files must be appended to the hand-written source list in `ros/src/overlume_ros/CMakeLists.txt` and to every gtest target that needs them.
- GPU tests skip cleanly without a GPU (`create_renderer` returns null, so they call `GTEST_SKIP()`).

## Status ledger

| Task | Status | Commit |
|---|---|---|
| A: node loud failure, `pointcloud_topic` default, `splat_radius` live | Done 2026-10-02 | a83cd38 |
| B1: library hybrid splat layer + `set_hybrid_splats` | Done 2026-10-02 | 579acef |
| B2: node feeds the layer, motion compensation, FREE_LOOK de-dup, pixel checks | Done 2026-10-02 | d690a14 |
| D: docs, status, changelog, README caption, candidate capture | Done 2026-10-02 | 41a720c |

Order: **A → B1 → B2 → D.** A is independent of B1 and could run in parallel, but both touch `overlume_node.cpp`, so they run serially.

---

## Task A: node loud failure, topic default, `splat_radius`

**Files:**

- `ros/src/overlume_ros/include/overlume_ros/diagnostics.hpp`, `src/diagnostics.cpp`
- `ros/src/overlume_ros/test/test_diagnostics.cpp`
- `ros/src/overlume_ros/include/overlume_ros/overlume_node.hpp`, `src/overlume_node.cpp`
- `ros/src/overlume_ros/config/default_params.yaml`
- `ros/src/overlume_ros/test/test_bowl_node_params.py` (only if it asserts the old defaults; check before editing)

**Interfaces:**

```cpp
// diagnostics.hpp
// Empty string == healthy. Otherwise names the param the operator must set.
std::string HybridStarvedReason(bool cloud_consumed, bool hybrid_enabled, bool has_cloud_sub);
//   consumed && !hybrid_enabled  -> "hybrid_enabled is false"
//   consumed && !has_cloud_sub   -> "pointcloud_topic is empty -- set pointcloud_topic and reconfigure"
//   else                         -> ""
diagnostic_msgs::msg::DiagnosticStatus BuildHybridStatus(const std::string& starved_reason);
//   name "hybrid", level ERROR + message=reason when non-empty, OK "ok" otherwise
```

Steps:

- [ ] **A1 (failing test first).** In `test_diagnostics.cpp`, add `HybridStarvedReason` cases: the full truth table, plus "the message names `pointcloud_topic`" and "the message names `hybrid_enabled`". Add `BuildHybridStatus` cases for the ERROR and OK levels. Build and run them; they fail because the symbols are undefined.
- [ ] **A2.** Implement both functions in `diagnostics.cpp`.
- [ ] **A3.** Wire the failure into the node so it surfaces whenever the state is or becomes true (configure, `render_mode`, `layer_surround_stitching`, `surround_stitching_profile`). The tick re-evaluates every ~33 ms, so one site covers all transitions:
  - In `timer_callback()`, next to the existing `set_hybrid_enabled` call (`overlume_node.cpp:~1111`), compute `reason = HybridStarvedReason(hybrid_cloud_consumed(), hybrid_enabled_, cloud_sub_ != nullptr)`.
  - If `reason` is non-empty, call `RCLCPP_WARN_THROTTLE(..., 5000, "hybrid: render_mode=%d/profile=%s consumes a lidar cloud but %s -- splats will not render", ...)`.
  - Store `reason` in a member `hybrid_starved_reason_`.
  - In `publish_diagnostics()` (`:1350`), append `BuildHybridStatus(hybrid_starved_reason_)` to `msg.status`.
- [ ] **A4.** Set `default_params.yaml:169` to `pointcloud_topic: "/iv_points_fusion"`, and `:183` to `splat_radius: 3` (CUDA shipped value, matching `m2o1_params.yaml:32`). Leave the code default for `pointcloud_topic` as `""`, so the library-only/minimal launch stays explicit. The warning covers it.
- [ ] **A5 (`splat_radius`).**
  - Replace `declare_parameter<int>("splat_radius", 2);` (`:394`) with `splat_radius_ = declare_parameter<int>("splat_radius", 3);`.
  - Add the member `int splat_radius_{3};`.
  - In `on_params`, accept `splat_radius` in `[0, 30]` (the GUI's range, `vcam_gui.py:96`) and reject anything else with a reason.
  - Add `float hybrid_splat_px() const { return 2.0f * splat_radius_ + 1.0f; }`. It is consumed in B2. A unit check is in B2, because the value has no visible effect until then.
- [ ] **A6. Revert check.** Removing the A3 wiring makes the gtest for `HybridStarvedReason` still pass, so A also adds a log assertion to `test_mode_dispatch.py` or `test_bowl_node_params.py`, whichever already launches the node and reads its log:
  - launch with `-p pointcloud_topic:=""`, `render_mode:=2`;
  - grep the log for `hybrid:` and `pointcloud_topic`;
  - expect FAIL if the warning is removed.
- [ ] **A7.** `ros/colcon_build.sh`, then the gate. Commit:

  ```
  fix(ros): hybrid fails loudly without a cloud; pointcloud_topic defaults to /iv_points_fusion

  default_params.yaml shipped hybrid_enabled:true with pointcloud_topic:"" so
  no cloud subscription was created and modes 2 / FREE_LOOK+hybrid silently
  rendered bowl only. The node now warns (throttled) and publishes a "hybrid"
  ERROR diagnostic whenever hybrid_cloud_consumed() holds without a
  subscription, naming the param to set. splat_radius is read and live-tunable
  (consumed as 2r+1 px by the hybrid splat layer in the next commit).

  Claude-Session: https://claude.ai/code/session_01WPqfpuRJqdZx9jSD6MVnfq
  ```

## Task B1: library hybrid splat layer

**Files:**

- `overlume/include/overlume/scene.h` (append the declaration and its Doxygen comment after `set_camera_frame`, matching the documented-header style of Restructure Task 5)
- `overlume/assets/materials/hybrid_splat.mat` (new)
- `overlume/src/hybrid_splats.{hpp,cpp}` (new)
- `overlume/src/hybrid_splats_test_hooks.hpp` (new; test (d) needs it): `struct HybridStencilState { bool view_stencil; StencilCompareFunc bowl, ground, grid; };` and `HybridStencilState hybrid_stencil_state_for_test(VisualRenderer*)`. It returns `view->isStencilBufferEnabled()` (`View.h:717`) plus the compare function **last applied** to each backdrop instance. Filament has no getter for that, so `apply_backdrop_stencil()` records it in `VisualRenderer`.
- `overlume/src/renderer_internal.hpp`, `src/renderer.cpp` (create/destroy, plus a call in `render_frame` right after `update_bowl`, `renderer.cpp:1134`)
- `overlume/src/bowl.cpp` (extract the ego anchor transform into a shared helper `mat4f ego_anchor_transform(const EgoState&)`, used by both `update_bowl` and the splat layer)
- `overlume/src/theme.{hpp,cpp}`, `overlume/assets/themes/{dark_adas,light_clay}.yaml`
- `overlume/tests/test_hybrid_splats.cpp` (new), `overlume/tests/test_theme.cpp`

**Material** (exactly the re-spike's `hybrid_splat_lin.mat`, renamed; matc accepted it and it matched the bowl to 1/255):

```
material {
    name : hybrid_splat,
    shadingModel : unlit,
    parameters : [
        { type : float, name : pointSizePx },
        { type : float, name : exposureCompensation }
    ],
    requires : [ color ],
    blending : opaque
}
vertex   { void materialVertex(inout MaterialVertexInputs material) { gl_PointSize = materialParams.pointSizePx; } }
fragment {
    void material(inout MaterialInputs material) {
        prepareMaterial(material);
        vec3 c = getColor().rgb;   // normalized UBYTE4 sRGB bytes, not decoded by the vertex fetch
        vec3 lin = mix(c / 12.92, pow((c + 0.055) / 1.055, vec3(2.4)), step(0.04045, c));
        material.baseColor = vec4(lin * materialParams.exposureCompensation, 1.0);   // == bowl.mat:109,160
    }
}
```

**Layer state (in `VisualRenderer`):**

- `hybridSplatMaterial`, plus one instance;
- `std::vector<Mesh> hybridSplatMeshes`, chunked by `detail::polyline_chunks` (uint16 index ceiling), the same as `point_cloud.cpp:96-129`. Reuse that file's vertex-buffer helper by moving `make_point_vertex_buffer` to a shared internal header, rather than copying it;
- `uint32_t hybridSplatCount`;
- `float hybridSplatSizePx`;
- `float bowlExposure` (set in `build_bowl()` from `cfg.exposure_compensation`, next to `bowl.cpp:202`);
- the last-applied backdrop compare functions (for the test hook).

**Instance state:** stencil write on, compare `A`, `DepthStencilPass = REPLACE`, ref 1, `setCullingMode(NONE)`. **Renderable:** `priority(0)`, `culling(false)`, no shadows, and a TransformManager component.

**Per frame (`update_hybrid_splats(r, ego)`):**

```
active = r.bowlVisible && r.bowl && r.hybridSplatCount > 0
for each splat mesh: in scene iff active; transform = ego_anchor_transform(ego)
pointSizePx = (size_px > 0 ? size_px : theme.hybrid_splat.size_px)
exposureCompensation = r.bowlExposure          // every frame: cheap, and survives every re-bake
apply_backdrop_stencil(r, active):
    r.view->setStencilBufferEnabled(active)
    for bg in {r.bowl->instance, r.groundMaterial, r.gridMaterial}:
        bg->setStencilCompareFunction(active ? NE : A); bg->setStencilReferenceValue(1)
        record the applied compare for the test hook
```

Also call `apply_backdrop_stencil()` in `build_bowl()` after creating the instance, because a re-bake creates a fresh instance (`bowl.cpp:131-136`).

Steps:

- [ ] **B1.1 (failing tests first, `test_hybrid_splats.cpp`).** Reuse the spike rig above. Calibrate the thresholds from the first passing run and record them in the test comment.
  - (a) `SplatsDrawOverBowlWhereAnOrdinaryRowIsHidden`: the same points rendered as a `SceneGraph::point_clouds` row show ≈0 green inside the bowl region, versus > 10000 green and > 3000 beyond-rim blue via `set_hybrid_splats`. This is the user-asked library proof.
  - (b) `EgoStillOccludesSplatsBehindIt`: a column at `(0, 1.0, 0.2..0.95)` behind the 1.5x1.5x1.2 fallback ego is fully hidden (count ≤ a small epsilon; re-spike: 0), and a column in front of the ego, placed where the camera sees it, is visible (> 50 px; calibrate its position on the first run, since the re-spike's `(0.4,-1.2)` column measured 0 cyan in this rig).
  - (c) `NearestSplatWins`: two coincident-in-screen splats of different colours at different depths. The nearer colour shows, whichever was submitted first (submit far-first and near-first and assert both).
  - (d) `StencilStateFollowsTheLayer` (via `hybrid_stencil_state_for_test`):
    - fresh renderer with the bowl: `{false, A, A, A}`;
    - after `set_hybrid_splats(pts)` + `render_frame`: `{true, NE, NE, NE}`;
    - after a bowl re-bake (`set_bowl_config` again) + `render_frame`: still `{true, NE, NE, NE}` on the **new** bowl instance;
    - after `set_hybrid_splats(nullptr, 0, 0)` + `render_frame`: `{false, A, A, A}`;
    - after `set_bowl_visible(false)` with points held: `{false, A, A, A}`.
  - (d2) `ClearingRestoresByteIdenticalFrames`: frame A is bowl + an ordinary row. Frame B is the same scene after `set_hybrid_splats(pts)` and then `set_hybrid_splats(nullptr, 0, 0)`. Frame A == frame B byte for byte. This guards **byte identity** (no leftover entity, no leftover parameter). It does **not** guard the compare reset; (d) does.
  - (e) `HiddenWhenBowlHidden`: with `set_bowl_visible(false)`, splats contribute 0 px.
  - (f) `SizeFromThemeTokenWhenZero`: `size_px = 0` vs `size_px = 1` over sparse points gives a covered-pixel ratio of about 49 (7² vs 1²); assert > 20.
  - (g) `SplatColourMatchesBowl`: the re-spike rig. The camera is uniform `(200,120,60)`, and a 13x13 patch of splats with that `rgba` at 0.03 m spacing lies on the bowl floor at `(1.6, 0, 0)`. Use a patch, not one splat, so bloom is the same on both sides. Diff against the bowl-only frame of the same scene and assert that no pixel differs by more than 3 on any channel (re-spike: max 1; without the decode: max 56). Run it twice: once at the default exposure, and once after re-baking with `exposure_compensation = 1.2`, so that a hard-coded 1.56, or an exposure that is only set at layer creation, also fails.

  `test_theme.cpp`: `hybrid_splat.size_px` parses from both shipped themes as 7.0 and defaults to 7.0 when absent.
- [ ] **B1.2.** Implement the material, `hybrid_splats.cpp`, the theme token and the API. Run `overlume/scripts/check_pod_header.sh`.
- [ ] **B1.3. Revert checks:**
  - Delete the stencil lines and (a) fails (spike variant 2: 4120 green with the stencil off vs 17045 with it on).
  - Delete the stencil reset, or the view toggle, or the `build_bowl()` re-apply, and (d) fails. (d2) does not, by construction (see Decision 1, *Byte-identity guard*).
  - Revert to the alternative Decision 1 rejected (splats `depthCulling:false` **and** `priority(7)`, drawn last) and (b) fails (re-spike: 115 yellow px behind the ego vs 0). `depthCulling:false` alone is not a valid revert: at priority 0 the ego still draws later and overwrites the splats.
  - Drop the sRGB decode, or the exposure term, and (g) fails (re-spike: max diff 56). Hard-code 1.56 and the 1.2 half of (g) fails.
- [ ] **B1.4.** Run the gate (stage 2 runs the new ctest). Golden suite: every existing golden must still pass unchanged. A failure there is a finding (the byte-identity guard broke), not something to re-shoot. Commit:

  ```
  feat(lib): hybrid splat layer drawn over the bowl (set_hybrid_splats)

  Mode 2 drew lidar as an ordinary fade-blended 2 px point row depth-tested
  against the opaque bowl, so ground points (under the lifted bowl floor) and
  points beyond the wall were hidden -- hybrid was indistinguishable from bowl
  (VM-094 runbook delta 0.493). New free function set_hybrid_splats() takes
  rig-frame colourised points; an opaque POINTS material at priority 0 writes
  stencil 1 and the bowl/ground/grid instances stencil-reject it, restoring the
  CUDA composite (splat wins over bowl, nearest splat wins, ego/objects still
  depth-occlude). Splat colour is sRGB-decoded and scaled by the bowl's
  exposureCompensation, so a splat matches the bowl pixel it covers (<= 3/255). Stencil is only enabled while the layer is non-empty, so all
  other frames are byte-identical. Size: theme token hybrid_splat.size_px (7).
  No struct change: kSceneVersion stays 8 (ADR-0004).

  Claude-Session: https://claude.ai/code/session_01WPqfpuRJqdZx9jSD6MVnfq
  ```

## Task B2: node feeds the layer; compensation; FREE_LOOK de-dup; pixel checks

**Files:**

- `ros/src/overlume_ros/include/overlume_ros/camera_ingest.hpp`, `src/camera_ingest.cpp`
- `ros/src/overlume_ros/include/overlume_ros/lidar_colorize.hpp`, `src/lidar_colorize.cpp`
- `ros/src/overlume_ros/src/overlume_node.cpp`, `include/overlume_ros/overlume_node.hpp`
- `ros/src/overlume_ros/test/test_camera_ingest.cpp`, `test/test_lidar_colorize.cpp`
- `ros/src/overlume_ros/test/test_mode_dispatch_pixels.py`

**Interfaces:**

```cpp
// camera_ingest.hpp
bool CameraIngest::cloud_motion_delta(double t_cloud, double& th, double& px, double& py);  // false = no odom / no stamps / |span|<1e-4 / |span|>kMaxCloudCompSpanS (0.5 s, throttled warn)
constexpr double kMaxCloudCompSpanS = 0.5;
constexpr double kTwistHistoryS = 2.0;   // replaces the literal 2.0 at camera_ingest.cpp:276; rig_delta returns false beyond it
overlume::CameraExtrinsics ApplyMotionDelta(const overlume::CameraExtrinsics&, const double delta_row_major[16]);  // == update_bowl's right/fwd/t math
void CameraIngest::fill_compensated_extrinsics(std::vector<overlume::CameraExtrinsics>&) const;  // static ext advanced by the last deltas update_motion_deltas() pushed
// lidar_colorize.hpp
void CompensateCloud(std::vector<overlume::Vec3>& rig_pts, double th, double px, double py);  // p' = Rd^T (p - pd), z unchanged (a381b57)
```

Steps:

- [ ] **B2.1 (failing tests first).**
  - `test_lidar_colorize.cpp`: `CompensateCloud` with `th=0, p=(1,0)` moves `(5,0,1)` to `(4,0,1)`; with `th=π/2` rotates as `Rd^T`; identity when everything is zero.
  - `test_camera_ingest.cpp`: `ApplyMotionDelta` with identity is a no-op; a pure x-translation shifts `t`; this must agree with `update_bowl`'s formula (`bowl.cpp:213-222`, transcribed as the expected value). A constant-twist deque must make `cloud_motion_delta` equal to `rig_delta` over `[t_cloud, t_max]`.
  - `test_camera_ingest.cpp` (span bound):
    - with a **non-empty** constant-twist deque (an empty one already makes `twist_at` fail on the first step, which would make the case vacuous), `rig_delta(twists, 0.0, 1e6, ...)` returns false at once, with th/px/py == 0. Put it under a `std::chrono` assert of < 10 ms, so the unbounded loop fails by time and not by hanging the suite (also give the gtest a ctest `TIMEOUT` of 30 s);
    - `cloud_motion_delta` with `t_cloud = t_max - 1e6` returns false with th/px/py == 0;
    - `t_cloud = t_max - 0.6` returns false; `t_max - 0.4` returns true (the 0.5 s edge);
    - `t_cloud = 0.0` (zero stamp) returns false.
  - Revert check: remove either guard and the corresponding 1e6 case fails the < 10 ms assert (or the ctest timeout).
- [ ] **B2.2. Node changes in `overlume_node.cpp`:**
  - The cloud callback stores `cloud_stamp_ = rclcpp::Time(msg->header.stamp).seconds()` with the points, under `cloud_mtx_`.
  - Remove the hybrid `point_clouds.push_back` row and the world-transform loop (`:1148-1166`). New flow:

    ```
    pts = copy cloud_pts_rig_ (+stamp)
    if cloud_motion_delta(stamp, th,px,py): CompensateCloud(pts, th,px,py)
    ext = fill_compensated_extrinsics()
    colorized = ColorizeFromCameras(pts, cams{ext,...}, rgb)
    set_hybrid_splats(renderer_, colorized.data(), n, hybrid_splat_px())
    ```

  - When `!(hybrid_cloud_consumed() && hybrid_enabled_ && camera_ingest_)`, call `set_hybrid_splats(renderer_, nullptr, 0, 0)`.
  - **Staleness:** if `sim_clock_sec_ - cloud_rx_sec_ > 2.0` (the profile row's `timeout_sec`, `urban_profile.yaml:56`), pass 0 points. A frozen lidar must not leave stale splats.
  - **FREE_LOOK de-dup:** in the `point_cloud_rows_` fill loop (`:1073-1082`), skip `pcr.adapter->fill()` when `hybrid_cloud_consumed() && hybrid_enabled_ && cloud_sub_ && pcr.topic == pointcloud_topic_`. The `bowl` profile is unchanged. `RenderMode::HYBRID`'s blanket clear (`:1114`) stays.
  - Reword the double-subscription warning (`:623-630`): the profile row is suppressed while hybrid consumes the cloud.
- [ ] **B2.3. Pixel checks in `test_mode_dispatch_pixels.py`:**
  - Set `DEFAULT_BAG` to `~/overlume-fixtures/stack_v3_full_sensors_2026-09-11` (Decision 4).
  - Add `/iv_points_fusion` to the played topics.
  - Add `-p pointcloud_topic:=/iv_points_fusion -p hybrid_enabled:=true` to `viz_cmd` (explicit even though A changed the default, so the calibration on `main` measures the depth-tested path rather than the no-op).
  - Add `--dump-dir DIR`: when given, write each averaged window as PNG (numpy → PIL; PIL is already used by `tools/`). D uses this for the candidate capture.
  - Add a helper `_mean_abs(a, b)` returning `float(np.abs(a-b).mean())`.

  New cases go after the existing ones, which stay as they are:

  - (i) **mode 2 vs mode 1:**
    - set `render_mode 1` → capture `b1`, `b2` (noise = `_mean_abs(b1, b2)`);
    - set `render_mode 2` → capture `h`;
    - assert `_mean_abs(h, b2) >= max(HYBRID_DELTA_FLOOR, noise * 3)`.
  - (ii) **FREE_LOOK + stitching, `hybrid` vs `bowl` profile:**
    - set `render_mode 3`, `layer_surround_stitching true`, `surround_stitching_profile bowl` → capture `s1`, `s2`;
    - set `surround_stitching_profile hybrid` → capture `sh`;
    - same assertion.
  - **Calibration (mandatory, recorded in the test docstring and the B2 commit body):**
    - run (i) and (ii) on this branch **with B1/B2 reverted** (`git stash` the B2 node changes, keeping the test change). Values are expected to be ≈ 0.5, as in the VM-094 runbook;
    - then run them with the fix;
    - set `HYBRID_DELTA_FLOOR` at the geometric mean of the two, rounded down. It **must FAIL on the reverted tree and PASS on the fix**. Paste both runs' numbers.
  - Run on the dev box, with GPU and bag present. Paste `PASS` lines into the commit body.
- [ ] **B2.4. Revert checks:**
  - Revert the node feed: the pixel check fails (calibration above).
  - Revert compensation or `ApplyMotionDelta`: the gtests fail.
  - Revert de-dup: the pixel delta cannot catch this (the duplicate is faint), so the node logs `hybrid: profile row <topic> suppressed` once per transition, and `test_mode_dispatch_pixels.py` greps the node log for it after case (ii). Removing the skip removes the log line and fails the test.
- [ ] **B2.5.** `ros/colcon_build.sh`, then the gate. Commit:

  ```
  fix(ros): feed hybrid lidar through set_hybrid_splats, ego-motion compensated

  Points stay rig-frame (the library anchors them like the bowl, so hybrid no
  longer needs a valid TF ego), are compensated from the cloud stamp to the
  image reference time (a381b57 semantics, via CameraIngest's odom twists) and
  colourised through the same per-camera motion deltas the bowl uses; size is
  2*splat_radius+1 px. FREE_LOOK + stitching profile 'hybrid' suppresses the
  profile row on pointcloud_topic it duplicated. test_mode_dispatch_pixels.py
  now finds its bag (~/overlume-fixtures; it silently SKIPped before), plays
  /iv_points_fusion and asserts mode2-vs-mode1 and hybrid-vs-bowl-profile
  mean-abs-diff floors calibrated to FAIL on the pre-fix path: <numbers>.

  Claude-Session: https://claude.ai/code/session_01WPqfpuRJqdZx9jSD6MVnfq
  ```

## Task D: docs

**Files:**

- `docs/runbooks/hybrid-golden-vm094.md`
- `docs/status.md`
- `CHANGELOG.md`
- `README.md:84`
- `docs/evidence/2026-10-02-hybrid-restore/` (new)

Steps:

- [ ] **D1.** Runbook: add a dated **Correction (2026-10-02)** block above the 0.493 table. It says the 0.493 / "visually indistinguishable" result *was the defect*: depth-tested 2 px fade points under/behind the bowl, plus the empty default `pointcloud_topic`. It links this plan and states that `hybrid_test_town_merged_node.png` predates the fix and awaits human re-promotion. Do not delete the original measurements.
- [ ] **D2.** `docs/status.md`:
  - Add a Shipped row: "Hybrid composite restore (lidar over bowl)", with date, commits, and this plan.
  - Add a Known-gaps line: the golden is not yet re-promoted, and motion compensation needs `odom_topic`, which is empty by default and absent in the fixture bag.
  - In the row, state the root cause, the fix and the checks in one line each.
- [ ] **D3.** `CHANGELOG.md` `[Unreleased]`: add a **Fixed** bullet ("hybrid mode rendered bowl-only: …") and an **Added** bullet (`set_hybrid_splats()`, theme token `hybrid_splat.size_px`, `hybrid` diagnostic).
- [ ] **D4.** README caption (`README.md:84`): change it to "bowl plus colourised lidar (node frame; pre-fix capture, re-shoot pending)", because the pictured frame is the buggy output. Leave the image link alone.
- [ ] **D5. Candidate capture.** Run `python3 ros/src/overlume_ros/test/test_mode_dispatch_pixels.py --dump-dir docs/evidence/2026-10-02-hybrid-restore`. Keep the mode-1, mode-2, `bowl`-profile and `hybrid`-profile frames. Add a `README.md` in that directory with the command, the bag, the commit, the measured deltas, and "candidate for human promotion over `overlume/tests/goldens/hybrid_test_town_merged_node.png`; not promoted". Inspect the PNGs (Read) before committing. Splats must visibly carpet the road and outline obstacles over the bowl.
- [ ] **D6.** Gate, then commit:

  ```
  docs(hybrid): record the composite root cause, fix and candidate capture

  Claude-Session: https://claude.ai/code/session_01WPqfpuRJqdZx9jSD6MVnfq
  ```

## Open questions

1. **Calibration of `pointcloud_transform` for the fixture bag.** The bag's lidar frame is `seyond`. The default transform (`default_params.yaml:170-182`, identity plus z = 1.15) is unverified for this bag. Mis-registration would still pass the delta floors but give wrong colours or positions. D5's visual inspection is the check. If the cloud is clearly mis-placed, the 180° lidar-yaw trap from the replay notes may apply, and that is a separate fix.
2. **Ego-motion compensation is inert by default.** `odom_topic: ""` is the default, and the fixture has no Odometry topic. Should a TF-derived twist source be added? That is out of scope here, the same limitation as the bowl.
3. **Should FREE_LOOK map/OGM layers also yield to ground splats?** They are not in the backdrop group, so lane paint at z ≈ 0.01-0.05 can still stripe ground splats in FREE_LOOK + `hybrid`. The current choice keeps them as foreground, which is the conservative reading of "as designed".
