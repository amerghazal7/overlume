# Visual Mode sign-off

Tracks the open sign-off items across Visual Mode epics. Created here by
the unified-engine migration's Task 4 (VM-093) Step 1 — no prior file
existed under this name in either the worktree or the main checkout at the
time this section was written (the master plan's Epic 5 row names this as
VM-043's file; VM-043's own on-robot rerun is still open, so this is this
epic's first use of it).

## Unified-engine parity sign-off (blocks Task 6)

Opened by Task 4 (VM-093) Step 1, 2026-09-11. Task 6 (VM-095) gates its own
cutover (mux deletion, `micropilot_rendering_node` decommission) on every
item below being closed or explicitly accepted.

| Item | Owner | Status |
|---|---|---|
| Bowl-vs-CUDA visual parity | Task 2 (VM-091)'s golden (`docs/visual_mode/bowl-golden-vm091.md`) + a live side-by-side | **APPROVED 2026-09-11** (fair parity package, user verdict) |
| Hybrid-vs-CUDA visual parity | Task 5 (VM-094)'s golden (`docs/visual_mode/hybrid-golden-vm094.md`) | **APPROVED 2026-09-11** (fair parity package + d7c0c80 golden, user verdict) |
| Self-view/robot-proxy parity | Task 3 (VM-092)'s golden | **APPROVED 2026-09-11** (robot proxy composited in every approved frame; masks structurally no-op on this rig) |
| Perf parity at each milestone | This epic's own perf-gate steps (Task 2 Step 5, Task 4 Steps 2/3, Task 5's own gate, Task 6's on-robot rerun) | **All closed.** Task 2 Step 5 + Task 4 Steps 2/3 closed (see `tools/budget_probe.md` Results (c)/(d)); Task 5's gate closed too (`hybrid_perf_gate.sh`: image_hz 30.271, render_ms p99 23.071 ms — see `docs/visual_mode/hybrid-golden-vm094.md`); **Task 6 (VM-095) Step 1 closed 2026-09-11 (dev-box proxy, RTX 3090 — no robot hardware reachable this session): all three modes PASS `>=30.0 Hz`/`<=33ms p99` with `bowl_enabled`+`hybrid_enabled` both true — BOWL 30.260 Hz/21.191ms, HYBRID 30.237 Hz/21.062ms, FREE_LOOK 30.296 Hz/13.229ms — see `tools/budget_probe.md` Results (b). The on-actual-robot rerun is a named deployment item, not this row's blocker (the plan's own text allows the most robot-representative box available when hardware is unreachable).** |
| Node-level bowl-visible/bowl-hidden pixel assertions for the per-mode dispatch (BOWL renders bowl, FREE_LOOK hides it, FREE_LOOK+Surround-Stitching shows it alongside the autonomy scene) + node-level "autonomy layers do not render in BOWL with live data flowing" pixel check, both reusing Task 2's sentinel-magenta check against a LIVE node render | Task 4 (VM-093) review round 1, deferred to Task 6's golden captures | **CLOSED 2026-09-11 (Task 6/VM-095 Step 0).** `test_mode_dispatch_pixels.py` (new, node-level, mean-abs-pixel-diff between averaged capture windows against a REAL running node fed the `stack_v3_full_sensors_2026-09-11` fixture bag's six real cameras): FREE_LOOK shows a toggled synthetic autonomy object (noise_floor=0px, signal=3048px); BOWL suppresses it while live data keeps flowing on `/perception/dynamic_objects_list` (noise_floor=1616px, signal=1589px — stays within BOWL's own live-video noise floor, does not clear it); FREE_LOOK+Surround-Stitching shows the object again alongside bowl content (noise_floor=1523px, signal=3576px); BOWL-vs-FREE_LOOK content itself differs by 75697px, Surround-Stitching-vs-plain-FREE_LOOK by 75224px (object off in both, isolating the mode/profile's own content difference). All three PASS. **BOWL→FREE_LOOK→BOWL `layer_*` restore is NOT part of this deferral** — it's a pure ROS-param round trip needing no render readback, check 5 of `test_mode_dispatch.py` proves a mode switch performs no param write-back; the member-level AND-never-overwrite guarantee is GTest-covered by `compose_layer_gates()` (test_scene_assembly.cpp) (review round 2, 2026-09-11). |

### Named exceptions the sign-off explicitly accepts (not parity gaps to close)

1. **`exposure_match`/`fill_blind_zone` shipped forced-off** (Decision 3) —
   visible seam-brightness steps between cameras and an empty (not
   inward-filled) ring around the robot are expected, not a regression to
   chase. Neither has a Filament-side implementation this epic.
2. **Mode 2's lidar colorization is first-match, not the CUDA reference's
   feather-weighted multi-camera blend** (Decision 5 / Task 5) — visible
   seams at camera-boundary points are expected.
3. **The robot overlay's asset changes shape.** The deployed real-robot
   proxy is `M02P.obj` + an OBJ→rig `robot_model_transform`
   (`m2o1_params.yaml:19-20`, old `default_params.yaml:14`); the merged
   node's ego is `ego_model_path` (glTF; default `""` falls back to a plain
   box, `ego.cpp:106-111`). Unless an M02P glTF is provisioned for the
   robot's config (the repo already ships the converter,
   `cuda/src/libs/visual_renderer/scripts/obj2gltf_m02p.py` — an
   asset/config task, not new code, resolved at Task 6 Step 2b), the bowl's
   robot overlay silently degrades to a box on the real robot. Sign-off
   either sees the provisioned glTF or explicitly accepts the box.
4. **Task 2's `bowl.mat` contributes at most 2 cameras per fragment**, chosen
   per TRIANGLE (`bowl_mesh.cpp`) — across a genuine triple-camera-overlap
   band, adjacent triangles can switch which 2 of the 3 overlapping cameras
   they blend, a mesh-aligned faceted seam `reproject.cu`'s all-camera blend
   does not produce (review round 1 minor finding, 2026-09-11).
   `test_bowl.cpp`'s `ThreeCameraOverlapStillPicksAConsistentPairPerTriangle`
   pins today's behavior. Sign-off either accepts this seam or a future task
   adds a third CUSTOM-attribute slot to lift the cap.
5. **The Filament bowl mesh terminates at `r = bowl_Rmax`**; the CUDA
   reference surface clamps HEIGHT at `Rmax` and keeps intersecting rays
   against that flat cap out to a 10,000-unit ray-march bound — effectively
   unbounded for any real camera view. Beyond `Rmax`, the Filament bowl shows
   `sky_color` where the CUDA node shows clamped-height camera pixels —
   accepted as a named parity exception rather than chased.
6. **Task 4 (VM-093) Step 3's own finding, added here 2026-09-11:**
   `rendering_node`'s own `/rendering/image` publish rate falls short of
   30 Hz under REAL full six-camera + lidar load in BOTH a co-residence run
   (26.33 Hz) and an old-node-ALONE control run (23.05 Hz) — see
   `tools/budget_probe.md` Results (d) for the full write-up. The control run
   proves this is NOT a co-residence regression (the alone case is, if
   anything, slower) — it is a pre-existing, previously-unmeasured (named
   fixture gap #2) property of the unmodified CUDA `rendering_node` pipeline
   against a bag this heavy, out of this epic's scope to fix before Task 6
   decommissions that node entirely. Sign-off either accepts this as a known
   CUDA-node characteristic that Task 6's cutover moots (there will be only
   one process left to measure) or escalates it as its own investigation —
   a call for whoever runs Task 6's own on-robot rerun, not blocking Task 4.
7. **The environment/buildings layer (Epic 4/VM-052) is NOT gated per
   render_mode_, contrary to what Task 4's round-1 commit claimed (review
   round 2, 2026-09-11).** `set_environment_source(renderer, nullptr, ...)`
   cannot hide a live source: `environment.cpp`'s null/empty-`source_uri`
   guard (`if (r == nullptr || source_uri == nullptr || source_uri[0] ==
   '\0') return false;`) returns *before* the `if (r->environmentSource)
   teardown()` line ever runs, so a null call after a real one is a no-op,
   not a hide — `render_frame()` only checks `environmentSource != nullptr`.
   The node-side per-mode gate this round-1 commit added was therefore dead
   code (it flipped its own bookkeeping flag but never actually stopped
   buildings from rendering) and has been deleted rather than kept as
   false cover. **Buildings render in BOWL/HYBRID too, whenever
   `environment_chunks_dir` is provisioned and the geo-anchor has solved.**

   **CLOSED (library-side blocker) by VM-096 (the vcam GUI Environment
   Tiles toggle task)**: `set_environment_visible(VisualRenderer*, bool)`
   (`scene.h`/`environment.cpp`) now exists -- a free function (ADR-0004:
   bumps nothing) that hides/shows whatever `EnvironmentSource` is
   installed without tearing it down, implemented identically for
   `BakedEnvironmentSource` and `StreamingEnvironmentSource`. The node's
   `on_params()` wires the ALREADY-declared `environment_enabled` param to
   it live, and the vcam GUI/WS bridge expose it as a Switch. This closes
   the blocker this finding named ("the fix needs a library-side
   `set_environment_visible()`").

   **Still open, by design, not silently patched**: this toggle is
   OPERATOR-DRIVEN (a manual GUI/WS switch), not automatically tied to
   `render_mode_` -- buildings still render in BOWL/HYBRID exactly as
   before unless an operator explicitly hides them. Auto-gating
   `environment_enabled` per render_mode (so BOWL/HYBRID never shows
   buildings without an explicit opt-in) remains a follow-up task if
   product wants that default changed; the primitive it would need
   (`set_environment_visible()`) now exists.
8. **Lidar colorization samples cameras through BASE extrinsics; the bowl
   samples through ego-motion-delta-compensated extrinsics** (VM-094 review
   round 1 minor finding, 2026-09-11). `camera_ingest_->fill_bowl_intrinsics()`
   returns `state_.extrinsics(i)` verbatim, while `bowl.cpp`'s `update_bowl`
   composes each camera's `motionDelta` (from `update_motion_deltas()`) into
   right/fwd/t before the bowl shader projects. The CUDA reference feeds one
   compensated `scaled_params` array to both the bowl kernel and the splat
   colorization (`rendering_node.cpp:747-793`); this node's two consumers
   diverge. On a moving robot with odometry flowing, the same physical
   surface gets bowl pixels sampled from a compensated pose and splat colors
   from an uncompensated one -- the plan's own figure for that spread is
   ~0.5 m of travel at this bag's real camera-phase spread. **Not exercised
   by any capture taken so far**: this epic's fixture bag
   (`stack_v2_full_sensors_2026-09-09`) carries no odometry topic, so
   `update_motion_deltas()` computes identity deltas throughout and this gap
   is latent, not visible, in every golden captured against it. Accepted as
   a named parity exception rather than chased in this task; closing it
   means applying `camera_ingest_`'s own per-camera `compensation_delta_4x4`
   to the extrinsics handed to `ColorizeFromCameras` too, a follow-up task
   once a bag (or live robot) with real odometry is available to verify
   against.
