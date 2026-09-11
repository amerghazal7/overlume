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
| Bowl-vs-CUDA visual parity | Task 2 (VM-091)'s golden (`docs/visual_mode/bowl-golden-vm091.md`) + a live side-by-side | Golden captured; still needs human-sanity approval (Task 2 Step 7 left unchecked pending that, not code work) |
| Hybrid-vs-CUDA visual parity | Task 5 (VM-094)'s golden | Not started (Task 5 not yet landed) |
| Self-view/robot-proxy parity | Task 3 (VM-092)'s golden | Not started (Task 3 not yet landed) |
| Perf parity at each milestone | This epic's own perf-gate steps (Task 2 Step 5, Task 4 Steps 2/3, Task 5's own gate, Task 6's on-robot rerun) | Task 2 Step 5 + Task 4 Steps 2/3 closed (see `tools/budget_probe.md` Results (c)/(d)); Task 5/Task 6 rows open |
| Node-level bowl-visible/bowl-hidden pixel assertions for the per-mode dispatch (BOWL renders bowl, FREE_LOOK hides it, FREE_LOOK+Surround-Stitching shows it alongside the autonomy scene) + node-level "autonomy layers do not render in BOWL with live data flowing" pixel check, both reusing Task 2's sentinel-magenta check against a LIVE node render | Task 4 (VM-093) review round 1, deferred to Task 6's golden captures | Not started -- no node-level camera-publishing/render-readback harness exists in this package today (`test_bowl.cpp`'s pixel/sentinel machinery is library-side, fenced by the concurrent VM-092 task for this task's duration); `bowl_visible_for_mode()`/`overlays_visible_for_mode()` are unit-tested directly (`test_scene_assembly.cpp`) as the cheaper stand-in, but that is a mask/predicate check, not a live-pixel one. **BOWL→FREE_LOOK→BOWL `layer_*` restore is NOT part of this deferral** — it's a pure ROS-param round trip needing no render readback, check 5 of `test_mode_dispatch.py` proves a mode switch performs no param write-back; the member-level AND-never-overwrite guarantee is GTest-covered by `compose_layer_gates()` (test_scene_assembly.cpp) (review round 2, 2026-09-11). |

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
   `environment_chunks_dir` is provisioned and the geo-anchor has solved** —
   an accepted gap, not silently patched, since the fix needs a library-side
   `set_environment_visible()` (or a reordered `set_environment_source()`
   that tears down before its null/empty-uri early return) and the library
   is fenced by the concurrent VM-092 task for this task's duration. Sign-off
   either accepts buildings appearing behind the bowl in BOWL/HYBRID when
   `environment_chunks_dir` is set, or a follow-up task adds the library-side
   toggle.
