# Hybrid golden — VM-094 Task 5 Step 5

Same fixture (`~/TPSProjector-fixtures/stack_v2_full_sensors_2026-09-09`,
`ROS_DOMAIN_ID=93`, single-pass playback), 1280×720 — captured from two
independently-running nodes, not a pixel diff (same Golden scoping rule and
same reason bowl-golden-vm091.md gives: first-match colorization is a
genuinely different mechanism than the CUDA feather-blended sampler,
Decision 5's own named fidelity exception):

- `cuda/src/libs/visual_renderer/tests/goldens/hybrid_test_town_cuda_reference.png`
  — `micropilot_rendering_node`, `initial_mode:=2` (CUDA hybrid: bowl +
  every-camera feather-blended lidar colorization), default params
  (`pointcloud_topic: /iv_points_fusion` is already its shipped default).
- `cuda/src/libs/visual_renderer/tests/goldens/hybrid_test_town_merged_node.png`
  — `micropilot_visualization_node` (this epic's Filament port),
  `bowl_enabled:=true render_mode:=2 hybrid_enabled:=true
  pointcloud_topic:=/iv_points_fusion` (mode content exclusivity, USER
  DIRECTIVE 2026-09-11: bowl + camera-colorized lidar + ego only, enforced
  by `mode_content_mask(HYBRID)`).

## Sanity check (not a pixel diff)

Both captures show the same recognizable close chase-cam scene (paved
road, lane markings, the ego robot's rear proxy, nearby crashed/parked
vehicles, roadside buildings) with a visibly speckled point texture
scattered across the road and vehicle surfaces in BOTH captures — the
camera-colorized lidar splats, present in both the CUDA reference and the
merged node's own output. The merged node's capture shows the SAME
camera-boundary artifacts the bowl golden already named (glitchy/rotated
vehicle sprites where two camera projections meet) — expected here too,
since HYBRID's bowl fallback is Task 2's own bowl, unchanged. The named
Decision 5 fidelity regression (first-match camera-colorization vs. the
CUDA reference's every-camera feather blend) is not distinguishable by eye
at this resolution/exposure in this one frame — a seam artifact specific to
first-match (a lidar point's color flipping between two cameras' pixel
values at a coverage boundary) would need a frame with a point cloud
concentration exactly straddling two cameras' overlap to show clearly; not
hunted for in this one capture.

## Perf gate (Task 5 Step 4, `hybrid_perf_gate.sh`)

Real fixture bag, dev-box proxy (RTX 3090-class, named fixture gap #1 still
open — no on-robot number exists yet, closes at Task 6), `/iv_points_fusion`
point count sampled from the bag: `width=163399 height=1` (an unorganized
cloud, 163399 points/message).

| case | render_mode | hybrid_enabled | image_hz | render_ms p50 | render_ms p99 |
|---|---|---|---|---|---|
| bowl_baseline | 1 (BOWL) | false | 29.695 | 10.792 | 23.128 |
| hybrid_on | 2 (HYBRID) | true | 30.271 | 11.430 | 23.071 |

**Pass** against Step 4's own gate (`image_hz >= 30.0` with hybrid active,
`render_ms p99 <= 33 ms`): 30.271 Hz, 23.071 ms p99. The colorization CPU
cost specifically (render_ms delta, hybrid vs. bowl-only, same methodology
Task 2's own perf gate used for the bowl bake's cost): p50 +0.638 ms, p99
essentially flat (-0.057 ms, within run-to-run noise) — the per-point
camera-match loop (163k points × up to 6 `ProjectToCameraUv` calls each,
worst case) is cheap relative to the bowl's own per-frame cost at this
point count. No new warnings beyond the pre-existing `camera bowl: stamp
spread` (unrelated, present in both cases — the fixture bag carries no
odometry topic) and the pre-existing
`/perception/dynamic_objects_list: dropped ... malformed` (unrelated,
present in both cases, a known fixture-format gap).

## Not yet human-sanity-approved

**This capture is produced; it has not been human-sanity-approved yet**
(Golden scoping rule: this step's actual acceptance criterion, same as
bowl-golden-vm091.md's own closing line). Plan Task 5 Step 5 is left
unchecked pending that review — see both PNGs above.
