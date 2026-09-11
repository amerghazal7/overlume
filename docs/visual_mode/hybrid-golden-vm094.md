# Hybrid golden — VM-094 Task 5 Step 5

Same fixture (`~/TPSProjector-fixtures/stack_v2_full_sensors_2026-09-09`,
`ROS_DOMAIN_ID=93`, single-pass playback, `--start-offset 40`), 1280×720 —
captured from two independently-running nodes, not a pixel diff (same Golden
scoping rule and same reason bowl-golden-vm091.md gives: first-match
colorization is a genuinely different mechanism than the CUDA feather-blended
sampler, Decision 5's own named fidelity exception):

- `cuda/src/libs/visual_renderer/tests/goldens/hybrid_test_town_cuda_reference.png`
  — `micropilot_rendering_node`, `initial_mode:=2` (CUDA hybrid: bowl +
  every-camera feather-blended lidar colorization), default params
  (`pointcloud_transform` t=(0,0,1.15) is the shipped default — see review
  round 1 finding 1 below).
- `cuda/src/libs/visual_renderer/tests/goldens/hybrid_test_town_merged_node.png`
  — `micropilot_visualization_node` (this epic's Filament port),
  `bowl_enabled:=true render_mode:=2 hybrid_enabled:=true
  pointcloud_topic:=/iv_points_fusion` with `default_params.yaml`'s
  `pointcloud_transform` (mode content exclusivity, USER DIRECTIVE
  2026-09-11: bowl + camera-colorized lidar + ego only, enforced by
  `mode_content_mask(HYBRID)`).

## Review round 1 finding 1: pointcloud_transform now matches across nodes

The prior capture ran the merged node with an IDENTITY `pointcloud_transform`
while the CUDA node's own shipped `default_params.yaml` carries t=(0,0,1.15)
(`rendering_node.cpp:91-94`: the fused `/iv_points_fusion` cloud is in the
calib-ego/top-lidar frame and needs the same +z ground offset the camera
extrinsics bake in) — the two nodes placed the same cloud 1.15 m apart under
their own defaults. `micropilot_visualization_node/config/default_params.yaml`
now ships t=(0,0,1.15) too (not an `m2o1_params.yaml`-only override — the old
node's OWN shipped default carries it), and both PNGs above were re-captured
against that same transform.

## Review round 1 finding 3: honest hybrid-on/hybrid-off A/B

The previous sanity section claimed the visible road/vehicle-surface speckle
in the merged-node capture WAS the camera-colorized lidar. It is not: a
same-offset, same-bag, same-params A/B (`render_mode:=2`, identical except
`hybrid_enabled:=true` vs `hybrid_enabled:=false`) shows that speckle is
present with hybrid OFF too — it is the camera-textured bowl's own per-camera
photographic noise/JPEG-ish artifacting, not lidar splats.

Measured over two fixed road-surface ROI strips (excluding the ego proxy),
same capture session as the two golden PNGs above:

| | hybrid_on | hybrid_off |
|---|---|---|
| Road-region high-frequency energy (Laplacian variance) | 95.662 | 94.969 |

Mean absolute per-pixel delta between the two full frames: **0.493** (8-bit
scale) — the two captures are visually indistinguishable. The colorized
splats DO render — this node's own `hybrid: colorized N/M lidar points
(P% coverage)` log line (throttled every 5s) confirms real, substantial
per-tick output during the capture window:

```
hybrid: colorized 78826/159927 lidar points (49.3% coverage)
hybrid: colorized 92972/157555 lidar points (59.0% coverage)
hybrid: colorized 93338/156959 lidar points (59.5% coverage)
hybrid: colorized 92459/154355 lidar points (59.9% coverage)
hybrid: colorized 92463/153132 lidar points (60.4% coverage)
```

Steady state at this frame: **~93k of ~155k input points colorized per tick
(~60% coverage)** — the rest are dropped, not appended with a sentinel color
(`lidar_colorize.cpp`'s Decision 5 first-match-or-drop rule: a point no
configured camera's frustum covers this tick is simply omitted).

So ~93k real colored points ARE pushed into the scene every tick, but on
this frame they sit ON the bowl surface. **Review round 2 correction:** the
previous version of this section claimed the CUDA reference frame has no
bowl underneath its splats — false, and contradicted by this doc's own
header above: the reference capture is `micropilot_rendering_node
initial_mode:=2`, i.e. CUDA mode 2, which is bowl + colorized lidar, same as
the merged node. Both frames are bowl-textured underneath.

The real mechanism is on-surface vs. off-surface: a lidar point that lands
ON the bowl surface (the ground, in this frame — the intersection is empty
of near-field 3-D structure) projects to essentially the SAME screen
position and SAME camera pixel the bowl fragment shader already shows
there, in both renderers — camera-colorized lidar painted back onto the
photograph it was sampled from reads as close to invisible regardless of
which renderer does it. A lidar point on real 3-D structure ABOVE the bowl
floor (a vehicle, a pedestrian, anything inside `Rmax` that isn't ground)
would project to a DIFFERENT screen position than the bowl surface's own
smear of that same geometry, and the splat would be visible there in either
renderer. **This frame contains no such object** — the previous version's
"blocky overturned car silhouette" is retracted: no such object is
identifiable in the committed reference PNG, and the claim should not have
been made.

This is the thing the human-sanity reviewer must actually judge, stated
honestly: this frame cannot demonstrate whether hybrid content adds visible
3D structure, because it contains no near-field 3-D object to demonstrate
it on — both renderers show near-invisible on-surface colorization here, by
construction, not by a property specific to this node. Human sign-off on
the ON-surface/OFF-surface distinction itself should be taken on a
re-captured frame containing a near-field object inside `Rmax` (a vehicle or
pedestrian in the intersection), where the two renderers' splats would
diverge from the bowl's own texture in a visually checkable way. That
re-capture has not been done as part of this fix.

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
unchecked pending that review — see both PNGs above, and the honest A/B
framing above for what to actually judge.
