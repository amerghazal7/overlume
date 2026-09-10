# Bowl golden — VM-091 Task 2 Step 7

Same frame (early steady-state, `~/TPSProjector-fixtures/stack_v2_full_sensors_2026-09-09`,
`ROS_DOMAIN_ID=93`, single-pass playback), real theme dir (`dark_adas`),
1280×720, quality 1 — captured from two independently-running nodes, not a
pixel diff (Global Constraints' Golden scoping rule; this samples through a
genuinely different mechanism than the CUDA backward sampler, per Decision 3):

- `cuda/src/libs/visual_renderer/tests/goldens/bowl_test_town_cuda_reference.png`
  — `micropilot_rendering_node`, `initial_mode:=1` (bowl-only), default params.
- `cuda/src/libs/visual_renderer/tests/goldens/bowl_test_town_dark_adas.png`
  — `micropilot_visualization_node` (this epic's Filament port), `bowl_enabled:=true`,
  `initial_mode:=3` (mode 3's own autonomy view, bowl composited underneath —
  Task 4 has not landed the bowl-mode dispatch yet, so this is the merged
  node's normal camera, not a top-down bowl-only view the way the CUDA
  capture is).

## exposure_compensation retuned 10.0 → 1.5

The first capture at the shipped 10.0 (picked before any real camera frame
had gone through this material) blew the bowl out to near-white — road
markings and background buildings were barely visible against a washed-out
ground plane, next to the CUDA reference's normal daytime asphalt. Retuned
down (10.0 → 3.0 → 1.5, three captures) until the bowl's overall brightness
and road-surface contrast matched the CUDA reference on the same frame.
`BowlConfig::exposure_compensation`'s default, `bowl.mat`'s header/comment,
`default_params.yaml`'s `bowl_exposure_compensation`, and
`visualization_node.hpp`'s member default were all updated to 1.5 (see their
own comments). Full library suite (188/188) and node suite (238/238)
re-confirmed green after the retune.

## Sanity check (not a pixel diff)

Both captures show the same recognizable scene (paved road, lane markings,
the ego robot's rear proxy mesh, a nearby vehicle marker, roadside
buildings/police-car prop) at comparable exposure. The mode-1 CUDA capture
is a clean top-down-ish bowl-only view; the merged-node capture is mode 3's
own forward/above autonomy camera with the bowl ground plane visible
underneath the HUD/HD-map/object overlays (expected — Task 4 has not yet
wired the bowl-mode dispatch that would show mode 1 as its own top-down
view from this node). The CUDA capture also shows a visibly glitched/
flipped police-car sprite near the frame's right edge — a pre-existing CUDA
reprojector artifact at extreme viewing angles, unrelated to this port.

**This capture is produced and exposure-matched; it has not been
human-sanity-approved yet** (Golden scoping rule: this step's actual
acceptance criterion). Plan Step 7 is left unchecked pending that review —
see both PNGs above.
