# Bowl golden — VM-091 Task 2 Step 7

Same frame (early steady-state, `~/TPSProjector-fixtures/stack_v2_full_sensors_2026-09-09`,
`ROS_DOMAIN_ID=93`, single-pass playback), real theme dir (`dark_adas`),
1280×720, quality 1 — captured from two independently-running nodes, not a
pixel diff (Global Constraints' Golden scoping rule; this samples through a
genuinely different mechanism than the CUDA backward sampler, per Decision 3):

- `overlume/tests/goldens/bowl_test_town_cuda_reference.png`
  — `micropilot_rendering_node`, `initial_mode:=1` (bowl-only), default params.
- `overlume/tests/goldens/bowl_test_town_dark_adas.png`
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

## Recapture, 2026-09-11: bowl color fidelity fix (sRGB camera textures + measured exposure)

User report: "the resulted stitching colors seems washed or way brighter
than the original frames." Two confirmed root causes, both fixed:

1. `camera_textures.cpp`'s `choose_camera_format()` was picking LINEAR
   RGB8/RGBA8 for camera pixels that are sRGB-encoded bytes (cv_bridge rgb8
   from the bgra8 wire) — sampled as linear, they got this renderer's own
   output OETF applied ON TOP of their own existing sRGB encoding, a
   double-encoding that reads as washed/brightened mid-tones. Fixed: SRGB8
   (fallback SRGB8_A8), so the sampler hardware-decodes sRGB->linear at
   sample time.
2. `bowl.mat`'s `exposureCompensation` was an EMPIRICAL GUESS (10.0, then
   1.5) picked by eyeballing a real bag capture — never a measurement, and
   specifically never re-measured against the corrected (post-fix) linear
   camera samples above. Replaced with a MEASURED value, 1.56
   (`tools/bowl_exposure_probe.cpp`'s gray-ramp binary search; see
   `scene.h`'s `BowlConfig::exposure_compensation` comment and
   `tests/test_bowl_exposure_calibration.cpp`, the standing regression).
   1.56 happens to sit very close to the prior 1.5 guess — a coincidence of
   this particular bag's midtones, not evidence the guess was secretly
   correct: the guess was never validated against a controlled input the
   way the gray ramp is.

`bowl_test_town_dark_adas.png` above is RECAPTURED against this fix (same
rig, same fixture bag, single-pass playback, `bowl_enabled:=true
initial_mode:=3 layer_surround_stitching:=true` — note `layer_surround_stitching`
is now required for the bowl to show under mode 3's free-look view; it did
not exist as a separate gate at the time of the original capture above).
`bowl_test_town_cuda_reference.png` is UNCHANGED (this fix touches only the
Filament port, never `micropilot_rendering_node`/reproject.cu).

Visual result: the new capture's road surface/buildings read as normal
daytime asphalt contrast, matching the CUDA reference far more closely than
the prior washed-out capture. Full library suite green (202/202, up from
188/188 at the time of the original capture — more tests landed since, plus
the 2 new exposure-calibration tests this fix adds), node rebuilt
(`visualization_node.hpp`'s `bowl_exposure_compensation_` default changed).

**This recapture is produced; it has not been human-sanity-approved yet**
(same Golden scoping rule as above) — committed as a candidate per the
promotion convention. Step 7's judgment call resets to this new capture.
