# Environment-source comparison captures (VM-096)

One render per GUI "Environment Tiles" preset, all at the **same geo anchor**
and the **same camera pose**, for a human to judge side by side. Produced by
the opt-in capture tests in
`cuda/src/libs/visual_renderer/tests/test_environment_stream.cpp`
(`EnvSourceCapture.*`) — never run by `ctest`/`ci_visual_mode.sh`, same
opt-in shape as `EnvironmentStreamPerf.GooglePresetLiveRenderMsDeltaVsOsmClay`
already in that file.

## The four presets (VM-096)

| GUI preset | `environment_source_uri` | This box |
|---|---|---|
| `baked`   | `""` (falls back to `environment_chunks_dir`) | committed fixture, offline |
| `osm`     | `ion://96188` | **LIVE** ion (Cesium OSM Buildings) |
| `google`  | `ion://2275207?materials=original&cache=off` | **LIVE** ion (Google Photorealistic 3D Tiles) |
| `clipped` | the node's own `environment_own_asset_uri` | **not configured** — no asset id exists to render (see below) |

## Anchor + pose (shared by every capture)

```cpp
constexpr mpviz::GeoAnchor kFixtureAnchor{25.0803, 55.3910, 0.0};  // Dubai — this
    // file's own anchor, already used by every environment test in this repo
    // (test_environment.cpp, test_environment_stream.cpp).

constexpr mpviz::Vec3 kCaptureBuildingsCentroid{-109.2, -17.1, 3.0};  // == test_environment.cpp's
    // kBuildingsCentroid, the baked fixture town's own real footprint centroid.

mpviz::CameraPose{
    {kCaptureBuildingsCentroid.x + 50, kCaptureBuildingsCentroid.y - 70, 40},
    {kCaptureBuildingsCentroid.x, kCaptureBuildingsCentroid.y, kCaptureBuildingsCentroid.z},
    60.0};
```

Chosen because it's the one map-frame point guaranteed non-empty for
**every** renderable source: the baked town's buildings sit exactly there by
construction, and `kFixtureAnchor` is a real Dubai location, so Cesium OSM
Buildings / Google Photorealistic 3D Tiles both have genuine content there
too — the whole point of a side-by-side comparison is that all three sources
show *something* at the identical spot. The eye/target offset itself is not
new: it's the same framing
`test_environment.cpp`'s `SetEnvironmentVisibleFalseHidesLoadedChunksWithoutTearingDown`
already uses for this exact centroid.

Renders are 960×720 (generous — for eyeballing, not SSIM; `golden.hpp`'s
`render_and_compare()` is fixed at 320×240 for CI speed and is deliberately
**not** used here), theme `dark_adas`.

## How to re-run each capture

Offline (baked, no token needed):

```bash
cd cuda/src/libs/visual_renderer/build
MPVIZ_CAPTURE_ENV_SOURCES=1 MPVIZ_CAPTURE_OUT_DIR=/tmp/env_source_capture \
    ./test_environment_stream --gtest_filter='EnvSourceCapture.Baked'
```

Live (osm / google) — needs `CESIUM_ION_TOKEN`. **Token-shadowing trap**: a
long-lived interactive shell can have an env var that a plain `bash -lc`
won't see (Ubuntu's `~/.bashrc` skips its exports under a non-interactive
shell), so run it exactly like this (per the epic's own documented
workaround):

```bash
cd cuda/src/libs/visual_renderer/build
env -u CESIUM_ION_TOKEN bash -lic '
    export MPVIZ_CAPTURE_ENV_SOURCES=1
    export MPVIZ_CAPTURE_OUT_DIR=/tmp/env_source_capture
    cd cuda/src/libs/visual_renderer/build
    ./test_environment_stream --gtest_filter="EnvSourceCapture.Osm"
' < /dev/null
```

(same for `EnvSourceCapture.Google`). Verify the token first with
`bash cuda/src/libs/visual_renderer/scripts/check_cesium_token.sh` (prints
`PASS`/`FAIL` only — never the token itself).

Each live test pumps up to 30s for the first tile, THEN keeps rendering for
**60 more real seconds** (paced with a 100ms sleep between ticks, not a tight
loop) before the capture frame. That settle phase is load-bearing, not
padding: the first tile to load is the tileset's coarse root, and a snapshot
taken right there renders as a totally flat, empty frame — real per-building
detail only appears after several more rounds of network-gated LOD
refinement, which needs actual elapsed wall-clock time (not just more
`render_frame()` ticks — those run in well under a millisecond each once
nothing new is happening, so a tight loop burns hundreds of ticks in under a
second of real time and gets nothing new back). Reproduced directly:
[`EnvironmentStreamPerf.GooglePresetLiveRenderMsDeltaVsOsmClay`](../../cuda/src/libs/visual_renderer/tests/test_environment_stream.cpp)'s
own "manual visual-confidence artifact" (which stops at first-tile-loaded)
produced exactly that blank frame when inspected directly during this task.
Each live capture takes ~60–65s wall-clock; do not shrink the settle window
without re-verifying the output actually shows content.

After running, copy the PNG(s) from `MPVIZ_CAPTURE_OUT_DIR` into
`docs/visual_mode/env_source_captures/` (committed) under the names below.

## The captures

| File | Preset | Content-verification (measured this run) |
|---|---|---|
| [`env_source_captures/env_source_baked.png`](env_source_captures/env_source_baked.png) | baked | luminance stddev 5.97, non-background fraction 11.7% |
| [`env_source_captures/env_source_osm.png`](env_source_captures/env_source_osm.png) | osm (live, ion://96188) | luminance stddev 6.02, non-background fraction 10.5% |
| [`env_source_captures/env_source_google.png`](env_source_captures/env_source_google.png) | google (live, ion://2275207, original materials) | luminance stddev 24.60, non-background fraction 60.6% |

"Non-background fraction" = share of pixels whose luminance differs from a
corner-pixel background sample by more than 10 levels — near-zero for a
blank/sky-only frame, well above zero once real geometry/texture is on
screen. Numbers are reported, never asserted on (`EXPECT_GT(loaded, 0u)` in
the test itself is the real pass/fail signal for "did this source load
anything at all").

**What each image shows:**

- **baked** — the offline fixture town's clay-shaded buildings (dark_adas
  theme), same content `tests/goldens/environment_test_town_dark_adas.png`
  already pins, viewed from this comparison package's shared pose.
- **osm** (Cesium OSM Buildings, clay remap — the GUI's default live preset) —
  real OSM-derived building massing near `kFixtureAnchor` (foreground clay
  blocks) plus a distant skyline silhouette on the horizon. Confirms the clay
  remap path renders live ion content, not just the fixture.
- **google** (Google Photorealistic 3D Tiles, `materials=original` — original
  textures, no clay remap) — real aerial-photogrammetry imagery: streets,
  rooftops, ground texture, at the LOD this box's network fetched within the
  60s settle window (still visibly low-resolution/blurry at close range —
  expected, not a bug: deeper LOD keeps refining with more real time).

## "clipped" — not renderable on this box

The 4th GUI preset resolves to the node's own `environment_own_asset_uri`
parameter (`environment_source_uri.hpp`), not a fixed public ion asset id —
and that parameter defaults to `""` and is **not configured** on this box
(`visualization_node.cpp:687`). There is no real ion asset id to point a
capture at; fabricating one would silently render some *other*, unrelated
asset and mislabel it "clipped". `EnvSourceCapture.Clipped` always
`GTEST_SKIP()`s with this exact reason (even with the capture opt-in set) —
the case is named and reported as not-renderable, not silently dropped from
the comparison package. No PNG exists for this preset.

## What is / isn't committed

- Committed: the 3 PNGs above (`docs/visual_mode/env_source_captures/`, ~2.2 MB
  total) and this note.
- Not committed: Google tile bytes (licensing — the epic's own recorded
  constraint; only the rendered PNG of the view is shipped), any
  `CESIUM_ION_TOKEN`/session token/ion response body (never printed, logged,
  or committed by any script or test in this package), and the raw capture
  working directory (`/tmp/env_source_capture` or wherever
  `MPVIZ_CAPTURE_OUT_DIR` points).
