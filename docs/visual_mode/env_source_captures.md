# Environment-source comparison captures (VM-096)

**Look at this first:**
[`env_source_captures/env_source_contact_sheet.png`](env_source_captures/env_source_contact_sheet.png)
— baked/osm/google side by side. The raw `dark_adas` captures below are the
canonical, unmodified renders, but baked and osm are near-unjudgeable
straight off disk (p99 luminance 38.9/255 — see "Display-aid contact sheet"
below), so the contact sheet applies a documented gain lift to make the
building massing visible without editing the canonical images.

One render per GUI "Environment Tiles" preset, all at the **same geo anchor**
and the **same camera pose**, for a human to judge side by side. Produced by
the opt-in capture tests in
`overlume/tests/test_environment_stream.cpp`
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
constexpr overlume::GeoAnchor kFixtureAnchor{25.0803, 55.3910, 0.0};  // Dubai — this
    // file's own anchor, already used by every environment test in this repo
    // (test_environment.cpp, test_environment_stream.cpp).

constexpr overlume::Vec3 kCaptureBuildingsCentroid{-109.2, -17.1, 3.0};  // == test_environment.cpp's
    // kBuildingsCentroid, the baked fixture town's own real footprint centroid.

overlume::CameraPose{
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

Offline (baked, no token needed). Run from the repo root:

```bash
cd overlume/build
OVERLUME_CAPTURE_ENV_SOURCES=1 OVERLUME_CAPTURE_OUT_DIR=/tmp/env_source_capture \
    ./test_environment_stream --gtest_filter='EnvSourceCapture.Baked'
```

Live (osm / google) — needs `CESIUM_ION_TOKEN`. **Token-shadowing trap**: a
long-lived interactive shell can have an env var that a plain `bash -lc`
won't see (Ubuntu's `~/.bashrc` skips its exports under a non-interactive
shell), so run it exactly like this (per the epic's own documented
workaround):

Run from the repo root (same starting directory as the offline block above —
the inner `cd` below is the only one, so this block is self-contained and can
be pasted as-is):

```bash
env -u CESIUM_ION_TOKEN bash -lic '
    export OVERLUME_CAPTURE_ENV_SOURCES=1
    export OVERLUME_CAPTURE_OUT_DIR=/tmp/env_source_capture
    cd overlume/build
    ./test_environment_stream --gtest_filter="EnvSourceCapture.Osm"
' < /dev/null
```

(same for `EnvSourceCapture.Google`). Verify the token first with
`bash overlume/scripts/check_cesium_token.sh` (prints
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
[`EnvironmentStreamPerf.GooglePresetLiveRenderMsDeltaVsOsmClay`](../../overlume/tests/test_environment_stream.cpp)'s
own "manual visual-confidence artifact" (which stops at first-tile-loaded)
produced exactly that blank frame when inspected directly during this task.
Each live capture takes ~60–65s wall-clock; do not shrink the settle window
without re-verifying the output actually shows content.

After running, copy the PNG(s) from `OVERLUME_CAPTURE_OUT_DIR` into
`docs/visual_mode/env_source_captures/` (committed) under the names below.

## The captures

| File | Preset | Content-verification (measured this run) |
|---|---|---|
| [`env_source_captures/env_source_baked.png`](env_source_captures/env_source_baked.png) | baked | luminance stddev 5.90, non-background fraction 12.3% |
| [`env_source_captures/env_source_osm.png`](env_source_captures/env_source_osm.png) | osm (live, ion://96188) | luminance stddev 6.02, non-background fraction 10.5% |
| [`env_source_captures/env_source_google.png`](env_source_captures/env_source_google.png) | google (live, ion://2275207, original materials) | luminance stddev 24.60, non-background fraction 60.6% |

**Attribution caption (Google Map Tiles terms).** `env_source_google.png` is
committed imagery rendered from Google Photorealistic 3D Tiles — credit:
**"3D Tiles data (c) Google"**, the exact string the node composites live
onto frames showing this preset (`overlume_node.cpp`, the
`environment_attribution` HUD line, `cesium.md` §6's Attribution bullet).
The node renders it via the HUD text primitive, but this doc's PNG is a pure
library render (`overlume::render_frame`, see the note below) that never
touches that HUD code path, so the credit above is carried here as a
caption instead. The exact required wording is NOT re-verified per this
doc capture — same pre-go-live manual check `cesium.md` §6 already calls
out for the live node — confirm current wording against Google's Platform
Terms before treating this caption (or the live HUD string) as final.

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

## Display-aid contact sheet

The raw `dark_adas` captures above are the canonical, reproducible renders —
but two of the three are close to unjudgeable as shipped: measured p99
luminance is 38.9/255 for both **baked** and **osm** (only 0.04% of
pixels exceed luminance 40), vs. 117.2/255 for **google**. The content is
genuinely present and correct for a dark ADAS theme — this is expected
exposure, not a bug — but a human can't eyeball building massing at that
brightness without display help.

[`env_source_captures/env_source_contact_sheet.png`](env_source_captures/env_source_contact_sheet.png)
applies a flat **3.0× linear gain** (`pixel * 3.0`, clipped to 255) to the
**baked** and **osm** panels only — chosen so their p99 luminance
(~117, `38.9 * 3.0`) lands near google's own p99 (117.2), putting all three
panels at a comparable, readable brightness. **google** is shown unmodified
(already well-exposed). This is a display aid only, generated from the
already-committed PNGs above — not a re-render, not a replacement for them,
and not asserted on by any test. Regenerate with (run from the repo root;
needs Pillow + numpy):

```bash
python3 - <<'PYEOF'
from PIL import Image, ImageDraw, ImageFont
import numpy as np

GAIN = 3.0  # documented display-aid multiplier -- baked/osm only, see note above
SRC = {
    "baked": ("docs/visual_mode/env_source_captures/env_source_baked.png", True),
    "osm": ("docs/visual_mode/env_source_captures/env_source_osm.png", True),
    "google": ("docs/visual_mode/env_source_captures/env_source_google.png", False),
}
PANEL_W, LABEL_H = 480, 34
font = ImageFont.load_default()
panels = []
for name, (path, lift) in SRC.items():
    im = Image.open(path).convert("RGB")
    if lift:
        arr = np.clip(np.asarray(im, dtype=np.float64) * GAIN, 0, 255).astype(np.uint8)
        im = Image.fromarray(arr, "RGB")
    im = im.resize((PANEL_W, int(im.height * PANEL_W / im.width)), Image.LANCZOS)
    label = f"{name} ({GAIN:.1f}x gain, display aid)" if lift else f"{name} (as captured)"
    panel = Image.new("RGB", (PANEL_W, im.height + LABEL_H), (20, 20, 20))
    panel.paste(im, (0, LABEL_H))
    ImageDraw.Draw(panel).text((6, 8), label, fill=(255, 255, 255), font=font)
    panels.append(panel)
sheet = Image.new("RGB", (sum(p.width for p in panels) + 8 * (len(panels) + 1),
                           max(p.height for p in panels) + 16), (40, 40, 40))
x = 8
for p in panels:
    sheet.paste(p, (x, 8))
    x += p.width + 8
sheet.save("docs/visual_mode/env_source_captures/env_source_contact_sheet.png")
PYEOF
```

## "clipped" — not renderable on this box

The 4th GUI preset resolves to the node's own `environment_own_asset_uri`
parameter, resolved server-side in `tools/vcam_ws_bridge.py`'s
`set_environment_source` branch, not a fixed public ion asset id —
and that parameter defaults to `""` and is **not configured** on this box
(declared at `overlume_node.cpp:687`). There is no real ion asset id to point a
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
  `OVERLUME_CAPTURE_OUT_DIR` points).
- The licensing question above is scoped to tile BYTES, not the only thing
  Google's Map Tiles terms actually govern: those terms require attribution
  wherever the imagery is DISPLAYED (`cesium.md` §6), and a committed
  rendered PNG of that imagery is a display, same as the live node's
  on-screen HUD. That is why `env_source_google.png` above ships with the
  attribution caption in this doc rather than being treated as
  licensing-clear just because no tile bytes were committed.
