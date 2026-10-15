# Terrain and imagery runbook

How a streamed environment source carries terrain with imagery draped on it. A source can open a
second cesium tileset, its terrain, beside its content tileset, and imagery rides on that terrain
as cesium raster overlays. Two providers use the same path:

| Source | Terrain | Imagery | Credit on screen |
|--------|---------|---------|------------------|
| `mapbox://buildings,terrain?imagery=satellite` | Mapbox terrain-DEM, z14 tiles meshed by Overlume's loader | Mapbox Satellite, up to z18 | "© Maxar", inside the Mapbox line ([`mapbox.md`](mapbox.md) §5) |
| `ion://96188?terrain=1&imagery=2` | Cesium World Terrain (ion asset 1), quantized-mesh through cesium-native's own loader | Bing Maps Aerial (ion asset 2) through ion | ion's credits for the imagery asset (§5) |

The design is
[`2026-10-15-imagery-terrain-design.md`](../design/2026-10-15-imagery-terrain-design.md). The
Mapbox side (the layer list, `imagery=satellite`, the DEM, request volume, the token) is in
[`mapbox.md`](mapbox.md); the ion account and `CESIUM_ION_TOKEN` are in
[`cesium.md`](cesium.md) §1 and §3. This page covers what both share, and the ion keys.

## 1. What a terrain tileset changes

- **Ground.** The terrain replaces the clay ground plane once the height under the ego has been
  sampled from it. Until then, when the sample fails, and with `replaces_ground=off` on an
  `ion://` URI, the plane stays.
- **Terrain following.** `follow_terrain`, `ground_bias` and `max_tilt_deg` keep their meaning
  ([`cesium.md`](cesium.md) §5c), but sample the terrain tileset instead of the content tileset.
- **Buildings.** ion's buildings and Cesium World Terrain already agree, so nothing moves. Mapbox
  buildings are re-based onto the DEM ([`mapbox.md`](mapbox.md) §3).
- **Colour.** A terrain tile shows `environment.terrain_color` (§4) until its imagery attaches,
  for good when imagery is off, and where a raster tile failed.
- **Radius.** `radius=` selects the tiles of both tilesets.
- **Fallback.** Terrain and imagery requests count toward the network-loss threshold (eight
  consecutive failed requests). On fallback both tilesets are torn down, the clay plane comes back
  and the attribution goes, as before.
- Without a terrain tileset every key means what it meant before this feature.

## 2. ion terrain and imagery

`ion://<asset>?…` keeps every key of [`cesium.md`](cesium.md) and adds:

| Key | Values | Default | Meaning |
|-----|--------|---------|---------|
| `terrain` | an ion asset id, `off` | `off` | A second tileset: that ion terrain asset. `1` is Cesium World Terrain. |
| `imagery` | an ion asset id, `off` | `off` | That ion imagery asset, draped on the terrain as a raster overlay. `2` is Bing Maps Aerial. Needs `terrain`. |

- `imagery=<id>` without `terrain=<id>` refuses the URI, and so does a value that is neither an
  asset id nor `off`: the source does not open, one log line says why, and a live switch leaves
  the previous source in place.
- The terrain asset opens as cesium-native opens any ion asset: its endpoint names a `layer.json`
  (quantized-mesh), and cesium-native's own terrain loader streams it. Overlume adds no terrain
  code for ion.
- If the terrain tileset cannot open (no such asset on the account, or the token is not allowed
  on it), the source opens without terrain and logs once
  `environment source: terrain tileset did not open (HTTP <code>) -- continuing without terrain`.
  The buildings stream on the clay plane as before, and nothing falls back.
- Both tilesets and the overlay go through the source's one request path: `CESIUM_ION_TOKEN`, the
  redacting logger, the failure counter and, with `cache=<dir>`, the same tile cache.
- Terrain and imagery requests count toward the ion account's usage, like the buildings'.

### The `osm-terrain` GUI entry

| Entry | `environment_source_uri` | Look |
|-------|--------------------------|------|
| `osm-terrain` | `ion://96188?terrain=1&imagery=2` | Cesium OSM Buildings in the theme's clay, on Cesium World Terrain with Bing Maps Aerial |

- Pick it in the vcam GUI's environment sources list, send
  `{"cmd":"set_environment_source","preset":"osm-terrain"}` to the bridge, or pass
  `--arm osm-terrain` to `tools/validate_logger_session.sh`.
- It is the provider-agnostic proof: the overlay path `mapbox-satellite` uses, with no Mapbox code
  involved.

## 3. Node parameters

| Parameter | Values | Default | Effect on the composed URI |
|-----------|--------|---------|----------------------------|
| `environment_terrain` | `""`, `disable`, `enable` (`mapbox://`), `ion:<asset id>` (`ion://`) | `""` | `mapbox://`: `enable` adds `terrain` to the layer list when it is not there. `ion://`: `ion:<id>` appends `terrain=<id>`, and `disable` appends `terrain=off`, when the URI has no `terrain=` key. |
| `environment_imagery` | `""`, `disable`, `satellite` (`mapbox://`), `ion:<asset id>` (`ion://`) | `""` | Appended when the URI has no `imagery=` key: `imagery=satellite` on `mapbox://`, `imagery=<id>` on `ion://`, and `imagery=off` for `disable` on both. |

- `""` leaves the URI alone, and the URI always wins: `disable` cannot remove a `terrain` already
  in a `mapbox://` layer list, or a `terrain=` or `imagery=` key already in the URI. The GUI
  entries carry their keys, so they look the same on every deployment.
- A value meant for the other scheme appends nothing. When the configured
  `environment_source_uri` ignores a value, the node warns once at configure, for example
  `environment_terrain 'ion:1' is ignored for environment source 'mapbox://buildings' (accepted:
  "", disable, enable for mapbox://, ion:<asset id> for ion://)`. So a deployment can set
  `environment_terrain: ion:1` and `environment_imagery: ion:2` and still switch to the `mapbox`
  entry: `mapbox://buildings` gets neither. A value that fits neither scheme (`sat`, `ion:x`) gets
  the same WARN whatever the source. A live switch logs no WARN; its `environment source armed:
  '<uri>'` line shows the URI that was opened.
- Set `environment_imagery` only together with an `environment_terrain` value for the same
  scheme. Imagery without terrain refuses the URI (`mapbox://buildings?imagery=satellite`,
  `ion://96188?follow_terrain=off&imagery=2`), so every source that lacks terrain fails to open,
  and a live switch keeps the previous source.
- The values are strings. YAML reads `on`, `off`, `true`, `yes` and bare numbers as booleans and
  integers, and the node declares both parameters as strings, so such a value fails the configure
  transition (`parameter 'environment_terrain' has invalid type: …`). `enable`, `disable`,
  `satellite` and `ion:1` need no quotes, in a YAML file or after `-p`.
- Read at configure, and applied to every source the node opens afterwards, live switches
  included. With `environment_terrain: ion:1` and `environment_imagery: ion:2`, the `osm` entry
  streams like `osm-terrain`, and so does `google`, whose 3D Tiles carry their own ground: leave
  both unset where `google` is used.
- With `disable` (or `""`), terrain and imagery stay off unless the URI asks for them: these are
  the element's disable knobs.

On the replay rig:

```bash
tools/validate_logger_session.sh "$SESSION_DIR" --arm osm -- \
    --param environment_terrain:=ion:1 --param environment_imagery:=ion:2
```

## 4. The theme token `environment.terrain_color`

- `environment: { tile_radius_m: …, terrain_color: [r, g, b] }` in a theme YAML
  (`overlume/assets/themes/`), beside `tile_radius_m`; both shipped themes set it.
- Absent, it defaults to the theme's `palette.ground`.
- It colours a terrain tile until its imagery attaches, for good when imagery is off, and where a
  raster tile failed. Imagery replaces it, scaled by the source's `brightness`.

## 5. Attribution

- The library reports the open source's overlay credits as plain text
  (`environment_source_credits()`, `<overlume/scene.h>`: HTML stripped, `; ` between credits). The
  node reads them once a second and puts each after the source's base line, separated by a space.
- `mapbox://` with `imagery=satellite`: "© Mapbox © OpenStreetMap © Maxar Improve this map", with
  the Mapbox logo ([`mapbox.md`](mapbox.md) §5).
- `ion://` with `imagery=`: ion's credits for the imagery asset alone, bottom-left in the HUD
  text colour, with no logo; for the `google` source after "3D Tiles data (c) Google". Bing's
  credits make a long line, cut at the frame's right edge like any long line.
- An `ion://` source without imagery has no overlay credits and draws nothing, as before.
- `environment_attribution` (the disable knob) and the visibility rule are unchanged: free look
  (mode 3), `environment_enabled`, and the source state `STREAMING`.

## 6. The renderer's overlay path

For whoever debugs a missing or wrong texture:

- The terrain tileset has its own `StreamRendererResources`, kind `Terrain`. In
  `prepareInMainThread` it swaps each terrain tile's materials for an instance of `terrain.mat`
  (`overlume/assets/materials/terrain.mat`), as the clay swap does for buildings.
- `terrain.mat` is lit, with parameters `terrainColor` (float3), `hasImagery` (int, 0 until a
  raster attaches), `imagery` (sampler2d, trilinear, clamp to edge), `uvTransform` (float4:
  cesium's translation in xy and its scale in zw, uv' = uv × scale + translation) and
  `brightness` (float). It has no fade: terrain tiles pop in, as the buildings do. Base colour:
  the imagery texel times `brightness` when `hasImagery` is 1, else `terrainColor`.
- cesium-native writes each overlay's texture coordinates (`_CESIUMOVERLAY_0`) into every terrain
  tile's glTF; the GLB step copies them into `TEXCOORD_0`, which the material samples.
- The five overlay callbacks of `IPrepareRendererResources`: `prepareRasterInLoadThread` turns
  cesium's decoded image into an RGBA8 buffer; `prepareRasterInMainThread` makes one
  `filament::Texture` per raster tile, 8-bit RGBA in the sRGB format and mipmapped;
  `attachRasterInMainThread` gives the terrain tile's material instance the texture, cesium's
  translation and scale as `uvTransform`, and `hasImagery = 1`, in one frame, so no white or
  black flash shows; `detachRasterInMainThread` sets `hasImagery = 0` and binds the 1×1
  placeholder back, so the tile shows `terrain_color` again; `freeRaster` frees the texture
  through the stream's deferred-free path, never while a frame may sample it.
- Where the imagery is finer than the terrain (z18 imagery on z14 DEM tiles), cesium-native
  creates upsampled child tiles, so the imagery near the ego keeps its own level.
- On a source switch or teardown, overlays detach before either tileset is destroyed.
- `environment_stream_overlay_probe()` (`overlume/src/environment_test_hooks.hpp`) counts the
  rasters attached and detached and the textures alive and freed.

## 7. Checks

Offline, in the gate (`tools/ci_visual_mode.sh`):

- `test_mapbox_dem`: DEM decoding, sampling and the terrain mesh.
- `test_environment_stream`: the overlay callbacks over a file-based fixture tileset with
  file-URL imagery, the ion `terrain=`/`imagery=` keys, and the ion proof: terrain and imagery
  over fixture ion endpoints (a `layer.json` and quantized-mesh tiles encoded by the test).
- `test_mapbox_terrain`: Mapbox terrain and imagery end to end over the fixture network, and the
  goldens below.
- `test_environment_source_uri` (node): the two parameters and the attribution text.
- The goldens `mapbox_terrain_synthetic_dark_adas` and `mapbox_terrain_synthetic_light_clay`.

Offline, outside the gate (each launches the node; run them after a node change):
`ros/src/overlume_ros/test/test_asset_paths.py` reads both parameters back and checks the
configure WARN, and
`ros/src/overlume_ros/test/test_environment_live_switch.py` check 6 proves that activation and a
live switch both compose them.

Opt-in, never in the gate, each needing its token and the network:

```bash
OVERLUME_LIVE_ION=1 overlume/build/test_mapbox_terrain --gtest_filter='IonLive.*'
OVERLUME_LIVE_MAPBOX=1 overlume/build/test_mapbox_terrain --gtest_filter='MapboxLive.*'
```

`OVERLUME_LIVE_ION` is its own switch: the ion performance test's `OVERLUME_LIVE_ION_PERF` does
not run these, and this one does not run that.

## 8. Troubleshooting

- **An `ion://` source with `imagery=` does not open.** `imagery=` without `terrain=`, or a value
  that is neither an asset id nor `off`. Add `terrain=1`, or fix the value.
- **`environment source: terrain tileset did not open (HTTP 401) -- continuing without
  terrain`.** The token is not allowed on that asset; `HTTP 404`: the account has no such asset.
  The buildings still stream on the clay plane. Check the asset in the ion dashboard
  ([`cesium.md`](cesium.md) §2).
- **Terrain, but no imagery: the ground shows `environment.terrain_color`.** Raster tiles fail
  (429, 5xx or a transport error). Overlume logs no line of its own for them: cesium-native logs
  each failed tile through the source's redacting logger, `Received response code <code> for
  image <url>`, with the token shown as `access_token=<redacted>`. Failed raster requests count
  toward the network-loss threshold. A token refused on the imagery asset (401 or 403) does not
  leave the terrain bare for long: each failed tile makes cesium try its parent, the failures
  pass eight in a row, and the whole source falls back (below).
- **The clay plane stays under the terrain.** `replaces_ground=off`, or the height under the ego
  has not been sampled yet, or failed (no terrain tile under the ego).
- **Terrain, imagery and buildings disappear, and the node warns `environment source: network
  loss detected -- …`.** Both tilesets are gone and the clay plane is back. Fix the network, then
  select the source again ([`mapbox.md`](mapbox.md) §8 lists the ways).
- **The Bing credit is cut off at the right edge.** The line is wider than the frame. Known limit
  (design §12).
- **`osm` shows terrain and imagery.** `environment_terrain` and `environment_imagery` are set on
  this deployment (§3).
- **Configure fails with `parameter 'environment_terrain' has invalid type`.** The value is a YAML
  boolean or number (`on`, `off`, `1`). Use `enable`, `disable` or `ion:1` (§3).
- **Every switch to a source without terrain fails, and the previous source stays.**
  `environment_imagery` is set without an `environment_terrain` value for that scheme (§3).
