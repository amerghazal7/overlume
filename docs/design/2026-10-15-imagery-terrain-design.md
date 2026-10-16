# Imagery on terrain: provider-agnostic terrain tilesets with raster overlays

**Status:** draft for the maintainer's review, 2026-10-15. **Branch:** `mapbox-environment`
(sub-project 2 continues on the branch of sub-project 1, so pull request #1 tracks both).
**Scope:** sub-project 2 of 2. Sub-project 1, the Mapbox buildings source, is
[`2026-10-14-mapbox-buildings-design.md`](2026-10-14-mapbox-buildings-design.md); its section 13
named this work.

## 1. Context and goal

Overlume streams environment content through cesium-native: ion 3D Tiles (`ion://`) and, since
sub-project 1, Mapbox landmark models and extruded footprints (`mapbox://buildings`). Both stand
on the flat map plane; the renderer's raster-overlay callbacks are stubs; there is no terrain
tileset. This sub-project adds real terrain under the buildings with imagery draped on it, for the
demo to Mapbox and the public release, with the interfaces provider-agnostic: Mapbox Satellite on
Mapbox terrain-DEM for the demo, and Cesium World Terrain with ion imagery wired through the same
interfaces as the proof that nothing is Mapbox-specific.

Non-goals: Mapbox style features, vector overlays, a DEM finer than z14, imagery on buildings,
water masks, 3D Tiles terrain formats other than quantized-mesh (ion) and DEM (Mapbox).

## 2. User decisions (2026-10-15)

| # | Decision |
|---|----------|
| D1 | Imagery on real terrain now, in one sub-project (not imagery on the flat plane first). |
| D2 | Providers on day one: Mapbox Satellite on Mapbox terrain-DEM (the demo) and Cesium World Terrain with ion imagery (the proof), both gated offline with fixtures. A generic `imagery=xyz:` key is not in scope. |
| D3 | Shape A: terrain as a second cesium tileset per source, imagery as cesium raster overlays on it, Mapbox buildings re-based onto the DEM. |
| D4 | GUI entries (sources, never "presets"): `mapbox-satellite`, `mapbox-original-satellite`, `osm-terrain`. |
| D5 | Same branch, PR #1 stays open and tracks this feature once delivered. |

Standing rules from sub-project 1 apply unchanged: tokens by name only; no Mapbox bytes in the
tree; no disk cache for Mapbox data; the attribution on screen whenever Mapbox content shows; every
behaviour change ships a check that fails when reverted; goldens promoted by a human.

## 3. Facts the design rests on

### Mapbox (Raster Tiles API, probed 2026-10-14 unless noted)

- Satellite: `https://api.mapbox.com/v4/mapbox.satellite/{z}/{x}/{y}@2x.jpg90?access_token=…`,
  512 px JPEG, z 0–22; in the UAE the detail stops near z18, so the overlay is capped at 18.
- Terrain-DEM: `https://api.mapbox.com/v4/mapbox.mapbox-terrain-dem-v1/{z}/{x}/{y}@2x.pngraw?access_token=…`,
  512 px RGB PNG, z ≤ 14; height in metres above sea level = −10000 + (R·65536 + G·256 + B)·0.1.
  The plan's first task confirms both URL forms with two requests, DEM and satellite, behind a consent
  step (codes and PASS/FAIL only). The satellite form above writes Mapbox's XYZ row; the template
  handed to cesium writes it as `{reverseY}` (section 5).
- Empty or missing tiles answer 204 or 404; 401/403 mean a rejected token; the terms forbid a disk
  cache and require attribution ("© Mapbox © OpenStreetMap", "Improve this map", and "© Maxar" for
  satellite imagery) plus the logo.

### cesium-native v0.64 (the build's `_deps/cesium-native-src`)

- `Cesium3DTilesSelection::Tileset` accepts an ion asset id; its ion loader detects terrain assets
  (`layer.json`, quantized-mesh) and loads them through `LayerJsonTerrainLoader` with no code of ours.
- `Tileset::getOverlays()` is a `RasterOverlayCollection`; overlays available: `UrlTemplateRasterOverlay`
  (`{x}`, `{y}`, `{z}` templates, `minimumLevel`/`maximumLevel`, tile size, credit),
  `IonRasterOverlay` (an ion imagery asset), Bing, Google Map Tiles, TMS, WMS, WMTS.
- `TilesetContentManager` generates the overlay texture coordinates (`_CESIUMOVERLAY_n`) for every
  loader's glTF after it loads, and creates upsampled child tiles (`RasterOverlayUpsampler`) where the
  mapped imagery is finer than the geometry, so a z14-only terrain tileset still carries fine imagery near
  the ego: the overlay is capped at z18, and with the default geometric error and cesium's 2048-px
  texture budget the refinement reaches z17 at the default radius (Task 10's measurement).
- `IPrepareRendererResources` has five overlay callbacks: `prepareRasterInLoadThread`,
  `prepareRasterInMainThread`, `freeRaster`, `attachRasterInMainThread(tile, overlayTextureCoordinateID,
  rasterTile, mainThreadResources, translation, scale)`, `detachRasterInMainThread`. Overlume's
  implementations are stubs today (`overlume/src/environment_stream.hpp`).
- `Tileset::sampleHeightMostDetailed` works on any tileset with geometry: Overlume's terrain following
  uses it today on the ion content tileset.

### Overlume today

- `StreamingEnvironmentSource` (`overlume/src/environment_stream.{hpp,cpp}`) owns one `Tileset`, a
  `CountingAssetAccessor` (eight consecutive failed requests → `STREAMING_FALLBACK`), the
  follow-terrain state (five along-track height samples → a plane fit → one offset-and-tilt transform
  on the tileset root, `ground_bias`, `max_tilt_deg`), `replaces_ground` → `provides_ground()` (the
  renderer hides the clay plane), and `StreamRendererResources`, which serialises each tile's glTF
  to a GLB in the load thread and loads it with gltfio in the main thread, swapping materials for clay.
- The Mapbox buildings loader (`overlume/src/mapbox_loader.cpp`) is a custom `TilesetContentLoader`:
  a z14 quadtree within `radius`, 17 requests per tile, three attempts with backoff, per-tile frames
  (`mapbox_tile_frame`), stats in `MapboxSourceStats` (`mapbox_test_hooks.hpp`).
- Filament materials of our own live in `overlume/assets/materials/*.mat` (compiled by matc; the
  height grid has `height_grid.mat` and `height_grid_faded.mat`).
- The node composes `environment_source_uri` from parameters (`environment_source_uri.hpp`), draws the
  attribution (`environment_attribution_kind`, `kMapboxAttributionText`, `CompositeAttribution`), and
  the vcam bridge maps GUI entries to URIs (`ENVIRONMENT_PRESET_URIS` in `tools/vcam_ws_bridge.py`).

## 4. Configuration

### URI

`mapbox://<layers>?<keys>`. The layer list is a comma list of `buildings` and `terrain`, each at most
once, in any order; empty, repeated or unknown names refuse the URI with one log line, as the single
name does today. Keys:

| Key | Values | Default | Meaning |
|-----|--------|---------|---------|
| `materials` | `clay`, `original` | `clay` | As today (buildings). |
| `radius` | finite > 0, metres | 500 | Selection radius for both tilesets. |
| `brightness` | finite > 0 | 1 | Scales `materials=original` base colours and the imagery texture. |
| `fallback` | directory | none | As today. |
| `cache` | `off` | `off` | As today; anything else refuses the URI. |
| `imagery` | `satellite`, `off` | `off` | Mapbox Satellite draped on the terrain. Requires `terrain` in the layer list; otherwise the URI is refused. |

`ion://<asset>?<keys>` keeps every key and adds:

| Key | Values | Default | Meaning |
|-----|--------|---------|---------|
| `terrain` | an ion asset id, `off` | `off` | A second tileset: the ion terrain asset (`1` is Cesium World Terrain). |
| `imagery` | an ion asset id, `off` | `off` | An `IonRasterOverlay` on the terrain tileset (`2` is Bing Maps Aerial). Requires `terrain`. |

With a terrain tileset, `follow_terrain`, `ground_bias` and `max_tilt_deg` steer the shared transform
from samples on the terrain tileset, and `replaces_ground` means the terrain owns the ground; without
one, every key means what it means today. A `mapbox://` source whose layer list has `terrain` runs
with the ion defaults fixed (follow-terrain on, `ground_bias` 0.3, `replaces_ground` on,
`max_tilt_deg` 2.0); the Mapbox URI carries no such keys.

### Node

Two configure-time string parameters, default `""` (leave the URI alone), appended by the composer
only when the URI lacks the key. Their values are strings YAML never reads as a bool or an int:
`environment_terrain` is `disable`, `enable` (for `mapbox://`, adds `terrain` to the layer list) or
`ion:<asset id>` (for `ion://`, appends `terrain=<id>`; `disable` appends `terrain=off`);
`environment_imagery` is `disable`, `satellite` (for `mapbox://`) or `ion:<asset id>` (for `ion://`).
A value meant for the other scheme is ignored with one WARN line at configure. The URI always
wins: `disable` cannot remove a key a URI carries, so the GUI entries stay explicit. Both are the
element's disable knobs for URIs that do not say.

### Theme

`environment.terrain_color` (RGB, default the theme's ground colour) joins `environment.tile_radius_m`
in both theme files: the terrain tile's colour until its imagery attaches, and when imagery is off.

### GUI

Three new entries in the bridge's list and the GUI's selector (`ENVIRONMENT_PRESETS` keeps its
identifier; the entries are environment sources):

| Entry | URI |
|-------|-----|
| `mapbox-satellite` | `mapbox://buildings,terrain?imagery=satellite` |
| `mapbox-original-satellite` | `mapbox://buildings,terrain?materials=original&imagery=satellite` |
| `osm-terrain` | `ion://96188?terrain=1&imagery=2` |

### Attribution

`environment_source_credits()` (section 5) returns the open source's overlay credits as plain text;
the node reads it once per second. The node's line becomes the kind's base text, then each credit
prefixed by a space, then (Mapbox) "Improve this map": with `imagery=satellite`, "© Mapbox ©
OpenStreetMap © Maxar Improve this map"; with ion imagery on a Google source, the Google line as
today followed by Bing's credit; an ion source that is not Google but has credits (`osm-terrain`)
draws the credits alone, with no logo (a new attribution kind, CREDITS). The logo rule and
`environment_attribution_logo` are unchanged.

## 5. Library design

### Two tilesets per source

`StreamingEnvironmentSource` gains an optional terrain `Tileset` beside its content tileset, built
from the parsed URI: for `ion://` the ion terrain asset; for `mapbox://` the `MapboxTerrainLoader`.
Both tilesets share the source's `CountingAssetAccessor`, task processor, redacting logger,
ECEF-to-map transform, selection views and the terrain transform. Per frame the source updates both
tilesets; `provides_ground()` is true when the terrain tileset exists, `replaces_ground` holds and the
height sample under the ego succeeded (otherwise today's rule for the content tileset). The terrain
tileset has its own `StreamRendererResources` instance flagged *terrain*, so its tiles get the terrain
material and the overlay callbacks act only on terrain tiles. Teardown destroys the terrain tileset first, then the content tileset, in the order the existing code
uses for one; cesium detaches each tile's rasters before freeing the tile inside the tileset's
destructor.

### `mapbox_dem` (C++17-clean)

`DemTile decode_dem_png(bytes)`: stb_image decodes the PNG (PNG only, a static copy inside the unit, so
the tests' and cesium's copies do not clash); the result must be 512×512 with at least three
channels, else `ok = false` with a reason. Heights are stored as `float` metres.
`double dem_sample(const DemTile&, double u, double v)` is bilinear in tile pixel units (0..512).
`DemMesh mesh_dem(const DemTile&, const TileFrame&, const TileId& t14, double z_ref_m, double skirt_m)`: a 65×65 vertex
grid at pixel coordinates 0, 8, …, 512 (so neighbouring tiles share their edge vertices; heights
are bilinear samples at those points), z = height − `z_ref_m`, positions in the tile frame
through the existing `project_to_frame`, normals from the central differences of the heights, UVs
0..1 across the tile (cesium rewrites them per overlay), and a skirt ring of the edge vertices moved
`skirt_m` (20 m) down, triangulated to the edge, so tiles never show cracks. `write_dem_gltf`, in the
terrain loader, builds the glTF (one mesh, one material, `POSITION`, `NORMAL`, `TEXCOORD_0`, `uint32`
indices).

### `mapbox_tiling` (extracted, no behaviour change)

The buildings loader's z14 quadtree, bounding volumes, attempt
counter with backoff (3 attempts, `kRetryWaitS` doubling), transient/permanent status rules and the
`first_time(key)` log dedup move to `mapbox_tiling.{hpp,cpp}` and both Mapbox loaders use them. The
buildings tests, unchanged, pin the extraction.

### `MapboxTerrainLoader` (C++20, `overlume_stream`)

Per z14 tile within the radius: one DEM request through the `DemCache`, decode, `mesh_dem`, glTF,
and the same `TileLoadResult` shapes as the buildings loader (`Success` with content, `RetryLater`,
`Failed`). Root and children as the buildings loader creates them; leaves at z14; cesium upsamples
below that for imagery. Stats: `terrain_tiles_built`, `dem_requests` appended to `MapboxSourceStats`.

### `DemCache`

Shared by both Mapbox loaders of one source: `CesiumAsync::SharedFuture<std::shared_ptr<const DemTile>>`
per z14 tile, created by whichever loader asks first (one in-flight request per tile; a transient
failure is re-asked by the next caller after half the retry wait), LRU of 32 tiles (about 1 MB each).
The DEM height at the anchor, `z_ref_m`, is memoised once it has a value and shares the anchor
tile's entry (the buildings loader asks for it right after its own requests); both loaders subtract it, so the terrain passes through
z = 0 at the anchor and each building's base equals the terrain at its centroid. The buildings loader
lifts every extruded footprint by the DEM height at its unclipped centroid and every landmark node by
the height at its anchor, minus `z_ref_m`; with `terrain` absent from the layer list, buildings keep
z = 0 as today.

### Heights, frames and the terrain transform

Tile frames stay as sub-project 1 defines them. DEM heights are orthometric; the geoid's slope over a
few kilometres is centimetres, and the follow-terrain transform, now sampling the terrain tileset,
absorbs the residual under the ego (offset clamp and `max_tilt_deg` as today). For ion, Cesium's
buildings and terrain already agree, so nothing is re-based.

### Raster overlays and the renderer

- Mapbox: `UrlTemplateRasterOverlay("mapbox-satellite", template with the token in the query,
  {maximumLevel = 18, tileWidth = tileHeight = 512, credit = "© Maxar"})`. The template given to
  cesium writes the row as `{reverseY}`: cesium's `{y}` counts from the south, Mapbox's XYZ row from
  the north, so the request that leaves the process carries Mapbox's row. Ion: `IonRasterOverlay`
  with the asset id and `CESIUM_ION_TOKEN`; its credits come from ion.
- `prepareRasterInLoadThread`: cesium's decoded `ImageAsset` pixels become an RGBA8 buffer
  (`RasterLoadThreadResult`). `prepareRasterInMainThread`: one `filament::Texture` (8-bit RGBA in the sRGB
  format, mipmapped) per raster tile; the main-thread result is that texture. `attachRasterInMainThread`:
  the terrain tile's material instance gets the texture, cesium's `translation`/`scale` as
  `uvTransform` (translation in .xy, scale in .zw; the material samples at (u, 1 − v) because the
  raster's row 0 is its north edge), and `hasImagery = 1`. `detachRasterInMainThread`: `hasImagery = 0`
  and the renderer's 1×1 white placeholder bound back. `freeRaster`: the texture goes through
  the existing deferred-free path, never while a frame may sample it.
- `terrain.mat` (`overlume/assets/materials/`), lit, parameters: `terrainColor` (float3),
  `hasImagery` (int), `imagery` (sampler2d, trilinear, clamp), `uvTransform` (float4: translation,
  scale), `brightness` (float). No fade: the stream applies none to new tiles today, and terrain
  tiles behave the same. The terrain renderer
  resources swap each terrain tile's material instances for a `terrain.mat` instance in
  `prepareInMainThread`, as the clay swap does for buildings.
- The GLB step copies `_CESIUMOVERLAY_0` into `TEXCOORD_0` for terrain tiles, so gltfio loads a UV set
  the material samples.

### Credits (public API)

`uint32_t environment_source_credits(VisualRenderer*, char* out, uint32_t cap)` in
`overlume/include/overlume/scene.h`: writes the open source's overlay credits as plain text (HTML
stripped, `; ` between credits), returns the length needed; a free function, so no `kSceneVersion`
bump (ADR-0004); `check_pod_header.sh` still passes. Empty when no overlay is open.

### Failure handling

| Case | Behaviour |
|------|-----------|
| `imagery` without `terrain`; unknown layer, key value or asset id shape | The parse fails, one log line, opening returns false. |
| DEM 204 or 404 | No terrain for that tile (the sky shows through; Mapbox's DEM is global, so rare). For the anchor's tile no tile has a zero: all terrain stays empty, buildings stay at z = 0, and one line names the tile. |
| Malformed DEM (size, channels, decode) | Tile skipped, one line per kind. |
| DEM 401/403 | That tile empty; "Mapbox rejected MAPBOX_TOKEN (HTTP 401)" once; counts as a failure. |
| Imagery 401/403/429/5xx/transport | A failed raster tile falls back to its parent's coarser imagery, level by level, so one failed tile shows its parent's imagery, not `terrainColor`; cesium logs `Received response code … for image …` (through the redacting logger: token masked) only when the root tile fails too. Every request counts toward network loss, and a rejected Mapbox imagery token fails every tile, so the parents' retries reach eight failures within one tile and the source falls back. An ion imagery token refusal happens at the asset endpoint instead: one `Could not access Cesium ion asset` line, an empty tile provider, one failure, no fallback, and bare terrain (`terrainColor`, no attribution). |
| Terrain tileset cannot open (asset missing, 401) | The source opens without terrain, one line; buildings as today. |
| Network loss (eight consecutive failures) | Whole source to `STREAMING_FALLBACK`: both tilesets torn down, clay plane back, attribution gone. |
| Source switched mid-build | Shared ownership for both tilesets and the cache; overlays detach first. |

Log lines stay one per endpoint and status per source. No disk cache for Mapbox DEM or imagery;
cesium's in-memory caches apply; ion keeps `cache=`.

## 6. Node

- Parameters `environment_terrain` and `environment_imagery` (section 4), composed by
  `compose_environment_source_uri` only when the URI lacks the key, with the layer-list rewrite for
  `mapbox://` (`terrain` appended to the list once).
- `environment_attribution_text(kind, credits)` is a pure function in `environment_source_uri.hpp`;
  `timer_callback` passes the library's credits (read once per second, not per frame).
- `test_asset_paths.py` reads both parameters back.

## 7. Tools and docs

- `tools/check_mapbox_token.sh` probes two more tiles, one DEM and one satellite, and prints one line
  with four HTTP codes (PASS only when all four are 200); its pytest grows accordingly.
- `tools/vcam_ws_bridge.py`, `tools/vcam_gui.py`: the three entries.
- `docs/runbooks/mapbox.md`: sections for the layer list, `imagery`, the DEM, request volume, the
  Maxar credit, failure rows; a new `docs/runbooks/terrain_imagery.md` for ion terrain and imagery,
  the theme token, the node parameters and the renderer's overlay path; `docs/README.md`,
  `docs/status.md`, `CHANGELOG.md`, `NOTICE` (if a new third-party artefact enters) updated.

## 8. Testing and checks

### Fixtures (synthetic only)

- `mapbox_dem_fixture.hpp`: `encode_dem_png(height_fn)` writes a 512×512 RGB PNG from a height
  function (a ramp east-west, a 40 m hill, a flat shelf, sea at 0) with the tests' PNG writer;
  `imagery_fixture.hpp`: 512-px tiles in one flat colour per tile, so attachment and UV mapping are
  pixel-checkable. The fixture network serves the DEM and satellite URL shapes beside the buildings'.
- Ion proof: a fixture ion endpoint JSON, a `layer.json`, and one quantized-mesh tile encoded by the
  test (header, zigzag-delta vertex data, indices), served to cesium-native's own loader.

### Library

- `test_mapbox_dem.cpp`: known pixels → heights (including −10000 + 0 and a 4000 m value), bilinear
  samples, mesh counts (65×65 vertices plus the skirt ring, index count), skirt depth, unit normals,
  UVs in 0..1, malformed inputs rejected with reasons.
- `test_environment_stream.cpp`: overlay callbacks over a fixture tileset through new hooks
  (`environment_stream_overlay_probe`: rasters attached, textures alive, detaches, frees), the
  terrain material's colour, the raster's orientation and brightness.
- `test_mapbox_terrain.cpp` (end to end over the fixture network): terrain tiles built and
  `dem_requests` equal to the tiles (one DEM per tile with two consumers), imagery requests present
  and the token only in query strings, a building on the hill with its base lifted (stats and a pixel
  probe), `provides_ground` true after the ego's sample and the clay plane gone (pixel probe), the
  fixture tile's colour visible at a known point, imagery 401 → cesium's parent retries exhaust the
  failure counter and the source falls back, network loss → fallback with both tilesets torn down, the URI refusals; `ion://…?terrain=…&imagery=…`
  over the fixture ion endpoints: terrain tiles and an attached raster.
- Goldens `mapbox_terrain_synthetic_{dark_adas,light_clay}.png`: the hill with quadrant imagery and
  buildings standing on it, compared at a fixed frame index since renderer creation (Filament's
  temporal dithering), promoted by the maintainer.

### Node and Python

- Composer tests for the two parameters; `environment_attribution_text` with and without credits;
  `test_asset_paths.py` read-back; `test_check_mapbox_token.py` for the four-code line;
  `test_vcam_ws_bridge.py` for the three entries.

### Opt-in live (never in the gate)

`MapboxLive.TestSiteTerrain` (`MAPBOX_TOKEN` + `OVERLUME_LIVE_MAPBOX=1`): 2 terrain tiles,
imagery requests > 0, no token in the log, a PNG in `/tmp`. `IonLive.WorldTerrainBing`
(`CESIUM_ION_TOKEN` + `OVERLUME_LIVE_ION=1`): terrain tiles > 0 and a raster attached.

### Gate

Every check above runs offline in `tools/ci_visual_mode.sh`; counts rise from the sub-project 1
baseline (library ctest 442 with 424 ok and 8 skipped, node 381, WS bridge 94, goldens 25, examples
6) and are recorded in the plan.

## 9. Demo

On the replay rig at both sites: the test site with `mapbox-satellite` (imagery under the
clay extrusions, Maxar on the line, terrain following the route), Downtown with
`mapbox-original-satellite` (landmarks on satellite ground), and `osm-terrain` as the provider proof.

## 10. Delivery

Same branch, same method as sub-project 1: a plan with parallel lanes (Sonnet implementers, Opus
gates, the orchestrator's rulings in its ledger), each task green on the gate and committed alone.
When the last task closes, the PR #1 description gains a section for this feature, `docs/status.md`
a Shipped row, and the plan its live acceptance results.

## 11. Acceptance

1. Over the fixture network, `mapbox://buildings,terrain?imagery=satellite` builds terrain tiles with
   imagery attached and buildings standing on the DEM; every existing golden unchanged.
2. The ion proof streams Cesium World Terrain with an ion overlay over fixture endpoints.
3. On the rig at both sites, the user confirms imagery under the buildings, buildings on the surface
   along the whole route, the attribution with Maxar, and `osm-terrain` with Bing's credit.
4. The network cut reaches the fallback with terrain and imagery gone.
5. `mapbox-satellite` render cost: p50 `render_ms` within 1.5× `osm` at Downtown.
6. No token in any log; no disk cache file written for Mapbox data; the gate green.

## 12. Risks and known limits

- Neighbouring DEM tiles share their edge vertex positions, but each samples its own edge pixels, so
  a one-pixel height step can remain under the skirt; sampling both tiles at shared edges is the
  upgrade if it is visible.
- Imagery resolution is capped at z18 (0.6 m/px); the DEM at z14 (about 10 m) smooths small relief.
- The ion overlay's credits are Bing's long line; the HUD cuts it at the frame edge as any long line.
- Request volume rises with the radius and the imagery level; the defaults keep a z14 tile near the
  ego at a few dozen imagery requests.
- Buildings re-based per centroid stay flat-bottomed: a long building on a slope floats at one end;
  Mapbox GL does the same.
- Building bases are lifted by the full-resolution bilinear DEM, while the drawn terrain is planar
  over an 8-pixel lattice (a vertex about every 35 m), so on fine relief a base can sit a little
  off the surface; interpolating the lift from the lattice, with the mesh's triangle split, is the
  upgrade.
