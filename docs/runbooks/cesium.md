# Cesium ion runbook (VM-060)

How a **new deployment** (a second robot/site with no access to this box)
registers with Cesium ion, picks a tileset for its operating area, and proves
the token actually works — before Epic 6's `StreamingEnvironmentSource`
(VM-062) ever tries to use it. Covers only the account/token/smoke-check
contract; the streaming source itself, its `source_uri` scheme, and its
config knobs are VM-062/VM-063 (pointers below).

## 1. Account + token

1. Create an account at [ion.cesium.com](https://ion.cesium.com) (or sign in
   with an existing one).
2. Go to **Access Tokens** → **Create token**.
3. Scope: **`assets:read`** only — that is all the renderer ever uses.
   (Only add `assets:list`/`assets:write` if you are going through the
   upload-and-clip path in section 2 below; the runtime token itself still
   only needs `assets:read`.)
4. Copy the generated token once — ion does not show it again.
5. On the deploy box, add it to the deploy user's `~/.bashrc` (or whatever
   sources the node's launch environment):
   ```bash
   export CESIUM_ION_TOKEN=<paste the token here>
   ```
   then `source ~/.bashrc` (or open a new shell) so it is live before running
   the smoke check in section 4.
6. **Never commit it, never echo it into a file, log, or `ros2 param dump`.**
   Same rule as `MAPBOX_TOKEN` (Epic 4 Decision 8) — env var, by name only,
   nowhere else.

## 2. Choosing the tileset for the operating area

Three supported paths:

- **Path A (default): Cesium OSM Buildings, ion curated asset `96188`.**
  Global coverage (includes the Dubai/Sharjah operating area), needs no
  upload. In the ion dashboard: **Asset Depot** → find "Cesium OSM
  Buildings" → **Add to my assets**. Done — asset `96188` is now on this
  account. This is the shipped default the node param expects
  (`ion://96188`, VM-063).
- **Path B (custom): upload and clip your own tileset.** Smaller,
  cache-friendlier tiles at the cost of a per-deployment upload step. In the
  ion dashboard: **My Assets** → **Add data**, upload/clip the area of
  interest, note the numeric asset id ion assigns it. That id is what
  replaces `96188` in the `ion://<assetId>` source URI.
- **Path C (VM-064): Google Photorealistic 3D Tiles, ion asset `2275207`,
  original-materials mode.** See section 6 below.

Record whichever `<assetId>` this deployment ends up using — it is the value
the node's `source_uri` param takes (`ion://<assetId>`, VM-063).

**Path B + the vcam GUI's "clipped" preset (VM-096):** the GUI/WS bridge's
Environment Tiles source select offers "clipped" as a one-click switch to
THIS deployment's own Path B asset, but it never guesses or fabricates an
id — set `environment_own_asset_uri` (`config/default_params.yaml`) to
`ion://<assetId>` with the id recorded above once this deployment has
uploaded one. Until then, "" (the shipped default) means "no own asset
yet" and the GUI greys that option out with a tooltip pointing back here.

## 3. The env var contract

- Exact name: **`CESIUM_ION_TOKEN`**. No alternate spelling, no config-file
  fallback.
- Read by the renderer library at `set_environment_source()` time via
  `getenv("CESIUM_ION_TOKEN")` — once, at open.
- Missing or empty → the environment layer fails to open with one WARN;
  nothing else in the node is affected (spec §9, "missing data renders
  nothing" — same non-fatal shape as a missing bake directory).
- Token rotation: replace the env var and restart the node. No param and no
  file of ours stores the token. Two places it *transits* that are worth
  knowing about (both found and closed at the 2026-09-17 final review):
  - cesium-native puts the token in the ion handshake URL
    (`/v1/assets/<id>/endpoint?access_token=…`). The library routes any
    `access_token=`-bearing request around the disk tile cache, so it never
    reaches `cesium-tiles.sqlite` regardless of what ion's response headers
    say — before the fix it stayed out of the cache only because that
    response happens to be uncacheable.
  - on a 401 / no-response, cesium-native logs that URL verbatim through the
    logger the library hands it. The library now hands it a redacting logger
    (`access_token=<redacted>`, `Bearer <redacted>`); a library test drives a
    bogus token through the 401 path and asserts nothing token-shaped reaches
    the log. Before the fix an expired token would have printed itself into
    the node's stdout/ROS log.
  - What the disk tile cache DOES hold: tile and `tileset.json` requests
    carry ion's **short-lived session token** as an `Authorization: Bearer`
    header, and cesium-native's `SqliteCache` persists request headers
    verbatim, so `cesium-tiles.sqlite` contains session tokens whenever
    `cache` is not `off`. That is the exposure class Decision 6 already
    accepted (it said "URLs"; it is the header) — never `CESIUM_ION_TOKEN`
    itself, which rides only on the bypassed handshake request.
  - If a token ever DOES land in a log or transcript, rotate it (§1); the
    rotation is the remedy, not the redaction.

## 4. Smoke check

Run:

```bash
overlume/scripts/check_cesium_token.sh [assetId]
```

(`assetId` defaults to `96188`.) This proves the token reaches the same ion
tileset endpoint the renderer will use — it does **not** touch the renderer
or any node process.

- **PASS** means BOTH stages returned HTTP 200: the `/v1/assets/<id>/endpoint`
  handshake, and the fetch of the `tileset.json` URL that handshake
  returned, using its short-lived session token. Endpoint-200 alone is not
  the AC — "token retrieves `tileset.json`" is checked literally, not by
  proxy.
- Exit codes: **0 = pass**, **1 = fail**, **2 = skip** (`CESIUM_ION_TOKEN`
  unset) — distinct so a wrapper or CI step can tell "broken" from
  "not configured".
- **FAIL, endpoint stage, HTTP 401**: bad or expired token — regenerate it
  in section 1 and re-export.
- **FAIL, endpoint stage, HTTP 404**: the account has not added this asset
  yet — go do section 2's "Add to my assets" step, then re-run.
- **FAIL, tileset stage**: the endpoint accepted the token and handed back a
  session token, but the tileset URL itself rejected or could not route that
  session token — this is an ion-side/network issue, not a token-typo issue;
  re-run once, then treat as a real outage if it repeats.
- The script never prints the token, the session token, or any response
  body — only PASS/FAIL and HTTP status codes.

## 5. What this does NOT cover

- **Baking the fallback/offline environment** — that is the separate baked
  pipeline (`bake_environment.py`), documented in
  [`environment_bake.md`](environment_bake.md) (Epic 4 / VM-042).
- **Disk tile cache and baked-fallback params** (`?cache=`, `&fallback=`,
  `&max_cache_items=` on the `ion://` source URI, and their defaults in
  `default_params.yaml`) — those are VM-063's param table, not this runbook;
  this runbook stops at "the token works against the chosen asset".

## 5b. Placement: rigid x/y, per-vertex ellipsoid-height z (Open Follow-up 4)

Every streamed tile is placed by one rigid ECEF->map matrix per asset root
(`compute_ecef_to_map()`) — accurate for x/y (curvature error ~d³/(6R²),
sub-mm at 10 km), but its z axis is a flat tangent plane at the anchor, so
real (ellipsoid-following) tile geometry sags below it at range (~d²/(2R):
0.08 m at 1 km, 0.54 m at 2.6 km, 7.85 m at 10 km) while the baked chunks and
the rendered robot both sit on a flat plane at z=0 — buildings floated below
the ground. Fixed by rewriting each vertex at load
(`strip_attributes_and_correct_heights()`, `environment_stream.cpp`): x/y
stay whatever the rigid transform gives; z is replaced with (WGS84 ellipsoid
height at that vertex) − `anchor.origin_height_m` (`GeoAnchor`, `scene.h`,
`kSceneVersion` 7), computed via `CesiumGeospatial::Ellipsoid::WGS84`, never
hand-rolled.

**2026-09-21 finding, verified live on the real-robot session replay
(Google 3D Tiles, ion asset 2275207):** the anchor's own ellipsoid height was
hard-coded 0.0 everywhere (`compute_ecef_to_map()`'s ENU origin and this
z-correction both), on the assumption the anchor sits exactly on the
ellipsoid — true in simulation (CARLA) but not on the real robot, where the
Fixposition NavSatFix altitude at the anchor is ~1.7 m: Google's streamed
ground rendered ~1.7 m above the road and buried it. Fixed by sampling the
real height instead of assuming zero: `GeoAnchorSolver` (`geo_anchor.cpp`,
`ros/src/overlume_ros`) now also accumulates `NavSatFix::altitude` (skipping
NaN samples) and sets `GeoAnchor::origin_height_m` to their mean. Two node
params tune it — `geo_datum_height_m` (default NaN = use the sampled mean;
finite overrides it, including under the `geo_datum_lat_deg`/`lon_deg`/
`heading_deg` override, which never samples NavSatFix) and
`geo_anchor_height_offset_m` (default 0.0, always added on top — the trim
knob for a receiver mounted above the `base_link` plane: its altitude then
reads h m too high, so enter `-h`). Both are logged in
the node's `geo-anchor solved` line's `--anchor-height` field.
`bake_environment.py`'s own `wgs_to_map(..., alt_m=0.0)` is unaffected — the
baked layer and the flattened robot both live on their own, separate flat
plane at z=0.

**Delivered accuracy** is bounded by the source geometry, not this formula:
real tile positions (the committed fixtures and, per `environment_stream.cpp`'s
own load-thread comment, real streamed OSM b3dm content) are stored as
absolute-ECEF **float32**, whose ULP is 0.25–0.5 m at Earth-radius magnitude
— so the correction is a no-op within roughly 1.5 km of the anchor (rounded
away below half a ULP) and leaves a ~0.04–0.11 m residual at the fixture's
actual tile ranges (3.8–6.6 km), not the sub-5-mm figure the formula gives on
doubles (see `docs/status.md` item 4 for the measured per-tile numbers).
Sub-decimetre accuracy at range would need tile positions rebased to a local
origin (RTC-style) before the float32 store, not a further per-vertex tweak.

**Node transforms are part of the vertex-to-ECEF map.** The formula above
recovers a vertex's ECEF position as `modelToEcef * p`, which only holds when
a mesh sits directly under the glTF root with an identity node transform —
true of the committed synthesized fixtures but not of real Google 3D Tiles
glbs (ion asset 2275207), which carry `scenes:[{nodes:[0]}]`,
`nodes:[{matrix:[...axis swap..., tx,ty,tz,1], mesh:0}]` (translation ~2.4e6
m) and node-LOCAL float32 positions. `strip_attributes_and_correct_heights()`
now walks primitives via `CesiumGltf::Model::forEachPrimitiveInScene(-1, …)`
and corrects each one against `modelToEcef * nodeTransform`, matching how
gltfio itself places the mesh at render time. The regression is pinned by
`EnvironmentStream.NodeMatrixEncodingRendersIdenticallyToEcefEncoding`
(fixtures `environment_tiles_fixture_session_{ecef,nodematrix}_0`): the same
geometry encoded identity-node (absolute ECEF) vs. node-matrix (Google's own
convention) must render to the same frame. Before the fix, ignoring the node
transform sent node-matrix vertices to the wrong ECEF position entirely —
symptom: the tile renders as one giant tilted slab across the sky, not a mere
height sag.

## 5c. Terrain following (2026-09-21, "option 2")

**Why:** the map frame this node renders into is flat — the ego's own z is
always 0 — but real terrain is not. 5b's per-vertex ellipsoid-height fix
gets Google's streamed ground meeting the road *at the anchor*; a few
hundred metres later, real relief (a rise, a dip) puts that same flat map
frame either above or below the actual streamed ground, so the environment
buries or floats away from everything else in the scene (the robot, the
HD-map layers, the point cloud). Terrain following corrects for that by
moving the *whole streamed environment* up/down to track the ground
elevation under the ego, instead of trying to re-flatten Google's mesh.

**What is sampled, and how often (2026-09-21 multi-point plane fit):** a
single global offset — sampling ONE point under the ego and shifting the
whole tileset by that scalar — isn't enough: the map frame is flat but a
real road has grade, so 20–50 m ahead the true ground has drifted off the
sampled height and crosses the road plane, rendering as photoreal wedges
slicing across the road. `StreamingEnvironmentSource` now samples **5
points along the ego's own heading** at along-track offsets
s = {−20, −10, 0, +10, +20} m from the ego's map (x, y) — each converted
through the same `mapToEcef_` transform 5b describes, then
`Ellipsoid::WGS84::cartesianToCartographic()` — in **one batched**
`Cesium3DTilesSelection::Tileset::sampleHeightMostDetailed()` call (it takes
a `std::vector` of positions and returns one `Future` with parallel
`positions`/`sampleSuccess` results), on the same cadence as before: once
when no sample is already in flight and either no sample has landed yet,
the ego has moved ≥5 m (map x/y) since the last batch, or ≥2 s have passed.
Sampling 5 points therefore costs ONE async request, not 5 — this is what
keeps the change inside the render-time budget (see Cost below). The
continuation lands on the main thread via
`asyncSystem_.dispatchMainThreadTasks()` (the same pump `update()` already
runs every tick) and captures only a shared `TerrainFollowState`, never the
source itself — the source can be destroyed with a sample batch in flight.

**Along-track only, deliberately:** lateral (across-track) samples are never
taken. The verge/kerb either side of the carriageway is genuinely higher
than the road surface, so a lateral sample would pull the fit toward the
kerb's own height/grade instead of the road's, corrupting the very slope
this fit exists to recover.

**The fit:** each successful sample is paired with its own along-track
offset `s` and least-squares fit to `h = a·s + c` (slope `a`, intercept
`c`). ≥3 hits is a real fit; 1–2 hits degrades to `a = 0, c = mean(h)` — the
pre-fit single-point behaviour, generalized; 0 hits keeps the PREVIOUS fit
and logs "no geometry hit", same as before. A successful batch also latches
`ground_hit` (unchanged from before — see Ground plane below).

**Smoothing and clamp — offset AND tilt:** the offset target is unchanged,
evaluated at the fit's own s = 0 (the intercept IS the fitted height at the
ego's own position): `-(intercept − anchor.origin_height_m) − ground_bias`,
clamped to ±30 m. The fit's slope also produces a TILT target —
`clamp(atan(slope), ±max_tilt_rad)` (`max_tilt_deg` below) — rotating the
whole streamed environment about the across-track axis so it matches the
fitted grade instead of just its height at one point: for a 2% grade, the
old offset-only path left ±0.4 m of error 20 m ahead/behind the sample
point; the tilt cancels that down to a few centimetres (the clamp's own
cosine term). A linear fit extrapolates forever, so the tilt is clamped —
`max_tilt_deg` (URI key `max_tilt_deg=<deg>`; node param
`environment_max_tilt_deg`, default 2.0) bounds both how wrong the far field
gets past the fitted ±20 m span and how far streamed buildings visibly lean
(a tilt rotates the whole scene, not just the ground). `max_tilt_deg=0`
disables tilt entirely — offset-only, byte-identical to the pre-fit
behaviour — the escape hatch if a tilted environment ever looks worse than
the wedges it replaces. Both offset and tilt first-order-smooth toward their
targets independently (see below), each snapping on the very first fit.

**Ground bias:** the follower parks the sampled ground `ground_bias=` metres
BELOW the map plane (URI key; node param `environment_ground_bias_m`, default
0.3). At 0 Google's ground lands exactly on the z=0 road plane and z-fights
the HD-map surface and ribbons — tiles and map elements flicker over each
other (seen live 2026-09-21). Raise it if terrain still pokes through at
range; the point-cloud `min_z_m` (0.35 on `urban`/`replay`) is the matching
knob for lidar ground returns.

**Cost:** `sampleHeightMostDetailed()` requests the MOST detailed tiles
under the ego, not necessarily the level being rendered, at least every 2 s
(or 5 m) — extra ion quota and bandwidth on a path that defaults ON; not yet
measured on the live rig. The 5-point batch is still ONE request per cycle
(see What is sampled above), so this didn't change with the multi-point fit
— `overlume/tests/test_environment_stream.cpp`'s
`EnvironmentStreamPerf.RenderMsDeltaAndWorstFrameWithFixtureLoaded` measured
the per-frame render-time delta flat (well under the 5 ms budget) before and
after; the only new per-frame work is one extra `mat4` build (the tilt
rotation) in `update_terrain_transform()`. `environment_follow_terrain` is
read once at configure; a runtime `ros2 param set` takes effect only when a
preset switch re-composes the URI.
The applied offset AND tilt each first-order-smooth toward their own target
with the SAME 0.5 s time constant — except the very first fit, which snaps
both immediately (no half-second slide-up/tilt-in from flat/0 on the first
frame a tileset loads).

**Mechanism:** every streamed tile's asset root is parented (Filament
`TransformManager::setParent`) under one shared "terrain root" entity owned
by `StreamRendererResources`; the smoothed offset AND tilt become that one
entity's transform (`set_terrain_transform(z, tilt_rad, pivot_x, pivot_y,
heading_rad)`, renamed from `set_ground_offset_z()`), so a single transform
moves/tilts the entire streamed environment regardless of how many tiles are
currently loaded:
`translation(pivot) · rotation(tilt_rad, lateral_axis) · translation(−pivot)
· translation({0, 0, z})`, where `pivot` is the ego map (x, y) the FIT was
sampled at (not the live ego position — a moving pivot would sway the whole
world with every ego jitter) and `lateral_axis = {−sin(heading), cos(heading), 0}`
is the across-track direction at the same heading the fit's samples were
taken along, so the rotation pitches about the across-track axis rather than
yawing the scene. `tilt_rad == 0` (`max_tilt_deg=0`, or a flat fit) reduces
this to exactly the old translation-only matrix. **The baked fallback source
is never parented or shifted** — VM-063's `fall_back()` tears down streaming
entirely before opening the baked directory, so a network-loss fallback is
unaffected by terrain following one way or the other.

**The knob:** the `follow_terrain=on|off` key on the `ion://` source URI
(`parse_ion_spec()`, `environment_stream.cpp`) — an unrecognized value fails
the whole URI parse (unlike `materials=`, which degrades to clay on a bad
value: there is no safe silent default for "did the caller mean to shift the
ground"). The ROS node's `environment_follow_terrain` param (default `true`)
composes this key onto any `ion://` `environment_source_uri`
(`compose_environment_source_uri()`, `environment_source_uri.hpp`) — never
onto a plain baked-chunk directory, which has no query string to append to.
Off keeps the pre-existing flat-map-frame behaviour exactly: nothing is
sampled, and the offset stays 0.

**Ground plane (needs `follow_terrain=on`):** the evidence this rule waits
for is a terrain height sample, which is only taken while terrain following
is on. With `follow_terrain=off` nothing is ever sampled, so the clay quad
stays and hides Google's roadside detail inside 60 m — the very symptom this
rule exists to fix. Leave `environment_follow_terrain` on for any
ground-bearing tileset.

**Ground plane:** the renderer's own 120 m clay ground quad
(`build_ground_plane()`, `renderer.cpp`) is removed from the scene once the
streamed tileset PROVES it has ground under the ego — a successful
`sampleHeightMostDetailed()` hit, latched for the life of the source (never
un-latched, so an intermittent later miss can't flicker the plane back in).
Cesium OSM Buildings (buildings-only) never samples ground, so it keeps the
clay plane; Google Photorealistic does, so the plane comes out once proven —
found live 2026-09-21: with the two surfaces no longer z-fighting (the
ground-bias fix above), the clay quad cleanly won z-order and simply hid
every bit of streamed roadside detail within its own 60 m half-extent.
`replaces_ground=off` on the `ion://` URI (or the node's
`environment_replaces_ground: false`) forces the clay plane to always stay,
regardless of evidence.

## 5d. Tile selection: two frustums (2026-09-21 coarse-LOD finding)

**The finding:** through VM-097, tile selection was driven by exactly ONE
view — a synthetic top-down camera 300 m above the ego, 256 px viewport, at
`maximumScreenSpaceError`=48. That geometry only refines a tile once its own
geometric error drops below ~85 m, so Google Photorealistic ground rendered
as a blurry, coarse mesh sitting metres above the real surface — and 5c's
terrain follower then sampled the MOST detailed height under the ego
(`sampleHeightMostDetailed()`) and lifted that coarse mesh clean through the
road.

**The fix:** `synthesize_view_and_pump()` now selects tiles with the real
render camera's own `ViewState` as a SECOND frustum, alongside the synthetic
one — standard Cesium usage (a tileset can be driven by more than one
simultaneous view). Tiles the render camera can actually see refine to real
detail through that frustum (a typical ~20 m range at the same
`maximumScreenSpaceError`=48 already refines to ~1-2 m geometric error, an
order of magnitude tighter than the synthetic view's own 300 m/256 px
geometry, so the threshold itself did not need to change); the synthetic
top-down view keeps doing its original job — a wide coverage frustum so
everything past the camera's view stays loaded coarse instead of unloading
and popping back in. Because `update()` runs before that frame's own
`camera->lookAt()`/`setProjection()` (`renderer.cpp`), the camera frustum
used here is always one frame stale — acceptable lag. `last_view_frustum_count()`
(1 before the render camera has ever been positioned, 2 once it has) is the
test hook this is pinned by (`EnvironmentStream.TileSelectionUsesRenderCameraAsSecondFrustum`).

## 6. Google Photorealistic 3D Tiles (VM-064)

A textured, photorealistic mesh, not clay — `default_params.yaml`'s
`environment_source_uri` preset for it is
`ion://2275207?materials=original&cache=off` (commented out by default;
uncomment to switch). This section covers the three things that make it a
different beast from Path A/B's clay tilesets.

- **Asset id, verified, not guessed.** `2275207` was confirmed at VM-064
  implementation (2026-09-16) directly against this account's live ion API
  — `GET /v1/assets/2275207` returned HTTP 200 with `type: 3DTILES` and a
  `name` field matching "Google", "Photorealistic", and "3D Tiles" (checked
  as redacted boolean substring matches, per this repo's standing rule
  never to print an ion response body — see section 3/4's own "never
  prints ... any response body" discipline, which extends to this
  verification too). `GET /v1/assets/2275207/endpoint` also returned HTTP
  200 (this account already has access; no separate "Add to my assets"
  step was needed for this asset, unlike Path A's `96188`) — note its
  response shape differs from the flat `url`/`accessToken` pair
  `check_cesium_token.sh` parses for a Cesium-hosted asset (it carries
  `externalType`/`options` instead, no top-level `accessToken`): this is
  why `check_cesium_token.sh 2275207` itself reports a parse FAIL even
  though the endpoint accepted the token — that script only proves the
  Cesium-hosted `assets:read` path (Path A/B); the renderer's own ingestion
  never uses that script's flat-field parse at all, it hands the asset id
  + token straight to cesium-native's `Tileset` ion constructor (Decision
  15.6), which already knows how to route an `externalType` asset. Treat
  `check_cesium_token.sh`'s FAIL on this one asset id as expected, not a
  regression — PASS still means what it always meant for `96188`/a Path B
  upload.
- **Attribution (Google Map Tiles terms).** Must be visible wherever tiles
  are displayed. The node draws a static line ("3D Tiles data (c) Google")
  through the existing HUD text primitive (`environment_attribution` param,
  on by default, drawn only while `materials=original` is active) — the
  exact legal wording is NOT re-verified per deployment by this code path
  (no ion credit text is parsed or rendered dynamically); before a real
  go-live, confirm the current required wording against Google's Platform
  Terms / the ion asset's own listed attribution and update the string in
  `overlume_node.cpp` if it has changed. If the HUD text machinery
  cannot carry a line for some deployment (font unavailable, etc. — see
  `hud_font_path`), the manual fallback is a physical/on-screen overlay
  sticker or a fixed compositing step downstream of this node; that gap
  would show up as the node's own one-shot WARN
  (`environment_attribution_warned_`). This "wherever tiles are displayed"
  requirement is not limited to the running node's own on-screen HUD — it
  applies equally to any committed doc capture that displays this imagery
  (e.g. `docs/runbooks/env_source_captures.md`'s `env_source_google.png`),
  which carries its own attribution caption for exactly this reason.
- **Cache terms.** The shipped preset ships `cache=off` (VM-063's existing
  `?cache=` knob, given the literal value `off` rather than a directory —
  Decision 14 / Task 5 Step 2(b)) until this deployment has verified
  Google's current Map Tiles cache-lifetime policy allows the on-disk
  SqliteCache's retention. `cache=off` means every tile request goes
  straight through the network accessor, nothing persisted to
  `overlume-tile-cache` — a real compliance lever, not a placeholder; verify
  the policy, then switch to a real `?cache=<dir>` (or drop the key for the
  library default) once confirmed compliant.
- **No golden ships for this mode.** A committed Google-tile fixture would
  itself need to clear the same redistribution-terms question as the cache
  policy above; this task does not fetch or commit any Google tile bytes.
  Automated coverage is the parse (`materials=original` recognized) and the
  no-remap behavior (both fixture-free, exercised via the existing OSM
  fixture with the test-only `materials_original` hook param) — the visual
  check against real Google content is the validation rig's live-token,
  live-network human step, same as this runbook's own smoke check.
- **Perf.** The plan expected a materially heavier `render_ms` delta than
  either clay preset (textured photoreal vs. untextured extrusions) —
  **measured INCONCLUSIVE at VM-064**: the one dev-box, single-tile,
  single-run measurement did NOT confirm that expectation (see the epic's
  own results block, `docs/plans/2026-08-18-visual-mode-epic6.md`).
  Treat the "materially heavier" claim as unverified until a sustained
  multi-tile, live-rig measurement is done, and budget accordingly;
  `maximumScreenSpaceError`/load-radius tuning are the named first levers
  on a miss.
- **Dim/oddly-coloured tiles (2026-09-22, real-robot replay finding).**
  Google's glbs declare `KHR_materials_unlit`, so they ignore this scene's
  own sun entirely — the shipped `dark_adas.yaml` theme was tuned for the
  renderer's own LIT clay geometry, with a deliberately dark HMI palette
  and exponential fog (`fog.density: 0.010`, ~63% fog colour by 100 m); an
  unlit photoreal mesh under that combination reads dim, dark, and oddly
  tinted even though nothing is actually malfunctioning. Two independent
  levers, either one usable alone: (1) `environment_brightness` (node
  param, default 1.0, composed as `brightness=<gain>` on the `ion://` URI
  only when it differs from that default) multiplies each streamed
  material's own base colour after load, on the `materials=original` path
  only — the clay path is untouched, since it rebinds every primitive to
  the renderer's own building material regardless. (2) lowering the
  theme's own `fog.density` reduces how much of the tinted-dark fog colour
  blends in at range, independent of the tiles' own unlit shading. Start
  with `environment_brightness` if only the streamed tiles read wrong;
  reach for the theme's `fog.density` if the whole scene (clay included)
  reads too fog-heavy.
- **Streamed-tile render radius.** The theme's `environment: { tile_radius_m }`
  (700.0 in the shipped themes) is the default; `environment_tile_radius_m`
  (node param, composed as `radius=<metres>` on the `ion://` URI only when
  it differs from 700.0) is an explicit override that wins when present.
  Either culls streamed 3D Tiles whose bounding box
  lies wholly beyond that horizontal distance from the ego, even when
  Cesium's own SSE-driven selection would otherwise pick them. It only gates
  which selected tiles get added to the scene (with a 4/3 unload band so a
  tile straddling the edge does not flicker); the fixed 300 m-high
  selection view used to drive that SSE selection is unaffected. Nothing is
  drawn beyond the radius — with `environment_replaces_ground` on, the clay
  ground plane stays dropped — so keep it at or beyond the fog/visible
  distance or the world ends in a hard edge.
