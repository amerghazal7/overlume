# Mapbox Buildings Environment Source (Design)

**Date:** 2026-10-14
**Status:** Approved by the user (2026-10-14) with one change: Mapbox appears in the GUI as two more
selectable environment sources, not as presets (§4). Implementation plan:
`docs/plans/2026-10-14-mapbox-buildings.md`.
**Branch:** `mapbox-environment` (off `main` at f2d0d5b). Local only: nothing from this branch is
pushed to `origin` until the user confirms Mapbox's sign-off.
**Scope:** sub-project 1 of 2 (buildings). Sub-project 2, provider-agnostic imagery on terrain,
gets its own spec (§13).
**Load-bearing rules:** tokens by name only; ADR-0003/0004 (no public-header change: the scheme
is parsed inside `set_environment_source`, `kSceneVersion` stays 9); no Mapbox bytes committed;
Mapbox attribution on screen whenever Mapbox content is (Product Terms 1.4); the element-config
directive; every behaviour change ships with a check that fails when it is reverted.

## 1. Context and goal

Overlume's environment layer draws buildings around the vehicle in visual mode (mode 3). One
string passed to `set_environment_source` selects the backend: a baked chunk directory, or Cesium
ion streaming (`ion://<asset>`). In the UAE the ion choices are clay OSM buildings (ion 96188) or
Google Photorealistic 3D Tiles, which are 2D-only there.

Mapbox serves two things for Dubai that are better:

1. landmark 3D models with baked ambient-occlusion textures (Downtown, Dubai Marina);
2. footprints with heights and building parts for every building (the streets-v8 `building`
   layer), which Mapbox's own Standard style extrudes.

Goal: `mapbox://buildings`, a streamed environment source selected like `ion://`, rendered by the
existing streaming pipeline, with Mapbox's attribution on screen, demo-ready at the test site,
Downtown Dubai.

Non-goals: imagery and terrain (sub-project 2); Mapbox style features (per-part colours, window
lights, LOD meshes, procedural facades); any disk cache; changes to the bake script; new public
API.

## 2. User decisions (2026-10-14)

| # | Question | Decision |
|---|----------|----------|
| D1 | Direction | Build buildings (this spec), then imagery on terrain. Demo to Mapbox, then release publicly citing them. The user has checked the licensing side. |
| D2 | Bake script's Mapbox Tilequery fallback | Leave it as is. |
| D3 | Demo site | Downtown Dubai, the test site (the Burj Khalifa, 25.1972, 55.2744), through a geo-datum override. |
| D4 | Architecture | A custom cesium-native loader inside the existing `StreamingEnvironmentSource`. Not a separate source class; a Mapbox pre-bake is excluded by the terms (no derived or stored data). |
| D5 | Publication | The branch stays local until sign-off. Tests use synthetic fixtures in Mapbox's format, never real Mapbox bytes. Whether the Mapbox logo file stays in the tree is part of the release check. |
| D6 | Execution | Dynamic workflows with as much parallelism as possible (user, 2026-10-14). |

## 3. What Mapbox serves (probed 2026-10-14)

Numbers are from the Downtown Dubai z14 tile 10707/7006. The probe scripts are not part of the
repo.

### Landmark models

- `https://api.mapbox.com/3dtiles/v1/mapbox.mapbox-3dbuildings-v1-meshopt/14/{x}/{y}.glb?access_token=…`
  is the variant the TileJSON of `mapbox.mapbox-3dbuildings-v1` lists for clients that decode
  meshopt. Current Mapbox GL JS prefers the other listed variant, `-meshopt-v2-lod`, whose scene 0
  is identical but whose vertex streams use meshopt codec v1, which the meshoptimizer Overlume
  links first cannot decode (§12); this source requests `-meshopt`, codec v0. The base (draco)
  tiles are empty in Dubai. Content exists at z14 only: every other zoom answers 204.
- Required extensions: `EXT_meshopt_compression` and `KHR_mesh_quantization` (POSITION float,
  NORMAL normalized byte, TEXCOORD_0 normalized ushort). Also used: `KHR_texture_transform`.
- Scene "Default Scene" (the only scene; the file has no `scene` key) holds, per landmark, one
  building node and one footprint node. The `-meshopt-v2-lod` variant adds a scene "LOD" of
  low-detail duplicates, with `mbx_bvh` on its nodes.
- Building node: mesh plus `matrix` = diag(s, −s, 1) with translation (tx, ty, 0). tx/ty are z14
  tile units 0..8192, y pointing south. s = 8192·2^14 / (2π · 6 371 008.8 · cos φ_node), the Web
  Mercator scale on the mean-radius sphere at the node's latitude. Local geometry is metres, x
  east, y north, z up, ground-relative (the Burj Khalifa model tops out at 780.5 m). Extras: `id`
  (uint64 as a string), `anchor`, `lights` (base64, ignored).
- Footprint node: same matrix as its building; POSITION-only triangles at z = 0; no material;
  extras `mapbox:footprint:id` equal to the building's `id`.
- Materials: double-sided, metallic 0, a JPEG in `occlusionTexture` only (with
  `KHR_texture_transform`, REPEAT wrap), `baseColorFactor` on some. No base colour texture.
- The Downtown tile holds 7 landmarks in 1.14 MB.

### Footprints

- `https://api.mapbox.com/v4/mapbox.mapbox-streets-v8/16/{x}/{y}.vector.pbf?access_token=…`.
  z16 is the source's maximum zoom, so it carries full detail. The body is always gzip, whatever
  `Accept-Encoding` says; libcurl inflates it under cesium's accessor.
- `building` layer: extent 4096 with a 33-unit buffer; every feature has a uint64 id; polygons
  with holes and multipolygons occur. Fields: `height` and `min_height` (doubles), `extrude` and
  `underground` (strings "true"/"false"), `type` (building, building:part, roof, …),
  `building_id` on parts.
- Standard's 3D filter is `extrude == "true"` and `underground == "false"`. `extrude = "false"`
  marks parent outlines of part sets (the Burj Khalifa outline is one).
- 16 z16 tiles make one z14 tile: 1,374 features in the Downtown z14 tile. 125 of 1,238 ids
  appear in more than one z16 tile as clipped copies.

### Status codes and headers

- 204 with an empty body: no landmarks in this z14 tile (or a zoom other than 14).
- 200 with only a `water` layer: open sea. Empty land does not produce 204 or 404.
- 401 `{"message":"Not Authorized - Invalid Token"}` on both endpoints for a bad token.
- Cache-Control max-age: 7 days for GLBs, 12 hours for MVTs. Rate limits: 30,000 requests per
  minute (3dtiles), 100,000 (v4).

### Coverage

Landmarks exist in Downtown Dubai and Dubai Marina; none in the Abu Dhabi samples or the other
blocks sampled. Footprints exist everywhere sampled. In Downtown, 6 of
the 7 landmarks stand on streets-v8 extrusions, which must be removed, as GL JS does.

## 4. Configuration

### URI

`mapbox://buildings[?key=value(&key=value)*]`

- The path is the layer list. This spec accepts only `buildings`; anything else fails to open
  with a log line naming the supported layers. Sub-project 2 adds `terrain`.
- The query parses like `parse_ion_spec`: split at the first `?`, pairs on `&`, key at the first
  `=`, the last value wins, unknown keys are ignored, and a bad value for a known key fails the
  whole parse.

| Key | Values | Default | Meaning |
|-----|--------|---------|---------|
| `materials` | `clay`, `original` | `clay` | As ion: any other value warns once and means `clay`. |
| `radius` | finite > 0 (m) | 500 | Load and display radius around the ego (§5). Not the theme's `tile_radius_m`, which stays the ion default. |
| `brightness` | finite > 0 | 1.0 | As ion: scales base colours under `materials=original`. |
| `fallback` | directory | none | As ion: baked chunks shown after network loss. |
| `cache` | `off` | `off` | `off` is accepted and changes nothing. Any other value fails the parse: Mapbox tiles are never written to disk. |

The ion-only keys (`follow_terrain`, `ground_bias`, `replaces_ground`, `max_tilt_deg`,
`max_cache_items`) are ignored like unknown keys. The source always runs with terrain following
off and does not replace the ground plane: it holds buildings only, and a roof hit must never
hide the clay ground.

Without cesium (`OVERLUME_ENABLE_CESIUM=OFF`), `mapbox://` returns false, as `ion://` does.

### Token

The library reads `MAPBOX_TOKEN` from the environment when a `mapbox://` source opens, the way
it reads `CESIUM_ION_TOKEN` for `ion://`. If it is unset or empty, `set_environment_source`
returns false and the library logs one line naming the variable. The token never appears in the
URI, a parameter, a log line, a model or a file.

### Node composition

`compose_environment_source_uri` treats `mapbox://` URIs as follows:

- `fallback=<environment_chunks_dir>` is appended as today.
- `cache=` is never appended (today it is appended to every scheme).
- `radius` (when `environment_tile_radius_m` ≠ 700) and `brightness` (when ≠ 1.0) are appended
  as for ion.
- `follow_terrain`, `ground_bias`, `replaces_ground` and `max_tilt_deg` are never appended.

The six `rfind("ion://", 0)` gates become two scheme predicates. Live switching needs no node
change: the `environment_source_uri` parameter branch already passes any scheme to the library.

### GUI source list

There are no presets. The vcam GUI's Environment Tiles selector lists the supported environment
sources (`baked`, `osm`, `clipped`, `google`), and it gains two more, kept in lockstep between
`tools/vcam_gui.py` and `tools/vcam_ws_bridge.py`:

- `mapbox` → `mapbox://buildings` (clay, the theme's look, like `osm`);
- `mapbox-original` → `mapbox://buildings?materials=original` (Mapbox's look, like `google`).

No new control is added. The bridge's `set_environment_source` handler is generic, so
`tools/validate_logger_session.sh --arm mapbox` works unchanged. The code's existing names for
this list (`ENVIRONMENT_PRESETS`, and the `preset` field of the GUI-to-bridge message) are not
renamed.

## 5. Library design

### Units

All new library code needs cesium. It compiles in the C++20 `overlume_stream` object library,
as `environment_stream.cpp` does, and none of it builds with `OVERLUME_ENABLE_CESIUM=OFF`.

| File | Responsibility | Uses |
|------|----------------|------|
| `overlume/src/mapbox_mvt.{hpp,cpp}` | Bounds-checked decoder for the MVT `building` layer: per feature the id, polygons (outer ring plus holes, by the MVT v2 winding rule) and the four fields. C++17-clean header with std types only. | nothing |
| `overlume/src/mapbox_extrude.{hpp,cpp}` | Footprint polygon to roof and wall triangles with normals: clip to the tile square, earcut roofs, walls from base to top, no wall on a clipped tile edge. Takes a callback that projects (tile x, tile y, height) to output metres. C++17-clean header. | `mapbox/earcut.hpp` (cesium's vcpkg, header-only) |
| `overlume/src/mapbox_tile_frame.{hpp,cpp}` | Web Mercator XYZ ↔ geodetic; the z14 tile's local frame; the exact tile-space → frame projection; the per-node affine linearisation. | CesiumGeospatial |
| `overlume/src/mapbox_landmarks.{hpp,cpp}` | Landmark GLB to placed building nodes plus footprint triangles; drops the LOD scene, footprint nodes and, in clay mode, textures. | CesiumGltfReader, tile frame |
| `overlume/src/mapbox_loader.{hpp,cpp}` | The `TilesetContentLoader`: quadtree, requests, retry policy, merge into one model per z14 tile. Also the `mapbox://` parser and `open_mapbox_environment_source`. | all of the above |
| `overlume/src/mapbox_test_hooks.hpp` | C++17-clean test entry points for the cesium-typed code, like `environment_test_hooks.hpp`. | — |

Changes to existing code:

- `environment_stream.{hpp,cpp}`:
  - `StreamingEnvironmentSource` accepts an optional custom tileset: a loader, its root tile and
    an "area selection" flag. It builds it with cesium's custom-loader `Tileset` constructor and
    its own options (below).
  - With area selection, `synthesize_view_and_pump` passes one area view (below) instead of the
    synthetic top-down view and the render-camera view. Mixing an area view with screen-space
    views makes cesium use screen-space error for all of them.
  - A map fixture accessor for tests: URL → status and body, or a thrown transport error, with a
    log of requested URLs. It sits beside `FileFixtureAssetAccessor` and reuses its response
    classes.
  - The `CountingAssetAccessor` fix (Failure handling).
- `environment.{hpp,cpp}`: a `mapbox://` branch in `set_environment_source` before the baked
  fall-through, and `open_mapbox_environment_source(spec, anchor)` declared beside
  `open_streaming_environment_source`.
- `overlume/CMakeLists.txt`: `mapbox_*.cpp` move from `overlume_obj` to `overlume_stream`. The new
  test file is dropped when cesium is off, and its target gets meshoptimizer's include directory
  for fixture encoding.

### Tiling and selection

- The loader models Web Mercator XYZ as a cesium quadtree. The root is an empty, unconditionally
  refined tile over the Mercator world with one child `QuadtreeTileID(0, 0, 0)`, the pattern
  cesium's `LayerJsonTerrainLoader` uses for EPSG:3857. Tile IDs carry Mapbox's XYZ y
  (north-down); only this loader interprets them.
- Levels 0–13 have empty content and are unconditionally refined. `loadTileContent` on them
  returns an empty success, because cesium may unload empty tiles and request them again.
- Level 14 tiles carry content, have geometric error 0 and no children.
- Every tile's bounding volume is a `BoundingRegion` over its exact geodetic rectangle, from the
  anchor height − 10 m to + 1000 m. A loaded z14 tile reports its real height range through
  `updatedBoundingVolume`.
- Selection uses cesium's area view: `ViewState(BoundingSphere{ego ECEF, radius}, 1.0)`. Every
  z14 tile whose region intersects the sphere loads and shows; nothing else is requested.
- The Mapbox tileset's options turn off `preloadSiblings` (otherwise the 3 siblings of each z14
  tile would load at preload priority). Fog culling needs no switch: cesium 0.64 never fog-culls
  an area view.
- The existing per-tile radius gate on scene membership still runs and admits every tile the
  area view loads.
- With the 500 m default: one tile while the ego is more than 500 m from every edge of its z14
  tile (about 2.2 km wide in Dubai), up to four near a corner. 500 m is the renderer's far plane,
  so every building that can be on screen is loaded.

### Building one z14 tile

The work runs on cesium worker threads, after copying what it needs out of `TileLoadInput`
(a stack object of references).

1. **Requests.** The landmark GLB and the 16 z16 MVTs go through the tileset's accessor, so
   counting and redaction apply. They are combined with `AsyncSystem::all`. Each request future
   first gets a catch that turns a transport error into a "failed, transient" result. One failure
   therefore never rejects the whole tile; an uncaught rejection would mark the cesium tile failed
   for good.
2. **Landmarks** (GLB 200):
   - `GltfReader::readGltf` runs with the tileset's reader options, except
     `decodeEmbeddedImages = false`: Filament decodes the JPEGs itself, so cesium's decode is
     wasted work. Cesium decodes meshopt and quantization.
   - Keep the scene named "Default Scene" (else `scene`, else 0) and take its building nodes.
   - Collect footprint triangles from nodes whose extras carry `mapbox:footprint:id`.
   - Drop the footprint nodes, the LOD scene, and every mesh, accessor, buffer view, buffer and
     byte range only they used, including meshopt's compressed source and fallback buffers.
     Clay mode also drops textures, images and samplers.
   - Each kept node's matrix becomes the affine map from its local metres to the tile frame
     (Placement).
3. **Extrusions:**
   - Each MVT is inflated when it is gzip (`CesiumUtility::isGzip` / `gunzip`), then decoded and
     filtered with Standard's rule.
   - A feature with no numeric `height`, or with `height <= min_height`, is skipped.
   - Each polygon is clipped to its z16 square; the 33-unit buffer would otherwise double up
     geometry.
   - It is extruded from `min_height` (0 when absent) to `height`. Roofs are triangulated with
     earcut, holes included.
   - Walls carry explicit per-face normals (duplicated vertices, facing outward).
   - No wall is built along a clipped tile edge, so buildings that cross z16 or z14 boundaries
     join without seams. No floor faces are built, as in GL JS.
4. **Conflation.** An extruded part whose footprint centroid lies inside any landmark footprint
   triangle is dropped: the model replaces it. GL JS uses the same footprints for the same
   purpose.
5. **Merge:**
   - One `CesiumGltf::Model` with one scene: the kept landmark nodes plus one extrusion node
     (one mesh, one material).
   - Result: `TileLoadResult{model, glTFUpAxis = Z, updatedBoundingVolume, …, Success, input
     ellipsoid}`.
   - `pCompletedRequest` stays null. Otherwise cesium copies that request's URL, which carries
     the token, into the model's extras, and Overlume would write it into the GLB.
6. **Hand-off.** From here the existing pipeline runs unchanged:
   - `prepareInLoadThread`: attribute strip, height correction, GLB write, and the normals pass,
     which skips primitives that already have normals;
   - `prepareInMainThread`: gltfio asset, clay material or originals, brightness.

### Placement

- **Tile frame.** Each z14 tile's frame is east-north-up at the tile centre, on the WGS84
  ellipsoid at the anchor's ellipsoidal height `origin_height_m` (cesium
  `eastNorthUpToFixedFrame`). It is the tile's transform, kept in double precision, so vertices
  stay small floats.
- **Extrusion vertices** go through an exact chain, vertex by vertex: tile coordinates →
  longitude and latitude (inverse Web Mercator) → ECEF at height `origin_height_m + z` → tile
  frame.
- **Landmark nodes.** The node matrix maps local metres into z14 tile units, as Mapbox defines
  them. Tile units to tile frame use the same chain as the extrusions, linearised at the node's
  anchor. Over 250 m the Mercator scale changes by 1.8·10⁻⁵, so at ground level the node's map
  stays within 1.1 cm of the per-vertex chain across a 500 m building. At 300 m up the two differ by
  up to 2.5 cm: the per-vertex chain bends with the ellipsoid, which a rigid building does not.
  Vertices are untouched; only the node matrix changes, so accessor bounds stay valid.
- **Why both go through tile space.** Landmark footprints and streets-v8 footprints then land on
  the same ground. Mapbox's scale uses the 6,371,008.8 m mean radius; re-deriving it from
  6,378,137 m would make the models 0.11% too large.
- **Heights.** The existing height correction in `prepareInLoadThread` sets map z to the
  ellipsoidal height minus `origin_height_m`. On this content it returns exactly the
  ground-relative heights: building bases at map z = 0 and roofs at `height`, at any distance
  from the anchor.
- **Map x/y** come from the same `ecefToMap_` as ion content, so heading and the map's sphere
  scaling apply unchanged.
- Buildings therefore stand on the flat map plane everywhere. There is no terrain following in
  this sub-project.

### Look

- `materials=clay` (default): the theme's building material on everything, as with the `osm`
  source. Landmark textures are dropped before upload.
- `materials=original`:
  - Landmarks keep Mapbox's materials (occlusion-textured, double-sided).
  - Extrusions get Mapbox Standard's default building colour, hsl(40°, 43%, 93%), as a linear
    base colour (≈ 0.912, 0.869, 0.787), with metallic 0 and roughness 1.
  - `brightness` scales both.
- Not in this sub-project: per-part landmark colours, window lights (`lights` extras), LOD meshes,
  procedural facades.

### Failure handling

| Case | Behaviour |
|------|-----------|
| `MAPBOX_TOKEN` unset or empty | Opening fails; `set_environment_source` returns false; one log line names the variable. |
| Unknown layer, bad key value, `cache=<dir>` | The parse fails and logs one line, and opening returns false. |
| Landmark GLB 204 | No landmarks in this tile (normal). |
| 401 or 403 on either endpoint | That part of the tile is empty. Logs "Mapbox rejected MAPBOX_TOKEN (HTTP 401)" once per endpoint. The responses count as failures, so eight in a row switch to the fallback. |
| MVT 404 | That z16 square is empty. |
| 429, 5xx, transport error | The tile is retried later (cesium RetryLater), at most 3 attempts per tile. After that it shows what loaded and logs once. The attempt count lives in the loader and is touched only in `loadTileContent` (main thread). The responses count as failures, as in the 401 row. If eight in a row fail (for example a whole tile answering 429 or 5xx), the network-loss row fires before any retry. |
| Malformed GLB, or a node with an unexpected matrix | Landmarks (or that node) skipped for the tile; extrusions still shown; one log line. |
| Malformed MVT or feature | That z16 square or feature is skipped. The decoder never reads out of bounds. |
| Network loss (8 consecutive failed requests) | Existing fallback: streamed content torn down, baked chunks from `fallback=`, state `STREAMING_FALLBACK`. |
| Source switched away while a tile is building | Safe: async work holds shared ownership of the loader state it uses, never a raw `this`. |

Log lines are limited to one per endpoint and status per source.

**Fix to existing behaviour, needed by the network-loss row.** `CountingAssetAccessor` counts
only HTTP non-2xx replies. A transport error (DNS failure, timeout, refused connection) rejects
the request future, and the counter's continuation never runs. So a real loss of connectivity
never triggers the fallback, for ion as well. The counter will also count rejected requests,
then rethrow so callers see the same error. A throwing-accessor regression test covers ion and
Mapbox.

### Secrets

- The token lives only in the loader's memory and in the request URLs it builds (the
  `access_token` query parameter). Cesium's custom-loader tilesets drop request headers anyway.
- Every loader log line goes through the redacting logger, which rewrites `access_token=…` to
  `<redacted>`. That includes curl's exception text, which carries the URL.
- `pCompletedRequest` stays null, so no URL reaches model extras or a written GLB.
- The Mapbox source is built with the cache off, so no cache database is even created. Its URLs
  would bypass the SQLite cache anyway (`TokenBypassAssetAccessor`).

## 6. Node: attribution

Mapbox Product Terms 1.4 require the Mapbox logo, "© Mapbox", "© OpenStreetMap", and "Improve
this map" in the lower-right corner. The output is a non-interactive video frame, so the links
named in the terms are rendered as text.

- **When:** all of the following hold:
  - `environment_attribution` is on (existing parameter, default true; it is the element's
    disable knob);
  - the environment is effectively visible (free look and enabled);
  - the source URI starts with `mapbox://`;
  - the state is `STREAMING`.

  This is independent of `hud_enabled`, as the Google line is. In `STREAMING_FALLBACK` only baked
  chunks are on screen, so nothing is drawn.
- **What:**
  - The logo at the bottom-left, 8 px from the edges, at its native size.
  - "© Mapbox © OpenStreetMap Improve this map" right-aligned at the bottom-right, 8 px from the
    edges, in the theme's HUD text colour and scale (the element's style tokens).
- **Gate:** one pure function, `environment_attribution_kind(visible, enabled, source_uri,
  state)`, returns none, google or mapbox. It replaces the inline condition at
  `overlume_node.cpp` ~1317 and is unit-tested; today's Google path has no test.
  - The scheme is checked first. The current Google gate is a substring test for
    `materials=original`, so it would also fire on `mapbox://buildings?materials=original`.
- **HUD primitives** (`hud_overlay`):
  - `DrawImageRgba` alpha-blends an RGBA8 image into the RGB8 frame (row 0 at the top), clipped
    at the frame edges.
  - `MeasureText` returns a string's width with the baked atlas, for right alignment.
  - The font atlas grows from ASCII to Latin-1 (U+0020–U+00FF), and `DrawText` decodes two-byte
    UTF-8. Today any byte outside ASCII draws nothing, which is why the Google line says "(c)".
    The atlas size grows if the larger set does not fit at the shipped HUD scales. ASCII glyphs
    rasterise exactly as before, so the existing HUD and callout goldens must not move.
- **Logo asset:** `ros/src/overlume_ros/assets/attribution/mapbox-logo.png` (Git LFS).
  - Rasterised at twice GL JS's 88×23 px size from the official Mapbox logo SVG that the
    `mapbox-gl` package ships, at a pinned version.
  - Provenance in an `ATTRIBUTION.md` beside it: source, version, SHA-256, rasteriser command.
  - Installed like the HUD font. Parameter `environment_attribution_logo_path`: an empty default
    resolves to the installed file.
  - Decoded once at configure with stb_image, which the node already downloads hash-pinned. It
    is compiled as `STB_IMAGE_STATIC` with PNG only in one node file: a non-static
    implementation would collide with the test executables that define their own.
  - A missing or unreadable logo warns once, and the text is still drawn.

## 7. Tools and docs

- **GUI source list:** `tools/vcam_gui.py` and `tools/vcam_ws_bridge.py` gain the two entries
  (§4), plus the bridge's docstring and its error string.
- **`tools/check_mapbox_token.sh`:** modelled on `check_cesium_token.sh`. It requests the
  Downtown landmark tile and one z16 MVT with `$MAPBOX_TOKEN` and prints PASS/FAIL and the HTTP
  codes only. It is the demo preflight and is never in the gate.
- **`tools/validate_visual_mode.sh`:** refreshes `MAPBOX_TOKEN` from a login shell exactly as it
  refreshes `CESIUM_ION_TOKEN`, so the node never runs with a stale value.
- **Docs:**
  - a new runbook, `docs/runbooks/mapbox.md`: token, URI and the GUI entries, attribution, the terms
    constraints as engineering rules, the demo at the test site, troubleshooting;
  - `README.md`, "Environment sources";
  - `docs/README.md`, index entries for this spec, the plan and the runbook;
  - `docs/status.md`, open items: Mapbox sign-off before any push, and the logo/trademark check
    before release;
  - `CHANGELOG.md`, `[Unreleased]`;
  - `NOTICE`, a Mapbox streaming-source block beside the Cesium OSM and Google blocks, pointing
    to the runbook for terms;
  - `environment_bake.md`, one line separating the bake fallback from the runtime source.

## 8. Testing and checks

Each item fails when the behaviour it guards is reverted. No test touches the network, needs a
token, or contains Mapbox bytes.

### Fixtures

Fixtures are built in the test process; nothing binary is committed.

- **MVT:** a small protobuf encoder in the test sources (layers, features, ids, tags, zigzag
  geometry commands), with a gzip-wrapped variant.
- **Landmark GLB in Mapbox's layout:**
  - building nodes with diag(s, −s, 1) matrices in tile units;
  - footprint nodes with `mapbox:footprint:id`;
  - a second "LOD" scene;
  - an occlusion-textured material;
  - quantized normals and texture coordinates;
  - buffer views compressed with `EXT_meshopt_compression`, using meshoptimizer's encoder at
    vertex codec v0, the version that extension and Mapbox use.

  Turning meshopt decoding off must fail the landmark tests.
- **Serving:** the map fixture accessor (§5) serves both.

### Library

In `overlume/tests/test_mapbox_buildings.cpp` (cesium builds only):

- **URI:**
  - the layer list, each key and the defaults (radius 500);
  - bad values fail; `cache=off` is accepted and `cache=<dir>` refused; ion-only keys are
    ignored;
  - with `MAPBOX_TOKEN` unset, `mapbox://` returns false and the log names the variable.
- **MVT decoder:**
  - features, ids, rings, holes and multipolygons;
  - every numeric value type for the height fields, and the string flags;
  - every prefix of a valid tile and a fixed set of random buffers decode without crashing or
    reading out of bounds. They return `ok = false` or count skipped features.
- **Extrusion:**
  - vertex and triangle counts for a box and for a courtyard (a hole);
  - walls running from `min_height` to `height`;
  - unit-length outward normals;
  - no walls on clipped edges;
  - filter: extrude false, underground true, missing height, and `height <= min_height` are all
    skipped.
- **Tile math:**
  - XYZ ↔ longitude/latitude round trip, and the z16 squares inside a z14 tile;
  - a known longitude/latitude lands within 2 cm of where `ecef_to_map_probe` puts it, after the
    full transform chain (tile frame, `ecefToMap_`, height correction); the node matrix is
    covered by the next item;
  - a landmark node's affine map is within 2.5 cm of the exact chain across ±250 m and 300 m of
    height (§5 Placement explains the bound).
- **Landmarks:**
  - only "Default Scene" building nodes reach the model, and no footprint or LOD node, mesh or
    buffer survives;
  - clay mode drops images, while original mode keeps the occlusion texture;
  - a node with an unexpected matrix is skipped.
- **Conflation:** an extrusion under a landmark footprint is dropped, and its neighbour stays.
- **End to end** (fixture accessor through `StreamingEnvironmentSource`):
  - the anchor's z14 tile loads, the state is `STREAMING`, and landmark and extrusion entities
    are in the scene;
  - neighbours answering 204 with empty MVTs add nothing;
  - only z14 tiles inside the radius are requested (from the accessor's URL log);
  - the requested URLs carry the bogus token, but neither the produced GLB nor any captured log
    text contains it;
  - 401 on every request reaches `STREAMING_FALLBACK`;
  - 503 followed by 200 retries and loads;
  - a throwing accessor trips the fallback (the counting fix, also asserted for ion);
  - switching away mid-build is safe.
- **Golden** `MapboxBuildingsGolden.SyntheticTileDarkAdas` (320×240, dark_adas, clay):
  - Present: one landmark tower, a courtyard block, a floating part (`min_height` > 0), and a
    building split across two z16 squares with no seam.
  - Absent: an extrusion under the landmark footprint, and an `extrude=false` outline.

  The candidate is shown to the user and committed only after promotion. No existing golden may
  move.

### Node

- `test_environment_source_uri.cpp`: Mapbox composition. No `cache=`; `fallback=` is appended;
  `radius` and `brightness` are appended when not default; ion-only keys never are; ion strings
  are unchanged.
- `environment_attribution_kind`: the truth table over the Google and Mapbox branches, every
  state, visibility and the parameter. `mapbox-original` never yields Google.
- `test_hud_overlay.cpp`:
  - `DrawImageRgba`: opaque, half-transparent and clipped blits, and an untouched buffer for a
    null image;
  - `MeasureText`: grows with the string and matches the drawn extent;
  - "©" draws pixels;
  - the existing HUD and callout goldens still pass.
- **Logo loading:** a PNG written by the test decodes, and a missing path is non-fatal.

### Python

`tools/test_vcam_ws_bridge.py`: the bridge accepts `mapbox` and `mapbox-original`, the GUI maps a
node's `mapbox://` URI back to the right entry, and the two tables stay in lockstep.

### Opt-in live

Never in the gate. Needs `MAPBOX_TOKEN` and `OVERLUME_LIVE_MAPBOX=1`.

- `MapboxLive.DowntownDubai` renders at Downtown Dubai, the test site.
  - It prints tile, landmark and extrusion counts and per-tile build times (no URLs), and writes
    PNGs to `OVERLUME_TMP_DIR`.
  - It asserts that Downtown has at least one landmark.
  - It is the demo rehearsal at library level.
- `tools/check_mapbox_token.sh`.

### Gate

Every task ends green on `tools/ci_visual_mode.sh`, run in the foreground.

## 9. Demo

The demo uses the replay rig (`tools/validate_logger_session.sh`) and the vcam GUI's free-look
camera. The runbook holds the exact commands.

- **Without an override:** the session's own GPS anchor, then `--arm mapbox` (or
  `mapbox-original`). Away from Downtown Dubai and Dubai Marina only extrusions appear: Mapbox has
  no landmark models there.
- **Downtown Dubai, the test site:** a full geo-datum override passed after `--`, together with
  `environment_source_uri:=mapbox://buildings…`, which arms at activate.
  - The override needs latitude, longitude and heading. A partial override is ignored and the
    anchor falls back to GPS.
  - The datum pins the map origin, not the robot. The rehearsal keeps the session's map heading
    and picks a latitude/longitude so the replayed path runs on open ground near the Burj
    Khalifa. The chosen values go into the runbook.
- **Preflight:** run `tools/check_mapbox_token.sh`. The rig owns the machine and stops any other
  rig.
- **Volume:** 17 requests per z14 tile, with one to four tiles live at a time.

## 10. Delivery

- **Documents:** this spec, and the plan at `docs/plans/2026-10-14-mapbox-buildings.md` with its
  own status ledger.
- **Parallel lanes** (D6). These six are independent from the start, each with its own tests:
  1. MVT decoder.
  2. Extrusion builder.
  3. Tile frame and landmark reader, with the GLB fixture builder.
  4. Streaming-source seam: custom tileset and area selection, map fixture accessor, counting
     fix, `mapbox://` dispatch and parser.
  5. Node: composition, attribution gate, HUD primitives, logo asset.
  6. Tools and docs: the two GUI entries, token check, rig token refresh, runbook,
     README/NOTICE/status/CHANGELOG.

  Then, in order:

  7. The loader, which wires lanes 1–4 together, with the end-to-end tests.
  8. The golden candidate, the live smoke and the demo rehearsal, ending at the human promotion
     gate.
- **Worktrees:** each lane works in its own git worktree off `mapbox-environment`, with its own
  build directory that reuses the already-fetched Filament and cesium sources. Lanes merge into
  the feature branch in dependency order.
- **Per task:** each task ends green on the gate and is committed on its own. Tasks run as
  dynamic workflows: session-model orchestrator, Sonnet implementers, Opus review gates, at most
  two fix rounds.
- **Before any push**, which comes only after Mapbox's sign-off: `tools/check_publish_leaks.sh`
  plus the lint checks. No token, no local path, no internal name.

## 11. Acceptance

1. `tools/ci_visual_mode.sh` is green with the promoted golden, and no existing golden moved.
2. `MapboxLive.DowntownDubai` passes, and `tools/check_mapbox_token.sh` passes.
3. A rehearsal on the replay rig, with and without the override, shows:
   - **Downtown:** the Burj Khalifa and the other landmarks render as models with no doubled
     extrusion under them, among streets of extruded buildings.
   - **Without an override:** extruded buildings on the map plane.
   - **Attribution:** logo and text visible in mode 3 while streaming; gone in fallback.
   - **GUI:** selecting `mapbox-original` shows Mapbox's look, and `mapbox` the theme's.
   - **Render cost:** steady-state `render_ms` on the diagnostics topic, in clay mode, within
     1.5× the `osm` source at the same pose.
   - **Secrets:** no `MAPBOX_TOKEN` value in the node log or the rig logs.
4. Cutting the network mid-run switches to the baked fallback within eight failed requests.

## 12. Risks and known limits

- **Undocumented endpoint:** `3dtiles/v1` is not a published Mapbox API and can change without
  notice. The loader validates what it reads and degrades to extrusions only.
- **Conflation by centroid:** a part whose centroid falls outside a landmark footprint but whose
  body overlaps the model stays, so geometry doubles there. A neighbour whose centroid falls
  inside is lost.
- **Flat ground:** buildings stand on map z = 0, and slopes are ignored until sub-project 2
  brings terrain.
- **Whole-tile loading:**
  - z14 tiles are about 2.2 km wide in Dubai and appear whole. The first takes up to about a
    second on a good link: 17 requests plus the build.
  - New gltfio assets are created on the render thread, so a frame stalls briefly when a large
    tile lands, as with ion.
- **Two meshoptimizer copies on the link line:** Filament's older copy resolves first. It decodes
  codec v0 only, which is why the source requests the `-meshopt` variant and not GL JS's
  `-meshopt-v2-lod` (codec v1; §3). If Mapbox moved `-meshopt` to codec v1 or
  `KHR_meshopt_compression`, decoding would fail, and the live smoke would show it as zero
  landmarks.
- **Cost:**
  - Vector tiles are billed per request: the 200k-per-month free tier covers about 12,500 z14
    tiles if only the 16 footprint requests are billed, about 11,700 if the landmark request is
    billed too.
  - 3D tiles have no published price.
  - There is no disk cache, by design.
- **Radius semantics differ from ion:** `radius` defaults to 500 m for `mapbox://`, while ion
  uses the theme's `tile_radius_m` (100 m in the shipped themes). For both, the node does not
  append its 700 m default.
- **Partial geo-datum overrides** are ignored with an ERROR log line. This is existing behaviour;
  the runbook documents it.

## 13. Next: sub-project 2

Imagery on terrain:

- raster overlays in Overlume's renderer (today its overlay callbacks are stubs);
- a terrain loader for Mapbox terrain-DEM (z14, 512 px; height = −10000 + (R·65536 + G·256 + B) ×
  0.1 m);
- satellite imagery through cesium's URL-template raster overlay;
- then buildings standing on that terrain.

It gets its own spec after this one ships.
