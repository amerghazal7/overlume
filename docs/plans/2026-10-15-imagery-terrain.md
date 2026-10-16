# Imagery on Terrain Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Real terrain under the streamed buildings with imagery draped on it, provider-agnostic: Mapbox Satellite on Mapbox terrain-DEM for the demo, Cesium World Terrain with ion imagery as the proof, both gated offline.

**Architecture:** Each streaming source gains an optional terrain `Tileset` beside its content tileset (cesium's own ion terrain loader, or our `MapboxTerrainLoader` over terrain-DEM tiles), imagery rides on the terrain tileset as cesium raster overlays, and the renderer implements the five overlay callbacks over a terrain material of ours. Mapbox buildings are re-based onto the DEM through a cache both Mapbox loaders share; the follow-terrain transform and `replaces_ground` now target the terrain tileset.

**Tech Stack:** C++20 (`overlume_stream`) and C++17-clean units, cesium-native 0.64 (`Cesium3DTilesSelection`, `CesiumRasterOverlays`, `CesiumQuantizedMeshTerrain`), Filament 1.56.5 (matc materials, gltfio), ROS 2 Humble node, Python tools and tests.

**Spec:** `docs/design/2026-10-15-imagery-terrain-design.md` (sections cited as §N). Sub-project 1's spec, `docs/design/2026-10-14-mapbox-buildings-design.md`, and its closed plan, `docs/plans/2026-10-14-mapbox-buildings.md`, define what already exists.

## Status ledger

| Task | Lane | State | Commit |
|------|------|-------|--------|
| 0. Build glue and lane worktrees | 0 | done; globs proven with probes, gate green at the fork db7e359 (442/424/8, 381, 94, 25, 6), lanes l1-l4 built; URL forms with consent: PASS (DEM tile HTTP 200, satellite tile HTTP 200) | none (no tracked change) |
| 1. DEM decode, sample and mesh | 1 | done | `db7e359..04908a2` |
| 2. Fixtures: DEM PNG, flat imagery, quantized-mesh | 1 | done | `04908a2..98bb981` |
| 3. Tiling helper extracted from the buildings loader | 2 | done | `db7e359..5c20d6c` |
| 4. URI keys: layer list, `imagery`, ion `terrain`/`imagery` | 2 | done | `5c20d6c..23f6bd9` |
| 5. Terrain material and the renderer's overlay callbacks | 3 | done | `db7e359..277d644` |
| 6. Node parameters, composer, attribution text | 4 | done | `db7e359..11eaa40` |
| 7. GUI sources, bridge, token check | 4 | done | `11eaa40..fe09029` |
| 8. Docs | 4 | done | `fe09029..992c883` |
| 9. DEM cache and buildings re-based onto the DEM | 5 | done | `199442c..0506187` |
| 10. Mapbox terrain loader and source wiring | 5 | done | `bf81642..9e22d9a` |
| 11. Second tileset, ion terrain and imagery, credits API | 6 | done | `199442c..3004004` |
| 12. End-to-end over the fixture network | 7 | done | `aa48d04..a3bd818` |
| 13. Golden candidates and promotion | 7 | done | `717415a..29b26f8` |
| 14. Opt-in live smoke | 7 | done; live run 2026-10-16 with the user's consent: operating area terrain tiles 2, DEM requests 2, building tiles 2, extruded 564, rasters attached 18, Maxar credit yes; `osm-terrain` terrain tiles 10, rasters attached 208; `tools/check_mapbox_token.sh` and `overlume/scripts/check_cesium_token.sh` PASS | `053b7b4` |
| 15. Rig rehearsal and close-out | 7 | pending | |

## Global Constraints

- **Tokens by name only.** `MAPBOX_TOKEN` and `CESIUM_ION_TOKEN` are never committed, echoed, logged or pasted into any file or transcript; scripts print PASS/FAIL and HTTP codes only; no test needs live network or a token. Overlay template URLs carry the token in their query string and are logged only through the redacting logger (`access_token=<redacted>`).
- **No Mapbox or ion bytes in the tree.** Fixtures are synthetic (§8). No Mapbox DEM, satellite or ion tile is ever committed.
- **No disk cache for Mapbox data** (Product Terms): the terrain-DEM and satellite requests go through accessors built with cache `off`; `environment_stream_has_disk_cache()` stays false for every `mapbox://` source. ion keeps `cache=`.
- **Public headers are POD-only and append-only** (ADR-0003/0004). This plan adds one free function to `overlume/include/overlume/scene.h`, `environment_source_credits`, and no struct change, so `kSceneVersion` stays; `overlume/scripts/check_pod_header.sh` must pass.
- **Attribution.** Whenever Mapbox content shows, the logo and the line show; with `imagery=satellite` the line carries "© Maxar"; with ion imagery on a Google source the node appends ion's credit, and an ion source without a Google line draws the credits alone (`AttributionKind::CREDITS`). `environment_attribution` and `environment_attribution_logo` keep their meaning.
- **GUI entries are environment sources**, never "presets", in every doc, label and log line (the code identifier `ENVIRONMENT_PRESETS` stays).
- **Behaviour changes ship a runnable check that fails when reverted;** every task ends green on `tools/ci_visual_mode.sh` in the foreground and is committed alone with a message that says what changed and why, ending with the `Claude-Session:` trailer the executing session supplies.
- **Goldens are promoted by a human** (Task 13's gate); golden frames are compared at a fixed frame index since renderer creation (Filament's temporal dithering makes a frame's bytes depend on its index).
- **Exact strings this plan pins** (§4, §5): the DEM template `https://api.mapbox.com/v4/mapbox.mapbox-terrain-dem-v1/{z}/{x}/{y}@2x.pngraw`, the satellite template `https://api.mapbox.com/v4/mapbox.satellite/{z}/{x}/{reverseY}@2x.jpg90` as passed to cesium's `UrlTemplateRasterOverlay` (cesium counts `{y}` from the south; `{reverseY}` is Mapbox's XYZ row; the request URL that leaves the process has the XYZ row in it), the satellite cap `maximumLevel = 18`, the satellite credit `© Maxar`, the DEM decode `−10000 + (R·65536 + G·256 + B)·0.1`, the mesh grid 65×65 vertices (64 cells, vertices at pixel coordinates 0, 8, …, 512, so neighbours share edge vertices), the skirt depth 20 m, the DEM cache size 32 tiles, the ion defaults terrain asset `1` and imagery asset `2`, the GUI entries `mapbox-satellite`, `mapbox-original-satellite`, `osm-terrain` and their URIs, the theme token `environment.terrain_color`, the node parameters `environment_terrain` and `environment_imagery`.
- **Never name internal repositories or local absolute paths in tracked files** (placeholders `$WORKTREE`, `$INTEGRATION`, `$SESSION_DIR`).

## Review Focus

Five input classes the spec implies but no task's own tests exercise unless the task listed here adds them:

1. A DEM tile whose edge heights differ from its neighbour's by several metres (a quantisation step across a tile edge): the skirt must hide the crack from the ego's viewpoint 50 m away. Pinned in Task 1 (mesh: edge vertices sit on the tile boundary; skirt depth 20 m) and Task 12 (two neighbouring fixture tiles with a 5 m step: no sky pixel between them along the shared edge at the probe row).
2. Imagery that arrives after the terrain tile is already drawn (the common case): the frame must show `terrain_color` first, then the texture, with no flash of white or black. Pinned in Task 5 (the material defaults `hasImagery = 0`; the attach sets the texture and the flag in one frame). Task 5 also pins the raster's orientation and cesium's translation/scale order (`EnvironmentStreamOverlay.ImageryKeepsNorthUp`), so Task 13's goldens inherit a checked mapping.
3. A long building spanning a slope: re-basing per centroid leaves one end floating by up to the height difference along it; a reasonable person expects it to sit, not float by metres. Pinned in Task 9 as a documented limit (a 20 m-long block on a 5% plane DEM, encoded by the fixture for this test, floats by less than 1 m, asserted), with the per-vertex alternative recorded in §12.
4. The user switches from `mapbox-satellite` to `mapbox` while imagery is attaching: rasters in flight must not touch a destroyed tile or texture. Pinned in Task 11 (teardown detaches overlays before destroying tilesets; a switch during a held raster request leaves `textures_alive` at 0 after teardown).
5. An ion terrain asset that answers 401 on `layer.json`: the source must still open with buildings and no terrain, say so once, and not fall back. Pinned in Task 11.

## Execution

Same method as sub-project 1: the orchestrator on the session model, implementers on Sonnet, review gates on Opus, at most two fix rounds per task, leftover minors applied by the orchestrator and recorded in its ledger. Everything stays on `mapbox-environment`; nothing is pushed until Task 15 closes the plan (the user's push policy: push at delivery, then PR #1's description gains this feature).

### Lanes

| Lane | Tasks | Starts after | Branch | ROS domain | Test tmp dir |
|------|-------|--------------|--------|-----------|--------------|
| 0 | 0 | this plan's commit | `mapbox-environment` | 77 | `/tmp` |
| 1 | 1, 2 | Task 0 | `terrain-env/l1` | 81 | `/tmp/overlume-l1` |
| 2 | 3, 4 | Task 0 | `terrain-env/l2` | 82 | `/tmp/overlume-l2` |
| 3 | 5 | Task 0 | `terrain-env/l3` | 83 | `/tmp/overlume-l3` |
| 4 | 6, 7, 8 | Task 0 | `terrain-env/l4` | 84 | `/tmp/overlume-l4` |
| 5 | 9, then 10 | 9: lanes 1 and 2 merged (Tasks 1–4). 10: Tasks 5 and 11 merged | `terrain-env/l5` | 85 | `/tmp/overlume-l5` |
| 6 | 11 | Tasks 2, 4, 5 merged | `terrain-env/l6` | 86 | `/tmp/overlume-l6` |
| 7 | 12, 13, 14, 15 | 12: every lane 1–6 merged. 13: Task 12 committed. 14: Task 13 committed. 15: Tasks 13 and 14 committed and the whole-branch review closed | `mapbox-environment` | 77 | `/tmp` |

Lanes 1–4 start together. Each lane touches only the files the File map gives it, so merges are conflict-free; a conflict is a plan deviation to report, not to resolve silently. Lane 7 edits, builds and gates in `$INTEGRATION`; while a lane 7 task has changes there, the orchestrator merges and commits nothing else there.

### Worktrees and builds

`$INTEGRATION` is the `mapbox-environment` worktree; lane worktrees are its siblings `$INTEGRATION-l1` … `$INTEGRATION-l6`, branches `terrain-env/lN`. Task 0 builds `$INTEGRATION` first, which warms ccache. Then, one lane at a time (the shared ezvcpkg prefix is not safe for concurrent configures):

```bash
git -C "$INTEGRATION" worktree add "$INTEGRATION-lN" -b terrain-env/lN mapbox-environment
cd "$INTEGRATION-lN"
mkdir -p /tmp/overlume-lN
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake \
    --toolchain "$PWD/overlume/cmake/toolchain-clang-libcxx.cmake" -S overlume -B overlume/build \
    -DOVERLUME_ENABLE_CESIUM=ON -DCMAKE_C_COMPILER_LAUNCHER=ccache \
    -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DOVERLUME_TEST_TMP_DIR_RUNTIME=/tmp/overlume-lN
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j16
ros/colcon_build.sh
```

Lanes 5 and 6 get their worktrees when they may start (lane 5 before Task 9; it merges `mapbox-environment` into `terrain-env/l5` again before Task 10, after Task 11 merged), from the merged `mapbox-environment` head, configured under the gate lock (`flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock cmake --toolchain …`).

**Later configures.** Every configure runs cesium's ezvcpkg step on the shared prefix and every gate configures inside the gate lock, so a lane's own configures take the same lock. A task that adds a `.cpp`, `.mat` or `test_*.cpp` file reconfigures before its first build and chains the build onto the reconfigure (the `CONFIGURE_DEPENDS` globs in `overlume/CMakeLists.txt` for `src/mapbox_*.cpp`, `assets/materials/*.mat` and `tests/test_*.cpp` would otherwise reconfigure inside `cmake --build`, outside the lock):

```bash
cd "$WORKTREE"
flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock cmake -S overlume -B overlume/build > /dev/null \
    && echo reconfigured \
    && CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
```

No `reconfigured` line and no CMake error above it means the lock wait ran out (exit 75) and nothing was built: run the step again.

**The gate in a lane** (every task's last step before its commit): prebuild at `-j8`, then the gate with the lane's ROS domain under the machine-wide gate lock (the WS bridge stage binds fixed ports, so gates never run concurrently; the lock waits at most 300 s, keeping the command inside the 600 s foreground limit):

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
ROS_DOMAIN_ID=8N CI_VISUAL_MODE_DOMAIN_ID=8N \
    flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock tools/ci_visual_mode.sh
```

Exit 75 means another lane held the lock for 300 s: run the same command again. Any other non-zero exit is a gate failure. Every `pytest` command passes `-p no:anyio`. Expected: six `PASS` lines and exit 0. Lanes 0 and 7 use domain 77 and no prefix.

**Baseline** (`mapbox-environment` at the spec commit 427f5d8, measured 2026-10-15): gate about 150 s. Gate counts: 442 library ctest entries (424 ok, 8 skipped), 381 node tests, 94 WS bridge tests, 25 goldens, 6 examples. A task's gate must show every count unchanged or higher, and the goldens all `ok`.

### Per-task loop

1. An implementer (Sonnet) follows the task's steps in the lane worktree, test first.
2. The gate is green; the implementer commits.
3. A review gate (Opus) checks the commit against the spec, the Global Constraints, the Interface contract and this plan, and checks that each new test fails when its behaviour is reverted.
4. At most two fix rounds, each ending green on the gate; leftover minors go to the orchestrator.
5. The orchestrator marks the ledger row.

### Merges

The orchestrator merges each finished lane into `mapbox-environment` with `git merge --no-ff <reviewed head>`, then runs the gate in `$INTEGRATION` (domain 77, under the gate lock) before the next merge. After the last merge, a whole-branch review runs before Task 15. Lane 7 runs in `$INTEGRATION`; its rows are marked after each of its commits.

## File map

| File | Tasks | Change |
|------|-------|--------|
| `overlume/CMakeLists.txt` | 0 | nothing expected: the `src/mapbox_*.cpp`, `assets/materials/*.mat` and `tests/test_*.cpp` globs already pick the new files up; Task 0 proves it with probe files and removes them. `tests/test_terrain_fixtures.cpp` is cesium-free (stb only), so hosted cesium-off CI builds it; the DEM round trip lives in `test_mapbox_dem.cpp` |
| `overlume/src/mapbox_dem.{hpp,cpp}` | 1 | create (C++17-clean) |
| `overlume/tests/test_mapbox_dem.cpp` | 1 | create |
| `overlume/tests/mapbox_dem_fixture.hpp`, `overlume/tests/imagery_fixture.hpp`, `overlume/tests/quantized_mesh_fixture.hpp`, `overlume/tests/test_terrain_fixtures.cpp` | 2 | create |
| `overlume/src/mapbox_tiling.{hpp,cpp}` | 3 | create (extracted) |
| `overlume/src/mapbox_loader.cpp` | 3, 4, 9, 10 | 3: tiling moved out, behaviour unchanged. 4: `parse_mapbox_spec` (same lane as 3). 9: DEM re-basing in `build_tile`. 10: `open_mapbox_source` builds the terrain tileset and the overlay; `make_mapbox_terrain_tileset` declared here |
| `overlume/src/mapbox_loader.hpp` | 4 | `MapboxSpec` layer list and `imagery` |
| `overlume/tests/test_mapbox_uri.cpp` | 4 | layer list and `imagery` cases |
| `overlume/src/environment_stream.cpp` | 4, 5, 11 | 4: `IonSpec` `terrain`/`imagery` keys, the `environment_stream_parse_terrain_assets` definition after `environment_stream_parse_radius`, and the `ion://{} refused: {}` line in `open_streaming_environment_source`. 5: terrain renderer resources and overlay callbacks. 11: the second tileset, overlays, sampling, `provides_ground`, credits, teardown |
| `overlume/src/environment_stream.hpp` | 5, 11 | 5: `StreamRendererResources::Kind`, overlay members. 11: terrain tileset members and `attach_terrain` |
| `overlume/src/environment_test_hooks.hpp` | 4, 5, 11 | 4: `environment_stream_parse_terrain_assets` after `environment_stream_parse_replaces_ground`. 5: `environment_stream_overlay_probe`, `install_fixture_terrain_with_file_imagery` appended at the end. 11: `environment_stream_has_terrain_tileset`, `environment_terrain_tiles_loaded`, ion fixture helpers appended after Task 5's |
| `overlume/tests/test_environment_stream.cpp` | 4, 5, 11 | 4: ion key parsing, right after the existing parse tests. 5: overlay callbacks over the file-based fixture tileset with file-URL imagery, appended at the end. 11: second tileset, ion proof, appended after Task 5's |
| `overlume/assets/materials/terrain.mat` | 5 | create |
| `overlume/src/renderer_internal.hpp`, `overlume/src/renderer.cpp` | 5 | `terrainMaterial` built at renderer creation beside `buildingMaterial`; theme token applied |
| `overlume/src/theme.{hpp,cpp}`, `overlume/src/theme_transition.cpp`, `overlume/assets/themes/dark_adas.yaml`, `overlume/assets/themes/light_clay.yaml`, `overlume/tests/test_theme.cpp`, `overlume/tests/test_theme_transition.cpp` | 5 | `environment.terrain_color`, blended like every colour |
| `overlume/src/mapbox_dem_cache.{hpp,cpp}` | 9 | create (C++20) |
| `overlume/src/mapbox_terrain_loader.cpp` | 10 | create (C++20) |
| `overlume/src/mapbox_test_hooks.hpp` | 9, 10 | 9: stats fields `dem_requests`; `mapbox_dem_url`. 10: `terrain_tiles_built`, `mapbox_satellite_url` |
| `overlume/tests/test_mapbox_buildings.cpp` | 9 | re-basing cases over the fixture network |
| `overlume/include/overlume/scene.h`, `overlume/src/environment.cpp` | 11 | `environment_source_credits` |
| `overlume/tests/test_mapbox_terrain.cpp` | 10, 12, 13, 14 | 10: create with the first terrain-tiles-built case. 12: end to end. 13: the golden tests. 14: the opt-in live tests |
| `overlume/tests/goldens/mapbox_terrain_synthetic_{dark_adas,light_clay}.png` | 13 | create (LFS), after the user's approval |
| `ros/src/overlume_ros/include/overlume_ros/environment_source_uri.hpp`, `ros/src/overlume_ros/include/overlume_ros/overlume_node.hpp`, `ros/src/overlume_ros/src/overlume_node.cpp`, `ros/src/overlume_ros/config/default_params.yaml`, `ros/src/overlume_ros/test/test_environment_source_uri.cpp`, `ros/src/overlume_ros/test/test_asset_paths.py` | 6, 12 | 6: parameters and their members, composer, `AttributionKind::CREDITS`, attribution text with credits (pure functions, unit-tested), and `ros/src/overlume_ros/test/test_environment_live_switch.py` check 6 (a `mapbox://buildings,terrain` switch with the token unset fails to open before any request). 12: the node's once-per-second read of `environment_source_credits` into the attribution line (members `environment_credits_`, `environment_credits_read_sec_`), after Task 11's function exists |
| `tools/vcam_ws_bridge.py`, `tools/vcam_gui.py`, `tools/test_vcam_ws_bridge.py`, `tools/check_mapbox_token.sh`, `tools/test_check_mapbox_token.py` | 7 | three entries; two more probes |
| `docs/runbooks/mapbox.md`, `docs/runbooks/terrain_imagery.md` (create), `docs/README.md`, `docs/status.md`, `CHANGELOG.md` | 8, 15 | 8: write. 15: the rehearsal's values and the close-out rows |
| `docs/plans/2026-10-15-imagery-terrain.md` | all | the orchestrator's ledger only |

## Interface contract

Signatures other tasks consume. A task implements its block exactly; a consumer builds against it exactly. A task may add members or helpers beyond its block when only its own lane consumes them; the blocks pin what crosses lanes.

### `overlume/src/mapbox_dem.hpp` (Task 1; C++17-clean, no cesium headers)

```cpp
namespace overlume::mapbox {
inline constexpr uint32_t kDemSize = 512;      // pixels per side of a terrain-DEM tile
inline constexpr uint32_t kDemGridCells = 64;  // mesh cells per side (one vertex every 8 pixels)
inline constexpr double kDemSkirtM = 20.0;     // skirt depth below the edge
struct DemTile {
    bool ok = false;
    std::string error;           // one line when !ok
    std::vector<float> height_m; // kDemSize * kDemSize, row-major, row 0 at the tile's north edge
    float at(uint32_t px, uint32_t py) const;  // px east, py south, both < kDemSize
};
// Decodes a terrain-DEM PNG (stb_image, 8-bit, 3 or 4 channels, exactly 512 x 512) with
// height = -10000 + (R * 65536 + G * 256 + B) * 0.1. Any other size, depth or decode failure: ok = false.
DemTile decode_dem_png(const uint8_t* bytes, size_t size);
// Bilinear height at pixel-centre coordinates (u east, v south, 0..512, clamped to the edge).
double dem_sample(const DemTile& dem, double u, double v);
struct DemMesh {
    std::vector<std::array<float, 3>> positions;  // tile frame metres, grid vertices first, then the skirt ring
    std::vector<std::array<float, 3>> normals;    // unit, from central differences; skirt copies its edge vertex
    std::vector<std::array<float, 2>> uvs;        // u east 0..1, v south 0..1 across the tile
    std::vector<uint32_t> indices;                // CCW seen from above, grid then skirt
    uint32_t grid_vertex_count = 0;               // (kDemGridCells + 1)^2
};
// Heights become z = height - z_ref_m; vertices sit at u, v = 0, 8, ..., 512 (pixel coordinates), so
// neighbouring tiles share their edge vertices and heights are bilinear samples at those points;
// positions through project_to_frame(frame, t14, u, v, kDemSize, z).
DemMesh mesh_dem(const DemTile& dem, const TileFrame& frame, const TileId& t14, double z_ref_m,
                 double skirt_m = kDemSkirtM);
}
```

### Test fixtures (Task 2; header-only, C++17, `overlume/tests/`)

```cpp
namespace overlume::testing::dem {
// A 512 x 512 RGB PNG whose pixel (px, py) encodes height_m(u = px + 0.5, v = py + 0.5) with the
// Mapbox terrain-RGB formula (stb_image_write). Heights are rounded to 0.1 m.
std::vector<uint8_t> encode_dem_png(const std::function<double(double u, double v)>& height_m);
// Named scenes used by several tasks: a ramp rising 0..20 m west to east; a hill of 40 m at the
// tile centre (gaussian, sigma 400 px); a shelf 10 m high on the north half; sea (0 everywhere).
double ramp_height_m(double u, double v);
double hill_height_m(double u, double v);
double shelf_height_m(double u, double v);
double sea_height_m(double u, double v);
}
namespace overlume::testing::imagery {
// A size x size RGB PNG in one colour (JPEG is not needed: the overlay reads any image cesium decodes).
std::vector<uint8_t> flat_png(uint32_t size, uint8_t r, uint8_t g, uint8_t b);
}
namespace overlume::testing::qmesh {
struct QuantizedMeshOptions { uint32_t grid = 8; double min_height_m = 0.0; double max_height_m = 0.0; };
// One quantized-mesh tile (header, zigzag-delta u/v/height, 16-bit indices, edge indices) covering
// the tile at x, y, level, flat at height (min == max) or a ramp between them.
std::vector<uint8_t> encode_quantized_mesh(uint32_t level, uint32_t x, uint32_t y, const QuantizedMeshOptions& o);
// A layer.json for a geographic tiling scheme with `available` for exactly the tiles listed.
std::string layer_json(const std::vector<std::array<uint32_t, 3>>& available_level_x_y);
// Cesium ion endpoint bodies the fixture network serves at https://api.cesium.com/v1/assets/{id}/endpoint
std::string ion_terrain_endpoint_json(const std::string& layer_json_base_url);  // type TERRAIN
std::string ion_imagery_endpoint_json(const std::string& tms_base_url);         // type IMAGERY, TMS url
std::string tilemapresource_xml(uint32_t tile_size);                           // for the TMS imagery
}
```

### `overlume/src/mapbox_tiling.hpp` (Task 3; C++20, `overlume_stream`; the contract's names replace today's internal ones: `region_of`, the body of `createTileChildren`, the `attempts_` map, `classify`, `LoaderState::first_time`)

```cpp
namespace overlume::mapbox {
inline constexpr uint32_t kMaxAttempts = 3;
inline constexpr double kRetryWaitS = 2.0;
struct Attempt { uint32_t count = 0; double not_before_s = 0.0; std::shared_ptr<std::atomic<bool>> settled; };
// Attempt bookkeeping keyed by z14 tile, main thread only (the loader's attempts_ map today).
class AttemptBook { public: bool waiting(const TileId& t14, double now_s) const; /* the retry wait still runs: RetryLater, no request */ Attempt& begin(const TileId& t14, double now_s); /* as the loader does today */ };
// Also exported, unchanged in meaning: struct LoaderConfig, kRegionBelowM, kRegionAboveM, clock_s (with mapbox_set_test_clock's definition), Fetched, Outcome, classify, fetch, note_status, success, and
// std::unique_ptr<Cesium3DTilesSelection::Tile> quadtree_root(Cesium3DTilesSelection::TilesetContentLoader& loader, double origin_height_m). Task 3's text lists the exact signatures; Task 10 consumes them.
// The z14 quadtree: root volume and children of a tile. The radius selection stays where it is today: the stream's area view (environment_stream.cpp) serves both tilesets; nothing moves for it.
Cesium3DTilesSelection::BoundingVolume tile_region(const TileId& t, double min_height_m, double max_height_m, const CesiumGeospatial::Ellipsoid& = CesiumGeospatial::Ellipsoid::WGS84);
std::vector<Cesium3DTilesSelection::Tile> quadtree_children(const Cesium3DTilesSelection::Tile& parent, const LoaderConfig& cfg, const CesiumGeospatial::Ellipsoid& = CesiumGeospatial::Ellipsoid::WGS84);
bool transient_status(uint16_t status);  // 429, 5xx, 0 (transport)
struct SourceLog { bool first_time(const std::string& key); };  // one line per key per source
}
```

### `overlume/src/mapbox_loader.hpp` additions (Task 4; C++17-clean)

```cpp
namespace overlume::mapbox {
struct MapboxSpec {
    bool buildings = true;        // layer list carries `buildings`
    bool terrain = false;         // layer list carries `terrain`
    bool imagery_satellite = false;  // imagery=satellite (requires terrain)
    bool materials_original = false; double radius_m = 500.0; double brightness = 1.0; std::string fallback_dir;
};
// Layer list: a comma list of `buildings` and `terrain`, each at most once, any order; empty,
// repeated or unknown names refuse ("mapbox:// supports only the layers 'buildings' and 'terrain', not '<name>'").
// imagery=off|satellite; satellite without terrain refuses ("mapbox:// imagery=satellite needs the
// 'terrain' layer"). Every refusal's error starts with "mapbox://".
MapboxSpecResult parse_mapbox_spec(const std::string& spec);
}
```

### `IonSpec` additions (Task 4; `environment_stream.cpp`, internal; tested through `environment_stream_parse_*` hooks)

```cpp
std::optional<int64_t> terrain_asset;  // terrain=<id>|off (default off)
std::optional<int64_t> imagery_asset;  // imagery=<id>|off (default off); set without terrain_asset -> parse fails
// environment_test_hooks.hpp (Task 4 declares it right after environment_stream_parse_replaces_ground; Task 5 appends its hooks at the end of the file, so the two lanes never touch the same lines):
// bool environment_stream_parse_terrain_assets(const char* ion_spec, int64_t* terrain, int64_t* imagery, bool* ok);
// returns true when the spec parses with a terrain asset; ids are 0 when off, absent or failed; every pointer may be null.
// Asset ids: digits only, > 0, int64; `off` lowercase; last value wins; an earlier bad value still fails.
```

### `overlume/src/environment_stream.hpp` additions (Tasks 5, 11)

```cpp
class StreamRendererResources /* existing */ {
public:
    enum class Kind : uint8_t { Content, Terrain };
    // Task 5: Terrain resources swap every tile's material for a terrain.mat instance in
    // prepareInMainThread and implement the five overlay callbacks; Content keeps today's behaviour.
    explicit StreamRendererResources(const glm::dmat4& ecefToMap, double anchorHeightM, bool materialsOriginal = false,
                                     double brightness = 1.0, Kind kind = Kind::Content);
    // Task 5: counters for the overlay probe.
    struct OverlayCounters { uint32_t rasters_attached = 0, rasters_detached = 0, textures_alive = 0, textures_freed = 0; };
    OverlayCounters overlay_counters() const;
};
class StreamingEnvironmentSource /* existing */ {
public:
    // Task 11: a second tileset. Called once before the first update(); overlays are already added
    // to `terrain->getOverlays()` by the caller. provides_ground(), height sampling and the
    // terrain transform target the terrain tileset when present.
    void attach_terrain(std::unique_ptr<Cesium3DTilesSelection::Tileset> terrain,
                        std::shared_ptr<StreamRendererResources> terrain_resources);
    bool has_terrain() const;
    // Task 11: the overlay credits of both tilesets, plain text, "; " separated, HTML stripped.
    std::string credits() const;
};
```

### `overlume/assets/materials/terrain.mat` (Task 5)

Lit, double-sided off, parameters: `float3 terrainColor`, `int hasImagery` (0/1), `sampler2d imagery` (trilinear, clamp to edge), `float4 uvTransform` (translation in .xy, scale in .zw: uv' = uv * uvTransform.zw + uvTransform.xy; the material samples at (uv'.x, 1 − uv'.y) because the raster's row 0 is its north edge while cesium's v = 0 is the south), `float brightness`. No fade: the stream applies none to new tiles today (they get the opaque clay instance), and terrain tiles behave the same. The imagery texture is 8-bit RGBA in the sRGB format (`SRGB8_A8`), mipmapped; detach sets `hasImagery = 0` and binds the renderer's 1×1 white placeholder (`renderer_internal.hpp` gains `filament::Texture* terrainPlaceholderTexture` beside `terrainMaterial`). Base colour = `hasImagery ? texture(imagery, uv') * brightness : terrainColor`. `renderer_internal.hpp`: `filament::Material* terrainMaterial`; `renderer.cpp` builds it beside the clay material and applies `theme.environment.terrain_color`.

### Theme (Task 5)

`Theme::Environment` (`overlume/src/theme.hpp`) gains `detail::Float3 terrain_color` beside `tile_radius_m`; YAML `environment: { tile_radius_m: …, terrain_color: [r, g, b] }`, default the theme's ground colour (`palette.ground`) when absent; `theme_transition.cpp` blends it like every other colour (the sentinel test in `test_theme_transition.cpp` guards the field).

### `overlume/src/environment_test_hooks.hpp` additions (Tasks 5, 11)

```cpp
namespace overlume::testing {
struct OverlayProbe { uint32_t rasters_attached, rasters_detached, textures_alive, textures_freed; };
OverlayProbe environment_stream_overlay_probe(overlume::VisualRenderer* r);  // Task 5 (terrain resources; zeros otherwise)
// Task 5: opens the file-based fixture tileset (as install_fixture_streaming_source) as a TERRAIN
// tileset with a UrlTemplateRasterOverlay whose template is `file://<dir>/{z}/{x}/{y}.png`.
bool install_fixture_terrain_with_file_imagery(overlume::VisualRenderer* r, const char* fixture_dir, const char* imagery_dir,
                                               double brightness = 1.0, uint32_t maximum_level = 10);
bool environment_stream_has_terrain_tileset(overlume::VisualRenderer* r);  // Task 11
uint64_t environment_terrain_tiles_loaded(overlume::VisualRenderer* r);     // Task 11: terrain tiles in the scene
}
```

### `overlume/src/mapbox_dem_cache.hpp` (Task 9; C++20)

```cpp
namespace overlume::mapbox {
class DemCache {
public:
    DemCache(std::shared_ptr<CesiumAsync::IAssetAccessor> accessor, CesiumAsync::AsyncSystem async,
             std::string dem_url_template_with_query, std::shared_ptr<spdlog::logger> redacting_logger);
    // One in-flight request per z14 tile, shared by every caller; LRU of 32 decoded tiles. The future
    // resolves to nullptr on 204/404 (empty) and on a permanent failure (one log line per status, like
    // the loader); after a transient failure the entry is dropped and the next caller asks again after
    // kRetryWaitS / 2.
    CesiumAsync::SharedFuture<std::shared_ptr<const DemTile>> get(const TileId& t14);
    // The DEM height at the anchor, memoised once it has a value; it shares the anchor tile's entry
    // (the buildings loader asks for it right after its own 17 requests, so no extra request is made).
    CesiumAsync::SharedFuture<std::optional<double>> reference_z(const LonLatDeg& anchor);
    uint64_t requests() const;  // for stats
};
}
```

### `MapboxSourceStats` additions (Tasks 9, 10; `mapbox_test_hooks.hpp`, append only)

```cpp
uint64_t dem_requests = 0;          // Task 9: DEM tiles fetched by the cache
uint64_t terrain_tiles_built = 0;   // Task 10: terrain z14 tiles that reached Success with content
std::string mapbox_dem_url(uint32_t x14, uint32_t y14);                 // Task 9, no query string
std::string mapbox_satellite_url(uint32_t z, uint32_t x, uint32_t y);   // Task 10, no query string
```

### `overlume/include/overlume/scene.h` addition (Task 11; free function, no version bump)

```cpp
/// Plain-text credits of the open streaming source's raster overlays ("; " separated, HTML
/// stripped), written to `out` (NUL-terminated, truncated to `cap`); returns the length needed.
/// 0 and an empty string when no source or no overlay is open.
uint32_t environment_source_credits(VisualRenderer*, char* out, uint32_t cap);
```

### Node (Task 6; `environment_source_uri.hpp`)

```cpp
inline std::string compose_environment_source_uri(/* existing params */, const std::string& environment_terrain = "",
                                                  const std::string& environment_imagery = "");
// Values are strings that YAML never reads as bool or int: environment_terrain is "" (append nothing),
// "disable", "enable" (mapbox://: adds `terrain` to the layer list when absent) or "ion:<id>" (ion://:
// appends terrain=<id> when the URI lacks it; "disable" appends terrain=off). environment_imagery is
// "", "disable", "satellite" (mapbox://) or "ion:<id>" (ion://), appended as imagery=… when the URI
// lacks the key. A value meant for the other scheme is ignored, with one WARN line at configure.
// The URI always wins: "disable" cannot remove a key the URI carries (the runbook says so).
inline std::string environment_attribution_text(AttributionKind kind, const std::string& credits);
// MAPBOX: "© Mapbox © OpenStreetMap" + (" " + each credit) + " Improve this map"; GOOGLE: today's
// text + (" " + each credit); CREDITS: the credits alone; NONE: "".
// AttributionKind gains CREDITS: an ion:// source that is not GOOGLE but whose credits are non-empty
// (osm-terrain with Bing imagery) draws the credits line without a logo.
// environment_attribution_kind(visible, enabled, source_uri, state, credits_non_empty) gains the last
// argument (default false keeps today's callers).
```

### Texts other tasks match verbatim

- Loader log lines: `mapbox:// terrain tile 14/{}/{} skipped: {}` (Task 9, the DEM cache, which decodes), `Mapbox rejected MAPBOX_TOKEN (HTTP {})` once per endpoint for the DEM (Task 9), `ion://{} refused: {}` for a refused ion URI (Task 4), `environment source: terrain tileset did not open (HTTP {}) -- continuing without terrain` (Task 11). No task emits a per-tile imagery failure line: cesium retries a failed raster tile through its parents and logs `Received response code {} for image {}.` through the source's redacting logger only when no parent can stand in, so the token is masked and the host may show; tests assert the token's absence, never the host's. Today's `mapbox://{} refused: {}` (mapbox_loader.cpp) stays as it is and Task 12 relies on it.
- `tools/check_mapbox_token.sh` line: `PASS: token retrieves the Downtown Dubai tiles (landmark tile HTTP 200, footprint tile HTTP 200, DEM tile HTTP 200, satellite tile HTTP 200)` (Task 7); FAIL and SKIP forms as today with the four codes.
- `docs/runbooks/mapbox.md` §3 table rows for `imagery` and the layer list (Task 8); Task 15 edits nothing there unless the rehearsal finds drift.

## Tasks

### Task 0: Build glue and lane worktrees

**Lane:** 0 (`$INTEGRATION`, branch `mapbox-environment`, ROS domain 77 with no prefix, test tmp dir
`/tmp`). The orchestrator runs it, as in sub-project 1, before any lane worktree exists: Step 6
creates the worktrees. Two steps reach the network. Step 6's fresh lane configures download the
Filament SDK, the cesium-native tarball, yaml-cpp, googletest and stb from GitHub, with no token
(`overlume/cmake/GetFilament.cmake:350`, `overlume/cmake/GetCesiumNative.cmake:72`, as sub-project
1's lane configures did). Step 8 sends two token-carrying requests to `api.mapbox.com`, only with
the user's consent (Step 7).
**Depends on:** this plan's commit.

Every command block below is one Bash call. Shell state does not carry over between calls, so each
block starts with `cd "$INTEGRATION"`, and the executor sets `INTEGRATION` (the absolute path of the
`mapbox-environment` worktree) at the top of the call. Never print a token, never use `set -x`, and
push nothing.

**Files:**
- No tracked file changes (File map: `overlume/CMakeLists.txt`, "nothing expected"). Task 0 has no
  commit.
- Create, then delete in Step 4 (never committed): `overlume/src/mapbox_probe.cpp`,
  `overlume/assets/materials/probe.mat`, `overlume/tests/test_mapbox_probe.cpp`
- Outside the repository: the worktrees `$INTEGRATION-l1` … `$INTEGRATION-l4` and the directories
  `/tmp/overlume-l1` … `/tmp/overlume-l4` (Step 6)
- Test: no committed test. Step 3 is the check; Step 5's gate confirms the fork point is green.

**Interfaces:**
- Consumes: nothing.
- Produces, for every lane:
  - Proven unchanged (Steps 2–3): a new `overlume/src/mapbox_*.cpp` compiles as C++20 in
    `overlume_stream`, sees `<CesiumRasterOverlays/*.h>` and vcpkg's `<stb_image.h>`, and its
    symbols reach the test executables through `liboverlume.a`. A new
    `overlume/assets/materials/<name>.mat` is compiled by matc at configure into
    `overlume/build/generated/overlume_materials/<name>_filamat.h`, which declares
    `overlume::materials::k<name>Filamat` and `k<name>FilamatSize` (`terrain.mat` gives
    `kterrainFilamat` and `kterrainFilamatSize`). A new `overlume/tests/test_mapbox_*.cpp` is a
    cesium-only executable with `<meshoptimizer.h>`, and its gtest cases register in ctest.
  - Any other new `overlume/tests/test_*.cpp`, `test_terrain_fixtures.cpp` included, is built in
    cesium-off builds too (the hosted build), so it must reference no `overlume_stream` symbol
    (`src/environment_stream.cpp`, `src/mapbox_*.cpp`). Task 2 complies: its test reads the DEM
    PNGs back with stb_image and the published formula, not `decode_dem_png`.
  - The worktrees `$INTEGRATION-l1` … `$INTEGRATION-l4` on branches `terrain-env/l1` …
    `terrain-env/l4` at `mapbox-environment`'s head (the plan's commit), each configured with
    `OVERLUME_TEST_TMP_DIR_RUNTIME=/tmp/overlume-lN` and built (library and node).
  - Ledger row 0 records whether the pinned terrain-DEM and satellite URL forms answer 200
    (Steps 7–8, spec §3).

**API notes:**
- Globs (`overlume/CMakeLists.txt`): `:208` globs `src/*.cpp` with `CONFIGURE_DEPENDS`. `:238-243`
  glob `src/environment_stream.cpp` and `src/mapbox_*.cpp` into `_overlume_stream_srcs` and always
  remove them from `OVERLUME_SOURCES`, so a cesium-off build compiles no `mapbox_*.cpp`. `:261-270`
  build them as the `overlume_stream` OBJECT library (`cxx_std_20`; its `flags.make` shows
  `-std=gnu++20`) only when cesium is on. `:585` globs `tests/*.cpp`. `:587-590` drop
  `test_environment_stream.cpp` and the `/test_mapbox_[^/]*\.cpp$` regex when cesium is off, and
  nothing else. `:591-596` copy `meshoptimizer.h` alone into `build/meshopt_include` when cesium is
  on. `:612-635` make each test its own executable with `src` and `STB_DIR` on its include path,
  linking `overlume gtest gtest_main`, plus `meshopt_include` for `test_mapbox_*`. `:641-654`
  register its cases with `gtest_discover_tests` (label `cpu`).
- Materials: `:180` globs `assets/materials/*.mat` with `CONFIGURE_DEPENDS`, and `:181` adds every
  `.mat` to `CMAKE_CONFIGURE_DEPENDS`. Editing an existing `.mat` therefore reconfigures at the next
  `cmake --build`, outside the gate lock, so Task 5 runs the reconfigure block (Execution) after
  each edit of `terrain.mat`. `:182-205` run matc at configure when a `.mat` is newer than its
  header (a matc error is a configure `FATAL_ERROR`) and write
  `${CMAKE_BINARY_DIR}/generated/overlume_materials/<name>_filamat.h` with
  `inline constexpr uint8_t k<name>Filamat[]` and `k<name>FilamatSize` in `overlume::materials`.
  That directory is on `overlume_obj`'s include path (`:290-292`) but not on `overlume_stream`'s
  (`:269`): `overlume/src/renderer.cpp:70-82` includes the material headers, so `terrain_filamat.h`
  belongs there (Task 5). matc 1.56.5 compiled this task's 13-line unlit probe material in a dry run.
- Overlay headers: `Cesium3DTilesSelection` links `CesiumQuantizedMeshTerrain` and
  `CesiumRasterOverlays` PUBLIC
  (`overlume/build/_deps/cesium-native-src/Cesium3DTilesSelection/CMakeLists.txt:42-57`), so their
  headers reach `overlume_stream`. The vendor-merge walk (`overlume/CMakeLists.txt:323-358`)
  already puts `IonRasterOverlay`'s constructors into `liboverlume.a` (`nm` shows
  `T _ZN20CesiumRasterOverlays16IonRasterOverlayC1…`). Signatures used by the probe:
  `UrlTemplateRasterOverlay(name, url, headers = {}, UrlTemplateRasterOverlayOptions = {}, RasterOverlayOptions = {})`
  (`CesiumRasterOverlays/include/CesiumRasterOverlays/UrlTemplateRasterOverlay.h:110-115`; options
  `credit` `:32`, `maximumLevel = 25` `:57`, `tileWidth = 256` `:62`, `tileHeight = 256` `:67`) and
  `IonRasterOverlay(name, ionAssetID, ionAccessToken, RasterOverlayOptions = {}, ionAssetEndpointUrl = "https://api.cesium.com/")`
  (`IonRasterOverlay.h:40-45`). Both derive from `ReferenceCountedNonThreadSafe<RasterOverlay>`
  (`RasterOverlay.h:140-141`) and are held by `CesiumUtility::IntrusivePointer`, whose
  `IntrusivePointer(T* p = nullptr)` is implicit (`CesiumUtility/include/CesiumUtility/IntrusivePointer.h:31`).
- stb_image in a `src/mapbox_*.cpp`: vcpkg's `stb_image.h` is on `overlume_stream`'s include path
  (the vcpkg `installed/<triplet>/include` directory the cesium targets carry as `-isystem`), but
  `STB_DIR` is not (it reaches tests at `:619-620` and `render_hero` at `:665` only). cesium
  compiles its own copy with `STB_IMAGE_STATIC`
  (`overlume/build/_deps/cesium-native-src/CesiumImage/src/ImageDecoder.cpp:20-24`), and every test
  executable links `overlume/tests/golden.cpp`, which defines a non-static
  `STB_IMAGE_IMPLEMENTATION` (`:6`). A stream unit therefore compiles its own copy with
  `STB_IMAGE_STATIC` and `STBI_ONLY_PNG`. The probe proves that compiles in `overlume_stream` and
  links beside `golden.cpp`'s copy: Task 1 needs no CMake change and no vendored header.
- Cesium off: `.github/workflows/build.yml:6` runs on every pull request (PR #1 included), `:32-37`
  configure with `-DOVERLUME_ENABLE_CESIUM=OFF`, and `:42-55` run `ctest -L cpu`. The gate cannot
  see a cesium-off link failure: `tools/ci_visual_mode.sh:56-57` reconfigures the existing cesium-on
  cache. Hence the second "Produces" bullet.
- Worktrees: `$INTEGRATION` is itself a linked worktree, so `git -C "$INTEGRATION" worktree add`
  registers the lanes in the same repository, and the LFS filters (`filter.lfs.*`, required)
  smudge the goldens on checkout. The node links `${OVERLUME_DIR}/build/liboverlume.a`
  (`ros/src/overlume_ros/CMakeLists.txt:50`), so each worktree builds its library before
  `ros/colcon_build.sh`, which builds the one package `overlume_ros`. The configure and build blocks
  start with `cd "$INTEGRATION-l$N" &&`, so a missing worktree stops them before anything touches
  `$INTEGRATION`'s own build. `/tmp/overlume-l1` … `-l7` remain from sub-project 1, and `mkdir -p`
  keeps them. `flock -w 300 -E 75` prints nothing when its wait runs out.
- Token-safe request: `tools/check_mapbox_token.sh:13-19` trims the token as the library does, and
  `:25-37` give `curl --config -` the URL on stdin and print only the HTTP code or curl's exit
  status. The login-shell read on fd 3 is sub-project 1's Task 16 Step 8. Downtown Dubai's z14
  tile is 10707/7006 in Mapbox's XYZ rows (`tools/check_mapbox_token.sh:38`, `:43`). Step 8's
  satellite request carries that XYZ row, as the request leaving the process does: the template
  the plan pins for cesium writes the row as `{reverseY}`, because cesium's `{y}` counts from the
  south (`overlume/build/_deps/cesium-native-src/CesiumRasterOverlays/src/UrlTemplateRasterOverlay.cpp:87-92`).
  Terrain-DEM tiles exist to z14 (spec §3).
- Gate baseline at the spec commit: 442 ctest entries (424 ok, 8 skipped), node
  `Summary: 381 tests, 0 errors, 0 failures, 0 skipped`, WS bridge `94 passed`, 25 goldens,
  6 examples. These were read back from this worktree's last gate log. Task 0 adds no test.

- [ ] **Step 1: Build the integration worktree** (also warms ccache for the lanes; an up-to-date
  build compiles nothing)

```bash
cd "$INTEGRATION"
flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock cmake \
    --toolchain "$PWD/overlume/cmake/toolchain-clang-libcxx.cmake" -S overlume -B overlume/build \
    -DOVERLUME_ENABLE_CESIUM=ON -DCMAKE_C_COMPILER_LAUNCHER=ccache \
    -DCMAKE_CXX_COMPILER_LAUNCHER=ccache > /dev/null \
    && echo configured \
    && CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j16 \
    && ros/colcon_build.sh 2>&1 | grep '^Summary:'
```

Expected: `configured`, the build finishes without an error, then
`Summary: 1 package finished [...]`. No `configured` line and no CMake error above it means the
lock wait ran out (exit 75): run the step again.

- [ ] **Step 2: Write the three throwaway probes**

```bash
cd "$INTEGRATION"
cat > overlume/src/mapbox_probe.cpp <<'EOF'
// Throwaway (Task 0): proves src/mapbox_*.cpp compile as C++20 in overlume_stream, with the raster
// overlay headers and vcpkg's stb_image.h on the include path. Never committed.
static_assert(__cplusplus >= 202002L, "src/mapbox_*.cpp must compile as C++20");

#include <CesiumRasterOverlays/IonRasterOverlay.h>
#include <CesiumRasterOverlays/UrlTemplateRasterOverlay.h>
#include <CesiumUtility/IntrusivePointer.h>

#define STB_IMAGE_STATIC
#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_PNG
#include <stb_image.h>

#include <string>

namespace overlume::testing {

std::string mapbox_probe_overlay_names() {
    CesiumRasterOverlays::UrlTemplateRasterOverlayOptions options;
    options.credit = "probe";
    options.maximumLevel = 18;
    options.tileWidth = 512;
    options.tileHeight = 512;
    const CesiumUtility::IntrusivePointer<CesiumRasterOverlays::RasterOverlay> url =
        new CesiumRasterOverlays::UrlTemplateRasterOverlay(
            "mapbox-satellite", "https://example.invalid/{z}/{x}/{y}.jpg", {}, options);
    const CesiumUtility::IntrusivePointer<CesiumRasterOverlays::RasterOverlay> ion =
        new CesiumRasterOverlays::IonRasterOverlay("ion-imagery", 2, "");
    return url->getName() + "," + ion->getName();
}

int mapbox_probe_png_channels(const unsigned char* bytes, int size) {
    int w = 0, h = 0, channels = 0;
    return stbi_info_from_memory(bytes, size, &w, &h, &channels) ? channels : -1;
}

}
EOF
cat > overlume/assets/materials/probe.mat <<'EOF'
// Throwaway (Task 0): proves assets/materials/*.mat are compiled by matc at configure.
material {
    name : probe,
    shadingModel : unlit,
    parameters : [
        { type : float3, name : probeColor }
    ]
}
fragment {
    void material(inout MaterialInputs material) {
        prepareMaterial(material);
        material.baseColor = vec4(materialParams.probeColor, 1.0);
    }
}
EOF
cat > overlume/tests/test_mapbox_probe.cpp <<'EOF'
// Throwaway (Task 0): proves tests/test_mapbox_*.cpp build cesium-only, see meshoptimizer.h and
// link what src/mapbox_*.cpp define. Never committed.
#include <gtest/gtest.h>
#include <meshoptimizer.h>

#include <string>

namespace overlume::testing {
std::string mapbox_probe_overlay_names();
int mapbox_probe_png_channels(const unsigned char* bytes, int size);
}

TEST(MapboxProbe, StreamUnitReachesTestsWithMeshopt) {
    EXPECT_EQ(overlume::testing::mapbox_probe_overlay_names(), "mapbox-satellite,ion-imagery");
    const unsigned char not_a_png[4] = {'n', 'o', 'p', 'e'};
    EXPECT_EQ(overlume::testing::mapbox_probe_png_channels(not_a_png, 4), -1);
    EXPECT_GT(meshopt_encodeVertexBufferBound(3, 12), 0u);
}
EOF
git status --short
```

Expected:

```
?? overlume/assets/materials/probe.mat
?? overlume/src/mapbox_probe.cpp
?? overlume/tests/test_mapbox_probe.cpp
```

(and the orchestrator's ` M docs/plans/2026-10-15-imagery-terrain.md` if its ledger has an
uncommitted edit; no step of this task stages it).

- [ ] **Step 3: Reconfigure, build and run the probes (cesium on)**

```bash
cd "$INTEGRATION"
flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock cmake -S overlume -B overlume/build > /dev/null \
    && echo reconfigured \
    && CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
```

Expected: `reconfigured`, then the build finishes without an error. No `reconfigured` line and no
CMake error above it means the lock wait ran out (exit 75): run the step again.

```bash
cd "$INTEGRATION"
grep -c 'kprobeFilamat\[\]' overlume/build/generated/overlume_materials/probe_filamat.h
overlume/build/test_mapbox_probe | tail -1
ctest --test-dir overlume/build -N -R '^MapboxProbe\.' | tail -1
```

Expected:

```
1
[  PASSED  ] 1 test.
Total Tests: 1
```

Anything else (a build error in a probe, a missing header or binary, a count other than these)
means the File map's "nothing expected" for `overlume/CMakeLists.txt` is wrong. That is a plan
deviation: stop, leave the probes in place, and report the output to the orchestrator, who decides
the CMake change and amends the File map before any lane starts.

- [ ] **Step 4: Remove the probes and their build outputs**

```bash
cd "$INTEGRATION"
rm overlume/src/mapbox_probe.cpp overlume/assets/materials/probe.mat \
    overlume/tests/test_mapbox_probe.cpp
rm -f overlume/build/test_mapbox_probe \
    overlume/build/generated/overlume_materials/probe.filamat \
    overlume/build/generated/overlume_materials/probe_filamat.h
flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock cmake -S overlume -B overlume/build > /dev/null \
    && echo reconfigured \
    && CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    && ctest --test-dir overlume/build -N | tail -1 \
    && git status --short
```

Expected: `reconfigured`, the build without an error, `Total Tests: 442` (the baseline's ctest
entries), and no status line (or only the orchestrator's
` M docs/plans/2026-10-15-imagery-terrain.md` if its ledger has an uncommitted edit). No
`reconfigured` line and no CMake error above it means the lock wait ran out (exit 75): run the
`flock` command and its chain again.

- [ ] **Step 5: Gate** (foreground, Bash timeout 600000 ms): the fork point every lane starts from

```bash
cd "$INTEGRATION"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock tools/ci_visual_mode.sh
```

Expected: six `PASS` lines, `OVERALL: PASS`, exit 0, and the baseline counts exactly, since Task 0
changes nothing:

```
PASS  POD header check
PASS  library ctest suite  (100% tests passed, 0 tests failed out of 442 (424 ok / 8 skipped))
PASS  node gtests  (Summary: 381 tests, 0 errors, 0 failures, 0 skipped)
PASS  WS bridge pytest suite  (94 passed in …)
PASS  golden suite: 25 ok, 0 skipped
PASS  examples  (6 run, all exited 0 with non-empty output)
```

Exit 75 means another session's gate held the lock for 300 s: run the same command again. Any
other non-zero exit is a gate failure at the plan's commit: stop and report it; no lane starts.

Task 0 has no commit, so there is nothing for a review gate to check. The orchestrator records
ledger row 0 as `done` with the Commit cell `none (no tracked change)` after Step 8 (or after
Step 7 on a no).

- [ ] **Step 6: Create the lane worktrees 1–4** (one lane at a time: the shared ezvcpkg prefix is
  not safe for concurrent configures)

Run Blocks A, B and C with `N=1`; then all three with `N=2`, `N=3` and `N=4`. The first line is the
only change. Each block is its own Bash call (foreground, timeout 600000 ms). The commands are
Execution's "Worktrees and builds" block, with the configure under the gate lock like every other
configure in this plan.

Block A, the worktree:

```bash
N=1
cd "$INTEGRATION" \
    && git -C "$INTEGRATION" worktree add "$INTEGRATION-l$N" -b "terrain-env/l$N" mapbox-environment \
    && mkdir -p "/tmp/overlume-l$N" \
    && file -b "$INTEGRATION-l$N/overlume/tests/goldens/mapbox_buildings_synthetic_dark_adas.png"
```

Expected (for `N=1`; the others alike with their number), where `HEAD is now at` names
`mapbox-environment`'s head, the plan's commit:

```
Preparing worktree (new branch 'terrain-env/l1')
HEAD is now at <hash> <the plan commit's subject>
PNG image data, 320 x 240, 8-bit/color RGB, non-interlaced
```

A `fatal:` from `worktree add` (the branch or the directory already exists) stops the block: report
it, do not delete anything. `ASCII text` instead of `PNG image data` means LFS did not smudge the
goldens: report it.

Block B, the configure:

```bash
N=1
cd "$INTEGRATION-l$N" \
    && CCACHE_BASEDIR="$(dirname "$INTEGRATION")" flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock \
        cmake --toolchain "$PWD/overlume/cmake/toolchain-clang-libcxx.cmake" -S overlume -B overlume/build \
        -DOVERLUME_ENABLE_CESIUM=ON -DCMAKE_C_COMPILER_LAUNCHER=ccache \
        -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DOVERLUME_TEST_TMP_DIR_RUNTIME="/tmp/overlume-l$N" \
        > "/tmp/overlume-l$N/configure.log" 2>&1 \
    && tail -1 "/tmp/overlume-l$N/configure.log"
```

Expected: `-- Build files have been written to: <$INTEGRATION-l1>/overlume/build`. No output and an
empty `/tmp/overlume-lN/configure.log` means the lock wait ran out (exit 75): run Block B again.
No output and a non-empty log is a configure failure: read the log.

Block C, the library and the node:

```bash
N=1
cd "$INTEGRATION-l$N" \
    && CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j16 \
        > "/tmp/overlume-l$N/build.log" 2>&1 \
    && echo built \
    && ros/colcon_build.sh 2>&1 | grep '^Summary:'
```

Expected: `built`, then `Summary: 1 package finished [...]`. No `built` line: read
`/tmp/overlume-lN/build.log`.

After lane 4:

```bash
cd "$INTEGRATION"
git -C "$INTEGRATION" worktree list | grep -c '\[terrain-env/l[1-4]\]$'
head="$(git -C "$INTEGRATION" rev-parse mapbox-environment)"
for n in 1 2 3 4; do
    if [[ "$(git -C "$INTEGRATION-l$n" rev-parse HEAD)" == "$head" ]]; then
        echo "l$n at mapbox-environment"
    else
        echo "l$n differs"
    fi
done
```

Expected:

```
4
l1 at mapbox-environment
l2 at mapbox-environment
l3 at mapbox-environment
l4 at mapbox-environment
```

Lanes 1–4 may start now.

- [ ] **Step 7: HUMAN (orchestrator). Ask the user's consent for the URL-form check (spec §3)**

Tell the user what the check does: two requests to `api.mapbox.com` with `MAPBOX_TOKEN`, one
terrain-DEM tile and one satellite tile of Downtown Dubai (z14 10707/7006), to confirm the two URL
templates the plan pins before Task 7 and lane 5 build on them. It prints HTTP codes only and keeps
no response body. Continue only on an explicit yes. On a no, write
`URL forms unconfirmed; Task 7's check confirms them` into ledger row 0 and end the task here.

- [ ] **Step 8: Confirm the two pinned URL forms (orchestrator)**

One command; never print the token.

```bash
cd "$INTEGRATION"
token="$(env -u MAPBOX_TOKEN bash -lic 'printf %s "${MAPBOX_TOKEN:-}" >&3' 3>&1 >/dev/null 2>&1)"
token="${token#"${token%%[![:space:]]*}"}"
token="${token%"${token##*[![:space:]]}"}"
if [[ -z "$token" ]]; then
    echo "SKIP: MAPBOX_TOKEN is not set (see docs/runbooks/mapbox.md)"
else
    codes=()
    for t in "DEM tile|https://api.mapbox.com/v4/mapbox.mapbox-terrain-dem-v1/14/10707/7006@2x.pngraw" \
             "satellite tile|https://api.mapbox.com/v4/mapbox.satellite/14/10707/7006@2x.jpg90"; do
        if code=$(printf 'url = "%s?access_token=%s"\n' "${t#*|}" "$token" \
                | curl -s --max-time 30 --config - -o /dev/null -w '%{http_code}'); then
            codes+=("${t%%|*} HTTP ${code}")
        else
            codes+=("${t%%|*} transport error (curl exit $?)")
        fi
    done
    if [[ "${codes[0]}" == "DEM tile HTTP 200" && "${codes[1]}" == "satellite tile HTTP 200" ]]; then
        echo "PASS: the pinned URL forms answer (${codes[0]}, ${codes[1]})"
    else
        echo "FAIL: ${codes[0]}, ${codes[1]}"
    fi
fi
```

Expected:

```
PASS: the pinned URL forms answer (DEM tile HTTP 200, satellite tile HTTP 200)
```

Write the line into ledger row 0. On `SKIP:`, export the token (runbook §1) and run the step again.
On `FAIL:`, hold Task 7 and lane 5 and take the codes to the user: 401 or 403 is the token (runbook
§2), and 404 means the pinned template (Global Constraints, "Exact strings") is wrong and needs the
user's ruling before any task builds on it. Lanes 1–3 and Task 6 do not use the URLs and continue.

### Task 1: DEM decode, sample and mesh

**Lane:** 1. **Depends on:** Task 0 (the lane worktree exists and is configured; the
`src/mapbox_*.cpp` glob, `overlume/CMakeLists.txt:238-240`, compiles `mapbox_dem.cpp` as C++20 in
`overlume_stream`, and the `tests/*.cpp` glob, `:585`, makes `test_mapbox_dem` a cesium-only test
binary through the `test_mapbox_*` filter, `:590`).

**Files:**
- Create: `overlume/src/mapbox_dem.hpp` (the contract, plus its includes)
- Create: `overlume/src/mapbox_dem.cpp` (C++17-clean, no cesium header; stb_image compiled static)
- Test: `overlume/tests/test_mapbox_dem.cpp` (suite `MapboxDem`, 11 tests, binary `test_mapbox_dem`)

**Interfaces:**
- Consumes: `TileId` (`overlume/src/mapbox_tile_frame.hpp:20-24`), `TileFrame` (`:48-53`),
  `TileFrame make_tile_frame(const TileId& z14, double origin_height_m);` (`:55`) and
  `std::array<double, 3> project_to_frame(const TileFrame& f, const TileId& src, double u, double v, double extent, double z_m);`
  (`:60-61`), all unchanged.
- Produces, for Tasks 9 and 10 (verbatim from the contract):
  - `inline constexpr uint32_t kDemSize = 512;`, `inline constexpr uint32_t kDemGridCells = 64;`,
    `inline constexpr double kDemSkirtM = 20.0;`
  - `struct DemTile { bool ok = false; std::string error; std::vector<float> height_m; float at(uint32_t px, uint32_t py) const; };`
  - `DemTile decode_dem_png(const uint8_t* bytes, size_t size);`
  - `double dem_sample(const DemTile& dem, double u, double v);`
  - `struct DemMesh { std::vector<std::array<float, 3>> positions; std::vector<std::array<float, 3>> normals; std::vector<std::array<float, 2>> uvs; std::vector<uint32_t> indices; uint32_t grid_vertex_count = 0; };`
  - `DemMesh mesh_dem(const DemTile& dem, const TileFrame& frame, const TileId& t14, double z_ref_m, double skirt_m = kDemSkirtM);`
  - all in `overlume::mapbox`. `DemTile::at` is defined inline in the header (one line, so
    consumers need no extra symbol).
- What consumers may rely on (pinned by `MapboxDem`):
  - `ok == false` means `height_m` is empty and `error` is exactly one of five fixed strings, one
    per kind, so Task 9's (the DEM cache's) `mapbox:// terrain tile 14/{}/{} skipped: {}` line can dedup by kind:
    `DEM tile is empty`, `DEM tile does not decode as PNG`, `DEM tile is not 512 x 512 pixels`,
    `DEM tile is not 8-bit`, `DEM tile has fewer than 3 channels`.
  - `dem_sample` returns 0.0 for a tile whose `height_m` is not 512 × 512 (an undecoded tile);
    NaN coordinates clamp to the west or north edge.
  - `mesh_dem` of a `!ok` tile is empty (every vector empty, `grid_vertex_count == 0`).
  - Grid vertex (i east, j south), 0 ≤ i, j ≤ 64, is `positions[j * 65 + i]`, placed at
    u = 8i, v = 8j tile pixels (so i = 0 and i = 64 lie on the tile's west and east edges) at
    z = `dem_sample(dem, 8i, 8j) - z_ref_m`. The skirt is `positions[4225 + k]`, k = 0..255, one
    per border vertex, ring order counter-clockwise seen from above starting at the north-west
    corner and running down the west edge; a skirt vertex copies its edge vertex's normal and UV.
    Counts: 4481 vertices, 26112 indices (24576 grid, 1536 skirt).

**API notes:**
- stb_image in the library. The tests get stb from `overlume/build/_deps/stb` (`overlume/CMakeLists.txt:152-166`),
  which is not on `overlume_stream`'s include path (`:269`: `src include`). But `overlume_stream`
  links `Cesium3DTilesSelection` and friends (`:264-268`), whose usage requirements put the
  ezvcpkg prefix's `installed/x64-linux-clang-libcxx/include` on its path
  (`overlume/build/CMakeFiles/overlume_stream.dir/flags.make`, `CXX_INCLUDES`), and that root holds
  `stb_image.h` v2.30, the same version as the tests' copy. cesium compiles it there static
  (`overlume/build/_deps/cesium-native-src/CesiumImage/src/ImageDecoder.cpp:18-24`:
  `STB_IMAGE_STATIC`, `STB_IMAGE_IMPLEMENTATION`, `STBI_NO_STDIO`), as does the node
  (`ros/src/overlume_ros/src/hud_overlay.cpp:9-14`, PNG only). Every test binary links golden.cpp's
  non-static implementation (`overlume/tests/golden.cpp:6-9`, `overlume/CMakeLists.txt:614-618`)
  with `--allow-multiple-definition` on the link line, so a non-static copy in the library would
  be silently merged with it; `mapbox_dem.cpp` therefore compiles stb static (`STBIDEF static`,
  `stb_image.h:396`), PNG-only, without stdio, and no CMake change is needed.
- stb_image v2.30 behaviour the decoder relies on: `stbi_info_from_memory` (`stb_image.h:494`)
  reads only the header, so the size and channel checks cost no decode; with `STBI_ONLY_PNG` a JPEG
  or an HTML error body fails it. `stbi_is_16_bit_from_memory` (`:496`, through `stbi__png_is16`,
  `:5329-5340`) reports the IHDR bit depth; stb never checks chunk CRCs, so the test makes a
  16-bit PNG by setting the IHDR depth byte (offset 24: 8 signature + 8 chunk header + 8
  width/height) of an 8-bit RGB PNG to 16. `stbi_load_from_memory(..., 3)` drops alpha. A PNG
  truncated to half its bytes passes the header checks and fails the load.
- The tile frame. `make_tile_frame` puts the frame origin at the tile centre (tile units 0.5,
  0.5, so DEM pixel coordinates 256, 256) at `origin_height_m`
  (`overlume/src/mapbox_tile_frame.cpp:78-90`). `project_to_frame` (`:92-99`) runs the exact chain:
  inverse spherical Mercator (`tile_point_to_lonlat`, `:46-50`), WGS84 ECEF at
  `origin_height_m + z_m`, then the frame. So the skirt's "down" is `z_m - skirt_m` along the
  ellipsoid normal (its offset from its edge vertex is exactly `skirt_m` long), and u = 512 of a
  tile and u = 0 of its east neighbour are the same point.
- Test build: `overlume/CMakeLists.txt:585-590` makes each `tests/*.cpp` an executable and drops
  `test_mapbox_*` when cesium is off; `:614-621` compile it as C++17 (`-std=gnu++17`) with `src`
  and `_deps/stb` on the include path and golden.cpp beside it, linking
  `overlume gtest gtest_main`; `:629-635` add meshoptimizer.h alone for `test_mapbox_*`. So the
  test includes no cesium header: it calls `make_tile_frame`/`project_to_frame` through the C++17
  `mapbox_tile_frame.hpp` and writes PNGs with `stb_image_write.h` (`stbi_write_png_to_func`,
  `stbi_write_jpg_to_func`, implemented in golden.cpp, as `overlume/tests/mapbox_glb_fixture.hpp:7-12`
  does).
- Numbers the expectations use: the operating area's z14 tile is 14/10712/7012
  (`overlume/tests/test_mapbox_tile_frame.cpp:54-55`), about 2,217 m across, so 8 px is about
  34.6 m and a skirt vertex's nearest grid vertex is its own edge vertex (20 m away, every other one
  at least 34.6 m). Decoded codes: 0 → −10000 m; 140000 = (2, 34, 224) → 4000 m; 100000 =
  (1, 134, 160) → 0 m; 16777215 = (255, 255, 255) → 1667721.5 m; 100102 = (1, 135, 6) → 10.2 m.
- Agreement with spec §5 (as amended in 7ec7092) and the head contract, nothing left for the
  orchestrator's ledger: grid vertices sit at u, v = 0, 8, …, 512 (pixel coordinates, so
  neighbours share their edge vertices; Tasks 9, 10 and 12 read this header); the PNG is decoded
  with stb_image, a static PNG-only copy inside the unit, since the unit takes no cesium header;
  `write_dem_gltf` is not in this unit: it lives in the terrain loader (Task 10).
- Verified before writing this task, in a scratch build against this worktree's
  `overlume_stream` and test flags (clang 18.1.8, libc++, the built `liboverlume.a`): the code
  below compiles warning-free under `-std=c++17 -Wall -Wextra -Wpedantic -Wshadow -Wconversion`,
  is clang-format 23.1.1's output, and passes 11/11; and each of these mutants of
  `mapbox_dem.cpp` fails at least one test: skirt raised instead of lowered, the 16-bit check
  removed, nearest-pixel sampling, grid spacing 504/64, grid wound clockwise, normals flipped,
  v UVs flipped, `z_ref_m` ignored, decode scale 0.01, a skirt ring short by one vertex,
  `kDemSkirtM` changed to 2.0 (fails only `SkirtHangsSkirtMetresBelowEveryEdgeVertex`).

- [ ] **Step 1: Write the contract header** `overlume/src/mapbox_dem.hpp`

The contract's declarations, with two comment changes and nothing else: `DemMesh`'s member
comments sit above the members (clang-format would otherwise wrap them mid-declaration), and
`mesh_dem`'s comment, the contract's own as clang-format wraps it, adds that the mesh is empty
when `!dem.ok`.

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include "mapbox_tile_frame.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace overlume::mapbox {

inline constexpr uint32_t kDemSize = 512;      // pixels per side of a terrain-DEM tile
inline constexpr uint32_t kDemGridCells = 64;  // mesh cells per side (one vertex every 8 pixels)
inline constexpr double kDemSkirtM = 20.0;     // skirt depth below the edge

struct DemTile {
    bool ok = false;
    std::string error;            // one line when !ok
    std::vector<float> height_m;  // kDemSize * kDemSize, row-major, row 0 at the tile's north edge
    float at(uint32_t px, uint32_t py) const {  // px east, py south, both < kDemSize
        return height_m[static_cast<size_t>(py) * kDemSize + px];
    }
};

// Decodes a terrain-DEM PNG (stb_image, 8-bit, 3 or 4 channels, exactly 512 x 512) with
// height = -10000 + (R * 65536 + G * 256 + B) * 0.1. Any other size, depth or decode failure:
// ok = false.
DemTile decode_dem_png(const uint8_t* bytes, size_t size);

// Bilinear height at pixel-centre coordinates (u east, v south, 0..512, clamped to the edge).
double dem_sample(const DemTile& dem, double u, double v);

struct DemMesh {
    // Tile frame metres, grid vertices first (row-major from the north-west corner), then the
    // skirt ring.
    std::vector<std::array<float, 3>> positions;
    // Unit, from central differences; a skirt vertex copies its edge vertex's.
    std::vector<std::array<float, 3>> normals;
    // u east 0..1, v south 0..1 across the tile.
    std::vector<std::array<float, 2>> uvs;
    // CCW seen from above, grid then skirt (the skirt faces out of the tile).
    std::vector<uint32_t> indices;
    uint32_t grid_vertex_count = 0;  // (kDemGridCells + 1)^2
};

// Heights become z = height - z_ref_m; vertices sit at u, v = 0, 8, ..., 512 (pixel coordinates),
// so neighbouring tiles share their edge vertices and heights are bilinear samples at those points;
// positions through project_to_frame(frame, t14, u, v, kDemSize, z). Empty when !dem.ok.
DemMesh mesh_dem(const DemTile& dem, const TileFrame& frame, const TileId& t14, double z_ref_m,
                 double skirt_m = kDemSkirtM);

}
```

- [ ] **Step 2: Write the failing test** `overlume/tests/test_mapbox_dem.cpp`

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "mapbox_dem.hpp"
#include "mapbox_tile_frame.hpp"

#include "stb_image_write.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace {

namespace mb = overlume::mapbox;

constexpr uint32_t kSide = mb::kDemGridCells + 1;           // 65 grid vertices per side
constexpr uint32_t kGridVertices = kSide * kSide;           // 4225
constexpr uint32_t kSkirtVertices = 4 * mb::kDemGridCells;  // 256: each border vertex once
// The operating area's z14 tile (test_mapbox_tile_frame.cpp), in a frame 12 m up so no height
// hides behind a zero.
const mb::TileId kTile{14, 10712, 7012};
constexpr double kOriginHeightM = 12.0;

// PNG bytes from raw 8-bit pixels (stb_image_write; golden.cpp holds its implementation).
std::vector<uint8_t> png(int w, int h, int channels, const std::vector<uint8_t>& pixels) {
    std::vector<uint8_t> out;
    stbi_write_png_to_func(
        [](void* context, void* data, int size) {
            auto* bytes = static_cast<std::vector<uint8_t>*>(context);
            bytes->insert(bytes->end(), static_cast<uint8_t*>(data),
                          static_cast<uint8_t*>(data) + size);
        },
        &out, w, h, channels, pixels.data(), w * channels);
    return out;
}

mb::DemTile decode(const std::vector<uint8_t>& bytes) {
    return mb::decode_dem_png(bytes.data(), bytes.size());
}

// A decoded tile whose pixel (px, py) holds f(px, py), without a PNG in between.
mb::DemTile tile_of(const std::function<float(uint32_t px, uint32_t py)>& f) {
    mb::DemTile t;
    t.ok = true;
    t.height_m.resize(size_t{mb::kDemSize} * mb::kDemSize);
    for (uint32_t py = 0; py < mb::kDemSize; ++py) {
        for (uint32_t px = 0; px < mb::kDemSize; ++px) {
            t.height_m[size_t{py} * mb::kDemSize + px] = f(px, py);
        }
    }
    return t;
}

// 0 m at the west edge, 20 m at the east edge.
mb::DemTile ramp() {
    return tile_of([](uint32_t px, uint32_t) { return 20.0f * (px + 0.5f) / mb::kDemSize; });
}

// 40 m at the tile centre, falling off over 100 px.
mb::DemTile hill() {
    return tile_of([](uint32_t px, uint32_t py) {
        const double du = px + 0.5 - 256.0, dv = py + 0.5 - 256.0;
        return static_cast<float>(40.0 * std::exp(-(du * du + dv * dv) / (2.0 * 100.0 * 100.0)));
    });
}

std::array<double, 3> sub(const std::array<float, 3>& a, const std::array<float, 3>& b) {
    return {double{a[0]} - b[0], double{a[1]} - b[1], double{a[2]} - b[2]};
}

std::array<double, 3> cross(const std::array<double, 3>& a, const std::array<double, 3>& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

double norm(const std::array<double, 3>& a) {
    return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}

// A tile-frame point in ECEF, through the frame's column-major frame_to_ecef.
std::array<double, 3> to_ecef(const mb::TileFrame& f, const std::array<float, 3>& p) {
    std::array<double, 3> out{};
    for (size_t r = 0; r < 3; ++r) {
        out[r] = f.frame_to_ecef[r] * p[0] + f.frame_to_ecef[4 + r] * p[1] +
                 f.frame_to_ecef[8 + r] * p[2] + f.frame_to_ecef[12 + r];
    }
    return out;
}

bool on_border(uint32_t g) {
    const uint32_t i = g % kSide, j = g / kSide;
    return i == 0 || j == 0 || i == kSide - 1 || j == kSide - 1;
}

}

TEST(MapboxDem, KnownPixelsDecodeToHeights) {
    for (const int channels : {3, 4}) {
        std::vector<uint8_t> pixels(size_t{mb::kDemSize} * mb::kDemSize * channels, 0);
        const auto set = [&](uint32_t px, uint32_t py, uint8_t r, uint8_t g, uint8_t b) {
            uint8_t* p = &pixels[(size_t{py} * mb::kDemSize + px) * channels];
            p[0] = r;
            p[1] = g;
            p[2] = b;
            if (channels == 4) p[3] = 7;  // alpha plays no part
        };
        set(511, 0, 2, 34, 224);       // 140000 -> 4000 m
        set(0, 511, 1, 134, 160);      // 100000 -> sea level
        set(511, 511, 255, 255, 255);  // 16777215 -> the code's ceiling
        set(100, 200, 1, 135, 6);      // 100102 -> 10.2 m
        const mb::DemTile t = decode(png(512, 512, channels, pixels));
        ASSERT_TRUE(t.ok) << t.error;
        EXPECT_TRUE(t.error.empty());
        ASSERT_EQ(t.height_m.size(), size_t{512} * 512);
        EXPECT_FLOAT_EQ(t.at(0, 0), -10000.0f) << "R = G = B = 0 is -10000 m";
        EXPECT_FLOAT_EQ(t.at(511, 0), 4000.0f) << "row 0 is the north edge, px 511 the east";
        EXPECT_FLOAT_EQ(t.at(0, 511), 0.0f);
        EXPECT_FLOAT_EQ(t.at(511, 511), 1667721.5f);
        EXPECT_NEAR(t.at(100, 200), 10.2f, 1e-3f);
        EXPECT_FLOAT_EQ(t.at(101, 200), -10000.0f);
    }
}

TEST(MapboxDem, MalformedTilesAreRejectedWithReasons) {
    const std::vector<uint8_t> rgb(size_t{512} * 512 * 3, 100);
    const std::vector<uint8_t> valid = png(512, 512, 3, rgb);
    std::vector<uint8_t> deep = valid;
    deep[24] = 16;  // IHDR bit depth (8 signature + 8 chunk header + 8 width/height): 16-bit RGB
    std::vector<uint8_t> jpeg;
    stbi_write_jpg_to_func(
        [](void* context, void* data, int size) {
            auto* bytes = static_cast<std::vector<uint8_t>*>(context);
            bytes->insert(bytes->end(), static_cast<uint8_t*>(data),
                          static_cast<uint8_t*>(data) + size);
        },
        &jpeg, 512, 512, 3, rgb.data(), 90);
    const std::string text = "<html>404 Not Found</html>";
    struct Case {
        const char* name;
        std::vector<uint8_t> bytes;
        const char* error;
    };
    const Case cases[] = {
        {"empty", {}, "DEM tile is empty"},
        {"text", {text.begin(), text.end()}, "DEM tile does not decode as PNG"},
        {"jpeg", jpeg, "DEM tile does not decode as PNG"},
        {"truncated",
         {valid.begin(), valid.begin() + static_cast<std::ptrdiff_t>(valid.size() / 2)},
         "DEM tile does not decode as PNG"},
        {"256 px", png(256, 256, 3, std::vector<uint8_t>(size_t{256} * 256 * 3, 100)),
         "DEM tile is not 512 x 512 pixels"},
        {"512 x 256", png(512, 256, 3, std::vector<uint8_t>(size_t{512} * 256 * 3, 100)),
         "DEM tile is not 512 x 512 pixels"},
        {"grey", png(512, 512, 1, std::vector<uint8_t>(size_t{512} * 512, 100)),
         "DEM tile has fewer than 3 channels"},
        {"grey + alpha", png(512, 512, 2, std::vector<uint8_t>(size_t{512} * 512 * 2, 100)),
         "DEM tile has fewer than 3 channels"},
        {"16-bit", deep, "DEM tile is not 8-bit"},
    };
    for (const Case& c : cases) {
        const mb::DemTile t = decode(c.bytes);
        EXPECT_FALSE(t.ok) << c.name;
        EXPECT_EQ(t.error, c.error) << c.name;
        EXPECT_TRUE(t.height_m.empty()) << c.name;
    }
    EXPECT_EQ(mb::decode_dem_png(nullptr, 10).error, "DEM tile is empty");
    EXPECT_TRUE(decode(valid).ok) << "the unmutated tile decodes";
}

TEST(MapboxDem, SamplingIsBilinearBetweenPixelCentres) {
    mb::DemTile cell = tile_of([](uint32_t, uint32_t) { return 0.0f; });
    cell.height_m[20 * 512 + 11] = 10.0f;  // (11, 20)
    cell.height_m[21 * 512 + 10] = 20.0f;  // (10, 21)
    cell.height_m[21 * 512 + 11] = 30.0f;  // (11, 21)
    EXPECT_DOUBLE_EQ(mb::dem_sample(cell, 10.5, 20.5), 0.0) << "a pixel centre is that pixel";
    EXPECT_DOUBLE_EQ(mb::dem_sample(cell, 11.5, 21.5), 30.0);
    EXPECT_DOUBLE_EQ(mb::dem_sample(cell, 10.75, 20.5), 2.5) << "a quarter of the way east";
    EXPECT_DOUBLE_EQ(mb::dem_sample(cell, 11.0, 21.0), 15.0) << "the mean of the four";
    EXPECT_DOUBLE_EQ(mb::dem_sample(cell, 10.5, 21.0), 10.0);

    const mb::DemTile plane =
        tile_of([](uint32_t px, uint32_t py) { return static_cast<float>(px + 1000.0 * py); });
    EXPECT_NEAR(mb::dem_sample(plane, 100.25, 300.75), 99.75 + 1000.0 * 300.25, 1e-6);
    EXPECT_NEAR(mb::dem_sample(plane, 0.0, 0.0), 0.0, 1e-9) << "clamped to the corner pixel";
    EXPECT_NEAR(mb::dem_sample(plane, 512.0, 512.0), 511.0 + 511000.0, 1e-9);
    EXPECT_NEAR(mb::dem_sample(plane, -40.0, 600.0), 511000.0, 1e-9);
    EXPECT_NEAR(mb::dem_sample(plane, 512.0, 256.0), 511.0 + 1000.0 * 255.5, 1e-6);
    EXPECT_EQ(mb::dem_sample(mb::DemTile{}, 256.0, 256.0), 0.0) << "an undecoded tile reads 0";
}

TEST(MapboxDem, MeshHasTheGridThenTheSkirtRing) {
    const mb::DemMesh m =
        mb::mesh_dem(hill(), mb::make_tile_frame(kTile, kOriginHeightM), kTile, 0.0);
    EXPECT_EQ(m.grid_vertex_count, kGridVertices);
    EXPECT_EQ(m.positions.size(), size_t{kGridVertices + kSkirtVertices});  // 4481
    EXPECT_EQ(m.normals.size(), m.positions.size());
    EXPECT_EQ(m.uvs.size(), m.positions.size());
    // 64 x 64 cells of two triangles, then two triangles per border segment.
    EXPECT_EQ(m.indices.size(), size_t{64 * 64 * 6 + kSkirtVertices * 6});  // 26112
    EXPECT_LT(*std::max_element(m.indices.begin(), m.indices.end()), m.positions.size());
}

TEST(MapboxDem, GridVerticesFollowTheDemThroughTheTileFrame) {
    const mb::TileFrame f = mb::make_tile_frame(kTile, kOriginHeightM);
    const mb::DemTile dem = hill();
    constexpr double kZRef = 15.0;
    const mb::DemMesh m = mb::mesh_dem(dem, f, kTile, kZRef);
    for (const auto [i, j] :
         {std::array<uint32_t, 2>{0, 0}, {64, 0}, {0, 64}, {64, 64}, {32, 32}, {1, 0}, {17, 45}}) {
        const double u = 8.0 * i, v = 8.0 * j;
        const std::array<double, 3> want =
            mb::project_to_frame(f, kTile, u, v, mb::kDemSize, mb::dem_sample(dem, u, v) - kZRef);
        const std::array<float, 3>& got = m.positions[j * kSide + i];
        for (size_t c = 0; c < 3; ++c) {
            EXPECT_NEAR(got[c], want[c], 2e-3) << "vertex (" << i << ", " << j << ") axis " << c;
        }
    }
    // The tile centre is the frame origin: z there is the height minus z_ref.
    const mb::DemTile flat = tile_of([](uint32_t, uint32_t) { return 100.0f; });
    const mb::DemMesh at_ref = mb::mesh_dem(flat, f, kTile, 100.0);
    const mb::DemMesh at_zero = mb::mesh_dem(flat, f, kTile, 0.0);
    const uint32_t centre = 32 * kSide + 32;
    for (size_t c = 0; c < 3; ++c) EXPECT_NEAR(at_ref.positions[centre][c], 0.0, 1e-3);
    EXPECT_NEAR(at_zero.positions[centre][2], 100.0, 1e-3);
    EXPECT_NEAR(at_zero.positions[centre][0], 0.0, 1e-3);
}

TEST(MapboxDem, EdgeVerticesSitOnTheTileBoundary) {
    const mb::DemTile sea = tile_of([](uint32_t, uint32_t) { return 0.0f; });
    const mb::TileId east{14, kTile.x + 1, kTile.y}, south{14, kTile.x, kTile.y + 1};
    const mb::TileFrame f = mb::make_tile_frame(kTile, kOriginHeightM);
    const mb::TileFrame fe = mb::make_tile_frame(east, kOriginHeightM);
    const mb::TileFrame fs = mb::make_tile_frame(south, kOriginHeightM);
    const mb::DemMesh m = mb::mesh_dem(sea, f, kTile, 0.0);
    const mb::DemMesh me = mb::mesh_dem(sea, fe, east, 0.0);
    const mb::DemMesh ms = mb::mesh_dem(sea, fs, south, 0.0);
    double worst = 0.0;
    for (uint32_t k = 0; k < kSide; ++k) {
        const std::array<double, 3> a = to_ecef(f, m.positions[k * kSide + 64]);   // our east edge
        const std::array<double, 3> b = to_ecef(fe, me.positions[k * kSide + 0]);  // its west edge
        const std::array<double, 3> c = to_ecef(f, m.positions[64 * kSide + k]);   // our south edge
        const std::array<double, 3> d = to_ecef(fs, ms.positions[0 * kSide + k]);  // its north edge
        worst = std::max({worst, norm({a[0] - b[0], a[1] - b[1], a[2] - b[2]}),
                          norm({c[0] - d[0], c[1] - d[1], c[2] - d[2]})});
    }
    EXPECT_LT(worst, 0.01) << "neighbouring tiles share their edge vertices (metres apart)";
}

TEST(MapboxDem, SkirtHangsSkirtMetresBelowEveryEdgeVertex) {
    const mb::TileFrame f = mb::make_tile_frame(kTile, kOriginHeightM);
    // The default argument is pinned to 20 m (Review Focus 1); 5 m proves the argument is used.
    const std::pair<mb::DemMesh, double> cases[] = {
        {mb::mesh_dem(hill(), f, kTile, 0.0), 20.0},
        {mb::mesh_dem(hill(), f, kTile, 0.0, 5.0), 5.0}};
    for (const auto& [m, skirt] : cases) {
        std::vector<int> hits(kGridVertices, 0);
        for (uint32_t s = kGridVertices; s < m.positions.size(); ++s) {
            uint32_t nearest = 0;
            double best = 1e300;
            for (uint32_t g = 0; g < kGridVertices; ++g) {
                const double d = norm(sub(m.positions[s], m.positions[g]));
                if (d < best) {
                    best = d;
                    nearest = g;
                }
            }
            ++hits[nearest];
            EXPECT_TRUE(on_border(nearest)) << "skirt vertex " << s;
            EXPECT_NEAR(best, skirt, 0.01) << "skirt vertex " << s;
            EXPECT_NEAR(m.positions[nearest][2] - m.positions[s][2], skirt, 0.01)
                << "skirt vertex " << s << " hangs below its edge vertex";
            EXPECT_EQ(m.normals[s], m.normals[nearest]) << "skirt vertex " << s;
            EXPECT_EQ(m.uvs[s], m.uvs[nearest]) << "skirt vertex " << s;
        }
        for (uint32_t g = 0; g < kGridVertices; ++g) {
            EXPECT_EQ(hits[g], on_border(g) ? 1 : 0) << "grid vertex " << g << " skirt " << skirt;
        }
    }
}

TEST(MapboxDem, NormalsAreUnitAndFollowTheSlope) {
    const mb::TileFrame f = mb::make_tile_frame(kTile, kOriginHeightM);
    for (const mb::DemTile& dem : {hill(), ramp()}) {
        const mb::DemMesh m = mb::mesh_dem(dem, f, kTile, 0.0);
        for (size_t k = 0; k < m.normals.size(); ++k) {
            const std::array<float, 3>& n = m.normals[k];
            ASSERT_NEAR(norm({n[0], n[1], n[2]}), 1.0, 1e-5) << "normal " << k;
            ASSERT_GT(n[2], 0.99) << "normal " << k << " points up";
        }
    }
    // The ramp rises 20 m over the tile's width: at the centre the normal leans west by that slope.
    const mb::DemMesh m = mb::mesh_dem(ramp(), f, kTile, 0.0);
    const std::array<double, 3> w = mb::project_to_frame(f, kTile, 0.0, 256.0, mb::kDemSize, 0.0);
    const std::array<double, 3> e = mb::project_to_frame(f, kTile, 512.0, 256.0, mb::kDemSize, 0.0);
    const double slope = 20.0 / (e[0] - w[0]);
    const std::array<float, 3>& n = m.normals[32 * kSide + 32];
    EXPECT_NEAR(n[0], -slope / std::sqrt(1.0 + slope * slope), 2e-5);
    EXPECT_NEAR(n[1], 0.0, 2e-5);
    // On the hill's east flank the ground falls away eastward, so the normal leans east.
    EXPECT_GT(mb::mesh_dem(hill(), f, kTile, 0.0).normals[32 * kSide + 40][0], 0.01);
}

TEST(MapboxDem, TrianglesFaceUpAndTheSkirtFacesOutward) {
    const mb::DemMesh m =
        mb::mesh_dem(hill(), mb::make_tile_frame(kTile, kOriginHeightM), kTile, 0.0);
    const size_t grid_indices = size_t{64} * 64 * 6;
    for (size_t t = 0; t < m.indices.size(); t += 3) {
        const std::array<float, 3>& a = m.positions[m.indices[t]];
        const std::array<float, 3>& b = m.positions[m.indices[t + 1]];
        const std::array<float, 3>& c = m.positions[m.indices[t + 2]];
        const std::array<double, 3> n = cross(sub(b, a), sub(c, a));
        if (t < grid_indices) {
            ASSERT_GT(n[2], 0.0) << "grid triangle " << t / 3 << " is counter-clockwise from above";
        } else {
            // The frame origin is the tile centre, so outward is away from (0, 0).
            const double cx = (a[0] + b[0] + c[0]) / 3.0, cy = (a[1] + b[1] + c[1]) / 3.0;
            ASSERT_GT(n[0] * cx + n[1] * cy, 0.0) << "skirt triangle " << t / 3 << " faces out";
        }
    }
}

TEST(MapboxDem, UvsSpanTheTileEastAndSouth) {
    const mb::DemMesh m =
        mb::mesh_dem(hill(), mb::make_tile_frame(kTile, kOriginHeightM), kTile, 0.0);
    for (const std::array<float, 2>& uv : m.uvs) {
        ASSERT_GE(uv[0], 0.0f);
        ASSERT_LE(uv[0], 1.0f);
        ASSERT_GE(uv[1], 0.0f);
        ASSERT_LE(uv[1], 1.0f);
    }
    EXPECT_EQ(m.uvs[0], (std::array<float, 2>{0.0f, 0.0f})) << "north-west corner";
    EXPECT_EQ(m.uvs[64], (std::array<float, 2>{1.0f, 0.0f})) << "north-east corner";
    EXPECT_EQ(m.uvs[64 * kSide], (std::array<float, 2>{0.0f, 1.0f})) << "south-west corner";
    EXPECT_EQ(m.uvs[17 * kSide + 9], (std::array<float, 2>{9.0f / 64, 17.0f / 64}));
}

TEST(MapboxDem, AnUndecodedTileMeshesToNothing) {
    const mb::DemMesh m = mb::mesh_dem(mb::DemTile{}, mb::make_tile_frame(kTile, 0.0), kTile, 0.0);
    EXPECT_EQ(m.grid_vertex_count, 0u);
    EXPECT_TRUE(m.positions.empty());
    EXPECT_TRUE(m.normals.empty());
    EXPECT_TRUE(m.uvs.empty());
    EXPECT_TRUE(m.indices.empty());
}
```

What each test catches:
- `KnownPixelsDecodeToHeights`: a wrong scale, offset or channel order in the formula, rows read
  bottom-up (row 0 is north), alpha taken as a channel (RGB and RGBA both run).
- `MalformedTilesAreRejectedWithReasons`: any of the five checks removed or reordered (each case
  must fail with its own reason; without the depth check the mutated PNG fails later as "does
  not decode", so the reason catches it), a truncated or non-PNG body accepted, partial heights
  left in a failed tile.
- `SamplingIsBilinearBetweenPixelCentres`: nearest-pixel sampling, a half-pixel shift (pixel
  centres at px + 0.5), missing edge clamps (reads outside the 512 × 512 buffer), a crash on an
  undecoded tile.
- `MeshHasTheGridThenTheSkirtRing`: any change to the 65 × 65 grid, the 256-vertex ring or the
  index layout.
- `GridVerticesFollowTheDemThroughTheTileFrame`: `z_ref_m` ignored or added, vertices not at
  u, v = 8i, 8j, positions not through `project_to_frame`.
- `EdgeVerticesSitOnTheTileBoundary`: an edge vertex inside the tile (spacing 504/64 or pixel
  centres at 511.5): neighbouring tiles would leave a gap (Review Focus 1).
- `SkirtHangsSkirtMetresBelowEveryEdgeVertex`: the skirt raised, of the wrong depth (the default
  is pinned to a literal 20 m, the explicit 5 m to its argument), missing or duplicating a border
  vertex, or not copying its edge vertex's normal and UV (Review Focus 1).
- `NormalsAreUnitAndFollowTheSlope`: normals not unit length, flipped, or not following the slope
  (the ramp's centre normal leans west by its 20 m rise over the tile's width).
- `TrianglesFaceUpAndTheSkirtFacesOutward`: the grid wound clockwise from above (back-face
  culling would hide the terrain), the skirt facing into the tile (culling would hide it from
  the neighbour's side, where the crack is seen).
- `UvsSpanTheTileEastAndSouth`: v flipped (north at 1) or UVs not spanning 0..1.
- `AnUndecodedTileMeshesToNothing`: a failed tile meshed as a flat plane.

- [ ] **Step 3: Run it and see it fail**

The new test file needs a configure to become a target; the configure holds the gate lock
(Execution, "Later configures").

```bash
cd "$WORKTREE"
flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock cmake -S overlume -B overlume/build > /dev/null \
    && echo reconfigured \
    && CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
        --target test_mapbox_dem
```

No `reconfigured` line and no CMake error above it means the lock wait ran out (exit 75) and nothing
was built: run the step again.

Expected: `reconfigured`, then the test compiles and its link fails with (each symbol reported
from several call sites)

```
undefined reference to `overlume::mapbox::decode_dem_png(unsigned char const*, unsigned long)'
undefined reference to `overlume::mapbox::dem_sample(overlume::mapbox::DemTile const&, double, double)'
undefined reference to `overlume::mapbox::mesh_dem(overlume::mapbox::DemTile const&, overlume::mapbox::TileFrame const&, overlume::mapbox::TileId const&, double, double)'
clang++: error: linker command failed with exit code 1 (use -v to see invocation)
```

and the build exits non-zero. A compile error instead means Step 1 or 2 was mistyped.

- [ ] **Step 4: Implement** `overlume/src/mapbox_dem.cpp`

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "mapbox_dem.hpp"

#include <algorithm>
#include <climits>
#include <cmath>

// stb_image from the vcpkg prefix cesium-native builds against (on overlume_stream's include
// path). Static and PNG-only, as cesium's own CesiumImage/src/ImageDecoder.cpp and the node's
// hud_overlay.cpp compile it, so these symbols never meet another stb_image at link time.
#define STB_IMAGE_STATIC
#define STBI_ONLY_PNG
#define STBI_NO_STDIO
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

namespace overlume::mapbox {

namespace {

constexpr size_t kDemPixels = size_t{kDemSize} * kDemSize;

DemTile failed(const char* reason) {
    DemTile out;
    out.error = reason;
    return out;
}

}

DemTile decode_dem_png(const uint8_t* bytes, size_t size) {
    if (bytes == nullptr || size == 0) return failed("DEM tile is empty");
    int w = 0, h = 0, channels = 0;
    if (size > static_cast<size_t>(INT_MAX) ||
        stbi_info_from_memory(bytes, static_cast<int>(size), &w, &h, &channels) == 0) {
        return failed("DEM tile does not decode as PNG");
    }
    const int len = static_cast<int>(size);
    if (w != static_cast<int>(kDemSize) || h != static_cast<int>(kDemSize)) {
        return failed("DEM tile is not 512 x 512 pixels");
    }
    if (stbi_is_16_bit_from_memory(bytes, len) != 0) return failed("DEM tile is not 8-bit");
    if (channels < 3) return failed("DEM tile has fewer than 3 channels");
    stbi_uc* rgb = stbi_load_from_memory(bytes, len, &w, &h, &channels, 3);
    if (rgb == nullptr) return failed("DEM tile does not decode as PNG");
    DemTile out;
    out.height_m.resize(kDemPixels);
    for (size_t i = 0; i < kDemPixels; ++i) {
        const stbi_uc* p = rgb + 3 * i;
        out.height_m[i] =
            static_cast<float>(-10000.0 + (p[0] * 65536.0 + p[1] * 256.0 + p[2]) * 0.1);
    }
    stbi_image_free(rgb);
    out.ok = true;
    return out;
}

double dem_sample(const DemTile& dem, double u, double v) {
    if (dem.height_m.size() != kDemPixels) return 0.0;
    // Pixel (px, py) holds the height at (px + 0.5, py + 0.5); fmin/fmax also map NaN to the edge.
    const double x = std::fmin(std::fmax(u - 0.5, 0.0), kDemSize - 1.0);
    const double y = std::fmin(std::fmax(v - 0.5, 0.0), kDemSize - 1.0);
    const uint32_t x0 = std::min(static_cast<uint32_t>(x), kDemSize - 2);
    const uint32_t y0 = std::min(static_cast<uint32_t>(y), kDemSize - 2);
    const double fx = x - x0, fy = y - y0;
    const double north = dem.at(x0, y0) * (1.0 - fx) + dem.at(x0 + 1, y0) * fx;
    const double south = dem.at(x0, y0 + 1) * (1.0 - fx) + dem.at(x0 + 1, y0 + 1) * fx;
    return north * (1.0 - fy) + south * fy;
}

DemMesh mesh_dem(const DemTile& dem, const TileFrame& frame, const TileId& t14, double z_ref_m,
                 double skirt_m) {
    DemMesh m;
    if (!dem.ok || dem.height_m.size() != kDemPixels) return m;
    constexpr uint32_t n = kDemGridCells + 1;  // vertices per side
    constexpr double step = static_cast<double>(kDemSize) / kDemGridCells;
    const auto place = [&](uint32_t i, uint32_t j, double down_m) {
        const double u = i * step, v = j * step;
        return project_to_frame(frame, t14, u, v, kDemSize,
                                dem_sample(dem, u, v) - z_ref_m - down_m);
    };
    const size_t vertices = size_t{n} * n + 4 * kDemGridCells;  // the grid, then the skirt ring
    m.positions.reserve(vertices);
    m.normals.reserve(vertices);  // so the skirt's copies below never reallocate under a reference
    m.uvs.reserve(vertices);
    std::vector<std::array<double, 3>> grid;
    grid.reserve(size_t{n} * n);
    for (uint32_t j = 0; j < n; ++j) {
        for (uint32_t i = 0; i < n; ++i) grid.push_back(place(i, j, 0.0));
    }
    const auto at = [&grid](uint32_t i, uint32_t j) { return grid[size_t{j} * n + i]; };
    for (uint32_t j = 0; j < n; ++j) {
        for (uint32_t i = 0; i < n; ++i) {
            const std::array<double, 3> p = at(i, j);
            m.positions.push_back(
                {static_cast<float>(p[0]), static_cast<float>(p[1]), static_cast<float>(p[2])});
            m.uvs.push_back(
                {static_cast<float>(i) / kDemGridCells, static_cast<float>(j) / kDemGridCells});
            // Central differences of the grid (one-sided on the border): east x north.
            const std::array<double, 3> e = at(std::min(i + 1, n - 1), j), w = at(i ? i - 1 : 0, j);
            const std::array<double, 3> no = at(i, j ? j - 1 : 0),
                                        s = at(i, std::min(j + 1, n - 1));
            const double ex = e[0] - w[0], ey = e[1] - w[1], ez = e[2] - w[2];
            const double nx = no[0] - s[0], ny = no[1] - s[1], nz = no[2] - s[2];
            const double cx = ey * nz - ez * ny, cy = ez * nx - ex * nz, cz = ex * ny - ey * nx;
            const double len = std::sqrt(cx * cx + cy * cy + cz * cz);
            m.normals.push_back({static_cast<float>(cx / len), static_cast<float>(cy / len),
                                 static_cast<float>(cz / len)});
        }
    }
    m.grid_vertex_count = n * n;
    for (uint32_t j = 0; j < kDemGridCells; ++j) {
        for (uint32_t i = 0; i < kDemGridCells; ++i) {
            const uint32_t nw = j * n + i, ne = nw + 1, sw = nw + n, se = sw + 1;
            m.indices.insert(m.indices.end(), {nw, sw, se, nw, se, ne});
        }
    }
    // The border, counter-clockwise seen from above: down the west edge, east along the south,
    // up the east, west along the north. Each border vertex once.
    std::vector<uint32_t> ring;
    for (uint32_t j = 0; j < kDemGridCells; ++j) ring.push_back(j * n);
    for (uint32_t i = 0; i < kDemGridCells; ++i) ring.push_back(kDemGridCells * n + i);
    for (uint32_t j = kDemGridCells; j > 0; --j) ring.push_back(j * n + kDemGridCells);
    for (uint32_t i = kDemGridCells; i > 0; --i) ring.push_back(i);
    const auto base = static_cast<uint32_t>(m.positions.size());
    const auto count = static_cast<uint32_t>(ring.size());
    for (const uint32_t r : ring) {
        const std::array<double, 3> p = place(r % n, r / n, skirt_m);
        m.positions.push_back(
            {static_cast<float>(p[0]), static_cast<float>(p[1]), static_cast<float>(p[2])});
        m.normals.push_back(m.normals[r]);
        m.uvs.push_back(m.uvs[r]);
    }
    // One quad per border segment, wound to face out of the tile.
    for (uint32_t k = 0; k < count; ++k) {
        const uint32_t e0 = ring[k], e1 = ring[(k + 1) % count];
        const uint32_t s0 = base + k, s1 = base + (k + 1) % count;
        m.indices.insert(m.indices.end(), {e0, s0, s1, e0, s1, e1});
    }
    return m;
}

}
```

- [ ] **Step 5: Run it and see it pass**

The configure picks `mapbox_dem.cpp` up through the `src/mapbox_*.cpp` glob; it holds the gate
lock, as in Step 3.

```bash
cd "$WORKTREE"
flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock cmake -S overlume -B overlume/build > /dev/null \
    && echo reconfigured \
    && CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
        --target test_mapbox_dem \
    && overlume/build/test_mapbox_dem
```

No `reconfigured` line and no CMake error above it means the lock wait ran out (exit 75) and nothing
was built: run the step again.

Expected: `reconfigured`, a clean build, and a run that ends

```
[==========] 11 tests from 1 test suite ran. (… ms total)
[  PASSED  ] 11 tests.
```

- [ ] **Step 6: Format**

```bash
cd "$WORKTREE"
git add overlume/src/mapbox_dem.hpp overlume/src/mapbox_dem.cpp overlume/tests/test_mapbox_dem.cpp
tools/check_format.sh --fix overlume/src/mapbox_dem.hpp overlume/src/mapbox_dem.cpp \
    overlume/tests/test_mapbox_dem.cpp
git diff --stat
git add overlume/src/mapbox_dem.hpp overlume/src/mapbox_dem.cpp overlume/tests/test_mapbox_dem.cpp
tools/check_format.sh && tools/check_spdx.sh
```

Expected: `check_format.sh: reformatted 3 files with clang-format 23.1.1`; `git diff --stat` prints
nothing (the code above is already clang-format 23.1.1's output with the repository's
`.clang-format`, so a diff means a line was typed differently, and the second `git add` stages the
fix); then `check_format.sh: PASS (218 files, clang-format 23.1.1)` and
`check_spdx.sh: PASS (376 files carry the SPDX identifier)` (215 and 373 at 427f5d8, which Task 0
leaves unchanged, plus these three; if the fork's counts differ, each expected count is the fork's
plus three).

- [ ] **Step 7: Revert check (do not commit)**: the skirt must hang below the edge and face out

```bash
cd "$WORKTREE"
sed -i 's/dem_sample(dem, u, v) - z_ref_m - down_m/dem_sample(dem, u, v) - z_ref_m + down_m/' \
    overlume/src/mapbox_dem.cpp
git diff --stat overlume/src/mapbox_dem.cpp
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 --target test_mapbox_dem
overlume/build/test_mapbox_dem | tail -5
git checkout -- overlume/src/mapbox_dem.cpp && git diff --stat overlume/src/mapbox_dem.cpp
```

Expected: the first diff reports `1 file changed, 1 insertion(+), 1 deletion(-)`; the run ends

```
[  FAILED  ] 2 tests, listed below:
[  FAILED  ] MapboxDem.SkirtHangsSkirtMetresBelowEveryEdgeVertex
[  FAILED  ] MapboxDem.TrianglesFaceUpAndTheSkirtFacesOutward

 2 FAILED TESTS
```

because the raised skirt stands 20 m above its edge (`... evaluates to -20`) and its quads face
into the tile. After the checkout (from the index Step 6 staged) the last diff prints nothing.

- [ ] **Step 8: Gate** (foreground, Bash timeout 600000 ms)

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
ROS_DOMAIN_ID=81 CI_VISUAL_MODE_DOMAIN_ID=81 \
    flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock tools/ci_visual_mode.sh
```

Expected (the baseline, which Task 0 leaves unchanged, plus the 11 `MapboxDem.*` tests):
- `PASS  POD header check`
- `PASS  library ctest suite  (100% tests passed, 0 tests failed out of 453 (435 ok / 8 skipped))`
- `PASS  node gtests  (Summary: 381 tests, 0 errors, 0 failures, 0 skipped)`
- `PASS  WS bridge pytest suite  (94 passed in …)`, the time varying
- `PASS  golden suite: 25 ok, 0 skipped`
- `PASS  examples  (6 run, all exited 0 with non-empty output)`
- `OVERALL: PASS`, exit 0.

Exit 75 means another lane held the lock for 300 s: run the same command again. Any other non-zero
exit is a gate failure. A WS bridge failure can also come from another session's gate on the same
ports, so rerun before treating it as a finding. The prebuild also rebuilds `liboverlume.a` and
`test_mapbox_dem` from the source Step 7 restored.

- [ ] **Step 9: Commit**

```bash
cd "$WORKTREE"
git add overlume/src/mapbox_dem.hpp overlume/src/mapbox_dem.cpp overlume/tests/test_mapbox_dem.cpp
git status --short
git commit -F - <<'EOF'
feat(mapbox): decode terrain-DEM tiles and mesh them in the tile frame

The terrain loader and the buildings' re-basing onto the ground need
heights from Mapbox terrain-DEM tiles. decode_dem_png reads the 512 px
terrain-RGB PNG with stb_image, compiled static and PNG-only from the
vcpkg prefix overlume_stream already sees (as cesium and the node do):
height = -10000 + (R * 65536 + G * 256 + B) * 0.1. Anything but an
8-bit, 3- or 4-channel 512 x 512 PNG is refused with one fixed reason
per kind, so the loader can log each kind once. dem_sample is bilinear
between pixel centres and clamped at the edges. mesh_dem builds a
65 x 65 grid (every 8 px from edge to edge, so neighbouring tiles share
their edge vertices) through project_to_frame with z relative to
z_ref_m, normals from central differences, UVs across the tile, and a
20 m skirt ring wound to face out of the tile, so a height step between
tiles never opens a crack.

MapboxDem pins known pixels (-10000 m, 4000 m, sea level, the code's
ceiling), each malformed kind with its reason, bilinear samples, the
counts (4481 vertices, 26112 indices), positions through the frame,
edges shared with the east and south neighbours, the skirt's depth,
unit normals that follow the slope, the winding, the UVs and the empty
mesh of an undecoded tile.

Claude-Session: <the executing session's URL>
EOF
```

Expected: `git status --short` lists exactly the three files, each as `A `; the commit succeeds.

### Task 2: Fixtures: DEM PNG, flat imagery, quantized-mesh

**Lane:** 1. **Depends on:** Task 1 (the fixture's DEM encoding is the inverse of
`decode_dem_png`'s formula; Tasks 9, 10, 11 and 12 consume these headers).

**Files:**
- Create: `overlume/tests/imagery_fixture.hpp` (header-only, C++17, `overlume::testing::imagery`)
- Create: `overlume/tests/mapbox_dem_fixture.hpp` (header-only, C++17, `overlume::testing::dem`)
- Create: `overlume/tests/quantized_mesh_fixture.hpp` (header-only, C++17, `overlume::testing::qmesh`)
- Test: `overlume/tests/test_terrain_fixtures.cpp` (suite `TerrainFixtures`, 8 tests, binary
  `test_terrain_fixtures`)

**Interfaces:**
- Consumes: `stb_image_write.h` (`stbi_write_png_to_func`) and, in the test, `stb_image.h`
  (`stbi_load_from_memory`, `stbi_image_free`), both implemented by golden.cpp in every test
  binary. No library symbol (API notes: the test also builds with cesium off).
- Produces, for Tasks 9, 10, 11 and 12 (verbatim from the contract):
  - `namespace overlume::testing::dem`:
    `std::vector<uint8_t> encode_dem_png(const std::function<double(double u, double v)>& height_m);`,
    `double ramp_height_m(double u, double v);`, `double hill_height_m(double u, double v);`,
    `double shelf_height_m(double u, double v);`, `double sea_height_m(double u, double v);`
  - `namespace overlume::testing::imagery`:
    `std::vector<uint8_t> flat_png(uint32_t size, uint8_t r, uint8_t g, uint8_t b);`
  - `namespace overlume::testing::qmesh`:
    `struct QuantizedMeshOptions { uint32_t grid = 8; double min_height_m = 0.0; double max_height_m = 0.0; };`,
    `std::vector<uint8_t> encode_quantized_mesh(uint32_t level, uint32_t x, uint32_t y, const QuantizedMeshOptions& o);`,
    `std::string layer_json(const std::vector<std::array<uint32_t, 3>>& available_level_x_y);`,
    `std::string ion_terrain_endpoint_json(const std::string& layer_json_base_url);`,
    `std::string ion_imagery_endpoint_json(const std::string& tms_base_url);`,
    `std::string tilemapresource_xml(uint32_t tile_size);`
  - All `inline`. Beyond the contract, additive only:
    `std::vector<uint8_t> imagery::png(int width, int height, int channels, const std::vector<uint8_t>& pixels);`
    (the PNG writer both image fixtures use; tests may hand-build tiles with it),
    `inline constexpr int dem::kDemSide = 512;`,
    `inline constexpr const char* qmesh::kFixtureAssetToken = "overlume-fixture-asset-token";`
    (a fixed fake bearer token, never a credential), `qmesh::kTerrainAttributionHtml`
    (`<span>Overlume fixture terrain</span>`), `qmesh::kImageryAttributionHtml`
    (`<b>Overlume fixture imagery</b>`; an HTML-stripped credit would read
    `Overlume fixture imagery`), `inline constexpr uint32_t qmesh::kTmsMaxLevel = 18;`.
- What consumers may rely on:
  - `encode_dem_png` decodes (through `decode_dem_png` or stb plus the formula) to within 0.05 m of
    the height function at every pixel centre. Scene values: ramp 0 m at u = 0, 20 m at u = 512;
    hill 40 m at (256, 256), 40·e^−0.4096 ≈ 26.5 m at the corners, steepest at the corners at
    40·362/400²·e^−0.4096 ≈ 0.060 m per pixel (about 1.4 % at z14's 4.33 m per pixel; 0.052 m per
    pixel at the edge midpoints); shelf 10 m for v < 256, 0 m below; sea 0 m. No named scene
    reaches 5 % (the ramp is 0.9 %, the shelf a step): a test that needs a steeper slope passes
    its own height function to `encode_dem_png`.
  - `encode_quantized_mesh`: tile (level, x, y) of the geographic TMS quadtree (2 × 1 root tiles,
    `180 / 2^level` degrees square, y = 0 at the south); a (grid + 1)² lattice with u, v =
    round(k · 32767 / grid); heights 0 (flat, min == max) or equal to u (a west-to-east ramp from
    min to max); counter-clockwise triangles from above; each edge lists its grid + 1 vertices;
    no extensions (cesium generates normals); empty for grid 0 or above 255. Size for grid 8:
    1438 bytes.
  - `layer_json`: projection `EPSG:4326`, format `quantized-mesh-1.0`, version `1.0.0`, tiles
    `{z}/{x}/{y}.terrain?v={version}` resolved against the layer.json URL, `maxzoom` the highest
    listed level, `available` one array per level 0..maxzoom with a one-tile range per listed
    tile (levels with none get `[]`). It lists exactly what it is given: cesium shows nothing
    unless both level-0 tiles, (0, 0, 0) and (0, 1, 0), are listed.
  - The ion bodies: `{"type":"TERRAIN","url":<base>,...}` and `{"type":"IMAGERY","url":<tms>,...}`
    with no `externalType`, which cesium opens as a Tile Map Service. Both base URLs must end with
    `/`: cesium resolves `layer.json` against the terrain URL, and adds the missing `/` to a TMS
    URL only when its path is at least 4 characters long (API notes), so a TMS base of
    `https://assets.fixture/2` would put the XML and the tiles at the host root. Both carry
    `kFixtureAssetToken` as
    `accessToken` (cesium sends it as `Authorization: Bearer …` on the asset requests) and one
    attribution. URLs must hold no `"` or `\` (no escaping).
  - `tilemapresource_xml`: SRS `EPSG:4326`, profile `geodetic`, PNG tiles of `tile_size`, levels
    0..18 at href `<level>`, so, with the `/`-terminated `<tms_base_url>` above, tile URLs are
    `<tms_base_url><level>/<x>/<y>.png` (y = 0 south). cesium first requests `<tms_base_url>`
    itself and, when that body does not parse as a tile map, `<tms_base_url>tilemapresource.xml`:
    the fixture network serves this XML at the latter (and may answer 404 at the former).

**API notes:**
- The quantized-mesh layout, as cesium-native 0.64 reads it
  (`overlume/build/_deps/cesium-native-src/CesiumQuantizedMeshTerrain/src/QuantizedMeshLoader.cpp`):
  the 88-byte header plus the `uint32` vertex count, `headerLength = 92` (`:137`); u, v, height
  `uint16` arrays, each zigzag-delta coded (`zigZagDecode`, `:142-144`; running sums, `:789-792`)
  and scaled by 1/32767, u lerping west to east and v south to north (`:794-799`); 16-bit
  indices while the vertex count is at most 65536 (`:219`), no padding before them; the index
  code is the high-water mark (`decodeIndices`, `:147-162`: `index = highest - code`, `highest`
  grows on a 0 code), so vertices must be numbered in order of first use; then the west, south,
  east and north edge index lists, each a `uint32` count and its indices (`:266-320`; cesium sorts
  them itself, `:505-615`); every edge needs at least one vertex, since the skirt index count is
  `(skirtVertexCount - 4) * 6` (`:752`). Only `BoundingSphereCenter` (as the vertices' RTC centre)
  and the minimum and maximum heights are used (`:763-772`); the horizon occlusion point is read
  and never used, hence the `ponytail:` value.
- The tiling: `LayerJsonTerrainLoader` maps projection `EPSG:4326` to a geographic quadtree with 2
  root tiles in x (`overlume/build/_deps/cesium-native-src/Cesium3DTilesSelection/src/LayerJsonTerrainLoader.cpp:477-482`),
  and `QuadtreeTilingScheme::tileToRectangle` puts y = 0 at the bottom (south)
  (`CesiumGeometry/src/QuadtreeTilingScheme.cpp:64-78`).
- layer.json fields cesium reads (`LayerJsonTerrainLoader.cpp`): `version` (`:313-318`), `tiles`
  (required, `:320-325`), `maxzoom` (default 30, `:327`), `extensions` (`:329-331`; none listed, so
  no `extensions=` query), `metadataAvailability` (absent, so availability comes from
  `available`, `:342-359`), `attribution` (`:362-368`), `projection` (default EPSG:4326,
  `:465-466`); the tile template's `{z}`, `{x}`, `{y}`, `{version}` placeholders (`:672-690`).
  `available` is an array per level from 0, each holding `{startX, startY, endX, endY}` objects
  (`QuantizedMeshLoader.cpp:1200-1237`).
- The ion endpoints: the terrain loader reads `type`, `url`, `accessToken`, `externalType` and
  `attributions[].html`/`collapsible` (`Cesium3DTilesSelection/src/CesiumIonTilesetLoader.cpp:242-285`)
  and for `TERRAIN` resolves `layer.json` against `url` (`:287-290`). The raster overlay requests
  `{ionAssetEndpointUrl}v1/assets/{id}/endpoint?access_token={token}`
  (`CesiumRasterOverlays/src/IonRasterOverlay.cpp:255`), requires `type` `IMAGERY` (`:448-458`),
  defaults `externalType` to `unknown` (`:463-466`) and then reads the body as a Tile Map Service
  (`url`, `accessToken`; `:383-388`, `:509`).
- The Tile Map Service (`CesiumRasterOverlays/src/TileMapServiceRasterOverlay.cpp`): the XML needs
  `TileSets` and an `SRS` containing 4326, 3857 or 900913 (`:237-258`), else cesium retries
  `<url>/tilemapresource.xml` (`:260-281`); `TileFormat` `extension`, `width`, `height` (`:319-326`);
  each `TileSet`'s `order` and `href` (`:334-347`); profile `geodetic` gives the geographic
  projection with 2 root tiles (`:385-394`); tile URLs resolve `<href>/<x>/<y>.<extension>`
  against the URL (`:143-150`). Both the XML retry (`:262-272`) and the tile base (`:466-476`)
  append a missing `/` only when the URL's path is at least 4 characters and does not end in
  `.xml`; otherwise the relative reference replaces the last path segment. Probed with
  `CesiumUtility::Uri`: base `https://assets.fixture/2` gives
  `https://assets.fixture/tilemapresource.xml` and `https://assets.fixture/18/5/3.png`; base
  `https://assets.fixture/2/` gives `https://assets.fixture/2/tilemapresource.xml` and
  `https://assets.fixture/2/18/5/3.png`. Hence the trailing-`/` rule.
- Why the test uses no library symbol: `test_terrain_fixtures.cpp` matches neither cesium-only
  filter (`overlume/CMakeLists.txt:587-590` drop only `test_environment_stream.cpp` and
  `test_mapbox_*`), and hosted CI configures with `-DOVERLUME_ENABLE_CESIUM=OFF` and builds every
  target (`.github/workflows/build.yml:33-40`), where `mapbox_dem.cpp` is not compiled at all
  (`overlume_stream` exists only with cesium, `overlume/CMakeLists.txt:261-270`). A call to
  `decode_dem_png` here would break that link, so the DEM PNGs are read back with stb_image and the
  published formula; `decode_dem_png` itself is pinned by `MapboxDem.KnownPixelsDecodeToHeights`.
  The tests' include path holds no JSON parser (`src`, `_deps/stb`, `include`, gtest), so the test
  carries a strict RFC 8259 syntax check and proves it rejects a missing and a trailing comma.
- Verified before writing this task, in scratch builds against this worktree's flags: the test
  passes 8/8 and its object references no `overlume` symbol; `decode_dem_png` reads every named
  scene back within 0.050 m; cesium-native's own `QuantizedMeshLoader::load` accepts
  `encode_quantized_mesh(15, 40000, 12000, {8, 10, 10})` and `{8, 10, 50}` with no error or warning
  (117 positions with its skirt, 576 indices), `loadAvailabilityRectangles` reads
  `layer_json({{0, 0, 0}, {0, 1, 0}, {2, 5, 3}})` as exactly those three tiles, and rapidjson parses
  both endpoint bodies (those probes are not part of this task: cesium loading the fixture end to
  end is Task 11's check); and each of these fixture mutants fails at least one test: the high-water
  mark never advanced, zigzag without its sign term, the south edge on row 1, the DEM code scaled
  ×100, no comma between availability ranges, clockwise triangles, hill sigma 100 px, `flat_png`
  writing blue into green.

- [ ] **Step 1: Write the failing test** `overlume/tests/test_terrain_fixtures.cpp`

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

// The terrain fixtures, pinned structurally. No library symbol is used: the cesium-OFF build
// (hosted CI) compiles and links every tests/test_*.cpp outside test_mapbox_* and
// test_environment_stream, so DEM PNGs are read back with stb_image (golden.cpp's) and the
// published terrain-RGB formula; decode_dem_png has its own known-pixel tests (test_mapbox_dem).

#include "imagery_fixture.hpp"
#include "mapbox_dem_fixture.hpp"
#include "quantized_mesh_fixture.hpp"

#include "stb_image.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace {

namespace dem = overlume::testing::dem;
namespace imagery = overlume::testing::imagery;
namespace qmesh = overlume::testing::qmesh;

struct Rgb {
    int width = 0, height = 0;
    std::vector<uint8_t> pixels;  // RGB
};

Rgb read_png(const std::vector<uint8_t>& bytes) {
    Rgb out;
    int channels = 0;
    stbi_uc* p = stbi_load_from_memory(bytes.data(), static_cast<int>(bytes.size()), &out.width,
                                       &out.height, &channels, 3);
    if (p != nullptr) {
        out.pixels.assign(p, p + size_t(out.width) * size_t(out.height) * 3);
        stbi_image_free(p);
    }
    return out;
}

// A strict JSON (RFC 8259) syntax check: true when `s` is exactly one JSON value.
struct JsonCheck {
    const std::string& s;
    size_t i = 0;

    void ws() {
        while (i < s.size() && (s[i] == ' ' || s[i] == '\t' || s[i] == '\r' || s[i] == '\n')) ++i;
    }
    bool lit(const char* word) {
        const size_t n = std::strlen(word);
        if (s.compare(i, n, word) != 0) return false;
        i += n;
        return true;
    }
    bool digits() {
        const size_t start = i;
        while (i < s.size() && std::isdigit(static_cast<unsigned char>(s[i])) != 0) ++i;
        return i > start;
    }
    bool number() {
        if (i < s.size() && s[i] == '-') ++i;
        if (!digits()) return false;
        if (i < s.size() && s[i] == '.') {
            ++i;
            if (!digits()) return false;
        }
        if (i < s.size() && (s[i] == 'e' || s[i] == 'E')) {
            ++i;
            if (i < s.size() && (s[i] == '+' || s[i] == '-')) ++i;
            return digits();
        }
        return true;
    }
    bool string() {
        if (i >= s.size() || s[i] != '"') return false;
        for (++i; i < s.size(); ++i) {
            const unsigned char c = static_cast<unsigned char>(s[i]);
            if (c == '"') {
                ++i;
                return true;
            }
            if (c < 0x20) return false;
            if (c == '\\') {
                if (++i >= s.size() || std::strchr("\"\\/bfnrtu", s[i]) == nullptr) return false;
                if (s[i] == 'u') {
                    for (int k = 0; k < 4; ++k) {
                        if (++i >= s.size() ||
                            std::isxdigit(static_cast<unsigned char>(s[i])) == 0) {
                            return false;
                        }
                    }
                }
            }
        }
        return false;
    }
    bool members(char close, bool object) {
        ++i;
        ws();
        if (i < s.size() && s[i] == close) {
            ++i;
            return true;
        }
        for (;;) {
            if (object) {
                ws();
                if (!string()) return false;
                ws();
                if (i >= s.size() || s[i++] != ':') return false;
            }
            if (!value()) return false;
            if (i < s.size() && s[i] == ',') {
                ++i;
                continue;
            }
            if (i < s.size() && s[i] == close) {
                ++i;
                return true;
            }
            return false;
        }
    }
    bool value() {
        ws();
        if (i >= s.size()) return false;
        bool ok = false;
        switch (s[i]) {
            case '{':
                ok = members('}', true);
                break;
            case '[':
                ok = members(']', false);
                break;
            case '"':
                ok = string();
                break;
            case 't':
                ok = lit("true");
                break;
            case 'f':
                ok = lit("false");
                break;
            case 'n':
                ok = lit("null");
                break;
            default:
                ok = number();
        }
        ws();
        return ok;
    }
};

bool parses_as_json(const std::string& s) {
    JsonCheck c{s};
    return c.value() && c.i == s.size();
}

bool contains(const std::string& haystack, const std::string& needle) {
    return haystack.find(needle) != std::string::npos;
}

// A parsed quantized-mesh tile: the fields the format defines, decoded here, not by cesium.
struct Qmesh {
    double center[3], bs_center[3], bs_radius;
    float min_h, max_h;
    uint32_t vertex_count = 0, triangle_count = 0;
    std::vector<uint16_t> u, v, h;               // after zigzag-delta decoding
    std::vector<uint32_t> indices;               // after high-water-mark decoding
    std::array<std::vector<uint16_t>, 4> edges;  // west, south, east, north
    size_t bytes_read = 0;
};

Qmesh parse(const std::vector<uint8_t>& b) {
    Qmesh q{};
    size_t at = 0;
    const auto read = [&b, &at](auto& value) {
        std::memcpy(&value, b.data() + at, sizeof value);
        at += sizeof value;
    };
    for (double& c : q.center) read(c);
    read(q.min_h);
    read(q.max_h);
    for (double& c : q.bs_center) read(c);
    read(q.bs_radius);
    at += 3 * sizeof(double);  // the horizon occlusion point
    EXPECT_EQ(at, 88u) << "the header is 88 bytes";
    read(q.vertex_count);
    for (std::vector<uint16_t>* values : {&q.u, &q.v, &q.h}) {
        int32_t running = 0;
        for (uint32_t k = 0; k < q.vertex_count; ++k) {
            uint16_t zz = 0;
            read(zz);
            running += (zz >> 1) ^ -(zz & 1);
            values->push_back(static_cast<uint16_t>(running));
        }
    }
    read(q.triangle_count);
    uint32_t highest = 0;
    for (uint32_t k = 0; k < q.triangle_count * 3; ++k) {
        uint16_t code = 0;
        read(code);
        q.indices.push_back(highest - code);
        if (code == 0) ++highest;
    }
    for (std::vector<uint16_t>& edge : q.edges) {
        uint32_t count = 0;
        read(count);
        edge.resize(count);
        for (uint16_t& index : edge) read(index);
    }
    q.bytes_read = at;
    return q;
}

}

TEST(TerrainFixtures, DemPngEncodesTheNamedScenes) {
    const std::pair<const char*, std::function<double(double, double)>> scenes[] = {
        {"ramp", dem::ramp_height_m},
        {"hill", dem::hill_height_m},
        {"shelf", dem::shelf_height_m},
        {"sea", dem::sea_height_m}};
    for (const auto& [name, height] : scenes) {
        const Rgb img = read_png(dem::encode_dem_png(height));
        ASSERT_EQ(img.width, 512) << name;
        ASSERT_EQ(img.height, 512) << name;
        double worst = 0.0;
        for (int py = 0; py < 512; ++py) {
            for (int px = 0; px < 512; ++px) {
                const uint8_t* p = &img.pixels[(size_t(py) * 512 + size_t(px)) * 3];
                const double decoded = -10000.0 + (p[0] * 65536.0 + p[1] * 256.0 + p[2]) * 0.1;
                worst = std::max(worst, std::abs(decoded - height(px + 0.5, py + 0.5)));
            }
        }
        EXPECT_LE(worst, 0.05 + 1e-6) << name << ": rounded to 0.1 m, so within 0.05 m";
    }
    EXPECT_DOUBLE_EQ(dem::ramp_height_m(0.0, 300.0), 0.0);
    EXPECT_DOUBLE_EQ(dem::ramp_height_m(512.0, 10.0), 20.0);
    EXPECT_DOUBLE_EQ(dem::hill_height_m(256.0, 256.0), 40.0);
    EXPECT_NEAR(dem::hill_height_m(0.0, 0.0), 40.0 * std::exp(-0.4096), 1e-9) << "sigma 400 px";
    EXPECT_DOUBLE_EQ(dem::shelf_height_m(100.0, 255.5), 10.0);
    EXPECT_DOUBLE_EQ(dem::shelf_height_m(100.0, 256.5), 0.0);
    EXPECT_DOUBLE_EQ(dem::sea_height_m(17.0, 400.0), 0.0);
}

TEST(TerrainFixtures, FlatPngDecodesToItsColour) {
    for (const uint32_t size : {512u, 256u}) {
        const Rgb img = read_png(imagery::flat_png(size, 10, 200, 30));
        ASSERT_EQ(img.width, static_cast<int>(size));
        ASSERT_EQ(img.height, static_cast<int>(size));
        for (size_t k = 0; k < img.pixels.size(); k += 3) {
            ASSERT_EQ(img.pixels[k], 10) << "pixel " << k / 3;
            ASSERT_EQ(img.pixels[k + 1], 200) << "pixel " << k / 3;
            ASSERT_EQ(img.pixels[k + 2], 30) << "pixel " << k / 3;
        }
    }
}

TEST(TerrainFixtures, QuantizedMeshHeaderAndCounts) {
    // Level 0, x 1 spans longitude 0..180 and latitude -90..90: its centre is (lon 90, lat 0).
    const std::vector<uint8_t> b = qmesh::encode_quantized_mesh(0, 1, 0, {8, 0.0, 0.0});
    const Qmesh q = parse(b);
    EXPECT_NEAR(q.center[0], 0.0, 1e-6);
    EXPECT_NEAR(q.center[1], 6378137.0, 1e-6);
    EXPECT_NEAR(q.center[2], 0.0, 1e-6);
    for (int c = 0; c < 3; ++c) EXPECT_EQ(q.bs_center[c], q.center[c]);
    // The lattice holds the corner at (lon 0, lat 0), ECEF (a, 0, 0), a * sqrt(2) from the centre.
    EXPECT_GE(q.bs_radius, 6378137.0 * std::sqrt(2.0) - 1e-3);
    EXPECT_EQ(q.min_h, 0.0f);
    EXPECT_EQ(q.max_h, 0.0f);
    EXPECT_EQ(q.vertex_count, 81u);     // 9 x 9
    EXPECT_EQ(q.triangle_count, 128u);  // 8 x 8 cells, two each
    for (const std::vector<uint16_t>& edge : q.edges) EXPECT_EQ(edge.size(), 9u);
    // 88 + 4 + 3 * 81 * 2 + 4 + 128 * 3 * 2 + 4 * (4 + 9 * 2): nothing after the edges.
    EXPECT_EQ(b.size(), 1438u);
    EXPECT_EQ(q.bytes_read, b.size());

    const Qmesh ramp = parse(qmesh::encode_quantized_mesh(14, 17000, 6000, {4, 10.0, 50.0}));
    EXPECT_EQ(ramp.min_h, 10.0f);
    EXPECT_EQ(ramp.max_h, 50.0f);
    EXPECT_EQ(ramp.vertex_count, 25u);
    EXPECT_TRUE(qmesh::encode_quantized_mesh(14, 0, 0, {0, 0.0, 0.0}).empty());
    EXPECT_TRUE(qmesh::encode_quantized_mesh(14, 0, 0, {256, 0.0, 0.0}).empty());
    EXPECT_FALSE(qmesh::encode_quantized_mesh(14, 0, 0, {255, 0.0, 0.0}).empty());
}

TEST(TerrainFixtures, QuantizedMeshVerticesDecodeFromZigzagDeltas) {
    for (const bool sloped : {false, true}) {
        const Qmesh q =
            parse(qmesh::encode_quantized_mesh(15, 40000, 12000, {8, 10.0, sloped ? 50.0 : 10.0}));
        std::set<std::pair<uint16_t, uint16_t>> lattice;
        for (uint32_t k = 0; k < q.vertex_count; ++k) {
            ASSERT_LE(q.u[k], 32767) << "vertex " << k;
            ASSERT_LE(q.v[k], 32767) << "vertex " << k;
            lattice.insert({q.u[k], q.v[k]});
            // Flat: every height code 0. The ramp rises west to east, so height tracks u.
            EXPECT_EQ(q.h[k], sloped ? q.u[k] : 0) << "vertex " << k;
        }
        ASSERT_EQ(lattice.size(), 81u) << "each lattice point once";
        std::set<uint16_t> steps;
        for (const auto& [u, v] : lattice) {
            steps.insert(u);
            steps.insert(v);
        }
        EXPECT_EQ(steps,
                  (std::set<uint16_t>{0, 4096, 8192, 12288, 16384, 20479, 24575, 28671, 32767}));
    }
}

TEST(TerrainFixtures, QuantizedMeshIndicesDecodeWithTheHighWaterMark) {
    const Qmesh q = parse(qmesh::encode_quantized_mesh(15, 40000, 12000, {8, 0.0, 0.0}));
    ASSERT_EQ(q.indices.size(), 384u);
    EXPECT_EQ(q.indices[0], 0u) << "the first index is a new vertex";
    std::set<uint32_t> used;
    for (size_t t = 0; t < q.indices.size(); t += 3) {
        const uint32_t a = q.indices[t], b = q.indices[t + 1], c = q.indices[t + 2];
        ASSERT_LT(std::max({a, b, c}), q.vertex_count) << "triangle " << t / 3;
        used.insert({a, b, c});
        // Counter-clockwise seen from above: u east, v north.
        const double cross = (double(q.u[b]) - q.u[a]) * (double(q.v[c]) - q.v[a]) -
                             (double(q.v[b]) - q.v[a]) * (double(q.u[c]) - q.u[a]);
        EXPECT_GT(cross, 0.0) << "triangle " << t / 3;
    }
    EXPECT_EQ(used.size(), size_t{q.vertex_count}) << "every vertex in some triangle";
    const char* names[] = {"west", "south", "east", "north"};
    for (int e = 0; e < 4; ++e) {
        std::set<uint16_t> distinct(q.edges[e].begin(), q.edges[e].end());
        EXPECT_EQ(distinct.size(), 9u) << names[e];
        for (const uint16_t k : q.edges[e]) {
            ASSERT_LT(k, q.vertex_count) << names[e];
            const uint16_t along = (e % 2 == 0) ? q.u[k] : q.v[k];
            EXPECT_EQ(along, e < 2 ? 0 : 32767) << names[e] << " edge vertex " << k;
        }
    }
}

TEST(TerrainFixtures, LayerJsonParsesAndListsTheAvailableTiles) {
    EXPECT_FALSE(parses_as_json("{\"a\":1 \"b\":2}")) << "the checker sees a missing comma";
    EXPECT_FALSE(parses_as_json("[1,]")) << "and a trailing one";
    EXPECT_TRUE(parses_as_json("{\"a\":[1,-2.5e3,true,null,\"x\\\"y\"]}"));

    const std::string json = qmesh::layer_json({{0, 0, 0}, {0, 1, 0}, {2, 5, 3}});
    EXPECT_TRUE(parses_as_json(json)) << json;
    EXPECT_TRUE(contains(json,
                         "\"available\":[[{\"startX\":0,\"startY\":0,\"endX\":0,\"endY\":0},"
                         "{\"startX\":1,\"startY\":0,\"endX\":1,\"endY\":0}],[],"
                         "[{\"startX\":5,\"startY\":3,\"endX\":5,\"endY\":3}]]"))
        << json;
    EXPECT_TRUE(contains(json, "\"format\":\"quantized-mesh-1.0\"")) << json;
    EXPECT_TRUE(contains(json, "\"projection\":\"EPSG:4326\"")) << json;
    EXPECT_TRUE(contains(json, "\"tiles\":[\"{z}/{x}/{y}.terrain?v={version}\"]")) << json;
    EXPECT_TRUE(contains(json, "\"maxzoom\":2")) << json;
    EXPECT_TRUE(parses_as_json(qmesh::layer_json({}))) << "no tiles: one empty level";
}

TEST(TerrainFixtures, IonEndpointBodiesParseWithTheirTypes) {
    const std::string terrain = qmesh::ion_terrain_endpoint_json("https://assets.fixture/1/");
    const std::string imagery = qmesh::ion_imagery_endpoint_json("https://assets.fixture/2/");
    for (const std::string* body : {&terrain, &imagery}) {
        EXPECT_TRUE(parses_as_json(*body)) << *body;
        EXPECT_TRUE(
            contains(*body, std::string("\"accessToken\":\"") + qmesh::kFixtureAssetToken + "\""))
            << *body;
        EXPECT_FALSE(contains(*body, "externalType")) << *body;
    }
    EXPECT_TRUE(contains(terrain, "\"type\":\"TERRAIN\",\"url\":\"https://assets.fixture/1/\""));
    EXPECT_TRUE(contains(terrain, std::string("\"html\":\"") + qmesh::kTerrainAttributionHtml));
    EXPECT_TRUE(contains(imagery, "\"type\":\"IMAGERY\",\"url\":\"https://assets.fixture/2/\""));
    EXPECT_TRUE(contains(imagery, std::string("\"html\":\"") + qmesh::kImageryAttributionHtml));
}

TEST(TerrainFixtures, TileMapResourceListsGeodeticLevels) {
    const std::string xml = qmesh::tilemapresource_xml(256);
    EXPECT_EQ(xml.rfind("<?xml version=\"1.0\"", 0), 0u);
    EXPECT_EQ(xml.substr(xml.size() - 11), "</TileMap>\n");
    EXPECT_TRUE(contains(xml, "<SRS>EPSG:4326</SRS>"));
    EXPECT_TRUE(contains(xml, "<TileSets profile=\"geodetic\">"));
    EXPECT_TRUE(contains(xml,
                         "<TileFormat width=\"256\" height=\"256\" mime-type=\"image/png\" "
                         "extension=\"png\"/>"));
    size_t tilesets = 0;
    for (size_t at = xml.find("<TileSet "); at != std::string::npos;
         at = xml.find("<TileSet ", at + 1)) {
        ++tilesets;
    }
    EXPECT_EQ(tilesets, size_t{qmesh::kTmsMaxLevel} + 1);
    for (uint32_t level = 0; level <= qmesh::kTmsMaxLevel; ++level) {
        const std::string l = std::to_string(level);
        EXPECT_TRUE(contains(xml, "<TileSet href=\"" + l + "\" order=\"" + l + "\"/>")) << level;
    }
    EXPECT_TRUE(contains(qmesh::tilemapresource_xml(512), "width=\"512\" height=\"512\""));
}
```

What each test catches:
- `DemPngEncodesTheNamedScenes`: a wrong formula, rounding or channel order in `encode_dem_png`,
  pixel centres off by half a pixel, any change to a named scene (ramp span, hill height or
  sigma, shelf half, sea level).
- `FlatPngDecodesToItsColour`: a wrong size or channel in `flat_png`.
- `QuantizedMeshHeaderAndCounts`: header fields out of order or of the wrong type (a known tile
  whose centre is ECEF (0, a, 0)), a bounding sphere that misses the lattice, wrong counts,
  stray bytes after the edge lists, grids beyond 16-bit indices accepted.
- `QuantizedMeshVerticesDecodeFromZigzagDeltas`: zigzag or delta coding broken, a lattice point
  missing or doubled, heights not following the ramp (or not 0 when flat).
- `QuantizedMeshIndicesDecodeWithTheHighWaterMark`: the high-water-mark code broken (indices out
  of range), vertices not numbered in first-use order, clockwise triangles, an unused vertex,
  an edge list holding a vertex off its edge or missing one.
- `LayerJsonParsesAndListsTheAvailableTiles`: invalid JSON (a missing or extra comma), a tile
  missing from `available` or put on the wrong level, a missing empty level, the projection,
  format, template or `maxzoom` changed.
- `IonEndpointBodiesParseWithTheirTypes`: invalid JSON, a wrong `type`, the URL (with its
  trailing `/`) not passed through verbatim, an `externalType` that would steer cesium away from
  the TMS path, the token or the attribution missing.
- `TileMapResourceListsGeodeticLevels`: a missing SRS, profile or level, hrefs that differ from
  the order, `tile_size` not applied.

- [ ] **Step 2: Run it and see it fail**

```bash
cd "$WORKTREE"
flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock cmake -S overlume -B overlume/build > /dev/null \
    && echo reconfigured \
    && CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
        --target test_terrain_fixtures
```

No `reconfigured` line and no CMake error above it means the lock wait ran out (exit 75) and nothing
was built: run the step again.

Expected: `reconfigured`, then the compile fails with

```
…/overlume/tests/test_terrain_fixtures.cpp:9:10: fatal error: 'imagery_fixture.hpp' file not found
```

and the build exits non-zero.

- [ ] **Step 3: Write the imagery fixture** `overlume/tests/imagery_fixture.hpp`

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

// Imagery tiles synthesized in the test process, so no Mapbox or ion imagery enters the tree: one
// flat colour per tile, which makes attachment and UV mapping pixel-checkable. Header-only C++17.
// stb_image_write's implementation comes from golden.cpp, which every test binary compiles.

#include "stb_image_write.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace overlume::testing::imagery {

// 8-bit pixels (row-major, `channels` per pixel) as PNG bytes.
inline std::vector<uint8_t> png(int width, int height, int channels,
                                const std::vector<uint8_t>& pixels) {
    std::vector<uint8_t> out;
    stbi_write_png_to_func(
        [](void* context, void* data, int size) {
            auto* bytes = static_cast<std::vector<uint8_t>*>(context);
            bytes->insert(bytes->end(), static_cast<uint8_t*>(data),
                          static_cast<uint8_t*>(data) + size);
        },
        &out, width, height, channels, pixels.data(), width * channels);
    return out;
}

// A size x size RGB PNG in one colour (JPEG is not needed: the overlay reads any image cesium
// decodes).
inline std::vector<uint8_t> flat_png(uint32_t size, uint8_t r, uint8_t g, uint8_t b) {
    std::vector<uint8_t> rgb(size_t{size} * size * 3);
    for (size_t i = 0; i < rgb.size(); i += 3) {
        rgb[i] = r;
        rgb[i + 1] = g;
        rgb[i + 2] = b;
    }
    return png(static_cast<int>(size), static_cast<int>(size), 3, rgb);
}

}
```

- [ ] **Step 4: Write the DEM fixture** `overlume/tests/mapbox_dem_fixture.hpp`

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

// Mapbox terrain-DEM tiles (512 px terrain-RGB PNGs) synthesized in the test process from a height
// function, so no Mapbox bytes enter the tree. Header-only C++17.

#include "imagery_fixture.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

namespace overlume::testing::dem {

inline constexpr int kDemSide = 512;  // pixels per side, as mapbox::kDemSize

// A 512 x 512 RGB PNG whose pixel (px, py) encodes height_m(u = px + 0.5, v = py + 0.5) with the
// Mapbox terrain-RGB formula (stb_image_write). Heights are rounded to 0.1 m.
inline std::vector<uint8_t> encode_dem_png(
    const std::function<double(double u, double v)>& height_m) {
    std::vector<uint8_t> rgb(size_t{kDemSide} * kDemSide * 3);
    for (int py = 0; py < kDemSide; ++py) {
        for (int px = 0; px < kDemSide; ++px) {
            // height = -10000 + code * 0.1, code = R * 65536 + G * 256 + B
            const double code = std::round((height_m(px + 0.5, py + 0.5) + 10000.0) * 10.0);
            const auto c = static_cast<uint32_t>(std::clamp(code, 0.0, 16777215.0));
            uint8_t* p = &rgb[(size_t(py) * kDemSide + size_t(px)) * 3];
            p[0] = static_cast<uint8_t>(c >> 16);
            p[1] = static_cast<uint8_t>(c >> 8);
            p[2] = static_cast<uint8_t>(c);
        }
    }
    return imagery::png(kDemSide, kDemSide, 3, rgb);
}

// Named scenes used by several tasks (u east, v south, in tile pixels 0..512).
// A ramp rising 0..20 m west to east.
inline double ramp_height_m(double u, double /*v*/) { return 20.0 * u / kDemSide; }

// A hill of 40 m at the tile centre (gaussian, sigma 400 px).
inline double hill_height_m(double u, double v) {
    const double du = u - kDemSide / 2.0, dv = v - kDemSide / 2.0;
    return 40.0 * std::exp(-(du * du + dv * dv) / (2.0 * 400.0 * 400.0));
}

// A shelf 10 m high on the north half.
inline double shelf_height_m(double /*u*/, double v) { return v < kDemSide / 2.0 ? 10.0 : 0.0; }

// Sea: 0 everywhere.
inline double sea_height_m(double /*u*/, double /*v*/) { return 0.0; }

}
```

- [ ] **Step 5: Write the quantized-mesh fixture** `overlume/tests/quantized_mesh_fixture.hpp`

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

// Cesium terrain synthesized in the test process, so no ion bytes enter the tree:
// quantized-mesh-1.0 tiles (https://github.com/CesiumGS/quantized-mesh: an 88-byte header, the
// vertex count, zigzag-delta u/v/height, high-water-mark 16-bit indices, edge indices; no
// extensions), the layer.json that lists them, the Cesium ion endpoint bodies and a TMS
// tilemapresource.xml. Header-only C++17. Little-endian hosts only, as every Overlume target is.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

namespace overlume::testing::qmesh {

struct QuantizedMeshOptions {
    uint32_t grid = 8;  // cells per side, 1..255 (so the vertices fit 16-bit indices)
    double min_height_m = 0.0;
    double max_height_m = 0.0;
};

// What the fixture endpoints hand out. The token is a fixed fake, never a credential.
inline constexpr const char* kFixtureAssetToken = "overlume-fixture-asset-token";
inline constexpr const char* kTerrainAttributionHtml = "<span>Overlume fixture terrain</span>";
inline constexpr const char* kImageryAttributionHtml = "<b>Overlume fixture imagery</b>";
inline constexpr uint32_t kTmsMaxLevel = 18;  // tilemapresource_xml lists levels 0..18

namespace detail {

template <typename T>
void put(std::vector<uint8_t>& out, T value) {
    uint8_t bytes[sizeof(T)];
    std::memcpy(bytes, &value, sizeof(T));
    out.insert(out.end(), bytes, bytes + sizeof(T));
}

inline uint16_t zigzag(int32_t d) {
    return static_cast<uint16_t>((static_cast<uint32_t>(d) << 1) ^ static_cast<uint32_t>(d >> 31));
}

// WGS84 longitude, latitude (degrees) and ellipsoidal height (m) to ECEF metres.
inline std::array<double, 3> ecef(double lon_deg, double lat_deg, double height_m) {
    constexpr double kA = 6378137.0, kE2 = 6.69437999014e-3;
    constexpr double kRad = 3.14159265358979323846 / 180.0;
    const double lon = lon_deg * kRad, lat = lat_deg * kRad;
    const double n = kA / std::sqrt(1.0 - kE2 * std::sin(lat) * std::sin(lat));
    return {(n + height_m) * std::cos(lat) * std::cos(lon),
            (n + height_m) * std::cos(lat) * std::sin(lon),
            (n * (1.0 - kE2) + height_m) * std::sin(lat)};
}

}

// One quantized-mesh tile (header, zigzag-delta u/v/height, 16-bit indices, edge indices) covering
// the tile at x, y, level of the geographic (EPSG:4326) TMS quadtree (2 x 1 root tiles, y = 0 at
// the south), flat at height (min == max) or a ramp between them rising west to east. Vertices are
// a (grid + 1)^2 lattice; empty when grid is 0 or above 255.
inline std::vector<uint8_t> encode_quantized_mesh(uint32_t level, uint32_t x, uint32_t y,
                                                  const QuantizedMeshOptions& o) {
    if (o.grid == 0 || o.grid > 255) return {};
    const uint32_t n = o.grid + 1;  // lattice vertices per side
    const double size_deg = 180.0 / std::ldexp(1.0, static_cast<int>(level));
    const double west = -180.0 + x * size_deg, south = -90.0 + y * size_deg;
    const double span_m = o.max_height_m - o.min_height_m;
    const auto quantize = [&o](uint32_t k) {
        return static_cast<uint16_t>((k * 32767u + o.grid / 2) / o.grid);
    };
    // Two triangles per cell, counter-clockwise seen from above; lattice vertex (i east, j north)
    // is j * n + i.
    std::vector<uint32_t> triangles;
    for (uint32_t j = 0; j < o.grid; ++j) {
        for (uint32_t i = 0; i < o.grid; ++i) {
            const uint32_t sw = j * n + i, se = sw + 1, nw = sw + n, ne = nw + 1;
            triangles.insert(triangles.end(), {sw, se, ne, sw, ne, nw});
        }
    }
    // The high-water-mark index code needs vertices numbered in order of first use.
    std::vector<uint32_t> id_of(size_t{n} * n, std::numeric_limits<uint32_t>::max()), lattice_of;
    for (const uint32_t g : triangles) {
        if (id_of[g] == std::numeric_limits<uint32_t>::max()) {
            id_of[g] = static_cast<uint32_t>(lattice_of.size());
            lattice_of.push_back(g);
        }
    }
    const std::array<double, 3> centre = detail::ecef(west + size_deg / 2.0, south + size_deg / 2.0,
                                                      (o.min_height_m + o.max_height_m) / 2.0);
    std::vector<uint16_t> us, vs, hs;
    double radius = 0.0;
    for (const uint32_t g : lattice_of) {
        const uint32_t i = g % n, j = g / n;
        us.push_back(quantize(i));
        vs.push_back(quantize(j));
        hs.push_back(span_m > 0.0 ? quantize(i) : uint16_t{0});
        const std::array<double, 3> p =
            detail::ecef(west + size_deg * i / o.grid, south + size_deg * j / o.grid,
                         o.min_height_m + span_m * i / o.grid);
        radius = std::max(radius, std::hypot(p[0] - centre[0], p[1] - centre[1], p[2] - centre[2]));
    }

    std::vector<uint8_t> out;
    for (const double c : centre) detail::put(out, c);  // CenterX, Y, Z
    detail::put(out, static_cast<float>(o.min_height_m));
    detail::put(out, static_cast<float>(o.max_height_m));
    for (const double c : centre) detail::put(out, c);  // BoundingSphereCenter
    detail::put(out, radius);                           // BoundingSphereRadius
    // ponytail: the horizon occlusion point is the ellipsoid-scaled centre, not CesiumJS's horizon
    // point; cesium-native reads it and never uses it. Compute the real point if a consumer culls.
    detail::put(out, centre[0] / 6378137.0);
    detail::put(out, centre[1] / 6378137.0);
    detail::put(out, centre[2] / 6356752.314245179);
    detail::put(out, static_cast<uint32_t>(lattice_of.size()));  // vertexCount
    for (const std::vector<uint16_t>* values : {&us, &vs, &hs}) {
        int32_t previous = 0;
        for (const uint16_t v : *values) {
            detail::put(out, detail::zigzag(int32_t{v} - previous));
            previous = v;
        }
    }
    detail::put(out, static_cast<uint32_t>(triangles.size() / 3));  // triangleCount
    uint32_t highest = 0;
    for (const uint32_t g : triangles) {
        detail::put(out, static_cast<uint16_t>(highest - id_of[g]));
        if (id_of[g] == highest) ++highest;
    }
    // Edge indices: west (i = 0), south (j = 0), east (i = grid), north (j = grid).
    for (int edge = 0; edge < 4; ++edge) {
        detail::put(out, n);
        for (uint32_t k = 0; k < n; ++k) {
            const uint32_t i = edge == 0 ? 0 : edge == 2 ? o.grid : k;
            const uint32_t j = edge == 1 ? 0 : edge == 3 ? o.grid : k;
            detail::put(out, static_cast<uint16_t>(id_of[j * n + i]));
        }
    }
    return out;
}

// A layer.json for a geographic tiling scheme with `available` for exactly the tiles listed,
// each (level, x, y). Tiles resolve against its URL as "{z}/{x}/{y}.terrain?v=1.0.0".
inline std::string layer_json(const std::vector<std::array<uint32_t, 3>>& available_level_x_y) {
    uint32_t max_level = 0;
    for (const std::array<uint32_t, 3>& t : available_level_x_y) {
        max_level = std::max(max_level, t[0]);
    }
    std::string available;
    for (uint32_t level = 0; level <= max_level; ++level) {
        std::string ranges;
        for (const std::array<uint32_t, 3>& t : available_level_x_y) {
            if (t[0] != level) continue;
            const std::string x = std::to_string(t[1]), y = std::to_string(t[2]);
            ranges += std::string(ranges.empty() ? "" : ",") + "{\"startX\":" + x +
                      ",\"startY\":" + y + ",\"endX\":" + x + ",\"endY\":" + y + "}";
        }
        available += std::string(level == 0 ? "" : ",") + "[" + ranges + "]";
    }
    return "{\"tilejson\":\"2.1.0\",\"format\":\"quantized-mesh-1.0\",\"version\":\"1.0.0\","
           "\"scheme\":\"tms\",\"projection\":\"EPSG:4326\","
           "\"tiles\":[\"{z}/{x}/{y}.terrain?v={version}\"],\"maxzoom\":" +
           std::to_string(max_level) + ",\"available\":[" + available + "]}";
}

// Cesium ion endpoint bodies the fixture network serves at
// https://api.cesium.com/v1/assets/{id}/endpoint. URLs carry no quote or backslash and end with
// '/': cesium resolves "layer.json" against the terrain URL, and adds a missing '/' to a TMS URL
// only when its path has 4 or more characters (a base like ".../2" would lose its last segment).
inline std::string ion_terrain_endpoint_json(const std::string& layer_json_base_url) {
    return std::string("{\"type\":\"TERRAIN\",\"url\":\"") + layer_json_base_url +
           "\",\"accessToken\":\"" + kFixtureAssetToken + "\",\"attributions\":[{\"html\":\"" +
           kTerrainAttributionHtml + "\",\"collapsible\":false}]}";
}

// Type IMAGERY with no externalType: cesium opens the URL as a Tile Map Service.
inline std::string ion_imagery_endpoint_json(const std::string& tms_base_url) {
    return std::string("{\"type\":\"IMAGERY\",\"url\":\"") + tms_base_url +
           "\",\"accessToken\":\"" + kFixtureAssetToken + "\",\"attributions\":[{\"html\":\"" +
           kImageryAttributionHtml + "\",\"collapsible\":false}]}";
}

// For the TMS imagery: a geodetic (EPSG:4326) tile map with levels 0..kTmsMaxLevel, each at
// href "<level>", so tiles are <tms_base_url><level>/<x>/<y>.png (y = 0 at the south).
inline std::string tilemapresource_xml(uint32_t tile_size) {
    const std::string s = std::to_string(tile_size);
    std::string xml =
        "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n"
        "<TileMap version=\"1.0.0\" tilemapservice=\"http://tms.osgeo.org/1.0.0\">\n"
        "  <SRS>EPSG:4326</SRS>\n"
        "  <BoundingBox minx=\"-180.0\" miny=\"-90.0\" maxx=\"180.0\" maxy=\"90.0\"/>\n"
        "  <Origin x=\"-180.0\" y=\"-90.0\"/>\n"
        "  <TileFormat width=\"" +
        s + "\" height=\"" + s + "\" mime-type=\"image/png\" extension=\"png\"/>\n" +
        "  <TileSets profile=\"geodetic\">\n";
    for (uint32_t level = 0; level <= kTmsMaxLevel; ++level) {
        const std::string l = std::to_string(level);
        xml += "    <TileSet href=\"" + l + "\" order=\"" + l + "\"/>\n";
    }
    return xml + "  </TileSets>\n</TileMap>\n";
}

}
```

- [ ] **Step 6: Run it and see it pass**

No new `.cpp`, so no configure: the target exists since Step 2.

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_terrain_fixtures \
    && overlume/build/test_terrain_fixtures
nm -u -C overlume/build/CMakeFiles/test_terrain_fixtures.dir/tests/test_terrain_fixtures.cpp.o \
    | grep -c 'overlume::' || true
```

Expected: a clean build, a run that ends

```
[==========] 8 tests from 1 test suite ran. (… ms total)
[  PASSED  ] 8 tests.
```

and then `0`: the test needs no library symbol, so it links in the cesium-off build too.

- [ ] **Step 7: Format**

```bash
cd "$WORKTREE"
git add overlume/tests/imagery_fixture.hpp overlume/tests/mapbox_dem_fixture.hpp \
    overlume/tests/quantized_mesh_fixture.hpp overlume/tests/test_terrain_fixtures.cpp
tools/check_format.sh --fix overlume/tests/imagery_fixture.hpp overlume/tests/mapbox_dem_fixture.hpp \
    overlume/tests/quantized_mesh_fixture.hpp overlume/tests/test_terrain_fixtures.cpp
git diff --stat
git add overlume/tests/imagery_fixture.hpp overlume/tests/mapbox_dem_fixture.hpp \
    overlume/tests/quantized_mesh_fixture.hpp overlume/tests/test_terrain_fixtures.cpp
tools/check_format.sh && tools/check_spdx.sh
```

Expected: `check_format.sh: reformatted 4 files with clang-format 23.1.1`; `git diff --stat` prints
nothing (the code above is already clang-format 23.1.1's output); then
`check_format.sh: PASS (222 files, clang-format 23.1.1)` and
`check_spdx.sh: PASS (380 files carry the SPDX identifier)` (218 and 376 after Task 1, plus these
four; if the fork's counts differed, each expected count is Task 1's plus four).

- [ ] **Step 8: Revert check (do not commit)**: the index code must advance its high-water mark

```bash
cd "$WORKTREE"
sed -i '/if (id_of\[g\] == highest) ++highest;/d' overlume/tests/quantized_mesh_fixture.hpp
git diff --stat overlume/tests/quantized_mesh_fixture.hpp
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_terrain_fixtures
overlume/build/test_terrain_fixtures | tail -4
git checkout -- overlume/tests/quantized_mesh_fixture.hpp \
    && git diff --stat overlume/tests/quantized_mesh_fixture.hpp
```

Expected: the first diff reports `1 file changed, 1 deletion(-)`; the run ends

```
[  FAILED  ] 1 test, listed below:
[  FAILED  ] TerrainFixtures.QuantizedMeshIndicesDecodeWithTheHighWaterMark

 1 FAILED TEST
```

because every index after the first then decodes as `0 - code`, far beyond the 81 vertices, which
cesium would read as out-of-range indices. After the checkout the last diff prints nothing.

- [ ] **Step 9: Gate** (foreground, Bash timeout 600000 ms)

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
ROS_DOMAIN_ID=81 CI_VISUAL_MODE_DOMAIN_ID=81 \
    flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock tools/ci_visual_mode.sh
```

Expected (Task 1's counts plus the 8 `TerrainFixtures.*` tests):
- `PASS  POD header check`
- `PASS  library ctest suite  (100% tests passed, 0 tests failed out of 461 (443 ok / 8 skipped))`
- `PASS  node gtests  (Summary: 381 tests, 0 errors, 0 failures, 0 skipped)`
- `PASS  WS bridge pytest suite  (94 passed in …)`, the time varying
- `PASS  golden suite: 25 ok, 0 skipped`
- `PASS  examples  (6 run, all exited 0 with non-empty output)`
- `OVERALL: PASS`, exit 0.

Exit 75 means another lane held the lock for 300 s: run the same command again. Any other non-zero
exit is a gate failure; a WS bridge failure can come from another session's gate on the same
ports, so rerun before treating it as a finding. The prebuild also rebuilds
`test_terrain_fixtures` from the header Step 8 restored.

- [ ] **Step 10: Commit**

```bash
cd "$WORKTREE"
git add overlume/tests/imagery_fixture.hpp overlume/tests/mapbox_dem_fixture.hpp \
    overlume/tests/quantized_mesh_fixture.hpp overlume/tests/test_terrain_fixtures.cpp
git status --short
git commit -F - <<'EOF'
test(terrain): synthetic DEM, imagery and quantized-mesh fixtures

The terrain and imagery work needs tiles without Mapbox or ion bytes in
the tree. mapbox_dem_fixture.hpp encodes a height function as a 512 px
terrain-RGB PNG and names the scenes later tests share: a 0..20 m ramp,
a 40 m hill (sigma 400 px), a 10 m shelf on the north half, and sea.
imagery_fixture.hpp writes one-colour PNG tiles. quantized_mesh_fixture.hpp
writes quantized-mesh-1.0 tiles (the 88-byte header, zigzag-delta
vertices, high-water-mark indices, edge lists), the layer.json that
lists them, the ion terrain and imagery endpoint bodies (TERRAIN, and
IMAGERY served as a Tile Map Service) and a geodetic tilemapresource.xml.

TerrainFixtures pins them with no library symbol, because the
cesium-off build compiles every test outside test_mapbox_*: the DEM
PNGs read back through stb_image and the published formula within
0.05 m, the flat colour, the quantized-mesh header, counts, vertex
stream and index code decoded in the test, and every JSON body through
a strict syntax check. cesium loading these tiles end to end is the ion
proof's check.

Claude-Session: <the executing session's URL>
EOF
```

Expected: `git status --short` lists exactly the four files, each as `A `; the commit succeeds.

### Task 3: Tiling helper extracted from the buildings loader

**Lane:** 2. **Depends on:** Task 0 (the lane worktree forks after it; Task 0 leaves every file this
task touches as it is at 427f5d8, so the line numbers below are 427f5d8's).

**Worktree:** `$WORKTREE` is `$INTEGRATION-l2` (branch `terrain-env/l2`, ROS domain 82, test
scratch `/tmp/overlume-l2`). Every path in this lane is relative to it, and every bash block starts
with `cd "$WORKTREE"`: a shell can start anywhere, including the primary checkout, which is never
built in or edited.

**Files:**
- Create: `overlume/src/mapbox_tiling.hpp` (the contract's block, made concrete, plus the request
  and result plumbing both Mapbox loaders need; C++20, `overlume_stream` only)
- Create: `overlume/src/mapbox_tiling.cpp`
- Modify: `overlume/src/mapbox_loader.cpp` (the include block, 10-44; the retry and region
  constants, 122-127; `struct LoaderConfig` and the head of `struct LoaderState`, 150-158;
  `LoaderState`'s log dedup, 167-174; `g_testClockS` through `success`, 192-270; `note_status`,
  363-384; `build_tile`'s last `return`, 525-526; the loader's class comment, 529-531; the attempt
  bookkeeping in `loadTileContent`, 545-562; `createTileChildren` and the private members,
  583-624; `make_root`, 626-641; its call in `open_mapbox_source`, 680; `mapbox_set_test_clock`,
  777)
- Test: none new. `overlume/tests/test_mapbox_buildings.cpp` and `overlume/tests/test_mapbox_uri.cpp`
  stay byte-identical and pin the move (no test can include `mapbox_tiling.hpp`: see API notes).

**Interfaces:**
- Consumes:

```cpp
// Cesium3DTilesSelection (cesium-native 0.64)
explicit Tile(TilesetContentLoader* pLoader, const TileID& tileID = {}) noexcept;
Tile(TilesetContentLoader* pLoader, const TileID& tileID, TileEmptyContent emptyContent) noexcept;
TilesetContentLoader* Tile::getLoader() const noexcept;
const TileID& Tile::getTileID() const noexcept;
void Tile::createChildTiles(std::vector<Tile>&& children);
using TileContentKind =
    std::variant<TileUnknownContent, TileEmptyContent, TileExternalContent, CesiumGltf::Model>;
// CesiumGeospatial
BoundingRegion(const GlobeRectangle& rectangle, double minimumHeight, double maximumHeight,
               const Ellipsoid& ellipsoid);
static GlobeRectangle GlobeRectangle::fromDegrees(double west, double south, double east,
                                                  double north);
static const Ellipsoid Ellipsoid::WGS84;
// overlume/src/mapbox_tile_frame.hpp (sub-project 1)
inline constexpr uint32_t kLandmarkZoom = 14;
struct TileId { uint32_t z = 0; uint32_t x = 0; uint32_t y = 0; };
LonLatRect tile_bounds(const TileId& t);
TileFrame make_tile_frame(const TileId& z14, double origin_height_m);
// overlume/src/mapbox_test_hooks.hpp:80-82 (declaration unchanged; its definition moves here)
void mapbox_set_test_clock(double seconds);
```

- Produces, for Tasks 9, 10 and 11 (`overlume/src/mapbox_tiling.hpp`, namespace
  `overlume::mapbox`). The contract's block, with its `/* as the loader does today */` made
  concrete:

```cpp
inline constexpr uint32_t kMaxAttempts = 3;
inline constexpr double kRetryWaitS = 2.0;
struct Attempt { uint32_t count = 0; double not_before_s = 0.0; std::shared_ptr<std::atomic<bool>> settled; };
class AttemptBook {
public:
    bool waiting(const TileId& t14, double now_s) const;  // the retry wait still runs: RetryLater, no request
    Attempt& begin(const TileId& t14, double now_s);
};
Cesium3DTilesSelection::BoundingVolume tile_region(
    const TileId& t, double min_height_m, double max_height_m,
    const CesiumGeospatial::Ellipsoid& ellipsoid = CesiumGeospatial::Ellipsoid::WGS84);
std::vector<Cesium3DTilesSelection::Tile> quadtree_children(
    const Cesium3DTilesSelection::Tile& parent, const LoaderConfig& cfg,
    const CesiumGeospatial::Ellipsoid& ellipsoid = CesiumGeospatial::Ellipsoid::WGS84);
bool transient_status(uint16_t status);  // 429, 5xx, 0 (transport)
struct SourceLog { bool first_time(const std::string& key); };  // members private
```

  The contract's "Also exported" line, with the exact signatures it leaves to this task (each was a
  name in `mapbox_loader.cpp`'s anonymous namespace, and the File map gives
  `mapbox_tiling.{hpp,cpp}` to no later task, so whatever Tasks 9-11 share must arrive here):
  - `struct LoaderConfig { std::string token; bool materials_original = false; double origin_height_m = 0.0; };`
    (the contract's `quadtree_children` names it; moved unchanged, the terrain loader reuses it
    and ignores `materials_original`).
  - `inline constexpr double kRegionBelowM = 10.0;`, `inline constexpr double kRegionAboveM = 1000.0;`
    (a tile spans the anchor height `-kRegionBelowM` to `+kRegionAboveM` until its build reports
    its real top).
  - `std::unique_ptr<Cesium3DTilesSelection::Tile> quadtree_root(Cesium3DTilesSelection::TilesetContentLoader& loader, double origin_height_m);`
    (the old `make_root`: the root and its one z0 child, both empty and always refined).
  - `double clock_s();` (the retry wait's clock; `mapbox_set_test_clock` freezes it for every
    Mapbox loader).
  - `struct Fetched { uint16_t status = 0; std::vector<std::byte> body; bool transport_error = false; };`,
    `enum class Outcome { Body, Empty, Auth, Transient, Other };`,
    `Outcome classify(const Fetched& f);`,
    `CesiumAsync::Future<Fetched> fetch(const CesiumAsync::AsyncSystem& asyncSystem, const std::shared_ptr<CesiumAsync::IAssetAccessor>& accessor, const std::string& url);`
    (never rejects; keeps the body of a 200 only).
  - `void note_status(SourceLog& seen, spdlog::logger& log, const char* endpoint, const Fetched& f, const TileId& t14);`
    (one line per endpoint and status per source: `Mapbox rejected MAPBOX_TOKEN (HTTP {}) for {};
    those parts of tiles stay empty`, `mapbox: {} still failing ({}) after {} attempts at tile
    14/{}/{}; showing what loaded`, `mapbox: {} answered HTTP {} at tile 14/{}/{}; that part stays
    empty`; silent for 200, 204 and 404).
  - `Cesium3DTilesSelection::TileLoadResult success(Cesium3DTilesSelection::TileContentKind content, std::optional<Cesium3DTilesSelection::BoundingVolume> bounds, const std::shared_ptr<CesiumAsync::IAssetAccessor>& accessor, const CesiumGeospatial::Ellipsoid& ellipsoid);`
    (Z-up glTF, `pCompletedRequest` left null so no tokened URL reaches the model).
  - The defaulted `ellipsoid` parameter on `tile_region` and `quadtree_children`, as the contract
    writes them: a call without it uses WGS84, the only ellipsoid any Overlume tileset uses;
    the buildings loader passes the one cesium hands `createTileChildren`, as it does today.

  What a consumer may rely on:
  - `AttemptBook` is main-thread only (cesium calls `loadTileContent` there). Per call of
    `loadTileContent` for a z14 tile: `if (book.waiting(t14, now)) return RetryLater` (no request,
    no attempt, no `retries` count), else `Attempt& a = book.begin(t14, now)` gives attempt number
    `a.count` (1, 2, 3) and the `settled` flag the worker sets when it stops retrying; attempt
    `n + 1` may start no sooner than `kRetryWaitS * 2^(n-1)` after attempt `n` started (2 s, then
    4 s). A tile whose last build settled starts again at attempt 1.
  - `classify`: a transport error, 429 and 5xx are `Transient`; 200 `Body`; 204 and 404 `Empty`;
    401 and 403 `Auth`; anything else `Other`. `transient_status` is the rule `classify` applies
    to the status; status 0 arrives only with `transport_error`.
  - `quadtree_children` builds children on `parent.getLoader()`; a tile whose id is not a
    `QuadtreeTileID`, or a z14 tile, gets none. Any other tile gets four, whatever its loader.
  - `SourceLog::first_time` is thread-safe (one mutex).

**API notes** (cesium-native 0.64.0, paths under `overlume/build/_deps/cesium-native-src/`):
- What moves, at 427f5d8 in `overlume/src/mapbox_loader.cpp`: `kMaxAttempts`, `kRetryWaitS`,
  `kRegionBelowM`, `kRegionAboveM` (122-127); `LoaderConfig` (150-156); `LoaderState`'s
  `logged_mutex`, `logged` and `first_time` (167-174); `g_testClockS` and `clock_s` (192-201);
  `Fetched`, `Outcome`, `classify`, `fetch` (203-245); `region_of` (247-254), now `tile_region`
  with the heights computed by the caller (`origin_height_m - kRegionBelowM`,
  `origin_height_m + top_m`, the same double expressions); `success` (256-270); `note_status`
  (363-384); the `attempts_` map, `Attempt` and the wait arithmetic (545-562, 612-623), now
  `AttemptBook`; the body of `createTileChildren` (583-610), now `quadtree_children`; `make_root`
  (626-641), now `quadtree_root`; `mapbox_set_test_clock` (777). What stays: the URLs, the
  extrusion and landmark build, `LoaderState`'s counters, the fixture network hooks and
  `open_mapbox_source`.
- Spec section 5 and the contract's comment name an "area selection within `radius`" / "the z14
  leaves within radius_m of the anchor". The loader has none: the radius selection is the stream
  source's area view (`CustomTileset::area_selection`, `overlume/src/environment_stream.hpp:325-332`;
  the sphere of `radiusM` around the ego, `overlume/src/environment_stream.cpp:1103-1111`), which
  a second tileset of the same source shares. Nothing moves for it.
- `Cesium3DTilesSelection/include/Cesium3DTilesSelection/Tile.h:152-154` and `:197-200`: the two
  constructors used; `:535` and `src/Tile.cpp:253-255`: `getLoader() const` returns the tile's
  non-const loader. Every tile of a Mapbox tileset is built with that tileset's loader
  (`quadtree_root` and `quadtree_children`), and
  `src/TilesetContentManager.cpp:1341-1354` calls `this->_pLoader->createTileChildren(tile, …)`
  with the manager's one loader, so `parent.getLoader()` is the `this` the old body used.
- `Cesium3DTilesSelection/include/Cesium3DTilesSelection/TilesetContentLoader.h:174-177`:
  `createTileChildren(const Tile&, const Ellipsoid& = WGS84)`; `TilesetOptions.h:287-293`: a
  tileset's ellipsoid is WGS84 unless set, and `overlume/src/environment_stream.cpp` never sets it.
- `Cesium3DTilesSelection/include/Cesium3DTilesSelection/TileLoadResult.h:38-42`:
  `TileContentKind`; `:172-186`: `createFailedResult` and `createRetryLaterResult`, which stay at
  the call sites.
- `CesiumCurl/src/CurlAssetAccessor.cpp:394-418`: a transport failure throws instead of answering
  (status 0 never comes with a response), and `MapFixtureAssetAccessor::fail_transport`
  (`overlume/src/environment_stream.hpp:194-204`) rejects the same way; `fetch` turns both into
  `transport_error = true`. So `classify` reaching `transient_status(0)` is unreachable and
  routing `classify`'s 429/5xx line through `transient_status` changes no outcome.
- `overlume/CMakeLists.txt:237-243`: `src/mapbox_*.cpp` compile as C++20 in `overlume_stream`, so
  `mapbox_tiling.cpp` joins it at the next configure; `:585-590` and `:612-621`: each
  `tests/test_*.cpp` is a C++17 executable with only `src`, stb and (for `test_mapbox_*`)
  meshoptimizer on its include path, so no test can include `mapbox_tiling.hpp` (cesium headers,
  and the C++20 `static_assert` that `mapbox_landmarks.hpp:6` also carries). The buildings tests
  pin the move instead: `test_mapbox_buildings.cpp:283-315` (`expect_retry_then_load`: no request
  while the wait runs at 1.9 s, attempt 2 at 2.0 s), `:587-618` (attempt 3 at 6.0 s, one "still
  failing … after 3 attempts" line), `:557-581` (one "Mapbox rejected MAPBOX_TOKEN" line per
  endpoint for 17 rejected requests), `:675` (four z14 tiles at a corner: the quadtree children)
  and `:462`, `:485` (only the tiles whose bounding regions meet the radius sphere are requested).
- `overlume/scripts/check_shared_exports.sh:13-15`: the shared library may export only
  `overlume::` names; every new symbol is in `overlume::mapbox`.
- Whole-binary runs of `test_mapbox_buildings` unset `OVERLUME_LIVE_MAPBOX` (`env -u`), so
  `MapboxLive.*` skips whatever the shell exports (`test_mapbox_buildings.cpp:1178-1182`); no
  step needs the network or a token.

This task changes no behaviour, so it writes no test: Step 1 records the pinned tests' green state,
Step 5 repeats it after the move, and Step 7 proves the loader now runs on the moved code.

- [ ] **Step 1: Run the pinned tests before the move**

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_mapbox_buildings test_mapbox_uri
env -u OVERLUME_LIVE_MAPBOX overlume/build/test_mapbox_buildings | tail -5
overlume/build/test_mapbox_uri | tail -1
```

Expected: the build finishes (little or nothing to do after Task 0's build), then

```
[==========] 27 tests from 3 test suites ran. (… ms total)
[  PASSED  ] 25 tests.
[  SKIPPED ] 2 tests, listed below:
[  SKIPPED ] MapboxLive.DowntownDubai
[  SKIPPED ] MapboxLive.OperatingArea
[  PASSED  ] 10 tests.
```

Every `MapboxBuildings*` test must pass here; a `[  SKIPPED ]` `no GPU/EGL` means this machine
cannot run them, so stop and run the lane on the GPU rig.

- [ ] **Step 2: Write `overlume/src/mapbox_tiling.hpp`**

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

static_assert(__cplusplus >= 202002L, "mapbox_tiling.hpp is C++20-only (overlume_stream)");

// What the Mapbox loaders share: the z14 quadtree and its bounding regions, requests that never
// reject, the status rules, the retry schedule and the per-source log dedup.

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <Cesium3DTilesSelection/BoundingVolume.h>
#include <Cesium3DTilesSelection/Tile.h>
#include <Cesium3DTilesSelection/TileLoadResult.h>
#include <Cesium3DTilesSelection/TilesetContentLoader.h>
#include <CesiumAsync/AsyncSystem.h>
#include <CesiumAsync/Future.h>
#include <CesiumAsync/IAssetAccessor.h>
#include <CesiumGeospatial/Ellipsoid.h>

#include "mapbox_tile_frame.hpp"

namespace spdlog {
class logger;
}

namespace overlume::mapbox {

inline constexpr uint32_t kMaxAttempts = 3;
// The wait before attempt 2, from attempt 1's start; doubled before attempt 3.
inline constexpr double kRetryWaitS = 2.0;
// Tile bounds span the anchor height -10 m to +1000 m until a built tile reports its real top.
inline constexpr double kRegionBelowM = 10.0;
inline constexpr double kRegionAboveM = 1000.0;

// Fixed when the source opens. Async work holds this and the loader's state by shared_ptr, never
// the loader itself, so switching the source away mid-build stays safe.
struct LoaderConfig {
    std::string token;  // goes into request URLs and nowhere else
    bool materials_original = false;
    double origin_height_m = 0.0;
};

// At most one log line per endpoint and status, or failure kind, per source.
struct SourceLog {
    bool first_time(const std::string& key);

private:
    std::mutex mutex_;
    std::set<std::string> logged_;
};

struct Attempt {
    uint32_t count = 0;
    double not_before_s = 0.0;  // clock_s() before which the next attempt may not start
    std::shared_ptr<std::atomic<bool>> settled;  // set by the build that stops retrying
};

// Attempt bookkeeping keyed by z14 tile. Main thread only: cesium calls loadTileContent there.
class AttemptBook {
public:
    // True while the tile's retry wait runs: its last attempt has not settled and now_s is before
    // that attempt's not_before_s. Cesium asks again for a RetryLater tile on its very next
    // selection; until the wait is over the answer is RetryLater again, with no request, no
    // attempt and no retries count.
    bool waiting(const TileId& t14, double now_s) const;
    // Starts the tile's next attempt: a fresh entry when there is none or the last build settled,
    // then count + 1 and not_before_s = now_s + kRetryWaitS * 2^(count - 1).
    Attempt& begin(const TileId& t14, double now_s);

private:
    // ponytail: one entry per z14 tile ever loaded; prune settled entries if drives ever span
    // thousands of tiles.
    std::unordered_map<uint64_t, Attempt> attempts_;
};

// The retry wait's clock in seconds: the steady clock, or where a test froze it
// (mapbox_set_test_clock).
double clock_s();

struct Fetched {
    uint16_t status = 0;
    std::vector<std::byte> body;  // kept for 200 only
    bool transport_error = false;
};

enum class Outcome { Body, Empty, Auth, Transient, Other };

Outcome classify(const Fetched& f);

// A status worth another attempt: 429, 5xx, or 0 (no HTTP answer: a transport error).
bool transient_status(uint16_t status);

// Never rejects: AsyncSystem::all would reject too, and cesium fails a tile whose load rejects
// for good. The transport error's text names the URL, token included, so it is dropped here.
CesiumAsync::Future<Fetched> fetch(const CesiumAsync::AsyncSystem& asyncSystem,
                                   const std::shared_ptr<CesiumAsync::IAssetAccessor>& accessor,
                                   const std::string& url);

// One line per endpoint and status per source; empty answers (204, 404) are normal and silent.
// Log lines name tiles, never URLs.
void note_status(SourceLog& seen, spdlog::logger& log, const char* endpoint, const Fetched& f,
                 const TileId& t14);

// A Success result for content in the tile's Z-up frame. pCompletedRequest stays null: cesium
// would copy that request's URL, token included, into the model.
Cesium3DTilesSelection::TileLoadResult success(
    Cesium3DTilesSelection::TileContentKind content,
    std::optional<Cesium3DTilesSelection::BoundingVolume> bounds,
    const std::shared_ptr<CesiumAsync::IAssetAccessor>& accessor,
    const CesiumGeospatial::Ellipsoid& ellipsoid);

// Tile t's bounding region from min_height_m to max_height_m above the ellipsoid.
Cesium3DTilesSelection::BoundingVolume tile_region(
    const TileId& t, double min_height_m, double max_height_m,
    const CesiumGeospatial::Ellipsoid& ellipsoid = CesiumGeospatial::Ellipsoid::WGS84);

// Web Mercator XYZ as a cesium quadtree: levels 0-13 are empty and always refined, and each
// level-14 tile carries the content. Ids carry Mapbox's XYZ y (north-down); only Mapbox loaders
// read them. The root and its single z0 child cover the Mercator world (tile_bounds of 0/0/0 is
// WebMercatorProjection::MAXIMUM_GLOBE_RECTANGLE), as LayerJsonTerrainLoader sets up EPSG:3857.
std::unique_ptr<Cesium3DTilesSelection::Tile> quadtree_root(
    Cesium3DTilesSelection::TilesetContentLoader& loader, double origin_height_m);

// The four children of a level 0-13 tile, on the parent's loader, row-major from the north-west;
// none for a z14 tile or a tile whose id is not a QuadtreeTileID. Each spans the anchor height
// -kRegionBelowM to +kRegionAboveM; a z14 child carries its tile frame as an absolute transform
// and geometric error 0.
std::vector<Cesium3DTilesSelection::Tile> quadtree_children(
    const Cesium3DTilesSelection::Tile& parent, const LoaderConfig& cfg,
    const CesiumGeospatial::Ellipsoid& ellipsoid = CesiumGeospatial::Ellipsoid::WGS84);

}
```

- [ ] **Step 3: Write `overlume/src/mapbox_tiling.cpp`**

`mapbox_set_test_clock` keeps its declaration in `mapbox_test_hooks.hpp`; its definition moves
here with the clock it sets.

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "mapbox_tiling.hpp"

#include "mapbox_test_hooks.hpp"

#include <CesiumAsync/IAssetRequest.h>
#include <CesiumAsync/IAssetResponse.h>
#include <CesiumGeometry/QuadtreeTileID.h>
#include <CesiumGeospatial/BoundingRegion.h>
#include <CesiumGeospatial/GlobeRectangle.h>

#include <glm/gtc/type_ptr.hpp>

#include <spdlog/spdlog.h>

#include <chrono>
#include <exception>
#include <span>
#include <variant>

namespace overlume::mapbox {

namespace {

using Cesium3DTilesSelection::Tile;
using Cesium3DTilesSelection::TileEmptyContent;
using Cesium3DTilesSelection::TileLoadResult;
using Cesium3DTilesSelection::TileLoadResultState;

// Where mapbox_set_test_clock froze clock_s(); negative: the steady clock.
std::atomic<double> g_testClockS{-1.0};

uint64_t attempt_key(const TileId& t14) { return (uint64_t{t14.x} << 32) | t14.y; }

}

bool SourceLog::first_time(const std::string& key) {
    std::lock_guard<std::mutex> lock(mutex_);
    return logged_.insert(key).second;
}

bool AttemptBook::waiting(const TileId& t14, double now_s) const {
    const auto it = attempts_.find(attempt_key(t14));
    if (it == attempts_.end()) return false;
    const Attempt& a = it->second;
    return a.settled && !a.settled->load() && a.count > 0 && now_s < a.not_before_s;
}

Attempt& AttemptBook::begin(const TileId& t14, double now_s) {
    Attempt& a = attempts_[attempt_key(t14)];
    if (!a.settled || a.settled->load()) {
        a = Attempt{0, 0.0, std::make_shared<std::atomic<bool>>(false)};
    }
    const uint32_t attempt = ++a.count;
    // ponytail: the wait runs from this attempt's start, so the tile's attempt state stays
    // main-thread only; an attempt slower than its wait retries at once, and a tile that
    // leaves the radius mid-wait resumes its old count when it is back (fewer attempts,
    // never more).
    a.not_before_s = now_s + kRetryWaitS * static_cast<double>(1u << (attempt - 1));
    return a;
}

double clock_s() {
    const double frozen = g_testClockS.load();
    if (frozen >= 0.0) return frozen;
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

bool transient_status(uint16_t status) { return status == 0 || status == 429 || status >= 500; }

Outcome classify(const Fetched& f) {
    if (f.transport_error) return Outcome::Transient;
    if (f.status == 200) return Outcome::Body;
    if (f.status == 204 || f.status == 404) return Outcome::Empty;
    if (f.status == 401 || f.status == 403) return Outcome::Auth;
    // Status 0 comes only with transport_error: curl and the fixture network reject instead.
    if (transient_status(f.status)) return Outcome::Transient;
    return Outcome::Other;
}

CesiumAsync::Future<Fetched> fetch(const CesiumAsync::AsyncSystem& asyncSystem,
                                   const std::shared_ptr<CesiumAsync::IAssetAccessor>& accessor,
                                   const std::string& url) {
    return accessor->get(asyncSystem, url, {})
        .thenImmediately([](std::shared_ptr<CesiumAsync::IAssetRequest>&& request) {
            Fetched f;
            const CesiumAsync::IAssetResponse* response = request ? request->response() : nullptr;
            if (response == nullptr) {
                f.transport_error = true;
                return f;
            }
            f.status = response->statusCode();
            if (f.status == 200) {
                const std::span<const std::byte> data = response->data();
                f.body.assign(data.begin(), data.end());
            }
            return f;
        })
        .catchImmediately([](std::exception&&) {
            Fetched f;
            f.transport_error = true;
            return f;
        });
}

void note_status(SourceLog& seen, spdlog::logger& log, const char* endpoint, const Fetched& f,
                 const TileId& t14) {
    const Outcome outcome = classify(f);
    if (outcome == Outcome::Body || outcome == Outcome::Empty) return;
    if (!seen.first_time(std::string(endpoint) + " " + std::to_string(f.status))) return;
    if (outcome == Outcome::Auth) {
        log.error("Mapbox rejected MAPBOX_TOKEN (HTTP {}) for {}; those parts of tiles stay empty",
                  f.status, endpoint);
    } else if (outcome == Outcome::Transient) {
        const std::string what =
            f.transport_error ? std::string("transport error") : "HTTP " + std::to_string(f.status);
        log.warn(
            "mapbox: {} still failing ({}) after {} attempts at tile 14/{}/{}; showing what "
            "loaded",
            endpoint, what, kMaxAttempts, t14.x, t14.y);
    } else {
        log.warn("mapbox: {} answered HTTP {} at tile 14/{}/{}; that part stays empty", endpoint,
                 f.status, t14.x, t14.y);
    }
}

TileLoadResult success(Cesium3DTilesSelection::TileContentKind content,
                       std::optional<Cesium3DTilesSelection::BoundingVolume> bounds,
                       const std::shared_ptr<CesiumAsync::IAssetAccessor>& accessor,
                       const CesiumGeospatial::Ellipsoid& ellipsoid) {
    TileLoadResult result{};
    result.contentKind = std::move(content);
    result.glTFUpAxis = CesiumGeometry::Axis::Z;
    result.updatedBoundingVolume = std::move(bounds);
    result.pAssetAccessor = accessor;
    result.state = TileLoadResultState::Success;
    result.ellipsoid = ellipsoid;
    return result;
}

Cesium3DTilesSelection::BoundingVolume tile_region(const TileId& t, double min_height_m,
                                                   double max_height_m,
                                                   const CesiumGeospatial::Ellipsoid& ellipsoid) {
    const LonLatRect b = tile_bounds(t);
    return CesiumGeospatial::BoundingRegion(
        CesiumGeospatial::GlobeRectangle::fromDegrees(b.west, b.south, b.east, b.north),
        min_height_m, max_height_m, ellipsoid);
}

std::unique_ptr<Tile> quadtree_root(Cesium3DTilesSelection::TilesetContentLoader& loader,
                                    double origin_height_m) {
    auto root =
        std::make_unique<Tile>(&loader, Cesium3DTilesSelection::TileID(), TileEmptyContent{});
    root->setUnconditionallyRefine();
    root->setBoundingVolume(tile_region(TileId{0, 0, 0}, origin_height_m - kRegionBelowM,
                                        origin_height_m + kRegionAboveM));
    std::vector<Tile> children;
    Tile& world =
        children.emplace_back(&loader, CesiumGeometry::QuadtreeTileID(0, 0, 0), TileEmptyContent{});
    world.setUnconditionallyRefine();
    world.setBoundingVolume(root->getBoundingVolume());
    root->createChildTiles(std::move(children));
    return root;
}

std::vector<Tile> quadtree_children(const Tile& parent, const LoaderConfig& cfg,
                                    const CesiumGeospatial::Ellipsoid& ellipsoid) {
    const auto* id = std::get_if<CesiumGeometry::QuadtreeTileID>(&parent.getTileID());
    if (id == nullptr || id->level >= kLandmarkZoom) return {};
    // Every tile of a Mapbox tileset carries its loader (quadtree_root and the lines below).
    Cesium3DTilesSelection::TilesetContentLoader* loader = parent.getLoader();
    std::vector<Tile> children;
    children.reserve(4);
    for (uint32_t row = 0; row < 2; ++row) {
        for (uint32_t col = 0; col < 2; ++col) {
            const TileId t{id->level + 1, 2 * id->x + col, 2 * id->y + row};
            const CesiumGeometry::QuadtreeTileID childId(t.z, t.x, t.y);
            const Cesium3DTilesSelection::BoundingVolume region =
                tile_region(t, cfg.origin_height_m - kRegionBelowM,
                            cfg.origin_height_m + kRegionAboveM, ellipsoid);
            if (t.z < kLandmarkZoom) {
                Tile& child = children.emplace_back(loader, childId, TileEmptyContent{});
                child.setUnconditionallyRefine();
                child.setBoundingVolume(region);
                continue;
            }
            // The transform is absolute: the z14 tile's east-north-up frame, in double.
            Tile& child = children.emplace_back(loader, childId);
            child.setTransform(
                glm::make_mat4(make_tile_frame(t, cfg.origin_height_m).frame_to_ecef.data()));
            child.setGeometricError(0.0);
            child.setBoundingVolume(region);
        }
    }
    return children;
}

}

namespace overlume::testing {

void mapbox_set_test_clock(double seconds) { overlume::mapbox::g_testClockS.store(seconds); }

}
```

- [ ] **Step 4: Move the shared code out of `overlume/src/mapbox_loader.cpp`**

Fourteen edits, top to bottom (line numbers at 427f5d8); each quoted block is unique in the file.

Edit 1 (lines 10-44), the include block (`mapbox_tiling.hpp` joins; the headers only the moved code used leave): replace

```cpp
#include "mapbox_test_hooks.hpp"
#include "mapbox_tile_frame.hpp"
#include "renderer_internal.hpp"

#include <Cesium3DTilesSelection/BoundingVolume.h>
#include <Cesium3DTilesSelection/Tile.h>
#include <Cesium3DTilesSelection/TileLoadResult.h>
#include <Cesium3DTilesSelection/TilesetContentLoader.h>
#include <CesiumAsync/IAssetRequest.h>
#include <CesiumAsync/IAssetResponse.h>
#include <CesiumCurl/CurlAssetAccessor.h>
#include <CesiumGeometry/QuadtreeTileID.h>
#include <CesiumGeospatial/BoundingRegion.h>
#include <CesiumGeospatial/Ellipsoid.h>
#include <CesiumGeospatial/GlobeRectangle.h>
#include <CesiumGltf/Model.h>
#include <CesiumUtility/Gzip.h>

#include <glm/gtc/type_ptr.hpp>

#include <spdlog/spdlog.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <limits>
#include <mutex>
#include <set>
#include <span>
#include <unordered_map>
#include <variant>
```

with

```cpp
#include "mapbox_test_hooks.hpp"
#include "mapbox_tile_frame.hpp"
#include "mapbox_tiling.hpp"
#include "renderer_internal.hpp"

#include <Cesium3DTilesSelection/BoundingVolume.h>
#include <Cesium3DTilesSelection/Tile.h>
#include <Cesium3DTilesSelection/TileLoadResult.h>
#include <Cesium3DTilesSelection/TilesetContentLoader.h>
#include <CesiumCurl/CurlAssetAccessor.h>
#include <CesiumGeometry/QuadtreeTileID.h>
#include <CesiumGeospatial/Ellipsoid.h>
#include <CesiumGltf/Model.h>
#include <CesiumUtility/Gzip.h>

#include <spdlog/spdlog.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <limits>
#include <mutex>
#include <span>
#include <variant>
```

Edit 2 (lines 122-127), the retry and region constants: delete

```cpp
constexpr uint32_t kMaxAttempts = 3;
// The wait before attempt 2, from attempt 1's start; doubled before attempt 3.
constexpr double kRetryWaitS = 2.0;
// Tile bounds span the anchor height -10 m to +1000 m until a built tile reports its real top.
constexpr double kRegionBelowM = 10.0;
constexpr double kRegionAboveM = 1000.0;
```

Edit 3 (lines 150-158), `LoaderConfig` (now in `mapbox_tiling.hpp`) and the head of `LoaderState`, which inherits the log dedup: replace

```cpp
// Fixed when the source opens. Async work holds this and LoaderState by shared_ptr, never the
// loader itself, so switching the source away mid-build stays safe.
struct LoaderConfig {
    std::string token;  // goes into request URLs and nowhere else
    bool materials_original = false;
    double origin_height_m = 0.0;
};

struct LoaderState {
```

with

```cpp
// first_time (SourceLog): at most one log line per endpoint and status, or failure kind.
struct LoaderState : SourceLog {
```

Edit 4 (lines 167-174), `LoaderState`'s own log dedup: delete

```cpp
    std::mutex logged_mutex;
    std::set<std::string> logged;

    // At most one log line per endpoint and status, or failure kind, per source.
    bool first_time(const std::string& key) {
        std::lock_guard<std::mutex> lock(logged_mutex);
        return logged.insert(key).second;
    }
```

Edit 5 (lines 192-270), the test clock, `clock_s`, `Fetched`, `Outcome`, `classify`, `fetch`, `region_of` and `success`: delete, with the blank line after it

```cpp
// The retry wait's clock in seconds: the steady clock, or where a test froze it
// (mapbox_set_test_clock).
std::atomic<double> g_testClockS{-1.0};

double clock_s() {
    const double frozen = g_testClockS.load();
    if (frozen >= 0.0) return frozen;
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

struct Fetched {
    uint16_t status = 0;
    std::vector<std::byte> body;  // kept for 200 only
    bool transport_error = false;
};

enum class Outcome { Body, Empty, Auth, Transient, Other };

Outcome classify(const Fetched& f) {
    if (f.transport_error) return Outcome::Transient;
    if (f.status == 200) return Outcome::Body;
    if (f.status == 204 || f.status == 404) return Outcome::Empty;
    if (f.status == 401 || f.status == 403) return Outcome::Auth;
    if (f.status == 429 || f.status >= 500) return Outcome::Transient;
    return Outcome::Other;
}

// Never rejects: AsyncSystem::all would reject too, and cesium fails a tile whose load rejects
// for good. The transport error's text names the URL, token included, so it is dropped here.
CesiumAsync::Future<Fetched> fetch(const CesiumAsync::AsyncSystem& asyncSystem,
                                   const std::shared_ptr<CesiumAsync::IAssetAccessor>& accessor,
                                   const std::string& url) {
    return accessor->get(asyncSystem, url, {})
        .thenImmediately([](std::shared_ptr<CesiumAsync::IAssetRequest>&& request) {
            Fetched f;
            const CesiumAsync::IAssetResponse* response = request ? request->response() : nullptr;
            if (response == nullptr) {
                f.transport_error = true;
                return f;
            }
            f.status = response->statusCode();
            if (f.status == 200) {
                const std::span<const std::byte> data = response->data();
                f.body.assign(data.begin(), data.end());
            }
            return f;
        })
        .catchImmediately([](std::exception&&) {
            Fetched f;
            f.transport_error = true;
            return f;
        });
}

Cesium3DTilesSelection::BoundingVolume region_of(const TileId& t, double origin_height_m,
                                                 double top_m,
                                                 const CesiumGeospatial::Ellipsoid& ellipsoid) {
    const LonLatRect b = tile_bounds(t);
    return CesiumGeospatial::BoundingRegion(
        CesiumGeospatial::GlobeRectangle::fromDegrees(b.west, b.south, b.east, b.north),
        origin_height_m - kRegionBelowM, origin_height_m + top_m, ellipsoid);
}

// A Success result for content in the tile's Z-up frame. pCompletedRequest stays null: cesium
// would copy that request's URL, token included, into the model.
TileLoadResult success(Cesium3DTilesSelection::TileContentKind content,
                       std::optional<Cesium3DTilesSelection::BoundingVolume> bounds,
                       const std::shared_ptr<CesiumAsync::IAssetAccessor>& accessor,
                       const CesiumGeospatial::Ellipsoid& ellipsoid) {
    TileLoadResult result{};
    result.contentKind = std::move(content);
    result.glTFUpAxis = CesiumGeometry::Axis::Z;
    result.updatedBoundingVolume = std::move(bounds);
    result.pAssetAccessor = accessor;
    result.state = TileLoadResultState::Success;
    result.ellipsoid = ellipsoid;
    return result;
}
```

Edit 6 (lines 363-384), `note_status`: delete, with the blank line after it

```cpp
// One line per endpoint and status per source; empty answers (204, 404) are normal and silent.
// Log lines name tiles, never URLs.
void note_status(LoaderState& st, spdlog::logger& log, const char* endpoint, const Fetched& f,
                 const TileId& t14) {
    const Outcome outcome = classify(f);
    if (outcome == Outcome::Body || outcome == Outcome::Empty) return;
    if (!st.first_time(std::string(endpoint) + " " + std::to_string(f.status))) return;
    if (outcome == Outcome::Auth) {
        log.error("Mapbox rejected MAPBOX_TOKEN (HTTP {}) for {}; those parts of tiles stay empty",
                  f.status, endpoint);
    } else if (outcome == Outcome::Transient) {
        const std::string what =
            f.transport_error ? std::string("transport error") : "HTTP " + std::to_string(f.status);
        log.warn(
            "mapbox: {} still failing ({}) after {} attempts at tile 14/{}/{}; showing what "
            "loaded",
            endpoint, what, kMaxAttempts, t14.x, t14.y);
    } else {
        log.warn("mapbox: {} answered HTTP {} at tile 14/{}/{}; that part stays empty", endpoint,
                 f.status, t14.x, t14.y);
    }
}
```

Edit 7 (lines 525-527), `build_tile`'s last `return` (the region's heights are now the caller's): replace

```cpp
    return success(std::move(model), region_of(t14, cfg.origin_height_m, top, ellipsoid), accessor,
                   ellipsoid);
}
```

with

```cpp
    return success(
        std::move(model),
        tile_region(t14, cfg.origin_height_m - kRegionBelowM, cfg.origin_height_m + top, ellipsoid),
        accessor, ellipsoid);
}
```

Edit 8 (lines 529-531), the loader's class comment (the quadtree's description moved to `quadtree_root`): replace

```cpp
// Web Mercator XYZ as a cesium quadtree: levels 0-13 are empty and always refined, and each
// level-14 tile carries the buildings. Ids carry Mapbox's XYZ y (north-down); only this loader
// reads them.
```

with

```cpp
// The buildings on mapbox_tiling's z14 quadtree: each level-14 tile carries one tile's
// footprints and landmarks.
```

Edit 9 (lines 545-562), the attempt bookkeeping in `loadTileContent` (the comment on waiting moved to `AttemptBook::waiting`): replace

```cpp
        const TileId t14{kLandmarkZoom, id->x, id->y};
        Attempt& a = attempts_[(uint64_t{id->x} << 32) | id->y];
        if (!a.settled || a.settled->load()) {
            a = Attempt{0, 0.0, std::make_shared<std::atomic<bool>>(false)};
        }
        // Cesium asks again for a RetryLater tile on its very next selection. Until the wait is
        // over the answer is RetryLater again: no request, no attempt, no retries count.
        const double now = clock_s();
        if (a.count > 0 && now < a.not_before_s) {
            return input.asyncSystem.createResolvedFuture(
                TileLoadResult::createRetryLaterResult(input.pAssetAccessor, nullptr));
        }
        const uint32_t attempt = ++a.count;
        // ponytail: the wait runs from this attempt's start, so the tile's attempt state stays
        // main-thread only; an attempt slower than its wait retries at once, and a tile that
        // leaves the radius mid-wait resumes its old count when it is back (fewer attempts,
        // never more).
        a.not_before_s = now + kRetryWaitS * static_cast<double>(1u << (attempt - 1));
```

with

```cpp
        const TileId t14{kLandmarkZoom, id->x, id->y};
        const double now = clock_s();
        if (attempts_.waiting(t14, now)) {
            return input.asyncSystem.createResolvedFuture(
                TileLoadResult::createRetryLaterResult(input.pAssetAccessor, nullptr));
        }
        const Attempt& a = attempts_.begin(t14, now);
        const uint32_t attempt = a.count;
```

Edit 10 (lines 583-610), `createTileChildren`: replace

```cpp
    TileChildrenResult createTileChildren(const Tile& tile,
                                          const CesiumGeospatial::Ellipsoid& ellipsoid) override {
        const auto* id = std::get_if<CesiumGeometry::QuadtreeTileID>(&tile.getTileID());
        if (id == nullptr || id->level >= kLandmarkZoom) return {{}, TileLoadResultState::Success};
        std::vector<Tile> children;
        children.reserve(4);
        for (uint32_t row = 0; row < 2; ++row) {
            for (uint32_t col = 0; col < 2; ++col) {
                const TileId t{id->level + 1, 2 * id->x + col, 2 * id->y + row};
                const CesiumGeometry::QuadtreeTileID childId(t.z, t.x, t.y);
                const Cesium3DTilesSelection::BoundingVolume region =
                    region_of(t, config_->origin_height_m, kRegionAboveM, ellipsoid);
                if (t.z < kLandmarkZoom) {
                    Tile& child = children.emplace_back(this, childId, TileEmptyContent{});
                    child.setUnconditionallyRefine();
                    child.setBoundingVolume(region);
                    continue;
                }
                // The transform is absolute: the z14 tile's east-north-up frame, in double.
                Tile& child = children.emplace_back(this, childId);
                child.setTransform(glm::make_mat4(
                    make_tile_frame(t, config_->origin_height_m).frame_to_ecef.data()));
                child.setGeometricError(0.0);
                child.setBoundingVolume(region);
            }
        }
        return {std::move(children), TileLoadResultState::Success};
    }
```

with

```cpp
    TileChildrenResult createTileChildren(const Tile& tile,
                                          const CesiumGeospatial::Ellipsoid& ellipsoid) override {
        return {quadtree_children(tile, *config_, ellipsoid), TileLoadResultState::Success};
    }
```

Edit 11 (lines 612-624), the private members: replace

```cpp
private:
    struct Attempt {
        uint32_t count = 0;
        double not_before_s = 0.0;  // clock_s() before which the next attempt may not start
        std::shared_ptr<std::atomic<bool>> settled;  // set by the build that stops retrying
    };

    std::shared_ptr<const LoaderConfig> config_;
    std::shared_ptr<LoaderState> state_;
    // Main thread only: cesium calls loadTileContent there. ponytail: one entry per z14 tile ever
    // loaded; prune settled entries if drives ever span thousands of tiles.
    std::unordered_map<uint64_t, Attempt> attempts_;
};
```

with

```cpp
private:
    std::shared_ptr<const LoaderConfig> config_;
    std::shared_ptr<LoaderState> state_;
    AttemptBook attempts_;  // main thread only: cesium calls loadTileContent there
};
```

Edit 12 (lines 626-641), `make_root` (now `quadtree_root`): delete, with the blank line after it

```cpp
// The root and its single z0 child cover the Mercator world (tile_bounds of 0/0/0 is
// WebMercatorProjection::MAXIMUM_GLOBE_RECTANGLE), as LayerJsonTerrainLoader sets up EPSG:3857.
std::unique_ptr<Tile> make_root(MapboxBuildingsLoader& loader, double origin_height_m) {
    auto root =
        std::make_unique<Tile>(&loader, Cesium3DTilesSelection::TileID(), TileEmptyContent{});
    root->setUnconditionallyRefine();
    root->setBoundingVolume(region_of(TileId{0, 0, 0}, origin_height_m, kRegionAboveM,
                                      CesiumGeospatial::Ellipsoid::WGS84));
    std::vector<Tile> children;
    Tile& world =
        children.emplace_back(&loader, CesiumGeometry::QuadtreeTileID(0, 0, 0), TileEmptyContent{});
    world.setUnconditionallyRefine();
    world.setBoundingVolume(root->getBoundingVolume());
    root->createChildTiles(std::move(children));
    return root;
}
```

Edit 13 (line 680), the root's construction in `open_mapbox_source`: replace

```cpp
    std::unique_ptr<Tile> root = make_root(*loader, anchor.origin_height_m);
```

with

```cpp
    std::unique_ptr<Tile> root = quadtree_root(*loader, anchor.origin_height_m);
```

Edit 14 (line 777), `mapbox_set_test_clock` (defined in `mapbox_tiling.cpp` now): delete, with the blank line above it

```cpp
void mapbox_set_test_clock(double seconds) { overlume::mapbox::g_testClockS.store(seconds); }
```

- [ ] **Step 5: Reconfigure, build, and run the pinned tests again**

The new `.cpp` needs a configure (the `src/mapbox_*.cpp` glob), which holds the gate lock:

```bash
cd "$WORKTREE"
flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock cmake -S overlume -B overlume/build > /dev/null \
    && echo reconfigured \
    && CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
```

No `reconfigured` line and no CMake error above it means the lock wait ran out (exit 75) and nothing
was built: run the step again. Then:

```bash
cd "$WORKTREE"
env -u OVERLUME_LIVE_MAPBOX overlume/build/test_mapbox_buildings | tail -5
overlume/build/test_mapbox_uri | tail -1
grep -nE 'region_of|make_root|struct Attempt|g_testClockS|logged_mutex|struct LoaderConfig|Outcome classify' \
    overlume/src/mapbox_loader.cpp; echo "grep exit $?"
git status --short overlume/tests
```

Expected: `reconfigured` and a clean build; the same six lines as Step 1 (25 passed, the two
`MapboxLive` skips, then `[  PASSED  ] 10 tests.`); `grep exit 1` with no line above it (nothing
of the moved code is left in the loader); `git status --short overlume/tests` prints nothing (both
test files byte-identical). A link error naming `overlume::mapbox::…` means `mapbox_tiling.cpp`
was not picked up: the configure did not run.

- [ ] **Step 6: Format**

```bash
cd "$WORKTREE"
git add overlume/src/mapbox_tiling.hpp overlume/src/mapbox_tiling.cpp overlume/src/mapbox_loader.cpp
tools/check_format.sh --fix overlume/src/mapbox_tiling.hpp overlume/src/mapbox_tiling.cpp \
    overlume/src/mapbox_loader.cpp
git diff --stat
git add overlume/src/mapbox_tiling.hpp overlume/src/mapbox_tiling.cpp overlume/src/mapbox_loader.cpp
tools/check_format.sh && tools/check_spdx.sh
```

Expected: `check_format.sh: reformatted 3 files with clang-format 23.1.1`; `git diff --stat` prints
nothing (the code above is clang-format 23.1.1's output, so a diff means a line was typed
differently, and the second `git add` stages the fix); then
`check_format.sh: PASS (217 files, clang-format 23.1.1)` and
`check_spdx.sh: PASS (375 files carry the SPDX identifier)` (215 and 373 at the lane's fork, plus
these two).

- [ ] **Step 7: Revert checks (do not commit)**: the loader must run on the moved schedule and
  status rule

First the schedule: a 1 s first wait lets attempt 2 start at 1.9 s, where the test wants none.

```bash
cd "$WORKTREE"
sed -i 's/inline constexpr double kRetryWaitS = 2.0;/inline constexpr double kRetryWaitS = 1.0;/' \
    overlume/src/mapbox_tiling.hpp
git diff --stat overlume/src/mapbox_tiling.hpp
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_mapbox_buildings
overlume/build/test_mapbox_buildings --gtest_filter='MapboxBuildings.ServerErrorRetriesThenLoads' \
    | tail -4
git checkout -- overlume/src/mapbox_tiling.hpp && git diff --stat overlume/src/mapbox_tiling.hpp
```

Expected: the first diff reports `1 file changed, 1 insertion(+), 1 deletion(-)`; the run fails at
"no request while the retry waits" (`times_requested(n, square0)` is 2, wanted 1) and ends

```
[  FAILED  ] 1 test, listed below:
[  FAILED  ] MapboxBuildings.ServerErrorRetriesThenLoads

 1 FAILED TEST
```

and after the checkout the last diff prints nothing. Then the status rule: with 503 no longer
transient, the tile builds at attempt 1 and never retries, while a transport error still retries.

```bash
cd "$WORKTREE"
sed -i 's/return status == 0 || status == 429 || status >= 500;/return status == 0;/' \
    overlume/src/mapbox_tiling.cpp
git diff --stat overlume/src/mapbox_tiling.cpp
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_mapbox_buildings
overlume/build/test_mapbox_buildings \
    --gtest_filter='MapboxBuildings.ServerErrorRetriesThenLoads:MapboxBuildings.TransportErrorRetriesThenLoads' \
    | tail -5
git checkout -- overlume/src/mapbox_tiling.cpp && git diff --stat overlume/src/mapbox_tiling.cpp
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_mapbox_buildings
```

Expected: `1 file changed, 1 insertion(+), 1 deletion(-)`; the run fails on "a transient answer
must send the tile back for another attempt" and ends

```
[  PASSED  ] 1 test.
[  FAILED  ] 1 test, listed below:
[  FAILED  ] MapboxBuildings.ServerErrorRetriesThenLoads

 1 FAILED TEST
```

After the checkout the diff prints nothing, and the last build leaves the binary current.

- [ ] **Step 8: Gate** (foreground, Bash timeout 600000 ms)

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
ROS_DOMAIN_ID=82 CI_VISUAL_MODE_DOMAIN_ID=82 \
    flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock tools/ci_visual_mode.sh
```

Expected (the baseline, unchanged: this task adds no test):
- `PASS  POD header check`
- `PASS  library ctest suite  (100% tests passed, 0 tests failed out of 442 (424 ok / 8 skipped))`
- `PASS  node gtests  (Summary: 381 tests, 0 errors, 0 failures, 0 skipped)`
- `PASS  WS bridge pytest suite  (94 passed in …)`, the time varying
- `PASS  golden suite: 25 ok, 0 skipped`
- `PASS  examples  (6 run, all exited 0 with non-empty output)`
- `OVERALL: PASS`, exit 0.

Exit 75 means another lane held the lock for 300 s: run the same command again. Any other non-zero
exit is a gate failure. A WS bridge failure can also come from another session's gate on the same
ports, so rerun before treating it as a finding.

- [ ] **Step 9: Commit**

```bash
cd "$WORKTREE"
git add overlume/src/mapbox_tiling.hpp overlume/src/mapbox_tiling.cpp overlume/src/mapbox_loader.cpp
git status --short
git diff --cached --stat | tail -1
git commit -F - <<'EOF'
refactor(mapbox): move the z14 tiling and retry plumbing to mapbox_tiling

The Mapbox terrain loader stands on the same z14 quadtree, requests,
status rules, retry schedule and log dedup as the buildings loader, so
they leave mapbox_loader.cpp for mapbox_tiling.{hpp,cpp} unchanged: the
quadtree root and children (quadtree_root, quadtree_children) and their
bounding regions (tile_region), the never-rejecting fetch with classify
and note_status, success, the three attempts with kRetryWaitS doubling
(AttemptBook, clock_s and the test clock behind mapbox_set_test_clock),
and SourceLog's first_time, which LoaderState now inherits. classify's
429/5xx rule becomes transient_status (429, 5xx, 0), for the DEM cache
to share (cesium handles the imagery overlay's failed tiles itself).

No behaviour changes: test_mapbox_buildings.cpp and test_mapbox_uri.cpp
are untouched and pass, and the gate counts are the fork's.

Claude-Session: <the executing session's URL>
EOF
```

Expected: `git status --short` lists exactly `M  overlume/src/mapbox_loader.cpp`,
`A  overlume/src/mapbox_tiling.cpp` and `A  overlume/src/mapbox_tiling.hpp`; the stat ends
`3 files changed, 365 insertions(+), 206 deletions(-)`; the commit succeeds.

---

### Task 4: URI keys: layer list, `imagery`, ion `terrain`/`imagery`

**Lane:** 2. **Depends on:** Task 0; runs after Task 3 in this lane (the `mapbox_loader.cpp` line
numbers and the counts below assume Task 3's commit).

**Worktree:** `$WORKTREE` is `$INTEGRATION-l2`, as in Task 3.

**Files:**
- Modify: `overlume/src/mapbox_loader.hpp` (`struct MapboxSpec`, 13-18; the comment above
  `parse_mapbox_spec`, 26-29)
- Modify: `overlume/src/mapbox_loader.cpp` (`parse_mapbox_spec`, 58-100 after Task 3, which is
  65-107 at 427f5d8). The parser lives here (File map row "3, 4, 9, 10"); Task 3 is this lane's,
  so no merge can conflict.
- Modify: `overlume/src/environment_stream.cpp`, three places and nothing else: `struct IonSpec`
  and `parse_ion_spec`'s key loop and `return` (63-75, 172-182); the refusal line at the top of
  `open_streaming_environment_source` (1267-1268, spec section 5's "one log line"); and one hook
  inserted directly after `environment_stream_parse_radius` (ends at 1647).
- Modify: `overlume/src/environment_test_hooks.hpp` (one declaration inserted directly after
  `environment_stream_parse_replaces_ground`, line 70, as head.md's File map row and contract
  comment place it). Lane 3 runs at the same time and edits this file and `environment_stream.cpp`'s `overlume::testing` block, but its
  anchors are pinned away from these hunks: Task 5 appends its hooks at the end of each (after
  `renderer_ground_plane_in_scene`, `environment_test_hooks.hpp:107`, and after
  `drive_ion_token_redaction_probe`, `environment_stream.cpp:1734-1749`), with nothing near
  `open_streaming_environment_source` or the parse hooks, and its tests appended at the end of
  `test_environment_stream.cpp`, where this task adds none. The merge stays clean.
- Test: `overlume/tests/test_mapbox_uri.cpp` (`LayerListMustBeExactlyBuildings`, 85-94, replaced by
  six tests; suite `MapboxUri` goes from 10 to 15 tests)
- Test: `overlume/tests/test_environment_stream.cpp` (four tests between `ParseIonSpecRadius`,
  808-826, and `TerrainPlaneFitProbe`, 828)

**Interfaces:**
- Consumes: nothing new (`parse_positive`, `mapbox_loader.cpp:44-54` after Task 3; the existing
  parse hooks' shape, `environment_stream.cpp:1609-1647`; `make_redacting_logger()`,
  `environment_stream.hpp:317`; the test hook `captured_cesium_log_text()`,
  `environment_test_hooks.hpp:101`).
- Produces (Interface contract, `overlume/src/mapbox_loader.hpp`, namespace `overlume::mapbox`):

```cpp
struct MapboxSpec {
    bool buildings = true;           // the layer list carries `buildings`
    bool terrain = false;            // the layer list carries `terrain`
    bool imagery_satellite = false;  // imagery=satellite (requires terrain)
    bool materials_original = false;
    double radius_m = kDefaultRadiusM;  // 500.0
    double brightness = 1.0;
    std::string fallback_dir;
};
MapboxSpecResult parse_mapbox_spec(const std::string& spec);
```

  The refusals, each `error` exactly (`<name>` and `<value>` are the URI's own text):
  - an empty, repeated or unknown layer name: `mapbox:// supports only the layers 'buildings' and
    'terrain', not '<name>'` (the contract's one text for all three, so `buildings,buildings`
    reads "not 'buildings'": the second mention is the one refused);
  - `imagery=` other than `satellite` or `off`: `mapbox:// imagery='<value>' is not 'satellite' or 'off'`;
  - `imagery=satellite`, as the last `imagery` value, without the `terrain` layer:
    `mapbox:// imagery=satellite needs the 'terrain' layer`;
  - `radius`, `brightness` and `cache` as today. No refusal contains the word `refused` (the
    loader's own log line adds it once, `test_mapbox_buildings.cpp:424-426`).

  And in `overlume/src/environment_stream.cpp` (internal) plus
  `overlume/src/environment_test_hooks.hpp`, namespace `overlume::testing`:

```cpp
std::optional<int64_t> terrain_asset;  // IonSpec: terrain=<id>|off (default off)
std::optional<int64_t> imagery_asset;  // IonSpec: imagery=<id>|off (default off); needs terrain_asset
// ion:// terrain=<id>|off and imagery=<id>|off: each asset id, 0 when off or absent or when the
// spec does not parse (*ok false); returns whether the spec parses with a terrain asset.
bool environment_stream_parse_terrain_assets(const char* ion_spec, int64_t* terrain,
                                             int64_t* imagery, bool* ok);
```

  The return value mirrors `environment_stream_parse_follow_terrain` (the value, and the parse
  result in the out flag); every pointer may be null.

  An `ion://` spec that does not parse now logs one line before `open_streaming_environment_source`
  returns nullptr, through the redacting `overlume.cesium` logger (the one
  `captured_cesium_log_text` reads), shaped as the `mapbox://` loader's `mapbox://{} refused: {}`:

```
ion://<spec> refused: the asset id or a key has a bad value (terrain= and imagery= take a positive asset id or off; imagery= needs terrain=)
```

  What a consumer (Tasks 6, 9, 10, 11) may rely on: `buildings` is false only when the list is
  exactly `terrain`; `imagery_satellite` implies `terrain`; `imagery_asset` implies
  `terrain_asset`; an asset id is a positive integer written in digits only; the last value of a
  repeated key wins and an earlier bad value still refuses. Until Tasks 9 and 10 wire the fields,
  `open_mapbox_source` reads none of the three new ones (`mapbox://terrain` opens the buildings
  source as `mapbox://buildings` does) and the ion source reads neither asset.

**API notes:**
- `parse_mapbox_spec` (`overlume/src/mapbox_loader.cpp:58-100` after Task 3): the layer list is
  `spec.substr(0, spec.find('?'))`, so the comma split runs on that text only; the query loop
  ignores unknown keys, which keeps `terrain=` (an ion key) meaningless on `mapbox://`, and returns
  at the first bad value of a known key. The new `imagery` branch follows that rule; the terrain
  requirement is checked after the loop, on the last value.
- `MapboxSpec`'s fields are only ever named (`MapboxSpec out;` in the parser; `spec.materials_original`,
  `spec.radius_m`, `spec.brightness`, `spec.fallback_dir` in `open_mapbox_source`, 427f5d8
  `mapbox_loader.cpp:660-685`), never aggregate-initialised, so new leading fields break no
  initialiser. The parser sets `out.buildings = false` before reading the list; the struct's
  `true` default serves code that builds a `MapboxSpec` by hand.
- `open_mapbox_source` logs a refusal as `mapbox://{} refused: {}` (427f5d8
  `mapbox_loader.cpp:651`), and `MapboxBuildings.RefusedUrisOpenNothing`
  (`test_mapbox_buildings.cpp:413-435`) wants `refused` once per refused URI, so no parser text may
  say it.
- `parse_ion_spec` (`overlume/src/environment_stream.cpp:77-182`) returns `std::nullopt` for any
  bad value, and today `open_streaming_environment_source` (`:1265-1268`) then returns nullptr
  without a log line; `set_environment_source` (`overlume/src/environment.cpp:186-205`) adds none.
  Spec section 5 wants "the parse fails, one log line, opening returns false", and once the parser
  refuses `ion://96188?imagery=2` a silent refusal would leave the user guessing, so this task adds
  the line. The parser stays pure (the `environment_stream_parse_*` hooks call it for every probe
  and must not log); the line goes in `open_streaming_environment_source`, the one caller that
  opens. The spec text carries no token (`CESIUM_ION_TOKEN` is read from the environment after the
  parse), and the redacting logger (`make_redacting_logger`, `environment_stream.hpp:317`) masks an
  `access_token=` anyway. The token check comes after the parse, so a test with the token unset
  sees the line for a refused spec and no line for one that parses.
- `captured_cesium_log_text` (`environment_stream.cpp:1729-1732`) returns everything the redacting
  logger printed in this process, up to 64 KiB (`kMaxCapturedBytes`); the test reads what was added
  after each call, as `test_mapbox_buildings.cpp:229-234` (`log_size`, `log_since`) does. Each
  ctest entry is its own process, so the gate never meets the cap.
- `std::stoll` skips leading whitespace and takes a sign, so `" 1"` and `"+1"` would parse; the
  digits-only check runs first. Twenty digits overflow and throw `std::out_of_range`, which the
  `catch` turns into a failed parse.
- The node appends `follow_terrain=on|off` to every ion URI it composes
  (`ros/src/overlume_ros/include/overlume_ros/environment_source_uri.hpp:47-49`); the key match is
  exact (`key == "terrain"`), and a test case pins that `follow_terrain=on` sets no terrain asset.
- `overlume/src/environment_test_hooks.hpp:6` already includes `<cstdint>`; the test file reaches
  `int64_t` through it.
- Tests compile as C++17 against googletest 1.15.2 (`overlume/CMakeLists.txt:612-621`);
  `ASSERT_TRUE` on a `std::optional` compiles, and gmock is off, so substring checks use the file's
  `mentions` helper.

- [ ] **Step 1: Plumbing: the new fields and the hook**

Plumbing first, so that Step 3's red run fails on behaviour, not on a missing symbol.

In `overlume/src/mapbox_loader.hpp`, replace

```cpp
struct MapboxSpec {
    bool materials_original = false;
    double radius_m = kDefaultRadiusM;
    double brightness = 1.0;
    std::string fallback_dir;
};
```

with

```cpp
struct MapboxSpec {
    bool buildings = true;           // the layer list carries `buildings`
    bool terrain = false;            // the layer list carries `terrain`
    bool imagery_satellite = false;  // imagery=satellite (requires terrain)
    bool materials_original = false;
    double radius_m = kDefaultRadiusM;
    double brightness = 1.0;
    std::string fallback_dir;
};
```

In `overlume/src/mapbox_loader.hpp`, replace

```cpp
// Parses the text after "mapbox://": the layer list (only "buildings"), then an optional query
// split at the first '?', pairs on '&', the key at the first '='. Unknown keys and pairs without
// '=' are ignored, the last value of a repeated key wins, and a bad value for a known key refuses
// the whole URI. Pure: no logging, no environment access.
```

with

```cpp
// Parses the text after "mapbox://": the layer list, a comma list of `buildings` and `terrain`,
// each at most once, in any order (an empty, repeated or unknown name refuses the URI), then an
// optional query split at the first '?', pairs on '&', the key at the first '='. Unknown keys and
// pairs without '=' are ignored, the last value of a repeated key wins, and a bad value for a
// known key refuses the whole URI. imagery=satellite needs the `terrain` layer; imagery=off is the
// default. Every refusal's error starts with "mapbox://". Pure: no logging, no environment access.
```

In `overlume/src/environment_stream.cpp`, replace

```cpp
    std::optional<double> radius_m;
};
```

with

```cpp
    std::optional<double> radius_m;
    std::optional<int64_t> terrain_asset;  // terrain=<id>|off (default off)
    std::optional<int64_t> imagery_asset;  // imagery=<id>|off (default off); needs terrain_asset
};
```

In `overlume/src/environment_stream.cpp`, replace

```cpp
    return spec.has_value() ? spec->radius_m.value_or(kDefaultTileRadiusM) : kDefaultTileRadiusM;
}
```

with

```cpp
    return spec.has_value() ? spec->radius_m.value_or(kDefaultTileRadiusM) : kDefaultTileRadiusM;
}

bool environment_stream_parse_terrain_assets(const char* ion_spec, int64_t* terrain,
                                             int64_t* imagery, bool* ok) {
    const std::optional<IonSpec> spec = parse_ion_spec(ion_spec ? ion_spec : "");
    if (ok) *ok = spec.has_value();
    if (terrain) *terrain = spec.has_value() ? spec->terrain_asset.value_or(0) : 0;
    if (imagery) *imagery = spec.has_value() ? spec->imagery_asset.value_or(0) : 0;
    return spec.has_value() && spec->terrain_asset.has_value();
}
```

In `overlume/src/environment_test_hooks.hpp`, replace

```cpp
bool environment_stream_parse_replaces_ground(const char* ion_spec, bool* out_parse_ok);
```

with

```cpp
bool environment_stream_parse_replaces_ground(const char* ion_spec, bool* out_parse_ok);

// ion:// terrain=<id>|off and imagery=<id>|off: each asset id, 0 when off or absent or when the
// spec does not parse (*ok false); returns whether the spec parses with a terrain asset.
bool environment_stream_parse_terrain_assets(const char* ion_spec, int64_t* terrain,
                                             int64_t* imagery, bool* ok);
```

- [ ] **Step 2: Write the failing tests**

In `overlume/tests/test_mapbox_uri.cpp`, replace

```cpp
TEST(MapboxUri, LayerListMustBeExactlyBuildings) {
    for (const char* spec : {"", "terrain", "buildings,terrain", "building", "Buildings",
                             "buildings/", "?radius=5", "terrain?radius=5"}) {
        const MapboxSpecResult r = parse_mapbox_spec(spec);
        EXPECT_FALSE(r.spec) << "'" << spec << "'";
        EXPECT_EQ(r.error.rfind("mapbox://", 0), 0u) << "'" << spec << "': " << r.error;
        // Quoted, so an echo of "buildings,terrain" or "buildings/" does not count.
        EXPECT_TRUE(mentions(r.error, "'buildings'")) << "'" << spec << "': " << r.error;
    }
}
```

with

```cpp
TEST(MapboxUri, LayerListInAnyOrderSetsTheLayers) {
    struct Case {
        const char* spec;
        bool buildings, terrain;
        double radius_m;
    };
    for (const Case& c :
         {Case{"buildings", true, false, 500.0}, Case{"terrain", false, true, 500.0},
          Case{"buildings,terrain", true, true, 500.0},
          Case{"terrain,buildings", true, true, 500.0},
          Case{"terrain,buildings?radius=250", true, true, 250.0},
          Case{"terrain?", false, true, 500.0}}) {
        const MapboxSpecResult r = parse_mapbox_spec(c.spec);
        ASSERT_TRUE(r.spec) << c.spec << ": " << r.error;
        EXPECT_EQ(r.spec->buildings, c.buildings) << c.spec;
        EXPECT_EQ(r.spec->terrain, c.terrain) << c.spec;
        EXPECT_FALSE(r.spec->imagery_satellite) << c.spec << ": imagery defaults to off";
        EXPECT_EQ(r.spec->radius_m, c.radius_m) << c.spec << ": the list ends at the first '?'";
        EXPECT_EQ(r.error, "") << c.spec;
    }
    const MapboxSpecResult key = parse_mapbox_spec("buildings?terrain=on");
    ASSERT_TRUE(key.spec) << key.error;
    EXPECT_FALSE(key.spec->terrain) << "layers come from the list; terrain= is an ion:// key";
}

TEST(MapboxUri, EmptyOrUnknownLayerNamesRefuseQuotingTheName) {
    const char* const cases[][2] = {
        {"", "''"},
        {"?radius=5", "''"},
        {"buildings,", "''"},
        {",terrain", "''"},
        {"buildings,,terrain", "''"},
        {"building", "'building'"},
        {"Buildings", "'Buildings'"},
        {"buildings/", "'buildings/'"},
        {"buildings, terrain", "' terrain'"},
        {"terrain,roads", "'roads'"},
        {"buildings;terrain", "'buildings;terrain'"},
    };
    for (const auto& [spec, name] : cases) {
        const MapboxSpecResult r = parse_mapbox_spec(spec);
        EXPECT_FALSE(r.spec) << "'" << spec << "'";
        EXPECT_EQ(r.error.rfind("mapbox://", 0), 0u) << "'" << spec << "': " << r.error;
        EXPECT_TRUE(mentions(r.error, "'buildings' and 'terrain'"))
            << "'" << spec << "': " << r.error;
        EXPECT_TRUE(mentions(r.error, std::string("not ") + name))
            << "'" << spec << "': " << r.error;
        EXPECT_FALSE(mentions(r.error, "refused")) << "the loader's log line says refused";
    }
}

TEST(MapboxUri, RepeatedLayerNamesRefuse) {
    const char* const cases[][2] = {
        {"buildings,buildings", "'buildings'"},
        {"terrain,terrain", "'terrain'"},
        {"buildings,terrain,buildings", "'buildings'"},
        {"terrain,buildings,terrain?imagery=satellite", "'terrain'"},
    };
    for (const auto& [spec, name] : cases) {
        const MapboxSpecResult r = parse_mapbox_spec(spec);
        EXPECT_FALSE(r.spec) << spec;
        EXPECT_EQ(r.error.rfind("mapbox://", 0), 0u) << spec << ": " << r.error;
        // The contract's one text for empty, repeated and unknown names.
        EXPECT_TRUE(mentions(r.error, "'buildings' and 'terrain'")) << spec << ": " << r.error;
        EXPECT_TRUE(mentions(r.error, std::string("not ") + name)) << spec << ": " << r.error;
        EXPECT_FALSE(mentions(r.error, "refused")) << spec << ": " << r.error;
    }
}

TEST(MapboxUri, ImagerySatelliteNeedsTheTerrainLayer) {
    for (const char* spec : {"terrain?imagery=satellite", "buildings,terrain?imagery=satellite",
                             "terrain,buildings?materials=original&imagery=satellite"}) {
        const MapboxSpecResult r = parse_mapbox_spec(spec);
        ASSERT_TRUE(r.spec) << spec << ": " << r.error;
        EXPECT_TRUE(r.spec->imagery_satellite) << spec;
    }
    for (const char* spec :
         {"buildings?imagery=satellite", "buildings?imagery=off&imagery=satellite"}) {
        const MapboxSpecResult r = parse_mapbox_spec(spec);
        EXPECT_FALSE(r.spec) << spec;
        EXPECT_EQ(r.error.rfind("mapbox://", 0), 0u) << spec << ": " << r.error;
        EXPECT_TRUE(mentions(r.error, "imagery")) << spec << ": " << r.error;
        EXPECT_TRUE(mentions(r.error, "'terrain'")) << spec << ": " << r.error;
        EXPECT_FALSE(mentions(r.error, "refused")) << spec << ": " << r.error;
    }
}

TEST(MapboxUri, ImageryOffNeedsNoTerrainAndTheLastValueWins) {
    for (const char* spec :
         {"buildings?imagery=off", "buildings,terrain?imagery=off",
          "terrain?imagery=satellite&imagery=off", "buildings?imagery=satellite&imagery=off"}) {
        const MapboxSpecResult r = parse_mapbox_spec(spec);
        ASSERT_TRUE(r.spec) << spec << ": " << r.error;
        EXPECT_FALSE(r.spec->imagery_satellite) << spec;
        EXPECT_EQ(r.warning, "") << spec;
    }
}

TEST(MapboxUri, UnknownImageryValuesRefuseNamingTheKey) {
    for (const char* spec :
         {"terrain?imagery=", "terrain?imagery=bogus", "terrain?imagery=Satellite",
          "terrain?imagery=on", "terrain?imagery=2", "terrain?imagery=bogus&imagery=satellite"}) {
        const MapboxSpecResult r = parse_mapbox_spec(spec);
        EXPECT_FALSE(r.spec) << spec;
        EXPECT_EQ(r.error.rfind("mapbox://", 0), 0u) << spec << ": " << r.error;
        EXPECT_TRUE(mentions(r.error, "imagery")) << spec << ": " << r.error;
        EXPECT_FALSE(mentions(r.error, "refused")) << spec << ": " << r.error;
    }
}
```

In `overlume/tests/test_environment_stream.cpp`, replace

```cpp
TEST(EnvironmentStream, TerrainPlaneFitProbe) {
```

with

```cpp
TEST(EnvironmentStream, ParseIonSpecTerrainAndImageryAssets) {
    struct Case {
        const char* spec;
        int64_t terrain, imagery;
    };
    for (const Case& c :
         {Case{"96188", 0, 0}, Case{"96188?terrain=off&imagery=off", 0, 0},
          Case{"96188?terrain=1", 1, 0}, Case{"96188?terrain=1&imagery=2", 1, 2},
          Case{"96188?imagery=2&terrain=1", 1, 2}, Case{"96188?terrain=1&imagery=off", 1, 0},
          Case{"96188?terrain=1&terrain=off", 0, 0}, Case{"96188?terrain=off&terrain=3", 3, 0},
          Case{"96188?terrain=1&imagery=2&imagery=3", 1, 3},
          Case{"96188?follow_terrain=on", 0, 0}}) {
        int64_t terrain = -1;
        int64_t imagery = -1;
        bool ok = false;
        const bool on = overlume::testing::environment_stream_parse_terrain_assets(c.spec, &terrain,
                                                                                   &imagery, &ok);
        EXPECT_TRUE(ok) << c.spec;
        EXPECT_EQ(on, c.terrain != 0) << c.spec;
        EXPECT_EQ(terrain, c.terrain) << c.spec;
        EXPECT_EQ(imagery, c.imagery) << c.spec;
    }
    bool ok = false;
    EXPECT_DOUBLE_EQ(overlume::testing::environment_stream_parse_radius(
                         "96188?terrain=1&imagery=2&radius=250", &ok),
                     250.0)
        << "the other keys still parse beside terrain and imagery";
    EXPECT_TRUE(ok);
}

TEST(EnvironmentStream, ParseIonSpecImageryNeedsTerrain) {
    for (const char* spec : {"96188?imagery=2", "96188?imagery=2&terrain=off",
                             "96188?terrain=1&imagery=2&terrain=off"}) {
        int64_t terrain = -1;
        int64_t imagery = -1;
        bool ok = true;
        EXPECT_FALSE(overlume::testing::environment_stream_parse_terrain_assets(spec, &terrain,
                                                                                &imagery, &ok))
            << spec;
        EXPECT_FALSE(ok) << spec << ": imagery without terrain fails the whole parse";
        EXPECT_EQ(terrain, 0) << spec;
        EXPECT_EQ(imagery, 0) << spec;
    }
    bool ok = false;
    overlume::testing::environment_stream_parse_terrain_assets("96188?imagery=off", nullptr,
                                                               nullptr, &ok);
    EXPECT_TRUE(ok) << "imagery=off needs no terrain";
}

TEST(EnvironmentStream, ParseIonSpecBadAssetIdFailsTheWholeParse) {
    for (const char* value :
         {"", "0", "-1", "+1", " 1", "1.5", "1x", "abc", "on", "OFF", "99999999999999999999"}) {
        for (const std::string& spec : {std::string("96188?terrain=") + value,
                                        std::string("96188?terrain=1&imagery=") + value}) {
            bool ok = true;
            overlume::testing::environment_stream_parse_terrain_assets(spec.c_str(), nullptr,
                                                                       nullptr, &ok);
            EXPECT_FALSE(ok) << "'" << spec << "'";
        }
    }
}

TEST(EnvironmentStream, ParseIonSpecRefusalLogsOneLine) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ScopedUnsetEnv no_token("CESIUM_ION_TOKEN");  // a spec that parses stops at the token
    const overlume::GeoAnchor a{25.0803, 55.3910, 0.0};
    for (const char* uri : {"ion://96188?imagery=2", "ion://96188?terrain=1x",
                            "ion://96188?terrain=1&imagery=0", "ion://96188?radius=abc"}) {
        const size_t from = overlume::testing::captured_cesium_log_text().size();
        EXPECT_FALSE(overlume::set_environment_source(r, uri, a)) << uri;
        const std::string text = overlume::testing::captured_cesium_log_text();
        const std::string logged = text.size() > from ? text.substr(from) : std::string();
        EXPECT_NE(logged.find(std::string(uri) + " refused: "), std::string::npos)
            << uri << ": '" << logged << "'";
        EXPECT_EQ(logged.find("refused"), logged.rfind("refused")) << "one line: " << logged;
    }
    const size_t from = overlume::testing::captured_cesium_log_text().size();
    EXPECT_FALSE(overlume::set_environment_source(r, "ion://96188?terrain=1&imagery=2", a));
    EXPECT_EQ(overlume::testing::captured_cesium_log_text().find("refused", from),
              std::string::npos)
        << "a spec that parses is not refused (with no token it opens nothing, silently)";
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStream, TerrainPlaneFitProbe) {
```

Which revert each test catches:
- `LayerListInAnyOrderSetsTheLayers`: refusing `terrain` alone or a two-name list in either order;
  `buildings` left true for `terrain` alone; the list not ending at the first `?`; the ion key
  `terrain=` read as the layer.
- `EmptyOrUnknownLayerNamesRefuseQuotingTheName`: accepting an empty name (a leading, trailing or
  doubled comma), a case or whitespace variant, or a separator other than `,`; an error that does
  not name both layers or quote the offending name; an error that says `refused`.
- `RepeatedLayerNamesRefuse`: dropping the repeat check; a repeat refused with a text other than
  the contract's.
- `ImagerySatelliteNeedsTheTerrainLayer`: dropping the terrain requirement (Step 7), or checking it
  on an earlier `imagery` value instead of the last.
- `ImageryOffNeedsNoTerrainAndTheLastValueWins`: requiring terrain for `off`; the first value
  winning.
- `UnknownImageryValuesRefuseNamingTheKey`: accepting an empty, case-variant or numeric value, or
  forgiving an earlier bad value.
- `ParseIonSpecTerrainAndImageryAssets`: either key not parsed or not cleared by `off`; the first
  value winning; `follow_terrain` read as `terrain`; the new keys breaking another key.
- `ParseIonSpecImageryNeedsTerrain`: dropping the requirement (Step 7), checking it before the
  last `terrain` value, or requiring terrain for `imagery=off`.
- `ParseIonSpecBadAssetIdFailsTheWholeParse`: accepting 0, a sign, whitespace, a fraction, a
  suffix, a word, `OFF` or an id beyond `int64_t`.
- `ParseIonSpecRefusalLogsOneLine`: a refused spec opening silently (Step 7), logging twice or
  without the spec, or a spec that parses being logged as refused.

- [ ] **Step 3: Run them and see them fail**

No file is new, so no configure:

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_mapbox_uri test_environment_stream
overlume/build/test_mapbox_uri | tail -10
overlume/build/test_environment_stream --gtest_filter='EnvironmentStream.ParseIonSpec*' | tail -8
```

Expected: both build; the first run ends

```
[  PASSED  ] 9 tests.
[  FAILED  ] 6 tests, listed below:
[  FAILED  ] MapboxUri.LayerListInAnyOrderSetsTheLayers
[  FAILED  ] MapboxUri.EmptyOrUnknownLayerNamesRefuseQuotingTheName
[  FAILED  ] MapboxUri.RepeatedLayerNamesRefuse
[  FAILED  ] MapboxUri.ImagerySatelliteNeedsTheTerrainLayer
[  FAILED  ] MapboxUri.ImageryOffNeedsNoTerrainAndTheLastValueWins
[  FAILED  ] MapboxUri.UnknownImageryValuesRefuseNamingTheKey

 6 FAILED TESTS
```

(the old parser refuses every list but `buildings` with "supports only the layer list
'buildings'", and ignores `imagery`), and the second

```
[  PASSED  ] 5 tests.
[  FAILED  ] 4 tests, listed below:
[  FAILED  ] EnvironmentStream.ParseIonSpecTerrainAndImageryAssets
[  FAILED  ] EnvironmentStream.ParseIonSpecImageryNeedsTerrain
[  FAILED  ] EnvironmentStream.ParseIonSpecBadAssetIdFailsTheWholeParse
[  FAILED  ] EnvironmentStream.ParseIonSpecRefusalLogsOneLine

 4 FAILED TESTS
```

(the five existing `ParseIonSpec*` tests pass; `terrain=` and `imagery=` are still ignored, and a
refused spec, `radius=abc` among them, logs nothing). A compile or link error instead means Step 1
or 2 was mistyped; a `[  SKIPPED ]` `no GPU/EGL` for `ParseIonSpecRefusalLogsOneLine` means this
machine cannot run the lane (Task 3 Step 1).

- [ ] **Step 4: Implement both parsers and the ion refusal line**

In `overlume/src/mapbox_loader.cpp`, replace

```cpp
    const std::string layers = spec.substr(0, q);
    if (layers != "buildings") {
        return {std::nullopt,
                "mapbox:// supports only the layer list 'buildings', not '" + layers + "'", ""};
    }
    MapboxSpec out;
    std::string warning;
```

with

```cpp
    const std::string layers = spec.substr(0, q);
    MapboxSpec out;
    out.buildings = false;
    size_t from = 0;
    while (true) {
        const size_t comma = layers.find(',', from);
        const std::string name =
            layers.substr(from, comma == std::string::npos ? comma : comma - from);
        bool* layer = nullptr;
        if (name == "buildings") layer = &out.buildings;
        if (name == "terrain") layer = &out.terrain;
        // Empty, unknown and repeated names share one text: the second mention is the one refused.
        if (layer == nullptr || *layer) {
            return {
                std::nullopt,
                "mapbox:// supports only the layers 'buildings' and 'terrain', not '" + name + "'",
                ""};
        }
        *layer = true;
        if (comma == std::string::npos) break;
        from = comma + 1;
    }
    std::string warning;
```

In `overlume/src/mapbox_loader.cpp`, replace

```cpp
            } else if (key == "cache" && val != "off") {
                return {std::nullopt,
                        "mapbox:// cache='" + val +
                            "' is not 'off': Mapbox tiles are never written to disk",
                        ""};
            }
        }
```

with

```cpp
            } else if (key == "cache" && val != "off") {
                return {std::nullopt,
                        "mapbox:// cache='" + val +
                            "' is not 'off': Mapbox tiles are never written to disk",
                        ""};
            } else if (key == "imagery") {
                if (val != "satellite" && val != "off")
                    return {std::nullopt,
                            "mapbox:// imagery='" + val + "' is not 'satellite' or 'off'", ""};
                out.imagery_satellite = val == "satellite";
            }
        }
```

In `overlume/src/mapbox_loader.cpp`, replace

```cpp
        pos = amp + 1;
    }
    return {out, "", warning};
}
```

with

```cpp
        pos = amp + 1;
    }
    if (out.imagery_satellite && !out.terrain)
        return {std::nullopt, "mapbox:// imagery=satellite needs the 'terrain' layer", ""};
    return {out, "", warning};
}
```

In `overlume/src/environment_stream.cpp`, replace

```cpp
                    out.radius_m = radius;
                } catch (const std::exception&) {
                    return std::nullopt;
                }
            }
        }
        if (amp == std::string::npos) break;
        pos = amp + 1;
    }
    return out;
}
```

with

```cpp
                    out.radius_m = radius;
                } catch (const std::exception&) {
                    return std::nullopt;
                }
            } else if (key == "terrain" || key == "imagery") {
                std::optional<int64_t>& asset =
                    key == "terrain" ? out.terrain_asset : out.imagery_asset;
                if (val == "off") {
                    asset.reset();
                } else {
                    // An ion asset id: digits only (std::stoll alone takes "+1" and " 1"), > 0.
                    if (val.empty() || val.find_first_not_of("0123456789") != std::string::npos)
                        return std::nullopt;
                    try {
                        asset = std::stoll(val);
                    } catch (const std::exception&) {
                        return std::nullopt;  // out of range
                    }
                    if (*asset <= 0) return std::nullopt;
                }
            }
        }
        if (amp == std::string::npos) break;
        pos = amp + 1;
    }
    // The imagery drapes on the terrain tileset: without one it has nothing to drape on.
    if (out.imagery_asset && !out.terrain_asset) return std::nullopt;
    return out;
}
```

In `overlume/src/environment_stream.cpp`, replace

```cpp
    const std::optional<IonSpec> spec = parse_ion_spec(ion_spec);
    if (!spec) return nullptr;
```

with

```cpp
    const std::optional<IonSpec> spec = parse_ion_spec(ion_spec);
    if (!spec) {
        // One line (spec section 5). The spec holds no token: CESIUM_ION_TOKEN is read below.
        make_redacting_logger()->error(
            "ion://{} refused: the asset id or a key has a bad value (terrain= and imagery= take a "
            "positive asset id or off; imagery= needs terrain=)",
            ion_spec);
        return nullptr;
    }
```

- [ ] **Step 5: Run them and see them pass**

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_mapbox_uri test_environment_stream test_mapbox_buildings
overlume/build/test_mapbox_uri | tail -2
overlume/build/test_environment_stream --gtest_filter='EnvironmentStream.ParseIonSpec*' | tail -2
overlume/build/test_mapbox_buildings --gtest_filter='MapboxBuildings.RefusedUrisOpenNothing' \
    | tail -1
```

Expected:

```
[==========] 15 tests from 1 test suite ran. (… ms total)
[  PASSED  ] 15 tests.
[==========] 9 tests from 1 test suite ran. (… ms total)
[  PASSED  ] 9 tests.
[  PASSED  ] 1 test.
```

The last line shows the `mapbox://` loader still logs each refusal once and that no new parser
text says `refused`.

- [ ] **Step 6: Format**

```bash
cd "$WORKTREE"
git add overlume/src/mapbox_loader.hpp overlume/src/mapbox_loader.cpp \
    overlume/src/environment_stream.cpp overlume/src/environment_test_hooks.hpp \
    overlume/tests/test_mapbox_uri.cpp overlume/tests/test_environment_stream.cpp
tools/check_format.sh --fix overlume/src/mapbox_loader.hpp overlume/src/mapbox_loader.cpp \
    overlume/src/environment_stream.cpp overlume/src/environment_test_hooks.hpp \
    overlume/tests/test_mapbox_uri.cpp overlume/tests/test_environment_stream.cpp
git diff --stat
git add overlume/src/mapbox_loader.hpp overlume/src/mapbox_loader.cpp \
    overlume/src/environment_stream.cpp overlume/src/environment_test_hooks.hpp \
    overlume/tests/test_mapbox_uri.cpp overlume/tests/test_environment_stream.cpp
tools/check_format.sh && tools/check_spdx.sh
```

Expected: `check_format.sh: reformatted 6 files with clang-format 23.1.1`; `git diff --stat` prints
nothing; then `check_format.sh: PASS (217 files, clang-format 23.1.1)` and
`check_spdx.sh: PASS (375 files carry the SPDX identifier)` (Task 3's counts: this task adds no
file).

- [ ] **Step 7: Revert checks (do not commit)**: each parser's imagery-needs-terrain rule, and
  the ion refusal line

```bash
cd "$WORKTREE"
sed -i 's/if (out.imagery_satellite \&\& !out.terrain)/if (false)/' overlume/src/mapbox_loader.cpp
sed -i 's/if (out.imagery_asset \&\& !out.terrain_asset) return std::nullopt;/if (false) return std::nullopt;/' \
    overlume/src/environment_stream.cpp
sed -i 's/make_redacting_logger()->error(/if (false) make_redacting_logger()->error(/' \
    overlume/src/environment_stream.cpp
git diff --stat
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_mapbox_uri test_environment_stream
overlume/build/test_mapbox_uri --gtest_filter='MapboxUri.ImagerySatelliteNeedsTheTerrainLayer' \
    | tail -4
overlume/build/test_environment_stream \
    --gtest_filter='EnvironmentStream.ParseIonSpecImageryNeedsTerrain:EnvironmentStream.ParseIonSpecRefusalLogsOneLine' \
    | tail -5
git checkout -- overlume/src/mapbox_loader.cpp overlume/src/environment_stream.cpp && git diff --stat
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_mapbox_uri test_environment_stream
```

Expected: the first diff reports `2 files changed, 3 insertions(+), 3 deletions(-)`; the runs end

```
[  FAILED  ] 1 test, listed below:
[  FAILED  ] MapboxUri.ImagerySatelliteNeedsTheTerrainLayer

 1 FAILED TEST
[  FAILED  ] 2 tests, listed below:
[  FAILED  ] EnvironmentStream.ParseIonSpecImageryNeedsTerrain
[  FAILED  ] EnvironmentStream.ParseIonSpecRefusalLogsOneLine

 2 FAILED TESTS
```

(`buildings?imagery=satellite` and `96188?imagery=2` then parse; `96188?radius=abc` still fails
its parse but logs nothing, so the log test fails whatever the imagery rule does); after the
checkout the last diff prints nothing (the checkout restores the staged files of Step 6), and the build leaves both
binaries current.

- [ ] **Step 8: Gate** (foreground, Bash timeout 600000 ms)

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
ROS_DOMAIN_ID=82 CI_VISUAL_MODE_DOMAIN_ID=82 \
    flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock tools/ci_visual_mode.sh
```

Expected (Task 3's gate plus nine entries: `MapboxUri` 10 → 15, four new `EnvironmentStream`
tests):
- `PASS  POD header check`
- `PASS  library ctest suite  (100% tests passed, 0 tests failed out of 451 (433 ok / 8 skipped))`
- `PASS  node gtests  (Summary: 381 tests, 0 errors, 0 failures, 0 skipped)`
- `PASS  WS bridge pytest suite  (94 passed in …)`, the time varying
- `PASS  golden suite: 25 ok, 0 skipped`
- `PASS  examples  (6 run, all exited 0 with non-empty output)`
- `OVERALL: PASS`, exit 0.

Exit 75 means another lane held the lock for 300 s: run the same command again. Any other non-zero
exit is a gate failure. A WS bridge failure can also come from another session's gate on the same
ports, so rerun before treating it as a finding.

- [ ] **Step 9: Commit**

```bash
cd "$WORKTREE"
git add overlume/src/mapbox_loader.hpp overlume/src/mapbox_loader.cpp \
    overlume/src/environment_stream.cpp overlume/src/environment_test_hooks.hpp \
    overlume/tests/test_mapbox_uri.cpp overlume/tests/test_environment_stream.cpp
git status --short
git diff --cached --stat | tail -1
git commit -F - <<'EOF'
feat(env): parse the terrain and imagery keys of mapbox:// and ion://

Sub-project 2 drapes imagery on terrain, and both sources are chosen by
URI, so the keys come first: the terrain and imagery sources can then be
configured and refused before the loader tasks wire them.

mapbox:// now takes a layer list of buildings and terrain, each at most
once and in any order (an empty, repeated or unknown name refuses the
URI with one line), and imagery=satellite|off, where satellite needs the
terrain layer. ion:// takes terrain=<asset id>|off and
imagery=<asset id>|off: imagery without terrain, or an id that is not a
positive integer in digits, fails the parse. The last value of a key
wins, as for every other key. A refused ion:// spec now logs one line
instead of failing silently, as the design's failure table asks.

Otherwise only the parsers change: the sources read none of the new
fields until the terrain tasks wire them, so until then mapbox://terrain
opens the buildings source. MapboxUri's exact-'buildings' test gives way
to six tests of the list and the imagery key; EnvironmentStream gains
three through the new environment_stream_parse_terrain_assets hook and
one for the refusal line.

Claude-Session: <the executing session's URL>
EOF
```

Expected: `git status --short` lists exactly the six files, each as `M `; the stat ends
`6 files changed, 268 insertions(+), 14 deletions(-)`; the commit succeeds. Then report Tasks 3
and 4 committed: lane 2 is ready to merge.

### Task 5: Terrain material and the renderer's overlay callbacks

**Lane:** 3. **Depends on:** Task 0 (lane worktree built; `assets/materials/*.mat` and
`tests/test_*.cpp` globs proven). Nothing else: no other task's code is consumed.

**Worktree:** `$WORKTREE` (`$INTEGRATION-l3`, branch `terrain-env/l3`, test tmp dir
`/tmp/overlume-l3`, ROS domain 83). Paths are relative to it, and every bash block starts with
`cd "$WORKTREE"`.

**Files** (line numbers as they are at the lane's fork, which Task 0 leaves equal to 427f5d8):
- Create: `overlume/assets/materials/terrain.mat`
- Modify: `overlume/src/theme.hpp` (`struct Environment {`, 133-135)
- Modify: `overlume/src/theme.cpp` (the `environment` parse, 176-181; the fallback theme's
  `t.environment.tile_radius_m = kThemeDefaultTileRadiusM;`, 313)
- Modify: `overlume/src/theme_transition.cpp` (`out.environment.tile_radius_m =`, 161-162)
- Modify: `overlume/assets/themes/dark_adas.yaml` and `overlume/assets/themes/light_clay.yaml`
  (`environment: { tile_radius_m: 100.0 }`, line 42 of each)
- Modify: `overlume/src/renderer_internal.hpp` (`filament::MaterialInstance* buildingMaterial =
  nullptr;`, 359)
- Modify: `overlume/src/renderer.cpp` (includes, 50 and 82; the end of `push_theme_to_scene`,
  451-453; `r->buildingMaterial = r->clayMaterial->createInstance();`, 798;
  `if (r->buildingMaterial) r->engine->destroy(r->buildingMaterial);`, 953)
- Modify: `overlume/src/environment_stream.hpp` (`#include <atomic>`, 10; `namespace spdlog {`,
  44-46; `class StreamRendererResources` through `struct LoadThreadGlb`, 251-310; the
  constructor's last parameter, 344; `force_fall_back_for_testing`, 369)
- Modify: `overlume/src/environment_stream.cpp` (includes, 26 and 36-37; before
  `void strip_attributes_and_correct_heights(`, 235; `prepareInLoadThread`, 711-712;
  `prepareInMainThread`, 752-765; `drain_pending_frees`, 814-826; the constructor, 834 and
  853-854; `update`, 889; after `fixture_grid_requested_leaves`, 1556-1560)
- Modify: `overlume/src/environment_test_hooks.hpp` (appended at the end of the file, after
  `bool renderer_ground_plane_in_scene(overlume::VisualRenderer* r);`, 107)
- Test: `overlume/tests/test_theme.cpp` (includes, 14-21; one test before
  `TEST(ThemeRibbonFade, TransitionToAFadeDisabledThemeHoldsTheMetresInsteadOfShrinkingThem)`, 639)
- Test: `overlume/tests/test_theme_transition.cpp` (`MakeSentinelTheme`, 257; the sentinel test,
  339)
- Test: `overlume/tests/test_environment_stream.cpp` (includes, 7-8; suite
  `EnvironmentStreamOverlay`, 6 tests appended after
  `EnvironmentStream.WithoutAreaSelectionACustomTilesetKeepsTheScreenSpaceViews`, which ends the
  file at 1485)

Lane 2 (Task 4) also edits `environment_stream.cpp`,
`environment_test_hooks.hpp` and `test_environment_stream.cpp`; every hunk here sits at the
anchors above, none of which is near `parse_ion_spec` (63-182), the `environment_stream_parse_*`
hooks (1609-1647) or the `ParseIonSpec*` tests (762-826). Task 4 puts its hook declaration right
after `environment_stream_parse_replaces_ground` (`environment_test_hooks.hpp:70`) and its tests
right after the existing parse tests, never at a file's end; Task 11 appends after this task's
blocks. A merge conflict is a plan deviation to report.

**Interfaces:**
- Consumes (cesium-native 0.64, Filament 1.56.5, verbatim):

```cpp
// Cesium3DTilesSelection/IPrepareRendererResources.h:151-173
virtual void attachRasterInMainThread(const Tile& tile, int32_t overlayTextureCoordinateID,
                                      const CesiumRasterOverlays::RasterOverlayTile& rasterTile,
                                      void* pMainThreadRendererResources,
                                      const glm::dvec2& translation, const glm::dvec2& scale) = 0;
virtual void detachRasterInMainThread(const Tile& tile, int32_t overlayTextureCoordinateID,
                                      const CesiumRasterOverlays::RasterOverlayTile& rasterTile,
                                      void* pMainThreadRendererResources) noexcept = 0;
// CesiumRasterOverlays/IPrepareRasterOverlayRendererResources.h:38, 58, 78
virtual void* prepareRasterInLoadThread(CesiumImage::ImageAsset& image,
                                        const std::any& rendererOptions) = 0;
virtual void* prepareRasterInMainThread(RasterOverlayTile& rasterTile, void* pLoadThreadResult) = 0;
virtual void freeRaster(const RasterOverlayTile& rasterTile, void* pLoadThreadResult,
                        void* pMainThreadResult) noexcept = 0;
// Cesium3DTilesSelection/Tileset.h:218, RasterOverlayCollection.h:148-149
RasterOverlayCollection& Tileset::getOverlays() noexcept;
void RasterOverlayCollection::add(
    const CesiumUtility::IntrusivePointer<const CesiumRasterOverlays::RasterOverlay>& pOverlay) noexcept;
// CesiumRasterOverlays/UrlTemplateRasterOverlay.h:28-74, 110-115
struct UrlTemplateRasterOverlayOptions { /* ... */ uint32_t minimumLevel = 0; uint32_t maximumLevel = 25;
                                         uint32_t tileWidth = 256; uint32_t tileHeight = 256; /* ... */ };
UrlTemplateRasterOverlay(const std::string& name, const std::string& url,
                         const std::vector<CesiumAsync::IAssetAccessor::THeader>& headers = {},
                         const UrlTemplateRasterOverlayOptions& urlTemplateOptions = {},
                         const RasterOverlayOptions& overlayOptions = {});
// Cesium3DTilesSelection/TileContent.h:180, 355
void* TileRenderContent::getRenderResources() const noexcept;
const TileRenderContent* TileContent::getRenderContent() const noexcept;
// filament: Material.h, MaterialInstance.h:130, 185-187, TextureSampler.h:81, Texture.h
MaterialInstance* Material::createInstance(const char* name = nullptr) const noexcept;
MaterialInstance* Material::getDefaultInstance() noexcept;
template <typename T> void MaterialInstance::setParameter(const char* name, T const& value);
void MaterialInstance::setParameter(const char* name, Texture const* texture,
                                    TextureSampler const& sampler);
TextureSampler(MinFilter min, MagFilter mag, WrapMode str = WrapMode::CLAMP_TO_EDGE) noexcept;
void Texture::setImage(Engine& engine, size_t level, PixelBufferDescriptor&& buffer) const;
void Texture::generateMipmaps(Engine& engine) const noexcept;
```

- Produces (the Interface contract, with the deviations the API notes justify marked):

```cpp
// overlume/src/environment_stream.hpp
class StreamRendererResources {
public:
    enum class Kind : uint8_t { Content, Terrain };
    struct OverlayCounters {
        uint32_t rasters_attached = 0, rasters_detached = 0, textures_alive = 0, textures_freed = 0;
    };
    // `Kind kind` is defaulted: the two parameters before it have defaults, so C++ requires one.
    explicit StreamRendererResources(const glm::dmat4& ecefToMap, double anchorHeightM,
                                     bool materialsOriginal = false, double brightness = 1.0,
                                     Kind kind = Kind::Content);
    OverlayCounters overlay_counters() const;
    void sync_terrain_color();  // beyond the contract: live tiles follow theme switches
};
struct RasterLoadThreadResult { uint32_t width = 0; uint32_t height = 0; std::vector<uint8_t> rgba; };
class StreamingEnvironmentSource {
public:
    // Beyond the contract, additive: the hook below opens a terrain-flavoured source before
    // Task 11's attach_terrain exists. Task 11 makes overlay_counters() read its terrain
    // resources when attach_terrain has run (else these), so the hook keeps working.
    StreamingEnvironmentSource(/* existing parameters */, CustomTileset custom_tileset = {},
                               StreamRendererResources::Kind resources_kind = {});
    StreamRendererResources::OverlayCounters overlay_counters() const;
    Cesium3DTilesSelection::RasterOverlayCollection& overlays_for_testing();
};
// overlume/src/environment_test_hooks.hpp, namespace overlume::testing
struct OverlayProbe { uint32_t rasters_attached, rasters_detached, textures_alive, textures_freed; };
OverlayProbe environment_stream_overlay_probe(overlume::VisualRenderer* r);
// `brightness` and `maximum_level` are the contract's trailing defaults: a three-argument call
// compiles.
bool install_fixture_terrain_with_file_imagery(overlume::VisualRenderer* r, const char* fixture_dir,
                                               const char* imagery_dir, double brightness = 1.0,
                                               uint32_t maximum_level = 10);
// overlume/src/renderer_internal.hpp, VisualRenderer
filament::Material* terrainMaterial = nullptr;
filament::Texture* terrainPlaceholderTexture = nullptr;  // 1x1 white, bound on detach
// overlume/src/theme.hpp, Theme::Environment (inside namespace overlume::detail)
Float3 terrain_color;  // the contract's detail::Float3, like every theme colour here
```

  `terrain.mat` parameters: `float3 terrainColor`, `int hasImagery`, `sampler2d imagery`,
  `float4 uvTransform` (translation.xy, scale.zw), `float brightness`; no fade parameter (API
  notes). For Task 11: the terrain tileset's resources are
  `std::make_shared<StreamRendererResources>(ecefToMap, anchorHeightM, false, brightness,
  StreamRendererResources::Kind::Terrain)`, and the source must call `set_renderer`,
  `sync_terrain_color()` and `drain_pending_frees()` on them every `update`, as it does on the
  content resources (Step 10).

  **Contract amendments** (the head.md items below are applied: the plan head and the spec,
  7ec7092, now carry them; lane 4's item is that lane's to apply):
  - head.md, `terrain.mat` paragraph: no fade parameter (the stream fades no tile; tiles pop in);
    the imagery texture is 8-bit RGBA in the sRGB format (`SRGB8_A8`), mipmapped; detach sets
    `hasImagery = 0` and binds the 1x1 white placeholder back; `renderer_internal.hpp` also gains
    `filament::Texture* terrainPlaceholderTexture`.
  - head.md, Theme paragraph: `detail::Float3 terrain_color` in `Theme::Environment`, not
    `float[3]`.
  - head.md, hooks block: `install_fixture_terrain_with_file_imagery` takes the two trailing
    defaults above (`double brightness = 1.0, uint32_t maximum_level = 10`).
  - head.md, File map: Task 5 also modifies `overlume/src/theme_transition.cpp` and
    `overlume/tests/test_theme_transition.cpp` (see below the Files list).
  - head.md, Review Focus: the raster's orientation and cesium's translation/scale order are
    pinned in Task 5 (`EnvironmentStreamOverlay.ImageryKeepsNorthUp`, Steps 18-19), not left to
    Task 13's goldens.
  - lane 4, Task 8's `docs/runbooks/terrain_imagery.md` text: drop "the stream's fade" (no fade
    parameter: terrain tiles pop in, as buildings do), say "an sRGB 8-bit mipmapped texture"
    for "RGBA8", and say detach puts the placeholder back and shows `terrain_color` again.

**API notes** (cesium-native 0.64.0 under `overlume/build/_deps/cesium-native-src/`; Filament
1.56.5 headers under `overlume/build/_deps/filament-1.56.5/filament/include/`, its `.cpp` sources
cited by path inside the 1.56.5 source release):
- **The stream applies no fade to new tiles.** `prepareInMainThread`
  (`environment_stream.cpp:738-800`) swaps every primitive to `r_->buildingMaterial`, one shared
  `clay.mat` instance (`renderer.cpp:798`) whose parameters are `baseColor`, `roughness`,
  `metallic` only (`clay.mat`); `environment_stream.cpp` contains no fade or alpha at all. The
  faded materials (`clay_faded.mat` reads `getColor().a`, `height_grid_faded.mat` an `alpha`
  parameter) serve map elements, ribbons, markers and height grids (`renderer.cpp:283-417`), never
  streamed tiles. So there is no stream fade to carry over (spec §5 and the contract say so), and
  `terrain.mat` has none: terrain tiles pop in as buildings do, and an opaque terrain
  keeps out of the blended queue. Review Focus 2 rests on `hasImagery = 0` in the default
  instance instead.
- **Overlay UVs.** `TilesetContentManager.cpp:446-455` generates them after every loader's glTF
  with `invertVCoordinate = false` and the base name `"_CESIUMOVERLAY_"`
  (`RasterOverlayUtilities.h:31-32`), one FLOAT VEC2 accessor per projection
  (`RasterOverlayUtilities.cpp:209-229`), so v = 0 is the tile rectangle's south edge.
  `strip_attributes_and_correct_heights` erases every attribute whose name starts with `_`
  (`environment_stream.cpp:239-245`), after `consolidate_buffers` (711-712), so the copy into
  `TEXCOORD_0` goes between the two. One mapped raster per overlay per geometry tile
  (`RasterMappedTo3DTile.cpp:303-356`, `addRealTile`), so overlay 0 uses coordinate set 0.
- **gltfio and UV0.** Overlume loads tiles with the ubershader provider (`objects.cpp:320-321`).
  A `TEXCOORD_0` no texture of the tile's material uses goes to UV0 when no other UV set is in use
  (`libs/gltfio/src/AssetLoader.cpp:1036-1066`, the UNUSED slot: "This allows clients to swap the
  glTF material with a custom material"); the fixture's three b3dm tiles carry no texture and no
  `TEXCOORD_0` (their glTF attributes are `POSITION`, `NORMAL`, `_BATCHID`). Without one, the
  ubershader provider still adds a dummy UV0 (`AssetLoader.cpp:1166-1170`,
  `UbershaderProvider.cpp:118-126`), so `requires : [ uv0 ]` never meets a vertex buffer without it,
  and a missing copy is invisible under one flat colour: the test checks the GLB itself
  (`capture_stream_glbs`, `mapbox_test_hooks.hpp:78-79`). `ensure_flat_normals` keeps every
  attribute it does not add (`gltf_normals.cpp:304-387`).
- **Image orientation.** Raster images keep row 0 at the north edge
  (`QuadtreeRasterOverlayTileProvider.cpp:382-411`: pixel y from `maximumY`). gltfio's own
  materials set `flipUV : false` (`libs/gltfio/materials/base.mat.in:8`) and upload image rows as
  decoded, so texture v = 0 is row 0. `terrain.mat` therefore sets `flipUV : false`, applies
  cesium's transform in cesium's space (`uv * scale + translation`,
  `IPrepareRendererResources.h:144-150`) and samples at `1 - v`. Flat imagery cannot see either
  choice, so `ImageryKeepsNorthUp` colours the imagery by latitude (red north of tile_root's region
  centre, blue south) and reads two of tile_root's buildings, one wholly north and one wholly south
  of that line (`make_tile_fixture.py`, `generate_tile_geometry` with seed `SEED + 1`: E 30.4,
  N 29.5, 11.8 x 16.0 m, 26.1 m tall; E -13.1, N -25.4, 14.4 x 8.2 m, 11.2 m tall, standing at the
  region's mid height, -2.74 m). The overlay UVs span the tile's bounding region
  (`TilesetContentManager.cpp:446-455` passes `pRegion->getRectangle()`), and the raster is that
  rectangle snapped out to whole source pixels, a straight blit
  (`QuadtreeRasterOverlayTileProvider.cpp`, `measureCombinedImage` 498, `combineImages` 620). So
  cesium's translation is near 0 and its scale near 1: swapping them samples near one corner (one
  colour on both buildings), and dropping the flip mirrors the raster about the region's middle
  latitude, which is the colour line, so the two colours swap. Each probe is a building's centre at
  half its height, which lands on that building's own faces from any side; the line from
  `kBlockPose`'s eye to either centre passes no other building (checked against all five boxes).
- **The fixture's `RTC_CENTER` moves the overlay UVs, not the tiles.** Every fixture b3dm holds
  absolute ECEF positions and also an `RTC_CENTER` (tile_root's feature table is
  `{"BATCH_LENGTH":1,"RTC_CENTER":[3283395.75,4757123.67,2687195.05]}`), which the stream's
  placement ignores (`make_tile_fixture.py:31-45`). cesium's overlay step does not:
  `B3dmToGltfConverter.cpp:168-182` turns `RTC_CENTER` into `CESIUM_RTC`, and
  `RasterOverlayUtilities.cpp:96-99` builds `rootTransform * RTC * Y_UP_TO_Z_UP` before it
  projects the positions to UVs. Each vertex then sits at about twice its ECEF vector, whose
  geodetic latitude is about 25.007 N for tile_root's centre (25.063 N for its north edge), south
  of the whole region, so v clamps to 0 (`RasterOverlayUtilities.cpp:309-320`) on every vertex
  while u stays right (scaling keeps the longitude). Flat imagery cannot see it, so the five
  flat-colour tests keep the shipped fixture; `ImageryKeepsNorthUp` runs on a tmp copy whose b3dm
  feature tables drop `RTC_CENTER` (`tiles_fixture_without_rtc`; the JSON keeps its length,
  padded with spaces, so the 28-byte header and the 8-byte alignment stay valid). On the shipped
  fixture both probes read the raster's south edge ('b'), which looks like a north-south mirror and
  is not. Any later task that drapes imagery on these b3dm fixtures inherits the trap; DEM and
  quantized-mesh terrain build glTF without `CESIUM_RTC`.
- **When the callbacks run.** `updateDoneState` calls `updateTileOverlays` only for Done tiles
  with render content (`TilesetContentManager.cpp:2105-2157`, 2124), so `attachRasterInMainThread`
  always meets a tile whose render resources are `prepareInMainThread`'s `FilamentAsset*`
  (or nullptr when that failed). `RasterMappedTo3DTile::update` (78-208) calls
  `loadInMainThread` (which runs `prepareRasterInMainThread`, `RasterOverlayTile.cpp:88-101`)
  then attach (186-193), detaches before re-attaching a better raster (120-124, 156-160), and
  marks a failed raster ready without attaching it (172-178); `detachFromTile` skips failed rasters
  (219-237). `unloadTileContent` detaches every mapped raster before `free`
  (`TilesetContentManager.cpp:1416-1421`, then `unloadDoneState`, 2175-2189). `freeRaster` runs in
  `~RasterOverlayTile` (`RasterOverlayTile.cpp:37-63`) on whichever thread drops the last
  reference, so it only queues, under `freeMutex_`, as `free` does (802-812).
- **Load thread.** `prepareRasterInLoadThread` runs on a worker after decoding
  (`ActivatedRasterOverlay.cpp:194-252`, call at 233); an image that fails to decode never reaches
  it and the raster tile becomes `Failed` (200-206). PNG and JPEG decode through stb to 4 channels,
  1 byte each (`CesiumImage/src/ImageDecoder.cpp:325-335`). The image may be shared through the
  provider's image cache, so the copy to RGBA8 does not modify it.
- **Texture.** `Texture::Builder::levels(0xff)` is clamped to the full chain
  (`filament/src/details/Texture.cpp:113-114, 157-171`); `generateMipmaps` needs a mipmappable
  format (526-544). gltfio builds base-colour textures exactly so: `SRGB8_A8`, `levels(0xff)`,
  `setImage`, `generateMipmaps` (`libs/gltfio/src/StbProvider.cpp:99-104, 187-193`). The spec's
  "RGBA8" is honoured as 8-bit RGBA in the sRGB format, so the lit material gets linear albedo, as
  for every glTF texture. The upload keeps its pixels alive with a heap vector freed in the
  descriptor's callback, the pattern of `ground_grid.cpp:128-137`.
- **Per-tile instances from the default instance.** `Material::createInstance` duplicates the
  material's default instance when one exists (`filament/src/details/Material.cpp:440-447`),
  copying its uniforms and its descriptor set, textures included (`MaterialInstance.cpp:118-122`).
  So `create_renderer` sets the defaults once (placeholder texture, `hasImagery 0`, identity
  `uvTransform`, `brightness 1`) and `push_theme_to_scene` keeps `terrainColor` current on the
  default instance; it runs at creation (`renderer.cpp:825`) and every frame of a theme transition
  (`apply_current_theme`, 1031-1044, before the environment update in `render_frame`, 1099 and
  1115). Live tiles are recoloured by `sync_terrain_color`, called first in every
  `StreamingEnvironmentSource::update` (`renderer.cpp` is C++17 and cannot see the stream's
  types). The default instance is destroyed with its material (`Material.cpp:374-386`); a
  material destroyed with instances still alive is a non-fatal precondition
  (`filament/src/details/Engine.cpp:1152-1172`), so a tile leaked past the teardown bound cannot
  abort `destroy_renderer`.
- **Detach puts the placeholder back.** A raster texture can be freed while the instance that
  sampled it lives on (a tile keeps its instance after a detach), so detach rebinds
  `terrainPlaceholderTexture`: no instance ever references a texture `drain_pending_frees`
  destroys. Destruction order in the drain: the asset (its renderables use the instance), then the
  instance; textures after.
- **The fixture tileset as terrain.** `FileFixtureAssetAccessor` serves `file://` URLs by
  stripping the scheme (`environment_stream.cpp:508-509`): 404 for a missing file (516-519), 200
  with the bytes otherwise. `UrlTemplateRasterOverlay` substitutes `{z}`, `{x}`, `{y}`,
  `{reverseY}` and further placeholders (`UrlTemplateRasterOverlay.cpp:72-120`,
  `loadQuadtreeTileImage`; the hook's template uses `{y}`, the Mapbox one `{reverseY}`) and decodes by content, not by
  content type (`RasterOverlayTileProvider.cpp:183-224`). cesium's `{y}` counts rows from the
  south edge of the web-mercator square (`QuadtreeTilingScheme.cpp:64-78`:
  `bottom = minimumY + y * yTileHeight`); `{reverseY}` is the XYZ row. The level follows from the
  geometry tile's error and is clamped to `[minimumLevel, maximumLevel]`
  (`QuadtreeRasterOverlayTileProvider.cpp:145-182`; `RasterOverlayUtilities.cpp:604-647`): a
  target raster tile spans geometric error x 256 x 2 / 48 on the ground (256-px tiles, raster SSE
  2, the stream's `kStreamMaxSseErr` 48), about 200 m for the fixture's 18.8 m leaves, which
  rounds to level 17. The fixture root region (lon 55.3709..55.4485, lat 25.0499..25.1368,
  `tileset.json`) sits inside the single level-10 tile x 669, south-row 585, so the hook caps the
  overlay at level 10 by default (one raster tile per leaf, every flat-colour test writes 86
  files); `ImageryKeepsNorthUp` caps it at 14 (8.7 m texels at this latitude, 188 files) so the
  two probed buildings sit more than two texels either side of the colour line. A tile that fails
  to load or decode falls back to its parent (`QuadtreeRasterOverlayTileProvider.cpp:124-129`),
  so the tests write every level 0..cap within one tile of the region. A 404 counts toward
  `kNetworkLossConsecutiveFailures` (8, `environment_stream.hpp:56`;
  `CountingAssetAccessor::note`, 151-159), which would drop the source to `STREAMING_FALLBACK`
  mid-test, so "no raster yet" uses tiles that answer 200 with undecodable bytes. The hook builds externals with cache `off` (as `install_fixture_grid_source`,
  1534-1554), so imagery of one test never serves another.
- **Leaves render before their rasters.** `Tile::isRenderable` waits for rasters
  (`Tile.cpp:218-239`), but the fixture tiles are leaves, which cesium lists whatever their state,
  and the render loop admits them once `Done` (`environment_stream.cpp:1124-1150`), so the frame
  shows `terrainColor` until the raster attaches.
- **Theme.** `Float3 to_float3(node)` throws on anything but a 3-sequence (`theme.cpp:18-23`),
  and `load_theme` turns a throw into `nullopt` (246-255); `set_theme` replaces a
  `transition_sec <= 0` by 0.8 s (`renderer.cpp:1181-1192`) and `apply_current_theme` measures it
  against the scene's `sim_time_sec`, so the switch test starts its transition 10 s in the past.
- **Pixel probes.** The golden pose of `EnvironmentStreamGolden.FixtureBlock_DarkAdas`
  (`{149, 352, 90} -> {69, 472, 0}`) shows the fixture block on about 2,600 of 76,800 pixels
  (counted in `tests/goldens/environment_stream_dark_adas.png`: 2,643 pixels with blue above 60).
  The probes use light_clay copies (fog 0.00025 against dark_adas's 0.010), so a green or red
  surface keeps a channel lead above 40 on lit and shaded faces; light_clay's sky is blue-led and
  its ground grey, so neither counts. `render_frame` runs the environment only while
  `ego.valid` (`renderer.cpp:1114-1116`). The block area was counted on clay, which is
  double-sided (`renderer.cpp:822`); terrain instances keep Filament's back-face culling (the
  contract's "double-sided off"; DEM and quantized-mesh tiles are counter-clockwise from above),
  so if a count misses, the fixture boxes' winding is a candidate cause to check before the
  thresholds. `hue_at` reads a 3x3 window around `project_to_screen`'s point, as
  `golden_luma` does (`test_mapbox_buildings.cpp:970-987`).
- **Build.** matc runs at configure time for every `assets/materials/*.mat` and writes
  `generated/overlume_materials/terrain_filamat.h` with `overlume::materials::kterrainFilamat` and
  `kterrainFilamatSize` (`overlume/CMakeLists.txt:178-206`), so the new material needs the
  reconfigure block. `terrain.mat` compiles with this build's matc for `-p desktop` and
  `-p mobile`. Tests link `golden.cpp`, which defines `STB_IMAGE_WRITE_IMPLEMENTATION`
  (`golden.cpp:8`); `test_environment_stream.cpp` already includes `stb_image_write.h`.

- [ ] **Step 1: Write the failing theme tests**

In `overlume/tests/test_theme.cpp`, replace

```cpp
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>
#include <optional>
#include <string>
```

with

```cpp
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <sstream>
#include <string>
```

In `overlume/tests/test_theme.cpp`, replace

```cpp
TEST(ThemeRibbonFade, TransitionToAFadeDisabledThemeHoldsTheMetresInsteadOfShrinkingThem) {
```

with

```cpp
TEST(ThemeEnvironment, TerrainColorParsesAndDefaultsToTheGround) {
    using overlume::detail::Float3;
    const auto expect_eq = [](const Float3& got, const Float3& want, const std::string& what) {
        EXPECT_FLOAT_EQ(got.r, want.r) << what;
        EXPECT_FLOAT_EQ(got.g, want.g) << what;
        EXPECT_FLOAT_EQ(got.b, want.b) << what;
    };
    for (const char* name : {"dark_adas", "light_clay"}) {
        const std::optional<overlume::detail::Theme> shipped =
            overlume::detail::load_theme(kThemeDir, name);
        ASSERT_TRUE(shipped.has_value()) << name;
        expect_eq(shipped->environment.terrain_color, shipped->palette.ground,
                  std::string(name) + ": the shipped terrain_color is the theme's ground");
    }
    const std::string fixtureDir = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
    const std::optional<overlume::detail::Theme> absent =
        overlume::detail::load_theme(fixtureDir, "tile_radius_tiny");
    ASSERT_TRUE(absent.has_value());
    expect_eq(absent->environment.terrain_color, absent->palette.ground,
              "an environment map without terrain_color defaults to the ground");
    const overlume::detail::Theme& fallback = overlume::detail::kFallbackTheme();
    expect_eq(fallback.environment.terrain_color, fallback.palette.ground, "the fallback theme");

    // An explicit value wins: dark_adas with its environment line rewritten.
    std::ifstream in(std::string(kThemeDir) + "/dark_adas.yaml");
    std::stringstream text;
    text << in.rdbuf();
    std::string yaml = text.str();
    const size_t at = yaml.find("environment:");
    ASSERT_NE(at, std::string::npos);
    yaml.replace(at, yaml.find('\n', at) - at,
                 "environment: { tile_radius_m: 100.0, terrain_color: [0.25, 0.5, 0.75] }");
    const std::string dir = std::string(OVERLUME_TMP_DIR) + "/overlume_theme_terrain_color";
    std::filesystem::create_directories(dir);
    std::ofstream(dir + "/explicit.yaml") << yaml;
    const std::optional<overlume::detail::Theme> explicitColor =
        overlume::detail::load_theme(dir, "explicit");
    ASSERT_TRUE(explicitColor.has_value());
    expect_eq(explicitColor->environment.terrain_color, Float3{0.25f, 0.5f, 0.75f},
              "environment.terrain_color: [0.25, 0.5, 0.75]");
}

TEST(ThemeRibbonFade, TransitionToAFadeDisabledThemeHoldsTheMetresInsteadOfShrinkingThem) {
```

In `overlume/tests/test_theme_transition.cpp`, replace

```cpp
    t.environment.tile_radius_m = scalar;
```

with

```cpp
    t.environment.tile_radius_m = scalar;
    t.environment.terrain_color = c;
```

In `overlume/tests/test_theme_transition.cpp`, replace

```cpp
    ExpectBetweenSentinels(mid.environment.tile_radius_m, "environment.tile_radius_m");
```

with

```cpp
    ExpectBetweenSentinels(mid.environment.tile_radius_m, "environment.tile_radius_m");
    ExpectBetweenSentinels(mid.environment.terrain_color, "environment.terrain_color");
```

- [ ] **Step 2: Build them and see them fail**

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_theme test_theme_transition 2>&1 | grep 'error:' | sort -u
```

Expected (the build stops; paths and line numbers vary):

```
.../overlume/tests/test_theme.cpp:...: error: no member named 'terrain_color' in 'overlume::detail::Theme::Environment'
```

(four such lines for `test_theme.cpp`; `test_theme_transition.cpp` gives two more if make reaches
it first).

- [ ] **Step 3: Add the theme token**

In `overlume/src/theme.hpp`, replace

```cpp
    struct Environment {
        float tile_radius_m = kThemeDefaultTileRadiusM;
    } environment;
```

with

```cpp
    struct Environment {
        float tile_radius_m = kThemeDefaultTileRadiusM;
        // Streamed terrain tiles until their imagery attaches, and with imagery off. The parser
        // defaults it to palette.ground.
        Float3 terrain_color;
    } environment;
```

In `overlume/src/theme.cpp`, replace

```cpp
    t.environment.tile_radius_m =
        (std::isfinite(tileRadius) && tileRadius > 0.0f) ? tileRadius : kThemeDefaultTileRadiusM;

    return t;
```

with

```cpp
    t.environment.tile_radius_m =
        (std::isfinite(tileRadius) && tileRadius > 0.0f) ? tileRadius : kThemeDefaultTileRadiusM;
    t.environment.terrain_color = (environment && environment["terrain_color"])
                                      ? to_float3(environment["terrain_color"])
                                      : t.palette.ground;

    return t;
```

In `overlume/src/theme.cpp`, replace

```cpp
        t.environment.tile_radius_m = kThemeDefaultTileRadiusM;
        t.ogm.dynamic = bake_ogm_ramp(
```

with

```cpp
        t.environment.tile_radius_m = kThemeDefaultTileRadiusM;
        t.environment.terrain_color = t.palette.ground;
        t.ogm.dynamic = bake_ogm_ramp(
```

In `overlume/src/theme_transition.cpp`, replace

```cpp
    out.environment.tile_radius_m =
        lerpf(a.environment.tile_radius_m, b.environment.tile_radius_m, w);
```

with

```cpp
    out.environment.tile_radius_m =
        lerpf(a.environment.tile_radius_m, b.environment.tile_radius_m, w);
    out.environment.terrain_color =
        blend_color(a.environment.terrain_color, b.environment.terrain_color, w);
```

In `overlume/assets/themes/dark_adas.yaml`, replace

```yaml
environment: { tile_radius_m: 100.0 }
```

with

```yaml
environment: { tile_radius_m: 100.0, terrain_color: [0.055, 0.055, 0.078] }
```

In `overlume/assets/themes/light_clay.yaml`, replace

```yaml
environment: { tile_radius_m: 100.0 }
```

with

```yaml
environment: { tile_radius_m: 100.0, terrain_color: [0.762, 0.716, 0.672] }
```

(each theme's own `palette.ground`, so no frame changes).

- [ ] **Step 4: Run them and see them pass**

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_theme test_theme_transition \
    && overlume/build/test_theme --gtest_filter='ThemeEnvironment.*' \
    && overlume/build/test_theme_transition \
        --gtest_filter='ThemeTransition.SentinelThemesDetectAnyUnblendedField'
```

Expected: the first run ends `[  PASSED  ] 2 tests.`
(`TileRadiusParsesDefaultsAndRejectsNonPositive` and `TerrainColorParsesAndDefaultsToTheGround`),
the second `[  PASSED  ] 1 test.`

- [ ] **Step 5: Write the material** `overlume/assets/materials/terrain.mat`

```
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

// Streamed terrain tiles: terrainColor until a raster overlay attaches (hasImagery 0), then the
// raster texture times brightness. uv0 is cesium's _CESIUMOVERLAY_0, which the stream's GLB step
// copies to TEXCOORD_0: v = 0 on the tile's south edge. uvTransform is cesium's translation (xy)
// and scale (zw), uv' = uv * scale + translation. flipUV is off so the transform runs in cesium's
// space; the lookup flips v once, because raster images keep row 0 at their north edge. No fade
// parameter: the stream fades no tile in (tiles pop in, as the clay buildings do).
material {
    name : terrain,
    shadingModel : lit,
    requires : [ uv0 ],
    flipUV : false,
    parameters : [
        { type : float3, name : terrainColor },
        { type : int, name : hasImagery },
        { type : sampler2d, name : imagery },
        { type : float4, name : uvTransform },
        { type : float, name : brightness }
    ]
}
fragment {
    void material(inout MaterialInputs material) {
        prepareMaterial(material);
        if (materialParams.hasImagery != 0) {
            vec2 uv = getUV0() * materialParams.uvTransform.zw + materialParams.uvTransform.xy;
            material.baseColor.rgb = texture(materialParams_imagery, vec2(uv.x, 1.0 - uv.y)).rgb *
                                     materialParams.brightness;
        } else {
            material.baseColor.rgb = materialParams.terrainColor;
        }
    }
}
```

The sampler's filtering (trilinear, clamp to edge) is set where the texture is bound (Step 10);
matc's sampler parameters carry no filter.

- [ ] **Step 6: Write the failing overlay tests** in `overlume/tests/test_environment_stream.cpp`

Replace

```cpp
#include "environment_test_hooks.hpp"
#include "golden.hpp"
```

with

```cpp
#include "environment_test_hooks.hpp"
#include "golden.hpp"
#include "mapbox_test_hooks.hpp"
```

and append, after the closing brace of
`TEST(EnvironmentStream, WithoutAreaSelectionACustomTilesetKeepsTheScreenSpaceViews)` (the end of
the file):

```cpp

namespace {

// The golden pose of EnvironmentStreamGolden.FixtureBlock_DarkAdas: the fixture block covers about
// 2,600 of the frame's 76,800 pixels (counted in environment_stream_dark_adas.png).
const overlume::CameraPose kBlockPose{{149, 352, 90}, {69, 472, 0}, 60.0};

// Two copies of light_clay (thin fog at the pose's 150 m, so hues reach the frame) that differ
// only in environment.terrain_color: terrain_green and terrain_red. Returns their directory.
std::string write_terrain_themes() {
    const std::string dir = std::string(OVERLUME_TMP_DIR) + "/overlume_terrain_themes";
    std::ifstream in(std::string(kThemeDir) + "/light_clay.yaml");
    std::stringstream text;
    text << in.rdbuf();
    const std::string yaml = text.str();
    const size_t at = yaml.find("environment:");
    if (at == std::string::npos) {
        // The renderer then fails to load the theme; the test already carries this failure.
        ADD_FAILURE() << "light_clay.yaml has no environment: line";
        return dir;
    }
    std::filesystem::create_directories(dir);
    for (const auto& [name, rgb] :
         {std::pair<std::string, std::string>{"terrain_green", "[0.05, 0.6, 0.05]"},
          std::pair<std::string, std::string>{"terrain_red", "[0.6, 0.05, 0.05]"}}) {
        std::string variant = yaml;
        variant.replace(at, yaml.find('\n', at) - at,
                        "environment: { tile_radius_m: 100.0, terrain_color: " + rgb + " }");
        std::ofstream(dir + "/" + name + ".yaml") << variant;
    }
    return dir;
}

struct Rgb {
    uint8_t r, g, b;
};

// 256-px imagery tiles at <dir>/{z}/{x}/{y}.png for every web-mercator tile of levels
// 0..max_level within one tile of the fixture tileset's region (lon 55.3709..55.4485,
// lat 25.0499..25.1368), rows counted from the south as cesium fills {y}. Each pixel row takes
// colour(latitude of the row's centre, degrees); row 0 is the tile's north edge. With decodable
// false each file holds bytes no image decoder reads: every request answers 200, no raster lands.
template <typename Colour>
void write_fixture_imagery(const std::string& dir, bool decodable, int max_level, Colour colour) {
    std::error_code ec;
    std::filesystem::remove_all(dir, ec);
    std::vector<uint8_t> rgb(256u * 256u * 3u);
    for (int z = 0; z <= max_level; ++z) {
        const int n = 1 << z;
        const auto column = [n](double lon) { return static_cast<int>((lon + 180.0) / 360.0 * n); };
        const auto row = [n](double lat) {
            const double fromNorth =
                (1.0 - std::asinh(std::tan(lat * M_PI / 180.0)) / M_PI) / 2.0 * n;
            return n - 1 - static_cast<int>(fromNorth);
        };
        for (int x = std::max(0, column(55.3709) - 1); x <= std::min(n - 1, column(55.4485) + 1);
             ++x) {
            const std::string folder = dir + "/" + std::to_string(z) + "/" + std::to_string(x);
            std::filesystem::create_directories(folder);
            for (int y = std::max(0, row(25.0499) - 1); y <= std::min(n - 1, row(25.1368) + 1);
                 ++y) {
                const std::string path = folder + "/" + std::to_string(y) + ".png";
                if (!decodable) {
                    std::ofstream(path, std::ios::binary) << "not an image";
                    continue;
                }
                for (size_t j = 0; j < 256; ++j) {
                    // Row j's centre as a fraction of the mercator square from its north edge.
                    const double t = (n - 1 - y + (static_cast<double>(j) + 0.5) / 256.0) / n;
                    const Rgb c =
                        colour(std::atan(std::sinh(M_PI * (1.0 - 2.0 * t))) * 180.0 / M_PI);
                    for (size_t i = 0; i < 256; ++i) {
                        uint8_t* p = &rgb[(j * 256u + i) * 3u];
                        p[0] = c.r;
                        p[1] = c.g;
                        p[2] = c.b;
                    }
                }
                ASSERT_NE(stbi_write_png(path.c_str(), 256, 256, 3, rgb.data(), 256 * 3), 0)
                    << path;
            }
        }
    }
}

// One colour everywhere, levels 0..10 (the hook's default cap).
void write_fixture_imagery(const std::string& dir, uint8_t red, uint8_t green, uint8_t blue,
                           bool decodable) {
    write_fixture_imagery(dir, decodable, 10, [&](double) { return Rgb{red, green, blue}; });
}

// Pixels whose red (green) channel leads both others by more than 40, and the frame's red sum.
struct Hues {
    size_t red = 0;
    size_t green = 0;
    uint64_t red_sum = 0;
};

Hues count_hues(const std::vector<uint8_t>& rgb) {
    Hues h;
    for (size_t i = 0; i + 2 < rgb.size(); i += 3) {
        const int r = rgb[i], g = rgb[i + 1], b = rgb[i + 2];
        if (r > g + 40 && r > b + 40) ++h.red;
        if (g > r + 40 && g > b + 40) ++h.green;
        h.red_sum += static_cast<uint64_t>(r);
    }
    return h;
}

// The JSON chunk of a GLB: a 12-byte header, the chunk's length and type, then the text.
std::string glb_json(const std::vector<uint8_t>& glb) {
    if (glb.size() < 20) return {};
    uint32_t length = 0;
    std::memcpy(&length, glb.data() + 12, sizeof(length));
    if (20 + static_cast<size_t>(length) > glb.size()) return {};
    return std::string(reinterpret_cast<const char*>(glb.data() + 20), length);
}

// Which channel leads the other two by more than 40 (on average) in the 3x3 pixels around where
// the map point `p` lands in `frame`, a 320x240 render through the renderer's current camera:
// 'r', 'g' or 'b'; '-' for none, and off-screen. The window is golden_luma's
// (test_mapbox_buildings.cpp), copied because the two tests build separate binaries.
char hue_at(overlume::VisualRenderer* r, const std::vector<uint8_t>& frame,
            const overlume::Vec3& p) {
    float u = 0.0f, v = 0.0f;
    if (!overlume::project_to_screen(r, p, &u, &v)) return '-';
    const int cx = std::clamp(static_cast<int>(u * 320.0f), 1, 318);
    const int cy = std::clamp(static_cast<int>(v * 240.0f), 1, 238);
    int sum[3] = {0, 0, 0};
    for (int y = cy - 1; y <= cy + 1; ++y) {
        for (int x = cx - 1; x <= cx + 1; ++x) {
            const size_t i = (static_cast<size_t>(y) * 320u + static_cast<size_t>(x)) * 3u;
            for (int c = 0; c < 3; ++c) sum[c] += frame[i + static_cast<size_t>(c)];
        }
    }
    for (int c = 0; c < 3; ++c) {
        if (sum[c] > sum[(c + 1) % 3] + 9 * 40 && sum[c] > sum[(c + 2) % 3] + 9 * 40) {
            return "rgb"[c];
        }
    }
    return '-';
}

// The fixture tileset without RTC_CENTER, copied to the tmp dir. make_tile_fixture.py writes
// absolute ECEF positions and an RTC_CENTER the stream ignores, but cesium folds RTC_CENTER into
// the positions it generates overlay UVs from (RasterOverlayUtilities.cpp:98): every vertex then
// sits at about twice its ECEF vector, 0.07 degrees south of tile_root, and v clamps to 0. The
// feature table JSON keeps its length (padded with spaces), so the b3dm header stays valid.
std::string tiles_fixture_without_rtc() {
    const std::string dir = std::string(OVERLUME_TMP_DIR) + "/overlume_tiles_fixture_no_rtc";
    std::filesystem::create_directories(dir);
    std::filesystem::copy_file(kTilesFixtureDir + "/tileset.json", dir + "/tileset.json",
                               std::filesystem::copy_options::overwrite_existing);
    for (const char* name : {"tile_root.b3dm", "tile_a.b3dm", "tile_b.b3dm"}) {
        std::ifstream in(kTilesFixtureDir + "/" + name, std::ios::binary);
        std::string b3dm((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        uint32_t jsonLength = 0;
        if (b3dm.size() >= 28) std::memcpy(&jsonLength, b3dm.data() + 12, sizeof(jsonLength));
        if (jsonLength < 18 || b3dm.size() < 28 + static_cast<size_t>(jsonLength)) {
            ADD_FAILURE() << name << " is not a b3dm with a feature table";
            continue;
        }
        std::string json = "{\"BATCH_LENGTH\":1}";
        json.resize(jsonLength, ' ');
        b3dm.replace(28, jsonLength, json);
        std::ofstream(dir + "/" + name, std::ios::binary) << b3dm;
    }
    return dir;
}

// A renderer on `theme` with the fixture tileset in `tiles_dir` open as terrain under the imagery
// in `imagery_dir` and the ego at the fixture block; nullptr without a GPU.
overlume::VisualRenderer* terrain_renderer(const std::string& theme_dir, const char* theme,
                                           const std::string& imagery_dir, double brightness = 1.0,
                                           uint32_t maximum_level = 10,
                                           const std::string& tiles_dir = kTilesFixtureDir) {
    overlume::RenderConfig cfg{320, 240, 1, theme_dir.c_str(), theme};
    auto* r = overlume::create_renderer(cfg);
    if (r == nullptr) return nullptr;
    EXPECT_TRUE(overlume::testing::install_fixture_terrain_with_file_imagery(
        r, tiles_dir.c_str(), imagery_dir.c_str(), brightness, maximum_level));
    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kFixtureBlockCenterMap;
    overlume::set_scene(r, s);
    return r;
}

// Renders kBlockPose until `done()` or 600 frames, then 30 more so the frame settles.
template <typename Done>
void render_block_until(overlume::VisualRenderer* r, std::vector<uint8_t>& frame, Done done) {
    for (int i = 0; i < 600 && !done(); ++i) {
        overlume::render_frame(r, kBlockPose, {frame.data(), 320, 240});
    }
    for (int i = 0; i < 30; ++i) overlume::render_frame(r, kBlockPose, {frame.data(), 320, 240});
}

bool tile_in_scene(overlume::VisualRenderer* r) {
    return overlume::testing::environment_loaded_chunk_count(r) >= 1;
}

bool raster_attached(overlume::VisualRenderer* r) {
    return overlume::testing::environment_stream_overlay_probe(r).rasters_attached >= 1;
}

}

TEST(EnvironmentStreamOverlay, TerrainColorShowsWhileNoRasterHasAttached) {
    const std::string imagery = std::string(OVERLUME_TMP_DIR) + "/overlume_imagery_undecodable";
    write_fixture_imagery(imagery, 0, 0, 0, false);
    auto* r = terrain_renderer(write_terrain_themes(), "terrain_green", imagery);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    std::vector<uint8_t> frame(320u * 240u * 3u);
    render_block_until(r, frame, [&] { return tile_in_scene(r); });
    ASSERT_TRUE(tile_in_scene(r)) << "the fixture tile never reached the scene";

    const overlume::testing::OverlayProbe probe =
        overlume::testing::environment_stream_overlay_probe(r);
    EXPECT_EQ(probe.rasters_attached, 0u) << "every imagery tile is undecodable";
    EXPECT_EQ(probe.textures_alive, 0u);
    const Hues hues = count_hues(frame);
    EXPECT_GT(hues.green, 800u) << "the terrain tile must show the theme's terrain_color (green) "
                                   "while no raster is attached";
    EXPECT_LT(hues.red, 20u);
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStreamOverlay, RasterAttachesAndItsColourCoversTheTile) {
    const std::string imagery = std::string(OVERLUME_TMP_DIR) + "/overlume_imagery_red";
    write_fixture_imagery(imagery, 255, 0, 0, true);
    overlume::testing::capture_stream_glbs(true);
    auto* r = terrain_renderer(write_terrain_themes(), "terrain_green", imagery);
    if (!r) {
        overlume::testing::capture_stream_glbs(false);
        GTEST_SKIP() << "no GPU/EGL";
    }
    std::vector<uint8_t> frame(320u * 240u * 3u);
    render_block_until(r, frame, [&] { return raster_attached(r); });
    const std::vector<std::vector<uint8_t>> glbs = overlume::testing::captured_stream_glbs();
    overlume::testing::capture_stream_glbs(false);

    const overlume::testing::OverlayProbe probe =
        overlume::testing::environment_stream_overlay_probe(r);
    ASSERT_GE(probe.rasters_attached, 1u) << "no raster attached within 600 frames";
    EXPECT_GE(probe.textures_alive, 1u);
    const auto glbs_with = [&](const char* text) {
        return std::count_if(glbs.begin(), glbs.end(), [&](const std::vector<uint8_t>& glb) {
            return glb_json(glb).find(text) != std::string::npos;
        });
    };
    EXPECT_GE(glbs_with("\"TEXCOORD_0\""), 1)
        << "the GLB step must copy _CESIUMOVERLAY_0 into TEXCOORD_0 for terrain tiles";
    EXPECT_EQ(glbs_with("_CESIUMOVERLAY_"), 0) << "gltfio refuses attributes outside the core set";
    const Hues hues = count_hues(frame);
    EXPECT_GT(hues.red, 800u) << "the fixture tile must show the imagery's red";
    EXPECT_LT(hues.green, 20u) << "terrain_color must be gone once the raster attaches";
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStreamOverlay, TeardownDetachesEveryRasterAndFreesEveryTexture) {
    const std::string imagery = std::string(OVERLUME_TMP_DIR) + "/overlume_imagery_red";
    write_fixture_imagery(imagery, 255, 0, 0, true);
    auto* r = terrain_renderer(write_terrain_themes(), "terrain_green", imagery);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    std::vector<uint8_t> frame(320u * 240u * 3u);
    render_block_until(r, frame, [&] { return raster_attached(r); });
    ASSERT_TRUE(raster_attached(r)) << "no raster attached within 600 frames";

    // Tears the tileset down and keeps the source, so its counters stay readable.
    ASSERT_TRUE(overlume::testing::environment_stream_force_fall_back(r));
    const overlume::testing::OverlayProbe probe =
        overlume::testing::environment_stream_overlay_probe(r);
    EXPECT_EQ(probe.rasters_detached, probe.rasters_attached)
        << "every attached raster must be detached before its tile is freed";
    EXPECT_EQ(probe.textures_alive, 0u) << "a raster texture outlived its tileset";
    EXPECT_GE(probe.textures_freed, 1u);
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStreamOverlay, BrightnessScalesTheImagery) {
    const std::string imagery = std::string(OVERLUME_TMP_DIR) + "/overlume_imagery_dim_red";
    write_fixture_imagery(imagery, 100, 0, 0, true);
    const std::string themes = write_terrain_themes();
    const auto red_sum_at = [&](double brightness) -> std::optional<uint64_t> {
        auto* r = terrain_renderer(themes, "terrain_green", imagery, brightness);
        if (!r) return std::nullopt;
        std::vector<uint8_t> frame(320u * 240u * 3u);
        render_block_until(r, frame, [&] { return raster_attached(r); });
        EXPECT_TRUE(raster_attached(r)) << "brightness " << brightness;
        const uint64_t sum = count_hues(frame).red_sum;
        overlume::destroy_renderer(r);
        return sum;
    };
    const std::optional<uint64_t> dim = red_sum_at(1.0);
    if (!dim) GTEST_SKIP() << "no GPU/EGL";
    const std::optional<uint64_t> bright = red_sum_at(3.0);
    ASSERT_TRUE(bright.has_value());
    EXPECT_GT(*bright, *dim + 40000u)
        << "brightness 3 must lift the imagery's red on the tile's ~2,600 pixels (frame red sum "
        << *dim << " at 1, " << *bright << " at 3)";
}

TEST(EnvironmentStreamOverlay, TerrainColorFollowsAThemeSwitchOnLiveTiles) {
    const std::string imagery = std::string(OVERLUME_TMP_DIR) + "/overlume_imagery_undecodable";
    write_fixture_imagery(imagery, 0, 0, 0, false);
    auto* r = terrain_renderer(write_terrain_themes(), "terrain_green", imagery);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    std::vector<uint8_t> frame(320u * 240u * 3u);
    render_block_until(r, frame, [&] { return tile_in_scene(r); });
    ASSERT_GT(count_hues(frame).green, 800u) << "test setup: the green terrain tile";

    // A transition that ended 9.9 s before the scene's sim time 0: the next frame is terrain_red.
    ASSERT_TRUE(overlume::set_theme(r, "terrain_red", -10.0, 0.1));
    for (int i = 0; i < 3; ++i) overlume::render_frame(r, kBlockPose, {frame.data(), 320, 240});
    const Hues hues = count_hues(frame);
    EXPECT_GT(hues.red, 800u) << "a live terrain tile must take the new theme's terrain_color";
    EXPECT_LT(hues.green, 20u);
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStreamOverlay, ImageryKeepsNorthUp) {
    // tile_root's region centre, where make_tile_fixture.py stands its five buildings (at the
    // region's mid height); the imagery is red north of this latitude and blue south of it.
    constexpr double kCentreLatDeg = 25.080924750000005;
    constexpr double kCentreLonDeg = 55.386325500000005;
    constexpr double kBaseHeightM = -2.7436;
    double x0 = 0, y0 = 0, z0 = 0;
    ASSERT_TRUE(overlume::testing::ecef_to_map_probe(
        kFixtureAnchor.origin_lat_deg, kFixtureAnchor.origin_lon_deg, 0.0, 0.0, kCentreLatDeg,
        kCentreLonDeg, kBaseHeightM, &x0, &y0, &z0));
    // Map x is north and y west. Each point is a building's centre at half its height, on that
    // building's own faces from any side. North: E 30.4, N 29.5, 11.8 x 16.0 m, 26.1 m tall.
    // South: E -13.1, N -25.4, 14.4 x 8.2 m, 11.2 m tall.
    const overlume::Vec3 northBuilding{x0 + 29.5, y0 - 30.4, z0 + 13.0};
    const overlume::Vec3 southBuilding{x0 - 25.4, y0 + 13.1, z0 + 5.6};

    // Level 14: 8.7 m texels, so both buildings lie more than two texels from the colour line.
    const std::string imagery = std::string(OVERLUME_TMP_DIR) + "/overlume_imagery_north_red";
    write_fixture_imagery(imagery, true, 14, [](double lat_deg) {
        return lat_deg > kCentreLatDeg ? Rgb{255, 0, 0} : Rgb{0, 0, 255};
    });
    // On the shipped fixture, cesium folds RTC_CENTER into the overlay UVs and both probes read
    // the raster's south edge (API notes); this copy drops it.
    auto* r = terrain_renderer(write_terrain_themes(), "terrain_green", imagery, 1.0, 14,
                               tiles_fixture_without_rtc());
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    std::vector<uint8_t> frame(320u * 240u * 3u);
    render_block_until(r, frame, [&] { return raster_attached(r); });
    ASSERT_TRUE(raster_attached(r)) << "no raster attached within 600 frames";

    EXPECT_EQ(hue_at(r, frame, northBuilding), 'r')
        << "the building north of the colour line must show the imagery's north (red); 'b' means "
           "the raster is mirrored north-south, 'g' that terrain_color still shows";
    EXPECT_EQ(hue_at(r, frame, southBuilding), 'b')
        << "the building south of the colour line must show the imagery's south (blue); 'r' means "
           "the raster is mirrored or sampled at one corner (translation and scale swapped)";
    overlume::destroy_renderer(r);
}
```

The thresholds rest on the measured block area: 800 is under a third of the 2,600 block pixels,
20 is noise; the brightness margin, 40,000, is about a third of 2,600 pixels times the 40-50 red
levels a tripled albedo of 0.127 gains on lit and shaded faces. `ImageryKeepsNorthUp` asserts no
count: each probe reads one building through a 3x3 window, and the window stays on it (the north
building is about 20 px across at its 130 m from the eye, the south one about 10 px at 190 m).
It opens `tiles_fixture_without_rtc()`, a tmp copy of the fixture whose b3dm feature tables lose
`RTC_CENTER`: the stream places the copy exactly as the original, but cesium's overlay UVs are then
the tile's own (API notes, "The fixture's `RTC_CENTER`").

- [ ] **Step 7: Reconfigure (matc compiles `terrain.mat`), build, and see the tests fail**

```bash
cd "$WORKTREE"
flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock cmake -S overlume -B overlume/build > /dev/null \
    && echo reconfigured \
    && CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
ls overlume/build/generated/overlume_materials/terrain_filamat.h
```

No `reconfigured` line and no CMake error above it means the lock wait ran out (exit 75) and
nothing was built: run the step again. Expected: `reconfigured`, then the build stops in
`test_environment_stream.cpp` with errors that include

```
error: no member named 'install_fixture_terrain_with_file_imagery' in namespace 'overlume::testing'
error: no member named 'environment_stream_overlay_probe' in namespace 'overlume::testing'
error: no type named 'OverlayProbe' in namespace 'overlume::testing'
```

and `ls` prints the generated header's path (a `matc failed on .../terrain.mat` CMake error instead
means the material text differs from Step 5).

- [ ] **Step 8: The renderer builds `terrain.mat` and applies the theme token**

In `overlume/src/renderer_internal.hpp`, replace

```cpp
    filament::MaterialInstance* buildingMaterial = nullptr;
    std::unique_ptr<EnvironmentSource> environmentSource;
```

with

```cpp
    filament::MaterialInstance* buildingMaterial = nullptr;
    // terrain.mat. Each streamed terrain tile gets an instance (environment_stream.cpp), copied
    // from the default instance: the theme's terrain_color, no imagery, the placeholder bound.
    filament::Material* terrainMaterial = nullptr;
    filament::Texture* terrainPlaceholderTexture = nullptr;  // 1x1 white, while no raster attaches
    std::unique_ptr<EnvironmentSource> environmentSource;
```

In `overlume/src/renderer.cpp`, replace

```cpp
#include <filament/SwapChain.h>
#include <filament/TransformManager.h>
```

with

```cpp
#include <filament/SwapChain.h>
#include <filament/TextureSampler.h>
#include <filament/TransformManager.h>
```

In `overlume/src/renderer.cpp`, replace

```cpp
#include "trajectory_carpet_faded_filamat.h"
```

with

```cpp
#include "trajectory_carpet_faded_filamat.h"
#include "terrain_filamat.h"
```

In `overlume/src/renderer.cpp` (the end of `push_theme_to_scene`), replace

```cpp
    r.buildingMaterial->setParameter("metallic", theme.material.metallic);
}
```

with

```cpp
    r.buildingMaterial->setParameter("metallic", theme.material.metallic);

    // New terrain tiles copy it; StreamRendererResources::sync_terrain_color recolours live ones.
    r.terrainMaterial->getDefaultInstance()->setParameter(
        "terrainColor", to_filament(theme.environment.terrain_color));
}
```

In `overlume/src/renderer.cpp` (`create_renderer`), replace

```cpp
    r->buildingMaterial = r->clayMaterial->createInstance();

```

with

```cpp
    r->buildingMaterial = r->clayMaterial->createInstance();

    r->terrainMaterial =
        filament::Material::Builder()
            .package(overlume::materials::kterrainFilamat, overlume::materials::kterrainFilamatSize)
            .build(*engine);
    r->terrainPlaceholderTexture = filament::Texture::Builder()
                                       .width(1)
                                       .height(1)
                                       .levels(1)
                                       .format(filament::Texture::InternalFormat::RGBA8)
                                       .sampler(filament::Texture::Sampler::SAMPLER_2D)
                                       .build(*engine);
    static const uint8_t kPlaceholderTexel[4] = {255, 255, 255, 255};
    r->terrainPlaceholderTexture->setImage(
        *engine, 0,
        filament::backend::PixelBufferDescriptor(
            kPlaceholderTexel, sizeof(kPlaceholderTexel),
            filament::backend::PixelBufferDescriptor::PixelDataFormat::RGBA,
            filament::backend::PixelBufferDescriptor::PixelDataType::UBYTE));
    // Filament's createInstance() copies the default instance: every terrain tile starts here.
    filament::MaterialInstance* terrainDefaults = r->terrainMaterial->getDefaultInstance();
    terrainDefaults->setParameter("imagery", r->terrainPlaceholderTexture,
                                  filament::TextureSampler());
    terrainDefaults->setParameter("hasImagery", int32_t{0});
    terrainDefaults->setParameter("uvTransform", float4{0.0f, 0.0f, 1.0f, 1.0f});
    terrainDefaults->setParameter("brightness", 1.0f);

```

(`push_theme_to_scene` runs after this, at `renderer.cpp:825`, so the default instance exists when
it sets `terrainColor`.)

In `overlume/src/renderer.cpp` (`destroy_renderer`, after the environment teardown at 926-928),
replace

```cpp
    if (r->buildingMaterial) r->engine->destroy(r->buildingMaterial);
```

with

```cpp
    if (r->buildingMaterial) r->engine->destroy(r->buildingMaterial);
    // After the environment teardown above (it destroys the tiles' instances); the material's
    // default instance holds the placeholder, so the texture goes last.
    if (r->terrainMaterial) r->engine->destroy(r->terrainMaterial);
    if (r->terrainPlaceholderTexture) r->engine->destroy(r->terrainPlaceholderTexture);
```

- [ ] **Step 9: Header: `Kind`, the callbacks, the counters, the source's seams**

In `overlume/src/environment_stream.hpp`, replace

```cpp
#include <atomic>
```

with

```cpp
#include <array>
#include <atomic>
```

In `overlume/src/environment_stream.hpp`, replace

```cpp
namespace spdlog {
class logger;
}
```

with

```cpp
namespace spdlog {
class logger;
}

namespace filament {
class MaterialInstance;
class Texture;
}
```

In `overlume/src/environment_stream.hpp`, replace the whole of

```cpp
class StreamRendererResources final : public Cesium3DTilesSelection::IPrepareRendererResources {
public:
    explicit StreamRendererResources(const glm::dmat4& ecefToMap, double anchorHeightM,
                                     bool materialsOriginal = false, double brightness = 1.0)
        : ecefToMap_(ecefToMap),
          anchorHeightM_(anchorHeightM),
          materialsOriginal_(materialsOriginal),
          brightness_(brightness) {}

    void set_renderer(VisualRenderer* r) { r_ = r; }

    CesiumAsync::Future<Cesium3DTilesSelection::TileLoadResultAndRenderResources>
    prepareInLoadThread(const CesiumAsync::AsyncSystem& asyncSystem,
                        Cesium3DTilesSelection::TileLoadResult&& tileLoadResult,
                        const glm::dmat4& transform, const std::any& rendererOptions) override;
    void* prepareInMainThread(Cesium3DTilesSelection::Tile& tile, void* pLoadThreadResult) override;
    void free(Cesium3DTilesSelection::Tile& tile, void* pLoadThreadResult,
              void* pMainThreadResult) noexcept override;

    void* prepareRasterInLoadThread(CesiumImage::ImageAsset&, const std::any&) override {
        return nullptr;
    }
    void* prepareRasterInMainThread(CesiumRasterOverlays::RasterOverlayTile&, void*) override {
        return nullptr;
    }
    void freeRaster(const CesiumRasterOverlays::RasterOverlayTile&, void*,
                    void*) noexcept override {}
    void attachRasterInMainThread(const Cesium3DTilesSelection::Tile&, int32_t,
                                  const CesiumRasterOverlays::RasterOverlayTile&, void*,
                                  const glm::dvec2&, const glm::dvec2&) override {}
    void detachRasterInMainThread(const Cesium3DTilesSelection::Tile&, int32_t,
                                  const CesiumRasterOverlays::RasterOverlayTile&,
                                  void*) noexcept override {}

    std::vector<filament::gltfio::FilamentAsset*> drain_pending_frees();
    void note_torn_down() { tornDown_.store(true); }

    void set_terrain_transform(double z, double tilt_rad, double pivot_x, double pivot_y,
                               double heading_rad);
    void destroy_terrain_root();

private:
    void ensure_terrain_root();

    VisualRenderer* r_ = nullptr;
    glm::dmat4 ecefToMap_;
    double anchorHeightM_ = 0.0;
    bool materialsOriginal_ = false;
    double brightness_ = 1.0;
    std::mutex freeMutex_;
    std::vector<filament::gltfio::FilamentAsset*> pendingFrees_;
    std::atomic<bool> tornDown_{false};
    utils::Entity terrainRoot_;
};

struct LoadThreadGlb {
    std::vector<std::byte> glbBytes;
    glm::dmat4 transform{1.0};
    bool ok = false;
};
```

with

```cpp
class StreamRendererResources final : public Cesium3DTilesSelection::IPrepareRendererResources {
public:
    // Content tiles get the clay building material (or keep their own with materials=original).
    // Terrain tiles get a terrain.mat instance each and take raster overlays through the five
    // overlay callbacks; Content resources never see a raster.
    enum class Kind : uint8_t { Content, Terrain };
    struct OverlayCounters {
        uint32_t rasters_attached = 0, rasters_detached = 0, textures_alive = 0, textures_freed = 0;
    };

    explicit StreamRendererResources(const glm::dmat4& ecefToMap, double anchorHeightM,
                                     bool materialsOriginal = false, double brightness = 1.0,
                                     Kind kind = Kind::Content)
        : ecefToMap_(ecefToMap),
          anchorHeightM_(anchorHeightM),
          materialsOriginal_(materialsOriginal),
          brightness_(brightness),
          kind_(kind) {}

    void set_renderer(VisualRenderer* r) { r_ = r; }

    CesiumAsync::Future<Cesium3DTilesSelection::TileLoadResultAndRenderResources>
    prepareInLoadThread(const CesiumAsync::AsyncSystem& asyncSystem,
                        Cesium3DTilesSelection::TileLoadResult&& tileLoadResult,
                        const glm::dmat4& transform, const std::any& rendererOptions) override;
    void* prepareInMainThread(Cesium3DTilesSelection::Tile& tile, void* pLoadThreadResult) override;
    void free(Cesium3DTilesSelection::Tile& tile, void* pLoadThreadResult,
              void* pMainThreadResult) noexcept override;

    // Load thread: the decoded image as RGBA8 (RasterLoadThreadResult). Main thread: one
    // mipmapped sRGB texture per raster tile. Attach: the tile's instance samples it through
    // cesium's translation and scale. Detach: back to terrainColor. Free: the texture joins the
    // deferred frees that drain_pending_frees destroys.
    void* prepareRasterInLoadThread(CesiumImage::ImageAsset& image,
                                    const std::any& rendererOptions) override;
    void* prepareRasterInMainThread(CesiumRasterOverlays::RasterOverlayTile& rasterTile,
                                    void* pLoadThreadResult) override;
    void freeRaster(const CesiumRasterOverlays::RasterOverlayTile& rasterTile,
                    void* pLoadThreadResult, void* pMainThreadResult) noexcept override;
    void attachRasterInMainThread(const Cesium3DTilesSelection::Tile& tile,
                                  int32_t overlayTextureCoordinateID,
                                  const CesiumRasterOverlays::RasterOverlayTile& rasterTile,
                                  void* pMainThreadRendererResources, const glm::dvec2& translation,
                                  const glm::dvec2& scale) override;
    void detachRasterInMainThread(const Cesium3DTilesSelection::Tile& tile,
                                  int32_t overlayTextureCoordinateID,
                                  const CesiumRasterOverlays::RasterOverlayTile& rasterTile,
                                  void* pMainThreadRendererResources) noexcept override;

    std::vector<filament::gltfio::FilamentAsset*> drain_pending_frees();
    // Main thread: pushes the active theme's environment.terrain_color to every live terrain tile
    // when it changed (new tiles copy it from terrain.mat's default instance).
    void sync_terrain_color();
    OverlayCounters overlay_counters() const { return counters_; }
    void note_torn_down() { tornDown_.store(true); }

    void set_terrain_transform(double z, double tilt_rad, double pivot_x, double pivot_y,
                               double heading_rad);
    void destroy_terrain_root();

private:
    void ensure_terrain_root();
    filament::MaterialInstance* terrain_instance(const Cesium3DTilesSelection::Tile& tile) const;

    VisualRenderer* r_ = nullptr;
    glm::dmat4 ecefToMap_;
    double anchorHeightM_ = 0.0;
    bool materialsOriginal_ = false;
    double brightness_ = 1.0;
    Kind kind_ = Kind::Content;
    std::mutex freeMutex_;
    std::vector<filament::gltfio::FilamentAsset*> pendingFrees_;
    std::vector<filament::Texture*> pendingTextureFrees_;  // under freeMutex_
    std::atomic<bool> tornDown_{false};
    utils::Entity terrainRoot_;
    // Main thread only: each terrain tile's instance, keyed by its FilamentAsset.
    std::unordered_map<const void*, filament::MaterialInstance*> terrainInstances_;
    std::array<float, 3> terrainColor_{-1.0f, -1.0f, -1.0f};  // last pushed by sync_terrain_color
    OverlayCounters counters_;                                // main thread only
};

struct LoadThreadGlb {
    std::vector<std::byte> glbBytes;
    glm::dmat4 transform{1.0};
    bool ok = false;
};

// One raster overlay tile's pixels, RGBA8, row 0 at the north edge, load thread to main thread.
struct RasterLoadThreadResult {
    uint32_t width = 0;
    uint32_t height = 0;
    std::vector<uint8_t> rgba;
};
```

In `overlume/src/environment_stream.hpp`, replace

```cpp
                               std::optional<double> radius_override_m = std::nullopt,
                               CustomTileset custom_tileset = {});
```

with

```cpp
                               std::optional<double> radius_override_m = std::nullopt,
                               CustomTileset custom_tileset = {},
                               StreamRendererResources::Kind resources_kind = {});
```

(`= {}` is `Kind::Content`; the spelled-out default would reflow the whole declaration under
clang-format.)

In `overlume/src/environment_stream.hpp`, replace

```cpp
    void force_fall_back_for_testing(VisualRenderer& r) { fall_back(r); }
```

with

```cpp
    void force_fall_back_for_testing(VisualRenderer& r) { fall_back(r); }

    StreamRendererResources::OverlayCounters overlay_counters() const {
        return renderResources_->overlay_counters();
    }
    // Test hooks add raster overlays here before the first update().
    Cesium3DTilesSelection::RasterOverlayCollection& overlays_for_testing() {
        return tileset_->getOverlays();
    }
```

- [ ] **Step 10: Source: the UV copy, the terrain swap, the five callbacks, the drain, the sync**

In `overlume/src/environment_stream.cpp`, replace

```cpp
#include <CesiumGltfWriter/GltfWriter.h>
```

with

```cpp
#include <CesiumGltfWriter/GltfWriter.h>
#include <CesiumImage/ImageAsset.h>
#include <CesiumRasterOverlays/UrlTemplateRasterOverlay.h>
```

In `overlume/src/environment_stream.cpp`, replace

```cpp
#include <filament/RenderableManager.h>
#include <filament/TransformManager.h>
```

with

```cpp
#include <filament/RenderableManager.h>
#include <filament/Texture.h>
#include <filament/TextureSampler.h>
#include <filament/TransformManager.h>

#include <backend/PixelBufferDescriptor.h>
```

In `overlume/src/environment_stream.cpp`, replace

```cpp
void strip_attributes_and_correct_heights(CesiumGltf::Model& model, const glm::dmat4& modelToEcef,
```

with

```cpp
// Terrain tiles: cesium writes overlay n's texture coordinates as _CESIUMOVERLAY_<n>, which the
// strip below drops (gltfio refuses non-core attributes). TEXCOORD_0 keeps overlay 0's: gltfio
// loads it as UV0 when the tile's own material samples no texture (AssetLoader.cpp, the UNUSED
// slot), and terrain.mat reads UV0.
void copy_overlay_uvs_to_texcoord0(CesiumGltf::Model& model) {
    for (CesiumGltf::Mesh& mesh : model.meshes) {
        for (CesiumGltf::MeshPrimitive& prim : mesh.primitives) {
            const auto it = prim.attributes.find("_CESIUMOVERLAY_0");
            if (it == prim.attributes.end()) continue;
            const int32_t accessor = it->second;
            prim.attributes["TEXCOORD_0"] = accessor;
        }
    }
}

void strip_attributes_and_correct_heights(CesiumGltf::Model& model, const glm::dmat4& modelToEcef,
```

In `overlume/src/environment_stream.cpp` (`prepareInLoadThread`), replace

```cpp
        if (consolidate_buffers(*model)) {
            strip_attributes_and_correct_heights(*model, pGlb->transform, ecefToMap_,
```

with

```cpp
        if (consolidate_buffers(*model)) {
            if (kind_ == Kind::Terrain) copy_overlay_uvs_to_texcoord0(*model);
            strip_attributes_and_correct_heights(*model, pGlb->transform, ecefToMap_,
```

In `overlume/src/environment_stream.cpp` (`prepareInMainThread`), replace

```cpp
    asset->releaseSourceData();

    filament::RenderableManager& rm = r_->engine->getRenderableManager();
    const utils::Entity* renderables = asset->getRenderableEntities();
    const size_t renderableCount = asset->getRenderableEntityCount();
    for (size_t i = 0; i < renderableCount; ++i) {
        const auto inst = rm.getInstance(renderables[i]);
        if (!inst.isValid()) continue;
        const size_t primCount = rm.getPrimitiveCount(inst);
        if (!materialsOriginal_) {
            for (size_t p = 0; p < primCount; ++p) {
                rm.setMaterialInstanceAt(inst, p, r_->buildingMaterial);
            }
        }
```

with

```cpp
    asset->releaseSourceData();

    // Terrain: one instance per tile, so each tile can carry its own raster. Content: the shared
    // clay instance unless materials=original.
    filament::MaterialInstance* swap = materialsOriginal_ ? nullptr : r_->buildingMaterial;
    if (kind_ == Kind::Terrain) {
        swap = r_->terrainMaterial->createInstance();
        swap->setParameter("brightness", static_cast<float>(brightness_));
        terrainInstances_[asset] = swap;
    }
    filament::RenderableManager& rm = r_->engine->getRenderableManager();
    const utils::Entity* renderables = asset->getRenderableEntities();
    const size_t renderableCount = asset->getRenderableEntityCount();
    for (size_t i = 0; i < renderableCount; ++i) {
        const auto inst = rm.getInstance(renderables[i]);
        if (!inst.isValid()) continue;
        const size_t primCount = rm.getPrimitiveCount(inst);
        if (swap != nullptr) {
            for (size_t p = 0; p < primCount; ++p) rm.setMaterialInstanceAt(inst, p, swap);
        }
```

In `overlume/src/environment_stream.cpp`, replace

```cpp
std::vector<filament::gltfio::FilamentAsset*> StreamRendererResources::drain_pending_frees() {
    std::vector<filament::gltfio::FilamentAsset*> toFree;
    {
        std::lock_guard<std::mutex> lock(freeMutex_);
        toFree.swap(pendingFrees_);
    }
    if (r_ == nullptr) return {};
    for (filament::gltfio::FilamentAsset* asset : toFree) {
        r_->scene->removeEntities(asset->getEntities(), asset->getEntityCount());
        r_->sharedAssetLoader->destroyAsset(asset);
    }
    return toFree;
}
```

with

```cpp
namespace {
// Trilinear, clamped: raster tiles meet at tile edges, so no wrap.
const filament::TextureSampler kImagerySampler(
    filament::TextureSampler::MinFilter::LINEAR_MIPMAP_LINEAR,
    filament::TextureSampler::MagFilter::LINEAR, filament::TextureSampler::WrapMode::CLAMP_TO_EDGE);
}

void* StreamRendererResources::prepareRasterInLoadThread(CesiumImage::ImageAsset& image,
                                                         const std::any&) {
    // ponytail: 8-bit RGB or RGBA, what cesium decodes PNG and JPEG to; any other layout attaches
    // nothing. KTX2 or 16-bit imagery needs a conversion here.
    const size_t pixels = static_cast<size_t>(std::max(image.width, 0)) *
                          static_cast<size_t>(std::max(image.height, 0));
    const size_t channels = static_cast<size_t>(image.channels);
    if (pixels == 0 || image.bytesPerChannel != 1 || (channels != 3 && channels != 4) ||
        image.compressedPixelFormat != CesiumImage::GpuCompressedPixelFormat::NONE ||
        image.pixelData.size() < pixels * channels) {
        return nullptr;
    }
    auto* out = new RasterLoadThreadResult{static_cast<uint32_t>(image.width),
                                           static_cast<uint32_t>(image.height),
                                           std::vector<uint8_t>(pixels * 4, 255)};
    const auto* src = reinterpret_cast<const uint8_t*>(image.pixelData.data());
    for (size_t i = 0; i < pixels; ++i) std::memcpy(&out->rgba[i * 4], &src[i * channels], 3);
    return out;
}

void* StreamRendererResources::prepareRasterInMainThread(CesiumRasterOverlays::RasterOverlayTile&,
                                                         void* pLoadThreadResult) {
    std::unique_ptr<RasterLoadThreadResult> pixels(
        static_cast<RasterLoadThreadResult*>(pLoadThreadResult));
    if (!pixels || tornDown_.load() || r_ == nullptr) return nullptr;
    filament::Engine& engine = *r_->engine;
    filament::Texture* texture = filament::Texture::Builder()
                                     .width(pixels->width)
                                     .height(pixels->height)
                                     .levels(0xff)  // clamped to the full chain
                                     .format(filament::Texture::InternalFormat::SRGB8_A8)
                                     .sampler(filament::Texture::Sampler::SAMPLER_2D)
                                     .build(engine);
    auto* heap = new std::vector<uint8_t>(std::move(pixels->rgba));
    texture->setImage(
        engine, 0,
        filament::backend::PixelBufferDescriptor(
            heap->data(), heap->size(),
            filament::backend::PixelBufferDescriptor::PixelDataFormat::RGBA,
            filament::backend::PixelBufferDescriptor::PixelDataType::UBYTE,
            [](void*, size_t, void* user) { delete static_cast<std::vector<uint8_t>*>(user); },
            heap));
    texture->generateMipmaps(engine);
    ++counters_.textures_alive;
    return texture;
}

void StreamRendererResources::freeRaster(const CesiumRasterOverlays::RasterOverlayTile&,
                                         void* pLoadThreadResult,
                                         void* pMainThreadResult) noexcept {
    delete static_cast<RasterLoadThreadResult*>(pLoadThreadResult);
    if (pMainThreadResult == nullptr || tornDown_.load()) return;
    std::lock_guard<std::mutex> lock(freeMutex_);
    pendingTextureFrees_.push_back(static_cast<filament::Texture*>(pMainThreadResult));
}

filament::MaterialInstance* StreamRendererResources::terrain_instance(
    const Cesium3DTilesSelection::Tile& tile) const {
    const Cesium3DTilesSelection::TileRenderContent* content = tile.getContent().getRenderContent();
    if (content == nullptr) return nullptr;
    const auto it = terrainInstances_.find(content->getRenderResources());
    return it != terrainInstances_.end() ? it->second : nullptr;
}

void StreamRendererResources::attachRasterInMainThread(
    const Cesium3DTilesSelection::Tile& tile, int32_t overlayTextureCoordinateID,
    const CesiumRasterOverlays::RasterOverlayTile&, void* pMainThreadRendererResources,
    const glm::dvec2& translation, const glm::dvec2& scale) {
    filament::MaterialInstance* mi = terrain_instance(tile);
    auto* texture = static_cast<filament::Texture*>(pMainThreadRendererResources);
    // ponytail: one overlay per terrain tileset, read from TEXCOORD_0; a second overlay's rasters
    // are ignored. A second sampler and UV set are the upgrade.
    if (mi == nullptr || texture == nullptr || overlayTextureCoordinateID != 0) return;
    mi->setParameter("imagery", texture, kImagerySampler);
    mi->setParameter(
        "uvTransform",
        filament::math::float4{static_cast<float>(translation.x), static_cast<float>(translation.y),
                               static_cast<float>(scale.x), static_cast<float>(scale.y)});
    mi->setParameter("hasImagery", int32_t{1});
    ++counters_.rasters_attached;
}

void StreamRendererResources::detachRasterInMainThread(
    const Cesium3DTilesSelection::Tile& tile, int32_t overlayTextureCoordinateID,
    const CesiumRasterOverlays::RasterOverlayTile&, void* pMainThreadRendererResources) noexcept {
    filament::MaterialInstance* mi = terrain_instance(tile);
    if (mi == nullptr || pMainThreadRendererResources == nullptr ||
        overlayTextureCoordinateID != 0 || r_ == nullptr) {
        return;
    }
    // The placeholder goes back in, so no instance still holds a texture freeRaster may destroy.
    mi->setParameter("imagery", r_->terrainPlaceholderTexture, kImagerySampler);
    mi->setParameter("hasImagery", int32_t{0});
    ++counters_.rasters_detached;
}

void StreamRendererResources::sync_terrain_color() {
    if (r_ == nullptr) return;
    const detail::Float3& c = r_->active_theme.environment.terrain_color;
    if (terrainColor_ == std::array<float, 3>{c.r, c.g, c.b}) return;
    terrainColor_ = {c.r, c.g, c.b};
    for (const auto& [asset, mi] : terrainInstances_) {
        mi->setParameter("terrainColor", filament::math::float3{c.r, c.g, c.b});
    }
}

std::vector<filament::gltfio::FilamentAsset*> StreamRendererResources::drain_pending_frees() {
    std::vector<filament::gltfio::FilamentAsset*> toFree;
    std::vector<filament::Texture*> textures;
    {
        std::lock_guard<std::mutex> lock(freeMutex_);
        toFree.swap(pendingFrees_);
        textures.swap(pendingTextureFrees_);
    }
    if (r_ == nullptr) return {};
    for (filament::gltfio::FilamentAsset* asset : toFree) {
        r_->scene->removeEntities(asset->getEntities(), asset->getEntityCount());
        auto terrain = terrainInstances_.extract(asset);
        r_->sharedAssetLoader->destroyAsset(asset);
        // After the asset: its renderables held the instance.
        if (!terrain.empty()) r_->engine->destroy(terrain.mapped());
    }
    for (filament::Texture* texture : textures) {
        r_->engine->destroy(texture);
        --counters_.textures_alive;
        ++counters_.textures_freed;
    }
    return toFree;
}
```

In `overlume/src/environment_stream.cpp` (the constructor), replace

```cpp
    double max_tilt_deg, double brightness, std::optional<double> radius_override_m,
    CustomTileset custom_tileset)
```

with

```cpp
    double max_tilt_deg, double brightness, std::optional<double> radius_override_m,
    CustomTileset custom_tileset, StreamRendererResources::Kind resources_kind)
```

and replace

```cpp
    renderResources_ = std::make_shared<StreamRendererResources>(
        ecefToMap_, anchor_.origin_height_m, materialsOriginal_, brightness_);
```

with

```cpp
    renderResources_ = std::make_shared<StreamRendererResources>(
        ecefToMap_, anchor_.origin_height_m, materialsOriginal_, brightness_, resources_kind);
```

In `overlume/src/environment_stream.cpp` (`StreamingEnvironmentSource::update`), replace

```cpp
    renderResources_->set_renderer(&r);
    for (filament::gltfio::FilamentAsset* freed : renderResources_->drain_pending_frees()) {
```

with

```cpp
    renderResources_->set_renderer(&r);
    renderResources_->sync_terrain_color();
    for (filament::gltfio::FilamentAsset* freed : renderResources_->drain_pending_frees()) {
```

- [ ] **Step 11: The two hooks**

In `overlume/src/environment_test_hooks.hpp`, append at the end of the file: replace its last
declaration

```cpp
bool renderer_ground_plane_in_scene(overlume::VisualRenderer* r);
```

with

```cpp
bool renderer_ground_plane_in_scene(overlume::VisualRenderer* r);

// The raster-overlay callbacks' counters of the open streaming source's renderer resources;
// zeros when no streaming source is open (and always for content resources, which never attach).
struct OverlayProbe {
    uint32_t rasters_attached, rasters_detached, textures_alive, textures_freed;
};
OverlayProbe environment_stream_overlay_probe(overlume::VisualRenderer* r);

// Opens `fixture_dir`'s tileset.json (the file-based fixture tileset, as
// install_fixture_streaming_source) as a terrain tileset at the fixture tests' anchor
// (25.0803 N, 55.3910 E, heading 0, height 0), without a disk cache, under a
// UrlTemplateRasterOverlay on `file://<imagery_dir>/{z}/{x}/{y}.png` capped at `maximum_level`
// (the fixture's leaves ask for level 17). cesium counts {y} from the south edge of the
// web-mercator square.
bool install_fixture_terrain_with_file_imagery(overlume::VisualRenderer* r, const char* fixture_dir,
                                               const char* imagery_dir, double brightness = 1.0,
                                               uint32_t maximum_level = 10);
```

In `overlume/src/environment_stream.cpp`, replace

```cpp
std::vector<std::pair<int, int>> fixture_grid_requested_leaves(FixtureGridHandle* handle) {
    if (handle == nullptr) return {};
    std::lock_guard<std::mutex> lock(handle->log->mutex);
    return handle->log->leaves;
}
```

with

```cpp
std::vector<std::pair<int, int>> fixture_grid_requested_leaves(FixtureGridHandle* handle) {
    if (handle == nullptr) return {};
    std::lock_guard<std::mutex> lock(handle->log->mutex);
    return handle->log->leaves;
}

bool install_fixture_terrain_with_file_imagery(overlume::VisualRenderer* r, const char* fixture_dir,
                                               const char* imagery_dir, double brightness,
                                               uint32_t maximum_level) {
    if (r == nullptr || fixture_dir == nullptr || imagery_dir == nullptr) return false;
    // The anchor every fixture test uses (kFixtureAnchor in test_environment_stream.cpp).
    constexpr overlume::GeoAnchor kAnchor{25.0803, 55.3910, 0.0, 0.0};
    CesiumAsync::AsyncSystem asyncSystem(std::make_shared<overlume::SimpleTaskProcessor>());
    std::shared_ptr<overlume::CountingAssetAccessor> counting;
    Cesium3DTilesSelection::TilesetExternals externals =
        overlume::build_externals(std::make_shared<overlume::FileFixtureAssetAccessor>(),
                                  asyncSystem, "off", overlume::kDefaultMaxCacheItems, &counting);
    auto source = std::make_unique<overlume::StreamingEnvironmentSource>(
        externals, 0, std::string(), std::string("file://") + fixture_dir + "/tileset.json",
        std::string(), kAnchor, std::move(counting), false, false, 0.0, true, 2.0, brightness,
        std::nullopt, overlume::CustomTileset{}, overlume::StreamRendererResources::Kind::Terrain);
    // At the default level 10 one tile holds the fixture's whole region, so a test writes a few
    // files. cesium fills {y} counting rows from the south (QuadtreeTilingScheme::tileToRectangle).
    CesiumRasterOverlays::UrlTemplateRasterOverlayOptions options;
    options.maximumLevel = maximum_level;
    source->overlays_for_testing().add(new CesiumRasterOverlays::UrlTemplateRasterOverlay(
        "fixture-imagery", std::string("file://") + imagery_dir + "/{z}/{x}/{y}.png", {}, options));
    source->set_visible(*r, r->environmentVisible);
    if (r->environmentSource) r->environmentSource->teardown(*r);
    r->environmentSource = std::move(source);
    return true;
}

OverlayProbe environment_stream_overlay_probe(overlume::VisualRenderer* r) {
    if (r == nullptr || !r->environmentSource) return {};
    auto* stream = dynamic_cast<overlume::StreamingEnvironmentSource*>(r->environmentSource.get());
    if (stream == nullptr) return {};
    const overlume::StreamRendererResources::OverlayCounters c = stream->overlay_counters();
    return {c.rasters_attached, c.rasters_detached, c.textures_alive, c.textures_freed};
}
```

- [ ] **Step 12: Run the tests and see them pass, with the neighbours they could disturb**

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    && overlume/build/test_environment_stream --gtest_filter='EnvironmentStreamOverlay.*' \
    && overlume/build/test_environment_stream --gtest_filter='EnvironmentStream.FixtureTilesLoadRenderAsClayAndCount:EnvironmentStream.ClayModeRemapsFirstPrimitiveToBuildingMaterial:EnvironmentStream.OriginalModeSkipsClayRemapOnFirstPrimitive:EnvironmentStreamGolden.FixtureBlock_DarkAdas' \
    && overlume/build/test_theme --gtest_filter='ThemeEnvironment.*'
```

Expected: a clean build; the first run ends

```
[==========] 6 tests from 1 test suite ran. (… ms total)
[  PASSED  ] 6 tests.
```

the second `[  PASSED  ] 4 tests.` (content tiles still swap to clay, or keep their materials, and
the golden is unchanged) and the third `[  PASSED  ] 2 tests.` If a count probe misses its
threshold (800 or 40,000), print the measured `count_hues` values before changing a number: a
miss means the block is not where the pose expects it or the colour does not reach the frame, and
either is a finding for the review gate, not a threshold to lower. If `ImageryKeepsNorthUp` reads
'-' at a probe, write the frame out (`stbi_write_png`) and look where the point lands before
moving it; a swapped 'r'/'b' is an orientation bug, never a probe to swap. If both probes read
'b', first check that the test opens `tiles_fixture_without_rtc()` (the shipped fixture's
`RTC_CENTER` clamps every v to the south edge) before suspecting the shader.

- [ ] **Step 13: Format**

```bash
cd "$WORKTREE"
CPP="overlume/src/environment_stream.hpp overlume/src/environment_stream.cpp \
    overlume/src/environment_test_hooks.hpp overlume/src/renderer.cpp \
    overlume/src/renderer_internal.hpp overlume/src/theme.hpp overlume/src/theme.cpp \
    overlume/src/theme_transition.cpp overlume/tests/test_environment_stream.cpp \
    overlume/tests/test_theme.cpp overlume/tests/test_theme_transition.cpp"
OTHER="overlume/assets/materials/terrain.mat overlume/assets/themes/dark_adas.yaml \
    overlume/assets/themes/light_clay.yaml"
git add $CPP $OTHER
tools/check_format.sh --fix $CPP
git diff --stat
git add $CPP
tools/check_format.sh && tools/check_spdx.sh
```

Expected: `check_format.sh: reformatted 11 files with clang-format 23.1.1`; `git diff --stat`
prints nothing (the code above is clang-format 23.1.1's output, so a diff means a line was typed
differently, and the second `git add` stages the fix); then
`check_format.sh: PASS (215 files, clang-format 23.1.1)` (no new C++ file) and
`check_spdx.sh: PASS (374 files carry the SPDX identifier)` (373 at the lane's fork plus
`terrain.mat`). `check_format.sh --fix` refuses non-C++ paths, so the `.mat` and the YAML files
stay out of it.

- [ ] **Step 14: Revert check (do not commit): the GLB check catches a missing UV copy**

```bash
cd "$WORKTREE"
sed -i '/if (kind_ == Kind::Terrain) copy_overlay_uvs_to_texcoord0(\*model);/d' \
    overlume/src/environment_stream.cpp
git diff --stat overlume/src/environment_stream.cpp
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_environment_stream
overlume/build/test_environment_stream \
    --gtest_filter='EnvironmentStreamOverlay.RasterAttachesAndItsColourCoversTheTile' | tail -4
git checkout -- overlume/src/environment_stream.cpp && git diff --stat overlume/src/environment_stream.cpp
```

Expected: the first diff reports `1 file changed, 1 deletion(-)`; the run prints
`the GLB step must copy _CESIUMOVERLAY_0 into TEXCOORD_0 for terrain tiles` and ends

```
[  FAILED  ] 1 test, listed below:
[  FAILED  ] EnvironmentStreamOverlay.RasterAttachesAndItsColourCoversTheTile

 1 FAILED TEST
```

while its colour checks still pass (gltfio's dummy UV0 samples one corner of a flat tile, which is
why the GLB is checked); the last diff prints nothing.

- [ ] **Step 15: Revert check (do not commit): the switch test catches live tiles that keep their
  colour**

```bash
cd "$WORKTREE"
sed -i '/renderResources_->sync_terrain_color();/d' overlume/src/environment_stream.cpp
git diff --stat overlume/src/environment_stream.cpp
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_environment_stream
overlume/build/test_environment_stream \
    --gtest_filter='EnvironmentStreamOverlay.TerrainColorFollowsAThemeSwitchOnLiveTiles' | tail -4
git checkout -- overlume/src/environment_stream.cpp && git diff --stat overlume/src/environment_stream.cpp
```

Expected: `1 file changed, 1 deletion(-)`; the run prints
`a live terrain tile must take the new theme's terrain_color` and ends with
`[  FAILED  ] EnvironmentStreamOverlay.TerrainColorFollowsAThemeSwitchOnLiveTiles` and
` 1 FAILED TEST`; the last diff prints nothing.

- [ ] **Step 16: Revert check (do not commit): the teardown test catches a texture that is never
  freed**

```bash
cd "$WORKTREE"
sed -i '/pendingTextureFrees_.push_back(static_cast<filament::Texture\*>(pMainThreadResult));/d' \
    overlume/src/environment_stream.cpp
git diff --stat overlume/src/environment_stream.cpp
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_environment_stream
overlume/build/test_environment_stream \
    --gtest_filter='EnvironmentStreamOverlay.TeardownDetachesEveryRasterAndFreesEveryTexture' \
    | tail -4
git checkout -- overlume/src/environment_stream.cpp && git diff --stat overlume/src/environment_stream.cpp
```

Expected: `1 file changed, 1 deletion(-)`; the run prints `a raster texture outlived its tileset`
and ends with
`[  FAILED  ] EnvironmentStreamOverlay.TeardownDetachesEveryRasterAndFreesEveryTexture` and
` 1 FAILED TEST` (Filament may also warn about leaked textures at engine shutdown); the last diff
prints nothing.

- [ ] **Step 17: Revert check (do not commit): the colour test catches terrain_color never
  reaching the material**

```bash
cd "$WORKTREE"
sed -i '/r.terrainMaterial->getDefaultInstance()->setParameter($/,+1d' overlume/src/renderer.cpp
git diff --stat overlume/src/renderer.cpp
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_environment_stream
overlume/build/test_environment_stream \
    --gtest_filter='EnvironmentStreamOverlay.TerrainColorShowsWhileNoRasterHasAttached' | tail -4
git checkout -- overlume/src/renderer.cpp && git diff --stat overlume/src/renderer.cpp
```

Expected: `1 file changed, 2 deletions(-)`; without the theme push the default instance's
`terrainColor` stays zero, the tile renders black, and the run prints
`the terrain tile must show the theme's terrain_color (green) while no raster is attached` and
ends with `[  FAILED  ] EnvironmentStreamOverlay.TerrainColorShowsWhileNoRasterHasAttached` and
` 1 FAILED TEST`; the last diff prints nothing.

- [ ] **Step 18: Revert check (do not commit): the orientation test catches a raster sampled
  without the v flip**

```bash
cd "$WORKTREE"
sed -i 's/vec2(uv.x, 1.0 - uv.y)/uv/' overlume/assets/materials/terrain.mat
git diff --stat overlume/assets/materials/terrain.mat
flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock cmake -S overlume -B overlume/build > /dev/null \
    && echo reconfigured \
    && CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
        --target test_environment_stream \
    && overlume/build/test_environment_stream \
        --gtest_filter='EnvironmentStreamOverlay.ImageryKeepsNorthUp' | grep -E 'must show|FAILED'
git checkout -- overlume/assets/materials/terrain.mat \
    && git diff --stat overlume/assets/materials/terrain.mat
```

Expected: `1 file changed, 1 insertion(+), 1 deletion(-)`; `reconfigured` (the edited `.mat` is
newer than its generated header, so the configure runs matc again; no `reconfigured` line means the
lock wait ran out: run the step again); then both probe messages (`the building north of the
colour line must show …` and `the building south of the colour line must show …`: the raster
mirrors about the colour line, so the colours swap), `[  FAILED  ]
EnvironmentStreamOverlay.ImageryKeepsNorthUp` and ` 1 FAILED TEST`; the last diff prints
nothing. Step 19's configure regenerates the restored material.

- [ ] **Step 19: Revert check (do not commit): the orientation test catches translation and scale
  in the wrong order**

```bash
cd "$WORKTREE"
sed -i 's/getUV0() \* materialParams.uvTransform.zw + materialParams.uvTransform.xy/getUV0() * materialParams.uvTransform.xy + materialParams.uvTransform.zw/' \
    overlume/assets/materials/terrain.mat
git diff --stat overlume/assets/materials/terrain.mat
flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock cmake -S overlume -B overlume/build > /dev/null \
    && echo reconfigured \
    && CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
        --target test_environment_stream \
    && overlume/build/test_environment_stream \
        --gtest_filter='EnvironmentStreamOverlay.ImageryKeepsNorthUp' | grep -E 'must show|FAILED'
git checkout -- overlume/assets/materials/terrain.mat \
    && git diff --stat overlume/assets/materials/terrain.mat
flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock cmake -S overlume -B overlume/build > /dev/null \
    && echo reconfigured \
    && CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
```

Expected: `1 file changed, 1 insertion(+), 1 deletion(-)`; `reconfigured`; the south probe's
message (`the building south of the colour line must show …`, which reads 'r': with translation
near 0 and scale near 1 every fragment samples near the raster's north-east corner) and the
`[  FAILED  ] EnvironmentStreamOverlay.ImageryKeepsNorthUp` and ` 1 FAILED TEST` lines; the
north probe may pass. Then the diff prints nothing, and the second `reconfigured` and a clean
build put the restored material back into the build inside the lock (the gate's prebuild would
otherwise reconfigure outside it).

- [ ] **Step 20: Gate** (foreground, Bash timeout 600000 ms)

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
ROS_DOMAIN_ID=83 CI_VISUAL_MODE_DOMAIN_ID=83 \
    flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock tools/ci_visual_mode.sh
```

Expected (the baseline plus `ThemeEnvironment.TerrainColorParsesAndDefaultsToTheGround` and the
six `EnvironmentStreamOverlay.*` tests; no new test name contains `Golden`):
- `PASS  POD header check`
- `PASS  library ctest suite  (100% tests passed, 0 tests failed out of 449 (431 ok / 8 skipped))`
- `PASS  node gtests  (Summary: 381 tests, 0 errors, 0 failures, 0 skipped)`
- `PASS  WS bridge pytest suite  (94 passed in …)`, the time varying
- `PASS  golden suite: 25 ok, 0 skipped`
- `PASS  examples  (6 run, all exited 0 with non-empty output)`
- `OVERALL: PASS`, exit 0.

Exit 75 means another lane held the lock for 300 s: run the same command again. Any other non-zero
exit is a gate failure. A WS bridge failure can also come from another session's gate on the same
ports, so rerun before treating it as a finding. The prebuild also rebuilds from the sources Steps
14-17 restored (Step 19 already rebuilt the restored material).

- [ ] **Step 21: Commit**

```bash
cd "$WORKTREE"
git add overlume/assets/materials/terrain.mat overlume/assets/themes/dark_adas.yaml \
    overlume/assets/themes/light_clay.yaml overlume/src/environment_stream.hpp \
    overlume/src/environment_stream.cpp overlume/src/environment_test_hooks.hpp \
    overlume/src/renderer.cpp overlume/src/renderer_internal.hpp overlume/src/theme.hpp \
    overlume/src/theme.cpp overlume/src/theme_transition.cpp \
    overlume/tests/test_environment_stream.cpp overlume/tests/test_theme.cpp \
    overlume/tests/test_theme_transition.cpp
git status --short
git commit -F - <<'EOF'
feat(stream): terrain material and raster overlays on terrain tiles

Imagery on terrain rides on cesium's raster overlays, and the renderer's
five overlay callbacks were stubs. StreamRendererResources gains a
Terrain flavour. Each terrain tile gets its own terrain.mat instance,
copied from the material's default instance, which carries the theme's
new environment.terrain_color, no imagery and a placeholder texture.
The GLB step copies _CESIUMOVERLAY_0 into TEXCOORD_0, which gltfio loads
as UV0. A raster tile becomes RGBA8 in the load thread and a mipmapped
sRGB texture in the main thread; attach sets the texture, cesium's
translation and scale and hasImagery on that tile's instance, detach
puts the placeholder back, and freed rasters join the deferred frees.
brightness scales the texture, and live terrain tiles follow theme
switches. terrain.mat has no fade parameter: the stream fades no tile.
Content resources keep today's behaviour.

environment.terrain_color defaults to the theme's ground colour, blends
in theme transitions, and is written into both shipped themes with
their ground colour, so no frame changes. The tests open the file-based
fixture tileset as terrain under file:// imagery
(install_fixture_terrain_with_file_imagery) and read the callbacks
through environment_stream_overlay_probe: terrain_color while no raster
has attached, the raster's colour once one has, every raster detached
and every texture freed on teardown, brightness, a theme switch on
live tiles, and imagery coloured by latitude landing north side north
on the fixture's buildings (which pins the v flip and the order of
cesium's translation and scale). That last test opens a copy of the
fixture without RTC_CENTER: cesium folds it into the positions it
derives overlay UVs from, which pushes every v to the south edge.

Claude-Session: <the executing session's URL>
EOF
```

Expected: `git status --short` lists exactly 14 files: `A  overlume/assets/materials/terrain.mat`
and thirteen `M ` lines; the commit succeeds.

### Task 6: Node parameters, composer, attribution text

**Lane:** 4. **Depends on:** Task 0 (the lane forks after it; nothing in Task 0 touches these
files). Task 8 documents what this task ships. Task 12 adds the node's once-a-second read of the
library's overlay credits (R5); the hand-off is under **Interfaces**.

**Deviation from head.md** (the orchestrator rules on it before lane 4 starts; the steps
assume it as proposed; head.md's File map, Task 6 row, already carries
`test_environment_live_switch.py` check 6):
1. **`docs/README.md` "## Plans"** gains this plan's entry in Task 8 Step 1 (Task 8 already owns
   `docs/README.md` in this lane), so the orchestrator does not add it at the plan commit.

**Files:**
- Modify: `ros/src/overlume_ros/include/overlume_ros/environment_source_uri.hpp` (includes 6-8;
  `is_mapbox_source_uri` and the head of `compose_environment_source_uri` 22-37; the end of the
  composer 80-90; `AttributionKind`, `kMapboxAttributionText` and `environment_attribution_kind`
  102-119)
- Modify: `ros/src/overlume_ros/include/overlume_ros/overlume_node.hpp` (members 233-234)
- Modify: `ros/src/overlume_ros/src/overlume_node.cpp` (declarations 452-453; the `on_activate`
  compose 785-788; the live-switch compose 900-904; the attribution block 1339-1365)
- Modify: `ros/src/overlume_ros/config/default_params.yaml` (63-64)
- Test: `ros/src/overlume_ros/test/test_environment_source_uri.cpp` (append after the last line,
  228)
- Test: `ros/src/overlume_ros/test/test_asset_paths.py` (docstring case 2 at 38-42; case 2 at
  217-247)
- Test: `ros/src/overlume_ros/test/test_environment_live_switch.py` (check 6: the end of
  docstring item 5 at 44-45; a check 6 run inserted before `main` at 281; `main`'s last line at
  361)

**Interfaces:**
- Consumes: `compose_environment_source_uri` (`environment_source_uri.hpp:24-90`) and its callers
  in `overlume_node.cpp` (`on_activate` 785, the live `environment_source_uri` switch 900, the
  network-loss WARN 1288); `environment_attribution_kind` (`environment_source_uri.hpp:110-119`);
  `CompositeAttribution` and `DrawText` (`hud_overlay.hpp`, unchanged). Task 4's parser rules
  appear here only as the strings the composer emits (no code dependency): the `mapbox://` layer
  list, `imagery=satellite|off`, and ion `terrain=<id>|off`, `imagery=<id>|off`.
- Produces (head.md's Node block, implemented exactly, plus the helpers it needs):

```cpp
namespace overlume::ros {
// True when the query carries `key=` as a key of its own (follow_terrain= is not terrain=).
inline bool has_query_key(const std::string& uri, const std::string& key);
// What a parameter value gives one scheme, "" for nothing: "disable" -> "off" on both;
// "ion:<digits>" -> "<digits>" on ion://; mapbox_value ("enable" for terrain, "satellite" for
// imagery) -> itself on mapbox://.
inline std::string environment_layer_value(const std::string& value, bool ion,
                                           const std::string& mapbox_value);
// The configure-time WARN for a value the composer ignores on the configured source (the other
// scheme's, or one that fits neither); "" when nothing is ignored.
inline std::string environment_layer_warning(const std::string& name, const std::string& value,
                                             const std::string& mapbox_value,
                                             const std::string& source_uri);
inline std::string compose_environment_source_uri(
    const std::string& environment_chunks_dir, const std::string& environment_source_uri,
    const std::string& environment_tile_cache_dir, bool follow_terrain = false,
    double ground_bias_m = kGroundBiasDefaultM, bool replaces_ground = kReplacesGroundDefault,
    double max_tilt_deg = kMaxTiltDegDefault, double brightness = kBrightnessDefault,
    double tile_radius_m = kTileRadiusDefaultM, const std::string& environment_terrain = "",
    const std::string& environment_imagery = "");
// environment_terrain: "" appends nothing; "enable" adds `terrain` to a mapbox:// layer list when
// absent; "ion:<id>" appends terrain=<id> to ion://, "disable" terrain=off (nothing on mapbox://,
// where no layer is off already). environment_imagery: "satellite" (mapbox://), "ion:<id>"
// (ion://) or "disable" (both) appended as imagery=<satellite|id|off>. Only when the URI lacks the
// key; every existing key, and its position, is unchanged; the other scheme's values append
// nothing.
enum class AttributionKind { NONE, GOOGLE, MAPBOX, CREDITS };  // CREDITS appended
inline constexpr char kGoogleAttributionText[] = "3D Tiles data (c) Google";
inline AttributionKind environment_attribution_kind(bool visible, bool enabled,
                                                    const std::string& source_uri,
                                                    overlume::EnvironmentSourceState state,
                                                    bool credits_non_empty = false);
// CREDITS: a STREAMING ion:// source that is not GOOGLE, with credits_non_empty.
inline std::string environment_attribution_text(AttributionKind kind, const std::string& credits);
// MAPBOX: "© Mapbox © OpenStreetMap" + (" " + each credit) + " Improve this map";
// GOOGLE: "3D Tiles data (c) Google" + (" " + each credit); CREDITS: the credits alone, space
// separated; NONE: "". `credits` is "; " separated, as environment_source_credits writes it;
// empty entries are skipped.
}
```

  Node parameters `environment_terrain` and `environment_imagery`: configure-time strings,
  default `""`, declared `declare_parameter<std::string>`; members `environment_terrain_` and
  `environment_imagery_` in `overlume_node.hpp`; `default_params.yaml` carries both as `""`. At
  configure, each value the configured `environment_source_uri` ignores logs one WARN
  (`environment_layer_warning`).

  **Hand-off to Task 12** (R5; it needs Task 11's `environment_source_credits` and this task's
  node code both merged). After this task `timer_callback` draws
  `environment_attribution_text(attribution, "")` under a `ponytail:` comment, and the kind call
  passes no `credits_non_empty`. Task 12 adds the members head.md names to `overlume_node.hpp`
  (`std::string environment_credits_;`, `double environment_credits_read_sec_{-1.0};`), reads
  `overlume::environment_source_credits(renderer_, buf, sizeof buf)` into `environment_credits_`
  once a second of node time (`sim_clock_sec_`) through `refresh_environment_credits`, a header
  helper Task 12 adds, passes `!environment_credits_.empty()` as the kind's last
  argument and `environment_credits_` as the text's, and replaces the `ponytail:` comment. A
  `CREDITS` source then draws its credits bottom-left (the `DrawText` branch), and `MAPBOX`
  gains "© Maxar".

**API notes:**
- `environment_source_uri.hpp:30-89`: the composer appends keys in a fixed order (cache,
  fallback, follow_terrain, ground_bias, replaces_ground, max_tilt_deg, brightness, radius) and
  each gate tests the raw `environment_source_uri` with `find("<key>=")`. A plain
  `find("terrain=")` would match `follow_terrain=`, which `:47-50` appends to every ion URI and a
  requested URI may carry, so the new keys use `has_query_key` (`?terrain=` or `&terrain=`). The
  new keys go last, terrain before imagery.
- `environment_source_uri.hpp:102-119`: `AttributionKind`, `kMapboxAttributionText` (also used by
  `test_hud_overlay.cpp:507-595`, so it stays) and `environment_attribution_kind`, whose truth
  table test (`test_environment_source_uri.cpp:193-228`) calls it with four arguments and stays
  as it is: with `credits_non_empty` false every row keeps its kind (`ion://96188` is `NONE`). The
  Google line is a literal in `overlume_node.cpp:1354`; it moves into `kGoogleAttributionText`
  with the same bytes.
- ROS 2 Humble parameter typing, probed with rclpy in a scratch directory (2026-10-15; rclcpp
  parses `--ros-args` with the same `rcl` parser): `-p x:=enable`, `disable`, `satellite`,
  `ion:1`, `ion:96188` and `'x:=""'` arrive as STRING; `-p x:=on` arrives as BOOL. So head.md's
  values all reach a `declare_parameter<std::string>` as strings. A YAML boolean (`on`, `off`,
  `true`, `yes`) or integer makes that declaration throw `rclcpp::exceptions::
  InvalidParameterTypeException` (`parameter 'environment_terrain' has invalid type: Wrong
  parameter type, parameter {environment_terrain} is of type {string}, setting it to {bool} is
  not allowed.`), which fails the configure transition, as a bad `initial_theme` does
  (`test_asset_paths.py` case 3). Task 8 documents it. `tools/validate_visual_mode.sh:32` turns
  `--param x` into `-p x`, so the rig parses the same way.
- The configure WARN covers what head.md's Node block names (a value for the other scheme) and
  one case it does not: a value that fits neither scheme (`sat`, `ion:x`). With a configured URI
  that is neither `mapbox://` nor `ion://` (`""` or a baked directory), only that second case
  warns, since a live switch may later open either scheme. The live switch itself logs nothing
  extra: its `environment source armed: '<uri>'` line shows the composed URI.
- `overlume_node.cpp:34` opens `on_configure`; the environment parameters are declared there
  (`:443-475`), after `environment_source_uri_` (`:445`), so the WARN sees the configured URI. The
  parameter callback has no branch for the `environment_*` tuning parameters, so a runtime set
  changes nothing: they are configure-time, and the two new ones are too.
- `overlume_node.cpp:785-788` and `:900-904` compose the URI the source opens; `:1288-1292`
  composes only to read `fallback=` back (`fallback_dir_from_source_uri`,
  `environment_source_uri.hpp:92-100`), which `terrain=`/`imagery=` never change, so that call
  keeps the defaults. `:913` stores the requested, uncomposed URI, which the GUI's exact match and
  `environment_attribution_kind` see.
- `overlume_node.cpp:1339-1365`: the attribution block. After this task it draws
  `environment_attribution_text(attribution, "")`: MAPBOX gives `kMapboxAttributionText`, GOOGLE
  the Google literal; CREDITS cannot occur yet (no credits), and NONE gives `""`, which draws
  nothing. Every frame is unchanged.
- `ros2param/verb/get.py` (Humble, `:62-64`, `:102-105`): `--hide-type` prints a declared string
  bare (an empty line for `""`), and for an undeclared name prints nothing on stdout (the
  `Parameter not set` return goes to stderr). `test_asset_paths.py`'s `_param_get` (`:109-111`)
  therefore returns `''` for both, so the read-back sets non-empty values (case 2). `main.cpp:12`
  builds the node with default `NodeOptions`, so an override for an undeclared name is ignored.
- `ros/src/overlume_ros/CMakeLists.txt:284-287`: `test_environment_source_uri` is header-only
  (`ament_add_gtest`, include dirs only), built by g++ 11.4 in C++17
  (`ros/build/overlume_ros/CMakeCache.txt:59`, `CMakeLists.txt:7`); binary
  `ros/build/overlume_ros/test_environment_source_uri`; 23 tests today. Tests after line 63 sit
  outside the file's namespaces and qualify `overlume::ros::`; so do the new ones.
- `ros/colcon_build.sh` adds colcon's `console_direct+` handler, and colcon's default
  `console_stderr` handler replays every stderr line when the job ends, so each compiler error
  appears twice in the build log; Step 2 counts distinct lines (`sort -u`).
- `test_asset_paths.py` is not registered with CMake (`:57-62`), so the gate does not run it; it
  needs this worktree's `ros/install` and a Release node (`:18-24`), and launches
  `/overlume_node`, so it runs with the lane's `ROS_DOMAIN_ID=84`.
- `test_environment_live_switch.py` is not registered either (its docstring, `:60-62`), so the
  gate does not run it; it launches the node twice today (checks 1-4, then check 5's
  `_run_hidden_arm_check`, which solves the anchor through `geo_datum_*`). Check 6 is a third
  launch of the same shape. It proves both compose calls without the network: with
  `MAPBOX_TOKEN` unset in the child's environment (`unset`, never printed), no `mapbox://` source
  can open. `open_mapbox_source` (`overlume/src/mapbox_loader.cpp:645-658`) returns null before
  any request, at the parse in this lane's tree (the layer list arrives with Task 4) and at the
  token check once Task 4 is merged (the token comes from `getenv` only, `:696`). The node then
  logs the composed URI: `set_environment_source: failed to open '%s' -- no buildings this run`
  from `on_activate` (`overlume_node.cpp:792`), and `set_environment_source: failed to open
  '<uri>' -- previous environment source left intact` from the live switch (`:921-923`). The
  live switch's value holds `?`, so it is single-quoted inside `_param_set`'s `bash -c` (an
  unquoted `&` would background the command). The radius is not appended: the node default 700
  equals `kTileRadiusDefaultM`.
- The header and the fourteen new tests below were compiled with `-Wall -Wextra` against
  googletest from `/opt/ros/humble/src/gtest_vendor` and run in a scratch directory: 37 tests
  pass, no warning; `clang-format` 23.1.1 with the repo's `.clang-format` leaves the header, the
  test file and the node files unchanged; the patched `overlume_node.cpp`/`overlume_node.hpp`
  pass `g++ -fsyntax-only -Wall -Wextra` with the node's own flags from
  `ros/build/overlume_ros/compile_commands.json`; both patched Python files parse.

- [ ] **Step 1: Write the failing composer, warning and attribution tests**

Append to the end of `ros/src/overlume_ros/test/test_environment_source_uri.cpp`:

```cpp

namespace {
// The two layer parameters, with every other node knob at its default.
std::string compose_layers(const std::string& uri, const std::string& terrain,
                           const std::string& imagery) {
    return overlume::ros::compose_environment_source_uri(
        "", uri, "", false, overlume::ros::kGroundBiasDefaultM,
        overlume::ros::kReplacesGroundDefault, overlume::ros::kMaxTiltDegDefault,
        overlume::ros::kBrightnessDefault, overlume::ros::kTileRadiusDefaultM, terrain, imagery);
}
}  // namespace

TEST(ComposeEnvironmentSourceUri, MapboxTerrainEnableAddsTheLayerOnce) {
    EXPECT_EQ(compose_layers("mapbox://buildings", "enable", ""), "mapbox://buildings,terrain");
    EXPECT_EQ(compose_layers("mapbox://buildings?materials=original", "enable", ""),
              "mapbox://buildings,terrain?materials=original");
    EXPECT_EQ(compose_layers("mapbox://buildings,terrain", "enable", ""),
              "mapbox://buildings,terrain");
    EXPECT_EQ(compose_layers("mapbox://terrain,buildings?imagery=satellite", "enable", ""),
              "mapbox://terrain,buildings?imagery=satellite");
    EXPECT_EQ(overlume::ros::compose_environment_source_uri("/baked", "mapbox://buildings",
                                                            "/tiles", true, 0.5, false, 5.0, 1.8,
                                                            250.0, "enable", "satellite"),
              "mapbox://buildings,terrain?fallback=/baked&brightness=1.8&radius=250&"
              "imagery=satellite");
}

TEST(ComposeEnvironmentSourceUri, MapboxTerrainDisableOrEmptyLeavesTheLayerList) {
    EXPECT_EQ(compose_layers("mapbox://buildings", "disable", ""), "mapbox://buildings");
    EXPECT_EQ(compose_layers("mapbox://buildings,terrain", "disable", ""),
              "mapbox://buildings,terrain");
    EXPECT_EQ(compose_layers("mapbox://buildings", "", ""), "mapbox://buildings");
}

TEST(ComposeEnvironmentSourceUri, MapboxImageryAppendedOnlyWhenTheUriLacksIt) {
    EXPECT_EQ(compose_layers("mapbox://buildings,terrain", "", "satellite"),
              "mapbox://buildings,terrain?imagery=satellite");
    EXPECT_EQ(compose_layers("mapbox://buildings,terrain", "", "disable"),
              "mapbox://buildings,terrain?imagery=off");
    EXPECT_EQ(compose_layers("mapbox://buildings,terrain?imagery=satellite", "", "disable"),
              "mapbox://buildings,terrain?imagery=satellite");
    EXPECT_EQ(compose_layers("mapbox://buildings,terrain?imagery=off", "", "satellite"),
              "mapbox://buildings,terrain?imagery=off");
}

TEST(ComposeEnvironmentSourceUri, IonTerrainAndImageryAppendedOnlyWhenTheUriLacksThem) {
    EXPECT_EQ(compose_layers("ion://96188", "ion:1", "ion:2"),
              "ion://96188?follow_terrain=off&terrain=1&imagery=2");
    EXPECT_EQ(compose_layers("ion://96188", "disable", "disable"),
              "ion://96188?follow_terrain=off&terrain=off&imagery=off");
    EXPECT_EQ(compose_layers("ion://96188?terrain=1&imagery=2", "disable", "disable"),
              "ion://96188?terrain=1&imagery=2&follow_terrain=off");
    EXPECT_EQ(compose_layers("ion://96188?imagery=3", "ion:1", "ion:2"),
              "ion://96188?imagery=3&follow_terrain=off&terrain=1");
}

TEST(ComposeEnvironmentSourceUri, ValuesForTheOtherSchemeOrNoSchemeAreIgnored) {
    EXPECT_EQ(compose_layers("mapbox://buildings", "ion:1", "ion:2"), "mapbox://buildings");
    EXPECT_EQ(compose_layers("ion://96188", "enable", "satellite"),
              "ion://96188?follow_terrain=off");
    EXPECT_EQ(compose_layers("ion://96188", "1", "ion:"), "ion://96188?follow_terrain=off");
    EXPECT_EQ(compose_layers("ion://96188", "ion:-1", "ion:2x"), "ion://96188?follow_terrain=off");
    EXPECT_EQ(compose_layers("mapbox://buildings", "on", "off"), "mapbox://buildings");
}

TEST(ComposeEnvironmentSourceUri, IonTerrainKeyIsNotFollowTerrain) {
    EXPECT_EQ(compose_layers("ion://96188?follow_terrain=on", "ion:1", ""),
              "ion://96188?follow_terrain=on&terrain=1");
}

TEST(ComposeEnvironmentSourceUri, EmptyLayerParametersAndBakedSourcesAppendNothing) {
    EXPECT_EQ(compose_layers("ion://96188", "", ""), "ion://96188?follow_terrain=off");
    EXPECT_EQ(compose_layers("/baked/chunks", "enable", "satellite"), "/baked/chunks");
    EXPECT_EQ(overlume::ros::compose_environment_source_uri(
                  "/baked/chunks", "", "", false, 0.3, true, 2.0, 1.0, 700.0, "ion:1", "ion:2"),
              "/baked/chunks");
}

TEST(ComposeEnvironmentSourceUri, IonUriWithEveryKnobAndBothLayersKeepsTheOrder) {
    EXPECT_EQ(
        overlume::ros::compose_environment_source_uri("/baked", "ion://96188", "/tiles", true, 0.5,
                                                      false, 5.0, 1.8, 250.0, "ion:1", "ion:2"),
        "ion://96188?cache=/tiles&fallback=/baked&follow_terrain=on&ground_bias=0.5&"
        "replaces_ground=off&max_tilt_deg=5&brightness=1.8&radius=250&terrain=1&imagery=2");
}

TEST(EnvironmentLayerWarning, NamesAValueTheConfiguredSourceIgnores) {
    using overlume::ros::environment_layer_warning;
    EXPECT_EQ(
        environment_layer_warning("environment_terrain", "ion:1", "enable", "mapbox://buildings"),
        "environment_terrain 'ion:1' is ignored for environment source 'mapbox://buildings' "
        "(accepted: \"\", disable, enable for mapbox://, ion:<asset id> for ion://)");
    EXPECT_EQ(
        environment_layer_warning("environment_imagery", "satellite", "satellite", "ion://96188"),
        "environment_imagery 'satellite' is ignored for environment source 'ion://96188' "
        "(accepted: \"\", disable, satellite for mapbox://, ion:<asset id> for ion://)");
    EXPECT_EQ(environment_layer_warning("environment_imagery", "sat", "satellite", ""),
              "environment_imagery 'sat' is ignored for environment source '' "
              "(accepted: \"\", disable, satellite for mapbox://, ion:<asset id> for ion://)");
}

TEST(EnvironmentLayerWarning, SilentForAValueThatApplies) {
    using overlume::ros::environment_layer_warning;
    EXPECT_EQ(
        environment_layer_warning("environment_terrain", "enable", "enable", "mapbox://buildings"),
        "");
    EXPECT_EQ(environment_layer_warning("environment_terrain", "ion:1", "enable", "ion://96188"),
              "");
    EXPECT_EQ(
        environment_layer_warning("environment_imagery", "disable", "satellite", "ion://96188"),
        "");
    EXPECT_EQ(environment_layer_warning("environment_imagery", "", "satellite", "ion://96188"), "");
    EXPECT_EQ(environment_layer_warning("environment_terrain", "ion:1", "enable", "/baked/chunks"),
              "");
    EXPECT_EQ(environment_layer_warning("environment_imagery", "satellite", "satellite", ""), "");
}

TEST(MapboxAttributionKind, CreditsMakeAnIonSourceOtherThanGoogleCredits) {
    using overlume::EnvironmentSourceState;
    using overlume::ros::AttributionKind;
    using overlume::ros::environment_attribution_kind;
    const auto streaming = EnvironmentSourceState::STREAMING;
    EXPECT_EQ(environment_attribution_kind(true, true, "ion://96188?terrain=1&imagery=2", streaming,
                                           true),
              AttributionKind::CREDITS);
    EXPECT_EQ(environment_attribution_kind(true, true, "ion://96188?terrain=1&imagery=2", streaming,
                                           false),
              AttributionKind::NONE);
    EXPECT_EQ(environment_attribution_kind(true, true, "ion://2275207?materials=original",
                                           streaming, true),
              AttributionKind::GOOGLE);
    EXPECT_EQ(
        environment_attribution_kind(true, true, "mapbox://buildings,terrain", streaming, true),
        AttributionKind::MAPBOX);
    EXPECT_EQ(environment_attribution_kind(true, true, "/baked/chunks", streaming, true),
              AttributionKind::NONE);
    EXPECT_EQ(environment_attribution_kind(true, true, "ion://96188", EnvironmentSourceState::BAKED,
                                           true),
              AttributionKind::NONE);
    EXPECT_EQ(environment_attribution_kind(false, true, "ion://96188", streaming, true),
              AttributionKind::NONE);
    EXPECT_EQ(environment_attribution_kind(true, false, "ion://96188", streaming, true),
              AttributionKind::NONE);
}

TEST(EnvironmentAttributionText, WithoutCreditsEachKindDrawsTodaysLine) {
    using overlume::ros::AttributionKind;
    using overlume::ros::environment_attribution_text;
    EXPECT_EQ(environment_attribution_text(AttributionKind::MAPBOX, ""),
              overlume::ros::kMapboxAttributionText);
    EXPECT_EQ(environment_attribution_text(AttributionKind::GOOGLE, ""),
              "3D Tiles data (c) Google");
    EXPECT_EQ(environment_attribution_text(AttributionKind::CREDITS, ""), "");
    EXPECT_EQ(environment_attribution_text(AttributionKind::NONE, ""), "");
}

TEST(EnvironmentAttributionText, CreditsFollowTheBaseTextAndPrecedeImproveThisMap) {
    using overlume::ros::AttributionKind;
    using overlume::ros::environment_attribution_text;
    const std::string bing = "Earthstar Geographics; \xC2\xA9 2026 Microsoft Corporation";
    EXPECT_EQ(environment_attribution_text(AttributionKind::MAPBOX, "\xC2\xA9 Maxar"),
              "\xC2\xA9 Mapbox \xC2\xA9 OpenStreetMap \xC2\xA9 Maxar Improve this map");
    EXPECT_EQ(environment_attribution_text(AttributionKind::GOOGLE, bing),
              "3D Tiles data (c) Google Earthstar Geographics \xC2\xA9 2026 Microsoft Corporation");
    EXPECT_EQ(environment_attribution_text(AttributionKind::CREDITS, bing),
              "Earthstar Geographics \xC2\xA9 2026 Microsoft Corporation");
    EXPECT_EQ(environment_attribution_text(AttributionKind::NONE, bing), "");
}

TEST(EnvironmentAttributionText, EmptyCreditsBetweenSeparatorsAreSkipped) {
    EXPECT_EQ(overlume::ros::environment_attribution_text(overlume::ros::AttributionKind::CREDITS,
                                                          "A; ; B; "),
              "A B");
}
```

What each catches when its behaviour is reverted (six of these reverts were applied to a scratch
copy of the Step 3 header: `has_query_key` replaced by a plain find, the fits-neither case
dropped, `CREDITS` tested ahead of Google, `CREDITS` returned without credits, the imagery
scheme gate dropped, a non-digit id accepted; each failed the test named for it, and the
credits-less `CREDITS` also failed the existing truth table):
- `MapboxTerrainEnableAddsTheLayerOnce`: the membership test dropped (`terrain` inserted twice),
  or `imagery=satellite` no longer appended for `mapbox://`.
- `MapboxTerrainDisableOrEmptyLeavesTheLayerList`: `disable` or `""` treated as `enable`.
- `MapboxImageryAppendedOnlyWhenTheUriLacksIt`: the mapbox imagery append removed, `disable` not
  mapped to `off`, or the URI's own `imagery=` overridden.
- `IonTerrainAndImageryAppendedOnlyWhenTheUriLacksThem`: the ion terrain append removed, the
  `ion:` prefix kept in the value, or `disable` not mapped to `off`.
- `ValuesForTheOtherSchemeOrNoSchemeAreIgnored`: `environment_layer_value` widened (a bare id, an
  empty or signed id, or `enable`/`satellite` accepted on `ion://`; then `environment_imagery:
  satellite` would break every switch to `osm`, and `ion:2` every switch to `mapbox`).
- `IonTerrainKeyIsNotFollowTerrain`: `has_query_key` replaced by a plain `find("terrain=")`.
- `EmptyLayerParametersAndBakedSourcesAppendNothing`: the `ion || mapbox` gate on imagery dropped.
- `IonUriWithEveryKnobAndBothLayersKeepsTheOrder`: the new keys moved ahead of an existing one.
- `EnvironmentLayerWarning.*`: the warning's scheme choice inverted, the fits-neither case
  dropped, or a warning for a value that applies.
- `MapboxAttributionKind.CreditsMakeAnIonSourceOtherThanGoogleCredits`: the `CREDITS` line in
  `environment_attribution_kind` removed, or placed ahead of the Google test, or `CREDITS`
  returned without credits.
- `EnvironmentAttributionText.*`: " Improve this map" dropped or moved before the credits; a
  changed Google literal; the `end > start` guard removed (double spaces).

- [ ] **Step 2: Build and see the test target fail to compile**

```bash
cd "$WORKTREE"
ros/colcon_build.sh overlume_ros > /tmp/overlume-l4/task6-build.log 2>&1; echo "exit=$?"
grep -oE "error: .*" /tmp/overlume-l4/task6-build.log | sort -u
tail -n 3 /tmp/overlume-l4/task6-build.log
```

Expected: a non-zero `exit=`; these eight distinct messages (g++ 11.4, UTF-8 quotes), all in
`test_environment_source_uri.cpp` (colcon prints each twice; see the API notes):

```text
error: ‘CREDITS’ is not a member of ‘overlume::ros::AttributionKind’
error: ‘environment_attribution_text’ has not been declared in ‘overlume::ros’
error: ‘environment_attribution_text’ is not a member of ‘overlume::ros’; did you mean ‘environment_attribution_kind’?
error: ‘environment_attribution_text’ was not declared in this scope
error: ‘environment_layer_warning’ has not been declared in ‘overlume::ros’
error: ‘environment_layer_warning’ was not declared in this scope
error: too many arguments to function ‘overlume::ros::AttributionKind overlume::ros::environment_attribution_kind(bool, bool, const string&, overlume::EnvironmentSourceState)’
error: too many arguments to function ‘std::string overlume::ros::compose_environment_source_uri(const string&, const string&, const string&, bool, double, bool, double, double, double)’
```

then the colcon summary:

```text
Summary: 0 packages finished […]
  1 package failed: overlume_ros
  1 package had stderr output: overlume_ros
```

- [ ] **Step 3: Implement the composer rules, the warning and the attribution text**

In `ros/src/overlume_ros/include/overlume_ros/environment_source_uri.hpp`, apply these four
replacements (each old block occurs once).

Replace

```cpp
#include <locale>
#include <sstream>
#include <string>
```

with

```cpp
#include <algorithm>
#include <locale>
#include <sstream>
#include <string>
```

Replace

```cpp
inline bool is_mapbox_source_uri(const std::string& uri) { return uri.rfind("mapbox://", 0) == 0; }

inline std::string compose_environment_source_uri(
    const std::string& environment_chunks_dir, const std::string& environment_source_uri,
    const std::string& environment_tile_cache_dir, bool follow_terrain = false,
    double ground_bias_m = kGroundBiasDefaultM, bool replaces_ground = kReplacesGroundDefault,
    double max_tilt_deg = kMaxTiltDegDefault, double brightness = kBrightnessDefault,
    double tile_radius_m = kTileRadiusDefaultM) {
    std::string source_uri = environment_chunks_dir;
    if (!environment_source_uri.empty()) {
        const bool ion = is_ion_source_uri(environment_source_uri);
        // Mapbox tiles never reach disk (Product Terms), so never cache=; the Mapbox source holds
        // buildings only, so the terrain and ground keys stay ion-only.
        const bool mapbox = is_mapbox_source_uri(environment_source_uri);
        source_uri = environment_source_uri;
        if (!mapbox && !environment_tile_cache_dir.empty() &&
```

with

```cpp
inline bool is_mapbox_source_uri(const std::string& uri) { return uri.rfind("mapbox://", 0) == 0; }

// True when the query carries `key=` as a key of its own: follow_terrain= is not terrain=.
inline bool has_query_key(const std::string& uri, const std::string& key) {
    return uri.find("?" + key + "=") != std::string::npos ||
           uri.find("&" + key + "=") != std::string::npos;
}

// What a value of environment_terrain or environment_imagery gives a source of one scheme, ""
// for nothing: "disable" gives off on both schemes, "ion:<digits>" gives <digits> on ion://, and
// `mapbox_value` ("enable" for terrain, "satellite" for imagery) gives itself on mapbox://.
inline std::string environment_layer_value(const std::string& value, bool ion,
                                           const std::string& mapbox_value) {
    if (value == "disable") return "off";
    if (!ion) return value == mapbox_value ? value : "";
    const std::string id = value.rfind("ion:", 0) == 0 ? value.substr(4) : "";
    return !id.empty() && id.find_first_not_of("0123456789") == std::string::npos ? id : "";
}

// The WARN line the node logs at configure when the composer ignores a layer parameter's value
// for the configured source: a value for the other scheme, or one that fits neither (a source
// that is neither mapbox:// nor ion:// warns only then). "" when nothing is ignored.
inline std::string environment_layer_warning(const std::string& name, const std::string& value,
                                             const std::string& mapbox_value,
                                             const std::string& source_uri) {
    const bool fits_ion = !environment_layer_value(value, true, mapbox_value).empty();
    const bool fits_mapbox = !environment_layer_value(value, false, mapbox_value).empty();
    const bool fits = is_ion_source_uri(source_uri)      ? fits_ion
                      : is_mapbox_source_uri(source_uri) ? fits_mapbox
                                                         : fits_ion || fits_mapbox;
    if (value.empty() || fits) return "";
    return name + " '" + value + "' is ignored for environment source '" + source_uri +
           "' (accepted: \"\", disable, " + mapbox_value +
           " for mapbox://, ion:<asset id> for ion://)";
}

inline std::string compose_environment_source_uri(
    const std::string& environment_chunks_dir, const std::string& environment_source_uri,
    const std::string& environment_tile_cache_dir, bool follow_terrain = false,
    double ground_bias_m = kGroundBiasDefaultM, bool replaces_ground = kReplacesGroundDefault,
    double max_tilt_deg = kMaxTiltDegDefault, double brightness = kBrightnessDefault,
    double tile_radius_m = kTileRadiusDefaultM, const std::string& environment_terrain = "",
    const std::string& environment_imagery = "") {
    std::string source_uri = environment_chunks_dir;
    if (!environment_source_uri.empty()) {
        const bool ion = is_ion_source_uri(environment_source_uri);
        // Mapbox tiles never reach disk (Product Terms), so never cache=; the mapbox:// parser
        // takes none of the ground keys, so they stay ion-only.
        const bool mapbox = is_mapbox_source_uri(environment_source_uri);
        source_uri = environment_source_uri;
        // mapbox:// carries terrain in its layer list, not as a key.
        if (mapbox && environment_layer_value(environment_terrain, false, "enable") == "enable") {
            const size_t list_end = std::min(source_uri.find('?'), source_uri.size());
            const size_t list_start = sizeof("mapbox://") - 1;
            const std::string list =
                "," + source_uri.substr(list_start, list_end - list_start) + ",";
            if (list.find(",terrain,") == std::string::npos) {
                source_uri.insert(list_end, ",terrain");
            }
        }
        if (!mapbox && !environment_tile_cache_dir.empty() &&
```

Replace

```cpp
            source_uri += "radius=" + v.str();
        }
    }
    return source_uri;
}
```

with

```cpp
            source_uri += "radius=" + v.str();
        }
        const std::string terrain = environment_layer_value(environment_terrain, true, "enable");
        if (ion && !terrain.empty() && !has_query_key(environment_source_uri, "terrain")) {
            source_uri += (source_uri.find('?') == std::string::npos ? "?" : "&");
            source_uri += "terrain=" + terrain;
        }
        const std::string imagery = environment_layer_value(environment_imagery, ion, "satellite");
        if ((ion || mapbox) && !imagery.empty() &&
            !has_query_key(environment_source_uri, "imagery")) {
            source_uri += (source_uri.find('?') == std::string::npos ? "?" : "&");
            source_uri += "imagery=" + imagery;
        }
    }
    return source_uri;
}
```

Replace

```cpp
enum class AttributionKind { NONE, GOOGLE, MAPBOX };

// Mapbox Product Terms 1.4 wording; the links become plain text on a video frame.
inline constexpr char kMapboxAttributionText[] =
    "\xC2\xA9 Mapbox \xC2\xA9 OpenStreetMap Improve this map";

// The attribution the frame owes. The scheme is checked first: the Google test is a substring
// match that mapbox://buildings?materials=original would pass too.
inline AttributionKind environment_attribution_kind(bool visible, bool enabled,
                                                    const std::string& source_uri,
                                                    overlume::EnvironmentSourceState state) {
    if (!visible || !enabled || state != overlume::EnvironmentSourceState::STREAMING) {
        return AttributionKind::NONE;
    }
    if (is_mapbox_source_uri(source_uri)) return AttributionKind::MAPBOX;
    if (source_uri.find("materials=original") != std::string::npos) return AttributionKind::GOOGLE;
    return AttributionKind::NONE;
}
```

with

```cpp
// CREDITS: an ion:// source other than Google whose raster overlays carry credits (osm-terrain's
// Bing imagery); it owes those credits alone, with no logo.
enum class AttributionKind { NONE, GOOGLE, MAPBOX, CREDITS };

// Mapbox Product Terms 1.4 wording; the links become plain text on a video frame.
inline constexpr char kMapboxAttributionText[] =
    "\xC2\xA9 Mapbox \xC2\xA9 OpenStreetMap Improve this map";

inline constexpr char kGoogleAttributionText[] = "3D Tiles data (c) Google";

// The attribution the frame owes. The scheme is checked first: the Google test is a substring
// match that mapbox://buildings?materials=original would pass too.
inline AttributionKind environment_attribution_kind(bool visible, bool enabled,
                                                    const std::string& source_uri,
                                                    overlume::EnvironmentSourceState state,
                                                    bool credits_non_empty = false) {
    if (!visible || !enabled || state != overlume::EnvironmentSourceState::STREAMING) {
        return AttributionKind::NONE;
    }
    if (is_mapbox_source_uri(source_uri)) return AttributionKind::MAPBOX;
    if (source_uri.find("materials=original") != std::string::npos) return AttributionKind::GOOGLE;
    if (credits_non_empty && is_ion_source_uri(source_uri)) return AttributionKind::CREDITS;
    return AttributionKind::NONE;
}

// The line for `kind`: its base text, then each overlay credit ("; " separated, as the library
// reports them) after a space, then Mapbox's "Improve this map". Empty: nothing to draw.
inline std::string environment_attribution_text(AttributionKind kind, const std::string& credits) {
    std::string each;
    for (size_t start = 0; start < credits.size();) {
        const size_t end = std::min(credits.find("; ", start), credits.size());
        if (end > start) each += " " + credits.substr(start, end - start);
        start = end + 2;
    }
    switch (kind) {
        case AttributionKind::MAPBOX:
            return "\xC2\xA9 Mapbox \xC2\xA9 OpenStreetMap" + each + " Improve this map";
        case AttributionKind::GOOGLE:
            return kGoogleAttributionText + each;
        case AttributionKind::CREDITS:
            return each.empty() ? each : each.substr(1);
        case AttributionKind::NONE:
            break;
    }
    return "";
}
```

- [ ] **Step 4: Build and run the composer tests**

```bash
cd "$WORKTREE"
ros/colcon_build.sh overlume_ros 2>&1 | tail -n 3
source /opt/ros/humble/setup.bash
ros/build/overlume_ros/test_environment_source_uri | tail -n 1
```

Expected: the build ends `Summary: 1 package finished […]`, then `[  PASSED  ] 37 tests.` (23
existing, 14 new). The node still builds: it calls `environment_attribution_kind` with four
arguments and the composer with nine, both still valid.

- [ ] **Step 5: Read both parameters back, and prove both compose calls use them**

In `ros/src/overlume_ros/test/test_asset_paths.py`, apply these five replacements.

Replace

```text
  2. Relaunch with -p initial_theme:=light_clay and
     -p environment_attribution_logo:=false -- configure succeeds,
     `ros2 param get initial_theme` reads back light_clay, the logo switch
     reads back false and the log says the Mapbox logo is off, still no
     fallback WARN.
```

with

```text
  2. Relaunch with -p initial_theme:=light_clay,
     -p environment_attribution_logo:=false, -p environment_terrain:=ion:1
     and -p environment_imagery:=sat -- configure succeeds,
     `ros2 param get initial_theme` reads back light_clay, the logo switch
     reads back false and the log says the Mapbox logo is off, the two
     layer parameters read back as the strings ion:1 and sat, the log warns
     once that environment_imagery 'sat' is ignored (it fits no source),
     still no fallback WARN.
```

Replace

```python
    proc = _launch("-p initial_theme:=light_clay -p environment_attribution_logo:=false", log2)
```

with

```python
    proc = _launch("-p initial_theme:=light_clay -p environment_attribution_logo:=false "
                   "-p environment_terrain:=ion:1 -p environment_imagery:=sat", log2)
```

Replace

```python
            print(f"FAIL: configure failed with initial_theme:=light_clay:\n"
                  f"{_read(log2)[-2000:]}", file=sys.stderr)
```

with

```python
            print(f"FAIL: configure failed with initial_theme:=light_clay and the layer "
                  f"parameters:\n{_read(log2)[-2000:]}", file=sys.stderr)
```

Replace

```python
            print(f"FAIL: `ros2 param get environment_attribution_logo` returned '{logo_on}', "
                  "expected 'false'.", file=sys.stderr)
            return 1
```

with

```python
            print(f"FAIL: `ros2 param get environment_attribution_logo` returned '{logo_on}', "
                  "expected 'false'.", file=sys.stderr)
            return 1
        layers = (_param_get("environment_terrain"), _param_get("environment_imagery"))
        if layers != ("ion:1", "sat"):
            print(f"FAIL: `ros2 param get` for environment_terrain and environment_imagery "
                  f"returned {layers}, expected ('ion:1', 'sat').", file=sys.stderr)
            return 1
        ignored = "environment_imagery 'sat' is ignored for environment source ''"
        if _read(log2).count(ignored) != 1:
            print(f"FAIL: expected one WARN containing {ignored!r}:\n{_read(log2)[-3000:]}",
                  file=sys.stderr)
            return 1
```

Replace

```python
        print("PASS (2/3): initial_theme:=light_clay configured cleanly, param reads back "
              "light_clay, environment_attribution_logo:=false reads back and logs the logo "
              "off, no fallback WARN.")
```

with

```python
        print("PASS (2/3): initial_theme:=light_clay configured cleanly, param reads back "
              "light_clay, environment_attribution_logo:=false reads back and logs the logo "
              "off, environment_terrain:=ion:1 and environment_imagery:=sat read back as "
              "strings and 'sat' is warned once as ignored, no fallback WARN.")
```

Reverts it catches: the parameters not declared (an undeclared override is ignored and reads
back `''`); `ion:1` mangled on the way in (a declaration other than a string); the configure
WARN loop removed, or run twice.

In `ros/src/overlume_ros/test/test_environment_live_switch.py`, apply these three replacements
(each old block occurs once).

Replace (the end of docstring item 5)

```text
     hides the now-visible source again even though environment_enabled
     stayed true, and `render_mode:=3` (FREE_LOOK) restores it.
```

with

```text
     hides the now-visible source again even though environment_enabled
     stayed true, and `render_mode:=3` (FREE_LOOK) restores it.
  6. (separate run, _run_layer_parameter_check()) **environment_terrain and
     environment_imagery reach both compose calls**: launched with
     environment_source_uri:=mapbox://buildings, environment_terrain:=enable,
     environment_imagery:=satellite, a geo_datum_* override and MAPBOX_TOKEN
     removed from the node's environment, so no source opens and no request
     is sent; the node's "failed to open '<uri>'" WARN names the composed URI,
     once from on_activate() and once from a live environment_source_uri
     switch.
```

Replace

```python
def main() -> int:
```

with

```python
def _run_layer_parameter_check() -> int:
    """Check 6 (imagery on terrain): both compose calls, on_activate() and the live
    environment_source_uri switch, pass environment_terrain and environment_imagery.

    MAPBOX_TOKEN is unset in the node's shell (never printed), so no mapbox:// source can open
    and nothing reaches the network: each attempt logs "set_environment_source: failed to open
    '<composed uri>'", and that URI is the proof.
    """
    log_fd, log_path = tempfile.mkstemp(prefix="viz_environment_layers_", suffix=".log")
    os.close(log_fd)

    viz_cmd = (
        f"unset MAPBOX_TOKEN && "
        f"source /opt/ros/humble/setup.bash && source {INSTALL_DIR}/setup.bash && "
        f"ros2 run overlume_ros overlume_node --ros-args "
        f"-p out_width:=160 -p out_height:=120 -p initial_mode:=3 "
        f"-p geo_datum_lat_deg:={FIXTURE_LAT_DEG} "
        f"-p geo_datum_lon_deg:={FIXTURE_LON_DEG} "
        f"-p geo_datum_heading_deg:={FIXTURE_HEADING_DEG} "
        f"-p environment_source_uri:=mapbox://buildings "
        f"-p environment_terrain:=enable -p environment_imagery:=satellite "
        f"> {log_path} 2>&1")
    viz_proc = _popen(viz_cmd)

    def _read_log() -> str:
        with open(log_path) as f:
            return f.read()

    try:
        if not _wait_running(viz_proc):
            print(f"FAIL: overlume_node (layer-parameter run) exited early:\n"
                  f"{_read_log()[-2000:]}", file=sys.stderr)
            return 1

        if not _lifecycle("configure") or not _lifecycle("activate"):
            print("FAIL: configure/activate failed (layer-parameter run).", file=sys.stderr)
            return 1
        time.sleep(0.3)
        expected = ("set_environment_source: failed to open "
                    "'mapbox://buildings,terrain?imagery=satellite'")
        if expected not in _read_log():
            print(f"FAIL (6a): on_activate() did not compose environment_terrain:=enable and "
                  f"environment_imagery:=satellite into the URI -- expected {expected!r}. "
                  f"Log tail:\n{_read_log()[-3000:]}", file=sys.stderr)
            return 1
        print("PASS (6a): on_activate() composes mapbox://buildings,terrain?imagery=satellite.")

        before = len(_read_log())
        if _param_set_ok("environment_source_uri", "'mapbox://buildings?materials=original'"):
            print("FAIL (6b): the live mapbox:// switch was accepted with MAPBOX_TOKEN unset -- "
                  "expected rejected.", file=sys.stderr)
            return 1
        time.sleep(0.3)
        log_text = _read_log()[before:]
        expected = ("set_environment_source: failed to open "
                    "'mapbox://buildings,terrain?materials=original&imagery=satellite'")
        if expected not in log_text:
            print(f"FAIL (6b): the live environment_source_uri switch did not compose the two "
                  f"layer parameters -- expected {expected!r}. New log:\n{log_text[-3000:]}",
                  file=sys.stderr)
            return 1
        print("PASS (6b): the live switch composes "
              "mapbox://buildings,terrain?materials=original&imagery=satellite.")

        print("PASS (check 6, separate run): environment_terrain and environment_imagery reach "
              "both compose calls.")
        return 0
    finally:
        _kill(viz_proc)

def main() -> int:
```

Replace

```python
    return _run_hidden_arm_check()
```

with

```python
    status = _run_hidden_arm_check()
    return status if status else _run_layer_parameter_check()
```

Reverts it catches: the two arguments dropped from the `on_activate` compose (6a finds
`failed to open 'mapbox://buildings'`); dropped from the live-switch compose (6b finds
`failed to open 'mapbox://buildings?materials=original'`); the parameters not declared (6a: the
override is ignored).

- [ ] **Step 6: Run both node checks and see them fail**

```bash
cd "$WORKTREE"
ROS_DOMAIN_ID=84 python3 ros/src/overlume_ros/test/test_asset_paths.py; echo "exit=$?"
ROS_DOMAIN_ID=84 python3 ros/src/overlume_ros/test/test_environment_live_switch.py; echo "exit=$?"
```

Run in the foreground (about a minute each: three node launches each). Expected, first command:
the `PASS (1/3): …` line, then on stderr

```text
FAIL: `ros2 param get` for environment_terrain and environment_imagery returned ('', ''), expected ('ion:1', 'sat').
```

and `exit=1`. Second command: `PASS (1/4)` to `PASS (4/4)`, `PASS: overlume_node environment
live-switch param test passed.`, `PASS (a)` to `PASS (d)` and `PASS (check 5, separate run): …`,
then on stderr a line starting

```text
FAIL (6a): on_activate() did not compose environment_terrain:=enable and environment_imagery:=satellite into the URI -- expected "set_environment_source: failed to open 'mapbox://buildings,terrain?imagery=satellite'".
```

whose log tail shows `set_environment_source: failed to open 'mapbox://buildings' -- no buildings
this run`, and `exit=1`.

- [ ] **Step 7: Declare the parameters, warn, pass them to the composer, draw the text**

In `ros/src/overlume_ros/include/overlume_ros/overlume_node.hpp`, replace

```cpp
    double environment_tile_radius_m_{700.0};
    bool environment_fallback_warned_{false};
```

with

```cpp
    double environment_tile_radius_m_{700.0};
    std::string environment_terrain_;  // "" leaves the URI alone (compose_environment_source_uri)
    std::string environment_imagery_;
    bool environment_fallback_warned_{false};
```

In `ros/src/overlume_ros/src/overlume_node.cpp`, apply these four replacements.

Replace

```cpp
    environment_tile_radius_m_ = declare_parameter<double>("environment_tile_radius_m", 700.0);
    environment_attribution_ = declare_parameter<bool>("environment_attribution", true);
```

with

```cpp
    environment_tile_radius_m_ = declare_parameter<double>("environment_tile_radius_m", 700.0);
    environment_terrain_ = declare_parameter<std::string>("environment_terrain", "");
    environment_imagery_ = declare_parameter<std::string>("environment_imagery", "");
    for (const std::string& warning :
         {environment_layer_warning("environment_terrain", environment_terrain_, "enable",
                                    environment_source_uri_),
          environment_layer_warning("environment_imagery", environment_imagery_, "satellite",
                                    environment_source_uri_)}) {
        if (!warning.empty()) RCLCPP_WARN(get_logger(), "%s", warning.c_str());
    }
    environment_attribution_ = declare_parameter<bool>("environment_attribution", true);
```

Replace (the `on_activate` compose)

```cpp
            environment_follow_terrain_, environment_ground_bias_m_, environment_replaces_ground_,
            environment_max_tilt_deg_, environment_brightness_, environment_tile_radius_m_);
```

with

```cpp
            environment_follow_terrain_, environment_ground_bias_m_, environment_replaces_ground_,
            environment_max_tilt_deg_, environment_brightness_, environment_tile_radius_m_,
            environment_terrain_, environment_imagery_);
```

Replace (the live-switch compose)

```cpp
                        environment_replaces_ground_, environment_max_tilt_deg_,
                        environment_brightness_, environment_tile_radius_m_);
```

with

```cpp
                        environment_replaces_ground_, environment_max_tilt_deg_,
                        environment_brightness_, environment_tile_radius_m_, environment_terrain_,
                        environment_imagery_);
```

The third compose call (`fallback_dir_from_source_uri(compose_environment_source_uri(…))`, the
network-loss WARN) stays as it is: it reads `fallback=` only.

Replace

```cpp
        overlume::environment_source_state(renderer_));
    if (attribution != AttributionKind::NONE) {
        const overlume::HudColors hud_colors = overlume::get_hud_colors(renderer_);
        const overlume::ros::HudRgb text_rgb{hud_colors.text_color[0], hud_colors.text_color[1],
                                             hud_colors.text_color[2]};
        const uint32_t w = static_cast<uint32_t>(out_width_);
        const uint32_t h = static_cast<uint32_t>(out_height_);
        const bool drawn =
            attribution == AttributionKind::MAPBOX
                ? overlume::ros::CompositeAttribution(frame_buf_.data(), w, h, mapbox_logo_,
                                                      kMapboxAttributionText, text_rgb,
                                                      hud_colors.scale, hud_font_path_.c_str())
                : overlume::ros::DrawText(frame_buf_.data(), w, h, "3D Tiles data (c) Google", 8.0f,
                                          static_cast<float>(out_height_) - 8.0f, text_rgb,
                                          hud_colors.scale, hud_font_path_.c_str());
        if (!drawn && !environment_attribution_warned_) {
            RCLCPP_WARN(get_logger(),
                        "environment attribution: failed to load/use font '%s' -- "
                        "%s attribution text not drawn this run",
                        hud_font_path_.c_str(),
                        attribution == AttributionKind::MAPBOX ? "Mapbox" : "Google");
```

with

```cpp
        overlume::environment_source_state(renderer_));
    // ponytail: no overlay credits yet; the once-a-second read of environment_source_credits()
    // will pass them here and to the kind above.
    const std::string attribution_text = environment_attribution_text(attribution, "");
    if (!attribution_text.empty()) {
        const overlume::HudColors hud_colors = overlume::get_hud_colors(renderer_);
        const overlume::ros::HudRgb text_rgb{hud_colors.text_color[0], hud_colors.text_color[1],
                                             hud_colors.text_color[2]};
        const uint32_t w = static_cast<uint32_t>(out_width_);
        const uint32_t h = static_cast<uint32_t>(out_height_);
        const bool drawn =
            attribution == AttributionKind::MAPBOX
                ? overlume::ros::CompositeAttribution(frame_buf_.data(), w, h, mapbox_logo_,
                                                      attribution_text.c_str(), text_rgb,
                                                      hud_colors.scale, hud_font_path_.c_str())
                : overlume::ros::DrawText(frame_buf_.data(), w, h, attribution_text.c_str(), 8.0f,
                                          static_cast<float>(out_height_) - 8.0f, text_rgb,
                                          hud_colors.scale, hud_font_path_.c_str());
        if (!drawn && !environment_attribution_warned_) {
            RCLCPP_WARN(get_logger(),
                        "environment attribution: failed to load/use font '%s' -- "
                        "%s attribution text not drawn this run",
                        hud_font_path_.c_str(),
                        attribution == AttributionKind::MAPBOX   ? "Mapbox"
                        : attribution == AttributionKind::GOOGLE ? "Google"
                                                                 : "overlay credits");
```

In `ros/src/overlume_ros/config/default_params.yaml`, replace

```yaml
    environment_tile_radius_m: 700.0
    environment_attribution: true
```

with

```yaml
    environment_tile_radius_m: 700.0
    environment_terrain: ""
    environment_imagery: ""
    environment_attribution: true
```

- [ ] **Step 8: Build and run both checks**

```bash
cd "$WORKTREE"
ros/colcon_build.sh overlume_ros 2>&1 | tail -n 3
ROS_DOMAIN_ID=84 python3 ros/src/overlume_ros/test/test_asset_paths.py; echo "exit=$?"
ROS_DOMAIN_ID=84 python3 ros/src/overlume_ros/test/test_environment_live_switch.py; echo "exit=$?"
source /opt/ros/humble/setup.bash
ros/build/overlume_ros/test_environment_source_uri | tail -n 1
```

Neither Python check is in the gate (neither is CMake-registered), so this step is their only
run; the review gate reruns them. Expected: `Summary: 1 package finished […]`; then
`PASS (1/3): …`,

```text
PASS (2/3): initial_theme:=light_clay configured cleanly, param reads back light_clay, environment_attribution_logo:=false reads back and logs the logo off, environment_terrain:=ion:1 and environment_imagery:=sat read back as strings and 'sat' is warned once as ignored, no fallback WARN.
```

`PASS (3/3): …`, `PASS: overlume_node asset-path resolution test passed.` and `exit=0`; then
checks 1-5 as in Step 6 and

```text
PASS (6a): on_activate() composes mapbox://buildings,terrain?imagery=satellite.
PASS (6b): the live switch composes mapbox://buildings,terrain?materials=original&imagery=satellite.
PASS (check 6, separate run): environment_terrain and environment_imagery reach both compose calls.
```

and `exit=0`; then `[  PASSED  ] 37 tests.`

- [ ] **Step 9: Format**

```bash
cd "$WORKTREE"
git add ros/src/overlume_ros/include/overlume_ros/environment_source_uri.hpp \
    ros/src/overlume_ros/include/overlume_ros/overlume_node.hpp \
    ros/src/overlume_ros/src/overlume_node.cpp ros/src/overlume_ros/config/default_params.yaml \
    ros/src/overlume_ros/test/test_environment_source_uri.cpp \
    ros/src/overlume_ros/test/test_asset_paths.py \
    ros/src/overlume_ros/test/test_environment_live_switch.py
tools/check_format.sh --fix
git diff --stat
tools/check_format.sh
tools/check_spdx.sh
```

Expected: `check_format.sh: reformatted … files with clang-format 23.1.1`; `git diff --stat`
prints nothing (the code above is already formatted); `check_format.sh: PASS (… files,
clang-format 23.1.1)`; `check_spdx.sh: PASS (… files carry the SPDX identifier)`.

- [ ] **Step 10: Gate**

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
ROS_DOMAIN_ID=84 CI_VISUAL_MODE_DOMAIN_ID=84 \
    flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock tools/ci_visual_mode.sh
```

Run in the foreground (Bash timeout 600000 ms). Exit 75 means another lane held the lock for
300 s: run the same command again. No library file changes, so no reconfigure. Expected (the
baseline plus this task's fourteen node tests):
- `PASS  POD header check`
- `PASS  library ctest suite  (100% tests passed, 0 tests failed out of 442 (424 ok / 8 skipped))`
- `PASS  node gtests  (Summary: 395 tests, 0 errors, 0 failures, 0 skipped)`
- `PASS  WS bridge pytest suite  (94 passed in …)`, the time varying
- `PASS  golden suite: 25 ok, 0 skipped`
- `PASS  examples  (6 run, all exited 0 with non-empty output)`
- `OVERALL: PASS`, exit 0.

- [ ] **Step 11: Commit**

```bash
cd "$WORKTREE"
git add ros/src/overlume_ros/include/overlume_ros/environment_source_uri.hpp \
    ros/src/overlume_ros/include/overlume_ros/overlume_node.hpp \
    ros/src/overlume_ros/src/overlume_node.cpp ros/src/overlume_ros/config/default_params.yaml \
    ros/src/overlume_ros/test/test_environment_source_uri.cpp \
    ros/src/overlume_ros/test/test_asset_paths.py \
    ros/src/overlume_ros/test/test_environment_live_switch.py
git commit -F - <<'EOF'
feat(node): environment_terrain and environment_imagery, attribution text with credits

Two configure-time string parameters add terrain and imagery to the
environment source URI when it lacks them; the default "" leaves the URI
alone. environment_terrain=enable adds `terrain` to a mapbox:// layer
list, ion:<id> becomes terrain=<id> on ion://, and disable becomes
terrain=off there. environment_imagery=satellite appends
imagery=satellite on mapbox://, ion:<id> appends imagery=<id> on ion://,
and disable appends imagery=off on both. The values are strings YAML
never reads as a bool or an int. A value meant for the other scheme
appends nothing and is warned once at configure, so one deployment can
set both and still switch sources; a value that fits neither scheme is
warned too. Keys are matched whole, so ion's follow_terrain= is not taken
for terrain=. test_asset_paths.py reads both back and checks the WARN;
test_environment_live_switch.py's check 6 proves that on_activate and
the live switch both compose them (offline: with no MAPBOX_TOKEN the
node's failed-to-open line names the composed URI).

environment_attribution_text(kind, credits) builds the attribution line:
the kind's base text, each overlay credit after a space, and Mapbox's
"Improve this map" last. environment_attribution_kind gains a
credits_non_empty argument (default false) and a CREDITS kind for an
ion:// source other than Google that has credits, whose line is the
credits alone, so ion imagery (Bing's, for osm-terrain) can be credited.
The node passes no credits yet, so every line on screen is unchanged
until the node reads the library's overlay credits once a second.

Claude-Session: <the executing session's URL>
EOF
```

---

### Task 7: GUI sources, bridge, token check

**Lane:** 4. **Depends on:** Task 6 (lane order only; no code dependency). Task 8 documents the
three entries and the preflight's four codes; Task 15 uses `--arm mapbox-satellite` and the
preflight.

**Files:**
- Modify: `tools/vcam_gui.py:106-113` (the environment source list and its URI table)
- Modify: `tools/vcam_ws_bridge.py:33-43` (docstring), `:91-98` (set and URI table), `:256-258`
  (the error string)
- Modify: `tools/check_mapbox_token.sh` (header comment 5-10; the probes 38-50; whole file below)
- Test: modify `tools/test_vcam_ws_bridge.py:135-137` (parametrize), after `:148` (URI pin),
  `:563-564` (resolve), `:594` (lockstep order); `tools/test_check_mapbox_token.py` (whole file
  below)

**Interfaces:**
- Consumes:
  - `def parse_cmd(text: str)` (`tools/vcam_ws_bridge.py:155`): its `set_environment_source`
    branch accepts a name only if it is in `ENVIRONMENT_PRESETS` (`:253-259`); the handler then
    sends `ENVIRONMENT_PRESET_URIS[preset]` (`:641`). Both generic.
  - `def resolve_environment_preset(source_uri, own_asset_uri)` (`tools/vcam_gui.py:129-140`):
    exact `==` against `ENVIRONMENT_PRESET_URIS_FIXED`; `initial_environment_sensitivity`
    (`:115-121`): true for every name but `"clipped"`; the combo rows come from
    `ENVIRONMENT_PRESETS` in list order (`:394-396`). No new code for the new rows.
  - The node stores the requested, uncomposed `environment_source_uri` after a successful switch
    (`ros/src/overlume_ros/src/overlume_node.cpp:913`), which is what exact matching sees; Task
    6's composer adds keys only to the composed copy.
  - Global Constraints: the entries `mapbox-satellite`, `mapbox-original-satellite`, `osm-terrain`
    and their URIs (spec §4), the DEM and satellite templates, and the PASS line of "Texts other
    tasks match verbatim".
- Produces:
  - GUI: `ENVIRONMENT_PRESETS == ["baked", "osm", "clipped", "google", "mapbox",
    "mapbox-original", "mapbox-satellite", "mapbox-original-satellite", "osm-terrain"]`; bridge:
    the same names in its set. Both URI tables gain
    `"mapbox-satellite": "mapbox://buildings,terrain?imagery=satellite"`,
    `"mapbox-original-satellite": "mapbox://buildings,terrain?materials=original&imagery=satellite"`
    and `"osm-terrain": "ion://96188?terrain=1&imagery=2"`.
  - The WS command `{"cmd": "set_environment_source", "preset": "mapbox-satellite" |
    "mapbox-original-satellite" | "osm-terrain"}`.
  - `tools/check_mapbox_token.sh`: no arguments; exactly one line on stdout, nothing on stderr:
    - `PASS: token retrieves the Downtown Dubai tiles (landmark tile HTTP 200, footprint tile HTTP 200, DEM tile HTTP 200, satellite tile HTTP 200)`,
      exit 0;
    - `FAIL: <landmark>, <footprint>, <DEM>, <satellite> (see docs/runbooks/mapbox.md)`, exit 1,
      each part `<label> HTTP <code>` or `<label> transport error (curl exit <n>)` for the labels
      `landmark tile`, `footprint tile`, `DEM tile`, `satellite tile`;
    - `SKIP: MAPBOX_TOKEN is not set (see docs/runbooks/mapbox.md)`, exit 2, no request sent.

**API notes:**
- `tools/check_mapbox_token.sh:25-36`: `check()` feeds curl its URL on stdin (`--config -`) and
  prints one part; `:38-50` today probes two tiles. The two new probes use the Global
  Constraints' templates: DEM `…/mapbox.mapbox-terrain-dem-v1/{z}/{x}/{y}@2x.pngraw` at z14
  10707/7006 (the landmark tile's own square; the DEM stops at z14) and satellite
  `…/mapbox.satellite/{z}/{x}/{y}@2x.jpg90` with Mapbox's XYZ row, the row a request leaves the
  process with (cesium's template spells it `{reverseY}`, R1), at z18 171320/112100 (the imagery
  cap; inside z16
  42830/28025, x and y times 4, and inside z14 10707/7006: 171320 / 16 = 10707.5, 112100 / 16 =
  7006.25).
- `tools/test_check_mapbox_token.py:22-31`: the stub curl routes by a substring of the config it
  reads; `*terrain-dem*` and `*satellite*` match only the new URLs (the footprint URL holds
  neither), and the `case` checks them before the footprint fallback.
- `tools/ci_visual_mode.sh:136-148`: stage 4 runs `tools/test_vcam_ws_bridge.py` and
  `tools/test_check_mapbox_token.py` with `-p no:anyio`; its 94 tests are 86 + 8 (collected in a
  scratch copy). `.github/workflows/lint.yml:63-66` runs `python -m pytest tools/ -q` on hosted
  lint, where the GUI tests skip on `pytest.importorskip("gi")`.
- `tools/test_vcam_ws_bridge.py:78-80`: `"preset": "satellite"` stays a malformed name (the entry
  is `mapbox-satellite`).
- Verified on a scratch copy of `tools/`: with the Step 1-2 tests and today's code, the `-k`
  subset gives `6 failed, 17 passed, 67 deselected` and the token file `7 failed, 3 passed`; after
  Steps 3-4, `23 passed, 67 deselected` and `10 passed`, `shellcheck` reports nothing on the new
  script, and the two files collect 100 tests. Reverts checked on the script: the DEM's
  `|| status=1` dropped fails `[dem-404]`; the satellite probe replaced by a fixed "HTTP 200"
  fails `[all-401]`, `[satellite-403]` and the stdin test; the satellite tile at z16 instead of
  z18 fails the stdin test.

- [ ] **Step 1: Extend the bridge and GUI tests**

In `tools/test_vcam_ws_bridge.py`, replace

```python
@pytest.mark.parametrize("preset", [
    "baked", "osm", "clipped", "google", "mapbox", "mapbox-original",
])
```

with

```python
@pytest.mark.parametrize("preset", [
    "baked", "osm", "clipped", "google", "mapbox", "mapbox-original", "mapbox-satellite",
    "mapbox-original-satellite", "osm-terrain",
])
```

Replace

```python
    assert ENVIRONMENT_PRESET_URIS["mapbox-original"] == "mapbox://buildings?materials=original"
```

with

```python
    assert ENVIRONMENT_PRESET_URIS["mapbox-original"] == "mapbox://buildings?materials=original"

def test_environment_imagery_entries_map_to_their_uris():
    # Terrain with imagery draped on it: Mapbox Satellite on terrain-DEM, and Cesium World Terrain
    # (ion asset 1) with Bing Maps Aerial (ion asset 2) under Cesium OSM Buildings.
    assert ENVIRONMENT_PRESET_URIS["mapbox-satellite"] == \
        "mapbox://buildings,terrain?imagery=satellite"
    assert ENVIRONMENT_PRESET_URIS["mapbox-original-satellite"] == \
        "mapbox://buildings,terrain?materials=original&imagery=satellite"
    assert ENVIRONMENT_PRESET_URIS["osm-terrain"] == "ion://96188?terrain=1&imagery=2"
```

In `test_resolve_environment_preset`, replace

```python
    assert resolve_environment_preset(
        "mapbox://buildings?materials=original", "") == "mapbox-original"
```

with

```python
    assert resolve_environment_preset(
        "mapbox://buildings?materials=original", "") == "mapbox-original"
    assert resolve_environment_preset(
        "mapbox://buildings,terrain?imagery=satellite", "") == "mapbox-satellite"
    assert resolve_environment_preset(
        "mapbox://buildings,terrain?materials=original&imagery=satellite", "") == \
        "mapbox-original-satellite"
    assert resolve_environment_preset("ion://96188?terrain=1&imagery=2", "") == "osm-terrain"
```

In `test_gui_and_bridge_environment_preset_tables_agree`, replace

```python
    assert GUI_PRESETS == ["baked", "osm", "clipped", "google", "mapbox", "mapbox-original"]
```

with

```python
    assert GUI_PRESETS == ["baked", "osm", "clipped", "google", "mapbox", "mapbox-original",
                           "mapbox-satellite", "mapbox-original-satellite", "osm-terrain"]
```

Reverts each test catches:
- `test_set_environment_source_valid[mapbox-satellite|mapbox-original-satellite|osm-terrain]`: a
  name missing from the bridge's set.
- `test_environment_imagery_entries_map_to_their_uris`: a bridge URI missing or changed (no `gi`
  needed, so hosted lint catches it too).
- `test_resolve_environment_preset`: a GUI URI missing or changed.
- `test_gui_and_bridge_environment_preset_tables_agree`: one file edited without the other, the
  GUI order changed, or a name with no URI (the handler's `ENVIRONMENT_PRESET_URIS[preset]` would
  raise `KeyError`).

- [ ] **Step 2: Replace the token-check tests**

Replace the whole of `tools/test_check_mapbox_token.py` with:

```python
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

"""Offline tests for tools/check_mapbox_token.sh: a stub curl stands in for the network.

Run: python3 -m pytest tools/test_check_mapbox_token.py -q -p no:anyio
"""

import os
import subprocess

import pytest

SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "check_mapbox_token.sh")
TOKEN = "pk.test-token-not-real"
LANDMARK_URL = ("https://api.mapbox.com/3dtiles/v1/mapbox.mapbox-3dbuildings-v1-meshopt/"
                "14/10707/7006.glb")
FOOTPRINT_URL = "https://api.mapbox.com/v4/mapbox.mapbox-streets-v8/16/42830/28025.vector.pbf"
DEM_URL = "https://api.mapbox.com/v4/mapbox.mapbox-terrain-dem-v1/14/10707/7006@2x.pngraw"
SATELLITE_URL = "https://api.mapbox.com/v4/mapbox.satellite/18/171320/112100@2x.jpg90"

# Logs its argv and the config it reads on stdin, then answers like `curl -w '%{http_code}'`.
# STUB_LANDMARK, STUB_FOOTPRINT, STUB_DEM and STUB_SATELLITE hold "<http code>:<curl exit status>".
STUB_CURL = r"""#!/bin/sh
config=$(cat)
printf 'argv: %s\n%s\n' "$*" "$config" >> "$STUB_LOG"
case "$config" in
    *3dtiles*) reply=$STUB_LANDMARK ;;
    *terrain-dem*) reply=$STUB_DEM ;;
    *satellite*) reply=$STUB_SATELLITE ;;
    *) reply=$STUB_FOOTPRINT ;;
esac
printf '%s' "${reply%%:*}"
exit "${reply#*:}"
"""


def run_check(tmp_path, token, landmark="200:0", footprint="200:0", dem="200:0",
              satellite="200:0"):
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    (bin_dir / "curl").write_text(STUB_CURL)
    (bin_dir / "curl").chmod(0o755)
    env = {k: v for k, v in os.environ.items() if k != "MAPBOX_TOKEN"}
    env.update(PATH=f"{bin_dir}{os.pathsep}{env['PATH']}", STUB_LOG=str(tmp_path / "curl.log"),
               STUB_LANDMARK=landmark, STUB_FOOTPRINT=footprint, STUB_DEM=dem,
               STUB_SATELLITE=satellite)
    if token is not None:
        env["MAPBOX_TOKEN"] = token
    return subprocess.run(["bash", SCRIPT], env=env, stdin=subprocess.DEVNULL,
                          capture_output=True, text=True, timeout=30)


@pytest.mark.parametrize("token", [None, "", " \n"])
def test_missing_token_skips_without_a_request(tmp_path, token):
    r = run_check(tmp_path, token)
    assert r.returncode == 2
    assert r.stdout == "SKIP: MAPBOX_TOKEN is not set (see docs/runbooks/mapbox.md)\n"
    assert r.stderr == ""
    assert not (tmp_path / "curl.log").exists()


def test_all_four_tiles_200_pass(tmp_path):
    r = run_check(tmp_path, TOKEN)
    assert r.returncode == 0, r.stdout
    assert r.stdout == ("PASS: token retrieves the Downtown Dubai tiles "
                        "(landmark tile HTTP 200, footprint tile HTTP 200, "
                        "DEM tile HTTP 200, satellite tile HTTP 200)\n")
    assert r.stderr == ""
    assert TOKEN not in r.stdout + r.stderr


@pytest.mark.parametrize("landmark,footprint,dem,satellite,codes", [
    ("401:0", "401:0", "401:0", "401:0",
     "landmark tile HTTP 401, footprint tile HTTP 401, DEM tile HTTP 401, satellite tile HTTP 401"),
    ("204:0", "200:0", "200:0", "200:0",
     "landmark tile HTTP 204, footprint tile HTTP 200, DEM tile HTTP 200, satellite tile HTTP 200"),
    ("200:0", "000:28", "200:0", "200:0",
     "landmark tile HTTP 200, footprint tile transport error (curl exit 28), DEM tile HTTP 200, "
     "satellite tile HTTP 200"),
    ("200:0", "200:0", "404:0", "200:0",
     "landmark tile HTTP 200, footprint tile HTTP 200, DEM tile HTTP 404, satellite tile HTTP 200"),
    ("200:0", "200:0", "200:0", "403:0",
     "landmark tile HTTP 200, footprint tile HTTP 200, DEM tile HTTP 200, satellite tile HTTP 403"),
], ids=["all-401", "landmark-204", "footprint-timeout", "dem-404", "satellite-403"])
def test_anything_but_four_200s_fails(tmp_path, landmark, footprint, dem, satellite, codes):
    r = run_check(tmp_path, TOKEN, landmark, footprint, dem, satellite)
    assert r.returncode == 1
    assert r.stdout == f"FAIL: {codes} (see docs/runbooks/mapbox.md)\n"
    assert r.stderr == ""


def test_token_reaches_curl_only_on_stdin_and_trimmed(tmp_path):
    # The library trims surrounding whitespace, so the preflight sends what the node will send.
    r = run_check(tmp_path, f" {TOKEN}\n", landmark="401:0", footprint="000:6")
    assert TOKEN not in r.stdout + r.stderr
    log = (tmp_path / "curl.log").read_text().splitlines()
    assert [line for line in log if line.startswith("argv: ")] == [
        "argv: -s --max-time 30 --config - -o /dev/null -w %{http_code}"] * 4
    assert [line for line in log if line.startswith("url = ")] == [
        f'url = "{LANDMARK_URL}?access_token={TOKEN}"',
        f'url = "{FOOTPRINT_URL}?access_token={TOKEN}"',
        f'url = "{DEM_URL}?access_token={TOKEN}"',
        f'url = "{SATELLITE_URL}?access_token={TOKEN}"',
    ]
```

Reverts each test catches: a probe dropped or its failure not counted (`[dem-404]`,
`[satellite-403]`, the stdin test); a URL off the pinned template, zoom or tile (the stdin test);
the four parts out of order or a label renamed (every PASS/FAIL test); the existing contract
(SKIP without a request, the token only on stdin and trimmed, nothing on stderr) as before.

- [ ] **Step 3: Run the new tests and see them fail**

```bash
cd "$WORKTREE"
python3 -m pytest tools/test_vcam_ws_bridge.py -q -p no:anyio -k "environment or mapbox or token"
python3 -m pytest tools/test_check_mapbox_token.py -q -p no:anyio
```

The `-k` filter keeps out the end-to-end tests that launch ROS nodes; the gate runs the whole
file. Expected, first command:

```text
FAILED tools/test_vcam_ws_bridge.py::test_set_environment_source_valid[mapbox-satellite]
FAILED tools/test_vcam_ws_bridge.py::test_set_environment_source_valid[mapbox-original-satellite]
FAILED tools/test_vcam_ws_bridge.py::test_set_environment_source_valid[osm-terrain]
FAILED tools/test_vcam_ws_bridge.py::test_environment_imagery_entries_map_to_their_uris
FAILED tools/test_vcam_ws_bridge.py::test_resolve_environment_preset - AssertionErr...
FAILED tools/test_vcam_ws_bridge.py::test_gui_and_bridge_environment_preset_tables_agree
6 failed, 17 passed, 67 deselected in …s
```

with these causes in the `E` lines: `ValueError: set_environment_source: preset must be "baked",
"osm", "clipped", "google", "mapbox" or "mapbox-original"` (three times), `KeyError:
'mapbox-satellite'`, `AssertionError: assert None == 'mapbox-satellite'`, and the list diff
`Right contains 3 more items, first extra item: 'mapbox-satellite'`.

Expected, second command: `7 failed, 3 passed in …s`: `test_all_four_tiles_200_pass` and the
five `test_anything_but_four_200s_fails` cases fail on the stdout line (two codes, not four; the
`[dem-404]` and `[satellite-403]` cases on `assert 0 == 1`), and
`test_token_reaches_curl_only_on_stdin_and_trimmed` on two `argv:` lines instead of four. The
three SKIP cases pass.

- [ ] **Step 4: Add the three entries to the GUI and the bridge**

In `tools/vcam_gui.py`, replace

```python
ENVIRONMENT_PRESETS = ["baked", "osm", "clipped", "google", "mapbox", "mapbox-original"]
ENVIRONMENT_PRESET_URIS_FIXED = {
    "baked": "",
    "osm": "ion://96188",
    "google": "ion://2275207?materials=original&cache=off",
    "mapbox": "mapbox://buildings",
    "mapbox-original": "mapbox://buildings?materials=original",
}
```

with

```python
ENVIRONMENT_PRESETS = [
    "baked", "osm", "clipped", "google", "mapbox", "mapbox-original", "mapbox-satellite",
    "mapbox-original-satellite", "osm-terrain",
]
ENVIRONMENT_PRESET_URIS_FIXED = {
    "baked": "",
    "osm": "ion://96188",
    "google": "ion://2275207?materials=original&cache=off",
    "mapbox": "mapbox://buildings",
    "mapbox-original": "mapbox://buildings?materials=original",
    "mapbox-satellite": "mapbox://buildings,terrain?imagery=satellite",
    "mapbox-original-satellite": "mapbox://buildings,terrain?materials=original&imagery=satellite",
    "osm-terrain": "ion://96188?terrain=1&imagery=2",
}
```

In `tools/vcam_ws_bridge.py`, replace the docstring lines

```text
    {"cmd": "set_environment_source", "preset": "baked"|"osm"|"clipped"|"google"|
     "mapbox"|"mapbox-original"}
```

with

```text
    {"cmd": "set_environment_source", "preset": "baked"|"osm"|"clipped"|"google"|
     "mapbox"|"mapbox-original"|"mapbox-satellite"|"mapbox-original-satellite"|
     "osm-terrain"}
```

replace

```text
        "mapbox-original"->"mapbox://buildings?materials=original" (no
        cache=off: that source never caches, see docs/runbooks/mapbox.md),
```

with

```text
        "mapbox-original"->"mapbox://buildings?materials=original" (no
        cache=off: that source never caches, see docs/runbooks/mapbox.md),
        "mapbox-satellite"->"mapbox://buildings,terrain?imagery=satellite",
        "mapbox-original-satellite"->
        "mapbox://buildings,terrain?materials=original&imagery=satellite",
        "osm-terrain"->"ion://96188?terrain=1&imagery=2" (Cesium World
        Terrain with Bing Maps Aerial, see docs/runbooks/terrain_imagery.md),
```

replace

```python
ENVIRONMENT_PRESETS = {"baked", "osm", "clipped", "google", "mapbox", "mapbox-original"}
ENVIRONMENT_PRESET_URIS = {
    "baked": "",
    "osm": "ion://96188",
    "google": "ion://2275207?materials=original&cache=off",
    "mapbox": "mapbox://buildings",
    "mapbox-original": "mapbox://buildings?materials=original",
}
```

with

```python
ENVIRONMENT_PRESETS = {
    "baked", "osm", "clipped", "google", "mapbox", "mapbox-original", "mapbox-satellite",
    "mapbox-original-satellite", "osm-terrain",
}
ENVIRONMENT_PRESET_URIS = {
    "baked": "",
    "osm": "ion://96188",
    "google": "ion://2275207?materials=original&cache=off",
    "mapbox": "mapbox://buildings",
    "mapbox-original": "mapbox://buildings?materials=original",
    "mapbox-satellite": "mapbox://buildings,terrain?imagery=satellite",
    "mapbox-original-satellite": "mapbox://buildings,terrain?materials=original&imagery=satellite",
    "osm-terrain": "ion://96188?terrain=1&imagery=2",
}
```

and replace

```python
                'set_environment_source: preset must be "baked", "osm", "clipped", "google", '
                '"mapbox" or "mapbox-original"')
```

with

```python
                'set_environment_source: preset must be "baked", "osm", "clipped", "google", '
                '"mapbox", "mapbox-original", "mapbox-satellite", "mapbox-original-satellite" '
                'or "osm-terrain"')
```

Nothing else changes: the handler's `else` branch and the GUI's send, sensitivity and reverse map
are generic, and the identifiers stay (`ENVIRONMENT_PRESETS`, the wire field `preset`).

- [ ] **Step 5: Probe the DEM and a satellite tile in the preflight**

Replace the whole of `tools/check_mapbox_token.sh` with (mode stays 755):

```bash
#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

# Demo preflight for the mapbox:// environment source (docs/runbooks/mapbox.md), shaped like
# overlume/scripts/check_cesium_token.sh: requests one Downtown Dubai tile from each endpoint the
# source uses (landmark, footprint, terrain-DEM, satellite) with $MAPBOX_TOKEN and prints one line.
# PASS (exit 0) when all four answer 200, FAIL (exit 1) otherwise, SKIP (exit 2) with no request
# when the token is unset. It prints HTTP codes and curl exit statuses only, never a URL, the
# token or a body. Not part of the gate: it needs the network and a real token.

set -u
# Trim surrounding whitespace as the library does, so this checks the token the node will send.
token="${MAPBOX_TOKEN:-}"
token="${token#"${token%%[![:space:]]*}"}"
token="${token%"${token##*[![:space:]]}"}"
if [ -z "${token}" ]; then
    echo "SKIP: MAPBOX_TOKEN is not set (see docs/runbooks/mapbox.md)"
    exit 2
fi

# Prints "<label> HTTP <code>" or "<label> transport error (curl exit <n>)" and succeeds only on
# HTTP 200. curl reads the URL from stdin, so the token never reaches its argv (ps). No -S:
# curl's error text could carry the URL, so only its exit status is reported.
check() {
    local code rc
    code=$(printf 'url = "%s?access_token=%s"\n' "$2" "${token}" \
        | curl -s --max-time 30 --config - -o /dev/null -w '%{http_code}')
    rc=$?
    if [ "${rc}" -ne 0 ]; then
        echo "$1 transport error (curl exit ${rc})"
        return 1
    fi
    echo "$1 HTTP ${code}"
    [ "${code}" = "200" ]
}

# Downtown Dubai: z14 10707/7006 holds landmark models and is its own terrain-DEM tile (the DEM
# stops at z14); z16 42830/28025 is one of its footprint tiles, and z18 171320/112100 a satellite
# tile inside that (z18 is the imagery cap). The endpoints are the ones the library requests.
landmarks="https://api.mapbox.com/3dtiles/v1/mapbox.mapbox-3dbuildings-v1-meshopt"
footprints="https://api.mapbox.com/v4/mapbox.mapbox-streets-v8"
dem="https://api.mapbox.com/v4/mapbox.mapbox-terrain-dem-v1"
satellite="https://api.mapbox.com/v4/mapbox.satellite"
status=0
landmark=$(check "landmark tile" "${landmarks}/14/10707/7006.glb") || status=1
footprint=$(check "footprint tile" "${footprints}/16/42830/28025.vector.pbf") || status=1
dem_tile=$(check "DEM tile" "${dem}/14/10707/7006@2x.pngraw") || status=1
satellite_tile=$(check "satellite tile" "${satellite}/18/171320/112100@2x.jpg90") || status=1
codes="${landmark}, ${footprint}, ${dem_tile}, ${satellite_tile}"
if [ "${status}" -eq 0 ]; then
    echo "PASS: token retrieves the Downtown Dubai tiles (${codes})"
    exit 0
fi
echo "FAIL: ${codes} (see docs/runbooks/mapbox.md)"
exit 1
```

Never run it in this task: it would use the real token and the network. Its tests use the stub.

- [ ] **Step 6: Run the tests and the lint checks and see them pass**

```bash
cd "$WORKTREE"
python3 -m pytest tools/test_vcam_ws_bridge.py -q -p no:anyio -k "environment or mapbox or token"
python3 -m pytest tools/test_check_mapbox_token.py -q -p no:anyio
shellcheck --severity=error tools/*.sh
shellcheck tools/check_mapbox_token.sh
python3 -m py_compile tools/*.py
git ls-files -s tools/check_mapbox_token.sh | cut -c1-6
```

Expected: `23 passed, 67 deselected in …s`; `10 passed in …s`; no output from either
`shellcheck` command or from `py_compile`; `100755`.

- [ ] **Step 7: Format**

```bash
cd "$WORKTREE"
git add tools/vcam_gui.py tools/vcam_ws_bridge.py tools/test_vcam_ws_bridge.py \
    tools/check_mapbox_token.sh tools/test_check_mapbox_token.py
tools/check_format.sh --fix
tools/check_spdx.sh
git status --short
```

Expected: `check_format.sh: reformatted … files with clang-format 23.1.1` (no C/C++ change in
this task, so nothing moves); `check_spdx.sh: PASS (… files carry the SPDX identifier)`; and
exactly the five staged files:

```text
M  tools/check_mapbox_token.sh
M  tools/test_check_mapbox_token.py
M  tools/test_vcam_ws_bridge.py
M  tools/vcam_gui.py
M  tools/vcam_ws_bridge.py
```

Any other line (a C/C++ file rewritten by `--fix`) is a deviation to report, not to commit.

- [ ] **Step 8: Gate**

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
ROS_DOMAIN_ID=84 CI_VISUAL_MODE_DOMAIN_ID=84 \
    flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock tools/ci_visual_mode.sh
```

Run in the foreground (Bash timeout 600000 ms). Exit 75 means another lane held the lock for
300 s: run the same command again. Expected: six `PASS` lines and `OVERALL: PASS`, exit 0, with
Task 6's counts (442 ctest entries, 424 ok / 8 skipped; 395 node tests; 25 goldens all `ok`; 6
examples) and the bridge line `PASS  WS bridge pytest suite  (100 passed in …)`: 94 plus the three
parametrize cases, the URI pin and the two new failure cases.

- [ ] **Step 9: Commit**

```bash
cd "$WORKTREE"
git add tools/vcam_gui.py tools/vcam_ws_bridge.py tools/test_vcam_ws_bridge.py \
    tools/check_mapbox_token.sh tools/test_check_mapbox_token.py
git commit -F - <<'EOF'
feat(tools): three terrain-and-imagery GUI sources; preflight probes DEM and satellite

The vcam GUI's environment sources list and the WS bridge gain three
entries, kept in lockstep: mapbox-satellite ->
mapbox://buildings,terrain?imagery=satellite (clay buildings standing on
the DEM, Mapbox Satellite on the ground), mapbox-original-satellite ->
the same with materials=original, and osm-terrain ->
ion://96188?terrain=1&imagery=2 (Cesium OSM Buildings on Cesium World
Terrain with Bing Maps Aerial, the provider-agnostic proof). The bridge
handler and the GUI's send, sensitivity and reverse map are generic, so
only the tables, the docstring and the error text change.

tools/check_mapbox_token.sh also requests the Downtown Dubai terrain-DEM
tile (z14 10707/7006, @2x.pngraw) and a satellite tile at the imagery
cap (z18 171320/112100, @2x.jpg90), and prints the four codes on its one
line; PASS needs all four at 200. Its stub-curl tests route the two new
endpoints and pin the URLs and their order.

Claude-Session: <the executing session's URL>
EOF
```

---

### Task 8: Docs

**Lane:** 4. **Depends on:** Task 7 (the runbooks document its entries and the preflight's four
codes) and Task 6 (the node parameters and the attribution text). Names from lanes 1-3, 5 and 6
(loader log lines, `terrain.mat`, the overlay probe, `environment_source_credits`, test and golden
names) appear only in code spans, so the link check does not depend on their files; the
whole-branch review re-checks them against the merged code. Task 15 closes open item 23 and adds
the Shipped row.

**Files:**
- Create: `docs/runbooks/terrain_imagery.md`
- Modify: `docs/runbooks/mapbox.md` (title and introduction 1-6; §2 at 44-63; §3 at 65-98; §4 at
  100-116; §5's first bullet 124-131; §6 at 154-158; §7 at 195-198, 219-220, 239-240, 249-254;
  §8 at 271-273, 282-283, 284-291, 304-305)
- Modify: `docs/README.md:31-40` (the `mapbox.md` bullet, a `terrain_imagery.md` bullet after it,
  the `env_source_captures.md` bullet) and `:122-124` (this plan's entry under `## Plans`, after
  the Mapbox buildings plan; the deviation under Task 6)
- Modify: `docs/status.md:129-131` (open item 23 after item 22, before "Known gaps"; appended,
  because other entries cite open items by number)
- Modify: `CHANGELOG.md:12-14` (`[Unreleased]` → `### Added`: a bullet after the Mapbox one)
- Test: `python3 tools/check_docs_links.py`

**Interfaces:**
- Consumes: spec §4 (URI keys, node parameters, theme token, GUI entries, attribution), §5
  (library design, failure table), §8 (checks), §12 (limits); the Global Constraints' pinned
  strings; "Texts other tasks match verbatim" (the loader log lines, the preflight's PASS
  line); Task 6's composer rules (string values `disable`, `enable`, `satellite`, `ion:<id>`; a
  value the configured source ignores warned once at configure; a YAML boolean fails the
  configure), its configure WARN text and the `CREDITS` kind; Task 7's entries and preflight
  output; rulings R2, R3, R5, R9 and R16 (below).
- Produces: `docs/runbooks/terrain_imagery.md` (sections 1-8); `docs/runbooks/mapbox.md` with the
  §3 table rows for the layer list and `imagery` that head.md names (Task 15 edits nothing there
  unless the rehearsal finds drift):

  ```markdown
  | layer list (the path) | `buildings`, `terrain`: a comma list, each at most once, in any order | none: at least one layer | `buildings`: landmark models and extruded footprints. `terrain`: Mapbox terrain-DEM tiles as the ground, with the buildings standing on them. An empty, repeated or unknown name refuses the URI, and one log line names the supported layers. |
  | `imagery` | `satellite`, `off` | `off` | `satellite`: Mapbox Satellite draped on the terrain, up to z18, credited "© Maxar" on screen (§5). Needs `terrain` in the layer list: without it the URI is refused (`mapbox:// imagery=satellite needs the 'terrain' layer`). |
  ```

  and `docs/status.md` open item 23, which Task 15 closes by replacing its opening
  `**OPEN 2026-10-15 — imagery on terrain (sub-project 2) in progress.**`.

**API notes** (facts the prose states, and where they come from):
- `tools/check_docs_links.py:31-43` scans `README.md`, `AGENTS.md`, …, `CHANGELOG.md`,
  `docs/README.md`, `docs/status.md`, then every `*.md` under `docs/runbooks`, `docs/design` and
  `docs/adr` (sorted); `:46-48` and `:107-108` drop fenced and inline code before matching links;
  `:120-127` print `Broken links (N):` and one `  <file>: -> <target>` per link, else
  `OK: no broken links`, exit 1 or 0. The plan file `docs/plans/2026-10-15-imagery-terrain.md`
  exists from the plan's commit on, so open item 23's link to it resolves.
- `docs/README.md:31-34` (the Mapbox bullet), `:35-40` (the captures bullet, which says "preset";
  the Global Constraints make GUI entries environment sources, so the touched bullet says source).
  `:122-124` is the Mapbox buildings plan's entry, the last dated one under `## Plans`; this
  plan's entry follows it (the deviation under Task 6).
- `docs/status.md:129` is open item 22, `:131` the `## Known gaps / accepted exceptions` heading.
- `CHANGELOG.md:8-18`: `[Unreleased]` holds one Added bullet (Mapbox buildings) and two Fixed
  bullets.
- `docs/runbooks/mapbox.md`: §1-§8 as listed under **Files**; other pages cite it by file (and
  `docs/README.md:40` by "section 6"), so section numbers stay.
- Attribution widths: `MeasureText` sums stb_truetype's baked advances at 32 px per HUD scale
  (`ros/src/overlume_ros/src/hud_overlay.cpp:39`, `:51-63`, `:208-213`). Replicated with
  fontTools 4.29.1 on `ros/src/overlume_ros/assets/fonts/NotoSans-Regular.ttf` (scale
  32 / (ascent − descent), the sum of `hmtx` advances): today's text gives 524.76 px, matching
  `mapbox.md` §5; with " © Maxar" 626.33 px. With the layout rule of §5 (logo 176 px at 8 px, an 8 px gap, 8 px margins), the
  text lifts above the logo when the width is under 200 + 626.33 = 826.33 px (below 827 px), no
  longer fits under 16 + 626.33 = 642.33 px (below 643 px), and at 1280 px is cut above HUD scale
  (1280 − 16) / 626.33 = 2.02.
- "The node reads them once a second" (`terrain_imagery.md` §5, `mapbox.md` §5) describes the
  plan's end state: it holds once Task 12 lands (R5). The whole-branch review checks it against
  the merged node.
- Imagery failures (R2, R3): no task logs a line per failed imagery tile. cesium-native logs each
  failed raster tile itself, `Received response code {} for image {url}`, through the source's
  redacting logger (the token shows as `access_token=<redacted>`, the host stays visible). A
  refused token on imagery cascades through cesium's parent fallback to more than eight failed
  requests, so it falls the whole source back like any eight failures. `Mapbox rejected
  MAPBOX_TOKEN (HTTP {})` comes once per endpoint for the landmark, footprint and DEM requests
  only (Task 9 owns the DEM's).
- A `mapbox://` source with `terrain` runs with the ion follow-terrain defaults fixed: follow
  on, `ground_bias` 0.3 m, `replaces_ground` on, `max_tilt_deg` 2.0 (R9); the URI takes none of
  those keys.
- `terrain.mat` (head.md, lane 3's notes): no fade parameter, so terrain tiles pop in as the
  buildings do; `uvTransform` holds cesium's translation in xy and its scale in zw; each raster
  becomes an 8-bit RGBA texture in the sRGB format, mipmapped; detach sets `hasImagery = 0` and
  binds the 1×1 placeholder back, so the tile shows `terrain_color` again.
- Tile sizes at Dubai's 25.2° N: a z14 tile is 40,075,017 m × cos 25.2° / 2^14 = 2,213 m; a z18
  tile 138 m (16 × 16 = 256 per z14 tile); 64 mesh cells put DEM vertices 34.6 m apart.
- `tools/validate_logger_session.sh:16-27` (`--arm <name>`; `--` forwards the rest) and
  `tools/validate_visual_mode.sh:32` (`--param x` becomes `-p x`, YAML-typed: Task 6's API notes).
- `overlume/assets/themes/dark_adas.yaml:4` (`palette.ground`) and `:42`
  (`environment: { tile_radius_m: 100.0 }`): where `terrain_color` joins (Task 5).
- ion assets 1 (Cesium World Terrain) and 2 (Bing Maps Aerial) are spec §4's defaults.
- `OVERLUME_LIVE_ION` (R16) is a new variable, documented here and named by Task 14's skip
  message; the ion performance test's `OVERLUME_LIVE_ION_PERF`
  (`overlume/tests/test_environment_stream.cpp:1122`) stays separate.

- [ ] **Step 1: Link the new runbook from the pages that index it** (the failing check first)

In `docs/README.md`, replace

```markdown
- [`mapbox.md`](runbooks/mapbox.md) — the `mapbox://buildings` source: the
  `MAPBOX_TOKEN` contract and `tools/check_mapbox_token.sh`, the URI and the
  two GUI entries, attribution, Mapbox's terms as engineering rules, the demo
  at the operating area and Downtown Dubai, and troubleshooting.
- [`env_source_captures.md`](runbooks/env_source_captures.md) — one
  committed render per Environment Tiles preset (baked / osm / google) for
  judging the streamed sources side by side. The `mapbox` and
  `mapbox-original` sources have no committed capture: a render of live
  Mapbox tiles is Mapbox-derived data, which the terms forbid storing (see
  [`mapbox.md`](runbooks/mapbox.md), section 6).
```

with

```markdown
- [`mapbox.md`](runbooks/mapbox.md) — the `mapbox://` source: the
  `MAPBOX_TOKEN` contract and `tools/check_mapbox_token.sh`, the URI (the
  `buildings` and `terrain` layers, `imagery=satellite`) and the four GUI
  entries, attribution with the Maxar credit, request volume, Mapbox's terms
  as engineering rules, the demo at the operating area and Downtown Dubai,
  and troubleshooting.
- [`terrain_imagery.md`](runbooks/terrain_imagery.md) — terrain with imagery
  draped on it, for any streamed source: ion's `terrain=` and `imagery=`
  keys and the `osm-terrain` entry, the `environment_terrain` and
  `environment_imagery` node parameters, the `environment.terrain_color`
  theme token, overlay credits on screen and the renderer's overlay path.
- [`env_source_captures.md`](runbooks/env_source_captures.md) — one
  committed render per environment source (baked / osm / google) for
  judging the streamed sources side by side. The four Mapbox sources
  (`mapbox`, `mapbox-original`, `mapbox-satellite`,
  `mapbox-original-satellite`) have no committed capture: a render of live
  Mapbox tiles is Mapbox-derived data, which the terms forbid storing (see
  [`mapbox.md`](runbooks/mapbox.md), section 6).
```

and replace (under `## Plans`)

```markdown
- [`2026-10-14-mapbox-buildings.md`](plans/2026-10-14-mapbox-buildings.md)
  — the `mapbox://buildings` environment source in 18 tasks over eight
  parallel lanes (implements `design/2026-10-14-mapbox-buildings-design.md`; closed 2026-10-15).
```

with

```markdown
- [`2026-10-14-mapbox-buildings.md`](plans/2026-10-14-mapbox-buildings.md)
  — the `mapbox://buildings` environment source in 18 tasks over eight
  parallel lanes (implements `design/2026-10-14-mapbox-buildings-design.md`; closed 2026-10-15).
- [`2026-10-15-imagery-terrain.md`](plans/2026-10-15-imagery-terrain.md)
  — imagery on terrain (sub-project 2): terrain tilesets with raster
  overlays for `mapbox://` and `ion://` in 16 tasks over eight lanes
  (implements `design/2026-10-15-imagery-terrain-design.md`).
```

In `docs/status.md`, replace (the end of open item 22, its blank line and the next heading)

```markdown
If it may not ship, remove it from the tree and let deployments supply it through `environment_attribution_logo_path`.

## Known gaps / accepted exceptions
```

with

```markdown
If it may not ship, remove it from the tree and let deployments supply it through `environment_attribution_logo_path`.

23. **OPEN 2026-10-15 — imagery on terrain (sub-project 2) in progress.** Terrain under the streamed buildings with imagery draped on it, provider-agnostic: `mapbox://buildings,terrain?imagery=satellite` (Mapbox Satellite on Mapbox terrain-DEM, the buildings re-based onto the DEM) and `ion://96188?terrain=1&imagery=2` (Cesium World Terrain with Bing Maps Aerial through ion), selectable as `mapbox-satellite`, `mapbox-original-satellite` and `osm-terrain` ([runbook](runbooks/terrain_imagery.md), [Mapbox runbook](runbooks/mapbox.md)). Built on `mapbox-environment` under [`plans/2026-10-15-imagery-terrain.md`](plans/2026-10-15-imagery-terrain.md), tracked by the branch's open pull request. Closes when that plan's last task records the rig rehearsal at both sites and the Shipped row.

## Known gaps / accepted exceptions
```

In `docs/runbooks/mapbox.md`, replace

```markdown
# Mapbox buildings runbook

How to run the `mapbox://buildings` environment source: Mapbox's landmark models plus building
footprints from Mapbox Streets v8, extruded to their heights, streamed around the vehicle in
visual mode (mode 3). The design is
[`2026-10-14-mapbox-buildings-design.md`](../design/2026-10-14-mapbox-buildings-design.md).
```

with

```markdown
# Mapbox runbook

How to run the `mapbox://` environment source: Mapbox's landmark models plus building footprints
from Mapbox Streets v8, extruded to their heights, and with the `terrain` layer Mapbox's
terrain-DEM under them, with Mapbox Satellite draped on it, streamed around the vehicle in visual
mode (mode 3). The designs are
[`2026-10-14-mapbox-buildings-design.md`](../design/2026-10-14-mapbox-buildings-design.md) and
[`2026-10-15-imagery-terrain-design.md`](../design/2026-10-15-imagery-terrain-design.md). What
Mapbox terrain shares with `ion://` terrain (the node parameters, the theme token, the overlay
credits, the renderer's overlay path) is in [`terrain_imagery.md`](terrain_imagery.md).
```

- [ ] **Step 2: Run the link check and see it fail**

```bash
cd "$WORKTREE"
python3 tools/check_docs_links.py; echo "exit=$?"
```

Expected:

```text
Broken links (3):
  docs/README.md: -> runbooks/terrain_imagery.md
  docs/status.md: -> runbooks/terrain_imagery.md
  docs/runbooks/mapbox.md: -> terrain_imagery.md
exit=1
```

A line for `plans/2026-10-15-imagery-terrain.md` (from `docs/README.md` or `docs/status.md`)
means the plan's commit is missing from the lane's base: a deviation to report. This check fails
again if the runbook is later deleted or renamed.

- [ ] **Step 3: Write the terrain and imagery runbook**

Create `docs/runbooks/terrain_imagery.md`:

````markdown
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
````

- [ ] **Step 4: Bring the Mapbox runbook up to the layer list and satellite**

In `docs/runbooks/mapbox.md`, apply these replacements (each old block occurs once).

Replace

````markdown
It requests the two endpoints the source uses, at Downtown Dubai: the landmark tile z14
10707/7006 and the footprint tile z16 42830/28025 inside it. Like
`overlume/scripts/check_cesium_token.sh`, it prints one line. A working token prints:

```text
PASS: token retrieves the Downtown Dubai tiles (landmark tile HTTP 200, footprint tile HTTP 200)
```
````

with

````markdown
It requests one tile from each of the four endpoints the source uses, at Downtown Dubai: the
landmark tile z14 10707/7006, the footprint tile z16 42830/28025 inside it, the terrain-DEM tile
z14 10707/7006 and the satellite tile z18 171320/112100 inside the footprint tile (z18 is the
imagery cap, §3). Like `overlume/scripts/check_cesium_token.sh`, it prints one line. A working
token prints:

```text
PASS: token retrieves the Downtown Dubai tiles (landmark tile HTTP 200, footprint tile HTTP 200, DEM tile HTTP 200, satellite tile HTTP 200)
```
````

Replace

```markdown
- `HTTP 401` on both: Mapbox rejects the token. Check it, or make a new one.
- `HTTP 403`: the token is valid but not allowed here (URL restrictions or scopes). Use a token
  without URL restrictions.
```

with

```markdown
- `HTTP 401` on all four: Mapbox rejects the token. Check it, or make a new one.
- `HTTP 403`: the token is valid but not allowed here (URL restrictions or scopes). Use a token
  without URL restrictions.
- `DEM tile` or `satellite tile` failing while the landmark and footprint tiles answer 200: the
  token does not reach the Raster Tiles API. `mapbox://buildings` works; the `terrain` layer and
  `imagery=satellite` do not. Check the token's scopes and URL restrictions.
```

Replace

```markdown
`mapbox://buildings[?key=value(&key=value)*]`

| Key | Values | Default | Meaning |
|-----|--------|---------|---------|
| `materials` | `clay`, `original` | `clay` | `clay`: the theme's building material, like `osm`. `original`: Mapbox's landmark materials, and Mapbox Standard's building colour on extrusions, like `google`. Any other value logs one warning and means `clay`. |
| `radius` | finite > 0 (m) | 500 | Load and display radius around the vehicle. |
| `brightness` | finite > 0 | 1.0 | Scales base colours under `materials=original`. |
| `fallback` | a baked chunk directory | none | Shown after network loss (§8). |
| `cache` | `off` | `off` | `off` changes nothing. Any other value refuses the URI: Mapbox tiles are never written to disk. |

- `buildings` is the only layer. Any other path refuses the URI, and one log line names the
  supported layers.
- Apart from `materials`, a bad value for a known key refuses the whole URI. Unknown keys are
  ignored, and so are the ion-only keys `follow_terrain`, `ground_bias`, `replaces_ground`,
  `max_tilt_deg` and `max_cache_items`.
- The source never follows terrain and never replaces the clay ground plane. Buildings stand on
  the flat map plane at z = 0.
```

with

```markdown
`mapbox://<layers>[?key=value(&key=value)*]`

| Key | Values | Default | Meaning |
|-----|--------|---------|---------|
| layer list (the path) | `buildings`, `terrain`: a comma list, each at most once, in any order | none: at least one layer | `buildings`: landmark models and extruded footprints. `terrain`: Mapbox terrain-DEM tiles as the ground, with the buildings standing on them. An empty, repeated or unknown name refuses the URI, and one log line names the supported layers. |
| `materials` | `clay`, `original` | `clay` | `clay`: the theme's building material, like `osm`. `original`: Mapbox's landmark materials, and Mapbox Standard's building colour on extrusions, like `google`. Any other value logs one warning and means `clay`. |
| `radius` | finite > 0 (m) | 500 | Load and display radius around the vehicle, for the buildings and the terrain. |
| `brightness` | finite > 0 | 1.0 | Scales base colours under `materials=original`, and the satellite imagery. |
| `fallback` | a baked chunk directory | none | Shown after network loss (§8). |
| `cache` | `off` | `off` | `off` changes nothing. Any other value refuses the URI: Mapbox tiles are never written to disk. |
| `imagery` | `satellite`, `off` | `off` | `satellite`: Mapbox Satellite draped on the terrain, up to z18, credited "© Maxar" on screen (§5). Needs `terrain` in the layer list: without it the URI is refused (`mapbox:// imagery=satellite needs the 'terrain' layer`). |

- Apart from `materials`, a bad value for a known key refuses the whole URI. Unknown keys are
  ignored, and so are the ion-only keys `follow_terrain`, `ground_bias`, `replaces_ground`,
  `max_tilt_deg` and `max_cache_items`.
- Without `terrain`, the source never follows terrain and never replaces the clay ground plane:
  buildings stand on the flat map plane at z = 0, as before.
- With `terrain`, the source opens a second tileset of z14 terrain-DEM tiles within `radius`: 512
  × 512 px each, meshed with one vertex every 8 px (about 35 m apart in Dubai) and a 20 m skirt
  that hides cracks between tiles. The DEM height at the anchor becomes map z = 0; every extrusion
  stands at the DEM height of its footprint's centroid, and every landmark at the height of its
  anchor. The follow-terrain transform samples the terrain with the ion defaults fixed (follow
  on, `ground_bias` 0.3 m, `replaces_ground` on, `max_tilt_deg` 2.0; the URI takes none of those
  keys), and the terrain replaces the clay ground plane once the height under the ego is sampled
  ([`terrain_imagery.md`](terrain_imagery.md) §1).
- One DEM request per z14 tile serves both the terrain and the buildings' heights. At most 32
  decoded DEM tiles stay in memory; none is written to disk.
```

Replace

```markdown
- `brightness=<environment_brightness>` when it is not 1.0;
- never `cache=`, and never an ion-only key.

Each z14 tile (about 2.2 km wide in Dubai) costs 17 requests: one landmark model tile and 16
footprint tiles. With the 500 m radius, one to four tiles are live at a time. Footprint tiles are
billed per request, so the 200,000-a-month free tier covers about 12,500 z14 tiles, or about
11,700 if Mapbox also bills the landmark requests. The landmark endpoint has no published price.
```

with

```markdown
- `brightness=<environment_brightness>` when it is not 1.0;
- `terrain` added to the layer list when `environment_terrain` is `enable` and the list lacks
  it;
- `imagery=satellite` when `environment_imagery` is `satellite`, `imagery=off` when it is
  `disable`, and only when the URI has no `imagery=` key
  ([`terrain_imagery.md`](terrain_imagery.md) §3);
- never `cache=`, and never an ion-only key.

Each z14 tile (about 2.2 km wide in Dubai) costs 17 requests for its buildings: one landmark
model tile and 16 footprint tiles. With the 500 m radius, one to four tiles are live at a time.
Footprint tiles are billed per request, so the 200,000-a-month free tier covers about 12,500 z14
tiles, or about 11,700 if Mapbox also bills the landmark requests. The landmark endpoint has no
published price.

The `terrain` layer adds one DEM request per z14 tile. `imagery=satellite` adds the satellite
tiles cesium selects for each terrain tile on screen: at z18, the cap, 16 × 16 = 256 tiles of
about 140 m cover one z14 tile, and terrain away from the ego takes coarser, fewer ones (a few
dozen near the ego, design §12). DEM and satellite tiles are Raster Tiles API requests, billed
per request.
```

Replace

```markdown
## 4. The two GUI entries

The vcam GUI's Environment tiles list (`tools/vcam_gui.py`, through `tools/vcam_ws_bridge.py`)
holds the supported environment sources. Two of them are Mapbox:

| Entry | `environment_source_uri` | Look |
|-------|--------------------------|------|
| `mapbox` | `mapbox://buildings` | the theme's clay, like `osm` |
| `mapbox-original` | `mapbox://buildings?materials=original` | Mapbox's, like `google` |
```

with

```markdown
## 4. The GUI entries

The vcam GUI's Environment tiles list (`tools/vcam_gui.py`, through `tools/vcam_ws_bridge.py`)
holds the supported environment sources. Four of them are Mapbox:

| Entry | `environment_source_uri` | Look |
|-------|--------------------------|------|
| `mapbox` | `mapbox://buildings` | the theme's clay, like `osm` |
| `mapbox-original` | `mapbox://buildings?materials=original` | Mapbox's, like `google` |
| `mapbox-satellite` | `mapbox://buildings,terrain?imagery=satellite` | the theme's clay buildings standing on the DEM, satellite imagery on the ground |
| `mapbox-original-satellite` | `mapbox://buildings,terrain?materials=original&imagery=satellite` | Mapbox's buildings on the DEM, satellite imagery on the ground |

`osm-terrain`, the same terrain and imagery path on `ion://`, is in
[`terrain_imagery.md`](terrain_imagery.md) §2.
```

Replace

```markdown
- Neither entry carries `cache=off`: the source never caches.
```

with

```markdown
- No Mapbox entry carries `cache=off`: the source never caches.
```

Replace

```markdown
  524.76 px times the HUD scale) the text no longer fits on a line, and its end, "Improve this
  map", is cut off. At 1280 px that happens above HUD scale 2.4.
```

with

```markdown
  524.76 px times the HUD scale) the text no longer fits on a line, and its end, "Improve this
  map", is cut off. At 1280 px that happens above HUD scale 2.4.
- **With `imagery=satellite`:** the text is "© Mapbox © OpenStreetMap © Maxar Improve this map",
  as Mapbox's terms ask for satellite imagery. The node reads the credit from the library once a
  second ([`terrain_imagery.md`](terrain_imagery.md) §5), so "© Maxar" joins the line within a
  second of the source opening. The longer text (626.33 px at HUD scale 1.0) moves above the logo
  below 827 px of output width and is cut off below 643 px (16 px plus 626.33 px times the HUD
  scale); at 1280 px, above HUD scale 2.0.
```

Replace

```markdown
- **No caching.** The source has no disk cache, accepts only `cache=off`, and writes no file.
- **No Mapbox data in the tree.** Tests build their landmark and footprint fixtures in the test
  process, in Mapbox's formats. Responses fetched while debugging (tiles, TileJSON, response
```

with

```markdown
- **No caching.** The source has no disk cache, accepts only `cache=off`, and writes no file. That
  covers the DEM and satellite tiles: only cesium's in-memory caches hold them.
- **No Mapbox data in the tree.** Tests build their landmark, footprint, DEM and imagery fixtures
  in the test process, in Mapbox's formats (a DEM is a height function encoded in terrain-RGB, an
  imagery tile one flat colour). Responses fetched while debugging (tiles, TileJSON, response
```

Replace

```markdown
- `--arm mapbox-original` shows Mapbox's look. The GUI list switches between the two live.
```

with

```markdown
- `--arm mapbox-original` shows Mapbox's look. The GUI list switches between the entries live.
- `--arm mapbox-satellite` adds the terrain and the satellite imagery: imagery under the clay
  extrusions, "© Maxar" on the attribution line, and the buildings standing on the surface along
  the route.
```

Replace

```markdown
- `geo_datum_height_m` and `geo_anchor_height_offset_m` are optional. They do not move Mapbox
  buildings, which stand on map z = 0 at any anchor height.
```

with

```markdown
- `geo_datum_height_m` and `geo_anchor_height_offset_m` are optional. They do not move Mapbox
  buildings or terrain: without `terrain` the buildings stand on map z = 0, and with it map z = 0
  is the DEM height at the anchor, at any anchor height.
```

Replace

```markdown
For Mapbox's look, quote the URI:
`--param 'environment_source_uri:=mapbox://buildings?materials=original'`.
```

with

```markdown
For Mapbox's look, quote the URI:
`--param 'environment_source_uri:=mapbox://buildings?materials=original'`. For Mapbox's look on
satellite ground (the `mapbox-original-satellite` entry):
`--param 'environment_source_uri:=mapbox://buildings,terrain?materials=original&imagery=satellite'`.
```

Replace

```markdown
- The logo and the text are visible in mode 3 while the source streams, and gone in fallback.
- `mapbox-original` shows Mapbox's look, and `mapbox` the theme's.
```

with

```markdown
- The logo and the text are visible in mode 3 while the source streams, and gone in fallback.
- `mapbox-original` shows Mapbox's look, and `mapbox` the theme's.
- With `mapbox-satellite` and `mapbox-original-satellite`: satellite imagery on the ground, no
  sky between terrain tiles, the buildings on the surface along the whole route, and "© Maxar" on
  the line. `mapbox-satellite`'s steady-state `render_ms` at Downtown stays within 1.5 times the
  `osm` source's (design §11).
```

Replace

```markdown
- **The log says `Mapbox rejected MAPBOX_TOKEN (HTTP 401)` and no buildings appear.** Mapbox
  rejects the token. Run `tools/check_mapbox_token.sh` and replace the token. The 401s count as
  failed requests, so eight in a row switch the source to its fallback (below).
```

with

```markdown
- **The log says `Mapbox rejected MAPBOX_TOKEN (HTTP 401)` and no buildings appear.** Mapbox
  rejects the token. The line comes once per endpoint (landmark, footprint, DEM); refused
  satellite tiles show as cesium-native's own lines instead (below). Run
  `tools/check_mapbox_token.sh` and replace the token. The 401s count as failed requests, so eight
  in a row switch the source to its fallback (below).
- **The source does not open: `mapbox:// imagery=satellite needs the 'terrain' layer`.** Add
  `terrain` to the layer list (`mapbox://buildings,terrain?imagery=satellite`), or set
  `environment_terrain: enable` ([`terrain_imagery.md`](terrain_imagery.md) §3).
- **The source does not open: `mapbox:// supports only the layers 'buildings' and 'terrain', not
  '…'`.** A misspelt, repeated or empty layer name.
- **Terrain, but no imagery: the ground shows `environment.terrain_color`.** Satellite tiles
  fail (429, 5xx or a transport error). The source logs no line of its own for them:
  cesium-native logs each failed tile, `Received response code <code> for image <url>`, through
  the source's redacting logger, so the URL shows `access_token=<redacted>`. Failed tiles count
  toward the network-loss fallback. A refused token (401 or 403 on the satellite tiles) cascades
  through cesium's parent tiles to more than eight failures, so the whole source falls back
  (below). Run `tools/check_mapbox_token.sh` (§2).
- **Sky through the ground, and one log line `mapbox:// terrain tile 14/…/… skipped: …`.**
  That DEM tile could not be decoded (wrong size or channels): it has no terrain, and each kind
  of failure is logged once. A DEM tile that answers 204 or 404 leaves a hole too, without a line;
  the DEM is global, so that is rare.
- **A small step in the ground along a straight north-south or east-west line.** Neighbouring DEM
  tiles disagree by a quantisation step at their shared edge; the 20 m skirt hides the crack, not
  the step. Known limit (design §12).
```

Replace

```markdown
  prints tile, landmark and extrusion counts, and report what it prints:
  `OVERLUME_LIVE_MAPBOX=1 overlume/build/test_mapbox_buildings --gtest_filter='MapboxLive.*'`.
```

with

```markdown
  prints tile, landmark and extrusion counts, and report what it prints:
  `OVERLUME_LIVE_MAPBOX=1 overlume/build/test_mapbox_buildings --gtest_filter='MapboxLive.*'`.
  The terrain and the satellite imagery have their own opt-in live test,
  `MapboxLive.OperatingAreaTerrain` in `test_mapbox_terrain`
  ([`terrain_imagery.md`](terrain_imagery.md) §7).
```

Replace

```markdown
- **The buildings, the logo and the text disappear, and the node warns
  `environment source: network loss detected -- switched to fallback dir '…'`** (or
  `... but none configured -- environment now empty`). Eight requests in a row failed: network
  loss, a Mapbox-side 429/5xx burst, or 401 on everything. The state is `STREAMING_FALLBACK`, and
  the baked chunks from `fallback=` (the node's `environment_chunks_dir`) show instead. The
```

with

```markdown
- **The buildings, the terrain, the logo and the text disappear, and the node warns
  `environment source: network loss detected -- switched to fallback dir '…'`** (or
  `... but none configured -- environment now empty`). Eight requests in a row failed: network
  loss, a Mapbox-side 429/5xx burst, or 401 on everything. The state is `STREAMING_FALLBACK`: both
  tilesets are gone, the clay plane is back, and the baked chunks from `fallback=` (the node's
  `environment_chunks_dir`) show instead. The
```

Replace

```markdown
- **Buildings float or sink on a slope.** The source has no terrain following: buildings stand on
  the flat map plane. Known limit until imagery on terrain lands (design §13).
```

with

```markdown
- **Buildings float or sink on a slope.** Without the `terrain` layer the source has no terrain:
  buildings stand on the flat map plane. Add `terrain`, or pick `mapbox-satellite`. With
  `terrain`, an extrusion stands at the DEM height of its footprint's centroid, so a long building
  on a slope floats at its downhill end. Known limit (imagery-terrain design §12); Mapbox GL does
  the same.
```

- [ ] **Step 5: Run the link check and see it pass**

```bash
cd "$WORKTREE"
python3 tools/check_docs_links.py; echo "exit=$?"
```

Expected: `OK: no broken links` and `exit=0`.

- [ ] **Step 6: CHANGELOG**

In `CHANGELOG.md`, replace

```markdown
checks the token. See `docs/runbooks/mapbox.md`.

### Fixed
```

with

```markdown
checks the token. See `docs/runbooks/mapbox.md`.
- Terrain with imagery draped on it, as a second tileset per streamed source, provider-agnostic. `mapbox://` takes a layer list (`buildings`, `terrain`) and `imagery=satellite`: Mapbox terrain-DEM tiles become the ground, the buildings stand on the DEM, and Mapbox Satellite (up to z18, credited "© Maxar") is draped on it. `ion://` takes `terrain=<asset>` and `imagery=<asset>`, for example Cesium World Terrain (`1`) with Bing Maps Aerial (`2`). The node parameters `environment_terrain` and `environment_imagery` add them when the URI lacks them; the attribution line carries the overlays' credits (`environment_source_credits()`, a new free function in `<overlume/scene.h>`; `kSceneVersion` unchanged); the theme token `environment.terrain_color` colours terrain without imagery. The vcam GUI lists three more environment sources, `mapbox-satellite`, `mapbox-original-satellite` and `osm-terrain`, and `tools/check_mapbox_token.sh` also checks a DEM and a satellite tile. See `docs/runbooks/terrain_imagery.md` and `docs/runbooks/mapbox.md`.

### Fixed
```

- [ ] **Step 7: Run the doc checks**

```bash
cd "$WORKTREE"
python3 tools/check_docs_links.py; echo "exit=$?"
grep -nE '\b(pk|sk|tk)\.[A-Za-z0-9_-]{20,}' docs/runbooks/terrain_imagery.md \
    docs/runbooks/mapbox.md docs/README.md docs/status.md CHANGELOG.md; echo "grep=$?"
grep -ci 'preset' docs/runbooks/terrain_imagery.md
grep -c 'imagery=satellite' docs/runbooks/mapbox.md
```

Expected: `OK: no broken links`, `exit=0`; no token-shaped match, `grep=1`; `1` (the bridge
command's wire field `"preset":"osm-terrain"`, the only "preset" on the page: the GUI entries are
environment sources); `10` (the lines that name `imagery=satellite`: one in §2, the new preflight
bullet; three in §3, the table row, the composer bullet and the request-volume paragraph; two in
§4; one in §5; one in §7; two in §8).

- [ ] **Step 8: Format**

```bash
cd "$WORKTREE"
git add docs/runbooks/terrain_imagery.md docs/runbooks/mapbox.md docs/README.md docs/status.md \
    CHANGELOG.md
tools/check_format.sh --fix
git status --short
```

Expected: `check_format.sh: reformatted … files with clang-format 23.1.1` (no C/C++ change in
this task, so nothing moves), and exactly the five staged files:

```text
M  CHANGELOG.md
M  docs/README.md
M  docs/runbooks/mapbox.md
A  docs/runbooks/terrain_imagery.md
M  docs/status.md
```

Any other line is a deviation to report.

- [ ] **Step 9: Gate**

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
ROS_DOMAIN_ID=84 CI_VISUAL_MODE_DOMAIN_ID=84 \
    flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock tools/ci_visual_mode.sh
```

Run in the foreground (Bash timeout 600000 ms). Exit 75 means another lane held the lock for
300 s: run the same command again. Expected: six `PASS` lines and `OVERALL: PASS`, exit 0, with
Task 7's counts (442 ctest entries, 424 ok / 8 skipped; 395 node tests; WS bridge `100 passed`;
25 goldens all `ok`; 6 examples). This task changes no code; the gate proves the lane is still
green on top of Task 7.

- [ ] **Step 10: Commit**

```bash
cd "$WORKTREE"
git add docs/runbooks/terrain_imagery.md docs/runbooks/mapbox.md docs/README.md docs/status.md \
    CHANGELOG.md
git commit -F - <<'EOF'
docs(terrain): terrain and imagery runbook; Mapbox runbook gains terrain and satellite

docs/runbooks/terrain_imagery.md is the operator's page for terrain with
imagery draped on it, for any streamed source: what a terrain tileset
changes (ground, terrain following, colour, fallback), ion's terrain= and
imagery= keys and the osm-terrain entry, the environment_terrain and
environment_imagery node parameters (string values; a value the
configured source ignores is warned at configure), the
environment.terrain_color theme token, the overlay credits on screen,
the renderer's overlay path, the offline and opt-in checks, and
troubleshooting.

docs/runbooks/mapbox.md gains the layer list and imagery=satellite in
its URI table, the DEM and its request volume, the two satellite GUI
entries, the Maxar credit and its width limits, the preflight's DEM and
satellite probes, the demo on satellite ground, and failure rows for the
terrain and the imagery. The docs index links the new runbook and
lists the plan, docs/status.md opens item 23 for the plan, and
CHANGELOG [Unreleased] gains the entry.

Claude-Session: <the executing session's URL>
EOF
```

### Task 9: DEM cache and buildings re-based onto the DEM

**Lane:** 5. **Depends on:** Tasks 1 and 2 (lane 1) and Tasks 3 and 4 (lane 2) merged into
`mapbox-environment`; the lane forks from that head.

**Files:**
- Create: `overlume/src/mapbox_dem_cache.hpp` (the contract's `DemCache`, plus `kDemUrlTemplate`,
  `kDemCacheTiles`, `fill_tile_template`, `dem_pixel_of`, `Rebase` and `rebase_of`, which Task 10
  reuses)
- Create: `overlume/src/mapbox_dem_cache.cpp` (also defines the hook `mapbox_dem_url`, and emits
  head.md's `mapbox:// terrain tile 14/{}/{} skipped: {}` and the DEM's
  `Mapbox rejected MAPBOX_TOKEN (HTTP {})`: the cache decodes, ruling R20)
- Modify: `overlume/src/mapbox_test_hooks.hpp` (`mapbox_footprint_url`'s declaration, 52;
  `MapboxSourceStats`, after `max_tile_top_m`, 71-73)
- Modify: `overlume/src/mapbox_loader.cpp` as Tasks 3 and 4 left it (line numbers after Task 4,
  whose `parse_mapbox_spec` edits add 23 lines above `LoaderState`: includes 6, 21, 25 and 35;
  `LoaderState`, 192; before `build_tile`, 346; `build_tile`'s retry, 356, landmarks, 381-385,
  extrusions, 400-420, and region, 440-443; the loader's constructor, 450-452, continuation,
  479-485, and members, 495-496; `open_mapbox_source`, 533-536; `mapbox_last_source_stats`, 630)
- Test: `overlume/tests/test_mapbox_buildings.cpp` (includes, 9 and 22;
  `AnchorTileLoadsWithStandardFilterApplied`, 457-458; nine tests after
  `LandmarksAloneStillBuildTheTile`, which ends at 915)
- No change to `overlume/src/mapbox_loader.hpp` (Task 4's only, ruling R19).

**Interfaces:**
- Consumes:

```cpp
// overlume/src/mapbox_dem.hpp (Task 1)
inline constexpr uint32_t kDemSize = 512;
struct DemTile { bool ok = false; std::string error; std::vector<float> height_m; /* at() */ };
DemTile decode_dem_png(const uint8_t* bytes, size_t size);
double dem_sample(const DemTile& dem, double u, double v);
// overlume/src/mapbox_tiling.hpp (Task 3, lane-2.md Step 2)
inline constexpr uint32_t kMaxAttempts = 3;
inline constexpr double kRetryWaitS = 2.0;
inline constexpr double kRegionBelowM = 10.0;
struct SourceLog { bool first_time(const std::string& key); };  // one line per key per source
struct Fetched { uint16_t status = 0; std::vector<std::byte> body; bool transport_error = false; };
enum class Outcome { Body, Empty, Auth, Transient, Other };
Outcome classify(const Fetched& f);
CesiumAsync::Future<Fetched> fetch(const CesiumAsync::AsyncSystem& asyncSystem,
                                   const std::shared_ptr<CesiumAsync::IAssetAccessor>& accessor,
                                   const std::string& url);  // never rejects
void note_status(SourceLog& seen, spdlog::logger& log, const char* endpoint, const Fetched& f,
                 const TileId& t14);
Cesium3DTilesSelection::TileLoadResult success(/* content, bounds, accessor, ellipsoid */);
Cesium3DTilesSelection::BoundingVolume tile_region(const TileId& t, double min_height_m,
                                                   double max_height_m,
                                                   const CesiumGeospatial::Ellipsoid& ellipsoid =
                                                       CesiumGeospatial::Ellipsoid::WGS84);
// overlume/src/mapbox_loader.hpp (Task 4)
struct MapboxSpec { bool buildings = true; bool terrain = false; bool imagery_satellite = false; /* ... */ };
// overlume/src/mapbox_tile_frame.hpp (sub-project 1)
TileId tile_containing(double lon_deg, double lat_deg, uint32_t z);
LonLatDeg tile_point_to_lonlat(const TileId& t, double u, double v, double extent);
// overlume/tests/mapbox_dem_fixture.hpp (Task 2)
std::vector<uint8_t> encode_dem_png(const std::function<double(double u, double v)>& height_m);
double hill_height_m(double u, double v);  // 40 m gaussian at the tile centre, sigma 400 px
```

- Produces, verbatim from the contract: `overlume/src/mapbox_dem_cache.hpp`'s `DemCache`
  (constructor, `get`, `reference_z`, `requests`), `MapboxSourceStats::dem_requests` and
  `std::string mapbox_dem_url(uint32_t x14, uint32_t y14);` (no query string). Beyond the
  contract, additive and in the new header only: `kDemUrlTemplate` (the pinned DEM template),
  `kDemCacheTiles` (32), `std::string fill_tile_template(std::string url_template, const TileId& t);`,
  `std::array<double, 2> dem_pixel_of(const LonLatDeg& p, const TileId& t14);`,
  `bool DemCache::failing(const TileId& t14) const;`, `struct Rebase` and
  `CesiumAsync::SharedFuture<Rebase> rebase_of(const std::shared_ptr<DemCache>& dem, const TileId& t14, const LonLatDeg& anchor);`.
  Behaviour: with `terrain` in the layer list, buildings stand on the DEM; without it nothing
  changes.

**How the cache and the re-basing work** (spec section 5, "DemCache"; read with the code in Steps
2 and 5):
- **One request per tile.** `get(t14)` returns the tile's `SharedFuture` when the cache holds one
  (moving it to the front of the LRU), else requests
  `https://api.mapbox.com/v4/mapbox.mapbox-terrain-dem-v1/14/{x}/{y}@2x.pngraw?access_token=…`
  through Task 3's `fetch` on the source's cache-off counting accessor and decodes the answer on a
  worker. 32 entries; the 33rd evicts the least recently used. 204 and 404 resolve to `nullptr`
  silently; a malformed PNG, a 401/403 and any other status resolve to `nullptr` with one line per
  status (or decode error) per source, naming the tile, never the URL (Task 3's `note_status` for
  the statuses it covers). The future never rejects (cesium fails a tile for good on a rejected
  load).
- **Failures for now.** A 429, 5xx or transport failure also records the steady-clock time it
  happened, and `failing(t14)` reports it. For `kRetryWaitS / 2` (1 s) every caller shares the
  failure, which covers the callers of one load attempt (the buildings tile, the anchor's
  reference, and from Task 10 the terrain tile); after that, `get()` drops the entry and asks
  again. The loaders' next attempt starts at least `kRetryWaitS` (2 s) later, so it always asks
  again.
- **The reference.** `reference_z(anchor)` samples the DEM of the anchor's z14 tile at the
  anchor's pixel (`dem_pixel_of`, `tile_containing`'s Web Mercator without the floor), memoised;
  while it has no value (the anchor's DEM missing) each call derives it again through `get()`, which
  answers a 404 from memory and a failure for now from the network once the 1 s is over.
- **The rebase.** `rebase_of(cache, t14, anchor)` combines the tile's DEM and the reference into a
  `SharedFuture<Rebase>`. `Rebase::active()` needs both; `Rebase::retry` says one of them is
  missing because its request failed for now (`failing()`, read once the futures resolve), which
  is worth another attempt. Task 10's terrain loader builds from the same `Rebase`.
- **The buildings.** With the cache, `loadTileContent` still sends its 17 requests first, then
  asks for the rebase; the build waits for both on the worker, as it waited for the answers alone.
  A rebase with `retry` sends the tile back exactly as a transient square does (`RetryLater` and
  `retries` + 1 while `attempt < kMaxAttempts`), so a DEM failing at start-up never pins a tile's
  buildings at z = 0 while the terrain recovers. An extruded footprint's base and roof rise by
  `dem_sample` at its unclipped centroid (z16 square units → DEM pixels of the z14 tile) minus the
  reference; a landmark node rises by the same at its anchor (the node matrix's translation,
  frame → ECEF → longitude and latitude → DEM pixel), along its matrix's up column. The region's
  top and floor follow. Without the terrain layer the `Rebase` is an empty, already-resolved one:
  the same build, the same geometry, no DEM request (the existing tests and the goldens pin it).

**API notes** (cesium-native 0.64, paths under `overlume/build/_deps/cesium-native-src/`; every
code block in this task was compiled with `-fsyntax-only -Wall -Wextra` against
`overlume/build/CMakeFiles/overlume_stream.dir/flags.make`, the tests against
`test_mapbox_buildings.dir/flags.make`, over lane 2's post-Task-3/4 sources and lane 1's
`mapbox_dem.hpp` and fixtures, and is a clang-format 23.1.1 fixed point):
- `CesiumAsync/include/CesiumAsync/AsyncSystem.h:291-310`: the variadic `all()` builds a tuple of
  `async::task`, so it cannot mix a `Future` with a `SharedFuture` (a probe doing so fails to
  compile in `async++/traits.h:133`). The build therefore chains: `SharedFuture::thenImmediately`
  (`SharedFuture.h:100-110`) unwraps a `Future` its continuation returns (`:94-95`,
  `Impl/ContinuationFutureType.h:14-17`) and passes the value as `const T&`
  (`Impl/WithTracing.h:48-75`); `Future::share()` (`Future.h:287`); `SharedFuture::isReady()`
  (`SharedFuture.h:283`) and `wait()` (`:226-238`, non-blocking once ready). In
  `get(t).thenImmediately(f(reference_z(…)))` the call on `get(t)` is sequenced before its
  argument (C++17), so the reference asks after the tile and shares its entry.
- `overlume/src/mapbox_tiling.{hpp,cpp}` (Task 3, lane-2.md Steps 2-3): `fetch` maps every
  answer, a rejection included, to a `Fetched`; `classify` puts 204/404 in `Empty`, 401/403 in
  `Auth`, 429/5xx/transport in `Transient`; `note_status` logs `Auth` and `Other` once per
  endpoint and status and stays silent on `Body` and `Empty`; its `Transient` text ("still failing
  after 3 attempts") does not fit a cache that logs on the first failure, so `read_dem` logs that
  case itself under the same key.
- `overlume/src/mapbox_loader.cpp` as Tasks 3 and 4 left it (lane-2.md Task 3 Step 4, then Task 4
  Step 4's `parse_mapbox_spec` edits, +23 lines above `LoaderState`): `LoaderState`, 180-210
  (inherits `SourceLog`); `build_tile`, 347-444 (the transient retry, 353-359; landmarks read
  first; `top`, 385; the extrusion loop, conflating by `footprint_centroid` and extruding
  `base`..`*b.height`; the region, 440-443); `loadTileContent`, 454-486, on the main thread,
  building on a worker at 479-485; `open_mapbox_source`, 502-542, with the cache-off externals;
  `mapbox_last_source_stats`, 615-632. Task 3's Edit 1 drops `<glm/gtc/type_ptr.hpp>`, which
  `frame_to_dem_px`'s `glm::make_mat4` needs, so Step 5 adds it back.
- `overlume/src/environment_stream.cpp:386-391`: `build_externals(…, "off", …)` hands the tileset
  the `CountingAssetAccessor` alone, no cache database, so the DEM requests through
  `externals.pAssetAccessor` count toward network loss (`environment_stream.hpp:151-159`: a 2xx
  resets the run, anything else adds one) and never reach the disk. `:301-345`: the redacting
  logger's sink is a `base_sink<std::mutex>`, safe from the worker that logs.
- `overlume/src/mapbox_tile_frame.cpp:52-62` `tile_containing` (the Web Mercator `dem_pixel_of`
  repeats without the floor); `:92-99` `project_to_frame` puts a point at `origin_height_m + z_m`
  on the ellipsoid, so a tile-frame z is metres above the anchor's ground (less the curvature, 1 mm
  at 110 m); `:106-144` `landmark_node_to_frame`: column 2 of the frame-mapped matrix
  (`a[8..10]`) is the local up per metre (`m[10]` is 1, `:116`) and column 3 (`a[12..14]`) the
  node's anchor at z = 0 (`:131`, `:140`).
- `overlume/src/mapbox_extrude.hpp:88-96`: `extrude_polygons` builds walls from `min_height_m`
  to `height_m` and no floor, requiring `height_m > min_height_m`; `:98-100`
  `footprint_centroid` in tile units, before clipping. `overlume/src/mapbox_landmarks.hpp:21-31`:
  the landmark model holds the kept building nodes only, in scene 0, matrices frame-mapped.
- `CesiumGltf::Node::matrix` is a `std::vector<double>` (probe `static_assert`), and the writer
  omits an identity matrix (`CesiumGltfWriter/generated/src/ModelJsonWriter.cpp:2497-2502`), so in
  a captured GLB only a landmark node writes `"matrix"`.
- `overlume/src/environment_stream.cpp:702-736`: the GLB is written after
  `strip_attributes_and_correct_heights` (235-279), which rewrites POSITION data but never the
  accessor's `min`/`max`, so a captured buildings GLB's POSITION bounds are the loader's own
  tile-frame metres; `ensure_flat_normals` returns a GLB whose primitives have normals unchanged
  (`overlume/src/gltf_normals.cpp:323-334`).
- `overlume/src/environment_stream.cpp:560-593`: the fixture accessor answers inside
  `createFuture`, on the calling thread, records every URL with its query, and a second
  `serve` of a URL replaces the first answer (the existing `expect_retry_then_load` relies on it).
- Tests (`test_mapbox_buildings.cpp` at 427f5d8, unchanged by Tasks 3 and 4): `kAttempt2AtS` and
  `kAttempt3AtS` 43-44, `serve_square` 108-112, `serve_anchor_tile` 133-138 (square i's box is
  `10 + i` m tall, so the highest is 25 m), `FrozenRetryClock` 273-280, `downtown_tile` 322,
  `landmark_at` 325-332, `kOverSquare10` (4696 z14 units) 337, `glb_json` 342-348,
  `serve_downtown` 352-360. The operating area's z14 tile is 2215.6 m across
  (2π · 6378137 · cos 25.08° / 16384), 4.33 m a DEM pixel; an anchor at (0.25, 0.25) of the tile is
  554 m from its west and north edges, so the 500 m radius reaches that tile alone. The contract's
  hill (40 m, sigma 400 px) is 36.1 m at the anchor's pixel (128, 128) and 40.0 m at the block's
  (262.25, 262.25): a 3.9 m lift. A 5 % slope is `0.05 · 4.33` = 0.216 m a pixel.

- [ ] **Step 1: Create the lane worktree** (the orchestrator, once Tasks 1-4 are merged)

```bash
git -C "$INTEGRATION" worktree add "$INTEGRATION-l5" -b terrain-env/l5 mapbox-environment
cd "$INTEGRATION-l5"
ls overlume/src/mapbox_dem.hpp overlume/src/mapbox_tiling.hpp overlume/tests/mapbox_dem_fixture.hpp
grep -c 'bool terrain = false;' overlume/src/mapbox_loader.hpp
grep -c 'bool waiting(const TileId& t14, double now_s) const;\|^std::unique_ptr<Cesium3DTilesSelection::Tile> quadtree_root(' \
    overlume/src/mapbox_tiling.hpp
mkdir -p /tmp/overlume-l5
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock \
    cmake --toolchain "$PWD/overlume/cmake/toolchain-clang-libcxx.cmake" -S overlume \
    -B overlume/build -DOVERLUME_ENABLE_CESIUM=ON -DCMAKE_C_COMPILER_LAUNCHER=ccache \
    -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DOVERLUME_TEST_TMP_DIR_RUNTIME=/tmp/overlume-l5
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j16
ros/colcon_build.sh
```

Expected: `ls` lists the three files, the first `grep` prints `1`, the second `2`, the configure
ends `-- Build files have been written to: …/overlume/build`, and the build and `colcon build`
finish without errors. Exit 75 from the configure means the lock wait ran out: run it again. From
here on `$WORKTREE` is `$INTEGRATION-l5`. The Replace blocks below quote
`overlume/src/mapbox_loader.cpp` as Task 3 leaves it (lane-2.md Step 4: `region_of` became
`tile_region`, `make_root` became `quadtree_root`, the attempt bookkeeping became
`AttemptBook::waiting` and `begin`), and the other files as at 427f5d8. An anchor that is not in
the file exactly once is a plan deviation: report it.

- [ ] **Step 2: The DEM cache and the stats hooks**

Create `overlume/src/mapbox_dem_cache.hpp`:

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

static_assert(__cplusplus >= 202002L, "mapbox_dem_cache.hpp is C++20-only (overlume_stream)");

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <list>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include <CesiumAsync/AsyncSystem.h>
#include <CesiumAsync/IAssetAccessor.h>
#include <CesiumAsync/SharedFuture.h>

#include "mapbox_dem.hpp"
#include "mapbox_tile_frame.hpp"
#include "mapbox_tiling.hpp"

namespace spdlog {
class logger;
}

namespace overlume::mapbox {

// Mapbox terrain-DEM: a 512 px terrain-RGB PNG per z14 tile. No query string.
inline constexpr char kDemUrlTemplate[] =
    "https://api.mapbox.com/v4/mapbox.mapbox-terrain-dem-v1/{z}/{x}/{y}@2x.pngraw";
inline constexpr size_t kDemCacheTiles = 32;  // about 1 MB of heights each

// `url_template` with `{z}`, `{x}` and `{y}` (XYZ: row 0 at the north edge) taken from t.
std::string fill_tile_template(std::string url_template, const TileId& t);

// Where a longitude and latitude fall in z14 tile t14's DEM: pixel coordinates u east, v south,
// 0..kDemSize across the tile (outside that range for a point outside the tile), as dem_sample
// takes them.
std::array<double, 2> dem_pixel_of(const LonLatDeg& p, const TileId& t14);

// The terrain-DEM tiles of one Mapbox source, shared by its buildings and terrain loaders.
class DemCache {
public:
    DemCache(std::shared_ptr<CesiumAsync::IAssetAccessor> accessor, CesiumAsync::AsyncSystem async,
             std::string dem_url_template_with_query,
             std::shared_ptr<spdlog::logger> redacting_logger);
    // One request per z14 tile, shared by every caller; LRU of 32 decoded tiles. The future
    // resolves to nullptr on 204/404 (empty) and on any failure (one log line per status, like the
    // loader). A failure for now (429, 5xx, transport) is shared for kRetryWaitS / 2 of steady
    // time, which covers every caller of one load attempt; a caller after that, a later attempt,
    // asks again.
    CesiumAsync::SharedFuture<std::shared_ptr<const DemTile>> get(const TileId& t14);
    // The DEM height at the anchor, memoised: the source's first request. While it has no value
    // (the anchor's DEM missing), each call derives it again through get().
    CesiumAsync::SharedFuture<std::optional<double>> reference_z(const LonLatDeg& anchor);
    // True when the tile's cached request failed for now; read once its future has resolved.
    bool failing(const TileId& t14) const;
    uint64_t requests() const;  // for stats

private:
    struct Entry {
        CesiumAsync::SharedFuture<std::shared_ptr<const DemTile>> dem;
        // Steady-clock seconds at which the request failed for now; negative otherwise.
        std::shared_ptr<std::atomic<double>> failed_at_s;
        std::list<uint64_t>::iterator recent;  // its place in recent_
    };

    std::shared_ptr<CesiumAsync::IAssetAccessor> accessor_;
    CesiumAsync::AsyncSystem async_;
    std::string urlTemplate_;  // carries the token: never logged
    std::shared_ptr<spdlog::logger> log_;
    std::shared_ptr<SourceLog> seen_ = std::make_shared<SourceLog>();
    mutable std::mutex mutex_;
    std::unordered_map<uint64_t, Entry> entries_;
    std::list<uint64_t> recent_;  // keys, most recently used first
    std::optional<CesiumAsync::SharedFuture<std::optional<double>>> reference_;
    std::atomic<uint64_t> requests_{0};
};

// A z14 tile's DEM and the anchor's height on its own DEM. With both, a point lifted by
// lift(u, v) stands on the terrain, which passes through z = 0 at the anchor.
struct Rebase {
    std::shared_ptr<const DemTile> dem;
    std::optional<double> z_ref_m;
    // One of them is missing because its request failed for now: worth another attempt.
    bool retry = false;
    bool active() const { return dem != nullptr && z_ref_m.has_value(); }
    // Metres to add at DEM pixel (u, v); active() only.
    double lift(double u, double v) const { return dem_sample(*dem, u, v) - *z_ref_m; }
};

// Main thread: the tile's DEM and the anchor's height, one request when the anchor lies in t14 and
// one per tile for both Mapbox loaders.
CesiumAsync::SharedFuture<Rebase> rebase_of(const std::shared_ptr<DemCache>& dem, const TileId& t14,
                                            const LonLatDeg& anchor);

}
```

Create `overlume/src/mapbox_dem_cache.cpp`:

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "mapbox_dem_cache.hpp"

#include "mapbox_test_hooks.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numbers>
#include <utility>

namespace overlume::mapbox {

namespace {

constexpr char kDemEndpoint[] = "terrain-DEM";

uint64_t key_of(const TileId& t14) { return (uint64_t{t14.x} << 32) | t14.y; }

double steady_s() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

// Worker thread: one DEM answer. Empty answers (204, 404) are normal and silent; any other answer
// without heights logs once per status, or per decode error, per source, naming the tile, never
// the URL. A failure for now also records when it happened.
std::shared_ptr<const DemTile> read_dem(const Fetched& f, spdlog::logger& log, SourceLog& seen,
                                        std::atomic<double>& failed_at_s, const TileId& t14) {
    const Outcome outcome = classify(f);
    if (outcome == Outcome::Body) {
        auto dem = std::make_shared<DemTile>(
            decode_dem_png(reinterpret_cast<const uint8_t*>(f.body.data()), f.body.size()));
        if (dem->ok) return dem;
        if (seen.first_time(std::string(kDemEndpoint) + " decode " + dem->error)) {
            log.warn("mapbox:// terrain tile 14/{}/{} skipped: {}", t14.x, t14.y, dem->error);
        }
        return nullptr;
    }
    if (outcome != Outcome::Transient) {
        note_status(seen, log, kDemEndpoint, f, t14);  // 401/403 and other statuses, once each
        return nullptr;
    }
    failed_at_s.store(steady_s());
    if (seen.first_time(std::string(kDemEndpoint) + " " + std::to_string(f.status))) {
        const std::string what =
            f.transport_error ? std::string("transport error") : "HTTP " + std::to_string(f.status);
        log.warn("mapbox: {} failing ({}) at tile 14/{}/{}; asked again on the tile's next attempt",
                 kDemEndpoint, what, t14.x, t14.y);
    }
    return nullptr;
}

}

std::string fill_tile_template(std::string url_template, const TileId& t) {
    const auto put = [&url_template](const std::string& key, uint32_t value) {
        const size_t at = url_template.find(key);
        if (at != std::string::npos) url_template.replace(at, key.size(), std::to_string(value));
    };
    put("{z}", t.z);
    put("{x}", t.x);
    put("{y}", t.y);
    return url_template;
}

std::array<double, 2> dem_pixel_of(const LonLatDeg& p, const TileId& t14) {
    // tile_containing's spherical Web Mercator, without the floor.
    constexpr double kMaxLatDeg = 85.0511287798;
    const double lat = std::clamp(p.lat, -kMaxLatDeg, kMaxLatDeg) * std::numbers::pi / 180.0;
    const double n = std::ldexp(1.0, static_cast<int>(t14.z));
    const double fx = (p.lon + 180.0) / 360.0 * n - t14.x;
    const double fy = (1.0 - std::asinh(std::tan(lat)) / std::numbers::pi) / 2.0 * n - t14.y;
    return {fx * kDemSize, fy * kDemSize};
}

DemCache::DemCache(std::shared_ptr<CesiumAsync::IAssetAccessor> accessor,
                   CesiumAsync::AsyncSystem async, std::string dem_url_template_with_query,
                   std::shared_ptr<spdlog::logger> redacting_logger)
    : accessor_(std::move(accessor)),
      async_(std::move(async)),
      urlTemplate_(std::move(dem_url_template_with_query)),
      log_(std::move(redacting_logger)) {}

CesiumAsync::SharedFuture<std::shared_ptr<const DemTile>> DemCache::get(const TileId& t14) {
    const uint64_t key = key_of(t14);
    std::lock_guard<std::mutex> lock(mutex_);
    if (const auto it = entries_.find(key); it != entries_.end()) {
        const double failedAt = it->second.failed_at_s->load();
        if (failedAt < 0.0 || steady_s() - failedAt < kRetryWaitS / 2.0) {
            recent_.splice(recent_.begin(), recent_, it->second.recent);
            return it->second.dem;
        }
        recent_.erase(it->second.recent);  // failed for now, a while ago: ask again
        entries_.erase(it);
    }
    requests_.fetch_add(1);
    auto failedAt = std::make_shared<std::atomic<double>>(-1.0);
    // fetch never rejects: the cesium callers would fail their tile for good.
    CesiumAsync::SharedFuture<std::shared_ptr<const DemTile>> dem =
        fetch(async_, accessor_, fill_tile_template(urlTemplate_, t14))
            .thenInWorkerThread([log = log_, seen = seen_, failedAt, t14](Fetched&& f) {
                return read_dem(f, *log, *seen, *failedAt, t14);
            })
            .share();
    recent_.push_front(key);
    entries_.emplace(key, Entry{dem, std::move(failedAt), recent_.begin()});
    if (entries_.size() > kDemCacheTiles) {
        entries_.erase(recent_.back());
        recent_.pop_back();
    }
    return dem;
}

CesiumAsync::SharedFuture<std::optional<double>> DemCache::reference_z(const LonLatDeg& anchor) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (reference_ && (!reference_->isReady() || reference_->wait().has_value())) {
            return *reference_;
        }
    }
    const TileId t = tile_containing(anchor.lon, anchor.lat, kLandmarkZoom);
    const std::array<double, 2> px = dem_pixel_of(anchor, t);
    CesiumAsync::SharedFuture<std::optional<double>> z =
        get(t)
            .thenImmediately(
                [px](const std::shared_ptr<const DemTile>& dem) -> std::optional<double> {
                    if (!dem) return std::nullopt;
                    return dem_sample(*dem, px[0], px[1]);
                })
            .share();
    std::lock_guard<std::mutex> lock(mutex_);
    reference_ = z;
    return z;
}

bool DemCache::failing(const TileId& t14) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = entries_.find(key_of(t14));
    return it != entries_.end() && it->second.failed_at_s->load() >= 0.0;
}

uint64_t DemCache::requests() const { return requests_.load(); }

CesiumAsync::SharedFuture<Rebase> rebase_of(const std::shared_ptr<DemCache>& dem, const TileId& t14,
                                            const LonLatDeg& anchor) {
    const TileId at = tile_containing(anchor.lon, anchor.lat, kLandmarkZoom);
    // The weak reference keeps the cache from owning its own futures' continuations.
    // ponytail: failing() reads the tile's entry when the rebase resolves; an entry evicted or
    // asked again in between reads as not failing, and that attempt settles at z = 0 as before
    // the retry existed.
    return dem->get(t14)
        .thenImmediately([ref = dem->reference_z(anchor), cache = std::weak_ptr<DemCache>(dem), t14,
                          at](const std::shared_ptr<const DemTile>& tile) mutable {
            return ref.thenImmediately([tile, cache, t14, at](const std::optional<double>& z) {
                Rebase r{tile, z};
                if (const std::shared_ptr<DemCache> c = cache.lock()) {
                    r.retry = (!tile && c->failing(t14)) || (!z && c->failing(at));
                }
                return r;
            });
        })
        .share();
}

}

namespace overlume::testing {

std::string mapbox_dem_url(uint32_t x14, uint32_t y14) {
    return overlume::mapbox::fill_tile_template(overlume::mapbox::kDemUrlTemplate,
                                                {overlume::mapbox::kLandmarkZoom, x14, y14});
}

}
```

In `overlume/src/mapbox_test_hooks.hpp`:

Replace

```cpp
std::string mapbox_footprint_url(uint32_t x16, uint32_t y16);  // no query string
```

with

```cpp
std::string mapbox_footprint_url(uint32_t x16, uint32_t y16);  // no query string
std::string mapbox_dem_url(uint32_t x14, uint32_t y14);        // no query string
```

and

Replace

```cpp
    // Highest region top over built tiles, metres above the anchor ground.
    double max_tile_top_m = 0.0;
};
```

with

```cpp
    // Highest region top over built tiles, metres above the anchor ground.
    double max_tile_top_m = 0.0;
    // DEM tiles the source's DemCache requested (terrain layer); 0 once the source is torn down.
    uint64_t dem_requests = 0;
};
```

Reconfigure (a new `src/mapbox_*.cpp`) and build:

```bash
cd "$WORKTREE"
flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock cmake -S overlume -B overlume/build > /dev/null \
    && echo reconfigured \
    && CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
```

Expected: `reconfigured`, then a clean build: `mapbox_dem_cache.cpp` compiles into
`overlume_stream`, nothing calls it yet. No `reconfigured` line and no CMake error above it means
the lock wait ran out (exit 75) and nothing was built: run the step again.

- [ ] **Step 3: Write the failing tests** (`overlume/tests/test_mapbox_buildings.cpp`)

Replace

```cpp
#include "mapbox_glb_fixture.hpp"
```

with

```cpp
#include "mapbox_dem_fixture.hpp"
#include "mapbox_glb_fixture.hpp"
```

Replace

```cpp
#include <cstdlib>
```

with

```cpp
#include <cstdlib>
#include <functional>
```

In `AnchorTileLoadsWithStandardFilterApplied`:

Replace

```cpp
    EXPECT_EQ(s.retries, 0u);
    EXPECT_GT(s.max_build_ms, 0.0);
```

with

```cpp
    EXPECT_EQ(s.retries, 0u);
    EXPECT_GT(s.max_build_ms, 0.0);
    EXPECT_EQ(s.dem_requests, 0u) << "no DEM without the terrain layer";
```

and after `LandmarksAloneStillBuildTheTile`:

Replace

```cpp
    EXPECT_EQ(count_of(glb_json(glbs[0]), "\"mesh\":"), 1u) << "the landmark node alone";
    overlume::destroy_renderer(r);
}
```

with

```cpp
    EXPECT_EQ(count_of(glb_json(glbs[0]), "\"mesh\":"), 1u) << "the landmark node alone";
    overlume::destroy_renderer(r);
}

namespace {

namespace dem = overlume::testing::dem;

constexpr double kDemPi = 3.14159265358979323846;

std::string dem_url_of(const TileId& t) { return overlume::testing::mapbox_dem_url(t.x, t.y); }

std::string tile_name(const TileId& t) {
    return "14/" + std::to_string(t.x) + "/" + std::to_string(t.y);
}

void serve_dem(MapboxFixtureNetwork* n, const TileId& t,
               const std::function<double(double, double)>& height_m) {
    overlume::testing::mapbox_fixture_serve(n, dem_url_of(t), 200, dem::encode_dem_png(height_m));
}

// Ground metres per DEM pixel (512 per z14 tile) at tile t's centre, spherical Web Mercator.
double metres_per_dem_px(const TileId& t) {
    const double lat = overlume::mapbox::tile_point_to_lonlat(t, 0.5, 0.5, 1.0).lat;
    return 2.0 * kDemPi * 6378137.0 * std::cos(lat * kDemPi / 180.0) / 16384.0 / 512.0;
}

// The DEM pixel (u east, v south) under (x, y) in units of z16 square `index` of its z14 tile:
// 4096 units and 128 DEM pixels a square, squares row-major from the north-west.
std::array<double, 2> square_dem_px(size_t index, double x, double y) {
    return {(static_cast<double>(index % 4) + x / 4096.0) * 128.0,
            (static_cast<double>(index / 4) + y / 4096.0) * 128.0};
}

// The N numbers after the first `"key":[` in a GLB's JSON chunk, read as numbers so the
// writer's float format does not matter; nullopt when the key is absent.
template <size_t N>
std::optional<std::array<double, N>> json_numbers(const std::vector<uint8_t>& glb,
                                                  const std::string& key) {
    const std::string json = glb_json(glb);
    const std::string prefix = "\"" + key + "\":[";
    const size_t at = json.find(prefix);
    if (at == std::string::npos) return std::nullopt;
    std::array<double, N> out{};
    const char* p = json.c_str() + at + prefix.size();
    for (double& v : out) {
        char* end = nullptr;
        v = std::strtod(p, &end);
        if (end == p) return std::nullopt;
        p = end + 1;  // past ',' or ']'
    }
    return out;
}

// The z bounds (min, max) of the extrusions' POSITION accessor in the captured buildings GLB,
// in tile-frame metres: the only GLB without UVs (terrain tiles carry TEXCOORD_0) and the only
// accessor with bounds in it.
std::optional<std::array<double, 2>> extrusion_z_bounds(
    const std::vector<std::vector<uint8_t>>& glbs) {
    for (const std::vector<uint8_t>& glb : glbs) {
        if (glb_json(glb).find("TEXCOORD_0") != std::string::npos) continue;
        const std::optional<std::array<double, 3>> lo = json_numbers<3>(glb, "min");
        const std::optional<std::array<double, 3>> hi = json_numbers<3>(glb, "max");
        if (lo && hi) return std::array<double, 2>{(*lo)[2], (*hi)[2]};
    }
    return std::nullopt;
}

// The tile's DEM answers `status` with `body` on each of `attempts` attempts (more than one for a
// failure for now, which sends the tile back): the anchor tile's buildings then build at z = 0,
// the DEM was asked once per attempt, and the log holds `line` `lines` times and never a URL.
void expect_buildings_at_ground(uint16_t status, std::vector<uint8_t> body, const std::string& line,
                                size_t lines, uint64_t attempts = 1) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    FrozenRetryClock clock;  // the test moves the retry clock itself
    const TileId t = op_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    serve_anchor_tile(n, t);
    overlume::testing::mapbox_fixture_serve(n, dem_url_of(t), status, std::move(body));
    const size_t logFrom = log_size();
    ASSERT_TRUE(install(r, "mapbox://buildings,terrain", anchor_at(t, 0.5, 0.5), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    for (uint64_t k = 1; k < attempts; ++k) {
        ASSERT_TRUE(pump(r, buf, 100, [k] { return stats().retries >= k; }))
            << "attempt " << k << " must send the tile back";
        // The cache shares a failure for 1 s of steady time; the retry clock is frozen.
        std::this_thread::sleep_for(std::chrono::milliseconds(1100));
        clock.set(k == 1 ? kAttempt2AtS : kAttempt3AtS);
    }
    ASSERT_TRUE(pump(r, buf, 400, [&] { return loaded(r) >= 1; }));
    const overlume::testing::MapboxSourceStats s = stats();
    EXPECT_EQ(s.extruded_features, kAnchorTileExtruded);
    EXPECT_DOUBLE_EQ(s.max_tile_top_m, 25.0)
        << "square 15's 25 m box, not lifted: without a DEM buildings keep z = 0";
    EXPECT_EQ(s.retries, attempts - 1);
    EXPECT_EQ(s.dem_requests, attempts)
        << "one request per attempt, shared by the anchor's reference and the tile";
    EXPECT_EQ(times_requested(n, dem_url_of(t)), attempts);
    EXPECT_EQ(state(r), EnvironmentSourceState::STREAMING);
    const std::string log = log_since(logFrom);
    EXPECT_EQ(count_of(log, line), lines) << log;
    EXPECT_EQ(log.find("api.mapbox.com"), std::string::npos) << "log lines name tiles, not URLs";
    overlume::destroy_renderer(r);
}

}

TEST(MapboxBuildings, TerrainLiftsABlockOnTheHill) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const TileId t = op_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    serve_dem(n, t, dem::hill_height_m);
    // Square 10's north-west corner is the tile centre, where the hill peaks.
    serve_square(n, overlume::mapbox::footprint_tiles_of(t)[10],
                 {mvt::building(1, {box(0, 0, 400, 400)}, 30.0)});
    overlume::testing::capture_stream_glbs(true);
    // 554 m from the tile's west and north edges: the 500 m radius reaches this tile alone.
    ASSERT_TRUE(install(r, "mapbox://buildings,terrain", anchor_at(t, 0.25, 0.25), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 400, [&] { return loaded(r) >= 1; }));
    const std::vector<std::vector<uint8_t>> glbs = overlume::testing::captured_stream_glbs();
    overlume::testing::capture_stream_glbs(false);
    const std::array<double, 2> block = square_dem_px(10, 200.0, 200.0);
    const double lift = dem::hill_height_m(block[0], block[1]) - dem::hill_height_m(128.0, 128.0);
    ASSERT_GT(lift, 1.0) << "the fixture hill must rise from the anchor to the block";
    const overlume::testing::MapboxSourceStats s = stats();
    EXPECT_EQ(s.tiles_built, 1u);
    EXPECT_EQ(s.extruded_features, 1u);
    EXPECT_NEAR(s.max_tile_top_m, 30.0 + lift, 0.2) << "the region's top follows the block";
    const std::optional<std::array<double, 2>> z = extrusion_z_bounds(glbs);
    ASSERT_TRUE(z.has_value()) << "no buildings GLB was written";
    EXPECT_NEAR((*z)[0], lift, 0.2)
        << "the base stands on the DEM at the block's centroid, minus the anchor's DEM height";
    EXPECT_NEAR((*z)[1], 30.0 + lift, 0.2) << "and the roof 30 m above it";
    EXPECT_EQ(s.dem_requests, 1u) << "the anchor's reference and the tile share one request";
    EXPECT_EQ(times_requested(n, dem_url_of(t)), 1u);
    overlume::destroy_renderer(r);
}

TEST(MapboxBuildings, TerrainAsksOneDemPerTile) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const TileId d = op_tile();
    const TileId a{14, d.x - 1, d.y - 1}, b{14, d.x, d.y - 1}, c{14, d.x - 1, d.y};
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    serve_dem(n, d, dem::hill_height_m);  // a, b and c answer 204: no DEM there
    // 44 m into d from the corner the four tiles share: 500 m reaches all four.
    ASSERT_TRUE(install(r, "mapbox://buildings,terrain", anchor_at(d, 0.02, 0.02), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    const auto asked = [&] {
        size_t k = 0;
        for (const TileId& t : {a, b, c, d}) k += requested(n).count(dem_url_of(t));
        return k;
    };
    ASSERT_TRUE(pump(r, buf, 400, [&] { return asked() == 4; }))
        << "each of the four tiles asks for its DEM; asked " << asked();
    frames(r, buf, 20);
    for (const TileId& t : {a, b, c, d}) {
        EXPECT_EQ(times_requested(n, dem_url_of(t)), 1u) << tile_name(t);
    }
    EXPECT_EQ(stats().dem_requests, 4u)
        << "d's DEM serves the anchor's reference and d's buildings alike";
    overlume::destroy_renderer(r);
}

TEST(MapboxBuildings, TerrainKeepsABlockOnASlopeWithinAMetre) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const TileId t = op_tile();
    const double metresPerPx = metres_per_dem_px(t);
    const double rise = 0.05 * metresPerPx;  // a 5 % slope up to the east, per DEM pixel
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    serve_dem(n, t, [rise](double u, double) { return rise * u; });
    // A block 20 m east-west and 10 m north-south, 12 m tall, centred 100 m east and 30 m south of
    // the tile centre, which is square 10's north-west corner.
    const double perM = 4096.0 / (128.0 * metresPerPx);  // square units per ground metre
    const auto at = [perM](double m) { return static_cast<int32_t>(std::lround(m * perM)); };
    const int32_t x0 = at(90.0), x1 = at(110.0), y0 = at(25.0), y1 = at(35.0);
    serve_square(n, overlume::mapbox::footprint_tiles_of(t)[10],
                 {mvt::building(1, {box(x0, y0, x1, y1)}, 12.0)});
    overlume::testing::capture_stream_glbs(true);
    ASSERT_TRUE(install(r, "mapbox://buildings,terrain", anchor_at(t, 0.5, 0.5), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 400, [&] { return loaded(r) >= 1; }));
    const std::vector<std::vector<uint8_t>> glbs = overlume::testing::captured_stream_glbs();
    overlume::testing::capture_stream_glbs(false);
    // Terrain heights relative to the anchor's, at DEM pixel 256 (the tile centre).
    const auto terrain = [rise](double u) { return rise * (u - 256.0); };
    const double centroid = terrain(square_dem_px(10, 0.5 * (x0 + x1), 0.0)[0]);
    const double west = terrain(square_dem_px(10, x0, 0.0)[0]);
    const double east = terrain(square_dem_px(10, x1, 0.0)[0]);
    const std::optional<std::array<double, 2>> z = extrusion_z_bounds(glbs);
    ASSERT_TRUE(z.has_value()) << "no buildings GLB was written";
    const double base = (*z)[0];
    EXPECT_NEAR(base, centroid, 0.25) << "the base is the DEM at the footprint's centroid";
    EXPECT_NEAR(stats().max_tile_top_m, base + 12.0, 0.25);
    EXPECT_GT(base - west, 0.0) << "the downhill end floats";
    EXPECT_LT(base - west, 1.0) << "Review Focus 3: a 20 m block on a 5 % slope floats under 1 m";
    EXPECT_LT(east - base, 1.0) << "and its uphill end sinks under 1 m";
    overlume::destroy_renderer(r);
}

TEST(MapboxBuildings, TerrainLiftsLandmarkNodes) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const TileId t = downtown_tile();
    const double rise = 0.05 * metres_per_dem_px(t);  // 5 % up to the east
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    serve_downtown(n, t,
                   glb::build_landmark_glb({landmark_at(t, 7, kOverSquare10, kOverSquare10)},
                                           glb::GlbOptions{}));
    serve_dem(n, t, [rise](double u, double) { return rise * u; });
    overlume::testing::capture_stream_glbs(true);
    ASSERT_TRUE(install(r, "mapbox://buildings,terrain", anchor_at(t, 0.5, 0.5), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 400, [&] { return loaded(r) >= 1; }));
    const std::vector<std::vector<uint8_t>> glbs = overlume::testing::captured_stream_glbs();
    overlume::testing::capture_stream_glbs(false);
    // The node's anchor lies 37.5 DEM pixels east of the tile centre, the anchor's pixel.
    const double lift = rise * (kOverSquare10 / 8192.0 * 512.0 - 256.0);
    const overlume::testing::MapboxSourceStats s = stats();
    EXPECT_EQ(s.landmark_nodes, 1u);
    EXPECT_NEAR(s.max_tile_top_m, 100.0 + lift, 0.2)
        << "the region reaches the landmark's 100 m roof on the DEM at its node's anchor";
    std::optional<std::array<double, 16>> m;
    for (const std::vector<uint8_t>& glb : glbs) {
        if ((m = json_numbers<16>(glb, "matrix"))) break;
    }
    ASSERT_TRUE(m.has_value()) << "no GLB carries the landmark node's matrix";
    EXPECT_NEAR((*m)[14], lift, 0.2) << "the node itself is lifted, not only the region";
    overlume::destroy_renderer(r);
}

TEST(MapboxBuildings, TerrainRetriesAFailingDem) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    FrozenRetryClock clock;
    const TileId t = op_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    overlume::testing::mapbox_fixture_serve(n, dem_url_of(t), 503, {});
    serve_square(n, overlume::mapbox::footprint_tiles_of(t)[10],
                 {mvt::building(1, {box(0, 0, 400, 400)}, 30.0)});
    overlume::testing::capture_stream_glbs(true);
    ASSERT_TRUE(install(r, "mapbox://buildings,terrain", anchor_at(t, 0.25, 0.25), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 100, [] { return stats().retries >= 1; }))
        << "a DEM failing for now must send the tile back, not build it at z = 0";
    EXPECT_EQ(loaded(r), 0u);
    serve_dem(n, t, dem::hill_height_m);
    // The cache shares a failure for 1 s of steady time; the retry clock is frozen.
    std::this_thread::sleep_for(std::chrono::milliseconds(1100));
    clock.set(kAttempt2AtS);
    ASSERT_TRUE(pump(r, buf, 400, [&] { return loaded(r) >= 1; }));
    const std::vector<std::vector<uint8_t>> glbs = overlume::testing::captured_stream_glbs();
    overlume::testing::capture_stream_glbs(false);
    const std::array<double, 2> block = square_dem_px(10, 200.0, 200.0);
    const double lift = dem::hill_height_m(block[0], block[1]) - dem::hill_height_m(128.0, 128.0);
    const std::optional<std::array<double, 2>> z = extrusion_z_bounds(glbs);
    ASSERT_TRUE(z.has_value()) << "no buildings GLB was written";
    EXPECT_NEAR((*z)[0], lift, 0.2) << "attempt 2 builds the block on the DEM that answered";
    const overlume::testing::MapboxSourceStats s = stats();
    EXPECT_EQ(s.retries, 1u);
    EXPECT_EQ(s.dem_requests, 2u) << "one request per attempt";
    EXPECT_EQ(times_requested(n, dem_url_of(t)), 2u);
    overlume::destroy_renderer(r);
}

TEST(MapboxBuildings, MissingDemLeavesBuildingsAtGround) {
    expect_buildings_at_ground(404, {}, "terrain-DEM", 0u);
}

TEST(MapboxBuildings, MalformedDemLeavesBuildingsAtGroundAndLogsOnce) {
    const std::string html = "<!DOCTYPE html><html><body>Service Unavailable</body></html>";
    expect_buildings_at_ground(200, std::vector<uint8_t>(html.begin(), html.end()),
                               "mapbox:// terrain tile " + tile_name(op_tile()) + " skipped: ", 1u);
}

TEST(MapboxBuildings, FailingDemLeavesBuildingsAtGroundAndLogsOnce) {
    expect_buildings_at_ground(
        503, {}, "mapbox: terrain-DEM failing (HTTP 503) at tile " + tile_name(op_tile()), 1u, 3u);
}

TEST(MapboxBuildings, RejectedDemLeavesBuildingsAtGroundAndLogsOnce) {
    expect_buildings_at_ground(401, {}, "Mapbox rejected MAPBOX_TOKEN (HTTP 401) for terrain-DEM",
                               1u);
}
```

What each test catches when its behaviour is reverted:
- `TerrainLiftsABlockOnTheHill`: extrusions not lifted (the GLB's POSITION bounds stay 0..30 m),
  a lift taken from the wrong pixel or without the reference subtracted (bounds off by metres),
  the region top not following, the reference and the tile asking twice.
- `TerrainAsksOneDemPerTile`: a tile that asks no DEM, or asks twice (d is also the anchor's tile,
  so the reference and d's build must share one request).
- `TerrainKeepsABlockOnASlopeWithinAMetre` (Review Focus 3): a base from the lowest or highest
  vertex instead of the centroid (the base 0.5 m off, and the downhill end floating 1 m or not at
  all), no lift at all.
- `TerrainLiftsLandmarkNodes`: landmark nodes left on the anchor's ground (`matrix[14]` 0, not
  8.1 m), or lifted at the wrong place; the region top not following them.
- `TerrainRetriesAFailingDem`: a tile built at z = 0 on a DEM failing for now instead of sent back,
  or a retry that asks the DEM from memory instead of the network.
- `MissingDemLeavesBuildingsAtGround`, `MalformedDemLeavesBuildingsAtGroundAndLogsOnce`,
  `FailingDemLeavesBuildingsAtGroundAndLogsOnce`, `RejectedDemLeavesBuildingsAtGroundAndLogsOnce`:
  a lift from a missing DEM, a second request inside one attempt, a 404, an unreadable DEM or a
  401 sent back for more attempts, a 503 given up before the third attempt, a 404 logged, a
  malformed DEM, a 503 or a 401 not logged or logged with its URL.
- `AnchorTileLoadsWithStandardFilterApplied` (amended): a DEM requested without the terrain layer.

- [ ] **Step 4: Run them and see them fail**

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_mapbox_buildings
overlume/build/test_mapbox_buildings --gtest_filter='MapboxBuildings.*' | tail -13
```

Expected: the tests compile against Step 2's hooks; the loader does not use the cache yet, so no
DEM is requested (`dem_requests` 0), nothing is lifted and nothing retries, and the run ends

```
[  FAILED  ] 9 tests, listed below:
[  FAILED  ] MapboxBuildings.TerrainLiftsABlockOnTheHill
[  FAILED  ] MapboxBuildings.TerrainAsksOneDemPerTile
[  FAILED  ] MapboxBuildings.TerrainKeepsABlockOnASlopeWithinAMetre
[  FAILED  ] MapboxBuildings.TerrainLiftsLandmarkNodes
[  FAILED  ] MapboxBuildings.TerrainRetriesAFailingDem
[  FAILED  ] MapboxBuildings.MissingDemLeavesBuildingsAtGround
[  FAILED  ] MapboxBuildings.MalformedDemLeavesBuildingsAtGroundAndLogsOnce
[  FAILED  ] MapboxBuildings.FailingDemLeavesBuildingsAtGroundAndLogsOnce
[  FAILED  ] MapboxBuildings.RejectedDemLeavesBuildingsAtGroundAndLogsOnce

 9 FAILED TESTS
```

The other 23 `MapboxBuildings` tests pass, the amended one included. On a machine without
GPU/EGL every test reports `[  SKIPPED ]`: run this step on the GPU rig.

- [ ] **Step 5: Re-base the buildings** (`overlume/src/mapbox_loader.cpp`)

The includes (`frame_to_dem_px` needs `glm::make_mat4`, whose header Task 3 dropped):

Replace

```cpp
#include "environment_stream.hpp"
#include "mapbox_extrude.hpp"
```

with

```cpp
#include "environment_stream.hpp"
#include "mapbox_dem_cache.hpp"
#include "mapbox_extrude.hpp"
```

Replace

```cpp
#include <CesiumGeospatial/Ellipsoid.h>
```

with

```cpp
#include <CesiumGeospatial/Cartographic.h>
#include <CesiumGeospatial/Ellipsoid.h>
```

Replace

```cpp
#include <spdlog/spdlog.h>
```

with

```cpp
#include <glm/gtc/type_ptr.hpp>

#include <spdlog/spdlog.h>
```

Replace

```cpp
#include <mutex>
```

with

```cpp
#include <mutex>
#include <numbers>
#include <optional>
```

`LoaderState` keeps the cache for the stats:

Replace

```cpp
    std::atomic<double> max_tile_top_m{0.0};
```

with

```cpp
    std::atomic<double> max_tile_top_m{0.0};
    // The source's DEM cache when the layer list carries `terrain`, for dem_requests; the loaders
    // own it.
    std::weak_ptr<const DemCache> dem;
```

Before `build_tile`, the pixel helpers:

Replace

```cpp
// Worker thread: one z14 tile from its answers, in request order.
```

with

```cpp
// (u, v) in units of z16 square `square` (`extent` per edge) → DEM pixels of its z14 tile t14.
std::array<double, 2> square_to_dem_px(const TileId& square, const TileId& t14, double u, double v,
                                       double extent) {
    constexpr double kPxPerSquare = kDemSize / 4.0;
    return {(static_cast<double>(square.x - 4 * t14.x) + u / extent) * kPxPerSquare,
            (static_cast<double>(square.y - 4 * t14.y) + v / extent) * kPxPerSquare};
}

// The DEM pixel under tile-frame point (x, y, z) of frame f's z14 tile: frame → ECEF → longitude
// and latitude, project_to_frame inverted.
std::array<double, 2> frame_to_dem_px(const TileFrame& f, double x, double y, double z) {
    const glm::dvec4 ecef = glm::make_mat4(f.frame_to_ecef.data()) * glm::dvec4(x, y, z, 1.0);
    const std::optional<CesiumGeospatial::Cartographic> c =
        CesiumGeospatial::Ellipsoid::WGS84.cartesianToCartographic(glm::dvec3(ecef));
    if (!c) return {kDemSize / 2.0, kDemSize / 2.0};
    constexpr double kDegPerRad = 180.0 / std::numbers::pi;
    return dem_pixel_of(LonLatDeg{c->longitude * kDegPerRad, c->latitude * kDegPerRad}, f.tile);
}

// Worker thread: one z14 tile from its answers, in request order.
```

`build_tile` takes the rebase:

Replace

```cpp
                          const std::vector<Fetched>& fetched,
                          const std::shared_ptr<CesiumAsync::IAssetAccessor>& accessor,
```

with

```cpp
                          const std::vector<Fetched>& fetched, const Rebase& rebase,
                          const std::shared_ptr<CesiumAsync::IAssetAccessor>& accessor,
```

sends the tile back for a DEM failing for now:

Replace

```cpp
    if (transient && attempt < kMaxAttempts) {
```

with

```cpp
    // On the terrain layer a DEM failing for now sends the tile back too: built at z = 0, its
    // buildings would stay buried or floating once the DEM answers.
    if ((transient || rebase.retry) && attempt < kMaxAttempts) {
```

lifts the landmark nodes:

Replace

```cpp
    TriangleMesh mesh;
    uint64_t extruded = 0;
    uint64_t conflated = 0;
    uint64_t walls = 0;
    double top = landmarks.ok ? landmark_top_m(landmarks.model) : 0.0;
```

with

```cpp
    // On the terrain layer each landmark node stands on the DEM at its anchor, the node's origin:
    // its matrix's third column is the local up per metre (landmark_node_to_frame).
    double low = 0.0;           // the lowest lift, for the region's floor
    double landmarkLift = 0.0;  // the highest node lift, for the region's top
    if (landmarks.ok && rebase.active()) {
        bool first = true;
        for (CesiumGltf::Node& node : landmarks.model.nodes) {
            if (node.matrix.size() != 16) continue;
            const std::array<double, 2> px =
                frame_to_dem_px(frame, node.matrix[12], node.matrix[13], node.matrix[14]);
            const double lift = rebase.lift(px[0], px[1]);
            for (size_t i = 0; i < 3; ++i) node.matrix[12 + i] += lift * node.matrix[8 + i];
            landmarkLift = first ? lift : std::max(landmarkLift, lift);
            low = std::min(low, lift);
            first = false;
        }
    }
    TriangleMesh mesh;
    uint64_t extruded = 0;
    uint64_t conflated = 0;
    uint64_t walls = 0;
    double top = landmarks.ok ? landmark_top_m(landmarks.model) + landmarkLift : 0.0;
```

lifts the extrusions:

Replace

```cpp
            const std::vector<TilePolygon> polygons = to_tile_polygons(b.polygons);
            // A landmark model replaces the extrusion whose unclipped footprint centroid it
            // covers, as GL JS does. read_landmark_tile returns an empty tile on every failure,
            // which is what the tests pin; this ok check and the merge's are belt and braces.
            if (landmarks.ok) {
                if (const std::optional<std::array<double, 2>> c = footprint_centroid(polygons)) {
                    const std::array<double, 3> p = project((*c)[0], (*c)[1], 0.0);
                    if (inside_any_footprint(landmarks.footprint_triangles, p[0], p[1])) {
                        ++conflated;
                        continue;
                    }
                }
            }
            // Clipping to the square's own extent drops the tile buffer, so the copies of a
            // building in neighbouring squares meet without doubled walls.
            const ExtrudeStats es =
                extrude_polygons(polygons, extent, base, *b.height, project, mesh);
            if (es.polygons_built == 0) continue;
            ++extruded;
            walls += es.wall_quads;
            top = std::max(top, *b.height);
```

with

```cpp
            const std::vector<TilePolygon> polygons = to_tile_polygons(b.polygons);
            const std::optional<std::array<double, 2>> c = footprint_centroid(polygons);
            // A landmark model replaces the extrusion whose unclipped footprint centroid it
            // covers, as GL JS does. read_landmark_tile returns an empty tile on every failure,
            // which is what the tests pin; this ok check and the merge's are belt and braces.
            if (landmarks.ok && c) {
                const std::array<double, 3> p = project((*c)[0], (*c)[1], 0.0);
                if (inside_any_footprint(landmarks.footprint_triangles, p[0], p[1])) {
                    ++conflated;
                    continue;
                }
            }
            // On the terrain layer the footprint stands on the DEM at its unclipped centroid,
            // flat-bottomed (spec section 12). ponytail: one base per square's copy, so a building
            // cut by a z16 square edge on a slope steps at the cut; key the lift by feature id
            // across the tile's squares if that shows.
            double lift = 0.0;
            if (rebase.active() && c) {
                const std::array<double, 2> px =
                    square_to_dem_px(square, t14, (*c)[0], (*c)[1], extent);
                lift = rebase.lift(px[0], px[1]);
            }
            // Clipping to the square's own extent drops the tile buffer, so the copies of a
            // building in neighbouring squares meet without doubled walls.
            const ExtrudeStats es =
                extrude_polygons(polygons, extent, base + lift, *b.height + lift, project, mesh);
            if (es.polygons_built == 0) continue;
            ++extruded;
            walls += es.wall_quads;
            top = std::max(top, *b.height + lift);
            low = std::min(low, lift);
```

and spans the region from the lowest lifted base (`build_tile`'s last statement, as Task 3's
Edit 7 left it):

Replace

```cpp
    return success(
        std::move(model),
        tile_region(t14, cfg.origin_height_m - kRegionBelowM, cfg.origin_height_m + top, ellipsoid),
        accessor, ellipsoid);
```

with

```cpp
    // From the lowest lifted base (kRegionBelowM under it, as before) to the highest top.
    return success(std::move(model),
                   tile_region(t14, cfg.origin_height_m + low - kRegionBelowM,
                               cfg.origin_height_m + top, ellipsoid),
                   accessor, ellipsoid);
```

The loader holds the cache and the anchor (Task 3's `begin` returns the tile's `Attempt` as `a`,
whose `settled` the continuation keeps):

Replace

```cpp
    MapboxBuildingsLoader(std::shared_ptr<const LoaderConfig> config,
                          std::shared_ptr<LoaderState> state)
        : config_(std::move(config)), state_(std::move(state)) {}
```

with

```cpp
    // `dem` is the source's DemCache when the layer list carries `terrain`, else null.
    MapboxBuildingsLoader(std::shared_ptr<const LoaderConfig> config,
                          std::shared_ptr<LoaderState> state, std::shared_ptr<DemCache> dem,
                          LonLatDeg anchor)
        : config_(std::move(config)),
          state_(std::move(state)),
          dem_(std::move(dem)),
          anchor_(anchor) {}
```

Replace

```cpp
        return input.asyncSystem.all(std::move(requests))
            .thenInWorkerThread([config = config_, state = state_, t14, attempt,
                                 settled = a.settled, accessor = input.pAssetAccessor,
                                 logger = input.pLogger,
                                 ellipsoid = input.ellipsoid](std::vector<Fetched>&& fetched) {
                return build_tile(*config, *state, t14, attempt, *settled, fetched, accessor,
                                  *logger, ellipsoid);
            });
```

with

```cpp
        // With the terrain layer the build also waits for the tile's DEM and the anchor's
        // height; without it, for nothing.
        CesiumAsync::SharedFuture<Rebase> rebase =
            dem_ ? rebase_of(dem_, t14, anchor_)
                 : input.asyncSystem.createResolvedFuture(Rebase{}).share();
        CesiumAsync::SharedFuture<std::vector<Fetched>> answers =
            input.asyncSystem.all(std::move(requests)).share();
        return rebase.thenImmediately([answers, config = config_, state = state_, t14, attempt,
                                       settled = a.settled, accessor = input.pAssetAccessor,
                                       logger = input.pLogger,
                                       ellipsoid = input.ellipsoid](const Rebase& r) mutable {
            return answers.thenInWorkerThread([r, config, state, t14, attempt, settled, accessor,
                                               logger,
                                               ellipsoid](const std::vector<Fetched>& fetched) {
                return build_tile(*config, *state, t14, attempt, *settled, fetched, r, accessor,
                                  *logger, ellipsoid);
            });
        });
```

The members (comments above each, so clang-format leaves Task 3's `attempts_` line alone):

Replace

```cpp
    std::shared_ptr<const LoaderConfig> config_;
    std::shared_ptr<LoaderState> state_;
```

with

```cpp
    std::shared_ptr<const LoaderConfig> config_;
    std::shared_ptr<LoaderState> state_;
    // Null without the terrain layer.
    std::shared_ptr<DemCache> dem_;
    // The map origin, whose DEM height is z = 0.
    LonLatDeg anchor_;
```

`open_mapbox_source` builds the cache with the terrain layer:

Replace

```cpp
    auto loader = std::make_unique<MapboxBuildingsLoader>(
        std::make_shared<const LoaderConfig>(
            LoaderConfig{std::move(token), spec.materials_original, anchor.origin_height_m}),
        std::move(state));
```

with

```cpp
    // The terrain layer: one DEM per z14 tile, shared by the source's loaders and never on disk
    // (the accessor is the cache-off counting one).
    const std::string query = "?access_token=" + token;
    std::shared_ptr<DemCache> dem;
    if (spec.terrain) {
        dem = std::make_shared<DemCache>(externals.pAssetAccessor, asyncSystem,
                                         std::string(kDemUrlTemplate) + query, log);
        state->dem = dem;
    }
    auto loader = std::make_unique<MapboxBuildingsLoader>(
        std::make_shared<const LoaderConfig>(
            LoaderConfig{std::move(token), spec.materials_original, anchor.origin_height_m}),
        std::move(state), dem, LonLatDeg{anchor.origin_lon_deg, anchor.origin_lat_deg});
```

and `mapbox_last_source_stats` reads it:

Replace

```cpp
    out.max_tile_top_m = state->max_tile_top_m.load();
```

with

```cpp
    out.max_tile_top_m = state->max_tile_top_m.load();
    if (const std::shared_ptr<const overlume::mapbox::DemCache> dem = state->dem.lock()) {
        out.dem_requests = dem->requests();
    }
```

- [ ] **Step 6: Run them and see them pass**

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_mapbox_buildings
env -u OVERLUME_LIVE_MAPBOX overlume/build/test_mapbox_buildings | tail -5
```

Expected: every `MapboxBuildings.*` and both `MapboxBuildingsGolden.*` tests `[       OK ]`, and
the run ends

```
[  PASSED  ] 34 tests.
[  SKIPPED ] 2 tests, listed below:
[  SKIPPED ] MapboxLive.DowntownDubai
[  SKIPPED ] MapboxLive.OperatingArea
```

(`env -u` keeps an exported `OVERLUME_LIVE_MAPBOX` from taking the `MapboxLive` tests to the
network.) The goldens, on `mapbox://buildings`, pin that nothing moves without the terrain layer.
`TerrainRetriesAFailingDem` and `FailingDemLeavesBuildingsAtGroundAndLogsOnce` spend 1.1 s and
2.2 s in their steady-clock waits. A failure here is a finding to diagnose, never an expectation
to loosen; on a machine without GPU/EGL run on the GPU rig.

- [ ] **Step 7: Format**

```bash
cd "$WORKTREE"
git add overlume/src/mapbox_dem_cache.hpp overlume/src/mapbox_dem_cache.cpp \
    overlume/src/mapbox_loader.cpp overlume/src/mapbox_test_hooks.hpp \
    overlume/tests/test_mapbox_buildings.cpp
tools/check_format.sh --fix && tools/check_format.sh && git diff --stat && tools/check_spdx.sh
```

Expected: `check_format.sh: reformatted N files with clang-format 23.1.1`, then
`check_format.sh: PASS (N files, clang-format 23.1.1)`, an empty `git diff --stat` (every block
above is clang-format 23.1.1 clean; if a file is listed, `git add` it again) and
`check_spdx.sh: PASS (… files carry the SPDX identifier)`.

- [ ] **Step 8: Revert checks (do not commit)**: the lifts, the shared request, the retry

```bash
cd "$WORKTREE"
sed -i 's/extrude_polygons(polygons, extent, base + lift, \*b.height + lift, project, mesh);/extrude_polygons(polygons, extent, base, *b.height, project, mesh);  \/\/ REVERT CHECK/' \
    overlume/src/mapbox_loader.cpp
sed -i 's/for (size_t i = 0; i < 3; ++i) node.matrix\[12 + i] += lift \* node.matrix\[8 + i];/for (size_t i = 0; i < 0; ++i) node.matrix[12 + i] += lift * node.matrix[8 + i];  \/\/ REVERT CHECK/' \
    overlume/src/mapbox_loader.cpp
grep -c 'REVERT CHECK' overlume/src/mapbox_loader.cpp
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_mapbox_buildings
overlume/build/test_mapbox_buildings --gtest_filter='MapboxBuildings.Terrain*' | tail -7
git checkout -- overlume/src/mapbox_loader.cpp
sed -i 's/^        if (failedAt < 0.0 || steady_s() - failedAt < kRetryWaitS \/ 2.0) {$/        if (false) {  \/\/ REVERT CHECK/' \
    overlume/src/mapbox_dem_cache.cpp
grep -c 'REVERT CHECK' overlume/src/mapbox_dem_cache.cpp
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_mapbox_buildings
overlume/build/test_mapbox_buildings \
    --gtest_filter='MapboxBuildings.Terrain*:MapboxBuildings.*DemLeavesBuildingsAtGround*' | tail -10
git checkout -- overlume/src/mapbox_dem_cache.cpp
sed -i 's/^    if ((transient || rebase.retry) \&\& attempt < kMaxAttempts) {$/    if (transient \&\& attempt < kMaxAttempts) {  \/\/ REVERT CHECK/' \
    overlume/src/mapbox_loader.cpp
grep -c 'REVERT CHECK' overlume/src/mapbox_loader.cpp
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_mapbox_buildings
overlume/build/test_mapbox_buildings \
    --gtest_filter='MapboxBuildings.TerrainRetriesAFailingDem:MapboxBuildings.FailingDem*' | tail -5
git checkout -- overlume/src/mapbox_loader.cpp
grep -c 'REVERT CHECK' overlume/src/mapbox_loader.cpp overlume/src/mapbox_dem_cache.cpp || true
git diff --stat
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_mapbox_buildings
```

Expected: the first `grep` prints `2`; with the extrusions and the nodes left on the ground the
first run ends

```
[  FAILED  ] 4 tests, listed below:
[  FAILED  ] MapboxBuildings.TerrainLiftsABlockOnTheHill
[  FAILED  ] MapboxBuildings.TerrainKeepsABlockOnASlopeWithinAMetre
[  FAILED  ] MapboxBuildings.TerrainLiftsLandmarkNodes
[  FAILED  ] MapboxBuildings.TerrainRetriesAFailingDem

 4 FAILED TESTS
```

on the GLB bounds (0 and 30 m for the hill block, a 0 m base on the slope and after the retry) and
on the node's `matrix[14]` (0, not 8.1 m); the region tops still follow, which is why the tests
read the geometry. The second `grep` prints `1`; with every `get()` asking again the second run
ends

```
[  FAILED  ] 7 tests, listed below:
[  FAILED  ] MapboxBuildings.TerrainLiftsABlockOnTheHill
[  FAILED  ] MapboxBuildings.TerrainAsksOneDemPerTile
[  FAILED  ] MapboxBuildings.TerrainRetriesAFailingDem
[  FAILED  ] MapboxBuildings.MissingDemLeavesBuildingsAtGround
[  FAILED  ] MapboxBuildings.MalformedDemLeavesBuildingsAtGroundAndLogsOnce
[  FAILED  ] MapboxBuildings.FailingDemLeavesBuildingsAtGroundAndLogsOnce
[  FAILED  ] MapboxBuildings.RejectedDemLeavesBuildingsAtGroundAndLogsOnce

 7 FAILED TESTS
```

on `dem_requests` and `times_requested` (twice the expected count: the anchor's reference asks
again beside the tile's own request). The build may warn that `failedAt` is unused (nothing builds
with `-Werror`). The third `grep` prints `1`; with the tile built at z = 0 whatever the DEM did,
the third run ends

```
[  FAILED  ] 2 tests, listed below:
[  FAILED  ] MapboxBuildings.TerrainRetriesAFailingDem
[  FAILED  ] MapboxBuildings.FailingDemLeavesBuildingsAtGroundAndLogsOnce

 2 FAILED TESTS
```

on `retries` never reaching 1. After the checkouts the last `grep` prints `…:0` for both files,
`git diff --stat` prints nothing, and the rebuild leaves the binary current.

- [ ] **Step 9: Gate** (foreground, Bash timeout 600000 ms)

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
ROS_DOMAIN_ID=85 CI_VISUAL_MODE_DOMAIN_ID=85 \
    flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock tools/ci_visual_mode.sh
```

Expected: six `PASS` lines (POD header check, library ctest suite, node gtests, WS bridge pytest
suite, golden suite, examples), `OVERALL: PASS`, exit 0. The library ctest count is 9 higher than
at the lane's fork (the gate the orchestrator ran in `$INTEGRATION` after merging Tasks 1-4),
every `MapboxBuildings.*` entry among the ok ones; the node, WS bridge, golden (all `ok`, none
moved) and example counts equal the fork's. Exit 75 means another lane held the lock for 300 s:
run the same command again. A WS bridge failure can also come from another session's gate on the
same ports: rerun before treating it as a finding.

- [ ] **Step 10: Commit**

```bash
cd "$WORKTREE"
git add overlume/src/mapbox_dem_cache.hpp overlume/src/mapbox_dem_cache.cpp \
    overlume/src/mapbox_loader.cpp overlume/src/mapbox_test_hooks.hpp \
    overlume/tests/test_mapbox_buildings.cpp
git status --short
git commit -F - <<'EOF'
feat(mapbox): DEM cache; buildings stand on the terrain-DEM

With `terrain` in the mapbox:// layer list, a DemCache shared by the
source's Mapbox loaders fetches each z14 tile's terrain-DEM once (LRU of
32 tiles, through the cache-off counting accessor, so no byte reaches the
disk), and the source's first request sets the reference: the DEM height
at the anchor. Each extruded footprint's base moves to the DEM height at
its unclipped centroid and each landmark node to the height at its
anchor, both minus the reference, so the terrain will pass through z = 0
at the anchor with every building standing on it. A DEM failing for now
(429, 5xx, transport) sends the tile back for another attempt, as a
failing footprint square does, so a start-up burst never leaves a tile's
buildings buried or floating while the terrain recovers; the failure is
shared for 1 s, so the callers of one attempt share one request and the
next attempt asks again. Without a DEM after that (204, 404, an
unreadable PNG, a rejected token, the third failure) buildings keep
z = 0; an unreadable DEM, a rejected token and a failure for now each
log one line per source, naming the tile, never the URL. Without the
terrain layer nothing changes: no DEM request, the same geometry.

MapboxSourceStats gains dem_requests and the test hooks mapbox_dem_url.
Nine MapboxBuildings tests on synthetic DEMs: a block lifted on the
hill, one DEM per tile across four tiles, a 20 m block on a 5 % slope
floating under 1 m at its downhill end (Review Focus 3), landmark nodes
lifted, a 503 then a 200 lifting the block on attempt 2, and 404,
malformed, persistent 503 and 401 DEMs leaving the buildings at z = 0
with their log lines.

Claude-Session: <the executing session's URL>
EOF
```

Expected: `git status --short` lists `A ` for the two new files and `M ` for the other three; the
commit succeeds. Then report Task 9 committed.

### Task 10: Mapbox terrain loader and source wiring

**Lane:** 5. **Depends on:** Task 9 (same worktree, after its commit) and Tasks 5 and 11 merged
into `mapbox-environment` (Step 1 brings them in), as head.md's Lanes table says: this task
calls Task 11's `StreamingEnvironmentSource::attach_terrain` and relies on Task 11's two-tileset
`update()`, terrain-targeted sampling, `credits()` and `build_externals`' credit system.

**Files:**
- Create: `overlume/src/mapbox_terrain_loader.cpp` (the loader and `make_mapbox_terrain_tileset`,
  which `mapbox_loader.cpp` declares, ruling R19: the File map gives this task no header)
- Modify: `overlume/src/mapbox_loader.cpp` as Task 9 left it (the Replace blocks quote Task 9's
  text: includes; after `#include <variant>`; after `landmark_url`; `LoaderState`; the loader's
  gate, constructor and members; the end of `open_mapbox_source`; the hooks
  `mapbox_footprint_url` and `mapbox_last_source_stats`)
- Modify: `overlume/src/mapbox_test_hooks.hpp` (after Task 9's `mapbox_dem_url`; after Task 9's
  `dem_requests`)
- Test: `overlume/tests/test_mapbox_terrain.cpp` (create; suite `MapboxTerrain`, 7 tests; Tasks
  12-14 extend it)

**Interfaces:**
- Consumes:

```cpp
// overlume/src/mapbox_dem.hpp (Task 1)
struct DemMesh { std::vector<std::array<float, 3>> positions, normals; std::vector<std::array<float, 2>> uvs;
                 std::vector<uint32_t> indices; uint32_t grid_vertex_count = 0; };
DemMesh mesh_dem(const DemTile& dem, const TileFrame& frame, const TileId& t14, double z_ref_m,
                 double skirt_m = kDemSkirtM);
// overlume/src/mapbox_tiling.hpp (Task 3)
inline constexpr uint32_t kMaxAttempts = 3;
struct Attempt { uint32_t count = 0; double not_before_s = 0.0; std::shared_ptr<std::atomic<bool>> settled; };
class AttemptBook { public: bool waiting(const TileId& t14, double now_s) const;
                            Attempt& begin(const TileId& t14, double now_s); };
double clock_s();  // the retry clock mapbox_set_test_clock freezes
struct LoaderConfig { std::string token; bool materials_original = false; double origin_height_m = 0.0; };
Cesium3DTilesSelection::TileLoadResult success(/* content, bounds, accessor, ellipsoid */);
Cesium3DTilesSelection::BoundingVolume tile_region(const TileId& t, double min_height_m, double max_height_m,
                                                   const CesiumGeospatial::Ellipsoid& ellipsoid = WGS84);
std::unique_ptr<Cesium3DTilesSelection::Tile> quadtree_root(Cesium3DTilesSelection::TilesetContentLoader& loader,
                                                            double origin_height_m);
std::vector<Cesium3DTilesSelection::Tile> quadtree_children(const Cesium3DTilesSelection::Tile& parent,
                                                            const LoaderConfig& cfg, /* ellipsoid */);
// Task 9: DemCache, kDemUrlTemplate, fill_tile_template, Rebase, rebase_of, LoaderState::dem,
// the loader's dem_/anchor_
// overlume/src/environment_stream.hpp (Tasks 5 and 11, and the existing classes)
enum class StreamRendererResources::Kind : uint8_t { Content, Terrain };
StreamRendererResources(const glm::dmat4& ecefToMap, double anchorHeightM,
                        bool materialsOriginal = false, double brightness = 1.0,
                        Kind kind = Kind::Content);
void StreamingEnvironmentSource::attach_terrain(std::unique_ptr<Cesium3DTilesSelection::Tileset> terrain,
                                                std::shared_ptr<StreamRendererResources> terrain_resources);
struct CustomTileset;  // loader, root, area_selection
glm::dmat4 compute_ecef_to_map(const GeoAnchor& anchor);
inline constexpr double kStreamMaxSseErr = 48.0;
// overlume/src/environment_test_hooks.hpp (Tasks 5 and 11, and the existing hooks)
OverlayProbe environment_stream_overlay_probe(overlume::VisualRenderer* r);
bool environment_stream_has_terrain_tileset(overlume::VisualRenderer* r);
uint64_t environment_terrain_tiles_loaded(overlume::VisualRenderer* r);
bool environment_stream_provides_ground(overlume::VisualRenderer* r);
bool renderer_ground_plane_in_scene(overlume::VisualRenderer* r);
double environment_terrain_offset_z(overlume::VisualRenderer* r);  // the follow-terrain offset
// overlume/include/overlume/scene.h (Task 11)
uint32_t environment_source_credits(VisualRenderer*, char* out, uint32_t cap);
// overlume/tests/mapbox_dem_fixture.hpp, imagery_fixture.hpp (Task 2)
double ramp_height_m(double u, double v);  // 0..20 m west to east
std::vector<uint8_t> flat_png(uint32_t size, uint8_t r, uint8_t g, uint8_t b);
```

- Produces, verbatim from the contract: `MapboxSourceStats::terrain_tiles_built` and
  `std::string mapbox_satellite_url(uint32_t z, uint32_t x, uint32_t y);` (XYZ, no query string).
  Behaviour: `mapbox://…terrain…` opens a terrain tileset beside the buildings and attaches it;
  `imagery=satellite` drapes Mapbox Satellite on it, its requests and cesium's failure lines going
  through the source's counting accessor and redacting logger (no line of ours, ruling R2); with
  the terrain layer the source follows the terrain and the terrain owns the ground; a layer list
  without `buildings` requests no buildings.

**How the terrain loader works** (spec section 5, `MapboxTerrainLoader`; read with Step 5's code):
- **Tiles.** `quadtree_root` and `quadtree_children` (Task 3), as the buildings loader uses them:
  levels 0-13 are empty and always refined, and the z14 leaves carry the tile frame as their
  transform. The terrain's z14 leaves get geometric error 2 m: the area view refines a tile whose
  error is 1 or more and cesium halves it for each level of upsampled children, so with imagery z14
  and z15 refine and the z16 upsampled tiles, whose imagery is z18, are the leaves near the ego.
  Without imagery a z14 leaf has no children and renders as itself.
- **Loading.** A z14 tile counts its attempt with Task 3's `AttemptBook` on `clock_s()` (which
  `mapbox_set_test_clock` freezes) exactly as the buildings loader does: while `waiting()`,
  `RetryLater` without a request; else `begin()`. It asks Task 9's `rebase_of` for its DEM and the
  anchor's reference, shared with the buildings through the cache, and meshes on a worker
  (`mesh_dem`, skirts 20 m), writes one glTF with `write_dem_gltf` (POSITION, NORMAL,
  TEXCOORD_0, uint32 indices, one material that the terrain resources replace) and returns Task 3's `success` with the region of
  its heights; `terrain_tiles_built` counts it. A DEM or reference failing for now
  (`Rebase::retry`) returns `RetryLater` until the third attempt; an empty or unreadable DEM, or
  the third attempt, settles as an empty tile (no terrain there).
- **The source.** `open_mapbox_source` builds the buildings source as before, with
  `follow_terrain` and `replaces_ground` set from the terrain layer; then the terrain tileset over
  a copy of the source's externals (Task 11's `build_externals`: the counting accessor, logger,
  task processor and credit system) with its own `StreamRendererResources` of kind `Terrain`,
  `maximumScreenSpaceError` 48 and no sibling preloading; with `imagery=satellite`, a `UrlTemplateRasterOverlay` named
  `mapbox-satellite` (512 px tiles, `maximumLevel` 18, credit "© Maxar", the token in the query
  string); then `attach_terrain`.

**API notes** (cesium-native 0.64, paths under `overlume/build/_deps/cesium-native-src/`; the
code was compiled as Task 9's, with Tasks 5 and 11's declarations stubbed from the contract and
lane-6.md, and every block, applied on top of Task 9's, is a clang-format 23.1.1 fixed point):
- `CesiumRasterOverlays/include/CesiumRasterOverlays/UrlTemplateRasterOverlay.h:28-74` the options
  (`credit` :32, `maximumLevel` :57, `tileWidth` :62, `tileHeight` :67), `:110-119` the
  constructor; `{y}` counts rows from the south (:87), `{reverseY}` from the north (:90).
  `src/UrlTemplateRasterOverlay.cpp:84-110` the placeholders; `:135-154` Web Mercator and its
  one-root quadtree by default; `CesiumGeometry/src/QuadtreeTilingScheme.cpp:64-78` row 0 at
  `minimumY`, the south; `QuadtreeTileID.cpp:16-20` `computeInvertedY`. Mapbox's raster tiles are
  XYZ (row 0 at the north), hence `{reverseY}` in the overlay's template: head.md pins the
  `{reverseY}` template passed to cesium, which `satellite_overlay_template()` builds;
  `kSatelliteTemplate` keeps the XYZ `{y}` form for `mapbox_satellite_url`.
  `CesiumUtility/src/Uri.cpp:258-288` substitutes only `{…}`, so the query string passes through.
- `src/UrlTemplateRasterOverlay.cpp:62-68`: the credit exists only when the externals carry a
  credit system; Task 11's `build_externals` sets one (lane-6.md, "What Task 10 may rely on"), so
  the copy carries it and "© Maxar" reaches `credits()`.
- `Cesium3DTilesSelection/src/RasterOverlayCollection.cpp:58-63`: overlays request through the
  tileset's `pAssetAccessor` (the fixture network sees imagery requests; they count toward network
  loss) and use its credit system; `include/…/RasterOverlayCollection.h:148`
  `add(IntrusivePointer<const RasterOverlay>)`; `Tileset.h:53-57` the custom-loader constructor,
  `:218` `getOverlays()`. `CesiumAsync/include/CesiumAsync/IAssetAccessor.h:39-70` the three
  virtuals the satellite accessor forwards.
- `CesiumRasterOverlays/src/RasterOverlay.cpp:100-131`: the overlay's `loadErrorCallback` fires
  only when its tile provider cannot be created. A raster tile's HTTP failure becomes
  `Received response code {} for image {url}.` (`RasterOverlayTileProvider.cpp:183-191`); a
  204 without a body becomes `Image response for {url} is empty.` (`:198-218`, no
  `allowEmptyImages` for a URL template). `QuadtreeRasterOverlayTileProvider.cpp:70-84` and
  `:124-140`: a failed quadtree tile loads its parent instead, level by level, and drops its own
  errors, so only a failure at level 0 (`minimumLevel`, 0 by default,
  `UrlTemplateRasterOverlay.h:52`) keeps its message; `:461-488` a set of images with no useful
  data passes those errors on, and `ActivatedRasterOverlay.cpp:194-212` logs them
  (`ErrorList::logError`, "Errors while loading image for tile:\n- …") through the tileset's
  logger, the source's redacting one. That is the spec's imagery failure row (section 5): no line
  of ours, and no rejected-token line for imagery (ruling R2; the spec keeps that line for the
  DEM).
- `src/TilesetContentManager.cpp:405-459` cesium writes the overlay texture coordinates into every
  loader's glTF; `:533-540` it reads the up axis from the result's `glTFUpAxis` (Z, set by
  `success`) and copies a `pCompletedRequest`'s URL into the model, which `success` therefore never
  sets; `:2105-2150` upsampled children only for a loaded tile with no real children and more
  imagery detail available; `:259` and `:280` a non-zero parent error, halved per child level;
  `:2070-2101` an empty z14 tile under ancestors that always refine is a hole, not a placeholder;
  `Tile.cpp:156-183` `getNonZeroGeometricError` falls back to 1e-5, which no area view refines.
  `TilesetSelection.cpp:329-336`: an area view's tile meets the threshold while its error is
  below it (1.0 here, `overlume/src/environment_stream.cpp:1106-1112`).
- `CesiumRasterOverlays/src/QuadtreeRasterOverlayTileProvider.cpp:145-182` the imagery level from
  the geometry tile's error (`RasterOverlayUtilities.cpp:605-647`), `:256-287` the corner tiles by
  `positionToTile`, which floors, from the tile region's projected rectangle, whose north edge
  equals the tiling boundary only to rounding (so a neighbour row may be mapped: the test serves a
  one-tile ring), `:312-325` the level lowered until the tile's texture fits 2048 px
  (`RasterOverlay.h:69`), and `:443-490`, `:620` the quadtree tiles combined into one image: one
  raster per terrain tile, which Task 5's single `imagery` sampler needs. A z14 tile maps 16 z16
  images, a z15 tile 16 z17, a z16 tile 16 z18: at the defaults the ego's tile asks about 144
  satellite tiles (16 + 64 + 64).
- `overlume/src/environment_stream.hpp:325-332` `CustomTileset`; `:336-344` the source's
  constructor (`follow_terrain`, `ground_bias_m` 0.3, `replaces_ground`, `max_tilt_deg` 2.0, as the
  buildings source passed them); `overlume/src/environment_stream.cpp:905-975` the terrain sample:
  five points along the ego's heading (map x at heading 0), a plane fit whose intercept is the
  height under the ego; `:992-996` the offset target `-(height - anchor height) - ground_bias`,
  clamped to ±30 m; `:406-430` `compute_ecef_to_map`: at heading 0 map x points north and map y
  west. Task 11 samples the terrain tileset whenever it replaces the ground, but moves the
  environment only with `follow_terrain` (lane-6.md, "Ground"), so the ground test reads
  `environment_terrain_offset_z` to pin the follow flag. `overlume/src/renderer.cpp:1118-1125`
  hides the ground plane while the source provides the ground.
- `overlume/src/environment.cpp:238-256`: `environment_loaded_chunk_count` is the source's
  `loaded_count()` (`environment_stream.cpp:1220-1223`, the content tileset's tiles in the scene),
  `environment_stream_provides_ground` and `renderer_ground_plane_in_scene`.
  `environment_stream_has_disk_cache` inspects the content tileset; the terrain tileset's accessor
  wraps the same counting accessor, so neither reaches a disk cache.

- [ ] **Step 1: Bring Tasks 5 and 11 into the lane**

```bash
cd "$WORKTREE"
git merge --no-ff \
    -m "Merge mapbox-environment into terrain-env/l5 for the terrain resources and the second tileset (Tasks 5, 11)" \
    -m "Claude-Session: <the executing session's URL>" mapbox-environment
grep -c 'void attach_terrain(\|enum class Kind' overlume/src/environment_stream.hpp
grep -c 'environment_stream_overlay_probe\|environment_stream_has_terrain_tileset\|environment_terrain_tiles_loaded' \
    overlume/src/environment_test_hooks.hpp
grep -c 'environment_source_credits' overlume/include/overlume/scene.h
grep -c 'pCreditSystem' overlume/src/environment_stream.cpp
flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock cmake -S overlume -B overlume/build > /dev/null \
    && echo reconfigured \
    && CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
env -u OVERLUME_LIVE_MAPBOX overlume/build/test_mapbox_buildings | tail -5
```

Expected: `Merge made by the 'ort' strategy.` with Task 5's and Task 11's files among those listed
(`overlume/assets/materials/terrain.mat`, `overlume/src/environment_stream.cpp`, …) and no
conflict: neither touches a file of this lane. The greps print `2`, `3`, `1` and at least `1`
(more where a comment repeats a name; a `0` means a declaration is missing). Then `reconfigured`,
a clean build, and Step 6 of Task 9's ending (`[  PASSED  ] 34 tests.`, two `MapboxLive` skips).
A conflict or a missing declaration is a plan deviation: stop and report it. No `reconfigured`
line and no CMake error above it means the lock wait ran out (exit 75): run the block again from
the `flock` line.

- [ ] **Step 2: Declare the stats and the satellite URL**

In `overlume/src/mapbox_test_hooks.hpp`:

Replace

```cpp
std::string mapbox_dem_url(uint32_t x14, uint32_t y14);        // no query string
```

with

```cpp
std::string mapbox_dem_url(uint32_t x14, uint32_t y14);        // no query string
// Mapbox Satellite's XYZ tile (row 0 at the north edge), no query string.
std::string mapbox_satellite_url(uint32_t z, uint32_t x, uint32_t y);
```

and

Replace

```cpp
    // DEM tiles the source's DemCache requested (terrain layer); 0 once the source is torn down.
    uint64_t dem_requests = 0;
```

with

```cpp
    // DEM tiles the source's DemCache requested (terrain layer); 0 once the source is torn down.
    uint64_t dem_requests = 0;
    // Terrain z14 tiles that reached Success with content.
    uint64_t terrain_tiles_built = 0;
```

In `overlume/src/mapbox_loader.cpp`, the satellite constants after `landmark_url`:

Replace

```cpp
std::string landmark_url(uint32_t x14, uint32_t y14) {
    return "https://api.mapbox.com/3dtiles/v1/mapbox.mapbox-3dbuildings-v1-meshopt/14/" +
           std::to_string(x14) + "/" + std::to_string(y14) + ".glb";
}
```

with

```cpp
std::string landmark_url(uint32_t x14, uint32_t y14) {
    return "https://api.mapbox.com/3dtiles/v1/mapbox.mapbox-3dbuildings-v1-meshopt/14/" +
           std::to_string(x14) + "/" + std::to_string(y14) + ".glb";
}

// Mapbox Satellite, 512 px JPEG tiles; in the UAE the detail stops near z18.
constexpr char kSatelliteTemplate[] =
    "https://api.mapbox.com/v4/mapbox.satellite/{z}/{x}/{y}@2x.jpg90";
constexpr uint32_t kSatelliteMaxLevel = 18;
constexpr uint32_t kSatelliteTilePx = 512;
constexpr char kSatelliteCredit[] = "\xC2\xA9 Maxar";

// The overlay's template: cesium's {y} counts rows from the south (QuadtreeTilingScheme.cpp:74)
// and Mapbox's XYZ y from the north, which is cesium's {reverseY}.
std::string satellite_overlay_template() {
    std::string url = kSatelliteTemplate;
    return url.replace(url.find("{y}"), 3, "{reverseY}");
}
```

the counter in `LoaderState` (the terrain loader shares it):

Replace

```cpp
    std::weak_ptr<const DemCache> dem;
```

with

```cpp
    std::weak_ptr<const DemCache> dem;
    // Terrain z14 tiles built, counted by the terrain loader (mapbox_terrain_loader.cpp).
    std::shared_ptr<std::atomic<uint64_t>> terrain_tiles_built =
        std::make_shared<std::atomic<uint64_t>>(0);
```

the hook:

Replace

```cpp
std::string mapbox_footprint_url(uint32_t x16, uint32_t y16) {
    return overlume::mapbox::footprint_url(x16, y16);
}
```

with

```cpp
std::string mapbox_footprint_url(uint32_t x16, uint32_t y16) {
    return overlume::mapbox::footprint_url(x16, y16);
}

std::string mapbox_satellite_url(uint32_t z, uint32_t x, uint32_t y) {
    return overlume::mapbox::fill_tile_template(overlume::mapbox::kSatelliteTemplate, {z, x, y});
}
```

and the stats:

Replace

```cpp
        out.dem_requests = dem->requests();
    }
```

with

```cpp
        out.dem_requests = dem->requests();
    }
    out.terrain_tiles_built = state->terrain_tiles_built->load();
```

- [ ] **Step 3: Write the failing tests** (create `overlume/tests/test_mapbox_terrain.cpp`)

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume/api.h"
#include "overlume/scene.h"

#include "environment_test_hooks.hpp"
#include "imagery_fixture.hpp"
#include "mapbox_dem_fixture.hpp"
#include "mapbox_mvt_fixture.hpp"
#include "mapbox_test_hooks.hpp"
#include "mapbox_tile_frame.hpp"
#include "test_paths.hpp"

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace {

namespace dem = overlume::testing::dem;
namespace imagery = overlume::testing::imagery;
namespace mvt = overlume::testing::mvt;
using overlume::EnvironmentSourceState;
using overlume::mapbox::TileId;
using overlume::testing::MapboxFixtureNetwork;

constexpr const char* kToken = "pk.test-token-not-real";
constexpr size_t kFrameBytes = 320u * 240u * 3u;
const overlume::CameraPose kPose{{0, -8, 3}, {0, 0, 0.5}, 60};
// The loaders' retry schedule on a frozen clock: attempt 2 no sooner than 2 s after attempt 1,
// attempt 3 no sooner than 4 s after attempt 2.
constexpr double kAttempt2AtS = 2.0;
constexpr double kAttempt3AtS = 6.0;
constexpr double kPi = 3.14159265358979323846;

// Tile math runs on use, not during static initialisation: it may lean on statics of other
// translation units. The operating area's z14 tile, 10712/7012.
TileId op_tile() { return overlume::mapbox::tile_containing(55.3910, 25.0803, 14); }

overlume::GeoAnchor anchor_at(const TileId& t, double u, double v) {
    const overlume::mapbox::LonLatDeg p = overlume::mapbox::tile_point_to_lonlat(t, u, v, 1.0);
    return overlume::GeoAnchor{p.lat, p.lon, 0.0, 0.0};
}

// Ground metres per DEM pixel (512 per z14 tile) at tile t's centre, spherical Web Mercator.
double metres_per_dem_px(const TileId& t) {
    const double lat = overlume::mapbox::tile_point_to_lonlat(t, 0.5, 0.5, 1.0).lat;
    return 2.0 * kPi * 6378137.0 * std::cos(lat * kPi / 180.0) / 16384.0 / 512.0;
}

std::string dem_url_of(const TileId& t) { return overlume::testing::mapbox_dem_url(t.x, t.y); }

void serve_dem(MapboxFixtureNetwork* n, const TileId& t,
               const std::function<double(double, double)>& height_m) {
    overlume::testing::mapbox_fixture_serve(n, dem_url_of(t), 200, dem::encode_dem_png(height_m));
}

// One 20 m building in square 10 of tile t (z16 units 1000-1400), gzip as Mapbox sends it.
void serve_block(MapboxFixtureNetwork* n, const TileId& t) {
    mvt::Layer layer;
    layer.name = "building";
    layer.features = {
        mvt::building(1, {{{1000, 1000}, {1400, 1000}, {1400, 1400}, {1000, 1400}}}, 20.0)};
    const TileId square = overlume::mapbox::footprint_tiles_of(t)[10];
    overlume::testing::mapbox_fixture_serve(
        n, overlume::testing::mapbox_footprint_url(square.x, square.y), 200,
        mvt::gzip_stored(mvt::encode_tile({layer})));
}

overlume::VisualRenderer* make_renderer() {
    return overlume::create_renderer(overlume::RenderConfig{320, 240, 1, kThemeDir, "dark_adas"});
}

bool install(overlume::VisualRenderer* r, const std::string& uri, const overlume::GeoAnchor& anchor,
             MapboxFixtureNetwork* n) {
    return overlume::testing::install_fixture_mapbox_source(r, uri.c_str(), anchor, n, kToken);
}

// Map x points north and map y west at heading 0 (compute_ecef_to_map).
void set_ego(overlume::VisualRenderer* r, double x, double y) {
    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = overlume::Vec3{x, y, 0.0};
    overlume::set_scene(r, s);
}

uint64_t loaded(overlume::VisualRenderer* r) {
    return overlume::testing::environment_loaded_chunk_count(r);
}

uint64_t terrain_loaded(overlume::VisualRenderer* r) {
    return overlume::testing::environment_terrain_tiles_loaded(r);
}

EnvironmentSourceState state(overlume::VisualRenderer* r) {
    return overlume::environment_source_state(r);
}

overlume::testing::MapboxSourceStats stats() {
    return overlume::testing::mapbox_last_source_stats();
}

// Renders until done() or max_frames; the sleep lets worker-thread builds land between frames.
template <typename Done>
bool pump(overlume::VisualRenderer* r, std::vector<uint8_t>& buf, int max_frames, Done done) {
    for (int i = 0; i < max_frames && !done(); ++i) {
        if (!overlume::render_frame(r, kPose, {buf.data(), 320, 240})) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return done();
}

void frames(overlume::VisualRenderer* r, std::vector<uint8_t>& buf, int count) {
    pump(r, buf, count, [] { return false; });
}

// Every requested URL, without its query string.
std::set<std::string> requested(MapboxFixtureNetwork* n) {
    std::set<std::string> out;
    for (const std::string& url : overlume::testing::mapbox_fixture_requested_urls(n)) {
        out.insert(url.substr(0, url.find('?')));
    }
    return out;
}

// How often `url` (without its query string) has been requested.
size_t times_requested(MapboxFixtureNetwork* n, const std::string& url) {
    size_t count = 0;
    for (const std::string& u : overlume::testing::mapbox_fixture_requested_urls(n)) {
        if (u.compare(0, u.find('?'), url) == 0) ++count;
    }
    return count;
}

size_t log_size() { return overlume::testing::captured_cesium_log_text().size(); }

std::string log_since(size_t from) {
    const std::string text = overlume::testing::captured_cesium_log_text();
    return text.size() > from ? text.substr(from) : std::string();
}

size_t count_of(const std::string& text, const std::string& needle) {
    size_t n = 0;
    for (size_t at = text.find(needle); at != std::string::npos; at = text.find(needle, at + 1)) {
        ++n;
    }
    return n;
}

bool ends_with(const std::string& s, const std::string& suffix) {
    return s.size() >= suffix.size() &&
           s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

// Freezes the loaders' retry clock at 0 s for one test, which then moves time with set(). The
// steady clock comes back when the test ends, however it ends.
class FrozenRetryClock {
public:
    FrozenRetryClock() { set(0.0); }
    ~FrozenRetryClock() { overlume::testing::mapbox_set_test_clock(-1.0); }
    FrozenRetryClock(const FrozenRetryClock&) = delete;
    FrozenRetryClock& operator=(const FrozenRetryClock&) = delete;
    void set(double seconds) { overlume::testing::mapbox_set_test_clock(seconds); }
};

}

TEST(MapboxTerrain, TerrainTileBuildsFromItsDem) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const TileId t = op_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    serve_dem(n, t, dem::hill_height_m);
    ASSERT_TRUE(install(r, "mapbox://terrain", anchor_at(t, 0.5, 0.5), n));
    EXPECT_TRUE(overlume::testing::environment_stream_has_terrain_tileset(r));
    EXPECT_FALSE(overlume::testing::environment_stream_has_disk_cache(r))
        << "no Mapbox byte reaches the disk (the hook reads the content tileset, whose counting "
           "accessor the terrain tileset's wraps)";
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 400, [&] { return terrain_loaded(r) >= 1; }))
        << "the terrain tile never reached the scene";
    const overlume::testing::MapboxSourceStats s = stats();
    EXPECT_EQ(s.terrain_tiles_built, 1u) << "500 m around the tile centre is one z14 tile";
    EXPECT_EQ(s.tiles_built, 0u) << "no buildings layer, no buildings";
    EXPECT_EQ(s.dem_requests, 1u);
    EXPECT_EQ(requested(n), std::set<std::string>{dem_url_of(t)})
        << "the terrain tile asks for its DEM and nothing else";
    EXPECT_EQ(overlume::testing::mapbox_fixture_requested_urls(n).size(), 1u) << "once";
    EXPECT_EQ(state(r), EnvironmentSourceState::STREAMING);
    overlume::destroy_renderer(r);
}

TEST(MapboxTerrain, OneDemFeedsBothLoaders) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const TileId t = op_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    serve_dem(n, t, dem::hill_height_m);
    serve_block(n, t);
    ASSERT_TRUE(install(r, "mapbox://buildings,terrain", anchor_at(t, 0.5, 0.5), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 400, [&] { return loaded(r) >= 1 && terrain_loaded(r) >= 1; }));
    const overlume::testing::MapboxSourceStats s = stats();
    EXPECT_EQ(s.tiles_built, 1u);
    EXPECT_EQ(s.terrain_tiles_built, 1u);
    EXPECT_EQ(s.dem_requests, 1u);
    EXPECT_EQ(times_requested(n, dem_url_of(t)), 1u)
        << "the anchor's reference, the buildings and the terrain share one request";
    overlume::destroy_renderer(r);
}

TEST(MapboxTerrain, MissingDemLeavesNoTerrainTile) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    FrozenRetryClock clock;
    const TileId t = op_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    overlume::testing::mapbox_fixture_serve(n, dem_url_of(t), 404, {});
    const size_t logFrom = log_size();
    ASSERT_TRUE(install(r, "mapbox://terrain", anchor_at(t, 0.5, 0.5), n));
    EXPECT_TRUE(overlume::testing::environment_stream_has_terrain_tileset(r));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 100, [&] { return times_requested(n, dem_url_of(t)) == 1; }));
    frames(r, buf, 20);
    clock.set(kAttempt2AtS);
    frames(r, buf, 20);
    clock.set(kAttempt3AtS);
    frames(r, buf, 20);
    EXPECT_EQ(stats().terrain_tiles_built, 0u);
    EXPECT_EQ(terrain_loaded(r), 0u) << "no terrain there: the sky shows through";
    EXPECT_EQ(times_requested(n, dem_url_of(t)), 1u)
        << "a 404 is no failure for now: the tile settles empty and nothing asks again";
    EXPECT_EQ(stats().dem_requests, 1u);
    EXPECT_EQ(state(r), EnvironmentSourceState::STREAMING) << "one 404 is not network loss";
    EXPECT_EQ(count_of(log_since(logFrom), "HTTP 404"), 0u) << "404 is an empty tile, not an error";
    overlume::destroy_renderer(r);
}

TEST(MapboxTerrain, FailingDemIsAskedAgainOnTheNextAttempt) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    FrozenRetryClock clock;
    const TileId t = op_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    overlume::testing::mapbox_fixture_serve(n, dem_url_of(t), 503, {});
    const size_t logFrom = log_size();
    ASSERT_TRUE(install(r, "mapbox://terrain", anchor_at(t, 0.5, 0.5), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 100, [&] { return times_requested(n, dem_url_of(t)) == 1; }));
    frames(r, buf, 20);
    EXPECT_EQ(times_requested(n, dem_url_of(t)), 1u)
        << "the anchor's reference and the tile share the attempt's one request";
    serve_dem(n, t, dem::hill_height_m);
    // The cache shares a failure for 1 s of steady time and the retry clock is frozen: wait it out.
    std::this_thread::sleep_for(std::chrono::milliseconds(1100));
    clock.set(kAttempt2AtS);
    ASSERT_TRUE(pump(r, buf, 400, [] { return stats().terrain_tiles_built == 1; }))
        << "attempt 2 must ask again and build from the 200";
    EXPECT_EQ(times_requested(n, dem_url_of(t)), 2u);
    EXPECT_EQ(stats().dem_requests, 2u);
    EXPECT_EQ(count_of(log_since(logFrom), "mapbox: terrain-DEM failing (HTTP 503)"), 1u);
    overlume::destroy_renderer(r);
}

TEST(MapboxTerrain, SatelliteDrapesXyzTilesOnTheTerrain) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const TileId t = op_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    serve_dem(n, t, dem::hill_height_m);
    // From z0 to the overlay's z18 cap, every XYZ tile over t and the ring of tiles around it
    // answers one flat 512 px image: what t's terrain may ask for when the rows are Mapbox's (row 0
    // at the north edge), the ring because cesium may map a neighbour row a rounding error away
    // (QuadtreeRasterOverlayTileProvider.cpp:256-287). `inside` keeps the z15-z18 tiles over t.
    std::set<std::string> served;
    std::set<std::string> inside;
    for (uint32_t z = 0; z <= 18; ++z) {
        const int64_t k = z <= 14 ? 1 : int64_t{1} << (z - 14);
        const int64_t x0 = z <= 14 ? t.x >> (14 - z) : t.x * k;
        const int64_t y0 = z <= 14 ? t.y >> (14 - z) : t.y * k;
        const int64_t side = int64_t{1} << z;
        for (int64_t y = y0 - 1; y <= y0 + k; ++y) {
            for (int64_t x = x0 - 1; x <= x0 + k; ++x) {
                if (x < 0 || y < 0 || x >= side || y >= side) continue;
                const std::string url = overlume::testing::mapbox_satellite_url(
                    z, static_cast<uint32_t>(x), static_cast<uint32_t>(y));
                served.insert(url);
                if (z >= 15 && x >= x0 && x < x0 + k && y >= y0 && y < y0 + k) inside.insert(url);
            }
        }
    }
    const std::vector<uint8_t> png = imagery::flat_png(512, 40, 120, 60);
    for (const std::string& url : served) overlume::testing::mapbox_fixture_serve(n, url, 200, png);
    ASSERT_TRUE(install(r, "mapbox://terrain?imagery=satellite", anchor_at(t, 0.5, 0.5), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 600, [&] {
        return overlume::testing::environment_stream_overlay_probe(r).rasters_attached >= 1;
    })) << "no satellite raster attached to the terrain";
    size_t overT = 0;
    for (const std::string& url : overlume::testing::mapbox_fixture_requested_urls(n)) {
        const std::string bare = url.substr(0, url.find('?'));
        if (bare.find("mapbox.satellite") == std::string::npos) continue;
        EXPECT_EQ(served.count(bare), 1u)
            << "not an XYZ tile over the terrain tile or its ring: " << bare;
        overT += inside.count(bare);
        EXPECT_TRUE(ends_with(url, std::string("?access_token=") + kToken)) << bare;
    }
    EXPECT_GT(overT, 0u) << "no z15-z18 imagery over the terrain tile itself";
    char credits[256] = {};
    overlume::environment_source_credits(r, credits, static_cast<uint32_t>(sizeof credits));
    EXPECT_NE(std::string(credits).find("\xC2\xA9 Maxar"), std::string::npos)
        << "the overlay's credit, for the attribution line: " << credits;
    overlume::destroy_renderer(r);
}

TEST(MapboxTerrain, FailedSatelliteTileLogsThroughTheRedactingLogger) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const TileId t = op_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    serve_dem(n, t, dem::hill_height_m);
    // Every imagery tile answers 204, which cesium reads as a failed image and replaces by its
    // parent, level by level, dropping the child's error; the z0 tile answers 401, the one failure
    // cesium keeps and logs. The 204s are successes to the network-loss count, so the source keeps
    // streaming.
    const std::string z0 = overlume::testing::mapbox_satellite_url(0, 0, 0);
    overlume::testing::mapbox_fixture_serve(n, z0, 401, {});
    const std::string line = "Received response code 401 for image ";
    const size_t logFrom = log_size();
    ASSERT_TRUE(install(r, "mapbox://terrain?imagery=satellite", anchor_at(t, 0.5, 0.5), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    const auto logged = [&] {
        return count_of(log_since(logFrom), line) >= 1 && stats().terrain_tiles_built >= 1;
    };
    ASSERT_TRUE(pump(r, buf, 600, logged))
        << "cesium's line for the failed tile never reached the source's log: "
        << log_since(logFrom);
    frames(r, buf, 20);
    const std::string log = log_since(logFrom);
    EXPECT_GE(times_requested(n, z0), 1u);
    EXPECT_EQ(log.find(kToken), std::string::npos) << "the token never reaches the log";
    EXPECT_EQ(stats().terrain_tiles_built, 1u) << "a failed raster leaves the terrain tile";
    EXPECT_EQ(state(r), EnvironmentSourceState::STREAMING);
    overlume::destroy_renderer(r);
}

TEST(MapboxTerrain, TerrainLayerReplacesTheGroundPlane) {
    for (const bool terrain : {true, false}) {
        auto* r = make_renderer();
        if (!r) GTEST_SKIP() << "no GPU/EGL";
        const TileId t = op_tile();
        auto* n = overlume::testing::make_mapbox_fixture_network(204);
        serve_dem(n, t, dem::ramp_height_m);  // 0..20 m west to east
        serve_block(n, t);
        ASSERT_TRUE(install(r, terrain ? "mapbox://buildings,terrain" : "mapbox://buildings",
                            anchor_at(t, 0.5, 0.5), n));
        // The ego stands 300 m east of the anchor (map y points west), up the ramp.
        constexpr double kEgoEastM = 300.0;
        set_ego(r, 0.0, -kEgoEastM);
        std::vector<uint8_t> buf(kFrameBytes);
        ASSERT_TRUE(pump(r, buf, 400, [&] { return loaded(r) >= 1; }));
        if (terrain) {
            EXPECT_TRUE(pump(r, buf, 400, [&] {
                return overlume::testing::environment_stream_provides_ground(r);
            })) << "the terrain owns the ground once the sample under the ego hits it";
            frames(r, buf, 2);
            EXPECT_FALSE(overlume::testing::renderer_ground_plane_in_scene(r))
                << "the clay ground plane is hidden";
            // Following the terrain moves the environment down by the ramp's rise under the ego
            // and the 0.3 m ground bias, so the ego stands on the terrain.
            const double rise = dem::ramp_height_m(256.0 + kEgoEastM / metres_per_dem_px(t), 0.0) -
                                dem::ramp_height_m(256.0, 0.0);
            const double expected = -(rise + 0.3);
            const auto onTerrain = [&] {
                return std::abs(overlume::testing::environment_terrain_offset_z(r) - expected) <
                       0.5;
            };
            EXPECT_TRUE(pump(r, buf, 200, onTerrain))
                << "the source follows the terrain: offset "
                << overlume::testing::environment_terrain_offset_z(r) << ", expected " << expected;
        } else {
            frames(r, buf, 20);
            EXPECT_FALSE(overlume::testing::environment_stream_provides_ground(r))
                << "buildings alone never replace the ground plane";
            EXPECT_TRUE(overlume::testing::renderer_ground_plane_in_scene(r));
        }
        overlume::destroy_renderer(r);
    }
}
```

What each test catches when its behaviour is reverted:
- `TerrainTileBuildsFromItsDem`: no terrain tileset, no terrain tile, a terrain tile that asks
  anything but its DEM, buildings requested although the layer list lacks `buildings`.
- `OneDemFeedsBothLoaders`: the terrain loader asking its own DEM beside the buildings' (spec
  section 8: one DEM per tile with two consumers).
- `MissingDemLeavesNoTerrainTile`: a terrain tile from no DEM, a 404 asked again on later
  attempts, a 404 logged or counted as network loss.
- `FailingDemIsAskedAgainOnTheNextAttempt`: a terrain tile that gives up after one failure, or
  asks twice inside one attempt.
- `SatelliteDrapesXyzTilesOnTheTerrain`: no overlay, an overlay on `{y}` (TMS rows: none of the
  served tiles, so nothing attaches), a token outside the query string, no "© Maxar" credit.
- `FailedSatelliteTileLogsThroughTheRedactingLogger`: a terrain tileset (and so its overlay)
  logging anywhere but the source's redacting logger, where the tests read cesium's line and the
  token is masked; a failed raster taking the terrain tile down or the source to its fallback.
- `TerrainLayerReplacesTheGroundPlane`: the terrain not owning the ground, the source not following
  it (`environment_terrain_offset_z` stays 0), or buildings alone hiding the ground plane.

- [ ] **Step 4: Run them and see them fail**

```bash
cd "$WORKTREE"
flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock cmake -S overlume -B overlume/build > /dev/null \
    && echo reconfigured \
    && CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
overlume/build/test_mapbox_terrain | tail -11
```

Expected: `reconfigured` (a new `tests/test_*.cpp`), a clean build (the test compiles against
Step 2), and with no terrain tileset yet the run ends

```
[  FAILED  ] 7 tests, listed below:
[  FAILED  ] MapboxTerrain.TerrainTileBuildsFromItsDem
[  FAILED  ] MapboxTerrain.OneDemFeedsBothLoaders
[  FAILED  ] MapboxTerrain.MissingDemLeavesNoTerrainTile
[  FAILED  ] MapboxTerrain.FailingDemIsAskedAgainOnTheNextAttempt
[  FAILED  ] MapboxTerrain.SatelliteDrapesXyzTilesOnTheTerrain
[  FAILED  ] MapboxTerrain.FailedSatelliteTileLogsThroughTheRedactingLogger
[  FAILED  ] MapboxTerrain.TerrainLayerReplacesTheGroundPlane

 7 FAILED TESTS
```

(`environment_stream_has_terrain_tileset` false, no terrain tile, no raster or imagery request,
no ground). The build may warn that `satellite_overlay_template`, `kSatelliteMaxLevel`,
`kSatelliteTilePx` and `kSatelliteCredit` are unused until Step 6 (nothing builds
with `-Werror`). No `reconfigured` line means the lock wait ran out: run the block again.

- [ ] **Step 5: The terrain loader** (create `overlume/src/mapbox_terrain_loader.cpp`)

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

// The Mapbox terrain loader (docs/design/2026-10-15-imagery-terrain-design.md, section 5): the
// z14 terrain-DEM tiles within the radius, meshed with skirts in each tile's east-north-up frame.
// Raster overlays drape on them, and cesium upsamples below z14 for finer imagery.

#include "environment_stream.hpp"
#include "mapbox_dem.hpp"
#include "mapbox_dem_cache.hpp"
#include "mapbox_tile_frame.hpp"
#include "mapbox_tiling.hpp"

#include <Cesium3DTilesSelection/Tile.h>
#include <Cesium3DTilesSelection/TileLoadResult.h>
#include <Cesium3DTilesSelection/TilesetContentLoader.h>
#include <CesiumGeometry/QuadtreeTileID.h>
#include <CesiumGeospatial/Ellipsoid.h>
#include <CesiumGltf/Model.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cstring>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace overlume::mapbox {

namespace {

using Cesium3DTilesSelection::Tile;
using Cesium3DTilesSelection::TileChildrenResult;
using Cesium3DTilesSelection::TileEmptyContent;
using Cesium3DTilesSelection::TileLoadInput;
using Cesium3DTilesSelection::TileLoadResult;
using Cesium3DTilesSelection::TileLoadResultState;

// The area view refines a tile whose geometric error is 1 or more (TilesetSelection.cpp:335) and
// cesium halves it for each level of upsampled children (TilesetContentManager.cpp:280): z14 (2)
// and z15 (1) refine, z16 (0.5) is the leaf near the ego, and a z16 tile maps z18 imagery, the
// overlay's cap (at most 2048 px a tile, QuadtreeRasterOverlayTileProvider.cpp:312-325).
constexpr double kTerrainGeometricErrorM = 2.0;

static_assert(sizeof(std::array<float, 3>) == 3 * sizeof(float) &&
              sizeof(std::array<float, 2>) == 2 * sizeof(float));

// One buffer (positions, normals, UVs, uint32 indices), one TRIANGLES primitive, one material
// (the terrain renderer resources swap it for terrain.mat), one node in scene 0.
CesiumGltf::Model write_dem_gltf(const DemMesh& mesh) {
    using CesiumGltf::Accessor;
    using CesiumGltf::BufferView;
    CesiumGltf::Model model;
    const size_t vertexCount = mesh.positions.size();
    const size_t vec3Bytes = vertexCount * sizeof(std::array<float, 3>);
    const size_t uvBytes = vertexCount * sizeof(std::array<float, 2>);
    const size_t indexBytes = mesh.indices.size() * sizeof(uint32_t);
    CesiumGltf::Buffer& buffer = model.buffers.emplace_back();
    buffer.cesium.data.resize(2 * vec3Bytes + uvBytes + indexBytes);
    std::byte* out = buffer.cesium.data.data();
    std::memcpy(out, mesh.positions.data(), vec3Bytes);
    std::memcpy(out + vec3Bytes, mesh.normals.data(), vec3Bytes);
    std::memcpy(out + 2 * vec3Bytes, mesh.uvs.data(), uvBytes);
    std::memcpy(out + 2 * vec3Bytes + uvBytes, mesh.indices.data(), indexBytes);
    buffer.byteLength = static_cast<int64_t>(buffer.cesium.data.size());

    const auto add_accessor = [&model](size_t offset, size_t length, int32_t target,
                                       int32_t componentType, const std::string& type,
                                       size_t count) {
        BufferView& view = model.bufferViews.emplace_back();
        view.buffer = 0;
        view.byteOffset = static_cast<int64_t>(offset);
        view.byteLength = static_cast<int64_t>(length);
        view.target = target;
        Accessor& accessor = model.accessors.emplace_back();
        accessor.bufferView = static_cast<int32_t>(model.bufferViews.size() - 1);
        accessor.componentType = componentType;
        accessor.type = type;
        accessor.count = static_cast<int64_t>(count);
        return static_cast<int32_t>(model.accessors.size() - 1);
    };
    const int32_t position =
        add_accessor(0, vec3Bytes, BufferView::Target::ARRAY_BUFFER, Accessor::ComponentType::FLOAT,
                     Accessor::Type::VEC3, vertexCount);
    const int32_t normal =
        add_accessor(vec3Bytes, vec3Bytes, BufferView::Target::ARRAY_BUFFER,
                     Accessor::ComponentType::FLOAT, Accessor::Type::VEC3, vertexCount);
    const int32_t uv =
        add_accessor(2 * vec3Bytes, uvBytes, BufferView::Target::ARRAY_BUFFER,
                     Accessor::ComponentType::FLOAT, Accessor::Type::VEC2, vertexCount);
    const int32_t indices = add_accessor(
        2 * vec3Bytes + uvBytes, indexBytes, BufferView::Target::ELEMENT_ARRAY_BUFFER,
        Accessor::ComponentType::UNSIGNED_INT, Accessor::Type::SCALAR, mesh.indices.size());

    // glTF requires POSITION bounds.
    std::vector<double> lo(3, std::numeric_limits<double>::max());
    std::vector<double> hi(3, std::numeric_limits<double>::lowest());
    for (const std::array<float, 3>& p : mesh.positions) {
        for (size_t k = 0; k < 3; ++k) {
            lo[k] = std::min(lo[k], static_cast<double>(p[k]));
            hi[k] = std::max(hi[k], static_cast<double>(p[k]));
        }
    }
    model.accessors[static_cast<size_t>(position)].min = lo;
    model.accessors[static_cast<size_t>(position)].max = hi;

    CesiumGltf::MaterialPBRMetallicRoughness& pbr =
        model.materials.emplace_back().pbrMetallicRoughness.emplace();
    pbr.metallicFactor = 0.0;
    pbr.roughnessFactor = 1.0;

    CesiumGltf::MeshPrimitive& primitive = model.meshes.emplace_back().primitives.emplace_back();
    primitive.attributes["POSITION"] = position;
    primitive.attributes["NORMAL"] = normal;
    primitive.attributes["TEXCOORD_0"] = uv;
    primitive.indices = indices;
    primitive.material = 0;
    primitive.mode = CesiumGltf::MeshPrimitive::Mode::TRIANGLES;

    model.nodes.emplace_back().mesh = 0;
    model.scenes.emplace_back().nodes.push_back(0);
    model.scene = 0;
    model.asset.version = "2.0";
    return model;
}

// Worker thread: one z14 tile from its DEM and the anchor's height on it (rebase_of).
TileLoadResult build_terrain_tile(const TileId& t14, uint32_t attempt, std::atomic<bool>& settled,
                                  const Rebase& rebase, double origin_height_m,
                                  std::atomic<uint64_t>& tiles_built,
                                  const std::shared_ptr<CesiumAsync::IAssetAccessor>& accessor,
                                  const CesiumGeospatial::Ellipsoid& ellipsoid) {
    if (!rebase.active()) {
        // A DEM failing for now goes back for another attempt; an empty or unreadable one, or the
        // last attempt, leaves no terrain there.
        if (rebase.retry && attempt < kMaxAttempts) {
            return TileLoadResult::createRetryLaterResult(accessor, nullptr);
        }
        settled = true;
        return success(TileEmptyContent{}, std::nullopt, accessor, ellipsoid);
    }
    settled = true;  // the next load of this tile starts counting attempts afresh
    const DemMesh mesh =
        mesh_dem(*rebase.dem, make_tile_frame(t14, origin_height_m), t14, *rebase.z_ref_m);
    double low = std::numeric_limits<double>::max();
    double high = std::numeric_limits<double>::lowest();
    for (const std::array<float, 3>& p : mesh.positions) {
        low = std::min(low, static_cast<double>(p[2]));
        high = std::max(high, static_cast<double>(p[2]));
    }
    ++tiles_built;
    return success(write_dem_gltf(mesh),
                   tile_region(t14, origin_height_m + low, origin_height_m + high, ellipsoid),
                   accessor, ellipsoid);
}

// The terrain on mapbox_tiling's z14 quadtree: each level-14 tile carries one tile's DEM mesh.
class MapboxTerrainLoader final : public Cesium3DTilesSelection::TilesetContentLoader {
public:
    MapboxTerrainLoader(std::shared_ptr<DemCache> dem, const GeoAnchor& anchor,
                        std::shared_ptr<std::atomic<uint64_t>> tiles_built)
        : dem_(std::move(dem)),
          anchor_{anchor.origin_lon_deg, anchor.origin_lat_deg},
          tilesBuilt_(std::move(tiles_built)),
          config_{std::string(), false, anchor.origin_height_m} {}

    CesiumAsync::Future<TileLoadResult> loadTileContent(const TileLoadInput& input) override {
        const auto* id = std::get_if<CesiumGeometry::QuadtreeTileID>(&input.tile.getTileID());
        if (id == nullptr || id->level != kLandmarkZoom) {
            return input.asyncSystem.createResolvedFuture(
                success(TileEmptyContent{}, std::nullopt, input.pAssetAccessor, input.ellipsoid));
        }
        const TileId t14{kLandmarkZoom, id->x, id->y};
        const double now = clock_s();
        if (attempts_.waiting(t14, now)) {
            return input.asyncSystem.createResolvedFuture(
                TileLoadResult::createRetryLaterResult(input.pAssetAccessor, nullptr));
        }
        const Attempt& a = attempts_.begin(t14, now);
        const uint32_t attempt = a.count;
        // TileLoadInput is a stack object of references: the continuation copies what it needs.
        return rebase_of(dem_, t14, anchor_)
            .thenInWorkerThread([t14, attempt, settled = a.settled,
                                 origin = config_.origin_height_m, built = tilesBuilt_,
                                 accessor = input.pAssetAccessor,
                                 ellipsoid = input.ellipsoid](const Rebase& rebase) {
                return build_terrain_tile(t14, attempt, *settled, rebase, origin, *built, accessor,
                                          ellipsoid);
            });
    }

    TileChildrenResult createTileChildren(const Tile& tile,
                                          const CesiumGeospatial::Ellipsoid& ellipsoid) override {
        std::vector<Tile> children = quadtree_children(tile, config_, ellipsoid);
        for (Tile& child : children) {
            const auto* id = std::get_if<CesiumGeometry::QuadtreeTileID>(&child.getTileID());
            if (id != nullptr && id->level == kLandmarkZoom) {
                child.setGeometricError(kTerrainGeometricErrorM);
            }
        }
        return {std::move(children), TileLoadResultState::Success};
    }

private:
    std::shared_ptr<DemCache> dem_;
    LonLatDeg anchor_;
    std::shared_ptr<std::atomic<uint64_t>> tilesBuilt_;
    LoaderConfig config_;   // the anchor height for quadtree_children; no token
    AttemptBook attempts_;  // main thread only: cesium calls loadTileContent there
};

}

CustomTileset make_mapbox_terrain_tileset(std::shared_ptr<DemCache> dem, const GeoAnchor& anchor,
                                          std::shared_ptr<std::atomic<uint64_t>> tiles_built) {
    auto loader =
        std::make_unique<MapboxTerrainLoader>(std::move(dem), anchor, std::move(tiles_built));
    std::unique_ptr<Tile> root = quadtree_root(*loader, anchor.origin_height_m);
    return CustomTileset{std::move(loader), std::move(root), true};
}

}
```

- [ ] **Step 6: Open the terrain tileset** (`overlume/src/mapbox_loader.cpp`)

The includes:

Replace

```cpp
#include <CesiumUtility/Gzip.h>
```

with

```cpp
#include <CesiumRasterOverlays/UrlTemplateRasterOverlay.h>
#include <CesiumUtility/Gzip.h>
#include <CesiumUtility/IntrusivePointer.h>
```

the terrain loader's factory, declared where it is called (the File map gives this task no
header):

Replace

```cpp
#include <variant>
```

with

```cpp
#include <variant>

namespace overlume::mapbox {

// mapbox_terrain_loader.cpp: the terrain-DEM quadtree (loader and root) over `dem`, counting its
// built tiles into `tiles_built`.
CustomTileset make_mapbox_terrain_tileset(std::shared_ptr<DemCache> dem, const GeoAnchor& anchor,
                                          std::shared_ptr<std::atomic<uint64_t>> tiles_built);

}
```

the buildings loader requests nothing without `buildings` in the layer list:

Replace

```cpp
        if (id == nullptr || id->level != kLandmarkZoom) {
            // Levels 0-13 hold nothing; cesium may unload an empty tile and ask for it again.
```

with

```cpp
        if (id == nullptr || id->level != kLandmarkZoom || !buildings_) {
            // Levels 0-13 hold nothing, and no level does when the layer list lacks `buildings`;
            // cesium may unload an empty tile and ask for it again.
```

Replace

```cpp
    // `dem` is the source's DemCache when the layer list carries `terrain`, else null.
    MapboxBuildingsLoader(std::shared_ptr<const LoaderConfig> config,
                          std::shared_ptr<LoaderState> state, std::shared_ptr<DemCache> dem,
                          LonLatDeg anchor)
        : config_(std::move(config)),
          state_(std::move(state)),
          dem_(std::move(dem)),
          anchor_(anchor) {}
```

with

```cpp
    // `dem` is the source's DemCache when the layer list carries `terrain`, else null; without
    // `buildings` in the list the tileset stays empty and requests nothing.
    MapboxBuildingsLoader(std::shared_ptr<const LoaderConfig> config,
                          std::shared_ptr<LoaderState> state, std::shared_ptr<DemCache> dem,
                          LonLatDeg anchor, bool buildings)
        : config_(std::move(config)),
          state_(std::move(state)),
          dem_(std::move(dem)),
          anchor_(anchor),
          buildings_(buildings) {}
```

Replace

```cpp
    // The map origin, whose DEM height is z = 0.
    LonLatDeg anchor_;
```

with

```cpp
    // The map origin, whose DEM height is z = 0.
    LonLatDeg anchor_;
    // The layer list carries `buildings`.
    bool buildings_ = true;
```

and the end of `open_mapbox_source` builds the terrain tileset, its overlay, and attaches them
(the root's line as Task 3's Edit 13 left it):

Replace

```cpp
        std::move(state), dem, LonLatDeg{anchor.origin_lon_deg, anchor.origin_lat_deg});
    std::unique_ptr<Tile> root = quadtree_root(*loader, anchor.origin_height_m);
    // Buildings only: never follows terrain, never replaces the ground plane.
    return std::make_unique<StreamingEnvironmentSource>(
        externals, 0, std::string(), std::string(), spec.fallback_dir, anchor, std::move(counting),
        spec.materials_original, false, 0.3, false, 2.0, spec.brightness, spec.radius_m,
        CustomTileset{std::move(loader), std::move(root), true});
}
```

with

```cpp
        state, dem, LonLatDeg{anchor.origin_lon_deg, anchor.origin_lat_deg}, spec.buildings);
    std::unique_ptr<Tile> root = quadtree_root(*loader, anchor.origin_height_m);
    // With the terrain layer the shared transform follows the terrain under the ego and the
    // terrain owns the ground; buildings alone do neither.
    const bool followTerrain = spec.terrain;
    const bool replacesGround = spec.terrain;
    auto source = std::make_unique<StreamingEnvironmentSource>(
        externals, 0, std::string(), std::string(), spec.fallback_dir, anchor, std::move(counting),
        spec.materials_original, followTerrain, 0.3, replacesGround, 2.0, spec.brightness,
        spec.radius_m, CustomTileset{std::move(loader), std::move(root), true});
    if (!spec.terrain) return source;

    // The terrain tileset: the source's externals (the counting accessor, so imagery failures
    // count toward network loss; the redacting logger, which masks the token in cesium's failure
    // lines; the task processor and the credit system), renderer resources of its own
    // (terrain.mat and the overlay callbacks), area-selected as the buildings are.
    auto resources = std::make_shared<StreamRendererResources>(
        compute_ecef_to_map(anchor), anchor.origin_height_m, false, spec.brightness,
        StreamRendererResources::Kind::Terrain);
    Cesium3DTilesSelection::TilesetExternals terrainExternals = externals;
    terrainExternals.pPrepareRendererResources = resources;
    Cesium3DTilesSelection::TilesetOptions options;
    options.maximumScreenSpaceError = kStreamMaxSseErr;
    options.preloadSiblings = false;
    CustomTileset terrainTiles =
        make_mapbox_terrain_tileset(dem, anchor, state->terrain_tiles_built);
    auto terrain = std::make_unique<Cesium3DTilesSelection::Tileset>(
        terrainExternals, std::move(terrainTiles.loader), std::move(terrainTiles.root), options);
    if (spec.imagery_satellite) {
        // The token rides in the query string, which cesium's template substitution leaves alone.
        CesiumRasterOverlays::UrlTemplateRasterOverlayOptions imagery;
        imagery.maximumLevel = kSatelliteMaxLevel;
        imagery.tileWidth = kSatelliteTilePx;
        imagery.tileHeight = kSatelliteTilePx;
        imagery.credit = kSatelliteCredit;
        terrain->getOverlays().add(
            CesiumUtility::IntrusivePointer<const CesiumRasterOverlays::RasterOverlay>(
                new CesiumRasterOverlays::UrlTemplateRasterOverlay(
                    "mapbox-satellite", satellite_overlay_template() + query, {}, imagery)));
    }
    source->attach_terrain(std::move(terrain), std::move(resources));
    return source;
}
```

- [ ] **Step 7: Run them and see them pass**

```bash
cd "$WORKTREE"
flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock cmake -S overlume -B overlume/build > /dev/null \
    && echo reconfigured \
    && CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
overlume/build/test_mapbox_terrain
env -u OVERLUME_LIVE_MAPBOX overlume/build/test_mapbox_buildings | tail -5
```

Expected: `reconfigured` (a new `src/mapbox_*.cpp`), a clean build, the 7 `MapboxTerrain` tests
each `[       OK ]` and `[  PASSED  ] 7 tests.` (`FailingDemIsAskedAgainOnTheNextAttempt` takes
about 1.1 s of its own wait); then Task 9's ending again (`[  PASSED  ] 34 tests.`, two
`MapboxLive` skips): its `buildings,terrain` tests now run with a terrain tileset beside them,
which shares their DEM requests and their retries' 1 s window, so they still see one DEM request
per tile and attempt. On a machine without GPU/EGL run on the GPU rig.

- [ ] **Step 8: Format**

```bash
cd "$WORKTREE"
git add overlume/src/mapbox_terrain_loader.cpp overlume/src/mapbox_loader.cpp \
    overlume/src/mapbox_test_hooks.hpp overlume/tests/test_mapbox_terrain.cpp
tools/check_format.sh --fix && tools/check_format.sh && git diff --stat && tools/check_spdx.sh
```

Expected: as Task 9's Step 7: `reformatted N files`, `PASS (N files, clang-format 23.1.1)`, an
empty `git diff --stat` and `check_spdx.sh: PASS (…)`.

- [ ] **Step 9: Revert checks (do not commit)**: the rows, the layer gate, the follow flag, the
  retry; then the ground flag and the logger

```bash
cd "$WORKTREE"
sed -i 's/^    return url.replace(url.find("{y}"), 3, "{reverseY}");$/    return url;  \/\/ REVERT CHECK/' \
    overlume/src/mapbox_loader.cpp
sed -i 's/^        if (id == nullptr || id->level != kLandmarkZoom || !buildings_) {$/        if (id == nullptr || id->level != kLandmarkZoom) {  \/\/ REVERT CHECK/' \
    overlume/src/mapbox_loader.cpp
sed -i 's/^    const bool followTerrain = spec.terrain;$/    const bool followTerrain = false;  \/\/ REVERT CHECK/' \
    overlume/src/mapbox_loader.cpp
sed -i 's/^        if (rebase.retry \&\& attempt < kMaxAttempts) {$/        if (false \&\& rebase.retry \&\& attempt < kMaxAttempts) {  \/\/ REVERT CHECK/' \
    overlume/src/mapbox_terrain_loader.cpp
grep -c 'REVERT CHECK' overlume/src/mapbox_loader.cpp overlume/src/mapbox_terrain_loader.cpp
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_mapbox_terrain
overlume/build/test_mapbox_terrain | tail -8
git checkout -- overlume/src/mapbox_loader.cpp overlume/src/mapbox_terrain_loader.cpp
sed -i 's/^    const bool replacesGround = spec.terrain;$/    const bool replacesGround = false;  \/\/ REVERT CHECK/' \
    overlume/src/mapbox_loader.cpp
sed -i 's/^    terrainExternals.pPrepareRendererResources = resources;$/&\n    terrainExternals.pLogger = spdlog::default_logger();  \/\/ REVERT CHECK/' \
    overlume/src/mapbox_loader.cpp
grep -c 'REVERT CHECK' overlume/src/mapbox_loader.cpp
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_mapbox_terrain
overlume/build/test_mapbox_terrain | tail -6
git checkout -- overlume/src/mapbox_loader.cpp
grep -c 'REVERT CHECK' overlume/src/mapbox_loader.cpp overlume/src/mapbox_terrain_loader.cpp || true
git diff --stat
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_mapbox_terrain
```

Expected: the first `grep` prints `overlume/src/mapbox_loader.cpp:3` and
`overlume/src/mapbox_terrain_loader.cpp:1`; the first run ends

```
[  FAILED  ] 4 tests, listed below:
[  FAILED  ] MapboxTerrain.TerrainTileBuildsFromItsDem
[  FAILED  ] MapboxTerrain.FailingDemIsAskedAgainOnTheNextAttempt
[  FAILED  ] MapboxTerrain.SatelliteDrapesXyzTilesOnTheTerrain
[  FAILED  ] MapboxTerrain.TerrainLayerReplacesTheGroundPlane

 4 FAILED TESTS
```

each on its own revert: the 17 buildings requests of `mapbox://terrain` beside the DEM; a terrain
tile that settles empty after the 503 and never asks again; TMS rows that nobody serves, so the
requests leave the served XYZ set and no raster attaches; an offset that stays 0 although the
terrain still owns the ground (Task 11 samples it without `follow_terrain`).
`OneDemFeedsBothLoaders`, `MissingDemLeavesNoTerrainTile` and
`FailedSatelliteTileLogsThroughTheRedactingLogger` pass, as they should under these four reverts
(the z0 tile has row 0 in both schemes). The second `grep` prints `2`; the second run ends

```
[  FAILED  ] 2 tests, listed below:
[  FAILED  ] MapboxTerrain.FailedSatelliteTileLogsThroughTheRedactingLogger
[  FAILED  ] MapboxTerrain.TerrainLayerReplacesTheGroundPlane

 2 FAILED TESTS
```

on cesium's 401 line printed by spdlog's default logger (the fake token in the clear on stdout,
never in the captured redacting log), and on a terrain that never provides the ground
(`provides_ground` needs `replaces_ground`), so the clay plane stays. After the checkouts the third
`grep` prints `…:0` for both, `git diff --stat` prints nothing, and the rebuild leaves the binary
current.

- [ ] **Step 10: Gate** (foreground, Bash timeout 600000 ms)

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
ROS_DOMAIN_ID=85 CI_VISUAL_MODE_DOMAIN_ID=85 \
    flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock tools/ci_visual_mode.sh
```

Expected: six `PASS` lines, `OVERALL: PASS`, exit 0. The library ctest count is 7 higher than at
Task 9's gate plus the entries Step 1's merge brought in (Tasks 5 and 11, and any other lane
merged into `mapbox-environment` since the fork), every `MapboxTerrain.*` and `MapboxBuildings.*`
entry among the ok ones; the goldens all `ok`, none moved; the node, WS bridge and example counts
are Task 9's gate's plus whatever Step 1's merge brought in. Exit 75: run the same command again.

- [ ] **Step 11: Commit**

```bash
cd "$WORKTREE"
git add overlume/src/mapbox_terrain_loader.cpp overlume/src/mapbox_loader.cpp \
    overlume/src/mapbox_test_hooks.hpp overlume/tests/test_mapbox_terrain.cpp
git status --short
git commit -F - <<'EOF'
feat(mapbox): terrain tileset from the terrain-DEM, satellite imagery

A mapbox:// layer list with `terrain` now opens a second cesium tileset
beside the buildings. Its loader walks the same z14 quadtree with the
same attempt rules (mapbox_tiling), builds each tile from its DEM through
the source's DemCache (one request, shared with the buildings), meshes it
with skirts into a glTF with positions, normals and UVs, and hands it to
the terrain renderer resources (terrain.mat and the overlay callbacks)
through attach_terrain. A DEM failing for now is asked again on the next
attempt; an empty or unreadable one leaves the tile empty at once. z14
tiles carry geometric error 2, so the area view refines cesium's
upsampled children to z16 near the ego, whose imagery is z18.
imagery=satellite adds Mapbox Satellite as a UrlTemplateRasterOverlay
(512 px, up to z18, credit "© Maxar") with cesium's {reverseY} in the
template: its {y} counts rows from the south, Mapbox's from the north.
The terrain tileset shares the source's counting accessor and redacting
logger, so imagery failures count toward network loss and cesium's own
failure lines reach the log with the token masked. With the terrain
layer the source follows the terrain and the terrain owns the ground; a
layer list without `buildings` requests no buildings.

MapboxSourceStats gains terrain_tiles_built and the test hooks
mapbox_satellite_url. MapboxTerrain (7 tests): a terrain tile from its
DEM alone, one DEM for both loaders, a 404 settling at once, a 503 asked
again on the next attempt, satellite tiles in XYZ rows with the token
only in the query and Maxar in the credits, a failed satellite tile
logged by cesium through the redacting logger with the terrain tile
kept, and the ground plane hidden by a terrain the source follows but
not by buildings alone.

Claude-Session: <the executing session's URL>
EOF
```

Expected: `git status --short` lists `A ` for the two new files and `M ` for the other two; the
commit succeeds. Then report Task 10 committed: lane 5 is ready to merge.

### Task 11: Second tileset, ion terrain and imagery, credits API

**Lane:** 6. **Depends on:** Tasks 2, 4 and 5 merged into `mapbox-environment`, as head.md's Lanes
table says (Task 2's `quantized_mesh_fixture.hpp` and `imagery_fixture.hpp`; Task 4's
`IonSpec::terrain_asset`, `IonSpec::imagery_asset` and its `ion://… refused:` line; Task 5's
`StreamRendererResources::Kind`, `OverlayCounters`, `overlay_counters()`, `sync_terrain_color()`,
`OverlayProbe` and `environment_stream_overlay_probe`). The lane forks from that head. The Replace
blocks quote the code as Tasks 4 and 5 leave it (lane-2.md and lane-3.md).

**Rulings this lane follows** (rulings.md):
1. **The node's once-a-second credits read is Task 12's** (R5; head.md's File map gives the node's
   `environment_credits_` read to Task 12, after this task's function exists). This task ships the
   library half, `environment_source_credits`, and touches no node file.
2. **No per-tile imagery failure line** (R2; head.md, "Texts other tasks match verbatim"; spec
   section 5, Failure handling, the imagery row). `RasterOverlayOptions::loadErrorCallback` fires
   only when a tile provider cannot be created, never for a tile (API notes). cesium logs each
   failed raster tile itself, `Received response code {} for image {url}.`, through the tileset's
   logger, which is the source's redacting one: the token is masked, the host shows. This task adds
   no line for it.

**Files:**
- Modify: `overlume/include/overlume/scene.h` (append after `uint32_t get_quality(VisualRenderer*);`,
  283, before the namespace's closing `}`)
- Modify: `overlume/src/environment_stream.hpp` (the include block, `#include <unordered_map>` 22;
  `MapFixtureAssetAccessor`: after
  `void release_held();` 209 and after `std::vector<std::string> requested_;` 248;
  `StreamRendererResources`: Task 5's `overlay_counters()`; `StreamingEnvironmentSource`: after
  `void force_fall_back_for_testing(VisualRenderer& r) { fall_back(r); }` 369 and Task 5's
  `overlay_counters()` below it, after `void fall_back(VisualRenderer& r);` 375, after
  `std::unique_ptr<EnvironmentSource> fallbackSource_;` 409; line numbers at 427f5d8, before Tasks
  4 and 5)
- Modify: `overlume/src/environment_stream.cpp` (includes after
  `#include <CesiumGltfWriter/GltfWriter.h>` 26; `build_externals` 378-404;
  `MapFixtureAssetAccessor::hold`/`release_held`/`respond`/`answer` 544-593; after
  `StreamingEnvironmentSource::provides_ground` 666-668; `update` 889-893;
  `maybe_trigger_terrain_sample` 951; `update_terrain_transform` 1025-1027;
  `synthesize_view_and_pump` 1098-1101 and 1118-1164; `teardown` 1166-1200; `set_visible` 1208-1217;
  `scene_membership_count` 1225-1235; `open_streaming_environment_source` 1265-1285 as Task 4 leaves
  it; after Task 5's `environment_stream_overlay_probe` definition in `namespace overlume::testing`)
- Modify: `overlume/src/environment_test_hooks.hpp` (immediately after Task 5's
  `install_fixture_terrain_with_file_imagery` declaration)
- Modify: `overlume/src/environment.cpp` (after `environment_source_state`, 229-232)
- Test: `overlume/tests/test_environment_stream.cpp` (include block 7-27; the Task 11 block appended
  at the end of the file, after Task 5's tests (Task 4's sit right after the existing parse tests);
  suite `EnvironmentStreamTerrain`, 8 tests)

**Interfaces:**
- Consumes:

```cpp
// overlume/tests/quantized_mesh_fixture.hpp, overlume/tests/imagery_fixture.hpp (Task 2)
namespace overlume::testing::qmesh {
struct QuantizedMeshOptions { uint32_t grid = 8; double min_height_m = 0.0; double max_height_m = 0.0; };
std::vector<uint8_t> encode_quantized_mesh(uint32_t level, uint32_t x, uint32_t y, const QuantizedMeshOptions& o);
std::string layer_json(const std::vector<std::array<uint32_t, 3>>& available_level_x_y);
std::string ion_terrain_endpoint_json(const std::string& layer_json_base_url);  // type TERRAIN
std::string ion_imagery_endpoint_json(const std::string& tms_base_url);         // type IMAGERY, TMS url
std::string tilemapresource_xml(uint32_t tile_size);
}
namespace overlume::testing::imagery {
std::vector<uint8_t> flat_png(uint32_t size, uint8_t r, uint8_t g, uint8_t b);
}
// environment_stream.cpp, IonSpec (Task 4)
std::optional<int64_t> terrain_asset;  // terrain=<id>|off
std::optional<int64_t> imagery_asset;  // imagery=<id>|off; set without terrain_asset -> parse fails
// environment_stream.cpp, open_streaming_environment_source (Task 4): a spec that does not parse
// logs "ion://{} refused: …" once (EnvironmentStream.ParseIonSpecRefusalLogsOneLine, token unset)
// environment_stream.hpp (Task 5)
class StreamRendererResources {
public:
    enum class Kind : uint8_t { Content, Terrain };
    StreamRendererResources(/* existing args, all defaulted */, Kind kind = Kind::Content);
    struct OverlayCounters { uint32_t rasters_attached = 0, rasters_detached = 0, textures_alive = 0, textures_freed = 0; };
    OverlayCounters overlay_counters() const { return counters_; }
    void sync_terrain_color();  // every update(), for each resources (lane-3.md, Task 5's API notes)
private:
    std::array<float, 3> terrainColor_{-1.0f, -1.0f, -1.0f};  // last pushed by sync_terrain_color
};
class StreamingEnvironmentSource {
public:
    StreamRendererResources::OverlayCounters overlay_counters() const;  // Task 11 points it at the terrain
};
// environment_test_hooks.hpp (Task 5)
struct OverlayProbe { uint32_t rasters_attached, rasters_detached, textures_alive, textures_freed; };
OverlayProbe environment_stream_overlay_probe(overlume::VisualRenderer* r);  // reads overlay_counters()
// existing: build_externals, make_redacting_logger, compute_ecef_to_map, parse_ion_spec,
// SimpleTaskProcessor, MapFixtureAssetAccessor, kStreamMaxSseErr (environment_stream.hpp/.cpp);
// environment_stream_provides_ground, renderer_ground_plane_in_scene,
// environment_stream_force_fall_back, environment_terrain_offset_z, captured_cesium_log_text,
// environment_loaded_chunk_count (environment_test_hooks.hpp)
```

- Produces, verbatim from the contract:

```cpp
// overlume/include/overlume/scene.h (free function, no kSceneVersion bump)
/// Plain-text credits of the open streaming source's raster overlays ("; " separated, HTML
/// stripped), written to `out` (NUL-terminated, truncated to `cap`); returns the length needed.
/// 0 and an empty string when no source or no overlay is open.
uint32_t environment_source_credits(VisualRenderer*, char* out, uint32_t cap);
// overlume/src/environment_stream.hpp, StreamingEnvironmentSource
void attach_terrain(std::unique_ptr<Cesium3DTilesSelection::Tileset> terrain,
                    std::shared_ptr<StreamRendererResources> terrain_resources);
bool has_terrain() const;
std::string credits() const;
// overlume/src/environment_test_hooks.hpp
bool environment_stream_has_terrain_tileset(overlume::VisualRenderer* r);
uint64_t environment_terrain_tiles_loaded(overlume::VisualRenderer* r);  // terrain tiles in the scene
```

  Beyond the contract, append-only: the ion fixture helpers the contract names for Task 11
  (`IonFixtureNetwork`, `make_ion_fixture_network`, `ion_fixture_serve`, `ion_fixture_serve_prefix`,
  `ion_fixture_hold_prefix`, `ion_fixture_release_held`, `ion_fixture_fail_all_transport`,
  `ion_fixture_requested_urls`, `install_fixture_ion_source`, `ion_fixture_terrain_probe`), one more
  fixture hook (`ion_fixture_terrain_color`) and one stream hook,
  `double environment_terrain_tile_lift_z(overlume::VisualRenderer* r)`, all in Step 2;
  `MapFixtureAssetAccessor::serve_prefix`, `hold_prefix` and `fail_all_transport`;
  `StreamRendererResources::terrain_color_for_testing()`; and on `StreamingEnvironmentSource`:
  `terrain_resources()`, `terrain_scene_count(VisualRenderer&)`, `terrain_tile_lift_z(VisualRenderer&)`. Task 5's
  `StreamingEnvironmentSource::overlay_counters()` now reads the terrain resources when a terrain
  tileset is attached (Task 5's own fixture, which opens its terrain as the content tileset, still
  reads the content resources), so `environment_stream_overlay_probe` keeps Task 5's body.

  What Task 10 (the Mapbox terrain) may rely on: `build_externals` now sets `pCreditSystem`, so a
  terrain `Tileset` built from a copy of the externals `build_externals` returned, with
  `pPrepareRendererResources` set to a `StreamRendererResources(compute_ecef_to_map(anchor),
  anchor.origin_height_m, spec.materials_original, spec.brightness,
  StreamRendererResources::Kind::Terrain)`, shares the source's accessor (its failures count toward
  network loss), task processor, logger and credit system, and its `UrlTemplateRasterOverlay`'s
  credit ("© Maxar") reaches `credits()`. Overlays are added before `attach_terrain`. The terrain
  takes the clay plane's place only when the source is constructed with `replaces_ground = true`
  (today's `open_mapbox_source` passes `false`; with `terrain` in the layer list Task 10 passes the
  ion follow-terrain defaults, follow-terrain on, `ground_bias` 0.3, `replaces_ground` on,
  `max_tilt_deg` 2.0, R9): such a source samples under the ego even without
  `follow_terrain`. A failed Mapbox Satellite tile adds no line of ours: cesium's own
  `Received response code … for image <url>.` reaches the shared redacting logger (ruling 2).

**How it works** (spec section 5; read with Steps 5-8):
- **Two tilesets.** `attach_terrain` stores the terrain tileset and its renderer resources. Each frame
  `update()` points both resources at the renderer, pushes the theme's `terrain_color` to both
  (Task 5's `sync_terrain_color()`) and drains both deferred-free lists, then
  `synthesize_view_and_pump` updates the content tileset and then the terrain tileset with the same
  views, calls `loadTiles` on both, dispatches the one `AsyncSystem` once, and syncs each tileset's
  rendered tiles into the scene with the radius rule of today (`sync_scene`, today's loop moved
  unchanged and run once per tileset). Terrain assets are tracked in `terrainInScene_`, so
  `loaded_count()` and `scene_membership_count()` keep meaning "buildings".
- **Ground.** `has_terrain()` is true while a terrain tileset is attached, unless its root-available
  event is ready with no root (layer.json refused, asset missing): the source then runs on its content
  tileset as today. The ground sample under the ego targets the terrain when `has_terrain()`, and a
  terrain that replaces the ground is sampled even without `follow_terrain` (the spec's
  `provides_ground` rule needs the sample); the transform still moves only with `follow_terrain`,
  and it is applied to both resources' roots at once, so terrain and buildings lift together.
- **Teardown.** Both tilesets' assets leave the scene, both free lists drain, then the terrain
  tileset is destroyed before the content tileset, and the pump runs until both destruction events
  are ready (the same bound). Destroying a tileset unloads every tile, and cesium detaches each
  tile's raster overlay tiles before freeing the tile (TilesetContentManager.cpp:1416-1420), so the
  overlays are detached before any tile goes, as the spec orders, without a removal loop of ours.
  `fall_back` runs this teardown, so network loss takes both tilesets down.
- **ion.** `open_ion_source` serves `open_streaming_environment_source` (curl, `CESIUM_ION_TOKEN`) and
  `install_fixture_ion_source` (a fixture network, a given token). With `terrain=<id>` it builds the
  terrain tileset from the ion asset with the content tileset's externals (renderer resources
  swapped for `Kind::Terrain` ones), a `loadErrorCallback` that logs
  `environment source: terrain tileset did not open (HTTP {}) -- continuing without terrain`, and,
  with `imagery=<id>`, an `IonRasterOverlay` on it.
- **Credits.** `credits()` references every activated overlay's tile-provider credits
  (`addCredits`, the same call cesium's view group makes each frame) in a fresh `CreditReferencer`,
  takes the credit system's unfiltered snapshot and keeps the credits that referencer holds (a
  `CreditReferencer` cannot list its credits), strips their HTML, drops empty and repeated texts, and
  joins them with `"; "`. Tileset credits (ion attributions of the content and terrain assets,
  layer.json's `attribution`, glTF copyrights) are never in the referencer, so they never appear.
  `environment_source_credits` copies that text like `snprintf`; without cesium it is a stub in
  `environment.cpp`.
- **Node.** Not this task (ruling 1): the node's once-a-second read of
  `environment_source_credits` is Task 12's.

**API notes** (cesium-native 0.64, paths under `overlume/build/_deps/cesium-native-src/`; Overlume
line numbers at 427f5d8, before Tasks 4 and 5):
- `Cesium3DTilesSelection/include/Cesium3DTilesSelection/Tileset.h:81-86` the ion constructor
  (`ionAssetEndpointUrl` defaults to `https://api.cesium.com/`), `:129` `getRootTileAvailableEvent()`
  (non-const), `:158`/`:164` `getExternals()`, `:206` `getRootTile() const`, `:218-221` `getOverlays()`,
  `:373` `sampleHeightMostDetailed`, `:382` `getDefaultViewGroup()`, `:410` `updateViewGroup`, `:446`
  `loadTiles`. `src/Tileset.cpp:126-136` the destructor marks the manager destroyed and
  `unloadAll()`s; `:401-403` `updateViewGroup` returns at once without a root; `:455-467` `loadTiles`
  fails pending height requests once the root event is ready with no root.
- `TilesetExternals.h:56` `pCreditSystem` (null unless set; `build_externals`,
  `environment_stream.cpp:378-404`, never set it). `TilesetOptions.h:57` `maximumScreenSpaceError`,
  `:211` `loadErrorCallback`; `TilesetLoadFailureDetails.h:52` `uint16_t statusCode{200}`.
- `src/TilesetContentManager.cpp:2215-2256` `propagateTilesetContentLoaderResult`: on errors it calls
  `loadErrorCallback` with the loader's `statusCode` and returns; on success it calls
  `this->_externals.pCreditSystem->createCredit` for every loader credit **without a null check**
  (:2244-2250). `LayerJsonTerrainLoader.cpp:168-173` always yields one credit per layer (an empty
  string when layer.json has no `attribution`), so an ion terrain tileset without a credit system
  dereferences null: the credit system in `build_externals` is required, not only for `credits()`.
  `:721` (custom loader), `:851`, `:913` (loader factory, which the ion constructor uses) resolve the
  root-available promise after the result propagated, root or not; `:1416-1420` detach a tile's
  rasters (`detachRasterInMainThread`) before freeing it; `:1477-1483` `unloadAll`.
- `src/CesiumIonTilesetLoader.cpp:402` the endpoint URL `{}v1/assets/{}/endpoint?access_token={}`;
  `:287-295` a `TERRAIN` endpoint resolves `layer.json` against its `url` and caches the endpoint in
  the process-wide `endpointCache` (`:50`) keyed by that URL, token included; the first open
  (cache miss, `:495-520`, then `mainThreadHandleEndpointResponse` `:194`) has no 401 retry,
  `refreshTokenIfNeeded` (`:525-547`) runs only on a cache hit (`:423-458`), so a 401 layer.json
  fires the load-error callback once. `LayerJsonTerrainLoader.cpp:560-592` a non-2xx `layer.json` sets `statusCode` (401 reaches the
  callback); `:143-163` a geographic layer.json gets two level-0 roots, x 0 (west) and 1 (east);
  `:663-700` tile URLs resolve the first `tiles` template against layer.json's URL and add
  `extensions=` only for listed extensions; `:1085-1102` children exist only where `available` lists
  one.
- `Cesium3DTilesSelection/include/Cesium3DTilesSelection/RasterOverlayCollection.h:138-140`
  `getActivatedOverlays() const`, `:148` `add(const IntrusivePointer<const RasterOverlay>&)`;
  `CesiumRasterOverlays/include/CesiumRasterOverlays/ActivatedRasterOverlay.h:128-132`
  `getTileProvider()` (null until the provider exists); `RasterOverlayTileProvider.h:267` virtual
  `addCredits(CreditReferencer&)`; `IonRasterOverlay.h:40-45` the constructor; `IonRasterOverlay.cpp:147-170`
  `addCredits` adds the ion attributions (`:640-651`, created only with a credit system) and the
  aggregated provider's; `:391` `getEndpointCache()` is process-wide, keyed by URL with the token;
  `:460-506` an endpoint without `externalType` becomes a `TileMapServiceRasterOverlay` on its `url`.
  `TileMapServiceRasterOverlay.cpp:296-306` fetches the URL itself, then (`:261-280`) its
  `tilemapresource.xml` when that is not a TMS document; `:142-150` tiles at
  `<url><href>/<x>/<y>.<ext>`. `UrlTemplateRasterOverlay.cpp:62-67` creates its credit only with a
  credit system. `CesiumUtility/include/CesiumUtility/IntrusivePointer.h:23` converts from `T*`
  implicitly.
- `CesiumRasterOverlays/src/RasterOverlay.cpp:116-125`: `RasterOverlayOptions::loadErrorCallback`
  fires only when a tile provider cannot be created; a raster tile's HTTP failure is only logged
  (`ActivatedRasterOverlay.cpp:194-201`), as `Received response code {} for image {url}.`
  (`RasterOverlayTileProvider.cpp:183-191`). `ActivatedRasterOverlay.cpp:253-330` `doLoad` keeps the
  raster tile and the activated overlay alive across the request, runs `prepareRasterInLoadThread` on
  a worker and the rest in a main-thread continuation; `RasterOverlayTile.cpp:37-63` the destructor
  calls `freeRaster` with the load-thread result when the tile never reached `Done`.
- `CesiumUtility/include/CesiumUtility/CreditReferencer.h:37` constructor from the system, `:102`
  `addCreditReference`, `:114` `isCreditReferenced` (no listing); `CreditSystem.h:224` `getHtml`,
  `:268` `getSnapshot`; `src/CreditSystem.cpp:190-233` with `CreditFilteringMode::None` every credit
  with a reference is in `currentCredits`; `:59-77` `createCredit` reuses a record with the same HTML
  and source. `src/CreditReferencer.cpp` `addCreditReference` adds the reference to the system at once;
  the destructor releases it. All of these run on the main thread, as `credits()` does.
- `src/TilesetHeightQuery.cpp:95` the sample ray starts 0.007 × the maximum radius (44.6 km) above the
  ellipsoid and keeps any triangle hit (`:405-412`), so a level-0 tile's chord, tens of kilometres
  below the surface under the ego, is a hit.
- Overlume: `environment_stream.hpp:198-249` `MapFixtureAssetAccessor` (exact keys without the query,
  `hold`); `environment_stream.cpp:560-593` `respond`/`answer`; `:666-668` `provides_ground`;
  `:905-989` the sample (`:951`); `:1005-1031` the transform; `:1075-1164` the pump and the scene
  sync; `:1166-1200` teardown; `:1263-1287` the ion opener; `:1289-1384` the fixture installers this
  task's hook copies; `:297-325` the redacting sink keeps up to 64 KiB of log text per process (ctest
  runs each test in its own process). `environment.cpp:186-213` dispatch, `:229-232`
  `environment_source_state`. `renderer.cpp:1115-1124` the environment updates, then
  `provides_ground()` takes the clay plane out or puts it back in the same frame.
  `CMakeLists.txt:59` C++17 by default, `:237-243` `environment_stream.cpp` is C++20 in
  `overlume_stream` (so `environment.cpp`, C++17 in `overlume_obj` with `OVERLUME_ENABLE_CESIUM=1`,
  `:281-286`, cannot include `environment_stream.hpp`: the credits function is defined in
  `environment_stream.cpp`, with a stub in `environment.cpp` only when cesium is off), `:318-319`
  `--allow-multiple-definition` on Linux (so the stub must be compiled out, not merely shadowed),
  `:585-590` `test_environment_stream.cpp` is cesium-only and C++17; `golden.cpp:6-8`, linked into
  every test, holds the stb implementations Task 2's fixtures use.
- Fixtures: `overlume/tests/fixtures/environment_tiles_fixture_0` (tileset.json and three b3dm,
  buildings only: `GroundPlaneKeptWhenTilesetIsBuildingsOnly`, `test_environment_stream.cpp:1019`);
  `environment_tiles_fixture_fallback_0` (16 leaves, byte-identical copies of `tile_root.b3dm`, its
  PROVENANCE.md); `environment_test_town_0` (baked chunks within 300 m of map (0, 0)); all at
  `kFixtureAnchor` (25.0803, 55.3910), in the eastern hemisphere's level-0 terrain tile.
- `src/TilesetSelection.cpp:611-627` `renderLeaf` lists a leaf for rendering whether or not its
  rasters are ready (`Tile::isRenderable`, `Tile.cpp:218-239`, only steers kicking), so a level-0
  terrain leaf whose raster request is held still enters the scene.
- Checked while drafting: the code of Steps 2-8 compiles (`-fsyntax-only`) at 427f5d8 with
  stand-ins for Task 2's headers, Task 4's two `IonSpec` fields and Task 5's `Kind`/
  `OverlayCounters`/`OverlayProbe`, under the `overlume_stream`, `overlume_obj` (with and without
  `OVERLUME_ENABLE_CESIUM`) and `test_environment_stream` flags, and it is clang-format 23.1.1's
  output; each sed in Step 11 changes exactly one line of that source. **Not checked: any run.** The
  600-frame budgets, the follow-terrain lift's 1 m tolerance on float32 transforms near 6.4e6 m, and
  the switch test's premise that finer imagery is still being requested once the first textures
  exist (upsampled children under a TMS overlay listing levels 0-18) have never executed; Step 9
  says what to do if one of them misses.

- [ ] **Step 1: Create the lane worktree** (the orchestrator, once Tasks 2, 4 and 5 are merged)

```bash
git -C "$INTEGRATION" worktree add "$INTEGRATION-l6" -b terrain-env/l6 mapbox-environment
cd "$INTEGRATION-l6"
mkdir -p /tmp/overlume-l6
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock \
    cmake --toolchain "$PWD/overlume/cmake/toolchain-clang-libcxx.cmake" -S overlume \
    -B overlume/build -DOVERLUME_ENABLE_CESIUM=ON -DCMAKE_C_COMPILER_LAUNCHER=ccache \
    -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DOVERLUME_TEST_TMP_DIR_RUNTIME=/tmp/overlume-l6
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j16
ros/colcon_build.sh
```

Expected: the configure ends `-- Build files have been written to: …/overlume/build`, the build and
`colcon build` finish without errors. Exit 75 means the lock wait ran out: run it again. From here on
`$WORKTREE` is `$INTEGRATION-l6`. Record the six counts of the orchestrator's gate after the last of
these merges (library ctest total, ok and skipped; node; WS bridge; goldens; examples): Step 12
compares against them. Record one more number for Step 9:

```bash
cd "$WORKTREE"
ctest --test-dir overlume/build -N -R 'EnvironmentStream|EnvSourceCapture|TokenRedaction|Mapbox' \
    | tail -n 1
```

Expected: `Total Tests: N`; Step 9 expects N + 8.

- [ ] **Step 2: Declare Task 11's interfaces**

In `overlume/include/overlume/scene.h`, replace

```cpp
uint32_t get_quality(VisualRenderer*);

}
```

with

```cpp
uint32_t get_quality(VisualRenderer*);

/// Plain-text credits of the open streaming source's raster overlays ("; " separated, HTML
/// stripped), written to `out` (NUL-terminated, truncated to `cap`); returns the length needed.
/// 0 and an empty string when no source or no overlay is open.
/// The length leaves the NUL out, as snprintf's does: a result >= cap means `out` was cut.
/// Call on the render_frame thread.
uint32_t environment_source_credits(VisualRenderer*, char* out, uint32_t cap);

}
```

In `overlume/src/environment_stream.hpp`, replace

```cpp
#include <unordered_map>
#include <vector>
```

with

```cpp
#include <unordered_map>
#include <utility>
#include <vector>
```

replace

```cpp
    void hold(const std::string& url_without_query);
    void release_held();
```

with

```cpp
    void hold(const std::string& url_without_query);
    void release_held();
    // Every URL starting with `prefix` that no exact rule names gets this answer; the longest
    // matching prefix wins, and a later rule for the same prefix replaces an earlier one.
    void serve_prefix(const std::string& prefix, uint16_t status, std::vector<std::byte> body);
    // Requests for a URL starting with `prefix` that no exact rule names stay pending, as hold()'s.
    void hold_prefix(const std::string& prefix);
    // From now on every request, and every held request release_held() answers, is a transport
    // error: the network is gone.
    void fail_all_transport();
```

replace

```cpp
    std::vector<Held> pending_;
    std::vector<std::string> requested_;
};
```

with

```cpp
    std::vector<Held> pending_;
    std::vector<std::string> requested_;
    std::vector<std::pair<std::string, Canned>> prefixes_;
    std::vector<std::string> heldPrefixes_;
    bool allTransportDown_ = false;
};
```

replace

```cpp
    void force_fall_back_for_testing(VisualRenderer& r) { fall_back(r); }
```

with

```cpp
    void force_fall_back_for_testing(VisualRenderer& r) { fall_back(r); }

    // Task 11: a second tileset. Called once before the first update(); overlays are already added
    // to `terrain->getOverlays()` by the caller. provides_ground(), height sampling and the
    // terrain transform target the terrain tileset when present.
    void attach_terrain(std::unique_ptr<Cesium3DTilesSelection::Tileset> terrain,
                        std::shared_ptr<StreamRendererResources> terrain_resources);
    bool has_terrain() const;
    // Task 11: the overlay credits of both tilesets, plain text, "; " separated, HTML stripped.
    std::string credits() const;

    // The terrain tileset's renderer resources, null without one. Kept after teardown.
    std::shared_ptr<StreamRendererResources> terrain_resources() const { return terrainResources_; }
    // Terrain tiles in the Filament scene.
    size_t terrain_scene_count(VisualRenderer& r) const;
    // World z minus local z of a terrain tile's root: the follow-terrain lift (NaN without one).
    double terrain_tile_lift_z(VisualRenderer& r) const;
```

replace Task 5's

```cpp
    StreamRendererResources::OverlayCounters overlay_counters() const {
        return renderResources_->overlay_counters();
    }
```

with

```cpp
    // The terrain resources' counters once attach_terrain has run, else these (Task 5's fixture
    // opens its terrain as the content tileset).
    StreamRendererResources::OverlayCounters overlay_counters() const {
        return (terrainResources_ ? terrainResources_ : renderResources_)->overlay_counters();
    }
```

replace

```cpp
    void fall_back(VisualRenderer& r);
```

with

```cpp
    void fall_back(VisualRenderer& r);
    // Adds the tiles of `result` within the radius to the scene, removes those that left it.
    void sync_scene(VisualRenderer& r, const Cesium3DTilesSelection::ViewUpdateResult& result,
                    Vec3 ego_map_pos, double radius_m,
                    std::unordered_map<const void*, bool>& in_scene);
```

and replace

```cpp
    bool fallenBack_ = false;
    std::unique_ptr<EnvironmentSource> fallbackSource_;
};
```

with

```cpp
    bool fallenBack_ = false;
    std::unique_ptr<EnvironmentSource> fallbackSource_;

    std::shared_ptr<StreamRendererResources> terrainResources_;
    std::unique_ptr<Cesium3DTilesSelection::Tileset> terrainTileset_;
    std::unordered_map<const void*, bool> terrainInScene_;
};
```

(Members are destroyed in reverse order, so a source destroyed without teardown frees the terrain
tileset before its resources.) In `StreamRendererResources`, replace Task 5's

```cpp
    OverlayCounters overlay_counters() const { return counters_; }
```

with

```cpp
    OverlayCounters overlay_counters() const { return counters_; }
    // The terrain_color sync_terrain_color last pushed (-1 each before the first push).
    std::array<float, 3> terrain_color_for_testing() const { return terrainColor_; }
```

In `overlume/src/environment_test_hooks.hpp`, immediately after Task 5's declaration of
`install_fixture_terrain_with_file_imagery` (the end of the file: Task 5 appended its two hooks
after `renderer_ground_plane_in_scene`), add

```cpp

// Task 11.
bool environment_stream_has_terrain_tileset(overlume::VisualRenderer* r);
// Terrain tiles in the Filament scene (0 while the environment is hidden).
uint64_t environment_terrain_tiles_loaded(overlume::VisualRenderer* r);
// World z minus local z of a terrain tile's root: the follow-terrain lift when max_tilt_deg is 0.
// NaN without a terrain tile in the scene.
double environment_terrain_tile_lift_z(overlume::VisualRenderer* r);
// A fixture network for ion:// sources (a MapFixtureAssetAccessor; never freed). Canned answers
// are keyed by URL without its query string; unmatched URLs answer 404.
struct IonFixtureNetwork;
IonFixtureNetwork* make_ion_fixture_network();
void ion_fixture_serve(IonFixtureNetwork* n, const std::string& url_without_query, uint16_t status,
                       std::vector<uint8_t> body);
// Every URL starting with `prefix` that no exact rule names; the longest prefix wins.
void ion_fixture_serve_prefix(IonFixtureNetwork* n, const std::string& prefix, uint16_t status,
                              std::vector<uint8_t> body);
// Requests for a URL starting with `prefix` that no exact rule names stay pending until
// ion_fixture_release_held answers them.
void ion_fixture_hold_prefix(IonFixtureNetwork* n, const std::string& prefix);
void ion_fixture_release_held(IonFixtureNetwork* n);
// The network is gone: every later request, and every held one released, is a transport error.
void ion_fixture_fail_all_transport(IonFixtureNetwork* n);
std::vector<std::string> ion_fixture_requested_urls(IonFixtureNetwork* n);
// Opens an ion:// URI exactly as set_environment_source does (parse, source, terrain tileset and
// imagery overlay, set_visible, teardown of the old source, swap), but over `n` and with `token`
// in place of CESIUM_ION_TOKEN.
bool install_fixture_ion_source(overlume::VisualRenderer* r, const char* ion_uri,
                                overlume::GeoAnchor anchor, IonFixtureNetwork* n,
                                const char* token);
// The overlay counters of the terrain resources of the last source installed over `n`, read even
// after that source was switched away or fell back. Zeros without one.
OverlayProbe ion_fixture_terrain_probe(IonFixtureNetwork* n);
// The environment.terrain_color last pushed to those terrain resources (-1 each before the first
// push), into `rgb`. False without terrain resources.
bool ion_fixture_terrain_color(IonFixtureNetwork* n, float rgb[3]);
```

- [ ] **Step 3: Write the failing tests** (`overlume/tests/test_environment_stream.cpp`)

In the include block, after `#include "golden.hpp"`, add the two fixture headers (skip a line Task 5
already added):

```cpp
#include "imagery_fixture.hpp"
#include "quantized_mesh_fixture.hpp"
```

and add `#include <array>` after `#include <algorithm>`, `#include <functional>` after
`#include <fstream>` and `#include <iterator>` after `#include <iostream>` (each unless already
there). Then append at the end of the file:

```cpp

// Task 11: an ion:// source with a second tileset (layer.json terrain with an ion imagery overlay)
// over a fixture network. Asset 96188 is a 3D Tiles endpoint whose tileset is a file fixture; asset
// 1 is terrain, the two level-0 tiles of the geographic scheme flat at 0 m (Task 2's encoder);
// asset 2 is TMS imagery whose every tile is one flat 512-px PNG. Each test uses its own token:
// cesium-native caches ion endpoints per URL, token included, for the life of the process.
namespace ion_terrain_fixture {

namespace qmesh = overlume::testing::qmesh;
using overlume::testing::IonFixtureNetwork;

const std::string kContentBase = "https://assets.ion.fixture/96188/";
const std::string kTerrainBase = "https://assets.ion.fixture/1/";
const std::string kImageryBase = "https://assets.ion.fixture/2/";
const std::string kAttributedImageryEndpoint =
    R"({"type":"IMAGERY","url":")" + kImageryBase +
    R"(","accessToken":"fixture-asset-token","attributions":[{"html":"<span><a href=\"https://fixture.example/terms\">Fixture Imagery</a> &amp; Co.</span>","collapsible":false}]})";
constexpr char kTerrainDidNotOpen401[] =
    "environment source: terrain tileset did not open (HTTP 401) -- continuing without terrain";
constexpr size_t kFrameBytes = 320u * 240u * 3u;

std::string endpoint_url(int64_t asset_id) {
    return "https://api.cesium.com/v1/assets/" + std::to_string(asset_id) + "/endpoint";
}

std::vector<uint8_t> bytes_of(const std::string& s) {
    return std::vector<uint8_t>(s.begin(), s.end());
}

std::vector<uint8_t> file_bytes(const std::filesystem::path& path) {
    std::ifstream f(path, std::ios::binary);
    return std::vector<uint8_t>(std::istreambuf_iterator<char>(f),
                                std::istreambuf_iterator<char>());
}

// Its attribution is the content tileset's own credit, which environment_source_credits omits.
void serve_content_endpoint(IonFixtureNetwork* n) {
    overlume::testing::ion_fixture_serve(
        n, endpoint_url(96188), 200,
        bytes_of(
            R"({"type":"3DTILES","url":")" + kContentBase +
            R"(tileset.json","accessToken":"fixture-asset-token","attributions":[{"html":"Fixture Buildings","collapsible":false}]})"));
}

// Asset 96188: the endpoint, then the fixture's tileset.json and every tile at its own URL.
void serve_content(IonFixtureNetwork* n, const std::string& fixture_dir) {
    serve_content_endpoint(n);
    for (const auto& entry : std::filesystem::directory_iterator(fixture_dir)) {
        const std::string name = entry.path().filename().string();
        if (name == "PROVENANCE.md") continue;
        overlume::testing::ion_fixture_serve(n, kContentBase + name, 200, file_bytes(entry.path()));
    }
}

// Asset 1: Task 2's terrain endpoint, a layer.json listing the two level-0 tiles and those tiles at
// layer.json's "{z}/{x}/{y}.terrain". A layer.json refused with `layer_json_status` serves no tile.
void serve_terrain(IonFixtureNetwork* n, uint16_t layer_json_status = 200) {
    overlume::testing::ion_fixture_serve(n, endpoint_url(1), 200,
                                         bytes_of(qmesh::ion_terrain_endpoint_json(kTerrainBase)));
    if (layer_json_status != 200) {
        overlume::testing::ion_fixture_serve(n, kTerrainBase + "layer.json", layer_json_status, {});
        return;
    }
    overlume::testing::ion_fixture_serve(n, kTerrainBase + "layer.json", 200,
                                         bytes_of(qmesh::layer_json({{0, 0, 0}, {0, 1, 0}})));
    for (const uint32_t x : {0u, 1u}) {
        overlume::testing::ion_fixture_serve(
            n, kTerrainBase + "0/" + std::to_string(x) + "/0.terrain", 200,
            qmesh::encode_quantized_mesh(0, x, 0, qmesh::QuantizedMeshOptions{}));
    }
}

// Asset 2: `endpoint_json`, Task 2's tilemapresource.xml at the TMS base and beside it, and one
// flat PNG for every other URL under the base.
void serve_imagery(IonFixtureNetwork* n, const std::string& endpoint_json) {
    overlume::testing::ion_fixture_serve(n, endpoint_url(2), 200, bytes_of(endpoint_json));
    const std::vector<uint8_t> xml = bytes_of(qmesh::tilemapresource_xml(512));
    overlume::testing::ion_fixture_serve(n, kImageryBase, 200, xml);
    overlume::testing::ion_fixture_serve(n, kImageryBase + "tilemapresource.xml", 200, xml);
    overlume::testing::ion_fixture_serve_prefix(
        n, kImageryBase, 200, overlume::testing::imagery::flat_png(512, 40, 160, 220));
}

bool is_imagery_tile_url(const std::string& url) {
    const std::string key = url.substr(0, url.find('?'));
    return key.rfind(kImageryBase, 0) == 0 && key != kImageryBase &&
           key != kImageryBase + "tilemapresource.xml";
}

size_t count_requested(IonFixtureNetwork* n, const std::function<bool(const std::string&)>& match) {
    const std::vector<std::string> urls = overlume::testing::ion_fixture_requested_urls(n);
    return static_cast<size_t>(std::count_if(urls.begin(), urls.end(), match));
}

size_t count_of(const std::string& text, const std::string& needle) {
    size_t count = 0;
    for (size_t pos = text.find(needle); pos != std::string::npos;
         pos = text.find(needle, pos + 1)) {
        ++count;
    }
    return count;
}

overlume::VisualRenderer* make_renderer() {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    return overlume::create_renderer(cfg);
}

void set_ego(overlume::VisualRenderer* r) {
    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kFixtureBlockCenterMap;
    overlume::set_scene(r, s);
}

// Renders kStdPose frames 5 ms apart until `done` holds; false if it never did.
bool pump(overlume::VisualRenderer* r, std::vector<uint8_t>& buf, int max_frames,
          const std::function<bool()>& done) {
    for (int i = 0; i < max_frames; ++i) {
        if (!overlume::render_frame(r, kStdPose, {buf.data(), 320, 240})) return false;
        if (done()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return done();
}

std::string credits(overlume::VisualRenderer* r) {
    std::vector<char> buf(overlume::environment_source_credits(r, nullptr, 0) + 1);
    overlume::environment_source_credits(r, buf.data(), static_cast<uint32_t>(buf.size()));
    return buf.data();
}

}

namespace itf = ion_terrain_fixture;

TEST(EnvironmentStreamTerrain, IonTerrainAndImageryLoadOverTheFixtureNetwork) {
    auto* r = itf::make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    auto* n = overlume::testing::make_ion_fixture_network();
    itf::serve_content(n, kTilesFixtureDir);
    itf::serve_terrain(n);
    itf::serve_imagery(n, itf::qmesh::ion_imagery_endpoint_json(itf::kImageryBase));
    ASSERT_TRUE(overlume::testing::install_fixture_ion_source(
        r, "ion://96188?terrain=1&imagery=2&cache=off", kFixtureAnchor, n,
        "ion-fixture-token-load"));
    itf::set_ego(r);
    EXPECT_TRUE(overlume::testing::environment_stream_has_terrain_tileset(r));

    std::vector<uint8_t> buf(itf::kFrameBytes);
    const auto terrainWithImagery = [&] {
        return overlume::testing::environment_terrain_tiles_loaded(r) > 0 &&
               overlume::testing::environment_stream_overlay_probe(r).rasters_attached > 0;
    };
    ASSERT_TRUE(itf::pump(r, buf, 600, terrainWithImagery))
        << "terrain tiles in the scene: " << overlume::testing::environment_terrain_tiles_loaded(r)
        << ", rasters attached: "
        << overlume::testing::environment_stream_overlay_probe(r).rasters_attached;
    EXPECT_GT(overlume::testing::environment_stream_overlay_probe(r).textures_alive, 0u);
    const auto buildings = [&] { return overlume::testing::environment_loaded_chunk_count(r) > 0; };
    EXPECT_TRUE(itf::pump(r, buf, 300, buildings))
        << "the content tileset must still stream its buildings";
    const auto easternTile = [](const std::string& url) {
        return url.rfind(itf::kTerrainBase + "0/1/0.terrain", 0) == 0;
    };
    EXPECT_GT(itf::count_requested(n, easternTile), 0u)
        << "the level-0 tile under the ego, the eastern hemisphere's";
    EXPECT_GT(itf::count_requested(n, itf::is_imagery_tile_url), 0u);

    overlume::set_environment_visible(r, false);
    EXPECT_EQ(overlume::testing::environment_terrain_tiles_loaded(r), 0u)
        << "hiding the environment must take the terrain out of the scene too";
    overlume::set_environment_visible(r, true);
    EXPECT_GT(overlume::testing::environment_terrain_tiles_loaded(r), 0u);
    EXPECT_EQ(overlume::environment_source_state(r), overlume::EnvironmentSourceState::STREAMING);
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStreamTerrain, TheTerrainSampleProvidesTheGroundWithoutFollowTerrain) {
    auto* r = itf::make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    auto* n = overlume::testing::make_ion_fixture_network();
    itf::serve_content(n, kTilesFixtureDir);
    itf::serve_terrain(n);
    ASSERT_TRUE(overlume::testing::install_fixture_ion_source(
        r, "ion://96188?terrain=1&cache=off", kFixtureAnchor, n, "ion-fixture-token-ground"));
    itf::set_ego(r);

    std::vector<uint8_t> buf(itf::kFrameBytes);
    bool hit = false;
    for (int i = 0; i < 600 && !hit; ++i) {
        overlume::render_frame(r, kStdPose, {buf.data(), 320, 240});
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        hit = overlume::testing::environment_stream_provides_ground(r);
    }
    ASSERT_TRUE(hit) << "the sample under the ego must hit the terrain tileset: this content "
                        "fixture is buildings only (GroundPlaneKeptWhenTilesetIsBuildingsOnly)";
    overlume::render_frame(r, kStdPose, {buf.data(), 320, 240});
    EXPECT_FALSE(overlume::testing::renderer_ground_plane_in_scene(r))
        << "the terrain must take the clay plane's place";
    EXPECT_EQ(overlume::testing::environment_terrain_offset_z(r), 0.0)
        << "without follow_terrain the sample must not move the environment";
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStreamTerrain, TheFollowTerrainLiftMovesTheTerrainTiles) {
    auto* r = itf::make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    auto* n = overlume::testing::make_ion_fixture_network();
    itf::serve_content(n, kTilesFixtureDir);
    itf::serve_terrain(n);
    ASSERT_TRUE(overlume::testing::install_fixture_ion_source(
        r, "ion://96188?terrain=1&cache=off&follow_terrain=on&max_tilt_deg=0&ground_bias=0",
        kFixtureAnchor, n, "ion-fixture-token-lift"));
    itf::set_ego(r);

    // One flat tile per hemisphere: its chords run tens of kilometres below the ellipsoid under
    // the ego, so the sample lands far below the anchor and the offset settles at its 30 m clamp.
    std::vector<uint8_t> buf(itf::kFrameBytes);
    const auto lifted = [&] {
        return overlume::testing::environment_terrain_tiles_loaded(r) > 0 &&
               std::abs(overlume::testing::environment_terrain_offset_z(r)) > 1.0;
    };
    ASSERT_TRUE(itf::pump(r, buf, 600, lifted))
        << "offset " << overlume::testing::environment_terrain_offset_z(r);
    for (int i = 0; i < 10; ++i) overlume::render_frame(r, kStdPose, {buf.data(), 320, 240});
    EXPECT_NEAR(overlume::testing::environment_terrain_tile_lift_z(r),
                overlume::testing::environment_terrain_offset_z(r), 1.0)
        << "the terrain tiles must ride the buildings' lift (1 m: float32 transforms whose "
           "translation is the anchor's ECEF distance, about 6.4e6 m)";
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStreamTerrain, TheTerrainResourcesFollowAThemeSwitch) {
    auto* r = itf::make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    auto* n = overlume::testing::make_ion_fixture_network();
    itf::serve_content(n, kTilesFixtureDir);
    itf::serve_terrain(n);
    ASSERT_TRUE(overlume::testing::install_fixture_ion_source(
        r, "ion://96188?terrain=1&cache=off", kFixtureAnchor, n, "ion-fixture-token-theme"));
    itf::set_ego(r);

    std::vector<uint8_t> buf(itf::kFrameBytes);
    const auto terrainInScene = [&] {
        return overlume::testing::environment_terrain_tiles_loaded(r) > 0;
    };
    ASSERT_TRUE(itf::pump(r, buf, 600, terrainInScene)) << "no terrain tile reached the scene";
    float dark[3] = {-1.0f, -1.0f, -1.0f};
    ASSERT_TRUE(overlume::testing::ion_fixture_terrain_color(n, dark));
    EXPECT_GE(dark[0], 0.0f) << "update() must push dark_adas's terrain_color to the terrain tiles";

    // A transition that ended 9.9 s before the scene's sim time 0: the next frame is light_clay.
    ASSERT_TRUE(overlume::set_theme(r, "light_clay", -10.0, 0.1));
    for (int i = 0; i < 3; ++i) overlume::render_frame(r, kStdPose, {buf.data(), 320, 240});
    float light[3] = {-1.0f, -1.0f, -1.0f};
    ASSERT_TRUE(overlume::testing::ion_fixture_terrain_color(n, light));
    EXPECT_FALSE(dark[0] == light[0] && dark[1] == light[1] && dark[2] == light[2])
        << "live terrain tiles must take light_clay's terrain_color";
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStreamTerrain, CreditsAreThePlainTextOfTheImageryAttributions) {
    auto* r = itf::make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    char untouched[16];
    std::memset(untouched, 'x', sizeof untouched);
    EXPECT_EQ(overlume::environment_source_credits(r, untouched, sizeof untouched), 0u);
    EXPECT_STREQ(untouched, "") << "no source is open";

    auto* n = overlume::testing::make_ion_fixture_network();
    itf::serve_content(n, kTilesFixtureDir);
    itf::serve_terrain(n);
    itf::serve_imagery(n, itf::kAttributedImageryEndpoint);
    ASSERT_TRUE(overlume::testing::install_fixture_ion_source(
        r, "ion://96188?terrain=1&imagery=2&cache=off", kFixtureAnchor, n,
        "ion-fixture-token-credits"));
    itf::set_ego(r);
    std::vector<uint8_t> buf(itf::kFrameBytes);
    const auto credited = [&] { return overlume::environment_source_credits(r, nullptr, 0) > 0; };
    ASSERT_TRUE(itf::pump(r, buf, 600, credited)) << "the imagery overlay's credits never appeared";

    EXPECT_EQ(itf::credits(r), "Fixture Imagery & Co.")
        << "tags stripped, &amp; decoded, and the content and terrain tilesets' own credits "
           "(\"Fixture Buildings\", the terrain endpoint's) left out";
    char shortBuf[8];
    EXPECT_EQ(overlume::environment_source_credits(r, shortBuf, sizeof shortBuf), 21u)
        << "the length needed, as snprintf returns it";
    EXPECT_STREQ(shortBuf, "Fixture");

    ASSERT_TRUE(overlume::testing::environment_stream_force_fall_back(r));
    EXPECT_EQ(overlume::environment_source_credits(r, nullptr, 0), 0u)
        << "the credits go with the streamed source";
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStreamTerrain, ATerrainLayerJsonAnswering401OpensWithoutTerrainAndDoesNotFallBack) {
    auto* r = itf::make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    auto* n = overlume::testing::make_ion_fixture_network();
    itf::serve_content(n, kTilesFixtureDir);
    itf::serve_terrain(n, 401);
    const size_t linesBefore =
        itf::count_of(overlume::testing::captured_cesium_log_text(), itf::kTerrainDidNotOpen401);
    ASSERT_TRUE(overlume::testing::install_fixture_ion_source(
        r, "ion://96188?terrain=1&cache=off", kFixtureAnchor, n, "ion-fixture-token-401"));
    itf::set_ego(r);

    std::vector<uint8_t> buf(itf::kFrameBytes);
    const auto buildingsWithoutTerrain = [&] {
        return !overlume::testing::environment_stream_has_terrain_tileset(r) &&
               overlume::testing::environment_loaded_chunk_count(r) > 0;
    };
    ASSERT_TRUE(itf::pump(r, buf, 600, buildingsWithoutTerrain))
        << "the source must drop the terrain whose root never came and keep its buildings";
    for (int i = 0; i < 60; ++i) overlume::render_frame(r, kStdPose, {buf.data(), 320, 240});

    EXPECT_EQ(overlume::environment_source_state(r), overlume::EnvironmentSourceState::STREAMING)
        << "a refused terrain is not network loss";
    EXPECT_FALSE(overlume::testing::environment_stream_has_terrain_tileset(r));
    EXPECT_EQ(overlume::testing::environment_terrain_tiles_loaded(r), 0u);
    EXPECT_GT(overlume::testing::environment_loaded_chunk_count(r), 0u);
    EXPECT_EQ(
        itf::count_of(overlume::testing::captured_cesium_log_text(), itf::kTerrainDidNotOpen401),
        linesBefore + 1)
        << "the source must say so, once";
    const auto terrainTile = [](const std::string& url) {
        return url.find(".terrain") != std::string::npos;
    };
    EXPECT_EQ(itf::count_requested(n, terrainTile), 0u) << "no terrain tile without a layer.json";
    overlume::destroy_renderer(r);
}

// Review Focus 4. The first rasters attach unheld; then every new imagery tile request is held
// (the upsampled terrain children keep asking for finer imagery: the TMS fixture lists levels
// 0-18) and the source is switched with one in flight. Once the old source is gone nothing
// dispatches its main-thread queue, so the late raster runs only its worker half
// (prepareRasterInLoadThread) and makes no texture. The teardown must free every texture the old
// terrain made, and the late raster must attach nowhere.
TEST(EnvironmentStreamTerrain, SwitchingAwayWithARasterRequestInFlightLeavesNoTexture) {
    auto* r = itf::make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    auto* n = overlume::testing::make_ion_fixture_network();
    itf::serve_content(n, kTilesFixtureDir);
    itf::serve_terrain(n);
    itf::serve_imagery(n, itf::qmesh::ion_imagery_endpoint_json(itf::kImageryBase));
    ASSERT_TRUE(overlume::testing::install_fixture_ion_source(
        r, "ion://96188?terrain=1&imagery=2&cache=off", kFixtureAnchor, n,
        "ion-fixture-token-switch"));
    itf::set_ego(r);

    std::vector<uint8_t> buf(itf::kFrameBytes);
    const auto textured = [&] {
        return overlume::testing::ion_fixture_terrain_probe(n).textures_alive > 0;
    };
    ASSERT_TRUE(itf::pump(r, buf, 600, textured)) << "the first rasters must attach unheld";

    overlume::testing::ion_fixture_hold_prefix(n, itf::kImageryBase);
    const size_t imageryRequests = itf::count_requested(n, itf::is_imagery_tile_url);
    const auto rasterInFlight = [&] {
        return itf::count_requested(n, itf::is_imagery_tile_url) > imageryRequests;
    };
    ASSERT_TRUE(itf::pump(r, buf, 600, rasterInFlight))
        << "no imagery tile was requested after the first rasters attached";
    const overlume::testing::OverlayProbe atSwitch =
        overlume::testing::ion_fixture_terrain_probe(n);
    ASSERT_GT(atSwitch.textures_alive, 0u);

    ASSERT_TRUE(overlume::set_environment_source(r, kTestTownDir.c_str(), kFixtureAnchor));
    overlume::testing::ion_fixture_release_held(n);
    const auto baked = [&] { return overlume::testing::environment_loaded_chunk_count(r) > 0; };
    EXPECT_TRUE(itf::pump(r, buf, 120, baked)) << "the baked source must load after the switch";
    EXPECT_EQ(overlume::environment_source_state(r), overlume::EnvironmentSourceState::BAKED);
    const overlume::testing::OverlayProbe after = overlume::testing::ion_fixture_terrain_probe(n);
    EXPECT_EQ(after.textures_alive, 0u)
        << "the teardown must free the old terrain's textures, and the late raster make none";
    EXPECT_GE(after.textures_freed, atSwitch.textures_alive + atSwitch.textures_freed);
    EXPECT_EQ(after.rasters_attached, atSwitch.rasters_attached)
        << "the late raster must not attach to a torn-down tile";
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStreamTerrain, NetworkLossFallsBackAndTearsDownBothTilesets) {
    auto* r = itf::make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    auto* n = overlume::testing::make_ion_fixture_network();
    itf::serve_content_endpoint(n);
    overlume::testing::ion_fixture_serve(
        n, itf::kContentBase + "tileset.json", 200,
        itf::file_bytes(kTilesFixtureFallbackDir + "/tileset.json"));
    // Its 16 tiles are byte-identical copies of tile_root.b3dm (the fixture's PROVENANCE.md), so
    // one prefix serves them all; held, they answer only when the network is already gone.
    overlume::testing::ion_fixture_serve_prefix(
        n, itf::kContentBase + "tile_", 200,
        itf::file_bytes(kTilesFixtureFallbackDir + "/tile_root.b3dm"));
    overlume::testing::ion_fixture_hold_prefix(n, itf::kContentBase + "tile_");
    itf::serve_terrain(n);
    itf::serve_imagery(n, itf::qmesh::ion_imagery_endpoint_json(itf::kImageryBase));
    ASSERT_TRUE(overlume::testing::install_fixture_ion_source(
        r, "ion://96188?terrain=1&imagery=2&cache=off", kFixtureAnchor, n,
        "ion-fixture-token-loss"));
    itf::set_ego(r);

    std::vector<uint8_t> buf(itf::kFrameBytes);
    const auto terrainWithImagery = [&] {
        return overlume::testing::environment_terrain_tiles_loaded(r) > 0 &&
               overlume::testing::environment_stream_overlay_probe(r).rasters_attached > 0;
    };
    ASSERT_TRUE(itf::pump(r, buf, 600, terrainWithImagery))
        << "terrain and imagery must load while the building tiles wait";
    ASSERT_GT(overlume::testing::environment_stream_overlay_probe(r).textures_alive, 0u);

    overlume::testing::ion_fixture_fail_all_transport(n);
    overlume::testing::ion_fixture_release_held(n);
    const auto fellBack = [&] {
        return overlume::environment_source_state(r) ==
               overlume::EnvironmentSourceState::STREAMING_FALLBACK;
    };
    ASSERT_TRUE(itf::pump(r, buf, 200, fellBack))
        << "eight consecutive transport errors must fall back";

    EXPECT_FALSE(overlume::testing::environment_stream_has_terrain_tileset(r));
    EXPECT_EQ(overlume::testing::environment_terrain_tiles_loaded(r), 0u);
    const overlume::testing::OverlayProbe probe = overlume::testing::ion_fixture_terrain_probe(n);
    EXPECT_EQ(probe.textures_alive, 0u) << "the fallback must free every raster texture";
    EXPECT_GT(probe.textures_freed, 0u);
    EXPECT_EQ(overlume::environment_source_credits(r, nullptr, 0), 0u);
    overlume::render_frame(r, kStdPose, {buf.data(), 320, 240});
    EXPECT_TRUE(overlume::testing::renderer_ground_plane_in_scene(r))
        << "the clay plane must come back with the terrain gone";
    overlume::destroy_renderer(r);
}
```

What each test fails on when its behaviour is reverted:
- `IonTerrainAndImageryLoadOverTheFixtureNetwork`: the terrain tileset not updated (no terrain tile
  ever in the scene), no `IonRasterOverlay` (no raster attached), `overlay_counters()` left on the
  content resources (no raster counted), or `set_visible` leaving the terrain out (terrain still in
  the scene while hidden).
- `TheTerrainSampleProvidesTheGroundWithoutFollowTerrain`: the sample still on the content tileset
  (buildings only: no hit), or sampling only with `follow_terrain` (no sample at all).
- `TheFollowTerrainLiftMovesTheTerrainTiles`: the transform applied to the content resources only
  (the terrain's lift stays 0 while the offset is 30 m).
- `TheTerrainResourcesFollowAThemeSwitch`: `update()` not calling `sync_terrain_color()` on the
  terrain resources (their colour stays at the -1 sentinel, so live ion terrain tiles would keep the
  old `terrainColor` through a theme switch). Task 5's `TerrainColorFollowsAThemeSwitchOnLiveTiles`
  cannot catch it: its fixture opens the terrain as the content tileset, whose resources Task 5
  already syncs.
- `CreditsAreThePlainTextOfTheImageryAttributions`: HTML kept, tileset credits mixed in, a missing
  credit system (no ion credit at all), or the snprintf contract broken.
- `ATerrainLayerJsonAnswering401OpensWithoutTerrainAndDoesNotFallBack` (Review Focus 5):
  `has_terrain()` blind to the failed root, or no log line, or a second one.
- `SwitchingAwayWithARasterRequestInFlightLeavesNoTexture` (Review Focus 4): a teardown that leaves
  the terrain tileset (its textures stay alive), a texture made or a raster attached for the request
  answered after the switch, a hang or crash in the switch.
- `NetworkLossFallsBackAndTearsDownBothTilesets`: teardown leaving the terrain tileset or its
  textures behind, credits outliving the source.

- [ ] **Step 4: Run it and see it fail**

No file is new, so no reconfigure is needed.

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_environment_stream
```

Expected: the library and the test compile; the test's link fails with, among its errors,

```
undefined reference to `overlume::environment_source_credits(overlume::VisualRenderer*, char*, unsigned int)'
undefined reference to `overlume::testing::make_ion_fixture_network()'
undefined reference to `overlume::testing::environment_stream_has_terrain_tileset(overlume::VisualRenderer*)'
undefined reference to `overlume::testing::environment_terrain_tiles_loaded(overlume::VisualRenderer*)'
undefined reference to `overlume::testing::ion_fixture_terrain_probe(overlume::testing::IonFixtureNetwork*)'
clang++: error: linker command failed with exit code 1 (use -v to see invocation)
```

and the build exits non-zero. A compile error instead means Step 2 or 3 was mistyped, or a merged
lane's name differs from the Interfaces block (report that as a plan deviation).

- [ ] **Step 5: The fixture accessor's prefix rules** (`overlume/src/environment_stream.cpp`)

Replace

```cpp
void MapFixtureAssetAccessor::release_held() {
    std::vector<Held> pending;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        held_.clear();
        pending.swap(pending_);
    }
```

with

```cpp
void MapFixtureAssetAccessor::serve_prefix(const std::string& prefix, uint16_t status,
                                           std::vector<std::byte> body) {
    std::lock_guard<std::mutex> lock(mutex_);
    prefixes_.emplace_back(prefix, Canned{status, std::move(body), false});
}

void MapFixtureAssetAccessor::hold_prefix(const std::string& prefix) {
    std::lock_guard<std::mutex> lock(mutex_);
    heldPrefixes_.push_back(prefix);
}

void MapFixtureAssetAccessor::fail_all_transport() {
    std::lock_guard<std::mutex> lock(mutex_);
    allTransportDown_ = true;
}

void MapFixtureAssetAccessor::release_held() {
    std::vector<Held> pending;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        held_.clear();
        heldPrefixes_.clear();
        pending.swap(pending_);
    }
```

replace

```cpp
        requested_.push_back(url);
        if (std::find(held_.begin(), held_.end(), key) != held_.end()) {
```

with

```cpp
        requested_.push_back(url);
        const auto startsKey = [&key](const std::string& p) { return key.rfind(p, 0) == 0; };
        if (std::find(held_.begin(), held_.end(), key) != held_.end() ||
            (canned_.find(key) == canned_.end() &&
             std::any_of(heldPrefixes_.begin(), heldPrefixes_.end(), startsKey))) {
```

and replace

```cpp
    Canned canned{unmatchedStatus_, {}, false};
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = canned_.find(url.substr(0, url.find('?')));
        if (it != canned_.end()) canned = it->second;
    }
```

with

```cpp
    Canned canned{unmatchedStatus_, {}, false};
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const std::string key = url.substr(0, url.find('?'));
        if (allTransportDown_) {
            canned.transport_error = true;
        } else if (const auto it = canned_.find(key); it != canned_.end()) {
            canned = it->second;
        } else {
            size_t longest = 0;
            for (const auto& [prefix, rule] : prefixes_) {
                if (key.rfind(prefix, 0) == 0 && prefix.size() >= longest) {
                    canned = rule;
                    longest = prefix.size();
                }
            }
        }
    }
```

Exact rules, exact holds and the unmatched status behave as before, so the Mapbox tests, which use
only those, see no change.

- [ ] **Step 6: The second tileset in the source** (`overlume/src/environment_stream.cpp`)

After `#include <CesiumGltfWriter/GltfWriter.h>` add (keep the block alphabetical with any include
Task 5 added there):

```cpp
#include <CesiumRasterOverlays/IonRasterOverlay.h>
#include <CesiumRasterOverlays/RasterOverlayTileProvider.h>
#include <CesiumUtility/CreditReferencer.h>
#include <CesiumUtility/CreditSystem.h>
```

In `build_externals`, replace

```cpp
    if (cache_dir == "off") {
        Cesium3DTilesSelection::TilesetExternals externals{nullptr, nullptr, asyncSystem};
        externals.pAssetAccessor = counting;
        externals.pLogger = logger;
        return externals;
    }
```

with

```cpp
    // Every source gets a credit system: overlays create their credits only through one, and a
    // layer.json terrain tileset dereferences it unchecked (TilesetContentManager.cpp:2244-2250).
    auto credits = std::make_shared<CesiumUtility::CreditSystem>();

    if (cache_dir == "off") {
        Cesium3DTilesSelection::TilesetExternals externals{nullptr, nullptr, asyncSystem};
        externals.pAssetAccessor = counting;
        externals.pLogger = logger;
        externals.pCreditSystem = credits;
        return externals;
    }
```

and replace

```cpp
    externals.pAssetAccessor = tokenSafe;
    externals.pLogger = logger;
    return externals;
```

with

```cpp
    externals.pAssetAccessor = tokenSafe;
    externals.pLogger = logger;
    externals.pCreditSystem = credits;
    return externals;
```

After `StreamingEnvironmentSource::provides_ground`, that is after

```cpp
bool StreamingEnvironmentSource::provides_ground() const {
    return !fallenBack_ && replacesGround_ && terrainState_ && terrainState_->ground_hit.load();
}
```

add

```cpp

void StreamingEnvironmentSource::attach_terrain(
    std::unique_ptr<Cesium3DTilesSelection::Tileset> terrain,
    std::shared_ptr<StreamRendererResources> terrain_resources) {
    terrainTileset_ = std::move(terrain);
    terrainResources_ = std::move(terrain_resources);
}

bool StreamingEnvironmentSource::has_terrain() const {
    // A terrain tileset whose root never came (layer.json refused, asset missing) has no root once
    // its root-available event is ready: the source then runs on its content tileset alone.
    return terrainTileset_ != nullptr && !(terrainTileset_->getRootTileAvailableEvent().isReady() &&
                                           terrainTileset_->getRootTile() == nullptr);
}

namespace {

// The plain text of an attribution's HTML: tags dropped, a few entities decoded, whitespace runs
// collapsed. ponytail: a tag scanner, not an HTML parser (a '>' inside an attribute value ends
// the tag early); use a real parser if an attribution ever shows markup on the line.
std::string credit_plain_text(const std::string& html) {
    static constexpr std::pair<const char*, const char*> kEntities[] = {
        {"&amp;", "&"}, {"&lt;", "<"},   {"&gt;", ">"},         {"&quot;", "\""},
        {"&#39;", "'"}, {"&nbsp;", " "}, {"&copy;", "\xC2\xA9"}};
    std::string out;
    bool inTag = false;
    for (size_t i = 0; i < html.size(); ++i) {
        const char c = html[i];
        if (inTag || c == '<') {
            inTag = c != '>';
            continue;
        }
        if (std::isspace(static_cast<unsigned char>(c)) != 0) {
            if (!out.empty() && out.back() != ' ') out += ' ';
            continue;
        }
        const auto entity = std::find_if(
            std::begin(kEntities), std::end(kEntities),
            [&](const auto& e) { return html.compare(i, std::strlen(e.first), e.first) == 0; });
        if (entity != std::end(kEntities)) {
            out += entity->second;
            i += std::strlen(entity->first) - 1;
            continue;
        }
        out += c;
    }
    while (!out.empty() && out.back() == ' ') out.pop_back();
    return out;
}

}

std::string StreamingEnvironmentSource::credits() const {
    if (fallenBack_ || tornDown_) return {};
    std::vector<std::string> texts;
    for (const Cesium3DTilesSelection::Tileset* tileset : {tileset_.get(), terrainTileset_.get()}) {
        if (tileset == nullptr || !tileset->getExternals().pCreditSystem) continue;
        CesiumUtility::CreditSystem& creditSystem = *tileset->getExternals().pCreditSystem;
        // CreditReferencer cannot list its credits: reference the overlays' credits, then keep the
        // system's referenced credits that this referencer holds.
        CesiumUtility::CreditReferencer overlayCredits(tileset->getExternals().pCreditSystem);
        for (const auto& activated : tileset->getOverlays().getActivatedOverlays()) {
            if (activated->getTileProvider() != nullptr) {
                activated->getTileProvider()->addCredits(overlayCredits);
            }
        }
        for (const CesiumUtility::Credit& credit :
             creditSystem.getSnapshot(CesiumUtility::CreditFilteringMode::None).currentCredits) {
            if (!overlayCredits.isCreditReferenced(credit)) continue;
            std::string text = credit_plain_text(creditSystem.getHtml(credit));
            if (!text.empty() && std::find(texts.begin(), texts.end(), text) == texts.end()) {
                texts.push_back(std::move(text));
            }
        }
    }
    std::string out;
    for (const std::string& text : texts) out += (out.empty() ? "" : "; ") + text;
    return out;
}

namespace {
size_t count_in_scene(VisualRenderer& r, const std::unordered_map<const void*, bool>& tracked) {
    size_t count = 0;
    for (const auto& [res, _] : tracked) {
        auto* asset = static_cast<filament::gltfio::FilamentAsset*>(const_cast<void*>(res));
        if (asset->getEntityCount() > 0 && r.scene->hasEntity(asset->getEntities()[0])) {
            ++count;
        }
    }
    return count;
}
}

size_t StreamingEnvironmentSource::terrain_scene_count(VisualRenderer& r) const {
    return fallenBack_ ? 0 : count_in_scene(r, terrainInScene_);
}

double StreamingEnvironmentSource::terrain_tile_lift_z(VisualRenderer& r) const {
    if (terrainInScene_.empty() || r.engine == nullptr) return std::nan("");
    auto* asset = static_cast<filament::gltfio::FilamentAsset*>(
        const_cast<void*>(terrainInScene_.begin()->first));
    filament::TransformManager& tm = r.engine->getTransformManager();
    const auto inst = tm.getInstance(asset->getRoot());
    if (!inst.isValid()) return std::nan("");
    return static_cast<double>(tm.getWorldTransform(inst)[3].z - tm.getTransform(inst)[3].z);
}
```

In `update`, replace (Task 5 added the `sync_terrain_color()` line)

```cpp
    renderResources_->set_renderer(&r);
    renderResources_->sync_terrain_color();
    for (filament::gltfio::FilamentAsset* freed : renderResources_->drain_pending_frees()) {
        inScene_.erase(freed);
    }
    synthesize_view_and_pump(r, ego_map_pos);
```

with

```cpp
    renderResources_->set_renderer(&r);
    renderResources_->sync_terrain_color();
    for (filament::gltfio::FilamentAsset* freed : renderResources_->drain_pending_frees()) {
        inScene_.erase(freed);
    }
    if (terrainResources_) {
        terrainResources_->set_renderer(&r);
        terrainResources_->sync_terrain_color();
        for (filament::gltfio::FilamentAsset* freed : terrainResources_->drain_pending_frees()) {
            terrainInScene_.erase(freed);
        }
    }
    synthesize_view_and_pump(r, ego_map_pos);
```

In `maybe_trigger_terrain_sample`, replace

```cpp
    tileset_->sampleHeightMostDetailed(cartos)
        .thenInMainThread([state, validS, pivotX, pivotY,
```

with

```cpp
    // With a terrain tileset, the ground under the ego is the terrain's.
    Cesium3DTilesSelection::Tileset& sampled = has_terrain() ? *terrainTileset_ : *tileset_;
    sampled.sampleHeightMostDetailed(cartos)
        .thenInMainThread([state, validS, pivotX, pivotY,
```

In `update_terrain_transform`, replace

```cpp
        renderResources_->set_terrain_transform(
            groundOffsetZ_, terrainTiltRad_, terrainState_->fit_pivot_x, terrainState_->fit_pivot_y,
            terrainState_->fit_pivot_heading_rad);
```

with

```cpp
        for (StreamRendererResources* res : {renderResources_.get(), terrainResources_.get()}) {
            if (res == nullptr) continue;
            res->set_terrain_transform(groundOffsetZ_, terrainTiltRad_, terrainState_->fit_pivot_x,
                                       terrainState_->fit_pivot_y,
                                       terrainState_->fit_pivot_heading_rad);
        }
```

In `synthesize_view_and_pump`, replace

```cpp
    if (followTerrain_) {
        maybe_trigger_terrain_sample(ego_map_pos, r.scene_buffer.active().ego.heading_rad);
        update_terrain_transform(deltaSeconds);
    }
```

with

```cpp
    // A terrain tileset that replaces the ground is sampled even without follow_terrain: the sample
    // under the ego is what lets it take the clay plane's place.
    if (followTerrain_ || (has_terrain() && replacesGround_)) {
        maybe_trigger_terrain_sample(ego_map_pos, r.scene_buffer.active().ego.heading_rad);
    }
    if (followTerrain_) update_terrain_transform(deltaSeconds);
```

and replace the function's tail, from `lastViewFrustumCount_ = views.size();` to its closing brace:

```cpp
    lastViewFrustumCount_ = views.size();
    const ViewUpdateResult& result = tileset_->updateViewGroup(*viewGroup_, views, deltaSeconds);
    tileset_->loadTiles();
    asyncSystem_.dispatchMainThreadTasks();

    std::unordered_map<const void*, bool> stillPresent;
    for (const auto& tilePtr : result.tilesToRenderThisFrame) {
        const Cesium3DTilesSelection::Tile* tile = tilePtr.get();
        if (tile == nullptr || !tile->isRenderContent()) continue;
        // Until Done, the render resources are still the worker's LoadThreadGlb, not an asset.
        if (tile->getState() != Cesium3DTilesSelection::TileLoadState::Done) continue;
        const auto* renderContent = tile->getContent().getRenderContent();
        if (renderContent == nullptr) continue;
        void* res = renderContent->getRenderResources();
        if (res == nullptr) continue;
        const CesiumGeometry::OrientedBoundingBox obb =
            Cesium3DTilesSelection::getOrientedBoundingBoxFromBoundingVolume(
                tile->getBoundingVolume(), CesiumGeospatial::Ellipsoid::WGS84);
        const glm::dvec4 centerMap4 = ecefToMap_ * glm::dvec4(obb.getCenter(), 1.0);
        const glm::dmat3& halfAxes = obb.getHalfAxes();
        const double extentM =
            glm::length(halfAxes[0]) + glm::length(halfAxes[1]) + glm::length(halfAxes[2]);
        const double dx = centerMap4.x - ego_map_pos.x;
        const double dy = centerMap4.y - ego_map_pos.y;
        const bool wasInScene = inScene_.find(res) != inScene_.end();
        const double limitM = wasInScene ? radiusM * kTileRadiusUnloadRatio : radiusM;
        if (std::hypot(dx, dy) - extentM > limitM) continue;
        auto* asset = static_cast<filament::gltfio::FilamentAsset*>(res);
        stillPresent[res] = true;
        if (visible_ && inScene_.find(res) == inScene_.end()) {
            r.scene->addEntities(asset->getEntities(), asset->getEntityCount());
        }
    }
    for (auto it = inScene_.begin(); it != inScene_.end();) {
        if (stillPresent.find(it->first) == stillPresent.end()) {
            auto* asset =
                static_cast<filament::gltfio::FilamentAsset*>(const_cast<void*>(it->first));
            if (visible_) {
                r.scene->removeEntities(asset->getEntities(), asset->getEntityCount());
            }
            it = inScene_.erase(it);
        } else {
            ++it;
        }
    }
    inScene_ = std::move(stillPresent);
}
```

with (the loop moves unchanged into `sync_scene`, `inScene_` and `radiusM` becoming its parameters)

```cpp
    lastViewFrustumCount_ = views.size();
    const ViewUpdateResult& result = tileset_->updateViewGroup(*viewGroup_, views, deltaSeconds);
    tileset_->loadTiles();
    // The terrain tileset selects from the same views; one dispatch serves both (one AsyncSystem).
    const ViewUpdateResult* terrainResult = nullptr;
    if (terrainTileset_) {
        terrainResult = &terrainTileset_->updateViewGroup(terrainTileset_->getDefaultViewGroup(),
                                                          views, deltaSeconds);
        terrainTileset_->loadTiles();
    }
    asyncSystem_.dispatchMainThreadTasks();

    sync_scene(r, result, ego_map_pos, radiusM, inScene_);
    if (terrainResult != nullptr) {
        sync_scene(r, *terrainResult, ego_map_pos, radiusM, terrainInScene_);
    }
}

void StreamingEnvironmentSource::sync_scene(VisualRenderer& r,
                                            const Cesium3DTilesSelection::ViewUpdateResult& result,
                                            Vec3 ego_map_pos, double radius_m,
                                            std::unordered_map<const void*, bool>& in_scene) {
    std::unordered_map<const void*, bool> stillPresent;
    for (const auto& tilePtr : result.tilesToRenderThisFrame) {
        const Cesium3DTilesSelection::Tile* tile = tilePtr.get();
        if (tile == nullptr || !tile->isRenderContent()) continue;
        // Until Done, the render resources are still the worker's LoadThreadGlb, not an asset.
        if (tile->getState() != Cesium3DTilesSelection::TileLoadState::Done) continue;
        const auto* renderContent = tile->getContent().getRenderContent();
        if (renderContent == nullptr) continue;
        void* res = renderContent->getRenderResources();
        if (res == nullptr) continue;
        const CesiumGeometry::OrientedBoundingBox obb =
            Cesium3DTilesSelection::getOrientedBoundingBoxFromBoundingVolume(
                tile->getBoundingVolume(), CesiumGeospatial::Ellipsoid::WGS84);
        const glm::dvec4 centerMap4 = ecefToMap_ * glm::dvec4(obb.getCenter(), 1.0);
        const glm::dmat3& halfAxes = obb.getHalfAxes();
        const double extentM =
            glm::length(halfAxes[0]) + glm::length(halfAxes[1]) + glm::length(halfAxes[2]);
        const double dx = centerMap4.x - ego_map_pos.x;
        const double dy = centerMap4.y - ego_map_pos.y;
        const bool wasInScene = in_scene.find(res) != in_scene.end();
        const double limitM = wasInScene ? radius_m * kTileRadiusUnloadRatio : radius_m;
        if (std::hypot(dx, dy) - extentM > limitM) continue;
        auto* asset = static_cast<filament::gltfio::FilamentAsset*>(res);
        stillPresent[res] = true;
        if (visible_ && in_scene.find(res) == in_scene.end()) {
            r.scene->addEntities(asset->getEntities(), asset->getEntityCount());
        }
    }
    for (auto it = in_scene.begin(); it != in_scene.end();) {
        if (stillPresent.find(it->first) == stillPresent.end()) {
            auto* asset =
                static_cast<filament::gltfio::FilamentAsset*>(const_cast<void*>(it->first));
            if (visible_) {
                r.scene->removeEntities(asset->getEntities(), asset->getEntityCount());
            }
            it = in_scene.erase(it);
        } else {
            ++it;
        }
    }
    in_scene = std::move(stillPresent);
}
```

Replace the whole of `teardown`:

```cpp
void StreamingEnvironmentSource::teardown(VisualRenderer& r) {
    if (fallbackSource_) {
        fallbackSource_->teardown(r);
        fallbackSource_.reset();
    }
    if (tornDown_) return;
    renderResources_->set_renderer(&r);

    for (auto& [res, _] : inScene_) {
        auto* asset = static_cast<filament::gltfio::FilamentAsset*>(const_cast<void*>(res));
        if (visible_) {
            r.scene->removeEntities(asset->getEntities(), asset->getEntityCount());
        }
    }
    inScene_.clear();
    renderResources_->drain_pending_frees();

    CesiumAsync::SharedFuture<void> destructionComplete =
        tileset_->getAsyncDestructionCompleteEvent();
    tileset_.reset();
    renderResources_->drain_pending_frees();

    int pumps = 0;
    for (; pumps < kTeardownPumpBound && !destructionComplete.isReady(); ++pumps) {
        asyncSystem_.dispatchMainThreadTasks();
    }
    if (pumps >= kTeardownPumpBound && !destructionComplete.isReady()) {
        ++leakedOnTeardownBound_;
    }

    renderResources_->drain_pending_frees();
    renderResources_->destroy_terrain_root();
    renderResources_->note_torn_down();
    tornDown_ = true;
}
```

with

```cpp
void StreamingEnvironmentSource::teardown(VisualRenderer& r) {
    if (fallbackSource_) {
        fallbackSource_->teardown(r);
        fallbackSource_.reset();
    }
    if (tornDown_) return;
    // The terrain goes first. Destroying a tileset unloads every tile, and cesium detaches a tile's
    // raster overlay tiles before it frees the tile (TilesetContentManager.cpp:1416-1420), so the
    // overlays are detached before either tileset's tiles are freed.
    std::vector<StreamRendererResources*> resources{renderResources_.get()};
    if (terrainResources_) resources.insert(resources.begin(), terrainResources_.get());
    for (StreamRendererResources* res : resources) res->set_renderer(&r);

    for (std::unordered_map<const void*, bool>* tracked : {&terrainInScene_, &inScene_}) {
        for (auto& [res, _] : *tracked) {
            auto* asset = static_cast<filament::gltfio::FilamentAsset*>(const_cast<void*>(res));
            if (visible_) {
                r.scene->removeEntities(asset->getEntities(), asset->getEntityCount());
            }
        }
        tracked->clear();
    }
    for (StreamRendererResources* res : resources) res->drain_pending_frees();

    std::vector<CesiumAsync::SharedFuture<void>> destructionComplete;
    if (terrainTileset_) {
        destructionComplete.push_back(terrainTileset_->getAsyncDestructionCompleteEvent());
        terrainTileset_.reset();
    }
    destructionComplete.push_back(tileset_->getAsyncDestructionCompleteEvent());
    tileset_.reset();
    for (StreamRendererResources* res : resources) res->drain_pending_frees();

    const auto allDestroyed = [&destructionComplete] {
        return std::all_of(destructionComplete.begin(), destructionComplete.end(),
                           [](const CesiumAsync::SharedFuture<void>& f) { return f.isReady(); });
    };
    int pumps = 0;
    for (; pumps < kTeardownPumpBound && !allDestroyed(); ++pumps) {
        asyncSystem_.dispatchMainThreadTasks();
    }
    if (pumps >= kTeardownPumpBound && !allDestroyed()) {
        ++leakedOnTeardownBound_;
    }

    for (StreamRendererResources* res : resources) {
        res->drain_pending_frees();
        res->destroy_terrain_root();
        res->note_torn_down();
    }
    tornDown_ = true;
}
```

(Task 5 does not change `teardown`.) In `set_visible`, replace

```cpp
    for (auto& [res, _] : inScene_) {
        auto* asset = static_cast<filament::gltfio::FilamentAsset*>(const_cast<void*>(res));
        if (visible_) {
            r.scene->addEntities(asset->getEntities(), asset->getEntityCount());
        } else {
            r.scene->removeEntities(asset->getEntities(), asset->getEntityCount());
        }
    }
```

with

```cpp
    for (std::unordered_map<const void*, bool>* tracked : {&inScene_, &terrainInScene_}) {
        for (auto& [res, _] : *tracked) {
            auto* asset = static_cast<filament::gltfio::FilamentAsset*>(const_cast<void*>(res));
            if (visible_) {
                r.scene->addEntities(asset->getEntities(), asset->getEntityCount());
            } else {
                r.scene->removeEntities(asset->getEntities(), asset->getEntityCount());
            }
        }
    }
```

and in `scene_membership_count`, replace

```cpp
    if (fallenBack_) return fallbackSource_ ? fallbackSource_->scene_membership_count(r) : 0;
    size_t count = 0;
    for (const auto& [res, _] : inScene_) {
        auto* asset = static_cast<filament::gltfio::FilamentAsset*>(const_cast<void*>(res));
        if (asset->getEntityCount() > 0 && r.scene->hasEntity(asset->getEntities()[0])) {
            ++count;
        }
    }
    return count;
```

with

```cpp
    if (fallenBack_) return fallbackSource_ ? fallbackSource_->scene_membership_count(r) : 0;
    return count_in_scene(r, inScene_);
```

- [ ] **Step 7: The ion terrain, its imagery and the credits API**

In `overlume/src/environment_stream.cpp`, replace `open_streaming_environment_source` as Task 4 left
it (lane-2.md, Task 4's refusal line):

```cpp
std::unique_ptr<EnvironmentSource> open_streaming_environment_source(const std::string& ion_spec,
                                                                     GeoAnchor anchor) {
    const std::optional<IonSpec> spec = parse_ion_spec(ion_spec);
    if (!spec) {
        // One line (spec section 5). The spec holds no token: CESIUM_ION_TOKEN is read below.
        make_redacting_logger()->error(
            "ion://{} refused: the asset id or a key has a bad value (terrain= and imagery= take a "
            "positive asset id or off; imagery= needs terrain=)",
            ion_spec);
        return nullptr;
    }

    const char* token = std::getenv("CESIUM_ION_TOKEN");
    if (token == nullptr || token[0] == '\0') return nullptr;

    const std::string cacheDir = spec->cache_dir.empty() ? default_cache_dir() : spec->cache_dir;

    auto curl = std::make_shared<CesiumCurl::CurlAssetAccessor>();
    CesiumAsync::AsyncSystem asyncSystem(std::make_shared<SimpleTaskProcessor>());
    std::shared_ptr<CountingAssetAccessor> counting;
    Cesium3DTilesSelection::TilesetExternals externals =
        build_externals(curl, asyncSystem, cacheDir, spec->max_cache_items, &counting);

    return std::make_unique<StreamingEnvironmentSource>(
        externals, spec->asset_id, std::string(token), std::string(), spec->fallback_dir, anchor,
        std::move(counting), spec->materials_original, spec->follow_terrain, spec->ground_bias_m,
        spec->replaces_ground, spec->max_tilt_deg, spec->brightness, spec->radius_m);
}
```

with

```cpp
namespace {

// Serves open_streaming_environment_source (curl, CESIUM_ION_TOKEN) and install_fixture_ion_source
// (a fixture network, a given token). A null `base` means curl. The spec is parsed, and a refusal
// logged, before the token is looked at: Task 4's ParseIonSpecRefusalLogsOneLine runs without one.
std::unique_ptr<StreamingEnvironmentSource> open_ion_source(
    const std::string& ion_spec, GeoAnchor anchor,
    std::shared_ptr<CesiumAsync::IAssetAccessor> base, const std::string& token) {
    const std::optional<IonSpec> spec = parse_ion_spec(ion_spec);
    if (!spec) {
        // One line (spec section 5). The spec holds no token: the caller passes it.
        make_redacting_logger()->error(
            "ion://{} refused: the asset id or a key has a bad value (terrain= and imagery= take a "
            "positive asset id or off; imagery= needs terrain=)",
            ion_spec);
        return nullptr;
    }
    // A spec that parses opens nothing without a token, silently.
    if (token.empty()) return nullptr;

    const std::string cacheDir = spec->cache_dir.empty() ? default_cache_dir() : spec->cache_dir;

    if (!base) base = std::make_shared<CesiumCurl::CurlAssetAccessor>();
    CesiumAsync::AsyncSystem asyncSystem(std::make_shared<SimpleTaskProcessor>());
    std::shared_ptr<CountingAssetAccessor> counting;
    Cesium3DTilesSelection::TilesetExternals externals =
        build_externals(std::move(base), asyncSystem, cacheDir, spec->max_cache_items, &counting);

    auto source = std::make_unique<StreamingEnvironmentSource>(
        externals, spec->asset_id, token, std::string(), spec->fallback_dir, anchor,
        std::move(counting), spec->materials_original, spec->follow_terrain, spec->ground_bias_m,
        spec->replaces_ground, spec->max_tilt_deg, spec->brightness, spec->radius_m);
    if (!spec->terrain_asset) return source;

    // The terrain shares the content tileset's accessor (so its failures count toward network
    // loss), task processor, logger and credit system; only its renderer resources differ.
    auto terrainResources = std::make_shared<StreamRendererResources>(
        compute_ecef_to_map(anchor), anchor.origin_height_m, false, spec->brightness,
        StreamRendererResources::Kind::Terrain);
    Cesium3DTilesSelection::TilesetExternals terrainExternals = externals;
    terrainExternals.pPrepareRendererResources = terrainResources;
    Cesium3DTilesSelection::TilesetOptions options;
    options.maximumScreenSpaceError = kStreamMaxSseErr;
    options.loadErrorCallback = [](const Cesium3DTilesSelection::TilesetLoadFailureDetails& d) {
        make_redacting_logger()->warn(
            "environment source: terrain tileset did not open (HTTP {}) -- continuing without "
            "terrain",
            d.statusCode);
    };
    auto terrain = std::make_unique<Cesium3DTilesSelection::Tileset>(
        terrainExternals, *spec->terrain_asset, token, options);
    if (spec->imagery_asset) {
        const CesiumUtility::IntrusivePointer<const CesiumRasterOverlays::RasterOverlay> imagery =
            new CesiumRasterOverlays::IonRasterOverlay("ion-imagery", *spec->imagery_asset, token);
        terrain->getOverlays().add(imagery);
    }
    source->attach_terrain(std::move(terrain), std::move(terrainResources));
    return source;
}

}

std::unique_ptr<EnvironmentSource> open_streaming_environment_source(const std::string& ion_spec,
                                                                     GeoAnchor anchor) {
    const char* token = std::getenv("CESIUM_ION_TOKEN");
    return open_ion_source(ion_spec, anchor, nullptr, token != nullptr ? token : "");
}

uint32_t environment_source_credits(VisualRenderer* r, char* out, uint32_t cap) {
    auto* stream = r != nullptr
                       ? dynamic_cast<StreamingEnvironmentSource*>(r->environmentSource.get())
                       : nullptr;
    const std::string text = stream != nullptr ? stream->credits() : std::string();
    if (out != nullptr && cap > 0) {
        const size_t n = std::min<size_t>(text.size(), cap - 1);
        std::memcpy(out, text.data(), n);
        out[n] = '\0';
    }
    return static_cast<uint32_t>(text.size());
}
```

The ion source without `terrain=` is built exactly as before; a refused spec still logs Task 4's one
line, and a spec that parses with an empty token still returns null, silently, before curl is
constructed.

In `overlume/src/environment.cpp`, replace

```cpp
EnvironmentSourceState environment_source_state(VisualRenderer* r) {
    if (r == nullptr || !r->environmentSource) return EnvironmentSourceState::NONE;
    return r->environmentSource->state();
}
```

with

```cpp
EnvironmentSourceState environment_source_state(VisualRenderer* r) {
    if (r == nullptr || !r->environmentSource) return EnvironmentSourceState::NONE;
    return r->environmentSource->state();
}

#ifndef OVERLUME_ENABLE_CESIUM
// Without cesium no source has raster overlays; environment_stream.cpp defines it otherwise.
uint32_t environment_source_credits(VisualRenderer*, char* out, uint32_t cap) {
    if (out != nullptr && cap > 0) out[0] = '\0';
    return 0;
}
#endif
```

- [ ] **Step 8: The hooks** (`overlume/src/environment_stream.cpp`, `namespace overlume::testing`)

After the closing brace of Task 5's `environment_stream_overlay_probe` definition, which stays as
Task 5 wrote it (it reads `stream->overlay_counters()`, which Step 2 pointed at the terrain
resources), add

```cpp

bool environment_stream_has_terrain_tileset(overlume::VisualRenderer* r) {
    if (r == nullptr || !r->environmentSource) return false;
    auto* stream = dynamic_cast<overlume::StreamingEnvironmentSource*>(r->environmentSource.get());
    return stream != nullptr && stream->has_terrain();
}

uint64_t environment_terrain_tiles_loaded(overlume::VisualRenderer* r) {
    if (r == nullptr || !r->environmentSource) return 0;
    auto* stream = dynamic_cast<overlume::StreamingEnvironmentSource*>(r->environmentSource.get());
    return stream != nullptr ? static_cast<uint64_t>(stream->terrain_scene_count(*r)) : 0;
}

double environment_terrain_tile_lift_z(overlume::VisualRenderer* r) {
    if (r == nullptr || !r->environmentSource) return std::nan("");
    auto* stream = dynamic_cast<overlume::StreamingEnvironmentSource*>(r->environmentSource.get());
    return stream != nullptr ? stream->terrain_tile_lift_z(*r) : std::nan("");
}

struct IonFixtureNetwork {
    std::shared_ptr<overlume::MapFixtureAssetAccessor> accessor;
    std::shared_ptr<overlume::StreamRendererResources> last_terrain;
};

namespace {
std::vector<std::byte> as_bytes(const std::vector<uint8_t>& body) {
    const auto* b = reinterpret_cast<const std::byte*>(body.data());
    return std::vector<std::byte>(b, b + body.size());
}
}

IonFixtureNetwork* make_ion_fixture_network() {
    return new IonFixtureNetwork{std::make_shared<overlume::MapFixtureAssetAccessor>(404), nullptr};
}

void ion_fixture_serve(IonFixtureNetwork* n, const std::string& url_without_query, uint16_t status,
                       std::vector<uint8_t> body) {
    n->accessor->serve(url_without_query, status, as_bytes(body));
}

void ion_fixture_serve_prefix(IonFixtureNetwork* n, const std::string& prefix, uint16_t status,
                              std::vector<uint8_t> body) {
    n->accessor->serve_prefix(prefix, status, as_bytes(body));
}

void ion_fixture_hold_prefix(IonFixtureNetwork* n, const std::string& prefix) {
    n->accessor->hold_prefix(prefix);
}

void ion_fixture_release_held(IonFixtureNetwork* n) { n->accessor->release_held(); }

void ion_fixture_fail_all_transport(IonFixtureNetwork* n) { n->accessor->fail_all_transport(); }

std::vector<std::string> ion_fixture_requested_urls(IonFixtureNetwork* n) {
    return n->accessor->requested_urls();
}

bool install_fixture_ion_source(overlume::VisualRenderer* r, const char* ion_uri,
                                overlume::GeoAnchor anchor, IonFixtureNetwork* n,
                                const char* token) {
    constexpr char kScheme[] = "ion://";
    if (r == nullptr || ion_uri == nullptr || n == nullptr) return false;
    const std::string uri(ion_uri);
    if (uri.rfind(kScheme, 0) != 0) return false;
    std::unique_ptr<overlume::StreamingEnvironmentSource> source = overlume::open_ion_source(
        uri.substr(sizeof(kScheme) - 1), anchor, n->accessor, token != nullptr ? token : "");
    if (!source) return false;
    n->last_terrain = source->terrain_resources();
    source->set_visible(*r, r->environmentVisible);
    if (r->environmentSource) r->environmentSource->teardown(*r);
    r->environmentSource = std::move(source);
    return true;
}

OverlayProbe ion_fixture_terrain_probe(IonFixtureNetwork* n) {
    if (n == nullptr || !n->last_terrain) return {};
    const overlume::StreamRendererResources::OverlayCounters c =
        n->last_terrain->overlay_counters();
    return {c.rasters_attached, c.rasters_detached, c.textures_alive, c.textures_freed};
}

bool ion_fixture_terrain_color(IonFixtureNetwork* n, float rgb[3]) {
    if (n == nullptr || !n->last_terrain) return false;
    const std::array<float, 3> c = n->last_terrain->terrain_color_for_testing();
    std::copy(c.begin(), c.end(), rgb);
    return true;
}
```

`open_ion_source` sits in an unnamed namespace inside `namespace overlume` in the same file, so
`overlume::open_ion_source` names it here.

- [ ] **Step 9: Run the tests and see them pass, with their neighbours**

Each test runs in its own process, as the gate runs them: the redacting sink keeps 64 KiB of log
text per process, and the 401 test counts lines in it.

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_environment_stream \
    && ctest --test-dir overlume/build -R '^EnvironmentStreamTerrain\.' --output-on-failure \
    | tail -n 4
```

Expected: a clean build and `100% tests passed, 0 tests failed out of 8`.

Nothing in this lane has run these tests before (API notes, "Not checked"). If one fails only on a
budget or a tolerance (a 600-frame pump that ends one condition short, the lift's 1 m, the switch
test finding no new imagery request within 600 frames), measure the value it reached, change that
one number once, and report it as a plan deviation with the measurement; never loosen it silently,
and never touch an assertion about textures, credits, log lines or fallback. A second miss is a
finding for the orchestrator.

Then the whole file and the Mapbox suites that share `MapFixtureAssetAccessor` and
`build_externals`, among them Task 4's `EnvironmentStream.ParseIonSpecRefusalLogsOneLine`, which runs with `CESIUM_ION_TOKEN` unset and
needs the refusal line Step 7 moved into `open_ion_source`:

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
ctest --test-dir overlume/build -R 'EnvironmentStream|EnvSourceCapture|TokenRedaction|Mapbox' \
    --output-on-failure | tail -n 4
```

Expected: `100% tests passed, 0 tests failed out of <N + 8>`, N being the count Step 1 recorded,
and only the live tests listed as not run (skipped: no token, no opt-in).

- [ ] **Step 10: POD header, the no-cesium stub, format**

```bash
cd "$WORKTREE"
overlume/scripts/check_pod_header.sh && echo pod-header-ok
OBJ_FLAGS=overlume/build/CMakeFiles/overlume_obj.dir/flags.make
eval "$(sed -n 's/^# compile CXX with //p' "$OBJ_FLAGS") \
    $(sed -n 's/^CXX_\(DEFINES\|INCLUDES\|FLAGS\) = //p' "$OBJ_FLAGS" | tr '\n' ' ') \
    -UOVERLUME_ENABLE_CESIUM -fsyntax-only overlume/src/environment.cpp" && echo stub-ok
FILES="overlume/include/overlume/scene.h overlume/src/environment_stream.hpp \
    overlume/src/environment_stream.cpp overlume/src/environment_test_hooks.hpp \
    overlume/src/environment.cpp overlume/tests/test_environment_stream.cpp"
git add $FILES
tools/check_format.sh --fix $FILES
git diff --stat
git add $FILES
tools/check_format.sh && tools/check_spdx.sh
```

Expected: `pod-header-ok` (the script prints nothing when every public header is free of `std::`);
`stub-ok` (the stub compiles in the configuration the gate never builds: `-U` after the flags'
`-DOVERLUME_ENABLE_CESIUM=1` wins); `check_format.sh: reformatted 6 files with clang-format 23.1.1`;
`git diff --stat` prints nothing (the code above is clang-format 23.1.1's output); then
`check_format.sh: PASS (… files, clang-format 23.1.1)` and `check_spdx.sh: PASS (… files carry the
SPDX identifier)`, both counts unchanged from the fork (no file is new).

- [ ] **Step 11: Revert checks (do not commit)**

Each check removes one behaviour, rebuilds the test and runs the one test that must catch it, then
restores the file from the index Step 10 staged. A rebuild that fails prints `BUILD FAILED` instead
of running a stale binary.

```bash
cd "$WORKTREE"
check() {  # $1: sed script, $2: test name
    sed -i "$1" overlume/src/environment_stream.cpp
    git diff --stat overlume/src/environment_stream.cpp
    if CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
        --target test_environment_stream > /dev/null; then
        overlume/build/test_environment_stream --gtest_filter="EnvironmentStreamTerrain.$2" \
            | tail -n 4
    else
        echo "BUILD FAILED: $2"
    fi
    git checkout -- overlume/src/environment_stream.cpp
    git diff --stat overlume/src/environment_stream.cpp
}
check 's/Tileset& sampled = has_terrain() ? \*terrainTileset_ : \*tileset_;/Tileset\& sampled = *tileset_;/' \
    TheTerrainSampleProvidesTheGroundWithoutFollowTerrain
check 's/getRootTileAvailableEvent().isReady() &&$/getRootTileAvailableEvent().isReady() \&\& false \&\&/' \
    ATerrainLayerJsonAnswering401OpensWithoutTerrainAndDoesNotFallBack
check 's/{renderResources_.get(), terrainResources_.get()}) {/{renderResources_.get()}) {/' \
    TheFollowTerrainLiftMovesTheTerrainTiles
check 's/std::string text = credit_plain_text(creditSystem.getHtml(credit));/std::string text = creditSystem.getHtml(credit);/' \
    CreditsAreThePlainTextOfTheImageryAttributions
check '/^        terrainTileset_.reset();$/d' NetworkLossFallsBackAndTearsDownBothTilesets
check '/^        terrainTileset_.reset();$/d' SwitchingAwayWithARasterRequestInFlightLeavesNoTexture
check '/^        terrainResources_->sync_terrain_color();$/d' TheTerrainResourcesFollowAThemeSwitch
check '/^        terrainTileset_->loadTiles();$/d' IonTerrainAndImageryLoadOverTheFixtureNetwork
```

Expected, for each of the eight: the first `git diff --stat` reports
`1 file changed, 1 insertion(+), 1 deletion(-)` (the fifth to the eighth: `1 deletion(-)`); no
`BUILD FAILED`; the filtered run ends

```
[  FAILED  ] 1 test, listed below:
[  FAILED  ] EnvironmentStreamTerrain.<the test named>

 1 FAILED TEST
```

because, in order: the sample hits only the buildings-only content tileset, so the ground is never
provided; a failed terrain still counts as terrain, so the source never drops it; the terrain root
never lifts while the offset reaches 30 m; the credit keeps its `<span>`/`<a>` tags and `&amp;`; the
terrain tileset survives the fallback, so `has_terrain()` stays true and its textures stay alive;
the terrain tileset survives the switch's teardown and dies only with the old source, after
`note_torn_down()`, when Task 5's `freeRaster` drops textures instead of queueing them, so
`textures_alive` stays above 0; the terrain resources' colour stays at the -1 sentinel; no terrain
tile ever loads (cesium loads tiles only in `loadTiles`), so none reaches the scene and the first
pump fails. After each checkout the last `git diff --stat` prints nothing.

- [ ] **Step 12: Gate** (foreground, Bash timeout 600000 ms)

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
ROS_DOMAIN_ID=86 CI_VISUAL_MODE_DOMAIN_ID=86 \
    flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock tools/ci_visual_mode.sh
```

Expected, against the counts Step 1 recorded:
- `PASS  POD header check`
- `PASS  library ctest suite  (100% tests passed, 0 tests failed out of <fork total + 8> (<fork ok + 8> ok / <fork skipped> skipped))`
- `PASS  node gtests  (Summary: <fork count> tests, 0 errors, 0 failures, 0 skipped)`
- `PASS  WS bridge pytest suite  (<fork count> passed in …)`, the time varying
- `PASS  golden suite: <fork count> ok, 0 skipped` (no golden changes: the credit system renders
  nothing, and sources without `terrain=` are unchanged)
- `PASS  examples  (6 run, all exited 0 with non-empty output)`
- `OVERALL: PASS`, exit 0.

Exit 75 means another lane held the lock for 300 s: run the same command again. Any other non-zero
exit is a gate failure. A WS bridge failure can also come from another session's gate on the same
ports, so rerun before treating it as a finding. The prebuild also rebuilds the test from the source
Step 11 restored.

- [ ] **Step 13: Commit**

```bash
cd "$WORKTREE"
git add overlume/include/overlume/scene.h overlume/src/environment_stream.hpp \
    overlume/src/environment_stream.cpp overlume/src/environment_test_hooks.hpp \
    overlume/src/environment.cpp overlume/tests/test_environment_stream.cpp
git status --short
git commit -F - <<'EOF'
feat(environment): a terrain tileset beside the content, ion terrain and imagery, credits

Imagery on terrain needs a second tileset per streaming source.
StreamingEnvironmentSource::attach_terrain adds one that shares the
source's accessor (its failures count toward network loss), task
processor, logger, credit system and views; update() drives both and
keeps both renderer resources on the theme's terrain_color. The ground
sample under the ego, provides_ground and the follow-terrain lift target
the terrain when it is there, and a terrain that replaces the ground is
sampled even without follow_terrain. A terrain whose root never comes
(layer.json refused, asset missing) leaves the source on its content
tileset after one "terrain tileset did not open" line, without a
fallback. Teardown destroys the terrain tileset first (cesium detaches
each tile's rasters before freeing it), then the content tileset, and
pumps until both are gone, so network loss and a source switch take
both down.

ion:// builds the terrain from terrain=<id> and drapes an IonRasterOverlay
for imagery=<id>; a refused spec still logs its one line before the
token is read. build_externals now gives every source a credit system:
overlays create credits only through one, and cesium dereferences it
unchecked for a layer.json terrain. environment_source_credits, a free
function in scene.h (no kSceneVersion bump), returns the overlays'
credits as plain text; the node's once-a-second read is Task 12's.

MapFixtureAssetAccessor gains prefix rules, prefix holds and a network
kill, so EnvironmentStreamTerrain runs Cesium World Terrain's shape
offline: tiles with an attached raster, the ground from the terrain, the
lift on terrain tiles, a theme switch on the terrain resources,
plain-text credits, a 401 layer.json (Review Focus 5), a switch with a raster in flight after textures exist
(Review Focus 4) and network loss.

Claude-Session: <the executing session's URL>
EOF
```

Expected: `git status --short` lists exactly the six files, each as `M `; the commit succeeds.

### Task 12: End-to-end over the fixture network

**Lane:** 7, in `$INTEGRATION`: lane 7's `$WORKTREE` is `$INTEGRATION`, its ROS domain 77 with no
prefix and its test tmp dir `/tmp` (Execution, Lanes). **Depends on:** every lane 1–6 merged into
`mapbox-environment`, with the merge gate green. While this task has changes in `$INTEGRATION`,
staged or not, the orchestrator merges and commits nothing there.

**Files:**
- Modify: `overlume/tests/test_mapbox_terrain.cpp` (Task 10 created it): the include block at the
  top; one block appended after the file's last `TEST`
- Modify: `ros/src/overlume_ros/include/overlume_ros/environment_source_uri.hpp`: `<cstdint>` and
  `refresh_environment_credits`, before the namespace's closing brace
- Modify: `ros/src/overlume_ros/include/overlume_ros/overlume_node.hpp`: the members
  `environment_credits_` and `environment_credits_read_sec_`
- Modify: `ros/src/overlume_ros/src/overlume_node.cpp`: `timer_callback`'s attribution block reads
  the credits once a second and passes them to the kind and the text
- Test: `overlume/tests/test_mapbox_terrain.cpp`, suite `MapboxTerrainEndToEnd`, 8 tests
- Test: `ros/src/overlume_ros/test/test_environment_source_uri.cpp`, suite `EnvironmentCredits`,
  2 tests appended at the end

**Interfaces:**
- Consumes, verbatim from the Interface contract:
  - `overlume/src/mapbox_test_hooks.hpp` (Tasks 9, 10): `uint64_t dem_requests` and
    `uint64_t terrain_tiles_built` in `struct MapboxSourceStats`;
    `std::string mapbox_dem_url(uint32_t x14, uint32_t y14);` and
    `std::string mapbox_satellite_url(uint32_t z, uint32_t x, uint32_t y);` (no query string).
  - `overlume/src/environment_test_hooks.hpp` (Tasks 5, 11):
    `struct OverlayProbe { uint32_t rasters_attached, rasters_detached, textures_alive, textures_freed; };`,
    `OverlayProbe environment_stream_overlay_probe(overlume::VisualRenderer* r);`,
    `bool environment_stream_has_terrain_tileset(overlume::VisualRenderer* r);`,
    `uint64_t environment_terrain_tiles_loaded(overlume::VisualRenderer* r);`
  - `overlume/include/overlume/scene.h` (Task 11):
    `uint32_t environment_source_credits(VisualRenderer*, char* out, uint32_t cap);`
  - `ros/src/overlume_ros/include/overlume_ros/environment_source_uri.hpp` (Task 6):
    `AttributionKind::CREDITS`;
    `inline std::string environment_attribution_text(AttributionKind kind, const std::string& credits);`;
    `environment_attribution_kind(visible, enabled, source_uri, state, credits_non_empty)`, whose
    last argument defaults to false.
  - Task 2's fixtures: `std::vector<uint8_t> encode_dem_png(const std::function<double(double u, double v)>& height_m);`
    and `double sea_height_m(double u, double v);` in `overlume::testing::dem`
    (`overlume/tests/mapbox_dem_fixture.hpp`);
    `std::vector<uint8_t> flat_png(uint32_t size, uint8_t r, uint8_t g, uint8_t b);` in
    `overlume::testing::imagery` (`overlume/tests/imagery_fixture.hpp`).
  - Existing: `make_mapbox_fixture_network`, `mapbox_fixture_serve`,
    `mapbox_fixture_fail_transport`, `mapbox_fixture_hold`, `mapbox_fixture_release_held`,
    `mapbox_fixture_requested_urls`, `mapbox_footprint_url`, `mapbox_landmark_url`,
    `install_fixture_mapbox_source`, `mapbox_last_source_stats`, `capture_stream_glbs`,
    `captured_stream_glbs`, `mapbox_tile_point_to_map_probe` (`overlume/src/mapbox_test_hooks.hpp:38-57`, `:75-79`);
    `environment_stream_has_disk_cache`, `captured_cesium_log_text`,
    `environment_stream_provides_ground`, `renderer_ground_plane_in_scene`
    (`overlume/src/environment_test_hooks.hpp:64`, `:101`, `:105`, `:107`); `struct Feature`,
    `struct Layer`, `encode_tile`, `gzip_stored`, `building(…)` (`overlume/tests/mapbox_mvt_fixture.hpp`);
    `TileId`, `LonLatDeg`, `tile_containing`, `tile_point_to_lonlat`, `footprint_tiles_of`
    (`overlume/src/mapbox_tile_frame.hpp:20-44`); `project_to_screen`, `set_environment_source`,
    `environment_source_state` (`overlume/include/overlume/scene.h:207`, `:266-275`).
- Produces, for Tasks 13 and 14, in `namespace { namespace e2e { … } }` of the test file:
  `kToken`, `kPi`, `kWidth`, `kHeight`, `kFrameBytes`, `kAnyPose`, `kTownDir`; `struct Rgb` and the
  fixture colours `kNorthWest`, `kNorthEast`, `kSouthWest`, `kSouthEast`, `kAncestor`;
  `using Colour = std::array<double, 3>;`; `TileId op_tile();`;
  `overlume::GeoAnchor anchor_in(const TileId& t, double u, double v);`;
  `overlume::Vec3 px_to_map(const overlume::GeoAnchor& a, const TileId& t, double u, double v, double z_m);`;
  `overlume::Vec3 lowered(const overlume::Vec3& p);`;
  `overlume::VisualRenderer* make_renderer(const char* theme);`;
  `bool install(overlume::VisualRenderer* r, const std::string& uri, const overlume::GeoAnchor& a, MapboxFixtureNetwork* n);`;
  `void set_ego(overlume::VisualRenderer* r, double x, double y);`; `pump(r, pose, frame, max_frames, done)`;
  `stats()`; `overlay(r)`; `bool streamed_with_imagery(overlume::VisualRenderer* r, uint64_t tiles);`;
  `std::string credits(overlume::VisualRenderer* r);`; `log_size()`, `log_since(from)`, `count_of`,
  `contains`, `bare`, `times_requested`, `requested_satellite`; `serve_dem`, `box`, `struct PxBox`,
  `px_box`, `serve_square`; `struct ImageryTile`, `imagery_tiles`, `serve_imagery`,
  `serve_plateau_scene`; `colour_at_px`, `colour_at`, `looks_like`, `near`, `str`.
- Produces, in `environment_source_uri.hpp`, for the node and Task 15:

  ```cpp
  // Re-reads `credits` through `read` when at least a second of node time has passed since
  // `read_sec`; `read` has environment_source_credits' contract. Returns true when it read.
  template <typename ReadCredits>
  inline bool refresh_environment_credits(double now_sec, double& read_sec, std::string& credits,
                                          ReadCredits&& read);
  ```

**API notes:**
- One binary per test file: `overlume/CMakeLists.txt:585-640` globs `tests/*.cpp`, links
  `tests/golden.cpp` into each, adds `src` and stb to the include path, gives `test_mapbox_*`
  `meshoptimizer.h` alone (`:629-635`) and defines `OVERLUME_TEST_DATA_DIR`, `OVERLUME_TMP_DIR` and
  `OVERLUME_DEFAULT_THEME_DIR` (`:636-640`). Tests are C++17 (`-std=gnu++17` in the target's
  `flags.make`): no structured binding is captured by a lambda below. The block was checked with
  the toolchain's clang++ `-fsyntax-only -Wall -Wextra` as C++17 against the repo's headers plus
  the contract's declarations, with no warning, and it is a fixed point of the pinned
  clang-format 23.1.1 with the repo's `.clang-format`. Its helpers sit in `namespace e2e` so that
  they cannot collide with whatever Task 10 named its own; the test bodies qualify every helper.
- The fixture network: `MapFixtureAssetAccessor` answers by URL without its query string and
  records every URL in full (`overlume/src/environment_stream.hpp:195-249`); an answer is looked up
  when it is given, so a held URL served 401 answers 401 on release, and `release_held` clears the
  held list before answering (`overlume/src/environment_stream.cpp:549-593`). Unmatched URLs answer
  the network's status: 204 here (2xx, an empty tile, not a failure), 404 in Task 13 so a stray
  request shows.
- Failures: `CountingAssetAccessor` counts every non-2xx answer and every transport error
  (`environment_stream.hpp:138-159`); eight in a row (`kNetworkLossConsecutiveFailures`,
  `environment_stream.hpp:56`) send the source to `STREAMING_FALLBACK` at its next update
  (`environment_stream.cpp:878-903`).
- Cesium's raster overlays (cesium-native 0.64 at `overlume/build/_deps/cesium-native-src`):
  - `UrlTemplateRasterOverlay`'s `{y}` counts from the south and `{reverseY}` from the north
    (`CesiumRasterOverlays/include/CesiumRasterOverlays/UrlTemplateRasterOverlay.h:85-91`,
    `src/UrlTemplateRasterOverlay.cpp:84-93`). Mapbox's XYZ rows count from the north, so the
    fixture serves `mapbox_satellite_url(z, x, y)` with Mapbox's own y, and a mirrored template
    shows as requests outside the served pyramid and wrong quadrant colours.
  - A geometry tile maps imagery at the level its geometric error asks for, capped by
    `maximumLevel` and lowered until the tiles fit `maximumTextureSize` (2048 px by default, so at
    most 4 × 4 tiles of 512 px) (`src/QuadtreeRasterOverlayTileProvider.cpp:145-182`, `:251-330`;
    `include/CesiumRasterOverlays/RasterOverlay.h:69`). A failed imagery tile is replaced by its
    parent, level by level down to `minimumLevel` (`QuadtreeRasterOverlayTileProvider.cpp:70-135`).
    So the fixture serves the whole level 0–18 pyramid over the z14 tile: 15 ancestors and 340
    descendants.
  - A failed raster with no ready ancestor is marked ready and attached without
    `attachRasterInMainThread` (`Cesium3DTilesSelection/src/RasterMappedTo3DTile.cpp:168-178`), so
    `rasters_attached` counts real attaches only. A z14 leaf is put in the render list even while
    its raster loads (`Cesium3DTilesSelection/src/TilesetSelection.cpp:611-620`; its empty,
    unconditionally refined ancestors never kick it, `:985-992`), and the stream admits it once it
    is `Done` (`environment_stream.cpp:1124-1150`), so held imagery leaves the terrain drawn in
    `terrainColor`.
  - Per-tile imagery failures reach cesium's logger as `Received response code {} for image {url}`
    (`src/RasterOverlayTileProvider.cpp:183-196`, logged by `src/ActivatedRasterOverlay.cpp:200-210`),
    through the externals' redacting sink, which replaces the token after `access_token=`
    (`overlume/src/environment_stream.cpp:281-298`); `RasterOverlayOptions::loadErrorCallback`
    fires only when the tile provider cannot be created (`src/RasterOverlay.cpp:111-127`). No task
    emits a per-tile imagery failure line (head.md, Texts; ruling R2: tests assert the token's
    absence, never the host's), so the 401 test and the token test assert only that the token
    stays out of the log, not a line count and not the absence of URLs or of the host.
- Pixels: `render_frame` looks at `pose.target` with +z up and a 0.1–500 m depth range
  (`overlume/src/renderer.cpp:1126-1130`); `project_to_screen` returns [0, 1] with y down through
  the last frame's camera (`renderer.cpp:1210-1223`); row 0 of the buffer is the top of the picture
  (`overlume/tests/test_theme.cpp:578`). The clear colour is the theme's `palette.sky`
  (`renderer.cpp:446-449`). light_clay's sky is blue `[0.355, 0.518, 0.889]`, its ground beige
  `[0.762, 0.716, 0.672]` and its fog density 0.00025 (`overlume/assets/themes/light_clay.yaml:3-5`,
  `:39`), 40 times thinner than dark_adas's 0.010 (`dark_adas.yaml:39`): the colour probes run in
  light_clay. The renderer drops the clay plane while the source `provides_ground()`
  (`renderer.cpp:1118-1124`).
- The follow-terrain fit samples five points 10 m apart, −20 to +20 m along the ego's heading
  (`environment_stream.cpp:924-938`) and offsets the environment by
  `-(intercept - anchor height) - ground_bias` (`:974-977`): on level ground under an ego with
  heading 0 (map +x), −0.3 m. `lowered()` applies it to probe points; 0.3 m moves no probe by more
  than a pixel.
- Placement: `mapbox_tile_point_to_map_probe` maps a point in tile units, `z_m` above the anchor's
  ground, through the stream's own chain (tile frame, ECEF, map frame, height correction)
  (`overlume/src/mapbox_tile_frame.cpp:150-171`; z ≥ 14 only). The tests place every probe and
  camera with it, in DEM pixels (512 per z14 tile), so nothing depends on a metres-per-pixel
  estimate. At the operating area's tile a DEM pixel is about 4.33 m east-west and 4.31 m
  north-south (the tile is 2216.8 m by 2204.6 m on WGS84).
- Scenes (heights checked with a model of the contract's encoder: 0.1 m rounding at pixel centres,
  bilinear sampling):
  - Plateau (`serve_plateau_scene`): 0 m west of DEM column 280, a 35 % ramp to 30 m at column
    300 (about 104–190 m east of the tile centre), 30 m east of it. One 15 m block, DEM pixels
    337.5–342.5 × 239.5–244.5, so its centroid (340, 242) samples 30 m and z_ref (the tile centre)
    0 m: its top is 45 m above the anchor's ground, and `max_tile_top_m` must say so. The camera
    looks east from (150 m, 60 m, 45 m), over the ramp's top, at the block's west wall 200 m away.
    A block left at z = 0 is buried in the plateau, and the ray through the wall's spot lands on
    the plateau 550 m out, in the north-east quadrant: green.
  - Flat sea with quadrant imagery: probes 6 pixels (26 m) into the northern quadrants and 3
    (13 m) into the southern ones, all inside the clay plane's 120 m square
    (`renderer.cpp:115`), from (0, −45 m, 40 m) looking at (0, 15 m, 0).
  - Corner: the anchor 0.2 % of an edge (4.4 m) inside the operating-area tile D from its
    north-west corner; level DEMs D 0 m, west 12 m, north 6 m, north-west 18 m; a 10 m box in
    square 5 of each. z_ref is D's 0 m, so the north-west box tops out at 28 m.
  - Step: the anchor about 50 m west of the operating-area tile's east edge; that tile at 0 m, its
    east neighbour at 5 m. Skirts (20 m, Task 1) hang from both edges; seen from the low side, the
    high tile's skirt, if it faces outwards, fills the step between −0.3 m and 4.7 m. The probe row
    at 1.7 m looks through the step: without that skirt the ray passes under the high tile's
    surface, which back-face culling hides from below, to the clear colour.
- Behaviour these tests rely on from Tasks 1–11 (in the spec and the contract, not in the code at
  427f5d8): a `mapbox://` source with `terrain` runs with the ion follow-terrain defaults fixed
  (follow-terrain on, `ground_bias` 0.3, `replaces_ground` on, `max_tilt_deg` 2.0; spec §4, ruling
  R9), so it samples the terrain tileset under the ego, lowers the environment by 0.3 m on level
  ground and replaces the ground (spec §5, `provides_ground()`); `max_tile_top_m` includes the
  DEM lift (the tile's region must, spec §5 DemCache); `environment_terrain_tiles_loaded` counts
  the terrain tiles in the scene; the fallback leaves `environment_stream_has_terrain_tileset` false; the
  Mapbox satellite overlay's credit is "© Maxar" (spec §5).
- Coverage: spec §8's ion proof (`ion://…?terrain=…&imagery=…` over the fixture ion endpoints:
  terrain tiles and an attached raster) is Task 11's, in `overlume/tests/test_environment_stream.cpp`
  (head.md, File map); this suite covers the `mapbox://` items.
- gtest 1.15.2 prints a failed test's summary line as `[  FAILED  ] Suite.Name` without a time,
  which Step 4 greps. `GTEST_SKIP` returns from the function it is in and marks the test skipped.
- The node's credits read (head.md, File map row for the node files; ruling R5; the hand-off Task
  6 records, owned here):
  - `timer_callback` (`ros/src/overlume_ros/src/overlume_node.cpp:993`) advances `sim_clock_sec_`
    by `kTimerPeriodSec` (0.033 s, `overlume_node.hpp:117-118`) at `:1009`, before the attribution
    block (`:1339-1352` at 427f5d8). Task 6 leaves that block computing
    `const AttributionKind attribution = environment_attribution_kind(…);` (four lines, no fifth
    argument), then a two-line `// ponytail:` comment, then
    `const std::string attribution_text = environment_attribution_text(attribution, "");`.
    Step 6 replaces exactly those lines. The node's `.cpp` is inside `namespace overlume::ros`
    (`:18`), so the header's functions need no qualifier there.
  - `test_environment_source_uri` compiles the header alone, with no library to link
    (`ros/src/overlume_ros/CMakeLists.txt:284-287`). So the helper takes the read as a callable,
    and the node passes a lambda over `environment_source_credits`. The test file closes its
    namespaces at `:62-63`, and the tests after that qualify `overlume::ros::`. The node package
    is C++17 (`CMakeLists.txt:7`), compiled with g++.
  - `environment_source_credits` asks for the length first with `(r, nullptr, 0)`, as Task 11's
    own test does, so no fixed buffer cuts the credits.
  - The ruling on the check (Task 6's hand-off left it to the orchestrator): a node run cannot
    open a source with overlays offline, because the node has no fixture network. The check is
    therefore the pure helper's unit test, which fails when the read is dropped or the credits are
    cut. The node-level proof is Task 15's rig acceptance: "© Maxar" on the `mapbox-satellite`
    line in Steps 7 and 14, and Bing's credit for `osm-terrain` in Step 10.
  - A source switch leaves the old credits on the line for at most a second. That is acceptable,
    so there is no reset on switch.

- [ ] **Step 1: Make sure the include block holds what this block uses**

Add each missing line to its group at the top of `overlume/tests/test_mapbox_terrain.cpp`: quoted
project headers first, then `<gtest/gtest.h>`, then standard headers, each group alphabetical, as
in `test_mapbox_buildings.cpp`.

```cpp
#include "overlume/api.h"
#include "overlume/scene.h"

#include "environment_test_hooks.hpp"
#include "golden.hpp"
#include "imagery_fixture.hpp"
#include "mapbox_dem_fixture.hpp"
#include "mapbox_mvt_fixture.hpp"
#include "mapbox_test_hooks.hpp"
#include "mapbox_tile_frame.hpp"
#include "test_paths.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <set>
#include <string>
#include <thread>
#include <vector>
```

Check:

```bash
cd "$INTEGRATION"
for h in '"overlume/api.h"' '"overlume/scene.h"' '"environment_test_hooks.hpp"' '"golden.hpp"' \
         '"imagery_fixture.hpp"' '"mapbox_dem_fixture.hpp"' '"mapbox_mvt_fixture.hpp"' \
         '"mapbox_test_hooks.hpp"' '"mapbox_tile_frame.hpp"' '"test_paths.hpp"' '<gtest/gtest.h>' \
         '<algorithm>' '<array>' '<chrono>' '<cmath>' '<cstdint>' '<functional>' '<map>' '<set>' \
         '<string>' '<thread>' '<vector>'; do
    grep -qxF "#include $h" overlume/tests/test_mapbox_terrain.cpp || echo "missing $h"
done
```

Expected: no output.

- [ ] **Step 2: Append the end-to-end block to the end of `overlume/tests/test_mapbox_terrain.cpp`**

**Orchestrator amendment (whole-branch review F16):** run this scene twice in the same test: once as
written (terrain only), and once with `mapbox://buildings,terrain?imagery=satellite` and the fixture
network serving one flat-colour imagery tile for every level z14..z17 covering both tiles (the
`imagery::flat_png` fixture; see `SatelliteDrapesXyzTilesOnTheTerrain` for the URL shapes), because with
imagery the edges near the ego are drawn by cesium's upsampled children, whose skirts come from
clipping our skirt triangles. Both passes probe the same shared-edge row and must show no sky
pixel. Name the second pass's expectations "with imagery" in their messages.

```cpp
namespace {

// Task 12's end-to-end helpers. A namespace of their own keeps them clear of the helpers Task 10
// wrote above; Tasks 13 and 14 reuse them.
namespace e2e {

namespace dem = overlume::testing::dem;
namespace imagery = overlume::testing::imagery;
namespace mvt = overlume::testing::mvt;
using overlume::mapbox::TileId;
using overlume::testing::MapboxFixtureNetwork;

constexpr const char* kToken = "pk.test-token-not-real";
constexpr double kPi = 3.14159265358979323846;
constexpr uint32_t kWidth = 320;
constexpr uint32_t kHeight = 240;
constexpr size_t kFrameBytes = size_t{kWidth} * kHeight * 3u;
// Where the ground under the ego is level, the follow-terrain transform lowers the environment by
// the ground bias (environment_stream.cpp:974-977).
constexpr double kGroundBiasM = 0.3;
// A Mapbox source selects tiles with its area view, not the camera, so any pose loads them.
const overlume::CameraPose kAnyPose{{0.0, -8.0, 3.0}, {0.0, 0.0, 0.5}, 60.0};
const std::string kTownDir =
    std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/environment_test_town_0";
const std::string kSatellitePrefix = "https://api.mapbox.com/v4/mapbox.satellite/";

struct Rgb {
    uint8_t r, g, b;
};
// The imagery fixture paints each quadrant of the z14 tile in one colour, far apart in hue from the
// others, from light_clay's blue sky and from the clay.
constexpr Rgb kNorthWest{220, 50, 40};   // red
constexpr Rgb kNorthEast{40, 180, 50};   // green
constexpr Rgb kSouthWest{230, 200, 30};  // yellow
constexpr Rgb kSouthEast{200, 40, 210};  // magenta
constexpr Rgb kAncestor{128, 128, 128};  // levels 0-14 cover all four quadrants

using Colour = std::array<double, 3>;

// The operating area's z14 tile, 14/10712/7012.
TileId op_tile() { return overlume::mapbox::tile_containing(55.3910, 25.0803, 14); }

// The anchor at (u, v) of tile t, in fractions of its edge, heading pi/2: map +x east, +y north.
overlume::GeoAnchor anchor_in(const TileId& t, double u, double v) {
    const overlume::mapbox::LonLatDeg p = overlume::mapbox::tile_point_to_lonlat(t, u, v, 1.0);
    return overlume::GeoAnchor{p.lat, p.lon, kPi / 2.0, 0.0};
}

// Map metres of DEM pixel coordinates (u east, v south, 512 per edge) of z14 tile t, z_m metres
// above the anchor's ground, through the stream's placement chain.
overlume::Vec3 px_to_map(const overlume::GeoAnchor& a, const TileId& t, double u, double v,
                         double z_m) {
    double xyz[3] = {0.0, 0.0, 0.0};
    EXPECT_TRUE(
        overlume::testing::mapbox_tile_point_to_map_probe(a, 14, t.x, t.y, u, v, 512.0, z_m, xyz));
    return overlume::Vec3{xyz[0], xyz[1], xyz[2]};
}

overlume::Vec3 lowered(const overlume::Vec3& p) {
    return overlume::Vec3{p.x, p.y, p.z - kGroundBiasM};
}

overlume::VisualRenderer* make_renderer(const char* theme) {
    return overlume::create_renderer(overlume::RenderConfig{kWidth, kHeight, 1, kThemeDir, theme});
}

bool install(overlume::VisualRenderer* r, const std::string& uri, const overlume::GeoAnchor& a,
             MapboxFixtureNetwork* n) {
    return overlume::testing::install_fixture_mapbox_source(r, uri.c_str(), a, n, kToken);
}

void set_ego(overlume::VisualRenderer* r, double x, double y) {
    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = overlume::Vec3{x, y, 0.0};
    overlume::set_scene(r, s);
}

// Renders until done() or max_frames; the sleep lets worker-thread work land between frames.
template <typename Done>
bool pump(overlume::VisualRenderer* r, const overlume::CameraPose& pose,
          std::vector<uint8_t>& frame, int max_frames, Done done) {
    for (int i = 0; i < max_frames && !done(); ++i) {
        if (!overlume::render_frame(r, pose, {frame.data(), kWidth, kHeight})) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return done();
}

overlume::testing::MapboxSourceStats stats() {
    return overlume::testing::mapbox_last_source_stats();
}

overlume::testing::OverlayProbe overlay(overlume::VisualRenderer* r) {
    return overlume::testing::environment_stream_overlay_probe(r);
}

// Both Mapbox loaders built `tiles` z14 tiles and a satellite raster is attached.
bool streamed_with_imagery(overlume::VisualRenderer* r, uint64_t tiles) {
    const overlume::testing::MapboxSourceStats s = stats();
    return s.tiles_built >= tiles && s.terrain_tiles_built >= tiles &&
           overlay(r).rasters_attached >= 1;
}

std::string credits(overlume::VisualRenderer* r) {
    std::array<char, 512> text{};
    overlume::environment_source_credits(r, text.data(), static_cast<uint32_t>(text.size()));
    return std::string(text.data());
}

size_t log_size() { return overlume::testing::captured_cesium_log_text().size(); }

std::string log_since(size_t from) {
    const std::string text = overlume::testing::captured_cesium_log_text();
    return text.size() > from ? text.substr(from) : std::string();
}

size_t count_of(const std::string& text, const std::string& needle) {
    size_t n = 0;
    for (size_t at = text.find(needle); at != std::string::npos; at = text.find(needle, at + 1)) {
        ++n;
    }
    return n;
}

bool contains(const std::vector<uint8_t>& bytes, const std::string& needle) {
    return std::search(bytes.begin(), bytes.end(), needle.begin(), needle.end()) != bytes.end();
}

std::string bare(const std::string& url) { return url.substr(0, url.find('?')); }

size_t times_requested(MapboxFixtureNetwork* n, const std::string& url) {
    size_t count = 0;
    for (const std::string& u : overlume::testing::mapbox_fixture_requested_urls(n)) {
        if (bare(u) == url) ++count;
    }
    return count;
}

// Every satellite request, without its query string.
std::vector<std::string> requested_satellite(MapboxFixtureNetwork* n) {
    std::vector<std::string> out;
    for (const std::string& u : overlume::testing::mapbox_fixture_requested_urls(n)) {
        if (u.rfind(kSatellitePrefix, 0) == 0) out.push_back(bare(u));
    }
    return out;
}

void serve_dem(MapboxFixtureNetwork* n, const TileId& t,
               const std::function<double(double, double)>& height_m) {
    overlume::testing::mapbox_fixture_serve(n, overlume::testing::mapbox_dem_url(t.x, t.y), 200,
                                            dem::encode_dem_png(height_m));
}

// An MVT exterior ring: positive surveyor's area in tile units (y down).
std::vector<std::array<int32_t, 2>> box(int32_t x0, int32_t y0, int32_t x1, int32_t y1) {
    return {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
}

// A box u0..u1 east, v0..v1 south, in DEM pixels of its z14 tile (128 per z16 square), as an MVT
// ring in the 4096 units of the z16 square it lies in; `square` indexes footprint_tiles_of
// (row * 4 + column).
struct PxBox {
    size_t square = 0;
    std::vector<std::array<int32_t, 2>> ring;
};

PxBox px_box(double u0, double v0, double u1, double v1) {
    const int col = static_cast<int>(u0 / 128.0);
    const int row = static_cast<int>(v0 / 128.0);
    const auto unit = [](double px, int index) {
        return static_cast<int32_t>(std::lround((px - 128.0 * index) * 32.0));
    };
    return {static_cast<size_t>(row * 4 + col),
            box(unit(u0, col), unit(v0, row), unit(u1, col), unit(v1, row))};
}

// Serves z16 square `index` of z14 tile t: a building layer of `features`, gzip as Mapbox sends it.
void serve_square(MapboxFixtureNetwork* n, const TileId& t, size_t index,
                  std::vector<mvt::Feature> features) {
    const TileId square = overlume::mapbox::footprint_tiles_of(t)[index];
    mvt::Layer layer;
    layer.name = "building";
    layer.features = std::move(features);
    overlume::testing::mapbox_fixture_serve(
        n, overlume::testing::mapbox_footprint_url(square.x, square.y), 200,
        mvt::gzip_stored(mvt::encode_tile({layer})));
}

struct ImageryTile {
    std::string url;  // no query string
    Rgb colour;
};

// The satellite tiles over z14 tile t at levels 0-18: its 15 ancestors (levels 0-14) and its 340
// descendants (levels 15-18), each descendant in the colour of the quadrant of t it lies in.
// ponytail: the whole pyramid (355 URLs, five PNGs) instead of predicting the levels cesium picks;
// cesium also falls back to a failed tile's parent (QuadtreeRasterOverlayTileProvider.cpp:120-135).
// A scene whose radius reaches a neighbouring z14 tile serves that tile's pyramid too.
std::vector<ImageryTile> imagery_tiles(const TileId& t) {
    std::vector<ImageryTile> out;
    for (uint32_t z = 0; z <= 14; ++z) {
        const uint32_t shift = 14 - z;
        out.push_back(
            {overlume::testing::mapbox_satellite_url(z, t.x >> shift, t.y >> shift), kAncestor});
    }
    for (uint32_t z = 15; z <= 18; ++z) {
        const uint32_t side = 1u << (z - 14);
        const uint32_t half = side / 2;
        for (uint32_t row = 0; row < side; ++row) {
            for (uint32_t col = 0; col < side; ++col) {
                const bool north = row < half;  // XYZ rows count from the north
                const bool west = col < half;
                const Rgb colour =
                    north ? (west ? kNorthWest : kNorthEast) : (west ? kSouthWest : kSouthEast);
                out.push_back(
                    {overlume::testing::mapbox_satellite_url(z, t.x * side + col, t.y * side + row),
                     colour});
            }
        }
    }
    return out;
}

// Serves imagery_tiles(t) as 512-px PNGs and returns their URLs.
std::set<std::string> serve_imagery(MapboxFixtureNetwork* n, const TileId& t) {
    std::map<uint32_t, std::vector<uint8_t>> pngs;  // one encoding per colour
    std::set<std::string> served;
    for (const ImageryTile& tile : imagery_tiles(t)) {
        const uint32_t key = (uint32_t{tile.colour.r} << 16) | (uint32_t{tile.colour.g} << 8) |
                             uint32_t{tile.colour.b};
        auto it = pngs.find(key);
        if (it == pngs.end()) {
            it = pngs.emplace(key,
                              imagery::flat_png(512, tile.colour.r, tile.colour.g, tile.colour.b))
                     .first;
        }
        overlume::testing::mapbox_fixture_serve(n, tile.url, 200, it->second);
        served.insert(tile.url);
    }
    return served;
}

// The plateau scene on z14 tile t: 0 m west of DEM column 280, a ramp to 30 m at column 300 and a
// 30 m plateau east of it; one 15 m block about 22 m square on the plateau (DEM pixels 337.5-342.5
// east, 239.5-244.5 south: about 350 m east and 60 m north of the tile centre); quadrant imagery.
// Returns the imagery URLs served.
std::set<std::string> serve_plateau_scene(MapboxFixtureNetwork* n, const TileId& t) {
    serve_dem(n, t, [](double u, double) {
        return u < 280.0 ? 0.0 : (u < 300.0 ? (u - 280.0) * 1.5 : 30.0);
    });
    const PxBox block = px_box(337.5, 239.5, 342.5, 244.5);
    serve_square(n, t, block.square, {mvt::building(301, {block.ring}, 15.0)});
    return serve_imagery(n, t);
}

// The 3x3 mean colour around pixel (x, y) of a 320x240 RGB frame, row 0 at the top.
Colour colour_at_px(const std::vector<uint8_t>& frame, int x, int y) {
    const int cx = std::clamp(x, 1, static_cast<int>(kWidth) - 2);
    const int cy = std::clamp(y, 1, static_cast<int>(kHeight) - 2);
    Colour sum{0.0, 0.0, 0.0};
    for (int j = cy - 1; j <= cy + 1; ++j) {
        for (int i = cx - 1; i <= cx + 1; ++i) {
            const size_t at = (static_cast<size_t>(j) * kWidth + static_cast<size_t>(i)) * 3u;
            for (size_t c = 0; c < 3; ++c) sum[c] += frame[at + c];
        }
    }
    for (double& c : sum) c /= 9.0;
    return sum;
}

// The colour where map point p lands in `frame`, rendered through the renderer's current camera
// (the last render_frame's). False off-screen.
bool colour_at(overlume::VisualRenderer* r, const std::vector<uint8_t>& frame,
               const overlume::Vec3& p, Colour* out) {
    float u = 0.0f, v = 0.0f;
    if (!overlume::project_to_screen(r, p, &u, &v)) return false;
    *out = colour_at_px(frame, static_cast<int>(u * static_cast<float>(kWidth)),
                        static_cast<int>(v * static_cast<float>(kHeight)));
    return true;
}

// ponytail: a hue test, not colour science. Every channel the fixture colour sets 100 or more
// above another must stay 40 above it after lighting, fog and tone mapping. If a theme change
// narrows that margin, compare against a rendered reference instead, as
// RejectedImageryKeepsTerrainColourAndCountsTowardFallback does.
bool looks_like(const Colour& c, const Rgb& want) {
    const std::array<int, 3> w{want.r, want.g, want.b};
    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = 0; j < 3; ++j) {
            if (w[i] - w[j] >= 100 && c[i] - c[j] < 40.0) return false;
        }
    }
    return true;
}

bool near(const Colour& a, const Colour& b, double tolerance) {
    for (size_t c = 0; c < 3; ++c) {
        if (std::abs(a[c] - b[c]) > tolerance) return false;
    }
    return true;
}

std::string str(const Colour& c) {
    return "(" + std::to_string(std::lround(c[0])) + ", " + std::to_string(std::lround(c[1])) +
           ", " + std::to_string(std::lround(c[2])) + ")";
}

}

}

// Spec §8: one DEM per tile with two consumers. Around a z14 corner four tiles load; each DEM is
// fetched once though the terrain loader, the buildings loader and the anchor's z_ref read it.
TEST(MapboxTerrainEndToEnd, CornerTilesFetchEachDemOnceForBothLoaders) {
    namespace mvt = overlume::testing::mvt;
    auto* r = e2e::make_renderer("dark_adas");
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    // The ego stands just inside D's north-west corner, where A (north-west), B (north), C (west)
    // and D meet: 500 m reaches all four tiles.
    const e2e::TileId d = e2e::op_tile();
    struct Ground {
        e2e::TileId tile;
        double height_m;
        uint64_t id;
    };
    // Level DEMs: D, the anchor's tile, at 0 m, so z_ref = 0; the other three higher.
    const std::array<Ground, 4> ground{{{{14, d.x - 1, d.y - 1}, 18.0, 401},
                                        {{14, d.x, d.y - 1}, 6.0, 402},
                                        {{14, d.x - 1, d.y}, 12.0, 403},
                                        {d, 0.0, 404}}};
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    for (const Ground& g : ground) {
        const double h = g.height_m;
        e2e::serve_dem(n, g.tile, [h](double, double) { return h; });
        // One 10 m box per tile, in z16 square 5 (row 1, column 1).
        e2e::serve_square(n, g.tile, 5,
                          {mvt::building(g.id, {e2e::box(1000, 1000, 1400, 1400)}, 10.0)});
    }
    ASSERT_TRUE(e2e::install(r, "mapbox://buildings,terrain", e2e::anchor_in(d, 0.002, 0.002), n));
    e2e::set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> frame(e2e::kFrameBytes);
    // Built is not yet in the scene: the main thread prepares a tile and the stream admits it in
    // later frames, so the pump waits for both.
    const bool built = e2e::pump(r, e2e::kAnyPose, frame, 600, [&] {
        const overlume::testing::MapboxSourceStats s = e2e::stats();
        return s.tiles_built == 4 && s.terrain_tiles_built == 4 &&
               overlume::testing::environment_terrain_tiles_loaded(r) == 4;
    });
    ASSERT_TRUE(built) << "tiles_built " << e2e::stats().tiles_built << ", terrain_tiles_built "
                       << e2e::stats().terrain_tiles_built << ", terrain tiles in the scene "
                       << overlume::testing::environment_terrain_tiles_loaded(r);
    const overlume::testing::MapboxSourceStats s = e2e::stats();
    EXPECT_EQ(s.dem_requests, 4u)
        << "one DEM request per tile, though the terrain loader, the buildings loader and the "
           "anchor's z_ref all read them";
    for (const Ground& g : ground) {
        EXPECT_EQ(e2e::times_requested(n, overlume::testing::mapbox_dem_url(g.tile.x, g.tile.y)),
                  1u)
            << "DEM 14/" << g.tile.x << "/" << g.tile.y;
    }
    EXPECT_NEAR(s.max_tile_top_m, 28.0, 0.2)
        << "A's 10 m box stands on A's DEM, 18 m above the anchor's ground (z_ref is D's 0 m)";
    EXPECT_EQ(overlume::testing::environment_terrain_tiles_loaded(r), 4u);
    EXPECT_EQ(overlume::environment_source_state(r), overlume::EnvironmentSourceState::STREAMING);
    overlume::destroy_renderer(r);
}

// Spec §8: imagery requests present, the token only in query strings.
TEST(MapboxTerrainEndToEnd, ImageryRequestedWithTheTokenOnlyInQueryStrings) {
    auto* r = e2e::make_renderer("dark_adas");
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const e2e::TileId t = e2e::op_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    const std::set<std::string> served = e2e::serve_plateau_scene(n, t);
    overlume::testing::capture_stream_glbs(true);
    const size_t logFrom = e2e::log_size();
    ASSERT_TRUE(e2e::install(r, "mapbox://buildings,terrain?imagery=satellite",
                             e2e::anchor_in(t, 0.5, 0.5), n));
    EXPECT_FALSE(overlume::testing::environment_stream_has_disk_cache(r))
        << "no cache database: no Mapbox byte reaches the disk";
    e2e::set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> frame(e2e::kFrameBytes);
    ASSERT_TRUE(
        e2e::pump(r, e2e::kAnyPose, frame, 600, [&] { return e2e::streamed_with_imagery(r, 1); }));
    const std::vector<std::vector<uint8_t>> glbs = overlume::testing::captured_stream_glbs();
    overlume::testing::capture_stream_glbs(false);

    for (const std::string& url : overlume::testing::mapbox_fixture_requested_urls(n)) {
        const std::string query = std::string("?access_token=") + e2e::kToken;
        EXPECT_TRUE(url.size() > query.size() &&
                    url.compare(url.size() - query.size(), query.size(), query) == 0)
            << e2e::bare(url);
    }
    const std::vector<std::string> satellite = e2e::requested_satellite(n);
    EXPECT_FALSE(satellite.empty()) << "the overlay requested no imagery";
    for (const std::string& url : satellite) {
        EXPECT_EQ(served.count(url), 1u)
            << "not in the z14 tile's level 0-18 pyramid (a level over 18 or a mirrored row): "
            << url;
    }
    EXPECT_EQ(e2e::times_requested(n, overlume::testing::mapbox_dem_url(t.x, t.y)), 1u);
    ASSERT_FALSE(glbs.empty());
    for (const std::vector<uint8_t>& glb : glbs) {
        EXPECT_FALSE(e2e::contains(glb, e2e::kToken)) << "a request URL reached a model";
        EXPECT_FALSE(e2e::contains(glb, "access_token"));
    }
    const std::string log = e2e::log_since(logFrom);
    EXPECT_EQ(log.find(e2e::kToken), std::string::npos);
    const std::string credited = e2e::credits(r);
    EXPECT_NE(credited.find("\xC2\xA9 Maxar"), std::string::npos)
        << "credits: '" << credited << "'";
    overlume::destroy_renderer(r);
}

// Spec §8: a building on the hill with its base lifted, by stats and a pixel probe.
TEST(MapboxTerrainEndToEnd, BuildingOnThePlateauStandsOnTheDem) {
    auto* r = e2e::make_renderer("light_clay");
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const e2e::TileId t = e2e::op_tile();
    const overlume::GeoAnchor a = e2e::anchor_in(t, 0.5, 0.5);
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    e2e::serve_plateau_scene(n, t);
    ASSERT_TRUE(e2e::install(r, "mapbox://buildings,terrain?imagery=satellite", a, n));
    e2e::set_ego(r, 0.0, 0.0);
    // The block's west wall at mid-height: its base is the DEM at its centroid (30 m) less z_ref
    // (the 0 m under the anchor), so 30 + 15 / 2 m above the anchor's ground.
    const overlume::Vec3 wall = e2e::lowered(e2e::px_to_map(a, t, 337.5, 242.0, 37.5));
    // Open plateau north-west of the block, in the tile's north-east quadrant: green imagery.
    const overlume::Vec3 plateau = e2e::lowered(e2e::px_to_map(a, t, 330.0, 236.0, 30.0));
    // From 150 m east of the ego, 45 m up, over the ramp's top edge. A block left at z = 0 is
    // buried in the plateau, and the ray through the wall's spot lands on green plateau.
    const overlume::CameraPose pose{{150.0, wall.y, 45.0}, {wall.x, wall.y, wall.z}, 60.0};
    std::vector<uint8_t> frame(e2e::kFrameBytes);
    ASSERT_TRUE(e2e::pump(r, pose, frame, 600, [&] {
        return e2e::streamed_with_imagery(r, 1) &&
               overlume::testing::environment_stream_provides_ground(r);
    }));
    e2e::pump(r, pose, frame, 30, [] { return false; });
    EXPECT_NEAR(e2e::stats().max_tile_top_m, 45.0, 0.3)
        << "the block's top must sit 30 m (the plateau) plus 15 m above the anchor's ground";

    ASSERT_TRUE(overlume::render_frame(r, pose, {frame.data(), e2e::kWidth, e2e::kHeight}));
    const e2e::Colour sky = e2e::colour_at_px(frame, 160, 2);
    e2e::Colour atWall{}, atPlateau{};
    ASSERT_TRUE(e2e::colour_at(r, frame, wall, &atWall));
    ASSERT_TRUE(e2e::colour_at(r, frame, plateau, &atPlateau));
    ASSERT_TRUE(e2e::looks_like(atPlateau, e2e::kNorthEast))
        << "the plateau must show the fixture's north-east green: " << e2e::str(atPlateau);
    EXPECT_FALSE(e2e::looks_like(atWall, e2e::kNorthEast))
        << "green imagery where the block's west wall belongs: the block was not lifted onto the "
           "DEM "
        << e2e::str(atWall);
    EXPECT_FALSE(e2e::near(atWall, sky, 10.0))
        << "sky where the block's west wall belongs " << e2e::str(atWall);
    overlume::destroy_renderer(r);
}

// Spec §8: provides_ground after the ego's sample, the clay plane gone (pixel probe) and the
// fixture tile's colour at known points, one per quadrant.
TEST(MapboxTerrainEndToEnd, TerrainReplacesTheClayPlaneAndShowsEachQuadrant) {
    auto* r = e2e::make_renderer("light_clay");
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const e2e::TileId t = e2e::op_tile();
    const overlume::GeoAnchor a = e2e::anchor_in(t, 0.5, 0.5);
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    e2e::serve_dem(n, t, overlume::testing::dem::sea_height_m);
    e2e::serve_imagery(n, t);
    ASSERT_TRUE(e2e::install(r, "mapbox://terrain?imagery=satellite", a, n));
    e2e::set_ego(r, 0.0, 0.0);
    const overlume::CameraPose pose{{0.0, -45.0, 40.0}, {0.0, 15.0, 0.0}, 60.0};
    std::vector<uint8_t> frame(e2e::kFrameBytes);
    const bool drawn = e2e::pump(r, pose, frame, 600, [&] {
        return e2e::stats().terrain_tiles_built == 1 && e2e::overlay(r).rasters_attached >= 1 &&
               overlume::testing::environment_stream_provides_ground(r);
    });
    ASSERT_TRUE(drawn) << "provides_ground "
                       << overlume::testing::environment_stream_provides_ground(r);
    e2e::pump(r, pose, frame, 30, [] { return false; });
    EXPECT_FALSE(overlume::testing::renderer_ground_plane_in_scene(r))
        << "the terrain owns the ground: the clay plane must leave the scene";

    // Six DEM pixels (about 26 m) from the tile centre into the northern quadrants, three (13 m)
    // into the southern ones: all inside the clay plane's 120 m square, which would cover them.
    struct Probe {
        const char* name;
        double u, v;
        e2e::Rgb colour;
    };
    const std::array<Probe, 4> probes{{{"north-east", 262.0, 250.0, e2e::kNorthEast},
                                       {"north-west", 250.0, 250.0, e2e::kNorthWest},
                                       {"south-west", 250.0, 259.0, e2e::kSouthWest},
                                       {"south-east", 262.0, 259.0, e2e::kSouthEast}}};
    ASSERT_TRUE(overlume::render_frame(r, pose, {frame.data(), e2e::kWidth, e2e::kHeight}));
    for (const Probe& p : probes) {
        e2e::Colour c{};
        ASSERT_TRUE(e2e::colour_at(r, frame, e2e::lowered(e2e::px_to_map(a, t, p.u, p.v, 0.0)), &c))
            << p.name;
        EXPECT_TRUE(e2e::looks_like(c, p.colour))
            << p.name << " ground reads " << e2e::str(c)
            << ": the clay plane covers the terrain, or the imagery is mirrored or missing";
    }
    overlume::destroy_renderer(r);
}

// Spec §8: imagery 401 -> terrainColor and the counter. Held requests keep the terrain tile drawn
// without a raster, so the probe must match a run with imagery off. Released as 401, every
// satellite request fails; cesium walks each failed tile up to level 0
// (QuadtreeRasterOverlayTileProvider.cpp:120-135), so the run of failures passes eight and the
// source falls back.
TEST(MapboxTerrainEndToEnd, RejectedImageryKeepsTerrainColourAndCountsTowardFallback) {
    // Both runs read frame kProbeFrame (counted from create_renderer): Filament's temporal dither
    // makes a frame's bytes depend on its index.
    constexpr int kProbeFrame = 120;
    const e2e::TileId t = e2e::op_tile();
    const overlume::GeoAnchor a = e2e::anchor_in(t, 0.5, 0.5);
    const overlume::CameraPose pose{{0.0, -45.0, 40.0}, {0.0, 15.0, 0.0}, 60.0};
    const auto terrain_drawn = [](overlume::VisualRenderer* r) {
        return e2e::stats().terrain_tiles_built == 1 &&
               overlume::testing::environment_terrain_tiles_loaded(r) >= 1 &&
               overlume::testing::environment_stream_provides_ground(r);
    };
    // Renders up to frame kProbeFrame, reaching terrain_drawn first; the probe's colour then.
    const auto probe_at_frame = [&](overlume::VisualRenderer* r, e2e::Colour* out) {
        std::vector<uint8_t> frame(e2e::kFrameBytes);
        int frames = 0;
        for (; frames < kProbeFrame - 1 && !terrain_drawn(r); ++frames) {
            if (!overlume::render_frame(r, pose, {frame.data(), e2e::kWidth, e2e::kHeight})) {
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        if (!terrain_drawn(r)) return false;
        for (; frames < kProbeFrame; ++frames) {
            if (!overlume::render_frame(r, pose, {frame.data(), e2e::kWidth, e2e::kHeight})) {
                return false;
            }
        }
        return e2e::colour_at(r, frame, e2e::lowered(e2e::px_to_map(a, t, 262.0, 250.0, 0.0)), out);
    };

    e2e::Colour reference{};
    {
        auto* r = e2e::make_renderer("light_clay");
        if (!r) GTEST_SKIP() << "no GPU/EGL";
        auto* n = overlume::testing::make_mapbox_fixture_network(204);
        e2e::serve_dem(n, t, overlume::testing::dem::sea_height_m);
        ASSERT_TRUE(e2e::install(r, "mapbox://terrain", a, n));
        e2e::set_ego(r, 0.0, 0.0);
        ASSERT_TRUE(probe_at_frame(r, &reference)) << "imagery off: the terrain never drew";
        overlume::destroy_renderer(r);
    }

    auto* r = e2e::make_renderer("light_clay");
    ASSERT_NE(r, nullptr);
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    e2e::serve_dem(n, t, overlume::testing::dem::sea_height_m);
    for (const e2e::ImageryTile& tile : e2e::imagery_tiles(t)) {
        overlume::testing::mapbox_fixture_serve(n, tile.url, 401, {});
        overlume::testing::mapbox_fixture_hold(n, tile.url);
    }
    const size_t logFrom = e2e::log_size();
    ASSERT_TRUE(
        e2e::install(r, "mapbox://terrain?imagery=satellite&fallback=" + e2e::kTownDir, a, n));
    e2e::set_ego(r, 0.0, 0.0);
    e2e::Colour held{};
    ASSERT_TRUE(probe_at_frame(r, &held)) << "imagery held: the terrain never drew";
    ASSERT_FALSE(e2e::requested_satellite(n).empty()) << "no imagery request is in flight";
    EXPECT_EQ(e2e::overlay(r).rasters_attached, 0u);
    EXPECT_TRUE(e2e::near(held, reference, 6.0))
        << "before its imagery the terrain must show terrainColor: " << e2e::str(held) << " vs "
        << e2e::str(reference) << " with imagery off";

    overlume::testing::mapbox_fixture_release_held(n);
    std::vector<uint8_t> frame(e2e::kFrameBytes);
    EXPECT_TRUE(e2e::pump(r, pose, frame, 120, [&] {
        return overlume::environment_source_state(r) ==
               overlume::EnvironmentSourceState::STREAMING_FALLBACK;
    })) << "rejected imagery requests must count toward the network-loss fallback";
    EXPECT_EQ(e2e::overlay(r).rasters_attached, 0u) << "a rejected raster must never attach";
    // Cesium logs each failed raster tile with its URL; the redacting sink must strip the token.
    EXPECT_EQ(e2e::log_since(logFrom).find(e2e::kToken), std::string::npos);
    overlume::destroy_renderer(r);
}

// Spec §8: network loss -> fallback with both tilesets torn down. Everything streams; then the
// ego drives into the east neighbour, whose every request fails in transport.
TEST(MapboxTerrainEndToEnd, NetworkLossTearsDownTerrainAndImagery) {
    auto* r = e2e::make_renderer("dark_adas");
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const e2e::TileId t = e2e::op_tile();
    const e2e::TileId east{14, t.x + 1, t.y};
    const overlume::GeoAnchor a = e2e::anchor_in(t, 0.5, 0.5);
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    e2e::serve_plateau_scene(n, t);
    overlume::testing::mapbox_fixture_fail_transport(
        n, overlume::testing::mapbox_dem_url(east.x, east.y));
    overlume::testing::mapbox_fixture_fail_transport(
        n, overlume::testing::mapbox_landmark_url(east.x, east.y));
    for (const e2e::TileId& square : overlume::mapbox::footprint_tiles_of(east)) {
        overlume::testing::mapbox_fixture_fail_transport(
            n, overlume::testing::mapbox_footprint_url(square.x, square.y));
    }
    ASSERT_TRUE(e2e::install(
        r, "mapbox://buildings,terrain?imagery=satellite&fallback=" + e2e::kTownDir, a, n));
    e2e::set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> frame(e2e::kFrameBytes);
    ASSERT_TRUE(e2e::pump(r, e2e::kAnyPose, frame, 600, [&] {
        return e2e::streamed_with_imagery(r, 1) && e2e::overlay(r).textures_alive >= 1;
    }));
    ASSERT_TRUE(overlume::testing::environment_stream_has_terrain_tileset(r));
    ASSERT_FALSE(e2e::credits(r).empty());

    // The east tile's centre: more than 1100 m from the old tile, so it alone is selected, and its
    // 18 requests (16 squares, the landmark tile, the DEM) fail in a row.
    const overlume::Vec3 eastCentre = e2e::px_to_map(a, east, 256.0, 256.0, 0.0);
    e2e::set_ego(r, eastCentre.x, eastCentre.y);
    ASSERT_TRUE(e2e::pump(r, e2e::kAnyPose, frame, 300, [&] {
        return overlume::environment_source_state(r) ==
               overlume::EnvironmentSourceState::STREAMING_FALLBACK;
    })) << "transport errors on a new tile must reach the fallback";
    e2e::pump(r, e2e::kAnyPose, frame, 2, [] { return false; });
    EXPECT_FALSE(overlume::testing::environment_stream_has_terrain_tileset(r))
        << "the fallback must tear the terrain tileset down too";
    EXPECT_EQ(e2e::overlay(r).textures_alive, 0u) << "imagery textures outlived the teardown";
    EXPECT_TRUE(overlume::testing::renderer_ground_plane_in_scene(r)) << "the clay plane is back";
    EXPECT_EQ(e2e::credits(r), "") << "no overlay is open, so no credit (and no attribution)";
    overlume::destroy_renderer(r);
}

// Spec §8: the URI refusals (layer list, imagery). Each opens nothing, logs one line and requests
// nothing.
TEST(MapboxTerrainEndToEnd, RefusedLayerListsAndImageryOpenNothing) {
    auto* r = e2e::make_renderer("dark_adas");
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const overlume::GeoAnchor a = e2e::anchor_in(e2e::op_tile(), 0.5, 0.5);
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    struct Refusal {
        const char* uri;
        const char* reason;  // a part of the logged reason; "" when the contract pins none
    };
    const std::array<Refusal, 7> refusals{
        {{"mapbox://buildings,buildings", ""},
         {"mapbox://terrain,roads",
          "supports only the layers 'buildings' and 'terrain', not 'roads'"},
         {"mapbox://buildings,,terrain", ""},
         {"mapbox://", ""},
         {"mapbox://buildings?imagery=satellite", "imagery=satellite needs the 'terrain' layer"},
         {"mapbox://terrain?imagery=streets", ""},
         {"mapbox://terrain?cache=tiles", ""}}};
    for (const Refusal& f : refusals) {
        const size_t logFrom = e2e::log_size();
        EXPECT_FALSE(e2e::install(r, f.uri, a, n)) << f.uri;
        const std::string logged = e2e::log_since(logFrom);
        EXPECT_EQ(e2e::count_of(logged, "refused"), 1u) << f.uri << ": " << logged;
        EXPECT_NE(logged.find(f.reason), std::string::npos) << f.uri << ": " << logged;
    }
    e2e::set_ego(r, 0.0, 0.0);  // a source opened by mistake would now request its tiles
    std::vector<uint8_t> frame(e2e::kFrameBytes);
    e2e::pump(r, e2e::kAnyPose, frame, 3, [] { return false; });
    EXPECT_EQ(overlume::environment_source_state(r), overlume::EnvironmentSourceState::NONE);
    EXPECT_TRUE(overlume::testing::mapbox_fixture_requested_urls(n).empty());
    overlume::destroy_renderer(r);
}

// Review Focus 1: neighbouring DEM tiles 5 m apart in height along their shared edge. Seen from
// the low side 60 m away, the high tile's skirt must fill the step: no sky along the edge.
TEST(MapboxTerrainEndToEnd, FiveMetreStepBetweenTilesShowsNoSkyAlongTheEdge) {
    auto* r = e2e::make_renderer("light_clay");
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const e2e::TileId west = e2e::op_tile();
    const e2e::TileId east{14, west.x + 1, west.y};
    // The ego about 50 m west of the shared edge (the tile is about 2217 m wide there).
    const overlume::GeoAnchor a = e2e::anchor_in(west, 1.0 - 50.0 / 2217.0, 0.5);
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    e2e::serve_dem(n, west, overlume::testing::dem::sea_height_m);
    e2e::serve_dem(n, east, [](double, double) { return 5.0; });
    ASSERT_TRUE(e2e::install(r, "mapbox://terrain", a, n));
    e2e::set_ego(r, 0.0, 0.0);
    const overlume::Vec3 edge = e2e::px_to_map(a, west, 512.0, 256.0, 0.0);
    const overlume::CameraPose pose{{edge.x - 60.0, edge.y, 3.0}, {edge.x, edge.y, 2.0}, 60.0};
    std::vector<uint8_t> frame(e2e::kFrameBytes);
    ASSERT_TRUE(e2e::pump(r, pose, frame, 600, [&] {
        return e2e::stats().terrain_tiles_built == 2 &&
               overlume::testing::environment_terrain_tiles_loaded(r) >= 2 &&
               overlume::testing::environment_stream_provides_ground(r);
    }));
    e2e::pump(r, pose, frame, 10, [] { return false; });
    ASSERT_TRUE(overlume::render_frame(r, pose, {frame.data(), e2e::kWidth, e2e::kHeight}));
    const e2e::Colour sky = e2e::colour_at_px(frame, 160, 2);
    // The sky reference is the top row (about 29 degrees up); the control is the low tile's ground
    // 30 m before the edge, which must not read as sky.
    e2e::Colour ground{};
    ASSERT_TRUE(e2e::colour_at(r, frame, e2e::lowered(e2e::px_to_map(a, west, 505.0, 256.0, 0.0)),
                               &ground));
    ASSERT_FALSE(e2e::near(ground, sky, 10.0)) << "the low tile's ground reads as sky";
    // The probe row: 1.7 m above the low tile's ground, inside the 5 m step, every 1.2 DEM pixels
    // (about 5 m) along 60 m of the edge.
    for (int k = -6; k <= 6; ++k) {
        e2e::Colour c{};
        const overlume::Vec3 p = e2e::px_to_map(a, west, 512.0, 256.0 + 1.2 * k, 1.7);
        ASSERT_TRUE(e2e::colour_at(r, frame, p, &c)) << "probe " << k << " off-screen";
        EXPECT_FALSE(e2e::near(c, sky, 10.0))
            << "sky through the step at probe " << k << " (y " << p.y << " m): no skirt, or one "
            << "too short or facing inwards";
    }
    overlume::destroy_renderer(r);
}
```

- [ ] **Step 3: Build and run the suite; all eight pass**

These tests check behaviour Tasks 1–11 delivered, so they pass at once; Step 4 shows each
expectation can fail.

```bash
cd "$INTEGRATION"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 --target test_mapbox_terrain
overlume/build/test_mapbox_terrain --gtest_filter='MapboxTerrainEndToEnd.*' 2>&1 | tail -3
```

Expected, with the time varying:

```
[----------] Global test environment tear-down
[==========] 8 tests from 1 test suite ran. (… ms total)
[  PASSED  ] 8 tests.
```

Other outcomes:
- `[  SKIPPED ]` lines: this machine has no GPU/EGL. Stop; these tests need the GPU rig.
- A failure is a finding against the task that owns the behaviour, never a value to adjust.
  Report it to the orchestrator with the failure text: `CornerTilesFetchEachDemOnceForBothLoaders`
  → Tasks 9 and 10 (the shared `DemCache`, re-basing); `ImageryRequestedWithTheTokenOnlyInQueryStrings`
  → Task 10 (the template, `{reverseY}`, `maximumLevel = 18`, the credit) and Task 11 (credits,
  logging); `BuildingOnThePlateauStandsOnTheDem` → Task 9; `TerrainReplacesTheClayPlaneAndShowsEachQuadrant`
  → Tasks 5 (the material and attach), 10 and 11 (sampling, `provides_ground`);
  `RejectedImageryKeepsTerrainColourAndCountsTowardFallback` → Tasks 5 and 11 (the colour before
  imagery, the counting, the token kept out of the log); `NetworkLossTearsDownTerrainAndImagery` → Task 11;
  `RefusedLayerListsAndImageryOpenNothing` → Task 4; `FiveMetreStepBetweenTilesShowsNoSkyAlongTheEdge`
  → Task 1 (edge vertices on the tile boundary, skirt depth and facing) and Task 10.

- [ ] **Step 4: Prove the expectations bite, then restore**

Each mutation edits a fresh copy of the block as a regression would leave the scene, rebuilds,
runs the suite and prints the failed tests; the last lines restore the file. The sed expressions
were checked against the block as drafted: each changes exactly one line, and each mutant
compiles.

```bash
cd "$INTEGRATION"
F=overlume/tests/test_mapbox_terrain.cpp
cp "$F" /tmp/test_mapbox_terrain.cpp.keep
run_mutant() {
    cp /tmp/test_mapbox_terrain.cpp.keep "$F"
    sed -i "$1" "$F"
    if cmp -s /tmp/test_mapbox_terrain.cpp.keep "$F"; then
        echo "mutation did not apply"
        return 1
    fi
    CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
        --target test_mapbox_terrain > /dev/null \
        && overlume/build/test_mapbox_terrain --gtest_filter='MapboxTerrainEndToEnd.*' 2>&1 \
        | grep -E '^\[  FAILED  \] MapboxTerrainEndToEnd\.[A-Za-z]+$'
}
echo "== imagery rows mirrored north-south (a {y} for {reverseY} template)"
run_mutant 's/const bool north = row < half;/const bool north = row >= half;/'
echo "== a 30 m step, deeper than the 20 m skirt"
run_mutant 's/e2e::serve_dem(n, east, \[\](double, double) { return 5\.0; });/e2e::serve_dem(n, east, [](double, double) { return 30.0; });/'
echo "== the plateau block 3 m tall, below the wall probe (as a block left buried shows)"
run_mutant 's/{mvt::building(301, {block\.ring}, 15\.0)}/{mvt::building(301, {block.ring}, 3.0)}/'
echo "== imagery answered 200 instead of 401 (a rejection the counter never sees)"
run_mutant 's/mapbox_fixture_serve(n, tile\.url, 401, {});/mapbox_fixture_serve(n, tile.url, 200, {});/'
echo "== an accepted URI among the refusals"
run_mutant 's/{"mapbox:\/\/terrain?cache=tiles", ""}/{"mapbox:\/\/terrain?cache=off", ""}/'
cp /tmp/test_mapbox_terrain.cpp.keep "$F"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_mapbox_terrain > /dev/null
overlume/build/test_mapbox_terrain --gtest_filter='MapboxTerrainEndToEnd.*' 2>&1 | tail -1
git diff --stat -- "$F"
```

Expected:

```
== imagery rows mirrored north-south (a {y} for {reverseY} template)
[  FAILED  ] MapboxTerrainEndToEnd.BuildingOnThePlateauStandsOnTheDem
[  FAILED  ] MapboxTerrainEndToEnd.TerrainReplacesTheClayPlaneAndShowsEachQuadrant
== a 30 m step, deeper than the 20 m skirt
[  FAILED  ] MapboxTerrainEndToEnd.FiveMetreStepBetweenTilesShowsNoSkyAlongTheEdge
== the plateau block 3 m tall, below the wall probe (as a block left buried shows)
[  FAILED  ] MapboxTerrainEndToEnd.BuildingOnThePlateauStandsOnTheDem
== imagery answered 200 instead of 401 (a rejection the counter never sees)
[  FAILED  ] MapboxTerrainEndToEnd.RejectedImageryKeepsTerrainColourAndCountsTowardFallback
== an accepted URI among the refusals
[  FAILED  ] MapboxTerrainEndToEnd.RefusedLayerListsAndImageryOpenNothing
[  PASSED  ] 8 tests.
```

then the diff stat of Step 2's addition only. A mutation that prints only its `==` header did not
fail: stop and report it. `mutation did not apply` means the block drifted from Step 2: fix the
sed expression, never the test. Behaviour reverts in the library (the skirt removed, the DEM lift
dropped, the overlay's rows flipped, transport errors uncounted) are the review gate's to try
(Execution, Per-task loop); these mutations show the test side can fail.

- [ ] **Step 5: Write the failing credits-read tests**

In `ros/src/overlume_ros/test/test_environment_source_uri.cpp`, add these two lines as a group of
their own after `#include "overlume/scene.h"`, unless Task 6 already added them:

```cpp
#include <cstdio>
#include <string>
```

Then append this block to the end of the file, after the last `TEST` (the file's namespaces close
at `:62-63`, so the block qualifies `overlume::ros::`):

```cpp

// Task 12: the node's once-a-second read of environment_source_credits into the attribution line.
// The fake reader keeps the library's contract (snprintf-like, returns the length needed).
struct FakeCredits {
    std::string text;
    uint32_t operator()(char* out, uint32_t cap) const {
        if (cap > 0) std::snprintf(out, cap, "%s", text.c_str());
        return static_cast<uint32_t>(text.size());
    }
};

TEST(EnvironmentCredits, FirstTickReadsThenOncePerSecondOfNodeTime) {
    FakeCredits library{"\xC2\xA9 Maxar"};
    std::string credits;
    double read_sec = -1.0;  // as the node's member starts
    EXPECT_TRUE(overlume::ros::refresh_environment_credits(0.25, read_sec, credits, library));
    EXPECT_EQ(credits, "\xC2\xA9 Maxar");
    library.text = "\xC2\xA9 Microsoft";
    EXPECT_FALSE(overlume::ros::refresh_environment_credits(1.0, read_sec, credits, library));
    EXPECT_EQ(credits, "\xC2\xA9 Maxar") << "read again within the second";
    EXPECT_TRUE(overlume::ros::refresh_environment_credits(1.25, read_sec, credits, library));
    EXPECT_EQ(credits, "\xC2\xA9 Microsoft");
    library.text.clear();
    EXPECT_TRUE(overlume::ros::refresh_environment_credits(2.25, read_sec, credits, library));
    EXPECT_EQ(credits, "") << "a source without overlays clears the line's credits";
}

TEST(EnvironmentCredits, LongCreditsAreReadWhole) {
    const FakeCredits library{std::string(3000, 'c')};
    std::string credits;
    double read_sec = -1.0;
    ASSERT_TRUE(overlume::ros::refresh_environment_credits(0.0, read_sec, credits, library));
    EXPECT_EQ(credits, library.text) << "cut to " << credits.size() << " bytes";
}
```

The times are exact in binary (0.25, 1.0, 1.25, 2.25), so no rounding sits on the one-second
boundary. Build and watch it fail:

```bash
cd "$INTEGRATION"
ros/colcon_build.sh overlume_ros > /tmp/task12-red.log 2>&1; echo "exit=$?"
grep -c "‘refresh_environment_credits’ is not a member of ‘overlume::ros’" /tmp/task12-red.log
```

Expected: a non-zero `exit=`, then `5` (one error per call; g++ quotes names with ‘ ’).

- [ ] **Step 6: The helper, the members and the node's read**

First check that Task 6 left what this step replaces:

```bash
cd "$INTEGRATION"
H=ros/src/overlume_ros/include/overlume_ros/environment_source_uri.hpp
N=ros/src/overlume_ros/src/overlume_node.cpp
grep -c 'bool credits_non_empty' "$H"
grep -c 'const AttributionKind attribution = environment_attribution_kind($' "$N"
grep -c 'const std::string attribution_text = environment_attribution_text(attribution, "");' "$N"
grep -c 'bool environment_attribution_warned_{false};' \
    ros/src/overlume_ros/include/overlume_ros/overlume_node.hpp
```

Expected: `1` four times. A `0` is a deviation from head.md's Node block or Task 6's hand-off.
Stop and report it to the orchestrator. Do not add the argument here: Task 6 owns
`environment_attribution_kind`.

In `ros/src/overlume_ros/include/overlume_ros/environment_source_uri.hpp`, add
`#include <cstdint>` to the standard-header group in alphabetical order, unless it is there. Then
insert this block before the namespace's closing brace (the file's last `}` line), after a blank
line:

```cpp
// The overlay credits for the attribution line, re-read when at least a second of node time has
// passed since `read_sec` (spec section 6: they change only when an overlay opens or the source
// switches, so a per-frame read buys nothing). `read` has environment_source_credits' contract:
// it writes at most `cap` bytes, NUL-terminated, and returns the length the credits need. Returns
// true when it read. Start `read_sec` at -1.0 so the first call reads.
template <typename ReadCredits>
inline bool refresh_environment_credits(double now_sec, double& read_sec, std::string& credits,
                                        ReadCredits&& read) {
    if (now_sec - read_sec < 1.0) return false;
    read_sec = now_sec;
    std::string buf(static_cast<size_t>(read(nullptr, 0u)) + 1, '\0');
    read(&buf[0], static_cast<uint32_t>(buf.size()));
    credits.assign(buf.c_str());
    return true;
}
```

In `ros/src/overlume_ros/include/overlume_ros/overlume_node.hpp`, replace

```cpp
    bool environment_attribution_warned_{false};
```

with

```cpp
    bool environment_attribution_warned_{false};
    // environment_source_credits, read once a second; sim_clock_sec_ of the last read (-1: never).
    std::string environment_credits_;
    double environment_credits_read_sec_{-1.0};
```

In `ros/src/overlume_ros/src/overlume_node.cpp` (`timer_callback`), replace Task 6's lines, from
`const AttributionKind attribution = environment_attribution_kind(` through the
`attribution_text` line. Task 6's hand-off has them as follows; the wording of its two
`// ponytail:` comment lines may differ, and both lines go too:

```cpp
    const AttributionKind attribution = environment_attribution_kind(
        environment_effectively_visible(render_mode, environment_enabled_),
        environment_attribution_, environment_source_uri_,
        overlume::environment_source_state(renderer_));
    // ponytail: no overlay credits yet; the once-a-second read of environment_source_credits()
    // will pass them here and to the kind above.
    const std::string attribution_text = environment_attribution_text(attribution, "");
```

with

```cpp
    // Overlay credits change only when an overlay opens or the source switches (spec section 6),
    // so they are read once a second of node time, not per frame.
    const auto read_credits = [this](char* out, uint32_t cap) {
        return overlume::environment_source_credits(renderer_, out, cap);
    };
    refresh_environment_credits(sim_clock_sec_, environment_credits_read_sec_, environment_credits_,
                                read_credits);
    const AttributionKind attribution = environment_attribution_kind(
        environment_effectively_visible(render_mode, environment_enabled_),
        environment_attribution_, environment_source_uri_,
        overlume::environment_source_state(renderer_), !environment_credits_.empty());
    const std::string attribution_text =
        environment_attribution_text(attribution, environment_credits_);
```

`!environment_credits_.empty()` is what turns `osm-terrain` (an ion source with Bing imagery)
into `CREDITS`. Without it the kind stays `NONE` and nothing is drawn. The rest of the block (the
`if (!attribution_text.empty())` branch and its draw calls) is Task 6's and stays.

```bash
cd "$INTEGRATION"
ros/colcon_build.sh overlume_ros 2>&1 | tail -n 3
source /opt/ros/humble/setup.bash
ros/build/overlume_ros/test_environment_source_uri --gtest_filter='EnvironmentCredits.*' | tail -n 1
ros/build/overlume_ros/test_environment_source_uri | grep -c '^\[  FAILED  \]'
```

Expected: the build ends `Summary: 1 package finished […]`, then `[  PASSED  ] 2 tests.`, then
`0`.

- [ ] **Step 7: Prove the credits tests bite, then restore**

```bash
cd "$INTEGRATION"
H=ros/src/overlume_ros/include/overlume_ros/environment_source_uri.hpp
cp "$H" /tmp/environment_source_uri.hpp.keep
run_mutant() {
    cp /tmp/environment_source_uri.hpp.keep "$H"
    sed -i "$1" "$H"
    if cmp -s /tmp/environment_source_uri.hpp.keep "$H"; then
        echo "mutation did not apply"
        return 1
    fi
    ros/colcon_build.sh overlume_ros > /dev/null 2>&1 \
        && ros/build/overlume_ros/test_environment_source_uri \
            --gtest_filter='EnvironmentCredits.*' 2>&1 \
        | grep -E '^\[  FAILED  \] EnvironmentCredits\.[A-Za-z]+$'
}
echo "== the credits never reach the line (the node's \"\" again)"
run_mutant 's/credits.assign(buf.c_str());/credits.clear();/'
echo "== read every frame"
run_mutant 's/if (now_sec - read_sec < 1\.0) return false;/if (now_sec - read_sec < 0.0) return false;/'
echo "== a fixed 1 KiB buffer"
run_mutant 's/std::string buf(static_cast<size_t>(read(nullptr, 0u)) + 1, .\\0.);/std::string buf(1024, 0);/'
cp /tmp/environment_source_uri.hpp.keep "$H"
ros/colcon_build.sh overlume_ros 2>&1 | tail -n 1
ros/build/overlume_ros/test_environment_source_uri --gtest_filter='EnvironmentCredits.*' | tail -n 1
```

Expected (checked against the block as drafted, with g++ `-std=gnu++17`: each expression changes
one line, and each mutant compiles):

```
== the credits never reach the line (the node's "" again)
[  FAILED  ] EnvironmentCredits.FirstTickReadsThenOncePerSecondOfNodeTime
[  FAILED  ] EnvironmentCredits.LongCreditsAreReadWhole
== read every frame
[  FAILED  ] EnvironmentCredits.FirstTickReadsThenOncePerSecondOfNodeTime
== a fixed 1 KiB buffer
[  FAILED  ] EnvironmentCredits.LongCreditsAreReadWhole
```

Then the restored build's `Summary` line and `[  PASSED  ] 2 tests.` The node's own call (the
lambda and the `credits_non_empty` argument) has no offline test: a node run cannot open a source
with overlays. Task 15's rig steps prove it on screen (API notes).

- [ ] **Step 8: Format**

```bash
cd "$INTEGRATION"
git add overlume/tests/test_mapbox_terrain.cpp \
        ros/src/overlume_ros/include/overlume_ros/environment_source_uri.hpp \
        ros/src/overlume_ros/include/overlume_ros/overlume_node.hpp \
        ros/src/overlume_ros/src/overlume_node.cpp \
        ros/src/overlume_ros/test/test_environment_source_uri.cpp
tools/check_format.sh --fix
git diff --stat
```

Expected: `check_format.sh: reformatted … files with clang-format 23.1.1`, then no `git diff`
output: every block was written formatted. If a file is listed, `git add` it.

- [ ] **Step 9: Gate** (foreground, Bash timeout 600000 ms, under the machine-wide gate lock;
  lane 7 uses domain 77 and no prefix)

```bash
cd "$INTEGRATION"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock tools/ci_visual_mode.sh
```

Expected: six `PASS` lines and exit 0. Compared with the gate after the last lane merge:
- the library ctest suite has 8 more entries, all ok (skipped without a GPU);
- the node tests have 2 more;
- the WS bridge, golden and example counts are unchanged.

Exit 75 means another gate held the lock for 300 s: run the same command again. Any other
non-zero exit is a gate failure.

- [ ] **Step 10: Commit**

```bash
cd "$INTEGRATION"
git add overlume/tests/test_mapbox_terrain.cpp \
        ros/src/overlume_ros/include/overlume_ros/environment_source_uri.hpp \
        ros/src/overlume_ros/include/overlume_ros/overlume_node.hpp \
        ros/src/overlume_ros/src/overlume_node.cpp \
        ros/src/overlume_ros/test/test_environment_source_uri.cpp
git commit -F - <<'EOF'
feat(terrain): end-to-end checks; the node draws the overlay credits

MapboxTerrainEndToEnd drives mapbox://buildings,terrain and
mapbox://terrain through the fixture network with synthetic DEM, MVT and
satellite tiles (spec section 8). Around a z14 corner each of four DEMs
is fetched once for the terrain loader, the buildings loader and the
anchor's z_ref. Every request carries the token only in its query
string, no model or log line holds it, and the satellite requests stay
inside the tile's level 0-18 pyramid. A block on a 30 m plateau stands
on the DEM (max_tile_top_m and a pixel probe). The terrain replaces the
clay plane and shows each imagery quadrant where it belongs. Held
imagery leaves terrainColor; released as 401 it never attaches, keeps
the token out of the log and counts toward the fallback. A network loss
tears both tilesets down. Refused layer lists and imagery keys open
nothing. A 5 m step between neighbouring tiles shows no sky along the
shared edge (Review Focus 1).

The node now reads environment_source_credits once a second of node
time (refresh_environment_credits, unit-tested in EnvironmentCredits)
and passes the credits to the attribution kind and text, so the
mapbox-satellite line carries the Maxar credit and osm-terrain draws
its imagery's credit line (spec section 6). No node test can open a
source with overlays offline; the rig rehearsal checks it on screen.

Claude-Session: <the executing session's URL>
EOF
```

---

### Task 13: Golden candidates and promotion

**Lane:** 7, in `$INTEGRATION` (domain 77, no prefix). **Depends on:** Task 12 committed (same file,
same worktree). The build writes test output to `/tmp` (`OVERLUME_TMP_DIR`,
`overlume/CMakeLists.txt:600-606`). The implementer runs Steps 1–4 and stops; the orchestrator runs
Step 5 with the user; only after the user's explicit approval does the implementer continue at
Step 6. No merge and no other commit in `$INTEGRATION` while this task has changes there.

**Files:**
- Modify: `overlume/tests/test_mapbox_terrain.cpp` (one block appended after Task 12's last test)
- Create: `overlume/tests/goldens/mapbox_terrain_synthetic_dark_adas.png` and
  `overlume/tests/goldens/mapbox_terrain_synthetic_light_clay.png` (Git LFS through
  `.gitattributes:3`; created only after the user approves the candidates)
- Test: `overlume/tests/test_mapbox_terrain.cpp`, `MapboxTerrainGolden.SyntheticHillDarkAdas` and
  `MapboxTerrainGolden.SyntheticHillLightClay`

**Interfaces:**
- Consumes: Task 12's `e2e` helpers (its Produces list): `make_renderer`, `op_tile`, `anchor_in`,
  `install`, `set_ego`, `stats`, `overlay`, `serve_imagery`, `serve_dem`, `px_box`, `serve_square`,
  `px_to_map`, `lowered`, `bare`, `colour_at_px`, `colour_at`, `looks_like`, `near`, `str`,
  `kNorthEast`, `kNorthWest`, `kWidth`, `kHeight`, `kFrameBytes`; the contract's
  `mapbox_dem_url`, `MapboxSourceStats::dem_requests` and `::terrain_tiles_built`,
  `environment_terrain_tiles_loaded`, `environment_stream_overlay_probe`; existing
  `double render_and_compare(overlume::VisualRenderer* r, const overlume::CameraPose& pose, const char* golden_png_path, const char* out_png_path);`
  and `kSsimMin` (`overlume/tests/golden.hpp:19`, `:121-122`), `make_mapbox_fixture_network`,
  `mapbox_fixture_serve`, `mapbox_fixture_requested_urls`, `mapbox_footprint_url`,
  `mapbox_landmark_url`, `footprint_tiles_of`, `environment_stream_provides_ground`.
- Produces: the two golden tests and the promoted goldens. Task 15's final gate depends on them,
  and its close-out script writes their `## Shipped` row (head.md's File map gives `docs/status.md`
  to Tasks 8 and 15 only), dated by this task's commit.

**API notes:**
- The pattern is `MapboxBuildingsGolden` (`overlume/tests/test_mapbox_buildings.cpp:992-1123`): one
  helper, two themes, the compared frame fixed since `create_renderer` (65 there) because
  Filament's temporal dither makes a frame's bytes depend on its index (`:1059-1069`), counts and
  requested URLs pinned beside the SSIM, and probes for what the SSIM misses.
- `render_and_compare` renders one more frame at the fixed 320 × 240, writes it first, and returns
  0.0 when the golden is missing (`overlume/tests/golden.cpp:68-95`); `kSsimMin` is 0.98
  (`golden.hpp:19`). Gate stage 5 counts gtest lines whose name contains `Golden`
  (`tools/ci_visual_mode.sh:153-155`).
- Terrain refinement: where the imagery has more detail than the geometry, cesium upsamples
  child tiles of the terrain once a tile and its rasters have loaded
  (`Cesium3DTilesSelection/src/TilesetContentManager.cpp:2140-2150`, `:218-300`), each loading its
  own rasters. With imagery the scene settles only after three raster stages: the z14 tile, then
  four z15 tiles, then four z16 tiles, about 144 satellite PNG decodes and nine 2048² RGBA
  textures with mipmaps on llvmpipe. `radius=150` does not shorten that chain (the ego at the tile
  centre touches all four z15 and z16 quadrants). The helper waits until the terrain tiles in the
  scene and the attached rasters have not changed for 10 frames and for one second of wall clock.
  The second is there because upsampling and decoding run on worker threads: on llvmpipe, ten
  frames (about 0.3 s) can pass between two raster stages with nothing new attached. The helper
  prints the frame it settled at and compares frame 200, which leaves a margin over a settle time
  that varies run to run. It fails if the scene settles after frame 199, rather than compare a
  frame of another dither phase.
- Quality 1 renders with FXAA, not TAA, with SSAO (`overlume/src/renderer.cpp:581-591`) and the
  sun's shadows (`ShadowOptionsForQuality`, `:606`), so a settled frame is byte-identical run to
  run (Step 4 proves it for these candidates).
- No ego is drawn: only `set_ego_model` (`overlume/include/overlume/scene.h:193`) creates one.
- gtest 1.15.2 prints a failed `EXPECT_TRUE(x)` as `Value of: x`, `Actual: false`,
  `Expected: true`; a failed `EXPECT_NEAR` as `The difference between a and b is …`; a failed
  `EXPECT_GT` as `Expected: (a) > (b), actual: … vs …`. Step 8 greps those.

**Scene.** One synthetic z14 tile, the operating area's 14/10712/7012, anchor at its centre with
heading π/2 (map +x east, +y north), the ego at the anchor. Heights below are above the anchor's
ground, from a model of the contract's encoder (0.1 m rounding at pixel centres, bilinear
sampling); z_ref, the DEM under the anchor, is 0.95 m (15 · e^(−75² / 2048) = 0.96 m before the
rounding). σ is 32 px, four cells of the terrain mesh's 8-pixel grid, so the mesh follows the DEM
to within 0.28 m (ruling R26). The top is 75 px (about 323 m) north of the anchor, so the ego
stands at the hill's south foot, where the ground rises 1.6 % and is level along its heading, and
the top is 14.05 m above it.

| Element | DEM pixels (u east, v south) | Map position | Height | In the golden |
|---|---|---|---|---|
| Hill | gaussian, peak 15 m at (256, 181), σ 32 px | peak about 323 m north of the ego | 14.05 m above z_ref at the top (15.0 − 0.95) | red west, green east of x = 0 |
| Summit block, MVT 501 | 256.5–259.0 × 179.8–182.2 | x 2–13 m, y 318–328 m | base 14.05 m (DEM 15.0 at its centroid), 8 m tall | on the summit |
| East-slope block, MVT 502 | 270.7–275.3 × 189.0–191.0 | x 64–84 m, y 280–289 m | base 11.6 m, 8 m tall; its corners' ground spans 10.94–12.16 m | on the slope: its downhill corner floats up to 0.66 m, uphill one sinks up to 0.56 m (spec §12) |
| Level block, MVT 503 | 258.2–260.0 × 250.1–251.9 | x 10–17 m, y 18–25 m | base 0.4 m, 8 m tall; its corners' ground spans 0.31–0.49 m | near the ego |
| Imagery | levels 15–18 by quadrant, 0–14 grey | | | NW red, NE green (SW yellow and SE magenta lie behind the camera's view) |

- All three blocks lie in z16 square 6 (row 1, column 2); the other 15 squares and the landmark
  tile answer 204, the DEM and every imagery tile 200, and any other request 404 and fails the
  URL check.
- The camera looks north from (0, −100 m, 35 m) at (0, 100 m, 7 m), 20° vertical field, 8° down:
  the hill fills the middle (ground 3 m or more above z_ref from row 57 to row 135 down the centre
  column), its crest is the skyline (row 57 at the centre, 62 at the edges) and the sky the top
  quarter (rows 0 to 56). The horizon is row 25 (120 − 680.6 · tan 8.0°), but the 500 m far plane
  ends the view just past the crest, so no ground shows above it. The bottom row meets the ground
  8.6 m north of the ego, so the southern quadrants stay out of the frame.
- light_clay probes (fog 0.00025 /m): the hill's flanks at (264, 215) and (248, 215), 7.33 m up
  (DEM 8.275 m there less z_ref; about ±35 m east, 177 m north), must read green and red; the
  summit block's south wall 4 m up from its base, (257.75, 182.2) at 18.05 m (14.05 + 4), must
  read neither green nor sky. A summit block left at z = 0 is buried (the ground there is 14.0 m),
  and the ray through that spot passes over the crest into the sky. dark_adas's fog (0.010 /m)
  makes colour margins unsafe and hides the hill (1 − e^(−0.010 × 423) ≈ 99 % at its top, 98 % at
  the slope block, 72 % at the level block), so its golden pins the SSIM, the counts and the URLs
  only.

- [ ] **Step 1: Make sure the include block holds what this block uses**

Add `#include <iostream>` to the standard-header group of `overlume/tests/test_mapbox_terrain.cpp`,
in alphabetical order (after `<functional>`), unless it is there; Task 12 added the rest.

```bash
cd "$INTEGRATION"
for h in '"golden.hpp"' '"mapbox_test_hooks.hpp"' '"environment_test_hooks.hpp"' '<array>' \
         '<chrono>' '<cmath>' '<iostream>' '<set>' '<string>' '<thread>' '<vector>'; do
    grep -qxF "#include $h" overlume/tests/test_mapbox_terrain.cpp || echo "missing $h"
done
```

Expected: no output.

- [ ] **Step 2: Append the golden block to the end of `overlume/tests/test_mapbox_terrain.cpp`**

```cpp
namespace {

namespace terrain_golden {

// The golden's hill: 15 m high, a gaussian of 32 DEM pixels (about 138 m: four cells of the
// terrain mesh's 8-pixel grid, which follows it to within 0.28 m) centred 75 pixels (about 323 m)
// north of the tile centre, on 0 m ground. Under the anchor (the tile centre) it is 0.95 m, so
// its top stands 14.05 m above the anchor's ground.
double hill_m(double u, double v) {
    const double du = u - 256.0;
    const double dv = v - 181.0;
    return 15.0 * std::exp(-(du * du + dv * dv) / 2048.0);
}

// Filament's temporal dither changes every frame's noise with the frame's index since
// create_renderer, so the compared frame is this one however early the scene settles. The imagery's
// three raster stages settle well before it; Step 3 prints when.
constexpr int kComparedFrame = 200;

// Present: the hill, red west and green east of the north-south line through the ego, and three 8 m
// blocks standing on the DEM: on the summit (just east of that line), on the east slope, and on
// level ground near the ego. Absent: the clay ground plane, and sky anywhere below the crest.
void synthetic_hill(const char* theme, const char* golden_png, const char* actual_png,
                    bool probe_colours) {
    namespace mvt = overlume::testing::mvt;
    auto* r = e2e::make_renderer(theme);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const e2e::TileId t = e2e::op_tile();
    const overlume::GeoAnchor a = e2e::anchor_in(t, 0.5, 0.5);
    // Unmatched URLs answer 404, so every request the scene needs is served below.
    auto* n = overlume::testing::make_mapbox_fixture_network();
    std::set<std::string> served = e2e::serve_imagery(n, t);
    e2e::serve_dem(n, t, hill_m);
    served.insert(overlume::testing::mapbox_dem_url(t.x, t.y));
    const e2e::PxBox summit = e2e::px_box(256.5, 179.8, 259.0, 182.2);
    const e2e::PxBox slope = e2e::px_box(270.7, 189.0, 275.3, 191.0);
    const e2e::PxBox level = e2e::px_box(258.2, 250.1, 260.0, 251.9);
    ASSERT_EQ(slope.square, summit.square);  // all three blocks lie in z16 square 6
    ASSERT_EQ(level.square, summit.square);
    e2e::serve_square(
        n, t, summit.square,
        {mvt::building(501, {summit.ring}, 8.0), mvt::building(502, {slope.ring}, 8.0),
         mvt::building(503, {level.ring}, 8.0)});
    const std::array<e2e::TileId, 16> squares = overlume::mapbox::footprint_tiles_of(t);
    for (size_t i = 0; i < squares.size(); ++i) {
        const std::string url = overlume::testing::mapbox_footprint_url(squares[i].x, squares[i].y);
        if (i != summit.square) overlume::testing::mapbox_fixture_serve(n, url, 204, {});
        served.insert(url);
    }
    const std::string landmark = overlume::testing::mapbox_landmark_url(t.x, t.y);
    overlume::testing::mapbox_fixture_serve(n, landmark, 204, {});
    served.insert(landmark);
    // radius=150 keeps the upsampled terrain to the tiles around the ego.
    ASSERT_TRUE(e2e::install(r, "mapbox://buildings,terrain?imagery=satellite&radius=150", a, n));
    e2e::set_ego(r, 0.0, 0.0);

    const overlume::CameraPose pose{{0.0, -100.0, 35.0}, {0.0, 100.0, 7.0}, 20.0};
    std::vector<uint8_t> frame(e2e::kFrameBytes);
    // Settled: both loaders built the tile, the ego's ground sample landed, a raster is attached,
    // and neither the terrain tiles in the scene nor the attached rasters changed for 10 frames
    // and for a second of wall clock. Upsampling and imagery decoding run on worker threads, so a
    // pause between raster stages can outlast 10 fast frames.
    int frames = 0;
    int still = 0;
    auto stillSince = std::chrono::steady_clock::now();
    bool settled = false;
    uint64_t lastTiles = 0;
    uint32_t lastRasters = 0;
    for (; frames < 1000 && !settled; ++frames) {
        ASSERT_TRUE(overlume::render_frame(r, pose, {frame.data(), e2e::kWidth, e2e::kHeight}));
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        const overlume::testing::MapboxSourceStats s = e2e::stats();
        const uint64_t tiles = overlume::testing::environment_terrain_tiles_loaded(r);
        const uint32_t rasters = e2e::overlay(r).rasters_attached;
        const bool ready = s.tiles_built == 1 && s.terrain_tiles_built == 1 && rasters >= 1 &&
                           overlume::testing::environment_stream_provides_ground(r);
        if (ready && tiles == lastTiles && rasters == lastRasters) {
            ++still;
        } else {
            still = 0;
            stillSince = std::chrono::steady_clock::now();
        }
        settled =
            still >= 10 && std::chrono::steady_clock::now() - stillSince >= std::chrono::seconds(1);
        lastTiles = tiles;
        lastRasters = rasters;
    }
    ASSERT_TRUE(settled) << "the golden scene never settled";
    std::cout << "[terrain_golden] " << theme << " settled at frame " << frames << "\n";
    // ponytail: a machine that settles after frame 199 fails here instead of comparing a frame of
    // another dither phase; raise kComparedFrame (and re-promote) if the gate machine ever does.
    ASSERT_LT(frames, kComparedFrame) << "settled at frame " << frames;
    for (; frames < kComparedFrame - 1; ++frames) {
        overlume::render_frame(r, pose, {frame.data(), e2e::kWidth, e2e::kHeight});
    }

    const overlume::testing::MapboxSourceStats stats = e2e::stats();
    EXPECT_EQ(stats.tiles_built, 1u);
    EXPECT_EQ(stats.terrain_tiles_built, 1u);
    EXPECT_EQ(stats.dem_requests, 1u) << "one DEM for both loaders";
    EXPECT_EQ(stats.extruded_features, 3u);
    EXPECT_EQ(stats.wall_quads, 12u);
    EXPECT_NEAR(stats.max_tile_top_m, 22.05, 0.5)
        << "the summit block's top: the DEM at its centroid (15.0 m) less z_ref (0.95 m) plus 8 m";
    EXPECT_EQ(overlume::environment_source_state(r), overlume::EnvironmentSourceState::STREAMING);
    for (const std::string& url : overlume::testing::mapbox_fixture_requested_urls(n)) {
        EXPECT_EQ(served.count(e2e::bare(url)), 1u) << "unexpected request " << e2e::bare(url);
    }

    const double ssim = overlume::testing::render_and_compare(r, pose, golden_png, actual_png);
    EXPECT_GT(ssim, overlume::testing::kSsimMin);

    // light_clay only (its fog is 40 times thinner than dark_adas's): the SSIM need not notice
    // mirrored imagery on a dim hill or a block buried in it, so three probes pin them.
    if (probe_colours) {
        ASSERT_TRUE(overlume::render_frame(r, pose, {frame.data(), e2e::kWidth, e2e::kHeight}));
        const e2e::Colour sky = e2e::colour_at_px(frame, 160, 2);
        e2e::Colour ne{}, nw{}, atSummit{};
        ASSERT_TRUE(
            e2e::colour_at(r, frame, e2e::lowered(e2e::px_to_map(a, t, 264.0, 215.0, 7.33)), &ne));
        ASSERT_TRUE(
            e2e::colour_at(r, frame, e2e::lowered(e2e::px_to_map(a, t, 248.0, 215.0, 7.33)), &nw));
        // The summit block's south wall, 4 m up from its base (14.05 m above the anchor's ground).
        ASSERT_TRUE(e2e::colour_at(
            r, frame, e2e::lowered(e2e::px_to_map(a, t, 257.75, 182.2, 18.05)), &atSummit));
        EXPECT_TRUE(e2e::looks_like(ne, e2e::kNorthEast))
            << "the hill's north-east flank must be green " << e2e::str(ne);
        EXPECT_TRUE(e2e::looks_like(nw, e2e::kNorthWest))
            << "the hill's north-west flank must be red " << e2e::str(nw);
        EXPECT_FALSE(e2e::looks_like(atSummit, e2e::kNorthEast))
            << "green where the summit block stands: it is buried in the hill "
            << e2e::str(atSummit);
        EXPECT_FALSE(e2e::near(atSummit, sky, 10.0))
            << "sky where the summit block stands " << e2e::str(atSummit);
    }
    overlume::destroy_renderer(r);
}

}

}

TEST(MapboxTerrainGolden, SyntheticHillDarkAdas) {
    terrain_golden::synthetic_hill(
        "dark_adas", OVERLUME_TEST_DATA_DIR "/tests/goldens/mapbox_terrain_synthetic_dark_adas.png",
        OVERLUME_TMP_DIR "/mapbox_terrain_synthetic_dark_adas_actual.png", false);
}

TEST(MapboxTerrainGolden, SyntheticHillLightClay) {
    terrain_golden::synthetic_hill(
        "light_clay",
        OVERLUME_TEST_DATA_DIR "/tests/goldens/mapbox_terrain_synthetic_light_clay.png",
        OVERLUME_TMP_DIR "/mapbox_terrain_synthetic_light_clay_actual.png", true);
}
```

Checked while this plan was written: appended after Task 12's block and before Task 14's, the file
is unchanged by the pinned clang-format 23.1.1 with the repo's `.clang-format`, and passes
`-fsyntax-only -Wall -Wextra` as C++17 with the toolchain's clang++ against the repo's headers and
the contract's declarations, with no warning.

- [ ] **Step 3: Build and run: both fail on the missing goldens and write the candidates**

```bash
cd "$INTEGRATION"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 --target test_mapbox_terrain
overlume/build/test_mapbox_terrain --gtest_filter='MapboxTerrainGolden.*'
ls -l /tmp/mapbox_terrain_synthetic_dark_adas_actual.png \
      /tmp/mapbox_terrain_synthetic_light_clay_actual.png
```

Expected, with line numbers and times varying:

```
[ RUN      ] MapboxTerrainGolden.SyntheticHillDarkAdas
[terrain_golden] dark_adas settled at frame …
…/overlume/tests/test_mapbox_terrain.cpp:…: Failure
Expected: (ssim) > (overlume::testing::kSsimMin), actual: 0 vs 0.98
[  FAILED  ] MapboxTerrainGolden.SyntheticHillDarkAdas (… ms)
[ RUN      ] MapboxTerrainGolden.SyntheticHillLightClay
[terrain_golden] light_clay settled at frame …
…/overlume/tests/test_mapbox_terrain.cpp:…: Failure
Expected: (ssim) > (overlume::testing::kSsimMin), actual: 0 vs 0.98
[  FAILED  ] MapboxTerrainGolden.SyntheticHillLightClay (… ms)
```

The SSIM lines are the only failures, and `ls` shows two non-empty PNGs. Report both settle
frames to the orchestrator for the ledger: the margin to frame 200 is what keeps the gate from
failing intermittently, so a settle frame above about 120 is worth a word. Other outcomes:
- `[  SKIPPED ]`: this machine has no GPU/EGL. Stop; candidates need the GPU rig.
- `the golden scene never settled` or `settled at frame N` as a failure: the terrain keeps
  refining past frame 199. Report it to the orchestrator with N; the fix is a plan change (a larger
  `kComparedFrame` or a smaller radius), never a skipped wait.
- A count or URL expectation fails as well: a finding against Task 9 or 10 (the counting rules of
  `MapboxSourceStats`, the DEM shared by both loaders, the re-basing). Report it; never change an
  expected count to absorb it.
- A probe fails as well: look at the light_clay candidate. If the picture shows the failure (a
  buried summit block, mirrored colours), it is a finding against Tasks 9 or 10. If the picture is
  right, this plan's probe geometry is wrong: report it to the orchestrator with the candidate.
  Never move a probe point to make it pass.

- [ ] **Step 4: Check both candidates are deterministic**

```bash
cd "$INTEGRATION"
cp /tmp/mapbox_terrain_synthetic_dark_adas_actual.png /tmp/mapbox_terrain_golden_dark_run1.png
cp /tmp/mapbox_terrain_synthetic_light_clay_actual.png /tmp/mapbox_terrain_golden_light_run1.png
overlume/build/test_mapbox_terrain --gtest_filter='MapboxTerrainGolden.*' > /dev/null 2>&1
cmp /tmp/mapbox_terrain_golden_dark_run1.png /tmp/mapbox_terrain_synthetic_dark_adas_actual.png \
    && echo "dark identical"
cmp /tmp/mapbox_terrain_golden_light_run1.png /tmp/mapbox_terrain_synthetic_light_clay_actual.png \
    && echo "light identical"
```

Expected: `dark identical` and `light identical`. If either differs, stop and find the
nondeterminism (a scene still refining at frame 200 is the likely one). A flaky candidate is never
shown or promoted.

- [ ] **Step 5: HUMAN GATE (orchestrator). Show both candidates to the user; promote nothing
  without explicit approval**

The orchestrator shows the frozen copies Step 4 proved deterministic, never the `_actual.png`
paths: any later run, a gate in `$INTEGRATION` included, rewrites those. First it records their
hashes:

```bash
sha256sum /tmp/mapbox_terrain_golden_dark_run1.png /tmp/mapbox_terrain_golden_light_run1.png \
    | tee /tmp/mapbox_terrain_golden_run1.sha256
```

Expected: two lines, `<64 hex digits>  /tmp/mapbox_terrain_golden_dark_run1.png` and the light
one. It then views both and puts them in front of the user (send them the files, or open them on
their display with `python3 overlume/tests/golden.py --show <png>`), with this checklist, for each
theme:

1. A hill rises ahead of the camera, its broad top about 320 m north of the ego and 14 m above the
   ego's ground, its crest the skyline. It is red on its west half and green on its east half,
   split along a north-south line straight ahead. The ground in front of it is the same two
   colours, red left of that line and green right of it.
2. Three clay blocks stand on the surface: one on the summit, just right of the split line, against
   the sky; one on the hill's right (east) slope, near the crest; one on level ground at the front
   right. The slope block's downhill (east) corner may show a gap under it of up to about 0.7 m and
   its uphill corner may sink up to about 0.6 m, about a pixel at that distance: the known limit of
   flat-bottomed re-basing (spec §12). Nothing floats higher, nothing is buried.
3. No clay ground plane anywhere: the imagery runs to the bottom of the frame.
4. No crack, gap or sky showing through the terrain; the sky is only above the crest.
5. dark_adas: the same scene darker, its fog thickening with distance until the hill (about 99 % at
   its top) is lost in it, so the near ground and the level block carry that golden; light_clay:
   bright, with a blue sky.

Outcomes:
- The user approves both: go to Step 6, with the hashes recorded above.
- The user asks for changes: adjust the scene or the camera in Step 2's block, rebuild and return
  to Step 3; Steps 4 and 5 then freeze and hash the new candidates. Never copy a candidate the user
  has not approved, and never overwrite a failing golden to make it pass.

- [ ] **Step 6: Promote the approved candidates**

```bash
cd "$INTEGRATION"
sha256sum -c /tmp/mapbox_terrain_golden_run1.sha256 \
    && cp /tmp/mapbox_terrain_golden_dark_run1.png \
          overlume/tests/goldens/mapbox_terrain_synthetic_dark_adas.png \
    && cp /tmp/mapbox_terrain_golden_light_run1.png \
          overlume/tests/goldens/mapbox_terrain_synthetic_light_clay.png
git add overlume/tests/goldens/mapbox_terrain_synthetic_dark_adas.png \
        overlume/tests/goldens/mapbox_terrain_synthetic_light_clay.png
git show ":overlume/tests/goldens/mapbox_terrain_synthetic_dark_adas.png" | head -1
git show ":overlume/tests/goldens/mapbox_terrain_synthetic_light_clay.png" | head -1
```

Expected: `/tmp/mapbox_terrain_golden_dark_run1.png: OK`, `/tmp/mapbox_terrain_golden_light_run1.png: OK`,
then `version https://git-lfs.github.com/spec/v1` twice: the staged blobs are LFS pointers. A
`FAILED` from `sha256sum` means the bytes are not the ones the user approved: nothing is copied,
`git add` fails on the missing files, and the candidates go back to Step 5.

- [ ] **Step 7: Re-run; both pass**

```bash
cd "$INTEGRATION"
overlume/build/test_mapbox_terrain --gtest_filter='MapboxTerrainGolden.*' | tail -1
```

Expected: `[  PASSED  ] 2 tests.` The SSIM is 1.0: the goldens are the candidates themselves.

- [ ] **Step 8: Prove the expectations bite, then restore**

The light_clay golden catches these reverts:
- Satellite rows mirrored north-south (a `{y}` template where `{reverseY}` belongs, or the rows
  served upside down): both flank probes fail and the SSIM drops. The first mutation shows it.
- Buildings not re-based onto the DEM (Task 9): the summit block is buried, `max_tile_top_m` falls
  to 8 m and the summit probe reads sky (the ray through its spot passes over the crest). The
  second mutation flattens the hill, which leaves the block at the same 0–8 m and shows both
  expectations fail.
- Changes to the terrain mesh, the material, the overlay mapping or the theme that move many
  pixels: the SSIM, in both themes.

Seams between tiles stay with Task 12's `FiveMetreStepBetweenTilesShowsNoSkyAlongTheEdge`: this
scene is one tile.

```bash
cd "$INTEGRATION"
F=overlume/tests/test_mapbox_terrain.cpp
cp "$F" /tmp/test_mapbox_terrain.cpp.keep
run_mutant() {
    cp /tmp/test_mapbox_terrain.cpp.keep "$F"
    sed -i "$1" "$F"
    if cmp -s /tmp/test_mapbox_terrain.cpp.keep "$F"; then
        echo "mutation did not apply"
        return 1
    fi
    CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
        --target test_mapbox_terrain > /dev/null \
        && overlume/build/test_mapbox_terrain \
            --gtest_filter='MapboxTerrainGolden.SyntheticHillLightClay' 2>&1 \
        | grep -E '^Value of: |^The difference between|^Expected: \(ssim\)'
}
echo "== satellite rows mirrored north-south"
run_mutant 's/const bool north = row < half;/const bool north = row >= half;/'
echo "== hill flattened: the summit block at 0-8 m"
run_mutant 's/return 15\.0 \* std::exp(-(du \* du + dv \* dv) \/ 2048\.0);/return 0.0 * std::exp(-(du * du + dv * dv) \/ 2048.0);/'
cp /tmp/test_mapbox_terrain.cpp.keep "$F"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_mapbox_terrain > /dev/null
overlume/build/test_mapbox_terrain --gtest_filter='MapboxTerrainGolden.*' | tail -1
```

Expected (the sed expressions were checked against the block as drafted: each changes exactly
one line, and each mutant compiles):

```
== satellite rows mirrored north-south
Expected: (ssim) > (overlume::testing::kSsimMin), actual: … vs 0.98
Value of: e2e::looks_like(ne, e2e::kNorthEast)
Value of: e2e::looks_like(nw, e2e::kNorthWest)
== hill flattened: the summit block at 0-8 m
The difference between stats.max_tile_top_m and 22.05 is …, which exceeds 0.5, where
Expected: (ssim) > (overlume::testing::kSsimMin), actual: … vs 0.98
Value of: e2e::near(atSummit, sky, 10.0)
[  PASSED  ] 2 tests.
```

The `Value of:` lines may come in either order within a mutation; the SSIM line sits after the
counts and before the probes, which the test checks after the SSIM. A mutation that prints only
its `==` header did not fail: stop and report it. `mutation did not apply` means the block drifted
from Step 2: fix the sed expression, never the test. The final line is the restored tests passing.

- [ ] **Step 9: Format**

```bash
cd "$INTEGRATION"
git add overlume/tests/test_mapbox_terrain.cpp \
        overlume/tests/goldens/mapbox_terrain_synthetic_dark_adas.png \
        overlume/tests/goldens/mapbox_terrain_synthetic_light_clay.png
tools/check_format.sh --fix
git diff --stat
```

Expected: `check_format.sh: reformatted … files with clang-format 23.1.1`, then no `git diff`
output. If a file is listed, `git add` it.

- [ ] **Step 10: Gate** (foreground, Bash timeout 600000 ms, under the gate lock)

```bash
cd "$INTEGRATION"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock tools/ci_visual_mode.sh
```

Expected: six `PASS` lines and exit 0. `PASS  golden suite: N ok, 0 skipped`, where N is two more
than at Task 12's gate, and every existing golden still passes, so none moved; the library ctest
suite has two more entries. The per-pixel drift audit does not apply: both goldens are new. Exit
75 means another gate held the lock for 300 s: run the same command again. Any other non-zero
exit is a gate failure.

- [ ] **Step 11: Commit the test and the goldens together**

```bash
cd "$INTEGRATION"
git add overlume/tests/test_mapbox_terrain.cpp \
        overlume/tests/goldens/mapbox_terrain_synthetic_dark_adas.png \
        overlume/tests/goldens/mapbox_terrain_synthetic_light_clay.png
git commit -F - <<'EOF'
test(terrain): synthetic-hill terrain goldens, promoted by the user

MapboxTerrainGolden renders one z14 tile served through the fixture
network in Mapbox's formats at 320x240, in dark_adas and light_clay: a
terrain-DEM hill under quadrant satellite imagery (red west, green east
of the ego) with three clay blocks re-based onto the DEM, on the summit,
the east slope and level ground, and no clay ground plane. Frame 200
since renderer creation is compared, well after the upsampled terrain
and its three raster stages have settled (10 frames and a second with
nothing new attached), because Filament's temporal dither ties a frame's
bytes to its index. The SSIM need not notice mirrored imagery or a
buried block, so the test also pins the source's counts (one DEM for
both loaders, max_tile_top_m 22.05 m) and the requested URLs, and in
light_clay probes both hill flanks and the summit block's wall. The user
approved the candidates before their hashed copies were promoted (LFS).
The Shipped row for them comes with the close-out (Task 15), which owns
docs/status.md. No existing golden moved.

Claude-Session: <the executing session's URL>
EOF
git lfs ls-files | grep mapbox_terrain_synthetic
```

Expected: the commit succeeds, and `git lfs ls-files` lists both goldens.

---

### Task 14: Opt-in live smoke

**Lane:** 7, in `$INTEGRATION` (domain 77, no prefix). **Depends on:** Task 13 committed (same file,
same worktree). Task 7's `tools/check_mapbox_token.sh` with the DEM and satellite probes is in the
branch. No merge and no other commit in `$INTEGRATION` while this task has changes there. The
implementer runs Steps 1–6 and 9; the orchestrator runs Steps 7, 8, 10 and 11 with the user,
because they reach the network with the real tokens.

**Files:**
- Modify: `overlume/tests/test_mapbox_terrain.cpp` (the include block; one block appended after
  Task 13's tests)
- Modify: `docs/plans/2026-10-15-imagery-terrain.md` (Status ledger row 14, orchestrator)
- Test: `overlume/tests/test_mapbox_terrain.cpp`, `MapboxLive.OperatingAreaTerrain` and
  `IonLive.WorldTerrainBing`

**Interfaces:**
- Consumes: `bool set_environment_source(VisualRenderer*, const char* source_uri, GeoAnchor anchor);`
  (`overlume/include/overlume/scene.h:266`), whose `mapbox://` dispatch reads `MAPBOX_TOKEN` and
  whose `ion://` dispatch reads `CESIUM_ION_TOKEN` (`overlume/src/environment.cpp:180-199`,
  `overlume/src/mapbox_loader.cpp:694-698`, `overlume/src/environment_stream.cpp:1265-1271`); Task
  12's `e2e` helpers `make_renderer`, `set_ego`, `stats`, `overlay`, `credits`, `log_size`,
  `log_since`, `kPi`, `kWidth`, `kHeight`, `kFrameBytes`; the contract's
  `environment_stream_has_terrain_tileset`, `environment_terrain_tiles_loaded`,
  `MapboxSourceStats::dem_requests` and `::terrain_tiles_built`; existing `render_and_compare`,
  `environment_loaded_chunk_count`; `tools/check_mapbox_token.sh` (Task 7), whose one stdout line
  is `PASS: token retrieves the Downtown Dubai tiles (landmark tile HTTP 200, footprint tile HTTP 200, DEM tile HTTP 200, satellite tile HTTP 200)`
  with exit 0 when all four answer 200 (FAIL, exit 1, and SKIP, exit 2, as today with the four
  codes); `overlume/scripts/check_cesium_token.sh`, whose PASS line is
  `PASS: token retrieves tileset.json for asset 96188 (endpoint 200, tileset 200)`
  (`overlume/scripts/check_cesium_token.sh:29`).
- Produces: `TEST(MapboxLive, OperatingAreaTerrain)`, `TEST(IonLive, WorldTerrainBing)` and the
  live counts in the plan's ledger.

**API notes:**
- The opt-in pattern copies sub-project 1's `MapboxLive.*` (`overlume/tests/test_mapbox_buildings.cpp:1125-1225`):
  skip unless the token, trimmed as the library trims it, is non-empty and the opt-in variable is
  `1`; pump until the work has landed and nothing changed for 3 s, or 30 s; write a PNG through
  `render_and_compare` against `/nonexistent_no_golden.png`; print counts, never a URL. The ion
  test's opt-in is `OVERLUME_LIVE_ION`, as spec §8 names it. Task 8 documents it beside
  `OVERLUME_LIVE_MAPBOX` (ruling R16); the ion perf test's `OVERLUME_LIVE_ION_PERF` stays
  separate.
- Operating area (25.0803 N, 55.3910 E): 208 m from the east edge of 14/10712/7012 and 589 m from
  its north edge, so with the 500 m radius its east neighbour loads too: two z14 tiles (sub-project
  1's live run: `tiles 2`), so two terrain tiles and two DEM requests.
- Volume, Mapbox: per z14 tile one DEM, 16 MVTs and one landmark tile (36 for two tiles); the
  satellite overlay asks 16 tiles of 512 px per z14 tile at most (`maximumTextureSize` 2048) plus
  those of the upsampled tiles near the ego, a few hundred at most; the token check adds four.
  ion: the endpoint handshakes of assets 96188, 1 and 2, `layer.json`, quantized-mesh tiles and
  Bing tiles through ion.
- No request recorder exists on the live path, so "imagery requests > 0" (spec §8) is read as
  `rasters_attached > 0`: a raster attaches only after its imagery arrived.
- libcurl honours `https_proxy` (`docs/runbooks/replay_logger_session.md` §6): pointed at a closed
  local port, every request fails in transport on the machine. Step 9 uses that for an offline red
  run. A Mapbox source then reaches `STREAMING_FALLBACK` (eight failures in a row); an ion source
  whose handshake fails stays `STREAMING` with nothing loaded (`docs/status.md` open item 18), so
  its red run fails on the terrain and raster counts, not the state.
- gtest prints `EnvironmentSourceState` as `1-byte object <02>` (STREAMING) and `<03>`
  (STREAMING_FALLBACK); the skip summary is `[  SKIPPED ] 2 tests, listed below:` and one line per
  test in run order.
- A login shell's startup files can print to stdout, so Steps 8 and 10 read each token back on
  fd 3, as `tools/validate_visual_mode.sh:177-190` does.

- [ ] **Step 1: Add the standard header this block uses**

Add `<cstdlib>` to the standard-header group of `overlume/tests/test_mapbox_terrain.cpp`, in
alphabetical order (after `<cstdint>`), unless it is there; Task 13 added `<iostream>`:

```cpp
#include <cstdlib>
```

Check:

```bash
cd "$INTEGRATION"
for h in '<cstdlib>' '<iostream>'; do
    grep -qxF "#include $h" overlume/tests/test_mapbox_terrain.cpp || echo "missing $h"
done
```

Expected: no output.

- [ ] **Step 2: Append the live tests to the end of `overlume/tests/test_mapbox_terrain.cpp`**

```cpp
namespace {

namespace live {

// The variable's value without surrounding ASCII whitespace, as the library trims a token. Never
// printed.
std::string trimmed_env(const char* name) {
    const char* value = std::getenv(name);
    const std::string s = value != nullptr ? value : "";
    const size_t first = s.find_first_not_of(" \t\n\v\f\r");
    if (first == std::string::npos) return {};
    return s.substr(first, s.find_last_not_of(" \t\n\v\f\r") - first + 1);
}

bool enabled(const char* token_name, const char* opt_in_name) {
    const char* opt_in = std::getenv(opt_in_name);
    return !trimmed_env(token_name).empty() && opt_in != nullptr && std::string(opt_in) == "1";
}

// Renders until done() holds and progress() has not changed for 3 s, or 30 s pass, then writes the
// frame to `png`. Returns the seconds waited.
template <typename Done, typename Progress>
double pump(overlume::VisualRenderer* r, const overlume::CameraPose& pose, const std::string& png,
            Done done, Progress progress) {
    std::vector<uint8_t> frame(e2e::kFrameBytes);
    const auto start = std::chrono::steady_clock::now();
    auto lastChange = start;
    uint64_t last = progress();
    while (std::chrono::steady_clock::now() - start < std::chrono::seconds(30)) {
        if (!overlume::render_frame(r, pose, {frame.data(), e2e::kWidth, e2e::kHeight})) break;
        const uint64_t now = progress();
        if (now != last) {
            last = now;
            lastChange = std::chrono::steady_clock::now();
        }
        if (done() && std::chrono::steady_clock::now() - lastChange > std::chrono::seconds(3)) {
            break;
        }
    }
    const double waitS =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    overlume::testing::render_and_compare(r, pose, "/nonexistent_no_golden.png", png.c_str());
    return waitS;
}

// The operating area (spec §9), heading pi/2: map +x east.
const overlume::GeoAnchor kOperatingArea{25.0803, 55.3910, e2e::kPi / 2.0, 0.0};
const overlume::CameraPose kLivePose{{0.0, -300.0, 200.0}, {0.0, 0.0, 0.0}, 60.0};

}

}

TEST(MapboxLive, OperatingAreaTerrain) {
    if (!live::enabled("MAPBOX_TOKEN", "OVERLUME_LIVE_MAPBOX")) {
        GTEST_SKIP() << "opt-in live-network check -- set MAPBOX_TOKEN and "
                        "OVERLUME_LIVE_MAPBOX=1 to run (never required by ctest)";
    }
    auto* r = e2e::make_renderer("dark_adas");
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const size_t logFrom = e2e::log_size();
    ASSERT_TRUE(overlume::set_environment_source(r, "mapbox://buildings,terrain?imagery=satellite",
                                                 live::kOperatingArea));
    e2e::set_ego(r, 0.0, 0.0);
    const std::string png =
        std::string(OVERLUME_TMP_DIR) + "/mapbox_live_operating_area_terrain.png";
    const double waitS = live::pump(
        r, live::kLivePose, png,
        [&] {
            return e2e::stats().terrain_tiles_built >= 2 && e2e::overlay(r).rasters_attached >= 1;
        },
        [&] {
            const overlume::testing::MapboxSourceStats s = e2e::stats();
            return s.tiles_built + s.terrain_tiles_built + e2e::overlay(r).rasters_attached +
                   overlume::testing::environment_terrain_tiles_loaded(r);
        });
    const overlume::testing::MapboxSourceStats s = e2e::stats();
    const overlume::testing::OverlayProbe o = e2e::overlay(r);
    const bool maxar = e2e::credits(r).find("\xC2\xA9 Maxar") != std::string::npos;
    // Counts only, never a URL: every request URL carries the token.
    std::cerr << "[MapboxLive] operating_area_terrain terrain_tiles_built=" << s.terrain_tiles_built
              << " dem_requests=" << s.dem_requests << " tiles_built=" << s.tiles_built
              << " extruded_features=" << s.extruded_features
              << " rasters_attached=" << o.rasters_attached
              << " textures_alive=" << o.textures_alive
              << " terrain_in_scene=" << overlume::testing::environment_terrain_tiles_loaded(r)
              << " maxar_credit=" << (maxar ? 1 : 0) << " wait_s=" << waitS << " png=" << png
              << "\n";
    EXPECT_EQ(overlume::environment_source_state(r), overlume::EnvironmentSourceState::STREAMING)
        << "fell back to baked chunks: run tools/check_mapbox_token.sh";
    EXPECT_EQ(s.terrain_tiles_built, 2u) << "the anchor's tile and its east neighbour, 208 m away";
    // At least: a transient failure on a live network retries the DEM; the count is printed above.
    EXPECT_GE(s.dem_requests, 2u) << "one DEM per tile for both loaders";
    EXPECT_GT(o.rasters_attached, 0u) << "no satellite imagery attached";
    EXPECT_TRUE(maxar) << "the credits lack the Maxar line";
    // Compares positions, so a failure prints numbers, never the token.
    EXPECT_EQ(e2e::log_since(logFrom).find(live::trimmed_env("MAPBOX_TOKEN")), std::string::npos)
        << "the token reached the log";
    overlume::destroy_renderer(r);
}

TEST(IonLive, WorldTerrainBing) {
    if (!live::enabled("CESIUM_ION_TOKEN", "OVERLUME_LIVE_ION")) {
        GTEST_SKIP() << "opt-in live-network check -- set CESIUM_ION_TOKEN and "
                        "OVERLUME_LIVE_ION=1 to run (never required by ctest)";
    }
    auto* r = e2e::make_renderer("dark_adas");
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const size_t logFrom = e2e::log_size();
    // The vcam GUI's osm-terrain source: OSM Buildings on Cesium World Terrain (asset 1) with
    // Bing Maps Aerial (asset 2).
    ASSERT_TRUE(overlume::set_environment_source(r, "ion://96188?terrain=1&imagery=2",
                                                 live::kOperatingArea));
    e2e::set_ego(r, 0.0, 0.0);
    const std::string png = std::string(OVERLUME_TMP_DIR) + "/ion_live_world_terrain_bing.png";
    const double waitS = live::pump(
        r, live::kLivePose, png,
        [&] {
            return overlume::testing::environment_terrain_tiles_loaded(r) >= 1 &&
                   e2e::overlay(r).rasters_attached >= 1;
        },
        [&] {
            return overlume::testing::environment_terrain_tiles_loaded(r) +
                   e2e::overlay(r).rasters_attached;
        });
    const bool hasTerrain = overlume::testing::environment_stream_has_terrain_tileset(r);
    const uint64_t terrainTiles = overlume::testing::environment_terrain_tiles_loaded(r);
    const overlume::testing::OverlayProbe o = e2e::overlay(r);
    const std::string credited = e2e::credits(r);
    std::cerr << "[IonLive] world_terrain_bing has_terrain=" << (hasTerrain ? 1 : 0)
              << " terrain_tiles=" << terrainTiles << " rasters_attached=" << o.rasters_attached
              << " textures_alive=" << o.textures_alive << " credit_chars=" << credited.size()
              << " in_scene=" << overlume::testing::environment_loaded_chunk_count(r)
              << " wait_s=" << waitS << " png=" << png << "\n";
    EXPECT_EQ(overlume::environment_source_state(r), overlume::EnvironmentSourceState::STREAMING);
    EXPECT_TRUE(hasTerrain)
        << "Cesium World Terrain did not open: run overlume/scripts/check_cesium_token.sh";
    EXPECT_GT(terrainTiles, 0u) << "no terrain tile reached the scene";
    EXPECT_GT(o.rasters_attached, 0u) << "no Bing imagery attached";
    EXPECT_NE(credited.find("Microsoft"), std::string::npos)
        << "ion's imagery credits lack Bing's line (whole-branch review F1): " << credited;
    EXPECT_EQ(e2e::log_since(logFrom).find(live::trimmed_env("CESIUM_ION_TOKEN")),
              std::string::npos)
        << "the token reached the log";
    overlume::destroy_renderer(r);
}
```

These catch what only real data shows: a template that does not match Mapbox's tiles (no raster
attaches), a DEM endpoint or decode that rejects real tiles (no terrain tile), a credit lost on the
way to the API, Cesium World Terrain or ion imagery that no longer opens. A source that never
streams ends in `STREAMING_FALLBACK` (Mapbox) or with no terrain (ion). Step 9 shows the failures.

- [ ] **Step 3: Build; without the opt-in both tests skip**

```bash
cd "$INTEGRATION"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 --target test_mapbox_terrain
env -u OVERLUME_LIVE_MAPBOX -u OVERLUME_LIVE_ION overlume/build/test_mapbox_terrain \
    --gtest_filter='MapboxLive.*:IonLive.*' | tail -3
env -u MAPBOX_TOKEN -u CESIUM_ION_TOKEN OVERLUME_LIVE_MAPBOX=1 OVERLUME_LIVE_ION=1 \
    overlume/build/test_mapbox_terrain --gtest_filter='MapboxLive.*:IonLive.*' | tail -3
```

Expected, twice:

```
[  SKIPPED ] 2 tests, listed below:
[  SKIPPED ] MapboxLive.OperatingAreaTerrain
[  SKIPPED ] IonLive.WorldTerrainBing
```

- [ ] **Step 4: Format**

```bash
cd "$INTEGRATION"
git add overlume/tests/test_mapbox_terrain.cpp
tools/check_format.sh --fix
git diff --stat
```

Expected: `check_format.sh: reformatted … files with clang-format 23.1.1`, then no `git diff`
output.

- [ ] **Step 5: Gate** (foreground, Bash timeout 600000 ms, under the gate lock; no network:
  neither opt-in variable is set in the gate, even where the login shell exports the tokens)

```bash
cd "$INTEGRATION"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock tools/ci_visual_mode.sh
```

Expected: six `PASS` lines and exit 0; the golden count unchanged from Task 13; the library ctest
suite two more entries, both skipped. Exit 75 means another gate held the lock for 300 s: run the
same command again. Any other non-zero exit is a gate failure.

- [ ] **Step 6: Commit**

```bash
cd "$INTEGRATION"
git add overlume/tests/test_mapbox_terrain.cpp
git commit -F - <<'EOF'
test(terrain): opt-in live smoke for Mapbox and ion terrain with Bing

MapboxLive.OperatingAreaTerrain opens
mapbox://buildings,terrain?imagery=satellite at the operating area and
IonLive.WorldTerrainBing the osm-terrain source,
ion://96188?terrain=1&imagery=2, through set_environment_source against
the real endpoints. Each pumps until its terrain and imagery have landed
and nothing changed for 3 s (30 s cap), writes a PNG to OVERLUME_TMP_DIR
and prints counts, never a URL. They assert two terrain tiles and at
least two DEM requests with a satellite raster and the Maxar credit
(Mapbox), and terrain tiles with a Bing raster and credits (ion), and
that no token reaches the log. Both skip unless their token is set and
OVERLUME_LIVE_MAPBOX=1 or OVERLUME_LIVE_ION=1, so the gate stays
network- and token-free (spec section 8).

Claude-Session: <the executing session's URL>
EOF
```

- [ ] **Step 7: HUMAN (orchestrator). Ask the user's consent for the live run**

Tell the user what the run does: it reaches `api.mapbox.com` with `MAPBOX_TOKEN` (four requests for
the token check; for the test 36 DEM, MVT and landmark requests and up to a few hundred satellite
tiles) and Cesium ion with `CESIUM_ION_TOKEN` (the token check's two requests; for the test the
handshakes of assets 96188, 1 and 2, terrain tiles and Bing tiles). It takes about two minutes,
prints counts only, and writes two PNGs of Mapbox and Bing content to `/tmp`, which never enter
the repository. The ion run also writes to the default tile cache
(`~/.cache/overlume-tile-cache`), as every `ion://` source does. Continue only on an explicit yes.

- [ ] **Step 8: Token preflight (orchestrator)**

Run the block as one command; shell state does not carry over between commands. Never print a
token.

```bash
cd "$INTEGRATION"
MAPBOX_TOKEN="$(env -u MAPBOX_TOKEN bash -lic 'printf %s "${MAPBOX_TOKEN:-}" >&3' \
    3>&1 >/dev/null 2>&1)"
CESIUM_ION_TOKEN="$(env -u CESIUM_ION_TOKEN bash -lic 'printf %s "${CESIUM_ION_TOKEN:-}" >&3' \
    3>&1 >/dev/null 2>&1)"
export MAPBOX_TOKEN CESIUM_ION_TOKEN
[[ -n "$MAPBOX_TOKEN" ]] && echo "MAPBOX_TOKEN set in the login shell" || echo "MAPBOX_TOKEN missing"
[[ -n "$CESIUM_ION_TOKEN" ]] && echo "CESIUM_ION_TOKEN set in the login shell" \
    || echo "CESIUM_ION_TOKEN missing"
tools/check_mapbox_token.sh; echo "mapbox check exit=$?"
overlume/scripts/check_cesium_token.sh; echo "cesium check exit=$?"
```

Expected:

```
MAPBOX_TOKEN set in the login shell
CESIUM_ION_TOKEN set in the login shell
PASS: token retrieves the Downtown Dubai tiles (landmark tile HTTP 200, footprint tile HTTP 200, DEM tile HTTP 200, satellite tile HTTP 200)
mapbox check exit=0
PASS: token retrieves tileset.json for asset 96188 (endpoint 200, tileset 200)
cesium check exit=0
```

On any `FAIL:` (exit 1) or `SKIP:` (exit 2), stop and follow `docs/runbooks/mapbox.md` §2 or
`docs/runbooks/cesium.md`.

- [ ] **Step 9: Red run, offline: sources that cannot stream fail both tests**

The bogus tokens and a closed local proxy port keep every request on the machine.

```bash
cd "$INTEGRATION"
env -u no_proxy -u NO_PROXY MAPBOX_TOKEN=pk.test-token-not-real \
    CESIUM_ION_TOKEN=ion.test-token-not-real OVERLUME_LIVE_MAPBOX=1 OVERLUME_LIVE_ION=1 \
    https_proxy=http://127.0.0.1:9 \
    overlume/build/test_mapbox_terrain --gtest_filter='MapboxLive.*:IonLive.*' \
    > /tmp/overlume_terrain_live_red.log 2>&1; echo "exit=$?"
grep -E '^\[(MapboxLive|IonLive)\]|Which is: 1-byte|actual: 0 vs 0|^\[  FAILED  \] (MapboxLive|IonLive)\.[A-Za-z]+$' \
    /tmp/overlume_terrain_live_red.log
grep -c -F -e 'pk.test-token-not-real' -e 'ion.test-token-not-real' /tmp/overlume_terrain_live_red.log
```

Expected after about a minute (each test pumps its full 30 s): `exit=1`. The `[MapboxLive]` line
shows `terrain_tiles_built=0 … rasters_attached=0 … maxar_credit=0` and the `[IonLive]` line
`terrain_tiles=0 rasters_attached=0`. The Mapbox test reports the state mismatch,
`Which is: 1-byte object <03>` against `Which is: 1-byte object <02>`; both report
`Expected: (o.rasters_attached) > (0u), actual: 0 vs 0`, and the ion test
`Expected: (terrainTiles) > (0u), actual: 0 vs 0`. Then come the two summary lines
`[  FAILED  ] MapboxLive.OperatingAreaTerrain` and `[  FAILED  ] IonLive.WorldTerrainBing`. The
last line is `0`: no log line carries either bogus token.

- [ ] **Step 10: Green run with the real tokens (orchestrator)**

```bash
cd "$INTEGRATION"
MAPBOX_TOKEN="$(env -u MAPBOX_TOKEN bash -lic 'printf %s "${MAPBOX_TOKEN:-}" >&3' \
    3>&1 >/dev/null 2>&1)"
CESIUM_ION_TOKEN="$(env -u CESIUM_ION_TOKEN bash -lic 'printf %s "${CESIUM_ION_TOKEN:-}" >&3' \
    3>&1 >/dev/null 2>&1)"
export MAPBOX_TOKEN CESIUM_ION_TOKEN
OVERLUME_LIVE_MAPBOX=1 OVERLUME_LIVE_ION=1 overlume/build/test_mapbox_terrain \
    --gtest_filter='MapboxLive.*:IonLive.*' > /tmp/overlume_terrain_live.log 2>&1; echo "exit=$?"
grep -E '^\[(MapboxLive|IonLive)\]|^\[  (PASSED|FAILED) +\]' /tmp/overlume_terrain_live.log
for name in MAPBOX_TOKEN CESIUM_ION_TOKEN; do
    grep -c -F -f <(printf '%s' "${!name}" | tr -d '[:space:]') \
        /tmp/overlume_terrain_live.log /tmp/overlume_terrain_live_red.log
done
```

Expected:
- `exit=0`.
- An `[MapboxLive] operating_area_terrain` line with `terrain_tiles_built=2`, `dem_requests=2`
  (more only after a transient failure's retry: the test asserts at least 2), `tiles_built=2`, `rasters_attached` above 0 and `maxar_credit=1`.
- An `[IonLive] world_terrain_bing` line with `has_terrain=1`, `terrain_tiles` and
  `rasters_attached` above 0, and `credit_chars` above 0.
- `[  PASSED  ] 2 tests.` A `[  PASSED  ] 0 tests.` means both skipped: a token was empty.
- Four lines ending `:0`, two per token: no log holds either token. The token goes to `grep` as a
  pattern file through a descriptor, trimmed of whitespace as the library trims it, so it never
  appears on a command line.

The orchestrator shows the user `/tmp/mapbox_live_operating_area_terrain.png` and
`/tmp/ion_live_world_terrain_bing.png`. They hold rendered Mapbox and Bing content: local only,
never committed.

- [ ] **Step 11: Record the counts in the plan's ledger (orchestrator), never a token**

```bash
cd "$INTEGRATION"
python3 - "$(date +%F)" "$(git log -1 --format=%h -- overlume/tests/test_mapbox_terrain.cpp)" \
    /tmp/overlume_terrain_live.log <<'EOF'
import sys

date, commit, log = sys.argv[1], sys.argv[2], sys.argv[3]
runs = {}
for line in open(log):
    if line.startswith("[MapboxLive] ") or line.startswith("[IonLive] "):
        site, *pairs = line.split()[1:]
        runs[site] = dict(pair.split("=", 1) for pair in pairs)
m = runs["operating_area_terrain"]
i = runs["world_terrain_bing"]
prefix = "| 14. Opt-in live smoke | 7 |"
row = (f"{prefix} done; live run {date} with the user's consent: operating area terrain tiles "
       f"{m['terrain_tiles_built']}, DEM requests {m['dem_requests']}, building tiles "
       f"{m['tiles_built']}, extruded {m['extruded_features']}, rasters attached "
       f"{m['rasters_attached']}, Maxar credit {'yes' if m['maxar_credit'] == '1' else 'no'}; "
       f"`osm-terrain` terrain tiles {i['terrain_tiles']}, rasters attached "
       f"{i['rasters_attached']}; `tools/check_mapbox_token.sh` and "
       f"`overlume/scripts/check_cesium_token.sh` PASS | `{commit}` |")
path = "docs/plans/2026-10-15-imagery-terrain.md"
lines = open(path).read().split("\n")
hits = [n for n, line in enumerate(lines) if line.startswith(prefix)]
assert len(hits) == 1, f"{path}: {len(hits)} ledger rows for Task 14"
lines[hits[0]] = row
open(path, "w").write("\n".join(lines))
print(row)
EOF
git add docs/plans/2026-10-15-imagery-terrain.md
git commit -F - <<'EOF'
docs(plan): record the imagery-on-terrain live smoke counts

Task 14's live run, made with the user's consent: terrain tiles, DEM
requests, building tiles, extrusions, attached rasters and the Maxar
credit at the operating area, and terrain tiles and rasters for
osm-terrain (Cesium World Terrain with Bing), from the [MapboxLive] and
[IonLive] lines, with both token checks PASS. Counts only: no token, no
URL.

Claude-Session: <the executing session's URL>
EOF
```

Expected: the new ledger row is printed, then the commit succeeds.

---

### Task 15: Rig rehearsal and close-out

**Lane:** 7, in `$INTEGRATION`. **Depends on:** Tasks 13 and 14 committed and the whole-branch
review closed (Execution, Merges). The orchestrator runs it, because the user judges the visuals
and the delivery reaches the public `origin`. `$SESSION_DIR` is the logger session to replay: the
one `tools/validate_logger_session.sh` uses by default (`$OVERLUME_SESSION`, else
`~/session_2026-09-01_13-57-00`, `tools/validate_logger_session.sh:8`), unless the user names
another. Every command block below is one command: shell state does not carry over between them.
Never print a token, never use `set -x` with tokens in the environment. The push (Step 22) and the
pull request edit (Step 23) are the user's chosen delivery (spec D5): they run only after every
earlier step passed.

**Files:**
- Modify: `docs/status.md` (two `## Shipped` rows, Task 13's goldens and the feature; open item
  23's opening, which Task 8 wrote, closed)
- Modify: `docs/plans/2026-10-15-imagery-terrain.md` (ledger row 15; a `### Live acceptance
  results` subsection)
- Modify: `docs/README.md` (the plan's entry marked closed)
- Modify, only when Step 19 finds drift: `docs/runbooks/mapbox.md`, `docs/runbooks/terrain_imagery.md`
- Outside the repository: `/tmp/overlume_terrain_rehearsal/` (helper, rig output, `results.txt`,
  the pull request bodies), `/tmp/overlume_validate/` (the rig's own logs)
- Remote: branch `mapbox-environment` on `origin`; the description of pull request #1

**Interfaces:**
- Consumes:
  - `tools/validate_logger_session.sh [SESSION_DIR] [--arm ENTRY] [--anchor-height-offset M] [-- <validate_visual_mode.sh args>]`
    (`tools/validate_logger_session.sh:16-29`)
  - the GUI and bridge entries `mapbox-satellite` → `mapbox://buildings,terrain?imagery=satellite`,
    `mapbox-original-satellite` → `mapbox://buildings,terrain?materials=original&imagery=satellite`,
    `osm-terrain` → `ion://96188?terrain=1&imagery=2` (Task 7), and `osm` → `ion://96188`
    (`tools/vcam_ws_bridge.py:92-98`)
  - `tools/check_mapbox_token.sh` (Task 7's four-code PASS line, verbatim in Task 14's Interfaces)
    and `overlume/scripts/check_cesium_token.sh`
  - the attribution line with the overlay credits (Tasks 6, 11 and 12): "© Mapbox © OpenStreetMap
    © Maxar Improve this map" with `imagery=satellite` (spec §4); for `osm-terrain`, the
    `CREDITS` line, the overlay credits alone
  - `docs/status.md` open item 23, whose opening Task 8 wrote as
    `**OPEN 2026-10-15 — imagery on terrain (sub-project 2) in progress.**`
  - node parameters `geo_datum_lat_deg`, `geo_datum_lon_deg`, `geo_datum_heading_deg`,
    `environment_source_uri`, `environment_chunks_dir`; the Downtown datum in
    `docs/runbooks/mapbox.md` §7.2's table
  - `overlume/tools/sample_diagnostics.py <seconds>`, `tools/kill_switch_proxy.py <port>`
  - the texts the Interface contract pins: the loader and source log lines, the token check's
    PASS line
- Produces: the rehearsal record (status rows, open item 23 closed, the plan's Live acceptance
  results, a closed plan), the pushed branch, and pull request #1's section for this feature.

**API notes:**
- The rig and its traps are sub-project 1's (its plan's Task 17 API notes,
  `docs/plans/2026-10-14-mapbox-buildings.md`): `kill_prior_rig` stops any node, bridge, GUI or bag
  player on the machine but not `mp_play` (`tools/validate_visual_mode.sh:72-110`,
  `tools/validate_logger_session.sh:48-51`); port 8765 must be free; both tokens are refreshed from
  a login shell on fd 3 (`tools/validate_visual_mode.sh:177-190`); the node logs to
  `/tmp/overlume_validate/overlume_node.log`; the GUI needs `DISPLAY`; the rig prints `Rig is up`
  (`validate_visual_mode.sh:409`); `--arm` retries `set_environment_source` over the bridge for
  120 s and prints `[arm] OK: environment source = …` (`validate_logger_session.sh:144-170`);
  `pgrep -f` patterns bracket one letter so they never match the shell running them.
- Node texts: `geo-anchor solved: --anchor-lat …` and
  `geo-anchor solved (geo_datum override): …` (`ros/src/overlume_ros/src/overlume_node.cpp:262`,
  `:289`); `environment source armed: '<composed uri>' (visible)` (`:795`, `:917`);
  `environment source: network loss detected -- switched to fallback dir '<dir>'` (`:1300`).
  Console lines carry `[SEVERITY] [time] [name]:`, so `awk -F'[][]'` reads the time as field 4.
- Composition: with `environment_chunks_dir` set the composer appends `fallback=` after the URI's
  own keys (`ros/src/overlume_ros/include/overlume_ros/environment_source_uri.hpp:42-46`); Task 6's
  `environment_terrain` and `environment_imagery` default to `""` and append nothing.
- The tile cache: an `ion://` source without `cache=` writes
  `${XDG_CACHE_HOME:-$HOME/.cache}/overlume-tile-cache/cesium-tiles.sqlite`
  (`overlume/src/environment_stream.cpp:184-193`, `:386-396`); a `mapbox://` source opens no cache
  (`cache` is only `off`). So before any `ion://` source runs, no file there may be newer than the
  rehearsal's start marker (acceptance 6).
- Network cut: `tools/kill_switch_proxy.py` is a CONNECT proxy that logs no URL; killing it refuses
  every new connection. Resident tiles never fail a request, so after the cut a live switch opens a
  fresh Mapbox source whose requests fail at the dead proxy; eight in a row reach the fallback.
- Downtown: `docs/runbooks/mapbox.md` §7.2 holds the datum sub-project 1 rehearsed for the default
  session (25.1952210, 55.2709401, heading 90.1350), which keeps the route on open ground near the
  Burj Khalifa. For another session, derive the datum with sub-project 1's method (its plan, Task 17
  Step 11) before Step 13.
- `render_ms`: `/overlume_node/diagnostics` carries it every frame;
  `overlume/tools/sample_diagnostics.py <seconds>` prints
  `render_ms_p50=… render_ms_p99=… render_ms_n=…` (`:41-45`). Acceptance 5 compares
  `mapbox-satellite` with `osm` at Downtown: p50 within 1.5×.
- Attribution: Task 6's `environment_attribution_kind` (`environment_source_uri.hpp`) gives
  `MAPBOX` for `mapbox://` and `GOOGLE` for an `ion://` URI with `materials=original`. It gives
  `CREDITS` for any other streaming `ion://` source only when its `credits_non_empty` argument is
  true, which Task 12's node code passes as `!environment_credits_.empty()` (head.md, Node; ruling
  R5). A `CREDITS` line is the overlays' credits alone, drawn bottom-left. So `osm-terrain` shows
  Bing's credits there (Step 10, spec acceptance 3). `osm` has no overlay and so no credits: its
  kind stays `NONE` and it draws nothing (Step 16). "© Maxar" (Steps 7 and 14) and Bing's credits
  reach the line only through Task 12's once-a-second read of `environment_source_credits` (spec
  §6). These rig steps are that read's node-level proof.
- Delivery: `origin` is public. `tools/check_publish_leaks.sh` scans added lines and commit
  messages of a range for local paths and the names in the untracked
  `.git/info/publish-denylist` (`tools/check_publish_leaks.sh:5-15`); its default range,
  `origin/main..HEAD`, covers both sub-projects. A pull request description is published too, so
  Step 23 runs the same patterns over the new body before `gh pr edit`. AGENTS.md's pre-push lint:
  `tools/check_format.sh`, `tools/check_spdx.sh`, `shellcheck --severity=error tools/*.sh`,
  `python3 tools/check_docs_links.py`.

- [ ] **Step 1: Preconditions and a current build**

```bash
cd "$INTEGRATION"
git status --short
git log --oneline -1
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
ros/colcon_build.sh 2>&1 | grep '^Summary:'
```

Expected: no `git status` output; the build finishes; `Summary: 1 package finished […]`.

- [ ] **Step 2: HUMAN. Ask before starting the rig**

Tell the user, and continue only on an explicit yes:
- The rig owns the machine: `tools/validate_visual_mode.sh` stops any running Overlume node, vcam
  bridge, GUI or bag player on it, from any checkout, and needs port 8765 and the replay's `/clock`.
- Network: Mapbox with `MAPBOX_TOKEN` (per z14 tile one DEM, 16 MVTs, one landmark tile and up to
  a few hundred satellite tiles, again after each source switch) and Cesium ion with
  `CESIUM_ION_TOKEN` (`osm-terrain` and `osm`).
- About an hour with the user at the GUI: they judge every visual check, and for 15 minutes the
  free-look camera must stay still while render time is sampled.
- The network cut, by the kill-switch proxy (the default: the node's HTTPS goes through
  `tools/kill_switch_proxy.py`, and killing it is the cut) or by the user disconnecting the
  machine when asked.
- After the rehearsal, the delivery the user chose: the branch is pushed to `origin` and pull
  request #1's description gains a section for this feature, after the leak and lint checks.

- [ ] **Step 3: Workspace and preflight**

```bash
cd "$INTEGRATION"
REHEARSAL=/tmp/overlume_terrain_rehearsal
SESSION_DIR="${SESSION_DIR:-${OVERLUME_SESSION:-$HOME/session_2026-09-01_13-57-00}}"
mkdir -p "$REHEARSAL" && : > "$REHEARSAL/start"
printf 'date: %s\nsession: %s\n' "$(date +%F)" "$(basename "$SESSION_DIR")" \
    > "$REHEARSAL/results.txt"
test -f "$SESSION_DIR/config/player.yaml" && test -f "$SESSION_DIR/config/topics.yaml" \
    && test -f "$SESSION_DIR/config/sensors_extrinsic_calib.yaml" \
    && test -d "$SESSION_DIR/transform/tf" && echo "session OK"
test -f "${MP_COLLECTOR_INSTALL:-$HOME/micropilot/micropilot_data_collector/install/setup.bash}" \
    && echo "collector OK"
[[ -n "${DISPLAY:-}" ]] && echo "DISPLAY set" || echo "DISPLAY unset"
RIG='[l]ib/overlume_ros/overlume_node|tools/[v]cam_ws_bridge\.py|tools/[v]cam_gui\.py'
pgrep -af "$RIG|[m]p_play|ros2 [b]ag play|tools/[k]ill_switch_proxy" || echo "no rig running"
ss -tlnH '( sport = :8765 or sport = :8899 )' | wc -l
MAPBOX_TOKEN="$(env -u MAPBOX_TOKEN bash -lic 'printf %s "${MAPBOX_TOKEN:-}" >&3' \
    3>&1 >/dev/null 2>&1)"
CESIUM_ION_TOKEN="$(env -u CESIUM_ION_TOKEN bash -lic 'printf %s "${CESIUM_ION_TOKEN:-}" >&3' \
    3>&1 >/dev/null 2>&1)"
export MAPBOX_TOKEN CESIUM_ION_TOKEN
m=FAIL; tools/check_mapbox_token.sh && m=PASS
c=FAIL; overlume/scripts/check_cesium_token.sh && c=PASS
printf 'token_checks: tools/check_mapbox_token.sh %s, overlume/scripts/check_cesium_token.sh %s\n' \
    "$m" "$c" | tee -a "$REHEARSAL/results.txt"
```

Expected:
- `session OK` and `collector OK` (the replay wrapper refuses to start without them,
  `tools/validate_logger_session.sh:32-35`); without either, stop and ask the user for the session
  or `MP_COLLECTOR_INSTALL`.
- `DISPLAY set`, `no rig running`, `0` (ports 8765 and 8899 free). A listed process belongs to a
  rig the user has just agreed to stop, and the rig's start stops it; a running `mp_play` must be
  stopped by hand. With `DISPLAY unset`, stop and ask the user for their display.
- `PASS: token retrieves the Downtown Dubai tiles (landmark tile HTTP 200, footprint tile HTTP 200, DEM tile HTTP 200, satellite tile HTTP 200)`,
  then `PASS: token retrieves tileset.json for asset 96188 (endpoint 200, tileset 200)`.
- `token_checks: tools/check_mapbox_token.sh PASS, overlume/scripts/check_cesium_token.sh PASS`.

On any `FAIL:` or `SKIP:` line, stop.

- [ ] **Step 4: Write the switch helper (outside the repository)**

```bash
REHEARSAL=/tmp/overlume_terrain_rehearsal
cat > "$REHEARSAL/set_env_source.py" <<'EOF'
import asyncio
import json
import sys

import websockets


async def main(entry):
    async with websockets.connect("ws://localhost:8765", open_timeout=5) as ws:
        # The bridge's wire field keeps its historical name.
        await ws.send(json.dumps({"cmd": "set_environment_source", "preset": entry}))
        async for raw in ws:
            msg = json.loads(raw)
            if msg.get("type") == "ack" and msg.get("cmd") == "set_environment_source":
                print(f"[switch] {entry}: success={msg.get('success')} {msg.get('reason', '')}")
                return 0 if msg.get("success") else 1
    return 1


sys.exit(asyncio.run(asyncio.wait_for(main(sys.argv[1]), 60)))
EOF
python3 -m py_compile "$REHEARSAL/set_env_source.py" && echo "helper OK"
```

Expected: `helper OK`. It sends what the GUI's Environment Tiles selector sends, and gives up after
60 s without an ack.

- [ ] **Step 5: Run A, the operating area: start the proxy and the rig**

(a) Start the kill-switch proxy with the Bash tool's background mode:

```bash
cd "$INTEGRATION"
python3 tools/kill_switch_proxy.py 8899 > /tmp/overlume_terrain_rehearsal/proxy.log 2>&1
```

Poll `grep -m1 listening /tmp/overlume_terrain_rehearsal/proxy.log` until it prints
`kill_switch_proxy: listening on 127.0.0.1:8899 (kill me to cut the network)`.

(b) Start the rig, also in background mode. The chunks directory gives the stream a `fallback=`
directory for the cut.

```bash
cd "$INTEGRATION"
SESSION_DIR="${SESSION_DIR:-${OVERLUME_SESSION:-$HOME/session_2026-09-01_13-57-00}}"
https_proxy=http://127.0.0.1:8899 no_proxy=127.0.0.1,localhost \
    tools/validate_logger_session.sh "$SESSION_DIR" --arm mapbox-satellite \
    --anchor-height-offset -1.8 -- \
    --param "environment_chunks_dir:=$PWD/overlume/tests/fixtures/environment_test_town_0" \
    > /tmp/overlume_terrain_rehearsal/runA.out 2>&1
```

If the user chose to disconnect the machine themselves, skip (a) and drop the two proxy variables
from (b).

- [ ] **Step 6: Wait for the arm; record the session's anchor**

Poll this with a Monitor until-loop until it prints; it also ends the wait when the wrapper has
exited.

```bash
grep -m1 -E '^\[arm\] (OK|FAIL)' /tmp/overlume_terrain_rehearsal/runA.out \
    || { ! pgrep -f '^bash tools/[v]alidate_logger_session\.sh' > /dev/null && echo "wrapper exited"; }
```

Expected: `[arm] OK: environment source = mapbox-satellite`. Earlier `[arm] refused: … geo-anchor
not solved yet …` lines are normal while the anchor solves from the replayed GPS. `wrapper exited`:
`tail -20 /tmp/overlume_terrain_rehearsal/runA.out` names the cause; fix it with the user and
restart at Step 5. `[arm] FAIL: not armed within 120 s`: poll
`grep -m1 'geo-anchor solved:' /tmp/overlume_validate/overlume_node.log` until it prints, then arm
by hand with `python3 /tmp/overlume_terrain_rehearsal/set_env_source.py mapbox-satellite`
(`[switch] mapbox-satellite: success=True`); with no `geo-anchor solved:` line within 5 minutes,
stop and read the node log.

```bash
REHEARSAL=/tmp/overlume_terrain_rehearsal
grep -E 'geo-anchor solved|environment source armed' /tmp/overlume_validate/overlume_node.log
grep -m1 -F '[launch] vcam_gui.py' "$REHEARSAL/runA.out"
grep -m1 -o 'geo-anchor solved: .*' /tmp/overlume_validate/overlume_node.log \
    | sed 's/^geo-anchor solved: /opa_anchor: /' >> "$REHEARSAL/results.txt"
NODE_PID="$(pgrep -f '[l]ib/overlume_ros/overlume_node' | head -1)"
tr '\0' '\n' < "/proc/$NODE_PID/environ" | grep -c -x 'https_proxy=http://127.0.0.1:8899'
```

Expected:
- The `geo-anchor solved:` line, and
  `environment source armed: 'mapbox://buildings,terrain?imagery=satellite&fallback=…/overlume/tests/fixtures/environment_test_town_0' (visible)`.
- `[launch] vcam_gui.py (log: /tmp/overlume_validate/vcam_gui.log)`: the GUI is up.
- `1`: the node's HTTPS goes through the proxy (read without printing its environment). With `0`,
  Step 11 would test nothing: stop and check Step 5's variables. Skip this check when the user
  disconnects instead.

- [ ] **Step 7: HUMAN. Visual checks at the operating area (mode 3, the GUI on `mapbox-satellite`)**

Ask the user to confirm, in the GUI's video:
1. Satellite imagery covers the ground, under the theme's clay extrusions, and no clay ground
   plane shows anywhere.
2. Along the whole route, the buildings stand on the ground surface: none floats above it or
   sinks into it by more than a storey (spec §12 allows a long block on a slope to float at one
   end).
3. The ego stays on the terrain surface as the route rises and falls: the terrain follows.
4. The logo sits bottom-left and "© Mapbox © OpenStreetMap © Maxar Improve this map" is
   right-aligned bottom-right, in the HUD's text colour. A line without "© Maxar" is a finding
   against Task 12's credits read (its only node-level check).

Record each verdict as one line of `results.txt`: the key, `confirmed` or `not confirmed`, and the
user's own words, never a token or a path. For example:

```bash
cat >> /tmp/overlume_terrain_rehearsal/results.txt <<'EOF'
opa_imagery: confirmed -- "satellite under the blocks, no plane"
EOF
```

The keys here are `opa_imagery`, `opa_surface`, `opa_follow` and `opa_attribution`. Later steps
record their checks the same way.

- [ ] **Step 8: HUMAN. The GUI's two Mapbox satellite entries**

Ask the user to pick `mapbox-original-satellite` in the GUI's Environment Tiles selector. Expect
Mapbox's look (warm off-white extrusions, landmark materials where Mapbox has them) on the same
satellite ground, with the attribution still on. Then ask them to pick `mapbox-satellite` again
and expect the theme's clay. Record `opa_gui_looks`.

- [ ] **Step 9: No tile-cache file while only Mapbox sources have run**

Both Mapbox satellite entries have now run; no `ion://` source has yet.

```bash
REHEARSAL=/tmp/overlume_terrain_rehearsal
CACHE_DIR="${XDG_CACHE_HOME:-$HOME/.cache}/overlume-tile-cache"
printf 'cache_files_a: %s\n' \
    "$(find "$CACHE_DIR" -type f -newer "$REHEARSAL/start" 2>/dev/null | wc -l)" \
    | tee -a "$REHEARSAL/results.txt"
```

Expected: `cache_files_a: 0`. Any other count means a Mapbox source wrote to the disk cache
(Product Terms; acceptance 6): stop and report it.

- [ ] **Step 10: HUMAN. The provider proof: `osm-terrain`**

Ask the user to pick `osm-terrain`. Expect Cesium World Terrain with Bing aerial imagery under the
OSM buildings in the theme's clay, and the ego on the terrain. Expect Bing's credits as text
bottom-left, in the HUD's text colour, with no logo: the `CREDITS` line (API notes; spec §4,
acceptance 3). No line at all is a finding against Task 12's credits read. Record
`osm_terrain_imagery` and `osm_terrain_credit`.

- [ ] **Step 11: The network cut reaches the baked fallback**

Cut, then switch to a fresh Mapbox source so that new requests fail.

```bash
REHEARSAL=/tmp/overlume_terrain_rehearsal
PROXY_PID="$(pgrep -f '^python3 tools/[k]ill_switch_proxy\.py 8899')"
kill "$PROXY_PID"
timeout 10 tail --pid="$PROXY_PID" -f /dev/null; echo "proxy wait rc=$?"
ss -tlnH '( sport = :8899 )' | wc -l
printf 'cut_method: %s\n' "kill-switch proxy killed" >> "$REHEARSAL/results.txt"
python3 "$REHEARSAL/set_env_source.py" mapbox-original-satellite
```

Expected: `proxy wait rc=0`, `0`, then `[switch] mapbox-original-satellite: success=True`. If the
user cuts the network instead, ask them to disconnect now, record
`cut_method: the user disconnected the machine`, and run only the last line.

Poll this until it prints:

```bash
grep -m1 'network loss detected' /tmp/overlume_validate/overlume_node.log
```

Expected: `environment source: network loss detected -- switched to fallback dir '…/environment_test_town_0'`.
If nothing appears within 60 s, acceptance 4 has failed: stop and report. Then:

```bash
REHEARSAL=/tmp/overlume_terrain_rehearsal
awk -F'[][]' '/environment source armed: .mapbox:\/\/buildings,terrain\?materials=original&imagery=satellite/ {armed = $4}
     /network loss detected/ {warn = $4}
     END {if (armed != "" && warn != "") printf "cut_seconds: %.2f\n", warn - armed;
          else print "cut_seconds: no WARN yet"}' /tmp/overlume_validate/overlume_node.log \
    | tee -a "$REHEARSAL/results.txt"
```

Expected: `cut_seconds:` a few seconds at most. Ask the user to confirm that the terrain and the
imagery are gone (the clay plane and the baked fallback chunks are back) and that the Mapbox logo
and text are gone; record `cut_visual` and `cut_attribution`. If they disconnected the machine,
ask them to reconnect now. The fallback's line goes unrecorded: it holds a local absolute path.

- [ ] **Step 12: Tear run A down; scan its logs for both tokens**

```bash
pkill -TERM -f '^bash tools/[v]alidate_logger_session\.sh'
for p in $(pgrep -f '^bash tools/[v]alidate_logger_session\.sh'); do
    timeout 60 tail --pid="$p" -f /dev/null
done
RIG='[l]ib/overlume_ros/overlume_node|tools/[v]cam_ws_bridge\.py|tools/[v]cam_gui\.py'
pgrep -af "$RIG|[m]p_play|tools/[k]ill_switch_proxy|^bash tools/[v]alidate_logger_session" \
    || echo "rig down"
```

Expected: `rig down` within a few seconds.

```bash
REHEARSAL=/tmp/overlume_terrain_rehearsal
LOGS=(/tmp/overlume_validate/*.log "$REHEARSAL/runA.out")
[[ -f "$REHEARSAL/proxy.log" ]] && LOGS+=("$REHEARSAL/proxy.log")
HITS=0
SCANNED=0
for name in MAPBOX_TOKEN CESIUM_ION_TOKEN; do
    login="$(env -u "$name" bash -lic "printf %s \"\${${name}:-}\" >&3" 3>&1 >/dev/null 2>&1)"
    for tok in "$login" "${!name:-}"; do
        [[ -n "$tok" ]] || continue
        SCANNED=$((SCANNED + 1))
        while IFS= read -r line; do
            echo "$line"
            HITS=$((HITS + ${line##*:}))
        done < <(grep -c -F -f <(printf '%s' "$tok" | tr -d '[:space:]') /dev/null "${LOGS[@]}")
    done
done
printf 'token_hits_a: %s\ntoken_files_a: %s\ntoken_scanned_a: %s\n' "$HITS" "${#LOGS[@]}" \
    "$SCANNED" | tee -a "$REHEARSAL/results.txt"
```

Expected: one `<file>:0` line per log and token (and `/dev/null:0`, which keeps grep's
`file:count` form), then `token_hits_a: 0`, the file count, and `token_scanned_a: 2` (4 when this
shell also exports both tokens). Each token goes to `grep` as a pattern file through a descriptor,
trimmed as the library trims it, so it never reaches a command line or the screen. Any non-zero
count is a leak: stop, tell the user to rotate that token, and report it. A `token_scanned_a`
below 2 means a token was not found to scan for, so a zero proves nothing: stop and find out why
the login shell gave none.

- [ ] **Step 13: Run B, Downtown: start the rig with the datum override**

(a) Read the datum from `docs/runbooks/mapbox.md` §7.2 (API notes) into the record:

```bash
cd "$INTEGRATION"
REHEARSAL=/tmp/overlume_terrain_rehearsal
datum() { sed -n "s/^| \`$1\` | \`\([-0-9.]*\)\` |\$/\1/p" docs/runbooks/mapbox.md; }
printf 'dt_lat: %s\ndt_lon: %s\ndt_hdg: %s\n' "$(datum geo_datum_lat_deg)" \
    "$(datum geo_datum_lon_deg)" "$(datum geo_datum_heading_deg)" | tee -a "$REHEARSAL/results.txt"
```

Expected: `dt_lat: 25.1952210`, `dt_lon: 55.2709401`, `dt_hdg: 90.1350` (sub-project 1's values;
the extraction was checked against today's runbook). An empty value means §7.2's table changed
shape: stop and read it.

(b) Start the rig in background mode, with the network on and no proxy:

```bash
cd "$INTEGRATION"
REHEARSAL=/tmp/overlume_terrain_rehearsal
SESSION_DIR="${SESSION_DIR:-${OVERLUME_SESSION:-$HOME/session_2026-09-01_13-57-00}}"
DT_LAT="$(sed -n 's/^dt_lat: //p' "$REHEARSAL/results.txt" | tail -1)"
DT_LON="$(sed -n 's/^dt_lon: //p' "$REHEARSAL/results.txt" | tail -1)"
DT_HDG="$(sed -n 's/^dt_hdg: //p' "$REHEARSAL/results.txt" | tail -1)"
tools/validate_logger_session.sh "$SESSION_DIR" --anchor-height-offset -1.8 -- \
    --param "geo_datum_lat_deg:=$DT_LAT" --param "geo_datum_lon_deg:=$DT_LON" \
    --param "geo_datum_heading_deg:=$DT_HDG" \
    --param 'environment_source_uri:=mapbox://buildings,terrain?materials=original&imagery=satellite' \
    > "$REHEARSAL/runB.out" 2>&1
```

Poll this with a Monitor until-loop until it prints:

```bash
grep -m1 '^Rig is up' /tmp/overlume_terrain_rehearsal/runB.out \
    || { ! pgrep -f '^bash tools/[v]alidate_logger_session\.sh' > /dev/null && echo "wrapper exited"; }
```

`wrapper exited` means the rig never came up: `tail -20 /tmp/overlume_terrain_rehearsal/runB.out`
names the cause. After `Rig is up`:

```bash
grep -E 'geo_datum override|environment source armed' /tmp/overlume_validate/overlume_node.log
```

Expected:
`geo-anchor solved (geo_datum override): --anchor-lat 25.19522100 --anchor-lon 55.27094010 --anchor-heading-deg 90.1350 --anchor-height -1.80`,
then `environment source armed: 'mapbox://buildings,terrain?materials=original&imagery=satellite' (visible)`.

- [ ] **Step 14: HUMAN. Visual checks at Downtown (mode 3, `mapbox-original-satellite`)**

Ask the user to confirm:
1. The Burj Khalifa and the tile's other landmarks render as textured models on satellite ground,
   with no extruded block doubled inside or around them.
2. Over one full loop of the replay (about 7 minutes for the default session), with the free-look
   camera high above the ego, the buildings stand on the surface and the route runs on it.
3. The attribution shows, with "© Maxar" on the line.

Record `dt_landmarks`, `dt_surface` and `dt_attribution`.

- [ ] **Step 15: HUMAN. The GUI's look switch at Downtown**

Ask the user to pick `mapbox-satellite` (the theme's clay on landmarks and extrusions, the same
satellite ground, the attribution on), then `mapbox-original-satellite` again. Record
`dt_gui_looks`.

- [ ] **Step 16: Render cost: `mapbox-satellite` against `osm` at the same poses**

Ask the user to leave the free-look camera alone for the next 15 minutes. Each source is sampled
over one full replay loop (420 s for the default session's 408 s loop; for another session its
loop length rounded up), after 30 s of settling that is sampled and discarded. Each block runs in
one Bash call (foreground, Bash timeout 600000 ms); a loop over about 480 s does not fit: put its
length in place of `420` and run the block in the Bash tool's background mode. Never split a loop.

```bash
cd "$INTEGRATION"
REHEARSAL=/tmp/overlume_terrain_rehearsal
source /opt/ros/humble/setup.bash
python3 "$REHEARSAL/set_env_source.py" mapbox-satellite || exit 1
python3 overlume/tools/sample_diagnostics.py 30 > /dev/null
printf 'mapbox_line: %s\n' "$(python3 overlume/tools/sample_diagnostics.py 420)" \
    | tee -a "$REHEARSAL/results.txt"
```

Expected: `[switch] mapbox-satellite: success=True`, then
`mapbox_line: render_ms_p50=… render_ms_p99=… render_ms_n=…`, n up to about 12600.

```bash
cd "$INTEGRATION"
REHEARSAL=/tmp/overlume_terrain_rehearsal
source /opt/ros/humble/setup.bash
python3 "$REHEARSAL/set_env_source.py" osm || exit 1
python3 overlume/tools/sample_diagnostics.py 30 > /dev/null
printf 'osm_line: %s\n' "$(python3 overlume/tools/sample_diagnostics.py 420)" \
    | tee -a "$REHEARSAL/results.txt"
```

Expected: `[switch] osm: success=True`, then an `osm_line:` of the same form.

```bash
python3 - /tmp/overlume_terrain_rehearsal/results.txt <<'EOF'
import re
import sys

path = sys.argv[1]
v = dict(line.split(": ", 1) for line in open(path).read().splitlines() if ": " in line)


def field(src, name):
    m = re.search(name + r"=([0-9.]+)", v[f"{src}_line"])
    assert m, f"{src}: no render_ms samples -- is the sampler on the rig's ROS domain?"
    return float(m.group(1))


mb50, mb99 = field("mapbox", "render_ms_p50"), field("mapbox", "render_ms_p99")
osm50, osm99 = field("osm", "render_ms_p50"), field("osm", "render_ms_p99")
ratio = mb50 / osm50
verdict = "PASS" if ratio <= 1.5 else "FAIL"
with open(path, "a") as f:
    f.write(f"mb_p50: {mb50:.1f}\nmb_p99: {mb99:.1f}\nosm_p50: {osm50:.1f}\nosm_p99: {osm99:.1f}\n")
    f.write(f"ratio: {ratio:.2f}\nratio_verdict: {verdict}\n")
print(f"mapbox-satellite p50 {mb50:.1f} ms / osm p50 {osm50:.1f} ms = {ratio:.2f}: {verdict} (bar 1.5)")
EOF
```

Expected: `… = 1.xx: PASS (bar 1.5)`, or lower. A `FAIL` is a finding for the user (acceptance 5):
report it, and do not close the plan.

The source is now `osm`. Ask the user to confirm that the Mapbox attribution is gone, and record
`osm_attribution`.

- [ ] **Step 17: Tear run B down; scan its logs and the ROS logs for both tokens**

```bash
pkill -TERM -f '^bash tools/[v]alidate_logger_session\.sh'
for p in $(pgrep -f '^bash tools/[v]alidate_logger_session\.sh'); do
    timeout 60 tail --pid="$p" -f /dev/null
done
RIG='[l]ib/overlume_ros/overlume_node|tools/[v]cam_ws_bridge\.py|tools/[v]cam_gui\.py'
pgrep -af "$RIG|[m]p_play|^bash tools/[v]alidate_logger_session" || echo "rig down"
```

Expected: `rig down`.

```bash
REHEARSAL=/tmp/overlume_terrain_rehearsal
mapfile -t ROSLOGS < <(find "${ROS_LOG_DIR:-${ROS_HOME:-$HOME/.ros}/log}" -type f \
    -newer "$REHEARSAL/start" 2>/dev/null)
LOGS=(/tmp/overlume_validate/*.log "$REHEARSAL/runB.out" "${ROSLOGS[@]}")
HITS=0
SCANNED=0
for name in MAPBOX_TOKEN CESIUM_ION_TOKEN; do
    login="$(env -u "$name" bash -lic "printf %s \"\${${name}:-}\" >&3" 3>&1 >/dev/null 2>&1)"
    for tok in "$login" "${!name:-}"; do
        [[ -n "$tok" ]] || continue
        SCANNED=$((SCANNED + 1))
        while IFS= read -r line; do
            HITS=$((HITS + ${line##*:}))
        done < <(grep -c -F -f <(printf '%s' "$tok" | tr -d '[:space:]') /dev/null "${LOGS[@]}")
    done
done
printf 'token_hits_b: %s\ntoken_files_b: %s\ntoken_scanned_b: %s\n' "$HITS" "${#LOGS[@]}" \
    "$SCANNED" | tee -a "$REHEARSAL/results.txt"
```

Expected: `token_hits_b: 0` over run B's rig logs and every ROS log file written since Step 3, from
both runs, then the file count and `token_scanned_b: 2` (or 4). A non-zero hit count is a leak:
stop, tell the user to rotate that token, and report it. Below 2 proves nothing: stop, as in
Step 12.

- [ ] **Step 18: Write the rehearsal into the docs**

The script appends two `## Shipped` rows to `docs/status.md` (Task 13's goldens, dated by their
commit, then the feature), closes open item 23, closes ledger row 15 and adds the Live acceptance
results to the plan, and marks the plan's `docs/README.md` entry closed. It checks everything
before it writes anything, and refuses when a token count is non-zero, a scan had fewer than both
tokens to scan for, the tile cache changed during the Mapbox-only phase, a visual check was not
confirmed, the render-cost verdict is `FAIL`, or an anchor it edits is missing or repeated (open
item 23's opening from Task 8; the plan's `docs/README.md` entry, ending in `).`, from Task 8; the
goldens' commit from Task 13).

```bash
cd "$INTEGRATION"
python3 - /tmp/overlume_terrain_rehearsal/results.txt <<'PYEOF'
import subprocess
import sys

v = {}
for line in open(sys.argv[1]).read().splitlines():
    if ": " in line:
        key, value = line.split(": ", 1)
        v[key] = value.strip()
for key in ("token_scanned_a", "token_scanned_b"):
    assert int(v[key]) >= 2, (f"{key} = {v[key]}: both tokens must be scanned for; fewer means a "
                              "zero proves nothing")
for key in ("token_hits_a", "token_hits_b"):
    assert v[key] == "0", f"{key} = {v[key]}: a token reached a log -- stop, rotate it, report"
assert v["cache_files_a"] == "0", "the tile cache changed during the Mapbox-only phase -- report"
for key in ("opa_imagery", "opa_surface", "opa_follow", "opa_attribution", "opa_gui_looks",
            "osm_terrain_imagery", "osm_terrain_credit", "cut_visual", "cut_attribution",
            "dt_landmarks", "dt_surface", "dt_attribution", "dt_gui_looks", "osm_attribution"):
    assert v[key].startswith("confirmed"), f"{key}: {v[key]} -- report it, do not close"
assert v["ratio_verdict"] == "PASS", "mapbox-satellite render_ms over 1.5x osm -- report, do not close"
LINK = "- [`2026-10-15-imagery-terrain.md`](plans/2026-10-15-imagery-terrain.md)"
readme = open("docs/README.md").read()
assert readme.count(LINK) == 1, "docs/README.md: the plan's entry must exist exactly once"
start = readme.index(LINK)
entry_end = readme.index("\n- ", start + len(LINK))
assert readme[entry_end - 2:entry_end] == ").", ("docs/README.md: the plan's entry must end "
                                                  "with a closing parenthesis and a full stop")
v["token_files"] = str(int(v["token_files_a"]) + int(v["token_files_b"]))
v["head"] = subprocess.run(["git", "rev-parse", "--short", "HEAD"], capture_output=True,
                           text=True, check=True).stdout.strip()
v["golden_date"] = subprocess.run(
    ["git", "log", "-1", "--format=%cs", "--",
     "overlume/tests/goldens/mapbox_terrain_synthetic_light_clay.png"],
    capture_output=True, text=True, check=True).stdout.strip()
assert v["golden_date"], "no commit adds the terrain goldens (Task 13)"

GOLDENS = (
    "| Imagery-on-terrain goldens `mapbox_terrain_synthetic_dark_adas` and "
    "`mapbox_terrain_synthetic_light_clay` (one synthetic z14 tile served in Mapbox's formats: "
    "a terrain-DEM hill under quadrant satellite imagery, red west and green east, with three "
    "clay blocks re-based onto the DEM on its summit, its east slope and level ground; one test "
    "body in two themes, with colour probes in light_clay) promoted by the user after viewing "
    "the candidates | {golden_date} | "
    "[`plans/2026-10-15-imagery-terrain.md`](plans/2026-10-15-imagery-terrain.md) |\n")

SHIPPED = (
    "| Imagery on terrain: each streaming source can carry a second cesium tileset of terrain "
    "with imagery as raster overlays on it. `mapbox://buildings,terrain?imagery=satellite` meshes "
    "Mapbox terrain-DEM tiles through a loader of ours, drapes Mapbox Satellite on them and "
    "re-bases the buildings onto the DEM (one DEM request per tile for both loaders, no disk "
    "cache); `ion://<asset>?terrain=1&imagery=2` streams Cesium World Terrain with Bing imagery "
    "through ion. GUI sources `mapbox-satellite`, `mapbox-original-satellite` and `osm-terrain`; "
    "node parameters `environment_terrain` and `environment_imagery`; theme token "
    "`environment.terrain_color`; `environment_source_credits` in `scene.h`. Rehearsed {date} on "
    "the replay rig at the operating area and at Downtown Dubai; `mapbox-satellite` `render_ms` "
    "p50 {mb_p50} ms against `osm` {osm_p50} ms; details in the plan's Live acceptance results | "
    "CLOSED {date} at `{head}` (rehearsal and docs in the following commit) | "
    "[`plans/2026-10-15-imagery-terrain.md`](plans/2026-10-15-imagery-terrain.md) |\n")
status = open("docs/status.md").read()
anchor = "\n\n## Open items"
assert status.count(anchor) == 1, "docs/status.md: expected one '## Open items' heading"
OPEN_23 = "23. **OPEN 2026-10-15 — imagery on terrain (sub-project 2) in progress.**"
assert status.count(OPEN_23) == 1, "docs/status.md: open item 23's opening must exist exactly once"
CLOSED_23 = ("23. **CLOSED {date} — imagery on terrain (sub-project 2), rehearsed at both sites; "
             "see the Shipped row.**")
plan = "docs/plans/2026-10-15-imagery-terrain.md"
text = open(plan).read()
LEDGER_15 = "| 15. Rig rehearsal and close-out | 7 |"
lines = text.split("\n")
rows = [i for i, line in enumerate(lines) if line.startswith(LEDGER_15)]
assert len(rows) == 1, f"{plan}: {len(rows)} ledger rows for Task 15"
plan_anchor = "\n## Global Constraints"
assert text.count(plan_anchor) == 1, f"{plan}: expected one '## Global Constraints' heading"

# Every check passed: write.
cut = status.index(anchor) + 1
status = status[:cut] + GOLDENS.format(**v) + SHIPPED.format(**v) + status[cut:]
open("docs/status.md", "w").write(status.replace(OPEN_23, CLOSED_23.format(**v)))
lines[rows[0]] = (LEDGER_15 + " done (see Live acceptance results) | the close-out commit after "
                  "`{head}` |").format(**v)
RESULTS = """### Live acceptance results ({date}, replay rig, `{session}`)

- Preflight: {token_checks}. Live smoke (Task 14): ledger row 14.
- Operating area, the session's own anchor ({opa_anchor}), armed with `--arm mapbox-satellite`. Satellite imagery under the clay extrusions, no clay plane: {opa_imagery}. Buildings on the surface along the whole route: {opa_surface}. The ego on the terrain surface (follow terrain): {opa_follow}. Attribution with Maxar while streaming: {opa_attribution}. GUI `mapbox-original-satellite`, then `mapbox-satellite`: {opa_gui_looks}. No tile-cache file written while only Mapbox sources ran: {cache_files_a} files.
- Provider proof, GUI `osm-terrain` at the operating area: Cesium World Terrain with Bing imagery under OSM buildings: {osm_terrain_imagery}. Bing's credits on screen, bottom-left: {osm_terrain_credit}.
- Network cut ({cut_method}), then a live switch to `mapbox-original-satellite` so that new requests fail: the `network loss detected` WARN came {cut_seconds} s after the switch. Terrain and imagery gone, the clay plane and the baked fallback back: {cut_visual}. Attribution after the fallback: {cut_attribution}.
- Downtown Dubai: datum `geo_datum_lat_deg:={dt_lat}`, `geo_datum_lon_deg:={dt_lon}`, `geo_datum_heading_deg:={dt_hdg}` (runbook `mapbox.md` §7.2), launched with `environment_source_uri:=mapbox://buildings,terrain?materials=original&imagery=satellite`. Landmark models on satellite ground with no doubled extrusion: {dt_landmarks}. Buildings on the surface over one full loop: {dt_surface}. Attribution with Maxar: {dt_attribution}. GUI `mapbox-satellite` against `mapbox-original-satellite`: {dt_gui_looks}.
- Render cost at Downtown, `render_ms` from `/overlume_node/diagnostics` over one full replay loop per source, after 30 s of settling, same free-look camera: `mapbox-satellite` p50 {mb_p50} ms, p99 {mb_p99} ms; `osm` p50 {osm_p50} ms, p99 {osm_p99} ms; p50 ratio {ratio} against the 1.5 bar: {ratio_verdict}. Mapbox attribution with `osm` selected: {osm_attribution}.
- Secrets: neither the `MAPBOX_TOKEN` nor the `CESIUM_ION_TOKEN` value appears in any of the {token_files} node, rig and ROS log files written during the rehearsal.
"""
text = "\n".join(lines)
open(plan, "w").write(text.replace(plan_anchor, "\n" + RESULTS.format(**v) + plan_anchor))
open("docs/README.md", "w").write(readme[:entry_end - 2] + "; closed {date}".format(**v) +
                                  readme[entry_end - 2:])
print("docs written for", v["date"])
PYEOF
git diff --stat
```

Expected: `docs written for <today>`, then a diff stat touching exactly `docs/README.md`,
`docs/plans/2026-10-15-imagery-terrain.md` and `docs/status.md` (`docs/status.md | 4 +++-`: two
rows added, item 23's line changed); the README entry's last line ends
``-design.md`; closed <today>).``; `grep -c '^23\. \*\*CLOSED' docs/status.md` prints `1`. The
script was dry-run against copies of today's `docs/status.md` with Task 8's item 23, of
`docs/README.md` with a plan entry of that shape, and of the plan head with its ledger, in a
scratch repository with a goldens commit: it wrote the three files as described, and with a
`token_scanned_b` of 1, a `not confirmed` check, item 23 missing, or no goldens commit, it refused
before writing anything. A failed assertion names what is
missing: fix the record, not the assertion.

- [ ] **Step 19: Runbook facts, docs links and the final gate**

Task 8 wrote the runbooks before the code they describe was merged. Check the facts they cite
against the merged code, and that nothing waits for this task:

```bash
cd "$INTEGRATION"
grep -rqF 'mapbox:// terrain tile 14/{}/{} skipped: {}' overlume/src \
    || echo "fact drifted: terrain tile skip line"
grep -qF 'mapbox:// imagery tile failed' docs/runbooks/mapbox.md docs/runbooks/terrain_imagery.md \
    && echo "fact drifted: the runbooks promise an imagery failure line no task emits"
grep -qF 'terrain tileset did not open (HTTP {}) -- continuing without ' overlume/src/environment_stream.cpp || echo "fact drifted: terrain open line"
grep -qF '"DEM tile"' tools/check_mapbox_token.sh && grep -qF '"satellite tile"' tools/check_mapbox_token.sh \
    || echo "fact drifted: token check labels"
for p in environment_terrain environment_imagery; do
    grep -qF "\"$p\"" ros/src/overlume_ros/src/overlume_node.cpp || echo "fact drifted: $p"
done
for e in mapbox-satellite mapbox-original-satellite osm-terrain; do
    grep -qF "\"$e\"" tools/vcam_ws_bridge.py && grep -qF "\"$e\"" tools/vcam_gui.py \
        || echo "fact drifted: GUI entry $e"
    grep -qF "$e" docs/runbooks/mapbox.md docs/runbooks/terrain_imagery.md \
        || echo "runbooks miss the GUI entry $e"
done
grep -qF 'TEST(MapboxLive, OperatingAreaTerrain)' overlume/tests/test_mapbox_terrain.cpp \
    && grep -qF 'TEST(IonLive, WorldTerrainBing)' overlume/tests/test_mapbox_terrain.cpp \
    || echo "fact drifted: live tests"
grep -rn 'filled in by Task 15' docs/ || true
```

Expected: no output. Each printed line names a runbook sentence or placeholder to reconcile in
Step 20's commit: change the runbook to what the merged code does (or fill the placeholder with
the rehearsal's value), and say so in the commit message. No task emits a per-tile imagery
failure line of ours (head.md, Texts; ruling R2), so the runbooks must not promise one: a failed
raster tile shows in the log as cesium's `Received response code {} for image …` line, through
the redacting logger with the token masked and the host visible. The terrain
open line is grepped only up to `continuing without ` because the source splits that literal
across two lines (clang-format's 100-column limit puts `terrain` on the next line).

Then the links and the gate, in the foreground, Bash timeout 600000 ms, under the gate lock:

```bash
cd "$INTEGRATION"
python3 tools/check_docs_links.py
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock tools/ci_visual_mode.sh
```

Expected: `OK: no broken links`, then six `PASS` lines and exit 0, with the counts of Task 14's
gate. Exit 75 means another gate held the lock for 300 s: run the same command again. Any other
non-zero exit is a gate failure.

- [ ] **Step 20: Commit**

```bash
cd "$INTEGRATION"
git add docs/README.md docs/plans/2026-10-15-imagery-terrain.md docs/status.md
git commit -F - <<'EOF'
docs(terrain): record the replay-rig rehearsal and close the plan

The rehearsal on the logger-session replay rig covered the operating
area (--arm mapbox-satellite, then mapbox-original-satellite and
osm-terrain in the GUI, then a network cut that reached the baked
fallback with terrain and imagery gone) and Downtown Dubai (sub-project
1's datum override, armed with
mapbox://buildings,terrain?materials=original&imagery=satellite, and the
mapbox-satellite against osm render cost). No token reached a log, and
no tile-cache file was written while only Mapbox sources ran.
docs/status.md gets the Shipped rows for the synthetic-hill goldens
(dated by their commit) and for the feature, and open item 23 is closed;
the plan gets its Live acceptance results, and its entry in
docs/README.md is marked closed.

Claude-Session: <the executing session's URL>
EOF
```

If Step 19 reconciled a runbook, `git add` it too and add a paragraph naming each sentence changed.

- [ ] **Step 21: Pre-push checks**

```bash
cd "$INTEGRATION"
git fetch origin
tools/check_format.sh
tools/check_spdx.sh
shellcheck --severity=error tools/*.sh && echo "shellcheck: PASS"
python3 tools/check_docs_links.py
tools/check_publish_leaks.sh
git diff --name-status --diff-filter=MD origin/main...HEAD -- overlume/tests/goldens \
    ros/src/overlume_ros/test/fixtures
git diff --name-status --diff-filter=A origin/main...HEAD -- overlume/tests/goldens
git rev-parse --abbrev-ref --symbolic-full-name '@{u}'
git rev-list --count HEAD..origin/mapbox-environment
git rev-list --count origin/mapbox-environment..HEAD
```

Expected:
- `check_format.sh: PASS (…)`, `check_spdx.sh: PASS (…)`, `shellcheck: PASS`,
  `OK: no broken links`.
- `PASS  publish leak check over origin/main..HEAD (… commits)`.
- Nothing from the first `git diff`: no existing golden moved or vanished (acceptance 1).
- Four lines from the second, the goldens both sub-projects add:
  `A	overlume/tests/goldens/mapbox_buildings_synthetic_dark_adas.png`,
  `A	overlume/tests/goldens/mapbox_buildings_synthetic_light_clay.png`,
  `A	overlume/tests/goldens/mapbox_terrain_synthetic_dark_adas.png`,
  `A	overlume/tests/goldens/mapbox_terrain_synthetic_light_clay.png`.
- `origin/mapbox-environment`, then `0` (nothing on the remote branch is missing here, so the push
  fast-forwards), then the number of commits this plan adds.

On a leak-check `FAIL`, stop and tell the user: the fix is a history rewrite of the unpushed
commits, which needs their go-ahead, before any push. Never push and fix after. On any other
failure, stop and fix it in a commit of its own (gate first), then rerun this step.

- [ ] **Step 22: Push the branch (the user's delivery)**

```bash
cd "$INTEGRATION"
git push origin mapbox-environment
git status -sb | head -1
git lfs ls-files | grep mapbox_terrain_synthetic
```

Expected: the push fast-forwards `mapbox-environment -> mapbox-environment` (Git LFS uploads the
two new goldens first), `## mapbox-environment...origin/mapbox-environment` with no `ahead`, and
both terrain goldens listed. A rejected push (the remote moved) means stop and tell the user;
never force.

- [ ] **Step 23: Pull request #1 gains a section for this feature**

```bash
cd "$INTEGRATION"
REHEARSAL=/tmp/overlume_terrain_rehearsal
gh pr view 1 --json number,state,headRefName --jq '"\(.number) \(.state) \(.headRefName)"'
gh pr view 1 --json body --jq .body > "$REHEARSAL/pr1_before.md"
python3 - "$REHEARSAL/results.txt" "$REHEARSAL/pr1_before.md" "$REHEARSAL/pr1_after.md" <<'PYEOF'
import subprocess
import sys

results, before, after = sys.argv[1], sys.argv[2], sys.argv[3]
v = {}
for line in open(results).read().splitlines():
    if ": " in line:
        key, value = line.split(": ", 1)
        v[key] = value.strip()
body = open(before).read()
HEADING = "## Imagery on terrain (sub-project 2)"
assert HEADING not in body, "PR #1 already has the section: edit it by hand, never twice"
v["range"] = subprocess.run(["git", "log", "-1", "--format=%h"], capture_output=True, text=True,
                            check=True).stdout.strip()
SECTION = """{heading}

Terrain under the streamed buildings with imagery draped on it, through provider-agnostic
interfaces. Design `docs/design/2026-10-15-imagery-terrain-design.md`, plan
`docs/plans/2026-10-15-imagery-terrain.md`, runbooks `docs/runbooks/mapbox.md` and
`docs/runbooks/terrain_imagery.md`.

- `mapbox://buildings,terrain?imagery=satellite`: Mapbox terrain-DEM tiles meshed by a loader of
  ours, Mapbox Satellite as a cesium raster overlay, buildings re-based onto the DEM. One DEM
  request per tile serves both loaders; no Mapbox byte reaches the disk.
- `ion://<asset>?terrain=1&imagery=2`: Cesium World Terrain with Bing imagery through ion, over
  the same interfaces.
- vcam GUI sources `mapbox-satellite`, `mapbox-original-satellite` and `osm-terrain`; node
  parameters `environment_terrain` and `environment_imagery`; theme token
  `environment.terrain_color`; `environment_source_credits()` in `scene.h` (a free function, so
  `kSceneVersion` stays); the attribution line carries the overlay credits ("© Maxar" for
  satellite).
- Checks, all offline in `tools/ci_visual_mode.sh`: synthetic DEM, imagery and quantized-mesh
  fixtures, end-to-end tests over the fixture network, and the goldens
  `mapbox_terrain_synthetic_dark_adas` and `mapbox_terrain_synthetic_light_clay`, promoted by the
  maintainer. Opt-in live smoke: `MapboxLive.OperatingAreaTerrain` and `IonLive.WorldTerrainBing`.
- Rehearsed {date} on the replay rig at the operating area (`mapbox-satellite`,
  `mapbox-original-satellite`, `osm-terrain`, and a network cut that reached the fallback) and at
  Downtown Dubai. `mapbox-satellite` p50 `render_ms` {mb_p50} ms against `osm` {osm_p50} ms
  (ratio {ratio}, bar 1.5). No token in any log.
- Delivered at `{range}`.
"""
text = body.rstrip("\n") + "\n\n" + SECTION.format(heading=HEADING, **v)
open(after, "w").write(text)
print(SECTION.format(heading=HEADING, **v), end="")
PYEOF
PATTERNS="$REHEARSAL/publish_patterns.txt"
printf '%s\n' '/home/[A-Za-z0-9._-]+/' '/Users/[A-Za-z0-9._-]+/' '/tmp/claude-[0-9]+' \
    '[.]claude/projects/' > "$PATTERNS"
DENY="$(git rev-parse --git-common-dir)/info/publish-denylist"
[[ -f "$DENY" ]] && grep -v -e '^#' -e '^$' "$DENY" >> "$PATTERNS"
grep -c -E -f "$PATTERNS" "$REHEARSAL/pr1_after.md"
```

Expected: `1 OPEN mapbox-environment`; the new section printed; then `0`: nothing local or
internal in the new body. A non-zero count: stop, show the user the matching lines of the section
(never of the denylist), and fix the section's text before anything is published. With `0`:

```bash
cd "$INTEGRATION"
gh pr edit 1 --body-file /tmp/overlume_terrain_rehearsal/pr1_after.md
gh pr view 1 --json body --jq .body | grep -c -F '## Imagery on terrain (sub-project 2)'
```

Expected: the edit prints the pull request's URL, then `1`. The script refuses to add the section
twice; if the description already holds it, the user edits it by hand. Tell the user the plan is
closed, the branch pushed, and pull request #1 now tracks this feature.

