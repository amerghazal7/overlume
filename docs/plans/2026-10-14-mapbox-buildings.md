# Mapbox Buildings Environment Source Implementation Plan

> **For agentic workers:** execute with dynamic workflows as `AGENTS.md` requires: the
> orchestrator on the session model, implementers on Sonnet, review gates on Opus, at most two
> fix rounds per task, leftover minors applied by the orchestrator. Lanes run in parallel as the
> Execution section describes. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `mapbox://buildings`, a streamed environment source selected like `ion://`: Mapbox's
landmark models plus extruded streets-v8 footprints around the vehicle, rendered by the existing
streaming pipeline, with Mapbox's attribution on screen, demo-ready at Downtown Dubai and at the
test site.

**Architecture:** A custom cesium-native `TilesetContentLoader` models Web Mercator XYZ as a
quadtree, and cesium's area view selects only the z14 tiles within the radius of the ego. Each
z14 tile merges its landmark GLB (meshopt decoded by cesium; the footprint nodes, any LOD scene
and, in clay mode, the textures dropped) with roofs and walls extruded from its 16 z16 vector
tiles, all placed in an east-north-up frame at the tile centre, so the existing height correction
lands building bases on map z = 0. `StreamingEnvironmentSource` gains a custom-tileset seam with
area selection; the node gains `mapbox://` URI composition and an attribution overlay; the vcam
GUI lists two more environment sources.

**Tech Stack:** C++20 library code on cesium-native 0.64.0 (CesiumGltf, CesiumGltfReader,
CesiumGeospatial, CesiumAsync) and `mapbox/earcut.hpp` 2.2.4 from cesium's vcpkg; C++17 tests on
googletest 1.15.2, with meshoptimizer 1.1's encoder and stb for fixtures; Filament 1.56.5 gltfio;
ROS 2 Humble (`overlume_ros`); Python 3 (vcam GUI and bridge); Bash.

**Spec:** `docs/design/2026-10-14-mapbox-buildings-design.md` (approved by the user 2026-10-14).
This plan argues from it; executors read both. Citations such as "fact-check F1" name the planning
session's verification notes, which are not in the repository; each fact they back is stated
where the plan uses it.

## Status ledger

| Task | Lane | State | Commit |
|------|------|-------|--------|
| 0. CMake wiring for the Mapbox units | 0 | pending | |
| 1. MVT building-layer decoder | 1 | pending | |
| 2. `mapbox://` URI parser | 1 | pending | |
| 3. Extrusion builder | 2 | pending | |
| 4. Tile math and the tile frame | 3 | done | `9619ee2..58aa6fa` |
| 5. Landmark reader | 3 | pending | |
| 6. Count transport errors as network failures | 4 | pending | |
| 7. Custom tilesets with area selection | 4 | pending | |
| 8. Node URI composition for `mapbox://` | 5 | pending | |
| 9. HUD primitives: Latin-1 text, `MeasureText`, `DrawImageRgba` | 5 | pending | |
| 10. Mapbox attribution on screen | 5 | pending | |
| 11. GUI sources, token check, rig token refresh | 6 | done | `9619ee2..8123319` |
| 12. Docs | 6 | done | `8123319..e40ad03` |
| 13. Mapbox loader: tiling, extrusions, failures, dispatch | 7 | pending | |
| 14. Landmarks and conflation in the loader | 7 | pending | |
| 15. Golden candidate and promotion | 8 | pending | |
| 16. Opt-in live smoke | 8 | pending | |
| 17. Rig rehearsal at both sites and close-out | 8 | pending | |

## Global Constraints

- **Tokens by name only.** `MAPBOX_TOKEN` and `CESIUM_ION_TOKEN` are never committed, echoed,
  logged, or pasted into any file or transcript. Scripts print PASS/FAIL and HTTP codes only. No
  test needs live network or a token; the opt-in live tests skip unless `OVERLUME_LIVE_MAPBOX=1`.
- The library reads `MAPBOX_TOKEN` with `getenv` only when a `mapbox://` source opens. The token
  never appears in the URI, a parameter, a log line, a model or a file. Every Mapbox
  `TileLoadResult` has `pCompletedRequest == nullptr`.
- Every Mapbox log line goes through the redacting logger (`make_redacting_logger()`, the
  `overlume.cesium` logger), at most one line per endpoint and status per source.
- **No Mapbox bytes in the tree** (Product Terms 1.10): fixtures are synthesized in the test
  process in Mapbox's format; probe output never enters the repository. The one Mapbox file the
  branch commits is the attribution logo (Task 10, spec §6), which the terms require on screen;
  whether it stays in the tree is part of the release check (spec D5).
- **No public-header change:** nothing under `overlume/include/overlume/` changes, `kSceneVersion`
  stays 9, and `overlume/scripts/check_pod_header.sh` passes.
- **Local only.** `mapbox-environment` and every lane branch stay local. Nothing is pushed to
  `origin` until the user confirms Mapbox's sign-off. Before that push: `tools/check_publish_leaks.sh`,
  `tools/check_format.sh`, `tools/check_spdx.sh`, `shellcheck --severity=error tools/*.sh` and
  `python3 tools/check_docs_links.py`.
- No local absolute path, agent scratch path or internal repository name in a tracked file or a
  commit message. Use `$INTEGRATION` (the `mapbox-environment` worktree), `$WORKTREE` (the current
  lane's worktree), `$PRIMARY_CHECKOUT`, `$PERCEPTION_REPO`, `$SESSION_DIR`.
- The primary checkout (`$PRIMARY_CHECKOUT`, another session's `release-packaging`) is never
  built in, switched or edited.
- Every Bash call starts in the primary checkout, so every command block starts with
  `cd "$WORKTREE"` (lanes 1–7) or `cd "$INTEGRATION"` (lanes 0 and 8), unless it uses only
  absolute paths.
- **Goldens are promoted by the user only.** No existing golden may move.
- Every behaviour change ships with a runnable check that fails when the change is reverted.
- Every task ends green on `tools/ci_visual_mode.sh`, run in the foreground (Bash timeout
  600000 ms), and is committed on its own. The message says what changed and why and ends with the
  executing session's `Claude-Session:` trailer line: the commit blocks write it as
  `Claude-Session: <the executing session's URL>`, which the executor replaces with the session
  URL its own context gives.
- New C++, shell, Python and CMake files start with `// SPDX-License-Identifier: Apache-2.0` and
  `// Copyright 2026 Amer Ghazal` (`#` comments for scripts, after the shebang). C++ follows the
  repo's `.clang-format` (clang-format 23.1.1): `git add`, then `tools/check_format.sh --fix`.
  Namespaces close with a bare `}`.
- All new library code is cesium-only: `overlume/src/mapbox_*.cpp` compile as C++20 in
  `overlume_stream`, and `overlume/tests/test_mapbox_*.cpp` are dropped when
  `OVERLUME_ENABLE_CESIUM=OFF` (Task 0). Tests are C++17 and see no cesium, glm or vcpkg header,
  except meshoptimizer's for `test_mapbox_*`; they reach cesium-typed code only through
  `src/mapbox_test_hooks.hpp` and `src/environment_test_hooks.hpp`.
- Classes and helpers that no test calls live in an anonymous namespace: the shared library
  exports every `overlume::*` symbol.
- **URI:** `mapbox://buildings[?key=value(&key=value)*]`. Keys: `materials` (`clay` default,
  `original`; any other value warns once and means `clay`), `radius` (finite > 0, default 500 m),
  `brightness` (finite > 0, default 1.0), `fallback` (a baked chunk directory), `cache` (only
  `off`; any other value refuses the URI). The ion-only keys (`follow_terrain`, `ground_bias`,
  `replaces_ground`, `max_tilt_deg`, `max_cache_items`) are ignored like unknown keys.
- The Mapbox source never follows terrain, never replaces the ground plane and never writes a
  cache file.
- **Endpoints** (the token is the only query parameter):
  `https://api.mapbox.com/3dtiles/v1/mapbox.mapbox-3dbuildings-v1-meshopt/14/{x}/{y}.glb?access_token=<token>`
  and `https://api.mapbox.com/v4/mapbox.mapbox-streets-v8/16/{x}/{y}.vector.pbf?access_token=<token>`.
  The landmark variant is `-meshopt`, vertex codec v0. GL JS's `-meshopt-v2-lod` variant uses
  vertex codec v1, which the meshoptimizer first on every link line (Filament 1.56.5's, without
  `meshopt_decodeVertexVersion`) cannot decode; scene 0 of the two variants is the same.
- **Token check:** `tools/check_mapbox_token.sh` mirrors `overlume/scripts/check_cesium_token.sh`
  and prints exactly one line on stdout. When `MAPBOX_TOKEN`, trimmed as the library trims it, is
  empty: `SKIP: MAPBOX_TOKEN is not set (see docs/runbooks/mapbox.md)`, exit 2, no request. When
  both Downtown tiles answer 200:
  `PASS: token retrieves the Downtown Dubai tiles (landmark tile HTTP 200, footprint tile HTTP 200)`,
  exit 0. Otherwise `FAIL: <landmark>, <footprint> (see docs/runbooks/mapbox.md)`, exit 1, each
  part `<label> HTTP <code>` or `<label> transport error (curl exit <n>)`. It never prints a token,
  a URL or a response body.
- **GUI:** no presets. The Environment Tiles list gains `mapbox` (→ `mapbox://buildings`) and
  `mapbox-original` (→ `mapbox://buildings?materials=original`), in lockstep between
  `tools/vcam_gui.py` and `tools/vcam_ws_bridge.py`. Existing identifiers (`ENVIRONMENT_PRESETS`,
  the wire field `preset`) keep their names.
- **Attribution:** the logo bottom-left and "© Mapbox © OpenStreetMap Improve this map" right-aligned
  bottom-right, 8 px from the edges, in the theme's HUD text colour and scale, whenever
  `environment_attribution` is on, the environment is visible, the URI starts with `mapbox://` and
  the state is `STREAMING`. Below 541 px of output width at HUD scale 1.0 the text's end is cut
  (Review Focus 5).
- The plan's commit adds the plan's `docs/README.md` entry under `## Plans (docs/plans/)`, ending
  ``(implements `design/2026-10-14-mapbox-buildings-design.md`).``. No task adds it again. Task 12
  adds only the runbook's index entry, and Task 17 inserts `; closed <date>` before the entry's
  final `).`, failing if the entry is missing.

## Review Focus

1. **A token with surrounding whitespace** (a shell export with a trailing newline or space): the
   library trims ASCII whitespace before use, the requests carry the trimmed token, and no log line
   contains it. Test in Task 13.
2. **The ego at a z14 corner, then crossing into the next tile:** the four corner tiles load,
   extrusions meet at the z14 seams without doubled walls, and tiles that leave the radius leave
   the scene. Test in Task 13.
3. **Garbage behind a 200 status** (an HTML page where an MVT should be, a truncated gzip body, a
   GLB with a bad header): that part of the tile is skipped with one log line, the rest of the
   tile shows, nothing crashes, and the tile is not failed for good. Tests in Tasks 13 and 14.
4. **An undecodable landmark GLB** (a corrupt meshopt stream, as a future codec change would
   produce): extrusions still show, one log line names the tile, and no half-decoded landmark mesh
   reaches the renderer. Tests in Tasks 5 and 14.
5. **Narrow output frames:** when the attribution text would overlap the logo, the text moves up
   one line, above the logo, and neither leaves the frame while the text fits on a line of its own
   (an output width of at least 16 px plus its advance: 541 px at HUD scale 1.0; at 1280 px, up to
   scale 2.4) and the frame is at least 86 px tall at HUD scale 1.0. Narrower, the text keeps its
   start and loses its end, "Improve this map"; shorter, the lifted line leaves through the top.
   Both are known limits, recorded in Task 10 (`CompositeAttribution`'s `ponytail:` comment and
   API notes) and runbook §5; the demo configuration (1280 × 720, scale 1.0) is clear of them.
   Test in Task 10.

## Execution

### Lanes

| Lane | Tasks | Starts after | Branch | ROS domain | Test tmp dir |
|------|-------|--------------|--------|-----------|--------------|
| 0 | 0 | this plan's commit | `mapbox-environment` | 77 | `/tmp` |
| 1 | 1, 2 | Task 0 | `mapbox-env/l1` | 81 | `/tmp/overlume-l1` |
| 2 | 3 | Task 0 | `mapbox-env/l2` | 82 | `/tmp/overlume-l2` |
| 3 | 4, 5 | Task 0 | `mapbox-env/l3` | 83 | `/tmp/overlume-l3` |
| 4 | 6, 7 | Task 0 | `mapbox-env/l4` | 84 | `/tmp/overlume-l4` |
| 5 | 8, 9, 10 | Task 0 | `mapbox-env/l5` | 85 | `/tmp/overlume-l5` |
| 6 | 11, 12 | Task 0 | `mapbox-env/l6` | 86 | `/tmp/overlume-l6` |
| 7 | 13, then 14 | 13: Tasks 1–4, 6, 7 merged. 14: Task 5 merged | `mapbox-env/l7` | 87 | `/tmp/overlume-l7` |
| 8 | 15, 16, 17 | 15: every lane 1–7 merged. 16: Task 15 committed. 17: Tasks 15 and 16 committed and the whole-branch review closed | `mapbox-environment` | 77 | `/tmp` |

Lanes 1–6 start together. Each lane touches only the files the File map gives it, so merges are
conflict-free; a conflict is a plan deviation to report, not to resolve silently.

### Worktrees and builds

`$INTEGRATION` is the `mapbox-environment` worktree; lane worktrees are its siblings
`$INTEGRATION-l1` … `$INTEGRATION-l7`. Task 0 builds `$INTEGRATION` first, which warms ccache.
Then, for each lane N of 1–6, one lane at a time (the shared ezvcpkg prefix is not safe for
concurrent configures):

```bash
git -C "$INTEGRATION" worktree add "$INTEGRATION-lN" -b mapbox-env/lN mapbox-environment
cd "$INTEGRATION-lN"
mkdir -p /tmp/overlume-lN
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake \
    --toolchain "$PWD/overlume/cmake/toolchain-clang-libcxx.cmake" -S overlume -B overlume/build \
    -DOVERLUME_ENABLE_CESIUM=ON -DCMAKE_C_COMPILER_LAUNCHER=ccache \
    -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DOVERLUME_TEST_TMP_DIR_RUNTIME=/tmp/overlume-lN
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j16
ros/colcon_build.sh
```

Lane 7's worktree is created the same way when Task 13 may start, from the merged
`mapbox-environment` head; other lanes may be gating then, so its configure runs under the gate
lock (`flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock cmake --toolchain …`).

**Later configures.** Every configure runs cesium's ezvcpkg step on the shared prefix, and every
gate configures inside the gate lock (`tools/ci_visual_mode.sh:56`), so a lane's own configures
take the same lock. A task that adds a `.cpp` file reconfigures before its first build and chains
the build onto the reconfigure: the `CONFIGURE_DEPENDS` globs (`overlume/CMakeLists.txt:208`,
`:581`) would otherwise reconfigure inside `cmake --build`, outside the lock.

```bash
cd "$WORKTREE"
flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock cmake -S overlume -B overlume/build > /dev/null \
    && echo reconfigured \
    && CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
```

No `reconfigured` line and no CMake error above it means the lock wait ran out (exit 75) and nothing
was built: run the step again.

**The gate in a lane** (every task's last step before its commit): prebuild at `-j8`, so the
gate's own `-j$(nproc)` build has nothing left to relink and six lanes stay within memory, then
the gate with the lane's ROS domain, holding the machine-wide gate lock. Gates never run
concurrently: stage 4's end-to-end tests bind the fixed WebSocket ports 18765 and 18766
(`tools/test_vcam_ws_bridge.py:255`, `:394`) and one forces `ROS_DOMAIN_ID=91` (`:602`), so two
gates at once fail each other. The lock waits at most 300 s, which keeps the whole command inside
the 600 s foreground limit (the gate itself takes about 143 s):

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
ROS_DOMAIN_ID=8N CI_VISUAL_MODE_DOMAIN_ID=8N \
    flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock tools/ci_visual_mode.sh
```

Exit 75 means another lane held the lock for 300 s: run the same command again. Any other
non-zero exit is a gate failure. Every `pytest` command in this plan passes `-p no:anyio`, as the
gate does (`tools/ci_visual_mode.sh:138-139`): this host's user-site anyio plugin aborts pytest
6.2.5 without it.

Expected: six `PASS` lines (POD header check, library ctest suite, node gtests, WS bridge pytest
suite, golden suite, examples) and exit 0. Lanes 0 and 8 use domain 77 and no prefix.

**Baseline** (`mapbox-environment` at the spec commit, measured 2026-10-14 on the 32-core
development machine): configure 40 s with the vcpkg prefix already installed, full library build
94 s at `-j16`, `ros/colcon_build.sh` 43 s, gate 143 s. Gate counts: 347 library ctest entries
(331 ok, 6 skipped), 363 node tests, 82 WS bridge tests, 23 goldens, 6 examples. A task's gate
must show every one of these counts unchanged or higher, and the goldens all `ok`.

### Per-task loop

1. An implementer (Sonnet) follows the task's steps in the lane worktree, test first.
2. The gate is green; the implementer commits.
3. A review gate (Opus) checks the commit against the spec, the Global Constraints, the Interface
   contract and this plan, and checks that each new test fails when its behaviour is reverted.
4. At most two fix rounds, each ending green on the gate; leftover minors go to the orchestrator.
5. The orchestrator marks the ledger row.

### Merges

The orchestrator merges each finished lane into `mapbox-environment` with
`git -C "$INTEGRATION" merge --no-ff mapbox-env/lN`, then runs the gate in `$INTEGRATION`
(domain 77, under the same gate lock) before the next merge. The gate lock only orders this
plan's lanes: a gate another session runs elsewhere on the machine can still clash on stage 4's
ports, which shows as a WS bridge test failure; rerun the gate before treating it as a finding.
Lane 3 merges twice: at Task 4's commit (so Task 13 can start) and after Task 5. After the last
merge, a whole-branch review (Opus) runs before Task 17.

Lane 7 merges `mapbox-environment` into `mapbox-env/l7` after lane 3's Task 5 merge, before Task
14 (its Step 1); the merge is conflict-free because Task 5's block of `mapbox_test_hooks.hpp` sits
above Task 4's and Task 13's below it. While a lane 8 task has changes in `$INTEGRATION`, staged
or not, the orchestrator merges and commits nothing there, ledger rows included; it marks lane 8's
rows after each lane 8 commit.

## File map

| File | Tasks | Change |
|------|-------|--------|
| `overlume/CMakeLists.txt` | 0 | `src/mapbox_*.cpp` into `overlume_stream`; `tests/test_mapbox_*.cpp` cesium-only, with meshoptimizer's include directory |
| `overlume/src/environment_stream.hpp` | 0, 6, 7, 13 | 0: the C++20 guard's message. 6: counting fix, fixture transport flag. 7: `CustomTileset`, area selection, `build_externals` and `make_redacting_logger` declared. 13: `MapFixtureAssetAccessor` |
| `overlume/src/environment_stream.cpp` | 6, 7, 13 | as above, plus 7: the render loop admits only `Done` tiles; 13: the test-only GLB capture |
| `overlume/src/environment_test_hooks.hpp` | 6, 7 | transport-failure switch; grid fixture hooks |
| `overlume/tests/test_environment_stream.cpp` | 6, 7 | ion transport regression; seam and area-selection tests |
| `overlume/src/mapbox_mvt.hpp`, `.cpp` | 1 | create |
| `overlume/tests/mapbox_mvt_fixture.hpp` | 1 | create: MVT encoder, stored-block gzip |
| `overlume/tests/test_mapbox_mvt.cpp` | 1 | create |
| `overlume/src/mapbox_loader.hpp`, `.cpp` | 2, 13, 14 | 2: create with the parser. 13, 14: the loader |
| `overlume/tests/test_mapbox_uri.cpp` | 2 | create |
| `overlume/src/mapbox_extrude.hpp`, `.cpp` | 3 | create |
| `overlume/tests/test_mapbox_extrude.cpp` | 3 | create |
| `overlume/src/mapbox_tile_frame.hpp`, `.cpp` | 4 | create |
| `overlume/src/mapbox_test_hooks.hpp` | 4, 5, 13 | 4: create. 5: its block directly after `namespace overlume::testing {`. 13: its block at the end |
| `overlume/tests/test_mapbox_tile_frame.cpp` | 4 | create |
| `overlume/src/mapbox_landmarks.hpp`, `.cpp` | 5 | create |
| `overlume/tests/mapbox_glb_fixture.hpp` | 5 | create: Mapbox-layout GLB builder |
| `overlume/tests/test_mapbox_landmarks.cpp` | 5 | create |
| `overlume/src/environment.hpp`, `.cpp` | 13 | `open_mapbox_environment_source`; the `mapbox://` dispatch |
| `overlume/tests/test_mapbox_buildings.cpp` | 13, 14, 15, 16 | 13: create (end to end). 14: the landmark and conflation tests. 15: golden. 16: live |
| `overlume/tests/goldens/mapbox_buildings_synthetic_dark_adas.png` | 15 | created only after the user promotes it |
| `ros/src/overlume_ros/include/overlume_ros/environment_source_uri.hpp`, `ros/src/overlume_ros/test/test_environment_source_uri.cpp` | 8, 10 | 8: Mapbox composition. 10: the attribution gate's scheme predicate |
| `ros/src/overlume_ros/include/overlume_ros/hud_overlay.hpp`, `ros/src/overlume_ros/src/hud_overlay.cpp`, `ros/src/overlume_ros/test/test_hud_overlay.cpp` | 9, 10 | 9: Latin-1 atlas, `MeasureText`, `DrawImageRgba`. 10: PNG loading and the attribution composite (the one node source the tests already compile) |
| `ros/src/overlume_ros/src/overlume_node.cpp`, `ros/src/overlume_ros/include/overlume_ros/overlume_node.hpp`, `ros/src/overlume_ros/config/default_params.yaml`, `ros/src/overlume_ros/CMakeLists.txt`, `ros/src/overlume_ros/assets/attribution/`, `ros/src/overlume_ros/test/test_asset_paths.py` | 10 | attribution gate, logo asset and parameter, logo path resolution test |
| `tools/vcam_gui.py`, `tools/vcam_ws_bridge.py`, `tools/test_vcam_ws_bridge.py`, `tools/check_mapbox_token.sh`, `tools/test_check_mapbox_token.py`, `tools/validate_visual_mode.sh` | 11 | GUI entries, token check and its offline test, rig refresh |
| `docs/runbooks/mapbox.md` | 12, 17 | 12: create. 17: the three §7.2 values, and any sentence Task 17's runbook-fact check finds drifted |
| `docs/README.md` | plan commit, 12, 17 | plan commit: the plan's entry. 12: the runbook's index entry. 17: marks the plan's entry closed |
| `README.md`, `CHANGELOG.md`, `NOTICE`, `docs/runbooks/environment_bake.md` | 12 | the Mapbox bullet; the Added and two Fixed entries; the Mapbox block; one sentence |
| `docs/status.md` | 12, 15, 17 | 12: open items. 15: the golden promotion's Shipped row. 17: Shipped rows and open items at close-out |
| `docs/plans/2026-10-14-mapbox-buildings.md` (this plan) | every task | the orchestrator updates the ledger; 16 and 17 add the live and rehearsal results |

## Interface contract

These declarations are what lanes build against. A lane that finds one wrong stops and reports
it to the orchestrator; it never changes another lane's interface on its own.

### `overlume/src/mapbox_mvt.hpp` (Task 1; C++17-clean)

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace overlume::mapbox {

// A vertex in MVT tile units: x right, y down, 0..extent inside the tile; the buffer lies outside.
struct MvtPoint {
    int32_t x = 0;
    int32_t y = 0;
};

// An exterior ring and its holes, classified by the MVT v2 winding rule (an exterior ring has a
// positive surveyor's-formula area in tile coordinates). Rings are open: the first vertex is not
// repeated at the end.
struct MvtPolygon {
    std::vector<MvtPoint> outer;
    std::vector<std::vector<MvtPoint>> holes;
};

// One feature of the streets-v8 `building` layer.
struct MvtBuilding {
    std::optional<uint64_t> id;
    std::vector<MvtPolygon> polygons;
    std::optional<double> height;      // `height` (m), from any numeric value type
    std::optional<double> min_height;  // `min_height` (m)
    std::optional<bool> extrude;       // `extrude`: the string "true"/"false" or a bool value
    std::optional<bool> underground;   // `underground`: likewise
    std::string type;                  // `type`, empty when absent
};

struct MvtBuildingLayer {
    bool ok = false;  // false: the tile itself is malformed and nothing in it is usable
    uint32_t extent = 4096;
    std::vector<MvtBuilding> buildings;
    uint32_t skipped_features = 0;  // malformed or non-polygon features left out
};

// Decodes the layer named "building" from an uncompressed MVT tile. Never reads outside
// [data, data + size). A well-formed tile without that layer gives ok = true and no buildings.
MvtBuildingLayer decode_mvt_building_layer(const uint8_t* data, size_t size);

// Mapbox Standard's 3D filter: `extrude` is true and `underground` is not true (a missing
// `underground` counts as above ground).
bool passes_standard_3d_filter(const MvtBuilding& b);

}
```

### `overlume/src/mapbox_loader.hpp` (Task 2 creates it; C++17-clean)

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <optional>
#include <string>

namespace overlume::mapbox {

inline constexpr double kDefaultRadiusM = 500.0;

struct MapboxSpec {
    bool materials_original = false;
    double radius_m = kDefaultRadiusM;
    double brightness = 1.0;
    std::string fallback_dir;
};

struct MapboxSpecResult {
    std::optional<MapboxSpec> spec;  // nullopt: the URI is refused
    std::string error;               // when refused: one line naming the layer list or the key
    std::string warning;             // when accepted with a caveat (an unknown `materials` value)
};

// Parses the text after "mapbox://": the layer list (only "buildings"), then an optional query
// split at the first '?', pairs on '&', the key at the first '='. Unknown keys and pairs without
// '=' are ignored, the last value of a repeated key wins, and a bad value for a known key refuses
// the whole URI. Pure: no logging, no environment access.
MapboxSpecResult parse_mapbox_spec(const std::string& spec);

}
```

### `overlume/src/mapbox_extrude.hpp` (Task 3; C++17-clean)

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

namespace overlume::mapbox {

// A ring in tile units (x right, y down). Open: the first vertex is not repeated.
using TileRing = std::vector<std::array<double, 2>>;

struct TilePolygon {
    TileRing outer;
    std::vector<TileRing> holes;
};

// Maps a point in the polygon's tile units, at z_m metres above the anchor's ground, to output
// metres (x east, y north, z up). Called for every ring vertex left after clipping, once at each
// height; walls and roof share the results.
using ProjectFn = std::function<std::array<double, 3>(double u, double v, double z_m)>;

// Triangles with per-face normals: faces of different orientation never share a vertex.
struct TriangleMesh {
    std::vector<float> positions;   // x, y, z per vertex, output metres
    std::vector<float> normals;     // unit length, x, y, z per vertex
    std::vector<uint32_t> indices;  // counter-clockwise seen from outside
};

struct ExtrudeStats {
    uint32_t polygons_in = 0;
    uint32_t polygons_built = 0;  // non-empty after clipping
    uint32_t wall_quads = 0;
    uint32_t roof_triangles = 0;
};

// Clips each polygon to the square [0, clip_extent]², then extrudes it from min_height_m to
// height_m: a roof at height_m (earcut, holes included) and walls between the two heights. Edges
// that lie on the clip square get no wall; there is no floor. Rings are re-oriented in output
// space (exterior counter-clockwise, holes clockwise) before walls and roofs are built. Appends to
// `out`. Requires finite heights with height_m > min_height_m; returns zero stats otherwise. A
// polygon with a non-finite coordinate is skipped.
ExtrudeStats extrude_polygons(const std::vector<TilePolygon>& polygons, double clip_extent,
                              double min_height_m, double height_m, const ProjectFn& project,
                              TriangleMesh& out);

// Area-weighted centroid of the exterior rings (tile units), before clipping; exteriors with a
// non-finite coordinate are left out. nullopt when the total area is zero.
std::optional<std::array<double, 2>> footprint_centroid(const std::vector<TilePolygon>& polygons);

}
```

### `overlume/src/mapbox_tile_frame.hpp` (Task 4; C++17-clean)

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <array>
#include <cstdint>
#include <optional>

namespace overlume::mapbox {

inline constexpr uint32_t kLandmarkZoom = 14;
inline constexpr uint32_t kFootprintZoom = 16;
inline constexpr double kLandmarkExtent = 8192.0;          // landmark node units per z14 tile
inline constexpr double kMapboxSphereRadiusM = 6371008.8;  // the radius Mapbox's scale uses

// Column-major 4×4, as glm and glTF store matrices.
using Mat4d = std::array<double, 16>;

struct TileId {
    uint32_t z = 0;
    uint32_t x = 0;
    uint32_t y = 0;  // XYZ: row 0 at the north edge
};

struct LonLatDeg {
    double lon = 0.0;
    double lat = 0.0;
};

struct LonLatRect {
    double west = 0.0, south = 0.0, east = 0.0, north = 0.0;  // degrees
};

// Spherical Web Mercator: (u, v) in units of tile t, `extent` units per tile edge → lon/lat.
LonLatDeg tile_point_to_lonlat(const TileId& t, double u, double v, double extent);

// The zoom-z tile containing lon/lat; latitude is clamped to the Web Mercator limit.
TileId tile_containing(double lon_deg, double lat_deg, uint32_t z);

LonLatRect tile_bounds(const TileId& t);

// The 16 z16 tiles of z14 tile t, row-major from the north-west corner.
std::array<TileId, 16> footprint_tiles_of(const TileId& z14);

// The z14 tile's frame: east-north-up at the tile centre, on the WGS84 ellipsoid at
// origin_height_m (cesium's eastNorthUpToFixedFrame).
struct TileFrame {
    TileId tile;
    double origin_height_m = 0.0;
    Mat4d frame_to_ecef{};
    Mat4d ecef_to_frame{};
};

TileFrame make_tile_frame(const TileId& z14, double origin_height_m);

// The exact chain: (u, v) in units of tile `src` (`extent` per edge), z_m metres above
// origin_height_m → longitude and latitude (inverse Web Mercator) → ECEF at origin_height_m + z_m
// → tile-frame metres.
std::array<double, 3> project_to_frame(const TileFrame& f, const TileId& src, double u, double v,
                                       double extent, double z_m);

// Mapbox's landmark scale at a latitude: 8192·2^14 / (2π·6371008.8·cos lat), tile units per metre.
double mapbox_node_scale(double lat_deg);

// A landmark node matrix (local metres → z14 tile units: diag(s, −s, 1) plus (tx, ty, 0)) → the
// affine map from local metres into f, linearised at (tx, ty). nullopt unless the matrix has that
// form with s within 1% of mapbox_node_scale at the node's latitude.
std::optional<Mat4d> landmark_node_to_frame(const TileFrame& f, const Mat4d& node_matrix);

}
```

### `overlume/src/mapbox_landmarks.hpp` (Task 5; C++20)

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

static_assert(__cplusplus >= 202002L, "mapbox_landmarks.hpp is C++20-only (overlume_stream)");

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include <CesiumGltf/Model.h>

#include "mapbox_tile_frame.hpp"

namespace overlume::mapbox {

struct LandmarkTile {
    // The kept building nodes only, all in scene 0; each node's matrix replaced by its
    // local-metres → tile-frame map. No nodes when the GLB held none that were usable.
    CesiumGltf::Model model;
    // Landmark footprint triangles in tile-frame metres: x0, y0, x1, y1, x2, y2.
    std::vector<std::array<double, 6>> footprint_triangles;
    uint32_t buildings_kept = 0;
    uint32_t nodes_skipped = 0;  // a node with an unexpected matrix
    bool ok = false;             // false: unreadable or undecodable; `error` says why
    std::string error;
};

// Reads one landmark GLB: cesium decodes meshopt and quantization, images are not decoded. Keeps
// the "Default Scene" scene (else `scene`, else scene 0), collects footprint triangles from nodes
// whose extras carry `mapbox:footprint:id`, and drops footprint nodes, the LOD scene and everything
// only they used, including meshopt's source and fallback buffers. keep_materials = false (clay)
// also drops textures, images and samplers. A bufferView still carrying EXT_meshopt_compression
// after reading, a required extension the reader does not decode, or a pruned model with a buffer,
// view or texture reference out of range makes the whole tile ok = false. A footprint node with a
// valid matrix yields its triangles even when its building node was skipped.
LandmarkTile read_landmark_tile(std::span<const std::byte> glb, const TileFrame& frame,
                                bool keep_materials);

// True when (x, y) in tile-frame metres lies inside any footprint triangle.
bool inside_any_footprint(const std::vector<std::array<double, 6>>& triangles, double x, double y);

}
```

### `overlume/src/environment_stream.hpp` additions (Tasks 6, 7, 13)

```cpp
// Task 6: FileFixtureAssetAccessor gains a transport switch (a rejected future, as curl's
// accessor throws on DNS failure, refusal or timeout):
explicit FileFixtureAssetAccessor(std::shared_ptr<std::atomic<bool>> killed = nullptr,
                                  std::shared_ptr<std::atomic<bool>> transport_down = nullptr);
// and CountingAssetAccessor counts a rejected request as a failure, then rethrows
// (std::runtime_error carrying the original what()).

// Task 7:
namespace spdlog { class logger; }  // (forward declaration at file scope)

struct CustomTileset {
    std::unique_ptr<Cesium3DTilesSelection::TilesetContentLoader> loader;
    std::unique_ptr<Cesium3DTilesSelection::Tile> root;
    // Select with cesium's area view, a sphere of the source's radius around the ego, as the only
    // view (geometric-error threshold 1.0); also turns preloadSiblings off. (Fog culling needs no
    // switch: cesium 0.64 never fog-culls an area view, TilesetSelection.cpp:471.)
    bool area_selection = false;
};

// StreamingEnvironmentSource's constructor gains one trailing parameter:
//     ..., std::optional<double> radius_override_m = std::nullopt,
//     CustomTileset custom_tileset = {});
// and the class gains:
//     bool area_selection() const;

// Moved out of the anonymous namespace in environment_stream.cpp, bodies unchanged; the default
// argument stays on this declaration only, so the definition drops its `= nullptr`:
std::shared_ptr<spdlog::logger> make_redacting_logger();
Cesium3DTilesSelection::TilesetExternals build_externals(
    std::shared_ptr<CesiumAsync::IAssetAccessor> base, const CesiumAsync::AsyncSystem& asyncSystem,
    const std::string& cache_dir, uint64_t max_cache_items,
    std::shared_ptr<CountingAssetAccessor>* out_counting = nullptr);

// Task 13: a test accessor beside FileFixtureAssetAccessor, reusing its response classes.
// Serves canned responses by URL with the query string removed, or rejects with a transport
// error; records every requested URL in full. Thread-safe.
class MapFixtureAssetAccessor final : public CesiumAsync::IAssetAccessor {
public:
    explicit MapFixtureAssetAccessor(uint16_t unmatched_status = 404);
    void serve(const std::string& url_without_query, uint16_t status, std::vector<std::byte> body);
    void fail_transport(const std::string& url_without_query);
    std::vector<std::string> requested_urls() const;
    // A held URL's requests stay pending until release_held() answers them, so a test can switch
    // sources while a tile load is in flight.
    void hold(const std::string& url_without_query);
    void release_held();
    // IAssetAccessor: get, request, tick.
};
```

### `overlume/src/environment_test_hooks.hpp` additions (Tasks 6, 7)

```cpp
// Task 6: make the fixture network reject every request with a transport error (no response),
// except the root tileset.json, which it spares as kill_fixture_network does: behind the SQLite
// cache the root request can start after the switch flips, and a tileset without a root would
// count one failure whether or not the counter fix is in.
void break_fixture_network_transport(FixtureStreamHandle* handle);

// Task 7: a custom-loader source over a synthetic grid around the anchor (layout in Task 7).
struct FixtureGridHandle;
FixtureGridHandle* install_fixture_grid_source(overlume::VisualRenderer* r,
                                               overlume::GeoAnchor anchor, double radius_m,
                                               bool area_selection);
// Leaf tiles cesium asked the grid loader to load, as (column, row), in request order.
std::vector<std::pair<int, int>> fixture_grid_requested_leaves(FixtureGridHandle* handle);
```

### `overlume/src/environment.hpp` addition (Task 13)

```cpp
// The text after "mapbox://". Reads MAPBOX_TOKEN; nullptr when the URI is refused or the token
// is unset or empty (one log line on the redacting logger either way).
std::unique_ptr<EnvironmentSource> open_mapbox_environment_source(const std::string& mapbox_spec,
                                                                  GeoAnchor anchor);
```

### `overlume/src/mapbox_test_hooks.hpp` (Task 4 creates it; Tasks 5 and 13 add their blocks)

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "overlume/api.h"
#include "overlume/scene.h"

namespace overlume::testing {

// Task 4. A point (u, v) in tile (z, x, y) units (`extent` per edge) at z_m metres above the
// anchor's ground, through the Mapbox placement chain (tile frame of its z14 tile, ECEF, the map
// frame of `anchor`, the stream's height correction) → map metres.
bool mapbox_tile_point_to_map_probe(overlume::GeoAnchor anchor, uint32_t z, uint32_t x, uint32_t y,
                                    double u, double v, double extent, double z_m,
                                    double out_map_xyz[3]);

// Task 5. Its block goes directly after `namespace overlume::testing {`, above Task 4's block, so
// that Task 13's block (appended at the end, possibly before Task 5 merges) merges cleanly.
// inside_any_footprint for C++17 tests (the real one is declared in C++20 mapbox_landmarks.hpp).
bool mapbox_inside_any_footprint_probe(const std::vector<std::array<double, 6>>& triangles,
                                       double x, double y);
struct MapboxLandmarkProbe {
    bool ok = false;
    std::string error;
    uint32_t buildings_kept = 0;
    uint32_t nodes_skipped = 0;
    uint32_t scenes = 0, nodes = 0, meshes = 0, accessors = 0, buffer_views = 0, buffers = 0;
    uint32_t images = 0, textures = 0, samplers = 0, materials = 0;
    uint32_t footprint_triangles = 0;
    uint64_t buffer_bytes = 0;            // summed size of every buffer's data
    bool meshopt_extension_left = false;  // EXT_meshopt_compression anywhere after reading
    std::vector<std::array<double, 16>> node_matrices;  // kept nodes, column-major, in order
};
MapboxLandmarkProbe probe_mapbox_landmark_tile(const std::vector<uint8_t>& glb, uint32_t x14,
                                               uint32_t y14, double origin_height_m,
                                               bool keep_materials);

// Task 13.
struct MapboxFixtureNetwork;  // wraps a MapFixtureAssetAccessor; never freed
MapboxFixtureNetwork* make_mapbox_fixture_network(uint16_t unmatched_status = 404);
void mapbox_fixture_serve(MapboxFixtureNetwork* n, const std::string& url_without_query,
                          uint16_t status, std::vector<uint8_t> body);
void mapbox_fixture_fail_transport(MapboxFixtureNetwork* n, const std::string& url_without_query);
// Requests for a held URL stay pending until mapbox_fixture_release_held answers them as served.
void mapbox_fixture_hold(MapboxFixtureNetwork* n, const std::string& url_without_query);
void mapbox_fixture_release_held(MapboxFixtureNetwork* n);
std::vector<std::string> mapbox_fixture_requested_urls(MapboxFixtureNetwork* n);
std::string mapbox_landmark_url(uint32_t x14, uint32_t y14);   // no query string
std::string mapbox_footprint_url(uint32_t x16, uint32_t y16);  // no query string
// Opens a mapbox:// URI exactly as set_environment_source does (parse, source, set_visible,
// teardown of the old source, swap), but over `n` and with `token` in place of MAPBOX_TOKEN.
bool install_fixture_mapbox_source(overlume::VisualRenderer* r, const char* mapbox_uri,
                                   overlume::GeoAnchor anchor, MapboxFixtureNetwork* n,
                                   const char* token);
// Counting rules: a tile counts once, when it reaches Success with content. A feature counts once
// per z16 square it is decoded from (a building clipped into two squares counts twice), and in
// exactly one of extruded_features (it produced geometry) or conflated_features (dropped under a
// landmark footprint); filtered-out features (extrude=false, underground, no height) count in
// neither.
struct MapboxSourceStats {
    uint64_t tiles_built = 0;         // z14 tiles that reached Success with content
    uint64_t landmark_nodes = 0;      // building nodes kept, summed over built tiles
    uint64_t extruded_features = 0;   // features extruded, summed
    uint64_t conflated_features = 0;  // features dropped under a landmark footprint (Task 14)
    uint64_t retries = 0;             // attempts that ended in RetryLater (waits not counted)
    double max_build_ms = 0.0;        // slowest tile build, worker time
    uint64_t wall_quads = 0;          // wall quads of the extruded features, summed
};
// The counters of the most recently opened Mapbox source in this process.
MapboxSourceStats mapbox_last_source_stats();
// Test-only: while on, every GLB the stream writes in prepareInLoadThread is kept; turning it on
// drops what was kept before.
void capture_stream_glbs(bool on);
std::vector<std::vector<uint8_t>> captured_stream_glbs();
// Test-only: freezes the clock the loader's retry wait reads at `seconds`; a negative value gives
// the steady clock back.
void mapbox_set_test_clock(double seconds);

}
```

### Test fixture headers (header-only, C++17, `overlume/tests/`)

```cpp
// mapbox_mvt_fixture.hpp (Task 1); namespace overlume::testing::mvt
struct Value;    // a vector-tile value: string, float, double, int64, uint64, sint64 or bool
Value str(std::string s);  Value dbl(double d);  Value flt(float f);
Value i64(int64_t i);      Value u64(uint64_t u); Value s64(int64_t i);  Value boolean(bool b);
struct Feature {
    std::optional<uint64_t> id;
    std::vector<std::vector<std::array<int32_t, 2>>> rings;  // open rings, written as given
    std::vector<std::pair<std::string, Value>> tags;
    uint32_t geom_type = 3;                                   // POLYGON
};
struct Layer {
    std::string name;
    uint32_t extent = 4096;
    uint32_t version = 2;
    std::vector<Feature> features;
};
std::vector<uint8_t> encode_tile(const std::vector<Layer>& layers);
std::vector<uint8_t> gzip_stored(const std::vector<uint8_t>& raw);  // gzip framing, stored blocks
// A streets-v8-style building feature: height, min_height, extrude, underground, type.
Feature building(uint64_t id, std::vector<std::vector<std::array<int32_t, 2>>> rings,
                 double height, double min_height = 0.0, bool extrude = true,
                 bool underground = false, std::string type = "building");

// mapbox_glb_fixture.hpp (Task 5); namespace overlume::testing::glb
struct LandmarkSpec {
    uint64_t id = 1;
    double tx = 4096.0, ty = 4096.0;  // node anchor, z14 tile units (0..8192)
    double lat_deg = 25.0;            // latitude for Mapbox's node scale
    double half_x_m = 10.0, half_y_m = 10.0, height_m = 100.0;  // the building box, local metres
    double footprint_half_m = 12.0;   // the footprint square's half size, local metres
};
struct GlbOptions {
    bool meshopt = true;          // EXT_meshopt_compression, vertex codec v0, on vertex and index views
    bool quantize = true;         // KHR_mesh_quantization: NORMAL normalized byte, TEXCOORD_0 normalized ushort
    bool lod_scene = true;        // a second scene "LOD" with low-detail duplicate nodes
    bool occlusion_texture = true;  // a material with a small JPEG occlusionTexture (stb)
    bool bad_matrix_on_first = false;  // the first building node gets a rotated matrix
    bool corrupt_meshopt = false;      // the first compressed vertex stream is garbled
};
std::vector<uint8_t> build_landmark_glb(const std::vector<LandmarkSpec>& landmarks,
                                        const GlbOptions& opts);
```

### Texts other tasks match verbatim

- `docs/runbooks/mapbox.md` §7.2: Task 12 writes these rows, each once in the file, and Task 17
  replaces each by exact match: ``| `geo_datum_lat_deg` | filled in by Task 17's rehearsal |``,
  ``| `geo_datum_lon_deg` | filled in by Task 17's rehearsal |`` and
  ``| `geo_datum_heading_deg` | filled in by Task 17's rehearsal |``.
- Status ledger rows begin `| 16. Opt-in live smoke | 8 |` and
  `| 17. Rig rehearsal at both sites and close-out | 8 |`; marking a row changes only its State and
  Commit cells.
- Node log texts that Tasks 16 and 17 grep, which no task changes: `geo-anchor solved: --anchor-lat …`,
  `geo-anchor solved (geo_datum override): …`, `environment source armed: '%s' (%s)` and
  `environment source: network loss detected -- switched to fallback dir '%s'`.

---

## Tasks

### Task 0: CMake wiring for the Mapbox units

**Lane:** 0 (the orchestrator, in `$INTEGRATION`, before any lane worktree exists).
**Depends on:** this plan's commit.

**Files:**
- Modify: `overlume/CMakeLists.txt` (the `environment_stream.cpp` removal at ~237, the
  `overlume_stream` target at ~257, the test list at ~583, the per-test carve-outs at ~612)
- Modify: `overlume/src/environment_stream.hpp:6-8` (the C++20 guard's message)

**Interfaces:**
- Consumes: nothing.
- Produces: any `overlume/src/mapbox_*.cpp` compiles as C++20 in `overlume_stream` (cesium builds
  only); any `overlume/tests/test_mapbox_*.cpp` is a cesium-only test executable that can include
  `<meshoptimizer.h>`.

- [ ] **Step 1: First build of the integration worktree** (also warms ccache for the lanes)

```bash
cd "$INTEGRATION"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake \
    --toolchain "$PWD/overlume/cmake/toolchain-clang-libcxx.cmake" -S overlume -B overlume/build \
    -DOVERLUME_ENABLE_CESIUM=ON -DCMAKE_C_COMPILER_LAUNCHER=ccache \
    -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j16
ros/colcon_build.sh
```

Expected: configure ends `-- Build files have been written to: …/overlume/build`; the build and
`colcon build` finish without errors.

- [ ] **Step 2: Route `src/mapbox_*.cpp` into `overlume_stream`**

In `overlume/CMakeLists.txt`, replace

```cmake
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/environment_stream.cpp")
    list(REMOVE_ITEM OVERLUME_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/environment_stream.cpp")
endif()
```

with

```cmake
# Cesium-only translation units: C++20, compiled in overlume_stream, never in overlume_obj.
file(GLOB _overlume_stream_srcs CONFIGURE_DEPENDS
     "${CMAKE_CURRENT_SOURCE_DIR}/src/environment_stream.cpp"
     "${CMAKE_CURRENT_SOURCE_DIR}/src/mapbox_*.cpp")
if(_overlume_stream_srcs)
    list(REMOVE_ITEM OVERLUME_SOURCES ${_overlume_stream_srcs})
endif()
```

and replace

```cmake
    add_library(overlume_stream OBJECT src/environment_stream.cpp)
```

with

```cmake
    add_library(overlume_stream OBJECT ${_overlume_stream_srcs})
```

- [ ] **Step 3: Make `tests/test_mapbox_*.cpp` cesium-only, with meshoptimizer's headers**

Replace

```cmake
    if(NOT OVERLUME_ENABLE_CESIUM)
        list(REMOVE_ITEM _overlume_test_srcs
             "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_environment_stream.cpp")
    endif()
```

with

```cmake
    if(NOT OVERLUME_ENABLE_CESIUM)
        list(REMOVE_ITEM _overlume_test_srcs
             "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_environment_stream.cpp")
        list(FILTER _overlume_test_srcs EXCLUDE REGEX "/test_mapbox_[a-z_]+\\.cpp$")
    endif()
```

and after the `test_gltf_normals` carve-out (`endif()` at ~618) add

```cmake
        if(_test_name MATCHES "^test_mapbox_")
            # The landmark fixtures use meshoptimizer's EXT_meshopt_compression encoder; its
            # archive is already on the link line through CesiumGltfReader.
            target_include_directories(${_test_name} PRIVATE
                $<TARGET_PROPERTY:meshoptimizer::meshoptimizer,INTERFACE_INCLUDE_DIRECTORIES>)
        endif()
```

- [ ] **Step 4: Widen the C++20 guard's message** in `overlume/src/environment_stream.hpp`

```cpp
static_assert(__cplusplus >= 202002L,
              "environment_stream.hpp is C++20-only -- include it only from overlume_stream "
              "sources (environment_stream.cpp, mapbox_*.cpp)");
```

- [ ] **Step 5: Prove the wiring with throwaway files** (never committed)

```bash
cd "$INTEGRATION"
cat > overlume/src/mapbox_wiring_probe.cpp <<'EOF'
static_assert(__cplusplus >= 202002L, "must compile as C++20");
#include <CesiumUtility/Gzip.h>
#include <mapbox/earcut.hpp>
EOF
cat > overlume/tests/test_mapbox_wiring_probe.cpp <<'EOF'
#include <gtest/gtest.h>
#include <meshoptimizer.h>
TEST(MapboxWiringProbe, MeshoptHeaderAndArchiveReachTests) {
    EXPECT_GT(meshopt_encodeVertexBufferBound(3, 12), 0u);
}
EOF
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j16 \
    && overlume/build/test_mapbox_wiring_probe
rm overlume/src/mapbox_wiring_probe.cpp overlume/tests/test_mapbox_wiring_probe.cpp
cmake -S overlume -B overlume/build > /dev/null && echo reconfigured
```

Expected: the build succeeds (the probe source compiles as C++20 in `overlume_stream`), the test
prints `[  PASSED  ] 1 test.`, and after removal `reconfigured`.

- [ ] **Step 6: Gate**

```bash
cd "$INTEGRATION"
cmake --build overlume/build -j8 \
    && flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock tools/ci_visual_mode.sh
```

Expected: six `PASS` lines, exit 0, and the baseline counts exactly (Task 0 adds no test):
347 library ctest entries (331 ok, 6 skipped), 363 node tests, 82 WS bridge tests, 23 goldens.

- [ ] **Step 7: Commit**

```bash
cd "$INTEGRATION"
git add overlume/CMakeLists.txt overlume/src/environment_stream.hpp
git commit -F - <<'EOF'
build(cmake): compile src/mapbox_*.cpp as C++20 in overlume_stream

The Mapbox buildings source (docs/plans/2026-10-14-mapbox-buildings.md) adds
cesium-typed units that must build like environment_stream.cpp: C++20, in
overlume_stream, cesium builds only. Their tests (tests/test_mapbox_*.cpp) are
dropped with cesium off and get meshoptimizer's headers for the
EXT_meshopt_compression fixtures. The lanes fork after this commit, so none of
them edits CMakeLists.txt.

Claude-Session: <the executing session's URL>
EOF
```

Then create the lane worktrees (Execution).

---

### Task 1: MVT building-layer decoder

**Lane:** 1. **Depends on:** Task 0 (`src/mapbox_*.cpp` compile as C++20 in `overlume_stream`;
`tests/test_mapbox_*.cpp` are cesium-only test executables).

**Files:**
- Create: `overlume/src/mapbox_mvt.hpp` (exactly the contract)
- Create: `overlume/src/mapbox_mvt.cpp`
- Create: `overlume/tests/mapbox_mvt_fixture.hpp` (header-only, C++17, `overlume::testing::mvt`)
- Test: `overlume/tests/test_mapbox_mvt.cpp` (suite `MapboxMvt`, 17 tests)

**Interfaces:**
- Consumes: nothing but the standard library.
- Produces, for Task 13 (verbatim from the contract):
  - `MvtBuildingLayer decode_mvt_building_layer(const uint8_t* data, size_t size);`
  - `bool passes_standard_3d_filter(const MvtBuilding& b);`
  - the structs `MvtPoint`, `MvtPolygon`, `MvtBuilding` and `MvtBuildingLayer`, all in
    `overlume::mapbox`.
  - What the loader may rely on: `ok == false` means `buildings` is empty and `skipped_features`
    is 0; when `ok`, `extent > 0`; every ring is open, has no repeated consecutive vertex and a
    non-zero area (so at least three vertices); an exterior ring has a positive surveyor's area in
    tile units (y down) and a hole a negative one; `height` and `min_height` are finite when present;
    `skipped_features` counts every feature of the layer that is not in `buildings`.
  - The fixture, `overlume/tests/mapbox_mvt_fixture.hpp` (namespace `overlume::testing::mvt`):
    `struct Value { std::vector<uint8_t> message; };` (one encoded Value message),
    `Value str(std::string s);`, `Value dbl(double d);`, `Value flt(float f);`,
    `Value i64(int64_t i);`, `Value u64(uint64_t u);`, `Value s64(int64_t i);`,
    `Value boolean(bool b);`, `struct Feature` and `struct Layer` as the contract declares them,
    `std::vector<uint8_t> encode_tile(const std::vector<Layer>& layers);`,
    `std::vector<uint8_t> gzip_stored(const std::vector<uint8_t>& raw);` and
    `Feature building(uint64_t id, std::vector<std::vector<std::array<int32_t, 2>>> rings,
    double height, double min_height = 0.0, bool extrude = true, bool underground = false,
    std::string type = "building");`. Beyond the contract, and additive only: the protobuf writers
    in `overlume::testing::mvt::detail` (`put_varint`, `put_tag`, `put_bytes`, `put_string`,
    `put_le`, `put_packed`, `command`, `zigzag32`), which tests use to hand-build malformed input.

**API notes:**
- `CesiumUtility::isGzip` and `gunzip` (cesium-native 0.64.0,
  `overlume/build/_deps/cesium-native-src/CesiumUtility/src/Gzip.cpp:18-23` and `:68-104`): `isGzip`
  wants at least 3 bytes starting `1f 8b`; `gunzip` runs `zng_inflateInit2(&strm, 16 + MAX_WBITS)`
  (gzip wrapper only) and returns false on `Z_DATA_ERROR` or `Z_BUF_ERROR`, so a wrong CRC-32 or
  ISIZE, or a truncated stream, fails. `gzip_stored` therefore writes a complete RFC 1952 header and
  trailer. Task 13 makes that call; these C++17 tests include no cesium header.
- MVT 2.1 (vector_tile.proto field numbers; geometry commands, section 4.3; exterior winding,
  section 4.3.4.4), and the streets-v8 facts the decoder serves
  (`docs/design/2026-10-14-mapbox-buildings-design.md:75-87`: extent 4096, a 33-unit buffer, uint64
  ids, holes and multipolygons, `height`/`min_height` as doubles, `extrude`/`underground` as strings).
- `std::bit_cast` (C++20 `<bit>`): in the toolchain's clang 18.1.8 / libc++ 18; `mapbox_mvt.cpp`
  compiles as C++20 in `overlume_stream` (Task 0).
- Test build: `overlume/CMakeLists.txt` ~581 makes each `tests/*.cpp` its own executable, ~609-611
  add `src` and stb to its include path and link `overlume gtest gtest_main`; tests compile as C++17
  with no cesium or vcpkg include. googletest 1.15.2 (~569) prints `std::optional`
  (`overlume/build/_deps/googletest-src/googletest/include/gtest/gtest-printers.h:882-908`) and
  direct-initialises `AssertionResult`'s bool (`gtest-assertion-result.h:155-161`), so
  `ASSERT_TRUE(an_optional)` compiles; gmock is off (~575), so substring checks use
  `std::string::find`.

- [ ] **Step 1: Write the contract header** `overlume/src/mapbox_mvt.hpp`

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace overlume::mapbox {

// A vertex in MVT tile units: x right, y down, 0..extent inside the tile; the buffer lies outside.
struct MvtPoint {
    int32_t x = 0;
    int32_t y = 0;
};

// An exterior ring and its holes, classified by the MVT v2 winding rule (an exterior ring has a
// positive surveyor's-formula area in tile coordinates). Rings are open: the first vertex is not
// repeated at the end.
struct MvtPolygon {
    std::vector<MvtPoint> outer;
    std::vector<std::vector<MvtPoint>> holes;
};

// One feature of the streets-v8 `building` layer.
struct MvtBuilding {
    std::optional<uint64_t> id;
    std::vector<MvtPolygon> polygons;
    std::optional<double> height;      // `height` (m), from any numeric value type
    std::optional<double> min_height;  // `min_height` (m)
    std::optional<bool> extrude;       // `extrude`: the string "true"/"false" or a bool value
    std::optional<bool> underground;   // `underground`: likewise
    std::string type;                  // `type`, empty when absent
};

struct MvtBuildingLayer {
    bool ok = false;  // false: the tile itself is malformed and nothing in it is usable
    uint32_t extent = 4096;
    std::vector<MvtBuilding> buildings;
    uint32_t skipped_features = 0;  // malformed or non-polygon features left out
};

// Decodes the layer named "building" from an uncompressed MVT tile. Never reads outside
// [data, data + size). A well-formed tile without that layer gives ok = true and no buildings.
MvtBuildingLayer decode_mvt_building_layer(const uint8_t* data, size_t size);

// Mapbox Standard's 3D filter: `extrude` is true and `underground` is not true (a missing
// `underground` counts as above ground).
bool passes_standard_3d_filter(const MvtBuilding& b);

}
```

- [ ] **Step 2: Write the fixture** `overlume/tests/mapbox_mvt_fixture.hpp`

It writes tiles the way the decoder must expect them in the wild: each layer's key and value tables
after its features, deduplicated, ids and geometry as given (rings are not re-oriented or closed),
and `gzip_stored` as a stream any inflater accepts.

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

// Mapbox Vector Tiles (MVT 2.1 protobuf) and gzip streams synthesized in the test process, so no
// Mapbox bytes enter the tree. Header-only, C++17.

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace overlume::testing::mvt {

// A vector-tile value (string, float, double, int64, uint64, sint64 or bool) as its encoded Value
// message.
struct Value {
    std::vector<uint8_t> message;
};

struct Feature {
    std::optional<uint64_t> id;
    std::vector<std::vector<std::array<int32_t, 2>>> rings;  // open rings, written as given
    std::vector<std::pair<std::string, Value>> tags;
    uint32_t geom_type = 3;  // POLYGON
};

struct Layer {
    std::string name;
    uint32_t extent = 4096;
    uint32_t version = 2;
    std::vector<Feature> features;
};

// Protobuf writers; tests also use them to hand-build malformed input.
namespace detail {

inline void put_varint(std::vector<uint8_t>& out, uint64_t v) {
    for (; v >= 0x80; v >>= 7) out.push_back(static_cast<uint8_t>(v | 0x80));
    out.push_back(static_cast<uint8_t>(v));
}

inline void put_tag(std::vector<uint8_t>& out, uint32_t field, uint32_t wire) {
    put_varint(out, (uint64_t{field} << 3) | wire);
}

inline void put_bytes(std::vector<uint8_t>& out, uint32_t field, const std::vector<uint8_t>& b) {
    put_tag(out, field, 2);
    put_varint(out, b.size());
    out.insert(out.end(), b.begin(), b.end());
}

inline void put_string(std::vector<uint8_t>& out, uint32_t field, const std::string& s) {
    put_bytes(out, field, std::vector<uint8_t>(s.begin(), s.end()));
}

inline void put_le(std::vector<uint8_t>& out, uint64_t v, int bytes) {
    for (int i = 0; i < bytes; ++i) out.push_back(static_cast<uint8_t>(v >> (8 * i)));
}

// Packed repeated uint32: feature tags and geometry.
inline void put_packed(std::vector<uint8_t>& out, uint32_t field, const std::vector<uint32_t>& v) {
    std::vector<uint8_t> payload;
    for (uint32_t x : v) put_varint(payload, x);
    put_bytes(out, field, payload);
}

inline uint32_t command(uint32_t id, uint32_t count) { return (id & 7) | (count << 3); }

inline uint32_t zigzag32(int32_t v) {
    return (static_cast<uint32_t>(v) << 1) ^ static_cast<uint32_t>(v >> 31);
}

inline Value scalar(uint32_t field, uint32_t wire, uint64_t bits) {
    Value v;
    put_tag(v.message, field, wire);
    if (wire == 0) {
        put_varint(v.message, bits);
    } else {
        put_le(v.message, bits, wire == 1 ? 8 : 4);
    }
    return v;
}

// The index of `item` in a layer's key or value table, appended when new.
template <typename T>
uint32_t intern(std::vector<T>& table, const T& item) {
    const auto it = std::find(table.begin(), table.end(), item);
    if (it != table.end()) return static_cast<uint32_t>(it - table.begin());
    table.push_back(item);
    return static_cast<uint32_t>(table.size() - 1);
}

inline uint32_t crc32_ieee(const std::vector<uint8_t>& bytes) {
    uint32_t crc = 0xffffffffu;
    for (uint8_t b : bytes) {
        crc ^= b;
        for (int k = 0; k < 8; ++k) crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}

}

inline Value str(std::string s) {
    Value v;
    detail::put_string(v.message, 1, s);
    return v;
}

inline Value flt(float f) {
    uint32_t bits = 0;
    std::memcpy(&bits, &f, sizeof bits);
    return detail::scalar(2, 5, bits);
}

inline Value dbl(double d) {
    uint64_t bits = 0;
    std::memcpy(&bits, &d, sizeof bits);
    return detail::scalar(3, 1, bits);
}

inline Value i64(int64_t i) { return detail::scalar(4, 0, static_cast<uint64_t>(i)); }

inline Value u64(uint64_t u) { return detail::scalar(5, 0, u); }

inline Value s64(int64_t i) {
    return detail::scalar(6, 0, (static_cast<uint64_t>(i) << 1) ^ static_cast<uint64_t>(i >> 63));
}

inline Value boolean(bool b) { return detail::scalar(7, 0, b ? 1 : 0); }

// Each layer is written version, name, extent, features, then the key and value tables: the order
// that makes a decoder read the whole layer before it can resolve a feature's tags.
inline std::vector<uint8_t> encode_tile(const std::vector<Layer>& layers) {
    std::vector<uint8_t> tile;
    for (const Layer& layer : layers) {
        std::vector<std::string> keys;
        std::vector<std::vector<uint8_t>> values;
        std::vector<uint8_t> msg;
        detail::put_tag(msg, 15, 0);
        detail::put_varint(msg, layer.version);
        detail::put_string(msg, 1, layer.name);
        detail::put_tag(msg, 5, 0);
        detail::put_varint(msg, layer.extent);
        for (const Feature& f : layer.features) {
            std::vector<uint32_t> tags;
            for (const auto& [key, value] : f.tags) {
                tags.push_back(detail::intern(keys, key));
                tags.push_back(detail::intern(values, value.message));
            }
            std::vector<uint32_t> geometry;
            int32_t cx = 0, cy = 0;  // the cursor persists across rings
            for (const auto& ring : f.rings) {
                for (size_t i = 0; i < ring.size(); ++i) {
                    if (i == 0) geometry.push_back(detail::command(1, 1));  // MoveTo
                    if (i == 1) {
                        const auto line_to = static_cast<uint32_t>(ring.size() - 1);
                        geometry.push_back(detail::command(2, line_to));  // LineTo
                    }
                    geometry.push_back(detail::zigzag32(ring[i][0] - cx));
                    geometry.push_back(detail::zigzag32(ring[i][1] - cy));
                    cx = ring[i][0];
                    cy = ring[i][1];
                }
                geometry.push_back(detail::command(7, 1));  // ClosePath
            }
            std::vector<uint8_t> feature;
            if (f.id) {
                detail::put_tag(feature, 1, 0);
                detail::put_varint(feature, *f.id);
            }
            detail::put_packed(feature, 2, tags);
            detail::put_tag(feature, 3, 0);
            detail::put_varint(feature, f.geom_type);
            detail::put_packed(feature, 4, geometry);
            detail::put_bytes(msg, 2, feature);
        }
        for (const std::string& key : keys) detail::put_string(msg, 3, key);
        for (const std::vector<uint8_t>& value : values) detail::put_bytes(msg, 4, value);
        detail::put_bytes(tile, 3, msg);
    }
    return tile;
}

// A complete gzip stream (RFC 1952) of stored deflate blocks (RFC 1951 3.2.4), so any inflater,
// CesiumUtility::gunzip included, reads it back and verifies its CRC-32 and size.
inline std::vector<uint8_t> gzip_stored(const std::vector<uint8_t>& raw) {
    std::vector<uint8_t> out = {0x1f, 0x8b, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff};
    size_t pos = 0;
    do {
        const size_t n = std::min<size_t>(raw.size() - pos, 0xffff);
        out.push_back(pos + n == raw.size() ? 0x01 : 0x00);  // BFINAL, BTYPE 00 (stored)
        detail::put_le(out, n, 2);
        detail::put_le(out, ~n & 0xffff, 2);  // NLEN
        out.insert(out.end(), raw.begin() + static_cast<std::ptrdiff_t>(pos),
                   raw.begin() + static_cast<std::ptrdiff_t>(pos + n));
        pos += n;
    } while (pos < raw.size());
    detail::put_le(out, detail::crc32_ieee(raw), 4);
    detail::put_le(out, raw.size() & 0xffffffffu, 4);  // ISIZE
    return out;
}

// A streets-v8-style building feature: height, min_height, extrude, underground, type.
inline Feature building(uint64_t id, std::vector<std::vector<std::array<int32_t, 2>>> rings,
                        double height, double min_height = 0.0, bool extrude = true,
                        bool underground = false, std::string type = "building") {
    Feature f;
    f.id = id;
    f.rings = std::move(rings);
    f.tags = {{"height", dbl(height)},
              {"min_height", dbl(min_height)},
              {"extrude", str(extrude ? "true" : "false")},
              {"underground", str(underground ? "true" : "false")},
              {"type", str(std::move(type))}};
    return f;
}

}
```

- [ ] **Step 3: Write the failing test** `overlume/tests/test_mapbox_mvt.cpp`

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "mapbox_mvt.hpp"

#include "mapbox_mvt_fixture.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace {

namespace mvt = overlume::testing::mvt;
using overlume::mapbox::decode_mvt_building_layer;
using overlume::mapbox::MvtBuilding;
using overlume::mapbox::MvtBuildingLayer;
using overlume::mapbox::MvtPoint;
using overlume::mapbox::MvtPolygon;
using Ring = std::vector<std::array<int32_t, 2>>;

MvtBuildingLayer decode(const std::vector<uint8_t>& tile) {
    return decode_mvt_building_layer(tile.data(), tile.size());
}

std::vector<uint8_t> bytes(const std::string& s) { return {s.begin(), s.end()}; }

mvt::Layer building_layer(std::vector<mvt::Feature> features) {
    mvt::Layer layer;
    layer.name = "building";
    layer.features = std::move(features);
    return layer;
}

// Clockwise on screen (y down), so a positive surveyor's area: an exterior ring.
Ring square(int32_t x, int32_t y, int32_t size) {
    return {{x, y}, {x + size, y}, {x + size, y + size}, {x, y + size}};
}

// Wound the other way: a hole.
Ring square_hole(int32_t x, int32_t y, int32_t size) {
    return {{x, y}, {x, y + size}, {x + size, y + size}, {x + size, y}};
}

Ring as_ring(const std::vector<MvtPoint>& points) {
    Ring ring;
    for (const MvtPoint& p : points) ring.push_back({p.x, p.y});
    return ring;
}

// The one building of a tile whose single feature (a square, no id) carries only `key` = `value`.
MvtBuilding with_tag(const std::string& key, mvt::Value value) {
    mvt::Feature f;
    f.rings = {square(0, 0, 10)};
    f.tags = {{key, std::move(value)}};
    const MvtBuildingLayer out = decode(mvt::encode_tile({building_layer({f})}));
    EXPECT_TRUE(out.ok);
    EXPECT_EQ(out.buildings.size(), 1u);
    return out.buildings.empty() ? MvtBuilding{} : out.buildings[0];
}

uint32_t cmd(uint32_t id, uint32_t count) { return mvt::detail::command(id, count); }

uint32_t zz(int32_t v) { return mvt::detail::zigzag32(v); }

// A 10 x 10 exterior ring as raw geometry: MoveTo, LineTo(3), ClosePath.
const std::vector<uint32_t> kSquare10 = {cmd(1, 1), zz(0),  zz(0),   cmd(2, 3), zz(10),   zz(0),
                                         zz(0),     zz(10), zz(-10), zz(0),     cmd(7, 1)};

template <typename T>
std::vector<T> with_tail(std::vector<T> head, const std::vector<T>& tail) {
    head.insert(head.end(), tail.begin(), tail.end());
    return head;
}

// A raw POLYGON Feature message, for tags and geometry the encoder never writes. Tag indices are
// 64-bit, so a test can write one past 32 bits.
std::vector<uint8_t> raw_feature(const std::vector<uint64_t>& tags,
                                 const std::vector<uint32_t>& geometry) {
    std::vector<uint8_t> packed_tags;
    for (const uint64_t t : tags) mvt::detail::put_varint(packed_tags, t);
    std::vector<uint8_t> f;
    mvt::detail::put_bytes(f, 2, packed_tags);
    mvt::detail::put_tag(f, 3, 0);
    mvt::detail::put_varint(f, 3);
    mvt::detail::put_packed(f, 4, geometry);
    return f;
}

// A raw building layer: `features` as given, the one key "height" and the one value `value`.
std::vector<uint8_t> raw_building_layer(
    const std::vector<std::vector<uint8_t>>& features,
    const std::vector<uint8_t>& value = mvt::dbl(10.0).message) {
    std::vector<uint8_t> layer;
    mvt::detail::put_string(layer, 1, "building");
    for (const std::vector<uint8_t>& f : features) mvt::detail::put_bytes(layer, 2, f);
    mvt::detail::put_string(layer, 3, "height");
    mvt::detail::put_bytes(layer, 4, value);
    return layer;
}

std::vector<uint8_t> raw_tile(const std::vector<std::vector<uint8_t>>& layers) {
    std::vector<uint8_t> tile;
    for (const std::vector<uint8_t>& layer : layers) mvt::detail::put_bytes(tile, 3, layer);
    return tile;
}

uint32_t le32(const std::vector<uint8_t>& b, size_t at) {
    return uint32_t{b[at]} | uint32_t{b[at + 1]} << 8 | uint32_t{b[at + 2]} << 16 |
           uint32_t{b[at + 3]} << 24;
}

// Reads a gzip stream of stored deflate blocks back. false unless every part is well formed.
bool read_gzip_stored(const std::vector<uint8_t>& gz, std::vector<uint8_t>& raw, uint32_t& crc,
                      uint32_t& isize) {
    const std::vector<uint8_t> header = {0x1f, 0x8b, 0x08, 0x00, 0x00,
                                         0x00, 0x00, 0x00, 0x00, 0xff};
    if (gz.size() < header.size() || !std::equal(header.begin(), header.end(), gz.begin()))
        return false;
    size_t p = header.size();
    for (bool last = false; !last;) {
        if (p + 5 > gz.size() || (gz[p] & 0xfe) != 0) return false;  // BTYPE 00 only
        last = (gz[p] & 1) != 0;
        const size_t len = gz[p + 1] | gz[p + 2] << 8;
        const size_t nlen = gz[p + 3] | gz[p + 4] << 8;
        if ((len ^ nlen) != 0xffff || p + 5 + len > gz.size()) return false;
        raw.insert(raw.end(), gz.begin() + p + 5, gz.begin() + p + 5 + len);
        p += 5 + len;
    }
    if (p + 8 != gz.size()) return false;
    crc = le32(gz, p);
    isize = le32(gz, p + 4);
    return true;
}

}

TEST(MapboxMvt, SquareBuildingKeepsItsIdRingAndFields) {
    const uint64_t id = (uint64_t{1} << 62) + 7;  // a nine-byte varint
    mvt::Layer layer = building_layer(
        {mvt::building(id, {square(100, 200, 50)}, 35.0, 3.0, true, false, "building:part")});
    layer.extent = 8192;
    const MvtBuildingLayer out = decode(mvt::encode_tile({layer}));
    ASSERT_TRUE(out.ok);
    EXPECT_EQ(out.extent, 8192u);
    EXPECT_EQ(out.skipped_features, 0u);
    ASSERT_EQ(out.buildings.size(), 1u);
    const MvtBuilding& b = out.buildings[0];
    EXPECT_EQ(b.id, std::optional<uint64_t>(id));
    ASSERT_EQ(b.polygons.size(), 1u);
    EXPECT_EQ(as_ring(b.polygons[0].outer), square(100, 200, 50));
    EXPECT_TRUE(b.polygons[0].holes.empty());
    EXPECT_EQ(b.height, std::optional<double>(35.0));
    EXPECT_EQ(b.min_height, std::optional<double>(3.0));
    EXPECT_EQ(b.extrude, std::optional<bool>(true));
    EXPECT_EQ(b.underground, std::optional<bool>(false));
    EXPECT_EQ(b.type, "building:part");
}

TEST(MapboxMvt, CourtyardHoleBelongsToItsExteriorInVersions1And2) {
    for (const uint32_t version : {1u, 2u}) {
        mvt::Layer layer =
            building_layer({mvt::building(7, {square(0, 0, 100), square_hole(30, 30, 40)}, 20.0)});
        layer.version = version;
        const MvtBuildingLayer out = decode(mvt::encode_tile({layer}));
        ASSERT_TRUE(out.ok) << "version " << version;
        ASSERT_EQ(out.buildings.size(), 1u) << "version " << version;
        ASSERT_EQ(out.buildings[0].polygons.size(), 1u) << "version " << version;
        const MvtPolygon& polygon = out.buildings[0].polygons[0];
        EXPECT_EQ(as_ring(polygon.outer), square(0, 0, 100)) << "version " << version;
        ASSERT_EQ(polygon.holes.size(), 1u) << "version " << version;
        EXPECT_EQ(as_ring(polygon.holes[0]), square_hole(30, 30, 40)) << "version " << version;
    }
}

TEST(MapboxMvt, MultipolygonStartsAPolygonAtEachExterior) {
    const MvtBuildingLayer out = decode(mvt::encode_tile({building_layer({mvt::building(
        8, {square(0, 0, 10), square(50, 50, 30), square_hole(60, 60, 10)}, 12.0)})}));
    ASSERT_TRUE(out.ok);
    ASSERT_EQ(out.buildings.size(), 1u);
    const std::vector<MvtPolygon>& polygons = out.buildings[0].polygons;
    ASSERT_EQ(polygons.size(), 2u);
    EXPECT_EQ(as_ring(polygons[0].outer), square(0, 0, 10));
    EXPECT_TRUE(polygons[0].holes.empty());
    EXPECT_EQ(as_ring(polygons[1].outer), square(50, 50, 30));
    ASSERT_EQ(polygons[1].holes.size(), 1u);
    EXPECT_EQ(as_ring(polygons[1].holes[0]), square_hole(60, 60, 10));
}

TEST(MapboxMvt, HeightFromEveryNumericValueType) {
    EXPECT_EQ(with_tag("height", mvt::dbl(828.5)).height, std::optional<double>(828.5));
    EXPECT_EQ(with_tag("height", mvt::flt(12.25f)).height, std::optional<double>(12.25));
    EXPECT_EQ(with_tag("height", mvt::i64(-3)).height, std::optional<double>(-3.0))
        << "a negative int64 is a ten-byte varint";
    EXPECT_EQ(with_tag("height", mvt::u64(40)).height, std::optional<double>(40.0));
    EXPECT_EQ(with_tag("height", mvt::s64(-7)).height, std::optional<double>(-7.0));
    EXPECT_EQ(with_tag("min_height", mvt::s64(212)).min_height, std::optional<double>(212.0));
    EXPECT_EQ(with_tag("min_height", mvt::flt(0.5f)).min_height, std::optional<double>(0.5));
    // Not a number, or not finite: absent, so the loader skips the feature instead of building NaN.
    EXPECT_FALSE(with_tag("height", mvt::str("15")).height);
    EXPECT_FALSE(with_tag("height", mvt::boolean(true)).height);
    EXPECT_FALSE(with_tag("height", mvt::dbl(std::nan(""))).height);
    EXPECT_FALSE(with_tag("height", mvt::dbl(HUGE_VAL)).height);
    EXPECT_FALSE(with_tag("type", mvt::str("roof")).height);
}

TEST(MapboxMvt, FlagsFromStringsOrBoolsAndTypeFromAString) {
    EXPECT_EQ(with_tag("extrude", mvt::str("true")).extrude, std::optional<bool>(true));
    EXPECT_EQ(with_tag("extrude", mvt::str("false")).extrude, std::optional<bool>(false));
    EXPECT_EQ(with_tag("extrude", mvt::boolean(true)).extrude, std::optional<bool>(true));
    EXPECT_EQ(with_tag("extrude", mvt::boolean(false)).extrude, std::optional<bool>(false));
    EXPECT_EQ(with_tag("underground", mvt::str("true")).underground, std::optional<bool>(true));
    EXPECT_EQ(with_tag("underground", mvt::boolean(false)).underground, std::optional<bool>(false));
    EXPECT_FALSE(with_tag("extrude", mvt::str("yes")).extrude);
    EXPECT_FALSE(with_tag("extrude", mvt::u64(1)).extrude);
    EXPECT_FALSE(with_tag("underground", mvt::str("TRUE")).underground);
    EXPECT_EQ(with_tag("type", mvt::str("roof")).type, "roof");
    EXPECT_EQ(with_tag("type", mvt::u64(3)).type, "");
    EXPECT_FALSE(with_tag("type", mvt::str("roof")).id) << "the feature has no id field";
}

TEST(MapboxMvt, OnlyTheLayerNamedBuildingIsDecoded) {
    mvt::Layer road;
    road.name = "road";
    road.features = {mvt::building(1, {square(0, 0, 10)}, 5.0)};
    mvt::Layer near_miss = road;
    near_miss.name = "buildings";
    const MvtBuildingLayer out = decode(mvt::encode_tile(
        {road, building_layer({mvt::building(2, {square(20, 20, 10)}, 9.0)}), near_miss}));
    ASSERT_TRUE(out.ok);
    ASSERT_EQ(out.buildings.size(), 1u);
    EXPECT_EQ(out.buildings[0].id, std::optional<uint64_t>(2));
    EXPECT_EQ(out.skipped_features, 0u);
}

TEST(MapboxMvt, TileWithoutABuildingLayerIsOkAndEmpty) {
    mvt::Layer water;
    water.name = "water";
    water.features = {mvt::building(1, {square(0, 0, 4096)}, 0.0)};
    const MvtBuildingLayer sea = decode(mvt::encode_tile({water}));
    EXPECT_TRUE(sea.ok);
    EXPECT_TRUE(sea.buildings.empty());
    EXPECT_EQ(sea.skipped_features, 0u);
    const MvtBuildingLayer empty = decode_mvt_building_layer(nullptr, 0);
    EXPECT_TRUE(empty.ok) << "an empty body is a tile with no layers";
    EXPECT_TRUE(empty.buildings.empty());
}

TEST(MapboxMvt, NonPolygonFeaturesAreSkippedAndCounted) {
    mvt::Feature line = mvt::building(1, {square(0, 0, 10)}, 5.0);
    line.geom_type = 2;  // LINESTRING
    mvt::Feature unknown = mvt::building(2, {square(0, 0, 10)}, 5.0);
    unknown.geom_type = 0;  // UNKNOWN
    const MvtBuildingLayer out = decode(mvt::encode_tile(
        {building_layer({line, unknown, mvt::building(3, {square(20, 20, 10)}, 6.0)})}));
    ASSERT_TRUE(out.ok);
    ASSERT_EQ(out.buildings.size(), 1u);
    EXPECT_EQ(out.buildings[0].id, std::optional<uint64_t>(3));
    EXPECT_EQ(out.skipped_features, 2u);
}

TEST(MapboxMvt, HoleBeforeAnyExteriorSkipsTheFeature) {
    const MvtBuildingLayer out = decode(mvt::encode_tile(
        {building_layer({mvt::building(1, {square_hole(30, 30, 40), square(0, 0, 100)}, 20.0),
                         mvt::building(2, {square(0, 0, 10)}, 5.0)})}));
    ASSERT_TRUE(out.ok);
    ASSERT_EQ(out.buildings.size(), 1u);
    EXPECT_EQ(out.buildings[0].id, std::optional<uint64_t>(2));
    EXPECT_EQ(out.skipped_features, 1u);
}

TEST(MapboxMvt, DegenerateRingsAndRepeatedVerticesAreDropped) {
    // The first vertex repeated at the end and (10, 0) written twice: both repeats go.
    const Ring closed = {{0, 0}, {10, 0}, {10, 0}, {10, 10}, {0, 10}, {0, 0}};
    const Ring line = {{0, 0}, {10, 0}, {20, 0}};  // collinear: zero area
    const Ring two = {{5, 5}, {6, 6}};             // fewer than three distinct vertices
    const MvtBuildingLayer out = decode(mvt::encode_tile({building_layer(
        {mvt::building(1, {line, closed, two}, 8.0), mvt::building(2, {two, line}, 8.0)})}));
    ASSERT_TRUE(out.ok);
    ASSERT_EQ(out.buildings.size(), 1u) << "a feature left without a ring is skipped";
    EXPECT_EQ(out.skipped_features, 1u);
    EXPECT_EQ(out.buildings[0].id, std::optional<uint64_t>(1));
    ASSERT_EQ(out.buildings[0].polygons.size(), 1u);
    EXPECT_EQ(as_ring(out.buildings[0].polygons[0].outer), square(0, 0, 10));
    EXPECT_TRUE(out.buildings[0].polygons[0].holes.empty());
}

TEST(MapboxMvt, MalformedFeaturesAreSkippedWhileTheRestDecode) {
    // Every case but the LineTo count is valid apart from one defect: with the check for that
    // defect removed, it would decode.
    const std::vector<uint8_t> good = raw_feature({0, 0}, kSquare10);
    const std::vector<std::vector<uint8_t>> features = {
        good,
        raw_feature({0, 1}, kSquare10),                  // value index out of range
        raw_feature({1, 0}, kSquare10),                  // key index out of range
        raw_feature({0, uint64_t{1} << 32}, kSquare10),  // a value index past 32 bits
        raw_feature({0}, kSquare10),                     // odd tag count
        // An 11-byte varint id, then a valid feature.
        with_tail({0x08, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00}, good),
        // A valid feature, then an id varint that still continues at its tenth byte.
        with_tail(good, {0x08, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80}),
        // A valid feature, then wire type 7.
        with_tail(good, {0x0f}),
        // A valid feature, then a geometry length past the feature's end.
        with_tail(good, {0x22, 0x05, 0x09}),
        // A LineTo count of 2^29 - 1 with one point behind it.
        raw_feature({0, 0}, {cmd(1, 1), zz(0), zz(0), cmd(2, (1u << 29) - 1), zz(1), zz(1)}),
        // LineTo before MoveTo, then a valid ring.
        raw_feature({0, 0}, {cmd(2, 1), zz(1), zz(1), cmd(1, 1), zz(-1), zz(-1), cmd(2, 3), zz(10),
                             zz(0), zz(0), zz(10), zz(-10), zz(0), cmd(7, 1)}),
        // A MoveTo of two points, otherwise a closed square.
        raw_feature({0, 0}, {cmd(1, 2), zz(0), zz(0), zz(10), zz(0), cmd(2, 2), zz(0), zz(10),
                             zz(-10), zz(0), cmd(7, 1)}),
        // A MoveTo while a ring is open, then a valid ring.
        raw_feature({0, 0},
                    with_tail({cmd(1, 1), zz(5), zz(5), cmd(2, 1), zz(1), zz(0)}, kSquare10)),
        // A valid ring, then one that is never closed.
        raw_feature({0, 0}, with_tail(kSquare10, {cmd(1, 1), zz(20), zz(20), cmd(2, 2), zz(9),
                                                  zz(0), zz(0), zz(9)})),
        // ClosePath without a ring, then a valid ring.
        raw_feature({0, 0}, with_tail({cmd(7, 1)}, kSquare10)),
        // A ClosePath count of 2 on an otherwise valid square.
        raw_feature({0, 0}, {cmd(1, 1), zz(0), zz(0), cmd(2, 3), zz(10), zz(0), zz(0), zz(10),
                             zz(-10), zz(0), cmd(7, 2)}),
        // A square whose LineTo has the unknown command id 4.
        raw_feature({0, 0}, {cmd(1, 1), zz(0), zz(0), cmd(4, 3), zz(10), zz(0), zz(0), zz(10),
                             zz(-10), zz(0), cmd(7, 1)}),
        // The cursor leaves int32; wrapped, the ring would be a positive exterior.
        raw_feature({0, 0}, {cmd(1, 1), zz(2147483647), zz(0), cmd(2, 2), zz(1), zz(0), zz(0),
                             zz(-1), cmd(7, 1)}),
        good,
    };
    const MvtBuildingLayer out = decode(raw_tile({raw_building_layer(features)}));
    ASSERT_TRUE(out.ok) << "feature errors never fail the tile";
    ASSERT_EQ(out.buildings.size(), 2u);
    EXPECT_EQ(out.skipped_features, features.size() - 2);
    for (const MvtBuilding& b : out.buildings) EXPECT_EQ(b.height, std::optional<double>(10.0));
}

TEST(MapboxMvt, BrokenFramingFailsTheTile) {
    const std::vector<uint8_t> good = raw_feature({0, 0}, kSquare10);
    // Wire types 3 (a group start) and 7, each followed by 0x00, which a length-delimited read
    // would take as an empty payload: only the wire type is wrong.
    const std::vector<uint8_t> group = with_tail(raw_building_layer({good}), {0x0b, 0x00});
    const std::vector<uint8_t> bad_value = raw_building_layer({good}, {0x0f, 0x00});
    std::vector<uint8_t> unnamed = {0x0f, 0x00};  // before the layer's name
    mvt::detail::put_string(unnamed, 1, "road");
    std::vector<uint8_t> road;
    mvt::detail::put_string(road, 1, "road");
    road.push_back(0x0f);  // wire type 7 after the name: never read
    // A valid building layer, then broken tile framing.
    const std::vector<uint8_t> trailing = with_tail(raw_tile({raw_building_layer({good})}), {0x0f});

    EXPECT_TRUE(decode(raw_tile({raw_building_layer({good})})).ok);
    for (const std::vector<uint8_t>& tile :
         {raw_tile({group}), raw_tile({bad_value}), raw_tile({unnamed, raw_building_layer({good})}),
          bytes("<!DOCTYPE html><html><body>Service unavailable</body></html>"),
          bytes("{\"message\":\"error\"}"), raw_tile({raw_building_layer({good}), unnamed}),
          raw_tile({raw_building_layer({good}), group}), trailing}) {
        const MvtBuildingLayer out = decode(tile);
        EXPECT_FALSE(out.ok) << tile.size() << " bytes";
        EXPECT_TRUE(out.buildings.empty()) << "nothing in a malformed tile is usable";
        EXPECT_EQ(out.skipped_features, 0u) << tile.size() << " bytes";
    }
    const MvtBuildingLayer skipped = decode(raw_tile({road, raw_building_layer({good})}));
    EXPECT_TRUE(skipped.ok) << "other layers are skipped by length";
    EXPECT_EQ(skipped.buildings.size(), 1u);
}

TEST(MapboxMvt, ZeroExtentFailsTheTile) {
    mvt::Layer layer = building_layer({mvt::building(1, {square(0, 0, 10)}, 5.0)});
    layer.extent = 0;
    const MvtBuildingLayer out = decode(mvt::encode_tile({layer}));
    EXPECT_FALSE(out.ok) << "tile units are divided by the extent";
    EXPECT_TRUE(out.buildings.empty());
    std::vector<uint8_t> wide = raw_building_layer({raw_feature({0, 0}, kSquare10)});
    mvt::detail::put_tag(wide, 5, 0);
    mvt::detail::put_varint(wide, uint64_t{1} << 32);
    EXPECT_FALSE(decode(raw_tile({wide})).ok) << "an extent of 2^32 would wrap to 0";
}

TEST(MapboxMvt, Standard3dFilterNeedsExtrudeAndNotUnderground) {
    struct Row {
        std::optional<bool> extrude, underground;
        bool passes;
    };
    const Row rows[] = {
        {true, false, true},          {true, std::nullopt, true},
        {true, true, false},          {false, false, false},
        {false, std::nullopt, false}, {false, true, false},
        {std::nullopt, false, false}, {std::nullopt, std::nullopt, false},
        {std::nullopt, true, false},
    };
    const auto text = [](const std::optional<bool>& v) {
        return v ? (*v ? "true" : "false") : "missing";
    };
    for (const Row& row : rows) {
        MvtBuilding b;
        b.extrude = row.extrude;
        b.underground = row.underground;
        EXPECT_EQ(overlume::mapbox::passes_standard_3d_filter(b), row.passes)
            << "extrude " << text(row.extrude) << ", underground " << text(row.underground);
    }
}

TEST(MapboxMvt, EveryPrefixOfAValidTileDecodesSafely) {
    mvt::Layer water;
    water.name = "water";
    water.features = {mvt::building(1, {square(0, 0, 4096)}, 0.0)};
    const mvt::Layer buildings =
        building_layer({mvt::building(2, {square(0, 0, 100), square_hole(30, 30, 40)}, 20.0),
                        mvt::building(3, {square(200, 200, 10), square(300, 300, 10)}, 30.0, 5.0),
                        mvt::building(4, {square(-33, -33, 50)}, 9.0, 0.0, false)});
    const std::vector<uint8_t> tile = mvt::encode_tile({water, buildings});
    const size_t water_end = mvt::encode_tile({water}).size();
    ASSERT_EQ(decode(tile).buildings.size(), 3u);
    // Each prefix is a view into the full buffer: a reader that ran past `size` would find the
    // real bytes behind the cut and decode buildings from them.
    for (size_t n = 0; n < tile.size(); ++n) {
        const MvtBuildingLayer out = decode_mvt_building_layer(tile.data(), n);
        EXPECT_EQ(out.ok, n == 0 || n == water_end) << "prefix of " << n << " bytes";
        EXPECT_TRUE(out.buildings.empty()) << "prefix of " << n << " bytes";
        EXPECT_EQ(out.skipped_features, 0u) << "prefix of " << n << " bytes";
    }
}

TEST(MapboxMvt, RandomAndMutatedBuffersDecodeSafely) {
    // No sanitizer runs in the gate: this proves no crash and the output invariants on every input.
    const std::vector<uint8_t> valid = mvt::encode_tile({building_layer(
        {mvt::building(2, {square(0, 0, 100), square_hole(30, 30, 40)}, 20.0),
         mvt::building(3, {square(200, 200, 10), square(300, 300, 10)}, 30.0, 5.0)})});
    uint32_t seed = 20261014u;
    const auto next = [&seed] {
        seed = seed * 1664525u + 1013904223u;
        return seed >> 8;
    };
    size_t buildings_seen = 0;
    for (int i = 0; i < 2000; ++i) {
        std::vector<uint8_t> buf;
        if (i % 2 == 0) {  // 0..512 random bytes
            buf.resize(next() % 513);
            for (uint8_t& byte : buf) byte = static_cast<uint8_t>(next());
        } else {  // the valid tile with one to four bytes overwritten
            buf = valid;
            for (uint32_t k = 1 + next() % 4; k > 0; --k)
                buf[next() % buf.size()] = static_cast<uint8_t>(next());
        }
        const MvtBuildingLayer out = decode(buf);
        if (!out.ok) {
            EXPECT_TRUE(out.buildings.empty()) << "buffer " << i;
            EXPECT_EQ(out.skipped_features, 0u) << "buffer " << i;
        }
        EXPECT_GT(out.extent, 0u) << "buffer " << i;
        for (const MvtBuilding& b : out.buildings) {
            ++buildings_seen;
            EXPECT_FALSE(b.polygons.empty()) << "buffer " << i;
            for (const MvtPolygon& polygon : b.polygons) {
                EXPECT_GE(polygon.outer.size(), 3u) << "buffer " << i;
                for (const auto& hole : polygon.holes) EXPECT_GE(hole.size(), 3u) << "buffer " << i;
            }
            if (b.height) EXPECT_TRUE(std::isfinite(*b.height)) << "buffer " << i;
        }
    }
    EXPECT_GT(buildings_seen, 0u) << "the mutations must reach the feature decoder";
}

TEST(MapboxMvt, GzipStoredWritesAValidStoredBlockStream) {
    const std::vector<uint8_t> nine = bytes("123456789");
    const std::vector<uint8_t> gz = mvt::gzip_stored(nine);
    ASSERT_GE(gz.size(), 3u);
    EXPECT_EQ(gz[0], 0x1f);
    EXPECT_EQ(gz[1], 0x8b);
    EXPECT_EQ(gz[2], 0x08);
    std::vector<uint8_t> raw;
    uint32_t crc = 0, isize = 0;
    ASSERT_TRUE(read_gzip_stored(gz, raw, crc, isize));
    EXPECT_EQ(raw, nine);
    EXPECT_EQ(crc, 0xcbf43926u) << "CRC-32's published check value for \"123456789\"";
    EXPECT_EQ(isize, 9u);

    std::vector<uint8_t> big(70000);  // two blocks: 65,535 + 4,465 bytes
    for (size_t i = 0; i < big.size(); ++i) big[i] = static_cast<uint8_t>(i * 7 + 3);
    raw.clear();
    ASSERT_TRUE(read_gzip_stored(mvt::gzip_stored(big), raw, crc, isize));
    EXPECT_EQ(raw, big);
    EXPECT_EQ(isize, 70000u);

    raw.clear();
    ASSERT_TRUE(read_gzip_stored(mvt::gzip_stored({}), raw, crc, isize));
    EXPECT_TRUE(raw.empty());
    EXPECT_EQ(crc, 0u);
    EXPECT_EQ(isize, 0u);
}
```

Which revert each test catches:
- `SquareBuildingKeepsItsIdRingAndFields`: dropping the id, the extent or any of the five tags, or
  a wrong delta decode.
- `CourtyardHoleBelongsToItsExteriorInVersions1And2`: resetting the cursor at each ring, not
  attaching holes, or decoding version 1 layers differently.
- `MultipolygonStartsAPolygonAtEachExterior`: treating every ring after the first as a hole, or
  attaching a later hole to the first polygon.
- `HeightFromEveryNumericValueType`: dropping any numeric type, reading int64 or sint64 wrongly, a
  varint cap below 10 bytes, removing the finiteness check, or reading numbers from strings or bools.
- `FlagsFromStringsOrBoolsAndTypeFromAString`: dropping the string or the bool form, accepting other
  strings or numbers as flags, a type from a non-string, or a default id.
- `OnlyTheLayerNamedBuildingIsDecoded`: decoding every layer, or matching the name by prefix.
- `TileWithoutABuildingLayerIsOkAndEmpty`: `ok = false` for a missing layer or an empty body.
- `NonPolygonFeaturesAreSkippedAndCounted`: removing the type check or the count.
- `HoleBeforeAnyExteriorSkipsTheFeature`: accepting a leading hole, or turning it into an exterior.
- `DegenerateRingsAndRepeatedVerticesAreDropped`: keeping a zero-area ring, a repeated vertex or a
  closing repeat of the first vertex, or keeping a feature left without a ring.
- `MalformedFeaturesAreSkippedWhileTheRestDecode`: removing any check in `decode_tags` (index range,
  odd count), `read_u32` (the 32-bit range), `read_varint` (the 10-byte cap: the loop bound, or the
  failure after ten continuing bytes), `decode_polygons` (command grammar, ClosePath count, cursor
  range, an unclosed last ring) or `decode_feature` (a framing error after valid fields); or failing
  the whole tile on a feature error. Every case but the LineTo count is valid apart from its one
  defect, so no other check rejects it first.
- `BrokenFramingFailsTheTile`: accepting groups or unknown wire types (each is followed by `0x00`,
  which a length-delimited read would take for an empty field), ignoring a broken value, returning
  partial results (in three cases a valid building layer comes before the broken framing), or
  reading a non-building layer past its name.
- `ZeroExtentFailsTheTile`: removing either half of the extent check: zero, or past 32 bits (2^32
  would wrap to 0).
- `Standard3dFilterNeedsExtrudeAndNotUnderground`: any change to the filter, including treating a
  missing `underground` as underground.
- `EveryPrefixOfAValidTileDecodesSafely`: removing the end check in `read_varint` or the length
  check in `next_field` (the reader then runs past `size` into the real bytes behind the cut).
- `RandomAndMutatedBuffersDecodeSafely`: crashes and broken output invariants. No sanitizer runs in
  the gate, so this test proves no crash and the invariants, not the absence of an out-of-bounds
  read; the prefix test above is the deterministic guard for reads past `size`.
- `GzipStoredWritesAValidStoredBlockStream`: the header, NLEN, CRC-32, ISIZE or the 65,535-byte
  block split.

- [ ] **Step 4: Run it and see it fail**

The new test file needs a configure to become a target (the generator is Unix Makefiles, which
knows no target until CMake has globbed it). Configure only while no build runs in this worktree.
Every configure re-runs cesium's ezvcpkg step against the shared vcpkg prefix, and every gate
configures too (`tools/ci_visual_mode.sh:56-57`), so the reconfigure holds the gate lock.

```bash
cd "$WORKTREE"
flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock cmake -S overlume -B overlume/build > /dev/null \
    && echo reconfigured \
    && CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
        --target test_mapbox_mvt
```

No `reconfigured` line and no CMake error above it means the lock wait ran out (exit 75) and nothing
was built: run the step again.

Expected: `reconfigured`, then the test compiles and its link fails with

```
undefined reference to `overlume::mapbox::decode_mvt_building_layer(unsigned char const*, unsigned long)'
undefined reference to `overlume::mapbox::passes_standard_3d_filter(overlume::mapbox::MvtBuilding const&)'
clang++: error: linker command failed with exit code 1 (use -v to see invocation)
```

and the build exits non-zero. A compile error instead means Step 1, 2 or 3 was mistyped.

- [ ] **Step 5: Implement the decoder** `overlume/src/mapbox_mvt.cpp`

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "mapbox_mvt.hpp"

#include <bit>
#include <cmath>
#include <limits>
#include <string_view>
#include <utility>

namespace overlume::mapbox {

namespace {

constexpr uint64_t kPolygon = 3;  // Feature.type POLYGON

// One protobuf field. Vector tiles use wire types 0 (varint), 1 (fixed64), 2 (length-delimited)
// and 5 (fixed32); groups (3, 4) and unknown types make the enclosing message malformed.
struct Field {
    uint64_t number = 0;
    uint32_t wire = 0;
    uint64_t varint = 0;            // wire type 0
    const uint8_t* data = nullptr;  // wire types 1, 2 and 5: the payload
    size_t size = 0;
};

// At most 10 bytes; a 10th byte's bits past bit 63 are dropped, as protobuf does.
bool read_varint(const uint8_t*& p, const uint8_t* end, uint64_t& out) {
    out = 0;
    for (int shift = 0; shift < 64; shift += 7) {
        if (p == end) return false;
        const uint8_t b = *p++;
        out |= static_cast<uint64_t>(b & 0x7f) << shift;
        if (!(b & 0x80)) return true;
    }
    return false;
}

bool read_u32(const uint8_t*& p, const uint8_t* end, uint32_t& out) {
    uint64_t v = 0;
    if (!read_varint(p, end, v) || v > std::numeric_limits<uint32_t>::max()) return false;
    out = static_cast<uint32_t>(v);
    return true;
}

// Reads the field at p and moves p past it. false: broken framing.
bool next_field(const uint8_t*& p, const uint8_t* end, Field& f) {
    uint64_t key = 0;
    if (!read_varint(p, end, key)) return false;
    f.number = key >> 3;
    f.wire = static_cast<uint32_t>(key & 7);
    if (f.wire == 0) return read_varint(p, end, f.varint);
    uint64_t size = 0;
    if (f.wire == 1) {
        size = 8;
    } else if (f.wire == 5) {
        size = 4;
    } else if (f.wire != 2 || !read_varint(p, end, size)) {
        return false;
    }
    if (size > static_cast<uint64_t>(end - p)) return false;
    f.data = p;
    f.size = static_cast<size_t>(size);
    p += f.size;
    return true;
}

uint64_t fixed_le(const Field& f) {
    uint64_t v = 0;
    for (size_t i = 0; i < f.size; ++i) v |= static_cast<uint64_t>(f.data[i]) << (8 * i);
    return v;
}

int64_t zigzag64(uint64_t v) { return static_cast<int64_t>((v >> 1) ^ (~(v & 1) + 1)); }

int32_t zigzag32(uint32_t v) { return static_cast<int32_t>((v >> 1) ^ (~(v & 1) + 1)); }

// A layer's Value message. It should hold one field; when it holds several, each is kept.
struct TagValue {
    std::optional<double> number;  // float, double, int64, uint64 or sint64
    std::optional<bool> boolean;
    std::optional<std::string> text;
};

bool decode_value(const Field& value, TagValue& out) {
    const uint8_t* const end = value.data + value.size;
    Field f;
    for (const uint8_t* p = value.data; p < end;) {
        if (!next_field(p, end, f)) return false;
        if (f.number == 1 && f.wire == 2) {
            out.text.emplace(reinterpret_cast<const char*>(f.data), f.size);
        } else if (f.number == 2 && f.wire == 5) {
            out.number = std::bit_cast<float>(static_cast<uint32_t>(fixed_le(f)));
        } else if (f.number == 3 && f.wire == 1) {
            out.number = std::bit_cast<double>(fixed_le(f));
        } else if (f.number == 4 && f.wire == 0) {
            out.number = static_cast<double>(static_cast<int64_t>(f.varint));
        } else if (f.number == 5 && f.wire == 0) {
            out.number = static_cast<double>(f.varint);
        } else if (f.number == 6 && f.wire == 0) {
            out.number = static_cast<double>(zigzag64(f.varint));
        } else if (f.number == 7 && f.wire == 0) {
            out.boolean = f.varint != 0;
        }
    }
    return true;
}

// A NaN or infinite height would reach the mesh; it counts as absent.
std::optional<double> finite(const std::optional<double>& n) {
    return n && std::isfinite(*n) ? n : std::nullopt;
}

// streets-v8 writes `extrude` and `underground` as the strings "true"/"false"; a bool also counts.
std::optional<bool> flag(const TagValue& v) {
    if (v.boolean) return v.boolean;
    if (v.text == "true") return true;
    if (v.text == "false") return false;
    return std::nullopt;
}

// Packed (key index, value index) pairs. false: an odd count or an index out of range.
bool decode_tags(const Field& tags, const std::vector<std::string>& keys,
                 const std::vector<TagValue>& values, MvtBuilding& b) {
    const uint8_t* p = tags.data;
    const uint8_t* const end = tags.data + tags.size;
    while (p < end) {
        uint32_t k = 0, v = 0;
        if (!read_u32(p, end, k) || !read_u32(p, end, v)) return false;
        if (k >= keys.size() || v >= values.size()) return false;
        const std::string& key = keys[k];
        const TagValue& value = values[v];
        if (key == "height") {
            b.height = finite(value.number);
        } else if (key == "min_height") {
            b.min_height = finite(value.number);
        } else if (key == "extrude") {
            b.extrude = flag(value);
        } else if (key == "underground") {
            b.underground = flag(value);
        } else if (key == "type" && value.text) {
            b.type = *value.text;
        }
    }
    return true;
}

// Drops a closing repeat of the first vertex, then classifies the ring by its surveyor's-formula
// area in tile coordinates (y down): positive starts a polygon, negative is a hole of the last
// one, zero (which covers fewer than three distinct vertices) drops the ring. false: a hole came
// before any exterior ring.
bool close_ring(std::vector<MvtPoint>& ring, std::vector<MvtPolygon>& polygons) {
    while (ring.size() > 1 && ring.back().x == ring.front().x && ring.back().y == ring.front().y)
        ring.pop_back();
    double twice_area = 0.0;
    for (size_t i = 0; i < ring.size(); ++i) {
        const MvtPoint& a = ring[i];
        const MvtPoint& b = ring[(i + 1) % ring.size()];
        twice_area += static_cast<double>(a.x) * b.y - static_cast<double>(b.x) * a.y;
    }
    if (twice_area > 0.0) {
        polygons.push_back({std::move(ring), {}});
    } else if (twice_area < 0.0) {
        if (polygons.empty()) return false;
        polygons.back().holes.push_back(std::move(ring));
    }
    return true;
}

// Polygon geometry (MVT 2.1 section 4.3): per ring a MoveTo of one point, a LineTo and a
// ClosePath, as zigzag deltas from a cursor that starts at (0, 0) and persists across rings.
// Repeated vertices are dropped. false: malformed.
bool decode_polygons(const Field& geometry, std::vector<MvtPolygon>& polygons) {
    constexpr uint32_t kMoveTo = 1, kLineTo = 2, kClosePath = 7;
    constexpr int64_t kMin = std::numeric_limits<int32_t>::min();
    constexpr int64_t kMax = std::numeric_limits<int32_t>::max();
    const uint8_t* p = geometry.data;
    const uint8_t* const end = geometry.data + geometry.size;
    int64_t x = 0, y = 0;
    std::vector<MvtPoint> ring;
    bool open = false;
    while (p < end) {
        uint32_t command = 0;
        if (!read_u32(p, end, command)) return false;
        const uint32_t id = command & 7, count = command >> 3;
        if (id == kClosePath) {
            if (count != 1 || !open || !close_ring(ring, polygons)) return false;
            open = false;
            continue;
        }
        if (id == kMoveTo && count == 1 && !open) {
            ring.clear();
            open = true;
        } else if (id != kLineTo || !open) {
            return false;
        }
        // Nothing is reserved from `count`: a forged count runs out of bytes first.
        for (uint32_t i = 0; i < count; ++i) {
            uint32_t dx = 0, dy = 0;
            if (!read_u32(p, end, dx) || !read_u32(p, end, dy)) return false;
            x += zigzag32(dx);
            y += zigzag32(dy);
            if (x < kMin || x > kMax || y < kMin || y > kMax) return false;
            const MvtPoint pt{static_cast<int32_t>(x), static_cast<int32_t>(y)};
            if (ring.empty() || pt.x != ring.back().x || pt.y != ring.back().y) ring.push_back(pt);
        }
    }
    return !open;
}

// One feature. false: not a polygon, malformed, or left without a ring; the caller counts it.
bool decode_feature(const Field& feature, const std::vector<std::string>& keys,
                    const std::vector<TagValue>& values, MvtBuilding& b) {
    const uint8_t* const end = feature.data + feature.size;
    Field f, tags, geometry;
    uint64_t type = 0;  // UNKNOWN
    for (const uint8_t* p = feature.data; p < end;) {
        if (!next_field(p, end, f)) return false;
        if (f.number == 1 && f.wire == 0) {
            b.id = f.varint;
        } else if (f.number == 2 && f.wire == 2) {
            tags = f;
        } else if (f.number == 3 && f.wire == 0) {
            type = f.varint;
        } else if (f.number == 4 && f.wire == 2) {
            geometry = f;
        }
    }
    return type == kPolygon && decode_tags(tags, keys, values, b) &&
           decode_polygons(geometry, b.polygons) && !b.polygons.empty();
}

// nullopt: the layer's framing breaks before its name, so it may be the building layer.
std::optional<bool> named_building(const Field& layer) {
    const uint8_t* const end = layer.data + layer.size;
    Field f;
    for (const uint8_t* p = layer.data; p < end;) {
        if (!next_field(p, end, f)) return std::nullopt;
        if (f.number == 1 && f.wire == 2)
            return std::string_view(reinterpret_cast<const char*>(f.data), f.size) == "building";
    }
    return false;
}

// false: the layer's framing or extent is broken, which leaves nothing in the tile usable.
bool decode_building_layer(const Field& layer, MvtBuildingLayer& out) {
    const uint8_t* const end = layer.data + layer.size;
    std::vector<std::string> keys;
    std::vector<TagValue> values;
    Field f;
    // The key and value tables may follow the features, so a first pass reads them; it also
    // checks the framing of every field. The version (field 15) is not read: version 1 layers
    // decode by the version 2 rules (streets-v8 tiles are version 2).
    for (const uint8_t* p = layer.data; p < end;) {
        if (!next_field(p, end, f)) return false;
        if (f.number == 3 && f.wire == 2) {
            keys.emplace_back(reinterpret_cast<const char*>(f.data), f.size);
        } else if (f.number == 4 && f.wire == 2) {
            if (!decode_value(f, values.emplace_back())) return false;
        } else if (f.number == 5 && f.wire == 0) {
            // Tile units are divided by the extent downstream.
            if (f.varint == 0 || f.varint > std::numeric_limits<uint32_t>::max()) return false;
            out.extent = static_cast<uint32_t>(f.varint);
        }
    }
    for (const uint8_t* p = layer.data; p < end;) {
        if (!next_field(p, end, f)) return false;
        if (f.number != 2 || f.wire != 2) continue;
        MvtBuilding b;
        if (decode_feature(f, keys, values, b)) {
            out.buildings.push_back(std::move(b));
        } else {
            ++out.skipped_features;
        }
    }
    return true;
}

}

MvtBuildingLayer decode_mvt_building_layer(const uint8_t* data, size_t size) {
    MvtBuildingLayer out;
    const uint8_t* const end = data + size;
    Field layer;
    for (const uint8_t* p = data; p < end;) {
        if (!next_field(p, end, layer)) return {};
        if (layer.number != 3 || layer.wire != 2) continue;  // Tile.layers
        const std::optional<bool> building = named_building(layer);
        if (!building || (*building && !decode_building_layer(layer, out))) return {};
    }
    out.ok = true;
    return out;
}

bool passes_standard_3d_filter(const MvtBuilding& b) {
    return b.extrude.value_or(false) && !b.underground.value_or(false);
}

}
```

- [ ] **Step 6: Run it and see it pass**

The configure picks `mapbox_mvt.cpp` up through Task 0's `src/mapbox_*.cpp` glob. It holds the
gate lock, as in Step 4.

```bash
cd "$WORKTREE"
flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock cmake -S overlume -B overlume/build > /dev/null \
    && echo reconfigured \
    && CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
        --target test_mapbox_mvt \
    && overlume/build/test_mapbox_mvt
```

No `reconfigured` line and no CMake error above it means the lock wait ran out (exit 75) and nothing
was built: run the step again.

Expected: `reconfigured`, a clean build, and a run that ends

```
[==========] 17 tests from 1 test suite ran. (… ms total)
[  PASSED  ] 17 tests.
```

- [ ] **Step 7: Format**

```bash
cd "$WORKTREE"
git add overlume/src/mapbox_mvt.hpp overlume/src/mapbox_mvt.cpp \
    overlume/tests/mapbox_mvt_fixture.hpp overlume/tests/test_mapbox_mvt.cpp
tools/check_format.sh --fix overlume/src/mapbox_mvt.hpp overlume/src/mapbox_mvt.cpp \
    overlume/tests/mapbox_mvt_fixture.hpp overlume/tests/test_mapbox_mvt.cpp
git diff --stat
git add overlume/src/mapbox_mvt.hpp overlume/src/mapbox_mvt.cpp \
    overlume/tests/mapbox_mvt_fixture.hpp overlume/tests/test_mapbox_mvt.cpp
tools/check_format.sh && tools/check_spdx.sh
```

Expected: `check_format.sh: reformatted 4 files with clang-format 23.1.1`; `git diff --stat` prints
nothing (the code above is already clang-format 23.1.1's output, so a diff means a line was typed
differently, and the second `git add` stages the fix); then
`check_format.sh: PASS (200 files, clang-format 23.1.1)` and
`check_spdx.sh: PASS (356 files carry the SPDX identifier)` (196 and 352 at the lane's fork, plus
these four).

- [ ] **Step 8: Revert check (do not commit)**: the prefix test must catch a read past `size`

```bash
cd "$WORKTREE"
sed -i '/if (size > static_cast<uint64_t>(end - p)) return false;/d' overlume/src/mapbox_mvt.cpp
git diff --stat overlume/src/mapbox_mvt.cpp
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 --target test_mapbox_mvt
overlume/build/test_mapbox_mvt --gtest_filter='MapboxMvt.EveryPrefixOfAValidTileDecodesSafely' \
    | tail -4
git checkout -- overlume/src/mapbox_mvt.cpp && git diff --stat overlume/src/mapbox_mvt.cpp
```

Expected: the first diff reports `1 file changed, 1 deletion(-)`; the filtered run ends

```
[  FAILED  ] 1 test, listed below:
[  FAILED  ] MapboxMvt.EveryPrefixOfAValidTileDecodesSafely

 1 FAILED TEST
```

because, without the check, a layer cut by the prefix is read to its declared length, through the
real bytes behind the cut, and its buildings decode. After the checkout the last diff prints
nothing. Run only this test while the check is gone: elsewhere the same reader would read past the
end of a vector.

- [ ] **Step 9: Gate** (foreground, Bash timeout 600000 ms)

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
ROS_DOMAIN_ID=81 CI_VISUAL_MODE_DOMAIN_ID=81 \
    flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock tools/ci_visual_mode.sh
```

Expected (the baseline, which Task 0 leaves unchanged, plus the 17 `MapboxMvt.*` tests):
- `PASS  POD header check`
- `PASS  library ctest suite  (100% tests passed, 0 tests failed out of 364 (348 ok / 6 skipped))`
- `PASS  node gtests  (Summary: 363 tests, 0 errors, 0 failures, 0 skipped)`
- `PASS  WS bridge pytest suite  (82 passed in …)`, the time varying
- `PASS  golden suite: 23 ok, 0 skipped`
- `PASS  examples  (6 run, all exited 0 with non-empty output)`
- `OVERALL: PASS`, exit 0.

Exit 75 means another lane held the lock for 300 s: run the same command again. Any other non-zero
exit is a gate failure. A WS bridge failure can also come from another session's gate on the same
ports, so rerun before treating it as a finding. The prebuild also rebuilds `test_mapbox_mvt` from
the source Step 8 restored.

- [ ] **Step 10: Commit**

```bash
cd "$WORKTREE"
git add overlume/src/mapbox_mvt.hpp overlume/src/mapbox_mvt.cpp \
    overlume/tests/mapbox_mvt_fixture.hpp overlume/tests/test_mapbox_mvt.cpp
git status --short
git commit -F - <<'EOF'
feat(mapbox): decode the streets-v8 building layer from vector tiles

The mapbox:// source extrudes footprints from Mapbox Vector Tiles, and
neither cesium-native nor the vcpkg prefix has an MVT or protobuf
decoder. decode_mvt_building_layer reads only the "building" layer with
a bounds-checked reader (varints of at most 10 bytes, framing checked on
every read, nothing reserved from a count on the wire): ids, polygons
classified by the MVT v2 winding rule with holes kept on their exterior,
and height, min_height, extrude, underground and type from any value
encoding. A malformed or ringless feature is skipped and counted; broken
tile or layer framing, or a zero extent, fails the whole tile.
passes_standard_3d_filter is Mapbox Standard's rule: extrude, and not
underground.

tests/mapbox_mvt_fixture.hpp synthesizes tiles and stored-block gzip in
the test process, so no Mapbox bytes enter the tree. MapboxMvt covers
the decoder and the filter, every prefix of a valid tile, and 2,000
random and mutated buffers.

Claude-Session: <the executing session's URL>
EOF
```

Expected: `git status --short` lists exactly the four files, each as `A `; the commit succeeds.

### Task 2: `mapbox://` URI parser

**Lane:** 1. **Depends on:** Task 0; runs after Task 1 in this lane (the file counts below assume
Task 1's commit).

**Files:**
- Create: `overlume/src/mapbox_loader.hpp` (exactly the contract)
- Create: `overlume/src/mapbox_loader.cpp` (only `parse_mapbox_spec`, namespace `overlume::mapbox`;
  Task 13 appends the loader below it)
- Test: `overlume/tests/test_mapbox_uri.cpp` (suite `MapboxUri`, 10 tests)

**Interfaces:**
- Consumes: nothing but the standard library.
- Produces, for Task 13 (verbatim from the contract): `inline constexpr double kDefaultRadiusM =
  500.0;`, `struct MapboxSpec`, `struct MapboxSpecResult` and
  `MapboxSpecResult parse_mapbox_spec(const std::string& spec);` in `overlume::mapbox`.
  - `error`, when refused, is one line that starts `mapbox://` and names the layer list or the key;
    it echoes only text from the URI.
  - `warning` is set on every parse that meets an unknown `materials` value. The parser is pure, so
    the spec's "warns once" is the caller's job: Task 13 logs it at most once per source, through
    the redacting logger.
  - An empty `fallback_dir` means no fallback, which is what the stream source already does with
    an empty directory (`overlume/src/environment_stream.cpp:785-787`).
  - `MapboxSpec` has no cache field: `cache=off` is accepted and changes nothing, and the source is
    always built with the cache off.

**API notes:**
- `parse_ion_spec` (`overlume/src/environment_stream.cpp:71-176`), whose grammar this mirrors: the
  split at the first `?` (:72), pairs on `&` (:87), the key at the first `=` (:89), `brightness`
  (:142-151) and `radius` (:160-169) as a fully consumed `std::stod` that is finite and > 0, and an
  unknown `materials` meaning clay with a warning (:102-114).
- `std::stod` (`<string>`): throws `std::invalid_argument` with no digits and `std::out_of_range` on
  overflow; both are caught as `std::exception`. It parses `inf` and `nan`, which `std::isfinite`
  then refuses.
- `compose_environment_source_uri`
  (`ros/src/overlume_ros/include/overlume_ros/environment_source_uri.hpp:32-36`) appends
  `fallback=<environment_chunks_dir>` only to a URI that does not already contain `fallback=`, so an
  explicit empty `fallback=` also suppresses the node's own fallback.

- [ ] **Step 1: Write the contract header** `overlume/src/mapbox_loader.hpp`

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <optional>
#include <string>

namespace overlume::mapbox {

inline constexpr double kDefaultRadiusM = 500.0;

struct MapboxSpec {
    bool materials_original = false;
    double radius_m = kDefaultRadiusM;
    double brightness = 1.0;
    std::string fallback_dir;
};

struct MapboxSpecResult {
    std::optional<MapboxSpec> spec;  // nullopt: the URI is refused
    std::string error;               // when refused: one line naming the layer list or the key
    std::string warning;             // when accepted with a caveat (an unknown `materials` value)
};

// Parses the text after "mapbox://": the layer list (only "buildings"), then an optional query
// split at the first '?', pairs on '&', the key at the first '='. Unknown keys and pairs without
// '=' are ignored, the last value of a repeated key wins, and a bad value for a known key refuses
// the whole URI. Pure: no logging, no environment access.
MapboxSpecResult parse_mapbox_spec(const std::string& spec);

}
```

- [ ] **Step 2: Write the failing test** `overlume/tests/test_mapbox_uri.cpp`

The empty `fallback=` decision: it is accepted and means no fallback, as an absent key does, and as
the last value it clears an earlier one. Refusing it would leave no way to switch the node's
automatic fallback off for one URI.

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "mapbox_loader.hpp"

#include <gtest/gtest.h>

#include <string>

namespace {

using overlume::mapbox::MapboxSpecResult;
using overlume::mapbox::parse_mapbox_spec;

bool mentions(const std::string& text, const std::string& part) {
    return text.find(part) != std::string::npos;
}

}

TEST(MapboxUri, DefaultsWithOrWithoutATrailingQuestionMark) {
    for (const char* spec : {"buildings", "buildings?"}) {
        const MapboxSpecResult r = parse_mapbox_spec(spec);
        ASSERT_TRUE(r.spec) << spec << ": " << r.error;
        EXPECT_FALSE(r.spec->materials_original) << spec;
        EXPECT_EQ(r.spec->radius_m, 500.0) << spec;
        EXPECT_EQ(r.spec->radius_m, overlume::mapbox::kDefaultRadiusM) << spec;
        EXPECT_EQ(r.spec->brightness, 1.0) << spec;
        EXPECT_EQ(r.spec->fallback_dir, "") << spec;
        EXPECT_EQ(r.error, "") << spec;
        EXPECT_EQ(r.warning, "") << spec;
    }
}

TEST(MapboxUri, EachKeySetsItsField) {
    const MapboxSpecResult r = parse_mapbox_spec(
        "buildings?materials=original&radius=250.5&brightness=1.75&fallback=/srv/overlume/chunks");
    ASSERT_TRUE(r.spec) << r.error;
    EXPECT_TRUE(r.spec->materials_original);
    EXPECT_EQ(r.spec->radius_m, 250.5);
    EXPECT_EQ(r.spec->brightness, 1.75);
    EXPECT_EQ(r.spec->fallback_dir, "/srv/overlume/chunks");
    EXPECT_EQ(r.warning, "");
    const MapboxSpecResult clay = parse_mapbox_spec("buildings?materials=clay");
    ASSERT_TRUE(clay.spec) << clay.error;
    EXPECT_FALSE(clay.spec->materials_original);
    EXPECT_EQ(clay.warning, "");
}

TEST(MapboxUri, BadNumbersRefuseTheUriNamingTheKey) {
    const char* const cases[][2] = {
        {"buildings?radius=0", "radius"},
        {"buildings?radius=-5", "radius"},
        {"buildings?radius=abc", "radius"},
        {"buildings?radius=12m", "radius"},
        {"buildings?radius=inf", "radius"},
        {"buildings?radius=nan", "radius"},
        {"buildings?radius=", "radius"},
        {"buildings?brightness=0", "brightness"},
        {"buildings?brightness=-1", "brightness"},
        {"buildings?brightness=dim", "brightness"},
        {"buildings?brightness=1.5x", "brightness"},
        {"buildings?brightness=", "brightness"},
    };
    for (const auto& [spec, key] : cases) {
        const MapboxSpecResult r = parse_mapbox_spec(spec);
        EXPECT_FALSE(r.spec) << spec;
        EXPECT_TRUE(mentions(r.error, key)) << spec << ": " << r.error;
    }
}

TEST(MapboxUri, CacheAcceptsOnlyOff) {
    const MapboxSpecResult off = parse_mapbox_spec("buildings?cache=off");
    EXPECT_TRUE(off.spec) << off.error;
    for (const char* spec : {"buildings?cache=/var/cache/overlume",
                             "buildings?cache=", "buildings?cache=on", "buildings?cache=OFF"}) {
        const MapboxSpecResult r = parse_mapbox_spec(spec);
        EXPECT_FALSE(r.spec) << spec;
        EXPECT_TRUE(mentions(r.error, "cache")) << spec << ": " << r.error;
    }
}

TEST(MapboxUri, LayerListMustBeExactlyBuildings) {
    for (const char* spec : {"", "terrain", "buildings,terrain", "building", "Buildings",
                             "buildings/", "?radius=5", "terrain?radius=5"}) {
        const MapboxSpecResult r = parse_mapbox_spec(spec);
        EXPECT_FALSE(r.spec) << "'" << spec << "'";
        // Quoted, so an echo of "buildings,terrain" or "buildings/" does not count.
        EXPECT_TRUE(mentions(r.error, "'buildings'")) << "'" << spec << "': " << r.error;
    }
}

TEST(MapboxUri, IonOnlyAndUnknownKeysAreIgnored) {
    // Values parse_ion_spec would refuse: ignoring a key must not validate it.
    const MapboxSpecResult r = parse_mapbox_spec(
        "buildings?follow_terrain=maybe&ground_bias=x&replaces_ground=2&max_tilt_deg=-1"
        "&max_cache_items=many&zoom=17&=5");
    ASSERT_TRUE(r.spec) << r.error;
    EXPECT_EQ(r.spec->radius_m, 500.0);
    EXPECT_EQ(r.spec->brightness, 1.0);
    EXPECT_FALSE(r.spec->materials_original);
    EXPECT_EQ(r.spec->fallback_dir, "");
    EXPECT_EQ(r.warning, "");
}

TEST(MapboxUri, LastValueOfARepeatedKeyWins) {
    const MapboxSpecResult r = parse_mapbox_spec(
        "buildings?radius=100&brightness=2&radius=250&materials=original&materials=clay"
        "&fallback=/srv/a&fallback=/srv/b");
    ASSERT_TRUE(r.spec) << r.error;
    EXPECT_EQ(r.spec->radius_m, 250.0);
    EXPECT_EQ(r.spec->brightness, 2.0);
    EXPECT_FALSE(r.spec->materials_original);
    EXPECT_EQ(r.spec->fallback_dir, "/srv/b");
    EXPECT_FALSE(parse_mapbox_spec("buildings?radius=0&radius=250").spec)
        << "a bad value refuses the URI even when a later one is good";
}

TEST(MapboxUri, PairsWithoutEqualsAreIgnored) {
    const MapboxSpecResult r = parse_mapbox_spec("buildings?radius&materials&brightness=2&&cache");
    ASSERT_TRUE(r.spec) << r.error;
    EXPECT_EQ(r.spec->radius_m, 500.0);
    EXPECT_FALSE(r.spec->materials_original);
    EXPECT_EQ(r.spec->brightness, 2.0);
    EXPECT_EQ(r.warning, "");
}

TEST(MapboxUri, UnknownMaterialsWarnsAndMeansClay) {
    const MapboxSpecResult r = parse_mapbox_spec("buildings?materials=bogus");
    ASSERT_TRUE(r.spec) << r.error;
    EXPECT_FALSE(r.spec->materials_original);
    EXPECT_TRUE(mentions(r.warning, "materials")) << r.warning;
    EXPECT_TRUE(mentions(r.warning, "bogus")) << r.warning;
    EXPECT_EQ(r.error, "");
    const MapboxSpecResult last = parse_mapbox_spec("buildings?materials=original&materials=bogus");
    ASSERT_TRUE(last.spec) << last.error;
    EXPECT_FALSE(last.spec->materials_original) << "the unknown value, given last, means clay";
}

TEST(MapboxUri, EmptyFallbackMeansNoFallback) {
    const MapboxSpecResult alone = parse_mapbox_spec("buildings?fallback=");
    ASSERT_TRUE(alone.spec) << alone.error;
    EXPECT_EQ(alone.spec->fallback_dir, "");
    const MapboxSpecResult cleared =
        parse_mapbox_spec("buildings?fallback=/srv/overlume/chunks&fallback=");
    ASSERT_TRUE(cleared.spec) << cleared.error;
    EXPECT_EQ(cleared.spec->fallback_dir, "") << "the empty value, given last, clears it";
}
```

Which revert each test catches:
- `DefaultsWithOrWithoutATrailingQuestionMark`: a changed default, or refusing `buildings?`.
- `EachKeySetsItsField`: any key not assigned to its field.
- `BadNumbersRefuseTheUriNamingTheKey`: dropping the full-consumption check (`12m`, `1.5x`), the
  finiteness check (`inf`, `nan`) or `> 0`; an error that does not name the key.
- `CacheAcceptsOnlyOff`: refusing `off`, or accepting any other value (empty and `OFF` included).
- `LayerListMustBeExactlyBuildings`: a prefix, case-insensitive or list match; an error that does
  not name `'buildings'`.
- `IonOnlyAndUnknownKeysAreIgnored`: validating, or refusing on, an ion-only key.
- `LastValueOfARepeatedKeyWins`: the first value winning, or forgiving an earlier bad value.
- `PairsWithoutEqualsAreIgnored`: treating a bare key as a value or an error.
- `UnknownMaterialsWarnsAndMeansClay`: refusing the URI, no warning, or keeping an earlier
  `original`.
- `EmptyFallbackMeansNoFallback`: refusing `fallback=`, or not clearing an earlier value.

- [ ] **Step 3: Run it and see it fail**

The reconfigure makes the test a target and holds the gate lock, as in Task 1's Step 4.

```bash
cd "$WORKTREE"
flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock cmake -S overlume -B overlume/build > /dev/null \
    && echo reconfigured \
    && CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
        --target test_mapbox_uri
```

No `reconfigured` line and no CMake error above it means the lock wait ran out (exit 75) and nothing
was built: run the step again.

Expected: `reconfigured`, then the test compiles and its link fails with

```
undefined reference to `overlume::mapbox::parse_mapbox_spec(std::__1::basic_string<char, std::__1::char_traits<char>, std::__1::allocator<char> > const&)'
clang++: error: linker command failed with exit code 1 (use -v to see invocation)
```

and the build exits non-zero.

- [ ] **Step 4: Implement the parser** `overlume/src/mapbox_loader.cpp`

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "mapbox_loader.hpp"

#include <cmath>
#include <exception>

namespace overlume::mapbox {

namespace {

// Fully consumed, finite and > 0, as parse_ion_spec reads `radius` and `brightness`.
bool parse_positive(const std::string& val, double& out) {
    try {
        size_t used = 0;
        const double v = std::stod(val, &used);
        if (used != val.size() || !std::isfinite(v) || v <= 0.0) return false;
        out = v;
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

}

MapboxSpecResult parse_mapbox_spec(const std::string& spec) {
    const size_t q = spec.find('?');
    const std::string layers = spec.substr(0, q);
    if (layers != "buildings") {
        return {std::nullopt,
                "mapbox:// supports only the layer list 'buildings', not '" + layers + "'", ""};
    }
    MapboxSpec out;
    std::string warning;
    const std::string rest = q == std::string::npos ? std::string() : spec.substr(q + 1);
    size_t pos = 0;
    while (pos < rest.size()) {
        const size_t amp = rest.find('&', pos);
        const std::string kv = rest.substr(pos, amp == std::string::npos ? amp : amp - pos);
        const size_t eq = kv.find('=');
        if (eq != std::string::npos) {
            const std::string key = kv.substr(0, eq);
            const std::string val = kv.substr(eq + 1);
            // Every other key, the ion-only ones included, is ignored.
            if (key == "materials") {
                out.materials_original = val == "original";
                if (val != "original" && val != "clay")
                    warning = "mapbox:// materials='" + val + "' is unknown, using clay";
            } else if (key == "radius" || key == "brightness") {
                if (!parse_positive(val, key == "radius" ? out.radius_m : out.brightness))
                    return {std::nullopt,
                            "mapbox:// " + key + "='" + val + "' is not a finite number > 0", ""};
            } else if (key == "fallback") {
                // Empty means no fallback, as when absent. It also keeps the node's composer from
                // appending its own, since that skips any URI already naming `fallback=`.
                out.fallback_dir = val;
            } else if (key == "cache" && val != "off") {
                return {std::nullopt,
                        "mapbox:// cache='" + val +
                            "' refused: Mapbox tiles are never written to disk (only cache=off)",
                        ""};
            }
        }
        if (amp == std::string::npos) break;
        pos = amp + 1;
    }
    return {out, "", warning};
}

}
```

- [ ] **Step 5: Run it and see it pass**

The reconfigure picks `mapbox_loader.cpp` up through Task 0's glob and holds the gate lock.

```bash
cd "$WORKTREE"
flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock cmake -S overlume -B overlume/build > /dev/null \
    && echo reconfigured \
    && CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
        --target test_mapbox_uri \
    && overlume/build/test_mapbox_uri
```

No `reconfigured` line and no CMake error above it means the lock wait ran out (exit 75) and nothing
was built: run the step again.

Expected: `reconfigured`, a clean build, and a run that ends

```
[==========] 10 tests from 1 test suite ran. (… ms total)
[  PASSED  ] 10 tests.
```

- [ ] **Step 6: Format**

```bash
cd "$WORKTREE"
git add overlume/src/mapbox_loader.hpp overlume/src/mapbox_loader.cpp \
    overlume/tests/test_mapbox_uri.cpp
tools/check_format.sh --fix overlume/src/mapbox_loader.hpp overlume/src/mapbox_loader.cpp \
    overlume/tests/test_mapbox_uri.cpp
git diff --stat
git add overlume/src/mapbox_loader.hpp overlume/src/mapbox_loader.cpp \
    overlume/tests/test_mapbox_uri.cpp
tools/check_format.sh && tools/check_spdx.sh
```

Expected: `check_format.sh: reformatted 3 files with clang-format 23.1.1`; `git diff --stat` prints
nothing; then `check_format.sh: PASS (203 files, clang-format 23.1.1)` and
`check_spdx.sh: PASS (359 files carry the SPDX identifier)`.

- [ ] **Step 7: Revert check (do not commit)**: full consumption must be guarded

```bash
cd "$WORKTREE"
sed -i 's/used != val.size() || //' overlume/src/mapbox_loader.cpp
git diff --stat overlume/src/mapbox_loader.cpp
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 --target test_mapbox_uri
overlume/build/test_mapbox_uri --gtest_filter='MapboxUri.BadNumbersRefuseTheUriNamingTheKey' \
    | tail -4
git checkout -- overlume/src/mapbox_loader.cpp && git diff --stat overlume/src/mapbox_loader.cpp
```

Expected: the first diff reports `1 file changed, 1 insertion(+), 1 deletion(-)`; the filtered run
ends

```
[  FAILED  ] 1 test, listed below:
[  FAILED  ] MapboxUri.BadNumbersRefuseTheUriNamingTheKey

 1 FAILED TEST
```

(`radius=12m` and `brightness=1.5x` are then accepted as 12 and 1.5); after the checkout the last
diff prints nothing.

- [ ] **Step 8: Gate** (foreground, Bash timeout 600000 ms)

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
ROS_DOMAIN_ID=81 CI_VISUAL_MODE_DOMAIN_ID=81 \
    flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock tools/ci_visual_mode.sh
```

Expected (Task 1's gate plus the 10 `MapboxUri.*` tests):
- `PASS  POD header check`
- `PASS  library ctest suite  (100% tests passed, 0 tests failed out of 374 (358 ok / 6 skipped))`
- `PASS  node gtests  (Summary: 363 tests, 0 errors, 0 failures, 0 skipped)`
- `PASS  WS bridge pytest suite  (82 passed in …)`, the time varying
- `PASS  golden suite: 23 ok, 0 skipped`
- `PASS  examples  (6 run, all exited 0 with non-empty output)`
- `OVERALL: PASS`, exit 0.

Exit 75 means another lane held the lock for 300 s: run the same command again. Any other non-zero
exit is a gate failure. A WS bridge failure can also come from another session's gate on the same
ports, so rerun before treating it as a finding.

- [ ] **Step 9: Commit**

```bash
cd "$WORKTREE"
git add overlume/src/mapbox_loader.hpp overlume/src/mapbox_loader.cpp \
    overlume/tests/test_mapbox_uri.cpp
git status --short
git commit -F - <<'EOF'
feat(mapbox): parse mapbox:// environment URIs

parse_mapbox_spec reads the text after "mapbox://" the way
parse_ion_spec reads ion:// specs: split at the first '?', pairs on
'&', the key at the first '=', the last value wins and unknown keys are
ignored. It stays pure: a refusal or the unknown-materials caveat comes
back as text for the caller's redacting logger. Only the layer list
"buildings" opens; radius (default 500 m) and brightness must be finite
and > 0; cache accepts only "off", because Mapbox tiles are never
written to disk; the ion-only keys are ignored like unknown ones. An
empty fallback= means no fallback, which also stops the node from
appending its own. The loader joins mapbox_loader.cpp in a later task.

Claude-Session: <the executing session's URL>
EOF
```

Expected: `git status --short` lists exactly the three files, each as `A `; the commit succeeds.

---

### Task 3: Extrusion builder

**Lane:** 2 (worktree `$INTEGRATION-l2`, branch `mapbox-env/l2`, ROS domain 82).
**Depends on:** Task 0 (`src/mapbox_*.cpp` compile as C++20 in `overlume_stream`;
`tests/test_mapbox_*.cpp` are cesium-only test executables).

**Files:**
- Create: `overlume/src/mapbox_extrude.hpp` (verbatim from the Interface contract)
- Create: `overlume/src/mapbox_extrude.cpp` (C++20, `overlume_stream`, includes
  `<mapbox/earcut.hpp>`)
- Test: `overlume/tests/test_mapbox_extrude.cpp` (suite `MapboxExtrude`, C++17, includes only
  `mapbox_extrude.hpp` from `src/`)
- Modify: nothing.

**Interfaces:**
- Consumes:
  - `template <typename N = uint32_t, typename Polygon> std::vector<N> mapbox::earcut(const Polygon& poly);`
    from `mapbox/earcut.hpp`, with `Polygon = std::vector<std::vector<std::array<double, 2>>>`.
  - Task 0's wiring (above). Nothing from another lane.
- Produces (consumed by Task 13, which passes a `ProjectFn` wrapping Task 4's
  `project_to_frame(frame, z16_tile, u, v, extent, z_m)`, and by Task 14's conflation):

```cpp
namespace overlume::mapbox {
using TileRing = std::vector<std::array<double, 2>>;
struct TilePolygon { TileRing outer; std::vector<TileRing> holes; };
using ProjectFn = std::function<std::array<double, 3>(double u, double v, double z_m)>;
struct TriangleMesh { std::vector<float> positions; std::vector<float> normals;
                      std::vector<uint32_t> indices; };
struct ExtrudeStats { uint32_t polygons_in = 0; uint32_t polygons_built = 0;
                      uint32_t wall_quads = 0; uint32_t roof_triangles = 0; };
ExtrudeStats extrude_polygons(const std::vector<TilePolygon>& polygons, double clip_extent,
                              double min_height_m, double height_m, const ProjectFn& project,
                              TriangleMesh& out);
std::optional<std::array<double, 2>> footprint_centroid(const std::vector<TilePolygon>& polygons);
}
```

**API notes:**
- `mapbox/earcut.hpp:810-815` (earcut-hpp 2.2.4 in cesium's vcpkg prefix,
  `installed/x64-linux-clang-libcxx/include`): `mapbox::earcut<N>(poly)`. Ring 0 is the
  exterior, the rest are holes.
- `mapbox/earcut.hpp:14-21`: points are read through `mapbox::util::nth<I, Point>::get`, which is
  `std::get<I>(t)`, so `std::array<double, 2>` needs no adapter.
- `mapbox/earcut.hpp:191-226, 426-438`: rings are linked in input order with `vertices += len`, so
  the returned indices address the concatenation exterior, hole 1, hole 2, and so on.
  `linkedList` re-winds every ring itself, so the roof does not depend on our re-winding.
- `mapbox/earcut.hpp:255-307, 310-328, 673-677`: an ear is cut only when
  `area(prev, ear, next) < 0`, where `area = (q.y - p.y)(r.x - q.x) - (q.x - p.x)(r.y - q.y)`,
  and is emitted as (prev, ear, next). Ears are therefore counter-clockwise with y up.
- `mapbox/earcut.hpp:372-397`: `cureLocalIntersections` emits (a, p, b) with no orientation test.
  It is the only source of clockwise triangles, and the `twisted` test ring reaches it.
- `mapbox/earcut.hpp:472-488`: a hole vertex on an exterior edge bridges straight to it
  (`if (x == hx) return m;`). The clip-edge hole test builds that case.
- Name lookup: inside `namespace overlume::mapbox`, `mapbox` finds `overlume::mapbox` first.
  The call is therefore spelled `::mapbox::earcut`, or it fails to compile.
- `overlume/CMakeLists.txt:256-269` (pre-Task 0 numbering; after Task 0 every `src/mapbox_*.cpp`):
  - `overlume_stream` compiles at `-std=gnu++20`, includes `src`, and its objects go into the
    static `overlume`.
  - The cesium targets it links carry the vcpkg include directory as `-isystem`
    (`overlume/build/CMakeFiles/overlume_stream.dir/flags.make`).
  - cesium-native wires `earcut` as a header-only INTERFACE target: its `CMakeLists.txt:131, 370`
    and `CesiumGeospatial/CMakeLists.txt:44`.
- `overlume/CMakeLists.txt:581-637`: each `tests/*.cpp` becomes `overlume/build/<name>`, built at
  `-std=gnu++17` with `-I overlume/src`, linked to the static `overlume`, and discovered into
  ctest under the `cpu` label.
- Downstream (Task 13 hands this mesh to the stream):
  - `overlume/src/environment_stream.cpp:231-275` rewrites POSITION heights and leaves NORMAL
    alone.
  - `overlume/src/gltf_normals.cpp:328` skips primitives that already carry NORMAL.
  - So these per-face normals reach the GPU unchanged.
- `overlume/cmake/overlume_exports.map:1-4`: the shared library exports `overlume::*` only.
  Earcut's instantiations stay local, and the helpers below live in an anonymous namespace.
- Verification: earcut 2.2.4's non-hashed path (fewer than 80 vertices, which covers every test
  ring) was ported line for line to Python together with the algorithm below.
  - Every expectation of Step 2's other 15 tests holds on that port, and each of their reverts
    makes at least one test fail there. The corner test was checked on the C++ only.
  - Inputs and arithmetic are exact IEEE doubles, so the C++ follows the same path.
  - The C++ of Steps 1, 2 and 4 was also built outside the repository, with the build's own
    compiler wrapper and flags (`-O3 -DNDEBUG`, gnu++20 for the source, gnu++17 for the test,
    clean at `-Wall -Wextra`), earcut 2.2.4 from the vcpkg prefix and the built gtest 1.15.2. All
    16 tests pass. Each revert in Step 2's table, applied to that C++, fails the test the table
    names, and the corner test's three reverts pass the other 15 tests.
  - The code blocks are already in clang-format 23.1.1's layout, so Step 6 changes nothing.

- [ ] **Step 1: Create the header** `overlume/src/mapbox_extrude.hpp`: the Interface contract's
  header, verbatim.

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

namespace overlume::mapbox {

// A ring in tile units (x right, y down). Open: the first vertex is not repeated.
using TileRing = std::vector<std::array<double, 2>>;

struct TilePolygon {
    TileRing outer;
    std::vector<TileRing> holes;
};

// Maps a point in the polygon's tile units, at z_m metres above the anchor's ground, to output
// metres (x east, y north, z up). Called for every ring vertex left after clipping, once at each
// height; walls and roof share the results.
using ProjectFn = std::function<std::array<double, 3>(double u, double v, double z_m)>;

// Triangles with per-face normals: faces of different orientation never share a vertex.
struct TriangleMesh {
    std::vector<float> positions;   // x, y, z per vertex, output metres
    std::vector<float> normals;     // unit length, x, y, z per vertex
    std::vector<uint32_t> indices;  // counter-clockwise seen from outside
};

struct ExtrudeStats {
    uint32_t polygons_in = 0;
    uint32_t polygons_built = 0;  // non-empty after clipping
    uint32_t wall_quads = 0;
    uint32_t roof_triangles = 0;
};

// Clips each polygon to the square [0, clip_extent]², then extrudes it from min_height_m to
// height_m: a roof at height_m (earcut, holes included) and walls between the two heights. Edges
// that lie on the clip square get no wall; there is no floor. Rings are re-oriented in output
// space (exterior counter-clockwise, holes clockwise) before walls and roofs are built. Appends to
// `out`. Requires finite heights with height_m > min_height_m; returns zero stats otherwise. A
// polygon with a non-finite coordinate is skipped.
ExtrudeStats extrude_polygons(const std::vector<TilePolygon>& polygons, double clip_extent,
                              double min_height_m, double height_m, const ProjectFn& project,
                              TriangleMesh& out);

// Area-weighted centroid of the exterior rings (tile units), before clipping; exteriors with a
// non-finite coordinate are left out. nullopt when the total area is zero.
std::optional<std::array<double, 2>> footprint_centroid(const std::vector<TilePolygon>& polygons);

}
```

- [ ] **Step 2: Write the failing test** `overlume/tests/test_mapbox_extrude.cpp`.
  - Footprints sit at u, v ≥ 1000, so no edge lies on the clip square unless a test means it to.
  - The projection is linear and mirrors v, as the real chain does.

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "mapbox_extrude.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

using overlume::mapbox::ExtrudeStats;
using overlume::mapbox::TilePolygon;
using overlume::mapbox::TileRing;
using overlume::mapbox::TriangleMesh;
using overlume::mapbox::extrude_polygons;
using overlume::mapbox::footprint_centroid;

namespace {

constexpr double kExtent = 4096.0;
constexpr double kMetresPerUnit = 0.25;
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// A linear stand-in for the tile-frame chain. Like the real one it mirrors v: tile y runs south,
// output y north.
std::array<double, 3> project(double u, double v, double z_m) {
    return {u * kMetresPerUnit, -v * kMetresPerUnit, z_m};
}

ExtrudeStats extrude(const std::vector<TilePolygon>& polygons, double min_height_m, double height_m,
                     TriangleMesh& out) {
    return extrude_polygons(polygons, kExtent, min_height_m, height_m, project, out);
}

TilePolygon polygon(TileRing outer, std::vector<TileRing> holes = {}) {
    return {std::move(outer), std::move(holes)};
}

// The MVT exterior winding: positive area in tile units, so clockwise once mirrored into output
// space.
TileRing box(double u0, double v0, double u1, double v1) {
    return {{u0, v0}, {u1, v0}, {u1, v1}, {u0, v1}};
}

TileRing reversed(TileRing ring) {
    std::reverse(ring.begin(), ring.end());
    return ring;
}

struct V3 {
    double x = 0.0, y = 0.0, z = 0.0;
};

V3 operator-(const V3& a, const V3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }

double dot(const V3& a, const V3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

V3 cross(const V3& a, const V3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

V3 position(const TriangleMesh& m, size_t i) {
    return {m.positions[3 * i], m.positions[3 * i + 1], m.positions[3 * i + 2]};
}

V3 normal(const TriangleMesh& m, size_t i) {
    return {m.normals[3 * i], m.normals[3 * i + 1], m.normals[3 * i + 2]};
}

struct Triangle {
    std::array<uint32_t, 3> v{};
    std::array<V3, 3> p{};
    V3 n;  // the first vertex's normal

    bool wall() const { return std::abs(n.z) < 0.5; }
    V3 centre() const {
        return {(p[0].x + p[1].x + p[2].x) / 3.0, (p[0].y + p[1].y + p[2].y) / 3.0,
                (p[0].z + p[1].z + p[2].z) / 3.0};
    }
    // Points to the side the triangle is counter-clockwise from.
    V3 winding() const { return cross(p[1] - p[0], p[2] - p[0]); }
    double area_from_above() const { return 0.5 * std::abs(winding().z); }
};

std::vector<Triangle> triangles(const TriangleMesh& m) {
    std::vector<Triangle> out;
    for (size_t k = 0; k + 2 < m.indices.size(); k += 3) {
        Triangle t;
        for (size_t c = 0; c < 3; ++c) {
            t.v[c] = m.indices[k + c];
            t.p[c] = position(m, t.v[c]);
        }
        t.n = normal(m, t.v[0]);
        out.push_back(t);
    }
    return out;
}

double roof_area(const TriangleMesh& m) {
    double area = 0.0;
    for (const Triangle& t : triangles(m)) {
        if (!t.wall()) area += t.area_from_above();
    }
    return area;
}

// Indices in range, unit normals shared by each face's three vertices, and every triangle
// counter-clockwise seen from the side its normal points to.
void expect_well_formed(const TriangleMesh& m) {
    ASSERT_EQ(m.positions.size(), m.normals.size());
    ASSERT_EQ(m.positions.size() % 3, 0u);
    ASSERT_EQ(m.indices.size() % 3, 0u);
    const size_t vertices = m.positions.size() / 3;
    for (uint32_t i : m.indices) ASSERT_LT(i, vertices);
    for (size_t i = 0; i < vertices; ++i) {
        const V3 n = normal(m, i);
        EXPECT_NEAR(std::sqrt(dot(n, n)), 1.0, 1e-6) << "vertex " << i;
    }
    for (const Triangle& t : triangles(m)) {
        for (size_t c = 1; c < 3; ++c) {
            const V3 n = normal(m, t.v[c]);
            EXPECT_TRUE(n.x == t.n.x && n.y == t.n.y && n.z == t.n.z)
                << "a face with mixed normals";
        }
        EXPECT_GT(dot(t.winding(), t.n), 0.0) << "a face clockwise seen from outside";
    }
}

}

TEST(MapboxExtrude, BoxGivesFourWallQuadsAndTwoRoofTriangles) {
    TriangleMesh m;
    const ExtrudeStats s = extrude({polygon(box(1000, 1000, 1100, 1100))}, 0.0, 10.0, m);
    EXPECT_EQ(s.polygons_in, 1u);
    EXPECT_EQ(s.polygons_built, 1u);
    EXPECT_EQ(s.wall_quads, 4u);
    EXPECT_EQ(s.roof_triangles, 2u);
    // Walls: 4 vertices and 6 indices per quad. Roof: the 4 corners and 2 triangles.
    EXPECT_EQ(m.positions.size(), (16u + 4u) * 3u);
    EXPECT_EQ(m.normals.size(), (16u + 4u) * 3u);
    EXPECT_EQ(m.indices.size(), 24u + 6u);
    expect_well_formed(m);
}

TEST(MapboxExtrude, WallsRunFromMinHeightToHeightAndThereIsNoFloor) {
    // A building standing on the ground, then a part floating from 20 m to 30 m.
    for (const auto& [min_m, height_m] : {std::pair{0.0, 10.0}, std::pair{20.0, 30.0}}) {
        TriangleMesh m;
        extrude({polygon(box(1000, 1000, 1100, 1100))}, min_m, height_m, m);
        size_t at_min = 0, at_top = 0;
        for (size_t i = 0; i < m.positions.size() / 3; ++i) {
            const V3 p = position(m, i);
            const V3 n = normal(m, i);
            if (std::abs(n.z) < 0.5) {
                EXPECT_TRUE(p.z == min_m || p.z == height_m) << "a wall vertex at " << p.z;
                if (p.z == min_m) ++at_min;
                if (p.z == height_m) ++at_top;
            } else {
                EXPECT_EQ(n.z, 1.0) << "a face that is neither wall nor roof";
                EXPECT_EQ(p.z, height_m);
            }
        }
        EXPECT_EQ(at_min, 8u);  // two per wall quad
        EXPECT_EQ(at_top, 8u);
    }
}

TEST(MapboxExtrude, EitherInputWindingGivesOutwardWallsAndAnUpwardRoof) {
    // box() comes out clockwise in output space and its reverse counter-clockwise: without
    // re-orientation, one of the two would have its walls facing in.
    const V3 centre{1050 * kMetresPerUnit, -1050 * kMetresPerUnit, 5.0};
    for (const TileRing& ring :
         {box(1000, 1000, 1100, 1100), reversed(box(1000, 1000, 1100, 1100))}) {
        TriangleMesh m;
        extrude({polygon(ring)}, 0.0, 10.0, m);
        expect_well_formed(m);
        size_t walls = 0, roofs = 0;
        for (const Triangle& t : triangles(m)) {
            if (t.wall()) {
                ++walls;
                EXPECT_EQ(t.n.z, 0.0);
                EXPECT_GT(dot(t.n, t.centre() - centre), 0.0) << "a wall facing in";
            } else {
                ++roofs;
                EXPECT_EQ(t.n.x, 0.0);
                EXPECT_EQ(t.n.y, 0.0);
                EXPECT_EQ(t.n.z, 1.0);
                EXPECT_GT(t.winding().z, 0.0) << "a roof triangle clockwise from above";
            }
        }
        EXPECT_EQ(walls, 8u);
        EXPECT_EQ(roofs, 2u);
    }
}

TEST(MapboxExtrude, CourtyardWallsFaceAwayFromTheSolidAndTheRoofLeavesTheHoleOpen) {
    // The hole in the MVT winding (against the exterior's), then reversed.
    const TileRing hole = reversed(box(1025, 1025, 1075, 1075));
    const V3 centre{1050 * kMetresPerUnit, -1050 * kMetresPerUnit, 5.0};
    for (const TileRing& h : {hole, reversed(hole)}) {
        TriangleMesh m;
        const ExtrudeStats s = extrude({polygon(box(1000, 1000, 1100, 1100), {h})}, 0.0, 10.0, m);
        EXPECT_EQ(s.wall_quads, 8u);
        expect_well_formed(m);
        size_t outer = 0, courtyard = 0;
        for (const Triangle& t : triangles(m)) {
            if (!t.wall()) continue;
            // The wall plane's signed distance from the centre, along the wall's normal.
            const double along = dot(t.n, t.centre() - centre);
            if (std::abs(std::abs(along) - 12.5) < 1e-3) {
                ++outer;
                EXPECT_GT(along, 0.0) << "an outer wall facing in";
            } else {
                ++courtyard;
                EXPECT_NEAR(along, -6.25, 1e-3) << "a courtyard wall facing the solid";
            }
        }
        EXPECT_EQ(outer, 8u);
        EXPECT_EQ(courtyard, 8u);
        // (100^2 - 50^2) units^2 at 0.25 m per unit.
        EXPECT_NEAR(roof_area(m), 468.75, 468.75e-6);
    }
}

TEST(MapboxExtrude, PolygonOutsideTheTileGivesNothing) {
    TriangleMesh m;
    const ExtrudeStats s = extrude(
        {polygon(box(-300, 1000, -200, 1100)), polygon(box(5000, 5000, 5100, 5100))}, 0.0, 10.0, m);
    EXPECT_EQ(s.polygons_in, 2u);
    EXPECT_EQ(s.polygons_built, 0u);
    EXPECT_EQ(s.wall_quads, 0u);
    EXPECT_EQ(s.roof_triangles, 0u);
    EXPECT_TRUE(m.positions.empty());
    EXPECT_TRUE(m.normals.empty());
    EXPECT_TRUE(m.indices.empty());
}

TEST(MapboxExtrude, NoWallOnTheClipEdgeSoNeighbouringTilesMeetWithoutSeamWalls) {
    // One 79 x 100-unit building seen from this tile and from its west neighbour, where u is
    // larger by the extent. u0 = -60 puts an unsnapped clip crossing 7e-15 off u == 0.
    TriangleMesh here, west;
    const ExtrudeStats a = extrude({polygon(box(-60, 1000, 19, 1100))}, 0.0, 10.0, here);
    const ExtrudeStats b =
        extrude({polygon(box(kExtent - 60, 1000, kExtent + 19, 1100))}, 0.0, 10.0, west);
    EXPECT_EQ(a.wall_quads, 3u);  // east, north and south
    EXPECT_EQ(b.wall_quads, 3u);  // west, north and south
    EXPECT_EQ(here.positions.size(), (12u + 4u) * 3u);
    EXPECT_EQ(here.indices.size(), 18u + 6u);
    expect_well_formed(here);
    expect_well_formed(west);
    for (const Triangle& t : triangles(here)) {
        if (t.wall()) {
            EXPECT_FALSE(t.p[0].x == 0.0 && t.p[1].x == 0.0 && t.p[2].x == 0.0);
        }
    }
    const double seam_x = kExtent * kMetresPerUnit;
    for (const Triangle& t : triangles(west)) {
        if (t.wall()) {
            EXPECT_FALSE(t.p[0].x == seam_x && t.p[1].x == seam_x && t.p[2].x == seam_x);
        }
    }
    // The two halves cover the footprint once: 19 and 60 units of its 79-unit width.
    EXPECT_NEAR(roof_area(here), 118.75, 118.75e-6);
    EXPECT_NEAR(roof_area(west), 375.0, 375e-6);
}

TEST(MapboxExtrude, CornerCrossingsGetWallsOnlyOnTheirInTileEdges) {
    // Over the north-west corner (u == 0, v == 0) and the south-east one (u == E, v == E).
    const double e = kExtent * kMetresPerUnit;
    const std::array<std::array<double, 2>, 2> seams = {{{0.0, 0.0}, {e, -e}}};  // output x, y
    const std::array<TileRing, 2> rings = {
        box(-60, -60, 19, 19), box(kExtent - 19, kExtent - 19, kExtent + 60, kExtent + 60)};
    for (size_t c = 0; c < 2; ++c) {
        TriangleMesh m;
        const ExtrudeStats s = extrude({polygon(rings[c])}, 0.0, 10.0, m);
        EXPECT_EQ(s.wall_quads, 2u);
        EXPECT_EQ(s.roof_triangles, 2u);
        expect_well_formed(m);
        for (const Triangle& t : triangles(m)) {
            if (!t.wall()) continue;
            for (size_t a = 0; a < 2; ++a) {
                const auto on = [&](const V3& p) { return (a == 0 ? p.x : p.y) == seams[c][a]; };
                EXPECT_FALSE(on(t.p[0]) && on(t.p[1]) && on(t.p[2])) << "a wall on a clip line";
            }
        }
    }
    // A cut across the corner: its ends lie on two different clip lines, so it keeps its wall.
    TriangleMesh m;
    const ExtrudeStats cut = extrude({polygon({{-10, 30}, {30, -10}, {-10, -10}})}, 0.0, 10.0, m);
    EXPECT_EQ(cut.wall_quads, 1u);
    EXPECT_EQ(cut.roof_triangles, 1u);
    expect_well_formed(m);
}

TEST(MapboxExtrude, HoleCrossingTheClipEdgeIsCutWithItsExterior) {
    TriangleMesh m;
    const ExtrudeStats s = extrude(
        {polygon(box(-40, 1010, 60, 1090), {reversed(box(-20, 1030, 20, 1070))})}, 0.0, 10.0, m);
    EXPECT_EQ(s.polygons_built, 1u);
    EXPECT_EQ(s.wall_quads, 6u);  // 3 outer and 3 courtyard walls, none on u == 0
    expect_well_formed(m);
    // (60 x 80 - 20 x 40) units^2 at 0.25 m per unit.
    EXPECT_NEAR(roof_area(m), 250.0, 250e-6);
}

TEST(MapboxExtrude, DegenerateRingsAndRepeatedVerticesAddNoFaces) {
    const TileRing line = {{1040, 1050}, {1060, 1050}, {1050, 1050}};
    TriangleMesh m;
    const ExtrudeStats s = extrude({polygon(line), polygon({{1000, 1000}, {1100, 1100}}),
                                    polygon(box(1000, 1000, 1100, 1100), {line})},
                                   0.0, 10.0, m);
    EXPECT_EQ(s.polygons_in, 3u);
    EXPECT_EQ(s.polygons_built, 1u);  // the collinear and the two-vertex exteriors are dropped,
    EXPECT_EQ(s.wall_quads, 4u);      // and so is the collinear hole
    EXPECT_EQ(s.roof_triangles, 2u);

    // A repeated vertex is a zero-length edge: no wall and no NaN normal.
    TriangleMesh r;
    const ExtrudeStats d =
        extrude({polygon({{1000, 1000}, {1100, 1000}, {1100, 1000}, {1100, 1100}, {1000, 1100}})},
                0.0, 10.0, r);
    EXPECT_EQ(d.wall_quads, 4u);
    EXPECT_EQ(d.roof_triangles, 2u);
    expect_well_formed(r);
}

TEST(MapboxExtrude, SelfTouchingAndSelfCrossingRingsGiveAWellFormedMesh) {
    // Two squares meeting at one corner, walked as one ring that visits the corner twice.
    const TileRing pinched = {{1000, 1000}, {1050, 1000}, {1050, 1050}, {1100, 1050},
                              {1100, 1100}, {1050, 1100}, {1050, 1050}, {1000, 1050}};
    // A twisted corner: the first and third edges cross at (1010, 1010). Earcut 2.2.4 cuts it in
    // its self-intersection pass, which emits a clockwise triangle that the roof must flip.
    const TileRing twisted = {{1020, 1000}, {1000, 1020}, {1020, 1020},
                              {1000, 1000}, {1060, 1080}, {1000, 1080}};
    for (const TileRing& ring : {pinched, twisted}) {
        TriangleMesh m;
        const ExtrudeStats s = extrude({polygon(ring)}, 0.0, 10.0, m);
        EXPECT_EQ(s.polygons_built, 1u);
        EXPECT_GT(s.roof_triangles, 0u);
        expect_well_formed(m);
    }
}

TEST(MapboxExtrude, NothingUnlessHeightsAreFiniteAndHeightIsAboveMinHeight) {
    const std::vector<std::pair<double, double>> cases = {{10.0, 10.0},  {12.0, 10.0}, {0.0, kInf},
                                                          {-kInf, 10.0}, {kNaN, 10.0}, {0.0, kNaN}};
    for (const auto& [min_m, height_m] : cases) {
        TriangleMesh m;
        const ExtrudeStats s = extrude({polygon(box(1000, 1000, 1100, 1100))}, min_m, height_m, m);
        EXPECT_EQ(s.polygons_in, 0u) << min_m << " " << height_m;
        EXPECT_EQ(s.polygons_built, 0u);
        EXPECT_EQ(s.wall_quads, 0u);
        EXPECT_EQ(s.roof_triangles, 0u);
        EXPECT_TRUE(m.positions.empty() && m.normals.empty() && m.indices.empty());
    }
}

TEST(MapboxExtrude, NonFiniteCoordinateSkipsItsPolygon) {
    TileRing bad_outer = box(1000, 1000, 1100, 1100);
    bad_outer[2][0] = kNaN;
    TileRing bad_hole = reversed(box(1225, 1025, 1275, 1075));
    bad_hole[1][1] = kInf;
    TriangleMesh m;
    const ExtrudeStats s =
        extrude({polygon(bad_outer), polygon(box(1200, 1000, 1300, 1100), {bad_hole}),
                 polygon(box(1400, 1000, 1500, 1100))},
                0.0, 10.0, m);
    EXPECT_EQ(s.polygons_in, 3u);
    EXPECT_EQ(s.polygons_built, 1u);
    EXPECT_EQ(s.wall_quads, 4u);
    EXPECT_EQ(s.roof_triangles, 2u);
    expect_well_formed(m);
    for (float f : m.positions) EXPECT_TRUE(std::isfinite(f));
}

TEST(MapboxExtrude, AppendsToTheMeshItIsGiven) {
    TriangleMesh m;
    extrude({polygon(box(1000, 1000, 1100, 1100))}, 0.0, 10.0, m);
    const TriangleMesh first = m;
    extrude({polygon(box(2000, 1000, 2100, 1100))}, 0.0, 10.0, m);
    ASSERT_EQ(m.positions.size(), 2 * first.positions.size());
    ASSERT_EQ(m.indices.size(), 2 * first.indices.size());
    EXPECT_TRUE(std::equal(first.positions.begin(), first.positions.end(), m.positions.begin()));
    EXPECT_TRUE(std::equal(first.indices.begin(), first.indices.end(), m.indices.begin()));
    // The same box 1000 units east: the same topology, offset past the first box's vertices.
    const uint32_t offset = static_cast<uint32_t>(first.positions.size() / 3);
    for (size_t k = 0; k < first.indices.size(); ++k) {
        EXPECT_EQ(m.indices[first.indices.size() + k], first.indices[k] + offset);
    }
    expect_well_formed(m);
}

TEST(MapboxExtrude, StatsCountPolygonsWallQuadsAndRoofTriangles) {
    TriangleMesh m;
    const ExtrudeStats s =
        extrude({polygon(box(1000, 1000, 1100, 1100)), polygon(box(-300, 1000, -200, 1100)),
                 polygon(box(-60, 2000, 19, 2100)),
                 polygon(box(3000, 3000, 3100, 3100), {reversed(box(3025, 3025, 3075, 3075))})},
                0.0, 10.0, m);
    EXPECT_EQ(s.polygons_in, 4u);
    EXPECT_EQ(s.polygons_built, 3u);  // not the one outside the tile
    EXPECT_EQ(s.wall_quads, 4u + 3u + 8u);
    EXPECT_EQ(s.roof_triangles, 2u + 2u + 8u);
    EXPECT_EQ(m.indices.size(), 3u * (2u * s.wall_quads + s.roof_triangles));
    // Roof vertices: the box's 4, the cut box's 4, the courtyard's 4 + 4.
    EXPECT_EQ(m.positions.size(), 3u * (4u * s.wall_quads + 4u + 4u + 8u));
}

TEST(MapboxExtrude, FootprintCentroidWeighsUnclippedExteriorsByArea) {
    const auto expect_at = [](const std::vector<TilePolygon>& polygons, double u, double v) {
        const auto c = footprint_centroid(polygons);
        ASSERT_TRUE(c.has_value());
        EXPECT_NEAR((*c)[0], u, 1e-9);
        EXPECT_NEAR((*c)[1], v, 1e-9);
    };
    expect_at({polygon(box(1000, 1000, 1100, 1100))}, 1050.0, 1050.0);
    // An L: a 20 x 10 bar and a 10 x 10 foot, in either winding.
    const TileRing ell = {{0, 0}, {20, 0}, {20, 10}, {10, 10}, {10, 20}, {0, 20}};
    expect_at({polygon(ell)}, 25.0 / 3.0, 25.0 / 3.0);
    expect_at({polygon(reversed(ell))}, 25.0 / 3.0, 25.0 / 3.0);
    // Two parts of opposite winding weighing 100 and 400; the hole does not count.
    expect_at({polygon(box(0, 0, 10, 10), {reversed(box(2, 2, 8, 8))}),
               polygon(reversed(box(20, 0, 40, 20)))},
              25.0, 9.0);
    // Not clipped to the tile.
    expect_at({polygon(box(-50, 1000, 50, 1100))}, 0.0, 1050.0);
    // A part with a non-finite coordinate is left out.
    TileRing bad = box(1200, 1000, 1300, 1100);
    bad[0][1] = kNaN;
    expect_at({polygon(box(1000, 1000, 1100, 1100)), polygon(bad)}, 1050.0, 1050.0);
}

TEST(MapboxExtrude, FootprintCentroidOfNoAreaIsNullopt) {
    EXPECT_FALSE(footprint_centroid({}).has_value());
    EXPECT_FALSE(footprint_centroid({polygon({})}).has_value());
    EXPECT_FALSE(footprint_centroid({polygon({{0, 0}, {10, 0}, {20, 0}})}).has_value());
    EXPECT_FALSE(footprint_centroid({polygon({{0, 0}, {10, 10}})}).has_value());
}
```

  The revert each test catches (each one checked on the compiled C++, see Verification above):

  | Test | Fails when this is reverted |
  |------|-----------------------------|
  | `BoxGivesFourWallQuadsAndTwoRoofTriangles` | a floor or a missing roof (counts); normals left unnormalised; one vertex shared between faces |
  | `WallsRunFromMinHeightToHeightAndThereIsNoFloor` | walls built from 0 m instead of `min_height_m`; a floor (a vertex with normal z = −1) |
  | `EitherInputWindingGivesOutwardWallsAndAnUpwardRoof` | re-winding exteriors in output space (with the MVT winding, `box()`'s walls face in) |
  | `CourtyardWallsFaceAwayFromTheSolidAndTheRoofLeavesTheHoleOpen` | re-winding holes (one hole winding faces the solid); holes left out of earcut (roof area 625) |
  | `PolygonOutsideTheTileGivesNothing` | the clip |
  | `NoWallOnTheClipEdgeSoNeighbouringTilesMeetWithoutSeamWalls` | the clip-line wall skip (4 + 4 walls, 2 doubled at the seam); the exact snap of crossings onto the line (the −60 crossing lands at 7.1e-15 and the cut gets a wall). The brief's −50..50 box lands on 0 even unsnapped, so it could not catch the snap |
  | `CornerCrossingsGetWallsOnlyOnTheirInTileEdges` | the v-line half of the clip-line skip (a wall on v == 0 or v == E); skipping an edge whose ends lie on any two clip lines (the corner cut loses its wall); re-winding `base` and `top` without `uv` (walls land on the clip lines). None of the three fails another test: those cross only u == 0 or u == E, and in each of their clipped 4-gons the seam edge is one that the mis-winding leaves in place |
  | `HoleCrossingTheClipEdgeIsCutWithItsExterior` | clipping holes (7 walls, wrong roof area) |
  | `DegenerateRingsAndRepeatedVerticesAddNoFaces` | the zero-area ring drop (12 walls); the zero-length-edge guard (a NaN normal) |
  | `SelfTouchingAndSelfCrossingRingsGiveAWellFormedMesh` | the roof's clockwise-triangle flip (`twisted`) |
  | `NothingUnlessHeightsAreFiniteAndHeightIsAboveMinHeight` | either half of the height guard |
  | `NonFiniteCoordinateSkipsItsPolygon` | the coordinate finiteness check |
  | `AppendsToTheMeshItIsGiven` | clearing `out`, or not offsetting indices by its vertex count |
  | `StatsCountPolygonsWallQuadsAndRoofTriangles` | counting `polygons_built` before the clip |
  | `FootprintCentroidWeighsUnclippedExteriorsByArea` | signed weights (gives (38.33, 11.67)); counting holes; clipping; non-finite parts |
  | `FootprintCentroidOfNoAreaIsNullopt` | the zero-total check (NaN instead of nullopt) |

- [ ] **Step 3: Run it and see it fail** (the header exists, nothing defines its functions yet)

```bash
cd "$WORKTREE"
flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock cmake -S overlume -B overlume/build > /dev/null \
    && echo reconfigured \
    && CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
        --target test_mapbox_extrude
```

No `reconfigured` line and no CMake error above it means the lock wait ran out (exit 75) and nothing
was built: run the step again.

Expected:
- `reconfigured`.
- The test compiles, then its link fails with
  ``undefined reference to `overlume::mapbox::extrude_polygons(`` and
  ``undefined reference to `overlume::mapbox::footprint_centroid(``.
- The build exits non-zero.

- [ ] **Step 4: Implement** `overlume/src/mapbox_extrude.cpp`

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "mapbox_extrude.hpp"

#include <mapbox/earcut.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>

namespace overlume::mapbox {

namespace {

using Point2 = std::array<double, 2>;
using Point3 = std::array<double, 3>;

// Below this area (tile units^2) a clipped ring is a sliver with no surface to build.
constexpr double kMinRingArea = 1e-9;

// Shoelace area over the first two coordinates: positive when counter-clockwise with y up.
template <typename Point>
double signed_area(const std::vector<Point>& ring) {
    double sum = 0.0;
    for (size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++) {
        sum += ring[j][0] * ring[i][1] - ring[i][0] * ring[j][1];
    }
    return 0.5 * sum;
}

bool all_finite(const TileRing& ring) {
    return std::all_of(ring.begin(), ring.end(),
                       [](const Point2& p) { return std::isfinite(p[0]) && std::isfinite(p[1]); });
}

// One Sutherland-Hodgman pass: keeps the side of the line p[axis] == bound at or below it
// (keep_below) or at or above it. Crossings are put exactly on the line, so an edge along the clip
// square is found by exact comparison.
TileRing clip_to_line(const TileRing& ring, size_t axis, double bound, bool keep_below) {
    const auto inside = [&](const Point2& p) {
        return keep_below ? p[axis] <= bound : p[axis] >= bound;
    };
    TileRing out;
    out.reserve(ring.size() + 2);
    for (size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++) {
        const Point2& prev = ring[j];
        const Point2& cur = ring[i];
        if (inside(cur) != inside(prev)) {
            const double t = (bound - prev[axis]) / (cur[axis] - prev[axis]);
            Point2 hit{};
            hit[axis] = bound;
            hit[1 - axis] = prev[1 - axis] + t * (cur[1 - axis] - prev[1 - axis]);
            out.push_back(hit);
        }
        if (inside(cur)) out.push_back(cur);
    }
    return out;
}

// The ring clipped to [0, e]^2, or empty when no area is left (which drops rings of fewer than
// three vertices too).
TileRing clip_ring(const TileRing& ring, double e) {
    TileRing r = clip_to_line(ring, 0, 0.0, false);
    r = clip_to_line(r, 0, e, true);
    r = clip_to_line(r, 1, 0.0, false);
    r = clip_to_line(r, 1, e, true);
    if (std::abs(signed_area(r)) < kMinRingArea) r.clear();
    return r;
}

// Both ends on one side of the clip square: taken as a cut through a building, not one of its
// walls. Known limit: a real wall lying exactly on that line is skipped too (the neighbour tile
// drops its zero-area copy); telling the two apart needs a per-vertex "made by the clip" flag.
bool on_one_clip_line(const Point2& a, const Point2& b, double e) {
    return (a[0] == b[0] && (a[0] == 0.0 || a[0] == e)) ||
           (a[1] == b[1] && (a[1] == 0.0 || a[1] == e));
}

Point3 sub(const Point3& a, const Point3& b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }

Point3 cross(const Point3& a, const Point3& b) {
    return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}

// A clipped ring with its vertices projected at the base and at the top.
struct Ring {
    TileRing uv;
    std::vector<Point3> base;
    std::vector<Point3> top;
};

// Projects the ring, then winds it so the solid lies left of every edge seen from above
// (exteriors counter-clockwise, holes clockwise). Decided in output space: the projection mirrors
// tile y, which runs south.
Ring project_ring(TileRing uv, double min_height_m, double height_m, const ProjectFn& project,
                  bool exterior) {
    Ring r;
    r.base.reserve(uv.size());
    r.top.reserve(uv.size());
    for (const Point2& p : uv) {
        r.base.push_back(project(p[0], p[1], min_height_m));
        r.top.push_back(project(p[0], p[1], height_m));
    }
    r.uv = std::move(uv);
    if ((signed_area(r.base) > 0.0) != exterior) {
        std::reverse(r.uv.begin(), r.uv.end());
        std::reverse(r.base.begin(), r.base.end());
        std::reverse(r.top.begin(), r.top.end());
    }
    return r;
}

uint32_t vertex_count(const TriangleMesh& m) {
    return static_cast<uint32_t>(m.positions.size() / 3);
}

void push_vertex(TriangleMesh& m, const Point3& p, const Point3& n) {
    for (size_t c = 0; c < 3; ++c) {
        m.positions.push_back(static_cast<float>(p[c]));
        m.normals.push_back(static_cast<float>(n[c]));
    }
}

uint32_t add_walls(const Ring& r, double e, TriangleMesh& out) {
    uint32_t quads = 0;
    const size_t n = r.uv.size();
    for (size_t i = 0; i < n; ++i) {
        const size_t j = (i + 1) % n;
        if (on_one_clip_line(r.uv[i], r.uv[j], e)) continue;
        const Point3& b0 = r.base[i];
        const Point3& b1 = r.base[j];
        const Point3& t1 = r.top[j];
        const Point3& t0 = r.top[i];
        // The quad's area vector (half the cross product of its diagonals) points away from the
        // solid on the edge's left.
        Point3 normal = cross(sub(t1, b0), sub(t0, b1));
        const double length =
            std::sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
        if (!(length > 0.0)) continue;  // a repeated vertex: no face
        for (double& c : normal) c /= length;
        const uint32_t k = vertex_count(out);
        for (const Point3& p : {b0, b1, t1, t0}) push_vertex(out, p, normal);
        out.indices.insert(out.indices.end(), {k, k + 1, k + 2, k, k + 2, k + 3});
        ++quads;
    }
    return quads;
}

uint32_t add_roof(const std::vector<Ring>& rings, TriangleMesh& out) {
    std::vector<std::vector<Point2>> polygon;  // the top rings seen from above
    std::vector<Point3> vertices;
    for (const Ring& r : rings) {
        std::vector<Point2>& flat = polygon.emplace_back();
        for (const Point3& p : r.top) {
            flat.push_back({p[0], p[1]});
            vertices.push_back(p);
        }
    }
    // Spelled ::mapbox because inside overlume::mapbox a bare `mapbox` names this namespace.
    const std::vector<uint32_t> tris = ::mapbox::earcut<uint32_t>(polygon);
    if (tris.empty()) return 0;
    const uint32_t k = vertex_count(out);
    for (const Point3& p : vertices) push_vertex(out, p, {0.0, 0.0, 1.0});
    for (size_t t = 0; t + 2 < tris.size(); t += 3) {
        const uint32_t a = tris[t];
        uint32_t b = tris[t + 1];
        uint32_t c = tris[t + 2];
        const Point3 ab = sub(vertices[b], vertices[a]);
        const Point3 ac = sub(vertices[c], vertices[a]);
        // Earcut's self-intersection pass can emit a clockwise triangle.
        if (ab[0] * ac[1] - ab[1] * ac[0] < 0.0) std::swap(b, c);
        out.indices.insert(out.indices.end(), {k + a, k + b, k + c});
    }
    return static_cast<uint32_t>(tris.size() / 3);
}

}

ExtrudeStats extrude_polygons(const std::vector<TilePolygon>& polygons, double clip_extent,
                              double min_height_m, double height_m, const ProjectFn& project,
                              TriangleMesh& out) {
    ExtrudeStats stats;
    // Also refuses NaN and infinite heights: values from a tile must not reach the mesh.
    if (!(std::isfinite(min_height_m) && std::isfinite(height_m) && height_m > min_height_m)) {
        return stats;
    }
    for (const TilePolygon& polygon : polygons) {
        ++stats.polygons_in;
        if (!all_finite(polygon.outer) ||
            !std::all_of(polygon.holes.begin(), polygon.holes.end(), all_finite)) {
            continue;
        }
        TileRing outer = clip_ring(polygon.outer, clip_extent);
        if (outer.empty()) continue;
        std::vector<Ring> rings;
        rings.push_back(project_ring(std::move(outer), min_height_m, height_m, project, true));
        for (const TileRing& hole : polygon.holes) {
            TileRing clipped = clip_ring(hole, clip_extent);
            if (clipped.empty()) continue;
            rings.push_back(
                project_ring(std::move(clipped), min_height_m, height_m, project, false));
        }
        ++stats.polygons_built;
        for (const Ring& r : rings) stats.wall_quads += add_walls(r, clip_extent, out);
        stats.roof_triangles += add_roof(rings, out);
    }
    return stats;
}

std::optional<std::array<double, 2>> footprint_centroid(const std::vector<TilePolygon>& polygons) {
    double total = 0.0, sum_u = 0.0, sum_v = 0.0;
    for (const TilePolygon& polygon : polygons) {
        const TileRing& r = polygon.outer;
        if (r.size() < 3 || !all_finite(r)) continue;
        // Moments about the first vertex keep the products small.
        const double u0 = r[0][0], v0 = r[0][1];
        double area2 = 0.0, mu = 0.0, mv = 0.0;  // 2 * area and 6 * area * centroid
        for (size_t i = 0, j = r.size() - 1; i < r.size(); j = i++) {
            const double uj = r[j][0] - u0, vj = r[j][1] - v0;
            const double ui = r[i][0] - u0, vi = r[i][1] - v0;
            const double c = uj * vi - ui * vj;
            area2 += c;
            mu += (uj + ui) * c;
            mv += (vj + vi) * c;
        }
        if (area2 == 0.0) continue;
        // Weighted by |area|, so exteriors of either winding count alike.
        const double w = std::abs(area2);
        sum_u += w * (u0 + mu / (3.0 * area2));
        sum_v += w * (v0 + mv / (3.0 * area2));
        total += w;
    }
    if (!(total > 0.0)) return std::nullopt;
    return std::array<double, 2>{sum_u / total, sum_v / total};
}

}
```

- [ ] **Step 5: Run it and see it pass** (the reconfigure picks up the new `src/mapbox_extrude.cpp`
  for `overlume_stream`)

```bash
cd "$WORKTREE"
flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock cmake -S overlume -B overlume/build > /dev/null \
    && echo reconfigured \
    && CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
        --target test_mapbox_extrude \
    && overlume/build/test_mapbox_extrude
```

No `reconfigured` line and no CMake error above it means the lock wait ran out (exit 75) and nothing
was built: run the step again.

Expected:
- `reconfigured`.
- The build ends with `[100%] Built target test_mapbox_extrude`.
- The run lists the 16 `MapboxExtrude.*` tests, each `[       OK ]`, and ends with
  `[  PASSED  ] 16 tests.`

- [ ] **Step 6: Format**

```bash
cd "$WORKTREE"
git add overlume/src/mapbox_extrude.hpp overlume/src/mapbox_extrude.cpp \
    overlume/tests/test_mapbox_extrude.cpp
tools/check_format.sh --fix overlume/src/mapbox_extrude.hpp overlume/src/mapbox_extrude.cpp \
    overlume/tests/test_mapbox_extrude.cpp
tools/check_format.sh overlume/src/mapbox_extrude.hpp overlume/src/mapbox_extrude.cpp \
    overlume/tests/test_mapbox_extrude.cpp
```

Expected: `check_format.sh: reformatted 3 files with clang-format 23.1.1`, then
`check_format.sh: PASS (3 files, clang-format 23.1.1)`.

- [ ] **Step 7: Gate** (foreground, Bash timeout 600000 ms)

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
ROS_DOMAIN_ID=82 CI_VISUAL_MODE_DOMAIN_ID=82 \
    flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock tools/ci_visual_mode.sh
```

Expected (exact counts: Task 0 adds no test, so the baseline plus the 16 `MapboxExtrude.*`
tests):
- `PASS  POD header check`
- `PASS  library ctest suite  (100% tests passed, 0 tests failed out of 363 (347 ok / 6 skipped))`
- `PASS  node gtests  (Summary: 363 tests, 0 errors, 0 failures, 0 skipped)`
- `PASS  WS bridge pytest suite  (82 passed in …)`, the time varying
- `PASS  golden suite: 23 ok, 0 skipped`
- `PASS  examples  (6 run, all exited 0 with non-empty output)`
- `OVERALL: PASS`, exit 0.

Exit 75: another lane held the lock for 300 s, so run the same command again. Any other non-zero
exit is a gate failure. A WS bridge failure can also come from another session's gate on the same
ports, so rerun before treating it as a finding.

- [ ] **Step 8: Commit**

```bash
cd "$WORKTREE"
git add overlume/src/mapbox_extrude.hpp overlume/src/mapbox_extrude.cpp \
    overlume/tests/test_mapbox_extrude.cpp
git commit -F - <<'EOF'
feat(mapbox): extrude streets-v8 footprints into walls and roofs

The mapbox://buildings source turns each z16 streets-v8 building footprint
into geometry (docs/plans/2026-10-14-mapbox-buildings.md, Task 3).
extrude_polygons clips every ring to the tile square (Sutherland-Hodgman,
crossings snapped onto the clip line) and re-winds rings in output space,
because the tile-to-frame projection mirrors tile y. Walls run from
min_height to height with outward per-face normals. Edges that lie on the
clip square get none, so buildings cut by z16 and z14 tile edges meet
without doubled walls. Known limit: a real wall lying exactly on a tile
edge (u or v exactly 0 or 4096) is skipped too, and the neighbour tile
drops its zero-area copy, so that face is missing. Roofs are earcut over
the top rings, holes included; any clockwise triangle from earcut's
self-intersection pass is flipped. There is no floor, as in GL JS.
Non-finite heights or coordinates build nothing. footprint_centroid gives
the unclipped, area-weighted centroid that conflation will test against
landmark footprints.

Claude-Session: <the executing session's URL>
EOF
git show --stat --format=%s HEAD
```

Expected: the subject line, then exactly three new files: `overlume/src/mapbox_extrude.cpp`,
`overlume/src/mapbox_extrude.hpp`, `overlume/tests/test_mapbox_extrude.cpp`.

---

### Task 4: Tile math and the tile frame

**Lane:** 3. **Depends on:** Task 0 (the lane-3 worktree forks after it).

**Files:**
- Create: `overlume/src/mapbox_tile_frame.hpp` (the Interface contract, verbatim)
- Create: `overlume/src/mapbox_tile_frame.cpp`
- Create: `overlume/src/mapbox_test_hooks.hpp` (the contract's head and its Task 4 block only)
- Test: `overlume/tests/test_mapbox_tile_frame.cpp` (suite `MapboxTileFrame`, 7 tests)

**Interfaces:**
- Consumes:
  - `glm::dmat4 compute_ecef_to_map(const GeoAnchor& anchor);` and
    `glm::dvec3 correct_ecef_point_height(const glm::dvec3& ecef_pos, const glm::dmat4& ecef_to_map, double anchor_height_m);`
    (`overlume/src/environment_stream.hpp:236-239`, namespace `overlume`; Task 0's guard message
    admits `mapbox_*.cpp`). The second keeps map x/y and sets map z to the ellipsoidal height minus
    `anchor_height_m` (`overlume/src/environment_stream.cpp:426-432`), exactly what
    `prepareInLoadThread` does to every vertex.
  - `bool ecef_to_map_probe(double origin_lat_deg, double origin_lon_deg, double heading_rad, double origin_height_m, double lat_deg, double lon_deg, double alt_m, double* out_x, double* out_y, double* out_z);`
    and `bool ecef_height_correction_probe(double origin_lat_deg, double origin_lon_deg, double heading_rad, double origin_height_m, double lat_deg, double lon_deg, double alt_m, double* out_z_uncorrected, double* out_z_corrected);`
    (`overlume/src/environment_test_hooks.hpp:36-42`, defined at
    `overlume/src/environment_stream.cpp:1359-1389`): the first takes a WGS84 geodetic point (alt is
    ellipsoidal) through ECEF into the anchor's map frame with no height correction; the second
    returns that point's tangent-plane z and its corrected z.
  - `struct GeoAnchor { double origin_lat_deg; double origin_lon_deg; double heading_rad; double origin_height_m; };`
    (`overlume/include/overlume/scene.h:211-216`).
- Produces: every declaration of `overlume/src/mapbox_tile_frame.hpp` and
  `bool overlume::testing::mapbox_tile_point_to_map_probe(overlume::GeoAnchor anchor, uint32_t z, uint32_t x, uint32_t y, double u, double v, double extent, double z_m, double out_map_xyz[3]);`,
  exactly as the contract states them. Lane 7 builds on them after lane 3's first merge.

**API notes** (cesium paths are relative to `overlume/build/_deps/cesium-native-src/`, v0.64.0):
- `static const Ellipsoid WGS84;` (`CesiumGeospatial/include/CesiumGeospatial/Ellipsoid.h:48`,
  radii 6378137, 6378137, 6356752.3142451793 at `CesiumGeospatial/src/Ellipsoid.cpp:17`) and
  `glm::dvec3 cartographicToCartesian(const Cartographic&) const noexcept` (`Ellipsoid.h:113-114`;
  `Ellipsoid.cpp:37-45`: the surface point plus `height` along the geodetic normal, so heights are
  ellipsoidal, the same convention `correct_ecef_point_height` undoes).
- `static constexpr Cartographic fromDegrees(double longitudeDegrees, double latitudeDegrees, double heightMeters = 0.0)`
  (`CesiumGeospatial/include/CesiumGeospatial/Cartographic.h:38-41`): longitude first.
- `static glm::dmat4x4 eastNorthUpToFixedFrame(const glm::dvec3& origin, const Ellipsoid& = WGS84)`
  (`CesiumGeospatial/include/CesiumGeospatial/GlobeTransforms.h:34-36`;
  `CesiumGeospatial/src/GlobeTransforms.cpp:36-44`: up = `geodeticSurfaceNormal(origin)`, east =
  normalize(−y, x, 0), north = up × east, translation = origin).
- `glm::inverse` (glm `matrix.hpp:156`); glm matrices are column-major, `m[column][row]`, the same
  order as `Mat4d` and glTF.
- `std::numbers::pi` (`<numbers>`, C++20: the .cpp compiles in `overlume_stream`).
- Not used, on purpose: `CesiumGeospatial::WebMercatorProjection` (its radius is WGS84's 6,378,137 m,
  E6) and `QuadtreeTilingScheme` (TMS rows, south up, E6). Tile ↔ lon/lat on the Web Mercator
  sphere does not depend on any radius, so it is written out; only Mapbox's node scale uses one,
  their 6,371,008.8 m mean radius (F1-scale).

- [ ] **Step 1: Create the header** `overlume/src/mapbox_tile_frame.hpp`, verbatim from the contract

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <array>
#include <cstdint>
#include <optional>

namespace overlume::mapbox {

inline constexpr uint32_t kLandmarkZoom = 14;
inline constexpr uint32_t kFootprintZoom = 16;
inline constexpr double kLandmarkExtent = 8192.0;          // landmark node units per z14 tile
inline constexpr double kMapboxSphereRadiusM = 6371008.8;  // the radius Mapbox's scale uses

// Column-major 4×4, as glm and glTF store matrices.
using Mat4d = std::array<double, 16>;

struct TileId {
    uint32_t z = 0;
    uint32_t x = 0;
    uint32_t y = 0;  // XYZ: row 0 at the north edge
};

struct LonLatDeg {
    double lon = 0.0;
    double lat = 0.0;
};

struct LonLatRect {
    double west = 0.0, south = 0.0, east = 0.0, north = 0.0;  // degrees
};

// Spherical Web Mercator: (u, v) in units of tile t, `extent` units per tile edge → lon/lat.
LonLatDeg tile_point_to_lonlat(const TileId& t, double u, double v, double extent);

// The zoom-z tile containing lon/lat; latitude is clamped to the Web Mercator limit.
TileId tile_containing(double lon_deg, double lat_deg, uint32_t z);

LonLatRect tile_bounds(const TileId& t);

// The 16 z16 tiles of z14 tile t, row-major from the north-west corner.
std::array<TileId, 16> footprint_tiles_of(const TileId& z14);

// The z14 tile's frame: east-north-up at the tile centre, on the WGS84 ellipsoid at
// origin_height_m (cesium's eastNorthUpToFixedFrame).
struct TileFrame {
    TileId tile;
    double origin_height_m = 0.0;
    Mat4d frame_to_ecef{};
    Mat4d ecef_to_frame{};
};

TileFrame make_tile_frame(const TileId& z14, double origin_height_m);

// The exact chain: (u, v) in units of tile `src` (`extent` per edge), z_m metres above
// origin_height_m → longitude and latitude (inverse Web Mercator) → ECEF at origin_height_m + z_m
// → tile-frame metres.
std::array<double, 3> project_to_frame(const TileFrame& f, const TileId& src, double u, double v,
                                       double extent, double z_m);

// Mapbox's landmark scale at a latitude: 8192·2^14 / (2π·6371008.8·cos lat), tile units per metre.
double mapbox_node_scale(double lat_deg);

// A landmark node matrix (local metres → z14 tile units: diag(s, −s, 1) plus (tx, ty, 0)) → the
// affine map from local metres into f, linearised at (tx, ty). nullopt unless the matrix has that
// form with s within 1% of mapbox_node_scale at the node's latitude.
std::optional<Mat4d> landmark_node_to_frame(const TileFrame& f, const Mat4d& node_matrix);

}
```

- [ ] **Step 2: Create the test-hook header** `overlume/src/mapbox_test_hooks.hpp` with the
  contract's includes and its Task 4 block only (Task 5 adds its block above Task 4's; Task 13
  appends its block at the end)

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "overlume/api.h"
#include "overlume/scene.h"

namespace overlume::testing {

// Task 4. A point (u, v) in tile (z, x, y) units (`extent` per edge) at z_m metres above the
// anchor's ground, through the Mapbox placement chain (tile frame of its z14 tile, ECEF, the map
// frame of `anchor`, the stream's height correction) → map metres.
bool mapbox_tile_point_to_map_probe(overlume::GeoAnchor anchor, uint32_t z, uint32_t x, uint32_t y,
                                    double u, double v, double extent, double z_m,
                                    double out_map_xyz[3]);

}
```

- [ ] **Step 3: Write the failing test** `overlume/tests/test_mapbox_tile_frame.cpp`

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume/scene.h"

#include "environment_test_hooks.hpp"
#include "mapbox_test_hooks.hpp"
#include "mapbox_tile_frame.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <optional>

namespace {

namespace mb = overlume::mapbox;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
// The test site (spec section 2, D3), with a heading and a non-zero ellipsoidal height so
// neither can hide behind a zero.
constexpr overlume::GeoAnchor kAnchor{25.1972, 55.2744, 0.35, 12.0};

// Spherical Web Mercator written out independently of the library: fractional tile x, y at zoom z.
std::array<double, 2> mercator_tile_xy(double lon_deg, double lat_deg, uint32_t z) {
    const double n = std::ldexp(1.0, static_cast<int>(z));
    return {(lon_deg + 180.0) / 360.0 * n,
            (1.0 - std::asinh(std::tan(lat_deg * kDegToRad)) / kPi) / 2.0 * n};
}

// Its inverse: (u, v) in units of tile t, `extent` per edge.
mb::LonLatDeg mercator_lonlat(const mb::TileId& t, double u, double v, double extent) {
    const double n = std::ldexp(1.0, static_cast<int>(t.z));
    return {(t.x + u / extent) / n * 360.0 - 180.0,
            std::atan(std::sinh(kPi * (1.0 - 2.0 * (t.y + v / extent) / n))) / kDegToRad};
}

mb::TileId anchor_tile() {
    return mb::tile_containing(kAnchor.origin_lon_deg, kAnchor.origin_lat_deg, mb::kLandmarkZoom);
}

}

TEST(MapboxTileFrame, XyzLonLatRoundTripAtBothDemoSites) {
    struct Site {
        const char* name;
        double lon, lat;
        uint32_t x, y;
    };
    // Downtown Dubai's z14 tile is 10707/7006 (spec section 3).
    const Site sites[] = {{"Downtown Dubai", 55.2744, 25.1972, 10707, 7006},
                          {"test site", 55.2744, 25.1972, 10707, 7006}};
    for (const Site& s : sites) {
        const std::array<double, 2> f = mercator_tile_xy(s.lon, s.lat, mb::kLandmarkZoom);
        EXPECT_EQ(static_cast<uint32_t>(std::floor(f[0])), s.x) << s.name;
        EXPECT_EQ(static_cast<uint32_t>(std::floor(f[1])), s.y) << s.name;

        const mb::TileId t = mb::tile_containing(s.lon, s.lat, mb::kLandmarkZoom);
        EXPECT_EQ(t.z, mb::kLandmarkZoom) << s.name;
        EXPECT_EQ(t.x, s.x) << s.name;
        EXPECT_EQ(t.y, s.y) << s.name;

        const mb::LonLatDeg back =
            mb::tile_point_to_lonlat(t, (f[0] - t.x) * mb::kLandmarkExtent,
                                     (f[1] - t.y) * mb::kLandmarkExtent, mb::kLandmarkExtent);
        EXPECT_NEAR(back.lon, s.lon, 1e-9) << s.name;
        EXPECT_NEAR(back.lat, s.lat, 1e-9) << s.name;

        const mb::LonLatRect b = mb::tile_bounds(t);
        EXPECT_LT(b.west, s.lon) << s.name;
        EXPECT_GT(b.east, s.lon) << s.name;
        EXPECT_LT(b.south, s.lat) << s.name;
        EXPECT_GT(b.north, s.lat) << s.name;
    }
}

TEST(MapboxTileFrame, TileContainingClampsToTheMercatorSquare) {
    const uint32_t last = (1u << mb::kLandmarkZoom) - 1;
    EXPECT_EQ(mb::tile_containing(0.0, 89.9, mb::kLandmarkZoom).y, 0u);
    EXPECT_EQ(mb::tile_containing(0.0, -89.9, mb::kLandmarkZoom).y, last);
    EXPECT_EQ(mb::tile_containing(180.0, 0.0, mb::kLandmarkZoom).x, last);
    EXPECT_EQ(mb::tile_containing(-180.0, 0.0, mb::kLandmarkZoom).x, 0u);

    const mb::LonLatRect world = mb::tile_bounds({0, 0, 0});
    EXPECT_NEAR(world.west, -180.0, 1e-9);
    EXPECT_NEAR(world.east, 180.0, 1e-9);
    EXPECT_NEAR(world.north, 85.0511287798066, 1e-9);
    EXPECT_NEAR(world.south, -85.0511287798066, 1e-9);
}

TEST(MapboxTileFrame, FootprintTilesAreTheSixteenZ16SquaresRowMajor) {
    const mb::TileId t = anchor_tile();
    const std::array<mb::TileId, 16> sub = mb::footprint_tiles_of(t);
    for (uint32_t j = 0; j < 4; ++j) {
        for (uint32_t i = 0; i < 4; ++i) {
            const mb::TileId& s = sub[j * 4 + i];
            EXPECT_EQ(s.z, mb::kFootprintZoom);
            EXPECT_EQ(s.x, 4 * t.x + i) << "i=" << i << " j=" << j;
            EXPECT_EQ(s.y, 4 * t.y + j) << "i=" << i << " j=" << j;
        }
    }
    const mb::LonLatRect whole = mb::tile_bounds(t);
    const mb::LonLatRect nw = mb::tile_bounds(sub[0]), se = mb::tile_bounds(sub[15]);
    EXPECT_NEAR(nw.west, whole.west, 1e-12);
    EXPECT_NEAR(nw.north, whole.north, 1e-12);
    EXPECT_NEAR(se.east, whole.east, 1e-12);
    EXPECT_NEAR(se.south, whole.south, 1e-12);
}

TEST(MapboxTileFrame, TileCentreIsTheFrameOriginWithEastNorthUpAxes) {
    const mb::TileId t = anchor_tile();
    const mb::TileFrame f = mb::make_tile_frame(t, kAnchor.origin_height_m);
    const auto at = [&](double u, double v, double z_m) {
        return mb::project_to_frame(f, t, u, v, mb::kLandmarkExtent, z_m);
    };
    for (const double c : at(4096.0, 4096.0, 0.0)) {
        EXPECT_NEAR(c, 0.0, 1e-6) << "the frame sits at the tile centre, origin_height_m up";
    }
    const std::array<double, 3> east = at(4196.0, 4096.0, 0.0);
    EXPECT_GT(east[0], 20.0);
    EXPECT_LT(std::abs(east[1]), 1e-3 * east[0]);
    const std::array<double, 3> north = at(4096.0, 3996.0, 0.0);
    EXPECT_GT(north[1], 20.0);
    EXPECT_LT(std::abs(north[0]), 1e-3 * north[1]);
    const std::array<double, 3> up = at(4096.0, 4096.0, 30.0);
    EXPECT_NEAR(up[0], 0.0, 1e-6);
    EXPECT_NEAR(up[1], 0.0, 1e-6);
    EXPECT_NEAR(up[2], 30.0, 1e-6);
}

TEST(MapboxTileFrame, PlacementChainMatchesTheStreamProbes) {
    const mb::TileId t = anchor_tile();
    struct Point {
        mb::TileId src;
        double u, v, extent;
    };
    // Three points of the z14 tile, and one of a z16 square, as the extrusions use them.
    const Point points[] = {{t, 0.0, 0.0, mb::kLandmarkExtent},
                            {t, 4096.0, 4096.0, mb::kLandmarkExtent},
                            {t, 7000.0, 1500.0, mb::kLandmarkExtent},
                            {mb::footprint_tiles_of(t)[5], 1000.0, 3000.0, 4096.0}};
    for (const Point& p : points) {
        for (const double z_m : {0.0, 30.0}) {
            double got[3] = {};
            ASSERT_TRUE(overlume::testing::mapbox_tile_point_to_map_probe(
                kAnchor, p.src.z, p.src.x, p.src.y, p.u, p.v, p.extent, z_m, got));
            const mb::LonLatDeg ll = mercator_lonlat(p.src, p.u, p.v, p.extent);
            const double alt = kAnchor.origin_height_m + z_m;
            double x = 0.0, y = 0.0, zTangent = 0.0, zRaw = 0.0, zCorrected = 0.0;
            ASSERT_TRUE(overlume::testing::ecef_to_map_probe(
                kAnchor.origin_lat_deg, kAnchor.origin_lon_deg, kAnchor.heading_rad,
                kAnchor.origin_height_m, ll.lat, ll.lon, alt, &x, &y, &zTangent));
            ASSERT_TRUE(overlume::testing::ecef_height_correction_probe(
                kAnchor.origin_lat_deg, kAnchor.origin_lon_deg, kAnchor.heading_rad,
                kAnchor.origin_height_m, ll.lat, ll.lon, alt, &zRaw, &zCorrected));
            EXPECT_NEAR(got[0], x, 0.02) << "z" << p.src.z << " u=" << p.u << " v=" << p.v;
            EXPECT_NEAR(got[1], y, 0.02) << "z" << p.src.z << " u=" << p.u << " v=" << p.v;
            EXPECT_NEAR(got[2], zCorrected, 0.02) << "u=" << p.u << " v=" << p.v;
            EXPECT_NEAR(got[2], z_m, 0.02) << "bases land on map z = 0 and roofs at their height";
        }
    }
}

TEST(MapboxTileFrame, LandmarkNodeAffineMapTracksTheExactChain) {
    const mb::TileId t = anchor_tile();
    const mb::TileFrame f = mb::make_tile_frame(t, kAnchor.origin_height_m);
    const double tx = 5000.0, ty = 3000.0;
    const double s =
        mb::mapbox_node_scale(mb::tile_point_to_lonlat(t, tx, ty, mb::kLandmarkExtent).lat);
    const std::optional<mb::Mat4d> a =
        mb::landmark_node_to_frame(f, {s, 0, 0, 0, 0, -s, 0, 0, 0, 0, 1, 0, tx, ty, 0, 1});
    ASSERT_TRUE(a.has_value());
    const mb::Mat4d& m = *a;
    // Heights run along the ellipsoid normal, so the z column is the anchor's normal exactly; here
    // it leans 6e-5 off the frame's z axis.
    const std::array<double, 3> ground =
        mb::project_to_frame(f, t, tx, ty, mb::kLandmarkExtent, 0.0);
    const std::array<double, 3> raised =
        mb::project_to_frame(f, t, tx, ty, mb::kLandmarkExtent, 1.0);
    for (size_t r = 0; r < 3; ++r) {
        EXPECT_NEAR(m[8 + r], raised[r] - ground[r], 1e-6) << "z column, row " << r;
    }
    for (const double dx : {-250.0, 0.0, 250.0}) {
        for (const double dy : {-250.0, 0.0, 250.0}) {
            for (const double dz : {0.0, 300.0}) {
                // Mapbox's matrix: local x east, y north, z up → tile units with v pointing south.
                const std::array<double, 3> exact =
                    mb::project_to_frame(f, t, tx + s * dx, ty - s * dy, mb::kLandmarkExtent, dz);
                const double ex = m[0] * dx + m[4] * dy + m[8] * dz + m[12] - exact[0];
                const double ey = m[1] * dx + m[5] * dy + m[9] * dz + m[13] - exact[1];
                const double ez = m[2] * dx + m[6] * dy + m[10] * dz + m[14] - exact[2];
                // 2.24 cm at worst, at a top corner: an affine map cannot follow the ellipsoid's
                // sag (9.8 mm at 354 m) or its up direction's tilt (1.7 cm at 300 m).
                EXPECT_LT(std::sqrt(ex * ex + ey * ey + ez * ez), 0.025)
                    << "dx=" << dx << " dy=" << dy << " dz=" << dz;
            }
        }
    }
}

TEST(MapboxTileFrame, LandmarkNodeRejectsUnexpectedMatrices) {
    const mb::TileId t = anchor_tile();
    const mb::TileFrame f = mb::make_tile_frame(t, kAnchor.origin_height_m);
    const double tx = 5000.0, ty = 3000.0;
    const double s =
        mb::mapbox_node_scale(mb::tile_point_to_lonlat(t, tx, ty, mb::kLandmarkExtent).lat);
    const mb::Mat4d good = {s, 0, 0, 0, 0, -s, 0, 0, 0, 0, 1, 0, tx, ty, 0, 1};
    EXPECT_TRUE(mb::landmark_node_to_frame(f, good).has_value());

    // 0.1 rad about z: the diagonal stays inside the 1% scale band (cos 0.1 = 0.995), so only the
    // rotation check can reject it.
    mb::Mat4d rotated = good;
    rotated[0] = s * std::cos(0.1);
    rotated[1] = s * std::sin(0.1);
    rotated[4] = s * std::sin(0.1);
    rotated[5] = -s * std::cos(0.1);
    EXPECT_FALSE(mb::landmark_node_to_frame(f, rotated).has_value()) << "rotated";

    mb::Mat4d scaled = good;
    scaled[0] *= 1.05;
    scaled[5] *= 1.05;
    EXPECT_FALSE(mb::landmark_node_to_frame(f, scaled).has_value()) << "scale 5% off";

    mb::Mat4d stretched = good;
    stretched[5] *= 1.001;
    EXPECT_FALSE(mb::landmark_node_to_frame(f, stretched).has_value()) << "x and y scales differ";

    mb::Mat4d lifted = good;
    lifted[14] = 5.0;
    EXPECT_FALSE(mb::landmark_node_to_frame(f, lifted).has_value()) << "z translation";

    // The Web Mercator scale on Mapbox's 6,371,008.8 m mean-radius sphere, not WGS84's 6,378,137 m
    // equatorial radius (0.11% apart).
    const double want = 8192.0 * 16384.0 / (2.0 * kPi * 6371008.8 * std::cos(25.0 * kDegToRad));
    EXPECT_NEAR(mb::mapbox_node_scale(25.0), want, want * 1e-12);
}
```

What each test catches when its behaviour is reverted:
- `XyzLonLatRoundTripAtBothDemoSites`: TMS (south-up) rows in `tile_containing` (y becomes
  16383 − y), a wrong inverse latitude in `tile_point_to_lonlat`, swapped bounds in `tile_bounds`.
  The expected tiles are computed in the test from an independent forward formula and also pinned
  as literals (Downtown 10707/7006 from spec §3).
- `TileContainingClampsToTheMercatorSquare`: dropping the `[0, n − 1]` clamp (longitude 180 gives
  column 16384) and a wrong Mercator limit at z0.
- `FootprintTilesAreTheSixteenZ16SquaresRowMajor`: column-major order, or a zoom other than 16.
- `TileCentreIsTheFrameOriginWithEastNorthUpAxes`: a frame on the ellipsoid surface instead of at
  `origin_height_m` (the centre lands at z = 12 m), a frame at the tile corner, or rotated or
  swapped axes.
- `PlacementChainMatchesTheStreamProbes`: `project_to_frame` ignoring `origin_height_m` (z off by
  12 m) or using `f.tile` instead of `src` (the z16 point lands kilometres away), a north-up `v`,
  and a chain without the height correction (z off by the tangent-plane sag, 0.34 m at the tile's
  north-west corner, 2.1 km from the anchor). The chain is otherwise an exact round trip, so the
  agreement is far inside 2 cm. Spec §8's chain bullet also names the node matrix; the next test
  covers it against the exact chain.
- `LandmarkNodeAffineMapTracksTheExactChain`: dropping the −s flip (y off by up to 500 m), taking
  the translation at the tile centre instead of the node anchor, or a z column other than the
  anchor's up direction. The direct column check catches the last one by about 40 times its 1e-6
  tolerance: 383 m from the tile centre the normal leans 6.0·10⁻⁵ off the frame's z axis. The
  grid alone would catch it by at most 8 mm (3.33 cm worst against 2.5 cm), and not at all nearer
  the tile centre. The grid's 2.5 cm bound is spec §8's.
- `LandmarkNodeRejectsUnexpectedMatrices`: removing any one validation (rotation, 1% scale band,
  isotropy, z translation) and a 6,378,137 m radius in `mapbox_node_scale`. The rotation is
  0.1 rad, so its diagonal (cos 0.1 = 0.995) stays inside the scale band and only the rotation
  check rejects it; reducing the off-diagonal loop to `{14}` fails this test.

- [ ] **Step 4: Run it and see it fail**

```bash
cd "$WORKTREE"
flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock cmake -S overlume -B overlume/build > /dev/null \
    && echo reconfigured \
    && CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
        --target test_mapbox_tile_frame
```

No `reconfigured` line and no CMake error above it means the lock wait ran out (exit 75) and nothing
was built: run the step again.

Expected: `reconfigured`, then the link of `test_mapbox_tile_frame` fails with lines such as
``undefined reference to `overlume::mapbox::tile_containing(double, double, unsigned int)'`` and
``undefined reference to `overlume::testing::mapbox_tile_point_to_map_probe(`` followed by
`linker command failed with exit code 1`; the build exits non-zero.

- [ ] **Step 5: Implement** `overlume/src/mapbox_tile_frame.cpp`

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "mapbox_tile_frame.hpp"

#include "environment_stream.hpp"
#include "mapbox_test_hooks.hpp"

#include <CesiumGeospatial/Cartographic.h>
#include <CesiumGeospatial/Ellipsoid.h>
#include <CesiumGeospatial/GlobeTransforms.h>

#include <glm/mat4x4.hpp>
#include <glm/matrix.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace overlume::mapbox {

namespace {

constexpr double kDegToRad = std::numbers::pi / 180.0;

glm::dmat4 to_glm(const Mat4d& m) {
    glm::dmat4 out(1.0);
    for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r) out[c][r] = m[static_cast<size_t>(c * 4 + r)];
    }
    return out;
}

Mat4d to_mat4d(const glm::dmat4& m) {
    Mat4d out{};
    for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r) out[static_cast<size_t>(c * 4 + r)] = m[c][r];
    }
    return out;
}

}

LonLatDeg tile_point_to_lonlat(const TileId& t, double u, double v, double extent) {
    const double n = std::ldexp(1.0, static_cast<int>(t.z));
    const double mercator = std::numbers::pi * (1.0 - 2.0 * (t.y + v / extent) / n);
    return {(t.x + u / extent) / n * 360.0 - 180.0, std::atan(std::sinh(mercator)) / kDegToRad};
}

TileId tile_containing(double lon_deg, double lat_deg, uint32_t z) {
    constexpr double kMaxLatDeg = 85.0511287798;
    const double lat = std::clamp(lat_deg, -kMaxLatDeg, kMaxLatDeg) * kDegToRad;
    const double n = std::ldexp(1.0, static_cast<int>(z));
    // NaN and anything west or north of the square land in column or row 0.
    const auto cell = [n](double f) {
        return f >= 0.0 ? static_cast<uint32_t>(std::min(std::floor(f), n - 1.0)) : 0u;
    };
    return {z, cell((lon_deg + 180.0) / 360.0 * n),
            cell((1.0 - std::asinh(std::tan(lat)) / std::numbers::pi) / 2.0 * n)};
}

LonLatRect tile_bounds(const TileId& t) {
    const LonLatDeg nw = tile_point_to_lonlat(t, 0.0, 0.0, 1.0);
    const LonLatDeg se = tile_point_to_lonlat(t, 1.0, 1.0, 1.0);
    return {nw.lon, se.lat, se.lon, nw.lat};
}

std::array<TileId, 16> footprint_tiles_of(const TileId& z14) {
    std::array<TileId, 16> out;
    for (uint32_t j = 0; j < 4; ++j) {
        for (uint32_t i = 0; i < 4; ++i) out[j * 4 + i] = {z14.z + 2, 4 * z14.x + i, 4 * z14.y + j};
    }
    return out;
}

TileFrame make_tile_frame(const TileId& z14, double origin_height_m) {
    const LonLatDeg centre = tile_point_to_lonlat(z14, 0.5, 0.5, 1.0);
    const glm::dvec3 origin = CesiumGeospatial::Ellipsoid::WGS84.cartographicToCartesian(
        CesiumGeospatial::Cartographic::fromDegrees(centre.lon, centre.lat, origin_height_m));
    const glm::dmat4 frameToEcef = CesiumGeospatial::GlobeTransforms::eastNorthUpToFixedFrame(
        origin, CesiumGeospatial::Ellipsoid::WGS84);
    TileFrame f;
    f.tile = z14;
    f.origin_height_m = origin_height_m;
    f.frame_to_ecef = to_mat4d(frameToEcef);
    f.ecef_to_frame = to_mat4d(glm::inverse(frameToEcef));
    return f;
}

std::array<double, 3> project_to_frame(const TileFrame& f, const TileId& src, double u, double v,
                                       double extent, double z_m) {
    const LonLatDeg ll = tile_point_to_lonlat(src, u, v, extent);
    const glm::dvec3 ecef = CesiumGeospatial::Ellipsoid::WGS84.cartographicToCartesian(
        CesiumGeospatial::Cartographic::fromDegrees(ll.lon, ll.lat, f.origin_height_m + z_m));
    const glm::dvec4 p = to_glm(f.ecef_to_frame) * glm::dvec4(ecef, 1.0);
    return {p.x, p.y, p.z};
}

double mapbox_node_scale(double lat_deg) {
    return kLandmarkExtent * std::ldexp(1.0, static_cast<int>(kLandmarkZoom)) /
           (2.0 * std::numbers::pi * kMapboxSphereRadiusM * std::cos(lat_deg * kDegToRad));
}

std::optional<Mat4d> landmark_node_to_frame(const TileFrame& f, const Mat4d& node_matrix) {
    const Mat4d& m = node_matrix;
    const double s = m[0], tx = m[12], ty = m[13];
    if (!(s > 0.0) || !std::isfinite(s) || !std::isfinite(tx) || !std::isfinite(ty)) {
        return std::nullopt;
    }
    // diag(s, -s, 1) plus (tx, ty, 0): no rotation, shear, projection or z translation.
    for (const int i : {1, 2, 3, 4, 6, 7, 8, 9, 11, 14}) {
        if (!(std::abs(m[i]) < 1e-9 * s)) return std::nullopt;
    }
    if (!(std::abs(-m[5] - s) <= 1e-6 * s) || !(std::abs(m[10] - 1.0) < 1e-9) ||
        !(std::abs(m[15] - 1.0) < 1e-9)) {
        return std::nullopt;
    }
    const double lat = tile_point_to_lonlat(f.tile, tx, ty, kLandmarkExtent).lat;
    if (!(std::abs(s / mapbox_node_scale(lat) - 1.0) <= 0.01)) return std::nullopt;

    // The exact chain, linearised at the anchor by central differences (1 tile unit, 1 m). Across
    // a 500 m × 500 m × 300 m building it stays within 2.3 cm of the exact chain: the Mercator
    // scale changes by 1.8e-5 over 250 m, the ellipsoid sags 9.8 mm at the 354 m corner, and the up
    // direction tilts by d/R, 1.7 cm at 300 m. So one node matrix places the whole model and its
    // vertices and accessor bounds stay untouched.
    const auto at = [&f](double u, double v, double w) {
        return project_to_frame(f, f.tile, u, v, kLandmarkExtent, w);
    };
    const std::array<double, 3> origin = at(tx, ty, 0.0);
    const std::array<double, 3> east = at(tx + 1.0, ty, 0.0), west = at(tx - 1.0, ty, 0.0);
    const std::array<double, 3> south = at(tx, ty + 1.0, 0.0), north = at(tx, ty - 1.0, 0.0);
    const std::array<double, 3> up = at(tx, ty, 1.0), down = at(tx, ty, -1.0);
    Mat4d a{};
    for (size_t r = 0; r < 3; ++r) {
        a[0 + r] = (east[r] - west[r]) / 2.0 * m[0];
        a[4 + r] = (south[r] - north[r]) / 2.0 * m[5];
        a[8 + r] = (up[r] - down[r]) / 2.0 * m[10];
        a[12 + r] = origin[r];
    }
    a[15] = 1.0;
    return a;
}

}

namespace overlume::testing {

bool mapbox_tile_point_to_map_probe(overlume::GeoAnchor anchor, uint32_t z, uint32_t x, uint32_t y,
                                    double u, double v, double extent, double z_m,
                                    double out_map_xyz[3]) {
    namespace mb = overlume::mapbox;
    if (out_map_xyz == nullptr || z < mb::kLandmarkZoom || z > 30) return false;
    const uint32_t shift = z - mb::kLandmarkZoom;
    const mb::TileFrame f =
        mb::make_tile_frame({mb::kLandmarkZoom, x >> shift, y >> shift}, anchor.origin_height_m);
    // As the stream places a vertex: the tile's transform (frame → ECEF), then ecefToMap_ and the
    // height correction of prepareInLoadThread.
    const std::array<double, 3> p = mb::project_to_frame(f, {z, x, y}, u, v, extent, z_m);
    const glm::dvec4 ecef = mb::to_glm(f.frame_to_ecef) * glm::dvec4(p[0], p[1], p[2], 1.0);
    const glm::dvec3 map = overlume::correct_ecef_point_height(
        glm::dvec3(ecef), overlume::compute_ecef_to_map(anchor), anchor.origin_height_m);
    out_map_xyz[0] = map.x;
    out_map_xyz[1] = map.y;
    out_map_xyz[2] = map.z;
    return true;
}

}
```

- [ ] **Step 6: Run it and see it pass**

```bash
cd "$WORKTREE"
flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock cmake -S overlume -B overlume/build > /dev/null \
    && echo reconfigured \
    && CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
        --target test_mapbox_tile_frame \
    && overlume/build/test_mapbox_tile_frame
```

No `reconfigured` line and no CMake error above it means the lock wait ran out (exit 75) and nothing
was built: run the step again.

Expected: `reconfigured` (Task 0's glob moves `src/mapbox_tile_frame.cpp` into `overlume_stream`),
a clean build, and `[  PASSED  ] 7 tests.`

- [ ] **Step 7: Format**

```bash
cd "$WORKTREE"
git add overlume/src/mapbox_tile_frame.hpp overlume/src/mapbox_tile_frame.cpp \
    overlume/src/mapbox_test_hooks.hpp overlume/tests/test_mapbox_tile_frame.cpp
tools/check_format.sh --fix overlume/src/mapbox_tile_frame.hpp overlume/src/mapbox_tile_frame.cpp \
    overlume/src/mapbox_test_hooks.hpp overlume/tests/test_mapbox_tile_frame.cpp
git diff --stat
tools/check_format.sh && tools/check_spdx.sh
```

Expected: `check_format.sh: reformatted 4 files with clang-format 23.1.1`, an empty `git diff --stat`
(the code above is already clang-format 23.1.1 clean), `check_format.sh: PASS (… files, clang-format
23.1.1)` and `check_spdx.sh: PASS (… files carry the SPDX identifier)`.

- [ ] **Step 8: Gate** (foreground, Bash timeout 600000 ms)

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
ROS_DOMAIN_ID=83 CI_VISUAL_MODE_DOMAIN_ID=83 \
    flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock tools/ci_visual_mode.sh
```

Exit 75 means another lane held the gate lock for 300 s: run the same command again. Any other
non-zero exit is a gate failure.

Expected: six `PASS` lines (POD header check, library ctest suite, node gtests, WS bridge pytest
suite, golden suite, examples), `OVERALL: PASS`, exit 0; the library ctest count is 7 higher than
at Task 0.

- [ ] **Step 9: Commit**

```bash
cd "$WORKTREE"
git add overlume/src/mapbox_tile_frame.hpp overlume/src/mapbox_tile_frame.cpp \
    overlume/src/mapbox_test_hooks.hpp overlume/tests/test_mapbox_tile_frame.cpp
git commit -F - <<'EOF'
feat(mapbox): Web Mercator tile math and the z14 tile frame

The Mapbox buildings source places landmark nodes and extruded footprints
through tile space, so both land on the same ground (spec section 5,
Placement). This adds XYZ <-> lon/lat on the Web Mercator sphere, an
east-north-up frame at each z14 tile's centre at the anchor's ellipsoidal
height, the exact tile -> frame chain the extrusions use, and the affine map
that replaces Mapbox's diag(s, -s, 1) landmark node matrices, validated
against Mapbox's 6371008.8 m mean-radius scale. mapbox_test_hooks.hpp starts
with a probe that runs the stream's own chain (tile frame, ECEF, ecefToMap_,
height correction); it agrees with ecef_to_map_probe within 2 cm, and the
affine map stays within 2.5 cm of the exact chain across a 500 m building.

Claude-Session: <the executing session's URL>
EOF
```

Then report Task 4 committed: lane 3's first merge into `mapbox-environment` happens at this
commit (Execution, Merges), which unblocks Task 13.

### Task 5: Landmark reader

**Lane:** 3. **Depends on:** Task 4 (same worktree, after its commit).

**Files:**
- Create: `overlume/src/mapbox_landmarks.hpp` (the Interface contract, verbatim)
- Create: `overlume/src/mapbox_landmarks.cpp`
- Create: `overlume/tests/mapbox_glb_fixture.hpp` (the contract's fixture API; helpers in
  `overlume::testing::glb::detail`)
- Modify: `overlume/src/mapbox_test_hooks.hpp:14-16` (as Task 4 created it: the contract's Task 5
  block goes between `namespace overlume::testing {` and Task 4's block, as its comment says)
- Test: `overlume/tests/test_mapbox_landmarks.cpp` (suite `MapboxLandmarks`, 12 tests)

**Interfaces:**
- Consumes (Task 4, `overlume/src/mapbox_tile_frame.hpp`): `Mat4d`, `TileId`, `TileFrame`,
  `kLandmarkZoom`, `kLandmarkExtent`,
  `TileFrame make_tile_frame(const TileId& z14, double origin_height_m);`,
  `std::optional<Mat4d> landmark_node_to_frame(const TileFrame& f, const Mat4d& node_matrix);`,
  `LonLatDeg tile_point_to_lonlat(const TileId& t, double u, double v, double extent);` and
  `std::array<double, 3> project_to_frame(const TileFrame& f, const TileId& src, double u, double v, double extent, double z_m);`.
- Produces:
  - `struct LandmarkTile`, `LandmarkTile read_landmark_tile(std::span<const std::byte> glb, const TileFrame& frame, bool keep_materials);`
    and `bool inside_any_footprint(const std::vector<std::array<double, 6>>& triangles, double x, double y);`
    exactly as the contract states them (Task 14 consumes them).
  - `struct MapboxLandmarkProbe` and
    `MapboxLandmarkProbe probe_mapbox_landmark_tile(const std::vector<uint8_t>& glb, uint32_t x14, uint32_t y14, double origin_height_m, bool keep_materials);`
    and
    `bool mapbox_inside_any_footprint_probe(const std::vector<std::array<double, 6>>& triangles, double x, double y);`
    (contract).
  - The fixture API `glb::LandmarkSpec`, `glb::GlbOptions` and
    `std::vector<uint8_t> build_landmark_glb(const std::vector<LandmarkSpec>& landmarks, const GlbOptions& opts);`
    (contract), used by Tasks 14 and 15.

**API notes** (cesium paths are relative to `overlume/build/_deps/cesium-native-src/`, v0.64.0):
- Link and include reach: `CesiumGltfContent` is a PUBLIC dependency of `Cesium3DTilesContent`
  (`Cesium3DTilesContent/CMakeLists.txt:54-62`), itself PUBLIC in `Cesium3DTilesSelection`
  (`Cesium3DTilesSelection/CMakeLists.txt:42-45`), which `overlume_stream` links
  (`overlume/CMakeLists.txt:257-264`). A build of this commit's CMake shows
  `CesiumGltfContent/include` in `overlume_stream`'s flags and `libCesiumGltfContent.a` on the test
  link line, so Task 0 needs no change.
- `GltfReaderResult GltfReader::readGltf(const std::span<const std::byte>& data, const GltfReaderOptions& options = GltfReaderOptions()) const;`
  (`CesiumGltfReader/include/CesiumGltfReader/GltfReader.h:194-196`), result `{model, errors,
  warnings}` (`GltfReader.h:28-43`). It is synchronous and resolves no URIs. Non-GLB bytes go to
  the JSON reader (`CesiumGltfReader/src/GltfReader.cpp:410-411`); a short or truncated GLB gives
  `errors` "Too short to be a valid GLB." or "GLB extends past the end of the buffer…"
  (`GltfReader.cpp:127`, `150`). `read_landmark_tile` checks the `glTF` magic itself first, because
  a JSON error body would otherwise parse as an empty glTF with no errors.
  - Errors and a model together: any JSON-level error drops the model
    (`CesiumJsonReader/include/CesiumJsonReader/JsonReader.h:75-76`), but a BIN chunk that buffer
    0 does not fit returns the model with an error ("…larger than the size of the GLB binary
    chunk…", `GltfReader.cpp:214-236`). Postprocessing still runs on that model (`:413`), with
    buffer 0 holding no bytes. Otherwise buffer 0 gets exactly `byteLength` bytes (`:250-252`).
- `GltfReaderOptions`: `decodeEmbeddedImages` (`GltfReader.h:101`) is the only field changed;
  `decodeMeshOptData` (`:119`), `dequantizeMeshData` (`:133`) and `applyTextureTransform` (`:139`)
  stay true. These defaults equal what the tileset passes (`TilesetContentOptions::toGltfReaderOptions`
  forwards only KTX2 targets, `applyTextureTransform` and primitive-mode options, all default in
  Overlume; fact-check E3, A1).
- Post-processing order: meshopt (`GltfReader.cpp:363`, only when `EXT_meshopt_compression` is in
  `extensionsUsed`), dequantization (`:375`), `KHR_texture_transform` (`:383`).
  - `decodeMeshOpt.cpp:143-152`: a stream that fails to decode only adds a warning and keeps its
    `EXT_meshopt_compression`; `:153-162` a decoded view moves to a new buffer and loses the
    extension; `:166-167` the names leave `extensionsUsed`/`extensionsRequired` either way. Hence
    the leftover-extension check.
  - Each pass that runs removes its own name the same way (`dequantizeMeshData.cpp:195`,
    `applyKhrTextureTransform.cpp:141`; `Model::removeExtensionRequired` erases from both lists,
    `CesiumGltf/src/Model.cpp:962-971`). A required extension the reader does not decode stays
    listed: `KHR_meshopt_compression` is never decoded, and its views keep pointing into the
    fallback buffer. Hence the `extensionsRequired` check. Mapbox's tiles require only
    `EXT_meshopt_compression` and `KHR_mesh_quantization` (fact-check F1), so a good read leaves
    the list empty.
  - `dequantizeMeshData.cpp:123-136`: each quantized attribute gets a new float view and buffer;
    the old ones become unreferenced.
  - `applyKhrTextureTransform.cpp:94-115`, `:135`: the occlusion texture's transform rewrites
    TEXCOORD_0 into a new buffer, view and accessor, then erases the extension from the material.
- `CesiumGltfContent::GltfUtilities` (`CesiumGltfContent/include/CesiumGltfContent/GltfUtilities.h:178-272`):
  `removeUnusedTextures`, `removeUnusedSamplers`, `removeUnusedImages`, `removeUnusedMeshes`,
  `removeUnusedAccessors`, `removeUnusedBufferViews`, `removeUnusedBuffers` (each
  `(Model&, const std::vector<int32_t>& extraUsed = {})`) and `compactBuffers(Model&)`.
  - **Trap:** `VisitTextureIds` (`CesiumGltfContent/src/GltfUtilities.cpp:760-777`) visits
    emissive, normal, base-colour and metallic-roughness textures but not `occlusionTexture`,
    Mapbox's only texture slot. Original mode therefore never calls the texture passes; clay mode
    clears every material texture reference first. Without that, `removeUnusedTextures` still
    empties the texture array and every material names texture 0 of nothing. cgltf (in Filament's
    `libgltfio_core.a`) rejects such a document: `cgltf_parse` returned 4
    (`cgltf_result_invalid_gltf`) for a material whose `occlusionTexture` index has no texture
    (measured), which would fail the whole merged tile. `kept_model_problem` rejects such a model,
    so the clay tests fail if the reset goes.
  - Meshes are found through `node.mesh` (`:953-958`), image views through `image.bufferView`
    (`:900-912`), buffers through `bufferView.buffer` and the meshopt extension's buffer
    (`:939-950`).
  - `compactBuffer` asserts `byteLength == data.size()` (`:1110`, `:1175`) and erases byte ranges
    (`:1161`); meshopt's fallback buffer has a byteLength and no data, so `removeUnusedBuffers`
    must run first (fact-check E, Design implications). Gaps ahead of a kept range shrink in
    multiples of 8 bytes (`:1119`).
  - Those asserts are `CESIUM_ASSERT`, plain `assert` (`CesiumUtility/include/CesiumUtility/Assert.h:8-20`),
    compiled out by `overlume_stream`'s `-DNDEBUG`. A kept buffer without its bytes therefore
    makes `deleteBufferRange` erase from an empty vector. Measured without the guard below: the
    fixture with `KHR_meshopt_compression` in place of `EXT_meshopt_compression` segfaults there
    (exit 139).
    A view running past its buffer is kept as is. The `removeUnused*` passes skip out-of-range
    indices (`:980`, `:986`), so only compaction needs a guard: `kept_model_problem` runs just
    before it.
- cesium shares one static `GltfReader` across worker threads
  (`Cesium3DTilesContent/src/BinaryToGltfConverter.cpp:14`); `read_landmark_tile` does the same
  with a function-local static, and `readGltf` is `const`.
- glTF structs: `Model::scene` (`-1` when absent) and `scenes` (`CesiumGltf/generated/include/CesiumGltf/ModelSpec.h:123`,
  `:128`); `NamedObject::name` is a plain `std::string` (`CesiumGltf/include/CesiumGltf/NamedObject.h:21`);
  `Node::children`, `matrix` (16 doubles, identity by default), `mesh` (`Node.h:37`, `:52`, `:57`);
  `Scene::nodes` (`Scene.h:24`); `Material` texture slots (`Material.h:52-86`,
  `MaterialPBRMetallicRoughness.h:44`, `:71`); `MeshPrimitive::attributes`, `indices`, `mode`
  (`MeshPrimitive.h:55`, `:64`, `:77`); `ExtensibleObject::hasExtension<T>()` and `extras`
  (`CesiumUtility/include/CesiumUtility/ExtensibleObject.h:22`, `:111`); `Model::getSafe`
  (`CesiumGltf/include/CesiumGltf/Model.h:200-226`, null for an out-of-range index);
  `Model::isExtensionUsed` / `isExtensionRequired` (`Model.h:274`, `:284`);
  `ExtensionBufferExtMeshoptCompression::fallback` and the view extension's fields
  (`ExtensionBufferExtMeshoptCompression.h:28`, `ExtensionBufferViewExtMeshoptCompression.h:60-95`).
- `AccessorView<T>(const Model&, int32_t)`, `status()`, `size()`, `operator[]`
  (`CesiumGltf/include/CesiumGltf/AccessorView.h:173`, `:192`, `:208`, `:216`);
  `IndexAccessorType = std::variant<std::monostate, AccessorView<uint8_t>, AccessorView<uint16_t>, AccessorView<uint32_t>>`
  and `getIndexAccessorView(const Model&, const MeshPrimitive&)` (`AccessorUtility.h:268-286`).
- Fixture encoder, `meshoptimizer.h` (vcpkg meshoptimizer 1.1; Task 0 puts its directory on the
  `test_mapbox_*` include paths): `meshopt_encodeIndexBuffer` and `…Bound` (`:250-251`),
  `meshopt_encodeVertexBuffer` and `…Bound` (`:341-342`, vertex size a multiple of 4),
  `meshopt_encodeVertexVersion` (`:362`, default 1, so the fixture sets 0). Filament 1.56.5's
  `overlume/build/_deps/filament-1.56.5/filament/lib/x86_64/libmeshoptimizer.a` defines all five
  symbols too and comes first on the link line (fact-check D2-meshopt-copies), so today the
  older v0-only encoder runs; either copy writes v0 (header byte `0xa0`). cesium decodes
  `TRIANGLES` views of stride 2 as u16 (`decodeMeshOpt.cpp:95-99`).
- JPEG: `typedef void stbi_write_func(void* context, void* data, int size);` and
  `int stbi_write_jpg_to_func(stbi_write_func* func, void* context, int x, int y, int comp, const void* data, int quality);`
  (`overlume/build/_deps/stb/stb_image_write.h:187`, `:193`, `extern "C"`); the implementation is
  in every test binary through `overlume/tests/golden.cpp:8-9`.
- Mapbox's layout (fact-check F1-meshopt, F1-meshopt-v2-lod): buffer 0 is the BIN chunk with the
  compressed streams and the JPEGs, buffer 1 the fallback; one scene, "Default Scene", and no
  top-level `scene`; building and footprint nodes share a `diag(s, −s, 1)` matrix; footprints are
  POSITION-only at z = 0 with extras `mapbox:footprint:id`; the only texture is an occlusion map
  with `KHR_texture_transform`. GL JS's `-meshopt-v2-lod` variant adds a second scene, "LOD"; the
  fixture writes one too, so the tests cover the reader dropping it. The fixture uses the `NONE`
  filter on every stream (Mapbox also uses `OCTAHEDRAL` and `EXPONENTIAL`); cesium's decode path
  is the same up to the filter pass. Vertex streams are codec v0 (`0xa0`) in the `-meshopt` tiles
  the plan fetches (Global Constraints, Endpoints) and codec v1 (`0xa1`) in `-meshopt-v2-lod`
  tiles; index streams are `0xe1` in both (measured on the probe's Downtown tiles).

- [ ] **Step 1: Create the header** `overlume/src/mapbox_landmarks.hpp`, verbatim from the contract

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

static_assert(__cplusplus >= 202002L, "mapbox_landmarks.hpp is C++20-only (overlume_stream)");

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

#include <CesiumGltf/Model.h>

#include "mapbox_tile_frame.hpp"

namespace overlume::mapbox {

struct LandmarkTile {
    // The kept building nodes only, all in scene 0; each node's matrix replaced by its
    // local-metres → tile-frame map. No nodes when the GLB held none that were usable.
    CesiumGltf::Model model;
    // Landmark footprint triangles in tile-frame metres: x0, y0, x1, y1, x2, y2.
    std::vector<std::array<double, 6>> footprint_triangles;
    uint32_t buildings_kept = 0;
    uint32_t nodes_skipped = 0;  // a node with an unexpected matrix
    bool ok = false;             // false: unreadable or undecodable; `error` says why
    std::string error;
};

// Reads one landmark GLB: cesium decodes meshopt and quantization, images are not decoded. Keeps
// the "Default Scene" scene (else `scene`, else scene 0), collects footprint triangles from nodes
// whose extras carry `mapbox:footprint:id`, and drops footprint nodes, the LOD scene and everything
// only they used, including meshopt's source and fallback buffers. keep_materials = false (clay)
// also drops textures, images and samplers. A bufferView still carrying EXT_meshopt_compression
// after reading, a required extension the reader does not decode, or a pruned model with a buffer,
// view or texture reference out of range makes the whole tile ok = false. A footprint node with a
// valid matrix yields its triangles even when its building node was skipped.
LandmarkTile read_landmark_tile(std::span<const std::byte> glb, const TileFrame& frame,
                                bool keep_materials);

// True when (x, y) in tile-frame metres lies inside any footprint triangle.
bool inside_any_footprint(const std::vector<std::array<double, 6>>& triangles, double x, double y);

}
```

- [ ] **Step 2: Add the Task 5 hooks** to `overlume/src/mapbox_test_hooks.hpp`: the contract's
  Task 5 block, verbatim, directly after `namespace overlume::testing {` and above Task 4's block,
  so that it cannot meet Task 13's append

Replace

```cpp
namespace overlume::testing {

// Task 4. A point (u, v) in tile (z, x, y) units (`extent` per edge) at z_m metres above the
```

with

```cpp
namespace overlume::testing {

// Task 5. Its block goes directly after `namespace overlume::testing {`, above Task 4's block, so
// that Task 13's block (appended at the end, possibly before Task 5 merges) merges cleanly.
// inside_any_footprint for C++17 tests (the real one is declared in C++20 mapbox_landmarks.hpp).
bool mapbox_inside_any_footprint_probe(const std::vector<std::array<double, 6>>& triangles,
                                       double x, double y);
struct MapboxLandmarkProbe {
    bool ok = false;
    std::string error;
    uint32_t buildings_kept = 0;
    uint32_t nodes_skipped = 0;
    uint32_t scenes = 0, nodes = 0, meshes = 0, accessors = 0, buffer_views = 0, buffers = 0;
    uint32_t images = 0, textures = 0, samplers = 0, materials = 0;
    uint32_t footprint_triangles = 0;
    uint64_t buffer_bytes = 0;            // summed size of every buffer's data
    bool meshopt_extension_left = false;  // EXT_meshopt_compression anywhere after reading
    std::vector<std::array<double, 16>> node_matrices;  // kept nodes, column-major, in order
};
MapboxLandmarkProbe probe_mapbox_landmark_tile(const std::vector<uint8_t>& glb, uint32_t x14,
                                               uint32_t y14, double origin_height_m,
                                               bool keep_materials);

// Task 4. A point (u, v) in tile (z, x, y) units (`extent` per edge) at z_m metres above the
```

- [ ] **Step 3: Create the fixture** `overlume/tests/mapbox_glb_fixture.hpp`

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

// Landmark GLBs in the layout Mapbox's 3dtiles endpoint serves (spec section 3), built in the test
// process so no Mapbox bytes are stored. Header-only C++17. stb_image_write's implementation comes
// from golden.cpp, which every test binary compiles.

#include <meshoptimizer.h>

#include "stb_image_write.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace overlume::testing::glb {

struct LandmarkSpec {
    uint64_t id = 1;
    double tx = 4096.0, ty = 4096.0;  // node anchor, z14 tile units (0..8192)
    double lat_deg = 25.0;            // latitude for Mapbox's node scale
    double half_x_m = 10.0, half_y_m = 10.0, height_m = 100.0;  // the building box, local metres
    double footprint_half_m = 12.0;  // the footprint square's half size, local metres
};

struct GlbOptions {
    bool meshopt = true;  // EXT_meshopt_compression, vertex codec v0, on vertex and index views
    // KHR_mesh_quantization: NORMAL normalized byte, TEXCOORD_0 normalized ushort.
    bool quantize = true;
    bool lod_scene = true;             // a second scene "LOD" with low-detail duplicate nodes
    bool occlusion_texture = true;     // a material with a small JPEG occlusionTexture (stb)
    bool bad_matrix_on_first = false;  // the first building node gets a rotated matrix
    bool corrupt_meshopt = false;      // the first compressed vertex stream is garbled
};

namespace detail {

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr int kFloat = 5126, kByte = 5120, kUnsignedShort = 5123;

inline std::string num(double v) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.17g", v);
    return buf;
}

inline std::string join(const std::vector<std::string>& items) {
    std::string out = "[";
    for (size_t i = 0; i < items.size(); ++i) out += (i == 0 ? "" : ",") + items[i];
    return out + "]";
}

inline void pad(std::vector<uint8_t>& bytes, size_t alignment) {
    while (bytes.size() % alignment != 0) bytes.push_back(0);
}

inline void put_u32(std::vector<uint8_t>& out, uint32_t v) {
    for (int i = 0; i < 4; ++i) out.push_back(static_cast<uint8_t>(v >> (8 * i)));
}

inline std::vector<uint8_t> float_bytes(const std::vector<float>& values) {
    std::vector<uint8_t> out(values.size() * sizeof(float));
    std::memcpy(out.data(), values.data(), out.size());
    return out;
}

// One mesh in local metres (x east, y north, z up).
struct Geometry {
    std::vector<float> positions;  // x, y, z
    std::vector<float> normals;    // x, y, z; empty: no NORMAL
    std::vector<float> uvs;        // u, v; empty: no TEXCOORD_0
    std::vector<uint16_t> indices;
};

// The box (-hx, -hy, 0)..(hx, hy, h): 24 vertices with per-face normals and UVs, 36 indices,
// counter-clockwise seen from outside.
inline Geometry box(double hx, double hy, double h) {
    const double lo[3] = {-hx, -hy, 0.0}, hi[3] = {hx, hy, h};
    static constexpr int kCorner[4][2] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
    Geometry g;
    for (int a = 0; a < 3; ++a) {
        for (const int sign : {1, -1}) {
            const int u = (a + 1) % 3, v = (a + 2) % 3;
            const auto base = static_cast<uint16_t>(g.positions.size() / 3);
            for (int k = 0; k < 4; ++k) {
                const int* c = kCorner[sign > 0 ? k : 3 - k];
                double p[3];
                p[a] = sign > 0 ? hi[a] : lo[a];
                p[u] = c[0] != 0 ? hi[u] : lo[u];
                p[v] = c[1] != 0 ? hi[v] : lo[v];
                for (int i = 0; i < 3; ++i) {
                    g.positions.push_back(static_cast<float>(p[i]));
                    g.normals.push_back(i == a ? static_cast<float>(sign) : 0.0f);
                }
                g.uvs.push_back(static_cast<float>(c[0]));
                g.uvs.push_back(static_cast<float>(c[1]));
            }
            for (const int i : {0, 1, 2, 0, 2, 3}) {
                g.indices.push_back(static_cast<uint16_t>(base + i));
            }
        }
    }
    return g;
}

// The square (-half, -half)..(half, half) at height z, counter-clockwise seen from above.
inline Geometry square(double half, double z, bool with_normals) {
    static constexpr int kCorner[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
    Geometry g;
    for (const auto& c : kCorner) {
        g.positions.push_back(static_cast<float>(c[0] * half));
        g.positions.push_back(static_cast<float>(c[1] * half));
        g.positions.push_back(static_cast<float>(z));
        if (with_normals) g.normals.insert(g.normals.end(), {0.0f, 0.0f, 1.0f});
    }
    g.indices = {0, 1, 2, 0, 2, 3};
    return g;
}

// A 16×16 grey ramp as a real JPEG stream.
inline std::vector<uint8_t> occlusion_jpeg() {
    constexpr int kSide = 16;
    std::vector<uint8_t> pixels(kSide * kSide * 3);
    for (int y = 0; y < kSide; ++y) {
        for (int x = 0; x < kSide; ++x) {
            for (int c = 0; c < 3; ++c) {
                pixels[(y * kSide + x) * 3 + c] = static_cast<uint8_t>(64 + 8 * x + 4 * y);
            }
        }
    }
    std::vector<uint8_t> jpeg;
    stbi_write_jpg_to_func(
        [](void* context, void* data, int size) {
            auto* out = static_cast<std::vector<uint8_t>*>(context);
            const auto* bytes = static_cast<const uint8_t*>(data);
            out->insert(out->end(), bytes, bytes + size);
        },
        &jpeg, kSide, kSide, 3, pixels.data(), 90);
    return jpeg;
}

// Buffer views, accessors and buffer 0 in Mapbox's layout. With meshopt, every view lives in the
// fallback buffer 1, which holds no bytes, and its EXT_meshopt_compression points at the
// compressed stream in buffer 0.
struct Writer {
    GlbOptions opts;
    bool corrupt_next_vertex_stream = false;
    std::vector<uint8_t> bin;   // buffer 0, the BIN chunk
    size_t fallback_bytes = 0;  // buffer 1's byteLength (meshopt only)
    std::vector<std::string> views, accessors;

    int add_view(const std::vector<uint8_t>& raw, const std::vector<uint8_t>& compressed,
                 size_t count, size_t stride, const char* mode, bool vertex) {
        const std::string view_stride = vertex ? ",\"byteStride\":" + std::to_string(stride) : "";
        pad(bin, 4);
        std::string json;
        if (opts.meshopt) {
            fallback_bytes = (fallback_bytes + 3) / 4 * 4;
            json = "{\"buffer\":1,\"byteOffset\":" + std::to_string(fallback_bytes) +
                   ",\"byteLength\":" + std::to_string(raw.size()) + view_stride +
                   ",\"extensions\":{\"EXT_meshopt_compression\":{\"buffer\":0,\"byteOffset\":" +
                   std::to_string(bin.size()) +
                   ",\"byteLength\":" + std::to_string(compressed.size()) +
                   ",\"byteStride\":" + std::to_string(stride) +
                   ",\"count\":" + std::to_string(count) + ",\"mode\":\"" + mode + "\"}}}";
            fallback_bytes += raw.size();
            bin.insert(bin.end(), compressed.begin(), compressed.end());
        } else {
            json = "{\"buffer\":0,\"byteOffset\":" + std::to_string(bin.size()) +
                   ",\"byteLength\":" + std::to_string(raw.size()) + view_stride + "}";
            bin.insert(bin.end(), raw.begin(), raw.end());
        }
        views.push_back(json);
        return static_cast<int>(views.size()) - 1;
    }

    int vertex_view(const std::vector<uint8_t>& raw, size_t count, size_t stride) {
        std::vector<uint8_t> compressed;
        if (opts.meshopt) {
            compressed.resize(meshopt_encodeVertexBufferBound(count, stride));
            compressed.resize(meshopt_encodeVertexBuffer(compressed.data(), compressed.size(),
                                                         raw.data(), count, stride));
            if (corrupt_next_vertex_stream) {
                // No valid header byte: every meshopt decoder rejects the stream.
                std::fill(compressed.begin(), compressed.end(), uint8_t{0xff});
                corrupt_next_vertex_stream = false;
            }
        }
        return add_view(raw, compressed, count, stride, "ATTRIBUTES", true);
    }

    int index_view(const std::vector<uint16_t>& indices, size_t vertex_count) {
        std::vector<uint8_t> raw(indices.size() * 2);
        std::memcpy(raw.data(), indices.data(), raw.size());
        std::vector<uint8_t> compressed;
        if (opts.meshopt) {
            const std::vector<unsigned int> wide(indices.begin(), indices.end());
            compressed.resize(meshopt_encodeIndexBufferBound(wide.size(), vertex_count));
            compressed.resize(meshopt_encodeIndexBuffer(compressed.data(), compressed.size(),
                                                        wide.data(), wide.size()));
        }
        return add_view(raw, compressed, indices.size(), 2, "TRIANGLES", false);
    }

    int image_view(const std::vector<uint8_t>& jpeg) {
        pad(bin, 4);
        views.push_back("{\"buffer\":0,\"byteOffset\":" + std::to_string(bin.size()) +
                        ",\"byteLength\":" + std::to_string(jpeg.size()) + "}");
        bin.insert(bin.end(), jpeg.begin(), jpeg.end());
        return static_cast<int>(views.size()) - 1;
    }

    int accessor(int view, int component_type, const char* type, size_t count, bool normalized,
                 const std::string& bounds = "") {
        accessors.push_back("{\"bufferView\":" + std::to_string(view) +
                            ",\"componentType\":" + std::to_string(component_type) +
                            ",\"count\":" + std::to_string(count) + ",\"type\":\"" + type + "\"" +
                            (normalized ? ",\"normalized\":true" : "") + bounds + "}");
        return static_cast<int>(accessors.size()) - 1;
    }

    // One TRIANGLES primitive: POSITION float with min/max; NORMAL and TEXCOORD_0 quantized as
    // Mapbox's are, or float; u16 indices.
    std::string mesh(const Geometry& g, int material) {
        const size_t n = g.positions.size() / 3;
        std::array<float, 3> lo{g.positions[0], g.positions[1], g.positions[2]}, hi = lo;
        for (size_t i = 0; i < n; ++i) {
            for (size_t c = 0; c < 3; ++c) {
                lo[c] = std::min(lo[c], g.positions[i * 3 + c]);
                hi[c] = std::max(hi[c], g.positions[i * 3 + c]);
            }
        }
        const std::string bounds = ",\"min\":[" + num(lo[0]) + "," + num(lo[1]) + "," + num(lo[2]) +
                                   "],\"max\":[" + num(hi[0]) + "," + num(hi[1]) + "," +
                                   num(hi[2]) + "]";
        std::string attributes =
            "\"POSITION\":" + std::to_string(accessor(vertex_view(float_bytes(g.positions), n, 12),
                                                      kFloat, "VEC3", n, false, bounds));
        if (!g.normals.empty()) {
            int normal = 0;
            if (opts.quantize) {
                std::vector<uint8_t> q(n * 4, 0);  // normalized BYTE x, y, z and a zero pad byte
                for (size_t i = 0; i < n; ++i) {
                    for (size_t c = 0; c < 3; ++c) {
                        q[i * 4 + c] = static_cast<uint8_t>(
                            static_cast<int8_t>(std::lround(g.normals[i * 3 + c] * 127.0f)));
                    }
                }
                normal = accessor(vertex_view(q, n, 4), kByte, "VEC3", n, true);
            } else {
                normal =
                    accessor(vertex_view(float_bytes(g.normals), n, 12), kFloat, "VEC3", n, false);
            }
            attributes += ",\"NORMAL\":" + std::to_string(normal);
        }
        if (!g.uvs.empty()) {
            int uv = 0;
            if (opts.quantize) {
                std::vector<uint8_t> q(n * 4);  // normalized UNSIGNED_SHORT u, v
                for (size_t i = 0; i < n * 2; ++i) {
                    const auto value = static_cast<uint16_t>(std::lround(g.uvs[i] * 65535.0f));
                    std::memcpy(&q[i * 2], &value, 2);
                }
                uv = accessor(vertex_view(q, n, 4), kUnsignedShort, "VEC2", n, true);
            } else {
                uv = accessor(vertex_view(float_bytes(g.uvs), n, 8), kFloat, "VEC2", n, false);
            }
            attributes += ",\"TEXCOORD_0\":" + std::to_string(uv);
        }
        const int indices =
            accessor(index_view(g.indices, n), kUnsignedShort, "SCALAR", g.indices.size(), false);
        return "{\"primitives\":[{\"attributes\":{" + attributes +
               "},\"indices\":" + std::to_string(indices) +
               (material >= 0 ? ",\"material\":" + std::to_string(material) : std::string()) +
               ",\"mode\":4}]}";
    }
};

// Mapbox's node matrix, column-major: diag(s, -s, 1) plus (tx, ty, 0), s from the mean-radius
// sphere at the node's latitude; `rotated` turns it 5 degrees about z, which keeps the diagonal
// within 0.4% of s, so only a rotation check can reject it.
inline std::string node_matrix(const LandmarkSpec& l, bool rotated) {
    const double s = 8192.0 * 16384.0 / (2.0 * kPi * 6371008.8 * std::cos(l.lat_deg * kPi / 180.0));
    const double c = rotated ? std::cos(kPi / 36.0) : 1.0, r = rotated ? std::sin(kPi / 36.0) : 0.0;
    const double m[16] = {s * c, s * r, 0, 0, s * r, -s * c, 0, 0, 0, 0, 1, 0, l.tx, l.ty, 0, 1};
    std::string out = "[";
    for (int k = 0; k < 16; ++k) out += (k == 0 ? "" : ",") + num(m[k]);
    return out + "]";
}

}

inline std::vector<uint8_t> build_landmark_glb(const std::vector<LandmarkSpec>& landmarks,
                                               const GlbOptions& opts) {
    using namespace detail;
    // EXT_meshopt_compression is specified on vertex codec v0, the only one Filament's older
    // meshoptimizer (first on the link line) decodes; meshoptimizer 1.1's encoder defaults to v1.
    meshopt_encodeVertexVersion(0);
    Writer w;
    w.opts = opts;
    w.corrupt_next_vertex_stream = opts.corrupt_meshopt;
    const size_t n = landmarks.size();

    // Meshes and nodes: buildings 0..n-1, footprints n..2n-1, LOD duplicates 2n..3n-1. The first
    // view written is building 0's POSITION, so the BIN chunk opens with a vertex stream.
    std::vector<std::string> meshes, nodes, materials, default_scene, lod_scene;
    for (size_t i = 0; i < n; ++i) {
        const LandmarkSpec& l = landmarks[i];
        meshes.push_back(w.mesh(box(l.half_x_m, l.half_y_m, l.height_m), static_cast<int>(i)));
    }
    for (size_t i = 0; i < n; ++i) {
        meshes.push_back(w.mesh(square(landmarks[i].footprint_half_m, 0.0, false), -1));
    }
    if (opts.lod_scene) {
        for (size_t i = 0; i < n; ++i) {
            const LandmarkSpec& l = landmarks[i];
            meshes.push_back(w.mesh(square(l.half_x_m, l.height_m, true), static_cast<int>(i)));
        }
    }
    for (size_t i = 0; i < n; ++i) {
        const LandmarkSpec& l = landmarks[i];
        const std::string id = "\"" + std::to_string(l.id) + "\"";
        nodes.push_back("{\"mesh\":" + std::to_string(i) +
                        ",\"matrix\":" + node_matrix(l, i == 0 && opts.bad_matrix_on_first) +
                        ",\"extras\":{\"id\":" + id + "}}");
        default_scene.push_back(std::to_string(i));
    }
    for (size_t i = 0; i < n; ++i) {
        const LandmarkSpec& l = landmarks[i];
        nodes.push_back("{\"mesh\":" + std::to_string(n + i) +
                        ",\"matrix\":" + node_matrix(l, false) +
                        ",\"extras\":{\"mapbox:footprint:id\":\"" + std::to_string(l.id) + "\"}}");
        default_scene.push_back(std::to_string(n + i));
    }
    if (opts.lod_scene) {
        for (size_t i = 0; i < n; ++i) {
            const LandmarkSpec& l = landmarks[i];
            nodes.push_back("{\"mesh\":" + std::to_string(2 * n + i) +
                            ",\"matrix\":" + node_matrix(l, false) + ",\"extras\":{\"id\":\"" +
                            std::to_string(l.id) + "\"}}");
            lod_scene.push_back(std::to_string(2 * n + i));
        }
    }
    for (size_t i = 0; i < n; ++i) {
        materials.push_back(
            std::string("{\"doubleSided\":true,\"pbrMetallicRoughness\":{\"metallicFactor\":0}") +
            (opts.occlusion_texture ? ",\"occlusionTexture\":{\"index\":0,\"extensions\":{"
                                      "\"KHR_texture_transform\":{\"scale\":[1,1]}}}"
                                    : "") +
            "}");
    }

    std::vector<std::string> used, required;
    if (opts.meshopt) {
        used.push_back("\"EXT_meshopt_compression\"");
        required.push_back("\"EXT_meshopt_compression\"");
    }
    if (opts.quantize) {
        used.push_back("\"KHR_mesh_quantization\"");
        required.push_back("\"KHR_mesh_quantization\"");
    }
    if (opts.occlusion_texture) used.push_back("\"KHR_texture_transform\"");

    // Mapbox writes no top-level "scene".
    std::string json = "{\"asset\":{\"version\":\"2.0\",\"generator\":\"overlume test fixture\"}";
    if (!used.empty()) json += ",\"extensionsUsed\":" + join(used);
    if (!required.empty()) json += ",\"extensionsRequired\":" + join(required);
    json += ",\"scenes\":[{\"name\":\"Default Scene\",\"nodes\":" + join(default_scene) + "}";
    if (opts.lod_scene) json += ",{\"name\":\"LOD\",\"nodes\":" + join(lod_scene) + "}";
    json += "],\"nodes\":" + join(nodes) + ",\"meshes\":" + join(meshes) +
            ",\"materials\":" + join(materials);
    if (opts.occlusion_texture) {
        // The JPEG goes last in the BIN chunk, after every compressed stream.
        const int view = w.image_view(occlusion_jpeg());
        json +=
            ",\"textures\":[{\"sampler\":0,\"source\":0}],\"samplers\":[{\"magFilter\":9729,"
            "\"minFilter\":9987}],\"images\":[{\"bufferView\":" +
            std::to_string(view) + ",\"mimeType\":\"image/jpeg\"}]";
    }
    json += ",\"accessors\":" + join(w.accessors) + ",\"bufferViews\":" + join(w.views) +
            ",\"buffers\":[{\"byteLength\":" + std::to_string(w.bin.size()) + "}";
    if (opts.meshopt) {
        json += ",{\"byteLength\":" + std::to_string(w.fallback_bytes) +
                ",\"extensions\":{\"EXT_meshopt_compression\":{\"fallback\":true}}}";
    }
    json += "]}";

    std::vector<uint8_t> bin = w.bin;
    pad(bin, 4);
    while (json.size() % 4 != 0) json.push_back(' ');
    std::vector<uint8_t> out;
    put_u32(out, 0x46546C67);  // "glTF"
    put_u32(out, 2);
    put_u32(out, static_cast<uint32_t>(12 + 8 + json.size() + 8 + bin.size()));
    put_u32(out, static_cast<uint32_t>(json.size()));
    put_u32(out, 0x4E4F534A);  // "JSON"
    out.insert(out.end(), json.begin(), json.end());
    put_u32(out, static_cast<uint32_t>(bin.size()));
    put_u32(out, 0x004E4942);  // "BIN"
    out.insert(out.end(), bin.begin(), bin.end());
    return out;
}

}
```

- [ ] **Step 4: Write the failing test** `overlume/tests/test_mapbox_landmarks.cpp`

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "mapbox_glb_fixture.hpp"
#include "mapbox_test_hooks.hpp"
#include "mapbox_tile_frame.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace {

namespace glb = overlume::testing::glb;
namespace mb = overlume::mapbox;
using overlume::testing::MapboxLandmarkProbe;

// The test site's z14 tile (test_mapbox_tile_frame.cpp pins it), anchored 12 m up.
constexpr uint32_t kX14 = 10707, kY14 = 7006;
constexpr double kOriginHeightM = 12.0;
// One fixture building after reading: a box of 24 vertices (per-face normals) and 36 u16
// indices, with POSITION, NORMAL and TEXCOORD_0 as floats (12 + 12 + 8 bytes per vertex).
constexpr uint64_t kBuildingBytes = 24 * (12 + 12 + 8) + 36 * 2;

glb::LandmarkSpec landmark(uint64_t id, double tx, double ty) {
    glb::LandmarkSpec l;
    l.id = id;
    l.tx = tx;
    l.ty = ty;
    l.lat_deg =
        mb::tile_point_to_lonlat({mb::kLandmarkZoom, kX14, kY14}, tx, ty, mb::kLandmarkExtent).lat;
    return l;
}

// Ids above 2^63, as Mapbox's are: they only survive as strings.
std::vector<glb::LandmarkSpec> two_landmarks() {
    return {landmark(18000000000000000001ull, 3000.0, 3000.0),
            landmark(18000000000000000002ull, 5200.0, 4800.0)};
}

MapboxLandmarkProbe probe(const std::vector<uint8_t>& bytes, bool keep_materials) {
    return overlume::testing::probe_mapbox_landmark_tile(bytes, kX14, kY14, kOriginHeightM,
                                                         keep_materials);
}

uint32_t u32_at(const std::vector<uint8_t>& b, size_t at) {
    return static_cast<uint32_t>(b[at]) | static_cast<uint32_t>(b[at + 1]) << 8 |
           static_cast<uint32_t>(b[at + 2]) << 16 | static_cast<uint32_t>(b[at + 3]) << 24;
}

// GLB layout: a 12-byte header, the JSON chunk (8-byte header + data), the BIN chunk.
size_t bin_chunk_header_at(const std::vector<uint8_t>& b) { return 20 + u32_at(b, 12); }

// The fixture with its JSON chunk edited (the fixture writes compact JSON, so substrings are
// exact) and the GLB lengths rewritten.
std::vector<uint8_t> with_json_edits(
    const std::vector<uint8_t>& b, const std::vector<std::pair<std::string, std::string>>& edits) {
    std::string json(b.begin() + 20, b.begin() + bin_chunk_header_at(b));
    for (const auto& [from, to] : edits) {
        const size_t at = json.find(from);
        EXPECT_NE(at, std::string::npos) << from;
        if (at != std::string::npos) json.replace(at, from.size(), to);
    }
    while (json.size() % 4 != 0) json.push_back(' ');
    const std::vector<uint8_t> bin(b.begin() + bin_chunk_header_at(b), b.end());
    std::vector<uint8_t> out;
    const auto put = [&out](size_t v) {
        for (int i = 0; i < 4; ++i) out.push_back(static_cast<uint8_t>(v >> (8 * i)));
    };
    put(0x46546C67);  // "glTF"
    put(2);
    put(12 + 8 + json.size() + bin.size());
    put(json.size());
    put(0x4E4F534A);  // "JSON"
    out.insert(out.end(), json.begin(), json.end());
    out.insert(out.end(), bin.begin(), bin.end());
    return out;
}

// The fixture with EXT_meshopt_compression renamed KHR_meshopt_compression throughout, which
// cesium 0.64 does not decode. The names are the same length, so the GLB keeps its layout.
std::vector<uint8_t> with_khr_meshopt(std::vector<uint8_t> b) {
    const std::string from = "EXT_meshopt_compression", to = "KHR_meshopt_compression";
    std::string json(b.begin() + 20, b.begin() + bin_chunk_header_at(b));
    for (size_t at = json.find(from); at != std::string::npos; at = json.find(from, at)) {
        json.replace(at, from.size(), to);
    }
    std::copy(json.begin(), json.end(), b.begin() + 20);
    return b;
}

// Every view in buffer 0 as written: no meshopt, no quantization.
glb::GlbOptions plain_options() {
    glb::GlbOptions opts;
    opts.meshopt = false;
    opts.quantize = false;
    return opts;
}

}

TEST(MapboxLandmarks, FixtureVertexStreamsUseCodecV0) {
    const std::vector<uint8_t> bytes = glb::build_landmark_glb(two_landmarks(), {});
    // The BIN chunk opens with building 0's POSITION stream.
    const size_t first = bin_chunk_header_at(bytes) + 8;
    ASSERT_GT(bytes.size(), first);
    EXPECT_EQ(static_cast<int>(bytes[first]), 0xa0)
        << "vertex codec v0, the version EXT_meshopt_compression specifies";
}

TEST(MapboxLandmarks, ClayKeepsOnlyTheDefaultSceneBuildings) {
    const MapboxLandmarkProbe p = probe(glb::build_landmark_glb(two_landmarks(), {}), false);
    ASSERT_TRUE(p.ok) << p.error;
    EXPECT_EQ(p.buildings_kept, 2u);
    EXPECT_EQ(p.nodes_skipped, 0u);
    EXPECT_EQ(p.scenes, 1u) << "the LOD scene goes";
    EXPECT_EQ(p.nodes, 2u) << "footprint and LOD nodes go";
    EXPECT_EQ(p.meshes, 2u);
    EXPECT_EQ(p.materials, 2u) << "materials stay; clay replaces them at upload";
    EXPECT_EQ(p.images, 0u);
    EXPECT_EQ(p.textures, 0u);
    EXPECT_EQ(p.samplers, 0u);
    // Per building: POSITION, NORMAL, TEXCOORD_0 and indices, each in its own view and buffer.
    EXPECT_EQ(p.accessors, 8u);
    EXPECT_EQ(p.buffer_views, 8u);
    EXPECT_EQ(p.buffers, 8u);
    EXPECT_EQ(p.footprint_triangles, 4u);
    EXPECT_FALSE(p.meshopt_extension_left);
    EXPECT_EQ(p.buffer_bytes, 2 * kBuildingBytes)
        << "only the buildings' decoded streams remain: no compressed source, fallback, "
           "quantized, footprint, LOD or image bytes";
}

TEST(MapboxLandmarks, OriginalKeepsTheOcclusionTextureAndCompactsBuffers) {
    const std::vector<uint8_t> bytes = glb::build_landmark_glb(two_landmarks(), {});
    const MapboxLandmarkProbe clay = probe(bytes, false);
    const MapboxLandmarkProbe original = probe(bytes, true);
    ASSERT_TRUE(original.ok) << original.error;
    EXPECT_EQ(original.buildings_kept, 2u);
    EXPECT_EQ(original.images, 1u);
    EXPECT_EQ(original.textures, 1u);
    EXPECT_EQ(original.samplers, 1u);
    EXPECT_EQ(original.materials, 2u);
    EXPECT_EQ(original.buffer_views, clay.buffer_views + 1) << "plus the JPEG's view";
    EXPECT_EQ(original.buffers, clay.buffers + 1) << "plus buffer 0, cut down to the JPEG";
    const uint64_t buffer0 = original.buffer_bytes - clay.buffer_bytes;
    const uint64_t jpeg = glb::detail::occlusion_jpeg().size();
    EXPECT_GE(buffer0, jpeg) << "buffer 0 keeps the JPEG";
    EXPECT_LE(buffer0, jpeg + 7)
        << "and is cut down to it: compactBuffers shrinks the gap ahead in multiples of 8 bytes";
}

TEST(MapboxLandmarks, KeptNodesPlaceTheirAnchorInTheTileFrame) {
    const std::vector<glb::LandmarkSpec> specs = two_landmarks();
    const MapboxLandmarkProbe p = probe(glb::build_landmark_glb(specs, {}), false);
    ASSERT_TRUE(p.ok) << p.error;
    ASSERT_EQ(p.node_matrices.size(), specs.size());
    const mb::TileId t{mb::kLandmarkZoom, kX14, kY14};
    const mb::TileFrame f = mb::make_tile_frame(t, kOriginHeightM);
    for (size_t i = 0; i < specs.size(); ++i) {
        const std::array<double, 16>& m = p.node_matrices[i];
        const std::array<double, 3> want =
            mb::project_to_frame(f, t, specs[i].tx, specs[i].ty, mb::kLandmarkExtent, 0.0);
        EXPECT_NEAR(m[12], want[0], 0.01) << "node " << i;
        EXPECT_NEAR(m[13], want[1], 0.01) << "node " << i;
        EXPECT_NEAR(m[14], want[2], 0.01) << "node " << i;
        EXPECT_EQ(m[15], 1.0) << "node " << i;
    }
}

TEST(MapboxLandmarks, NodeWithUnexpectedMatrixIsSkipped) {
    glb::GlbOptions opts;
    opts.bad_matrix_on_first = true;
    const MapboxLandmarkProbe p = probe(glb::build_landmark_glb(two_landmarks(), opts), false);
    ASSERT_TRUE(p.ok) << p.error;
    EXPECT_EQ(p.nodes_skipped, 1u);
    EXPECT_EQ(p.buildings_kept, 1u);
    EXPECT_EQ(p.nodes, 1u);
    EXPECT_EQ(p.meshes, 1u);
    EXPECT_EQ(p.footprint_triangles, 4u) << "the footprints keep their own, valid matrices";
}

TEST(MapboxLandmarks, CorruptMeshoptStreamFailsTheWholeTile) {
    glb::GlbOptions opts;
    opts.corrupt_meshopt = true;
    const MapboxLandmarkProbe p = probe(glb::build_landmark_glb(two_landmarks(), opts), false);
    EXPECT_FALSE(p.ok);
    EXPECT_NE(p.error.find("meshopt"), std::string::npos) << p.error;
    EXPECT_EQ(p.nodes, 0u) << "no half-decoded landmark reaches the renderer";
    EXPECT_EQ(p.meshes, 0u);
    EXPECT_EQ(p.buffers, 0u);
    EXPECT_EQ(p.footprint_triangles, 0u);
}

// Spec section 12: a switch to KHR_meshopt_compression shows as zero landmarks, not a crash.
TEST(MapboxLandmarks, KhrMeshoptCompressionFailsCleanly) {
    const MapboxLandmarkProbe p =
        probe(with_khr_meshopt(glb::build_landmark_glb(two_landmarks(), {})), false);
    EXPECT_FALSE(p.ok);
    EXPECT_NE(p.error.find("KHR_meshopt_compression"), std::string::npos) << p.error;
    EXPECT_EQ(p.nodes, 0u);
    EXPECT_EQ(p.buffers, 0u);
    EXPECT_EQ(p.footprint_triangles, 0u);
}

// Inputs that pass the reader and the extension checks, then fail the pruned model's checks.
TEST(MapboxLandmarks, InconsistentKeptModelFailsCleanly) {
    struct Case {
        std::vector<uint8_t> glb;
        bool keep_materials;
        const char* what;
    };
    const Case cases[] = {
        {with_json_edits(with_khr_meshopt(glb::build_landmark_glb(two_landmarks(), {})),
                         {{"\"extensionsRequired\":[\"KHR_meshopt_compression\",",
                           "\"extensionsRequired\":["}}),
         false, "KHR_meshopt_compression used but not required: views in the empty fallback"},
        {with_json_edits(
             glb::build_landmark_glb(two_landmarks(), plain_options()),
             {{"\"byteOffset\":0,\"byteLength\":", "\"byteOffset\":0,\"byteLength\":9"}}),
         false, "a bufferView running past the end of buffer 0"},
        {with_json_edits(
             glb::build_landmark_glb(two_landmarks(), {}),
             {{"\"occlusionTexture\":{\"index\":0", "\"occlusionTexture\":{\"index\":1"}}),
         true, "a material naming texture 1 of 1"},
    };
    for (const Case& c : cases) {
        const MapboxLandmarkProbe p = probe(c.glb, c.keep_materials);
        EXPECT_FALSE(p.ok) << c.what;
        EXPECT_FALSE(p.error.empty()) << c.what;
        EXPECT_EQ(p.nodes, 0u) << c.what;
        EXPECT_EQ(p.footprint_triangles, 0u) << c.what;
    }
}

TEST(MapboxLandmarks, NonGlbAndTruncatedInputFailCleanly) {
    const std::vector<uint8_t> good = glb::build_landmark_glb(two_landmarks(), {});
    const std::string html = "<html><body>502 Bad Gateway</body></html>";
    const std::string json = "{\"message\":\"not a tile\"}";
    std::vector<uint8_t> version3 = good;  // a GLB header with an unknown container version
    version3[4] = 3;
    const std::vector<std::vector<uint8_t>> inputs = {
        {},
        std::vector<uint8_t>(html.begin(), html.end()),
        std::vector<uint8_t>(json.begin(), json.end()),
        version3,
        std::vector<uint8_t>(good.begin(), good.begin() + 30),
        std::vector<uint8_t>(good.begin(), good.begin() + good.size() / 2),
    };
    for (const std::vector<uint8_t>& input : inputs) {
        const MapboxLandmarkProbe p = probe(input, false);
        EXPECT_FALSE(p.ok) << input.size() << " bytes";
        EXPECT_FALSE(p.error.empty()) << input.size() << " bytes";
        EXPECT_EQ(p.nodes, 0u) << input.size() << " bytes";
    }

    // Buffer 0 claiming more bytes than the BIN chunk holds: cesium returns the model and an
    // error, and the error alone fails the tile.
    const MapboxLandmarkProbe inflated = probe(
        with_json_edits(glb::build_landmark_glb(two_landmarks(), plain_options()),
                        {{"\"buffers\":[{\"byteLength\":", "\"buffers\":[{\"byteLength\":9"}}),
        false);
    EXPECT_FALSE(inflated.ok);
    EXPECT_NE(inflated.error.find("binary chunk"), std::string::npos) << inflated.error;
    EXPECT_EQ(inflated.nodes, 0u);
}

TEST(MapboxLandmarks, SceneIsDefaultSceneThenSceneThenFirst) {
    const std::vector<uint8_t> base = glb::build_landmark_glb(two_landmarks(), {});
    // Scene 0 holds the buildings and their footprints; scene 1 the LOD nodes, no footprints.
    const MapboxLandmarkProbe unnamed = probe(
        with_json_edits(base, {{"\"name\":\"Default Scene\",", ""}, {"\"name\":\"LOD\",", ""}}),
        false);
    ASSERT_TRUE(unnamed.ok) << unnamed.error;
    EXPECT_EQ(unnamed.buildings_kept, 2u) << "no names, no scene: scene 0";
    EXPECT_EQ(unnamed.footprint_triangles, 4u) << "no names, no scene: scene 0";

    const MapboxLandmarkProbe renamed =
        probe(with_json_edits(base, {{"\"name\":\"Default Scene\"", "\"name\":\"Other\""},
                                     {"\"name\":\"LOD\"", "\"name\":\"Default Scene\""}}),
              false);
    ASSERT_TRUE(renamed.ok) << renamed.error;
    EXPECT_EQ(renamed.buildings_kept, 2u) << "\"Default Scene\" wins at index 1";
    EXPECT_EQ(renamed.footprint_triangles, 0u) << "\"Default Scene\" wins at index 1";

    const MapboxLandmarkProbe pointed =
        probe(with_json_edits(base, {{"\"name\":\"Default Scene\",", ""},
                                     {"\"name\":\"LOD\",", ""},
                                     {"{\"asset\"", "{\"scene\":1,\"asset\""}}),
              false);
    ASSERT_TRUE(pointed.ok) << pointed.error;
    EXPECT_EQ(pointed.buildings_kept, 2u) << "no names: the top-level scene, 1";
    EXPECT_EQ(pointed.footprint_triangles, 0u) << "no names: the top-level scene, 1";
}

TEST(MapboxLandmarks, UncompressedUnquantizedVariantReadsTheSame) {
    const MapboxLandmarkProbe a = probe(glb::build_landmark_glb(two_landmarks(), {}), false);
    const MapboxLandmarkProbe b =
        probe(glb::build_landmark_glb(two_landmarks(), plain_options()), false);
    ASSERT_TRUE(a.ok) << a.error;
    ASSERT_TRUE(b.ok) << b.error;
    EXPECT_EQ(b.buildings_kept, a.buildings_kept);
    EXPECT_EQ(b.scenes, a.scenes);
    EXPECT_EQ(b.nodes, a.nodes);
    EXPECT_EQ(b.meshes, a.meshes);
    EXPECT_EQ(b.accessors, a.accessors);
    EXPECT_EQ(b.materials, a.materials);
    EXPECT_EQ(b.images, a.images);
    EXPECT_EQ(b.footprint_triangles, a.footprint_triangles);
    EXPECT_FALSE(b.meshopt_extension_left);
    ASSERT_EQ(b.node_matrices.size(), a.node_matrices.size());
    for (size_t i = 0; i < a.node_matrices.size(); ++i) {
        for (size_t k = 0; k < 16; ++k) {
            EXPECT_NEAR(b.node_matrices[i][k], a.node_matrices[i][k], 1e-9) << i << "," << k;
        }
    }
}

TEST(MapboxLandmarks, InsideAnyFootprintTruthTable) {
    using Triangles = std::vector<std::array<double, 6>>;
    const std::array<double, 6> ccw{0, 0, 10, 0, 0, 10};
    const std::array<double, 6> cw{0, 0, 0, 10, 10, 0};
    const std::array<double, 6> distant{100, 100, 110, 100, 100, 110};
    const std::array<double, 6> point{5, 5, 5, 5, 5, 5};
    struct Case {
        Triangles triangles;
        double x, y;
        bool inside;
        const char* what;
    };
    const Case cases[] = {
        {{ccw}, 2, 2, true, "inside a counter-clockwise triangle"},
        {{cw}, 2, 2, true, "inside a clockwise triangle"},
        {{ccw}, 6, 6, false, "beyond the hypotenuse"},
        {{ccw}, -1, 2, false, "west of the triangle"},
        {{ccw}, 5, 0, true, "on an edge"},
        {{ccw}, 5, 5, true, "on the edge two triangles of a square share"},
        {{ccw}, 0, 0, true, "on a corner"},
        {{ccw, distant}, 102, 102, true, "inside the second triangle"},
        {{ccw, distant}, 50, 50, false, "between the triangles"},
        {{point}, 5, 5, false, "a degenerate triangle covers nothing"},
        {{point}, 70, -3, false, "a degenerate triangle covers nothing"},
        {{}, 0, 0, false, "no footprints"},
    };
    for (const Case& c : cases) {
        EXPECT_EQ(overlume::testing::mapbox_inside_any_footprint_probe(c.triangles, c.x, c.y),
                  c.inside)
            << c.what;
    }
}
```

What each test catches when its behaviour is reverted:
- `FixtureVertexStreamsUseCodecV0`: a fixture that writes the JPEG ahead of the streams, or one
  that drops `meshopt_encodeVertexVersion(0)` once vcpkg's 1.1 encoder resolves first (while
  Filament's older encoder resolves first it writes v0 regardless).
- `ClayKeepsOnlyTheDefaultSceneBuildings`: keeping the LOD scene (scenes 2, nodes 4), keeping
  footprint nodes (nodes and meshes 4), keeping textures in clay mode (images 1), skipping any
  `removeUnused*` pass (accessor, view or buffer counts and bytes grow: the compressed BIN chunk,
  the quantized streams, the footprint and LOD streams; without `removeUnusedBuffers` the empty
  fallback buffer fails `kept_model_problem`), `decodeMeshOptData = false` (ok = false, the views
  keep the extension) and `dequantizeMeshData = false` (ok = false, `KHR_mesh_quantization`
  stays required). Dropping `material.occlusionTexture.reset()` fails it too: the materials then
  name a texture the model no longer has, and `kept_model_problem` fails the tile (six tests
  fail). The exact byte count replaces the brief's "below the GLB size minus the JPEG", which
  passes even with the compressed buffer kept,
  because the fixture's JSON chunk outweighs the JPEG and the decoded bytes.
- `OriginalKeepsTheOcclusionTextureAndCompactsBuffers`: dropping textures in original mode, or
  running `removeUnusedTextures` there (it deletes the occlusion-only texture, and the material
  is left naming it), and skipping `compactBuffers`: buffer 0 then keeps the whole BIN chunk,
  1579 bytes against the bound of the 651-byte JPEG plus 7.
- `KeptNodesPlaceTheirAnchorInTheTileFrame`: leaving Mapbox's tile-unit matrix on the node
  (translation 3000, 3000 instead of frame metres), or keeping nodes out of scene order.
- `NodeWithUnexpectedMatrixIsSkipped`: dropping the `landmark_node_to_frame` validation (2 kept,
  0 skipped). The fixture's bad matrix is a 5° rotation, whose diagonal stays inside the 1% scale
  band, so reducing the off-diagonal loop to `{14}` fails this test too.
- `CorruptMeshoptStreamFailsTheWholeTile` (Review Focus 4): dropping the leftover-extension check
  (building 0's POSITION then still points into the empty fallback buffer, and the tile fails
  later, in `kept_model_problem`, with an error that does not name meshopt).
- `KhrMeshoptCompressionFailsCleanly` (spec §12, Review Focus 4): dropping the
  `extensionsRequired` check (the tile then fails in `kept_model_problem`, with an error that
  does not name the extension). Without both checks, `compactBuffers` segfaults on this input
  (exit 139).
- `InconsistentKeptModelFailsCleanly`: dropping any one of `kept_model_problem`'s three checks.
  Without the buffer check, the first input segfaults in `compactBuffers` (exit 139). Without the
  view check, the second reads as ok with a view past its buffer. Without the texture check, the
  third reads as ok naming a texture the model does not have.
- `NonGlbAndTruncatedInputFailCleanly` (Review Focus 3, reader side): dropping the magic check
  (the JSON body reads as an empty tile with ok = true) and treating reader errors as warnings.
  For the last input cesium returns the model and an error; without the errors check it fails
  later, in `kept_model_problem`, with an error that does not name the binary chunk.
- `SceneIsDefaultSceneThenSceneThenFirst`: always taking scene 0 (the renamed case keeps 4
  footprint triangles), ignoring the top-level `scene` (the pointed case keeps 4), and requiring
  the name (the unnamed case keeps no buildings).
- `UncompressedUnquantizedVariantReadsTheSame`: code that assumes the fallback buffer or quantized
  attributes exist.
- `InsideAnyFootprintTruthTable`: a test of one winding only (the clockwise case), an exclusive edge
  test (the edge, corner and diagonal cases; a centroid on a square's diagonal must count), checking
  only the first triangle, and dropping the degenerate-triangle guard (a triangle collapsed to a
  point has all three edge functions zero and would claim every point).

- [ ] **Step 5: Run it and see it fail**

```bash
cd "$WORKTREE"
flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock cmake -S overlume -B overlume/build > /dev/null \
    && echo reconfigured \
    && CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
        --target test_mapbox_landmarks
```

No `reconfigured` line and no CMake error above it means the lock wait ran out (exit 75) and nothing
was built: run the step again.

Expected: `reconfigured`, then the link of `test_mapbox_landmarks` fails with
``undefined reference to `overlume::testing::probe_mapbox_landmark_tile(`` and
``undefined reference to `overlume::testing::mapbox_inside_any_footprint_probe(`` followed by
`linker command failed with exit code 1`; the build exits non-zero. (The fixture's meshoptimizer
and stb calls link: only the two hooks are missing.)

- [ ] **Step 6: Implement** `overlume/src/mapbox_landmarks.cpp`

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "mapbox_landmarks.hpp"

#include "mapbox_test_hooks.hpp"

#include <CesiumGltf/AccessorUtility.h>
#include <CesiumGltf/AccessorView.h>
#include <CesiumGltf/ExtensionBufferExtMeshoptCompression.h>
#include <CesiumGltf/ExtensionBufferViewExtMeshoptCompression.h>
#include <CesiumGltfContent/GltfUtilities.h>
#include <CesiumGltfReader/GltfReader.h>

#include <glm/vec3.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <optional>
#include <type_traits>
#include <variant>

namespace overlume::mapbox {

namespace {

using CesiumGltfContent::GltfUtilities;

constexpr char kFootprintIdKey[] = "mapbox:footprint:id";

// GL JS's rule: the scene named "Default Scene", else `scene`, else the first.
const CesiumGltf::Scene* choose_scene(const CesiumGltf::Model& model) {
    for (const CesiumGltf::Scene& scene : model.scenes) {
        if (scene.name == "Default Scene") return &scene;
    }
    const CesiumGltf::Scene* pointed = CesiumGltf::Model::getSafe(&model.scenes, model.scene);
    return pointed != nullptr ? pointed : CesiumGltf::Model::getSafe(&model.scenes, 0);
}

// Appends the footprint mesh's triangles, mapped by `to_frame`, as tile-frame x, y.
void append_footprint_triangles(const CesiumGltf::Model& model, const CesiumGltf::Mesh& mesh,
                                const Mat4d& to_frame, std::vector<std::array<double, 6>>& out) {
    for (const CesiumGltf::MeshPrimitive& prim : mesh.primitives) {
        const auto position = prim.attributes.find("POSITION");
        if (prim.mode != CesiumGltf::MeshPrimitive::Mode::TRIANGLES ||
            position == prim.attributes.end()) {
            continue;
        }
        const CesiumGltf::AccessorView<glm::vec3> positions(model, position->second);
        if (positions.status() != CesiumGltf::AccessorViewStatus::Valid) continue;
        std::vector<int64_t> corners;
        std::visit(
            [&](const auto& indices) {
                if constexpr (std::is_same_v<std::decay_t<decltype(indices)>, std::monostate>) {
                    for (int64_t i = 0; i < positions.size(); ++i) corners.push_back(i);
                } else if (indices.status() == CesiumGltf::AccessorViewStatus::Valid) {
                    for (int64_t i = 0; i < indices.size(); ++i) {
                        corners.push_back(static_cast<int64_t>(indices[i]));
                    }
                }
            },
            CesiumGltf::getIndexAccessorView(model, prim));
        for (size_t i = 0; i + 2 < corners.size(); i += 3) {
            std::array<double, 6> triangle{};
            bool valid = true;
            for (size_t k = 0; k < 3; ++k) {
                const int64_t c = corners[i + k];
                if (c < 0 || c >= positions.size()) {
                    valid = false;
                    break;
                }
                const glm::vec3& p = positions[c];
                triangle[2 * k] =
                    to_frame[0] * p.x + to_frame[4] * p.y + to_frame[8] * p.z + to_frame[12];
                triangle[2 * k + 1] =
                    to_frame[1] * p.x + to_frame[5] * p.y + to_frame[9] * p.z + to_frame[13];
            }
            if (valid) out.push_back(triangle);
        }
    }
}

// What the pruned model must hold before compactBuffers, which erases byte ranges unchecked (its
// asserts are compiled out), and before gltfio, whose cgltf rejects the whole tile's GLB over one
// dangling index. Clay mode's texture pruning is where this reader could leave one; other indices
// are not checked.
const char* kept_model_problem(const CesiumGltf::Model& model) {
    for (const CesiumGltf::Buffer& buffer : model.buffers) {
        if (buffer.byteLength != static_cast<int64_t>(buffer.cesium.data.size())) {
            return "a kept buffer does not hold its bytes";
        }
    }
    for (const CesiumGltf::BufferView& view : model.bufferViews) {
        const CesiumGltf::Buffer* buffer = CesiumGltf::Model::getSafe(&model.buffers, view.buffer);
        if (buffer == nullptr || view.byteOffset < 0 || view.byteLength < 0 ||
            view.byteOffset > buffer->byteLength ||
            view.byteLength > buffer->byteLength - view.byteOffset) {
            return "a kept bufferView lies outside its buffer";
        }
    }
    const auto missing = [&model](const auto& slot) {
        return slot.has_value() &&
               CesiumGltf::Model::getSafe(&model.textures, slot->index) == nullptr;
    };
    for (const CesiumGltf::Material& m : model.materials) {
        const auto& pbr = m.pbrMetallicRoughness;
        if (missing(m.occlusionTexture) || missing(m.normalTexture) || missing(m.emissiveTexture) ||
            (pbr && (missing(pbr->baseColorTexture) || missing(pbr->metallicRoughnessTexture)))) {
            return "a material names a texture the model does not have";
        }
    }
    return nullptr;
}

}

LandmarkTile read_landmark_tile(std::span<const std::byte> glb, const TileFrame& frame,
                                bool keep_materials) {
    LandmarkTile out;
    // A 200 can still carry an HTML or JSON body; cesium would read JSON as an empty glTF.
    if (glb.size() < 4 || std::memcmp(glb.data(), "glTF", 4) != 0) {
        out.error = "not a GLB (no glTF magic)";
        return out;
    }
    CesiumGltfReader::GltfReaderOptions options;
    // Filament decodes the JPEGs itself; cesium's decoded copy would be thrown away.
    options.decodeEmbeddedImages = false;
    static const CesiumGltfReader::GltfReader reader;
    CesiumGltfReader::GltfReaderResult read = reader.readGltf(glb, options);
    if (!read.model || !read.errors.empty()) {
        out.error = read.errors.empty() ? std::string("GLB without a model") : read.errors.front();
        return out;
    }
    CesiumGltf::Model& model = *read.model;
    // cesium drops the extension names even when a stream fails to decode; the failed view keeps
    // its extension and still points at the fallback buffer, which has no bytes.
    for (const CesiumGltf::BufferView& view : model.bufferViews) {
        if (view.hasExtension<CesiumGltf::ExtensionBufferViewExtMeshoptCompression>()) {
            out.error = "EXT_meshopt_compression: a compressed stream did not decode";
            return out;
        }
    }
    // cesium's decoders take the extension they handle off extensionsRequired; a name left is one
    // it does not decode, such as KHR_meshopt_compression, whose views would still point into the
    // empty fallback buffer.
    if (!model.extensionsRequired.empty()) {
        out.error =
            "GLB requires " + model.extensionsRequired.front() + ", which the reader cannot decode";
        return out;
    }

    const CesiumGltf::Scene* scene = choose_scene(model);
    const std::vector<int32_t> roots = scene != nullptr ? scene->nodes : std::vector<int32_t>{};
    std::vector<CesiumGltf::Node> buildings;
    // Only scene roots are walked: Mapbox's landmark nodes have no children.
    for (const int32_t index : roots) {
        const CesiumGltf::Node* node = CesiumGltf::Model::getSafe(&model.nodes, index);
        const CesiumGltf::Mesh* mesh =
            node != nullptr ? CesiumGltf::Model::getSafe(&model.meshes, node->mesh) : nullptr;
        if (mesh == nullptr) continue;
        std::optional<Mat4d> to_frame;
        if (node->matrix.size() == 16) {
            Mat4d m{};
            std::copy(node->matrix.begin(), node->matrix.end(), m.begin());
            to_frame = landmark_node_to_frame(frame, m);
        }
        if (!to_frame) {
            ++out.nodes_skipped;
            continue;
        }
        if (node->extras.count(kFootprintIdKey) != 0) {
            append_footprint_triangles(model, *mesh, *to_frame, out.footprint_triangles);
            continue;
        }
        CesiumGltf::Node& kept = buildings.emplace_back();
        kept.mesh = node->mesh;
        kept.matrix.assign(to_frame->begin(), to_frame->end());
    }
    out.buildings_kept = static_cast<uint32_t>(buildings.size());

    // One scene of the kept buildings, then everything only the dropped nodes used goes.
    model.nodes = std::move(buildings);
    model.scenes.assign(1, CesiumGltf::Scene{});
    model.scenes[0].name = "Default Scene";
    for (int32_t i = 0; i < static_cast<int32_t>(model.nodes.size()); ++i) {
        model.scenes[0].nodes.push_back(i);
    }
    model.scene = 0;
    model.skins.clear();  // skins and animations name nodes by index
    model.animations.clear();
    if (!keep_materials) {
        // Clay replaces every material at upload. removeUnusedTextures does not visit
        // occlusionTexture, Mapbox's only texture slot, so the references go first (and original
        // mode, which keeps them, must not call it).
        for (CesiumGltf::Material& material : model.materials) {
            material.occlusionTexture.reset();
            material.normalTexture.reset();
            material.emissiveTexture.reset();
            if (material.pbrMetallicRoughness) {
                material.pbrMetallicRoughness->baseColorTexture.reset();
                material.pbrMetallicRoughness->metallicRoughnessTexture.reset();
            }
        }
        GltfUtilities::removeUnusedTextures(model);
        GltfUtilities::removeUnusedSamplers(model);
        GltfUtilities::removeUnusedImages(model);
    }
    GltfUtilities::removeUnusedMeshes(model);
    GltfUtilities::removeUnusedAccessors(model);
    GltfUtilities::removeUnusedBufferViews(model);
    // Before compactBuffers: the meshopt fallback buffer has a byteLength but no bytes, which
    // compactBuffer cannot shrink.
    GltfUtilities::removeUnusedBuffers(model);
    if (const char* problem = kept_model_problem(model)) return LandmarkTile{.error = problem};
    GltfUtilities::compactBuffers(model);

    out.model = std::move(model);
    out.ok = true;
    return out;
}

bool inside_any_footprint(const std::vector<std::array<double, 6>>& triangles, double x, double y) {
    for (const std::array<double, 6>& t : triangles) {
        const double area2 = (t[2] - t[0]) * (t[5] - t[1]) - (t[3] - t[1]) * (t[4] - t[0]);
        if (!(std::abs(area2) > 0.0)) continue;  // a degenerate triangle covers nothing
        // Edge functions of either winding; zero on an edge, so a point on the diagonal of a
        // footprint split in two counts as inside.
        const double e0 = (t[2] - t[0]) * (y - t[1]) - (t[3] - t[1]) * (x - t[0]);
        const double e1 = (t[4] - t[2]) * (y - t[3]) - (t[5] - t[3]) * (x - t[2]);
        const double e2 = (t[0] - t[4]) * (y - t[5]) - (t[1] - t[5]) * (x - t[4]);
        if ((e0 >= 0.0 && e1 >= 0.0 && e2 >= 0.0) || (e0 <= 0.0 && e1 <= 0.0 && e2 <= 0.0)) {
            return true;
        }
    }
    return false;
}

}

namespace overlume::testing {

MapboxLandmarkProbe probe_mapbox_landmark_tile(const std::vector<uint8_t>& glb, uint32_t x14,
                                               uint32_t y14, double origin_height_m,
                                               bool keep_materials) {
    namespace mb = overlume::mapbox;
    const mb::TileFrame frame = mb::make_tile_frame({mb::kLandmarkZoom, x14, y14}, origin_height_m);
    const mb::LandmarkTile tile =
        mb::read_landmark_tile(std::as_bytes(std::span(glb)), frame, keep_materials);
    const CesiumGltf::Model& m = tile.model;
    const auto count = [](const auto& items) { return static_cast<uint32_t>(items.size()); };
    MapboxLandmarkProbe p;
    p.ok = tile.ok;
    p.error = tile.error;
    p.buildings_kept = tile.buildings_kept;
    p.nodes_skipped = tile.nodes_skipped;
    p.scenes = count(m.scenes);
    p.nodes = count(m.nodes);
    p.meshes = count(m.meshes);
    p.accessors = count(m.accessors);
    p.buffer_views = count(m.bufferViews);
    p.buffers = count(m.buffers);
    p.images = count(m.images);
    p.textures = count(m.textures);
    p.samplers = count(m.samplers);
    p.materials = count(m.materials);
    p.footprint_triangles = count(tile.footprint_triangles);
    p.meshopt_extension_left = m.isExtensionUsed("EXT_meshopt_compression") ||
                               m.isExtensionRequired("EXT_meshopt_compression");
    for (const CesiumGltf::Buffer& buffer : m.buffers) {
        p.buffer_bytes += buffer.cesium.data.size();
        p.meshopt_extension_left =
            p.meshopt_extension_left ||
            buffer.hasExtension<CesiumGltf::ExtensionBufferExtMeshoptCompression>();
    }
    for (const CesiumGltf::BufferView& view : m.bufferViews) {
        p.meshopt_extension_left =
            p.meshopt_extension_left ||
            view.hasExtension<CesiumGltf::ExtensionBufferViewExtMeshoptCompression>();
    }
    for (const CesiumGltf::Node& node : m.nodes) {
        std::array<double, 16> matrix{};
        std::copy_n(node.matrix.begin(), std::min<size_t>(16, node.matrix.size()), matrix.begin());
        p.node_matrices.push_back(matrix);
    }
    return p;
}

bool mapbox_inside_any_footprint_probe(const std::vector<std::array<double, 6>>& triangles,
                                       double x, double y) {
    return overlume::mapbox::inside_any_footprint(triangles, x, y);
}

}
```

- [ ] **Step 7: Run it and see it pass**

```bash
cd "$WORKTREE"
flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock cmake -S overlume -B overlume/build > /dev/null \
    && echo reconfigured \
    && CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
        --target test_mapbox_landmarks test_mapbox_tile_frame \
    && overlume/build/test_mapbox_landmarks && overlume/build/test_mapbox_tile_frame
```

No `reconfigured` line and no CMake error above it means the lock wait ran out (exit 75) and nothing
was built: run the step again.

Expected: `reconfigured`, a clean build, `[  PASSED  ] 12 tests.` and then `[  PASSED  ] 7 tests.`

- [ ] **Step 8: Format**

```bash
cd "$WORKTREE"
git add overlume/src/mapbox_landmarks.hpp overlume/src/mapbox_landmarks.cpp \
    overlume/src/mapbox_test_hooks.hpp overlume/tests/mapbox_glb_fixture.hpp \
    overlume/tests/test_mapbox_landmarks.cpp
tools/check_format.sh --fix overlume/src/mapbox_landmarks.hpp overlume/src/mapbox_landmarks.cpp \
    overlume/src/mapbox_test_hooks.hpp overlume/tests/mapbox_glb_fixture.hpp \
    overlume/tests/test_mapbox_landmarks.cpp
git diff --stat
tools/check_format.sh && tools/check_spdx.sh
```

Expected: `check_format.sh: reformatted 5 files with clang-format 23.1.1`, an empty `git diff --stat`,
`check_format.sh: PASS (… files, clang-format 23.1.1)` and `check_spdx.sh: PASS (… files carry the
SPDX identifier)`.

- [ ] **Step 9: Gate** (foreground, Bash timeout 600000 ms)

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
ROS_DOMAIN_ID=83 CI_VISUAL_MODE_DOMAIN_ID=83 \
    flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock tools/ci_visual_mode.sh
```

Exit 75 means another lane held the gate lock for 300 s: run the same command again. Any other
non-zero exit is a gate failure.

Expected: six `PASS` lines (POD header check, library ctest suite, node gtests, WS bridge pytest
suite, golden suite, examples), `OVERALL: PASS`, exit 0; the library ctest count is 12 higher than
after Task 4.

- [ ] **Step 10: Commit**

```bash
cd "$WORKTREE"
git add overlume/src/mapbox_landmarks.hpp overlume/src/mapbox_landmarks.cpp \
    overlume/src/mapbox_test_hooks.hpp overlume/tests/mapbox_glb_fixture.hpp \
    overlume/tests/test_mapbox_landmarks.cpp
git commit -F - <<'EOF'
feat(mapbox): read landmark GLBs into placed building nodes

read_landmark_tile turns one Mapbox landmark GLB into what the loader merges
(spec section 5, Building one z14 tile): cesium decodes meshopt and
quantization, images stay encoded for Filament, the "Default Scene" building
nodes are kept with their matrices replaced by the tile-frame map, footprint
triangles are collected for conflation, and footprints, the LOD scene,
meshopt's compressed and fallback buffers and, in clay mode, textures are
pruned. A stream that fails to decode fails the whole tile, so no
half-decoded mesh reaches the renderer (Review Focus 4). So does a required
extension cesium does not decode, such as KHR_meshopt_compression (spec
section 12), and a pruned model that compactBuffers or gltfio could not use:
a buffer without its bytes, a view past its buffer's end, or a material
naming a missing texture. compactBuffers' asserts are compiled out in
release builds, so on such a buffer it would erase out of range. The tests
build Mapbox-layout GLBs in the test process with meshoptimizer's codec-v0
encoder and stb's JPEG writer; no Mapbox bytes are stored.

Claude-Session: <the executing session's URL>
EOF
```

Then report Task 5 committed: lane 3's second merge (Execution, Merges) unblocks Task 14.

---

### Task 6: Count transport errors as network failures

**Lane:** 4. **Depends on:** Task 0 (the lane worktree forks after it).

**Worktree:** `$WORKTREE` is `$INTEGRATION-l4`. Every path in this lane is relative to it, and
every bash block starts with `cd "$WORKTREE"`: a shell can start anywhere, including the primary
checkout, which is never built in or edited.

**Files:**
- Modify: `overlume/src/environment_stream.hpp` (the `<optional>` … `<thread>` includes, 18-20;
  `CountingAssetAccessor::get`, `request` and `note`, 114-138; `FileFixtureAssetAccessor`, 151-173)
- Modify: `overlume/src/environment_stream.cpp` (the fixture classes' anonymous namespace and the
  head of `FileFixtureAssetAccessor::makeRequest`, 471-479; `FixtureStreamHandle`, 1160-1162;
  `make_fixture_source`, 1184-1189; `install_fixture_streaming_source_with_fallback`, 1228-1237;
  `revive_fixture_network`, 1243-1245)
- Modify: `overlume/src/environment_test_hooks.hpp` (`revive_fixture_network`, 34)
- Test: `overlume/tests/test_environment_stream.cpp` (a new test after
  `NetworkLossWithNoFallbackDirStillTransitionsAndStaysEmpty`, 658-681)

**Interfaces:**
- Consumes:

```cpp
// CesiumAsync
template <typename Func>
CesiumImpl::ContinuationFutureType_t<Func, T> Future<T>::thenImmediately(Func&& f) &&;
template <typename Func> Future<T> Future<T>::catchImmediately(Func&& f) &&;
template <typename T> Promise<T> AsyncSystem::createPromise() const;
template <typename TException> void Promise<T>::reject(TException error) const;
Future<T> Promise<T>::getFuture() const;
// overlume::testing (existing)
FixtureStreamHandle* install_fixture_streaming_source_with_fallback(
    overlume::VisualRenderer* r, const char* fixture_dir, const char* fallback_baked_dir,
    overlume::GeoAnchor anchor, bool follow_terrain = false);
std::string captured_cesium_log_text();
// test_environment_stream.cpp:613-620 (existing helper)
void pump_until_state(overlume::VisualRenderer* r, const overlume::CameraPose& pose,
                      std::vector<uint8_t>& buf, overlume::EnvironmentSourceState stop_state,
                      int max_ticks = 30);
```
- Produces (Interface contract):

```cpp
// overlume/src/environment_stream.hpp, overlume::FileFixtureAssetAccessor
explicit FileFixtureAssetAccessor(std::shared_ptr<std::atomic<bool>> killed = nullptr,
                                  std::shared_ptr<std::atomic<bool>> transport_down = nullptr);
// overlume/src/environment_test_hooks.hpp, overlume::testing
void break_fixture_network_transport(FixtureStreamHandle* handle);
```

  and the behaviour: `CountingAssetAccessor::get` and `request` count a rejected request as one
  consecutive failure, then rethrow `std::runtime_error(e.what())`.

**API notes** (cesium-native 0.64.0, paths under `overlume/build/_deps/cesium-native-src/`):
- `CesiumAsync/include/CesiumAsync/Future.h:202-206`: `catchImmediately` runs its callback on the
  rejecting thread and returns `Future<T>`; `:325-335` types the result as
  `ContinuationFutureType_t<Func, std::exception>`, so the callback declares
  `-> std::shared_ptr<CesiumAsync::IAssetRequest>` (a body that only throws would deduce `void`).
- `CesiumAsync/include/CesiumAsync/Impl/CatchFunction.h:25-33`: the callback is called as
  `f(std::move(e))` inside `catch (std::exception& e)`, so `e.what()` is valid there and a `throw`
  from it rejects the returned future.
- `Future.h:120-126`: `thenImmediately` is a value continuation, skipped when the parent rejects
  (async++ `task_base.h:540-555`, fact-check A6). That is why `note()` never ran on a transport
  error.
- `CesiumAsync/include/CesiumAsync/AsyncSystem.h:94-98` `createPromise<T>()`;
  `CesiumAsync/include/CesiumAsync/Promise.h:40-42` `reject`, `:62-64` `getFuture` (call once);
  `AsyncSystem.h:322-326` `createResolvedFuture(T&&)`, given an rvalue as `makeRequest` returns one.
- `CesiumCurl/src/CurlAssetAccessor.cpp:413`: curl's accessor throws `std::runtime_error` (text
  carries the URL) on a DNS failure, refused connection or timeout. The fixture switch rejects the
  same way.
- `CesiumAsync/src/CachingAssetAccessor.cpp:184-197`: behind the cache every request, the root
  manifest included, runs on the cache thread;
  `Cesium3DTilesSelection/src/TilesetContentManager.cpp:740-741` issues the manifest request when
  the `Tileset` is built, so a test can flip a switch before that request runs. That is why both
  switches spare `tileset.json` (the kill switch already does,
  `overlume/src/environment_stream.cpp:477-479`), as the head's contract for
  `break_fixture_network_transport` records.
- `Cesium3DTilesSelection/src/TilesetJsonLoader.cpp:1234`: the tile request's `thenInWorkerThread`
  is skipped on rejection, so the rejection reaches `TilesetContentManager.cpp:1312-1321`, which
  marks the tile `Failed` and logs `"An unexpected error occurred when loading tile: {}"` with
  `e.what()` through `externals.pLogger`, the redacting `overlume.cesium` logger. curl's text
  carries the request URL, token included, so the fixture's error text ends in a fixed bogus
  `?access_token=fixture-transport-token`. The test asserts that the redacting sink's capture holds
  `access_token=<redacted>` and never the bogus token. Redaction itself is tested by
  `TokenRedaction.NeverLeaksIntoLogText`, and for `mapbox://` URLs by Task 13.
- `overlume/src/environment_stream.cpp:1396-1411`: `drive_ion_token_redaction_probe` stops at the
  first captured `access_token=`, and the capture lasts as long as the process. gtest runs the
  `EnvironmentStream` suite (first registered at `test_environment_stream.cpp:93`) before
  `TokenRedaction` (`:588`), so in one process the transport test's redacted line would end the
  probe before the probe's own ion URL is logged. ctest runs each test in its own process, so the
  gate is unaffected. This lane's filtered runs give `TokenRedaction.*` its own process for the
  same reason.
- `overlume/tests/fixtures/environment_tiles_fixture_fallback_0/tileset.json` has 16 content tiles,
  twice `kNetworkLossConsecutiveFailures` (8), and the kill-switch tests already reach eight
  consecutive failures on it within 30 ticks.

- [ ] **Step 1: Give the file fixture a transport switch**

Test plumbing first, so that Step 3's red run fails on behaviour, not on a missing symbol.

In `overlume/src/environment_stream.hpp`, replace

```cpp
#include <optional>
#include <string>
#include <thread>
```

with

```cpp
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
```

In `overlume/src/environment_stream.hpp`, replace

```cpp
class FileFixtureAssetAccessor : public CesiumAsync::IAssetAccessor {
public:
    explicit FileFixtureAssetAccessor(std::shared_ptr<std::atomic<bool>> killed = nullptr)
        : killed_(std::move(killed)) {}

    CesiumAsync::Future<std::shared_ptr<CesiumAsync::IAssetRequest>> get(
        const CesiumAsync::AsyncSystem& asyncSystem, const std::string& url,
        const std::vector<THeader>&) override {
        return asyncSystem.createResolvedFuture(makeRequest("GET", url));
    }
    CesiumAsync::Future<std::shared_ptr<CesiumAsync::IAssetRequest>> request(
        const CesiumAsync::AsyncSystem& asyncSystem, const std::string& verb,
        const std::string& url, const std::vector<THeader>&,
        const std::span<const std::byte>&) override {
        return asyncSystem.createResolvedFuture(makeRequest(verb, url));
    }
    void tick() noexcept override {}

private:
    std::shared_ptr<CesiumAsync::IAssetRequest> makeRequest(const std::string& verb,
                                                            const std::string& url);
    std::shared_ptr<std::atomic<bool>> killed_;
};
```

with

```cpp
class FileFixtureAssetAccessor : public CesiumAsync::IAssetAccessor {
public:
    // transport_down rejects every request but tileset.json with no response, as curl's accessor
    // does on a DNS failure, refused connection or timeout.
    explicit FileFixtureAssetAccessor(std::shared_ptr<std::atomic<bool>> killed = nullptr,
                                      std::shared_ptr<std::atomic<bool>> transport_down = nullptr)
        : killed_(std::move(killed)), transportDown_(std::move(transport_down)) {}

    CesiumAsync::Future<std::shared_ptr<CesiumAsync::IAssetRequest>> get(
        const CesiumAsync::AsyncSystem& asyncSystem, const std::string& url,
        const std::vector<THeader>&) override {
        return respond(asyncSystem, "GET", url);
    }
    CesiumAsync::Future<std::shared_ptr<CesiumAsync::IAssetRequest>> request(
        const CesiumAsync::AsyncSystem& asyncSystem, const std::string& verb,
        const std::string& url, const std::vector<THeader>&,
        const std::span<const std::byte>&) override {
        return respond(asyncSystem, verb, url);
    }
    void tick() noexcept override {}

private:
    CesiumAsync::Future<std::shared_ptr<CesiumAsync::IAssetRequest>> respond(
        const CesiumAsync::AsyncSystem& asyncSystem, const std::string& verb,
        const std::string& url);
    std::shared_ptr<CesiumAsync::IAssetRequest> makeRequest(const std::string& verb,
                                                            const std::string& url);
    std::shared_ptr<std::atomic<bool>> killed_;
    std::shared_ptr<std::atomic<bool>> transportDown_;
};
```

In `overlume/src/environment_stream.cpp`, replace

```cpp
constexpr char kFileScheme[] = "file://";

}

std::shared_ptr<CesiumAsync::IAssetRequest> FileFixtureAssetAccessor::makeRequest(
    const std::string& verb, const std::string& url) {
    const bool isRootManifest =
        url.size() >= 12 && url.compare(url.size() - 12, 12, "tileset.json") == 0;
    if (killed_ && killed_->load() && !isRootManifest) {
```

with

```cpp
constexpr char kFileScheme[] = "file://";

// Both fixture switches spare the root manifest: behind the cache its request runs on the cache
// thread, possibly after a test has already flipped a switch.
bool is_root_manifest(const std::string& url) {
    return url.size() >= 12 && url.compare(url.size() - 12, 12, "tileset.json") == 0;
}

}

CesiumAsync::Future<std::shared_ptr<CesiumAsync::IAssetRequest>> FileFixtureAssetAccessor::respond(
    const CesiumAsync::AsyncSystem& asyncSystem, const std::string& verb, const std::string& url) {
    if (transportDown_ && transportDown_->load() && !is_root_manifest(url)) {
        const auto promise =
            asyncSystem.createPromise<std::shared_ptr<CesiumAsync::IAssetRequest>>();
        // curl's text carries the request URL, token included; this one carries a bogus token.
        promise.reject(std::runtime_error("fixture transport error: " + url +
                                          "?access_token=fixture-transport-token"));
        return promise.getFuture();
    }
    return asyncSystem.createResolvedFuture(makeRequest(verb, url));
}

std::shared_ptr<CesiumAsync::IAssetRequest> FileFixtureAssetAccessor::makeRequest(
    const std::string& verb, const std::string& url) {
    if (killed_ && killed_->load() && !is_root_manifest(url)) {
```

In `overlume/src/environment_stream.cpp`, replace

```cpp
struct FixtureStreamHandle {
    std::shared_ptr<std::atomic<bool>> killed;
};
```

with

```cpp
struct FixtureStreamHandle {
    std::shared_ptr<std::atomic<bool>> killed;
    std::shared_ptr<std::atomic<bool>> transport_down;
};
```

In `overlume/src/environment_stream.cpp`, replace

```cpp
    std::shared_ptr<std::atomic<bool>> killed, bool materials_original, bool follow_terrain = false,
    std::optional<double> radius_override_m = std::nullopt) {
    if (fixture_dir == nullptr) return nullptr;
    auto fileAccessor = std::make_shared<overlume::FileFixtureAssetAccessor>(std::move(killed));
```

with

```cpp
    std::shared_ptr<std::atomic<bool>> killed, bool materials_original, bool follow_terrain = false,
    std::optional<double> radius_override_m = std::nullopt,
    std::shared_ptr<std::atomic<bool>> transport_down = nullptr) {
    if (fixture_dir == nullptr) return nullptr;
    auto fileAccessor = std::make_shared<overlume::FileFixtureAssetAccessor>(
        std::move(killed), std::move(transport_down));
```

In `overlume/src/environment_stream.cpp`, replace

```cpp
    auto killed = std::make_shared<std::atomic<bool>>(false);
    auto source =
        make_fixture_source(fixture_dir, fallback_baked_dir, anchor, killed, false, follow_terrain);
    if (!source) {
        r->environmentSource.reset();
        return nullptr;
    }
    r->environmentSource = std::move(source);
    return new FixtureStreamHandle{std::move(killed)};
}
```

with

```cpp
    auto killed = std::make_shared<std::atomic<bool>>(false);
    auto transportDown = std::make_shared<std::atomic<bool>>(false);
    auto source = make_fixture_source(fixture_dir, fallback_baked_dir, anchor, killed, false,
                                      follow_terrain, std::nullopt, transportDown);
    if (!source) {
        r->environmentSource.reset();
        return nullptr;
    }
    r->environmentSource = std::move(source);
    return new FixtureStreamHandle{std::move(killed), std::move(transportDown)};
}
```

In `overlume/src/environment_stream.cpp`, replace

```cpp
void revive_fixture_network(FixtureStreamHandle* handle) {
    if (handle && handle->killed) handle->killed->store(false);
}
```

with

```cpp
void revive_fixture_network(FixtureStreamHandle* handle) {
    if (handle && handle->killed) handle->killed->store(false);
}

void break_fixture_network_transport(FixtureStreamHandle* handle) {
    if (handle && handle->transport_down) handle->transport_down->store(true);
}
```

In `overlume/src/environment_test_hooks.hpp`, replace

```cpp
void revive_fixture_network(FixtureStreamHandle* handle);
```

with

```cpp
void revive_fixture_network(FixtureStreamHandle* handle);

// Make the fixture network reject every tile request with a transport error (no response);
// tileset.json is still served, as with kill_fixture_network.
void break_fixture_network_transport(FixtureStreamHandle* handle);
```

- [ ] **Step 2: Write the failing test**

It mirrors `NetworkDeadFromFirstRequestFallsBackToBakedChunksOnce`, with the transport switch in
place of the kill switch.

In `overlume/tests/test_environment_stream.cpp`, replace

```cpp
    EXPECT_EQ(overlume::testing::environment_loaded_chunk_count(r), 0u);
    EXPECT_TRUE(overlume::render_frame(r, kStdPose, {buf.data(), 320, 240}));
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStream, MaterialsOriginalDefaultsFalse) {
```

with

```cpp
    EXPECT_EQ(overlume::testing::environment_loaded_chunk_count(r), 0u);
    EXPECT_TRUE(overlume::render_frame(r, kStdPose, {buf.data(), 320, 240}));
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStream, TransportErrorsCountAsNetworkLossAndFallBack) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    auto* handle = overlume::testing::install_fixture_streaming_source_with_fallback(
        r, kTilesFixtureFallbackDir.c_str(), kTestTownDir.c_str(), kFixtureAnchor);
    ASSERT_NE(handle, nullptr);

    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = kFixtureBlockCenterMap;
    overlume::set_scene(r, s);

    std::vector<uint8_t> buf(320u * 240u * 3u);
    overlume::testing::break_fixture_network_transport(handle);
    pump_until_state(r, kStdPose, buf, overlume::EnvironmentSourceState::STREAMING_FALLBACK);

    EXPECT_EQ(overlume::environment_source_state(r),
              overlume::EnvironmentSourceState::STREAMING_FALLBACK)
        << "rejected requests (curl's DNS failure, refusal or timeout) never reached the "
           "consecutive-failure counter, so a real loss of connectivity never falls back";
    EXPECT_GT(overlume::testing::environment_loaded_chunk_count(r), 0u)
        << "the baked fallback chunks must load";
    const std::string logText = overlume::testing::captured_cesium_log_text();
    EXPECT_NE(
        logText.find("An unexpected error occurred when loading tile: fixture transport error: "),
        std::string::npos)
        << "the counter must rethrow the transport error with its original text";
    EXPECT_NE(logText.find("access_token=<redacted>"), std::string::npos)
        << "the transport error's text, URL and token included, must reach the redacting logger";
    EXPECT_EQ(logText.find("fixture-transport-token"), std::string::npos)
        << "a token in a transport error's text must never reach the log";
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStream, MaterialsOriginalDefaultsFalse) {
```

- [ ] **Step 3: Run it and see it fail on the old `CountingAssetAccessor`**

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_environment_stream
overlume/build/test_environment_stream \
    --gtest_filter='EnvironmentStream.TransportErrorsCountAsNetworkLossAndFallBack'
```

Expected: the source stays `STREAMING`, because no rejection reaches the counter:

```
.../overlume/tests/test_environment_stream.cpp:702: Failure
Expected equality of these values:
  overlume::environment_source_state(r)
    Which is: 1-byte object <02>
  overlume::EnvironmentSourceState::STREAMING_FALLBACK
    Which is: 1-byte object <03>
rejected requests (curl's DNS failure, refusal or timeout) never reached the consecutive-failure counter, so a real loss of connectivity never falls back
.../overlume/tests/test_environment_stream.cpp:705: Failure
Expected: (overlume::testing::environment_loaded_chunk_count(r)) > (0u), actual: 0 vs 0
the baked fallback chunks must load
[  FAILED  ] EnvironmentStream.TransportErrorsCountAsNetworkLossAndFallBack
```

The three log-text expectations already hold here: the old counter passes the rejection through
untouched, and cesium logs it through the redacting logger.

- [ ] **Step 4: Count rejected requests, then rethrow**

`get` and `request` both route through one `counted`, so this test covers both.

In `overlume/src/environment_stream.hpp`, replace

```cpp
    CesiumAsync::Future<std::shared_ptr<CesiumAsync::IAssetRequest>> get(
        const CesiumAsync::AsyncSystem& asyncSystem, const std::string& url,
        const std::vector<THeader>& headers) override {
        return inner_->get(asyncSystem, url, headers)
            .thenImmediately([this](std::shared_ptr<CesiumAsync::IAssetRequest>&& req) {
                note(req.get());
                return req;
            });
    }
    CesiumAsync::Future<std::shared_ptr<CesiumAsync::IAssetRequest>> request(
        const CesiumAsync::AsyncSystem& asyncSystem, const std::string& verb,
        const std::string& url, const std::vector<THeader>& headers,
        const std::span<const std::byte>& payload) override {
        return inner_->request(asyncSystem, verb, url, headers, payload)
            .thenImmediately([this](std::shared_ptr<CesiumAsync::IAssetRequest>&& req) {
                note(req.get());
                return req;
            });
    }
    void tick() noexcept override { inner_->tick(); }

    int consecutive_failures() const { return consecutiveFailures_.load(); }

private:
    void note(const CesiumAsync::IAssetRequest* req) {
```

with

```cpp
    CesiumAsync::Future<std::shared_ptr<CesiumAsync::IAssetRequest>> get(
        const CesiumAsync::AsyncSystem& asyncSystem, const std::string& url,
        const std::vector<THeader>& headers) override {
        return counted(inner_->get(asyncSystem, url, headers));
    }
    CesiumAsync::Future<std::shared_ptr<CesiumAsync::IAssetRequest>> request(
        const CesiumAsync::AsyncSystem& asyncSystem, const std::string& verb,
        const std::string& url, const std::vector<THeader>& headers,
        const std::span<const std::byte>& payload) override {
        return counted(inner_->request(asyncSystem, verb, url, headers, payload));
    }
    void tick() noexcept override { inner_->tick(); }

    int consecutive_failures() const { return consecutiveFailures_.load(); }

private:
    // A transport error (curl: DNS failure, refused connection, timeout) rejects the future with
    // no response at all. Count it like a failed reply, then rethrow so the caller still sees it.
    CesiumAsync::Future<std::shared_ptr<CesiumAsync::IAssetRequest>> counted(
        CesiumAsync::Future<std::shared_ptr<CesiumAsync::IAssetRequest>>&& reply) {
        return std::move(reply)
            .thenImmediately([this](std::shared_ptr<CesiumAsync::IAssetRequest>&& req) {
                note(req.get());
                return req;
            })
            .catchImmediately(
                [this](std::exception&& e) -> std::shared_ptr<CesiumAsync::IAssetRequest> {
                    note(nullptr);
                    throw std::runtime_error(e.what());
                });
    }
    void note(const CesiumAsync::IAssetRequest* req) {
```

- [ ] **Step 5: Run it and see it pass, with the kill-switch tests**

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_environment_stream
overlume/build/test_environment_stream \
    --gtest_filter='EnvironmentStream.TransportErrors*:EnvironmentStream.Network*:EnvironmentStream.DiskCache*'
overlume/build/test_environment_stream --gtest_filter='TokenRedaction.*'
```

Expected: `[  PASSED  ] 4 tests.`, then `[  PASSED  ] 1 test.` The new test is the check that fails
when the `catchImmediately` in `CountingAssetAccessor::counted` is removed (Step 3's output).
`NetworkDead…`, `NetworkLoss…` and `DiskCacheServesTilesWithNetworkDead` cover the kill switch's
move onto `is_root_manifest`. `TokenRedaction.*` gets its own process (API notes), and its probe
goes through the new `counted`.

- [ ] **Step 6: Format**

```bash
cd "$WORKTREE"
git add overlume/src/environment_stream.hpp overlume/src/environment_stream.cpp \
    overlume/src/environment_test_hooks.hpp overlume/tests/test_environment_stream.cpp
tools/check_format.sh --fix && tools/check_format.sh && git diff --stat
```

Expected: `check_format.sh: reformatted N files with clang-format 23.1.1`, then
`check_format.sh: PASS (N files, clang-format 23.1.1)`, and `git diff --stat` prints nothing (the
code above is already formatted; if it lists a file, `git add` that file again).

- [ ] **Step 7: Gate** (foreground, Bash timeout 600000 ms)

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
ROS_DOMAIN_ID=84 CI_VISUAL_MODE_DOMAIN_ID=84 \
    flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock tools/ci_visual_mode.sh
```

Exit 75 means another lane held the gate lock for 300 s: run the same command again. Expected:
six `PASS` lines (POD header check, library ctest suite, node gtests, WS bridge pytest suite,
golden suite, examples) and exit 0. The library line counts one entry and one `ok` more than the
fork's baseline: `out of 348 (332 ok / 6 skipped)`. The other counts are unchanged (363 node
tests, 82 WS bridge tests, 23 goldens all `ok`, 6 examples).

- [ ] **Step 8: Commit**

```bash
cd "$WORKTREE"
git add overlume/src/environment_stream.hpp overlume/src/environment_stream.cpp \
    overlume/src/environment_test_hooks.hpp overlume/tests/test_environment_stream.cpp
git commit -F - <<'EOF'
fix(stream): count transport errors as network failures

CountingAssetAccessor saw only HTTP replies. A transport error (curl's
DNS failure, refused connection or timeout) rejects the request future,
the counter's continuation never ran, and a real loss of connectivity
never reached the eight-failure fallback, for ion as for the coming
mapbox:// source. A rejected request now counts as a failure and is
rethrown with its original text, so cesium still fails the tile and
logs it through the redacting logger.

The file fixture gains a transport switch that rejects every tile
request as curl's accessor does, its error text carrying a bogus token
as curl's carries the request URL; like the kill switch it spares the
root manifest, whose request runs on the cache thread and can start
after a test flips the switch. TransportErrorsCountAsNetworkLossAndFallBack
fails without the counter change, and checks that the rethrown text
reaches the log with the token redacted.

Claude-Session: <the executing session's URL>
EOF
```

### Task 7: Custom tilesets with area selection

**Lane:** 4. **Depends on:** Task 6 (same lane; two of the hunks below use its lines as context).

**Worktree:** `$WORKTREE` (`$INTEGRATION-l4`), as in Task 6: paths are relative to it, and every
bash block starts with `cd "$WORKTREE"`.

**Files** (line numbers as they are after Task 6):
- Modify: `overlume/src/environment_stream.hpp` (includes, 29-30; file scope before
  `namespace overlume {`, 40-43; after `correct_ecef_point_height`, 253-256; the constructor's last
  parameter, 264-265; `last_view_frustum_count`, 285; `tileRadiusOverrideM_`, 322)
- Modify: `overlume/src/environment_stream.cpp` (includes, 15-17; the forward declaration of
  `make_redacting_logger`, 229-231; `g_redactingSinkForTest` to `TokenBypassAssetAccessor`, 327-341;
  `TokenBypassAssetAccessor`'s end to `compute_ecef_to_map`, 365-398; the constructor, 745-776;
  `synthesize_view_and_pump`'s views, 1006-1017, and the head of its render loop, 1018-1020; after
  `break_fixture_network_transport`, 1268-1270)
- Modify: `overlume/src/environment_test_hooks.hpp` (includes, 6-8; after
  `break_fixture_network_transport`, 36-38)
- Test: `overlume/tests/test_environment_stream.cpp` (`pump_until_loaded`, 80-89; four tests
  appended after `EnvSourceCapture.Clipped`, which ends the file at 1356)

**Interfaces:**
- Consumes:

```cpp
// Cesium3DTilesSelection
Tileset(const TilesetExternals& externals, std::unique_ptr<TilesetContentLoader>&& pCustomLoader,
        std::unique_ptr<Tile>&& pRootTile, const TilesetOptions& options = TilesetOptions());
ViewState(const BoundingVolume& boundingVolume, double geometricErrorThreshold,
          const CesiumGeospatial::Ellipsoid& ellipsoid CESIUM_DEFAULT_ELLIPSOID);
virtual CesiumAsync::Future<TileLoadResult> TilesetContentLoader::loadTileContent(
    const TileLoadInput& input) = 0;
virtual TileChildrenResult TilesetContentLoader::createTileChildren(
    const Tile& tile, const CesiumGeospatial::Ellipsoid& ellipsoid CESIUM_DEFAULT_ELLIPSOID) = 0;
explicit Tile(TilesetContentLoader* pLoader, const TileID& tileID = {}) noexcept;
Tile(TilesetContentLoader* pLoader, const TileID& tileID, TileEmptyContent emptyContent) noexcept;
void Tile::setUnconditionallyRefine() noexcept;
void Tile::setBoundingVolume(const BoundingVolume& value) noexcept;
void Tile::setTransform(const glm::dmat4x4& value) noexcept;
void Tile::createChildTiles(std::vector<Tile>&& children);
TileLoadState Tile::getState() const noexcept;  // TileLoadState::Done: main-thread resources in
// CesiumGeometry, CesiumGeospatial
constexpr BoundingSphere(const glm::dvec3& center, double radius) noexcept;
static glm::dmat4x4 GlobeTransforms::eastNorthUpToFixedFrame(
    const glm::dvec3& origin, const Ellipsoid& ellipsoid CESIUM_DEFAULT_ELLIPSOID) noexcept;
BoundingRegion(const GlobeRectangle& rectangle, double minimumHeight, double maximumHeight,
               const Ellipsoid& ellipsoid CESIUM_DEFAULT_ELLIPSOID);
constexpr GlobeRectangle(double west, double south, double east, double north) noexcept;  // radians
// Task 6
FileFixtureAssetAccessor();  // both switches null
```
- Produces (Interface contract):

```cpp
// overlume/src/environment_stream.hpp
namespace spdlog {
class logger;
}
namespace overlume {
std::shared_ptr<spdlog::logger> make_redacting_logger();
Cesium3DTilesSelection::TilesetExternals build_externals(
    std::shared_ptr<CesiumAsync::IAssetAccessor> base, const CesiumAsync::AsyncSystem& asyncSystem,
    const std::string& cache_dir, uint64_t max_cache_items,
    std::shared_ptr<CountingAssetAccessor>* out_counting = nullptr);
struct CustomTileset {
    std::unique_ptr<Cesium3DTilesSelection::TilesetContentLoader> loader;
    std::unique_ptr<Cesium3DTilesSelection::Tile> root;
    bool area_selection = false;
};
// StreamingEnvironmentSource: constructor gains a trailing `CustomTileset custom_tileset = {}`,
// and the class gains `bool area_selection() const;`
}
// overlume/src/environment_test_hooks.hpp, overlume::testing
struct FixtureGridHandle;
FixtureGridHandle* install_fixture_grid_source(overlume::VisualRenderer* r,
                                               overlume::GeoAnchor anchor, double radius_m,
                                               bool area_selection);
std::vector<std::pair<int, int>> fixture_grid_requested_leaves(FixtureGridHandle* handle);
```

**API notes** (cesium-native 0.64.0, paths under `overlume/build/_deps/cesium-native-src/`; the
fact-check notes and these sources agree on every point used here):
- `Cesium3DTilesSelection/include/Cesium3DTilesSelection/Tileset.h:53-57`: the custom-loader
  constructor. `src/TilesetContentManager.cpp:693-725` (`createFromLoader`) installs loader and root
  from a main-thread continuation, so the root exists after the first `dispatchMainThreadTasks()`,
  which `synthesize_view_and_pump` calls every update (one frame of latency).
- `include/Cesium3DTilesSelection/ViewState.h:127-130`, `src/ViewState.cpp:124-154`: the area
  constructor (viewport 0 × 0; its cartographic position is the centre of the sphere's estimated
  rectangle, `src/BoundingVolume.cpp:107`); `ViewState.cpp:214-221`: it reports the threshold as
  every tile's screen-space error. `CesiumGeometry/include/CesiumGeometry/BoundingSphere.h:24-25`;
  `include/Cesium3DTilesSelection/BoundingVolume.h:30-37` (the variant `BoundingVolume`).
- `src/GeneralCullingVolume.cpp:89-104` and `src/BoundingVolume.cpp:283-286`: an area view culls by
  intersection, a region being visible when `region.computeDistanceSquaredToPosition(centre) < r²`;
  `CesiumGeospatial/src/BoundingRegion.cpp` (`computeDistanceSquaredToPosition`) measures to the
  rectangle's edge planes and box, i.e. to the cell's nearest edge, which is what the test's
  `grid_cells_within` computes.
- `src/TilesetSelection.cpp:1138-1160`: the views are reduced to the minimum threshold, and a
  view without one wins, so next to a screen-space view every view selects by screen-space error:
  the area view must be alone. `:329-335`: a leaf with geometric error 0 meets the 1.0 threshold.
  `:1084-1094`: an unconditionally refined root is visited even when culled. `:1120-1126`: with
  `preloadSiblings` (TilesetOptions.h:79, default true) every culled sibling is queued at preload
  priority. `:471`: fog culling never culls a view whose viewport height is 0, and the area
  view's is 0, so area selection needs no `enableFogCulling` switch (the contract and the spec
  agree).
- Unconditional refinement and `loadingDescendantLimit` (TilesetOptions.h:91, 20):
  `src/Tile.cpp:218-239` makes an unconditionally refined tile with children never renderable, so
  `TilesetSelection.cpp:988-990` kicks the root's children only when more than 20 of them are not
  yet renderable (there are 16), and even then the queue-restoring branch at `:760-763` skips
  unconditionally refined tiles. The leaves' loads are never dropped, and nor are the leaves:
  `renderLeaf` (`:611-628`, reached at `:840`) lists a leaf in `tilesToRenderThisFrame` whatever
  its load state, from the first selection frame on. A regular tileset hides that, because a
  renderable parent kicks its not-yet-renderable children out of the list (`:970-993`).
- What a listed leaf holds before `Done`: `TilesetContentManager.cpp:2001-2050` (`setTileContent`,
  on the main thread once the worker finishes) stores the worker's render resources, Overlume's
  `LoadThreadGlb*`, in the tile's render content (`:122-123`) and marks the tile `ContentLoaded`.
  Only `finishLoading` (`:1655-1663`) swaps in `prepareInMainThread`'s `FilamentAsset*` and marks
  it `Done`, and `prepareInMainThread` deletes the `LoadThreadGlb` (`environment_stream.cpp:635`).
  A leaf that turns `ContentLoaded` inside `dispatchMainThreadTasks()` is already in the frame's
  render list, and the render loop (`environment_stream.cpp:1001-1007` at the fork) checks only
  `isRenderContent()` and a non-null pointer, so it adds a `LoadThreadGlb*` to the scene and to
  `inScene_` as a `FilamentAsset*`. Its removal in a later frame reads freed memory, which in
  `AreaSelectionFollowsTheEgo` is a SIGSEGV in `filament::FScene::removeEntities`. Step 2 skips
  every tile that is not `Done`
  (`include/Cesium3DTilesSelection/Tile.h:540` `getState`, `:55-99` `TileLoadState`), and Step 9
  shows `AreaSelectionFollowsTheEgo` crashing without that line. The Mapbox quadtree (empty,
  unconditionally refined z0–z13) has the grid's shape. Reading the source (not reproduced), an
  ion tileset can also list a leaf that is not ready, when a sibling rendered last frame and so
  nothing is kicked (`TilesetSelection.cpp:972-973`). The guard is therefore not specific to the
  grid.
- `src/TilesetContentManager.cpp:1896-1926`: up to `maximumSimultaneousTileLoads` (20 > 16) loads
  start per `loadTiles`, so preloaded siblings are requested in the first selection frame: the
  exactness test catches `preloadSiblings` (Step 8 runs that revert).
- `include/Cesium3DTilesSelection/Tile.h:152-154` (unknown content, `Unloaded`), `:197-200` (empty
  content, `ContentLoaded`), `:279` `createChildTiles`, `:302` `setBoundingVolume`, `:391`
  `setUnconditionallyRefine`, `:434` `setTransform` (absolute, already including any parent); all in
  the public section. `include/Cesium3DTilesSelection/TileID.h:34-39`; `src/TileID.cpp:45-48`
  with `src/Tile.cpp:478-481`: `TileID()` is an empty string, not loadable, so the root holds no
  reference, is never unloaded (`TilesetContentManager.cpp:1366-1369`) and never loaded
  (`:1168-1177`); only leaves reach `loadTileContent`.
- `src/EllipsoidTilesetLoader.cpp:56-90`: the pattern used for the grid: an empty, unconditionally
  refined root whose children are built up front and handed to `createChildTiles` before the
  custom-loader `Tileset` is constructed. Chosen over creating them in `createTileChildren` because
  the tree is then complete before cesium's first selection, with no latent-children frame;
  `createTileChildren` only answers "no children" for leaves, and a `Failed` answer is never asked
  again (`TilesetContentManager.cpp:1342-1363`).
- `include/Cesium3DTilesSelection/TilesetContentLoader.h:157` `loadTileContent`, `:174-177`
  `createTileChildren`, `:120-124` `TileChildrenResult`; `TileLoadResult.h:76-163` field order,
  aggregate-initialised as `EllipsoidTilesetLoader.cpp:94-103` does;
  `TilesetContentManager.cpp:1255` calls `loadTileContent` synchronously from `loadTiles` (the main
  thread); `:533-535` skips a null `pCompletedRequest`.
- `CesiumGeospatial/include/CesiumGeospatial/GlobeTransforms.h:34-36`, `Ellipsoid.h:114, 128`,
  `Cartographic.h:21-26, 38-41`, `GlobeRectangle.h:59-64`, `BoundingRegion.h:30-34`;
  `CesiumGeometry/include/CesiumGeometry/QuadtreeTileID.h:29-30`.
- CesiumGltf (`CesiumGltf/generated/include/CesiumGltf/`): `Asset.h:36` (`version` is empty by
  default, so it is set), `BufferSpec.h:33` (`byteLength`, which
  `CesiumGltfWriter/src/GltfWriter.cpp:160-198` writes from the model, so it is set),
  `BufferView.h:36-46`, `AccessorSpec.h:38, 53, 65, 71, 93, 112, 130, 139, 154, 169`,
  `MeshPrimitive.h:55, 64`, `Node.h:57`, `Scene.h:24`, `ModelSpec.h:123`.
- Overlume, unchanged: `prepareInLoadThread` (`environment_stream.cpp` 601-631 at the fork) keeps
  one buffer as it is, corrects heights through the leaf's transform, writes the GLB, and
  `ensure_flat_normals` (`overlume/src/gltf_normals.cpp:304-387`) adds the NORMAL the triangle
  lacks; positions come first in the buffer so the float accessor stays 4-byte aligned. The radius
  gate keeps a tile when `hypot(dx, dy) - extentM <= radius`, `extentM` summing the region box's
  half axes (about 500 + 500 + 55 m for a cell): a central cell (centre 707 m from the anchor)
  passes at 500 m, an edge cell (1581 m) needs 526 m and a corner (2121 m) 1066 m.

The grid: 16 leaves of 1 km, cell (c, r) spanning east [(c − 2) km, (c − 1) km] and north
[(1 − r) km, (2 − r) km] of the anchor in its tangent plane, heights anchor − 10 m to
anchor + 100 m, each with an east-north-up frame at its centre. Two choices differ from the lane
brief: leaves are `QuadtreeTileID(2, c, r)` (a 4 × 4 grid is quadtree level 2; cesium never reads
the level of a custom loader's IDs), and the second radius is 1200 m, not 1600 m: at 1600 m every
cell is in reach (the corners are 1414 m away), so that test could not tell area selection from
loading everything.

- [ ] **Step 1: Header seam: `CustomTileset`, the constructor parameter, the shared declarations**

In `overlume/src/environment_stream.hpp`, replace

```cpp
#include <Cesium3DTilesSelection/Tileset.h>
#include <Cesium3DTilesSelection/TilesetExternals.h>
```

with

```cpp
#include <Cesium3DTilesSelection/Tileset.h>
#include <Cesium3DTilesSelection/TilesetContentLoader.h>
#include <Cesium3DTilesSelection/TilesetExternals.h>
```

In `overlume/src/environment_stream.hpp`, replace

```cpp
#include "environment.hpp"
#include "overlume/scene.h"

namespace overlume {
```

with

```cpp
#include "environment.hpp"
#include "overlume/scene.h"

namespace spdlog {
class logger;
}

namespace overlume {
```

In `overlume/src/environment_stream.hpp`, replace

```cpp
glm::dvec3 correct_ecef_point_height(const glm::dvec3& ecef_pos, const glm::dmat4& ecef_to_map,
                                     double anchor_height_m);

class StreamingEnvironmentSource : public EnvironmentSource {
```

with

```cpp
glm::dvec3 correct_ecef_point_height(const glm::dvec3& ecef_pos, const glm::dmat4& ecef_to_map,
                                     double anchor_height_m);

std::shared_ptr<spdlog::logger> make_redacting_logger();

// Wraps `base` in a CountingAssetAccessor, behind the SQLite cache unless cache_dir is "off".
Cesium3DTilesSelection::TilesetExternals build_externals(
    std::shared_ptr<CesiumAsync::IAssetAccessor> base, const CesiumAsync::AsyncSystem& asyncSystem,
    const std::string& cache_dir, uint64_t max_cache_items,
    std::shared_ptr<CountingAssetAccessor>* out_counting = nullptr);

struct CustomTileset {
    std::unique_ptr<Cesium3DTilesSelection::TilesetContentLoader> loader;
    std::unique_ptr<Cesium3DTilesSelection::Tile> root;
    // Select with cesium's area view, a sphere of the source's radius around the ego, as the only
    // view (geometric-error threshold 1.0); also turns preloadSiblings off. (Fog culling needs no
    // switch: cesium 0.64 never fog-culls an area view, TilesetSelection.cpp:471.)
    bool area_selection = false;
};

class StreamingEnvironmentSource : public EnvironmentSource {
```

In `overlume/src/environment_stream.hpp`, replace

```cpp
                               double max_tilt_deg = 2.0, double brightness = 1.0,
                               std::optional<double> radius_override_m = std::nullopt);
```

with

```cpp
                               double max_tilt_deg = 2.0, double brightness = 1.0,
                               std::optional<double> radius_override_m = std::nullopt,
                               CustomTileset custom_tileset = {});
```

In `overlume/src/environment_stream.hpp`, replace

```cpp
    size_t last_view_frustum_count() const { return lastViewFrustumCount_; }
```

with

```cpp
    size_t last_view_frustum_count() const { return lastViewFrustumCount_; }

    bool area_selection() const { return areaSelection_; }
```

In `overlume/src/environment_stream.hpp`, replace

```cpp
    std::optional<double> tileRadiusOverrideM_;
```

with

```cpp
    std::optional<double> tileRadiusOverrideM_;
    bool areaSelection_ = false;
```

- [ ] **Step 2: Source seam: logger and externals leave the anonymous namespace; the custom-loader
  branch; only `Done` tiles enter the scene**

The bodies of `make_redacting_logger` and `build_externals` do not change; `build_externals` loses
only its default argument, which now lives in the header. The render loop treats every listed
tile's render resources as a `FilamentAsset*`, which they are only once the tile is `Done`. The
grid's leaves, like the Mapbox quadtree's, are listed before that (API notes), so the loop now
skips tiles that are not `Done`. Without that line `AreaSelectionFollowsTheEgo` crashes (Step 9).

In `overlume/src/environment_stream.cpp`, replace

```cpp
#include <CesiumGeometry/Transforms.h>
#include <CesiumGeospatial/Cartographic.h>
#include <CesiumGeospatial/Ellipsoid.h>
```

with

```cpp
#include <CesiumGeometry/BoundingSphere.h>
#include <CesiumGeometry/QuadtreeTileID.h>
#include <CesiumGeometry/Transforms.h>
#include <CesiumGeospatial/BoundingRegion.h>
#include <CesiumGeospatial/Cartographic.h>
#include <CesiumGeospatial/Ellipsoid.h>
#include <CesiumGeospatial/GlobeRectangle.h>
#include <CesiumGeospatial/GlobeTransforms.h>
```

In `overlume/src/environment_stream.cpp`, replace

```cpp
std::shared_ptr<spdlog::logger> make_redacting_logger();

void strip_attributes_and_correct_heights(CesiumGltf::Model& model, const glm::dmat4& modelToEcef,
```

with

```cpp
void strip_attributes_and_correct_heights(CesiumGltf::Model& model, const glm::dmat4& modelToEcef,
```

In `overlume/src/environment_stream.cpp`, replace

```cpp
std::shared_ptr<RedactingSink> g_redactingSinkForTest;

std::shared_ptr<spdlog::logger> make_redacting_logger() {
```

with

```cpp
std::shared_ptr<RedactingSink> g_redactingSinkForTest;

}

std::shared_ptr<spdlog::logger> make_redacting_logger() {
```

In `overlume/src/environment_stream.cpp`, replace

```cpp
    return logger;
}

class TokenBypassAssetAccessor final : public CesiumAsync::IAssetAccessor {
```

with

```cpp
    return logger;
}

namespace {

class TokenBypassAssetAccessor final : public CesiumAsync::IAssetAccessor {
```

In `overlume/src/environment_stream.cpp`, replace

```cpp
    std::shared_ptr<CesiumAsync::IAssetAccessor> direct_;
};

Cesium3DTilesSelection::TilesetExternals build_externals(
    std::shared_ptr<CesiumAsync::IAssetAccessor> base, const CesiumAsync::AsyncSystem& asyncSystem,
    const std::string& cache_dir, uint64_t max_cache_items,
    std::shared_ptr<CountingAssetAccessor>* out_counting = nullptr) {
```

with

```cpp
    std::shared_ptr<CesiumAsync::IAssetAccessor> direct_;
};

}

Cesium3DTilesSelection::TilesetExternals build_externals(
    std::shared_ptr<CesiumAsync::IAssetAccessor> base, const CesiumAsync::AsyncSystem& asyncSystem,
    const std::string& cache_dir, uint64_t max_cache_items,
    std::shared_ptr<CountingAssetAccessor>* out_counting) {
```

In `overlume/src/environment_stream.cpp`, replace

```cpp
    externals.pLogger = logger;
    return externals;
}

}

glm::dmat4 compute_ecef_to_map(const GeoAnchor& anchor) {
```

with

```cpp
    externals.pLogger = logger;
    return externals;
}

glm::dmat4 compute_ecef_to_map(const GeoAnchor& anchor) {
```

In `overlume/src/environment_stream.cpp`, replace

```cpp
    double max_tilt_deg, double brightness, std::optional<double> radius_override_m)
    : asyncSystem_(externals.asyncSystem),
```

with

```cpp
    double max_tilt_deg, double brightness, std::optional<double> radius_override_m,
    CustomTileset custom_tileset)
    : asyncSystem_(externals.asyncSystem),
```

In `overlume/src/environment_stream.cpp`, replace

```cpp
      tileRadiusOverrideM_(radius_override_m),
      countingAccessor_(std::move(counting_accessor)) {
```

with

```cpp
      tileRadiusOverrideM_(radius_override_m),
      areaSelection_(custom_tileset.area_selection),
      countingAccessor_(std::move(counting_accessor)) {
```

In `overlume/src/environment_stream.cpp`, replace

```cpp
    options.maximumScreenSpaceError = kStreamMaxSseErr;
    if (asset_id > 0 && !ion_access_token.empty()) {
```

with

```cpp
    options.maximumScreenSpaceError = kStreamMaxSseErr;
    if (custom_tileset.loader) {
        tileset_ = std::make_unique<Cesium3DTilesSelection::Tileset>(
            externals, std::move(custom_tileset.loader), std::move(custom_tileset.root), options);
    } else if (asset_id > 0 && !ion_access_token.empty()) {
```

In `overlume/src/environment_stream.cpp`, replace

```cpp
    for (const auto& tilePtr : result.tilesToRenderThisFrame) {
        const Cesium3DTilesSelection::Tile* tile = tilePtr.get();
        if (tile == nullptr || !tile->isRenderContent()) continue;
```

with

```cpp
    for (const auto& tilePtr : result.tilesToRenderThisFrame) {
        const Cesium3DTilesSelection::Tile* tile = tilePtr.get();
        if (tile == nullptr || !tile->isRenderContent()) continue;
        // Until Done, the render resources are still the worker's LoadThreadGlb, not an asset.
        if (tile->getState() != Cesium3DTilesSelection::TileLoadState::Done) continue;
```

- [ ] **Step 3: The grid fixture and its hooks**

In `overlume/src/environment_stream.cpp`, replace

```cpp
void break_fixture_network_transport(FixtureStreamHandle* handle) {
    if (handle && handle->transport_down) handle->transport_down->store(true);
}
```

with

```cpp
void break_fixture_network_transport(FixtureStreamHandle* handle) {
    if (handle && handle->transport_down) handle->transport_down->store(true);
}

namespace {

constexpr uint32_t kGridCellsPerSide = 4;
constexpr double kGridCellM = 1000.0;

struct GridRequestLog {
    std::mutex mutex;
    std::vector<std::pair<int, int>> leaves;
};

// One triangle in the tile frame, so every leaf becomes a real asset in the scene.
CesiumGltf::Model one_triangle_model() {
    static constexpr float kPositions[9] = {-5, -5, 0, 5, -5, 0, 0, 5, 10};
    static constexpr uint16_t kIndices[3] = {0, 1, 2};

    CesiumGltf::Model model;
    model.asset.version = "2.0";
    model.buffers.resize(1);
    model.bufferViews.resize(2);
    model.accessors.resize(2);
    model.meshes.resize(1);
    model.nodes.resize(1);
    model.scenes.resize(1);

    std::vector<std::byte>& bytes = model.buffers[0].cesium.data;
    bytes.resize(sizeof(kPositions) + sizeof(kIndices));
    std::memcpy(bytes.data(), kPositions, sizeof(kPositions));
    std::memcpy(bytes.data() + sizeof(kPositions), kIndices, sizeof(kIndices));
    model.buffers[0].byteLength = static_cast<int64_t>(bytes.size());

    model.bufferViews[0].buffer = 0;
    model.bufferViews[0].byteLength = sizeof(kPositions);
    model.bufferViews[1].buffer = 0;
    model.bufferViews[1].byteOffset = sizeof(kPositions);
    model.bufferViews[1].byteLength = sizeof(kIndices);

    CesiumGltf::Accessor& position = model.accessors[0];
    position.bufferView = 0;
    position.count = 3;
    position.componentType = CesiumGltf::Accessor::ComponentType::FLOAT;
    position.type = CesiumGltf::Accessor::Type::VEC3;
    position.min = {-5.0, -5.0, 0.0};
    position.max = {5.0, 5.0, 10.0};
    CesiumGltf::Accessor& index = model.accessors[1];
    index.bufferView = 1;
    index.count = 3;
    index.componentType = CesiumGltf::Accessor::ComponentType::UNSIGNED_SHORT;
    index.type = CesiumGltf::Accessor::Type::SCALAR;

    CesiumGltf::MeshPrimitive& primitive = model.meshes[0].primitives.emplace_back();
    primitive.attributes["POSITION"] = 0;
    primitive.indices = 1;
    model.nodes[0].mesh = 0;
    model.scenes[0].nodes.push_back(0);
    model.scene = 0;
    return model;
}

class GridTestLoader final : public Cesium3DTilesSelection::TilesetContentLoader {
public:
    explicit GridTestLoader(std::shared_ptr<GridRequestLog> log) : log_(std::move(log)) {}

    // Only leaves get here: the root holds empty content under a non-loadable ID.
    CesiumAsync::Future<Cesium3DTilesSelection::TileLoadResult> loadTileContent(
        const Cesium3DTilesSelection::TileLoadInput& input) override {
        const auto& id = std::get<CesiumGeometry::QuadtreeTileID>(input.tile.getTileID());
        {
            std::lock_guard<std::mutex> lock(log_->mutex);
            log_->leaves.emplace_back(static_cast<int>(id.x), static_cast<int>(id.y));
        }
        return input.asyncSystem.createResolvedFuture(Cesium3DTilesSelection::TileLoadResult{
            one_triangle_model(),
            CesiumGeometry::Axis::Z,
            std::nullopt,
            std::nullopt,
            std::nullopt,
            input.pAssetAccessor,
            nullptr,
            {},
            Cesium3DTilesSelection::TileLoadResultState::Success,
            input.ellipsoid});
    }

    Cesium3DTilesSelection::TileChildrenResult createTileChildren(
        const Cesium3DTilesSelection::Tile&, const CesiumGeospatial::Ellipsoid&) override {
        return {{}, Cesium3DTilesSelection::TileLoadResultState::Failed};
    }

private:
    std::shared_ptr<GridRequestLog> log_;
};

// The root covers the whole square and refines unconditionally into the 16 leaves. They are built
// here, as EllipsoidTilesetLoader builds its root's children, so the tree is complete before
// cesium's first selection. Leaf (c, r) is QuadtreeTileID(2, c, r): column c west to east, row r
// north to south, each with its own east-north-up frame at its centre.
std::unique_ptr<Cesium3DTilesSelection::Tile> make_grid_root(GridTestLoader* loader,
                                                             const overlume::GeoAnchor& anchor) {
    using CesiumGeospatial::Cartographic;
    using CesiumGeospatial::Ellipsoid;
    using CesiumGeospatial::GlobeTransforms;
    const glm::dmat4 anchorEnu = GlobeTransforms::eastNorthUpToFixedFrame(
        Ellipsoid::WGS84.cartographicToCartesian(Cartographic::fromDegrees(
            anchor.origin_lon_deg, anchor.origin_lat_deg, anchor.origin_height_m)));
    // The point east_m, north_m of the anchor on its tangent plane, as longitude and latitude.
    const auto lonLat = [&](double east_m, double north_m) {
        return *Ellipsoid::WGS84.cartesianToCartographic(
            glm::dvec3(anchorEnu * glm::dvec4(east_m, north_m, 0.0, 1.0)));
    };
    const auto region = [&](double west_m, double south_m, double east_m, double north_m) {
        const Cartographic sw = lonLat(west_m, south_m);
        const Cartographic ne = lonLat(east_m, north_m);
        return CesiumGeospatial::BoundingRegion(
            CesiumGeospatial::GlobeRectangle(sw.longitude, sw.latitude, ne.longitude, ne.latitude),
            anchor.origin_height_m - 10.0, anchor.origin_height_m + 100.0);
    };

    const double half = 0.5 * kGridCellsPerSide * kGridCellM;
    auto root = std::make_unique<Cesium3DTilesSelection::Tile>(
        loader, Cesium3DTilesSelection::TileID(), Cesium3DTilesSelection::TileEmptyContent{});
    root->setUnconditionallyRefine();
    root->setBoundingVolume(region(-half, -half, half, half));

    std::vector<Cesium3DTilesSelection::Tile> leaves;
    leaves.reserve(kGridCellsPerSide * kGridCellsPerSide);
    for (uint32_t c = 0; c < kGridCellsPerSide; ++c) {
        for (uint32_t r = 0; r < kGridCellsPerSide; ++r) {
            const double west = -half + c * kGridCellM;
            const double north = half - r * kGridCellM;
            Cesium3DTilesSelection::Tile& leaf =
                leaves.emplace_back(loader, CesiumGeometry::QuadtreeTileID(2, c, r));
            leaf.setBoundingVolume(region(west, north - kGridCellM, west + kGridCellM, north));
            const Cartographic centre = lonLat(west + 0.5 * kGridCellM, north - 0.5 * kGridCellM);
            leaf.setTransform(
                GlobeTransforms::eastNorthUpToFixedFrame(Ellipsoid::WGS84.cartographicToCartesian(
                    Cartographic(centre.longitude, centre.latitude, anchor.origin_height_m))));
        }
    }
    root->createChildTiles(std::move(leaves));
    return root;
}

}

struct FixtureGridHandle {
    std::shared_ptr<GridRequestLog> log;
};

FixtureGridHandle* install_fixture_grid_source(overlume::VisualRenderer* r,
                                               overlume::GeoAnchor anchor, double radius_m,
                                               bool area_selection) {
    if (r == nullptr) return nullptr;
    auto log = std::make_shared<GridRequestLog>();
    auto loader = std::make_unique<GridTestLoader>(log);
    std::unique_ptr<Cesium3DTilesSelection::Tile> root = make_grid_root(loader.get(), anchor);
    CesiumAsync::AsyncSystem asyncSystem(std::make_shared<overlume::SimpleTaskProcessor>());
    std::shared_ptr<overlume::CountingAssetAccessor> counting;
    Cesium3DTilesSelection::TilesetExternals externals =
        overlume::build_externals(std::make_shared<overlume::FileFixtureAssetAccessor>(),
                                  asyncSystem, "off", overlume::kDefaultMaxCacheItems, &counting);
    auto source = std::make_unique<overlume::StreamingEnvironmentSource>(
        externals, 0, std::string(), std::string(), std::string(), anchor, std::move(counting),
        false, false, 0.0, true, 2.0, 1.0, radius_m,
        overlume::CustomTileset{std::move(loader), std::move(root), area_selection});
    source->set_visible(*r, r->environmentVisible);
    if (r->environmentSource) r->environmentSource->teardown(*r);
    r->environmentSource = std::move(source);
    return new FixtureGridHandle{std::move(log)};
}

std::vector<std::pair<int, int>> fixture_grid_requested_leaves(FixtureGridHandle* handle) {
    if (handle == nullptr) return {};
    std::lock_guard<std::mutex> lock(handle->log->mutex);
    return handle->log->leaves;
}
```

In `overlume/src/environment_test_hooks.hpp`, replace

```cpp
#include <cstdint>
#include <optional>
#include <string>
```

with

```cpp
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>
```

In `overlume/src/environment_test_hooks.hpp`, replace

```cpp
// Make the fixture network reject every tile request with a transport error (no response);
// tileset.json is still served, as with kill_fixture_network.
void break_fixture_network_transport(FixtureStreamHandle* handle);
```

with

```cpp
// Make the fixture network reject every tile request with a transport error (no response);
// tileset.json is still served, as with kill_fixture_network.
void break_fixture_network_transport(FixtureStreamHandle* handle);

// A custom-loader source over a 4 x 4 grid of 1 km cells centred on the anchor: column c runs west
// to east, row r north to south, and the anchor is the corner the four central cells share.
struct FixtureGridHandle;
FixtureGridHandle* install_fixture_grid_source(overlume::VisualRenderer* r,
                                               overlume::GeoAnchor anchor, double radius_m,
                                               bool area_selection);
// Leaf tiles cesium asked the grid loader to load, as (column, row), in request order.
std::vector<std::pair<int, int>> fixture_grid_requested_leaves(FixtureGridHandle* handle);
```

- [ ] **Step 4: Write the failing tests**

`pump_until_loaded` gains a target count (default 1, which keeps every existing call's meaning), and
four tests go at the end of the file. The ego moves the way the existing tests move it: `set_scene`
with a new `ego.position`.

In `overlume/tests/test_environment_stream.cpp`, replace

```cpp
                           std::vector<uint8_t>& buf, int max_frames = 200) {
    uint64_t count = 0;
    for (int i = 0; i < max_frames; ++i) {
        if (!overlume::render_frame(r, pose, {buf.data(), 320, 240})) return count;
        count = overlume::testing::environment_loaded_chunk_count(r);
        if (count > 0) return count;
    }
    return count;
}
```

with

```cpp
                           std::vector<uint8_t>& buf, int max_frames = 200, uint64_t want = 1) {
    uint64_t count = 0;
    for (int i = 0; i < max_frames; ++i) {
        if (!overlume::render_frame(r, pose, {buf.data(), 320, 240})) return count;
        count = overlume::testing::environment_loaded_chunk_count(r);
        if (count >= want) return count;
    }
    return count;
}
```

In `overlume/tests/test_environment_stream.cpp`, replace

```cpp
           "configured on this box (empty default) -- no real ion asset id exists to render, "
           "so this case is reported as not-renderable rather than faked with a made-up id";
}
```

with

```cpp
           "configured on this box (empty default) -- no real ion asset id exists to render, "
           "so this case is reported as not-renderable rather than faked with a made-up id";
}

namespace {

using GridCells = std::vector<std::pair<int, int>>;

// The grid fixture in metres east and north of the anchor: cell (c, r) spans east
// [(c - 2) km, (c - 1) km] and north [(1 - r) km, (2 - r) km]. Returns the cells whose rectangle
// lies within radius_m of (east_m, north_m), in (c, r) order.
GridCells grid_cells_within(double east_m, double north_m, double radius_m) {
    GridCells cells;
    for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r) {
            const double west = (c - 2) * 1000.0, south = (1 - r) * 1000.0;
            const double dx = std::max({west - east_m, 0.0, east_m - (west + 1000.0)});
            const double dy = std::max({south - north_m, 0.0, north_m - (south + 1000.0)});
            if (std::hypot(dx, dy) < radius_m) cells.emplace_back(c, r);
        }
    }
    return cells;
}

GridCells sorted(GridCells cells) {
    std::sort(cells.begin(), cells.end());
    return cells;
}

// kFixtureAnchor has heading 0: map +x is north and +y is west. Map metres differ from ground
// metres by under 0.4% here, and every test point keeps 100 m or more from a cell decision.
void set_ego_east_north(overlume::VisualRenderer* r, double east_m, double north_m) {
    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = overlume::Vec3{north_m, -east_m, 0.0};
    overlume::set_scene(r, s);
}

}

TEST(EnvironmentStream, AreaSelectionLoadsOnlyTheCellsInsideTheRadius) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    auto* grid = overlume::testing::install_fixture_grid_source(r, kFixtureAnchor, 500.0, true);
    ASSERT_NE(grid, nullptr);
    set_ego_east_north(r, 0.0, 0.0);

    std::vector<uint8_t> buf(320u * 240u * 3u);
    ASSERT_GE(pump_until_loaded(r, kStdPose, buf, 300, 4), 4u)
        << "the four cells around the ego never loaded";
    for (int i = 0; i < 30; ++i) overlume::render_frame(r, kStdPose, {buf.data(), 320, 240});

    const GridCells central = {{1, 1}, {1, 2}, {2, 1}, {2, 2}};
    EXPECT_EQ(sorted(overlume::testing::fixture_grid_requested_leaves(grid)), central)
        << "a 500 m sphere at the shared corner touches only the four central cells; any other "
           "request came from a screen-space view or a preloaded sibling";
    EXPECT_EQ(overlume::testing::environment_loaded_chunk_count(r), 4u);
    EXPECT_EQ(overlume::environment_source_state(r), overlume::EnvironmentSourceState::STREAMING);
    EXPECT_EQ(overlume::testing::environment_stream_last_view_frustum_count(r), 1)
        << "the area view must be the only view";
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStream, AreaSelectionRequestsEveryCellTheSphereTouches) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    auto* grid = overlume::testing::install_fixture_grid_source(r, kFixtureAnchor, 1200.0, true);
    ASSERT_NE(grid, nullptr);
    set_ego_east_north(r, 0.0, 0.0);
    const GridCells expected = grid_cells_within(0.0, 0.0, 1200.0);
    ASSERT_EQ(expected.size(), 12u)
        << "test setup: 1200 m reaches the edge cells (1000 m away), not the corners (1414 m)";

    std::vector<uint8_t> buf(320u * 240u * 3u);
    ASSERT_GE(pump_until_loaded(r, kStdPose, buf, 300, 12), 12u);
    for (int i = 0; i < 30; ++i) overlume::render_frame(r, kStdPose, {buf.data(), 320, 240});

    EXPECT_EQ(sorted(overlume::testing::fixture_grid_requested_leaves(grid)), expected);
    EXPECT_EQ(overlume::testing::environment_loaded_chunk_count(r), 12u);
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStream, AreaSelectionFollowsTheEgo) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    auto* grid = overlume::testing::install_fixture_grid_source(r, kFixtureAnchor, 500.0, true);
    ASSERT_NE(grid, nullptr);
    set_ego_east_north(r, 0.0, 0.0);
    std::vector<uint8_t> buf(320u * 240u * 3u);
    ASSERT_GE(pump_until_loaded(r, kStdPose, buf, 300, 4), 4u);
    const size_t before = overlume::testing::fixture_grid_requested_leaves(grid).size();

    set_ego_east_north(r, 1600.0, 0.0);
    const GridCells expected = grid_cells_within(1600.0, 0.0, 500.0);
    ASSERT_EQ(expected, (GridCells{{3, 1}, {3, 2}})) << "test setup";
    uint64_t loaded = 0;
    for (int i = 0; i < 300 && loaded != 2; ++i) {
        overlume::render_frame(r, kStdPose, {buf.data(), 320, 240});
        loaded = overlume::testing::environment_loaded_chunk_count(r);
    }
    for (int i = 0; i < 30; ++i) overlume::render_frame(r, kStdPose, {buf.data(), 320, 240});

    const GridCells all = overlume::testing::fixture_grid_requested_leaves(grid);
    ASSERT_GE(all.size(), before);
    EXPECT_EQ(sorted(GridCells(all.begin() + before, all.end())), expected)
        << "1600 m east of the anchor the sphere touches only the two eastern cells either side "
           "of the ego's row boundary";
    EXPECT_EQ(overlume::testing::environment_loaded_chunk_count(r), 2u)
        << "the central cells must leave the scene once the sphere leaves them";
    overlume::destroy_renderer(r);
}

TEST(EnvironmentStream, WithoutAreaSelectionACustomTilesetKeepsTheScreenSpaceViews) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    auto* grid = overlume::testing::install_fixture_grid_source(r, kFixtureAnchor, 500.0, false);
    ASSERT_NE(grid, nullptr);
    set_ego_east_north(r, 0.0, 0.0);
    std::vector<uint8_t> buf(320u * 240u * 3u);
    ASSERT_GE(pump_until_loaded(r, kStdPose, buf, 300, 4), 4u);
    for (int i = 0; i < 30; ++i) overlume::render_frame(r, kStdPose, {buf.data(), 320, 240});

    EXPECT_GT(overlume::testing::fixture_grid_requested_leaves(grid).size(), 4u)
        << "without area selection cesium keeps today's two views and preloads culled siblings";
    EXPECT_EQ(overlume::testing::environment_stream_last_view_frustum_count(r), 2);
    overlume::destroy_renderer(r);
}
```

- [ ] **Step 5: Run them and see the area tests fail without area selection**

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_environment_stream
overlume/build/test_environment_stream --gtest_filter='EnvironmentStream.*AreaSelection*'
```

Expected: `[  PASSED  ] 1 test.` (`WithoutAreaSelectionACustomTilesetKeepsTheScreenSpaceViews`) and
`3 FAILED TESTS`, because the seam builds the custom tileset but still selects with the two
screen-space views and preloads siblings:
- `AreaSelectionLoadsOnlyTheCellsInsideTheRadius`: the request log holds all 16 cells,

  ```
    sorted(overlume::testing::fixture_grid_requested_leaves(grid))
      Which is: { (0, 0), (0, 1), (0, 2), (0, 3), (1, 0), (1, 1), (1, 2), (1, 3), (2, 0), (2, 1), (2, 2), (2, 3), (3, 0), (3, 1), (3, 2), (3, 3) }
    central
      Which is: { (1, 1), (1, 2), (2, 1), (2, 2) }
  ```

  and `overlume::testing::environment_stream_last_view_frustum_count(r)` is `2`, not `1`.
- `AreaSelectionRequestsEveryCellTheSphereTouches`:
  `Expected: (pump_until_loaded(r, kStdPose, buf, 300, 12)) >= (12u), actual: 8 vs 12` (only cells
  inside the two screen-space views render: the four central ones and the western column; any value
  below 12 is this failure).
- `AreaSelectionFollowsTheEgo`: `sorted(GridCells(all.begin() + before, all.end()))` `Which is: {}`
  against `expected` `Which is: { (3, 1), (3, 2) }` (every cell was already requested as a preloaded
  sibling), then `overlume::testing::environment_loaded_chunk_count(r)` `Which is: 4` against `2u`
  `Which is: 2`.

These are the checks that fail when area selection is reverted. The fourth test is the one that
fails if area selection were applied regardless of the flag (it would see one view and four
requests).

- [ ] **Step 6: Select by area**

`preloadSiblings` goes off with area selection (fog culling needs no switch, API notes), and the
area view replaces both screen-space views; the radius is now computed before the views, and the
radius gate below them is unchanged.

In `overlume/src/environment_stream.cpp`, replace

```cpp
    options.maximumScreenSpaceError = kStreamMaxSseErr;
    if (custom_tileset.loader) {
```

with

```cpp
    options.maximumScreenSpaceError = kStreamMaxSseErr;
    if (areaSelection_) {
        // Otherwise every culled sibling still loads, at preload priority.
        options.preloadSiblings = false;
    }
    if (custom_tileset.loader) {
```

In `overlume/src/environment_stream.cpp`, replace

```cpp
    const std::optional<ViewState> cameraView = camera_view_state(r, mapToEcef_);
    const std::vector<ViewState> views = cameraView.has_value()
                                             ? std::vector<ViewState>{view, *cameraView}
                                             : std::vector<ViewState>{view};
    lastViewFrustumCount_ = views.size();
    const ViewUpdateResult& result = tileset_->updateViewGroup(*viewGroup_, views, deltaSeconds);
    tileset_->loadTiles();
    asyncSystem_.dispatchMainThreadTasks();

    const double radiusM = tileRadiusOverrideM_.value_or(
        static_cast<double>(r.active_theme.environment.tile_radius_m));
    std::unordered_map<const void*, bool> stillPresent;
```

with

```cpp
    const double radiusM = tileRadiusOverrideM_.value_or(
        static_cast<double>(r.active_theme.environment.tile_radius_m));
    std::vector<ViewState> views;
    if (areaSelection_) {
        // The only view: next to a screen-space view, cesium would select by screen-space error.
        const glm::dvec3 egoEcef(mapToEcef_ *
                                 glm::dvec4(ego_map_pos.x, ego_map_pos.y, ego_map_pos.z, 1.0));
        views.emplace_back(Cesium3DTilesSelection::BoundingVolume(
                               CesiumGeometry::BoundingSphere(egoEcef, radiusM)),
                           1.0);
    } else {
        const std::optional<ViewState> cameraView = camera_view_state(r, mapToEcef_);
        views = cameraView.has_value() ? std::vector<ViewState>{view, *cameraView}
                                       : std::vector<ViewState>{view};
    }
    lastViewFrustumCount_ = views.size();
    const ViewUpdateResult& result = tileset_->updateViewGroup(*viewGroup_, views, deltaSeconds);
    tileset_->loadTiles();
    asyncSystem_.dispatchMainThreadTasks();

    std::unordered_map<const void*, bool> stillPresent;
```

- [ ] **Step 7: Run them and see them pass, with the neighbours they could disturb**

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_environment_stream
overlume/build/test_environment_stream \
    --gtest_filter='EnvironmentStream.*AreaSelection*:EnvironmentStream.TransportErrors*:EnvironmentStream.TileSelection*:EnvironmentStream.Radius*:EnvironmentStream.FixtureTiles*:EnvironmentStreamGolden.*'
overlume/build/test_environment_stream --gtest_filter='TokenRedaction.*'
```

Expected: `[  PASSED  ] 11 tests.`, then `[  PASSED  ] 1 test.` The eleven are:
- the four grid tests;
- `TransportErrors…`, Task 6's counter behind the moved `build_externals`;
- `TileSelectionUsesRenderCameraAsSecondFrustum`, `RadiusExcludesTileOutsideConfiguredRadius`,
  `FixtureTilesLoadRenderAsClayAndCount` and the three `EnvironmentStreamGolden` tests, the URL
  path through the render loop that now skips tiles that are not `Done`.

`TokenRedaction…` covers the moved `build_externals` and logger, in its own process (Task 6's API
notes).

- [ ] **Step 8: Prove the exactness test catches `preloadSiblings`**

```bash
cd "$WORKTREE"
sed -i 's/^        options.preloadSiblings = false;$/        options.preloadSiblings = true;  \/\/ REVERT CHECK/' \
    overlume/src/environment_stream.cpp
grep -c 'REVERT CHECK' overlume/src/environment_stream.cpp
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_environment_stream
overlume/build/test_environment_stream \
    --gtest_filter='EnvironmentStream.AreaSelectionLoadsOnlyTheCellsInsideTheRadius'
sed -i 's/^        options.preloadSiblings = true;  \/\/ REVERT CHECK$/        options.preloadSiblings = false;/' \
    overlume/src/environment_stream.cpp
grep -c 'REVERT CHECK' overlume/src/environment_stream.cpp || true
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_environment_stream
```

Expected: the first `grep` prints `1`. The test fails on the request set alone
(`Which is: { (0, 0), … (3, 3) }`, the twelve culled cells loading at preload priority) while its
view-count expectation passes. The second `grep` prints `0`. Then rerun Step 7's commands:
`[  PASSED  ] 11 tests.` and `[  PASSED  ] 1 test.`

- [ ] **Step 9: Prove the follow test catches the `Done` guard**

```bash
cd "$WORKTREE"
sed -i 's|^        if (tile->getState() != Cesium3DTilesSelection::TileLoadState::Done) continue;$|        // REVERT CHECK: Done guard removed|' \
    overlume/src/environment_stream.cpp
grep -c 'REVERT CHECK' overlume/src/environment_stream.cpp
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_environment_stream
overlume/build/test_environment_stream \
    --gtest_filter='EnvironmentStream.AreaSelectionFollowsTheEgo'; echo "exit $?"
sed -i 's|^        // REVERT CHECK: Done guard removed$|        if (tile->getState() != Cesium3DTilesSelection::TileLoadState::Done) continue;|' \
    overlume/src/environment_stream.cpp
grep -c 'REVERT CHECK' overlume/src/environment_stream.cpp || true
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_environment_stream
```

Expected: the first `grep` prints `1`. The run stops after
`[ RUN      ] EnvironmentStream.AreaSelectionFollowsTheEgo` and the GL banner, with no result line:
bash reports a `Segmentation fault` (followed by `(core dumped)` where, as on the development
machine, cores go to apport), and the `echo` prints `exit 139`. The loop
let the leaves' `LoadThreadGlb` pointers into `inScene_`, `prepareInMainThread` freed them, and the
first removal after the ego moves reads one (API notes). The other grid tests can pass without the
line, by counting those pointers, so this test is the guard's check. The second `grep` prints `0`.
Then rerun Step 7's commands: `[  PASSED  ] 11 tests.` and `[  PASSED  ] 1 test.`

- [ ] **Step 10 (only if Step 5 or Step 7 shows a load that never arrives, such as `the four cells
  around the ego never loaded`, beyond the failures listed in Step 5, or a run that dies with a
  signal, such as `Segmentation fault`): diagnose before changing anything**

A run that dies gets a backtrace first (the dying test's name in the filter):

```bash
cd "$WORKTREE"
gdb -batch -ex run -ex bt --args overlume/build/test_environment_stream \
    --gtest_filter='EnvironmentStream.AreaSelectionFollowsTheEgo'
```

Read it this way:
- `received signal SIGSEGV, Segmentation fault.`, with frame `#0` in
  `filament::FScene::removeEntities` or `addEntities` and frame `#1` in
  `overlume::StreamingEnvironmentSource::synthesize_view_and_pump`: `inScene_` holds a pointer that
  is not a live `FilamentAsset`. Check that Step 2's `Done` guard is in the loop, before
  `getRenderResources()` is read (API notes).
- Any other backtrace: stop and report it, with the backtrace, to the orchestrator.

For a load that never arrives, add temporary logging to `synthesize_view_and_pump`. After
`asyncSystem_.dispatchMainThreadTasks();`:

```cpp
    make_redacting_logger()->info(
        "grid diag: views={} root={} visited={} culled={} render={} workerQueue={}", views.size(),
        tileset_->getRootTile() != nullptr, result.tilesVisited, result.tilesCulled,
        result.tilesToRenderThisFrame.size(), result.workerThreadTileLoadQueueLength);
```

and inside the radius-gate loop, after `const double limitM = …;`:

```cpp
        make_redacting_logger()->info(
            "grid diag: centre {:.0f},{:.0f} hypot {:.0f} extent {:.0f} limit {:.0f}",
            centerMap4.x, centerMap4.y, std::hypot(dx, dy), extentM, limitM);
```

and inside `if (areaSelection_) {`, after `views.emplace_back(…);`:

```cpp
        if (const Cesium3DTilesSelection::Tile* root = tileset_->getRootTile()) {
            for (const Cesium3DTilesSelection::Tile& child : root->getChildren()) {
                make_redacting_logger()->info(
                    "grid diag: leaf distance {:.0f} m",
                    std::sqrt(std::get<CesiumGeospatial::BoundingRegion>(child.getBoundingVolume())
                                  .computeDistanceSquaredToPosition(egoEcef)));
            }
        }
```

and inside the render loop, just before the `Done` guard:

```cpp
        make_redacting_logger()->info("grid diag: tile state {}",
                                      static_cast<int>(tile->getState()));
```

Rebuild and rerun the failing test (its name in the filter):

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_environment_stream
overlume/build/test_environment_stream \
    --gtest_filter='EnvironmentStream.AreaSelectionLoadsOnlyTheCellsInsideTheRadius' 2>&1 \
    | grep 'grid diag' | head -60
```

Read the output in this order:
1. `root=false` after the second frame: the root never arrived. `createFromLoader` installs it from
   a main-thread continuation (`TilesetContentManager.cpp:693-725`); check that the constructor took
   the `custom_tileset.loader` branch.
2. `root=true`, `render=0`, `culled=16`: the sphere touches no leaf. The `leaf distance` lines must
   read about 0 m for the four central cells (the ego is their shared corner) and about 1000 m for
   the edge cells; anything else points at `make_grid_root`'s longitude/latitude conversion.
3. `render` ≥ 4 but no `centre` lines: the leaves are listed but none reached `Done` with render
   resources. `tile state` lines that stay at 2 (`ContentLoaded`) mean `finishLoading` never ran
   for them; report that to the orchestrator. Lines at 3 (`Done`) mean `prepareInMainThread`
   returned null: print `overlume::testing::captured_cesium_log_text()` from the test and look for
   `An unexpected error occurred when loading tile` and `is not a valid float32 VEC3`, then log
   `res.errors` after `writeGlb` in `prepareInLoadThread`, and whether `createAsset` returned null
   in `prepareInMainThread`.
4. `centre` lines with `hypot - extent > limit` for a central cell: the radius gate drops it;
   compare with the expected 707 m centre distance and about 1055 m extent.

Remove every `grid diag` line before Step 11 (`git diff` must show only this task's hunks).

- [ ] **Step 11: Format**

```bash
cd "$WORKTREE"
git add overlume/src/environment_stream.hpp overlume/src/environment_stream.cpp \
    overlume/src/environment_test_hooks.hpp overlume/tests/test_environment_stream.cpp
tools/check_format.sh --fix && tools/check_format.sh && git diff --stat
```

Expected: `check_format.sh: reformatted N files with clang-format 23.1.1`, then
`check_format.sh: PASS (N files, clang-format 23.1.1)`, and `git diff --stat` prints nothing (if it
lists a file, `git add` that file again).

- [ ] **Step 12: Gate** (foreground, Bash timeout 600000 ms)

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
ROS_DOMAIN_ID=84 CI_VISUAL_MODE_DOMAIN_ID=84 \
    flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock tools/ci_visual_mode.sh
```

Exit 75 means another lane held the gate lock for 300 s: run the same command again. Expected:
six `PASS` lines and exit 0. The library line counts four entries and four `ok` more than after
Task 6: `out of 352 (336 ok / 6 skipped)`. The other counts are unchanged (363 node tests, 82 WS
bridge tests, 23 goldens all `ok`, 6 examples).

- [ ] **Step 13: Commit**

```bash
cd "$WORKTREE"
git add overlume/src/environment_stream.hpp overlume/src/environment_stream.cpp \
    overlume/src/environment_test_hooks.hpp overlume/tests/test_environment_stream.cpp
git commit -F - <<'EOF'
feat(stream): custom tilesets with area selection

StreamingEnvironmentSource takes an optional CustomTileset (a cesium
TilesetContentLoader, its root tile and an area-selection flag) and
builds it with cesium's custom-loader Tileset constructor, which the
mapbox:// loader needs. With area selection the only view is cesium's
area view, a sphere of the source's radius around the ego with
geometric-error threshold 1.0, so exactly the tiles touching the sphere
load: next to a screen-space view cesium would select every view by
screen-space error. preloadSiblings goes off, since each culled sibling
would still load at preload priority; cesium never fog-culls an area
view, so fog culling needs no switch. The per-tile radius gate is
unchanged, and nothing changes without the flag.

The render loop now skips tiles that are not Done. Until then cesium
still holds the worker's LoadThreadGlb where the loop expects a
FilamentAsset, and prepareInMainThread frees it on the way to Done.
Leaves under a parent that refines unconditionally and never renders
(the grid's root, the mapbox quadtree) are listed in every load state,
so a leaf turning ContentLoaded mid-frame went into the scene and
inScene_, and the removal after the free crashed. URL and ion tilesets
can reach the same path when a not-yet-ready leaf is not kicked because
a sibling rendered last frame; the guard closes it there too.

make_redacting_logger and build_externals leave the anonymous namespace,
bodies unchanged, for the loader's translation unit. A synthetic 4 x 4
grid of 1 km cells proves the selection: a 500 m sphere at the shared
corner requests exactly the four central cells, 1200 m the twelve the
cell rectangles predict, and moving the ego 1.6 km east requests only
the two eastern cells. That last test crashes without the Done guard.

Claude-Session: <the executing session's URL>
EOF
```

---

### Task 8: Node URI composition for `mapbox://`

**Lane:** 5. **Depends on:** Task 0 (the lane forks after it; nothing in Task 0 touches these files).

**Files:**
- Modify: `ros/src/overlume_ros/include/overlume_ros/environment_source_uri.hpp` (the defaults at
  12-16, `compose_environment_source_uri` at 18-86)
- Test: `ros/src/overlume_ros/test/test_environment_source_uri.cpp` (append after the last test,
  line 155)

**Interfaces:**
- Consumes: nothing new. `compose_environment_source_uri` keeps its signature
  (`environment_source_uri.hpp:18-23`); its three callers in `overlume_node.cpp` (on_activate at
  763, the live `environment_source_uri` switch at 878, the network-loss WARN at 1266) are
  untouched.
- Produces:

```cpp
namespace overlume::ros {
inline bool is_ion_source_uri(const std::string& uri);     // starts with "ion://"
inline bool is_mapbox_source_uri(const std::string& uri);  // starts with "mapbox://"
// compose_environment_source_uri, unchanged signature. For mapbox://: fallback= as for every
// scheme; brightness= when != 1.0 and radius= when != 700, as for ion://; never cache=,
// follow_terrain, ground_bias, replaces_ground or max_tilt_deg. ion:// and baked strings are
// byte-identical to today.
}
```

A composed `mapbox://` URI therefore carries only keys Task 2's `parse_mapbox_spec` accepts
(`materials` from the GUI string, `fallback`, `brightness`, `radius`); `cache=<dir>`, which that
parser refuses, is never added.

**API notes:**
- `environment_source_uri.hpp:25-36`: `cache=` and `fallback=` are appended to any scheme today;
  `:37-83`: six `rfind("ion://", 0) == 0` gates (follow_terrain 37, ground_bias 42,
  replaces_ground 51, max_tilt_deg 57, brightness 66, radius 75). The append order is the output
  order, so the gates change in place and keep their order.
- Numbers go through a classic-locale `std::ostringstream` at default precision (`:45-49`):
  1.8 → `1.8`, 250.0 → `250`, 5.0 → `5`.
- The test target is header-only (`ros/src/overlume_ros/CMakeLists.txt:281-283`); its binary is
  `ros/build/overlume_ros/test_environment_source_uri`. It has 18 tests today.

- [ ] **Step 1: Write the failing tests**

Append to the end of `ros/src/overlume_ros/test/test_environment_source_uri.cpp`:

```cpp

TEST(ComposeEnvironmentSourceUri, MapboxGetsFallbackButNeverCache) {
    EXPECT_EQ(overlume::ros::compose_environment_source_uri("/baked/chunks", "mapbox://buildings",
                                                            "/mnt/data/tiles"),
              "mapbox://buildings?fallback=/baked/chunks");
    EXPECT_EQ(overlume::ros::compose_environment_source_uri(
                  "/baked/chunks", "mapbox://buildings?materials=original", "/mnt/data/tiles"),
              "mapbox://buildings?materials=original&fallback=/baked/chunks");
}

TEST(ComposeEnvironmentSourceUri, MapboxNeverGetsTheIonOnlyKeys) {
    EXPECT_EQ(overlume::ros::compose_environment_source_uri("", "mapbox://buildings", "", true, 0.5,
                                                            false, 5.0),
              "mapbox://buildings");
}

TEST(ComposeEnvironmentSourceUri, MapboxGetsBrightnessAndRadiusWhenNotDefault) {
    EXPECT_EQ(overlume::ros::compose_environment_source_uri("", "mapbox://buildings", "", true, 0.3,
                                                            true, 2.0, 1.0, 700.0),
              "mapbox://buildings");
    EXPECT_EQ(overlume::ros::compose_environment_source_uri(
                  "/baked", "mapbox://buildings", "/tiles", true, 0.5, false, 5.0, 1.8, 250.0),
              "mapbox://buildings?fallback=/baked&brightness=1.8&radius=250");
    EXPECT_EQ(overlume::ros::compose_environment_source_uri("", "mapbox://buildings?radius=300", "",
                                                            true, 0.3, true, 2.0, 1.8, 250.0),
              "mapbox://buildings?radius=300&brightness=1.8");
}

TEST(ComposeEnvironmentSourceUri, IonUriWithEveryKnobSetIsUnchanged) {
    EXPECT_EQ(overlume::ros::compose_environment_source_uri("/baked", "ion://96188", "/tiles", true,
                                                            0.5, false, 5.0, 1.8, 250.0),
              "ion://96188?cache=/tiles&fallback=/baked&follow_terrain=on&ground_bias=0.5&"
              "replaces_ground=off&max_tilt_deg=5&brightness=1.8&radius=250");
}
```

What each catches when its behaviour is reverted: `MapboxGetsFallbackButNeverCache` fails if the
`!mapbox` cache gate goes (the URI regains `cache=`); `MapboxGetsBrightnessAndRadiusWhenNotDefault`
fails if brightness or radius go back to ion-only; `MapboxNeverGetsTheIonOnlyKeys` fails if a
terrain or ground key is widened to `mapbox://`; `IonUriWithEveryKnobSetIsUnchanged` pins every ion
key and their order (it passes before and after, together with the 18 existing exact-string
tests).

- [ ] **Step 2: Run the tests and see two fail**

```bash
cd "$WORKTREE"
ros/colcon_build.sh overlume_ros 2>&1 | tail -n 3
source /opt/ros/humble/setup.bash
ros/build/overlume_ros/test_environment_source_uri
```

Expected: the build ends `Summary: 1 package finished`. The run reports `22 tests`, of which two
fail, with these actual values:

```
Which is: "mapbox://buildings?cache=/mnt/data/tiles&fallback=/baked/chunks"
Which is: "mapbox://buildings?materials=original&cache=/mnt/data/tiles&fallback=/baked/chunks"
Which is: "mapbox://buildings?cache=/tiles&fallback=/baked"
Which is: "mapbox://buildings?radius=300"
[  FAILED  ] ComposeEnvironmentSourceUri.MapboxGetsFallbackButNeverCache
[  FAILED  ] ComposeEnvironmentSourceUri.MapboxGetsBrightnessAndRadiusWhenNotDefault
 2 FAILED TESTS
```

- [ ] **Step 3: Replace the six ion gates with two scheme predicates**

In `ros/src/overlume_ros/include/overlume_ros/environment_source_uri.hpp`, apply these eight
replacements (each old block occurs once).

Replace

```cpp
inline constexpr double kTileRadiusDefaultM = 700.0;

inline std::string compose_environment_source_uri(
```

with

```cpp
inline constexpr double kTileRadiusDefaultM = 700.0;

inline bool is_ion_source_uri(const std::string& uri) { return uri.rfind("ion://", 0) == 0; }

inline bool is_mapbox_source_uri(const std::string& uri) { return uri.rfind("mapbox://", 0) == 0; }

inline std::string compose_environment_source_uri(
```

Replace

```cpp
        source_uri = environment_source_uri;
        if (!environment_tile_cache_dir.empty() &&
```

with

```cpp
        const bool ion = is_ion_source_uri(environment_source_uri);
        // Mapbox tiles never reach disk (Product Terms), so never cache=; the Mapbox source holds
        // buildings only, so the terrain and ground keys stay ion-only.
        const bool mapbox = is_mapbox_source_uri(environment_source_uri);
        source_uri = environment_source_uri;
        if (!mapbox && !environment_tile_cache_dir.empty() &&
```

Replace

```cpp
        if (environment_source_uri.rfind("ion://", 0) == 0 &&
            environment_source_uri.find("follow_terrain=") == std::string::npos) {
```

with

```cpp
        if (ion && environment_source_uri.find("follow_terrain=") == std::string::npos) {
```

Replace

```cpp
        if (environment_source_uri.rfind("ion://", 0) == 0 &&
            environment_source_uri.find("ground_bias=") == std::string::npos &&
```

with

```cpp
        if (ion && environment_source_uri.find("ground_bias=") == std::string::npos &&
```

Replace

```cpp
        if (environment_source_uri.rfind("ion://", 0) == 0 &&
            environment_source_uri.find("replaces_ground=") == std::string::npos &&
```

with

```cpp
        if (ion && environment_source_uri.find("replaces_ground=") == std::string::npos &&
```

Replace

```cpp
        if (environment_source_uri.rfind("ion://", 0) == 0 &&
            environment_source_uri.find("max_tilt_deg=") == std::string::npos &&
```

with

```cpp
        if (ion && environment_source_uri.find("max_tilt_deg=") == std::string::npos &&
```

Replace

```cpp
        if (environment_source_uri.rfind("ion://", 0) == 0 &&
            environment_source_uri.find("brightness=") == std::string::npos &&
```

with

```cpp
        if ((ion || mapbox) && environment_source_uri.find("brightness=") == std::string::npos &&
```

Replace

```cpp
        if (environment_source_uri.rfind("ion://", 0) == 0 &&
            environment_source_uri.find("radius=") == std::string::npos &&
```

with

```cpp
        if ((ion || mapbox) && environment_source_uri.find("radius=") == std::string::npos &&
```

Check that no inline gate is left:

```bash
cd "$WORKTREE"
grep -c 'rfind("ion://"' ros/src/overlume_ros/include/overlume_ros/environment_source_uri.hpp
```

Expected: `1`, the predicate (`6` before this step).

- [ ] **Step 4: Run the tests and see them pass**

```bash
cd "$WORKTREE"
ros/colcon_build.sh overlume_ros 2>&1 | tail -n 3
source /opt/ros/humble/setup.bash
ros/build/overlume_ros/test_environment_source_uri
```

Expected: `Summary: 1 package finished`, then `[  PASSED  ] 22 tests.`

- [ ] **Step 5: Format**

```bash
cd "$WORKTREE"
git add ros/src/overlume_ros/include/overlume_ros/environment_source_uri.hpp \
    ros/src/overlume_ros/test/test_environment_source_uri.cpp
tools/check_format.sh --fix
git diff --stat
tools/check_format.sh
```

Expected: `check_format.sh: reformatted … files with clang-format 23.1.1`; `git diff --stat` prints
nothing (the code above is already formatted); then `check_format.sh: PASS (… files, clang-format
23.1.1)`.

- [ ] **Step 6: Gate**

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
ROS_DOMAIN_ID=85 CI_VISUAL_MODE_DOMAIN_ID=85 \
    flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock tools/ci_visual_mode.sh
```

Run in the foreground (Bash timeout 600000 ms). Exit 75 means another lane held the lock for
300 s: run the same command again. Expected (the fork's baseline plus Task 8's four node tests):
- `PASS  POD header check`
- `PASS  library ctest suite  (100% tests passed, 0 tests failed out of 347 (331 ok / 6 skipped))`
- `PASS  node gtests  (Summary: 367 tests, 0 errors, 0 failures, 0 skipped)`
- `PASS  WS bridge pytest suite  (82 passed in …)`, the time varying
- `PASS  golden suite: 23 ok, 0 skipped`
- `PASS  examples  (6 run, all exited 0 with non-empty output)`
- `OVERALL: PASS`, exit 0.

- [ ] **Step 7: Commit**

```bash
cd "$WORKTREE"
git add ros/src/overlume_ros/include/overlume_ros/environment_source_uri.hpp \
    ros/src/overlume_ros/test/test_environment_source_uri.cpp
git commit -F - <<'EOF'
feat(node): compose mapbox:// environment URIs without cache= or ion-only keys

compose_environment_source_uri appended cache=<dir> to every scheme, and
the mapbox:// parser refuses that key because Mapbox tiles may never be
written to disk. Two scheme predicates now replace the six rfind("ion://")
gates: mapbox:// gets fallback= as before, plus brightness= and radius=
when the node's values differ from their defaults, and never cache=,
follow_terrain, ground_bias, replaces_ground or max_tilt_deg (the source
holds buildings only). ion:// and baked strings are byte-identical; a new
case pins an ion URI with every knob set.

Claude-Session: <the executing session's URL>
EOF
```

---

### Task 9: HUD primitives: Latin-1 text, `MeasureText`, `DrawImageRgba`

**Lane:** 5. **Depends on:** Task 8 (lane order only; no code dependency).

**Files:**
- Modify: `ros/src/overlume_ros/include/overlume_ros/hud_overlay.hpp` (the `DrawText`
  declaration at 26-27)
- Modify: `ros/src/overlume_ros/src/hud_overlay.cpp` (atlas constants 25-28, `FontAtlas` 30-37,
  `bake` 39-52, `draw_line` 78-109, after `DrawText` 151-163)
- Test: `ros/src/overlume_ros/test/test_hud_overlay.cpp` (includes 6-12, the stb block 118-121,
  append after the golden test that ends at 228)

**Interfaces:**
- Consumes: the node's pinned `stb_truetype.h` (`CMakeLists.txt:146-155`; implementation compiled
  in `hud_overlay.cpp:6-7`, extern, so the test TU can call it too).
- Produces:

```cpp
namespace overlume::ros {
// Unchanged signature; `text` is now UTF-8: U+0020..U+007E and U+00A0..U+00FF have glyphs, other
// code points advance the pen half a line, bytes starting no 1- or 2-byte sequence are skipped.
bool DrawText(uint8_t* rgb, uint32_t width, uint32_t height, const char* text, float x, float y,
              HudRgb rgb_color, float scale, const char* font_path);
// The pen advance DrawText makes for `text`, in pixels; 0 when the font cannot be used.
float MeasureText(const char* text, float scale, const char* font_path);
// RGBA8 (row 0 at the top, no padding) alpha-blended into the RGB8 frame, top-left at (x, y),
// clipped at the frame edges; a null image draws nothing.
void DrawImageRgba(uint8_t* rgb, uint32_t width, uint32_t height, const uint8_t* rgba,
                   uint32_t image_width, uint32_t image_height, int x, int y);
}
```

**API notes:**
- `stb_truetype.h` (node pin, SHA-256 `ecd30b05…`): `stbtt_bakedchar` 527-531;
  `stbtt_BakeFontBitmap` 533-541, implementation 3813-3857. Glyph i goes to the next free spot
  after glyphs 0..i-1, so baking 224 code points from U+0020 leaves the first 95 exactly where the
  95-glyph bake put them; it returns `-i` when glyph i does not fit and 0 when none fit.
- `stbtt_GetBakedQuad` 549-553, implementation 3859-3878: with `opengl_fillrule = 1`, `q.x0` and
  `q.x1` are whole pixels and `q.x1 - q.x0` is the glyph's atlas width, so `draw_line` samples atlas
  texel `x0 + i` wherever the glyph sits (float error about 1e-4 texel against a 0.5 margin). A
  grown atlas therefore draws the same pixels; the oracle test checks it at scale 2.5.
- `hud_overlay.cpp:54-66`: one cached atlas keyed on (font, scale rounded to 0.1); `:68-76`
  `blend_pixel`; `:78-109` `draw_line`, where a byte outside 32..126 advances `0.5 * pixel_height`
  (80-84) and draws nothing, which is why the Google line says "(c)".
- NotoSans-Regular at 32 px, measured by replaying stb's packing loop (3831-3855) over the font's
  glyph boxes: the Latin-1 bake uses 149 of 512 rows (ASCII alone 74). 512 x 512 holds Latin-1 up
  to HUD scale 2.0 and ASCII alone up to 3.3 (fact-check B2 guessed "~2" for ASCII); 1024 holds
  Latin-1 up to 4.0 and 2048 at least up to 6.0. U+007F..U+009F map to `.notdef`, a 10 x 17 px box at
  32 px, baked but never drawn.
- `DrawText`'s other callers pass ASCII only: `callouts.cpp:57` (`DrawCallout`) and the Google line
  in `overlume_node.cpp:1323-1329`. The goldens that must not move:
  `test/fixtures/hud_overlay_720p_golden.png` (`test_hud_overlay.cpp:166-228`) and
  `callouts_720p_golden.png` (`test_callouts.cpp`, `CalloutsGolden`).
- Test binaries: `ros/build/overlume_ros/test_hud_overlay` (7 tests today: `HudOverlay` 6,
  `HudOverlayGolden` 1) and `ros/build/overlume_ros/test_callouts` (4). Both compile
  `src/hud_overlay.cpp` (`CMakeLists.txt:453-467`). The node is compiled by GCC 11 (Ubuntu 22.04),
  so compile errors read `‘X’ is not a member of ‘overlume::ros’` (typographic quotes under a
  UTF-8 locale, plain `'` under `LANG=C`).

- [ ] **Step 1: Write the ASCII oracle test**

In `ros/src/overlume_ros/test/test_hud_overlay.cpp`, replace

```cpp
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
```

with

```cpp
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>
```

replace

```cpp
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
```

with

```cpp
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include "stb_truetype.h"
```

and append to the end of the file:

```cpp

namespace {

// The pre-Latin-1 renderer, verbatim: an ASCII-only 512 x 512 bake and the old draw loop. DrawText
// must reproduce it byte for byte on ASCII text; that is what keeps the HUD and callout goldens
// where they are.
void draw_ascii_reference(std::vector<uint8_t>& rgb, const std::string& text, float x, float y,
                          float scale, overlume::ros::HudRgb c) {
    constexpr int kDim = 512, kFirst = 32, kCount = 95;
    std::ifstream in(OVERLUME_NODE_FONT_PATH, std::ios::binary);
    const std::vector<unsigned char> ttf((std::istreambuf_iterator<char>(in)),
                                         std::istreambuf_iterator<char>());
    const float pixel_height = 32.0f * (std::round(scale * 10.0f) / 10.0f);
    std::vector<unsigned char> bitmap(static_cast<size_t>(kDim) * kDim, 0);
    std::vector<stbtt_bakedchar> chars(kCount);
    ASSERT_GT(stbtt_BakeFontBitmap(ttf.data(), 0, pixel_height, bitmap.data(), kDim, kDim, kFirst,
                                   kCount, chars.data()),
              0);
    for (const unsigned char ch : text) {
        if (ch < kFirst || ch >= kFirst + kCount) {
            x += pixel_height * 0.5f;
            continue;
        }
        stbtt_aligned_quad q;
        stbtt_GetBakedQuad(chars.data(), kDim, kDim, ch - kFirst, &x, &y, &q, 1);
        const int gx0 = static_cast<int>(std::floor(q.x0));
        const int gx1 = static_cast<int>(std::ceil(q.x1));
        const int gy0 = static_cast<int>(std::floor(q.y0));
        const int gy1 = static_cast<int>(std::ceil(q.y1));
        if (gx1 <= gx0 || gy1 <= gy0) continue;
        const float u_span = q.s1 - q.s0, v_span = q.t1 - q.t0;
        const float gw = static_cast<float>(gx1 - gx0), gh = static_cast<float>(gy1 - gy0);
        for (int py = gy0; py < gy1; ++py) {
            const float v = q.t0 + ((py - gy0) + 0.5f) / gh * v_span;
            const int ay = static_cast<int>(v * kDim);
            if (ay < 0 || ay >= kDim) continue;
            for (int px = gx0; px < gx1; ++px) {
                const float u = q.s0 + ((px - gx0) + 0.5f) / gw * u_span;
                const int ax = static_cast<int>(u * kDim);
                if (ax < 0 || ax >= kDim) continue;
                const float coverage = bitmap[static_cast<size_t>(ay) * kDim + ax] / 255.0f;
                if (px < 0 || py < 0 || px >= static_cast<int>(kWidth) ||
                    py >= static_cast<int>(kHeight) || coverage <= 0.0f) {
                    continue;
                }
                uint8_t* p = rgb.data() + (static_cast<size_t>(py) * kWidth + px) * 3;
                p[0] = static_cast<uint8_t>(p[0] * (1.0f - coverage) + c.r * 255.0f * coverage);
                p[1] = static_cast<uint8_t>(p[1] * (1.0f - coverage) + c.g * 255.0f * coverage);
                p[2] = static_cast<uint8_t>(p[2] * (1.0f - coverage) + c.b * 255.0f * coverage);
            }
        }
    }
}

}

TEST(HudOverlay, AsciiTextRasterisesExactlyAsTheAsciiOnlyAtlasDid) {
    std::string printable;
    for (char ch = 32; ch < 127; ++ch) printable += ch;
    const std::string lines[] = {printable.substr(0, 24), printable.substr(24, 24),
                                 printable.substr(48, 24), printable.substr(72),
                                 "12.3 m/s\tVISUAL\n3.2 m"};
    const overlume::ros::HudRgb color{0.90f, 0.92f, 0.95f};
    for (const float scale : {1.0f, 1.5f, 2.5f}) {
        std::vector<uint8_t> actual(static_cast<size_t>(kWidth) * kHeight * 3, kBackground);
        std::vector<uint8_t> expected = actual;
        for (size_t i = 0; i < 5; ++i) {
            const float y = 90.0f + 130.0f * static_cast<float>(i);
            ASSERT_TRUE(overlume::ros::DrawText(actual.data(), kWidth, kHeight, lines[i].c_str(),
                                                24.0f, y, color, scale, OVERLUME_NODE_FONT_PATH));
            draw_ascii_reference(expected, lines[i], 24.0f, y, scale, color);
        }
        EXPECT_TRUE(actual == expected)
            << "ASCII text rasterised differently at HUD scale " << scale;
    }
}
```

This test guards a non-change: all 95 printable ASCII glyphs plus a tab and a newline (the
half-line advance) at HUD scales 1.0 (the shipped themes), 1.5, and 2.5, where the Latin-1 atlas
must grow to 1024. It fails if the Latin-1 bake moves or reshapes an ASCII glyph or changes an
ASCII advance, for example a code-point offset error, a different pixel height, or a decode that
treats a control byte differently.

- [ ] **Step 2: Run the oracle against the unchanged renderer and see it pass**

```bash
cd "$WORKTREE"
ros/colcon_build.sh overlume_ros 2>&1 | tail -n 3
source /opt/ros/humble/setup.bash
ros/build/overlume_ros/test_hud_overlay \
    --gtest_filter='HudOverlay.AsciiTextRasterisesExactlyAsTheAsciiOnlyAtlasDid'
```

Expected: `Summary: 1 package finished`, then `[  PASSED  ] 1 test.` This proves the oracle
reproduces today's renderer before anything changes.

- [ ] **Step 3: Write the failing tests**

Append to the end of `ros/src/overlume_ros/test/test_hud_overlay.cpp`:

```cpp

TEST(HudOverlay, Latin1CopyrightSignDrawsPixels) {
    std::vector<uint8_t> rgb(static_cast<size_t>(kWidth) * kHeight * 3, kBackground);
    const std::vector<uint8_t> before = rgb;
    ASSERT_TRUE(overlume::ros::DrawText(rgb.data(), kWidth, kHeight, "\xC2\xA9", 100.0f, 100.0f,
                                        overlume::ros::HudRgb{1.0f, 1.0f, 1.0f}, 1.0f,
                                        OVERLUME_NODE_FONT_PATH));
    EXPECT_TRUE(rgb != before) << "U+00A9 drew nothing";
}

TEST(HudOverlay, MeasureTextDecodesUtf8AndSkipsInvalidBytes) {
    const auto width = [](const std::string& text) {
        return overlume::ros::MeasureText(text.c_str(), 1.0f, OVERLUME_NODE_FONT_PATH);
    };
    const std::string lone_continuation = std::string("A\x80") + "B";
    const std::string truncated_pair = "A\xC2";
    const std::string three_byte_euro = std::string("A\xE2\x82\xAC") + "B";
    const std::string c1_control = std::string("A\xC2\x85") + "B";  // U+0085 has no glyph
    EXPECT_GT(width("\xC2\xA9"), 0.0f);
    EXPECT_FLOAT_EQ(width(lone_continuation), width("AB"));
    EXPECT_FLOAT_EQ(width(truncated_pair), width("A"));
    EXPECT_FLOAT_EQ(width(three_byte_euro), width("AB"));
    EXPECT_NEAR(width(c1_control), width("AB") + 16.0f, 1e-3f);
}

TEST(HudOverlay, MeasureTextMatchesTheDrawnExtent) {
    const char* kText = "Improve this map";
    const float short_width = overlume::ros::MeasureText("Improve", 1.0f, OVERLUME_NODE_FONT_PATH);
    const float width = overlume::ros::MeasureText(kText, 1.0f, OVERLUME_NODE_FONT_PATH);
    EXPECT_GT(short_width, 0.0f);
    EXPECT_GT(width, short_width);
    EXPECT_EQ(overlume::ros::MeasureText(kText, 1.0f, "/nonexistent/does_not_exist.ttf"), 0.0f);

    std::vector<uint8_t> rgb(static_cast<size_t>(kWidth) * kHeight * 3, kBackground);
    constexpr float kX = 100.0f;
    ASSERT_TRUE(overlume::ros::DrawText(rgb.data(), kWidth, kHeight, kText, kX, 100.0f,
                                        overlume::ros::HudRgb{1.0f, 1.0f, 1.0f}, 1.0f,
                                        OVERLUME_NODE_FONT_PATH));
    int left = static_cast<int>(kWidth), right = -1;
    for (uint32_t y = 0; y < kHeight; ++y) {
        for (uint32_t x = 0; x < kWidth; ++x) {
            if (rgb[(static_cast<size_t>(y) * kWidth + x) * 3] == kBackground) continue;
            left = std::min(left, static_cast<int>(x));
            right = std::max(right, static_cast<int>(x));
        }
    }
    EXPECT_NEAR(left, kX, 2.0);
    EXPECT_NEAR(right + 1, kX + width, 2.0);
}

TEST(HudOverlay, DrawImageRgbaBlendsAndIgnoresANullImage) {
    constexpr uint32_t kW = 8, kH = 6;
    std::vector<uint8_t> background(static_cast<size_t>(kW) * kH * 3, 0);
    for (size_t i = 2; i < background.size(); i += 3) background[i] = 255;
    // 3 x 2: an opaque red top row; below it red at alpha 128, 128 and 0.
    const uint8_t image[] = {255, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255,
                             255, 0, 0, 128, 255, 0, 0, 128, 255, 0, 0, 0};
    const auto pixel = [](const std::vector<uint8_t>& f, uint32_t x, uint32_t y) {
        const uint8_t* p = &f[(static_cast<size_t>(y) * kW + x) * 3];
        return std::array<int, 3>{p[0], p[1], p[2]};
    };
    const std::array<int, 3> red{255, 0, 0}, blue{0, 0, 255}, half{128, 0, 127};

    std::vector<uint8_t> frame = background;
    overlume::ros::DrawImageRgba(frame.data(), kW, kH, nullptr, 3, 2, 1, 1);
    EXPECT_TRUE(frame == background);

    overlume::ros::DrawImageRgba(frame.data(), kW, kH, image, 3, 2, 1, 1);
    EXPECT_EQ(pixel(frame, 1, 1), red);
    EXPECT_EQ(pixel(frame, 3, 1), red);
    EXPECT_EQ(pixel(frame, 1, 2), half);
    EXPECT_EQ(pixel(frame, 3, 2), blue);
    EXPECT_EQ(pixel(frame, 0, 0), blue);
    EXPECT_EQ(pixel(frame, 4, 1), blue);
}

TEST(HudOverlay, DrawImageRgbaClipsWithoutWrappingOrOverrun) {
    constexpr uint32_t kW = 8, kH = 6;
    // The frame sits between two guard rows of 7s: a missing vertical clip writes into a guard row,
    // and a missing horizontal clip wraps onto the neighbouring row, inside the frame.
    std::vector<uint8_t> buf(static_cast<size_t>(kW) * (kH + 2) * 3, 7);
    uint8_t* frame = buf.data() + kW * 3;
    std::fill(frame, frame + kW * kH * 3, 0);
    const uint8_t red[] = {255, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255,
                           255, 0, 0, 255, 255, 0, 0, 255, 255, 0, 0, 255};
    overlume::ros::DrawImageRgba(frame, kW, kH, red, 3, 2, kW - 2, 1);  // past the right edge
    overlume::ros::DrawImageRgba(frame, kW, kH, red, 3, 2, -1, 3);      // past the left edge
    overlume::ros::DrawImageRgba(frame, kW, kH, red, 3, 2, 2, -1);      // past the top
    overlume::ros::DrawImageRgba(frame, kW, kH, red, 3, 2, 2, kH - 1);  // past the bottom
    std::vector<uint8_t> expected(buf.size(), 7);
    std::fill(expected.begin() + kW * 3, expected.begin() + kW * (kH + 1) * 3, 0);
    const auto paint = [&](uint32_t x, uint32_t y) { expected[((y + 1) * kW + x) * 3] = 255; };
    for (const uint32_t y : {1u, 2u}) {
        paint(6, y);
        paint(7, y);
    }
    for (const uint32_t y : {3u, 4u}) {
        paint(0, y);
        paint(1, y);
    }
    for (const uint32_t x : {2u, 3u, 4u}) {
        paint(x, 0);
        paint(x, kH - 1);
    }
    EXPECT_TRUE(buf == expected);
}
```

What each catches when reverted: `Latin1CopyrightSignDrawsPixels` fails with the ASCII-only atlas
or the byte loop (U+00A9 then draws nothing). `MeasureTextDecodesUtf8AndSkipsInvalidBytes` fails
if invalid bytes advance the pen as they do today (`A\x80B` is wider than `AB`) or if a code point
without a glyph stops advancing half a line (16 px at scale 1). `MeasureTextMatchesTheDrawnExtent`
fails if the measure and the drawn pen diverge (predicted ink: columns 100 to 294 for a pen end of
295.6). `DrawImageRgbaBlendsAndIgnoresANullImage` fails on wrong blending (the expected half-alpha
value is the integer blend `(d * (255 - a) + s * a + 127) / 255`) or if a null image writes.
`DrawImageRgbaClipsWithoutWrappingOrOverrun` fails if any one clip goes (`fx >= width`, `fx < 0`
or the row test). The frame sits between two guard rows, so a missing column clip wraps the blit
onto the neighbouring row inside the frame and a missing row clip writes into a guard row; the
whole-buffer comparison sees either, with no sanitizer and no write outside the buffer. Each of
the three reverts was run against Step 6's code: this test alone fails, with exit code 1.

- [ ] **Step 4: Build and see it fail**

```bash
cd "$WORKTREE"
ros/colcon_build.sh overlume_ros 2>&1 | grep -E "error:|Summary|failed"
```

Expected: errors including `error: ‘MeasureText’ is not a member of ‘overlume::ros’` and
`error: ‘DrawImageRgba’ is not a member of ‘overlume::ros’` in `test_hud_overlay.cpp`, then
`Summary: 0 packages finished` and `1 package failed: overlume_ros`.

- [ ] **Step 5: Declare the primitives**

In `ros/src/overlume_ros/include/overlume_ros/hud_overlay.hpp`, replace

```cpp
bool DrawText(uint8_t* rgb, uint32_t width, uint32_t height, const char* text, float x, float y,
              HudRgb rgb_color, float scale, const char* font_path);
```

with

```cpp
// `text` is UTF-8. U+0020..U+007E and U+00A0..U+00FF have glyphs; any other code point advances
// the pen by half the line height, and bytes that start no one- or two-byte sequence are skipped.
bool DrawText(uint8_t* rgb, uint32_t width, uint32_t height, const char* text, float x, float y,
              HudRgb rgb_color, float scale, const char* font_path);

// How far DrawText advances the pen for `text`, in pixels; 0 when the font cannot be used.
float MeasureText(const char* text, float scale, const char* font_path);

// Alpha-blends an RGBA8 image (row 0 at the top, no row padding) into the RGB8 frame with its
// top-left pixel at (x, y), clipped at the frame edges. A null image draws nothing.
void DrawImageRgba(uint8_t* rgb, uint32_t width, uint32_t height, const uint8_t* rgba,
                   uint32_t image_width, uint32_t image_height, int x, int y);
```

- [ ] **Step 6: Implement the Latin-1 atlas, the UTF-8 decode and the two primitives**

In `ros/src/overlume_ros/src/hud_overlay.cpp`, apply these six replacements.

Replace

```cpp
constexpr int kAtlasDim = 512;
constexpr int kFirstChar = 32;
constexpr int kNumChars = 95;
constexpr float kBasePixelHeight = 32.0f;
```

with

```cpp
constexpr int kAtlasDim = 512;
// U+0020..U+00FF in code-point order: ASCII packs into the same atlas spots as the old ASCII-only
// bake, so its glyphs rasterise exactly as before. DEL and the C1 controls are baked too (as the
// font's .notdef box) but never drawn.
constexpr int kFirstChar = 32;
constexpr int kNumChars = 224;
constexpr float kBasePixelHeight = 32.0f;
```

Replace

```cpp
    float pixel_height = 0.0f;
    std::vector<unsigned char> bitmap;
```

with

```cpp
    float pixel_height = 0.0f;
    int dim = kAtlasDim;
    std::vector<unsigned char> bitmap;
```

Replace

```cpp
    atlas.bitmap.assign(static_cast<size_t>(kAtlasDim) * kAtlasDim, 0);
    atlas.chars.assign(static_cast<size_t>(kNumChars), stbtt_bakedchar{});
    const int rows_used =
        stbtt_BakeFontBitmap(ttf.data(), 0, pixel_height, atlas.bitmap.data(), kAtlasDim, kAtlasDim,
                             kFirstChar, kNumChars, atlas.chars.data());
    return rows_used > 0;
}
```

with

```cpp
    atlas.chars.assign(static_cast<size_t>(kNumChars), stbtt_bakedchar{});
    // NotoSans fits 512 x 512 up to HUD scale 2.0; a larger scale gets a larger atlas instead of
    // losing the HUD (the ASCII-only atlas fitted up to 3.3).
    for (atlas.dim = kAtlasDim; atlas.dim <= 4 * kAtlasDim; atlas.dim *= 2) {
        atlas.bitmap.assign(static_cast<size_t>(atlas.dim) * atlas.dim, 0);
        if (stbtt_BakeFontBitmap(ttf.data(), 0, pixel_height, atlas.bitmap.data(), atlas.dim,
                                 atlas.dim, kFirstChar, kNumChars, atlas.chars.data()) > 0) {
            return true;
        }
    }
    return false;
}
```

Replace

```cpp
float draw_line(uint8_t* rgb, uint32_t width, uint32_t height, const FontAtlas& atlas,
                const std::string& text, float x, float y, float r, float g, float b) {
    for (unsigned char c : text) {
        if (c < kFirstChar || c >= kFirstChar + kNumChars) {
            x += atlas.pixel_height * 0.5f;
            continue;
        }
        stbtt_aligned_quad q;
        stbtt_GetBakedQuad(atlas.chars.data(), kAtlasDim, kAtlasDim, c - kFirstChar, &x, &y, &q, 1);
```

with

```cpp
// The code point starting at text[i], with i moved past it; -1 for a byte that starts no
// well-formed one- or two-byte UTF-8 sequence (that byte alone is skipped).
int next_code_point(const std::string& text, size_t& i) {
    const unsigned char lead = static_cast<unsigned char>(text[i++]);
    if (lead < 0x80) return lead;
    if (lead >= 0xC2 && lead <= 0xDF && i < text.size()) {
        const unsigned char cont = static_cast<unsigned char>(text[i]);
        if ((cont & 0xC0) == 0x80) {
            ++i;
            return ((lead & 0x1F) << 6) | (cont & 0x3F);
        }
    }
    return -1;
}

bool has_glyph(int cp) { return (cp >= 0x20 && cp <= 0x7E) || (cp >= 0xA0 && cp <= 0xFF); }

// Lays `text` out from the pen at (x, y) and returns the pen x after it. Draws only when `rgb` is
// not null, so measuring and drawing advance the pen identically.
float draw_line(uint8_t* rgb, uint32_t width, uint32_t height, const FontAtlas& atlas,
                const std::string& text, float x, float y, float r, float g, float b) {
    for (size_t i = 0; i < text.size();) {
        const int cp = next_code_point(text, i);
        if (cp < 0) continue;
        if (!has_glyph(cp)) {
            x += atlas.pixel_height * 0.5f;
            continue;
        }
        stbtt_aligned_quad q;
        stbtt_GetBakedQuad(atlas.chars.data(), atlas.dim, atlas.dim, cp - kFirstChar, &x, &y, &q,
                           1);
        if (rgb == nullptr) continue;
```

Replace

```cpp
            const int ay = static_cast<int>(v * kAtlasDim);
            if (ay < 0 || ay >= kAtlasDim) continue;
            for (int px = gx0; px < gx1; ++px) {
                const float u = q.s0 + ((px - gx0) + 0.5f) / gw * u_span;
                const int ax = static_cast<int>(u * kAtlasDim);
                if (ax < 0 || ax >= kAtlasDim) continue;
                const float coverage =
                    atlas.bitmap[static_cast<size_t>(ay) * kAtlasDim + ax] / 255.0f;
```

with

```cpp
            const int ay = static_cast<int>(v * atlas.dim);
            if (ay < 0 || ay >= atlas.dim) continue;
            for (int px = gx0; px < gx1; ++px) {
                const float u = q.s0 + ((px - gx0) + 0.5f) / gw * u_span;
                const int ax = static_cast<int>(u * atlas.dim);
                if (ax < 0 || ax >= atlas.dim) continue;
                const float coverage =
                    atlas.bitmap[static_cast<size_t>(ay) * atlas.dim + ax] / 255.0f;
```

Replace

```cpp
void DrawLine(uint8_t* rgb, uint32_t width, uint32_t height, float x0, float y0, float x1, float y1,
              HudRgb rgb_color) {
```

with

```cpp
float MeasureText(const char* text, float scale, const char* font_path) {
    if (text == nullptr || font_path == nullptr || font_path[0] == '\0') return 0.0f;
    const FontAtlas* atlas = get_atlas(font_path, scale > 0.0f ? scale : 1.0f);
    if (atlas == nullptr) return 0.0f;
    return draw_line(nullptr, 0, 0, *atlas, std::string(text), 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

void DrawImageRgba(uint8_t* rgb, uint32_t width, uint32_t height, const uint8_t* rgba,
                   uint32_t image_width, uint32_t image_height, int x, int y) {
    if (rgb == nullptr || rgba == nullptr) return;
    for (uint32_t iy = 0; iy < image_height; ++iy) {
        const int64_t fy = static_cast<int64_t>(y) + iy;
        if (fy < 0 || fy >= static_cast<int64_t>(height)) continue;
        for (uint32_t ix = 0; ix < image_width; ++ix) {
            const int64_t fx = static_cast<int64_t>(x) + ix;
            if (fx < 0 || fx >= static_cast<int64_t>(width)) continue;
            const uint8_t* src = rgba + (static_cast<size_t>(iy) * image_width + ix) * 4;
            uint8_t* dst = rgb + (static_cast<size_t>(fy) * width + static_cast<size_t>(fx)) * 3;
            const uint32_t a = src[3];
            for (int c = 0; c < 3; ++c) {
                dst[c] = static_cast<uint8_t>((dst[c] * (255u - a) + src[c] * a + 127u) / 255u);
            }
        }
    }
}

void DrawLine(uint8_t* rgb, uint32_t width, uint32_t height, float x0, float y0, float x1, float y1,
              HudRgb rgb_color) {
```

For ASCII input the new `draw_line` runs the same float operations in the same order as the old
one (`v * atlas.dim` is `v * 512` while the atlas is 512), which is what Step 7's oracle run
confirms.

- [ ] **Step 7: Build and run; the oracle and both goldens must hold**

```bash
cd "$WORKTREE"
ros/colcon_build.sh overlume_ros 2>&1 | tail -n 3
source /opt/ros/humble/setup.bash
ros/build/overlume_ros/test_hud_overlay
ros/build/overlume_ros/test_callouts
```

Expected: `Summary: 1 package finished`; `test_hud_overlay` ends `[  PASSED  ] 13 tests.`
(including `HudOverlay.AsciiTextRasterisesExactlyAsTheAsciiOnlyAtlasDid` and
`HudOverlayGolden.SyntheticSceneWithHud720pLowPreset`); `test_callouts` ends
`[  PASSED  ] 4 tests.` (including `CalloutsGolden.SyntheticFrameWithVisibleCallout720pLowPreset`).
A failing golden is a finding to report, never a file to overwrite.

- [ ] **Step 8: Format**

```bash
cd "$WORKTREE"
git add ros/src/overlume_ros/include/overlume_ros/hud_overlay.hpp \
    ros/src/overlume_ros/src/hud_overlay.cpp ros/src/overlume_ros/test/test_hud_overlay.cpp
tools/check_format.sh --fix
git diff --stat
tools/check_format.sh
```

Expected: `check_format.sh: reformatted … files with clang-format 23.1.1`; `git diff --stat` prints
nothing; `check_format.sh: PASS (… files, clang-format 23.1.1)`.

- [ ] **Step 9: Gate**

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
ROS_DOMAIN_ID=85 CI_VISUAL_MODE_DOMAIN_ID=85 \
    flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock tools/ci_visual_mode.sh
```

Run in the foreground (Bash timeout 600000 ms). Exit 75 means another lane held the lock for
300 s: run the same command again. Expected (Task 8's gate plus the six new `test_hud_overlay`
tests):
- `PASS  POD header check`
- `PASS  library ctest suite  (100% tests passed, 0 tests failed out of 347 (331 ok / 6 skipped))`
- `PASS  node gtests  (Summary: 373 tests, 0 errors, 0 failures, 0 skipped)`
- `PASS  WS bridge pytest suite  (82 passed in …)`, the time varying
- `PASS  golden suite: 23 ok, 0 skipped`
- `PASS  examples  (6 run, all exited 0 with non-empty output)`
- `OVERALL: PASS`, exit 0.

- [ ] **Step 10: Commit**

```bash
cd "$WORKTREE"
git add ros/src/overlume_ros/include/overlume_ros/hud_overlay.hpp \
    ros/src/overlume_ros/src/hud_overlay.cpp ros/src/overlume_ros/test/test_hud_overlay.cpp
git commit -F - <<'EOF'
feat(node): HUD text decodes UTF-8 over a Latin-1 atlas; add MeasureText and DrawImageRgba

The Mapbox attribution needs a copyright sign, right alignment and a logo
blit. The font atlas now bakes U+0020..U+00FF in code-point order, so ASCII
keeps its atlas spots and rasterises byte for byte as before: a new test
renders every printable ASCII glyph through the old ASCII-only bake and draw
loop at HUD scales 1.0, 1.5 and 2.5 and compares, and the HUD and callout
goldens hold. DrawText decodes one- and two-byte UTF-8 and skips bytes that
start neither; code points without a glyph still advance half a line.
MeasureText returns the pen advance through the same layout code.
DrawImageRgba alpha-blends an RGBA8 image into the RGB8 frame, clipped at
the edges. A scale whose Latin-1 bake overflows 512 x 512 (above 2.0 with
NotoSans) gets a 1024 or 2048 atlas instead of losing the HUD.

Claude-Session: <the executing session's URL>
EOF
```

---

### Task 10: Mapbox attribution on screen

**Lane:** 5. **Depends on:** Task 8 (`is_mapbox_source_uri`) and Task 9 (`MeasureText`,
`DrawImageRgba` and the layout code in `hud_overlay.cpp`).

**Files:**
- Create: `ros/src/overlume_ros/assets/attribution/mapbox-logo.png` (generated in Step 1, Git LFS),
  `ros/src/overlume_ros/assets/attribution/ATTRIBUTION.md`
- Modify: `ros/src/overlume_ros/include/overlume_ros/environment_source_uri.hpp` (includes 6-8,
  append after `fallback_dir_from_source_uri`; Task 8's file too, as the File map gives it)
- Modify: `ros/src/overlume_ros/include/overlume_ros/hud_overlay.hpp`,
  `ros/src/overlume_ros/src/hud_overlay.cpp` (Task 9's files too, as the File map gives them)
- Modify: `ros/src/overlume_ros/include/overlume_ros/overlume_node.hpp` (members 235-237)
- Modify: `ros/src/overlume_ros/src/overlume_node.cpp` (parameters 453-454, the attribution block
  1317-1337)
- Modify: `ros/src/overlume_ros/config/default_params.yaml` (64)
- Modify: `ros/src/overlume_ros/CMakeLists.txt` (fonts install 262-263, `test_environment_source_uri`
  281-283, `overlume_node_test_paths` 300-305)
- Modify: `ros/src/overlume_ros/test/test_asset_paths.py` (case 1 at 170-199)
- Test: `ros/src/overlume_ros/test/test_environment_source_uri.cpp`,
  `ros/src/overlume_ros/test/test_hud_overlay.cpp`

**Interfaces:**
- Consumes: `is_mapbox_source_uri` (Task 8); `MeasureText`, `DrawImageRgba` and the file-local
  `get_atlas`/`draw_line` (Task 9); `environment_effectively_visible(RenderMode, bool)`
  (`scene_assembly.hpp:59`); `overlume::environment_source_state(VisualRenderer*)` and
  `EnvironmentSourceState` (`overlume/scene.h:268-275`); `overlume::get_hud_colors` and `HudColors`
  (`scene.h:199-205`).
- Produces:

```cpp
namespace overlume::ros {
// environment_source_uri.hpp (header-only; now includes "overlume/scene.h")
enum class AttributionKind { NONE, GOOGLE, MAPBOX };
// "© Mapbox © OpenStreetMap Improve this map" in UTF-8
inline constexpr char kMapboxAttributionText[] =
    "\xC2\xA9 Mapbox \xC2\xA9 OpenStreetMap Improve this map";
// NONE unless visible && enabled && state == STREAMING; then MAPBOX for a mapbox:// URI, else
// GOOGLE when the URI contains materials=original (today's test), else NONE.
inline AttributionKind environment_attribution_kind(bool visible, bool enabled,
                                                    const std::string& source_uri,
                                                    overlume::EnvironmentSourceState state);
// hud_overlay.hpp
struct RgbaImage {
    std::vector<uint8_t> rgba;  // row 0 at the top, 4 bytes per pixel, no row padding
    uint32_t width = 0;
    uint32_t height = 0;
};
bool LoadPngRgba(const char* path, RgbaImage& out);  // false (out empty): missing or not a PNG
// Logo bottom-left at native size, text right-aligned bottom-right, ink 8 px from the edges;
// text within 8 px of the logo moves up a line, 8 px above the logo; text wider than the frame
// between the margins is cut at the right edge. False (logo still drawn) when the font cannot be
// used.
bool CompositeAttribution(uint8_t* rgb, uint32_t width, uint32_t height, const RgbaImage& logo,
                          const char* text, HudRgb text_rgb, float scale, const char* font_path);
}
```

  Node parameter `environment_attribution_logo_path` (string, default `""`, configure-time): an
  empty value resolves to `<share>/overlume_ros/assets/attribution/mapbox-logo.png` when readable,
  and the resolved value is written back, exactly as `hud_font_path` resolves. Installed files:
  `share/overlume_ros/assets/attribution/{mapbox-logo.png,ATTRIBUTION.md}`.

**API notes:**
- `overlume_node.cpp:1317-1337`: today's gate is `environment_effectively_visible(...) &&
  environment_attribution_ && environment_source_uri_.find("materials=original") != npos && state
  == STREAMING`, a substring test that `mapbox://buildings?materials=original` would also pass. It
  reads the raw parameter `environment_source_uri_`, which a successful live switch updates
  (`:891`). `:893`, `:1456` and `:1498` reset `environment_attribution_warned_`; they stay as they
  are. The gate ignores `hud_enabled_`, and so does the Mapbox branch (spec section 6).
- `overlume_node.cpp:191-200`: the `hud_font_path` resolution that the logo parameter mirrors
  (`ament_index_cpp::get_package_share_directory`, `std::ifstream(...).good()`, `set_parameter`).
- `stb_image.h` (node pin, SHA-256 `594c2fe3…`, already on the include path of
  `overlume_node_lib`, `test_hud_overlay` and `test_callouts`; `CMakeLists.txt:231, 457, 465`):
  `STB_IMAGE_STATIC` makes every `stbi_*` function `static` (395-398); `STBI_ONLY_PNG` compiles
  only the PNG decoder (549-556); `stbi_load` (427, implementation 1366) with
  `desired_channels = 4` gives RGBA8 with row 0 at the top (flip-on-load defaults off, 1114);
  `stbi_image_free` (491). `test_hud_overlay.cpp:118-121` and `test_callouts.cpp:111-114` define a
  non-static `STB_IMAGE_IMPLEMENTATION` and both executables compile `src/hud_overlay.cpp`
  (`CMakeLists.txt:453, 461`): the static copy in `hud_overlay.cpp` has internal linkage, so the
  two never collide. This is the node's one stb_image implementation TU.
- `stb_image_write.h:176`: `stbi_write_png(filename, w, h, comp, data, stride)`, used by the
  test through its own implementation.
- ROS Humble's googletest: `testing::TempDir()` (`gtest.h:2374`) returns `/tmp/` on Linux
  (`gtest.cc:6136-6156`).
- The logo: `mapbox-gl` 3.33.0 is the latest release on the public npm registry on 2026-10-14; its
  `dist.integrity` (sha512) was verified against the downloaded tarball. The package ships the logo
  only as a `data:image/svg+xml` URI in `package/dist/mapbox-gl.css`. The top-level
  `a.mapboxgl-ctrl-logo` rule (`width: 88px; height: 23px`, `viewBox="0 0 88 23"`) holds the
  default logo; two more copies sit inside `@media (-ms-high-contrast: active)` and
  `(-ms-high-contrast: black-on-white)` and are not used. The SVG is pure paths (no text, no
  fonts): a white mark and wordmark at opacity 0.9 over a black outline at opacity 0.3.
- Rasteriser: neither `rsvg-convert` (`librsvg2-bin`, apt candidate 2.52.5+dfsg-3ubuntu0.2) nor
  `cairosvg` is installed on this Ubuntu 22.04.5 box, so installing either needs sudo or pip. The
  same librsvg 2.52.5 is installed through `gir1.2-rsvg-2.0`, with `python3-gi-cairo` 3.42.1 and
  pycairo 1.20.1 (the GTK stack the vcam GUI already uses), and Step 1 uses it:
  `Rsvg.Handle.new_from_file`, `Handle.render_document(cr, viewport)` (librsvg 2.46 and later),
  `cairo.ImageSurface(FORMAT_ARGB32, 176, 46)`, `write_to_png` (straight-alpha RGBA PNG). The
  exact Step 1 block was run twice on this box and gave identical bytes: SVG SHA-256
  `0b72e543…99aa0` (3012 bytes), PNG SHA-256 `ae67ebad…0c31` (176 x 46 RGBA, 3709 fully
  transparent pixels, 1589 with alpha 200 or more, maximum alpha 230).
- `.gitattributes:3` (`*.png filter=lfs diff=lfs merge=lfs -text`) covers the new PNG;
  `git check-attr filter` on its path reports `lfs`. git-lfs 3.0.2 is installed.
- Layout facts at HUD scale 1.0 (NotoSans): the attribution text advances 524.76 px, its ink spans
  18 px above to 6 px below the baseline (the `p` descender: `yMin` -240 units, 5.64 px). On the
  shipped 1280-wide output the text starts at x = 747, clear of the 176-px logo (which ends at 184,
  plus the 8-px gap), so it stays on the bottom line. Below 725 px of width it moves up, and below
  541 px (in general 16 + 524.76 x the HUD scale) it no longer fits on a line of its own: it then
  keeps its start and loses its end, "Improve this map". The lifted line's 24 ink rows leave
  through the top of a frame shorter than 86 px (8 + 46 + 8 + 24 with the shipped logo). The
  `ponytail:` comment in `CompositeAttribution` names the width limit; Review Focus 5 accepts it
  as known, and the runbook's §5 (Task 12) records it.

- [ ] **Step 1: Add the logo with its provenance**

Create `ros/src/overlume_ros/assets/attribution/ATTRIBUTION.md`:

````markdown
# Attribution assets

## mapbox-logo.png: the Mapbox logo

- **What:** the logo Mapbox GL JS draws in the corner of a map (a white mark and wordmark with a
  dark outline, on a transparent background), rasterised to 176 x 46 px, twice GL JS's 88 x 23 CSS
  size. RGBA8 PNG, stored in Git LFS through the repository's `*.png` rule (`.gitattributes`).
- **Why it is here:** Mapbox Product Terms 1.4 require the Mapbox logo, "© Mapbox",
  "© OpenStreetMap" and "Improve this map" on any map that shows Mapbox content. While a
  `mapbox://` environment source is streaming, the node draws this file at the bottom left and
  that line at the bottom right (parameters `environment_attribution` and
  `environment_attribution_logo_path`).
- **Source:** the npm package `mapbox-gl` 3.33.0 from the public registry, no token needed:
  `https://registry.npmjs.org/mapbox-gl/-/mapbox-gl-3.33.0.tgz`, npm integrity
  `sha512-2HpIrVedccHedolZyMVG/7FXdpgmsLTENgEDsX71x3/gKDGfYMf1FCRY0prntknkn/JXcraUKXlBTv7vkr0EXg==`.
  The SVG is the percent-encoded `data:` URI of the top-level `a.mapboxgl-ctrl-logo` rule in
  `package/dist/mapbox-gl.css`; the two `@media (-ms-high-contrast: ...)` variants are not used.
- **SHA-256:**
  - the extracted SVG (3012 bytes, not committed):
    `0b72e543243c47f8202f6dc7d4b45062939e85f863df2f8feb08bc90a8e99aa0`
  - `mapbox-logo.png`: `ae67ebad76fb0d383d971455b1ddfab8d564822eb77eb6fc54e5a98e50fa0c31`
- **Rasteriser:** librsvg 2.52.5 with cairo 1.16.0 through PyGObject, as Ubuntu 22.04 ships them
  (`gir1.2-rsvg-2.0`, `python3-gi-cairo`; the same librsvg that `rsvg-convert` from
  `librsvg2-bin` wraps). Other versions can write different PNG bytes; record the new hash.
- **Licence and trademark:** the Mapbox logo is a trademark of Mapbox, Inc. and is not covered by
  this repository's Apache-2.0 licence. `mapbox-gl` 3.x is licensed under the Mapbox Terms of
  Service (`package/LICENSE.txt`). Whether this file may stay in the tree is part of the release
  check (decision D5 of `docs/design/2026-10-14-mapbox-buildings-design.md`): it is settled with
  Mapbox before anything from this work is pushed to the public `origin`.

Regenerate from the repository root (needs network; prints the two hashes above):

```bash
(
set -euo pipefail
for tool in curl openssl tar base64 python3 sha256sum; do
    command -v "$tool" > /dev/null || { echo "missing tool: $tool"; exit 1; }
done
python3 -I -c 'import cairo, gi; gi.require_version("Rsvg", "2.0"); from gi.repository import Rsvg' \
    || { echo "missing librsvg bindings: sudo apt-get install gir1.2-rsvg-2.0 python3-gi-cairo"; exit 1; }
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT
curl -sfL -o "$work/mapbox-gl.tgz" https://registry.npmjs.org/mapbox-gl/-/mapbox-gl-3.33.0.tgz
test "sha512-$(openssl dgst -sha512 -binary "$work/mapbox-gl.tgz" | base64 -w0)" = \
    "sha512-2HpIrVedccHedolZyMVG/7FXdpgmsLTENgEDsX71x3/gKDGfYMf1FCRY0prntknkn/JXcraUKXlBTv7vkr0EXg==" \
    || { echo "mapbox-gl-3.33.0.tgz failed its npm integrity check"; exit 1; }
tar -xzf "$work/mapbox-gl.tgz" -C "$work" package/dist/mapbox-gl.css
python3 -I - "$work/package/dist/mapbox-gl.css" "$work/mapbox-logo.svg" <<'PY'
import re
import sys
import urllib.parse

css = open(sys.argv[1], encoding="utf-8").read()
m = re.search(r'a\.mapboxgl-ctrl-logo\{[^}]*?background-image:url\("data:image/svg\+xml;'
              r'charset=utf-8,([^"]+)"\)', css)
head = css[: m.start()] if m else ""
assert m and head.count("{") == head.count("}"), "no top-level a.mapboxgl-ctrl-logo rule"
open(sys.argv[2], "w", encoding="utf-8", newline="").write(urllib.parse.unquote(m.group(1)))
PY
mkdir -p ros/src/overlume_ros/assets/attribution
python3 -I - "$work/mapbox-logo.svg" ros/src/overlume_ros/assets/attribution/mapbox-logo.png <<'PY'
import sys

import cairo
import gi

gi.require_version("Rsvg", "2.0")
from gi.repository import Rsvg  # noqa: E402

handle = Rsvg.Handle.new_from_file(sys.argv[1])
surface = cairo.ImageSurface(cairo.FORMAT_ARGB32, 176, 46)
viewport = Rsvg.Rectangle()
viewport.x, viewport.y, viewport.width, viewport.height = 0, 0, 176, 46
assert handle.render_document(cairo.Context(surface), viewport)
surface.write_to_png(sys.argv[2])
PY
sha256sum "$work/mapbox-logo.svg" ros/src/overlume_ros/assets/attribution/mapbox-logo.png
)
```
````

Then run the recorded command itself, so the file and its provenance cannot drift apart:

````bash
cd "$WORKTREE"
bash <(sed -n '/^```bash$/,/^```$/{/^```/d;p}' ros/src/overlume_ros/assets/attribution/ATTRIBUTION.md)
git check-attr filter ros/src/overlume_ros/assets/attribution/mapbox-logo.png
````

Expected:

```
0b72e543243c47f8202f6dc7d4b45062939e85f863df2f8feb08bc90a8e99aa0  <mktemp dir>/mapbox-logo.svg
ae67ebad76fb0d383d971455b1ddfab8d564822eb77eb6fc54e5a98e50fa0c31  ros/src/overlume_ros/assets/attribution/mapbox-logo.png
ros/src/overlume_ros/assets/attribution/mapbox-logo.png: filter: lfs
```

A `missing ...` line names the tool or the apt packages to install; stop and report it rather than
switching rasteriser (a different rasteriser changes the PNG hash recorded above).

- [ ] **Step 2: CMake: install the asset, give the tests its path and the public headers**

In `ros/src/overlume_ros/CMakeLists.txt`, replace

```cmake
install(DIRECTORY assets/fonts
  DESTINATION share/${PROJECT_NAME}/assets)
```

with

```cmake
install(DIRECTORY assets/fonts
  DESTINATION share/${PROJECT_NAME}/assets)

install(DIRECTORY assets/attribution
  DESTINATION share/${PROJECT_NAME}/assets)
```

replace

```cmake
  ament_add_gtest(test_environment_source_uri test/test_environment_source_uri.cpp)
  target_include_directories(test_environment_source_uri PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
```

with

```cmake
  ament_add_gtest(test_environment_source_uri test/test_environment_source_uri.cpp)
  target_include_directories(test_environment_source_uri PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OVERLUME_DIR}/include)
```

and replace

```cmake
    OVERLUME_NODE_FONT_PATH="${CMAKE_CURRENT_SOURCE_DIR}/assets/fonts/NotoSans-Regular.ttf")
```

with

```cmake
    OVERLUME_NODE_FONT_PATH="${CMAKE_CURRENT_SOURCE_DIR}/assets/fonts/NotoSans-Regular.ttf"
    OVERLUME_NODE_MAPBOX_LOGO_PATH="${CMAKE_CURRENT_SOURCE_DIR}/assets/attribution/mapbox-logo.png")
```

The test target stays header-only: `overlume/scene.h` only declares `EnvironmentSourceState` and
functions the test never calls, so nothing new is linked.

- [ ] **Step 3: Write the failing tests**

In `ros/src/overlume_ros/test/test_environment_source_uri.cpp`, replace

```cpp
#include "overlume_ros/environment_source_uri.hpp"
```

with

```cpp
#include "overlume_ros/environment_source_uri.hpp"

#include "overlume/scene.h"
```

and append to the end of the file:

```cpp

TEST(MapboxAttributionKind, TruthTableOverStateVisibilityParameterAndScheme) {
    using overlume::EnvironmentSourceState;
    using overlume::ros::AttributionKind;
    struct Case {
        const char* uri;
        AttributionKind when_streaming;
    };
    const Case cases[] = {
        {"", AttributionKind::NONE},
        {"/baked/chunks", AttributionKind::NONE},
        {"ion://96188", AttributionKind::NONE},
        {"ion://2275207?materials=original&cache=off", AttributionKind::GOOGLE},
        {"mapbox://buildings", AttributionKind::MAPBOX},
        {"mapbox://buildings?materials=original", AttributionKind::MAPBOX},
    };
    const EnvironmentSourceState states[] = {
        EnvironmentSourceState::NONE, EnvironmentSourceState::BAKED,
        EnvironmentSourceState::STREAMING, EnvironmentSourceState::STREAMING_FALLBACK};
    for (const Case& c : cases) {
        for (const EnvironmentSourceState state : states) {
            for (const bool visible : {false, true}) {
                for (const bool enabled : {false, true}) {
                    const AttributionKind expected =
                        visible && enabled && state == EnvironmentSourceState::STREAMING
                            ? c.when_streaming
                            : AttributionKind::NONE;
                    EXPECT_EQ(
                        overlume::ros::environment_attribution_kind(visible, enabled, c.uri, state),
                        expected)
                        << "uri '" << c.uri << "' state " << static_cast<int>(state) << " visible "
                        << visible << " enabled " << enabled;
                }
            }
        }
    }
}
```

In `ros/src/overlume_ros/test/test_hud_overlay.cpp`, replace

```cpp
#include <gtest/gtest.h>

#include <algorithm>
```

with

```cpp
#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
```

replace

```cpp
#include <iterator>
#include <string>
#include <vector>

#include "overlume/api.h"
#include "overlume/scene.h"
```

with

```cpp
#include <iterator>
#include <limits>
#include <string>
#include <vector>

#include "overlume/api.h"
#include "overlume/scene.h"
#include "overlume_ros/environment_source_uri.hpp"
```

and append to the end of the file:

```cpp

TEST(HudOverlay, LoadPngRgbaDecodesAWrittenPngAndAMissingPathIsNonFatal) {
    const std::string path =
        testing::TempDir() + "overlume_logo_" + std::to_string(getpid()) + ".png";
    // 3 x 2, straight alpha; distinct bytes catch a channel or row swap.
    const uint8_t pixels[] = {255, 0,  0,  255, 0,  255, 0,  128, 0,  0,   255, 0,
                              10,  20, 30, 40,  50, 60,  70, 80,  90, 100, 110, 120};
    ASSERT_NE(stbi_write_png(path.c_str(), 3, 2, 4, pixels, 3 * 4), 0);
    overlume::ros::RgbaImage image;
    const bool loaded = overlume::ros::LoadPngRgba(path.c_str(), image);
    std::remove(path.c_str());
    ASSERT_TRUE(loaded);
    EXPECT_EQ(image.width, 3u);
    EXPECT_EQ(image.height, 2u);
    EXPECT_EQ(image.rgba, std::vector<uint8_t>(std::begin(pixels), std::end(pixels)));

    EXPECT_FALSE(overlume::ros::LoadPngRgba("/nonexistent/mapbox-logo.png", image));
    EXPECT_TRUE(image.rgba.empty());
    EXPECT_EQ(image.width, 0u);
    EXPECT_FALSE(overlume::ros::LoadPngRgba(OVERLUME_NODE_FONT_PATH, image)) << "a TTF is no PNG";
}

TEST(MapboxAttribution, ShippedLogoIsTwiceGlJsSize) {
    overlume::ros::RgbaImage logo;
    ASSERT_TRUE(overlume::ros::LoadPngRgba(OVERLUME_NODE_MAPBOX_LOGO_PATH, logo))
        << "is the Git LFS object fetched? (git lfs pull)";
    EXPECT_EQ(logo.width, 176u);
    EXPECT_EQ(logo.height, 46u);
    size_t clear = 0, solid = 0;
    for (size_t i = 3; i < logo.rgba.size(); i += 4) {
        clear += logo.rgba[i] == 0;
        solid += logo.rgba[i] >= 200;
    }
    EXPECT_GT(clear, 0u) << "the logo keeps its transparent background";
    EXPECT_GT(solid, 0u);
}

namespace {

// An opaque magenta stand-in for the logo, easy to tell from white text.
overlume::ros::RgbaImage magenta_logo(uint32_t w, uint32_t h) {
    overlume::ros::RgbaImage logo;
    logo.width = w;
    logo.height = h;
    for (uint32_t i = 0; i < w * h; ++i) logo.rgba.insert(logo.rgba.end(), {255, 0, 255, 255});
    return logo;
}

struct PixelBox {
    int x0 = std::numeric_limits<int>::max(), y0 = std::numeric_limits<int>::max();
    int x1 = -1, y1 = -1;  // exclusive
    size_t count = 0;
};

// Sorts every pixel that left the background into the logo (exact magenta) or the text.
void classify(const std::vector<uint8_t>& rgb, uint32_t w, uint32_t h, PixelBox& logo,
              PixelBox& text) {
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            const uint8_t* p = &rgb[(static_cast<size_t>(y) * w + x) * 3];
            PixelBox* box = nullptr;
            if (p[0] == 255 && p[1] == 0 && p[2] == 255) {
                box = &logo;
            } else if (p[0] != kBackground || p[1] != kBackground || p[2] != kBackground) {
                box = &text;
            }
            if (box == nullptr) continue;
            box->x0 = std::min(box->x0, static_cast<int>(x));
            box->y0 = std::min(box->y0, static_cast<int>(y));
            box->x1 = std::max(box->x1, static_cast<int>(x) + 1);
            box->y1 = std::max(box->y1, static_cast<int>(y) + 1);
            ++box->count;
        }
    }
}

constexpr overlume::ros::HudRgb kWhite{1.0f, 1.0f, 1.0f};

}

TEST(MapboxAttribution, LogoBottomLeftAndTextBottomRightOnTheMargins) {
    std::vector<uint8_t> rgb(static_cast<size_t>(kWidth) * kHeight * 3, kBackground);
    ASSERT_TRUE(overlume::ros::CompositeAttribution(
        rgb.data(), kWidth, kHeight, magenta_logo(40, 12), overlume::ros::kMapboxAttributionText,
        kWhite, 1.0f, OVERLUME_NODE_FONT_PATH));
    PixelBox logo, text;
    classify(rgb, kWidth, kHeight, logo, text);
    const int w = static_cast<int>(kWidth), h = static_cast<int>(kHeight);
    EXPECT_EQ(logo.count, 40u * 12u);
    EXPECT_EQ(logo.x0, 8);
    EXPECT_EQ(logo.x1, 8 + 40);
    EXPECT_EQ(logo.y0, h - 8 - 12);
    EXPECT_EQ(logo.y1, h - 8);
    const float text_width = overlume::ros::MeasureText(overlume::ros::kMapboxAttributionText, 1.0f,
                                                        OVERLUME_NODE_FONT_PATH);
    EXPECT_NEAR(text.x1, w - 8, 2.0) << "right-aligned on the 8 px margin";
    EXPECT_NEAR(text.x0, w - 8 - text_width, 3.0);
    EXPECT_LE(text.y1, h - 8) << "descenders end on the bottom margin";
    EXPECT_GE(text.y1, h - 8 - 2);
}

TEST(MapboxAttribution, NarrowFrameLiftsTheTextAboveTheLogoAndKeepsBothInside) {
    const float text_width = overlume::ros::MeasureText(overlume::ros::kMapboxAttributionText, 1.0f,
                                                        OVERLUME_NODE_FONT_PATH);
    ASSERT_GT(text_width, 0.0f);
    // Room for the text on a line of its own, not for text and logo side by side.
    const uint32_t frame_w = static_cast<uint32_t>(std::ceil(text_width)) + 16 + 20;
    const uint32_t frame_h = 120;
    std::vector<uint8_t> rgb(static_cast<size_t>(frame_w) * frame_h * 3, kBackground);
    ASSERT_TRUE(overlume::ros::CompositeAttribution(
        rgb.data(), frame_w, frame_h, magenta_logo(40, 12), overlume::ros::kMapboxAttributionText,
        kWhite, 1.0f, OVERLUME_NODE_FONT_PATH));
    PixelBox logo, text;
    classify(rgb, frame_w, frame_h, logo, text);
    EXPECT_EQ(logo.count, 40u * 12u) << "the text must not cover the logo";
    EXPECT_EQ(logo.x0, 8);
    EXPECT_EQ(logo.y1, static_cast<int>(frame_h) - 8);
    EXPECT_LE(text.y1, logo.y0 - 8) << "the text moves up a line, a margin above the logo";
    EXPECT_GT(text.y0, 0) << "the text left the top of the frame";
    EXPECT_GE(text.x0, 8 - 2) << "the text left the left edge";
    EXPECT_NEAR(text.x1, static_cast<int>(frame_w) - 8, 2.0);
}

TEST(MapboxAttribution, MissingLogoOrFontDegradesPieceByPiece) {
    std::vector<uint8_t> rgb(static_cast<size_t>(kWidth) * kHeight * 3, kBackground);
    EXPECT_TRUE(overlume::ros::CompositeAttribution(
        rgb.data(), kWidth, kHeight, overlume::ros::RgbaImage{},
        overlume::ros::kMapboxAttributionText, kWhite, 1.0f, OVERLUME_NODE_FONT_PATH));
    PixelBox no_logo, text;
    classify(rgb, kWidth, kHeight, no_logo, text);
    EXPECT_EQ(no_logo.count, 0u);
    EXPECT_GT(text.count, 0u) << "a missing logo must not suppress the text";

    std::fill(rgb.begin(), rgb.end(), kBackground);
    EXPECT_FALSE(overlume::ros::CompositeAttribution(
        rgb.data(), kWidth, kHeight, magenta_logo(40, 12), overlume::ros::kMapboxAttributionText,
        kWhite, 1.0f, "/nonexistent/does_not_exist.ttf"));
    PixelBox logo, no_text;
    classify(rgb, kWidth, kHeight, logo, no_text);
    EXPECT_EQ(logo.count, 40u * 12u) << "a missing font must not suppress the logo";
    EXPECT_EQ(no_text.count, 0u);
}

TEST(MapboxAttribution, TextUsesTheGivenHudColourAndScale) {
    std::vector<uint8_t> rgb(static_cast<size_t>(kWidth) * kHeight * 3, kBackground);
    ASSERT_TRUE(overlume::ros::CompositeAttribution(
        rgb.data(), kWidth, kHeight, overlume::ros::RgbaImage{},
        overlume::ros::kMapboxAttributionText, overlume::ros::HudRgb{0.0f, 1.0f, 0.0f}, 2.0f,
        OVERLUME_NODE_FONT_PATH));
    PixelBox no_logo, text;
    classify(rgb, kWidth, kHeight, no_logo, text);
    size_t green = 0;
    for (size_t i = 0; i < rgb.size(); i += 3) green += rgb[i + 1] > rgb[i] + 60;
    EXPECT_GT(green, 100u) << "the text is not in the given HUD colour";
    const float width_at_2 = overlume::ros::MeasureText(overlume::ros::kMapboxAttributionText, 2.0f,
                                                        OVERLUME_NODE_FONT_PATH);
    EXPECT_NEAR(text.x0, static_cast<int>(kWidth) - 8 - width_at_2, 4.0)
        << "the text is not laid out at the given HUD scale";
}
```

What each catches when its behaviour is reverted:
- `MapboxAttributionKind.TruthTable…` (6 URIs x 4 states x visibility x parameter = 96 cases):
  fails if the Mapbox branch goes (the `mapbox://` rows expect `MAPBOX`), if the Google substring
  test runs before the scheme check (`mapbox://buildings?materials=original` would give `GOOGLE`;
  `mapbox-original` must never yield Google), or if the `STREAMING`, visibility or parameter
  condition is dropped (`STREAMING_FALLBACK` rows expect `NONE`).
- `LoadPngRgba…` fails if decoding loses a channel, swaps rows or premultiplies, or if a missing
  path or a non-PNG fails to return false with an empty image.
- `ShippedLogoIsTwiceGlJsSize` fails if the asset is missing, is an LFS pointer, or was rasterised
  at another size or without alpha.
- `LogoBottomLeftAndTextBottomRightOnTheMargins` fails if either element leaves its corner or its
  8 px margin. `NarrowFrameLiftsTheTextAboveTheLogoAndKeepsBothInside` is Review Focus 5: if the
  text does not move up, it lands on the logo's rows and covers logo pixels; if it moves too
  little, `text.y1` exceeds the logo's top minus 8; if it leaves the frame, its visible ink touches
  an edge. `MissingLogoOrFontDegradesPieceByPiece` fails if a missing logo suppresses the text or
  a missing font suppresses the logo.
- `TextUsesTheGivenHudColourAndScale` pins the style tokens (spec section 6): green text at scale
  2.0 leaves over 10,000 pixels greener than red by more than 60 and starts at x = 224, against
  222.5 predicted. Drawing white instead of `text_rgb` leaves no such pixel, and laying out at
  scale 1.0 instead of `scale` starts the text near x = 748. Both reverts were run; each fails
  this test alone.

- [ ] **Step 4: Build and see it fail**

```bash
cd "$WORKTREE"
ros/colcon_build.sh overlume_ros 2>&1 | grep -E "error:|Summary|failed"
```

Expected: errors including `‘AttributionKind’ has not been declared in ‘overlume::ros’` and
`‘environment_attribution_kind’ is not a member of ‘overlume::ros’`
(`test_environment_source_uri.cpp`), and/or `‘RgbaImage’ is not a member of ‘overlume::ros’`,
`‘LoadPngRgba’ is not a member of ‘overlume::ros’`, `‘CompositeAttribution’ is not a member of
‘overlume::ros’` and `‘kMapboxAttributionText’ is not a member of ‘overlume::ros’`
(`test_hud_overlay.cpp`). Then `Summary: 0 packages finished` and `1 package failed: overlume_ros`.

- [ ] **Step 5: The attribution gate**

In `ros/src/overlume_ros/include/overlume_ros/environment_source_uri.hpp`, replace

```cpp
#include <locale>
#include <sstream>
#include <string>

namespace overlume::ros {
```

with

```cpp
#include <locale>
#include <sstream>
#include <string>

#include "overlume/scene.h"

namespace overlume::ros {
```

and replace the end of the file

```cpp
    return composed_source_uri.substr(
        value_start, value_end == std::string::npos ? std::string::npos : value_end - value_start);
}

}
```

with

```cpp
    return composed_source_uri.substr(
        value_start, value_end == std::string::npos ? std::string::npos : value_end - value_start);
}

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

}
```

The escapes are UTF-8 for U+00A9; each `\xA9` is followed by a space, so no hex digit is absorbed.

- [ ] **Step 6: Declare the logo loader and the composite**

In `ros/src/overlume_ros/include/overlume_ros/hud_overlay.hpp`, replace

```cpp
#include <cstdint>

#include "overlume/scene.h"
```

with

```cpp
#include <cstdint>
#include <vector>

#include "overlume/scene.h"
```

and replace

```cpp
void DrawImageRgba(uint8_t* rgb, uint32_t width, uint32_t height, const uint8_t* rgba,
                   uint32_t image_width, uint32_t image_height, int x, int y);
```

with

```cpp
void DrawImageRgba(uint8_t* rgb, uint32_t width, uint32_t height, const uint8_t* rgba,
                   uint32_t image_width, uint32_t image_height, int x, int y);

struct RgbaImage {
    std::vector<uint8_t> rgba;  // row 0 at the top, 4 bytes per pixel, no row padding
    uint32_t width = 0;
    uint32_t height = 0;
};

// Decodes a PNG file to RGBA8. False, with `out` left empty, when the file is missing or is not
// a decodable PNG.
bool LoadPngRgba(const char* path, RgbaImage& out);

// A provider attribution: `logo` at native size in the bottom-left corner and `text` right-aligned
// in the bottom-right, the ink of both 8 px from the frame edges. Text that would come within 8 px
// of the logo moves up a line, to 8 px above the logo's top edge. Text wider than the frame between
// the margins starts at the left margin and is cut at the right edge. An empty logo is skipped;
// false (the logo still drawn) when the font cannot be used.
bool CompositeAttribution(uint8_t* rgb, uint32_t width, uint32_t height, const RgbaImage& logo,
                          const char* text, HudRgb text_rgb, float scale, const char* font_path);
```

- [ ] **Step 7: Implement them in `hud_overlay.cpp`**

In `ros/src/overlume_ros/src/hud_overlay.cpp`, apply these four replacements.

Replace

```cpp
#define STB_TRUETYPE_IMPLEMENTATION
#include "stb_truetype.h"

#include <cmath>
```

with

```cpp
#define STB_TRUETYPE_IMPLEMENTATION
#include "stb_truetype.h"

// The node's only stb_image implementation: static, so test executables that compile this file
// beside their own STB_IMAGE_IMPLEMENTATION still link; PNG only, for the attribution logo.
#define STB_IMAGE_STATIC
#define STBI_ONLY_PNG
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <algorithm>
#include <cmath>
```

Replace

```cpp
// Lays `text` out from the pen at (x, y) and returns the pen x after it. Draws only when `rgb` is
// not null, so measuring and drawing advance the pen identically.
float draw_line(uint8_t* rgb, uint32_t width, uint32_t height, const FontAtlas& atlas,
                const std::string& text, float x, float y, float r, float g, float b) {
```

with

```cpp
// Lays `text` out from the pen at (x, y) and returns the pen x after it. Draws only when `rgb` is
// not null, so measuring and drawing advance the pen identically. `ink_bottom`, when given, is
// raised to the lowest glyph edge below y.
float draw_line(uint8_t* rgb, uint32_t width, uint32_t height, const FontAtlas& atlas,
                const std::string& text, float x, float y, float r, float g, float b,
                float* ink_bottom = nullptr) {
```

Replace

```cpp
        stbtt_GetBakedQuad(atlas.chars.data(), atlas.dim, atlas.dim, cp - kFirstChar, &x, &y, &q,
                           1);
        if (rgb == nullptr) continue;
```

with

```cpp
        stbtt_GetBakedQuad(atlas.chars.data(), atlas.dim, atlas.dim, cp - kFirstChar, &x, &y, &q,
                           1);
        if (ink_bottom != nullptr) *ink_bottom = std::max(*ink_bottom, q.y1 - y);
        if (rgb == nullptr) continue;
```

Replace

```cpp
void DrawLine(uint8_t* rgb, uint32_t width, uint32_t height, float x0, float y0, float x1, float y1,
              HudRgb rgb_color) {
```

with

```cpp
bool LoadPngRgba(const char* path, RgbaImage& out) {
    out = RgbaImage{};
    if (path == nullptr || path[0] == '\0') return false;
    int w = 0, h = 0, channels = 0;
    stbi_uc* pixels = stbi_load(path, &w, &h, &channels, 4);
    if (pixels == nullptr) return false;
    out.rgba.assign(pixels, pixels + static_cast<size_t>(w) * static_cast<size_t>(h) * 4);
    out.width = static_cast<uint32_t>(w);
    out.height = static_cast<uint32_t>(h);
    stbi_image_free(pixels);
    return true;
}

bool CompositeAttribution(uint8_t* rgb, uint32_t width, uint32_t height, const RgbaImage& logo,
                          const char* text, HudRgb text_rgb, float scale, const char* font_path) {
    if (rgb == nullptr || width == 0 || height == 0) return false;
    constexpr int kMargin = 8;
    const int logo_top = static_cast<int>(height) - kMargin - static_cast<int>(logo.height);
    if (!logo.rgba.empty()) {
        DrawImageRgba(rgb, width, height, logo.rgba.data(), logo.width, logo.height, kMargin,
                      logo_top);
    }
    if (text == nullptr || font_path == nullptr || font_path[0] == '\0') return false;
    const FontAtlas* atlas = get_atlas(font_path, scale > 0.0f ? scale : 1.0f);
    if (atlas == nullptr) return false;
    const std::string line(text);
    float ink_bottom = 0.0f;
    const float text_width =
        draw_line(nullptr, 0, 0, *atlas, line, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, &ink_bottom);
    const float x = static_cast<float>(width) - kMargin - text_width;
    float y = static_cast<float>(height) - kMargin - ink_bottom;
    if (!logo.rgba.empty() && x < static_cast<float>(kMargin + logo.width + kMargin)) {
        y = static_cast<float>(logo_top - kMargin) - ink_bottom;
    }
    // ponytail: text wider than the frame between the margins keeps its start and loses its end,
    // "Improve this map"; stack two right-aligned lines above the logo if outputs that narrow ship.
    draw_line(rgb, width, height, *atlas, line, std::max(x, static_cast<float>(kMargin)), y,
              text_rgb.r, text_rgb.g, text_rgb.b);
    return true;
}

void DrawLine(uint8_t* rgb, uint32_t width, uint32_t height, float x0, float y0, float x1, float y1,
              HudRgb rgb_color) {
```

`MeasureText`, `CompositeAttribution`, `CompositeHud` and the callouts all use the theme's
`hud.scale` with the same font, so they share the one cached atlas and nothing re-bakes per frame.
With a whole-pixel baseline, `q.y1 - y` is a whole number of pixels (the glyph's bitmap bottom), so
the text's lowest drawn row leaves exactly 8 empty rows below it: above the bottom edge, or above
the logo's top row in the lifted layout.

- [ ] **Step 8: Build and run the tests and see them pass**

```bash
cd "$WORKTREE"
ros/colcon_build.sh overlume_ros 2>&1 | tail -n 3
source /opt/ros/humble/setup.bash
ros/build/overlume_ros/test_environment_source_uri
ros/build/overlume_ros/test_hud_overlay
ros/build/overlume_ros/test_callouts
```

Expected: `Summary: 1 package finished`; then `[  PASSED  ] 23 tests.`, `[  PASSED  ] 19 tests.`
and `[  PASSED  ] 4 tests.` (both goldens still pass).

- [ ] **Step 9: Wire the node: the logo parameter, the decode at configure, the gate**

In `ros/src/overlume_ros/include/overlume_ros/overlume_node.hpp`, replace

```cpp
    bool environment_attribution_{true};
    bool environment_attribution_warned_{false};
    std::string environment_own_asset_uri_;
```

with

```cpp
    bool environment_attribution_{true};
    bool environment_attribution_warned_{false};
    RgbaImage mapbox_logo_;  // decoded once at configure; empty when the file is unusable
    std::string environment_own_asset_uri_;
```

In `ros/src/overlume_ros/src/overlume_node.cpp`, replace

```cpp
    environment_attribution_ = declare_parameter<bool>("environment_attribution", true);
    environment_own_asset_uri_ = declare_parameter<std::string>("environment_own_asset_uri", "");
```

with

```cpp
    environment_attribution_ = declare_parameter<bool>("environment_attribution", true);
    auto logo_path = declare_parameter<std::string>("environment_attribution_logo_path", "");
    if (logo_path.empty()) {
        const std::string installed_logo =
            ament_index_cpp::get_package_share_directory("overlume_ros") +
            "/assets/attribution/mapbox-logo.png";
        if (std::ifstream(installed_logo).good()) {
            logo_path = installed_logo;
        }
    }
    set_parameter(rclcpp::Parameter("environment_attribution_logo_path", logo_path));
    if (!LoadPngRgba(logo_path.c_str(), mapbox_logo_)) {
        RCLCPP_WARN(get_logger(),
                    "environment attribution: Mapbox logo '%s' is missing or not a PNG -- "
                    "mapbox:// sources show the attribution text without the logo",
                    logo_path.c_str());
    }
    environment_own_asset_uri_ = declare_parameter<std::string>("environment_own_asset_uri", "");
```

and replace

```cpp
    if (environment_effectively_visible(render_mode, environment_enabled_) &&
        environment_attribution_ &&
        environment_source_uri_.find("materials=original") != std::string::npos &&
        overlume::environment_source_state(renderer_) ==
            overlume::EnvironmentSourceState::STREAMING) {
        const overlume::HudColors hud_colors = overlume::get_hud_colors(renderer_);
        if (!overlume::ros::DrawText(
                frame_buf_.data(), static_cast<uint32_t>(out_width_),
                static_cast<uint32_t>(out_height_), "3D Tiles data (c) Google", 8.0f,
                static_cast<float>(out_height_) - 8.0f,
                overlume::ros::HudRgb{hud_colors.text_color[0], hud_colors.text_color[1],
                                      hud_colors.text_color[2]},
                hud_colors.scale, hud_font_path_.c_str()) &&
            !environment_attribution_warned_) {
            RCLCPP_WARN(get_logger(),
                        "environment attribution: failed to load/use font '%s' -- "
                        "Google attribution not drawn this run",
                        hud_font_path_.c_str());
            environment_attribution_warned_ = true;
        }
    }
```

with

```cpp
    const AttributionKind attribution = environment_attribution_kind(
        environment_effectively_visible(render_mode, environment_enabled_),
        environment_attribution_, environment_source_uri_,
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
            environment_attribution_warned_ = true;
        }
    }
```

In `ros/src/overlume_ros/config/default_params.yaml`, replace

```yaml
    environment_attribution: true
```

with

```yaml
    environment_attribution: true
    environment_attribution_logo_path: ""
```

The Google branch draws exactly as before; its line stays "(c)" (switching it to "©" is not in
this plan). Its warning gains one word ("Google attribution text not drawn this run"). On the
Mapbox branch a font failure still leaves the logo on screen
(`MissingLogoOrFontDegradesPieceByPiece`), so the message names the text. No test or tool matches
the old wording: `git grep` finds it only at its definition. The live-switch and teardown resets
of `environment_attribution_warned_` (893, 1456, 1498) cover both branches unchanged. The logo
warning is logged once per configure, which is the decode.

- [ ] **Step 10: The asset-path check for the new parameter**

In `ros/src/overlume_ros/test/test_asset_paths.py`, replace

```python
        theme_dir = _param_get("theme_assets_dir")
        hud_font = _param_get("hud_font_path")
        ego_path = _param_get("ego_model_path")
```

with

```python
        theme_dir = _param_get("theme_assets_dir")
        hud_font = _param_get("hud_font_path")
        ego_path = _param_get("ego_model_path")
        logo_path = _param_get("environment_attribution_logo_path")
```

replace

```python
        if not hud_font.startswith(SHARE_DIR):
            print(f"FAIL: hud_font_path '{hud_font}' does not start with "
                  f"installed share dir '{SHARE_DIR}'.", file=sys.stderr)
            return 1
```

with

```python
        if not hud_font.startswith(SHARE_DIR):
            print(f"FAIL: hud_font_path '{hud_font}' does not start with "
                  f"installed share dir '{SHARE_DIR}'.", file=sys.stderr)
            return 1
        if not logo_path.startswith(SHARE_DIR):
            print(f"FAIL: environment_attribution_logo_path '{logo_path}' does not start "
                  f"with installed share dir '{SHARE_DIR}'.", file=sys.stderr)
            return 1
```

replace

```python
        if "theme assets failed to load" in log_text:
            print(f"FAIL: unexpected theme-fallback WARN in log:\n{log_text[-3000:]}",
                  file=sys.stderr)
            return 1
        if ego_resolved and "set_ego_model: failed to load" in log_text:
```

with

```python
        if "theme assets failed to load" in log_text:
            print(f"FAIL: unexpected theme-fallback WARN in log:\n{log_text[-3000:]}",
                  file=sys.stderr)
            return 1
        if "Mapbox logo" in log_text:
            print(f"FAIL: the installed Mapbox logo did not load:\n{log_text[-3000:]}",
                  file=sys.stderr)
            return 1
        if ego_resolved and "set_ego_model: failed to load" in log_text:
```

and replace

```python
        print(f"PASS (1/3): theme_assets_dir='{theme_dir}', hud_font_path='{hud_font}', "
              f"ego_model_path='{ego_path}' (resolved={ego_resolved}), no fallback WARN.")
```

with

```python
        print(f"PASS (1/3): theme_assets_dir='{theme_dir}', hud_font_path='{hud_font}', "
              f"ego_model_path='{ego_path}' (resolved={ego_resolved}), "
              f"environment_attribution_logo_path='{logo_path}', no fallback WARN.")
```

Then build and run it on the lane's ROS domain (it launches the node, so it needs the GPU; it is
not part of the gate):

```bash
cd "$WORKTREE"
ros/colcon_build.sh overlume_ros 2>&1 | tail -n 3
source /opt/ros/humble/setup.bash && source ros/install/setup.bash
ROS_DOMAIN_ID=85 python3 ros/src/overlume_ros/test/test_asset_paths.py
```

Expected: `Summary: 1 package finished`; then `PASS (1/3): …,
environment_attribution_logo_path='…/share/overlume_ros/assets/attribution/mapbox-logo.png', no
fallback WARN.`, `PASS (2/3): …`, `PASS (3/3): …` and `PASS: overlume_node asset-path resolution
test passed.` Without the default resolution or the install rule, case 1 prints
`FAIL: environment_attribution_logo_path '' does not start with installed share dir …`, and with an
unreadable logo it prints `FAIL: the installed Mapbox logo did not load`.

The node's wiring (Step 9's two hunks in `timer_callback` and `on_configure`) has compile-level
coverage only: the gate and the composite have unit tests above, and the Task 17 rehearsal checks
logo and text on screen while streaming and gone in fallback (spec section 11).

- [ ] **Step 11: Format**

```bash
cd "$WORKTREE"
git add ros/src/overlume_ros/include/overlume_ros/environment_source_uri.hpp \
    ros/src/overlume_ros/include/overlume_ros/hud_overlay.hpp \
    ros/src/overlume_ros/src/hud_overlay.cpp \
    ros/src/overlume_ros/include/overlume_ros/overlume_node.hpp \
    ros/src/overlume_ros/src/overlume_node.cpp \
    ros/src/overlume_ros/test/test_environment_source_uri.cpp \
    ros/src/overlume_ros/test/test_hud_overlay.cpp
tools/check_format.sh --fix
git diff --stat
tools/check_format.sh
```

Expected: `check_format.sh: reformatted … files with clang-format 23.1.1`; `git diff --stat` prints
nothing; `check_format.sh: PASS (… files, clang-format 23.1.1)`.

- [ ] **Step 12: Gate**

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
ROS_DOMAIN_ID=85 CI_VISUAL_MODE_DOMAIN_ID=85 \
    flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock tools/ci_visual_mode.sh
```

Run in the foreground (Bash timeout 600000 ms). Exit 75 means another lane held the lock for
300 s: run the same command again. Expected (Task 9's gate plus seven node tests: one in
`test_environment_source_uri`, six in `test_hud_overlay`):
- `PASS  POD header check`
- `PASS  library ctest suite  (100% tests passed, 0 tests failed out of 347 (331 ok / 6 skipped))`
- `PASS  node gtests  (Summary: 380 tests, 0 errors, 0 failures, 0 skipped)`
- `PASS  WS bridge pytest suite  (82 passed in …)`, the time varying
- `PASS  golden suite: 23 ok, 0 skipped`
- `PASS  examples  (6 run, all exited 0 with non-empty output)`
- `OVERALL: PASS`, exit 0.

- [ ] **Step 13: Commit**

```bash
cd "$WORKTREE"
git add ros/src/overlume_ros/assets/attribution/mapbox-logo.png \
    ros/src/overlume_ros/assets/attribution/ATTRIBUTION.md \
    ros/src/overlume_ros/include/overlume_ros/environment_source_uri.hpp \
    ros/src/overlume_ros/include/overlume_ros/hud_overlay.hpp \
    ros/src/overlume_ros/src/hud_overlay.cpp \
    ros/src/overlume_ros/include/overlume_ros/overlume_node.hpp \
    ros/src/overlume_ros/src/overlume_node.cpp \
    ros/src/overlume_ros/config/default_params.yaml \
    ros/src/overlume_ros/CMakeLists.txt \
    ros/src/overlume_ros/test/test_environment_source_uri.cpp \
    ros/src/overlume_ros/test/test_hud_overlay.cpp \
    ros/src/overlume_ros/test/test_asset_paths.py
git commit -F - <<'EOF'
feat(node): Mapbox attribution on screen while a mapbox:// source streams

Mapbox Product Terms 1.4 require the Mapbox logo, "© Mapbox",
"© OpenStreetMap" and "Improve this map" on any frame that shows Mapbox
content. environment_attribution_kind() replaces the inline Google gate and
checks the scheme first, because the old substring test for
materials=original would have put Google's line on mapbox-original. It
returns MAPBOX only in STREAMING (in STREAMING_FALLBACK only baked chunks
show) and, like the Google line, ignores hud_enabled. CompositeAttribution
draws the logo bottom-left at native size and the text right-aligned
bottom-right, both 8 px from the edges, in the theme's HUD text colour and
scale; text that would come within 8 px of the logo moves up a line, above
it (Review Focus 5). The logo, assets/attribution/mapbox-logo.png (Git LFS),
is rasterised from the SVG that mapbox-gl 3.33.0 ships, by the command
recorded in its ATTRIBUTION.md with both SHA-256s, and is decoded once at
configure through a static, PNG-only stb_image in hud_overlay.cpp, so test
executables with their own stb_image still link. The new parameter
environment_attribution_logo_path resolves "" to the installed file like
hud_font_path; a missing or unreadable logo warns once and the text is
still drawn. Keeping the logo in the tree is part of the release check
(spec D5).

Claude-Session: <the executing session's URL>
EOF
git lfs ls-files | grep mapbox-logo
```

Expected: the commit succeeds, and `git lfs ls-files` prints
`ae67ebad76 * ros/src/overlume_ros/assets/attribution/mapbox-logo.png` (the LFS object id is the
PNG's SHA-256).

---

### Task 11: GUI sources, token check, rig token refresh

**Lane:** 6. **Depends on:** Task 0 (the lane forks after it). Task 12 documents what this task
ships; Task 17 uses `--arm mapbox` and the preflight.

**Files:**
- Modify: `tools/vcam_gui.py:106-111` (the environment source list and its URI table)
- Modify: `tools/vcam_ws_bridge.py:33-42` (docstring), `:87-92` (list and URI table), `:249-251`
  (the error string)
- Modify: `tools/validate_visual_mode.sh:177-184` (the token refresh)
- Create: `tools/check_mapbox_token.sh` (mode 755)
- Test: modify `tools/test_vcam_ws_bridge.py:135-141` (parametrize, URI pin), `:548-557`
  (resolve), `:573-587` (lockstep), and append after `:729` (refresh); create
  `tools/test_check_mapbox_token.py`

**Interfaces:**
- Consumes:
  - `def parse_cmd(text: str)` (`tools/vcam_ws_bridge.py:149`): its `set_environment_source`
    branch accepts a name only if it is in `ENVIRONMENT_PRESETS` (`:247-252`); the handler then
    sends `ENVIRONMENT_PRESET_URIS[preset]` through `set_environment_source_async(self, uri: str)`
    (`:451`, `:633-635`).
  - `def resolve_environment_preset(source_uri, own_asset_uri)` (`tools/vcam_gui.py:127-138`):
    exact `==` against `ENVIRONMENT_PRESET_URIS_FIXED`, else `"clipped"` or `None`.
  - `def initial_environment_sensitivity(preset: str) -> bool` (`tools/vcam_gui.py:113-119`):
    true for every name except `"clipped"`, so the new rows are selectable with no new code.
  - The node stores the requested, uncomposed `environment_source_uri` after a successful switch
    (`ros/src/overlume_ros/src/overlume_node.cpp:889-891`), which is what exact matching sees.
  - Global Constraints: the URIs `mapbox://buildings` and `mapbox://buildings?materials=original`,
    the two endpoints, and the token check's output contract (one `PASS:` line and exit 0, a
    `FAIL:` line and exit 1, a `SKIP:` line and exit 2 without a request).
- Produces:
  - GUI: `ENVIRONMENT_PRESETS == ["baked", "osm", "clipped", "google", "mapbox", "mapbox-original"]`;
    bridge: the same names in its `ENVIRONMENT_PRESETS` set. Both URI tables gain
    `"mapbox": "mapbox://buildings"` and `"mapbox-original": "mapbox://buildings?materials=original"`.
  - The WS command `{"cmd": "set_environment_source", "preset": "mapbox" | "mapbox-original"}`.
  - `tools/check_mapbox_token.sh`: no arguments; prints exactly one line on stdout, nothing on
    stderr:
    - `PASS: token retrieves the Downtown Dubai tiles (landmark tile HTTP 200, footprint tile HTTP 200)`,
      exit 0;
    - `FAIL: <landmark>, <footprint> (see docs/runbooks/mapbox.md)`, exit 1, where each part is
      `<label> HTTP <code>` or `<label> transport error (curl exit <n>)` for the labels
      `landmark tile` and `footprint tile`;
    - `SKIP: MAPBOX_TOKEN is not set (see docs/runbooks/mapbox.md)`, exit 2, with no request sent.
  - `tools/validate_visual_mode.sh` exports a `MAPBOX_TOKEN` re-read from a login shell before the
    node starts, and prints
    `[token] MAPBOX_TOKEN in this shell differs from the login shell's -- using the login shell's`
    when the values differ.

**API notes:**
- `tools/vcam_gui.py:391-394`: combo rows are built from `ENVIRONMENT_PRESETS` in list order;
  `:471-478` sends the row name as `preset`; `:564-567` selects a row by name.
- `tools/validate_logger_session.sh:144-170`: `--arm <name>` sends the name to the bridge
  unchanged, so `--arm mapbox` needs only the bridge entry.
- `tools/test_vcam_ws_bridge.py:16-18` imports the bridge without ROS; the GUI tests use
  `pytest.importorskip("gi")` (`:549`, `:560`, `:566`, `:580`), so only the gate (system Python
  with `gi`) runs them; `test_mapbox_entries_map_to_the_buildings_uris` needs no `gi`.
- `tools/ci_visual_mode.sh:136-147`: stage 4 runs only `tools/test_vcam_ws_bridge.py`, with
  `-p no:anyio`. On this host pytest 6.2.5 fails to start without that flag
  (`ModuleNotFoundError: No module named '_pytest.scope'`, from the user-site anyio plugin), so
  every pytest command below carries it, as the Execution section requires.
- `overlume/scripts/check_cesium_token.sh`: the model. One line per run: `SKIP: … is not set
  (see docs/runbooks/cesium.md)` and exit 2 (`:7-10`), `FAIL: …` and exit 1, `PASS: token
  retrieves … (endpoint 200, tileset 200)` and exit 0 (`:29`). It feeds curl a config on stdin
  (`--config -`) so the secret never reaches argv (`:11-13`); the new script does the same with
  `url = "…"`.
- curl 7.81.0 (`curl --version` on this host): with `-s` and `-w '%{http_code}'`, a refused
  connection prints `000`, exits 7 and writes nothing to stderr (checked on loopback). `-S` would
  print error text, so the script reports only curl's exit status.
- `tools/validate_visual_mode.sh:177-184`: today's refresh captures the login shell's whole
  stdout. Ubuntu's `/etc/bash.bashrc:43-47` prints a sudo hint on stdout when
  `$HOME/.sudo_as_admin_successful` is absent; measured with a fake `HOME`, the captured value
  was 127 bytes for a 22-byte token, while reading it back on fd 3 was exact. The loop below
  reads both tokens back on fd 3. Against this host's real login files, the
  fd-3 read-back of a non-secret variable (`HOME`) came back exact in about 1 s, so no startup
  file here leaves a background job holding fd 3 (such a job would make the read wait).
- bash 5.1: `${!name}` (indirect expansion) and the `[![:space:]]` trim idiom; `printenv A B`
  prints the values in argument order.
- The footprint tile z16 42830/28025 lies inside z14 10707/7006 (x 42828-42831, y 28024-28027).

- [ ] **Step 1: Extend the bridge and GUI tests**

In `tools/test_vcam_ws_bridge.py`, replace

```python
@pytest.mark.parametrize("preset", ["baked", "osm", "clipped", "google"])
def test_set_environment_source_valid(preset):
```

with

```python
@pytest.mark.parametrize("preset", [
    "baked", "osm", "clipped", "google", "mapbox", "mapbox-original",
])
def test_set_environment_source_valid(preset):
```

Replace

```python
def test_google_preset_ships_cache_off():
    assert ENVIRONMENT_PRESET_URIS["google"].endswith("cache=off")
```

with

```python
def test_google_preset_ships_cache_off():
    assert ENVIRONMENT_PRESET_URIS["google"].endswith("cache=off")

def test_mapbox_entries_map_to_the_buildings_uris():
    # No cache=off: the mapbox:// source never writes a cache.
    assert ENVIRONMENT_PRESET_URIS["mapbox"] == "mapbox://buildings"
    assert ENVIRONMENT_PRESET_URIS["mapbox-original"] == "mapbox://buildings?materials=original"
```

In `test_resolve_environment_preset`, replace

```python
    assert resolve_environment_preset(
        "ion://2275207?materials=original&cache=off", "") == "google"
```

with

```python
    assert resolve_environment_preset(
        "ion://2275207?materials=original&cache=off", "") == "google"
    assert resolve_environment_preset("mapbox://buildings", "") == "mapbox"
    assert resolve_environment_preset(
        "mapbox://buildings?materials=original", "") == "mapbox-original"
```

In `test_gui_and_bridge_environment_preset_tables_agree`, replace

```python
    assert set(GUI_PRESETS) == ENVIRONMENT_PRESETS
```

with

```python
    assert GUI_PRESETS == ["baked", "osm", "clipped", "google", "mapbox", "mapbox-original"]
    assert set(GUI_PRESETS) == ENVIRONMENT_PRESETS
    assert set(ENVIRONMENT_PRESET_URIS) == ENVIRONMENT_PRESETS - {"clipped"}
```

Append to the end of the file, after `test_launcher_runs_preflight_after_teardown_and_checks_the_bridge`
(the block starts with the two blank lines that separate sections in this file):

```python


# --- token refresh (tools/validate_visual_mode.sh) ---------------------------------------------
# Runs the launcher's refresh block on its own. HOME is a temp dir, so the login shell it starts
# reads only the fake profile below, never a real token.

def test_launcher_refreshes_both_tokens_from_the_login_shell(tmp_path):
    import subprocess
    with open(os.path.join(os.path.dirname(__file__), "validate_visual_mode.sh")) as f:
        script = f.read()
    start = script.index("for _token_name in ")
    marker = "unset _fresh_token _token_name\n"
    end = script.index(marker, start) + len(marker)
    (tmp_path / ".bash_profile").write_text(
        "echo 'startup files may print'\n"
        "export CESIUM_ION_TOKEN=ion-test-token-not-real\n"
        "export MAPBOX_TOKEN=pk.test-token-not-real\n")
    env = {k: v for k, v in os.environ.items() if k not in ("CESIUM_ION_TOKEN", "MAPBOX_TOKEN")}
    env.update(HOME=str(tmp_path), MAPBOX_TOKEN="pk.stale-token-not-real")
    r = subprocess.run(
        ["bash", "-c", "set -euo pipefail\n" + script[start:end] +
         'printenv CESIUM_ION_TOKEN MAPBOX_TOKEN > "$HOME/tokens"\n'],
        env=env, stdin=subprocess.DEVNULL, capture_output=True, text=True, timeout=60)
    assert r.returncode == 0, r.stderr
    assert (tmp_path / "tokens").read_text().split() == [
        "ion-test-token-not-real", "pk.test-token-not-real"]
    assert ("[token] MAPBOX_TOKEN in this shell differs from the login shell's -- "
            "using the login shell's") in r.stdout
    assert "token-not-real" not in r.stdout + r.stderr
```

Reverts each test catches:
- `test_set_environment_source_valid[mapbox|mapbox-original]`: a name missing from the bridge's
  `ENVIRONMENT_PRESETS`.
- `test_mapbox_entries_map_to_the_buildings_uris`: a bridge URI missing or changed (no `gi`
  needed, so hosted lint catches it too).
- `test_resolve_environment_preset`: a GUI URI missing or changed.
- `test_gui_and_bridge_environment_preset_tables_agree`: one file edited without the other, the
  GUI order changed, or a name with no URI (the handler's `ENVIRONMENT_PRESET_URIS[preset]` would
  raise `KeyError`).
- `test_launcher_refreshes_both_tokens_from_the_login_shell`: `MAPBOX_TOKEN` dropped from the
  loop; the single-token block restored (`ValueError: substring not found`); the fd-3 read-back
  reverted to a stdout capture (the fake profile's `echo` then lands in the token: checked, 1
  failed); a `[token]` line that prints a value.

- [ ] **Step 2: Write the offline token-check test**

Create `tools/test_check_mapbox_token.py`:

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

# Logs its argv and the config it reads on stdin, then answers like `curl -w '%{http_code}'`.
# STUB_LANDMARK and STUB_FOOTPRINT hold "<http code>:<curl exit status>".
STUB_CURL = r"""#!/bin/sh
config=$(cat)
printf 'argv: %s\n%s\n' "$*" "$config" >> "$STUB_LOG"
case "$config" in
    *3dtiles*) reply=$STUB_LANDMARK ;;
    *) reply=$STUB_FOOTPRINT ;;
esac
printf '%s' "${reply%%:*}"
exit "${reply#*:}"
"""


def run_check(tmp_path, token, landmark="200:0", footprint="200:0"):
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    (bin_dir / "curl").write_text(STUB_CURL)
    (bin_dir / "curl").chmod(0o755)
    env = {k: v for k, v in os.environ.items() if k != "MAPBOX_TOKEN"}
    env.update(PATH=f"{bin_dir}{os.pathsep}{env['PATH']}", STUB_LOG=str(tmp_path / "curl.log"),
               STUB_LANDMARK=landmark, STUB_FOOTPRINT=footprint)
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


def test_both_tiles_200_pass(tmp_path):
    r = run_check(tmp_path, TOKEN)
    assert r.returncode == 0, r.stdout
    assert r.stdout == ("PASS: token retrieves the Downtown Dubai tiles "
                        "(landmark tile HTTP 200, footprint tile HTTP 200)\n")
    assert TOKEN not in r.stdout + r.stderr


@pytest.mark.parametrize("landmark,footprint,codes", [
    ("401:0", "401:0", "landmark tile HTTP 401, footprint tile HTTP 401"),
    ("204:0", "200:0", "landmark tile HTTP 204, footprint tile HTTP 200"),
    ("200:0", "000:28", "landmark tile HTTP 200, footprint tile transport error (curl exit 28)"),
], ids=["both-401", "landmark-204", "footprint-timeout"])
def test_anything_but_two_200s_fails(tmp_path, landmark, footprint, codes):
    r = run_check(tmp_path, TOKEN, landmark, footprint)
    assert r.returncode == 1
    assert r.stdout == f"FAIL: {codes} (see docs/runbooks/mapbox.md)\n"


def test_token_reaches_curl_only_on_stdin_and_trimmed(tmp_path):
    # The library trims surrounding whitespace, so the preflight sends what the node will send.
    r = run_check(tmp_path, f" {TOKEN}\n", landmark="401:0", footprint="000:6")
    assert TOKEN not in r.stdout + r.stderr
    log = (tmp_path / "curl.log").read_text().splitlines()
    assert [line for line in log if line.startswith("argv: ")] == [
        "argv: -s --max-time 30 --config - -o /dev/null -w %{http_code}"] * 2
    assert [line for line in log if line.startswith("url = ")] == [
        f'url = "{LANDMARK_URL}?access_token={TOKEN}"',
        f'url = "{FOOTPRINT_URL}?access_token={TOKEN}"',
    ]
```

Reverts each test catches (each checked on a scratch copy): dropping the unset check (the stub
logs a request: 3 failed); dropping the trim (2 failed); the URL and token on curl's command line
instead of stdin (3 failed); `set -x` (the token reaches stderr: 5 failed); exit 1 instead of 2
on `SKIP:` (3 failed); exit 0 after `FAIL:` (3 failed); any HTTP code counted as a pass (2
failed).

- [ ] **Step 3: Run the new tests and see them fail**

```bash
cd "$WORKTREE"
python3 -m pytest tools/test_vcam_ws_bridge.py -q -p no:anyio -k "environment or mapbox or token"
python3 -m pytest tools/test_check_mapbox_token.py -q -p no:anyio
```

The `-k` filter keeps out the three end-to-end tests that launch ROS nodes; the gate runs the
whole file. Expected, first command (pytest may append a truncated reason to a `FAILED` line):

```text
FAILED tools/test_vcam_ws_bridge.py::test_set_environment_source_valid[mapbox]
FAILED tools/test_vcam_ws_bridge.py::test_set_environment_source_valid[mapbox-original]
FAILED tools/test_vcam_ws_bridge.py::test_mapbox_entries_map_to_the_buildings_uris
FAILED tools/test_vcam_ws_bridge.py::test_resolve_environment_preset
FAILED tools/test_vcam_ws_bridge.py::test_gui_and_bridge_environment_preset_tables_agree
FAILED tools/test_vcam_ws_bridge.py::test_launcher_refreshes_both_tokens_from_the_login_shell
6 failed, 13 passed, 67 deselected in …s
```

with these causes in the `E` lines:
`ValueError: set_environment_source: preset must be "baked", "osm", "clipped" or "google"`,
`KeyError: 'mapbox'`, `AssertionError: assert None == 'mapbox'`, the list diff
`- ['baked', 'osm', 'clipped', 'google', 'mapbox', 'mapbox-original']` /
`+ ['baked', 'osm', 'clipped', 'google']`, and `ValueError: substring not found`.

Expected, second command: `8 failed in …s`. The `E` lines are `assert 127 == 2` for the three
`test_missing_token_skips_without_a_request` cases, `assert 127 == 1` for the three
`test_anything_but_two_200s_fails` cases and `assert 127 == 0` for `test_both_tiles_200_pass`,
each with `check_mapbox_token.sh: No such file or directory` in the captured stderr, and
`test_token_reaches_curl_only_on_stdin_and_trimmed` fails with
`FileNotFoundError: [Errno 2] No such file or directory: '…/curl.log'`.

- [ ] **Step 4: Add the two entries to the GUI and the bridge**

In `tools/vcam_gui.py`, replace

```python
ENVIRONMENT_PRESETS = ["baked", "osm", "clipped", "google"]
ENVIRONMENT_PRESET_URIS_FIXED = {
    "baked": "",
    "osm": "ion://96188",
    "google": "ion://2275207?materials=original&cache=off",
}
```

with

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

In `tools/vcam_ws_bridge.py`, replace the docstring lines

```text
    {"cmd": "set_environment_source", "preset": "baked"|"osm"|"clipped"|"google"}
        (VM-096 -- resolves `preset` to overlume_node's
        `environment_source_uri` string server-side: "baked"->"",
        "osm"->"ion://96188",
        "google"->"ion://2275207?materials=original&cache=off" (cache=off
        is the shipped compliance lever, see default_params.yaml/cesium.md),
        "clipped"->that node's OWN `environment_own_asset_uri` param
```

with

```text
    {"cmd": "set_environment_source", "preset": "baked"|"osm"|"clipped"|"google"|
     "mapbox"|"mapbox-original"}
        (VM-096 -- resolves `preset` to overlume_node's
        `environment_source_uri` string server-side: "baked"->"",
        "osm"->"ion://96188",
        "google"->"ion://2275207?materials=original&cache=off" (cache=off
        is the shipped compliance lever, see default_params.yaml/cesium.md),
        "mapbox"->"mapbox://buildings",
        "mapbox-original"->"mapbox://buildings?materials=original" (no
        cache=off: that source never caches, see docs/runbooks/mapbox.md),
        "clipped"->that node's OWN `environment_own_asset_uri` param
```

replace

```python
ENVIRONMENT_PRESETS = {"baked", "osm", "clipped", "google"}
ENVIRONMENT_PRESET_URIS = {
    "baked": "",
    "osm": "ion://96188",
    "google": "ion://2275207?materials=original&cache=off",
}
```

with

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

and replace

```python
            raise ValueError(
                'set_environment_source: preset must be "baked", "osm", "clipped" or "google"')
```

with

```python
            raise ValueError(
                'set_environment_source: preset must be "baked", "osm", "clipped", "google", '
                '"mapbox" or "mapbox-original"')
```

Nothing else changes: the handler's `else` branch (`:633-634`) and the GUI's send, sensitivity
and reverse map are generic. Names stay as they are (`ENVIRONMENT_PRESETS`, the wire field
`preset`).

- [ ] **Step 5: Refresh `MAPBOX_TOKEN` in the launcher**

In `tools/validate_visual_mode.sh`, replace

```bash
_fresh_token="$(env -u CESIUM_ION_TOKEN bash -lic 'printf %s "${CESIUM_ION_TOKEN:-}"' 2>/dev/null || true)"
if [[ -n "${_fresh_token}" ]]; then
    if [[ -n "${CESIUM_ION_TOKEN:-}" && "${CESIUM_ION_TOKEN}" != "${_fresh_token}" ]]; then
        echo "[token] CESIUM_ION_TOKEN in this shell differs from the login shell's -- using the login shell's"
    fi
    export CESIUM_ION_TOKEN="${_fresh_token}"
fi
unset _fresh_token
```

with

```bash
# A long-lived terminal can hold a pre-rotation token, so read each one from a fresh login shell.
# The value comes back on fd 3: the startup files' own output must not end up in the token.
for _token_name in CESIUM_ION_TOKEN MAPBOX_TOKEN; do
    _fresh_token="$(env -u "${_token_name}" bash -lic "printf %s \"\${${_token_name}:-}\" >&3" \
        3>&1 >/dev/null 2>&1 || true)"
    if [[ -n "${_fresh_token}" ]]; then
        if [[ -n "${!_token_name:-}" && "${!_token_name}" != "${_fresh_token}" ]]; then
            echo "[token] ${_token_name} in this shell differs from the login shell's --" \
                "using the login shell's"
        fi
        export "${_token_name}=${_fresh_token}"
    fi
done
unset _fresh_token _token_name
```

`echo` joins its two arguments with one space, so the message text for `CESIUM_ION_TOKEN` is
unchanged. Neither branch prints a value.

- [ ] **Step 6: Create the preflight**

Create `tools/check_mapbox_token.sh`:

```bash
#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

# Demo preflight for the mapbox:// environment source (docs/runbooks/mapbox.md), shaped like
# overlume/scripts/check_cesium_token.sh: requests the Downtown Dubai landmark tile and one
# footprint tile inside it with $MAPBOX_TOKEN and prints one line. PASS (exit 0) when both answer
# 200, FAIL (exit 1) otherwise, SKIP (exit 2) with no request when the token is unset. It prints
# HTTP codes and curl exit statuses only, never a URL, the token or a body. Not part of the gate:
# it needs the network and a real token.

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

# Downtown Dubai: z14 10707/7006 holds landmark models; z16 42830/28025 is one of its footprint
# tiles. The endpoints are the ones the library requests.
landmarks="https://api.mapbox.com/3dtiles/v1/mapbox.mapbox-3dbuildings-v1-meshopt"
footprints="https://api.mapbox.com/v4/mapbox.mapbox-streets-v8"
status=0
landmark=$(check "landmark tile" "${landmarks}/14/10707/7006.glb") || status=1
footprint=$(check "footprint tile" "${footprints}/16/42830/28025.vector.pbf") || status=1
if [ "${status}" -eq 0 ]; then
    echo "PASS: token retrieves the Downtown Dubai tiles (${landmark}, ${footprint})"
    exit 0
fi
echo "FAIL: ${landmark}, ${footprint} (see docs/runbooks/mapbox.md)"
exit 1
```

```bash
cd "$WORKTREE"
chmod 755 tools/check_mapbox_token.sh
```

Never run it in this task: it would use the real token and the network. Its tests use the stub.

- [ ] **Step 7: Run the tests and the lint checks and see them pass**

```bash
cd "$WORKTREE"
python3 -m pytest tools/test_vcam_ws_bridge.py -q -p no:anyio -k "environment or mapbox or token"
python3 -m pytest tools/test_check_mapbox_token.py -q -p no:anyio
shellcheck --severity=error tools/*.sh
shellcheck tools/check_mapbox_token.sh
python3 -m py_compile tools/*.py
```

Expected: `19 passed, 67 deselected in …s`; `8 passed in …s`; no output from either
shellcheck command (the new script is clean at every severity, and the launcher change adds no
finding; it removes the old block's SC2016 note); no output from `py_compile`.

- [ ] **Step 8: Format**

```bash
cd "$WORKTREE"
git add tools/vcam_gui.py tools/vcam_ws_bridge.py tools/test_vcam_ws_bridge.py \
    tools/check_mapbox_token.sh tools/test_check_mapbox_token.py tools/validate_visual_mode.sh
tools/check_format.sh --fix
tools/check_spdx.sh
git ls-files -s tools/check_mapbox_token.sh | cut -c1-6
git status --short
```

Expected: `check_format.sh: reformatted 196 files with clang-format 23.1.1` (this task has no
C/C++ change, so nothing moves); `check_spdx.sh: PASS (354 files carry the SPDX identifier)`;
`100755`; and exactly the six staged files:

```text
A  tools/check_mapbox_token.sh
A  tools/test_check_mapbox_token.py
M  tools/test_vcam_ws_bridge.py
M  tools/validate_visual_mode.sh
M  tools/vcam_gui.py
M  tools/vcam_ws_bridge.py
```

Any other line (a C/C++ file rewritten by `--fix`) is a deviation to report, not to commit.

- [ ] **Step 9: Gate**

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
ROS_DOMAIN_ID=86 CI_VISUAL_MODE_DOMAIN_ID=86 \
    flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock tools/ci_visual_mode.sh
```

Expected: six `PASS` lines (POD header check, library ctest suite, node gtests, WS bridge pytest
suite, golden suite, examples), `OVERALL: PASS` and exit 0, with the baseline counts otherwise
unchanged (347 ctest entries, 363 node tests, 23 goldens all `ok`, 6 examples). The bridge line
reads `PASS  WS bridge pytest suite  (86 passed in …)`: the 82 tests of today's file plus the two
new parametrize cases, the URI pin and the refresh test. Exit 75 means another lane held the gate
lock for 300 s: run the same command again.

- [ ] **Step 10: Commit**

```bash
cd "$WORKTREE"
git add tools/vcam_gui.py tools/vcam_ws_bridge.py tools/test_vcam_ws_bridge.py \
    tools/check_mapbox_token.sh tools/test_check_mapbox_token.py tools/validate_visual_mode.sh
git commit -F - <<'EOF'
feat(tools): mapbox and mapbox-original environment sources, token preflight

The vcam GUI's Environment tiles list and the WS bridge gain two entries,
kept in lockstep: mapbox -> mapbox://buildings (the theme's clay, like
osm) and mapbox-original -> mapbox://buildings?materials=original
(Mapbox's look, like google). Neither carries cache=off: the mapbox://
source never caches. No new control and no renames: the bridge handler is
generic, so validate_logger_session.sh --arm mapbox works unchanged. The
lockstep test now also pins the GUI order and that every name but clipped
has a URI.

tools/check_mapbox_token.sh is the demo preflight, shaped like
check_cesium_token.sh. It requests the Downtown Dubai landmark tile and
one footprint tile inside it and prints one PASS, FAIL or SKIP line with
the HTTP codes only (exit 0, 1 or 2). It keeps the token off curl's argv
and skips without a request when MAPBOX_TOKEN is unset.
tools/test_check_mapbox_token.py drives it through a stub curl, offline.
The gate does not run that file; run it with
python3 -m pytest tools/test_check_mapbox_token.py -q -p no:anyio.

validate_visual_mode.sh refreshes MAPBOX_TOKEN from a login shell as it
refreshes CESIUM_ION_TOKEN, in one loop. The value now comes back on
fd 3: the old stdout capture took in whatever the startup files printed
(Ubuntu's sudo hint, for a user who has never run sudo, turned a 22-byte
token into 127 bytes).

Claude-Session: <the executing session's URL>
EOF
```

### Task 12: Docs

**Lane:** 6. **Depends on:** Task 11 (the runbook documents its entries, its script and the
refresh). Task 17 fills the runbook's §7.2 values table.

**Files:**
- Create: `docs/runbooks/mapbox.md`
- Modify: `README.md:170` (Environment sources: a bullet after the Cesium one)
- Modify: `docs/README.md:28-30` (runbook index: an entry after `cesium.md`). The plan's own entry
  under `## Plans (docs/plans/)` is not this task's: it comes with the plan's commit (Global
  Constraints).
- Modify: `docs/status.md:123-125` (open items 21 and 22, before "Known gaps"; appended
  after item 20, because other entries cite open items by number)
- Modify: `docs/runbooks/environment_bake.md:106-109` (one sentence before "Output contract")
- Modify: `CHANGELOG.md:8-10` (`[Unreleased]`: the Added entry and two Fixed entries)
- Modify: `NOTICE:300-303` (a Mapbox block between the Google block and the fixtures block)
- Test: `python3 tools/check_docs_links.py`

**Interfaces:**
- Consumes: the Global Constraints (URI keys, endpoints, attribution rule, local-only rule); the
  spec's §4 (node composition, Task 8), §5 failure table (Tasks 6 and 13, including the log line
  `Mapbox rejected MAPBOX_TOKEN (HTTP 401)`), §6 (Task 10: `environment_attribution_logo_path`,
  `ros/src/overlume_ros/assets/attribution/mapbox-logo.png`, and its narrow-frame limits: the
  text moves above the logo below 725 px of width at HUD scale 1.0 and is cut off below 541 px),
  §8 (Task 16: `MapboxLive.*` behind `OVERLUME_LIVE_MAPBOX=1` in `test_mapbox_buildings`), §9
  (the demo); Task 11's GUI entries, preflight output (one `PASS:`, `FAIL:` or `SKIP:` line, exit
  0, 1 or 2) and `[token]` line. Names from other lanes appear only in code spans, so the link
  check does not depend on their files;
  Task 17's runbook-fact check re-checks them against the merged code.
- Produces: `docs/runbooks/mapbox.md`, sections 1 to 8. Its §7.2 table holds exactly these three
  rows, each once in the file, and Task 17 replaces each row by exact match with the rehearsal's
  value (Interface contract, "Texts other tasks match verbatim"):

  ```markdown
  | `geo_datum_lat_deg` | filled in by Task 17's rehearsal |
  | `geo_datum_lon_deg` | filled in by Task 17's rehearsal |
  | `geo_datum_heading_deg` | filled in by Task 17's rehearsal |
  ```

**API notes** (facts the prose states, and where they come from):
- `tools/check_docs_links.py:31-43` scans `README.md`, `CHANGELOG.md`, `docs/README.md`,
  `docs/status.md` and every `*.md` under `docs/runbooks`; `:46-47` skips code spans and fences;
  `:120-127` prints `Broken links (N):` and one `  <file>: -> <target>` per link, else
  `OK: no broken links`. It does not scan `NOTICE`.
- `ros/src/overlume_ros/src/overlume_node.cpp:242-251`: `geo_datum_lat_deg`,
  `geo_datum_lon_deg`, `geo_datum_heading_deg`, `geo_datum_height_m` (all `double`, NaN) and
  `geo_anchor_height_offset_m` (0.0); `:252-266` a complete override solves at configure and logs
  `geo-anchor solved (geo_datum override): …`; `:268-272` a partial one logs an ERROR and is
  ignored; `:288-292` the GPS solve logs `geo-anchor solved: --anchor-lat … --anchor-heading-deg …`.
- `overlume_node.cpp:758-781`: `on_activate` arms the launch-time URI only when the anchor is
  already solved (`:762`), logging `environment source armed: '<uri>' (…)` or
  `set_environment_source: failed to open '<uri>' -- no buildings this run`; otherwise it warns
  `no geo-anchor solved yet at on_activate() -- environment layer disabled this run` (`:776-781`).
  A later GPS solve only logs (`:277-294`) and nothing re-arms, so only a live switch loads a
  source then. `:869-903`: a live switch is refused until the anchor solves, and a failed open
  answers `… -- previous environment source left intact`; `:1262-1281`: the fallback WARNs
  `environment source: network loss detected -- switched to fallback dir '<dir>'` and
  `… but none configured -- environment now empty`; `:697`: the publisher `~/diagnostics`.
- `tools/validate_visual_mode.sh:283-284` activates right after configure, long before a GPS
  solve's 500 fixes. `:252-256` starts the node with no flag that turns off rclcpp's file
  logging, so each node log line (prefixed `[INFO] [<time>] [overlume_node]: `) lands both in
  `/tmp/overlume_validate/overlume_node.log` and under
  `${ROS_LOG_DIR:-${ROS_HOME:-$HOME/.ros}/log}`.
- `ros/src/overlume_ros/src/diagnostics.cpp:54-57`: the `render_ms` status and key.
- `ros/src/overlume_ros/include/overlume_ros/geo_anchor.hpp:30-31`: 500 fixes and a 20 m
  baseline before a GPS solve; `ros/src/overlume_ros/src/geo_anchor.cpp:106-112`: all three
  datum values or none.
- `overlume/src/environment_stream.cpp:398-424`: heading is the compass bearing of map +x (90 is
  east); `overlume/src/environment_stream.hpp:50`: 8 consecutive failures;
  `overlume/src/environment_stream.cpp:764-789`: once fallen back, a source stays so for its life.
- `tools/validate_logger_session.sh:16-29` (`--arm`, and `--` forwards the rest to
  `validate_visual_mode.sh`), `:144-170` (`--arm` retries every 5 s for 120 s, then prints
  `[arm] OK: environment source = <name>`); `tools/validate_visual_mode.sh:69`, `:218`, `:256`
  (`/tmp/overlume_validate/overlume_node.log`), `:72-116` (any prior rig is killed; port 8765).
- `NOTICE:282-300`: the shape of the Cesium OSM and Google blocks (`Terms:` plus a
  "License per upstream" sentence); `docs/runbooks/cesium.md` §1 and §6: the token export and
  attribution conventions this runbook follows.

- [ ] **Step 1: Link the runbook from the four pages that index it** (the failing check first)

In `README.md`, replace

```markdown
  Streaming supports terrain following, a geo-anchor from NavSatFix, automatic fallback to the baked source, and live switching with no restart. It needs `CESIUM_ION_TOKEN` set in your environment. See the [runbook](docs/runbooks/cesium.md).
```

with

```markdown
  Streaming supports terrain following, a geo-anchor from NavSatFix, automatic fallback to the baked source, and live switching with no restart. It needs `CESIUM_ION_TOKEN` set in your environment. See the [runbook](docs/runbooks/cesium.md).
- **Mapbox buildings streaming.** `mapbox://buildings` streams Mapbox's landmark models and extruded building footprints around the vehicle in the theme's clay; `mapbox://buildings?materials=original` keeps Mapbox's own look. Like `ion://`, it falls back to the baked source on network loss and switches live. It draws Mapbox's attribution on screen and needs `MAPBOX_TOKEN` set in your environment. See the [runbook](docs/runbooks/mapbox.md).
```

In `docs/README.md`, replace

```markdown
  contract + the `CESIUM_ION_TOKEN` smoke check; the Google Photorealistic
  3D Tiles preset.
```

with

```markdown
  contract + the `CESIUM_ION_TOKEN` smoke check; the Google Photorealistic
  3D Tiles preset.
- [`mapbox.md`](runbooks/mapbox.md) — the `mapbox://buildings` source: the
  `MAPBOX_TOKEN` contract and `tools/check_mapbox_token.sh`, the URI and the
  two GUI entries, attribution, Mapbox's terms as engineering rules, the demo
  at the test site and Downtown Dubai, and troubleshooting.
```

In `docs/status.md`, replace (the end of open item 20, its blank line and the next heading)

```markdown
or pass it in the documented build command and the gate.

## Known gaps / accepted exceptions
```

with

```markdown
or pass it in the documented build command and the gate.

21. **OPEN 2026-10-14 — Mapbox sign-off before any push.** The `mapbox://buildings` source ([runbook](runbooks/mapbox.md)) lives on the local `mapbox-environment` branch. Nothing from it is pushed to `origin` until the maintainer confirms Mapbox's sign-off, which follows the demo to Mapbox. Its landmark endpoint (`3dtiles/v1`) is undocumented. Before that push: `tools/check_publish_leaks.sh` and the lint checks in `AGENTS.md`.

22. **OPEN 2026-10-14 — Mapbox logo and trademark check before release.** The node draws the Mapbox logo from `ros/src/overlume_ros/assets/attribution/mapbox-logo.png` (provenance in the `ATTRIBUTION.md` beside it). Before a public release, confirm with Mapbox that the logo file may ship in the repository and in packages, and that the on-screen wording and placement meet their attribution terms. If it may not ship, remove it from the tree and let deployments supply it through `environment_attribution_logo_path`.

## Known gaps / accepted exceptions
```

In `docs/runbooks/environment_bake.md`, replace

````markdown
MAPBOX_TOKEN=<your token> python3 bake_environment.py --anchor-lat ... --anchor-lon ... --out DIR
```

## Output contract
````

with

````markdown
MAPBOX_TOKEN=<your token> python3 bake_environment.py --anchor-lat ... --anchor-lon ... --out DIR
```

This bake fallback is not the runtime `mapbox://` environment source, which
streams Mapbox buildings while the node runs and never stores them; see
[`mapbox.md`](mapbox.md).

## Output contract
````

- [ ] **Step 2: Run the link check and see it fail**

```bash
cd "$WORKTREE"
python3 tools/check_docs_links.py; echo "exit=$?"
```

Expected:

```text
Broken links (4):
  README.md: -> docs/runbooks/mapbox.md
  docs/README.md: -> runbooks/mapbox.md
  docs/status.md: -> runbooks/mapbox.md
  docs/runbooks/environment_bake.md: -> mapbox.md
exit=1
```

This is the check that fails if the runbook is later deleted or renamed.

- [ ] **Step 3: Write the runbook**

Create `docs/runbooks/mapbox.md`:

````markdown
# Mapbox buildings runbook

How to run the `mapbox://buildings` environment source: Mapbox's landmark models plus building
footprints from Mapbox Streets v8, extruded to their heights, streamed around the vehicle in
visual mode (mode 3). The design is
[`2026-10-14-mapbox-buildings-design.md`](../design/2026-10-14-mapbox-buildings-design.md).

Until the maintainer confirms Mapbox's sign-off, the source stays on the local
`mapbox-environment` branch and nothing from it reaches `origin` ([`status.md`](../status.md),
open items).

## 1. The token

- The library reads `MAPBOX_TOKEN` from its environment each time a `mapbox://` source opens, and
  trims surrounding whitespace. No parameter, file or URI key carries it.
- Unset or empty: the source does not open, and one log line names the variable. A live switch
  is refused and the previous source stays. When the launch-time URI arms at activate (§7.2), the
  node logs `set_environment_source: failed to open '…' -- no buildings this run`.
- Put it in the deploy user's `~/.bashrc`, then open a new shell:

  ```bash
  export MAPBOX_TOKEN=<paste the token here>
  ```

- `tools/validate_visual_mode.sh` re-reads `MAPBOX_TOKEN` and `CESIUM_ION_TOKEN` from a fresh
  login shell before it starts the node, because a long-lived terminal can hold a stale value. It
  prints a `[token]` line when the two differed, never the value.
- The token never appears in the URI, a parameter, a log line, a model or a file. The library
  sends it only as the `access_token` query parameter of its own requests, its log lines go
  through the redacting logger (`access_token=<redacted>`), and the source writes no cache.
- To rotate it, change the export and restart the rig. A running node keeps the environment it
  started with.
- If the token ever lands in a log, a file or a transcript, rotate it. Redaction is not the
  remedy.

## 2. Preflight: `tools/check_mapbox_token.sh`

Run it before every demo. It is never part of the gate: it needs the network and the real token.

```bash
tools/check_mapbox_token.sh
```

It requests the two endpoints the source uses, at Downtown Dubai: the landmark tile z14
10707/7006 and the footprint tile z16 42830/28025 inside it. Like
`overlume/scripts/check_cesium_token.sh`, it prints one line. A working token prints:

```text
PASS: token retrieves the Downtown Dubai tiles (landmark tile HTTP 200, footprint tile HTTP 200)
```

- Exit 0 on `PASS:`. Exit 1 on `FAIL:`, which names each tile's HTTP code or transport error.
  Exit 2 on `SKIP:`.
- `SKIP: MAPBOX_TOKEN is not set`: no request was sent. Export the token (§1).
- `HTTP 401` on both: Mapbox rejects the token. Check it, or make a new one.
- `HTTP 403`: the token is valid but not allowed here (URL restrictions or scopes). Use a token
  without URL restrictions.
- `landmark tile HTTP 204`: Mapbox no longer serves landmarks there. The landmark endpoint is
  undocumented (§8); the source still shows extrusions.
- `transport error (curl exit N)`: no network. N is curl's exit status: 6 for DNS, 7 for a
  refused connection, 28 for the 30 s timeout.
- The script prints no URL, token or response body. curl reads the URL from stdin, so the token
  is not on its command line either.

## 3. The URI

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
- A library built without cesium (`OVERLUME_ENABLE_CESIUM=OFF`) refuses `mapbox://`, as it refuses
  `ion://`.

The node composes the URI from `environment_source_uri` and its own parameters:

- `fallback=<environment_chunks_dir>` when that directory is set;
- `radius=<environment_tile_radius_m>` when it is not 700, the node default. At 700 the source's
  own 500 m default applies;
- `brightness=<environment_brightness>` when it is not 1.0;
- never `cache=`, and never an ion-only key.

Each z14 tile (about 2.2 km wide in Dubai) costs 17 requests: one landmark model tile and 16
footprint tiles. With the 500 m radius, one to four tiles are live at a time. Footprint tiles are
billed per request, so the 200,000-a-month free tier covers about 12,500 z14 tiles, or about
11,700 if Mapbox also bills the landmark requests. The landmark endpoint has no published price.

## 4. The two GUI entries

The vcam GUI's Environment tiles list (`tools/vcam_gui.py`, through `tools/vcam_ws_bridge.py`)
holds the supported environment sources. Two of them are Mapbox:

| Entry | `environment_source_uri` | Look |
|-------|--------------------------|------|
| `mapbox` | `mapbox://buildings` | the theme's clay, like `osm` |
| `mapbox-original` | `mapbox://buildings?materials=original` | Mapbox's, like `google` |

- Pick one in the GUI, send `{"cmd":"set_environment_source","preset":"mapbox"}` to the bridge,
  or pass `--arm mapbox` to `tools/validate_logger_session.sh`.
- The node refuses the switch until its geo anchor is solved, as for every source.
- The GUI shows an entry as selected only when the node's `environment_source_uri` is exactly
  that entry's string. A hand-set URI such as `mapbox://buildings?radius=800` leaves the list as
  it was.
- Neither entry carries `cache=off`: the source never caches.

## 5. Attribution

Mapbox's Product Terms (1.4) require the Mapbox logo, "© Mapbox", "© OpenStreetMap" and "Improve
this map" wherever Mapbox content is shown. The output is a video frame, so the link texts are
drawn as text.

- **What:** the logo bottom-left at its native size, and "© Mapbox © OpenStreetMap Improve this
  map" right-aligned bottom-right, both 8 px from the frame edges. The text uses the theme's HUD
  text colour and scale. When the frame is too narrow for both on one line, the text moves up one
  line, above the logo: at HUD scale 1.0, below 725 px of output width. Below 541 px at that scale
  (in general, 16 px plus 524.76 px times the HUD scale) the text no longer fits on a line, and
  its end, "Improve this map", is cut off. At 1280 px that happens from HUD scale 2.5.
- **When:** all of these hold:
  - `environment_attribution` is true (the default; the element's disable knob);
  - the environment is visible: free look (mode 3) and `environment_enabled`;
  - `environment_source_uri` starts with `mapbox://`;
  - the source state is `STREAMING`.

  It does not depend on `hud_enabled`. In `STREAMING_FALLBACK` only baked chunks are on screen, so
  nothing is drawn.
- **Logo file:** `environment_attribution_logo_path`, read at configure. Empty, the default,
  means the installed `assets/attribution/mapbox-logo.png`; its provenance is in the
  `ATTRIBUTION.md` beside it. A missing or unreadable logo warns once, and the text is still
  drawn.
- Keep `environment_attribution` on wherever a person sees Mapbox content, at an output width
  and HUD scale that show the whole text. A screenshot or a recording that shows Mapbox content
  keeps the attribution in the frame.

## 6. Mapbox's terms as engineering rules

- **No caching.** The source has no disk cache, accepts only `cache=off`, and writes no file.
- **No Mapbox data in the tree.** Tests build their landmark and footprint fixtures in the test
  process, in Mapbox's formats. Responses fetched while debugging (tiles, TileJSON, response
  bodies) stay outside the repository. No pre-bake of `mapbox://` content exists or may be added,
  and a bake made through the bake script's Mapbox fallback must not be committed either. The one
  Mapbox file in the tree is the logo (§5), which the terms require on screen; the release check
  below covers it.
- **Attribution visible** whenever Mapbox content is (§5).
- **The token by name only** (§1).
- **Local until sign-off.** The `mapbox-environment` branch and its lane branches are not pushed to
  `origin` until the maintainer confirms Mapbox's sign-off. Before that push, run
  `tools/check_publish_leaks.sh` and the lint checks listed in `AGENTS.md`.
- **Logo and trademark.** Whether the logo file may stay in the tree is part of the release check
  ([`status.md`](../status.md), open items).

The bake script's Mapbox fallback ([`environment_bake.md`](environment_bake.md)) is a separate,
older path and is unchanged.

## 7. The demo

Both sites use the replay rig and the vcam GUI's free look. The rig owns the machine: starting it
stops any rig already running on the box (same process names, port 8765).

Before either site:

1. Build the library with cesium on, and the node, in the checkout you run from (`AGENTS.md`,
   "Build and test").
2. Export `MAPBOX_TOKEN` (§1) and run `tools/check_mapbox_token.sh` (§2).
3. Set `SESSION_DIR` to the logger session's directory
   ([`replay_logger_session.md`](replay_logger_session.md)).

### 7.1 Test site

The session's own GPS anchor places the map at the test site (25.1972, 55.2744).

```bash
tools/validate_logger_session.sh "$SESSION_DIR" --arm mapbox
```

- The anchor solves from the replayed GPS after 500 fixes and 20 m of travel. `--arm` retries the
  switch every 5 s for up to 120 s, then prints `[arm] OK: environment source = mapbox`.
- Expect extruded buildings only: Mapbox has no landmark models there.
- `--arm mapbox-original` shows Mapbox's look. The GUI list switches between the two live.

### 7.2 Downtown Dubai

A full geo-datum override moves the map to Downtown:

- `geo_datum_lat_deg`, `geo_datum_lon_deg` and `geo_datum_heading_deg` must all be set. A partial
  override is logged as an ERROR and ignored. The anchor then waits for GPS, nothing arms at
  activate (the node warns `no geo-anchor solved yet at on_activate()`), and no buildings show
  until a live switch (a GUI entry or `--arm`) after the GPS solve, which streams them at the
  test site.
- The override pins where map (0, 0) is, and the compass bearing of map +x (90.0 means map +x
  points east). It does not move the robot: the replayed path keeps its map coordinates. The
  rehearsal keeps the session's map heading and picks the latitude and longitude so that the path
  runs on open ground near the Burj Khalifa.
- The table's values fit the rehearsed session. For another session, read its map heading from
  `--anchor-heading-deg` in the `geo-anchor solved:` line of a §7.1 run, and choose a latitude
  and longitude that move its route's centre onto open ground near the Burj Khalifa (the target
  minus the route centre's east and north offset).
- With all three set, the anchor is solved at configure, so the launch-time
  `environment_source_uri` arms at activate. No `--arm` is needed.
- `geo_datum_height_m` and `geo_anchor_height_offset_m` are optional. They do not move Mapbox
  buildings, which stand on map z = 0 at any anchor height.
- The parameters are doubles: write `90.0`, not `90`.

| Parameter | Value |
|-----------|-------|
| `geo_datum_lat_deg` | filled in by Task 17's rehearsal |
| `geo_datum_lon_deg` | filled in by Task 17's rehearsal |
| `geo_datum_heading_deg` | filled in by Task 17's rehearsal |

Set `DATUM_LAT`, `DATUM_LON` and `DATUM_HEADING` to the table's values, then:

```bash
tools/validate_logger_session.sh "$SESSION_DIR" -- \
    --param environment_source_uri:=mapbox://buildings \
    --param "geo_datum_lat_deg:=${DATUM_LAT}" \
    --param "geo_datum_lon_deg:=${DATUM_LON}" \
    --param "geo_datum_heading_deg:=${DATUM_HEADING}"
```

For Mapbox's look, quote the URI:
`--param 'environment_source_uri:=mapbox://buildings?materials=original'`.

The node log, `/tmp/overlume_validate/overlume_node.log`, shows a line containing
`geo-anchor solved (geo_datum override): …` at configure and a line containing
`environment source armed: 'mapbox://buildings` at activate.

Expect the Burj Khalifa and the other landmarks as models, with no doubled extrusion under them,
among streets of extruded buildings.

### 7.3 What to check

- The logo and the text are visible in mode 3 while the source streams, and gone in fallback.
- `mapbox-original` shows Mapbox's look, and `mapbox` the theme's.
- Steady-state `render_ms` (the `render_ms` status on `/overlume_node/diagnostics`), in clay mode,
  stays within 1.5 times the `osm` source's at the same pose.
- No token value in any log. Run both checks in a shell where §1's export is in effect: with
  `MAPBOX_TOKEN` empty, every line would count as a hit. The token is read from a pipe, never put
  on a command line. The first prints one count per rig log, and every count must be 0. The
  second scans the node's own ROS log files and must print nothing:

  ```bash
  grep -cF -f <(printf '%s\n' "$MAPBOX_TOKEN") /tmp/overlume_validate/*.log
  grep -rcF -f <(printf '%s\n' "$MAPBOX_TOKEN") "${ROS_LOG_DIR:-${ROS_HOME:-$HOME/.ros}/log}" \
      | grep -v ':0$'
  ```

## 8. Troubleshooting

- **The switch is refused with `failed to open '…' -- previous environment source left intact`,
  and the log names `MAPBOX_TOKEN`.** The node's environment has no token. Export it (§1) and
  restart the rig.
- **The log says `Mapbox rejected MAPBOX_TOKEN (HTTP 401)` and no buildings appear.** Mapbox
  rejects the token. Run `tools/check_mapbox_token.sh` and replace the token. The 401s count as
  failed requests, so eight in a row switch the source to its fallback (below).
- **No buildings over the sea.** A footprint tile over open water holds only a `water` layer, so
  there is nothing to extrude. Nothing is wrong.
- **Extrusions, but no landmark models.** Mapbox serves landmark models in Downtown Dubai and
  Dubai Marina. None were found in the Abu Dhabi samples or around the test site: there the
  landmark tile answers 204 and only extrusions show.
- **No landmark models in Downtown, and one log line names a landmark tile.** The landmark tile
  could not be read. Its endpoint (`3dtiles/v1`) is undocumented and can change without notice,
  for example to a new compression codec. Extrusions still show. Run the opt-in live test, which
  prints tile, landmark and extrusion counts, and report what it prints:
  `OVERLUME_LIVE_MAPBOX=1 overlume/build/test_mapbox_buildings --gtest_filter='MapboxLive.*'`.
- **The buildings, the logo and the text disappear, and the node warns
  `environment source: network loss detected -- switched to fallback dir '…'`** (or
  `... but none configured -- environment now empty`). Eight requests in a row failed: network
  loss, or 401 on everything. The state is `STREAMING_FALLBACK`, and the baked chunks from
  `fallback=` (the node's `environment_chunks_dir`) show instead. The fallback lasts for the
  source's life. Fix the cause, then select the source again: pick another GUI entry and then the
  Mapbox one, rerun `--arm`, or run
  `ros2 param set /overlume_node environment_source_uri mapbox://buildings`.
- **A tile answers 429 or 5xx, or a request fails in transport.** The tile is retried, at most 3
  attempts. After that it shows what loaded, and one line is logged.
- **A landmark doubled by an extrusion, or a missing neighbour, at a landmark's edge.**
  Extrusions under a landmark are removed by their footprint's centroid, so a part whose centroid
  lies outside the landmark's footprint stays. Known limit.
- **A wall missing along a straight north-south or east-west line.** Walls along a z16 tile edge
  are left out, so that buildings cut by tile edges meet without doubled walls. A real wall lying
  exactly on such an edge (tile coordinate 0 or 4096) is left out too. Both demo sites' street
  grids are rotated, so this is rare there. Known limit.
- **Buildings float or sink on a slope.** The source has no terrain following: buildings stand on
  the flat map plane. Known limit until imagery on terrain lands (design §13).
- **The GUI list does not show `mapbox` while Mapbox streams.** The node's
  `environment_source_uri` is not exactly an entry's string (a hand-set key, for example). Pick
  the entry to set the exact string.
````

The `<paste the token here>` line is the instruction to the operator, as in
`docs/runbooks/cesium.md` §1; no token value goes into the file.

- [ ] **Step 4: Run the link check and see it pass**

```bash
cd "$WORKTREE"
python3 tools/check_docs_links.py; echo "exit=$?"
```

Expected: `OK: no broken links` and `exit=0`.

- [ ] **Step 5: CHANGELOG and NOTICE**

In `CHANGELOG.md`, replace

```markdown
## [Unreleased]

## [1.0.0] - 2026-10-07
```

with

```markdown
## [Unreleased]

### Added

- `mapbox://buildings`, a streamed environment source selected like `ion://`: Mapbox's landmark models plus building footprints from Mapbox Streets v8, extruded to their heights around the vehicle. URI keys: `materials` (`clay` or `original`), `radius` (default 500 m), `brightness`, `fallback` and `cache=off`. The library reads `MAPBOX_TOKEN` when the source opens; Mapbox tiles are never cached. While the source streams, the node draws the Mapbox logo and "© Mapbox © OpenStreetMap Improve this map" (`environment_attribution`; the logo file is `environment_attribution_logo_path`). The vcam GUI lists two more environment sources, `mapbox` and `mapbox-original`, and `tools/check_mapbox_token.sh` checks the token. See `docs/runbooks/mapbox.md`.

### Fixed

- Network loss now switches a streamed source to its fallback when requests fail in transport. A transport error (DNS failure, refused connection, timeout) rejected the request without reaching the consecutive-failure counter, so a real loss of connectivity never triggered the fallback, for `ion://` as well.
- `tools/validate_visual_mode.sh` reads the refreshed tokens back on a separate file descriptor, so output from the login shell's startup files (Ubuntu's sudo hint, for one) can no longer end up in `CESIUM_ION_TOKEN`. It now refreshes `MAPBOX_TOKEN` the same way.

## [1.0.0] - 2026-10-07
```

In `NOTICE`, replace

```text
wording and cache-lifetime policy as due for a pre-go-live manual recheck.

--------------------------------------------------------------------------------
Committed test fixtures (synthesized, no third-party data)
```

with

```text
wording and cache-lifetime policy as due for a pre-go-live manual recheck.

--------------------------------------------------------------------------------
Mapbox 3D buildings and Mapbox Streets v8 (optional live streaming source)
--------------------------------------------------------------------------------
Terms:    Mapbox Terms of Service and Product Terms; see
          docs/runbooks/mapbox.md for the token, the on-screen attribution
          this deployment must show (the Mapbox logo, "© Mapbox",
          "© OpenStreetMap", "Improve this map") and the no-cache,
          no-stored-data rules.
License per upstream (Mapbox terms); not re-verified beyond that runbook's
citation. The mapbox:// source streams at run time with the deployment's
own MAPBOX_TOKEN (by name only): no Mapbox data is stored, cached or
committed in this repository, and the tests use synthetic fixtures built in
Mapbox's formats. Mapbox Streets v8 carries OpenStreetMap data (ODbL),
credited on screen by "© OpenStreetMap". The Mapbox logo
(ros/src/overlume_ros/assets/attribution/mapbox-logo.png, provenance in the
ATTRIBUTION.md beside it) is a Mapbox trademark, shipped only to meet that
attribution requirement; whether it may stay in this repository is an open
release check (docs/status.md).

--------------------------------------------------------------------------------
Committed test fixtures (synthesized, no third-party data)
```

Every new NOTICE line fits in 80 columns, like its neighbours.

- [ ] **Step 6: Run the doc checks**

```bash
cd "$WORKTREE"
python3 tools/check_docs_links.py; echo "exit=$?"
grep -nE '\b(pk|sk|tk)\.[A-Za-z0-9_-]{20,}' docs/runbooks/mapbox.md README.md docs/README.md \
    docs/status.md CHANGELOG.md NOTICE docs/runbooks/environment_bake.md; echo "grep=$?"
grep -c "filled in by Task 17's rehearsal" docs/runbooks/mapbox.md
grep -c 'Task 17' docs/runbooks/mapbox.md
```

Expected: `OK: no broken links`, `exit=0`; no token-shaped match, `grep=1`; `3` (the three table
rows Task 17 fills); `3` again (those rows are the runbook's only mention of Task 17, so nothing
is left dangling once Task 17 fills them).

- [ ] **Step 7: Format**

```bash
cd "$WORKTREE"
git add docs/runbooks/mapbox.md README.md docs/README.md docs/status.md CHANGELOG.md NOTICE \
    docs/runbooks/environment_bake.md
tools/check_format.sh --fix
git status --short
```

Expected: `check_format.sh: reformatted 196 files with clang-format 23.1.1` (no C/C++ change in
this task, so nothing moves), and exactly the seven staged files:

```text
M  CHANGELOG.md
M  NOTICE
M  README.md
M  docs/README.md
M  docs/runbooks/environment_bake.md
A  docs/runbooks/mapbox.md
M  docs/status.md
```

Any other line is a deviation to report.

- [ ] **Step 8: Gate**

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
ROS_DOMAIN_ID=86 CI_VISUAL_MODE_DOMAIN_ID=86 \
    flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock tools/ci_visual_mode.sh
```

Expected: six `PASS` lines (POD header check, library ctest suite, node gtests, WS bridge pytest
suite `(86 passed in …)`, golden suite, examples), `OVERALL: PASS` and exit 0, with the same
counts as Task 11's gate. Exit 75 means another lane held the gate lock for 300 s: run the same
command again. This task changes no code; the gate proves the lane is still green on top of
Task 11.

- [ ] **Step 9: Commit**

```bash
cd "$WORKTREE"
git add docs/runbooks/mapbox.md README.md docs/README.md docs/status.md CHANGELOG.md NOTICE \
    docs/runbooks/environment_bake.md
git commit -F - <<'EOF'
docs(mapbox): runbook for mapbox://buildings, with index, status, changelog and NOTICE entries

docs/runbooks/mapbox.md is the operator's page for the Mapbox buildings
source: the MAPBOX_TOKEN contract and tools/check_mapbox_token.sh, the URI
keys and the node's composition, the two GUI entries, the on-screen
attribution, Mapbox's terms as engineering rules (no caching, no Mapbox
data in the tree, attribution visible, local until sign-off), the demo at
the test site and at Downtown Dubai (the full geo-datum override; its
values table is filled in by Task 17's rehearsal), and troubleshooting.

The README's Environment sources and the docs index link it, so the link
check fails if it goes missing. docs/status.md opens two items: Mapbox
sign-off before any push, and the logo and trademark check before release.
CHANGELOG [Unreleased] gains the Added entry and two Fixed entries
(transport errors now count toward the fallback; the rig's token refresh).
NOTICE gets a Mapbox block beside the Cesium OSM and Google ones, and
environment_bake.md separates the bake script's Tilequery fallback from
the runtime source.

Claude-Session: <the executing session's URL>
EOF
```

---

### Task 13: Mapbox loader: tiling, extrusions, failures, dispatch

**Lane:** 7. **Depends on:** Tasks 1–4, 6 and 7 merged into `mapbox-environment` (the lane forks
from that head). Task 5 is not needed: Task 13 requests no landmark tile.

**Files:**
- Create: `overlume/tests/test_mapbox_buildings.cpp` (suite `MapboxBuildings`, 16 tests)
- Modify: `overlume/src/mapbox_loader.cpp` (Task 2's include block, 4-7; the loader is appended
  after `parse_mapbox_spec`, which ends the file at ~73)
- Modify: `overlume/src/mapbox_test_hooks.hpp` (the Task 13 block goes after Task 4's
  `mapbox_tile_point_to_map_probe`, before the closing `}`: ~19-23 as Task 4 left the file, ~38-42
  once Task 5's block is in)
- Modify: `overlume/src/environment_stream.hpp` (`MapFixtureAssetAccessor` between
  `FileFixtureAssetAccessor`, which ends ~193 once Tasks 6 and 7 are in, and
  `StreamRendererResources`)
- Modify: `overlume/src/environment_stream.cpp` (includes, 6-8; after
  `FileFixtureAssetAccessor::makeRequest`, ~525; before `prepareInLoadThread`, ~625; its GLB
  write, ~645; before `captured_cesium_log_text`, near the end; all line numbers after Tasks 6 and
  7)
- Modify: `overlume/src/environment.hpp` (after `open_streaming_environment_source`, 80-83)
- Modify: `overlume/src/environment.cpp` (the scheme prefixes and the `ion://` branch, 179-197)

**Interfaces:**
- Consumes:

```cpp
// overlume/src/mapbox_mvt.hpp (Task 1)
MvtBuildingLayer decode_mvt_building_layer(const uint8_t* data, size_t size);
bool passes_standard_3d_filter(const MvtBuilding& b);
// overlume/src/mapbox_loader.hpp (Task 2)
MapboxSpecResult parse_mapbox_spec(const std::string& spec);
// overlume/src/mapbox_extrude.hpp (Task 3)
ExtrudeStats extrude_polygons(const std::vector<TilePolygon>& polygons, double clip_extent,
                              double min_height_m, double height_m, const ProjectFn& project,
                              TriangleMesh& out);
// overlume/src/mapbox_tile_frame.hpp (Task 4)
LonLatDeg tile_point_to_lonlat(const TileId& t, double u, double v, double extent);
TileId tile_containing(double lon_deg, double lat_deg, uint32_t z);
LonLatRect tile_bounds(const TileId& t);
std::array<TileId, 16> footprint_tiles_of(const TileId& z14);
TileFrame make_tile_frame(const TileId& z14, double origin_height_m);
std::array<double, 3> project_to_frame(const TileFrame& f, const TileId& src, double u, double v,
                                       double extent, double z_m);
// overlume/src/environment_stream.hpp (Tasks 6 and 7, and the existing classes)
struct CustomTileset;  // loader, root, area_selection
StreamingEnvironmentSource(Cesium3DTilesSelection::TilesetExternals externals, int64_t asset_id,
                           std::string ion_access_token, std::string root_tileset_uri,
                           std::string fallback_baked_dir, GeoAnchor anchor,
                           std::shared_ptr<CountingAssetAccessor> counting_accessor,
                           bool materials_original, bool follow_terrain, double ground_bias_m,
                           bool replaces_ground, double max_tilt_deg, double brightness,
                           std::optional<double> radius_override_m, CustomTileset custom_tileset);
std::shared_ptr<spdlog::logger> make_redacting_logger();
Cesium3DTilesSelection::TilesetExternals build_externals(
    std::shared_ptr<CesiumAsync::IAssetAccessor> base, const CesiumAsync::AsyncSystem& asyncSystem,
    const std::string& cache_dir, uint64_t max_cache_items,
    std::shared_ptr<CountingAssetAccessor>* out_counting = nullptr);
class SimpleTaskProcessor;  // explicit SimpleTaskProcessor(int threadCount = 2)
inline constexpr uint64_t kDefaultMaxCacheItems = 4096;
// overlume/src/environment_test_hooks.hpp (existing)
uint64_t environment_loaded_chunk_count(overlume::VisualRenderer* r);
bool environment_stream_first_primitive_is_clay(overlume::VisualRenderer* r);
std::string captured_cesium_log_text();
// overlume/tests/mapbox_mvt_fixture.hpp (Task 1): mvt::Layer, mvt::Feature, mvt::building,
// mvt::encode_tile, mvt::gzip_stored
```

- Produces: the contract's Task 13 declarations, verbatim: `open_mapbox_environment_source` in
  `environment.hpp`, `MapFixtureAssetAccessor` in `environment_stream.hpp` and the Task 13 block
  of `mapbox_test_hooks.hpp`. Behaviour: `set_environment_source(r, "mapbox://…", anchor)` opens a
  streamed buildings source (false without cesium, as `ion://`).

**How the loader works** (spec section 5; read with the code in Step 6):
- **Quadtree.** The root and its one child `QuadtreeTileID(0, 0, 0)` span the Mercator world, are
  empty and refine unconditionally. `createTileChildren` gives levels 0–12 four empty,
  unconditionally refined children, and level 13 four z14 children with unknown content,
  geometric error 0, the absolute east-north-up `frame_to_ecef` as their transform and a
  `BoundingRegion` from 10 m below to 1000 m above the anchor's ground; z14 tiles get no children.
  IDs carry Mapbox's y (north-down): children are (2x + i, 2y + j). Cesium creates the children of
  each visited tile during the same traversal, so the first selection reaches z14.
- **Loading.** `loadTileContent` answers z < 14 with an empty Success. For a z14 tile it counts the
  attempt (main thread, `attempts_`), requests the 16 MVTs through the tileset's accessor (so
  counting and redaction apply), maps each answer to a small `Fetched` and a rejection to a
  transport failure (so `AsyncSystem::all` never rejects), and builds on a worker. The
  continuation captures the config, the state and a per-attempt `settled` flag by `shared_ptr`,
  never the loader. (Cesium also keeps the content manager, which owns the loader, alive while a
  load is in flight, so the captures are defence in depth.)
- **Failures.** 200 is a body; 204 and 404 are empty and silent; 401/403 log "Mapbox rejected
  MAPBOX_TOKEN (HTTP 401) for streets-v8 footprints" once per endpoint and status, with no URL;
  any 429, 5xx or transport failure on attempts 1 and 2 returns `RetryLater`, and on attempt 3
  the tile builds from what loaded and logs once. Other statuses log once and leave their square
  empty. A body that is gzip but does not inflate, or no vector tile, skips its square with one
  line per kind. Transport errors are never logged with their text: it carries the URL.
- **The retry wait.** Cesium loads a `RetryLater` tile again on its very next selection, so
  without a wait all three attempts would run within about three frames. An attempt is one round
  of requests, counted in `loadTileContent` (main thread) when its requests go out. Each attempt
  sets the tile's not-before time: attempt 2 starts no sooner than 2 s after attempt 1 started,
  attempt 3 no sooner than 4 s after attempt 2 started. Until then `loadTileContent` answers
  `RetryLater` at once, with no request, no attempt counted and no `retries` count. A 429 or 503
  burst shorter than about 6 s therefore cannot use up the three attempts, and nothing is
  requested (or billed) while a tile waits. The wait reads `clock_s()`, the steady clock, which
  `mapbox_set_test_clock` can freeze, so the retry tests move time instead of sleeping. Skipped:
  carrying an attempt's 200 answers into the next one; a tile costs at most three rounds of
  requests, so add it if billing shows retries matter.
- **Building.** Standard's filter, a numeric `height` above `min_height`, the square's own layer
  extent as the clip extent and `project_to_frame` with the z16 square as the source tile; one
  buffer, one TRIANGLES primitive with per-face normals, one material (Mapbox Standard's linear
  building colour, metallic 0, roughness 1; clay replaces it at upload), one identity node in
  scene 0. No geometry gives an empty Success. `updatedBoundingVolume` carries the real top
  (nothing a test can observe: the area view and the radius gate decide the same either way). The
  result never carries the request.
- **Opening.** One function serves `open_mapbox_environment_source` (curl, `MAPBOX_TOKEN`) and
  `install_fixture_mapbox_source` (the fixture network, a given token): parse (refusal or warning
  through the redacting logger), trim the token (empty: one line naming `MAPBOX_TOKEN`), the
  cache-off externals, then `StreamingEnvironmentSource` with follow-terrain and ground
  replacement off, the URI's radius and an area-selected `CustomTileset`. The `AsyncSystem` runs
  on two worker threads, `SimpleTaskProcessor()`'s default, exactly as the ion source's does
  (`open_streaming_environment_source`, `environment_stream.cpp:1145` before Tasks 6 and 7).
  Curl blocks a worker for each request in flight, so a tile's 16 (17 with landmarks) requests
  run two at a time, as ion's tile requests do.

**API notes** (cesium-native 0.64.0, paths under `overlume/build/_deps/cesium-native-src/`; the
fact-check notes and these sources agree on every point used):
- `Cesium3DTilesSelection/include/Cesium3DTilesSelection/TilesetContentLoader.h:37-114`
  `TileLoadInput` (references: `tile` :75, `asyncSystem` :86, `pAssetAccessor` :92, `pLogger` :97,
  `ellipsoid` :113), `:120-125` `TileChildrenResult`, `:157` `loadTileContent`, `:174-177`
  `createTileChildren`.
- `TileLoadResult.h:76-163` (fields; `state` has no default, so `success()` sets every field it
  needs by name), `:183-185` `createRetryLaterResult`; `src/TilesetContentLoader.cpp:60-74`.
- `src/TilesetContentManager.cpp`: `:1226` the tile is `ContentLoading` before `:1257` calls the
  loader synchronously from `Tileset::loadTiles` on the main thread; `:1312-1321` a rejected load
  fails the tile for good; `:2005-2010` `RetryLater` makes it `FailedTemporarily`, without a log
  line; `:1169-1170` with `src/Tile.cpp:276-280`: a `FailedTemporarily` tile is loaded again on
  the very next selection, with no delay (hence the loader's own wait); `:1291-1293` a result
  other than a Success model goes straight to the main-thread continuation, with no worker hop;
  `:1255` and `:1295`: the manager keeps itself (and so its loader) alive until a load's
  main-thread continuation runs; `:1342-1363` `createTileChildren`: Success creates the children,
  an empty Success means none, ever; `:533-535` a `pCompletedRequest`'s URL goes into the
  model's extras; `:1401-1413` empty content can be unloaded and is then loaded again;
  `:957-962` the manager's destructor resolves the destruction event. `src/Tileset.cpp:126-136`:
  the `Tileset` destructor never waits for loads in flight (the held-request test relies on it).
- `src/TilesetSelection.cpp:1054` updates (and so creates the children of) every visited tile
  before culling it; `:611-625` `renderLeaf` queues the leaf's load.
- `include/Cesium3DTilesSelection/Tile.h:152-154` (unknown content), `:197-200` (empty content),
  `:279` `createChildTiles`, `:302` `setBoundingVolume`, `:351` `setGeometricError`, `:391`
  `setUnconditionallyRefine`, `:434` `setTransform` (absolute), `:445` `getTileID`;
  `TileID.h:30-34`; `BoundingVolume.h:30-37`. Passing the `QuadtreeTileID` to the constructor
  keeps cesium's reference count right (`src/Tile.cpp:59-111`).
- `CesiumGeometry/include/CesiumGeometry/QuadtreeTileID.h:29-30`, `:90-100`;
  `CesiumGeospatial/include/CesiumGeospatial/BoundingRegion.h:30-34`, `GlobeRectangle.h:101-110`
  (`fromDegrees`), `Ellipsoid.h:48` (`WGS84`).
- `CesiumAsync/include/CesiumAsync/AsyncSystem.h:66` `createFuture` (calls `f` at once), `:94`
  `createPromise`, `:222` `all(std::vector<Future<T>>&&)` (input order; rejects if any input does),
  `:322` `createResolvedFuture`; `Future.h:70` `thenInWorkerThread`, `:120` `thenImmediately`,
  `:202` `catchImmediately`; `Impl/CatchFunction.h:25-33` (the callback gets `std::exception&&`);
  `Promise.h:26` `resolve`, `:40` `reject`, `:62` `getFuture`. A promise resolved on the test
  thread runs the `thenImmediately` continuations there.
- `CesiumAsync/include/CesiumAsync/IAssetAccessor.h:39-42` `get`; `IAssetRequest.h:42`
  `response()`; `IAssetResponse.h:27` `statusCode()`, `:42` `data()`.
- `CesiumUtility/include/CesiumUtility/Gzip.h:14` `isGzip`, `:40` `gunzip`; `src/Gzip.cpp:68-104`:
  a truncated stream ends in `Z_BUF_ERROR` and `gunzip` returns false.
- `CesiumCurl/include/CesiumCurl/CurlAssetAccessor.h:98`; `src/CurlAssetAccessor.cpp:349-354`
  (construction is `curl_global_init` only, so it is built only for an accepted URI);
  `:366-394` `get` runs a blocking `curl_easy_perform` inside `runInWorkerThread`.
- CesiumGltf (`CesiumGltf/generated/include/CesiumGltf/`): `AccessorSpec.h:44, 53, 65, 71, 93-169`;
  `BufferView.h:27, 30, 36-64`; `BufferSpec.h:33`;
  `CesiumGltf/include/CesiumGltf/BufferCesium.h:17`; `MeshPrimitive.h:41, 55-77`;
  `Material.h:52`; `MaterialPBRMetallicRoughness.h:31, 52, 60`; `Node.h:57`; `Scene.h:24`;
  `ModelSpec.h:61, 123, 128`; `Asset.h:36`.
  `CesiumGltfWriter/generated/src/ModelJsonWriter.cpp:2684-2702`: `baseColorFactor` and
  `metallicFactor` are written when they differ from their defaults.
- glm (cesium's vcpkg prefix): `glm/gtc/type_ptr.hpp:220` `make_mat4` reads column-major, as
  `Mat4d` stores. spdlog 1.17 (same prefix): `spdlog/logger.h:144` `warn`, `:149` `error`.
- Overlume (line numbers before Tasks 6 and 7): `environment_stream.cpp:436-469` the fixture
  response and request classes; `:297-325` the redacting sink keeps the redacted text, appended,
  up to 64 KiB per process; `:601-631` `prepareInLoadThread` (one buffer kept as is, heights
  corrected, `writeGlb`, normals only where missing); `:1041-1075` teardown pumps at most 2000
  times; `environment.cpp:184-205` the dispatch; `renderer.cpp:1114-1116` the environment updates
  only for a valid ego; `test_environment_stream.cpp:49-78` the `ScopedUnsetEnv` and Windows shim
  copied into the new test.
- Fixtures: `overlume/tests/fixtures/environment_test_town_0` is the baked fallback (chunks within
  300 m of map (0, 0) load, `environment.cpp:34-38`). The test site's z14 tile is 10707/7006;
  its anchor point (25.1972, 55.2744) lies 208 m from the tile's east edge, so the tests anchor at
  the tile's centre (every edge about 1108 m away) or at its north-west corner.

- [ ] **Step 1: Create the lane worktree** (the orchestrator, once Tasks 1–4, 6 and 7 are merged)

```bash
git -C "$INTEGRATION" worktree add "$INTEGRATION-l7" -b mapbox-env/l7 mapbox-environment
cd "$INTEGRATION-l7"
mkdir -p /tmp/overlume-l7
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock \
    cmake --toolchain "$PWD/overlume/cmake/toolchain-clang-libcxx.cmake" -S overlume \
    -B overlume/build -DOVERLUME_ENABLE_CESIUM=ON -DCMAKE_C_COMPILER_LAUNCHER=ccache \
    -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DOVERLUME_TEST_TMP_DIR_RUNTIME=/tmp/overlume-l7
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j16
ros/colcon_build.sh
```

Expected: the configure ends `-- Build files have been written to: …/overlume/build`, the build
and `colcon build` finish without errors. The configure holds the gate lock because every
configure re-runs cesium's ezvcpkg step on the shared prefix; exit 75 means the wait ran out: run
it again. From here on `$WORKTREE` is `$INTEGRATION-l7`.

- [ ] **Step 2: Declare Task 13's interfaces**

In `overlume/src/mapbox_test_hooks.hpp`, replace

```cpp
                                    double u, double v, double extent, double z_m,
                                    double out_map_xyz[3]);

}
```

with

```cpp
                                    double u, double v, double extent, double z_m,
                                    double out_map_xyz[3]);

// Task 13.
struct MapboxFixtureNetwork;  // wraps a MapFixtureAssetAccessor; never freed
MapboxFixtureNetwork* make_mapbox_fixture_network(uint16_t unmatched_status = 404);
void mapbox_fixture_serve(MapboxFixtureNetwork* n, const std::string& url_without_query,
                          uint16_t status, std::vector<uint8_t> body);
void mapbox_fixture_fail_transport(MapboxFixtureNetwork* n, const std::string& url_without_query);
// Requests for a held URL stay pending until mapbox_fixture_release_held answers them as served.
void mapbox_fixture_hold(MapboxFixtureNetwork* n, const std::string& url_without_query);
void mapbox_fixture_release_held(MapboxFixtureNetwork* n);
std::vector<std::string> mapbox_fixture_requested_urls(MapboxFixtureNetwork* n);
std::string mapbox_landmark_url(uint32_t x14, uint32_t y14);   // no query string
std::string mapbox_footprint_url(uint32_t x16, uint32_t y16);  // no query string
// Opens a mapbox:// URI exactly as set_environment_source does (parse, source, set_visible,
// teardown of the old source, swap), but over `n` and with `token` in place of MAPBOX_TOKEN.
bool install_fixture_mapbox_source(overlume::VisualRenderer* r, const char* mapbox_uri,
                                   overlume::GeoAnchor anchor, MapboxFixtureNetwork* n,
                                   const char* token);
// Counting rules: a tile counts once, when it reaches Success with content. A feature counts once
// per z16 square it is decoded from (a building clipped into two squares counts twice), and in
// exactly one of extruded_features (it produced geometry) or conflated_features (dropped under a
// landmark footprint); filtered-out features (extrude=false, underground, no height) count in
// neither.
struct MapboxSourceStats {
    uint64_t tiles_built = 0;         // z14 tiles that reached Success with content
    uint64_t landmark_nodes = 0;      // building nodes kept, summed over built tiles
    uint64_t extruded_features = 0;   // features extruded, summed
    uint64_t conflated_features = 0;  // features dropped under a landmark footprint (Task 14)
    uint64_t retries = 0;             // attempts that ended in RetryLater (waits not counted)
    double max_build_ms = 0.0;        // slowest tile build, worker time
    uint64_t wall_quads = 0;          // wall quads of the extruded features, summed
};
// The counters of the most recently opened Mapbox source in this process.
MapboxSourceStats mapbox_last_source_stats();
// Test-only: while on, every GLB the stream writes in prepareInLoadThread is kept; turning it on
// drops what was kept before.
void capture_stream_glbs(bool on);
std::vector<std::vector<uint8_t>> captured_stream_glbs();
// Test-only: freezes the clock the loader's retry wait reads at `seconds`; a negative value gives
// the steady clock back.
void mapbox_set_test_clock(double seconds);

}
```

In `overlume/src/environment.hpp`, replace

```cpp
std::unique_ptr<EnvironmentSource> open_streaming_environment_source(const std::string& ion_spec,
                                                                     GeoAnchor anchor);

}
```

with

```cpp
std::unique_ptr<EnvironmentSource> open_streaming_environment_source(const std::string& ion_spec,
                                                                     GeoAnchor anchor);

// The text after "mapbox://". Reads MAPBOX_TOKEN; nullptr when the URI is refused or the token
// is unset or empty (one log line on the redacting logger either way).
std::unique_ptr<EnvironmentSource> open_mapbox_environment_source(const std::string& mapbox_spec,
                                                                  GeoAnchor anchor);

}
```

In `overlume/src/environment_stream.hpp`, replace

```cpp
    std::shared_ptr<std::atomic<bool>> killed_;
    std::shared_ptr<std::atomic<bool>> transportDown_;
};

class StreamRendererResources final : public Cesium3DTilesSelection::IPrepareRendererResources {
```

with

```cpp
    std::shared_ptr<std::atomic<bool>> killed_;
    std::shared_ptr<std::atomic<bool>> transportDown_;
};

// Test accessor beside FileFixtureAssetAccessor, reusing its response classes: canned answers keyed
// by URL without its query string, or a rejected future (a transport error); every requested URL
// is recorded in full. Thread-safe.
class MapFixtureAssetAccessor final : public CesiumAsync::IAssetAccessor {
public:
    explicit MapFixtureAssetAccessor(uint16_t unmatched_status = 404)
        : unmatchedStatus_(unmatched_status) {}

    void serve(const std::string& url_without_query, uint16_t status, std::vector<std::byte> body);
    void fail_transport(const std::string& url_without_query);
    std::vector<std::string> requested_urls() const;
    // A held URL's requests stay pending until release_held() answers them, so a test can switch
    // sources while a tile load is in flight.
    void hold(const std::string& url_without_query);
    void release_held();

    CesiumAsync::Future<std::shared_ptr<CesiumAsync::IAssetRequest>> get(
        const CesiumAsync::AsyncSystem& asyncSystem, const std::string& url,
        const std::vector<THeader>&) override {
        return respond(asyncSystem, "GET", url);
    }
    CesiumAsync::Future<std::shared_ptr<CesiumAsync::IAssetRequest>> request(
        const CesiumAsync::AsyncSystem& asyncSystem, const std::string& verb,
        const std::string& url, const std::vector<THeader>&,
        const std::span<const std::byte>&) override {
        return respond(asyncSystem, verb, url);
    }
    void tick() noexcept override {}

private:
    using RequestPromise = CesiumAsync::Promise<std::shared_ptr<CesiumAsync::IAssetRequest>>;
    struct Canned {
        uint16_t status = 0;
        std::vector<std::byte> body;
        bool transport_error = false;
    };
    struct Held {
        RequestPromise promise;
        std::string verb;
        std::string url;
    };

    CesiumAsync::Future<std::shared_ptr<CesiumAsync::IAssetRequest>> respond(
        const CesiumAsync::AsyncSystem& asyncSystem, const std::string& verb,
        const std::string& url);
    void answer(const RequestPromise& promise, const std::string& verb,
                const std::string& url) const;

    uint16_t unmatchedStatus_;
    mutable std::mutex mutex_;
    std::unordered_map<std::string, Canned> canned_;
    std::vector<std::string> held_;
    std::vector<Held> pending_;
    std::vector<std::string> requested_;
};

class StreamRendererResources final : public Cesium3DTilesSelection::IPrepareRendererResources {
```

- [ ] **Step 3: Write the failing test** `overlume/tests/test_mapbox_buildings.cpp`

Every test but the 401 one builds its fixture network with `make_mapbox_fixture_network(204)`: a
URL the test does not serve answers 204 (empty), as Mapbox does for a z14 tile without landmarks.
204 is a 2xx, so unserved URLs never count as network failures. MVT bodies come from Task 1's
encoder; even squares are gzip (as Mapbox sends them), odd ones raw (as curl hands them over after
inflating). No frame is ever rendered with a source opened through `MAPBOX_TOKEN`: such a source
uses curl.

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume/api.h"
#include "overlume/scene.h"

#include "environment_test_hooks.hpp"
#include "mapbox_mvt_fixture.hpp"
#include "mapbox_test_hooks.hpp"
#include "mapbox_tile_frame.hpp"
#include "test_paths.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace {

namespace mvt = overlume::testing::mvt;
using overlume::EnvironmentSourceState;
using overlume::mapbox::TileId;
using overlume::testing::MapboxFixtureNetwork;

constexpr const char* kToken = "pk.test-token-not-real";
// anchor_square_body: one box per z16 square; square 0's five extra features build nothing.
constexpr uint64_t kAnchorTileExtruded = 16;
// The loader's retry schedule on a frozen clock: attempt 1 at 0 s, attempt 2 no sooner than 2 s
// after it, attempt 3 no sooner than 4 s after attempt 2.
constexpr double kAttempt2AtS = 2.0;
constexpr double kAttempt3AtS = 6.0;
constexpr size_t kFrameBytes = 320u * 240u * 3u;
const overlume::CameraPose kPose{{0, -8, 3}, {0, 0, 0.5}, 60};
const std::string kTestTownDir =
    std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/environment_test_town_0";

#ifdef _WIN32
// MSVC's CRT has no setenv/unsetenv; _putenv_s with "" removes the variable.
int setenv(const char* name, const char* value, int) { return _putenv_s(name, value); }
int unsetenv(const char* name) { return _putenv_s(name, ""); }
#endif

class ScopedUnsetEnv {
public:
    explicit ScopedUnsetEnv(const char* name) : name_(name) {
        const char* v = std::getenv(name);
        if (v) {
            had_value_ = true;
            saved_ = v;
        }
        ::unsetenv(name);
    }
    ~ScopedUnsetEnv() {
        if (had_value_)
            ::setenv(name_, saved_.c_str(), 1);
        else
            ::unsetenv(name_);
    }
    ScopedUnsetEnv(const ScopedUnsetEnv&) = delete;
    ScopedUnsetEnv& operator=(const ScopedUnsetEnv&) = delete;

private:
    const char* name_;
    bool had_value_ = false;
    std::string saved_;
};

// Tile math runs on use, not during static initialisation: it may lean on statics of other
// translation units.
TileId op_tile() { return overlume::mapbox::tile_containing(55.2744, 25.1972, 14); }

overlume::GeoAnchor anchor_at(const TileId& t, double u, double v) {
    const overlume::mapbox::LonLatDeg p = overlume::mapbox::tile_point_to_lonlat(t, u, v, 1.0);
    return overlume::GeoAnchor{p.lat, p.lon, 0.0, 0.0};
}

std::string url_of(const TileId& square) {
    return overlume::testing::mapbox_footprint_url(square.x, square.y);
}

// An exterior ring: positive surveyor's area in tile units (y down).
std::vector<std::array<int32_t, 2>> box(int32_t x0, int32_t y0, int32_t x1, int32_t y1) {
    return {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
}

// One z16 square's body: gzip as Mapbox sends it, or raw as curl hands it over after inflating.
std::vector<uint8_t> building_tile(std::vector<mvt::Feature> features, bool gzip = true) {
    mvt::Layer layer;
    layer.name = "building";
    layer.features = std::move(features);
    const std::vector<uint8_t> raw = mvt::encode_tile({layer});
    return gzip ? mvt::gzip_stored(raw) : raw;
}

void serve_square(MapboxFixtureNetwork* n, const TileId& square,
                  std::vector<mvt::Feature> features) {
    overlume::testing::mapbox_fixture_serve(n, url_of(square), 200,
                                            building_tile(std::move(features)));
}

// Square i of a tile: one 54 m box, 10 + i m tall. Square 0 also carries an extrude=false outline,
// an underground part, a part with height <= min_height and a part with no height, which the
// filter must skip, and a part wholly in the tile buffer, which passes the filter but clips to
// nothing and so must not count. Even squares are gzip, odd ones raw: the loader must read both.
std::vector<uint8_t> anchor_square_body(size_t i) {
    std::vector<mvt::Feature> features{
        mvt::building(100 + i, {box(1000, 1000, 1400, 1400)}, 10.0 + static_cast<double>(i))};
    if (i == 0) {
        features.push_back(mvt::building(1, {box(2000, 2000, 2400, 2400)}, 30.0, 0.0, false));
        features.push_back(mvt::building(2, {box(2600, 2000, 3000, 2400)}, 30.0, 0.0, true, true));
        features.push_back(mvt::building(3, {box(2000, 2600, 2400, 3000)}, 5.0, 5.0));
        mvt::Feature unmeasured = mvt::building(4, {box(2600, 2600, 3000, 3000)}, 9.0);
        unmeasured.tags.erase(unmeasured.tags.begin());  // its `height` tag
        features.push_back(std::move(unmeasured));
        features.push_back(mvt::building(5, {box(-30, 100, -2, 200)}, 30.0));
    }
    return building_tile(std::move(features), i % 2 == 0);
}

void serve_anchor_tile(MapboxFixtureNetwork* n, const TileId& t) {
    const std::array<TileId, 16> squares = overlume::mapbox::footprint_tiles_of(t);
    for (size_t i = 0; i < squares.size(); ++i) {
        overlume::testing::mapbox_fixture_serve(n, url_of(squares[i]), 200, anchor_square_body(i));
    }
}

// Every requested URL, without its query string.
std::set<std::string> requested(MapboxFixtureNetwork* n) {
    std::set<std::string> out;
    for (const std::string& url : overlume::testing::mapbox_fixture_requested_urls(n)) {
        out.insert(url.substr(0, url.find('?')));
    }
    return out;
}

std::set<std::string> requested_mvt(MapboxFixtureNetwork* n) {
    std::set<std::string> out;
    for (const std::string& url : requested(n)) {
        if (url.find("mapbox-streets-v8") != std::string::npos) out.insert(url);
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

std::set<std::string> squares_of(const std::vector<TileId>& tiles) {
    std::set<std::string> out;
    for (const TileId& t : tiles) {
        for (const TileId& square : overlume::mapbox::footprint_tiles_of(t)) {
            out.insert(url_of(square));
        }
    }
    return out;
}

overlume::VisualRenderer* make_renderer() {
    return overlume::create_renderer(overlume::RenderConfig{320, 240, 1, kThemeDir, "dark_adas"});
}

bool install(overlume::VisualRenderer* r, const std::string& uri, const overlume::GeoAnchor& anchor,
             MapboxFixtureNetwork* n, const char* token = kToken) {
    return overlume::testing::install_fixture_mapbox_source(r, uri.c_str(), anchor, n, token);
}

void set_ego(overlume::VisualRenderer* r, double x, double y) {
    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = overlume::Vec3{x, y, 0.0};
    overlume::set_scene(r, s);
}

uint64_t loaded(overlume::VisualRenderer* r) {
    return overlume::testing::environment_loaded_chunk_count(r);
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

// Renders `count` frames, letting worker-thread work land between them.
void frames(overlume::VisualRenderer* r, std::vector<uint8_t>& buf, int count) {
    pump(r, buf, count, [] { return false; });
}

// For work that finishes on a worker thread with no frame to drive it.
template <typename Done>
bool wait_until(Done done) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (!done() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return done();
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

// The first baseColorFactor in a GLB's JSON chunk, read as numbers so the writer's float format
// does not matter.
std::optional<std::array<double, 4>> base_color_factor(const std::vector<uint8_t>& glb) {
    const std::string text(glb.begin(), glb.end());
    const std::string key = "\"baseColorFactor\":[";
    const size_t at = text.find(key);
    if (at == std::string::npos) return std::nullopt;
    std::array<double, 4> rgba{};
    const char* p = text.c_str() + at + key.size();
    for (double& c : rgba) {
        char* end = nullptr;
        c = std::strtod(p, &end);
        if (end == p) return std::nullopt;
        p = end + 1;  // past ',' or ']'
    }
    return rgba;
}

bool ends_with(const std::string& s, const std::string& suffix) {
    return s.size() >= suffix.size() &&
           s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

// Freezes the loader's retry clock at 0 s for one test, which then moves time with set() instead
// of sleeping through a wait. The steady clock comes back when the test ends, however it ends.
class FrozenRetryClock {
public:
    FrozenRetryClock() { set(0.0); }
    ~FrozenRetryClock() { overlume::testing::mapbox_set_test_clock(-1.0); }
    FrozenRetryClock(const FrozenRetryClock&) = delete;
    FrozenRetryClock& operator=(const FrozenRetryClock&) = delete;
    void set(double seconds) { overlume::testing::mapbox_set_test_clock(seconds); }
};

// Square 0 answers 503 (or fails in transport) until the tile has been sent back once, then 200.
void expect_retry_then_load(bool transport) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    FrozenRetryClock clock;
    const TileId t = op_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    serve_anchor_tile(n, t);
    const std::string square0 = url_of(overlume::mapbox::footprint_tiles_of(t)[0]);
    if (transport) {
        overlume::testing::mapbox_fixture_fail_transport(n, square0);
    } else {
        overlume::testing::mapbox_fixture_serve(n, square0, 503, {});
    }
    ASSERT_TRUE(install(r, "mapbox://buildings", anchor_at(t, 0.5, 0.5), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 100, [] { return stats().retries >= 1; }))
        << "a transient answer must send the tile back for another attempt";
    overlume::testing::mapbox_fixture_serve(n, square0, 200, anchor_square_body(0));
    clock.set(kAttempt2AtS - 0.1);
    frames(r, buf, 20);
    EXPECT_EQ(times_requested(n, square0), 1u) << "no request while the retry waits";
    EXPECT_EQ(loaded(r), 0u);
    EXPECT_EQ(stats().retries, 1u) << "waiting is not another attempt";
    clock.set(kAttempt2AtS);
    ASSERT_TRUE(pump(r, buf, 400, [&] { return loaded(r) >= 1; }));
    EXPECT_EQ(times_requested(n, square0), 2u);
    EXPECT_EQ(stats().extruded_features, kAnchorTileExtruded)
        << "the retry must build square 0 from its 200";
    EXPECT_EQ(stats().retries, 1u) << "the second attempt found every square answered";
    EXPECT_EQ(state(r), EnvironmentSourceState::STREAMING);
    overlume::destroy_renderer(r);
}

}

TEST(MapboxBuildings, UnsetTokenRefusesAndNamesTheVariable) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ScopedUnsetEnv noToken("MAPBOX_TOKEN");
    const overlume::GeoAnchor anchor = anchor_at(op_tile(), 0.5, 0.5);
    const size_t logFrom = log_size();
    EXPECT_FALSE(overlume::set_environment_source(r, "mapbox://buildings", anchor));
    EXPECT_EQ(state(r), EnvironmentSourceState::NONE);
    EXPECT_NE(log_since(logFrom).find("MAPBOX_TOKEN"), std::string::npos)
        << "the refusal must name the variable to set";

    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    EXPECT_FALSE(install(r, "mapbox://buildings", anchor, n, " \t\n"))
        << "a token of only whitespace is no token";
    set_ego(r, 0.0, 0.0);  // a source opened by mistake would now request its tile
    std::vector<uint8_t> buf(kFrameBytes);
    frames(r, buf, 3);
    EXPECT_TRUE(overlume::testing::mapbox_fixture_requested_urls(n).empty());
    overlume::destroy_renderer(r);
}

TEST(MapboxBuildings, RefusedUrisOpenNothing) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    ScopedUnsetEnv noToken("MAPBOX_TOKEN");  // a URI accepted by mistake still reaches no network
    const overlume::GeoAnchor anchor = anchor_at(op_tile(), 0.5, 0.5);
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    for (const char* uri : {"mapbox://roads", "mapbox://buildings?cache=tiles",
                            "mapbox://buildings?radius=-5", "mapbox://buildings?brightness=0"}) {
        const size_t logFrom = log_size();
        EXPECT_FALSE(install(r, uri, anchor, n)) << uri;
        EXPECT_NE(log_since(logFrom).find("refused"), std::string::npos) << uri;
        EXPECT_FALSE(overlume::set_environment_source(r, uri, anchor)) << uri;
    }
    set_ego(r, 0.0, 0.0);  // a source opened by mistake would now request its tile
    std::vector<uint8_t> buf(kFrameBytes);
    frames(r, buf, 3);
    EXPECT_EQ(state(r), EnvironmentSourceState::NONE);
    EXPECT_TRUE(overlume::testing::mapbox_fixture_requested_urls(n).empty());
    overlume::destroy_renderer(r);
}

TEST(MapboxBuildings, AnchorTileLoadsWithStandardFilterApplied) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const TileId t = op_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    serve_anchor_tile(n, t);
    ASSERT_TRUE(install(r, "mapbox://buildings", anchor_at(t, 0.5, 0.5), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 400, [&] { return loaded(r) >= 1; }));
    EXPECT_EQ(state(r), EnvironmentSourceState::STREAMING);
    EXPECT_EQ(loaded(r), 1u);
    const overlume::testing::MapboxSourceStats s = stats();
    EXPECT_EQ(s.tiles_built, 1u);
    EXPECT_EQ(s.extruded_features, kAnchorTileExtruded)
        << "16 boxes extrude; the extrude=false outline, the underground part, the part with "
           "height <= min_height and the part with no height must not, and the part in the tile "
           "buffer builds nothing, so it must not count";
    EXPECT_EQ(s.retries, 0u);
    EXPECT_GT(s.max_build_ms, 0.0);
    overlume::destroy_renderer(r);
}

TEST(MapboxBuildings, RequestsOnlyTheTileInsideTheRadius) {
    // The -meshopt landmark variant, vertex codec v0: GL JS's -meshopt-v2-lod tiles use codec v1,
    // which the linked meshoptimizer cannot decode. Checked before the GPU skip, so it always runs.
    EXPECT_EQ(overlume::testing::mapbox_landmark_url(1, 2),
              "https://api.mapbox.com/3dtiles/v1/mapbox.mapbox-3dbuildings-v1-meshopt/14/1/2.glb");
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const TileId t = op_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    serve_anchor_tile(n, t);
    ASSERT_TRUE(install(r, "mapbox://buildings", anchor_at(t, 0.5, 0.5), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 400, [&] { return loaded(r) >= 1; }));
    const std::set<std::string> expected = squares_of({t});
    EXPECT_EQ(requested(n), expected)
        << "mid-tile with the 500 m default radius, only the ego's z14 tile may be requested";
    EXPECT_EQ(overlume::testing::mapbox_fixture_requested_urls(n).size(), expected.size())
        << "each URL exactly once";
    overlume::destroy_renderer(r);
}

TEST(MapboxBuildings, RadiusKeyWidensTheRequestedTiles) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const TileId t = op_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);  // neighbours answer empty
    serve_anchor_tile(n, t);
    ASSERT_TRUE(install(r, "mapbox://buildings?radius=1300", anchor_at(t, 0.5, 0.5), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 400, [&] { return loaded(r) >= 1; }));
    for (int i = 0; i < 5; ++i) overlume::render_frame(r, kPose, {buf.data(), 320, 240});
    const TileId north{14, t.x, t.y - 1}, south{14, t.x, t.y + 1};
    const TileId west{14, t.x - 1, t.y}, east{14, t.x + 1, t.y};
    EXPECT_EQ(requested_mvt(n), squares_of({t, north, south, west, east}))
        << "1300 m from the tile centre reaches the four edge neighbours (about 1108 m away) but "
           "no corner neighbour (about 1566 m)";
    EXPECT_EQ(stats().tiles_built, 1u) << "neighbours answering 204 with empty MVTs add nothing";
    EXPECT_EQ(loaded(r), 1u);
    overlume::destroy_renderer(r);
}

TEST(MapboxBuildings, TokenRidesOnlyInRequestUrls) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const TileId t = op_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    serve_anchor_tile(n, t);
    overlume::testing::capture_stream_glbs(true);
    const size_t logFrom = log_size();
    ASSERT_TRUE(install(r, "mapbox://buildings", anchor_at(t, 0.5, 0.5), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 400, [&] { return loaded(r) >= 1; }));
    const std::vector<std::vector<uint8_t>> glbs = overlume::testing::captured_stream_glbs();
    overlume::testing::capture_stream_glbs(false);

    const std::vector<std::string> urls = overlume::testing::mapbox_fixture_requested_urls(n);
    ASSERT_FALSE(urls.empty());
    for (const std::string& url : urls) {
        EXPECT_TRUE(ends_with(url, std::string("?access_token=") + kToken))
            << url.substr(0, url.find('?'));
    }
    ASSERT_FALSE(glbs.empty());
    for (const std::vector<uint8_t>& glb : glbs) {
        EXPECT_FALSE(contains(glb, kToken)) << "a request URL reached the model";
        EXPECT_FALSE(contains(glb, "access_token"));
    }
    EXPECT_EQ(log_since(logFrom).find(kToken), std::string::npos);
    overlume::destroy_renderer(r);
}

TEST(MapboxBuildings, TokenWhitespaceIsTrimmed) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const TileId t = op_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    serve_anchor_tile(n, t);
    const size_t logFrom = log_size();
    ASSERT_TRUE(
        install(r, "mapbox://buildings", anchor_at(t, 0.5, 0.5), n, " pk.test-token-not-real\n"));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 400, [&] { return loaded(r) >= 1; }));
    const std::vector<std::string> urls = overlume::testing::mapbox_fixture_requested_urls(n);
    ASSERT_FALSE(urls.empty());
    for (const std::string& url : urls) {
        EXPECT_TRUE(ends_with(url, "?access_token=pk.test-token-not-real"))
            << url.substr(0, url.find('?'));
        EXPECT_EQ(url.find_first_of(" \t\r\n"), std::string::npos);
    }
    EXPECT_EQ(log_since(logFrom).find(kToken), std::string::npos);
    overlume::destroy_renderer(r);
}

TEST(MapboxBuildings, RejectedTokenFallsBackAndLogsOnce) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const TileId t = op_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(401);
    const size_t logFrom = log_size();
    ASSERT_TRUE(
        install(r, "mapbox://buildings?fallback=" + kTestTownDir, anchor_at(t, 0.5, 0.5), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    EXPECT_TRUE(pump(r, buf, 30, [&] {
        return state(r) == EnvironmentSourceState::STREAMING_FALLBACK;
    })) << "a run of rejected requests counts as network loss";
    EXPECT_TRUE(pump(r, buf, 30, [&] { return loaded(r) > 0; })) << "the baked fallback shows";
    ASSERT_TRUE(wait_until([] { return stats().max_build_ms > 0.0; }))
        << "the rejected tile's build never finished";
    const std::string log = log_since(logFrom);
    EXPECT_EQ(count_of(log, "Mapbox rejected MAPBOX_TOKEN (HTTP 401) for streets-v8 footprints"),
              1u)
        << "16 rejected squares, one line";
    EXPECT_EQ(log.find("api.mapbox.com"), std::string::npos) << "log lines name tiles, not URLs";
    overlume::destroy_renderer(r);
}

TEST(MapboxBuildings, ServerErrorRetriesThenLoads) { expect_retry_then_load(false); }

TEST(MapboxBuildings, TransportErrorRetriesThenLoads) { expect_retry_then_load(true); }

TEST(MapboxBuildings, PersistentServerErrorGivesUpAfterThreeAttempts) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    FrozenRetryClock clock;
    const TileId t = op_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    serve_anchor_tile(n, t);
    const std::string square0 = url_of(overlume::mapbox::footprint_tiles_of(t)[0]);
    overlume::testing::mapbox_fixture_serve(n, square0, 503, {});
    const size_t logFrom = log_size();
    ASSERT_TRUE(install(r, "mapbox://buildings", anchor_at(t, 0.5, 0.5), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 100, [] { return stats().retries >= 1; }));
    clock.set(kAttempt2AtS);
    ASSERT_TRUE(pump(r, buf, 100, [] { return stats().retries >= 2; }))
        << "attempt 2 starts once the first wait is over";
    clock.set(kAttempt3AtS - 0.1);
    frames(r, buf, 20);
    EXPECT_EQ(times_requested(n, square0), 2u) << "the second wait is twice the first";
    clock.set(kAttempt3AtS);
    ASSERT_TRUE(pump(r, buf, 400, [&] { return loaded(r) >= 1; }))
        << "after three attempts the tile shows what loaded";
    const overlume::testing::MapboxSourceStats s = stats();
    EXPECT_EQ(s.retries, 2u) << "attempts 1 and 2 retry; attempt 3 builds";
    EXPECT_EQ(times_requested(n, square0), 3u);
    EXPECT_EQ(s.extruded_features, kAnchorTileExtruded - 1) << "square 0 never answered";
    EXPECT_EQ(count_of(log_since(logFrom),
                       "streets-v8 footprints still failing (HTTP 503) after 3 attempts"),
              1u);
    overlume::destroy_renderer(r);
}

TEST(MapboxBuildings, TransportLossEverywhereFallsBack) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const TileId t = op_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    for (const TileId& square : overlume::mapbox::footprint_tiles_of(t)) {
        overlume::testing::mapbox_fixture_fail_transport(n, url_of(square));
    }
    overlume::testing::mapbox_fixture_fail_transport(
        n, overlume::testing::mapbox_landmark_url(t.x, t.y));
    ASSERT_TRUE(
        install(r, "mapbox://buildings?fallback=" + kTestTownDir, anchor_at(t, 0.5, 0.5), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    EXPECT_TRUE(pump(r, buf, 30, [&] {
        return state(r) == EnvironmentSourceState::STREAMING_FALLBACK;
    })) << "requests that fail in transport (no HTTP answer at all) must count as network loss";
    EXPECT_TRUE(pump(r, buf, 30, [&] { return loaded(r) > 0; }));
    overlume::destroy_renderer(r);
}

TEST(MapboxBuildings, GarbageBehindOkSkipsOnlyThoseSquares) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const TileId t = op_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    serve_anchor_tile(n, t);
    const std::array<TileId, 16> squares = overlume::mapbox::footprint_tiles_of(t);
    const std::string html = "<!DOCTYPE html><html><body>Service Unavailable</body></html>";
    const std::vector<uint8_t> htmlBody(html.begin(), html.end());
    std::vector<uint8_t> truncated = anchor_square_body(2);  // an even square: gzip
    truncated.resize(truncated.size() / 2);
    overlume::testing::mapbox_fixture_serve(n, url_of(squares[0]), 200, htmlBody);
    overlume::testing::mapbox_fixture_serve(n, url_of(squares[1]), 404, {});
    overlume::testing::mapbox_fixture_serve(n, url_of(squares[2]), 200, truncated);
    overlume::testing::mapbox_fixture_serve(n, url_of(squares[3]), 400, {});
    overlume::testing::mapbox_fixture_serve(n, url_of(squares[4]), 200, htmlBody);
    const size_t logFrom = log_size();
    ASSERT_TRUE(install(r, "mapbox://buildings", anchor_at(t, 0.5, 0.5), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 400, [&] { return loaded(r) >= 1; }))
        << "garbage in some squares must not fail the tile";
    EXPECT_EQ(state(r), EnvironmentSourceState::STREAMING);
    const overlume::testing::MapboxSourceStats s = stats();
    EXPECT_EQ(s.extruded_features, kAnchorTileExtruded - 5) << "squares 0-4 add nothing";
    EXPECT_EQ(s.retries, 0u) << "garbage and 4xx are not transient";
    const std::string log = log_since(logFrom);
    EXPECT_EQ(count_of(log, "is not valid gzip"), 1u);
    EXPECT_EQ(count_of(log, "is not a vector tile"), 1u) << "two HTML squares, one line";
    EXPECT_EQ(count_of(log, "answered HTTP 400"), 1u);
    EXPECT_EQ(count_of(log, "HTTP 404"), 0u) << "404 is an empty square, not an error";
    overlume::destroy_renderer(r);
}

TEST(MapboxBuildings, CornerLoadsFourTilesWithoutDoubledSeamWalls) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    // The ego stands on the corner where A (north-west), B (north), C (west) and D meet.
    const TileId d = op_tile();
    const TileId a{14, d.x - 1, d.y - 1}, b{14, d.x, d.y - 1}, c{14, d.x - 1, d.y};
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    using overlume::mapbox::footprint_tiles_of;
    serve_square(n, footprint_tiles_of(a)[15],
                 {mvt::building(1, {box(3600, 3600, 3900, 3900)}, 20.0)});
    serve_square(n, footprint_tiles_of(b)[12],
                 {mvt::building(2, {box(200, 3600, 500, 3900)}, 20.0)});
    // One 80 m building across the C|D seam, cut as Mapbox cuts it: each copy runs 33 units (the
    // tile buffer) into the neighbouring square.
    serve_square(n, footprint_tiles_of(c)[3],
                 {mvt::building(3, {box(3800, 400, 4129, 700)}, 20.0)});
    serve_square(n, footprint_tiles_of(d)[0], {mvt::building(3, {box(-33, 400, 300, 700)}, 20.0)});
    ASSERT_TRUE(install(r, "mapbox://buildings", anchor_at(d, 0.0, 0.0), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 400, [&] { return loaded(r) == 4; }))
        << "500 m around a z14 corner reaches all four tiles; loaded " << loaded(r);
    EXPECT_EQ(requested_mvt(n), squares_of({a, b, c, d}));
    const overlume::testing::MapboxSourceStats s = stats();
    EXPECT_EQ(s.extruded_features, 4u);
    EXPECT_EQ(s.wall_quads, 4u + 4u + 6u)
        << "A and B have 4 walls each. The seam building's halves have 3 each: its 4 walls, with "
           "the north and south ones cut once at the seam, and none along the seam (doubled seam "
           "walls make 8)";

    set_ego(r, -600.0, -600.0);  // heading 0: map +x is north, +y west, so 600 m south and east
    EXPECT_TRUE(pump(r, buf, 100, [&] { return loaded(r) == 1; }))
        << "600 m into D, the other three tiles must leave the scene; loaded " << loaded(r);
    EXPECT_EQ(requested_mvt(n), squares_of({a, b, c, d})) << "nothing new is requested";
    overlume::destroy_renderer(r);
}

// A smoke test; sanitizers are not in the gate. Cesium keeps the content manager, which owns the
// loader, alive until a load's main-thread continuation runs (TilesetContentManager.cpp:1255,
// :1295). Here that continuation is never dispatched, so the loader outlives the stale build and
// the loader's shared_ptr captures are defence in depth. The test pins that a switch with a tile
// load in flight neither blocks nor crashes, that the new source works, and that the stale build
// completes on the old source's worker.
TEST(MapboxBuildings, SwitchingAwayMidBuildIsSafe) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const TileId t = op_tile();
    const overlume::GeoAnchor anchor = anchor_at(t, 0.5, 0.5);
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    serve_anchor_tile(n, t);
    const std::string held = url_of(overlume::mapbox::footprint_tiles_of(t)[0]);
    overlume::testing::mapbox_fixture_hold(n, held);
    ASSERT_TRUE(install(r, "mapbox://buildings", anchor, n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 30, [&] { return requested(n).count(held) == 1; }));
    ASSERT_EQ(stats().tiles_built, 0u) << "the tile must still wait on the held square";

    // Tear the Mapbox source down while its tile load is in flight, then let the load finish.
    ASSERT_TRUE(overlume::set_environment_source(r, kTestTownDir.c_str(), anchor));
    overlume::testing::mapbox_fixture_release_held(n);
    EXPECT_TRUE(wait_until([] { return stats().tiles_built == 1; }))
        << "the stale build must run to completion on the old source's worker";
    EXPECT_TRUE(pump(r, buf, 60, [&] { return loaded(r) > 0; }));
    EXPECT_EQ(state(r), EnvironmentSourceState::BAKED);
    overlume::destroy_renderer(r);
}

TEST(MapboxBuildings, ClayReplacesAndOriginalKeepsTheExtrusionMaterial) {
    for (const bool original : {false, true}) {
        auto* r = make_renderer();
        if (!r) GTEST_SKIP() << "no GPU/EGL";
        const TileId t = op_tile();
        auto* n = overlume::testing::make_mapbox_fixture_network(204);
        serve_anchor_tile(n, t);
        overlume::testing::capture_stream_glbs(true);
        ASSERT_TRUE(
            install(r, original ? "mapbox://buildings?materials=original" : "mapbox://buildings",
                    anchor_at(t, 0.5, 0.5), n));
        set_ego(r, 0.0, 0.0);
        std::vector<uint8_t> buf(kFrameBytes);
        ASSERT_TRUE(pump(r, buf, 400, [&] { return loaded(r) >= 1; }));
        const std::vector<std::vector<uint8_t>> glbs = overlume::testing::captured_stream_glbs();
        overlume::testing::capture_stream_glbs(false);
        EXPECT_EQ(overlume::testing::environment_stream_first_primitive_is_clay(r), !original)
            << (original ? "materials=original must keep the model's own material"
                         : "clay must replace it with the theme's building material");
        ASSERT_EQ(glbs.size(), 1u);
        const std::optional<std::array<double, 4>> color = base_color_factor(glbs[0]);
        ASSERT_TRUE(color.has_value()) << "the extrusion material has a base colour";
        EXPECT_NEAR((*color)[0], 0.912, 1e-9) << "Mapbox Standard's building colour, linear";
        EXPECT_NEAR((*color)[1], 0.869, 1e-9);
        EXPECT_NEAR((*color)[2], 0.787, 1e-9);
        EXPECT_EQ((*color)[3], 1.0);
        overlume::destroy_renderer(r);
    }
}
```

What each test catches when its behaviour is reverted:
- `UnsetTokenRefusesAndNamesTheVariable`: dropping the `mapbox://` branch in
  `set_environment_source` (the baked fall-through refuses silently: no line names
  `MAPBOX_TOKEN`); dropping the token check, or checking emptiness before trimming (the whitespace
  token opens a source, which requests its tile once the ego is valid; Step 10 runs that revert).
- `RefusedUrisOpenNothing`: opening a source for a URI the parser refuses (with a valid ego and
  three frames, such a source requests its tile); a refusal that logs nothing.
- `AnchorTileLoadsWithStandardFilterApplied`: skipping the gunzip (the 8 gzip squares fail) or
  requiring it (the 8 raw ones fail); dropping Standard's filter (18 features, not 16: the
  extrude=false outline and the underground part extrude); counting a feature that built nothing
  (17: the part in the tile buffer); no content (nothing reaches the scene).
- `RequestsOnlyTheTileInsideTheRadius`: `area_selection = false` in the `CustomTileset` (the
  render camera's frustum and preloaded siblings request other tiles); asking for a square twice;
  the landmark URL switched to GL JS's `-meshopt-v2-lod` variant, whose codec-v1 vertex streams
  would leave every live tile without landmarks (this check runs without a GPU too).
- `RadiusKeyWidensTheRequestedTiles`: not passing the URI's `radius` to the source (the theme's
  100 m reaches no neighbour); a tile whose 16 squares answered 204 returning a model or counting
  as built (spec section 8: neighbours answering 204 add nothing).
- `TokenRidesOnlyInRequestUrls`: a URL without the token; a result that carries its request
  (cesium writes its URL, token included, into the model's extras and so into the GLB); a log line
  holding the token outside a URL (the redacting logger rewrites only `access_token=…`).
- `TokenWhitespaceIsTrimmed` (Review Focus 1): dropping the trim (Step 10).
- `RejectedTokenFallsBackAndLogsOnce`: a line per square instead of one per endpoint and status; a
  line carrying the URL; not passing `fallback=` to the source (nothing shows after the fallback).
- `ServerErrorRetriesThenLoads`: treating 5xx as final (no retry, square 0 missing); no wait
  before a retry, or a wait shorter than 2 s (square 0 is asked again at 1.9 s on the frozen clock;
  Step 10 runs that revert); counting a wait as a retry.
- `TransportErrorRetriesThenLoads`: dropping `catchImmediately` from `fetch` (the rejection reaches
  cesium, which fails the tile for good: it never loads); no wait before a retry (Step 10).
- `PersistentServerErrorGivesUpAfterThreeAttempts`: no cap (the tile never loads), another cap, or
  giving up without the line; no wait, or a second wait shorter than 4 s (square 0 is asked a third
  time at 5.9 s; Step 10).
- `TransportLossEverywhereFallsBack`: Task 6's counting fix reverted (transport errors never count,
  so the stream never falls back).
- `GarbageBehindOkSkipsOnlyThoseSquares` (Review Focus 3): failing or retrying the whole tile for
  one bad body; a line per square; logging a 404.
- `CornerLoadsFourTilesWithoutDoubledSeamWalls` (Review Focus 2): a clip extent other than the
  layer's (the seam building gets 4 + 3 or 4 + 4 walls instead of 3 + 3: 15 or 16, not 14; Step
  10 runs that revert); no area selection (other tiles requested); far tiles left in the scene.
  The seam building's six walls are exactly the unsplit outline's: four walls, the north and south
  ones cut once at the seam, none along it.
- `SwitchingAwayMidBuildIsSafe`: a smoke test; sanitizers are not in the gate. It fails if a switch
  with a load in flight blocks or crashes, or if the stale build never completes on the old
  source's worker. It cannot catch a continuation that captures the loader's `this`: cesium keeps
  the content manager, which owns the loader, alive until the load's main-thread continuation runs
  (`TilesetContentManager.cpp:1255`, `:1295`), and that continuation is never dispatched here, so
  the loader outlives the stale build. The `shared_ptr` captures are defence in depth.
- `ClayReplacesAndOriginalKeepsTheExtrusionMaterial`: not passing `materials=original` to the
  source (clay both times); a wrong extrusion colour.

- [ ] **Step 4: Run it and see it fail**

The new test file needs a configure to become a target. The configure holds the gate lock, as in
Step 1, and the build runs only after it: a build that found the new file itself would
reconfigure outside the lock.

```bash
cd "$WORKTREE"
flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock cmake -S overlume -B overlume/build > /dev/null \
    && echo reconfigured \
    && CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
        --target test_mapbox_buildings
```

Expected: `reconfigured`, then the library builds (nothing calls the new declarations yet) and the
test's link fails with lines such as

```
undefined reference to `overlume::testing::make_mapbox_fixture_network(unsigned short)'
undefined reference to `overlume::testing::install_fixture_mapbox_source(overlume::VisualRenderer*, char const*, overlume::GeoAnchor, overlume::testing::MapboxFixtureNetwork*, char const*)'
undefined reference to `overlume::testing::mapbox_last_source_stats()'
undefined reference to `overlume::testing::capture_stream_glbs(bool)'
clang++: error: linker command failed with exit code 1 (use -v to see invocation)
```

and the build exits non-zero. A compile error instead means Step 2 or 3 was mistyped. No
`reconfigured` line and no CMake error above it means the lock wait ran out (exit 75) and nothing
was built: run the step again.

- [ ] **Step 5: The fixture accessor and the GLB capture** (`overlume/src/environment_stream.cpp`)

Replace

```cpp
#include "environment_test_hooks.hpp"
#include "gltf_normals.hpp"
#include "renderer_internal.hpp"
```

with

```cpp
#include "environment_test_hooks.hpp"
#include "gltf_normals.hpp"
#include "mapbox_test_hooks.hpp"
#include "renderer_internal.hpp"
```

Replace

```cpp
    return std::make_shared<FixtureAssetRequest>(
        verb, url, std::make_unique<FixtureAssetResponse>(200, std::move(bytes)));
}

struct TerrainFollowState {
```

with

```cpp
    return std::make_shared<FixtureAssetRequest>(
        verb, url, std::make_unique<FixtureAssetResponse>(200, std::move(bytes)));
}

void MapFixtureAssetAccessor::serve(const std::string& url_without_query, uint16_t status,
                                    std::vector<std::byte> body) {
    std::lock_guard<std::mutex> lock(mutex_);
    canned_[url_without_query] = Canned{status, std::move(body), false};
}

void MapFixtureAssetAccessor::fail_transport(const std::string& url_without_query) {
    std::lock_guard<std::mutex> lock(mutex_);
    canned_[url_without_query] = Canned{0, {}, true};
}

std::vector<std::string> MapFixtureAssetAccessor::requested_urls() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return requested_;
}

void MapFixtureAssetAccessor::hold(const std::string& url_without_query) {
    std::lock_guard<std::mutex> lock(mutex_);
    held_.push_back(url_without_query);
}

void MapFixtureAssetAccessor::release_held() {
    std::vector<Held> pending;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        held_.clear();
        pending.swap(pending_);
    }
    // Outside the lock: continuations run inline on this thread when a promise resolves.
    for (const Held& h : pending) answer(h.promise, h.verb, h.url);
}

CesiumAsync::Future<std::shared_ptr<CesiumAsync::IAssetRequest>> MapFixtureAssetAccessor::respond(
    const CesiumAsync::AsyncSystem& asyncSystem, const std::string& verb, const std::string& url) {
    const std::string key = url.substr(0, url.find('?'));
    {
        std::lock_guard<std::mutex> lock(mutex_);
        requested_.push_back(url);
        if (std::find(held_.begin(), held_.end(), key) != held_.end()) {
            pending_.push_back(
                Held{asyncSystem.createPromise<std::shared_ptr<CesiumAsync::IAssetRequest>>(), verb,
                     url});
            return pending_.back().promise.getFuture();
        }
    }
    return asyncSystem.createFuture<std::shared_ptr<CesiumAsync::IAssetRequest>>(
        [&](const RequestPromise& promise) { answer(promise, verb, url); });
}

void MapFixtureAssetAccessor::answer(const RequestPromise& promise, const std::string& verb,
                                     const std::string& url) const {
    Canned canned{unmatchedStatus_, {}, false};
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = canned_.find(url.substr(0, url.find('?')));
        if (it != canned_.end()) canned = it->second;
    }
    if (canned.transport_error) {
        // Like curl's, the message carries the URL: only a redacting logger may print it.
        promise.reject(std::runtime_error("fixture transport error for " + url));
        return;
    }
    std::shared_ptr<CesiumAsync::IAssetRequest> request = std::make_shared<FixtureAssetRequest>(
        verb, url, std::make_unique<FixtureAssetResponse>(canned.status, std::move(canned.body)));
    promise.resolve(std::move(request));
}

struct TerrainFollowState {
```

Replace

```cpp
    utils::EntityManager::get().destroy(terrainRoot_);
    terrainRoot_ = {};
}

CesiumAsync::Future<Cesium3DTilesSelection::TileLoadResultAndRenderResources>
StreamRendererResources::prepareInLoadThread(
```

with

```cpp
    utils::EntityManager::get().destroy(terrainRoot_);
    terrainRoot_ = {};
}

namespace {

// Test-only (capture_stream_glbs): the GLBs prepareInLoadThread writes while the flag is on.
std::atomic<bool> g_captureStreamGlbs{false};
std::mutex g_capturedGlbsMutex;
std::vector<std::vector<uint8_t>> g_capturedGlbs;

}

CesiumAsync::Future<Cesium3DTilesSelection::TileLoadResultAndRenderResources>
StreamRendererResources::prepareInLoadThread(
```

Replace

```cpp
                bytes = ensure_flat_normals(std::move(bytes));
                pGlb->glbBytes.assign(
```

with

```cpp
                bytes = ensure_flat_normals(std::move(bytes));
                if (g_captureStreamGlbs.load()) {
                    std::lock_guard<std::mutex> lock(g_capturedGlbsMutex);
                    g_capturedGlbs.push_back(bytes);
                }
                pGlb->glbBytes.assign(
```

Replace

```cpp
std::string captured_cesium_log_text() {
```

with

```cpp
void capture_stream_glbs(bool on) {
    std::lock_guard<std::mutex> lock(g_capturedGlbsMutex);
    if (on) g_capturedGlbs.clear();
    g_captureStreamGlbs.store(on);
}

std::vector<std::vector<uint8_t>> captured_stream_glbs() {
    std::lock_guard<std::mutex> lock(g_capturedGlbsMutex);
    return g_capturedGlbs;
}

std::string captured_cesium_log_text() {
```

- [ ] **Step 6: The loader** (`overlume/src/mapbox_loader.cpp`)

Replace Task 2's include block

```cpp
#include "mapbox_loader.hpp"

#include <cmath>
#include <exception>
```

with

```cpp
#include "mapbox_loader.hpp"

#include "environment_stream.hpp"
#include "mapbox_extrude.hpp"
#include "mapbox_mvt.hpp"
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

and replace the file's end, after `parse_mapbox_spec`,

```cpp
        if (amp == std::string::npos) break;
        pos = amp + 1;
    }
    return {out, "", warning};
}

}
```

with

```cpp
        if (amp == std::string::npos) break;
        pos = amp + 1;
    }
    return {out, "", warning};
}

}

namespace overlume::mapbox {

namespace {

using Cesium3DTilesSelection::Tile;
using Cesium3DTilesSelection::TileChildrenResult;
using Cesium3DTilesSelection::TileEmptyContent;
using Cesium3DTilesSelection::TileLoadInput;
using Cesium3DTilesSelection::TileLoadResult;
using Cesium3DTilesSelection::TileLoadResultState;

constexpr uint32_t kMaxAttempts = 3;
// The wait before attempt 2, from attempt 1's start; doubled before attempt 3.
constexpr double kRetryWaitS = 2.0;
// Tile bounds span the anchor height -10 m to +1000 m until a built tile reports its real top.
constexpr double kRegionBelowM = 10.0;
constexpr double kRegionAboveM = 1000.0;
constexpr char kStreets[] = "streets-v8 footprints";

std::string footprint_url(uint32_t x16, uint32_t y16) {
    return "https://api.mapbox.com/v4/mapbox.mapbox-streets-v8/16/" + std::to_string(x16) + "/" +
           std::to_string(y16) + ".vector.pbf";
}

std::string landmark_url(uint32_t x14, uint32_t y14) {
    return "https://api.mapbox.com/3dtiles/v1/mapbox.mapbox-3dbuildings-v1-meshopt/14/" +
           std::to_string(x14) + "/" + std::to_string(y14) + ".glb";
}

// A shell export can leave spaces or a newline around the token; a request URL must not carry them.
std::string trim_ascii_whitespace(const std::string& s) {
    constexpr char kWhitespace[] = " \t\n\v\f\r";
    const size_t first = s.find_first_not_of(kWhitespace);
    if (first == std::string::npos) return {};
    return s.substr(first, s.find_last_not_of(kWhitespace) - first + 1);
}

// Fixed when the source opens. Async work holds this and LoaderState by shared_ptr, never the
// loader itself, so switching the source away mid-build stays safe.
struct LoaderConfig {
    std::string token;  // goes into request URLs and nowhere else
    bool materials_original = false;
    double origin_height_m = 0.0;
};

struct LoaderState {
    std::atomic<uint64_t> tiles_built{0};
    std::atomic<uint64_t> landmark_nodes{0};
    std::atomic<uint64_t> extruded_features{0};
    std::atomic<uint64_t> conflated_features{0};
    std::atomic<uint64_t> retries{0};
    std::atomic<uint64_t> wall_quads{0};
    std::atomic<double> max_build_ms{0.0};
    std::mutex logged_mutex;
    std::set<std::string> logged;

    // At most one log line per endpoint and status, or failure kind, per source.
    bool first_time(const std::string& key) {
        std::lock_guard<std::mutex> lock(logged_mutex);
        return logged.insert(key).second;
    }

    void note_build_ms(double ms) {
        double seen = max_build_ms.load();
        while (ms > seen && !max_build_ms.compare_exchange_weak(seen, ms)) {
        }
    }
};

std::mutex g_lastStateMutex;
std::shared_ptr<LoaderState> g_lastState;  // the latest source's, for mapbox_last_source_stats()

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

std::vector<TilePolygon> to_tile_polygons(const std::vector<MvtPolygon>& polygons) {
    const auto ring = [](const std::vector<MvtPoint>& points) {
        TileRing out;
        out.reserve(points.size());
        for (const MvtPoint& p : points) {
            out.push_back({static_cast<double>(p.x), static_cast<double>(p.y)});
        }
        return out;
    };
    std::vector<TilePolygon> out;
    out.reserve(polygons.size());
    for (const MvtPolygon& polygon : polygons) {
        TilePolygon& t = out.emplace_back();
        t.outer = ring(polygon.outer);
        for (const std::vector<MvtPoint>& hole : polygon.holes) t.holes.push_back(ring(hole));
    }
    return out;
}

// Appends the extrusions as one mesh (positions, normals and uint32 indices in a new buffer) with
// Mapbox Standard's building colour, under an identity node in the model's scene.
void append_extrusions(CesiumGltf::Model& model, const TriangleMesh& mesh) {
    using CesiumGltf::Accessor;
    using CesiumGltf::BufferView;
    const size_t vertexBytes = mesh.positions.size() * sizeof(float);
    const size_t indexBytes = mesh.indices.size() * sizeof(uint32_t);
    CesiumGltf::Buffer& buffer = model.buffers.emplace_back();
    buffer.cesium.data.resize(2 * vertexBytes + indexBytes);
    std::memcpy(buffer.cesium.data.data(), mesh.positions.data(), vertexBytes);
    std::memcpy(buffer.cesium.data.data() + vertexBytes, mesh.normals.data(), vertexBytes);
    std::memcpy(buffer.cesium.data.data() + 2 * vertexBytes, mesh.indices.data(), indexBytes);
    buffer.byteLength = static_cast<int64_t>(buffer.cesium.data.size());
    const int32_t bufferIndex = static_cast<int32_t>(model.buffers.size() - 1);

    const auto add_accessor = [&](size_t offset, size_t length, int32_t target,
                                  int32_t componentType, const std::string& type, size_t count) {
        BufferView& view = model.bufferViews.emplace_back();
        view.buffer = bufferIndex;
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
    const size_t vertexCount = mesh.positions.size() / 3;
    const int32_t position =
        add_accessor(0, vertexBytes, BufferView::Target::ARRAY_BUFFER,
                     Accessor::ComponentType::FLOAT, Accessor::Type::VEC3, vertexCount);
    const int32_t normal =
        add_accessor(vertexBytes, vertexBytes, BufferView::Target::ARRAY_BUFFER,
                     Accessor::ComponentType::FLOAT, Accessor::Type::VEC3, vertexCount);
    const int32_t indices = add_accessor(
        2 * vertexBytes, indexBytes, BufferView::Target::ELEMENT_ARRAY_BUFFER,
        Accessor::ComponentType::UNSIGNED_INT, Accessor::Type::SCALAR, mesh.indices.size());

    // glTF requires POSITION bounds.
    std::vector<double> lo(3, std::numeric_limits<double>::max());
    std::vector<double> hi(3, std::numeric_limits<double>::lowest());
    for (size_t i = 0; i < mesh.positions.size(); ++i) {
        lo[i % 3] = std::min(lo[i % 3], static_cast<double>(mesh.positions[i]));
        hi[i % 3] = std::max(hi[i % 3], static_cast<double>(mesh.positions[i]));
    }
    model.accessors[static_cast<size_t>(position)].min = lo;
    model.accessors[static_cast<size_t>(position)].max = hi;

    // Clay mode replaces this material at upload; materials=original shows it.
    CesiumGltf::MaterialPBRMetallicRoughness& pbr =
        model.materials.emplace_back().pbrMetallicRoughness.emplace();
    pbr.baseColorFactor = {0.912, 0.869, 0.787, 1.0};  // Standard's hsl(40, 43%, 93%), linear
    pbr.metallicFactor = 0.0;
    pbr.roughnessFactor = 1.0;

    CesiumGltf::MeshPrimitive& primitive = model.meshes.emplace_back().primitives.emplace_back();
    primitive.attributes["POSITION"] = position;
    primitive.attributes["NORMAL"] = normal;
    primitive.indices = indices;
    primitive.material = static_cast<int32_t>(model.materials.size() - 1);
    primitive.mode = CesiumGltf::MeshPrimitive::Mode::TRIANGLES;

    model.nodes.emplace_back().mesh = static_cast<int32_t>(model.meshes.size() - 1);
    if (model.scenes.empty()) model.scenes.emplace_back();
    if (model.scene < 0) model.scene = 0;
    model.scenes[static_cast<size_t>(model.scene)].nodes.push_back(
        static_cast<int32_t>(model.nodes.size() - 1));
    if (model.asset.version.empty()) model.asset.version = "2.0";
}

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

// The building layer of one z16 square; ok = false when the answer holds none (an empty answer, a
// refusal or garbage, each kind logged once per source).
MvtBuildingLayer read_square(LoaderState& st, spdlog::logger& log, const Fetched& f,
                             const TileId& square, const TileId& t14) {
    if (classify(f) != Outcome::Body) {
        note_status(st, log, kStreets, f, t14);
        return {};
    }
    // curl inflates a gzip Content-Encoding itself; a body that still is gzip is inflated here.
    std::span<const std::byte> bytes(f.body);
    std::vector<std::byte> inflated;
    if (CesiumUtility::isGzip(bytes)) {
        if (!CesiumUtility::gunzip(bytes, inflated)) {
            if (st.first_time("mvt-gzip")) {
                log.warn("mapbox: square 16/{}/{} is not valid gzip; skipped", square.x, square.y);
            }
            return {};
        }
        bytes = std::span<const std::byte>(inflated);
    }
    MvtBuildingLayer layer =
        decode_mvt_building_layer(reinterpret_cast<const uint8_t*>(bytes.data()), bytes.size());
    if (!layer.ok && st.first_time("mvt-decode")) {
        log.warn("mapbox: square 16/{}/{} is not a vector tile; skipped", square.x, square.y);
    }
    return layer;
}

// Worker thread: one z14 tile from its answers, in request order.
TileLoadResult build_tile(const LoaderConfig& cfg, LoaderState& st, const TileId& t14,
                          uint32_t attempt, std::atomic<bool>& settled,
                          const std::vector<Fetched>& fetched,
                          const std::shared_ptr<CesiumAsync::IAssetAccessor>& accessor,
                          spdlog::logger& log, const CesiumGeospatial::Ellipsoid& ellipsoid) {
    const auto started = std::chrono::steady_clock::now();
    const bool transient = std::any_of(fetched.begin(), fetched.end(), [](const Fetched& f) {
        return classify(f) == Outcome::Transient;
    });
    if (transient && attempt < kMaxAttempts) {
        ++st.retries;
        return TileLoadResult::createRetryLaterResult(accessor, nullptr);
    }
    settled = true;  // the next load of this tile starts counting attempts afresh

    const TileFrame frame = make_tile_frame(t14, cfg.origin_height_m);
    const std::array<TileId, 16> squares = footprint_tiles_of(t14);
    TriangleMesh mesh;
    uint64_t extruded = 0;
    uint64_t walls = 0;
    double top = 0.0;
    for (size_t i = 0; i < squares.size(); ++i) {
        const TileId& square = squares[i];
        const MvtBuildingLayer layer = read_square(st, log, fetched[i], square, t14);
        if (!layer.ok) continue;
        const double extent = static_cast<double>(layer.extent);
        const ProjectFn project = [&frame, &square, extent](double u, double v, double z) {
            return project_to_frame(frame, square, u, v, extent, z);
        };
        for (const MvtBuilding& b : layer.buildings) {
            const double base = b.min_height.value_or(0.0);
            if (!passes_standard_3d_filter(b) || !b.height || !std::isfinite(*b.height) ||
                !std::isfinite(base) || *b.height <= base) {
                continue;
            }
            // Clipping to the square's own extent drops the tile buffer, so the copies of a
            // building in neighbouring squares meet without doubled walls.
            const ExtrudeStats es = extrude_polygons(to_tile_polygons(b.polygons), extent, base,
                                                     *b.height, project, mesh);
            if (es.polygons_built == 0) continue;
            ++extruded;
            walls += es.wall_quads;
            top = std::max(top, *b.height);
        }
    }
    st.extruded_features += extruded;
    st.wall_quads += walls;
    st.note_build_ms(
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started)
            .count());

    if (mesh.indices.empty()) return success(TileEmptyContent{}, std::nullopt, accessor, ellipsoid);
    CesiumGltf::Model model;
    append_extrusions(model, mesh);
    ++st.tiles_built;
    return success(std::move(model), region_of(t14, cfg.origin_height_m, top, ellipsoid), accessor,
                   ellipsoid);
}

// Web Mercator XYZ as a cesium quadtree: levels 0-13 are empty and always refined, and each
// level-14 tile carries the buildings. Ids carry Mapbox's XYZ y (north-down); only this loader
// reads them.
class MapboxBuildingsLoader final : public Cesium3DTilesSelection::TilesetContentLoader {
public:
    MapboxBuildingsLoader(std::shared_ptr<const LoaderConfig> config,
                          std::shared_ptr<LoaderState> state)
        : config_(std::move(config)), state_(std::move(state)) {}

    CesiumAsync::Future<TileLoadResult> loadTileContent(const TileLoadInput& input) override {
        const auto* id = std::get_if<CesiumGeometry::QuadtreeTileID>(&input.tile.getTileID());
        if (id == nullptr || id->level != kLandmarkZoom) {
            // Levels 0-13 hold nothing; cesium may unload an empty tile and ask for it again.
            return input.asyncSystem.createResolvedFuture(
                success(TileEmptyContent{}, std::nullopt, input.pAssetAccessor, input.ellipsoid));
        }
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
        // main-thread only; an attempt slower than its wait retries at once.
        a.not_before_s = now + kRetryWaitS * static_cast<double>(1u << (attempt - 1));

        // TileLoadInput is a stack object of references: the continuation copies what it needs.
        const std::string query = "?access_token=" + config_->token;
        std::vector<CesiumAsync::Future<Fetched>> requests;
        for (const TileId& square : footprint_tiles_of(t14)) {
            requests.push_back(fetch(input.asyncSystem, input.pAssetAccessor,
                                     footprint_url(square.x, square.y) + query));
        }
        return input.asyncSystem.all(std::move(requests))
            .thenInWorkerThread([config = config_, state = state_, t14, attempt,
                                 settled = a.settled, accessor = input.pAssetAccessor,
                                 logger = input.pLogger,
                                 ellipsoid = input.ellipsoid](std::vector<Fetched>&& fetched) {
                return build_tile(*config, *state, t14, attempt, *settled, fetched, accessor,
                                  *logger, ellipsoid);
            });
    }

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

// Serves open_mapbox_environment_source (curl, MAPBOX_TOKEN) and install_fixture_mapbox_source
// (a fixture network, a given token) alike. A null `base` means curl.
std::unique_ptr<EnvironmentSource> open_mapbox_source(
    const std::string& mapbox_spec, GeoAnchor anchor,
    std::shared_ptr<CesiumAsync::IAssetAccessor> base, const std::string& raw_token) {
    const std::shared_ptr<spdlog::logger> log = make_redacting_logger();
    const MapboxSpecResult parsed = parse_mapbox_spec(mapbox_spec);
    if (!parsed.spec) {
        log->error("mapbox://{} refused: {}", mapbox_spec, parsed.error);
        return nullptr;
    }
    if (!parsed.warning.empty()) log->warn("mapbox://{}: {}", mapbox_spec, parsed.warning);
    std::string token = trim_ascii_whitespace(raw_token);
    if (token.empty()) {
        log->error("mapbox:// needs MAPBOX_TOKEN in the environment; the source was not opened");
        return nullptr;
    }
    const MapboxSpec& spec = *parsed.spec;
    if (!base) base = std::make_shared<CesiumCurl::CurlAssetAccessor>();
    // ponytail: two worker threads, the default the ion source uses too. Curl blocks one per
    // request in flight, so a tile's requests go two at a time; pass a threadCount here if
    // MapboxLive shows a first tile well over a second.
    CesiumAsync::AsyncSystem asyncSystem(std::make_shared<SimpleTaskProcessor>());
    std::shared_ptr<CountingAssetAccessor> counting;
    // "off": no cache database at all, so no Mapbox byte reaches the disk.
    Cesium3DTilesSelection::TilesetExternals externals =
        build_externals(std::move(base), asyncSystem, "off", kDefaultMaxCacheItems, &counting);

    auto state = std::make_shared<LoaderState>();
    {
        std::lock_guard<std::mutex> lock(g_lastStateMutex);
        g_lastState = state;
    }
    auto loader = std::make_unique<MapboxBuildingsLoader>(
        std::make_shared<const LoaderConfig>(
            LoaderConfig{std::move(token), spec.materials_original, anchor.origin_height_m}),
        std::move(state));
    std::unique_ptr<Tile> root = make_root(*loader, anchor.origin_height_m);
    // Buildings only: never follows terrain, never replaces the ground plane.
    return std::make_unique<StreamingEnvironmentSource>(
        externals, 0, std::string(), std::string(), spec.fallback_dir, anchor, std::move(counting),
        spec.materials_original, false, 0.3, false, 2.0, spec.brightness, spec.radius_m,
        CustomTileset{std::move(loader), std::move(root), true});
}

}

}

namespace overlume {

std::unique_ptr<EnvironmentSource> open_mapbox_environment_source(const std::string& mapbox_spec,
                                                                  GeoAnchor anchor) {
    const char* token = std::getenv("MAPBOX_TOKEN");
    return mapbox::open_mapbox_source(mapbox_spec, anchor, nullptr, token != nullptr ? token : "");
}

}

namespace overlume::testing {

struct MapboxFixtureNetwork {
    std::shared_ptr<overlume::MapFixtureAssetAccessor> accessor;
};

MapboxFixtureNetwork* make_mapbox_fixture_network(uint16_t unmatched_status) {
    return new MapboxFixtureNetwork{
        std::make_shared<overlume::MapFixtureAssetAccessor>(unmatched_status)};
}

void mapbox_fixture_serve(MapboxFixtureNetwork* n, const std::string& url_without_query,
                          uint16_t status, std::vector<uint8_t> body) {
    const auto* bytes = reinterpret_cast<const std::byte*>(body.data());
    n->accessor->serve(url_without_query, status,
                       std::vector<std::byte>(bytes, bytes + body.size()));
}

void mapbox_fixture_fail_transport(MapboxFixtureNetwork* n, const std::string& url_without_query) {
    n->accessor->fail_transport(url_without_query);
}

void mapbox_fixture_hold(MapboxFixtureNetwork* n, const std::string& url_without_query) {
    n->accessor->hold(url_without_query);
}

void mapbox_fixture_release_held(MapboxFixtureNetwork* n) { n->accessor->release_held(); }

std::vector<std::string> mapbox_fixture_requested_urls(MapboxFixtureNetwork* n) {
    return n->accessor->requested_urls();
}

std::string mapbox_landmark_url(uint32_t x14, uint32_t y14) {
    return overlume::mapbox::landmark_url(x14, y14);
}

std::string mapbox_footprint_url(uint32_t x16, uint32_t y16) {
    return overlume::mapbox::footprint_url(x16, y16);
}

bool install_fixture_mapbox_source(overlume::VisualRenderer* r, const char* mapbox_uri,
                                   overlume::GeoAnchor anchor, MapboxFixtureNetwork* n,
                                   const char* token) {
    constexpr char kScheme[] = "mapbox://";
    if (r == nullptr || mapbox_uri == nullptr || n == nullptr) return false;
    const std::string uri(mapbox_uri);
    if (uri.rfind(kScheme, 0) != 0) return false;
    std::unique_ptr<overlume::EnvironmentSource> source = overlume::mapbox::open_mapbox_source(
        uri.substr(sizeof(kScheme) - 1), anchor, n->accessor, token != nullptr ? token : "");
    if (!source) return false;
    source->set_visible(*r, r->environmentVisible);
    if (r->environmentSource) r->environmentSource->teardown(*r);
    r->environmentSource = std::move(source);
    return true;
}

MapboxSourceStats mapbox_last_source_stats() {
    std::shared_ptr<overlume::mapbox::LoaderState> state;
    {
        std::lock_guard<std::mutex> lock(overlume::mapbox::g_lastStateMutex);
        state = overlume::mapbox::g_lastState;
    }
    MapboxSourceStats out;
    if (!state) return out;
    out.tiles_built = state->tiles_built.load();
    out.landmark_nodes = state->landmark_nodes.load();
    out.extruded_features = state->extruded_features.load();
    out.conflated_features = state->conflated_features.load();
    out.retries = state->retries.load();
    out.max_build_ms = state->max_build_ms.load();
    out.wall_quads = state->wall_quads.load();
    return out;
}

void mapbox_set_test_clock(double seconds) { overlume::mapbox::g_testClockS.store(seconds); }

}
```

- [ ] **Step 7: The `mapbox://` dispatch** (`overlume/src/environment.cpp`)

Replace

```cpp
constexpr char kIonPrefix[] = "ion://";
constexpr size_t kIonPrefixLen = sizeof(kIonPrefix) - 1;
}
```

with

```cpp
constexpr char kIonPrefix[] = "ion://";
constexpr size_t kIonPrefixLen = sizeof(kIonPrefix) - 1;
constexpr char kMapboxPrefix[] = "mapbox://";
constexpr size_t kMapboxPrefixLen = sizeof(kMapboxPrefix) - 1;
}
```

and replace

```cpp
        source = open_streaming_environment_source(uri.substr(kIonPrefixLen), anchor);
#else
        return false;
#endif
    } else {
```

with

```cpp
        source = open_streaming_environment_source(uri.substr(kIonPrefixLen), anchor);
#else
        return false;
#endif
    } else if (uri.compare(0, kMapboxPrefixLen, kMapboxPrefix) == 0) {
#ifdef OVERLUME_ENABLE_CESIUM
        source = open_mapbox_environment_source(uri.substr(kMapboxPrefixLen), anchor);
#else
        return false;
#endif
    } else {
```

`OVERLUME_ENABLE_CESIUM` is defined for `overlume_obj`, which compiles `environment.cpp`
(`overlume/CMakeLists.txt:282`); `open_mapbox_environment_source` itself lives in the cesium-only
`mapbox_loader.cpp`.

- [ ] **Step 8: Run the tests and see them pass**

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_mapbox_buildings test_mapbox_uri test_environment_stream
overlume/build/test_mapbox_buildings
overlume/build/test_mapbox_uri | tail -1
ctest --test-dir overlume/build -R '^(EnvironmentStream|TokenRedaction)\.' --output-on-failure
```

Expected: the 16 `MapboxBuildings.*` tests each `[       OK ]` and `[  PASSED  ] 16 tests.`; then
`[  PASSED  ] 10 tests.` for Task 2's parser, untouched by the loader below it; then ctest runs the
stream suite and the redaction probe, which cover the edits to `environment_stream.{hpp,cpp}`,
each test in its own process as the gate runs them (the redacting sink's capture lasts as long as
the process, lane 4's Task 6 API notes), and ends `100% tests passed, 0 tests failed out of 40`.
The 40 are the 34 `EnvironmentStream` tests at the spec commit, Task 6's 1 and Task 7's 4, and
`TokenRedaction`'s 1; if lane 4's review rounds changed its test count, expect what its Task 7
commit left. On a machine without GPU/EGL every `MapboxBuildings` test reports `[  SKIPPED ]`:
run this step on the GPU rig. A `MapboxBuildings` failure here is a finding to diagnose before
going on, never an expectation to loosen.

- [ ] **Step 9: Format**

```bash
cd "$WORKTREE"
git add overlume/src/environment.cpp overlume/src/environment.hpp \
    overlume/src/environment_stream.cpp overlume/src/environment_stream.hpp \
    overlume/src/mapbox_loader.cpp overlume/src/mapbox_test_hooks.hpp \
    overlume/tests/test_mapbox_buildings.cpp
tools/check_format.sh --fix && tools/check_format.sh && git diff --stat && tools/check_spdx.sh
```

Expected: `check_format.sh: reformatted N files with clang-format 23.1.1`, then
`check_format.sh: PASS (N files, clang-format 23.1.1)`, an empty `git diff --stat` (every block
above is already clang-format 23.1.1 clean; if a file is listed, `git add` it again) and
`check_spdx.sh: PASS (… files carry the SPDX identifier)`.

- [ ] **Step 10: Revert checks (do not commit)**: the trim (Review Focus 1), the clip extent
  (Review Focus 2) and the retry wait

```bash
cd "$WORKTREE"
sed -i 's/^    std::string token = trim_ascii_whitespace(raw_token);$/    std::string token = raw_token;  \/\/ REVERT CHECK/' \
    overlume/src/mapbox_loader.cpp
sed -i 's/extrude_polygons(to_tile_polygons(b.polygons), extent, base,$/extrude_polygons(to_tile_polygons(b.polygons), 1.0e9, base,  \/\/ REVERT CHECK/' \
    overlume/src/mapbox_loader.cpp
sed -i 's/^        if (a.count > 0 && now < a.not_before_s) {$/        if (false \&\& a.count > 0 \&\& now < a.not_before_s) {  \/\/ REVERT CHECK/' \
    overlume/src/mapbox_loader.cpp
grep -c 'REVERT CHECK' overlume/src/mapbox_loader.cpp
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_mapbox_buildings
overlume/build/test_mapbox_buildings \
    --gtest_filter='MapboxBuildings.UnsetTokenRefusesAndNamesTheVariable:MapboxBuildings.TokenWhitespaceIsTrimmed:MapboxBuildings.ServerErrorRetriesThenLoads:MapboxBuildings.TransportErrorRetriesThenLoads:MapboxBuildings.PersistentServerErrorGivesUpAfterThreeAttempts:MapboxBuildings.CornerLoadsFourTilesWithoutDoubledSeamWalls' \
    | tail -9
git checkout -- overlume/src/mapbox_loader.cpp
grep -c 'REVERT CHECK' overlume/src/mapbox_loader.cpp || true
git diff --stat
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_mapbox_buildings
```

Expected: the first `grep` prints `3`; the build warns that `trim_ascii_whitespace` is unused
(the trim revert's doing; nothing builds with `-Werror`); the run ends

```
[  FAILED  ] 6 tests, listed below:
[  FAILED  ] MapboxBuildings.UnsetTokenRefusesAndNamesTheVariable
[  FAILED  ] MapboxBuildings.TokenWhitespaceIsTrimmed
[  FAILED  ] MapboxBuildings.ServerErrorRetriesThenLoads
[  FAILED  ] MapboxBuildings.TransportErrorRetriesThenLoads
[  FAILED  ] MapboxBuildings.PersistentServerErrorGivesUpAfterThreeAttempts
[  FAILED  ] MapboxBuildings.CornerLoadsFourTilesWithoutDoubledSeamWalls

 6 FAILED TESTS
```

The first two fail on the untrimmed token: `" \t\n"` opens a source, which requests its tile once
the ego is valid, and every URL ends in `?access_token= pk.test-token-not-real\n`. The three retry
tests fail on `times_requested(n, square0)`: with no wait, cesium's next selection starts the next
attempt at once, so square 0 is asked a second time at 1.9 s on the frozen clock (a third at
5.9 s in the persistent test), and in the two retry-then-load tests the tile loads before the wait
is over. The last fails on `s.wall_quads`: 15, not 14, because the west half no longer loses its
wall along the seam. `git checkout` restores the version Step 9 staged: the second `grep` prints
`0`, `git diff --stat` prints nothing, and the rebuild leaves the binary current.

- [ ] **Step 11: Gate** (foreground, Bash timeout 600000 ms)

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
ROS_DOMAIN_ID=87 CI_VISUAL_MODE_DOMAIN_ID=87 \
    flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock tools/ci_visual_mode.sh
```

Expected: six `PASS` lines (POD header check, library ctest suite, node gtests, WS bridge pytest
suite, golden suite, examples), `OVERALL: PASS`, exit 0. The library ctest count is 16 higher than
at the lane's fork, every `MapboxBuildings.*` entry among the ok ones; the node, WS bridge, golden
(all `ok`, none moved) and example counts equal the fork's. Exit 75 means another lane held the
lock for 300 s: run the same command again. A WS bridge failure can also come from another
session's gate on the same ports: rerun before treating it as a finding.

- [ ] **Step 12: Commit**

```bash
cd "$WORKTREE"
git add overlume/src/environment.cpp overlume/src/environment.hpp \
    overlume/src/environment_stream.cpp overlume/src/environment_stream.hpp \
    overlume/src/mapbox_loader.cpp overlume/src/mapbox_test_hooks.hpp \
    overlume/tests/test_mapbox_buildings.cpp
git status --short
git commit -F - <<'EOF'
feat(stream): stream mapbox://buildings through a custom cesium loader

The Mapbox buildings source (docs/plans/2026-10-14-mapbox-buildings.md,
Task 13) models Web Mercator XYZ as a cesium quadtree: levels 0-13 are
empty and always refined, and each z14 tile the area view selects around
the ego is built on a worker from its 16 streets-v8 z16 tiles (gzip or
raw, Mapbox Standard's 3D filter, clipped to their own square and extruded
in the tile's east-north-up frame) into one glTF node. 429, 5xx and
transport errors send the tile back for at most three attempts, the
second no sooner than 2 s and the third 4 s after the one before began:
cesium asks for a RetryLater tile again every frame, and until the wait
is over the loader answers RetryLater without a request, so a short
burst of 429s or 503s cannot use up the attempts. 401 and 403 log once
per endpoint without a URL; a body that does not decode costs its
square and one line. The token comes from MAPBOX_TOKEN when the source
opens, trimmed of whitespace, and travels only in request URLs: results
never carry their request, so no URL reaches a model, a GLB or a log.
set_environment_source gains the mapbox:// branch.

MapboxBuildings (16 tests) runs on a map fixture accessor that serves,
fails or holds requests by URL, with synthetic MVTs: token handling and
trimming, radius-exact requests, 401 and transport fallback, retries,
their wait (on a frozen test clock) and the attempt cap, garbage bodies,
the four tiles at a corner and their seam, a switch with a load in
flight, and the materials.

Claude-Session: <the executing session's URL>
EOF
```

Expected: `git status --short` lists exactly the seven files, `A ` for the new test and `M ` for
the rest; the commit succeeds. Then report Task 13 committed.

### Task 14: Landmarks and conflation in the loader

**Lane:** 7. **Depends on:** Task 13 (same worktree, after its commit) and Task 5 merged into
`mapbox-environment` (Step 1 brings it into the lane).

**Files:**
- Modify: `overlume/src/mapbox_loader.cpp` (as Task 13 left it: includes, 7-8; constants, 127;
  before `build_tile`, 404; `build_tile`, 421-459; `loadTileContent`'s requests, 501-503)
- Modify: `overlume/tests/test_mapbox_buildings.cpp` (as Task 13 left it: includes, 7-8; the
  namespace aliases, 28; a helper block before the first test, 314;
  `RequestsOnlyTheTileInsideTheRadius`, 393; `RejectedTokenFallsBackAndLogsOnce`, 492-494; five
  tests appended after the last, 687)
- No change to `overlume/src/mapbox_test_hooks.hpp`: `MapboxSourceStats` already carries
  `landmark_nodes` and `conflated_features`.

**Interfaces:**
- Consumes:

```cpp
// overlume/src/mapbox_landmarks.hpp (Task 5)
struct LandmarkTile {
    CesiumGltf::Model model;  // kept building nodes only, all in scene 0, frame-mapped matrices
    std::vector<std::array<double, 6>> footprint_triangles;  // tile-frame metres
    uint32_t buildings_kept = 0;
    uint32_t nodes_skipped = 0;
    bool ok = false;
    std::string error;
};
LandmarkTile read_landmark_tile(std::span<const std::byte> glb, const TileFrame& frame,
                                bool keep_materials);
bool inside_any_footprint(const std::vector<std::array<double, 6>>& triangles, double x, double y);
// overlume/src/mapbox_extrude.hpp (Task 3)
std::optional<std::array<double, 2>> footprint_centroid(const std::vector<TilePolygon>& polygons);
// overlume/tests/mapbox_glb_fixture.hpp (Task 5): glb::LandmarkSpec, glb::GlbOptions,
std::vector<uint8_t> build_landmark_glb(const std::vector<LandmarkSpec>& landmarks,
                                        const GlbOptions& opts);
// Task 13: build_tile, append_extrusions, note_status, classify and the test helpers.
```

- Produces: no new declaration. Behaviour: each z14 tile also requests
  `…/3dtiles/v1/mapbox.mapbox-3dbuildings-v1-meshopt/14/{x}/{y}.glb?access_token=…`
  (its 17th request), merges the kept landmark nodes with the extrusion node, and drops the
  extrusions whose unclipped footprint centroid falls inside a landmark footprint triangle.

**The merge, and why this one:** the extrusions are appended to `LandmarkTile.model` with Task 13's
`append_extrusions`, which already appends a buffer, its views and accessors, a mesh, a material
and a node to whatever model it is given, and starts scene 0 when the model has none.
`Model::merge` (`CesiumGltf/src/Model.cpp:393-422`) builds a combined default scene only when both
models set `scene`, and then as a third scene beside the two originals, and it needs a second model
built only to be copied; appending needs neither and changes no index of the landmark model.
Landmarks are read before the extrusions because their footprints decide which features to drop.
An unreadable GLB (`ok = false`: a bad header, a stream that does not decode) contributes nothing,
not even footprints, and logs once naming the tile; a node with an unexpected matrix logs once
more. 204 means no landmarks; 401/403, 429/5xx and transport failures take the same paths as the
squares'.

**API notes:**
- `overlume/src/mapbox_landmarks.cpp` (Task 5): `read_landmark_tile` checks the `glTF` magic
  before cesium reads (error `not a GLB (no glTF magic)`), returns `ok = false` with the reader's
  first error or `EXT_meshopt_compression: a compressed stream did not decode` when a view kept
  its extension, never a partial model; it keeps materials and drops images, textures and samplers
  only when `keep_materials` is false; the kept model has one scene, `scene = 0`, and several
  buffers, which `consolidate_buffers` (`overlume/src/environment_stream.cpp:199-227`) merges with
  the extrusion buffer before `writeGlb`.
- `CesiumGltf/include/CesiumGltf/Model.h:200-209` `getSafe`; `AccessorSpec.h:154` `max` (the
  landmark's POSITION bounds, in local metres above its ground).
- `CesiumGltfWriter/generated/src/ModelJsonWriter.cpp:2333-2336`: `images` is written only when
  the model has one, which is what the clay test reads.
- `CountingAssetAccessor` (`overlume/src/environment_stream.hpp:109-149`): any 2xx resets the
  consecutive-failure count, so a transport-loss test must fail the landmark URL too (Task 13's
  `TransportLossEverywhereFallsBack` already does).
- `overlume/tests/mapbox_glb_fixture.hpp` (Task 5): a landmark's footprint is a square of
  `footprint_half_m` (12 m) around its node anchor `(tx, ty)` in z14 units, its building a 10 m
  half box; `corrupt_meshopt` garbles the first vertex stream; `bad_matrix_on_first` rotates the
  first building node. Downtown's z14 tile is 10707/7006 (spec section 3); a z14 unit there is
  2213.3 m / 8192 = 0.270 m.

- [ ] **Step 1: Bring Task 5 into the lane**

```bash
cd "$WORKTREE"
git merge --no-ff \
    -m "Merge mapbox-environment into mapbox-env/l7 for the landmark reader (Task 5)" \
    -m "Claude-Session: <the executing session's URL>" mapbox-environment
flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock cmake -S overlume -B overlume/build > /dev/null \
    && echo reconfigured \
    && CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
overlume/build/test_mapbox_landmarks | tail -1
overlume/build/test_mapbox_buildings | tail -1
```

Expected: `Merge made by the 'ort' strategy.` with `overlume/src/mapbox_landmarks.cpp`,
`overlume/src/mapbox_landmarks.hpp`, `overlume/tests/mapbox_glb_fixture.hpp` and
`overlume/tests/test_mapbox_landmarks.cpp` among the files (when Task 5 was in the fork:
`Already up to date.`, or a merge of other lanes' commits without those files); no conflict,
because Task 5's block of `mapbox_test_hooks.hpp` sits above Task 4's and Task 13's below it.
Then `reconfigured`, a clean build, `[  PASSED  ] 12 tests.` (Task 5's suite as lane 3 drafted it;
if its review rounds changed the count, expect what Task 5's commit reported) and
`[  PASSED  ] 16 tests.`. A conflict is a plan deviation: stop and report it. No
`reconfigured` line and no CMake error above it means the lock wait ran out (exit 75) and nothing
was built: run the block again from the `flock` line. This is the merge the head's Merges section
names.

- [ ] **Step 2: Write the failing tests** (`overlume/tests/test_mapbox_buildings.cpp`)

Replace

```cpp
#include "environment_test_hooks.hpp"
#include "mapbox_mvt_fixture.hpp"
```

with

```cpp
#include "environment_test_hooks.hpp"
#include "mapbox_glb_fixture.hpp"
#include "mapbox_mvt_fixture.hpp"
```

Replace

```cpp
namespace mvt = overlume::testing::mvt;
```

with

```cpp
namespace glb = overlume::testing::glb;
namespace mvt = overlume::testing::mvt;
```

Replace

```cpp
TEST(MapboxBuildings, UnsetTokenRefusesAndNamesTheVariable) {
```

with

```cpp
namespace {

// Downtown Dubai's z14 tile, 10707/7006, where Mapbox serves landmark models.
TileId downtown_tile() { return overlume::mapbox::tile_containing(55.2744, 25.1972, 14); }

// A landmark node at (tx, ty) z14 units, with Mapbox's node scale for its own latitude.
glb::LandmarkSpec landmark_at(const TileId& t, uint64_t id, double tx, double ty) {
    glb::LandmarkSpec spec;
    spec.id = id;
    spec.tx = tx;
    spec.ty = ty;
    spec.lat_deg = overlume::mapbox::tile_point_to_lonlat(t, tx, ty, 8192.0).lat;
    return spec;
}

// Square 5 (row 1, column 1) holds its box at z16 units 1000-1400; the box centre is z14 unit
// 2048 + 1200 / 2 on both axes. Square 10 (row 2, column 2): 4096 + 1200 / 2.
constexpr double kOverSquare5 = 2648.0;
constexpr double kOverSquare10 = 4696.0;
// anchor_square_body's 16 boxes plus the neighbour serve_downtown adds to square 5.
constexpr uint64_t kDowntownFeatures = kAnchorTileExtruded + 1;

// A GLB's JSON chunk: the first chunk, after the 12-byte file header and its own 8-byte header.
std::string glb_json(const std::vector<uint8_t>& glb) {
    if (glb.size() < 20) return {};
    const size_t length = size_t{glb[12]} | (size_t{glb[13]} << 8) | (size_t{glb[14]} << 16) |
                          (size_t{glb[15]} << 24);
    if (glb.size() - 20 < length) return {};
    return std::string(glb.begin() + 20, glb.begin() + 20 + static_cast<std::ptrdiff_t>(length));
}

// The tile's squares as anchor_square_body, square 5 with a 27 m box added 81 m east of the
// landmark spot (its centroid outside the 12 m footprint), and `glb_bytes` as its landmark tile.
void serve_downtown(MapboxFixtureNetwork* n, const TileId& t,
                    const std::vector<uint8_t>& glb_bytes) {
    serve_anchor_tile(n, t);
    serve_square(n, overlume::mapbox::footprint_tiles_of(t)[5],
                 {mvt::building(105, {box(1000, 1000, 1400, 1400)}, 15.0),
                  mvt::building(205, {box(1700, 1100, 1900, 1300)}, 12.0)});
    overlume::testing::mapbox_fixture_serve(n, overlume::testing::mapbox_landmark_url(t.x, t.y),
                                            200, glb_bytes);
}

// An unusable landmark tile costs its landmarks only: every extrusion shows, no half-decoded mesh
// reaches the GLB, and one log line names the tile.
void expect_extrusions_only(const TileId& t, const std::vector<uint8_t>& glb_bytes) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    serve_downtown(n, t, glb_bytes);
    overlume::testing::capture_stream_glbs(true);
    const size_t logFrom = log_size();
    ASSERT_TRUE(install(r, "mapbox://buildings", anchor_at(t, 0.5, 0.5), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 400, [&] { return loaded(r) >= 1; }));
    const std::vector<std::vector<uint8_t>> glbs = overlume::testing::captured_stream_glbs();
    overlume::testing::capture_stream_glbs(false);
    const overlume::testing::MapboxSourceStats s = stats();
    EXPECT_EQ(s.landmark_nodes, 0u);
    EXPECT_EQ(s.conflated_features, 0u) << "no footprints come from an unusable tile";
    EXPECT_EQ(s.extruded_features, kDowntownFeatures);
    ASSERT_EQ(glbs.size(), 1u);
    EXPECT_FALSE(contains(glbs[0], "EXT_meshopt_compression")) << "a half-decoded mesh got through";
    const std::string tile = "14/" + std::to_string(t.x) + "/" + std::to_string(t.y);
    const std::string log = log_since(logFrom);
    EXPECT_EQ(count_of(log, "landmarks of tile " + tile + " skipped"), 1u);
    EXPECT_EQ(log.find("api.mapbox.com"), std::string::npos);
    overlume::destroy_renderer(r);
}

}

TEST(MapboxBuildings, UnsetTokenRefusesAndNamesTheVariable) {
```

Replace

```cpp
    const std::set<std::string> expected = squares_of({t});
    EXPECT_EQ(requested(n), expected)
```

with

```cpp
    std::set<std::string> expected = squares_of({t});
    expected.insert(overlume::testing::mapbox_landmark_url(t.x, t.y));
    EXPECT_EQ(requested(n), expected)
```

Replace

```cpp
        << "16 rejected squares, one line";
```

with

```cpp
        << "16 rejected squares, one line";
    EXPECT_EQ(count_of(log, "Mapbox rejected MAPBOX_TOKEN (HTTP 401) for 3D landmarks"), 1u)
        << "one line per endpoint";
```

and after the last test, replace

```cpp
        EXPECT_EQ((*color)[3], 1.0);
        overlume::destroy_renderer(r);
    }
}
```

with

```cpp
        EXPECT_EQ((*color)[3], 1.0);
        overlume::destroy_renderer(r);
    }
}

TEST(MapboxBuildings, LandmarkReplacesTheExtrusionBeneathIt) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const TileId t = downtown_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    serve_downtown(n, t,
                   glb::build_landmark_glb({landmark_at(t, 7, kOverSquare5, kOverSquare5)},
                                           glb::GlbOptions{}));
    overlume::testing::capture_stream_glbs(true);
    ASSERT_TRUE(install(r, "mapbox://buildings", anchor_at(t, 0.5, 0.5), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 400, [&] { return loaded(r) >= 1; }));
    const std::vector<std::vector<uint8_t>> glbs = overlume::testing::captured_stream_glbs();
    overlume::testing::capture_stream_glbs(false);
    EXPECT_EQ(loaded(r), 1u) << "landmark and extrusions share the tile's one model";
    const overlume::testing::MapboxSourceStats s = stats();
    EXPECT_EQ(s.landmark_nodes, 1u);
    EXPECT_EQ(s.conflated_features, 1u) << "square 5's box stands under the landmark footprint";
    EXPECT_EQ(s.extruded_features, kDowntownFeatures - 1) << "its neighbour 81 m east stays";
    ASSERT_EQ(glbs.size(), 1u);
    const std::string json = glb_json(glbs[0]);
    EXPECT_EQ(count_of(json, "\"mesh\":"), 2u)
        << "one landmark node and one extrusion node (only nodes have a \"mesh\" key)";
    EXPECT_NE(json.find("\"nodes\":[0,1]"), std::string::npos) << "both in the tile's one scene";
    overlume::destroy_renderer(r);
}

TEST(MapboxBuildings, ClayDropsLandmarkImagesOriginalKeepsOcclusion) {
    for (const bool original : {false, true}) {
        auto* r = make_renderer();
        if (!r) GTEST_SKIP() << "no GPU/EGL";
        const TileId t = downtown_tile();
        auto* n = overlume::testing::make_mapbox_fixture_network(204);
        serve_downtown(n, t,
                       glb::build_landmark_glb({landmark_at(t, 7, kOverSquare5, kOverSquare5)},
                                               glb::GlbOptions{}));
        overlume::testing::capture_stream_glbs(true);
        ASSERT_TRUE(
            install(r, original ? "mapbox://buildings?materials=original" : "mapbox://buildings",
                    anchor_at(t, 0.5, 0.5), n));
        set_ego(r, 0.0, 0.0);
        std::vector<uint8_t> buf(kFrameBytes);
        ASSERT_TRUE(pump(r, buf, 400, [&] { return loaded(r) >= 1; }));
        const std::vector<std::vector<uint8_t>> glbs = overlume::testing::captured_stream_glbs();
        overlume::testing::capture_stream_glbs(false);
        EXPECT_EQ(stats().landmark_nodes, 1u);
        ASSERT_EQ(glbs.size(), 1u);
        EXPECT_EQ(contains(glbs[0], "\"images\""), original)
            << (original ? "original keeps the landmark's JPEG" : "clay drops every image");
        EXPECT_EQ(contains(glbs[0], "\"occlusionTexture\""), original);
        overlume::destroy_renderer(r);
    }
}

TEST(MapboxBuildings, CorruptMeshoptKeepsExtrusionsAndLogsTheTileOnce) {
    const TileId t = downtown_tile();
    glb::GlbOptions opts;
    opts.corrupt_meshopt = true;
    expect_extrusions_only(
        t, glb::build_landmark_glb({landmark_at(t, 7, kOverSquare5, kOverSquare5)}, opts));
}

TEST(MapboxBuildings, BadGlbHeaderKeepsExtrusionsAndLogsTheTileOnce) {
    const TileId t = downtown_tile();
    std::vector<uint8_t> bytes =
        glb::build_landmark_glb({landmark_at(t, 7, kOverSquare5, kOverSquare5)}, glb::GlbOptions{});
    bytes[0] = 'x';  // the "glTF" magic, broken
    expect_extrusions_only(t, bytes);
}

TEST(MapboxBuildings, UnexpectedNodeMatrixSkipsOnlyThatLandmark) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const TileId t = downtown_tile();
    auto* n = overlume::testing::make_mapbox_fixture_network(204);
    glb::GlbOptions opts;
    opts.bad_matrix_on_first = true;
    // Landmark 6, whose building node the fixture rotates, stands over square 10's box; landmark 7
    // over square 5's. Conflation uses every footprint the reader returns. In Mapbox's layout a
    // footprint node carries its building's matrix, so a matrix that skips a building skips its
    // footprint too and the extrusion beneath stays. The fixture rotates only the building node:
    // landmark 6's footprint survives, square 10's box goes, and nothing stands at that spot.
    serve_downtown(n, t,
                   glb::build_landmark_glb({landmark_at(t, 6, kOverSquare10, kOverSquare10),
                                            landmark_at(t, 7, kOverSquare5, kOverSquare5)},
                                           opts));
    const size_t logFrom = log_size();
    ASSERT_TRUE(install(r, "mapbox://buildings", anchor_at(t, 0.5, 0.5), n));
    set_ego(r, 0.0, 0.0);
    std::vector<uint8_t> buf(kFrameBytes);
    ASSERT_TRUE(pump(r, buf, 400, [&] { return loaded(r) >= 1; }));
    const overlume::testing::MapboxSourceStats s = stats();
    EXPECT_EQ(s.landmark_nodes, 1u) << "landmark 7 is kept";
    EXPECT_EQ(s.conflated_features, 2u)
        << "square 5's box under landmark 7, square 10's under landmark 6's surviving footprint";
    EXPECT_EQ(s.extruded_features, kDowntownFeatures - 2);
    EXPECT_EQ(count_of(log_since(logFrom), "with an unexpected matrix skipped"), 1u);
    overlume::destroy_renderer(r);
}
```

What each test catches when its behaviour is reverted:
- `LandmarkReplacesTheExtrusionBeneathIt`: no landmark request or no merge (`landmark_nodes` 0);
  no conflation (`conflated_features` 0, 17 extruded; Step 6 runs that revert); a conflated
  feature counted as extruded too; conflating by more than the centroid (the box 81 m east would
  go too); a merge that drops either node from the written GLB (one `"mesh":` key, not two) or
  leaves one out of the scene (spec section 8: landmark and extrusion entities are in the scene).
- `ClayDropsLandmarkImagesOriginalKeepsOcclusion`: `keep_materials` not wired to
  `materials=original` (the occlusion texture lost in original mode, or the image kept in clay).
- `CorruptMeshoptKeepsExtrusionsAndLogsTheTileOnce` (Review Focus 4) and
  `BadGlbHeaderKeepsExtrusionsAndLogsTheTileOnce` (Review Focus 3): failing or retrying the tile for
  an unreadable landmark GLB (no extrusions), merging a half-read model (`EXT_meshopt_compression`
  in the GLB), footprints from an unreadable tile (a conflated box), no line or a line per tile.
- `UnexpectedNodeMatrixSkipsOnlyThatLandmark`: dropping the whole landmark tile for one bad node
  (`landmark_nodes` 0, `conflated_features` 0), or skipping the node silently. It also pins the
  conflation rule, every footprint the reader returns (the contract's `read_landmark_tile` keeps
  the footprint of a building it skipped): conflating only against the kept buildings' footprints
  keeps square 10's box (`conflated_features` 1).
- `RequestsOnlyTheTileInsideTheRadius` (amended): the landmark URL missing or asked twice.
- `RejectedTokenFallsBackAndLogsOnce` (amended): no line, or more than one, for the landmark
  endpoint.

- [ ] **Step 3: Run them and see them fail**

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_mapbox_buildings
overlume/build/test_mapbox_buildings | tail -10
```

Expected: the tests compile against Task 13's loader, which requests no landmark tile, and the run
ends

```
[  FAILED  ] 7 tests, listed below:
[  FAILED  ] MapboxBuildings.RequestsOnlyTheTileInsideTheRadius
[  FAILED  ] MapboxBuildings.RejectedTokenFallsBackAndLogsOnce
[  FAILED  ] MapboxBuildings.LandmarkReplacesTheExtrusionBeneathIt
[  FAILED  ] MapboxBuildings.ClayDropsLandmarkImagesOriginalKeepsOcclusion
[  FAILED  ] MapboxBuildings.CorruptMeshoptKeepsExtrusionsAndLogsTheTileOnce
[  FAILED  ] MapboxBuildings.BadGlbHeaderKeepsExtrusionsAndLogsTheTileOnce
[  FAILED  ] MapboxBuildings.UnexpectedNodeMatrixSkipsOnlyThatLandmark

 7 FAILED TESTS
```

The other 14 pass: the corner and radius tests compare MVT URLs only.

- [ ] **Step 4: Request, read and merge the landmark tile; conflate**
  (`overlume/src/mapbox_loader.cpp`)

Replace

```cpp
#include "mapbox_extrude.hpp"
#include "mapbox_mvt.hpp"
```

with

```cpp
#include "mapbox_extrude.hpp"
#include "mapbox_landmarks.hpp"
#include "mapbox_mvt.hpp"
```

Replace

```cpp
constexpr char kStreets[] = "streets-v8 footprints";
```

with

```cpp
constexpr char kStreets[] = "streets-v8 footprints";
constexpr char kLandmarks[] = "3D landmarks";
constexpr size_t kLandmarkIndex = 16;  // after the 16 squares in a tile's requests
```

Replace

```cpp
// Worker thread: one z14 tile from its answers, in request order.
```

with

```cpp
// Highest POSITION z over a landmark model's meshes: local metres above the ground.
double landmark_top_m(const CesiumGltf::Model& model) {
    double top = 0.0;
    for (const CesiumGltf::Mesh& mesh : model.meshes) {
        for (const CesiumGltf::MeshPrimitive& primitive : mesh.primitives) {
            const auto it = primitive.attributes.find("POSITION");
            if (it == primitive.attributes.end()) continue;
            const CesiumGltf::Accessor* position =
                CesiumGltf::Model::getSafe(&model.accessors, it->second);
            if (position != nullptr && position->max.size() == 3) {
                top = std::max(top, position->max[2]);
            }
        }
    }
    return top;
}

// Worker thread: one z14 tile from its answers, in request order.
```

Replace

```cpp
    const std::array<TileId, 16> squares = footprint_tiles_of(t14);
    TriangleMesh mesh;
    uint64_t extruded = 0;
    uint64_t walls = 0;
    double top = 0.0;
```

with

```cpp
    const std::array<TileId, 16> squares = footprint_tiles_of(t14);
    // Landmarks first: their footprints decide which extrusions to drop. An unreadable GLB costs
    // the landmarks of this tile, never its extrusions.
    LandmarkTile landmarks;
    const Fetched& glb = fetched[kLandmarkIndex];
    if (classify(glb) == Outcome::Body) {
        landmarks = read_landmark_tile(glb.body, frame, cfg.materials_original);
        if (!landmarks.ok && st.first_time("glb")) {
            log.warn("mapbox: landmarks of tile 14/{}/{} skipped: {}", t14.x, t14.y,
                     landmarks.error);
        }
        if (landmarks.ok && landmarks.nodes_skipped > 0 && st.first_time("glb-node")) {
            log.warn("mapbox: tile 14/{}/{}: {} landmark node(s) with an unexpected matrix skipped",
                     t14.x, t14.y, landmarks.nodes_skipped);
        }
    } else {
        note_status(st, log, kLandmarks, glb, t14);
    }
    TriangleMesh mesh;
    uint64_t extruded = 0;
    uint64_t conflated = 0;
    uint64_t walls = 0;
    double top = landmarks.ok ? landmark_top_m(landmarks.model) : 0.0;
```

Replace

```cpp
            // Clipping to the square's own extent drops the tile buffer, so the copies of a
            // building in neighbouring squares meet without doubled walls.
            const ExtrudeStats es = extrude_polygons(to_tile_polygons(b.polygons), extent, base,
                                                     *b.height, project, mesh);
```

with

```cpp
            const std::vector<TilePolygon> polygons = to_tile_polygons(b.polygons);
            // A landmark model replaces the extrusion whose unclipped footprint centroid it
            // covers, as GL JS does.
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
```

Replace

```cpp
    st.extruded_features += extruded;
    st.wall_quads += walls;
```

with

```cpp
    st.extruded_features += extruded;
    st.conflated_features += conflated;
    st.wall_quads += walls;
```

Replace

```cpp
    if (mesh.indices.empty()) return success(TileEmptyContent{}, std::nullopt, accessor, ellipsoid);
    CesiumGltf::Model model;
    append_extrusions(model, mesh);
    ++st.tiles_built;
```

with

```cpp
    const bool haveLandmarks = landmarks.ok && landmarks.buildings_kept > 0;
    if (mesh.indices.empty() && !haveLandmarks) {
        return success(TileEmptyContent{}, std::nullopt, accessor, ellipsoid);
    }
    // The landmark model already holds its nodes in scene 0; the extrusions join it there.
    CesiumGltf::Model model = haveLandmarks ? std::move(landmarks.model) : CesiumGltf::Model();
    if (!mesh.indices.empty()) append_extrusions(model, mesh);
    if (haveLandmarks) st.landmark_nodes += landmarks.buildings_kept;
    ++st.tiles_built;
```

and in `loadTileContent` replace

```cpp
            requests.push_back(fetch(input.asyncSystem, input.pAssetAccessor,
                                     footprint_url(square.x, square.y) + query));
        }
```

with

```cpp
            requests.push_back(fetch(input.asyncSystem, input.pAssetAccessor,
                                     footprint_url(square.x, square.y) + query));
        }
        requests.push_back(fetch(input.asyncSystem, input.pAssetAccessor,
                                 landmark_url(t14.x, t14.y) + query));  // kLandmarkIndex
```

- [ ] **Step 5: Run them and see them pass**

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_mapbox_buildings test_mapbox_landmarks
overlume/build/test_mapbox_buildings
overlume/build/test_mapbox_landmarks | tail -1
```

Expected: all 21 `MapboxBuildings.*` tests `[       OK ]` and `[  PASSED  ] 21 tests.`, then
`[  PASSED  ] 12 tests.` for Task 5's suite (or the count Task 5's commit reported, as in Step 1).
On a machine without GPU/EGL the `MapboxBuildings` tests skip: run this on the GPU rig.

- [ ] **Step 6: Format, then the revert check (do not commit)**: conflation

```bash
cd "$WORKTREE"
git add overlume/src/mapbox_loader.cpp overlume/tests/test_mapbox_buildings.cpp
tools/check_format.sh --fix && tools/check_format.sh && git diff --stat
sed -i 's/^            if (landmarks.ok) {$/            if (false \&\& landmarks.ok) {  \/\/ REVERT CHECK/' \
    overlume/src/mapbox_loader.cpp
grep -c 'REVERT CHECK' overlume/src/mapbox_loader.cpp
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_mapbox_buildings
overlume/build/test_mapbox_buildings \
    --gtest_filter='MapboxBuildings.LandmarkReplacesTheExtrusionBeneathIt:MapboxBuildings.UnexpectedNodeMatrixSkipsOnlyThatLandmark' \
    | tail -5
git checkout -- overlume/src/mapbox_loader.cpp
grep -c 'REVERT CHECK' overlume/src/mapbox_loader.cpp || true
git diff --stat
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8 \
    --target test_mapbox_buildings
```

Expected: `check_format.sh: reformatted N files with clang-format 23.1.1`, then
`check_format.sh: PASS (N files, clang-format 23.1.1)` and an empty `git diff --stat`; the `grep`
prints `1`; both tests fail on `s.conflated_features` and `s.extruded_features` (0 and 17, where
`LandmarkReplacesTheExtrusionBeneathIt` wants 1 and 16 and
`UnexpectedNodeMatrixSkipsOnlyThatLandmark` 2 and 15), the run ending

```
[  FAILED  ] 2 tests, listed below:
[  FAILED  ] MapboxBuildings.LandmarkReplacesTheExtrusionBeneathIt
[  FAILED  ] MapboxBuildings.UnexpectedNodeMatrixSkipsOnlyThatLandmark

 2 FAILED TESTS
```

After the checkout the second `grep` prints `0`, `git diff --stat` prints nothing, and the rebuild
leaves the binary current.

- [ ] **Step 7: Gate** (foreground, Bash timeout 600000 ms)

```bash
cd "$WORKTREE"
CCACHE_BASEDIR="$(dirname "$INTEGRATION")" cmake --build overlume/build -j8
ROS_DOMAIN_ID=87 CI_VISUAL_MODE_DOMAIN_ID=87 \
    flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock tools/ci_visual_mode.sh
```

Expected: six `PASS` lines, `OVERALL: PASS`, exit 0; the library ctest count is 17 higher than at
Task 13's gate (Task 5's 12 entries from Step 1's merge, or the count Task 5's commit reported,
and Task 14's 5), or 5 higher when Task 5 was already in the fork (Step 1 printed
`Already up to date.` or merged none of Task 5's files), with every `MapboxBuildings.*` entry ok.
No golden moved; the node, WS bridge and example counts are Task 13's gate's plus whatever tests
Step 1's merge brought in from lanes 5 and 6. Exit 75: run the same command again.

- [ ] **Step 8: Commit**

```bash
cd "$WORKTREE"
git add overlume/src/mapbox_loader.cpp overlume/tests/test_mapbox_buildings.cpp
git status --short
git commit -F - <<'EOF'
feat(mapbox): merge landmark models into z14 tiles, conflate extrusions

Each z14 tile now also requests its landmark GLB (the -meshopt variant,
vertex codec v0; GL JS's -meshopt-v2-lod tiles use codec v1, which the
linked meshoptimizer cannot decode) and reads it with Task 5's reader
before the extrusions: a streets-v8 feature whose unclipped footprint
centroid lies inside a landmark footprint triangle is dropped, as GL JS
drops it, so Downtown's towers do not stand inside doubled extrusions.
The extrusion node is appended to the landmark model's one scene, with
one more buffer, so no second model needs merging. An unreadable
landmark tile (a bad header, a meshopt stream that does not decode)
costs only its landmarks and one line naming the tile; a node with an
unexpected matrix costs that node and one line; 204 means no landmarks.

Five MapboxBuildings tests build Mapbox-layout GLBs in the test process:
the merge and conflation, clay versus original materials, a corrupt
stream, a bad header and a rotated node.

Claude-Session: <the executing session's URL>
EOF
```

Expected: `git status --short` lists the two files as `M `; the commit succeeds. Then report Task 14
committed: lane 7 is ready to merge (Task 15 starts once every lane 1–7 is merged).

---

### Task 15: Golden candidate and promotion

**Lane:** 8. **Depends on:** every lane 1–7 merged into `mapbox-environment`, so Tasks 1–14 are
all in the branch (Execution, Lanes). Lane 8 edits, builds and gates in `$INTEGRATION`, the
worktree where the orchestrator merges, so nothing may merge while a lane 8 task has changes there,
and the orchestrator commits nothing there (ledger rows included) until that task's commit
(Execution, Merges): Step 10 stages files that any other commit would sweep in. The build writes
test output to `/tmp` (`OVERLUME_TMP_DIR`, `overlume/CMakeLists.txt:590-596`). The implementer
runs Steps 1–4 and stops; the orchestrator runs Step 5 with the user; only after the user's
explicit approval does the implementer continue at Step 6.

**Files:**
- Modify: `overlume/tests/test_mapbox_buildings.cpp` (the include block at the top; one block
  appended after the last test, which Task 14 wrote)
- Create: `overlume/tests/goldens/mapbox_buildings_synthetic_dark_adas.png` (Git LFS through
  `.gitattributes:3`; created only after the user approves the candidate)
- Modify: `docs/status.md` (one row at the end of the `## Shipped` table)
- Test: `overlume/tests/test_mapbox_buildings.cpp`, `MapboxBuildingsGolden.SyntheticTileDarkAdas`

**Interfaces:**
- Consumes, from the Interface contract:
  - `overlume/src/mapbox_test_hooks.hpp` (Task 13):
    `MapboxFixtureNetwork* make_mapbox_fixture_network(uint16_t unmatched_status = 404);`,
    `void mapbox_fixture_serve(MapboxFixtureNetwork* n, const std::string& url_without_query, uint16_t status, std::vector<uint8_t> body);`,
    `std::vector<std::string> mapbox_fixture_requested_urls(MapboxFixtureNetwork* n);`,
    `std::string mapbox_landmark_url(uint32_t x14, uint32_t y14);`,
    `std::string mapbox_footprint_url(uint32_t x16, uint32_t y16);`,
    `bool install_fixture_mapbox_source(overlume::VisualRenderer* r, const char* mapbox_uri, overlume::GeoAnchor anchor, MapboxFixtureNetwork* n, const char* token);`,
    `struct MapboxSourceStats` and `MapboxSourceStats mapbox_last_source_stats();`
  - `overlume/src/mapbox_tile_frame.hpp` (Task 4, C++17-clean, no cesium type): `struct TileId`,
    `struct LonLatDeg`, `LonLatDeg tile_point_to_lonlat(const TileId& t, double u, double v, double extent);`,
    `std::array<TileId, 16> footprint_tiles_of(const TileId& z14);`, `kLandmarkExtent`
  - `overlume/tests/mapbox_mvt_fixture.hpp` (Task 1): `struct Feature`, `struct Layer`,
    `std::vector<uint8_t> encode_tile(const std::vector<Layer>& layers);`,
    `std::vector<uint8_t> gzip_stored(const std::vector<uint8_t>& raw);`,
    `Feature building(uint64_t id, std::vector<std::vector<std::array<int32_t, 2>>> rings, double height, double min_height = 0.0, bool extrude = true, bool underground = false, std::string type = "building");`
  - `overlume/tests/mapbox_glb_fixture.hpp` (Task 5): `struct LandmarkSpec`, `struct GlbOptions`,
    `std::vector<uint8_t> build_landmark_glb(const std::vector<LandmarkSpec>& landmarks, const GlbOptions& opts);`
  - existing: `double render_and_compare(overlume::VisualRenderer* r, const overlume::CameraPose& pose, const char* golden_png_path, const char* out_png_path);`
    (`overlume/tests/golden.hpp:121-122`), `kSsimMin` (`golden.hpp:16-20`),
    `uint64_t environment_loaded_chunk_count(overlume::VisualRenderer* r);`
    (`overlume/src/environment_test_hooks.hpp:15`), `kThemeDir` (`overlume/tests/test_paths.hpp:14-16`),
    `bool project_to_screen(VisualRenderer*, Vec3 world_point, float* out_x, float* out_y);`
    (`overlume/include/overlume/scene.h:207`)
- Produces: `TEST(MapboxBuildingsGolden, SyntheticTileDarkAdas)` and the promoted golden. No
  other task calls into them; Task 17's final gate depends on the golden being promoted.

**API notes:**
- Public API: `create_renderer`, `destroy_renderer`, `render_frame` (`overlume/include/overlume/api.h:33-35`);
  `CameraPose` (`api.h:11-15`); `RenderConfig` (`api.h:17-23`); `FrameView` (`api.h:25-29`); `Vec3`
  (`overlume/include/overlume/scene.h:13-15`); `EgoState` (`scene.h:39-44`); `set_scene`
  (`scene.h:189`); `GeoAnchor{origin_lat_deg, origin_lon_deg, heading_rad, origin_height_m}`
  (`scene.h:211-216`); `environment_source_state` and `EnvironmentSourceState : uint8_t` with
  `STREAMING = 2`, `STREAMING_FALLBACK = 3` (`scene.h:268-275`).
- The environment updates only while the ego is valid (`overlume/src/renderer.cpp:1114-1116`). The
  camera is `lookAt(eye, target, +z)` with a 0.1–500 m depth range (`renderer.cpp:1127-1130`).
  The clay ground plane is ±60 m (`renderer.cpp:115`), centred on the ego snapped to the 2 m grid
  pitch (`renderer.cpp:1047-1059`, `renderer_internal.hpp:71`).
- `render_and_compare` renders at the fixed 320×240 (`overlume/tests/golden.cpp:24-25`), writes
  the frame first and returns 0.0 when the golden is missing (`golden.cpp:68-94`); `kSsimMin` is
  0.98 (`golden.hpp:19`). `python3 overlume/tests/golden.py --show PNG` opens a PNG for a look
  (`golden.py:35-42`).
- Quality 1 renders with FXAA, not TAA (`renderer.cpp:583-591`), so a settled frame is
  byte-identical run to run; the streamed fixture golden relies on the same
  (`overlume/tests/test_environment_stream.cpp:428-450`). It also turns on SSAO and the sun's
  shadows (`renderer.cpp:579`, `:678`), and streamed tiles cast and receive them
  (`environment_stream.cpp:661-662` before Tasks 6, 7 and 13).
- `project_to_screen` projects through the camera the last `render_frame` set
  (`renderer.cpp:1127-1130`, `:1210-1223`) and returns coordinates normalised to [0, 1] with y
  down. Row 0 of a frame buffer is the top of the picture: `ThemeGolden.EmptyWorld_DarkAdas`
  asserts the bottom third brighter than the top (`overlume/tests/test_theme.cpp:578`), and the
  node draws callouts at `project_to_screen`'s point times the frame size
  (`ros/src/overlume_ros/src/callouts.cpp:50-53`).
- No ego is drawn: only `set_ego_model` (`scene.h:193`) creates one, and `update_ego_transform`
  returns early without it (`overlume/src/ego.cpp:146`). `ego.valid = 1` here only lets the
  environment update.
- A 404 counts as a failed request (`CountingAssetAccessor`, `overlume/src/environment_stream.hpp:109-149`)
  and eight in a row trip the fallback (`environment_stream.hpp:50`, `environment_stream.cpp:769-774`),
  so every square the loader requests is served with a 200.
- MVT exterior rings have a positive surveyor's area in tile coordinates with y down, holes a
  negative one (`mapbox_mvt.hpp` in the Interface contract; MVT 2.1 §4.3.4.4).
- Gate stage 5 counts gtest lines whose name contains `Golden` (`tools/ci_visual_mode.sh:150-171`).
- gtest 1.15.2 prints `EXPECT_EQ` mismatches as `Expected equality of these values:` with
  `Which is:` lines, and a scoped enum without `operator<<` as raw bytes, e.g.
  `1-byte object <03>` (googletest `gtest-printers.h:308-353`, `src/gtest-printers.cc:89-92`).

**Scene.** Designed in tile units, then rendered on the GPU while this plan was written, as a
stand-in: the same boxes in map metres as one baked chunk. A baked chunk takes the same
`r->buildingMaterial` and shadows as a streamed clay tile (`overlume/src/environment.cpp:81-84`;
`environment_stream.cpp:658-662` before Tasks 6, 7 and 13). It used this camera, theme and quality.
Every present element is in frame, and both absent slots are in clear view. The conflated block
appearing changes 2.3 % of the pixels, the extruded outline 2.8 %, yet the block SSIM against the
stand-in stays at 0.989 and 0.993, so the counts catch those, not the SSIM. A span standing on the
ground (SSIM 0.999) and a courtyard without its hole (0.997) each move under 1 % of the pixels, so
four probe pixels catch those (Step 2).

| Element | Source | Footprint (m east, m north of the tile centre) | Heights | In the golden |
|---|---|---|---|---|
| Tower | landmark GLB node, id 1, meshopt + quantized + LOD scene + occlusion texture | box ±8 m around (−30, 185); footprint ±10 m | 0–64 m | present |
| Block under the tower | MVT 103, square (1, 1) | −45..−15, 170..200 (centroid (−30, 185), inside the footprint) | 0–30 m | absent: conflated |
| Courtyard block | MVT 101, square (1, 1) | −55..−25, 110..140; hole −47..−33, 118..132 | 0–9 m | present, open courtyard |
| Split block | MVT 102 in squares (1, 1) and (2, 1), each copy clipped to the 33-unit buffer as Mapbox serves it | −12..12, 170..195 | 0–20 m | present, no seam at e = 0 |
| Gate pillars | MVT 104 and 105, `building:part`, square (2, 1) | 18..24 and 46..52, 120..126 | 0–18 m | present |
| Gate span | MVT 106, `building:part`, square (2, 1) | 24..46, 120..126 | 10–18 m | present, floating |
| Outline | MVT 107, `extrude=false`, square (2, 1) | 25..50, 160..185 | 0–15 m | absent: filtered |

- The anchor is the centre of z14 tile 14/10707/7006, the test site's tile, with heading
  π/2: map +x is east, +y north, and map (0, 0) is the corner its four central z16 squares share.
  There 1 m is about 3.698 z14 units and 7.395 z16 units.
- The ego sits at map (0, 150, 0). That is more than 500 m from every edge of the 2.2 km z14 tile,
  so the area view selects this tile alone (spec §5, Tiling and selection), and its ±60 m ground
  plane holds every element.
- The camera looks north from (0, 45, 80) at (0, 165, 5) with a 60° vertical field, about 32°
  down: the courtyard opening and the ground under the span both show. `kStdPose`
  (`test_environment_stream.cpp:47`) looks along the ground from 8 m behind the ego and would hide
  the courtyard and both absent slots behind the front row, so this follows the streamed fixture
  golden's raised pose instead (`test_environment_stream.cpp:440`).
- Probes: the 3×3 mean luminance (Rec. 709, as `golden.cpp:27`) where four map points land. On
  the stand-in, open ground at (0, 100, 0) reads 42.3 and the left pillar's south face at
  (21, 120, 9) 49.5. The ray through (35, 123, 5) passes under the span to sunlit ground near
  (37, 128, 0) and reads 41.7, and 48.9 (clay) when the span stands on the ground. The courtyard
  floor at (−40, 130, 0) lies in the block's shadow and reads 37.3, and 57.8 (the lit roof)
  without the hole. So each probe sits much nearer one reference than the other: distances to
  ground and clay of 0.5 and 7.8 (span) and 5.0 and 12.2 (courtyard) as built; 6.7 and 0.6, and
  15.6 and 8.4, mutated.
- Served: the tile's landmark GLB and all 16 MVTs, gzip-wrapped as Mapbox always serves them.
  Squares without buildings get a tile with an empty `building` layer. Any other request is a
  test failure.

- [ ] **Step 1: Make sure the include block holds what this test uses**

Tasks 13 and 14 created the file, and their include block already has most of these lines. Add
each missing one to its group at the top of `overlume/tests/test_mapbox_buildings.cpp`: quoted
project headers first, then `<gtest/gtest.h>`, then standard headers, each group alphabetical, as
in `test_environment_stream.cpp`.

```cpp
#include "overlume/api.h"
#include "overlume/scene.h"

#include "environment_test_hooks.hpp"
#include "golden.hpp"
#include "mapbox_glb_fixture.hpp"
#include "mapbox_mvt_fixture.hpp"
#include "mapbox_test_hooks.hpp"
#include "mapbox_tile_frame.hpp"
#include "test_paths.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <set>
#include <string>
#include <vector>
```

Check:

```bash
cd "$INTEGRATION"
for h in '"overlume/api.h"' '"overlume/scene.h"' '"environment_test_hooks.hpp"' '"golden.hpp"' \
         '"mapbox_glb_fixture.hpp"' '"mapbox_mvt_fixture.hpp"' '"mapbox_test_hooks.hpp"' \
         '"mapbox_tile_frame.hpp"' '"test_paths.hpp"' '<gtest/gtest.h>' '<algorithm>' '<array>' \
         '<cmath>' '<cstdint>' '<set>' '<string>' '<vector>'; do
    grep -qxF "#include $h" overlume/tests/test_mapbox_buildings.cpp || echo "missing $h"
done
```

Expected: no output.

- [ ] **Step 2: Append the golden test to the end of `overlume/tests/test_mapbox_buildings.cpp`**

```cpp
namespace {

constexpr double kGoldenPi = 3.14159265358979323846;

// The golden's synthetic z14 tile is the test site's (55.2744 E, 25.1972 N). Its anchor is
// the tile centre with heading pi/2: map +x is east, +y north, and map (0, 0) is the corner the
// four central z16 squares share.
const overlume::mapbox::TileId kGoldenZ14{14, 10707, 7006};

struct GoldenFrame {
    overlume::GeoAnchor anchor{};
    double z14_per_m = 0.0;  // z14 tile units per ground metre at the tile centre
};

GoldenFrame golden_frame() {
    const overlume::mapbox::LonLatDeg c = overlume::mapbox::tile_point_to_lonlat(
        kGoldenZ14, 4096.0, 4096.0, overlume::mapbox::kLandmarkExtent);
    GoldenFrame g;
    g.anchor = overlume::GeoAnchor{c.lat, c.lon, kGoldenPi / 2.0, 0.0};
    // 8192 units per tile, 2^14 tiles around the equator (spherical Web Mercator).
    g.z14_per_m = overlume::mapbox::kLandmarkExtent * 16384.0 /
                  (2.0 * kGoldenPi * 6378137.0 * std::cos(c.lat * kGoldenPi / 180.0));
    return g;
}

// z14 units -> units of the z16 square `index` (its column or row inside the z14 tile), clamped
// to the 33-unit buffer Mapbox clips its tiles to.
int32_t golden_z16(double z14_units, int index) {
    return static_cast<int32_t>(
        std::lround(std::clamp(2.0 * z14_units - 4096.0 * index, -33.0, 4129.0)));
}

using GoldenRing = std::vector<std::array<int32_t, 2>>;

// The rectangle east e0..e1, north n0..n1 (metres from the tile centre) in z16 square (col, row),
// clockwise in tile space (y down): an MVT exterior ring.
GoldenRing golden_rect(const GoldenFrame& g, int col, int row, double e0, double n0, double e1,
                       double n1) {
    const int32_t x0 = golden_z16(4096.0 + e0 * g.z14_per_m, col);
    const int32_t x1 = golden_z16(4096.0 + e1 * g.z14_per_m, col);
    const int32_t y0 = golden_z16(4096.0 - n1 * g.z14_per_m, row);
    const int32_t y1 = golden_z16(4096.0 - n0 * g.z14_per_m, row);
    return {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
}

// The same rectangle wound the other way: an MVT interior ring.
GoldenRing golden_hole(const GoldenFrame& g, int col, int row, double e0, double n0, double e1,
                       double n1) {
    GoldenRing ring = golden_rect(g, col, row, e0, n0, e1, n1);
    std::reverse(ring.begin(), ring.end());
    return ring;
}

// The mean luminance (Rec. 709, as golden.cpp) of the 3x3 pixels around where the map point `p`
// lands in `frame`, a 320x240 render through the renderer's current camera. False off-screen.
bool golden_luma(overlume::VisualRenderer* r, const std::vector<uint8_t>& frame,
                 const overlume::Vec3& p, double* out) {
    float u = 0.0f, v = 0.0f;
    if (!overlume::project_to_screen(r, p, &u, &v)) return false;
    const int cx = std::clamp(static_cast<int>(u * 320.0f), 1, 318);
    const int cy = std::clamp(static_cast<int>(v * 240.0f), 1, 238);
    double sum = 0.0;
    for (int y = cy - 1; y <= cy + 1; ++y) {
        for (int x = cx - 1; x <= cx + 1; ++x) {
            const size_t i = (static_cast<size_t>(y) * 320u + static_cast<size_t>(x)) * 3u;
            sum += 0.2126 * frame[i] + 0.7152 * frame[i + 1] + 0.0722 * frame[i + 2];
        }
    }
    *out = sum / 9.0;
    return true;
}

}

// Present: a landmark tower, a courtyard block, a gate whose span floats 10 m up, and a block split
// across the z16 squares (1, 1) and (2, 1). Absent: the block under the tower's footprint
// (conflated) and an extrude=false outline. Everything stands on the ego's 120 m ground plane.
TEST(MapboxBuildingsGolden, SyntheticTileDarkAdas) {
    namespace mvt = overlume::testing::mvt;
    namespace glb = overlume::testing::glb;

    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const GoldenFrame g = golden_frame();
    glb::LandmarkSpec tower;
    tower.id = 1;
    tower.tx = 4096.0 - 30.0 * g.z14_per_m;
    tower.ty = 4096.0 - 185.0 * g.z14_per_m;
    tower.lat_deg = overlume::mapbox::tile_point_to_lonlat(kGoldenZ14, tower.tx, tower.ty,
                                                           overlume::mapbox::kLandmarkExtent)
                        .lat;
    tower.half_x_m = 8.0;
    tower.half_y_m = 8.0;
    tower.height_m = 64.0;
    tower.footprint_half_m = 10.0;

    // Features by z16 square, indexed row * 4 + col as footprint_tiles_of orders them.
    std::array<std::vector<mvt::Feature>, 16> squares{};
    squares[5].push_back(mvt::building(101,
                                       {golden_rect(g, 1, 1, -55.0, 110.0, -25.0, 140.0),
                                        golden_hole(g, 1, 1, -47.0, 118.0, -33.0, 132.0)},
                                       9.0));
    squares[5].push_back(
        mvt::building(102, {golden_rect(g, 1, 1, -12.0, 170.0, 12.0, 195.0)}, 20.0));
    squares[6].push_back(
        mvt::building(102, {golden_rect(g, 2, 1, -12.0, 170.0, 12.0, 195.0)}, 20.0));
    squares[5].push_back(
        mvt::building(103, {golden_rect(g, 1, 1, -45.0, 170.0, -15.0, 200.0)}, 30.0));
    squares[6].push_back(mvt::building(104, {golden_rect(g, 2, 1, 18.0, 120.0, 24.0, 126.0)}, 18.0,
                                       0.0, true, false, "building:part"));
    squares[6].push_back(mvt::building(105, {golden_rect(g, 2, 1, 46.0, 120.0, 52.0, 126.0)}, 18.0,
                                       0.0, true, false, "building:part"));
    squares[6].push_back(mvt::building(106, {golden_rect(g, 2, 1, 24.0, 120.0, 46.0, 126.0)}, 18.0,
                                       10.0, true, false, "building:part"));
    squares[6].push_back(
        mvt::building(107, {golden_rect(g, 2, 1, 25.0, 160.0, 50.0, 185.0)}, 15.0, 0.0, false));

    auto* net = overlume::testing::make_mapbox_fixture_network();
    std::set<std::string> served;
    auto serve = [&](const std::string& url, std::vector<uint8_t> body) {
        overlume::testing::mapbox_fixture_serve(net, url, 200, std::move(body));
        served.insert(url);
    };
    serve(overlume::testing::mapbox_landmark_url(kGoldenZ14.x, kGoldenZ14.y),
          glb::build_landmark_glb({tower}, glb::GlbOptions{}));
    const auto z16 = overlume::mapbox::footprint_tiles_of(kGoldenZ14);
    for (size_t i = 0; i < z16.size(); ++i) {
        serve(overlume::testing::mapbox_footprint_url(z16[i].x, z16[i].y),
              mvt::gzip_stored(mvt::encode_tile({mvt::Layer{"building", 4096, 2, squares[i]}})));
    }
    ASSERT_TRUE(overlume::testing::install_fixture_mapbox_source(r, "mapbox://buildings", g.anchor,
                                                                 net, "pk.test-token-not-real"));

    // 150 m north of the centre: more than 500 m from every edge of the z14 tile, so the area
    // view selects this tile alone.
    overlume::SceneGraph s{};
    s.ego.valid = 1;
    s.ego.position = overlume::Vec3{0.0, 150.0, 0.0};
    overlume::set_scene(r, s);

    const overlume::CameraPose pose{{0.0, 45.0, 80.0}, {0.0, 165.0, 5.0}, 60.0};
    std::vector<uint8_t> frame(320u * 240u * 3u);
    for (int i = 0; i < 1000 && overlume::testing::environment_loaded_chunk_count(r) == 0; ++i) {
        ASSERT_TRUE(overlume::render_frame(r, pose, {frame.data(), 320, 240}));
    }
    ASSERT_EQ(overlume::testing::environment_loaded_chunk_count(r), 1u)
        << "the golden z14 tile never reached the scene";
    for (int i = 0; i < 60; ++i) overlume::render_frame(r, pose, {frame.data(), 320, 240});

    const overlume::testing::MapboxSourceStats stats =
        overlume::testing::mapbox_last_source_stats();
    EXPECT_EQ(stats.tiles_built, 1u);
    EXPECT_EQ(stats.landmark_nodes, 1u) << "the tower";
    EXPECT_EQ(stats.extruded_features, 6u)
        << "courtyard, both halves of the split block, two pillars and the span";
    EXPECT_EQ(stats.conflated_features, 1u) << "the block under the tower's footprint";
    EXPECT_EQ(overlume::environment_source_state(r), overlume::EnvironmentSourceState::STREAMING);
    for (const std::string& url : overlume::testing::mapbox_fixture_requested_urls(net)) {
        const std::string bare = url.substr(0, url.find('?'));
        EXPECT_EQ(served.count(bare), 1u) << "unexpected request " << bare;
    }

    const double ssim = overlume::testing::render_and_compare(
        r, pose, OVERLUME_TEST_DATA_DIR "/tests/goldens/mapbox_buildings_synthetic_dark_adas.png",
        OVERLUME_TMP_DIR "/mapbox_buildings_synthetic_dark_adas_actual.png");
    EXPECT_GT(ssim, overlume::testing::kSsimMin);

    // The SSIM misses a span that stands on the ground and a courtyard without its hole (each
    // moves under 1 % of the pixels), so probes pin them: what shows under the span and through the
    // hole must read as ground, not clay. The references are open ground and the left pillar's
    // south face, which is what a span standing on the ground would show.
    ASSERT_TRUE(overlume::render_frame(r, pose, {frame.data(), 320, 240}));
    double ground = 0.0, clay = 0.0, underSpan = 0.0, courtyard = 0.0;
    ASSERT_TRUE(golden_luma(r, frame, {0.0, 100.0, 0.0}, &ground));
    ASSERT_TRUE(golden_luma(r, frame, {21.0, 120.0, 9.0}, &clay));
    ASSERT_TRUE(golden_luma(r, frame, {35.0, 123.0, 5.0}, &underSpan));
    ASSERT_TRUE(golden_luma(r, frame, {-40.0, 130.0, 0.0}, &courtyard));
    EXPECT_LT(std::abs(underSpan - ground), std::abs(underSpan - clay))
        << "clay under the gate's span: its min_height was not applied";
    EXPECT_LT(std::abs(courtyard - ground), std::abs(courtyard - clay))
        << "clay in the courtyard: its hole was not cut";
    overlume::destroy_renderer(r);
}
```

Checked while this plan was written: appended, with Task 16's block, to the file as lane 7's
Tasks 13 and 14 draft it, the result is unchanged by the pinned clang-format 23.1.1 with the repo's
`.clang-format`, and it passes `-fsyntax-only -Wall` as C++17 with the toolchain's clang++ against
the repo's headers and the contract's declarations, with no warning. The block-scope `mvt` and
`glb` aliases repeat Task 14's file-scope ones, which is legal and keeps the block self-contained.

- [ ] **Step 3: Build and run it: it fails on the missing golden and writes the candidate**

```bash
cd "$INTEGRATION"
cmake --build overlume/build -j8 --target test_mapbox_buildings
overlume/build/test_mapbox_buildings --gtest_filter='MapboxBuildingsGolden.*'
ls -l /tmp/mapbox_buildings_synthetic_dark_adas_actual.png
```

Expected, with the line number and time varying:

```
[ RUN      ] MapboxBuildingsGolden.SyntheticTileDarkAdas
…/overlume/tests/test_mapbox_buildings.cpp:…: Failure
Expected: (ssim) > (overlume::testing::kSsimMin), actual: 0 vs 0.98
[  FAILED  ] MapboxBuildingsGolden.SyntheticTileDarkAdas (… ms)
```

The SSIM line is the only failure, and `ls` shows a non-empty PNG. Other outcomes:
- `[  SKIPPED ]`: this machine has no GPU/EGL. Stop; a candidate needs the GPU rig.
- A stats or URL expectation fails as well: lane 7's loader deviates from the counting rules the
  contract pins in `mapbox_test_hooks.hpp` (the comment above `MapboxSourceStats`: per z16
  square, conflated features in `conflated_features` only, filtered ones in neither). Report it
  to the orchestrator as a lane 7 finding. Never change an expected count to absorb it.
- A probe expectation fails as well (`clay under the gate's span` or `clay in the courtyard`):
  the loader dropped `min_height` or the hole, a lane 7 finding (Tasks 1, 3 and 13 carry them).
  Report it. Never move a probe point to make it pass.

- [ ] **Step 4: Check the candidate is deterministic**

```bash
cd "$INTEGRATION"
cp /tmp/mapbox_buildings_synthetic_dark_adas_actual.png /tmp/mapbox_golden_run1.png
overlume/build/test_mapbox_buildings --gtest_filter='MapboxBuildingsGolden.*' > /dev/null 2>&1
cmp /tmp/mapbox_golden_run1.png /tmp/mapbox_buildings_synthetic_dark_adas_actual.png && echo identical
```

Expected: `identical`. If the files differ, stop and find the nondeterminism. A flaky candidate is
never shown or promoted.

- [ ] **Step 5: HUMAN GATE (orchestrator). Show the candidate to the user; promote nothing without
  explicit approval**

The orchestrator shows the frozen copy Step 4 proved deterministic, never the `_actual.png` path:
any later run of the test, a gate in `$INTEGRATION` included, rewrites that one. First it records
the copy's hash:

```bash
sha256sum /tmp/mapbox_golden_run1.png | tee /tmp/mapbox_golden_run1.sha256
```

Expected: one line, `<64 hex digits>  /tmp/mapbox_golden_run1.png`. It then views
`/tmp/mapbox_golden_run1.png` and puts it in front of the user (send them the file, or open it on
their display with `python3 overlume/tests/golden.py --show /tmp/mapbox_golden_run1.png`), with
this checklist:

1. A tall tower stands at the back left: the landmark, 64 m. Its base is a slim 16 m square, and no
   wider, 30 m tall block wraps its lower half. That block would be the extrusion conflation
   removes.
2. At the front left, a low 9 m block has an open square courtyard in its roof.
3. At the front right, a gate: two pillars joined by a span whose underside is clearly above the
   ground, with ground visible beneath it.
4. At the back centre, one 20 m block reads as a single building, with no line, step or gap down
   its middle where two z16 squares meet.
5. At the back right, an empty patch of ground: an `extrude=false` outline must not be extruded.
6. Everything is in the theme's clay, stands on the ground plane and has complete faces, with no
   speckle from z-fighting. No ego is drawn: the test never loads an ego model.

Outcomes:
- The user approves: go to Step 6, with the hash recorded above.
- The user asks for changes: adjust the scene or the camera in Step 2's block, rebuild, and return
  to Step 3. Steps 4 and 5 then freeze and hash the new candidate. Never copy a candidate the user
  has not approved, and never overwrite a failing golden to make it pass.

- [ ] **Step 6: Promote the approved candidate**

```bash
cd "$INTEGRATION"
sha256sum -c /tmp/mapbox_golden_run1.sha256 \
    && cp /tmp/mapbox_golden_run1.png overlume/tests/goldens/mapbox_buildings_synthetic_dark_adas.png
git add overlume/tests/goldens/mapbox_buildings_synthetic_dark_adas.png
git show ":overlume/tests/goldens/mapbox_buildings_synthetic_dark_adas.png" | head -1
```

Expected: `/tmp/mapbox_golden_run1.png: OK`, then `version https://git-lfs.github.com/spec/v1`.
The staged blob is an LFS pointer, not the PNG bytes. `FAILED` from `sha256sum` means the bytes
are not the ones the user approved: nothing is copied, `git add` fails on the missing file, and
the candidate goes back to Step 5.

- [ ] **Step 7: Re-run; it passes**

```bash
cd "$INTEGRATION"
overlume/build/test_mapbox_buildings --gtest_filter='MapboxBuildingsGolden.*' | tail -1
```

Expected: `[  PASSED  ] 1 test.` The SSIM is 1.0, because the golden is the candidate itself.

- [ ] **Step 8: Prove the expectations bite, then restore**

This test catches these reverts:
- Task 14's conflation reverted: the block under the tower is extruded. `conflated_features` drops
  to 0, `extruded_features` rises to 7, and a 30 m block wraps the tower.
- Standard's 3D filter reverted (Task 1's `passes_standard_3d_filter`, applied in Task 13): the
  outline is extruded and `extruded_features` rises to 7. The first mutation below shows it.
- A broken landmark path (meshopt decode, node placement; Task 5): `landmark_nodes` drops to 0 and
  the tower is gone.
- `min_height` lost on the way to the extruder (Tasks 1, 3 and 13): the span stands on the
  ground, and the probe under it reads clay. The second mutation shows it.
- Holes lost (Task 1's ring classification, Task 3's roof, Task 13's polygon conversion): the
  courtyard's roof closes, and the courtyard probe reads clay. The third mutation shows it.
- Changes to placement, the clay material, the theme or the merged content that move many pixels:
  the SSIM.

The SSIM alone would miss every mutation here: on the stand-in it stays at 0.993 for the extruded
outline, 0.999 for the span and 0.997 for the courtyard. Seams stay with Task 3's unit tests and
Task 13's `CornerLoadsFourTilesWithoutDoubledSeamWalls` (wall counts): a doubled seam wall lies
inside the building and shows in no frame.

Each mutation edits a fresh copy of the test's scene, as the loader regression would leave the
tile, then builds, runs and prints the failures; the last lines restore the file.

```bash
cd "$INTEGRATION"
cp overlume/tests/test_mapbox_buildings.cpp /tmp/test_mapbox_buildings.cpp.keep
run_mutant() {
    cp /tmp/test_mapbox_buildings.cpp.keep overlume/tests/test_mapbox_buildings.cpp
    sed -i "$@" overlume/tests/test_mapbox_buildings.cpp
    if cmp -s /tmp/test_mapbox_buildings.cpp.keep overlume/tests/test_mapbox_buildings.cpp; then
        echo "mutation did not apply"
        return 1
    fi
    cmake --build overlume/build -j8 --target test_mapbox_buildings > /dev/null \
        && overlume/build/test_mapbox_buildings --gtest_filter='MapboxBuildingsGolden.*' 2>&1 \
        | grep -E -A4 '^Expected equality|^Expected: \(std::abs'
}
echo "== outline extruded"
run_mutant 's/185\.0)}, 15\.0, 0\.0, false));/185.0)}, 15.0, 0.0, true));/'
echo "== span on the ground"
run_mutant 's/ 10\.0, true, false, "building:part"/ 0.0, true, false, "building:part"/'
echo "== courtyard without its hole"
run_mutant -e '/golden_hole(g, 1, 1, -47\.0, 118\.0, -33\.0, 132\.0)},$/d' \
    -e 's/\(golden_rect(g, 1, 1, -55\.0, 110\.0, -25\.0, 140\.0)\),$/\1},/'
cp /tmp/test_mapbox_buildings.cpp.keep overlume/tests/test_mapbox_buildings.cpp
cmake --build overlume/build -j8 --target test_mapbox_buildings > /dev/null
overlume/build/test_mapbox_buildings --gtest_filter='MapboxBuildingsGolden.*' | tail -1
```

Expected, each mutation's lines after its `==` header (the three sed expressions were checked
against Tasks 13–15's code as drafted: each changes exactly its lines, and each mutant compiles):

```
== outline extruded
Expected equality of these values:
  stats.extruded_features
    Which is: 7
  6u
    Which is: 6
== span on the ground
Expected: (std::abs(underSpan - ground)) < (std::abs(underSpan - clay)), actual: … vs …
clay under the gate's span: its min_height was not applied
…
== courtyard without its hole
Expected: (std::abs(courtyard - ground)) < (std::abs(courtyard - clay)), actual: … vs …
clay in the courtyard: its hole was not cut
…
[  PASSED  ] 1 test.
```

The `…` lines are gtest's `[  FAILED  ]` lines and run summary. On the stand-in the two `actual:`
pairs were about 6.7 vs 0.6 and 15.6 vs 8.4; the first number must be the larger. A mutation that
prints only its `==` header did not fail: stop and report it. `mutation did not apply` means the
block's text drifted from Step 2: fix the sed expression, never the test. The final line is the
restored test passing.

- [ ] **Step 9: Record the promotion in `docs/status.md`**

This appends one row to the end of the `## Shipped` table, dated the day the user approved.

```bash
cd "$INTEGRATION"
python3 - "$(date +%F)" <<'EOF'
import sys

path = "docs/status.md"
row = ("| Mapbox buildings golden `mapbox_buildings_synthetic_dark_adas` (a synthetic z14 tile "
       "served in Mapbox's formats, a meshopt landmark GLB and 16 gzip MVTs: a landmark tower, a "
       "courtyard block, a gate whose span floats 10 m up and a block split across two z16 "
       "squares; the block under the tower's footprint and an `extrude=false` outline stay "
       "absent) promoted by the user after viewing the candidate | " + sys.argv[1] +
       " | [`plans/2026-10-14-mapbox-buildings.md`](plans/2026-10-14-mapbox-buildings.md) |\n")
text = open(path).read()
anchor = "\n\n## Open items"
assert text.count(anchor) == 1, "docs/status.md: expected one '## Open items' heading"
cut = text.index(anchor) + 1
open(path, "w").write(text[:cut] + row + text[cut:])
print(row, end="")
EOF
git diff --stat docs/status.md
```

Expected: the row is printed, then ` docs/status.md | 1 +`.

- [ ] **Step 10: Format**

```bash
cd "$INTEGRATION"
git add overlume/tests/test_mapbox_buildings.cpp \
        overlume/tests/goldens/mapbox_buildings_synthetic_dark_adas.png docs/status.md
tools/check_format.sh --fix
git diff --stat
```

Expected: `check_format.sh: reformatted … files with clang-format 23.1.1`, then no `git diff`
output, because the block was written formatted. If a file is listed, `git add` it.

- [ ] **Step 11: Gate** (foreground, Bash timeout 600000 ms, holding the machine-wide gate lock as
  every gate in this plan does)

```bash
cd "$INTEGRATION"
cmake --build overlume/build -j8 \
    && flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock tools/ci_visual_mode.sh
```

Expected: six `PASS` lines and exit 0. `PASS  golden suite: N ok, 0 skipped`, where N is one more
than at the previous gate run in `$INTEGRATION`, and every existing golden still passes, so none
moved. The per-pixel drift audit does not apply: this golden is new, with no earlier baseline.
Exit 75 means another gate held the lock for 300 s: run the same command again. Any other
non-zero exit is a gate failure.

- [ ] **Step 12: Commit the test, the golden and the status row together**

```bash
cd "$INTEGRATION"
git add overlume/tests/test_mapbox_buildings.cpp \
        overlume/tests/goldens/mapbox_buildings_synthetic_dark_adas.png docs/status.md
git commit -F - <<'EOF'
test(mapbox): synthetic-tile golden for mapbox://buildings, promoted by the user

MapboxBuildingsGolden.SyntheticTileDarkAdas renders one z14 tile served
through the fixture network in Mapbox's formats (a meshopt landmark GLB and
16 gzip MVTs) at 320x240 in dark_adas clay. Present: a landmark tower, a
courtyard block, a gate whose span floats 10 m up, and a block split across
two z16 squares. Absent: the block under the tower's footprint (conflated)
and an extrude=false outline. A whole-frame SSIM need not notice one block
more or less, so the test also pins the source's counts and the requested
URLs, and probes four pixels: what shows under the span and through the
courtyard's hole must read as ground, not clay, so a lost min_height or a
lost hole fails. The user approved the candidate before its hashed copy was
promoted to overlume/tests/goldens/mapbox_buildings_synthetic_dark_adas.png
(LFS); docs/status.md records the promotion. No existing golden moved.

Claude-Session: <the executing session's URL>
EOF
git lfs ls-files | grep mapbox_buildings_synthetic_dark_adas
```

Expected: the commit succeeds, and `git lfs ls-files` lists the golden.

---

### Task 16: Opt-in live smoke

**Lane:** 8. **Depends on:** Task 15 committed (same file, same worktree). Task 11's
`tools/check_mapbox_token.sh` is already in the branch: lane 8 starts only after every other lane
has merged. Runs in `$INTEGRATION`, under Task 15's rule: no merge and no other commit there while
this task has changes. The implementer runs Steps 1–6 and 9; the orchestrator runs Steps 7, 8, 10
and 11 with the user, because they reach the network with the real token.

**Files:**
- Modify: `overlume/tests/test_mapbox_buildings.cpp` (the include block; one block appended after
  Task 15's test)
- Modify: `docs/plans/2026-10-14-mapbox-buildings.md` (Status ledger row 16, orchestrator)
- Test: `overlume/tests/test_mapbox_buildings.cpp`, `MapboxLive.DowntownDubai` and
  `MapboxLive.TestSite`

**Interfaces:**
- Consumes: `bool set_environment_source(VisualRenderer*, const char* source_uri, GeoAnchor anchor);`
  (`scene.h:266`) with Task 13's `mapbox://` dispatch, which reads `MAPBOX_TOKEN` itself;
  `MapboxSourceStats mapbox_last_source_stats();` (contract, Task 13);
  `environment_loaded_chunk_count` and `render_and_compare` as in Task 15;
  `tools/check_mapbox_token.sh` (Task 11), which prints exactly one line on stdout, as the Global
  Constraints pin:
  `PASS: token retrieves the Downtown Dubai tiles (landmark tile HTTP 200, footprint tile HTTP 200)`
  and exit 0 when both tiles answer 200;
  `FAIL: <landmark>, <footprint> (see docs/runbooks/mapbox.md)` and exit 1 otherwise;
  `SKIP: MAPBOX_TOKEN is not set (see docs/runbooks/mapbox.md)` and exit 2, with no request, when
  the token, trimmed as the library trims it, is empty.
- Produces: `TEST(MapboxLive, DowntownDubai)`, `TEST(MapboxLive, TestSite)`, and the live counts
  in the plan's ledger.

**API notes:**
- The opt-in pattern copies `EnvironmentStreamPerf.GooglePresetLiveRenderMsDeltaVsOsmClay`
  (`overlume/tests/test_environment_stream.cpp:1083-1144`): skip without the token and an explicit
  opt-in variable, a 30 s pump, and `render_and_compare` against `/nonexistent_no_golden.png`
  only to write a PNG.
- Downtown: the Burj Khalifa (25.197139 N, 55.274111 E) lies in z14 tile 14/10707/7006, 868 m from
  its nearest (north) edge, so one tile loads. The 2026-10-14 probe found 7 landmarks there (spec
  §3). Test site: (25.1972 N, 55.2744 E) lies 208 m from the east edge of 14/10707/7006 and
  589 m from its north edge, so its east neighbour loads too: two tiles.
- Volume: 17 requests per z14 tile (one landmark GLB, 16 MVTs), so the green run makes 51.
- libcurl honours `https_proxy` (`docs/runbooks/replay_logger_session.md` §6). Pointed at a closed
  local port, every request fails in transport before leaving the machine; with Task 6 those
  failures count, and eight in a row trip `STREAMING_FALLBACK` (`environment_stream.hpp:50`,
  `environment_stream.cpp:769-774`). Step 9 uses that for an offline red run.
- gtest's skip summary is `[  SKIPPED ] 2 tests, listed below:` followed by one line per test
  (googletest `src/gtest.cc:3654-3655`); `EnvironmentSourceState` prints as `1-byte object <02>`
  (STREAMING) and `<03>` (STREAMING_FALLBACK), as in Task 15's API notes.
- A login shell's startup files can print to stdout, so Steps 8 and 10 read the token back on
  fd 3, as Task 11 does in `tools/validate_visual_mode.sh`.

- [ ] **Step 1: Add the three standard headers this block uses**

Add any of these missing to the standard-header group at the top of
`overlume/tests/test_mapbox_buildings.cpp`, in alphabetical order:

```cpp
#include <chrono>
#include <cstdlib>
#include <iostream>
```

Check:

```bash
cd "$INTEGRATION"
for h in '<chrono>' '<cstdlib>' '<iostream>'; do
    grep -qxF "#include $h" overlume/tests/test_mapbox_buildings.cpp || echo "missing $h"
done
```

Expected: no output.

- [ ] **Step 2: Append the live tests to the end of `overlume/tests/test_mapbox_buildings.cpp`**

```cpp
namespace {

constexpr double kLivePi = 3.14159265358979323846;

bool mapbox_live_enabled() {
    const char* token = std::getenv("MAPBOX_TOKEN");
    const char* live = std::getenv("OVERLUME_LIVE_MAPBOX");
    return token != nullptr && *token != '\0' && live != nullptr && std::string(live) == "1";
}

// Renders until the ego's tiles are in the scene and no tile has landed for 3 s, or 30 s pass,
// then writes the frame to OVERLUME_TMP_DIR and prints counts. Never a URL: every request URL
// carries the token.
overlume::testing::MapboxSourceStats pump_mapbox_live(overlume::VisualRenderer* r, const char* site,
                                                      const overlume::CameraPose& pose) {
    std::vector<uint8_t> buf(320u * 240u * 3u);
    const auto start = std::chrono::steady_clock::now();
    auto last_change = start;
    uint64_t built = 0;
    while (std::chrono::steady_clock::now() - start < std::chrono::seconds(30)) {
        if (!overlume::render_frame(r, pose, {buf.data(), 320, 240})) break;
        const uint64_t now_built = overlume::testing::mapbox_last_source_stats().tiles_built;
        if (now_built != built) {
            built = now_built;
            last_change = std::chrono::steady_clock::now();
        }
        if (built > 0 && overlume::testing::environment_loaded_chunk_count(r) > 0 &&
            std::chrono::steady_clock::now() - last_change > std::chrono::seconds(3)) {
            break;
        }
    }
    const double wait_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    const overlume::testing::MapboxSourceStats stats =
        overlume::testing::mapbox_last_source_stats();
    const std::string png = std::string(OVERLUME_TMP_DIR) + "/mapbox_live_" + site + ".png";
    overlume::testing::render_and_compare(r, pose, "/nonexistent_no_golden.png", png.c_str());
    std::cerr << "[MapboxLive] " << site << " tiles_built=" << stats.tiles_built
              << " landmark_nodes=" << stats.landmark_nodes
              << " extruded_features=" << stats.extruded_features
              << " conflated_features=" << stats.conflated_features << " retries=" << stats.retries
              << " max_build_ms=" << stats.max_build_ms
              << " in_scene=" << overlume::testing::environment_loaded_chunk_count(r)
              << " wait_s=" << wait_s << " png=" << png << "\n";
    return stats;
}

}

TEST(MapboxLive, DowntownDubai) {
    if (!mapbox_live_enabled()) {
        GTEST_SKIP() << "opt-in live-network check -- set MAPBOX_TOKEN and "
                        "OVERLUME_LIVE_MAPBOX=1 to run (never required by ctest)";
    }
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    // The Burj Khalifa; heading pi/2 puts map +x east, so the camera looks north at the tower.
    const overlume::GeoAnchor anchor{25.197139, 55.274111, kLivePi / 2.0, 0.0};
    ASSERT_TRUE(overlume::set_environment_source(r, "mapbox://buildings", anchor));
    overlume::SceneGraph s{};
    s.ego.valid = 1;
    overlume::set_scene(r, s);

    const overlume::CameraPose pose{{0.0, -380.0, 220.0}, {0.0, 0.0, 120.0}, 60.0};
    const overlume::testing::MapboxSourceStats stats = pump_mapbox_live(r, "downtown", pose);
    EXPECT_EQ(overlume::environment_source_state(r), overlume::EnvironmentSourceState::STREAMING)
        << "fell back to baked chunks: run tools/check_mapbox_token.sh";
    EXPECT_GE(stats.landmark_nodes, 1u)
        << "no landmark kept at Downtown: a codec change or a rejected node matrix";
    overlume::destroy_renderer(r);
}

TEST(MapboxLive, TestSite) {
    if (!mapbox_live_enabled()) {
        GTEST_SKIP() << "opt-in live-network check -- set MAPBOX_TOKEN and "
                        "OVERLUME_LIVE_MAPBOX=1 to run (never required by ctest)";
    }
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    auto* r = overlume::create_renderer(cfg);
    if (!r) GTEST_SKIP() << "no GPU/EGL";
    const overlume::GeoAnchor anchor{25.1972, 55.2744, kLivePi / 2.0, 0.0};
    ASSERT_TRUE(overlume::set_environment_source(r, "mapbox://buildings", anchor));
    overlume::SceneGraph s{};
    s.ego.valid = 1;
    overlume::set_scene(r, s);

    const overlume::CameraPose pose{{0.0, -300.0, 200.0}, {0.0, 0.0, 0.0}, 60.0};
    const overlume::testing::MapboxSourceStats stats = pump_mapbox_live(r, "operating_area", pose);
    EXPECT_EQ(overlume::environment_source_state(r), overlume::EnvironmentSourceState::STREAMING)
        << "fell back to baked chunks: run tools/check_mapbox_token.sh";
    EXPECT_GT(stats.extruded_features, 0u) << "no footprint extruded at the test site";
    overlume::destroy_renderer(r);
}
```

These tests catch the faults that only real Mapbox data shows. A meshopt codec change, or a
reverted landmark path, drops Downtown's `landmark_nodes` to 0, as spec §12 predicts. A broken
MVT path drops the test site's `extruded_features` to 0. A source that never streams (a
rejected token, no network) ends in `STREAMING_FALLBACK`. Step 9 shows the failures.

- [ ] **Step 3: Build; without the opt-in both tests skip**

```bash
cd "$INTEGRATION"
cmake --build overlume/build -j8 --target test_mapbox_buildings
env -u OVERLUME_LIVE_MAPBOX overlume/build/test_mapbox_buildings --gtest_filter='MapboxLive.*' \
    | tail -3
env -u MAPBOX_TOKEN OVERLUME_LIVE_MAPBOX=1 \
    overlume/build/test_mapbox_buildings --gtest_filter='MapboxLive.*' | tail -3
```

Expected, twice:

```
[  SKIPPED ] 2 tests, listed below:
[  SKIPPED ] MapboxLive.DowntownDubai
[  SKIPPED ] MapboxLive.TestSite
```

- [ ] **Step 4: Format**

```bash
cd "$INTEGRATION"
git add overlume/tests/test_mapbox_buildings.cpp
tools/check_format.sh --fix
git diff --stat
```

Expected: `check_format.sh: reformatted … files with clang-format 23.1.1`, then no `git diff`
output.

- [ ] **Step 5: Gate** (foreground, Bash timeout 600000 ms, under the gate lock; no network:
  `OVERLUME_LIVE_MAPBOX` is unset in the gate, even where the login shell exports `MAPBOX_TOKEN`)

```bash
cd "$INTEGRATION"
cmake --build overlume/build -j8 \
    && flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock tools/ci_visual_mode.sh
```

Expected: six `PASS` lines and exit 0; the golden count is unchanged from Task 15, and the library
ctest suite shows two more skipped entries. Exit 75 means another gate held the lock for 300 s:
run the same command again. Any other non-zero exit is a gate failure.

- [ ] **Step 6: Commit**

```bash
cd "$INTEGRATION"
git add overlume/tests/test_mapbox_buildings.cpp
git commit -F - <<'EOF'
test(mapbox): opt-in live smoke at Downtown Dubai and the test site

MapboxLive.DowntownDubai and MapboxLive.TestSite open
mapbox://buildings through set_environment_source against the real
endpoints, pump until the ego's tiles land (30 s cap), write a PNG to
OVERLUME_TMP_DIR and print tile, landmark and extrusion counts and the
slowest build time, never a URL (every request URL carries the token). They
assert a landmark at Downtown, which a meshopt codec change would zero
(spec section 12), and extrusions at the test site. Both skip unless
MAPBOX_TOKEN is set and OVERLUME_LIVE_MAPBOX=1, so the gate stays network-
and token-free.

Claude-Session: <the executing session's URL>
EOF
```

- [ ] **Step 7: HUMAN (orchestrator). Ask the user's consent for the live run**

Tell the user what the run does: it reaches `api.mapbox.com` with `MAPBOX_TOKEN`, two requests for
the token check and 51 for the tests (17 per z14 tile: one tile at Downtown, two at the operating
area). It takes about a minute, prints counts only, and writes two PNGs of Mapbox content to
`/tmp`, which never enter the repository. Continue only on an explicit yes.

- [ ] **Step 8: Token preflight (orchestrator)**

Run the block as one command; shell state does not carry over between commands. Never print the
token.

```bash
cd "$INTEGRATION"
MAPBOX_TOKEN="$(env -u MAPBOX_TOKEN bash -lic 'printf %s "${MAPBOX_TOKEN:-}" >&3' \
    3>&1 >/dev/null 2>&1)"
export MAPBOX_TOKEN
[[ -n "$MAPBOX_TOKEN" ]] && echo "MAPBOX_TOKEN set in the login shell" || echo "MAPBOX_TOKEN missing"
tools/check_mapbox_token.sh; echo "check exit=$?"
```

Expected:

```
MAPBOX_TOKEN set in the login shell
PASS: token retrieves the Downtown Dubai tiles (landmark tile HTTP 200, footprint tile HTTP 200)
check exit=0
```

On `FAIL:` (exit 1) or `SKIP:` (exit 2), stop and follow `docs/runbooks/mapbox.md` §2.

- [ ] **Step 9: Red run, offline: a source that cannot stream fails both tests**

The bogus token and a closed local proxy port keep every request on the machine.

```bash
cd "$INTEGRATION"
env -u no_proxy -u NO_PROXY MAPBOX_TOKEN=pk.test-token-not-real OVERLUME_LIVE_MAPBOX=1 \
    https_proxy=http://127.0.0.1:9 \
    overlume/build/test_mapbox_buildings --gtest_filter='MapboxLive.*' \
    > /tmp/overlume_mapbox_live_red.log 2>&1; echo "exit=$?"
grep -E '^\[MapboxLive\]|Which is: 1-byte|actual: 0 vs|^\[  FAILED  \] MapboxLive' \
    /tmp/overlume_mapbox_live_red.log
grep -c -F 'pk.test-token-not-real' /tmp/overlume_mapbox_live_red.log
```

Expected after about a minute (each test pumps its full 30 s): `exit=1`. Both `[MapboxLive]` lines
show `tiles_built=0 landmark_nodes=0 extruded_features=0`. Each test reports the state mismatch,
`Which is: 1-byte object <03>` against `Which is: 1-byte object <02>`. Then come
`Expected: (stats.landmark_nodes) >= (1u), actual: 0 vs 1` and
`Expected: (stats.extruded_features) > (0u), actual: 0 vs 0`, the two `[  FAILED  ] MapboxLive.…`
result lines and the two summary lines. The last line is `0`: no log line carries the token, not
even the bogus one. The run also shows Task 6 end to end: without its fix the transport errors
never reach the counter, the state stays `STREAMING`, and only the count expectations fail.

- [ ] **Step 10: Green run with the real token (orchestrator)**

```bash
cd "$INTEGRATION"
MAPBOX_TOKEN="$(env -u MAPBOX_TOKEN bash -lic 'printf %s "${MAPBOX_TOKEN:-}" >&3' \
    3>&1 >/dev/null 2>&1)"
export MAPBOX_TOKEN
OVERLUME_LIVE_MAPBOX=1 overlume/build/test_mapbox_buildings --gtest_filter='MapboxLive.*' \
    > /tmp/overlume_mapbox_live.log 2>&1; echo "exit=$?"
grep -E '^\[MapboxLive\]|^\[  (PASSED|FAILED) +\]' /tmp/overlume_mapbox_live.log
grep -c -F -f <(printf '%s' "$MAPBOX_TOKEN" | tr -d '[:space:]') \
    /tmp/overlume_mapbox_live.log /tmp/overlume_mapbox_live_red.log
```

Expected:
- `exit=0`.
- A `downtown` line with `tiles_built=1` and `landmark_nodes` of at least 1 (the probe saw 7
  there).
- An `operating_area` line with `landmark_nodes=0` (Mapbox has none there) and
  `extruded_features` above 0, with `tiles_built=2`.
- `[  PASSED  ] 2 tests.` A `[  PASSED  ] 0 tests.` means both skipped: the token was empty.
- `/tmp/overlume_mapbox_live.log:0` and `/tmp/overlume_mapbox_live_red.log:0`. This is
  `grep -c -F` with the token as the pattern, trimmed of whitespace as the library trims it, but
  the pattern goes in through a file descriptor, so the token never appears in a command line.

The orchestrator shows the user `/tmp/mapbox_live_downtown.png` and
`/tmp/mapbox_live_operating_area.png`. They hold rendered Mapbox content: local only, never
committed.

- [ ] **Step 11: Record the counts in the plan's ledger (orchestrator), never a token**

```bash
cd "$INTEGRATION"
python3 - "$(date +%F)" "$(git log -1 --format=%h -- overlume/tests/test_mapbox_buildings.cpp)" \
    /tmp/overlume_mapbox_live.log <<'EOF'
import sys

date, commit, log = sys.argv[1], sys.argv[2], sys.argv[3]
runs = {}
for line in open(log):
    if line.startswith("[MapboxLive] "):
        site, *pairs = line.split()[1:]
        runs[site] = dict(pair.split("=", 1) for pair in pairs)


def counts(site):
    r = runs[site]
    return (f"tiles {r['tiles_built']}, landmarks {r['landmark_nodes']}, extruded "
            f"{r['extruded_features']}, conflated {r['conflated_features']}, slowest build "
            f"{float(r['max_build_ms']):.0f} ms")


prefix = "| 16. Opt-in live smoke | 8 |"
row = (f"{prefix} done; live run {date} with the user's consent: Downtown {counts('downtown')}; "
       f"test site {counts('operating_area')}; `tools/check_mapbox_token.sh` PASS | "
       f"`{commit}` |")
path = "docs/plans/2026-10-14-mapbox-buildings.md"
lines = open(path).read().split("\n")
hits = [i for i, line in enumerate(lines) if line.startswith(prefix)]
assert len(hits) == 1, f"{path}: {len(hits)} ledger rows for Task 16"
lines[hits[0]] = row
open(path, "w").write("\n".join(lines))
print(row)
EOF
git add docs/plans/2026-10-14-mapbox-buildings.md
git commit -F - <<'EOF'
docs(plan): record the Mapbox live smoke counts

Task 16's live run, made with the user's consent: tiles, landmarks,
extrusions, conflations and the slowest tile build per site, from the
[MapboxLive] lines, and tools/check_mapbox_token.sh PASS. Counts only:
no token, no URL.

Claude-Session: <the executing session's URL>
EOF
```

Expected: the new ledger row is printed, then the commit succeeds.

---

### Task 17: Rig rehearsal at both sites and close-out

**Lane:** 8. **Depends on:** every task merged into `mapbox-environment`, Tasks 15 and 16
committed, and the whole-branch review closed (Execution, Merges). Runs in `$INTEGRATION`. The
orchestrator runs it, because the user judges the visuals. `$SESSION_DIR` is the logger session
to replay: the one `tools/validate_logger_session.sh` uses by default (`$OVERLUME_SESSION`, else
`~/session_<stamp>`, `tools/validate_logger_session.sh:8`), unless the user names
another. Every command block below is one command: shell state does not carry over between them.
Never print a token, never use `set -x` with tokens in the environment, and push nothing.

**Files:**
- Modify: `docs/runbooks/mapbox.md` (the three §7.2 value rows Task 12 writes, as the Interface
  contract's "Texts other tasks match verbatim" pins them; any sentence Step 18 finds drifted)
- Modify: `docs/status.md` (one `## Shipped` row; an update appended to open item 18)
- Modify: `docs/plans/2026-10-14-mapbox-buildings.md` (ledger row 17; a `### Live acceptance
  results` subsection)
- Modify: `docs/README.md` (the plan's entry, which the plan's commit adds, marked closed; Global
  Constraints)
- Outside the repository: `/tmp/overlume_mapbox_rehearsal/` (helpers, rig output, `results.txt`),
  `/tmp/overlume_validate/` (the rig's own logs)

**Interfaces:**
- Consumes:
  - `tools/validate_logger_session.sh [SESSION_DIR] [--arm ENTRY] [--anchor-height-offset M] [-- <validate_visual_mode.sh args>]`,
    where ENTRY names one of the GUI's environment sources (`tools/validate_logger_session.sh:16-29`)
  - the GUI and bridge entries `mapbox` → `mapbox://buildings` and `mapbox-original` →
    `mapbox://buildings?materials=original`, and `osm` → `ion://96188`
    (`tools/vcam_ws_bridge.py:87-92` plus Task 11)
  - `tools/check_mapbox_token.sh` and the rig's `MAPBOX_TOKEN` refresh (Task 11)
  - the attribution overlay (Task 10)
  - node parameters `geo_datum_lat_deg`, `geo_datum_lon_deg`, `geo_datum_heading_deg`,
    `environment_source_uri`, `environment_chunks_dir`
  - `overlume/tools/sample_diagnostics.py <seconds>`, `tools/kill_switch_proxy.py <port>`
  - the runbook's §7.2 rows, the ledger rows and the node log texts, as the Interface contract's
    "Texts other tasks match verbatim" pins them
- Produces: the rehearsal record (runbook values, status rows, the plan's Live acceptance
  results) and a closed plan.

**API notes:**
- Rig: `kill_prior_rig` kills any running node, bridge, GUI or bag player on the machine, whichever
  checkout started them (`tools/validate_visual_mode.sh:72-110`), but not `the logger's player`: the replay
  wrapper refuses to start while `/clock` has a publisher (`tools/validate_logger_session.sh:48-51`).
  Port 8765 must be free (`validate_visual_mode.sh:116`). `CESIUM_ION_TOKEN` is refreshed from a
  login shell (`:177-184`), and Task 11 does the same for `MAPBOX_TOKEN`, reading both back on
  fd 3. The node runs with `--param` overrides and logs to `/tmp/overlume_validate/overlume_node.log`
  (`:32`, `:69`, `:253-256`). The GUI starts only when `DISPLAY` is set (`:309-317`), and the rig
  prints `Rig is up` (`:403`). Its teardown runs on INT or TERM (`:196-216`).
- Replay wrapper: `--` forwards the rest to `validate_visual_mode.sh` (`:24`, `:139-141`). `--arm`
  waits for `Rig is up`, then retries `set_environment_source` over the bridge for 120 s and prints
  `[arm] OK: environment source = …` (`:144-170`). Its teardown runs on INT or TERM (`:94-109`).
  The −1.8 m antenna offset comes from `docs/runbooks/replay_logger_session.md:129-136`.
- Bridge: `{"cmd": "set_environment_source", "preset": "<entry>"}` is acknowledged with
  `{"type": "ack", "cmd": "set_environment_source", "success": …, "reason": …}`
  (`tools/vcam_ws_bridge.py:33`, `:53`), which is what the GUI's selector sends
  (`tools/vcam_gui.py:471-478`).
- Node (line numbers before Task 10's edits):
  - A geo-datum override needs latitude, longitude and heading, or it is ignored. When complete it
    logs `geo-anchor solved (geo_datum override): --anchor-lat %.8f --anchor-lon %.8f --anchor-heading-deg %.4f --anchor-height %.2f`
    (`ros/src/overlume_ros/src/overlume_node.cpp:252-275`).
  - The GPS solve logs `geo-anchor solved: --anchor-lat …` with the same fields (`:276-294`).
  - A solved anchor arms the source at activate (`:757-781`). A live switch logs
    `environment source armed: '<composed uri>' (visible)` (`:869-903`). Console lines carry
    wall-clock timestamps in the default `[SEVERITY] [time] [name]:` format.
  - The one-shot fallback warning reads `environment source: network loss detected -- switched to fallback dir '<dir>'`
    (`:1262-1281`); a live switch re-arms it (`:892`).
  - `render_ms` times `render_frame` on the 33 ms timer and is published every frame
    (`:784`, `:1251-1260`, `:1383`).
- Map frame: `MapToWgs` gives east = x·sin h − y·cos h and north = x·cos h + y·sin h, then degrees
  on a 6371 km sphere (`ros/src/overlume_ros/src/geo_anchor.cpp:15`, `:30-41`). Heading is the
  compass bearing of map +x (`overlume/src/environment_stream.cpp:417-423`).
- `render_ms`: `/overlume_node/diagnostics` carries a status named `render_ms` with the key
  `render_ms` (`ros/src/overlume_ros/src/diagnostics.cpp:53-58`).
  `overlume/tools/sample_diagnostics.py <seconds>` prints
  `render_ms_p50=… render_ms_p99=… render_ms_n=…` (`:15-48`).
- Network cut: `tools/kill_switch_proxy.py` is a CONNECT proxy that logs no URL, and killing it
  refuses every new connection (`:5-13`, `:71`; `docs/runbooks/replay_logger_session.md` §6).
  With Task 6, curl's transport errors count as failures, and at 8 the stream falls back at the
  next update (`environment_stream.hpp:50`, `environment_stream.cpp:769-774`).
- Session map frame: the route's map → base_link poses are in `$SESSION_DIR/transform/tf/*.csv`,
  one message per file (columns `stamp_sec,stamp_nanosec,frame_id,child_frame_id,tx,ty,…`). For
  the default session they span x 111.6..385.8 m and y −210.5..53.6 m over 20418 poses at 50 Hz
  (a 408 s loop). The solved heading is about 90.12° (90.1178° in the run behind the tests'
  `kSessionGroundAnchor`, `overlume/tests/test_environment_stream.cpp:40`). The GPS solve waits
  for 500 fixes and a 20 m baseline (`ros/src/overlume_ros/include/overlume_ros/geo_anchor.hpp:30-31`)
  from wherever the looping replay is when the node starts, so its last digits vary run to run.
- `pgrep -f` and `pkill -f` match the full command line of every process, including the shell
  that runs the command block, whose command line holds the pattern text. Every pattern below
  therefore brackets one letter (`[l]ib/…`) or anchors on the program (`^bash tools/…`), so it
  never matches that shell.

- [ ] **Step 1: Preconditions and a current build**

```bash
cd "$INTEGRATION"
git status --short
git log --oneline -1
cmake --build overlume/build -j8
ros/colcon_build.sh 2>&1 | grep '^Summary:'
```

Expected: no `git status` output; the build finishes; `Summary: 1 package finished […]`.

- [ ] **Step 2: HUMAN. Ask before starting the rig**

Tell the user, and continue only on an explicit yes:
- The rig owns the machine. `tools/validate_visual_mode.sh` kills any running Overlume node, vcam
  bridge, GUI or bag player on this machine, from any checkout, including another session's, and
  needs port 8765 and the replay's `/clock` for itself.
- Network: Mapbox with `MAPBOX_TOKEN` (17 requests per z14 tile: two tiles at the test site,
  one at Downtown, again after each source switch), and Cesium ion with `CESIUM_ION_TOKEN` for
  the `osm` comparison.
- About 45 minutes with the user at the GUI: they judge every visual check, and for 15 of those
  minutes the free-look camera must stay still while render time is sampled.
- The network cut, by one of two methods. The default is the kill-switch proxy: the node's HTTPS
  goes through `tools/kill_switch_proxy.py`, killing it is the cut, and nothing on the machine
  changes. The alternative is that the user disconnects the machine when asked.

- [ ] **Step 3: Workspace and preflight**

```bash
cd "$INTEGRATION"
REHEARSAL=/tmp/overlume_mapbox_rehearsal
SESSION_DIR="${SESSION_DIR:-${OVERLUME_SESSION:-$HOME/session_<stamp>}}"
mkdir -p "$REHEARSAL" && : > "$REHEARSAL/start"
printf 'date: %s\nsession: %s\n' "$(date +%F)" "$(basename "$SESSION_DIR")" \
    > "$REHEARSAL/results.txt"
test -f "$SESSION_DIR/config/player.yaml" && test -f "$SESSION_DIR/config/topics.yaml" \
    && test -f "$SESSION_DIR/config/sensors_extrinsic_calib.yaml" \
    && test -d "$SESSION_DIR/transform/tf" && echo "session OK"
test -f "${COLLECTOR_INSTALL:-$HOST_WS/data_collector/install/setup.bash}" \
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
- `session OK` and `collector OK`: the replay wrapper refuses to start without the session's
  three config files or the data collector's install (`tools/validate_logger_session.sh:32-35`).
  Without either line, stop and ask the user for the session or `COLLECTOR_INSTALL`.
- `DISPLAY set`, `no rig running`, `0` (ports 8765 and 8899 free). A listed
  process belongs to a rig the user has just agreed to stop, and the rig's start stops it, which
  also frees 8765 when that bridge holds it; a running `the logger's player` must be stopped by hand. Port
  8765 held by anything else, or 8899 held at all, must be freed first. With `DISPLAY unset` the
  GUI would not start: stop and ask the user for their display.
- `PASS: token retrieves the Downtown Dubai tiles (landmark tile HTTP 200, footprint tile HTTP 200)`,
  then `PASS: token retrieves tileset.json for asset 96188 (endpoint 200, tileset 200)`.
- `token_checks: tools/check_mapbox_token.sh PASS, overlume/scripts/check_cesium_token.sh PASS`.

On any `FAIL:` or `SKIP:` line, stop.

- [ ] **Step 4: Write the two rehearsal helpers (outside the repository)**

```bash
REHEARSAL=/tmp/overlume_mapbox_rehearsal
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
cat > "$REHEARSAL/datum.py" <<'EOF'
import csv, glob, math, sys

session, hdg, t_lat, t_lon = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
xs, ys = [], []
for path in glob.glob(f"{session}/transform/tf/*.csv"):
    with open(path, newline="") as fh:
        for row in csv.DictReader(fh):
            if row["frame_id"] == "map" and row["child_frame_id"] == "base_link":
                xs.append(float(row["tx"]))
                ys.append(float(row["ty"]))
if not xs:
    sys.exit("no map -> base_link rows under transform/tf")
cx, cy = (min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0
# MapToWgs (geo_anchor.cpp) solved for the datum: the route's centre lands on the target.
s, c = math.sin(math.radians(hdg)), math.cos(math.radians(hdg))
east, north = cx * s - cy * c, cx * c + cy * s
m_per_deg = 6371000.0 * math.pi / 180.0
lat0 = t_lat - north / m_per_deg
lon0 = t_lon - east / (m_per_deg * math.cos(math.radians(lat0)))
print(f"x {min(xs):.1f}..{max(xs):.1f} m, y {min(ys):.1f}..{max(ys):.1f} m, "
      f"centre ({cx:.1f}, {cy:.1f}), {len(xs)} poses", file=sys.stderr)
print(f"{lat0:.7f} {lon0:.7f} {hdg:.4f}")
EOF
python3 -m py_compile "$REHEARSAL/set_env_source.py" "$REHEARSAL/datum.py" && echo "helpers OK"
```

Expected: `helpers OK`. `set_env_source.py` sends what the GUI's Environment Tiles selector sends,
so the timed steps can switch sources without a click; it gives up after 60 s without an ack.

- [ ] **Step 5: Run A, the test site: start the proxy and the rig**

(a) Start the kill-switch proxy with the Bash tool's background mode:

```bash
cd "$INTEGRATION"
python3 tools/kill_switch_proxy.py 8899 > /tmp/overlume_mapbox_rehearsal/proxy.log 2>&1
```

Poll `grep -m1 listening /tmp/overlume_mapbox_rehearsal/proxy.log` until it prints
`kill_switch_proxy: listening on 127.0.0.1:8899 (kill me to cut the network)`.

(b) Start the rig, also in background mode. The chunks directory gives the stream a `fallback=`
directory for the cut.

```bash
cd "$INTEGRATION"
SESSION_DIR="${SESSION_DIR:-${OVERLUME_SESSION:-$HOME/session_<stamp>}}"
https_proxy=http://127.0.0.1:8899 no_proxy=127.0.0.1,localhost \
    tools/validate_logger_session.sh "$SESSION_DIR" --arm mapbox --anchor-height-offset -1.8 -- \
    --param "environment_chunks_dir:=$PWD/overlume/tests/fixtures/environment_test_town_0" \
    > /tmp/overlume_mapbox_rehearsal/runA.out 2>&1
```

If the user chose to disconnect the machine themselves, skip (a) and drop the two proxy variables
from (b).

- [ ] **Step 6: Wait for the arm; record the session's anchor and heading**

Poll this, with a Monitor until-loop, until it prints. It also ends the wait when the wrapper has
exited. Every early exit leaves no `[arm]` line: a missing session file or collector install, a
`/clock` that already has a publisher, a player that never publishes, or a rig that fails to come
up (`tools/validate_logger_session.sh:32-36`, `:48-51`, `:126-132`, `:146`).

```bash
grep -m1 -E '^\[arm\] (OK|FAIL)' /tmp/overlume_mapbox_rehearsal/runA.out \
    || { ! pgrep -f '^bash tools/[v]alidate_logger_session\.sh' > /dev/null && echo "wrapper exited"; }
```

Expected: `[arm] OK: environment source = mapbox`. Earlier `[arm] refused: … geo-anchor not solved
yet …` lines are normal while the anchor solves from the replayed GPS. Other outcomes:
- `wrapper exited`: `tail -20 /tmp/overlume_mapbox_rehearsal/runA.out` names the cause
  (`FATAL: …`, `not a logger session …`, `data collector install not found …`, a rig failure).
  Fix it with the user, then restart at Step 5.
- `[arm] FAIL: not armed within 120 s`: the window can close before the GPS anchor solves (500
  fixes and a 20 m baseline). Poll `grep -m1 'geo-anchor solved:' /tmp/overlume_validate/overlume_node.log`
  the same way until it prints, then arm by hand with
  `python3 /tmp/overlume_mapbox_rehearsal/set_env_source.py mapbox`, which prints
  `[switch] mapbox: success=True`, and go on. If no `geo-anchor solved:` line appears within
  5 minutes, stop and read `/tmp/overlume_validate/overlume_node.log`.

```bash
REHEARSAL=/tmp/overlume_mapbox_rehearsal
grep -E 'geo-anchor solved|environment source armed' /tmp/overlume_validate/overlume_node.log
grep -m1 -F '[launch] vcam_gui.py' "$REHEARSAL/runA.out"
grep -m1 -o 'geo-anchor solved: .*' /tmp/overlume_validate/overlume_node.log \
    | sed 's/^geo-anchor solved: /opa_anchor: /' >> "$REHEARSAL/results.txt"
grep -m1 -o -- '--anchor-heading-deg [-0-9.]*' /tmp/overlume_validate/overlume_node.log \
    | awk '{print "opa_heading: " $2}' | tee -a "$REHEARSAL/results.txt"
NODE_PID="$(pgrep -f '[l]ib/overlume_ros/overlume_node' | head -1)"
tr '\0' '\n' < "/proc/$NODE_PID/environ" | grep -c -x 'https_proxy=http://127.0.0.1:8899'
```

Expected:
- The `geo-anchor solved:` line, and
  `environment source armed: 'mapbox://buildings?fallback=…/overlume/tests/fixtures/environment_test_town_0' (visible)`.
- `[launch] vcam_gui.py (log: /tmp/overlume_validate/vcam_gui.log)`: the GUI is up.
- `opa_heading:` about 90.12 for the default session (90.1178 in the run behind
  `kSessionGroundAnchor`; the last digits vary with where the replay was when the node started).
- `1`: the node's environment carries the proxy, so its HTTPS goes through it (the count reads
  the node's environment without printing it). With `0`, Step 9 would test nothing: stop and
  check the proxy variables in Step 5. When the user disconnects instead, skip this check.

- [ ] **Step 7: HUMAN. Visual checks at the test site (mode 3, the GUI on `mapbox`)**

Ask the user to confirm, in the GUI's video:
1. Extruded clay buildings stand on the map plane along the route, and no landmark model appears.
   Mapbox serves none here.
2. Mapbox's logo sits 8 px from the bottom-left corner, and "© Mapbox © OpenStreetMap Improve this
   map" is right-aligned at the bottom-right, in the HUD's text colour.
3. A z14 tile boundary runs north-south about 76 m east of the route's eastern end (map
   x ≈ 462 m for the default session). Buildings that cross it show no wall, gap or doubled
   geometry along it. This is Review Focus 2 on live data. Ask first whether any building in view
   crosses that line. If none does, the check has nothing to see: record it as below and go on.

Record each verdict as one line of `results.txt`: the key, `confirmed` or `not confirmed`, and the
user's own words, never a token or a path. For example, for the first check:

```bash
cat >> /tmp/overlume_mapbox_rehearsal/results.txt <<'EOF'
opa_extrusions: confirmed -- "clay blocks along the road, all standing on the plane"
EOF
```

When no building crosses the z14 line, the third check's line is
`opa_seam: confirmed -- no building crosses the z14 line in view; seams pinned by Task 13's CornerLoadsFourTilesWithoutDoubledSeamWalls`.

The keys here are `opa_extrusions`, `opa_attribution` and `opa_seam`. Later steps record their
checks the same way.

- [ ] **Step 8: HUMAN. The GUI's two Mapbox entries**

Ask the user to pick `mapbox-original` in the GUI's Environment Tiles selector. Expect Mapbox's
look: warm off-white extrusions (Mapbox Standard's default building colour, spec §5 Look), with
the attribution still on. Then ask them to pick `mapbox` again and expect the theme's clay. Record
`opa_gui_looks`.

- [ ] **Step 9: The network cut reaches the baked fallback**

With the proxy, cut, then make the stream request something new. Resident tiles alone never fail
a request (`docs/status.md` open item 18), so a live switch opens a fresh source, which needs no
network to open, and its tiles' requests (17 per z14 tile) then fail at the dead proxy.

```bash
REHEARSAL=/tmp/overlume_mapbox_rehearsal
PROXY_PID="$(pgrep -f '^python3 tools/[k]ill_switch_proxy\.py 8899')"
kill "$PROXY_PID"
timeout 10 tail --pid="$PROXY_PID" -f /dev/null; echo "proxy wait rc=$?"
ss -tlnH '( sport = :8899 )' | wc -l
printf 'cut_method: %s\n' "kill-switch proxy killed" >> "$REHEARSAL/results.txt"
python3 "$REHEARSAL/set_env_source.py" mapbox-original
```

Expected: `proxy wait rc=0`, `0` (nothing listens on 8899), then
`[switch] mapbox-original: success=True`. If the user cuts the network instead, ask them to
disconnect now, record `cut_method: the user disconnected the machine`, and run only the last
line.

Poll this until it prints:

```bash
grep -m1 'network loss detected' /tmp/overlume_validate/overlume_node.log
```

Expected: `environment source: network loss detected -- switched to fallback dir '…/environment_test_town_0'`.
If nothing appears within 60 s, acceptance 4 has failed: stop and report. Then:

```bash
REHEARSAL=/tmp/overlume_mapbox_rehearsal
awk -F'[][]' '/environment source armed: .mapbox:\/\/buildings\?materials=original/ {armed = $4}
     /network loss detected/ {warn = $4}
     END {if (armed != "" && warn != "") printf "cut_seconds: %.2f\n", warn - armed;
          else print "cut_seconds: no WARN yet"}' /tmp/overlume_validate/overlume_node.log \
    | tee -a "$REHEARSAL/results.txt"
```

Expected: `cut_seconds:` a few seconds at most. The WARN shows the transition on real transport
errors, end to end. The threshold itself, eight consecutive failed requests
(`kNetworkLossConsecutiveFailures`), is pinned for transport errors by Task 6's
`EnvironmentStream.TransportErrorsCountAsNetworkLossAndFallBack` and for Mapbox by Task 13's
transport-failure test; the rig adds the proof that curl's real errors now count.

Ask the user to confirm that the Mapbox logo and text are gone, and record `cut_attribution`. If
they disconnected the machine, ask them to reconnect now. The fallback's line deliberately goes
unrecorded: it holds a local absolute path.

- [ ] **Step 10: Tear run A down; check its logs for the token**

```bash
pkill -TERM -f '^bash tools/[v]alidate_logger_session\.sh'
for p in $(pgrep -f '^bash tools/[v]alidate_logger_session\.sh'); do
    timeout 60 tail --pid="$p" -f /dev/null
done
RIG='[l]ib/overlume_ros/overlume_node|tools/[v]cam_ws_bridge\.py|tools/[v]cam_gui\.py'
pgrep -af "$RIG|[m]p_play|tools/[k]ill_switch_proxy|^bash tools/[v]alidate_logger_session" \
    || echo "rig down"
```

Expected: `rig down` within a few seconds; the wrapper's teardown stops the node, the bridge, the
GUI, the player and the static TF publishers.

```bash
REHEARSAL=/tmp/overlume_mapbox_rehearsal
TOK_LOGIN="$(env -u MAPBOX_TOKEN bash -lic 'printf %s "${MAPBOX_TOKEN:-}" >&3' \
    3>&1 >/dev/null 2>&1)"
LOGS=(/tmp/overlume_validate/*.log "$REHEARSAL/runA.out")
[[ -f "$REHEARSAL/proxy.log" ]] && LOGS+=("$REHEARSAL/proxy.log")
HITS=0
SCANNED=0
for tok in "$TOK_LOGIN" "${MAPBOX_TOKEN:-}"; do
    [[ -n "$tok" ]] || continue
    SCANNED=$((SCANNED + 1))
    while IFS= read -r line; do
        echo "$line"
        HITS=$((HITS + ${line##*:}))
    done < <(grep -c -F -f <(printf '%s' "$tok" | tr -d '[:space:]') /dev/null "${LOGS[@]}")
done
printf 'token_hits_a: %s\ntoken_files_a: %s\ntoken_scanned_a: %s\n' "$HITS" "${#LOGS[@]}" \
    "$SCANNED" | tee -a "$REHEARSAL/results.txt"
```

Expected: one `<file>:0` line per log and token (and `/dev/null:0`, which keeps grep's
`file:count` form even for one file), then `token_hits_a: 0`, the file count, and
`token_scanned_a: 1` or `2` (2 when this shell also exports `MAPBOX_TOKEN`). This is `grep -c -F`
with the token as the pattern, trimmed as the library trims it, but the pattern goes in through a
file descriptor rather than as an argument, so the token never appears in the process list and is
never printed. Any non-zero count is a leak: stop, tell the user to rotate the token, and report
it. `token_scanned_a: 0` means no token was found to scan for, so the zero proves nothing: stop
and find out why the login shell gave no `MAPBOX_TOKEN`.

- [ ] **Step 11: The Downtown datum, from the session's route extent**

The datum pins map (0, 0) to a latitude and longitude, not the robot: the replayed route keeps its
map coordinates. So pick a target T for the route's centre and solve the datum that puts the
centre there. Keeping the session's map heading keeps the route's true-north orientation, so it
has the same shape on the ground as at the test site. `datum.py` reads the route's
map → base_link poses, takes the centre (cx, cy) of their extent, turns it into east and north
metres with the node's own `MapToWgs` rotation, and subtracts that offset from T on the node's
6371 km sphere.

T = 25.19451 N, 55.27341 E: 280 m south and 80 m west of the Burj Khalifa's landmark node
(25.19703 N, 55.27421 E in the 2026-10-14 probe). While this plan was written, the default
session's whole route was laid over the probe's streets-v8 building outlines for targets on a
20 m grid within 400 m of the tower, then on a 10 m grid around the clear ones. At this T all
20418 poses fall outside every outline, at least 18.7 m from the nearest, and 155–413 m from the
tower, inside the 500 m far plane; a target 10 m off in any direction still clears every outline
by at least 9 m. The probe's data stayed outside the repository; only these coordinates are
recorded.

```bash
REHEARSAL=/tmp/overlume_mapbox_rehearsal
SESSION_DIR="${SESSION_DIR:-${OVERLUME_SESSION:-$HOME/session_<stamp>}}"
HDG="$(sed -n 's/^opa_heading: //p' "$REHEARSAL/results.txt" | tail -1)"
T_LAT=25.19451
T_LON=55.27341
python3 "$REHEARSAL/datum.py" "$SESSION_DIR" "$HDG" "$T_LAT" "$T_LON" \
    > "$REHEARSAL/datum.txt" 2> "$REHEARSAL/datum.err"
cat "$REHEARSAL/datum.err" "$REHEARSAL/datum.txt"
read -r DT_LAT DT_LON DT_HDG < "$REHEARSAL/datum.txt"
printf 'dt_extent: %s\ndt_target: %s N, %s E\ndt_lat: %s\ndt_lon: %s\ndt_hdg: %s\n' \
    "$(cat "$REHEARSAL/datum.err")" "$T_LAT" "$T_LON" "$DT_LAT" "$DT_LON" "$DT_HDG" \
    >> "$REHEARSAL/results.txt"
```

Expected for the default session, shown for heading 90.1178:

```
x 111.6..385.8 m, y -210.5..53.6 m, centre (248.7, -78.5), 20418 poses
25.1952203 55.2709398 90.1178
```

The second line carries this run's `opa_heading`, so its heading, and with it the datum's last
digits, can differ from the example. With another session the numbers differ; the method does
not. When Step 13 moves the
target, rerun this step with the new `T_LAT`/`T_LON`; at this latitude 1 m north is 1/111195
degree and 1 m east about 1/100620 degree. The later values override the earlier ones in
`results.txt`.

- [ ] **Step 12: Run B, Downtown: start the rig with the full override**

Start it in background mode, with the network on and no proxy:

```bash
cd "$INTEGRATION"
REHEARSAL=/tmp/overlume_mapbox_rehearsal
SESSION_DIR="${SESSION_DIR:-${OVERLUME_SESSION:-$HOME/session_<stamp>}}"
read -r DT_LAT DT_LON DT_HDG < "$REHEARSAL/datum.txt"
tools/validate_logger_session.sh "$SESSION_DIR" --anchor-height-offset -1.8 -- \
    --param "geo_datum_lat_deg:=$DT_LAT" --param "geo_datum_lon_deg:=$DT_LON" \
    --param "geo_datum_heading_deg:=$DT_HDG" \
    --param 'environment_source_uri:=mapbox://buildings?materials=original' \
    > "$REHEARSAL/runB.out" 2>&1
```

Poll this, with a Monitor until-loop, until it prints; as in Step 6, it also ends the wait when the
wrapper has exited:

```bash
grep -m1 '^Rig is up' /tmp/overlume_mapbox_rehearsal/runB.out \
    || { ! pgrep -f '^bash tools/[v]alidate_logger_session\.sh' > /dev/null && echo "wrapper exited"; }
```

`wrapper exited` means the rig never came up: `tail -20 /tmp/overlume_mapbox_rehearsal/runB.out`
names the cause. Fix it with the user and restart this step. After `Rig is up`:

```bash
grep -E 'geo_datum override|environment source armed' /tmp/overlume_validate/overlume_node.log
```

Expected:
`geo-anchor solved (geo_datum override): --anchor-lat 25.19522030 --anchor-lon 55.27093980 --anchor-heading-deg 90.1178 --anchor-height -1.80`
(with this run's datum and heading from `datum.txt`, whose last digits can differ), then
`environment source armed: 'mapbox://buildings?materials=original' (visible)`. The override
solves the anchor at configure, so the source arms at activate; no `--arm` is needed.

- [ ] **Step 13: HUMAN. Visual checks at Downtown (mode 3, `mapbox-original`)**

Ask the user to confirm:
1. The Burj Khalifa and the tile's other landmarks render as textured models, with no
   warm-white extruded block inside or around any of them, among streets of extruded buildings.
2. Over one full loop of the replay (about 7 minutes for the default session), with the free-look
   camera high above the ego, the route runs on open ground near the tower and never through a
   building.
3. The attribution is visible.

If the route crosses buildings, ask the user which way to move it. Shift the target that way in
steps of 20 m, keeping it within about 350 m of the tower so the tower stays inside the 500 m far
plane from the whole route. Then tear run B down (Step 16's teardown block), rerun Step 11 with
the new target, and restart at Step 12. The datum is fixed for the node's lifetime, so each
attempt needs a restart.

Record `dt_landmarks`, `dt_open_ground` and `dt_attribution`.

- [ ] **Step 14: HUMAN. The GUI's look switch at Downtown**

Ask the user to pick `mapbox` (expect the theme's clay on the landmarks and the extrusions, with
the attribution still on), then `mapbox-original` again. Record `dt_gui_looks`.

- [ ] **Step 15: Render cost: clay `mapbox` against `osm` at the same poses**

Ask the user to leave the free-look camera alone for the next 15 minutes. Each source is sampled
over one full replay loop (420 s for the default session's 408 s loop; for another session, its
loop length rounded up), so both see the same poses. The first 30 s after each switch are sampled
and discarded: tiles loading are not steady state. The GUI's selector may lag behind these
scripted switches; the video is what counts.

Each sampling block runs in one Bash call: up to 60 s for the switch's ack, 30 s of settling, then
the loop. A foreground call ends at 600 s, so the default 420 s (510 s at most) fits, and a loop
longer than about 480 s does not. For such a session, put the loop length in place of `420` and
run each block in the Bash tool's background mode, then read its output when it exits. One
sample over the whole loop is what makes the p50 comparable: never split a loop into halves.

```bash
cd "$INTEGRATION"
REHEARSAL=/tmp/overlume_mapbox_rehearsal
source /opt/ros/humble/setup.bash
python3 "$REHEARSAL/set_env_source.py" mapbox || exit 1
python3 overlume/tools/sample_diagnostics.py 30 > /dev/null
printf 'mapbox_line: %s\n' "$(python3 overlume/tools/sample_diagnostics.py 420)" \
    | tee -a "$REHEARSAL/results.txt"
```

Expected (foreground, Bash timeout 600000 ms; about 7.5 minutes):
`[switch] mapbox: success=True`, then
`mapbox_line: render_ms_p50=… render_ms_p99=… render_ms_n=…`, with n up to about 12600 (one
sample per rendered frame on the node's 30 Hz timer).

```bash
cd "$INTEGRATION"
REHEARSAL=/tmp/overlume_mapbox_rehearsal
source /opt/ros/humble/setup.bash
python3 "$REHEARSAL/set_env_source.py" osm || exit 1
python3 overlume/tools/sample_diagnostics.py 30 > /dev/null
printf 'osm_line: %s\n' "$(python3 overlume/tools/sample_diagnostics.py 420)" \
    | tee -a "$REHEARSAL/results.txt"
```

Expected: `[switch] osm: success=True`, then an `osm_line:` of the same form.

```bash
python3 - /tmp/overlume_mapbox_rehearsal/results.txt <<'EOF'
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
print(f"mapbox p50 {mb50:.1f} ms / osm p50 {osm50:.1f} ms = {ratio:.2f}: {verdict} (bar 1.5)")
EOF
```

Expected: `… = 1.xx: PASS (bar 1.5)`, or lower. A `FAIL` is a finding for the user (acceptance 3):
report it, and do not close the plan.

The source is now `osm`. Ask the user to confirm that the Mapbox attribution is gone, because the
gate checks the scheme, and record `osm_attribution`.

- [ ] **Step 16: Tear run B down; check its logs and the ROS logs for the token**

Teardown:

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
REHEARSAL=/tmp/overlume_mapbox_rehearsal
TOK_LOGIN="$(env -u MAPBOX_TOKEN bash -lic 'printf %s "${MAPBOX_TOKEN:-}" >&3' \
    3>&1 >/dev/null 2>&1)"
mapfile -t ROSLOGS < <(find "${ROS_LOG_DIR:-${ROS_HOME:-$HOME/.ros}/log}" -type f \
    -newer "$REHEARSAL/start" 2>/dev/null)
LOGS=(/tmp/overlume_validate/*.log "$REHEARSAL/runB.out" "${ROSLOGS[@]}")
HITS=0
SCANNED=0
for tok in "$TOK_LOGIN" "${MAPBOX_TOKEN:-}"; do
    [[ -n "$tok" ]] || continue
    SCANNED=$((SCANNED + 1))
    while IFS= read -r line; do
        HITS=$((HITS + ${line##*:}))
    done < <(grep -c -F -f <(printf '%s' "$tok" | tr -d '[:space:]') /dev/null "${LOGS[@]}")
done
printf 'token_hits_b: %s\ntoken_files_b: %s\ntoken_scanned_b: %s\n' "$HITS" "${#LOGS[@]}" \
    "$SCANNED" | tee -a "$REHEARSAL/results.txt"
```

Expected: `token_hits_b: 0`, over run B's rig logs and every ROS log file written since Step 3,
from both runs, then the file count and `token_scanned_b: 1` or `2`. Any non-zero hit count is a
leak: stop, tell the user to rotate the token, and report it. `token_scanned_b: 0` proves nothing:
stop, as in Step 10.

- [ ] **Step 17: Write the rehearsal into the docs**

The script fills the runbook's §7.2 values, appends the `## Shipped` row and the update to open
item 18 in `docs/status.md`, closes ledger row 17 and adds the Live acceptance results to the plan,
and marks the plan's `docs/README.md` entry closed. It refuses to write anything when a token
count is non-zero, a token scan had no token to scan for, a visual check was not confirmed, the
render-cost verdict is `FAIL`, or the plan's `docs/README.md` entry, which the plan's commit adds,
is missing.

```bash
cd "$INTEGRATION"
python3 - /tmp/overlume_mapbox_rehearsal/results.txt <<'PYEOF'
import subprocess
import sys

v = {}
for line in open(sys.argv[1]).read().splitlines():
    if ": " in line:
        key, value = line.split(": ", 1)
        v[key] = value.strip()
for key in ("token_scanned_a", "token_scanned_b"):
    assert int(v[key]) >= 1, f"{key} = {v[key]}: no token was scanned for; zero hits prove nothing"
for key in ("token_hits_a", "token_hits_b"):
    assert v[key] == "0", f"{key} = {v[key]}: the token reached a log -- stop, rotate it, report"
for key in ("opa_extrusions", "opa_attribution", "opa_seam", "opa_gui_looks", "cut_attribution",
            "dt_landmarks", "dt_open_ground", "dt_attribution", "dt_gui_looks", "osm_attribution"):
    assert v[key].startswith("confirmed"), f"{key}: {v[key]} -- report it, do not close"
assert v["ratio_verdict"] == "PASS", "clay render_ms over 1.5x osm -- report, do not close"
LINK = "- [`2026-10-14-mapbox-buildings.md`](plans/2026-10-14-mapbox-buildings.md)"
readme = open("docs/README.md").read()
assert readme.count(LINK) == 1, ("docs/README.md: the plan's entry must exist once; the plan's "
                                 "commit adds it")
end = readme.index(").\n", readme.index(LINK))  # the entry's closing "(implements ...)."
v["token_files"] = str(int(v["token_files_a"]) + int(v["token_files_b"]))
v["head"] = subprocess.run(["git", "rev-parse", "--short", "HEAD"], capture_output=True,
                           text=True, check=True).stdout.strip()


def replace_once(path, old, new):
    text = open(path).read()
    assert text.count(old) == 1, f"{path}: {text.count(old)} matches for {old[:70]!r}"
    open(path, "w").write(text.replace(old, new))


for param, key in (("geo_datum_lat_deg", "dt_lat"), ("geo_datum_lon_deg", "dt_lon"),
                   ("geo_datum_heading_deg", "dt_hdg")):
    replace_once("docs/runbooks/mapbox.md", f"| `{param}` | filled in by Task 17's rehearsal |",
                 f"| `{param}` | `{v[key]}` |")

SHIPPED = (
    "| Mapbox buildings environment source `mapbox://buildings`, selectable in the vcam GUI as "
    "`mapbox` (the theme's clay) and `mapbox-original` (Mapbox's look): landmark models plus "
    "extruded streets-v8 footprints through a custom cesium-native loader with area selection "
    "around the ego, Mapbox attribution on screen while streaming, and transport errors counted "
    "toward the network-loss fallback (ion too). Rehearsed {date} on the replay rig at the "
    "test site and at Downtown Dubai; clay `render_ms` p50 {mb_p50} ms against `osm` "
    "{osm_p50} ms; details in the plan's Live acceptance results. Local only: nothing is pushed "
    "before Mapbox's sign-off | CLOSED {date} at `{head}` (rehearsal and docs in the following "
    "commit) | [`plans/2026-10-14-mapbox-buildings.md`](plans/2026-10-14-mapbox-buildings.md) |\n")
status = open("docs/status.md").read()
anchor = "\n\n## Open items"
assert status.count(anchor) == 1, "docs/status.md: expected one '## Open items' heading"
cut = status.index(anchor) + 1
open("docs/status.md", "w").write(status[:cut] + SHIPPED.format(**v) + status[cut:])

ITEM18 = "Needs a library change + fixture tests; not done in the closing pass."
replace_once("docs/status.md", ITEM18, ITEM18 + (
    " **Update {date} (Mapbox plan):** transport errors now count toward the threshold, for ion "
    "too. A `mapbox://` source has no handshake and each new z14 tile issues 17 requests, so a "
    "network that is dead when it arms, or cut before its next new tile, does reach the fallback "
    "(rehearsed: the WARN came {cut_seconds} s after a live switch with the network cut). "
    "Resident tiles still never trigger it, and the ion handshake case above stays open."
).format(**v))

LEDGER_17 = "| 17. Rig rehearsal at both sites and close-out | 8 |"
plan = "docs/plans/2026-10-14-mapbox-buildings.md"
lines = open(plan).read().split("\n")
rows = [i for i, line in enumerate(lines) if line.startswith(LEDGER_17)]
assert len(rows) == 1, f"{plan}: {len(rows)} ledger rows for Task 17"
lines[rows[0]] = (LEDGER_17 + " done (see Live acceptance results) | the close-out commit after "
                  "`{head}` |").format(**v)
RESULTS = """### Live acceptance results ({date}, replay rig, `{session}`)

- Preflight: {token_checks}. Live smoke (Task 16): ledger row 16.
- Test site, the session's own anchor ({opa_anchor}), armed with `--arm mapbox`. Extruded buildings on the map plane and no landmark models: {opa_extrusions}. The z14 seam east of the route: {opa_seam}. Attribution while streaming: {opa_attribution}. GUI `mapbox-original`, then `mapbox`: {opa_gui_looks}.
- Network cut ({cut_method}), then a live switch to `mapbox-original` so that new requests fail: the `network loss detected` WARN came {cut_seconds} s after the switch. Attribution after the fallback: {cut_attribution}.
- Downtown Dubai: route extent {dt_extent}; route centre placed at {dt_target}; datum `geo_datum_lat_deg:={dt_lat}`, `geo_datum_lon_deg:={dt_lon}`, `geo_datum_heading_deg:={dt_hdg}` (the session's heading), launched with `environment_source_uri:=mapbox://buildings?materials=original`. Landmarks as models with no doubled extrusion: {dt_landmarks}. Route on open ground: {dt_open_ground}. Attribution: {dt_attribution}. GUI `mapbox` against `mapbox-original`: {dt_gui_looks}.
- Render cost at Downtown, `render_ms` from `/overlume_node/diagnostics` over one full replay loop per source, after 30 s of settling, same free-look camera: `mapbox` (clay) p50 {mb_p50} ms, p99 {mb_p99} ms; `osm` p50 {osm_p50} ms, p99 {osm_p99} ms; p50 ratio {ratio} against the 1.5 bar: {ratio_verdict}. Mapbox attribution with `osm` selected: {osm_attribution}.
- Secrets: the `MAPBOX_TOKEN` value appears in none of the {token_files} node, rig and ROS log files written during the rehearsal.
"""
text = "\n".join(lines)
anchor = "\n## Global Constraints"
assert text.count(anchor) == 1, f"{plan}: expected one '## Global Constraints' heading"
open(plan, "w").write(text.replace(anchor, "\n" + RESULTS.format(**v) + anchor))

open("docs/README.md", "w").write(readme[:end] + "; closed {date}".format(**v) + readme[end:])
print("docs written for", v["date"])
PYEOF
git diff --stat
```

Expected: `docs written for <today>`, then a diff stat touching exactly `docs/README.md`,
`docs/plans/2026-10-14-mapbox-buildings.md`, `docs/runbooks/mapbox.md` and `docs/status.md`. The
README entry's last line then ends ``-design.md`; closed <today>).``. The script was dry-run
against copies of today's `docs/status.md`, of `docs/README.md` with the plan's entry as staged
with the plan, of this plan as assembled and of Task 12's draft runbook; with the entry removed,
and with a zero `token_scanned_b`, it refused before writing anything. A failed assertion names
the anchor or value that is missing: fix the record, not the assertion. An assertion that fails
after the writes began (a `docs/status.md` or plan anchor) leaves files half written: restore them
with `git checkout -- docs/` (nothing else in `docs/` has changed since Step 1) before running it
again.

- [ ] **Step 18: Runbook facts, docs links and the final gate**

Task 12 wrote `docs/runbooks/mapbox.md` before the code it describes was merged: the 401 log
line, the unreadable-landmark-tile line, the 3 attempts and the missing-token line (Tasks 13 and
14), the logo parameter (Task 10) and the `MapboxLive.*` command (Task 16). Check each against the
merged code:

```bash
cd "$INTEGRATION"
for s in 'Mapbox rejected MAPBOX_TOKEN (HTTP {})' 'landmarks of tile 14/{}/{} skipped' \
         'kMaxAttempts = 3' 'mapbox:// needs MAPBOX_TOKEN'; do
    grep -qF "$s" overlume/src/mapbox_loader.cpp || echo "runbook fact drifted: $s"
done
grep -qF '"environment_attribution_logo_path"' ros/src/overlume_ros/src/overlume_node.cpp \
    || echo "runbook fact drifted: logo parameter"
grep -qF 'TEST(MapboxLive, DowntownDubai)' overlume/tests/test_mapbox_buildings.cpp \
    || echo "runbook fact drifted: MapboxLive"
```

Expected: no output. Each printed line names a runbook sentence to reconcile in Step 19's commit:
change `docs/runbooks/mapbox.md` to what the merged code does, and say so in the commit message.

Then the links and the gate, in the foreground, Bash timeout 600000 ms, under the gate lock:

```bash
cd "$INTEGRATION"
python3 tools/check_docs_links.py
cmake --build overlume/build -j8 \
    && flock -w 300 -E 75 /tmp/overlume-mapbox-gate.lock tools/ci_visual_mode.sh
```

Expected: `OK: no broken links`, then six `PASS` lines and exit 0, with the counts of Task 16's
gate. Exit 75 means another gate held the lock for 300 s: run the same command again. Any other
non-zero exit is a gate failure.

- [ ] **Step 19: Commit**

```bash
cd "$INTEGRATION"
git add docs/README.md docs/plans/2026-10-14-mapbox-buildings.md docs/runbooks/mapbox.md \
        docs/status.md
git commit -F - <<'EOF'
docs(mapbox): record the replay-rig rehearsal at both sites and close the plan

The rehearsal on the logger-session replay rig covered the test site
(--arm mapbox, then mapbox-original in the GUI, then a network cut that
reached the baked fallback) and Downtown Dubai (a full geo-datum override
that keeps the session's map heading and places the replayed route on open
ground near the Burj Khalifa, armed with
mapbox://buildings?materials=original). The runbook's Downtown table now
holds the chosen datum. docs/status.md gets the Shipped row and an update to
open item 18: a Mapbox source reaches the fallback on a dead network, while
ion's handshake case stays open. The plan records the measured results and
is marked closed in docs/README.md. Nothing is pushed: the branch stays local
until Mapbox's sign-off.

Claude-Session: <the executing session's URL>
EOF
```

- [ ] **Step 20: Pre-push readiness, recorded but not pushed**

```bash
cd "$INTEGRATION"
tools/check_format.sh
tools/check_spdx.sh
shellcheck --severity=error tools/*.sh && echo "shellcheck: PASS"
python3 -m pytest tools/test_check_mapbox_token.py -q -p no:anyio
tools/check_publish_leaks.sh
git diff --name-status --diff-filter=MD origin/main...HEAD -- overlume/tests/goldens \
    ros/src/overlume_ros/test/fixtures
git diff --name-status --diff-filter=A origin/main...HEAD -- overlume/tests/goldens
git rev-parse --abbrev-ref --symbolic-full-name '@{u}'
```

Expected:
- `check_format.sh: PASS (…)`, `check_spdx.sh: PASS (…)` and `shellcheck: PASS`.
- `8 passed in …s`: the token check's own tests, which the gate never runs (its stage 4 runs only
  `tools/test_vcam_ws_bridge.py`, `tools/ci_visual_mode.sh:139`), so a later change to
  `tools/check_mapbox_token.sh` is re-checked here before any push.
- `PASS  publish leak check over origin/main..HEAD (… commits)`.
- Nothing from the first `git diff`: no existing golden moved or vanished, in the library or the
  node (acceptance 1).
- One line from the second: `A	overlume/tests/goldens/mapbox_buildings_synthetic_dark_adas.png`.
- `fatal: no upstream configured for branch 'mapbox-environment'`: nothing is pushed.

On a leak-check `FAIL`, stop and tell the user. The fix is a history rewrite on the unpushed
branch, which needs their go-ahead, and it must happen before any push. Never push now; the push
waits for Mapbox's sign-off and its own lint pass.

---
