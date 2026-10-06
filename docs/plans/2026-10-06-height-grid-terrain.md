# Height-Grid Terrain Layer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. In this repo, execution runs as a dynamic workflow (AGENTS.md): session-model orchestrator, Sonnet implementers, Opus review gates, at most two fix rounds per task.

**Goal:** Render the offroad geometric costmap node's terrain height map as a shaded 3D heightfield in visual mode (mode 3), fed by metric heights the node publishes on `/debug_ogm_2`.

**Architecture:** The perception node encodes its float height image as a fixed-window `OccupancyGrid` on `/debug_ogm_2`. A new node adapter (`height_grid`) decodes it to float metres and places it with the same helper as the OGM adapter. The node then hands it to the library as a new POD layer, `HeightGridLayer` (`kSceneVersion` 9). The library draws one lit heightfield mesh per layer, coloured by a theme ramp, and cuts the clay ground and grid lines out of the terrain footprint so dips stay visible.

**Tech Stack:** C++20 (clang/libc++ library, gcc node), Filament 1.56.5 (`.mat` materials compiled by `matc`), yaml-cpp, ROS 2 Humble (`rclcpp`, `nav_msgs/OccupancyGrid`), GoogleTest/ament_gtest, OpenCV in the perception repo, Python 3 + NumPy for the probe.

**Spec:** `docs/design/2026-10-06-height-grid-terrain-design.md` (read it with this plan; the plan argues from it).

## Status ledger

| Task | State | Commit |
|---|---|---|
| 1 Public scene API and deep copy | done | |
| 2 Theme tokens | done | |
| 3 Heightfield renderer | done | |
| 4 Ground replacement (deviation: `DiscardReveals*` threshold 10 -> 5; measured ~9.5 with the hole, ~0.5 reverted, light_clay quality 1; a theme change pushing it toward 5 is a signal, not a flake) | done | |
| 5 Grid placement helper | done | |
| 6 Profile keys, adapter, SceneAssembly | done | |
| 7 Node wiring, layer flag, offroad profile | done | |
| 8 Perception encoder and live probe | done | |
| 9 Goldens | pending (human promotion) | |
| 10 Docs, status and live acceptance | pending | |

## Global Constraints

Paths below use placeholders: `$WORKTREE` is the git worktree this plan was executed in (branch `feat/height-grid`), `$PRIMARY_CHECKOUT` the repository's primary checkout (busy with other work at the time), `$PERCEPTION_REPO` the perception stack repository that produces `/debug_ogm_2`, and `$SCRATCH` a per-session scratch directory.

- Branch `feat/height-grid`, cut from `main`. One commit per task, and the message says what changed and why. The last line of every message is `Claude-Session: https://claude.ai/code/session_018o7iLaY69qZAcNUfeD2VUi`.
- Every task ends green on `tools/ci_visual_mode.sh`, run in the **foreground**.
- Public headers are POD-only and append-only (ADR-0003/0004). `overlume/scripts/check_pod_header.sh` must pass: no `std::` anywhere in `overlume/include/overlume/*`, comments included. The struct change bumps `kSceneVersion` 8 → 9, exactly once (Task 1).
- `HeightGridLayer` layout on LP64: size 64. Offsets: origin 0, yaw_rad 24, resolution_m 32, width_cells 40, height_cells 44, heights_m 48, last_update_sec 56. `SceneGraph` 216 → 232, with `height_grids` at 216 and `height_grid_count` at 224.
- The encoding window is −2.0 .. +3.0 m (0.05 m steps), linear 0..100, −1 = unknown. It is set in two places that must match: `$PERCEPTION_REPO/config/off_road/m2/geometric_cost_map_config.yaml` (`debug_height_encoding`) and the profile row (`height_min_m`, `height_max_m`).
- Heights are metres above the grid origin plane (the nominal ground under the ego). In the layer, NaN means unknown.
- Theme defaults: `ground_bias_m` −0.05, `roughness` 0.9, `unknown_color` = `palette.ground`. The fallback ramp is `palette.ground` at 0.0 m and the alert warning colour at 2.5 m.
- Staleness fade runs 0.5 → 1.0 s via `SceneBuffer::staleness_alpha`. The convention is fresh-opaque / stale-translucent, and faded materials premultiply alpha.
- The terrain shows only in mode 3 (FREE_LOOK), and only while `layer_height_grids` is true (default true, live-settable). A profile may contain at most one `height_grid` row.
- Goldens are promoted by a human. A failing golden is a finding, never a file to overwrite. Existing goldens must not move.
- the perception repo edits are left uncommitted (detached HEAD, with unrelated local edits). Only `CESIUM_ION_TOKEN`/`MAPBOX_TOKEN` by name; no test needs network or a token.
- **Work only in the worktree `$WORKTREE`** (branch `feat/height-grid`). `$PRIMARY_CHECKOUT` is another session's live checkout of `release-packaging`: never read from it, build in it or switch its branch. The worktree has its own `overlume/build` (Filament tarball seeded from the main checkout) and its own `ros/build` / `ros/install`, so the gate never touches the install a live rig in the main checkout runs from.

## Review Focus

Each line is pinned by a test in the owning task.

1. **Layers far from the world origin** (the live rig's map coordinates are tens to hundreds of metres from 0). The ground hole must land under the terrain footprint, so the discard has to use API world space (`getUserWorldPosition()`), not camera-relative space. Pinned in Task 4: `DiscardRevealsTerrainFarFromWorldOrigin`, with the ego and the layer at (200, 200).
2. **A theme switch while the terrain is fading** (alpha in (0, 1), per-slot faded instance bound), between themes with different `ground_bias_m` and roughness. The vertex buffer re-uploads once with the new bias, the faded instance receives the new parameters, and the slot stays faded. Pinned in Task 3: `ThemeSwitchChangingGroundBiasReuploads` plus the `height_grid_fade_roughness` hook.
3. **The layer origin moves while nothing else changes** (the ego drives; `last_update_sec`, dims, resolution, yaw and bias are unchanged). No vertex re-upload, the renderable transform moves, and the ground hole follows. Pinned in Task 3 (origin-move test, with hooks reading the real TransformManager state) and Task 4 (hole centre for a non-zero, yawed origin).
4. **A degenerate first layer** (`width/height >= 2`, `resolution > 0`, but `heights_m == nullptr`, which is what `OwnedScene` produces for an empty copy). No terrain is drawn and the ground hole stays disabled, so the clay ground is intact. Pinned in Task 1 (empty-copy semantics) and Task 4 (hole disabled).
5. **Live frames with no relief** (all-unknown frames just after the perception node starts, or near-flat terrain spanning one or two 0.05 m levels). The probe excludes those frames from the correlation check and does not count them as failures. It FAILs only if too few frames qualify, or if a qualifying frame has r <= 0.99 or mismatched unknown masks. Pinned in Task 8: the probe `--self-test` mutation cases.

Known limits accepted in this plan (not bugs):
- `last_update_sec` comes from the node's sim clock, so on a paused `/clock` a new grid that arrives at the same sim time does not re-upload. The OGM path behaves the same way (Task 6).
- The perception `debug_height_encoding` parse (block absent, or max <= min falling back to the defaults) has no automated check. Only `encode_height_cell` has one (Task 8).
- The 3 ms per-update budget is measured only live, in Task 10.
- The `ground_lines.mat` discard has no pixel check (Task 4). The ground grid lines are invisible in both shipped themes: `build_grid_lines` emits only line endpoints at +/-60 m (`kGroundHalfExtent`), past `grid.fade_end_m` 40, so every grid vertex has alpha 0 and nothing is drawn to discard. Reverting the discard leaves every `GroundHole` test green. That latent grid-fade defect predates this plan; a check becomes possible once lines are subdivided (or a fixture theme fades past 85 m).



---

### Task 1: Public scene API and deep copy

Adds `HeightGridLayer` and the two trailing `SceneGraph` fields (`kSceneVersion` 8 -> 9, additive per ADR-0004), deep-copies the heights in `OwnedScene::assign`, and pins the layout on both sides. No renderer code consumes the new fields yet (Task 3 does); value-initialised `SceneGraph{}` consumers keep working (a grep for non-empty aggregate initialisers of `SceneGraph` across the repo found none).

Every edit below is anchored by the quoted text, not by a line number. Find the anchor with `grep -n "<text>" <file>`. All paths and commands are under the worktree `$WORKTREE` (branch `feat/height-grid`); never touch `$PRIMARY_CHECKOUT`.

**Files:**
- Modify: `$WORKTREE/overlume/include/overlume/scene.h` (the `kSceneVersion` constant; new struct after the closing `};` of `GroundGridLayer`; two fields appended to `SceneGraph` after `uint32_t trajectory_carpet_count;`)
- Modify: `$WORKTREE/overlume/src/scene_buffer.hpp` (two members after `std::vector<std::vector<uint8_t>> grid_cells;`)
- Modify: `$WORKTREE/overlume/src/scene_buffer.cpp` (`OwnedScene::assign`, insert after the `view.grids = grids.data();` line)
- Test/Modify: `$WORKTREE/overlume/tests/test_scene_buffer.cpp` (include block; the `static_assert(overlume::kSceneVersion == 8` assert; new `HeightGridLayer` asserts after the `offsetof(overlume::TrajectoryCarpet, last_update_sec) == 16` assert; the `sizeof(overlume::SceneGraph) == 216` assert; new offsets after the `offsetof(overlume::SceneGraph, trajectory_carpet_count) == 208` assert; new runtime tests after the `NoNewPublish_KeepsPreviousActiveScene` test)
- Test/Modify: `$WORKTREE/ros/src/overlume_ros/test/test_scene_layout.cpp` (the `kSceneVersion == 8` assert; the `sizeof(overlume::SceneGraph) == 216` assert; new asserts after the `trajectory_carpet_count) == 208` assert)
- Modify: `$WORKTREE/CHANGELOG.md` (`[Unreleased]` / `### Changed`, new bullet after the bullet beginning "- `GroundGridLayer` (`overlume/include/overlume/scene.h`) gains `yaw_rad`")
- Modify: `$WORKTREE/README.md` (the sentence "The scene ABI is `kSceneVersion = 8`.")

**Interfaces:**
- Consumes: `overlume::Vec3`, `overlume::SceneGraph`, `overlume::detail::OwnedScene`, `overlume::detail::SceneBuffer` (existing).
- Produces (names exactly as in the contract):
  - `overlume::HeightGridLayer { Vec3 origin; double yaw_rad; double resolution_m; uint32_t width_cells, height_cells; const float* heights_m; double last_update_sec; }` (LP64: sizeof 64; origin 0, yaw_rad 24, resolution_m 32, width_cells 40, height_cells 44, heights_m 48, last_update_sec 56)
  - `SceneGraph::height_grids` (offset 216), `SceneGraph::height_grid_count` (offset 224); `sizeof(SceneGraph)` 232
  - `overlume::kSceneVersion == 9`
  - `OwnedScene::height_grids` (`std::vector<HeightGridLayer>`), `OwnedScene::height_grid_cells` (`std::vector<std::vector<float>>`); `assign()` deep-copies heights, with null `heights_m` or zero width/height giving an empty copy and `heights_m == nullptr`.

- [ ] **Step 1: Write the failing tests (layout asserts and runtime deep-copy tests)**

In `overlume/tests/test_scene_buffer.cpp`:

(a) Add `#include <cmath>` to the include block (between `<algorithm>` and `<cstddef>`):

```cpp
#include <algorithm>
#include <cmath>
#include <cstddef>
```

(b) Replace the `static_assert(overlume::kSceneVersion == 8,` assert:

```cpp
static_assert(overlume::kSceneVersion == 9,
              "bump this alongside every additive scene.h change, and update the "
              "node-side test_scene_layout.cpp mirror");
```

(c) Insert immediately after the `offsetof(overlume::TrajectoryCarpet, last_update_sec) == 16` assert (its full two-line statement), before the `sizeof(overlume::SceneGraph)` line:

```cpp

static_assert(sizeof(overlume::HeightGridLayer) == 64, "HeightGridLayer layout, ADR-0004 additive");
static_assert(offsetof(overlume::HeightGridLayer, origin) == 0,
              "HeightGridLayer layout, ADR-0004 additive");
static_assert(offsetof(overlume::HeightGridLayer, yaw_rad) == 24,
              "HeightGridLayer layout, ADR-0004 additive");
static_assert(offsetof(overlume::HeightGridLayer, resolution_m) == 32,
              "HeightGridLayer layout, ADR-0004 additive");
static_assert(offsetof(overlume::HeightGridLayer, width_cells) == 40,
              "HeightGridLayer layout, ADR-0004 additive");
static_assert(offsetof(overlume::HeightGridLayer, height_cells) == 44,
              "HeightGridLayer layout, ADR-0004 additive");
static_assert(offsetof(overlume::HeightGridLayer, heights_m) == 48,
              "HeightGridLayer layout, ADR-0004 additive");
static_assert(offsetof(overlume::HeightGridLayer, last_update_sec) == 56,
              "HeightGridLayer layout, ADR-0004 additive");
```

(d) Change the `SceneGraph` size assert to:

```cpp
static_assert(sizeof(overlume::SceneGraph) == 232, "SceneGraph layout, ADR-0004 additive");
```

(e) Insert after the `offsetof(overlume::SceneGraph, trajectory_carpet_count) == 208` assert (its full two-line statement):

```cpp
static_assert(offsetof(overlume::SceneGraph, height_grids) == 216,
              "SceneGraph layout, ADR-0004 additive");
static_assert(offsetof(overlume::SceneGraph, height_grid_count) == 224,
              "SceneGraph layout, ADR-0004 additive");
```

(f) Insert the runtime tests after the closing `}` of `NoNewPublish_KeepsPreviousActiveScene`, before the `StalenessAlpha` tests:

```cpp

TEST(SceneBuffer, HeightGrid_DeepCopySurvivesSourceMutation) {
    overlume::detail::SceneBuffer buf;
    std::vector<float> heights = {0.0f, 1.0f, std::nanf(""), 3.5f, -2.0f, 0.25f};
    overlume::HeightGridLayer layer{};
    layer.origin = {1.0, 2.0, 3.0};
    layer.yaw_rad = 0.5;
    layer.resolution_m = 0.25;
    layer.width_cells = 3;
    layer.height_cells = 2;
    layer.heights_m = heights.data();
    layer.last_update_sec = 7.0;
    std::vector<overlume::HeightGridLayer> layers = {layer};

    overlume::SceneGraph g{};
    g.height_grids = layers.data();
    g.height_grid_count = 1;
    buf.publish(g);

    // Mutate and free-reuse the caller's buffers: the published copy must not move.
    std::fill(heights.begin(), heights.end(), 99.0f);
    layers[0].width_cells = 1;
    layers[0].yaw_rad = 9.0;

    const overlume::SceneGraph& active = buf.active();
    ASSERT_EQ(active.height_grid_count, 1u);
    const overlume::HeightGridLayer& a = active.height_grids[0];
    ASSERT_NE(a.heights_m, nullptr);
    EXPECT_NE(a.heights_m, heights.data());
    EXPECT_EQ(a.width_cells, 3u);
    EXPECT_EQ(a.height_cells, 2u);
    EXPECT_DOUBLE_EQ(a.yaw_rad, 0.5);
    EXPECT_DOUBLE_EQ(a.resolution_m, 0.25);
    EXPECT_DOUBLE_EQ(a.origin.z, 3.0);
    EXPECT_DOUBLE_EQ(a.last_update_sec, 7.0);
    EXPECT_FLOAT_EQ(a.heights_m[0], 0.0f);
    EXPECT_FLOAT_EQ(a.heights_m[1], 1.0f);
    EXPECT_TRUE(std::isnan(a.heights_m[2]));  // unknown cells survive the copy
    EXPECT_FLOAT_EQ(a.heights_m[3], 3.5f);
    EXPECT_FLOAT_EQ(a.heights_m[4], -2.0f);
    EXPECT_FLOAT_EQ(a.heights_m[5], 0.25f);
}

TEST(SceneBuffer, HeightGrid_NullHeightsOrZeroDimsGiveEmptyCopy) {
    overlume::detail::SceneBuffer buf;
    float one = 1.0f;
    overlume::HeightGridLayer null_heights{};
    null_heights.width_cells = 4;
    null_heights.height_cells = 4;  // heights_m == nullptr
    overlume::HeightGridLayer zero_width{};
    zero_width.width_cells = 0;
    zero_width.height_cells = 4;
    zero_width.heights_m = &one;
    overlume::HeightGridLayer zero_height{};
    zero_height.width_cells = 4;
    zero_height.height_cells = 0;
    zero_height.heights_m = &one;
    std::vector<overlume::HeightGridLayer> layers = {null_heights, zero_width, zero_height};

    overlume::SceneGraph g{};
    g.height_grids = layers.data();
    g.height_grid_count = static_cast<uint32_t>(layers.size());
    buf.publish(g);

    const overlume::SceneGraph& active = buf.active();
    ASSERT_EQ(active.height_grid_count, 3u);
    for (uint32_t i = 0; i < 3; ++i) EXPECT_EQ(active.height_grids[i].heights_m, nullptr) << i;
}

TEST(SceneBuffer, HeightGrid_RepublishReplacesAndClears) {
    overlume::detail::SceneBuffer buf;
    std::vector<float> heights(4, 1.0f);
    overlume::HeightGridLayer layer{};
    layer.width_cells = 2;
    layer.height_cells = 2;
    layer.heights_m = heights.data();
    overlume::SceneGraph g{};
    g.height_grids = &layer;
    g.height_grid_count = 1;
    buf.publish(g);  // first buffer
    heights.assign(4, 2.0f);
    buf.publish(g);  // second buffer
    ASSERT_NE(buf.active().height_grids[0].heights_m, nullptr);
    EXPECT_FLOAT_EQ(buf.active().height_grids[0].heights_m[0], 2.0f);

    // Third publish reuses the first buffer with a null-heights layer: the view must
    // report a null pointer and count 1 (slot reuse must not resurrect the earlier copy).
    overlume::HeightGridLayer bare{};
    bare.width_cells = 2;
    bare.height_cells = 2;  // heights_m == nullptr
    overlume::SceneGraph g3{};
    g3.height_grids = &bare;
    g3.height_grid_count = 1;
    buf.publish(g3);
    ASSERT_EQ(buf.active().height_grid_count, 1u);
    EXPECT_EQ(buf.active().height_grids[0].heights_m, nullptr);

    // And an empty scene reuses the second buffer: the count must read zero.
    overlume::SceneGraph empty{};
    buf.publish(empty);
    EXPECT_EQ(buf.active().height_grid_count, 0u);
}
```

In `ros/src/overlume_ros/test/test_scene_layout.cpp`:

(g) Replace the `kSceneVersion == 8` assert (line 10):

```cpp
static_assert(overlume::kSceneVersion == 9, "node/library scene.h version drifted");
```

(h) Replace the `sizeof(overlume::SceneGraph) == 216` assert with `== 232`, and after the `offsetof(overlume::SceneGraph, trajectory_carpet_count) == 208` assert (its full two-line statement) add:

```cpp
static_assert(sizeof(overlume::HeightGridLayer) == 64, "node/library scene.h version drifted");
static_assert(offsetof(overlume::HeightGridLayer, origin) == 0,
              "node/library scene.h version drifted");
static_assert(offsetof(overlume::HeightGridLayer, yaw_rad) == 24,
              "node/library scene.h version drifted");
static_assert(offsetof(overlume::HeightGridLayer, resolution_m) == 32,
              "node/library scene.h version drifted");
static_assert(offsetof(overlume::HeightGridLayer, width_cells) == 40,
              "node/library scene.h version drifted");
static_assert(offsetof(overlume::HeightGridLayer, height_cells) == 44,
              "node/library scene.h version drifted");
static_assert(offsetof(overlume::HeightGridLayer, heights_m) == 48,
              "node/library scene.h version drifted");
static_assert(offsetof(overlume::HeightGridLayer, last_update_sec) == 56,
              "node/library scene.h version drifted");
static_assert(offsetof(overlume::SceneGraph, height_grids) == 216,
              "node/library scene.h version drifted");
static_assert(offsetof(overlume::SceneGraph, height_grid_count) == 224,
              "node/library scene.h version drifted");
```

- [ ] **Step 2: Run the tests and confirm they fail**

```
cd $WORKTREE
cmake --build overlume/build -j --target test_scene_buffer 2>&1 | tail -20
```

The target name `test_scene_buffer` is confirmed (tests are globbed from `overlume/tests/*.cpp`). A full configure is not needed here: `overlume/build/CMakeCache.txt` already carries `OVERLUME_ENABLE_CESIUM:BOOL=ON` and the worktree toolchain, and `cmake --build` re-configures by itself when a glob changes. Configuring is safe whenever no build is running in this worktree (`pgrep -af '[T]PSProjector-height-grid/(overlume|ros)/build' || echo idle`). (The gate's own configure omits the flag and relies on that cached value, so Cesium tests stay on.) If another build is still running in `overlume/build`, wait for it to finish before starting this one. Expected: compile FAILURE in `test_scene_buffer.cpp` with `static assertion failed` on `kSceneVersion == 9` and/or `no member named 'HeightGridLayer' in namespace 'overlume'`. Nothing links, so no test runs.

- [ ] **Step 3: Implement the public header change**

`overlume/include/overlume/scene.h`:

(a) Change the constant:

```cpp
constexpr uint32_t kSceneVersion = 9;
```

(b) Insert after the closing `};` of `GroundGridLayer`, before `struct AlertPolygon`. Use plain `///` lines and avoid the token `std::` in comments (check_pod_header.sh greps for it):

```cpp

/// A metric terrain heightfield: one height per cell, drawn as a lit 3D surface.
/// Cell (i, j) is the cell i along the grid's +x axis and j along its +y axis; its centre
/// sits at origin + R(yaw_rad) * ((i + 0.5) * resolution_m, (j + 0.5) * resolution_m).
struct HeightGridLayer {
    /// World position of the corner of cell (0, 0). origin.z is the height-zero plane.
    Vec3 origin;
    /// Rotation of the grid's +x axis about world +z, radians.
    double yaw_rad;
    /// Cell edge length, metres.
    double resolution_m;
    uint32_t width_cells, height_cells;
    /// width_cells * height_cells values, row-major (row j runs along +y), metres above
    /// origin.z. NaN marks an unknown cell. The renderer copies this array during set_scene,
    /// so the caller keeps ownership. Null (or a zero dimension) draws nothing.
    const float* heights_m;
    double last_update_sec;
};
```

(c) Append to the END of `SceneGraph` (after `uint32_t trajectory_carpet_count;`):

```cpp
    const HeightGridLayer* height_grids;
    uint32_t height_grid_count;
```

- [ ] **Step 4: Implement the deep copy**

`overlume/src/scene_buffer.hpp`, after `std::vector<std::vector<uint8_t>> grid_cells;`:

```cpp
    std::vector<HeightGridLayer> height_grids;
    std::vector<std::vector<float>> height_grid_cells;
```

`overlume/src/scene_buffer.cpp`, after `view.grids = grids.data();`, before the `alerts.assign` line:

```cpp

    height_grids.assign(src.height_grids, src.height_grids + src.height_grid_count);
    height_grid_cells.resize(src.height_grid_count);
    for (uint32_t i = 0; i < src.height_grid_count; ++i) {
        const HeightGridLayer& s = src.height_grids[i];
        const size_t cell_count =
            s.heights_m ? static_cast<size_t>(s.width_cells) * s.height_cells : 0;
        if (cell_count > 0) {
            height_grid_cells[i].assign(s.heights_m, s.heights_m + cell_count);
            height_grids[i].heights_m = height_grid_cells[i].data();
        } else {
            height_grid_cells[i].clear();
            height_grids[i].heights_m = nullptr;
        }
    }
    view.height_grids = height_grids.data();
```

(`view = src` at the top already copies `height_grid_count`; `view.height_grids` is re-pointed at the owned copy like every other layer. The explicit `clear()`/nullptr branch avoids relying on `vector::data()` of an empty vector. Null `src.height_grids` with count 0 is fine: the other layers already rely on the same no-op `assign`.)

- [ ] **Step 5: Run the tests and confirm they pass**

```
cd $WORKTREE
cmake --build overlume/build -j 2>&1 | tail -5
ctest --test-dir overlume/build -R "SceneBuffer|StalenessAlpha" --output-on-failure
overlume/scripts/check_pod_header.sh && echo PASS
```

Expected: build succeeds, the 3 new `SceneBuffer.HeightGrid_*` tests and the existing SceneBuffer/StalenessAlpha tests pass, and the POD gate exits 0 with no output of its own (it prints only on a violation), so the trailing `echo PASS` is what you see. Sanity check: temporarily delete `height_grids[i].heights_m = nullptr;` from the else branch, rebuild, confirm `HeightGrid_NullHeightsOrZeroDimsGiveEmptyCopy` fails on the `zero_width`/`zero_height` layers (that test, not `HeightGrid_RepublishReplacesAndClears`, is the guard for the clearing branch), then revert.

- [ ] **Step 6: CHANGELOG entry**

In `CHANGELOG.md`, under `## [Unreleased]` / `### Changed`, directly after the bullet beginning "- `GroundGridLayer` (`overlume/include/overlume/scene.h`) gains `yaw_rad`", add:

```markdown
- New `HeightGridLayer` (`overlume/include/overlume/scene.h`): a metric terrain heightfield (origin, yaw, resolution, `width_cells` x `height_cells` float heights in metres above `origin.z`, NaN = unknown). `SceneGraph` gains `height_grids` / `height_grid_count` appended at its end — `kSceneVersion` 8 → 9, additive per ADR-0004; `sizeof(SceneGraph)` 216 → 232 on LP64. `set_scene` deep-copies the height arrays like every other layer. Callers that value-initialise `SceneGraph{}` need no change; no renderer behaviour changes in this entry.
```

- [ ] **Step 7: Update the README version note**

```
cd $WORKTREE
sed -i 's/The scene ABI is `kSceneVersion = 8`\./The scene ABI is `kSceneVersion = 9`./' README.md
grep -n 'kSceneVersion = ' README.md
```

Expected: line 34 now reads "The scene ABI is `kSceneVersion = 9`." (The other `kSceneVersion` mention, "A struct change bumps `kSceneVersion`", carries no number and needs no change.)

- [ ] **Step 8: Run the gate in the foreground**

```
cd $WORKTREE
tools/ci_visual_mode.sh
```

`ros/install` does not exist in this worktree and stage 3 fails loudly without it, so build the node first (once; it is slow): `$WORKTREE/ros/colcon_build.sh overlume_ros`. The gate script itself derives `REPO_ROOT` from its own location, so nothing in it is hard-wired to the other checkout.

Expected: all 6 stages pass (POD header, library build + ctest, node colcon build + test including `test_scene_layout`, ws bridge pytest, golden counts, examples). Do not background it.

- [ ] **Step 9: Commit**

```
cd $WORKTREE
git add overlume/include/overlume/scene.h overlume/src/scene_buffer.hpp overlume/src/scene_buffer.cpp overlume/tests/test_scene_buffer.cpp ros/src/overlume_ros/test/test_scene_layout.cpp CHANGELOG.md README.md
git commit -m "feat(scene): HeightGridLayer in the public scene, kSceneVersion 9

Add HeightGridLayer (origin, yaw, resolution, width x height float heights in
metres above origin.z, NaN = unknown) and append height_grids/height_grid_count
to the end of SceneGraph; kSceneVersion 8 -> 9, additive per ADR-0004
(sizeof(SceneGraph) 216 -> 232 on LP64). OwnedScene::assign deep-copies each
layer's heights so the caller may reuse its buffer after set_scene; null
heights_m or a zero dimension yields an empty copy with a null pointer.
Layout asserts are mirrored in the library and node tests, and runtime tests
cover mutation after publish, NaN preservation, the empty-copy cases and a
republish that must clear a reused slot. README's scene ABI note follows the
bump.
This lands the type ahead of its renderer (a later task) so every consumer
compiles against the final layout once.

Claude-Session: https://claude.ai/code/session_018o7iLaY69qZAcNUfeD2VUi"
```

---

### Task 2: Theme tokens for the height grid

Adds the optional `height_grid` theme block (colour ramp by height, unknown-cell colour, roughness, ground bias), its parse with soft defaults, a ramp baker, the fallback-theme block, cross-fade blending, and the starting values in both shipped themes (spec section 8). Nothing consumes the tokens yet: uploading the baked ramp as a texture is Task 3. No golden may change in this task.

All paths and commands are relative to the worktree root `$WORKTREE` (branch `feat/height-grid`). Every command block starts with `cd` into it. Never read, edit or run anything in `$PRIMARY_CHECKOUT`. Line numbers below are hints only; every edit is anchored by quoted text. Do not touch the hybrid-splat code: the new theme block goes next to the `ogm`/`environment` parse, not next to the `hybrid_splat` parse.

**Files:**
- Modify: `overlume/src/theme.hpp` (new types directly before `struct Theme {`; new member directly after `} environment;`; new declaration directly after `Theme::OgmRamp bake_ogm_ramp(std::vector<OgmRampStop> stops);`)
- Modify: `overlume/src/theme.cpp` (parse block after `t.ogm.geometric = parse_ramp("geometric");`; `bake_height_ramp` after the end of `bake_ogm_ramp`, before `std::optional<Theme> load_theme(`; fallback block after `t.ogm.geometric = t.ogm.dynamic;` in `kFallbackTheme()`)
- Modify: `overlume/src/theme_transition.cpp` (`blend()`, after the `out.environment.tile_radius_m =` statement, before `return out;`)
- Modify: `overlume/assets/themes/dark_adas.yaml` (append at end, after the `ogm:` block)
- Modify: `overlume/assets/themes/light_clay.yaml` (append at end, after the `ogm:` block)
- Create: `overlume/tests/fixtures/themes/height_grid_partial.yaml`
- Test: `overlume/tests/test_theme.cpp` (new tests appended at end of file; extend `ThemeLoad.BuiltinFallbackMatchesDarkAdasYaml`)
- Test: `overlume/tests/test_theme_transition.cpp` (extend `MakeSentinelTheme` and `SentinelThemesDetectAnyUnblendedField`; new tests appended at end of file)

No CMake change: `overlume/CMakeLists.txt` globs `tests/*.cpp` (`CONFIGURE_DEPENDS`) and each test file is its own executable (`test_theme`, `test_theme_transition`); theme assets are read from the source tree via `OVERLUME_DEFAULT_THEME_DIR`.

**Interfaces:**
Consumes: `detail::Float3`, `detail::Theme`, `blend_color(const Float3&, const Float3&, float)` and `lerpf` (theme_transition.cpp), `kThemeDir` (defined in test_theme.cpp / test_theme_transition.cpp, already in use), `OVERLUME_TEST_DATA_DIR`.
Produces (namespace `overlume::detail`, `overlume/src/theme.hpp`):
```cpp
struct HeightRampStop { float height_m = 0.0f; Float3 color; };
struct HeightRampTable {
    static constexpr size_t kEntries = 256;
    std::array<Float3, kEntries> color{};
    float min_m = 0.0f;
    float max_m = 1.0f;
};
// Theme gains:
struct HeightGrid {
    std::vector<HeightRampStop> ramp;
    Float3 unknown_color;
    float roughness = 0.9f;
    float ground_bias_m = -0.05f;
} height_grid;
HeightRampTable bake_height_ramp(std::vector<HeightRampStop> stops);
```

- [ ] **Step 1: Write the failing tests**

(a) Create the partial-block fixture (a copy of `ogm_ramp.yaml`, which has a full palette including `alert.warning`, plus a `height_grid` block with roughness and bias but no ramp or unknown_color):

```bash
cd $WORKTREE/overlume/tests/fixtures/themes
cp ogm_ramp.yaml height_grid_partial.yaml
sed -i 's/^name: ogm_ramp$/name: height_grid_partial/' height_grid_partial.yaml
cat >> height_grid_partial.yaml <<'EOF'
height_grid: { roughness: 0.4, ground_bias_m: 0.2 }
EOF
head -1 height_grid_partial.yaml   # expect: name: height_grid_partial
```

(b) Append to the end of `overlume/tests/test_theme.cpp`, after the closing brace of `TEST(ThemePalette, HybridSplatSizeParsesFromBothThemesAndDefaultsWhenMissing)`, which is the last test in the file:

```cpp
TEST(ThemeHeightGrid, ShippedThemesAuthorTheSpecRampAndTokens) {
    for (const char* name : {"dark_adas", "light_clay"}) {
        const std::optional<overlume::detail::Theme> t =
            overlume::detail::load_theme(kThemeDir, name);
        ASSERT_TRUE(t.has_value()) << name;
        const auto& hg = t->height_grid;
        ASSERT_EQ(hg.ramp.size(), 5u) << name;
        const float heights[] = {-1.0f, 0.0f, 0.5f, 1.5f, 2.5f};
        for (size_t i = 0; i < 5; ++i) {
            EXPECT_FLOAT_EQ(hg.ramp[i].height_m, heights[i]) << name << " stop " << i;
        }
        EXPECT_NEAR(hg.ramp[1].color.r, t->palette.ground.r, 1e-4f)
            << name << ": the 0 m stop is the theme's own ground colour";
        EXPECT_NEAR(hg.ramp[1].color.g, t->palette.ground.g, 1e-4f) << name;
        EXPECT_NEAR(hg.ramp[1].color.b, t->palette.ground.b, 1e-4f) << name;
        EXPECT_NEAR(hg.unknown_color.r, t->palette.ground.r, 1e-4f) << name;
        EXPECT_NEAR(hg.unknown_color.g, t->palette.ground.g, 1e-4f) << name;
        EXPECT_NEAR(hg.unknown_color.b, t->palette.ground.b, 1e-4f) << name;
        EXPECT_NEAR(hg.roughness, 0.9f, 1e-6f) << name;
        EXPECT_NEAR(hg.ground_bias_m, -0.05f, 1e-6f) << name;
    }
    const auto dark = overlume::detail::load_theme(kThemeDir, "dark_adas");
    const auto light = overlume::detail::load_theme(kThemeDir, "light_clay");
    ASSERT_TRUE(dark.has_value());
    ASSERT_TRUE(light.has_value());
    EXPECT_NEAR(dark->height_grid.ramp[4].color.r, 0.85f, 1e-4f);
    EXPECT_NEAR(dark->height_grid.ramp[0].color.b, 0.24f, 1e-4f);
    EXPECT_NEAR(light->height_grid.ramp[4].color.r, 0.95f, 1e-4f);
    EXPECT_NEAR(light->height_grid.ramp[0].color.b, 0.72f, 1e-4f);
}

TEST(ThemeHeightGrid, AbsentBlockFallsBackToGroundToWarningRamp) {
    const std::string fixtureDir = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
    const std::optional<overlume::detail::Theme> t =
        overlume::detail::load_theme(fixtureDir, "sun_dir_a");
    ASSERT_TRUE(t.has_value());
    const auto& hg = t->height_grid;
    ASSERT_EQ(hg.ramp.size(), 2u);
    EXPECT_FLOAT_EQ(hg.ramp[0].height_m, 0.0f);
    EXPECT_FLOAT_EQ(hg.ramp[1].height_m, 2.5f);
    EXPECT_FLOAT_EQ(hg.ramp[0].color.r, t->palette.ground.r);
    EXPECT_FLOAT_EQ(hg.ramp[0].color.g, t->palette.ground.g);
    EXPECT_FLOAT_EQ(hg.ramp[0].color.b, t->palette.ground.b);
    EXPECT_FLOAT_EQ(hg.ramp[1].color.r, t->palette.alert.warning.r);
    EXPECT_FLOAT_EQ(hg.ramp[1].color.g, t->palette.alert.warning.g);
    EXPECT_FLOAT_EQ(hg.ramp[1].color.b, t->palette.alert.warning.b);
    EXPECT_FLOAT_EQ(hg.unknown_color.r, t->palette.ground.r);
    EXPECT_FLOAT_EQ(hg.unknown_color.g, t->palette.ground.g);
    EXPECT_FLOAT_EQ(hg.unknown_color.b, t->palette.ground.b);
    EXPECT_FLOAT_EQ(hg.roughness, 0.9f);
    EXPECT_FLOAT_EQ(hg.ground_bias_m, -0.05f);
}

TEST(ThemeHeightGrid, PartialBlockKeepsExplicitTokensAndFallsBackForTheRest) {
    const std::string fixtureDir = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
    const std::optional<overlume::detail::Theme> t =
        overlume::detail::load_theme(fixtureDir, "height_grid_partial");
    ASSERT_TRUE(t.has_value());
    const auto& hg = t->height_grid;
    EXPECT_FLOAT_EQ(hg.roughness, 0.4f);
    EXPECT_FLOAT_EQ(hg.ground_bias_m, 0.2f);
    ASSERT_EQ(hg.ramp.size(), 2u) << "a block without a ramp gets the ground-to-warning pair";
    EXPECT_FLOAT_EQ(hg.ramp[1].color.r, t->palette.alert.warning.r);
    EXPECT_FLOAT_EQ(hg.unknown_color.g, t->palette.ground.g);
}

TEST(ThemeHeightGrid, BakeInterpolatesLinearlyBetweenStops) {
    const auto t = overlume::detail::bake_height_ramp(
        {{0.0f, {0.0f, 0.0f, 0.0f}}, {1.0f, {1.0f, 0.5f, 0.0f}}});
    EXPECT_FLOAT_EQ(t.min_m, 0.0f);
    EXPECT_FLOAT_EQ(t.max_m, 1.0f);
    EXPECT_FLOAT_EQ(t.color[0].r, 0.0f);
    EXPECT_NEAR(t.color[51].r, 51.0f / 255.0f, 1e-5f);
    EXPECT_NEAR(t.color[51].g, 0.5f * 51.0f / 255.0f, 1e-5f);
    EXPECT_NEAR(t.color[255].r, 1.0f, 1e-6f);
    EXPECT_NEAR(t.color[255].g, 0.5f, 1e-6f);
}

TEST(ThemeHeightGrid, BakeSortsStopsAndMapsTheStopRangeToTheTable) {
    const auto t = overlume::detail::bake_height_ramp(
        {{3.0f, {1.0f, 0.0f, 0.0f}}, {-1.0f, {0.0f, 0.0f, 1.0f}}, {1.0f, {0.0f, 1.0f, 0.0f}}});
    EXPECT_FLOAT_EQ(t.min_m, -1.0f);
    EXPECT_FLOAT_EQ(t.max_m, 3.0f);
    EXPECT_FLOAT_EQ(t.color[0].b, 1.0f) << "first entry is the lowest stop";
    EXPECT_NEAR(t.color[255].r, 1.0f, 1e-6f) << "last entry is the highest stop";
    // The middle stop (1 m) sits at k = (1 - -1) / 4 * 255 = 127.5: both neighbours are
    // within half a step of pure green.
    EXPECT_GT(t.color[127].g, 0.99f);
    EXPECT_GT(t.color[128].g, 0.99f);
}

TEST(ThemeHeightGrid, BakeClampsBeyondTheLastStopAndHandlesASingleStop) {
    const auto one = overlume::detail::bake_height_ramp({{2.0f, {0.2f, 0.4f, 0.6f}}});
    EXPECT_FLOAT_EQ(one.min_m, 2.0f);
    EXPECT_NEAR(one.max_m, 2.001f, 1e-6f) << "a lone stop still gets a non-degenerate range";
    for (size_t k : {0u, 1u, 128u, 255u}) {
        EXPECT_FLOAT_EQ(one.color[k].r, 0.2f) << k;
        EXPECT_FLOAT_EQ(one.color[k].g, 0.4f) << k;
        EXPECT_FLOAT_EQ(one.color[k].b, 0.6f) << k;
    }
}

TEST(ThemeHeightGrid, BakeOfNoStopsIsAllBlackWithUnitRange) {
    const auto t = overlume::detail::bake_height_ramp({});
    EXPECT_FLOAT_EQ(t.min_m, 0.0f);
    EXPECT_FLOAT_EQ(t.max_m, 1.0f);
    for (const auto& c : t.color) {
        EXPECT_FLOAT_EQ(c.r, 0.0f);
        EXPECT_FLOAT_EQ(c.g, 0.0f);
        EXPECT_FLOAT_EQ(c.b, 0.0f);
    }
}
```

(c) In `ThemeLoad.BuiltinFallbackMatchesDarkAdasYaml` (`overlume/tests/test_theme.cpp`), insert directly after the test's last line, which is the following (about line 417), and before the test's closing `}`. The lambda `near3` is already defined in that test:

```cpp
    EXPECT_NEAR(fb.ribbon.margin_local_m, dark->ribbon.margin_local_m, 1e-4f);
```

Insert after it:

```cpp
    ASSERT_EQ(fb.height_grid.ramp.size(), dark->height_grid.ramp.size());
    for (size_t i = 0; i < fb.height_grid.ramp.size(); ++i) {
        EXPECT_NEAR(fb.height_grid.ramp[i].height_m, dark->height_grid.ramp[i].height_m, 1e-4f);
        near3(fb.height_grid.ramp[i].color, dark->height_grid.ramp[i].color);
    }
    near3(fb.height_grid.unknown_color, dark->height_grid.unknown_color);
    EXPECT_NEAR(fb.height_grid.roughness, dark->height_grid.roughness, 1e-4f);
    EXPECT_NEAR(fb.height_grid.ground_bias_m, dark->height_grid.ground_bias_m, 1e-4f);
```

(d) In `overlume/tests/test_theme_transition.cpp`, extend `MakeSentinelTheme`: insert directly after `t.ogm.geometric.alpha.fill(scalar);` and before `return t;`:

```cpp
    t.height_grid.ramp = {{scalar, c}, {scalar + 1.0f, c}};
    t.height_grid.unknown_color = c;
    t.height_grid.roughness = scalar;
    t.height_grid.ground_bias_m = scalar;
```

and extend `SentinelThemesDetectAnyUnblendedField`: insert directly after the line `ExpectBetweenSentinels(mid.environment.tile_radius_m, "environment.tile_radius_m");` and before `ExpectBetweenSentinels(mid.ogm.dynamic.color[50], ...)`:

```cpp
    ASSERT_EQ(mid.height_grid.ramp.size(), 2u);
    ExpectBetweenSentinels(mid.height_grid.ramp[0].height_m, "height_grid.ramp[0].height_m");
    ExpectBetweenSentinels(mid.height_grid.ramp[0].color, "height_grid.ramp[0].color");
    ExpectBetweenSentinels(mid.height_grid.ramp[1].height_m, "height_grid.ramp[1].height_m");
    ExpectBetweenSentinels(mid.height_grid.unknown_color, "height_grid.unknown_color");
    ExpectBetweenSentinels(mid.height_grid.roughness, "height_grid.roughness");
    ExpectBetweenSentinels(mid.height_grid.ground_bias_m, "height_grid.ground_bias_m");
```

(e) Append to the end of `overlume/tests/test_theme_transition.cpp` (after the last test, which ends with `overlume::destroy_renderer(r);` and `}`):

```cpp
TEST(ThemeTransition, HeightGridMidpointBlendsRampColourHeightsAndGroundBias) {
    overlume::detail::Theme a;
    overlume::detail::Theme b;
    a.height_grid.ramp = {{0.0f, {0.0f, 0.0f, 0.0f}}, {2.0f, {0.1f, 0.2f, 0.3f}}};
    b.height_grid.ramp = {{0.0f, {1.0f, 1.0f, 1.0f}}, {4.0f, {0.9f, 0.5f, 0.1f}}};
    a.height_grid.unknown_color = {0.0f, 0.0f, 0.0f};
    b.height_grid.unknown_color = {1.0f, 1.0f, 1.0f};
    a.height_grid.roughness = 0.2f;
    b.height_grid.roughness = 0.8f;
    a.height_grid.ground_bias_m = -0.1f;
    b.height_grid.ground_bias_m = 0.3f;

    const overlume::detail::Theme mid = overlume::detail::blend(a, b, 0.5f);

    ASSERT_EQ(mid.height_grid.ramp.size(), 2u);
    EXPECT_NEAR(mid.height_grid.ramp[1].height_m, 3.0f, 1e-5f);
    const auto expect = overlume::detail::blend_color(a.height_grid.ramp[1].color,
                                                      b.height_grid.ramp[1].color, 0.5f);
    EXPECT_NEAR(mid.height_grid.ramp[1].color.r, expect.r, 1e-5f);
    EXPECT_NEAR(mid.height_grid.ramp[1].color.g, expect.g, 1e-5f);
    EXPECT_NEAR(mid.height_grid.ramp[1].color.b, expect.b, 1e-5f);
    const auto unk = overlume::detail::blend_color(a.height_grid.unknown_color,
                                                   b.height_grid.unknown_color, 0.5f);
    EXPECT_NEAR(mid.height_grid.unknown_color.r, unk.r, 1e-5f);
    EXPECT_NEAR(mid.height_grid.roughness, 0.5f, 1e-5f);
    EXPECT_NEAR(mid.height_grid.ground_bias_m, 0.1f, 1e-5f);
}

TEST(ThemeTransition, HeightGridRampsWithDifferentStopCountsSnapAtTheMidpoint) {
    overlume::detail::Theme a;
    overlume::detail::Theme b;
    a.height_grid.ramp = {{0.0f, {0.0f, 0.0f, 0.0f}}, {2.0f, {0.1f, 0.1f, 0.1f}}};
    b.height_grid.ramp = {
        {-1.0f, {1.0f, 0.0f, 0.0f}}, {0.0f, {0.0f, 1.0f, 0.0f}}, {1.0f, {0.0f, 0.0f, 1.0f}}};

    const overlume::detail::Theme early = overlume::detail::blend(a, b, 0.25f);
    ASSERT_EQ(early.height_grid.ramp.size(), 2u) << "before the midpoint a's ramp is kept";
    EXPECT_FLOAT_EQ(early.height_grid.ramp[1].height_m, 2.0f);

    const overlume::detail::Theme late = overlume::detail::blend(a, b, 0.75f);
    ASSERT_EQ(late.height_grid.ramp.size(), 3u) << "after the midpoint b's ramp is taken";
    EXPECT_FLOAT_EQ(late.height_grid.ramp[0].height_m, -1.0f);
    EXPECT_FLOAT_EQ(late.height_grid.ramp[2].color.b, 1.0f);
}
```

If `blend_color` is not visible from the test (it is declared in `theme_transition.hpp`, already included), stop and report rather than redeclaring it.

- [ ] **Step 2: Run the tests and confirm the expected failure**

A build in this worktree may already be running (colcon or cmake, often invoked with absolute paths). Wait until this prints nothing:

```bash
pgrep -af '[T]PSProjector-height-grid/(overlume|ros)/build'
```

Then:

```bash
cd $WORKTREE
cmake --toolchain "$PWD/overlume/cmake/toolchain-clang-libcxx.cmake" -S overlume -B overlume/build -DOVERLUME_ENABLE_CESIUM=ON \
  && cmake --build overlume/build -j --target test_theme test_theme_transition
```
Expected: FAIL at compile time of `test_theme` and `test_theme_transition` ("no member named 'height_grid' in 'overlume::detail::Theme'", "no member named 'bake_height_ramp'"). No library change yet.

- [ ] **Step 3: Add the types to `overlume/src/theme.hpp`**

Insert directly before the line `struct Theme {` (which follows `struct Float3 {...};`):

```cpp
struct HeightRampStop {
    float height_m = 0.0f;
    Float3 color;
};

struct HeightRampTable {
    static constexpr size_t kEntries = 256;
    std::array<Float3, kEntries> color{};
    float min_m = 0.0f;
    float max_m = 1.0f;
};

```

Insert directly after the `Environment` member, i.e. after the line `    } environment;` (the last member of `Theme`, before its closing `};`):

```cpp

    struct HeightGrid {
        std::vector<HeightRampStop> ramp;
        Float3 unknown_color;
        float roughness = 0.9f;
        float ground_bias_m = -0.05f;
    } height_grid;
```

Insert directly after `Theme::OgmRamp bake_ogm_ramp(std::vector<OgmRampStop> stops);`:

```cpp

// Sorts the stops ascending, maps [first stop height, last stop height] onto kEntries
// entries (linear between stops), and records that range. No stops -> all-black, [0, 1].
HeightRampTable bake_height_ramp(std::vector<HeightRampStop> stops);
```

(`<array>` and `<vector>` are already included in `theme.hpp`.)

- [ ] **Step 4: Parse the block in `overlume/src/theme.cpp`**

Insert directly after `    t.ogm.geometric = parse_ramp("geometric");` (and before `const YAML::Node environment = root["environment"];`):

```cpp

    const YAML::Node heightGrid = root["height_grid"];
    if (heightGrid && heightGrid["ramp"]) {
        for (const auto& s : heightGrid["ramp"]) {
            const float h = s["height_m"].as<float>();
            if (!std::isfinite(h)) continue;  // .nan/.inf would break the sort in bake_height_ramp
            t.height_grid.ramp.push_back({h, to_float3(s["color"])});
        }
    }
    if (t.height_grid.ramp.empty()) {
        t.height_grid.ramp = {{0.0f, t.palette.ground}, {2.5f, t.palette.alert.warning}};
    }
    t.height_grid.unknown_color = (heightGrid && heightGrid["unknown_color"])
                                      ? to_float3(heightGrid["unknown_color"])
                                      : t.palette.ground;
    t.height_grid.roughness = std::clamp(
        (heightGrid && heightGrid["roughness"]) ? heightGrid["roughness"].as<float>() : 0.9f, 0.0f,
        1.0f);
    const float groundBias = (heightGrid && heightGrid["ground_bias_m"])
                                 ? heightGrid["ground_bias_m"].as<float>()
                                 : -0.05f;
    t.height_grid.ground_bias_m = std::isfinite(groundBias) ? groundBias : -0.05f;
```

(`<algorithm>` and `<cmath>` are already included; `to_float3` is the existing helper used throughout `load_theme`. A ramp made only of non-finite stops ends up empty and takes the ground-to-warning fallback.)

Insert `bake_height_ramp` after the closing `}` of `bake_ogm_ramp` (the function ending with `ramp.alpha[v] = a.alpha + (b.alpha - a.alpha) * w;` / `}` / `return ramp;` / `}`), directly before `std::optional<Theme> load_theme(const std::string& dir, const std::string& name) {`:

```cpp

HeightRampTable bake_height_ramp(std::vector<HeightRampStop> stops) {
    HeightRampTable table;
    if (stops.empty()) return table;
    std::sort(stops.begin(), stops.end(), [](const HeightRampStop& a, const HeightRampStop& b) {
        return a.height_m < b.height_m;
    });
    table.min_m = stops.front().height_m;
    table.max_m = std::max(stops.back().height_m, table.min_m + 1e-3f);
    const float span = table.max_m - table.min_m;
    for (size_t k = 0; k < HeightRampTable::kEntries; ++k) {
        const float h = table.min_m + static_cast<float>(k) /
                                          static_cast<float>(HeightRampTable::kEntries - 1) * span;
        size_t hi = 0;
        while (hi < stops.size() && stops[hi].height_m < h) ++hi;
        if (hi == 0) {
            table.color[k] = stops.front().color;
        } else if (hi == stops.size()) {
            table.color[k] = stops.back().color;
        } else {
            // stops[hi - 1].height_m < h <= stops[hi].height_m, so the divisor is positive.
            const HeightRampStop& a = stops[hi - 1];
            const HeightRampStop& b = stops[hi];
            const float w = (h - a.height_m) / (b.height_m - a.height_m);
            table.color[k] = {a.color.r + (b.color.r - a.color.r) * w,
                              a.color.g + (b.color.g - a.color.g) * w,
                              a.color.b + (b.color.b - a.color.b) * w};
        }
    }
    return table;
}
```

In `kFallbackTheme()`, directly after `        t.ogm.geometric = t.ogm.dynamic;` (before `return t;`) insert (values equal `dark_adas.yaml`, spec section 8):

```cpp
        t.height_grid.ramp = {{-1.0f, {0.08f, 0.12f, 0.24f}},
                              {0.0f, {0.055f, 0.055f, 0.078f}},
                              {0.5f, {0.16f, 0.26f, 0.18f}},
                              {1.5f, {0.55f, 0.46f, 0.26f}},
                              {2.5f, {0.85f, 0.84f, 0.78f}}};
        t.height_grid.unknown_color = t.palette.ground;
        t.height_grid.roughness = 0.9f;
        t.height_grid.ground_bias_m = -0.05f;
```

- [ ] **Step 5: Blend the block in `overlume/src/theme_transition.cpp`**

In `blend()`, the function ends with:

```cpp
    out.environment.tile_radius_m =
        lerpf(a.environment.tile_radius_m, b.environment.tile_radius_m, w);

    return out;
```

Insert directly after that `out.environment.tile_radius_m = ...;` statement, before `return out;` (the `blend_ramp` lambda for `ogm` and the `hybrid_splat.size_px` lerp above it are untouched):

```cpp

    // Ramps with the same stop count cross-fade stop by stop (height lerped, colour in Oklab);
    // with different counts there is no pairing, so the ramp snaps at the midpoint.
    if (a.height_grid.ramp.size() == b.height_grid.ramp.size()) {
        out.height_grid.ramp.reserve(a.height_grid.ramp.size());
        for (size_t i = 0; i < a.height_grid.ramp.size(); ++i) {
            const HeightRampStop& sa = a.height_grid.ramp[i];
            const HeightRampStop& sb = b.height_grid.ramp[i];
            out.height_grid.ramp.push_back(
                {lerpf(sa.height_m, sb.height_m, w), blend_color(sa.color, sb.color, w)});
        }
    } else {
        out.height_grid.ramp = w >= 0.5f ? b.height_grid.ramp : a.height_grid.ramp;
    }
    out.height_grid.unknown_color =
        blend_color(a.height_grid.unknown_color, b.height_grid.unknown_color, w);
    out.height_grid.roughness = lerpf(a.height_grid.roughness, b.height_grid.roughness, w);
    out.height_grid.ground_bias_m =
        lerpf(a.height_grid.ground_bias_m, b.height_grid.ground_bias_m, w);
```

`out` is a default-constructed `Theme` in `blend()` (`Theme out;`), so the block above must, and does, assign all four height_grid fields. Confirm `w` is the clamped blend weight variable name used by the neighbouring `lerpf(..., w)` calls and that `HeightRampStop` resolves inside `overlume::detail` (it does: the file is in that namespace).

- [ ] **Step 6: Author the block in both shipped themes**

```bash
cd $WORKTREE/overlume/assets/themes
cat >> dark_adas.yaml <<'EOF'
height_grid:
  ramp:
    - { height_m: -1.0, color: [0.08, 0.12, 0.24] }
    - { height_m:  0.0, color: [0.055, 0.055, 0.078] }
    - { height_m:  0.5, color: [0.16, 0.26, 0.18] }
    - { height_m:  1.5, color: [0.55, 0.46, 0.26] }
    - { height_m:  2.5, color: [0.85, 0.84, 0.78] }
  unknown_color: [0.055, 0.055, 0.078]
  roughness: 0.9
  ground_bias_m: -0.05
EOF
cat >> light_clay.yaml <<'EOF'
height_grid:
  ramp:
    - { height_m: -1.0, color: [0.55, 0.62, 0.72] }
    - { height_m:  0.0, color: [0.762, 0.716, 0.672] }
    - { height_m:  0.5, color: [0.66, 0.70, 0.56] }
    - { height_m:  1.5, color: [0.80, 0.66, 0.46] }
    - { height_m:  2.5, color: [0.95, 0.93, 0.88] }
  unknown_color: [0.762, 0.716, 0.672]
  roughness: 0.9
  ground_bias_m: -0.05
EOF
grep -n "^  ground:" dark_adas.yaml light_clay.yaml   # confirm the 0 m stop and unknown_color equal palette.ground
```
Expected: dark_adas ground `[0.055, 0.055, 0.078]`, light_clay ground `[0.762, 0.716, 0.672]`. If either differs, copy the real palette.ground into that theme's 0.0 stop and `unknown_color` (the test asserts they match).

- [ ] **Step 7: Run the tests and confirm they pass**

```bash
cd $WORKTREE
cmake --build overlume/build -j \
  && ctest --test-dir overlume/build -R 'Theme|Oklab' --output-on-failure
```
Expected: all `Theme*` and `OklabHelpers` tests pass, including the 7 new `ThemeHeightGrid.*` tests, the 2 new `ThemeTransition.HeightGrid*` tests, the extended `ThemeLoad.BuiltinFallbackMatchesDarkAdasYaml` and `ThemeTransition.SentinelThemesDetectAnyUnblendedField`. GPU-dependent tests may report SKIPPED when no renderer can be created; that is fine.

- [ ] **Step 8: Prove the new checks fail when the change is reverted**

Revert by file copy, not `git stash` (refs/stash is shared by every worktree of this repo, and other sessions are active). Run the restore `cp` even if the build or ctest command fails or is interrupted; if the chain is killed, run the last two lines by hand before anything else.

```bash
cd $WORKTREE
cp overlume/src/theme_transition.cpp /tmp/task2_theme_transition.cpp.bak
git checkout -- overlume/src/theme_transition.cpp
cmake --build overlume/build -j --target test_theme_transition \
  && ctest --test-dir overlume/build -R 'ThemeTransition.HeightGrid|SentinelThemes'
cp /tmp/task2_theme_transition.cpp.bak overlume/src/theme_transition.cpp
git diff --stat overlume/src/theme_transition.cpp   # must show the blend insertion again
cmake --build overlume/build -j
```
Expected: with the blend code removed, `HeightGridMidpointBlends...`, `HeightGridRampsWithDifferentStopCounts...` and the sentinel test FAIL (blended ramp empty / roughness at the 0.9 default); after the restore and rebuild they pass again. The `git diff --stat` line must list `theme_transition.cpp` with the insertion; if it shows nothing, the restore did not happen, so stop and redo Step 5.

- [ ] **Step 9: Run the gate in the foreground**

`tools/ci_visual_mode.sh` derives `REPO_ROOT` from its own location, so it runs against the worktree with no edits. Stage 3 needs this checkout's own node install, which does not exist yet in the worktree (`ros/install/setup.bash` is absent). Make sure no colcon or cmake build is already running here (`pgrep -af '[T]PSProjector-height-grid/(overlume|ros)/build'` prints nothing), then build it once and run the gate:

```bash
cd $WORKTREE
pgrep -af '[T]PSProjector-height-grid/(overlume|ros)/build'   # must print nothing
test -f ros/install/setup.bash || ros/colcon_build.sh
tools/ci_visual_mode.sh
```
Expected: all 6 stages pass (POD header, library build + ctest, node colcon build + test, ws bridge pytest, golden counts, examples). No golden may change; if one fails, treat it as a finding and stop (do not overwrite goldens). Run in the foreground (a background run can be killed by a spurious low-memory guard). Do not set `CI_VISUAL_MODE_ROS_APPS_INSTALL` to anything under `$PRIMARY_CHECKOUT`.

- [ ] **Step 10: Commit**

```bash
cd $WORKTREE
git add overlume/src/theme.hpp overlume/src/theme.cpp overlume/src/theme_transition.cpp \
  overlume/assets/themes/dark_adas.yaml overlume/assets/themes/light_clay.yaml \
  overlume/tests/fixtures/themes/height_grid_partial.yaml \
  overlume/tests/test_theme.cpp overlume/tests/test_theme_transition.cpp
git commit -m "$(cat <<'EOF'
feat(theme): height_grid tokens, ramp baker, fallback and cross-fade

Adds the optional height_grid theme block (colour ramp by height above
nominal ground, unknown_color, roughness, ground_bias_m) with soft
defaults (ground-to-warning two-stop ramp, non-finite stops skipped),
bake_height_ramp() producing a 256-entry table, the block in
kFallbackTheme(), pairwise Oklab blending in theme_transition (ramps
with different stop counts snap at the midpoint), and the spec section 8
starting values in dark_adas and light_clay. The renderer does not
consume the tokens yet; they are the input for the heightfield renderer
task.

Claude-Session: https://claude.ai/code/session_018o7iLaY69qZAcNUfeD2VUi
EOF
)"
```

---

### Task 3: Heightfield renderer

Prerequisites: Task 1 (HeightGridLayer, SceneGraph.height_grids / height_grid_count, OwnedScene deep copy) and Task 2 (theme.height_grid, HeightRampStop, HeightRampTable, bake_height_ramp, fixture themes) are merged on `feat/height-grid`. This task does NOT touch the ground (Task 4) or goldens (Task 9).

Worktree rule: every command runs from `$WORKTREE`. Never touch `$PRIMARY_CHECKOUT` (another session's checkout).

**Files:**
- Create: `overlume/assets/materials/height_grid.mat`
- Create: `overlume/assets/materials/height_grid_faded.mat`
- Create: `overlume/src/height_grid.hpp`
- Create: `overlume/src/height_grid.cpp` (renderer code plus test-hook definitions at the bottom, like `ground_grid.cpp`)
- Create: `overlume/src/height_grid_test_hooks.hpp`
- Create (test): `overlume/tests/test_height_grid.cpp` (auto-globbed by `overlume/CMakeLists.txt`, target `test_height_grid`; `src/` is on the test include path, as for `ground_grid_test_hooks.hpp`)
- Modify: `docs/design/2026-10-06-height-grid-terrain-design.md` (section 5 amendment, Step 12)
- Modify: `overlume/src/renderer_internal.hpp` (add the height-grid members right after the line `std::vector<GroundGridSlot> groundGridSlots;`, which sits just before `static constexpr size_t kAlertSeverityCount = 3;`)
- Modify: `overlume/src/renderer.cpp` (anchors by quoted text; line numbers are hints from `feat/height-grid` before Tasks 1-2 land):
  - ~:16 `#include "ground_grid_test_hooks.hpp"` -> add `#include "height_grid.hpp"` after it
  - ~:73 `#include "ground_grid_filamat.h"` -> add the two `height_grid*_filamat.h` includes after it
  - `push_theme_to_scene` (~:359): insert the ramp bake and parameter push after the `for (size_t kind ...)` ground-grid ramp loop (its closing brace is just before `const detail::Float3 alertTints[...]`), before that line
  - `create_renderer` (~:847): insert material/instance/texture creation before `r->pointCloudMaterial = filament::Material::Builder()`; add one culling line right after `r->buildingMaterial->setCullingMode(filament::backend::CullingMode::NONE);` (~:899)
  - `destroy_renderer` (~:1046): teardown after `if (r->groundGridMaterial) r->engine->destroy(r->groundGridMaterial);`
  - `render_frame` (~:1143): `update_height_grids(*r, ...)` right after `update_ground_grids(*r, r->scene_buffer.active());`

`groundMaterial` / `gridMaterial` (created from `clayMaterial` / `clayFadedMaterial` at ~:796) and the hybrid stencil state applied to them by `apply_backdrop_stencil` in `hybrid_splats.cpp` are NOT touched here. The height-grid instance is deliberately not stenciled: the node masks the layer off in BOWL and HYBRID modes (later task).

**Interfaces:**

Consumes (from Tasks 1-2):
- `overlume::HeightGridLayer`, `SceneGraph::height_grids`, `SceneGraph::height_grid_count` (scene.h)
- `detail::Theme::HeightGrid { std::vector<HeightRampStop> ramp; Float3 unknown_color; float roughness; float ground_bias_m; } height_grid;`
- `detail::HeightRampTable { static constexpr size_t kEntries = 256; std::array<Float3,kEntries> color; float min_m, max_m; }` and `detail::HeightRampTable detail::bake_height_ramp(std::vector<detail::HeightRampStop>)`. `HeightRampTable` and `bake_height_ramp` are declared in `overlume/src/theme.hpp` by Task 2; `renderer_internal.hpp` already includes `theme.hpp`, so no new include is needed there. (If Task 2 put them in a different header, add `#include "<that header>"` to `renderer_internal.hpp` in Step 5.)
- fixture themes `overlume/tests/fixtures/themes/sun_dir_a.yaml` (no height_grid block: ground_bias_m -0.05, default roughness 0.9 from Task 2's soft default / spec section 8) and `height_grid_partial.yaml` (`height_grid: { roughness: 0.4, ground_bias_m: 0.2 }`)
- existing: `fill_tangent_frames`, `destroy_mesh`, `Mesh`, `Vertex` (renderer_internal.hpp), `detail::SceneBuffer::staleness_alpha`, `kStaleFadeStartSec` (0.5), `kStaleFadeTimeoutSec` (1.0), `overlume::set_theme(r, name, at_sec, transition_sec)` (note: `transition_sec <= 0` is replaced by 0.8 s, so tests that need an instant switch start the transition in the past with a short duration)

Produces:
```cpp
// overlume/src/height_grid.hpp  (namespace overlume)
void update_height_grids(VisualRenderer& r, const SceneGraph& scene);
void apply_height_grid_params(filament::MaterialInstance& inst, const VisualRenderer& r,
                              const detail::Theme::HeightGrid& g);
void destroy_height_grid_slots(VisualRenderer& r);

// VisualRenderer additions (renderer_internal.hpp)
struct HeightGridSlot { Mesh mesh; filament::MaterialInstance* fadeInstance = nullptr;
    uint32_t width = 0, height = 0; double resolution = 0, yaw = 0; Vec3 origin{};
    double lastUpdateSec = -1; uint32_t uploadCount = 0; int materialState = 0;
    std::vector<float> cpuPositions; std::vector<float> cpuCustom; bool inScene = false;
    float alpha = 0.0f; float groundBias = 0.0f; };
std::vector<HeightGridSlot> heightGridSlots;
filament::Material* heightGridMaterial; filament::Material* heightGridFadedMaterial;
filament::MaterialInstance* heightGridInstance;
filament::Texture* heightGridRampTexture; detail::HeightRampTable heightGridRamp;

// overlume/src/height_grid_test_hooks.hpp  (namespace overlume::testing)
size_t   height_grid_slot_count(overlume::VisualRenderer* r);
uint32_t height_grid_vertex_count(overlume::VisualRenderer* r, size_t slot);
uint32_t height_grid_index_count(overlume::VisualRenderer* r, size_t slot);
bool     height_grid_vertex(overlume::VisualRenderer* r, size_t slot, uint32_t i, uint32_t j,
                            float out_pos[3], float out_custom[2]);
uint32_t height_grid_upload_count(overlume::VisualRenderer* r, size_t slot);
int      height_grid_material_state(overlume::VisualRenderer* r, size_t slot);
float    height_grid_fade_roughness(overlume::VisualRenderer* r, size_t slot);
```
(`ground_hole_state` is added to the hooks header by Task 4.) The material-state and vertex-position hooks read back what Filament actually holds (bound material instance, entity transform), not the slot's bookkeeping fields, so reverting the bind / scene add-remove / transform calls fails the tests (same convention as `hybrid_splats.cpp`: the hook reports what was applied, not what was intended). `slot.materialState` and `slot.origin` remain as bookkeeping only.

---

- [ ] **Step 1: Write the failing test**

Create `overlume/tests/test_height_grid.cpp`:

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume/api.h"
#include "overlume/scene.h"

#include "height_grid_test_hooks.hpp"
#include "test_paths.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace {

// theme.height_grid.ground_bias_m of the shipped themes (spec section 8).
constexpr float kBiasM = -0.05f;

void render_once(overlume::VisualRenderer* r) {
    std::vector<uint8_t> pixels(320u * 240u * 3u);
    overlume::FrameView view{pixels.data(), 320, 240};
    overlume::CameraPose pose{{-8, -8, 6}, {0, 0, 0}, 60.0};
    EXPECT_TRUE(overlume::render_frame(r, pose, view));
}

overlume::HeightGridLayer make_layer(uint32_t w, uint32_t h, const std::vector<float>& heights,
                                     double last_update_sec) {
    overlume::HeightGridLayer l{};
    l.origin = {0.0, 0.0, 0.0};
    l.yaw_rad = 0.0;
    l.resolution_m = 1.0;
    l.width_cells = w;
    l.height_cells = h;
    l.heights_m = heights.data();
    l.last_update_sec = last_update_sec;
    return l;
}

overlume::SceneGraph make_scene(const overlume::HeightGridLayer* layers, uint32_t n,
                                double sim_time_sec) {
    overlume::SceneGraph s{};
    s.sim_time_sec = sim_time_sec;
    s.height_grids = layers;
    s.height_grid_count = n;
    return s;
}

overlume::VisualRenderer* make_renderer() {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, "dark_adas"};
    return overlume::create_renderer(cfg);
}

// Renderer on the Task 2 fixture themes: sun_dir_a has ground_bias_m -0.05 (block absent),
// height_grid_partial has ground_bias_m 0.2 and roughness 0.4.
overlume::VisualRenderer* make_fixture_renderer() {
    const std::string fixtureDir = std::string(OVERLUME_TEST_DATA_DIR) + "/tests/fixtures/themes";
    overlume::RenderConfig cfg{320, 240, 1, fixtureDir.c_str(), "sun_dir_a"};
    return overlume::create_renderer(cfg);
}

}

TEST(HeightGrid, VertexAndIndexCountsFollowDimensions) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<float> a(5 * 4, 0.0f);
    overlume::HeightGridLayer layer = make_layer(5, 4, a, 1.0);
    overlume::SceneGraph s = make_scene(&layer, 1, 1.0);
    overlume::set_scene(r, s);
    render_once(r);
    ASSERT_EQ(overlume::testing::height_grid_slot_count(r), 1u);
    EXPECT_EQ(overlume::testing::height_grid_vertex_count(r, 0), 20u);
    EXPECT_EQ(overlume::testing::height_grid_index_count(r, 0), 6u * 4u * 3u);

    std::vector<float> b(3 * 3, 0.0f);
    layer = make_layer(3, 3, b, 2.0);
    s = make_scene(&layer, 1, 2.0);
    overlume::set_scene(r, s);
    render_once(r);
    EXPECT_EQ(overlume::testing::height_grid_vertex_count(r, 0), 9u);
    EXPECT_EQ(overlume::testing::height_grid_index_count(r, 0), 6u * 2u * 2u);
    overlume::destroy_renderer(r);
}

TEST(HeightGrid, KnownAndUnknownVertexHeightAndCustomAttribute) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<float> heights(3 * 3, 0.25f);
    heights[1 * 3 + 1] = 1.5f;
    heights[0 * 3 + 2] = std::numeric_limits<float>::quiet_NaN();
    overlume::HeightGridLayer layer = make_layer(3, 3, heights, 1.0);
    layer.origin = {10.0, -5.0, 2.0};
    layer.resolution_m = 0.5;
    overlume::SceneGraph s = make_scene(&layer, 1, 1.0);
    overlume::set_scene(r, s);
    render_once(r);

    // Positions come back through the renderable's TransformManager transform (origin) plus the
    // uploaded relative position, so a missing setTransform fails here.
    float pos[3], custom[2];
    ASSERT_TRUE(overlume::testing::height_grid_vertex(r, 0, 1, 1, pos, custom));
    EXPECT_NEAR(pos[0], 10.75f, 1e-4f);
    EXPECT_NEAR(pos[1], -4.25f, 1e-4f);
    EXPECT_NEAR(pos[2], 2.0f + 1.5f + kBiasM, 1e-4f);
    EXPECT_NEAR(custom[0], 1.5f, 1e-6f);
    EXPECT_NEAR(custom[1], 1.0f, 1e-6f);

    ASSERT_TRUE(overlume::testing::height_grid_vertex(r, 0, 2, 0, pos, custom));
    EXPECT_NEAR(pos[0], 11.25f, 1e-4f);
    EXPECT_NEAR(pos[1], -4.75f, 1e-4f);
    EXPECT_NEAR(pos[2], 2.0f + kBiasM, 1e-4f);
    EXPECT_NEAR(custom[0], 0.0f, 1e-6f);
    EXPECT_NEAR(custom[1], 0.0f, 1e-6f);

    EXPECT_FALSE(overlume::testing::height_grid_vertex(r, 0, 3, 0, pos, custom));
    EXPECT_FALSE(overlume::testing::height_grid_vertex(r, 1, 0, 0, pos, custom));
    overlume::destroy_renderer(r);
}

TEST(HeightGrid, YawRotatesVertexPositionsAboutTheOrigin) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<float> heights(3 * 2, 0.0f);
    overlume::HeightGridLayer layer = make_layer(3, 2, heights, 1.0);
    layer.origin = {1.0, 2.0, 0.0};
    layer.yaw_rad = 1.5707963267948966;
    overlume::SceneGraph s = make_scene(&layer, 1, 1.0);
    overlume::set_scene(r, s);
    render_once(r);

    // cell (2,0) centre is (2.5, 0.5) in the grid frame -> (-0.5, 2.5) after +90 deg yaw.
    float pos[3], custom[2];
    ASSERT_TRUE(overlume::testing::height_grid_vertex(r, 0, 2, 0, pos, custom));
    EXPECT_NEAR(pos[0], 1.0f - 0.5f, 1e-4f);
    EXPECT_NEAR(pos[1], 2.0f + 2.5f, 1e-4f);
    overlume::destroy_renderer(r);
}

TEST(HeightGrid, VertexBufferIsReuploadedOnlyWhenLastUpdateChanges) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<float> heights(4 * 4, 0.5f);
    overlume::HeightGridLayer layer = make_layer(4, 4, heights, 5.0);
    overlume::SceneGraph s = make_scene(&layer, 1, 5.0);
    overlume::set_scene(r, s);
    render_once(r);
    EXPECT_EQ(overlume::testing::height_grid_upload_count(r, 0), 1u);

    render_once(r);
    overlume::set_scene(r, s);
    render_once(r);
    EXPECT_EQ(overlume::testing::height_grid_upload_count(r, 0), 1u)
        << "unchanged last_update_sec must not re-upload";

    heights.assign(heights.size(), 0.75f);
    layer.last_update_sec = 6.0;
    s.sim_time_sec = 6.0;
    overlume::set_scene(r, s);
    render_once(r);
    EXPECT_EQ(overlume::testing::height_grid_upload_count(r, 0), 2u);
    float pos[3], custom[2];
    ASSERT_TRUE(overlume::testing::height_grid_vertex(r, 0, 0, 0, pos, custom));
    EXPECT_NEAR(custom[0], 0.75f, 1e-6f);
    overlume::destroy_renderer(r);
}

// The state is read from Filament (entity in scene? which material instance is bound?), so
// removing the bind_material / scene add / scene remove calls makes this test fail.
TEST(HeightGrid, StaleFadeSwitchesOpaqueThenFadedThenRemoved) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<float> heights(3 * 3, 0.0f);
    overlume::HeightGridLayer layer = make_layer(3, 3, heights, 10.0);
    overlume::SceneGraph s = make_scene(&layer, 1, 10.0);
    overlume::set_scene(r, s);
    EXPECT_EQ(overlume::testing::height_grid_material_state(r, 0), 0);
    render_once(r);
    EXPECT_EQ(overlume::testing::height_grid_material_state(r, 0), 1);

    s.sim_time_sec = 10.75;
    overlume::set_scene(r, s);
    render_once(r);
    EXPECT_EQ(overlume::testing::height_grid_material_state(r, 0), 2);

    s.sim_time_sec = 11.5;
    overlume::set_scene(r, s);
    render_once(r);
    EXPECT_EQ(overlume::testing::height_grid_material_state(r, 0), 0);

    s.sim_time_sec = 10.0;
    overlume::set_scene(r, s);
    render_once(r);
    EXPECT_EQ(overlume::testing::height_grid_material_state(r, 0), 1)
        << "fresh data must bring the terrain back";
    EXPECT_EQ(overlume::testing::height_grid_upload_count(r, 0), 1u);
    overlume::destroy_renderer(r);
}

TEST(HeightGrid, SlotCountFollowsLayerCount) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<float> a(3 * 3, 0.0f), b(4 * 4, 1.0f);
    overlume::HeightGridLayer layers[2] = {make_layer(3, 3, a, 1.0), make_layer(4, 4, b, 1.0)};
    overlume::SceneGraph s = make_scene(layers, 2, 1.0);
    overlume::set_scene(r, s);
    render_once(r);
    ASSERT_EQ(overlume::testing::height_grid_slot_count(r), 2u);
    EXPECT_EQ(overlume::testing::height_grid_vertex_count(r, 1), 16u);

    s.height_grid_count = 1;
    overlume::set_scene(r, s);
    render_once(r);
    EXPECT_EQ(overlume::testing::height_grid_slot_count(r), 1u);
    EXPECT_EQ(overlume::testing::height_grid_vertex_count(r, 1), 0u);

    s.height_grids = nullptr;
    s.height_grid_count = 0;
    overlume::set_scene(r, s);
    render_once(r);
    EXPECT_EQ(overlume::testing::height_grid_slot_count(r), 0u);
    overlume::destroy_renderer(r);
}

// Spec section 5: a ground_bias_m change re-uploads the vertex buffer (z = h + new bias) and the
// theme push updates the per-slot faded instance too. Run once opaque, once mid-fade.
TEST(HeightGrid, ThemeSwitchChangingGroundBiasReuploads) {
    for (const double sim : {10.0, 10.75}) {
        auto* r = make_fixture_renderer();
        if (!r) GTEST_SKIP() << "no GPU/EGL";
        const bool faded = sim != 10.0;
        const int expectedState = faded ? 2 : 1;

        std::vector<float> heights(3 * 3, 0.5f);
        overlume::HeightGridLayer layer = make_layer(3, 3, heights, 10.0);
        overlume::SceneGraph s = make_scene(&layer, 1, sim);
        overlume::set_scene(r, s);
        render_once(r);

        float pos[3], custom[2];
        EXPECT_EQ(overlume::testing::height_grid_upload_count(r, 0), 1u) << "sim=" << sim;
        EXPECT_EQ(overlume::testing::height_grid_material_state(r, 0), expectedState);
        ASSERT_TRUE(overlume::testing::height_grid_vertex(r, 0, 1, 1, pos, custom));
        EXPECT_NEAR(pos[2], 0.45f, 1e-5f) << "sun_dir_a: default bias -0.05";
        const float roughnessBefore = overlume::testing::height_grid_fade_roughness(r, 0);
        if (!faded) {
            EXPECT_EQ(roughnessBefore, -1.0f) << "no per-slot faded instance while opaque";
        } else {
            // sun_dir_a has no height_grid block: the roughness default comes from Task 2 and is
            // 0.9 (spec section 8). It must differ from height_grid_partial's 0.4 by > 0.1.
            EXPECT_NEAR(roughnessBefore, 0.9f, 1e-5f)
                << "Task 2's absent-block default roughness is expected to be 0.9";
        }

        // The transition starts one second in the past and lasts 0.2 s, so it has completed when
        // the next frame applies the theme (set_theme would turn a 0 s duration into 0.8 s).
        // Same layer, same last_update_sec; the new theme has ground_bias_m 0.2, roughness 0.4.
        ASSERT_TRUE(overlume::set_theme(r, "height_grid_partial", sim - 1.0, 0.2));
        render_once(r);
        EXPECT_EQ(overlume::testing::height_grid_upload_count(r, 0), 2u)
            << "bias change must re-upload exactly once, sim=" << sim;
        ASSERT_TRUE(overlume::testing::height_grid_vertex(r, 0, 1, 1, pos, custom));
        EXPECT_NEAR(pos[2], 0.7f, 1e-5f) << "z = h + new bias, sim=" << sim;
        EXPECT_NEAR(custom[0], 0.5f, 1e-6f);
        EXPECT_EQ(overlume::testing::height_grid_material_state(r, 0), expectedState)
            << "the slot must keep its material state across the switch";

        const float roughnessAfter = overlume::testing::height_grid_fade_roughness(r, 0);
        if (!faded) {
            EXPECT_EQ(roughnessAfter, -1.0f);
        } else {
            EXPECT_NEAR(roughnessAfter, 0.4f, 1e-5f)
                << "the theme push must reach the per-slot faded instance";
            EXPECT_GT(std::fabs(roughnessAfter - roughnessBefore), 0.1f)
                << "faded-instance roughness did not change across the theme switch";
        }

        render_once(r);
        EXPECT_EQ(overlume::testing::height_grid_upload_count(r, 0), 2u)
            << "no further re-upload once the bias settled";
        overlume::destroy_renderer(r);
    }
}

// Deliberate deviation from spec section 5: the origin is NOT a re-upload trigger. It only moves
// the renderable transform; positions are stored relative to the origin. The moved position is
// read back from the TransformManager, so a missing setTransform fails here.
TEST(HeightGrid, OriginMoveWithoutNewDataDoesNotReupload) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    std::vector<float> heights(3 * 3, 0.0f);
    overlume::HeightGridLayer layer = make_layer(3, 3, heights, 5.0);
    overlume::SceneGraph s = make_scene(&layer, 1, 5.0);
    overlume::set_scene(r, s);
    render_once(r);

    float before[3], after[3], custom[2];
    ASSERT_TRUE(overlume::testing::height_grid_vertex(r, 0, 1, 1, before, custom));
    EXPECT_EQ(overlume::testing::height_grid_upload_count(r, 0), 1u);

    layer.origin = {5.0, -2.0, 0.0};
    overlume::set_scene(r, s);
    render_once(r);
    EXPECT_EQ(overlume::testing::height_grid_upload_count(r, 0), 1u)
        << "origin-only change must not re-upload";
    ASSERT_TRUE(overlume::testing::height_grid_vertex(r, 0, 1, 1, after, custom));
    EXPECT_NEAR(after[0] - before[0], 5.0f, 1e-5f);
    EXPECT_NEAR(after[1] - before[1], -2.0f, 1e-5f);
    EXPECT_NEAR(after[2] - before[2], 0.0f, 1e-5f);
    overlume::destroy_renderer(r);
}
```

- [ ] **Step 2: Run it and confirm it fails**

(A build may already be running in `overlume/build`; wait for it to finish before this step, do not kill it.)

```bash
cd $WORKTREE
cmake --toolchain "$PWD/overlume/cmake/toolchain-clang-libcxx.cmake" -S overlume -B overlume/build -DOVERLUME_ENABLE_CESIUM=ON
cmake --build overlume/build -j --target test_height_grid
```
Expected: compile failure `fatal error: 'height_grid_test_hooks.hpp' file not found` (nothing of this task exists yet).

- [ ] **Step 3: Create the two materials**

`overlume/assets/materials/height_grid.mat`:

```
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

material {
    name : height_grid,
    shadingModel : lit,
    parameters : [
        { type : sampler2d, name : rampTexture },
        { type : float, name : rampMinM },
        { type : float, name : rampMaxM },
        { type : float3, name : unknownColor },
        { type : float, name : roughness }
    ],
    requires : [ custom0 ],
    variables : [ heightKnown ]
}
vertex {
    void materialVertex(inout MaterialVertexInputs material) {
        material.heightKnown = vec4(getCustom0().xy, 0.0, 0.0);
    }
}
fragment {
    void material(inout MaterialInputs material) {
        prepareMaterial(material);
        float h = variable_heightKnown.x;
        float known = clamp(variable_heightKnown.y, 0.0, 1.0);
        float span = max(materialParams.rampMaxM - materialParams.rampMinM, 1e-4);
        float t = clamp((h - materialParams.rampMinM) / span, 0.0, 1.0);
        vec3 ramp = texture(materialParams_rampTexture, vec2((t * 255.0 + 0.5) / 256.0, 0.5)).rgb;
        material.baseColor.rgb = mix(materialParams.unknownColor, ramp, known);
        material.roughness = materialParams.roughness;
        material.metallic = 0.0;
    }
}
```

`overlume/assets/materials/height_grid_faded.mat`:

```
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

material {
    name : height_grid_faded,
    shadingModel : lit,
    parameters : [
        { type : sampler2d, name : rampTexture },
        { type : float, name : rampMinM },
        { type : float, name : rampMaxM },
        { type : float3, name : unknownColor },
        { type : float, name : roughness },
        { type : float, name : alpha }
    ],
    requires : [ custom0 ],
    variables : [ heightKnown ],
    blending : fade
}
vertex {
    void materialVertex(inout MaterialVertexInputs material) {
        material.heightKnown = vec4(getCustom0().xy, 0.0, 0.0);
    }
}
fragment {
    void material(inout MaterialInputs material) {
        prepareMaterial(material);
        float h = variable_heightKnown.x;
        float known = clamp(variable_heightKnown.y, 0.0, 1.0);
        float span = max(materialParams.rampMaxM - materialParams.rampMinM, 1e-4);
        float t = clamp((h - materialParams.rampMinM) / span, 0.0, 1.0);
        vec3 ramp = texture(materialParams_rampTexture, vec2((t * 255.0 + 0.5) / 256.0, 0.5)).rgb;
        material.baseColor.rgb = mix(materialParams.unknownColor, ramp, known) * materialParams.alpha;
        material.roughness = materialParams.roughness;
        material.metallic = 0.0;
        material.baseColor.a = materialParams.alpha;
    }
}
```
(Syntax mirrors `bowl.mat` for `requires: [custom0]` + `variables` + `variable_<name>`, and `ground_grid.mat` for the sampler ramp, the `alpha` parameter and `blending : fade` with premultiplied output. The shader samples at texel centres, so the NEAREST sampler set in Step 7 gives exact ramp entries. `overlume/CMakeLists.txt` globs `assets/materials/*.mat` with CONFIGURE_DEPENDS and generates `<name>_filamat.h` with symbols `overlume::materials::k<name>Filamat` / `...FilamatSize`, hence `kheight_gridFilamat` and `kheight_grid_fadedFilamat`.)

- [ ] **Step 4: Create `overlume/src/height_grid_test_hooks.hpp`**

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstddef>
#include <cstdint>

#include "overlume/api.h"
#include "overlume/scene.h"

namespace overlume::testing {

size_t height_grid_slot_count(overlume::VisualRenderer* r);

uint32_t height_grid_vertex_count(overlume::VisualRenderer* r, size_t slot);

uint32_t height_grid_index_count(overlume::VisualRenderer* r, size_t slot);

// World position and CUSTOM0 = (height_m, known) of cell (i, j). The position is the uploaded
// origin-relative position plus the translation Filament's TransformManager currently holds for
// the slot's entity (what was applied, not what the slot bookkeeping says).
bool height_grid_vertex(overlume::VisualRenderer* r, size_t slot, uint32_t i, uint32_t j,
                        float out_pos[3], float out_custom[2]);

uint32_t height_grid_upload_count(overlume::VisualRenderer* r, size_t slot);

// Derived from Filament state: 0 = entity not in the scene (or no renderable), 1 = the shared
// opaque instance is bound, 2 = the slot's own faded instance is bound, -1 = anything else.
int height_grid_material_state(overlume::VisualRenderer* r, size_t slot);

// "roughness" parameter read back from the slot's per-slot faded instance; -1.0 when the slot has
// no faded instance (opaque, removed, or out of range).
float height_grid_fade_roughness(overlume::VisualRenderer* r, size_t slot);

}
```

- [ ] **Step 5: Add the members to `VisualRenderer` (`overlume/src/renderer_internal.hpp`)**

Insert immediately after the line `std::vector<GroundGridSlot> groundGridSlots;` (`HeightRampTable` comes from `theme.hpp`, already included here):

```cpp
    filament::Material* heightGridMaterial = nullptr;
    filament::Material* heightGridFadedMaterial = nullptr;
    filament::MaterialInstance* heightGridInstance = nullptr;
    filament::Texture* heightGridRampTexture = nullptr;
    detail::HeightRampTable heightGridRamp;

    struct HeightGridSlot {
        Mesh mesh;
        filament::MaterialInstance* fadeInstance = nullptr;
        uint32_t width = 0, height = 0;
        double resolution = 0.0;
        double yaw = 0.0;
        Vec3 origin{};              // bookkeeping only; tests read the TransformManager
        double lastUpdateSec = -1.0;
        uint32_t uploadCount = 0;
        int materialState = 0;      // bookkeeping only; tests read the bound material instance
        std::vector<float> cpuPositions;
        std::vector<float> cpuCustom;
        bool inScene = false;
        float alpha = 0.0f;
        float groundBias = 0.0f;
    };
    std::vector<HeightGridSlot> heightGridSlots;
```

- [ ] **Step 6: Create `overlume/src/height_grid.hpp`**

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include "renderer_internal.hpp"
#include "overlume/scene.h"

namespace overlume {

void update_height_grids(VisualRenderer& r, const SceneGraph& scene);

// Sets the ramp texture, ramp span, unknown colour and roughness on a height_grid /
// height_grid_faded instance. `g` is passed explicitly because push_theme_to_scene runs before
// r.active_theme is updated during a theme transition.
void apply_height_grid_params(filament::MaterialInstance& inst, const VisualRenderer& r,
                              const detail::Theme::HeightGrid& g);

void destroy_height_grid_slots(VisualRenderer& r);

}
```

- [ ] **Step 7: Create `overlume/src/height_grid.cpp`**

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "height_grid.hpp"
#include "height_grid_test_hooks.hpp"
#include "renderer_internal.hpp"
#include "overlume/scene.h"

#include <filament/Box.h>
#include <filament/IndexBuffer.h>
#include <filament/MaterialInstance.h>
#include <filament/RenderableManager.h>
#include <filament/Scene.h>
#include <filament/TextureSampler.h>
#include <filament/TransformManager.h>
#include <filament/VertexBuffer.h>

#include <math/mat4.h>
#include <math/vec2.h>
#include <math/vec3.h>
#include <math/vec4.h>

#include <utils/EntityManager.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace overlume {

namespace {

using filament::math::float2;
using filament::math::float3;
using filament::math::float4;

// Heights are bounded by the encoding window in practice; culling is off, so this only feeds
// shadow-receiver bounds.
constexpr float kHeightGridBoundsZM = 25.0f;

struct HeightGridVertex {
    float3 position;
    float4 tangentFrame;
    float2 custom0;
};

filament::IndexBuffer* make_height_index_buffer(filament::Engine& engine, uint32_t w, uint32_t h) {
    auto* indices = new std::vector<uint32_t>();
    indices->reserve(static_cast<size_t>(w - 1) * (h - 1) * 6);
    for (uint32_t j = 0; j + 1 < h; ++j) {
        for (uint32_t i = 0; i + 1 < w; ++i) {
            const uint32_t a = j * w + i;
            const uint32_t b = a + 1;
            const uint32_t c = a + w;
            const uint32_t d = c + 1;
            indices->insert(indices->end(), {a, b, d, a, d, c});
        }
    }
    filament::IndexBuffer* ib = filament::IndexBuffer::Builder()
                                    .indexCount(static_cast<uint32_t>(indices->size()))
                                    .bufferType(filament::IndexBuffer::IndexType::UINT)
                                    .build(engine);
    ib->setBuffer(engine, filament::IndexBuffer::BufferDescriptor(
                              indices->data(), indices->size() * sizeof(uint32_t),
                              [](void*, size_t, void* user) {
                                  delete static_cast<std::vector<uint32_t>*>(user);
                              },
                              indices));
    return ib;
}

filament::VertexBuffer* make_height_vertex_buffer(filament::Engine& engine, uint32_t count) {
    return filament::VertexBuffer::Builder()
        .vertexCount(count)
        .bufferCount(1)
        .attribute(filament::VertexAttribute::POSITION, 0,
                   filament::VertexBuffer::AttributeType::FLOAT3,
                   offsetof(HeightGridVertex, position), sizeof(HeightGridVertex))
        .attribute(filament::VertexAttribute::TANGENTS, 0,
                   filament::VertexBuffer::AttributeType::FLOAT4,
                   offsetof(HeightGridVertex, tangentFrame), sizeof(HeightGridVertex))
        .attribute(filament::VertexAttribute::CUSTOM0, 0,
                   filament::VertexBuffer::AttributeType::FLOAT2,
                   offsetof(HeightGridVertex, custom0), sizeof(HeightGridVertex))
        .build(engine);
}

void upload_height_vertices(filament::Engine& engine, filament::VertexBuffer* vb,
                            std::vector<HeightGridVertex> verts) {
    auto* heap = new std::vector<HeightGridVertex>(std::move(verts));
    vb->setBufferAt(engine, 0,
                    filament::VertexBuffer::BufferDescriptor(
                        heap->data(), heap->size() * sizeof(HeightGridVertex),
                        [](void*, size_t, void* user) {
                            delete static_cast<std::vector<HeightGridVertex>*>(user);
                        },
                        heap));
}

// One vertex per cell centre, positions relative to the layer origin (the renderable's transform
// carries the origin). Fills the slot's CPU copies, which the test hooks read back.
std::vector<HeightGridVertex> build_height_vertices(VisualRenderer::HeightGridSlot& slot,
                                                    const HeightGridLayer& g, float bias) {
    const size_t w = g.width_cells;
    const size_t h = g.height_cells;
    const size_t n = w * h;
    const auto res = static_cast<float>(g.resolution_m);
    const auto c = static_cast<float>(std::cos(g.yaw_rad));
    const auto s = static_cast<float>(std::sin(g.yaw_rad));

    std::vector<float> z(n);
    slot.cpuPositions.resize(n * 3);
    slot.cpuCustom.resize(n * 2);
    for (size_t j = 0; j < h; ++j) {
        for (size_t i = 0; i < w; ++i) {
            const size_t k = j * w + i;
            const float hv = g.heights_m[k];
            const bool known = std::isfinite(hv);
            z[k] = (known ? hv : 0.0f) + bias;
            const float lx = (static_cast<float>(i) + 0.5f) * res;
            const float ly = (static_cast<float>(j) + 0.5f) * res;
            slot.cpuPositions[k * 3 + 0] = c * lx - s * ly;
            slot.cpuPositions[k * 3 + 1] = s * lx + c * ly;
            slot.cpuPositions[k * 3 + 2] = z[k];
            slot.cpuCustom[k * 2 + 0] = known ? hv : 0.0f;
            slot.cpuCustom[k * 2 + 1] = known ? 1.0f : 0.0f;
        }
    }

    std::vector<float3> normals(n);
    for (size_t j = 0; j < h; ++j) {
        const size_t j0 = j > 0 ? j - 1 : j;
        const size_t j1 = j + 1 < h ? j + 1 : j;
        for (size_t i = 0; i < w; ++i) {
            const size_t i0 = i > 0 ? i - 1 : i;
            const size_t i1 = i + 1 < w ? i + 1 : i;
            const float dzdx = (z[j * w + i1] - z[j * w + i0]) / (static_cast<float>(i1 - i0) * res);
            const float dzdy = (z[j1 * w + i] - z[j0 * w + i]) / (static_cast<float>(j1 - j0) * res);
            const float3 nl = normalize(float3{-dzdx, -dzdy, 1.0f});
            normals[j * w + i] = float3{c * nl.x - s * nl.y, s * nl.x + c * nl.y, nl.z};
        }
    }

    std::vector<Vertex> plain(n);
    fill_tangent_frames(plain, normals);

    std::vector<HeightGridVertex> verts(n);
    for (size_t k = 0; k < n; ++k) {
        verts[k].position = float3{slot.cpuPositions[k * 3 + 0], slot.cpuPositions[k * 3 + 1],
                                   slot.cpuPositions[k * 3 + 2]};
        verts[k].tangentFrame = plain[k].tangentFrame;
        verts[k].custom0 = float2{slot.cpuCustom[k * 2 + 0], slot.cpuCustom[k * 2 + 1]};
    }
    return verts;
}

void build_height_mesh(VisualRenderer& r, VisualRenderer::HeightGridSlot& slot,
                       const HeightGridLayer& g) {
    Mesh& mesh = slot.mesh;
    mesh.vertexCount = g.width_cells * g.height_cells;
    mesh.vb = make_height_vertex_buffer(*r.engine, mesh.vertexCount);
    mesh.ib = make_height_index_buffer(*r.engine, g.width_cells, g.height_cells);
    mesh.entity = utils::EntityManager::get().create();
    const auto res = static_cast<float>(g.resolution_m);
    const float reach =
        std::hypot(static_cast<float>(g.width_cells), static_cast<float>(g.height_cells)) * res +
        res;
    filament::RenderableManager::Builder(1)
        .boundingBox({{0, 0, 0}, {reach, reach, kHeightGridBoundsZM}})
        .geometry(0, filament::RenderableManager::PrimitiveType::TRIANGLES, mesh.vb, mesh.ib)
        .material(0, r.heightGridInstance)
        .culling(false)
        .castShadows(false)
        .receiveShadows(true)
        .build(*r.engine, mesh.entity);
    r.engine->getTransformManager().create(mesh.entity);
}

void clear_slot(VisualRenderer& r, VisualRenderer::HeightGridSlot& slot) {
    destroy_mesh(*r.engine, *r.scene, slot.mesh);
    if (slot.fadeInstance != nullptr) r.engine->destroy(slot.fadeInstance);
    slot = VisualRenderer::HeightGridSlot{};
}

void bind_material(VisualRenderer& r, VisualRenderer::HeightGridSlot& slot,
                   filament::MaterialInstance* mat) {
    filament::RenderableManager& rm = r.engine->getRenderableManager();
    const auto ri = rm.getInstance(slot.mesh.entity);
    if (ri.isValid()) rm.setMaterialInstanceAt(ri, 0, mat);
}

void release_fade_instance(VisualRenderer& r, VisualRenderer::HeightGridSlot& slot) {
    if (slot.fadeInstance == nullptr) return;
    bind_material(r, slot, r.heightGridInstance);
    r.engine->destroy(slot.fadeInstance);
    slot.fadeInstance = nullptr;
}

}

void apply_height_grid_params(filament::MaterialInstance& inst, const VisualRenderer& r,
                              const detail::Theme::HeightGrid& g) {
    // NEAREST like groundGridRampTexture; the shader samples at texel centres.
    inst.setParameter("rampTexture", r.heightGridRampTexture,
                      filament::TextureSampler(filament::TextureSampler::MinFilter::NEAREST,
                                               filament::TextureSampler::MagFilter::NEAREST));
    inst.setParameter("rampMinM", r.heightGridRamp.min_m);
    inst.setParameter("rampMaxM", r.heightGridRamp.max_m);
    inst.setParameter("unknownColor",
                      float3{g.unknown_color.r, g.unknown_color.g, g.unknown_color.b});
    inst.setParameter("roughness", g.roughness);
}

void destroy_height_grid_slots(VisualRenderer& r) {
    for (auto& slot : r.heightGridSlots) clear_slot(r, slot);
    r.heightGridSlots.clear();
}

void update_height_grids(VisualRenderer& r, const SceneGraph& s) {
    const uint32_t count = s.height_grids != nullptr ? s.height_grid_count : 0;
    while (r.heightGridSlots.size() > count) {
        clear_slot(r, r.heightGridSlots.back());
        r.heightGridSlots.pop_back();
    }
    if (r.heightGridSlots.size() < count) r.heightGridSlots.resize(count);

    filament::TransformManager& tm = r.engine->getTransformManager();
    const float bias = r.active_theme.height_grid.ground_bias_m;

    for (uint32_t i = 0; i < count; ++i) {
        const HeightGridLayer& g = s.height_grids[i];
        VisualRenderer::HeightGridSlot& slot = r.heightGridSlots[i];

        if (g.heights_m == nullptr || g.width_cells < 2 || g.height_cells < 2 ||
            !(g.resolution_m > 0.0)) {
            clear_slot(r, slot);
            continue;
        }

        const bool dimsChanged = slot.mesh.vb == nullptr || slot.width != g.width_cells ||
                                 slot.height != g.height_cells;
        if (dimsChanged) {
            const uint32_t uploads = slot.uploadCount;
            clear_slot(r, slot);
            slot.uploadCount = uploads;
            build_height_mesh(r, slot, g);
            slot.width = g.width_cells;
            slot.height = g.height_cells;
        }

        // The origin is not part of this gate: it only moves the renderable's transform (below).
        const bool dataChanged = dimsChanged || slot.lastUpdateSec != g.last_update_sec ||
                                 slot.resolution != g.resolution_m || slot.yaw != g.yaw_rad ||
                                 slot.groundBias != bias;
        if (dataChanged) {
            upload_height_vertices(*r.engine, slot.mesh.vb, build_height_vertices(slot, g, bias));
            slot.lastUpdateSec = g.last_update_sec;
            slot.resolution = g.resolution_m;
            slot.yaw = g.yaw_rad;
            slot.groundBias = bias;
            ++slot.uploadCount;
        }

        slot.origin = g.origin;
        const auto ti = tm.getInstance(slot.mesh.entity);
        if (ti.isValid()) {
            tm.setTransform(ti, filament::math::mat4f::translation(
                                    float3{static_cast<float>(g.origin.x),
                                           static_cast<float>(g.origin.y),
                                           static_cast<float>(g.origin.z)}));
        }

        const float alpha = detail::SceneBuffer::staleness_alpha(
            s.sim_time_sec, g.last_update_sec, kStaleFadeStartSec, kStaleFadeTimeoutSec);
        slot.alpha = alpha;

        if (alpha <= 0.0f) {
            if (slot.inScene) {
                r.scene->remove(slot.mesh.entity);
                slot.inScene = false;
            }
            release_fade_instance(r, slot);
            slot.materialState = 0;
            continue;
        }

        if (!slot.inScene) {
            r.scene->addEntity(slot.mesh.entity);
            slot.inScene = true;
        }
        if (alpha >= 1.0f) {
            release_fade_instance(r, slot);
            slot.materialState = 1;
        } else {
            if (slot.fadeInstance == nullptr) {
                slot.fadeInstance = r.heightGridFadedMaterial->createInstance();
                slot.fadeInstance->setCullingMode(filament::backend::CullingMode::NONE);
                apply_height_grid_params(*slot.fadeInstance, r, r.active_theme.height_grid);
                // ponytail: the faded renderable sits in the blended queue at default priority 4,
                // so for up to 0.5 s of stale fade it draws after (and paints over) the ground-grid
                // quads (priority 0/1) although it lies below them. Accepted transient; if it
                // shows in review, give the faded renderable priority 0 via
                // RenderableManager::setPriority.
                bind_material(r, slot, slot.fadeInstance);
            }
            slot.fadeInstance->setParameter("alpha", alpha);
            slot.materialState = 2;
        }
    }
}

}

namespace overlume::testing {

size_t height_grid_slot_count(overlume::VisualRenderer* r) {
    return r == nullptr ? 0 : r->heightGridSlots.size();
}

uint32_t height_grid_vertex_count(overlume::VisualRenderer* r, size_t slot) {
    if (r == nullptr || slot >= r->heightGridSlots.size()) return 0;
    const auto& s = r->heightGridSlots[slot];
    return s.mesh.vb == nullptr ? 0 : static_cast<uint32_t>(s.mesh.vb->getVertexCount());
}

uint32_t height_grid_index_count(overlume::VisualRenderer* r, size_t slot) {
    if (r == nullptr || slot >= r->heightGridSlots.size()) return 0;
    const auto& s = r->heightGridSlots[slot];
    return s.mesh.ib == nullptr ? 0 : static_cast<uint32_t>(s.mesh.ib->getIndexCount());
}

bool height_grid_vertex(overlume::VisualRenderer* r, size_t slot, uint32_t i, uint32_t j,
                        float out_pos[3], float out_custom[2]) {
    if (r == nullptr || out_pos == nullptr || out_custom == nullptr ||
        slot >= r->heightGridSlots.size()) {
        return false;
    }
    const auto& s = r->heightGridSlots[slot];
    if (i >= s.width || j >= s.height || s.mesh.entity.isNull()) return false;
    const size_t k = static_cast<size_t>(j) * s.width + i;
    if (k * 3 + 2 >= s.cpuPositions.size() || k * 2 + 1 >= s.cpuCustom.size()) return false;
    // Origin comes from Filament's TransformManager, not from slot.origin.
    auto& tm = r->engine->getTransformManager();
    const auto ti = tm.getInstance(s.mesh.entity);
    if (!ti.isValid()) return false;
    const filament::math::mat4f m = tm.getTransform(ti);
    for (int c = 0; c < 3; ++c) out_pos[c] = s.cpuPositions[k * 3 + c] + m[3][c];
    out_custom[0] = s.cpuCustom[k * 2 + 0];
    out_custom[1] = s.cpuCustom[k * 2 + 1];
    return true;
}

uint32_t height_grid_upload_count(overlume::VisualRenderer* r, size_t slot) {
    if (r == nullptr || slot >= r->heightGridSlots.size()) return 0;
    return r->heightGridSlots[slot].uploadCount;
}

int height_grid_material_state(overlume::VisualRenderer* r, size_t slot) {
    if (r == nullptr || slot >= r->heightGridSlots.size()) return 0;
    const auto& s = r->heightGridSlots[slot];
    if (s.mesh.entity.isNull() || !r->scene->hasEntity(s.mesh.entity)) return 0;
    auto& rm = r->engine->getRenderableManager();
    const auto ri = rm.getInstance(s.mesh.entity);
    if (!ri.isValid()) return 0;
    const filament::MaterialInstance* bound = rm.getMaterialInstanceAt(ri, 0);
    if (bound == r->heightGridInstance) return 1;
    if (bound != nullptr && bound == s.fadeInstance) return 2;
    return -1;
}

float height_grid_fade_roughness(overlume::VisualRenderer* r, size_t slot) {
    if (r == nullptr || slot >= r->heightGridSlots.size()) return -1.0f;
    const auto& s = r->heightGridSlots[slot];
    return s.fadeInstance != nullptr ? s.fadeInstance->getParameter<float>("roughness") : -1.0f;
}

}
```

- [ ] **Step 8: Wire `renderer.cpp`**

8a. Includes. After the line `#include "ground_grid_test_hooks.hpp"` add `#include "height_grid.hpp"`. After the line `#include "ground_grid_filamat.h"` add:
```cpp
#include "height_grid_filamat.h"
#include "height_grid_faded_filamat.h"
```

8b. `push_theme_to_scene`: insert immediately after the closing brace of the `for (size_t kind = 0; kind < VisualRenderer::kGroundGridKindCount; ++kind) { ... }` loop (the one ending with `texels));` / `}`), before `const detail::Float3 alertTints[VisualRenderer::kAlertSeverityCount] = {`:
```cpp
    r.heightGridRamp = detail::bake_height_ramp(theme.height_grid.ramp);
    {
        constexpr size_t kRampEntries = detail::HeightRampTable::kEntries;
        auto* texels = new std::vector<float>(kRampEntries * 4);
        for (size_t k = 0; k < kRampEntries; ++k) {
            (*texels)[k * 4 + 0] = r.heightGridRamp.color[k].r;
            (*texels)[k * 4 + 1] = r.heightGridRamp.color[k].g;
            (*texels)[k * 4 + 2] = r.heightGridRamp.color[k].b;
            (*texels)[k * 4 + 3] = 1.0f;
        }
        r.heightGridRampTexture->setImage(
            *r.engine, 0,
            filament::Texture::PixelBufferDescriptor(
                texels->data(), texels->size() * sizeof(float), filament::Texture::Format::RGBA,
                filament::Texture::Type::FLOAT,
                [](void*, size_t, void* user) { delete static_cast<std::vector<float>*>(user); },
                texels));
    }
    apply_height_grid_params(*r.heightGridInstance, r, theme.height_grid);
    for (auto& slot : r.heightGridSlots) {
        if (slot.fadeInstance != nullptr) {
            apply_height_grid_params(*slot.fadeInstance, r, theme.height_grid);
        }
    }
```

8c. `create_renderer`: insert immediately before `r->pointCloudMaterial = filament::Material::Builder()`:
```cpp
    r->heightGridMaterial = filament::Material::Builder()
                                .package(overlume::materials::kheight_gridFilamat,
                                         overlume::materials::kheight_gridFilamatSize)
                                .build(*engine);
    r->heightGridFadedMaterial = filament::Material::Builder()
                                     .package(overlume::materials::kheight_grid_fadedFilamat,
                                              overlume::materials::kheight_grid_fadedFilamatSize)
                                     .build(*engine);
    r->heightGridInstance = r->heightGridMaterial->createInstance();
    r->heightGridRampTexture =
        filament::Texture::Builder()
            .width(static_cast<uint32_t>(detail::HeightRampTable::kEntries))
            .height(1)
            .levels(1)
            .format(filament::Texture::InternalFormat::RGBA32F)
            .sampler(filament::Texture::Sampler::SAMPLER_2D)
            .build(*engine);

```
and right after `r->buildingMaterial->setCullingMode(filament::backend::CullingMode::NONE);` add:
```cpp
    r->heightGridInstance->setCullingMode(filament::backend::CullingMode::NONE);
```
(`push_theme_to_scene(*r, theme);` directly follows that block, so the opaque instance gets its parameters and the ramp texture its first upload there.)

8d. `destroy_renderer`: right after `if (r->groundGridMaterial) r->engine->destroy(r->groundGridMaterial);` add:
```cpp
    destroy_height_grid_slots(*r);
    if (r->heightGridInstance) r->engine->destroy(r->heightGridInstance);
    if (r->heightGridRampTexture) r->engine->destroy(r->heightGridRampTexture);
    if (r->heightGridMaterial) r->engine->destroy(r->heightGridMaterial);
    if (r->heightGridFadedMaterial) r->engine->destroy(r->heightGridFadedMaterial);
```

8e. `render_frame`: after `update_ground_grids(*r, r->scene_buffer.active());` add
```cpp
    update_height_grids(*r, r->scene_buffer.active());
```
(`apply_current_theme` runs first in `render_frame`, so `r->active_theme` already holds the new bias and the faded-instance push has happened before `update_height_grids` reads them.)

- [ ] **Step 9: Build and run the new tests (expect pass)**

```bash
cd $WORKTREE
cmake --build overlume/build -j
ctest --test-dir overlume/build -R '^HeightGrid\.' --output-on-failure
```
Expected: matc compiles both new `.mat` files at (re)configure with no error, build succeeds, the 8 `HeightGrid.*` tests PASS (or SKIPPED with "no GPU/EGL" on a headless box; on the dev box they must PASS, not skip). The `^HeightGrid\.` anchor keeps Task 1/2 tests that merely contain "HeightGrid" in their names out of the count. Prove the checks bite, reverting each after:
- comment out `++slot.uploadCount;` -> `VertexBufferIsReuploadedOnlyWhenLastUpdateChanges` fails;
- drop `|| slot.groundBias != bias` from `dataChanged` -> `ThemeSwitchChangingGroundBiasReuploads` fails (upload stays 1);
- delete the `for (auto& slot : r.heightGridSlots)` push loop in `push_theme_to_scene` -> the fade-roughness assert in `ThemeSwitchChangingGroundBiasReuploads` fails (faded iteration: roughness stays at the old theme value);
- add `|| slot.origin.x != g.origin.x` to `dataChanged` -> `OriginMoveWithoutNewDataDoesNotReupload` fails;
- delete the `bind_material(r, slot, slot.fadeInstance);` call -> `StaleFadeSwitchesOpaqueThenFadedThenRemoved` fails (hook reports -1 instead of 2);
- delete the `r.scene->remove(slot.mesh.entity);` call -> `StaleFadeSwitchesOpaqueThenFadedThenRemoved` fails (entity still in scene, state not 0);
- delete the `r.scene->addEntity(slot.mesh.entity);` call -> `StaleFadeSwitchesOpaqueThenFadedThenRemoved` fails (state stays 0 instead of 1);
- delete the `tm.setTransform(...)` call -> `OriginMoveWithoutNewDataDoesNotReupload` and `KnownAndUnknownVertexHeightAndCustomAttribute` fail.

- [ ] **Step 10: Run the existing library tests (no regression)**

```bash
cd $WORKTREE
ctest --test-dir overlume/build --output-on-failure
```
Serial, as the gate runs it (many tests open their own EGL/Filament engine; parallel runs can cause GPU contention the gate never shows). Expected: all pass; no golden moved (nothing draws unless `height_grid_count > 0`; the hybrid-splat tests and goldens are unaffected because `groundMaterial` / `gridMaterial` are untouched).

- [ ] **Step 11: Run the gate in the foreground**

`tools/ci_visual_mode.sh` derives `REPO_ROOT` from its own location, so run from the worktree it builds and tests the worktree (nothing is hard-wired to `$PRIMARY_CHECKOUT`). Stage 3 (node gtests) needs this checkout's own `ros/install/setup.bash`. If it is missing (for example after a clean), build it first with `ros/colcon_build.sh`, in the foreground; the guarded command below does that.

```bash
cd $WORKTREE
[ -f ros/install/setup.bash ] || ros/colcon_build.sh
tools/ci_visual_mode.sh
```
Expected: all 6 stages pass (POD header check unaffected: only private `src/` files changed). If `ros/colcon_build.sh` cannot run, `CI_VISUAL_MODE_ROS_APPS_INSTALL=/path/to/setup.bash` may point stage 3 at another install space, but do not use the other session's checkout for that without the maintainer's say-so.

- [ ] **Step 12: Record the origin deviation and the fade transient in the spec**

In `docs/design/2026-10-06-height-grid-terrain-design.md` section 5 "Mesh", replace the bullet text

```
- The vertex buffer is re-uploaded only when `last_update_sec`, the dimensions, the
  resolution, the origin or the yaw change. A test hook counts uploads.
```
with
```
- The vertex buffer is re-uploaded only when `last_update_sec`, the dimensions, the
  resolution, the yaw or `ground_bias_m` change. A test hook counts uploads.

> Amendment (Task 3): the layer origin is not a re-upload trigger. Vertex positions are stored
> relative to the origin and the renderable's transform carries it, so an origin-only change moves
> the mesh without touching the vertex buffer. `ground_bias_m` is a trigger, as the "Theme push"
> subsection already states. Pinned by HeightGrid.OriginMoveWithoutNewDataDoesNotReupload and
> HeightGrid.ThemeSwitchChangingGroundBiasReuploads.
>
> Known transient (Task 3): while a slot is in its stale fade (alpha in (0,1), at most 0.5 s) the
> faded terrain renderable sits in the blended queue at default priority, so it overpaints the
> ground-grid cost quads even though it lies below them (the documented Filament blended-queue
> trap). Accepted; fix by lowering the faded renderable's priority if it is ever visible in review.
```
(Edit in place with the Edit tool, locating the bullet by the text "The vertex buffer is re-uploaded only when"; the edit is docs-only. Task 4's `EnabledWithExpectedFootprintForYawedLayer` already covers a non-zero origin for the hole, so no further hand-off is needed.)

- [ ] **Step 13: Commit**

```bash
cd $WORKTREE
git add overlume/assets/materials/height_grid.mat overlume/assets/materials/height_grid_faded.mat \
  overlume/src/height_grid.hpp overlume/src/height_grid.cpp overlume/src/height_grid_test_hooks.hpp \
  overlume/src/renderer_internal.hpp overlume/src/renderer.cpp overlume/tests/test_height_grid.cpp \
  docs/design/2026-10-06-height-grid-terrain-design.md
git commit -m "feat(overlume): heightfield renderer for HeightGridLayer

Add update_height_grids: one vertex per cell centre (positions relative to the
layer origin, the renderable transform carries it), z = height + ground_bias_m,
unknown cells flat at the bias, central-difference normals through
fill_tangent_frames, CUSTOM0 = (height_m, known), uint32 index buffer rebuilt
only on dimension change, vertex re-upload gated on last_update_sec / dims /
resolution / yaw / ground_bias (not origin; spec section 5 amended). New
height_grid and height_grid_faded materials sample a 256x1 RGBA32F NEAREST ramp
baked from theme.height_grid.ramp by push_theme_to_scene, which also updates
the per-slot faded instances. Staleness follows the fresh-opaque /
stale-translucent convention: opaque shared instance at alpha 1, per-slot faded
instance between, removed from the scene at 0. Test hooks read back the bound
material instance, scene membership and entity transform from Filament rather
than slot bookkeeping, and test_height_grid.cpp covers counts, vertex z and
custom0, yaw, upload gating, the fade states, slot shrink, a theme switch
changing ground_bias and faded-instance roughness (opaque and mid-fade), and an
origin-only move. Nothing draws until a scene carries height grids, so no
golden moves.

Claude-Session: https://claude.ai/code/session_018o7iLaY69qZAcNUfeD2VUi"
```

---

### Task 4: Ground replacement inside the terrain footprint

All commands run from the worktree root. Start every shell block with `cd $WORKTREE`. Never touch `$PRIMARY_CHECKOUT`.

**Files:**
- Create: `overlume/assets/materials/ground_clay.mat` (copy of `clay.mat` + hole params + discard + no-op vertex block)
- Create: `overlume/assets/materials/ground_lines.mat` (copy of `clay_faded.mat` + hole params + discard + no-op vertex block)
- Create: `overlume/tests/test_ground_hole.cpp` (auto-globbed by the `tests/*.cpp` GLOB in `overlume/CMakeLists.txt`, `_overlume_test_srcs`; linked with `golden.cpp`; registered through `gtest_discover_tests`, so `ctest -R GroundHole` selects it)
- Modify: `overlume/src/renderer_internal.hpp` (`VisualRenderer`: after the `gridMaterial` member, add `groundClayMaterial`, `groundLinesMaterial`, `GroundHole groundHole`)
- Modify: `overlume/src/renderer.cpp` (anchor every edit by the quoted text, not by line number; line numbers are hints from `feat/height-grid`):
  - includes (~:60-76): after `#include "clay_faded_filamat.h"` add the two new `_filamat.h` headers; after `#include <math/vec3.h>` add `#include <math/vec2.h>`
  - material creation (~:796): the two lines `r->groundMaterial = r->clayMaterial->createInstance();` / `r->gridMaterial = r->clayFadedMaterial->createInstance();`
  - new `update_ground_hole` in the anonymous namespace (~:1114), directly after `update_ground_grid_transform`
  - call from `render_frame` (~:1139-1143), directly after `update_height_grids(*r, r->scene_buffer.active());` (added by Task 3 right after `update_ground_grids(...)`)
  - teardown (~:1077): directly after `if (r->gridMaterial) r->engine->destroy(r->gridMaterial);`
- Modify: `overlume/src/height_grid_test_hooks.hpp` (Task 3): declare `ground_hole_state`
- Modify: `overlume/src/height_grid.cpp` (Task 3): implement `ground_hole_state` in the `namespace overlume::testing` block at the bottom
- Not modified: `overlume/src/hybrid_splats.cpp`, `overlume/src/bowl.cpp` (they keep applying stencil state to the swapped instances; see Design decision 4)
- Test: `overlume/tests/test_ground_hole.cpp`

**Interfaces:**
Consumes (Tasks 1 and 3): `overlume::HeightGridLayer`, `SceneGraph::height_grids` / `height_grid_count`, `kSceneVersion == 9`, `update_height_grids(VisualRenderer&, const SceneGraph&)` already called in `render_frame`, hook header `height_grid_test_hooks.hpp` including `height_grid_vertex_count`. Existing on main: `detail::SceneBuffer::staleness_alpha(now, last, 0.5, 1.0)` (`scene_buffer.hpp`), `kStaleFadeStartSec` / `kStaleFadeTimeoutSec` (`renderer_internal.hpp`), `overlume::testing::hybrid_stencil_state_for_test` (`hybrid_splats_test_hooks.hpp`).
Produces:
- `VisualRenderer::groundClayMaterial`, `groundLinesMaterial` (`filament::Material*`), and
  ```cpp
  struct GroundHole { bool enabled = false; float center[2] = {0, 0}; float axis_x[2] = {1, 0}; float half_extent[2] = {0, 0}; } groundHole;
  ```
- Material parameters on `ground_clay.mat` and `ground_lines.mat`: `float2 holeCenter`, `float2 holeAxisX`, `float2 holeHalfExtent`, `float holeEnabled` (plus `baseColor`, `roughness`, `metallic`).
- `bool overlume::testing::ground_hole_state(overlume::VisualRenderer* r, float out_center[2], float out_axis_x[2], float out_half_extent[2]);` returns `enabled`; any out pointer may be null; returns false for a null renderer.

**Design decisions (read before coding)**

1. World position in the shader: use `getUserWorldPosition()`, not `getWorldPosition()`. In Filament 1.56.5, `getWorldPosition()` returns the camera-shifted rendering-space position; `getUserWorldPosition()` applies `getUserWorldFromWorldMatrix()` to get the true API-level position. The hole rectangle is in API world coordinates (same frame as `HeightGridLayer.origin` and the ego), so `getUserWorldPosition()` is the only correct choice. The ground quad's entity transform (ego snap, 2 m pitch, `update_ground_grid_transform`) is already folded into the fragment world position. The far-from-origin test proves this choice; the near-origin test alone cannot.
2. The hole is computed from `scene.height_grids[0]` in `renderer.cpp` with exactly Task 3's draw condition (`update_height_grids`): `width_cells >= 2`, `height_cells >= 2`, `resolution_m > 0`, `heights_m != nullptr`, staleness alpha `> 0` (Task 3 removes the renderable at alpha <= 0 and clears the slot when dims/heights are invalid). So "hole open" coincides with "terrain mesh is drawn" (spec section 6). A layer with no heights (OwnedScene empty copy, or a bad caller buffer) draws no terrain and therefore cuts no hole.
3. `holeEnabled == 0` must leave every existing golden pixel-identical: the branch is `if (holeEnabled > 0.5) { ... discard; }`. The shared `clay.mat` / `clay_faded.mat` are untouched (still used by every other instance; `clayFadedMaterial` may end up with no instances, but it is kept so this task changes nothing else).
4. Stencil / hybrid-splat interaction (main has the hybrid feature). `grep -rn "groundMaterial\|gridMaterial" overlume/src` on this branch lists exactly two users outside `renderer.cpp`: `hybrid_splats.cpp` lines 50-51 (`apply(r.groundMaterial, r.hybridGroundNe); apply(r.gridMaterial, r.hybridGridNe);` inside `apply_backdrop_stencil`, which calls `setStencilCompareFunction` / `setStencilReferenceValue` on each instance). That state is per `MaterialInstance`, not per `Material`. `apply_backdrop_stencil` runs when the hybrid layer toggles (`hybrid_splats.cpp:119`, `if (active != r.hybridStencilOn)`) and after a bowl re-bake (`bowl.cpp:215`); both run after the instances exist and nothing recreates them. The swap only changes which `Material` the two instances are created from, at the same place and in the same order in `create_renderer` (the instances are still created before `create_hybrid_splat_material(*r)`). So the stencil path keeps working unchanged: `apply_backdrop_stencil` records `hybridGroundNe` / `hybridGridNe` on the new instances, and `discard` only suppresses a fragment's colour/depth/stencil writes, it does not interact with the stencil compare on the backdrop instances. Step 9 runs the `HybridSplats*` suite explicitly; `HybridSplats.StencilStateFollowsTheLayer` asserts `ground_ne` / `grid_ne` through the hook on exactly these instances. Also confirm `push_theme_to_scene` (`renderer.cpp` ~:360-366) sets `baseColor`, `roughness`, `metallic` on `groundMaterial` / `gridMaterial` by name; the new materials declare exactly those, so nothing is dropped. `setCullingMode(NONE)` on both instances (~:877-878) is per-instance and unchanged.
5. Depth/structure variant: a material with no vertex code gets Filament's shared default depth/structure program, which does not contain the fragment `discard`. At quality 1 (every test and the node) SSAO is on and its structure pass would draw the full z=0 ground over the hole, shading every ditch against a phantom plane. Both materials therefore carry a no-op `vertex { void materialVertex(inout MaterialVertexInputs material) { } }` block, which makes Filament build a custom depth program that includes the discard. matc's own fragment-discard detection may already force a custom depth program; the block is kept as insurance so the result does not depend on that detection.
6. No `OVERLUME_TMP_DIR` and no `kSsimMin` on this branch. Golden actuals are written to `/tmp/<golden>_actual.png` and compared with `EXPECT_GT(ssim, 0.98)`. This task adds no golden test and promotes no golden.

- [ ] **Step 1: Write the failing test**

Create `overlume/tests/test_ground_hole.cpp`:

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume/api.h"
#include "overlume/scene.h"

#include "height_grid_test_hooks.hpp"
#include "test_paths.hpp"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <vector>

#include <gtest/gtest.h>

namespace {

constexpr uint32_t kW = 320;
constexpr uint32_t kH = 240;

overlume::VisualRenderer* make_renderer() {
    overlume::RenderConfig cfg{kW, kH, 1, kThemeDir, "light_clay"};
    return overlume::create_renderer(cfg);
}

// Renders one frame; the ground hole is recomputed inside render_frame.
std::vector<uint8_t> render_scene(overlume::VisualRenderer* r, const overlume::SceneGraph& s,
                                  const overlume::CameraPose& pose) {
    overlume::set_scene(r, s);
    std::vector<uint8_t> pixels(static_cast<size_t>(kW) * kH * 3u);
    overlume::FrameView view{pixels.data(), kW, kH};
    EXPECT_TRUE(overlume::render_frame(r, pose, view));
    return pixels;
}

overlume::SceneGraph base_scene(double now) {
    overlume::SceneGraph s{};
    s.sim_time_sec = now;
    s.ego = {{0, 0, 0}, 0.0, 0.0, 1};
    return s;
}

// A yawed 40x20 layer at 0.5 m: centre = origin + R(30 deg) * (10, 5) = (16.160254, 5.330127),
// half extents = (39/2 * 0.5, 19/2 * 0.5) = (9.75, 4.75), axis_x = (cos 30, sin 30).
struct YawedLayer {
    std::vector<float> heights = std::vector<float>(40u * 20u, 0.0f);
    overlume::HeightGridLayer layer{};
    explicit YawedLayer(double last_update_sec) {
        layer.origin = {10.0, -4.0, 0.0};
        layer.yaw_rad = 0.5235987755982988;  // 30 degrees
        layer.resolution_m = 0.5;
        layer.width_cells = 40;
        layer.height_cells = 20;
        layer.heights_m = heights.data();
        layer.last_update_sec = last_update_sec;
    }
};

// Mean absolute RGB difference over the 9x9 patch at the image centre.
double centre_patch_diff(const std::vector<uint8_t>& a, const std::vector<uint8_t>& b) {
    double diff = 0.0;
    for (uint32_t y = kH / 2 - 4; y <= kH / 2 + 4; ++y) {
        for (uint32_t x = kW / 2 - 4; x <= kW / 2 + 4; ++x) {
            for (uint32_t c = 0; c < 3; ++c) {
                const size_t i = (static_cast<size_t>(y) * kW + x) * 3u + c;
                diff += std::abs(static_cast<int>(a[i]) - static_cast<int>(b[i]));
            }
        }
    }
    return diff / (81.0 * 3.0);
}

}

TEST(GroundHole, EnabledWithExpectedFootprintForYawedLayer) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    YawedLayer l(10.0);
    overlume::SceneGraph s = base_scene(10.0);
    s.height_grids = &l.layer;
    s.height_grid_count = 1;
    render_scene(r, s, {{-14, -14, 10}, {0, 0, 0}, 60.0});

    float center[2] = {}, axis[2] = {}, half[2] = {};
    ASSERT_TRUE(overlume::testing::ground_hole_state(r, center, axis, half));
    EXPECT_NEAR(center[0], 16.160254f, 1e-4f);
    EXPECT_NEAR(center[1], 5.330127f, 1e-4f);
    EXPECT_NEAR(axis[0], 0.8660254f, 1e-5f);
    EXPECT_NEAR(axis[1], 0.5f, 1e-5f);
    EXPECT_NEAR(half[0], 9.75f, 1e-5f);
    EXPECT_NEAR(half[1], 4.75f, 1e-5f);
    overlume::destroy_renderer(r);
}

TEST(GroundHole, DisabledWithoutLayerAndAfterLayerRemoved) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const overlume::CameraPose pose{{-14, -14, 10}, {0, 0, 0}, 60.0};
    render_scene(r, base_scene(10.0), pose);
    EXPECT_FALSE(overlume::testing::ground_hole_state(r, nullptr, nullptr, nullptr));

    YawedLayer l(10.0);
    overlume::SceneGraph s = base_scene(10.0);
    s.height_grids = &l.layer;
    s.height_grid_count = 1;
    render_scene(r, s, pose);
    EXPECT_TRUE(overlume::testing::ground_hole_state(r, nullptr, nullptr, nullptr));

    render_scene(r, base_scene(10.1), pose);
    EXPECT_FALSE(overlume::testing::ground_hole_state(r, nullptr, nullptr, nullptr));
    overlume::destroy_renderer(r);
}

TEST(GroundHole, DisabledOnceFullyStale) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const overlume::CameraPose pose{{-14, -14, 10}, {0, 0, 0}, 60.0};
    YawedLayer l(10.0);
    overlume::SceneGraph s = base_scene(10.0);
    s.height_grids = &l.layer;
    s.height_grid_count = 1;

    render_scene(r, s, pose);  // age 0: alpha 1
    EXPECT_TRUE(overlume::testing::ground_hole_state(r, nullptr, nullptr, nullptr));
    s.sim_time_sec = 10.7;  // age 0.7: alpha 0.6, terrain fading, hole still open
    render_scene(r, s, pose);
    EXPECT_TRUE(overlume::testing::ground_hole_state(r, nullptr, nullptr, nullptr));
    s.sim_time_sec = 11.0;  // age 1.0: alpha 0, terrain removed, ground restored
    render_scene(r, s, pose);
    EXPECT_FALSE(overlume::testing::ground_hole_state(r, nullptr, nullptr, nullptr));
    overlume::destroy_renderer(r);
}

// Valid dims and resolution but heights_m == nullptr (what OwnedScene::assign produces for an
// empty copy, and what any caller with a bad buffer sends). Task 3 draws no terrain, so the
// ground must stay intact: no hole.
TEST(GroundHole, DisabledWhenLayerHasNoHeights) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    overlume::HeightGridLayer layer{};
    layer.origin = {0.0, 0.0, 0.0};
    layer.yaw_rad = 0.0;
    layer.resolution_m = 1.0;
    layer.width_cells = 10;
    layer.height_cells = 10;
    layer.heights_m = nullptr;
    layer.last_update_sec = 10.0;
    overlume::SceneGraph s = base_scene(10.0);
    s.height_grids = &layer;
    s.height_grid_count = 1;
    render_scene(r, s, {{-14, -14, 10}, {0, 0, 0}, 60.0});

    EXPECT_FALSE(overlume::testing::ground_hole_state(r, nullptr, nullptr, nullptr));
    EXPECT_EQ(overlume::testing::height_grid_vertex_count(r, 0), 0u);
    overlume::destroy_renderer(r);
}

// The shader discard itself: a uniform -1 m ditch is invisible under the opaque z=0 ground
// quad unless the quad has a hole. Looking straight down at the footprint centre, the pixels
// must differ from the same frame without a layer. Layer 21x21 at 1 m with origin (-10,-10)
// covers x,y in [-9.5, 10.5]; the camera ray hits (0, 0, 0), inside it. Reverting the discard
// leaves the clay ground visible and this test fails. (No ego model is loaded in tests, so
// nothing occludes the centre patch in either frame.)
TEST(GroundHole, DiscardRevealsTerrainBelowGround) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const overlume::CameraPose pose{{0.0, -0.5, 10.0}, {0, 0, 0}, 60.0};
    const std::vector<uint8_t> without = render_scene(r, base_scene(10.0), pose);

    std::vector<float> heights(21u * 21u, -1.0f);
    overlume::HeightGridLayer layer{};
    layer.origin = {-10.0, -10.0, 0.0};
    layer.yaw_rad = 0.0;
    layer.resolution_m = 1.0;
    layer.width_cells = 21;
    layer.height_cells = 21;
    layer.heights_m = heights.data();
    layer.last_update_sec = 10.0;
    overlume::SceneGraph s = base_scene(10.0);
    s.height_grids = &layer;
    s.height_grid_count = 1;
    const std::vector<uint8_t> with = render_scene(r, s, pose);

    EXPECT_GT(centre_patch_diff(with, without), 10.0)
        << "centre pixels did not change: the ground is not discarded";
    overlume::destroy_renderer(r);
}

// Same check with layer and camera ~200 m from the world origin, as on the live rig where map
// coordinates are far from 0. The camera-relative getWorldPosition() differs from the true
// position by the eye offset (~200 m), so a hole built on it lands far from the footprint and
// this test fails; getUserWorldPosition() passes. Layer 21x21 at 1 m with origin (190,190)
// covers x,y in [190.5, 210.5] (cell-centre vertices) around the centre (200.5, 200.5); the
// camera looks straight down at (200, 200). The ego sits at (200, 200) in both frames so the
// 60 m ground quad snaps under the layer and the "without" frame shows clay, not background;
// otherwise the diff would be vacuous.
TEST(GroundHole, DiscardRevealsTerrainFarFromWorldOrigin) {
    auto* r = make_renderer();
    if (!r) GTEST_SKIP() << "no GPU/EGL";

    const overlume::CameraPose pose{{200.0, 199.5, 10.0}, {200, 200, 0}, 60.0};
    overlume::SceneGraph s0 = base_scene(10.0);
    s0.ego = {{200, 200, 0}, 0.0, 0.0, 1};
    const std::vector<uint8_t> without = render_scene(r, s0, pose);

    std::vector<float> heights(21u * 21u, -1.0f);
    overlume::HeightGridLayer layer{};
    layer.origin = {190.0, 190.0, 0.0};
    layer.yaw_rad = 0.0;
    layer.resolution_m = 1.0;
    layer.width_cells = 21;
    layer.height_cells = 21;
    layer.heights_m = heights.data();
    layer.last_update_sec = 10.0;
    overlume::SceneGraph s = base_scene(10.0);
    s.ego = {{200, 200, 0}, 0.0, 0.0, 1};
    s.height_grids = &layer;
    s.height_grid_count = 1;
    const std::vector<uint8_t> with = render_scene(r, s, pose);

    EXPECT_GT(centre_patch_diff(with, without), 10.0)
        << "hole is not under the layer footprint far from the world origin";
    overlume::destroy_renderer(r);
}
```

- [ ] **Step 2: Run it and see it fail**

```
cd $WORKTREE
cmake --toolchain "$PWD/overlume/cmake/toolchain-clang-libcxx.cmake" -S overlume -B overlume/build -DOVERLUME_ENABLE_CESIUM=ON && cmake --build overlume/build -j --target test_ground_hole
```
Expected: compile failure in `test_ground_hole.cpp`: `no member named 'ground_hole_state' in namespace 'overlume::testing'`. (Do this only when no other build is running in `overlume/build`.)

- [ ] **Step 3: Create the two materials**

`clay.mat` and `clay_faded.mat` on this branch were read and match the bodies below apart from the inserted hole block. Both new files were compiled with the worktree's matc 1.56.5 (`overlume/build/_deps/filament-1.56.5/filament/bin/matc -a opengl -p desktop`) and build cleanly, including `discard` ahead of `prepareMaterial`.

`overlume/assets/materials/ground_clay.mat`:

```
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

// clay.mat plus a footprint discard: fragments whose true world xy lies inside the rotated
// rectangle (centre, axisX, half extents) are dropped while holeEnabled > 0.5, so the height-grid
// terrain replaces the flat ground. getUserWorldPosition() is the API-level world position;
// getWorldPosition() is camera-shifted in Filament 1.56 and would put the hole in the wrong place.
// The empty vertex block keeps a custom depth/structure program (which carries the discard) even
// if matc's discard detection changes; without vertex code Filament may share its default
// depth/structure program, which has no discard, and SSAO would see the full ground plane.
material {
    name : ground_clay,
    shadingModel : lit,
    parameters : [
        { type : float3, name : baseColor },
        { type : float, name : roughness },
        { type : float, name : metallic },
        { type : float2, name : holeCenter },
        { type : float2, name : holeAxisX },
        { type : float2, name : holeHalfExtent },
        { type : float, name : holeEnabled }
    ]
}
vertex {
    void materialVertex(inout MaterialVertexInputs material) {
    }
}
fragment {
    void material(inout MaterialInputs material) {
        if (materialParams.holeEnabled > 0.5) {
            float2 d = getUserWorldPosition().xy - materialParams.holeCenter;
            float u = dot(d, materialParams.holeAxisX);
            float v = dot(d, float2(-materialParams.holeAxisX.y, materialParams.holeAxisX.x));
            if (abs(u) <= materialParams.holeHalfExtent.x &&
                abs(v) <= materialParams.holeHalfExtent.y) {
                discard;
            }
        }
        prepareMaterial(material);
        material.baseColor.rgb = materialParams.baseColor;
        material.roughness = materialParams.roughness;
        material.metallic = materialParams.metallic;
    }
}
```

`overlume/assets/materials/ground_lines.mat`:

```
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

// clay_faded.mat plus the same footprint discard as ground_clay.mat, so the ground grid lines
// also vanish under the height-grid terrain. Empty vertex block: see ground_clay.mat.
material {
    name : ground_lines,
    shadingModel : lit,
    parameters : [
        { type : float3, name : baseColor },
        { type : float, name : roughness },
        { type : float, name : metallic },
        { type : float2, name : holeCenter },
        { type : float2, name : holeAxisX },
        { type : float2, name : holeHalfExtent },
        { type : float, name : holeEnabled }
    ],
    requires : [ color ],
    blending : fade
}
vertex {
    void materialVertex(inout MaterialVertexInputs material) {
    }
}
fragment {
    void material(inout MaterialInputs material) {
        if (materialParams.holeEnabled > 0.5) {
            float2 d = getUserWorldPosition().xy - materialParams.holeCenter;
            float u = dot(d, materialParams.holeAxisX);
            float v = dot(d, float2(-materialParams.holeAxisX.y, materialParams.holeAxisX.x));
            if (abs(u) <= materialParams.holeHalfExtent.x &&
                abs(v) <= materialParams.holeHalfExtent.y) {
                discard;
            }
        }
        prepareMaterial(material);
        material.baseColor.rgb = materialParams.baseColor * getColor().a;
        material.roughness = materialParams.roughness;
        material.metallic = materialParams.metallic;
        material.baseColor.a = getColor().a;
    }
}
```

The generated headers are `ground_clay_filamat.h` / `ground_lines_filamat.h` with symbols `kground_clayFilamat(Size)` / `kground_linesFilamat(Size)` (the `overlume/CMakeLists.txt` GLOB over `assets/materials/*.mat` produces them; no CMake edit needed).

- [ ] **Step 4: Add the renderer members**

In `overlume/src/renderer_internal.hpp`, after the line `filament::MaterialInstance* gridMaterial = nullptr;` insert:

```cpp
    filament::Material* groundClayMaterial = nullptr;
    filament::Material* groundLinesMaterial = nullptr;
    // Footprint of height_grids[0] in world xy; the ground and grid instances discard inside it.
    struct GroundHole {
        bool enabled = false;
        float center[2] = {0.0f, 0.0f};
        float axis_x[2] = {1.0f, 0.0f};
        float half_extent[2] = {0.0f, 0.0f};
    } groundHole;
```

- [ ] **Step 5: Build the materials and swap the instances in `renderer.cpp`**

5a. After the line `#include "clay_faded_filamat.h"` add:

```cpp
#include "ground_clay_filamat.h"
#include "ground_lines_filamat.h"
```
and after `#include <math/vec3.h>` add `#include <math/vec2.h>` (the block currently reads `mat4.h`, `vec3.h`, `vec4.h`, `quat.h`).

5b. Replace

```cpp
    r->groundMaterial = r->clayMaterial->createInstance();
    r->gridMaterial = r->clayFadedMaterial->createInstance();
```
with:

```cpp
    r->groundClayMaterial = filament::Material::Builder()
                                .package(overlume::materials::kground_clayFilamat,
                                         overlume::materials::kground_clayFilamatSize)
                                .build(*engine);
    r->groundLinesMaterial = filament::Material::Builder()
                                 .package(overlume::materials::kground_linesFilamat,
                                          overlume::materials::kground_linesFilamatSize)
                                 .build(*engine);

    r->groundMaterial = r->groundClayMaterial->createInstance();
    r->gridMaterial = r->groundLinesMaterial->createInstance();
```
Keep the position: the next lines (`r->laneMaterial = r->clayMaterial->createInstance();` ...) and the later `r->groundMaterial->setCullingMode(...NONE)` / `r->gridMaterial->setCullingMode(...NONE)` stay as they are, and both instances are still created before `create_hybrid_splat_material(*r)` (Design decision 4). No initial hole is set: Filament zero-initialises parameters, so `holeEnabled` is 0 until the first `render_frame`, which pushes it.

5c. Teardown: directly after `if (r->gridMaterial) r->engine->destroy(r->gridMaterial);` add:

```cpp
    if (r->groundClayMaterial) r->engine->destroy(r->groundClayMaterial);
    if (r->groundLinesMaterial) r->engine->destroy(r->groundLinesMaterial);
```
(Instances go before their parent material; this stays ahead of the `clayMaterial` / `clayFadedMaterial` destroys.)

- [ ] **Step 6: Compute and push the hole every frame**

In the anonymous namespace that holds `update_ground_grid_transform` (the `namespace {` just before `bool render_frame(...)`), after that function and before the closing `}` add:

```cpp
void update_ground_hole(VisualRenderer& r, const SceneGraph& s) {
    VisualRenderer::GroundHole hole{};
    if (s.height_grids != nullptr && s.height_grid_count > 0) {
        const HeightGridLayer& g = s.height_grids[0];
        const float alpha = detail::SceneBuffer::staleness_alpha(
            s.sim_time_sec, g.last_update_sec, kStaleFadeStartSec, kStaleFadeTimeoutSec);
        // Same condition under which update_height_grids draws the terrain (spec section 6:
        // the hole is open only while the terrain is). heights_m == nullptr draws nothing.
        if (g.width_cells >= 2 && g.height_cells >= 2 && g.resolution_m > 0.0 &&
            g.heights_m != nullptr && alpha > 0.0f) {
            const double c = std::cos(g.yaw_rad);
            const double sn = std::sin(g.yaw_rad);
            const double lx = 0.5 * g.width_cells * g.resolution_m;
            const double ly = 0.5 * g.height_cells * g.resolution_m;
            hole.enabled = true;
            hole.center[0] = static_cast<float>(g.origin.x + c * lx - sn * ly);
            hole.center[1] = static_cast<float>(g.origin.y + sn * lx + c * ly);
            hole.axis_x[0] = static_cast<float>(c);
            hole.axis_x[1] = static_cast<float>(sn);
            hole.half_extent[0] = static_cast<float>(0.5 * (g.width_cells - 1) * g.resolution_m);
            hole.half_extent[1] = static_cast<float>(0.5 * (g.height_cells - 1) * g.resolution_m);
        }
    }
    r.groundHole = hole;
    for (filament::MaterialInstance* m : {r.groundMaterial, r.gridMaterial}) {
        m->setParameter("holeCenter", filament::math::float2{hole.center[0], hole.center[1]});
        m->setParameter("holeAxisX", filament::math::float2{hole.axis_x[0], hole.axis_x[1]});
        m->setParameter("holeHalfExtent",
                        filament::math::float2{hole.half_extent[0], hole.half_extent[1]});
        m->setParameter("holeEnabled", hole.enabled ? 1.0f : 0.0f);
    }
}
```

This mirrors Task 3's `update_height_grids` exactly: clear slot when `heights_m == nullptr || width_cells < 2 || height_cells < 2 || !(resolution_m > 0)`, remove the renderable when alpha `<= 0`. If Task 3's final code differs, mirror it here. The centre uses `w*res/2` while the half extent uses `(w-1)/2*res`: cell-centre vertices run from `0.5*res` to `(w-0.5)*res` in local space, whose midpoint is exactly `w*res/2`, so both are consistent with Task 3's vertex layout.

In `render_frame`, directly after `update_height_grids(*r, r->scene_buffer.active());` add:

```cpp
    update_ground_hole(*r, r->scene_buffer.active());
```
(`update_ground_hole` lives in the same anonymous namespace above `render_frame`, so no declaration is needed. It runs after `update_hybrid_splats`, which only touches stencil state, so the two do not interfere.)

- [ ] **Step 7: Add the test hook**

Append to `overlume/src/height_grid_test_hooks.hpp`, inside `namespace overlume::testing`, after the existing declarations:

```cpp
// Returns whether the ground hole is enabled; outputs (any may be null) are the hole centre,
// the unit +x axis of the grid and the half extents, all in world xy metres.
bool ground_hole_state(overlume::VisualRenderer* r, float out_center[2], float out_axis_x[2],
                       float out_half_extent[2]);
```

Append to the `namespace overlume::testing { ... }` block at the bottom of `overlume/src/height_grid.cpp`:

```cpp
bool ground_hole_state(overlume::VisualRenderer* r, float out_center[2], float out_axis_x[2],
                       float out_half_extent[2]) {
    if (r == nullptr) return false;
    const auto& h = r->groundHole;
    for (size_t k = 0; k < 2; ++k) {
        if (out_center) out_center[k] = h.center[k];
        if (out_axis_x) out_axis_x[k] = h.axis_x[k];
        if (out_half_extent) out_half_extent[k] = h.half_extent[k];
    }
    return h.enabled;
}
```
Make sure `height_grid.cpp` includes `<cstddef>` (needs `size_t`; add if absent).

- [ ] **Step 8: Build and run the new tests (expected: pass), then prove the checks bite**

```
cd $WORKTREE
cmake --toolchain "$PWD/overlume/cmake/toolchain-clang-libcxx.cmake" -S overlume -B overlume/build -DOVERLUME_ENABLE_CESIUM=ON && cmake --build overlume/build -j && ctest --test-dir overlume/build -R GroundHole --output-on-failure
```
Expected: 6 tests pass (`GTEST_SKIP` with no GPU does not count as passing the pixel checks; run on the GPU/EGL box). If configure stops with "matc failed on .../ground_clay.mat" (or `ground_lines.mat`), the `.mat` body was mistyped: fix it against Step 3 and reconfigure.

Margin check (once, after the first green run): the two `DiscardReveals*` tests assume the theme's ground ramp colour at about -1 m differs from `palette.ground` by a mean of well over 10 per channel. Temporarily add `std::cout << "centre_patch_diff=" << centre_patch_diff(with, without) << "\n";` before each `EXPECT_GT` (not committed), rerun `ctest --test-dir overlume/build -R 'GroundHole.DiscardReveals' --output-on-failure -V`, and confirm both values are well above 10. If the margin is thin, change the uniform height from `-1.0f` to `+1.5f` (or another ramp stop that clearly differs from `palette.ground`) in both tests. Do not use NaN heights: unknown cells render in `palette.ground`. Remove the temporary prints afterwards.

Then, temporarily and one at a time (never committed; restore each with `git checkout -- <file>` or by undoing the edit, rebuild and rerun):
1. Comment out `discard;` in `ground_clay.mat`: `ctest --test-dir overlume/build -R 'GroundHole.DiscardReveals' --output-on-failure` must FAIL both `DiscardRevealsTerrainBelowGround` and `DiscardRevealsTerrainFarFromWorldOrigin`.
2. Replace `getUserWorldPosition()` with `getWorldPosition()` in `ground_clay.mat`: `DiscardRevealsTerrainFarFromWorldOrigin` must FAIL (the near-origin test may still pass; that is why the far test exists). If it passes, the test is vacuous: fix the scene before continuing.
3. Remove the `heights_m != nullptr` clause in `update_ground_hole`: `DisabledWhenLayerHasNoHeights` must FAIL.

- [ ] **Step 9: Hybrid stencil path, then the full library ctest (golden regression)**

9a. The stencil state that `hybrid_splats.cpp` applies to `groundMaterial` / `gridMaterial` must survive the material swap. Run the hybrid suite first and by name:

```
cd $WORKTREE
ctest --test-dir overlume/build -R 'HybridSplats' --output-on-failure
```
Expected: every `HybridSplats.*` test passes, in particular `StencilStateFollowsTheLayer` (asserts `ground_ne` and `grid_ne` through `hybrid_stencil_state_for_test` flip on and off with the splat layer, including after a bowl re-bake) and `ClearingRestoresByteIdenticalFrames` (frames with the layer cleared match frames without it, which exercises the ground/grid instances with `holeEnabled = 0`). On a GPU-less box these skip; run this step on the GPU/EGL box.

9b. Everything:

```
cd $WORKTREE
ctest --test-dir overlume/build --output-on-failure
```
Expected: all pass, in particular every existing golden under `overlume/tests/goldens/` (all 20 counted by gate stage 5, including `ogm_offroad_light_clay`, the theme showcase and the hybrid goldens). With `holeEnabled = 0` the ground and grid must render pixel-identically to before; a golden failure here is a finding, never a file to overwrite (AGENTS.md hard rule). Diff the actual PNG that the failing test wrote at `/tmp/<golden>_actual.png` (for example `/tmp/ogm_offroad_light_clay_actual.png`) against `overlume/tests/goldens/<golden>.png` before touching anything; do not promote goldens. If a golden moves only at quality 1, suspect the new depth/structure program (SSAO now sees the custom program); report it rather than overwriting.

- [ ] **Step 10: Run the gate in the foreground**

`tools/ci_visual_mode.sh` derives `REPO_ROOT` from its own location, so running it from the worktree builds and tests the worktree (nothing in it or in `docs/runbooks/ci_gate.md` is wired to `$PRIMARY_CHECKOUT`). Stage 3 needs this checkout's `ros/install/setup.bash`, which a fresh worktree does not have; build it once first (skip if `ros/install` already exists):

```
cd $WORKTREE
test -f ros/install/setup.bash || ros/colcon_build.sh
tools/ci_visual_mode.sh
```
Expected: all six stages PASS (POD header, library build + ctest, node build + test, ws bridge pytest, golden counts, examples). Run in the foreground, not the background. Alternatively point stage 3 at an existing install with `CI_VISUAL_MODE_ROS_APPS_INSTALL=/path/to/setup.bash`, but never an install under `$PRIMARY_CHECKOUT`.

- [ ] **Step 11: Commit**

```
cd $WORKTREE
git add overlume/assets/materials/ground_clay.mat overlume/assets/materials/ground_lines.mat overlume/src/renderer_internal.hpp overlume/src/renderer.cpp overlume/src/height_grid.cpp overlume/src/height_grid_test_hooks.hpp overlume/tests/test_ground_hole.cpp
git commit -m "feat(visual): ground and grid lines discard inside the height-grid footprint

Add ground_clay.mat and ground_lines.mat (copies of clay / clay_faded with a
rotated-rectangle world-xy discard driven by holeCenter, holeAxisX, holeHalfExtent
and holeEnabled; world position via getUserWorldPosition(), the true API-level
position, while getWorldPosition() is camera-shifted in Filament 1.56). Both carry
a no-op vertex block so Filament builds a custom depth/structure program that also
discards; otherwise SSAO would shade every ditch against a phantom ground plane.
groundMaterial and gridMaterial are now instances of these, created in the same
place as before so the hybrid-splat stencil state that hybrid_splats.cpp applies to
them per instance keeps working. render_frame computes the hole from
height_grids[0] after update_height_grids, enabled only while the terrain itself is
drawn (valid dims, heights_m non-null, staleness alpha above 0), and pushes it to
both instances every frame. With no layer holeEnabled is 0 and rendering is
unchanged, so no existing golden moves. Without the hole every ditch and downhill
slope would be buried under the opaque z = 0 quad. Tests cover the yawed footprint,
removal and staleness, a layer with no heights, and pixel-level discard both near
and 200 m from the world origin.

Claude-Session: https://claude.ai/code/session_018o7iLaY69qZAcNUfeD2VUi"
```

---

### Task 5: Shared grid placement helper (node refactor)

Behaviour-preserving extraction of the TF lookup / frame_id override / origin transform / NaN check / flatten_z / yaw composition out of `OgmAdapter::ingest` into a free function that Task 6's `HeightGridAdapter` reuses. `test_ogm_adapter.cpp` must pass UNCHANGED.

All commands run from the worktree root `$WORKTREE` (every block starts with `cd` there). Nothing in `tools/ci_visual_mode.sh`, `ros/colcon_build.sh` or `docs/runbooks/ci_gate.md` is hard-wired to another checkout: the gate derives `REPO_ROOT` from its own location, so running it from the worktree uses the worktree's `overlume/build` and `ros/install`.

**Prerequisites (worktree state):** `ros/install` does not exist yet in the worktree. Step 3's build fails at CMake configure by design and creates nothing; Step 8's successful `ros/colcon_build.sh overlume_ros` creates `ros/install`, which the Step 8 `source` line and gate stage 3 (Step 9) both need. The node build also needs `overlume/build/liboverlume.a` from the library build, and that archive already exists from earlier builds, so its presence does not prove the running build has finished. Before Step 3 and again before Step 9 (stage 2 of the gate runs `cmake --build overlume/build` itself) check that nothing is still building in that tree:
```bash
cd $WORKTREE
pgrep -af '[T]PSProjector-height-grid/(overlume|ros)/build' || echo idle
ls overlume/build/liboverlume.a
```
Proceed only when it prints `idle` (otherwise wait for the other build to report completion). Never start a second build into `overlume/build` while one is running.

**Files:**
- Create: `ros/src/overlume_ros/include/overlume_ros/adapters/grid_placement.hpp`
- Create: `ros/src/overlume_ros/src/adapters/grid_placement.cpp`
- Create (test): `ros/src/overlume_ros/test/test_grid_placement.cpp`
- Modify: `ros/src/overlume_ros/src/adapters/ogm.cpp` (include block; delete the `HasNan` helper; in `ingest` replace the `tf2::Transform xform;` .. `if (HasNan(originTf)) {...}` block and the `origin_ = ...` .. `yaw_rad_ = ...` lines)
- Modify: `ros/src/overlume_ros/CMakeLists.txt` (anchored by text, see Steps 2 and 7)

Note: the header lives at `include/overlume_ros/adapters/grid_placement.hpp`, which differs from the spec section 7 literal path `src/adapters/grid_placement.{hpp,cpp}` on purpose, to follow the repo convention that every adapter header (e.g. `ogm.hpp`) lives under `include/overlume_ros/adapters/`. The `.cpp` stays under `src/adapters/`.

**Interfaces:**
Consumes: `overlume::ros::FrameTransformer` (`lookup(const std_msgs::msg::Header&, tf2::Transform&) const`, `flatten_z() const`) from `overlume_ros/frame_transform.hpp`; `overlume::Vec3` from `overlume/scene.h`.
Produces:
```cpp
namespace overlume::ros {
enum class PlacementResult { kOk, kNoTf, kNonFinite };
struct GridPlacement { overlume::Vec3 origin; double yaw_rad; };
PlacementResult place_grid(const std_msgs::msg::Header& header,
                           const geometry_msgs::msg::Pose& grid_origin,
                           const std::string& frame_override,
                           const FrameTransformer& tf, GridPlacement& out);
}
```
`out` is written only on `kOk`. `frame_override` empty means keep `header.frame_id`.

Test command used in Step 8:
```bash
cd $WORKTREE
source /opt/ros/humble/setup.bash && source ros/install/setup.bash
(cd ros && colcon test --packages-select overlume_ros --ctest-args -R 'test_grid_placement|test_ogm_adapter' && colcon test-result --verbose)
```
(Run `ros/colcon_build.sh overlume_ros` before it whenever sources changed.)

- [ ] **Step 1: Write the failing test**

Create `ros/src/overlume_ros/test/test_grid_placement.cpp`:
```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/adapters/grid_placement.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <gtest/gtest.h>
#include <rclcpp/clock.hpp>
#include <tf2_ros/buffer.h>

using overlume::ros::FrameTransformer;
using overlume::ros::GridPlacement;
using overlume::ros::PlacementResult;
using overlume::ros::place_grid;

namespace {

struct TfFixture {
    std::shared_ptr<rclcpp::Clock> clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf2_ros::Buffer buffer{clock};
    FrameTransformer tf{buffer};
    FrameTransformer tf_keep_z{buffer, "map", false};

    // map <- base_link: +90 deg about z, translated 10 m along x, 1.5 m up.
    void add_base_link() {
        geometry_msgs::msg::TransformStamped t;
        t.header.frame_id = "map";
        t.child_frame_id = "base_link";
        t.transform.translation.x = 10.0;
        t.transform.translation.z = 1.5;
        t.transform.rotation.z = std::sin(M_PI / 4.0);
        t.transform.rotation.w = std::cos(M_PI / 4.0);
        buffer.setTransform(t, "test", true);
    }
};

std_msgs::msg::Header header(const std::string& frame) {
    std_msgs::msg::Header h;
    h.frame_id = frame;
    return h;
}

geometry_msgs::msg::Pose pose(double x, double y, double z, double yaw) {
    geometry_msgs::msg::Pose p;
    p.position.x = x;
    p.position.y = y;
    p.position.z = z;
    p.orientation.z = std::sin(yaw / 2.0);
    p.orientation.w = std::cos(yaw / 2.0);
    return p;
}

}

TEST(GridPlacement, MapFrameIsIdentityOnOriginAndYaw) {
    TfFixture f;
    GridPlacement out{};
    ASSERT_EQ(place_grid(header("map"), pose(1.0, 2.0, 0.0, 0.3), "", f.tf, out),
              PlacementResult::kOk);
    EXPECT_DOUBLE_EQ(out.origin.x, 1.0);
    EXPECT_DOUBLE_EQ(out.origin.y, 2.0);
    EXPECT_NEAR(out.yaw_rad, 0.3, 1e-12);
}

TEST(GridPlacement, EmptyFrameIdIsIdentity) {
    TfFixture f;
    GridPlacement out{};
    ASSERT_EQ(place_grid(header(""), pose(4.0, -5.0, 0.0, 0.0), "", f.tf, out),
              PlacementResult::kOk);
    EXPECT_DOUBLE_EQ(out.origin.x, 4.0);
    EXPECT_DOUBLE_EQ(out.origin.y, -5.0);
}

TEST(GridPlacement, FrameOverrideReplacesHeaderFrame) {
    TfFixture f;
    f.add_base_link();
    GridPlacement out{};
    // Header says "map" (identity); the override sends it through base_link.
    ASSERT_EQ(place_grid(header("map"), pose(1.0, 0.0, 0.0, 0.0), "base_link", f.tf, out),
              PlacementResult::kOk);
    EXPECT_NEAR(out.origin.x, 10.0, 1e-9);
    EXPECT_NEAR(out.origin.y, 1.0, 1e-9);
}

TEST(GridPlacement, RotatedTfComposesWithInfoOrientationIntoYaw) {
    TfFixture f;
    f.add_base_link();
    GridPlacement out{};
    ASSERT_EQ(place_grid(header("base_link"), pose(1.0, 0.0, 0.0, 0.3), "", f.tf, out),
              PlacementResult::kOk);
    EXPECT_NEAR(out.origin.x, 10.0, 1e-9);
    EXPECT_NEAR(out.origin.y, 1.0, 1e-9);
    EXPECT_NEAR(out.yaw_rad, M_PI / 2.0 + 0.3, 1e-9);
}

TEST(GridPlacement, FlattenZZeroesOriginZ) {
    TfFixture f;
    f.add_base_link();
    GridPlacement flat{};
    GridPlacement kept{};
    ASSERT_EQ(place_grid(header("base_link"), pose(0.0, 0.0, 2.0, 0.0), "", f.tf, flat),
              PlacementResult::kOk);
    ASSERT_EQ(place_grid(header("base_link"), pose(0.0, 0.0, 2.0, 0.0), "", f.tf_keep_z, kept),
              PlacementResult::kOk);
    EXPECT_DOUBLE_EQ(flat.origin.z, 0.0);
    EXPECT_NEAR(kept.origin.z, 3.5, 1e-9);
}

TEST(GridPlacement, MissingTfIsNoTfAndLeavesOutUntouched) {
    TfFixture f;
    GridPlacement out{};
    out.origin = {7.0, 7.0, 7.0};
    out.yaw_rad = 7.0;
    EXPECT_EQ(place_grid(header("nowhere"), pose(1.0, 2.0, 0.0, 0.0), "", f.tf, out),
              PlacementResult::kNoTf);
    EXPECT_DOUBLE_EQ(out.origin.x, 7.0);
    EXPECT_DOUBLE_EQ(out.yaw_rad, 7.0);
}

TEST(GridPlacement, NanOriginIsNonFinite) {
    TfFixture f;
    GridPlacement out{};
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(place_grid(header("map"), pose(nan, 0.0, 0.0, 0.0), "", f.tf, out),
              PlacementResult::kNonFinite);
}
```

- [ ] **Step 2: Register the test in CMake**

In `ros/src/overlume_ros/CMakeLists.txt`, immediately after the line `  target_link_libraries(test_ogm_adapter ${yaml_cpp_vendor_TARGETS} overlume_node_test_paths)` (end of the `test_ogm_adapter` block, inside `if(BUILD_TESTING)`, two-space indent; unique in the file) add:
```cmake

  ament_add_gtest(test_grid_placement
    test/test_grid_placement.cpp
    src/adapters/grid_placement.cpp
    src/frame_transform.cpp)
  target_include_directories(test_grid_placement PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/test
    ${OVERLUME_DIR}/include)
  ament_target_dependencies(test_grid_placement ${THIS_PACKAGE_DEPS})
  target_link_libraries(test_grid_placement ${yaml_cpp_vendor_TARGETS})
```

- [ ] **Step 3: Run it, expect failure**

First run the idle check from the Prerequisites paragraph. Then:
```bash
cd $WORKTREE
ros/colcon_build.sh overlume_ros
```
Expected: build FAILS at CMake configure/generate: `Cannot find source file: src/adapters/grid_placement.cpp`. That is the red state; it creates no install tree, and the `colcon test` command cannot run until the build succeeds (Step 8).

- [ ] **Step 4: Create the header**

`ros/src/overlume_ros/include/overlume_ros/adapters/grid_placement.hpp`:
```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/header.hpp>

#include "overlume_ros/frame_transform.hpp"
#include "overlume/scene.h"

namespace overlume::ros {

enum class PlacementResult { kOk, kNoTf, kNonFinite };

struct GridPlacement {
    overlume::Vec3 origin;
    double yaw_rad;
};

// Places an OccupancyGrid-style grid (header + info.origin pose) into the map
// frame. frame_override, when non-empty, replaces header.frame_id before the
// TF lookup. origin.z is zeroed when tf.flatten_z(). `out` is written only on
// kOk.
PlacementResult place_grid(const std_msgs::msg::Header& header,
                           const geometry_msgs::msg::Pose& grid_origin,
                           const std::string& frame_override,
                           const FrameTransformer& tf, GridPlacement& out);

}
```
(`frame_transform.hpp` does not include `<string>`/`<utility>` itself; the `<string>` include here keeps this header self-sufficient.)

- [ ] **Step 5: Create the implementation**

`ros/src/overlume_ros/src/adapters/grid_placement.cpp` (logic copied verbatim from `OgmAdapter::ingest`, lines 57-69 and 79-84 on main):
```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/adapters/grid_placement.hpp"

#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

namespace overlume::ros {
namespace {

bool HasNan(const tf2::Vector3& v) {
    return std::isnan(v.x()) || std::isnan(v.y()) || std::isnan(v.z());
}

}

PlacementResult place_grid(const std_msgs::msg::Header& header_in,
                           const geometry_msgs::msg::Pose& grid_origin,
                           const std::string& frame_override,
                           const FrameTransformer& tf, GridPlacement& out) {
    tf2::Transform xform;
    std_msgs::msg::Header header = header_in;
    if (!frame_override.empty()) header.frame_id = frame_override;
    if (!tf.lookup(header, xform)) return PlacementResult::kNoTf;

    const auto& p = grid_origin.position;
    const tf2::Vector3 originTf = xform * tf2::Vector3(p.x, p.y, p.z);
    if (HasNan(originTf)) return PlacementResult::kNonFinite;

    out.origin = {originTf.x(), originTf.y(), tf.flatten_z() ? 0.0 : originTf.z()};
    const auto& q = grid_origin.orientation;
    const tf2::Quaternion gridInMap =
        xform.getRotation() * tf2::Quaternion(q.x, q.y, q.z, q.w).normalized();
    const tf2::Vector3 xAxis = tf2::Transform(gridInMap) * tf2::Vector3(1.0, 0.0, 0.0);
    out.yaw_rad = std::atan2(xAxis.y(), xAxis.x());
    return PlacementResult::kOk;
}

}
```

- [ ] **Step 6: Refactor `OgmAdapter::ingest` to call it**

In `ros/src/overlume_ros/src/adapters/ogm.cpp`:

1. Replace the include block
```cpp
#include "overlume_ros/adapters/ogm.hpp"

#include <cmath>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
```
with
```cpp
#include "overlume_ros/adapters/ogm.hpp"
#include "overlume_ros/adapters/grid_placement.hpp"

#include <cmath>
```
(i.e. delete the two tf2 includes, which become unused because ogm.cpp no longer touches `tf2::Transform`/`Vector3`/`Quaternion`; keep `<cmath>` for `std::lround`; `ogm.hpp` and `grid_placement.hpp` bring in everything else ogm.cpp uses.)
2. Delete the helper
```cpp
bool HasNan(const tf2::Vector3& v) {
    return std::isnan(v.x()) || std::isnan(v.y()) || std::isnan(v.z());
}

```
(it sits between `KindFromRole` and `ConvertCell`; now unused here).
3. Replace the block starting `    tf2::Transform xform;` through
```cpp
    if (HasNan(originTf)) {
        ++stats_.dropped_malformed;
        return;
    }
```
with:
```cpp
    GridPlacement placement{};
    switch (place_grid(msg.header, msg.info.origin, row_.frame_id, tf_, placement)) {
        case PlacementResult::kOk: break;
        case PlacementResult::kNoTf: ++stats_.dropped_no_tf; return;
        case PlacementResult::kNonFinite: ++stats_.dropped_malformed; return;
    }
```
4. Replace these lines (directly after `cells_ = std::move(next);`):
```cpp
    origin_ = {originTf.x(), originTf.y(), tf_.flatten_z() ? 0.0 : originTf.z()};
    const auto& q = msg.info.origin.orientation;
    const tf2::Quaternion gridInMap =
        xform.getRotation() * tf2::Quaternion(q.x, q.y, q.z, q.w).normalized();
    const tf2::Vector3 xAxis = tf2::Transform(gridInMap) * tf2::Vector3(1.0, 0.0, 0.0);
    yaw_rad_ = std::atan2(xAxis.y(), xAxis.x());
```
with:
```cpp
    origin_ = placement.origin;
    yaw_rad_ = placement.yaw_rad;
```
keeping `resolution_m_ = msg.info.resolution;` onwards unchanged. Order of side effects is preserved: placement failures still return before the cell conversion and before `cells_` is touched.

- [ ] **Step 7: Wire CMake sources**

In `ros/src/overlume_ros/CMakeLists.txt`, use multi-line anchors (the single line `src/adapters/ogm.cpp` appears in both blocks, so a one-line anchor is not unique):

- Library (`add_library(overlume_node_lib SHARED ...)`, two-space indent). Replace
```cmake
  src/adapters/path.cpp
  src/adapters/ogm.cpp
  src/adapters/collision.cpp
```
with
```cmake
  src/adapters/path.cpp
  src/adapters/ogm.cpp
  src/adapters/grid_placement.cpp
  src/adapters/collision.cpp
```
- Test (`ament_add_gtest(test_ogm_adapter ...)`, four-space indent; it compiles `ogm.cpp` directly rather than linking the lib, so it needs the new object). Replace
```cmake
    test/test_ogm_adapter.cpp
    src/adapters/ogm.cpp
    src/profile.cpp
```
with
```cmake
    test/test_ogm_adapter.cpp
    src/adapters/ogm.cpp
    src/adapters/grid_placement.cpp
    src/profile.cpp
```

- [ ] **Step 8: Run tests, expect pass**

```bash
cd $WORKTREE
pgrep -af '[T]PSProjector-height-grid/(overlume|ros)/build' || echo idle
ros/colcon_build.sh overlume_ros
source /opt/ros/humble/setup.bash && source ros/install/setup.bash
(cd ros && colcon test --packages-select overlume_ros --ctest-args -R 'test_grid_placement|test_ogm_adapter' && colcon test-result --verbose)
git diff --stat -- ros/src/overlume_ros/test/test_ogm_adapter.cpp
```
Expected: `idle` first; the build succeeds and creates `ros/install`; all 7 `GridPlacement.*` tests pass; every existing `OgmAdapter.*` test passes; the final `git diff --stat` prints nothing (test_ogm_adapter.cpp unmodified).

- [ ] **Step 9: Run the gate in the foreground**

```bash
cd $WORKTREE
pgrep -af '[T]PSProjector-height-grid/(overlume|ros)/build' || echo idle
tools/ci_visual_mode.sh
```
Expected: `idle` first (do not start the gate while another build is writing into `overlume/build`; stage 2 runs `cmake --build overlume/build` itself); then all 6 stages pass (POD header, library ctest, node gtests, WS bridge pytest, golden suite, examples). Stage 3 needs `ros/install/setup.bash`, created by Step 8's `ros/colcon_build.sh`. No goldens change in this task.

- [ ] **Step 10: Commit**

```bash
cd $WORKTREE
git add ros/src/overlume_ros/include/overlume_ros/adapters/grid_placement.hpp \
        ros/src/overlume_ros/src/adapters/grid_placement.cpp \
        ros/src/overlume_ros/src/adapters/ogm.cpp \
        ros/src/overlume_ros/test/test_grid_placement.cpp \
        ros/src/overlume_ros/CMakeLists.txt
git commit -m "refactor(ros): extract grid placement from OgmAdapter into place_grid

The TF lookup, frame_id override, origin transform, NaN check, flatten_z and
yaw composition inside OgmAdapter::ingest move to a shared place_grid()
helper so the upcoming HeightGridAdapter places its grid identically instead
of copying the logic. OgmAdapter maps kNoTf to dropped_no_tf and kNonFinite
to dropped_malformed exactly as before; test_ogm_adapter is unchanged and
passes. test_grid_placement covers identity/empty frame, frame override,
rotated-TF yaw composition, flatten_z, missing TF and NaN origin.

Claude-Session: https://claude.ai/code/session_018o7iLaY69qZAcNUfeD2VUi"
```

---

### Task 6: Profile keys, HeightGridAdapter and SceneAssembly

All paths are relative to the worktree root `$WORKTREE`. Every command block starts with `cd $WORKTREE`. Do not touch `$PRIMARY_CHECKOUT`.

**Files:**
- Create: `ros/src/overlume_ros/include/overlume_ros/adapters/height_grid.hpp`
- Create: `ros/src/overlume_ros/src/adapters/height_grid.cpp`
- Create: `ros/src/overlume_ros/test/test_height_grid_adapter.cpp`
- Create: `ros/src/overlume_ros/test/fixtures/height_grid_synthetic.yaml`
- Modify: `ros/src/overlume_ros/include/overlume_ros/profile.hpp` (struct `ProfileRow`, after `std::string encoding{"occupancy"};`)
- Modify: `ros/src/overlume_ros/src/profile.cpp` (`RoleSets`, `TypeSets`, `KnownAdapters`, `KnownRowKeys`, the `frame_id` and `encoding` blocks in `ParseRow`, `ValidateRow` after the `update_topic` check, the duplicate-pair loop in `BuildProfile`; `subscriptions_for` needs no code change)
- Modify: `ros/src/overlume_ros/include/overlume_ros/scene_assembly.hpp` (struct `SceneAssembly`, add member after `trajectory_carpets`)
- Modify: `ros/src/overlume_ros/src/scene_assembly.cpp` (`SceneAssembly::clear()` and `SceneAssembly::point_at()`)
- Modify: `ros/src/overlume_ros/CMakeLists.txt` (add the source to `overlume_node_lib` after `src/adapters/ogm.cpp`; add `ament_add_gtest(test_height_grid_adapter ...)` after the `test_ogm_adapter` block). Anchor by quoted text: Task 5 also edits this file, so line numbers move.
- Modify: `docs/runbooks/profile_authoring.md` (key table rows `adapter`, `frame_id`, `encoding`, new rows `height_min_m` / `height_max_m`, adapter table row; Step 4i)
- Test: `ros/src/overlume_ros/test/test_height_grid_adapter.cpp`, additions to `ros/src/overlume_ros/test/test_profile.cpp` (append at end of file) and `ros/src/overlume_ros/test/test_scene_assembly.cpp` (extend `ClearEmptiesEveryCategoryNotJustMapElements`, plus one new test at the end)

**Interfaces:**
- Consumes (Task 1): `overlume::HeightGridLayer { Vec3 origin; double yaw_rad; double resolution_m; uint32_t width_cells, height_cells; const float* heights_m; double last_update_sec; }` and `SceneGraph::height_grids` / `SceneGraph::height_grid_count` from `overlume/scene.h` (main is at `kSceneVersion` 8 / `sizeof(SceneGraph)` 216 until Task 1 lands; Task 1 makes them 9 / 232).
- Consumes (Task 5, not on the branch yet, created by Task 5): `ros/src/overlume_ros/include/overlume_ros/adapters/grid_placement.hpp`:
  `enum class PlacementResult { kOk, kNoTf, kNonFinite }; struct GridPlacement { overlume::Vec3 origin; double yaw_rad; }; PlacementResult place_grid(const std_msgs::msg::Header& header, const geometry_msgs::msg::Pose& grid_origin, const std::string& frame_override, const FrameTransformer& tf, GridPlacement& out);`
  and `src/adapters/grid_placement.cpp`. Task 5 adds it to `overlume_node_lib` and to the `test_ogm_adapter` sources.
  **Behavioural contract this task relies on:** `place_grid` applies the `frame_id` override (replaces `header.frame_id` before the lookup), the TF lookup (`kNoTf` on failure), the NaN check on the transformed origin (`kNonFinite`), `flatten_z` (`out.origin.z = 0` when `tf.flatten_z()`), and the yaw composition (TF rotation composed with the grid orientation), exactly as `OgmAdapter::ingest` did inline before Task 5. `HeightGridAdapter` does none of these itself; the flatten, no-TF, non-finite, base_link-yaw and frame-override tests in Step 1 exercise them through `place_grid`.
  Before starting, confirm with:
  `ls ros/src/overlume_ros/include/overlume_ros/adapters/grid_placement.hpp ros/src/overlume_ros/src/adapters/grid_placement.cpp`
  and `grep -n flatten_z ros/src/overlume_ros/src/adapters/grid_placement.cpp`
  (the grep must hit; if it does not, Task 5 left `flatten_z` to its caller: stop and fix Task 5 rather than adding it here). If the header path differs, adjust the single `#include` in Step 5 and the source path in the Step 1 CMake block.
- Produces:
  - `ProfileRow::height_min_m`, `ProfileRow::height_max_m` (both `double`, default 0.0).
  - `float overlume::ros::decode_height_cell(int8_t v, bool normalized, double min_m, double max_m, bool& malformed);`
  - `class overlume::ros::HeightGridAdapter { HeightGridAdapter(const ProfileRow& row, const FrameTransformer& tf); void ingest(const nav_msgs::msg::OccupancyGrid& msg, double sim_time_sec); void fill(SceneAssembly& out) const; const AdapterStats& stats() const; void mark_stale_tick(); };`
  - `SceneAssembly::height_grids` (`std::vector<overlume::HeightGridLayer>`), cleared in `clear()`, exposed in `point_at()`.
  - Profile semantics: adapter `height_grid`, role `terrain`, type `nav_msgs/msg/OccupancyGrid`.

**Worktree build prerequisites (read first):**
- `ros/build`, `ros/install` and `ros/log` do not exist in the worktree yet (they are gitignored). `ros/colcon_build.sh` does `cd` to its own directory, so it builds into `$WORKTREE/ros/{build,install,log}` and never touches the other checkout.
- The node CMake needs the prebuilt library `overlume/build/liboverlume.a` of this worktree (the library build in `overlume/build` is running elsewhere; do not start or disturb it). Before Step 2, check `ls overlume/build/liboverlume.a`; if it is missing, wait for that build to finish.
- `tools/ci_visual_mode.sh` resolves `REPO_ROOT` from its own location and nothing in it or in `docs/runbooks/ci_gate.md` is hard-wired to the other checkout. Its stage 3 requires `ros/install/setup.bash` to exist (it fails with "build it first" otherwise), which Step 2's build creates. Do not set `CI_VISUAL_MODE_ROS_APPS_INSTALL`.

- [ ] **Step 1: Write the fixture, the tests and the CMake test target (all failing)**

Create `ros/src/overlume_ros/test/fixtures/height_grid_synthetic.yaml`:

```yaml
# height_grid_synthetic.yaml. SYNTHETIC, hand-authored (no recorded
# /debug_ogm_2 bag exists). 5x2 grid, resolution 0.2m, origin (1.0, 2.0, 0.5),
# frame map. Encoded against the shipped window min -2.0 .. max 3.0.
# Row 0 (y=0): -1 (unknown), 0, 50, 100, 127 (illegal: malformed + NaN).
# Row 1 (y=1): 1, 99, 25, 75, -1 (unknown).
header:
  frame_id: map
  stamp: {sec: 0, nanosec: 0}
info:
  resolution: 0.2
  width: 5
  height: 2
  origin:
    position: {x: 1.0, y: 2.0, z: 0.5}
    orientation: {w: 1.0}
data: [-1, 0, 50, 100, 127,
        1, 99, 25,  75,  -1]
```

Create `ros/src/overlume_ros/test/test_height_grid_adapter.cpp`:

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/adapters/height_grid.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <gtest/gtest.h>
#include <rclcpp/clock.hpp>
#include <tf2_ros/buffer.h>

#include "fixture_msgs.hpp"

using overlume::ros::decode_height_cell;
using overlume::ros::FrameTransformer;
using overlume::ros::HeightGridAdapter;
using overlume::ros::ProfileRow;
using overlume::ros::SceneAssembly;

namespace {

struct TfFixture {
    std::shared_ptr<rclcpp::Clock> clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    tf2_ros::Buffer buffer{clock};
    FrameTransformer tf{buffer};
};

ProfileRow HeightRow(const std::string& encoding = "height_linear") {
    ProfileRow r;
    r.topic = "/debug_ogm_2";
    r.type = "nav_msgs/msg/OccupancyGrid";
    r.adapter = "height_grid";
    r.role = "terrain";
    r.encoding = encoding;
    r.height_min_m = -2.0;
    r.height_max_m = 3.0;
    return r;
}

const overlume::HeightGridLayer* OnlyLayer(const SceneAssembly& asm_) {
    return asm_.height_grids.size() == 1 ? &asm_.height_grids[0] : nullptr;
}

}

TEST(DecodeHeightCell, LinearMapsZeroAndHundredToTheWindowEnds) {
    bool malformed = false;
    EXPECT_FLOAT_EQ(decode_height_cell(0, false, -2.0, 3.0, malformed), -2.0f);
    EXPECT_FLOAT_EQ(decode_height_cell(100, false, -2.0, 3.0, malformed), 3.0f);
    EXPECT_FLOAT_EQ(decode_height_cell(50, false, -2.0, 3.0, malformed), 0.5f);
    EXPECT_FALSE(malformed);
}

TEST(DecodeHeightCell, NormalizedMapsOneAndHundredToTheWindowEndsAndClampsZero) {
    bool malformed = false;
    EXPECT_FLOAT_EQ(decode_height_cell(1, true, -2.0, 3.0, malformed), -2.0f);
    EXPECT_FLOAT_EQ(decode_height_cell(100, true, -2.0, 3.0, malformed), 3.0f);
    EXPECT_FLOAT_EQ(decode_height_cell(0, true, -2.0, 3.0, malformed), -2.0f)
        << "(0-1)/99 is negative and must clamp to the window minimum";
    EXPECT_FLOAT_EQ(decode_height_cell(50, true, -2.0, 3.0, malformed),
                    static_cast<float>(-2.0 + 49.0 / 99.0 * 5.0));
    EXPECT_FALSE(malformed);
}

TEST(DecodeHeightCell, MinusOneIsUnknownNanAndNotMalformed) {
    for (const bool normalized : {false, true}) {
        bool malformed = false;
        EXPECT_TRUE(std::isnan(decode_height_cell(-1, normalized, -2.0, 3.0, malformed)));
        EXPECT_FALSE(malformed) << "-1 is the legal unknown marker";
    }
}

TEST(DecodeHeightCell, OutOfRangeValuesAreNanAndFlagMalformed) {
    for (const bool normalized : {false, true}) {
        for (const int v : {101, 127, -2, -128}) {
            bool malformed = false;
            EXPECT_TRUE(std::isnan(decode_height_cell(static_cast<int8_t>(v), normalized, -2.0,
                                                      3.0, malformed)))
                << v;
            EXPECT_TRUE(malformed) << v;
        }
    }
}

TEST(HeightGridAdapter, LinearFixturePopulatesLayerGeometryAndHeights) {
    auto msg = overlume::ros::testing::load_occupancy_grid("height_grid_synthetic.yaml");
    ASSERT_EQ(msg.info.width, 5u);
    ASSERT_EQ(msg.info.height, 2u);
    TfFixture kTf;
    HeightGridAdapter a(HeightRow(), kTf.tf);
    a.ingest(msg, 1.0);

    SceneAssembly asm_;
    a.fill(asm_);
    const overlume::HeightGridLayer* g = OnlyLayer(asm_);
    ASSERT_NE(g, nullptr);
    EXPECT_DOUBLE_EQ(g->origin.x, 1.0);
    EXPECT_DOUBLE_EQ(g->origin.y, 2.0);
    EXPECT_DOUBLE_EQ(g->origin.z, 0.0) << "flatten_z (default true) zeroes the origin z";
    EXPECT_DOUBLE_EQ(g->yaw_rad, 0.0);
    EXPECT_NEAR(g->resolution_m, 0.2, 1e-6);
    EXPECT_EQ(g->width_cells, 5u);
    EXPECT_EQ(g->height_cells, 2u);
    EXPECT_DOUBLE_EQ(g->last_update_sec, 1.0);
    ASSERT_NE(g->heights_m, nullptr);

    EXPECT_TRUE(std::isnan(g->heights_m[0])) << "-1 is unknown";
    EXPECT_FLOAT_EQ(g->heights_m[1], -2.0f) << "v=0";
    EXPECT_FLOAT_EQ(g->heights_m[2], 0.5f) << "v=50";
    EXPECT_FLOAT_EQ(g->heights_m[3], 3.0f) << "v=100";
    EXPECT_TRUE(std::isnan(g->heights_m[4])) << "127 is illegal";
    EXPECT_FLOAT_EQ(g->heights_m[5], -1.95f) << "v=1";
    EXPECT_FLOAT_EQ(g->heights_m[6], 2.95f) << "v=99";
    EXPECT_FLOAT_EQ(g->heights_m[7], -0.75f) << "v=25";
    EXPECT_FLOAT_EQ(g->heights_m[8], 1.75f) << "v=75";
    EXPECT_TRUE(std::isnan(g->heights_m[9]));

    EXPECT_EQ(a.stats().msgs, 1u);
    EXPECT_DOUBLE_EQ(a.stats().last_msg_sec, 1.0);
    EXPECT_EQ(a.stats().dropped_malformed, 1u)
        << "one illegal cell (127) -- counted once per message, and the message is still applied";
}

TEST(HeightGridAdapter, NormalizedEncodingUsesTheOneToHundredScale) {
    auto msg = overlume::ros::testing::load_occupancy_grid("height_grid_synthetic.yaml");
    TfFixture kTf;
    HeightGridAdapter a(HeightRow("height_normalized"), kTf.tf);
    a.ingest(msg, 1.0);

    SceneAssembly asm_;
    a.fill(asm_);
    const overlume::HeightGridLayer* g = OnlyLayer(asm_);
    ASSERT_NE(g, nullptr);
    EXPECT_FLOAT_EQ(g->heights_m[1], -2.0f) << "v=0 clamps to min";
    EXPECT_FLOAT_EQ(g->heights_m[3], 3.0f) << "v=100";
    EXPECT_FLOAT_EQ(g->heights_m[5], -2.0f) << "v=1 is the window minimum";
    EXPECT_FLOAT_EQ(g->heights_m[2], static_cast<float>(-2.0 + 49.0 / 99.0 * 5.0)) << "v=50";
    EXPECT_TRUE(std::isnan(g->heights_m[0]));
}

TEST(HeightGridAdapter, SizeMismatchAndZeroDimsAreDroppedAndCounted) {
    TfFixture kTf;
    HeightGridAdapter a(HeightRow(), kTf.tf);

    auto shortMsg = overlume::ros::testing::load_occupancy_grid("height_grid_synthetic.yaml");
    shortMsg.data.pop_back();
    a.ingest(shortMsg, 1.0);

    auto zeroMsg = overlume::ros::testing::load_occupancy_grid("height_grid_synthetic.yaml");
    zeroMsg.info.width = 0;
    zeroMsg.data.clear();
    a.ingest(zeroMsg, 1.1);

    SceneAssembly asm_;
    a.fill(asm_);
    EXPECT_TRUE(asm_.height_grids.empty()) << "nothing was accepted, so fill() emits nothing";
    EXPECT_EQ(a.stats().msgs, 2u);
    EXPECT_EQ(a.stats().dropped_malformed, 2u);
}

TEST(HeightGridAdapter, MissingTransformIsDroppedAsNoTf) {
    auto msg = overlume::ros::testing::load_occupancy_grid("height_grid_synthetic.yaml");
    msg.header.frame_id = "frame_nobody_publishes";
    TfFixture kTf;
    HeightGridAdapter a(HeightRow(), kTf.tf);
    a.ingest(msg, 1.0);

    SceneAssembly asm_;
    a.fill(asm_);
    EXPECT_TRUE(asm_.height_grids.empty());
    EXPECT_EQ(a.stats().dropped_no_tf, 1u);
    EXPECT_EQ(a.stats().dropped_malformed, 0u);
}

TEST(HeightGridAdapter, NonFiniteOriginIsDroppedAsMalformed) {
    auto msg = overlume::ros::testing::load_occupancy_grid("height_grid_synthetic.yaml");
    msg.info.origin.position.x = std::numeric_limits<double>::quiet_NaN();
    TfFixture kTf;
    HeightGridAdapter a(HeightRow(), kTf.tf);
    a.ingest(msg, 1.0);

    SceneAssembly asm_;
    a.fill(asm_);
    EXPECT_TRUE(asm_.height_grids.empty());
    EXPECT_EQ(a.stats().dropped_malformed, 1u);
}

TEST(HeightGridAdapter, FlattenZOffKeepsTheOriginZ) {
    auto msg = overlume::ros::testing::load_occupancy_grid("height_grid_synthetic.yaml");
    msg.info.origin.position.z = 2.5;
    TfFixture kTf;
    FrameTransformer keepZ(kTf.buffer, "map", false);
    HeightGridAdapter a(HeightRow(), keepZ);
    a.ingest(msg, 1.0);

    SceneAssembly asm_;
    a.fill(asm_);
    const overlume::HeightGridLayer* g = OnlyLayer(asm_);
    ASSERT_NE(g, nullptr);
    EXPECT_DOUBLE_EQ(g->origin.z, 2.5);
}

TEST(HeightGridAdapter, FlattenZOnZeroesAnOriginZThatIsNotZero) {
    auto msg = overlume::ros::testing::load_occupancy_grid("height_grid_synthetic.yaml");
    msg.info.origin.position.z = 2.5;
    TfFixture kTf;
    HeightGridAdapter a(HeightRow(), kTf.tf);
    a.ingest(msg, 1.0);

    SceneAssembly asm_;
    a.fill(asm_);
    const overlume::HeightGridLayer* g = OnlyLayer(asm_);
    ASSERT_NE(g, nullptr);
    EXPECT_DOUBLE_EQ(g->origin.z, 0.0);
}

TEST(HeightGridAdapter, BaseLinkGridTakesTheVehicleYawIntoTheMapFrame) {
    auto msg = overlume::ros::testing::load_occupancy_grid("height_grid_synthetic.yaml");
    msg.header.frame_id = "base_link";
    msg.info.origin.position.x = -25.0;
    msg.info.origin.position.y = -25.0;
    msg.info.origin.orientation.w = 1.0;
    TfFixture kTf;
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "map";
    t.child_frame_id = "base_link";
    t.transform.translation.x = 10.0;
    t.transform.rotation.z = std::sin(M_PI / 4.0);
    t.transform.rotation.w = std::cos(M_PI / 4.0);
    kTf.buffer.setTransform(t, "test", true);

    HeightGridAdapter a(HeightRow(), kTf.tf);
    a.ingest(msg, 1.0);
    SceneAssembly asm_;
    a.fill(asm_);
    const overlume::HeightGridLayer* g = OnlyLayer(asm_);
    ASSERT_NE(g, nullptr);
    EXPECT_NEAR(g->yaw_rad, M_PI / 2.0, 1e-9) << "vehicle yaw must reach the layer";
    EXPECT_NEAR(g->origin.x, 35.0, 1e-9);
    EXPECT_NEAR(g->origin.y, -25.0, 1e-9);
}

TEST(HeightGridAdapter, FrameIdOverrideReplacesTheHeaderFrame) {
    auto msg = overlume::ros::testing::load_occupancy_grid("height_grid_synthetic.yaml");
    msg.header.frame_id = "frame_nobody_publishes";
    TfFixture kTf;
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "map";
    t.child_frame_id = "seyond";
    t.transform.translation.x = 3.0;
    t.transform.translation.y = 4.0;
    t.transform.rotation.w = 1.0;
    kTf.buffer.setTransform(t, "test", true);

    auto row = HeightRow();
    row.frame_id = "seyond";
    HeightGridAdapter a(row, kTf.tf);
    a.ingest(msg, 1.0);
    SceneAssembly asm_;
    a.fill(asm_);
    const overlume::HeightGridLayer* g = OnlyLayer(asm_);
    ASSERT_NE(g, nullptr) << "the override frame must be looked up instead of the header's";
    EXPECT_NEAR(g->origin.x, 4.0, 1e-9);
    EXPECT_NEAR(g->origin.y, 6.0, 1e-9);
    EXPECT_EQ(a.stats().dropped_no_tf, 0u);
}

TEST(HeightGridAdapter, LaterMessageReplacesTheEarlierOneAndFillPointsAtOwnedHeights) {
    auto first = overlume::ros::testing::load_occupancy_grid("height_grid_synthetic.yaml");
    auto second = first;
    second.data.assign(10, 100);
    TfFixture kTf;
    HeightGridAdapter a(HeightRow(), kTf.tf);
    a.ingest(first, 1.0);
    a.ingest(second, 2.0);

    SceneAssembly asm_;
    a.fill(asm_);
    const overlume::HeightGridLayer* g = OnlyLayer(asm_);
    ASSERT_NE(g, nullptr) << "one adapter, one layer";
    EXPECT_DOUBLE_EQ(g->last_update_sec, 2.0);
    for (uint32_t i = 0; i < 10; ++i) EXPECT_FLOAT_EQ(g->heights_m[i], 3.0f) << i;
    EXPECT_EQ(a.stats().msgs, 2u);

    SceneAssembly again;
    a.fill(again);
    ASSERT_EQ(again.height_grids.size(), 1u);
    EXPECT_EQ(again.height_grids[0].heights_m, g->heights_m)
        << "fill() must point at the adapter's own buffer, not copy per call";
}

TEST(HeightGridAdapter, FillEmitsNothingBeforeTheFirstMessage) {
    TfFixture kTf;
    HeightGridAdapter a(HeightRow(), kTf.tf);
    SceneAssembly asm_;
    a.fill(asm_);
    EXPECT_TRUE(asm_.height_grids.empty());
}

TEST(HeightGridAdapter, MarkStaleTickCountsDroppedStale) {
    TfFixture kTf;
    HeightGridAdapter a(HeightRow(), kTf.tf);
    a.mark_stale_tick();
    a.mark_stale_tick();
    EXPECT_EQ(a.stats().dropped_stale, 2u);
}

TEST(HeightGridAdapter, OneRowYieldsExactlyOneSubscription) {
    const auto specs = overlume::ros::subscriptions_for(HeightRow());
    ASSERT_EQ(specs.size(), 1u);
    EXPECT_EQ(specs[0].topic, "/debug_ogm_2");
    EXPECT_EQ(specs[0].type, "nav_msgs/msg/OccupancyGrid");
}
```

Append to `ros/src/overlume_ros/test/test_profile.cpp` (the file ends with the `ShippedRobotOffroadProfileTargetsTheRealRobotTopics` test; it already has `using overlume::ros::load_profile_string;`, `find_row` and `subscriptions_for`):

```cpp
namespace {

constexpr const char* kHeightRowYaml =
    "  - {topic: /debug_ogm_2, type: nav_msgs/msg/OccupancyGrid, adapter: height_grid,"
    " role: terrain, height_min_m: -2.0, height_max_m: 3.0, best_effort: true}\n";

}

TEST(Profile, HeightGridRowParsesWithEncodingDefaultingToHeightLinear) {
    std::vector<std::string> errs;
    auto p = load_profile_string(std::string("name: t\nrows:\n") + kHeightRowYaml, errs);
    ASSERT_TRUE(p.has_value()) << (errs.empty() ? "" : errs[0]);
    EXPECT_TRUE(errs.empty());
    const auto& r = p->rows[0];
    EXPECT_EQ(r.adapter, "height_grid");
    EXPECT_EQ(r.role, "terrain");
    EXPECT_EQ(r.encoding, "height_linear");
    EXPECT_DOUBLE_EQ(r.height_min_m, -2.0);
    EXPECT_DOUBLE_EQ(r.height_max_m, 3.0);
    EXPECT_TRUE(r.best_effort);
    const auto specs = subscriptions_for(r);
    ASSERT_EQ(specs.size(), 1u);
    EXPECT_EQ(specs[0].topic, "/debug_ogm_2");
    EXPECT_EQ(specs[0].type, "nav_msgs/msg/OccupancyGrid");
    EXPECT_TRUE(specs[0].best_effort);
}

TEST(Profile, HeightGridAcceptsHeightNormalizedAndFrameId) {
    std::vector<std::string> errs;
    auto p = load_profile_string(
        "name: t\nrows:\n"
        "  - {topic: /g, type: nav_msgs/msg/OccupancyGrid, adapter: height_grid, role: terrain,"
        " height_min_m: -1.0, height_max_m: 4.0, encoding: height_normalized, frame_id: seyond}\n",
        errs);
    ASSERT_TRUE(p.has_value()) << (errs.empty() ? "" : errs[0]);
    EXPECT_EQ(p->rows[0].encoding, "height_normalized");
    EXPECT_EQ(p->rows[0].frame_id, "seyond");
}

TEST(Profile, HeightGridRequiresBothWindowKeys) {
    std::vector<std::string> errs;
    EXPECT_FALSE(
        load_profile_string(
            "name: t\nrows:\n  - {topic: /g, type: nav_msgs/msg/OccupancyGrid,"
            " adapter: height_grid, role: terrain, height_max_m: 3.0}\n",
            errs)
            .has_value());
    ASSERT_FALSE(errs.empty());
    EXPECT_NE(errs[0].find("height_min_m"), std::string::npos) << errs[0];

    errs.clear();
    EXPECT_FALSE(
        load_profile_string(
            "name: t\nrows:\n  - {topic: /g, type: nav_msgs/msg/OccupancyGrid,"
            " adapter: height_grid, role: terrain, height_min_m: -2.0}\n",
            errs)
            .has_value());
    ASSERT_FALSE(errs.empty());
    EXPECT_NE(errs[0].find("height_max_m"), std::string::npos) << errs[0];
}

TEST(Profile, HeightGridWindowMustHaveMaxAboveMin) {
    for (const char* window : {"height_min_m: 3.0, height_max_m: 3.0",
                               "height_min_m: 3.0, height_max_m: -2.0"}) {
        std::vector<std::string> errs;
        EXPECT_FALSE(load_profile_string(std::string("name: t\nrows:\n  - {topic: /g,"
                                                     " type: nav_msgs/msg/OccupancyGrid,"
                                                     " adapter: height_grid, role: terrain, ") +
                                             window + "}\n",
                                         errs)
                         .has_value())
            << window;
        ASSERT_FALSE(errs.empty()) << window;
        EXPECT_NE(errs[0].find("height_max_m"), std::string::npos) << errs[0];
    }
}

TEST(Profile, HeightWindowKeysAreRejectedOnOtherAdapters) {
    std::vector<std::string> errs;
    EXPECT_FALSE(
        load_profile_string("name: t\nrows:\n  - {topic: /g, type: nav_msgs/msg/OccupancyGrid,"
                            " adapter: ogm, role: dynamic_ogm, height_min_m: -2.0}\n",
                            errs)
            .has_value());
    ASSERT_FALSE(errs.empty());
    EXPECT_NE(errs[0].find("height_min_m"), std::string::npos) << errs[0];

    errs.clear();
    EXPECT_FALSE(
        load_profile_string("name: t\nrows:\n  - {topic: /g, type: nav_msgs/msg/OccupancyGrid,"
                            " adapter: ogm, role: dynamic_ogm, height_max_m: 3.0}\n",
                            errs)
            .has_value());
    ASSERT_FALSE(errs.empty());
    EXPECT_NE(errs[0].find("height_max_m"), std::string::npos) << errs[0];
}

TEST(Profile, HeightGridEncodingRejectsOgmValuesAndOgmRejectsHeightValues) {
    std::vector<std::string> errs;
    EXPECT_FALSE(
        load_profile_string(
            "name: t\nrows:\n  - {topic: /g, type: nav_msgs/msg/OccupancyGrid,"
            " adapter: height_grid, role: terrain, height_min_m: -2.0, height_max_m: 3.0,"
            " encoding: occupancy}\n",
            errs)
            .has_value());
    ASSERT_FALSE(errs.empty());
    EXPECT_NE(errs[0].find("height_linear|height_normalized"), std::string::npos) << errs[0];

    errs.clear();
    EXPECT_FALSE(
        load_profile_string("name: t\nrows:\n  - {topic: /g, type: nav_msgs/msg/OccupancyGrid,"
                            " adapter: ogm, role: dynamic_ogm, encoding: height_linear}\n",
                            errs)
            .has_value());
    ASSERT_FALSE(errs.empty());
    EXPECT_NE(errs[0].find("occupancy|costmap"), std::string::npos) << errs[0];
}

TEST(Profile, HeightGridRejectsUpdateTopic) {
    std::vector<std::string> errs;
    EXPECT_FALSE(
        load_profile_string(
            "name: t\nrows:\n  - {topic: /g, type: nav_msgs/msg/OccupancyGrid,"
            " adapter: height_grid, role: terrain, height_min_m: -2.0, height_max_m: 3.0,"
            " update_topic: /g_updates}\n",
            errs)
            .has_value());
    ASSERT_FALSE(errs.empty());
    EXPECT_NE(errs[0].find("update_topic"), std::string::npos) << errs[0];
}

TEST(Profile, HeightGridRejectsWrongRoleAndWrongType) {
    std::vector<std::string> errs;
    EXPECT_FALSE(
        load_profile_string(
            "name: t\nrows:\n  - {topic: /g, type: nav_msgs/msg/OccupancyGrid,"
            " adapter: height_grid, role: dynamic_ogm, height_min_m: -2.0, height_max_m: 3.0}\n",
            errs)
            .has_value());
    errs.clear();
    EXPECT_FALSE(
        load_profile_string(
            "name: t\nrows:\n  - {topic: /g, type: sensor_msgs/msg/PointCloud2,"
            " adapter: height_grid, role: terrain, height_min_m: -2.0, height_max_m: 3.0}\n",
            errs)
            .has_value());
}

TEST(Profile, AtMostOneHeightGridRowPerProfile) {
    std::vector<std::string> errs;
    EXPECT_FALSE(
        load_profile_string(
            std::string("name: t\nrows:\n") + kHeightRowYaml +
                "  - {topic: /debug_ogm_3, type: nav_msgs/msg/OccupancyGrid,"
                " adapter: height_grid, role: terrain, height_min_m: -2.0, height_max_m: 3.0}\n",
            errs)
            .has_value());
    ASSERT_FALSE(errs.empty());
    EXPECT_NE(errs[0].find("height_grid"), std::string::npos) << errs[0];
    EXPECT_NE(errs[0].find("at most one"), std::string::npos) << errs[0];
}
```

Edit `ros/src/overlume_ros/test/test_scene_assembly.cpp`, inside `TEST(SceneAssembly, ClearEmptiesEveryCategoryNotJustMapElements)`: after the line `asm_.trajectory_carpets.push_back(overlume::TrajectoryCarpet{});` add `asm_.height_grids.push_back(overlume::HeightGridLayer{});`, and after `EXPECT_EQ(scene.trajectory_carpet_count, 0u);` add `EXPECT_EQ(scene.height_grid_count, 0u);`. Both anchor lines occur more than once in the file, so make each Edit's old_string unique by including the neighbouring lines of this one test (it is the first test in the file that pushes every category); do not touch the other occurrences. Then append this test at the end of the file (after `RespineWithoutLocalRibbonLeavesCarpetOnItsOwnSpine`):

```cpp
TEST(SceneAssembly, HeightGridsReachTheSceneGraphAndAreClearedBetweenTicks) {
    SceneAssembly asm_;
    const float heights[4] = {0.0f, 1.0f, 2.0f, 3.0f};
    overlume::HeightGridLayer g{};
    g.width_cells = 2;
    g.height_cells = 2;
    g.resolution_m = 0.2;
    g.heights_m = heights;
    asm_.height_grids.push_back(g);

    overlume::SceneGraph scene{};
    asm_.point_at(scene);
    ASSERT_EQ(scene.height_grid_count, 1u);
    EXPECT_EQ(scene.height_grids, asm_.height_grids.data());
    EXPECT_EQ(scene.height_grids[0].heights_m, heights);

    asm_.clear();
    overlume::SceneGraph next{};
    asm_.point_at(next);
    EXPECT_EQ(next.height_grid_count, 0u);
}
```

Edit `ros/src/overlume_ros/CMakeLists.txt`. Directly after the line `target_link_libraries(test_ogm_adapter ${yaml_cpp_vendor_TARGETS} overlume_node_test_paths)` (the last line of the `test_ogm_adapter` block; re-find it by text because Task 5 adds a `grid_placement.cpp` source above it and a `test_grid_placement` block that may follow it) insert:

```cmake

  ament_add_gtest(test_height_grid_adapter
    test/test_height_grid_adapter.cpp
    src/adapters/height_grid.cpp
    src/adapters/grid_placement.cpp
    src/profile.cpp
    src/frame_transform.cpp
    src/adapters/dynamic_objects.cpp
    test/fixture_msgs.cpp)
  target_include_directories(test_height_grid_adapter PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/test
    ${OVERLUME_DIR}/include)
  ament_target_dependencies(test_height_grid_adapter ${THIS_PACKAGE_DEPS})
  target_link_libraries(test_height_grid_adapter ${yaml_cpp_vendor_TARGETS} overlume_node_test_paths)
```

In `overlume_node_lib`, add `  src/adapters/height_grid.cpp` on the line after `  src/adapters/grid_placement.cpp` (Task 5's entry, itself right after `  src/adapters/ogm.cpp`). If Task 5's entry is not there, add it after `  src/adapters/ogm.cpp`. (`overlume_node_test_paths` already supplies `OVERLUME_NODE_FIXTURES_DIR`, so the new fixture is found with no further CMake change.)

- [ ] **Step 2: Run the build and tests, expect failure**

```bash
cd $WORKTREE
ls overlume/build/liboverlume.a
ros/colcon_build.sh overlume_ros 2>&1 | tail -30
```

Expected: FAIL at CMake configure with `Cannot find source file: src/adapters/height_grid.cpp` (once that file exists, compile errors for the missing `overlume_ros/adapters/height_grid.hpp`, `ProfileRow::height_min_m` and `SceneAssembly::height_grids`). This confirms the new tests cannot pass before the implementation. (The very first build in this worktree also creates `ros/build`, `ros/install` and `ros/log`; it is slow, and a failure here before `ros/install` exists is normal. If a failure is about `liboverlume.a` or `OVERLUME_LIB` being absent, the library build has not finished; wait, do not rebuild it from here.)

- [ ] **Step 3: Implement the ProfileRow fields**

In `ros/src/overlume_ros/include/overlume_ros/profile.hpp`, replace

```cpp
    std::string frame_id;
    std::string encoding{"occupancy"};
};
```

with

```cpp
    std::string frame_id;
    std::string encoding{"occupancy"};

    double height_min_m{0.0};
    double height_max_m{0.0};
};
```

- [ ] **Step 4: Implement the profile rules in `ros/src/overlume_ros/src/profile.cpp`**

4a. Add `#include <cmath>` to the include block, after `#include <algorithm>`.

4b. In `RoleSets()` add after the `{"trajectory_carpet", {"carpet"}},` line (the last entry):

```cpp
        {"height_grid", {"terrain"}},
```

4c. In `TypeSets()` add after the `{"trajectory_carpet", {"visualization_msgs/msg/MarkerArray"}},` line (the last entry):

```cpp
        {"height_grid", {"nav_msgs/msg/OccupancyGrid"}},
```

4d. Replace the `KnownAdapters()` list with:

```cpp
    static const std::set<std::string> kAdapters = {
        "dynamic_objects", "path",        "hd_map",            "ogm", "collision", "generic",
        "tf_axes",         "point_cloud", "trajectory_carpet", "height_grid"};
```

4e. Replace the `KnownRowKeys()` list with:

```cpp
    static const std::set<std::string> kKeys = {"topic",        "type",
                                                "adapter",      "role",
                                                "update_topic", "timeout_sec",
                                                "max_rate_hz",  "namespaces",
                                                "ns_default",   "transient_local",
                                                "best_effort",  "junction_interior_boundaries",
                                                "color_mode",   "max_points",
                                                "stride",       "min_z_m",
                                                "frame_id",     "encoding",
                                                "height_min_m", "height_max_m"};
```

4f. In `ParseRow`, replace the whole `frame_id` block and the whole `encoding` block (from `if (node["frame_id"]) {` through the closing brace of `if (node["encoding"]) { ... }`, ending just before `const std::string ns_default_str = ...`) with:

```cpp
    if (node["frame_id"]) {
        if (out.adapter != "point_cloud" && out.adapter != "ogm" &&
            out.adapter != "height_grid") {
            errors.push_back(RowTag(file, idx, out.topic) +
                             "frame_id is only valid on adapter: point_cloud, ogm or "
                             "height_grid rows");
            ok = false;
        } else {
            out.frame_id = node["frame_id"].as<std::string>();
        }
    }
    if (out.adapter == "height_grid") out.encoding = "height_linear";
    if (node["encoding"]) {
        const std::string enc = node["encoding"].as<std::string>();
        if (out.adapter == "ogm") {
            if (enc != "occupancy" && enc != "costmap") {
                errors.push_back(RowTag(file, idx, out.topic) + "encoding '" + enc +
                                 "' must be one of occupancy|costmap");
                ok = false;
            } else {
                out.encoding = enc;
            }
        } else if (out.adapter == "height_grid") {
            if (enc != "height_linear" && enc != "height_normalized") {
                errors.push_back(RowTag(file, idx, out.topic) + "encoding '" + enc +
                                 "' must be one of height_linear|height_normalized");
                ok = false;
            } else {
                out.encoding = enc;
            }
        } else {
            errors.push_back(RowTag(file, idx, out.topic) +
                             "encoding is only valid on adapter: ogm or height_grid rows");
            ok = false;
        }
    }

    for (const char* key : {"height_min_m", "height_max_m"}) {
        if (node[key]) {
            if (out.adapter != "height_grid") {
                errors.push_back(RowTag(file, idx, out.topic) + key +
                                 " is only valid on adapter: height_grid rows");
                ok = false;
            }
        } else if (out.adapter == "height_grid") {
            errors.push_back(RowTag(file, idx, out.topic) + key +
                             " is required on adapter: height_grid rows");
            ok = false;
        }
    }
    if (out.adapter == "height_grid") {
        if (node["height_min_m"]) out.height_min_m = node["height_min_m"].as<double>();
        if (node["height_max_m"]) out.height_max_m = node["height_max_m"].as<double>();
    }
```

(The two required-key checks must live in `ParseRow`, not `ValidateRow`: `ProfileRow` carries 0.0 defaults, so only the YAML node can tell "absent" from "0". The existing `Profile.EncodingIsRejectedOffOgmRowsAndForUnknownValues` test still passes: a point_cloud row with `encoding` is rejected, and an ogm row with `encoding: rgb` yields a message containing "encoding".)

4g. In `ValidateRow`, replace

```cpp
    if (!row.update_topic.empty() && row.adapter != "ogm")
        return fail("update_topic is only valid on adapter: ogm rows");
```

with (the existing rejection already covers height_grid, since it is not `ogm`):

```cpp
    if (!row.update_topic.empty() && row.adapter != "ogm")
        return fail("update_topic is only valid on adapter: ogm rows");

    if (row.adapter == "height_grid") {
        if (!std::isfinite(row.height_min_m) || !std::isfinite(row.height_max_m) ||
            !(row.height_max_m > row.height_min_m))
            return fail("height_max_m must be finite and greater than height_min_m");
    }
```

4h. In `BuildProfile`, replace the duplicate-pair loop (from `std::set<std::pair<std::string, std::string>> seen_topic_adapter;` through the closing brace of the `for` loop, just before `if (hard_fail) return std::nullopt;`) with the following (adds the one-row rule; the pair check is unchanged):

```cpp
    std::set<std::pair<std::string, std::string>> seen_topic_adapter;
    bool seen_height_grid = false;
    for (size_t i = 0; i < profile.rows.size(); ++i) {
        const auto& row = profile.rows[i];
        if (row.adapter == "tf_axes") continue;
        if (row.adapter == "height_grid") {
            if (seen_height_grid) {
                errors.push_back(RowTag(file, row_file_idx[i], row.topic) +
                                 "at most one adapter: height_grid row per profile -- the "
                                 "renderer's ground hole follows a single terrain layer");
                hard_fail = true;
            }
            seen_height_grid = true;
        }
        auto key = std::make_pair(row.topic, row.adapter);
        if (!seen_topic_adapter.insert(key).second) {
            errors.push_back(RowTag(file, row_file_idx[i], row.topic) +
                             "duplicate (topic, adapter) pair -- another row already " +
                             "subscribes '" + row.topic + "' with adapter '" + row.adapter + "'");
            hard_fail = true;
        }
    }
```

`subscriptions_for` needs no edit: it returns one `SubSpec` from `{row.topic, row.type, row.best_effort, row.transient_local}` and only `ogm` adds an update subscription (covered by `HeightGridRowParsesWith...` and `OneRowYieldsExactlyOneSubscription`).

4i. Update `docs/runbooks/profile_authoring.md` (spec section 10 names it as a delivery item; line numbers are hints, anchor by the quoted text):

1. In the `adapter` key row, extend the list `one of ...|point_cloud\|trajectory_carpet` to end `...\|point_cloud\|trajectory_carpet\|height_grid`.
2. In the `frame_id` row, replace `**`adapter: point_cloud` / `ogm` only**` with `**`adapter: point_cloud` / `ogm` / `height_grid` only**`. Leave the rest of the row unchanged.
3. In the `encoding` row, replace `**`adapter: ogm` only** — `occupancy` is` with `**`adapter: ogm` / `height_grid` only** — on `ogm`, `occupancy` is`, and append to the end of the row (before the closing `|`): ` On `height_grid` the default is `height_linear` (0–100 maps linearly onto `height_min_m..height_max_m`, −1 unknown, anything else counted malformed and hidden); `height_normalized` maps 1–100 onto the window (0 clamps to the minimum) and is **not metric**, for A/B against `/debug_ogm_1` only. The `occupancy` / `costmap` values are rejected on `height_grid` rows and the height values are rejected on `ogm` rows`.
4. Directly after the `encoding` row insert two rows in the same table:

```
| `height_min_m` | **on `height_grid`** | — | **`adapter: height_grid` only** — metres above the grid origin plane that cell value 0 (`height_linear`) / 1 (`height_normalized`) decodes to. Required on `height_grid` rows, rejected elsewhere; must be finite |
| `height_max_m` | **on `height_grid`** | — | **`adapter: height_grid` only** — metres for cell value 100. Required on `height_grid` rows, rejected elsewhere; must be finite and `> height_min_m` |
```

5. In the adapter/role/type table, directly after the `trajectory_carpet` row (`| `trajectory_carpet` | `carpet` | `visualization_msgs/msg/MarkerArray` |`) insert:

```
| `height_grid` | `terrain` | `nav_msgs/msg/OccupancyGrid` |
```

6. In the adapter-behaviour list, after the `trajectory_carpet` bullet add one bullet: `- **`height_grid`** — `OccupancyGrid` carrying heights (not occupancy) → `HeightGridLayer` (terrain mesh). At most one `height_grid` row per profile (the renderer's ground hole follows a single terrain layer); `update_topic` is rejected. The layer-gate / mode-content table below is added by the layer-gate task, not here.`

If a quoted anchor is not found verbatim, grep the file for the neighbouring text and keep the same table-column layout; do not restructure the document.

- [ ] **Step 5: Implement the adapter**

Create `ros/src/overlume_ros/include/overlume_ros/adapters/height_grid.hpp`:

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#pragma once

#include <cstdint>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "overlume_ros/adapter_stats.hpp"
#include "overlume_ros/frame_transform.hpp"
#include "overlume_ros/profile.hpp"
#include "overlume_ros/scene_assembly.hpp"
#include "overlume/scene.h"

namespace overlume::ros {

// One OccupancyGrid cell -> metres above the grid origin plane. -1 is unknown (NaN, not
// malformed). 0..100 decodes linearly (min + v/100*(max-min)) or, when `normalized`, on the
// 1..100 scale (min + clamp((v-1)/99, 0, 1)*(max-min)). Anything else is NaN and sets
// `malformed`.
float decode_height_cell(int8_t v, bool normalized, double min_m, double max_m, bool& malformed);

class HeightGridAdapter {
public:
    HeightGridAdapter(const ProfileRow& row, const overlume::ros::FrameTransformer& tf);

    void ingest(const nav_msgs::msg::OccupancyGrid& msg, double sim_time_sec);

    void fill(overlume::ros::SceneAssembly& out) const;

    const AdapterStats& stats() const { return stats_; }

    void mark_stale_tick() { ++stats_.dropped_stale; }

private:
    ProfileRow row_;
    const overlume::ros::FrameTransformer& tf_;
    bool normalized_{false};

    bool has_grid_{false};
    overlume::Vec3 origin_{};
    double yaw_rad_ = 0.0;
    double resolution_m_{0.0};
    uint32_t width_cells_{0};
    uint32_t height_cells_{0};
    std::vector<float> heights_;
    double last_update_sec_{0.0};
    AdapterStats stats_;
};

}
```

Create `ros/src/overlume_ros/src/adapters/height_grid.cpp`:

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume_ros/adapters/height_grid.hpp"

#include <algorithm>
#include <limits>
#include <utility>

#include "overlume_ros/adapters/grid_placement.hpp"

namespace overlume::ros {

float decode_height_cell(int8_t v, bool normalized, double min_m, double max_m, bool& malformed) {
    constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
    if (v == -1) return kNaN;
    if (v < 0 || v > 100) {
        malformed = true;
        return kNaN;
    }
    const double t = normalized ? std::clamp((v - 1) / 99.0, 0.0, 1.0) : v / 100.0;
    return static_cast<float>(min_m + t * (max_m - min_m));
}

HeightGridAdapter::HeightGridAdapter(const ProfileRow& row,
                                     const overlume::ros::FrameTransformer& tf)
    : row_(row), tf_(tf), normalized_(row.encoding == "height_normalized") {}

void HeightGridAdapter::ingest(const nav_msgs::msg::OccupancyGrid& msg, double sim_time_sec) {
    ++stats_.msgs;

    if (msg.info.width == 0 || msg.info.height == 0) {
        ++stats_.dropped_malformed;
        return;
    }
    if (msg.data.size() != static_cast<size_t>(msg.info.width) * msg.info.height) {
        ++stats_.dropped_malformed;
        return;
    }

    GridPlacement placement{};
    switch (place_grid(msg.header, msg.info.origin, row_.frame_id, tf_, placement)) {
        case PlacementResult::kNoTf:
            ++stats_.dropped_no_tf;
            return;
        case PlacementResult::kNonFinite:
            ++stats_.dropped_malformed;
            return;
        case PlacementResult::kOk:
            break;
    }

    std::vector<float> next(msg.data.size());
    bool malformed = false;
    for (size_t i = 0; i < msg.data.size(); ++i) {
        next[i] = decode_height_cell(msg.data[i], normalized_, row_.height_min_m,
                                     row_.height_max_m, malformed);
    }
    if (malformed) ++stats_.dropped_malformed;

    heights_ = std::move(next);
    origin_ = placement.origin;
    yaw_rad_ = placement.yaw_rad;
    resolution_m_ = msg.info.resolution;
    width_cells_ = msg.info.width;
    height_cells_ = msg.info.height;
    has_grid_ = true;
    last_update_sec_ = sim_time_sec;
    stats_.last_msg_sec = sim_time_sec;
}

void HeightGridAdapter::fill(overlume::ros::SceneAssembly& out) const {
    if (!has_grid_) return;

    overlume::HeightGridLayer g{};
    g.origin = origin_;
    g.yaw_rad = yaw_rad_;
    g.resolution_m = resolution_m_;
    g.width_cells = width_cells_;
    g.height_cells = height_cells_;
    g.heights_m = heights_.data();
    g.last_update_sec = last_update_sec_;
    out.height_grids.push_back(g);
}

}
```

- [ ] **Step 6: Implement SceneAssembly support**

In `ros/src/overlume_ros/include/overlume_ros/scene_assembly.hpp`, inside `struct SceneAssembly`, add after `std::vector<overlume::TrajectoryCarpet> trajectory_carpets;` (before `respined_carpet_points`):

```cpp
    std::vector<overlume::HeightGridLayer> height_grids;
```

In `ros/src/overlume_ros/src/scene_assembly.cpp`, in `SceneAssembly::clear()` add after `trajectory_carpets.clear();`:

```cpp
    height_grids.clear();
```

and in `SceneAssembly::point_at()` add after the `scene.trajectory_carpet_count = static_cast<uint32_t>(trajectory_carpets.size());` line:

```cpp
    scene.height_grids = height_grids.data();
    scene.height_grid_count = static_cast<uint32_t>(height_grids.size());
```

(`LayerFlags`, its `static_assert(sizeof(LayerFlags) == 8, ...)`, `apply_layer_gates`, `mode_content_mask` and `compose_layer_gates` are Task 7's; do not touch them here.)

- [ ] **Step 7: Build and run the node tests, expect pass**

```bash
cd $WORKTREE
ros/colcon_build.sh overlume_ros 2>&1 | tail -15
source /opt/ros/humble/setup.bash && source ros/install/setup.bash
ctest --test-dir ros/build/overlume_ros -R "test_height_grid_adapter|test_profile|test_scene_assembly|test_ogm_adapter|test_grid_placement" --output-on-failure
```

Expected: build succeeds; all test binaries PASS (the new `DecodeHeightCell.*`, `HeightGridAdapter.*`, `Profile.HeightGrid*`, `Profile.AtMostOneHeightGridRowPerProfile`, `Profile.HeightWindowKeysAreRejectedOnOtherAdapters`, `SceneAssembly.HeightGridsReach...`; the existing `Profile.EncodingIsRejectedOffOgmRowsAndForUnknownValues`, the shipped-profile tests and all `OgmAdapter.*` unchanged). Revert check (do not commit): temporarily delete the `height_grids.clear();` line in `SceneAssembly::clear()`, rebuild, confirm `SceneAssembly.ClearEmptiesEveryCategoryNotJustMapElements` fails, then restore the line and rebuild.

- [ ] **Step 8: Run the gate in the foreground**

```bash
cd $WORKTREE
tools/ci_visual_mode.sh
```

Expected: all 6 stages PASS (1 POD header check, 2 library ctest, 3 node colcon build and test including the new `test_height_grid_adapter`, 4 WS bridge pytest, 5 golden suite, 6 examples). Do not background it (a background run can be killed by the low-memory guard). It needs `ros/install/setup.bash` in the worktree, which Step 2/7 created, and the library build in `overlume/build` complete.

- [ ] **Step 9: Commit**

```bash
cd $WORKTREE
git add ros/src/overlume_ros/include/overlume_ros/profile.hpp \
        ros/src/overlume_ros/src/profile.cpp \
        ros/src/overlume_ros/include/overlume_ros/adapters/height_grid.hpp \
        ros/src/overlume_ros/src/adapters/height_grid.cpp \
        ros/src/overlume_ros/include/overlume_ros/scene_assembly.hpp \
        ros/src/overlume_ros/src/scene_assembly.cpp \
        ros/src/overlume_ros/CMakeLists.txt \
        ros/src/overlume_ros/test/test_height_grid_adapter.cpp \
        ros/src/overlume_ros/test/fixtures/height_grid_synthetic.yaml \
        ros/src/overlume_ros/test/test_profile.cpp \
        ros/src/overlume_ros/test/test_scene_assembly.cpp \
        docs/runbooks/profile_authoring.md
git commit -m "$(cat <<'EOF'
feat(node): height_grid profile adapter, HeightGridAdapter and SceneAssembly layer

Add the height_grid profile adapter (role terrain, nav_msgs/msg/OccupancyGrid) with
height_min_m/height_max_m (both required, max > min), encoding height_linear (default) or
height_normalized, frame_id allowed, update_topic rejected, and at most one such row per
profile. HeightGridAdapter decodes an OccupancyGrid into metric float heights (-1 and
illegal cells become NaN, a message with any illegal cell counts one dropped_malformed but
is still applied), places it with place_grid, and fills one overlume::HeightGridLayer.
SceneAssembly carries height_grids through clear() and point_at(). The profile_authoring
runbook documents the new adapter and keys. The node is not wired to the adapter yet; that
is the next task.

Claude-Session: https://claude.ai/code/session_018o7iLaY69qZAcNUfeD2VUi
EOF
)"
```

**Known limits (recorded, not fixed):**
- `last_update_sec` is the node's `sim_clock_sec_` passed to `ingest`. On a paused `/clock` (or any run where sim time does not advance between messages) it repeats, so a new grid ingested at the same sim time carries an unchanged `last_update_sec` and the renderer's "re-upload only when `last_update_sec` changes" rule will not re-upload it. The OGM path behaves identically. Behaviour is deliberately unchanged.
- `height_normalized` is for A/B of `/debug_ogm_1` only and is not metric.
- Every height-grid message with at least one illegal cell counts once in `dropped_malformed` yet is still applied (same convention as `OgmAdapter`).
- The runbook's layer-gate / mode-content table (`layer_*` rows) gains its `height_grid` entry in Task 7 with the `LayerFlags` change, not here.

---

### Task 7: Node wiring, layer flag and offroad profile

All commands run from the worktree root `$WORKTREE`. Never touch `$PRIMARY_CHECKOUT`.

**Files** (all under `$WORKTREE/`; line numbers are hints from the worktree, anchor by the quoted text):
- Modify: `ros/src/overlume_ros/include/overlume_ros/scene_assembly.hpp` (`LayerFlags`, ~:31-40)
- Modify: `ros/src/overlume_ros/src/scene_assembly.cpp` (`apply_layer_gates` ~:89-98, `static_assert` ~:100, `mode_content_mask` ~:104-113, `compose_layer_gates` ~:127-137)
- Modify: `ros/src/overlume_ros/include/overlume_ros/overlume_node.hpp` (include after `hd_map.hpp` ~:38, `HeightGridRow` after `ogm_update_subs_;` ~:162, `layer_height_grids_` after ~:256)
- Modify: `ros/src/overlume_ros/src/overlume_node.cpp` (param declare ~:211, `on_params` ~:789, subscription loop after ~:563, per-tick loop after the `ogm_rows_` loop ~:1060, `user_layer_flags` ~:1130, `publish_diagnostics` ~:1352 and ~:1370, clears ~:1427 and ~:1467)
- Modify: `ros/src/overlume_ros/config/default_params.yaml` (after `layer_trajectory_carpet: true`, ~:40)
- Modify: `ros/src/overlume_ros/config/robot-offroad_profile.yaml` (after the `/perception/geometric_costmap` row, ~:25)
- Modify: `docs/runbooks/profile_authoring.md` (`adapter` row ~:46, `update_topic` row :48, `frame_id` row :60, `encoding` row :61, role/type table ~:75, per-adapter semantics ~:169-171, layer table ~:211)
- Modify: `README.md` (`**Live parameters:**` line, :242)
- Test: `ros/src/overlume_ros/test/test_scene_assembly.cpp` (mask tests ~:172-206; new cases inserted AFTER `ModeContentMaskFreeLookIsAllTrue`, i.e. after the `using` declarations at :168-170), `ros/src/overlume_ros/test/test_profile.cpp` (`ShippedRobotOffroadProfileTargetsTheRealRobotTopics`, ends ~:706)

**Interfaces:**
Consumes (all from earlier tasks, exact):
- Task 1: `overlume::HeightGridLayer`, `SceneGraph::height_grids` / `height_grid_count` (contract: `kSceneVersion` 9, `sizeof(HeightGridLayer)` 64, `sizeof(SceneGraph)` 232; main today is version 8 / 216, so these only hold once Task 1 has landed).
- Task 6: `std::vector<overlume::HeightGridLayer> SceneAssembly::height_grids` (cleared in `clear()`, pointed at in `point_at()`; Task 6 already extends `ClearEmptiesEveryCategoryNotJustMapElements`, so this task does NOT touch that test); `overlume::ros::HeightGridAdapter` in `overlume_ros/adapters/height_grid.hpp` with `HeightGridAdapter(const ProfileRow&, const FrameTransformer&)`, `void ingest(const nav_msgs::msg::OccupancyGrid&, double sim_time_sec)`, `void fill(SceneAssembly&) const`, `const AdapterStats& stats() const`, `void mark_stale_tick()`; `ProfileRow::height_min_m` / `height_max_m`; profile validation accepting `adapter: height_grid`, `role: terrain`, `encoding: height_linear|height_normalized` (default `height_linear`), `frame_id`, rejecting `update_topic`, at most one height_grid row; `subscriptions_for(row)` returning one `nav_msgs/msg/OccupancyGrid` SubSpec for it. (`height_grid.hpp` does not exist on this branch until Task 6 lands; this task must run after Task 6.)

Produces:
- `struct LayerFlags { ...; bool trajectory_carpet = true; bool height_grids = true; };` (9 bools, `static_assert(sizeof(LayerFlags) == 9)`).
- `mode_content_mask`: `height_grids` false for BOWL and HYBRID, true for FREE_LOOK.
- `apply_layer_gates` clears `asm_.height_grids` when `!flags.height_grids`.
- Node param `layer_height_grids` (bool, default true, live-settable).
- `OverlumeNode::HeightGridRow { std::unique_ptr<overlume::ros::HeightGridAdapter> adapter; double timeout_sec; std::string topic; uint64_t warned_malformed = 0; uint64_t warned_no_tf = 0; }`, members `height_grid_rows_`, `height_grid_subs_`.
- Shipped `robot-offroad` row `/debug_ogm_2`.

Notes on main: this task touches no renderer/material code, so the hybrid-splat stencil state on `groundMaterial`/`gridMaterial` (`hybrid_splats.cpp`) is unaffected; the node's hybrid row code (`hybrid_row_suppressed_`, `set_hybrid_splats` in the per-tick body) is left alone, and the height-grid loops are inserted next to the `ogm_rows_` code, not near it. No test enumerates all `LayerFlags` fields or all default params; only the explicit per-field mask tests in `test_scene_assembly.cpp`, which Step 1 extends. `tools/vcam_ws_bridge.py` and `tools/vcam_gui.py` keep their own `LAYER_NAMES` and are deliberately NOT touched; the node param `layer_height_grids` is the disable knob (acceptance uses `layer_height_grids:=false`).

---

- [ ] **Step 1: Write the failing tests**

In `ros/src/overlume_ros/test/test_scene_assembly.cpp`:

(a) In `ModeContentMaskBowlIsAllFalse` add after `EXPECT_FALSE(mask.trajectory_carpet);`:

```cpp
    EXPECT_FALSE(mask.height_grids);
```

In `ModeContentMaskHybridAllowsOnlyPointClouds` add after `EXPECT_FALSE(mask.trajectory_carpet);`:

```cpp
    EXPECT_FALSE(mask.height_grids);
```

In `ModeContentMaskFreeLookIsAllTrue` add after `EXPECT_TRUE(mask.trajectory_carpet);`:

```cpp
    EXPECT_TRUE(mask.height_grids);
```

(b) Add these new cases immediately AFTER the closing `}` of `ModeContentMaskFreeLookIsAllTrue` (and before `ComposeLayerGatesBowlMasksEverythingEvenWhenUserWantsItOn`). They must come after the `using overlume::ros::compose_layer_gates; using overlume::ros::mode_content_mask; using overlume::ros::RenderMode;` block (~:168-170), which they rely on:

```cpp
TEST(SceneAssembly, ApplyLayerGatesHeightGridsOffZeroesOnlyHeightGrids) {
    SceneAssembly asm_;
    asm_.height_grids.push_back(overlume::HeightGridLayer{});
    asm_.objects.push_back(overlume::TrackedObject{});

    LayerFlags flags;
    flags.height_grids = false;
    apply_layer_gates(asm_, flags);

    overlume::SceneGraph scene{};
    asm_.point_at(scene);
    EXPECT_EQ(scene.height_grid_count, 0u);
    EXPECT_EQ(scene.object_count, 1u);
}

TEST(SceneAssembly, ApplyLayerGatesHeightGridsOnLeavesItIntact) {
    SceneAssembly asm_;
    asm_.height_grids.push_back(overlume::HeightGridLayer{});

    LayerFlags flags;
    apply_layer_gates(asm_, flags);

    overlume::SceneGraph scene{};
    asm_.point_at(scene);
    EXPECT_EQ(scene.height_grid_count, 1u);
    EXPECT_EQ(scene.height_grids, asm_.height_grids.data());
}

TEST(SceneAssembly, ComposeLayerGatesHeightGridsFollowsModeAndUser) {
    LayerFlags user;
    EXPECT_FALSE(compose_layer_gates(user, mode_content_mask(RenderMode::BOWL)).height_grids);
    EXPECT_FALSE(compose_layer_gates(user, mode_content_mask(RenderMode::HYBRID)).height_grids);
    EXPECT_TRUE(compose_layer_gates(user, mode_content_mask(RenderMode::FREE_LOOK)).height_grids);

    user.height_grids = false;
    EXPECT_FALSE(compose_layer_gates(user, mode_content_mask(RenderMode::FREE_LOOK)).height_grids);
}
```

In `ros/src/overlume_ros/test/test_profile.cpp`, in `ShippedRobotOffroadProfileTargetsTheRealRobotTopics`, add after `EXPECT_EQ(find_row(*p, "/perception/dynamic_ogm"), nullptr);`:

```cpp
    const auto* terrain = find_row(*p, "/debug_ogm_2");
    ASSERT_NE(terrain, nullptr);
    EXPECT_EQ(terrain->adapter, "height_grid");
    EXPECT_EQ(terrain->role, "terrain");
    EXPECT_EQ(terrain->encoding, "height_linear");
    EXPECT_DOUBLE_EQ(terrain->height_min_m, -2.0)
        << "must match debug_height_encoding in the perception geometric_cost_map_config.yaml";
    EXPECT_DOUBLE_EQ(terrain->height_max_m, 3.0);
    EXPECT_TRUE(terrain->best_effort);
```

- [ ] **Step 2: Run to see the failure**

The gate stage 3 needs this checkout's own `ros/install` (the worktree has none yet), so build it now; this also produces the node build used below.

```bash
cd $WORKTREE && ros/colcon_build.sh 2>&1 | tail -30
```

Expected: compile error in `test_scene_assembly.cpp`: `LayerFlags` has no member `height_grids` (exact wording depends on the compiler). `test_profile` has not been built to a red state yet; its red state is observed after Step 5 (see the note there).

- [ ] **Step 3: `LayerFlags` and the gates**

`ros/src/overlume_ros/include/overlume_ros/scene_assembly.hpp`, in `struct LayerFlags` replace

```cpp
    bool trajectory_carpet = true;
};
```

with

```cpp
    bool trajectory_carpet = true;
    bool height_grids = true;
};
```

`ros/src/overlume_ros/src/scene_assembly.cpp`:

In `apply_layer_gates` after `if (!flags.trajectory_carpet) asm_.trajectory_carpets.clear();` add:

```cpp
    if (!flags.height_grids) asm_.height_grids.clear();
```

Replace `static_assert(sizeof(LayerFlags) == 8,` with `static_assert(sizeof(LayerFlags) == 9,`.

In `mode_content_mask` replace the BOWL and HYBRID returns with:

```cpp
        case RenderMode::BOWL:
            return LayerFlags{false, false, false, false, false, false, false, false, false};
        case RenderMode::HYBRID:
            return LayerFlags{false, false, false, false, false, false, true, false, false};
```

(FREE_LOOK keeps `return LayerFlags{};`, all true.)

In `compose_layer_gates` after `user.trajectory_carpet && mask.trajectory_carpet,` add:

```cpp
        user.height_grids && mask.height_grids,
```

- [ ] **Step 4: Node header**

`ros/src/overlume_ros/include/overlume_ros/overlume_node.hpp`: after the line `#include "overlume_ros/adapters/hd_map.hpp"` add:

```cpp
#include "overlume_ros/adapters/height_grid.hpp"
```

After the `ogm_update_subs_;` declaration (the two-line `std::vector<rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr>\n        ogm_update_subs_;`, immediately before `struct CollisionRow {`) add:

```cpp

    struct HeightGridRow {
        std::unique_ptr<overlume::ros::HeightGridAdapter> adapter;
        double timeout_sec;
        std::string topic;
        uint64_t warned_malformed = 0;
        uint64_t warned_no_tf = 0;
    };
    std::vector<HeightGridRow> height_grid_rows_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr> height_grid_subs_;
```

After `bool layer_trajectory_carpet_{true};` add:

```cpp
    bool layer_height_grids_{true};
```

- [ ] **Step 5: Node source**

`ros/src/overlume_ros/src/overlume_node.cpp`:

(a) Param declaration: after `layer_trajectory_carpet_ = declare_parameter<bool>("layer_trajectory_carpet", true);` add

```cpp
    layer_height_grids_ = declare_parameter<bool>("layer_height_grids", true);
```

(b) `on_params`: replace

```cpp
            else if (n == "layer_trajectory_carpet")
                layer_trajectory_carpet_ = p.as_bool();
```

with

```cpp
            else if (n == "layer_trajectory_carpet")
                layer_trajectory_carpet_ = p.as_bool();
            else if (n == "layer_height_grids")
                layer_height_grids_ = p.as_bool();
```

(c) Subscription loop: directly after `RCLCPP_INFO(get_logger(), "ogm: %zu row(s) subscribed", ogm_rows_.size());` add

```cpp

    for (const auto& row : profile->rows) {
        if (row.adapter != "height_grid") continue;
        const auto specs = overlume::ros::subscriptions_for(row);
        if (specs.empty()) continue;
        const auto& gridSpec = specs[0];

        auto adapter = std::make_unique<overlume::ros::HeightGridAdapter>(row, *frame_transformer_);
        overlume::ros::HeightGridAdapter* adapter_ptr = adapter.get();

        rclcpp::QoS gridQos(10);
        if (gridSpec.best_effort) gridQos.best_effort();
        if (gridSpec.transient_local) gridQos.transient_local();
        height_grid_subs_.push_back(create_subscription<nav_msgs::msg::OccupancyGrid>(
            gridSpec.topic, gridQos,
            [this, adapter_ptr](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                adapter_ptr->ingest(*msg, sim_clock_sec_);
            }));
        height_grid_rows_.push_back(HeightGridRow{std::move(adapter), row.timeout_sec, row.topic});
    }
    RCLCPP_INFO(get_logger(), "height_grid: %zu row(s) subscribed", height_grid_rows_.size());
```

(`HeightGridRow` has default member initialisers, so the 3-value aggregate init is valid in C++14+, same as `OgmRow` above it.)

(d) Per-tick: directly after the `for (auto& gr : ogm_rows_) { ... }` block (it ends with `gr.adapter->fill(scene_asm_);\n    }`, immediately before `for (auto& cr : collision_rows_) {`) add

```cpp

    for (auto& hgr : height_grid_rows_) {
        if (hgr.adapter->stats().msgs == 0) continue;
        warn_on_drop_growth(get_logger(), *get_clock(), hgr.topic, hgr.adapter->stats(),
                            hgr.warned_malformed, hgr.warned_no_tf);
        if (sim_clock_sec_ - hgr.adapter->stats().last_msg_sec > hgr.timeout_sec) {
            hgr.adapter->mark_stale_tick();
            continue;
        }
        hgr.adapter->fill(scene_asm_);
    }
```

(e) User flags (near `apply_layer_gates(scene_asm_, compose_layer_gates(...))`): replace

```cpp
    const LayerFlags user_layer_flags{
        layer_objects_, layer_paths_,   layer_map_elements_, layer_grids_,
        layer_alerts_,  layer_markers_, layer_point_clouds_, layer_trajectory_carpet_};
```

with

```cpp
    const LayerFlags user_layer_flags{
        layer_objects_, layer_paths_,   layer_map_elements_, layer_grids_,
        layer_alerts_,  layer_markers_, layer_point_clouds_, layer_trajectory_carpet_,
        layer_height_grids_};
```

(f) Diagnostics: in `publish_diagnostics` replace

```cpp
    rows.reserve(hd_map_rows_.size() + dynamic_objects_rows_.size() + path_rows_.size() +
                 ogm_rows_.size() + collision_rows_.size() + generic_marker_rows_.size() +
                 point_cloud_rows_.size() + carpet_rows_.size());
```

with

```cpp
    rows.reserve(hd_map_rows_.size() + dynamic_objects_rows_.size() + path_rows_.size() +
                 ogm_rows_.size() + height_grid_rows_.size() + collision_rows_.size() +
                 generic_marker_rows_.size() + point_cloud_rows_.size() + carpet_rows_.size());
```

and after `for (const auto& gr : ogm_rows_) append_row(gr.topic, gr.adapter->stats(), gr.timeout_sec);` add

```cpp
    for (const auto& hgr : height_grid_rows_)
        append_row(hgr.topic, hgr.adapter->stats(), hgr.timeout_sec);
```

(g) Both teardown blocks (each contains `ogm_grid_subs_.clear(); ogm_update_subs_.clear(); ogm_rows_.clear();`): after each `ogm_rows_.clear();` add

```cpp
    height_grid_subs_.clear();
    height_grid_rows_.clear();
```

Check with `grep -n "ogm_rows_.clear" ros/src/overlume_ros/src/overlume_node.cpp`: exactly two hits, both get the two lines.

Red check for the profile test: before doing Step 6, re-run the filtered test command from Step 8 and expect `ShippedRobotOffroadProfileTargetsTheRealRobotTopics` to fail on `ASSERT_NE(terrain, nullptr)` (the row does not exist yet).

- [ ] **Step 6: Params and offroad profile**

`ros/src/overlume_ros/config/default_params.yaml`: after `    layer_trajectory_carpet: true` add

```yaml
    layer_height_grids: true
```

`ros/src/overlume_ros/config/robot-offroad_profile.yaml`: after the `/perception/geometric_costmap` row (before the blank line and the `collision` rows) add

```yaml

  # Terrain heightfield from the perception debug grid. height_min_m/height_max_m MUST
  # match debug_height_encoding in the perception repo
  # config/off_road/m2/geometric_cost_map_config.yaml (cell v = round((h-min)/(max-min)*100),
  # -1 = unknown). A mismatch silently rescales every height on screen.
  - {topic: /debug_ogm_2, type: nav_msgs/msg/OccupancyGrid,
     adapter: height_grid, role: terrain, encoding: height_linear,
     height_min_m: -2.0, height_max_m: 3.0, best_effort: true}
```

- [ ] **Step 7: Docs, `docs/runbooks/profile_authoring.md` and `README.md`**

Task 6 (Step 4i) already added the `height_grid` key rows, the role/type row and the adapter bullet to `docs/runbooks/profile_authoring.md`. This task adds only the layer-gate pieces it owns (anchor by the quoted text):

1. In the `update_topic` row replace `**`adapter: ogm` only**` with `**`adapter: ogm` only** (`height_grid` rejects it)`.

2. In Task 6's `height_grid` bullet, replace its last sentence `The layer-gate / mode-content table below is added by the layer-gate task, not here.` with:
   `` `layer_height_grids` (below) is its whole-layer disable; it renders only in `render_mode` 3 (hidden in modes 1/2 by the per-mode mask).``

3. In the layer table add after `| `ogm` → `grids` | `layer_grids` |`:

```
| `height_grid` → `height_grids` | `layer_height_grids` (mode 3 only) |
```

Edit `README.md` (:242), in the `**Live parameters:**` line replace `` `layer_trajectory_carpet`, `layer_surround_stitching` `` with `` `layer_trajectory_carpet`, `layer_height_grids`, `layer_surround_stitching` ``.

- [ ] **Step 8: Run the node tests, expect pass**

```bash
cd $WORKTREE && ros/colcon_build.sh 2>&1 | tail -20
cd $WORKTREE/ros && set +u && source /opt/ros/humble/setup.bash && source install/setup.bash && set -u && colcon test --packages-select overlume_ros --ctest-args -R "test_scene_assembly|test_profile|test_scene_layout" --event-handlers console_direct+ 2>&1 | tail -40 && colcon test-result --verbose
```

(`ci_visual_mode.sh` stage 3 uses `colcon build --packages-select overlume_ros` + `colcon test --packages-select overlume_ros` + `colcon test-result --all --verbose` in `ros/` after sourcing `/opt/ros/humble/setup.bash` and the checkout's own `ros/install/setup.bash`; the filtered invocation above is the same with `-R`. If a test target name differs, run `ctest -N` in `ros/build/overlume_ros` to list them.) Expected: `test_scene_assembly` all pass (3 new + 3 extended), `test_profile` passes (`ShippedRobotOffroadProfileTargetsTheRealRobotTopics` sees the `/debug_ogm_2` row, which also proves Task 6's validation accepts it), `test_scene_layout` unchanged; `colcon test-result` reports 0 failures.

Revert check: temporarily set the last value of `mode_content_mask(BOWL)` to `true` -> `ModeContentMaskBowlIsAllFalse` fails; remove the `height_grids` line in `apply_layer_gates` -> `ApplyLayerGatesHeightGridsOffZeroesOnlyHeightGrids` fails. Restore both.

- [ ] **Step 9: Gate, in the foreground**

```bash
cd $WORKTREE && tools/ci_visual_mode.sh
```

Nothing in the script is hard-wired to `$PRIMARY_CHECKOUT`: `REPO_ROOT` is derived from the script path, the node stage sources `${REPO_ROOT}/ros/install/setup.bash` (so Step 2's `ros/colcon_build.sh` must have run in the worktree first, otherwise stage 3 FAILs with "not found ... build it first"), and the library builds into `${REPO_ROOT}/overlume/build`. Expected: all 6 stages pass (POD header, library build+ctest, node colcon build+test, ws bridge pytest, golden counts, examples). Do not background it.

- [ ] **Step 10: Commit**

```bash
cd $WORKTREE && git add \
  ros/src/overlume_ros/include/overlume_ros/scene_assembly.hpp \
  ros/src/overlume_ros/src/scene_assembly.cpp \
  ros/src/overlume_ros/include/overlume_ros/overlume_node.hpp \
  ros/src/overlume_ros/src/overlume_node.cpp \
  ros/src/overlume_ros/config/default_params.yaml \
  ros/src/overlume_ros/config/robot-offroad_profile.yaml \
  ros/src/overlume_ros/test/test_scene_assembly.cpp \
  ros/src/overlume_ros/test/test_profile.cpp \
  docs/runbooks/profile_authoring.md \
  README.md
git commit -m "feat(node): wire the height-grid terrain row, layer_height_grids flag and offroad profile row

overlume_node subscribes height_grid rows (QoS depth 10, best_effort and
transient_local honoured), gates each per tick on message count, drop growth
and staleness, fills SceneAssembly::height_grids, reports a diagnostics row
and clears the row state in both teardown paths. LayerFlags gains
height_grids (9th bool) wired through apply_layer_gates and
compose_layer_gates; mode_content_mask enables it only in FREE_LOOK so modes
1/2 (bowl/hybrid) never show terrain. New live param layer_height_grids
(default true) is the standing disable knob. robot-offroad gains the
/debug_ogm_2 height_linear row (-2..3 m, must match the perception
debug_height_encoding window). profile_authoring.md documents the adapter,
keys, role/type, one-row limit, unknown = -1 and the layer knob; README
lists the new live parameter.

Claude-Session: https://claude.ai/code/session_018o7iLaY69qZAcNUfeD2VUi"
```

---

### Task 8: Perception metric encoder and live probe

**Files:**

Part A (OUTSIDE the overlume repo, `$PERCEPTION_REPO`, detached HEAD with 4 unrelated local edits; every change below is left UNCOMMITTED and nothing in `install/` or `build/` is touched):
- Create: `src/libs/perception_libs/perception_grid_map/include/perception_grid_map/height_encoding.h` (header-only `encode_height_cell`)
- Modify: `src/libs/perception_libs/perception_interfaces/include/perception_interfaces/grid_maps/cost_map.h` (after the `m_debug_ogm_image` member, ~:84)
- Modify: `src/libs/perception_libs/perception_grid_map/src/geometric_cost_map.cpp` (include at :1; defaults after :62; override block after :156; fill after :648 in `update_cost_map`)
- Modify: `src/ros_apps/src/perception_apps/perception_offroad_stack/src/geometric_cost_map_node.cpp` (replace commented block :364-394)
- Modify: `config/off_road/m2/geometric_cost_map_config.yaml` and `config/off_road/m2_sim/geometric_cost_map_config.yaml` (new block between `dims:` and `tuning_params:`, after line 43)
- Scratch only (not in any repo): `<SCR>/check_encode_height_cell.cpp`, `<SCR>/perception_build/`, where `SCR=$SCRATCH`

Part B (overlume repo, worktree `$WORKTREE`, branch `feat/height-grid`, committed):
- Create: `tools/probe_height_grid.py` (mode 0755, SPDX header, so `tools/check_spdx.sh` passes)
- Test: `python3 tools/probe_height_grid.py --self-test` (pure numpy, no ROS)

**Interfaces:**
Consumes: the `/debug_ogm_2` contract of the spec section 3 (no other task's output). `/debug_ogm_1` is unchanged.
Produces:
- C++ `inline int8_t encode_height_cell(float h_above, float min_m, float max_m)` in `perception_grid_map/height_encoding.h`. Returns `clamp(round((h_above - min_m) / (max_m - min_m) * 100), 0, 100)`. The caller emits -1 for unknown cells.
- `mp::perception::interfaces::CostMap` gains `cv::Mat m_debug_height_metric_image` (CV_8SC1, same size as `m_debug_ogm_image`), `float m_debug_height_min_m`, `float m_debug_height_max_m`.
- `/debug_ogm_2` carries metric heights on the same header and info as `/debug_ogm_1`.
- Python, `tools/probe_height_grid.py`: `decode_linear(values_int8, min_m, max_m) -> np.ndarray[float64]` (NaN where unknown or invalid); `compare(ogm1_int8, ogm2_int8, min_m, max_m) -> {"unknown_mask_equal": bool, "pearson_r": float, "n_known": int, "n_used": int, "n_levels": int}`; `qualifies(result) -> bool` (n_used >= 100 and n_levels >= 5). CLI: `--self-test`, `--min-m` (default -2.0), `--max-m` (3.0), `--seconds` (10). It prints `PASS  ...` / `FAIL  ...` / `INFO  ...` lines and exits 0 or 1.

Set these shell variables for every command below. All overlume-repo commands run from the worktree root, so every such block starts with `cd $WORKTREE`. Never run anything in `$PRIMARY_CHECKOUT` (another session's checkout).

```bash
SCR=$SCRATCH
PERC=$PERCEPTION_REPO
GM=$PERC/src/libs/perception_libs/perception_grid_map
WT=$WORKTREE
```

---

#### Part A: perception side (uncommitted)

- [ ] **Step 1: Write the failing standalone encoder check (scratchpad)**

Create `$SCR/check_encode_height_cell.cpp`:

```cpp
// Standalone check of encode_height_cell (no OpenCV/PCL/ROS needed).
// g++ -std=c++14 -Wall -Wextra -Werror -I$GM/include check_encode_height_cell.cpp -o check_encode_height_cell
#include <cassert>
#include <cmath>
#include <cstdio>

#include "perception_grid_map/height_encoding.h"

int main()
{
    const float lo = -2.0f;
    const float hi = 3.0f;

    // Window edges and the interior (0.05 m per level over a 5 m window).
    assert(encode_height_cell(-2.0f, lo, hi) == 0);
    assert(encode_height_cell(3.0f, lo, hi) == 100);
    assert(encode_height_cell(-1.0f, lo, hi) == 20);
    assert(encode_height_cell(0.0f, lo, hi) == 40);  // flat ground under the ego
    assert(encode_height_cell(0.5f, lo, hi) == 50);

    // Rounding, not truncation: 0.024 m -> level 40.48 -> 40, 0.026 m -> 40.52 -> 41.
    assert(encode_height_cell(0.024f, lo, hi) == 40);
    assert(encode_height_cell(0.026f, lo, hi) == 41);

    // Out-of-window heights clamp to the ends.
    assert(encode_height_cell(-50.0f, lo, hi) == 0);
    assert(encode_height_cell(50.0f, lo, hi) == 100);

    // Never -1 (reserved for unknown), always monotonic, and decoding is within half a level.
    int prev = 0;
    for (float h = -6.0f; h <= 7.0f; h += 0.013f)
    {
        const int v = encode_height_cell(h, lo, hi);
        assert(v >= 0 && v <= 100);
        assert(v >= prev);
        prev = v;
        if (h >= lo && h <= hi)
        {
            const float decoded = lo + static_cast<float>(v) / 100.0f * (hi - lo);
            assert(std::fabs(decoded - h) <= 0.0255f);
        }
    }

    // A different window scales accordingly.
    assert(encode_height_cell(0.0f, -1.0f, 1.0f) == 50);

    std::puts("PASS  encode_height_cell");
    return 0;
}
```

- [ ] **Step 2: Run it, expect the compile failure**

```bash
g++ -std=c++14 -Wall -Wextra -Werror -I$GM/include $SCR/check_encode_height_cell.cpp -o $SCR/check_encode_height_cell
```

Expected: `fatal error: perception_grid_map/height_encoding.h: No such file or directory`.

- [ ] **Step 3: Create the header**

Create `$GM/include/perception_grid_map/height_encoding.h`:

```cpp
#ifndef _PERCEPTION_HEIGHT_ENCODING_
#define _PERCEPTION_HEIGHT_ENCODING_

#include <algorithm>
#include <cmath>
#include <cstdint>

/// Fixed-scale metric height encoding published on /debug_ogm_2.
///
/// Level = clamp(round((h_above - min_m) / (max_m - min_m) * 100), 0, 100), i.e. 0.05 m per level
/// for the default -2..+3 m window. h_above is metres above the grid origin plane. Unknown cells
/// are the caller's job (-1); this function never returns a negative value. Header-only so the
/// encoding can be checked without OpenCV/PCL. The window must match the overlume profile row
/// (height_min_m / height_max_m) that decodes it.
inline int8_t encode_height_cell(float h_above, float min_m, float max_m)
{
    const float level = std::round((h_above - min_m) / (max_m - min_m) * 100.0f);
    return static_cast<int8_t>(std::min(100.0f, std::max(0.0f, level)));
}

#endif
```

- [ ] **Step 4: Run the check, expect PASS**

```bash
g++ -std=c++14 -Wall -Wextra -Werror -I$GM/include $SCR/check_encode_height_cell.cpp -o $SCR/check_encode_height_cell && $SCR/check_encode_height_cell
```

Expected: `PASS  encode_height_cell`, exit 0. (If you flip `std::round` to a truncating cast, the 0.026 m assert fails, which is the revert check.)

- [ ] **Step 5: `cost_map.h` members**

```diff
--- a/src/libs/perception_libs/perception_interfaces/include/perception_interfaces/grid_maps/cost_map.h
+++ b/src/libs/perception_libs/perception_interfaces/include/perception_interfaces/grid_maps/cost_map.h
@@ -83,3 +83,9 @@ class CostMap
     cv::Mat m_debug_ogm_image;  ///< Debug occupancy grid image
+
+    /// Metric debug heights (CV_8SC1, same size as m_debug_ogm_image): 0..100 linear over
+    /// [m_debug_height_min_m, m_debug_height_max_m] metres above the grid origin plane, -1 unknown.
+    cv::Mat m_debug_height_metric_image;
+    float m_debug_height_min_m = -2.0f;  ///< Lower bound of the metric height window [m]
+    float m_debug_height_max_m = 3.0f;   ///< Upper bound of the metric height window [m]
 };
```

(Anchor: the line `cv::Mat m_debug_ogm_image;  ///< Debug occupancy grid image` followed by `};`. Apply by context, not by line number.)

- [ ] **Step 6: `geometric_cost_map.cpp`: include, init (both allocation sites), fill**

6a. Include (line 1):

```diff
 #include "perception_grid_map/geometric_cost_map.h"
+#include "perception_grid_map/height_encoding.h"
 
 template class GeometricCostMap<pcl::PointXYZI>;
```

6b. Built-in defaults in `init`, right after the first `m_debug_ogm_image` allocation (~:62):

```diff
     cost_map.m_debug_ogm_image = cv::Mat(cost_map.m_cell_num_y, cost_map.m_cell_num_x, CV_8UC1);
+    // /debug_ogm_2 metric heights; -1 (unknown) until the first update_cost_map fills it.
+    cost_map.m_debug_height_metric_image =
+        cv::Mat(cost_map.m_cell_num_y, cost_map.m_cell_num_x, CV_8SC1, cv::Scalar(-1));
+    cost_map.m_debug_height_min_m = -2.0f;
+    cost_map.m_debug_height_max_m = 3.0f;
     // m_debug_ogm_map_image = cv::Mat(cost_map.m_map_offset_i, cost_map.m_map_offset_j,
```

6c. Config override block, the re-allocation after the dims are re-read (~:156) plus the optional block parse:

```diff
         cost_map.m_debug_ogm_image = cv::Mat(cost_map.m_cell_num_y, cost_map.m_cell_num_x, CV_8UC1);
+        cost_map.m_debug_height_metric_image =
+            cv::Mat(cost_map.m_cell_num_y, cost_map.m_cell_num_x, CV_8SC1, cv::Scalar(-1));
+
+        // Optional metric window of the /debug_ogm_2 height encoding. Keep it identical to the
+        // overlume profile row (height_min_m / height_max_m) that decodes it. Absent block or
+        // max_m <= min_m keeps the -2..+3 m defaults set above.
+        const YAML::Node height_enc = cost_map_cfg["debug_height_encoding"];
+        if (height_enc)
+        {
+            const float min_m = height_enc["min_m"].as<float>(cost_map.m_debug_height_min_m);
+            const float max_m = height_enc["max_m"].as<float>(cost_map.m_debug_height_max_m);
+            if (max_m > min_m)
+            {
+                cost_map.m_debug_height_min_m = min_m;
+                cost_map.m_debug_height_max_m = max_m;
+            }
+            else
+            {
+                std::cerr << "[GeometricCostMap] debug_height_encoding needs max_m > min_m (got min_m="
+                          << min_m << ", max_m=" << max_m << "); using defaults "
+                          << cost_map.m_debug_height_min_m << ".." << cost_map.m_debug_height_max_m
+                          << " m" << std::endl;
+            }
+        }
 
         ////////////////////////////////////////////////
         //////////////// Tuning Params /////////////////
```

6d. Fill in `update_cost_map`, immediately after the 3x3 gap fill (`avg_heights.copyTo(ogm_height_image, fill_mask);`, ~:648) and BEFORE the per-frame debug normalisation (`// Debug: visualize height map in 0..254 ...`, ~:650):

```diff
     avg_heights.copyTo(ogm_height_image, fill_mask);
 
+    // Metric debug heights for /debug_ogm_2: fixed-window encoding of the gap-filled height map,
+    // metres above the grid origin plane (ego z in map + origin z). Unknown cells -> -1. Done
+    // before the per-frame min-max normalisation below, which rescales the data in place.
+    {
+        const float origin_plane_z = translation(2) + cost_map.m_position_z;
+        const float unknown_height = static_cast<float>(cost_map.m_unknown_height_value);
+        cost_map.m_debug_height_metric_image.create(cost_map.m_cell_num_y, cost_map.m_cell_num_x,
+                                                    CV_8SC1);
+        for (int j = 0; j < cost_map.m_cell_num_y; ++j)
+        {
+            const float* src = ogm_height_image.ptr<float>(j);
+            int8_t* dst = cost_map.m_debug_height_metric_image.ptr<int8_t>(j);
+            for (int i = 0; i < cost_map.m_cell_num_x; ++i)
+            {
+                dst[i] = (src[i] == unknown_height)
+                             ? static_cast<int8_t>(-1)
+                             : encode_height_cell(src[i] - origin_plane_z,
+                                                  cost_map.m_debug_height_min_m,
+                                                  cost_map.m_debug_height_max_m);
+            }
+        }
+    }
+
     // Debug: visualize height map in 0..254 while keeping unknown cells at 255.
```

Other `m_debug_ogm_image` allocation sites (grep result): `dynamic_cost_map.cpp:61`, `semantic_cost_map.cpp:71,119` and `perception_mapping/grid_mapping.cpp:36` belong to other maps that never publish `/debug_ogm_2`; they are deliberately left alone (the new image stays empty for them). The only `GeometricCostMap` sites are `:62` and `:156`, both covered above.

- [ ] **Step 7: Node publish**

In `src/ros_apps/src/perception_apps/perception_offroad_stack/src/geometric_cost_map_node.cpp`, replace the whole commented-out block (the `// /////// Publish Map Debug  Heights  OGM` banner, lines ~364-394, through `// m_debug_ogm_2_publisher->publish(*debug_map_ros_ogm_msg);`) with:

```cpp
    ///////////////////////////////////////////////////////////////
    ///// Publish Metric Debug Heights OGM (/debug_ogm_2) /////////
    ///////////////////////////////////////////////////////////////
    nav_msgs::msg::OccupancyGrid debug_height_ros_ogm_msg;
    m_ros_interfaces->build_occupancy_grid_from_cost_map(
        m_cost_map, "base_link", lidar_msg->header.stamp, debug_height_ros_ogm_msg);
    // Same geometry and stamp as the main grid; the data is the fixed-window metric encoding
    // (0..100 over m_debug_height_min_m..max_m above the origin plane, -1 unknown).
    debug_height_ros_ogm_msg.data.assign(
        m_cost_map.m_debug_height_metric_image.ptr<signed char>(0),
        m_cost_map.m_debug_height_metric_image.ptr<signed char>(0) +
            m_cost_map.m_debug_height_metric_image.total());
    m_debug_ogm_2_publisher->publish(debug_height_ros_ogm_msg);
```

The `/debug_ogm_1` block right below it stays untouched. The publisher already exists (`on_configure` ~:27, `on_activate` ~:95).

- [ ] **Step 8: Config blocks (both off-road configs)**

In `config/off_road/m2/geometric_cost_map_config.yaml` and `config/off_road/m2_sim/geometric_cost_map_config.yaml`, between the `dims:` block (ends `min_height_threshold: -2.0`, line 43) and the `Tuning Params` banner:

```diff
     min_height_threshold: -2.0
 
+  ############################################################
+  # Metric height encoding published on /debug_ogm_2 (0..100 over min_m..max_m metres above the
+  # grid origin plane, -1 = unknown). Must match height_min_m / height_max_m of the
+  # `height_grid` row in overlume's ros/src/overlume_ros/config/robot-offroad_profile.yaml.
+  # Removing the block keeps these defaults; max_m <= min_m falls back to them with a warning.
+  ############################################################
+  debug_height_encoding:
+    min_m: -2.0
+    max_m:  3.0
+
   ################################################
   ############### Tuning Params ##################
```

(`GeometricCost Map` is the top-level key and `cost_map_cfg` is that block, so the new block is read as `cost_map_cfg["debug_height_encoding"]`. The gated_commuities configs are left alone; they take the defaults. Note: `geometric_cost_map_node.cpp` hard-codes `config/off_road/m2/geometric_cost_map_config.yaml`, so the node loads only the m2 file. The m2_sim copy is kept in sync for parity and future use; it has no effect today, CARLA runs included.)

- [ ] **Step 9: Compile verification without touching the live install or build trees**

Do NOT configure the the perception repo libs superproject here: its configure step (`perception_libs/CMakeLists.txt`, "Pre-stage public headers") runs `file(COPY ...)` into the hard-set `install/libs` prefix, which would overwrite the installed `cost_map.h` with the new layout while the installed `libperception_grid_map.so` stays old (an ABI mismatch for every later rebuild). Instead (a) replays the one recorded compile command of `geometric_cost_map.cpp` from the live `compile_commands.json` (read-only), with the new headers put first via `-iquote` (the code includes them with `"..."`, and `-iquote` dirs are searched before the installed `-I`/`-isystem` ones), writing the object into scratch and dropping the dependency-file flags that would write `.d` files into `build/`. (b) builds the node package isolated.

```bash
source $PERC/install/setup.bash          # sources /opt/ros/humble + platform + manager, read-only
export USE_PLATFORM_DEPS=TRUE            # same as the live build (see build/CMakeCache.txt OpenCV_DIR)
touch $SCR/stamp
OVERLAY="-iquote $PERC/src/libs/perception_libs/perception_interfaces/include -iquote $GM/include"

# (a) compile the changed library TU against the new headers; object goes to scratch
mkdir -p $SCR/perception_build
python3 -I - "$PERC" "$SCR" <<'EOF'
import json, shlex, subprocess, sys
perc, scr = sys.argv[1], sys.argv[2]
overlay = ["-iquote", f"{perc}/src/libs/perception_libs/perception_interfaces/include",
           "-iquote", f"{perc}/src/libs/perception_libs/perception_grid_map/include"]
for e in json.load(open(f"{perc}/build/compile_commands.json")):
    if e["file"].endswith("perception_grid_map/src/geometric_cost_map.cpp"):
        args = shlex.split(e["command"])
        args[args.index("-o") + 1] = f"{scr}/perception_build/geometric_cost_map.cpp.o"
        out, skip = [], False
        for a in args:                       # drop dependency-file output (-MD/-MMD/-MT x/-MF x)
            if skip:
                skip = False
            elif a in ("-MT", "-MF"):
                skip = True
            elif a not in ("-MD", "-MMD"):
                out.append(a)
        out[1:1] = overlay
        sys.exit(subprocess.call(out, cwd=e["directory"]))
sys.exit("geometric_cost_map.cpp not found in compile_commands.json")
EOF

# (b) the node, isolated colcon build and install bases, only the affected package and its deps
cd $PERC/src/ros_apps
COLCON_LOG_PATH=$SCR/perception_build/colcon_log colcon build \
  --build-base $SCR/perception_build/ros_apps/build \
  --install-base $SCR/perception_build/ros_apps/install \
  --packages-up-to perception_offroad_stack \
  --cmake-args -DCMAKE_CXX_FLAGS="$OVERLAY"
```

Expected: (a) exit 0, no diagnostics; (b) `Summary: 4 packages finished` (`perception_apps_interfaces`, `perception_timestamp_matching_ros`, `perception_visualization`, `perception_offroad_stack`; the count may differ if the visualization package pulls more). None of these four packages copies files into `install/` at configure time, so with explicit `--build-base`/`--install-base` this is safe. Notes: (b) is a compile/link check only. The resulting node links against the OLD installed `libperception_grid_map.so`, so do NOT run it. If the compiler reports `no member named m_debug_height_metric_image`, the overlay lost to the installed header, so recheck the `-iquote` flags. Confirm the live install and build trees were not touched and only the intended source files changed:

```bash
find $PERC/install $PERC/build -newer $SCR/stamp | head      # expected: no output
git -C $PERC status --short
```

Expected `git status`: the 4 pre-existing ` M` files (trt_server CMakeLists, offroad CMakeLists, `dynamic_cost_map_node.cpp`, integration_tests CMakeLists) plus ` M` for `cost_map.h`, `geometric_cost_map.cpp`, `geometric_cost_map_node.cpp`, the two yaml configs, and `?? .../height_encoding.h`. Do NOT `git add`, `commit` or `stash` anything in this repo.

---

#### Part B: live probe (overlume worktree, committed)

Acceptance rule (spec 11.2), as implemented below: the unknown masks must match on EVERY paired frame. Pearson r is judged only on QUALIFYING frames: at least 100 cells known in both and unclamped in `/debug_ogm_2`, and at least 5 distinct unclamped `/debug_ogm_2` levels. Frames that do not qualify (all-unknown frames right after the costmap node starts, near-flat asphalt where `/debug_ogm_2` spans only one or two 0.05 m levels while `/debug_ogm_1` stretches the same noise over 1..100) are printed on their own `INFO` line and are not an r failure. The run FAILs only if fewer than 10 frames qualify, or a qualifying frame has r <= 0.99 (NaN counts as failing), or any pair has mismatched unknown masks.

- [ ] **Step 10: Write `tools/probe_height_grid.py` (core, self-test and live mode in one file)**

Create `$WORKTREE/tools/probe_height_grid.py` (this is the complete file):

```python
#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

"""probe_height_grid.py — checks the /debug_ogm_2 metric height encoding against /debug_ogm_1.

/debug_ogm_1 is the geometric node's height map min-max normalised per frame into 1..100 (int8 -1 =
unknown); /debug_ogm_2 is the same height map in a FIXED metric window (0..100 over
[--min-m, --max-m] metres above the grid origin plane, -1 = unknown). For the same lidar frame the
two must agree on which cells are unknown and be linearly related everywhere the metric encoding is
not clamped.

Live mode (needs a sourced ROS 2 environment and the perception costmap nodes running):
    python3 tools/probe_height_grid.py --seconds 10
subscribes best-effort to both topics, pairs frames by header stamp and prints PASS/FAIL for
  * /debug_ogm_2 arrival rate >= 5 Hz,
  * identical unknown masks on every paired frame,
  * at least 10 qualifying frames, and
  * Pearson r > 0.99 between /debug_ogm_1 values and decoded /debug_ogm_2 heights on every
    qualifying frame (worst frame reported).
A frame qualifies when >= 100 cells are known in both topics and not clamped in /debug_ogm_2 AND
those cells span >= 5 distinct /debug_ogm_2 levels. Other frames (all-unknown right after a node
start, near-flat ground where the fixed 0.05 m quantisation leaves one or two levels) cannot yield
a meaningful r; they are reported on an INFO line and excluded, never failed on r.
Exit code 0 only when every PASS/FAIL line passes. The checks are scale-invariant, so a
--min-m/--max-m that differs from the perception config still passes; a window mismatch only shows
as wrong terrain heights by eye.

    python3 tools/probe_height_grid.py --self-test
runs the same comparison on synthetic round trips (no ROS traffic) and checks that deliberately
broken encodings fail and that excluded frames do not.
"""
from __future__ import annotations

import argparse
import contextlib
import io
import sys
import time
from types import SimpleNamespace

import numpy as np

OGM1_TOPIC = "/debug_ogm_1"
OGM2_TOPIC = "/debug_ogm_2"
MIN_RATE_HZ = 5.0
MIN_PEARSON_R = 0.99
MIN_USED_CELLS = 100   # cells known in both and unclamped in ogm2 for a frame to qualify
MIN_LEVELS = 5         # distinct unclamped ogm2 levels for a frame to qualify
MIN_QUALIFYING = 10    # qualifying frames needed for the run to pass
UNKNOWN = -1
LEVELS = 100
MAX_PENDING = 32  # unpaired frames kept per topic while waiting for the other one


def decode_linear(values_int8, min_m, max_m):
    """Decode /debug_ogm_2 cells to metres: min + v/100*(max-min); NaN for -1 and any invalid value."""
    v = np.asarray(values_int8, dtype=np.int8)
    out = np.full(v.shape, np.nan, dtype=np.float64)
    ok = (v >= 0) & (v <= LEVELS)
    out[ok] = min_m + v[ok].astype(np.float64) / LEVELS * (max_m - min_m)
    return out


def compare(ogm1_int8, ogm2_int8, min_m, max_m):
    """Compare one /debug_ogm_1 frame with its /debug_ogm_2 partner.

    Returns unknown_mask_equal (both topics mark the same cells -1), pearson_r over cells known in
    both and not clamped in ogm2 (level 0 or 100), n_known (known in both), n_used (the cells r was
    computed over) and n_levels (distinct ogm2 levels among those cells). pearson_r is NaN when
    fewer than 2 cells remain or either side is constant.
    """
    a = np.asarray(ogm1_int8, dtype=np.int8)
    b = np.asarray(ogm2_int8, dtype=np.int8)
    if a.shape != b.shape:
        raise ValueError(f"shape mismatch: {a.shape} vs {b.shape}")
    unk1 = a == UNKNOWN
    unk2 = b == UNKNOWN
    heights2 = decode_linear(b, min_m, max_m)
    used = ~unk1 & ~unk2 & (b > 0) & (b < LEVELS)
    r = float("nan")
    if int(used.sum()) >= 2:
        x = a[used].astype(np.float64)
        y = heights2[used]
        if x.std() > 0.0 and y.std() > 0.0:
            r = float(np.corrcoef(x, y)[0, 1])
    return {
        "unknown_mask_equal": bool(np.array_equal(unk1, unk2)),
        "pearson_r": r,
        "n_known": int((~unk1 & ~unk2).sum()),
        "n_used": int(used.sum()),
        "n_levels": int(np.unique(b[used]).size),
    }


def qualifies(result):
    """A frame is judged on r only with enough unclamped cells spanning enough distinct levels."""
    return bool(result["n_used"] >= MIN_USED_CELLS and result["n_levels"] >= MIN_LEVELS)


def passes(result):
    """One qualifying frame passes when the unknown masks agree and r > MIN_PEARSON_R (NaN fails)."""
    return bool(result["unknown_mask_equal"] and result["pearson_r"] > MIN_PEARSON_R)


# --- live pairing and report ----------------------------------------------------------------


def _line(ok, text):
    print(f"{'PASS' if ok else 'FAIL'}  {text}")
    return ok


class Pairer:
    """Collects both topics, pairs frames by exact header stamp and compares each pair."""

    def __init__(self, min_m, max_m):
        self.min_m = min_m
        self.max_m = max_m
        self.pending = {1: {}, 2: {}}
        self.count = {1: 0, 2: 0}
        self.arrivals2 = []
        self.results = []
        self.malformed = 0
        self.shape_mismatch = 0

    def on_msg(self, which, msg):
        now = time.monotonic()
        self.count[which] += 1
        if which == 2:
            self.arrivals2.append(now)
        grid = np.asarray(msg.data, dtype=np.int8)
        if grid.size == 0 or grid.size != msg.info.width * msg.info.height:
            self.malformed += 1
            return
        grid = grid.reshape(msg.info.height, msg.info.width)
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        other = self.pending[3 - which].pop(key, None)
        if other is None:
            mine = self.pending[which]
            mine[key] = grid
            while len(mine) > MAX_PENDING:
                mine.pop(next(iter(mine)))  # dicts keep insertion order: drop the oldest
            return
        a, b = (grid, other) if which == 1 else (other, grid)
        if a.shape != b.shape:
            self.shape_mismatch += 1
            return
        self.results.append(compare(a, b, self.min_m, self.max_m))

    def rate2_hz(self):
        if len(self.arrivals2) < 2:
            return 0.0
        span = self.arrivals2[-1] - self.arrivals2[0]
        return (len(self.arrivals2) - 1) / span if span > 0.0 else 0.0


def report(p, check_rate=True):
    """Print the verdict lines for a Pairer; True only when every PASS/FAIL line passes."""
    ok = True
    if check_rate:
        rate = p.rate2_hz()
        ok &= _line(rate >= MIN_RATE_HZ,
                    f"{OGM2_TOPIC} rate  ({rate:.1f} Hz, need >= {MIN_RATE_HZ:.1f}; {p.count[2]} msgs, "
                    f"{OGM1_TOPIC} {p.count[1]} msgs)")
    n = len(p.results)
    if n == 0:
        _line(False, f"stamp-paired frames  (0 pairs; {p.malformed} malformed, "
                     f"{p.shape_mismatch} shape mismatches)")
        return False
    ok &= _line(True, f"stamp-paired frames  ({n} pairs)")
    equal = sum(1 for r in p.results if r["unknown_mask_equal"])
    ok &= _line(equal == n, f"unknown masks equal  ({equal}/{n} pairs)")

    qual = [r for r in p.results if qualifies(r)]
    n_sparse = sum(1 for r in p.results if r["n_used"] < MIN_USED_CELLS)
    n_flat = n - len(qual) - n_sparse
    print(f"INFO  excluded for insufficient relief  ({n - len(qual)}/{n} frames: {n_sparse} with "
          f"< {MIN_USED_CELLS} usable cells, {n_flat} with < {MIN_LEVELS} distinct levels)")
    ok &= _line(len(qual) >= MIN_QUALIFYING,
                f"qualifying frames  ({len(qual)}, need >= {MIN_QUALIFYING})")
    if qual:
        rs = [r["pearson_r"] for r in qual]
        worst = float("nan") if any(np.isnan(rs)) else min(rs)
        used = int(np.median([r["n_used"] for r in qual]))
        ok &= _line(worst > MIN_PEARSON_R,
                    f"pearson r  (worst of {len(qual)} qualifying {worst:.4f}, need > {MIN_PEARSON_R}; "
                    f"median {used} cells/frame)")
    return bool(ok)


def run_live(min_m, max_m, seconds):
    try:
        import rclpy
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import QoSProfile, ReliabilityPolicy
    except ImportError as exc:
        print(f"FAIL  rclpy not importable ({exc}); source the ROS 2 environment first")
        return 1

    rclpy.init()
    node = rclpy.create_node("probe_height_grid")
    pairer = Pairer(min_m, max_m)
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(OccupancyGrid, OGM1_TOPIC, lambda m: pairer.on_msg(1, m), qos)
    node.create_subscription(OccupancyGrid, OGM2_TOPIC, lambda m: pairer.on_msg(2, m), qos)

    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    return 0 if report(pairer) else 1


# --- self-test -------------------------------------------------------------------------------


def _synthetic_field(n=120, seed=7):
    """Slope + berm + ditch + noise, an unknown patch and two out-of-window patches (metres)."""
    rng = np.random.default_rng(seed)
    y, x = np.mgrid[0:n, 0:n].astype(np.float64)
    h = 0.015 * (x - n / 2.0)
    h += 1.2 * np.exp(-((x - 30) ** 2 + (y - 40) ** 2) / (2 * 6.0 ** 2))
    h -= 1.0 * np.exp(-((x - 80) ** 2 + (y - 70) ** 2) / (2 * 8.0 ** 2))
    h += rng.normal(0.0, 0.02, h.shape)
    h[100:104, 10:14] = 4.0   # above the +3 m window: clamps to level 100
    h[5:8, 100:103] = -3.0    # below the -2 m window: clamps to level 0
    h[50:60, 90:100] = np.nan  # unknown patch
    return h


def _encode_per_frame(h):
    """Mirror of the node's /debug_ogm_1 encoding: known cells min-max scaled to 1..100, else 255."""
    known = ~np.isnan(h)
    lo, hi = float(h[known].min()), float(h[known].max())
    v = (np.where(known, h, lo) - lo) * (99.0 / max(1e-6, hi - lo)) + 1.0
    v = np.where(known, np.floor(v + 0.5), 255.0)
    return v.astype(np.uint8).view(np.int8)  # the uint8 255 goes on the wire as int8 -1


def _encode_metric(h, min_m, max_m):
    """Mirror of encode_height_cell (perception_grid_map/height_encoding.h); unknown -> -1."""
    known = ~np.isnan(h)
    x = (np.where(known, h, min_m) - min_m) / (max_m - min_m) * LEVELS
    v = np.clip(np.floor(x + 0.5), 0, LEVELS)
    return np.where(known, v, UNKNOWN).astype(np.int8)


def _msg(stamp_sec, grid):
    """Minimal stand-in for nav_msgs/OccupancyGrid as far as Pairer.on_msg reads it."""
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp_sec, nanosec=0)),
        info=SimpleNamespace(width=grid.shape[1], height=grid.shape[0]),
        data=grid.reshape(-1).tolist())


def _run_pairer(frames, min_m, max_m):
    """Feed (ogm1, ogm2) frame pairs through a Pairer; returns (report ok, printed text)."""
    p = Pairer(min_m, max_m)
    for k, (a, b) in enumerate(frames):
        p.on_msg(1, _msg(k, a))
        p.on_msg(2, _msg(k, b))
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        ok = report(p, check_rate=False)  # synthetic feed has no meaningful arrival rate
    return ok, buf.getvalue()


def self_test():
    min_m, max_m = -2.0, 3.0

    d = decode_linear(np.array([-1, 0, 50, 100, 101, -5], dtype=np.int8), min_m, max_m)
    assert np.isnan(d[0]) and np.isnan(d[4]) and np.isnan(d[5]), d
    assert d[1] == -2.0 and d[2] == 0.5 and d[3] == 3.0, d

    h = _synthetic_field()
    o1 = _encode_per_frame(h)
    o2 = _encode_metric(h, min_m, max_m)

    good = compare(o1, o2, min_m, max_m)
    assert good["unknown_mask_equal"], good
    assert good["pearson_r"] > MIN_PEARSON_R, good
    assert good["n_known"] == int((~np.isnan(h)).sum()), good
    assert good["n_used"] < good["n_known"], good  # the clamped patches were excluded
    assert qualifies(good) and passes(good), good

    # One cell known in ogm1 but unknown in ogm2: the masks differ.
    one_off = o2.copy()
    one_off[tuple(np.argwhere(one_off != UNKNOWN)[0])] = UNKNOWN
    bad_mask = compare(o1, one_off, min_m, max_m)
    assert not bad_mask["unknown_mask_equal"] and not passes(bad_mask), bad_mask

    # Same known set but the values scrambled: masks equal, r near 0, still a qualifying frame.
    shuffled = o2.copy()
    idx = np.flatnonzero(shuffled != UNKNOWN)
    shuffled.flat[idx] = shuffled.flat[np.random.default_rng(1).permutation(idx)]
    bad_r = compare(o1, shuffled, min_m, max_m)
    assert bad_r["unknown_mask_equal"] and abs(bad_r["pearson_r"]) < 0.5, bad_r
    assert qualifies(bad_r) and not passes(bad_r), bad_r

    # Constant ogm2 (a dead encoder): one level, so the frame is excluded, not scored.
    dead = np.where(o2 != UNKNOWN, 50, UNKNOWN).astype(np.int8)
    bad_dead = compare(o1, dead, min_m, max_m)
    assert np.isnan(bad_dead["pearson_r"]) and not qualifies(bad_dead), bad_dead

    # Excluded-not-failed (a): an all-unknown pair, as right after the costmap node starts.
    unk = np.full(h.shape, UNKNOWN, dtype=np.int8)
    all_unknown = compare(unk, unk, min_m, max_m)
    assert all_unknown["unknown_mask_equal"] and all_unknown["n_used"] == 0, all_unknown
    assert np.isnan(all_unknown["pearson_r"]) and not qualifies(all_unknown), all_unknown

    # Excluded-not-failed (b): near-flat ground, 0.01 m noise on every known cell. ogm2 spans only a
    # couple of 0.05 m levels while ogm1 stretches the same noise over 1..100.
    flat_h = np.random.default_rng(3).normal(0.0, 0.01, h.shape)
    flat1 = _encode_per_frame(flat_h)
    flat2 = _encode_metric(flat_h, min_m, max_m)
    flat = compare(flat1, flat2, min_m, max_m)
    assert flat["unknown_mask_equal"] and flat["n_used"] >= MIN_USED_CELLS, flat
    assert flat["n_levels"] < MIN_LEVELS and not qualifies(flat), flat

    # Run-level verdicts through the real Pairer + report path (excluded frames never fail on r).
    goods = []
    for seed in range(12):
        hs = _synthetic_field(seed=seed)
        goods.append((_encode_per_frame(hs), _encode_metric(hs, min_m, max_m)))
    excluded = [(unk, unk), (flat1, flat2)]

    ok, text = _run_pairer(goods + excluded, min_m, max_m)
    assert ok, text                                  # 12 good + 2 excluded: PASS
    assert "excluded for insufficient relief  (2/14" in text, text

    ok, text = _run_pairer(goods[:9] + excluded, min_m, max_m)
    assert not ok and "FAIL  qualifying frames  (9" in text, text   # too few qualifying frames

    ok, text = _run_pairer([(o1, dead)] * 12, min_m, max_m)
    assert not ok and "FAIL  qualifying frames  (0" in text, text   # dead encoder cannot pass

    ok, text = _run_pairer(goods + [(unk, flat2)], min_m, max_m)
    assert not ok and "FAIL  unknown masks equal" in text, text     # masks are checked on every pair

    ok, text = _run_pairer(goods[:11] + [(o1, shuffled)], min_m, max_m)
    assert not ok and "FAIL  pearson r" in text, text               # one bad qualifying frame fails

    print(f"PASS  self-test  (r={good['pearson_r']:.5f}, n_used={good['n_used']}/{good['n_known']}, "
          f"3 broken cases rejected, 2 low-relief cases excluded, 5 run verdicts)")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description="Check /debug_ogm_2 metric heights against /debug_ogm_1.")
    ap.add_argument("--self-test", action="store_true", help="synthetic round trip, no ROS")
    ap.add_argument("--min-m", type=float, default=-2.0, help="encoding window floor [m]")
    ap.add_argument("--max-m", type=float, default=3.0, help="encoding window ceiling [m]")
    ap.add_argument("--seconds", type=float, default=10.0, help="live capture duration")
    args = ap.parse_args(argv)
    if args.self_test:
        return self_test()
    if args.max_m <= args.min_m:
        ap.error("--max-m must be greater than --min-m")
    return run_live(args.min_m, args.max_m, args.seconds)


if __name__ == "__main__":
    sys.exit(main())
```

Then:

```bash
cd $WORKTREE && chmod +x tools/probe_height_grid.py
```

- [ ] **Step 11: Run the self-test, expect PASS**

```bash
cd $WORKTREE && python3 tools/probe_height_grid.py --self-test; echo "exit=$?"
```

Expected: one line `PASS  self-test  (r=0.99..., n_used=N/M, 3 broken cases rejected, 2 low-relief cases excluded, 5 run verdicts)` and `exit=0`. If the flat-field assert `flat["n_levels"] < MIN_LEVELS` fails, the 0.01 m noise produced 5 or more levels (should not: sigma 0.2 level, so at most 3); do not loosen the test, check `_encode_metric`. Also run `python3 tools/probe_height_grid.py --help` (expected: usage text, exit 0) and `python3 tools/probe_height_grid.py --min-m 3 --max-m 3` (expected: `error: --max-m must be greater than --min-m`, exit 2), both from the worktree root.

- [ ] **Step 12: Prove the self-test fails when the logic is reverted (mutations on scratch copies)**

```bash
cd $WORKTREE
SCR=$SCRATCH
sed 's/ok = (v >= 0) \& (v <= LEVELS)/ok = (v >= 1) \& (v <= LEVELS)/' tools/probe_height_grid.py > $SCR/probe_mut1.py
python3 $SCR/probe_mut1.py --self-test; echo "exit=$?"
sed 's/ \& (b > 0) \& (b < LEVELS)//' tools/probe_height_grid.py > $SCR/probe_mut2.py
python3 $SCR/probe_mut2.py --self-test; echo "exit=$?"
sed 's/^MIN_LEVELS = 5 /MIN_LEVELS = 0 /' tools/probe_height_grid.py > $SCR/probe_mut3.py
python3 $SCR/probe_mut3.py --self-test; echo "exit=$?"
sed 's/^MIN_USED_CELLS = 100 /MIN_USED_CELLS = 0 /; s/^MIN_LEVELS = 5 /MIN_LEVELS = 0 /' tools/probe_height_grid.py > $SCR/probe_mut4.py
python3 $SCR/probe_mut4.py --self-test; echo "exit=$?"
```

Expected: all four die with `AssertionError` and `exit=1` (mutation 1 on the `decode_linear` level-0 assert, mutation 2 on `n_used < n_known`; mutations 3 and 4 also die with `AssertionError`, `exit=1`). If any prints PASS, the self-test does not guard that logic: stop and fix. (Live mode cannot run in CI; its acceptance run is Task 10: with the new perception build and costmap nodes up, `python3 tools/probe_height_grid.py --seconds 10` must print five PASS lines (rate, paired frames, unknown masks, qualifying frames, pearson r), at most one INFO excluded-frames line, and exit 0. On a flat lot expect many excluded frames; if fewer than 10 qualify, drive over varied terrain or lengthen `--seconds`, do not weaken the thresholds.)

- [ ] **Step 13: Syntax-only check of the live path without ROS**

```bash
cd $WORKTREE
python3 -I -c "import ast,sys; ast.parse(open('tools/probe_height_grid.py').read()); print('ok')"
python3 tools/probe_height_grid.py --seconds 1; echo "exit=$?"
```

Expected: `ok`; then, in a shell WITHOUT ROS sourced, `FAIL  rclpy not importable (...)` and `exit=1` (this exercises the lazy-import guard). In a ROS-sourced shell with no publishers the same command prints `FAIL  /debug_ogm_2 rate ...` and `FAIL  stamp-paired frames  (0 pairs...)`, exit 1.

- [ ] **Step 14: SPDX check, then the gate in the foreground**

`tools/ci_visual_mode.sh` does not run `tools/check_spdx.sh` (that is a lint job), so run it explicitly first:

```bash
cd $WORKTREE && git add -N tools/probe_height_grid.py && tools/check_spdx.sh
```

Expected: `check_spdx.sh: PASS (N files carry the SPDX identifier)` (`git add -N` makes the new tool visible to `git ls-files`, which that script uses for `tools/*.py`).

Then the gate. It resolves `REPO_ROOT` from its own location (nothing is hard-wired to `$PRIMARY_CHECKOUT`), so running it from the worktree builds and tests the worktree (library in `overlume/build`, node via the worktree's own `ros/install`). Stage 3 needs `ros/install/setup.bash` in the worktree; if `ls ros/install/setup.bash` fails, run `cd $WORKTREE && ros/colcon_build.sh` first (it installs into `ros/install`). Prior tasks' gate runs normally leave it in place. Never point the gate at `$PRIMARY_CHECKOUT`.

```bash
cd $WORKTREE && tools/ci_visual_mode.sh
```

Expected: all 6 stages pass and `OVERALL: PASS` (the new file is a tools script; stage 4 only runs `tools/test_vcam_ws_bridge.py`, which is unaffected). Do not background it.

- [ ] **Step 15: Commit only the probe (overlume worktree, branch `feat/height-grid`)**

```bash
cd $WORKTREE
git add tools/probe_height_grid.py
git status --short   # expected: only tools/probe_height_grid.py staged
git commit -m "feat(tools): probe_height_grid.py checks /debug_ogm_2 against /debug_ogm_1

Adds the live acceptance probe for the height-grid terrain feed. It pairs
/debug_ogm_1 (per-frame min-max) and /debug_ogm_2 (fixed metric window) by header
stamp and prints PASS/FAIL for: /debug_ogm_2 at >= 5 Hz, identical unknown masks on
every pair, at least 10 qualifying frames, and Pearson r > 0.99 on every
qualifying frame. A frame qualifies with >= 100 cells known in both and unclamped
in /debug_ogm_2 spanning >= 5 distinct levels; all-unknown frames after a node
start and near-flat ground (one or two 0.05 m levels) are reported on an INFO line
and excluded rather than failed, because r is undefined or meaningless there.
The comparison core (decode_linear, compare, qualifies) is pure numpy; --self-test
runs synthetic round trips, rejects mask-mismatch, scrambled and constant
encodings, excludes the all-unknown and flat-noise cases, and checks five
run-level verdicts through the real pairing path, so it fails when the logic is
reverted. rclpy is imported lazily so the self-test needs no ROS. The
perception-side encoder lives in the perception repo and is left uncommitted
there.

Claude-Session: https://claude.ai/code/session_018o7iLaY69qZAcNUfeD2VUi"
```

Do NOT stage or commit anything under `$PERCEPTION_REPO`; Part A stays as working-tree edits for the user.

---

### Task 9: Golden scenes (candidates, human promotion)

All commands run from the worktree root `$WORKTREE` (branch `feat/height-grid`). Never touch `$PRIMARY_CHECKOUT`.

**Files:**
- Modify `overlume/tests/golden.hpp`. Add `struct HeightGridScene` and the `make_height_grid_terrain_scene` declaration directly after the line `GridScene make_two_layer_grids(double now);` (just before `double render_and_compare(overlume::VisualRenderer* r, ...`).
- Modify `overlume/tests/golden.cpp`. Add `#include <limits>` between `#include <fstream>` and `#include <sstream>` (the include block is `<algorithm> <cmath> <cstdint> <fstream> <sstream> <string> <tuple> <vector>`). Add the builder after the closing brace of `make_two_layer_grids` and before `overlume::Vec3 centroid(const std::vector<overlume::MapElement>& elems) {`.
- Create `overlume/tests/test_height_grid_golden.cpp`. `overlume/CMakeLists.txt` globs `tests/*.cpp` with `CONFIGURE_DEPENDS` and links `tests/golden.cpp` into every test executable, so no CMake edit is needed. Target name is `test_height_grid_golden`. `OVERLUME_TEST_DATA_DIR` and `OVERLUME_DEFAULT_THEME_DIR` are compile definitions set per test target there.
- Modify `docs/status.md`: add one row to the `## Shipped` table (Step 10), the status ledger entry for the promoted goldens.
- Create (human-gated, only after explicit user approval) `overlume/tests/goldens/height_grid_terrain_dark_adas.png`, `overlume/tests/goldens/height_grid_terrain_light_clay.png`, `overlume/tests/goldens/height_grid_terrain_yawed_dark_adas.png`. `.gitattributes` has `*.png filter=lfs diff=lfs merge=lfs -text`, so `git add` stores LFS pointers (git-lfs 3.0.2 is installed). Existing goldens in the worktree are real PNGs (smudged), and the repo's last promotion (a3baf88, "test(goldens): promote the restored hybrid capture (maintainer-approved)") was a plain copy over the file plus a commit with a `Claude-Session:` trailer; we follow the same flow.

**Interfaces:**
- Consumes: Task 1 (`overlume::HeightGridLayer`, `SceneGraph::height_grids` / `height_grid_count`; scene version 9, `sizeof(HeightGridLayer)` 64, `sizeof(SceneGraph)` 232), Task 3 (renderer draws height grids), Task 4 (ground plane gets a hole under the grid including a rotated footprint; it must keep the hybrid-splat stencil state on `groundMaterial`/`gridMaterial` working, but that is Task 4's concern), plus existing `overlume::GroundGridLayer` (fields `kind, origin, resolution_m, width_cells, height_cells, cells, last_update_sec, yaw_rad`, scene.h:87-95), `overlume::testing::render_and_compare` (golden.hpp), `kThemeDir` (`test_paths.hpp`), `OVERLUME_TEST_DATA_DIR`. All of Tasks 1-4 must be committed first, otherwise the candidates would not show the feature and the human review is meaningless. Not consumed (they do not exist on this branch): `OVERLUME_TMP_DIR`, `kSsimMin`.
- Status ledger: this task itself adds the `docs/status.md` row for the promoted goldens (Step 10); no later task is relied on for it.
- Produces:
    ```cpp
    // golden.hpp, namespace overlume::testing
    struct HeightGridScene {
        std::vector<float> heights;                        // row-major, heights[j * width + i]; NaN = unknown
        std::vector<uint8_t> cost_cells;                   // 120x120 cost grid, same footprint
        std::vector<overlume::HeightGridLayer> height_grids;  // exactly 1
        std::vector<overlume::GroundGridLayer> grids;         // exactly 1 (kind 1, geometric)
        /* deleted copy, defaulted move: same idiom as GridScene */
    };
    HeightGridScene make_height_grid_terrain_scene(double now, double yaw_rad = 0.0);
    ```
    Golden PNGs `height_grid_terrain_dark_adas.png`, `height_grid_terrain_light_clay.png`, `height_grid_terrain_yawed_dark_adas.png` (320x240). Tests: `HeightGridTerrainScene.BuilderPlacesTheFourFeatures`, `HeightGridTerrainScene.YawRotatesBothGridsAndKeepsHeights` (CPU), `HeightGridGolden.TerrainDarkAdas`, `HeightGridGolden.TerrainLightClay`, `HeightGridGolden.TerrainYawedDarkAdas`. The suite name must contain `Golden`: gate stage 5 counts `[ OK ] ... Golden...` lines (tools/ci_visual_mode.sh:150-153).

**Scene design (fixed numbers).** The grid is 120x120 cells at 0.2 m (24 m x 24 m). Origin (4, -12, 0), yaw 0 (or `yaw_rad` for the yawed golden), so at yaw 0 it spans x 4..28 and y -12..12, centred 16 m ahead of an ego at the origin. Cell (i, j) is column i along grid +x, row j along grid +y, stored as `heights[j*120+i]`. Rotation is about the origin corner (4, -12), identical for the height grid and the cost grid.
- Berm: columns i in [8, 52), tent ridge centred at row 30, half-width 12 rows, peak +1.2 m.
- Ditch: same columns, tent trench centred at row 90, half-width 12 rows, depth -0.8 m.
- Sloped half: columns i >= 64, linear ramp, h = 1.5*(i-64)/(119-64).
- Unknown patch: columns [24, 40), rows [62, 74) are NaN, in the flat strip between berm and ditch (rows 42..78).
- Cost grid: kind 1, same origin, yaw and 0.2 m resolution. Cost-100 block at columns [12,22) x rows [48,58), cost-70 block at columns [40,48) x rows [50,54), one cost-90 cell at (30,45). All in the flat strip.
- Camera: `CameraPose{{-4, -16, 10}, {16, 0, 0}, 60.0}` for all three goldens (yawed one uses yaw 0.35 rad, ~20 deg).
- Deviation from the spec: spec section 9 names two goldens (`dark_adas`, `light_clay`). The third golden (yaw 0.35) goes beyond spec section 9; it covers the rotated hole from spec section 6. If the human reviewer rejects it, drop it (the `TerrainYawedDarkAdas` test, its PNG, and its Step 5/7/10 mentions) and keep the two the spec requires.

- [ ] **Step 1: Add the declaration to `overlume/tests/golden.hpp`**

Use Edit: old_string `GridScene make_two_layer_grids(double now);` new_string that same line followed by a blank line and:

```cpp
struct HeightGridScene {
    std::vector<float> heights;
    std::vector<uint8_t> cost_cells;
    std::vector<overlume::HeightGridLayer> height_grids;
    std::vector<overlume::GroundGridLayer> grids;

    HeightGridScene() = default;
    HeightGridScene(const HeightGridScene&) = delete;
    HeightGridScene& operator=(const HeightGridScene&) = delete;
    HeightGridScene(HeightGridScene&&) = default;
    HeightGridScene& operator=(HeightGridScene&&) = default;
};

// 120x120 grid at 0.2 m centred ahead of an ego at the origin: a +1.2 m berm, a -0.8 m ditch,
// a 0 -> +1.5 m ramp half, a NaN (unknown) patch, and a cost grid with a few high-cost cells.
// yaw_rad rotates both grids about their shared origin corner.
HeightGridScene make_height_grid_terrain_scene(double now, double yaw_rad = 0.0);
```

- [ ] **Step 2: Add the builder to `overlume/tests/golden.cpp`**

First Edit the include block: old_string `#include <fstream>\n#include <sstream>` new_string `#include <fstream>\n#include <limits>\n#include <sstream>`. Then Edit with old_string `overlume::Vec3 centroid(const std::vector<overlume::MapElement>& elems) {` (unique) and prepend:

```cpp
HeightGridScene make_height_grid_terrain_scene(double now, double yaw_rad) {
    constexpr uint32_t kW = 120, kH = 120;
    constexpr double kRes = 0.2;
    constexpr overlume::Vec3 kOrigin{4.0, -12.0, 0.0};

    HeightGridScene s;
    s.heights.assign(static_cast<size_t>(kW) * kH, 0.0f);
    auto tent = [](uint32_t row, double centre, double half) {
        return std::max(0.0, 1.0 - std::abs(static_cast<double>(row) - centre) / half);
    };
    for (uint32_t j = 0; j < kH; ++j) {
        for (uint32_t i = 0; i < kW; ++i) {
            double h = 0.0;
            if (i >= 8 && i < 52) h = 1.2 * tent(j, 30.0, 12.0) - 0.8 * tent(j, 90.0, 12.0);
            if (i >= 64) h = 1.5 * static_cast<double>(i - 64) / static_cast<double>(kW - 1 - 64);
            if (i >= 24 && i < 40 && j >= 62 && j < 74) {
                s.heights[static_cast<size_t>(j) * kW + i] =
                    std::numeric_limits<float>::quiet_NaN();
                continue;
            }
            s.heights[static_cast<size_t>(j) * kW + i] = static_cast<float>(h);
        }
    }

    s.cost_cells.assign(static_cast<size_t>(kW) * kH, 0);
    for (uint32_t j = 48; j < 58; ++j)
        for (uint32_t i = 12; i < 22; ++i) s.cost_cells[static_cast<size_t>(j) * kW + i] = 100;
    for (uint32_t j = 50; j < 54; ++j)
        for (uint32_t i = 40; i < 48; ++i) s.cost_cells[static_cast<size_t>(j) * kW + i] = 70;
    s.cost_cells[static_cast<size_t>(45) * kW + 30] = 90;

    overlume::HeightGridLayer hg{};
    hg.origin = kOrigin;
    hg.yaw_rad = yaw_rad;
    hg.resolution_m = kRes;
    hg.width_cells = kW;
    hg.height_cells = kH;
    hg.heights_m = s.heights.data();
    hg.last_update_sec = now;
    s.height_grids.push_back(hg);

    overlume::GroundGridLayer cost{};
    cost.kind = 1;
    cost.origin = kOrigin;
    cost.resolution_m = kRes;
    cost.width_cells = kW;
    cost.height_cells = kH;
    cost.cells = s.cost_cells.data();
    cost.last_update_sec = now;
    cost.yaw_rad = yaw_rad;
    s.grids.push_back(cost);
    return s;
}

```

(The builder lives inside the existing `namespace overlume::testing`, like its neighbours. The field names `origin, yaw_rad, resolution_m, width_cells, height_cells, heights_m, last_update_sec` are Task 1's `HeightGridLayer` contract; if the committed Task 1 header spells any differently, fix the builder to match the header, not the other way round.)

- [ ] **Step 3: Create `overlume/tests/test_height_grid_golden.cpp`**

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Amer Ghazal

#include "overlume/api.h"
#include "overlume/scene.h"

#include "golden.hpp"
#include "test_paths.hpp"

#include <cmath>
#include <string>

#include <gtest/gtest.h>

namespace {

// Renders the synthetic terrain scene under `theme` and compares to `golden_name`. The actual frame
// lands in /tmp/<golden_name>_actual.png (the candidate a human promotes). Returns -1 without a
// renderer (no GPU/EGL), 0.0 when the golden is missing or the frame fails.
double render_terrain(const char* theme, const std::string& golden_name, double yaw_rad) {
    overlume::RenderConfig cfg{320, 240, 1, kThemeDir, theme};
    auto* r = overlume::create_renderer(cfg);
    if (!r) return -1.0;

    overlume::testing::HeightGridScene t =
        overlume::testing::make_height_grid_terrain_scene(10.0, yaw_rad);
    overlume::SceneGraph s{};
    s.sim_time_sec = 10.0;
    s.ego = {{0, 0, 0}, 0.0, 0.0, 1};
    s.height_grids = t.height_grids.data();
    s.height_grid_count = static_cast<uint32_t>(t.height_grids.size());
    s.grids = t.grids.data();
    s.grid_count = static_cast<uint32_t>(t.grids.size());
    overlume::set_scene(r, s);

    const std::string golden =
        std::string(OVERLUME_TEST_DATA_DIR) + "/tests/goldens/" + golden_name + ".png";
    const std::string actual = std::string("/tmp/") + golden_name + "_actual.png";
    const overlume::CameraPose pose{{-4, -16, 10}, {16, 0, 0}, 60.0};
    const double ssim =
        overlume::testing::render_and_compare(r, pose, golden.c_str(), actual.c_str());
    overlume::destroy_renderer(r);
    return ssim;
}

}

// CPU-only: guards the builder itself, so a golden can't silently pin a flat or empty scene.
TEST(HeightGridTerrainScene, BuilderPlacesTheFourFeatures) {
    const auto t = overlume::testing::make_height_grid_terrain_scene(10.0);
    ASSERT_EQ(t.height_grids.size(), 1u);
    ASSERT_EQ(t.grids.size(), 1u);
    const auto& g = t.height_grids[0];
    ASSERT_EQ(g.width_cells, 120u);
    ASSERT_EQ(g.height_cells, 120u);
    auto h = [&](uint32_t i, uint32_t j) { return g.heights_m[j * g.width_cells + i]; };
    EXPECT_NEAR(h(30, 30), 1.2f, 1e-4) << "berm crest";
    EXPECT_NEAR(h(30, 90), -0.8f, 1e-4) << "ditch floor";
    EXPECT_NEAR(h(119, 10), 1.5f, 1e-4) << "ramp top";
    EXPECT_NEAR(h(64, 10), 0.0f, 1e-4) << "ramp foot";
    EXPECT_TRUE(std::isnan(h(30, 68))) << "unknown patch";
    EXPECT_FALSE(std::isnan(h(30, 40)));
    EXPECT_EQ(t.grids[0].cells[48 * 120 + 12], 100);
    EXPECT_EQ(t.grids[0].width_cells, g.width_cells);
    EXPECT_DOUBLE_EQ(g.yaw_rad, 0.0);
    EXPECT_DOUBLE_EQ(t.grids[0].yaw_rad, 0.0);
}

// CPU-only: the yawed variant rotates both grids by the same angle about the same corner and keeps
// the heights identical, so the yawed golden differs from the plain one only by placement.
TEST(HeightGridTerrainScene, YawRotatesBothGridsAndKeepsHeights) {
    const auto plain = overlume::testing::make_height_grid_terrain_scene(10.0);
    const auto yawed = overlume::testing::make_height_grid_terrain_scene(10.0, 0.35);
    EXPECT_DOUBLE_EQ(yawed.height_grids[0].yaw_rad, 0.35);
    EXPECT_DOUBLE_EQ(yawed.grids[0].yaw_rad, 0.35);
    EXPECT_DOUBLE_EQ(yawed.height_grids[0].origin.x, plain.height_grids[0].origin.x);
    EXPECT_DOUBLE_EQ(yawed.grids[0].origin.y, plain.grids[0].origin.y);
    ASSERT_EQ(yawed.heights.size(), plain.heights.size());
    EXPECT_NEAR(yawed.heights[30 * 120 + 30], 1.2f, 1e-4);
}

TEST(HeightGridGolden, TerrainDarkAdas) {
    const double ssim = render_terrain("dark_adas", "height_grid_terrain_dark_adas", 0.0);
    if (ssim < 0.0) GTEST_SKIP() << "no GPU/EGL";
    EXPECT_GT(ssim, 0.98);
}

TEST(HeightGridGolden, TerrainLightClay) {
    const double ssim = render_terrain("light_clay", "height_grid_terrain_light_clay", 0.0);
    if (ssim < 0.0) GTEST_SKIP() << "no GPU/EGL";
    EXPECT_GT(ssim, 0.98);
}

// Pixel coverage for yaw != 0: the grid and the ground-plane hole under it are both rotated, so a
// hole that ignored yaw would show ground through the terrain or a missing ground strip.
TEST(HeightGridGolden, TerrainYawedDarkAdas) {
    const double ssim = render_terrain("dark_adas", "height_grid_terrain_yawed_dark_adas", 0.35);
    if (ssim < 0.0) GTEST_SKIP() << "no GPU/EGL";
    EXPECT_GT(ssim, 0.98);
}
```

- [ ] **Step 4: Build and run; the golden tests must FAIL**

Do not start this while another build is running in `overlume/build`; wait for it to finish.

```bash
cd $WORKTREE
cmake --toolchain "$PWD/overlume/cmake/toolchain-clang-libcxx.cmake" -S overlume -B overlume/build -DOVERLUME_ENABLE_CESIUM=ON
cmake --build overlume/build -j --target test_height_grid_golden
ctest --test-dir overlume/build -R 'HeightGrid(Terrain|Golden)' --output-on-failure
```

Expected:
- Both `HeightGridTerrainScene.*` CPU tests pass. These two pin the builder and are expected green on first run (before the builder exists they only fail to compile); the red-to-green evidence for this task is the three goldens failing at SSIM 0 before promotion and passing after.
- `HeightGridGolden.TerrainDarkAdas`, `TerrainLightClay` and `TerrainYawedDarkAdas` FAIL with `Expected: (ssim) > (0.98), actual: 0 vs 0.98`. The golden file is missing, so `render_and_compare` returns 0.0 after writing the frame (golden.cpp: `if (golden == nullptr) return 0.0;`).
- If instead they show SKIPPED, this machine has no GPU/EGL. Stop and run on the GPU rig, because candidates cannot be produced without it.
- If the CPU tests fail to compile, a `HeightGridLayer` field name differs from the Task 1 header; fix the builder to match.

- [ ] **Step 5: Collect the candidates (no repo writes)**

```bash
cd $WORKTREE
ls -l /tmp/height_grid_terrain_dark_adas_actual.png /tmp/height_grid_terrain_light_clay_actual.png /tmp/height_grid_terrain_yawed_dark_adas_actual.png
# determinism: re-run and confirm byte-identical candidates before showing them
for n in dark_adas light_clay yawed_dark_adas; do cp /tmp/height_grid_terrain_${n}_actual.png /tmp/hgt_${n}_run1.png; done
ctest --test-dir overlume/build -R HeightGridGolden >/dev/null || true
for n in dark_adas light_clay yawed_dark_adas; do cmp /tmp/hgt_${n}_run1.png /tmp/height_grid_terrain_${n}_actual.png; done
```

Expected: all three `cmp` calls print nothing (deterministic render). If they differ, stop and investigate nondeterminism (a stale-fade alpha or an animated element). Do not promote a flaky candidate.

- [ ] **Step 6: HUMAN GATE. The orchestrator shows all three candidates to the user; nothing is promoted without explicit approval**

The orchestrator (not the implementer) displays `/tmp/height_grid_terrain_dark_adas_actual.png`, `/tmp/height_grid_terrain_light_clay_actual.png` and `/tmp/height_grid_terrain_yawed_dark_adas_actual.png` to the user, with this checklist:
1. The berm reads as a raised ridge, the ditch as a trench, the ramp half as a smooth rise toward +x. Height-ramp colours differ visibly between low and high.
2. The unknown (NaN) patch renders in the theme's `unknown_color`, with no hole and no spike.
3. The flat ground plane does not show through or z-fight under the grid (Task 4 hole), and the surrounding ground is still drawn.
4. The cost-grid cells (one 10x10 block, one 8x4 block, one single cell) are visible on the flat strip.
5. Dark and light themes both look intentional.
6. Yawed golden: the terrain is rotated about 20 degrees about its near corner, the cost cells rotate with it, and the ground hole edge follows the rotated footprint (no sliver of ground inside the terrain, no ground strip missing just outside it).

Outcomes:
- The user approves all three: go to Step 7.
- The user rejects the yawed golden only: drop it per the Scene design note and promote the other two.
- The user rejects or asks for changes otherwise: adjust the builder or camera, return to Step 4, re-show. Never copy a candidate the user has not approved. A failing golden is never overwritten to make it pass.

- [ ] **Step 7: Promote on approval (exact names)**

```bash
cd $WORKTREE
cp /tmp/height_grid_terrain_dark_adas_actual.png        overlume/tests/goldens/height_grid_terrain_dark_adas.png
cp /tmp/height_grid_terrain_light_clay_actual.png       overlume/tests/goldens/height_grid_terrain_light_clay.png
cp /tmp/height_grid_terrain_yawed_dark_adas_actual.png  overlume/tests/goldens/height_grid_terrain_yawed_dark_adas.png
```

- [ ] **Step 8: Rebuild everything, then re-run; must PASS**

Step 4 built only `test_height_grid_golden`, so rebuild all targets first; otherwise `ctest` would run stale binaries for every other suite.

```bash
cd $WORKTREE
cmake --build overlume/build -j
ctest --test-dir overlume/build -R 'HeightGrid(Terrain|Golden)' --output-on-failure
ctest --test-dir overlume/build --output-on-failure
```

Expected: 5/5 of the first run pass (SSIM 1.0 against itself); the full library suite is green on freshly linked binaries. This also confirms the pre-existing goldens still match after Tasks 1-4.

- [ ] **Step 9: Run the gate in the foreground, from the worktree**

`tools/ci_visual_mode.sh` derives `REPO_ROOT` from its own location, so it runs against the worktree with no path edits (library build goes to the worktree's `overlume/build`). Stage 3 FAILS if `ros/install/setup.bash` is missing (tools/ci_visual_mode.sh lines 95-103, "build it first"), so build it with `ros/colcon_build.sh` first (it needs `overlume/build/liboverlume.a`, which Step 8 produced):

```bash
cd $WORKTREE
ros/colcon_build.sh                       # writes ros/install in the worktree; skip if it already exists
tools/ci_visual_mode.sh                   # foreground, never backgrounded
# alternative if you must borrow another install space instead of building:
#   CI_VISUAL_MODE_ROS_APPS_INSTALL=/path/to/ros/install/setup.bash tools/ci_visual_mode.sh
```

Expected: all 6 stages PASS. Stage 5 reports the golden `ok` count up by 3 (the three new `HeightGridGolden` tests; the `HeightGridTerrainScene` tests are not counted there).

- [ ] **Step 10: Add the status row and commit**

First append one row to the end of the `## Shipped` table in `docs/status.md` (Edit: anchor on the table's last row, the line directly before the blank line that precedes `## Open items`, and add this row after it, matching the `| text | date | — |` column shape):

```
| Height-grid terrain goldens `height_grid_terrain_dark_adas`, `height_grid_terrain_light_clay` and `height_grid_terrain_yawed_dark_adas` (synthetic 120x120 @ 0.2 m scene: berm, ditch, ramp, NaN patch, cost overlay; yaw 0.35 rad covers the rotated ground hole) promoted by the user after viewing the candidates | 2026-10-06 | — |
```

Use the actual date of promotion if it differs from 2026-10-06.

```bash
cd $WORKTREE
git add overlume/tests/golden.hpp overlume/tests/golden.cpp overlume/tests/test_height_grid_golden.cpp docs/status.md \
        overlume/tests/goldens/height_grid_terrain_dark_adas.png \
        overlume/tests/goldens/height_grid_terrain_light_clay.png \
        overlume/tests/goldens/height_grid_terrain_yawed_dark_adas.png
# the staged PNGs must be LFS pointers, not raw blobs (expect no output)
for f in overlume/tests/goldens/height_grid_terrain_{dark_adas,light_clay,yawed_dark_adas}.png; do
  git show ":$f" | head -1 | grep -q '^version https://git-lfs' || echo "NOT LFS: $f"
done
git commit -m "test(goldens): height-grid terrain goldens, promoted by the user

Adds make_height_grid_terrain_scene (120x120 @ 0.2 m: +1.2 m berm, -0.8 m ditch,
0 -> +1.5 m ramp half, NaN unknown patch, plus a cost-grid overlay with high-cost
cells, optional yaw) and three goldens: height_grid_terrain_dark_adas,
height_grid_terrain_light_clay and height_grid_terrain_yawed_dark_adas (yaw 0.35 rad,
covers the rotated footprint and its ground hole in pixels). Candidates were
rendered, shown to the user, and promoted only after the user explicitly approved
them. CPU tests pin the builder's features and yaw so the goldens cannot silently
freeze a flat, empty or unrotated scene. docs/status.md records the promotion.

Claude-Session: https://claude.ai/code/session_018o7iLaY69qZAcNUfeD2VUi"
git lfs ls-files | grep height_grid_terrain   # after the commit: all three listed
```

**Per-pixel drift audit (AGENTS.md rule): does not apply to the new goldens.** The audit is required after palette-wide changes, where a golden can pass whole-frame SSIM >= 0.98 while pinning a stale palette. These three are brand new, rendered from current code and current `dark_adas` / `light_clay` palettes and approved by eye, so there is no older baseline to drift from. What does carry that risk is the existing goldens, if Task 2 or Task 4 changed any shared token or the ground material. Step 8's full-suite run only proves SSIM >= 0.98 for those. If any existing golden's `*_actual.png` differs from its committed PNG, the audit applies to that change, not to this task. Do not re-promote existing goldens in this commit.

---

### Task 10: Docs, status ledger and live acceptance

**Files:**
- Modify: `docs/status.md` (Shipped table, after the last row)
- Modify: `docs/runbooks/theme_showcase.md` (theme token documentation)
- Modify: `docs/plans/2026-10-06-height-grid-terrain.md` (this plan's Status ledger)
- Modify: `CHANGELOG.md` `[Unreleased]` (feature line, next to Task 1's `kSceneVersion` line)

**Interfaces:**
- Consumes: everything from Tasks 1–9. That includes `tools/probe_height_grid.py` (Task 8), the `layer_height_grids` param (Task 7), the `robot-offroad` profile row (Task 7) and the `/debug_ogm_2` publisher in the perception build (Task 8).
- Produces: the closed ledger and the acceptance evidence.

- [ ] **Step 1: Document the theme tokens**

In `docs/runbooks/theme_showcase.md`, add a `height_grid` subsection where the file lists the other theme blocks:

```markdown
### `height_grid` (terrain heightfield)

| Key | Type | Default | Meaning |
|---|---|---|---|
| `ramp` | list of `{height_m, color}` | `palette.ground` at 0.0 m, alert warning at 2.5 m | Colour by height above nominal ground; linear between stops, clamped outside them. |
| `unknown_color` | `[r, g, b]` | `palette.ground` | Colour of cells the perception node has no height for (drawn flat at ground level). |
| `roughness` | float | 0.9 | Lit-material roughness of the terrain. |
| `ground_bias_m` | float | −0.05 | Added to every vertex height; keeps flat terrain just under the cost grids (+0.010 / +0.015 m). |
```

- [ ] **Step 2: Changelog feature line**

Under `CHANGELOG.md` `[Unreleased]`, next to Task 1's `kSceneVersion` entry, add:

```markdown
- Visual mode draws the offroad terrain height map (`height_grid` profile
  adapter, `/debug_ogm_2` metric encoding) as a shaded heightfield that
  replaces the clay ground inside its footprint; theme block `height_grid`,
  disable with `layer_height_grids:=false`.
```

- [ ] **Step 3: Gate**

Run: `cd $WORKTREE && tools/ci_visual_mode.sh` (foreground)
Expected: all six stages pass. Golden counts include the two `height_grid_terrain_*` goldens as OK.

- [ ] **Step 4: Rebuild perception and restart the costmap nodes (user action)**

The live stack runs from `$PERCEPTION_REPO/install`. The orchestrator asks the user to rebuild it, then restart `geometric_cost_map_node` and `dynamic_cost_map_node` from their terminals. The rebuild must be FULL: first the whole perception libs superproject, then all of `ros_apps`. Task 8's new `CostMap` members change its size and layout, and every library or node that holds a `CostMap` by value (dynamic, semantic, objects, geometric) must be rebuilt together. A partial colcon rebuild, or a node built against the old installed `libperception_grid_map.so`, is an ODR/ABI mismatch. (Task 8 built only scratch compile checks; it established no live rebuild command.) Then check:

Run: `cd $WORKTREE && source /opt/ros/humble/setup.bash && ROS_DOMAIN_ID=7 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp python3 tools/probe_height_grid.py --seconds 10`
Expected: `PASS` on the rate, unknown-mask and correlation lines, exit code 0.

The probe's checks do not depend on scale or offset (mask equality and Pearson r), so a wrong `origin_plane_z` subtraction or a wrong window in `update_cost_map` would still PASS. Nothing else checks that fill automatically (accepted under Known limits). So also check by eye: flat ground under the ego must decode to about 0 m on `/debug_ogm_2` (raw value about 40 with the −2.0 .. +3.0 m window).

- [ ] **Step 5: Live visual acceptance (spec §11)**

Run: `cd $WORKTREE && LIVE_SIM_TIME=true tools/validate_visual_mode.sh --live --profile robot-offroad` (runs the worktree's `ros/install`; stop any live rig started from the main checkout first, since both use the same ROS domain and node name)
Check in mode 3, and capture a frame of each for the user:
1. Terrain is visible; berms and walls rise through the cost grids; ditches and downhill slopes are visible (not buried by the clay ground).
2. `ros2 param set /overlume_node layer_height_grids false` hides the terrain and the clay ground comes back (the `/debug_ogm_2` diagnostics row keeps counting msgs while hidden); `true` restores it. The node log shows `height_grid: 1 row(s) subscribed` at startup. (Task 7 review: the subscription, per-tick gate, diagnostics row and teardown clears are covered only by compilation; these checks are their acceptance.)
3. Modes 1 and 2 show no terrain.
4. `/overlume_node/diagnostics` has a `/debug_ogm_2` row with `message: ok`, `dropped_malformed: 0`, `dropped_no_tf: 0`. Its `render_ms` p50 is within 3 ms of a run with `layer_height_grids false` (acceptance 3).

- [ ] **Step 6: Ledgers**

- In this plan's Status ledger, mark each task done with its commit hash (from `git log --oneline main..feat/height-grid`).
- In `docs/status.md`, record the known gap from the Task 7 review: `LAYER_NAMES` in `tools/vcam_ws_bridge.py` / `tools/vcam_gui.py` has no `height_grids` entry, so the terrain toggles only via `ros2 param set /overlume_node layer_height_grids`, not from the GUI.
- In `docs/status.md`, add a Shipped row:

```markdown
| Height-grid terrain layer (offroad height map as a heightfield, `kSceneVersion` 9) | CLOSED <date> at `<last commit>` | [`plans/2026-10-06-height-grid-terrain.md`](plans/2026-10-06-height-grid-terrain.md) |
```

Use the real date and hash from git. Never invent them.

- [ ] **Step 7: Commit**

```bash
cd $WORKTREE
git add docs/status.md docs/runbooks/theme_showcase.md docs/plans/2026-10-06-height-grid-terrain.md CHANGELOG.md
git commit -F - <<'EOF'
docs: close the height-grid terrain plan

Theme token docs, changelog feature line, status ledger row and the plan's
own ledger, after the live CARLA offroad acceptance run (probe PASS,
terrain visible in mode 3, layer_height_grids toggles it, modes 1-2 clean).

Claude-Session: https://claude.ai/code/session_018o7iLaY69qZAcNUfeD2VUi
EOF
```
