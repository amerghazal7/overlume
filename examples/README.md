# overlume examples

Six small, self-contained C++ programs against overlume's public API
(`<overlume/api.h>`, `<overlume/scene.h>`) only — none of them includes
anything from `overlume/src`. Each renders a headless frame and writes it
out as an image file, then exits 0.

The generated Doxygen reference these programs are written against is
published at <https://amerghazal7.github.io/overlume/>; how to build it
locally is in the repository's
[docs/README.md](https://github.com/amerghazal7/overlume/blob/main/docs/README.md#api-documentation)
(absolute link so it also works from the published copy of this page).

## Building and running

Built as part of the ordinary library configure, under
`OVERLUME_BUILD_EXAMPLES` (default `ON`):

```sh
cmake --toolchain "$PWD/overlume/cmake/toolchain-clang-libcxx.cmake" -S overlume -B overlume/build -DOVERLUME_ENABLE_CESIUM=ON
cmake --build overlume/build -j"$(nproc)"
./overlume/build/examples/01_hello_frame
```

(A relative `--toolchain` path resolves against the build tree first, the
source tree second — with `-S overlume` a relative path here would become
`overlume/overlume/cmake/...`, which doesn't exist. Run this from the repo
root; the absolute form above is what `README.md` and `AGENTS.md` use too.)

Each program takes three optional arguments: an output image path (default:
the example's own name, e.g. `01_hello_frame.png`), a theme assets directory
(default: the shipped `overlume/assets/themes`, baked in at compile time),
and a per-class model directory (default: the shipped `overlume/assets/models`,
same baked-in pattern — the car/truck_van/pedestrian glTF/GLB clay models
`set_object_model_dir()` loads; see `overlume/assets/models/ATTRIBUTION.md`
for their CC0 sources, and the note under `02`/`04` below for which classes
still fall back to the procedural clay box). For example:

```sh
./overlume/build/examples/02_scene_population /tmp/out.png overlume/assets/themes overlume/assets/models
```

## Image format

Every example writes a **PNG** via `stb_image_write.h` — a single-header,
public-domain encoder already vendored by `overlume/CMakeLists.txt` for the
test suite's own golden captures, so using it here adds no new third-party
dependency.

## The examples

1. **`01_hello_frame`** — the smallest possible program: `create_renderer()`,
   one `CameraPose`, one `render_frame()`, write the PNG. Never calls
   `set_scene()` — rendering with no published scene is a legal, fully
   defined state. Its only `<overlume/scene.h>` call is
   `theme_assets_loaded()`, to report whether the theme dir loaded.
2. **`02_scene_population`** — populates the core driving-scene categories:
   the ego, one `TrackedObject` per `ObjectClass`, one `MapElement` per
   `MapKind` (including `ROAD_SURFACE` and `CROSSWALK`), and one
   behavior-role `PathRibbon`. (The scene graph also carries ground grids,
   generic markers, HUD, and trajectory carpets, which no example builds;
   point clouds and alerts are covered separately, in `06`.) Calls
   `set_scene()` once and `render_frame()` twice with no `set_scene()` in
   between to demonstrate the freeze-frame/double-buffer contract
   explicitly: the second frame must reproduce the first, because
   `render_frame()` always re-derives the picture from whatever was last
   *published*, not from anything render time itself changes. Calls
   `set_object_model_dir()` right after `create_renderer()`, so `CAR`,
   `TRUCK_VAN`, and `PEDESTRIAN` render as their real glTF meshes; `BUS`,
   `CYCLIST` (no shipped CC0 model — see `ATTRIBUTION.md`) and `UNKNOWN`
   (always the clay box, by design) still render as the procedural clay box.
3. **`03_themes`** — loads both shipped themes (`dark_adas`, `light_clay`),
   calls `set_theme()` to start a transition between them, and renders using
   the deterministic clock the API exposes (`SceneGraph::sim_time_sec`)
   rather than wall-clock time, so the mid-transition frame it writes is
   reproducible.
4. **`04_virtual_camera`** — a small set of named camera "presets" (plain
   `CameraPose` literals — the public API has no preset registry of its
   own), a linear tween between two of them, and `project_to_screen()` to
   project a world point onto each pose's screen. Also calls
   `set_object_model_dir()` after `create_renderer()`; its three
   `TrackedObject`s (`CAR`, `TRUCK_VAN`, `PEDESTRIAN`) all have shipped
   models, so none of them fall back to the clay box here.
5. **`05_environment`** — loads a baked environment chunk index from the
   same committed fixture directory the library's own environment tests
   use (`overlume/tests/fixtures/environment_test_town_0`). Only if
   `CESIUM_ION_TOKEN` is set in the environment does it also try the
   streaming (`ion://96188`) backend briefly; otherwise it prints one line
   saying streaming was skipped and exits 0 — a CI run has no token, and
   this example never needs one to pass. The streaming path exists only when
   the library was configured with `-DOVERLUME_ENABLE_CESIUM=ON` (the default since 2026-09-18; hosted CI passes OFF); with it OFF the open call returns false and the example continues.
6. **`06_overlays_and_pointcloud`** — a synthetic point cloud, a
   warning-severity alert polygon, a live HUD-color query
   (`get_hud_colors()`), and a live quality-preset switch
   (`set_quality()`/`get_quality()`) against the same renderer, no
   re-create. Publishes no `TrackedObject`s, so it doesn't call
   `set_object_model_dir()` — there's nothing for a class model to apply to.

## The reference integration app

These examples show the raw library API in isolation. The real, full
integration lives at `ros/src/overlume_ros`: a ROS 2 lifecycle node whose
**adapters** translate incoming ROS messages (TF, detections, point clouds,
map layers, camera images, ...) into the POD structs these examples build
by hand, and whose **profile YAML** maps ROS topics to those adapters at
launch time — so adding a new data source to a deployment is a YAML row,
not a code change. See `docs/runbooks/profile_authoring.md` for how a
profile is put together.
