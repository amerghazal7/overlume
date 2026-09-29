# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

The Overlume open-source restructure (`docs/plans/2026-09-17-overlume-restructure.md`):

- Renamed the project to **Overlume**: `mpviz`/`MPVIZ_*` identifiers,
  namespaces, the ROS package (`micropilot_visualization_node` →
  `overlume_ros`, node `overlume_node`), env vars, and cache directories,
  all renamed with data contracts (topics, frames, services, fixture bags)
  left byte-identical.
- Restructured the repository layout: the library moved to `overlume/`, the
  ROS 2 workspace to `ros/` (`ros/src/overlume_ros/`), legacy CUDA/NumPy/GL
  prototype code and agent-config sprawl deleted, tracked binaries moved to
  Git LFS.
- Added `examples/`: six self-contained C++ programs against the public API
  headers only (`<overlume/scene.h>`, `<overlume/api.h>`), run headless as a
  gate stage.
- Added a generated API documentation target (`docs`, Doxygen +
  doxygen-awesome-css) over the public headers and examples, plus
  `include/overlume/version.h` with a configure-time drift check against
  `project(overlume VERSION ...)`.
- Restructured docs into one index (`docs/README.md`), one status ledger
  (`docs/status.md`), and `docs/runbooks/`/`docs/design/`/`docs/plans/`
  (with `docs/plans/archive/` for retired prototype documents).
- Added open-source project scaffolding: `LICENSE`, `NOTICE`,
  `CODE_OF_CONDUCT.md`, `SECURITY.md`, `CONTRIBUTING.md`, issue/PR templates,
  and hosted CI (`.github/workflows/`: lint, build, docs, release).
- One repo-wide `.clang-format` (clang-format 23.1.1, pinned) applied to
  every C++ source in a single commit listed in `.git-blame-ignore-revs`
  (`git config blame.ignoreRevsFile .git-blame-ignore-revs`), and an
  `SPDX-License-Identifier: Apache-2.0` header on every first-party source
  file, both enforced by `tools/check_format.sh` and `tools/check_spdx.sh`.
- Agent instructions consolidated into `AGENTS.md` (imported by `CLAUDE.md`);
  the repository knowledge-graph hook is no longer strict.

`overlume_ros` environment/buildings follow-ups (maintainer decision,
2026-09-18):

- `on_activate()` now arms the configured environment source
  (`environment_chunks_dir`/`environment_source_uri`) regardless of
  `environment_enabled` — a deployment launched with the disable knob off is
  now recoverable from the GUI Switch alone, no preset re-pick needed.
- Buildings are now gated per `render_mode`: `environment_effectively_visible()`
  (`scene_assembly.hpp`) shows them only in FREE_LOOK (VISUAL), hiding them
  unconditionally in BOWL/HYBRID — closes `docs/runbooks/signoff.md`
  exception 7 in full.
- Node library no longer exports cesium-native's vendored C-library symbols
  (`sqlite3_*`, `curl_*`, `SSL_*`, `EVP_*`, zlib), which interposed the system
  copies ROS loads into the same process; a linker version script hides just
  those, and `test_exported_symbols` guards both that and the C++ ABI runtime.
- `gps_topic` parameter, `replay` profile, `validate_visual_mode.sh
  --profile-dir` / `--param`, and a runbook for replaying real-robot logger
  sessions (including a network-loss check via a local kill-switch proxy).
- Examples load the shipped per-class models (`set_object_model_dir`);
  `OVERLUME_ENABLE_CESIUM` now defaults ON; `check_spdx.sh` scans untracked
  files; `check_format.sh` accepts explicit C/C++ paths.
- Measured the Google Photorealistic preset against OSM clay and no environment on a live replayed robot session: render_ms p50 9.5 / 10.4 / 9.9 ms, p99 22.7 / 24.9 / 23.3 ms — not materially heavier; `kMaxTileCreatesPerTick` remains unimplemented by evidence, not omission.
- Fixed streamed 3D Tiles sagging below the map plane at range (docs/status.md item 4): each vertex is now corrected to the true WGS84 ellipsoid height at load, on top of the existing rigid x/y transform, instead of the flat-tangent-plane approximation that sagged ~d²/(2R) (0.53 m at 2.6 km, 7.8 m at 10 km). Delivered accuracy is bounded by the pre-existing float32 quantization of the tile source geometry, not the sub-mm formula on doubles: a no-op within ~1.5 km of the anchor, ~0.1 m residual at the committed fixture's actual tile ranges (3.8–6.6 km) — see docs/status.md item 4 for the measured before/after on the real fixtures.

### Added

- `brightness=<gain>` on an `ion://` source URI and the node's `environment_brightness` param (default 1.0) scale streamed tiles' base colour. Google Photorealistic tiles declare `KHR_materials_unlit`, so they ignore the sun and read dim under the shipped dark theme's fog and palette; lowering `fog.density` in the theme is the other lever (`docs/runbooks/cesium.md`).
- Terrain following for the streamed environment (maintainer decision 2026-09-21, "option 2"): the `follow_terrain=on|off` key on an `ion://` source URI (an unrecognised value fails the whole parse) and the node's `environment_follow_terrain` param (default `true`) sample the tileset's own ground height under the ego (`Tileset::sampleHeightMostDetailed()`, re-sampled every ≥5 m or ≥2 s) and shift the whole streamed environment (one Filament terrain-root entity every tile is parented under) so the sampled ground meets the flat map plane at the robot — first sample snaps, later ones ease in over 0.5 s, clamped to ±30 m; the baked fallback is never shifted. Guarded by `EnvironmentStream.FollowTerrainShiftsEnvironmentToGroundUnderEgo` against a new synthesized fixture with a 3 m ground quad at the real-robot session's anchor (`make_tile_fixture.py --ground-height`). See `docs/runbooks/cesium.md` §5c.
- Terrain follower ground bias: `ground_bias=<m>` on the `ion://` URI / node param `environment_ground_bias_m` (default 0.3 m) parks the sampled ground below the map plane — at 0 Google's ground was coplanar with the HD-map road surface and ribbons and z-fought them (seen live 2026-09-21). `urban`/`replay` point-cloud `min_z_m` raised 0.2 → 0.35 m: the 0.2–0.4 m band of ground returns rendered as a dark blanket ahead of the robot.
- Point-cloud rows gain an optional `min_z_m` profile tunable: drops points whose map-frame z is below the configured road plane (NaN default = filter off, unchanged behaviour) so lidar ground returns stop z-fighting with the HD-map road surface and the other road-drawn elements; `urban`/`replay` ship `min_z_m: 0.2` (maintainer decision 2026-09-21). Dropped points are counted per adapter (`dropped_below_min_z`), not yet surfaced in the node's WARN line.

- Profile rows gain `frame_id:` (point_cloud / ogm: override a mislabelled header frame for the TF lookup) and `encoding: costmap` (ogm: decode Nav2 uint8 costs instead of treating them as malformed). New shipped profile `robot-offroad` for the real robot's topics. `validate_visual_mode.sh` gains `--static-tf` and `--shm`.

### Changed

- Streamed 3D Tiles selection now uses the real render camera as a second selection frustum next to the synthetic top-down coverage view (`StreamingEnvironmentSource::synthesize_view_and_pump()`). The synthetic view alone (256 px, 300 m up, 1.3 rad) stopped refining at ~85 m geometric error, so Google Photorealistic ground rendered as a coarse mesh metres above the detailed surface the terrain follower samples — which then lifted that coarse mesh through the road (2026-09-21 live finding). With the camera frustum (same `kStreamMaxSseErr` 48) tiles near the vehicle refine to ~1–2 m. One frustum while the camera is still at Filament's default pose, two once positioned; guarded by `EnvironmentStream.TileSelectionUsesRenderCameraAsSecondFrustum`. The terrain follower logs each sample (`terrain sample under ego: …`) through the redacting logger. `tools/validate_logger_session.sh --anchor-height-offset M` forwards `geo_anchor_height_offset_m` (this robot's receiver reads ~1.8 m above the map plane).
- `GeoAnchor` (`overlume/include/overlume/scene.h`) gains `origin_height_m` — `kSceneVersion` 6 → 7, additive per ADR-0004 (`docs/adr/0004-scene-interface-versioning.md`). The anchor's WGS84 ellipsoid height is now a real, sampled field instead of an implicit 0.0: `GeoAnchorSolver` (`ros/src/overlume_ros`) sets it to the mean of the accepted (non-NaN) `NavSatFix::altitude` samples, and two new node params, `geo_datum_height_m` (override) and `geo_anchor_height_offset_m` (trim), tune it further (docs/status.md item 4, docs/runbooks/cesium.md section 5b).
- `GroundGridLayer` (`overlume/include/overlume/scene.h`) gains `yaw_rad` — `kSceneVersion` 7 → 8, additive per ADR-0004. A grid published in a vehicle frame (e.g. `base_link` costmaps) now rotates with the vehicle instead of being drawn map-axis-aligned; the OGM adapter composes TF rotation with `info.origin.orientation`.
- Ground-grid free (0) cells are now transparent and the material premultiplies alpha; previously an all-free upper layer painted an opaque ground-coloured sheet over every obstacle of the layer beneath it. `ogm_offroad_light_clay` golden promoted (0.23% drift).

### Fixed

- OGM profile rows without `update_topic` were silently never subscribed (real costmaps publish no updates topic).
- The synthesized road-surface fill no longer blinks. It was rebuilt from scratch every tick and emitted only for lanes whose left and right rails both survived that tick, so lanes dropped in and out and the carriageway collapsed to a ribbon and back. Measured on the real-robot replay: the road's pixel count swung 89 percent between consecutive frames while the terrain held steady, and a captured normal/anomalous frame pair showed an identical camera, ego, lane markings and ribbon with only the fill changing. The upstream source was innocent, every boundary namespace being present in 60 of 60 sampled messages. Each lane's last complete rail pair is now cached and reused when a rail is missing for a tick, retired on the row's timeout and on a clock rewind so a looping replay cannot leave a ghost carriageway. The blink predates the streamed environment; the clay ground plane hid it because what showed through was the same dull colour.
- Streamed terrain now follows the road's grade instead of one global height. The follower sampled a single point under the robot and shifted the whole tileset by that scalar, so on any grade the true ground drifted from the map plane and sliced across the road tens of metres ahead. It now takes five heights along the robot's heading (0, +-10, +-20 m) in ONE batched `sampleHeightMostDetailed()` request on the same cadence, least-squares fits a line through them, and pitches the terrain about the across-track axis to lay that grade flat, pivoted where the samples were taken so the world cannot sway with ego jitter. Lateral samples are deliberately excluded: the verge is genuinely higher than the carriageway and would drag the fit up. The tilt is clamped (`max_tilt_deg`, default 2 deg, `0` restores offset-only) because a fitted tilt extrapolates forever, which bounds both far-field error and how far streamed buildings lean. Measured cost is under 0.1 ms of render time, against a 5 ms budget. With a 2 percent grade the terrain lands within 0.02 m of target at 20 m ahead and behind, where the single offset was 0.4 m out.
- The renderer's own clay ground plane (`build_ground_plane()`, a 120 m square opaque quad re-centred on the ego) covered every bit of streamed roadside detail within 60 m once the ground bias above stopped it z-fighting with Google's terrain — the quad simply won and hid the tiles, which defeats the point of 3D Tiles (verified live 2026-09-21: with the HD-map layer switched off the quad remained, ruling that layer out). The plane is now removed from the scene once the installed source PROVES it has ground under the ego — one successful `Tileset::sampleHeightMostDetailed()` hit, latched for the source's life so an intermittent miss cannot flicker it back, and dropped again on network-loss fallback so the baked path keeps its ground. Decided by evidence rather than preset name: Google Photorealistic hits and loses the plane, Cesium OSM Buildings is buildings-only, never hits, and keeps it. `EnvironmentSource::provides_ground()` (overridden by the streaming source only), reconciled once per frame in `render_frame()`; forced off with `replaces_ground=off` / `environment_replaces_ground: false`. Needs `follow_terrain` on, since the sample is the evidence. Guarded by `GroundPlaneHiddenWhenTilesetSuppliesGround`, `GroundPlaneKeptWhenTilesetIsBuildingsOnly` and `GroundPlaneReturnsAfterNetworkLossFallback`.

- `strip_attributes_and_correct_heights()` ignored glTF node transforms, recovering each vertex's ECEF position as `modelToEcef * p` — correct only when a mesh sits directly under the glTF root with an identity node transform. Real Google 3D Tiles glbs (ion asset 2275207) instead carry a per-node matrix (axis swap + a ~2.4e6 m translation) with node-LOCAL positions, so streamed Google tiles rendered as one giant tilted slab across the sky. Fixed by correcting each primitive against `modelToEcef * nodeTransform` (`CesiumGltf::Model::forEachPrimitiveInScene`) instead of `modelToEcef` alone, matching how gltfio itself places the mesh at render time. Guarded by `EnvironmentStream.NodeMatrixEncodingRendersIdenticallyToEcefEncoding` (docs/status.md item 4). The float32 quantization ceiling noted for the ellipsoid-height correction applies to absolute-ECEF-encoded content only; node-matrix tiles store node-local positions, where the correction survives the float32 store intact.
- Streamed Google 3D Tiles ground rendered ~1.7 m above the road on the real robot, verified live on the real-robot session replay (ion asset 2275207): the anchor's ellipsoid height was hard-coded 0.0 in `compute_ecef_to_map()` and `correct_ecef_point_height()` (only correct in simulation, where the anchor really sits on the ellipsoid — the real robot's Fixposition NavSatFix altitude at the anchor is ~1.7 m). Fixed by threading the new `GeoAnchor::origin_height_m` through both (docs/status.md item 4).

## [0.1.0] - 2026-09-17

The first tagged release (tag `v0.1.0` on the post-restructure tree,
2026-09-17; release created by `.github/workflows/release.yml`). Covers
everything delivered before the open-source restructure. One line per shipped item, sourced from
[`docs/status.md`](docs/status.md)'s own Shipped table (dates/hashes as
recorded there; see that table for the full provenance notes on the two
entries with a recorded historical-document discrepancy).

### Added

- **Epic 0 — Contract spike**: Filament hello-frame, mode mux, virtual-camera
  parity, GPU budget probe. Commits `cc900c1`…`5b479bd`, 2026-08-18 (GPU-budget
  item VM-043 closed 2026-09-11 on a dev-box proxy).
- **Epic 1 — Core scene & dark theme**: closed 2026-08-20 at `bd5e11e`.
- **Epic 2 — Autonomy data ingestion**: closed 2026-09-07 at `c5ea38c` (gate
  passed at `26f17f0`).
- **Epic 3 — HUD, polish & controls**: closed 2026-09-09 at `af628df`.
- **VM-077 — new-stack rendering / flicker root-cause fix** (Epic 3
  follow-on): closed 2026-09-10.
- **Epic 4 — Clay buildings (`EnvironmentLayer`), Tasks 1-3**: done
  2026-09-10.
- **Epic 5 — Hardening & delivery**: VM-040 (quality governor) and VM-044
  (asset packaging) done 2026-09-11; VM-041 (perf benchmark +
  `ci_visual_mode.sh`) and VM-042 (docs) delivered in parallel; VM-043 (live
  validation sign-off) superseded by the unified-engine migration's own
  parity sign-off.
- **Unified-engine migration** (VM-090…095): CUDA→Filament cutover, the mode
  mux and `micropilot_rendering_node` retired. Closed 2026-09-11, all 6
  tasks done.
- **Epic 6 — v1.1: Cesium 3D Tiles streaming** (VM-060…064) + post-close
  tail: closed 2026-09-16, final cross-cutting review 2026-09-17.

[Unreleased]: https://github.com/amerghazal7/overlume/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/amerghazal7/overlume/releases/tag/v0.1.0
