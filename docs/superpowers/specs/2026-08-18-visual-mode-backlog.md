# Visual Mode — Backlog

Companion to `2026-08-18-visual-mode-design.md`. Ordered by phase; each story
lists acceptance criteria (AC). Execution: dynamic workflows (orch Fable /
impl Sonnet / review Opus) per project directive.

## Epic 0 — Contract spike (de-risk everything first)

- **VM-001 Filament headless hello-frame.** Build a minimal clang/libc++
  static `visual_renderer` lib (POD API) that renders a lit cube on a grid
  to a readable headless swapchain and returns RGB8.
  AC: standalone example writes a PNG on the dev box; no X server; header
  passes the POD-only check.
- **VM-002 Skeleton visualization node + mode mux.** Lifecycle node
  publishing VM-001's frame to `/rendering/image` at 30 Hz; global
  `/rendering/set_mode` handling in BOTH nodes (CUDA node idles on 3).
  AC: smoke test drives 1→3→2 with exactly-one-publisher and no consumer
  re-subscribe; existing node tests still green.
- **VM-003 vcam parity.** `~/set_virtual_cam` / `~/set_look` / `~/vcam_state`
  on the new node; WS bridge fan-out.
  AC: contract test proves identical framing math vs CUDA node for the 5
  presets; GUI orbit works against mode 3 unmodified.
- **VM-004 On-robot GPU budget measurement.** Run the spike beside the CUDA
  node + perception on robot hardware; record frame time & contention.
  AC: measured numbers recorded in the plan doc; go/no-go on 720p30 target.

## Epic 1 — Core scene & dark theme

- **VM-010 SceneGraph + double buffer.** Domain structs, staging/render swap.
  AC: unit tests for swap semantics and staleness fade timers.
- **VM-011 Theme system.** YAML tokens → materials; BOTH `dark_adas.yaml`
  and `light_clay.yaml` ship here (v1, per product direction 2026-08-18).
  AC: golden image of an empty themed world (ground, grid, sky/fog) per
  theme; no per-theme code branches.
- **VM-014 Animated theme toggle.** `set_theme` eases all tokens
  current→target (default 0.8 s smoothstep, Oklab palette lerp, sun/IBL
  crossfade); GUI day/night toggle + WS command.
  AC: golden at t=0/0.4/0.8 s of a transition (deterministic clock);
  no frame drop > 1 during switch; toggle mid-transition retargets smoothly.
- **VM-012 Ego robot.** M02P→glTF conversion script; TF-driven ego pose +
  speed; clay-box fallback.
  AC: golden image with ego; speed matches TF finite difference on a
  recorded fixture.
- **VM-013 Camera presets/tween port.** Eased tween identical to CUDA node.
  AC: contract test vectors pass.

## Epic 2 — Autonomy data ingestion

- **VM-020 Profile YAML loader** (urban/offroad topic rows → subscriptions).
  AC: bad rows rejected with clear errors; both shipped profiles load.
- **VM-021 DynamicObjectsAdapter + class inference.** MarkerArray →
  TrackedObjects; label keywords + bbox-dims fallback table (config-driven).
  AC: fixture tests from a recorded bag; inference table test; malformed
  markers dropped and counted.
- **VM-022 Clay object rendering.** CC0 glTF pack normalized; instancing;
  bbox-driven scaling; velocity arrows; predicted-path ribbons.
  AC: golden image with a mixed-class scene; 50 objects < 2 ms scene-update.
- **VM-023 Path ribbons.** Behavior (hero emissive) + global + local.
  AC: golden image; ribbon regenerates correctly on path change.
- **VM-024 HdMapAdapter + lane styling.**
  AC: golden from recorded HD-map fixture; cached (no per-frame rebuild).
- **VM-025 OGM ground layers** (dynamic + gradient, with `_updates`).
  AC: fixture test incl. partial updates; offroad profile golden.
- **VM-026 Collision alert polygons.**
  AC: golden with sweep + predicted polygons in warning materials.
- **VM-027 Generic marker fallback renderer.** All Marker primitive types,
  pooled renderables.
  AC: parity test rendering a synthetic MarkerArray of every type; any
  extra topic displayable via one profile row.

## Epic 3 — HUD, polish, controls

- **VM-030 SDF text + HUD overlay.** Speed chip, mode indicator.
  AC: golden; text legible at 720p low preset.
- **VM-031 Alert callouts.** Leader-line chips anchored to 3D objects
  (e.g. nearest-obstacle distance from collision topics).
  AC: golden; callout tracks object across camera moves.
- **VM-032 Layer visibility + quality presets** end to end (params + WS
  commands + GUI panel; theme toggle already shipped in VM-014).
  AC: WS E2E test toggles each layer and preset.
- **VM-033 (moved into VM-011/VM-014 — both themes + animated toggle ship
  in Epic 1.)**
- **VM-034 Staleness fades + diagnostics topic** surfaced in GUI.
  AC: silencing a topic fades its layer; diagnostics shows per-topic age.

## Epic 4 — Clay buildings (EnvironmentLayer, §4.5)

- **VM-050 Geo-anchor.** `geo_datum` param (lat/lon/heading of map origin)
  or NavSatFix+`gps_link` sampling → local-ENU transform; layer disabled
  with WARN when absent.
  AC: unit test round-trips WGS84↔map-frame within 0.1 m over a 2 km area.
- **VM-051 Bake pipeline.** `scripts/bake_environment.py`: bbox → OSM
  footprints+heights (Overpass or local extract; Mapbox MVT as alternative
  source) → extruded, chunked glTF + tileset index JSON; verification
  overlay image (footprints vs a recorded ego track).
  AC: bake of the CARLA-town / real operating area completes offline from a
  cached extract; overlay image sanity-approved.
- **VM-052 Runtime chunk loading + culling.** Index → distance-enabled
  chunks, theme building material.
  AC: golden with baked town; frame-time delta < 2 ms at target preset.
- **VM-053 → promoted to Epic 6 (v1.1).** See below.

## Epic 6 — v1.1: 3D Tiles streaming (committed, starts immediately after v1.0)

- **VM-060 Cesium ion registration + tileset access.** User performs the
  registration (offered 2026-08-18); obtain a Cesium OSM Buildings token,
  store like the Mapbox token (env var, never committed).
  AC: token retrieves tileset.json for the operating area.
- **VM-061 cesium-native build integration.** Pin a cesium-native release
  behind the same clang/libc++ + POD-boundary rules as Filament.
  AC: builds in CI alongside Filament; POD header check still passes.
- **VM-062 3D Tiles streaming EnvironmentSource.** cesium-native tile
  selection/loading behind the `EnvironmentSource` seam (VM-052); glTF tile
  payloads re-materialized with the theme's clay building material; geo
  placement via the VM-050 anchor; disk tile cache for offline robustness.
  AC: baked-source goldens still pass with the streaming source swapped in
  over the same area; frame-time budget held while streaming.
- **VM-063 Source selection + fallback.** Profile/param chooses
  `baked | streamed`; streamed falls back to baked chunks on network loss.
  AC: fallback e2e test (kill network mid-run → baked chunks appear, WARN).

## Epic 5 — Hardening & delivery

- **VM-040 Quality auto-drop with hysteresis.**
  AC: synthetic-load test triggers drop + log; recovers.
- **VM-041 Perf benchmark + CI wiring.** Golden/adapter/contract suites in
  CI (GPU tests skip cleanly without GPU), Filament build pinned + cached.
  AC: clean-checkout build documented and reproducible.
- **VM-042 Docs + runbook.** README section, profile-authoring guide for the
  autonomy team, environment-bake guide, deployment notes.
  AC: autonomy-team member can add a topic via profile YAML using only docs.
- **VM-043 Live validation.** Full stack on CARLA bridge + a real-robot bag;
  side-by-side review vs rviz for parity sign-off.
  AC: product + autonomy sign-off checklist complete.

## Future (explicitly deferred)

- Minimap inset (can reuse bake data);
  typed perception-topic adapter; hybrid mode (camera-imagery ground +
  synthetic overlays); interactive picking over WS; async readback; wheel/
  turn animations on clay models; `cuda/` directory rename.
