# Environment-bake runbook

How to bake OSM building footprints into the chunked, map-frame glTF format
the visualization node loads as background environment geometry, and how to
wire the output into the node. Grounded directly in
`overlume/scripts/bake_environment.py` (read in full for
this doc) and the node wiring in
`ros/src/micropilot_visualization_node/src/visualization_node.cpp`.
Every command below was actually run against the script's committed test
fixtures (network-free) while writing this doc — see "Verified" at the
bottom.

> Deployment/two-node-topology notes are out of scope for this guide — see
> [`README.md`](README.md) for the VM-095 deferral.

## What the script does (pipeline order)

1. Fetch building footprints for a bbox around an anchor point. **Overpass
   API is primary**; `--cache FILE` is a read-through cache (read if present,
   else fetch live and write) — this is what makes `--selfcheck` network-free.
2. Project every footprint vertex from WGS84 into the map frame using the
   *same* equirectangular math as the node's own `geo_anchor.cpp` `WgsToMap`
   — a second, independent Python port, pinned against a committed
   cross-language fixture (`tests/fixtures/geo_anchor_cpp_pin_0.json`).
3. Extrude each footprint to a height (`height` tag → `building:levels` tag ×
   3.0 m/level → 6.0 m default fallback), baking the heading rotation
   **per-geometry**, never scene-level (a documented trimesh export trap —
   see the script's own module docstring and `build_chunk_scene()`).
4. Bucket footprints into ~256 m cells and write one `.glb` per non-empty
   cell plus one `index.yaml`.
5. Render `verification_overlay.png` (footprints + the recorded ego track) —
   a human sanity-check image, not something a script can judge "looks
   right".

## The real CLI

```
bake_environment.py --anchor-lat LAT --anchor-lon LON [--anchor-heading-deg DEG]
                    --out DIR [--cache FILE] [--radius-m R] [--ego-track CSV]
bake_environment.py --anchor-file anchor.yaml --out DIR [--cache FILE] ...
bake_environment.py --selfcheck
```

| flag | meaning |
|---|---|
| `--anchor-lat` / `--anchor-lon` | anchor origin, decimal degrees |
| `--anchor-heading-deg` | bearing of map-frame +X from true north, degrees (default `0.0`) |
| `--anchor-file` | YAML side-file instead of the three flags above: `anchor_lat_deg` / `anchor_lon_deg` / `anchor_heading_deg` (heading optional, defaults `0.0`) |
| `--out` | output directory (required unless `--selfcheck`) |
| `--cache` | Overpass response cache, read-through |
| `--radius-m` | fetch bbox radius around the anchor (default `300.0` m) |
| `--ego-track` | optional CSV (`lat,lon,map_x,map_y`, Task 1's own fixture format) drawn onto the verification overlay |
| `--selfcheck` | runs the whole pipeline against committed fixtures, network-free, no other flags needed |

## Where the anchor numbers come from

**Never hand-guess `--anchor-lat`/`--anchor-lon`/`--anchor-heading-deg`.**
The node solves this anchor itself (VM-050, `GeoAnchorSolver` in
`geo_anchor.cpp`, sampling `map`→`base_link` TF plus `NavSatFix` fixes) and
**logs it in exactly this script's flag order** the moment it solves
(`visualization_node.cpp`'s `gps_sub_` callback):

```
geo-anchor solved: --anchor-lat %.8f --anchor-lon %.8f --anchor-heading-deg %.4f
```

Copy those three logged numbers onto this script's command line verbatim —
no unit conversion, no reordering (the script's own module docstring states
this explicitly, and the log line is deliberately formatted to make the copy
mechanical). If the node was configured with a `geo_datum_lat_deg` /
`geo_datum_lon_deg` / `geo_datum_heading_deg` override (all three, or the
override is rejected as a config error), the same log line fires
synchronously at `on_configure()` instead of waiting on live GPS+TF, tagged
`(geo_datum override)`.

## Fetch source: Overpass primary, Mapbox fallback (env var only)

Overpass (`overpass-api.de`) is always tried first. The Mapbox fallback is
attempted **only if** Overpass returns empty/errors **and** the environment
variable

```
MAPBOX_TOKEN
```

is set — read via `os.environ.get("MAPBOX_TOKEN")`, never logged, never
committed, never accepted as a CLI flag. If Overpass is empty and
`MAPBOX_TOKEN` is unset, the script prints one `WARN` to stderr and proceeds
with **zero footprints** for that bbox — not a hard failure (this is the
same "missing data renders nothing" rule the rest of the visual-mode stack
follows). The Mapbox path is a documented approximation (Tilequery sampling,
small synthetic square footprints, not true polygon outlines) — treat it as
a fallback of last resort, not a primary source.

Set the token for a single invocation without it lingering in your shell
history or environment:

```bash
MAPBOX_TOKEN=<your token> python3 bake_environment.py --anchor-lat ... --anchor-lon ... --out DIR
```

## Output contract

```
<out>/
  index.yaml          # {chunk_size_m: 256.0, chunks: [{id, path, center:[x,y,0], radius_m}, ...]}
  chunks/
    chunk_<i>_<j>.glb  # one non-empty 256 m cell's footprints, per-geometry-rotated into the map frame
  verification_overlay.png   # or .ppm if PIL isn't installed
```

Each `chunk_<i>_<j>.glb` carries `POSITION` **and** `NORMAL` per vertex
(`bake_environment.py`'s `export(..., include_normals=True)`). Chunks baked
before this change still carry `POSITION` only — the load-time path covers
that gap transparently: `ensure_flat_normals()` (`gltf_normals.cpp`) fills in
flat per-face normals for any primitive that arrives without a `NORMAL`
attribute, so an older bake keeps loading and rendering correctly, just
without the smoother per-vertex normals a fresh bake provides.

`index.yaml` never lists an empty cell (every footprint in it was
unrepairable) — absence of a chunk file means "nothing there", not "not
baked yet".

## Verify the bake: `verification_overlay.png`

This is a **human** check (the script's own docstring: "spec's own AC:
overlay image sanity-approved"), not something automated. Open it and
confirm the baked footprints (gray polygons) sit where you expect relative
to the recorded ego track (red line, when `--ego-track` was given) — a
bake with the wrong anchor typically shows the footprints and the ego track
in entirely different places, or rotated relative to each other.

## Wiring the output into the node (VM-052 / VM-063 / VM-096)

Five node parameters, all declared at `on_configure()`:

- `environment_enabled` (default `true`) — the disable knob. **Live-settable**
  via `on_params()` (`ros2 param set` / the vcam GUI's Environment Tiles
  switch): it toggles the renderer's visibility flag through
  `mpviz::set_environment_visible()`, which hides/shows an already-loaded
  source without tearing down its chunk index.
- `environment_chunks_dir` (default `""` — "not configured", per-checkout):
  the baked output directory (this doc's `--out`).
- `environment_source_uri` (default `""`): an optional `ion://` streaming
  source; empty means "use `environment_chunks_dir` as a baked dir" —
  byte-for-byte the pre-VM-063 behavior. **Live-settable** via `on_params()`
  (same param name), which re-arms a new source, gated on the geo-anchor
  already being solved (same precondition `on_activate()`'s own arming path
  enforces).
- `environment_tile_cache_dir` (default `""`): appended to a non-empty
  `environment_source_uri` as `?cache=<dir>` (skipped if the URI already
  carries a `cache=` key, e.g. an explicit `cache=off`).
- `environment_own_asset_uri` (default `""`): this deployment's own "clipped"
  ion asset id, read back by the vcam GUI/WS bridge to resolve its "clipped"
  preset — the node itself never dereferences it.

(`environment_attribution`, a sixth `environment_*` param controlling the
Google 3D Tiles attribution line, is unrelated to source selection — see
`cesium.md`.)

At `on_activate()` the node applies a three-way gate before calling
`mpviz::set_environment_source()`:

1. `environment_enabled` **and** `environment_chunks_dir` **and**
   `environment_source_uri` all empty → one `WARN` naming both params
   ("environment layer disabled this run"), the entry point is never called
   — this is the shipped default, not a failure. A streaming-only deployment
   (an `ion://` URI, no bake) does **not** hit this gate — it arms via
   `environment_source_uri` alone.
2. `environment_enabled` **and** (a chunks dir or a source URI is set)
   **and** the geo-anchor solver has already solved (or was overridden) →
   `set_environment_source()` is called with the composed source URI
   (`compose_environment_source_uri()`, `environment_source_uri.hpp`) and the
   solved/overridden anchor; a failure to open it is a `WARN`, not fatal
   ("no buildings this run").
3. `environment_enabled` but no geo-anchor solved yet at `on_activate()` time
   → one `WARN`, environment layer disabled for this run. Solving from live
   `NavSatFix`+TF takes real motion, so a run with no `geo_datum_*` override
   commonly reaches `on_activate()` before the anchor is solved — this is a
   stated, accepted gap (spec Sec.4.5/9), not silently patched around. This
   is the one precondition the live `environment_source_uri` switch in
   `on_params()` still enforces too — a switch attempted before the anchor
   solves is rejected, leaving the previous source (if any) intact.

No restart is required to change source or visibility once the node is
active: bake with the node's own logged anchor, point `environment_chunks_dir`
at the bake's `--out` directory (or set `environment_source_uri` to an
`ion://` streaming spec), and toggle `environment_enabled` or switch
`environment_source_uri` live via `ros2 param set` or the vcam GUI, subject
to the geo-anchor precondition above. See `docs/visual_mode/cesium.md` for
the streamed (`ion://`) source presets in detail.

## Verified

Ran directly against this repo's committed fixtures while writing this doc
(no network, no live rig):

```bash
cd overlume/scripts
python3 bake_environment.py --selfcheck
# selfcheck: PASS (7/7)

python3 bake_environment.py \
  --anchor-lat 25.0803 --anchor-lon 55.3910 --anchor-heading-deg 20.0 \
  --out <scratch-dir> \
  --cache ../tests/fixtures/environment_overpass_cache_0.json \
  --ego-track ../../ros/src/micropilot_visualization_node/test/fixtures/geo_anchor_samples_0.csv
# wrote 2 chunk(s), 5 footprint(s), index.yaml, verification_overlay.png

python3 bake_environment.py --anchor-file anchor.yaml --out <scratch-dir> \
  --cache ../tests/fixtures/environment_overpass_cache_0.json
# wrote 2 chunk(s), 5 footprint(s), index.yaml, verification_overlay.png
```

confirming: `--selfcheck` passes clean; both the `--anchor-lat/-lon/-heading-deg`
and `--anchor-file` anchor forms produce identical `index.yaml` + `chunks/*.glb`
+ `verification_overlay.png` output against the committed Overpass cache
fixture, with no network access in either run.
