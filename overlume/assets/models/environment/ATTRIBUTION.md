# Environment data attribution (Epic 4 Task 2 / VM-051)

Building footprint data baked by `scripts/bake_environment.py` is © OpenStreetMap
contributors, licensed under the Open Database License (ODbL) 1.0 —
https://opendatacommons.org/licenses/odbl/1.0/ — the same "credit the source"
convention `assets/models/ATTRIBUTION.md` already applies to the CC0 model
packs, applied here to a data source instead of a model pack.

- Source: Overpass API (`https://overpass-api.de/api/interpreter`), `way["building"]`
  query around the fixture anchor (lat 25.1972, lon 55.2744 — Burj Khalifa,
  Downtown Dubai, UAE).
- License: Open Database License (ODbL) v1.0 — https://www.openstreetmap.org/copyright
- Attribution required by ODbL: "© OpenStreetMap contributors."
- Fallback source (Decision 8, only if Overpass is empty/errors and `MAPBOX_TOKEN`
  is set): Mapbox — subject to Mapbox's own terms, not ODbL; the token itself is
  never committed, logged, or written into `index.yaml` (env var only).

## Test fixture provenance

`tests/fixtures/environment_overpass_cache_0.json` — 5 real building footprints
(`way["building"]`, 73 nodes), fetched LIVE from `overpass-api.de` on 2026-10-27
(OSM base timestamp `2026-10-27T22:04:12Z`) with the query `bake_environment.py`
builds for the default 300 m bbox around the fixture anchor (Downtown Dubai,
around the Burj Khalifa), then trimmed to five complete ways for a small
committed fixture (same convention as `tests/fixtures/hd_map_local_elements_0.geom`:
a short real slice, not a full extract). Each kept way has all its tags and
nodes exactly as returned: 1047612083 (Il Primo), 636375464 (the Address
Residences Dubai Opera podium), 1094649646, and 1330380402 / 1330380403 (two
ticket kiosks on the lake boardwalk). The tower itself and every way tagged
with `height`/`building:levels` were left out, so no footprint in this fixture
carries those tags — the height-fallback chain's three cases (Decision 9) are
exercised separately in `bake_environment.py`'s own `--selfcheck` using
synthetic in-memory footprints, not by mutating this real fixture with
invented tags. The fetch used `curl` with a descriptive `User-Agent`:
`overpass-api.de` answered the script's own `requests` POST with HTTP 406.

`tests/fixtures/environment_test_town_0/` is that cache baked at the fixture
anchor, heading 0, with the default radius and 256 m chunking (3 chunks). Its
`.glb` files carry POSITION only, as the original pre-normals bake did
(`test_gltf_normals.cpp` uses one as its normal-less input), so the bake
exports without normals. From `overlume/`:

```bash
python3 -c 'import sys, trimesh; sys.path.insert(0, "scripts"); import bake_environment as b
export = trimesh.Scene.export
trimesh.Scene.export = lambda self, *a, **kw: export(self, *a, **{**kw, "include_normals": False})
sys.exit(b.main(["--anchor-lat", "25.1972", "--anchor-lon", "55.2744", "--anchor-heading-deg", "0",
                 "--out", "tests/fixtures/environment_test_town_0",
                 "--cache", "tests/fixtures/environment_overpass_cache_0.json"]))'
```

`tests/fixtures/geo_anchor_cpp_pin_0.json` is not OSM data — see its own
`_provenance` field.
