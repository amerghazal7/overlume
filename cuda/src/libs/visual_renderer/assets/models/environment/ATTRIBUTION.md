# Environment data attribution (Epic 4 Task 2 / VM-051)

Building footprint data baked by `scripts/bake_environment.py` is © OpenStreetMap
contributors, licensed under the Open Database License (ODbL) 1.0 —
https://opendatacommons.org/licenses/odbl/1.0/ — the same "credit the source"
convention `assets/models/ATTRIBUTION.md` already applies to the CC0 model
packs, applied here to a data source instead of a model pack.

- Source: Overpass API (`https://overpass-api.de/api/interpreter`), `way["building"]`
  query, operating area around the recorded anchor (lat 25.0803, lon 55.3910 —
  Dubai/Sharjah, UAE, Epic 4 plan Decision 6).
- License: Open Database License (ODbL) v1.0 — https://www.openstreetmap.org/copyright
- Attribution required by ODbL: "© OpenStreetMap contributors."
- Fallback source (Decision 8, only if Overpass is empty/errors and `MAPBOX_TOKEN`
  is set): Mapbox — subject to Mapbox's own terms, not ODbL; the token itself is
  never committed, logged, or written into `index.yaml` (env var only).

## Test fixture provenance

`tests/fixtures/environment_overpass_cache_0.json` — 5 real building footprints
(`way["building"]`, 28 nodes), fetched LIVE from `overpass-api.de` on 2026-09-10
for the bbox around the recorded operating area's anchor, trimmed to a small
handful of complete ways for a small committed fixture (same convention as
`tests/fixtures/hd_map_local_elements_0.geom`: a short real slice, not a full
extract). No footprint in this fixture carries `height`/`building:levels` tags
(true of the real data returned) — the height-fallback chain's three cases
(Decision 9) are exercised separately in `bake_environment.py`'s own
`--selfcheck` using synthetic in-memory footprints, not by mutating this real
fixture with invented tags.

`tests/fixtures/geo_anchor_cpp_pin_0.json` is not OSM data — see its own
`_provenance` field.
