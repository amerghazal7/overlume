# environment_tiles_fixture_session_ecef_0 provenance

Synthesized entirely by `overlume/scripts/make_tile_fixture.py` -- nothing in
this directory is fetched from Cesium ion, OpenStreetMap, or any other
service. Every `.b3dm` byte is generated locally from the WGS84 ellipsoid
plus this file's own fixed-seed RNG (deterministic; regenerating reproduces
the same bytes). No token, no network, no third-party data anywhere in this
directory.

## Regenerate

```
python3 overlume/scripts/make_tile_fixture.py --anchor-lat 25.08001258 --anchor-lon 55.38847719 --name environment_tiles_fixture_session_ecef_0 --no-fallback
```

Seed: `20260917` (module constant `SEED`, `make_tile_fixture.py`).
Encoding: `ecef` -- absolute-ECEF-scale authored positions under an identity node (this generator's usual convention).
Anchor: lat 25.08001258 deg, lon 55.38847719 deg -- every TILE_REGION
(and the tileset root region) shifted by the same lat/lon delta from the
generator's own implicit default anchor (25.0803 N / 55.391 E,
`DEFAULT_ANCHOR_LAT_DEG`/`DEFAULT_ANCHOR_LON_DEG` in `make_tile_fixture.py`),
same anchor `test_environment_stream.cpp` installs as `GeoAnchor` when
loading this fixture.

## What's here

- `tileset.json`: a content-less root + 3 children -- `tile_root.b3dm` (the
  leaf tile whose region contains the anchor), `tile_a.b3dm` (a sibling),
  `tile_b.b3dm` (a second sibling). Region numbers are
  `environment_tiles_fixture_0`'s own region numbers, shifted by the anchor
  delta above (so every tile keeps the same footprint relative to the new
  anchor). No fallback dir generated for this fixture (`--no-fallback`).
