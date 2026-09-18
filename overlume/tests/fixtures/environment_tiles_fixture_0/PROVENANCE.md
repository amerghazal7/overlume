# environment_tiles_fixture_0 provenance

Synthesized entirely by `overlume/scripts/make_tile_fixture.py` -- nothing in
this directory is fetched from Cesium ion, OpenStreetMap, or any other
service. Every `.b3dm` byte is generated locally from the WGS84 ellipsoid
plus this file's own fixed-seed RNG (deterministic; regenerating reproduces
the same bytes).

## Regenerate

```
python3 overlume/scripts/make_tile_fixture.py [output_root]
```

`output_root` is optional (defaults to this repo, writing back to
`tests/fixtures/...` in place) -- pass a scratch directory to verify
determinism (diff two independent runs) without touching the committed
fixtures.

Seed: `20260917` (module constant `SEED`, `make_tile_fixture.py`).

## What's here

- `tileset.json`: a content-less root + 3 children, same structure/region
  numbers (just numbers, not licensed data) as the fixture this replaces --
  `tile_root.b3dm` (the leaf tile whose region contains the test anchor,
  same convention as before), `tile_a.b3dm` (a sibling), `tile_b.b3dm`
  (a second sibling).
- `tile_root.b3dm`: single glTF buffer, carries NORMAL, and carries a
  per-vertex `_BATCHID` accessor (float32 SCALAR, all zeros) -- the
  plain/default case (also the tile duplicated 16x into
  `environment_tiles_fixture_fallback_0/`). `_BATCHID` is here so
  `environment_stream.cpp`'s `strip_custom_vertex_attributes()` (the guard
  a real committed ion b3dm always needed, since gltfio's `createAsset()`
  rejects any non-core vertex attribute) has a real attribute to strip in
  this fixture too -- a regression there now blanks the golden, exactly as
  it would against the real tiles this replaces. tile_root is the tile
  this matters for: it dominates the golden frame (tile_a/tile_b sit
  ~3.8 km / ~6.6 km from the camera, i.e. sub-pixel).
- `tile_a.b3dm`: TWO glTF buffers (one embedded in the GLB, one a base64
  `data:` URI) -- exercises `consolidate_buffers()`'s multi-buffer merge
  path. Carries NORMAL.
- `tile_b.b3dm`: single buffer, deliberately carries NO NORMAL attribute --
  a real streamed-tile case for `ensure_flat_normals()` to actually patch.

Each tile's synthetic "buildings" are a handful of extruded box footprints
(fixed seed, derived from the tile's own region center) placed at their true
absolute ECEF position -- see `make_tile_fixture.py`'s own module comment
for the up-axis convention this depends on and why the b3dm feature table's
`RTC_CENTER` (present in every tile, for format realism) is inert to this
project's own placement math.

No token, no network, no third-party data anywhere in this directory.
