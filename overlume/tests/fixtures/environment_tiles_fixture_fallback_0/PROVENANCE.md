# environment_tiles_fixture_fallback_0 provenance

Synthesized, same generator as `environment_tiles_fixture_0/` (see that
directory's own PROVENANCE.md) -- `overlume/scripts/make_tile_fixture.py`,
same fixed seed, nothing fetched from any service.

Every `.b3dm` file here is a byte-identical copy of
`environment_tiles_fixture_0/tile_root.b3dm`, duplicated under 16 distinct
filenames so the tileset has 16 real, independently-requestable leaf tiles
sharing tile_root's own region instead of 3 -- see the network-loss e2e
(`test_environment_stream.cpp`,
`NetworkDeadFromFirstRequestFallsBackToBakedChunksOnce`) and this fixture's
predecessor's own PROVENANCE.md for why 16 real duplicate leaves (not 3) are
needed to deterministically cross `kNetworkLossConsecutiveFailures` inside a
single short-lived test process.

## Regenerate

```
python3 overlume/scripts/make_tile_fixture.py [output_root]
```

`output_root` is optional, same meaning as in `environment_tiles_fixture_0`'s
own PROVENANCE.md.

## What's here

- `tileset.json`: one content-less root (region/geometricError copied from
  `environment_tiles_fixture_0`'s own root) with 16 children, each
  `{boundingVolume, geometricError}` copied from that fixture's
  `tile_root.b3dm` child entry.
- `tile_root.b3dm` + `tile_extra_1.b3dm` .. `tile_extra_15.b3dm`: 16
  byte-identical copies of `environment_tiles_fixture_0/tile_root.b3dm`.

No token, no network, no third-party data anywhere in this directory.
