#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal
"""Generates the synthesized b3dm/tileset.json test fixtures that replaced
the committed-from-Cesium-ion ones (VM-097: purge real ion/OSM tile bytes
from history by path).

Produces THREE tiles -- tile_root.b3dm, tile_a.b3dm, tile_b.b3dm -- each a
handful of deterministic (fixed seed), hand-authored extruded-box "buildings"
wrapped in a real b3dm envelope (28-byte header + a feature table carrying
BATCH_LENGTH and RTC_CENTER) around a plain glTF 2.0 GLB, PLUS the two
tileset.json manifests (the 3-tile root fixture and the 16-tile
same-region fallback fixture used by the network-loss e2e). Nothing here is
fetched from any service -- every byte is generated locally from the
WGS84 ellipsoid + this file's own RNG.

Regenerate with (deterministic output; optional output_root, see below):
    python3 overlume/scripts/make_tile_fixture.py [output_root]

Design notes (see the task's own PROVENANCE.md files for the "why", this is
the "how"):

  - Region bounding volumes / geometricError in both tileset.json files are
    copied VERBATIM from the pre-existing (real-tile) fixtures' own
    tileset.json -- they are just numbers (a lat/lon/height box + an error
    metric), not licensed OSM data, and reusing them keeps every existing
    anchor-containment/placement assumption in the test suite valid with no
    re-derivation needed.

  - Vertex positions are baked as REAL, absolute ECEF-scale numbers (not
    RTC-relative small numbers): environment_stream.cpp's own
    StreamRendererResources::prepareInLoadThread() never reads the b3dm
    feature table's RTC_CENTER back out (verified against cesium-native
    0.64.0: CesiumGltfContent::GltfUtilities::applyRtcCenter() -- the only
    thing that ever folds RTC_CENTER into a placement transform -- is called
    solely from the raster-overlay and bounding-volume code paths, never
    from the render-content path our IPrepareRendererResources
    implementation sits on). RTC_CENTER is still emitted (computed from the
    tile region's own center, WGS84) so the feature table has the shape a
    real b3dm carries and cesium-native's B3dmToGltfConverter still tags the
    parsed model with the CESIUM_RTC glTF extension (exercising that parse
    path) -- it is just inert for OUR placement math, by design, matching
    how the real committed tiles this replaces already worked (no separate
    RTC_CENTER node, per environment_stream.cpp's own comment).

  - Each authored vertex is pre-rotated by the INVERSE of
    CesiumGeometry::Transforms::Y_UP_TO_Z_UP (glTF's default up-axis
    assumption, which every b3dm/gltf tile gets whether it wants it or not)
    so that after cesium-native applies that fixed rotation, the geometry
    lands at its true ECEF position: Y_UP_TO_Z_UP maps (x,y,z) -> (x,-z,y),
    so an authored vertex of (X_ecef, Z_ecef, -Y_ecef) round-trips to
    (X_ecef, Y_ecef, Z_ecef).

  - tile_root: single glTF buffer, HAS NORMAL, and carries a per-vertex
    `_BATCHID` accessor (float32 SCALAR, all zeros) -- the plain/default
    case (also the one duplicated 16x into the fallback fixture). The
    `_BATCHID` attribute is what environment_stream.cpp's own
    strip_attributes_and_correct_heights() strips before gltfio's createAsset()
    ever sees the primitive (a real committed ion b3dm always carried one;
    a regression in that strip must blank the golden exactly as it would
    against the real tiles this replaces -- gate round 1 finding 1). Only
    tile_root carries it: tile_root's geometry dominates the golden frame
    (tile_a sits ~3.8 km and tile_b ~6.6 km from the camera, i.e.
    sub-pixel), so this is the one tile whose attributes are load-bearing
    for that image.
  - tile_a: TWO glTF buffers (buffer 0 embedded in the GLB's BIN chunk,
    buffer 1 a base64 data: URI holding the index data) so
    consolidate_buffers()'s multi-buffer merge path stays exercised; HAS
    NORMAL (kept the "already has normals, passes through byte-for-byte"
    case test_gltf_normals.cpp already pins).
  - tile_b: single buffer, NO NORMAL attribute -- a real streamed-tile case
    for ensure_flat_normals() to actually patch, not just the synthetic
    baked-chunk case.

Pass an output root as argv[1] to write the fixtures somewhere other than
this repo's own tests/fixtures/ (e.g. a scratch tree, to verify
determinism by diffing two independent runs):
    python3 overlume/scripts/make_tile_fixture.py [output_root]

CLI options (all optional; the no-argument invocation above is unchanged and
must keep reproducing environment_tiles_fixture_0 / _fallback_0
byte-identically):
    --encoding {ecef,node-matrix}  default ecef. node-matrix authors
        node-LOCAL positions under a real glTF node {mesh, matrix} (Google's
        own 3D Tiles convention) instead of this generator's usual
        absolute-ECEF-scale positions under an identity node -- see
        build_tile_glb()'s own comment for the exact math.
    --anchor-lat / --anchor-lon    default the implicit anchor every
        TILE_REGION below is already built around (DEFAULT_ANCHOR_LAT_DEG /
        DEFAULT_ANCHOR_LON_DEG). Shifting either translates every
        TILE_REGION (and the tileset root region) by the same lat/lon delta.
    --name          fixture directory base name (default
        environment_tiles_fixture_0). The fallback dir's own name is derived
        from this (see fallback_dir_name()).
    --no-fallback   skip the 16-tile fallback fixture dir entirely.
"""
import argparse
import base64
import json
import math
import os
import random
import struct
import sys

SEED = 20260917

# ---- WGS84 ellipsoid -------------------------------------------------------
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def geodetic_to_ecef(lat_rad, lon_rad, h_m):
    sin_lat, cos_lat = math.sin(lat_rad), math.cos(lat_rad)
    sin_lon, cos_lon = math.sin(lon_rad), math.cos(lon_rad)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (n + h_m) * cos_lat * cos_lon
    y = (n + h_m) * cos_lat * sin_lon
    z = (n * (1.0 - WGS84_E2) + h_m) * sin_lat
    return (x, y, z)


def enu_basis(lat_rad, lon_rad):
    """East/north/up unit vectors, in ECEF, at (lat_rad, lon_rad)."""
    sin_lat, cos_lat = math.sin(lat_rad), math.cos(lat_rad)
    sin_lon, cos_lon = math.sin(lon_rad), math.cos(lon_rad)
    east = (-sin_lon, cos_lon, 0.0)
    north = (-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat)
    up = (cos_lat * cos_lon, cos_lat * sin_lon, sin_lat)
    return east, north, up


def add(*vs):
    return tuple(sum(c) for c in zip(*vs))


def scale(v, s):
    return tuple(c * s for c in v)


def ecef_to_authored(ecef):
    """Inverse of CesiumGeometry::Transforms::Y_UP_TO_Z_UP, see module doc."""
    x, y, z = ecef
    return (x, z, -y)


def apply_node_matrix_rotation_transpose(v):
    """R^T for the node-matrix encoding's R = [[1,0,0],[0,0,1],[0,-1,0]] (the
    3x3 upper-left of the {"matrix":[1,0,0,0, 0,0,-1,0, 0,1,0,0, tx,ty,tz,1]}
    node build_tile_glb() emits for --encoding node-matrix, column-major per
    the glTF spec). R is a pure axis-swap (orthogonal, no scale), so
    R^T == R^-1 -- this is what turns an authored (world-frame, identity-node
    convention) vector into the node-LOCAL vector a real node matrix expects,
    same direction ecef_to_authored() above already goes in for the
    Y_UP_TO_Z_UP fixed rotation."""
    x, y, z = v
    return (x, -z, y)


# ---- Synthetic building geometry (local ENU offsets from a tile's own
#      region center) -------------------------------------------------------
def make_box(cx, cy, sx, sy, h):
    """One extruded box footprint -> (verts_enu, normals_enu, tris), each
    vert/normal in local (east, north, up) offsets from the tile origin.
    Unwelded (one vertex per face corner, no shared vertices across faces --
    same convention gltf_normals.cpp's own comment documents for every other
    producer in this codebase) so a flat per-face normal is also the correct
    per-vertex normal, and so ensure_flat_normals()'s AREA-WEIGHTED
    accumulation (which sums over every face touching a vertex) degenerates
    to the same single-face value on tile_b's NORMAL-less case. Bottom face
    omitted -- never visible from outside a building sitting on the ground.
    """
    hx, hy = sx / 2.0, sy / 2.0
    p = {
        "000": (cx - hx, cy - hy, 0.0),
        "100": (cx + hx, cy - hy, 0.0),
        "110": (cx + hx, cy + hy, 0.0),
        "010": (cx - hx, cy + hy, 0.0),
        "001": (cx - hx, cy - hy, h),
        "101": (cx + hx, cy - hy, h),
        "111": (cx + hx, cy + hy, h),
        "011": (cx - hx, cy + hy, h),
    }
    # (face vertices in outward-CCW order, face normal)
    faces = [
        (["001", "101", "111", "011"], (0.0, 0.0, 1.0)),   # top
        (["100", "110", "111", "101"], (1.0, 0.0, 0.0)),   # +east
        (["010", "000", "001", "011"], (-1.0, 0.0, 0.0)),  # -east
        (["010", "011", "111", "110"], (0.0, 1.0, 0.0)),   # +north
        (["000", "100", "101", "001"], (0.0, -1.0, 0.0)),  # -north
    ]
    verts, normals, tris = [], [], []
    for corners, n in faces:
        base = len(verts)
        for c in corners:
            verts.append(p[c])
            normals.append(n)
        tris += [(base, base + 1, base + 2), (base, base + 2, base + 3)]
    return verts, normals, tris


def generate_ground_quad_geometry(half_extent_m=300.0):
    """One flat quad (two triangles, a single flat 'up' normal), +-
    half_extent_m around the tile's own ENU origin -- the --ground-height
    fixture's single tile geometry (no RNG, deterministic; unlike
    generate_tile_geometry() above this is not a scatter of buildings)."""
    p = {
        "00": (-half_extent_m, -half_extent_m, 0.0),
        "10": (half_extent_m, -half_extent_m, 0.0),
        "11": (half_extent_m, half_extent_m, 0.0),
        "01": (-half_extent_m, half_extent_m, 0.0),
    }
    corners = ["00", "10", "11", "01"]
    n = (0.0, 0.0, 1.0)
    verts = [p[c] for c in corners]
    normals = [n] * len(corners)
    tris = [(0, 1, 2), (0, 2, 3)]
    return verts, normals, tris


def ground_quad_region(anchor_lat_rad, anchor_lon_rad, height_m, half_extent_m=300.0):
    """Bounding region (same [west, south, east, north, min_h, max_h]-in-
    radians shape as TILE_REGIONS/ROOT_REGION above) covering a
    +-half_extent_m quad centered on (anchor_lat_rad, anchor_lon_rad) at
    ellipsoid height height_m. Flat-earth deg/radian approximation
    (half_extent_m / WGS84_A) -- good enough for a bounding VOLUME on a
    300 m fixture quad, same precision every other region in this file
    already assumes."""
    dlat = half_extent_m / WGS84_A
    dlon = half_extent_m / (WGS84_A * max(math.cos(anchor_lat_rad), 1e-6))
    return [anchor_lon_rad - dlon, anchor_lat_rad - dlat, anchor_lon_rad + dlon,
            anchor_lat_rad + dlat, height_m - 1.0, height_m + 1.0]


def generate_tile_geometry(seed, n_buildings=5):
    """Buildings scattered in a plausible city-block footprint (+-40m),
    5-16m footprints, 6-28m tall -- returns (verts_enu, normals_enu, tris)."""
    rng = random.Random(seed)
    all_verts, all_normals, all_tris = [], [], []
    for _ in range(n_buildings):
        cx = rng.uniform(-40.0, 40.0)
        cy = rng.uniform(-40.0, 40.0)
        sx = rng.uniform(5.0, 16.0)
        sy = rng.uniform(5.0, 16.0)
        h = rng.uniform(6.0, 28.0)
        verts, normals, tris = make_box(cx, cy, sx, sy, h)
        offset = len(all_verts)
        all_verts += verts
        all_normals += normals
        all_tris += [(a + offset, b + offset, c + offset) for a, b, c in tris]
    return all_verts, all_normals, all_tris


# ---- glTF/GLB authoring ----------------------------------------------------
def pad4(n):
    return (4 - (n % 4)) % 4


def f32_bytes(vals):
    return struct.pack("<%df" % len(vals), *vals)


def u16_bytes(vals):
    return struct.pack("<%dH" % len(vals), *vals)


def build_glb(gltf_json, bin_chunk):
    json_text = json.dumps(gltf_json, separators=(",", ":"))
    json_bytes = json_text.encode("utf-8") + b" " * pad4(len(json_text))
    bin_bytes = bytes(bin_chunk) + b"\x00" * pad4(len(bin_chunk))
    total = 12 + 8 + len(json_bytes) + (8 + len(bin_bytes) if bin_bytes else 0)
    out = struct.pack("<III", 0x46546C67, 2, total)  # "glTF", version 2
    out += struct.pack("<II", len(json_bytes), 0x4E4F534A) + json_bytes  # "JSON"
    if bin_bytes:
        out += struct.pack("<II", len(bin_bytes), 0x004E4942) + bin_bytes  # "BIN\0"
    return out


def build_tile_glb(ecef_center, include_normal, multi_buffer, seed, include_batchid=False,
                    encoding="ecef", geometry=None):
    """One tile's GLB: a single mesh primitive combining every synthetic
    building's geometry, positions authored per the module doc's
    Y-UP_TO_Z_UP inverse so cesium-native's fixed up-axis conversion lands
    them at their true ECEF position.

    `encoding="ecef"` (default): identity node ({"mesh": 0}), POSITION is the
    authored value directly -- this generator's original convention.

    `encoding="node-matrix"`: mimics a real Google 3D Tiles glb -- the node
    is {"mesh":0, "matrix":[1,0,0,0, 0,0,-1,0, 0,1,0,0, tx,ty,tz,1]}
    (column-major, R = the fixed axis-swap 3x3, t = the tile center in the
    SAME authored Y-up frame the ecef encoding authors every vertex in), and
    each POSITION is node-LOCAL: R^T @ (p_authored - t) (R orthogonal, so
    R^T == R^-1 -- applying the node matrix back at render time exactly
    reconstructs p_authored, so a correct consumer renders this identically
    to the ecef encoding). Normals are rotated by R^T too (no translation).
    """
    verts_enu, normals_enu, tris = geometry if geometry is not None else generate_tile_geometry(seed=seed)
    lat = math.asin(ecef_center[2] / math.sqrt(sum(c * c for c in ecef_center)))
    lon = math.atan2(ecef_center[1], ecef_center[0])
    east, north, up = enu_basis(lat, lon)
    t_authored = ecef_to_authored(ecef_center)

    positions = []
    normals = []
    for (ex, ny, uz), (nex, nny, nuz) in zip(verts_enu, normals_enu):
        vertex_ecef = add(ecef_center, scale(east, ex), scale(north, ny), scale(up, uz))
        p_authored = ecef_to_authored(vertex_ecef)
        normal_ecef = add(scale(east, nex), scale(north, nny), scale(up, nuz))
        normal_authored = ecef_to_authored(normal_ecef)
        if encoding == "node-matrix":
            d = tuple(a - b for a, b in zip(p_authored, t_authored))
            positions.append(apply_node_matrix_rotation_transpose(d))
            normals.append(apply_node_matrix_rotation_transpose(normal_authored))
        else:
            positions.append(p_authored)
            normals.append(normal_authored)
    indices = [i for tri in tris for i in tri]

    pos_flat = [c for v in positions for c in v]
    pos_min = [min(pos_flat[i::3]) for i in range(3)]
    pos_max = [max(pos_flat[i::3]) for i in range(3)]

    pos_bytes = f32_bytes(pos_flat)
    idx_bytes = u16_bytes(indices)

    buffers = []
    buffer_views = []
    accessors = []

    # POSITION (+ NORMAL, if requested) always land in buffer 0 -- the GLB's
    # own embedded BIN chunk (buffer 0 never carries a "uri", per the glTF
    # 2.0 spec's own GLB convention).
    buf0 = bytearray(pos_bytes)
    buffer_views.append({"buffer": 0, "byteOffset": 0, "byteLength": len(pos_bytes)})
    accessors.append({
        "bufferView": 0, "componentType": 5126, "count": len(positions),
        "type": "VEC3", "min": pos_min, "max": pos_max,
    })
    position_accessor = 0
    next_accessor = 1
    normal_accessor = None
    if include_normal:
        norm_flat = [c for v in normals for c in v]
        norm_bytes = f32_bytes(norm_flat)
        buffer_views.append({
            "buffer": 0, "byteOffset": len(buf0), "byteLength": len(norm_bytes),
        })
        buf0 += norm_bytes
        accessors.append({
            "bufferView": len(buffer_views) - 1, "componentType": 5126,
            "count": len(normals), "type": "VEC3",
        })
        normal_accessor = next_accessor
        next_accessor += 1

    batchid_accessor = None
    if include_batchid:
        # A per-vertex `_BATCHID` (float32 SCALAR, all zeros -- one real
        # value would do, every real b3dm batch table assigns SOME id) so
        # strip_attributes_and_correct_heights() has a real attribute to strip
        # (gate round 1 finding 1). Appended to buffer 0 alongside
        # POSITION/NORMAL, same as a real b3dm's own layout.
        batchid_bytes = f32_bytes([0.0] * len(positions))
        buffer_views.append({
            "buffer": 0, "byteOffset": len(buf0), "byteLength": len(batchid_bytes),
        })
        buf0 += batchid_bytes
        accessors.append({
            "bufferView": len(buffer_views) - 1, "componentType": 5126,
            "count": len(positions), "type": "SCALAR",
        })
        batchid_accessor = next_accessor
        next_accessor += 1

    if multi_buffer:
        # A SECOND glTF buffer (Decision 7/15.4's consolidate_buffers() path
        # needs a real multi-buffer tile to exercise) -- a self-contained
        # base64 data: URI, so this fixture stays a single committed file
        # with no companion .bin. CesiumGltfReader resolves data: URIs
        # synchronously (no network) when it parses the GLB's JSON chunk.
        buffers.append({"byteLength": len(buf0)})  # buffer 0: embedded, no uri
        idx_pad = idx_bytes + b"\x00" * pad4(len(idx_bytes))
        buffers.append({
            "byteLength": len(idx_bytes),
            "uri": "data:application/octet-stream;base64," +
                   base64.b64encode(idx_pad).decode("ascii"),
        })
        buffer_views.append({"buffer": 1, "byteOffset": 0, "byteLength": len(idx_bytes)})
    else:
        buf0 += idx_bytes
        buffer_views.append({
            "buffer": 0, "byteOffset": len(buf0) - len(idx_bytes),
            "byteLength": len(idx_bytes),
        })
        buffers.append({"byteLength": len(buf0)})
    index_accessor = len(accessors)
    accessors.append({
        "bufferView": len(buffer_views) - 1, "componentType": 5123,
        "count": len(indices), "type": "SCALAR",
    })

    attributes = {"POSITION": position_accessor}
    if normal_accessor is not None:
        attributes["NORMAL"] = normal_accessor
    if batchid_accessor is not None:
        attributes["_BATCHID"] = batchid_accessor

    if encoding == "node-matrix":
        node = {
            "mesh": 0,
            "matrix": [1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0,
                       t_authored[0], t_authored[1], t_authored[2], 1],
        }
    else:
        node = {"mesh": 0}

    gltf = {
        "asset": {"version": "2.0", "generator": "overlume make_tile_fixture.py"},
        "scene": 0,
        "scenes": [{"nodes": [0]}],
        "nodes": [node],
        "meshes": [{"primitives": [{
            "attributes": attributes, "indices": index_accessor, "material": 0, "mode": 4,
        }]}],
        "materials": [{
            "pbrMetallicRoughness": {
                "baseColorFactor": [0.55, 0.52, 0.48, 1.0],
                "metallicFactor": 0.0, "roughnessFactor": 0.9,
            },
            "doubleSided": True,
        }],
        "buffers": buffers,
        "bufferViews": buffer_views,
        "accessors": accessors,
    }
    return build_glb(gltf, buf0)


def region_center_ecef(region):
    west, south, east, north, min_h, max_h = region
    lat = (south + north) / 2.0
    lon = (west + east) / 2.0
    h = (min_h + max_h) / 2.0
    return geodetic_to_ecef(lat, lon, h)


def build_b3dm(glb_bytes, rtc_center):
    feature_table = json.dumps({"BATCH_LENGTH": 1, "RTC_CENTER": list(rtc_center)},
                               separators=(",", ":"))
    header_len = 28
    ft_json = feature_table.encode("utf-8")
    ft_json += b" " * ((8 - (header_len + len(ft_json)) % 8) % 8)
    total_len = header_len + len(ft_json) + len(glb_bytes)
    header = struct.pack("<4sIIIIII", b"b3dm", 1, total_len, len(ft_json), 0, 0, 0)
    return header + ft_json + glb_bytes


# ---- Tileset regions (copied verbatim from the pre-existing real-tile
#      fixtures -- coordinates only, not licensed OSM content) -------------
ROOT_REGION = [0.9664051710363271, 0.43720384316056576, 0.9677587977896256,
               0.4387199792658467, -24.79683634514544, 39.20705412661656]
ROOT_GEOMETRIC_ERROR = 100000.0
CHILD_GEOMETRIC_ERROR = 18.81526850096646
TILE_REGIONS = {
    "tile_root": [0.9664115484694138, 0.43755554620346443, 0.9669359326431756,
                  0.43793388646140696, -11.596608468970631, 6.109402182930353],
    "tile_a": [0.9664073422259165, 0.4379571900975796, 0.9671745086799476,
               0.43871955166017995, -23.948978754749383, 32.93258501058904],
    "tile_b": [0.9671738681441122, 0.4384497377204557, 0.967733008804598,
               0.4386812399377695, -8.958795091376492, 39.20705412661656],
}

# The implicit anchor TILE_REGIONS/ROOT_REGION above are already built
# around -- test_environment_stream.cpp's own kFixtureAnchor. --anchor-lat/
# --anchor-lon shift every region by the same lat/lon delta FROM this pair,
# in radians (region tuples are already radians), leaving each tile's
# footprint identical relative to the new anchor.
DEFAULT_ANCHOR_LAT_DEG = 25.0803
DEFAULT_ANCHOR_LON_DEG = 55.3910


def shift_region(region, dlat_rad, dlon_rad):
    """region = [west, south, east, north, min_h, max_h] in radians (west/
    east are longitude, south/north latitude, per geodetic_to_ecef()'s own
    convention) -- translate the whole box by a fixed lat/lon delta."""
    west, south, east, north, min_h, max_h = region
    return [west + dlon_rad, south + dlat_rad, east + dlon_rad, north + dlat_rad, min_h, max_h]


def fallback_dir_name(name):
    """environment_tiles_fixture_0 -> environment_tiles_fixture_fallback_0
    (insert "fallback_" before a trailing numeric suffix, same shape as the
    committed pair); any other name gets a plain "_fallback" suffix."""
    base, sep, suffix = name.rpartition("_")
    if sep and suffix.isdigit():
        return base + "_fallback_" + suffix
    return name + "_fallback"


def write_tileset(path, root_region, children):
    tileset = {
        "asset": {"version": "1.0"},
        "geometricError": ROOT_GEOMETRIC_ERROR,
        "root": {
            "boundingVolume": {"region": root_region},
            "geometricError": ROOT_GEOMETRIC_ERROR,
            "refine": "ADD",
            "children": children,
        },
    }
    with open(path, "w") as f:
        json.dump(tileset, f, indent=2)
        f.write("\n")


def child_entry(region, uri):
    return {
        "boundingVolume": {"region": region},
        "geometricError": CHILD_GEOMETRIC_ERROR,
        "content": {"uri": uri},
    }


PROVENANCE_MAIN = """# environment_tiles_fixture_0 provenance

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

Seed: `{seed}` (module constant `SEED`, `make_tile_fixture.py`).

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
  `environment_stream.cpp`'s `strip_attributes_and_correct_heights()` (the guard
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
"""

PROVENANCE_FALLBACK = """# environment_tiles_fixture_fallback_0 provenance

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
"""

PROVENANCE_GENERIC = """# {name} provenance

Synthesized entirely by `overlume/scripts/make_tile_fixture.py` -- nothing in
this directory is fetched from Cesium ion, OpenStreetMap, or any other
service. Every `.b3dm` byte is generated locally from the WGS84 ellipsoid
plus this file's own fixed-seed RNG (deterministic; regenerating reproduces
the same bytes). No token, no network, no third-party data anywhere in this
directory.

## Regenerate

```
{cmdline}
```

Seed: `{seed}` (module constant `SEED`, `make_tile_fixture.py`).
Encoding: `{encoding}`{encoding_note}
Anchor: lat {anchor_lat} deg, lon {anchor_lon} deg -- every TILE_REGION
(and the tileset root region) shifted by the same lat/lon delta from the
generator's own implicit default anchor ({default_lat} N / {default_lon} E,
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
{ground_height_line}"""


def parse_args(argv):
    p = argparse.ArgumentParser(
        description="Generates the synthesized b3dm/tileset.json test fixtures -- see this "
                    "module's own docstring.")
    p.add_argument("output_root", nargs="?", default=None,
                   help="write here instead of this repo's own tests/fixtures/ (e.g. a scratch "
                        "dir, to verify determinism)")
    p.add_argument("--encoding", choices=["ecef", "node-matrix"], default="ecef",
                   help="ecef (default, this generator's original convention): absolute-ECEF-"
                        "scale authored positions under an identity node. node-matrix: Google-"
                        "3D-Tiles-style node-LOCAL positions under a {mesh, matrix} node.")
    p.add_argument("--anchor-lat", type=float, default=DEFAULT_ANCHOR_LAT_DEG,
                   help="degrees (default %s); shifts every TILE_REGION by the same lat delta "
                        "from the implicit default anchor" % DEFAULT_ANCHOR_LAT_DEG)
    p.add_argument("--anchor-lon", type=float, default=DEFAULT_ANCHOR_LON_DEG,
                   help="degrees (default %s); same, longitude" % DEFAULT_ANCHOR_LON_DEG)
    p.add_argument("--name", default="environment_tiles_fixture_0",
                   help="fixture directory base name under tests/fixtures/")
    p.add_argument("--no-fallback", action="store_true",
                   help="skip the 16-tile fallback fixture dir")
    p.add_argument("--ground-height", type=float, default=None,
                   help="metres, ellipsoid height (default: none -- no ground tile emitted). "
                        "When given, emits an extra tile_ground.b3dm: one flat quad (two "
                        "triangles, a flat normal) spanning +-300m around the anchor at this "
                        "height, with its own tileset.json region entry.")
    args = p.parse_args(argv)
    _validate_args(p, args)
    return args


def _cmdline_for_provenance(args):
    """Reconstructs the argv that produced this run's own fixture, for that
    fixture's committed PROVENANCE.md -- only the non-default flags, same
    spirit as argparse's own defaulting (nothing here changes the actual
    generation, it's just the "how to regenerate this" text)."""
    parts = ["python3 overlume/scripts/make_tile_fixture.py"]
    if args.encoding != "ecef":
        parts.append("--encoding %s" % args.encoding)
    if args.anchor_lat != DEFAULT_ANCHOR_LAT_DEG:
        parts.append("--anchor-lat %s" % args.anchor_lat)
    if args.anchor_lon != DEFAULT_ANCHOR_LON_DEG:
        parts.append("--anchor-lon %s" % args.anchor_lon)
    if args.name != "environment_tiles_fixture_0":
        parts.append("--name %s" % args.name)
    if args.no_fallback:
        parts.append("--no-fallback")
    if args.ground_height is not None:
        parts.append("--ground-height %s" % args.ground_height)
    return " ".join(parts)


def _validate_args(p, args):
    # PROVENANCE_FALLBACK names environment_tiles_fixture_0 throughout, so a
    # renamed fixture can only be generated without its fallback twin.
    if args.name != "environment_tiles_fixture_0" and not args.no_fallback:
        p.error("--name requires --no-fallback (the fallback provenance text is fixture_0-specific)")


def _is_default_invocation(args):
    return (args.encoding == "ecef" and args.anchor_lat == DEFAULT_ANCHOR_LAT_DEG and
            args.anchor_lon == DEFAULT_ANCHOR_LON_DEG and
            args.name == "environment_tiles_fixture_0" and not args.no_fallback and
            args.ground_height is None)


def main():
    args = parse_args(sys.argv[1:])
    # Optional positional output_root: an output root other than this repo's
    # own tree (e.g. a scratch dir, to verify determinism by diffing two
    # independent runs without touching the committed fixtures) -- gate
    # round 1 finding 5.
    root = (os.path.abspath(args.output_root) if args.output_root
            else os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    main_dir = os.path.join(root, "tests", "fixtures", args.name)
    os.makedirs(main_dir, exist_ok=True)
    fallback_dir = None
    if not args.no_fallback:
        fallback_dir = os.path.join(root, "tests", "fixtures", fallback_dir_name(args.name))
        os.makedirs(fallback_dir, exist_ok=True)

    dlat = math.radians(args.anchor_lat - DEFAULT_ANCHOR_LAT_DEG)
    dlon = math.radians(args.anchor_lon - DEFAULT_ANCHOR_LON_DEG)
    regions = {name: shift_region(r, dlat, dlon) for name, r in TILE_REGIONS.items()}
    shifted_root_region = shift_region(ROOT_REGION, dlat, dlon)

    tiles = {}
    specs = {
        "tile_root": {"include_normal": True, "multi_buffer": False, "seed": SEED + 1,
                       "include_batchid": True},
        "tile_a": {"include_normal": True, "multi_buffer": True, "seed": SEED + 2,
                   "include_batchid": False},
        "tile_b": {"include_normal": False, "multi_buffer": False, "seed": SEED + 3,
                   "include_batchid": False},
    }
    for name, spec in specs.items():
        region = regions[name]
        ecef_center = region_center_ecef(region)
        glb = build_tile_glb(ecef_center, spec["include_normal"], spec["multi_buffer"], spec["seed"],
                              include_batchid=spec["include_batchid"], encoding=args.encoding)
        b3dm = build_b3dm(glb, ecef_center)
        tiles[name] = b3dm
        with open(os.path.join(main_dir, name + ".b3dm"), "wb") as f:
            f.write(b3dm)

    children = [
        child_entry(regions["tile_root"], "tile_root.b3dm"),
        child_entry(regions["tile_a"], "tile_a.b3dm"),
        child_entry(regions["tile_b"], "tile_b.b3dm"),
    ]
    if args.ground_height is not None:
        anchor_lat_rad = math.radians(args.anchor_lat)
        anchor_lon_rad = math.radians(args.anchor_lon)
        ground_region = ground_quad_region(anchor_lat_rad, anchor_lon_rad, args.ground_height)
        ground_ecef_center = geodetic_to_ecef(anchor_lat_rad, anchor_lon_rad, args.ground_height)
        ground_glb = build_tile_glb(ground_ecef_center, include_normal=True, multi_buffer=False,
                                    seed=SEED, include_batchid=True, encoding=args.encoding,
                                    geometry=generate_ground_quad_geometry())
        # RTC_CENTER = (0,0,0), NOT ground_ecef_center like every other
        # tile's own build_b3dm() call: this tile's authored vertex
        # positions are already absolute-ECEF (module doc, "Vertex
        # positions are baked as REAL, absolute ECEF-scale numbers"), and
        # this generator's own rendering path never reads RTC_CENTER back
        # out (same doc) -- BUT cesium-native's height-sampling path
        # (Cesium3DTilesSelection::Tileset::sampleHeightMostDetailed ->
        # CesiumGltfContent::GltfUtilities::intersectRayGltfModel ->
        # applyRtcCenter()) DOES fold RTC_CENTER back into the ray-model
        # transform. A nonzero RTC_CENTER here (as every other tile
        # authors, for b3dm format realism) would double-translate this
        # tile's already-absolute geometry away from the query ray,
        # silently failing every height sample (verified empirically:
        # sampleSuccess stayed false with 0 warnings until this was zeroed).
        ground_b3dm = build_b3dm(ground_glb, (0.0, 0.0, 0.0))
        with open(os.path.join(main_dir, "tile_ground.b3dm"), "wb") as f:
            f.write(ground_b3dm)
        children.append(child_entry(ground_region, "tile_ground.b3dm"))
        print("tile_ground.b3dm: %d bytes" % len(ground_b3dm))

    write_tileset(os.path.join(main_dir, "tileset.json"), shifted_root_region, children)
    with open(os.path.join(main_dir, "PROVENANCE.md"), "w") as f:
        if _is_default_invocation(args):
            f.write(PROVENANCE_MAIN.format(seed=SEED))
        else:
            encoding_note = (
                " -- Google-3D-Tiles-style node-LOCAL positions under a "
                "{mesh, matrix} node (see build_tile_glb()'s own comment)."
                if args.encoding == "node-matrix" else
                " -- absolute-ECEF-scale authored positions under an identity node "
                "(this generator's usual convention).")
            ground_height_line = (
                "- `tile_ground.b3dm`: a flat quad (two triangles, one flat normal) spanning "
                "+-300 m around the anchor at ellipsoid height %s m (`--ground-height`), its "
                "own tileset.json region entry.\n" % args.ground_height
                if args.ground_height is not None else "")
            f.write(PROVENANCE_GENERIC.format(
                name=args.name, seed=SEED, encoding=args.encoding, encoding_note=encoding_note,
                anchor_lat=args.anchor_lat, anchor_lon=args.anchor_lon,
                default_lat=DEFAULT_ANCHOR_LAT_DEG, default_lon=DEFAULT_ANCHOR_LON_DEG,
                cmdline=_cmdline_for_provenance(args), ground_height_line=ground_height_line))

    if fallback_dir is not None:
        # Fallback fixture: 16 byte-identical copies of tile_root, all
        # sharing its own (shifted) region.
        root_region = regions["tile_root"]
        fallback_children = [child_entry(root_region, "tile_root.b3dm")]
        with open(os.path.join(fallback_dir, "tile_root.b3dm"), "wb") as f:
            f.write(tiles["tile_root"])
        for i in range(1, 16):
            name = "tile_extra_%d.b3dm" % i
            with open(os.path.join(fallback_dir, name), "wb") as f:
                f.write(tiles["tile_root"])
            fallback_children.append(child_entry(root_region, name))
        write_tileset(os.path.join(fallback_dir, "tileset.json"), shifted_root_region,
                      fallback_children)
        with open(os.path.join(fallback_dir, "PROVENANCE.md"), "w") as f:
            f.write(PROVENANCE_FALLBACK)

    for name, b3dm in tiles.items():
        print("%s.b3dm: %d bytes" % (name, len(b3dm)))
    print("wrote %s" % main_dir)
    if fallback_dir is not None:
        print("wrote %s" % fallback_dir)


if __name__ == "__main__":
    main()
