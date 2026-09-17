#!/usr/bin/env python3
"""Bake OSM building footprints into chunked, map-frame glTF + an index
(Epic 4 Task 2 / VM-051).

Usage:
  bake_environment.py --anchor-lat LAT --anchor-lon LON [--anchor-heading-deg DEG]
                      --out DIR [--cache FILE] [--radius-m R]
  bake_environment.py --anchor-file anchor.yaml --out DIR [--cache FILE]
  bake_environment.py --selfcheck

`--anchor-lat/--anchor-lon/--anchor-heading-deg` are the SAME field order
Task 1's `overlume_node.cpp` logs the solved anchor in (VM-050 Step 4) --
copy those three logged numbers here with no unit conversion or reordering.

What this script does, precisely:
  1. Fetch building footprints for a bbox around the anchor. Overpass API is
     PRIMARY; `--cache FILE` is a read-through cache (read if present, else
     fetch live and write) so `--selfcheck` never touches the network
     (Epic 4 plan Task 2 Step 1). Mapbox (`MAPBOX_TOKEN` env var, Decision 8)
     is a fallback attempted ONLY if Overpass returns empty/errors AND the
     token is set; otherwise one WARN, zero footprints, no hard failure
     (spec Sec.9's "missing data renders nothing").
  2. Project every footprint vertex from WGS84 into the map frame using
     Task 1's own WgsToMap equirectangular math (`geo_anchor.cpp`), a second,
     independent Python implementation pinned against the C++ one via a
     committed fixture (`tests/fixtures/geo_anchor_cpp_pin_0.json`) -- see
     `wgs_to_map()`'s own comment.
  3. Extrude each footprint to a height from Decision 9's fallback chain
     (`height` tag -> `building:levels` * kMetersPerLevel -> kDefaultBuildingHeightM),
     baking the WGS84->map-frame rotation PER-GEOMETRY, never scene-level --
     the trimesh vertex-baking trap (`obj2gltf_m02p.py:17-27`): trimesh's glb
     exporter silently normalizes away a scene-level `apply_transform`
     (verified live in this task: a scene-level transform round-trips back
     to the pre-transform centroid; a per-geometry one does not). See
     `build_chunk_scene()`.
  4. Bucket footprints into ~256 m quadtree cells (`chunk_size_m`) and write
     one `.glb` per non-empty cell plus `index.yaml` (Decision 5: YAML, not
     JSON -- yaml-cpp is already linked library-side, no new JSON dep).
  5. Render `verification_overlay.png` (footprints + the recorded ego track)
     for the AC's own human sanity check (spec Sec.4.5) -- PIL, `.ppm`
     fallback if PIL is unavailable.

Self-check (--selfcheck): runs the whole pipeline against the committed
Overpass cache fixture and a handful of synthetic cases, asserting each
step's own contract. No pytest framework, matching `normalize_models.py`'s
convention.
"""
import argparse
import json
import math
import os
import struct
import sys
from pathlib import Path
from typing import List, NamedTuple, Optional, Tuple

import numpy as np
import requests
import shapely.geometry
import trimesh
import yaml

# --------------------------------------------------------------------------
# Geo math -- Python port of geo_anchor.cpp's WgsToMap (Task 1, VM-050).
# Kept in sync via the cross-language pin selfcheck below, not by shared
# source (this script has no C++ dependency; a second small copy on the
# Python side of the ABI-adjacent boundary is the same "two copies, one per
# toolchain" call Epic 3 already made for point_at/dash-walking).
# --------------------------------------------------------------------------


class GeoAnchor(NamedTuple):
    origin_lat_deg: float
    origin_lon_deg: float
    heading_rad: float  # bearing of map-frame +X from true north, radians


_DEG_TO_RAD = math.pi / 180.0
# Mean Earth radius (m) -- matches geo_anchor.cpp's kEarthRadiusM exactly;
# a spherical approximation is the AC's own stated sufficiency bound
# ("~2 km area"), not a survey-grade ellipsoid.
_EARTH_RADIUS_M = 6371000.0


def wgs_to_local_enu(anchor: GeoAnchor, lat_deg: float, lon_deg: float) -> Tuple[float, float]:
    """WGS84 -> local ENU (east, north) around the anchor's own origin, with
    NO heading rotation applied yet -- the "before per-geometry transform"
    frame `build_chunk_scene()` builds footprint meshes in."""
    lat0_rad = anchor.origin_lat_deg * _DEG_TO_RAD
    east = _EARTH_RADIUS_M * math.cos(lat0_rad) * (lon_deg - anchor.origin_lon_deg) * _DEG_TO_RAD
    north = _EARTH_RADIUS_M * (lat_deg - anchor.origin_lat_deg) * _DEG_TO_RAD
    return east, north


def local_enu_to_wgs(anchor: GeoAnchor, east: float, north: float) -> Tuple[float, float]:
    """Inverse of wgs_to_local_enu -- used only to construct synthetic
    lat/lon footprints for --selfcheck (a realistic OSM-shaped input built
    from a known map-frame target), not part of the bake pipeline itself."""
    lat0_rad = anchor.origin_lat_deg * _DEG_TO_RAD
    lat = anchor.origin_lat_deg + north / (_EARTH_RADIUS_M * _DEG_TO_RAD)
    lon = anchor.origin_lon_deg + east / (_EARTH_RADIUS_M * _DEG_TO_RAD * math.cos(lat0_rad))
    return lat, lon


def wgs_to_map(anchor: GeoAnchor, lat_deg: float, lon_deg: float, alt_m: float = 0.0) -> Tuple[float, float, float]:
    """Full WGS84 -> map-frame conversion (ENU + heading rotation). Pinned
    against the C++ WgsToMap's own output for the same anchor and 4 probe
    offsets test_geo_anchor.cpp uses -- see tests/fixtures/geo_anchor_cpp_pin_0.json
    and _selfcheck_cross_language_pin()."""
    east, north = wgs_to_local_enu(anchor, lat_deg, lon_deg)
    h = anchor.heading_rad
    s, c = math.sin(h), math.cos(h)
    return east * s + north * c, -east * c + north * s, alt_m


def _heading_rotation_matrix(heading_rad: float) -> np.ndarray:
    """4x4 rotation-only matrix matching wgs_to_map's east/north -> x/y
    rotation, for per-geometry apply_transform (never scene-level -- the
    trimesh vertex-baking trap, obj2gltf_m02p.py:17-27)."""
    s, c = math.sin(heading_rad), math.cos(heading_rad)
    return np.array(
        [
            [s, c, 0.0, 0.0],
            [-c, s, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


# --------------------------------------------------------------------------
# Height fallback chain (Decision 9, spec Sec.12's risk row, applied exactly).
# --------------------------------------------------------------------------

kMetersPerLevel = 3.0
kDefaultBuildingHeightM = 6.0  # two levels' worth


def footprint_height_m(tags: dict) -> float:
    height = tags.get("height")
    if height is not None:
        try:
            return float(height)
        except (TypeError, ValueError):
            pass
    levels = tags.get("building:levels")
    if levels is not None:
        try:
            return float(levels) * kMetersPerLevel
        except (TypeError, ValueError):
            pass
    return kDefaultBuildingHeightM


# --------------------------------------------------------------------------
# Fetch: Overpass PRIMARY (cache read-through), Mapbox fallback (Decision 8).
# --------------------------------------------------------------------------

OVERPASS_URL = "https://overpass-api.de/api/interpreter"
OVERPASS_TIMEOUT_S = 30
# ~300 m: padded past the recorded operating area's own measured net
# displacement (~102 m over 205.7 s, Epic 4 plan Decision 6) for margin.
DEFAULT_RADIUS_M = 300.0
MAPBOX_TOKEN_ENV = "MAPBOX_TOKEN"  # Decision 8 -- env var only, never logged/committed.

_OVERPASS_QUERY_TMPL = (
    "[out:json][timeout:25];\n"
    "(\n"
    '  way["building"]({south:.7f},{west:.7f},{north:.7f},{east:.7f});\n'
    ");\n"
    "out body;\n>;\nout skel qt;\n"
)


def _bbox_for_anchor(lat_deg: float, lon_deg: float, radius_m: float) -> Tuple[float, float, float, float]:
    dlat = (radius_m / _EARTH_RADIUS_M) / _DEG_TO_RAD
    dlon = (radius_m / (_EARTH_RADIUS_M * math.cos(lat_deg * _DEG_TO_RAD))) / _DEG_TO_RAD
    return lat_deg - dlat, lon_deg - dlon, lat_deg + dlat, lon_deg + dlon  # south, west, north, east


def fetch_overpass_json(anchor_lat: float, anchor_lon: float, radius_m: float,
                         cache_path: Optional[Path]) -> Optional[dict]:
    """Overpass PRIMARY with --cache read-through: read the cache if it
    exists (NO network call -- this is what makes --selfcheck network-free),
    else fetch live and write the cache. Returns None on any live-fetch
    error (non-fatal -- caller WARNs and proceeds with zero footprints)."""
    if cache_path is not None and cache_path.exists():
        return json.loads(cache_path.read_text())

    south, west, north, east = _bbox_for_anchor(anchor_lat, anchor_lon, radius_m)
    query = _OVERPASS_QUERY_TMPL.format(south=south, west=west, north=north, east=east)
    try:
        resp = requests.post(OVERPASS_URL, data=query, timeout=OVERPASS_TIMEOUT_S)
        resp.raise_for_status()
        data = resp.json()
    except Exception as exc:  # network/HTTP/JSON error -- all non-fatal here
        print(f"WARN: Overpass fetch failed: {exc}", file=sys.stderr)
        return None

    if cache_path is not None:
        cache_path.parent.mkdir(parents=True, exist_ok=True)
        cache_path.write_text(json.dumps(data))
    return data


def parse_overpass_footprints(data: Optional[dict]) -> List[dict]:
    """Overpass JSON -> [{"way_id", "tags", "ring": [(lat, lon), ...]}, ...].
    Ways with a node ref not present in the response (can happen at a bbox
    edge) are skipped rather than crashing -- real OSM data has this shape
    occasionally, and a partial ring is not a usable footprint."""
    if not data or "elements" not in data:
        return []
    nodes = {}
    ways = []
    for el in data["elements"]:
        if el.get("type") == "node":
            nodes[el["id"]] = (el["lat"], el["lon"])
        elif el.get("type") == "way" and "building" in el.get("tags", {}):
            ways.append(el)

    footprints = []
    for w in ways:
        ring = [nodes[n] for n in w["nodes"] if n in nodes]
        if len(ring) < 3:
            continue  # degenerate -- not a usable polygon
        footprints.append({"way_id": w["id"], "tags": w.get("tags", {}), "ring": ring})
    return footprints


def fetch_mapbox_fallback_footprints(anchor: GeoAnchor, radius_m: float, token: str) -> List[dict]:
    """Best-effort Mapbox fallback (Decision 8): only reached when Overpass
    is empty/errored AND MAPBOX_TOKEN is set. Uses the Tilequery API
    (https://docs.mapbox.com/api/maps/tilequery/) sampling a small grid of
    points across the bbox and building a small square footprint (side
    kMapboxFallbackFootprintSideM) around each hit -- Tilequery's own
    response geometry for a polygon feature is always a POINT (where the
    query point fell within it), not the true outline, so this is a
    documented approximation, not a full footprint.
    ponytail: real polygon outlines need vector-tile decoding
    (mapbox_vector_tile / mercantile, neither installed here per the
    reuse-first ladder -- adding a new dependency for a fallback path no
    --selfcheck test exercises isn't warranted). Upgrade if the Mapbox path
    ever becomes load-bearing (i.e. Overpass coverage turns out to be
    genuinely insufficient for a real operating area).
    """
    kMapboxFallbackFootprintSideM = 8.0
    kMapboxFallbackGridStepM = 40.0
    south, west, north, east = _bbox_for_anchor(anchor.origin_lat_deg, anchor.origin_lon_deg, radius_m)
    footprints = []
    lat = south
    while lat <= north:
        lon = west
        while lon <= east:
            try:
                resp = requests.get(
                    f"https://api.mapbox.com/v4/mapbox.mapbox-streets-v8/tilequery/{lon},{lat}.json",
                    params={"layers": "building", "radius": 5, "access_token": token},
                    timeout=10,
                )
                resp.raise_for_status()
                data = resp.json()
            except Exception:
                data = {"features": []}
            for feat in data.get("features", []):
                fx, fy = wgs_to_local_enu(anchor, lat, lon)
                half = kMapboxFallbackFootprintSideM / 2.0
                # ring in local ENU converted back to lat/lon so downstream
                # code (parse_overpass_footprints's own consumers) sees the
                # same {"way_id","tags","ring":[(lat,lon),...]} shape either
                # source produces.
                ring_ll = [
                    local_enu_to_wgs(anchor, fx - half, fy - half),
                    local_enu_to_wgs(anchor, fx + half, fy - half),
                    local_enu_to_wgs(anchor, fx + half, fy + half),
                    local_enu_to_wgs(anchor, fx - half, fy + half),
                ]
                footprints.append({"way_id": f"mapbox_{lat:.6f}_{lon:.6f}", "tags": {}, "ring": ring_ll})
            lon += (kMapboxFallbackGridStepM / _EARTH_RADIUS_M) / _DEG_TO_RAD
        lat += (kMapboxFallbackGridStepM / _EARTH_RADIUS_M) / _DEG_TO_RAD
    return footprints


def fetch_footprints(anchor: GeoAnchor, radius_m: float, cache_path: Optional[Path]) -> List[dict]:
    """Orchestrates Overpass PRIMARY + Mapbox fallback exactly per Decision 8:
    Mapbox is attempted only if Overpass is empty/errored AND MAPBOX_TOKEN is
    set; otherwise one WARN, zero footprints (spec Sec.9)."""
    data = fetch_overpass_json(anchor.origin_lat_deg, anchor.origin_lon_deg, radius_m, cache_path)
    footprints = parse_overpass_footprints(data)
    if footprints:
        return footprints

    token = os.environ.get(MAPBOX_TOKEN_ENV)
    if token:
        footprints = fetch_mapbox_fallback_footprints(anchor, radius_m, token)
        if footprints:
            return footprints
        print("WARN: Overpass returned no footprints and the Mapbox fallback "
              "also returned none -- proceeding with zero footprints for this bbox.",
              file=sys.stderr)
    else:
        print(f"WARN: Overpass returned no footprints and {MAPBOX_TOKEN_ENV} is not "
              "set -- proceeding with zero footprints for this bbox (spec Sec.9: "
              "missing data renders nothing).", file=sys.stderr)
    return []


# --------------------------------------------------------------------------
# Chunking + bake (Decision 3: map-frame placement happens HERE, at bake
# time, never re-projected at load time).
# --------------------------------------------------------------------------

CHUNK_SIZE_M = 256.0
CHUNK_RADIUS_M = CHUNK_SIZE_M * math.sqrt(2.0) / 2.0  # bounding-sphere radius of one cell


def build_footprint_mesh_local_enu(fp: dict, anchor: GeoAnchor) -> Optional[trimesh.Trimesh]:
    """Extrudes one footprint to its Decision-9 height, in UNROTATED local
    ENU (anchor origin only, no heading) -- the "before per-geometry
    transform" frame. shapely.Polygon + trimesh.creation.extrude_polygon,
    both already-installed dependencies (no hand-rolled triangulation)."""
    ring_enu = [wgs_to_local_enu(anchor, lat, lon) for lat, lon in fp["ring"]]
    polygon = shapely.geometry.Polygon(ring_enu)
    if not polygon.is_valid:
        polygon = polygon.buffer(0)  # best-effort repair of minor self-intersections
    # buffer(0) can return a MultiPolygon or an empty geometry for a badly
    # self-intersecting way -- extrude_polygon would raise. Same non-fatal
    # "missing data renders nothing" shape as parse_overpass_footprints'
    # len(ring) < 3 skip: return None, caller skips with one WARN.
    if not isinstance(polygon, shapely.geometry.Polygon) or polygon.is_empty:
        return None
    height = footprint_height_m(fp["tags"])
    return trimesh.creation.extrude_polygon(polygon, height)


def footprint_map_centroid(fp: dict, anchor: GeoAnchor) -> Tuple[float, float]:
    """Footprint centroid in the MAP frame -- used only to bucket the
    footprint into a chunk cell. Rotating the (cheap, 2D) centroid directly
    is equivalent to rotating every vertex first and then centroiding (the
    heading rotation is linear), so this doesn't need to re-load the
    exported mesh."""
    ring_enu = [wgs_to_local_enu(anchor, lat, lon) for lat, lon in fp["ring"]]
    cx, cy = shapely.geometry.Polygon(ring_enu).centroid.coords[0]
    s, c = math.sin(anchor.heading_rad), math.cos(anchor.heading_rad)
    return cx * s + cy * c, -cx * c + cy * s


def chunk_key(x: float, y: float, chunk_size_m: float = CHUNK_SIZE_M) -> Tuple[int, int]:
    return math.floor(x / chunk_size_m), math.floor(y / chunk_size_m)


def build_chunk_scene(footprints: List[dict], anchor: GeoAnchor) -> trimesh.Scene:
    """Builds every footprint's mesh in unrotated local ENU, adds each as
    its OWN geometry in one Scene, then bakes the heading rotation into
    vertices PER-GEOMETRY -- never scene-level (the trimesh vertex-baking
    trap, obj2gltf_m02p.py:17-27: a scene-level apply_transform is silently
    normalized away by the glb exporter's Y-up node-matrix rewrite;
    per-geometry vertices survive export/reload, verified directly against
    this trimesh version during implementation)."""
    scene = trimesh.Scene()
    for fp in footprints:
        mesh = build_footprint_mesh_local_enu(fp, anchor)
        if mesh is None:
            print(f"WARN: footprint way {fp['way_id']} has an unrepairable ring -- skipped")
            continue
        scene.add_geometry(mesh, node_name=f"footprint_{fp['way_id']}")

    M = _heading_rotation_matrix(anchor.heading_rad)
    for geom in scene.geometry.values():  # PER-GEOMETRY, not scene.apply_transform(M)
        geom.apply_transform(M)
    return scene


def bake(anchor: GeoAnchor, footprints: List[dict], out_dir: Path) -> dict:
    """Chunks `footprints` (~256 m quadtree cells), writes chunks/*.glb +
    index.yaml under out_dir, and returns the index dict."""
    buckets = {}
    for fp in footprints:
        x, y = footprint_map_centroid(fp, anchor)
        buckets.setdefault(chunk_key(x, y), []).append(fp)

    chunks_dir = out_dir / "chunks"
    chunks_dir.mkdir(parents=True, exist_ok=True)

    index_chunks = []
    for (i, j), fps in sorted(buckets.items()):
        chunk_id = f"chunk_{i}_{j}"
        scene = build_chunk_scene(fps, anchor)
        if not scene.geometry:
            continue  # every footprint in this cell was unrepairable -- no empty chunk file
        # include_normals=True: trimesh's glb exporter otherwise only writes
        # NORMAL when something already touched mesh.vertex_normals before
        # export (its own default is "include only if already cached" --
        # nothing here ever reads .vertex_normals, so every chunk baked
        # without this came out POSITION-only and rendered flat/unlit
        # regardless of palette or lighting). The load-time fix
        # (gltf_normals.hpp) still covers chunks already baked without this
        # -- this is the cheap, correct-at-the-source half for future bakes.
        scene.export(chunks_dir / f"{chunk_id}.glb", include_normals=True)
        index_chunks.append(
            {
                "id": chunk_id,
                "path": f"chunks/{chunk_id}.glb",
                "center": [(i + 0.5) * CHUNK_SIZE_M, (j + 0.5) * CHUNK_SIZE_M, 0.0],
                "radius_m": CHUNK_RADIUS_M,
            }
        )

    index = {"chunk_size_m": CHUNK_SIZE_M, "chunks": index_chunks}
    (out_dir / "index.yaml").write_text(yaml.safe_dump(index, sort_keys=False))
    return index


def _glb_has_normals(glb_path: Path) -> bool:
    """True iff every mesh primitive this chunk's .glb carries actually
    wrote a NORMAL attribute (finding #31: include_normals=True above is
    otherwise unchecked at the source -- only ensure_flat_normals()
    covers a regression, silently, at load time). Reads the GLB's JSON
    chunk directly (12-byte header + 8-byte chunk header, glTF 2.0 binary
    spec) rather than through trimesh, which synthesizes vertex_normals on
    access regardless of whether the file itself has them."""
    data = glb_path.read_bytes()
    chunk_length, chunk_type = struct.unpack_from("<I4s", data, 12)
    if chunk_type != b"JSON":
        return False
    gltf = json.loads(data[20 : 20 + chunk_length])
    primitives = [p for mesh in gltf.get("meshes", []) for p in mesh.get("primitives", [])]
    return bool(primitives) and all("NORMAL" in p.get("attributes", {}) for p in primitives)


def _load_all_vertices(glb_path: Path) -> np.ndarray:
    """Loads a baked chunk .glb and returns every vertex across all its
    geometries (a chunk may hold >1 footprint as separate geometries,
    Decision 4 -- Task 3's material remap works regardless of grouping)."""
    loaded = trimesh.load(glb_path)
    if isinstance(loaded, trimesh.Scene):
        if not loaded.geometry:
            return np.zeros((0, 3))
        return np.vstack([g.vertices for g in loaded.geometry.values()])
    return loaded.vertices


# --------------------------------------------------------------------------
# Verification overlay (spec Sec.4.5's own AC: "overlay image sanity-approved").
# --------------------------------------------------------------------------


def load_ego_track_xy(csv_path: Path) -> List[Tuple[float, float]]:
    """Reads Task 1's own committed (lat, lon, map_x, map_y) fixture and
    returns the map_x/map_y columns -- reused as-is (no new ego-track
    fixture for this task; this CSV already IS a short recorded slice of
    the real map-frame ego track)."""
    track = []
    for line in csv_path.read_text().splitlines():
        if not line or line.startswith("#"):
            continue
        _lat, _lon, x, y = line.split(",")
        track.append((float(x), float(y)))
    return track


def _footprint_map_ring(fp: dict, anchor: GeoAnchor) -> List[Tuple[float, float]]:
    return [wgs_to_map(anchor, lat, lon)[:2] for lat, lon in fp["ring"]]


def render_verification_overlay(footprints: List[dict], anchor: GeoAnchor,
                                 ego_track_xy: List[Tuple[float, float]], out_path: Path,
                                 size: Tuple[int, int] = (800, 800)) -> Path:
    """Rasterizes every footprint's map-frame outline + the recorded ego
    track onto one image -- a human-sanity-check artifact (spec's own AC),
    not something an automated test can judge "looks right". PIL if
    available; else a `.ppm` fallback (still openable), so a missing PIL
    doesn't hard-fail the bake."""
    out_path.parent.mkdir(parents=True, exist_ok=True)
    points = [pt for fp in footprints for pt in _footprint_map_ring(fp, anchor)] + list(ego_track_xy)
    if not points:
        points = [(0.0, 0.0), (1.0, 1.0)]  # degenerate but non-crashing bounds
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    margin = 0.05 * max(max(xs) - min(xs), max(ys) - min(ys), 1.0)
    min_x, max_x = min(xs) - margin, max(xs) + margin
    min_y, max_y = min(ys) - margin, max(ys) + margin
    w, h = size

    def to_px(x: float, y: float) -> Tuple[int, int]:
        px = int((x - min_x) / (max_x - min_x) * (w - 1))
        py = int((1.0 - (y - min_y) / (max_y - min_y)) * (h - 1))  # flip Y for image coords
        return px, py

    try:
        from PIL import Image, ImageDraw

        img = Image.new("RGB", size, (255, 255, 255))
        draw = ImageDraw.Draw(img)
        for fp in footprints:
            ring_px = [to_px(x, y) for x, y in _footprint_map_ring(fp, anchor)]
            draw.polygon(ring_px, outline=(40, 40, 40), fill=(190, 190, 200))
        if len(ego_track_xy) >= 2:
            draw.line([to_px(x, y) for x, y in ego_track_xy], fill=(220, 30, 30), width=2)
        out_path = out_path.with_suffix(".png")
        img.save(out_path)
    except ImportError:
        # documented fallback: a raw PPM (P6), still openable by any image
        # viewer, no new dependency.
        buf = bytearray(b"\xff" * (w * h * 3))

        def set_px(px: int, py: int, color: Tuple[int, int, int]) -> None:
            if 0 <= px < w and 0 <= py < h:
                i = (py * w + px) * 3
                buf[i : i + 3] = bytes(color)

        for fp in footprints:
            for x, y in _footprint_map_ring(fp, anchor):
                set_px(*to_px(x, y), (40, 40, 40))
        for x, y in ego_track_xy:
            set_px(*to_px(x, y), (220, 30, 30))
        out_path = out_path.with_suffix(".ppm")
        with open(out_path, "wb") as f:
            f.write(f"P6\n{w} {h}\n255\n".encode("ascii"))
            f.write(bytes(buf))
    return out_path


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

_HERE = Path(__file__).resolve().parent
_FIXTURES_DIR = _HERE.parent / "tests" / "fixtures"
_DEFAULT_OVERPASS_CACHE = _FIXTURES_DIR / "environment_overpass_cache_0.json"


def _load_anchor_from_args(args: argparse.Namespace) -> GeoAnchor:
    if args.anchor_file:
        data = yaml.safe_load(Path(args.anchor_file).read_text())
        return GeoAnchor(
            float(data["anchor_lat_deg"]),
            float(data["anchor_lon_deg"]),
            math.radians(float(data.get("anchor_heading_deg", 0.0))),
        )
    if args.anchor_lat is None or args.anchor_lon is None:
        raise SystemExit("--anchor-lat/--anchor-lon (or --anchor-file) are required")
    return GeoAnchor(args.anchor_lat, args.anchor_lon, math.radians(args.anchor_heading_deg or 0.0))


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--anchor-lat", type=float)
    parser.add_argument("--anchor-lon", type=float)
    parser.add_argument("--anchor-heading-deg", type=float, default=0.0)
    parser.add_argument("--anchor-file", type=str, help="YAML side-file: anchor_lat_deg/anchor_lon_deg/anchor_heading_deg")
    parser.add_argument("--out", type=str)
    parser.add_argument("--cache", type=str, help="Overpass response cache (read-through)")
    parser.add_argument("--radius-m", type=float, default=DEFAULT_RADIUS_M)
    parser.add_argument("--ego-track", type=str,
                         help="Task 1's (lat,lon,map_x,map_y) CSV fixture, for the verification "
                              "overlay's ego-track line (optional -- overlay is footprints-only "
                              "without it)")
    parser.add_argument("--selfcheck", action="store_true")
    args = parser.parse_args(argv)

    if args.selfcheck:
        return _selfcheck()

    if not args.out:
        parser.error("--out is required")
    anchor = _load_anchor_from_args(args)
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    cache_path = Path(args.cache) if args.cache else None

    footprints = fetch_footprints(anchor, args.radius_m, cache_path)
    index = bake(anchor, footprints, out_dir)
    # Ego track is optional (--ego-track, Task 1's own committed CSV, e.g.
    # ros/src/overlume_ros/test/fixtures/geo_anchor_samples_0.csv) --
    # footprints-only overlay if not given, rather than hard-failing a bake
    # over a QA-only line (spec's AC is about footprint placement sanity
    # first; the ego track is corroborating context, not a requirement).
    ego_track = load_ego_track_xy(Path(args.ego_track)) if args.ego_track else []
    overlay_path = render_verification_overlay(footprints, anchor, ego_track, out_dir / "verification_overlay.png")
    print(f"wrote {len(index['chunks'])} chunk(s), {len(footprints)} footprint(s), "
          f"index.yaml, {overlay_path.name}")
    return 0


# --------------------------------------------------------------------------
# --selfcheck -- no pytest framework, matching normalize_models.py's convention.
# Each _selfcheck_* mirrors one plan Step (0-5); all must pass.
# --------------------------------------------------------------------------


def _selfcheck_step0_cache_no_network() -> bool:
    """Step 0/1: --selfcheck must NEVER hit the network -- monkeypatch
    requests.post to assert it is never called when a cache file exists,
    and that footprints still come out of the cache."""
    calls = {"n": 0}
    orig_post = requests.post

    def _blocked_post(*_a, **_kw):
        calls["n"] += 1
        raise AssertionError("selfcheck must never hit the network")

    requests.post = _blocked_post
    try:
        data = fetch_overpass_json(25.0803, 55.3910, DEFAULT_RADIUS_M, _DEFAULT_OVERPASS_CACHE)
    finally:
        requests.post = orig_post

    footprints = parse_overpass_footprints(data)
    ok = calls["n"] == 0 and len(footprints) > 0
    print(f"[step0 cache-no-network] live_calls={calls['n']} footprints={len(footprints)} -> {'OK' if ok else 'FAIL'}")
    return ok


def _selfcheck_step2_centroid_and_trap(tmp_dir: Path) -> bool:
    """Step 2: synthetic single footprint at a known map-frame offset from
    the anchor; bake; reload the .glb; assert its vertex centroid lands at
    the expected map-frame XY within 0.1 m -- the exact failure mode the
    trimesh vertex-baking trap warns about (a scene-level transform still
    "succeeds" with silently wrong geometry, verified live in this task:
    scene-level transform round-trips a centroid back to (0,0), per-geometry
    does not)."""
    anchor = GeoAnchor(25.0803, 55.3910, 0.3)  # nonzero heading -- discriminates the trap
    target_east, target_north = 150.0, -80.0  # a known, arbitrary local-ENU target
    half = 5.0
    ring_enu = [
        (target_east - half, target_north - half),
        (target_east + half, target_north - half),
        (target_east + half, target_north + half),
        (target_east - half, target_north + half),
    ]
    ring_ll = [local_enu_to_wgs(anchor, e, n) for e, n in ring_enu]
    fp = {"way_id": "synthetic_0", "tags": {}, "ring": ring_ll}

    index = bake(anchor, [fp], tmp_dir)
    assert len(index["chunks"]) == 1
    glb_path = tmp_dir / index["chunks"][0]["path"]
    verts = _load_all_vertices(glb_path)

    expected_x = target_east * math.sin(anchor.heading_rad) + target_north * math.cos(anchor.heading_rad)
    expected_y = -target_east * math.cos(anchor.heading_rad) + target_north * math.sin(anchor.heading_rad)
    actual = verts.mean(axis=0)
    dx, dy = actual[0] - expected_x, actual[1] - expected_y
    ok = math.hypot(dx, dy) < 0.1
    print(f"[step2 centroid-trap] expected=({expected_x:.3f},{expected_y:.3f}) "
          f"actual=({actual[0]:.3f},{actual[1]:.3f}) err={math.hypot(dx, dy):.4f} m -> {'OK' if ok else 'FAIL'}")
    return ok


def _selfcheck_cross_language_pin() -> bool:
    """Step 2 (cross-language pin): Python's wgs_to_map() must match the
    committed C++-generated fixture within 1e-3 m per point."""
    fixture = json.loads((_FIXTURES_DIR / "geo_anchor_cpp_pin_0.json").read_text())
    a = fixture["anchor"]
    anchor = GeoAnchor(a["origin_lat_deg"], a["origin_lon_deg"], a["heading_rad"])
    ok = True
    for probe in fixture["probes"]:
        x, y, z = wgs_to_map(anchor, probe["lat"], probe["lon"])
        err = max(abs(x - probe["map_x"]), abs(y - probe["map_y"]), abs(z - probe["map_z"]))
        point_ok = err < 1e-3
        ok = ok and point_ok
        print(f"[cross-lang pin] dlat={probe['dlat']} dlon={probe['dlon']} err={err:.2e} m -> "
              f"{'OK' if point_ok else 'FAIL'}")
    # Nonzero-heading probes pin the rotation term itself -- a sign drift
    # confined to the sin factor passes every heading-0 probe unchanged.
    for probe in fixture.get("probes_nonzero_heading", []):
        pa = probe["anchor"]
        p_anchor = GeoAnchor(pa["origin_lat_deg"], pa["origin_lon_deg"], pa["heading_rad"])
        x, y, z = wgs_to_map(p_anchor, probe["lat"], probe["lon"])
        err = max(abs(x - probe["map_x"]), abs(y - probe["map_y"]), abs(z - probe["map_z"]))
        point_ok = err < 1e-3
        ok = ok and point_ok
        print(f"[cross-lang pin] heading={pa['heading_rad']} err={err:.2e} m -> "
              f"{'OK' if point_ok else 'FAIL'}")
    return ok


def _selfcheck_step3_chunking(tmp_dir: Path) -> bool:
    """Step 3: two footprints >256 m apart bake into two distinct chunk
    files; two placed in the same cell bake into one."""
    anchor = GeoAnchor(25.0803, 55.3910, 0.0)  # heading 0 -- map frame == local ENU, easiest to reason about

    def _square_fp(way_id: str, cx: float, cy: float, side: float = 4.0) -> dict:
        half = side / 2.0
        ring_enu = [
            (cx - half, cy - half), (cx + half, cy - half),
            (cx + half, cy + half), (cx - half, cy + half),
        ]
        return {"way_id": way_id, "tags": {}, "ring": [local_enu_to_wgs(anchor, e, n) for e, n in ring_enu]}

    far = [_square_fp("far_a", 10.0, 10.0), _square_fp("far_b", 300.0, 10.0)]
    index_far = bake(anchor, far, tmp_dir / "far")
    ok_far = len(index_far["chunks"]) == 2

    near = [_square_fp("near_a", 10.0, 10.0), _square_fp("near_b", 20.0, 20.0)]
    index_near = bake(anchor, near, tmp_dir / "near")
    ok_near = len(index_near["chunks"]) == 1

    # Index-row contract: center/radius_m are exactly what Task 3's distance
    # cull consumes -- assert them, don't just count files. Vertex containment
    # has a known ceiling: footprints are bucketed by CENTROID, so a building
    # straddling a cell edge can poke outside the cell's bounding sphere by up
    # to ~half its own extent -- Task 3's cull margin must absorb that
    # overhang (these 4 m test squares sit well inside their cells).
    ok_rows = True
    for base, index in (("far", index_far), ("near", index_near)):
        for row in index["chunks"]:
            i, j = (int(v) for v in row["id"].split("_")[1:3])
            ok_rows &= row["center"] == [(i + 0.5) * CHUNK_SIZE_M, (j + 0.5) * CHUNK_SIZE_M, 0.0]
            ok_rows &= row["radius_m"] == CHUNK_RADIUS_M
            verts = _load_all_vertices(tmp_dir / base / row["path"])
            d = np.hypot(verts[:, 0] - row["center"][0], verts[:, 1] - row["center"][1])
            ok_rows &= bool((d <= row["radius_m"]).all())

    ok = ok_far and ok_near and ok_rows
    print(f"[step3 chunking] far->{len(index_far['chunks'])} chunk(s) (want 2), "
          f"near->{len(index_near['chunks'])} chunk(s) (want 1), "
          f"index rows {'OK' if ok_rows else 'FAIL'} -> {'OK' if ok else 'FAIL'}")
    return ok


def _selfcheck_step4_overlay(tmp_dir: Path) -> bool:
    """Step 4: the overlay artifact is written, non-empty, and has
    non-degenerate pixel dimensions -- this test only proves the artifact
    exists and is well-formed; "looks right" is the AC's own human step."""
    anchor = GeoAnchor(25.0803, 55.3910, 0.1)
    fp = {"way_id": "overlay_fp", "tags": {}, "ring": [
        local_enu_to_wgs(anchor, e, n) for e, n in [(-5, -5), (5, -5), (5, 5), (-5, 5)]
    ]}
    ego_track = [(0.0, 0.0), (10.0, 5.0), (20.0, -3.0)]

    # Pins load_ego_track_xy()'s column contract (lat,lon,map_x,map_y) against
    # Task 1's exact fixture format, independent of Task 1's file being present.
    csv_path = tmp_dir / "ego_track_0.csv"
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    csv_path.write_text("# lat_deg,lon_deg,map_x,map_y\n25.08031512,55.39096843,-48.514142,-1.027173\n")
    parsed_track = load_ego_track_xy(csv_path)
    ok_parse = parsed_track == [(-48.514142, -1.027173)]
    print(f"[step4 ego-track-parse] parsed={parsed_track} -> {'OK' if ok_parse else 'FAIL'}")

    out_path = render_verification_overlay([fp], anchor, ego_track, tmp_dir / "verification_overlay.png")
    ok = out_path.exists() and out_path.stat().st_size > 0
    if ok:
        if out_path.suffix == ".png":
            from PIL import Image

            with Image.open(out_path) as img:
                ok = img.width > 1 and img.height > 1
        else:  # .ppm fallback
            header = out_path.read_bytes()[:32].decode("ascii", errors="ignore")
            w, h = map(int, header.split("\n")[1].split())
            ok = w > 1 and h > 1
    print(f"[step4 overlay] wrote {out_path.name} ({out_path.stat().st_size} bytes) -> {'OK' if ok else 'FAIL'}")
    return ok and ok_parse


def _selfcheck_step5_height_fallback(tmp_dir: Path) -> bool:
    """Step 5: three footprints -- height tag, building:levels tag, neither
    -- each baked alone (well-separated so each lands in its own chunk) and
    checked against Decision 9's fallback chain via the .glb's own Z extent."""
    anchor = GeoAnchor(25.0803, 55.3910, 0.0)

    def _square_fp(way_id: str, cx: float, cy: float, tags: dict) -> dict:
        half = 4.0
        ring_enu = [(cx - half, cy - half), (cx + half, cy - half), (cx + half, cy + half), (cx - half, cy + half)]
        return {"way_id": way_id, "tags": tags, "ring": [local_enu_to_wgs(anchor, e, n) for e, n in ring_enu]}

    cases = [
        ("has_height", _square_fp("has_height", 10.0, 10.0, {"height": "12.5"}), 12.5),
        ("has_levels", _square_fp("has_levels", 400.0, 10.0, {"building:levels": "3"}), 3 * kMetersPerLevel),
        ("has_neither", _square_fp("has_neither", 800.0, 10.0, {}), kDefaultBuildingHeightM),
    ]
    ok = True
    for name, fp, expected_h in cases:
        index = bake(anchor, [fp], tmp_dir / name)
        glb_path = tmp_dir / name / index["chunks"][0]["path"]
        verts = _load_all_vertices(glb_path)
        z_extent = verts[:, 2].max() - verts[:, 2].min()
        case_ok = abs(z_extent - expected_h) < 1e-6
        ok = ok and case_ok
        print(f"[step5 height-fallback] {name}: z_extent={z_extent:.3f} expected={expected_h:.3f} -> "
              f"{'OK' if case_ok else 'FAIL'}")
    return ok


def _selfcheck_end_to_end(tmp_dir: Path) -> bool:
    """Full pipeline against the committed Overpass cache fixture: output
    chunk count > 0, every chunk .glb non-empty and loadable, index.yaml
    parses and every listed chunk file exists, footprint vertex count per
    chunk is sane (> 0), and every chunk actually carries NORMAL (finding
    #31 -- the at-the-source half of the include_normals=True change was
    otherwise unchecked)."""
    anchor = GeoAnchor(25.0803, 55.3910, 0.0)
    footprints = fetch_footprints(anchor, DEFAULT_RADIUS_M, _DEFAULT_OVERPASS_CACHE)
    out_dir = tmp_dir / "end_to_end"
    index = bake(anchor, footprints, out_dir)

    ok = len(index["chunks"]) > 0
    parsed = yaml.safe_load((out_dir / "index.yaml").read_text())
    ok = ok and parsed == index
    for entry in index["chunks"]:
        glb_path = out_dir / entry["path"]
        ok = ok and glb_path.exists() and glb_path.stat().st_size > 0
        verts = _load_all_vertices(glb_path)
        ok = ok and verts.shape[0] > 0
        ok = ok and _glb_has_normals(glb_path)
    print(f"[end-to-end] chunks={len(index['chunks'])} footprints={len(footprints)} -> {'OK' if ok else 'FAIL'}")
    return ok


def _selfcheck() -> int:
    import tempfile

    def _blocked(*_a, **_kw):
        raise AssertionError("selfcheck must never hit the network")

    orig_post, orig_get = requests.post, requests.get
    requests.post, requests.get = _blocked, _blocked
    try:
        with tempfile.TemporaryDirectory(prefix="bake_environment_selfcheck_") as td:
            tmp_dir = Path(td)
            results = [
                _selfcheck_step0_cache_no_network(),
                _selfcheck_step2_centroid_and_trap(tmp_dir / "step2"),
                _selfcheck_cross_language_pin(),
                _selfcheck_step3_chunking(tmp_dir / "step3"),
                _selfcheck_step4_overlay(tmp_dir / "step4"),
                _selfcheck_step5_height_fallback(tmp_dir / "step5"),
                _selfcheck_end_to_end(tmp_dir / "e2e"),
            ]
    finally:
        requests.post, requests.get = orig_post, orig_get
    ok = all(results)
    print(f"selfcheck: {'PASS' if ok else 'FAIL'} ({sum(results)}/{len(results)})")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
