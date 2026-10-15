# Mapbox buildings runbook

How to run the `mapbox://buildings` environment source: Mapbox's landmark models plus building
footprints from Mapbox Streets v8, extruded to their heights, streamed around the vehicle in
visual mode (mode 3). The design is
[`2026-10-14-mapbox-buildings-design.md`](../design/2026-10-14-mapbox-buildings-design.md).

Until the maintainer confirms Mapbox's sign-off, the source stays on the local
`mapbox-environment` branch and nothing from it reaches `origin` ([`status.md`](../status.md),
open items).

## 1. The token

- The library reads `MAPBOX_TOKEN` from its environment each time a `mapbox://` source opens, and
  trims surrounding whitespace. No parameter, file or URI key carries it.
- Unset or empty: the source does not open, and one log line names the variable. A live switch
  is refused and the previous source stays. When the launch-time URI arms at activate (§7.2), the
  node logs `set_environment_source: failed to open '…' -- no buildings this run`.
- Put it in the deploy user's `~/.bashrc`, then open a new shell:

  ```bash
  export MAPBOX_TOKEN=<paste the token here>
  ```

- `tools/validate_visual_mode.sh` re-reads `MAPBOX_TOKEN` and `CESIUM_ION_TOKEN` from a fresh
  login shell before it starts the node, because a long-lived terminal can hold a stale value. It
  prints a `[token]` line when the two differed, never the value.
- The token never appears in the URI, a parameter, a log line, a model or a file. The library
  sends it only as the `access_token` query parameter of its own requests, its log lines go
  through the redacting logger (`access_token=<redacted>`), and the source writes no cache.
- To rotate it, change the export and restart the rig. A running node keeps the environment it
  started with.
- If the token ever lands in a log, a file or a transcript, rotate it. Redaction is not the
  remedy.

## 2. Preflight: `tools/check_mapbox_token.sh`

Run it before every demo. It is never part of the gate: it needs the network and the real token.

```bash
tools/check_mapbox_token.sh
```

It requests the two endpoints the source uses, at Downtown Dubai: the landmark tile z14
10707/7006 and the footprint tile z16 42830/28025 inside it. Like
`overlume/scripts/check_cesium_token.sh`, it prints one line. A working token prints:

```text
PASS: token retrieves the Downtown Dubai tiles (landmark tile HTTP 200, footprint tile HTTP 200)
```

- Exit 0 on `PASS:`. Exit 1 on `FAIL:`, which names each tile's HTTP code or transport error.
  Exit 2 on `SKIP:`.
- `SKIP: MAPBOX_TOKEN is not set`: no request was sent. Export the token (§1).
- `HTTP 401` on both: Mapbox rejects the token. Check it, or make a new one.
- `HTTP 403`: the token is valid but not allowed here (URL restrictions or scopes). Use a token
  without URL restrictions.
- `landmark tile HTTP 204`: Mapbox no longer serves landmarks there. The landmark endpoint is
  undocumented (§8); the source still shows extrusions.
- `transport error (curl exit N)`: no network. N is curl's exit status: 6 for DNS, 7 for a
  refused connection, 28 for the 30 s timeout.
- The script prints no URL, token or response body. curl reads the URL from stdin, so the token
  is not on its command line either.

## 3. The URI

`mapbox://buildings[?key=value(&key=value)*]`

| Key | Values | Default | Meaning |
|-----|--------|---------|---------|
| `materials` | `clay`, `original` | `clay` | `clay`: the theme's building material, like `osm`. `original`: Mapbox's landmark materials, and Mapbox Standard's building colour on extrusions, like `google`. Any other value logs one warning and means `clay`. |
| `radius` | finite > 0 (m) | 500 | Load and display radius around the vehicle. |
| `brightness` | finite > 0 | 1.0 | Scales base colours under `materials=original`. |
| `fallback` | a baked chunk directory | none | Shown after network loss (§8). |
| `cache` | `off` | `off` | `off` changes nothing. Any other value refuses the URI: Mapbox tiles are never written to disk. |

- `buildings` is the only layer. Any other path refuses the URI, and one log line names the
  supported layers.
- Apart from `materials`, a bad value for a known key refuses the whole URI. Unknown keys are
  ignored, and so are the ion-only keys `follow_terrain`, `ground_bias`, `replaces_ground`,
  `max_tilt_deg` and `max_cache_items`.
- The source never follows terrain and never replaces the clay ground plane. Buildings stand on
  the flat map plane at z = 0.
- A library built without cesium (`OVERLUME_ENABLE_CESIUM=OFF`) refuses `mapbox://`, as it refuses
  `ion://`.

The node composes the URI from `environment_source_uri` and its own parameters:

- `fallback=<environment_chunks_dir>` when that directory is set;
- `radius=<environment_tile_radius_m>` when it is not 700, the node default. At 700 the source's
  own 500 m default applies;
- `brightness=<environment_brightness>` when it is not 1.0;
- never `cache=`, and never an ion-only key.

Each z14 tile (about 2.2 km wide in Dubai) costs 17 requests: one landmark model tile and 16
footprint tiles. With the 500 m radius, one to four tiles are live at a time. Footprint tiles are
billed per request, so the 200,000-a-month free tier covers about 12,500 z14 tiles, or about
11,700 if Mapbox also bills the landmark requests. The landmark endpoint has no published price.

## 4. The two GUI entries

The vcam GUI's Environment tiles list (`tools/vcam_gui.py`, through `tools/vcam_ws_bridge.py`)
holds the supported environment sources. Two of them are Mapbox:

| Entry | `environment_source_uri` | Look |
|-------|--------------------------|------|
| `mapbox` | `mapbox://buildings` | the theme's clay, like `osm` |
| `mapbox-original` | `mapbox://buildings?materials=original` | Mapbox's, like `google` |

- Pick one in the GUI, send `{"cmd":"set_environment_source","preset":"mapbox"}` to the bridge,
  or pass `--arm mapbox` to `tools/validate_logger_session.sh`.
- The node refuses the switch until its geo anchor is solved, as for every source.
- The GUI shows an entry as selected only when the node's `environment_source_uri` is exactly
  that entry's string. A hand-set URI such as `mapbox://buildings?radius=800` leaves the list as
  it was.
- Neither entry carries `cache=off`: the source never caches.

## 5. Attribution

Mapbox's Product Terms (1.4) require the Mapbox logo, "© Mapbox", "© OpenStreetMap" and "Improve
this map" wherever Mapbox content is shown. The output is a video frame, so the link texts are
drawn as text.

- **What:** the logo bottom-left at its native size, and "© Mapbox © OpenStreetMap Improve this
  map" right-aligned bottom-right, both 8 px from the frame edges. The text uses the theme's HUD
  text colour and scale. When the frame is too narrow for both on one line, the text moves up one
  line, above the logo: at HUD scale 1.0, below 725 px of output width. At HUD scale 1.0 the
  lifted line needs a frame at least 86 px tall (8 + 46 + 8 + 24 with the shipped logo); in a
  shorter frame it leaves through the top. Below 541 px at that scale (in general, 16 px plus
  524.76 px times the HUD scale) the text no longer fits on a line, and its end, "Improve this
  map", is cut off. At 1280 px that happens above HUD scale 2.4.
- **When:** all of these hold:
  - `environment_attribution` is true (the default; the element's disable knob);
  - the environment is visible: free look (mode 3) and `environment_enabled`;
  - `environment_source_uri` starts with `mapbox://`;
  - the source state is `STREAMING`.

  It does not depend on `hud_enabled`. In `STREAMING_FALLBACK` only baked chunks are on screen, so
  nothing is drawn.
- **Logo file:** `environment_attribution_logo_path`, read at configure. Empty, the default,
  means the installed `assets/attribution/mapbox-logo.png`; its provenance is in the
  `ATTRIBUTION.md` beside it. A missing or unreadable logo warns once, and the text is still
  drawn.
- Keep `environment_attribution` on wherever a person sees Mapbox content, at an output width
  and HUD scale that show the whole text. A screenshot or a recording that shows Mapbox content
  keeps the attribution in the frame.

## 6. Mapbox's terms as engineering rules

- **No caching.** The source has no disk cache, accepts only `cache=off`, and writes no file.
- **No Mapbox data in the tree.** Tests build their landmark and footprint fixtures in the test
  process, in Mapbox's formats. Responses fetched while debugging (tiles, TileJSON, response
  bodies) stay outside the repository. No pre-bake of `mapbox://` content exists or may be added,
  and a bake made through the bake script's Mapbox fallback must not be committed either. The one
  Mapbox file in the tree is the logo (§5), which the terms require on screen; the release check
  below covers it.
- **Attribution visible** whenever Mapbox content is (§5).
- **The token by name only** (§1).
- **Local until sign-off.** The `mapbox-environment` branch and its lane branches are not pushed to
  `origin` until the maintainer confirms Mapbox's sign-off. Before that push, run
  `tools/check_publish_leaks.sh` and the lint checks listed in `AGENTS.md`.
- **Logo and trademark.** Whether the logo file may stay in the tree is part of the release check
  ([`status.md`](../status.md), open items).

The bake script's Mapbox fallback ([`environment_bake.md`](environment_bake.md)) is a separate,
older path and is unchanged.

## 7. The demo

Both sites use the replay rig and the vcam GUI's free look. The rig owns the machine: starting it
stops any rig already running on the box (same process names, port 8765).

Before either site:

1. Build the library with cesium on, and the node, in the checkout you run from (`AGENTS.md`,
   "Build and test").
2. Export `MAPBOX_TOKEN` (§1) and run `tools/check_mapbox_token.sh` (§2).
3. Set `SESSION_DIR` to the logger session's directory
   ([`replay_logger_session.md`](replay_logger_session.md)).

### 7.1 Operating area

The session's own GPS anchor places the map at the operating area (25.0803, 55.3910).

```bash
tools/validate_logger_session.sh "$SESSION_DIR" --arm mapbox
```

- The anchor solves from the replayed GPS after 500 fixes and 20 m of travel. `--arm` retries the
  switch every 5 s for up to 120 s, then prints `[arm] OK: environment source = mapbox`.
- Expect extruded buildings only: Mapbox has no landmark models there.
- `--arm mapbox-original` shows Mapbox's look. The GUI list switches between the two live.

### 7.2 Downtown Dubai

A full geo-datum override moves the map to Downtown:

- `geo_datum_lat_deg`, `geo_datum_lon_deg` and `geo_datum_heading_deg` must all be set. A partial
  override is logged as an ERROR and ignored. The anchor then waits for GPS, nothing arms at
  activate (the node warns `no geo-anchor solved yet at on_activate()`), and no buildings show
  until a live switch (a GUI entry or `--arm`) after the GPS solve, which streams them at the
  operating area.
- The override pins where map (0, 0) is, and the compass bearing of map +x (90.0 means map +x
  points east). It does not move the robot: the replayed path keeps its map coordinates. The
  rehearsal keeps the session's map heading and picks the latitude and longitude so that the path
  runs on open ground near the Burj Khalifa.
- The table's values fit the rehearsed session. For another session, read its map heading from
  `--anchor-heading-deg` in the `geo-anchor solved:` line of a §7.1 run, and choose a latitude
  and longitude that move its route's centre onto open ground near the Burj Khalifa (the target
  minus the route centre's east and north offset).
- With all three set, the anchor is solved at configure, so the launch-time
  `environment_source_uri` arms at activate. No `--arm` is needed.
- `geo_datum_height_m` and `geo_anchor_height_offset_m` are optional. They do not move Mapbox
  buildings, which stand on map z = 0 at any anchor height.
- The parameters are doubles: write `90.0`, not `90`.

| Parameter | Value |
|-----------|-------|
| `geo_datum_lat_deg` | filled in by Task 17's rehearsal |
| `geo_datum_lon_deg` | filled in by Task 17's rehearsal |
| `geo_datum_heading_deg` | filled in by Task 17's rehearsal |

Set `DATUM_LAT`, `DATUM_LON` and `DATUM_HEADING` to the table's values, then:

```bash
tools/validate_logger_session.sh "$SESSION_DIR" -- \
    --param environment_source_uri:=mapbox://buildings \
    --param "geo_datum_lat_deg:=${DATUM_LAT}" \
    --param "geo_datum_lon_deg:=${DATUM_LON}" \
    --param "geo_datum_heading_deg:=${DATUM_HEADING}"
```

For Mapbox's look, quote the URI:
`--param 'environment_source_uri:=mapbox://buildings?materials=original'`.

The node log, `/tmp/overlume_validate/overlume_node.log`, shows a line containing
`geo-anchor solved (geo_datum override): …` at configure and a line containing
`environment source armed: 'mapbox://buildings` at activate.

Expect the Burj Khalifa and the other landmarks as models, with no doubled extrusion under them,
among streets of extruded buildings.

### 7.3 What to check

- The logo and the text are visible in mode 3 while the source streams, and gone in fallback.
- `mapbox-original` shows Mapbox's look, and `mapbox` the theme's.
- Steady-state `render_ms` (the `render_ms` status on `/overlume_node/diagnostics`), in clay mode,
  stays within 1.5 times the `osm` source's at the same pose.
- No token value in any log. Run both checks in a shell where §1's export is in effect: with
  `MAPBOX_TOKEN` empty, every line would count as a hit. The token is read from a pipe, never put
  on a command line. The first prints one count per rig log, and every count must be 0. The
  second scans the node's own ROS log files and must print nothing:

  ```bash
  grep -cF -f <(printf '%s' "$MAPBOX_TOKEN" | tr -d '[:space:]') /tmp/overlume_validate/*.log
  grep -rcF -f <(printf '%s' "$MAPBOX_TOKEN" | tr -d '[:space:]') \
      "${ROS_LOG_DIR:-${ROS_HOME:-$HOME/.ros}/log}" | grep -v ':0$'
  ```

## 8. Troubleshooting

- **The switch is refused with `failed to open '…' -- previous environment source left intact`,
  and the log names `MAPBOX_TOKEN`.** The node's environment has no token. Export it (§1) and
  restart the rig.
- **The log says `Mapbox rejected MAPBOX_TOKEN (HTTP 401)` and no buildings appear.** Mapbox
  rejects the token. Run `tools/check_mapbox_token.sh` and replace the token. The 401s count as
  failed requests, so eight in a row switch the source to its fallback (below).
- **No buildings over the sea.** A footprint tile over open water holds only a `water` layer, so
  there is nothing to extrude. Nothing is wrong.
- **Extrusions, but no landmark models.** Mapbox serves landmark models in Downtown Dubai and
  Dubai Marina. None were found in the Abu Dhabi samples or around the operating area: there the
  landmark tile answers 204 and only extrusions show.
- **No landmark models in Downtown, and one log line names a landmark tile.** The landmark tile
  could not be read. Its endpoint (`3dtiles/v1`) is undocumented and can change without notice,
  for example to a new compression codec. Extrusions still show. Run the opt-in live test, which
  prints tile, landmark and extrusion counts, and report what it prints:
  `OVERLUME_LIVE_MAPBOX=1 overlume/build/test_mapbox_buildings --gtest_filter='MapboxLive.*'`.
- **The buildings, the logo and the text disappear, and the node warns
  `environment source: network loss detected -- switched to fallback dir '…'`** (or
  `... but none configured -- environment now empty`). Eight requests in a row failed: network
  loss, a Mapbox-side 429/5xx burst, or 401 on everything. The state is `STREAMING_FALLBACK`, and the baked chunks from
  `fallback=` (the node's `environment_chunks_dir`) show instead. The fallback lasts for the
  source's life. Fix the cause, then select the source again: pick another GUI entry and then the
  Mapbox one, rerun `--arm`, or run
  `ros2 param set /overlume_node environment_source_uri mapbox://buildings`.
- **A tile answers 429 or 5xx, or a request fails in transport.** A partial failure is retried, at
  most 3 attempts. After that the tile shows what loaded, and one line is logged. A whole tile
  answering 429 or 5xx (17 requests) trips the network-loss fallback before any retry; see the
  "buildings disappear" entry above, and select the source again.
- **A landmark doubled by an extrusion, or a missing neighbour, at a landmark's edge.**
  Extrusions under a landmark are removed by their footprint's centroid, so a part whose centroid
  lies outside the landmark's footprint stays. Known limit.
- **A wall missing along a straight north-south or east-west line.** Walls along a z16 tile edge
  are left out, so that buildings cut by tile edges meet without doubled walls. A real wall lying
  exactly on such an edge (tile coordinate 0 or 4096) is left out too. Both demo sites' street
  grids are rotated, so this is rare there. Known limit.
- **Buildings float or sink on a slope.** The source has no terrain following: buildings stand on
  the flat map plane. Known limit until imagery on terrain lands (design §13).
- **The GUI list does not show `mapbox` while Mapbox streams.** The node's
  `environment_source_uri` is not exactly an entry's string (a hand-set key, for example). Pick
  the entry to set the exact string.
