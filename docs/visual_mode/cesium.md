# Cesium ion runbook (VM-060)

How a **new deployment** (a second robot/site with no access to this box)
registers with Cesium ion, picks a tileset for its operating area, and proves
the token actually works — before Epic 6's `StreamingEnvironmentSource`
(VM-062) ever tries to use it. Covers only the account/token/smoke-check
contract; the streaming source itself, its `source_uri` scheme, and its
config knobs are VM-062/VM-063 (pointers below).

## 1. Account + token

1. Create an account at [ion.cesium.com](https://ion.cesium.com) (or sign in
   with an existing one).
2. Go to **Access Tokens** → **Create token**.
3. Scope: **`assets:read`** only — that is all the renderer ever uses.
   (Only add `assets:list`/`assets:write` if you are going through the
   upload-and-clip path in section 2 below; the runtime token itself still
   only needs `assets:read`.)
4. Copy the generated token once — ion does not show it again.
5. On the deploy box, add it to the deploy user's `~/.bashrc` (or whatever
   sources the node's launch environment):
   ```bash
   export CESIUM_ION_TOKEN=<paste the token here>
   ```
   then `source ~/.bashrc` (or open a new shell) so it is live before running
   the smoke check in section 4.
6. **Never commit it, never echo it into a file, log, or `ros2 param dump`.**
   Same rule as `MAPBOX_TOKEN` (Epic 4 Decision 8) — env var, by name only,
   nowhere else.

## 2. Choosing the tileset for the operating area

Three supported paths:

- **Path A (default): Cesium OSM Buildings, ion curated asset `96188`.**
  Global coverage (includes the Dubai/Sharjah operating area), needs no
  upload. In the ion dashboard: **Asset Depot** → find "Cesium OSM
  Buildings" → **Add to my assets**. Done — asset `96188` is now on this
  account. This is the shipped default the node param expects
  (`ion://96188`, VM-063).
- **Path B (custom): upload and clip your own tileset.** Smaller,
  cache-friendlier tiles at the cost of a per-deployment upload step. In the
  ion dashboard: **My Assets** → **Add data**, upload/clip the area of
  interest, note the numeric asset id ion assigns it. That id is what
  replaces `96188` in the `ion://<assetId>` source URI.
- **Path C (VM-064): Google Photorealistic 3D Tiles, ion asset `2275207`,
  original-materials mode.** See section 6 below.

Record whichever `<assetId>` this deployment ends up using — it is the value
the node's `source_uri` param takes (`ion://<assetId>`, VM-063).

**Path B + the vcam GUI's "clipped" preset (VM-096):** the GUI/WS bridge's
Environment Tiles source select offers "clipped" as a one-click switch to
THIS deployment's own Path B asset, but it never guesses or fabricates an
id — set `environment_own_asset_uri` (`config/default_params.yaml`) to
`ion://<assetId>` with the id recorded above once this deployment has
uploaded one. Until then, "" (the shipped default) means "no own asset
yet" and the GUI greys that option out with a tooltip pointing back here.

## 6. Google Photorealistic 3D Tiles (VM-064)

A textured, photorealistic mesh, not clay — `default_params.yaml`'s
`environment_source_uri` preset for it is
`ion://2275207?materials=original&cache=off` (commented out by default;
uncomment to switch). This section covers the three things that make it a
different beast from Path A/B's clay tilesets.

- **Asset id, verified, not guessed.** `2275207` was confirmed at VM-064
  implementation (2026-09-16) directly against this account's live ion API
  — `GET /v1/assets/2275207` returned HTTP 200 with `type: 3DTILES` and a
  `name` field matching "Google", "Photorealistic", and "3D Tiles" (checked
  as redacted boolean substring matches, per this repo's standing rule
  never to print an ion response body — see section 3/4's own "never
  prints ... any response body" discipline, which extends to this
  verification too). `GET /v1/assets/2275207/endpoint` also returned HTTP
  200 (this account already has access; no separate "Add to my assets"
  step was needed for this asset, unlike Path A's `96188`) — note its
  response shape differs from the flat `url`/`accessToken` pair
  `check_cesium_token.sh` parses for a Cesium-hosted asset (it carries
  `externalType`/`options` instead, no top-level `accessToken`): this is
  why `check_cesium_token.sh 2275207` itself reports a parse FAIL even
  though the endpoint accepted the token — that script only proves the
  Cesium-hosted `assets:read` path (Path A/B); the renderer's own ingestion
  never uses that script's flat-field parse at all, it hands the asset id
  + token straight to cesium-native's `Tileset` ion constructor (Decision
  15.6), which already knows how to route an `externalType` asset. Treat
  `check_cesium_token.sh`'s FAIL on this one asset id as expected, not a
  regression — PASS still means what it always meant for `96188`/a Path B
  upload.
- **Attribution (Google Map Tiles terms).** Must be visible wherever tiles
  are displayed. The node draws a static line ("3D Tiles data (c) Google")
  through the existing HUD text primitive (`environment_attribution` param,
  on by default, drawn only while `materials=original` is active) — the
  exact legal wording is NOT re-verified per deployment by this code path
  (no ion credit text is parsed or rendered dynamically); before a real
  go-live, confirm the current required wording against Google's Platform
  Terms / the ion asset's own listed attribution and update the string in
  `visualization_node.cpp` if it has changed. If the HUD text machinery
  cannot carry a line for some deployment (font unavailable, etc. — see
  `hud_font_path`), the manual fallback is a physical/on-screen overlay
  sticker or a fixed compositing step downstream of this node; that gap
  would show up as the node's own one-shot WARN
  (`environment_attribution_warned_`).
- **Cache terms.** The shipped preset ships `cache=off` (VM-063's existing
  `?cache=` knob, given the literal value `off` rather than a directory —
  Decision 14 / Task 5 Step 2(b)) until this deployment has verified
  Google's current Map Tiles cache-lifetime policy allows the on-disk
  SqliteCache's retention. `cache=off` means every tile request goes
  straight through the network accessor, nothing persisted to
  `mpviz-tile-cache` — a real compliance lever, not a placeholder; verify
  the policy, then switch to a real `?cache=<dir>` (or drop the key for the
  library default) once confirmed compliant.
- **No golden ships for this mode.** A committed Google-tile fixture would
  itself need to clear the same redistribution-terms question as the cache
  policy above; this task does not fetch or commit any Google tile bytes.
  Automated coverage is the parse (`materials=original` recognized) and the
  no-remap behavior (both fixture-free, exercised via the existing OSM
  fixture with the test-only `materials_original` hook param) — the visual
  check against real Google content is the validation rig's live-token,
  live-network human step, same as this runbook's own smoke check.
- **Perf.** Expect a materially heavier `render_ms` delta than either clay
  preset (textured photoreal vs. untextured extrusions) — record it in the
  epic's results block the same way as the OSM-clay preset's own number;
  `maximumScreenSpaceError`/load-radius tuning are the named first levers
  on a miss.

## 3. The env var contract

- Exact name: **`CESIUM_ION_TOKEN`**. No alternate spelling, no config-file
  fallback.
- Read by the renderer library at `set_environment_source()` time via
  `getenv("CESIUM_ION_TOKEN")` — once, at open.
- Missing or empty → the environment layer fails to open with one WARN;
  nothing else in the node is affected (spec §9, "missing data renders
  nothing" — same non-fatal shape as a missing bake directory).
- Token rotation: replace the env var and restart the node. Nothing else
  (no cache file, no param) stores the token.

## 4. Smoke check

Run:

```bash
cuda/src/libs/visual_renderer/scripts/check_cesium_token.sh [assetId]
```

(`assetId` defaults to `96188`.) This proves the token reaches the same ion
tileset endpoint the renderer will use — it does **not** touch the renderer
or any node process.

- **PASS** means BOTH stages returned HTTP 200: the `/v1/assets/<id>/endpoint`
  handshake, and the fetch of the `tileset.json` URL that handshake
  returned, using its short-lived session token. Endpoint-200 alone is not
  the AC — "token retrieves `tileset.json`" is checked literally, not by
  proxy.
- Exit codes: **0 = pass**, **1 = fail**, **2 = skip** (`CESIUM_ION_TOKEN`
  unset) — distinct so a wrapper or CI step can tell "broken" from
  "not configured".
- **FAIL, endpoint stage, HTTP 401**: bad or expired token — regenerate it
  in section 1 and re-export.
- **FAIL, endpoint stage, HTTP 404**: the account has not added this asset
  yet — go do section 2's "Add to my assets" step, then re-run.
- **FAIL, tileset stage**: the endpoint accepted the token and handed back a
  session token, but the tileset URL itself rejected or could not route that
  session token — this is an ion-side/network issue, not a token-typo issue;
  re-run once, then treat as a real outage if it repeats.
- The script never prints the token, the session token, or any response
  body — only PASS/FAIL and HTTP status codes.

## 5. What this does NOT cover

- **Baking the fallback/offline environment** — that is the separate baked
  pipeline (`bake_environment.py`), documented in
  [`environment_bake.md`](environment_bake.md) (Epic 4 / VM-042).
- **Disk tile cache and baked-fallback params** (`?cache=`, `&fallback=`,
  `&max_cache_items=` on the `ion://` source URI, and their defaults in
  `default_params.yaml`) — those are VM-063's param table, not this runbook;
  this runbook stops at "the token works against the chosen asset".
