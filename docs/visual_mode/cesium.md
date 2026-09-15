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

Two supported paths (a third — Google Photorealistic 3D Tiles — ships with
VM-064; not covered here):

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

Record whichever `<assetId>` this deployment ends up using — it is the value
the node's `source_uri` param takes (`ion://<assetId>`, VM-063; not
implemented as of this runbook).

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
