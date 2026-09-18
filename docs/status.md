# Status

The one status ledger for Overlume — replaces the plan-embedded ledgers'
summary role and both prior backlog documents
(`docs/plans/archive/visual_mode_project_backlog.md`, an Azure DevOps export,
and the design spec's own backlog table). The plans themselves keep their
own per-task ledgers as the detailed record; this page is the roll-up.

## Shipped

| Item | Date | Plan |
|---|---|---|
| Epic 0 — Contract spike (Filament hello-frame, mode mux, vcam parity, GPU budget) | commits `cc900c1`…`5b479bd`, 2026-08-18 (range sourced from [`plans/archive/visual_mode_project_backlog.md`](plans/archive/visual_mode_project_backlog.md), not the Epic 0 plan itself, which cites neither hash; a second live source, [`plans/2026-08-18-visual-mode-epic1.md`](plans/2026-08-18-visual-mode-epic1.md), gives a conflicting start hash `fac1535` for the same epic's prerequisite — both endpoints resolve in git and are dated 2026-08-18); GPU-budget item (VM-043) closed 2026-09-11 on a dev-box proxy | [`plans/2026-08-18-visual-mode.md`](plans/2026-08-18-visual-mode.md) |
| Epic 1 — Core scene & dark theme | CLOSED 2026-08-20 at `bd5e11e` | [`plans/2026-08-18-visual-mode-epic1.md`](plans/2026-08-18-visual-mode-epic1.md) |
| Epic 2 — Autonomy data ingestion | CLOSED 2026-09-07 at `c5ea38c` (gate PASSED at `26f17f0`; the plan records the gate date as 2026-08-20 but that commit is dated 2026-09-07 in git — a historical-document discrepancy, left uncorrected per this restructure's rule) | [`plans/2026-08-18-visual-mode-epic2.md`](plans/2026-08-18-visual-mode-epic2.md) |
| Epic 3 — HUD, polish & controls | CLOSED 2026-09-09 at `af628df` | [`plans/2026-08-18-visual-mode-epic3.md`](plans/2026-08-18-visual-mode-epic3.md) |
| VM-077 — new-stack rendering / flicker root-cause fix (Epic 3 follow-on) | CLOSED 2026-09-10, user-approved ("Flicker gone!") | [`plans/2026-09-09-vm077-new-stack-rendering.md`](plans/2026-09-09-vm077-new-stack-rendering.md) |
| Epic 4 — Clay buildings (EnvironmentLayer), Tasks 1-3 | Done 2026-09-10 | [`plans/2026-08-18-visual-mode-epic4.md`](plans/2026-08-18-visual-mode-epic4.md) (ambiguous source: the plan's own opening line still reads "Status: NOT STARTED", stale against its own Status ledger table showing all 3 tasks Done — a historical document, left uncorrected per this restructure's rule) |
| Epic 5 — Hardening & delivery | No standalone plan was ever authored; its items were distributed in parallel with the unified-engine migration (user decision 2026-09-11): VM-040 (quality governor) and VM-044 (asset packaging) done 2026-09-11; VM-041 (perf benchmark + `ci_visual_mode.sh`) and VM-042 (docs — the runbooks this restructure moved) pulled forward earlier; VM-043 (live validation sign-off) superseded by the migration's own Task 6 parity sign-off | [`plans/2026-08-18-visual-mode.md`](plans/2026-08-18-visual-mode.md)'s own Epic 5 section |
| Unified-engine migration (VM-090…095) — CUDA→Filament cutover, mux and `micropilot_rendering_node` retired | CLOSED 2026-09-11, all 6 tasks Done | [`plans/2026-09-10-unified-engine-migration.md`](plans/2026-09-10-unified-engine-migration.md) |
| Epic 6 — v1.1: Cesium 3D Tiles streaming (VM-060…064) + post-close tail | CLOSED 2026-09-16, final cross-cutting review 2026-09-17 | [`plans/2026-08-18-visual-mode-epic6.md`](plans/2026-08-18-visual-mode-epic6.md) |
| Restructure Task 0 — LFS + delete legacy code/agent-config sprawl | ☑ 2026-09-17 — `5247be6`, `82a7214`, `f47f38b` (LFS pointers) | [`plans/2026-09-17-overlume-restructure.md`](plans/2026-09-17-overlume-restructure.md) |
| Restructure Task 1 — move `overlume/`/`ros/`, fix hard-coded paths | ☑ 2026-09-17 (30de849) | [`plans/2026-09-17-overlume-restructure.md`](plans/2026-09-17-overlume-restructure.md) |
| Restructure Task 2 — identifier rename (`mpviz`→`overlume`, `MPVIZ_`→`OVERLUME_`, node→`overlume_ros`/`overlume_node`) | ☑ 2026-09-17 (9ceda6e) | [`plans/2026-09-17-overlume-restructure.md`](plans/2026-09-17-overlume-restructure.md) |
| Restructure Task 3 — docs restructure, this status ledger, root README | 2026-09-17 (1ca23e3) | [`plans/2026-09-17-overlume-restructure.md`](plans/2026-09-17-overlume-restructure.md) |
| Restructure Task 4 — `examples/` against the public API + gate stage 6 | 2026-09-17 (dd59435) | [`plans/2026-09-17-overlume-restructure.md`](plans/2026-09-17-overlume-restructure.md) |
| Restructure Task 5 — Doxygen `docs` target, documented public headers, `version.h` | 2026-09-17 (99c8d29) | [`plans/2026-09-17-overlume-restructure.md`](plans/2026-09-17-overlume-restructure.md) |
| Restructure Task 6 — LICENSE/NOTICE/SPDX, CONTRIBUTING, CoC, SECURITY, CHANGELOG, `.github` CI + release, one repo-wide clang-format | 2026-09-17 (2b96cc7, 790aa0a) | [`plans/2026-09-17-overlume-restructure.md`](plans/2026-09-17-overlume-restructure.md) |
| Restructure Task 7 — graphify snapshot prune, vendored skill refresh, non-strict read hook, hooks documented in AGENTS.md, memory refresh | 2026-09-17 (adf901d) | [`plans/2026-09-17-overlume-restructure.md`](plans/2026-09-17-overlume-restructure.md) |
| Restructure Task 8 — final cross-cutting review (50 findings fixed) + clean-worktree build; plan CLOSED | 2026-09-17 (see git log: `chore(restructure): Task 8`) | [`plans/2026-09-17-overlume-restructure.md`](plans/2026-09-17-overlume-restructure.md) |

## Open items

From Epic 6's "Post-close tail + final review" section
(`plans/2026-08-18-visual-mode-epic6.md`), copied faithfully:

1. **PERFORMED 2026-09-18 on the live rig (replayed real-robot session, streamed OSM tiles resident, network cut by killing a local proxy the node's curl was tunnelled through): the node kept rendering its resident tiles at 30 Hz, no crash, no orphaned geometry — and NO fallback fired in 6.5 min, because the trigger counts failed requests and this short looping route never needed a tile it did not already hold; with the tile cache warm, even a cache-on cut is invisible. Second scenario, network dead at ARM time: one failed handshake, no retries, so the 8-consecutive-failure threshold is never reached and the environment simply stays empty — recorded as item 18. The cable pull itself is therefore done; the fallback TRANSITION with tiles resident remains observable only on a route that outruns its loaded tiles.** Original item: Live-rig cable pull** (Task 4 Step 3) — never run. Still the only
   coverage for fallback-from-live-STREAMING with tiles resident (teardown
   discipline). Owner: first on-robot session with the token in the
   environment.
2. CLOSED 2026-09-18 (this commit, maintainer decision): `on_activate()`
   now arms the configured source regardless of `environment_enabled`,
   applying visibility via `set_environment_visible()` afterward — a
   deployment launched disabled is recovered by the GUI Switch alone, no
   preset re-pick needed.** Original item: Arm-hidden at launch is a product
   call. `on_activate()` arms nothing when `environment_enabled:=false`; a
   deployment launched disabled cannot be recovered by the GUI Switch alone
   without also picking a preset.
3. **CLOSED 2026-09-18 (user decision): default is now ON; hosted CI and the docs job pass OFF explicitly to stay token- and network-free.** Original item: `OVERLUME_ENABLE_CESIUM` defaults OFF (`overlume/CMakeLists.txt`) — a
   fresh `build/` configure yields a node with no streaming backend, so the
   GUI's osm/google/clipped presets fail to open there; CI green depends on
   the existing cache carrying it ON. Decide default-ON, or make
   `ci_visual_mode.sh` pass the flag before the next clean-checkout build.
4. **OPEN — needs a design (per-tile geodetic correction vs the baked flat-plane convention), not a tweak; not attempted in the 2026-09-18 closing pass.** Streamed-vs-baked vertical sag (~d²/2R, 0.54 m at 2.6 km) that no
   constant offset cancels — no follow-up task named beyond "Task 4/follow-up".
5. **CLOSED 2026-09-18 — measured on the live node (286 sqlite3_, 18 curl_easy_, 536 SSL_, 913 EVP_ symbols exported while the process also mapped the system copies); fixed with a linker version script that hides only the vendored C libraries (a first attempt with --exclude-libs,ALL also hid libc++abi's __cxa_throw and aborted the node on its first tf2 exception — caught by the gate); test_exported_symbols checks both halves.** Original item: C-library interposition sub-check** (sqlite3/curl/openssl/zlib vs a
   real running node's loaded set) deferred at VM-061 for lack of a node
   binary; the binary exists now, the check was never run.
6. **OPEN — needs a controlled multi-tile perf measurement on the live rig; not attempted in the 2026-09-18 closing pass.** `kMaxTileCreatesPerTick` still a named knob only; Google-preset perf
   unconfirmed beyond a single tile.
7. **OPEN — maintainer legal check, external to the code.** Google pre-go-live manual checks: exact attribution wording, current
   Map Tiles cache-lifetime policy (`cache=off` ships until verified).
8. **OPEN — deployment config (`environment_own_asset_uri`), nothing to change in the repo.** `clipped` preset ungraded — no `environment_own_asset_uri` configured
   on this deployment.
9. CLOSED 2026-09-18 (this commit, maintainer decision):
   `environment_effectively_visible(RenderMode, bool)`
   (`ros/src/overlume_ros/include/overlume_ros/scene_assembly.hpp`) gates
   buildings to FREE_LOOK (VISUAL) only — BOWL/HYBRID always hide them now,
   regardless of `environment_enabled`.** Original item: Per-`render_mode`
   auto-gating of buildings (`docs/runbooks/signoff.md` exception 7's
   remaining half).
10. **OPEN — theme tuning judged by eye; deferred until a reference-free target is agreed.** Light theme: roof-to-wall contrast 6.2L vs. the reference's ~14L
    (suspected ACES shoulder compression); grid/distance-fade invisible on
    empty ground. Dark theme: `EmptyWorld_DarkAdas` ground-vs-sky guard
    margin ~1.2 levels — re-measure before touching sky/ground/sun.

This restructure's own recorded follow-ups (from its plan's Status ledger):

11. **CLOSED 2026-09-18** — merged into `overlume::ros` (survey found zero symbol collisions; `overlume::ros::testing` for the fixtures). Original item: **Two node namespaces.** The node keeps `overlume_node` (31 files, the
    former `visualization_node`) and `overlume::ros` (the former
    `micropilot::visualization_app`); test fixtures live in
    `overlume_node::testing`. Merging them is a real refactor (a
    name-collision check across 31 files), owed to Task 8's review or a
    follow-up, not a mechanical rename.
12. **CLOSED 2026-09-17 (Task 8: clean worktree, `git lfs pull` bytes identical, README quick start followed verbatim, gate at the frozen counts).** Original item: Clean-clone check owed to Task 8. Task 1's own gate substituted
    cache-provenance verification (a fresh CMakeCache carries no old paths;
    all three consumers agree) for a true fresh-clone build; a real fresh
    `git clone` + `git lfs pull` + clean build is still owed.

(Item 3 above and this restructure's own recorded "`OVERLUME_ENABLE_CESIUM`
default OFF" follow-up are the same fact, restated post-rename — not two
separate opens.)

13. CLOSED 2026-09-17 — all four workflows green on the first push and on every push since; still true that the hosted build depends on apt.llvm.org keeping the pinned LLVM 18.1.8 debs. The four GitHub workflows (`lint`, `build`, `docs`, `release`) all ran green on 2026-09-17 (first push, `v0.1.0` release created, Pages live at <https://amerghazal7.github.io/overlume/>); the hosted build depends on apt.llvm.org's jammy pool keeping the pinned LLVM 18.1.8 `.deb`s (re-pin `LLVM_PKG_VERSION` in `overlume/scripts/setup_toolchain_cesium.sh` when it prunes).
14. CLOSED 2026-09-18 — examples call set_object_model_dir() (third argv, default overlume/assets/models); CAR/TRUCK_VAN/PEDESTRIAN load, BUS/CYCLIST still fall back to the clay box because no CC0 model ships for them (asset gap, see overlume/assets/models/ATTRIBUTION.md). `examples/` pass no model-assets directory, so tracked objects render as the procedural clay-box fallback (one WARN per class); exposing the models dir the way the theme dir is would fix it.
15. CLOSED 2026-09-18 — both purges done (filter-repo, force-pushed); the GitHub Support request for unreachable/LFS objects is drafted for the maintainer to file.** Original item: The two vendor HMI reference images the themes were authored against were removed from the tree on 2026-09-17 (third-party captures, no redistribution basis); purged from git/LFS history on 2026-09-17 via git filter-repo; the real Cesium ion tile fixtures (22 .b3dm + 2 tileset.json) were replaced by payloads generated by `overlume/scripts/make_tile_fixture.py` and purged the same way on 2026-09-18. A GitHub Support request for the unreachable/LFS objects is drafted for the maintainer.
16. DOCUMENTED (runbook `replay_logger_session.md` §1 and CONTRIBUTING) — maintainer-machine quirk, fresh clones unaffected; no code change. `git lfs install` collides with graphify's `post-checkout` hook on the maintainer's machine (append the LFS lines by hand); fresh clones are unaffected.

17. **Data-collector gaps found on the first real-robot replay (2026-09-18, external to this repo):** the logger records `/tf_static` from only the first latched publisher, so URDF statics are missing on replay; the recorded topic set omitted every `nav_msgs/Path`, the dynamic objects, OGMs and the speed feed (see `runbooks/replay_logger_session.md` §5). The `/perception/gradient_ogm` profile row no longer matches any stack publisher and should be retired or re-pointed.

18. **Fallback trigger is demand-driven and needs 8 consecutive failed requests (found 2026-09-18 on the live rig).** A network cut while all needed tiles are resident produces no failed request, so streaming never falls back (acceptable: nothing disappears); a network that is already dead when the source is armed produces one failed ion handshake and no retry, so the threshold is never reached and the environment stays empty with no WARN. Proposed fix: treat a failed handshake / root tileset.json as terminal (fall back immediately), and optionally add a time-based probe (no successful request within T seconds of arming → fall back). Needs a library change + fixture tests; not done in the closing pass.

## Known gaps / accepted exceptions

From `docs/runbooks/signoff.md`'s "Named exceptions the sign-off explicitly
accepts" list:

1. [`exposure_match`/`fill_blind_zone` shipped forced-off](runbooks/signoff.md#named-exceptions-the-sign-off-explicitly-accepts-not-parity-gaps-to-close) — visible seam-brightness steps and an empty near-field ring are expected, not a regression.
2. [Mode 2's lidar colorization is first-match, not feather-weighted](runbooks/signoff.md#named-exceptions-the-sign-off-explicitly-accepts-not-parity-gaps-to-close) — visible seams at camera-boundary points are expected.
3. [The robot overlay's asset changes shape](runbooks/signoff.md#named-exceptions-the-sign-off-explicitly-accepts-not-parity-gaps-to-close) — without a provisioned M02P glTF, the ego overlay degrades to a plain box on the real robot.
4. [`bowl.mat` contributes at most 2 cameras per fragment](runbooks/signoff.md#named-exceptions-the-sign-off-explicitly-accepts-not-parity-gaps-to-close) — a mesh-aligned faceted seam across triple-camera-overlap bands.
5. [The Filament bowl mesh terminates at `r = bowl_Rmax`](runbooks/signoff.md#named-exceptions-the-sign-off-explicitly-accepts-not-parity-gaps-to-close) — beyond it, sky color instead of the CUDA reference's clamped-height cap.
6. [`rendering_node`'s `/rendering/image` publish rate fell short of 30 Hz under real full-sensor load](runbooks/signoff.md#named-exceptions-the-sign-off-explicitly-accepts-not-parity-gaps-to-close) — a pre-existing CUDA-node property, mooted by the cutover.
7. [Environment/buildings are not gated per `render_mode`](runbooks/signoff.md#named-exceptions-the-sign-off-explicitly-accepts-not-parity-gaps-to-close) — the visibility toggle is operator-driven (`set_environment_visible()`), not auto-gated; see open item 9 above.
8. [Lidar colorization samples cameras through base extrinsics; the bowl uses ego-motion-compensated extrinsics](runbooks/signoff.md#named-exceptions-the-sign-off-explicitly-accepts-not-parity-gaps-to-close) — latent (not exercised) against every fixture bag captured so far, none of which carries odometry.

## How to update this file

- **Shipped** grows by one line per epic/task closed, sourced from that
  plan's own Status ledger (date + commit hash) — never invent a date or
  hash; if the plan's own ledger is ambiguous or stale, say so here rather
  than smoothing it over (see the Epic 4 row above for the pattern).
- **Open items** carries forward until whoever owns it closes it in the
  plan that opened it; remove a line only when that plan's ledger (or a new
  plan) records it closed, and copy the closure note here.
- **Known gaps** mirrors `docs/runbooks/signoff.md`'s own exception list —
  update both together; don't let this list drift from that one.
- This file replaces the two prior backlog documents; do not resurrect a
  third status surface — new status goes here, not into a plan's own ledger
  table (those track task-execution steps, not project status) and not into
  chat or memory.
