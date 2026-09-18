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
| Restructure Task 0 — LFS + delete legacy code/agent-config sprawl | ☑ 2026-09-17 — `2c54d12`, `65a6a3f`, `ef83764` (LFS pointers) | [`plans/2026-09-17-overlume-restructure.md`](plans/2026-09-17-overlume-restructure.md) |
| Restructure Task 1 — move `overlume/`/`ros/`, fix hard-coded paths | ☑ 2026-09-17 (fc99796) | [`plans/2026-09-17-overlume-restructure.md`](plans/2026-09-17-overlume-restructure.md) |
| Restructure Task 2 — identifier rename (`mpviz`→`overlume`, `MPVIZ_`→`OVERLUME_`, node→`overlume_ros`/`overlume_node`) | ☑ 2026-09-17 (71b10c4) | [`plans/2026-09-17-overlume-restructure.md`](plans/2026-09-17-overlume-restructure.md) |
| Restructure Task 3 — docs restructure, this status ledger, root README | 2026-09-17 (041f056) | [`plans/2026-09-17-overlume-restructure.md`](plans/2026-09-17-overlume-restructure.md) |
| Restructure Task 4 — `examples/` against the public API + gate stage 6 | 2026-09-17 (91a211a) | [`plans/2026-09-17-overlume-restructure.md`](plans/2026-09-17-overlume-restructure.md) |
| Restructure Task 5 — Doxygen `docs` target, documented public headers, `version.h` | 2026-09-17 (e1c48a2) | [`plans/2026-09-17-overlume-restructure.md`](plans/2026-09-17-overlume-restructure.md) |
| Restructure Task 6 — LICENSE/NOTICE/SPDX, CONTRIBUTING, CoC, SECURITY, CHANGELOG, `.github` CI + release, one repo-wide clang-format | 2026-09-17 (7db1ebb, 623d088) | [`plans/2026-09-17-overlume-restructure.md`](plans/2026-09-17-overlume-restructure.md) |
| Restructure Task 7 — graphify snapshot prune, vendored skill refresh, non-strict read hook, hooks documented in AGENTS.md, memory refresh | 2026-09-17 (2f8b732) | [`plans/2026-09-17-overlume-restructure.md`](plans/2026-09-17-overlume-restructure.md) |
| Restructure Task 8 — final cross-cutting review (50 findings fixed) + clean-worktree build; plan CLOSED | 2026-09-17 (see git log: `chore(restructure): Task 8`) | [`plans/2026-09-17-overlume-restructure.md`](plans/2026-09-17-overlume-restructure.md) |

## Open items

From Epic 6's "Post-close tail + final review" section
(`plans/2026-08-18-visual-mode-epic6.md`), copied faithfully:

1. **Live-rig cable pull** (Task 4 Step 3) — never run. Still the only
   coverage for fallback-from-live-STREAMING with tiles resident (teardown
   discipline). Owner: first on-robot session with the token in the
   environment.
2. **Arm-hidden at launch is a product call.** `on_activate()` arms nothing
   when `environment_enabled:=false`; a deployment launched disabled cannot
   be recovered by the GUI Switch alone without also picking a preset.
3. **`OVERLUME_ENABLE_CESIUM` defaults OFF** (`overlume/CMakeLists.txt`) — a
   fresh `build/` configure yields a node with no streaming backend, so the
   GUI's osm/google/clipped presets fail to open there; CI green depends on
   the existing cache carrying it ON. Decide default-ON, or make
   `ci_visual_mode.sh` pass the flag before the next clean-checkout build.
4. **Streamed-vs-baked vertical sag** (~d²/2R, 0.54 m at 2.6 km) that no
   constant offset cancels — no follow-up task named beyond "Task 4/follow-up".
5. **C-library interposition sub-check** (sqlite3/curl/openssl/zlib vs a
   real running node's loaded set) deferred at VM-061 for lack of a node
   binary; the binary exists now, the check was never run.
6. **`kMaxTileCreatesPerTick`** still a named knob only; Google-preset perf
   unconfirmed beyond a single tile.
7. **Google pre-go-live manual checks**: exact attribution wording, current
   Map Tiles cache-lifetime policy (`cache=off` ships until verified).
8. **`clipped` preset** ungraded — no `environment_own_asset_uri` configured
   on this deployment.
9. **Per-`render_mode` auto-gating of buildings**
   (`docs/runbooks/signoff.md` exception 7's remaining half).
10. **Light theme**: roof-to-wall contrast 6.2L vs. the reference's ~14L
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
12. **Clean-clone check owed to Task 8.** Task 1's own gate substituted
    cache-provenance verification (a fresh CMakeCache carries no old paths;
    all three consumers agree) for a true fresh-clone build; a real fresh
    `git clone` + `git lfs pull` + clean build is still owed.

(Item 3 above and this restructure's own recorded "`OVERLUME_ENABLE_CESIUM`
default OFF" follow-up are the same fact, restated post-rename — not two
separate opens.)

13. The four GitHub workflows (`lint`, `build`, `docs`, `release`) all ran green on 2026-09-17 (first push, `v0.1.0` release created, Pages live at <https://amerghazal7.github.io/overlume/>); the hosted build depends on apt.llvm.org's jammy pool keeping the pinned LLVM 18.1.8 `.deb`s (re-pin `LLVM_PKG_VERSION` in `overlume/scripts/setup_toolchain_cesium.sh` when it prunes).
14. `examples/` pass no model-assets directory, so tracked objects render as the procedural clay-box fallback (one WARN per class); exposing the models dir the way the theme dir is would fix it.
15. The two vendor HMI reference images the themes were authored against were removed from the tree on 2026-09-17 (third-party captures, no redistribution basis); purged from git/LFS history on 2026-09-17 via git filter-repo; the real Cesium ion tile fixtures (22 .b3dm + 2 tileset.json) were replaced by payloads generated by `overlume/scripts/make_tile_fixture.py` and purged the same way on 2026-09-18. A GitHub Support request for the unreachable/LFS objects is drafted for the maintainer.
16. `git lfs install` collides with graphify's `post-checkout` hook on the maintainer's machine (append the LFS lines by hand); fresh clones are unaffected.

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
