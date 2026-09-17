# Docs index

Overlume's documentation, restructured 2026-09-17
(`docs/plans/2026-09-17-overlume-restructure.md`). This page is the one
index; `docs/status.md` is the one status ledger.

## Folders

- **`runbooks/`** — operational how-tos for a person running or deploying
  the node: profile authoring, environment baking, Cesium ion setup, the
  CI gate, the parity goldens, and the sign-off ledger.
- **`design/`** — the visual-mode design spec and its backlog, the design
  record Epics 0-6 were built against.
- **`adr/`** — architecture decision records (unchanged by this restructure).
- **`plans/`** — the live restructure plan, the per-epic implementation
  plans (history of *how* each epic was built), and `plans/archive/` for
  retired-prototype plans/specs and the superseded project backlog export.
- **`evidence/`** — captured measurement artifacts (LFS) cited by the plans.
- **`assets/`** — reference images cited by docs and the root README.

## Runbooks (`docs/runbooks/`)

- [`profile_authoring.md`](runbooks/profile_authoring.md) — add a topic to
  the node's rendered scene via profile YAML only (autonomy-team audience).
- [`environment_bake.md`](runbooks/environment_bake.md) — bake OSM building
  footprints into the environment-chunk format the node loads at
  `on_activate()`.
- [`cesium.md`](runbooks/cesium.md) — Cesium ion account/token/asset
  contract + the `CESIUM_ION_TOKEN` smoke check; the Google Photorealistic
  3D Tiles preset.
- [`env_source_captures.md`](runbooks/env_source_captures.md) — one
  committed render per Environment Tiles preset (baked / osm / google) for
  judging the streamed sources side by side.
- [`theme_showcase.md`](runbooks/theme_showcase.md) — opt-in whole-palette
  single-frame capture harness used to judge theme candidates.
- [`signoff.md`](runbooks/signoff.md) — the open/closed sign-off items
  across epics, including the named exceptions the sign-off accepts.
- [`bowl-golden-vm091.md`](runbooks/bowl-golden-vm091.md) /
  [`hybrid-golden-vm094.md`](runbooks/hybrid-golden-vm094.md) — the
  unified-engine migration's parity golden packages.
- [`ci_gate.md`](runbooks/ci_gate.md) — what `tools/ci_visual_mode.sh`
  actually runs, what its green does and does not cover, and the
  `viz_benchmark` perf tool.

> **Scope note:** deployment/architecture notes for a two-node topology were
> deferred at VM-095 and never re-entered — the unified-engine migration
> (`docs/plans/2026-09-10-unified-engine-migration.md`) deleted that
> topology, so `profile_authoring.md` and `environment_bake.md` above cover
> only the docs half of that deferred item.

## Design (`docs/design/`)

- [`2026-08-18-visual-mode-design.md`](design/2026-08-18-visual-mode-design.md)
  — the visual-mode design spec (accepted; amended by the 2026-09-07 plan
  review).
- [`2026-08-18-visual-mode-backlog.md`](design/2026-08-18-visual-mode-backlog.md)
  — the epic/task backlog spec the design was broken down into.

## ADRs (`docs/adr/`)

- [`0001-filament-pin-and-headless-egl.md`](adr/0001-filament-pin-and-headless-egl.md)
- [`0002-two-node-mode-mux.md`](adr/0002-two-node-mode-mux.md) — superseded by ADR-0006.
- [`0003-pod-boundary-clang-libcxx-lib.md`](adr/0003-pod-boundary-clang-libcxx-lib.md)
- [`0004-scene-interface-versioning.md`](adr/0004-scene-interface-versioning.md)
- [`0005-camera-frame-pod-boundary.md`](adr/0005-camera-frame-pod-boundary.md)
- [`0006-one-node-unified-engine.md`](adr/0006-one-node-unified-engine.md)

## Plans (`docs/plans/`)

- [`2026-09-17-overlume-restructure.md`](plans/2026-09-17-overlume-restructure.md)
  — the live restructure plan (this reorganization).
- [`2026-08-18-visual-mode.md`](plans/2026-08-18-visual-mode.md) — the
  master rolling-wave plan.
- [`2026-08-18-visual-mode-epic1.md`](plans/2026-08-18-visual-mode-epic1.md)
  … [`epic2`](plans/2026-08-18-visual-mode-epic2.md),
  [`epic3`](plans/2026-08-18-visual-mode-epic3.md),
  [`epic4`](plans/2026-08-18-visual-mode-epic4.md),
  [`epic6`](plans/2026-08-18-visual-mode-epic6.md) — per-epic implementation
  plans (Epic 5 was never authored as its own plan document; see
  `docs/status.md`).
- [`2026-09-09-vm077-new-stack-rendering.md`](plans/2026-09-09-vm077-new-stack-rendering.md)
  — the flicker/new-stack-rendering follow-on to Epic 3.
- [`2026-09-10-unified-engine-migration.md`](plans/2026-09-10-unified-engine-migration.md)
  — the CUDA-to-Filament unified-engine cutover.
- [`archive/`](plans/archive/README.md) — retired NumPy/GL/CUDA prototype
  plans and specs (2026-06/07), plus the superseded Azure DevOps project
  backlog export.

## Status

[`docs/status.md`](status.md) — the single status ledger: shipped epics,
open items, known gaps/accepted exceptions, and how to update it.

## Evidence and assets

- [`docs/evidence/`](evidence/) (LFS) — measurement captures cited by the
  plans (`vm040-governor-2026-09-11/`, `vm077-flicker-2026-09-10/`).
- [`docs/assets/`](assets/) — reference images
  (`visualization-reference-1.jpg`, `visualization-reference-2.jpg`) used
  as styling ground truth.
- [`docs/runbooks/env_source_captures/`](runbooks/env_source_captures/) (LFS)
  — the committed PNGs `env_source_captures.md` documents.

## Path map for readers of pre-2026-09-17 documents

| Old path | New path |
|---|---|
| `cuda/src/libs/visual_renderer` | `overlume/` |
| `cuda/src/ros_apps/src/micropilot_visualization_node` | `ros/src/overlume_ros/` |
| `cuda/scripts/ros_apps_build` | `ros/` |
| `docs/visual_mode/` | `docs/runbooks/` |
| `docs/visual_mode/README.md`'s pre-merge-gate + benchmark sections | `docs/runbooks/ci_gate.md` |
| `docs/superpowers/plans/2026-08-*`, `2026-09-*` | `docs/plans/` |
| `docs/superpowers/plans/2026-06-*`, `2026-07-*` | `docs/plans/archive/` |
| `docs/superpowers/specs/2026-06-*`, `2026-07-*` | `docs/plans/archive/` |
| `docs/superpowers/specs/2026-08-18-visual-mode-{design,backlog}.md` | `docs/design/` |
| `docs/visual_mode_project_backlog.md` | `docs/plans/archive/visual_mode_project_backlog.md` (superseded by `docs/status.md`) |
| `mpviz` / `MPVIZ_*` / `visual_renderer` (identifiers) | `overlume` / `OVERLUME_*` / `overlume` |
| `micropilot_visualization_node` (ROS package/node) | `overlume_ros` / node `overlume_node` |

Historical documents under `docs/plans/2026-08-18-*`, `2026-09-09-*`,
`2026-09-10-*`, and `docs/plans/archive/` were **not** edited for this
restructure — every path and identifier inside them is pre-restructure; use
the table above to translate.
