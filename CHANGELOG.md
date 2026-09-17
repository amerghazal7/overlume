# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

The Overlume open-source restructure (`docs/plans/2026-09-17-overlume-restructure.md`):

- Renamed the project to **Overlume**: `mpviz`/`MPVIZ_*` identifiers,
  namespaces, the ROS package (`micropilot_visualization_node` →
  `overlume_ros`, node `overlume_node`), env vars, and cache directories,
  all renamed with data contracts (topics, frames, services, fixture bags)
  left byte-identical.
- Restructured the repository layout: the library moved to `overlume/`, the
  ROS 2 workspace to `ros/` (`ros/src/overlume_ros/`), legacy CUDA/NumPy/GL
  prototype code and agent-config sprawl deleted, tracked binaries moved to
  Git LFS.
- Added `examples/`: six self-contained C++ programs against the public API
  headers only (`<overlume/scene.h>`, `<overlume/api.h>`), run headless as a
  gate stage.
- Added a generated API documentation target (`docs`, Doxygen +
  doxygen-awesome-css) over the public headers and examples, plus
  `include/overlume/version.h` with a configure-time drift check against
  `project(overlume VERSION ...)`.
- Restructured docs into one index (`docs/README.md`), one status ledger
  (`docs/status.md`), and `docs/runbooks/`/`docs/design/`/`docs/plans/`
  (with `docs/plans/archive/` for retired prototype documents).
- Added open-source project scaffolding: `LICENSE`, `NOTICE`,
  `CODE_OF_CONDUCT.md`, `SECURITY.md`, `CONTRIBUTING.md`, issue/PR templates,
  and hosted CI (`.github/workflows/`: lint, build, docs, release).

## [0.1.0] - 2026-09-17

The first tagged release, covering everything delivered before the
open-source restructure. One line per shipped item, sourced from
[`docs/status.md`](docs/status.md)'s own Shipped table (dates/hashes as
recorded there; see that table for the full provenance notes on the two
entries with a recorded historical-document discrepancy).

### Added

- **Epic 0 — Contract spike**: Filament hello-frame, mode mux, virtual-camera
  parity, GPU budget probe. Commits `66c1340`…`e619d41`, 2026-08-18 (GPU-budget
  item VM-043 closed 2026-09-11 on a dev-box proxy).
- **Epic 1 — Core scene & dark theme**: closed 2026-08-20 at `e47b057`.
- **Epic 2 — Autonomy data ingestion**: closed 2026-09-07 at `fd72331` (gate
  passed at `37d41fe`).
- **Epic 3 — HUD, polish & controls**: closed 2026-09-09 at `d62f8e3`.
- **VM-077 — new-stack rendering / flicker root-cause fix** (Epic 3
  follow-on): closed 2026-09-10.
- **Epic 4 — Clay buildings (`EnvironmentLayer`), Tasks 1-3**: done
  2026-09-10.
- **Epic 5 — Hardening & delivery**: VM-040 (quality governor) and VM-044
  (asset packaging) done 2026-09-11; VM-041 (perf benchmark +
  `ci_visual_mode.sh`) and VM-042 (docs) delivered in parallel; VM-043 (live
  validation sign-off) superseded by the unified-engine migration's own
  parity sign-off.
- **Unified-engine migration** (VM-090…095): CUDA→Filament cutover, the mode
  mux and `micropilot_rendering_node` retired. Closed 2026-09-11, all 6
  tasks done.
- **Epic 6 — v1.1: Cesium 3D Tiles streaming** (VM-060…064) + post-close
  tail: closed 2026-09-16, final cross-cutting review 2026-09-17.

[Unreleased]: https://github.com/amerghazal7/overlume/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/amerghazal7/overlume/releases/tag/v0.1.0
