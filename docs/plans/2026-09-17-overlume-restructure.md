# Overlume open-source restructure — implementation plan

> **For agentic workers:** execute task-by-task with the repo's mandatory workflow
> pattern (orchestrator Fable, implementers Sonnet, review gates Opus, ≤2 fix
> rounds per task). Each task ends green on `tools/ci_visual_mode.sh` and is
> committed on its own. Checkboxes track status; this file is the ledger for
> the restructure and nothing else is.

**Goal:** turn the TPSProjector monorepo into **Overlume**, a polished,
contributor-ready open-source project: one real-time rendering library, one
ROS 2 integration app, C++ examples of the public API, generated API docs,
and the release scaffolding top-tier projects ship with.

**Architecture (unchanged):** a clang/libc++ Filament library behind a POD-only
public header pair, consumed by a gcc ROS 2 lifecycle node through that
boundary (ADR-0003/0004/0006). This plan moves, renames, deletes and documents.
It changes no rendering behaviour: every golden must pass byte-identical
before and after each task.

**Decisions taken by the user (2026-09-17):** delete legacy code outright;
git-LFS acceptable; rename `cuda/` now; project name **Overlume**; license
**Apache-2.0**, copyright **Amer Ghazal**; **full identifier rename** (library,
namespace, env vars, ROS package); add `examples/`; add generated API docs.
The user renames the GitHub remote themselves.

## Global constraints

- **Behaviour freeze.** No rendering, theme, or protocol change in any task.
  Goldens are not re-promoted in this plan. `ci_visual_mode.sh` OVERALL PASS
  after every task, with the same ok/skipped counts as before the task
  (library 242 / 232 ok / 6 skipped; node 281; bridge 74; goldens 20).
- **Data contracts are not identifiers.** ROS topic/frame/service strings that
  name the robot's autonomy stack (`/micropilot/...`, `/perception/...`),
  fixture bag contents, profile YAML topic rows, and recorded fixtures stay
  byte-identical. Only code identifiers, package names, env vars, paths and
  docs rename.
- **History is not rewritten.** LFS conversion uses `--no-rewrite`; historical
  plan documents keep their old paths and get a path-map note, not edits.
- **Token rules stand.** `CESIUM_ION_TOKEN` / `MAPBOX_TOKEN` by name only;
  nothing in the new CI may need them.
- **ADR-0004 stands.** Public headers stay POD-only and append-only; the
  namespace rename does not bump `kSceneVersion` (a rename is not a layout
  change; `test_scene_layout.cpp`'s cross-toolchain `== 6` mirror proves it).

## Target layout

```
overlume/                       the library (was cuda/src/libs/visual_renderer)
  include/overlume/{scene.h,api.h}
  src/  tests/  assets/  cmake/  scripts/  tools/
ros/                            colcon workspace root (was cuda/src/ros_apps + cuda/scripts/ros_apps_build)
  src/overlume_ros/             the ROS 2 node (was micropilot_visualization_node)
  colcon_build.sh  config_colcon.yaml
examples/                       C++ API usage, built by the library's CMake
tools/                          operator + gate scripts (unchanged home)
docs/
  README.md                     the single docs index
  adr/                          unchanged
  runbooks/                     was docs/visual_mode/*.md (+ its capture PNGs)
  design/                       the visual-mode design spec + backlog spec
  plans/  plans/archive/        live plans here; June/July-2026 prototype docs archived
  status.md                     ONE status ledger (replaces plan-embedded ledgers + both backlogs)
  evidence/                     LFS
  assets/                       reference images (was assets/*.jpg)
.github/                        workflows, issue/PR templates
LICENSE  NOTICE  README.md  CONTRIBUTING.md  CODE_OF_CONDUCT.md  SECURITY.md  CHANGELOG.md
AGENTS.md (canonical agent instructions)  CLAUDE.md (imports AGENTS.md)
```

## Status ledger

| Task | Title | Status |
|---|---|---|
| 0 | LFS + deletion of legacy code and agent-config sprawl | ☑ 2026-09-17 — 782a2ad (deletions, 94 files), 7512863 (.gitattributes), LFS pointer commit (73 binaries, `--no-rewrite`, bytes verified identical). Gate PASS with unchanged counts. Note: `git lfs install` collides with graphify's `post-checkout` hook — the LFS lines were appended to the existing hooks by hand; contributors get this via `git lfs install` on a fresh clone where no conflict exists. |
| 1 | Move the two packages; fix every hard-coded path | ☑ 2026-09-17 — `overlume/`, `ros/src/micropilot_visualization_node/`, `ros/colcon_build.sh` (322 renames); node CMake locates the library as `<repo>/overlume` + `overlume/build`; `ros/` is the colcon workspace with default bases (the `--build-base` trap is gone). Gate PASS with unchanged counts after a FRESH library configure at `overlume/build` and a fresh colcon build. Recorded exceptions: the residue grep keeps historical lines in append-only records — ADR-0002/0003/0006, the two dated design documents under docs/design/, and a frozen VM-040 evidence capture (10 lines across 6 files at the final review) — covered by the path map in docs/README.md, not edited. The plan's clean-worktree gate was substituted by cache-provenance verification (new CMakeCache has no old paths; all three consumers agree); a true fresh clone incl. `git lfs pull` is owed to Task 8. Round-1 gate caught two node Python tests with a stale 6-level `REPO_ROOT` that made them silently SKIP — fixed to 4 levels, both now run and pass. |
| 2 | Identifier rename: `mpviz`→`overlume`, `MPVIZ_`→`OVERLUME_`, `micropilot_visualization_node`→`overlume_ros` | ☑ 2026-09-17 — one-shot `overlume/scripts/rename_identifiers.sh` + six `git mv`s (include dirs, package dir, `overlume_node.{cpp,hpp,launch.py}`). ROS graph: node `visualization_node`→`overlume_node`, so `/overlume_node/…` services/params/diagnostics; params YAML root key follows; every consumer (bridge, GUI, validate, perf gates, tests) updated. Class `VisualizationNode`→`OverlumeNode`. Toolchain caches moved to `~/.cache/overlume-toolchain{,-cesium}`, tile cache default `overlume-tile-cache`. Fresh library configure + fresh colcon build; gate PASS at unchanged counts, goldens byte-identical (LFS pointers unchanged). Round-1 gate caught `sample_diagnostics.py` still on the old diagnostics topic (would have broken all four perf gates) — fixed. Gate wording caveat: the rename grep is met for identifiers; the same historical files listed in row 1 keep old names by design. **Follow-up recorded:** the node keeps TWO namespaces, `overlume_node` (31 files, the former `visualization_node`) and `overlume::ros` (the former `micropilot::visualization_app`); test fixtures live in `overlume_node::testing`. Merging them is a real refactor (name-collision check across 31 files), owed to Task 8's review or a follow-up, not a sed. |
| 3 | Docs restructure + single status ledger + root README | ☑ 2026-09-17 — dc030b2. runbooks/, design/, plans/ (+archive/), docs/README.md index + path map, docs/status.md as the single ledger, root README rewritten, tools/check_docs_links.py (live set green; LICENSE allowlisted until Task 6). Gate round 1 caught a README build command with a relative --toolchain path that CMake resolves against the build dir — fixed and re-run. Two status.md source discrepancies (Epic 0 commit range provenance, Epic 2 gate date vs commit date) are stated in the file rather than smoothed. |
| 4 | `examples/` — C++ public-API examples | ☑ 2026-09-17 — this commit. Six programs against <overlume/api.h>/<overlume/scene.h> only, PNG output via a tiny shared common.hpp, built under OVERLUME_BUILD_EXAMPLES (EXISTS-guarded) and run as ci_visual_mode.sh stage 6 ("6 run"). Gate round 1: 04_virtual_camera wrote a near-empty frame under a comment calling it the most interesting one — given a small visible scene, comment made truthful; the fix-round implementer returned placeholder output, so the orchestrator applied that fix and verified the frame by eye. The legacy overlume/examples/hello_frame.cpp and its glob block are removed (superseded by 01_hello_frame). Recorded for Task 8: examples pass no model-assets dir, so objects render as the procedural clay-box fallback (a WARN per class); consider exposing the models dir the same way the theme dir is. examples/README.md gains its API-docs link in Task 5. |
| 5 | API docs (Doxygen) + Doxygen-grade public headers | ☑ 2026-09-17 — `docs` CMake target (opt-in, not in ALL; `OVERLUME_BUILD_DOCS` ON iff Doxygen found; doxygen-awesome-css v2.5.0 pinned with SHA256; EXISTS-guarded like examples), `Doxyfile.in` at the repo root, zero warnings under `WARN_AS_ERROR=FAIL_ON_WARNINGS`; every public symbol in scene.h/api.h documented with the original rationale preserved and NO declaration changed (verified by diff filter); `include/overlume/version.h` (POD, macros + a derived `kVersionString`) with a configure-time drift check against `project(overlume VERSION 0.1.0)` — a version bump must touch both or configure fails. **Deviations from the plan text:** Doxygen INPUT is `overlume/include`, `examples/`, and a dedicated `docs/api_mainpage.md` main page (not `README.md`, whose Markdown links Doxygen turns into unresolvable `\ref`s; not `overlume/src` — internals are not the API reference; not `docs/*.md`); output lands in `overlume/build/docs/html` (Task 6's Pages workflow publishes that path, not `build-docs/html`); `project(VERSION)` + `version.h` were pulled forward from Task 6. |
| 6 | Open-source scaffolding: LICENSE/NOTICE/SPDX, CONTRIBUTING, CoC, SECURITY, CHANGELOG, `.github` CI + release | ☑ 2026-09-17 — 83ad0b1 (scaffolding + workflows) and 6b54884 (one repo-wide clang-format, clang-format 23.1.1 pinned via PyPI, SPDX on 242 files, ctest `cpu`/`gpu` labels; listed in `.git-blame-ignore-revs`). Repo made public and Pages (source: GitHub Actions) + private vulnerability reporting enabled 2026-09-17; site <https://amerghazal7.github.io/overlume/>. **Unverified until the first push:** all four workflows (parsed and read against the actions' inputs only); the hosted build depends on apt.llvm.org's jammy pool keeping the pinned LLVM 18.1.8 `.deb`s. Gate round 1 (6b) caught a `nogpu` label that ctest's unanchored `-L` regex would have matched against `gpu`; renamed `cpu`. |
| 7 | Graphify + agent hooks cleanup; memory update | ☑ 2026-09-17 — snapshot dirs pruned (103 MB → ~70 MB), `graphify install` refreshed the vendored skill, the Read/Glob hook is no longer `--strict`, AGENTS.md documents the hooks and the user-level GateGuard, graph rebuilt after the moves. Memory files (outside the repo) rewritten for Overlume: project + architecture memories replaced, a restructure memory added, every stale path/identifier swept. |
| 8 | Final cross-cutting review (Opus) + close | ☑ 2026-09-17 — six Opus reviewers (first visit, licensing/security, identifiers/contracts, CI workflows, examples/API docs, plan close) + one refuter per finding: 56 raised, 50 confirmed (7 blocking, 6 major, 37 minor), 6 refuted; all confirmed items fixed this commit. Clean-worktree build (Task 1's owed gate) PASSED: fresh `git lfs pull` (bytes identical), toolchain no-op from cache, README quick start followed verbatim, gate at the frozen counts. Blocking items and their fixes: SPDX missing on the two gate scripts (lint would have failed on the first push); no git-lfs instructions anywhere (README + CONTRIBUTING prerequisite added); the quick start's fixture-bag step relied on an internal recording (documented, `--bag`/`record_fixture_bag.sh`, default path now `$OVERLUME_FIXTURES`); the two vendor HMI reference JPGs were unattributed third-party captures — REMOVED from the tree, every citation reworded, NOTICE untouched by design; **their purge from git/LFS history and the push itself are the user's decisions (history was already on the remote as TPSProjector).** Majors: NOTICE now covers the committed Cesium OSM Buildings tile fixtures (ODbL; ion terms flagged as an open legal check); five perf-gate scripts hard-coded the author's home path (repo-root derivation); `nogpu` residue in build.yml/CONTRIBUTING (→ `cpu`); ledger residue wording corrected. Repository settings applied via `gh api`: Pages (source GitHub Actions), private vulnerability reporting, secret scanning + push protection. Also: hero image under a neutral `docs/assets/hero.png`, workflow permissions least-privilege, Doxygen tarball checksum, `--verify-tag`, CHANGELOG 0.1.0 wording pending the tag, Doxygen `STRIP_FROM_PATH`. |

Path map for readers of pre-2026-09-17 documents: `cuda/src/libs/visual_renderer` → `overlume/`; `cuda/src/ros_apps/src/micropilot_visualization_node` → `ros/src/overlume_ros/`; `cuda/scripts/ros_apps_build` → `ros/`; `docs/visual_mode/` → `docs/runbooks/`; `docs/superpowers/{plans,specs}` → `docs/plans/` and `docs/design/` (June/July 2026 items under `plans/archive/`).

---

## Task 0 — LFS + delete legacy

**Delete (git rm):** `tpsprojector/`, `tests/`, `run_tests.sh`, `pytest.ini`,
`requirements.txt` (root; a `tools/requirements.txt` replaces it),
`scripts/gl_montage.py`, `cuda/src/libs/rendering_reprojector/`, `cuda/tools/`,
`cuda/examples/`, `cuda/tests/`, `cuda/cmake/`, `cuda/CMakeLists.txt`,
`cuda/src/libs/CMakeLists.txt`, `cuda/scripts/libs_build/`, `.superpowers/`,
`.codex/`, `.cursor/`, `docs/agents/`, `assets/*.rviz` (moved to the node's
`config/rviz/`), `assets/*.jpg` (moved to `docs/assets/`).
Consumers to fix: `micropilot_visualization_node/scripts/autotune_config.py`
imports `tpsprojector` and `rendering_reprojector` — it tuned the removed CUDA
node; delete it. Theme YAML comments cite `assets/visualization-reference-2.jpg`
→ `docs/assets/`. `cuda/.clang-format` → root `.clang-format`.

**LFS:** `git lfs install`; `.gitattributes` tracks `*.png *.gif *.jpg *.glb
*.b3dm *.bin *.ttf` via LFS; `git lfs migrate import --no-rewrite --include=…`
converts the tracked binaries in one commit without rewriting history. Verify
`git lfs ls-files | wc -l` equals the tracked binary count and that a clean
`git clone` + `git lfs pull` gets identical bytes (sha256 of one golden).

**Agent config:** `AGENTS.md` becomes the canonical instructions (graphify rules,
workflow pattern, token rules, build/test commands); `CLAUDE.md` becomes a
one-line `@AGENTS.md` import; `.claude/CLAUDE.md` deleted; `.claude/settings.json`
keeps only hooks that still apply.

**Gate:** `ci_visual_mode.sh` OVERALL PASS with unchanged counts; `git grep -l
"tpsprojector\|rendering_reprojector"` returns only `docs/` history and
`CHANGELOG.md`.

## Task 1 — Move the packages, fix every path

`git mv cuda/src/libs/visual_renderer overlume`; `git mv
cuda/src/ros_apps/src/micropilot_visualization_node ros/src/micropilot_visualization_node`
(the package RENAME is Task 2; this task moves only); `git mv
cuda/scripts/ros_apps_build/* ros/`; remove the now-empty `cuda/`.

Files with hard-coded paths to fix (from the 2026-09-17 scan): `tools/ci_visual_mode.sh`
(5), `tools/validate_visual_mode.sh` (5), `ros/colcon_build.sh` (5) +
`config_colcon.yaml` (2), the node's 10 Python integration tests (5 each: the
`cuda/install/ros_apps` setup.bash path), `tools/test_vcam_ws_bridge.py` (3),
`tools/vcam_ws_bridge.py`, `tools/flicker_measure.sh`, node `scripts/
provision_ego_model.sh`, node `CMakeLists.txt` (library location, toolchain
`find_program`), `overlume/scripts/setup_toolchain*.sh`, `overlume/tools/*_perf_gate.sh`
and `budget_probe.sh`, `.gitignore` (7 rules; replace the four colcon
run-from-anywhere rules with `ros/{build,install,log}/`), `docs/visual_mode/*.md`
runbooks (moved in Task 3, paths fixed here), root `README.md` (rewritten in
Task 3; paths fixed here). Colcon: `ros/` IS the workspace, so `colcon build`
runs from `ros/` with default bases — retire the `--build-base` trap.

**Gate:** clean-checkout build in a temp worktree: `overlume/scripts/setup_toolchain_cesium.sh`,
library configure+build, `ros/colcon_build.sh`, then `ci_visual_mode.sh` OVERALL
PASS. `git grep -n "cuda/src\|ros_apps"` outside `docs/plans` and `CHANGELOG.md`
returns nothing.

## Task 2 — Identifier rename

| from | to | scope |
|---|---|---|
| `namespace mpviz` / `mpviz::` | `overlume` | 4,238 occurrences, 183 files |
| `MPVIZ_*` (20 env vars + CMake options) | `OVERLUME_*` | 208 occurrences, 33 files; `MPVIZ_ENABLE_CESIUM`→`OVERLUME_ENABLE_CESIUM` etc. |
| `visual_renderer` (target, include dir, CMake project) | `overlume` | `#include <overlume/scene.h>`; target `overlume::overlume` |
| ROS package `micropilot_visualization_node`, include dir, node name, executable | `overlume_ros`, node `overlume_node` | 218 occurrences, 102 files |
| C++ `namespace micropilot::visualization_app` | `overlume::ros` | node sources |
| spdlog logger `"mpviz.cesium"` | `"overlume.cesium"` | one line |
| golden/fixture FILE names | unchanged | names do not encode identifiers |
| topic/frame/service DATA strings, profile YAML rows, fixture bags | **unchanged** | data contract |
| historical docs under `docs/plans/` | **unchanged** | path map covers them |

Mechanical: one reviewed `sed` script committed under `overlume/scripts/rename_identifiers.sh`
(kept for the record, marked one-shot), applied to code/tests/tools/runbooks;
`git mv` for the include dir and package dir; CMake target/alias names; the
WS bridge protocol is untouched (it never carried identifiers).

**Gate:** clean rebuild both packages; `ci_visual_mode.sh` OVERALL PASS with
unchanged counts and all 20 goldens byte-identical (`git status` shows no
golden change); `git grep -n "mpviz\|MPVIZ_\|micropilot_visualization_node"`
returns only `docs/plans/`, `CHANGELOG.md`, and profile/topic data rows.

## Task 3 — Docs restructure + single status ledger + root README

- `docs/visual_mode/*.md` + `env_source_captures/` → `docs/runbooks/`; index in `docs/README.md`.
- `docs/superpowers/plans/2026-08-*` + `2026-09-*` → `docs/plans/`; all `2026-06-*`/`2026-07-*`
  plans and specs → `docs/plans/archive/` with a one-paragraph `archive/README.md`
  saying they describe the retired NumPy/GL/CUDA prototypes.
- `docs/superpowers/specs/2026-08-18-visual-mode-{design,backlog}.md` → `docs/design/`.
- **`docs/status.md`**: the one ledger. Sections: shipped epics (one line each,
  pointing at the plan), open items (the 10 from Epic 6's close + this plan's),
  known gaps, how to update it. `docs/visual_mode_project_backlog.md` → `docs/plans/archive/`.
- Root `README.md` rewritten for Overlume: what it is (one paragraph), a hero
  image (`docs/runbooks/env_source_captures/…` or a theme showcase render),
  features, quick start (toolchain → build → run the headless example → run the
  ROS node on the fixture bag), architecture diagram (Mermaid), docs links,
  contributing, license. `docs/visual_mode/README.md`'s CI-gate section moves
  to `docs/runbooks/ci_gate.md`.

**Gate:** every link in `docs/**/*.md` and `README.md` resolves (a link-check
script in `tools/`); no file left under `docs/superpowers/` or `docs/visual_mode/`.

## Task 4 — `examples/`

Self-contained C++ programs against **public headers only**
(`<overlume/scene.h>`, `<overlume/api.h>`), each ≤200 lines, each writing a PNG
and exiting 0, built by `overlume/CMakeLists.txt` under `OVERLUME_BUILD_EXAMPLES`
(default ON), run headless by a new `ci_visual_mode.sh` stage:

1. `01_hello_frame` — create a headless renderer, load a theme dir, render one frame, write PNG.
2. `02_scene_population` — ego pose, map elements of every `MapKind`, tracked objects of every class, a behavior ribbon; the freeze-frame/double-buffer contract shown explicitly.
3. `03_themes` — load both shipped themes, trigger `set_theme` and render mid-transition frames.
4. `04_virtual_camera` — presets, the tween, projecting a world point to screen.
5. `05_environment` — baked chunks from a directory; `ion://` streaming only if `CESIUM_ION_TOKEN` is set (skips cleanly otherwise).
6. `06_overlays_and_pointcloud` — point cloud, alert polygons, HUD color query.

`examples/README.md` explains each and links to the API docs. The ROS node is
described as "the reference integration app", and its adapters/profile
concept gets a paragraph so a contributor knows where the ROS-specific code
lives.

**Gate:** examples build with the library, run on the GPU box, and produce
non-empty PNGs; a review checks they use no `src/`-private header.

## Task 5 — API docs

Doxygen (chosen over Sphinx/Breathe: the public API is two POD C headers and a
C++ library, Doxygen renders that natively, one tool, no Python toolchain).
`Doxyfile` at repo root: input = `overlume/include`, `overlume/src` (internal,
`INTERNAL_DOCS=NO`), `examples/`, `docs/*.md`; `USE_MDFILE_AS_MAINPAGE=README.md`;
theme `doxygen-awesome-css` fetched by CMake `FetchContent` for the `docs`
target; output `build-docs/html` (ignored). GitHub Pages workflow builds and
publishes on push to `main`. Public headers get a Doxygen pass (`@file`,
`@brief`, `@param`, `@since kSceneVersion N`) without changing declarations;
the POD header check still passes.

**Gate:** `doxygen` runs with zero warnings under `WARN_AS_ERROR=YES` for the
public headers; the Pages workflow is green on a branch push.

## Task 6 — Open-source scaffolding

`LICENSE` (Apache-2.0, "Copyright 2026 Amer Ghazal"); `NOTICE` with third-party
attributions (Filament, cesium-native, spdlog, fmt, yaml-cpp, stb, the fonts
and models already under `assets/*/ATTRIBUTION.md`, Cesium OSM Buildings and
Google Photorealistic 3D Tiles terms pointers); SPDX header
`// SPDX-License-Identifier: Apache-2.0` on every first-party source file;
`CONTRIBUTING.md` (toolchain, build, gate, golden promotion convention, ADR
process, commit style, review expectations); `CODE_OF_CONDUCT.md` (Contributor
Covenant 2.1); `SECURITY.md` (token handling, how to report); `CHANGELOG.md`
(Keep a Changelog; `Unreleased` → `0.1.0` with the epics as history);
versioning already landed in Task 5 (`project(VERSION)` + `version.h`); `.github/`: `ISSUE_TEMPLATE/{bug,feature}.yml`,
`PULL_REQUEST_TEMPLATE.md`, `workflows/lint.yml` (clang-format check,
shellcheck, `python -m py_compile` + `pytest tools/` unit subset),
`workflows/build.yml` (library configure+build with `OVERLUME_ENABLE_CESIUM=OFF`
and `ctest -L cpu` — labels are `cpu`/`gpu`, not `nogpu`, because ctest's `-L` regex is unanchored and `nogpu` would also match a `-L gpu` selector; GPU tests documented as self-hosted only),
`workflows/docs.yml` (Task 5), `workflows/release.yml` (tag `v*` → GitHub
release with the CHANGELOG section). The user creates the tag; nothing in this
plan publishes.

**Gate:** all workflows pass on a branch push; `reuse lint` (or a grep) shows
every first-party source file carries the SPDX line.

## Task 7 — Graphify + agent hooks + memory

Prune `graphify-out` dated snapshots to the latest; `graphify install` to fix
the 0.8.49→0.9.40 skill drift; `AGENTS.md` drops the wiki claim; strict-hook
default relaxed to `GRAPHIFY_HOOK_STRICT=0` for Markdown edits; note that the
fact-forcing GateGuard is a user-level plugin (not repo config) and record the
recommendation to disable it for `docs/**`. Update the six memory files that
cite old paths/identifiers.

## Task 8 — Final review + close

Cross-cutting Opus review (structure, links, licence/SPDX coverage, identifier
residue, CI health, examples honesty, docs accuracy); fixes; `status.md` marks
the restructure closed; `CHANGELOG.md` `0.1.0` entry final; `graphify update .`.
