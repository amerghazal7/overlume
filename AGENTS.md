# Agent instructions — Overlume

Canonical instructions for AI coding agents working in this repository.
`CLAUDE.md` imports this file; keep everything here, nothing there.

## What this repo is

Overlume: a real-time rendering library (Filament, clang/libc++) behind a
POD-only public header pair, plus a ROS 2 lifecycle node that feeds it live
robot data. Layout and the 2026-09-17 restructure plan:
`docs/plans/2026-09-17-overlume-restructure.md` (its path map covers older
documents that still cite pre-restructure paths).

## Hard rules

- **Tokens by name only.** `CESIUM_ION_TOKEN` and `MAPBOX_TOKEN` are never
  committed, echoed, logged, or pasted into any file or transcript. Scripts
  print PASS/FAIL and HTTP codes only. No test may need live network or a token.
- **Public headers are POD-only and append-only** (ADR-0003/0004). Struct
  changes bump `kSceneVersion`; free functions bump nothing.
  `overlume/scripts/check_pod_header.sh` must pass.
- **Goldens are promoted by a human.** A failing golden is a finding, not a
  file to overwrite. Whole-frame SSIM passing is not proof a golden is current;
  use the per-pixel drift audit after any palette-wide change.
- **Behaviour changes ship with a runnable check** that fails when the change
  is reverted.
- **Nothing local or internal reaches the public `origin`** (GitHub, public).
  Before merging into `main` and again before every push, run
  `tools/check_publish_leaks.sh` (default range `origin/main..HEAD`; pass the
  merge range otherwise). It scans added lines and commit messages for local
  absolute paths (home directories, agent scratch dirs) and for the internal
  names in the untracked `.git/info/publish-denylist`. Never commit that list,
  and never name internal repositories in tracked files: use placeholders such
  as `$WORKTREE` or `$PERCEPTION_REPO`. On a hit, rewrite the unpushed commits
  before publishing. Pushed history cannot be recalled, so never push first
  and fix after. In the same pre-push pass, run the lint job's checks, which
  `tools/ci_visual_mode.sh` does not cover: `tools/check_format.sh`,
  `tools/check_spdx.sh`, `shellcheck --severity=error tools/*.sh` and
  `python3 tools/check_docs_links.py`.

## How work is done

- Implementation and reviews run as dynamic workflows: orchestrator on the
  session model, implementers on Sonnet, review gates on Opus, at most two fix
  rounds per task; leftover minors are applied by the orchestrator.
- Every task ends green on `tools/ci_visual_mode.sh` (run in the foreground; a
  background run can be killed by a spurious low-memory guard) and is
  committed on its own with a message that says what changed and why.
- Status truth lives in `docs/status.md` (the single status ledger — shipped
  epics, open items, known gaps) and the active plan's own ledger, not in
  chat and not in memory. Docs are indexed at `docs/README.md`; runbooks live
  under `docs/runbooks/`, live plans under `docs/plans/` (retired-prototype
  history under `docs/plans/archive/`).

## Codebase questions

A knowledge graph exists at `graphify-out/` (ignored, regenerate with
`graphify update .`). Prefer `graphify query "<question>"`, `graphify path`,
`graphify explain` before grepping raw source; they return a scoped subgraph.
The graph is AST-only: for structure, docs, or tooling questions, read the
tree directly.

## Build and test

Library: `overlume/scripts/setup_toolchain_cesium.sh`, then `cmake --toolchain
"$PWD/overlume/cmake/toolchain-clang-libcxx.cmake" -S overlume -B overlume/build -DOVERLUME_ENABLE_CESIUM=ON
&& cmake --build overlume/build -j`. Node: `ros/colcon_build.sh`.
Gate: `tools/ci_visual_mode.sh`. Live rig: `tools/validate_visual_mode.sh --live`.
Logger-session replay (mp_play + static TFs + live rig): `tools/validate_logger_session.sh [SESSION_DIR]`.
API docs: `cmake --build overlume/build --target docs` (Doxygen; not part of
the default build) — see `docs/README.md`'s "API documentation" section.

## Hooks in this repo

`.claude/settings.json` runs `graphify hook-guard` before searches and reads
(non-strict: it nudges, it does not block). A separate user-level plugin
("GateGuard" fact-forcing, `ECC_GATEGUARD`) is NOT part of this repo; if it
is active in your environment it will ask for importers/rollback facts
before edits, including Markdown — set `ECC_GATEGUARD=off` for docs-only
sessions. The knowledge graph under `graphify-out/` is ignored by git;
rebuild it with `graphify update .` after code moves.
