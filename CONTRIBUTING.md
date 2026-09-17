# Contributing to Overlume

Thanks for considering a contribution. This document covers toolchain setup,
build commands, the pre-merge gate, golden promotion, the ADR process, code
style, commit messages, and PR expectations.

Agent instructions (for AI coding assistants working in this repo) live in
[`AGENTS.md`](AGENTS.md); this file is written for a person, `AGENTS.md` for
a tool, but they describe the same project.

## Code of Conduct

Participation in this project is governed by the
[Code of Conduct](CODE_OF_CONDUCT.md).

## Toolchain

**Git LFS first.** Goldens, fixtures, fonts, models and doc images are LFS
objects: `git lfs install` before cloning, or `git lfs pull` in an existing
clone. Without it those files are 130-byte pointers and the golden gate
stage fails instead of skipping.

The library is built with a pinned, root-less clang-18/libc++-18 toolchain.
The ROS node stays on the system's gcc/libstdc++ — see
`docs/adr/0003-pod-boundary-clang-libcxx-lib.md` for why the two never mix
at a `std::` boundary. Bootstrap it with:

```bash
overlume/scripts/setup_toolchain_cesium.sh
```

This is the single required bootstrap step regardless of whether you build
with `OVERLUME_ENABLE_CESIUM` ON or OFF: `overlume/cmake/toolchain-clang-libcxx.cmake`
resolves its clang-18/libc++-18 prefix from `~/.cache/overlume-toolchain-cesium`
only, and FATAL_ERRORs if that prefix isn't there. It is idempotent, needs
no root/sudo access, and also fetches the vcpkg toolchain used to build
cesium-native under the same clang/libc++ overlay triplet — running it once
covers both cases.

## Build commands

**Library:**

```bash
cmake --toolchain "$PWD/overlume/cmake/toolchain-clang-libcxx.cmake" \
    -B overlume/build -S overlume -DOVERLUME_ENABLE_CESIUM=ON
cmake --build overlume/build -j
```

**ROS 2 node:**

```bash
cd ros && ./colcon_build.sh
```

**API docs** (Doxygen, opt-in target):

```bash
cmake --build overlume/build --target docs
```

See the root [`README.md`](README.md) for the headless example and
fixture-bag run commands.

## The gate

`tools/ci_visual_mode.sh` is the pre-merge gate — run it before opening a PR:

```bash
tools/ci_visual_mode.sh
```

It runs, in order: a POD public-header check, the library's `ctest` suite,
the ROS node's `colcon test` suite, the WebSocket bridge's pytest suite, a
golden-suite OK/SKIPPED breakdown, and the six `examples/` programs headless. **It requires a GPU/EGL-capable box** —
without one, the renderer and golden-image stages fail, not skip.

What green does **not** cover — several Python integration tests not wired
into `colcon test`, and bridge tests that silently pass when `ros/install`
isn't built — is spelled out in
[`docs/runbooks/ci_gate.md`](docs/runbooks/ci_gate.md), along with what the
newer hosted CI (below) does and does not overlap with it.

### Hosted CI

GitHub Actions runs lint, a CPU-only, library-only build (`ctest -L cpu`
— see `docs/runbooks/ci_gate.md` for exactly which selector runs today),
the Doxygen docs build, and
tag-triggered releases — see `docs/runbooks/ci_gate.md`'s "Hosted CI"
section. It does not build or test the ROS node. **The GPU/EGL gate above
stays local** — hosted CI has no GPU runner, so a green Actions run is not a
substitute for running `tools/ci_visual_mode.sh` yourself before merging a
rendering change.

## Golden promotion

A failing golden (pixel-comparison test) is a **finding**, not a file to
overwrite. Promoting a new golden is a **human decision**, made after looking
at the actual pixels — not something CI does automatically:

- Whole-frame SSIM passing is **not proof** a golden is still correct; two
  frames can be structurally similar while a real regression hides in a
  small region.
- Do the per-pixel drift audit (diff image + a crop of anything that
  changed) before promoting, especially after any palette-wide or
  lighting-wide change.
- State in the PR description which goldens you promoted and why (what you
  looked at, what changed on purpose).

## ADR process

Architecture decisions live in `docs/adr/` as append-only, numbered
markdown files (`NNNN-title.md`). A new decision gets a new ADR; an old one
is marked **Superseded by ADR-NNNN** rather than edited or deleted (see
`docs/adr/0002-two-node-mode-mux.md`, superseded by ADR-0006, for the
pattern).

The public headers (`overlume/include/overlume/{scene.h,api.h}`) are
**append-only** per ADR-0004: new struct fields and enum values are
appended, never reordered or removed within a major version; a layout
change bumps `constexpr uint32_t kSceneVersion` in `scene.h`. Free-function
-only additions bump nothing. `overlume/scripts/check_pod_header.sh` (run
by the gate) enforces the POD-only half of this boundary.

## Code style

One repo-wide `.clang-format` at the root, applied to the whole tree in one
commit that is listed in `.git-blame-ignore-revs` — run
`git config blame.ignoreRevsFile .git-blame-ignore-revs` once so `git blame`
looks through it. Before committing:

```bash
tools/check_format.sh
```

Every first-party source file (not third-party/vendored code) carries an
SPDX header:

```
// SPDX-License-Identifier: Apache-2.0
```

checked by:

```bash
tools/check_spdx.sh
```

## Commit messages

```
type(scope): subject

Body explains what changed and why — the gate output, or a summary of it,
counts as part of "why" for a behavior-affecting change.
```

`type` is one of `feat`, `fix`, `refactor`, `docs`, `chore`, `test`, matching
this repo's existing history (`git log --oneline`). `scope` names the area
touched (`visual`, `restructure`, a package name, etc.).

## Pull requests

- Keep PRs small and focused on one task/change.
- Paste the gate output (`tools/ci_visual_mode.sh`'s PASS/FAIL summary) in
  the PR description.
- Update docs alongside the code they describe.
- If the change affects project status, update `docs/status.md` — the
  single status ledger (shipped epics, open items, known gaps). Don't start
  a second status surface in a plan document or elsewhere.
- State explicitly whether any golden was promoted, and by whom (a human,
  per the convention above) — never "CI promoted it."

## Agent instructions

If you're an AI coding agent (or driving one) in this repo, read
[`AGENTS.md`](AGENTS.md) first — it's the canonical instruction set (hard
rules, workflow pattern, build/test commands, the graphify knowledge graph).
`CLAUDE.md` is a one-line import of it.
