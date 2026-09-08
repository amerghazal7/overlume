# Knowledge graph: graphify

This repo keeps a graphify knowledge graph of the whole corpus (C++/CUDA/Python sources, ROS configs, ADRs, plans, and the golden render fixtures). Agents should read the graph before exploring the tree by hand, and keep it current after changing code.

## Where it lives

Everything is under `graphify-out/`, which is **gitignored** — the graph is a local build artifact, not shared state. Each clone builds its own.

- `graphify-out/graph.json` — the graph itself, the input to every query
- `graphify-out/GRAPH_REPORT.md` — audit report: god nodes, communities, surprising connections
- `graphify-out/graph.html` — interactive graph, open in a browser
- `graphify-out/cache/` — extraction cache, keyed by file content hash

Because it is gitignored, **a fresh clone has no graph.** If `graphify-out/graph.json` is missing, build it once with `/graphify .` before relying on the query-first rule below. Everything after that is incremental.

## Query before you explore

For any question about how this codebase fits together, query the graph first. It returns a scoped subgraph that is usually far smaller than the equivalent grep output, and it surfaces cross-file and INFERRED edges that grep cannot see.

- `graphify query "<question>"` — general codebase or architecture question
- `graphify query "<question>" --dfs` — trace one specific path instead of a broad neighbourhood
- `graphify path "<A>" "<B>"` — shortest dependency path between two symbols
- `graphify explain "<concept>"` — a node and everything around it
- `graphify affected "<symbol>"` — reverse traversal: what breaks if this changes
- `graphify god-nodes` — the architectural hubs

This applies to **subagents too**. When spawning a subagent that explores code, restate the query-first rule in its prompt; a subagent does not inherit this file's context automatically.

Go straight to `Read`/`Grep`/`Glob` when:

1. The graph has already oriented you and you are editing or debugging specific lines.
2. `graphify-out/graph.json` does not exist yet.
3. You need exact current file contents — the graph describes structure, not the live text of a file you are about to edit.

Read `GRAPH_REPORT.md` only for a broad architecture review, when `query`/`path`/`explain` did not surface enough.

## Keeping the graph current

**Code changes are automatic.** A `post-commit` hook re-extracts changed code files and rebuilds `graph.json`. It runs detached, so `git commit` returns immediately; watch it with:

```bash
tail -f ~/.cache/graphify-rebuild.log
```

The hook is AST-only — no LLM, no API cost. It skips itself during rebase, merge, and cherry-pick so it never blocks a `--continue`. A `post-checkout` hook keeps the graph aligned when switching branches.

**Docs and images are not automatic.** The hook deliberately ignores non-code files. After changing ADRs under `docs/adr/`, plans under `docs/superpowers/`, ROS YAML configs, or the golden PNGs, refresh them yourself:

```bash
/graphify . --update      # re-extract only new/changed files
```

Mid-session, after editing code but before committing, you can refresh without waiting for the hook:

```bash
graphify update .         # AST-only, no API cost
```

Rebuild from scratch (`/graphify .`) only after a large refactor that deletes or moves many files, or when the graph looks structurally wrong. A full rebuild re-runs LLM extraction on docs and images, so it costs tokens; the content-hash cache makes unchanged files free.

If a rebuild legitimately shrinks the graph — you deleted a lot of code — graphify refuses to overwrite a larger `graph.json` unless you pass `--force`.

## Hook management

```bash
graphify hook status      # check post-commit / post-checkout / merge driver
graphify hook install     # (re)install
graphify hook uninstall   # remove
GRAPHIFY_SKIP_HOOK=1 git commit ...   # one-off bypass
```

Re-run `graphify hook install` if the graphify install moves — the hook pins an interpreter path at install time.

## Notes

- `.gitattributes` registers a union-merge driver for `graphify-out/graph.json`. It is inert while `graphify-out/` stays gitignored, and only matters if this repo ever starts tracking the graph.
- The agent-facing rules are mirrored in `.cursor/rules/graphify.mdc` (Cursor) and the `## graphify` section of `AGENTS.md`. Both are written by `graphify cursor install` / `graphify codex install`; re-running those commands regenerates them, so keep repo-specific guidance here rather than editing them by hand.
