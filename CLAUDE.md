## graphify

This project has a knowledge graph at graphify-out/ with god nodes, community structure, and cross-file relationships.

Rules:
- For codebase questions, first run `graphify query "<question>"` when graphify-out/graph.json exists. Use `graphify path "<A>" "<B>"` for relationships and `graphify explain "<concept>"` for focused concepts. These return a scoped subgraph, usually much smaller than GRAPH_REPORT.md or raw grep output.
- If graphify-out/wiki/index.md exists, use it for broad navigation instead of raw source browsing.
- Read graphify-out/GRAPH_REPORT.md only for broad architecture review when query/path/explain do not surface enough context.
- After modifying code, run `graphify update .` to keep the graph current (AST-only, no API cost).

## Graphify context selection

Query the graph **before** choosing files for context. Full policy: `docs/agents/graphify.md`.

1. Run `graphify query "<the task>"` (or `path` / `explain` / `affected`).
2. Include only the `source_file` paths that subgraph cites — plus the live file you are about to edit.
3. Do not dump `GRAPH_REPORT.md`, the wiki, or unrelated trees unless the query missed.

This applies to subagents. The PreToolUse hook is **strict**: the first Read/Glob in a session is blocked until a `graphify query` has run (`GRAPHIFY_HOOK_STRICT=0` to disable).
