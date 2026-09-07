# ADR-0004: Scene interface is additive-only and versioned (replaces freeze-then-lift)

**Status:** Accepted (2026-09-07 plan review)
**Supersedes:** the "frozen after Epic N" / "Epic 3 freeze lift" language in the
master plan, Epic 1 and Epic 2 plans, backlog VM-035/VM-036, and the top comment
of `scene.h`.

## Context

Epics 0–2 declared the public headers frozen at each review gate, then added
entry points anyway as "legal additive" (`theme_assets_loaded`, `theme_parses`,
`set_object_model_dir`) and scheduled a formal "freeze lift" in Epic 3 for
`PointCloud` and `MapElement.kind`. The freeze forced one real spec deviation
(HD-map layer pops instead of fading because `MapElement` has no
`last_update_sec`). Behavior in practice was already additive-only.

## Decision

- Public headers are **additive-only**: new fields are appended to structs, new
  enum values are appended, new entry points are added; nothing is renamed,
  reordered or removed within a major version.
- `scene.h` carries `constexpr uint32_t kSceneVersion` bumped on every change.
  The `sizeof`/`offsetof` `static_assert` tables that already exist in
  `tests/test_scene_buffer.cpp` (Epic 1 Task 1) stay the layout guard; a node-side
  gtest mirrors them so the gcc build fails loudly on a mismatch too.
- Goldens and adapter tests must stay green across additive changes; a change
  that needs a golden re-promotion is a rendering change, not an interface
  change, and is reviewed as such.
- Any epic may add fields when it needs them. "Freeze lift" ceases to exist as a
  step.

## Consequences

- `MapElement.last_update_sec` and `MapElement.kind` can land when needed, and
  the HD-map fade deviation closes.
- The node and the library must be rebuilt together on a version bump (they are
  already built by one script; the static_asserts make a mismatch loud).
- Deprecated fields accumulate until a deliberate major bump.

## Revisit when

- A field must change meaning or type (then plan a major bump with both sides
  rebuilt and goldens re-promoted in one commit).
