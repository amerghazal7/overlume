# Visual-mode docs

> **Scope note (2026-09-11):** VM-042's deployment/architecture half (two-node
> topology notes) is **deferred to post-cutover, VM-095** — the in-flight
> unified-engine migration deletes that topology, so documenting it now would
> document a dead layout. Only the docs half of VM-042 ships here: the
> profile-authoring guide and the environment-bake runbook, both below.

- [`profile_authoring.md`](profile_authoring.md) — add a topic to the
  visualization node via profile YAML only (autonomy-team audience).
- [`environment_bake.md`](environment_bake.md) — bake OSM building footprints
  into the environment-chunk format the node loads at `on_activate()`.
