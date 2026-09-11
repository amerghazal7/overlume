# ADR-0006: One node, one Filament engine — the mux is retired

**Status:** Accepted (2026-09-11, unified-engine migration Task 6 / VM-095 cutover)
**Supersedes:** ADR-0002 (two-node mode mux)

## Context

ADR-0002 split rendering across two processes — `micropilot_rendering_node`
(CUDA reprojector, modes 1–2) and `micropilot_visualization_node` (Filament,
mode 3) — muxed on a shared `/rendering/image` topic via `/rendering/set_mode`.
That design bought crash isolation and independent deployment at the cost of
two GPU contexts, a mux protocol, and two rendering code paths for what is
conceptually one camera-surround view.

The 2026-09-10 USER DECISION (this epic's charter, quoted in full at the top
of `docs/superpowers/plans/2026-09-10-unified-engine-migration.md`) asked for
the opposite tradeoff explicitly: "migrate everything implemented in the
rendering node to the new engine so we have on node one view and we can
switch the mode between all modes and everything use the same rendering
engine." Tasks 1–5 of that plan ported the bowl (mode 1) and camera-colorized
lidar (mode 2) onto the Filament `visual_renderer` engine already hosting
mode 3, behind a still-live mux and an unauthoritative local `render_mode_`
switch, so the migration could be judged by a real side-by-side against the
CUDA node's own output before anything old was touched. That judgment
happened: `docs/visual_mode/signoff.md`'s three parity rows (bowl-vs-CUDA,
hybrid-vs-CUDA, self-view/robot-proxy) are user-APPROVED, 2026-09-11.

## Decision

- `micropilot_visualization_node` is the ONLY rendering process. It renders
  all three modes (bowl, camera-colorized-lidar hybrid, full autonomy
  free-look) through the same Filament `visual_renderer` engine, switched
  locally by `render_mode_` — no second GPU context, no mux arbitration.
- `/rendering/set_mode` (`Int32`, 1|2|3) is KEPT as the wire-compatible
  control surface — every existing external consumer (the WS bridge, GUI,
  operator tooling) still publishes to it unchanged. Its handler is now a
  direct `render_mode_ = msg->data` assignment: no `active_mode_`/
  `render_mode_` split, no legacy-topic republish, no mux decision. The
  subscription KEEPS its VM-037 `transient_local + reliable` QoS — a
  restarted merged node is itself a late-joiner against the bridge's durable
  publisher, and that durability is what lets a crash/lifecycle restart
  resume the operator's last-published mode instead of the node's own
  declare-time default.
- `micropilot_rendering_node` (the ROS package: node source, launch files,
  smoke tests, `SetVirtualCam.srv`'s old home) is decommissioned outright.
  `cuda/src/libs/rendering_reprojector/` (the CUDA kernel library itself)
  is KEPT — it is a separate, still-active dependency of the Python
  prototype (`tpsprojector/app.py` and its eight `import tpscuda` pytest
  files), unaffected by this ADR; this decision is scoped to the ROS
  rendering pipeline only.
- `bowl_enabled`/`hybrid_enabled` ship `true` by default — the migration is
  complete, not merely capable.

## Consequences

This deliberately gives up every consequence ADR-0002 named, as a
user-directed tradeoff (the charter above), not an oversight:

- **Crash isolation between renderer and autonomy pipeline is gone.** A
  Filament fault now takes down the same process that ingests every
  autonomy topic, where it previously took down only the mode-3 view while
  modes 1–2 kept running on the CUDA node.
- **Independent deployment is gone.** One binary now owns all three modes;
  it can no longer be restarted/updated without interrupting whichever mode
  was active.
- **Two GPU contexts is gone** — by design, this is the point: one Filament
  context now serves every mode, at whatever GPU cost that consolidation
  carries (measured, not assumed: Task 4's co-residence gate and this
  task's own on-robot rerun both record real numbers rather than asserting
  the merge is free).

In exchange: one rendering code path per visual concept (the bowl mesh, the
lidar colorization, the autonomy scene) instead of two independently
maintained ones; one process to reason about for GPU budget; the local mode
switch is now authoritative, not a capability sitting behind a still-live
safety net.

## Revisit when

- A future requirement re-introduces the need for crash isolation between
  the renderer and the autonomy pipeline (e.g., a safety case that cannot
  tolerate a renderer fault affecting perception/planning ingestion) — at
  that point, re-splitting the process is the fix, not patching around a
  shared-fault-domain symptom.
- A third distinct rendering engine appears for a genuinely different GPU
  API — same reasoning ADR-0002 itself named for its own revisit condition,
  inherited here unchanged.
