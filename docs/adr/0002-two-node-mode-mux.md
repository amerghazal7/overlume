# ADR-0002: Visual mode is a separate lifecycle node muxed on one stream

**Status:** Superseded by ADR-0006 (2026-09-11 — unified-engine migration
Task 6 / VM-095 cutover). Kept, not deleted, per this repo's own ADR
convention (ADR-0004's own header does the same for the language it
replaced) — the mux this ADR describes is what the cutover commit deletes.
**Re-argued in:** spec §3.1; master plan Epic 0 Tasks 3–5; WS bridge fix-ups.

## Context

Modes 1–2 are rendered by the CUDA `micropilot_rendering_node` from six cameras.
Mode 3 needs a different GPU API (GL via Filament), different inputs (autonomy
topics), and must not destabilize the shipped modes. Consumers must keep one
subscription to `/rendering/image`.

## Decision

- `micropilot_visualization_node` is a **separate process**. Both nodes subscribe
  to the global `/rendering/set_mode` (Int32 1|2|3) and publish to the same
  `/rendering/image` + `/rendering/camera_info`; exactly one publishes at a time.
  Last write wins; `initial_mode` (default 1) on both nodes settles startup.
- The vcam surface (`~/set_virtual_cam`, `~/set_look`, `~/vcam_state`) is
  re-implemented on the new node with identical semantics; the tween math is
  ported verbatim. The srv type is reused from `micropilot_rendering_node`.
- The WS bridge fans camera commands to **both** namespaces and treats one
  namespace as authoritative for `~/vcam_state` based on the last commanded mode.

## Consequences

- Crash isolation and independent deployment; the CUDA node changed only to
  idle on mode 3.
- Both nodes publish `~/vcam_state` continuously; any new consumer must apply
  the same "authoritative namespace" rule the bridge uses, or it will flicker.
- A one-frame overlap or gap at switch time is accepted by design.
- Two processes hold GPU contexts; the inactive one costs ingest CPU only.

## Revisit when

- A consumer needs zero-gap switching, **or** a third renderer appears (then a
  generic mux node owning the output topic is cheaper than N-way coordination).
