# Overlume API reference {#mainpage}

Overlume is a real-time rendering library that turns a robot's live data into
a polished, human-friendly picture. This site is the generated reference for
its **public API**: the two POD-only headers every consumer includes, the
version header, and the example programs.

## Where to start

- `overlume/scene.h` — the data model. `overlume::SceneGraph` and the structs
  it points at (`EgoState`, `TrackedObject`, `MapElement`, `PathRibbon`,
  point clouds, alerts, HUD), plus the entry points that consume them
  (`set_scene()`, `render_frame()`, themes, environment sources, camera
  helpers). Everything here is plain-old-data and append-only: struct changes
  bump `overlume::kSceneVersion`, free functions bump nothing.
- `overlume/api.h` — renderer lifetime and configuration
  (`RenderConfig`, `create_renderer()`, `destroy_renderer()`).
- `overlume/version.h` — the release version as macros and a string.
- **Examples** — six self-contained programs under `examples/` in the
  repository, from a single headless frame to environment streaming;
  `examples/README.md` explains each one. They are not part of this reference.

## Beyond the reference

Narrative documentation lives in the repository, not on this site: the root
`README.md` (what Overlume is, quick start, architecture), `docs/README.md`
(index of runbooks, design documents, ADRs, plans), and `docs/status.md`
(the single status ledger). The ROS 2 node under `ros/src/overlume_ros/` is
the reference integration app and is documented there, not here.
