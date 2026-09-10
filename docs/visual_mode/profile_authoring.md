# Profile authoring guide

How to add a topic to the visualization node's rendered scene **using only a
profile YAML edit** — no C++ change, no rebuild of the node (a restart to
re-run `on_configure()` is enough). Everything below is grounded directly in
the loader source: `cuda/src/ros_apps/src/micropilot_visualization_node/src/profile.cpp`
(`ParseRow`/`ValidateRow`/`BuildProfile`) and
`include/micropilot_visualization_node/profile.hpp` (the `ProfileRow` struct).
Every field named here exists in `ValidateRow`; nothing here is aspirational.

> Deployment/two-node-topology notes are out of scope for this guide — see
> [`README.md`](README.md) for the VM-095 deferral.

## Where the profile lives, and how it's picked

`visualization_node.cpp`'s `on_configure()` reads two parameters:

- `profile` (default `"urban"`)
- `profile_dir` (default `""` → the package's installed `config/` share dir)

and loads `<profile_dir>/<profile>_profile.yaml`, e.g.
`cuda/src/ros_apps/src/micropilot_visualization_node/config/urban_profile.yaml`.
A row you add there takes effect on the **next** `on_configure()` (node
restart or lifecycle reconfigure) — the profile is not re-read live.

Load failure (a bad row) is **fatal**: the node logs every collected error
(not just the first) and refuses to configure. A row with an unrecognized
*extra* key is only a **warning** — logged, load still succeeds — profiles
are hand-edited by the autonomy team and a typo'd optional key must not take
the node down (`profile.hpp`'s own comment; `ParseRow`, profile.cpp:248–253).

## Row schema — every field `ValidateRow`/`ParseRow` actually reads

One row = one YAML flow-map inside the top-level `rows:` sequence. Fields,
all keys `KnownRowKeys()` recognizes (profile.cpp:93–101):

| key | required? | default | notes |
|---|---|---|---|
| `topic` | yes, except `adapter: tf_axes` | — | ROS topic name |
| `type` | yes, except `adapter: tf_axes` | — | ROS message type string, must be one `adapter` accepts (see below) |
| `adapter` | **always** | — | one of `dynamic_objects\|path\|hd_map\|ogm\|collision\|generic\|tf_axes\|point_cloud\|trajectory_carpet` |
| `role` | **always** | — | closed set **per adapter** (see table below) |
| `update_topic` | no | `""` | **`adapter: ogm` only** |
| `timeout_sec` | no | `2.0` | must be `>= 1.0` |
| `max_rate_hz` | no | `0.0` | must be `>= 0`; `0` = no limit |
| `ns_default` | no | `polyline` | `drop\|polyline\|polygon` |
| `namespaces` | no | `[]` | list of `{prefix, render, kind?}` — see below; **only consumed by `hd_map`/`dynamic_objects`/`generic` rows** (they call `classify()`/`match_rule()`); accepted-but-inert on every other adapter |
| `transient_local` | no | `false` | latched/`TRANSIENT_LOCAL` publisher |
| `best_effort` | no | `false` | `BEST_EFFORT` publisher |
| `junction_interior_boundaries` | no | `true` | **`adapter: hd_map` only** — `ParseRow` rejects it elsewhere (profile.cpp:143–151) |
| `color_mode` | no | `"auto"` | **`adapter: point_cloud` only** — rejected elsewhere (profile.cpp:155–163) |
| `max_points` | no | `0` | **`adapter: point_cloud` only** — rejected elsewhere (profile.cpp:164–172); `0` = no cap |
| `stride` | no | `1` | **`adapter: point_cloud` only** — rejected elsewhere (profile.cpp:173–181); must be `>= 1` |

Row defaults come straight from the `ProfileRow` struct
(`profile.hpp:45–89`).

### Adapters, their role sets, and their message types

`RoleSets()` and `TypeSets()` (profile.cpp:52–83) are the closed sets
`ValidateRow` checks `role`/`type` against:

| `adapter` | valid `role`(s) | valid `type` |
|---|---|---|
| `hd_map` | `lane` | `visualization_msgs/msg/MarkerArray` |
| `path` | `behavior`, `global`, `local` | `nav_msgs/msg/Path` |
| `ogm` | `dynamic_ogm`, `gradient_ogm` | `nav_msgs/msg/OccupancyGrid` |
| `collision` | `collision`, `predicted`, `merged_object`, `sweep`, `merged_ego` | `visualization_msgs/msg/MarkerArray` |
| `dynamic_objects` | `tracked` | `visualization_msgs/msg/MarkerArray` |
| `generic` | `neutral` | `visualization_msgs/msg/MarkerArray` |
| `point_cloud` | `points` | `sensor_msgs/msg/PointCloud2` |
| `trajectory_carpet` | `carpet` | `visualization_msgs/msg/MarkerArray` |
| `tf_axes` | `debug` | *(none — see below)* |

**These sets are closed.** `role: <anything else>` fails validation
(profile.cpp:285–286: `"role '...' is not valid for adapter '...'"`). Adding
a genuinely new role (a new ribbon color/z-order slot, a new severity tier,
…) is a **code change** (new entry in `RoleSets()`, plus whatever the
adapter's role→behavior switch does with it) — out of scope for a
YAML-only edit. Reusing an existing role for a new topic is exactly what a
YAML-only edit *can* do (see the worked example below).

`adapter: tf_axes` is the one exception to `topic`/`type` being required: it
is a **producer**, not a subscriber (it draws 3 colored TF axes from the
node's own `tf2::Buffer` every tick, nothing published on the wire) — its row
must **not** set `topic` or `type` at all (profile.cpp:272–276).

### Namespace rules (`hd_map` / `dynamic_objects` / `generic` only)

```yaml
namespaces: [
  {prefix: <ns-prefix>, render: drop|polyline|polygon, kind: <optional>}]
ns_default: drop|polyline|polygon   # applied when no prefix matches
```

- **Longest matching prefix wins**; no match → `ns_default` (`classify()`,
  profile.cpp:423–427 / `match_rule()`, profile.cpp:412–421).
- `prefix` must be non-empty — an empty prefix would match every namespace
  and silently override `ns_default` for everything, which `ParseRow`
  refuses to accept as anything but a typo (profile.cpp:202–211).
- `kind` (one of `other|centerline|left_boundary|right_boundary|crosswalk|
  stopline|junction|road_edge`) is legal **only** on the render verdict it
  makes sense for: `crosswalk` only on `render: polygon`, every other
  non-`other` kind only on `render: polyline` — `KindIsLegalOnRender()`,
  profile.cpp:44–49, enforced at profile.cpp:311–313. `road_surface` is
  **never** a legal YAML value — it is synthesized by `HdMapAdapter` itself
  from paired boundary rails, never present on the wire (profile.cpp:24–26).
- A duplicate `prefix` within one row's `namespaces` list is rejected
  (profile.cpp:308–310).

### QoS: `best_effort` / `transient_local`

Both are read straight through into the `rclcpp::QoS` the node builds per
row (`visualization_node.cpp`'s per-adapter subscription loops: `if
(spec.best_effort) qos.best_effort();` / `if (spec.transient_local)
qos.transient_local();`). Get this wrong and the topic connects to *nothing*,
silently, forever — no error anywhere, because ROS2 QoS mismatches simply
never negotiate a match. Two load-bearing examples already in
`urban_profile.yaml`:

- `/iv_points_fusion` (`point_cloud`) ships `best_effort: true` because the
  sensor publishes (and the recorded bag replays) `BEST_EFFORT` — a default
  `RELIABLE` subscription would receive nothing.
- The commented-out `/sim/ground_truth/boxes` debug row needs
  `best_effort: true` for the same reason (the bag's own `metadata.yaml`
  records `reliability: 2`).

Check before setting either flag on a new row:
`ros2 topic info -v <topic>` (or grep the recorded bag's `metadata.yaml`) —
never guess.

### `update_topic` (`adapter: ogm` only)

An `ogm` row optionally carries a second topic for incremental
`map_msgs/msg/OccupancyGridUpdate` patches to the base grid.
`subscriptions_for()` returns **two** `SubSpec`s for such a row (base +
update), both bound to the same `OgmAdapter` instance
(`profile.cpp:429–445`). `best_effort` propagates to both subscriptions;
`transient_local` is **only** applied to the base-grid subscription — an
update stream is inherently `VOLATILE` (each patch supersedes the last), so a
`TRANSIENT_LOCAL` subscriber on it would simply never match, silently and
permanently.

## Per-adapter row semantics (what actually happens to a message)

Condensed from each adapter's own header comment
(`include/micropilot_visualization_node/adapters/*.hpp`) — read the header
directly for the full malformed-data rules; this is the shape a new row's
author needs to reason about placement/behavior.

- **`hd_map`** — `MarkerArray` → `MapElement`. Namespace `kind` decides lane
  geometry class; `lane_id` comes from `marker.id`. `junction_interior_boundaries`
  (default `true`) controls whether `LEFT_/RIGHT_BOUNDARY` elements render
  through a junction polygon uncut; it is a no-op on a feed that never
  carries `JUNCTION`-kind geometry.
- **`path`** — `nav_msgs/Path` → one `PathRibbon` per row, keyed by `role`
  (`behavior|global|local` — **fixed** 3-slot enum, `mpviz::PathRole`,
  `scene.h:43`; each has its own ribbon color/z-lift baked into the renderer,
  not the profile). A new (valid, ≥2-pose) `Path` message **replaces** the
  stored ribbon wholesale, never merges.
- **`ogm`** — `OccupancyGrid` (+ optional `update_topic` patches) →
  `GroundGridLayer`. `role` (`dynamic_ogm`→kind 0, `gradient_ogm`→kind 1) is
  the only thing that picks the rendered kind.
- **`collision`** — `MarkerArray` (`LINE_STRIP` only) → `AlertPolygon`.
  Severity comes **only** from `role` (not namespaces): `collision`→critical,
  `predicted`/`merged_object`→warning, `sweep`/`merged_ego`→info.
- **`dynamic_objects`** — `MarkerArray` → `TrackedObject`, fused across the
  namespaces a real tracker publishes (bbox/text/arrow/hd-map-path);
  namespace `render: drop` is how a redundant namespace (e.g. a duplicate
  path-as-dots) is silently discarded.
- **`generic`** — the parity fallback: **any** `MarkerArray` topic not
  otherwise stylized. All 12 ROS `Marker::type` values map onto the
  library's 10-value `MarkerPrimitive` enum (`CUBE_LIST`/`SPHERE_LIST` fan
  out to one entry per point). This is genuinely "one YAML row, zero code" —
  the mechanism the AC for this guide is built around.
- **`point_cloud`** — `PointCloud2` → `PointCloud`. `color_mode` picks the
  per-point coloring tier (`auto|rgb|intensity|height|flat`); `stride`
  decimates (keep every Nth point, `>=1`), `max_points` caps the survivor
  count (`0` = uncapped). `flatten_z` is deliberately **not** applied here —
  real 3D height is load-bearing data for this category, unlike every other
  adapter's 2D map-plane convention.
- **`trajectory_carpet`** — `MarkerArray` (`TRIANGLE_LIST`, quads of 6
  points) → a centerline-station ribbon, stacked with the `PathRole` ribbons.
  One row ships today; a second would need its own topic, same shape.
- **`tf_axes`** — no topic/type (see above); generates 3 colored `LINE_LIST`
  axes from the live tf2 buffer every tick. Ships commented out by default
  (noisy on a stack with many frames).

## Layer visibility knobs (whole-category disable, not per-row)

Each `SceneAssembly` category — and therefore every row filed under a given
`adapter` — is gated by one boolean **node parameter**, live-tunable without
a restart (`on_params()`, `visualization_node.cpp`):

| adapter → category | parameter |
|---|---|
| `dynamic_objects` → `objects` | `layer_objects` |
| `path` → `paths` | `layer_paths` |
| `hd_map` → `map_elements` | `layer_map_elements` |
| `ogm` → `grids` | `layer_grids` |
| `collision` → `alerts` | `layer_alerts` |
| `generic` → `markers` | `layer_markers` |
| `point_cloud` → `point_clouds` | `layer_point_clouds` |
| `trajectory_carpet` → `trajectory_carpet` | `layer_trajectory_carpet` |

A new row under an existing `adapter` needs no new disable knob — it
inherits its category's existing one.

## Worked example: adding a new `path` topic end-to-end

Say a new planner node publishes `/planning/parking_maneuver_path` as a
`nav_msgs/Path`, and it should render like the existing local-path ribbon.

**1. Pick the role.** `adapter: path`'s role set is the closed
`{behavior, global, local}` (`RoleSets()`, profile.cpp:55). This topic is a
local maneuver path, so reuse `role: local` — it is *not* a new role, it is
another producer of the same rendered category (exactly like
`/local_vel_path` and `/local_path` already share `role: local` in
`urban_profile.yaml`). Inventing a brand-new role (say, `parking`) is not a
YAML-only change: `ValidateRow` would reject it at profile.cpp:285–286
unless `RoleSets()["path"]` is edited in `profile.cpp` and `RoleFromString()`
in `adapters/path.cpp` gains a matching branch — that is a code change, out
of this guide's scope.

**2. Write the row.** Append to `urban_profile.yaml`'s `rows:` list:

```yaml
  - {topic: /planning/parking_maneuver_path, type: nav_msgs/msg/Path,
     adapter: path, role: local, timeout_sec: 2.0}
```

Checked against `ValidateRow` line by line: `adapter` present and known
(profile.cpp:269–270); not `tf_axes`, so `topic`/`type` are required and
present, and `type` is `path`'s one legal type (profile.cpp:278–281);
`role` present and a member of `{behavior, global, local}`
(profile.cpp:284–286); `update_topic` empty, so the ogm-only check
(profile.cpp:288–289) doesn't apply; `timeout_sec: 2.0 >= 1.0`
(profile.cpp:291–293); `max_rate_hz` at its `0.0` default, `>= 0`
(profile.cpp:295); no `namespaces`, so nothing to check there
(profile.cpp:307–314); and no earlier row in the file already subscribes
this exact `(topic, adapter)` pair (profile.cpp:356–367).

**3. Layer gate.** No new knob needed — `layer_paths` (default `true`)
already governs every `path` row.

**4. Theme tokens.** None needed either. `role: local` reuses the ribbon
color/z-lift the renderer already has wired for `PathRole::LOCAL`
(`ribbon.cpp`'s `kRibbonZLiftByRoleM`/`theme.palette.ribbon_local`) — a new
theme token is only needed when a row wants a color no existing role
already carries, which (per step 1) means a code change, not this guide.

**5. Roll it out and confirm.** Restart (or lifecycle-reconfigure) the node.
`on_configure()` logs, per row, one of:

```
profile 'urban' loaded (N rows) from '<path>'
  /planning/parking_maneuver_path -> path/local (qos: reliable)
```

and `~/diagnostics` carries this row's `last_msg_age_sec` once the topic
starts publishing — that is the confirmation the row is live, with no rig
required beyond the node itself observing traffic on the topic.

## Validating a new row without a live rig

`load_profile()`/`load_profile_string()` are pure and `rclcpp`-free
(`profile.hpp`'s own file comment) specifically so a row can be checked with
no ROS graph at all. The node's own test suite
(`test/test_profile.cpp`, `test/test_path_adapter.cpp`, …) exercises exactly
this path; running it against an edited profile is the safe way to confirm a
new row loads and validates before wiring it into a live topic.
