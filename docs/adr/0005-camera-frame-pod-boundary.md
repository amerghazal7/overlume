# ADR-0005: Camera-frame POD boundary — `set_camera_frame()`/`set_bowl_config()`

**Status:** Accepted (2026-09-10, unified-engine migration Task 1 / VM-090)
**Context:** docs/superpowers/plans/2026-09-10-unified-engine-migration.md,
Decision 2, and the Decision resolutions section (user input, 2026-09-10)

## Context

Mode 1 (bowl) migrates the CUDA reprojector's six-camera compositing onto
Filament. The camera pixels themselves must cross the clang/libc++ (library)
↔ gcc/libstdc++ (ROS node) POD boundary (ADR-0003) every tick a camera image
changes. Three options were weighed for how:

- **Option A** — a `CameraImagePlane[]` `SceneGraph` category, deep-copied by
  `set_scene()`'s existing staging buffer. Rejected: at production size (6
  cameras × 1280×720×3 ≈ 16.6 MB) this is ~1 GB/s of CPU memcpy for staging
  alone, every tick the images change, against a buffer class whose largest
  current occupant (an OGM grid) is a couple of small single-channel arrays —
  the wrong entry point for this size class.
- **Option C** — keep the CUDA reprojector, feed Filament one composited
  RGBA image. Rejected outright: it does not migrate anything and fails the
  epic's literal charter ("everything use the same rendering engine").
- **Option B** — `set_camera_frame(VisualRenderer*, uint32_t cam_idx, const
  uint8_t* rgb, uint32_t width, uint32_t height, uint64_t frame_id, ...)`, a
  POD entry point outside `SceneGraph`, pushing straight to a persistent
  `filament::Texture` the same way `ground_grid.cpp` already uploads OGM
  textures, gated by per-camera dirty tracking (an O(1) `frame_id`
  integer compare, never a content `memcmp` at this size — that would
  reintroduce exactly the cost Option A was rejected for, one layer down).
  **Chosen.**

`filament::Stream`/`Texture::setExternalStream` was also ruled out: its two
supported configurations (`Stream.h`) are both Android platform primitives
(`AHardwareBuffer` / `SurfaceTexture`), unavailable on this Linux/EGL target.

## Decision

### Copy semantics: release-callback `set_camera_frame` ships

`filament::backend::PixelBufferDescriptor` (`BufferDescriptor.h`) takes a
buffer **by reference**, plus a `Callback = void(*)(void*, size_t, void*)`
function pointer and a `void* user`, releasing the buffer via that callback
once the driver has consumed it. `set_camera_frame`'s full signature:

```c
bool set_camera_frame(VisualRenderer*, uint32_t cam_idx,
                       const uint8_t* rgb, uint32_t width, uint32_t height,
                       uint64_t frame_id,
                       void (*release)(void*, size_t, void*) = nullptr,
                       void* user = nullptr);
```

Both the function pointer and the `void*` are POD, so this crosses `api.h`/
`scene.h` cleanly, same as every other pointer/dimension tuple in this
codebase.

**Honest framing — this is one copy vs. two, not zero vs. one.** Task 2's
node-side ingest decodes every frame via `cv_bridge::toCvCopy(msg, "rgb8")` —
an unconditional full-size conversion copy into a new `cv::Mat` — which stays
on the critical path in *both* variants; the buffer handed to Filament is
that `cv::Mat`'s heap, not the SHM message's. What the release-callback shape
eliminates is only the **library-side** `setImage()` heap copy: with
`release == nullptr` (the copy-on-call default, same shape as
`ground_grid.cpp`'s own upload), `set_camera_frame` heap-copies `rgb` into a
`std::vector<uint8_t>` freed synchronously by the driver callback — a second
full-size copy on top of the conversion copy, at production size a further
~16.6–24 MB/tick of heap-copy-and-free, the same cost class Option A was
rejected for above. With `release != nullptr`, the caller's already-owned
buffer (Task 2's converted `cv::Mat`) is wrapped by reference and only freed
via `release()` once Filament has actually consumed it — one copy
(conversion) instead of two.

**Decision (user input, 2026-09-10): the release-callback shape ships.**
`release`/`user` default to `nullptr`, so every call site that passes only
the first six arguments (this task's own tests) gets the unaffected
copy-on-call behavior; Task 2 Step 6 is the first real caller expected to
supply a non-null `release` (dropping its converted `cv::Mat` once Filament's
callback fires).

**Ownership transfer through this POD boundary is unconditional.** When
`release` is non-null, `set_camera_frame` ALWAYS takes ownership of `rgb`:
`release()` is invoked exactly once on every call, whether the upload
happened, was skipped by the dirty gate (repeated `frame_id`), or was
rejected outright (null/unconfigured `r`, out-of-range `cam_idx`, or a
width/height mismatch) -- the caller must never free `rgb` itself. A
conditional release would leak Task 2 Step 6's converted `cv::Mat` on every
skipped or rejected call.

### Wire-encoding check (Decision 2's third option)

Before this decision, the plan named a conditional third option: if the six
`*/raw_images` topics were already `rgb8`, `cv_bridge::toCvShare` (no
conversion) plus capturing the `sensor_msgs::Image::SharedPtr` itself in
`release`/`user` would achieve genuine zero-per-tick-memcpy. **Checked
against this epic's fixture bag, `stack_v2_full_sensors_2026-09-09`: all six
camera topics publish `bgra8`, 1440×928 (step 5760) — NOT `rgb8`.** The
`toCvShare` zero-copy option is therefore off the table; `cv_bridge`
conversion (`bgra8` → `rgb8`) is unavoidable on the node side, and the real
decision is exactly the one-copy-vs-two framing above, not zero-vs-one.
Worst-case per-tick traffic figures scale with these real wire dims: 6 ×
1440 × 928 × 3 ≈ 24 MB/tick converted, not the plan's earlier 1280×720-derived
~16.6 MB estimate — Task 2 Step 5's perf-gate budget lines carry the
measured-at-fixture-dims figure.

### Dirty tracking: `frame_id` integer compare, not content `memcmp`

`set_camera_frame` stores the last-uploaded `frame_id` per camera slot; a
call whose `frame_id` matches the stored value is a no-op upload (O(1)
integer compare). This is deliberate: a full-content compare at production
size (6 × 1280×720×3 ≈ 16.6 MB) is exactly the ~0.5 GB/s CPU cost Option A
was rejected for, just moved one layer down. The real dirty signal is
expected to come from the node (Task 2: the ROS image-callback firing IS "a
new frame arrived"); this library-side check is a backstop against a caller
bug (a stale `frame_id` on a real call), never a content-dedup mechanism.

### Threading contract

`set_camera_frame()` and `set_bowl_config()` both end in `filament::Engine`
calls (`Texture::setImage`; texture allocation) and **must be called from the
same thread that calls `render_frame()`** — the Engine thread. This is the
same contract `scene.h`'s `set_scene()` already documents (Filament requires
all Engine calls on one thread), and holds today because the merged node
runs a single-threaded executor (`rclcpp::spin`); it must be revisited if the
executor ever changes to multi-threaded.

`release` is dispatched by Filament, not by this library, and not
necessarily on the thread that called `set_camera_frame()`. With no
`CallbackHandler` supplied, `BufferDescriptor.h:41-49` (1.56.5) documents
the `Callback` guarantee as: called on Filament's own main thread, must be
lightweight, must not call Filament APIs. The 1.56.5 SDK ships headers
only, so the actual dispatch thread is not verifiable in-repo — treat the
callback as arriving on another thread: it must be thread-safe and must not
call back into this library. A plain `delete`/`free` is safe as-is; a
shared frame pool needs its own synchronization, or a Filament
`CallbackHandler` if Task 2 wants dispatch on a thread of its choosing.

## Consequences

- `set_camera_frame`/`set_bowl_config`/`set_bowl_visible`/
  `set_camera_motion_delta` are additive `scene.h` entry points; `kMaxBowlCameras`,
  `CameraExtrinsics`, `CameraIntrinsics`, `BowlConfig` are additive POD
  structs. `kSceneVersion` bumps 4 → 5 (this task's only bump).
- Every pre-Task-2 call site (this task's own tests) passes only the first
  six arguments and gets the unaffected copy-on-call contract; Task 2's node
  wiring is the only place a real `release`/`user` pair needs to be supplied.
- This task allocates persistent per-camera RGB8 textures only; no bowl
  mesh/material exists yet — Task 2's `bowl.cpp` is the first real
  mesh/material consumer of these texture slots.
- `BowlConfig`'s own layout is still being finalized WITHIN this unreleased
  kSceneVersion 5 by Task 2 (VM-091) — it has no external consumer until
  Task 6's cutover, so a field add/reorder there during Task 2 (e.g.
  `exposure_compensation`, added at review round 1) is the type's own
  definition settling, not the "silent layout drift inside a shipped
  version" ADR-0004 exists to catch, as long as both static_assert mirrors
  (`test_scene_buffer.cpp`, `test_scene_layout.cpp`) are updated together in
  the same commit each time (they were). Any `BowlConfig` layout change
  after Task 6's cutover bumps `kSceneVersion` normally, same as every other
  struct on this boundary.

## Revisit when

- The node's executor changes from single-threaded to multi-threaded (the
  threading contract above must be re-derived, same as `set_scene()`'s own
  revisit note).
- A camera driver ever ships genuine `rgb8` on the wire (the `toCvShare` +
  `SharedPtr`-capture zero-copy path becomes viable and worth adding
  alongside the release-callback shape, not instead of it).
