# Robot Proxy Rendering (live CUDA pipeline) — Design

Date: 2026-07-06
Status: approved (approach A)

## Problem

No real camera sees the robot (they are mounted on it, looking outward), so the
live virtual view published by `micropilot_rendering_node` shows an empty hole
where the robot is. The Python prototype already composites a placeholder box
(`tpsprojector/robot.py`); the live CUDA path has nothing. The real robot mesh
is available: `M02P.obj` + `M02P.mtl` (Blender 5 export, 523k verts / ~1M tris
after triangulation, plain `Kd` materials, no textures).

## Decision

Implement robot proxy rendering **in the CUDA `rendering_reprojector` library**
(approach A), consumed by the ROS node. The Python prototype keeps its box.

Rejected: offline mesh-to-binary converter (extra step on every model swap);
GL/EGL offscreen rendering in the node (heavy dependency, breaks the lib's
no-GL philosophy).

## Model facts (verified empirically from the files)

- Units: meters. Dimensions 2.01 L × 1.48 W × 1.28 H; underside at y=0.014
  (origin already at ground level, no z-offset needed).
- Axes: obj +X = robot forward (HeadLight at x≈+0.88, TailLight at x≈−0.94),
  obj +Y = up, obj +Z = robot right (RightBlinkerLight at z≈+0.36).
  → OBJ→rig rotation: `rig = [obj_x, −obj_z, obj_y]` (det +1, right-handed).
- Faces: mostly triangles; some quads/n-gons up to 100-gon → fan-triangulate.
- 21 materials, `Kd` colors only (white body, black, rims, sirens, lights),
  zero `map_*` lines → per-vertex baked colors suffice.

## Architecture

### Library (`cuda/src/libs/rendering_reprojector`)

- New API: `Reprojector::upload_robot_mesh(const float* tri_data, std::size_t n_tris)`
  — `tri_data` is interleaved per-vertex `[x y z r g b]` (rig frame, colors
  pre-baked), 3 vertices per triangle, 18 floats per triangle. Uploaded once to
  a persistent device buffer (upload-once philosophy; grow-only like existing
  buffers). Passing `n_tris == 0` clears the mesh.
- When a mesh is present, every `render_bowl` / `render_depth` / `render_hybrid`
  composites the robot after the environment — callers do not change.
- New kernel file `src/kernels/robot_raster.cu`:
  1. clear pass: robot z-buffer (`uint64` per pixel) to far.
  2. raster pass: one thread per triangle — transform 3 verts to vcam frame
     with the existing `CameraParams` (R, t, K), reject behind-camera tris,
     compute screen bbox, edge-function coverage walk, per-pixel
     `atomicMin(uint64)` with `depth << 32 | rgba8` packing.
  3. composite pass: where robot depth < far, overwrite the env RGBA pixel
     (alpha = 1). The robot sits at the rig center and is always nearer than
     the bowl/depth environment, so unconditional overwrite is correct
     (same rule as the prototype's `robot.composite`).
- Bindings: expose `upload_robot_mesh` through the existing pybind11 module
  (numpy `(N, 18)` or flat float array in, no return).

### Node (`micropilot_rendering_node`)

- New parameters:
  - `robot_model_path` (string, default `""` = disabled → current behavior).
  - `robot_model_transform` (12 floats `[R(9 row-major) | t(3)]`, same layout
    as `camera_extrinsics`); default is the verified OBJ→rig rotation
    `[1,0,0, 0,0,-1, 0,1,0]` with zero translation. This is the calibration
    knob for a differently-exported future model.
- Loader (node-side, plain C++, no new deps, runs in `on_configure`):
  - Parse OBJ `v` / `vn` / `f` / `usemtl` / `mtllib`; fan-triangulate n-gons;
    resolve the MTL path relative to the OBJ.
  - Parse MTL `newmtl` / `Kd`. Missing or unreadable MTL → neutral gray
    (0.7, 0.7, 0.7) for all materials.
  - Bake per-vertex color = `Kd * (0.35 + 0.65 * max(0, dot(n, L)))` with a
    fixed light direction (from above-front, rig frame), using the file's
    normals; faces without normals use the triangle's geometric normal.
  - Apply `robot_model_transform`, emit the interleaved buffer, call
    `upload_robot_mesh`.
- Error handling: missing/unparsable OBJ → `RCLCPP_ERROR`, node configures and
  renders without the robot (never fatal).
- The 143 MB model stays **outside git**; the generated config's
  `robot_model_path` points at its absolute location (currently
  `/home/ag7/Downloads/M02P.obj`). `autotune_config.py` gains a
  `--robot-model` passthrough so regeneration preserves it.

## Testing

- Python test via bindings: upload a single known triangle, render with a known
  vcam, assert the covered pixels match the pinhole projection of the triangle
  and the env shows through elsewhere; assert `n_tris == 0` restores env-only
  output.
- Loader check: node-side parse of a tiny inline OBJ/MTL fixture (few faces,
  one quad to prove triangulation, one material) — assert vertex count, color,
  transform applied. Run as part of the existing smoke test if practical.
- Visual: live rig, top_down + orbit — robot visible, correct heading
  (headlights forward), wheels on the ground.

## Non-goals (YAGNI)

Textures, translucent/ghost mode, per-frame lighting, decimation, prototype
(`robot.py`) OBJ loading, tf2-driven robot pose (robot is rig-fixed at origin).
