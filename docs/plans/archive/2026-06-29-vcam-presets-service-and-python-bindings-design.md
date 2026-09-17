# Virtual-cam presets + ROS service, and Python prototype on C++ bindings

Date: 2026-06-29

## Goal

1. **ROS service** on `micropilot_rendering_node` to switch the virtual camera
   between 5 presets at runtime (the YAML config pose = preset 1, plus reverse
   follow-me, left side, right side, top-down).
2. **Re-point the Python prototype** (`tpsprojector`) so its core environment
   reprojection runs through the C++ `tpscuda` bindings instead of the in-tree
   NumPy/GL renderers.

## Key geometric fact

The rig frame is x-forward, y-left, z-up (same frame the existing
`virtual_pose` lives in). `look_at(eye, target)` with `world_up=+z` and columns
`(right, down, fwd)` exactly reproduces the config pose's rotation when
`eye = t`, `target = t + fwd` (verified by hand). So **every preset is a
`(eye, target)` look-point pair**, including preset 1 derived from the YAML
pose. Eased switching is therefore a smoothstep interpolation of the look-points
with `look_at` rebuilt each frame — no rotation interpolation, no special case.

## Part A — preset service (C++)

### Interface — `srv/SetVirtualCam.srv`
```
int32 preset     # 1=config 2=reverse_follow 3=left_side 4=right_side 5=top_down
---
bool success
string active    # name of the now-active preset, or an error reason
```

### Presets (rig frame, x-fwd / y-left / z-up)
| # | name | eye | target |
|---|------|-----|--------|
| 1 | config | `t` (from YAML pose) | `t + fwd` |
| 2 | reverse_follow | `[+4, 0, 2.5]` | `[-2, 0, 0.3]` |
| 3 | left_side | `[0, +4, 2.5]` | `[0, 0, 0.5]` |
| 4 | right_side | `[0, -4, 2.5]` | `[0, 0, 0.5]` |
| 5 | top_down | `[0, 0, 8]` | `[0, 0.001, 0]` |

All presets share the configured `virtual_vfov_deg` / `K`.

### Mechanics
- Node-local `look_at(eye, target)` helper (mirrors `transforms.look_at`).
- State: `src_(eye,target)`, `dst_(eye,target)`, `cur_(eye,target)`, `tween_t_`.
  `on_configure` derives preset 1 from `virtual_pose`, sets all three to it,
  `tween_t_=1`, builds `vcam_` once.
- Service callback: validate `1 <= preset <= 5`; set `src_=cur_`,
  `dst_=preset[i]`, `tween_t_=0`; reply `success` + name. Out-of-range →
  `success=false`, view unchanged.
- 30 Hz timer: if `tween_t_ < 1`, advance by `0.033/0.5` per tick, smoothstep,
  lerp look-points, rebuild `vcam_` via `look_at`. At rest, hold `dst_`.
- Single-threaded executor (`rclcpp::spin`) → service and timer never overlap;
  no mutex (marked with a `ponytail:` note).

### Build wiring
- `CMakeLists.txt`: `rosidl_generate_interfaces(... srv/SetVirtualCam.srv ...)`
  + `rosidl_get_typesupport_target` linked into `rendering_node_lib`.
- `package.xml`: `rosidl_default_generators` (buildtool), `rosidl_default_runtime`
  (exec), member of `rosidl_interface_packages`.

## Part B — Python prototype on `tpscuda`

`Engine` holds one `tpscuda.Reprojector(width, height)`: `set_cameras` +
`upload_images` once; `upload_depth` once when mode needs depth.
`_render_env(vc)` delegates to `render_bowl` / `render_depth` / `render_hybrid`
(vcam dict `{K,R,t,width,height}`), returns `(rgb, alpha>0.5)`.

Stays in Python (no C++ equivalent): robot composite, ground-truth
`scene.render`, PSNR/SSIM/diff, sky fill, presets/tween, pygame display.

Removed from the app: the `backend='gl'` path and the GL FBO fast-path in
`main()` — CUDA is the sole env core; `main()` uses `synthesize()` →
`present_array`. The `NumpyRenderer` / `DepthRenderer` / GL renderer classes
remain in the tree as the `test_cuda_parity` reference implementations.

## Testing
- C++: extend `test/smoke_test.py` — call `/rendering/set_virtual_cam` for
  presets 1–5 and an invalid index; assert `success` / `active`.
- Python: `test_app.py` gets the `tpscuda`-unavailable skip guard (it now runs
  through CUDA). `test_cuda_parity.py` already validates CUDA≈NumPy.

## Risks
- `test_app.py` mode/coverage thresholds rely on CUDA≈NumPy parity — already
  asserted by `test_cuda_parity` (bowl PSNR>40, hybrid mask>0.90).
- rosidl-in-node-package wiring is the fiddly part; verified by a colcon build.
