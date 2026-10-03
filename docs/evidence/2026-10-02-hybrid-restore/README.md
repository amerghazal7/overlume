# Hybrid restore: candidate capture (2026-10-02)

Candidates for human promotion over
`overlume/tests/goldens/hybrid_test_town_merged_node.png`. **Not promoted.**

Command (GPU box, ROS sourced):

    python3 ros/src/overlume_ros/test/test_mode_dispatch_pixels.py \
        --dump-dir docs/evidence/2026-10-02-hybrid-restore

Bag: `~/overlume-fixtures/stack_v3_full_sensors_2026-09-11`. Code: branch
`fix/hybrid-composite` at d690a14 (plus this docs commit). Frames are
averaged capture windows; the bag keeps moving between them.

| file | content |
|---|---|
| `mode1_bowl.png` | mode 1 (bowl) |
| `mode2_hybrid.png` | mode 2 (hybrid) |
| `stitch_bowl.png` | FREE_LOOK + Surround Stitching, `bowl` profile |
| `stitch_hybrid.png` | FREE_LOOK + Surround Stitching, `hybrid` profile |

Measured mean-abs deltas (this run, after the registration fix): mode 2 vs
mode 1 = 13.089 (noise 5.223, need >= 10.447); stitching hybrid vs bowl
profile = 16.344 (noise 6.218, need >= 12.436). Pre-fix the same metric was
5.3-6.4 (= noise).

Registration: the first candidates (shipped `pointcloud_transform` tz=1.15, the
m2o1 value) showed doubled crosswalk stripes, because the fixture bag's lidar
sits 2.4 m above base_link and the splat ground landed 1.28 m below the bowl
floor. `default_params.yaml` now ships tz=2.444 (tf_static 2.4 + the 0.044
rig offset the camera extrinsics carry); manhole shift mode 2 vs mode 1 went
from 20.8 px to under 1 px at 640x480, and the stripes are single in
`mode2_hybrid.png`. Guard: `test_default_params_hybrid.py` (ground z of the
bag cloud through the shipped transform must be within 0.15 m of 0). These
files replace the earlier misregistered candidates; still not promoted.

## 1280x720 candidate (2026-10-03)

`hybrid_test_town_merged_node_1280x720_candidate.png` is a 1280x720 candidate for the
golden; `hybrid_golden_vs_candidate.png` is the side-by-side (left: current golden,
right: candidate). **Not promoted.** Code: main at 78baa7f (set_hybrid_splats restored,
`pointcloud_transform` tz=2.444, `pointcloud_topic` /iv_points_fusion).

Method: the runbook recipe (hybrid-golden-vm094.md, 2026-09-11 re-shot section):
`ROS_DOMAIN_ID=93`, single pass, `--start-offset 0`, frame grabbed ~40 s after
launch (single published `/rendering/image` frame, rgb8, no averaging).

    ros2 run overlume_ros overlume_node --ros-args \
      --params-file <share>/config/default_params.yaml -p initial_mode:=3 -p render_mode:=2 \
      -p use_sim_time:=true -p profile:=urban -p bowl_enabled:=true -p hybrid_enabled:=true \
      -p pointcloud_topic:=/iv_points_fusion          # then lifecycle configure + activate
    python3 tools/tf_flatten_fixture.py &
    ros2 bag play ~/overlume-fixtures/stack_v3_full_sensors_2026-09-11 --clock \
      --start-offset 0 --qos-profile-overrides-path ~/overlume-fixtures/qos_full.yaml \
      --remap /tf:=/tf_raw

Frame grab: a ~10-line rclpy subscriber saving one `/rendering/image` frame (the runbooks
name no grab tool). Log at capture: `hybrid: colorized 83725/163632 lidar points (51.2%)`.

Re-captured 2026-10-03 with the M02P ego (see below); sha256 prefix of the candidate PNG
`3275158d29195c35`. Log at capture: `hybrid: colorized 84217/160658 lidar points (52.4%)`.

Correction: the first 1280x720 candidate showed a giant clay box because the proprietary,
git-ignored M02P mesh (`ros/src/overlume_ros/assets/ego/ATTRIBUTION.md`) was not in this
worktree. It is now symlinked from the main checkout (still untracked) and `colcon_build.sh
overlume_ros` re-installed it. Evidence it is in use: `overlume_node.cpp:172` resolves an empty
`ego_model_path` to `share/overlume_ros/assets/ego/M02P.glb`, that 75 MB mesh is present under
`ros/install/`, the log has no clay-box fallback line, and the M02P silhouette is in the frame.

Camera: no override needed. The golden-era node (362cb89,
`cuda/src/ros_apps/src/micropilot_visualization_node/config/default_params.yaml`) and today's
`default_params.yaml` ship the same `virtual_pose` [-4,0,3.5 | 2,0,-0.5] and vfov 80; the
"lower, farther" look was the 4.5 m clay box vs. the small M02P. With the M02P the ego
occupies the same pixels as in the golden (x 570-715, y 320-480).

Hybrid ON vs OFF (same bag offset, identical params except `hybrid_enabled`): mean-abs
delta 12.975 (8-bit; OFF has no colorize log lines). Crosswalk stripes are single and
aligned between ON and OFF, lane paint and manhole coincide; ON adds blocky splats on the
sky edge, off-bowl structure and the passing car. The long smeared streak at the upper left is
that car's camera projection on the bowl wall (the golden has the same kind of smear from its
black car); it is not a registration defect.

Residual differences from the golden: different scene (the golden is an earlier capture
with a red car and a black police car at the intersection; this is the v3 bag at
~40 s, ego on the crosswalk) and the splat rendering (opaque stencil splats, new since the
golden). Not scene-matched; judged on ego, framing and registration.

The CUDA-era `*_cuda_reference.png` goldens were retired 2026-10-03; the side-by-side above compares against the merged-node golden only.
