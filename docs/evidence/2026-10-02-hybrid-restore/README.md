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

Differences from the existing golden (documented, not hidden): the golden is from the v2
fixture bag and a build that shipped an M02P ego mesh and a lower, farther free-look
camera; today's shipped defaults give the clay-box ego (`ego_model_path: ""`) and a
higher, closer default camera on the v3 bag, so framing and scene differ. Not
scene-matched; judged on registration only.

Visual check: lidar splats visible (blocky colorized points on road, sky edge and the
off-bowl structure); crosswalk stripes single, manhole and lane paint aligned, no doubled
ground.
