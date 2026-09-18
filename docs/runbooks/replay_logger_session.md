# Replaying a Micropilot logger session into Overlume

How to validate Overlume visually against a session recorded on the real
robot by the Micropilot data logger (`mp_record`), which is **not** a
rosbag2 bag: a session directory holds JPEG cameras, PCD lidar frames, CDR
message blobs, CSV transforms and a `config/` folder. `ros2 bag play`
cannot read it; the logger's own `mp_play` replays it onto ROS topics.
Written 2026-09-18 from the first real-robot session validated this way
(`session_2026-09-01_13-57-00`, 408 s, 18 topics).

## 1. Prerequisites

- The data collector workspace built from CURRENT source
  (`~/micropilot/micropilot_data_collector`, `scripts/ros_apps_build/colcon_build.sh
  --packages-select data_logger`). Asynchronous playback and the tf/tf_static
  fix landed in September 2026; an older installed `mp_play` rejects every
  frame ("No accepted synchronized frames") or publishes only the master
  topic.
- Overlume's node built (`ros/colcon_build.sh`) with the streaming backend
  (`OVERLUME_ENABLE_CESIUM=ON`) if you want to test 3D Tiles.
- `CESIUM_ION_TOKEN` exported in `~/.bashrc`. Launch the rig from a **fresh
  login shell** (`env -u CESIUM_ION_TOKEN bash -lic '…'`): a long-lived
  shell keeps a pre-rotation token in its environment and the node
  inherits it.

## 2. Player config

Copy the session's own `config/player.yaml` and change three keys:

```yaml
mode: asynchronous          # no frame matching: every topic replays on its own timeline
playback:
  start_paused: false
  loop: true
  publish_clock: true       # already true in the recorded config
sync:
  require_all_topics: false # otherwise every master frame needs all 18 topics within 50 ms,
                            # which the two latched single-message topics can never satisfy
```

Per-topic QoS is taken from the session's `config/topics.yaml`, not from
`publish.qos` in the player config — the recorder's BEST_EFFORT
subscriptions become BEST_EFFORT publishers on replay. See §4.

```bash
source ~/micropilot/micropilot_data_collector/install/setup.bash
ros2 run data_logger mp_play --session ~/session_<stamp> --player /path/to/player_loop.yaml
```

Check it: `ros2 topic hz /clock /tf /iv_points_fusion /hd_map_local_elements`.

## 3. Static transforms the recorder does not capture

The logger records `/tf_static` as ONE latched message — the first
publisher it hears (Fixposition's `FP_POI→FP_VRTK→FP_CAM`). The robot's
URDF statics from `robot_state_publisher` (`base_link→<lidar frame>`,
camera frames) are missing, so the node drops every lidar cloud
"without TF". Publish them from the session's own
`config/sensors_extrinsic_calib.yaml`:

- `frame_renames` says which calibration key applies to which frame
  (`top_lidar: seyond` → the `top_lidar.lidar_to_ego` matrix is
  `base_link→seyond`).
- **Read the whole 4×4.** On this robot `lidar_to_ego` (and
  `fixposition_to_ego`) is `diag(-1, -1, 1)`: a 180° yaw with zero
  translation. Publishing it as identity mirrors the cloud left/right.

```bash
ros2 run tf2_ros static_transform_publisher --frame-id base_link --child-frame-id seyond --yaw 3.14159265
ros2 run tf2_ros static_transform_publisher --frame-id base_link --child-frame-id FP_POI --yaw 3.14159265
```

Report to the data-collector team: `/tf_static` should be recorded from
every publisher (or the URDF statics re-published at replay).

## 4. A replay profile

Reliable subscriptions never connect to best-effort publishers, so the
marker rows (`/hd_map_local_elements`, the planner's carpet and collision
markers, the cruise debug marker) need `best_effort: true` on replay. The
shipped `replay_profile.yaml` is `urban_profile.yaml` with exactly that:

```bash
tools/validate_visual_mode.sh --live --profile replay --param gps_topic:=/fixposition/odometry_llh
```

`--param key:=value` (repeatable) forwards extra node parameters;
`--profile-dir <dir>` loads `<profile>_profile.yaml` (and
`class_inference.yaml`, which must sit next to it) from outside the
installed package. `gps_topic` is the NavSatFix source the geo anchor
samples — the simulator default `/sim/feedback/gps` never solves on a
robot, and without a solved anchor the environment source cannot be
switched ("geo-anchor not solved yet").

Run the rig with sim time, since the player owns `/clock`:

```bash
env -u CESIUM_ION_TOKEN bash -lic 'LIVE_SIM_TIME=true tools/validate_visual_mode.sh --live --profile replay --param gps_topic:=/fixposition/odometry_llh'
```

Then switch the Environment Tiles source in the GUI (or over the bridge:
`{"cmd":"set_environment_source","preset":"osm"}`); the ack carries the
node's reason when it refuses.

## 5. What this session could and could not show

Recorded (18 topics): one camera, five lidar clouds, Fixposition
odometry/NavSatFix/IMU, `/map_odometry`, `/tf`, `/tf_static`, the local
and global HD map, the planner's trajectory carpet, collision markers and
cruise debug marker. **Not recorded**, so absent from the render although
the stack still publishes them: the four `nav_msgs/Path` ribbons
(`/local_path`, `/local_vel_path`, `/navigation/global_path`,
`/behavior_path_planner/output_path_visualization`),
`/perception/dynamic_objects_list`, `/perception/dynamic_ogm`,
`/behavior_path_planner/collision_markers`, `/local_map_corners`,
`/robot/feedback/robot_speed_mps`. Add them to the recorder's topic set
for a full visual validation. `/perception/gradient_ogm` no longer exists
in the stack source — that profile row is stale.

Bowl/hybrid modes need the six-camera rig; this session has one camera.
