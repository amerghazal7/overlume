# Replaying a recorded bag into Overlume

How to validate Overlume visually against a rosbag2 bag recorded on a real
robot (or a simulator): record it, replay it with the shipped `replay`
profile, supply the transforms the recording lacks, and arm an environment
source once the geo anchor solves.

## 1. Prerequisites

- Overlume's node built (`ros/colcon_build.sh`) with the streaming backend
  (`OVERLUME_ENABLE_CESIUM=ON`) if you want to test 3D Tiles.
- `CESIUM_ION_TOKEN` exported in `~/.bashrc`. `validate_visual_mode.sh`
  re-reads it from a fresh login shell before launching the node, because a
  long-lived terminal keeps a pre-rotation token in its environment and the
  node would inherit it: ion then answers 401 on every tileset and nothing
  renders (seen 2026-09-21). The script prints a `[token]` line when the
  shell's value differed from the login shell's.

## 2. Record

`tools/record_fixture_bag.sh OUT_DIR [EXTRA_TOPIC ...]` wraps `ros2 bag
record` over a baseline topic set (`/tf`, `/tf_static`, the HD map, dynamic
objects, paths, speed, GPS) plus any extra topics you name; edit its
`BASELINE` list for your stack. Or record directly:

```bash
ros2 bag record -o my_bag /tf /tf_static /your/cloud /your/navsatfix
```

Record every topic a profile row names, or that row renders nothing. A
recording captures `/tf_static` only from the publishers it heard, so check
it (section 4).

## 3. Play

```bash
ros2 bag play my_bag --clock --loop
```

Check it: `ros2 topic hz /clock /tf`. The player owns `/clock`, so run the rig
with sim time (section 5).

## 4. Static transforms the recording lacks

A bag that missed a robot's `robot_state_publisher` statics (for example
`base_link` to the lidar frame) makes the node drop every cloud "without TF".
Publish them through the rig, one `--static-tf "parent child x y z qx qy qz qw"`
per frame (repeatable):

```bash
--static-tf "base_link lidar 0 0 1.07 0 0 1 0"
```

**Read the whole 4x4** of the calibration you take them from, yaw included. A
`diag(-1, -1, 1)` rotation is a 180 degree yaw with zero translation:
publishing it as identity mirrors the cloud left/right. A cloud stamped with
one frame but published in another is relabelled by the profile row's
`frame_id`.

## 5. Run

```bash
LIVE_SIM_TIME=true tools/validate_visual_mode.sh --live --profile replay \
    --param gps_topic:=<NavSatFix topic>
```

`--param key:=value` (repeatable) forwards extra node parameters.
`gps_topic` is the NavSatFix source the geo anchor samples; the simulator
default `/sim/feedback/gps` never solves on a robot, and without a solved
anchor the environment source cannot be switched ("geo-anchor not solved yet").

The shipped `replay_profile.yaml` is `urban_profile.yaml` with every row
`best_effort: true`. Reliable subscriptions never connect to best-effort
publishers, and a replayed recording publishes best-effort topics best-effort;
a transient-local subscription never matches a volatile publisher either, so
no row is transient-local.

### Arm an environment source

Once the geo anchor solves, switch the Environment Tiles source in the GUI (or
over the bridge: `{"cmd":"set_environment_source","preset":"osm"}`); the ack
carries the node's reason when it refuses.

### Anchor height

The anchor's height (used to place streamed tiles' ground correctly, see
`docs/runbooks/cesium.md` section 5b) comes from the NavSatFix stream's
`altitude` field by default. Trim it with the `geo_anchor_height_offset_m`
param (added to the height, so an antenna h m above `base_link` needs `-h`), or
pin it outright with `geo_datum_height_m`. Compare the altitude mean with the
ground the source's tiles sample under the ego: a receiver that reads well
above the map plane needs a negative offset.

Real relief along the route (not just at the anchor) is handled by the
`environment_follow_terrain` param (default `true`, see
`docs/runbooks/cesium.md` section 5c) — leave it on for a replay of any route
with meaningful grade.

## 6. A robot integration

A robot integration keeps its own params profile, calibration and topic profile
in its own package and builds the node against the installed library
(`OVERLUME_ROS_LINK=system`, see the README). The rig reaches them with:

- `--params-profile NAME|/path/robot_params.yaml` layers a params profile over
  `default_params.yaml`;
- `--profile NAME --profile-dir <dir>` loads `<NAME>_profile.yaml` (and
  `class_inference.yaml`, which must sit next to it) from outside the
  installed package;
- `--bridge-port 8766` moves the GUI's bridge off 8765, which a stack running
  Foxglove's bridge already holds; without it the rig's preflight refuses to
  start while that bridge runs.

Run the rig on a robot from an interactive terminal: a non-interactive shell
(`ssh robot <command>`) does not read the `~/.bashrc` that exports the DDS
settings, and the node then sees none of the robot's topics.

## 7. Network-loss check without pulling a cable

Run a tiny local CONNECT proxy and point the node's curl at it; killing the
proxy is the cable pull (libcurl honours `https_proxy`; cesium-native sets no
proxy of its own):

```bash
python3 tools/kill_switch_proxy.py 8899 &   # minimal stdlib CONNECT proxy; logs no URLs or headers
env -u CESIUM_ION_TOKEN bash -lic 'https_proxy=http://127.0.0.1:8899 no_proxy=127.0.0.1,localhost LIVE_SIM_TIME=true \
  tools/validate_visual_mode.sh --live --profile replay --param gps_topic:=<NavSatFix topic> \
  --param environment_chunks_dir:=$PWD/overlume/tests/fixtures/environment_test_town_0'
# arm osm, wait for tunnels: ss -tnp | grep -c ':8899 '   then: kill <proxy pid>
```

What to expect (measured 2026-09-18): resident tiles keep rendering and the
node stays healthy; the baked fallback fires only when a NEW tile request
fails eight times in a row — so use `cache=off` on the source URI (a warm
disk cache serves everything offline) and a route long enough to leave the
loaded tiles. A network that is dead at arm time never reaches the threshold
(one failed handshake, no retry) — see `docs/status.md`'s open item on the
demand-driven trigger.
