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
