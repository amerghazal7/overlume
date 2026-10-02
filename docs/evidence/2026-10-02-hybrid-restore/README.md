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

Measured mean-abs deltas (this run): mode 2 vs mode 1 = 15.281 (noise 5.486,
need >= 10.973); stitching hybrid vs bowl profile = 19.240 (noise 6.097, need
>= 12.195). Pre-fix the same metric was 5.3-6.4 (= noise).

Visual check: mode 2 shows blocky splat carpet over the road and a splat
outline of the pedestrian over the bowl texture. Frames are only 320x240 and
the lidar-yaw/transform registration (plan open question 1) was judged
plausible, not verified to the pixel.
