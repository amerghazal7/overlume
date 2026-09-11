# GPU budget probe (Epic 0 Task 6 / VM-004)

Two tables live here. **(a) PROXY** was measured on the dev box on 2026-09-07
during the plan review. **(b) ON-ROBOT** is open and blocks VM-043 sign-off.

## Why a proxy

The robot was not reachable during the review. The dev box has the same CPU
and GPU class (NVIDIA GeForce RTX 3090, 24 GB) per the user, so the numbers
bound what the robot can do **for the visualization node alone**. They do NOT
measure contention with the CUDA node's real render load: the fixture bag
carries no camera topics, so `micropilot_rendering_node` in "mode 2" was
running its pipeline with no input.

## Procedure (a) — reproducible

Script: `budget_probe.sh` (kept next to this file). For each case it
1. starts `visualization_node` with `--params-file default_params.yaml`,
   `initial_mode:=3 use_sim_time:=true out_width:=1280 out_height:=720
   profile:=urban quality:=<q>`, configures + activates it (lifecycle),
2. optionally starts `micropilot_rendering_node` (default params) and either
   leaves it on mode 2 or publishes `/rendering/set_mode 3` so it idles,
3. starts `tools/tf_flatten_fixture.py` and
   `ros2 bag play $HOME/TPSProjector-fixtures/epic2_fixtures_full --loop --clock
   --qos-profile-overrides-path qos_full.yaml --remap /tf:=/tf_raw`,
4. after a 12 s warm-up samples `ros2 topic hz /rendering/image --window 100`
   (~14 s), `nvidia-smi dmon -s um -c 8` (GPU SM % and memory-controller %,
   whole GPU), and `top` CPU % of each node process,
5. tears everything down. Isolated on `ROS_DOMAIN_ID=93`.

The node's publish timer is a fixed 33 ms wall timer, so `image_hz` saturates
at ~30 Hz; headroom must be read from GPU SM % (fraction of the 33 ms budget
the GPU is busy) until the `render_ms` diagnostic (VM-034) exists. A case
whose `image_hz` falls below 30 is over budget.

Lessons that cost time (keep): never `pkill -f` a pattern that also appears
in the calling shell's argv (bracket trick `visualization_[n]ode` + skip `$$`);
never kill the `ros2` CLI daemon between cases — restart it once at script
start on a fresh domain instead.

## Results (a) — PROXY, dev box, 2026-09-07

GPU idle baseline before the run: SM 16 % (desktop compositor), mem 0 %.

| case | quality | rendering_node | image_hz | GPU SM % | GPU mem % | viz CPU % | rnode CPU % |
|---|---|---|---|---|---|---|---|
| q0_alone | 0 (low) | – | 30.3 | 27 | 0 | 33.5 | – |
| q1_alone | 1 (med) | – | 30.3 | 35 | 0 | 39.0 | – |
| q2_alone | 2 (high) | – | 29.8 | 29 | 0 | 41.0 | – |
| q1_rnode_idle | 1 | running, idle on mode 3 | 30.3 | 21 | 0 | 37.0 | 0.0 |
| q1_rnode_mode2 | 1 | running, mode 2, no camera input | 30.3 | 24 | 0 | 39.5 | 0.5 |

Interpretation (proxy only):

- Every preset holds the 30 Hz timer at 1280×720 alone on this GPU. GPU SM %
  sits 5–20 points above the 16 % desktop baseline; the medium-vs-high
  difference is inside the sampling noise of 8 one-second `dmon` samples, so
  the presets cannot be ranked by cost from this table. GPU memory-controller
  load is nil (readback of a 720p RGB8 frame is small).
- Node CPU is 33–41 % of one core at 30 Hz: ingest + `set_scene` deep copy +
  synchronous readback + `sensor_msgs/Image` publish. This is the part that
  will scale with scene size, not the GPU.
- Co-residence with the CUDA node is **unmeasured**: without camera topics the
  CUDA node does no work (0–0.5 % CPU, no GPU delta). The two rnode rows only
  prove the mux keeps the visualization node's rate intact while the other
  node exists.
- Decision: keep the shipped default `quality: 1` (medium). `high` is viable
  alone but its margin under real contention is unknown; promote it only if
  the on-robot table (b) shows headroom. No resolution reduction is warranted
  by the proxy.

## Results (b) — ON-ROBOT (open; VM-043 blocker)

Same matrix on robot hardware with perception + cameras feeding the CUDA node,
plus `render_ms` p50/p99 from the VM-034 diagnostic, CUDA-node fps delta and
perception fps delta. Record the go/adjust decision on the 720p30 assumption
in the master plan's "Epic 0 results" line.

## Results (c) — VM-091 Task 2 Step 5: bowl camera-ingest gate, real camera bag, 2026-09-11

**These rows are their own baseline, not comparable to (a)/(b) above.** (a)'s
`q1_alone`/`q1_rnode_idle`/`q1_rnode_mode2` were all measured on
`epic2_fixtures_full`, which carries **no camera topics at all** — a bowl
that needs six live camera images to render anything cannot be gated against
a camera-starved number. This table uses this epic's own fixture bag,
`~/TPSProjector-fixtures/stack_v2_full_sensors_2026-09-09` (six real
`bgra8` 1440×928 cameras + `camera_info`, `ROS_DOMAIN_ID=93`, `SensorDataQoS`/
best_effort sensor subs, fresh single-pass playback — no `--loop`), driven by
`tools/bowl_perf_gate.sh` + `tools/sample_diagnostics.py` (checked in
alongside `budget_probe.sh`; same procedure shape, `render_ms` read directly
from the VM-034 `~/diagnostics` topic instead of estimated). 1280×720, quality 1
(medium), `initial_mode:=3`, `~29s` of steady playback sampled per case
(15s warm-up + 14s `topic hz`/`nvidia-smi dmon`/diagnostics window).

| case | bowl_enabled | image_hz | GPU SM % | GPU mem % | viz CPU % | render_ms p50 | render_ms p99 | n |
|---|---|---|---|---|---|---|---|---|
| bowl_off (5a baseline) | false | 30.30 | 27 | 0 | 45.5 | 10.505 | 11.694 | 425 |
| bowl_on | true | 30.28 | 34 | 0 | 85.0 | 16.820 | 22.358 | 425 |
| bowl_on_driving (synthetic-odometry) | true | 30.26 | 36 | 0 | 78.5 | 17.048 | 21.689 | 425 |

**Pass/fail:** `image_hz >= 30.0` holds in all three cases (30.30, 30.28,
30.26 — the fixed 33ms publish timer, same rule as table (a)). `render_ms`
p99 stays well under the 33ms ceiling in every case (11.694ms bowl-off,
22.358ms bowl-on, 21.689ms bowl-on-driving). GPU SM % delta over the
bowl-off baseline is +7 points (27→34) bowl-on, +9 points (27→36)
bowl-on-driving; viz CPU % delta is +39.5 points (45.5%→85.0%, one core)
bowl-on, +33 points (45.5%→78.5%) bowl-on-driving — real and worth tracking
as this epic's own per-fragment-sampling cost grows (more cameras dirty per
tick, higher tessellation), but not over budget today. **No bar is missed;
nothing here is absorbed silently.**

**`bowl_on_driving` (VM-091 gate close-out finding 6) — measured 2026-09-11
with a SYNTHETIC odometry publisher, labeled as such:** `stack_v2_full_
sensors_2026-09-09` carries **no odometry topic at all** (`ros2 bag info`
lists no `nav_msgs/msg/Odometry` topic), so `camera_ingest_`'s twist buffer
stayed empty for every other row in this table regardless of `odom_topic`
— `compensation_delta_4x4()` returns the identity matrix for every camera,
every tick, on those rows. This row instead runs `tools/
synthetic_odom_publisher.py` (committed alongside this script, ~20 lines) at
50 Hz on `/synthetic/odom` with constant `vx=2.0 m/s`, `wz=0.1 rad/s`,
`use_sim_time` honored (subscribes `/clock` so its stamps land in the same
sim-time base as the bag's camera images) — this is what makes
`rig_delta()`'s per-tick Euler integration and a genuinely non-identity
`compensation_delta_4x4()` matrix actually run and upload, not the
identity-matrix fallback every other row measures. Result: **no measurable
regression vs. `bowl_on`** (render_ms p50 17.048 vs 16.820, p99 21.689 vs
22.358 — within this rig's own run-to-run noise; GPU SM% +2, viz CPU% -6.5)
— the per-tick `rig_delta()` integration cost is small next to the
per-fragment sampling cost the `bowl_on` row already measures, at this bag's
own camera dirty rate. **Worst-case caveat, stated rather than assumed
away:** this bag's six cameras publish at **~9 Hz each** (not the node's
30 Hz render tick), so most ticks see only 0–2 cameras dirty (`set_camera_
frame()` calls), not all 6 simultaneously-dirty-and-driving at once — the
theoretical worst case named in the plan's own Step 5 text (~6×1440×928×3 ≈
24 MB/tick, Decision resolution 1's measured wire dims) is NOT what this row
measures; it measures this fixture's real (sparser) dirty-camera cadence
under real per-tick ego-motion compensation instead.

**Upload/conversion bandwidth, at the REAL wire dims (not the plan's
1280×720-derived ~16.6 MB estimate):**
- Wire encoding: `bgra8`, 1440×928, confirmed via this bag's own
  `camera_info`/`raw_images` — 4 bytes/px × 1440 × 928 × 6 cameras ≈
  **32.07 MB/tick** of DDS `sensor_msgs/Image` payload (received, not a CPU
  memcpy line item — cv_bridge's `toCvCopy` is the first place this repo's
  own code touches the buffer).
- `cv_bridge::toCvCopy(msg, "rgb8")` conversion copy (bgra8→rgb8, dropping
  the alpha channel and reordering B/R): 1440 × 928 × 3 bytes/px × 6 cameras
  ≈ **24.05 MB/tick** worst case (all six cameras dirty the same tick) — this
  is the figure the finding names (`~24 MB/tick`), not the plan's
  1280×720-derived ~16.6 MB estimate. This copy is unavoidable in every
  variant (Decision 2's honest one-copy-vs-two framing) and is on the
  `render_ms` critical path in the image callback (gated entirely behind
  `bowl_enabled_`, so it costs nothing in the `bowl_off` row above).
- **Release-callback `set_camera_frame` shipped (ADR-0005, Decision
  resolution 1)** — `camera_ingest.cpp`'s image callback hands Filament the
  SAME `cv_bridge`-converted buffer directly (a heap-allocated
  `cv_bridge::CvImagePtr` copy is just a `shared_ptr` refcount bump, freed by
  Filament's release callback once consumed), so the ~24.05 MB/tick figure
  above is the ONLY copy on this path — there is no second ~24 MB/tick
  library-side `setImage()` heap-copy-and-free line item to add (that
  second copy is exactly what the release-callback shape was chosen to
  avoid, Task 1/ADR-0005).
- GPU-side `setImage()` upload traffic is the same ~24.05 MB/tick figure
  (the converted RGB8 buffer is what's actually uploaded) — already
  reflected in the GPU SM %/mem % columns above (mem % reads 0 at this
  scale; six 1440×928×3 uploads/tick is small next to this GPU's memory
  bandwidth).
