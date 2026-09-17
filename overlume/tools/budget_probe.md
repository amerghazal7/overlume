# GPU budget probe (Epic 0 Task 6 / VM-004)

Two tables live here. **(a) PROXY** was measured on the dev box on 2026-09-07
**(b) ON-ROBOT** closed 2026-09-11 by the VM-095 Step 1 rerun (dev-box RTX 3090 proxy; on-actual-robot rerun a named deployment item) — closes VM-043 per Decision 10.

## Why a proxy

The robot was not reachable during the review. The dev box has the same CPU
and GPU class (NVIDIA GeForce RTX 3090, 24 GB) per the user, so the numbers
bound what the robot can do **for the visualization node alone**. They do NOT
measure contention with the CUDA node's real render load: the fixture bag
carries no camera topics, so `micropilot_rendering_node` in "mode 2" was
running its pipeline with no input.

## Procedure (a) — reproducible

Script: `budget_probe.sh` (kept next to this file). **Single-process shape as
of the unified-engine migration's Task 6 (VM-095) cutover** —
`micropilot_rendering_node` no longer exists to optionally start; the
`rendering_node`/`rnode`-cased branches this procedure used to describe
(`q1_rnode_idle`/`q1_rnode_mode2`) are gone from the script (Step 5), and
this text is reworded to match rather than describing a step someone would
otherwise try to follow against a deleted package. For each case the script
1. starts `overlume_node` with `--params-file default_params.yaml`,
   `initial_mode:=3 use_sim_time:=true out_width:=1280 out_height:=720
   profile:=urban quality:=<q>`, configures + activates it (lifecycle),
2. starts `tools/tf_flatten_fixture.py` and
   `ros2 bag play $HOME/TPSProjector-fixtures/epic2_fixtures_full --loop --clock
   --qos-profile-overrides-path qos_full.yaml --remap /tf:=/tf_raw`,
3. after a 12 s warm-up samples `ros2 topic hz /rendering/image --window 100`
   (~14 s), `nvidia-smi dmon -s um -c 8` (GPU SM % and memory-controller %,
   whole GPU), and `top` CPU % of each node process,
4. tears everything down. Isolated on `ROS_DOMAIN_ID=93`.

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

## Results (b) — ON-ROBOT (closed 2026-09-11, Task 6/VM-095 Step 1)

**Closes VM-043 for real** (Decision 10) — there is only one process left to
measure: the merged `overlume_node` as the ONLY rendering process, all
three modes, real camera+lidar input from the fixture bag, `bowl_enabled`
AND `hybrid_enabled` both `true` (this is what production runs post-Step-6,
not an isolated capability check).

No robot hardware was reachable from this session — run on this dev box
(**GPU: NVIDIA GeForce RTX 3090, 24576 MiB** — named fixture gap #1, the
first time this repo records a GPU model/class) as the most
robot-representative box available, per the plan's own allowance. This row
is the dev-box proxy standing in for the robot; the on-actual-robot rerun
stays a named open item for deployment (see `deviations`/on_robot_rerun in
this pass's own report).

Harness: `on_robot_perf_gate.sh`, fixture bag `stack_v3_full_sensors_2026-09-11`
(67s, fresh single-pass playback, `--rate 1.0`, ROS_DOMAIN_ID=93), 20s warm-up
(6 cameras' CameraInfo complete + bowl bake + lidar flowing, same warm-up
`hybrid_perf_gate.sh` uses), 14s sampling window, `quality:=1`.

| Mode | image_hz | render_ms p50 | render_ms p99 | GPU SM% / mem-ctrl% | viz CPU% | Pass/fail (`>=30.0 Hz`, `<=33ms` p99) |
|---|---|---|---|---|---|---|
| BOWL (1) | 30.260 | 16.445 ms | 21.191 ms | 38 / 0 | 78.1% | **PASS** |
| HYBRID (2) | 30.237 | 15.781 ms | 21.062 ms | 33 / 0 | 82.5% | **PASS** |
| FREE_LOOK (3) | 30.296 | 11.600 ms | 13.229 ms | 28 / 0 | 66.7% | **PASS** |

All three modes clear both bars on this dev-box proxy, with real headroom
under the 33ms p99 ceiling (BOWL/HYBRID's own worst case, 21.2ms, is ~64% of
budget; FREE_LOOK sits at ~40%). `image_hz` saturates at the fixed 33ms
publish timer in every case (as table (a) already established), so the real
signal here is `render_ms` p99 and GPU SM%, both comfortably clear.

Benign warnings seen in every case's log (not a regression, both pre-exist
this pass): `camera bowl: stamp spread 0.150s exceeds max_sync_latency
0.120s` (this bag's bm/br cameras under-deliver at 67%/77% of the best
camera — the named fixture-bag recording deficit, not a rig bug, stated
here rather than chased) and `/perception/dynamic_objects_list: dropped N
malformed` (this bag's own real marker stream, unrelated to this node's
render path).

**Go decision on the 720p30 assumption:** GO — every mode clears 30Hz/33ms
on the dev-box proxy with the full production default (`bowl_enabled` AND
`hybrid_enabled` true) and real six-camera + lidar load, no resolution
reduction warranted. The on-robot rerun (real hardware, not this proxy)
remains the deployment-time confirmation, named as a deviation in this
pass's own report.

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

## Task 4 (VM-093) Step 2 — CycloneDDS SHM launch config carried forward, 2026-09-11

Verified by launching the merged node via its OWN launch file
(`ros2 launch overlume_ros overlume_node.launch.py`,
`CYCLONEDDS_URI` explicitly unset in the parent shell first) and reading the
running process's actual environment (`/proc/<pid>/environ`):
`CYCLONEDDS_URI=file:///$HOME/.config/cyclonedds/cyclonedds.xml` —
present, and byte-identical (`diff` against `rendering_node.launch.py`'s own
`SetEnvironmentVariable` call/value — the only difference is this file's own
added prose comment) to the SHM-forcing config the original fix commit
`69b5a3a` shipped. `iox-roudi` (the Iceoryx SHM daemon SHM requires) was
already running on this box throughout (`pgrep iox-roudi`).

**Honest scope note:** reproducing the ORIGINAL fix's exact measured
collapse (sim FPS 32→8 Hz) needs a live, synchronous-mode CARLA sim — the
mechanism is CARLA's own lock-step publish blocking on transport
acknowledgment, which a recorded-bag replay (this environment's only
available camera source) does not reproduce; a bag's single publisher/single
subscriber transport cost is real but small at this fixture's ~9 Hz-per-camera
publish rate regardless of SHM vs. UDP. This step's verification is therefore
scoped to "the launch file forces the identical config, and the SHM daemon
is present to use it" — a real, checkable fact — not a fresh live-CARLA
collapse repro (Named fixture gap: no live CARLA rig in this environment,
same class of gap Global Constraints already names for GPU/on-robot numbers).

## Results (d) — Task 4 (VM-093) Step 3: old node + merged node co-residence, real camera bag, 2026-09-11

**First real number for "old CUDA node + new merged node running together"
(named fixture gap #2) — this is the actual state production is in for the
whole rollout window between Task 4 and Task 6 (Decision 7).** Driven by
`tools/mode_consolidation_perf_gate.sh` (checked in alongside
`bowl_perf_gate.sh`), same fixture bag/procedure shape (`ROS_DOMAIN_ID=93`,
`SensorDataQoS`/best_effort, fresh single-pass playback, GPU confirmed quiet
via `nvidia-smi` before measuring). **Old node run from the MAIN checkout's
already-built install, READ-ONLY — this task never edits/rebuilds
`micropilot_rendering_node`;** new node run from this worktree's own scratch
colcon install. Both `initial_mode:=2` (old node authoritative for the
shared `/rendering/image` mux topic — real mode-2 CUDA reprojection, fed by
the bag's six cameras + `/iv_points_fusion`; new node's own `active_mode_==2`
early-return skips ITS render/publish loop, matching production, while
`bowl_enabled:=true` keeps its `camera_ingest_` doing real per-tick work —
"still-camera-ingesting", per the plan's own Step 3 text).

| case | image_hz (`/rendering/image`, old node's publish) | GPU SM % | GPU mem % | old-node CPU % | new-node CPU % | new-node own tick rate (diag n/14s) |
|---|---|---|---|---|---|---|
| co_residence (old node mode2 + new node bowl-ingesting) | 26.33 | 21 | 0 | 112.5 | 19 | 425 (≈30.4 Hz — healthy) |
| rnode_alone_mode2 (CONTROL, no new node running at all) | 23.05 | 6 | 0 | 0 (pid-capture artifact, see below) | – | – |

**Baseline for comparison:** Results (c)'s `bowl_off` row (new node alone,
bowl disabled, GPU SM 27%) — Task 2 Step 5a's own same-bag number, per this
task's instructions (NOT `q1_rnode_idle`/`q1_rnode_mode2` above, which carry
no camera topics).

**Finding, reported honestly rather than silently passed:** `image_hz` on
the OLD node's own `/rendering/image` publish loop falls short of the 30 Hz
bar in BOTH rows (26.33 co-resident, 23.05 alone) — the first-ever
measurement of `rendering_node`'s real mode-2 CUDA throughput against a REAL
six-camera + lidar bag (fixture gap #2: this genuinely never existed before,
even for today's shipped two-node design). **The CONTROL row (old node run
completely ALONE, no new node process at all, same bag/procedure) isolates
the cause: co-residence is NOT what's costing the rate** — the alone case is
*slower* (23.05 Hz) than the co-resident case (26.33 Hz), a difference that
sits inside this rig's own run-to-run noise (bag-playback jitter, desktop
GPU/CPU scheduling), not a systematic co-residence penalty in either
direction. This is a pre-existing property of the unmodified CUDA
`rendering_node` pipeline under real full-sensor load, out of this task's
scope to fix (Global Constraints: this epic does not touch `rendering_node`'s
CUDA code until Task 6 decommission) — named here as a review-gate finding
for Task 4 Step 1's parity checklist, not absorbed silently. GPU SM % shows
no concerning co-residence increase (21% co-resident vs. 27% Step 5a
baseline — actually lower, within noise); new-node CPU (19%, ingest-only,
not rendering) and its own 30.4 Hz internal tick rate (diagnostics sample
count 425/14s) are both healthy — Task 4's own dispatch code adds no
measurable regression.

**Known measurement gap, named rather than silently accepted -- FIXED, 2026-09-11
review round 1:** the CONTROL row's old-node CPU% originally read 0 — a
`pgrep` timing artifact (the sampled pid window landed before/after the
actual `ros2 run`-wrapped process settled), not a real reading, from a
throwaway one-off control script that has since been folded into
`tools/mode_consolidation_perf_gate.sh` itself as its `run_rnode_alone()`
case (`CASE=rnode_alone`, or `CASE=both` to run co-residence then the
control in one invocation) with a `wait_pid()` retry loop that polls until
`pgrep` finds the pid AND `top` returns a real (non-empty) sample for it,
instead of a fixed `sleep` before one `pgrep` call. The co-residence row's
112.5% (over one core, plausible for a multi-threaded CUDA pipeline) was
already the trustworthy CPU figure; the control row's CPU% is now
re-runnable and auditable from the checked-in script rather than a one-off
that produced the 0 artifact above. Not blocking this gate's own pass/fail
line, which turns on `image_hz`/GPU SM %, not old-node CPU.
