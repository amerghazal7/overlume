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
