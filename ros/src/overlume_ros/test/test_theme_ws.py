#!/usr/bin/env python3
"""WS-driven `set_theme` end-to-end test: overlume_node + vcam_ws_bridge.

Epic 1 Task 3 (VM-014, docs/superpowers/plans/2026-08-18-visual-mode-epic1.md
Task 3 Step 12): proves the node-clock wiring (Step 9) end to end, not just
that overlume::set_theme() blends correctly in a unit test (test_theme_transition
.cpp already covers that with a driven, deterministic clock). Specifically:
Step 9's `sim_clock_sec_` advance + `set_scene()` call is what lets a running
node's theme transition ever finish -- without it, this whole test would
observe frames that never change after the WS command, even though the
library-level blend math is correct.

  1. Launch overlume_node (initial_mode:=3, so it renders+publishes
     immediately once activated -- no mux/rendering_node needed, `set_theme`
     is a visual-mode-only concept) + vcam_ws_bridge.py, both real
     subprocesses (mirrors tools/test_vcam_ws_bridge.py's
     test_bridge_e2e_mode3_orbit_and_frames "real bridge process" E2E
     pattern, Epic 0 Task 5).
  2. Capture a frame from /rendering/image BEFORE sending set_theme.
  3. Send {"cmd": "set_theme", "theme": "light_clay"} over the WS bridge.
  4. Keep collecting /rendering/image frames past the default 0.8s
     transition_sec (+ margin), tracking inter-frame gaps.
  5. Assert BOTH:
       (a) no gap between consecutive /rendering/image frames wider than a
           few timer ticks (33ms each) across the whole observation window,
           including the switch -- a broken set_scene()/render_frame() wire
           would still pass this trivially (frame cadence is independent of
           whether the theme clock ever advances), which is exactly why (b)
           below is the one that actually exercises Step 9's wiring.
       (b) the last frame differs from the first by more than a small
           noise-floor threshold (mean absolute per-channel pixel
           difference) -- this is what a broken Step 9 wire (SceneBuffer::
           active().sim_time_sec stuck at 0 forever) would fail: the theme
           would never visibly finish blending outside a unit test that
           drives set_scene() directly.

Run (ROS + this repo's ros install sourced first):
    source /opt/ros/humble/setup.bash
    source ros/install/setup.bash
    python3 ros/src/overlume_ros/test/test_theme_ws.py

Skips cleanly (prints SKIP, exit 0) when this repo's ROS install isn't
present -- same spirit as the GL tests skipping without a GPU: this test
needs colcon_build.sh to have run first.
"""

import asyncio
import json
import os
import subprocess
import sys
import time

OUT_W, OUT_H = 160, 120
# Default set_theme() transition_sec (overlume::set_theme's 0.8s default, see
# scene.h) + margin for the node's 33ms tick + GPU scheduling jitter (may
# be sharing the box with the live CARLA sim -- never assume an idle GPU).
TRANSITION_SEC = 0.8
WAIT_MARGIN_SEC = 2.0
# A single timer tick is 33ms; the AC is "no frame drop > 1 tick" (~66ms at
# 30Hz). 0.15s (~4-5 ticks) keeps GPU-contention headroom (last measured max
# gap on this dev box was 0.039s) while still failing on a real per-frame
# stall -- 1.0s was too loose to catch anything but a multi-second wedge.
MAX_FRAME_GAP_SEC = 0.15

# 4 levels up from this file is the repo root (see smoke_test.py's
# 3-levels-up to INSTALL_DIR for the analogous offset).
REPO_ROOT = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", ".."))
INSTALL_DIR = os.path.join(REPO_ROOT, "ros", "install")
BRIDGE_SCRIPT = os.path.join(REPO_ROOT, "tools", "vcam_ws_bridge.py")
WS_PORT = 18766  # fixed test port; distinct from the default 8765 and from
                  # tools/test_vcam_ws_bridge.py's own 18765


def _popen(cmd: str) -> subprocess.Popen:
    return subprocess.Popen(["bash", "-c", cmd],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                            start_new_session=True)


def _kill(proc: subprocess.Popen):
    try:
        os.killpg(os.getpgid(proc.pid), 15)
    except ProcessLookupError:
        pass
    try:
        proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(proc.pid), 9)
        except ProcessLookupError:
            pass
        proc.wait()


def _lifecycle(node_name: str, transition: str, timeout: float = 15.0) -> bool:
    cmd = f"source /opt/ros/humble/setup.bash && ros2 lifecycle set {node_name} {transition}"
    result = subprocess.run(["bash", "-c", cmd], capture_output=True, text=True, timeout=timeout)
    if result.returncode != 0:
        print(f"  [lifecycle {transition}] stderr: {result.stderr.strip()}", file=sys.stderr)
    return result.returncode == 0


def _wait_running(proc: subprocess.Popen, timeout: float = 12.0) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout:
        time.sleep(0.2)
        if proc.poll() is not None:
            return False
    return True


def mean_abs_diff(a: bytes, b: bytes) -> float:
    n = min(len(a), len(b))
    return sum(abs(a[i] - b[i]) for i in range(n)) / n if n else 0.0


def main() -> int:
    if not os.path.isdir(INSTALL_DIR):
        print("SKIP: ros/install not built -- run colcon_build.sh first.")
        return 0

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    import websockets

    viz_cmd = (
        f"source /opt/ros/humble/setup.bash && source {INSTALL_DIR}/setup.bash && "
        f"ros2 run overlume_ros overlume_node --ros-args "
        f"-p out_width:={OUT_W} -p out_height:={OUT_H} -p initial_mode:=3")
    bridge_cmd = (
        f"source /opt/ros/humble/setup.bash && source {INSTALL_DIR}/setup.bash && "
        f"python3 {BRIDGE_SCRIPT} --port {WS_PORT}")

    viz_proc = _popen(viz_cmd)
    bridge_proc = None
    rclpy.init()

    class FrameRecorder(Node):
        def __init__(self):
            super().__init__("theme_ws_frame_recorder")
            self.frames: list[tuple[float, bytes]] = []  # (recv_time, rgb bytes)
            self.create_subscription(Image, "/rendering/image", self._on_image, 10)

        def _on_image(self, msg: Image):
            self.frames.append((time.time(), bytes(msg.data)))

    recorder = FrameRecorder()
    try:
        if not _wait_running(viz_proc):
            stderr = viz_proc.stderr.read().decode(errors="replace")
            print(f"FAIL: overlume_node exited early:\n{stderr[-2000:]}", file=sys.stderr)
            return 1
        if not _lifecycle("/overlume_node", "configure"):
            print("FAIL: configure transition failed", file=sys.stderr)
            return 1
        if not _lifecycle("/overlume_node", "activate"):
            print("FAIL: activate transition failed", file=sys.stderr)
            return 1

        bridge_proc = _popen(bridge_cmd)

        async def run():
            uri = f"ws://127.0.0.1:{WS_PORT}"
            deadline = time.time() + 12.0
            ws = None
            last_err = None
            while time.time() < deadline and ws is None:
                try:
                    ws = await websockets.connect(uri, open_timeout=1.0)
                except OSError as e:
                    last_err = e
                    await asyncio.sleep(0.3)
            if ws is None:
                raise RuntimeError(f"could not connect to bridge: {last_err}")

            try:
                # Drain frames for a moment so we have a real "before" frame
                # rendered under the node's initial theme (dark_adas, the
                # shipped default).
                t0 = time.time()
                while time.time() - t0 < 1.0:
                    rclpy.spin_once(recorder, timeout_sec=0.05)
                if not recorder.frames:
                    raise RuntimeError("no frames received before sending set_theme")
                before_frame = recorder.frames[-1][1]

                await ws.send(json.dumps({"cmd": "set_theme", "theme": "light_clay"}))

                # Keep collecting frames across the whole transition window.
                t0 = time.time()
                while time.time() - t0 < TRANSITION_SEC + WAIT_MARGIN_SEC:
                    rclpy.spin_once(recorder, timeout_sec=0.05)
                if len(recorder.frames) < 2:
                    raise RuntimeError(
                        f"too few frames after set_theme ({len(recorder.frames)}) to "
                        "check cadence/before-after difference")
                after_frame = recorder.frames[-1][1]
                return before_frame, after_frame
            finally:
                await ws.close()

        before_frame, after_frame = asyncio.run(run())

        # (a) cadence: no gap between consecutive frames wider than
        # MAX_FRAME_GAP_SEC across the whole observation window, including
        # the moment the WS command landed.
        times = [t for t, _ in recorder.frames]
        gaps = [b - a for a, b in zip(times, times[1:])]
        max_gap = max(gaps) if gaps else 0.0
        if max_gap > MAX_FRAME_GAP_SEC:
            print(f"FAIL: /rendering/image had a {max_gap:.3f}s gap across the "
                  f"set_theme switch (> {MAX_FRAME_GAP_SEC}s) -- frames stalled.",
                  file=sys.stderr)
            return 1
        print(f"INFO: max inter-frame gap {max_gap:.3f}s across {len(recorder.frames)} "
              f"frames -- OK.")

        # (b) the theme actually finished blending: last frame must differ
        # visibly from the pre-switch frame. dark_adas vs. light_clay differ
        # by ~100+ mean luminance levels (tests/test_theme.cpp's own FrameStats
        # guards) -- 10.0 is a generous noise floor, nowhere near that gap,
        # so this only fails if the theme never actually changed on screen.
        diff = mean_abs_diff(before_frame, after_frame)
        if diff <= 10.0:
            print(f"FAIL: before/after set_theme frames are ~identical (mean abs "
                  f"diff {diff:.2f}) -- the node's sim clock likely never advanced "
                  f"(Step 9 set_scene() wiring), so the transition never finished.",
                  file=sys.stderr)
            return 1
        print(f"INFO: before/after mean abs pixel diff {diff:.2f} -- OK.")

        print("PASS: overlume_node WS set_theme end-to-end test passed.")
        return 0
    except RuntimeError as e:
        print(f"FAIL: {e}", file=sys.stderr)
        return 1
    finally:
        recorder.destroy_node()
        rclpy.shutdown()
        if bridge_proc is not None:
            _kill(bridge_proc)
        _kill(viz_proc)


if __name__ == "__main__":
    sys.exit(main())
