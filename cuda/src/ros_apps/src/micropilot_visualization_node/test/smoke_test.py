#!/usr/bin/env python3
"""Headless smoke test for micropilot_visualization_node.

Epic 0 Task 3 (docs/superpowers/plans/2026-08-18-visual-mode.md):
  - initial_mode:=3 -> the node renders+publishes on its 30 Hz timer;
    assert >= 5 frames on /rendering/image within 3 s, with the configured
    encoding/dimensions.
  - initial_mode:=1 -> the node stays idle (mode mux not "3"); assert ZERO
    frames arrive within 2 s.

Task 4 (VM-093):
  - Scenario 3 -- with initial_mode:=3 held fixed (this node stays the
    mux-authoritative renderer throughout, Decision 7 untouched), drive the
    LOCAL render_mode param through 1 (BOWL) -> 2 (HYBRID) -> 3 (FREE_LOOK)
    -> 1 (BOWL) via `ros2 param set` -- no message on any topic, matching
    the plan's own "no ROS message exchanged" framing for this switch --
    and assert frames of the same configured shape keep flowing with no
    crash at every step.

Mirrors micropilot_rendering_node/test/smoke_test.py's subprocess + rclpy
pattern. Headless — no GUI, no blocking loop. Hard timeout per scenario.
"""

import os
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

OUT_W = 320
OUT_H = 240

INSTALL_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 "../../../../../install/ros_apps"))


class FrameCounter(Node):
    def __init__(self):
        super().__init__("smoke_frame_counter")
        self.frames = []
        self.create_subscription(Image, "/rendering/image", self._on_image, 10)

    def _on_image(self, msg: Image):
        self.frames.append(msg)


def launch_node(initial_mode: int) -> subprocess.Popen:
    cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"source {INSTALL_DIR}/setup.bash && "
        f"ros2 run micropilot_visualization_node visualization_node "
        f"--ros-args -p out_width:={OUT_W} -p out_height:={OUT_H} "
        f"-p initial_mode:={initial_mode}"
    )
    return subprocess.Popen(
        ["bash", "-c", cmd],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        start_new_session=True)


def kill(proc: subprocess.Popen):
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


def lifecycle(transition: str, timeout: float = 15.0) -> bool:
    cmd = (f"source /opt/ros/humble/setup.bash && "
           f"ros2 lifecycle set /visualization_node {transition}")
    result = subprocess.run(["bash", "-c", cmd], capture_output=True,
                            text=True, timeout=timeout)
    if result.returncode != 0:
        print(f"  [lifecycle {transition}] stderr: {result.stderr.strip()}", file=sys.stderr)
    return result.returncode == 0


def param_set(name: str, value_literal: str) -> bool:
    cmd = f"source /opt/ros/humble/setup.bash && ros2 param set /visualization_node {name} {value_literal}"
    result = subprocess.run(["bash", "-c", cmd], capture_output=True, text=True, timeout=15.0)
    return result.returncode == 0 and "Set parameter successful" in result.stdout


def wait_for_start(proc: subprocess.Popen, timeout: float = 12.0) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout:
        time.sleep(0.2)
        if proc.poll() is not None:
            return False
    return True


def run_scenario(initial_mode: int, watch_s: float) -> list:
    """Launch, configure+activate, spin for watch_s, return received frames."""
    node_proc = launch_node(initial_mode)
    counter = None
    try:
        if not wait_for_start(node_proc):
            stderr = node_proc.stderr.read().decode(errors="replace")
            raise RuntimeError(f"node exited early (code {node_proc.returncode}):\n{stderr[-2000:]}")

        if not lifecycle("configure"):
            raise RuntimeError("configure transition failed")
        if not lifecycle("activate"):
            raise RuntimeError("activate transition failed")

        rclpy.init()
        counter = FrameCounter()
        deadline = time.time() + watch_s
        while time.time() < deadline:
            rclpy.spin_once(counter, timeout_sec=0.1)
        return counter.frames
    finally:
        if counter is not None:
            counter.destroy_node()
            rclpy.shutdown()
        kill(node_proc)


def run_mode_cycle_scenario(sequence: list, watch_s: float = 1.5) -> str:
    """Launch once with initial_mode:=3 (mux-authoritative throughout), then
    drive the LOCAL render_mode param through `sequence` via `ros2 param
    set` -- NO message on /rendering/set_mode or any other topic -- and
    check frames of the configured shape keep flowing with no crash at
    every step. Returns "" on success, else a failure message."""
    node_proc = launch_node(initial_mode=3)
    counter = None
    try:
        if not wait_for_start(node_proc):
            stderr = node_proc.stderr.read().decode(errors="replace")
            return f"node exited early (code {node_proc.returncode}):\n{stderr[-2000:]}"
        if not lifecycle("configure"):
            return "configure transition failed"
        if not lifecycle("activate"):
            return "activate transition failed"

        rclpy.init()
        counter = FrameCounter()
        for mode in sequence:
            if not param_set("render_mode", str(mode)):
                return f"ros2 param set render_mode {mode} was rejected"
            counter.frames.clear()
            deadline = time.time() + watch_s
            while time.time() < deadline:
                rclpy.spin_once(counter, timeout_sec=0.1)
            if node_proc.poll() is not None:
                return f"node crashed after render_mode:={mode}"
            if len(counter.frames) == 0:
                return f"expected frames to keep flowing after render_mode:={mode}, got 0"
            f0 = counter.frames[0]
            if f0.encoding != "rgb8" or f0.width != OUT_W or f0.height != OUT_H:
                return (f"render_mode:={mode}: expected rgb8 {OUT_W}x{OUT_H}, got "
                        f"{f0.encoding} {f0.width}x{f0.height}")
        return ""
    finally:
        if counter is not None:
            counter.destroy_node()
            rclpy.shutdown()
        kill(node_proc)


def main() -> int:
    # ── Scenario 1: initial_mode=3 -> frames flow ────────────────────────────
    print("INFO: scenario 1 -- initial_mode=3, expect >=5 frames in 3s ...")
    try:
        frames = run_scenario(initial_mode=3, watch_s=3.0)
    except RuntimeError as e:
        print(f"FAIL: {e}", file=sys.stderr)
        return 1

    if len(frames) < 5:
        print(f"FAIL: expected >=5 frames on /rendering/image within 3s, got {len(frames)}",
              file=sys.stderr)
        return 1
    f0 = frames[0]
    if f0.encoding != "rgb8" or f0.width != OUT_W or f0.height != OUT_H:
        print(f"FAIL: expected rgb8 {OUT_W}x{OUT_H}, got {f0.encoding} {f0.width}x{f0.height}",
              file=sys.stderr)
        return 1
    print(f"INFO: got {len(frames)} frames, {f0.encoding} {f0.width}x{f0.height} -- OK.")

    # ── Scenario 2: initial_mode=1 -> zero frames ────────────────────────────
    print("INFO: scenario 2 -- initial_mode=1, expect ZERO frames in 2s ...")
    try:
        frames = run_scenario(initial_mode=1, watch_s=2.0)
    except RuntimeError as e:
        print(f"FAIL: {e}", file=sys.stderr)
        return 1

    if len(frames) != 0:
        print(f"FAIL: expected 0 frames with initial_mode=1, got {len(frames)}", file=sys.stderr)
        return 1
    print("INFO: 0 frames with initial_mode=1 -- OK.")

    # ── Scenario 3 (Task 4/VM-093): local render_mode cycle, no topic ────────
    print("INFO: scenario 3 -- initial_mode=3 fixed, cycling render_mode "
          "1(BOWL)->2(HYBRID)->3(FREE_LOOK)->1(BOWL) via `ros2 param set` ...")
    err = run_mode_cycle_scenario([1, 2, 3, 1])
    if err:
        print(f"FAIL: {err}", file=sys.stderr)
        return 1
    print("INFO: frames kept flowing at the configured shape through the whole "
          "render_mode cycle, no crash -- OK.")

    print("PASS: visualization_node mode-mux smoke test passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
