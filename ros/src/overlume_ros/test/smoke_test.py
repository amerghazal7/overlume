#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

"""Headless smoke test for overlume_ros.

Epic 0 Task 3 (docs/plans/2026-08-18-visual-mode.md), as amended
by the unified-engine migration's Task 6 (VM-095) Step 2/3:
  - initial_mode:=3 -> the node renders+publishes on its 30 Hz timer;
    assert >= 5 frames on /rendering/image within 3 s, with the configured
    encoding/dimensions.
  - initial_mode:=1 -> Scenario 2's ORIGINAL assertion here was ZERO frames
    (the two-node mux's `active_mode_ != 3` early return used to gate
    render/readback/publish on being the mux-selected renderer). Task 6
    Step 3 deleted that early return -- this node now renders and publishes
    EVERY tick regardless of render_mode_, the sole rendering authority for
    all three modes. Scenario 2 below is rewritten to assert frames DO flow
    at initial_mode:=1 too. **Frames-flow only, by design**: this scenario
    launches the node BARE (no params file, no camera topics), so the
    declare-time defaults apply and the frame is content-blank regardless
    of the shipped default -- config/default_params.yaml ships
    bowl_enabled/hybrid_enabled TRUE since the VM-095 cutover (Step 6), but
    a real bowl-content assertion needs the six-camera fixture bag; that
    coverage lives in test_mode_dispatch_pixels.py, not here.

Task 4 (VM-093):
  - Scenario 3 -- with initial_mode:=3 held fixed (this node stays the
    mux-authoritative renderer throughout, Decision 7 untouched), drive the
    LOCAL render_mode param through 1 (BOWL) -> 2 (HYBRID) -> 3 (FREE_LOOK)
    -> 1 (BOWL) via `ros2 param set` -- no message on any topic, matching
    the plan's own "no ROS message exchanged" framing for this switch --
    and assert frames of the same configured shape keep flowing with no
    crash at every step.

Task 6 (VM-095) Step 2 -- the post-cutover mode-selection surface:
  - Scenario 4: publish 1->2->3 on /rendering/set_mode against the merged
    (now-sole) node; assert render_mode_ (`~/vcam_state` index 7) follows
    each published value directly, no mux-arbitration delay or race (there
    is no `active_mode_` left to arbitrate through -- Step 3 deleted it).
  - Scenario 5 (restart case): publish a mode on /rendering/set_mode, kill
    and relaunch the node with NO `render_mode`/`initial_mode` override,
    assert it resumes the previously-published mode (delivered by the
    topic's VM-037 transient_local+reliable QoS to the relaunched node as a
    late-joiner) rather than falling back to its own declare-time default
    (1) -- the behavior micropilot_rendering_node/test/smoke_test.py's
    `test_restart_rejoins_live_mode` hardened, now this node's alone to
    keep (this task's Files list, Decision 8).

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
from std_msgs.msg import Float64MultiArray, Int32

OUT_W = 320
OUT_H = 240

INSTALL_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 "../../../install"))

class FrameCounter(Node):
    def __init__(self):
        super().__init__("smoke_frame_counter")
        self.frames = []
        self.create_subscription(Image, "/rendering/image", self._on_image, 10)

    def _on_image(self, msg: Image):
        self.frames.append(msg)

class VcamStateWatcher(Node):
    """Scenario 4/5 (Task 6/VM-095 Step 2): tracks ~/vcam_state's index 7
    (render_mode_) so a topic-driven mode switch can be asserted without
    relying on frame *content* (every mode publishes frames now, Step 3)."""

    def __init__(self):
        super().__init__("smoke_vcam_state_watcher")
        self.render_mode = None
        self.create_subscription(
            Float64MultiArray, "/overlume_node/vcam_state", self._on_state, 10)

    def _on_state(self, msg: Float64MultiArray):
        if len(msg.data) >= 8:
            self.render_mode = int(msg.data[7])

def make_mode_publisher(node: Node):
    """Scenario 4/5: a publisher on the global /rendering/set_mode topic,
    same transient_local+reliable QoS as the node's own subscription
    (VM-037 Step (a)) -- a volatile publisher would be QoS-incompatible and
    silently never deliver. Built on a node the caller already owns (and
    has already called rclpy.init() for) -- rclpy only tolerates one
    Context.init() per process, so this does not init/shutdown its own."""
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
    return node.create_publisher(
        Int32, "/rendering/set_mode",
        QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                   durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

def publish_mode_and_wait(pub, spin_node: Node, mode: int, spins: int = 10):
    """Publish `mode` repeatedly for a beat so a not-yet-fully-matched
    subscriber (this run's watcher, or a not-yet-launched relaunch node in
    Scenario 5) still catches it once discovery completes, then let the
    durable QoS hold the last value resident for any later late-joiner."""
    for _ in range(spins):
        pub.publish(Int32(data=mode))
        rclpy.spin_once(spin_node, timeout_sec=0.1)

def launch_node(initial_mode: int | None) -> subprocess.Popen:
    mode_arg = f"-p initial_mode:={initial_mode} " if initial_mode is not None else ""
    cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"source {INSTALL_DIR}/setup.bash && "
        f"ros2 run overlume_ros overlume_node "
        f"--ros-args -p out_width:={OUT_W} -p out_height:={OUT_H} "
        f"{mode_arg}"
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
           f"ros2 lifecycle set /overlume_node {transition}")
    result = subprocess.run(["bash", "-c", cmd], capture_output=True,
                            text=True, timeout=timeout)
    if result.returncode != 0:
        print(f"  [lifecycle {transition}] stderr: {result.stderr.strip()}", file=sys.stderr)
    return result.returncode == 0

def param_set(name: str, value_literal: str) -> bool:
    cmd = f"source /opt/ros/humble/setup.bash && ros2 param set /overlume_node {name} {value_literal}"
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

    print("INFO: scenario 2 -- initial_mode=1, expect frames to flow too "
          "(post-cutover: no mux early-return left to gate on) ...")
    try:
        frames = run_scenario(initial_mode=1, watch_s=2.0)
    except RuntimeError as e:
        print(f"FAIL: {e}", file=sys.stderr)
        return 1

    if len(frames) == 0:
        print("FAIL: expected frames to flow with initial_mode=1 post-cutover "
              "(render/publish is unconditional now, Step 3) -- got 0",
              file=sys.stderr)
        return 1
    print(f"INFO: got {len(frames)} frames at initial_mode=1 -- OK "
          "(bare launch: declare-defaults, no cameras -- content-blank by "
          "design; frames-flow is this scenario's whole scope).")

    print("INFO: scenario 3 -- initial_mode=3 fixed, cycling render_mode "
          "1(BOWL)->2(HYBRID)->3(FREE_LOOK)->1(BOWL) via `ros2 param set` ...")
    err = run_mode_cycle_scenario([1, 2, 3, 1])
    if err:
        print(f"FAIL: {err}", file=sys.stderr)
        return 1
    print("INFO: frames kept flowing at the configured shape through the whole "
          "render_mode cycle, no crash -- OK.")

    print("INFO: scenario 4 -- publishing 1->2->3 on /rendering/set_mode, "
          "expect render_mode_ (vcam_state[7]) to follow directly ...")
    node_proc = launch_node(initial_mode=None)
    watcher = None
    try:
        if not wait_for_start(node_proc):
            stderr = node_proc.stderr.read().decode(errors="replace")
            print(f"FAIL: node exited early (code {node_proc.returncode}):\n"
                  f"{stderr[-2000:]}", file=sys.stderr)
            return 1
        if not lifecycle("configure") or not lifecycle("activate"):
            print("FAIL: configure/activate failed.", file=sys.stderr)
            return 1
        rclpy.init()
        watcher = VcamStateWatcher()
        mode_pub = make_mode_publisher(watcher)
        for mode in (1, 2, 3):
            deadline = time.time() + 3.0
            while time.time() < deadline and watcher.render_mode != mode:
                publish_mode_and_wait(mode_pub, watcher, mode, spins=1)
            if watcher.render_mode != mode:
                print(f"FAIL: render_mode_ never followed /rendering/set_mode:={mode} "
                      f"(last seen {watcher.render_mode})", file=sys.stderr)
                return 1
        print("PASS: render_mode_ follows /rendering/set_mode directly, no mux delay.")
    finally:
        if watcher is not None:
            watcher.destroy_node()
            rclpy.shutdown()
        kill(node_proc)

    print("INFO: scenario 5 -- publish render_mode=2 on /rendering/set_mode, "
          "kill+relaunch with no initial_mode/render_mode override, expect "
          "the relaunched node to resume mode 2 (late-join on the durable "
          "publish, not its own default of 1) ...")
    node_proc = launch_node(initial_mode=None)
    watcher = None
    pub_node = None
    try:
        if not wait_for_start(node_proc):
            stderr = node_proc.stderr.read().decode(errors="replace")
            print(f"FAIL: node exited early (code {node_proc.returncode}):\n"
                  f"{stderr[-2000:]}", file=sys.stderr)
            return 1
        if not lifecycle("configure") or not lifecycle("activate"):
            print("FAIL: configure/activate failed.", file=sys.stderr)
            return 1
        rclpy.init()
        pub_node = Node("smoke_set_mode_pub_s5")
        mode_pub = make_mode_publisher(pub_node)
        publish_mode_and_wait(mode_pub, pub_node, 2, spins=15)
        time.sleep(1.0)
        kill(node_proc)

        node_proc = launch_node(initial_mode=None)
        if not wait_for_start(node_proc):
            stderr = node_proc.stderr.read().decode(errors="replace")
            print(f"FAIL: relaunched node exited early (code {node_proc.returncode}):\n"
                  f"{stderr[-2000:]}", file=sys.stderr)
            return 1
        if not lifecycle("configure") or not lifecycle("activate"):
            print("FAIL: relaunched node configure/activate failed.", file=sys.stderr)
            return 1
        watcher = VcamStateWatcher()
        deadline = time.time() + 5.0
        while time.time() < deadline and watcher.render_mode != 2:
            rclpy.spin_once(watcher, timeout_sec=0.1)
        if watcher.render_mode != 2:
            print(f"FAIL: relaunched node did not resume mode 2 (declare-time "
                  f"default is 1) -- got render_mode_={watcher.render_mode}. The "
                  f"transient_local+reliable QoS on /rendering/set_mode must be "
                  f"delivering the durable publish to a late-joining subscriber.",
                  file=sys.stderr)
            return 1
        print("PASS: relaunched node resumed the durably-published mode (2), "
              "not its own declare-time default -- restart-rejoins-live-mode holds.")
    finally:
        if watcher is not None:
            watcher.destroy_node()
        if pub_node is not None:
            pub_node.destroy_node()
        if watcher is not None or pub_node is not None:
            rclpy.shutdown()
        kill(node_proc)

    print("PASS: overlume_node mode-mux smoke test passed.")
    return 0

if __name__ == "__main__":
    sys.exit(main())
