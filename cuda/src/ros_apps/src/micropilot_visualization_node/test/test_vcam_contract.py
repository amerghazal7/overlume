#!/usr/bin/env python3
"""vcam contract-parity test: micropilot_visualization_node vs. rendering_node.

Epic 0 Task 5 (docs/superpowers/plans/2026-08-18-visual-mode.md): the
visualization node re-implements the CUDA node's vcam surface (same
srv/topic layouts, spec §6) under its own namespace. This test launches
BOTH nodes side by side and proves the contract, not just that
visualization_node's surface exists in isolation:

  1. ~/set_virtual_cam: calling the same preset k on both nodes drives their
     ~/vcam_state eye/target to numerically identical values (tol 1e-9)
     once each node's eased tween settles.
  2. ~/set_look: publishing the same 6 floats to both nodes' ~/set_look
     produces an identical echoed ~/vcam_state layout
     ([eye xyz | target xyz | active_preset=0 | active_mode]) on both.

visualization_node's virtual_pose is launched to match what rendering_node's
default virtual_pose (R|t) reduces to as a look-point (eye=(0,-4,0),
target=(0,-5,0) — see rendering_node.cpp's default R/t and its "config"
preset derivation) so preset 1 ("config", and everything derived from it)
lines up between the two independently-configured nodes, not just the
config-independent presets (left_side/right_side/top_down).

Mirrors micropilot_rendering_node/test/smoke_test.py's subprocess + rclpy
pattern. Headless — no GUI. Hard timeout per scenario.
"""

import os
import subprocess
import sys
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

OUT_W = 160
OUT_H = 120

# visualization_node's virtual_pose (eye|target, 6 floats) chosen to match
# rendering_node's DEFAULT virtual_pose (R|t) reduced to a look-point:
#   R row0=[1,0,0] row1=[0,0,-1] row2=[0,1,0] -> forward = R col2 = (0,-1,0)
#   t = (0,-4,0) -> eye=t=(0,-4,0), target=eye+forward=(0,-5,0)
MATCHING_VIRTUAL_POSE = [0.0, -4.0, 0.0, 0.0, -5.0, 0.0]

INSTALL_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 "../../../../../install/ros_apps"))
RENDERING_LIBS = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)),
                 "../../../../../install/libs/rendering_reprojector/libs"))


def _env():
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = RENDERING_LIBS + ":" + env.get("LD_LIBRARY_PATH", "")
    return env


def launch_rendering_node() -> subprocess.Popen:
    cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"source {INSTALL_DIR}/setup.bash && "
        f"ros2 run micropilot_rendering_node rendering_node "
        f"--ros-args -p out_width:={OUT_W} -p out_height:={OUT_H} "
        f"-p initial_mode:=1"
    )
    return subprocess.Popen(["bash", "-c", cmd], env=_env(), stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, start_new_session=True)


def launch_visualization_node() -> subprocess.Popen:
    pose = ",".join(str(v) for v in MATCHING_VIRTUAL_POSE)
    cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"source {INSTALL_DIR}/setup.bash && "
        f"ros2 run micropilot_visualization_node visualization_node "
        f"--ros-args -p out_width:={OUT_W} -p out_height:={OUT_H} "
        f"-p initial_mode:=1 -p virtual_pose:=[{pose}]"
    )
    return subprocess.Popen(["bash", "-c", cmd], env=_env(), stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, start_new_session=True)


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


def wait_for_start(proc: subprocess.Popen, timeout: float = 12.0) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout:
        time.sleep(0.2)
        if proc.poll() is not None:
            return False
    return True


def lifecycle(node_name: str, transition: str, timeout: float = 15.0) -> bool:
    cmd = (f"source /opt/ros/humble/setup.bash && "
           f"ros2 lifecycle set {node_name} {transition}")
    result = subprocess.run(["bash", "-c", cmd], env=_env(), capture_output=True,
                            text=True, timeout=timeout)
    if result.returncode != 0:
        print(f"  [lifecycle {node_name} {transition}] stderr: {result.stderr.strip()}",
              file=sys.stderr)
    return result.returncode == 0


def call_set_virtual_cam(node_name: str, preset: int, timeout: float = 15.0):
    """Call <node_name>/set_virtual_cam via subprocess; return (ok, stdout)."""
    cmd = (f"source /opt/ros/humble/setup.bash && "
           f"source {INSTALL_DIR}/setup.bash && "
           f"ros2 service call {node_name}/set_virtual_cam "
           f"micropilot_rendering_node/srv/SetVirtualCam '{{preset: {preset}}}'")
    result = subprocess.run(["bash", "-c", cmd], env=_env(), capture_output=True,
                            text=True, timeout=timeout)
    out = result.stdout.strip()
    print(f"  [{node_name}/set_virtual_cam preset={preset}] "
          f"{out.splitlines()[-1] if out else ''}", file=sys.stderr)
    return result.returncode == 0, out


class StateWatcher(Node):
    """Subscribes both nodes' ~/vcam_state."""

    def __init__(self):
        super().__init__("vcam_contract_test_node")
        self.rendering_state: list | None = None
        self.viz_state: list | None = None
        self.create_subscription(
            Float64MultiArray, "/rendering_node/vcam_state", self._on_rendering, 10)
        self.create_subscription(
            Float64MultiArray, "/visualization_node/vcam_state", self._on_viz, 10)
        self.look_pub_rendering = self.create_publisher(
            Float64MultiArray, "/rendering_node/set_look", 10)
        self.look_pub_viz = self.create_publisher(
            Float64MultiArray, "/visualization_node/set_look", 10)

    def _on_rendering(self, msg: Float64MultiArray):
        self.rendering_state = list(msg.data)

    def _on_viz(self, msg: Float64MultiArray):
        self.viz_state = list(msg.data)


def wait_until(predicate, timeout: float) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return predicate()


def main() -> int:
    rendering_proc = launch_rendering_node()
    viz_proc = launch_visualization_node()
    rclpy.init()
    watcher = StateWatcher()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(watcher)
    import threading
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    success = False
    try:
        print("INFO: waiting for both nodes to start …")
        for proc, name in ((rendering_proc, "rendering_node"), (viz_proc, "visualization_node")):
            if not wait_for_start(proc):
                stderr = proc.stderr.read().decode(errors="replace")
                print(f"FAIL: {name} exited early:\n{stderr[-2000:]}", file=sys.stderr)
                return 1

        for node_name in ("/rendering_node", "/visualization_node"):
            if not lifecycle(node_name, "configure"):
                print(f"FAIL: {node_name} configure failed", file=sys.stderr)
                return 1
            if not lifecycle(node_name, "activate"):
                print(f"FAIL: {node_name} activate failed", file=sys.stderr)
                return 1

        # ── Part 1: set_virtual_cam preset parity ────────────────────────────
        # All 5 presets: with matching virtual_pose (above), preset 1 (config)
        # and everything derived from it (preset 2, reverse_follow) line up
        # too, not just the config-independent 3/4/5.
        expected_names = ["config", "reverse_follow", "left_side", "right_side", "top_down"]
        for preset, name in enumerate(expected_names, start=1):
            print(f"INFO: preset {preset} ({name}) on both nodes …")
            ok_r, out_r = call_set_virtual_cam("/rendering_node", preset)
            ok_v, out_v = call_set_virtual_cam("/visualization_node", preset)
            if not ok_r or f"active='{name}'" not in out_r:
                print(f"FAIL: rendering_node preset {preset} bad response:\n{out_r}",
                      file=sys.stderr)
                return 1
            if not ok_v or f"active='{name}'" not in out_v:
                print(f"FAIL: visualization_node preset {preset} bad response:\n{out_v}",
                      file=sys.stderr)
                return 1

            # Wait for both tweens to settle (~0.5s) and states to arrive.
            if not wait_until(lambda: watcher.rendering_state is not None
                              and watcher.viz_state is not None, timeout=5.0):
                print("FAIL: vcam_state never arrived from one or both nodes",
                      file=sys.stderr)
                return 1
            time.sleep(0.8)  # settle past the 0.5s tween

            r = watcher.rendering_state
            v = watcher.viz_state
            if r is None or v is None or len(r) != 8 or len(v) != 8:
                print(f"FAIL: preset {preset}: unexpected state shapes r={r} v={v}",
                      file=sys.stderr)
                return 1
            diffs = [abs(r[i] - v[i]) for i in range(6)]
            if any(d > 1e-9 for d in diffs):
                print(f"FAIL: preset {preset} ({name}): eye/target diverge "
                      f"rendering={r[:6]} visualization={v[:6]} diffs={diffs}",
                      file=sys.stderr)
                return 1
            print(f"INFO: preset {preset} ({name}) converged: {v[:6]} -- OK.")

        # ── Part 2: set_look echo parity ─────────────────────────────────────
        look = [1.25, -3.5, 2.0, 0.1, 0.2, 0.3]
        print(f"INFO: set_look {look} on both nodes …")
        msg = Float64MultiArray()
        msg.data = look
        watcher.rendering_state = None
        watcher.viz_state = None

        def look_applied():
            if watcher.rendering_state is None or watcher.viz_state is None:
                watcher.look_pub_rendering.publish(msg)
                watcher.look_pub_viz.publish(msg)
                return False
            r, v = watcher.rendering_state, watcher.viz_state
            return (len(r) == 8 and len(v) == 8
                    and all(abs(r[i] - look[i]) < 1e-6 for i in range(6))
                    and all(abs(v[i] - look[i]) < 1e-6 for i in range(6)))

        if not wait_until(look_applied, timeout=10.0):
            print(f"FAIL: set_look never echoed identically on both nodes; "
                  f"rendering={watcher.rendering_state} visualization={watcher.viz_state}",
                  file=sys.stderr)
            return 1
        r, v = watcher.rendering_state, watcher.viz_state
        if int(r[6]) != 0 or int(v[6]) != 0:
            print(f"FAIL: set_look should report free-look (preset=0); "
                  f"rendering[6]={r[6]} visualization[6]={v[6]}", file=sys.stderr)
            return 1
        print(f"INFO: set_look echoed identically on both nodes: {v[:6]} preset=0 -- OK.")

        print("PASS: vcam contract parity verified (presets + set_look).")
        success = True
        return 0
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        watcher.destroy_node()
        rclpy.shutdown()
        kill(rendering_proc)
        kill(viz_proc)
        if not success:
            for proc, name in ((rendering_proc, "rendering_node"), (viz_proc, "visualization_node")):
                try:
                    stderr_data = proc.stderr.read()
                    if stderr_data:
                        print(f"--- {name} stderr ---", file=sys.stderr)
                        print(stderr_data.decode(errors="replace")[-3000:], file=sys.stderr)
                except Exception:
                    pass


if __name__ == "__main__":
    sys.exit(main())
