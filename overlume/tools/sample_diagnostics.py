#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

"""Sample /overlume_node/diagnostics for `duration` seconds, print
render_ms p50/p99 and sample count. VM-091 Task 2 Step 5 perf gate.
ponytail: one-shot sampler script, not a reusable tool -- no CLI framework.
"""
import sys
import time
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 14.0
    rclpy.init()
    node = Node("diag_sampler")
    samples = []

    def cb(msg: DiagnosticArray):
        for status in msg.status:
            if status.name == "render_ms":
                for kv in status.values:
                    if kv.key == "render_ms":
                        try:
                            samples.append(float(kv.value))
                        except ValueError:
                            pass

    node.create_subscription(DiagnosticArray, "/overlume_node/diagnostics", cb, 10)
    end = time.time() + duration
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    samples.sort()
    n = len(samples)
    if n == 0:
        print("render_ms_p50=NA render_ms_p99=NA render_ms_n=0")
        return
    p50 = samples[int(0.50 * (n - 1))]
    p99 = samples[int(0.99 * (n - 1))]
    print(f"render_ms_p50={p50:.3f} render_ms_p99={p99:.3f} render_ms_n={n}")


if __name__ == "__main__":
    main()
