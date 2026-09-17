#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

"""Periodic /tf_static re-broadcaster for SHM (iceoryx) deployments.

CycloneDDS 0.10 (ROS 2 Humble) does not deliver TRANSIENT_LOCAL history to
late-joining subscribers when the pub/sub pair matches over iceoryx shared
memory: a node started after the stack never receives the one-shot static
transforms, so every TF lookup through a static frame fails forever
("frame does not exist"). Proven live 2026-09-14: an SHM subscriber got
0/195 map<-seyond lookups while a plain-UDP subscriber got 198/198 from the
same publishers.

This relay MUST be launched with CYCLONEDDS_URI unset (network path) so it
receives the real static history, then re-broadcasts the accumulated set at
1 Hz -- fresh samples reach SHM subscribers regardless of join order. It
never invents a transform; it only repeats what the stack itself published.
"""
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


def main() -> int:
    if os.environ.get("CYCLONEDDS_URI"):
        # Wrong transport = the relay itself misses the history it exists to
        # replay. Refuse loudly rather than run as a silent no-op.
        print("tf_static_relay: CYCLONEDDS_URI is set -- launch with "
              "`env -u CYCLONEDDS_URI` so the relay uses the network path",
              file=sys.stderr)
        return 1

    rclpy.init()
    node = Node("tf_static_relay")
    qos = QoSProfile(depth=100,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    latest = {}  # (parent, child) -> TransformStamped (latest wins, tf2 semantics)

    def on_static(msg: TFMessage) -> None:
        for t in msg.transforms:
            key = (t.header.frame_id, t.child_frame_id)
            # Skip our own re-broadcasts: identical (parent, child, transform)
            # already stored -- keeps the sub->pub loop idempotent.
            prev = latest.get(key)
            if prev is not None and prev.transform == t.transform:
                continue
            latest[key] = t

    pub = node.create_publisher(TFMessage, "/tf_static", qos)
    node.create_subscription(TFMessage, "/tf_static", on_static, qos)

    def rebroadcast() -> None:
        if latest:
            pub.publish(TFMessage(transforms=list(latest.values())))

    node.create_timer(1.0, rebroadcast)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
