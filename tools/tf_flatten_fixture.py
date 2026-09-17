#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

"""Fixture-bag TF flattener (user directive 2026-08-19: always ignore
elevation in fixture-bag validation — playback-side only, node untouched).

Play the bag with `--remap /tf:=/tf_raw`; this relays /tf_raw -> /tf with
translation.z zeroed on every map->base_link transform. /tf_static and all
other transforms pass through unchanged.

Run (ROS sourced):
    python3 tools/tf_flatten_fixture.py
"""
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class TfFlatten(Node):
    def __init__(self):
        super().__init__("tf_flatten")
        self._pub = self.create_publisher(TFMessage, "/tf", 100)
        self.create_subscription(TFMessage, "/tf_raw", self._relay, 100)

    def _relay(self, msg: TFMessage):
        for t in msg.transforms:
            if t.header.frame_id == "map" and t.child_frame_id == "base_link":
                t.transform.translation.z = 0.0
        self._pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(TfFlatten())


if __name__ == "__main__":
    main()
