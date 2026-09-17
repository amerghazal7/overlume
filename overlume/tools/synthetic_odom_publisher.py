#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Amer Ghazal

"""Publish constant-vx/wz nav_msgs/Odometry at 50Hz. VM-091 Task 2 Step 5
perf gate (finding 6): stack_v3_full_sensors_2026-09-11 carries no odometry
topic at all, so camera_ingest_'s rig_delta()/compensation_delta_4x4() never
run their real integration path without this -- this stand-in makes the
"bowl_on_driving" perf row exercise that per-tick math instead of the
identity-matrix no-odometry fallback every other row measures.
ponytail: one-shot rig-gate helper, not a reusable tool -- no CLI framework.
"""
import sys
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry


def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else "/synthetic/odom"
    vx = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
    wz = float(sys.argv[3]) if len(sys.argv) > 3 else 0.1
    rclpy.init()
    # use_sim_time=True: the bag's /clock, not wall time -- stamps must land
    # in the same time base as the (sim-time) camera image stamps this
    # feeds rig_delta()/twist_at() against, or every lookup clamps/misses.
    node = Node("synthetic_odom_publisher",
                parameter_overrides=[Parameter("use_sim_time", value=True)])
    pub = node.create_publisher(Odometry, topic, 10)
    msg = Odometry()
    msg.twist.twist.linear.x = vx
    msg.twist.twist.angular.z = wz

    def tick():
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)

    node.create_timer(1.0 / 50.0, tick)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
