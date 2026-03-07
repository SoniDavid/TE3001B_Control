#!/usr/bin/env python3
"""
Standalone joint-state logger for comparing PD vs CTC runs.

Subscribes to /joint_states and writes a timestamped CSV with joint
positions, velocities, and efforts.  Run it alongside either controller:

  # Terminal A — run the controller
  ros2 launch xarm_ctc_challenge challenge.launch.py controller_type:=PD

  # Terminal B — log joint states with a label
  ros2 run xarm_ctc_challenge joint_state_logger --ros-args -p label:=pd -p csv_dir:=data

Then repeat with controller_type:=CTC and label:=ctc.
Use the challenge_analysis tool to compare the resulting CSVs.
"""

import csv
import os
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
)
from sensor_msgs.msg import JointState

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']


class JointStateLogger(Node):

    def __init__(self):
        super().__init__('joint_state_logger')

        label   = str(self.declare_parameter('label',   'run').value)
        csv_dir = str(self.declare_parameter('csv_dir', 'data').value)

        os.makedirs(csv_dir, exist_ok=True)
        ts    = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        fname = os.path.join(csv_dir, f'jointlog_{label}_{ts}.csv')
        self._f = open(fname, 'w', newline='')
        self._w = csv.writer(self._f)
        self._w.writerow(
            ['time_s'] +
            [f'q{j}_rad'  for j in range(1, 7)] +
            [f'qd{j}_rads' for j in range(1, 7)] +
            [f'effort{j}_Nm' for j in range(1, 7)]
        )
        self._row_buf = []

        self.get_logger().info(f'Logging /joint_states → {os.path.abspath(fname)}')

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(JointState, '/joint_states', self._cb, qos)

    def _cb(self, msg: JointState):
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        t = self.get_clock().now().nanoseconds * 1e-9
        q      = [0.0] * 6
        qd     = [0.0] * 6
        effort = [0.0] * 6
        for j, jname in enumerate(JOINT_NAMES):
            if jname in name_to_idx:
                idx = name_to_idx[jname]
                q[j]      = msg.position[idx] if msg.position else 0.0
                qd[j]     = msg.velocity[idx] if msg.velocity else 0.0
                effort[j] = msg.effort[idx]   if msg.effort   else 0.0

        self._row_buf.append([t] + q + qd + effort)
        if len(self._row_buf) >= 50:
            self._flush()

    def _flush(self):
        if self._row_buf:
            self._w.writerows(self._row_buf)
            self._f.flush()
            self._row_buf.clear()

    def destroy_node(self):
        self._flush()
        self._f.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = JointStateLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
