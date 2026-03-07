#!/usr/bin/env python3
"""Move xArm Lite 6 to home position via FollowJointTrajectory action."""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import numpy as np

Q_HOME = [-1.14, 0.30, 1.70, 0.50, 0.5, 0.0]  # joint5=0.5 avoids wrist singularity
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
MOVE_TIME_SEC = 8  # generous time to ensure controller settles precisely


class GoHome(Node):

    def __init__(self):
        super().__init__('go_home')
        self._client = ActionClient(
            self, FollowJointTrajectory,
            '/lite6_traj_controller/follow_joint_trajectory')
        self._last_q = None
        self._js_sub = self.create_subscription(
            JointState, '/joint_states', self._js_cb, 10)

    def _js_cb(self, msg):
        n2i = {n: i for i, n in enumerate(msg.name)}
        if all(j in n2i for j in JOINT_NAMES):
            self._last_q = np.array([msg.position[n2i[j]] for j in JOINT_NAMES])

    def send(self):
        self.get_logger().info('Waiting for trajectory controller...')
        self._client.wait_for_server()

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = JOINT_NAMES

        pt = JointTrajectoryPoint()
        pt.positions = Q_HOME
        pt.velocities = [0.0] * 6
        pt.time_from_start = Duration(sec=MOVE_TIME_SEC, nanosec=0)
        goal.trajectory.points = [pt]

        self.get_logger().info(
            f'Moving to home: {np.round(Q_HOME, 3).tolist()}  ({MOVE_TIME_SEC} s)')
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._goal_cb)

    def _goal_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Goal rejected')
            rclpy.shutdown()
            return
        self.get_logger().info('Goal accepted — moving...')
        handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result().result
        if result.error_code == 0:
            if self._last_q is not None:
                err = np.array(Q_HOME) - self._last_q
                self.get_logger().info(
                    f'Reached home.\n'
                    f'  Target : {np.round(Q_HOME, 4)}\n'
                    f'  Actual : {np.round(self._last_q, 4)}\n'
                    f'  Error  : {np.round(err, 4)}  (norm={np.linalg.norm(err)*180/np.pi:.2f} deg)')
            else:
                self.get_logger().info('Reached home position.')
        else:
            self.get_logger().error(
                f'Failed: error_code={result.error_code}  {result.error_string}\n'
                f'  Last q: {np.round(self._last_q, 4) if self._last_q is not None else "unknown"}')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = GoHome()
    node.send()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
