#!/usr/bin/env python3
"""ib2_msgs/CtlCommandアクションを呼び出す汎用クライアント。"""
import math

import rclpy
from rclpy.action import ActionClient

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from std_msgs.msg import Header

from ib2_msgs.action import CtlCommand
from ib2_msgs.msg import CtlStatusType


def quaternion_from_euler(roll, pitch, yaw):
    cr = math.cos(roll / 2.0)
    sr = math.sin(roll / 2.0)
    cp = math.cos(pitch / 2.0)
    sp = math.sin(pitch / 2.0)
    cy = math.cos(yaw / 2.0)
    sy = math.sin(yaw / 2.0)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


class CtlCommandClient:
    def __init__(self, node, action_name='ctl/command_ros2'):
        self._node = node
        self._action_name = action_name
        self._client = ActionClient(node, CtlCommand, action_name)

    def wait_for_server(self, timeout_sec=5.0):
        self._node.get_logger().info('Checking control action server...')
        if not self._client.wait_for_server(timeout_sec=timeout_sec):
            self._node.get_logger().error(f'Control action server ({self._action_name}) NOT FOUND')
            return False
        return True

    def send_relative_move(self, x, y, z, roll, pitch, yaw):
        """相対移動ゴールを送信し、結果を受け取るまで同期的に待機する。成功/失敗をboolで返す。"""
        q = quaternion_from_euler(roll, pitch, yaw)

        goal_ctl = CtlCommand.Goal()
        goal_ctl.target = PoseStamped()
        goal_ctl.target.header = Header()
        goal_ctl.target.header.stamp = self._node.get_clock().now().to_msg()
        goal_ctl.target.header.frame_id = 'body'

        goal_ctl.target.pose = Pose()
        goal_ctl.target.pose.position = Point(x=x, y=y, z=z)
        goal_ctl.target.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        move_type_obj = CtlStatusType()
        move_type_obj.type = getattr(CtlStatusType, 'MOVE_TO_RELATIVE_TARGET', 30)
        goal_ctl.type = move_type_obj

        self._node.get_logger().info(f"Sending move goal: x={x}, y={y}, z={z}")

        f_goal = self._client.send_goal_async(goal_ctl)
        rclpy.spin_until_future_complete(self._node, f_goal)

        goal_handle = f_goal.result()
        if not goal_handle.accepted:
            self._node.get_logger().error('Ctl goal rejected')
            return False

        f_result = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, f_result)

        return True
