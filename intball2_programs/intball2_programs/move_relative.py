#!/usr/bin/env python3
"""ROS2 (Humble) simple relative mover.
Cleaned up dead code and improved shutdown handling.
"""
import argparse
import threading
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from std_msgs.msg import Header
from builtin_interfaces.msg import Time as BITime

from ib2_msgs.action import CtlCommand
from ib2_msgs.msg import CtlStatusType
from platform_msgs.msg import UserNodeStatus


def quaternion_from_euler(roll: float, pitch: float, yaw: float):
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


class SimpleMoveNode(Node):
    def __init__(self, params):
        super().__init__('simple_move_node')
        self.params = params
        self.status_msg = "Initializing"

        # Publishers
        self.pub_status = self.create_publisher(UserNodeStatus, '/ib2_user/status', 1)
        self.pub_done = self.create_publisher(BITime, '/ib2_user/complete', 1)

        # Action clients
        self.client_ctl = ActionClient(self, CtlCommand, 'ctl/command_ros2')

    def update_status_loop(self):
        """Continuously publish status at ~5 Hz."""
        while rclpy.ok():
            msg = UserNodeStatus()
            msg.stamp = self.get_clock().now().to_msg()
            data = self.status_msg.encode('utf-8')[:800]
            # パディング処理をスッキリ記述
            msg.msg = list(data.ljust(800, b'\x00'))
            self.pub_status.publish(msg)
            time.sleep(0.2)

    def wait_for_action_servers(self) -> bool:
        """Wait for the required control server."""
        self.get_logger().info('Checking control action server...')
        # 5秒待機
        if not self.client_ctl.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Control action server (ctl/command_ros2) NOT FOUND')
            return False
        return True

    def execute(self):
        # 1. サーバーの存在確認
        if not self.wait_for_action_servers():
            return

        # 2. 移動目標の構築
        self.status_msg = 'Moving'
        p = self.params
        q = quaternion_from_euler(p['roll'], p['pitch'], p['yaw'])

        goal_ctl = CtlCommand.Goal()
        goal_ctl.target = PoseStamped()
        goal_ctl.target.header = Header()
        goal_ctl.target.header.stamp = self.get_clock().now().to_msg()
        goal_ctl.target.header.frame_id = 'body' 
        
        goal_ctl.target.pose = Pose()
        goal_ctl.target.pose.position = Point(x=p['x'], y=p['y'], z=p['z'])
        goal_ctl.target.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # 移動タイプの指定
        move_type_obj = CtlStatusType()
        move_type_obj.type = getattr(CtlStatusType, 'MOVE_TO_RELATIVE_TARGET', 30)
        goal_ctl.type = move_type_obj

        self.get_logger().info(f"Sending move goal: x={p['x']}, y={p['y']}, z={p['z']}")
        
        # 3. アクション送信と結果待機
        f_goal = self.client_ctl.send_goal_async(goal_ctl)
        rclpy.spin_until_future_complete(self, f_goal)
        
        goal_handle = f_goal.result()
        if not goal_handle.accepted:
            self.get_logger().error('Ctl goal rejected')
            return

        f_result = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, f_result)
        
        # 4. 完了通知
        self.status_msg = 'Done'
        self.pub_done.publish(self.get_clock().now().to_msg())
        self.get_logger().info('Move Completed.')

    def run(self):
        # ステータス更新スレッド開始
        t = threading.Thread(target=self.update_status_loop, daemon=True)
        t.start()
        
        # メイン処理実行
        self.execute()
        
        # 最後のメッセージが飛ぶのを少し待つ
        if rclpy.ok():
            time.sleep(1.0)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('-x', type=float, default=0.0, help='relative move x [m]')
    parser.add_argument('-y', type=float, default=0.0, help='relative move y [m]')
    parser.add_argument('-z', type=float, default=0.0, help='relative move z [m]')
    parser.add_argument('-r', '--roll', type=float, default=0.0, help='relative roll [deg]')
    parser.add_argument('-p', '--pitch', type=float, default=0.0, help='relative pitch [deg]')
    parser.add_argument('-w', '--yaw', type=float, default=0.0, help='relative yaw [deg]')
    args = parser.parse_args()

    # 度数法からラジアンへ変換
    args.roll = math.radians(args.roll)
    args.pitch = math.radians(args.pitch)
    args.yaw = math.radians(args.yaw)

    return vars(args)


def main():
    rclpy.init()
    node = None
    try:
        params = parse_args()
        node = SimpleMoveNode(params)
        node.run()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()