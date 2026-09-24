#!/usr/bin/env python3
"""ROS2 (Humble) simple absolute mover.

dock(dock_bodyフレーム)を原点とした絶対座標・絶対姿勢への移動。move_relative.py
の絶対座標版で、構造は同じ(CtlCommandClient.send_absolute_moveを呼ぶだけ)。
"""
import argparse
import threading
import math
import time

import rclpy
from rclpy.node import Node

from intball2_programs.ros import CompletePublisher, CtlCommandClient, UserNodeStatusPublisher


class SimpleMoveAbsoluteNode(Node):
    def __init__(self, params):
        super().__init__('simple_move_absolute_node')
        self.params = params
        self.status_msg = "Initializing"

        self.status_pub = UserNodeStatusPublisher(self)
        self.complete_pub = CompletePublisher(self)
        self.ctl_client = CtlCommandClient(self)

    def update_status_loop(self):
        """Continuously publish status at ~5 Hz."""
        while rclpy.ok():
            self.status_pub.publish(self.status_msg)
            time.sleep(0.2)

    def execute(self):
        # 1. サーバーの存在確認
        if not self.ctl_client.wait_for_server():
            return

        # 2. 移動目標(dock原点の絶対座標)の送信と結果待機
        self.status_msg = 'Moving'
        p = self.params
        if not self.ctl_client.send_absolute_move(
            p['x'], p['y'], p['z'], p['roll'], p['pitch'], p['yaw']
        ):
            return

        # 3. 完了通知
        self.status_msg = 'Done'
        self.complete_pub.publish()
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
    parser.add_argument('-x', type=float, default=0.0, help='absolute target x [m] (dock origin)')
    parser.add_argument('-y', type=float, default=0.0, help='absolute target y [m] (dock origin)')
    parser.add_argument('-z', type=float, default=0.0, help='absolute target z [m] (dock origin)')
    parser.add_argument('-r', '--roll', type=float, default=0.0, help='absolute roll [deg]')
    parser.add_argument('-p', '--pitch', type=float, default=0.0, help='absolute pitch [deg]')
    parser.add_argument('-w', '--yaw', type=float, default=0.0, help='absolute yaw [deg]')
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
        node = SimpleMoveAbsoluteNode(params)
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
