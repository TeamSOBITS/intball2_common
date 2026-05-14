#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from datetime import datetime
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Picture(Node):
    def __init__(self, topic_name="/camera_main/image_raw"):
        super().__init__('camera_node')
        self.cv_image = None
        self.bridge = CvBridge()

        # ROS 2 のサブスクライバ作成 (QoS=10)
        # topic_name はここでしか使わないため、selfに保持せず直接渡す
        self.create_subscription(Image, topic_name, self.image_callback, 10)

    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # cv_bridge が投げる複数例外をまとめて扱う
            self.get_logger().warn(f"Picture: failed to convert image message: {exc}")

    def camera_picture(self, picture_name="evidence", wait_timeout=5.0):
        
        # 1. このスクリプトが存在するディレクトリを取得
        current_dir = os.path.dirname(os.path.abspath(__file__))
        
        # 2. 相対パスでディレクトリを指定 (同じフォルダ内の picture フォルダ)
        directory_path = os.path.join(current_dir, "..", "picture/") 

        self.get_logger().info(f"picture_path: {directory_path}")
        
        if not os.path.exists(directory_path):
            os.makedirs(directory_path)

        start = self.get_clock().now()
        
        # 画像を受信するまで待機
        while self.cv_image is None and rclpy.ok():
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9  # 秒単位に変換
            if elapsed >= wait_timeout:
                self.get_logger().warn(f"Picture: no image received within {wait_timeout:.1f} sec")
                return False
            
            # コールバックを実行させるためのspin処理
            rclpy.spin_once(self, timeout_sec=0.05)

        # ループを抜けた際、画像がまだNoneならここで終了 (Ctrl+Cで中断された場合など)
        if self.cv_image is None:
            self.get_logger().warn("Picture: image is still empty after wait")
            return False

        # 画像取得が確定してからファイル名を生成する（処理の最適化）
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        filename = f"{picture_name}_{timestamp}.jpg"
        save_path = os.path.join(directory_path, filename)
        
        # 直接 self.cv_image を保存
        success = cv2.imwrite(save_path, self.cv_image)

        if success:
            self.get_logger().info(f"Picture: saved {save_path}")
        else:
            self.get_logger().warn(f"Picture: failed to save {save_path}")
        return success


def main(args=None):
    rclpy.init(args=args)
    
    node = Picture()
    try:
        # メイン処理の実行
        node.camera_picture("test_1")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()