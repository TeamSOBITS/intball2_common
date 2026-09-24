#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from datetime import datetime
import cv2
import rclpy
from rclpy.node import Node

from intball2_programs.ros import ImageSubscriber

# 保存先の既定値。ROSパラメータ output_dir で変更できる。
DEFAULT_OUTPUT_DIR = "~/intball2_pictures"


class Picture(Node):
    def __init__(self, topic_name="/camera_main/image_raw"):
        super().__init__('camera_node')
        self.image_sub = ImageSubscriber(self, topic_name)
        output_dir = self.declare_parameter('output_dir', DEFAULT_OUTPUT_DIR).value
        self.directory_path = os.path.expanduser(output_dir)

    def camera_picture(self, picture_name="evidence", wait_timeout=5.0):

        # 保存先ディレクトリ(ROSパラメータ output_dir)
        directory_path = self.directory_path

        self.get_logger().info(f"picture_path: {directory_path}")

        if not os.path.exists(directory_path):
            os.makedirs(directory_path)

        # 画像を受信するまで待機
        if not self.image_sub.wait_until_received(wait_timeout):
            return False

        # 画像取得が確定してからファイル名を生成する（処理の最適化）
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        filename = f"{picture_name}_{timestamp}.jpg"
        save_path = os.path.join(directory_path, filename)

        # 直接 self.image_sub.cv_image を保存
        success = cv2.imwrite(save_path, self.image_sub.cv_image)

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