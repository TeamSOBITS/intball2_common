#!/usr/bin/env python3
"""sensor_msgs/Imageを購読する汎用サブスクライバ。"""
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImageSubscriber:
    def __init__(self, node, topic='/camera_main/image_raw', qos=10):
        self._node = node
        self._bridge = CvBridge()
        self.cv_image = None
        node.create_subscription(Image, topic, self._on_message, qos)

    def _on_message(self, msg):
        try:
            self.cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # cv_bridge が投げる複数例外をまとめて扱う
            self._node.get_logger().warn(f"ImageSubscriber: failed to convert image message: {exc}")

    def wait_until_received(self, timeout_sec=5.0):
        """画像を受信するまで待機し、受信できたか(bool)を返す。"""
        start = self._node.get_clock().now()
        while self.cv_image is None and rclpy.ok():
            elapsed = (self._node.get_clock().now() - start).nanoseconds / 1e9
            if elapsed >= timeout_sec:
                self._node.get_logger().warn(
                    f"ImageSubscriber: no image received within {timeout_sec:.1f} sec"
                )
                return False
            rclpy.spin_once(self._node, timeout_sec=0.05)
        return self.cv_image is not None
