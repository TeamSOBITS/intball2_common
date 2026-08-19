#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32
from vision_msgs.msg import Detection2DArray
from message_filters import ApproximateTimeSynchronizer, Subscriber


class BboxCenterDistance(Node):
    """bboxの中心画素の3D点から、ロボット(カメラ原点)までのユークリッド距離を測る。"""

    def __init__(self):
        super().__init__('bbox_center_distance')

        self.declare_parameter('detection_topic', '/yolo_ros/object_boxes')
        self.declare_parameter('pointcloud_topic', '/stereo/points2')
        self.declare_parameter('target_class', 'person')
        self.declare_parameter('sync_slop', 0.1)
        self.declare_parameter('center_window', 2)  # 中心画素 ±window px の範囲を使う

        det_topic = self.get_parameter('detection_topic').value
        pc_topic = self.get_parameter('pointcloud_topic').value
        self.target_class = self.get_parameter('target_class').value
        slop = self.get_parameter('sync_slop').value
        self.window = self.get_parameter('center_window').value

        det_sub = Subscriber(self, Detection2DArray, det_topic)
        pc_sub = Subscriber(self, PointCloud2, pc_topic)
        self.sync = ApproximateTimeSynchronizer(
            [det_sub, pc_sub], queue_size=5, slop=slop
        )
        self.sync.registerCallback(self._cb)

        self.pub = self.create_publisher(Float32, '/distance/bbox_center', 10)

        self.get_logger().info(
            f'BboxCenterDistance started\n'
            f'  detections: {det_topic}\n'
            f'  pointcloud: {pc_topic}\n'
            f'  target    : {self.target_class}'
        )

    def _cb(self, det_msg: Detection2DArray, pc_msg: PointCloud2):
        # stereo_image_proc は organized point cloud を出力する（height = 画像高さ）
        if pc_msg.height <= 1:
            self.get_logger().warn(
                'Point cloud is not organized (height=1). '
                'Cannot look up bbox center point.',
                throttle_duration_sec=5.0,
            )
            return

        xyz = self._pc_to_xyz(pc_msg)  # (H, W, 3)
        h, w = pc_msg.height, pc_msg.width

        for det in det_msg.detections:
            if det.id != self.target_class:
                continue

            cx = int(round(det.bbox.center.position.x))
            cy = int(round(det.bbox.center.position.y))
            score = det.results[0].hypothesis.score if det.results else 0.0

            # 中心画素周辺の小窓から有効なXYZ点を集め、単一画素のNaN/ノイズに強くする
            x1 = max(0, cx - self.window)
            x2 = min(w, cx + self.window + 1)
            y1 = max(0, cy - self.window)
            y2 = min(h, cy + self.window + 1)
            region = xyz[y1:y2, x1:x2, :].reshape(-1, 3)
            valid = np.isfinite(region).all(axis=1) & (region[:, 2] > 0)
            valid_points = region[valid]

            if len(valid_points) == 0:
                self.get_logger().warn(
                    'No valid depth point at bbox center',
                    throttle_duration_sec=2.0,
                )
                continue

            point = np.median(valid_points, axis=0)
            # ロボット(カメラ原点)から中心点までのユークリッド距離
            dist = float(np.linalg.norm(point))
            self.pub.publish(Float32(data=dist))
            self.get_logger().info(
                f'[{self.target_class}] dist={dist:.3f}m  '
                f'score={score:.2f}  center=({cx},{cy})'
            )

    @staticmethod
    def _pc_to_xyz(msg: PointCloud2) -> np.ndarray:
        """Organized PointCloud2 を (H, W, 3) の float32 配列に変換する。"""
        offsets = {f.name: f.offset for f in msg.fields}
        dt = np.dtype({
            'names': ['x', 'y', 'z'],
            'formats': [np.float32, np.float32, np.float32],
            'offsets': [offsets['x'], offsets['y'], offsets['z']],
            'itemsize': msg.point_step,
        })
        arr = np.frombuffer(bytes(msg.data), dtype=dt)
        return np.column_stack([arr['x'], arr['y'], arr['z']]).reshape(
            msg.height, msg.width, 3
        )


def main(args=None):
    rclpy.init(args=args)
    node = BboxCenterDistance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
