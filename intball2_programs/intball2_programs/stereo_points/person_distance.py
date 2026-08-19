#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32
from vision_msgs.msg import Detection2DArray
from message_filters import ApproximateTimeSynchronizer, Subscriber


class PersonDistance(Node):
    def __init__(self):
        super().__init__('person_distance')

        self.declare_parameter('detection_topic', '/yolo_ros/object_boxes')
        self.declare_parameter('pointcloud_topic', '/stereo/points2')
        self.declare_parameter('target_class', 'person')
        self.declare_parameter('sync_slop', 0.1)

        det_topic = self.get_parameter('detection_topic').value
        pc_topic = self.get_parameter('pointcloud_topic').value
        self.target_class = self.get_parameter('target_class').value
        slop = self.get_parameter('sync_slop').value

        det_sub = Subscriber(self, Detection2DArray, det_topic)
        pc_sub = Subscriber(self, PointCloud2, pc_topic)
        self.sync = ApproximateTimeSynchronizer(
            [det_sub, pc_sub], queue_size=5, slop=slop
        )
        self.sync.registerCallback(self._cb)

        self.pub = self.create_publisher(Float32, '/distance/person', 10)

        self.get_logger().info(
            f'PersonDistance started\n'
            f'  detections: {det_topic}\n'
            f'  pointcloud: {pc_topic}\n'
            f'  target    : {self.target_class}'
        )

    def _cb(self, det_msg: Detection2DArray, pc_msg: PointCloud2):
        # stereo_image_proc は organized point cloud を出力する（height = 画像高さ）
        if pc_msg.height <= 1:
            self.get_logger().warn(
                'Point cloud is not organized (height=1). '
                'Cannot use bounding-box extraction.',
                throttle_duration_sec=5.0,
            )
            return

        xyz = self._pc_to_xyz(pc_msg)  # (H, W, 3)

        for det in det_msg.detections:
            if det.id != self.target_class:
                continue

            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            bw = det.bbox.size_x
            bh = det.bbox.size_y
            score = det.results[0].hypothesis.score if det.results else 0.0

            # バウンディングボックスをピクセル範囲に変換
            x1 = max(0, int(cx - bw / 2))
            x2 = min(pc_msg.width,  int(cx + bw / 2))
            y1 = max(0, int(cy - bh / 2))
            y2 = min(pc_msg.height, int(cy + bh / 2))

            # BB内の有効な Z値（奥行き）を抽出
            z_region = xyz[y1:y2, x1:x2, 2]
            valid_z = z_region[np.isfinite(z_region) & (z_region > 0)]

            if len(valid_z) == 0:
                self.get_logger().warn(
                    'No valid depth points in person bounding box',
                    throttle_duration_sec=2.0,
                )
                continue

            dist = float(np.median(valid_z))
            self.pub.publish(Float32(data=dist))
            self.get_logger().info(
                f'[{self.target_class}] dist={dist:.3f}m  '
                f'score={score:.2f}  '
                f'bbox=({cx:.0f},{cy:.0f}) {bw:.0f}x{bh:.0f}px'
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
    node = PersonDistance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
