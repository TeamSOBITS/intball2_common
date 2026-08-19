#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32
import struct


class PointCloudDistance(Node):
    def __init__(self):
        super().__init__('pointcloud_distance')

        self.declare_parameter('input_topic', '/stereo/points2')
        self.declare_parameter('center_ratio', 0.2)   # 中心領域の割合（画角に対する比率）
        # ステレオの視差探索限界による測距不能距離（baseline=0.05m, fx≈476.7px,
        # disparity_max=98 のとき理論下限 ≈0.243m）にマージンを乗せた閾値。
        # これより近い値はマッチング破綻によるノイズの可能性が高く信用できない。
        self.declare_parameter('min_reliable_distance', 0.28)

        input_topic = self.get_parameter('input_topic').value
        self.center_ratio = self.get_parameter('center_ratio').value
        self.min_reliable_distance = self.get_parameter('min_reliable_distance').value

        self.create_subscription(PointCloud2, input_topic, self._cb, 10)

        self.pub_min = self.create_publisher(Float32, '/distance/min', 10)
        self.pub_center = self.create_publisher(Float32, '/distance/center', 10)

        self.get_logger().info(f'PointCloudDistance started: {input_topic}')

    def _cb(self, msg: PointCloud2):
        xyz = self._parse_xyz(msg)
        if xyz is None or len(xyz) == 0:
            return

        # カメラからの距離 = Z 軸方向の値（前方）
        z = xyz[:, 2]

        # 視差探索限界より遠い＝信頼できる点のみを対象にする
        reliable = z > self.min_reliable_distance

        # 最近傍距離（信頼できる点が無ければ「測距限界より近い＝最大限危険」とみなす）
        if np.any(reliable):
            min_dist = float(np.min(z[reliable]))
        else:
            min_dist = self.min_reliable_distance
            self.get_logger().warn(
                'All points are closer than the stereo blind zone '
                f'({self.min_reliable_distance:.3f}m) - treating as max danger',
                throttle_duration_sec=1.0,
            )

        # 中心領域の平均距離（x, y が原点付近の点のみ）
        center_mask = self._center_mask(xyz)
        reliable_center_mask = center_mask & reliable
        if np.any(reliable_center_mask):
            center_dist = float(np.mean(z[reliable_center_mask]))
        elif np.any(center_mask):
            # 中心方向に点はあるが、全て測距限界より近い
            center_dist = self.min_reliable_distance
        else:
            # 中心方向に有効な点が無い（何も検出されていない）
            center_dist = float('nan')

        self.pub_min.publish(Float32(data=min_dist))
        self.pub_center.publish(Float32(data=center_dist))

        self.get_logger().info(
            f'min={min_dist:.3f}m  center={center_dist:.3f}m',
            throttle_duration_sec=1.0,
        )

    def _center_mask(self, xyz: np.ndarray) -> np.ndarray:
        x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        # x/z, y/z が center_ratio 以内 ≒ カメラ中心方向
        with np.errstate(divide='ignore', invalid='ignore'):
            ang_x = np.abs(x / z)
            ang_y = np.abs(y / z)
        return (ang_x < self.center_ratio) & (ang_y < self.center_ratio)

    @staticmethod
    def _parse_xyz(msg: PointCloud2) -> np.ndarray:
        """PointCloud2 から XYZ を float32 numpy 配列として取り出す。"""
        # オフセットを field 定義から取得
        offsets = {f.name: f.offset for f in msg.fields}
        if not all(k in offsets for k in ('x', 'y', 'z')):
            return None

        ox, oy, oz = offsets['x'], offsets['y'], offsets['z']
        step = msg.point_step
        n = msg.width * msg.height
        data = bytes(msg.data)

        xyz = np.empty((n, 3), dtype=np.float32)
        for i in range(n):
            base = i * step
            xyz[i, 0] = struct.unpack_from('<f', data, base + ox)[0]
            xyz[i, 1] = struct.unpack_from('<f', data, base + oy)[0]
            xyz[i, 2] = struct.unpack_from('<f', data, base + oz)[0]

        # nan / inf / 負のZ を除外
        valid = np.isfinite(xyz).all(axis=1) & (xyz[:, 2] > 0)
        return xyz[valid]


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudDistance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
