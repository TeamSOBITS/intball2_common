"""
ply_target_extractor.py
========================
/ply_points (PointCloud2) を購読し、3D ROI（直方体領域）の内側にある点だけを
抽出して /target_pointcloud として再publishするノード。

【設計方針】
- ply_publisher.py が作るPointCloud2のフィールド定義
    x   : offset  0, FLOAT32
    y   : offset  4, FLOAT32
    z   : offset  8, FLOAT32
    rgb : offset 12, FLOAT32 (ビットパック済み)
    point_step = 16 byte
  を前提とし、このノードでは同じdtypeでバイト列を読み書きするだけで、
  ply_publisher.py / ply_gpu_utils.py / ply_color_utils.py の色計算ロジックには
  一切手を加えない。

- ROIの範囲は6個のROS2パラメータ（roi_x_min/x_max/y_min/y_max/z_min/z_max）で
  外部から変更可能にする。デフォルト値はclaude.mdのStep1の例と同じ。

- 抽出結果に加えて、ROIの直方体をRViz2で確認できるよう
  visualization_msgs/Marker（CUBE, 半透明）を /target_roi_marker にpublishする。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker
import numpy as np


class PlyTargetExtractor(Node):
    def __init__(self):
        super().__init__('ply_target_extractor')

        # --- ROIパラメータ宣言（デフォルト値はclaude.md Step1の例）---
        self.declare_parameter('roi_x_min', 1.0)
        self.declare_parameter('roi_x_max', 2.0)
        self.declare_parameter('roi_y_min', -0.5)
        self.declare_parameter('roi_y_max', 0.5)
        self.declare_parameter('roi_z_min', 0.3)
        self.declare_parameter('roi_z_max', 1.5)

        # Subscriber: ply_publisher.py が publish する点群
        self.sub = self.create_subscription(
            PointCloud2, 'ply_points', self.pointcloud_callback, 10)

        # Publisher: ROI内の点群 / ROI可視化用Marker
        self.pub_target = self.create_publisher(PointCloud2, 'target_pointcloud', 10)
        self.pub_marker = self.create_publisher(Marker, 'target_roi_marker', 10)

        # PointCloud2の構造化配列dtype（ply_publisher.pyのbuffer定義と完全一致させる）
        self._pc2_dtype = np.dtype([
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('rgb', np.float32),
        ])

        self.get_logger().info(
            'PlyTargetExtractor started. Subscribing to "ply_points", '
            'publishing "target_pointcloud" and "target_roi_marker".')

    def _get_roi(self):
        """現在のROIパラメータ値を(x_min, x_max, y_min, y_max, z_min, z_max)として取得する。"""
        gp = self.get_parameter
        return (
            gp('roi_x_min').get_parameter_value().double_value,
            gp('roi_x_max').get_parameter_value().double_value,
            gp('roi_y_min').get_parameter_value().double_value,
            gp('roi_y_max').get_parameter_value().double_value,
            gp('roi_z_min').get_parameter_value().double_value,
            gp('roi_z_max').get_parameter_value().double_value,
        )

    def pointcloud_callback(self, msg: PointCloud2):
        x_min, x_max, y_min, y_max, z_min, z_max = self._get_roi()

        # フィールド構成が想定と異なる場合はエラーとして処理を中断する
        # （ply_publisher.py側でフィールド定義が変わった場合の事故防止）
        if msg.point_step != self._pc2_dtype.itemsize:
            self.get_logger().error(
                f'Unexpected point_step={msg.point_step} '
                f'(expected {self._pc2_dtype.itemsize}). '
                'ply_publisher.py のPointCloud2フィールド定義と一致していません。')
            return

        # バイト列 → 構造化NumPy配列（x, y, z, rgbの4フィールド）
        arr = np.frombuffer(msg.data, dtype=self._pc2_dtype, count=msg.width)

        # --- 3D ROI判定 ---
        # 各点 p=(x,y,z) が
        #   x_min <= x <= x_max  かつ
        #   y_min <= y <= y_max  かつ
        #   z_min <= z <= z_max
        # を全て満たす点だけを残す（3軸すべての条件のAND）。
        mask = (
            (arr['x'] >= x_min) & (arr['x'] <= x_max) &
            (arr['y'] >= y_min) & (arr['y'] <= y_max) &
            (arr['z'] >= z_min) & (arr['z'] <= z_max)
        )
        filtered = arr[mask]  # ROI内の点だけの新しい構造化配列（rgbはそのままコピー、色の再計算はしない）

        self._publish_target_pointcloud(filtered, msg.header.frame_id, msg.header.stamp)
        self._publish_roi_marker(
            x_min, x_max, y_min, y_max, z_min, z_max,
            msg.header.frame_id, msg.header.stamp)

        self.get_logger().info(
            f'ROI filter: {msg.width} points -> {len(filtered)} points inside ROI.')

    def _publish_target_pointcloud(self, filtered: np.ndarray, frame_id: str, stamp):
        """ROI抽出後の点群を、ply_publisher.pyと同一レイアウトのPointCloud2として送出する。"""
        out = PointCloud2()
        out.header.stamp = stamp
        out.header.frame_id = frame_id
        out.height = 1
        out.width = len(filtered)
        out.is_bigendian = False
        out.is_dense = True

        out.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        out.point_step = 16
        out.row_step = out.point_step * out.width
        out.data = filtered.tobytes()

        self.pub_target.publish(out)

    def _publish_roi_marker(self, x_min, x_max, y_min, y_max, z_min, z_max,
                             frame_id: str, stamp):
        """ROIの直方体をRViz2で確認するための半透明CUBE Markerを送出する。

        中心位置:
            x_c = (x_min + x_max) / 2
            y_c = (y_min + y_max) / 2
            z_c = (z_min + z_max) / 2
        直方体サイズ:
            width  = x_max - x_min
            depth  = y_max - y_min
            height = z_max - z_min
        （claude.md Step2の中心位置・大きさの式と同一）
        """
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = 'target_roi'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = (x_min + x_max) / 2.0
        marker.pose.position.y = (y_min + y_max) / 2.0
        marker.pose.position.z = (z_min + z_max) / 2.0
        marker.pose.orientation.w = 1.0  # 回転なし

        # スケール0だとRViz2で描画エラーになるため最小値をクランプ
        marker.scale.x = max(x_max - x_min, 1e-3)
        marker.scale.y = max(y_max - y_min, 1e-3)
        marker.scale.z = max(z_max - z_min, 1e-3)

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.25  # 半透明にして中の点群が見えるようにする

        self.pub_marker.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = PlyTargetExtractor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
