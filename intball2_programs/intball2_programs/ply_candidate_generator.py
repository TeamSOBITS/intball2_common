"""
ply_candidate_generator.py
===========================
/target_pointcloud (PointCloud2) を購読し、
  1. Bounding Box（中心位置・大きさ）を計算
  2. 対象中心の周囲に球面状の撮影候補地点を生成
  3. 各候補地点から対象中心を見る姿勢(quaternion)を計算
  4. 候補地点をTFフレームとしてbroadcast
するノード。

【設計方針】
- Bounding Boxは、固定のROIパラメータではなく、実際に届いた
  target_pointcloudの点のmin/maxから毎回計算する
  （claude.md Step2: 「最初はBounding Boxの中心を利用する」に対応。
   将来ROI抽出方法が変わっても追従できるようにするため）。
- 候補地点の生成式・視線ベクトルの式はclaude.md Step3/Step4と同一。
- TFの配信は/target_pointcloud受信毎（ply_publisher.pyのtimer周期=2秒に連動）に
  行う。候補が0個（target_pointcloudが空）の場合は警告ログを出してスキップする。
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class PlyCandidateGenerator(Node):
    def __init__(self):
        super().__init__('ply_candidate_generator')

        # --- 候補地点生成パラメータ（デフォルト値はclaude.md Step3の例）---
        self.declare_parameter('candidate_radius', 1.0)                      # r [m]
        self.declare_parameter('candidate_theta_deg', [45.0, 90.0, 135.0])   # 極角のリスト [deg]
        self.declare_parameter('candidate_phi_step_deg', 45.0)               # 方位角ステップ [deg]

        self.sub = self.create_subscription(
            PointCloud2, 'target_pointcloud', self.pointcloud_callback, 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        # PointCloud2の構造化配列dtype（ply_publisher.py / ply_target_extractor.pyと同一）
        self._pc2_dtype = np.dtype([
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('rgb', np.float32),
        ])

        self.get_logger().info(
            'PlyCandidateGenerator started. Subscribing to "target_pointcloud", '
            'broadcasting candidate viewpoints as TF frames.')

    def pointcloud_callback(self, msg: PointCloud2):
        if msg.point_step != self._pc2_dtype.itemsize:
            self.get_logger().error(
                f'Unexpected point_step={msg.point_step} '
                f'(expected {self._pc2_dtype.itemsize}). '
                'ply_target_extractor.py のPointCloud2フィールド定義と一致していません。')
            return

        if msg.width == 0:
            self.get_logger().warn(
                'target_pointcloud is empty (0 points inside ROI). '
                'Skipping candidate TF broadcast.')
            return

        arr = np.frombuffer(msg.data, dtype=self._pc2_dtype, count=msg.width)

        # --- Step2: Bounding Box / 中心位置の計算（実データのmin/maxから）---
        x_min, x_max = float(arr['x'].min()), float(arr['x'].max())
        y_min, y_max = float(arr['y'].min()), float(arr['y'].max())
        z_min, z_max = float(arr['z'].min()), float(arr['z'].max())

        center = np.array([
            (x_min + x_max) / 2.0,
            (y_min + y_max) / 2.0,
            (z_min + z_max) / 2.0,
        ])

        radius = self.get_parameter('candidate_radius').get_parameter_value().double_value
        theta_list_deg = self.get_parameter('candidate_theta_deg').get_parameter_value().double_array_value
        phi_step_deg = self.get_parameter('candidate_phi_step_deg').get_parameter_value().double_value

        if phi_step_deg <= 0.0:
            self.get_logger().error(
                f'candidate_phi_step_deg must be > 0 (got {phi_step_deg}). Skipping.')
            return

        stamp = msg.header.stamp
        frame_id = msg.header.frame_id

        num_candidates = 0
        for theta_deg in theta_list_deg:
            theta = math.radians(theta_deg)
            phi_deg = 0.0
            while phi_deg < 360.0:
                phi = math.radians(phi_deg)

                # --- Step3: 球面状の候補地点生成 ---
                # P(theta, phi) = C + r * [sin(theta)cos(phi), sin(theta)sin(phi), cos(theta)]
                position = center + radius * np.array([
                    math.sin(theta) * math.cos(phi),
                    math.sin(theta) * math.sin(phi),
                    math.cos(theta),
                ])

                # --- Step4: 姿勢（対象中心を見る視線方向）の計算 ---
                # d = C - P
                forward_vec = center - position
                norm = np.linalg.norm(forward_vec)
                if norm < 1e-9:
                    # r=0等で視線ベクトルが定義できない場合はスキップ
                    phi_deg += phi_step_deg
                    continue
                forward = forward_vec / norm

                quat = self._look_at_quaternion(forward)
                child_frame_id = f'candidate_th{int(round(theta_deg))}_ph{int(round(phi_deg))}'
                self._broadcast_transform(frame_id, child_frame_id, position, quat, stamp)
                num_candidates += 1

                phi_deg += phi_step_deg

        self.get_logger().info(
            f'BBox center=({center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f}), '
            f'size=({x_max - x_min:.3f}, {y_max - y_min:.3f}, {z_max - z_min:.3f}) '
            f'-> {num_candidates} candidate TF frames broadcasted.')

    def _broadcast_transform(self, parent_frame, child_frame, position, quat, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = float(position[0])
        t.transform.translation.y = float(position[1])
        t.transform.translation.z = float(position[2])
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def _look_at_quaternion(forward: np.ndarray, up_hint: np.ndarray = None):
        """forward方向(対象中心への視線)を向くカメラ姿勢のクォータニオン(x,y,z,w)を計算する。

        カメラ座標系の割り当て:
            x軸 = right (画像の右方向)
            y軸 = up    (画像の上方向)
            z軸 = forward (光軸、対象中心方向)

        forwardがワールドのZ軸(上下方向)とほぼ平行な場合、outer product(right)が
        ほぼゼロベクトルになり特異点となるため、up_hintを(1,0,0)に切り替える。
        """
        if up_hint is None:
            up_hint = np.array([0.0, 0.0, 1.0])

        if abs(np.dot(forward, up_hint)) > 0.999:
            up_hint = np.array([1.0, 0.0, 0.0])

        right = np.cross(up_hint, forward)
        right = right / np.linalg.norm(right)
        true_up = np.cross(forward, right)

        # 各列がカメラ座標系のx,y,z軸をワールド座標系で表す回転行列
        R = np.column_stack((right, true_up, forward))
        return PlyCandidateGenerator._quaternion_from_matrix(R)

    @staticmethod
    def _quaternion_from_matrix(R: np.ndarray):
        """3x3回転行列からクォータニオン(x,y,z,w)を求める（Shepperdの方法）。"""
        m00, m01, m02 = R[0]
        m10, m11, m12 = R[1]
        m20, m21, m22 = R[2]
        trace = m00 + m11 + m22

        if trace > 0.0:
            s = 0.5 / math.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (m21 - m12) * s
            y = (m02 - m20) * s
            z = (m10 - m01) * s
        elif (m00 > m11) and (m00 > m22):
            s = 2.0 * math.sqrt(1.0 + m00 - m11 - m22)
            w = (m21 - m12) / s
            x = 0.25 * s
            y = (m01 + m10) / s
            z = (m02 + m20) / s
        elif m11 > m22:
            s = 2.0 * math.sqrt(1.0 + m11 - m00 - m22)
            w = (m02 - m20) / s
            x = (m01 + m10) / s
            y = 0.25 * s
            z = (m12 + m21) / s
        else:
            s = 2.0 * math.sqrt(1.0 + m22 - m00 - m11)
            w = (m10 - m01) / s
            x = (m02 + m20) / s
            y = (m12 + m21) / s
            z = 0.25 * s

        return (x, y, z, w)


def main(args=None):
    rclpy.init(args=args)
    node = PlyCandidateGenerator()
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
