#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""3DGS撮影スクリプトの経路を、ロボットを動かさずにrvizへ表示する。

gs_3d_image_picture の経路(locations/gs_capture_waypoints.yaml)を、spawn位置
(spawn_locations.yaml の ib2_spawn)から始めて、各地点を順に結ぶ。

経路は線(先頭→末尾で色が変わる)、各地点での camera_main の向きは矢印
(body +x方向)、地点の番号は文字で表示する。すべて iss_body フレーム。
経路のyamlは撮影スクリプトと同じものを読むので、そちらを変えれば表示も変わる。

    ros2 run intball2_programs gs_path_preview
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from intball2_programs.gs_capture.gs_3d_image_picture import (
    WAYPOINTS_FRAME,
    load_spawn_pose,
    load_waypoints,
)
from intball2_programs.ros import (
    MarkerArrayPublisher,
    make_arrow_marker,
    make_delete_all_marker,
    make_line_strip_marker,
    make_text_marker,
)

MARKER_TOPIC = '/gs_path_preview'
REPUBLISH_PERIOD_SEC = 2.0

# camera_main(URDFのcameraF_link)はbodyの+x方向を向いている。
CAMERA_AXIS_BODY = np.array([1.0, 0.0, 0.0])
CAMERA_ARROW_LENGTH_M = 0.3
LABEL_OFFSET_M = np.array([0.0, 0.0, 0.08])

SEQUENCE_COLOR_START = (0.2, 0.4, 1.0)  # 青 → 赤
SEQUENCE_COLOR_END = (1.0, 0.2, 0.2)


def compute_sequence_poses(start_pos, start_rot):
    """spawnから始めて経路の地点を順に[(ラベル, 位置, 回転行列), ...]で返す。"""
    poses = [('S', start_pos, start_rot)]
    for i, (_name, pos, rot) in enumerate(load_waypoints(), start=1):
        poses.append((str(i), pos, rot))
    return poses


def _gradient(start, end, n):
    """startからendへn段階で変わる色[r, g, b, 1.0]のリスト。"""
    return [
        tuple(s + (e - s) * (k / max(n - 1, 1)) for s, e in zip(start, end)) + (1.0,)
        for k in range(n)
    ]


def build_path_markers(ns, poses, color_start, color_end):
    """経路の線・カメラ向きの矢印・ラベル文字のMarkerを作る。"""
    colors = _gradient(color_start, color_end, len(poses))
    markers = [make_line_strip_marker(
        WAYPOINTS_FRAME, f'{ns}_path', 0, [p for _, p, _ in poses], colors
    )]
    for k, ((_, pos, rot), color) in enumerate(zip(poses, colors)):
        tip = pos + rot @ CAMERA_AXIS_BODY * CAMERA_ARROW_LENGTH_M
        markers.append(make_arrow_marker(WAYPOINTS_FRAME, f'{ns}_camera', k, pos, tip, color))

    # 回転だけのステップは同じ位置に重なるので、位置ごとにラベルをまとめる。
    grouped = {}
    for label, pos, _ in poses:
        grouped.setdefault(tuple(np.round(pos, 2)), (pos, []))[1].append(label)
    for k, (pos, labels) in enumerate(grouped.values()):
        markers.append(make_text_marker(
            WAYPOINTS_FRAME, f'{ns}_label', k, pos + LABEL_OFFSET_M, ','.join(labels),
            (1.0, 1.0, 1.0, 1.0),
        ))
    return markers


class GsPathPreview(Node):
    """3DGS撮影経路をMarkerArrayで配信し続けるノード(ロボットは動かさない)。"""

    def __init__(self):
        super().__init__('gs_path_preview_node')
        poses = compute_sequence_poses(*load_spawn_pose())
        self._log_poses('sequence', poses)
        self._markers = [make_delete_all_marker()] + build_path_markers(
            'sequence', poses, SEQUENCE_COLOR_START, SEQUENCE_COLOR_END
        )

        # rvizを後から起動しても表示されるようにtransient_localにし、定期的にも再送する。
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = MarkerArrayPublisher(self, MARKER_TOPIC, qos)
        self._publish()
        self.create_timer(REPUBLISH_PERIOD_SEC, self._publish)
        self.get_logger().info(
            f'[preview] publishing sequence path on {MARKER_TOPIC} ({WAYPOINTS_FRAME})'
        )

    def _publish(self) -> None:
        self._pub.publish(self._markers)

    def _log_poses(self, name, poses) -> None:
        for label, pos, rot in poses:
            cam = rot @ CAMERA_AXIS_BODY
            self.get_logger().info(
                f'[preview] {name} {label:>6}: pos=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) '
                f'camera=({cam[0]:+.2f}, {cam[1]:+.2f}, {cam[2]:+.2f})'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GsPathPreview()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
