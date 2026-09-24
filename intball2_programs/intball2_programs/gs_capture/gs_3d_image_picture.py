#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""3DGS(3D Gaussian Splatting)学習データ収集用の撮影+移動シーケンス。

ISS船内(きぼう)の通路を経路の地点の順に巡りながら、main(前向き)とステレオの
right(右横向き)の2台で撮影する。ステレオのleftはrightと5cmしか離れておらず同じ
方向を向いているため、3DGS用には使わない。

撮影は時間ごとではなく、ロボットが動いた量で行う(ほぼ同じ写真を増やさないため):

- 前回の撮影から CAPTURE_MIN_DIST_M 以上動いた、または CAPTURE_MIN_ANGLE_DEG 以上
  回転したとき(TF ``iss_body`` <- ``body`` で判定)
- 各地点に着いたとき(前回の撮影とほぼ同じ姿勢なら撮らない)

撮るときは、すべてのカメラの画像が新しいかを確かめる。前回保存した画像と同じ時刻か、
そのときのTFの時刻より IMAGE_MAX_LAG_SEC 以上前の画像があれば古いとみなし、どの
カメラも保存しない(mainとステレオの番号をそろえるため)。その位置と向きは「撮れなかった
場所」として記録し、移動は止めずに続ける(移動の途中でロボットを止める方法が無いため)。
すべての地点を回った後、撮れなかった場所へ移動時間が短い順(位置と向きの両方で見積もる)に
戻り、止まった状態で新しい画像を REVISIT_IMAGE_WAIT_SEC まで待って撮り直す。これを最大
REVISIT_MAX_ROUNDS 回繰り返す。

経路は locations/gs_capture_waypoints.yaml に iss_bodyフレームで定義されており、
起動時に読み込む。JPMメッシュ(media/meshes/iss/jpm.dae)とカメラの視野(80度x80度)を
使った最適化(``ros2 run intball2_programs gs_optimize_path``)で作る。経路は
``ros2 run intball2_programs gs_path_preview`` でrvizに表示して確認できる。

移動は ib2_msgs/CtlCommand アクション(MOVE_TO_ABSOLUTE_TARGET, frame_id=
'dock_body')を1地点ずつ完了待ちしながら逐次送信する。各地点への移動直前に
TF(``dock_body`` <- ``iss_body``)を読み、dock_bodyフレームへ変換してから送る。
CtlCommandClient.send_absolute_move() は結果が届くまで
rclpy.spin_until_future_complete(self) でこのノード自身をスピンするため、その間も
撮影判定のタイマーは呼ばれ続け、移動中・回転中も撮影できる。撮り直しのための移動中は
タイマーでは撮影しない。

写真は ``<output_dir>/<実行開始日時>/<カメラ名>/`` に保存する。output_dir は
ROSパラメータで指定でき、省略時は intball2_programs/3dgs_pictures。

    ros2 run intball2_programs gs_3d_image_picture
    ros2 run intball2_programs gs_3d_image_picture --ros-args -p output_dir:=/path/to/dir
"""
import math
import os
import time
from datetime import datetime

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from intball2_programs.gs_capture.pose_math import matrix_to_quat, matrix_to_rpy, quat_to_matrix
from intball2_programs.ros import CtlCommandClient, ImageSubscriber, TFClient

# main はロボットの前(body +x)、right(ステレオ)は右横(body +y)を向いている。
CAMERA_TOPICS = {
    "camera_main": "/camera_main/image_raw",
    "camera_right": "/camera_right/image_raw",
}

DEFAULT_OUTPUT_DIR = '/root/colcon_ws/src/intball2_common/intball2_programs/3dgs_pictures'

# 撮影のきっかけ(前回の撮影からの移動量)。写真を400枚以内にするため、隣の写真と
# 5割以上重なる範囲で広めにしている(視野80度に対して、30度回っても約6割重なる)。
CAPTURE_MIN_DIST_M = 0.45
CAPTURE_MIN_ANGLE_DEG = 30.0
# 地点に着いたとき、前回の撮影からこれ以上動いていれば撮る。
ARRIVAL_MIN_DIST_M = 0.05
ARRIVAL_MIN_ANGLE_DEG = 5.0
CAPTURE_CHECK_PERIOD_SEC = 0.2
# 画像の時刻がTF(ロボットの位置)の時刻よりこれ以上前なら、その位置の画像ではない(古い)。
IMAGE_MAX_LAG_SEC = 0.5
# 撮れなかった場所へ戻ったとき、新しい画像を待つ時間と、撮り直しを繰り返す回数。
REVISIT_IMAGE_WAIT_SEC = 10.0
REVISIT_MAX_ROUNDS = 2
# ロボットの移動・回転の速さ(撮り直しの順番と、gs_coverageの所要時間の見積もりに使う)。
# 移動は2026-09-24の実行(0.8mを約21秒)から。回転は実際に測っていない仮の値。
ROBOT_LIN_SPEED_MPS = 0.8 / 21.0
ROBOT_ROT_SPEED_DEGPS = 90.0 / 30.0

# send_absolute_move()が要求する、dockを原点とした絶対座標フレーム。
MOVE_TARGET_FRAME = "dock_body"
# 経路の地点(WAYPOINTS_FILE)を定義しているフレーム。
WAYPOINTS_FRAME = "iss_body"
TF_LOOKUP_TIMEOUT_SEC = 5.0

# 経路の地点を定義したyaml(locations/以下)。上から順に巡る。
WAYPOINTS_FILE = 'gs_capture_waypoints.yaml'


# TFの参照フレーム(sobits_intball2_gncのcontrol_nodeと同じもの。Gazebo直結の
# ほぼ真値で、Navigationのon/offに依存しない)。
TF_REFERENCE_FRAME = "iss_body"
TF_TARGET_FRAME = "body"
# 起動直後の数秒間、issdyn等のプラグインが追いつくまでTFはまだ本当の値に収束
# しておらず、原点付近の仮の値を返すことがある(stamp=0のダミーとは別物)。その
# ため、単に最初に取れた値を使うのではなく、値がSETTLE_TOLERANCE_M以内で
# SETTLE_DURATION_SEC秒以上安定するまで待ってから採用する。
TF_STARTUP_TIMEOUT_SEC = 15.0
SETTLE_DURATION_SEC = 2.0
SETTLE_TOLERANCE_M = 0.05

# locations/spawn_locations.yaml の ib2_spawn に書かれた本来のspawn位置(iss_body
# フレーム)。起動直後にTFで読んだ実位置がここから大きくずれている場合、前回の
# 実行やコリジョンでロボットが動いたままシミュレータがリセットされていない可能性
# が高いため、安全な原点とはみなさず実行を中断する。
SPAWN_LOCATION_NAME = "ib2_spawn"
SPAWN_POSITION_TOLERANCE_M = 0.3


def _load_locations(filename):
    share_dir = get_package_share_directory('intball2_programs')
    with open(os.path.join(share_dir, 'locations', filename), 'r', encoding='utf-8') as handle:
        return yaml.safe_load(handle) or {}


def _entry_to_pose(entry):
    t, r = entry['translation'], entry['rotation']
    return np.array([t['x'], t['y'], t['z']]), quat_to_matrix(r['x'], r['y'], r['z'], r['w'])


def load_spawn_pose():
    """spawn_locations.yamlからib2のspawn姿勢(位置, 回転行列)をiss_bodyフレームで読む。"""
    return _entry_to_pose(_load_locations('spawn_locations.yaml')[SPAWN_LOCATION_NAME])


def load_waypoints():
    """経路の地点を、yamlに書かれた順に[(名前, 位置, 回転行列), ...]で返す。"""
    return [(name, *_entry_to_pose(entry))
            for name, entry in _load_locations(WAYPOINTS_FILE).items()]


def save_waypoints(path, waypoints, header=''):
    """[(名前, 位置, 回転行列), ...]を load_waypoints() で読める形式のyamlに書き出す。

    headerは先頭に書くコメント(各行を'# 'で始めること)。名前の辞書順が巡る順番に
    なるようにしておくこと(yamlはキーの辞書順で書き出す)。
    """
    data = {}
    for name, pos, rot in waypoints:
        q = matrix_to_quat(rot) + 0.0  # -0.0 を 0.0 にする
        data[name] = {
            'rotation': {'w': float(q[3]), 'x': float(q[0]), 'y': float(q[1]), 'z': float(q[2])},
            'translation': {'x': float(pos[0]), 'y': float(pos[1]), 'z': float(pos[2])},
        }
    with open(path, 'w', encoding='utf-8') as handle:
        handle.write(header)
        yaml.safe_dump(data, handle, sort_keys=True, default_flow_style=False)


def _quat_angle(q0, q1):
    """2つのクォータニオン[x, y, z, w]の間の回転角[rad]。"""
    return 2.0 * math.acos(min(1.0, abs(float(np.dot(q0, q1)))))


def travel_time(pos0, quat0, pos1, quat1):
    """(pos0, quat0)から(pos1, quat1)への移動にかかる時間の見積もり[s]。

    ロボットは移動と回転を別々に行うようなので、それぞれの時間を足す。
    """
    dist = float(np.linalg.norm(np.asarray(pos1, dtype=float) - np.asarray(pos0, dtype=float)))
    angle = math.degrees(_quat_angle(quat0, quat1))
    return dist / ROBOT_LIN_SPEED_MPS + angle / ROBOT_ROT_SPEED_DEGPS


def order_by_travel_time(start_pos, start_quat, poses):
    """(start_pos, start_quat)から、移動時間が一番短い姿勢へ順に進む順番でposesを並べ替える。

    posesの各要素は(位置[x, y, z], クォータニオン[x, y, z, w])(撮れなかった場所)。
    """
    remaining = list(poses)
    ordered = []
    current = (start_pos, start_quat)
    while remaining:
        k = min(range(len(remaining)),
                key=lambda i: travel_time(*current, remaining[i][0], remaining[i][1]))
        ordered.append(remaining.pop(k))
        current = ordered[-1]
    return ordered


class ThreeDGSImagePicture(Node):
    """移動量に応じた撮影 + 経路の地点への絶対移動を行うノード。"""

    def __init__(self):
        super().__init__('threedgs_image_picture_node')

        self._subs = {name: ImageSubscriber(self, topic) for name, topic in CAMERA_TOPICS.items()}
        self._ctl_client = CtlCommandClient(self)

        self._tf_client = TFClient(self)
        self._expected_spawn_pos, _ = load_spawn_pose()
        self._waypoints = load_waypoints()
        self._start_pos = None  # spawn直後の実位置(iss_bodyフレーム)

        # <output_dir>/<実行開始日時>/<カメラ名>/ に保存する(実行ごとにフォルダを分ける)。
        output_dir = self.declare_parameter('output_dir', DEFAULT_OUTPUT_DIR).value
        run_dir = os.path.join(
            os.path.expanduser(output_dir), datetime.now().strftime('%Y%m%d_%H%M%S')
        )
        self._save_dirs = {name: os.path.join(run_dir, name) for name in CAMERA_TOPICS}
        for name, save_dir in self._save_dirs.items():
            os.makedirs(save_dir, exist_ok=True)
            self.get_logger().info(f'[3dgs] {name}: saving pictures under {save_dir}')

        self._shot_count = 0
        self._stale_count = 0  # 画像が古くて保存しなかった回数
        self._revisit_count = 0  # 撮り直しで保存できた回数
        self._last_capture_pose = None  # 前回撮影したときの(位置, クォータニオン, TFの時刻)
        self._last_saved_stamps = {name: None for name in CAMERA_TOPICS}
        self._missed = []  # 撮れなかった場所の(位置, クォータニオン)
        # 移動を始めるまでは撮影しない。
        self._capturing = False
        self._capture_timer = self.create_timer(CAPTURE_CHECK_PERIOD_SEC, self._on_capture_timer)

    def _moved_since_last_capture(self, pose, min_dist_m, min_angle_deg) -> bool:
        if self._last_capture_pose is None:
            return True
        (pos, quat, _), (last_pos, last_quat, _) = pose, self._last_capture_pose
        return (float(np.linalg.norm(pos - last_pos)) >= min_dist_m
                or math.degrees(_quat_angle(quat, last_quat)) >= min_angle_deg)

    def _on_capture_timer(self) -> None:
        """移動中: 前回の撮影から十分に動いていれば撮る。"""
        if not self._capturing:
            return
        pose = self._lookup_pose()
        if pose is not None and self._moved_since_last_capture(
                pose, CAPTURE_MIN_DIST_M, CAPTURE_MIN_ANGLE_DEG):
            self._capture(pose)

    def _capture_on_arrival(self) -> None:
        """地点に着いたとき: 前回の撮影とほぼ同じ姿勢でなければ撮る。"""
        pose = self._lookup_pose()
        if pose is None:
            self.get_logger().warn('[3dgs] TF not available on arrival, skipping capture')
            return
        if self._moved_since_last_capture(pose, ARRIVAL_MIN_DIST_M, ARRIVAL_MIN_ANGLE_DEG):
            self._capture(pose)

    def _capture(self, pose) -> None:
        """画像がすべて新しければ保存する。古ければ保存せず、撮れなかった場所に記録する。

        どちらの場合も、撮影のきっかけ(前回の撮影位置)はこの姿勢に更新する。
        """
        self._last_capture_pose = pose
        stale = self._stale_cameras(pose[2])
        if stale:
            self._stale_count += 1
            self._missed.append((pose[0], pose[1]))
            self.get_logger().warn(
                f'[3dgs] stale image from {stale}, not saved; will revisit '
                f'pos={np.round(pose[0], 3).tolist()}'
            )
            return
        self._save_images(pose)

    def _stale_cameras(self, pose_stamp):
        """新しくない画像のカメラ名のリスト(すべて新しければ空)。"""
        stale = []
        for name, sub in self._subs.items():
            if (sub.cv_image is None or sub.stamp is None
                    or sub.stamp == self._last_saved_stamps[name]
                    or sub.stamp < pose_stamp - IMAGE_MAX_LAG_SEC):
                stale.append(name)
        return stale

    def _save_images(self, pose, reason='') -> None:
        """すべてのカメラの最新画像を保存する。"""
        self._shot_count += 1
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
        self.get_logger().info(
            f'[3dgs] shot {self._shot_count:04d}{reason} pos={np.round(pose[0], 3).tolist()}'
        )
        for name, sub in self._subs.items():
            filename = f'{name}_{self._shot_count:04d}_{timestamp}.jpg'
            path = os.path.join(self._save_dirs[name], filename)
            if cv2.imwrite(path, sub.cv_image):
                self._last_saved_stamps[name] = sub.stamp
                self.get_logger().info(f'[3dgs] saved {path}')
            else:
                self.get_logger().warn(f'[3dgs] failed to save {path}')

    def _lookup_pose(self):
        """TFから(位置, 姿勢クォータニオン, TFの時刻[s])を読み取る。取得できなければNone。"""
        pose = self._tf_client.lookup_pose_stamped(TF_REFERENCE_FRAME, TF_TARGET_FRAME)
        if pose is None:
            return None
        pos, quat, stamp = pose
        return np.array(pos), np.array(quat), stamp

    def _wait_for_start_pose(self, timeout_sec: float = TF_STARTUP_TIMEOUT_SEC) -> bool:
        """実位置がTFで安定するのを待ってから、原点(spawn)として記録する。

        起動直後の数秒間はissdyn等のプラグインが追いついておらず、TFが原点付近
        の仮の値を返すことがある(stamp=0のダミーとは別物)。そのため最初に取れた
        値をすぐ採用せず、SETTLE_TOLERANCE_M以内でSETTLE_DURATION_SEC秒以上
        変化しなくなるまで待つ。安定後、spawn_locations.yamlの本来のspawn位置
        から大きくずれている場合は、前回の実行やコリジョンでロボットが動いた
        ままシミュレータがリセットされていない可能性が高いため中断する。
        """
        deadline = time.monotonic() + timeout_sec
        last_pos = None
        stable_since = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            pose = self._lookup_pose()
            if pose is None:
                last_pos, stable_since = None, None
                continue

            pos = pose[0]
            now = time.monotonic()
            if last_pos is None or float(np.linalg.norm(pos - last_pos)) > SETTLE_TOLERANCE_M:
                last_pos, stable_since = pos, now
                continue
            if now - stable_since < SETTLE_DURATION_SEC:
                continue

            # SETTLE_DURATION_SEC秒以上安定した値を採用する。
            offset = float(np.linalg.norm(pos - self._expected_spawn_pos))
            if offset > SPAWN_POSITION_TOLERANCE_M:
                self.get_logger().error(
                    f'[3dgs] current pos(iss_body)={pos.tolist()} is {offset:.2f}m away '
                    f'from the expected spawn position {self._expected_spawn_pos.tolist()} '
                    f'(tolerance {SPAWN_POSITION_TOLERANCE_M}m). The robot was likely left '
                    'displaced by a previous run/collision and the simulator was not reset. '
                    'Reset the simulator (respawn to spawn_locations.yaml) before retrying.'
                )
                return False
            self._start_pos = pos
            self.get_logger().info(
                f'[3dgs] start pose recorded (settled): pos={self._start_pos.tolist()}'
            )
            return True
        self.get_logger().error(
            f'[3dgs] TF {TF_REFERENCE_FRAME} <- {TF_TARGET_FRAME} did not settle '
            f'within {timeout_sec:.1f}s; aborting'
        )
        return False

    def _to_move_frame(self, pos, rot, timeout_sec: float = TF_LOOKUP_TIMEOUT_SEC):
        """iss_bodyの(位置, 回転行列)をdock_bodyの(x, y, z, roll, pitch, yaw)に変換する。

        TF(MOVE_TARGET_FRAME <- WAYPOINTS_FRAME)はURDFの固定関節による静的なTFで
        stamp=0になるため、allow_zero_stamp=Trueで読む。取れない間はリトライし、
        timeout_sec以内に取得できなければNoneを返す。
        """
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            frame_pose = self._tf_client.lookup_pose(
                MOVE_TARGET_FRAME, WAYPOINTS_FRAME, allow_zero_stamp=True
            )
            if frame_pose is None:
                continue
            frame_pos, frame_quat = frame_pose
            frame_rot = quat_to_matrix(*frame_quat)
            x, y, z = frame_rot @ pos + np.array(frame_pos)
            return (x, y, z) + matrix_to_rpy(frame_rot @ rot)
        self.get_logger().error(
            f'[3dgs] TF {MOVE_TARGET_FRAME} <- {WAYPOINTS_FRAME} not available within '
            f'{timeout_sec:.1f}s'
        )
        return None

    def _move_to(self, label, position, rot) -> bool:
        """iss_bodyの(位置, 回転行列)へ絶対移動し、着くまで待つ。"""
        target = self._to_move_frame(position, rot)
        if target is None:
            self.get_logger().error(f'[3dgs] {label}: TF lookup failed')
            return False
        x, y, z, roll, pitch, yaw = target
        self.get_logger().info(
            f'[3dgs] {label}: iss_body pos={np.round(position, 3).tolist()} '
            f'camera={np.round(rot[:, 0], 2).tolist()} -> {MOVE_TARGET_FRAME} '
            f'pos=({x:.3f}, {y:.3f}, {z:.3f}) rpy=({math.degrees(roll):.1f}, '
            f'{math.degrees(pitch):.1f}, {math.degrees(yaw):.1f})deg'
        )
        if not self._ctl_client.send_absolute_move(x, y, z, roll, pitch, yaw):
            self.get_logger().error(f'[3dgs] {label}: move failed')
            return False
        return True

    def _wait_and_capture(self, timeout_sec: float = REVISIT_IMAGE_WAIT_SEC) -> bool:
        """止まった状態で、すべてのカメラの新しい画像を待って保存する。撮れたかを返す。"""
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            pose = self._lookup_pose()
            if pose is not None and not self._stale_cameras(pose[2]):
                self._save_images(pose, reason=' (revisit)')
                return True
        return False

    def _revisit_missed(self) -> bool:
        """撮れなかった場所へ移動時間が短い順に戻って撮り直す。移動に失敗したらFalseを返す。"""
        for round_no in range(1, REVISIT_MAX_ROUNDS + 1):
            if not self._missed:
                return True
            current = self._lookup_pose()
            start = current[:2] if current is not None else self._missed[0]
            targets = order_by_travel_time(*start, self._missed)
            self._missed = []
            self.get_logger().info(
                f'[3dgs] revisit round {round_no}/{REVISIT_MAX_ROUNDS}: {len(targets)} places'
            )
            for i, (pos, quat) in enumerate(targets, start=1):
                label = f'revisit {round_no}-{i}/{len(targets)}'
                if not self._move_to(label, pos, quat_to_matrix(*quat)):
                    return False
                if self._wait_and_capture():
                    self._revisit_count += 1
                else:
                    self.get_logger().warn(
                        f'[3dgs] {label}: no fresh image within {REVISIT_IMAGE_WAIT_SEC:.0f}s'
                    )
                    self._missed.append((pos, quat))
        return True

    def run_move_sequence(self) -> bool:
        """移動量に応じて撮影しながら経路の地点を順に巡り、撮れなかった場所を撮り直す。"""
        if not self._ctl_client.wait_for_server():
            return False
        if not self._wait_for_start_pose():
            return False

        self._capturing = True
        self._capture_on_arrival()  # スタート地点で1回撮る
        total = len(self._waypoints)
        for i, (label, position, rot) in enumerate(self._waypoints, start=1):
            if not self._move_to(f'waypoint {i}/{total} ({label})', position, rot):
                self.get_logger().error('[3dgs] aborting sequence')
                return False
            self._capture_on_arrival()
        # 撮り直しの移動中は、タイマーでは撮影しない。
        self._capturing = False

        if not self._revisit_missed():
            self.get_logger().error('[3dgs] aborting revisit')
            return False
        self.get_logger().info(
            f'[3dgs] move sequence complete: {self._shot_count} shots x '
            f'{len(CAMERA_TOPICS)} cameras (revisit {self._revisit_count}), '
            f'stale {self._stale_count} times, still missed {len(self._missed)}'
        )
        for pos, _quat in self._missed:
            self.get_logger().warn(f'[3dgs] still missed: pos={np.round(pos, 3).tolist()}')
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ThreeDGSImagePicture()
    try:
        node.run_move_sequence()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
