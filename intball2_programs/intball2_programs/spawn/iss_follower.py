"""ISSからの相対位置に置いたGazeboモデルを、ISSの動き(ドリフト)に追従させる部品。

ISS自体はGazebo world座標上で動くため、spawn後も周期的にISSの現在姿勢と相対位置を
合成し直し、`/gazebo/set_model_state`を送り続ける必要がある。

ISS姿勢は`/gazebo/get_model_state`で必要な分だけ取得する。`/gazebo/model_states`は
約930Hzで届き、購読し続けるだけでCPUを約1コア使うため使わない
(docs/light_spawn_load_measurement.md参照)。
"""
import time

import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import Point, Pose

from intball2_programs.ros import GetModelStateServiceClient, ModelStatePublisher
from intball2_programs.spawn.quaternion import quaternion_multiply, rotate_vector

ISS_MODEL_NAME = 'iss'
WORLD_FRAME = 'world'


class IssFollower:
    def __init__(self, node):
        self._node = node
        self._get_state = GetModelStateServiceClient(node)
        self._model_state_pub = ModelStatePublisher(node)
        self.iss_pose = None

    def wait_until_ready(self, timeout_sec=10.0):
        """サービスを待ち、ISS姿勢を取得する。timeout_sec以内に取得できなければFalse。

        bridge起動直後などに1回目の呼び出しだけ失敗することがあるため、期限内は再試行する。
        """
        if not self._get_state.wait_for_service(timeout_sec=timeout_sec):
            return False
        deadline = self._node.get_clock().now() + Duration(seconds=timeout_sec)
        while rclpy.ok() and self._node.get_clock().now() < deadline:
            self.iss_pose = self._get_state.call(ISS_MODEL_NAME)
            if self.iss_pose is not None:
                return True
            time.sleep(0.5)
        self._node.get_logger().error(f'Failed to get pose of model: {ISS_MODEL_NAME}')
        return False

    def exists(self, model_name):
        """model_nameのモデルがGazebo上に存在するか。"""
        return self._get_state.call(model_name) is not None

    def request_update(self):
        """ISS姿勢の更新を非同期に要求する(周期処理の先頭で呼ぶ。反映は次のspin以降)。"""
        self._get_state.call_async(ISS_MODEL_NAME, self._on_iss_pose)

    def _on_iss_pose(self, pose):
        self.iss_pose = pose

    def to_world_pose(self, offset=(0.0, 0.0, 0.0), orientation=None):
        """ISS座標系での位置offsetと姿勢orientationを、world座標のPoseに変換する。

        orientation省略時はISSと同じ向き。
        """
        iss = self.iss_pose
        offset_rot = rotate_vector(iss.orientation, offset)
        position = Point(
            x=iss.position.x + offset_rot[0],
            y=iss.position.y + offset_rot[1],
            z=iss.position.z + offset_rot[2],
        )
        if orientation is None:
            world_orientation = iss.orientation
        else:
            world_orientation = quaternion_multiply(iss.orientation, orientation)
        return Pose(position=position, orientation=world_orientation)

    def publish(self, model_name, offset=(0.0, 0.0, 0.0), orientation=None):
        """model_nameを、ISSの現在姿勢から見てoffset/orientationの位置へ移動させる。"""
        if self.iss_pose is None:
            return
        self._model_state_pub.publish(
            model_name, self.to_world_pose(offset, orientation), WORLD_FRAME
        )
