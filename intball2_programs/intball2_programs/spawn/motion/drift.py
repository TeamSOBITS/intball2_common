"""spawn後はGazeboの物理に任せてモデルを漂わせるMotion(spawn_model -d)。

- world(-dのみの場合): 初速0のまま放置する(score_managerと同じ)。
       ISSから見ると、ISSの回転で流れていく。
- iss: ISSに対して静止した初速(ISSのslope角速度による剛体速度)を1回だけ与える。
       ISSから見ると、ISSの振動(周期50秒、±数cm)を除いてほぼ静止する。

どちらも周期的な位置の上書きはしない。周期処理ではRViz表示用に、モデルとISSの姿勢を
間引いて非同期に取得するだけ(docs/drift_spawn_plan.md参照)。
"""
from geometry_msgs.msg import Point, Pose, Twist, Vector3

from intball2_programs.ros import GetModelStateServiceClient, SetModelStateServiceClient
from intball2_programs.spawn.iss_constants import ISS_COG_OFFSET, ISS_SLOPE_ANGULAR_VEL
from intball2_programs.spawn.iss_follower import WORLD_FRAME
from intball2_programs.spawn.motion.base import Motion
from intball2_programs.spawn.quaternion import (
    quaternion_conjugate, quaternion_multiply, rotate_vector,
)

DRIFT_ISS = 'iss'
DRIFT_WORLD = 'world'
DRIFT_MODES = (DRIFT_ISS, DRIFT_WORLD)

# RViz表示用の姿勢取得を何tickに1回行うか(0.1秒周期 × 5 = 2Hz)。
POSE_UPDATE_TICKS = 5


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


class DriftMotion(Motion):
    def __init__(self, node, iss_follower, model_name, relative_pos, orientation, mode):
        super().__init__(node, iss_follower, model_name, relative_pos, orientation)
        self._mode = mode
        self._get_state = GetModelStateServiceClient(node)
        self._model_pose = None
        self._tick_count = 0

    def start(self):
        if self._mode == DRIFT_WORLD:
            return True
        return self._set_iss_rest_velocity()

    def _set_iss_rest_velocity(self):
        """ISSに対して静止するよう、spawn位置でのISSの剛体速度を初速として設定する。"""
        set_state = SetModelStateServiceClient(self._node)
        if not set_state.wait_for_service(timeout_sec=5.0):
            return False
        # spawn中にISSが動いた分を反映するため、最新のISS姿勢で位置を計算し直す。
        if not self._iss_follower.wait_until_ready(timeout_sec=5.0):
            return False
        iss = self._iss_follower.iss_pose
        pose = self._iss_follower.to_world_pose(self._relative_pos, self._orientation)

        cog_rot = rotate_vector(iss.orientation, ISS_COG_OFFSET)
        cog = (
            iss.position.x + cog_rot[0],
            iss.position.y + cog_rot[1],
            iss.position.z + cog_rot[2],
        )
        r = (pose.position.x - cog[0], pose.position.y - cog[1], pose.position.z - cog[2])
        vx, vy, vz = _cross(ISS_SLOPE_ANGULAR_VEL, r)
        wx, wy, wz = ISS_SLOPE_ANGULAR_VEL
        twist = Twist(linear=Vector3(x=vx, y=vy, z=vz), angular=Vector3(x=wx, y=wy, z=wz))

        self._node.get_logger().info(
            f'Drift(iss): initial velocity ({vx:.6f}, {vy:.6f}, {vz:.6f}) m/s'
        )
        return set_state.call(self._model_name, pose, WORLD_FRAME, twist)

    def tick(self):
        if self._tick_count % POSE_UPDATE_TICKS == 0:
            self._iss_follower.request_update()
            self._get_state.call_async(self._model_name, self._on_model_pose)
        self._tick_count += 1

    def _on_model_pose(self, pose):
        self._model_pose = pose

    def relative_pose(self):
        iss = self._iss_follower.iss_pose
        model = self._model_pose
        if iss is None or model is None:
            return None
        iss_inv = quaternion_conjugate(iss.orientation)
        x, y, z = rotate_vector(iss_inv, (
            model.position.x - iss.position.x,
            model.position.y - iss.position.y,
            model.position.z - iss.position.z,
        ))
        return Pose(
            position=Point(x=x, y=y, z=z),
            orientation=quaternion_multiply(iss_inv, model.orientation),
        )
