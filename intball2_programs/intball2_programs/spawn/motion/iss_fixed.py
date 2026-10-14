"""モデルをISSに固定し、ISSの動きに追従させるMotion(spawn_modelのデフォルト)。"""
from geometry_msgs.msg import Point, Pose

from intball2_programs.spawn.motion.base import Motion


class IssFixedMotion(Motion):
    def tick(self):
        self._iss_follower.request_update()
        self._iss_follower.publish(self._model_name, self._relative_pos, self._orientation)

    def relative_pose(self):
        x, y, z = self._relative_pos
        return Pose(position=Point(x=x, y=y, z=z), orientation=self._orientation)
