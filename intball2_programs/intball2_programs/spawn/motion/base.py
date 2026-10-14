"""spawn後のモデルの動かし方(Motion)の基底クラス。"""


class Motion:
    """spawn_modelがspawn直後と周期処理で呼ぶインターフェース。

    node: rclpyのNode
    iss_follower: ISS姿勢を取得済みのIssFollower
    model_name: Gazebo上のモデルインスタンス名
    relative_pos: iss_body基準の位置[m]
    orientation: iss_body基準の姿勢(geometry_msgs/Quaternion)
    """

    def __init__(self, node, iss_follower, model_name, relative_pos, orientation):
        self._node = node
        self._iss_follower = iss_follower
        self._model_name = model_name
        self._relative_pos = relative_pos
        self._orientation = orientation

    def start(self):
        """spawn直後に1回呼ぶ。失敗したらFalse。"""
        return True

    def tick(self):
        """0.1秒周期で呼ぶ。"""

    def relative_pose(self):
        """RViz表示用。iss_body基準のPoseを返す(未取得ならNone)。"""
        raise NotImplementedError
