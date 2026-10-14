#!/usr/bin/env python3
"""gazebo_msgs/GetModelStateサービスを呼び出す汎用クライアント。"""
import rclpy
from gazebo_msgs.srv import GetModelState


class GetModelStateServiceClient:
    def __init__(self, node, service='/gazebo/get_model_state'):
        self._node = node
        self._service_name = service
        self._cli = node.create_client(GetModelState, service)
        self._pending = None

    def wait_for_service(self, timeout_sec=10.0):
        if not self._cli.wait_for_service(timeout_sec=timeout_sec):
            self._node.get_logger().error(f'Service {self._service_name} not available.')
            return False
        return True

    def call(self, model_name, relative_entity_name='world', timeout_sec=5.0):
        """model_nameの姿勢(geometry_msgs/Pose)を同期で返す。モデルが無い/タイムアウトならNone。"""
        future = self._cli.call_async(self._build_request(model_name, relative_entity_name))
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout_sec)
        if future.result() is None:
            self._node.get_logger().warn('GetModelState call timed out.')
            return None
        res = future.result()
        return res.pose if res.success else None

    def call_async(self, model_name, on_pose, relative_entity_name='world'):
        """model_nameの姿勢を非同期に要求し、取得できたらon_pose(pose)を呼ぶ。

        前回の要求が未完了なら何もしない(応答が遅いときに要求を積み上げないため)。
        応答はnodeをspinしている間に処理される。
        """
        if self._pending is not None and not self._pending.done():
            return
        self._pending = self._cli.call_async(self._build_request(model_name, relative_entity_name))

        def _done(future):
            res = future.result()
            if res is not None and res.success:
                on_pose(res.pose)

        self._pending.add_done_callback(_done)

    @staticmethod
    def _build_request(model_name, relative_entity_name):
        req = GetModelState.Request()
        req.model_name = model_name
        req.relative_entity_name = relative_entity_name
        return req
