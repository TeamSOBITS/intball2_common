#!/usr/bin/env python3
"""gazebo_msgs/SetModelStateサービスを呼び出す汎用クライアント。"""
import rclpy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Twist


class SetModelStateServiceClient:
    def __init__(self, node, service='/gazebo/set_model_state'):
        self._node = node
        self._service_name = service
        self._cli = node.create_client(SetModelState, service)

    def wait_for_service(self, timeout_sec=10.0):
        if not self._cli.wait_for_service(timeout_sec=timeout_sec):
            self._node.get_logger().error(f'Service {self._service_name} not available.')
            return False
        return True

    def call(self, model_name, pose, reference_frame, twist=None, timeout_sec=5.0):
        """model_nameの姿勢(と速度)を同期で設定する。成功したらTrue。

        twist省略時は速度0を設定する。
        """
        req = SetModelState.Request()
        req.model_state = ModelState()
        req.model_state.model_name = model_name
        req.model_state.pose = pose
        req.model_state.twist = twist if twist is not None else Twist()
        req.model_state.reference_frame = reference_frame

        future = self._cli.call_async(req)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout_sec)
        if future.result() is None:
            self._node.get_logger().error('SetModelState call timed out.')
            return False
        res = future.result()
        if not res.success:
            self._node.get_logger().error(f'SetModelState failed: {res.status_message}')
        return res.success
