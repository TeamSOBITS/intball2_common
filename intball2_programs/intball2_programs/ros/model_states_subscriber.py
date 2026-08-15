#!/usr/bin/env python3
"""gazebo_msgs/ModelStatesを購読する汎用サブスクライバ。"""
import rclpy
from rclpy.duration import Duration
from gazebo_msgs.msg import ModelStates


class ModelStatesSubscriber:
    def __init__(self, node, topic='/gazebo/model_states', qos=10):
        self._node = node
        self.model_states = None
        node.create_subscription(ModelStates, topic, self._on_message, qos)

    def _on_message(self, msg):
        self.model_states = msg

    def wait_until_received(self, timeout_sec=5.0):
        deadline = self._node.get_clock().now() + Duration(seconds=timeout_sec)
        while rclpy.ok() and self._node.get_clock().now() < deadline:
            if self.model_states is not None:
                return
            rclpy.spin_once(self._node, timeout_sec=0.1)
        raise RuntimeError(f'No ModelStates message received within {timeout_sec}s')

    def get_pose(self, model_name):
        """指定モデルの現在姿勢(geometry_msgs/Pose)を返す。未受信/未検出ならNone。"""
        if self.model_states is None or model_name not in self.model_states.name:
            return None
        return self.model_states.pose[self.model_states.name.index(model_name)]
