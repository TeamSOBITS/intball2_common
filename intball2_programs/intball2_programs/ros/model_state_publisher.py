#!/usr/bin/env python3
"""gazebo_msgs/ModelStateを配信する汎用パブリッシャ。"""
from gazebo_msgs.msg import ModelState


class ModelStatePublisher:
    def __init__(self, node, topic='/gazebo/set_model_state', qos=10):
        self._pub = node.create_publisher(ModelState, topic, qos)

    def publish(self, model_name, pose, reference_frame):
        state = ModelState()
        state.model_name = model_name
        state.pose = pose
        state.reference_frame = reference_frame
        self._pub.publish(state)
