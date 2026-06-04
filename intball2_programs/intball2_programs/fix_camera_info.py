# Copyright 2026 intball2
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Republish camera_info with frame_id changed to optical frame convention."""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo


class FixCameraInfo(Node):
    """Change camera_info frame_id from *_link to *_optical_frame and republish."""

    def __init__(self):
        super().__init__('fix_camera_info')
        self._pub_left = self.create_publisher(
            CameraInfo, '/camera_left/camera_info_fixed', qos_profile_sensor_data)
        self._pub_right = self.create_publisher(
            CameraInfo, '/camera_right/camera_info_fixed', qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, '/camera_left/camera_info', self._cb_left, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, '/camera_right/camera_info', self._cb_right, qos_profile_sensor_data)

    def _cb_left(self, msg):
        msg.header.frame_id = 'cameraL_optical_frame'
        # Left camera is the reference: Tx (P[0,3]) must be 0.
        # Gazebo publishes the same non-zero Tx for both cameras, which breaks stereo depth.
        p = list(msg.p)
        p[3] = 0.0
        msg.p = p
        self._pub_left.publish(msg)

    def _cb_right(self, msg):
        msg.header.frame_id = 'cameraR_optical_frame'
        # Gazebo publishes a negative Tx for both cameras. For stereo depth to yield
        # positive Z (scene in front), the right camera's Tx must be positive when
        # disparities are negative (Gazebo's reversed-baseline convention).
        p = list(msg.p)
        p[3] = abs(p[3])
        msg.p = p
        self._pub_right.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FixCameraInfo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
