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

"""CropPointCloud node: filters PointCloud2 by XYZ bounding box."""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32, read_points_numpy


def filter_pointcloud(msg, min_x, max_x, min_y, max_y, min_z, max_z):
    """Return a new PointCloud2 containing only points within the given bounds."""
    pts = read_points_numpy(msg, field_names=['x', 'y', 'z'], skip_nans=True)
    if pts.size == 0:
        return create_cloud_xyz32(msg.header, pts)
    mask = (
        (pts[:, 0] >= min_x) & (pts[:, 0] <= max_x)
        & (pts[:, 1] >= min_y) & (pts[:, 1] <= max_y)
        & (pts[:, 2] >= min_z) & (pts[:, 2] <= max_z)
    )
    return create_cloud_xyz32(msg.header, pts[mask])


class CropPointCloud(Node):
    """Subscribe /stereo/points2, publish XYZ-filtered /stereo/points2_filtered."""

    def __init__(self):
        super().__init__('crop_pointcloud')
        self._bounds = (-2.0, 2.0, -2.0, 2.0, -0.1, 5.0)
        self._pub = self.create_publisher(PointCloud2, '/stereo/points2_filtered', 10)
        self.create_subscription(PointCloud2, '/stereo/points2', self._cb, 10)

    def _cb(self, msg):
        self._pub.publish(filter_pointcloud(msg, *self._bounds))


def main(args=None):
    rclpy.init(args=args)
    node = CropPointCloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
