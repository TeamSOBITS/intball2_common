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
from sensor_msgs_py.point_cloud2 import dtype_from_fields


def filter_pointcloud(msg, min_x, max_x, min_y, max_y, min_z, max_z):
    """Return a new PointCloud2 keeping all original fields, filtered by XYZ bounds."""
    dtype = dtype_from_fields(msg.fields, point_step=msg.point_step)
    pts = np.frombuffer(bytes(msg.data), dtype=dtype)
    x = pts['x'].astype(np.float32)
    y = pts['y'].astype(np.float32)
    z = pts['z'].astype(np.float32)
    mask = (
        np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        & (x >= min_x) & (x <= max_x)
        & (y >= min_y) & (y <= max_y)
        & (z >= min_z) & (z <= max_z)
    )
    filtered = pts[mask]
    out = PointCloud2()
    out.header = msg.header
    out.height = 1
    out.width = len(filtered)
    out.fields = msg.fields
    out.is_bigendian = msg.is_bigendian
    out.point_step = msg.point_step
    out.row_step = msg.point_step * len(filtered)
    out.data = filtered.tobytes()
    out.is_dense = True
    return out


class CropPointCloud(Node):
    """Subscribe /stereo/points2, publish XYZ-filtered /stereo/points2_filtered."""

    def __init__(self):
        super().__init__('crop_pointcloud')
        self._bounds = (-1.5, 1.0, -0.3, 2.0, 0.2, 3.0)
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
