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

"""Tests for crop_pointcloud filter logic."""

import sys
import os

import numpy as np
import pytest
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32, read_points_numpy
from std_msgs.msg import Header

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from intball2_programs.crop_pointcloud import filter_pointcloud  # noqa: E402


def _make_cloud(points, frame_id='cameraL_optical_frame'):
    header = Header()
    header.frame_id = frame_id
    arr = np.array(points, dtype=np.float32).reshape(-1, 3)
    return create_cloud_xyz32(header, arr)


def test_filter_removes_negative_z():
    """z < -0.1 の点が除去される。"""
    cloud = _make_cloud([[0.0, 0.0, 1.0], [0.0, 0.0, -1.0]])
    result = filter_pointcloud(cloud, -2.0, 2.0, -2.0, 2.0, -0.1, 5.0)
    pts = read_points_numpy(result, field_names=['x', 'y', 'z'], skip_nans=True)
    assert pts.shape[0] == 1
    assert pts[0, 2] == pytest.approx(1.0)


def test_filter_removes_out_of_range_x():
    """x > 2.0 の点が除去される。"""
    cloud = _make_cloud([[0.0, 0.0, 1.0], [3.0, 0.0, 1.0]])
    result = filter_pointcloud(cloud, -2.0, 2.0, -2.0, 2.0, -0.1, 5.0)
    pts = read_points_numpy(result, field_names=['x', 'y', 'z'], skip_nans=True)
    assert pts.shape[0] == 1
    assert pts[0, 0] == pytest.approx(0.0)


def test_filter_empty_cloud():
    """空の点群を渡してもクラッシュしない。"""
    cloud = _make_cloud([])
    result = filter_pointcloud(cloud, -2.0, 2.0, -2.0, 2.0, -0.1, 5.0)
    pts = read_points_numpy(result, field_names=['x', 'y', 'z'], skip_nans=True)
    assert pts.shape[0] == 0


def test_filter_preserves_frame_id():
    """フィルタ後も header.frame_id が保持される。"""
    cloud = _make_cloud([[0.0, 0.0, 1.0]], frame_id='cameraL_optical_frame')
    result = filter_pointcloud(cloud, -2.0, 2.0, -2.0, 2.0, -0.1, 5.0)
    assert result.header.frame_id == 'cameraL_optical_frame'
