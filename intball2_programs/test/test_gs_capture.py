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

"""Tests for gs_capture pose math and shot simulation."""

import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from intball2_programs.gs_capture import gs_coverage  # noqa: E402
from intball2_programs.gs_capture.gs_3d_image_picture import (  # noqa: E402
    ROBOT_LIN_SPEED_MPS,
    ROBOT_ROT_SPEED_DEGPS,
    order_by_travel_time,
    travel_time,
)
from intball2_programs.gs_capture.pose_math import (  # noqa: E402
    camera_rotation,
    matrix_to_quat,
    matrix_to_rpy,
    quat_to_matrix,
    rpy_to_matrix,
)
from intball2_programs.ros.ctl_command_client import quaternion_from_euler  # noqa: E402

# spawn_locations.yaml の ib2_spawn の姿勢(カメラが通路の奥 -y を向き、上が +z)
SPAWN_QUAT = (-0.707106, 0.707106, 0.0, 0.0)


def _random_rotations(n, seed=0):
    rng = np.random.default_rng(seed)
    for _ in range(n):
        q = rng.normal(size=4)
        yield quat_to_matrix(*(q / np.linalg.norm(q)))


def test_camera_rotation_matches_spawn_pose():
    rot = camera_rotation((0, -1, 0), (0, 0, 1))
    assert np.allclose(rot, quat_to_matrix(*SPAWN_QUAT), atol=1e-5)


def test_camera_rotation_is_proper_rotation():
    rot = camera_rotation((1, 1, 1), (0, 0, 1))
    assert np.allclose(rot.T @ rot, np.eye(3))
    assert np.isclose(np.linalg.det(rot), 1.0)
    assert np.allclose(rot[:, 0], np.ones(3) / math.sqrt(3))


def test_camera_rotation_rejects_parallel_up():
    with pytest.raises(ValueError):
        camera_rotation((0, 0, 1), (0, 0, 1))


def test_matrix_to_quat_round_trip():
    for rot in _random_rotations(20):
        q = matrix_to_quat(rot)
        assert q[3] >= 0
        assert np.allclose(quat_to_matrix(*q), rot)


def test_matrix_to_rpy_matches_ctl_command_convention():
    # send_absolute_move() は quaternion_from_euler(roll, pitch, yaw) で目標姿勢を作る
    for rot in _random_rotations(20, seed=1):
        q = quaternion_from_euler(*matrix_to_rpy(rot))
        assert np.allclose(quat_to_matrix(*q), rot)
        assert np.allclose(rpy_to_matrix(*matrix_to_rpy(rot)), rot)


def test_simulate_shots_straight_move():
    # 1m まっすぐ動くと、CAPTURE_MIN_DIST_M ごとの撮影 + 着いたときの撮影になる
    rot = np.eye(3)
    keys = [(np.zeros(3), rot), (np.array([1.0, 0.0, 0.0]), rot)]
    shots, total = gs_coverage.simulate_shots(keys)
    expected = 1 + math.floor(1.0 / gs_coverage.CAPTURE_MIN_DIST_M)
    assert expected <= len(shots) <= expected + 1
    assert np.allclose(shots[-1][0], [1.0, 0.0, 0.0])
    assert total == pytest.approx(1.0 / gs_coverage.LIN_SPEED_MPS)


def test_simulate_shots_rotation_in_place():
    # その場で90度回ると、CAPTURE_MIN_ANGLE_DEG ごとに撮影する
    keys = [(np.zeros(3), np.eye(3)), (np.zeros(3), rpy_to_matrix(0.0, 0.0, math.pi / 2))]
    shots, _ = gs_coverage.simulate_shots(keys)
    expected = 1 + math.floor(90.0 / gs_coverage.CAPTURE_MIN_ANGLE_DEG)
    assert expected - 1 <= len(shots) <= expected + 1


def _yaw_quat(deg):
    return matrix_to_quat(rpy_to_matrix(0.0, 0.0, math.radians(deg)))


def test_travel_time_adds_move_and_rotation():
    t = travel_time((0.0, 0.0, 0.0), _yaw_quat(0.0), (1.0, 0.0, 0.0), _yaw_quat(90.0))
    assert t == pytest.approx(1.0 / ROBOT_LIN_SPEED_MPS + 90.0 / ROBOT_ROT_SPEED_DEGPS)


def test_order_by_travel_time_visits_closest_position_next():
    q = _yaw_quat(0.0)
    poses = [((3.0, 0.0, 0.0), q), ((1.0, 0.0, 0.0), q), ((2.0, 0.0, 0.0), q),
             ((-0.5, 0.0, 0.0), q)]
    ordered = order_by_travel_time((0.0, 0.0, 0.0), q, poses)
    assert [p[0][0] for p in ordered] == [-0.5, 1.0, 2.0, 3.0]


def test_order_by_travel_time_rotates_in_one_direction():
    # 同じ位置で向きだけ違う場所は、回転が少ない順(0 -> 30 -> 60度)に回る
    pos = (0.0, 0.0, 0.0)
    poses = [(pos, _yaw_quat(60.0)), (pos, _yaw_quat(30.0))]
    ordered = order_by_travel_time(pos, _yaw_quat(0.0), poses)
    yaws = [round(math.degrees(2 * math.atan2(q[2], q[3]))) for _, q in ordered]
    assert yaws == [30, 60]


def test_order_by_travel_time_empty():
    assert order_by_travel_time((0.0, 0.0, 0.0), _yaw_quat(0.0), []) == []
