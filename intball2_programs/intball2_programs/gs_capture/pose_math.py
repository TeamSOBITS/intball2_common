#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""3DGS撮影スクリプトで使う姿勢計算(ROSに依存しない)。

回転は3x3回転行列で扱う。rpyの規約は ctl_command_client.quaternion_from_euler と
同じ内因性Z-Y-X(R = Rz(yaw) * Ry(pitch) * Rx(roll))。
"""
import math

import numpy as np


def quat_to_matrix(x, y, z, w):
    """クォータニオン[x, y, z, w]を3x3回転行列にする。"""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat(rot):
    """3x3回転行列をクォータニオン[x, y, z, w](w >= 0)にする。"""
    m = np.asarray(rot, dtype=float)
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0:
        s = 2.0 * math.sqrt(trace + 1.0)
        q = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        q = [0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s]
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        q = [(m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s]
    else:
        s = 2.0 * math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        q = [(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s]
    q = np.array(q)
    q /= np.linalg.norm(q)
    return q if q[3] >= 0 else -q


def rpy_to_matrix(roll, pitch, yaw):
    """(roll, pitch, yaw)[rad]から3x3回転行列を返す(URDFのrpyも同じ規約)。"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def matrix_to_rpy(rot):
    """3x3回転行列から(roll, pitch, yaw)[rad]を返す(quaternion_from_eulerの逆変換)。"""
    roll = math.atan2(rot[2, 1], rot[2, 2])
    pitch = math.asin(max(-1.0, min(1.0, -rot[2, 0])))
    yaw = math.atan2(rot[1, 0], rot[0, 0])
    return roll, pitch, yaw


def camera_rotation(camera_dir, up_hint):
    """body +x(camera_main)がcamera_dirを向き、画像の上がup_hint寄りになる回転行列。

    bodyは x=前(カメラ), y=右, z=下 の座標系なので、z軸はup_hintの逆向きにする。
    up_hintはcamera_dirと垂直な成分だけを使う(平行だと向きが決まらないので不可)。
    """
    bx = np.asarray(camera_dir, dtype=float)
    bx = bx / np.linalg.norm(bx)
    up = np.asarray(up_hint, dtype=float)
    up = up - np.dot(up, bx) * bx
    if np.linalg.norm(up) < 1e-6:
        raise ValueError(f'up_hint {up_hint} is parallel to camera_dir {camera_dir}')
    bz = -up / np.linalg.norm(up)
    by = np.cross(bz, bx)
    return np.column_stack([bx, by, bz])
