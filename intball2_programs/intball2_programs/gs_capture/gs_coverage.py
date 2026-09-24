#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""3DGS撮影経路の評価(JPMメッシュのどこが、どの方向から写るか)。ROSに依存しない計算。

gs_optimize_path から使う。メッシュの位置は urdf/iss.urdf、カメラの取り付け位置は
urdf/ib2.urdf、撮影に使うカメラと撮影のきっかけは gs_3d_image_picture の定数を読む
ので、それらを変えれば評価も合わせて変わる。

必要なライブラリ(撮影ノードには不要なので package.xml には入れていない):

    pip3 install --user "trimesh==4.0.10" pycollada rtree embreex

trimesh は numpy 1.21(Ubuntu 22.04 標準)で動く 4.0.x を使う。embreex が無くても
動くが、光線の計算が約1000倍遅くなる。
"""
import math
import os
import xml.etree.ElementTree as ElementTree

import numpy as np
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation, Slerp

from intball2_programs.gs_capture.gs_3d_image_picture import (
    ARRIVAL_MIN_ANGLE_DEG,
    ARRIVAL_MIN_DIST_M,
    CAMERA_TOPICS,
    CAPTURE_MIN_ANGLE_DEG,
    CAPTURE_MIN_DIST_M,
    ROBOT_LIN_SPEED_MPS,
    ROBOT_ROT_SPEED_DEGPS,
)
from intball2_programs.gs_capture.pose_math import rpy_to_matrix

try:
    import trimesh
    _TRIMESH_IMPORT_ERROR = None
except ImportError as e:
    trimesh = None
    _TRIMESH_IMPORT_ERROR = e

try:
    from trimesh.ray.ray_pyembree import RayMeshIntersector as _EmbreeIntersector
except ImportError:
    _EmbreeIntersector = None

INSTALL_HINT = 'pip3 install --user "trimesh==4.0.10" pycollada rtree embreex'

# camera_info(fx=fy=476.7, 800x800px)から求めた視野は 80度 x 80度。
HALF_FOV_RAD = math.radians(40.0)
MAX_INCIDENCE_RAD = math.radians(75.0)  # これより斜めに見た面は写ったとみなさない
MAX_RANGE_M = 4.0
# 所要時間の見積もり用(gs_3d_image_picture の定数を使う)。
LIN_SPEED_MPS = ROBOT_LIN_SPEED_MPS
ROT_SPEED_RADPS = math.radians(ROBOT_ROT_SPEED_DEGPS)
SIM_STEP_SEC = 0.2  # 経路をこの秒数ごとに区切って撮影判定する

# gs_3d_image_picture のカメラ名 → ib2.urdf のカメラの関節名
CAMERA_JOINTS = {
    'camera_main': 'cameraF_joint',
    'camera_left': 'cameraL_joint',
    'camera_right': 'cameraR_joint',
}
JPM_VISUAL_NAME = 'jpm'
# 通路の表面として評価する範囲(iss_bodyのy)と、内側から見える面を選ぶための格子点。
SURFACE_Y_RANGE = (-10.6, -2.34)
INTERIOR_GRID = [(x, y, z) for x in (10.3, 10.9, 11.5) for z in (4.3, 4.9, 5.5)
                 for y in np.arange(-3.0, -10.0, -0.5)]


def require_mesh_libraries():
    """メッシュ計算に必要なライブラリが無ければ、入れ方を添えてImportErrorにする。"""
    if trimesh is None:
        raise ImportError(
            f'trimesh is required for path evaluation ({_TRIMESH_IMPORT_ERROR}). '
            f'Install with: {INSTALL_HINT}'
        )


def _share_path(*parts):
    return os.path.join(get_package_share_directory('intball2_programs'), *parts)


def _origin(element):
    """URDFの<origin>から(位置, 回転行列)を返す。"""
    origin = element.find('origin')
    xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()]
    rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()]
    return np.array(xyz), rpy_to_matrix(*rpy)


def load_jpm_mesh():
    """iss.urdf の jpm のメッシュを iss_body フレームに置いて読み、(mesh, 光線計算器)を返す。"""
    require_mesh_libraries()
    root = ElementTree.parse(_share_path('urdf', 'iss.urdf')).getroot()
    visual = next(v for v in root.iter('visual') if v.get('name') == JPM_VISUAL_NAME)
    uri = visual.find('geometry/mesh').get('filename')
    mesh_path = _share_path(uri.split('package://intball2_programs/', 1)[1])
    mesh = trimesh.load(mesh_path, force='scene').dump(concatenate=True)
    pos, rot = _origin(visual)
    transform = np.eye(4)
    transform[:3, :3], transform[:3, 3] = rot, pos
    mesh.apply_transform(transform)
    if _EmbreeIntersector is not None:
        return mesh, _EmbreeIntersector(mesh)
    print(f'[gs_coverage] embreex not found; ray casting will be slow ({INSTALL_HINT})')
    return mesh, mesh.ray


def load_camera_mounts(camera_names=None):
    """ib2.urdf から[(カメラ名, bodyからの位置, bodyからの回転行列), ...]を返す。"""
    camera_names = list(CAMERA_TOPICS) if camera_names is None else camera_names
    root = ElementTree.parse(_share_path('urdf', 'ib2.urdf')).getroot()
    joints = {j.get('name'): j for j in root.iter('joint')}
    return [(name, *_origin(joints[CAMERA_JOINTS[name]])) for name in camera_names]


def direction_bins():
    """見る方向を区別するための42方向(隣どうし約30度)。"""
    require_mesh_libraries()
    return trimesh.creation.icosphere(1).vertices


def first_hit(ray, origins, dirs):
    """各光線が最初にメッシュに当たる距離(当たらなければinf)。"""
    out = np.full(len(origins), np.inf)
    if len(origins) == 0:
        return out
    _, idx, loc = ray.intersects_id(origins, dirs, multiple_hits=False, return_locations=True)
    out[idx] = np.linalg.norm(loc - origins[idx], axis=1)
    return out


def clearance(mesh, points):
    """各点からメッシュ表面までの最短距離(正確な値)。"""
    _, dist, _ = trimesh.proximity.ProximityQuery(mesh).on_surface(np.asarray(points))
    return dist


def interior_surface(mesh, ray, n=6000, seed=0):
    """通路の内側から見える表面の点と法線をn個サンプリングする。"""
    rng = np.random.default_rng(seed)
    pts, fidx = trimesh.sample.sample_surface(mesh, n * 5, seed=seed)
    box = (pts[:, 1] > SURFACE_Y_RANGE[0]) & (pts[:, 1] < SURFACE_Y_RANGE[1])
    pts, nrm = pts[box], mesh.face_normals[fidx[box]]
    seen = np.zeros(len(pts), bool)
    for g in np.array(INTERIOR_GRID):
        v = pts - g
        dist = np.linalg.norm(v, axis=1)
        hit = first_hit(ray, np.repeat(g[None], len(pts), 0), v / dist[:, None])
        seen |= hit >= dist - 0.02
    pts, nrm = pts[seen], nrm[seen]
    if len(pts) > n:
        keep = rng.choice(len(pts), n, replace=False)
        pts, nrm = pts[keep], nrm[keep]
    return pts, nrm


def visible_from(ray, pts, nrm, cam_pos, cam_rot):
    """カメラ(位置, 回転)から写る点のindexと、点→カメラの単位ベクトルを返す。"""
    v = pts - cam_pos
    fwd = v @ cam_rot[:, 0]
    t = math.tan(HALF_FOV_RAD)
    ok = (fwd > 0.05) & (np.abs(v @ cam_rot[:, 1]) <= fwd * t) \
        & (np.abs(v @ cam_rot[:, 2]) <= fwd * t)
    dist = np.linalg.norm(v, axis=1)
    ok &= dist <= MAX_RANGE_M
    vd = -v / dist[:, None]
    # 法線の表裏はメッシュによって違うので絶対値で見る(裏側は遮蔽判定で除かれる)
    ok &= np.abs((vd * nrm).sum(1)) > math.cos(MAX_INCIDENCE_RAD)
    idx = np.where(ok)[0]
    hit = first_hit(ray, np.repeat(cam_pos[None], len(idx), 0), -vd[idx])
    vis = idx[hit >= dist[idx] - 0.02]
    return vis, vd[vis]


def camera_poses(mounts, body_pos, body_rot):
    """ロボットの姿勢から、各カメラの(名前, 位置, 回転行列)を返す。"""
    return [(name, body_pos + body_rot @ off, body_rot @ rot) for name, off, rot in mounts]


def rot_angle(r0, r1):
    """2つの回転行列の間の回転角[rad]。"""
    return (Rotation.from_matrix(r0).inv() * Rotation.from_matrix(r1)).magnitude()


def segment_time(p0, r0, p1, r1):
    """(p0, r0)から(p1, r1)への移動にかかる時間の見積もり[s]。"""
    dist = np.linalg.norm(np.asarray(p1) - np.asarray(p0))
    return max(dist / LIN_SPEED_MPS, rot_angle(r0, r1) / ROT_SPEED_RADPS, 1.0)


def simulate_shots(keys):
    """keys=[(位置, 回転行列), ...]を順に動いたときの撮影姿勢のリストと総時間[s]。

    gs_3d_image_picture と同じく、前回の撮影から CAPTURE_MIN_DIST_M 以上動いた/
    CAPTURE_MIN_ANGLE_DEG 以上回ったときと、各地点に着いたとき(前回とほぼ同じ姿勢
    なら除く)に撮影する。移動は直線、回転は一定の速さと仮定する。
    """
    shots, total = [keys[0]], 0.0
    last_p, last_r = keys[0]

    def moved(p, r, dist_m, angle_deg):
        return (np.linalg.norm(p - last_p) >= dist_m
                or math.degrees(rot_angle(last_r, r)) >= angle_deg)

    for (p0, r0), (p1, r1) in zip(keys[:-1], keys[1:]):
        dur = segment_time(p0, r0, p1, r1)
        slerp = Slerp([0, 1], Rotation.from_matrix([r0, r1]))
        for s in np.arange(SIM_STEP_SEC, dur, SIM_STEP_SEC):
            a = s / dur
            p, r = p0 + (p1 - p0) * a, slerp(a).as_matrix()
            if moved(p, r, CAPTURE_MIN_DIST_M, CAPTURE_MIN_ANGLE_DEG):
                shots.append((p, r))
                last_p, last_r = p, r
        if moved(p1, r1, ARRIVAL_MIN_DIST_M, ARRIVAL_MIN_ANGLE_DEG):
            shots.append((p1, r1))
            last_p, last_r = p1, r1
        total += dur
    return shots, total


def region_of(p):
    """表面の点が通路のどの面か(iss_bodyの座標で大まかに分ける)。"""
    x, y, z = p
    if y < -9.8:
        return '奥の壁'
    if y > -3.0:
        return '手前(入口側)'
    if z < 4.1:
        return '床'
    if z > 5.7:
        return '天井'
    if x < 10.1:
        return '右の壁'
    if x > 11.7:
        return '左の壁'
    return 'その他'


REGIONS = ['床', '天井', '右の壁', '左の壁', '奥の壁', '手前(入口側)', 'その他', '全体']


def evaluate(ray, pts, nrm, mounts, keys):
    """経路(keys)を撮影したときの評価を返す。

    戻り値は(場所ごとの評価の辞書, 総時間[s], 撮影回数)。評価は、写った割合(seen)、
    5枚以上写った割合(five)、3方向以上から写った割合(dirs3)、平均の方向数(dirs_mean)。
    """
    bins_dirs = direction_bins()
    shots, total = simulate_shots(keys)
    count = np.zeros(len(pts), int)
    bins = np.zeros((len(pts), len(bins_dirs)), bool)
    for body_pos, body_rot in shots:
        for _, cam_pos, cam_rot in camera_poses(mounts, body_pos, body_rot):
            vis, vd = visible_from(ray, pts, nrm, cam_pos, cam_rot)
            count[vis] += 1
            bins[vis, np.argmax(vd @ bins_dirs.T, axis=1)] = True
    nbins = bins.sum(1)
    reg = np.array([region_of(p) for p in pts])
    rows = {}
    for name in REGIONS:
        sel = np.ones(len(pts), bool) if name == '全体' else reg == name
        if sel.sum() == 0:
            continue
        rows[name] = dict(area=sel.mean(), seen=(count[sel] > 0).mean(),
                          five=(count[sel] >= 5).mean(), dirs3=(nbins[sel] >= 3).mean(),
                          dirs_mean=nbins[sel].mean())
    return rows, total, len(shots)


def print_table(title, rows, total, n_shots, n_cameras):
    """evaluate() の結果を表にして表示する。"""
    print(f'\n=== {title}  (所要 約{total / 60:.1f} 分, 撮影 {n_shots} 回 x {n_cameras}台 '
          f'= {n_shots * n_cameras} 枚) ===')
    print(f'{"場所":<10}{"面積":>6}{"写った":>8}{"5枚以上":>8}{"3方向以上":>10}{"平均方向数":>10}')
    for name, r in rows.items():
        print(f'{name:<10}{r["area"] * 100:5.0f}%{r["seen"] * 100:7.0f}%{r["five"] * 100:7.0f}%'
              f'{r["dirs3"] * 100:9.0f}%{r["dirs_mean"]:10.1f}')
