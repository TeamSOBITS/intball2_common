#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""3DGS撮影の経路(止まる地点と順番)を、JPMメッシュを使って最適化する。

1. 通路内の候補位置 x 候補姿勢ごとに、撮影に使うカメラで写る(表面の点, 見る方向)を
   求める(gs_coverage)。
2. 「増える(点, 方向)の価値 / 経路に加えたときに増える撮影回数」が最大の候補を1つずつ
   選び、撮影回数の増え方が一番小さい場所に挿入する。写真の枚数の上限まで繰り返す。
3. 2-optで順番を整え、壁との距離を確かめてから、今の経路と比べて評価を表示する。

結果は locations/gs_capture_waypoints.yaml(gs_3d_image_picture が読む経路)に書き出す。
上書きしたくないときは --output で別の場所を指定するか、--dry-run で評価だけ表示する。
必要なライブラリは gs_coverage を参照。

    ros2 run intball2_programs gs_optimize_path
    ros2 run intball2_programs gs_optimize_path --max-images 300 --dry-run
"""
import argparse
import itertools
import time
from datetime import datetime

import numpy as np

from intball2_programs.gs_capture import gs_coverage as cov
from intball2_programs.gs_capture.gs_3d_image_picture import (
    CAMERA_TOPICS,
    CAPTURE_MIN_ANGLE_DEG,
    CAPTURE_MIN_DIST_M,
    load_spawn_pose,
    load_waypoints,
    save_waypoints,
)
from intball2_programs.gs_capture.pose_math import camera_rotation, matrix_to_quat

# gs_3d_image_picture の DEFAULT_OUTPUT_DIR と同じく、ソースの場所に直接書く。
DEFAULT_OUTPUT = ('/root/colcon_ws/src/intball2_common/intball2_programs/locations/'
                  'gs_capture_waypoints.yaml')
ROBOT_RADIUS_M = 0.10  # Int-Ball2 の半径

# 候補位置(iss_body)。通路の断面 3x3 と、通路方向に0.75m間隔。
CANDIDATE_X = (10.4, 10.9, 11.4)
CANDIDATE_Z = (4.3, 4.9, 5.5)
CANDIDATE_Y = np.arange(-3.8, -9.85, -0.75)
# 候補姿勢: mainカメラの向き(26方向) x 画像の上方向(6方向)
UP_HINTS = ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1))
# 1つの点がすでに持っている方向の数ごとの、新しい方向の価値(4方向で打ち止め)。
DIR_WEIGHTS = np.array([1.0, 0.7, 0.5, 0.3, 0.0])


def candidate_orientations():
    """重複を除いた候補の回転行列のリスト。"""
    rots, seen = [], set()
    for d in itertools.product((-1, 0, 1), repeat=3):
        if not any(d):
            continue
        for u in UP_HINTS:
            if np.linalg.norm(np.cross(np.array(d) / np.linalg.norm(d), u)) < 1e-3:
                continue
            rot = camera_rotation(d, u)
            key = tuple(np.round(matrix_to_quat(rot), 4))
            if key not in seen:
                seen.add(key)
                rots.append(rot)
    return rots


def shots_between(p0, q0, positions, quats):
    """1つの姿勢(p0, q0)から多数の姿勢へ移動したときに増える撮影回数の見積もり。"""
    dist = np.linalg.norm(positions - p0, axis=1)
    ang = 2 * np.arccos(np.clip(np.abs(quats @ q0), 0, 1))
    return np.floor(np.maximum(dist / CAPTURE_MIN_DIST_M,
                               np.degrees(ang) / CAPTURE_MIN_ANGLE_DEG)) + 1.0


def tour_shots(tour):
    """(位置, クォータニオン)の列を回ったときの撮影回数の見積もり。"""
    return 1 + sum(shots_between(a[0], a[1], b[0][None], b[1][None])[0]
                   for a, b in zip(tour[:-1], tour[1:]))


def compute_visibility(ray, pts, nrm, mounts, positions, rots, n_bins):
    """候補ごとに写る(点, 方向)の番号を1本の配列にまとめて返す。"""
    bins_dirs = cov.direction_bins()
    pair_lists, cands = [], []
    for p in positions:
        for rot in rots:
            ids = [vis * n_bins + np.argmax(vd @ bins_dirs.T, axis=1)
                   for _, cp, cr in cov.camera_poses(mounts, p, rot)
                   for vis, vd in [cov.visible_from(ray, pts, nrm, cp, cr)]]
            pair_lists.append(np.unique(np.concatenate(ids)))
            cands.append((p, rot))
    lengths = np.array([len(x) for x in pair_lists])
    offsets = np.concatenate([[0], np.cumsum(lengths)[:-1]])
    return cands, np.concatenate(pair_lists), offsets, lengths


def greedy_tour(cands, pairs, offsets, lengths, n_pts, n_bins, start, max_shots):
    """撮影1回あたりの価値が最大の候補を、一番安い場所に挿入していく。"""
    cand_pos = np.array([c[0] for c in cands])
    cand_q = np.array([matrix_to_quat(c[1]) for c in cands])
    pair_pts = pairs // n_bins
    covered = np.zeros(n_pts * n_bins, bool)
    nbins_pt = np.zeros(n_pts, int)
    used = np.zeros(len(cands), bool)
    tour = [(start[0], matrix_to_quat(start[1]), -1)]
    while True:
        w = np.where(covered[pairs], 0.0,
                     DIR_WEIGHTS[np.minimum(nbins_pt[pair_pts], len(DIR_WEIGHTS) - 1)])
        gain = np.add.reduceat(w, offsets)
        gain[used] = 0
        best_cost = shots_between(tour[-1][0], tour[-1][1], cand_pos, cand_q)
        best_at = np.full(len(cands), len(tour))
        for k in range(len(tour) - 1):
            (pa, qa, _), (pb, qb, _) = tour[k], tour[k + 1]
            base = shots_between(pa, qa, pb[None], qb[None])[0]
            cost = (shots_between(pa, qa, cand_pos, cand_q)
                    + shots_between(pb, qb, cand_pos, cand_q) - base)
            better = cost < best_cost
            best_cost[better], best_at[better] = cost[better], k + 1
        c = int(np.argmax(gain / np.maximum(best_cost, 1.0)))
        if gain[c] <= 0:
            break
        new_tour = tour[:best_at[c]] + [(cand_pos[c], cand_q[c], c)] + tour[best_at[c]:]
        if tour_shots(new_tour) > max_shots:
            break
        tour = new_tour
        used[c] = True
        seg = pairs[offsets[c]:offsets[c] + lengths[c]]
        new = seg[~covered[seg]]
        covered[new] = True
        np.add.at(nbins_pt, new // n_bins, 1)
    return tour


def two_opt(tour):
    """先頭(スタート地点)を固定したまま、撮影回数が減るように順番を入れ替える。"""
    improved = True
    while improved:
        improved = False
        for i in range(1, len(tour) - 1):
            for j in range(i + 1, len(tour)):
                cand = tour[:i] + tour[i:j + 1][::-1] + tour[j + 1:]
                if tour_shots(cand) < tour_shots(tour) - 0.5:
                    tour, improved = cand, True
    return tour


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--max-images', type=int, default=400,
                        help='写真の枚数の上限(撮影回数 x カメラ台数)')
    parser.add_argument('--min-clearance', type=float, default=0.3,
                        help='候補位置の、壁からロボット中心までの最小距離[m]')
    parser.add_argument('--samples', type=int, default=6000, help='評価に使う表面の点の数')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--output', default=DEFAULT_OUTPUT, help='経路を書き出すyaml')
    parser.add_argument('--dry-run', action='store_true', help='書き出さずに評価だけ表示する')
    args = parser.parse_args()

    t_start = time.time()
    mesh, ray = cov.load_jpm_mesh()
    mounts = cov.load_camera_mounts()
    n_cameras = len(mounts)
    max_shots = args.max_images // n_cameras
    pts, nrm = cov.interior_surface(mesh, ray, n=args.samples, seed=args.seed)
    n_bins = len(cov.direction_bins())
    print(f'cameras: {[m[0] for m in mounts]}, surface samples: {len(pts)}, '
          f'max images: {args.max_images} ({max_shots} shots)')

    positions = np.array([(x, y, z) for x in CANDIDATE_X for z in CANDIDATE_Z
                          for y in CANDIDATE_Y])
    positions = positions[cov.clearance(mesh, positions) >= args.min_clearance]
    rots = candidate_orientations()
    cands, pairs, offsets, lengths = compute_visibility(
        ray, pts, nrm, mounts, positions, rots, n_bins)
    print(f'candidates: {len(positions)} positions x {len(rots)} orientations '
          f'({time.time() - t_start:.0f}s)')

    start = load_spawn_pose()
    tour = two_opt(greedy_tour(cands, pairs, offsets, lengths, len(pts), n_bins,
                               start, max_shots))
    keys = [(p, cands[c][1] if c >= 0 else start[1]) for p, _, c in tour]
    # 実際の撮影判定で数え直し、上限を超えていたら末尾の地点から削る
    while len(cov.simulate_shots(keys)[0]) > max_shots:
        tour, keys = tour[:-1], keys[:-1]

    path = np.vstack([np.linspace(a, b, max(2, int(np.linalg.norm(b - a) / 0.05) + 1))
                      for (a, _), (b, _) in zip(keys[1:-1], keys[2:])])
    margin = cov.clearance(mesh, path).min() - ROBOT_RADIUS_M
    print(f'stops: {len(tour) - 1}, wall margin (robot radius {ROBOT_RADIUS_M}m, '
          f'excluding spawn): {margin:.2f}m ({time.time() - t_start:.0f}s)')

    current = [start] + [(p, r) for _, p, r in load_waypoints()]
    for title, k in [('今の経路', current), ('最適化した経路', keys)]:
        rows, total, n_shots = cov.evaluate(ray, pts, nrm, mounts, k)
        cov.print_table(title, rows, total, n_shots, n_cameras)

    if args.dry_run:
        return
    header = (
        '# 3DGS学習データ撮影(gs_3d_image_picture)で順に巡る地点(iss_bodyからの相対オフセット)。\n'
        '# 上から順に移動する。rotation は body(x=前: camera_main, y=右: ステレオ, z=下)の向き。\n'
        f'# gs_optimize_path で作成({datetime.now():%Y-%m-%d}、カメラ {list(CAMERA_TOPICS)}、'
        f'写真{args.max_images}枚以内、壁からロボット中心まで{args.min_clearance}m以上)。\n'
        '# 作り直すときは: ros2 run intball2_programs gs_optimize_path\n'
    )
    save_waypoints(args.output,
                   [(f'3DGS_waypoint_{i:02d}', p, r) for i, (p, r) in enumerate(keys[1:], 1)],
                   header)
    print(f'\nwrote {len(keys) - 1} waypoints to {args.output}')


if __name__ == '__main__':
    main()
