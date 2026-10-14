#!/usr/bin/env python3
"""Analyze saved 200px source rays; no raycast, replay, ROS, or production masking."""
import argparse
import hashlib
import json
from pathlib import Path
import shlex

import numpy as np

BINS = ((0, 5), (5, 10), (10, 20), (20, 40), (40, None))


def valid(depth):
    return np.isfinite(depth) & (depth > 0)


def error_metrics(depth, reference, mask):
    selected = mask & valid(depth) & valid(reference)
    error = depth[selected].astype(np.float64) - reference[selected]
    return {'count': int(error.size),
            'signed_mean_m': float(error.mean()) if error.size else None,
            'signed_median_m': float(np.median(error)) if error.size else None,
            'absolute_p95_m': float(np.percentile(abs(error), 95)) if error.size else None,
            'rmse_m': float(np.sqrt(np.mean(error * error))) if error.size else None}


def edge_masks(shape):
    height, width = shape
    y, x = np.indices(shape)
    distances = {'left': x, 'right': width - 1 - x,
                 'top': y, 'bottom': height - 1 - y}
    return {f'{side}:{lo}-{hi if hi is not None else "plus"}':
            (distance >= lo) & (distance < hi if hi is not None else True)
            for side, distance in distances.items() for lo, hi in BINS}


def crop_mask(shape, margin):
    if margin < 0:
        raise ValueError('negative crop margin')
    height, width = shape
    y, x = np.indices(shape)
    return (x >= margin) & (x < width - margin) & (y >= margin) & (y < height - margin)


def analyze(reference, depths):
    if reference.ndim not in (2, 3) or any(a.shape != reference.shape for a in depths.values()):
        raise ValueError('all depths must share a 2D grid or stacked 2D grids')
    if set(depths) != {'captured', 'newgray', 'refined'}:
        raise ValueError('captured/newgray/refined required')
    common = valid(reference)
    for depth in depths.values():
        common = common & valid(depth)
    changed = valid(depths['newgray']) & valid(depths['refined']) & (
        depths['newgray'] != depths['refined'])
    retained = valid(depths['newgray']) & valid(depths['refined']) & ~changed

    def region(mask):
        mask = np.broadcast_to(mask, reference.shape)
        return {'grid_pixel_count': int(mask.sum()),
                'reference_count': int((mask & valid(reference)).sum()),
                'common_valid_count': int((mask & common).sum()),
                'variants': {name: {
                    'valid_count': int((mask & valid(depth) & valid(reference)).sum()),
                    'all_valid_count': int((mask & valid(depth)).sum()),
                    'reference_only_count': int((mask & valid(reference) & ~valid(depth)).sum()),
                    'valid_on_invalid_reference_count': int((mask & valid(depth) & ~valid(reference)).sum()),
                    'own_valid_error': error_metrics(depth, reference, mask),
                    'common_valid_error': error_metrics(depth, reference, mask & common)}
                    for name, depth in depths.items()},
                'updated_common': {name: error_metrics(depths[name], reference, mask & common & changed)
                                   for name in ('newgray', 'refined')},
                'retained_coarse_common': error_metrics(depths['newgray'], reference, mask & common & retained)}

    full = np.ones(reference.shape[-2:], dtype=bool)
    crops = {}
    for margin in (0, 5, 10, 20):
        mask = crop_mask(reference.shape[-2:], margin)
        kept = region(mask)
        removed = region(~mask)
        crops[str(margin)] = {'retained': kept, 'lost': removed,
                             'retained_grid_fraction': float(mask.mean())}
    return {'full': region(full), 'edge_bins': {name: region(mask)
            for name, mask in edge_masks(reference.shape[-2:]).items()}, 'symmetric_crops': crops,
            'refined_finite_mask_matches_newgray': bool(np.array_equal(valid(depths['newgray']),
                                                                       valid(depths['refined'])))}


def load_depth(directory, shape, stamp, frame, expected_k):
    lines = (directory / 'metadata.txt').read_text().splitlines()
    width, height, actual_stamp, actual_frame = shlex.split(lines[0])
    k = np.fromstring(lines[1], sep=' ')
    if ((int(height), int(width)) != shape or int(actual_stamp) != stamp or actual_frame != frame
            or not np.array_equal(k, expected_k)):
        raise ValueError(f'source rays/header mismatch: {directory}')
    return np.fromfile(directory / 'depth.bin', dtype=np.float32).reshape(shape)


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mesh-dir', type=Path, required=True)
    parser.add_argument('--coarse-dir', type=Path, required=True)
    parser.add_argument('--refined-dir', type=Path, required=True)
    parser.add_argument('--variant', default='baseline_d4_b5_u10')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError('output already exists')
    checkout = Path(__file__).resolve().parents[2]
    if args.output.resolve().is_relative_to(checkout):
        raise ValueError('output must be outside checkout')
    mesh_summary = args.mesh_dir / 'mesh_reference_summary.json'
    mesh_rows = json.loads(mesh_summary.read_text())['results']
    references = []
    depths = {key: [] for key in ('captured', 'newgray', 'refined')}
    frames = []
    hashes = {str(mesh_summary): sha256(mesh_summary), str(Path(__file__).resolve()): sha256(Path(__file__))}
    for row in mesh_rows:
        name = Path(row['archive']).stem
        npz_path = args.mesh_dir / row['archive']
        with np.load(npz_path, allow_pickle=False) as archive:
            reference = archive['mesh_left_depth_m'].copy()
            captured = archive['stereo_depth_m'].copy()
        if reference.shape != (200, 200):
            raise ValueError('analysis expects documented 200x200 source rays')
        info = row['capture_metadata']['camera_info']['stereo']
        frame = info['frame_id'] if 'frame_id' in info else info['header']['frame_id']
        k = np.asarray(info['k'])
        stamp = row['source_stamp_ns']
        dirs = {'newgray': args.coarse_dir / name / args.variant,
                'refined': args.refined_dir / name / args.variant}
        local = {'captured': captured, **{key: load_depth(directory, reference.shape, stamp, frame, k)
                                         for key, directory in dirs.items()}}
        references.append(reference)
        for key, depth in local.items():
            depths[key].append(depth)
        frames.append({'sample': name, 'source_stamp_ns': stamp, 'analysis': analyze(reference, local)})
        for path in [npz_path, *[directory / file for directory in dirs.values()
                                for file in ('depth.bin', 'metadata.txt')]]:
            hashes[str(path)] = sha256(path)
    if not frames:
        raise ValueError('no frames')
    report = {'schema_version': 1, 'arguments': {key: str(value) for key, value in vars(args).items()},
              'clock': 'No timing collected; source stamps are simulation-time metadata only',
              'definition': 'Pooled pixel errors on fixed original 200px source rays. Three-way common mask per region; own-valid errors also retained.',
              'edge_definition': 'Distances from each image side separately; corner pixels overlap between side reports. No overlap between bins on the same side.',
              'crop_definition': 'Proposed NaN exclusion only, no resampling/raycast, no production change. Cropped RMSE changes the evaluated population; lost region retained explicitly.',
              'limits': 'Five saved frames of one static scene; static visual mesh hypothesis, not full Gazebo ground truth. Outermost bands may contain no valid matcher depth.',
              'input_sha256': hashes, 'frames': frames,
              'pooled': analyze(np.stack(references), {key: np.stack(value) for key, value in depths.items()})}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print(args.output)


if __name__ == '__main__':
    main()
