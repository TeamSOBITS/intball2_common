#!/usr/bin/env python3
"""Experimental local ZNCC refinement; offline, no timing output."""
import argparse
import json
from pathlib import Path
import shlex

import numpy as np

from compare_replay_precision import common_metrics, decoded, pooled, synthetic_pair
from depth_comparison import depth_metrics, resample_reference
from replay_capture import output_directory, sha256


def grayscale(raw, wire):
    channels = 1 if wire['encoding'] == 'mono8' else 3
    if wire['encoding'] not in ['mono8', 'rgb8', 'bgr8']:
        raise ValueError('unsupported image encoding')
    view = np.ndarray((wire['height'], wire['width'], channels), dtype=np.uint8,
                      buffer=raw, strides=(wire['step'], channels, 1)).astype(np.float64)
    if channels == 1:
        return view[..., 0]
    weights = [.299, .587, .114] if wire['encoding'] == 'rgb8' else [.114, .587, .299]
    return view @ np.asarray(weights)


def refinement_inputs(left, right, full_k, factor):
    if factor not in [1, 2] or left.shape[0] % factor or left.shape[1] % factor:
        raise ValueError('refinement downsample must be 1/2 and divide input dimensions')
    k = np.asarray(full_k, dtype=np.float64).reshape(3, 3).copy()
    if factor == 2:
        height, width = left.shape
        left = left.reshape(height // 2, 2, width // 2, 2).mean(axis=(1, 3))
        right = right.reshape(height // 2, 2, width // 2, 2).mean(axis=(1, 3))
        k[0, 0] /= factor
        k[1, 1] /= factor
        k[0, 2] = (k[0, 2] + .5) / factor - .5
        k[1, 2] = (k[1, 2] + .5) / factor - .5
    return left, right, k.reshape(-1)


def box_sum(array, radius):
    padded = np.pad(array, radius)
    integral = np.pad(padded, ((1, 0), (1, 0))).cumsum(0).cumsum(1)
    size = 2 * radius + 1
    return integral[size:, size:] - integral[:-size, size:] - integral[size:, :-size] + integral[:-size, :-size]


def sample_bilinear(array, xs, ys):
    x0, y0 = np.floor(xs).astype(int), np.floor(ys).astype(int)
    x1, y1 = np.clip(x0 + 1, 0, array.shape[1] - 1), np.clip(y0 + 1, 0, array.shape[0] - 1)
    wx, wy = xs - x0, ys - y0
    x0, y0 = np.clip(x0, 0, array.shape[1] - 1), np.clip(y0, 0, array.shape[0] - 1)
    return ((array[np.ix_(y0, x0)] * (1 - wx) + array[np.ix_(y0, x1)] * wx) * (1 - wy[:, None]) +
            (array[np.ix_(y1, x0)] * (1 - wx) + array[np.ix_(y1, x1)] * wx) * wy[:, None])


def refine(left, right, depth, full_k, output_k, window=15, search_radius=2):
    """Only refine existing valid depth; reject weak texture and boundary minima.

    Evaluate refinement-resolution box ZNCC at output pixel rays. The search is a
    narrow integer-disparity band about the coarse prediction, followed by a
    three-point parabola. This is a prototype, not production depth processing.
    """
    if left.shape != right.shape or window < 3 or window % 2 == 0 or search_radius < 1:
        raise ValueError('invalid image pair or refinement support')
    f = np.asarray(full_k).reshape(3, 3)
    o = np.asarray(output_k).reshape(3, 3)
    xs = (np.arange(depth.shape[1]) - o[0, 2]) * f[0, 0] / o[0, 0] + f[0, 2]
    ys = (np.arange(depth.shape[0]) - o[1, 2]) * f[1, 1] / o[1, 1] + f[1, 2]
    valid = np.isfinite(depth) & (depth > 0)
    prediction = np.zeros(depth.shape)
    prediction[valid] = -f[0, 0] * .05 / depth[valid]
    center = np.rint(prediction).astype(int)
    if not valid.any():
        return np.full(depth.shape, np.nan, dtype=np.float32), {'accepted_count': 0}
    ds = np.arange(center[valid].min() - search_radius - 1, center[valid].max() + search_radius + 2)
    radius, count = window // 2, window * window
    sum_l = box_sum(left, radius)
    var_l = np.maximum(0, box_sum(left * left, radius) - sum_l * sum_l / count)
    texture_l = sample_bilinear(var_l / count, xs, ys)
    costs = []
    for d in ds:
        shifted = np.zeros_like(right)
        if abs(d) < right.shape[1]:
            if d < 0:
                shifted[:, :d] = right[:, -d:]
            elif d > 0:
                shifted[:, d:] = right[:, :-d]
            else:
                shifted[:] = right
        sum_r = box_sum(shifted, radius)
        var_r = np.maximum(0, box_sum(shifted * shifted, radius) - sum_r * sum_r / count)
        covariance = box_sum(left * shifted, radius) - sum_l * sum_r / count
        denominator = np.sqrt(var_l * var_r)
        correlation = np.divide(covariance, denominator, out=np.zeros_like(left), where=denominator > 1e-9)
        cost = sample_bilinear(1 - correlation, xs, ys)
        texture_r = sample_bilinear(var_r / count, xs, ys)
        support = ((xs[None, :] - d - radius >= 0) & (xs[None, :] - d + radius < left.shape[1]) &
                   (xs[None, :] - radius >= 0) & (xs[None, :] + radius < left.shape[1]) &
                   (ys[:, None] - radius >= 0) & (ys[:, None] + radius < left.shape[0]))
        cost[~support | (texture_l < 1) | (texture_r < 1)] = np.inf
        costs.append(cost)
    costs = np.stack(costs)
    allowed = abs(ds[:, None, None] - center) <= search_radius
    best = np.argmin(np.where(allowed, costs, np.inf), axis=0)
    yy, xx = np.indices(depth.shape)
    middle = costs[best, yy, xx]
    low = costs[np.clip(best - 1, 0, len(ds) - 1), yy, xx]
    high = costs[np.clip(best + 1, 0, len(ds) - 1), yy, xx]
    finite_curve = np.isfinite(low) & np.isfinite(middle) & np.isfinite(high)
    curvature = np.zeros(depth.shape)
    curvature[finite_curve] = low[finite_curve] - 2 * middle[finite_curve] + high[finite_curve]
    offset = np.zeros(depth.shape)
    curved = finite_curve & (curvature > 1e-6)
    offset[curved] = .5 * (low[curved] - high[curved]) / curvature[curved]
    accepted = valid & curved & (middle < .3) & (abs(ds[best] - center) < search_radius)
    subpixel = ds[best] + np.clip(offset, -.5, .5)
    z = np.full(depth.shape, np.nan, dtype=np.float32)
    safe = accepted & (subpixel < 0)
    z[safe] = -f[0, 0] * .05 / subpixel[safe]
    range_valid = np.isfinite(z) & (z >= .25) & (z <= 3)
    z[~range_valid] = np.nan
    return z, {'input_valid_count': int(valid.sum()), 'accepted_count': int(range_valid.sum()),
               'rejected_count': int(valid.sum() - range_valid.sum()),
               'rejections': {
                   'nonfinite_cost_curve_support_or_texture': int((valid & ~finite_curve).sum()),
                   'nonconvex_cost_curve': int((valid & finite_curve & ~curved).sum()),
                   'low_correlation': int((valid & curved & (middle >= .3)).sum()),
                   'search_boundary': int((valid & curved & (middle < .3) & (abs(ds[best] - center) >= search_radius)).sum()),
                   'nonnegative_disparity': int((accepted & ~safe).sum()),
                   'outside_depth_range': int((safe & ~range_valid).sum()),
               },
               'window_px_refinement': window, 'search_radius_px_refinement': search_radius,
               'minimum_texture_variance': 1, 'minimum_correlation': .7,
               'policy': 'reject search-boundary minima; never fill initial invalid pixels'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['capture', 'comparison-dir', 'output-dir']:
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--window', type=int, default=15)
    parser.add_argument('--refinement-downsample', type=int, choices=[1, 2], default=1)
    args = parser.parse_args()
    previous_path = args.comparison_dir / 'precision_summary.json'
    previous = json.loads(previous_path.read_text())
    destination = output_directory(args.output_dir)
    destination.mkdir(parents=True, exist_ok=False)
    records, bases, refineds, hybrids, references = [], [], [], [], []
    synthetic = []
    first_metadata = None
    for row in previous['per_frame']:
        path = args.capture / 'input_samples' / row['archive']
        with np.load(path, allow_pickle=False) as archive:
            metadata = json.loads(archive['metadata_json_utf8'].tobytes())
            first_metadata = first_metadata or metadata
            k = metadata['camera_info']['stereo']['k']
            full_k = metadata['camera_info']['camera_info']['k']
            base = decoded(archive, metadata, 'stereo_depth')
            left = grayscale(archive['camera_image_bytes'].tobytes(), metadata['images']['camera'])
            right = grayscale(archive['right_image_bytes'].tobytes(), metadata['images']['right'])
            left, right, refinement_k = refinement_inputs(left, right, full_k, args.refinement_downsample)
            new, flags = refine(left, right, base, refinement_k, k, args.window)
            reference = resample_reference(decoded(archive, metadata, 'reference_depth'),
                                           metadata['camera_info']['reference']['k'], k, base.shape)
            bases.append(base)
            refineds.append(new)
            hybrid = np.where(np.isfinite(new), new, base)
            hybrids.append(hybrid)
            references.append(reference)
            np.save(destination / (path.stem + '_refined.npy'), new)
            records.append({'archive': path.name, 'source_stamp_ns': metadata['source_stamp_ns'],
                            'flags': flags, 'metrics': depth_metrics(new, reference),
                            'common_valid': common_metrics(base, new, reference),
                            'confidence_gated_keep_coarse_metrics': depth_metrics(hybrid, reference),
                            'coarse_pixels_retained': int((np.isfinite(base) & ~np.isfinite(new)).sum())})
    for plane in previous['synthetic']:
        depth_m = plane['true_optical_depth_m']
        arrays, metadata, _ = synthetic_pair(first_metadata, depth_m)
        work = args.comparison_dir / f'plane_{depth_m:g}m' / 'baseline_d4_b5_u10'
        width, height, _, _ = shlex.split((work / 'metadata.txt').read_text().splitlines()[0])
        base = np.fromfile(work / 'depth.bin', np.float32).reshape(int(height), int(width))
        left = grayscale(arrays['camera_image_bytes'].tobytes(), metadata['images']['camera'])
        right = grayscale(arrays['right_image_bytes'].tobytes(), metadata['images']['right'])
        left, right, refinement_k = refinement_inputs(
            left, right, metadata['camera_info']['camera_info']['k'], args.refinement_downsample)
        new, flags = refine(left, right, base, refinement_k,
                            metadata['camera_info']['stereo']['k'], args.window)
        reference = np.full(base.shape, depth_m, dtype=np.float32)
        reference[:20] = reference[-20:] = np.nan
        reference[:, :20] = reference[:, -20:] = np.nan
        synthetic.append({'true_optical_depth_m': depth_m, 'flags': flags,
                          'metrics': depth_metrics(new, reference),
                          'common_valid': common_metrics(base, new, reference)})
    if not bases:
        raise ValueError('no complete pairs')
    base = np.concatenate([a.ravel() for a in bases])[None, :]
    new = np.concatenate([a.ravel() for a in refineds])[None, :]
    ref = np.concatenate([a.ravel() for a in references])[None, :]
    result = {'algorithm': 'experimental local ZNCC + three-point parabola',
              'capture': str(args.capture.resolve()), 'comparison_summary': str(previous_path.resolve()),
              'comparison_summary_sha256': sha256(previous_path),
              'limitations': 'manual NumPy prototype; no simulation latency evidence; virtual depth is voxel reference, not mesh ground truth',
              'window_refinement_px': args.window, 'search_radius_refinement_px': 2,
              'refinement_downsample': args.refinement_downsample,
              'refinement_image_contract': 'AREA averaged floating grayscale; production uint8 rounding not represented exactly',
              'confidence_gated_keep_coarse_policy': 'accepted refinement replaces existing depth; rejected refinement retains original coarse value explicitly; initial NaNs stay NaN',
              'per_frame': records, 'pooled': {'baseline': pooled(bases, references),
                                              'refined': pooled(refineds, references),
                                              'confidence_gated_keep_coarse': pooled(hybrids, references),
                                              'common_valid': common_metrics(base, new, ref)},
              'synthetic': synthetic}
    (destination / 'refinement_summary.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps({'summary': str(destination / 'refinement_summary.json'), 'count': len(records)}))


if __name__ == '__main__':
    main()
