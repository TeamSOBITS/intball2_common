#!/usr/bin/env python3
"""Compare core settings on identical captured input; no ROS or elapsed timings."""
import argparse
import copy
import json
import os
from pathlib import Path
import shlex
import subprocess
from types import SimpleNamespace

import numpy as np

from depth_comparison import decode_depth, depth_metrics, resample_reference
from replay_capture import CONFIG_KEYS, frame_manifest, output_directory, parameters, sha256, token


def conditions(base):
    changes = {
        'baseline_d4_b5_u10': {},
        'd4_b3': {'matcher.block_size_px': 3},
        'd4_b7': {'matcher.block_size_px': 7},
        'd4_b5_u15': {'matcher.uniqueness_ratio': 15},
        'd2_b5': {'processing.downsample': 2, 'matcher.min_disparity_px': -50,
                  'matcher.num_disparities_px': 64},
        'd2_b9': {'processing.downsample': 2, 'matcher.min_disparity_px': -50,
                  'matcher.num_disparities_px': 64, 'matcher.block_size_px': 9},
    }
    expected = {'processing.downsample': 4, 'matcher.min_disparity_px': -25,
                'matcher.num_disparities_px': 32, 'matcher.block_size_px': 5,
                'matcher.uniqueness_ratio': 10}
    if any(base[key] != value for key, value in expected.items()):
        raise ValueError('this experiment requires the documented d4/b5/u10 baseline')
    return {name: {**{key: base[key] for key in CONFIG_KEYS}, **change}
            for name, change in changes.items()}


def decoded(archive, metadata, name):
    wire = metadata['images'][name]
    return decode_depth(SimpleNamespace(**wire, data=archive[name + '_bytes'].tobytes()))


def run_core(work, frames, config, runner, library, environment, refinement=False):
    work.mkdir()
    manifest = '\n'.join([' '.join(token(config[key]) for key in CONFIG_KEYS), *frames]) + '\n'
    (work / 'manifest.txt').write_text(manifest)
    command = [str(runner), str(work / 'manifest.txt'), str(work / 'depth.bin'),
               str(work / 'metadata.txt')]
    if refinement:
        command.append('--refine')
    subprocess.run(command, check=True, env=environment)
    lines = (work / 'metadata.txt').read_text().splitlines()
    width, height, stamp, frame = shlex.split(lines[0])
    loaded = Path(shlex.split(lines[3])[0]).resolve(strict=True)
    if not loaded.samefile(library):
        raise ValueError('unexpected loaded core library')
    depth = np.fromfile(work / 'depth.bin', dtype=np.float32).reshape(int(height), int(width))
    return depth, np.fromstring(lines[1], sep=' '), int(stamp), frame


def pooled(arrays, references):
    return depth_metrics(np.concatenate([a.reshape(-1) for a in arrays])[None, :],
                         np.concatenate([r.reshape(-1) for r in references])[None, :])


def common_metrics(first, second, reference):
    mask = np.isfinite(first) & (first > 0) & np.isfinite(second) & (second > 0)
    shared = np.where(mask, reference, np.nan)
    return {'baseline': depth_metrics(first, shared), 'candidate': depth_metrics(second, shared)}


def synthetic_pair(metadata, depth_m, seed=731):
    """Band-limited deterministic Lambertian frontoparallel plane, zero skew.

    Negative disparity d=x_left-x_right; right(x)=left(x+d). Quantized image
    interpolation models a synthetic raster pair, not Gazebo rendering noise.
    """
    info = metadata['camera_info']['camera_info']
    height, width = info['height'], info['width']
    rng = np.random.default_rng(seed)
    noise = rng.standard_normal((height, width))
    fy = np.fft.fftfreq(height)[:, None]
    fx = np.fft.rfftfreq(width)[None, :]
    spectrum = np.fft.rfft2(noise)
    texture = sum(np.fft.irfft2(spectrum * np.exp(-2 * np.pi**2 * sigma**2 * (fx**2 + fy**2)),
                               s=noise.shape) for sigma in [2.0, 6.0])
    texture = 127 + 32 * texture / texture.std()
    left = np.clip(texture, 0, 255)
    disparity = -float(info['k'][0]) * 0.05 / depth_m
    x = np.arange(width) + disparity
    low = np.floor(x).astype(int)
    alpha = x - low
    right = left[:, np.clip(low, 0, width - 1)] * (1 - alpha) + left[:, np.clip(low + 1, 0, width - 1)] * alpha
    arrays = {name + '_image_bytes': np.repeat(np.rint(array).astype(np.uint8)[..., None], 3, axis=2).reshape(-1)
              for name, array in [('camera', left), ('right', right)]}
    changed = copy.deepcopy(metadata)
    for name in ['camera', 'right']:
        changed['images'][name].update(encoding='rgb8', step=width * 3,
                                     is_bigendian=False, data_length=height * width * 3)
    return arrays, changed, disparity


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ['capture', 'config', 'live-config', 'runner', 'core-library', 'output-dir']:
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--synthetic', action='store_true')
    parser.add_argument('--refine', action='store_true', help='enable typed-default native core refinement')
    parser.add_argument('--allow-baseline-change', action='store_true',
                        help='permit intentionally changed core; record capture equality')
    args = parser.parse_args()
    config = parameters(args.config)
    live = parameters(args.live_config)
    if any(config[key] != live[key] for key in CONFIG_KEYS):
        raise ValueError('startup and live core configs differ')
    variants = conditions(config)
    runner, library = args.runner.resolve(strict=True), args.core_library.resolve(strict=True)
    environment = os.environ.copy()
    environment['LD_LIBRARY_PATH'] = str(library.parent) + ':' + environment.get('LD_LIBRARY_PATH', '')
    destination = output_directory(args.output_dir)
    destination.mkdir(parents=True, exist_ok=False)
    outputs = {name: [] for name in variants}
    references, captured_depths, records, skipped = [], [], [], []
    first_metadata = None
    for path in sorted((args.capture / 'input_samples').glob('*.npz')):
        with np.load(path, allow_pickle=False) as archive:
            metadata = json.loads(archive['metadata_json_utf8'].tobytes())
            if any(key not in archive for key in ['camera_image_bytes', 'right_image_bytes']) or any(
                key not in metadata['camera_info'] for key in ['camera_info', 'right_info']
            ):
                skipped.append({'archive': path.name, 'missing': metadata['missing']})
                continue
            first_metadata = first_metadata or copy.deepcopy(metadata)
            sample_dir = destination / path.stem
            sample_dir.mkdir()
            frames = [frame_manifest(sample_dir, archive, metadata, 'camera', 'camera_info', 'left_image_stamp_ns'),
                      frame_manifest(sample_dir, archive, metadata, 'right', 'right_info', 'right_image_stamp_ns')]
            captured = decoded(archive, metadata, 'stereo_depth')
            native_ref = decoded(archive, metadata, 'reference_depth')
            baseline_k = metadata['camera_info']['stereo']['k']
            reference = resample_reference(native_ref, metadata['camera_info']['reference']['k'],
                                           baseline_k, captured.shape)
            references.append(reference)
            captured_depths.append(captured)
            sample = {'archive': path.name, 'source_stamp_ns': metadata['source_stamp_ns'],
                      'captured_usable_replay': metadata['usable_replay'], 'variants': {}}
            for name, settings in variants.items():
                depth, k, stamp, frame = run_core(sample_dir / name, frames, settings, runner, library, environment, args.refine)
                if stamp != metadata['source_stamp_ns'] or frame != metadata['images']['stereo_depth']['frame_id']:
                    raise ValueError('runner output header changed')
                factor = settings['processing.downsample']
                source_k = np.array(metadata['camera_info']['camera_info']['k']).reshape(3, 3)
                expected_k = source_k.copy()
                expected_k[0, 0] /= factor
                expected_k[1, 1] /= factor
                expected_k[0, 2] = (source_k[0, 2] + .5) / factor - .5
                expected_k[1, 2] = (source_k[1, 2] + .5) / factor - .5
                if not np.array_equal(k.reshape(3, 3), expected_k):
                    raise ValueError('output K is inconsistent with pixel-center resizing')
                if name == 'baseline_d4_b5_u10':
                    sample['baseline_capture_equal'] = bool(np.array_equal(depth, captured, equal_nan=True))
                    if not sample['baseline_capture_equal'] and not args.allow_baseline_change:
                        raise ValueError('baseline replay differs from captured depth')
                aligned = resample_reference(depth, k, baseline_k, captured.shape)
                outputs[name].append(aligned)
                sample['variants'][name] = {'native_shape': list(depth.shape), 'native_k': k.tolist(),
                                            'metrics': depth_metrics(aligned, reference),
                                            'common_valid_vs_captured': common_metrics(captured, aligned, reference)}
                native_lines = (sample_dir / name / 'metadata.txt').read_text().splitlines()
                if len(native_lines) > 4:
                    values = native_lines[4].split()
                    sample['variants'][name]['native_refinement'] = dict(zip([
                        'enabled', 'downsample', 'window_size_px', 'search_radius_px',
                        'min_texture_variance', 'min_correlation', 'attempted', 'updated', 'retained'],
                        [float(value) for value in values]))
            for name in variants:
                sample['variants'][name]['common_valid'] = common_metrics(outputs['baseline_d4_b5_u10'][-1], outputs[name][-1], reference)
            sample['captured_coarse_metrics'] = depth_metrics(captured, reference)
            records.append(sample)
    if not references:
        raise ValueError('no complete captured input pairs')
    summary = {
        'reference_contract': 'virtual occupied-voxel optical Z; not Gazebo triangle mesh ground truth',
        'comparison_grid': 'captured stereo 200x200 pixel rays; nearest resampling, no hole filling',
        'limitations': 'five saved frames of one scene; repetition is not independent scene evidence; 200px reference cannot validate full 400px output detail',
        'config_path': str(args.config.resolve()), 'config_sha256': sha256(args.config),
        'live_config_path': str(args.live_config.resolve()), 'live_config_sha256': sha256(args.live_config),
        'runner_path': str(runner), 'runner_sha256': sha256(runner),
        'core_library': str(library), 'core_library_sha256': sha256(library),
        'conditions': variants, 'reprocessed_count': len(records), 'skipped': skipped,
        'baseline_change_permitted': args.allow_baseline_change,
        'native_refinement_enabled': args.refine,
        'per_frame': records, 'pooled': {}, 'synthetic': [],
        'captured_coarse_pooled': pooled(captured_depths, references),
    }
    for name in variants:
        base = np.concatenate([a.reshape(-1) for a in outputs['baseline_d4_b5_u10']])[None, :]
        candidate = np.concatenate([a.reshape(-1) for a in outputs[name]])[None, :]
        ref = np.concatenate([a.reshape(-1) for a in references])[None, :]
        captured_all = np.concatenate([a.reshape(-1) for a in captured_depths])[None, :]
        summary['pooled'][name] = {'metrics': pooled(outputs[name], references),
                                   'common_valid': common_metrics(base, candidate, ref),
                                   'common_valid_vs_captured': common_metrics(captured_all, candidate, ref)}
    if args.synthetic:
        for depth_m in [.5, 1., 2., 2.8]:
            work = destination / f'plane_{depth_m:g}m'
            work.mkdir()
            arrays, metadata, disparity = synthetic_pair(first_metadata, depth_m)
            frames = [frame_manifest(work, arrays, metadata, 'camera', 'camera_info', 'left_image_stamp_ns'),
                      frame_manifest(work, arrays, metadata, 'right', 'right_info', 'right_image_stamp_ns')]
            plane = {'true_optical_depth_m': depth_m, 'fullres_disparity_px': disparity, 'variants': {}}
            baseline_k = metadata['camera_info']['stereo']['k']
            ref = np.full((200, 200), depth_m, dtype=np.float32)
            # Interior excludes generated-image shift boundaries and all matcher ROI edges.
            ref[:20] = np.nan
            ref[-20:] = np.nan
            ref[:, :20] = np.nan
            ref[:, -20:] = np.nan
            for name, settings in variants.items():
                depth, k, _, _ = run_core(work / name, frames, settings, runner, library, environment, args.refine)
                aligned = resample_reference(depth, k, baseline_k, ref.shape)
                plane['variants'][name] = depth_metrics(aligned, ref)
            summary['synthetic'].append(plane)
    (destination / 'precision_summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n')
    print(json.dumps({'summary': str(destination / 'precision_summary.json'),
                      'reprocessed_count': len(records), 'skipped_count': len(skipped)}, allow_nan=False))


if __name__ == '__main__':
    main()
