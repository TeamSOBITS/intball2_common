#!/usr/bin/env python3
"""Offline core replay of captured exact bundles. No ROS, graph, or elapsed-time output."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import numpy as np
import yaml

CONFIG_KEYS = [
    'camera.calibration_mode', 'camera.input_geometry', 'processing.downsample',
    'processing.opencv_threads', 'matcher.min_disparity_px', 'matcher.num_disparities_px',
    'matcher.block_size_px', 'matcher.uniqueness_ratio', 'matcher.speckle_window_size_px',
    'matcher.speckle_range_px', 'matcher.disp12_max_diff_px', 'matcher.disparity_units',
    'depth.min_depth_m', 'depth.max_depth_m', 'points.publish_points', 'points.layout',
    'points.stride',
]


def parameters(path):
    document = yaml.safe_load(path.read_text())
    blocks = [entry['ros__parameters'] for entry in document.values()
              if isinstance(entry, dict) and 'ros__parameters' in entry]
    if len(blocks) != 1:
        raise ValueError('configuration needs one ros__parameters block')
    block = blocks[0]
    result = {}
    def flatten(value, prefix=''):
        for name, item in value.items():
            key = prefix + name
            if isinstance(item, dict):
                flatten(item, key + '.')
            else:
                result[key] = item
    flatten(block)
    if any(key not in result for key in CONFIG_KEYS):
        raise ValueError('configuration missing core values')
    return result


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def token(value):
    return str(int(value)) if isinstance(value, bool) else str(value)


def frame_manifest(directory, archive, metadata, image_name, info_name, stamp_key):
    image, info = metadata['images'][image_name], metadata['camera_info'][info_name]
    if image['stamp_ns'] != metadata['source_stamps'][stamp_key]:
        raise ValueError('raw image does not belong to accepted bundle')
    info_stamp_key = 'left_info_stamp_ns' if image_name == 'camera' else 'right_info_stamp_ns'
    if info['stamp_ns'] != metadata['source_stamps'][info_stamp_key]:
        raise ValueError('raw CameraInfo does not belong to accepted bundle')
    if image['frame_id'] != info['frame_id'] or (image['width'], image['height']) != (info['width'], info['height']):
        raise ValueError('raw image/CameraInfo frame or shape mismatch')
    optical = {'cameraL_link': 'cameraL_optical_frame', 'cameraR_link': 'cameraR_optical_frame'}.get(
        image['frame_id'], image['frame_id'])
    raw_name = image_name + '_image_bytes'
    raw = archive[raw_name].tobytes()
    if len(raw) != image['data_length'] or len(raw) < image['step'] * image['height']:
        raise ValueError('raw image byte length mismatch')
    binary = directory / (image_name + '.bin')
    binary.write_bytes(raw)
    lines = [f'{json.dumps(str(binary))} {image["width"]} {image["height"]} {image["step"]} '
             f'{image["encoding"]} {image["stamp_ns"]} {json.dumps(optical)}',
             f'{info["width"]} {info["height"]} {json.dumps(info["distortion_model"])}']
    for key in ['k', 'r', 'p']:
        lines.append(' '.join(map(repr, info[key])))
    lines.append(' '.join(map(str, [len(info['d']), *info['d']])))
    roi = info['roi']
    lines.append(' '.join(map(token, [info['binning_x'], info['binning_y'], roi['x_offset'],
                                      roi['y_offset'], roi['width'], roi['height'], roi['do_rectify']])))
    return '\n'.join(lines)


def output_directory(path):
    directory = path.resolve()
    checkout = Path(__file__).resolve().parents[2]
    if directory.is_relative_to(checkout):
        raise ValueError('replay outputs must be outside the checkout')
    return directory


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--capture', type=Path, required=True)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--live-config', type=Path)
    parser.add_argument('--runner', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--core-library', type=Path, required=True)
    args = parser.parse_args(argv)
    directory = output_directory(args.output_dir)
    directory.mkdir(parents=True, exist_ok=False)
    runner = args.runner.resolve(strict=True)
    library = args.core_library.resolve(strict=True)
    environment = os.environ.copy()
    environment['LD_LIBRARY_PATH'] = str(library.parent) + (':' + environment['LD_LIBRARY_PATH'] if environment.get('LD_LIBRARY_PATH') else '')
    config = parameters(args.config)
    if args.live_config:
        live = parameters(args.live_config)
        if any(config[key] != live[key] for key in CONFIG_KEYS):
            raise ValueError('startup and live core config differ')
    results, skipped = [], []
    for path in sorted((args.capture / 'input_samples').glob('*.npz')):
        with np.load(path, allow_pickle=False) as archive:
            metadata = json.loads(archive['metadata_json_utf8'].tobytes())
            required = ['camera_image_bytes', 'right_image_bytes']
            if any(key not in archive for key in required) or any(
                key not in metadata['camera_info'] for key in ['camera_info', 'right_info']
            ):
                skipped.append({'archive': path.name, 'reason': 'incomplete raw bundle',
                                'missing': metadata['missing']})
                continue
            work = directory / path.stem
            work.mkdir(exist_ok=False)
            manifest = '\n'.join([
                ' '.join(token(config[key]) for key in CONFIG_KEYS),
                frame_manifest(work, archive, metadata, 'camera', 'camera_info', 'left_image_stamp_ns'),
                frame_manifest(work, archive, metadata, 'right', 'right_info', 'right_image_stamp_ns'),
            ]) + '\n'
            (work / 'manifest.txt').write_text(manifest)
            subprocess.run([str(runner), str(work / 'manifest.txt'),
                            str(work / 'depth.bin'), str(work / 'metadata.txt')], check=True, env=environment)
            loaded_library = Path(shlex.split((work / 'metadata.txt').read_text().splitlines()[3])[0]).resolve(strict=True)
            if not loaded_library.samefile(library):
                raise ValueError('runner loaded a different core library than --core-library')
            lines = (work / 'metadata.txt').read_text().splitlines()
            width, height, stamp, frame = shlex.split(lines[0])
            width, height, stamp = int(width), int(height), int(stamp)
            replay = np.fromfile(work / 'depth.bin', dtype=np.float32).reshape(height, width)
            captured = metadata['images']['stereo_depth']
            dtype = np.dtype('>f4' if captured['is_bigendian'] else '<f4')
            original = np.ndarray((captured['height'], captured['width']), dtype=dtype,
                                  buffer=archive['stereo_depth_bytes'].tobytes(),
                                  strides=(captured['step'], 4)).astype(np.float32)
            shape_match = replay.shape == original.shape
            nan_mask_match = bool(shape_match and np.array_equal(np.isnan(replay), np.isnan(original)))
            finite_mask_match = bool(shape_match and np.array_equal(np.isfinite(replay), np.isfinite(original)))
            both_finite = np.isfinite(replay) & np.isfinite(original) if shape_match else np.zeros(replay.shape, dtype=bool)
            max_difference = float(np.max(np.abs(replay[both_finite].astype(float) - original[both_finite]))) if both_finite.any() else None
            k = np.fromstring(lines[1], sep=' ')
            p = np.fromstring(lines[2], sep=' ')
            result = {
                'archive': path.name, 'captured_usable_replay': metadata['usable_replay'],
                'shape': list(replay.shape), 'shape_match': shape_match,
                'source_stamp_ns': stamp,
                'stamp_match': stamp == captured['stamp_ns'] == metadata['source_stamp_ns'],
                'frame_id': frame, 'frame_match': frame == captured['frame_id'],
                'k_match': bool(np.array_equal(k, metadata['camera_info']['stereo']['k'])),
                'p_match': bool(np.array_equal(p, metadata['camera_info']['stereo']['p'])),
                'nan_mask_match': nan_mask_match, 'finite_mask_match': finite_mask_match,
                'finite_pixels': int(np.count_nonzero(both_finite)),
                'max_abs_difference_m': max_difference,
                'depth_equal': bool(shape_match and np.array_equal(replay, original, equal_nan=True)),
            }
            results.append(result)
    passed = bool(results) and all(all(row[key] for key in [
        'shape_match', 'stamp_match', 'frame_match', 'k_match', 'p_match',
        'nan_mask_match', 'finite_mask_match', 'depth_equal'
    ]) for row in results)
    summary = {'success': passed, 'reprocessed_count': len(results), 'skipped_count': len(skipped),
               'config_path': str(args.config), 'config_sha256': sha256(args.config),
               'core_config': {key: config[key] for key in CONFIG_KEYS},
               'live_config_checked': args.live_config is not None,
               'live_config_path': str(args.live_config) if args.live_config else None,
               'runner_path': str(runner), 'runner_sha256': sha256(runner),
               'core_library': str(library),
               'core_library_sha256': sha256(library),
               'results': results, 'skipped': skipped}
    (directory / 'replay_summary.json').write_text(json.dumps(summary, allow_nan=False, indent=2) + '\n')
    print(json.dumps(summary, allow_nan=False))
    return 0 if passed else 1


if __name__ == '__main__':
    sys.exit(main())
