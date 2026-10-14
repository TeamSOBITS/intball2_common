#!/usr/bin/env python3
"""Offline installed stereo_image_proc precision on identical archived bundles. No elapsed timings."""
import argparse
import json
import os
from pathlib import Path
import shlex
import subprocess

import numpy as np

from compare_replay_precision import decoded, pooled
from depth_comparison import depth_metrics, resample_reference
from replay_capture import CONFIG_KEYS, frame_manifest, output_directory, parameters, sha256, token


def core_output(directory, sample):
    choices = [directory / sample / 'depth.bin', directory / sample / 'baseline_d4_b5_u10' / 'depth.bin']
    path = next((candidate for candidate in choices if candidate.exists()), None)
    if path is None:
        raise ValueError(f'missing saved core output {directory}/{sample}')
    metadata_path = path.parent / 'metadata.txt'
    lines = metadata_path.read_text().splitlines()
    width, height, stamp, frame = shlex.split(lines[0])
    depth = np.fromfile(path, np.float32).reshape(int(height), int(width))
    return depth, np.fromstring(lines[1], sep=' '), int(stamp), frame, {
        'depth_path': str(path.resolve()), 'depth_sha256': sha256(path),
        'metadata_path': str(metadata_path.resolve()), 'metadata_sha256': sha256(metadata_path)}


def compare(args):
    destination = output_directory(args.output_dir)
    destination.mkdir(parents=True, exist_ok=False)
    runner, library = args.runner.resolve(strict=True), args.library.resolve(strict=True)
    config = parameters(args.config)
    if config['processing.downsample'] != 4 or any(config.get('processing.roi_' + edge + '_px', 0)
                                                 for edge in ('left', 'right', 'top', 'bottom')):
        raise ValueError('comparison requires the uncropped 200-grid baseline')
    environment = os.environ.copy()
    environment['LD_LIBRARY_PATH'] = str(library.parent) + ':' + environment.get('LD_LIBRARY_PATH', '')
    subprocess.run([str(runner), '--self-test'], check=True, env=environment)
    records, skipped, references = [], [], []
    names = ('ours_coarse', 'ours_refined', 'sip_common800', 'sip_ours200_mode_sgbm')
    populations = {name: [] for name in names}
    source_info = []
    for path in sorted((args.capture / 'input_samples').glob('*.npz')):
        with np.load(path, allow_pickle=False) as archive:
            metadata = json.loads(archive['metadata_json_utf8'].tobytes())
            mesh_path = args.mesh_dir / path.name
            if not mesh_path.exists() or not metadata.get('usable_replay'):
                skipped.append({'archive': path.name, 'reason': 'missing exact-pose mesh or incomplete raw bundle',
                                'capture_missing': metadata.get('missing')})
                continue
            sample = path.stem
            work = destination / sample
            work.mkdir()
            frames = [frame_manifest(work, archive, metadata, 'camera', 'camera_info', 'left_image_stamp_ns'),
                      frame_manifest(work, archive, metadata, 'right', 'right_info', 'right_image_stamp_ns')]
            manifest = work / 'manifest.txt'
            manifest.write_text('\n'.join([' '.join(token(config[key]) for key in CONFIG_KEYS), *frames]) + '\n')
            captured = decoded(archive, metadata, 'stereo_depth')
            baseline_k = np.asarray(metadata['camera_info']['stereo']['k'])
            virtual = resample_reference(decoded(archive, metadata, 'reference_depth'),
                                         metadata['camera_info']['reference']['k'], baseline_k, captured.shape)
            with np.load(mesh_path, allow_pickle=False) as mesh:
                reference = mesh['mesh_left_depth_m'].copy()
            if reference.shape != captured.shape:
                raise ValueError('mesh reference and comparison grid differ')
            references.append(reference)
            row = {'archive': path.name, 'archive_sha256': sha256(path), 'mesh_path': str(mesh_path.resolve()),
                   'mesh_sha256': sha256(mesh_path), 'source_stamp_ns': metadata['source_stamp_ns'], 'conditions': {}}
            candidates = {}
            for name, directory in [('ours_coarse', args.our_baseline_dir), ('ours_refined', args.our_refined_dir)]:
                depth, k, stamp, frame, provenance = core_output(directory, sample)
                if depth.shape != captured.shape or not np.array_equal(k, baseline_k) or stamp != metadata['source_stamp_ns'] or frame != metadata['images']['stereo_depth']['frame_id']:
                    raise ValueError('saved our output ray/header mismatch')
                candidates[name] = depth
                row['conditions'][name] = {'provenance': provenance}
            for profile, name in [('common800', 'sip_common800'), ('ours200', 'sip_ours200_mode_sgbm')]:
                condition = work / profile
                condition.mkdir()
                subprocess.run([str(runner), str(manifest), str(condition / 'disparity.bin'),
                                str(condition / 'depth.bin'), str(condition / 'metadata.json'), profile],
                               check=True, env=environment)
                native = json.loads((condition / 'metadata.json').read_text())
                if not Path(native['loaded_library']).resolve(strict=True).samefile(library):
                    raise ValueError('unexpected SIP library loaded')
                if native['source_stamp_ns'] != metadata['source_stamp_ns'] or native['right_stamp_ns'] != metadata['source_stamps']['right_image_stamp_ns']:
                    raise ValueError('SIP replay changed the source stamps')
                depth = np.fromfile(condition / 'depth.bin', np.float32).reshape(native['height'], native['width'])
                candidates[name] = resample_reference(depth, native['k'], baseline_k, captured.shape)
                row['conditions'][name] = {'native_metadata': native, 'native_depth_sha256': sha256(condition / 'depth.bin')}
                np.save(condition / 'depth_native.npy', depth)
            common = np.logical_and.reduce([np.isfinite(depth) & (depth > 0) for depth in candidates.values()])
            for name, depth in candidates.items():
                populations[name].append(depth)
                row['conditions'][name]['own_valid_metrics_mesh'] = depth_metrics(depth, reference)
                row['conditions'][name]['common_all_four_metrics_mesh'] = depth_metrics(depth, np.where(common, reference, np.nan))
                row['conditions'][name]['own_valid_metrics_virtual'] = depth_metrics(depth, virtual)
            np.savez(work / 'candidates.npz', **candidates, mesh_left_depth_m=reference,
                     virtual_depth_m=virtual, common_all_four_mask=common, comparison_k=baseline_k)
            records.append(row)
            source_info.append(str(path.resolve()))
    if not records:
        raise ValueError('no complete raw/mesh population')
    shared = [np.logical_and.reduce([np.isfinite(populations[name][index]) & (populations[name][index] > 0)
                                   for name in names]) for index in range(len(records))]
    report = {'success': True, 'reprocessed_count': len(records), 'skipped': skipped,
              'clock_measurements': 'None: offline precision only, no elapsed timing or sim-time latency inferred',
              'comparison_grid': 'Same archived 200x200 optical pixel rays and mesh_left_depth_m reference',
              'limits': ['SIP common800 native depth is nearest-resampled at half-center 200 rays: at most 0.5 native pixel offset; not a native800 mesh validation',
                         'SIP uses installed MODE_SGBM (0); ours uses MODE_SGBM_3WAY (2); oursettings profile is therefore not the same algorithm',
                         'common800 is the original common launch settings, not retuned live min_disparity=-110 settings',
                         'Depth conversion uses SIP returned corrected d and Z=f*t/d, explicit OpenCV block/search ROI, finite positive 0.25-3m; no hole filling',
                         'Static visual-mesh reference omits dynamic models, alpha textures and Gazebo rasterization; five frames of one scene'],
              'runner': str(runner), 'runner_sha256': sha256(runner),
              'runner_source_sha256': sha256(Path(__file__).with_name('replay_stereo_image_proc.cpp')),
              'installed_library': str(library), 'installed_library_sha256': sha256(library),
              'installed_header_sha256': sha256(Path('/opt/ros/humble/include/stereo_image_proc/stereo_processor.hpp')),
              'config_path': str(args.config.resolve()), 'config_sha256': sha256(args.config),
              'mesh_summary_sha256': sha256(args.mesh_dir / 'mesh_reference_summary.json'),
              'per_frame': records,
              'pooled_own_valid_metrics_mesh': {name: pooled(populations[name], references) for name in names},
              'pooled_common_all_four_metrics_mesh': {name: pooled(populations[name], [np.where(mask, ref, np.nan) for mask, ref in zip(shared, references)]) for name in names}}
    (destination / 'comparison_summary.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    for name, result in report['pooled_own_valid_metrics_mesh'].items():
        print(name, 'valid=', result['stereo_valid_fraction'], 'RMSE=', result['rmse_m'], 'P90=', result['absolute_p90_m'])
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('capture', 'config', 'mesh-dir', 'our-baseline-dir', 'our-refined-dir', 'runner', 'library', 'output-dir'):
        parser.add_argument('--' + name, type=Path, required=True)
    return compare(parser.parse_args())


if __name__ == '__main__':
    raise SystemExit(main())
