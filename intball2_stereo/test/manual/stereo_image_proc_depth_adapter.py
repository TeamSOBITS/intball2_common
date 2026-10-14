#!/usr/bin/env python3
"""Explicit SIP disparity-to-depth comparison adapter; simulation timestamps only.

DisparityImage already subtracts the principal-point difference: Z=f*t/d.
The primary output preserves full resolution. A nearest reduced output records
its half-source-pixel ray approximation. No SIP publish timestamp is inferred.
"""
import argparse
from collections import OrderedDict
import copy
import csv
import json
import math
from pathlib import Path
import sys

import numpy as np

from depth_comparison import decode_depth


class ConversionError(ValueError):
    pass


def stamp_ns(stamp):
    seconds, nanos = int(stamp.sec), int(stamp.nanosec)
    if seconds < 0 or not 0 <= nanos < 1_000_000_000:
        raise ConversionError('invalid_source_stamp')
    return seconds * 1_000_000_000 + nanos


def optical_frame(frame):
    if not frame:
        raise ConversionError('empty_source_frame')
    return 'cameraL_optical_frame' if frame == 'cameraL_link' else frame


def source_header(disparity):
    source = disparity.header
    stamp = stamp_ns(source.stamp)
    frame = optical_frame(source.frame_id)
    inner = disparity.image.header
    if inner.frame_id or stamp_ns(inner.stamp):
        if stamp_ns(inner.stamp) != stamp or optical_frame(inner.frame_id) != frame:
            raise ConversionError('outer_inner_header_mismatch')
    return stamp, frame


def native_metadata(disparity):
    roi = disparity.valid_window
    return {'f_px': float(disparity.f), 'signed_t_m': float(disparity.t),
            'min_disparity_px': float(disparity.min_disparity),
            'max_disparity_px': float(disparity.max_disparity),
            'delta_d_px': float(disparity.delta_d),
            'valid_window': {name: int(getattr(roi, name)) for name in
                             ['x_offset', 'y_offset', 'width', 'height']},
            'image_width': int(disparity.image.width), 'image_height': int(disparity.image.height),
            'image_encoding': disparity.image.encoding, 'image_step': int(disparity.image.step),
            'image_is_bigendian': bool(disparity.image.is_bigendian)}


def valid_rectangle(disparity, policy, sgbm, principal_difference_px):
    width, height = int(disparity.image.width), int(disparity.image.height)
    roi = disparity.valid_window
    raw = (int(roi.x_offset), int(roi.y_offset), int(roi.width), int(roi.height))
    x, y, w, h = raw
    metadata_valid = min(x, y, w, h) >= 0 and w > 0 and h > 0 and x + w <= width and y + h <= height
    if policy == 'message':
        if not metadata_valid:
            raise ConversionError('invalid_valid_window_metadata')
        return raw, metadata_valid
    if policy != 'compute-sgbm-valid-roi' or sgbm is None or principal_difference_px is None:
        raise ConversionError('explicit_sgbm_and_principal_difference_required')
    minimum, count, block = sgbm
    if (not all(isinstance(v, int) for v in sgbm) or count <= 0 or count % 16 or
            block < 3 or block % 2 == 0 or not math.isfinite(principal_difference_px)):
        raise ConversionError('invalid_explicit_sgbm_parameters')
    radius = block // 2
    # All searched raw disparities must have left/right block support. Signed
    # bounds intentionally avoid the upstream unsigned-ROI failure mode.
    raw_low = minimum
    raw_high = minimum + count - 1
    x = int(math.ceil(radius + max(0, raw_high)))
    end = int(math.floor(width - radius + min(0, raw_low)))
    y, h = radius, height - 2 * radius
    x, end = max(0, x), min(width, end)
    if end <= x or h <= 0:
        raise ConversionError('empty_computed_support_window')
    return (x, y, end - x, h), metadata_valid


def convert_disparity(disparity, info, *, min_depth=.25, max_depth=3., output_downsample=1,
                      valid_window_policy='message', sgbm=None, principal_difference_px=None):
    stamp, frame = source_header(disparity)
    if stamp_ns(info.header.stamp) != stamp or optical_frame(info.header.frame_id) != frame:
        raise ConversionError('camera_info_stamp_or_frame_mismatch')
    width, height = int(disparity.image.width), int(disparity.image.height)
    if int(info.width) != width or int(info.height) != height:
        raise ConversionError('camera_info_shape_mismatch')
    if output_downsample not in [1, 4] or width % output_downsample or height % output_downsample:
        raise ConversionError('unsupported_output_downsample_or_shape')
    if not np.all(np.isfinite([min_depth, max_depth])) or not 0 < min_depth < max_depth:
        raise ConversionError('invalid_depth_range')
    p = np.asarray(info.p, dtype=np.float64).reshape(3, 4)
    if not np.isfinite(p).all() or p[0, 0] <= 0 or p[1, 1] <= 0 or not np.allclose(
            [p[0, 1], p[1, 0], *p[2]], [0, 0, 0, 0, 1, 0]):
        raise ConversionError('unsupported_rectified_camera_projection')
    f, t = float(disparity.f), float(disparity.t)
    minimum, maximum = float(disparity.min_disparity), float(disparity.max_disparity)
    if not np.all(np.isfinite([f, t, minimum, maximum])) or f <= 0 or t == 0 or minimum > maximum:
        raise ConversionError('invalid_disparity_geometry_or_search_range')
    if not np.isclose(f, p[0, 0], rtol=1e-5, atol=1e-5):
        raise ConversionError('disparity_f_camera_info_projection_mismatch')
    rectangle, original_roi_valid = valid_rectangle(disparity, valid_window_policy, sgbm,
                                                   principal_difference_px)
    if disparity.image.encoding != '32FC1':
        raise ConversionError('unsupported_disparity_encoding')
    principal_difference_assumed_zero = principal_difference_px is None
    principal_difference_px = 0. if principal_difference_assumed_zero else principal_difference_px
    if not math.isfinite(principal_difference_px):
        raise ConversionError('invalid_principal_difference')
    try:
        values = decode_depth(disparity.image)
    except ValueError as error:
        raise ConversionError(str(error)) from error
    x, y, w, h = rectangle
    support = np.zeros(values.shape, dtype=bool)
    support[y:y + h, x:x + w] = True
    raw_values = values.astype(np.float64) + principal_difference_px
    mask = support & np.isfinite(values) & (values != 0) & (raw_values >= minimum) & (raw_values <= maximum)
    if valid_window_policy == 'compute-sgbm-valid-roi':
        radius = sgbm[2] // 2
        right_x = np.arange(width)[None, :] - (values.astype(float) + principal_difference_px)
        mask &= (right_x - radius >= 0) & (right_x + radius <= width - 1)
    depth = np.full(values.shape, np.nan, dtype=np.float32)
    z = np.zeros(values.shape, dtype=np.float64)
    np.divide(f * t, values, out=z, where=mask)
    mask &= np.isfinite(z) & (z >= min_depth) & (z <= max_depth) & (z <= np.finfo(np.float32).max)
    depth[mask] = z[mask].astype(np.float32)
    native_valid = int(mask.sum())
    factor = output_downsample
    if factor > 1:
        # Ideal area-resize centre is 4*i+1.5; nearest ties select 4*i+2.
        indices_y = np.floor((np.arange(height // factor) + .5) * factor).astype(int)
        indices_x = np.floor((np.arange(width // factor) + .5) * factor).astype(int)
        depth = depth[np.ix_(indices_y, indices_x)].copy()
        p = p.copy()
        p[0, 0] /= factor
        p[1, 1] /= factor
        p[0, 2] = (p[0, 2] + .5) / factor - .5
        p[1, 2] = (p[1, 2] + .5) / factor - .5
        p[0, 3] /= factor
        p[1, 3] /= factor
    k = p[:, :3].copy()
    calibration = {'width': depth.shape[1], 'height': depth.shape[0], 'k': k.reshape(-1).tolist(),
                   'p': p.reshape(-1).tolist(), 'r': [1., 0., 0., 0., 1., 0., 0., 0., 1.],
                   'd': [], 'distortion_model': 'plumb_bob', 'stamp_ns': stamp, 'frame_id': frame}
    metadata = {**native_metadata(disparity), 'source_stamp_ns': stamp,
                'source_frame_id': disparity.header.frame_id, 'output_frame_id': frame,
                'known_gazebo_optical_alias_applied': disparity.header.frame_id == 'cameraL_link',
                'frame_contract': 'known Gazebo optical-axis image alias only; no numeric rotation',
                'depth_formula': 'f * signed_t / principal-adjusted_disparity',
                'valid_window_policy': valid_window_policy, 'original_valid_window_in_bounds': original_roi_valid,
                'effective_support_window': list(rectangle), 'explicit_sgbm': list(sgbm) if sgbm else None,
                'principal_difference_px': principal_difference_px,
                'principal_difference_assumed_zero': principal_difference_assumed_zero,
                'native_valid_depth_count': native_valid, 'output_valid_depth_count': int(np.isfinite(depth).sum()),
                'output_downsample': factor, 'nearest_vs_ideal_source_pixel_offset': .5 if factor > 1 else 0.,
                'min_depth_m': min_depth, 'max_depth_m': max_depth,
                'source_camera_info': {'k': list(info.k), 'p': list(info.p), 'r': list(info.r),
                                       'd': list(info.d), 'distortion_model': info.distortion_model}}
    return depth, calibration, metadata


class SimulationClockTracker:
    def __init__(self):
        self.previous = 0

    def sample(self, now_ns):
        if now_ns <= 0:
            return ('simulation_clock_rewound' if self.previous > 0
                    else 'simulation_clock_uninitialized')
        reason = 'simulation_clock_rewound' if self.previous > now_ns else None
        self.previous = now_ns
        return reason


def simulation_timing(source_ns, receive_ns, conversion_start_ns, ready_ns, publish_ns, reason=None):
    result = {'source_stamp_ns': source_ns, 'disparity_receive_sim_ns': receive_ns,
              'conversion_start_sim_ns': conversion_start_ns, 'adapter_ready_sim_ns': ready_ns,
              'adapter_publish_sim_ns': publish_ns, 'adapter_publish_boundary': 'depth_and_info_publish_completed',
              'sip_native_publish_sim_ns': None}
    if reason is None:
        if min(receive_ns, conversion_start_ns, ready_ns, publish_ns) <= 0:
            reason = 'simulation_clock_uninitialized'
        elif source_ns < 0 or receive_ns < source_ns:
            reason = 'source_stamp_future_or_negative'
        elif not receive_ns <= conversion_start_ns <= ready_ns <= publish_ns:
            reason = 'simulation_clock_rewound_or_intervals_out_of_order'
    native_reason = None
    if receive_ns <= 0:
        native_reason = 'simulation_clock_uninitialized'
    elif source_ns < 0 or receive_ns < source_ns:
        native_reason = 'source_stamp_future_or_negative'
    elif reason in ['simulation_clock_uninitialized', 'simulation_clock_rewound']:
        native_reason = reason
    result['native_timing_valid'] = native_reason is None
    result['native_timing_invalid_reason'] = native_reason
    result['timing_valid'] = reason is None
    result['timing_invalid_reason'] = reason
    intervals = {'source_to_disparity_receive_sim_s': (receive_ns, source_ns),
                 'disparity_receive_to_conversion_start_sim_s': (conversion_start_ns, receive_ns),
                 'adapter_conversion_sim_s': (ready_ns, conversion_start_ns),
                 'adapter_ready_to_publish_sim_s': (publish_ns, ready_ns),
                 'source_to_adapter_publish_sim_s': (publish_ns, source_ns)}
    result.update({key: (end - start) / 1e9 if reason is None else None
                   for key, (end, start) in intervals.items()})
    stage_available = reason is None and publish_ns > conversion_start_ns
    result['adapter_stage_timing_available'] = stage_available
    result['adapter_stage_timing_unavailable_reason'] = (None if stage_available else
        reason or 'no_simulation_clock_advance_during_adapter')
    if not stage_available:
        result['adapter_conversion_sim_s'] = None
        result['adapter_ready_to_publish_sim_s'] = None
    result['source_to_disparity_receive_sim_s'] = ((receive_ns - source_ns) / 1e9
        if native_reason is None else None)
    return result


def assign_camera_calibration(info, calibration):
    # ROS generated setters reject integer members in float64 sequences.
    info.k = [float(value) for value in calibration['k']]
    info.p = [float(value) for value in calibration['p']]
    info.r = [float(value) for value in calibration['r']]
    info.d = [float(value) for value in calibration['d']]


def finite_json(value):
    if isinstance(value, dict):
        return {key: finite_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite_json(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return 'NaN' if math.isnan(value) else ('+Infinity' if value > 0 else '-Infinity')
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--disparity-topic', default='/stereo/disparity')
    parser.add_argument('--camera-info-topic', default='/stereo/left/camera_info')
    parser.add_argument('--depth-topic', default='/sip_adapter/depth')
    parser.add_argument('--output-info-topic', default='/sip_adapter/camera_info')
    parser.add_argument('--timing-topic', default='/sip_adapter/timing')
    parser.add_argument('--duration-sim-s', type=float, default=0.)
    parser.add_argument('--max-info-wait-sim-s', type=float, default=.25)
    parser.add_argument('--queue-size', type=int, default=100)
    parser.add_argument('--max-samples', type=int, default=10000)
    parser.add_argument('--output-downsample', type=int, choices=[1, 4], default=1)
    parser.add_argument('--valid-window-policy', choices=['message', 'compute-sgbm-valid-roi'], default='message')
    parser.add_argument('--sgbm-min-disparity', type=int)
    parser.add_argument('--sgbm-num-disparities', type=int)
    parser.add_argument('--sgbm-block-size', type=int)
    parser.add_argument('--principal-difference-px', type=float)
    args, ros_args = parser.parse_known_args(argv)
    if (not math.isfinite(args.duration_sim_s) or args.duration_sim_s < 0 or
            not math.isfinite(args.max_info_wait_sim_s) or args.max_info_wait_sim_s <= 0 or
            not 1 <= args.queue_size <= 10000 or not 1 <= args.max_samples <= 100000):
        parser.error('invalid simulation duration, queue or sample limits')
    sgbm = None
    if args.valid_window_policy != 'message':
        values = [args.sgbm_min_disparity, args.sgbm_num_disparities, args.sgbm_block_size]
        if any(value is None for value in values) or args.principal_difference_px is None or not math.isfinite(args.principal_difference_px):
            parser.error('computed support requires all explicit SGBM values and principal difference')
        sgbm = tuple(values)
    output = args.output_dir.resolve()
    if output.is_relative_to(Path(__file__).resolve().parents[2]):
        parser.error('adapter outputs must be outside checkout')
    output.mkdir(parents=True, exist_ok=False)

    import rclpy
    from rclpy.node import Node
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image, RegionOfInterest
    from stereo_msgs.msg import DisparityImage
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

    class Adapter(Node):
        def __init__(self):
            super().__init__('stereo_image_proc_depth_adapter', parameter_overrides=[Parameter('use_sim_time', value=True)])
            if not self.get_parameter('use_sim_time').value:
                raise RuntimeError('adapter requires use_sim_time=true')
            # Data callbacks serialize shared queues. /clock stays in the node's
            # default group so the second executor thread can update simulation
            # time while conversion is executing.
            self.data_callback_group = MutuallyExclusiveCallbackGroup()
            self.depth_publisher = self.create_publisher(Image, args.depth_topic, qos_profile_sensor_data)
            self.info_publisher = self.create_publisher(CameraInfo, args.output_info_topic, qos_profile_sensor_data)
            self.timing_publisher = self.create_publisher(DiagnosticArray, args.timing_topic, qos_profile_sensor_data)
            self.infos, self.pending = OrderedDict(), OrderedDict()
            self.clock_tracker = SimulationClockTracker()
            self.rows, self.frames = [], []
            self.counts = {name: 0 for name in ['disparity_received', 'camera_info_received', 'published',
                                             'conversion_rejected', 'missing_camera_info', 'pending_overflow',
                                             'camera_info_cache_overflow', 'camera_info_rejected', 'clock_rewinds']}
            self.start_ns = None
            self.done, self.complete = False, False
            self.stop_reason = 'interrupted'
            self.runtime_exception = None
            self.create_subscription(DisparityImage, args.disparity_topic, self.receive_disparity, qos_profile_sensor_data,
                                     callback_group=self.data_callback_group)
            self.create_subscription(CameraInfo, args.camera_info_topic, self.receive_info, qos_profile_sensor_data,
                                     callback_group=self.data_callback_group)
            self.create_timer(.02, self.poll, callback_group=self.data_callback_group)

        def clock_sample(self):
            now = self.get_clock().now().nanoseconds
            reason = self.clock_tracker.sample(now)
            if reason == 'simulation_clock_rewound':
                self.counts['clock_rewinds'] += 1
                for stamp in list(self.pending):
                    self.reject(stamp, 'simulation_clock_rewound')
                self.infos.clear()
                self.done, self.stop_reason = True, reason
            if self.start_ns is None and now > 0:
                self.start_ns = now
            return now, reason

        def receive_info(self, message):
            now, reason = self.clock_sample()
            if self.done:
                return
            self.counts['camera_info_received'] += 1
            try:
                stamp = stamp_ns(message.header.stamp)
            except ConversionError:
                self.counts['camera_info_rejected'] += 1
                return
            if reason or stamp > now:
                self.counts['camera_info_rejected'] += 1
                return
            self.infos[stamp] = message
            while len(self.infos) > args.queue_size:
                self.infos.popitem(last=False)
                self.counts['camera_info_cache_overflow'] += 1
            if stamp in self.pending:
                self.convert(stamp)

        def receive_disparity(self, message):
            now, reason = self.clock_sample()
            if self.done:
                return
            self.counts['disparity_received'] += 1
            try:
                stamp = stamp_ns(message.header.stamp)
            except ConversionError:
                stamp = -1
                reason = 'invalid_source_stamp'
            if stamp in self.pending:
                self.reject(stamp, 'duplicate_source_stamp')
            self.pending[stamp] = (message, now, reason)
            if reason or stamp > now:
                self.reject(stamp, reason or 'source_stamp_future')
                return
            while len(self.pending) > args.queue_size:
                self.reject(next(iter(self.pending)), 'pending_queue_overflow')
                self.counts['pending_overflow'] += 1
            if stamp in self.infos:
                self.convert(stamp)

        def reject(self, stamp, reason):
            message, receive, clock_reason = self.pending.pop(stamp)
            self.counts['conversion_rejected'] += 1
            if 'missing_camera_info' in reason:
                self.counts['missing_camera_info'] += 1
            row = simulation_timing(stamp, receive, 0, 0, 0, clock_reason or reason)
            row.update(conversion_success=False, conversion_rejection_reason=reason)
            self.rows.append(row)
            self.frames.append({'source_stamp_ns': stamp, 'conversion_success': False,
                                'conversion_rejection_reason': reason,
                                'source_frame_id': message.header.frame_id,
                                'matched_left_camera_info_p': list(self.infos[stamp].p) if stamp in self.infos else None,
                                **native_metadata(message)})
            self.enforce_capacity()

        def enforce_capacity(self):
            if len(self.rows) >= args.max_samples and not self.done:
                self.done, self.stop_reason = True, 'sample_capacity_reached'

        def convert(self, stamp):
            message, receive, clock_reason = self.pending[stamp]
            started, start_reason = self.clock_sample()
            if self.done:
                return
            try:
                depth, calibration, metadata = convert_disparity(
                    message, self.infos[stamp], output_downsample=args.output_downsample,
                    valid_window_policy=args.valid_window_policy, sgbm=sgbm,
                    principal_difference_px=args.principal_difference_px)
            except (ValueError, TypeError) as error:
                self.reject(stamp, str(error))
                return
            image = Image()
            image.header = copy.deepcopy(message.header)
            image.header.frame_id = calibration['frame_id']
            image.width, image.height = calibration['width'], calibration['height']
            image.encoding, image.step = '32FC1', depth.shape[1] * 4
            image.is_bigendian = sys.byteorder == 'big'
            image.data = depth.tobytes()
            info = copy.deepcopy(self.infos[stamp])
            info.header = copy.deepcopy(image.header)
            info.width, info.height = image.width, image.height
            assign_camera_calibration(info, calibration)
            info.distortion_model = calibration['distortion_model']
            info.binning_x = info.binning_y = 0
            info.roi = RegionOfInterest()
            ready, ready_reason = self.clock_sample()
            if self.done:
                return
            self.pending.pop(stamp)
            try:
                self.depth_publisher.publish(image)
                self.info_publisher.publish(info)
            except Exception:
                self.pending[stamp] = (message, receive, clock_reason)
                raise
            published, publish_reason = self.clock_sample()
            timing = simulation_timing(stamp, receive, started, ready, published,
                                       clock_reason or start_reason or ready_reason or publish_reason)
            timing.update(conversion_success=True, conversion_rejection_reason=None)
            diagnostic = DiagnosticArray()
            diagnostic.header = copy.deepcopy(image.header)
            status = DiagnosticStatus()
            status.name = 'stereo_image_proc_depth_adapter/simulation_timing'
            status.level = DiagnosticStatus.OK if timing['timing_valid'] else DiagnosticStatus.ERROR
            status.message = 'native receive and adapter stages separated; SIP native publish unknown'
            status.values = [KeyValue(key=key, value=str(value)) for key, value in timing.items()]
            diagnostic.status = [status]
            self.timing_publisher.publish(diagnostic)
            self.rows.append(timing)
            self.frames.append({'conversion_success': True, **metadata, 'output_calibration': calibration})
            self.counts['published'] += 1
            self.enforce_capacity()

        def poll(self):
            now, reason = self.clock_sample()
            if self.done or reason:
                return
            for stamp, (_, receive, _) in list(self.pending.items()):
                if now - receive > args.max_info_wait_sim_s * 1e9:
                    self.reject(stamp, 'missing_camera_info_sim_timeout')
            if args.duration_sim_s and self.start_ns is not None and now - self.start_ns >= args.duration_sim_s * 1e9:
                self.done, self.complete, self.stop_reason = True, True, 'simulation_duration_reached'

        def save(self):
            for stamp in list(self.pending):
                self.reject(stamp, 'missing_camera_info_at_stop')
            fieldnames = list(simulation_timing(0, 0, 0, 0, 0)) + ['conversion_success', 'conversion_rejection_reason']
            with (output / 'native_adapter_timing.csv').open('w', newline='') as stream:
                writer = csv.DictWriter(stream, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(self.rows)
            summary = {'complete': self.complete, 'stop_reason': self.stop_reason,
                       'arguments': {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()},
                       'counts': self.counts, 'runtime_exception': self.runtime_exception,
                       'simulation_window_start_ns': self.start_ns,
                       'frames': self.frames, 'timing_sample_count': len(self.rows),
                       'timing_contract': 'source-to-disparity-receive is native SIP plus delivery; native publish unknown; adapter conversion/join/publish separate; all intervals simulation clock',
                       'quality_contract': 'exact left CameraInfo stamp; SIP accepted right/info pairing is unknown; no fabricated ours raw-bundle stamps',
                       'executor_contract': 'two threads; data callbacks mutually exclusive; default clock group independent; shutdown joins before saving',
                       'io_contract': 'CSV/JSON saved after executor threads stop'}
            (output / 'summary.json').write_text(json.dumps(finite_json(summary), indent=2, allow_nan=False) + '\n')

    rclpy.init(args=ros_args)
    node = None
    executor = None
    try:
        node = Adapter()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        (output / 'COLLECTION_STARTED').write_text('adapter collection started\n')
        while rclpy.ok() and not node.done:
            executor.spin_once(timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        if node is not None:
            node.complete = False
            node.done = True
            node.stop_reason = 'runtime_exception'
            node.runtime_exception = {'type': type(error).__name__, 'message': str(error)}
        raise
    finally:
        if executor is not None:
            executor.shutdown()
            # Humble has no public wait_for_threads flag; join its thread pool
            # after stopping dispatch so queued callbacks cannot race with save.
            executor._executor.shutdown(wait=True)
        if node is not None:
            if node.runtime_exception is not None:
                for stamp in list(node.pending):
                    node.reject(stamp, 'runtime_exception')
            node.save()
            if node.complete:
                (output / 'COMPLETE').write_text('simulation duration reached; summary saved\n')
            else:
                (output / 'INCOMPLETE').write_text(node.stop_reason + '\n')
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
