#!/usr/bin/env python3
"""Collect stereo/reference quality and latency using the ROS simulation clock."""

import argparse
from collections import OrderedDict
import csv
import json
from pathlib import Path
import sys

import numpy as np

from depth_comparison import decode_depth, depth_metrics, resample_reference, timing_percentiles
from input_capture import (BoundedInputCapture, StaticMountStore, camera_info_metadata, diagnostic_delta,
                           capture_event_sim_time, image_metadata, transform_matrix, validate_input_pair)


TIMING_METRIC_FIELDS = ["source_to_publish_sim_s", "source_to_depth_ready_sim_s", "processing_sim_s",
                        "source_to_left_callback_sim_s", "left_callback_to_bundle_sim_s",
                        "last_callback_to_bundle_sim_s", "four_stream_arrival_span_sim_s",
                        "four_stream_source_stamp_span_sim_s", "bundle_to_worker_start_sim_s",
                        "worker_setup_sim_s", "preprocessing_sim_s", "matching_sim_s",
                        "depth_conversion_sim_s", "refinement_sim_s", "core_output_metadata_sim_s", "output_copy_sim_s",
                        "depth_ready_to_publish_sim_s"]


def timing_values(values):
    return {item.key: int(item.value) if item.key.endswith("_stamp_ns") else float(item.value)
            for item in values}


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def cache_put(cache, key, value, limit):
    cache[key] = value
    while len(cache) > limit:
        cache.popitem(last=False)


def finite_json(value):
    if isinstance(value, dict):
        return {key: finite_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite_json(item) for item in value]
    if isinstance(value, np.ndarray):
        return finite_json(value.tolist())
    if isinstance(value, (float, np.floating)):
        return float(value) if np.isfinite(value) else None
    if isinstance(value, np.integer):
        return int(value)
    return value


def arguments_metadata(args):
    values = dict(vars(args))
    values["output_dir"] = str(args.output_dir)
    return values


def reference_timing_row(header_stamp_ns, receive_sim_ns):
    return {"reference_header_stamp_ns": header_stamp_ns,
            "reference_receive_sim_ns": receive_sim_ns,
            "reference_header_to_output_sim_s": (receive_sim_ns - header_stamp_ns) / 1e9}


def capture_status(start_ns, last_ns, duration_ns, failure=None, interrupted=False):
    complete = (not interrupted and start_ns is not None and
                last_ns >= start_ns + duration_ns)
    if interrupted:
        failure = failure or "capture interrupted before finalization"
    elif not complete:
        failure = failure or "simulation capture duration not reached"
    return {"complete": complete, "interrupted": interrupted, "failure": failure}


def pose_motion(poses, midpoint_ns):
    if len(poses) < 2:
        raise ValueError("at least two navigation poses are required")
    _, origin, quaternion = min(poses, key=lambda pose: abs(pose[0] - midpoint_ns))
    distances, angles = [], []
    origin_norm = np.linalg.norm(quaternion)
    if not np.isfinite(origin_norm) or origin_norm <= 0:
        raise ValueError("invalid navigation quaternion")
    quaternion = quaternion / origin_norm
    for _, position, rotation in poses:
        norm = np.linalg.norm(rotation)
        if not np.isfinite(norm) or norm <= 0:
            raise ValueError("invalid navigation quaternion")
        distances.append(float(np.linalg.norm(position - origin)))
        dot = np.clip(abs(np.dot(rotation / norm, quaternion)), 0.0, 1.0)
        angles.append(float(np.degrees(2 * np.arccos(dot))))
    return max(distances), max(angles)


def image_info_match(image, info):
    if (stamp_ns(image.header.stamp) != stamp_ns(info.header.stamp)
            or image.header.frame_id != info.header.frame_id
            or image.width != info.width or image.height != info.height):
        raise ValueError("image/CameraInfo stamp, frame or shape mismatch")
    k = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
    if not np.all(np.isfinite(k)) or k[0, 0] <= 0 or k[1, 1] <= 0:
        raise ValueError("invalid CameraInfo intrinsics")
    return k


def validate_capture_evidence(rows, samples, camera_validated, timing, policy="required"):
    if not rows or not samples or not camera_validated:
        raise ValueError("missing timing, aligned quality samples or validated camera input")
    if policy == "required":
        if not timing:
            raise ValueError("missing simulation timing diagnostics")
        if not any(row["source_stamp_ns"] in timing for row in rows):
            raise ValueError("simulation timing diagnostics do not match captured depths")
    elif policy != "observer-only":
        raise ValueError("unknown node timing policy")


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--duration-sim-s", type=float, default=20.0)
    parser.add_argument("--sample-hz", type=float, default=5.0)
    parser.add_argument("--max-reference-stamp-difference-s", type=float, default=0.1)
    parser.add_argument("--camera-image-topic", default="/camera_left/image_raw")
    parser.add_argument("--camera-info-topic", default="/camera_left/camera_info")
    parser.add_argument("--right-image-topic", default="/camera_right/image_raw")
    parser.add_argument("--right-info-topic", default="/camera_right/camera_info")
    parser.add_argument("--stereo-depth-topic", default="/stereo/depth")
    parser.add_argument("--stereo-info-topic", default="/stereo/camera_info")
    parser.add_argument("--timing-topic", default="/stereo/timing")
    parser.add_argument("--node-timing-policy", choices=("required", "observer-only"), default="required")
    parser.add_argument("--diagnostics-topic", default="/diagnostics")
    parser.add_argument("--reference-depth-topic", default="/virtual_camera/stereo/depth")
    parser.add_argument("--reference-info-topic", default="/virtual_camera/stereo/camera_info")
    parser.add_argument("--navigation-topic", default="/nav/true/navigation")
    parser.add_argument("--pose-source", choices=("tf", "navigation"), default="tf")
    parser.add_argument("--world-frame", default="iss_body")
    parser.add_argument("--body-frame", default="body")
    parser.add_argument("--optical-frame", default="cameraL_optical_frame")
    parser.add_argument("--right-optical-frame", default="cameraR_optical_frame")
    parser.add_argument("--save-inputs", action="store_true")
    parser.add_argument("--max-input-samples", type=int, default=10)
    parser.add_argument("--max-input-bytes", type=int, default=64 * 1024 * 1024)
    parser.add_argument("--min-depth-m", type=float, default=0.25)
    parser.add_argument("--max-depth-m", type=float, default=3.0)
    args = parser.parse_args(argv)
    if args.save_inputs and args.node_timing_policy == "observer-only":
        parser.error("observer-only cannot save raw input bundles without the four node source stamps")
    for name in ("duration_sim_s", "sample_hz", "max_reference_stamp_difference_s"):
        value = getattr(args, name)
        if not np.isfinite(value) or value <= 0:
            parser.error(f"{name} must be positive and finite")
    if args.sample_hz * args.duration_sim_s > 1000:
        parser.error("capture is limited to 1000 quality samples")
    if not 1 <= args.max_input_samples <= 1000 or not 0 < args.max_input_bytes < 2**31 - 1:
        parser.error("raw sample limit must be 1..1000 and bytes 1..2147483646")
    if args.save_inputs and args.output_dir.resolve().is_relative_to(Path(__file__).resolve().parents[2]):
        parser.error("raw inputs must be saved outside the checkout")
    return args


def main(argv=None):
    args = arguments(argv)
    # ROS imports remain below argument parsing so --help needs no ROS runtime.
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
    from rclpy.duration import Duration
    from rclpy.time import Time
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer, TransformException

    class Collector(Node):
        def __init__(self):
            super().__init__("compare_sim_depth", parameter_overrides=[Parameter("use_sim_time", value=True)])
            self.start_ns = None
            self.last_clock_ns = 0
            self.error = None
            self.timing = OrderedDict()
            self.timing_invalid_count = 0
            self.quality_queue_dropped = 0
            self.caches = {name: OrderedDict() for name in ("camera", "camera_info", "stereo_info", "reference", "reference_info")}
            self.last_stamps = {}
            self.pending = OrderedDict()
            self.rows = []
            self.reference_rows = []
            self.stream_counts = {}
            self.diagnostics_first = None
            self.diagnostics_last = None
            self.raw_capture = BoundedInputCapture(args.max_input_samples, args.max_input_bytes) if args.save_inputs else None
            self.raw_missing_count = 0
            self.samples = []
            self.poses = []
            self.camera_validated = 0
            self.next_sample_ns = None
            self.tf_buffer = Buffer(cache_time=Duration(seconds=30), node=self)
            self.mount_buffer = Buffer(cache_time=Duration(seconds=30)) if args.save_inputs else None
            self.mount_store = StaticMountStore((args.body_frame, args.world_frame)) if args.save_inputs else None
            if args.save_inputs:
                self.caches.update({name: OrderedDict() for name in ("right", "right_info")})
            self.subscriptions_owned = []
            for name, topic, kind in (
                ("camera", args.camera_image_topic, Image),
                ("camera_info", args.camera_info_topic, CameraInfo),
                ("right", args.right_image_topic, Image),
                ("right_info", args.right_info_topic, CameraInfo),
                ("stereo", args.stereo_depth_topic, Image),
                ("stereo_info", args.stereo_info_topic, CameraInfo),
                ("timing", args.timing_topic, DiagnosticArray),
                ("reference", args.reference_depth_topic, Image),
                ("reference_info", args.reference_info_topic, CameraInfo),
            ):
                self.subscriptions_owned.append(self.create_subscription(
                    kind, topic, lambda message, stream=name: self.guarded(stream, message), qos_profile_sensor_data))
            self.subscriptions_owned.append(self.create_subscription(
                DiagnosticArray, args.diagnostics_topic, lambda message: self.guarded("diagnostics", message),
                QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)))
            if args.pose_source == "navigation":
                from ib2_msgs.msg import Navigation
                self.subscriptions_owned.append(self.create_subscription(
                    Navigation, args.navigation_topic, lambda message: self.guarded("pose", message), qos_profile_sensor_data))
            else:
                self.subscriptions_owned.append(self.create_subscription(
                    TFMessage, "/tf", lambda message: self.guarded("tf", message), qos_profile_sensor_data))
            if args.save_inputs:
                static_qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                        reliability=ReliabilityPolicy.RELIABLE)
                self.subscriptions_owned.append(self.create_subscription(
                    TFMessage, "/tf_static", lambda message: self.guarded("mount", message), static_qos))
                if args.pose_source == "navigation":
                    self.subscriptions_owned.append(self.create_subscription(
                        TFMessage, "/tf", lambda message: self.guarded("tf", message), qos_profile_sensor_data))

        def now_ns(self):
            now = self.get_clock().now().nanoseconds
            if now < self.last_clock_ns:
                raise ValueError("simulation clock rewound")
            self.last_clock_ns = now
            if now > 0 and self.start_ns is None:
                self.start_ns = now
            return now

        def guarded(self, stream, message):
            try:
                self.receive(stream, message)
            except Exception as error:
                self.error = f"{stream}: {type(error).__name__}: {error}"

        def receive(self, stream, message):
            # Transient-local static messages may arrive only once, before /clock.
            now = capture_event_sim_time(stream, message, self.now_ns, self.mount_store, self.mount_buffer)
            if now is None or now <= 0:
                return
            if stream == "diagnostics":
                for status in message.status:
                    if status.name == "intball2_stereo/processing":
                        snapshot = {"receive_sim_ns": now, "header_stamp_ns": stamp_ns(message.header.stamp),
                                    "values": {item.key: item.value for item in status.values}}
                        if self.diagnostics_first is None:
                            self.diagnostics_first = snapshot
                        self.diagnostics_last = snapshot
                return
            if stream == "tf":
                for transform in message.transforms:
                    self.tf_buffer.set_transform(transform, "depth_comparison_dynamic_tf")
                try:
                    transform = self.tf_buffer.lookup_transform(
                        args.world_frame, args.body_frame, Time(), timeout=Duration(seconds=0))
                except TransformException:
                    return
                pose_stamp = stamp_ns(transform.header.stamp)
                if abs(now - pose_stamp) > 100000000:
                    return
                value = transform.transform
                position = np.array([value.translation.x, value.translation.y, value.translation.z])
                rotation = np.array([value.rotation.x, value.rotation.y, value.rotation.z, value.rotation.w])
                if args.pose_source == "tf":
                    self.append_pose(pose_stamp, position, rotation)
                return
            if stream == "timing":
                for status in message.status:
                    if status.name != "intball2_stereo/simulation_timing":
                        continue
                    if status.level != DiagnosticStatus.OK:
                        self.timing_invalid_count += 1
                        continue
                    values = timing_values(status.values)
                    required = {"source_to_publish_sim_s", "source_to_depth_ready_sim_s", "processing_sim_s"}
                    if not required.issubset(values) or not all(np.isfinite(value) for value in values.values()):
                        raise ValueError("invalid timing diagnostic values")
                    if message.header.frame_id != args.optical_frame:
                        raise ValueError("timing diagnostic optical frame mismatch")
                    cache_put(self.timing, stamp_ns(message.header.stamp), values, 100000)
                self.pair_samples()
                return
            if stream == "pose":
                pose = message.pose.pose
                position = np.array([pose.position.x, pose.position.y, pose.position.z])
                rotation = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
                self.append_pose(stamp_ns(message.pose.header.stamp), position, rotation)
                return
            stamp = stamp_ns(message.header.stamp)
            if stamp <= self.last_stamps.get(stream, -1):
                raise ValueError("nonmonotonic source stamps")
            self.last_stamps[stream] = stamp
            self.stream_counts[stream] = self.stream_counts.get(stream, 0) + 1
            if stream in ("right", "right_info") and not args.save_inputs:
                return
            if stream == "reference":
                self.reference_rows.append(reference_timing_row(stamp, now))
                if len(self.reference_rows) > 100000:
                    raise ValueError("reference timing sample capacity exceeded")
            if stream == "stereo":
                received = self.caches["camera"].get(stamp)
                self.rows.append({"source_stamp_ns": stamp, "output_receive_sim_ns": now,
                                  "source_to_output_sim_s": (now - stamp) / 1e9,
                                  "input_receive_to_output_sim_s": None if received is None else (now - received[1]) / 1e9})
                if len(self.rows) > 100000:
                    raise ValueError("timing sample capacity exceeded")
                if self.next_sample_ns is None or stamp >= self.next_sample_ns:
                    self.next_sample_ns = stamp + round(1e9 / args.sample_hz)
                    if len(self.pending) >= 16:
                        self.quality_queue_dropped += 1
                    cache_put(self.pending, stamp, message, 16)
            else:
                cache_put(self.caches[stream], stamp, (message, now), 64 if stream in ("camera", "camera_info", "right", "right_info") else 16)
            self.pair_samples()

        def append_pose(self, stamp, position, rotation):
            if not np.all(np.isfinite(position)) or not np.all(np.isfinite(rotation)) or np.linalg.norm(rotation) <= 0:
                raise ValueError("nonfinite position or invalid quaternion")
            if self.poses and stamp <= self.poses[-1][0]:
                return
            if len(self.poses) >= 100000:
                raise ValueError("pose sample capacity exceeded")
            self.poses.append((stamp, position, rotation))

        def exact_camera_pose(self, requested_ns, optical_frame):
            self.mount_store.install(self.mount_buffer)
            query = Time(nanoseconds=requested_ns, clock_type=self.get_clock().clock_type)
            body = self.tf_buffer.lookup_transform(args.world_frame, args.body_frame, query,
                                                   timeout=Duration(seconds=0))
            if stamp_ns(body.header.stamp) != requested_ns:
                raise TransformException("body transform did not return requested source stamp")
            mounting = self.mount_buffer.lookup_transform(args.body_frame, optical_frame, Time(),
                                                          timeout=Duration(seconds=0))
            def components(transform):
                value = transform.transform
                return ([value.translation.x, value.translation.y, value.translation.z],
                        [value.rotation.x, value.rotation.y, value.rotation.z, value.rotation.w])
            body_translation, body_rotation = components(body)
            mount_translation, mount_rotation = components(mounting)
            return {"parent": args.world_frame, "child": optical_frame,
                    "requested_stamp_ns": requested_ns, "returned_body_stamp_ns": stamp_ns(body.header.stamp),
                    "mode": "TF at requested source time; TF may interpolate; no nearest/latest body fallback",
                    "world_to_body": transform_matrix(body_translation, body_rotation).tolist(),
                    "body_to_optical_fixed": transform_matrix(mount_translation, mount_rotation).tolist(),
                    "world_to_optical": (transform_matrix(body_translation, body_rotation)
                                         @ transform_matrix(mount_translation, mount_rotation)).tolist()}

        def save_raw_sample(self, sample, depth, stereo_info, reference, reference_info):
            if self.raw_capture is None:
                return
            if len(self.raw_capture.samples) >= args.max_input_samples:
                self.raw_capture.limit_rejections += 1
                sample["raw_capture"] = {"saved": False, "reason": "sample capacity"}
                return
            stamp = sample["stamp_ns"]
            timing = self.timing.get(stamp, {})
            required = ("left_image_stamp_ns", "right_image_stamp_ns", "left_info_stamp_ns", "right_info_stamp_ns")
            missing = ["timing." + key for key in required if key not in timing]
            messages = {}
            for name, key in zip(("camera", "right", "camera_info", "right_info"), required):
                entry = self.caches[name].get(timing[key]) if key in timing else None
                if entry is None:
                    missing.append(name + " exact source stamp")
                else:
                    messages[name] = entry[0]
            arrays = {"stereo_depth_bytes": np.frombuffer(depth.data, dtype=np.uint8),
                      "reference_depth_bytes": np.frombuffer(reference.data, dtype=np.uint8)}
            metadata = {"format_version": 1, "source_stamp_ns": stamp,
                        "reference_pose_stamp_ns": sample["reference_stamp_ns"],
                        "config_contract": "Run configuration must remain fixed; root stores YAML separately",
                        "source_stamps": {key: timing.get(key) for key in required},
                        "bundle_association": "Exact joins on C++ accepted bundle's four integer source stamps; approximate synchronization is retained",
                        "four_source_stamp_span_ns": max(timing[key] for key in required) - min(timing[key] for key in required) if not missing else None,
                        "images": {"stereo_depth": image_metadata(depth), "reference_depth": image_metadata(reference)},
                        "camera_info": {"stereo": camera_info_metadata(stereo_info), "reference": camera_info_metadata(reference_info)},
                        "poses": {}, "missing": missing}
            metadata["pose_requirements"] = "Dynamic world-to-body TF at each requested source time and static body-to-camera optical mounts, including the right optical frame"
            for name in ("camera", "right"):
                if name in messages:
                    metadata["images"][name] = image_metadata(messages[name])
                    arrays[name + "_image_bytes"] = np.frombuffer(messages[name].data, dtype=np.uint8)
            for name in ("camera_info", "right_info"):
                if name in messages:
                    metadata["camera_info"][name] = camera_info_metadata(messages[name])
            for image_name, info_name in (("camera", "camera_info"), ("right", "right_info")):
                if image_name in messages and info_name in messages:
                    validate_input_pair(messages[image_name], messages[info_name])
            for name, requested, optical in (
                ("left", timing.get("left_image_stamp_ns"), args.optical_frame),
                ("right", timing.get("right_image_stamp_ns"), args.right_optical_frame),
                ("reference", sample["reference_stamp_ns"], args.optical_frame)):
                if requested is None:
                    continue
                try:
                    metadata["poses"][name] = self.exact_camera_pose(requested, optical)
                except TransformException as error:
                    missing.append(name + " exact pose: " + str(error))
            metadata["usable_replay"] = not missing
            metadata["active_config_revision"] = None if self.diagnostics_last is None else self.diagnostics_last["values"].get("active_config_revision")
            metadata["config_revision_source"] = "Latest periodic diagnostic snapshot, not a per-frame revision; fixed run configuration required"
            index = self.raw_capture.add(arrays, metadata)
            if missing:
                self.raw_missing_count += 1
            sample["raw_capture"] = {"saved": index is not None, "sample_index": index,
                                     "usable_replay": not missing, "missing": missing,
                                     "reason": "byte capacity" if index is None else None}

        def pair_samples(self, final=False):
            camera = self.caches["camera"]
            camera_info = self.caches["camera_info"]
            for stamp in set(camera).intersection(camera_info):
                if not camera[stamp][2:] and image_info_match(camera[stamp][0], camera_info[stamp][0]) is not None:
                    camera[stamp] = (*camera[stamp], True)
                    self.camera_validated += 1
            references = self.caches["reference"]
            tolerance = round(args.max_reference_stamp_difference_s * 1e9)
            for stamp, depth in list(self.pending.items()):
                if args.save_inputs and stamp not in self.timing and not final:
                    continue
                info = self.caches["stereo_info"].get(stamp)
                eligible = set(references).intersection(self.caches["reference_info"])
                if info is None or not eligible:
                    continue
                if stamp not in eligible and not final and max(eligible) < stamp + tolerance:
                    continue
                reference_stamp = min(eligible, key=lambda value: abs(value - stamp))
                if abs(reference_stamp - stamp) > tolerance:
                    continue
                reference = references[reference_stamp][0]
                reference_info = self.caches["reference_info"][reference_stamp][0]
                stereo_k = image_info_match(depth, info[0])
                reference_k = image_info_match(reference, reference_info)
                if depth.header.frame_id != args.optical_frame or reference.header.frame_id != args.optical_frame:
                    raise ValueError("stereo/reference optical frame differs from expected frame")
                stereo_array = decode_depth(depth)
                native_reference = decode_depth(reference)
                reference_array = resample_reference(native_reference, reference_k, stereo_k, stereo_array.shape)
                self.samples.append({"stamp_ns": stamp, "reference_stamp_ns": reference_stamp,
                                     "reference_stamp_difference_s": (reference_stamp - stamp) / 1e9,
                                     "metrics": depth_metrics(stereo_array, reference_array, args.min_depth_m, args.max_depth_m)})
                self.save_raw_sample(self.samples[-1], depth, info[0], reference, reference_info)
                self.profile = {"stereo_depth_m": stereo_array, "reference_depth_m": reference_array,
                                "reference_native_depth_m": native_reference,
                                "stereo_k": stereo_k, "reference_k": reference_k,
                                "source_stamp_ns": np.int64(stamp), "reference_stamp_ns": np.int64(reference_stamp)}
                del self.pending[stamp]

    rclpy.init(args=[])
    node = Collector()
    try:
        interrupted = False
        try:
            while rclpy.ok() and node.error is None:
                rclpy.spin_once(node, timeout_sec=0.1)
                try:
                    now = node.now_ns()
                except Exception as error:
                    node.error = str(error)
                    break
                if node.start_ns is not None and now - node.start_ns >= round(args.duration_sim_s * 1e9):
                    break
        except (KeyboardInterrupt, ExternalShutdownException) as error:
            interrupted = True
            node.error = node.error or f"capture interrupted: {type(error).__name__}"
        status = capture_status(node.start_ns, node.last_clock_ns,
                                round(args.duration_sim_s * 1e9), node.error, interrupted)
        node.error = status["failure"]
        translation, angle = None, None
        try:
            node.pair_samples(final=True)
            translation, angle = pose_motion(node.poses, node.start_ns + round(args.duration_sim_s * 5e8))
            if translation > 0.005 or angle > 0.2:
                raise ValueError(f"capture moved: {translation:.6f} m, {angle:.6f} deg")
            validate_capture_evidence(node.rows, node.samples, node.camera_validated,
                                      node.timing, args.node_timing_policy)
            if node.poses[0][0] > min(item["stamp_ns"] for item in node.samples) + 100000000 or node.poses[-1][0] < max(item["stamp_ns"] for item in node.samples) - 100000000:
                raise ValueError("navigation poses do not cover quality capture window")
        except Exception as error:
            node.error = node.error or str(error)
        args.output_dir.mkdir(parents=True, exist_ok=True)
        node_timing_fields = sorted(set(TIMING_METRIC_FIELDS).union(*(values.keys() for values in node.timing.values())))
        for row in node.rows:
            values = node.timing.get(row["source_stamp_ns"], {}) if args.node_timing_policy == "required" else {}
            for key in node_timing_fields:
                row[key] = values.get(key)
        with (args.output_dir / "timing.csv").open("w", newline="") as output:
            writer = csv.DictWriter(output, fieldnames=["source_stamp_ns", "output_receive_sim_ns", "source_to_output_sim_s", "input_receive_to_output_sim_s"] + node_timing_fields)
            writer.writeheader()
            writer.writerows(node.rows)
        with (args.output_dir / "reference_timing.csv").open("w", newline="") as output:
            writer = csv.DictWriter(output, fieldnames=["reference_header_stamp_ns", "reference_receive_sim_ns", "reference_header_to_output_sim_s"])
            writer.writeheader()
            writer.writerows(node.reference_rows)
        if hasattr(node, "profile"):
            np.savez_compressed(args.output_dir / "sample.npz", **node.profile)
        raw_summary = {"enabled": args.save_inputs}
        if node.raw_capture is not None:
            raw_summary.update(node.raw_capture.save(args.output_dir / "input_samples"))
            raw_summary["missing_sample_count"] = node.raw_missing_count
            raw_summary["static_mount_children_received"] = sorted(node.mount_store.transforms)
            raw_summary["usable_replay_count"] = sum(item.get("raw_capture", {}).get("saved", False) and item["raw_capture"]["usable_replay"] for item in node.samples)
        diagnostics_summary = {"first": node.diagnostics_first, "last": node.diagnostics_last}
        diagnostics_summary["measurement"] = "Counter differences cover the first/last captured diagnostic snapshots, not necessarily the full capture window"
        diagnostics_summary["missing"] = node.diagnostics_first is None or node.diagnostics_last is None
        if node.diagnostics_first is not None and node.diagnostics_last is not None:
            diagnostics_summary["counter_delta"] = diagnostic_delta(node.diagnostics_first["values"], node.diagnostics_last["values"])
            diagnostics_summary["observed_window_sim_s"] = (node.diagnostics_last["receive_sim_ns"] - node.diagnostics_first["receive_sim_ns"]) / 1e9
            diagnostics_summary["first_diag_sim_ns"] = node.diagnostics_first["header_stamp_ns"]
            diagnostics_summary["last_diag_sim_ns"] = node.diagnostics_last["header_stamp_ns"]
            diagnostics_summary["covered_duration_sim_s"] = (node.diagnostics_last["header_stamp_ns"] - node.diagnostics_first["header_stamp_ns"]) / 1e9
            diagnostics_summary["shorter_than_capture_window"] = diagnostics_summary["covered_duration_sim_s"] < args.duration_sim_s
            diagnostics_summary["config_revision_changed"] = node.diagnostics_first["values"].get("active_config_revision") != node.diagnostics_last["values"].get("active_config_revision")
            raw_summary["configuration_remained_fixed"] = not diagnostics_summary["config_revision_changed"]
            if diagnostics_summary["config_revision_changed"]:
                raw_summary["usable_replay_count"] = 0
                raw_summary["invalid_reason"] = "configuration revision changed during captured diagnostic interval"
        summary = {"success": node.error is None, "failure": node.error, "arguments": arguments_metadata(args),
                   "capture_complete": status["complete"], "capture_interrupted": status["interrupted"],
                   "clock": "ROS use_sim_time", "start_sim_ns": node.start_ns,
                   "node_timing_metadata": {
                       "policy": args.node_timing_policy,
                       "phase_metrics_available": args.node_timing_policy == "required",
                       "missing_phase_values_intentional": args.node_timing_policy == "observer-only",
                       "measurement": "Observer source-to-output age remains measured; node processing phases require matching diagnostics"},
                   "end_sim_ns": node.last_clock_ns, "camera_pairs_validated": node.camera_validated,
                   "pose_max_translation_m": translation, "pose_max_angle_deg": angle,
                   "timing_count": len(node.rows), "quality_samples": node.samples,
                   "input_stream_stamp_counts": node.stream_counts,
                   "raw_capture": raw_summary, "node_diagnostics_window": diagnostics_summary,
                   "reference_timing_count": len(node.reference_rows),
                   "reference_header_to_output_sim_s": timing_percentiles([row["reference_header_to_output_sim_s"] for row in node.reference_rows]),
                   "reference_timing_metadata": {
                       "clock": "ROS use_sim_time; same observer and capture period as stereo",
                       "header_stamp_meaning": "Body TF pose stamp used by the virtual camera, not a camera image acquisition stamp",
                       "measurement": "Observer receive simulation time minus virtual depth header stamp; includes delivery to this collector"},
                   "output_hz_sim": len(node.rows) / ((node.last_clock_ns - node.start_ns) / 1e9) if node.start_ns is not None and node.last_clock_ns > node.start_ns else None,
                   "quality_queue_dropped": node.quality_queue_dropped,
                   "invalid_node_timing_count": node.timing_invalid_count,
                   "unmatched_node_timing_count": sum(row["source_to_publish_sim_s"] is None for row in node.rows),
                   "unmatched_quality_count": len(node.pending),
                   "source_to_output_sim_s": timing_percentiles([row["source_to_output_sim_s"] for row in node.rows]),
                   "input_receive_to_output_sim_s": timing_percentiles([row["input_receive_to_output_sim_s"] for row in node.rows if row["input_receive_to_output_sim_s"] is not None])}
        for key in TIMING_METRIC_FIELDS:
            summary[key] = timing_percentiles([row[key] for row in node.rows if row[key] is not None])
        (args.output_dir / "summary.json").write_text(json.dumps(finite_json(summary), indent=2, allow_nan=False) + "\n")
        print(json.dumps({"success": node.error is None, "failure": node.error, "timing_count": len(node.rows), "quality_count": len(node.samples)}))
        return 0 if node.error is None else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
