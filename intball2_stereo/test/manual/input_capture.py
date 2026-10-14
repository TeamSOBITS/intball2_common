"""ROS-free, bounded replay input archives with original wire-format pixels."""

from io import BytesIO
import json
from pathlib import Path
import zipfile

import numpy as np


class StaticMountStore:
    """Retain mounting transforms independently of simulation-clock startup."""

    def __init__(self, blocked_children):
        self.blocked_children = set(blocked_children)
        self.transforms = {}

    def receive(self, transforms):
        for transform in transforms:
            child = transform.child_frame_id
            if child not in self.blocked_children:
                if child not in self.transforms and len(self.transforms) >= 256:
                    raise ValueError("static transform capacity exceeded")
                self.transforms[child] = transform

    def install(self, buffer):
        for transform in self.transforms.values():
            buffer.set_transform_static(transform, "depth_comparison_camera_mount")


def capture_event_sim_time(stream, message, clock_reader, mount_store, mount_buffer):
    if stream == "mount":
        mount_store.receive(message.transforms)
        mount_store.install(mount_buffer)
        return None
    return clock_reader()


def header_metadata(message):
    return {"stamp_ns": int(message.header.stamp.sec) * 1000000000 + int(message.header.stamp.nanosec),
            "frame_id": message.header.frame_id}


def image_metadata(message):
    return {**header_metadata(message), "height": int(message.height), "width": int(message.width),
            "encoding": message.encoding, "is_bigendian": int(message.is_bigendian),
            "step": int(message.step), "data_length": len(message.data)}


def camera_info_metadata(message):
    return {**header_metadata(message), "height": int(message.height), "width": int(message.width),
            "distortion_model": message.distortion_model, "d": list(message.d),
            "k": list(message.k), "r": list(message.r), "p": list(message.p),
            "binning_x": int(message.binning_x), "binning_y": int(message.binning_y),
            "roi": {key: getattr(message.roi, key) for key in
                    ("x_offset", "y_offset", "width", "height", "do_rectify")}}


def validate_input_pair(image, info):
    # Approximate synchronization permits distinct image/calibration stamps.
    if (image.header.frame_id != info.header.frame_id
            or image.width != info.width or image.height != info.height):
        raise ValueError("input image/calibration frame or dimensions differ")
    channels = {"mono8": 1, "bgr8": 3, "rgb8": 3}.get(image.encoding)
    if channels is None or image.width <= 0 or image.height <= 0:
        raise ValueError("unsupported input encoding or invalid dimensions")
    if image.step < image.width * channels or len(image.data) < image.step * image.height:
        raise ValueError("input wire-format buffer is shorter than step * height")


def transform_matrix(translation, quaternion):
    translation = np.asarray(translation, dtype=np.float64)
    quaternion = np.asarray(quaternion, dtype=np.float64)
    norm = np.linalg.norm(quaternion)
    if translation.shape != (3,) or quaternion.shape != (4,) or not np.all(np.isfinite(translation)) or not np.isfinite(norm) or norm <= 0:
        raise ValueError("invalid transform")
    x, y, z, w = quaternion / norm
    matrix = np.eye(4)
    matrix[:3, :3] = [[1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
                     [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
                     [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)]]
    matrix[:3, 3] = translation
    return matrix


def _array_archive_size(name, array):
    header = BytesIO()
    np.lib.format.write_array_header_1_0(header, {
        "descr": np.lib.format.dtype_to_descr(array.dtype),
        "fortran_order": False, "shape": array.shape})
    member_name = (name + ".npy").encode("utf-8")
    # ZIP_STORED: exact local header + payload + central-directory entry.
    return 30 + len(member_name) + header.tell() + array.nbytes + 46 + len(member_name)


class BoundedInputCapture:
    def __init__(self, max_samples, max_bytes):
        if max_samples <= 0 or not 0 < max_bytes < zipfile.ZIP64_LIMIT:
            raise ValueError("capture limits must be positive and bytes below ZIP64 limit")
        self.max_samples = max_samples
        self.max_bytes = max_bytes
        self.samples = []
        self.reserved_bytes = 0
        self.array_bytes = 0
        self.limit_rejections = 0

    def add(self, arrays, metadata):
        if len(self.samples) >= self.max_samples:
            self.limit_rejections += 1
            return None
        metadata_bytes = json.dumps(metadata, allow_nan=False, separators=(",", ":")).encode("utf-8")
        views = {name: np.asarray(array) for name, array in arrays.items()}
        views["metadata_json_utf8"] = np.frombuffer(metadata_bytes, dtype=np.uint8)
        for name, array in views.items():
            if not name.isidentifier() or array.dtype.hasobject:
                raise ValueError("capture arrays require safe names and non-object dtypes")
        reserved = 22 + sum(_array_archive_size(name, array) for name, array in views.items())
        if self.reserved_bytes + reserved > self.max_bytes:
            self.limit_rejections += 1
            return None
        owned = {name: np.array(array, copy=True, order="C") for name, array in views.items()}
        self.samples.append(owned)
        self.reserved_bytes += reserved
        self.array_bytes += sum(array.nbytes for array in owned.values())
        return len(self.samples) - 1

    def save(self, directory):
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=False)
        written = 0
        for index, arrays in enumerate(self.samples):
            target = directory / f"sample_{index:04d}.npz"
            with zipfile.ZipFile(target, mode="x", compression=zipfile.ZIP_STORED) as archive:
                for name, array in arrays.items():
                    with archive.open(name + ".npy", "w") as member:
                        np.lib.format.write_array(member, array, allow_pickle=False)
            written += target.stat().st_size
            if written > self.max_bytes or written > self.reserved_bytes:
                raise RuntimeError("archive exceeded precomputed capacity")
        return {"saved_count": len(self.samples), "reserved_bytes": self.reserved_bytes,
                "owned_array_bytes": self.array_bytes, "written_bytes": written,
                "limit_rejections": self.limit_rejections,
                "storage": "uncompressed NPZ; disk I/O occurs after capture window"}


def diagnostic_delta(first, last):
    counters = ("sync_matched_bundles", "sync_late_or_duplicate_messages", "sync_queue_overflow_messages",
                "sync_interval_dropped_messages", "worker_dropped_pairs", "processed_pairs", "failed_pairs")
    result = {}
    for key in counters:
        if key in first and key in last:
            difference = int(last[key]) - int(first[key])
            if difference < 0:
                raise ValueError("diagnostic counter rewound: " + key)
            result[key] = difference
    return result
