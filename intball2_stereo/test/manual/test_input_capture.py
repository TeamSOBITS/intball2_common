from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import json
import unittest

import numpy as np

from input_capture import (BoundedInputCapture, StaticMountStore, camera_info_metadata, diagnostic_delta,
                           capture_event_sim_time, image_metadata, transform_matrix, validate_input_pair)


def messages():
    header = SimpleNamespace(stamp=SimpleNamespace(sec=9000000, nanosec=3), frame_id="left_link")
    image = SimpleNamespace(header=header, width=2, height=2, step=8, encoding="rgb8",
                            is_bigendian=1, data=bytearray(range(16)))
    info = SimpleNamespace(header=header, width=2, height=2, distortion_model="plumb_bob",
                           d=[.1, 0, 0, 0, 0], k=[3., 0, 1, 0, 3, 1, 0, 0, 1],
                           r=[1., 0, 0, 0, 1, 0, 0, 0, 1], p=[3., 0, 1, -0.3, 0, 3, 1, 0, 0, 0, 1, 0],
                           binning_x=0, binning_y=0,
                           roi=SimpleNamespace(x_offset=0, y_offset=0, width=0, height=0, do_rectify=False))
    return image, info


class InputCaptureTests(unittest.TestCase):
    def test_static_mounts_arriving_before_clock_are_retained_and_restored(self):
        store = StaticMountStore(("body", "iss_body"))
        buffer = SimpleNamespace(values={}, set_transform_static=lambda transform, authority:
                                 buffer.values.update({transform.child_frame_id: transform}))
        transforms = [SimpleNamespace(child_frame_id=frame) for frame in
                      ("body", "cameraL_link", "cameraL_optical_frame", "cameraR_optical_frame")]
        # Intake requires neither a clock nor its first nonzero value.
        message = SimpleNamespace(transforms=transforms)
        self.assertIsNone(capture_event_sim_time("mount", message, lambda: 0, store, buffer))
        self.assertEqual(capture_event_sim_time("camera", message, lambda: 0, store, buffer), 0)
        self.assertNotIn("body", buffer.values)
        self.assertIn("cameraL_optical_frame", buffer.values)
        self.assertEqual(capture_event_sim_time("camera", message, lambda: 1000000000, store, buffer), 1000000000)
        buffer.values.clear()
        store.install(buffer)
        self.assertEqual(set(buffer.values), {"cameraL_link", "cameraL_optical_frame", "cameraR_optical_frame"})

    def test_archive_preserves_wire_format_calibration_and_ownership(self):
        image, info = messages()
        metadata = {"image": image_metadata(image), "camera_info": camera_info_metadata(info)}
        capture = BoundedInputCapture(2, 100000)
        self.assertEqual(capture.add({"left_bytes": np.frombuffer(image.data, dtype=np.uint8)}, metadata), 0)
        image.data[0] = 99
        info.p[3] = 42
        with TemporaryDirectory() as directory:
            result = capture.save(Path(directory) / "inputs")
            self.assertEqual(result["written_bytes"], result["reserved_bytes"])
            with np.load(Path(directory) / "inputs/sample_0000.npz", allow_pickle=False) as saved:
                self.assertEqual(saved["left_bytes"][0], 0)
                restored = json.loads(saved["metadata_json_utf8"].tobytes())
                self.assertEqual(restored["image"]["step"], 8)
                self.assertEqual(restored["image"]["encoding"], "rgb8")
                self.assertEqual(restored["image"]["is_bigendian"], 1)
                self.assertEqual(restored["image"]["stamp_ns"], 9000000000000003)
                self.assertEqual(restored["camera_info"]["p"][3], -0.3)
                self.assertEqual(restored["camera_info"]["d"][0], .1)

    def test_capacity_is_checked_before_owned_copy_and_matches_disk_size(self):
        arrays = {"pixels": np.arange(32, dtype=np.uint8)}
        probe = BoundedInputCapture(1, 100000)
        probe.add(arrays, {"stamp": 3})
        exact = BoundedInputCapture(2, probe.reserved_bytes)
        self.assertEqual(exact.add(arrays, {"stamp": 3}), 0)
        self.assertIsNone(exact.add(arrays, {"stamp": 4}))
        self.assertEqual(exact.limit_rejections, 1)
        too_small = BoundedInputCapture(2, probe.reserved_bytes - 1)
        self.assertIsNone(too_small.add(arrays, {"stamp": 3}))
        self.assertEqual(too_small.array_bytes, 0)
        with TemporaryDirectory() as directory:
            self.assertEqual(exact.save(Path(directory) / "inputs")["written_bytes"], probe.reserved_bytes)

    def test_sample_limit_and_unsafe_arrays_are_rejected(self):
        capture = BoundedInputCapture(1, 100000)
        with self.assertRaises(ValueError):
            capture.add({"../pixels": np.zeros(1)}, {})
        with self.assertRaises(ValueError):
            capture.add({"pixels": np.array([object()])}, {})
        capture.add({"pixels": np.zeros(1)}, {})
        self.assertIsNone(capture.add({"pixels": np.zeros(1)}, {}))
        with self.assertRaises(ValueError):
            BoundedInputCapture(1, 2**31)

    def test_input_pair_allows_distinct_source_stamps_and_validates_rgb_stride(self):
        image, info = messages()
        info.header = SimpleNamespace(stamp=SimpleNamespace(sec=9000000, nanosec=7), frame_id="left_link")
        validate_input_pair(image, info)
        image.step = 5
        with self.assertRaises(ValueError):
            validate_input_pair(image, info)
        image.step = 8
        image.encoding = "16UC1"
        with self.assertRaises(ValueError):
            validate_input_pair(image, info)

    def test_body_pose_and_camera_mount_compose_in_correct_direction(self):
        body = transform_matrix([1, 2, 3], [0, 0, np.sqrt(.5), np.sqrt(.5)])
        mount = transform_matrix([.025, .14, 0], [0, 0, 0, 1])
        np.testing.assert_allclose((body @ mount)[:3, 3], [.86, 2.025, 3])
        np.testing.assert_allclose(body[:3, :3] @ [1, 0, 0], [0, 1, 0], atol=1e-15)
        with self.assertRaises(ValueError):
            transform_matrix([0, 0, 0], [0, 0, 0, 0])

    def test_diagnostic_counter_differences_ignore_gauges_and_detect_reset(self):
        self.assertEqual(diagnostic_delta({"processed_pairs": "10", "active_config_revision": "1"},
                                          {"processed_pairs": "17", "active_config_revision": "2"}),
                         {"processed_pairs": 7})
        self.assertEqual(diagnostic_delta({}, {}), {})
        with self.assertRaises(ValueError):
            diagnostic_delta({"failed_pairs": "4"}, {"failed_pairs": "0"})


if __name__ == "__main__":
    unittest.main()
