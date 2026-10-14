"""Collector helpers: no ROS graph or runtime imports."""

from collections import OrderedDict
import json
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np

from compare_sim_depth import arguments, arguments_metadata, cache_put, capture_status, finite_json, image_info_match, pose_motion, reference_timing_row, stamp_ns, timing_values, validate_capture_evidence


class CollectorHelpers(unittest.TestCase):
    def test_external_observer_policy_only_relaxes_node_phase_requirement(self):
        rows, samples = [{"source_stamp_ns": 100}], [{"stamp_ns": 100}]
        self.assertEqual(arguments(["--output-dir", "/tmp/collector-policy-test"]).node_timing_policy, "required")
        with self.assertRaisesRegex(ValueError, "missing simulation timing"):
            validate_capture_evidence(rows, samples, 1, {})
        with self.assertRaisesRegex(ValueError, "do not match"):
            validate_capture_evidence(rows, samples, 1, {200: {}})
        validate_capture_evidence(rows, samples, 1, {}, "observer-only")
        for incomplete in [([], samples, 1), (rows, [], 1), (rows, samples, 0)]:
            with self.assertRaisesRegex(ValueError, "aligned quality"):
                validate_capture_evidence(*incomplete, {}, "observer-only")
        with self.assertRaises(SystemExit):
            arguments(["--output-dir", "/tmp/collector-policy-test", "--node-timing-policy", "observer-only", "--save-inputs"])

    def test_interruption_never_promotes_saved_partial_data_to_completed_capture(self):
        status = capture_status(100, 150, 100, interrupted=True)
        self.assertFalse(status['complete'])
        self.assertTrue(status['interrupted'])
        self.assertIsNotNone(status['failure'])
        self.assertFalse(capture_status(100, 250, 100, interrupted=True)['complete'])

    def test_completion_requires_simulation_duration_and_preserves_original_failure(self):
        self.assertTrue(capture_status(100, 200, 100)['complete'])
        self.assertIsNone(capture_status(100, 200, 100)['failure'])
        self.assertFalse(capture_status(None, 0, 100)['complete'])
        status = capture_status(100, 150, 100, failure='clock rewound')
        self.assertEqual(status['failure'], 'clock rewound')
        self.assertFalse(status['complete'])

    def test_timing_integer_nanosecond_stamps_survive_above_float_precision(self):
        values = timing_values([SimpleNamespace(key="right_image_stamp_ns", value="9007199254740993"),
                                SimpleNamespace(key="bundle_to_worker_start_sim_s", value="0.004")])
        self.assertEqual(values["right_image_stamp_ns"], 9007199254740993)
        self.assertIsInstance(values["right_image_stamp_ns"], int)
        self.assertAlmostEqual(values["bundle_to_worker_start_sim_s"], .004)

    def test_reference_timing_uses_header_and_observer_simulation_stamps(self):
        row = reference_timing_row(12000000000, 12025000000)
        self.assertEqual(row["reference_header_stamp_ns"], 12000000000)
        self.assertEqual(row["reference_receive_sim_ns"], 12025000000)
        self.assertAlmostEqual(row["reference_header_to_output_sim_s"], 0.025)
        self.assertAlmostEqual(reference_timing_row(12025000000, 12000000000)
                               ["reference_header_to_output_sim_s"], -0.025)

    def test_default_reference_topics_match_existing_stereo_virtual_camera(self):
        args = arguments(["--output-dir", "/tmp/depth-comparison"])
        self.assertEqual(args.reference_depth_topic, "/virtual_camera/stereo/depth")
        self.assertEqual(args.reference_info_topic, "/virtual_camera/stereo/camera_info")

    def test_argument_serialization_preserves_output_path_for_summary_save(self):
        args = SimpleNamespace(output_dir=Path("/tmp/depth-comparison"), duration_sim_s=20.0)
        metadata = arguments_metadata(args)
        self.assertEqual(metadata["output_dir"], "/tmp/depth-comparison")
        self.assertEqual(args.output_dir / "summary.json", Path("/tmp/depth-comparison/summary.json"))
        self.assertEqual(json.loads(json.dumps(metadata))["duration_sim_s"], 20.0)
        metadata["duration_sim_s"] = 1.0
        self.assertEqual(args.duration_sim_s, 20.0)

    def test_bounded_cache_preserves_latest_and_replace(self):
        values = OrderedDict()
        for number in range(5):
            cache_put(values, number, number * 2, 2)
        self.assertEqual(list(values), [3, 4])
        cache_put(values, 4, 99, 2)
        self.assertEqual(values[4], 99)

    def test_pose_motion_uses_midpoint_and_quaternion_sign_equivalence(self):
        poses = [(0, np.array([0.003, 0, 0]), np.array([0, 0, 0, -1.])),
                 (10, np.zeros(3), np.array([0, 0, 0, 1.])),
                 (20, np.array([-0.004, 0, 0]), np.array([0, 0, 0, 1.]))]
        distance, angle = pose_motion(poses, 10)
        self.assertAlmostEqual(distance, 0.004)
        self.assertAlmostEqual(angle, 0)
        with self.assertRaises(ValueError):
            pose_motion(poses[:1], 0)
        with self.assertRaises(ValueError):
            pose_motion([(0, np.zeros(3), np.zeros(4)), poses[1]], 10)

    def test_pose_motion_detects_orientation(self):
        rotation = np.array([0, 0, np.sin(np.radians(0.15)), np.cos(np.radians(0.15))])
        poses = [(0, np.zeros(3), np.array([0, 0, 0, 1.])), (10, np.zeros(3), rotation)]
        distance, angle = pose_motion(poses, 0)
        self.assertEqual(distance, 0)
        self.assertAlmostEqual(angle, 0.3)

    def test_image_info_requires_same_header_dimensions_and_intrinsics(self):
        header = SimpleNamespace(stamp=SimpleNamespace(sec=2, nanosec=7), frame_id="optical")
        image = SimpleNamespace(header=header, width=10, height=8)
        info = SimpleNamespace(header=header, width=10, height=8, k=[3, 0, 5, 0, 3, 4, 0, 0, 1])
        self.assertEqual(stamp_ns(header.stamp), 2000000007)
        self.assertEqual(image_info_match(image, info).shape, (3, 3))
        info.width = 9
        with self.assertRaises(ValueError):
            image_info_match(image, info)
        info.width = 10
        info.k[0] = np.nan
        with self.assertRaises(ValueError):
            image_info_match(image, info)

    def test_strict_json_replaces_nonfinite_values(self):
        data = finite_json({"values": np.array([np.nan, np.inf, -np.inf, 0.1]), "count": np.int64(2)})
        self.assertEqual(data["values"][:3], [None, None, None])
        self.assertEqual(json.loads(json.dumps(data, allow_nan=False))["count"], 2)


if __name__ == "__main__":
    unittest.main()
