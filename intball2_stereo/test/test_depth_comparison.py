"""Numerical tests for depth comparison without importing ROS or opening graph."""

import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np

SPEC = importlib.util.spec_from_file_location(
    "depth_comparison", Path(__file__).parent / "manual" / "depth_comparison.py"
)
comparison = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(comparison)


def camera_k(fx=2, fy=2, cx=1, cy=1):
    return [fx, 0, cx, 0, fy, cy, 0, 0, 1]


class DepthDecodeTests(unittest.TestCase):
    def test_padded_rows_endian_nan_and_owned_storage(self):
        for bigendian in [False, True]:
            data = bytearray(24)
            source = np.ndarray(
                (2, 2), dtype=">f4" if bigendian else "<f4", buffer=data, strides=(12, 4)
            )
            source[:] = [[1.25, np.nan], [np.inf, 3]]
            message = SimpleNamespace(
                encoding="32FC1", width=2, height=2, step=12,
                is_bigendian=bigendian, data=data
            )
            decoded = comparison.decode_depth(message)
            source[:] = 9
            np.testing.assert_equal(decoded, [[1.25, np.nan], [np.inf, 3]])
            self.assertTrue(decoded.flags.owndata)
            self.assertEqual(decoded.dtype, np.dtype(np.float32))

    def test_reject_malformed_messages(self):
        message = SimpleNamespace(
            encoding="32FC1", width=2, height=2, step=8,
            is_bigendian=False, data=bytes(16)
        )
        for name, value in [("encoding", "16UC1"), ("width", 0), ("step", 7),
                            ("data", bytes(15))]:
            bad = SimpleNamespace(**vars(message))
            setattr(bad, name, value)
            with self.assertRaises(ValueError):
                comparison.decode_depth(bad)


class RayResamplingTests(unittest.TestCase):
    def test_resized_pixel_centres_and_original_virtual_k(self):
        reference = np.arange(64, dtype=np.float32).reshape(8, 8)
        # Left input has cx=3.5, virtual camera has cx=4.0. factor=2
        # stereo cx=(3.5+.5)/2-.5=1.5. The virtual rays map to [1,3,5,7].
        actual = comparison.resample_reference(
            reference, camera_k(8, 8, 4, 4), camera_k(4, 4, 1.5, 1.5), (4, 4)
        )
        np.testing.assert_equal(actual, reference[np.ix_([1, 3, 5, 7], [1, 3, 5, 7])])
        # Merely slicing at [0,2,4,6] would compare different rays.
        self.assertNotEqual(actual[0, 0], reference[0, 0])

    def test_identity_preserves_categories_and_out_of_fov_is_nan(self):
        reference = np.array([[np.nan, np.inf], [-np.inf, 2]], dtype=np.float32)
        actual = comparison.resample_reference(reference, camera_k(), camera_k(), (2, 2))
        np.testing.assert_equal(actual, reference)
        shifted = comparison.resample_reference(
            reference, camera_k(), camera_k(cx=3), (2, 4)
        )
        self.assertTrue(np.all(np.isnan(shifted[:, :2])))
        np.testing.assert_equal(shifted[:, 2:], reference)

    def test_reject_nonpositive_or_skew_intrinsics(self):
        for k in [camera_k(fx=0), camera_k(cy=np.nan), [2, 1, 1, 0, 2, 1, 0, 0, 1]]:
            with self.assertRaises(ValueError):
                comparison.resample_reference(np.ones((2, 2)), k, camera_k(), (2, 2))


class DepthMetricsTests(unittest.TestCase):
    def test_error_sign_percentiles_rmse_and_rates(self):
        reference = np.array([[1, 1, 1, 1, 1, 1]], dtype=np.float64)
        stereo = np.array([[0.8, 1, 1.2, 1.4, np.nan, np.inf]], dtype=np.float64)
        actual = comparison.depth_metrics(stereo, reference)
        self.assertEqual(actual["reference_count"], 6)
        self.assertEqual(actual["stereo_valid_count"], 4)
        self.assertAlmostEqual(actual["stereo_valid_fraction"], 4/6)
        self.assertAlmostEqual(actual["signed_mean_m"], 0.1)
        self.assertAlmostEqual(actual["signed_median_m"], 0.1)
        self.assertAlmostEqual(actual["absolute_p90_m"], 0.34)
        self.assertAlmostEqual(actual["absolute_p95_m"], 0.37)
        self.assertAlmostEqual(actual["rmse_m"], np.sqrt(0.06))
        self.assertEqual(actual["too_near_fraction"], 0.25)
        self.assertEqual(actual["too_far_fraction"], 0.5)

    def test_bands_partition_range_and_other_categories_count_valid_stereo(self):
        reference = np.array([[.25, .5, 1, 2, 3, np.nan, np.inf, -np.inf, 0, .2, 3.1]])
        stereo = np.ones_like(reference)
        actual = comparison.depth_metrics(stereo, reference)
        self.assertEqual(actual["reference_count"], 5)
        self.assertEqual(
            [v["reference_count"] for v in actual["bands"].values()], [1, 1, 1, 2]
        )
        self.assertEqual(actual["stereo_valid_on_nonfinite_reference_count"], 3)
        for category in actual["reference_other_categories"].values():
            self.assertEqual(category["reference_count"], 1)
            self.assertEqual(category["stereo_valid_count"], 1)
        json.dumps(actual, allow_nan=False)

    def test_empty_population_uses_json_null_and_shape_errors_are_rejected(self):
        actual = comparison.depth_metrics(np.array([[np.nan]]), np.array([[np.inf]]))
        self.assertIsNone(actual["stereo_valid_fraction"])
        self.assertIsNone(actual["rmse_m"])
        json.dumps(actual, allow_nan=False)
        with self.assertRaises(ValueError):
            comparison.depth_metrics(np.ones((1, 2)), np.ones((2, 1)))


class TimingTests(unittest.TestCase):
    def test_reject_negative_and_nonfinite_without_hiding_them(self):
        result = comparison.timing_percentiles([0, 1, 2, -1, np.nan, np.inf])
        self.assertEqual(result["sample_count"], 6)
        self.assertEqual(result["valid_count"], 3)
        self.assertEqual(result["excluded_negative_count"], 1)
        self.assertEqual(result["excluded_nonfinite_count"], 2)
        self.assertEqual(result["mean_s"], 1)
        self.assertEqual(result["p50_s"], 1)
        self.assertAlmostEqual(result["p95_s"], 1.9)
        self.assertIsNone(comparison.timing_percentiles([])["p95_s"])
        json.dumps(comparison.timing_percentiles([]), allow_nan=False)


if __name__ == "__main__":
    unittest.main()
