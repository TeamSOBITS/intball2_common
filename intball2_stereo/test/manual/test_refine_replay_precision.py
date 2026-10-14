"""Known signed shifts and refusal of unobservable depth refinement."""
import unittest

import numpy as np

from refine_replay_precision import refine, refinement_inputs


class RefinementTest(unittest.TestCase):
    def test_negative_integer_shift_recovers_known_depth_without_filling_holes(self):
        left = np.random.default_rng(83).uniform(0, 255, (60, 80))
        right = np.zeros_like(left)
        right[:, 5:] = left[:, :-5]
        k = [100, 0, 40, 0, 100, 30, 0, 0, 1]
        predicted = np.full(left.shape, 1.2, dtype=np.float32)
        predicted[25:30, 30:35] = np.nan
        result, flags = refine(left, right, predicted, k, k, window=7)
        central = result[15:45, 15:60]
        self.assertGreater(np.count_nonzero(np.isfinite(central)), 1000)
        self.assertAlmostEqual(float(np.nanmedian(central)), 1., places=2)
        self.assertTrue(np.isnan(result[25:30, 30:35]).all())
        self.assertLessEqual(flags['accepted_count'], flags['input_valid_count'])

    def test_area_reduction_preserves_pixel_center_ray_calibration(self):
        pixels = np.arange(64, dtype=float).reshape(8, 8)
        k = [100, 0, 4.5, 0, 100, 4.5, 0, 0, 1]
        left, right, reduced_k = refinement_inputs(pixels, pixels, k, 2)
        self.assertEqual(left.shape, (4, 4))
        self.assertEqual(left[0, 0], 4.5)
        np.testing.assert_array_equal(left, right)
        self.assertEqual(reduced_k[0], 50)
        self.assertEqual(reduced_k[2], 2.)
        self.assertEqual(k[2], 4.5)

    def test_flat_images_and_all_invalid_predictions_remain_unknown(self):
        pixels = np.full((40, 60), 128., dtype=np.float64)
        k = [100, 0, 30, 0, 100, 20, 0, 0, 1]
        for depth in [np.full(pixels.shape, 1., dtype=np.float32),
                      np.full(pixels.shape, np.nan, dtype=np.float32)]:
            result, flags = refine(pixels, pixels, depth, k, k)
            self.assertTrue(np.isnan(result).all())
            self.assertEqual(flags['accepted_count'], 0)


if __name__ == '__main__':
    unittest.main()
