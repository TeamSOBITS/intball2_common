"""Offline experiment contracts: missingness and synthetic signed geometry."""
import unittest

import numpy as np

from compare_replay_precision import common_metrics, conditions, synthetic_pair
from replay_capture import CONFIG_KEYS


class PrecisionComparisonTest(unittest.TestCase):
    def test_common_valid_does_not_compare_different_pixel_populations(self):
        reference = np.array([[1., 1., 1., np.inf]], dtype=np.float32)
        baseline = np.array([[1.1, np.nan, 2., 1.]], dtype=np.float32)
        candidate = np.array([[1., 1., np.nan, 1.]], dtype=np.float32)
        metrics = common_metrics(baseline, candidate, reference)
        self.assertEqual(metrics['baseline']['reference_count'], 1)
        self.assertEqual(metrics['candidate']['reference_count'], 1)
        self.assertAlmostEqual(metrics['baseline']['rmse_m'], .1, places=6)
        self.assertEqual(metrics['candidate']['rmse_m'], 0.)

    def test_conditions_preserve_all_core_keys_and_scale_disparity_search(self):
        base = dict.fromkeys(CONFIG_KEYS, 0)
        base.update({'processing.downsample': 4, 'matcher.min_disparity_px': -25,
                     'matcher.num_disparities_px': 32, 'matcher.block_size_px': 5,
                     'matcher.uniqueness_ratio': 10})
        result = conditions(base)
        for config in result.values():
            self.assertEqual(set(config), set(CONFIG_KEYS))
        self.assertEqual(result['d2_b5']['matcher.min_disparity_px'], -50)
        self.assertEqual(result['d2_b5']['matcher.num_disparities_px'], 64)
        self.assertEqual(base['processing.downsample'], 4)

    def test_synthetic_negative_disparity_corresponds_to_right_shift(self):
        metadata = {'camera_info': {'camera_info': {'height': 40, 'width': 64,
                                                    'k': [100, 0, 32, 0, 100, 20, 0, 0, 1]}},
                    'images': {'camera': {}, 'right': {}}}
        arrays, changed, disparity = synthetic_pair(metadata, 1.)
        self.assertEqual(disparity, -5.)
        left = arrays['camera_image_bytes'].reshape(40, 64, 3)
        right = arrays['right_image_bytes'].reshape(40, 64, 3)
        np.testing.assert_array_equal(right[:, 5:], left[:, :-5])
        self.assertEqual(changed['images']['camera']['step'], 192)
        self.assertEqual(metadata['images']['camera'], {})


if __name__ == '__main__':
    unittest.main()
