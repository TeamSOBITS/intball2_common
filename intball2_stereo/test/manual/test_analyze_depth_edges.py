import unittest

import numpy as np

from analyze_depth_edges import analyze, crop_mask, edge_masks


class DepthEdgesTests(unittest.TestCase):
    def test_directional_bins_partition_each_side_and_keep_corners(self):
        masks = edge_masks((200, 200))
        for side in ('left', 'right', 'top', 'bottom'):
            total = sum(mask.astype(int) for name, mask in masks.items() if name.startswith(side + ':'))
            np.testing.assert_array_equal(total, np.ones((200, 200)))
        self.assertTrue(masks['left:0-5'][0, 0])
        self.assertTrue(masks['top:0-5'][0, 0])
        self.assertFalse(masks['right:0-5'][0, 0])

    def test_common_mask_missingness_and_coarse_retention(self):
        reference = np.ones((50, 50))
        coarse = reference + .2
        refined = reference.copy()
        refined[20, 20] = coarse[20, 20]
        refined[21, 21] = np.nan
        result = analyze(reference, {'captured': coarse, 'newgray': coarse, 'refined': refined})
        self.assertEqual(result['full']['common_valid_count'], 2499)
        self.assertEqual(result['full']['variants']['refined']['reference_only_count'], 1)
        self.assertEqual(result['full']['updated_common']['refined']['count'], 2498)
        self.assertEqual(result['full']['retained_coarse_common']['count'], 1)
        self.assertFalse(result['refined_finite_mask_matches_newgray'])

    def test_crop_reports_lost_bad_pixels_and_preserves_zero_margin(self):
        reference = np.ones((50, 50))
        depth = reference.copy()
        depth[:, :5] += 1
        result = analyze(reference, {key: depth for key in ('captured', 'newgray', 'refined')})
        self.assertEqual(result['symmetric_crops']['0']['lost']['grid_pixel_count'], 0)
        crop = result['symmetric_crops']['5']
        self.assertEqual(crop['retained']['common_valid_count'], 1600)
        self.assertEqual(crop['lost']['common_valid_count'], 900)
        self.assertEqual(crop['retained']['variants']['refined']['common_valid_error']['rmse_m'], 0)
        self.assertGreater(crop['lost']['variants']['refined']['common_valid_error']['rmse_m'], 0)
        self.assertEqual(crop_mask((10, 10), 20).sum(), 0)
        with self.assertRaises(ValueError):
            crop_mask((10, 10), -1)


if __name__ == '__main__':
    unittest.main()
