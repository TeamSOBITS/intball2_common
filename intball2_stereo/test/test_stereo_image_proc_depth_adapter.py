"""Numerical adapter regressions, without ROS graph or clock dependencies."""
import sys
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent / 'manual'))
from stereo_image_proc_depth_adapter import (ConversionError, SimulationClockTracker,
    assign_camera_calibration, convert_disparity, simulation_timing, valid_rectangle)


def fixture(values=None, endian=False, pad=0):
    values = np.full((8, 16), -5., np.float32) if values is None else np.asarray(values, np.float32)
    height, width = values.shape
    header = NS(stamp=NS(sec=12, nanosec=345), frame_id='cameraL_link')
    data = b''.join(row.astype('>f4' if endian else '<f4').tobytes() + bytes(pad) for row in values)
    image = NS(header=header, height=height, width=width, encoding='32FC1',
               step=width * 4 + pad, is_bigendian=endian, data=data)
    disparity = NS(header=header, image=image, f=100., t=-.05, min_disparity=-16., max_disparity=15.,
                   delta_d=.0625, valid_window=NS(x_offset=0, y_offset=0, width=width, height=height))
    p = [100., 0., 8., 0., 0., 100., 4., 0., 0., 0., 1., 0.]
    info = NS(header=header, width=width, height=height, p=p,
              k=[100., 0., 8., 0., 100., 4., 0., 0., 1.], r=np.eye(3).ravel().tolist(),
              d=[], distortion_model='plumb_bob')
    return disparity, info


class AdapterTests(unittest.TestCase):
    def test_signed_geometry_endian_stride_ownership(self):
        for endian in [False, True]:
            message, info = fixture(endian=endian, pad=12)
            original = message.image.data
            depth, camera, meta = convert_disparity(message, info)
            np.testing.assert_array_equal(depth, np.ones((8, 16), np.float32))
            self.assertEqual(camera['frame_id'], 'cameraL_optical_frame')
            self.assertEqual(camera['stamp_ns'], 12_000_000_345)
            self.assertEqual(camera['p'], info.p)
            self.assertEqual(message.image.data, original)
            self.assertTrue(meta['known_gazebo_optical_alias_applied'])
            message.t = .05
            depth, _, _ = convert_disparity(message, info)
            self.assertFalse(np.isfinite(depth).any())

    def test_invalid_disparity_and_roi(self):
        values = np.full((8, 16), -5., np.float32)
        values[0, :6] = [np.nan, np.inf, -np.inf, 0., -17., 5.]
        message, info = fixture(values)
        depth, _, _ = convert_disparity(message, info)
        self.assertTrue(np.isnan(depth[0, :6]).all())
        message.valid_window.width = 2 ** 32 - 1
        with self.assertRaisesRegex(ConversionError, 'valid_window'):
            convert_disparity(message, info)
        message.image.encoding = '16UC1'
        message.valid_window.width = 16
        with self.assertRaisesRegex(ConversionError, 'encoding'):
            convert_disparity(message, info)

    def test_exact_header_and_shape(self):
        message, info = fixture()
        info.header = NS(stamp=NS(sec=12, nanosec=346), frame_id='cameraL_link')
        with self.assertRaisesRegex(ConversionError, 'stamp_or_frame'):
            convert_disparity(message, info)
        message, info = fixture()
        info.width = 17
        with self.assertRaisesRegex(ConversionError, 'shape'):
            convert_disparity(message, info)

    def test_nearest_grid_and_projection(self):
        message, info = fixture()
        depth, camera, meta = convert_disparity(message, info, output_downsample=4)
        self.assertEqual(depth.shape, (2, 4))
        self.assertEqual(camera['k'][0], 25.)
        self.assertEqual(camera['k'][2], 1.625)
        self.assertEqual(meta['nearest_vs_ideal_source_pixel_offset'], .5)

    def test_computed_roi_signed_support_and_principal_adjustment(self):
        values = np.full((8, 64), -2., np.float32)
        message, info = fixture(values)
        message.min_disparity, message.max_disparity = -4., 11.
        message.valid_window.width = 2 ** 32 - 1
        roi, original_valid = valid_rectangle(message, 'compute-sgbm-valid-roi', (-4, 16, 3), 3.)
        self.assertEqual(roi, (12, 1, 47, 6))
        self.assertFalse(original_valid)
        # D is principal-adjusted; raw disparity +1 remains inside matcher search.
        depth, _, meta = convert_disparity(message, info, valid_window_policy='compute-sgbm-valid-roi',
                                           sgbm=(-4, 16, 3), principal_difference_px=3.)
        np.testing.assert_array_equal(depth[1:7, 12:59], np.full((6, 47), 2.5, np.float32))
        self.assertFalse(meta['original_valid_window_in_bounds'])
        message.min_disparity, message.max_disparity = 0., 15.
        depth, _, _ = convert_disparity(message, info, principal_difference_px=3.,
                                       valid_window_policy='compute-sgbm-valid-roi', sgbm=(0, 16, 3))
        self.assertTrue(np.isfinite(depth).any())
        with self.assertRaisesRegex(ConversionError, 'parameters'):
            valid_rectangle(message, 'compute-sgbm-valid-roi', (-4, 16, 3), np.nan)

    def test_installed_ros_camera_info_setters(self):
        try:
            from sensor_msgs.msg import CameraInfo
        except ImportError:
            self.skipTest('installed ROS sensor_msgs unavailable')
        message, info = fixture()
        _, calibration, _ = convert_disparity(message, info)
        output = CameraInfo()
        assign_camera_calibration(output, calibration)
        np.testing.assert_array_equal(output.r, np.eye(3).ravel())
        np.testing.assert_array_equal(output.p, info.p)
        self.assertTrue(all(isinstance(value, float) for value in calibration['r']))
        with self.assertRaisesRegex(ConversionError, 'depth_range'):
            convert_disparity(message, info, min_depth=np.nan)

    def test_native_timing_retained_for_failed_conversion(self):
        timing = simulation_timing(1_000_000_000, 1_020_000_000, 0, 0, 0, 'invalid_valid_window_metadata')
        self.assertTrue(timing['native_timing_valid'])
        self.assertFalse(timing['timing_valid'])
        self.assertEqual(timing['source_to_disparity_receive_sim_s'], .02)
        self.assertIsNone(timing['adapter_conversion_sim_s'])
        self.assertIsNone(timing['sip_native_publish_sim_ns'])
        for reason in ['simulation_clock_rewound', 'simulation_clock_uninitialized']:
            self.assertIsNone(simulation_timing(1, 2, 0, 0, 0, reason)['source_to_disparity_receive_sim_s'])
        self.assertFalse(simulation_timing(3, 2, 2, 2, 2)['native_timing_valid'])

    def test_simulation_clock_and_stage_validation(self):
        clock = SimulationClockTracker()
        self.assertEqual(clock.sample(0), 'simulation_clock_uninitialized')
        self.assertIsNone(clock.sample(10))
        self.assertEqual(clock.sample(9), 'simulation_clock_rewound')
        self.assertEqual(clock.sample(0), 'simulation_clock_rewound')
        timing = simulation_timing(1, 2, 3, 4, 5)
        self.assertTrue(timing['timing_valid'])
        self.assertEqual(timing['adapter_conversion_sim_s'], 1e-9)
        self.assertFalse(simulation_timing(1, 2, 4, 3, 5)['timing_valid'])
        frozen = simulation_timing(1, 2, 2, 2, 2)
        self.assertTrue(frozen['native_timing_valid'])
        self.assertFalse(frozen['adapter_stage_timing_available'])
        self.assertIsNone(frozen['adapter_conversion_sim_s'])


if __name__ == '__main__':
    unittest.main()
