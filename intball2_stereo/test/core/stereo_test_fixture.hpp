#ifndef INTBALL2_STEREO_TEST_CORE_STEREO_TEST_FIXTURE_HPP_
#define INTBALL2_STEREO_TEST_CORE_STEREO_TEST_FIXTURE_HPP_

#include "intball2_stereo/core/stereo_types.hpp"

namespace intball2_stereo::test
{
inline core::CameraCalibration calibration(int width_px = 200, int height_px = 200,
                                           double fx_px = 100.0, double tx_px_m = 0.0)
{
  core::CameraCalibration c;
  c.width = width_px;
  c.height = height_px;
  c.k = {fx_px, 0, (width_px - 1) / 2.0, 0, fx_px, (height_px - 1) / 2.0, 0, 0, 1};
  c.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  c.p = {fx_px, 0, c.k[2], tx_px_m, 0, fx_px, c.k[5], 0, 0, 0, 1, 0};
  c.distortion_model = "plumb_bob";
  c.d = {0, 0, 0, 0, 0};
  return c;
}

inline core::ImageFrame frame(const cv::Mat &pixels, const core::CameraCalibration &c,
                              const char *frame_id = "left_optical")
{
  return {core::ImageBuffer::copy_from(pixels), c, {123456789}, frame_id};
}
} // namespace intball2_stereo::test
#endif // INTBALL2_STEREO_TEST_CORE_STEREO_TEST_FIXTURE_HPP_
