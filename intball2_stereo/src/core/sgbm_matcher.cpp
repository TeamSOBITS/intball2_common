#include "intball2_stereo/core/sgbm_matcher.hpp"

#include <stdexcept>

namespace intball2_stereo::core
{
SgbmMatcher::SgbmMatcher(const MatcherConfig &config) : config_(config)
{
  StereoConfig check;
  check.camera.calibration_mode = CalibrationMode::kCameraInfo;
  check.matcher = config;
  require_valid_config(check);
  const int block_area = config.block_size_px * config.block_size_px;
  matcher_ = cv::StereoSGBM::create(
      config.min_disparity_px, config.num_disparities_px, config.block_size_px, 8 * block_area,
      32 * block_area, config.disp12_max_diff_px, 31, config.uniqueness_ratio,
      config.speckle_window_size_px, config.speckle_range_px, cv::StereoSGBM::MODE_SGBM_3WAY);
}

const cv::Mat &SgbmMatcher::compute(const cv::Mat &left_gray, const cv::Mat &right_gray)
{
  if (left_gray.empty() || left_gray.dims != 2 || right_gray.dims != 2 ||
      left_gray.type() != CV_8UC1 || right_gray.type() != CV_8UC1 ||
      left_gray.size() != right_gray.size()) {
    throw std::invalid_argument("SGBM requires equal-sized mono8 images");
  }
  StereoConfig check;
  check.camera.calibration_mode = CalibrationMode::kCameraInfo;
  check.matcher = config_;
  check.processing.downsample = 1;
  const auto errors = validate_config_for_input(check, left_gray.cols, left_gray.rows);
  if (!errors.empty()) {
    throw std::invalid_argument(errors.front());
  }
  matcher_->compute(left_gray, right_gray, disparity_);
  return disparity_;
}
} // namespace intball2_stereo::core
