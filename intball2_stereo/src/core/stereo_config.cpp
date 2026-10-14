#include "intball2_stereo/core/stereo_config.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <stdexcept>

namespace intball2_stereo::core
{
std::vector<std::string> validate_config(const StereoConfig &config)
{
  std::vector<std::string> errors;
  const auto check = [&errors](bool condition, const char *reason) {
    if (!condition) {
      errors.emplace_back(reason);
    }
  };
  check(config.camera.calibration_mode == CalibrationMode::kCameraInfo ||
            config.camera.calibration_mode == CalibrationMode::kGazeboNegative5cm,
        "camera.calibration_mode is invalid");
  check(config.camera.input_geometry == InputGeometry::kRaw ||
            config.camera.input_geometry == InputGeometry::kAlreadyRectified,
        "camera.input_geometry is invalid");
  check(config.matcher.disparity_units == DisparityUnits::kProcessedPixels,
        "matcher.disparity_units must be processed_pixels");
  const auto &matcher = config.matcher;
  check(matcher.min_disparity_px >= -4096 && matcher.min_disparity_px <= 4096,
        "matcher.min_disparity_px must be in [-4096, 4096]");
  check(matcher.num_disparities_px > 0 && matcher.num_disparities_px <= 4096 &&
            matcher.num_disparities_px % 16 == 0,
        "matcher.num_disparities_px must be a multiple of 16 in [16, 4096]");
  const auto invalid_fixed = (static_cast<std::int64_t>(matcher.min_disparity_px) - 1) * 16;
  const auto maximum_fixed =
      (static_cast<std::int64_t>(matcher.min_disparity_px) + matcher.num_disparities_px - 1) * 16;
  check(invalid_fixed >= -32768 && invalid_fixed <= 32767 && maximum_fixed >= -32768 &&
            maximum_fixed <= 32767,
        "matcher disparity interval and min-1 sentinel must fit signed 16-bit SGBM output at scale "
        "16");
  check(matcher.block_size_px >= 3 && matcher.block_size_px <= 255 &&
            matcher.block_size_px % 2 == 1,
        "matcher.block_size_px must be odd in [3, 255]");
  check(matcher.uniqueness_ratio >= 0 && matcher.uniqueness_ratio <= 100,
        "matcher.uniqueness_ratio must be in [0, 100]");
  check(matcher.speckle_window_size_px >= 0 && matcher.speckle_window_size_px <= 1000000,
        "matcher.speckle_window_size_px must be in [0, 1000000]");
  check(matcher.speckle_range_px >= 0 && matcher.speckle_range_px <= 4096,
        "matcher.speckle_range_px must be in [0, 4096]");
  check(matcher.disp12_max_diff_px >= -1 && matcher.disp12_max_diff_px <= 4096,
        "matcher.disp12_max_diff_px must be in [-1, 4096]");
  if (config.camera.calibration_mode == CalibrationMode::kGazeboNegative5cm) {
    check(matcher.min_disparity_px < 0,
          "Gazebo negative disparity mode needs a search interval containing negative pixels");
    check(config.camera.input_geometry == InputGeometry::kAlreadyRectified,
          "Gazebo negative 5cm mode requires already_rectified input");
  }
  check(std::isfinite(config.depth.min_depth_m) && config.depth.min_depth_m > 0.0,
        "depth.min_depth_m must be finite and positive");
  check(std::isfinite(config.depth.max_depth_m) &&
            config.depth.max_depth_m > config.depth.min_depth_m,
        "depth.max_depth_m must be finite and greater than min_depth_m");
  check(config.points.layout == PointsLayout::kOrganized ||
            config.points.layout == PointsLayout::kCompact,
        "points.layout is invalid");
  check(config.points.stride >= 1 && config.points.stride <= 4096,
        "points.stride must be in [1, 4096]");
  check(config.points.layout != PointsLayout::kOrganized || config.points.stride == 1,
        "organized points require points.stride=1, even when publishing is disabled");
  check(config.processing.downsample == 1 || config.processing.downsample == 2 ||
            config.processing.downsample == 4,
        "processing.downsample must be 1, 2 or 4");
  check(config.processing.opencv_threads >= 1 && config.processing.opencv_threads <= 64,
        "processing.opencv_threads must be in [1, 64]");
  const auto &processing = config.processing;
  for (const auto margin : {processing.roi_left_px, processing.roi_right_px, processing.roi_top_px,
                            processing.roi_bottom_px}) {
    check(margin >= 0 && margin <= 4096, "processing.roi_*_px must be in [0, 4096] output pixels");
  }
  if (processing.roi_left_px || processing.roi_right_px || processing.roi_top_px ||
      processing.roi_bottom_px) {
    check(config.camera.input_geometry == InputGeometry::kAlreadyRectified,
          "pre-SGBM ROI requires already_rectified input");
  }
  const auto &refinement = config.refinement;
  check(refinement.downsample == 1 || refinement.downsample == 2 || refinement.downsample == 4,
        "refinement.downsample must be 1, 2 or 4");
  check(refinement.window_size_px >= 3 && refinement.window_size_px <= 31 &&
            refinement.window_size_px % 2 == 1,
        "refinement.window_size_px must be odd in [3, 31]");
  check(refinement.search_radius_px >= 1 && refinement.search_radius_px <= 8,
        "refinement.search_radius_px must be in [1, 8]");
  check(std::isfinite(refinement.min_texture_variance) && refinement.min_texture_variance > 0,
        "refinement.min_texture_variance must be finite and positive");
  check(std::isfinite(refinement.min_correlation) && refinement.min_correlation >= 0 &&
            refinement.min_correlation <= 1,
        "refinement.min_correlation must be in [0, 1]");
  if (refinement.enabled) {
    check(config.camera.input_geometry == InputGeometry::kAlreadyRectified,
          "refinement requires already_rectified input");
    check(refinement.downsample <= config.processing.downsample,
          "refinement resolution must be at least matching resolution");
  }
  return errors;
}

std::vector<std::string> validate_config_for_input(const StereoConfig &config, int input_width_px,
                                                   int input_height_px)
{
  auto errors = validate_config(config);
  if (!errors.empty()) {
    return errors;
  }
  const int factor = config.processing.downsample;
  if (input_width_px <= 0 || input_height_px <= 0 || input_width_px % factor != 0 ||
      input_height_px % factor != 0) {
    errors.emplace_back("input dimensions must be positive and divisible by downsample");
    return errors;
  }
  if (config.refinement.enabled &&
      (input_width_px % config.refinement.downsample ||
       input_height_px % config.refinement.downsample ||
       input_width_px / config.refinement.downsample < config.refinement.window_size_px + 2 ||
       input_height_px / config.refinement.downsample < config.refinement.window_size_px + 2)) {
    errors.emplace_back("refinement window/resolution must fit input dimensions");
  }
  const int width =
      input_width_px / factor - config.processing.roi_left_px - config.processing.roi_right_px;
  const int height =
      input_height_px / factor - config.processing.roi_top_px - config.processing.roi_bottom_px;
  if (width <= 0 || height <= 0) {
    errors.emplace_back("processing ROI margins leave no output-grid image");
    return errors;
  }
  const auto &matcher = config.matcher;
  const auto upper =
      static_cast<std::int64_t>(matcher.min_disparity_px) + matcher.num_disparities_px;
  if (matcher.num_disparities_px >= width || matcher.min_disparity_px <= -width || upper > width ||
      matcher.block_size_px > width || matcher.block_size_px > height) {
    errors.emplace_back("matcher search interval/block must fit the downsampled matching ROI");
  }
  const auto radius = matcher.block_size_px / 2;
  const auto begin = std::max<std::int64_t>(0, upper - 1) + radius;
  const auto end =
      static_cast<std::int64_t>(width) + std::min(0, matcher.min_disparity_px) - radius;
  if (begin >= end || height <= 2 * radius) {
    errors.emplace_back("matcher leaves no valid block-supported disparity region");
  }
  return errors;
}

void require_valid_config(const StereoConfig &config)
{
  const auto errors = validate_config(config);
  if (errors.empty()) {
    return;
  }
  std::ostringstream reason;
  for (const auto &error : errors) {
    reason << error << '\n';
  }
  throw std::invalid_argument(reason.str());
}
} // namespace intball2_stereo::core
