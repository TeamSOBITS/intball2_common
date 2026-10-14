#include "intball2_stereo/core/stereo_config.hpp"

#include <climits>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
int checks = 0;

void check(bool condition, const char *reason)
{
  ++checks;
  if (!condition) {
    throw std::runtime_error(reason);
  }
}

template <typename Function> void rejects(Function function, const char *reason)
{
  bool rejected = false;
  try {
    function();
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  check(rejected, reason);
}
} // namespace

int main()
{
  using intball2_stereo::core::CalibrationMode;
  using intball2_stereo::core::DisparityUnits;
  using intball2_stereo::core::ImageBuffer;
  using intball2_stereo::core::InputGeometry;
  using intball2_stereo::core::PointsLayout;
  using intball2_stereo::core::require_valid_config;
  using intball2_stereo::core::StereoConfig;
  using intball2_stereo::core::validate_config;
  using intball2_stereo::core::validate_config_for_input;
  try {
    const StereoConfig defaults;
    check(validate_config(defaults).empty(), "default config must be valid");
    check(validate_config_for_input(defaults, 800, 800).empty(), "800 -> 200 default must fit");
    check(defaults.processing.downsample == 4 && !defaults.points.publish_points &&
              defaults.points.layout == PointsLayout::kOrganized && defaults.points.stride == 1,
          "public defaults changed");

    const std::vector<std::function<void(StereoConfig &)>> invalid_cases = {
        [](auto &c) { c.camera.calibration_mode = static_cast<CalibrationMode>(99); },
        [](auto &c) { c.camera.input_geometry = static_cast<InputGeometry>(99); },
        [](auto &c) { c.camera.input_geometry = InputGeometry::kRaw; },
        [](auto &c) { c.matcher.disparity_units = static_cast<DisparityUnits>(99); },
        [](auto &c) { c.matcher.min_disparity_px = 0; },
        [](auto &c) { c.matcher.min_disparity_px = INT_MIN; },
        [](auto &c) { c.matcher.min_disparity_px = INT_MAX; },
        [](auto &c) { c.matcher.num_disparities_px = 0; },
        [](auto &c) { c.matcher.num_disparities_px = 31; },
        [](auto &c) { c.matcher.num_disparities_px = INT_MAX; },
        [](auto &c) { c.matcher.block_size_px = 4; },
        [](auto &c) { c.matcher.uniqueness_ratio = 101; },
        [](auto &c) { c.matcher.speckle_window_size_px = -1; },
        [](auto &c) { c.matcher.speckle_range_px = -1; },
        [](auto &c) { c.matcher.disp12_max_diff_px = -2; },
        [](auto &c) { c.depth.min_depth_m = 0.0; },
        [](auto &c) { c.depth.min_depth_m = std::numeric_limits<double>::quiet_NaN(); },
        [](auto &c) { c.depth.max_depth_m = std::numeric_limits<double>::infinity(); },
        [](auto &c) { c.depth.max_depth_m = c.depth.min_depth_m; },
        [](auto &c) { c.points.layout = static_cast<PointsLayout>(99); },
        [](auto &c) { c.points.stride = 2; },
        [](auto &c) { c.points.stride = 0; },
        [](auto &c) { c.processing.downsample = 3; },
        [](auto &c) { c.processing.opencv_threads = 0; }};
    for (const auto &mutate : invalid_cases) {
      auto config = defaults;
      mutate(config);
      check(!validate_config(config).empty(), "invalid configuration accepted");
      rejects([&] { require_valid_config(config); }, "invalid configuration did not throw");
    }

    auto config = defaults;
    config.points.layout = PointsLayout::kCompact;
    config.points.stride = 4;
    check(validate_config(config).empty(), "compact stride must be supported");
    config.camera.calibration_mode = CalibrationMode::kCameraInfo;
    config.camera.input_geometry = InputGeometry::kRaw;
    config.matcher.min_disparity_px = 0;
    check(validate_config(config).empty(), "ordinary raw camera_info must be configurable");

    config.matcher.min_disparity_px = -2047;
    config.matcher.num_disparities_px = 4096;
    check(!validate_config(config).empty(), "positive int16 disparity overflow accepted");
    config.matcher.num_disparities_px = 4080;
    check(validate_config(config).empty(), "lower representable sentinel rejected");
    config.matcher.min_disparity_px = -2048;
    check(!validate_config(config).empty(), "min-1 int16 sentinel overflow accepted");
    config.matcher.min_disparity_px = 2032;
    config.matcher.num_disparities_px = 16;
    check(validate_config(config).empty(), "upper representable disparity rejected");
    config.matcher.min_disparity_px = 2033;
    check(!validate_config(config).empty(), "upper int16 overflow accepted");

    check(!validate_config_for_input(defaults, 799, 800).empty(), "fractional resize accepted");
    check(!validate_config_for_input(defaults, 0, 800).empty(), "empty dimensions accepted");
    check(!validate_config_for_input(defaults, 100, 100).empty(), "oversized matcher accepted");
    config = defaults;
    config.matcher.min_disparity_px = -300;
    config.matcher.num_disparities_px = 16;
    check(!validate_config_for_input(config, 800, 800).empty(),
          "off-image negative interval accepted");
    config = defaults;
    config.processing.downsample = 1;
    config.matcher.min_disparity_px = -98;
    config.matcher.num_disparities_px = 128;
    check(validate_config_for_input(config, 800, 800).empty(), "full-size retuned config rejected");

    cv::Mat source(2, 2, CV_8UC1, cv::Scalar(7));
    const auto owned = ImageBuffer::copy_from(source);
    source.setTo(99);
    check(owned.pixels().at<unsigned char>(0, 0) == 7, "owned image aliases input");
    rejects([] { ImageBuffer::copy_from(cv::Mat{}); }, "empty image accepted");
    rejects([&] { ImageBuffer::borrow(source, {}); }, "ownerless borrow accepted");
    std::weak_ptr<std::vector<unsigned char>> lifetime;
    {
      auto owner = std::make_shared<std::vector<unsigned char>>(4, 42);
      lifetime = owner;
      cv::Mat view(2, 2, CV_8UC1, owner->data());
      const auto borrowed = ImageBuffer::borrow(view, owner);
      const auto copy = borrowed;
      owner.reset();
      check(!lifetime.expired(), "external image storage released too early");
      check(copy.pixels().at<unsigned char>(1, 1) == 42, "borrowed copied image lost pixels");
    }
    check(lifetime.expired(), "external image owner leaked");
    std::cout << checks << " core contract checks passed\n";
  } catch (const std::exception &error) {
    std::cerr << "core contract failure: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
