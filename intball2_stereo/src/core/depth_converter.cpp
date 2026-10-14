#include "intball2_stereo/core/depth_converter.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include <opencv2/calib3d.hpp>

namespace intball2_stereo::core
{
namespace
{
bool overlaps_storage(const cv::Mat &left, const cv::Mat &right)
{
  if (left.empty() || right.empty()) {
    return false;
  }
  const auto left_begin = reinterpret_cast<std::uintptr_t>(left.datastart);
  const auto left_end = reinterpret_cast<std::uintptr_t>(left.dataend);
  const auto right_begin = reinterpret_cast<std::uintptr_t>(right.datastart);
  const auto right_end = reinterpret_cast<std::uintptr_t>(right.dataend);
  return left_begin < right_end && right_begin < left_end;
}
} // namespace

double DepthConverter::convert(const cv::Mat &disparity_fixed, const CameraModel &model,
                               const MatcherConfig &matcher, const DepthConfig &depth,
                               const cv::Mat &left_valid, const cv::Mat &right_valid,
                               cv::Mat &depth_m)
{
  StereoConfig check;
  check.camera.calibration_mode = CalibrationMode::kCameraInfo;
  check.matcher = matcher;
  check.depth = depth;
  check.processing.downsample = 1;
  const auto errors = validate_config_for_input(check, model.width_px(), model.height_px());
  if (!errors.empty()) {
    throw std::invalid_argument(errors.front());
  }
  const cv::Size size(model.width_px(), model.height_px());
  if (disparity_fixed.dims != 2 || disparity_fixed.type() != CV_16SC1 ||
      disparity_fixed.size() != size || left_valid.dims != 2 || right_valid.dims != 2 ||
      left_valid.type() != CV_8UC1 || right_valid.type() != CV_8UC1 || left_valid.size() != size ||
      right_valid.size() != size) {
    throw std::invalid_argument(
        "depth conversion needs model-sized CV_16S disparity and CV_8U masks");
  }
  if (overlaps_storage(depth_m, disparity_fixed) || overlaps_storage(depth_m, left_valid) ||
      overlaps_storage(depth_m, right_valid)) {
    throw std::invalid_argument("depth output must not alias disparity or validity masks");
  }
  depth_m.create(size, CV_32FC1);
  const auto nan = std::numeric_limits<float>::quiet_NaN();
  depth_m.setTo(nan);
  const cv::Rect full(0, 0, size.width, size.height);
  const auto roi = cv::getValidDisparityROI(full, full, matcher.min_disparity_px,
                                            matcher.num_disparities_px, matcher.block_size_px);
  const int invalid = (matcher.min_disparity_px - 1) * 16;
  const int maximum = (matcher.min_disparity_px + matcher.num_disparities_px) * 16;
  const double numerator = model.numerator_px_m();
  const double principal = model.principal_difference_px();
  std::uint64_t valid_count = 0;
  for (int row = roi.y; row < roi.y + roi.height; ++row) {
    const auto *disparities = disparity_fixed.ptr<std::int16_t>(row);
    const auto *left_mask = left_valid.ptr<unsigned char>(row);
    const auto *right_mask = right_valid.ptr<unsigned char>(row);
    auto *output = depth_m.ptr<float>(row);
    for (int column = roi.x; column < roi.x + roi.width; ++column) {
      const int fixed = disparities[column];
      if (!left_mask[column] || fixed <= invalid || fixed < matcher.min_disparity_px * 16 ||
          fixed >= maximum) {
        continue;
      }
      const double disparity = fixed / 16.0;
      const double right_x = column - disparity;
      if (right_x < 0 || right_x > size.width - 1) {
        continue;
      }
      const int right_floor = static_cast<int>(std::floor(right_x));
      const int right_ceil = static_cast<int>(std::ceil(right_x));
      if (!right_mask[right_floor] || !right_mask[right_ceil]) {
        continue;
      }
      const double denominator = disparity - principal;
      if (std::abs(denominator) <= 1e-12) {
        continue;
      }
      const double z_m = numerator / denominator;
      if (!std::isfinite(z_m) || z_m < depth.min_depth_m || z_m > depth.max_depth_m ||
          z_m > std::numeric_limits<float>::max()) {
        continue;
      }
      const auto stored_z = static_cast<float>(z_m);
      if (!std::isfinite(stored_z) || stored_z <= 0) {
        continue;
      }
      output[column] = stored_z;
      ++valid_count;
    }
  }
  return static_cast<double>(valid_count) / (static_cast<double>(size.width) * size.height);
}
} // namespace intball2_stereo::core
