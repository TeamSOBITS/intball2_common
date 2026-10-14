#include "intball2_stereo/core/pointcloud_generator.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace intball2_stereo::core
{
PointCloudResult PointCloudGenerator::generate(const DepthResult &depth, const PointsConfig &config)
{
  if (depth.depth_m.empty() || depth.depth_m.dims != 2 || depth.depth_m.type() != CV_32FC1 ||
      depth.calibration.width != static_cast<unsigned>(depth.depth_m.cols) ||
      depth.calibration.height != static_cast<unsigned>(depth.depth_m.rows) ||
      depth.optical_frame_id.empty()) {
    throw std::invalid_argument("pointcloud needs matching depth/calibration dimensions and frame");
  }
  if ((config.layout != PointsLayout::kOrganized && config.layout != PointsLayout::kCompact) ||
      config.stride < 1 || (config.layout == PointsLayout::kOrganized && config.stride != 1)) {
    throw std::invalid_argument(
        "pointcloud stride must be positive and organized stride must be 1");
  }
  const auto &k = depth.calibration.k;
  const double fx = k[0], fy = k[4], cx = k[2], cy = k[5];
  if (!std::isfinite(fx) || !std::isfinite(fy) || !std::isfinite(cx) || !std::isfinite(cy) ||
      fx <= 0 || fy <= 0) {
    throw std::invalid_argument("pointcloud intrinsics need finite positive focal lengths");
  }
  PointCloudResult output;
  output.layout = config.layout;
  output.stamp = depth.stamp;
  output.optical_frame_id = depth.optical_frame_id;
  const bool organized = config.layout == PointsLayout::kOrganized;
  const auto width = depth.calibration.width;
  const auto height = depth.calibration.height;
  const auto stride = static_cast<std::uint64_t>(config.stride);
  const auto sampled_width = (static_cast<std::uint64_t>(width) - 1) / stride + 1;
  const auto sampled_height = (static_cast<std::uint64_t>(height) - 1) / stride + 1;
  const auto capacity = sampled_width * sampled_height;
  if (capacity > output.points.max_size()) {
    throw std::invalid_argument("pointcloud size exceeds storage limits");
  }
  output.points.reserve(static_cast<std::size_t>(capacity));
  const auto nan = std::numeric_limits<float>::quiet_NaN();
  // Use unsigned wide indices so a large stride cannot overflow at loop increment.
  for (std::uint64_t row = 0; row < height; row += stride) {
    const auto *values = depth.depth_m.ptr<float>(static_cast<int>(row));
    for (std::uint64_t col = 0; col < width; col += stride) {
      const auto z = values[col];
      const auto x = static_cast<float>((static_cast<double>(col) - cx) * z / fx);
      const auto y = static_cast<float>((static_cast<double>(row) - cy) * z / fy);
      if (std::isfinite(z) && z > 0 && std::isfinite(x) && std::isfinite(y)) {
        output.points.push_back({x, y, z});
      } else if (organized) {
        output.points.push_back({nan, nan, nan});
      }
    }
  }
  if (!organized && output.points.size() > std::numeric_limits<std::uint32_t>::max()) {
    throw std::invalid_argument("compact pointcloud width exceeds uint32");
  }
  output.width = organized ? width : static_cast<std::uint32_t>(output.points.size());
  output.height = organized ? height : 1;
  return output;
}
} // namespace intball2_stereo::core
