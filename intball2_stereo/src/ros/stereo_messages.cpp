#include "intball2_stereo/ros/stereo_messages.hpp"
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
namespace intball2_stereo::ros
{
std::int64_t stamp_nanoseconds(const builtin_interfaces::msg::Time &stamp)
{
  if (stamp.nanosec >= 1000000000U) {
    throw std::invalid_argument("stamp.nanosec must be less than 1e9");
  }
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}
core::CameraCalibration camera_calibration(const sensor_msgs::msg::CameraInfo &info)
{
  return {
      info.width,
      info.height,
      info.distortion_model,
      info.k,
      info.d,
      info.r,
      info.p,
      {info.roi.x_offset, info.roi.y_offset, info.roi.width, info.roi.height, info.roi.do_rectify},
      info.binning_x,
      info.binning_y};
}
namespace
{
core::ImageFrame image_frame(const sensor_msgs::msg::Image::ConstSharedPtr &image,
                             const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info,
                             core::CalibrationMode mode, bool right)
{
  if (!image || !info) {
    throw std::invalid_argument("input messages must not be null");
  }
  const auto stamp = stamp_nanoseconds(image->header.stamp);
  core::ImageEncoding encoding;
  int channels = 1;
  if (image->encoding == "mono8") {
    encoding = core::ImageEncoding::kMono8;
  } else if (image->encoding == "bgr8" || image->encoding == "rgb8") {
    channels = 3;
    encoding = image->encoding == "bgr8" ? core::ImageEncoding::kBgr8 : core::ImageEncoding::kRgb8;
  } else {
    throw std::invalid_argument("input encoding must be mono8, bgr8 or rgb8");
  }
  if (image->width == 0 || image->height == 0 ||
      image->width > static_cast<unsigned>(std::numeric_limits<int>::max()) ||
      image->height > static_cast<unsigned>(std::numeric_limits<int>::max()) ||
      image->step < static_cast<std::uint64_t>(image->width) * channels ||
      static_cast<std::uint64_t>(image->step) * image->height > image->data.size()) {
    throw std::invalid_argument("invalid image dimensions, step or data size");
  }
  auto frame_id = image->header.frame_id;
  auto info_frame_id = info->header.frame_id;
  if (mode == core::CalibrationMode::kGazeboNegative5cm) {
    const std::string link = right ? "cameraR_link" : "cameraL_link";
    const std::string optical = right ? "cameraR_optical_frame" : "cameraL_optical_frame";
    if (frame_id == link) {
      frame_id = optical;
    }
    if (info_frame_id == link) {
      info_frame_id = optical;
    }
  }
  if (image->width != info->width || image->height != info->height || frame_id.empty() ||
      frame_id != info_frame_id) {
    throw std::invalid_argument("image and CameraInfo dimensions/optical frame must agree");
  }
  // cv::Mat has no const-data constructor. The borrowed view is read-only by contract.
  cv::Mat view(static_cast<int>(image->height), static_cast<int>(image->width),
               CV_MAKETYPE(CV_8U, channels), const_cast<unsigned char *>(image->data.data()),
               image->step);
  auto buffer = core::ImageBuffer::borrow(view, image);
  auto calibration = camera_calibration(*info);
  return {std::move(buffer), std::move(calibration), {stamp}, std::move(frame_id), encoding};
}
} // namespace
core::ImagePair image_pair(const InputBundle &input, core::CalibrationMode mode)
{
  auto left = image_frame(input.left_image, input.left_info, mode, false);
  auto right = image_frame(input.right_image, input.right_info, mode, true);
  return {std::move(left), std::move(right)};
}
OutputMessages output_messages(const core::DepthResult &result)
{
  if (result.depth_m.empty() || result.depth_m.type() != CV_32FC1 ||
      result.calibration.width != static_cast<unsigned>(result.depth_m.cols) ||
      result.calibration.height != static_cast<unsigned>(result.depth_m.rows) ||
      result.optical_frame_id.empty() ||
      result.depth_m.cols > std::numeric_limits<int>::max() / 4) {
    throw std::invalid_argument("invalid depth result shape/type/frame");
  }
  auto sec = result.stamp.nanoseconds / 1000000000LL;
  auto nsec = result.stamp.nanoseconds % 1000000000LL;
  if (nsec < 0) {
    --sec;
    nsec += 1000000000LL;
  }
  if (sec < std::numeric_limits<std::int32_t>::min() ||
      sec > std::numeric_limits<std::int32_t>::max()) {
    throw std::invalid_argument("output stamp is outside ROS Time range");
  }
  OutputMessages output;
  auto &depth = output.depth;
  depth.header.stamp.sec = static_cast<std::int32_t>(sec);
  depth.header.stamp.nanosec = static_cast<std::uint32_t>(nsec);
  depth.header.frame_id = result.optical_frame_id;
  depth.width = result.depth_m.cols;
  depth.height = result.depth_m.rows;
  depth.encoding = "32FC1";
  const std::uint16_t endian_probe = 1;
  depth.is_bigendian = *reinterpret_cast<const unsigned char *>(&endian_probe) == 0;
  depth.step = depth.width * sizeof(float);
  depth.data.resize(static_cast<std::size_t>(depth.step) * depth.height);
  for (int row = 0; row < result.depth_m.rows; ++row) {
    std::memcpy(depth.data.data() + static_cast<std::size_t>(row) * depth.step,
                result.depth_m.ptr(row), depth.step);
  }
  auto &info = output.camera_info;
  const auto &c = result.calibration;
  info.header = depth.header;
  info.width = c.width;
  info.height = c.height;
  info.distortion_model = c.distortion_model;
  info.k = c.k;
  info.d = c.d;
  info.r = c.r;
  info.p = c.p;
  info.binning_x = c.binning_x;
  info.binning_y = c.binning_y;
  info.roi.x_offset = c.roi.x_offset;
  info.roi.y_offset = c.roi.y_offset;
  info.roi.width = c.roi.width;
  info.roi.height = c.roi.height;
  info.roi.do_rectify = c.roi.do_rectify;
  return output;
}
sensor_msgs::msg::PointCloud2 pointcloud_message(const core::PointCloudResult &result)
{
  constexpr std::uint32_t kPointStep = 12;
  static_assert(sizeof(float) == 4, "PointCloud2 FLOAT32 requires 4-byte floats");
  const bool organized = result.layout == core::PointsLayout::kOrganized;
  if ((result.layout != core::PointsLayout::kOrganized &&
       result.layout != core::PointsLayout::kCompact) ||
      result.height == 0 || (organized && result.width == 0) ||
      (!organized && result.height != 1) || result.optical_frame_id.empty() ||
      static_cast<std::uint64_t>(result.width) * result.height != result.points.size() ||
      result.width > std::numeric_limits<std::uint32_t>::max() / kPointStep) {
    throw std::invalid_argument("invalid pointcloud shape/layout/frame or row_step overflow");
  }
  const auto row_step = result.width * kPointStep;
  const auto byte_count = static_cast<std::uint64_t>(row_step) * result.height;
  sensor_msgs::msg::PointCloud2 output;
  if (byte_count > output.data.max_size()) {
    throw std::invalid_argument("pointcloud byte size exceeds storage limits");
  }
  auto sec = result.stamp.nanoseconds / 1000000000LL;
  auto nsec = result.stamp.nanoseconds % 1000000000LL;
  if (nsec < 0) {
    --sec;
    nsec += 1000000000LL;
  }
  if (sec < std::numeric_limits<std::int32_t>::min() ||
      sec > std::numeric_limits<std::int32_t>::max()) {
    throw std::invalid_argument("output stamp is outside ROS Time range");
  }
  output.header.stamp.sec = static_cast<std::int32_t>(sec);
  output.header.stamp.nanosec = static_cast<std::uint32_t>(nsec);
  output.header.frame_id = result.optical_frame_id;
  output.width = result.width;
  output.height = result.height;
  output.point_step = kPointStep;
  output.row_step = row_step;
  const std::uint16_t endian_probe = 1;
  output.is_bigendian = *reinterpret_cast<const unsigned char *>(&endian_probe) == 0;
  const char *names[] = {"x", "y", "z"};
  for (std::uint32_t i = 0; i < 3; ++i) {
    sensor_msgs::msg::PointField field;
    field.name = names[i];
    field.offset = i * sizeof(float);
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
    output.fields.push_back(std::move(field));
  }
  output.data.resize(static_cast<std::size_t>(byte_count));
  output.is_dense = true;
  for (std::size_t i = 0; i < result.points.size(); ++i) {
    const auto &point = result.points[i];
    const float values[] = {point.x_m, point.y_m, point.z_m};
    std::memcpy(output.data.data() + i * kPointStep, values, kPointStep);
    if (!std::isfinite(point.x_m) || !std::isfinite(point.y_m) || !std::isfinite(point.z_m)) {
      output.is_dense = false;
    }
  }
  return output;
}
} // namespace intball2_stereo::ros
