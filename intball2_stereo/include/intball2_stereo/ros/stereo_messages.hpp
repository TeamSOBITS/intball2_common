#ifndef INTBALL2_STEREO_ROS_STEREO_MESSAGES_HPP_
#define INTBALL2_STEREO_ROS_STEREO_MESSAGES_HPP_
#include "intball2_stereo/core/stereo_config.hpp"
#include <array>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
namespace intball2_stereo::ros
{
struct InputBundle
{
  sensor_msgs::msg::Image::ConstSharedPtr left_image;
  sensor_msgs::msg::Image::ConstSharedPtr right_image;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr left_info;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr right_info;
  double synchronization_wait_s = 0.0;
  // Stream order: left image, right image, left info, right info.
  std::array<std::int64_t, 4> source_stamp_ns{};
  std::array<std::int64_t, 4> callback_arrival_sim_ns{};
  std::int64_t bundle_ready_sim_ns = 0;
  bool simulation_clock_rewound = false;
};
std::int64_t stamp_nanoseconds(const builtin_interfaces::msg::Time &stamp);
core::CameraCalibration camera_calibration(const sensor_msgs::msg::CameraInfo &info);
core::ImagePair image_pair(const InputBundle &input,
                           core::CalibrationMode mode = core::CalibrationMode::kCameraInfo);
struct OutputMessages
{
  sensor_msgs::msg::Image depth;
  sensor_msgs::msg::CameraInfo camera_info;
};
// Owns output bytes: safe across the next processor invocation.
OutputMessages output_messages(const core::DepthResult &result);
sensor_msgs::msg::PointCloud2 pointcloud_message(const core::PointCloudResult &result);
} // namespace intball2_stereo::ros
#endif
