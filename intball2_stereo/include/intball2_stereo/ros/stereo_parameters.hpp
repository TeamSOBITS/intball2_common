#ifndef INTBALL2_STEREO_ROS_STEREO_PARAMETERS_HPP_
#define INTBALL2_STEREO_ROS_STEREO_PARAMETERS_HPP_

#include <cstdint>
#include <string>
#include <variant>
#include <vector>

#include <rclcpp/node.hpp>
#include <rclcpp/parameter.hpp>

#include "intball2_stereo/core/stereo_config.hpp"

namespace intball2_stereo::ros
{
enum class UpdatePolicy
{
  kStartupOnly,
  kFutureFrameBoundary
};

using ParameterDefault = std::variant<bool, std::int64_t, double, std::string>;

struct ParameterSpec
{
  std::string name;
  ParameterDefault default_value;
  UpdatePolicy update_policy;
  std::string description;
};

struct TopicsConfig
{
  std::string left_image = "/camera_left/image_raw";
  std::string right_image = "/camera_right/image_raw";
  std::string left_camera_info = "/camera_left/camera_info";
  std::string right_camera_info = "/camera_right/camera_info";
  std::string depth = "/stereo/depth";
  std::string camera_info = "/stereo/camera_info";
  std::string points = "/stereo/points";
  std::string diagnostics = "/diagnostics";
  std::string timing = "/stereo/timing";
};

struct SyncConfig
{
  int queue_size = 5;
  double max_interval_s = 0.02;
};

struct QosConfig
{
  // SensorData QoS contract; configuring more policies is outside step 1.
  int depth = 5;
};

struct DiagnosticsConfig
{
  double publish_interval_s = 1.0;
};

struct StereoParametersConfig
{
  core::StereoConfig core;
  TopicsConfig topics;
  SyncConfig sync;
  QosConfig qos;
  DiagnosticsConfig diagnostics;
};

struct ParameterSnapshot
{
  std::vector<rclcpp::Parameter> values;
};

// This wrapper does not inherit Node or register callbacks. Validation never
// applies settings; the worker reads committed values at a frame boundary.
class StereoParameters
{
public:
  static const std::vector<ParameterSpec> &specifications();
  static StereoParametersConfig declare_and_read(rclcpp::Node &node);
  static StereoParametersConfig read(const rclcpp::Node &node);
  // Pure conversion/validation seam; no rclcpp::init, Node, graph or executor.
  // Missing values use typed defaults; duplicate/unknown names are rejected.
  static StereoParametersConfig read_values(const std::vector<rclcpp::Parameter> &values);
  // Validate a whole proposed transaction without mutating either input.
  // Missing current values use defaults; startup-only updates are rejected.
  static StereoParametersConfig
  validate_updates(const ParameterSnapshot &current_snapshot,
                   const std::vector<rclcpp::Parameter> &proposed_values);
};

// Worker-owned immutable configuration; construct a replacement before commit.
class StereoFrameConfiguration
{
public:
  explicit StereoFrameConfiguration(ParameterSnapshot snapshot);
  bool matches(const ParameterSnapshot &snapshot) const;
  const StereoParametersConfig &config() const;

private:
  ParameterSnapshot snapshot_;
  StereoParametersConfig config_;
};

std::vector<std::string> validate_parameters(const StereoParametersConfig &config);
} // namespace intball2_stereo::ros

#endif // INTBALL2_STEREO_ROS_STEREO_PARAMETERS_HPP_
