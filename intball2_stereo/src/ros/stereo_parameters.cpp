#include "intball2_stereo/ros/stereo_parameters.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rmw/validate_full_topic_name.h>

namespace intball2_stereo::ros
{
namespace
{
using Config = StereoParametersConfig;

struct Binding
{
  ParameterSpec spec;
  std::function<void(Config &, const rclcpp::Parameter &)> assign;
};

template <typename T> T read_value(const rclcpp::Parameter &parameter)
{
  if constexpr (std::is_same_v<T, int>) {
    const auto value = parameter.as_int();
    if (value < std::numeric_limits<int>::min() || value > std::numeric_limits<int>::max()) {
      throw std::invalid_argument("integer does not fit int");
    }
    return static_cast<int>(value);
  } else if constexpr (std::is_same_v<T, bool>) {
    return parameter.as_bool();
  } else if constexpr (std::is_same_v<T, double>) {
    return parameter.as_double();
  } else if constexpr (std::is_same_v<T, std::string>) {
    return parameter.as_string();
  } else {
    const auto &value = parameter.as_string();
    if constexpr (std::is_same_v<T, core::CalibrationMode>) {
      if (value == "camera_info") {
        return core::CalibrationMode::kCameraInfo;
      }
      if (value == "gazebo_negative_5cm") {
        return core::CalibrationMode::kGazeboNegative5cm;
      }
    } else if constexpr (std::is_same_v<T, core::InputGeometry>) {
      if (value == "raw") {
        return core::InputGeometry::kRaw;
      }
      if (value == "already_rectified") {
        return core::InputGeometry::kAlreadyRectified;
      }
    } else if constexpr (std::is_same_v<T, core::PointsLayout>) {
      if (value == "organized") {
        return core::PointsLayout::kOrganized;
      }
      if (value == "compact") {
        return core::PointsLayout::kCompact;
      }
    } else if constexpr (std::is_same_v<T, core::DisparityUnits>) {
      if (value == "processed_pixels") {
        return core::DisparityUnits::kProcessedPixels;
      }
    }
    throw std::invalid_argument("unsupported enum value: " + value);
  }
}

template <typename T> ParameterDefault default_value(T value)
{
  if constexpr (std::is_same_v<T, int>) {
    return static_cast<std::int64_t>(value);
  } else if constexpr (std::is_same_v<T, core::CalibrationMode>) {
    return std::string(value == core::CalibrationMode::kCameraInfo ? "camera_info"
                                                                   : "gazebo_negative_5cm");
  } else if constexpr (std::is_same_v<T, core::InputGeometry>) {
    return std::string(value == core::InputGeometry::kRaw ? "raw" : "already_rectified");
  } else if constexpr (std::is_same_v<T, core::PointsLayout>) {
    return std::string(value == core::PointsLayout::kOrganized ? "organized" : "compact");
  } else if constexpr (std::is_same_v<T, core::DisparityUnits>) {
    return std::string("processed_pixels");
  } else {
    return value;
  }
}

template <typename Accessor>
void add_binding(std::vector<Binding> &bindings, const char *name, UpdatePolicy policy,
                 const char *description, Accessor accessor)
{
  Config defaults;
  using T = std::remove_reference_t<decltype(accessor(defaults))>;
  bindings.push_back({{name, default_value(accessor(defaults)), policy, description},
                      [accessor](Config &config, const rclcpp::Parameter &parameter) {
                        accessor(config) = read_value<T>(parameter);
                      }});
}

const std::vector<Binding> &bindings()
{
  static const auto result = [] {
    std::vector<Binding> values;
    constexpr auto kStartup = UpdatePolicy::kStartupOnly;
    constexpr auto kFrame = UpdatePolicy::kFutureFrameBoundary;
    // Each field appears once: metadata, declaration, read and assignment share
    // these bindings. Defaults always come from typed config initializers.
#define ADD_PARAMETER(name, policy, description, field)                                            \
  add_binding(                                                                                     \
      values, name, policy, description, [](Config & config) -> auto & { return config.field; })
    ADD_PARAMETER(
        "camera.calibration_mode", kFrame,
        "camera_info or gazebo_negative_5cm; Gazebo mode accepts uncorrected source info only",
        core.camera.calibration_mode);
    ADD_PARAMETER("camera.input_geometry", kFrame,
                  "raw or already_rectified; never rectify an already corrected input again",
                  core.camera.input_geometry);
    ADD_PARAMETER("processing.downsample", kFrame, "1, 2 or 4; retune matcher in processed pixels",
                  core.processing.downsample);
    ADD_PARAMETER("processing.opencv_threads", kStartup, "OpenCV thread limit, 1 through 64",
                  core.processing.opencv_threads);
    ADD_PARAMETER(
        "processing.roi_left_px", kFrame,
        "Left matching crop margin in published output-grid pixels; default 0; rectified only",
        core.processing.roi_left_px);
    ADD_PARAMETER(
        "processing.roi_right_px", kFrame,
        "Right matching crop margin in published output-grid pixels; default 0; rectified only",
        core.processing.roi_right_px);
    ADD_PARAMETER(
        "processing.roi_top_px", kFrame,
        "Top matching crop margin in published output-grid pixels; default 0; rectified only",
        core.processing.roi_top_px);
    ADD_PARAMETER(
        "processing.roi_bottom_px", kFrame,
        "Bottom matching crop margin in published output-grid pixels; default 0; rectified only",
        core.processing.roi_bottom_px);
    ADD_PARAMETER("matcher.min_disparity_px", kFrame,
                  "Minimum signed disparity in processed pixels", core.matcher.min_disparity_px);
    ADD_PARAMETER("matcher.num_disparities_px", kFrame,
                  "Processed-pixel search width; positive multiple of 16",
                  core.matcher.num_disparities_px);
    ADD_PARAMETER("matcher.disparity_units", kFrame, "Only processed_pixels is accepted",
                  core.matcher.disparity_units);
    ADD_PARAMETER("matcher.block_size_px", kFrame,
                  "Odd block size in processed pixels, 3 through 255", core.matcher.block_size_px);
    ADD_PARAMETER("matcher.uniqueness_ratio", kFrame, "SGBM uniqueness percentage, 0 through 100",
                  core.matcher.uniqueness_ratio);
    ADD_PARAMETER("matcher.speckle_window_size_px", kFrame,
                  "Maximum speckle component area in processed image pixels; 0 disables",
                  core.matcher.speckle_window_size_px);
    ADD_PARAMETER("matcher.speckle_range_px", kFrame,
                  "Speckle disparity variation in processed pixels", core.matcher.speckle_range_px);
    ADD_PARAMETER("matcher.disp12_max_diff_px", kFrame,
                  "Left/right consistency tolerance in processed pixels; -1 disables",
                  core.matcher.disp12_max_diff_px);
    ADD_PARAMETER("refinement.enabled", kFrame,
                  "Optional fine-resolution refinement; retain coarse depth when confidence fails",
                  core.refinement.enabled);
    ADD_PARAMETER("refinement.downsample", kFrame,
                  "1, 2 or 4; refinement resolution must cover matching resolution",
                  core.refinement.downsample);
    ADD_PARAMETER("refinement.window_size_px", kFrame,
                  "Odd correlation window in refinement pixels, 3 through 31",
                  core.refinement.window_size_px);
    ADD_PARAMETER("refinement.search_radius_px", kFrame,
                  "Correlation search radius in refinement pixels, 1 through 8",
                  core.refinement.search_radius_px);
    ADD_PARAMETER("refinement.min_texture_variance", kFrame,
                  "Finite positive minimum intensity variance for refinement",
                  core.refinement.min_texture_variance);
    ADD_PARAMETER("refinement.min_correlation", kFrame,
                  "Minimum normalized correlation, 0 through 1", core.refinement.min_correlation);
    ADD_PARAMETER("depth.min_depth_m", kFrame,
                  "Positive finite optical-axis depth lower limit in metres",
                  core.depth.min_depth_m);
    ADD_PARAMETER("depth.max_depth_m", kFrame, "Finite depth upper limit, greater than lower limit",
                  core.depth.max_depth_m);
    ADD_PARAMETER("points.publish_points", kFrame, "Optional XYZ generation and publication",
                  core.points.publish_points);
    ADD_PARAMETER("points.layout", kFrame, "organized (stride 1) or compact", core.points.layout);
    ADD_PARAMETER("points.stride", kFrame, "Compact sampling stride; organized always requires 1",
                  core.points.stride);
    ADD_PARAMETER("topics.left_image", kStartup, "Left image input topic", topics.left_image);
    ADD_PARAMETER("topics.right_image", kStartup, "Right image input topic", topics.right_image);
    ADD_PARAMETER("topics.left_camera_info", kStartup, "Left calibration input topic",
                  topics.left_camera_info);
    ADD_PARAMETER("topics.right_camera_info", kStartup, "Right calibration input topic",
                  topics.right_camera_info);
    ADD_PARAMETER("topics.depth", kStartup, "Depth output topic", topics.depth);
    ADD_PARAMETER("topics.camera_info", kStartup, "Output-resolution calibration topic",
                  topics.camera_info);
    ADD_PARAMETER("topics.points", kStartup, "XYZ output topic", topics.points);
    ADD_PARAMETER("topics.diagnostics", kStartup, "Diagnostic array output topic",
                  topics.diagnostics);
    ADD_PARAMETER("topics.timing", kStartup, "Per-frame simulation-clock timing output topic",
                  topics.timing);
    ADD_PARAMETER("diagnostics.publish_interval_s", kStartup,
                  "Finite diagnostic publication interval in [0.01, 60] seconds",
                  diagnostics.publish_interval_s);
    ADD_PARAMETER("sync.queue_size", kStartup, "Bounded synchronization queue size, 1 through 1000",
                  sync.queue_size);
    ADD_PARAMETER("sync.max_interval_s", kStartup,
                  "Finite permitted pair stamp difference in [0, 1] seconds", sync.max_interval_s);
    ADD_PARAMETER("qos.depth", kStartup, "SensorData QoS depth, 1 through 1000", qos.depth);
#undef ADD_PARAMETER
    return values;
  }();
  return result;
}
} // namespace

const std::vector<ParameterSpec> &StereoParameters::specifications()
{
  static const auto specs = [] {
    std::vector<ParameterSpec> result;
    for (const auto &binding : bindings()) {
      result.push_back(binding.spec);
    }
    return result;
  }();
  return specs;
}

StereoParametersConfig StereoParameters::declare_and_read(rclcpp::Node &node)
{
  for (const auto &spec : specifications()) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = spec.description;
    descriptor.read_only = spec.update_policy == UpdatePolicy::kStartupOnly;
    descriptor.additional_constraints =
        spec.update_policy == UpdatePolicy::kStartupOnly
            ? "startup-only"
            : "validated together and applied at the next frame boundary";
    std::visit([&](const auto &value) { node.declare_parameter(spec.name, value, descriptor); },
               spec.default_value);
  }
  return read(node);
}

StereoParametersConfig StereoParameters::read(const rclcpp::Node &node)
{
  std::vector<std::string> names;
  for (const auto &spec : specifications()) {
    names.push_back(spec.name);
  }
  return read_values(node.get_parameters(names));
}

StereoParametersConfig
StereoParameters::validate_updates(const ParameterSnapshot &current_snapshot,
                                   const std::vector<rclcpp::Parameter> &proposed_values)
{
  read_values(current_snapshot.values);
  auto candidate = current_snapshot.values;
  std::set<std::string> seen;
  for (const auto &parameter : proposed_values) {
    const auto &name = parameter.get_name();
    if (!seen.insert(name).second) {
      throw std::invalid_argument("duplicate parameter update: " + name);
    }
    const auto &specs = specifications();
    const auto spec = std::find_if(specs.begin(), specs.end(),
                                   [&](const ParameterSpec &entry) { return entry.name == name; });
    if (spec == specs.end()) {
      throw std::invalid_argument("unknown parameter: " + name);
    }
    if (spec->update_policy == UpdatePolicy::kStartupOnly) {
      throw std::invalid_argument("startup-only parameter: " + name);
    }
    const auto current =
        std::find_if(candidate.begin(), candidate.end(),
                     [&](const rclcpp::Parameter &entry) { return entry.get_name() == name; });
    if (current == candidate.end()) {
      candidate.push_back(parameter);
    } else {
      *current = parameter;
    }
  }
  return read_values(candidate);
}

StereoParametersConfig StereoParameters::read_values(const std::vector<rclcpp::Parameter> &values)
{
  Config config;
  std::set<std::string> seen;
  for (const auto &parameter : values) {
    if (!seen.insert(parameter.get_name()).second) {
      throw std::invalid_argument("duplicate parameter: " + parameter.get_name());
    }
    const auto &all = bindings();
    const auto found = std::find_if(all.begin(), all.end(), [&](const Binding &binding) {
      return binding.spec.name == parameter.get_name();
    });
    if (found == all.end()) {
      throw std::invalid_argument("unknown parameter: " + parameter.get_name());
    }
    try {
      found->assign(config, parameter);
    } catch (const std::exception &error) {
      throw std::invalid_argument(parameter.get_name() + ": " + error.what());
    }
  }
  const auto errors = validate_parameters(config);
  if (!errors.empty()) {
    std::ostringstream reason;
    for (const auto &error : errors) {
      reason << error << '\n';
    }
    throw std::invalid_argument(reason.str());
  }
  return config;
}

StereoFrameConfiguration::StereoFrameConfiguration(ParameterSnapshot snapshot)
    : snapshot_(std::move(snapshot)), config_(StereoParameters::read_values(snapshot_.values))
{
}

bool StereoFrameConfiguration::matches(const ParameterSnapshot &snapshot) const
{
  return snapshot_.values == snapshot.values;
}

const StereoParametersConfig &StereoFrameConfiguration::config() const
{
  return config_;
}

std::vector<std::string> validate_parameters(const StereoParametersConfig &config)
{
  auto errors = core::validate_config(config.core);
  for (const auto *topic :
       {&config.topics.left_image, &config.topics.right_image, &config.topics.left_camera_info,
        &config.topics.right_camera_info, &config.topics.depth, &config.topics.camera_info,
        &config.topics.points, &config.topics.diagnostics, &config.topics.timing}) {
    int validation_result = RMW_TOPIC_VALID;
    const auto status = rmw_validate_full_topic_name(topic->c_str(), &validation_result, nullptr);
    if (status != RMW_RET_OK || validation_result != RMW_TOPIC_VALID) {
      errors.emplace_back("topic must be a valid absolute ROS name: " + *topic);
    }
  }
  if (config.sync.queue_size < 1 || config.sync.queue_size > 1000) {
    errors.emplace_back("sync.queue_size must be in [1, 1000]");
  }
  if (!std::isfinite(config.sync.max_interval_s) || config.sync.max_interval_s < 0.0 ||
      config.sync.max_interval_s > 1.0) {
    errors.emplace_back("sync.max_interval_s must be finite and in [0, 1]");
  }
  if (config.qos.depth < 1 || config.qos.depth > 1000) {
    errors.emplace_back("qos.depth must be in [1, 1000]");
  }
  if (!std::isfinite(config.diagnostics.publish_interval_s) ||
      config.diagnostics.publish_interval_s < 0.01 ||
      config.diagnostics.publish_interval_s > 60.0) {
    errors.emplace_back("diagnostics.publish_interval_s must be finite and in [0.01, 60]");
  }
  return errors;
}
} // namespace intball2_stereo::ros
