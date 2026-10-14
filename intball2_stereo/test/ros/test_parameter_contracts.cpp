#include "intball2_stereo/ros/stereo_parameters.hpp"

#include <iostream>
#include <limits>
#include <memory>
#include <set>
#include <stdexcept>

#include <rclcpp/parameter_map.hpp>

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

void rejects(const std::vector<rclcpp::Parameter> &values, const char *reason)
{
  bool rejected = false;
  try {
    intball2_stereo::ros::StereoParameters::read_values(values);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  check(rejected, reason);
}
} // namespace

int main()
{
  // Intentionally no rclcpp::init(), Node, executor, subscriptions or publishers.
  using intball2_stereo::core::PointsLayout;
  using intball2_stereo::ros::StereoParameters;
  using intball2_stereo::ros::UpdatePolicy;
  try {
    const auto &specs = StereoParameters::specifications();
    const auto defaults = StereoParameters::read_values({});
    check(defaults.core.processing.downsample == 4, "wrong typed downsample default");
    std::set<std::string> names;
    std::vector<rclcpp::Parameter> all_defaults;
    for (const auto &spec : specs) {
      check(names.insert(spec.name).second, "duplicate registry entry");
      check(!spec.description.empty(), "missing parameter description");
      all_defaults.push_back(
          std::visit([&](const auto &value) { return rclcpp::Parameter(spec.name, value); },
                     spec.default_value));
      if (spec.name == "points.publish_points" || spec.name == "processing.downsample" ||
          spec.name.rfind("refinement.", 0) == 0) {
        check(spec.update_policy == UpdatePolicy::kFutureFrameBoundary,
              "wrong frame boundary classification");
      }
      if (spec.name.rfind("topics.", 0) == 0 || spec.name.rfind("sync.", 0) == 0 ||
          spec.name.rfind("qos.", 0) == 0 || spec.name == "processing.opencv_threads") {
        check(spec.update_policy == UpdatePolicy::kStartupOnly, "wrong startup classification");
      }
    }
    StereoParameters::read_values(all_defaults);
    // Own the parser allocation explicitly; Humble's convenience YAML loader
    // in this runtime leaves its C parser allocation alive after returning.
    const std::unique_ptr<rcl_params_t, decltype(&rcl_yaml_node_struct_fini)> parsed(
        rcl_yaml_node_struct_init(rcutils_get_default_allocator()), &rcl_yaml_node_struct_fini);
    check(static_cast<bool>(parsed), "YAML parser allocation failed");
    check(rcl_parse_yaml_file(STEREO_PARAMS_YAML, parsed.get()), "YAML parse failed");
    const auto yaml = rclcpp::parameter_map_from(parsed.get());
    const auto found = yaml.find("/stereo_node");
    check(found != yaml.end(), "YAML node selector missing");
    check(found->second.size() == specs.size(), "YAML and registry parameter counts differ");
    StereoParameters::read_values(found->second);
    for (const auto &expected : all_defaults) {
      bool matches = false;
      for (const auto &actual : found->second) {
        if (actual.get_name() == expected.get_name()) {
          matches = actual.get_parameter_value() == expected.get_parameter_value();
        }
      }
      check(matches, "YAML default differs from typed registry default");
    }

    rejects({{"matcher.min_disparity_px", 1.0}}, "floating-point integer parameter accepted");
    rejects({{"points.publish_points", "false"}}, "string boolean accepted");
    rejects({{"depth.min_depth_m", 1}}, "integer double accepted");
    rejects({{"processing.downsample", std::numeric_limits<std::int64_t>::max()}},
            "integer overflow accepted");
    rejects({{"processing.downsample", std::numeric_limits<std::int64_t>::min()}},
            "integer underflow accepted");
    rejects({{"depth.min_depth_m", std::numeric_limits<double>::quiet_NaN()}}, "NaN accepted");
    rejects({{"depth.max_depth_m", std::numeric_limits<double>::infinity()}}, "Inf accepted");
    rejects({{"camera.calibration_mode", "automatic"}}, "invalid calibration mode accepted");
    rejects({{"camera.input_geometry", "unknown"}}, "invalid input geometry accepted");
    rejects({{"points.layout", "unordered"}}, "invalid layout accepted");
    rejects({{"matcher.disparity_units", "original_pixels"}}, "ambiguous disparity units accepted");
    rejects({{"matcher.disparity_units", "fixed_point_16"}},
            "fixed-point disparity units accepted");
    rejects({{"points.stride", 2}}, "organized stride accepted while publishing disabled");
    rejects({{"not_a_parameter", 1}}, "unknown parameter accepted");
    rejects({{"points.stride", 1}, {"points.stride", 1}}, "duplicate parameter accepted");
    rejects({{"sync.max_interval_s", -0.1}}, "negative sync interval accepted");
    rejects({{"sync.max_interval_s", std::numeric_limits<double>::infinity()}},
            "infinite sync interval accepted");
    rejects({{"qos.depth", 0}}, "zero QoS depth accepted");
    for (const auto *bad : {"", "/", "/bad//name", "/1name", "/bad name", "relative"}) {
      rejects({{"topics.left_image", bad}}, "invalid absolute topic accepted");
    }
    const auto compact = StereoParameters::read_values(
        {{"points.layout", "compact"}, {"points.stride", 4}, {"points.publish_points", true}});
    check(compact.core.points.layout == PointsLayout::kCompact && compact.core.points.stride == 4 &&
              compact.core.points.publish_points,
          "typed compact parameters not applied together");
    std::cout << checks << " parameter contract checks passed without starting ROS\n";
  } catch (const std::exception &error) {
    std::cerr << "parameter contract failure: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
