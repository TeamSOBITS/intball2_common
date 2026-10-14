#ifndef INTBALL2_STEREO_ROS_STEREO_NODE_HPP_
#define INTBALL2_STEREO_ROS_STEREO_NODE_HPP_
#include "intball2_stereo/core/stereo_processor.hpp"
#include "intball2_stereo/ros/stereo_diagnostics.hpp"
#include "intball2_stereo/ros/stereo_input.hpp"
#include "intball2_stereo/ros/stereo_worker.hpp"
#include <rclcpp/rclcpp.hpp>
namespace intball2_stereo::ros
{
class StereoNode : public rclcpp::Node
{
public:
  explicit StereoNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~StereoNode() override;

private:
  void publish_diagnostics();
  std::unique_ptr<core::StereoProcessor> processor_;
  std::int64_t previous_timing_publish_ns_ = 0;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr timing_publisher_;
  std::unique_ptr<StereoFrameConfiguration> active_configuration_;
  std::vector<std::string> parameter_names_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  std::mutex diagnostics_mutex_;
  DiagnosticSnapshot diagnostics_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_publisher_;
  std::unique_ptr<StereoWorker> worker_;
  std::unique_ptr<StereoInput> input_;
  std::array<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr, 2> images_;
  std::array<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr, 2> infos_;
};
} // namespace intball2_stereo::ros
#endif
