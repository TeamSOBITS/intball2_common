#include "intball2_stereo/ros/stereo_node.hpp"
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<intball2_stereo::ros::StereoNode>());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_node"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
