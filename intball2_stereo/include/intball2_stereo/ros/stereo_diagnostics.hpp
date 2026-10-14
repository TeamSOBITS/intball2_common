#ifndef INTBALL2_STEREO_ROS_STEREO_DIAGNOSTICS_HPP_
#define INTBALL2_STEREO_ROS_STEREO_DIAGNOSTICS_HPP_

#include "intball2_stereo/ros/stereo_input.hpp"
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <string>

namespace intball2_stereo::ros
{
struct DiagnosticSnapshot
{
  core::ProcessingStatistics statistics;
  SynchronizationStatistics synchronization;
  std::uint64_t processed_pairs = 0;
  std::uint64_t failed_pairs = 0;
  std::uint64_t config_revision = 0;
  std::uint64_t worker_dropped_pairs = 0;
  std::string last_error;
};
// Caller supplies the ROS clock stamp; processing durations use steady_clock.
diagnostic_msgs::msg::DiagnosticArray
diagnostic_messages(const DiagnosticSnapshot &snapshot, const builtin_interfaces::msg::Time &stamp);
} // namespace intball2_stereo::ros

#endif
