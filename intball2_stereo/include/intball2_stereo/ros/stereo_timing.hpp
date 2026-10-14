#ifndef INTBALL2_STEREO_ROS_STEREO_TIMING_HPP_
#define INTBALL2_STEREO_ROS_STEREO_TIMING_HPP_

#include <array>
#include <cstdint>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_msgs/msg/header.hpp>

namespace intball2_stereo::ros
{
struct SimulationTiming
{
  bool use_sim_time = false;
  std::int64_t source_stamp_ns = 0;
  std::int64_t worker_start_ns = 0;
  std::int64_t depth_ready_ns = 0;
  std::int64_t publish_start_ns = 0;
  std::int64_t previous_publish_start_ns = 0;
  std::array<std::int64_t, 4> input_source_stamp_ns{};
  std::array<std::int64_t, 4> callback_arrival_sim_ns{};
  std::int64_t bundle_ready_sim_ns = 0;
  bool input_clock_rewound = false;
  bool processing_stages_recorded = false;
  std::array<std::int64_t, 6> processing_stage_sim_ns{};
};
// All samples belong to the ROS simulation clock; wall/steady times are excluded.
diagnostic_msgs::msg::DiagnosticArray
simulation_timing_message(const SimulationTiming &timing,
                          const std_msgs::msg::Header &source_header);
} // namespace intball2_stereo::ros

#endif
