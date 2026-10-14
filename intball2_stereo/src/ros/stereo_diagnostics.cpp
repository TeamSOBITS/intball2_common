#include "intball2_stereo/ros/stereo_diagnostics.hpp"
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

namespace intball2_stereo::ros
{
namespace
{
std::string number(double value)
{
  std::ostringstream text;
  text << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
  return text.str();
}
void value(diagnostic_msgs::msg::DiagnosticStatus &status, const char *key, std::string text)
{
  diagnostic_msgs::msg::KeyValue entry;
  entry.key = key;
  entry.value = std::move(text);
  status.values.push_back(std::move(entry));
}
} // namespace

diagnostic_msgs::msg::DiagnosticArray
diagnostic_messages(const DiagnosticSnapshot &snapshot, const builtin_interfaces::msg::Time &stamp)
{
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = stamp;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "intball2_stereo/processing";
  status.hardware_id = "stereo_cameras";
  if (!snapshot.last_error.empty()) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = snapshot.last_error;
  } else if (snapshot.processed_pairs == 0) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "waiting for first successful frame";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "last frame processed successfully";
  }
  const auto &statistics = snapshot.statistics;
  value(status, "synchronization_wait_s", number(statistics.synchronization_wait_s));
  value(status, "preprocessing_s", number(statistics.preprocessing_s));
  value(status, "matching_s", number(statistics.matching_s));
  value(status, "quality_filter_s", number(statistics.quality_filter_s));
  value(status, "depth_conversion_s", number(statistics.depth_conversion_s));
  value(status, "pointcloud_generation_s", number(statistics.pointcloud_generation_s));
  value(status, "valid_depth_fraction", number(statistics.valid_depth_fraction));
  value(status, "matching_input_pixels", std::to_string(statistics.matching_input_pixels));
  value(status, "full_disparity_eligible_pixels",
        std::to_string(statistics.full_disparity_eligible_pixels));
  value(status, "roi_disparity_eligible_pixels",
        std::to_string(statistics.roi_disparity_eligible_pixels));
  value(status, "roi_additional_search_excluded_pixels",
        std::to_string(statistics.roi_additional_search_excluded_pixels));
  value(status, "worker_dropped_pairs", std::to_string(snapshot.worker_dropped_pairs));
  value(status, "processed_pairs", std::to_string(snapshot.processed_pairs));
  value(status, "failed_pairs", std::to_string(snapshot.failed_pairs));
  value(status, "active_config_revision", std::to_string(snapshot.config_revision));
  value(status, "last_error", snapshot.last_error);
  const auto &sync = snapshot.synchronization;
  value(status, "sync_matched_bundles", std::to_string(sync.matched_bundles));
  value(status, "sync_late_or_duplicate_messages", std::to_string(sync.late_or_duplicate_messages));
  value(status, "sync_queue_overflow_messages", std::to_string(sync.queue_overflow_messages));
  value(status, "sync_interval_dropped_messages", std::to_string(sync.interval_dropped_messages));
  const std::array<const char *, 4> stream_names{"left_image", "right_image", "left_info",
                                                 "right_info"};
  for (std::size_t i = 0; i < sync.streams.size(); ++i) {
    const auto &stream = sync.streams[i];
    const auto prefix = std::string("input_") + stream_names[i] + "_";
    const auto counter = [&](const char *suffix, auto count) {
      const auto key = prefix + suffix;
      value(status, key.c_str(), std::to_string(count));
    };
    counter("received_messages", stream.received_messages);
    counter("late_or_duplicate_messages", stream.late_or_duplicate_messages);
    counter("queue_overflow_messages", stream.queue_overflow_messages);
    counter("interval_dropped_messages", stream.interval_dropped_messages);
    counter("queued_messages", stream.queued_messages);
    counter("source_to_callback_sim_count", stream.source_to_callback_sim_count);
    counter("source_to_callback_sim_sum_ns", stream.source_to_callback_sim_sum_ns);
    counter("source_to_callback_sim_min_ns", stream.source_to_callback_sim_min_ns);
    counter("source_to_callback_sim_max_ns", stream.source_to_callback_sim_max_ns);
    counter("invalid_simulation_clock_samples", stream.invalid_simulation_clock_samples);
    counter("last_source_stamp_ns", stream.last_source_stamp_ns);
    counter("last_callback_sim_ns", stream.last_callback_sim_ns);
  }
  message.status.push_back(std::move(status));
  return message;
}
} // namespace intball2_stereo::ros
