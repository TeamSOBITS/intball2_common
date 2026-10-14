#include "intball2_stereo/ros/stereo_timing.hpp"
#include <algorithm>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

namespace intball2_stereo::ros
{
namespace
{
void seconds(diagnostic_msgs::msg::DiagnosticStatus &status, const char *key, std::int64_t ns)
{
  std::ostringstream text;
  text << std::setprecision(std::numeric_limits<double>::max_digits10)
       << static_cast<double>(ns) / 1e9;
  diagnostic_msgs::msg::KeyValue value;
  value.key = key;
  value.value = text.str();
  status.values.push_back(std::move(value));
}
} // namespace

diagnostic_msgs::msg::DiagnosticArray
simulation_timing_message(const SimulationTiming &timing,
                          const std_msgs::msg::Header &source_header)
{
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header = source_header;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "intball2_stereo/simulation_timing";
  status.hardware_id = "stereo_cameras";
  const auto &arrivals = timing.callback_arrival_sim_ns;
  const auto first_arrival = *std::min_element(arrivals.begin(), arrivals.end());
  const auto last_arrival = *std::max_element(arrivals.begin(), arrivals.end());
  const auto &stages = timing.processing_stage_sim_ns;
  const bool missing_stage_clock =
      timing.processing_stages_recorded &&
      std::any_of(stages.begin(), stages.end(), [](auto sample) { return sample <= 0; });
  const bool backwards_stage_clock =
      timing.processing_stages_recorded &&
      (stages.front() < timing.worker_start_ns || stages.back() > timing.depth_ready_ns ||
       !std::is_sorted(stages.begin(), stages.end()));
  bool future_input = false;
  for (std::size_t i = 0; i < arrivals.size(); ++i) {
    future_input = future_input || timing.input_source_stamp_ns[i] < 0 ||
                   timing.input_source_stamp_ns[i] > arrivals[i];
  }
  if (!timing.use_sim_time) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "simulation timing unavailable: use_sim_time inactive";
  } else if (timing.input_clock_rewound ||
             (timing.previous_publish_start_ns > 0 &&
              timing.worker_start_ns < timing.previous_publish_start_ns) ||
             (timing.worker_start_ns > 0 && timing.depth_ready_ns < timing.worker_start_ns) ||
             (timing.depth_ready_ns > 0 && timing.publish_start_ns < timing.depth_ready_ns)) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "simulation clock moved backwards";
  } else if (timing.worker_start_ns <= 0 || timing.depth_ready_ns <= 0 ||
             timing.publish_start_ns <= 0 || first_arrival <= 0 ||
             timing.bundle_ready_sim_ns <= 0) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "simulation timing unavailable: clock uninitialized";
  } else if (missing_stage_clock) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "simulation timing unavailable: processing stage clock uninitialized";
  } else if (backwards_stage_clock) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "simulation clock moved backwards between processing stages";
  } else if (last_arrival > timing.bundle_ready_sim_ns ||
             timing.bundle_ready_sim_ns > timing.worker_start_ns) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "simulation clock moved backwards between callbacks, bundle and worker";
  } else if (future_input || timing.source_stamp_ns != timing.input_source_stamp_ns[0]) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "source stamp is outside the simulation clock interval";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "simulation timing measured";
    status.values.reserve(timing.processing_stages_recorded ? 36 : 22);
    seconds(status, "source_stamp_sim_s", timing.source_stamp_ns);
    seconds(status, "worker_start_sim_s", timing.worker_start_ns);
    seconds(status, "depth_ready_sim_s", timing.depth_ready_ns);
    seconds(status, "publish_start_sim_s", timing.publish_start_ns);
    seconds(status, "source_to_depth_ready_sim_s", timing.depth_ready_ns - timing.source_stamp_ns);
    seconds(status, "source_to_publish_sim_s", timing.publish_start_ns - timing.source_stamp_ns);
    seconds(status, "processing_sim_s", timing.depth_ready_ns - timing.worker_start_ns);
    const std::array<const char *, 4> source_keys{"left_image_stamp_ns", "right_image_stamp_ns",
                                                  "left_info_stamp_ns", "right_info_stamp_ns"};
    const std::array<const char *, 4> arrival_keys{"left_callback_sim_s", "right_callback_sim_s",
                                                   "left_info_callback_sim_s",
                                                   "right_info_callback_sim_s"};
    for (std::size_t i = 0; i < arrivals.size(); ++i) {
      diagnostic_msgs::msg::KeyValue source_value;
      source_value.key = source_keys[i];
      source_value.value = std::to_string(timing.input_source_stamp_ns[i]);
      status.values.push_back(std::move(source_value));
      seconds(status, arrival_keys[i], arrivals[i]);
    }
    seconds(status, "bundle_ready_sim_s", timing.bundle_ready_sim_ns);
    seconds(status, "source_to_left_callback_sim_s", arrivals[0] - timing.source_stamp_ns);
    seconds(status, "left_callback_to_bundle_sim_s", timing.bundle_ready_sim_ns - arrivals[0]);
    seconds(status, "last_callback_to_bundle_sim_s", timing.bundle_ready_sim_ns - last_arrival);
    seconds(status, "four_stream_arrival_span_sim_s", last_arrival - first_arrival);
    const auto source_range = std::minmax_element(timing.input_source_stamp_ns.begin(),
                                                  timing.input_source_stamp_ns.end());
    seconds(status, "four_stream_source_stamp_span_sim_s",
            *source_range.second - *source_range.first);
    seconds(status, "bundle_to_worker_start_sim_s",
            timing.worker_start_ns - timing.bundle_ready_sim_ns);
    if (timing.processing_stages_recorded) {
      const std::array<const char *, 6> keys{"core_start_sim_s", "preprocessed_sim_s",
                                             "matched_sim_s",    "converted_sim_s",
                                             "refined_sim_s",    "core_finished_sim_s"};
      for (std::size_t i = 0; i < stages.size(); ++i) {
        seconds(status, keys[i], stages[i]);
      }
      seconds(status, "worker_setup_sim_s", stages[0] - timing.worker_start_ns);
      seconds(status, "preprocessing_sim_s", stages[1] - stages[0]);
      seconds(status, "matching_sim_s", stages[2] - stages[1]);
      seconds(status, "depth_conversion_sim_s", stages[3] - stages[2]);
      seconds(status, "refinement_sim_s", stages[4] - stages[3]);
      seconds(status, "core_output_metadata_sim_s", stages[5] - stages[4]);
      seconds(status, "output_copy_sim_s", timing.depth_ready_ns - stages[5]);
      seconds(status, "depth_ready_to_publish_sim_s",
              timing.publish_start_ns - timing.depth_ready_ns);
    }
  }
  message.status.push_back(std::move(status));
  return message;
}
} // namespace intball2_stereo::ros
