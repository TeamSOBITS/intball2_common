#include "intball2_stereo/ros/stereo_input.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
namespace intball2_stereo::ros
{
StereoInput::StereoInput(SyncConfig config, Callback callback, SimulationClock simulation_clock)
    : callback_(std::move(callback)), simulation_clock_(std::move(simulation_clock))
{
  if (config.queue_size <= 0 || !std::isfinite(config.max_interval_s) ||
      config.max_interval_s < 0 || config.max_interval_s > 1.0 || !callback_) {
    throw std::invalid_argument("invalid synchronization configuration/callback");
  }
  queue_size_ = static_cast<std::size_t>(config.queue_size);
  max_interval_ns_ = static_cast<std::int64_t>(std::llround(config.max_interval_s * 1e9));
}
void StereoInput::add_image(bool right, sensor_msgs::msg::Image::ConstSharedPtr image)
{
  if (!image) {
    throw std::invalid_argument("null image");
  }
  const auto arrival = simulation_clock_sample();
  const auto stamp = stamp_nanoseconds(image->header.stamp);
  add(right ? 1 : 0, {stamp,
                      std::chrono::steady_clock::now(),
                      arrival.first,
                      arrival.second,
                      std::move(image),
                      {}});
}
void StereoInput::add_info(bool right, sensor_msgs::msg::CameraInfo::ConstSharedPtr info)
{
  if (!info) {
    throw std::invalid_argument("null CameraInfo");
  }
  const auto arrival = simulation_clock_sample();
  const auto stamp = stamp_nanoseconds(info->header.stamp);
  add(right ? 3 : 2, {stamp,
                      std::chrono::steady_clock::now(),
                      arrival.first,
                      arrival.second,
                      {},
                      std::move(info)});
}
std::pair<std::int64_t, bool> StereoInput::simulation_clock_sample()
{
  // Compare clock samples in sampling order, independent of queue-lock acquisition.
  std::lock_guard<std::mutex> lock(clock_mutex_);
  const auto now = simulation_clock_ ? simulation_clock_() : 0;
  const bool rewound = last_clock_sim_ns_ > 0 && now < last_clock_sim_ns_;
  last_clock_sim_ns_ = now;
  return {now, rewound};
}
SynchronizationStatistics StereoInput::statistics() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto snapshot = statistics_;
  for (std::size_t i = 0; i < queues_.size(); ++i) {
    snapshot.streams[i].queued_messages = queues_[i].size();
  }
  return snapshot;
}
void StereoInput::add(std::size_t stream, Entry entry)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto &stream_statistics = statistics_.streams[stream];
  ++stream_statistics.received_messages;
  stream_statistics.last_source_stamp_ns = entry.stamp;
  stream_statistics.last_callback_sim_ns = entry.arrival_sim_ns;
  if (entry.simulation_clock_rewound || entry.arrival_sim_ns <= 0 || entry.stamp < 0 ||
      entry.stamp > entry.arrival_sim_ns) {
    ++stream_statistics.invalid_simulation_clock_samples;
  } else {
    const auto delay = static_cast<std::uint64_t>(entry.arrival_sim_ns - entry.stamp);
    if (stream_statistics.source_to_callback_sim_count == 0) {
      stream_statistics.source_to_callback_sim_min_ns = delay;
    } else {
      stream_statistics.source_to_callback_sim_min_ns =
          std::min(stream_statistics.source_to_callback_sim_min_ns, delay);
    }
    stream_statistics.source_to_callback_sim_max_ns =
        std::max(stream_statistics.source_to_callback_sim_max_ns, delay);
    stream_statistics.source_to_callback_sim_sum_ns += delay;
    ++stream_statistics.source_to_callback_sim_count;
  }
  if (seen_[stream] && entry.stamp <= last_stamp_[stream]) {
    ++statistics_.late_or_duplicate_messages;
    ++stream_statistics.late_or_duplicate_messages;
    return;
  }
  seen_[stream] = true;
  last_stamp_[stream] = entry.stamp;
  auto &queue = queues_[stream];
  queue.push_back(std::move(entry));
  if (queue.size() > queue_size_) {
    queue.pop_front();
    ++statistics_.queue_overflow_messages;
    ++stream_statistics.queue_overflow_messages;
  }
  while (std::all_of(queues_.begin(), queues_.end(), [](const auto &q) { return !q.empty(); })) {
    std::size_t oldest = 0;
    std::int64_t newest = queues_[0].front().stamp;
    for (std::size_t i = 1; i < queues_.size(); ++i) {
      if (queues_[i].front().stamp < queues_[oldest].front().stamp) {
        oldest = i;
      }
      newest = std::max(newest, queues_[i].front().stamp);
    }
    if (newest - queues_[oldest].front().stamp > max_interval_ns_) {
      queues_[oldest].pop_front();
      ++statistics_.interval_dropped_messages;
      ++statistics_.streams[oldest].interval_dropped_messages;
      continue;
    }
    InputBundle bundle{queues_[0].front().image, queues_[1].front().image, queues_[2].front().info,
                       queues_[3].front().info};
    const auto ready = simulation_clock_sample();
    bundle.bundle_ready_sim_ns = ready.first;
    bundle.simulation_clock_rewound = ready.second;
    for (std::size_t i = 0; i < queues_.size(); ++i) {
      const auto &candidate = queues_[i].front();
      bundle.source_stamp_ns[i] = candidate.stamp;
      bundle.callback_arrival_sim_ns[i] = candidate.arrival_sim_ns;
      bundle.simulation_clock_rewound =
          bundle.simulation_clock_rewound || candidate.simulation_clock_rewound;
    }
    auto first_arrival = queues_[0].front().arrival;
    for (const auto &q : queues_) {
      first_arrival = std::min(first_arrival, q.front().arrival);
    }
    bundle.synchronization_wait_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - first_arrival).count();
    ++statistics_.matched_bundles;
    for (auto &q : queues_) {
      q.pop_front();
    }
    callback_(std::move(bundle));
  }
}
} // namespace intball2_stereo::ros
