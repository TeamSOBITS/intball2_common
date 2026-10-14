#ifndef INTBALL2_STEREO_ROS_STEREO_INPUT_HPP_
#define INTBALL2_STEREO_ROS_STEREO_INPUT_HPP_
#include "intball2_stereo/ros/stereo_messages.hpp"
#include "intball2_stereo/ros/stereo_parameters.hpp"
#include <array>
#include <chrono>
#include <deque>
#include <functional>
#include <mutex>
#include <utility>
namespace intball2_stereo::ros
{
struct InputStreamStatistics
{
  std::uint64_t received_messages = 0;
  std::uint64_t late_or_duplicate_messages = 0;
  std::uint64_t queue_overflow_messages = 0;
  std::uint64_t interval_dropped_messages = 0;
  std::uint64_t queued_messages = 0;
  std::uint64_t source_to_callback_sim_count = 0;
  std::uint64_t source_to_callback_sim_sum_ns = 0;
  std::uint64_t source_to_callback_sim_min_ns = 0;
  std::uint64_t source_to_callback_sim_max_ns = 0;
  std::uint64_t invalid_simulation_clock_samples = 0;
  std::int64_t last_source_stamp_ns = 0;
  std::int64_t last_callback_sim_ns = 0;
};
struct SynchronizationStatistics
{
  std::uint64_t matched_bundles = 0;
  std::uint64_t late_or_duplicate_messages = 0;
  std::uint64_t queue_overflow_messages = 0;
  std::uint64_t interval_dropped_messages = 0;
  // left image, right image, left CameraInfo, right CameraInfo; before sync discard.
  std::array<InputStreamStatistics, 4> streams{};
};
// Bounded four-stream approximate synchronization, using one camera clock.
// Ordered stamps per stream are required; old/duplicate arrivals are discarded.
// The callback only hands messages to the worker and must not reenter this object.
class StereoInput
{
public:
  using Callback = std::function<void(InputBundle)>;
  using SimulationClock = std::function<std::int64_t()>;
  StereoInput(SyncConfig config, Callback callback, SimulationClock simulation_clock = {});
  void add_image(bool right, sensor_msgs::msg::Image::ConstSharedPtr image);
  void add_info(bool right, sensor_msgs::msg::CameraInfo::ConstSharedPtr info);
  SynchronizationStatistics statistics() const;

private:
  struct Entry
  {
    std::int64_t stamp;
    std::chrono::steady_clock::time_point arrival;
    std::int64_t arrival_sim_ns;
    bool simulation_clock_rewound;
    sensor_msgs::msg::Image::ConstSharedPtr image;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr info;
  };
  void add(std::size_t stream, Entry entry);
  std::pair<std::int64_t, bool> simulation_clock_sample();
  std::size_t queue_size_;
  std::int64_t max_interval_ns_;
  Callback callback_;
  SimulationClock simulation_clock_;
  std::mutex clock_mutex_;
  std::int64_t last_clock_sim_ns_ = 0;
  mutable std::mutex mutex_;
  SynchronizationStatistics statistics_;
  std::array<std::deque<Entry>, 4> queues_;
  std::array<std::int64_t, 4> last_stamp_{};
  std::array<bool, 4> seen_{};
};
} // namespace intball2_stereo::ros
#endif
