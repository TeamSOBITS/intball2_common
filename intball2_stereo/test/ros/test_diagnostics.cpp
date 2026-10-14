#include "intball2_stereo/ros/stereo_diagnostics.hpp"
#include <algorithm>
#include <chrono>
#include <future>
#include <gtest/gtest.h>
#include <thread>

namespace intball2_stereo::test
{
namespace
{
auto image(std::uint32_t stamp)
{
  auto message = std::make_shared<sensor_msgs::msg::Image>();
  message->header.stamp.nanosec = stamp;
  return message;
}
auto info(std::uint32_t stamp)
{
  auto message = std::make_shared<sensor_msgs::msg::CameraInfo>();
  message->header.stamp.nanosec = stamp;
  return message;
}
void feed(ros::StereoInput &input, std::uint32_t stamp)
{
  input.add_image(false, image(stamp));
  input.add_image(true, image(stamp));
  input.add_info(false, info(stamp));
  input.add_info(true, info(stamp));
}
std::string value(const diagnostic_msgs::msg::DiagnosticStatus &status, const std::string &key)
{
  const auto found = std::find_if(status.values.begin(), status.values.end(),
                                  [&](const auto &entry) { return entry.key == key; });
  EXPECT_NE(found, status.values.end());
  return found == status.values.end() ? "" : found->value;
}
TEST(Diagnostics, ReportsEveryStageCounterAndConfigurationRevision)
{
  ros::DiagnosticSnapshot snapshot;
  snapshot.statistics = {0.125, 0.25, 0.5, 0, 0.75, 1, 0.625, 4};
  snapshot.synchronization = {8, 2, 3, 5};
  snapshot.processed_pairs = 7;
  snapshot.failed_pairs = 1;
  snapshot.config_revision = 9;
  snapshot.worker_dropped_pairs = 4;
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 42;
  stamp.nanosec = 123;
  auto message = ros::diagnostic_messages(snapshot, stamp);
  EXPECT_EQ(message.header.stamp, stamp);
  ASSERT_EQ(message.status.size(), 1U);
  const auto &status = message.status.front();
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(std::stod(value(status, "synchronization_wait_s")), 0.125);
  EXPECT_EQ(std::stod(value(status, "preprocessing_s")), 0.25);
  EXPECT_EQ(std::stod(value(status, "matching_s")), 0.5);
  EXPECT_EQ(std::stod(value(status, "quality_filter_s")), 0);
  EXPECT_EQ(std::stod(value(status, "depth_conversion_s")), 0.75);
  EXPECT_EQ(std::stod(value(status, "pointcloud_generation_s")), 1);
  EXPECT_EQ(std::stod(value(status, "valid_depth_fraction")), 0.625);
  EXPECT_EQ(value(status, "processed_pairs"), "7");
  EXPECT_EQ(value(status, "failed_pairs"), "1");
  EXPECT_EQ(value(status, "active_config_revision"), "9");
  EXPECT_EQ(value(status, "worker_dropped_pairs"), "4");
  EXPECT_EQ(value(status, "sync_matched_bundles"), "8");
  EXPECT_EQ(value(status, "sync_late_or_duplicate_messages"), "2");
  EXPECT_EQ(value(status, "sync_queue_overflow_messages"), "3");
  EXPECT_EQ(value(status, "sync_interval_dropped_messages"), "5");
  EXPECT_EQ(value(status, "last_error"), "");
}
TEST(Diagnostics, FailureRecoveryAndNoArtificialQualityThreshold)
{
  ros::DiagnosticSnapshot snapshot;
  auto initial = ros::diagnostic_messages(snapshot, builtin_interfaces::msg::Time{});
  EXPECT_EQ(initial.status.front().level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  snapshot.last_error = "invalid calibration";
  snapshot.failed_pairs = 1;
  auto failed = ros::diagnostic_messages(snapshot, builtin_interfaces::msg::Time{});
  EXPECT_EQ(failed.status.front().level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(failed.status.front().message, "invalid calibration");
  EXPECT_EQ(value(failed.status.front(), "last_error"), "invalid calibration");
  snapshot.last_error.clear();
  snapshot.processed_pairs = 1;
  snapshot.statistics.valid_depth_fraction = 0;
  auto recovered = ros::diagnostic_messages(snapshot, builtin_interfaces::msg::Time{});
  EXPECT_EQ(recovered.status.front().level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(value(recovered.status.front(), "failed_pairs"), "1");
}
TEST(SynchronizationStatistics, DistinguishesAllDiscardReasonsAndSuccessfulBundles)
{
  std::vector<ros::InputBundle> matched;
  ros::StereoInput input({1, 0}, [&](auto bundle) { matched.push_back(std::move(bundle)); });
  input.add_image(false, image(1));
  input.add_image(false, image(2)); // queue overflow
  input.add_image(false, image(2)); // duplicate
  input.add_image(false, image(1)); // late
  input.add_image(true, image(3));
  input.add_info(false, info(3));
  input.add_info(true, info(3)); // stamp 2 falls outside interval
  input.add_image(false, image(3));
  ASSERT_EQ(matched.size(), 1U);
  const auto counters = input.statistics();
  EXPECT_EQ(counters.matched_bundles, 1U);
  EXPECT_EQ(counters.late_or_duplicate_messages, 2U);
  EXPECT_EQ(counters.queue_overflow_messages, 1U);
  EXPECT_EQ(counters.interval_dropped_messages, 1U);
  EXPECT_EQ(counters.streams[0].received_messages, 5U);
  EXPECT_EQ(counters.streams[0].late_or_duplicate_messages, 2U);
  EXPECT_EQ(counters.streams[0].queue_overflow_messages, 1U);
  EXPECT_EQ(counters.streams[0].interval_dropped_messages, 1U);
  EXPECT_EQ(counters.streams[0].queued_messages, 0U);
  for (std::size_t i = 1; i < 4; ++i) {
    EXPECT_EQ(counters.streams[i].received_messages, 1U);
    EXPECT_EQ(counters.streams[i].queued_messages, 0U);
  }
  EXPECT_GE(matched.front().synchronization_wait_s, 0);
}
TEST(SynchronizationStatistics, WaitUsesArrivalClockAndSnapshotsAreThreadSafe)
{
  std::promise<void> arrived, continue_arrival;
  auto continuation = continue_arrival.get_future();
  double wait = 0;
  ros::StereoInput input({2, 0}, [&](auto bundle) { wait = bundle.synchronization_wait_s; });
  const auto start = std::chrono::steady_clock::now();
  auto producer = std::async(std::launch::async, [&] {
    input.add_image(false, image(999));
    arrived.set_value();
    continuation.wait();
    input.add_image(true, image(999));
    input.add_info(false, info(999));
    input.add_info(true, info(999));
  });
  auto ready = arrived.get_future();
  const auto arrival_status = ready.wait_for(std::chrono::seconds(5));
  EXPECT_EQ(arrival_status, std::future_status::ready);
  if (arrival_status != std::future_status::ready) {
    continue_arrival.set_value();
    producer.get();
    return;
  }
  EXPECT_EQ(input.statistics().matched_bundles, 0U);
  continue_arrival.set_value();
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (producer.wait_for(std::chrono::seconds(0)) != std::future_status::ready &&
         std::chrono::steady_clock::now() < deadline) {
    EXPECT_LE(input.statistics().matched_bundles, 1U);
    std::this_thread::yield();
  }
  ASSERT_EQ(producer.wait_for(std::chrono::seconds(0)), std::future_status::ready)
      << "producer did not finish within five seconds";
  producer.get();
  EXPECT_EQ(input.statistics().matched_bundles, 1U);
  EXPECT_GE(wait, 0);
  EXPECT_LE(wait, std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count());
  feed(input, 1000);
  EXPECT_EQ(input.statistics().matched_bundles, 2U);
}
TEST(SynchronizationStatistics, SimulationSamplesStayWithTheirSelectedMessages)
{
  std::vector<ros::InputBundle> outputs;
  const std::array<std::int64_t, 6> samples{10, 20, 30, 40, 50, 60};
  std::size_t next = 0;
  ros::StereoInput input(
      {1, 0}, [&](auto bundle) { outputs.push_back(std::move(bundle)); },
      [&] { return samples.at(next++); });
  input.add_image(false, image(1));
  input.add_image(false, image(2));
  input.add_image(true, image(2));
  input.add_info(false, info(2));
  input.add_info(true, info(2));
  ASSERT_EQ(outputs.size(), 1U);
  const auto &bundle = outputs.front();
  EXPECT_EQ(bundle.source_stamp_ns, (std::array<std::int64_t, 4>{2, 2, 2, 2}));
  EXPECT_EQ(bundle.callback_arrival_sim_ns, (std::array<std::int64_t, 4>{20, 30, 40, 50}));
  EXPECT_EQ(bundle.bundle_ready_sim_ns, 60);
  EXPECT_FALSE(bundle.simulation_clock_rewound);
  EXPECT_EQ(input.statistics().queue_overflow_messages, 1U);
}
TEST(SynchronizationStatistics, SimulationRewindIsRememberedAfterClockRecovers)
{
  std::vector<ros::InputBundle> outputs;
  const std::array<std::int64_t, 5> samples{100, 110, 105, 120, 130};
  std::size_t next = 0;
  ros::StereoInput input(
      {2, 0}, [&](auto bundle) { outputs.push_back(std::move(bundle)); },
      [&] { return samples.at(next++); });
  feed(input, 10);
  ASSERT_EQ(outputs.size(), 1U);
  EXPECT_TRUE(outputs.front().simulation_clock_rewound);
  EXPECT_EQ(outputs.front().bundle_ready_sim_ns, 130);
  EXPECT_EQ(outputs.front().callback_arrival_sim_ns,
            (std::array<std::int64_t, 4>{100, 110, 105, 120}));
}

TEST(SynchronizationStatistics, SimArrivalStatisticsIncludeInputsDiscardedBeforeBundleFormation)
{
  std::int64_t now = 110;
  ros::StereoInput input(
      {1, 0}, [](auto) {}, [&] { return now; });
  input.add_image(false, image(100));
  now = 140;
  input.add_image(false, image(120));
  now = 150;
  input.add_image(false, image(120));
  auto snapshot = input.statistics();
  const auto &stream = snapshot.streams[0];
  EXPECT_EQ(snapshot.matched_bundles, 0U);
  EXPECT_EQ(stream.received_messages, 3U);
  EXPECT_EQ(stream.queue_overflow_messages, 1U);
  EXPECT_EQ(stream.late_or_duplicate_messages, 1U);
  EXPECT_EQ(stream.queued_messages, 1U);
  EXPECT_EQ(stream.source_to_callback_sim_count, 3U);
  EXPECT_EQ(stream.source_to_callback_sim_sum_ns, 60U);
  EXPECT_EQ(stream.source_to_callback_sim_min_ns, 10U);
  EXPECT_EQ(stream.source_to_callback_sim_max_ns, 30U);
  EXPECT_EQ(stream.last_source_stamp_ns, 120);
  EXPECT_EQ(stream.last_callback_sim_ns, 150);
  EXPECT_EQ(stream.invalid_simulation_clock_samples, 0U);
  ros::DiagnosticSnapshot diagnostic;
  diagnostic.synchronization = snapshot;
  const auto status =
      ros::diagnostic_messages(diagnostic, builtin_interfaces::msg::Time{}).status.front();
  EXPECT_EQ(value(status, "input_left_image_received_messages"), "3");
  EXPECT_EQ(value(status, "input_left_image_queue_overflow_messages"), "1");
  EXPECT_EQ(value(status, "input_left_image_source_to_callback_sim_sum_ns"), "60");
  EXPECT_EQ(value(status, "input_right_image_received_messages"), "0");
}

TEST(SynchronizationStatistics, InvalidArrivalClocksAreCountedWithoutInventedDurations)
{
  std::int64_t now = 0;
  ros::StereoInput input(
      {5, 0}, [](auto) {}, [&] { return now; });
  input.add_image(false, image(10));
  now = 100;
  input.add_image(false, image(101));
  now = 90;
  input.add_image(false, image(80));
  now = 200;
  input.add_image(false, image(150));
  const auto snapshot = input.statistics();
  const auto &stream = snapshot.streams[0];
  EXPECT_EQ(stream.received_messages, 4U);
  EXPECT_EQ(stream.invalid_simulation_clock_samples, 3U);
  EXPECT_EQ(stream.source_to_callback_sim_count, 1U);
  EXPECT_EQ(stream.source_to_callback_sim_sum_ns, 50U);
  EXPECT_EQ(stream.source_to_callback_sim_min_ns, 50U);
  EXPECT_EQ(stream.source_to_callback_sim_max_ns, 50U);
}
} // namespace
} // namespace intball2_stereo::test
