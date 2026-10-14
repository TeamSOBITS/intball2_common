#include "../core/stereo_test_fixture.hpp"
#include "intball2_stereo/core/stereo_processor.hpp"
#include "intball2_stereo/ros/stereo_timing.hpp"
#include <algorithm>
#include <gtest/gtest.h>

namespace intball2_stereo::test
{
namespace
{
std::string value(const diagnostic_msgs::msg::DiagnosticStatus &status, const char *key)
{
  const auto found = std::find_if(status.values.begin(), status.values.end(),
                                  [&](const auto &entry) { return entry.key == key; });
  EXPECT_NE(found, status.values.end());
  return found == status.values.end() ? "" : found->value;
}
ros::SimulationTiming with_input(ros::SimulationTiming timing)
{
  timing.input_source_stamp_ns.fill(timing.source_stamp_ns);
  timing.callback_arrival_sim_ns.fill(timing.worker_start_ns);
  timing.bundle_ready_sim_ns = timing.worker_start_ns;
  return timing;
}
TEST(SimulationTiming, SourceHeaderAndSimClockDurationsArePreserved)
{
  ros::SimulationTiming timing{true, 1000000000, 1250000000, 1500000000, 1750000000, 900000000};
  timing.input_source_stamp_ns = {1000000000, 1010000000, 1000000000, 1010000000};
  timing.callback_arrival_sim_ns = {1100000000, 1150000000, 1175000000, 1200000000};
  timing.bundle_ready_sim_ns = 1225000000;
  std_msgs::msg::Header header;
  header.stamp.sec = 1;
  header.frame_id = "cameraL_link";
  auto message = ros::simulation_timing_message(timing, header);
  EXPECT_EQ(message.header, header);
  ASSERT_EQ(message.status.size(), 1U);
  const auto &status = message.status.front();
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(status.values.size(), 22U);
  EXPECT_DOUBLE_EQ(std::stod(value(status, "source_stamp_sim_s")), 1);
  EXPECT_DOUBLE_EQ(std::stod(value(status, "worker_start_sim_s")), 1.25);
  EXPECT_DOUBLE_EQ(std::stod(value(status, "depth_ready_sim_s")), 1.5);
  EXPECT_DOUBLE_EQ(std::stod(value(status, "publish_start_sim_s")), 1.75);
  EXPECT_DOUBLE_EQ(std::stod(value(status, "source_to_depth_ready_sim_s")), 0.5);
  EXPECT_DOUBLE_EQ(std::stod(value(status, "source_to_publish_sim_s")), 0.75);
  EXPECT_DOUBLE_EQ(std::stod(value(status, "processing_sim_s")), 0.25);
  EXPECT_EQ(value(status, "left_image_stamp_ns"), "1000000000");
  EXPECT_EQ(value(status, "right_image_stamp_ns"), "1010000000");
  EXPECT_EQ(value(status, "left_info_stamp_ns"), "1000000000");
  EXPECT_EQ(value(status, "right_info_stamp_ns"), "1010000000");
  EXPECT_DOUBLE_EQ(std::stod(value(status, "left_callback_sim_s")), 1.1);
  EXPECT_DOUBLE_EQ(std::stod(value(status, "right_callback_sim_s")), 1.15);
  EXPECT_DOUBLE_EQ(std::stod(value(status, "left_info_callback_sim_s")), 1.175);
  EXPECT_DOUBLE_EQ(std::stod(value(status, "right_info_callback_sim_s")), 1.2);
  EXPECT_DOUBLE_EQ(std::stod(value(status, "bundle_ready_sim_s")), 1.225);
  EXPECT_DOUBLE_EQ(std::stod(value(status, "source_to_left_callback_sim_s")), 0.1);
  EXPECT_DOUBLE_EQ(std::stod(value(status, "left_callback_to_bundle_sim_s")), 0.125);
  EXPECT_DOUBLE_EQ(std::stod(value(status, "last_callback_to_bundle_sim_s")), 0.025);
  EXPECT_DOUBLE_EQ(std::stod(value(status, "four_stream_arrival_span_sim_s")), 0.1);
  EXPECT_DOUBLE_EQ(std::stod(value(status, "four_stream_source_stamp_span_sim_s")), 0.01);
  EXPECT_DOUBLE_EQ(std::stod(value(status, "bundle_to_worker_start_sim_s")), 0.025);
  EXPECT_DOUBLE_EQ(std::stod(value(status, "source_to_depth_ready_sim_s")),
                   std::stod(value(status, "source_to_left_callback_sim_s")) +
                       std::stod(value(status, "left_callback_to_bundle_sim_s")) +
                       std::stod(value(status, "bundle_to_worker_start_sim_s")) +
                       std::stod(value(status, "processing_sim_s")));
}
TEST(SimulationTiming, FrozenSimulationClockHasZeroDurations)
{
  const auto message = ros::simulation_timing_message(with_input({true, 100, 100, 100, 100, 100}),
                                                      std_msgs::msg::Header{});
  const auto &status = message.status.front();
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_DOUBLE_EQ(std::stod(value(status, "processing_sim_s")), 0);
  EXPECT_DOUBLE_EQ(std::stod(value(status, "source_to_publish_sim_s")), 0);
}
TEST(SimulationTiming, DisabledOrUninitializedClockHasNoInventedNumbers)
{
  for (const auto &timing : {ros::SimulationTiming{false, 10, 20, 30, 40, 0},
                             ros::SimulationTiming{true, 0, 0, 0, 0, 0}}) {
    const auto message =
        ros::simulation_timing_message(with_input(timing), std_msgs::msg::Header{});
    EXPECT_EQ(message.status.front().level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
    EXPECT_TRUE(message.status.front().values.empty());
  }
}
TEST(SimulationTiming, BackwardsClockAndFutureSourceRejectNumbers)
{
  for (const auto &timing :
       {ros::SimulationTiming{true, 10, 20, 19, 40, 0},
        ros::SimulationTiming{true, 10, 20, 30, 29, 0},
        ros::SimulationTiming{true, 10, 20, 30, 40, 21},
        ros::SimulationTiming{true, 21, 20, 30, 40, 0},
        ros::SimulationTiming{true, -1, 20, 30, 40, 0}, ros::SimulationTiming{true, 0, 0, 0, 0, 40},
        ros::SimulationTiming{true, 10, 20, 0, 0, 0}}) {
    const auto message =
        ros::simulation_timing_message(with_input(timing), std_msgs::msg::Header{});
    EXPECT_EQ(message.status.front().level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
    EXPECT_TRUE(message.status.front().values.empty());
  }
}
TEST(SimulationTiming, SourceStampsKeepIntegerPrecisionBeyondDoubleMantissa)
{
  constexpr std::int64_t kSource = 9007199254740993;
  const auto timing = with_input({true, kSource, kSource + 10, kSource + 20, kSource + 30, 0});
  const auto message = ros::simulation_timing_message(timing, std_msgs::msg::Header{});
  EXPECT_EQ(message.status.front().level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(value(message.status.front(), "left_image_stamp_ns"), "9007199254740993");
  EXPECT_EQ(value(message.status.front(), "right_image_stamp_ns"), "9007199254740993");
}
TEST(SimulationTiming, MissingArrivalAndBundleClocksHaveNoNumbers)
{
  auto timing = with_input({true, 10, 30, 40, 50, 0});
  timing.callback_arrival_sim_ns[2] = 0;
  auto message = ros::simulation_timing_message(timing, std_msgs::msg::Header{});
  EXPECT_EQ(message.status.front().level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_TRUE(message.status.front().values.empty());
  timing = with_input({true, 10, 30, 40, 50, 0});
  timing.bundle_ready_sim_ns = 0;
  message = ros::simulation_timing_message(timing, std_msgs::msg::Header{});
  EXPECT_EQ(message.status.front().level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_TRUE(message.status.front().values.empty());
}
TEST(SimulationTiming, ArrivalBundleAndSourceInconsistencyRejectNumbers)
{
  const auto baseline = with_input({true, 10, 30, 40, 50, 0});
  auto timing = baseline;
  timing.callback_arrival_sim_ns[2] = 31;
  auto message = ros::simulation_timing_message(timing, std_msgs::msg::Header{});
  EXPECT_EQ(message.status.front().level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_TRUE(message.status.front().values.empty());
  timing = baseline;
  timing.bundle_ready_sim_ns = 31;
  message = ros::simulation_timing_message(timing, std_msgs::msg::Header{});
  EXPECT_EQ(message.status.front().level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_TRUE(message.status.front().values.empty());
  timing = baseline;
  timing.input_source_stamp_ns[1] = 31;
  message = ros::simulation_timing_message(timing, std_msgs::msg::Header{});
  EXPECT_EQ(message.status.front().level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_TRUE(message.status.front().values.empty());
  timing = baseline;
  timing.input_clock_rewound = true;
  message = ros::simulation_timing_message(timing, std_msgs::msg::Header{});
  EXPECT_EQ(message.status.front().level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_TRUE(message.status.front().values.empty());
}

TEST(SimulationTiming, CoreStagesAndWrapperCopyFormOneConsistentSimClockInterval)
{
  auto timing = with_input({true, 10, 30, 100, 110, 0});
  timing.processing_stages_recorded = true;
  timing.processing_stage_sim_ns = {40, 50, 80, 90, 93, 95};
  const auto status =
      ros::simulation_timing_message(timing, std_msgs::msg::Header{}).status.front();
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(status.values.size(), 36U);
  double sum = 0;
  for (const auto *key :
       {"worker_setup_sim_s", "preprocessing_sim_s", "matching_sim_s", "depth_conversion_sim_s",
        "refinement_sim_s", "core_output_metadata_sim_s", "output_copy_sim_s"}) {
    sum += std::stod(value(status, key));
  }
  EXPECT_NEAR(sum, std::stod(value(status, "processing_sim_s")), 1e-20);
  EXPECT_DOUBLE_EQ(std::stod(value(status, "matching_sim_s")), 30e-9);
  EXPECT_DOUBLE_EQ(std::stod(value(status, "output_copy_sim_s")), 5e-9);
  EXPECT_DOUBLE_EQ(std::stod(value(status, "depth_ready_to_publish_sim_s")), 10e-9);
}

TEST(SimulationTiming, MissingOrBackwardsCoreStageClockRejectsStageDurations)
{
  auto baseline = with_input({true, 10, 30, 100, 110, 0});
  baseline.processing_stages_recorded = true;
  baseline.processing_stage_sim_ns = {40, 50, 80, 90, 93, 95};
  for (const auto stages : {std::array<std::int64_t, 6>{40, 0, 80, 90, 93, 95},
                            std::array<std::int64_t, 6>{29, 50, 80, 90, 93, 95},
                            std::array<std::int64_t, 6>{40, 50, 49, 90, 93, 95},
                            std::array<std::int64_t, 6>{40, 50, 80, 90, 93, 101},
                            std::array<std::int64_t, 6>{40, 50, 80, 90, 89, 95}}) {
    auto timing = baseline;
    timing.processing_stage_sim_ns = stages;
    const auto status =
        ros::simulation_timing_message(timing, std_msgs::msg::Header{}).status.front();
    EXPECT_NE(status.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
    EXPECT_TRUE(status.values.empty());
  }
}

TEST(ProcessingStageObserver, RecordsBoundariesWithoutChangingDepthOrInventingCompletionOnFailure)
{
  core::StereoConfig config;
  config.processing.downsample = 1;
  core::StereoProcessor processor(config);
  cv::Mat pixels(200, 200, CV_8UC1);
  cv::RNG random(20261010);
  random.fill(pixels, cv::RNG::UNIFORM, 0, 256);
  cv::Mat right_pixels(200, 200, CV_8UC1, cv::Scalar(0));
  pixels(cv::Rect(0, 0, 192, 200)).copyTo(right_pixels(cv::Rect(8, 0, 192, 200)));
  const core::ImagePair pair{frame(pixels, calibration()),
                             frame(right_pixels, calibration(), "right_optical")};
  const auto expected = processor.process(pair).depth_m.clone();
  std::vector<core::ProcessingStage> stages;
  const auto &result = processor.process(pair, [&](auto stage) { stages.push_back(stage); });
  EXPECT_EQ(stages,
            (std::vector<core::ProcessingStage>{core::ProcessingStage::kPreprocessingStarted,
                                                core::ProcessingStage::kPreprocessingFinished,
                                                core::ProcessingStage::kMatchingFinished,
                                                core::ProcessingStage::kDepthConversionFinished,
                                                core::ProcessingStage::kRefinementFinished,
                                                core::ProcessingStage::kOutputPrepared}));
  EXPECT_EQ(cv::countNonZero(expected != expected),
            cv::countNonZero(result.depth_m != result.depth_m));
  const cv::Mat finite_mask = expected == expected;
  ASSERT_GT(cv::countNonZero(finite_mask), 0);
  EXPECT_DOUBLE_EQ(cv::norm(expected, result.depth_m, cv::NORM_INF, finite_mask), 0);
  EXPECT_EQ(result.depth_m.size(), expected.size());
  stages.clear();
  auto invalid = pair;
  invalid.left.optical_frame_id.clear();
  EXPECT_THROW(processor.process(invalid, [&](auto stage) { stages.push_back(stage); }),
               std::invalid_argument);
  EXPECT_TRUE(stages.empty());
}
} // namespace
} // namespace intball2_stereo::test
