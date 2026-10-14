#include "intball2_stereo/ros/stereo_parameters.hpp"

#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "../core/stereo_test_fixture.hpp"
#include "intball2_stereo/core/stereo_processor.hpp"

namespace intball2_stereo::ros
{
namespace
{
using Parameters = std::vector<rclcpp::Parameter>;

TEST(ParameterUpdates, PartialUpdatePreservesCommittedValuesAndSnapshot)
{
  const Parameters current{{"matcher.uniqueness_ratio", 37}, {"depth.max_depth_m", 5.0}};
  const auto before = StereoParameters::read_values(current);
  const auto next = StereoParameters::validate_updates(ParameterSnapshot{current},
                                                       {{"processing.downsample", 2}});
  EXPECT_EQ(next.core.processing.downsample, 2);
  EXPECT_EQ(next.core.matcher.uniqueness_ratio, 37);
  EXPECT_DOUBLE_EQ(next.core.depth.max_depth_m, 5.0);
  EXPECT_EQ(before.core.processing.downsample, 4);
  EXPECT_EQ(current.size(), 2U);
}

TEST(ParameterUpdates, CoupledChangesAreValidatedAsOneTransaction)
{
  const Parameters current{{"depth.min_depth_m", 0.5}, {"depth.max_depth_m", 1.0}};
  EXPECT_THROW(
      StereoParameters::validate_updates(ParameterSnapshot{current}, {{"depth.min_depth_m", 2.0}}),
      std::invalid_argument);
  const auto next = StereoParameters::validate_updates(
      ParameterSnapshot{current}, {{"depth.min_depth_m", 2.0}, {"depth.max_depth_m", 4.0}});
  EXPECT_DOUBLE_EQ(next.core.depth.min_depth_m, 2.0);
  EXPECT_DOUBLE_EQ(next.core.depth.max_depth_m, 4.0);
  const auto compact = StereoParameters::validate_updates(
      ParameterSnapshot{{}},
      {{"points.publish_points", true}, {"points.layout", "compact"}, {"points.stride", 3}});
  EXPECT_TRUE(compact.core.points.publish_points);
  EXPECT_EQ(compact.core.points.layout, core::PointsLayout::kCompact);
  EXPECT_EQ(compact.core.points.stride, 3);
}

TEST(ParameterUpdates, RebuiltProcessorAppliesNewSnapshotWithoutChangingOldProcessor)
{
  const auto before = StereoParameters::read_values({});
  core::StereoProcessor old_processor(before.core);
  const auto left = test::calibration(800, 800, 400);
  const auto right = test::calibration(800, 800, 400);
  const cv::Mat pixels(800, 800, CV_8UC1, cv::Scalar(128));
  const core::ImagePair pair{test::frame(pixels, left),
                             test::frame(pixels, right, "right_optical")};
  EXPECT_EQ(old_processor.process(pair).depth_m.size(), cv::Size(200, 200));

  const auto next = StereoParameters::validate_updates(ParameterSnapshot{{}},
                                                       {{"processing.downsample", 2},
                                                        {"matcher.min_disparity_px", -49},
                                                        {"matcher.num_disparities_px", 64}});
  core::StereoProcessor new_processor(next.core);
  EXPECT_EQ(new_processor.process(pair).depth_m.size(), cv::Size(400, 400));
  EXPECT_EQ(old_processor.process(pair).depth_m.size(), cv::Size(200, 200));
  EXPECT_EQ(before.core.processing.downsample, 4);
}

TEST(ParameterUpdates, RejectedTransactionLeavesBothInputsUnchanged)
{
  const Parameters current{{"depth.min_depth_m", 0.5}, {"matcher.uniqueness_ratio", 37}};
  const Parameters updates{{"matcher.uniqueness_ratio", 45}, {"depth.max_depth_m", 0.1}};
  EXPECT_THROW(StereoParameters::validate_updates(ParameterSnapshot{current}, updates),
               std::invalid_argument);
  const auto after = StereoParameters::read_values(current);
  EXPECT_EQ(after.core.matcher.uniqueness_ratio, 37);
  EXPECT_DOUBLE_EQ(after.core.depth.min_depth_m, 0.5);
  EXPECT_DOUBLE_EQ(after.core.depth.max_depth_m, 3.0);
  EXPECT_EQ(updates.front().as_int(), 45);
  EXPECT_DOUBLE_EQ(updates.back().as_double(), 0.1);
}

TEST(ParameterUpdates, RejectsStartupOnlyValuesIncludingUnchangedValues)
{
  for (const auto &spec : StereoParameters::specifications()) {
    if (spec.update_policy == UpdatePolicy::kStartupOnly) {
      const auto value = std::visit(
          [&](const auto &item) { return rclcpp::Parameter(spec.name, item); }, spec.default_value);
      EXPECT_THROW(StereoParameters::validate_updates(ParameterSnapshot{{}}, {value}),
                   std::invalid_argument)
          << spec.name;
    }
  }
}

TEST(ParameterUpdates, RejectsUnknownDuplicateAndUnsetValues)
{
  EXPECT_THROW(StereoParameters::validate_updates(ParameterSnapshot{{}}, {{"missing", 1}}),
               std::invalid_argument);
  EXPECT_THROW(
      StereoParameters::validate_updates(
          ParameterSnapshot{{}}, {{"processing.downsample", 2}, {"processing.downsample", 4}}),
      std::invalid_argument);
  EXPECT_THROW(StereoParameters::validate_updates(ParameterSnapshot{{}},
                                                  {rclcpp::Parameter("points.publish_points")}),
               std::invalid_argument);
}

TEST(ParameterUpdates, DiagnosticStartupConfigurationHasTypedDefaultsAndBounds)
{
  const auto defaults = StereoParameters::read_values({});
  EXPECT_DOUBLE_EQ(defaults.diagnostics.publish_interval_s, 1.0);
  EXPECT_EQ(defaults.topics.diagnostics, "/diagnostics");
  for (const double value : {0.0, 0.009, 60.001, std::numeric_limits<double>::quiet_NaN(),
                             std::numeric_limits<double>::infinity()}) {
    EXPECT_THROW(StereoParameters::read_values({{"diagnostics.publish_interval_s", value}}),
                 std::invalid_argument);
  }
  for (const double value : {0.01, 60.0}) {
    EXPECT_DOUBLE_EQ(StereoParameters::read_values({{"diagnostics.publish_interval_s", value}})
                         .diagnostics.publish_interval_s,
                     value);
  }
  EXPECT_THROW(StereoParameters::read_values({{"topics.diagnostics", "relative"}}),
               std::invalid_argument);
  EXPECT_THROW(StereoParameters::validate_updates(ParameterSnapshot{{}},
                                                  {{"diagnostics.publish_interval_s", 2.0}}),
               std::invalid_argument);
  EXPECT_THROW(StereoParameters::validate_updates(ParameterSnapshot{{}},
                                                  {{"topics.diagnostics", "/custom"}}),
               std::invalid_argument);
}

TEST(ParameterUpdates, RejectsInvalidTypesEnumsBoundsAndNonfiniteValues)
{
  const std::vector<Parameters> invalid{
      {{"processing.downsample", 2.0}},
      {{"processing.downsample", std::numeric_limits<std::int64_t>::max()}},
      {{"depth.min_depth_m", 1}},
      {{"points.publish_points", "true"}},
      {{"camera.calibration_mode", "automatic"}},
      {{"points.layout", "unordered"}},
      {{"matcher.disparity_units", "original_pixels"}},
      {{"matcher.num_disparities_px", 17}},
      {{"matcher.block_size_px", 4}},
      {{"matcher.uniqueness_ratio", 101}},
      {{"depth.min_depth_m", std::numeric_limits<double>::quiet_NaN()}},
      {{"depth.max_depth_m", std::numeric_limits<double>::infinity()}},
      {{"points.stride", 2}}};
  for (const auto &updates : invalid) {
    EXPECT_THROW(StereoParameters::validate_updates(ParameterSnapshot{{}}, updates),
                 std::invalid_argument)
        << updates.front().get_name();
  }
}

TEST(ParameterUpdates, ValidatesCurrentSnapshotBeforeMerging)
{
  EXPECT_THROW(StereoParameters::validate_updates(ParameterSnapshot{{{"missing", 1}}}, {}),
               std::invalid_argument);
  EXPECT_THROW(StereoParameters::validate_updates(
                   ParameterSnapshot{{{"points.stride", 1}, {"points.stride", 1}}}, {}),
               std::invalid_argument);
  const auto unchanged =
      StereoParameters::validate_updates(ParameterSnapshot{{{"processing.downsample", 2}}}, {});
  EXPECT_EQ(unchanged.core.processing.downsample, 2);
}

TEST(ParameterUpdates, RefinementIsOptInAndChangesAtTheFrameBoundary)
{
  const auto original = StereoParameters::read_values({});
  EXPECT_FALSE(original.core.refinement.enabled);
  const auto candidate =
      StereoParameters::validate_updates(ParameterSnapshot{{}}, {{"refinement.enabled", true}});
  EXPECT_TRUE(candidate.core.refinement.enabled);
  EXPECT_FALSE(original.core.refinement.enabled);
  EXPECT_THROW(
      StereoParameters::validate_updates(ParameterSnapshot{{}}, {{"refinement.enabled", "true"}}),
      std::invalid_argument);
  const auto disabled = StereoParameters::validate_updates(
      ParameterSnapshot{{{"refinement.enabled", true}}}, {{"refinement.enabled", false}});
  EXPECT_FALSE(disabled.core.refinement.enabled);
}

TEST(ParameterUpdates, NativeRefinementProfileIsAtomicAndPreservesActiveFrame)
{
  const ParameterSnapshot current{{{"processing.downsample", 1}}};
  const StereoFrameConfiguration active(current);
  EXPECT_THROW(StereoParameters::validate_updates(current, {{"refinement.enabled", true}}),
               std::invalid_argument);
  const Parameters updates{{"refinement.enabled", true},
                           {"refinement.downsample", 1},
                           {"refinement.window_size_px", 17},
                           {"refinement.search_radius_px", 3},
                           {"refinement.min_texture_variance", 2.0},
                           {"refinement.min_correlation", 0.8}};
  const auto candidate = StereoParameters::validate_updates(current, updates);
  EXPECT_TRUE(candidate.core.refinement.enabled);
  EXPECT_EQ(candidate.core.refinement.downsample, 1);
  EXPECT_EQ(candidate.core.refinement.window_size_px, 17);
  EXPECT_EQ(candidate.core.refinement.search_radius_px, 3);
  EXPECT_DOUBLE_EQ(candidate.core.refinement.min_texture_variance, 2.0);
  EXPECT_DOUBLE_EQ(candidate.core.refinement.min_correlation, 0.8);
  Parameters next = current.values;
  next.insert(next.end(), updates.begin(), updates.end());
  const StereoFrameConfiguration prepared(ParameterSnapshot{next});
  EXPECT_FALSE(active.matches(ParameterSnapshot{next}));
  EXPECT_FALSE(active.config().core.refinement.enabled);
  EXPECT_EQ(active.config().core.refinement.downsample, 2);
  EXPECT_EQ(active.config().core.refinement.window_size_px, 9);
  EXPECT_EQ(active.config().core.refinement.search_radius_px, 2);
  EXPECT_EQ(prepared.config().core.refinement.downsample, 1);
  EXPECT_EQ(prepared.config().core.refinement.window_size_px, 17);
  EXPECT_EQ(prepared.config().core.refinement.search_radius_px, 3);
  const std::vector<Parameters> invalid{
      {{"refinement.window_size_px", 17.0}},
      {{"refinement.window_size_px", 18}},
      {{"refinement.search_radius_px", 9}},
      {{"refinement.min_texture_variance", 0.0}},
      {{"refinement.min_correlation", 1.1}},
      {{"refinement.min_correlation", std::numeric_limits<double>::quiet_NaN()}},
      {{"refinement.downsample", 2}}};
  for (const auto &values : invalid) {
    EXPECT_THROW(StereoParameters::validate_updates(ParameterSnapshot{next}, values),
                 std::invalid_argument)
        << values.front().get_name();
  }
  EXPECT_FALSE(active.config().core.refinement.enabled);
  EXPECT_TRUE(prepared.config().core.refinement.enabled);
}

TEST(FrameConfiguration, UnchangedSnapshotReusesTypedConfigurationUntilAtomicReplacement)
{
  const ParameterSnapshot initial{{{"depth.min_depth_m", 0.5}, {"depth.max_depth_m", 1.0}}};
  auto active = std::make_unique<StereoFrameConfiguration>(initial);
  const auto *original = &active->config();
  EXPECT_TRUE(active->matches(initial));
  EXPECT_EQ(&active->config(), original);
  const ParameterSnapshot update{{{"depth.min_depth_m", 2.0}, {"depth.max_depth_m", 4.0}}};
  EXPECT_FALSE(active->matches(update));
  auto prepared = std::make_unique<StereoFrameConfiguration>(update);
  EXPECT_DOUBLE_EQ(active->config().core.depth.min_depth_m, 0.5);
  EXPECT_DOUBLE_EQ(active->config().core.depth.max_depth_m, 1.0);
  core::StereoProcessor replacement(prepared->config().core);
  active = std::move(prepared);
  EXPECT_TRUE(active->matches(update));
  EXPECT_DOUBLE_EQ(active->config().core.depth.min_depth_m, 2.0);
  EXPECT_DOUBLE_EQ(active->config().core.depth.max_depth_m, 4.0);
}

TEST(FrameConfiguration, RejectedPreparationLeavesPreviousConfigurationAndSnapshotUsable)
{
  const ParameterSnapshot initial{{{"processing.downsample", 4}, {"points.publish_points", false}}};
  StereoFrameConfiguration active(initial);
  const auto *original = &active.config();
  EXPECT_THROW(StereoFrameConfiguration(ParameterSnapshot{{{"depth.min_depth_m", 4.0}}}),
               std::invalid_argument);
  EXPECT_THROW(StereoFrameConfiguration(
                   ParameterSnapshot{{{"points.publish_points", true}, {"points.stride", 2}}}),
               std::invalid_argument);
  EXPECT_TRUE(active.matches(initial));
  EXPECT_EQ(&active.config(), original);
  EXPECT_FALSE(active.config().core.points.publish_points);
}
TEST(ParameterUpdates, MatchingRoiMarginsAreOutputPixelFrameValuesAndRejectRawAtomically)
{
  const auto initial = StereoParameters::read_values({});
  EXPECT_EQ(initial.core.processing.roi_left_px, 0);
  const auto candidate =
      StereoParameters::validate_updates(ParameterSnapshot{{}}, {{"processing.roi_left_px", 5},
                                                                 {"processing.roi_right_px", 10},
                                                                 {"processing.roi_top_px", 7},
                                                                 {"processing.roi_bottom_px", 11}});
  EXPECT_EQ(candidate.core.processing.roi_left_px, 5);
  EXPECT_EQ(candidate.core.processing.roi_right_px, 10);
  EXPECT_EQ(candidate.core.processing.roi_top_px, 7);
  EXPECT_EQ(candidate.core.processing.roi_bottom_px, 11);
  EXPECT_EQ(initial.core.processing.roi_left_px, 0);
  EXPECT_THROW(
      StereoParameters::validate_updates(ParameterSnapshot{{}}, {{"processing.roi_left_px", -1}}),
      std::invalid_argument);
  EXPECT_THROW(
      StereoParameters::validate_updates(ParameterSnapshot{{}}, {{"processing.roi_left_px", 4097}}),
      std::invalid_argument);
  EXPECT_THROW(
      StereoParameters::validate_updates(ParameterSnapshot{{}}, {{"processing.roi_left_px", 5.0}}),
      std::invalid_argument);
  EXPECT_THROW(StereoParameters::validate_updates(ParameterSnapshot{{}},
                                                  {{"camera.calibration_mode", "camera_info"},
                                                   {"camera.input_geometry", "raw"},
                                                   {"processing.roi_left_px", 5}}),
               std::invalid_argument);
}
} // namespace
} // namespace intball2_stereo::ros
