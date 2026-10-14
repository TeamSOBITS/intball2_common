#include <cmath>
#include <cstdint>
#include <memory>

#include <gtest/gtest.h>
#include <opencv2/calib3d.hpp>

#include "intball2_stereo/core/stereo_processor.hpp"
#include "stereo_test_fixture.hpp"

namespace
{
namespace core = intball2_stereo::core;
using intball2_stereo::test::calibration;
using intball2_stereo::test::frame;

core::StereoConfig plane_config(bool negative)
{
  core::StereoConfig config;
  config.camera.calibration_mode = core::CalibrationMode::kCameraInfo;
  config.processing.downsample = 1;
  config.matcher.min_disparity_px = negative ? -17 : 0;
  config.depth.min_depth_m = .5;
  return config;
}

core::ImagePair plane_pair(bool negative)
{
  cv::Mat left(200, 200, CV_8UC1);
  cv::RNG random(9821);
  random.fill(left, cv::RNG::UNIFORM, 0, 256);
  cv::Mat right(left.size(), left.type(), cv::Scalar(0));
  const int source_x = negative ? 0 : 8;
  const int target_x = negative ? 8 : 0;
  left(cv::Rect(source_x, 0, 192, 200)).copyTo(right(cv::Rect(target_x, 0, 192, 200)));
  return {frame(left, calibration(200, 200, 100)),
          frame(right, calibration(200, 200, 100, negative ? 8 : -8), "right_optical")};
}

TEST(MatchingRoi, CroppedModelPreservesSignedBaselinePrincipalDifferenceAndPixelRays)
{
  for (const bool negative : {false, true}) {
    auto left = calibration(200, 200, 100);
    auto right = calibration(200, 200, 100, negative ? 5 : -5);
    right.k[2] += negative ? 2 : -2;
    right.p[2] = right.k[2];
    const auto config = plane_config(negative);
    const core::CameraModel full(left, right, config.camera, 1);
    const cv::Rect roi(7, 9, 150, 170);
    const auto cropped = full.cropped(roi);
    EXPECT_EQ(cropped.width_px(), roi.width);
    EXPECT_EQ(cropped.height_px(), roi.height);
    EXPECT_DOUBLE_EQ(cropped.numerator_px_m(), full.numerator_px_m());
    EXPECT_DOUBLE_EQ(cropped.principal_difference_px(), full.principal_difference_px());
    EXPECT_DOUBLE_EQ((20 - cropped.left().p[2]) / cropped.left().p[0],
                     (20 + roi.x - full.left().p[2]) / full.left().p[0]);
    EXPECT_DOUBLE_EQ((30 - cropped.left().p[6]) / cropped.left().p[5],
                     (30 + roi.y - full.left().p[6]) / full.left().p[5]);
    EXPECT_DOUBLE_EQ(cropped.right().p[3], full.right().p[3]);
    EXPECT_DOUBLE_EQ(full.left().p[2], left.p[2]);
    cv::Mat disparity(roi.size(), CV_16SC1, cv::Scalar(negative ? -128 : 128));
    cv::Mat mask(roi.size(), CV_8UC1, cv::Scalar(255));
    cv::Mat depth;
    core::DepthConverter::convert(disparity, cropped, config.matcher, config.depth, mask, mask,
                                  depth);
    EXPECT_NEAR(depth.at<float>(60, 60), 5. / 6., 1e-7);
    EXPECT_THROW(full.cropped(cv::Rect(-1, 0, 10, 10)), std::invalid_argument);
    EXPECT_THROW(full.cropped(cv::Rect(0, 0, 201, 10)), std::invalid_argument);
  }
  auto config = plane_config(true);
  config.camera.input_geometry = core::InputGeometry::kRaw;
  const core::CameraModel raw(calibration(), calibration(200, 200, 100, 8), config.camera, 1);
  EXPECT_THROW(raw.cropped(cv::Rect(1, 1, 190, 190)), std::invalid_argument);
}

TEST(MatchingRoi, SmallMatchingRestoresFullCalibrationAndNanExteriorWithoutMutatingBorrowedInputs)
{
  for (const bool negative : {false, true}) {
    auto config = plane_config(negative);
    auto pair = plane_pair(negative);
    auto owner = std::make_shared<cv::Mat>(pair.left.image.pixels().clone());
    pair.left.image = core::ImageBuffer::borrow(*owner, owner);
    const auto source = owner->clone();
    const auto right_source = pair.right.image.pixels().clone();
    const auto full_result = core::StereoProcessor(config).process(pair);
    config.processing.roi_left_px = 5;
    config.processing.roi_right_px = 10;
    config.processing.roi_top_px = 7;
    config.processing.roi_bottom_px = 11;
    core::StereoProcessor processor(config);
    const auto &result = processor.process(pair);
    const cv::Rect roi(5, 7, 185, 182);
    EXPECT_EQ(result.depth_m.size(), cv::Size(200, 200));
    EXPECT_TRUE(core::same_calibration(result.calibration, full_result.calibration));
    EXPECT_EQ(result.stamp.nanoseconds, pair.left.stamp.nanoseconds);
    EXPECT_EQ(result.optical_frame_id, pair.left.optical_frame_id);
    EXPECT_NE(result.depth_m.data, owner->data);
    EXPECT_DOUBLE_EQ(cv::norm(source, *owner, cv::NORM_INF), 0);
    EXPECT_DOUBLE_EQ(cv::norm(right_source, pair.right.image.pixels(), cv::NORM_INF), 0);
    std::uint64_t valid = 0;
    double error = 0;
    for (int y = 0; y < 200; ++y) {
      for (int x = 0; x < 200; ++x) {
        const auto depth = result.depth_m.at<float>(y, x);
        if (!roi.contains(cv::Point(x, y))) {
          EXPECT_TRUE(std::isnan(depth));
        } else if (std::isfinite(depth)) {
          ++valid;
          error += std::abs(depth - 1.);
        }
      }
    }
    ASSERT_GT(valid, 20000U);
    EXPECT_LT(error / valid, .02);
    EXPECT_NEAR(result.statistics.valid_depth_fraction, static_cast<double>(valid) / 40000, 1e-15);
    const cv::Rect full(0, 0, 200, 200), local(0, 0, roi.width, roi.height);
    const auto eligible = [&](const cv::Rect &image) {
      return cv::getValidDisparityROI(image, image, config.matcher.min_disparity_px,
                                      config.matcher.num_disparities_px,
                                      config.matcher.block_size_px);
    };
    EXPECT_EQ(result.statistics.matching_input_pixels, static_cast<std::uint64_t>(roi.area()));
    EXPECT_EQ(result.statistics.full_disparity_eligible_pixels,
              static_cast<std::uint64_t>(eligible(full).area()));
    EXPECT_EQ(result.statistics.roi_disparity_eligible_pixels,
              static_cast<std::uint64_t>(eligible(local).area()));
    EXPECT_EQ(result.statistics.roi_additional_search_excluded_pixels,
              static_cast<std::uint64_t>((eligible(full) & roi).area() - eligible(local).area()));
    EXPECT_GT(result.statistics.roi_additional_search_excluded_pixels, 0U);
  }
}

TEST(MatchingRoi, ZeroMarginsFollowExistingFullMatchingAndRefinerUsesRestoredFullGrid)
{
  auto config = plane_config(true);
  const auto pair = plane_pair(true);
  core::StereoProcessor processor(config);
  const auto original = processor.process(pair).depth_m.clone();
  core::CameraModel model(pair.left.calibration, pair.right.calibration, config.camera, 1);
  core::ImagePreprocessor preprocessor(model, config.matcher.block_size_px);
  const auto &prepared = preprocessor.prepare(pair);
  core::SgbmMatcher matcher(config.matcher);
  cv::Mat expected;
  core::DepthConverter::convert(matcher.compute(prepared.left_gray, prepared.right_gray), model,
                                config.matcher, config.depth, prepared.left_valid,
                                prepared.right_valid, expected);
  EXPECT_EQ(cv::countNonZero((original == original) != (expected == expected)), 0);
  EXPECT_DOUBLE_EQ(cv::norm(original, expected, cv::NORM_INF, expected == expected), 0);
  config.refinement.enabled = true;
  config.refinement.downsample = 1;
  config.processing.roi_left_px = config.processing.roi_right_px = 10;
  core::StereoProcessor refined(config);
  const auto &result = refined.process(pair);
  EXPECT_EQ(result.depth_m.size(), original.size());
  EXPECT_GT(result.statistics.refinement_attempted_pixels, 0U);
  EXPECT_TRUE(std::isnan(result.depth_m.at<float>(100, 0)));
  EXPECT_TRUE(std::isnan(result.depth_m.at<float>(100, 199)));
  EXPECT_DOUBLE_EQ(result.calibration.p[2], model.left().p[2]);
}

TEST(MatchingRoi, RejectsRawNegativeMarginsEmptyAndUnsupportedSearchRegions)
{
  auto config = plane_config(true);
  config.processing.roi_left_px = -1;
  EXPECT_FALSE(core::validate_config(config).empty());
  config.processing.roi_left_px = 1;
  config.camera.input_geometry = core::InputGeometry::kRaw;
  EXPECT_FALSE(core::validate_config(config).empty());
  config.camera.input_geometry = core::InputGeometry::kAlreadyRectified;
  config.processing.roi_left_px = 180;
  EXPECT_FALSE(core::validate_config_for_input(config, 200, 200).empty());
  config.processing.roi_right_px = 20;
  EXPECT_FALSE(core::validate_config_for_input(config, 200, 200).empty());
}
} // namespace
