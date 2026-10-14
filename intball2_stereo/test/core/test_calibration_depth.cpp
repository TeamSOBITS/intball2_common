#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "intball2_stereo/core/depth_converter.hpp"
#include "stereo_test_fixture.hpp"

namespace
{
using intball2_stereo::core::CalibrationMode;
using intball2_stereo::core::CameraConfig;
using intball2_stereo::core::CameraModel;
using intball2_stereo::core::DepthConfig;
using intball2_stereo::core::DepthConverter;
using intball2_stereo::core::MatcherConfig;
using intball2_stereo::test::calibration;

CameraConfig standard_camera()
{
  CameraConfig config;
  config.calibration_mode = CalibrationMode::kCameraInfo;
  return config;
}

TEST(CameraCalibration, PixelCentresAndTranslationScaleTogether)
{
  auto left = calibration(800, 800, 477);
  auto right = left;
  right.p[3] = -477 * 0.05;
  const CameraModel model(left, right, standard_camera(), 4);
  EXPECT_EQ(model.width_px(), 200);
  EXPECT_EQ(model.height_px(), 200);
  EXPECT_DOUBLE_EQ(model.left().k[0], 119.25);
  EXPECT_DOUBLE_EQ(model.left().k[2], 99.5);
  EXPECT_DOUBLE_EQ(model.left().p[2], 99.5);
  EXPECT_DOUBLE_EQ(model.numerator_px_m(), 119.25 * 0.05);
  // Independent projection of the same 3D point through resized coordinates.
  const double x_m = 0.17;
  const double z_m = 1.2;
  const double original_u = 477 * x_m / z_m + left.k[2];
  EXPECT_NEAR(model.left().k[0] * x_m / z_m + model.left().k[2], (original_u + 0.5) / 4 - 0.5,
              1e-12);
}

TEST(CameraCalibration, GazeboOverridePreservesNegativeBaselineAndRejectsFixedInfo)
{
  auto left = calibration();
  auto right = left;
  left.p[3] = right.p[3] = 8.0;
  const CameraConfig gazebo;
  const CameraModel model(left, right, gazebo, 1);
  EXPECT_DOUBLE_EQ(model.left().p[3], 0);
  EXPECT_DOUBLE_EQ(model.right().p[3], 5);
  EXPECT_DOUBLE_EQ(model.numerator_px_m(), -5);
  left.p[3] = 0;
  right.p[3] = 5;
  EXPECT_THROW(CameraModel(left, right, gazebo, 1), std::invalid_argument);
  EXPECT_NO_THROW(CameraModel(left, right, standard_camera(), 1));
}

TEST(CameraCalibration, RebasesNonzeroLeftTranslation)
{
  auto left = calibration(200, 200, 100, 7);
  auto right = calibration(200, 200, 100, 2);
  const CameraModel model(left, right, standard_camera(), 1);
  EXPECT_DOUBLE_EQ(model.left().p[3], 0);
  EXPECT_DOUBLE_EQ(model.right().p[3], -5);
}

TEST(CameraCalibration, InvalidGeometryIsRejectedBeforeMatching)
{
  const auto left = calibration();
  const auto right = calibration(200, 200, 100, -5);
  auto bad = left;
  bad.k[0] = 0;
  EXPECT_THROW(CameraModel(bad, right, standard_camera(), 1), std::invalid_argument);
  bad = left;
  bad.p[2] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(CameraModel(bad, right, standard_camera(), 1), std::invalid_argument);
  bad = right;
  bad.p[6] += 1;
  EXPECT_THROW(CameraModel(left, bad, standard_camera(), 1), std::invalid_argument);
  bad = right;
  bad.p[7] = 1;
  EXPECT_THROW(CameraModel(left, bad, standard_camera(), 1), std::invalid_argument);
  bad = left;
  bad.r[0] = 2;
  EXPECT_THROW(CameraModel(bad, right, standard_camera(), 1), std::invalid_argument);
  bad = left;
  bad.roi.x_offset = 4;
  bad.roi.width = 100;
  bad.roi.height = 100;
  EXPECT_THROW(CameraModel(bad, right, standard_camera(), 1), std::invalid_argument);
  bad = left;
  bad.binning_x = 2;
  EXPECT_THROW(CameraModel(bad, right, standard_camera(), 1), std::invalid_argument);
  bad = left;
  bad.distortion_model = "equidistant";
  EXPECT_THROW(CameraModel(bad, right, standard_camera(), 1), std::invalid_argument);
  bad = left;
  bad.width = 199;
  EXPECT_THROW(CameraModel(left, bad, standard_camera(), 4), std::invalid_argument);
  EXPECT_THROW(CameraModel(left, left, standard_camera(), 1), std::invalid_argument);
}

TEST(CameraCalibration, SearchMustCoverBothCalibratedDepthLimits)
{
  const CameraModel model(calibration(), calibration(200, 200, 100, -5), standard_camera(), 1);
  MatcherConfig matcher;
  EXPECT_THROW(model.require_search_coverage(matcher, DepthConfig{}), std::invalid_argument);
  matcher.min_disparity_px = 0;
  EXPECT_NO_THROW(model.require_search_coverage(matcher, DepthConfig{}));
  matcher.num_disparities_px = 16;
  EXPECT_THROW(model.require_search_coverage(matcher, DepthConfig{}), std::invalid_argument);
}

TEST(CameraCalibration, SearchAndBlockMustLeaveAnActualImageRegion)
{
  intball2_stereo::core::StereoConfig config;
  config.camera = standard_camera();
  config.processing.downsample = 1;
  config.matcher.min_disparity_px = 0;
  config.matcher.num_disparities_px = 16;
  config.matcher.block_size_px = 31;
  EXPECT_FALSE(intball2_stereo::core::validate_config_for_input(config, 32, 32).empty());
}

TEST(DepthConversion, PositiveAndNegativeDisparityHaveTheSamePhysicalDepth)
{
  for (const int sign : {-1, 1}) {
    const CameraModel model(calibration(), calibration(200, 200, 100, -sign * 5), standard_camera(),
                            1);
    MatcherConfig matcher;
    matcher.min_disparity_px = sign > 0 ? 0 : -25;
    cv::Mat disparity(200, 200, CV_16SC1, cv::Scalar(sign * 8 * 16));
    cv::Mat valid(200, 200, CV_8UC1, cv::Scalar(255));
    cv::Mat depth;
    const auto fraction =
        DepthConverter::convert(disparity, model, matcher, {}, valid, valid, depth);
    EXPECT_FLOAT_EQ(depth.at<float>(100, 100), 0.625F);
    EXPECT_GT(fraction, 0.7);
    EXPECT_LT(fraction, 1.0);
    EXPECT_TRUE(std::isnan(depth.at<float>(100, 0)));
    EXPECT_TRUE(std::isnan(depth.at<float>(100, 199)));
  }
}

TEST(DepthConversion, PrincipalDifferenceAndSubpixelDisparityAreIncluded)
{
  auto left = calibration();
  auto right = calibration(200, 200, 100, -5);
  right.p[2] = left.p[2] - 2;
  const CameraModel model(left, right, standard_camera(), 1);
  MatcherConfig matcher;
  matcher.min_disparity_px = 0;
  cv::Mat disparity(200, 200, CV_16SC1, cv::Scalar(10.5 * 16));
  cv::Mat valid(200, 200, CV_8UC1, cv::Scalar(255));
  cv::Mat depth;
  DepthConverter::convert(disparity, model, matcher, {}, valid, valid, depth);
  EXPECT_NEAR(depth.at<float>(100, 100), 5.0 / 8.5, 1e-7);
  // Physical projection, independent of converter formula.
  const double z_m = 5.0 / 8.5;
  const double u_left = 100 * 0.1 / z_m + left.p[2];
  const double u_right = 100 * (0.1 - 0.05) / z_m + right.p[2];
  EXPECT_NEAR(u_left - u_right, 10.5, 1e-12);
}

TEST(DepthConversion, ZeroDisparityCanBeValidWithPrincipalOffset)
{
  auto left = calibration();
  auto right = calibration(200, 200, 100, 5);
  right.p[2] -= 2;
  const CameraModel model(left, right, standard_camera(), 1);
  const MatcherConfig matcher;
  const cv::Mat disparity(200, 200, CV_16SC1, cv::Scalar(0));
  const cv::Mat valid(200, 200, CV_8UC1, cv::Scalar(255));
  cv::Mat depth;
  DepthConverter::convert(disparity, model, matcher, {}, valid, valid, depth);
  EXPECT_FLOAT_EQ(depth.at<float>(100, 100), 2.5F);
}

TEST(DepthConversion, InvalidTooNearAndTooFarRemainNanAndBuffersAreReused)
{
  auto left = calibration();
  auto right = calibration(200, 200, 100, -5);
  right.p[2] -= 2;
  const CameraModel model(left, right, standard_camera(), 1);
  MatcherConfig matcher;
  matcher.min_disparity_px = 0;
  cv::Mat disparity(200, 200, CV_16SC1, cv::Scalar(10 * 16));
  cv::Mat valid(200, 200, CV_8UC1, cv::Scalar(255));
  // Invalid sentinel, zero denominator, negative depth, too near, too far, outside search.
  const int values[] = {-16, 2 * 16, 1 * 16, 31 * 16, 3 * 16, 40 * 16};
  for (int i = 0; i < 6; ++i) {
    disparity.at<std::int16_t>(100, 80 + i) = values[i];
  }
  cv::Mat depth;
  DepthConverter::convert(disparity, model, matcher, {}, valid, valid, depth);
  for (int i = 0; i < 6; ++i) {
    EXPECT_TRUE(std::isnan(depth.at<float>(100, 80 + i)));
  }
  const auto *storage = depth.data;
  disparity.setTo(-16);
  EXPECT_DOUBLE_EQ(DepthConverter::convert(disparity, model, matcher, {}, valid, valid, depth), 0);
  EXPECT_EQ(storage, depth.data);
  EXPECT_TRUE(std::isnan(depth.at<float>(100, 100)));
}

TEST(DepthConversion, RightRectificationMaskChecksBothSubpixelSamples)
{
  const CameraModel model(calibration(), calibration(200, 200, 100, -5), standard_camera(), 1);
  MatcherConfig matcher;
  matcher.min_disparity_px = 0;
  cv::Mat disparity(200, 200, CV_16SC1, cv::Scalar(8.5 * 16));
  cv::Mat left_valid(200, 200, CV_8UC1, cv::Scalar(255));
  cv::Mat right_valid = left_valid.clone();
  right_valid.at<unsigned char>(100, 92) = 0;
  left_valid.at<unsigned char>(100, 110) = 0;
  cv::Mat depth;
  DepthConverter::convert(disparity, model, matcher, {}, left_valid, right_valid, depth);
  EXPECT_TRUE(std::isnan(depth.at<float>(100, 100)));
  EXPECT_TRUE(std::isnan(depth.at<float>(100, 110)));
  EXPECT_TRUE(std::isfinite(depth.at<float>(100, 105)));
}

TEST(DepthConversion, NonContiguousRowsAreSupportedAndWrongTypesRejected)
{
  const CameraModel model(calibration(), calibration(200, 200, 100, -5), standard_camera(), 1);
  MatcherConfig matcher;
  matcher.min_disparity_px = 0;
  cv::Mat storage(202, 204, CV_16SC1, cv::Scalar(8 * 16));
  const auto disparity = storage(cv::Rect(1, 1, 200, 200));
  cv::Mat valid(200, 200, CV_8UC1, cv::Scalar(255));
  cv::Mat depth;
  DepthConverter::convert(disparity, model, matcher, {}, valid, valid, depth);
  EXPECT_FLOAT_EQ(depth.at<float>(100, 100), 0.625F);
  cv::Mat wrong(200, 200, CV_32FC1);
  EXPECT_THROW(DepthConverter::convert(wrong, model, matcher, {}, valid, valid, depth),
               std::invalid_argument);
  auto aliases_input = disparity;
  EXPECT_THROW(DepthConverter::convert(disparity, model, matcher, {}, valid, valid, aliases_input),
               std::invalid_argument);
  EXPECT_EQ(disparity.at<std::int16_t>(100, 100), 8 * 16);
}
} // namespace
