#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "intball2_stereo/core/pointcloud_generator.hpp"
#include "stereo_test_fixture.hpp"

namespace
{
using namespace intball2_stereo;
core::DepthResult depth_result()
{
  core::DepthResult result;
  result.depth_m = cv::Mat(3, 4, CV_32FC1, cv::Scalar(2));
  result.calibration = test::calibration(4, 3, 2);
  result.calibration.k[4] = 4;
  result.stamp.nanoseconds = 987654321;
  result.optical_frame_id = "left_optical";
  return result;
}
TEST(PointCloud, ProjectsOpticalDepthWithIndependentFocalLengthsAndKeepsHeader)
{
  const auto result = depth_result();
  const auto cloud = core::PointCloudGenerator::generate(result, {});
  EXPECT_EQ(cloud.width, 4U);
  EXPECT_EQ(cloud.height, 3U);
  EXPECT_EQ(cloud.layout, core::PointsLayout::kOrganized);
  EXPECT_EQ(cloud.stamp.nanoseconds, result.stamp.nanoseconds);
  EXPECT_EQ(cloud.optical_frame_id, result.optical_frame_id);
  ASSERT_EQ(cloud.points.size(), 12U);
  EXPECT_FLOAT_EQ(cloud.points[0].x_m, -1.5F);
  EXPECT_FLOAT_EQ(cloud.points[0].y_m, -0.5F);
  EXPECT_FLOAT_EQ(cloud.points[0].z_m, 2);
  EXPECT_FLOAT_EQ(cloud.points[11].x_m, 1.5F);
  EXPECT_FLOAT_EQ(cloud.points[11].y_m, 0.5F);
}
TEST(PointCloud, NonContiguousDepthOwnsPointsAndInvalidPixelsAreNanXyz)
{
  auto result = depth_result();
  cv::Mat storage(3, 7, CV_32FC1, cv::Scalar(2));
  result.depth_m = storage(cv::Rect(1, 0, 4, 3));
  ASSERT_FALSE(result.depth_m.isContinuous());
  const float invalid[] = {std::numeric_limits<float>::quiet_NaN(),
                           std::numeric_limits<float>::infinity(), 0, -1};
  for (int col = 0; col < 4; ++col) {
    result.depth_m.at<float>(1, col) = invalid[col];
  }
  const auto cloud = core::PointCloudGenerator::generate(result, {});
  storage.setTo(9);
  EXPECT_FLOAT_EQ(cloud.points[0].z_m, 2);
  for (int i = 4; i < 8; ++i) {
    EXPECT_TRUE(std::isnan(cloud.points[i].x_m));
    EXPECT_TRUE(std::isnan(cloud.points[i].y_m));
    EXPECT_TRUE(std::isnan(cloud.points[i].z_m));
  }
}
TEST(PointCloud, CompactStrideSamplesImageGridAndOmitsInvalidPoints)
{
  auto result = depth_result();
  result.depth_m.at<float>(0, 2) = std::numeric_limits<float>::quiet_NaN();
  core::PointsConfig config;
  config.layout = core::PointsLayout::kCompact;
  config.stride = 2;
  const auto cloud = core::PointCloudGenerator::generate(result, config);
  ASSERT_EQ(cloud.points.size(), 3U);
  EXPECT_EQ(cloud.width, 3U);
  EXPECT_EQ(cloud.height, 1U);
  EXPECT_FLOAT_EQ(cloud.points[1].x_m, -1.5F);
  EXPECT_FLOAT_EQ(cloud.points[1].y_m, 0.5F);
  EXPECT_FLOAT_EQ(cloud.points[2].x_m, 0.5F);
  result.depth_m.setTo(std::numeric_limits<float>::quiet_NaN());
  const auto empty = core::PointCloudGenerator::generate(result, config);
  EXPECT_TRUE(empty.points.empty());
  EXPECT_EQ(empty.width, 0U);
  EXPECT_EQ(empty.height, 1U);
  config.stride = std::numeric_limits<int>::max();
  result.depth_m.setTo(2);
  EXPECT_EQ(core::PointCloudGenerator::generate(result, config).points.size(), 1U);
}
TEST(PointCloud, RejectsMalformedDepthCalibrationAndLayout)
{
  auto result = depth_result();
  core::PointsConfig config;
  config.stride = 2;
  EXPECT_THROW(core::PointCloudGenerator::generate(result, config), std::invalid_argument);
  config.layout = core::PointsLayout::kCompact;
  config.stride = 0;
  EXPECT_THROW(core::PointCloudGenerator::generate(result, config), std::invalid_argument);
  config.stride = 1;
  config.layout = static_cast<core::PointsLayout>(99);
  EXPECT_THROW(core::PointCloudGenerator::generate(result, config), std::invalid_argument);
  result.calibration.k[0] = 0;
  EXPECT_THROW(core::PointCloudGenerator::generate(result, {}), std::invalid_argument);
  result = depth_result();
  result.calibration.k[2] = std::numeric_limits<double>::infinity();
  EXPECT_THROW(core::PointCloudGenerator::generate(result, {}), std::invalid_argument);
  result = depth_result();
  result.calibration.width = 5;
  EXPECT_THROW(core::PointCloudGenerator::generate(result, {}), std::invalid_argument);
  result = depth_result();
  result.depth_m = cv::Mat(3, 4, CV_16UC1);
  EXPECT_THROW(core::PointCloudGenerator::generate(result, {}), std::invalid_argument);
  result = depth_result();
  result.optical_frame_id.clear();
  EXPECT_THROW(core::PointCloudGenerator::generate(result, {}), std::invalid_argument);
}
} // namespace
