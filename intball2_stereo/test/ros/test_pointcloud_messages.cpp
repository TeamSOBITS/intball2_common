#include <cmath>
#include <cstring>
#include <limits>

#include <gtest/gtest.h>

#include "intball2_stereo/ros/stereo_messages.hpp"

namespace
{
using namespace intball2_stereo;
core::PointCloudResult cloud_result()
{
  core::PointCloudResult result;
  result.points = {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}, {10, 11, 12}};
  result.width = 2;
  result.height = 2;
  result.stamp.nanoseconds = -1;
  result.optical_frame_id = "left_optical";
  return result;
}
float value(const sensor_msgs::msg::PointCloud2 &message, std::size_t offset)
{
  float result;
  std::memcpy(&result, message.data.data() + offset, sizeof(result));
  return result;
}
TEST(PointCloudMessages, PackedNativeFloatXyzOwnsBytesAndPreservesHeader)
{
  auto result = cloud_result();
  const auto output = ros::pointcloud_message(result);
  EXPECT_EQ(output.header.stamp.sec, -1);
  EXPECT_EQ(output.header.stamp.nanosec, 999999999U);
  EXPECT_EQ(output.header.frame_id, "left_optical");
  EXPECT_EQ(output.width, 2U);
  EXPECT_EQ(output.height, 2U);
  EXPECT_EQ(output.point_step, 12U);
  EXPECT_EQ(output.row_step, 24U);
  EXPECT_EQ(output.data.size(), 48U);
  EXPECT_TRUE(output.is_dense);
  const std::uint16_t probe = 1;
  EXPECT_EQ(output.is_bigendian, *reinterpret_cast<const unsigned char *>(&probe) == 0);
  ASSERT_EQ(output.fields.size(), 3U);
  const char *names[] = {"x", "y", "z"};
  for (unsigned i = 0; i < 3; ++i) {
    EXPECT_EQ(output.fields[i].name, names[i]);
    EXPECT_EQ(output.fields[i].offset, i * 4);
    EXPECT_EQ(output.fields[i].datatype, sensor_msgs::msg::PointField::FLOAT32);
    EXPECT_EQ(output.fields[i].count, 1U);
  }
  result.points.clear();
  EXPECT_FLOAT_EQ(value(output, 0), 1);
  EXPECT_FLOAT_EQ(value(output, 4), 2);
  EXPECT_FLOAT_EQ(value(output, 8), 3);
  EXPECT_FLOAT_EQ(value(output, 44), 12);
}
TEST(PointCloudMessages, DenseReflectsEveryCoordinateAndEmptyCompactIsValid)
{
  auto result = cloud_result();
  result.points[2].y_m = std::numeric_limits<float>::quiet_NaN();
  auto output = ros::pointcloud_message(result);
  EXPECT_FALSE(output.is_dense);
  EXPECT_TRUE(std::isnan(value(output, 28)));
  result.points[2].y_m = 8;
  result.points[0].x_m = std::numeric_limits<float>::infinity();
  EXPECT_FALSE(ros::pointcloud_message(result).is_dense);
  result.layout = core::PointsLayout::kCompact;
  result.height = 1;
  result.width = 0;
  result.points.clear();
  output = ros::pointcloud_message(result);
  EXPECT_TRUE(output.data.empty());
  EXPECT_EQ(output.row_step, 0U);
  EXPECT_EQ(output.height, 1U);
  EXPECT_TRUE(output.is_dense);
}
TEST(PointCloudMessages, RejectsMalformedShapeFrameLayoutAndStampWithoutAllocating)
{
  auto result = cloud_result();
  result.width = 3;
  EXPECT_THROW(ros::pointcloud_message(result), std::invalid_argument);
  result = cloud_result();
  result.layout = core::PointsLayout::kCompact;
  EXPECT_THROW(ros::pointcloud_message(result), std::invalid_argument);
  result = cloud_result();
  result.layout = static_cast<core::PointsLayout>(99);
  EXPECT_THROW(ros::pointcloud_message(result), std::invalid_argument);
  result = cloud_result();
  result.width = std::numeric_limits<std::uint32_t>::max();
  EXPECT_THROW(ros::pointcloud_message(result), std::invalid_argument);
  result = cloud_result();
  result.height = 0;
  EXPECT_THROW(ros::pointcloud_message(result), std::invalid_argument);
  result = cloud_result();
  result.optical_frame_id.clear();
  EXPECT_THROW(ros::pointcloud_message(result), std::invalid_argument);
  result = cloud_result();
  result.stamp.nanoseconds = std::numeric_limits<std::int64_t>::max();
  EXPECT_THROW(ros::pointcloud_message(result), std::invalid_argument);
}
} // namespace
