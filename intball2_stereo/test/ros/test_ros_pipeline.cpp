#include "../core/stereo_test_fixture.hpp"
#include "intball2_stereo/core/stereo_processor.hpp"
#include "intball2_stereo/ros/stereo_input.hpp"
#include "intball2_stereo/ros/stereo_worker.hpp"
#include <chrono>
#include <cmath>
#include <cstring>
#include <future>
#include <gtest/gtest.h>
#include <limits>
namespace intball2_stereo::test
{
namespace
{
using namespace std::chrono_literals;
using ros::InputBundle;
auto image(std::uint32_t stamp = 0, const std::string &frame_id = "left_optical")
{
  auto m = std::make_shared<sensor_msgs::msg::Image>();
  m->header.stamp.nanosec = stamp;
  m->header.frame_id = frame_id;
  m->width = 200;
  m->height = 200;
  m->step = 204;
  m->encoding = "mono8";
  m->data.resize(m->step * m->height, 42);
  return m;
}
auto info(std::uint32_t stamp = 0, const std::string &frame_id = "left_optical")
{
  auto m = std::make_shared<sensor_msgs::msg::CameraInfo>();
  auto c = calibration();
  m->header.stamp.nanosec = stamp;
  m->header.frame_id = frame_id;
  m->width = c.width;
  m->height = c.height;
  m->k = c.k;
  m->p = c.p;
  m->r = c.r;
  m->d = c.d;
  m->distortion_model = c.distortion_model;
  return m;
}
InputBundle bundle(std::uint32_t stamp = 0)
{
  return {image(stamp), image(stamp, "right_optical"), info(stamp), info(stamp, "right_optical")};
}
void feed(ros::StereoInput &sync, const InputBundle &b)
{
  sync.add_image(false, b.left_image);
  sync.add_info(true, b.right_info);
  sync.add_image(true, b.right_image);
  sync.add_info(false, b.left_info);
}
TEST(Messages, BorrowedPaddedImageKeepsOwnerAndCalibration)
{
  auto b = bundle(17);
  std::weak_ptr<const sensor_msgs::msg::Image> owner = b.left_image;
  auto pair = ros::image_pair(b);
  EXPECT_EQ(pair.left.image.pixels().data, b.left_image->data.data());
  EXPECT_EQ(pair.left.image.pixels().step, 204U);
  EXPECT_EQ(pair.left.stamp.nanoseconds, 17);
  EXPECT_EQ(pair.right.optical_frame_id, "right_optical");
  EXPECT_EQ(pair.left.calibration.p, b.left_info->p);
  b = {};
  EXPECT_FALSE(owner.expired());
  EXPECT_EQ(pair.left.image.pixels().at<unsigned char>(199, 199), 42);
}
TEST(Messages, GazeboLinkFramesAreNormalizedWithoutChangingCalibration)
{
  InputBundle b{image(1, "cameraL_link"), image(1, "cameraR_link"), info(1, "cameraL_link"),
                info(1, "cameraR_link")};
  auto pair = ros::image_pair(b, core::CalibrationMode::kGazeboNegative5cm);
  EXPECT_EQ(pair.left.optical_frame_id, "cameraL_optical_frame");
  EXPECT_EQ(pair.right.optical_frame_id, "cameraR_optical_frame");
  EXPECT_EQ(pair.left.calibration.p, b.left_info->p);
  EXPECT_EQ(b.left_image->header.frame_id, "cameraL_link");
  EXPECT_EQ(ros::image_pair(b).left.optical_frame_id, "cameraL_link");
  b.left_image = image(1, "cameraL_optical_frame");
  EXPECT_EQ(ros::image_pair(b, core::CalibrationMode::kGazeboNegative5cm).left.optical_frame_id,
            "cameraL_optical_frame");
}
TEST(Messages, RejectsMalformedInputs)
{
  auto b = bundle();
  auto m = image();
  b.left_image = m;
  m->encoding = "16UC1";
  EXPECT_THROW(ros::image_pair(b), std::invalid_argument);
  m->encoding = "mono8";
  m->data.resize(1);
  EXPECT_THROW(ros::image_pair(b), std::invalid_argument);
  m->data.resize(m->step * m->height);
  m->step = 1;
  EXPECT_THROW(ros::image_pair(b), std::invalid_argument);
  m->step = 204;
  m->header.frame_id = "wrong";
  EXPECT_THROW(ros::image_pair(b), std::invalid_argument);
  m->header.frame_id = "left_optical";
  m->header.stamp.nanosec = 1000000000;
  EXPECT_THROW(ros::image_pair(b), std::invalid_argument);
  b.left_image.reset();
  EXPECT_THROW(ros::image_pair(b), std::invalid_argument);
}
TEST(Messages, PartialPairFailureReleasesBorrowedOwner)
{
  auto b = bundle();
  std::weak_ptr<const sensor_msgs::msg::Image> left = b.left_image;
  auto invalid_right = image();
  invalid_right->header.stamp.nanosec = 1000000000;
  b.right_image = invalid_right;
  EXPECT_THROW(ros::image_pair(b), std::invalid_argument);
  b = {};
  EXPECT_TRUE(left.expired());
}
TEST(Messages, ColorEncodingsAndInfoPreserveSignedValues)
{
  auto b = bundle();
  auto m = image();
  m->step = 604;
  m->data.resize(m->step * m->height);
  b.left_image = m;
  m->encoding = "rgb8";
  EXPECT_EQ(ros::image_pair(b).left.encoding, core::ImageEncoding::kRgb8);
  m->encoding = "bgr8";
  EXPECT_EQ(ros::image_pair(b).left.encoding, core::ImageEncoding::kBgr8);
  auto c = info();
  c->p[3] = -5;
  c->roi.x_offset = 7;
  c->binning_x = 2;
  auto converted = ros::camera_calibration(*c);
  EXPECT_EQ(converted.p[3], -5);
  EXPECT_EQ(converted.roi.x_offset, 7U);
  EXPECT_EQ(converted.binning_x, 2U);
}
TEST(Messages, OutputOwnsNonContiguousDepthAndPreservesNanHeaderProjection)
{
  cv::Mat storage(3, 5, CV_32FC1, cv::Scalar(1.25));
  core::DepthResult result;
  result.depth_m = storage(cv::Rect(1, 0, 3, 3));
  result.depth_m.at<float>(1, 1) = std::numeric_limits<float>::quiet_NaN();
  result.calibration = calibration(3, 3);
  result.stamp.nanoseconds = -1;
  result.optical_frame_id = "left_optical";
  auto output = ros::output_messages(result);
  EXPECT_EQ(output.depth.encoding, "32FC1");
  EXPECT_EQ(output.depth.step, 12U);
  EXPECT_EQ(output.depth.data.size(), 36U);
  EXPECT_EQ(output.depth.header.stamp.sec, -1);
  EXPECT_EQ(output.depth.header.stamp.nanosec, 999999999U);
  EXPECT_EQ(output.camera_info.header, output.depth.header);
  EXPECT_EQ(output.camera_info.k, result.calibration.k);
  EXPECT_EQ(output.camera_info.p, result.calibration.p);
  storage.setTo(9);
  float value;
  std::memcpy(&value, output.depth.data.data(), sizeof(value));
  EXPECT_FLOAT_EQ(value, 1.25F);
  std::memcpy(&value, output.depth.data.data() + 16, sizeof(value));
  EXPECT_TRUE(std::isnan(value));
  result.depth_m = cv::Mat();
  EXPECT_THROW(ros::output_messages(result), std::invalid_argument);
}
TEST(Synchronization, FourStreamsAndInclusiveInterval)
{
  std::vector<InputBundle> outputs;
  ros::StereoInput sync({2, 0.000000010}, [&](auto b) { outputs.push_back(std::move(b)); });
  auto b = bundle(100);
  b.right_image = image(110, "right_optical");
  sync.add_image(false, b.left_image);
  sync.add_image(true, b.right_image);
  sync.add_info(false, b.left_info);
  EXPECT_TRUE(outputs.empty());
  sync.add_info(true, b.right_info);
  ASSERT_EQ(outputs.size(), 1U);
  EXPECT_EQ(outputs[0].right_image, b.right_image);
  feed(sync, b); // duplicate stamps do not reuse data
  EXPECT_EQ(outputs.size(), 1U);
}
TEST(Synchronization, BoundedQueueDropsOldestAndRejectsOldArrivals)
{
  std::vector<InputBundle> outputs;
  ros::StereoInput sync({1, 0}, [&](auto b) { outputs.push_back(std::move(b)); });
  auto a = bundle(10);
  auto b = bundle(20);
  sync.add_image(false, a.left_image);
  sync.add_image(false, b.left_image);
  feed(sync, a);
  EXPECT_TRUE(outputs.empty());
  feed(sync, b);
  ASSERT_EQ(outputs.size(), 1U);
  EXPECT_EQ(outputs[0].left_image, b.left_image);
}
TEST(Synchronization, OutsideIntervalIsDiscardedWithoutStaleCalibration)
{
  int count = 0;
  ros::StereoInput sync({5, 0.000000010}, [&](auto) { ++count; });
  auto b = bundle(100);
  b.right_info = info(111, "right_optical");
  feed(sync, b);
  EXPECT_EQ(count, 0);
  feed(sync, bundle(200));
  EXPECT_EQ(count, 1);
  EXPECT_THROW((ros::StereoInput({0, 0}, [](auto) {})), std::invalid_argument);
  EXPECT_THROW(sync.add_info(false, {}), std::invalid_argument);
}
TEST(Worker, LatestPendingOnlyAndFailureRecovery)
{
  std::promise<void> entered, release, finished;
  auto released = release.get_future().share();
  std::vector<std::int64_t> processed;
  int errors = 0;
  ros::StereoWorker worker(
      [&](const InputBundle &b) {
        auto stamp = ros::stamp_nanoseconds(b.left_image->header.stamp);
        processed.push_back(stamp);
        if (stamp == 1) {
          entered.set_value();
          released.wait();
          throw std::runtime_error("test failure");
        }
        finished.set_value();
      },
      [&](auto error) {
        ++errors;
        EXPECT_THROW(std::rethrow_exception(error), std::runtime_error);
        throw std::runtime_error("reporting failure");
      });
  worker.submit(bundle(1));
  auto status = entered.get_future().wait_for(5s);
  EXPECT_EQ(status, std::future_status::ready);
  if (status != std::future_status::ready) {
    release.set_value();
    worker.stop();
    return;
  }
  worker.submit(bundle(2));
  worker.submit(bundle(3));
  EXPECT_EQ(worker.dropped_pairs(), 1U);
  release.set_value();
  EXPECT_EQ(finished.get_future().wait_for(5s), std::future_status::ready);
  worker.stop();
  EXPECT_EQ(processed, (std::vector<std::int64_t>{1, 3}));
  EXPECT_EQ(errors, 1);
  EXPECT_FALSE(worker.submit(bundle(4)));
  worker.stop();
}
TEST(Worker, StopDiscardsPendingAndWaitsForActiveJob)
{
  std::promise<void> entered, release;
  auto released = release.get_future().share();
  int count = 0;
  ros::StereoWorker worker(
      [&](auto &) {
        ++count;
        entered.set_value();
        released.wait();
      },
      [](auto) {});
  worker.submit(bundle(1));
  auto status = entered.get_future().wait_for(5s);
  EXPECT_EQ(status, std::future_status::ready);
  if (status != std::future_status::ready) {
    release.set_value();
    worker.stop();
    return;
  }
  worker.submit(bundle(2));
  auto stopping = std::async(std::launch::async, [&] { worker.stop(); });
  // stop may start after release; use a bounded yield loop to observe its state.
  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (worker.dropped_pairs() == 0 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::yield();
  }
  EXPECT_EQ(worker.dropped_pairs(), 1U);
  release.set_value();
  EXPECT_EQ(stopping.wait_for(5s), std::future_status::ready);
  stopping.get();
  EXPECT_EQ(count, 1);
}
TEST(Pipeline, SyntheticPairThroughWorkerProducesDepthAndCameraInfo)
{
  auto b = bundle(123);
  auto left = image(123), right = image(124, "right_optical");
  cv::Mat l(200, 200, CV_8UC1, left->data.data(), left->step);
  cv::Mat r(200, 200, CV_8UC1, right->data.data(), right->step);
  cv::RNG random(42);
  random.fill(l, cv::RNG::UNIFORM, 0, 256);
  r.setTo(0);
  l(cv::Rect(0, 0, 192, 200)).copyTo(r(cv::Rect(8, 0, 192, 200)));
  b.left_image = left;
  b.right_image = right;
  core::StereoConfig config;
  config.processing.downsample = 1;
  config.matcher.speckle_window_size_px = 0;
  config.matcher.uniqueness_ratio = 0;
  core::StereoProcessor processor(config);
  std::promise<ros::OutputMessages> done;
  ros::StereoWorker worker(
      [&](auto &input) {
        done.set_value(ros::output_messages(processor.process(ros::image_pair(input))));
      },
      [&](auto error) { done.set_exception(error); });
  ros::StereoInput sync({5, 0.02}, [&](auto input) { worker.submit(std::move(input)); });
  auto result = done.get_future();
  feed(sync, b);
  const auto status = result.wait_for(5s);
  worker.stop();
  ASSERT_EQ(status, std::future_status::ready);
  auto output = result.get();
  EXPECT_EQ(output.depth.header.stamp.nanosec, 123U);
  EXPECT_EQ(output.camera_info.header, output.depth.header);
  EXPECT_EQ(output.camera_info.width, 200U);
  float z;
  std::memcpy(&z, output.depth.data.data() + 100 * output.depth.step + 100 * sizeof(float),
              sizeof(z));
  EXPECT_NEAR(z, 0.625F, 0.01F);
}
} // namespace
} // namespace intball2_stereo::test
