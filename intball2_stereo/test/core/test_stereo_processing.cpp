#include <algorithm>
#include <cmath>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>

#include "intball2_stereo/core/stereo_processor.hpp"
#include "stereo_test_fixture.hpp"

namespace
{
using intball2_stereo::core::CalibrationMode;
using intball2_stereo::core::CameraConfig;
using intball2_stereo::core::CameraModel;
using intball2_stereo::core::ImageEncoding;
using intball2_stereo::core::ImagePair;
using intball2_stereo::core::ImagePreprocessor;
using intball2_stereo::core::InputGeometry;
using intball2_stereo::core::MatcherConfig;
using intball2_stereo::core::SgbmMatcher;
using intball2_stereo::core::StereoConfig;
using intball2_stereo::core::StereoProcessor;
using intball2_stereo::test::calibration;
using intball2_stereo::test::frame;

CameraConfig standard_camera(InputGeometry geometry = InputGeometry::kAlreadyRectified)
{
  CameraConfig config;
  config.calibration_mode = CalibrationMode::kCameraInfo;
  config.input_geometry = geometry;
  return config;
}

TEST(ImagePreprocessing, AreaResizeColourEncodingAndOwnedOutput)
{
  const auto left = calibration(800, 800, 400);
  const auto right = calibration(800, 800, 400, -20);
  cv::Mat pixels(800, 800, CV_8UC3, cv::Scalar(255, 0, 0));
  auto left_frame = frame(pixels, left);
  auto right_frame = frame(pixels, right, "right_optical");
  left_frame.encoding = ImageEncoding::kBgr8;
  right_frame.encoding = ImageEncoding::kRgb8;
  ImagePair pair{left_frame, right_frame};
  ImagePreprocessor preprocessor(CameraModel(left, right, standard_camera(), 4), 5);
  const auto &output = preprocessor.prepare(pair);
  EXPECT_EQ(output.left_gray.size(), cv::Size(200, 200));
  EXPECT_EQ(output.left_gray.type(), CV_8UC1);
  EXPECT_EQ(output.left_gray.at<unsigned char>(100, 100), 29);
  EXPECT_EQ(output.right_gray.at<unsigned char>(100, 100), 76);
  EXPECT_NE(output.left_gray.data, pair.left.image.pixels().data);
  const auto *buffer = output.left_gray.data;
  preprocessor.prepare(pair);
  EXPECT_EQ(buffer, output.left_gray.data);
}

TEST(ImagePreprocessing, ColourResizeReorderingHasBoundedRoundingDifference)
{
  const auto left = calibration(64, 64, 40);
  const auto right = calibration(64, 64, 40, -2);
  cv::Mat pixels(64, 64, CV_8UC3);
  cv::RNG random(772);
  random.fill(pixels, cv::RNG::UNIFORM, 0, 256);
  const auto original = pixels.clone();
  cv::Mat resized;
  cv::resize(pixels, resized, cv::Size(16, 16), 0, 0, cv::INTER_AREA);
  for (const bool bgr : {false, true}) {
    auto left_frame = frame(pixels, left);
    auto right_frame = frame(pixels, right, "right_optical");
    left_frame.encoding = right_frame.encoding = bgr ? ImageEncoding::kBgr8 : ImageEncoding::kRgb8;
    ImagePreprocessor preprocessor(CameraModel(left, right, standard_camera(), 4), 5);
    const auto &output = preprocessor.prepare({left_frame, right_frame});
    cv::Mat previous_order, difference;
    cv::cvtColor(resized, previous_order, bgr ? cv::COLOR_BGR2GRAY : cv::COLOR_RGB2GRAY);
    cv::absdiff(output.left_gray, previous_order, difference);
    double maximum;
    cv::minMaxLoc(difference, nullptr, &maximum);
    EXPECT_LE(maximum, 1);
    EXPECT_GT(maximum, 0);
    EXPECT_EQ(cv::norm(pixels, original, cv::NORM_INF), 0);
    EXPECT_NE(output.left_gray.data, pixels.data);
  }
}

TEST(ImagePreprocessing, RectifiedResizeMatchesReferenceAndRetainsSourceGray)
{
  const auto left = calibration(800, 800, 400);
  const auto right = calibration(800, 800, 400, -20);
  const int original_threads = cv::getNumThreads();
  for (const int threads : {0, 4}) {
    cv::setNumThreads(threads);
    for (const auto encoding :
         {ImageEncoding::kMono8, ImageEncoding::kBgr8, ImageEncoding::kRgb8}) {
      const bool mono = encoding == ImageEncoding::kMono8;
      auto storage = std::make_shared<cv::Mat>(800, 804, mono ? CV_8UC1 : CV_8UC3);
      cv::RNG random(20261011);
      random.fill(*storage, cv::RNG::UNIFORM, 0, 256);
      const cv::Mat pixels = (*storage)(cv::Rect(1, 0, 800, 800));
      EXPECT_FALSE(pixels.isContinuous());
      const auto original = pixels.clone();
      auto left_frame = frame(pixels, left);
      auto right_frame = frame(pixels, right, "right_optical");
      left_frame.image = right_frame.image =
          intball2_stereo::core::ImageBuffer::borrow(pixels, storage);
      left_frame.encoding = right_frame.encoding = encoding;
      cv::Mat source_gray;
      if (mono) {
        source_gray = pixels;
      } else {
        cv::cvtColor(pixels, source_gray,
                     encoding == ImageEncoding::kBgr8 ? cv::COLOR_BGR2GRAY : cv::COLOR_RGB2GRAY);
      }
      for (const int factor : {1, 2, 4}) {
        SCOPED_TRACE(::testing::Message()
                     << "encoding=" << static_cast<int>(encoding) << " downsample=" << factor);
        ImagePreprocessor preprocessor(CameraModel(left, right, standard_camera(), factor), 5);
        const auto &output = preprocessor.prepare({left_frame, right_frame});
        cv::Mat expected;
        cv::resize(source_gray, expected, cv::Size(800 / factor, 800 / factor), 0, 0,
                   cv::INTER_AREA);
        EXPECT_EQ(cv::norm(output.left_gray, expected, cv::NORM_INF), 0);
        EXPECT_EQ(cv::norm(output.right_gray, expected, cv::NORM_INF), 0);
        EXPECT_EQ(cv::norm(output.left_source_gray.pixels, source_gray, cv::NORM_INF), 0);
        EXPECT_EQ(output.left_source_gray.source_owner.has_value(), mono);
        EXPECT_EQ(output.left_source_gray.source_data, pixels.data);
        EXPECT_NE(output.left_gray.data, pixels.data);
        EXPECT_NE(output.left_gray.data, output.left_source_gray.pixels.data);
        EXPECT_EQ(cv::countNonZero(output.left_valid), expected.total());
        EXPECT_EQ(cv::norm(pixels, original, cv::NORM_INF), 0);
        const auto *owned_buffer = output.left_gray.data;
        preprocessor.prepare({left_frame, right_frame});
        EXPECT_EQ(output.left_gray.data, owned_buffer);
      }
    }
  }
  cv::setNumThreads(original_threads == 1 ? 0 : original_threads);
}

TEST(ImagePreprocessing, FourPixelAreaResizeMatchesEveryPossibleBlockSum)
{
  constexpr int kMaximumSum = 16 * 255;
  const int width = 4 * (kMaximumSum + 1);
  auto storage = std::make_shared<cv::Mat>(4, width + 3, CV_8UC1, cv::Scalar(0));
  cv::Mat pixels = (*storage)(cv::Rect(1, 0, width, 4));
  for (int sum = 0; sum <= kMaximumSum; ++sum) {
    int remaining = sum;
    for (int sample = 0; sample < 16; ++sample) {
      const int value = std::min(remaining, 255);
      pixels.at<unsigned char>(sample / 4, 4 * sum + sample % 4) = value;
      remaining -= value;
    }
  }
  const auto left = calibration(width, 4, 40);
  const auto right = calibration(width, 4, 40, -2);
  ImagePair pair{
      {intball2_stereo::core::ImageBuffer::borrow(pixels, storage), left, {123}, "left_optical"},
      {intball2_stereo::core::ImageBuffer::borrow(pixels, storage), right, {123}, "right_optical"}};
  const int original_threads = cv::getNumThreads();
  for (const int threads : {1, 4}) {
    cv::setNumThreads(threads == 1 ? 0 : threads);
    cv::Mat reference;
    cv::resize(pixels, reference, cv::Size(kMaximumSum + 1, 1), 0, 0, cv::INTER_AREA);
    ImagePreprocessor preprocessor(CameraModel(left, right, standard_camera(), 4), 5);
    const auto &output = preprocessor.prepare(pair);
    EXPECT_EQ(cv::norm(output.left_gray, reference, cv::NORM_INF), 0);
    EXPECT_EQ(cv::norm(output.right_gray, reference, cv::NORM_INF), 0);
  }
  cv::setNumThreads(original_threads == 1 ? 0 : original_threads);
}

TEST(ImagePreprocessing, RawIdentityAndDistortionAreHandledWithoutDoubleRectification)
{
  auto left = calibration();
  auto right = calibration(200, 200, 100, -5);
  cv::Mat ramp(200, 200, CV_8UC1);
  for (int row = 0; row < ramp.rows; ++row) {
    for (int column = 0; column < ramp.cols; ++column) {
      ramp.at<unsigned char>(row, column) = column;
    }
  }
  ImagePair pair{frame(ramp, left), frame(ramp, right, "right_optical")};
  ImagePreprocessor identity(CameraModel(left, right, standard_camera(InputGeometry::kRaw), 1), 5);
  EXPECT_EQ(identity.prepare(pair).left_gray.at<unsigned char>(100, 150), 150);
  left.d[0] = right.d[0] = 0.5;
  pair.left.calibration = left;
  pair.right.calibration = right;
  ImagePreprocessor raw(CameraModel(left, right, standard_camera(InputGeometry::kRaw), 1), 5);
  const auto &corrected = raw.prepare(pair);
  // Analytic radial-distortion projection into a linear horizontal intensity ramp.
  const double x = (150 - left.k[2]) / left.k[0];
  const double y = (100 - left.k[5]) / left.k[4];
  const double raw_u = left.k[0] * x * (1 + 0.5 * (x * x + y * y)) + left.k[2];
  EXPECT_NEAR(corrected.left_gray.at<unsigned char>(100, 150), raw_u, 0.6);
  EXPECT_EQ(corrected.left_valid.at<unsigned char>(100, 199), 0);
  EXPECT_EQ(corrected.left_valid.at<unsigned char>(100, 100), 255);
  ImagePreprocessor rectified(CameraModel(left, right, standard_camera(), 1), 5);
  EXPECT_EQ(rectified.prepare(pair).left_gray.at<unsigned char>(100, 150), 150);
}

TEST(ImagePreprocessing, TypeShapeAndChangedCalibrationAreRejected)
{
  const auto left = calibration();
  const auto right = calibration(200, 200, 100, -5);
  ImagePreprocessor preprocessor(CameraModel(left, right, standard_camera(), 1), 5);
  cv::Mat wrong_type(200, 200, CV_16UC1, cv::Scalar(0));
  ImagePair pair{frame(wrong_type, left), frame(wrong_type, right, "right_optical")};
  EXPECT_THROW(preprocessor.prepare(pair), std::invalid_argument);
  cv::Mat pixels(200, 200, CV_8UC1, cv::Scalar(0));
  pair = {frame(pixels, left), frame(pixels, right, "right_optical")};
  pair.left.encoding = static_cast<ImageEncoding>(99);
  EXPECT_THROW(preprocessor.prepare(pair), std::invalid_argument);
  pair.left.encoding = ImageEncoding::kMono8;
  pair.left.calibration.p[2] += 1;
  EXPECT_THROW(preprocessor.prepare(pair), std::invalid_argument);
  pair.left.calibration = left;
  cv::Mat wrong_shape(100, 100, CV_8UC1, cv::Scalar(0));
  pair.left.image = intball2_stereo::core::ImageBuffer::copy_from(wrong_shape);
  EXPECT_THROW(preprocessor.prepare(pair), std::invalid_argument);
}

TEST(StereoMatching, BothDisparitySignsAndAllResolutionsReconstructKnownPlane)
{
  cv::Mat texture(200, 200, CV_8UC1);
  cv::RNG random(20261010);
  random.fill(texture, cv::RNG::UNIFORM, 0, 256);
  cv::Mat left_pixels;
  cv::resize(texture, left_pixels, cv::Size(800, 800), 0, 0, cv::INTER_NEAREST);
  for (const int sign : {-1, 1}) {
    cv::Mat right_pixels(800, 800, CV_8UC1, cv::Scalar(0));
    const int shift = 32;
    const int left_begin = sign > 0 ? shift : 0;
    const int right_begin = sign > 0 ? 0 : shift;
    left_pixels(cv::Rect(left_begin, 0, 800 - shift, 800))
        .copyTo(right_pixels(cv::Rect(right_begin, 0, 800 - shift, 800)));
    for (const int factor : {1, 2, 4}) {
      SCOPED_TRACE(::testing::Message() << "sign=" << sign << " downsample=" << factor);
      StereoConfig config;
      config.processing.downsample = factor;
      config.matcher.min_disparity_px = sign > 0 ? 0
                                                 : (factor == 1   ? -98
                                                    : factor == 2 ? -49
                                                                  : -25);
      config.matcher.num_disparities_px = 128 / factor;
      const auto left = calibration(800, 800, 400, sign < 0 ? 7 : 0);
      const auto right = calibration(800, 800, 400, sign < 0 ? 7 : -20);
      if (sign > 0) {
        config.camera = standard_camera();
      }
      ImagePair pair{frame(left_pixels, left), frame(right_pixels, right, "right_optical")};
      StereoProcessor processor(config);
      const auto &result = processor.process(pair);
      const int width = 800 / factor;
      EXPECT_EQ(result.depth_m.size(), cv::Size(width, width));
      EXPECT_EQ(result.depth_m.type(), CV_32FC1);
      std::vector<float> central_depths;
      for (int row = width / 4; row < 3 * width / 4; ++row) {
        for (int column = width / 4; column < 3 * width / 4; ++column) {
          const auto z = result.depth_m.at<float>(row, column);
          ASSERT_FALSE(std::isinf(z));
          if (std::isfinite(z)) {
            central_depths.push_back(z);
          }
        }
      }
      ASSERT_GT(central_depths.size(), static_cast<std::size_t>(width * width * 0.23));
      std::sort(central_depths.begin(), central_depths.end());
      // 400px * .05m / 32px = .625m, independent of processing resolution/sign.
      EXPECT_NEAR(central_depths[central_depths.size() / 2], 0.625, 0.015);
      EXPECT_NEAR(central_depths[central_depths.size() * 9 / 10], 0.625, 0.03);
      EXPECT_EQ(result.stamp.nanoseconds, 123456789);
      EXPECT_EQ(result.optical_frame_id, "left_optical");
      EXPECT_DOUBLE_EQ(result.calibration.k[0], 400.0 / factor);
      EXPECT_DOUBLE_EQ(result.calibration.k[2], (399.5 + 0.5) / factor - 0.5);
      EXPECT_EQ(result.calibration.k[2], result.calibration.p[2]);
      EXPECT_TRUE(result.calibration.d.empty());
      EXPECT_GE(result.statistics.matching_s, 0);
      EXPECT_GT(result.statistics.valid_depth_fraction, 0.6);
      const auto *storage = result.depth_m.data;
      pair.left.stamp.nanoseconds += 17;
      const auto &next = processor.process(pair);
      EXPECT_EQ(storage, next.depth_m.data);
      EXPECT_EQ(next.stamp.nanoseconds, 123456806);
      pair.left.calibration.p[2] += 1;
      pair.right.calibration.p[2] += 1;
      const auto &changed = processor.process(pair);
      EXPECT_DOUBLE_EQ(changed.calibration.k[2], (400.5 + 0.5) / factor - 0.5);
    }
  }
}

TEST(StereoMatching, FlatImagesDoNotBecomeFreeSpace)
{
  const cv::Mat flat(200, 200, CV_8UC1, cv::Scalar(128));
  auto left = calibration();
  auto right = calibration();
  ImagePair pair{frame(flat, left), frame(flat, right, "right_optical")};
  StereoConfig config;
  config.processing.downsample = 1;
  StereoProcessor processor(config);
  const auto &result = processor.process(pair);
  EXPECT_EQ(result.depth_m.type(), CV_32FC1);
  for (int row = 0; row < 200; ++row) {
    for (int column = 0; column < 200; ++column) {
      EXPECT_FALSE(std::isinf(result.depth_m.at<float>(row, column)));
    }
  }
}

TEST(StereoMatching, InvalidMatcherInputAndOpticalFrameChangesAreRejected)
{
  MatcherConfig matcher;
  SgbmMatcher sgbm(matcher);
  cv::Mat pixels(200, 200, CV_8UC1, cv::Scalar(0));
  cv::Mat wrong(200, 200, CV_8UC3, cv::Scalar(0));
  EXPECT_THROW(sgbm.compute(pixels, wrong), std::invalid_argument);
  EXPECT_THROW(sgbm.compute(cv::Mat{}, pixels), std::invalid_argument);
  StereoConfig config;
  config.camera = standard_camera(InputGeometry::kRaw);
  config.matcher.min_disparity_px = 0;
  config.processing.downsample = 1;
  auto left = calibration();
  auto right = calibration(200, 200, 100, -5);
  ImagePair pair{frame(pixels, left), frame(pixels, right, "right_optical")};
  StereoProcessor processor(config);
  pair.left.optical_frame_id.clear();
  EXPECT_THROW(processor.process(pair), std::invalid_argument);
  pair.left.optical_frame_id = "left_optical";
  left.r = {0, -1, 0, 1, 0, 0, 0, 0, 1};
  pair.left.calibration = left;
  EXPECT_THROW(processor.process(pair), std::invalid_argument);
}
} // namespace
