#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>

#include "intball2_stereo/core/depth_refiner.hpp"
#include "intball2_stereo/core/image_preprocessor.hpp"
#include "stereo_test_fixture.hpp"

namespace
{
using namespace intball2_stereo::core;
using intball2_stereo::test::calibration;
using intball2_stereo::test::frame;

TEST(DepthRefinement, IntegerQuarterMomentsMatchFloatReferenceExactly)
{
  for (const int pattern : {0, 1, 2}) {
    cv::Mat allocation(74, 78, CV_8UC1, cv::Scalar(13));
    cv::Mat source = allocation(cv::Rect(3, 2, 68, 68));
    if (pattern == 0) {
      cv::RNG random(983);
      random.fill(source, cv::RNG::UNIFORM, 0, 256);
    } else if (pattern == 1) {
      source.setTo(255);
    } else {
      for (int row = 0; row < source.rows; ++row) {
        for (int column = 0; column < source.cols; ++column) {
          source.at<std::uint8_t>(row, column) = (row + column) % 2 ? 255 : 0;
        }
      }
    }
    ASSERT_FALSE(source.isContinuous());
    for (const int downsample : {1, 2}) {
      cv::Mat resized;
      cv::Mat gray = source;
      if (downsample != 1) {
        cv::resize(source, resized, cv::Size(source.cols / downsample, source.rows / downsample), 0,
                   0, cv::INTER_AREA);
        gray = resized;
      }
      for (const int ratio : {1, 2, 4}) {
        cv::Mat quarters, floats;
        if (ratio % 2 == 0) {
          cv::boxFilter(gray, quarters, CV_16UC1, cv::Size(2, 2), cv::Point(0, 0), false,
                        cv::BORDER_CONSTANT | cv::BORDER_ISOLATED);
          quarters.convertTo(floats, CV_32FC1, 0.25);
        } else {
          gray.convertTo(floats, CV_32FC1);
          gray.convertTo(quarters, CV_16UC1, 4.0);
        }
        for (const int window : {3, 9, 17, 31}) {
          SCOPED_TRACE(::testing::Message() << "pattern=" << pattern << " downsample=" << downsample
                                            << " ratio=" << ratio << " window=" << window);
          cv::Mat reference_sum, reference_square, integer_sum, integer_square;
          const cv::Size kernel(window, window);
          cv::boxFilter(floats, reference_sum, CV_64F, kernel, cv::Point(-1, -1), false,
                        cv::BORDER_CONSTANT);
          cv::sqrBoxFilter(floats, reference_square, CV_64F, kernel, cv::Point(-1, -1), false,
                           cv::BORDER_CONSTANT);
          cv::boxFilter(quarters, integer_sum, CV_64F, kernel, cv::Point(-1, -1), false,
                        cv::BORDER_CONSTANT);
          cv::sqrBoxFilter(quarters, integer_square, CV_64F, kernel, cv::Point(-1, -1), false,
                           cv::BORDER_CONSTANT);
          const double samples = window * window;
          for (int row = 0; row < gray.rows; ++row) {
            for (int column = 0; column < gray.cols; ++column) {
              const double expected_sum = reference_sum.at<double>(row, column);
              const double actual_sum = integer_sum.at<double>(row, column) * 0.25;
              const double expected_square = reference_square.at<double>(row, column);
              const double actual_square = integer_square.at<double>(row, column) * 0.0625;
              ASSERT_EQ(std::memcmp(&expected_sum, &actual_sum, sizeof(double)), 0);
              ASSERT_EQ(std::memcmp(&expected_square, &actual_square, sizeof(double)), 0);
              const double expected_variance =
                  expected_square - expected_sum * expected_sum / samples;
              const double actual_variance = actual_square - actual_sum * actual_sum / samples;
              ASSERT_EQ(std::memcmp(&expected_variance, &actual_variance, sizeof(double)), 0);
            }
          }
        }
      }
    }
  }
}

TEST(DepthRefinement, BothBaselineSignsPrincipalOffsetAndPixelCentreRatios)
{
  cv::Mat left_pixels(200, 200, CV_8UC1);
  cv::RNG random(338);
  random.fill(left_pixels, cv::RNG::UNIFORM, 0, 256);
  cv::GaussianBlur(left_pixels, left_pixels, cv::Size(5, 5), 1.2);
  for (const int sign : {-1, 1}) {
    cv::Mat right_pixels(200, 200, CV_8UC1, cv::Scalar(0));
    const int shift = 8;
    left_pixels(cv::Rect(sign > 0 ? shift : 0, 0, 200 - shift, 200))
        .copyTo(right_pixels(cv::Rect(sign > 0 ? 0 : shift, 0, 200 - shift, 200)));
    auto left = calibration();
    auto right = calibration(200, 200, 100, -(sign * shift - 1));
    right.k[2] -= 1;
    right.p[2] -= 1;
    CameraConfig camera;
    camera.calibration_mode = CalibrationMode::kCameraInfo;
    camera.input_geometry = InputGeometry::kAlreadyRectified;
    for (const int factor : {1, 2, 4}) {
      SCOPED_TRACE(::testing::Message() << "sign=" << sign << " factor=" << factor);
      CameraModel model(left, right, camera, factor);
      RefinementConfig config;
      config.enabled = true;
      config.downsample = factor == 1 ? 1 : 2;
      DepthRefiner refiner(config);
      ImagePair pair{frame(left_pixels, left), frame(right_pixels, right, "right_optical")};
      const auto original = pair.left.image.pixels().clone();
      const auto original_right = pair.right.image.pixels().clone();
      cv::Mat depths(model.height_px(), model.width_px(), CV_32FC1, cv::Scalar(1.1));
      depths.at<float>(depths.rows / 2, depths.cols / 2) = std::numeric_limits<float>::quiet_NaN();
      ProcessingStatistics statistics;
      refiner.refine(pair, model, factor, {}, depths, statistics);
      EXPECT_TRUE(std::isnan(depths.at<float>(depths.rows / 2, depths.cols / 2)));
      EXPECT_EQ(cv::norm(pair.left.image.pixels(), original, cv::NORM_INF), 0);
      EXPECT_EQ(cv::norm(pair.right.image.pixels(), original_right, cv::NORM_INF), 0);
      EXPECT_EQ(depths.at<float>(0, 0), 1.1F);
      EXPECT_GT(statistics.refinement_updated_pixels,
                static_cast<std::uint64_t>(depths.total() / 2));
      std::vector<float> central;
      for (int row = depths.rows / 4; row < 3 * depths.rows / 4; ++row) {
        for (int column = depths.cols / 4; column < 3 * depths.cols / 4; ++column) {
          if (std::isfinite(depths.at<float>(row, column))) {
            central.push_back(depths.at<float>(row, column));
          }
        }
      }
      std::sort(central.begin(), central.end());
      ASSERT_FALSE(central.empty());
      EXPECT_NEAR(central[central.size() / 2], 1.0, 0.01);
      EXPECT_EQ(statistics.refinement_attempted_pixels,
                statistics.refinement_updated_pixels + statistics.refinement_retained_pixels);
    }
  }
}

TEST(DepthRefinement, WeakTextureAndDisabledRefinementRetainCoarseDepth)
{
  const cv::Mat pixels(200, 200, CV_8UC1, cv::Scalar(128));
  const auto left = calibration();
  const auto right = calibration(200, 200, 100, -5);
  CameraConfig camera;
  camera.calibration_mode = CalibrationMode::kCameraInfo;
  CameraModel model(left, right, camera, 4);
  ImagePair pair{frame(pixels, left), frame(pixels, right, "right_optical")};
  for (const bool enabled : {false, true}) {
    RefinementConfig config;
    config.enabled = enabled;
    DepthRefiner refiner(config);
    cv::Mat depth(50, 50, CV_32FC1, cv::Scalar(1.25));
    depth.at<float>(25, 25) = std::numeric_limits<float>::quiet_NaN();
    ProcessingStatistics statistics;
    refiner.refine(pair, model, 4, {}, depth, statistics);
    EXPECT_EQ(depth.at<float>(24, 24), 1.25F);
    EXPECT_TRUE(std::isnan(depth.at<float>(25, 25)));
    EXPECT_EQ(statistics.refinement_updated_pixels, 0U);
    EXPECT_EQ(statistics.refinement_attempted_pixels, enabled ? 2499U : 0U);
  }
}

TEST(DepthRefinement, SearchBoundaryAndInvalidPublicInputsAreNotApplied)
{
  cv::Mat pixels(200, 200, CV_8UC1);
  cv::RNG random(451);
  random.fill(pixels, cv::RNG::UNIFORM, 0, 256);
  cv::GaussianBlur(pixels, pixels, cv::Size(5, 5), 1.2);
  cv::Mat right_pixels(200, 200, CV_8UC1, cv::Scalar(0));
  pixels(cv::Rect(8, 0, 192, 200)).copyTo(right_pixels(cv::Rect(0, 0, 192, 200)));
  const auto left = calibration();
  const auto right = calibration(200, 200, 100, -8);
  CameraConfig camera;
  camera.calibration_mode = CalibrationMode::kCameraInfo;
  ImagePair pair{frame(pixels, left), frame(right_pixels, right, "right_optical")};
  CameraModel model(left, right, camera, 4);
  RefinementConfig config;
  config.enabled = true;
  DepthRefiner refiner(config);
  cv::Mat depth(50, 50, CV_32FC1, cv::Scalar(2));
  ProcessingStatistics statistics;
  refiner.refine(pair, model, 4, {}, depth, statistics);
  // True fine disparity=4 is the edge of the predicted 2 +/- 2 search band.
  EXPECT_EQ(depth.at<float>(25, 25), 2.F);
  camera.input_geometry = InputGeometry::kRaw;
  CameraModel raw(left, right, camera, 4);
  EXPECT_THROW(refiner.refine(pair, raw, 4, {}, depth, statistics), std::invalid_argument);
  DepthConfig invalid;
  invalid.max_depth_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(refiner.refine(pair, model, 4, invalid, depth, statistics), std::invalid_argument);
  cv::Mat alias(50, 50, CV_32FC1, pair.left.image.pixels().data);
  EXPECT_THROW(refiner.refine(pair, model, 4, {}, alias, statistics), std::invalid_argument);
}

TEST(DepthRefinement, ParallelRowsPreserveValuesAndExactCounts)
{
  cv::Mat left_pixels(200, 200, CV_8UC1);
  cv::RNG random(941);
  random.fill(left_pixels, cv::RNG::UNIFORM, 0, 256);
  cv::GaussianBlur(left_pixels, left_pixels, cv::Size(5, 5), 1.2);
  cv::Mat right_pixels(200, 200, CV_8UC1, cv::Scalar(0));
  left_pixels(cv::Rect(0, 0, 192, 200)).copyTo(right_pixels(cv::Rect(8, 0, 192, 200)));
  const auto left = calibration();
  const auto right = calibration();
  CameraConfig camera;
  CameraModel model(left, right, camera, 4);
  ImagePair pair{frame(left_pixels, left), frame(right_pixels, right, "right_optical")};
  RefinementConfig config;
  config.enabled = true;
  DepthRefiner refiner(config);
  const int previous_threads = cv::getNumThreads();
  cv::Mat reference;
  ProcessingStatistics reference_counts;
  for (const int threads : {0, 2, 4}) {
    cv::setNumThreads(threads);
    cv::Mat depth(50, 50, CV_32FC1, cv::Scalar(.75));
    depth.at<float>(25, 25) = std::numeric_limits<float>::quiet_NaN();
    ProcessingStatistics counts;
    refiner.refine(pair, model, 4, {}, depth, counts);
    if (reference.empty()) {
      reference = depth.clone();
      reference_counts = counts;
    } else {
      EXPECT_EQ(counts.refinement_attempted_pixels, reference_counts.refinement_attempted_pixels);
      EXPECT_EQ(counts.refinement_updated_pixels, reference_counts.refinement_updated_pixels);
      EXPECT_EQ(counts.refinement_retained_pixels, reference_counts.refinement_retained_pixels);
      for (int row = 0; row < depth.rows; ++row) {
        for (int column = 0; column < depth.cols; ++column) {
          const float expected = reference.at<float>(row, column);
          const float actual = depth.at<float>(row, column);
          if (std::isnan(expected)) {
            EXPECT_TRUE(std::isnan(actual));
          } else {
            EXPECT_EQ(actual, expected);
          }
        }
      }
    }
  }
  cv::setNumThreads(previous_threads);
}

TEST(DepthRefinement, MaximumWindowAndSaturatedQuarterSamplesDoNotOverflow)
{
  cv::Mat left_pixels(200, 200, CV_8UC1, cv::Scalar(255));
  cv::RNG random(740);
  for (int row = 0; row < left_pixels.rows; row += 4) {
    for (int column = 0; column < left_pixels.cols; column += 4) {
      if (random.uniform(0, 16) == 0) {
        left_pixels(cv::Rect(column, row, 4, 4)).setTo(0);
      }
    }
  }
  cv::Mat right_pixels(200, 200, CV_8UC1, cv::Scalar(255));
  left_pixels(cv::Rect(0, 0, 192, 200)).copyTo(right_pixels(cv::Rect(8, 0, 192, 200)));
  const auto left = calibration();
  const auto right = calibration();
  CameraConfig camera;
  CameraModel model(left, right, camera, 4);
  ImagePair pair{frame(left_pixels, left), frame(right_pixels, right, "right_optical")};
  RefinementConfig config;
  config.enabled = true;
  config.window_size_px = 31;
  DepthRefiner refiner(config);
  cv::Mat depth(50, 50, CV_32FC1, cv::Scalar(.75));
  ProcessingStatistics counts;
  refiner.refine(pair, model, 4, {}, depth, counts);
  EXPECT_GT(counts.refinement_updated_pixels, 500U);
  EXPECT_NEAR(depth.at<float>(25, 25), .625, .02);
  EXPECT_EQ(depth.at<float>(0, 0), .75F);
}

TEST(DepthRefinement, InvalidSettingsAndRawCoordinatesAreRejected)
{
  StereoConfig config;
  config.refinement.enabled = true;
  config.refinement.window_size_px = 4;
  EXPECT_FALSE(validate_config(config).empty());
  config.refinement.window_size_px = 9;
  config.camera.calibration_mode = CalibrationMode::kCameraInfo;
  config.camera.input_geometry = InputGeometry::kRaw;
  EXPECT_FALSE(validate_config(config).empty());
  config.camera.input_geometry = InputGeometry::kAlreadyRectified;
  config.processing.downsample = 1;
  EXPECT_FALSE(validate_config(config).empty());
  config.processing.downsample = 4;
  config.refinement.min_texture_variance = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(validate_config(config).empty());
}

TEST(DepthRefinement, OverlappingProductsMatchIsolatedWindowsAcrossFramesAndGaps)
{
  for (const int sign : {-1, 1}) {
    for (const int window : {9, 31}) {
      SCOPED_TRACE(::testing::Message() << "sign=" << sign << " window=" << window);
      const auto left = calibration();
      const auto right = calibration(200, 200, 100, -sign * 8);
      CameraConfig camera;
      camera.calibration_mode = CalibrationMode::kCameraInfo;
      CameraModel model(left, right, camera, 4);
      RefinementConfig config;
      config.enabled = true;
      config.window_size_px = window;
      DepthRefiner refiner(config);
      std::uint64_t all_updates = 0;
      for (const int seed : {184, 951}) {
        auto owner = std::make_shared<cv::Mat>(202, 204, CV_8UC1);
        cv::Mat pixels = (*owner)(cv::Rect(1, 1, 200, 200));
        cv::RNG random(seed);
        random.fill(pixels, cv::RNG::UNIFORM, 0, 256);
        cv::GaussianBlur(pixels, pixels, cv::Size(5, 5), 1.2);
        cv::Mat right_pixels(200, 200, CV_8UC1, cv::Scalar(0));
        pixels(cv::Rect(sign > 0 ? 8 : 0, 0, 192, 200))
            .copyTo(right_pixels(cv::Rect(sign > 0 ? 0 : 8, 0, 192, 200)));
        right_pixels(cv::Rect(84, 0, 32, 200)).setTo(128);
        ImagePair pair{{ImageBuffer::borrow(pixels, owner), left, {seed}, "left_optical"},
                       frame(right_pixels, right, "right_optical")};
        ASSERT_FALSE(pair.left.image.pixels().isContinuous());
        cv::Mat allocation(52, 52, CV_32FC1, cv::Scalar(-3));
        cv::Mat dense = allocation(cv::Rect(1, 1, 50, 50));
        dense.setTo(std::numeric_limits<float>::quiet_NaN());
        constexpr int kRow = 25;
        for (int column = 0; column < dense.cols; ++column) {
          if (column < 15 || column > 20) {
            dense.at<float>(kRow, column) = column % 3 == 0 ? 1.1F : .9F;
          }
        }
        const auto coarse = dense.clone();
        ProcessingStatistics dense_counts;
        refiner.refine(pair, model, 4, {}, dense, dense_counts);
        std::uint64_t isolated_updates = 0;
        for (int column = 0; column < dense.cols; ++column) {
          if (!std::isfinite(coarse.at<float>(kRow, column))) {
            EXPECT_TRUE(std::isnan(dense.at<float>(kRow, column)));
            continue;
          }
          cv::Mat isolated(50, 50, CV_32FC1, cv::Scalar(std::numeric_limits<float>::quiet_NaN()));
          isolated.at<float>(kRow, column) = coarse.at<float>(kRow, column);
          ProcessingStatistics isolated_counts;
          refiner.refine(pair, model, 4, {}, isolated, isolated_counts);
          EXPECT_EQ(dense.at<float>(kRow, column), isolated.at<float>(kRow, column));
          isolated_updates += isolated_counts.refinement_updated_pixels;
        }
        EXPECT_EQ(dense_counts.refinement_updated_pixels, isolated_updates);
        EXPECT_EQ(dense_counts.refinement_attempted_pixels, 44U);
        EXPECT_EQ(allocation.at<float>(0, 0), -3.F);
        all_updates += isolated_updates;
      }
      EXPECT_GT(all_updates, 0U);
    }
  }
}

TEST(DepthRefinement, BorrowedSourceRoiMatchesClonedLogicalImageAtEveryRatio)
{
  for (const int sign : {-1, 1}) {
    auto left_owner = std::make_shared<cv::Mat>(206, 208, CV_8UC1, cv::Scalar(255));
    auto right_owner = std::make_shared<cv::Mat>(206, 208, CV_8UC1, cv::Scalar(13));
    cv::Mat pixels = (*left_owner)(cv::Rect(3, 2, 200, 200));
    cv::Mat right_pixels = (*right_owner)(cv::Rect(3, 2, 200, 200));
    cv::RNG random(914);
    random.fill(pixels, cv::RNG::UNIFORM, 0, 256);
    cv::GaussianBlur(pixels, pixels, cv::Size(5, 5), 1.2);
    right_pixels.setTo(0);
    pixels(cv::Rect(sign > 0 ? 8 : 0, 0, 192, 200))
        .copyTo(right_pixels(cv::Rect(sign > 0 ? 0 : 8, 0, 192, 200)));
    const auto left = calibration();
    const auto right = calibration(200, 200, 100, -sign * 8);
    const auto original_left = left_owner->clone();
    const auto original_right = right_owner->clone();
    ImagePair borrowed{
        {ImageBuffer::borrow(pixels, left_owner), left, {123}, "left_optical"},
        {ImageBuffer::borrow(right_pixels, right_owner), right, {123}, "right_optical"}};
    ImagePair cloned{frame(pixels.clone(), left),
                     frame(right_pixels.clone(), right, "right_optical")};
    CameraConfig camera;
    camera.calibration_mode = CalibrationMode::kCameraInfo;
    RefinementConfig config;
    config.enabled = true;
    config.downsample = 1;
    for (const int factor : {1, 2, 4}) {
      CameraModel model(left, right, camera, factor);
      DepthRefiner roi_refiner(config), clone_refiner(config);
      cv::Mat first(model.height_px(), model.width_px(), CV_32FC1, cv::Scalar(1.1));
      first.at<float>(first.rows / 2, first.cols / 2) = std::numeric_limits<float>::quiet_NaN();
      cv::Mat second = first.clone();
      ProcessingStatistics first_counts, second_counts;
      roi_refiner.refine(borrowed, model, factor, {}, first, first_counts);
      clone_refiner.refine(cloned, model, factor, {}, second, second_counts);
      EXPECT_EQ(std::memcmp(first.data, second.data, first.total() * first.elemSize()), 0);
      EXPECT_EQ(first_counts.refinement_updated_pixels, second_counts.refinement_updated_pixels);
      EXPECT_GT(first_counts.refinement_updated_pixels, 0U);
    }
    EXPECT_EQ(cv::norm(*left_owner, original_left, cv::NORM_INF), 0);
    EXPECT_EQ(cv::norm(*right_owner, original_right, cv::NORM_INF), 0);
  }
}

TEST(DepthRefinement, SharedGrayMatchesStandaloneForEveryEncoding)
{
  cv::Mat mono(200, 200, CV_8UC1);
  cv::RNG random(732);
  random.fill(mono, cv::RNG::UNIFORM, 0, 256);
  for (const auto encoding : {ImageEncoding::kMono8, ImageEncoding::kRgb8, ImageEncoding::kBgr8}) {
    cv::Mat pixels;
    if (encoding == ImageEncoding::kMono8) {
      pixels = mono;
    } else {
      std::vector<cv::Mat> channels{mono, 255 - mono, mono / 2};
      cv::merge(channels, pixels);
    }
    const auto left = calibration();
    const auto right = calibration(200, 200, 100, -5);
    CameraConfig camera;
    camera.calibration_mode = CalibrationMode::kCameraInfo;
    CameraModel model(left, right, camera, 4);
    cv::Mat right_pixels = cv::Mat::zeros(pixels.size(), pixels.type());
    pixels(cv::Rect(5, 0, 195, 200)).copyTo(right_pixels(cv::Rect(0, 0, 195, 200)));
    ImagePair pair{frame(pixels, left), frame(right_pixels, right, "right_optical")};
    pair.left.encoding = encoding;
    pair.right.encoding = encoding;
    ImagePreprocessor preprocessor(model, 5);
    const auto &prepared = preprocessor.prepare(pair);
    cv::Mat expected;
    if (encoding == ImageEncoding::kMono8) {
      expected = mono;
      EXPECT_EQ(prepared.left_source_gray.pixels.data, pair.left.image.pixels().data);
      ASSERT_TRUE(prepared.left_source_gray.source_owner);
    } else {
      cv::cvtColor(pixels, expected,
                   encoding == ImageEncoding::kRgb8 ? cv::COLOR_RGB2GRAY : cv::COLOR_BGR2GRAY);
      EXPECT_FALSE(prepared.left_source_gray.source_owner);
    }
    EXPECT_EQ(cv::norm(expected, prepared.left_source_gray.pixels, cv::NORM_INF), 0);
    cv::Mat expected_matching;
    cv::resize(expected, expected_matching, cv::Size(50, 50), 0, 0, cv::INTER_AREA);
    EXPECT_EQ(cv::norm(expected_matching, prepared.left_gray, cv::NORM_INF), 0);
    RefinementConfig config;
    config.enabled = true;
    DepthRefiner standalone(config), shared(config);
    cv::Mat first(50, 50, CV_32FC1, cv::Scalar(1.1));
    first.at<float>(10, 10) = std::numeric_limits<float>::quiet_NaN();
    cv::Mat second = first.clone();
    ProcessingStatistics first_stats, second_stats;
    standalone.refine(pair, model, 4, {}, first, first_stats);
    shared.refine(pair, prepared.left_source_gray, prepared.right_source_gray, model, 4, {}, second,
                  second_stats);
    EXPECT_EQ(std::memcmp(first.data, second.data, first.total() * first.elemSize()), 0);
    EXPECT_GT(first_stats.refinement_updated_pixels, 0U);
    EXPECT_EQ(first_stats.refinement_attempted_pixels, second_stats.refinement_attempted_pixels);
    EXPECT_EQ(first_stats.refinement_updated_pixels, second_stats.refinement_updated_pixels);
    EXPECT_EQ(first_stats.refinement_retained_pixels, second_stats.refinement_retained_pixels);
    EXPECT_EQ(cv::norm(pair.left.image.pixels(), pixels, cv::NORM_INF), 0);
  }
}

TEST(DepthRefinement, SourceGrayKeepsBorrowedMonoOwnerAfterPairRelease)
{
  const auto left = calibration();
  const auto right = calibration(200, 200, 100, -5);
  CameraConfig camera;
  camera.calibration_mode = CalibrationMode::kCameraInfo;
  ImagePreprocessor preprocessor(CameraModel(left, right, camera, 4), 5);
  auto owner = std::make_shared<std::vector<unsigned char>>(40000, 73);
  std::weak_ptr<std::vector<unsigned char>> weak = owner;
  const PreparedImages *retained = nullptr;
  {
    cv::Mat pixels(200, 200, CV_8UC1, owner->data());
    ImagePair pair{{ImageBuffer::borrow(pixels, owner), left, {123}, "left_optical"},
                   {ImageBuffer::borrow(pixels, owner), right, {123}, "right_optical"}};
    const auto &prepared = preprocessor.prepare(pair);
    retained = &prepared;
    EXPECT_EQ(prepared.left_source_gray.pixels.data, owner->data());
    owner.reset();
  }
  EXPECT_FALSE(weak.expired());
  ASSERT_NE(retained, nullptr);
  EXPECT_EQ(retained->left_source_gray.pixels.at<unsigned char>(0, 0), 73);
  // The external message is gone; the retained view is still readable.
  ImagePair replacement{frame(cv::Mat(200, 200, CV_8UC1, cv::Scalar(44)), left),
                        frame(cv::Mat(200, 200, CV_8UC1, cv::Scalar(44)), right, "right_optical")};
  const auto &old = preprocessor.prepare(replacement);
  EXPECT_TRUE(weak.expired());
  EXPECT_EQ(old.left_source_gray.pixels.at<unsigned char>(0, 0), 44);
}

TEST(DepthRefinement, ProvidedGrayRejectsStaleShapeAndDepthAliasButDisabledDoesNoWork)
{
  const auto left = calibration();
  const auto right = calibration(200, 200, 100, -5);
  CameraConfig camera;
  camera.calibration_mode = CalibrationMode::kCameraInfo;
  CameraModel model(left, right, camera, 4);
  cv::Mat pixels(200, 200, CV_8UC1, cv::Scalar(93));
  ImagePair pair{frame(pixels, left), frame(pixels, right, "right_optical")};
  ImagePreprocessor preprocessor(model, 5);
  const auto &prepared = preprocessor.prepare(pair);
  RefinementConfig config;
  config.enabled = true;
  DepthRefiner refiner(config);
  cv::Mat depth(50, 50, CV_32FC1, cv::Scalar(1));
  ProcessingStatistics statistics;
  auto invalid = prepared.left_source_gray;
  ++invalid.stamp.nanoseconds;
  EXPECT_THROW(
      refiner.refine(pair, invalid, prepared.right_source_gray, model, 4, {}, depth, statistics),
      std::invalid_argument);
  invalid = prepared.left_source_gray;
  invalid.source_data = pair.right.image.pixels().data;
  EXPECT_THROW(
      refiner.refine(pair, invalid, prepared.right_source_gray, model, 4, {}, depth, statistics),
      std::invalid_argument);
  invalid = prepared.left_source_gray;
  invalid.pixels = cv::Mat(100, 100, CV_8UC1);
  EXPECT_THROW(
      refiner.refine(pair, invalid, prepared.right_source_gray, model, 4, {}, depth, statistics),
      std::invalid_argument);
  cv::Mat allocation(200, 200, CV_32FC1, cv::Scalar(1));
  depth = allocation(cv::Rect(0, 0, 50, 50));
  invalid = prepared.left_source_gray;
  invalid.pixels = cv::Mat(200, 200, CV_8UC1, allocation.data);
  EXPECT_THROW(
      refiner.refine(pair, invalid, prepared.right_source_gray, model, 4, {}, depth, statistics),
      std::invalid_argument);
  config.enabled = false;
  DepthRefiner disabled(config);
  EXPECT_NO_THROW(
      disabled.refine(pair, SourceGrayImage{}, SourceGrayImage{}, model, 4, {}, depth, statistics));
  EXPECT_EQ(statistics.refinement_attempted_pixels, 0U);
  EXPECT_EQ(depth.at<float>(0, 0), 1);
}

} // namespace
