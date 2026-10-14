#include "intball2_stereo/core/stereo_processor.hpp"

#include <chrono>
#include <cmath>
#include <limits>

#include <opencv2/calib3d.hpp>
#include <stdexcept>
#include <utility>

namespace intball2_stereo::core
{
StereoProcessor::StereoProcessor(StereoConfig config)
    : config_(config), matcher_(config_.matcher), refiner_(config_.refinement)
{
  require_valid_config(config_);
}

const DepthResult &StereoProcessor::process(const ImagePair &pair, const StageObserver &observer)
{
  const auto errors = validate_config_for_input(config_, pair.left.image.pixels().cols,
                                                pair.left.image.pixels().rows);
  if (!errors.empty()) {
    throw std::invalid_argument(errors.front());
  }
  if (pair.left.optical_frame_id.empty() || pair.right.optical_frame_id.empty()) {
    throw std::invalid_argument("image optical frame identifiers must not be empty");
  }
  if (observer) {
    observer(ProcessingStage::kPreprocessingStarted);
  }
  const auto start = std::chrono::steady_clock::now();
  if (!preprocessor_ ||
      !preprocessor_->camera_model().matches_input(pair.left.calibration, pair.right.calibration)) {
    CameraModel model(pair.left.calibration, pair.right.calibration, config_.camera,
                      config_.processing.downsample);
    if (config_.camera.input_geometry == InputGeometry::kRaw) {
      const std::array<double, 9> identity{1, 0, 0, 0, 1, 0, 0, 0, 1};
      for (std::size_t i = 0; i < identity.size(); ++i) {
        if (std::abs(pair.left.calibration.r[i] - identity[i]) > 1e-8) {
          throw std::invalid_argument(
              "raw left R changes optical orientation; provide a rectified input/frame first");
        }
      }
    }
    model.require_search_coverage(config_.matcher, config_.depth);
    const auto &processing = config_.processing;
    matching_roi_ = {processing.roi_left_px, processing.roi_top_px,
                     model.width_px() - processing.roi_left_px - processing.roi_right_px,
                     model.height_px() - processing.roi_top_px - processing.roi_bottom_px};
    if (processing.roi_left_px || processing.roi_right_px || processing.roi_top_px ||
        processing.roi_bottom_px) {
      matching_model_ = std::make_unique<CameraModel>(model.cropped(matching_roi_));
    }
    preprocessor_ =
        std::make_unique<ImagePreprocessor>(std::move(model), config_.matcher.block_size_px);
  }
  const auto &prepared = preprocessor_->prepare(pair);
  const auto preprocessed = std::chrono::steady_clock::now();
  if (observer) {
    observer(ProcessingStage::kPreprocessingFinished);
  }
  const auto &disparity =
      matcher_.compute(matching_model_ ? prepared.left_gray(matching_roi_) : prepared.left_gray,
                       matching_model_ ? prepared.right_gray(matching_roi_) : prepared.right_gray);
  const auto matched = std::chrono::steady_clock::now();
  if (observer) {
    observer(ProcessingStage::kMatchingFinished);
  }
  const auto &model = preprocessor_->camera_model();
  output_.statistics = {};
  const auto &matching_model = matching_model_ ? *matching_model_ : model;
  const auto fraction = DepthConverter::convert(
      disparity, matching_model, config_.matcher, config_.depth,
      matching_model_ ? prepared.left_valid(matching_roi_) : prepared.left_valid,
      matching_model_ ? prepared.right_valid(matching_roi_) : prepared.right_valid,
      matching_model_ ? matching_depth_m_ : output_.depth_m);
  if (matching_model_) {
    // Restore the original pixel rays; the refiner and published calibration remain full-grid.
    output_.depth_m.create(model.height_px(), model.width_px(), CV_32FC1);
    output_.depth_m.setTo(std::numeric_limits<float>::quiet_NaN());
    matching_depth_m_.copyTo(output_.depth_m(matching_roi_));
  }
  const cv::Rect full(0, 0, model.width_px(), model.height_px());
  const cv::Rect matching_full(0, 0, matching_model.width_px(), matching_model.height_px());
  const auto eligible = [&](const cv::Rect &image) {
    return cv::getValidDisparityROI(image, image, config_.matcher.min_disparity_px,
                                    config_.matcher.num_disparities_px,
                                    config_.matcher.block_size_px);
  };
  const auto full_eligible = eligible(full);
  const auto roi_eligible = eligible(matching_full);
  const auto pixel_count = [](const cv::Rect &image) {
    return static_cast<std::uint64_t>(image.width) * static_cast<std::uint64_t>(image.height);
  };
  output_.statistics.matching_input_pixels = pixel_count(matching_full);
  output_.statistics.full_disparity_eligible_pixels = pixel_count(full_eligible);
  output_.statistics.roi_disparity_eligible_pixels = pixel_count(roi_eligible);
  output_.statistics.roi_additional_search_excluded_pixels =
      pixel_count(full_eligible & matching_roi_) - pixel_count(roi_eligible);
  output_.statistics.valid_depth_fraction =
      matching_model_ ? fraction * static_cast<double>(pixel_count(matching_full)) /
                            static_cast<double>(pixel_count(full))
                      : fraction;
  const auto converted = std::chrono::steady_clock::now();
  if (observer) {
    observer(ProcessingStage::kDepthConversionFinished);
  }
  refiner_.refine(pair, prepared.left_source_gray, prepared.right_source_gray, model,
                  config_.processing.downsample, config_.depth, output_.depth_m,
                  output_.statistics);
  const auto refined = std::chrono::steady_clock::now();
  if (observer) {
    observer(ProcessingStage::kRefinementFinished);
  }
  output_.statistics.preprocessing_s = std::chrono::duration<double>(preprocessed - start).count();
  output_.statistics.matching_s = std::chrono::duration<double>(matched - preprocessed).count();
  output_.statistics.depth_conversion_s =
      std::chrono::duration<double>(converted - matched).count();
  output_.statistics.quality_filter_s = std::chrono::duration<double>(refined - converted).count();
  // Published depth describes a rectified left camera. GNC reads K rather than P.
  output_.calibration = model.left();
  output_.calibration.k = {
      model.left().p[0], 0, model.left().p[2], 0, model.left().p[5], model.left().p[6], 0, 0, 1};
  output_.calibration.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  output_.calibration.d.clear();
  output_.calibration.distortion_model = "plumb_bob";
  output_.stamp = pair.left.stamp;
  output_.optical_frame_id = pair.left.optical_frame_id;
  if (observer) {
    observer(ProcessingStage::kOutputPrepared);
  }
  return output_;
}
} // namespace intball2_stereo::core
