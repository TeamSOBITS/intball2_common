#ifndef INTBALL2_STEREO_CORE_STEREO_PROCESSOR_HPP_
#define INTBALL2_STEREO_CORE_STEREO_PROCESSOR_HPP_

#include <functional>
#include <memory>

#include "intball2_stereo/core/depth_converter.hpp"
#include "intball2_stereo/core/depth_refiner.hpp"
#include "intball2_stereo/core/image_preprocessor.hpp"
#include "intball2_stereo/core/sgbm_matcher.hpp"

namespace intball2_stereo::core
{
enum class ProcessingStage
{
  kPreprocessingStarted,
  kPreprocessingFinished,
  kMatchingFinished,
  kDepthConversionFinished,
  kRefinementFinished,
  kOutputPrepared
};

class StereoProcessor
{
public:
  explicit StereoProcessor(StereoConfig config);
  StereoProcessor(const StereoProcessor &) = delete;
  StereoProcessor &operator=(const StereoProcessor &) = delete;
  // Single caller only. Output is borrowed until the next process() or destruction.
  // Retaining callers must clone depth_m. Reconstruct to change configuration.
  using StageObserver = std::function<void(ProcessingStage)>;
  // Observer records stage boundaries only; it must not reenter this processor.
  const DepthResult &process(const ImagePair &pair, const StageObserver &observer = {});

private:
  StereoConfig config_;
  SgbmMatcher matcher_;
  DepthRefiner refiner_;
  std::unique_ptr<ImagePreprocessor> preprocessor_;
  std::unique_ptr<CameraModel> matching_model_;
  cv::Rect matching_roi_;
  cv::Mat matching_depth_m_;
  DepthResult output_;
};
} // namespace intball2_stereo::core
#endif // INTBALL2_STEREO_CORE_STEREO_PROCESSOR_HPP_
