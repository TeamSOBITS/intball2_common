#ifndef INTBALL2_STEREO_CORE_DEPTH_REFINER_HPP_
#define INTBALL2_STEREO_CORE_DEPTH_REFINER_HPP_

#include <vector>

#include "intball2_stereo/core/camera_model.hpp"

namespace intball2_stereo::core
{
class DepthRefiner
{
public:
  explicit DepthRefiner(RefinementConfig config);
  // Refine existing finite depth only. Rejected estimates retain coarse values.
  void refine(const ImagePair &pair, const CameraModel &model, int matching_downsample,
              const DepthConfig &depth_config, cv::Mat &depth_m, ProcessingStatistics &statistics);

  // Provided gray must describe this exact source frame, without aliasing mutable depth.
  void refine(const ImagePair &pair, const SourceGrayImage &left_gray,
              const SourceGrayImage &right_gray, const CameraModel &model, int matching_downsample,
              const DepthConfig &depth_config, cv::Mat &depth_m, ProcessingStatistics &statistics);

private:
  struct Buffers
  {
    cv::Mat colour_gray, resized, scaled_quarters, window_sum, window_square_sum;
  };
  struct RowCounts
  {
    std::uint64_t attempted = 0;
    std::uint64_t updated = 0;
  };
  void prepare(const cv::Mat &source_gray, int ratio, Buffers &buffers);
  void validate_inputs(const ImagePair &pair, const CameraModel &model, int matching_downsample,
                       const DepthConfig &depth_config, const cv::Mat &depth_m) const;
  void refine_gray(const cv::Mat &left_gray, const cv::Mat &right_gray, const CameraModel &model,
                   int matching_downsample, const DepthConfig &depth_config, cv::Mat &depth_m,
                   ProcessingStatistics &statistics);
  RefinementConfig config_;
  Buffers left_, right_;
  std::vector<RowCounts> row_counts_;
};
} // namespace intball2_stereo::core
#endif // INTBALL2_STEREO_CORE_DEPTH_REFINER_HPP_
