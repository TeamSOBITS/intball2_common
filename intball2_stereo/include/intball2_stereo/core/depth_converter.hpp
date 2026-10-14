#ifndef INTBALL2_STEREO_CORE_DEPTH_CONVERTER_HPP_
#define INTBALL2_STEREO_CORE_DEPTH_CONVERTER_HPP_

#include "intball2_stereo/core/camera_model.hpp"

namespace intball2_stereo::core
{
class DepthConverter
{
public:
  // Writes/reuses CV_32FC1 output, invalid=NaN. Returns valid pixel fraction.
  // Output must not share storage with disparity or validity masks.
  static double convert(const cv::Mat &disparity_fixed, const CameraModel &model,
                        const MatcherConfig &matcher, const DepthConfig &depth,
                        const cv::Mat &left_valid, const cv::Mat &right_valid, cv::Mat &depth_m);
};
} // namespace intball2_stereo::core
#endif // INTBALL2_STEREO_CORE_DEPTH_CONVERTER_HPP_
