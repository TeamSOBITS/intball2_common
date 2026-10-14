#ifndef INTBALL2_STEREO_CORE_SGBM_MATCHER_HPP_
#define INTBALL2_STEREO_CORE_SGBM_MATCHER_HPP_

#include <opencv2/calib3d.hpp>

#include "intball2_stereo/core/stereo_config.hpp"

namespace intball2_stereo::core
{
class SgbmMatcher
{
public:
  explicit SgbmMatcher(const MatcherConfig &config);
  SgbmMatcher(const SgbmMatcher &) = delete;
  SgbmMatcher &operator=(const SgbmMatcher &) = delete;
  // CV_16SC1, disparity*16. Invalid=(min_disparity_px-1)*16.
  // Single caller; storage is reused by the next compute().
  const cv::Mat &compute(const cv::Mat &left_gray, const cv::Mat &right_gray);

private:
  MatcherConfig config_;
  cv::Ptr<cv::StereoSGBM> matcher_;
  cv::Mat disparity_;
};
} // namespace intball2_stereo::core
#endif // INTBALL2_STEREO_CORE_SGBM_MATCHER_HPP_
