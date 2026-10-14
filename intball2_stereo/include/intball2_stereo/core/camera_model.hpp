#ifndef INTBALL2_STEREO_CORE_CAMERA_MODEL_HPP_
#define INTBALL2_STEREO_CORE_CAMERA_MODEL_HPP_

#include "intball2_stereo/core/stereo_config.hpp"

namespace intball2_stereo::core
{
bool same_calibration(const CameraCalibration &left, const CameraCalibration &right);

// Horizontal stereo, expressed in the left rectified optical frame.
class CameraModel
{
public:
  CameraModel(const CameraCalibration &left, const CameraCalibration &right,
              const CameraConfig &config, int downsample);
  // Shift both processed calibrations by the same pixel origin; never reapply baseline overrides.
  CameraModel cropped(const cv::Rect &roi) const;
  bool matches_input(const CameraCalibration &left, const CameraCalibration &right) const;
  void require_search_coverage(const MatcherConfig &matcher, const DepthConfig &depth) const;
  const CameraCalibration &left() const;
  const CameraCalibration &right() const;
  InputGeometry input_geometry() const;
  int width_px() const;
  int height_px() const;
  double numerator_px_m() const;
  double principal_difference_px() const;

private:
  CameraCalibration source_left_;
  CameraCalibration source_right_;
  CameraCalibration left_;
  CameraCalibration right_;
  InputGeometry input_geometry_;
};
} // namespace intball2_stereo::core
#endif // INTBALL2_STEREO_CORE_CAMERA_MODEL_HPP_
