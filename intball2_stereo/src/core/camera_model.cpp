#include "intball2_stereo/core/camera_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace intball2_stereo::core
{
namespace
{
bool near(double left, double right)
{
  return std::abs(left - right) <= 1e-8 * std::max({1.0, std::abs(left), std::abs(right)});
}

void require(bool condition, const char *reason)
{
  if (!condition) {
    throw std::invalid_argument(reason);
  }
}

template <typename Container> bool all_finite(const Container &values)
{
  return std::all_of(values.begin(), values.end(), [](double v) { return std::isfinite(v); });
}

void validate_calibration(const CameraCalibration &c)
{
  require(c.width > 0 && c.height > 0 &&
              c.width <= static_cast<std::uint32_t>(std::numeric_limits<int>::max()) &&
              c.height <= static_cast<std::uint32_t>(std::numeric_limits<int>::max()),
          "calibration dimensions must fit positive int");
  require(all_finite(c.k) && all_finite(c.p) && all_finite(c.r) && all_finite(c.d),
          "calibration matrices must be finite");
  require(c.k[0] > 0 && c.k[4] > 0 && c.p[0] > 0 && c.p[5] > 0,
          "calibration focal lengths must be positive");
  require(near(c.k[1], 0) && near(c.k[3], 0) && near(c.k[6], 0) && near(c.k[7], 0) &&
              near(c.k[8], 1),
          "only canonical zero-skew pinhole K is supported");
  require(near(c.p[1], 0) && near(c.p[4], 0) && near(c.p[7], 0) && near(c.p[8], 0) &&
              near(c.p[9], 0) && near(c.p[10], 1) && near(c.p[11], 0),
          "only canonical horizontal stereo P is supported");
  require(c.binning_x <= 1 && c.binning_y <= 1,
          "binned CameraInfo must be normalized before entering core");
  const bool empty_roi = c.roi.width == 0 && c.roi.height == 0;
  const bool full_roi = c.roi.width == c.width && c.roi.height == c.height;
  require(c.roi.x_offset == 0 && c.roi.y_offset == 0 && (empty_roi || full_roi),
          "cropped CameraInfo ROI must be normalized before entering core");
  require(c.distortion_model.empty() || c.distortion_model == "plumb_bob" ||
              c.distortion_model == "rational_polynomial",
          "unsupported distortion model (only pinhole models supported)");
  require(c.d.empty() || c.d.size() == 4 || c.d.size() == 5 || c.d.size() == 8 ||
              c.d.size() == 12 || c.d.size() == 14,
          "unsupported pinhole distortion coefficient count");
  require(!c.distortion_model.empty() ||
              std::all_of(c.d.begin(), c.d.end(), [](double v) { return v == 0; }),
          "nonzero distortion requires a named model");
  const cv::Mat rotation(3, 3, CV_64FC1, const_cast<double *>(c.r.data()));
  require(cv::norm(rotation * rotation.t() - cv::Mat::eye(3, 3, CV_64FC1), cv::NORM_INF) < 1e-6 &&
              std::abs(cv::determinant(rotation) - 1.0) < 1e-6,
          "rectification R must be a proper rotation");
}

CameraCalibration scaled(CameraCalibration c, int downsample)
{
  const double scale = 1.0 / downsample;
  // OpenCV resize maps pixel centres: x_out = (x_in + .5)/factor - .5.
  const double offset = (scale - 1.0) * 0.5;
  for (int column = 0; column < 3; ++column) {
    c.k[column] = scale * c.k[column] + offset * c.k[6 + column];
    c.k[3 + column] = scale * c.k[3 + column] + offset * c.k[6 + column];
  }
  for (int column = 0; column < 4; ++column) {
    c.p[column] = scale * c.p[column] + offset * c.p[8 + column];
    c.p[4 + column] = scale * c.p[4 + column] + offset * c.p[8 + column];
  }
  c.width /= downsample;
  c.height /= downsample;
  c.roi = {};
  c.binning_x = 0;
  c.binning_y = 0;
  return c;
}
} // namespace

bool same_calibration(const CameraCalibration &a, const CameraCalibration &b)
{
  return a.width == b.width && a.height == b.height && a.distortion_model == b.distortion_model &&
         a.k == b.k && a.d == b.d && a.r == b.r && a.p == b.p && a.roi.x_offset == b.roi.x_offset &&
         a.roi.y_offset == b.roi.y_offset && a.roi.width == b.roi.width &&
         a.roi.height == b.roi.height && a.roi.do_rectify == b.roi.do_rectify &&
         a.binning_x == b.binning_x && a.binning_y == b.binning_y;
}

CameraModel::CameraModel(const CameraCalibration &left, const CameraCalibration &right,
                         const CameraConfig &config, int downsample)
    : source_left_(left), source_right_(right), input_geometry_(config.input_geometry)
{
  StereoConfig check;
  check.camera = config;
  check.processing.downsample = downsample;
  require_valid_config(check);
  validate_calibration(left);
  validate_calibration(right);
  require(left.width == right.width && left.height == right.height,
          "left/right calibration dimensions differ");
  require(left.width % downsample == 0 && left.height % downsample == 0,
          "calibration dimensions must be divisible by downsample");
  require(near(left.p[0], right.p[0]) && near(left.p[5], right.p[5]) && near(left.p[6], right.p[6]),
          "horizontal rectified pair needs matching fx, fy and cy");
  left_ = scaled(left, downsample);
  right_ = scaled(right, downsample);
  if (config.calibration_mode == CalibrationMode::kGazeboNegative5cm) {
    require(
        near(left.p[3], right.p[3]),
        "Gazebo override needs uncorrected equal-Tx info; fixed info requires camera_info mode");
    left_.p[3] = 0.0;
    right_.p[3] = right_.p[0] * 0.05;
  } else {
    // Rebase both projection matrices to the left optical origin.
    right_.p[3] -= left_.p[3];
    left_.p[3] = 0.0;
  }
  require(std::isfinite(numerator_px_m()) && std::abs(numerator_px_m()) > 1e-12,
          "stereo baseline must be finite and nonzero");
  require(std::isfinite(principal_difference_px()), "principal point difference must be finite");
}

CameraModel CameraModel::cropped(const cv::Rect &roi) const
{
  require(input_geometry_ == InputGeometry::kAlreadyRectified,
          "cropped camera model requires already_rectified input");
  const cv::Rect full(0, 0, width_px(), height_px());
  require(roi.width > 0 && roi.height > 0 && roi.x >= 0 && roi.y >= 0 &&
              roi.x <= full.width - roi.width && roi.y <= full.height - roi.height,
          "matching ROI must fit the processed camera image");
  auto result = *this;
  for (auto *calibration : {&result.left_, &result.right_}) {
    calibration->width = static_cast<std::uint32_t>(roi.width);
    calibration->height = static_cast<std::uint32_t>(roi.height);
    calibration->k[2] -= roi.x;
    calibration->k[5] -= roi.y;
    calibration->p[2] -= roi.x;
    calibration->p[6] -= roi.y;
  }
  return result;
}

bool CameraModel::matches_input(const CameraCalibration &left, const CameraCalibration &right) const
{
  return same_calibration(source_left_, left) && same_calibration(source_right_, right);
}

void CameraModel::require_search_coverage(const MatcherConfig &matcher,
                                          const DepthConfig &depth) const
{
  const double first = principal_difference_px() + numerator_px_m() / depth.min_depth_m;
  const double last = principal_difference_px() + numerator_px_m() / depth.max_depth_m;
  require(std::min(first, last) >= matcher.min_disparity_px &&
              std::max(first, last) <= matcher.min_disparity_px + matcher.num_disparities_px - 1,
          "matcher interval does not cover calibrated depth limits in processed pixels");
}

const CameraCalibration &CameraModel::left() const
{
  return left_;
}
const CameraCalibration &CameraModel::right() const
{
  return right_;
}
InputGeometry CameraModel::input_geometry() const
{
  return input_geometry_;
}
int CameraModel::width_px() const
{
  return static_cast<int>(left_.width);
}
int CameraModel::height_px() const
{
  return static_cast<int>(left_.height);
}
double CameraModel::numerator_px_m() const
{
  return left_.p[3] - right_.p[3];
}
double CameraModel::principal_difference_px() const
{
  return left_.p[2] - right_.p[2];
}
} // namespace intball2_stereo::core
