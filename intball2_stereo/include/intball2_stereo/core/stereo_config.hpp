#ifndef INTBALL2_STEREO_CORE_STEREO_CONFIG_HPP_
#define INTBALL2_STEREO_CORE_STEREO_CONFIG_HPP_

#include <string>
#include <vector>

#include "intball2_stereo/core/stereo_types.hpp"

namespace intball2_stereo::core
{
enum class CalibrationMode
{
  kCameraInfo,
  kGazeboNegative5cm
};

enum class InputGeometry
{
  kRaw,
  kAlreadyRectified
};

// The only supported unit is pixels AFTER downsampling, never SGBM fixed-point.
enum class DisparityUnits
{
  kProcessedPixels
};

struct CameraConfig
{
  CalibrationMode calibration_mode = CalibrationMode::kGazeboNegative5cm;
  InputGeometry input_geometry = InputGeometry::kAlreadyRectified;
};

struct MatcherConfig
{
  // Initial 200px Gazebo search: fx ~= 119px, |B| = 0.05m gives
  // disparities ~= [-23.8, -2.0] over 0.25--3m. Only width needs /16.
  int min_disparity_px = -25;
  int num_disparities_px = 32;
  int block_size_px = 5;
  int uniqueness_ratio = 10;
  int speckle_window_size_px = 100;
  int speckle_range_px = 2;
  int disp12_max_diff_px = 1;
  DisparityUnits disparity_units = DisparityUnits::kProcessedPixels;
};

struct DepthConfig
{
  double min_depth_m = 0.25;
  double max_depth_m = 3.0;
};

struct PointsConfig
{
  bool publish_points = false;
  PointsLayout layout = PointsLayout::kOrganized;
  int stride = 1;
};

struct ProcessingConfig
{
  int downsample = 4;
  int opencv_threads = 1;
  // Symmetric stereo crop margins in the published (downsampled) output grid.
  int roi_left_px = 0;
  int roi_right_px = 0;
  int roi_top_px = 0;
  int roi_bottom_px = 0;
};

struct RefinementConfig
{
  bool enabled = false;
  int downsample = 2;
  int window_size_px = 9;
  int search_radius_px = 2;
  double min_texture_variance = 1.0;
  double min_correlation = 0.7;
};

struct StereoConfig
{
  CameraConfig camera;
  MatcherConfig matcher;
  DepthConfig depth;
  PointsConfig points;
  ProcessingConfig processing;
  RefinementConfig refinement;
};

// Collect errors without logging, reading parameters, or applying computation.
std::vector<std::string> validate_config(const StereoConfig &config);
// Additional bounds require actual input dimensions; no resizing is performed.
std::vector<std::string> validate_config_for_input(const StereoConfig &config, int input_width_px,
                                                   int input_height_px);
void require_valid_config(const StereoConfig &config);
} // namespace intball2_stereo::core

#endif // INTBALL2_STEREO_CORE_STEREO_CONFIG_HPP_
