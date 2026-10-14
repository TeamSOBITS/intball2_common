#include "intball2_stereo/core/stereo_processor.hpp"
#include <dlfcn.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
namespace
{
using namespace intball2_stereo::core;

ImageFrame read_frame(std::istream &input)
{
  std::string path, encoding, frame;
  int width, height;
  std::size_t step;
  std::int64_t stamp;
  if (!(input >> std::quoted(path) >> width >> height >> step >> encoding >> stamp >>
        std::quoted(frame))) {
    throw std::invalid_argument("invalid image manifest");
  }
  if (width <= 0 || height <= 0)
    throw std::invalid_argument("invalid image shape");
  int channels = encoding == "mono8" ? 1 : 3;
  if (encoding != "mono8" && encoding != "rgb8" && encoding != "bgr8")
    throw std::invalid_argument("unsupported raw image encoding");
  if (step > std::numeric_limits<std::size_t>::max() / static_cast<std::size_t>(height)) {
    throw std::invalid_argument("raw image byte size overflow");
  }
  if (step < static_cast<std::size_t>(width) * channels)
    throw std::invalid_argument("short row step");
  std::ifstream file(path, std::ios::binary);
  if (!file)
    throw std::invalid_argument("missing raw byte file");
  std::vector<unsigned char> bytes((std::istreambuf_iterator<char>(file)), {});
  if (bytes.size() < step * static_cast<std::size_t>(height))
    throw std::invalid_argument("short raw bytes");
  cv::Mat view(height, width, CV_MAKETYPE(CV_8U, channels), bytes.data(), step);
  CameraCalibration c;
  input >> c.width >> c.height >> std::quoted(c.distortion_model);
  for (auto &v : c.k)
    input >> v;
  for (auto &v : c.r)
    input >> v;
  for (auto &v : c.p)
    input >> v;
  std::size_t d_size;
  if (!(input >> d_size)) {
    throw std::invalid_argument("invalid calibration manifest");
  }
  if (d_size > 100)
    throw std::invalid_argument("excess distortion coefficient count");
  c.d.resize(d_size);
  for (auto &v : c.d)
    input >> v;
  input >> c.binning_x >> c.binning_y >> c.roi.x_offset >> c.roi.y_offset >> c.roi.width >>
      c.roi.height >> c.roi.do_rectify;
  if (!input || c.width != static_cast<unsigned>(width) ||
      c.height != static_cast<unsigned>(height)) {
    throw std::invalid_argument("invalid calibration shape or manifest");
  }
  return {ImageBuffer::copy_from(view),
          c,
          {stamp},
          frame,
          encoding == "mono8"  ? ImageEncoding::kMono8
          : encoding == "rgb8" ? ImageEncoding::kRgb8
                               : ImageEncoding::kBgr8};
}
} // namespace

int main(int argc, char **argv)
{
  try {
    using namespace intball2_stereo::core;
    if (argc != 4 && argc != 5)
      throw std::invalid_argument("manifest depth.bin metadata.txt required");
    std::ifstream input(argv[1]);
    if (!input)
      throw std::invalid_argument("missing manifest");
    StereoConfig config;
    if (argc == 5) {
      if (std::string(argv[4]) != "--refine") {
        throw std::invalid_argument("unknown replay option");
      }
      config.refinement.enabled = true;
    }
    std::string calibration_mode, geometry, units, layout;
    input >> calibration_mode >> geometry >> config.processing.downsample >>
        config.processing.opencv_threads >> config.matcher.min_disparity_px >>
        config.matcher.num_disparities_px >> config.matcher.block_size_px >>
        config.matcher.uniqueness_ratio >> config.matcher.speckle_window_size_px >>
        config.matcher.speckle_range_px >> config.matcher.disp12_max_diff_px >> units >>
        config.depth.min_depth_m >> config.depth.max_depth_m >> config.points.publish_points >>
        layout >> config.points.stride;
    if (calibration_mode != "gazebo_negative_5cm" || geometry != "already_rectified" ||
        units != "processed_pixels" || layout != "organized")
      throw std::invalid_argument("runner only supports verified captured config enums");
    config.camera.calibration_mode = CalibrationMode::kGazeboNegative5cm;
    config.camera.input_geometry = InputGeometry::kAlreadyRectified;
    config.matcher.disparity_units = DisparityUnits::kProcessedPixels;
    config.points.layout = PointsLayout::kOrganized;
    auto left = read_frame(input);
    auto right = read_frame(input);
    if (!input)
      throw std::invalid_argument("invalid manifest");
    cv::setNumThreads(config.processing.opencv_threads == 1 ? 0 : config.processing.opencv_threads);
    StereoProcessor processor(config);
    const auto &result = processor.process({std::move(left), std::move(right)});
    std::ofstream depth(argv[2], std::ios::binary);
    const auto row_bytes = static_cast<std::size_t>(result.depth_m.cols) * sizeof(float);
    if (row_bytes > static_cast<std::size_t>(std::numeric_limits<std::streamsize>::max())) {
      throw std::invalid_argument("depth row size exceeds stream write limits");
    }
    for (int row = 0; row < result.depth_m.rows; ++row) {
      depth.write(reinterpret_cast<const char *>(result.depth_m.ptr<float>(row)),
                  static_cast<std::streamsize>(row_bytes));
    }
    std::ofstream metadata(argv[3]);
    metadata << result.depth_m.cols << ' ' << result.depth_m.rows << ' ' << result.stamp.nanoseconds
             << ' ' << std::quoted(result.optical_frame_id) << '\n';
    metadata << std::setprecision(17);
    for (auto v : result.calibration.k)
      metadata << v << ' ';
    metadata << '\n';
    for (auto v : result.calibration.p)
      metadata << v << ' ';
    metadata << '\n';
    Dl_info library_info{};
    if (dladdr(reinterpret_cast<const void *>(&require_valid_config), &library_info) == 0 ||
        !library_info.dli_fname) {
      throw std::runtime_error("cannot identify loaded core library");
    }
    metadata << std::quoted(library_info.dli_fname) << '\n';
    metadata << config.refinement.enabled << ' ' << config.refinement.downsample << ' '
             << config.refinement.window_size_px << ' ' << config.refinement.search_radius_px << ' '
             << config.refinement.min_texture_variance << ' ' << config.refinement.min_correlation
             << ' ' << result.statistics.refinement_attempted_pixels << ' '
             << result.statistics.refinement_updated_pixels << ' '
             << result.statistics.refinement_retained_pixels << '\n';
    if (!depth || !metadata)
      throw std::runtime_error("output write failed");
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
