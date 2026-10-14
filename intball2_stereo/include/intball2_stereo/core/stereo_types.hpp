#ifndef INTBALL2_STEREO_CORE_STEREO_TYPES_HPP_
#define INTBALL2_STEREO_CORE_STEREO_TYPES_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

namespace intball2_stereo::core
{
// Source camera clock only. Never subtract a steady_clock time from this stamp.
struct CaptureStamp
{
  std::int64_t nanoseconds = 0;
};

struct RegionOfInterest
{
  std::uint32_t x_offset = 0;
  std::uint32_t y_offset = 0;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  bool do_rectify = false;
};

struct CameraCalibration
{
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::string distortion_model;
  std::array<double, 9> k{};
  std::vector<double> d;
  std::array<double, 9> r{};
  // Preserve the signed translation. P[3] = -fx * signed_baseline_m.
  // Positive right P[3] therefore implies a negative signed baseline.
  std::array<double, 12> p{};
  RegionOfInterest roi;
  // Preserve CameraInfo zero (unspecified/equivalent to one); conversion is later.
  std::uint32_t binning_x = 0;
  std::uint32_t binning_y = 0;
};

// A view is read-only by contract. Copies keep OpenCV storage or an external owner
// alive. Borrowing without a lifetime owner is deliberately unavailable.
class ImageBuffer
{
public:
  static ImageBuffer copy_from(const cv::Mat &pixels)
  {
    if (pixels.empty()) {
      throw std::invalid_argument("image must not be empty");
    }
    return ImageBuffer(pixels.clone(), {});
  }

  static ImageBuffer borrow(const cv::Mat &pixels, std::shared_ptr<const void> owner)
  {
    if (pixels.empty() || !owner) {
      throw std::invalid_argument("borrowed image needs pixels and a lifetime owner");
    }
    return ImageBuffer(pixels, std::move(owner));
  }

  const cv::Mat &pixels() const
  {
    return pixels_;
  }

private:
  ImageBuffer(cv::Mat pixels, std::shared_ptr<const void> owner)
      : pixels_(std::move(pixels)), owner_(std::move(owner))
  {
  }

  cv::Mat pixels_;
  std::shared_ptr<const void> owner_;
};

enum class ImageEncoding
{
  kMono8,
  kBgr8,
  kRgb8
};

struct ImageFrame
{
  ImageBuffer image;
  CameraCalibration calibration;
  CaptureStamp stamp;
  std::string optical_frame_id;
  ImageEncoding encoding = ImageEncoding::kMono8;
};

// Source-resolution gray view, valid until the producing preprocessor's next prepare().
// Mono retains the external owner without copying pixels; colour conversion owns pixels.
struct SourceGrayImage
{
  cv::Mat pixels;
  std::optional<ImageBuffer> source_owner;
  CaptureStamp stamp;
  std::string optical_frame_id;
  ImageEncoding source_encoding = ImageEncoding::kMono8;
  const unsigned char *source_data = nullptr;
};

struct ImagePair
{
  ImageFrame left;
  ImageFrame right;
};

enum class PointsLayout
{
  kOrganized,
  kCompact
};

struct PointXyz
{
  float x_m;
  float y_m;
  float z_m;
};

// Organized: width/height match depth and invalid pixels remain NaN XYZ.
// Compact: height=1, width=points.size(), invalid points are omitted.
struct PointCloudResult
{
  std::vector<PointXyz> points;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  PointsLayout layout = PointsLayout::kOrganized;
  CaptureStamp stamp;
  std::string optical_frame_id;
};

struct ProcessingStatistics
{
  double synchronization_wait_s = 0.0;
  double preprocessing_s = 0.0;
  double matching_s = 0.0;
  double quality_filter_s = 0.0;
  double depth_conversion_s = 0.0;
  double pointcloud_generation_s = 0.0;
  double valid_depth_fraction = 0.0;
  std::uint64_t dropped_pairs = 0;
  std::uint64_t matching_input_pixels = 0;
  std::uint64_t full_disparity_eligible_pixels = 0;
  std::uint64_t roi_disparity_eligible_pixels = 0;
  std::uint64_t roi_additional_search_excluded_pixels = 0;
  std::uint64_t refinement_attempted_pixels = 0;
  std::uint64_t refinement_updated_pixels = 0;
  std::uint64_t refinement_retained_pixels = 0;
};

// Processor output: CV_32FC1 optical-axis depth in metres; invalid=NaN.
// Stamp/frame/calibration refer to the left image, calibration to output pixels.
struct DepthResult
{
  cv::Mat depth_m;
  CameraCalibration calibration;
  CaptureStamp stamp;
  std::string optical_frame_id;
  ProcessingStatistics statistics;
};
} // namespace intball2_stereo::core

#endif // INTBALL2_STEREO_CORE_STEREO_TYPES_HPP_
