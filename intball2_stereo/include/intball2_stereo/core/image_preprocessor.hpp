#ifndef INTBALL2_STEREO_CORE_IMAGE_PREPROCESSOR_HPP_
#define INTBALL2_STEREO_CORE_IMAGE_PREPROCESSOR_HPP_

#include "intball2_stereo/core/camera_model.hpp"

namespace intball2_stereo::core
{
struct PreparedImages
{
  SourceGrayImage left_source_gray;
  SourceGrayImage right_source_gray;
  cv::Mat left_gray;
  cv::Mat right_gray;
  cv::Mat left_valid;
  cv::Mat right_valid;
};

class ImagePreprocessor
{
public:
  explicit ImagePreprocessor(CameraModel model, int block_size_px);
  ImagePreprocessor(const ImagePreprocessor &) = delete;
  ImagePreprocessor &operator=(const ImagePreprocessor &) = delete;
  // Views remain valid until the next prepare() or destruction.
  const PreparedImages &prepare(const ImagePair &pair);
  const CameraModel &camera_model() const;

private:
  struct Buffers
  {
    cv::Mat resized;
    cv::Mat gray;
    cv::Mat map_x;
    cv::Mat map_y;
  };
  void prepare_image(const ImageFrame &frame, const CameraCalibration &calibration,
                     Buffers &buffers, cv::Mat &output, SourceGrayImage &source_gray);
  void prepare_maps(const CameraCalibration &calibration, Buffers &buffers, cv::Mat &valid,
                    int block_size_px);
  CameraModel model_;
  Buffers left_;
  Buffers right_;
  PreparedImages output_;
};
} // namespace intball2_stereo::core
#endif // INTBALL2_STEREO_CORE_IMAGE_PREPROCESSOR_HPP_
