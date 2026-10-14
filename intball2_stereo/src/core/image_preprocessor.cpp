#include "intball2_stereo/core/image_preprocessor.hpp"

#include <stdexcept>
#include <utility>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace intball2_stereo::core
{
namespace
{
void resize_area_four(const cv::Mat &input, cv::Mat &output, cv::Size size)
{
  output.create(size, CV_8UC1);
  cv::parallel_for_(cv::Range(0, size.height), [&](const cv::Range &rows) {
    for (int row = rows.start; row < rows.end; ++row) {
      const auto *first = input.ptr<unsigned char>(4 * row);
      const auto *second = input.ptr<unsigned char>(4 * row + 1);
      const auto *third = input.ptr<unsigned char>(4 * row + 2);
      const auto *fourth = input.ptr<unsigned char>(4 * row + 3);
      auto *destination = output.ptr<unsigned char>(row);
      for (int column = 0; column < size.width; ++column) {
        const int source_column = 4 * column;
        int sum = 0;
        for (int offset = 0; offset < 4; ++offset) {
          sum += first[source_column + offset] + second[source_column + offset] +
                 third[source_column + offset] + fourth[source_column + offset];
        }
        // The sum and power-of-two scale are exact; retain OpenCV's area rounding.
        destination[column] =
            cv::saturate_cast<unsigned char>(static_cast<float>(sum) * (1.0F / 16.0F));
      }
    }
  });
}
} // namespace

ImagePreprocessor::ImagePreprocessor(CameraModel model, int block_size_px)
    : model_(std::move(model))
{
  if (block_size_px < 3 || block_size_px > 255 || block_size_px % 2 == 0) {
    throw std::invalid_argument("preprocessor block size must be odd in [3, 255]");
  }
  prepare_maps(model_.left(), left_, output_.left_valid, block_size_px);
  prepare_maps(model_.right(), right_, output_.right_valid, block_size_px);
}

void ImagePreprocessor::prepare_maps(const CameraCalibration &c, Buffers &buffers, cv::Mat &valid,
                                     int block_size_px)
{
  const cv::Size size(model_.width_px(), model_.height_px());
  valid.create(size, CV_8UC1);
  valid.setTo(255);
  if (model_.input_geometry() == InputGeometry::kAlreadyRectified) {
    return;
  }
  const cv::Mat k(3, 3, CV_64FC1, const_cast<double *>(c.k.data()));
  const cv::Mat r(3, 3, CV_64FC1, const_cast<double *>(c.r.data()));
  const cv::Mat p(3, 4, CV_64FC1, const_cast<double *>(c.p.data()));
  const cv::Mat d(c.d, false);
  cv::initUndistortRectifyMap(k, d, r, p, size, CV_32FC1, buffers.map_x, buffers.map_y);
  for (int row = 0; row < size.height; ++row) {
    auto *mask = valid.ptr<unsigned char>(row);
    const auto *x = buffers.map_x.ptr<float>(row);
    const auto *y = buffers.map_y.ptr<float>(row);
    for (int column = 0; column < size.width; ++column) {
      mask[column] = x[column] >= 0 && static_cast<double>(x[column]) <= size.width - 1 &&
                             y[column] >= 0 && static_cast<double>(y[column]) <= size.height - 1
                         ? 255
                         : 0;
    }
  }
  // A valid centre needs valid rectification samples throughout its SGBM block.
  cv::erode(valid, valid,
            cv::getStructuringElement(cv::MORPH_RECT, cv::Size(block_size_px, block_size_px)),
            cv::Point(-1, -1), 1, cv::BORDER_CONSTANT, cv::Scalar(0));
}

void ImagePreprocessor::prepare_image(const ImageFrame &frame, const CameraCalibration &c,
                                      Buffers &buffers, cv::Mat &output,
                                      SourceGrayImage &source_gray)
{
  const auto &input = frame.image.pixels();
  if (input.dims != 2 || input.cols != static_cast<int>(frame.calibration.width) ||
      input.rows != static_cast<int>(frame.calibration.height)) {
    throw std::invalid_argument("image dimensions must match CameraInfo dimensions");
  }
  const bool mono = frame.encoding == ImageEncoding::kMono8;
  const bool bgr = frame.encoding == ImageEncoding::kBgr8;
  const bool rgb = frame.encoding == ImageEncoding::kRgb8;
  if ((!mono && !bgr && !rgb) || input.type() != (mono ? CV_8UC1 : CV_8UC3)) {
    throw std::invalid_argument("input must be mono8, bgr8 or rgb8 with matching cv::Mat type");
  }
  const cv::Size size(static_cast<int>(c.width), static_cast<int>(c.height));
  const cv::Mat *gray = &input;
  if (!mono) {
    cv::cvtColor(input, buffers.gray, bgr ? cv::COLOR_BGR2GRAY : cv::COLOR_RGB2GRAY);
    gray = &buffers.gray;
  }
  source_gray.pixels = *gray;
  source_gray.source_owner = mono ? std::optional<ImageBuffer>(frame.image) : std::nullopt;
  source_gray.stamp = frame.stamp;
  source_gray.optical_frame_id = frame.optical_frame_id;
  source_gray.source_encoding = frame.encoding;
  source_gray.source_data = input.data;
  if (model_.input_geometry() == InputGeometry::kAlreadyRectified && gray->size() != size) {
    // Resize owns its destination; no intermediate copy is needed for rectified input.
    if (gray->cols % 4 == 0 && gray->rows % 4 == 0 && gray->cols / 4 == size.width &&
        gray->rows / 4 == size.height) {
      resize_area_four(*gray, output, size);
    } else {
      cv::resize(*gray, output, size, 0, 0, cv::INTER_AREA);
    }
    return;
  }
  if (gray->size() != size) {
    cv::resize(*gray, buffers.resized, size, 0, 0, cv::INTER_AREA);
    gray = &buffers.resized;
  }
  if (model_.input_geometry() == InputGeometry::kRaw) {
    cv::remap(*gray, output, buffers.map_x, buffers.map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT,
              cv::Scalar(0));
  } else {
    // Owned/reusable output; never return a borrow of the source message.
    gray->copyTo(output);
  }
}

const PreparedImages &ImagePreprocessor::prepare(const ImagePair &pair)
{
  if (!model_.matches_input(pair.left.calibration, pair.right.calibration)) {
    throw std::invalid_argument("preprocessor calibration changed; rebuild cached maps");
  }
  prepare_image(pair.left, model_.left(), left_, output_.left_gray, output_.left_source_gray);
  prepare_image(pair.right, model_.right(), right_, output_.right_gray, output_.right_source_gray);
  return output_;
}

const CameraModel &ImagePreprocessor::camera_model() const
{
  return model_;
}
} // namespace intball2_stereo::core
