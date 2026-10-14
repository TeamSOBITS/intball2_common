// Offline replay uses the installed public library; it does not initialize a ROS graph.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <dlfcn.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <stereo_image_proc/stereo_processor.hpp>

namespace
{
struct Frame
{
  cv::Mat pixels;
  sensor_msgs::msg::CameraInfo info;
  std::string frame;
  std::int64_t stamp_ns = 0;
};

struct Settings
{
  int factor = 1;
  int minimum = -98;
  int range = 128;
  int block = 15;
  int uniqueness = 5;
  int speckle_size = 375;
  int speckle_range = 10;
  int disp12 = 67;
  int cap = 12;
};

Frame read_frame(std::istream &input)
{
  std::string filename, encoding;
  Frame result;
  int width = 0, height = 0;
  std::size_t step = 0;
  input >> std::quoted(filename) >> width >> height >> step >> encoding >> result.stamp_ns >>
      std::quoted(result.frame);
  const int channels = encoding == "mono8" ? 1 : 3;
  if (!input || width <= 0 || height <= 0 ||
      (encoding != "mono8" && encoding != "rgb8" && encoding != "bgr8") ||
      step < static_cast<std::size_t>(width) * channels ||
      step > std::numeric_limits<std::size_t>::max() / static_cast<std::size_t>(height)) {
    throw std::invalid_argument("invalid replay image layout");
  }
  std::ifstream raw(filename, std::ios::binary);
  std::vector<unsigned char> bytes((std::istreambuf_iterator<char>(raw)), {});
  if (bytes.size() < step * static_cast<std::size_t>(height)) {
    throw std::invalid_argument("short replay image");
  }
  const cv::Mat view(height, width, CV_MAKETYPE(CV_8U, channels), bytes.data(), step);
  if (channels == 1) {
    result.pixels = view.clone();
  } else {
    cv::cvtColor(view, result.pixels, encoding == "rgb8" ? cv::COLOR_RGB2GRAY : cv::COLOR_BGR2GRAY);
  }
  auto &info = result.info;
  input >> info.width >> info.height >> std::quoted(info.distortion_model);
  for (auto &value : info.k)
    input >> value;
  for (auto &value : info.r)
    input >> value;
  for (auto &value : info.p)
    input >> value;
  std::size_t distortion_size = 0;
  input >> distortion_size;
  if (!input || distortion_size > 100)
    throw std::invalid_argument("invalid distortion length");
  info.d.resize(distortion_size);
  for (auto &value : info.d)
    input >> value;
  input >> info.binning_x >> info.binning_y >> info.roi.x_offset >> info.roi.y_offset >>
      info.roi.width >> info.roi.height >> info.roi.do_rectify;
  if (!input || info.width != static_cast<unsigned>(width) ||
      info.height != static_cast<unsigned>(height))
    throw std::invalid_argument("calibration shape mismatch");
  const std::array<double, 9> identity{1, 0, 0, 0, 1, 0, 0, 0, 1};
  if (info.r != identity || info.binning_x || info.binning_y || info.roi.x_offset ||
      info.roi.y_offset ||
      std::any_of(info.d.begin(), info.d.end(), [](double value) { return value != 0; })) {
    throw std::invalid_argument(
        "runner requires the archived already-rectified zero-distortion pair");
  }
  return result;
}

void scale(Frame &frame, int factor)
{
  if (frame.info.width % factor || frame.info.height % factor)
    throw std::invalid_argument("nondivisible resize");
  const double scale = 1. / factor;
  const double offset = (scale - 1.) * .5;
  auto &c = frame.info;
  for (int column = 0; column < 3; ++column) {
    c.k[column] = scale * c.k[column] + offset * c.k[6 + column];
    c.k[3 + column] = scale * c.k[3 + column] + offset * c.k[6 + column];
  }
  for (int column = 0; column < 4; ++column) {
    c.p[column] = scale * c.p[column] + offset * c.p[8 + column];
    c.p[4 + column] = scale * c.p[4 + column] + offset * c.p[8 + column];
  }
  c.width /= factor;
  c.height /= factor;
  c.roi = sensor_msgs::msg::RegionOfInterest{};
  if (factor != 1)
    cv::resize(frame.pixels, frame.pixels,
               cv::Size(static_cast<int>(c.width), static_cast<int>(c.height)), 0, 0,
               cv::INTER_AREA);
}

void configure(stereo_image_proc::StereoProcessor &processor, const Settings &settings)
{
  processor.setStereoType(stereo_image_proc::StereoProcessor::SGBM);
  processor.setSgbmMode(cv::StereoSGBM::MODE_SGBM);
  processor.setMinDisparity(settings.minimum);
  processor.setDisparityRange(settings.range);
  processor.setCorrelationWindowSize(settings.block);
  processor.setUniquenessRatio(static_cast<float>(settings.uniqueness));
  processor.setSpeckleSize(settings.speckle_size);
  processor.setSpeckleRange(settings.speckle_range);
  processor.setDisp12MaxDiff(settings.disp12);
  processor.setPreFilterCap(settings.cap);
  processor.setP1(200);
  processor.setP2(800);
}

cv::Mat depth_from_disparity(const stereo_msgs::msg::DisparityImage &message,
                             const image_geometry::StereoCameraModel &model,
                             const Settings &settings)
{
  const auto &image = message.image;
  if (image.width > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
      image.height > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
      image.encoding != "32FC1" || image.is_bigendian || image.step < image.width * sizeof(float) ||
      image.data.size() < static_cast<std::size_t>(image.step) * image.height) {
    throw std::invalid_argument("unexpected SIP disparity encoding/layout");
  }
  const cv::Mat disparity(static_cast<int>(image.height), static_cast<int>(image.width), CV_32FC1,
                          const_cast<unsigned char *>(image.data.data()), image.step);
  cv::Mat depth(disparity.size(), CV_32FC1, cv::Scalar(std::numeric_limits<float>::quiet_NaN()));
  const cv::Rect full(0, 0, disparity.cols, disparity.rows);
  // processDisparity does not promise a populated message.valid_window. Match the explicit support
  // contract.
  const auto roi =
      cv::getValidDisparityROI(full, full, settings.minimum, settings.range, settings.block);
  const double principal_difference = model.left().cx() - model.right().cx();
  const double invalid = settings.minimum - 1. - principal_difference;
  for (int y = roi.y; y < roi.y + roi.height; ++y) {
    for (int x = roi.x; x < roi.x + roi.width; ++x) {
      const auto d = disparity.at<float>(y, x);
      if (!std::isfinite(d) || d <= invalid || std::abs(d) <= 1e-12)
        continue;
      const double z = static_cast<double>(message.f) * message.t / d;
      if (std::isfinite(z) && z >= .25 && z <= 3.)
        depth.at<float>(y, x) = static_cast<float>(z);
    }
  }
  return depth;
}

std::string loaded_library()
{
  constexpr auto kSymbol =
      "_ZNK17stereo_image_proc15StereoProcessor16processDisparityERKN2cv3MatES4_RKN14image_"
      "geometry17StereoCameraModelERN11stereo_msgs3msg15DisparityImage_ISaIvEEE";
  Dl_info info{};
  const auto *address = dlsym(RTLD_DEFAULT, kSymbol);
  if (!address || !dladdr(address, &info) || !info.dli_fname)
    throw std::runtime_error("SIP library provenance unavailable");
  return info.dli_fname;
}

void write_matrix(const std::string &filename, const cv::Mat &matrix)
{
  std::ofstream output(filename, std::ios::binary);
  for (int y = 0; y < matrix.rows; ++y)
    output.write(reinterpret_cast<const char *>(matrix.ptr<float>(y)),
                 static_cast<std::streamsize>(matrix.cols) *
                     static_cast<std::streamsize>(sizeof(float)));
  if (!output)
    throw std::runtime_error("output write failed");
}

void self_test()
{
  for (const int sign : {-1, 1}) {
    sensor_msgs::msg::CameraInfo left, right;
    left.width = right.width = 200;
    left.height = right.height = 200;
    left.k = right.k = {100, 0, 99.5, 0, 100, 99.5, 0, 0, 1};
    left.r = right.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    left.p = {100, 0, 99.5, 0, 0, 100, 99.5, 0, 0, 0, 1, 0};
    right.p = left.p;
    right.p[2] -= sign * 2;
    right.p[3] = -sign * 8;
    image_geometry::StereoCameraModel model;
    model.fromCameraInfo(left, right);
    cv::Mat a(200, 200, CV_8UC1), b(200, 200, CV_8UC1, cv::Scalar(0));
    cv::RNG random(483);
    random.fill(a, cv::RNG::UNIFORM, 0, 256);
    a(cv::Rect(sign < 0 ? 0 : 8, 0, 192, 200)).copyTo(b(cv::Rect(sign < 0 ? 8 : 0, 0, 192, 200)));
    Settings settings;
    settings.minimum = sign < 0 ? -17 : 0;
    settings.range = 32;
    settings.block = 5;
    stereo_image_proc::StereoProcessor processor;
    configure(processor, settings);
    stereo_msgs::msg::DisparityImage result;
    result.header.frame_id = "left_optical";
    result.header.stamp.sec = 123;
    result.header.stamp.nanosec = 7;
    processor.processDisparity(a, b, model, result);
    const auto depth = depth_from_disparity(result, model, settings);
    if (!std::isfinite(depth.at<float>(100, 100)) ||
        std::abs(depth.at<float>(100, 100) - 4. / 3.) > .04 || (result.t > 0) != (sign > 0) ||
        !std::isnan(depth.at<float>(0, 0)) || result.header.frame_id != "left_optical" ||
        result.header.stamp.sec != 123 || result.header.stamp.nanosec != 7) {
      throw std::runtime_error("SIP signed baseline/principal correction/support test failed");
    }
    auto sentinel = result;
    const auto invalid =
        static_cast<float>(settings.minimum - 1. - (model.left().cx() - model.right().cx()));
    std::memcpy(sentinel.image.data.data() + static_cast<std::size_t>(100) * sentinel.image.step +
                    100 * sizeof(float),
                &invalid, sizeof(invalid));
    if (!std::isnan(depth_from_disparity(sentinel, model, settings).at<float>(100, 100))) {
      throw std::runtime_error(
          "corrected min-1 sentinel must remain invalid even with negative f*t");
    }
  }
  std::cout << "SIP signed baseline/principal correction/support tests passed\n";
}
} // namespace

int main(int argc, char **argv)
{
  try {
    cv::setNumThreads(0); // The one-thread policy matches the tested production wrapper.
    if (argc == 2 && std::string(argv[1]) == "--self-test") {
      self_test();
      return 0;
    }
    if (argc != 6)
      throw std::invalid_argument(
          "manifest disparity.bin depth.bin metadata.json common800|ours200 required");
    const std::string profile(argv[5]);
    Settings settings;
    if (profile == "ours200")
      settings = {4, -25, 32, 5, 10, 100, 2, 1, 31};
    else if (profile != "common800")
      throw std::invalid_argument("unknown profile");
    std::ifstream input(argv[1]);
    std::string config_line;
    std::getline(input, config_line); // Reuse replay_capture's portable frame records; profile
                                      // settings are explicit below.
    auto left = read_frame(input), right = read_frame(input);
    if (left.info.p[3] != right.info.p[3] || left.info.width != 800 || left.info.height != 800)
      throw std::invalid_argument("expected uncorrected Gazebo 800 pair");
    left.info.p[3] = 0;
    right.info.p[3] = right.info.p[0] * .05;
    scale(left, settings.factor);
    scale(right, settings.factor);
    image_geometry::StereoCameraModel model;
    model.fromCameraInfo(left.info, right.info);
    stereo_image_proc::StereoProcessor processor;
    configure(processor, settings);
    stereo_msgs::msg::DisparityImage result;
    processor.processDisparity(left.pixels, right.pixels, model, result);
    result.header.frame_id = left.frame;
    result.header.stamp.sec = static_cast<std::int32_t>(left.stamp_ns / 1000000000);
    result.header.stamp.nanosec = static_cast<std::uint32_t>(left.stamp_ns % 1000000000);
    result.image.header = result.header;
    const auto depth = depth_from_disparity(result, model, settings);
    const cv::Mat disparity(static_cast<int>(result.image.height),
                            static_cast<int>(result.image.width), CV_32FC1,
                            result.image.data.data(), result.image.step);
    write_matrix(argv[2], disparity);
    write_matrix(argv[3], depth);
    const cv::Rect full(0, 0, depth.cols, depth.rows);
    const auto eligible =
        cv::getValidDisparityROI(full, full, settings.minimum, settings.range, settings.block);
    std::ofstream output(argv[4]);
    output << std::setprecision(17) << "{\"profile\":" << std::quoted(profile)
           << ",\"loaded_library\":" << std::quoted(loaded_library()) << ",\"width\":" << depth.cols
           << ",\"height\":" << depth.rows << ",\"source_stamp_ns\":" << left.stamp_ns
           << ",\"right_stamp_ns\":" << right.stamp_ns
           << ",\"frame_id\":" << std::quoted(left.frame) << ",\"f\":" << result.f
           << ",\"t\":" << result.t << ",\"delta_d\":" << result.delta_d
           << ",\"minimum\":" << settings.minimum << ",\"range\":" << settings.range
           << ",\"block\":" << settings.block << ",\"uniqueness\":" << settings.uniqueness
           << ",\"speckle_size\":" << settings.speckle_size
           << ",\"speckle_range\":" << settings.speckle_range << ",\"disp12\":" << settings.disp12
           << ",\"pre_filter_cap\":" << settings.cap
           << ",\"P1\":200,\"P2\":800,\"mode\":0,\"opencv_threads\":1,\"min_depth_m\":0.25,\"max_"
              "depth_m\":3.0,\"k\":[";
    for (std::size_t i = 0; i < left.info.k.size(); ++i)
      output << (i ? "," : "") << left.info.k[i];
    output << "],\"p\":[";
    for (std::size_t i = 0; i < left.info.p.size(); ++i)
      output << (i ? "," : "") << left.info.p[i];
    output << "],\"computed_valid_roi\":[" << eligible.x << ',' << eligible.y << ','
           << eligible.width << ',' << eligible.height << "],\"returned_valid_window\":["
           << result.valid_window.x_offset << ',' << result.valid_window.y_offset << ','
           << result.valid_window.width << ',' << result.valid_window.height << "]}\n";
    if (!output)
      throw std::runtime_error("metadata write failed");
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
