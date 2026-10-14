#include "intball2_stereo/core/depth_refiner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace intball2_stereo::core
{
DepthRefiner::DepthRefiner(RefinementConfig config) : config_(config)
{
  StereoConfig check;
  check.refinement = config;
  require_valid_config(check);
}

void DepthRefiner::prepare(const cv::Mat &input, int ratio, Buffers &buffers)
{
  auto &resized = buffers.resized;
  const cv::Mat *gray = &input;
  if (config_.downsample != 1) {
    cv::resize(*gray, resized,
               cv::Size(input.cols / config_.downsample, input.rows / config_.downsample), 0, 0,
               cv::INTER_AREA);
    gray = &resized;
  }
  if (ratio % 2 == 0) {
    // Exact uint8 sums encode quarter integers at the half-pixel output rays.
    cv::boxFilter(*gray, buffers.scaled_quarters, CV_16UC1, cv::Size(2, 2), cv::Point(0, 0), false,
                  cv::BORDER_CONSTANT | cv::BORDER_ISOLATED);
  } else {
    gray->convertTo(buffers.scaled_quarters, CV_16UC1, 4.0);
  }
  const cv::Size window(config_.window_size_px, config_.window_size_px);
  // Integer quarter samples keep both moments exact without a float image conversion.
  cv::boxFilter(buffers.scaled_quarters, buffers.window_sum, CV_64F, window, cv::Point(-1, -1),
                false, cv::BORDER_CONSTANT);
  cv::sqrBoxFilter(buffers.scaled_quarters, buffers.window_square_sum, CV_64F, window,
                   cv::Point(-1, -1), false, cv::BORDER_CONSTANT);
}

void DepthRefiner::validate_inputs(const ImagePair &pair, const CameraModel &model,
                                   int matching_downsample, const DepthConfig &depth_config,
                                   const cv::Mat &depth_m) const
{
  if (!std::isfinite(depth_config.min_depth_m) || depth_config.min_depth_m <= 0 ||
      !std::isfinite(depth_config.max_depth_m) ||
      depth_config.max_depth_m <= depth_config.min_depth_m) {
    throw std::invalid_argument("refinement depth interval must be finite and positive");
  }
  if (model.input_geometry() != InputGeometry::kAlreadyRectified ||
      (matching_downsample != 1 && matching_downsample != 2 && matching_downsample != 4) ||
      matching_downsample < config_.downsample || matching_downsample % config_.downsample ||
      depth_m.dims != 2 || depth_m.type() != CV_32FC1 || depth_m.cols != model.width_px() ||
      depth_m.rows != model.height_px()) {
    throw std::invalid_argument("refinement needs rectified, model-sized depth and integral ratio");
  }
  for (const auto *frame : {&pair.left, &pair.right}) {
    const auto &pixels = frame->image.pixels();
    if (reinterpret_cast<std::uintptr_t>(depth_m.datastart) <
            reinterpret_cast<std::uintptr_t>(pixels.dataend) &&
        reinterpret_cast<std::uintptr_t>(pixels.datastart) <
            reinterpret_cast<std::uintptr_t>(depth_m.dataend)) {
      throw std::invalid_argument("refinement depth must not alias source storage");
    }
    const bool mono = frame->encoding == ImageEncoding::kMono8;
    if (pixels.dims != 2 || pixels.type() != (mono ? CV_8UC1 : CV_8UC3) ||
        (!mono && frame->encoding != ImageEncoding::kRgb8 &&
         frame->encoding != ImageEncoding::kBgr8) ||
        pixels.cols != static_cast<std::int64_t>(model.width_px()) * matching_downsample ||
        pixels.rows != static_cast<std::int64_t>(model.height_px()) * matching_downsample) {
      throw std::invalid_argument("refinement source dimensions/encoding must match camera model");
    }
  }
}

void DepthRefiner::refine(const ImagePair &pair, const CameraModel &model, int matching_downsample,
                          const DepthConfig &depth_config, cv::Mat &depth_m,
                          ProcessingStatistics &statistics)
{
  if (!config_.enabled) {
    return;
  }
  validate_inputs(pair, model, matching_downsample, depth_config, depth_m);
  const auto gray = [](const ImageFrame &frame, cv::Mat &converted) -> const cv::Mat & {
    if (frame.encoding == ImageEncoding::kMono8) {
      return frame.image.pixels();
    }
    cv::cvtColor(frame.image.pixels(), converted,
                 frame.encoding == ImageEncoding::kBgr8 ? cv::COLOR_BGR2GRAY : cv::COLOR_RGB2GRAY);
    return converted;
  };
  const auto &left_gray = gray(pair.left, left_.colour_gray);
  const auto &right_gray = gray(pair.right, right_.colour_gray);
  refine_gray(left_gray, right_gray, model, matching_downsample, depth_config, depth_m, statistics);
}

void DepthRefiner::refine(const ImagePair &pair, const SourceGrayImage &left_gray,
                          const SourceGrayImage &right_gray, const CameraModel &model,
                          int matching_downsample, const DepthConfig &depth_config,
                          cv::Mat &depth_m, ProcessingStatistics &statistics)
{
  if (!config_.enabled) {
    return;
  }
  validate_inputs(pair, model, matching_downsample, depth_config, depth_m);
  const auto validate = [&depth_m](const ImageFrame &frame, const SourceGrayImage &gray) {
    const auto &source = frame.image.pixels();
    if (gray.pixels.dims != 2 || gray.pixels.type() != CV_8UC1 ||
        gray.pixels.size() != source.size() || gray.stamp.nanoseconds != frame.stamp.nanoseconds ||
        gray.optical_frame_id != frame.optical_frame_id || gray.source_encoding != frame.encoding ||
        gray.source_data != source.data) {
      throw std::invalid_argument("provided gray must describe the exact source frame");
    }
    if (reinterpret_cast<std::uintptr_t>(depth_m.datastart) <
            reinterpret_cast<std::uintptr_t>(gray.pixels.dataend) &&
        reinterpret_cast<std::uintptr_t>(gray.pixels.datastart) <
            reinterpret_cast<std::uintptr_t>(depth_m.dataend)) {
      throw std::invalid_argument("refinement depth must not alias provided gray storage");
    }
  };
  validate(pair.left, left_gray);
  validate(pair.right, right_gray);
  refine_gray(left_gray.pixels, right_gray.pixels, model, matching_downsample, depth_config,
              depth_m, statistics);
}

void DepthRefiner::refine_gray(const cv::Mat &left_gray, const cv::Mat &right_gray,
                               const CameraModel &model, int matching_downsample,
                               const DepthConfig &depth_config, cv::Mat &depth_m,
                               ProcessingStatistics &statistics)
{
  const int ratio = matching_downsample / config_.downsample;
  prepare(left_gray, ratio, left_);
  prepare(right_gray, ratio, right_);
  const int width = left_.scaled_quarters.cols;
  const int height = left_.scaled_quarters.rows;
  const int radius = config_.window_size_px / 2;
  const int phase = (ratio - 1) / 2;
  const int extra = ratio % 2 == 0 ? 1 : 0;
  const double samples = config_.window_size_px * config_.window_size_px;
  const double numerator = model.numerator_px_m() * ratio;
  const double principal = model.principal_difference_px() * ratio;
  row_counts_.assign(static_cast<std::size_t>(depth_m.rows), RowCounts{});
  cv::parallel_for_(cv::Range(0, depth_m.rows), [&](const cv::Range &range) {
    struct WindowProduct
    {
      int x = -1;
      int bank = -1;
    };
    std::vector<WindowProduct> products(2 * static_cast<std::size_t>(width) + 1);
    constexpr std::size_t kMaxProductBanks = 32;
    // Bound bank storage to O(image width), even when every coarse disparity differs.
    std::array<std::vector<std::uint32_t>, kMaxProductBanks> banks;
    std::vector<std::uint32_t> vertical(static_cast<std::size_t>(width));
    for (int row = range.start; row < range.end; ++row) {
      std::fill(products.begin(), products.end(), WindowProduct{});
      std::size_t used_banks = 0;
      auto &counts = row_counts_[static_cast<std::size_t>(row)];
      auto *depths = depth_m.ptr<float>(row);
      const int y = row * ratio + phase;
      const auto finite_count = std::count_if(depths, depths + depth_m.cols,
                                              [](float z) { return std::isfinite(z) && z > 0; });
      // Sparse depth rows cannot amortize computing a full row's product bank.
      const bool dense_row = finite_count * config_.window_size_px > width;
      for (int column = 0; column < depth_m.cols; ++column) {
        const float coarse = depths[column];
        if (!std::isfinite(coarse) || coarse <= 0) {
          continue;
        }
        ++counts.attempted;
        const int x = column * ratio + phase;
        if (x - radius < 0 || y - radius < 0 || x + radius + extra >= width ||
            y + radius + extra >= height) {
          continue;
        }
        const double predicted = principal + numerator / coarse;
        if (!std::isfinite(predicted) || std::abs(predicted) > width) {
          continue;
        }
        const int center = cvRound(predicted);
        const double sum_l = left_.window_sum.ptr<double>(y)[x] * 0.25;
        const double square_l = left_.window_square_sum.ptr<double>(y)[x] * 0.0625;
        const double variance_l = square_l - sum_l * sum_l / samples;
        if (variance_l / samples < config_.min_texture_variance) {
          continue;
        }
        std::array<double, 17> costs{};
        costs.fill(std::numeric_limits<double>::infinity());
        const int count = 2 * config_.search_radius_px + 1;
        int best = -1;
        double minimum = std::numeric_limits<double>::infinity();
        for (int candidate = 0; candidate < count; ++candidate) {
          const int disparity = center + candidate - config_.search_radius_px;
          const int right_x = x - disparity;
          if (right_x - radius < 0 || right_x + radius + extra >= width) {
            continue;
          }
          const double sum_r = right_.window_sum.ptr<double>(y)[right_x] * 0.25;
          const double square_r = right_.window_square_sum.ptr<double>(y)[right_x] * 0.0625;
          const double variance_r = square_r - sum_r * sum_r / samples;
          if (variance_r / samples < config_.min_texture_variance) {
            continue;
          }
          const auto index = disparity + static_cast<std::int64_t>(width);
          auto &cached = products[static_cast<std::size_t>(index)];
          if (dense_row && cached.bank < 0 && used_banks < banks.size() && cached.x >= 0 &&
              (x - cached.x) * 2 < config_.window_size_px) {
            const int first = std::max(0, disparity);
            const int end = std::min(width, width + disparity);
            std::fill(vertical.begin() + first, vertical.begin() + end, 0);
            for (int dy = -radius; dy <= radius; ++dy) {
              const auto *left = left_.scaled_quarters.ptr<std::uint16_t>(y + dy);
              const auto *right = right_.scaled_quarters.ptr<std::uint16_t>(y + dy);
              for (int column = first; column < end; ++column) {
                vertical[static_cast<std::size_t>(column)] +=
                    static_cast<std::uint32_t>(left[column]) * right[column - disparity];
              }
            }
            cached.bank = static_cast<int>(used_banks++);
            auto &sums = banks[static_cast<std::size_t>(cached.bank)];
            sums.resize(static_cast<std::size_t>(width));
            std::uint32_t sum = 0;
            for (int column = first; column < first + config_.window_size_px; ++column) {
              sum += vertical[static_cast<std::size_t>(column)];
            }
            sums[static_cast<std::size_t>(first) + static_cast<std::size_t>(radius)] = sum;
            for (int column = first + config_.window_size_px; column < end; ++column) {
              sum -= vertical[static_cast<std::size_t>(column - config_.window_size_px)];
              sum += vertical[static_cast<std::size_t>(column)];
              sums[static_cast<std::size_t>(column - radius)] = sum;
            }
          }
          // The validated 31x31 maximum gives 1020^2 * 961 < 2^32.
          std::uint32_t quarter_product = 0;
          if (cached.bank >= 0) {
            quarter_product =
                banks[static_cast<std::size_t>(cached.bank)][static_cast<std::size_t>(x)];
          } else {
            for (int dy = -radius; dy <= radius; ++dy) {
              const auto *left = left_.scaled_quarters.ptr<std::uint16_t>(y + dy);
              const auto *right = right_.scaled_quarters.ptr<std::uint16_t>(y + dy);
              for (int dx = -radius; dx <= radius; ++dx) {
                quarter_product += static_cast<std::uint32_t>(left[x + dx]) * right[right_x + dx];
              }
            }
          }
          cached.x = x;
          const double product = quarter_product / 16.0;
          const double correlation =
              (product - sum_l * sum_r / samples) / std::sqrt(variance_l * variance_r);
          costs[candidate] = 1 - correlation;
          if (costs[candidate] < minimum) {
            minimum = costs[candidate];
            best = candidate;
          }
        }
        if (best <= 0 || best >= count - 1 || minimum > 1 - config_.min_correlation ||
            !std::isfinite(costs[best - 1]) || !std::isfinite(costs[best + 1])) {
          continue;
        }
        const double curvature = costs[best - 1] - 2 * minimum + costs[best + 1];
        if (curvature <= 1e-6) {
          continue;
        }
        const double offset =
            std::clamp(0.5 * (costs[best - 1] - costs[best + 1]) / curvature, -0.5, 0.5);
        const double disparity = center + best - config_.search_radius_px + offset;
        if (std::abs(disparity - principal) <= 1e-12) {
          continue;
        }
        const double z = numerator / (disparity - principal);
        if (!std::isfinite(z) || z < depth_config.min_depth_m || z > depth_config.max_depth_m ||
            z > std::numeric_limits<float>::max()) {
          continue;
        }
        const auto stored = static_cast<float>(z);
        if (!std::isfinite(stored) || stored <= 0) {
          continue;
        }
        depths[column] = stored;
        ++counts.updated;
      }
    }
  });
  for (const auto &counts : row_counts_) {
    statistics.refinement_attempted_pixels += counts.attempted;
    statistics.refinement_updated_pixels += counts.updated;
  }
  statistics.refinement_retained_pixels =
      statistics.refinement_attempted_pixels - statistics.refinement_updated_pixels;
}
} // namespace intball2_stereo::core
