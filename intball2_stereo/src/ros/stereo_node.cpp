#include "intball2_stereo/ros/stereo_node.hpp"
#include "intball2_stereo/core/pointcloud_generator.hpp"
#include "intball2_stereo/ros/stereo_timing.hpp"
#include <chrono>
#include <stdexcept>
namespace intball2_stereo::ros
{
StereoNode::StereoNode(const rclcpp::NodeOptions &options) : Node("stereo_node", options)
{
  const auto config = StereoParameters::declare_and_read(*this);
  for (const auto &spec : StereoParameters::specifications()) {
    parameter_names_.push_back(spec.name);
  }
  active_configuration_ = std::make_unique<StereoFrameConfiguration>(
      ParameterSnapshot{get_parameters(parameter_names_)});
  parameter_callback_ =
      add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &updates) {
        rcl_interfaces::msg::SetParametersResult result;
        try {
          StereoParameters::validate_updates(ParameterSnapshot{get_parameters(parameter_names_)},
                                             updates);
          result.successful = true;
        } catch (const std::exception &error) {
          result.successful = false;
          result.reason = error.what();
        }
        return result;
      });
  diagnostics_.config_revision = 1;
  cv::setNumThreads(
      config.core.processing.opencv_threads == 1 ? 0 : config.core.processing.opencv_threads);
  processor_ = std::make_unique<core::StereoProcessor>(config.core);
  auto qos = rclcpp::SensorDataQoS();
  qos.keep_last(static_cast<std::size_t>(config.qos.depth));
  depth_publisher_ = create_publisher<sensor_msgs::msg::Image>(config.topics.depth, qos);
  info_publisher_ = create_publisher<sensor_msgs::msg::CameraInfo>(config.topics.camera_info, qos);
  points_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(config.topics.points, qos);
  diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      config.topics.diagnostics, rclcpp::QoS(config.qos.depth));
  timing_publisher_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(config.topics.timing, qos);
  worker_ = std::make_unique<StereoWorker>(
      [this](const InputBundle &bundle) {
        SimulationTiming timing;
        timing.use_sim_time = get_parameter("use_sim_time").as_bool();
        timing.worker_start_ns = get_clock()->now().nanoseconds();
        timing.source_stamp_ns = stamp_nanoseconds(bundle.left_image->header.stamp);
        timing.previous_publish_start_ns = previous_timing_publish_ns_;
        timing.input_source_stamp_ns = bundle.source_stamp_ns;
        timing.callback_arrival_sim_ns = bundle.callback_arrival_sim_ns;
        timing.bundle_ready_sim_ns = bundle.bundle_ready_sim_ns;
        timing.input_clock_rewound = bundle.simulation_clock_rewound;
        // Read accepted parameters together; validation callbacks never mutate worker state.
        ParameterSnapshot parameters{get_parameters(parameter_names_)};
        if (!active_configuration_->matches(parameters)) {
          auto configuration = std::make_unique<StereoFrameConfiguration>(std::move(parameters));
          auto replacement = std::make_unique<core::StereoProcessor>(configuration->config().core);
          processor_ = std::move(replacement);
          active_configuration_ = std::move(configuration);
          std::lock_guard<std::mutex> lock(diagnostics_mutex_);
          ++diagnostics_.config_revision;
        }
        const auto &frame_config = active_configuration_->config();
        const auto pair = image_pair(bundle, frame_config.core.camera.calibration_mode);
        timing.processing_stages_recorded = true;
        const auto &result =
            processor_->process(pair, [this, &timing](core::ProcessingStage stage) {
              timing.processing_stage_sim_ns[static_cast<std::size_t>(stage)] =
                  get_clock()->now().nanoseconds();
            });
        auto statistics = result.statistics;
        statistics.synchronization_wait_s = bundle.synchronization_wait_s;
        auto output = output_messages(result);
        timing.depth_ready_ns = get_clock()->now().nanoseconds();
        std::optional<sensor_msgs::msg::PointCloud2> points;
        if (frame_config.core.points.publish_points) {
          const auto start = std::chrono::steady_clock::now();
          points = pointcloud_message(
              core::PointCloudGenerator::generate(result, frame_config.core.points));
          statistics.pointcloud_generation_s =
              std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
        }
        timing.use_sim_time = timing.use_sim_time && get_parameter("use_sim_time").as_bool();
        timing.publish_start_ns = get_clock()->now().nanoseconds();
        depth_publisher_->publish(output.depth);
        timing_publisher_->publish(simulation_timing_message(timing, output.depth.header));
        previous_timing_publish_ns_ = timing.use_sim_time ? timing.publish_start_ns : 0;
        info_publisher_->publish(output.camera_info);
        if (points) {
          points_publisher_->publish(*points);
        }
        {
          std::lock_guard<std::mutex> lock(diagnostics_mutex_);
          diagnostics_.statistics = statistics;
          ++diagnostics_.processed_pairs;
          diagnostics_.last_error.clear();
        }
      },
      [this](std::exception_ptr error) {
        std::string reason;
        try {
          std::rethrow_exception(std::move(error));
        } catch (const std::exception &e) {
          reason = e.what();
        } catch (...) {
          reason = "unknown exception";
        }
        RCLCPP_ERROR(get_logger(), "Stereo processing failed: %s", reason.c_str());
        std::lock_guard<std::mutex> lock(diagnostics_mutex_);
        ++diagnostics_.failed_pairs;
        diagnostics_.last_error = std::move(reason);
      });
  input_ = std::make_unique<StereoInput>(
      config.sync, [this](InputBundle bundle) { worker_->submit(std::move(bundle)); },
      [this] {
        return get_parameter("use_sim_time").as_bool() ? get_clock()->now().nanoseconds() : 0;
      });
  const std::array<std::string, 2> image_topics{config.topics.left_image,
                                                config.topics.right_image};
  const std::array<std::string, 2> info_topics{config.topics.left_camera_info,
                                               config.topics.right_camera_info};
  for (std::size_t i = 0; i < 2; ++i) {
    images_[i] = create_subscription<sensor_msgs::msg::Image>(
        image_topics[i], qos, [this, i](sensor_msgs::msg::Image::ConstSharedPtr image) {
          try {
            input_->add_image(i == 1, std::move(image));
          } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "%s", e.what());
          }
        });
    infos_[i] = create_subscription<sensor_msgs::msg::CameraInfo>(
        info_topics[i], qos, [this, i](sensor_msgs::msg::CameraInfo::ConstSharedPtr info) {
          try {
            input_->add_info(i == 1, std::move(info));
          } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "%s", e.what());
          }
        });
  }
  diagnostics_timer_ =
      create_wall_timer(std::chrono::duration<double>(config.diagnostics.publish_interval_s),
                        [this] { publish_diagnostics(); });
}
void StereoNode::publish_diagnostics()
{
  DiagnosticSnapshot snapshot;
  {
    std::lock_guard<std::mutex> lock(diagnostics_mutex_);
    snapshot = diagnostics_;
  }
  snapshot.synchronization = input_->statistics();
  snapshot.worker_dropped_pairs = worker_->dropped_pairs();
  diagnostics_publisher_->publish(diagnostic_messages(snapshot, now()));
}
StereoNode::~StereoNode()
{
  if (worker_) {
    worker_->stop();
  }
}
} // namespace intball2_stereo::ros
