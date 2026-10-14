#ifndef INTBALL2_STEREO_ROS_STEREO_WORKER_HPP_
#define INTBALL2_STEREO_ROS_STEREO_WORKER_HPP_
#include "intball2_stereo/ros/stereo_messages.hpp"
#include <condition_variable>
#include <exception>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>
namespace intball2_stereo::ros
{
// One active job plus at most one waiting job. Both callbacks run only on the worker.
class StereoWorker
{
public:
  using Process = std::function<void(const InputBundle &)>;
  using OnError = std::function<void(std::exception_ptr)>;
  StereoWorker(Process process, OnError on_error);
  ~StereoWorker();
  StereoWorker(const StereoWorker &) = delete;
  StereoWorker &operator=(const StereoWorker &) = delete;
  bool submit(InputBundle input);
  // Call from the owner thread, never from a worker callback. Discards pending work.
  void stop();
  std::uint64_t dropped_pairs() const;

private:
  void run();
  Process process_;
  OnError on_error_;
  mutable std::mutex mutex_;
  std::condition_variable ready_;
  std::optional<InputBundle> pending_;
  bool stopping_ = false;
  std::uint64_t dropped_pairs_ = 0;
  std::thread thread_;
};
} // namespace intball2_stereo::ros
#endif
