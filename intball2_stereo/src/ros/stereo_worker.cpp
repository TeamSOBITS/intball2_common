#include "intball2_stereo/ros/stereo_worker.hpp"
#include <stdexcept>
#include <utility>
namespace intball2_stereo::ros
{
StereoWorker::StereoWorker(Process process, OnError on_error)
    : process_(std::move(process)), on_error_(std::move(on_error))
{
  if (!process_ || !on_error_) {
    throw std::invalid_argument("worker requires process and error callbacks");
  }
  thread_ = std::thread(&StereoWorker::run, this);
}
StereoWorker::~StereoWorker()
{
  stop();
}
bool StereoWorker::submit(InputBundle input)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (stopping_) {
    return false;
  }
  if (pending_) {
    ++dropped_pairs_;
  }
  pending_ = std::move(input);
  ready_.notify_one();
  return true;
}
void StereoWorker::stop()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
    if (pending_) {
      ++dropped_pairs_;
      pending_.reset();
    }
  }
  ready_.notify_one();
  if (thread_.joinable()) {
    thread_.join();
  }
}
std::uint64_t StereoWorker::dropped_pairs() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return dropped_pairs_;
}
void StereoWorker::run()
{
  for (;;) {
    std::optional<InputBundle> input;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      ready_.wait(lock, [this] { return stopping_ || pending_.has_value(); });
      if (stopping_) {
        return;
      }
      input.swap(pending_);
    }
    try {
      process_(*input);
    } catch (...) {
      try {
        on_error_(std::current_exception());
      } catch (...) {
        // A reporting failure must not kill the worker or terminate the process.
      }
    }
  }
}
} // namespace intball2_stereo::ros
