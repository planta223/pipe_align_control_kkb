#pragma once
#include "xarm_motion/motion_core.hpp"
#include <condition_variable>
#include <chrono>
#include <mutex>
#include <optional>
#include <stdexcept>

namespace xarm_motion::detail {
struct RobotReport {
  Pose tcp = Pose::Identity(), world = Pose::Identity();
  int axis = 0, simulation = -1, mode = -1, brakes = 0, motors = 0;
  double received_at = 0;
  std::string error;
};

// 소켓 연결이 아닌 해석된 rich report의 수신을 기다린다.
class ReportState {
public:
  void publish(RobotReport report) {
    std::lock_guard<std::mutex> guard(mutex_);
    latest_ = std::move(report);
    changed_.notify_all();
  }
  void wait_first(double timeout, const std::function<bool()>& cancelled) {
    const auto until = std::chrono::steady_clock::now()+std::chrono::duration<double>(timeout);
    std::unique_lock<std::mutex> guard(mutex_);
    while (!latest_) {
      if (cancelled()) throw std::runtime_error("실행이 취소되었습니다.");
      if (std::chrono::steady_clock::now() >= until)
        throw std::runtime_error("첫 rich 상태 보고 수신 시간 초과");
      changed_.wait_for(guard,std::chrono::milliseconds(10));
    }
    if (cancelled()) throw std::runtime_error("실행이 취소되었습니다.");
  }
  RobotReport snapshot(double now, double timeout) const {
    std::lock_guard<std::mutex> guard(mutex_);
    if (!latest_) throw std::runtime_error("첫 rich 상태 보고를 아직 받지 못했습니다.");
    if (!latest_->error.empty()) throw std::runtime_error(latest_->error);
    if (!std::isfinite(latest_->received_at) || std::abs(now-latest_->received_at) > timeout)
      throw std::runtime_error("rich 상태 보고 갱신 시간 초과");
    return *latest_;
  }
private:
  mutable std::mutex mutex_;
  std::condition_variable changed_;
  std::optional<RobotReport> latest_;
};
}  // namespace xarm_motion::detail
