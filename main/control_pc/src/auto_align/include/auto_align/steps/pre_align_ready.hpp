#pragma once
#include "auto_align/collision_checker.hpp"

namespace auto_align {
namespace pre_align_ready {
// 01·02는 world 기준, 03은 저장된 팔 base 기준 목표를 사용한다.
xm::Pose vertical_move_target_world(const xm::Pose& current_world, const xm::Pose& target_world);
xm::Pose horizontal_move_target_world(const xm::Pose& vertical_world, const xm::Pose& target_world);
xm::Pose orientation_align_target_base(const xm::Pose& target_base);
}  // namespace pre_align_ready

void require_execution_enabled(const YAML::Node& control);
CollisionSettings collision_settings(const YAML::Node& control);
xm::Pose mounting_transform(const YAML::Node& frames, const std::string& side);
std::array<xm::Pose,3> ready_waypoints(const xm::Pose& current,
                                    const xm::Pose& target, const xm::Pose& world_T_base);

// 세 구간을 미리 계산하고 각 구간의 시간을 양팔에 공통 적용한다.
class PreAlignReady {
public:
  PreAlignReady(std::vector<xm::MotionArm>& arms, const std::array<xm::Pose,2>& mounts,
                const xm::Limits& limits, const std::function<bool()>& cancelled);
  const std::array<double,3>& phase_durations() const { return durations_; }
  std::string phase(double elapsed) const;
private:
  std::array<double,3> durations_{};
};
}  // namespace auto_align
