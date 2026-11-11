#include "auto_align/steps/pre_align_ready.hpp"

namespace auto_align::pre_align_ready {
xm::Pose horizontal_move_target_world(const xm::Pose& vertical_world, const xm::Pose& target_world) {
  // 01의 방향과 목표 높이를 유지하고 목표 X·Y로 이동한다.
  xm::Pose horizontal = vertical_world;
  horizontal.translation() = target_world.translation();
  return horizontal;
}
}  // namespace auto_align::pre_align_ready
