#include "auto_align/steps/pre_align_ready.hpp"

namespace auto_align::pre_align_ready {
xm::Pose orientation_align_target_base(const xm::Pose& target_base) {
  // 02에서 목표 위치에 도달했으므로 저장된 목표 방향까지 정렬한다.
  // 저장된 base 자세를 그대로 반환해 기존 좌표 변환의 수치도 유지한다.
  return target_base;
}
}  // namespace auto_align::pre_align_ready
