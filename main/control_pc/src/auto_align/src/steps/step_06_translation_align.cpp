#include "auto_align/vision_align.hpp"

namespace auto_align {
bool VisionAlign::positions_ok(const VisionPair& pair) const {
  for(size_t side=0;side<2;++side)
    if((pair.pipes[side].endpoint-targets_[side]).norm()>settings_.position_tolerance) return false;
  return (pair.pipes[0].endpoint-pair.pipes[1].endpoint).tail<2>().norm()<=settings_.relative_position_tolerance;
}
bool VisionAlign::translation_stage() {
  io_.report("STEP6: 끝단 Y/Z 정렬·X 간격 유지 보정·검증");
  for(int iteration=0;iteration<settings_.max_iterations;++iteration) {
    if(!angles_ok(current_)) return false;
    if(positions_ok(current_)) return true;
    const auto stamp=current_.stamp;
    for(size_t side=0;side<2;++side) {
      if((features(current_,side).head<3>()-targets_[side]).norm()>settings_.position_tolerance*.4)
        correct(side,false);
      if(!angles_ok(current_)) return false;
    }
    if(current_.stamp==stamp) current_=measure();
  }
  throw std::runtime_error("step6 병진 반복 상한을 넘었습니다.");
}
} // namespace auto_align
