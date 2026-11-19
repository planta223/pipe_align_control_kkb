#include "auto_align/vision_align.hpp"

namespace auto_align {
void VisionAlign::run() {
  initialize();
  for(int cycle=0;cycle<settings_.max_iterations;++cycle) {
    rotation_stage();
    if(!translation_stage()) continue;
    io_.report("STEP7: 양측 각도·끝단·간격의 시간 기반 통합 검증");
    const auto first=current_.stamp;
    int samples=1;
    while(angles_ok(current_) && positions_ok(current_)) {
      if(samples>=settings_.success_samples && (current_.stamp-first)*1e-9>=settings_.hold) {
        io_.report("VERIFIED: step4~7 비전 정렬 조건 유지 확인"); return;
      }
      const auto previous=current_.stamp;
      current_=measure();
      if(current_.stamp<=previous) throw std::runtime_error("새 영상 없이 성공 유지 시간을 늘릴 수 없습니다.");
      ++samples;
    }
  }
  throw std::runtime_error("step7 전체 재검증 횟수를 넘었습니다.");
}
} // namespace auto_align
