#include "auto_align/vision_align.hpp"
#include <chrono>
#include <sstream>

namespace auto_align {
void VisionAlign::save_verified() {
  for(size_t side=0;side<2;++side) for(size_t kind=0;kind<2;++kind) {
    const auto& n=counts_[side][kind];
    std::ostringstream out;
    out<<"VISION_MOVES: "<<(side==0?"left":"right")<<(kind==1?" rotation":" translation")
      <<" learning="<<n.learning<<" correction="<<n.correction<<" validation="<<n.validation
      <<" reused="<<n.reused<<" rejected="<<n.rejected<<" total="<<moves_
      <<" / validation is a subset of correction";
    io_.report(out.str());
  }
  if(!cache_ || !settings_.cache_enabled) return;
  auto entries=cache_->entries();
  bool updated=false;
  const double now=std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
  for(size_t side=0;side<2;++side) for(size_t kind=0;kind<2;++kind)
    if(learned_[side][kind] && (!cached_[side][kind] || counts_[side][kind].reused>0)) {
      const auto tcp=io_.tcp_pose(side);
      if(tcp) {
        entries[side][kind]=VisionCacheEntry{jacobians_[side][kind],current_.pipes[side],*tcp,now,
          current_.position_error[side],current_.angle_error[side]};
        updated=true;
      }
    }
  if(!updated) return;
  // Only persistence errors are nonfatal. SDK reads and safety/vision faults above
  // must still stop the run; interrupted runs never reach this commit point.
  try { cache_->save(entries);io_.report("CACHE: saved / step7 통과한 학습값만 원자적으로 저장"); }
  catch(const std::exception& e) { io_.report(std::string("CACHE: save_failed / ")+e.what()); }
}
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
        save_verified();
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
