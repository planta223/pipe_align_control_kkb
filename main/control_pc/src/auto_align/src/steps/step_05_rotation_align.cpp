#include "auto_align/vision_align.hpp"
#include <Eigen/SVD>
#include <Eigen/Cholesky>
#include <sstream>

namespace auto_align {
namespace {
void require(bool ok,const std::string& text) { if(!ok) throw std::runtime_error(text); }
}
Eigen::Vector3d visual_solve(const Eigen::MatrixXd& j,const Eigen::VectorXd& error,
                            double gain,double damping,double maximum) {
  require(j.cols()==3 && j.rows()==error.size() && j.rows()>=2 && j.rows()<=3 &&
    j.allFinite() && error.allFinite() && gain>0 && damping>0 && maximum>0,"유효한 UVS 선형계가 필요합니다.");
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(j);
  require(svd.singularValues().minCoeff()>.1 && svd.singularValues().maxCoeff()/svd.singularValues().minCoeff()<30,
    "UVS 대응 관계가 불충분하거나 특이합니다. 시험 이동·비전 품질을 확인하세요.");
  Eigen::MatrixXd normal=j*j.transpose()+damping*damping*Eigen::MatrixXd::Identity(j.rows(),j.rows());
  Eigen::Vector3d delta=-gain*j.transpose()*normal.ldlt().solve(error);
  require(delta.allFinite(),"UVS 보정량이 유효하지 않습니다.");
  if(delta.norm()>maximum) delta*=maximum/delta.norm();
  return delta;
}
void VisionAlign::probe(size_t side,bool rotation,const Eigen::Vector3d& request,const std::string& label) {
  const auto kind=rotation?1:0;
  require(counts_[side][kind].learning<(rotation?4:3),"학습 이동 예산을 소진했습니다. 회전은 팔당 최대 4회입니다.");
  const auto before=current_;Eigen::Vector3d actual;
  std::ostringstream out;
  out<<"PROBE: "<<(side==0?"left":"right")<<(rotation?" rotation":" translation")<<" axis="<<label
     <<" requested=["<<request.transpose()<<"] norm="<<request.norm()<<" / 복귀 없는 연속 학습";
  io_.report(out.str());++counts_[side][kind].learning;
  current_=move(side,request,rotation,&actual);
  const Features change=features(current_,side)-features(before,side);
  probes_[side][kind].push_back({actual,change,
    std::max(settings_.validation_position_floor,std::hypot(before.position_error[side],current_.position_error[side])),
    std::max(settings_.validation_angle_floor,std::hypot(before.angle_error[side],current_.angle_error[side]))});
  std::ostringstream measured;
  measured<<"UVS_PROBE_RESPONSE: "<<(side==0?"left":"right")<<(rotation?" rotation":" translation")
    <<" actual=["<<actual.transpose()<<"] observed=["<<change.transpose()<<"] position_noise_m="
    <<probes_[side][kind].back().position_noise<<" angle_noise_rad="<<probes_[side][kind].back().angle_noise;
  io_.report(measured.str());
}
bool VisionAlign::fit_probes(size_t side,bool rotation) {
  const auto kind=rotation?1:0;const auto& samples=probes_[side][kind];
  const int n=static_cast<int>(samples.size()),rows=rotation?2:3;
  Eigen::MatrixXd commands(3,n),weighted_commands(n,3),weighted_responses(n,5),signal(rows,n);
  for(int i=0;i<n;++i) {
    const auto& sample=samples[i];const double noise=rotation?sample.angle_noise:sample.position_noise;
    commands.col(i)=sample.actual;weighted_commands.row(i)=sample.actual.transpose()/noise;
    weighted_responses.row(i)=sample.change.transpose()/noise;
    signal.col(i)=rotation?Eigen::VectorXd(sample.change.tail<2>()/noise):Eigen::VectorXd(sample.change.head<3>()/noise);
  }
  const Eigen::JacobiSVD<Eigen::MatrixXd> input_svd(commands);
  require(input_svd.singularValues().minCoeff()>.4*(rotation?settings_.probe_rotation:settings_.probe_translation),
    "초기 시험 이동의 실제 변화가 부족합니다.");
  const Eigen::JacobiSVD<Eigen::MatrixXd> fit(weighted_commands,Eigen::ComputeThinU|Eigen::ComputeThinV);
  auto& j=jacobians_[side][kind];j=fit.solve(weighted_responses).transpose();
  const Eigen::JacobiSVD<Eigen::MatrixXd> information(signal);
  const double minimum=information.singularValues().minCoeff();
  bool usable=minimum>=settings_.minimum_response_snr && j.allFinite();
  try {
    if(rotation) visual_solve(j.bottomRows<2>(),Eigen::Vector2d::Zero(),1,settings_.damping,settings_.max_rotation);
    else visual_solve(j.topRows<3>(),Eigen::Vector3d::Zero(),1,settings_.damping,settings_.max_translation);
  } catch(const std::runtime_error&) { usable=false; }
  for(const auto& sample:samples) {
    const Features residual=sample.change-j*sample.actual;
    const double error=rotation?residual.tail<2>().norm():residual.head<3>().norm();
    const double magnitude=rotation?sample.change.tail<2>().norm():sample.change.head<3>().norm();
    usable &= error<=std::max(3*(rotation?sample.angle_noise:sample.position_noise),.35*magnitude);
  }
  std::ostringstream report;
  const Eigen::MatrixXd task=rotation?Eigen::MatrixXd(j.bottomRows<2>()):Eigen::MatrixXd(j.topRows<3>());
  const Eigen::JacobiSVD<Eigen::MatrixXd> task_svd(task);
  report<<"UVS_LEARN_QUALITY: "<<(side==0?"left":"right")<<(rotation?" rotation":" translation")
    <<" probes="<<n<<" task_dimensions="<<rows<<" minimum_signal="<<minimum
    <<" required="<<settings_.minimum_response_snr<<" input_sigma_min="<<input_svd.singularValues().minCoeff()
    <<" task_sigma_min="<<task_svd.singularValues().minCoeff()<<" result="<<(usable?"accepted":"insufficient")
    <<" jacobian_row_major=[";
  for(int row=0;row<5;++row) for(int col=0;col<3;++col) report<<j(row,col)<<' ';
  report<<']';
  io_.report(report.str());return usable;
}
void VisionAlign::retry_rotation(size_t side) {
  require(counts_[side][1].learning==3,"추가 회전 시험 예산을 소진했습니다. 팔당 1회만 허용합니다.");
  const Eigen::MatrixXd task=jacobians_[side][1].bottomRows<2>();
  require(task.allFinite(),"유효한 시험 대응 관계가 없습니다.");
  Eigen::MatrixXd signal(2,probes_[side][1].size());
  for(size_t i=0;i<probes_[side][1].size();++i)
    signal.col(i)=probes_[side][1][i].change.tail<2>()/probes_[side][1][i].angle_noise;
  const Eigen::JacobiSVD<Eigen::MatrixXd> information(signal,Eigen::ComputeFullU);
  const Eigen::JacobiSVD<Eigen::MatrixXd> inverse(task,Eigen::ComputeThinU|Eigen::ComputeThinV);
  Eigen::Vector3d direction=inverse.solve(information.matrixU().col(1));
  // Excite the least certain task direction, not the unobservable pipe roll.
  // A rank-deficient provisional map needs a bounded exploratory fallback.
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(task,Eigen::ComputeFullV);
  if(!direction.allFinite() || direction.norm()<1e-8) direction=svd.matrixV().col(1);
  probe(side,true,settings_.retry_rotation*direction.normalized(),"adaptive");
}
void VisionAlign::learn(size_t side,bool rotation) {
  const auto kind=rotation?1:0;
  require(probes_[side][kind].empty(),"기존 시험이 있습니다. 추가 시험 예산 안에서만 재추정하세요.");
  io_.report(std::string(rotation?"STEP5":"STEP6")+": "+(side==0?"left":"right")+" UVS 시험 이동·대응 관계 추정");
  cached_[side][kind]=false;learned_[side][kind]=false;
  const double amount=rotation?settings_.probe_rotation:settings_.probe_translation;
  for(int axis=0;axis<3;++axis) probe(side,rotation,amount*Eigen::Vector3d::Unit(axis),std::to_string(axis));
  bool usable=fit_probes(side,rotation);
  if(!usable && rotation) {retry_rotation(side);usable=fit_probes(side,rotation);}
  require(usable,"신규 학습의 관측 정보가 부족합니다. 시험 한도 안에서 신뢰도를 확보하지 못했습니다.");
  learned_[side][kind]=true;
}
void VisionAlign::refresh(size_t side,bool rotation) {
  const auto kind=rotation?1:0;
  cached_[side][kind]=false;learned_[side][kind]=false;
  if(probes_[side][kind].empty()) learn(side,rotation);
  else {
    require(rotation,"병진 응답 검증 실패: 신규 학습 예산을 소진했습니다.");
    retry_rotation(side);
    require(fit_probes(side,true),"추가 시험 후에도 회전 학습 품질이 부족합니다.");
    learned_[side][kind]=true;
  }
  untrusted_[side][kind]=0;stagnant_[side][kind]=0;
}
bool VisionAlign::reuse(size_t side,bool rotation) {
  const auto kind=rotation?1:0;
  if(!cache_ || !settings_.cache_enabled || !cache_->entries()[side][kind]) return false;
  const auto& saved=*cache_->entries()[side][kind];
  const auto tcp=io_.tcp_pose(side);
  const auto& pipe=current_.pipes[side];
  if(!tcp || (pipe.endpoint-saved.reference.endpoint).norm()>settings_.cache_position_radius ||
    pipe.axis.dot(saved.reference.axis)<std::cos(settings_.cache_rotation_radius) ||
    (tcp->translation()-saved.tcp.translation()).norm()>settings_.cache_position_radius ||
    xm::rotation_error(*tcp,saved.tcp)>settings_.cache_rotation_radius) {
    io_.report(std::string("CACHE: pose_mismatch ")+(side==0?"left":"right")+(rotation?" rotation":" translation"));
    return false;
  }
  jacobians_[side][kind]=saved.jacobian;learned_[side][kind]=true;cached_[side][kind]=true;
  io_.report(std::string("CACHE: candidate ")+(side==0?"left":"right")+(rotation?" rotation":" translation")+
    " / 다음 보정 방향의 실제 응답 검증 필요");
  return true;
}
ResponseStatus VisionAlign::response_status(size_t side,bool rotation,const VisionPair& before,
                             const Features& predicted,const Features& observed) const {
  const double position_noise=std::max(settings_.validation_position_floor,
    std::hypot(before.position_error[side],current_.position_error[side]));
  const double angle_noise=std::max(settings_.validation_angle_floor,
    std::hypot(before.angle_error[side],current_.angle_error[side]));
  const auto agrees=[this](const Eigen::VectorXd& p,const Eigen::VectorXd& o,double noise) {
    const double pn=p.norm(),on=o.norm();
    return pn>=settings_.minimum_response_snr*noise && on>=settings_.minimum_response_snr*noise && on>=.5*pn && on<=1.5*pn &&
      p.dot(o)>=.8*pn*on && (p-o).norm()<=std::max(.35*pn,3*noise);
  };
  // Judge the active task only. Coupled endpoint/axis changes remain logged,
  // bounded by check(), and verified on stage transitions. A one-off angular
  // disturbance during translation must not become a mandatory angular model.
  if(rotation) {
    if(std::min(predicted.tail<2>().norm(),observed.tail<2>().norm())<settings_.minimum_response_snr*angle_noise) return ResponseStatus::LowSignal;
    return agrees(predicted.tail<2>(),observed.tail<2>(),angle_noise)?ResponseStatus::Accepted:ResponseStatus::Mismatch;
  }
  if(std::min(predicted.head<3>().norm(),observed.head<3>().norm())<settings_.minimum_response_snr*position_noise) return ResponseStatus::LowSignal;
  return agrees(predicted.head<3>(),observed.head<3>(),position_noise)?ResponseStatus::Accepted:ResponseStatus::Mismatch;
}
bool VisionAlign::angles_ok(const VisionPair& pair) const {
  for(size_t i=0;i<2;++i)
    if(std::acos(std::clamp(std::abs(pair.pipes[i].axis.x()),0.,1.))>settings_.angle_tolerance) return false;
  return std::acos(std::clamp(std::abs(pair.pipes[0].axis.dot(pair.pipes[1].axis)),0.,1.))<=settings_.relative_angle_tolerance;
}
void VisionAlign::correct(size_t side,bool rotation) {
  const auto kind=rotation?1:0;
  if(!learned_[side][kind] && !reuse(side,rotation)) learn(side,rotation);
  auto& j=jacobians_[side][rotation?1:0];
  const auto before=current_; const Features f=features(before,side);
  Eigen::Vector3d delta;
  double old_error;
  if(rotation) {
    old_error=f.tail<2>().norm();
    delta=visual_solve(j.bottomRows<2>(),f.tail<2>(),settings_.gain,settings_.damping,settings_.max_rotation);
  } else {
    old_error=(f.head<3>()-targets_[side]).norm();
    delta=visual_solve(j.topRows<3>(),f.head<3>()-targets_[side],settings_.gain,settings_.damping,settings_.max_translation);
  }
  if(delta.norm()<1e-7) return;
  Eigen::Vector3d actual;
  current_=move(side,delta,rotation,&actual);
  ++counts_[side][kind].correction;
  const Features change=features(current_,side)-f;
  const Features predicted=j*actual;
  const auto status=response_status(side,rotation,before,predicted,change);
  ++counts_[side][kind].validation;
  const bool accepted=status==ResponseStatus::Accepted;
  const char* result=accepted?"accepted_direction":status==ResponseStatus::LowSignal?"low_signal":"mismatch";
  std::ostringstream validation;
  validation<<"UVS_RESPONSE: "<<(side==0?"left":"right")<<(rotation?" rotation":" translation")
    <<" source="<<(cached_[side][kind]?"cache":"learned")<<" result="<<result
    <<" requested=["<<delta.transpose()<<"] actual=["<<actual.transpose()<<"] predicted=["<<predicted.transpose()
    <<"] observed=["<<change.transpose()<<"] position_noise_m="
    <<std::max(settings_.validation_position_floor,std::hypot(before.position_error[side],current_.position_error[side]))
    <<" angle_noise_rad="<<std::max(settings_.validation_angle_floor,std::hypot(before.angle_error[side],current_.angle_error[side]));
  io_.report(validation.str());
  if(cached_[side][kind]) {
    io_.report(std::string("CACHE_VALIDATE: ")+(side==0?"left":"right")+(rotation?" rotation":" translation")+" result="+result);
    if(accepted && counts_[side][kind].reused==0) ++counts_[side][kind].reused;
  }
  if(!accepted) ++counts_[side][kind].rejected;
  if(status==ResponseStatus::Mismatch) { refresh(side,rotation);return; }
  const double next_error=rotation?features(current_,side).tail<2>().norm():
    (features(current_,side).head<3>()-targets_[side]).norm();
  const double noise=rotation?settings_.max_angle_noise:settings_.max_position_noise;
  auto& worsening=worsening_[side][rotation?1:0];
  if(next_error>old_error+noise) ++worsening;
  else if(next_error<old_error) worsening=0;
  require(worsening<settings_.max_worsening,"보정 오차가 반복해서 악화됐습니다.");
  const double uncertainty=rotation?
    std::max(settings_.validation_angle_floor,std::hypot(before.angle_error[side],current_.angle_error[side])):
    std::max(settings_.validation_position_floor,std::hypot(before.position_error[side],current_.position_error[side]));
  if(old_error-next_error>uncertainty) stagnant_[side][kind]=0;
  else ++stagnant_[side][kind];
  if(status==ResponseStatus::LowSignal) ++untrusted_[side][kind];
  else untrusted_[side][kind]=0;
  if(!(rotation?angles_ok(current_):positions_ok(current_)) &&
    (untrusted_[side][kind]>=settings_.max_untrusted_responses || stagnant_[side][kind]>=settings_.max_untrusted_responses)) {
    io_.report("UVS: 응답 신뢰도·개선량 정체 / 남은 시험 예산 안에서 재추정");
    refresh(side,rotation);return;
  }
  // Broyden update with a small blend; both endpoint and axis coupling are retained.
  if(accepted && actual.norm()>(rotation?settings_.max_rotation:settings_.max_translation)*.2) {
    const Features residual=change-j*actual;
    const VisualJacobian candidate=j+.2*residual*actual.transpose()/actual.squaredNorm();
    try {
      if(rotation) visual_solve(candidate.bottomRows<2>(),Eigen::Vector2d::Zero(),1,settings_.damping,settings_.max_rotation);
      else visual_solve(candidate.topRows<3>(),Eigen::Vector3d::Zero(),1,settings_.damping,settings_.max_translation);
      if(candidate.allFinite()) j=candidate;
    } catch(const std::runtime_error&) { io_.report("UVS: 특이한 온라인 갱신을 제외하고 직전 대응 관계 유지"); }
  } else if(!accepted) io_.report("UVS: 신호 부족으로 온라인 행렬 갱신 보류");
  std::ostringstream out;
  out<<(rotation?"STEP5":"STEP6")<<": "<<(side==0?"left":"right")
     <<" error "<<old_error<<" -> "<<next_error<<(rotation?" rad":" m")<<" / moves="<<moves_;
  io_.report(out.str());
}
void VisionAlign::rotation_stage() {
  io_.report("STEP5: 카메라 X축 평행 회전 보정·검증");
  for(int iteration=0;iteration<settings_.max_iterations;++iteration) {
    if(angles_ok(current_)) return;
    const auto stamp=current_.stamp;
    for(size_t side=0;side<2;++side) {
      // Also reduce residuals below the individual tolerance when the relative
      // angle is the failing condition (opposite residuals can add up).
      if(features(current_,side).tail<2>().norm()>settings_.angle_tolerance*.4)
        correct(side,true);
    }
    // Every move already ends with a fresh settled pair. Only collect another
    // window if no move produced a new measurement during this iteration.
    if(current_.stamp==stamp) current_=measure();
  }
  throw std::runtime_error("step5 회전 반복 상한을 넘었습니다.");
}
} // namespace auto_align
