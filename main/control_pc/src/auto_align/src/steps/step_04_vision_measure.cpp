#include "auto_align/vision_align.hpp"
#include <Eigen/SVD>
#include <cmath>

namespace auto_align {
namespace {
void require(bool ok, const std::string& message) { if (!ok) throw std::runtime_error(message); }
double positive(const YAML::Node& node,const char* key) {
  const double value=node[key].as<double>();
  require(std::isfinite(value) && value>0,std::string("vision_align.")+key+": 양의 유한한 수치가 필요합니다.");
  return value;
}
}
VisionSettings vision_settings(const YAML::Node& control) {
  const auto n=control["vision_align"]; VisionSettings s{};
  s.topics={n["left_topic"].as<std::string>(),n["right_topic"].as<std::string>()};
  s.frame=n["frame_id"].as<std::string>();
  require(!s.frame.empty() && !s.topics[0].empty() && !s.topics[1].empty() && s.topics[0]!=s.topics[1],"서로 다른 좌우 비전 토픽과 frame이 필요합니다.");
  s.length=positive(n,"pipe_length_m");
  for(size_t i=0;i<2;++i) {
    s.endpoint_z[i]=n[i==0?"left_endpoint_z_m":"right_endpoint_z_m"].as<double>();
    require(std::isfinite(s.endpoint_z[i]) && (std::abs(s.endpoint_z[i])<1e-9 || std::abs(s.endpoint_z[i]-s.length)<1e-9),"끝단 Z는 0 또는 pipe_length_m이어야 합니다.");
  }
#define READ(field,key) s.field=positive(n,key)
  READ(max_age,"max_pose_age_s"); READ(pair_timeout,"pair_timeout_s");
  READ(sample_window,"sample_window_s"); READ(sample_timeout,"sample_timeout_s"); READ(settle,"settle_s");
  READ(angle_tolerance,"angle_tolerance_rad"); READ(relative_angle_tolerance,"relative_angle_tolerance_rad");
  READ(position_tolerance,"position_tolerance_m"); READ(relative_position_tolerance,"relative_position_tolerance_m");
  READ(hold,"success_hold_s"); READ(max_position_noise,"max_position_noise_m"); READ(max_angle_noise,"max_angle_noise_rad");
  READ(probe_translation,"probe_translation_m"); READ(probe_rotation,"probe_rotation_rad");
  READ(max_translation,"max_translation_step_m"); READ(max_rotation,"max_rotation_step_rad");
  READ(linear_speed,"linear_speed_m_s"); READ(angular_speed,"angular_speed_rad_s");
  READ(gain,"gain"); READ(damping,"damping"); READ(max_travel,"max_endpoint_travel_m"); READ(max_turn,"max_axis_turn_rad");
  READ(min_gap,"minimum_gap_m"); READ(gap_drift,"max_gap_change_m"); READ(deadline,"total_timeout_s");
#undef READ
  s.min_samples=n["minimum_samples"].as<int>(); s.max_iterations=n["max_stage_iterations"].as<int>();
  s.success_samples=n["minimum_success_samples"].as<int>();
  s.max_moves=n["max_moves"].as<int>(); s.max_worsening=n["max_worsening"].as<int>();
  require(s.min_samples>=3 && s.min_samples<=100 && s.success_samples>=3 && s.success_samples<=100 &&
    s.max_iterations>0 && s.max_moves>=24 && s.max_worsening>0,"측정 수·반복 상한이 유효하지 않습니다.");
  require(s.gain<=1 && s.sample_timeout>s.sample_window && s.pair_timeout>=.7 && s.max_age>=.5 &&
    s.probe_translation<=.005 && s.probe_rotation<=.035 && s.max_translation<=.005 && s.max_rotation<=.035 &&
    s.angle_tolerance<.2 && s.max_turn<1.0 && s.max_position_noise<s.position_tolerance && s.max_angle_noise<s.angle_tolerance,
    "비전 설정의 시간·이동량·허용값 조합이 유효하지 않습니다.");
  return s;
}
void require_vision_enabled(const YAML::Node& control) {
  const auto n=control["execution"]["vision_align_enabled"];
  require(n && n.IsScalar() && n.Tag()!="!" && n.Tag()!="tag:yaml.org,2002:str" && n.Scalar()=="true",
    "execution.vision_align_enabled=true를 파일에서 설정하세요.");
}
PipeObservation pipe_observation(const xm::Pose& pose,double z) {
  require(std::isfinite(z) && pose.matrix().allFinite() && std::abs(pose.linear().determinant()-1)<1e-6 &&
    (pose.linear().transpose()*pose.linear()-Eigen::Matrix3d::Identity()).norm()<1e-6,"유효한 pipe pose가 필요합니다.");
  return {pose*Eigen::Vector3d(0,0,z),pose.linear().col(2)};
}
void VisionPairs::add(size_t side,int64_t stamp,double received,const PipeObservation& pipe) {
  require(side<2 && stamp>0 && std::isfinite(received) && pipe.endpoint.allFinite() && pipe.axis.allFinite() &&
    std::abs(pipe.axis.norm()-1)<1e-6,"유효한 비전 측정이 필요합니다.");
  if(stamp<=last_stamp_) return;
  pending_[side].emplace(stamp,Value{received,pipe});
  while(pending_[side].size()>128) pending_[side].erase(pending_[side].begin());
}
std::optional<VisionPair> VisionPairs::take() {
  for(auto i=pending_[0].rbegin();i!=pending_[0].rend();++i) {
    const auto j=pending_[1].find(i->first);
    if(j==pending_[1].end()) continue;
    VisionPair pair{{i->second.pipe,j->second.pipe},i->first,std::max(i->second.received,j->second.received)};
    last_stamp_=pair.stamp;
    for(auto& p:pending_) p.erase(p.begin(),p.upper_bound(last_stamp_));
    return pair;
  }
  return std::nullopt;
}
VisionPair average_pairs(const std::vector<VisionPair>& samples,const VisionSettings& s) {
  require(samples.size()>=static_cast<size_t>(s.min_samples),"새 좌우 측정 쌍이 부족합니다.");
  require((samples.back().stamp-samples.front().stamp)*1e-9>=s.sample_window,"측정 시간창이 부족합니다.");
  for(size_t i=1;i<samples.size();++i) require(samples[i].stamp>samples[i-1].stamp,"중복·역순 영상은 평균에 사용할 수 없습니다.");
  VisionPair out=samples.back();
  for(size_t side=0;side<2;++side) {
    Eigen::Vector3d p=Eigen::Vector3d::Zero(),a=Eigen::Vector3d::Zero();
    for(const auto& pair:samples) {
      require(pair.pipes[side].axis.dot(samples.front().pipes[side].axis)>.5,"CAD 축 부호가 바뀌었습니다. 끝단 대응을 다시 확인하세요.");
      p+=pair.pipes[side].endpoint; a+=pair.pipes[side].axis;
    }
    p/=samples.size(); a.normalize();
    double position_variance=0,angle_variance=0;
    for(const auto& pair:samples) {
      position_variance+=(pair.pipes[side].endpoint-p).squaredNorm();
      const double angle=std::acos(std::clamp(a.dot(pair.pipes[side].axis),-1.,1.));
      angle_variance+=angle*angle;
    }
    require(std::sqrt(position_variance/samples.size())<=s.max_position_noise &&
      std::sqrt(angle_variance/samples.size())<=s.max_angle_noise,"정지 상태의 비전 변동이 허용 범위를 넘었습니다.");
    out.pipes[side]={p,a};
  }
  return out;
}
VisionAlign::VisionAlign(VisionSettings settings,VisionIO& io):settings_(std::move(settings)),io_(io) {}
void VisionAlign::initialize() {
  io_.report("STEP4: 좌우 끝단 측정·고정 목표 설정");
  current_=initial_=io_.measure();
  last_stamp_=current_.stamp;
  require(std::abs(initial_.pipes[0].endpoint.x()-initial_.pipes[1].endpoint.x())>settings_.min_gap,"초기 끝단 간격이 부족합니다.");
  const Eigen::Vector3d center=(initial_.pipes[0].endpoint+initial_.pipes[1].endpoint)/2;
  for(size_t i=0;i<2;++i) {
    signs_[i]=initial_.pipes[i].axis.x()>=0?1:-1;
    require(std::abs(initial_.pipes[i].axis.x())>std::cos(.7),"초기 축이 카메라 X축에서 40도 이상 벗어났습니다.");
    const double normal_x=initial_.pipes[i].axis.x()*(settings_.endpoint_z[i]>0?1:-1);
    require(normal_x*(initial_.pipes[1-i].endpoint.x()-initial_.pipes[i].endpoint.x())>0,
      "선택한 CAD 끝단의 바깥 법선이 서로 마주보지 않습니다. endpoint_z 설정을 확인하세요.");
    targets_[i]={initial_.pipes[i].endpoint.x(),center.y(),center.z()};
  }
  check(current_);
}
Features VisionAlign::features(const VisionPair& pair,size_t side) const {
  const auto& p=pair.pipes[side]; const Eigen::Vector3d a=signs_[side]*p.axis;
  Features f; f.head<3>()=p.endpoint;
  f[3]=std::atan2(a.y(),a.x()); f[4]=std::atan2(a.z(),a.x()); return f;
}
void VisionAlign::check(const VisionPair& pair) const {
  require(io_.elapsed()<=settings_.deadline,"UVS 전체 시간 상한을 넘었습니다.");
  const double initial_dx=initial_.pipes[1].endpoint.x()-initial_.pipes[0].endpoint.x();
  const double dx=pair.pipes[1].endpoint.x()-pair.pipes[0].endpoint.x();
  require(dx*initial_dx>0 && std::abs(dx)>=settings_.min_gap && std::abs(dx-initial_dx)<=settings_.gap_drift,
    "끝단 간격·순서가 허용 범위를 벗어났습니다.");
  for(size_t i=0;i<2;++i) {
    require((pair.pipes[i].endpoint-initial_.pipes[i].endpoint).norm()<=settings_.max_travel,"끝단 누적 이동 범위를 넘었습니다.");
    require(pair.pipes[i].axis.dot(initial_.pipes[i].axis)>=std::cos(settings_.max_turn),"축의 누적 회전 범위를 넘었습니다.");
  }
}
VisionPair VisionAlign::measure() {
  auto pair=io_.measure();
  require(pair.stamp>last_stamp_,"이전 영상으로 새 보정을 검증할 수 없습니다.");
  check(pair);last_stamp_=pair.stamp;return pair;
}
VisionPair VisionAlign::move(size_t side,const Eigen::Vector3d& delta,bool rotation,Eigen::Vector3d* actual) {
  require(++moves_<=settings_.max_moves,"UVS 이동 횟수 상한을 넘었습니다.");
  const auto measured=io_.move(side,delta,rotation);
  require(measured.allFinite() && measured.norm()>1e-7,"실측 보정 이동량이 부족합니다.");
  if(actual) *actual=measured;
  return measure();
}
} // namespace auto_align
