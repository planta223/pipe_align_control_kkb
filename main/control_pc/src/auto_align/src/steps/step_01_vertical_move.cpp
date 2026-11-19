#include "auto_align/steps/pre_align_ready.hpp"
#include <atomic>
#include <future>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <sstream>

namespace auto_align {
namespace {
void require(bool condition, const std::string& text) {
  if (!condition) throw std::runtime_error(text);
}
double positive(const YAML::Node& node, const std::string& key) {
  const double value = node[key].as<double>();
  require(std::isfinite(value) && value > 0,key+": 양의 유한한 수치가 필요합니다.");
  return value;
}
class ReadyTrajectory final : public xm::MotionTrajectory {
public:
  ReadyTrajectory(std::vector<std::unique_ptr<xm::MotionPlan>> plans,
                  std::array<double,3> moves, double hold)
    : plans_(std::move(plans)), moves_(moves), hold_(hold) {
    for (size_t i = 0; i < plans_.size(); ++i) {
      duration_ += moves_[i]+(i<2?hold_:0);
      speed_ = std::max(speed_,plans_[i]->joint_speed_bound()*plans_[i]->duration()/moves_[i]);
      speeds_ = speeds_.cwiseMax(plans_[i]->joint_speed_bounds()*plans_[i]->duration()/moves_[i]);
    }
  }
  xm::Joints sample(double fraction) const override {
    const auto [phase,t] = location(fraction);
    return plans_[phase]->sample(t);
  }
  xm::Pose flange(double fraction) const override {
    const auto [phase,t] = location(fraction);
    return plans_[phase]->flange(t);
  }
  double duration() const override { return duration_; }
  double joint_speed_bound() const override { return speed_; }
  xm::Joints joint_speed_bounds() const override { return speeds_; }
private:
  std::pair<size_t,double> location(double fraction) const {
    require(std::isfinite(fraction),"유효한 경로 시각이 필요합니다.");
    double seconds = std::clamp(fraction,0.0,1.0)*duration_;
    for (size_t i = 0; i < 3; ++i) {
      const double span = moves_[i]+(i<2?hold_:0);
      if (seconds <= span || i == 2) return {i,std::clamp(seconds/moves_[i],0.0,1.0)};
      seconds -= span;
    }
    throw std::runtime_error("경로 구간 계산 실패");
  }
  std::vector<std::unique_ptr<xm::MotionPlan>> plans_;
  std::array<double,3> moves_;
  double hold_, duration_ = 0, speed_ = 0;
  xm::Joints speeds_ = xm::Joints::Zero();
};
}  // namespace

RunOptions run_options(int argc,char** argv) {
  require(argc>=3 && std::string(argv[1])=="--step","--step 단계가 필요합니다.");
  const std::string step=argv[2];
  require(step=="pre_align_ready" || step=="vision_align" || step=="vision_measure","알 수 없는 단계입니다.");
  RunOptions options;
  for(int i=3;i<argc;++i) {
    const std::string key=argv[i];
    if(key=="--auto-start") {
      require(!options.auto_start && step!="vision_measure","--auto-start는 이동 단계에서 한 번만 지정하세요.");
      options.auto_start=true;
    } else if(key=="--relearn") {
      require(step=="vision_align" && !options.relearn,"--relearn은 vision_align에서 한 번만 지정하세요.");
      options.relearn=true;
    } else if(key=="--expected-inputs") {
      require(options.expected_inputs.empty() && i+1<argc,"--expected-inputs 값이 필요합니다.");
      options.expected_inputs=argv[++i];
      require(!options.expected_inputs.empty() && options.expected_inputs.rfind("--",0)!=0,"--expected-inputs 값이 필요합니다.");
    } else throw std::runtime_error("알 수 없는 실행 옵션: "+key);
  }
  return options;
}

void require_execution_enabled(const YAML::Node& control) {
  const auto value = control["execution"]["auto_align_enabled"];
  require(value && value.IsScalar() && value.Tag() != "!" &&
    value.Tag() != "tag:yaml.org,2002:str" && value.Scalar() == "true",
    "auto_align 실행이 비활성화됐습니다. control.yaml의 execution.auto_align_enabled를 파일에서 직접 true로 바꾸세요.");
}
ReadyTarget ready_target(const YAML::Node& control, const YAML::Node& coordinates) {
  const auto value = control["pre_align_ready"]["target_pose"];
  require(value && value.IsScalar() && !value.IsNull() &&
    value.Scalar().find_first_not_of(" \t\r\n") != std::string::npos,
    "pre_align_ready.target_pose에 arm_poses.yaml의 목표 이름이 필요합니다.");
  const auto name = value.as<std::string>();
  return {name,{xm::named_pose(coordinates,name,"left"),xm::named_pose(coordinates,name,"right")}};
}
std::string input_fingerprint(const std::string& control, const std::string& coordinates) {
  // FNV-1a는 설정 일치 확인용이며 보안 서명으로 사용하지 않는다.
  std::uint64_t hash = 14695981039346656037ULL;
  for (const auto* text : {&control,&coordinates}) {
    for (unsigned char byte : *text) { hash ^= byte; hash *= 1099511628211ULL; }
    hash *= 1099511628211ULL; // 파일 사이의 NUL 구분자
  }
  std::ostringstream result;
  result << std::hex << std::setfill('0') << std::setw(16) << hash;
  return result.str();
}
CollisionSettings collision_settings(const YAML::Node& control) {
  CollisionSettings settings;
  const auto node = control["pre_align_ready"];
  settings.margin = positive(node,"collision_margin_m");
  settings.interval = positive(node,"collision_check_interval_s");
  settings.minimum_interval = positive(node,"collision_minimum_interval_s");
  require(settings.interval <= 0.1 && settings.minimum_interval <= settings.interval,
    "충돌 검사 간격은 최소 간격 이상·0.1초 이하여야 합니다.");
  return settings;
}
xm::Pose mounting_transform(const YAML::Node& frames, const std::string& side) {
  const auto node = frames["fixed_transforms"]["base_T_"+side+"_base"];
  const auto rotation = node["rotation_matrix"], position = node["translation_m"];
  require(rotation.IsSequence() && rotation.size() == 3 && position.IsSequence() && position.size() == 3,
    "base 장착 회전·위치가 필요합니다.");
  YAML::Node matrix;
  for (size_t row = 0; row < 3; ++row) {
    require(rotation[row].IsSequence() && rotation[row].size() == 3,"장착 회전은 3×3이어야 합니다.");
    YAML::Node values;
    for (size_t col = 0; col < 3; ++col) values.push_back(rotation[row][col].as<double>());
    values.push_back(position[row].as<double>()); matrix.push_back(values);
  }
  YAML::Node last;
  for (double value : {0.,0.,0.,1.}) last.push_back(value);
  matrix.push_back(last);
  return xm::matrix_pose(matrix);
}
namespace pre_align_ready {
xm::Pose vertical_move_target_world(const xm::Pose& current_world, const xm::Pose& target_world) {
  xm::Pose vertical = current_world;
  vertical.translation().z() = target_world.translation().z();
  return vertical;
}
}  // namespace pre_align_ready

std::array<xm::Pose,3> ready_waypoints(const xm::Pose& current,
                                    const xm::Pose& target, const xm::Pose& mount) {
  const xm::Pose start_world = mount*current, target_world = mount*target;
  const xm::Pose vertical = pre_align_ready::vertical_move_target_world(start_world,target_world);
  const xm::Pose horizontal = pre_align_ready::horizontal_move_target_world(vertical,target_world);
  return {mount.inverse()*vertical,mount.inverse()*horizontal,
    pre_align_ready::orientation_align_target_base(target)};
}

PreAlignReady::PreAlignReady(std::vector<xm::MotionArm>& arms,
                            const std::array<xm::Pose,2>& mounts,
                            const xm::Limits& limits, const std::function<bool()>& cancelled) {
  require(arms.size() == 2 && arms[0].side == "left" && arms[1].side == "right",
    "pre_align_ready는 왼팔·오른팔을 함께 사용합니다.");
  require(arms[0].backend && arms[1].backend && arms[0].backend!=arms[1].backend &&
    arms[0].model && arms[1].model,"양팔은 서로 다른 backend가 필요합니다.");
  std::array<std::vector<std::unique_ptr<xm::MotionPlan>>,2> paths;
  struct Result {
    std::vector<std::unique_ptr<xm::MotionPlan>> paths;
    std::array<double,3> phase_seconds{};
    xm::KinematicsStats sdk;
    double seconds=0;
  };
  const double started=xm::steady_seconds();
  std::atomic<bool> aborted{false};
  std::mutex error_mutex;
  std::exception_ptr error;
  const auto compute=[&](size_t side) {
    try {
      Result result;
      const double begun=xm::steady_seconds();
      const auto& arm=arms[side];
      const auto waypoints=ready_waypoints(arm.start,arm.target,mounts[side]);
      xm::SdkKinematics kinematics(*arm.backend,*arm.model,arm.flange_T_tcp);
      xm::Joints seed=arm.prepared.joints;
      for(size_t phase=0;phase<3;++phase) {
        require(!aborted.load() && !cancelled(),"경로 계산이 취소됐습니다.");
        const double phase_started=xm::steady_seconds();
        auto plan=std::make_unique<xm::MotionPlan>(kinematics,seed,waypoints[phase],arm.flange_T_tcp,limits);
        result.phase_seconds[phase]=xm::steady_seconds()-phase_started;
        seed=plan->sample(1);
        result.paths.push_back(std::move(plan));
      }
      result.sdk=kinematics.stats(); result.seconds=xm::steady_seconds()-begun;
      return result;
    } catch(...) {
      std::lock_guard<std::mutex> lock(error_mutex);
      if(!error) error=std::current_exception();
      aborted.store(true);
      throw;
    }
  };
  // Each worker owns one arm's SDK calls; join both before touching the plans
  // or propagating an error, so no worker can outlive its backend.
  std::array<std::future<Result>,2> jobs{
    std::async(std::launch::async,compute,0),std::async(std::launch::async,compute,1)};
  std::array<Result,2> results;
  for(size_t side=0;side<2;++side) {
    try { results[side]=jobs[side].get(); } catch(...) { /* error retained above */ }
  }
  if(error) std::rethrow_exception(error);
  std::ostringstream diagnostic;
  diagnostic<<std::fixed<<std::setprecision(6)<<"SDK_PLAN mode=parallel total_s="<<xm::steady_seconds()-started;
  for(size_t side=0;side<2;++side) {
    const auto& r=results[side]; const auto& s=r.sdk;
    diagnostic<<" / "<<arms[side].side<<" total_s="<<r.seconds
      <<" phase_s=["<<r.phase_seconds[0]<<" "<<r.phase_seconds[1]<<" "<<r.phase_seconds[2]<<"]"
      <<" fk_calls="<<s.fk_calls<<" fk_s="<<s.fk_seconds
      <<" solve_calls="<<s.solve_calls<<" solve_s="<<s.solve_seconds<<" solve_iterations="<<s.solve_iterations
      <<" fallback_ik_calls="<<s.fallback_ik_calls<<" fallback_ik_s="<<s.fallback_ik_seconds;
    paths[side]=std::move(results[side].paths);
    for(size_t phase=0;phase<3;++phase)
      durations_[phase]=std::max(durations_[phase],paths[side][phase]->duration());
  }
  planning_diagnostics_=diagnostic.str();
  for (size_t side = 0; side < 2; ++side)
    arms[side].plan = std::make_unique<ReadyTrajectory>(std::move(paths[side]),durations_,limits.arrival_hold);
  for (size_t phase = 0; phase < 2; ++phase) durations_[phase] += limits.arrival_hold;
}
std::string PreAlignReady::phase(double elapsed) const {
  const std::array<std::string,3> names{"vertical","horizontal","orientation"};
  for (size_t i = 0; i < 3; ++i) {
    if (elapsed <= durations_[i] || i == 2) return names[i];
    elapsed -= durations_[i];
  }
  return names.back();
}
}  // namespace auto_align
