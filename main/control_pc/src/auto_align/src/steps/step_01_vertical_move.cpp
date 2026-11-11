#include "auto_align/steps/pre_align_ready.hpp"
#include <cmath>

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
};
}  // namespace

void require_execution_enabled(const YAML::Node& control) {
  const auto value = control["execution"]["auto_align_enabled"];
  require(value && value.IsScalar() && value.Tag() != "!" &&
    value.Tag() != "tag:yaml.org,2002:str" && value.Scalar() == "true",
    "auto_align 실행이 비활성화됐습니다. control.yaml의 execution.auto_align_enabled를 파일에서 직접 true로 바꾸세요.");
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
  std::array<std::vector<std::unique_ptr<xm::MotionPlan>>,2> paths;
  for (size_t side = 0; side < 2; ++side) {
    auto& arm = arms[side];
    const auto waypoints = ready_waypoints(arm.start,arm.target,mounts[side]);
    xm::SdkKinematics kinematics(*arm.backend,*arm.model,arm.flange_T_tcp);
    xm::Joints seed = arm.prepared.joints;
    for (size_t phase = 0; phase < 3; ++phase) {
      require(!cancelled(),"경로 계산이 취소됐습니다.");
      auto plan = std::make_unique<xm::MotionPlan>(kinematics,seed,waypoints[phase],arm.flange_T_tcp,limits);
      seed = plan->sample(1);
      durations_[phase] = std::max(durations_[phase],plan->duration());
      paths[side].push_back(std::move(plan));
    }
  }
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
