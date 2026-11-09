#include "xarm_motion/motion_core.hpp"

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace xarm_motion {
namespace {
void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}
bool boolean(const YAML::Node& value, const std::string& name) {
  require(value && value.IsScalar() && value.Tag() != "!" &&
    value.Tag() != "tag:yaml.org,2002:str" &&
    (value.Scalar() == "true" || value.Scalar() == "false"), name + ": true 또는 false가 필요합니다.");
  return value.Scalar() == "true";
}
double positive(const YAML::Node& parent, const std::string& key) {
  const double value = parent[key].as<double>();
  require(std::isfinite(value) && value > 0, key + ": 양의 유한한 수치가 필요합니다.");
  return value;
}
Eigen::Vector3d vector3(const YAML::Node& yaml) {
  require(yaml.IsSequence() && yaml.size() == 3, "수치 3개가 필요합니다.");
  Eigen::Vector3d result;
  for (int i = 0; i < 3; ++i) result[i] = yaml[i].as<double>();
  require(result.allFinite(), "유한한 수치가 필요합니다.");
  return result;
}
void validate_pose(const Pose& value) {
  require(value.matrix().allFinite() &&
    (value.matrix().row(3) - Eigen::RowVector4d(0, 0, 0, 1)).norm() < 1e-9 &&
    (value.linear().transpose() * value.linear() - Eigen::Matrix3d::Identity()).norm() < 1e-6 &&
    std::abs(value.linear().determinant() - 1) < 1e-6, "올바른 4×4 강체 변환이 필요합니다.");
}
KDL::JntArray kdl_joints(const Joints& values) {
  KDL::JntArray result(7);
  for (int i = 0; i < 7; ++i) result(i) = values[i];
  return result;
}
KDL::Frame kdl_pose(const Pose& pose) {
  const Eigen::Matrix3d r = pose.linear();
  return KDL::Frame(KDL::Rotation(r(0,0), r(0,1), r(0,2), r(1,0), r(1,1), r(1,2),
    r(2,0), r(2,1), r(2,2)), KDL::Vector(pose.translation()[0], pose.translation()[1], pose.translation()[2]));
}
}  // namespace

double steady_seconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
void require_standalone_enabled(const YAML::Node& yaml) {
  require(boolean(yaml["execution"]["standalone_enabled"], "execution.standalone_enabled"),
    "xarm_motion 단독 실행이 비활성화되어 있습니다.\n"
    "main/control_pc/config/control.yaml의 execution.standalone_enabled 값을 true로 변경한 뒤 다시 실행하세요.");
}
Config read_config(const YAML::Node& yaml) {
  Config c{};
  const auto p = yaml["limits"], v = yaml["validation"];
  c.limits = {positive(p,"control_rate_hz"), positive(p,"linear_speed_m_s"),
    positive(p,"angular_speed_rad_s"), positive(p,"position_tracking_error_m"),
    positive(p,"orientation_tracking_error_rad"), positive(p,"joint_speed_coefficient_rad_s"),
    positive(p,"joint_step_dt_cap_s"), positive(p,"joint_tracking_error_rad"),
    positive(p,"joint_acceleration_rad_s2"), positive(v,"feedback_timeout_s"),
    positive(v,"rich_report_timeout_s"),
    positive(v,"start_position_tolerance_m"), positive(v,"start_orientation_tolerance_rad"),
    positive(v,"arrival_position_tolerance_m"), positive(v,"arrival_orientation_tolerance_rad"),
    positive(v,"arrival_joint_tolerance_rad"), positive(v,"stationary_joint_speed_rad_s"),
    positive(v,"arrival_hold_s"), positive(v,"arrival_timeout_s")};
  require(c.limits.rate >= 50 && c.limits.rate <= 200, "제어 주기는 50~200 Hz여야 합니다.");
  require(c.limits.feedback_timeout > c.limits.period(), "피드백 타임아웃은 제어 주기보다 커야 합니다.");
  require(c.limits.report_timeout > c.limits.feedback_timeout, "rich 보고 타임아웃은 관절 조회 타임아웃보다 커야 합니다.");
  require(c.limits.arrival_timeout > c.limits.arrival_hold, "도착 타임아웃은 유지 시간보다 커야 합니다.");
  for (int i = 0; i < 2; ++i) {
    const auto r = yaml["robots"][i == 0 ? "left" : "right"];
    c.ips[i] = r["ip"].as<std::string>();
    c.model1300[i] = boolean(r["model1300"], "model1300");
  }
  require(c.ips[0] != c.ips[1], "양팔 IP는 서로 달라야 합니다.");
  return c;
}
Pose matrix_pose(const YAML::Node& yaml) {
  require(yaml.IsSequence() && yaml.size() == 4, "4×4 행렬이 필요합니다.");
  Pose value;
  for (int i = 0; i < 4; ++i) {
    require(yaml[i].IsSequence() && yaml[i].size() == 4, "4×4 행렬이 필요합니다.");
    for (int j = 0; j < 4; ++j) value.matrix()(i,j) = yaml[i][j].as<double>();
  }
  validate_pose(value);
  return value;
}
Pose named_pose(const YAML::Node& yaml, const std::string& name, const std::string& side) {
  require(yaml["poses"] && yaml["poses"][name], "등록되지 않은 자세: " + name);
  return matrix_pose(yaml["poses"][name][side]["base_T_flange"]);
}
Pose tcp_transform(const YAML::Node& yaml, const std::string& side) {
  const auto value = yaml["fixed_transforms"][side + "_flange_T_" + side + "_tcp"];
  Pose pose = Pose::Identity();
  pose.translation() = vector3(value["translation_m"]);
  const auto rpy = vector3(value["rpy_rad"]);
  pose.linear() = (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX())).toRotationMatrix();
  return pose;
}
double rotation_error(const Pose& a, const Pose& b) {
  return Eigen::AngleAxisd(a.linear().transpose() * b.linear()).angle();
}
bool pose_close(const Pose& a, const Pose& b, double position, double angle) {
  return a.matrix().allFinite() && b.matrix().allFinite() &&
    (a.translation() - b.translation()).norm() <= position && rotation_error(a,b) <= angle;
}
Pose interpolate(const Pose& start, const Pose& target, double fraction) {
  require(std::isfinite(fraction) && fraction >= 0 && fraction <= 1, "보간 비율은 0~1이어야 합니다.");
  Pose value = Pose::Identity();
  value.translation() = start.translation() * (1-fraction) + target.translation() * fraction;
  value.linear() = Eigen::Quaterniond(start.linear()).slerp(fraction,
    Eigen::Quaterniond(target.linear())).normalized().toRotationMatrix();
  return value;
}

ArmKinematics::ArmKinematics(const std::string& file) {
  urdf::Model model;
  KDL::Tree tree;
  require(model.initFile(file) && kdl_parser::treeFromUrdfModel(model, tree) &&
    tree.getChain("robot_base", "link_eef", chain_) && chain_.getNrOfJoints() == 7,
    "공식 xArm7 URDF 체인을 읽을 수 없습니다: " + file);
  int i = 0;
  for (const auto& segment : chain_.segments) {
    if (segment.getJoint().getType() == KDL::Joint::Fixed) continue;
    const auto joint = model.getJoint(segment.getJoint().getName());
    require(joint && joint->limits, "URDF 관절 제한이 필요합니다.");
    lower_[i] = joint->limits->lower;
    upper_[i] = joint->limits->upper;
    ++i;
  }
}
void ArmKinematics::check(const Joints& joints) const {
  require(joints.allFinite() && (joints.array() >= lower_.array()-1e-8).all() &&
    (joints.array() <= upper_.array()+1e-8).all(), "관절각이 유한하지 않거나 URDF 제한을 벗어났습니다.");
}
Pose ArmKinematics::fk(const Joints& joints) const {
  check(joints);
  KDL::ChainFkSolverPos_recursive solver(chain_);
  KDL::Frame frame;
  require(solver.JntToCart(kdl_joints(joints), frame) >= 0, "FK 계산 실패");
  Pose pose = Pose::Identity();
  for (int i = 0; i < 3; ++i) {
    pose.translation()[i] = frame.p[i];
    for (int j = 0; j < 3; ++j) pose.linear()(i,j) = frame.M(i,j);
  }
  return pose;
}
Joints ArmKinematics::solve(const Pose& target, const Joints& seed) const {
  validate_pose(target);
  check(seed);
  KDL::ChainFkSolverPos_recursive fk_solver(chain_);
  KDL::ChainIkSolverVel_pinv velocity_solver(chain_, 1e-8);
  KDL::ChainIkSolverPos_NR_JL solver(chain_, kdl_joints(lower_), kdl_joints(upper_),
    fk_solver, velocity_solver, 500, 1e-7);
  KDL::JntArray output(7);
  require(solver.CartToJnt(kdl_joints(seed), kdl_pose(target), output) >= 0, "로컬 IK 계산 실패");
  Joints value = output.data;
  check(value);
  require(pose_close(fk(value), target, 1e-6, 1e-6), "IK 잔차가 허용 범위를 벗어났습니다.");
  return value;
}

MotionPlan::MotionPlan(const ArmKinematics& arm, const Joints& actual, const Pose& target,
                       const Pose& tcp, const Limits& limits) {
  validate_pose(target);
  const Pose start = arm.fk(actual);
  const double distance = (target.translation() - start.translation()).norm();
  const double angle = rotation_error(start, target);
  const int count = std::max(2, static_cast<int>(std::ceil(std::max(distance/.001, angle/.01))));
  require(count <= 10000, "경로 계산 범위를 초과했습니다.");
  knots_.push_back(actual);
  for (int i = 1; i <= count; ++i) {
    const Joints next = arm.solve(interpolate(start, target, double(i)/count), knots_.back());
    require((next-knots_.back()).cwiseAbs().maxCoeff() <= limits.joint_error,
      "인접 IK 해가 관절 연속성 제한을 벗어났습니다.");
    knots_.push_back(next);
  }
  tangents_.assign(knots_.size(), Joints::Zero());
  for (int i = 1; i < count; ++i) {
    for (int j = 0; j < 7; ++j) {
      const double a = knots_[i][j]-knots_[i-1][j], b = knots_[i+1][j]-knots_[i][j];
      if (a*b > 0) tangents_[i][j] = std::copysign(std::min(std::abs((a+b)/2),
        3*std::min(std::abs(a),std::abs(b))), a);
    }
  }
  tangents_.front() = knots_[1]-knots_[0];
  tangents_.back() = knots_.back()-knots_[knots_.size()-2];
  // 시작·도착의 감속은 개별 구간 대신 전체 경로의 smoothstep 시각에 적용한다.
  duration_ = std::max(1.0, 1.5*std::max(
    (distance + tcp.translation().norm()*angle)/limits.linear_speed, angle/limits.angular_speed));
  double derivative = 0, acceleration = 0;
  for (int i = 0; i < count; ++i) {
    const Joints a = 2*knots_[i]-2*knots_[i+1]+tangents_[i]+tangents_[i+1];
    const Joints b = -3*knots_[i]+3*knots_[i+1]-2*tangents_[i]-tangents_[i+1];
    for (int j = 0; j < 7; ++j) {
      for (double u : {0.0, 1.0, std::abs(a[j]) > 1e-15 ? std::clamp(-b[j]/(3*a[j]),0.0,1.0) : 0.0}) {
        derivative = std::max(derivative, std::abs((3*a[j]*u*u+2*b[j]*u+tangents_[i][j])*count));
      }
      acceleration = std::max(acceleration, std::max(std::abs(2*b[j]),
        std::abs(6*a[j]+2*b[j]))*count*count);
    }
  }
  // 원래 JointRamp 수식의 실제 주기당 상한에 맞추고 시간 오차 여유를 둔다.
  acceleration = 2.25*acceleration+6*derivative;
  derivative *= 1.5;
  duration_ = std::max({duration_, 1.5*derivative/limits.effective_joint_speed(),
    1.5*std::sqrt(acceleration/limits.joint_acceleration)});
  bool valid = false;
  for (int attempt = 0; attempt < 6; ++attempt) {
    require(duration_ <= 600, "제한값 적용 후 경로가 600초를 초과했습니다.");
    const int steps = static_cast<int>(std::ceil(duration_*limits.rate));
    const double dt = duration_/steps;
    double ratio = 1;
    Joints previous = actual, previous_velocity = Joints::Zero();
    Pose previous_flange = start, previous_tcp = start*tcp;
    for (int i = 1; i <= steps; ++i) {
      const double t = double(i)/steps;
      const Joints q = sample(t), velocity = (q-previous)/dt;
      arm.check(q);
      const Pose flange = arm.fk(q), tip = flange*tcp;
      require(pose_close(flange, interpolate(start,target,t*t*(3-2*t)), 0.0002, 0.001),
        "관절 보간 경로가 Cartesian 경로에서 벗어났습니다.");
      ratio = std::max({ratio, velocity.cwiseAbs().maxCoeff()/limits.effective_joint_speed(),
        std::sqrt((velocity-previous_velocity).cwiseAbs().maxCoeff()/dt/limits.joint_acceleration),
        (flange.translation()-previous_flange.translation()).norm()/dt/limits.linear_speed,
        (tip.translation()-previous_tcp.translation()).norm()/dt/limits.linear_speed,
        rotation_error(previous_flange,flange)/dt/limits.angular_speed});
      previous = q; previous_velocity = velocity; previous_flange = flange; previous_tcp = tip;
    }
    ratio = std::max(ratio, std::sqrt(previous_velocity.cwiseAbs().maxCoeff()/dt/limits.joint_acceleration));
    if (ratio <= 1.000001) { valid = true; break; }
    duration_ *= ratio*1.1;
  }
  require(valid, "경로 속도·가속도 검증에 실패했습니다.");
}
Joints MotionPlan::sample(double fraction) const {
  require(std::isfinite(fraction), "유효한 경로 시각이 필요합니다.");
  const double s = std::clamp(fraction,0.0,1.0);
  const double t = s*s*(3-2*s)*(knots_.size()-1);
  const size_t i = std::min(static_cast<size_t>(t), knots_.size()-2);
  const double u = t-i;
  return (2*u*u*u-3*u*u+1)*knots_[i] + (u*u*u-2*u*u+u)*tangents_[i] +
    (-2*u*u*u+3*u*u)*knots_[i+1] + (u*u*u-u*u)*tangents_[i+1];
}

MotionSession::MotionSession(std::vector<MotionArm> arms, Limits limits,
                             std::function<bool()> cancelled, std::function<double()> clock)
  : arms_(std::move(arms)), limits_(limits), cancelled_(std::move(cancelled)), clock_(std::move(clock)) {
  require(!arms_.empty(), "제어할 팔이 필요합니다.");
}
MotionSession::~MotionSession() { try { stop(); } catch (...) {} }
void MotionSession::check_cancelled() const { require(!cancelled_(), "실행이 취소되었습니다."); }
std::vector<Feedback> MotionSession::feedback(double now) {
  std::vector<Feedback> values;
  for (auto& arm : arms_) {
    check_cancelled();
    Feedback value = arm.backend->read();
    arm.model->check(value.joints);
    validate_pose(value.base_T_tcp);
    require(value.velocities.allFinite() && std::isfinite(value.received_at) &&
      std::abs(now-value.received_at) <= limits_.feedback_timeout, arm.side+": 피드백 타임아웃");
    values.push_back(value);
  }
  last_feedback_ = values;
  return values;
}
void MotionSession::validate_start(const MotionArm& arm, const Feedback& value) const {
  require(pose_close(value.base_T_tcp*arm.flange_T_tcp.inverse(), arm.start,
    limits_.start_position, limits_.start_orientation), arm.side+": 실제 자세가 지정한 시작 자세와 다릅니다.");
  require(value.velocities.cwiseAbs().maxCoeff() <= limits_.stationary_speed,
    arm.side+": 로봇이 정지 상태가 아닙니다.");
  validate_tracking(arm, value, value.joints);
}
void MotionSession::validate_tracking(const MotionArm& arm, const Feedback& value, const Joints& q) const {
  require((q-value.joints).cwiseAbs().maxCoeff() <= limits_.joint_error,
    arm.side+": 관절 추종 오차 초과");
  require(pose_close(arm.model->fk(q)*arm.flange_T_tcp, value.base_T_tcp,
    limits_.position_error, limits_.orientation_error),
    arm.side+": TCP 추종 오차 초과 또는 공식 모델과 실측 좌표계 불일치");
}
void MotionSession::prepare(double now) {
  require(!active_, "이동 중에는 경로를 준비할 수 없습니다.");
  prepared_ = false; duration_ = 0;
  const auto values = feedback(now);
  for (size_t i = 0; i < arms_.size(); ++i) {
    auto& arm = arms_[i];
    validate_start(arm, values[i]);
    arm.prepared = values[i];
    check_cancelled();
    arm.plan = std::make_unique<MotionPlan>(*arm.model, values[i].joints, arm.target,
      arm.flange_T_tcp, limits_);
    duration_ = std::max(duration_,arm.plan->duration());
  }
  prepared_ = true;
}
void MotionSession::begin(double now) {
  require(prepared_ && !active_, "Enter 실행 전에 경로 준비가 필요합니다.");
  const auto values = feedback(now);
  previous_.clear(); velocity_.clear();
  for (size_t i = 0; i < arms_.size(); ++i) {
    validate_start(arms_[i], values[i]);
    require((values[i].joints-arms_[i].prepared.joints).cwiseAbs().maxCoeff() <= limits_.arrival_joint,
      arms_[i].side+": 경로 계산 후 실제 관절각이 바뀌었습니다. 다시 실행하세요.");
    previous_.push_back(values[i].joints);
    velocity_.push_back(Joints::Zero());
  }
  check_cancelled();
  active_ = true; // 일부 팔의 활성화 실패도 모든 팔의 중지 대상으로 처리한다.
  try {
    for (auto& arm : arms_) { check_cancelled(); arm.backend->enable(); }
    started_ = previous_at_ = clock_();
    arrived_at_ = -1;
  } catch (...) { stop(); throw; }
}
bool MotionSession::tick(double now) {
  require(active_, "실행 중인 이동이 없습니다.");
  try {
    check_cancelled();
    const double dt = now-previous_at_;
    require(std::isfinite(dt) && dt > 0 && dt <= limits_.feedback_timeout, "제어 주기 타임아웃");
    const auto values = feedback(now);
    if (dt < limits_.period()*0.85) return false;
    const double fraction = std::clamp((now-started_)/duration_,0.0,1.0);
    std::vector<Joints> commands, velocities;
    bool arrived = fraction == 1;
    for (size_t i = 0; i < arms_.size(); ++i) {
      const auto& arm = arms_[i];
      validate_tracking(arm, values[i], previous_[i]);
      const Joints command = arm.plan->sample(fraction), velocity = (command-previous_[i])/dt;
      const Pose old_flange = arm.model->fk(previous_[i]), next_flange = arm.model->fk(command);
      const Pose old_tcp = old_flange*arm.flange_T_tcp, next_tcp = next_flange*arm.flange_T_tcp;
      require((command-previous_[i]).cwiseAbs().maxCoeff() <=
        limits_.joint_speed*std::min(dt,limits_.joint_dt_cap)+1e-9, arm.side+": 관절 증분 제한 초과");
      require((velocity-velocity_[i]).cwiseAbs().maxCoeff()/dt <= limits_.joint_acceleration+1e-7,
        arm.side+": 관절 가속도 제한 초과");
      require(pose_close(old_tcp,next_tcp,limits_.linear_speed*dt+1e-8,
        limits_.angular_speed*dt+1e-8) && pose_close(old_flange,next_flange,
        limits_.linear_speed*dt+1e-8,limits_.angular_speed*dt+1e-8), arm.side+": Cartesian 속도 제한 초과");
      validate_tracking(arm, values[i], command);
      arrived &= pose_close(values[i].base_T_tcp,arm.target*arm.flange_T_tcp,
        limits_.arrival_position,limits_.arrival_orientation) &&
        (values[i].joints-command).cwiseAbs().maxCoeff() <= limits_.arrival_joint &&
        values[i].velocities.cwiseAbs().maxCoeff() <= limits_.stationary_speed;
      commands.push_back(command); velocities.push_back(velocity);
    }
    for (size_t i = 0; i < arms_.size(); ++i) {
      check_cancelled();
      require(clock_()-now <= limits_.feedback_timeout, "명령 전송 전 피드백 타임아웃");
      arms_[i].backend->send(commands[i]);
    }
    previous_ = std::move(commands); velocity_ = std::move(velocities); previous_at_ = now;
    if (arrived) { if (arrived_at_ < 0) arrived_at_ = now; }
    else arrived_at_ = -1;
    if (arrived_at_ >= 0 && now-arrived_at_ >= limits_.arrival_hold) { stop(); return true; }
    require(now-started_ <= duration_+limits_.arrival_timeout, "실측 자세의 도착 확인 시간 초과");
    return false;
  } catch (...) { stop(); throw; }
}
void MotionSession::stop() {
  if (!active_) return;
  active_ = false;
  std::string errors;
  for (auto& arm : arms_) {
    try { arm.backend->stop(); }
    catch (const std::exception& e) { errors += arm.side+": "+e.what()+"; "; }
  }
  require(errors.empty(), "중지 요청 실패: "+errors);
}
}  // namespace xarm_motion
