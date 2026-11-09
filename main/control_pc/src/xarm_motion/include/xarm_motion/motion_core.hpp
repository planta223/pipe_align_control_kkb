#pragma once

#include <Eigen/Geometry>
#include <kdl/chain.hpp>
#include <yaml-cpp/yaml.h>
#include <array>
#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace xarm_motion {
double steady_seconds();
using Joints = Eigen::Matrix<double, 7, 1>;
using Pose = Eigen::Isometry3d;

struct Limits {
  double rate, linear_speed, angular_speed, position_error, orientation_error;
  double joint_speed, joint_dt_cap, joint_error, joint_acceleration;
  double feedback_timeout, report_timeout, start_position, start_orientation;
  double arrival_position, arrival_orientation, arrival_joint, stationary_speed;
  double arrival_hold, arrival_timeout;
  double period() const { return 1.0 / rate; }
  double effective_joint_speed() const {
    return joint_speed * std::min(period(), joint_dt_cap) / period();
  }
};
struct Config {
  Limits limits;
  std::array<std::string, 2> ips;
  std::array<bool, 2> model1300;
};
void require_standalone_enabled(const YAML::Node& yaml);
Config read_config(const YAML::Node& yaml);
Pose matrix_pose(const YAML::Node& yaml);
Pose named_pose(const YAML::Node& yaml, const std::string& name, const std::string& side);
Pose tcp_transform(const YAML::Node& yaml, const std::string& side);
Pose interpolate(const Pose& start, const Pose& target, double fraction);
double rotation_error(const Pose& a, const Pose& b);
bool pose_close(const Pose& a, const Pose& b, double position, double angle);

class ArmKinematics {
public:
  explicit ArmKinematics(const std::string& urdf_file);
  Pose fk(const Joints& joints) const;
  Joints solve(const Pose& target, const Joints& seed) const;
  void check(const Joints& joints) const;
private:
  KDL::Chain chain_;
  Joints lower_, upper_;
};

// 경로 전체를 먼저 계산한 뒤 관절 속도·가속도에 맞춰 시간을 늘린다.
class MotionPlan {
public:
  MotionPlan(const ArmKinematics& arm, const Joints& actual, const Pose& target,
             const Pose& flange_T_tcp, const Limits& limits);
  Joints sample(double fraction) const;
  double duration() const { return duration_; }
private:
  std::vector<Joints> knots_, tangents_;
  double duration_ = 0;
};

struct Feedback {
  Joints joints, velocities;
  Pose base_T_tcp;
  double received_at = 0;
};
class ArmBackend {
public:
  virtual ~ArmBackend() = default;
  virtual Feedback read() = 0;
  virtual void enable() = 0;
  virtual void send(const Joints& joints) = 0;
  virtual void stop() = 0;
};
struct MotionArm {
  std::string side;
  ArmBackend* backend;
  const ArmKinematics* model;
  Pose flange_T_tcp, start, target;
  Feedback prepared;
  std::unique_ptr<MotionPlan> plan;
};

// SDK와 터미널 입력을 분리해 연결 없는 테스트에서도 실행 순서를 검증한다.
class MotionSession {
public:
  MotionSession(std::vector<MotionArm> arms, Limits limits, std::function<bool()> cancelled,
                std::function<double()> clock = steady_seconds);
  ~MotionSession();
  std::vector<Feedback> feedback(double now);
  void prepare(double now);
  void begin(double now);
  bool tick(double now);
  void stop();
  double duration() const { return duration_; }
  const std::vector<MotionArm>& arms() const { return arms_; }
  const std::vector<Feedback>& last_feedback() const { return last_feedback_; }
private:
  void check_cancelled() const;
  void validate_start(const MotionArm& arm, const Feedback& value) const;
  void validate_tracking(const MotionArm& arm, const Feedback& value, const Joints& command) const;
  std::vector<MotionArm> arms_;
  Limits limits_;
  std::function<bool()> cancelled_;
  std::function<double()> clock_;
  std::vector<Joints> previous_, velocity_;
  std::vector<Feedback> last_feedback_;
  double duration_ = 0, started_ = 0, previous_at_ = 0, arrived_at_ = -1;
  bool prepared_ = false, active_ = false;
};
}  // namespace xarm_motion
