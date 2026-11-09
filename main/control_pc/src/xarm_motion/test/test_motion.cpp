#include "xarm_motion/motion_core.hpp"
#include "report_state.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <future>

namespace xm = xarm_motion;
namespace {
const std::string config_dir = std::string(TEST_CONTROL_PC_DIR)+"/config/";
xm::Config config() { return xm::read_config(YAML::LoadFile(config_dir+"control.yaml")); }
xm::Joints seed(bool right = false) {
  xm::Joints q;
  if (right) q << 4.6,14.8,-15.5,43.4,-69.7,112.6,-8;
  else q << 156.5,-19.1,-134.7,41.8,62.8,106,-130;
  return q*(std::acos(-1.0)/180);
}
class FakeBackend : public xm::ArmBackend {
public:
  FakeBackend(const xm::ArmKinematics& model, const xm::Pose& tcp, double& clock, xm::Joints q)
    : model(model), tcp(tcp), clock(clock), q(std::move(q)) {}
  xm::Feedback read() override {
    ++reads;
    if (fail_read) throw std::runtime_error("fake feedback failure");
    xm::Pose pose = model.fk(q)*tcp;
    pose.translation() += pose_error;
    return {q,velocity,pose,clock-age};
  }
  void enable() override {
    ++enables; owned = true;
    if (fail_enable) throw std::runtime_error("fake enable failure");
  }
  void send(const xm::Joints& command) override {
    EXPECT_TRUE(owned);
    ++sends;
    if (fail_send) throw std::runtime_error("fake send failure");
    velocity = (command-q)/.01;
    q = command;
  }
  void stop() override {
    ++stops;
    if (fail_stop) throw std::runtime_error("fake stop failure");
    owned = false; velocity.setZero();
  }
  const xm::ArmKinematics& model;
  xm::Pose tcp;
  double& clock;
  xm::Joints q, velocity = xm::Joints::Zero();
  Eigen::Vector3d pose_error = Eigen::Vector3d::Zero();
  double age = 0;
  int reads = 0, enables = 0, sends = 0, stops = 0;
  bool fail_read = false, fail_send = false, fail_enable = false, fail_stop = false, owned = false;
};
class MotionTest : public ::testing::Test {
protected:
  xm::ArmKinematics model{TEST_MODEL};
  xm::Limits limits = config().limits;
  xm::Pose tcp = xm::tcp_transform(YAML::LoadFile(config_dir+"fixed_frames.yaml"),"left");
  double clock = 100;
  bool cancelled = false;
  FakeBackend left{model,tcp,clock,seed()}, right{model,tcp,clock,seed(true)};
  xm::MotionArm arm(const std::string& side, FakeBackend& backend) {
    auto target_joints = backend.q;
    target_joints[5] += .015;
    return {side,&backend,&model,tcp,model.fk(backend.q),model.fk(target_joints),{},nullptr};
  }
  std::unique_ptr<xm::MotionSession> session(bool both = false) {
    std::vector<xm::MotionArm> arms;
    arms.push_back(arm("left",left));
    if (both) arms.push_back(arm("right",right));
    return std::make_unique<xm::MotionSession>(std::move(arms),limits,
      [this]{ return cancelled; },[this]{ return clock; });
  }
};

TEST(Configuration, GateRejectsDisabledMissingAndNonBoolean) {
  for (const std::string value : {"false","0","1","null","yes","\"true\"","[]"}) {
    const auto yaml = YAML::Load("execution: {standalone_enabled: "+value+"}");
    EXPECT_ANY_THROW(xm::require_standalone_enabled(yaml)) << value;
  }
  EXPECT_ANY_THROW(xm::require_standalone_enabled(YAML::Load("{}")));
  EXPECT_NO_THROW(xm::require_standalone_enabled(YAML::Load("execution: {standalone_enabled: true}")));
  auto disabled = YAML::LoadFile(config_dir+"control.yaml");
  disabled["execution"]["standalone_enabled"] = false;
  EXPECT_THROW(xm::require_standalone_enabled(disabled),std::runtime_error);
}
TEST(ReportState, WaitsForDecodedDataInsteadOfSocketConnection) {
  xm::detail::ReportState state;
  auto waiting = std::async(std::launch::async,[&] { state.wait_first(1,[]{return false;}); });
  EXPECT_EQ(waiting.wait_for(std::chrono::milliseconds(30)),std::future_status::timeout);
  xm::detail::RobotReport report;
  report.axis = 7; report.simulation = 0; report.received_at = xm::steady_seconds();
  report.tcp.translation() = Eigen::Vector3d(0,.116,.116);
  state.publish(report);
  ASSERT_EQ(waiting.wait_for(std::chrono::milliseconds(500)),std::future_status::ready);
  EXPECT_NO_THROW(waiting.get());
  const auto received = state.snapshot(xm::steady_seconds(),1);
  EXPECT_EQ(received.axis,7);
  EXPECT_NEAR(received.tcp.translation()[1],.116,1e-12);
  EXPECT_NEAR(received.tcp.translation()[2],.116,1e-12);
}
TEST(ReportState, MissingStaleInvalidAndCancelledReportsBlock) {
  xm::detail::ReportState state;
  EXPECT_ANY_THROW(state.snapshot(xm::steady_seconds(),.1));
  EXPECT_ANY_THROW(state.wait_first(.02,[]{return false;}));
  EXPECT_ANY_THROW(state.wait_first(1,[]{return true;}));
  xm::detail::RobotReport report;
  report.received_at = xm::steady_seconds()-.2;
  state.publish(report);
  EXPECT_ANY_THROW(state.snapshot(xm::steady_seconds(),.1));
  report.received_at = xm::steady_seconds(); report.error = "invalid report";
  state.publish(report);
  EXPECT_ANY_THROW(state.snapshot(xm::steady_seconds(),.1));
}
TEST(ReportState, FiveHzStatusDoesNotUseTheJointQueryDeadline) {
  const auto limits = config().limits;
  xm::detail::ReportState state;
  xm::detail::RobotReport report;
  const double now = xm::steady_seconds();
  report.received_at = now-.19;
  state.publish(report);
  EXPECT_NO_THROW(state.snapshot(now,limits.report_timeout));
  EXPECT_ANY_THROW(state.snapshot(now,limits.feedback_timeout));
  EXPECT_ANY_THROW(state.snapshot(now+limits.report_timeout,limits.report_timeout));
  EXPECT_DOUBLE_EQ(limits.feedback_timeout,.1);
}
TEST(Configuration, NumericLimitsAndAddressesAreValidated) {
  auto yaml = YAML::LoadFile(config_dir+"control.yaml");
  yaml["limits"]["joint_acceleration_rad_s2"] = -1;
  EXPECT_ANY_THROW(xm::read_config(yaml));
  yaml = YAML::LoadFile(config_dir+"control.yaml");
  yaml["limits"]["linear_speed_m_s"] = ".nan";
  EXPECT_ANY_THROW(xm::read_config(yaml));
  yaml = YAML::LoadFile(config_dir+"control.yaml");
  yaml["robots"]["right"]["ip"] = yaml["robots"]["left"]["ip"].as<std::string>();
  EXPECT_ANY_THROW(xm::read_config(yaml));
  EXPECT_NEAR(config().limits.effective_joint_speed(),.08,1e-12);
}
TEST(Configuration, PosesRejectNonRigidAndUnknownValues) {
  auto poses = YAML::LoadFile(config_dir+"arm_poses.yaml");
  EXPECT_ANY_THROW(xm::named_pose(poses,"unknown","left"));
  auto matrix = YAML::Clone(poses["poses"]["init1"]["left"]["base_T_flange"]);
  matrix[3][0] = 1;
  EXPECT_ANY_THROW(xm::matrix_pose(matrix));
  matrix = YAML::Clone(poses["poses"]["init1"]["left"]["base_T_flange"]);
  matrix[0][0] = 9;
  EXPECT_ANY_THROW(xm::matrix_pose(matrix));
  matrix = YAML::Clone(poses["poses"]["init1"]["left"]["base_T_flange"]);
  matrix[0][0] = ".nan";
  EXPECT_ANY_THROW(xm::matrix_pose(matrix));
}
TEST_F(MotionTest, OfficialModelRoundTripAndJointLimits) {
  EXPECT_LT((model.solve(model.fk(seed()),seed())-seed()).norm(),1e-8);
  auto invalid = seed(); invalid[3] = -2;
  EXPECT_ANY_THROW(model.fk(invalid));
  invalid[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_ANY_THROW(model.solve(xm::Pose::Identity(),invalid));
}
TEST_F(MotionTest, NamedPlansStartAtMeasuredJointsAndRespectLimits) {
  const auto poses = YAML::LoadFile(config_dir+"arm_poses.yaml");
  const auto frames = YAML::LoadFile(config_dir+"fixed_frames.yaml");
  for (const auto& side : {"left","right"}) {
    const xm::Joints actual = seed(std::string(side)=="right");
    const auto tip = xm::tcp_transform(frames,side);
    for (const auto& name : {"init1_floor_parallel_v1","init1_floor_parallel_v2"}) {
      const auto target = xm::named_pose(poses,name,side);
      xm::MotionPlan plan(model,actual,target,tip,limits);
      EXPECT_LT((plan.sample(0)-actual).norm(),1e-12);
      EXPECT_TRUE(xm::pose_close(model.fk(plan.sample(1)),target,1e-6,1e-6));
      const int count = static_cast<int>(std::ceil(plan.duration()*limits.rate));
      const double dt = plan.duration()/count;
      xm::Joints previous = actual, velocity = xm::Joints::Zero();
      xm::Pose previous_tcp = model.fk(previous)*tip;
      for (int i = 1; i <= count; ++i) {
        const auto q = plan.sample(double(i)/count);
        const xm::Joints next_velocity = (q-previous)/dt;
        EXPECT_LE((q-previous).cwiseAbs().maxCoeff(),limits.joint_speed*std::min(dt,limits.joint_dt_cap)+1e-9);
        EXPECT_LE((next_velocity-velocity).cwiseAbs().maxCoeff()/dt,limits.joint_acceleration+1e-6);
        const auto next_tcp = model.fk(q)*tip;
        EXPECT_TRUE(xm::pose_close(previous_tcp,next_tcp,limits.linear_speed*dt+1e-8,limits.angular_speed*dt+1e-8));
        previous = q; velocity = next_velocity; previous_tcp = next_tcp;
      }
    }
  }
}
TEST_F(MotionTest, PreparationAndFeedbackDoNotEnableOrSend) {
  auto s = session(true);
  s->prepare(clock); s->feedback(clock);
  EXPECT_EQ(left.enables+right.enables+left.sends+right.sends,0);
  s->stop();
  EXPECT_EQ(left.stops+right.stops,0);
}
TEST_F(MotionTest, StartMismatchBlocksBothArmsBeforeEnable) {
  auto left_arm = arm("left",left), right_arm = arm("right",right);
  right_arm.start.translation()[0] += .02;
  std::vector<xm::MotionArm> arms;
  arms.push_back(std::move(left_arm)); arms.push_back(std::move(right_arm));
  xm::MotionSession s(std::move(arms),limits,[]{return false;},[this]{return clock;});
  EXPECT_ANY_THROW(s.prepare(clock));
  EXPECT_EQ(left.enables+right.enables+left.sends+right.sends,0);
}
TEST_F(MotionTest, UnreachableTargetBlocksBeforeEnable) {
  auto value = arm("left",left);
  value.target.translation() = Eigen::Vector3d(2,0,0);
  std::vector<xm::MotionArm> arms; arms.push_back(std::move(value));
  xm::MotionSession s(std::move(arms),limits,[]{return false;},[this]{return clock;});
  EXPECT_ANY_THROW(s.prepare(clock));
  EXPECT_EQ(left.enables+left.sends,0);
}
TEST_F(MotionTest, MovementWhileWaitingRequiresRestart) {
  auto s = session(); s->prepare(clock);
  left.q[0] += .02;
  EXPECT_ANY_THROW(s->begin(clock));
  EXPECT_EQ(left.enables+left.sends,0);
}
TEST_F(MotionTest, MovingOrStaleOrMismatchedFeedbackCannotPrepare) {
  auto s = session();
  left.velocity[0] = .02;
  EXPECT_ANY_THROW(s->prepare(clock));
  left.velocity.setZero(); left.age = .2;
  EXPECT_ANY_THROW(s->prepare(clock));
  left.age = 0; left.pose_error[0] = .02;
  EXPECT_ANY_THROW(s->prepare(clock));
  EXPECT_EQ(left.enables+left.sends,0);
}
TEST_F(MotionTest, PartialEnableFailureStopsBoth) {
  auto s = session(true); s->prepare(clock);
  right.fail_enable = true;
  EXPECT_ANY_THROW(s->begin(clock));
  EXPECT_EQ(left.stops,1); EXPECT_EQ(right.stops,1);
  EXPECT_EQ(left.sends+right.sends,0);
}
TEST_F(MotionTest, CancelStopsBothAndNeverResumes) {
  auto s = session(true); s->prepare(clock); s->begin(clock);
  cancelled = true; clock += .01;
  EXPECT_ANY_THROW(s->tick(clock));
  EXPECT_EQ(left.stops,1); EXPECT_EQ(right.stops,1);
  EXPECT_EQ(left.sends+right.sends,0);
  cancelled = false;
  EXPECT_ANY_THROW(s->tick(clock+.01));
}
TEST_F(MotionTest, StopFailureStillAttemptsTheOtherArm) {
  auto s = session(true); s->prepare(clock); s->begin(clock);
  left.fail_stop = true;
  EXPECT_ANY_THROW(s->stop());
  EXPECT_EQ(left.stops,1); EXPECT_EQ(right.stops,1);
  EXPECT_FALSE(right.owned);
}
TEST_F(MotionTest, JointTrackingFailureStopsBothBeforeNextCommand) {
  auto s = session(true); s->prepare(clock); s->begin(clock);
  right.q[0] += .06;
  clock += .01;
  EXPECT_ANY_THROW(s->tick(clock));
  EXPECT_EQ(left.sends+right.sends,0);
  EXPECT_EQ(left.stops,1); EXPECT_EQ(right.stops,1);
}
TEST_F(MotionTest, FeedbackAndSendFailuresStopBoth) {
  for (bool fail_read : {true,false}) {
    auto s = session(true); s->prepare(clock); s->begin(clock);
    right.fail_read = fail_read; right.fail_send = !fail_read;
    clock += .01;
    EXPECT_ANY_THROW(s->tick(clock));
    EXPECT_EQ(left.stops,right.stops);
    EXPECT_FALSE(left.owned); EXPECT_FALSE(right.owned);
    right.fail_read = right.fail_send = false;
  }
}
TEST_F(MotionTest, TrackingFailureOrLongCycleStopsBoth) {
  for (bool stall : {false,true}) {
    auto s = session(true); s->prepare(clock); s->begin(clock);
    if (!stall) right.pose_error[0] = .02;
    clock += stall ? .2 : .01;
    EXPECT_ANY_THROW(s->tick(clock));
    EXPECT_FALSE(left.owned); EXPECT_FALSE(right.owned);
    right.pose_error.setZero();
  }
}
TEST_F(MotionTest, DualArmCompletesUsingMeasuredArrival) {
  auto s = session(true); s->prepare(clock); s->begin(clock);
  bool done = false;
  const int count = static_cast<int>(std::ceil((s->duration()+limits.arrival_hold+1)*limits.rate));
  for (int i = 0; i < count && !done; ++i) {
    clock += .01;
    done = s->tick(clock);
  }
  EXPECT_TRUE(done);
  EXPECT_GT(left.sends,0); EXPECT_GT(right.sends,0);
  EXPECT_EQ(left.stops,1); EXPECT_EQ(right.stops,1);
  EXPECT_FALSE(left.owned); EXPECT_FALSE(right.owned);
}
TEST_F(MotionTest, ArrivalRequiresActualPoseNotOnlyLastCommand) {
  auto s = session(); s->prepare(clock); s->begin(clock);
  // 추종 허용 5 mm 이내지만 도착 허용 2 mm 밖에 둔다.
  left.pose_error[0] = .003;
  bool threw = false;
  const int count = static_cast<int>(std::ceil((s->duration()+limits.arrival_timeout+1)*limits.rate));
  for (int i = 0; i < count; ++i) {
    clock += .01;
    try { EXPECT_FALSE(s->tick(clock)); }
    catch (const std::runtime_error& e) {
      EXPECT_NE(std::string(e.what()).find("도착 확인"),std::string::npos);
      threw = true; break;
    }
  }
  EXPECT_TRUE(threw); EXPECT_EQ(left.stops,1);
}
}  // namespace
