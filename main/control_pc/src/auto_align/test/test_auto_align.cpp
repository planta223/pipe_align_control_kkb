#include "auto_align/steps/pre_align_ready.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <limits>
#include <tuple>

namespace aa = auto_align;
namespace xm = xarm_motion;
namespace {
const std::filesystem::path root = TEST_CONTROL_PC_DIR;
xm::Limits limits() { return xm::read_config(YAML::LoadFile((root/"config/control.yaml").string())).limits; }
xm::Joints seed(bool right = false) {
  xm::Joints q;
  if (right) q << 4.6,14.8,-15.5,43.4,-69.7,112.6,-8;
  else q << 156.5,-19.1,-134.7,41.8,62.8,106,-130;
  return q*(std::acos(-1.0)/180);
}
std::string official_model() {
  return ament_index_cpp::get_package_share_directory("xarm_motion")+"/models/xarm7_false.urdf";
}
std::string fixture(const std::string& name, const std::string& content) {
  const auto directory = root/"_colcon/build/auto_align/fixtures";
  std::filesystem::create_directories(directory);
  const auto path = directory/(name+".urdf");
  std::ofstream(path) << content;
  return path.string();
}
std::string pendulum() {
  return fixture("pendulum",R"(<robot name="test">
    <link name="root"/><link name="link1"><collision><origin xyz="1 0 0"/>
    <geometry><sphere radius="0.02"/></geometry></collision></link>
    <joint name="joint1" type="revolute"><parent link="root"/><child link="link1"/>
    <axis xyz="0 0 1"/><limit lower="-3.14" upper="3.14" effort="1" velocity="1"/></joint></robot>)");
}
std::array<xm::Pose,2> separated_mounts() {
  auto second = xm::Pose::Identity(); second.translation().x() = 10;
  return {xm::Pose::Identity(),second};
}
aa::EnvironmentBox box(const Eigen::Vector3d& center, double size = 0.04) {
  xm::Pose pose = xm::Pose::Identity(); pose.translation() = center;
  return {"test_obstacle",pose,Eigen::Vector3d::Constant(size)};
}
class FakeBackend : public xm::ArmBackend {
public:
  FakeBackend(const xm::ArmKinematics& model, xm::Pose tcp, xm::Joints joints, double& clock)
    : model(model),tcp(std::move(tcp)),q(std::move(joints)),clock(clock) {}
  xm::Feedback read() override { return {q,velocity,model.fk(q)*tcp,clock}; }
  xm::Pose tcp_fk(const xm::Joints& joints) override {
    if (owned) throw std::runtime_error("이동 중 FK 조회");
    ++fk_calls; return model.fk(joints)*tcp;
  }
  xm::Joints tcp_ik(const xm::Pose&) override { throw xm::IkUnavailable("가짜 SDK IK 미제공"); }
  void enable() override { ++enables; owned = true; sent_at = clock; }
  void send(const xm::Joints& command) override {
    if (!owned) throw std::runtime_error("활성화 전 명령");
    ++sends; velocity = (command-q)/(clock-sent_at); sent_at = clock; q = command;
  }
  void stop() override { ++stops; owned = false; velocity.setZero(); }
  const xm::ArmKinematics& model;
  xm::Pose tcp;
  xm::Joints q, velocity = xm::Joints::Zero();
  double& clock;
  double sent_at = 0;
  bool owned = false;
  int enables = 0, sends = 0, stops = 0, fk_calls = 0;
};
}  // namespace

TEST(Configuration, GateRejectsDisabledMissingAndStringAndHasSeparateControl) {
  for (const auto& value : {"false","0","1","null","yes","\"true\"","[]"})
    EXPECT_THROW(aa::require_execution_enabled(YAML::Load("execution: {auto_align_enabled: "+std::string(value)+"}")),std::runtime_error);
  EXPECT_THROW(aa::require_execution_enabled(YAML::Load("execution: {standalone_enabled: true}")),std::runtime_error);
  EXPECT_NO_THROW(aa::require_execution_enabled(YAML::Load("execution: {auto_align_enabled: true}")));
}
TEST(Configuration, ClearanceAndTimeParametersAreValidated) {
  auto control = YAML::LoadFile((root/"config/control.yaml").string());
  EXPECT_DOUBLE_EQ(aa::collision_settings(control).margin,0.02);
  control["pre_align_ready"]["collision_margin_m"] = 0;
  EXPECT_THROW(aa::collision_settings(control),std::runtime_error);
  control["pre_align_ready"]["collision_margin_m"] = .02;
  control["pre_align_ready"]["collision_minimum_interval_s"] = .2;
  EXPECT_THROW(aa::collision_settings(control),std::runtime_error);
}
TEST(Configuration, ReadyTargetSelectsBothArmsFromConfiguredName) {
  auto control = YAML::LoadFile((root/"config/control.yaml").string());
  auto coordinates = YAML::LoadFile((root/"config/arm_poses.yaml").string());
  coordinates["poses"]["future_ready"] = YAML::Clone(coordinates["poses"]["pre_align_ready_v2"]);
  coordinates["poses"]["future_ready"]["left"]["base_T_flange"][0][3] = .4;
  for (const std::string name : {"pre_align_ready","pre_align_ready_v2","future_ready"}) {
    control["pre_align_ready"]["target_pose"] = name;
    const auto target = aa::ready_target(control,coordinates);
    EXPECT_EQ(target.name,name);
    EXPECT_LT((target.flange[0].matrix()-xm::named_pose(coordinates,name,"left").matrix()).norm(),1e-12);
    EXPECT_LT((target.flange[1].matrix()-xm::named_pose(coordinates,name,"right").matrix()).norm(),1e-12);
  }
}
TEST(Configuration, ReadyTargetRejectsMissingUnknownOrMalformedBeforePlanning) {
  const auto coordinates = YAML::LoadFile((root/"config/arm_poses.yaml").string());
  for (const std::string value : {"null","false","3","[]","{}","''","' '","unknown"}) {
    const auto control = YAML::Load("pre_align_ready: {target_pose: "+value+"}");
    EXPECT_ANY_THROW(aa::ready_target(control,coordinates)) << value;
  }
  EXPECT_ANY_THROW(aa::ready_target(YAML::Load("{}"),coordinates));
  const auto control = YAML::Load("pre_align_ready: {target_pose: pre_align_ready_v2}");
  for (const auto& side : {"left","right"}) {
    auto broken = YAML::Clone(coordinates);
    broken["poses"]["pre_align_ready_v2"].remove(side);
    EXPECT_ANY_THROW(aa::ready_target(control,broken));
    broken = YAML::Clone(coordinates);
    broken["poses"]["pre_align_ready_v2"][side]["base_T_flange"][0][0] = ".nan";
    EXPECT_ANY_THROW(aa::ready_target(control,broken));
  }
}
TEST(Configuration, InputFingerprintMatchesPythonAndDetectsEitherFileChanging) {
  EXPECT_EQ(aa::input_fingerprint("",""),"08328807b4eb6fed");
  EXPECT_EQ(aa::input_fingerprint("control","poses"),"feeec3e03e80b89c");
  EXPECT_EQ(aa::input_fingerprint("목표","poses"),"fe6319ad788cbf87");
  EXPECT_NE(aa::input_fingerprint("control2","poses"),aa::input_fingerprint("control","poses"));
  EXPECT_NE(aa::input_fingerprint("control","poses2"),aa::input_fingerprint("control","poses"));
  EXPECT_NE(aa::input_fingerprint("ab","c"),aa::input_fingerprint("a","bc"));
}
TEST(Waypoints, WorldVerticalThenHorizontalThenOrientationWorksWithTiltedMount) {
  const auto frames = YAML::LoadFile((root/"config/fixed_frames.yaml").string());
  for (const auto& side : {"left","right"}) {
    const auto mount = aa::mounting_transform(frames,side);
    xm::Pose start = xm::Pose::Identity(), target = xm::Pose::Identity();
    start.translation() = Eigen::Vector3d(.2,.1,.3);
    start.linear() = Eigen::AngleAxisd(.2,Eigen::Vector3d::UnitX()).toRotationMatrix();
    target.translation() = Eigen::Vector3d(.3,-.1,.4);
    target.linear() = Eigen::AngleAxisd(.5,Eigen::Vector3d::UnitY()).toRotationMatrix();
    const auto points = aa::ready_waypoints(start,target,mount);
    const xm::Pose first = mount*points[0], second = mount*points[1], current = mount*start, goal = mount*target;
    EXPECT_LT((first.translation().head<2>()-current.translation().head<2>()).norm(),1e-9);
    EXPECT_NEAR(first.translation().z(),goal.translation().z(),1e-9);
    EXPECT_LT((first.linear()-current.linear()).norm(),1e-9);
    EXPECT_LT((second.translation()-goal.translation()).norm(),1e-9);
    EXPECT_LT((second.linear()-current.linear()).norm(),1e-9);
    EXPECT_LT((points[2].matrix()-target.matrix()).norm(),1e-9);
  }
}
TEST(Collision, DetectsObstacleAndTwentyMillimeterMargin) {
  const auto model = pendulum(); const auto mounts = separated_mounts();
  aa::DualJoints q{xm::Joints::Zero(),xm::Joints::Zero()};
  aa::CollisionChecker safe({model,model},mounts,{box({1.10,0,0})},{});
  EXPECT_NO_THROW(safe.check(q));
  aa::CollisionChecker near({model,model},mounts,{box({1.055,0,0})},{});
  EXPECT_THROW(near.check(q),std::runtime_error);
  aa::CollisionChecker overlap({model,model},mounts,{box({1,0,0})},{});
  EXPECT_THROW(overlap.check(q),std::runtime_error);
}
TEST(Collision, AdaptiveCheckFindsCollisionBetweenClearEndpointsAndMidpoint) {
  const auto model = pendulum(); auto settings = aa::CollisionSettings(); settings.interval = 1;
  aa::CollisionChecker checker({model,model},separated_mounts(),{box({std::cos(-.5),std::sin(-.5),0})},settings);
  const auto sample = [](double t) -> aa::DualJoints {
    aa::DualJoints q{xm::Joints::Zero(),xm::Joints::Zero()}; q[0][0] = -1+2*t; return q;
  };
  EXPECT_NO_THROW(checker.check(sample(0)));
  EXPECT_NO_THROW(checker.check(sample(.5)));
  EXPECT_NO_THROW(checker.check(sample(1)));
  EXPECT_THROW(checker.check_path(sample,1,{2,0},[]{return false;}),std::runtime_error);
}
TEST(Collision, DetectsOtherArmAndNonAdjacentSelfCollision) {
  const auto model = pendulum(); aa::DualJoints q{xm::Joints::Zero(),xm::Joints::Zero()};
  aa::CollisionChecker other({model,model},{xm::Pose::Identity(),xm::Pose::Identity()},{},{});
  EXPECT_THROW(other.check(q),std::runtime_error);
  const auto self = fixture("self",R"(<robot name="self">
    <link name="root"><collision><geometry><sphere radius="0.05"/></geometry></collision></link>
    <link name="a"/><link name="b"><collision><geometry><sphere radius="0.05"/></geometry></collision></link>
    <joint name="joint1" type="revolute"><parent link="root"/><child link="a"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
    <joint name="joint2" type="revolute"><parent link="a"/><child link="b"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint></robot>)");
  aa::CollisionChecker checker({self,model},separated_mounts(),{},{});
  EXPECT_THROW(checker.check(q),std::runtime_error);
}
TEST(Collision, CancellationAndMalformedGeometryFailClosed) {
  const auto model = pendulum(); aa::CollisionChecker checker({model,model},separated_mounts(),{},{});
  auto sample = [](double) -> aa::DualJoints { return {xm::Joints::Zero(),xm::Joints::Zero()}; };
  EXPECT_THROW(checker.check_path(sample,1,{0,0},[]{return true;}),std::runtime_error);
  EXPECT_THROW(checker.check_path(sample,1,{-1,0},[]{return false;}),std::runtime_error);
  EXPECT_THROW((aa::CollisionChecker({"/missing.urdf",model},separated_mounts(),{},{})),std::runtime_error);
  auto invalid = box({0,0,0}); invalid.world_T_box.translation().x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW((aa::CollisionChecker({model,model},separated_mounts(),{invalid},{})),std::runtime_error);
}
TEST(Environment, BoxesMatchExistingVisualizationConvention) {
  const auto input = YAML::Load(R"(floor_z_m: -1.0
table: {enabled: true, size_m: [0.8, 1.6, 0.04], position_m: [0.1, 0.2, -0.3], rpy_rad: [0, 0, 0]}
blind: {enabled: true, width_m: 1.6, height_m: 2.0, thickness_m: 0.01, gap_from_table_m: 0.03, offset_m: [0, 0, 0], rpy_rad: [0, 0, 0]}
right_wall: {enabled: true, length_m: 0.8, height_m: 2.0, thickness_m: 0.01, gap_from_table_m: 0.1, offset_m: [0, 0, 0], rpy_rad: [0, 0, 0]})");
  const auto boxes = aa::environment_boxes(input);
  ASSERT_EQ(boxes.size(),3u);
  EXPECT_LT((boxes[0].world_T_box.translation()-Eigen::Vector3d(.5,.2,-.32)).norm(),1e-12);
  EXPECT_LT((boxes[1].world_T_box.translation()-Eigen::Vector3d(.935,.2,0)).norm(),1e-12);
  EXPECT_LT((boxes[2].world_T_box.translation()-Eigen::Vector3d(.5,-.705,0)).norm(),1e-12);
  auto hidden = YAML::Clone(input); hidden["blind"]["enabled"] = false;
  EXPECT_EQ(aa::environment_boxes(hidden).size(),2u);
}
class PreAlignTiming : public ::testing::TestWithParam<std::tuple<bool,bool>> {};
TEST_P(PreAlignTiming, EntireSequenceKeepsCheckedPathAndLimitsWithoutRuntimeFkOrIk) {
  xm::ArmKinematics model(official_model());
  const auto frames = YAML::LoadFile((root/"config/fixed_frames.yaml").string());
  const auto coordinates = YAML::LoadFile((root/"config/arm_poses.yaml").string());
  const auto constraints = limits(); double clock = 100;
  auto control = YAML::LoadFile((root/"config/control.yaml").string());
  control["pre_align_ready"]["target_pose"] = std::get<1>(GetParam()) ? "pre_align_ready_v2" : "pre_align_ready";
  const auto target = aa::ready_target(control,coordinates);
  FakeBackend left(model,xm::tcp_transform(frames,"left"),seed(),clock);
  FakeBackend right(model,xm::tcp_transform(frames,"right"),seed(true),clock);
  std::vector<xm::MotionArm> arms;
  for (size_t side = 0; side < 2; ++side) {
    const std::string name = side == 0 ? "left" : "right";
    auto* backend = side == 0 ? &left : &right;
    const auto value = backend->read();
    arms.push_back({name,backend,&model,backend->tcp,value.base_T_tcp*backend->tcp.inverse(),
      target.flange[side],value,nullptr});
  }
  aa::PreAlignReady step(arms,{aa::mounting_transform(frames,"left"),aa::mounting_transform(frames,"right")},constraints,[]{return false;});
  const double duration = arms[0].plan->duration();
  EXPECT_DOUBLE_EQ(duration,arms[1].plan->duration());
  EXPECT_EQ(step.phase(0),"vertical");
  EXPECT_EQ(step.phase(step.phase_durations()[0]+.01),"horizontal");
  EXPECT_EQ(step.phase(duration-.01),"orientation");
  for (auto& arm : arms) {
    const double dt = .001, bound = arm.plan->joint_speed_bound();
    for (double t = dt; t < duration; t += dt)
      EXPECT_LE((arm.plan->sample(t/duration)-arm.plan->sample((t-dt)/duration)).cwiseAbs().maxCoeff()/dt,bound+1e-7);
  }
  // 기존 벽시계 진행 방식은 30 ms 지연 때 명령 증분 상한을 넘는다.
  double original_max_step = 0;
  for (double t = .03; t < duration; t += .01)
    for (const auto& arm : arms)
      original_max_step = std::max(original_max_step,
        (arm.plan->sample(t/duration)-arm.plan->sample((t-.03)/duration)).cwiseAbs().maxCoeff());
  EXPECT_GT(original_max_step,constraints.joint_speed*constraints.joint_dt_cap);
  xm::MotionSession session(std::move(arms),constraints,[]{return false;},[&]{return clock;});
  session.prepare_planned(clock);
  EXPECT_EQ(left.enables+right.enables+left.sends+right.sends,0);
  left.q[0] += .006;
  EXPECT_THROW(session.begin(clock),std::runtime_error);
  EXPECT_EQ(left.enables+right.enables,0);
  left.q[0] -= .006;
  session.begin(clock);
  const int calls = left.fk_calls+right.fk_calls;
  bool done = false;
  const std::array<double,9> intervals{.01,.015,.02,.03,.01,.009,.03,.06,.01};
  double previous_progress = 0;
  std::array<xm::Joints,2> previous{left.q,right.q};
  std::array<xm::Joints,2> velocity{xm::Joints::Zero(),xm::Joints::Zero()};
  std::array<bool,3> phases{false,false,false};
  bool adjusted = false;
  const double began = clock;
  for (int i = 0; i < 61000; ++i) {
    const double dt = std::get<0>(GetParam()) ? intervals[i%intervals.size()] : .01;
    clock += dt;
    done = session.tick(clock);
    const double progress = session.path_elapsed();
    EXPECT_GE(progress,previous_progress);
    EXPECT_LE(progress-previous_progress,dt+1e-9);
    adjusted |= progress-previous_progress < dt*duration/session.duration()-1e-8;
    for (size_t side = 0; side < 2; ++side) {
      const auto& arm = session.arms()[side];
      const auto& backend = side == 0 ? left : right;
      const xm::Joints delta = backend.q-previous[side], next_velocity = delta/dt;
      EXPECT_LE(delta.cwiseAbs().maxCoeff(),constraints.joint_speed*std::min(dt,constraints.joint_dt_cap)+1.1e-9);
      EXPECT_LE((next_velocity-velocity[side]).cwiseAbs().maxCoeff()/dt,constraints.joint_acceleration+1.1e-7);
      EXPECT_LT((backend.q-arm.plan->sample(progress/duration)).norm(),1e-10);
      const auto before = model.fk(previous[side]), after = model.fk(backend.q);
      EXPECT_TRUE(xm::pose_close(before,after,constraints.linear_speed*dt+1e-7,constraints.angular_speed*dt+1e-7));
      EXPECT_TRUE(xm::pose_close(before*backend.tcp,after*backend.tcp,
        constraints.linear_speed*dt+1e-7,constraints.angular_speed*dt+1e-7));
      previous[side] = backend.q; velocity[side] = next_velocity;
    }
    const auto phase = step.phase(progress);
    phases[phase == "vertical" ? 0 : phase == "horizontal" ? 1 : 2] = true;
    previous_progress = progress;
    if (done) break;
  }
  EXPECT_TRUE(phases[0] && phases[1] && phases[2]);
  EXPECT_NEAR(session.path_elapsed(),duration,1e-9);
  if (std::get<0>(GetParam())) { EXPECT_TRUE(adjusted); EXPECT_GT(clock-began,session.duration()); }
  EXPECT_TRUE(done); EXPECT_EQ(left.fk_calls+right.fk_calls,calls);
  EXPECT_GT(left.stops,0); EXPECT_GT(right.stops,0);
}

INSTANTIATE_TEST_SUITE_P(Scheduler,PreAlignTiming,::testing::Combine(::testing::Bool(),::testing::Bool()));
