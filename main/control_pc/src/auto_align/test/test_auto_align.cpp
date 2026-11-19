#include "auto_align/steps/pre_align_ready.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <limits>
#include <tuple>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

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
TEST(Configuration, AutoStartIsExplicitAndDoesNotReplaceInputFingerprint) {
  const auto parse=[](std::initializer_list<const char*> input) {
    std::vector<std::string> strings(input.begin(),input.end());
    std::vector<char*> argv;
    for(auto& arg:strings) argv.push_back(arg.data());
    return aa::run_options(static_cast<int>(argv.size()),argv.data());
  };
  EXPECT_FALSE(parse({"auto_align","--step","pre_align_ready"}).auto_start);
  EXPECT_TRUE(parse({"auto_align","--step","vision_align","--relearn","--auto-start"}).relearn);
  EXPECT_THROW(parse({"auto_align","--step","pre_align_ready","--relearn"}),std::runtime_error);
  EXPECT_THROW(parse({"auto_align","--step","vision_measure","--relearn"}),std::runtime_error);
  EXPECT_THROW(parse({"auto_align","--step","vision_align","--relearn","--relearn"}),std::runtime_error);
  for(const char* step:{"pre_align_ready","vision_align"}) {
    auto options=parse({"auto_align","--step",step,"--auto-start","--expected-inputs","abc"});
    EXPECT_TRUE(options.auto_start); EXPECT_EQ(options.expected_inputs,"abc");
    EXPECT_TRUE(parse({"auto_align","--step",step,"--expected-inputs","abc","--auto-start"}).auto_start);
  }
  EXPECT_THROW(parse({"auto_align","--step","vision_measure","--auto-start"}),std::runtime_error);
  EXPECT_THROW(parse({"auto_align","--step","pre_align_ready","--auto-start","--auto-start"}),std::runtime_error);
  EXPECT_THROW(parse({"auto_align","--step","pre_align_ready","--expected-inputs"}),std::runtime_error);
  EXPECT_THROW(parse({"auto_align","--step","pre_align_ready","--expected-inputs","--auto-start"}),std::runtime_error);
  EXPECT_THROW(parse({"auto_align","--step","pre_align_ready","--unexpected"}),std::runtime_error);
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
TEST(Collision, NewStartPrefixIsCheckedBeforeEnableAndOriginalSuffixIsReused) {
  xm::ArmKinematics model(official_model());
  const auto frames = YAML::LoadFile((root/"config/fixed_frames.yaml").string());
  const auto constraints = limits();
  for (bool blocked : {false,true}) {
    SCOPED_TRACE(blocked);
    double clock = 100;
    FakeBackend left(model,xm::tcp_transform(frames,"left"),seed(),clock);
    FakeBackend right(model,xm::tcp_transform(frames,"right"),seed(true),clock);
    const std::array<xm::Joints,2> original{left.q,right.q};
    std::vector<xm::MotionArm> arms;
    for (size_t side = 0; side < 2; ++side) {
      auto* backend = side == 0 ? &left : &right;
      const auto value = backend->read();
      const auto flange = value.base_T_tcp*backend->tcp.inverse();
      xm::SdkKinematics kinematics(*backend,model,backend->tcp);
      auto plan = std::make_unique<xm::MotionPlan>(kinematics,value.joints,flange,backend->tcp,constraints);
      arms.push_back({side == 0 ? "left" : "right",backend,&model,backend->tcp,
                      flange,flange,value,std::move(plan)});
    }
    // A synthetic sphere at unit radius from joint1 clears the box on the
    // original path. A 0.004 rad start drift violates its 20 mm margin when blocked.
    auto mounts = separated_mounts();
    for (size_t side = 0; side < 2; ++side)
      mounts[side].linear() = Eigen::AngleAxisd(-original[side][0],Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const auto collision_model = pendulum();
    aa::CollisionChecker checker({collision_model,collision_model},mounts,
                                  {box({1,blocked ? .062 : .1,0})},{});
    const double duration = std::max(arms[0].plan->duration(),arms[1].plan->duration());
    size_t original_samples = 0;
    checker.check_path([&](double t)->aa::DualJoints {
      ++original_samples;
      return {arms[0].plan->sample(t/duration),arms[1].plan->sample(t/duration)};
    },duration,{arms[0].plan->joint_speed_bound(),arms[1].plan->joint_speed_bound()},[]{return false;});
    xm::MotionSession session(std::move(arms),constraints,[]{return false;},[&]{return clock;});
    session.prepare_planned(clock);
    left.q[0] += .004;
    bool checked = false;
    size_t prefix_samples = 0;
    session.set_start_path_validator([&](const std::vector<xm::MotionArm>& connected,
                                        double path_seconds,double connection_seconds) {
      checked = true;
      EXPECT_EQ(left.enables+right.enables,0);
      EXPECT_GT(connection_seconds,0);
      EXPECT_LT(connection_seconds,path_seconds);
      checker.check_path([&](double t)->aa::DualJoints {
        ++prefix_samples;
        EXPECT_LE(t,connection_seconds);
        return {connected[0].plan->sample(t/path_seconds),connected[1].plan->sample(t/path_seconds)};
      },connection_seconds,{connected[0].plan->joint_speed_bound(),connected[1].plan->joint_speed_bound()},[]{return false;});
    });
    if (blocked) {
      EXPECT_THROW(session.begin(clock),std::runtime_error);
      EXPECT_EQ(left.enables+right.enables,0);
    } else {
      EXPECT_NO_THROW(session.begin(clock));
      EXPECT_EQ(left.enables+right.enables,2);
      EXPECT_LT(prefix_samples,original_samples);
      for (size_t side = 0; side < 2; ++side)
        EXPECT_LT((session.arms()[side].plan->sample(1)-original[side]).norm(),1e-12);
      session.stop();
    }
    EXPECT_TRUE(checked);
    EXPECT_GT(prefix_samples,0u);
    EXPECT_EQ(left.sends+right.sends,0);
  }
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

namespace {
struct PlanningGate {
  std::mutex mutex;
  std::condition_variable ready;
  int arrivals=0;
};
class ConcurrentBackend : public FakeBackend {
public:
  using FakeBackend::FakeBackend;
  PlanningGate* gate=nullptr;
  std::thread::id owner;
  bool fail=false;
  int active_queries=0;
  xm::Pose tcp_fk(const xm::Joints& q) override {
    struct Active { int& count; explicit Active(int& n):count(n){++count;} ~Active(){--count;} } active(active_queries);
    if(owner==std::thread::id()) {
      owner=std::this_thread::get_id();
      std::unique_lock<std::mutex> lock(gate->mutex);
      ++gate->arrivals; gate->ready.notify_all();
      if(!gate->ready.wait_for(lock,std::chrono::seconds(2),[&]{return gate->arrivals==2;}))
        throw std::runtime_error("Arm planning did not overlap");
    }
    if(fail) throw std::runtime_error("Synthetic parallel SDK failure");
    return FakeBackend::tcp_fk(q);
  }
};
std::string branched_model() {
  return fixture("branched",R"(<robot name="branched">
    <link name="root"/>
    <link name="a"><collision><origin xyz="1 0 0"/><geometry><sphere radius="0.02"/></geometry></collision>
      <collision><origin xyz="0 1 0"/><geometry><sphere radius="0.02"/></geometry></collision></link>
    <link name="b"><collision><origin xyz="0.4 0.1 -0.2"/><geometry><sphere radius="0.02"/></geometry></collision></link>
    <link name="c"><collision><origin xyz="0.3 0.1 0.2"/><geometry><sphere radius="0.02"/></geometry></collision></link>
    <joint name="joint1" type="revolute"><parent link="root"/><child link="a"/><origin xyz="0.3 -0.2 0.1" rpy="0.2 -0.1 0.3"/>
      <axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
    <joint name="fixed_b" type="fixed"><parent link="a"/><child link="b"/><origin xyz="0.2 0.3 0.1" rpy="-0.2 0.1 0.2"/></joint>
    <joint name="joint2" type="revolute"><parent link="a"/><child link="c"/><origin xyz="-0.3 0.2 0.1" rpy="0.1 0.2 -0.1"/>
      <axis xyz="1 0 0"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint></robot>)");
}
Eigen::Vector3d reference_point(const std::string& file,const std::string& link,
                              const xm::Joints& q,const Eigen::Vector3d& local,const xm::Pose& mount) {
  KDL::Tree tree; KDL::Chain chain;
  if(!kdl_parser::treeFromFile(file,tree) || !tree.getChain("root",link,chain))
    throw std::runtime_error("Reference FK setup failed");
  KDL::JntArray values(chain.getNrOfJoints()); size_t index=0;
  for(const auto& segment:chain.segments) if(segment.getJoint().getType()!=KDL::Joint::Fixed)
    values(index++)=q[segment.getJoint().getName().back()-'1'];
  KDL::Frame frame;
  if(KDL::ChainFkSolverPos_recursive(chain).JntToCart(values,frame)<0)
    throw std::runtime_error("Reference FK failed");
  const auto point=frame*KDL::Vector(local.x(),local.y(),local.z());
  return mount*Eigen::Vector3d(point.x(),point.y(),point.z());
}
}
TEST(Planning, ArmsOverlapAndFailuresJoinBothWorkersBeforeReturning) {
  xm::ArmKinematics model(official_model());
  const auto frames=YAML::LoadFile((root/"config/fixed_frames.yaml").string());
  for(bool fail:{false,true}) {
    double clock=100; PlanningGate gate;
    ConcurrentBackend left(model,xm::tcp_transform(frames,"left"),seed(),clock);
    ConcurrentBackend right(model,xm::tcp_transform(frames,"right"),seed(true),clock);
    left.gate=right.gate=&gate; left.fail=fail;
    std::vector<xm::MotionArm> arms;
    for(size_t side=0;side<2;++side) {
      auto* backend=side==0?&left:&right;
      const auto feedback=backend->read();
      const auto flange=feedback.base_T_tcp*backend->tcp.inverse();
      arms.push_back({side==0?"left":"right",backend,&model,backend->tcp,flange,flange,feedback,nullptr});
    }
    const auto construct=[&] {
      aa::PreAlignReady ready(arms,{xm::Pose::Identity(),xm::Pose::Identity()},limits(),[]{return false;});
      EXPECT_NE(ready.planning_diagnostics().find("mode=parallel"),std::string::npos);
      EXPECT_NE(ready.planning_diagnostics().find("fk_calls="),std::string::npos);
    };
    if(fail) {
      EXPECT_THROW(construct(),std::runtime_error);
      EXPECT_FALSE(arms[0].plan); EXPECT_FALSE(arms[1].plan);
    } else EXPECT_NO_THROW(construct());
    EXPECT_NE(left.owner,right.owner);
    EXPECT_NE(left.owner,std::this_thread::get_id());
    EXPECT_EQ(gate.arrivals,2);
    EXPECT_EQ(left.active_queries+right.active_queries,0);
    EXPECT_EQ(left.enables+right.enables+left.sends+right.sends,0);
  }
}
TEST(Collision, SharedFramesMatchIndependentKdlWithFixedBranchesAndMultipleShapes) {
  const auto file=branched_model();
  auto mounts=separated_mounts(); mounts[1].translation().x()=1000;
  mounts[0].translation()=Eigen::Vector3d(.5,-.4,.2);
  mounts[0].linear()=Eigen::AngleAxisd(.4,Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const std::vector<std::pair<std::string,Eigen::Vector3d>> targets{
    {"a",{1,0,0}},{"a",{0,1,0}},{"b",{.4,.1,-.2}},{"c",{.3,.1,.2}}};
  for(double angle:{-1.1,.2,1.4}) for(const auto& target:targets) {
    aa::DualJoints q{xm::Joints::Zero(),xm::Joints::Zero()}; q[0][0]=angle; q[0][1]=.7;
    const auto point=reference_point(file,target.first,q[0],target.second,mounts[0]);
    aa::CollisionChecker far({file,file},mounts,{box(point+Eigen::Vector3d(0,0,5))},{});
    EXPECT_NO_THROW(far.check(q));
    EXPECT_EQ(far.stats().samples,1u);
    EXPECT_EQ(far.stats().fk_evaluations,8u); // four link frames per arm, not per shape
    EXPECT_GT(far.stats().aabb_rejections,0u);
    aa::CollisionChecker contact({file,file},mounts,{box(point)},{});
    EXPECT_THROW(contact.check(q),std::runtime_error);
    EXPECT_GT(contact.stats().collision_queries,0u);
  }
  auto settings=aa::CollisionSettings(); settings.interval=1;
  const auto path=[](double t)->aa::DualJoints {
    aa::DualJoints q{xm::Joints::Zero(),xm::Joints::Zero()}; q[0][0]=-.7+1.4*t; q[0][1]=-.4+1.6*t; return q;
  };
  const auto obstacle=reference_point(file,"c",path(.25)[0],{.3,.1,.2},mounts[0]);
  aa::CollisionChecker checker({file,file},mounts,{box(obstacle)},settings);
  for(double t:{0.,.5,1.}) EXPECT_NO_THROW(checker.check(path(t)));
  std::array<xm::Joints,2> speeds{xm::Joints::Zero(),xm::Joints::Zero()}; speeds[0][0]=1.4; speeds[0][1]=1.6;
  EXPECT_THROW(checker.check_path_joint_bounds(path,1,speeds,[]{return false;}),std::runtime_error);
  EXPECT_GT(checker.stats().subdivisions,0u);
}
TEST(Collision, PerJointBoundsExcludeInactiveAndSharedAncestorMotion) {
  const auto file=fixture("common_ancestor",R"(<robot name="common">
    <link name="root"/><link name="common"/><link name="mid"/>
    <link name="b"><collision><origin xyz="1 0 0"/><geometry><sphere radius="0.01"/></geometry></collision></link>
    <link name="c"><collision><origin xyz="1.07 0 0"/><geometry><sphere radius="0.01"/></geometry></collision></link>
    <joint name="joint1" type="revolute"><parent link="root"/><child link="common"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="1" velocity="2"/></joint>
    <joint name="fixed_b" type="fixed"><parent link="common"/><child link="b"/></joint>
    <joint name="joint2" type="revolute"><parent link="common"/><child link="mid"/><axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="1" velocity="2"/></joint>
    <joint name="joint3" type="revolute"><parent link="mid"/><child link="c"/><axis xyz="1 0 0"/><limit lower="-3" upper="3" effort="1" velocity="2"/></joint></robot>)");
  auto mounts=separated_mounts(); mounts[1].translation().x()=1000;
  auto settings=aa::CollisionSettings(); settings.interval=1;
  aa::CollisionChecker checker({file,file},mounts,{},settings);
  const auto path=[](double t)->aa::DualJoints {
    aa::DualJoints q{xm::Joints::Zero(),xm::Joints::Zero()}; q[0][0]=-1+2*t; return q;
  };
  checker.check_path(path,1,{2,0},[]{return false;});
  const auto scalar_samples=checker.stats().samples;
  std::array<xm::Joints,2> speeds{xm::Joints::Zero(),xm::Joints::Zero()}; speeds[0][0]=2;
  checker.check_path_joint_bounds(path,1,speeds,[]{return false;});
  EXPECT_EQ(checker.stats().samples,3u);
  EXPECT_EQ(checker.stats().subdivisions,0u);
  EXPECT_GT(scalar_samples,checker.stats().samples);
  RecordProperty("scalar_samples",static_cast<int>(scalar_samples));
  RecordProperty("per_joint_samples",static_cast<int>(checker.stats().samples));
  speeds[0][0]=-1;
  EXPECT_THROW(checker.check_path_joint_bounds(path,1,speeds,[]{return false;}),std::runtime_error);
}

TEST(Collision, IndependentArmsStillDetectCollisionBetweenClearEndpointsAndMidpoint) {
  const auto file=pendulum();
  auto mounts=separated_mounts(); mounts[1].translation().x()=2;
  mounts[1].linear()=Eigen::AngleAxisd(std::acos(-1.),Eigen::Vector3d::UnitZ()).toRotationMatrix();
  auto settings=aa::CollisionSettings(); settings.interval=1;
  aa::CollisionChecker checker({file,file},mounts,{},settings);
  const auto path=[](double t)->aa::DualJoints {
    aa::DualJoints q{xm::Joints::Zero(),xm::Joints::Zero()}; q[0][0]=q[1][0]=-.3+1.2*t; return q;
  };
  for(double t:{0.,.5,1.}) EXPECT_NO_THROW(checker.check(path(t)));
  std::array<xm::Joints,2> speeds{xm::Joints::Zero(),xm::Joints::Zero()}; speeds[0][0]=speeds[1][0]=1.2;
  EXPECT_THROW(checker.check_path_joint_bounds(path,1,speeds,[]{return false;}),std::runtime_error);
  EXPECT_GT(checker.stats().subdivisions,0u);
}
