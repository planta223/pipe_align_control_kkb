#include "auto_align/vision_align.hpp"
#include <gtest/gtest.h>
#include <filesystem>
#include <limits>

namespace aa=auto_align;
namespace xm=xarm_motion;
namespace {
YAML::Node config() { return YAML::LoadFile((std::filesystem::path(TEST_CONTROL_PC_DIR)/"config/control.yaml").string()); }
aa::VisionSettings settings() { return aa::vision_settings(config()); }
aa::PipeObservation pipe(Eigen::Vector3d p,Eigen::Vector3d a=Eigen::Vector3d::UnitX()) { return {p,a}; }
aa::VisionPair sample(int64_t stamp) { return {{pipe({-.1,0,.4}),pipe({.1,0,.4},-Eigen::Vector3d::UnitX())},stamp,0}; }
class Scene final : public aa::VisionIO {
public:
  explicit Scene(double hz=3):period(1/hz) {
    bases[0]=(Eigen::AngleAxisd(.35,Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(.3,Eigen::Vector3d::UnitY())).toRotationMatrix();
    bases[1]=(Eigen::AngleAxisd(-.4,Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(.5,Eigen::Vector3d::UnitX())).toRotationMatrix();
    state=sample(1);
    state.pipes[0].endpoint={-.12,0,.5};state.pipes[1].endpoint={.12,.012,.49};
    state.pipes[0].axis=Eigen::AngleAxisd(.13,Eigen::Vector3d::UnitZ())*Eigen::Vector3d::UnitX();
    state.pipes[1].axis=-(Eigen::AngleAxisd(-.07,Eigen::Vector3d::UnitY())*Eigen::Vector3d::UnitX());
    for(size_t i=0;i<2;++i) offsets[i]=.12*state.pipes[i].axis;
  }
  aa::VisionPair measure() override {
    clock+=std::max(.8,3*period);
    if(!duplicate) state.stamp=static_cast<int64_t>((100+clock)*1e9);
    if(dropout && commands>0) throw std::runtime_error("synthetic tracking lost");
    auto measured=state;
    if(noisy) for(size_t i=0;i<2;++i) {
      const double n=std::sin(clock*7+i);
      measured.pipes[i].endpoint+=.0002*n*Eigen::Vector3d(1,-1,1);
      measured.pipes[i].axis=Eigen::AngleAxisd(.001*n,Eigen::Vector3d::UnitY())*measured.pipes[i].axis;
    }
    return measured;
  }
  Eigen::Vector3d move(size_t side,const Eigen::Vector3d& u,bool rotation) override {
    ++commands;clock+=1;
    if(rotation) {
      const Eigen::Matrix3d r=bases[side]*Eigen::AngleAxisd(u.norm(),u.normalized()).toRotationMatrix()*bases[side].transpose();
      state.pipes[side].endpoint+=(r-Eigen::Matrix3d::Identity())*offsets[side];
      offsets[side]=r*offsets[side];state.pipes[side].axis=r*state.pipes[side].axis;
    } else if(!singular) {
      state.pipes[side].endpoint+=bases[side]*u;
      if(disturb_translation) {
        state.pipes[side].axis=Eigen::AngleAxisd(.09,Eigen::Vector3d::UnitY())*state.pipes[side].axis;
        disturb_translation=false;
      }
    }
    return u;
  }
  double elapsed() const override { return clock; }
  void report(const std::string& message) override { reports.push_back(message); }
  aa::VisionPair state;
  std::array<Eigen::Matrix3d,2> bases;
  std::array<Eigen::Vector3d,2> offsets;
  std::vector<std::string> reports;
  double clock=0,period;
  int commands=0;
  bool singular=false,dropout=false,duplicate=false,noisy=false,disturb_translation=false;
};
}
TEST(VisionConfig, GateAndBoundsAreIndependentOfReadyGate) {
  auto c=config();
  c["execution"]["vision_align_enabled"]=false;
  EXPECT_THROW(aa::require_vision_enabled(c),std::runtime_error);
  c["execution"]["vision_align_enabled"]=true;EXPECT_NO_THROW(aa::require_vision_enabled(c));
  c=YAML::Load("execution: {vision_align_enabled: 'true'}");EXPECT_THROW(aa::require_vision_enabled(c),std::runtime_error);
  c=config();c["vision_align"]["probe_translation_m"]=.1;EXPECT_THROW(aa::vision_settings(c),std::runtime_error);
  c=config();c["vision_align"]["left_endpoint_z_m"]=.158;EXPECT_THROW(aa::vision_settings(c),std::runtime_error);
}
TEST(VisionGeometry, CadEndpointAndZAxisBecomeCameraCoordinates) {
  xm::Pose pose=xm::Pose::Identity();pose.linear()=Eigen::AngleAxisd(std::acos(-1.)/2,Eigen::Vector3d::UnitY()).toRotationMatrix();
  pose.translation()=Eigen::Vector3d(.1,.2,.3);const auto p=aa::pipe_observation(pose,.316);
  EXPECT_LT((p.endpoint-Eigen::Vector3d(.416,.2,.3)).norm(),1e-12);
  EXPECT_LT((p.axis-Eigen::Vector3d::UnitX()).norm(),1e-12);
}
TEST(VisionPairs, ExactTimestampsOnlyAndConsumedImagesNeverReturn) {
  aa::VisionPairs b;b.add(0,100,1,pipe({0,0,0}));b.add(1,101,1,pipe({0,0,0}));EXPECT_FALSE(b.take());
  b.add(1,100,2,pipe({1,0,0}));const auto p=b.take();ASSERT_TRUE(p);EXPECT_EQ(p->stamp,100);EXPECT_EQ(p->received,2);
  b.add(0,100,3,pipe({0,0,0}));b.add(1,100,3,pipe({0,0,0}));EXPECT_FALSE(b.take());
  b.add(0,101,4,pipe({0,0,0}));ASSERT_TRUE(b.take());EXPECT_EQ(b.last_stamp(),101);
}
TEST(VisionPairs, RateIndependentWindowRejectsDuplicatesNoiseAndAxisFlips) {
  auto s=settings();
  for(double hz:{3.,10.,25.}) {
    std::vector<aa::VisionPair> data;
    for(int i=0;i<=int(std::ceil(s.sample_window*hz))+2;++i) data.push_back(sample(int64_t((100+i/hz)*1e9)));
    EXPECT_NO_THROW(aa::average_pairs(data,s));
    auto duplicate=data;duplicate[1].stamp=duplicate[0].stamp;EXPECT_THROW(aa::average_pairs(duplicate,s),std::runtime_error);
    auto noisy=data;noisy[1].pipes[0].endpoint.y()+=.1;EXPECT_THROW(aa::average_pairs(noisy,s),std::runtime_error);
    auto flipped=data;flipped[1].pipes[0].axis*=-1;EXPECT_THROW(aa::average_pairs(flipped,s),std::runtime_error);
  }
}
TEST(VisualSolve, SingularMappingFailsAndCorrectionsAreBounded) {
  EXPECT_THROW(aa::visual_solve(Eigen::Matrix3d::Zero(),Eigen::Vector3d::Ones(),.5,.05,.002),std::runtime_error);
  auto u=aa::visual_solve(Eigen::Matrix3d::Identity(),Eigen::Vector3d::Ones(),.5,.05,.002);
  EXPECT_NEAR(u.norm(),.002,1e-12);EXPECT_LT(u.maxCoeff(),0);
}
TEST(VisionController, BothArmsConvergeWithUnknownDifferentBasesAtVariableRates) {
  for(double hz:{3.,10.,25.}) {
    Scene io(hz);auto s=settings();s.deadline=600;
    const auto initial=io.state;
    aa::VisionAlign controller(s,io);ASSERT_NO_THROW(controller.run())<<"hz="<<hz;
    EXPECT_GT(controller.moves(),24);EXPECT_LT(controller.moves(),s.max_moves);
    for(size_t i=0;i<2;++i) {
      EXPECT_LE(std::acos(std::abs(io.state.pipes[i].axis.x())),s.angle_tolerance);
      EXPECT_LE((io.state.pipes[i].endpoint-controller.targets()[i]).norm(),s.position_tolerance);
      EXPECT_DOUBLE_EQ(controller.targets()[i].x(),initial.pipes[i].endpoint.x());
      EXPECT_DOUBLE_EQ(controller.targets()[i].y(),.006);EXPECT_DOUBLE_EQ(controller.targets()[i].z(),.495);
    }
    ASSERT_FALSE(io.reports.empty());EXPECT_EQ(io.reports.back().substr(0,8),"VERIFIED");
  }
}
TEST(VisionController, InvalidEndPairAndTrackingLossStopWithoutFurtherMoves) {
  Scene wrong;auto s=settings();s.endpoint_z={0,0};aa::VisionAlign invalid(s,wrong);
  EXPECT_THROW(invalid.run(),std::runtime_error);EXPECT_EQ(wrong.commands,0);
  Scene lost;lost.dropout=true;aa::VisionAlign dropout(settings(),lost);
  EXPECT_THROW(dropout.run(),std::runtime_error);EXPECT_EQ(lost.commands,1);
  Scene frozen;frozen.duplicate=true;aa::VisionAlign repeated(settings(),frozen);
  EXPECT_THROW(repeated.run(),std::runtime_error);EXPECT_EQ(frozen.commands,1);
}
TEST(VisionController, SingularTranslationAndMovementBudgetFailClosed) {
  Scene singular;singular.singular=true;auto s=settings();s.deadline=600;
  aa::VisionAlign impossible(s,singular);EXPECT_THROW(impossible.run(),std::runtime_error);
  Scene limited;s.max_moves=2;aa::VisionAlign budget(s,limited);EXPECT_THROW(budget.run(),std::runtime_error);EXPECT_EQ(limited.commands,2);
}
TEST(VisionController, NoiseAndTranslationDisturbanceKeepTargetAndReturnToRotation) {
  Scene io;io.noisy=true;io.disturb_translation=true;auto s=settings();s.deadline=600;
  aa::VisionAlign controller(s,io);EXPECT_NO_THROW(controller.run());
  int rotation_entries=0;
  for(const auto& line:io.reports) if(line.find("STEP5: 카메라")==0) ++rotation_entries;
  EXPECT_GE(rotation_entries,2);
}
