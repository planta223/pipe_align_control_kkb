#include "auto_align/vision_align.hpp"
#include <gtest/gtest.h>
#include <filesystem>
#include <limits>
#include <fstream>
#include <chrono>
#include <unistd.h>

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
    measured.position_error.fill(position_error);measured.angle_error.fill(angle_error);
    if(noisy) for(size_t i=0;i<2;++i) {
      const double n=std::sin(clock*7+i);
      measured.pipes[i].endpoint+=.0002*n*Eigen::Vector3d(1,-1,1);
      measured.pipes[i].axis=Eigen::AngleAxisd(.001*n,Eigen::Vector3d::UnitY())*measured.pipes[i].axis;
    }
    return measured;
  }
  Eigen::Vector3d move(size_t side,const Eigen::Vector3d& u,bool rotation) override {
    ++commands;clock+=1;
    if(commands==change_after) bases[0]=Eigen::AngleAxisd(.8,Eigen::Vector3d::UnitX()).toRotationMatrix()*bases[0];
    const Eigen::Vector3d actual=actual_scale*u;
    if(rotation) {
      const Eigen::Matrix3d r=bases[side]*Eigen::AngleAxisd(actual.norm(),actual.normalized()).toRotationMatrix()*bases[side].transpose();
      state.pipes[side].endpoint+=(r-Eigen::Matrix3d::Identity())*offsets[side];
      offsets[side]=r*offsets[side];state.pipes[side].axis=r*state.pipes[side].axis;
    } else if(!singular) {
      state.pipes[side].endpoint+=bases[side]*actual;
      if(disturb_translation) {
        state.pipes[side].axis=Eigen::AngleAxisd(.09,Eigen::Vector3d::UnitY())*state.pipes[side].axis;
        disturb_translation=false;
      }
    }
    return actual;
  }
  std::optional<xm::Pose> tcp_pose(size_t side) override { return tcp[side]; }
  double elapsed() const override { return clock; }
  void report(const std::string& message) override { reports.push_back(message); }
  aa::VisionPair state;
  std::array<Eigen::Matrix3d,2> bases;
  std::array<Eigen::Vector3d,2> offsets;
  std::array<xm::Pose,2> tcp{xm::Pose::Identity(),xm::Pose::Identity()};
  std::vector<std::string> reports;
  double clock=0,period;
  double actual_scale=1;
  double position_error=0,angle_error=0;
  int commands=0;
  int change_after=-1;
  bool singular=false,dropout=false,duplicate=false,noisy=false,disturb_translation=false;
};
struct CacheFile {
  std::filesystem::path directory=std::filesystem::path(TEST_CONTROL_PC_DIR)/"_colcon/build/auto_align/fixtures"/
    ("vision_cache_"+std::to_string(getpid())+"_"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::path path=directory/"jacobians.yaml";
  ~CacheFile() { std::filesystem::remove_all(directory); }
};
int cold_save(const aa::VisionSettings& s,const std::filesystem::path& path) {
  Scene io;aa::VisionCache cache(path,"test-geometry",s);cache.load();
  aa::VisionAlign controller(s,io,&cache);controller.run();
  return controller.moves();
}
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
TEST(VisionPairs, DriftUncertaintyDoesNotDisappearWithManyCorrelatedSamples) {
  auto s=settings();std::vector<aa::VisionPair> data;
  for(int i=0;i<=100;++i) {
    auto pair=sample(int64_t((100+i*.02)*1e9));
    for(auto& p:pair.pipes) {
      p.endpoint.y()+=i*.00002;
      p.axis=Eigen::AngleAxisd(i*.00005,Eigen::Vector3d::UnitZ())*p.axis;
    }
    data.push_back(pair);
  }
  const auto mean=aa::average_pairs(data,s);
  EXPECT_NEAR(mean.position_error[0],.001,1e-12);
  EXPECT_NEAR(mean.angle_error[0],.0025,1e-10);
}
TEST(VisionController, BothArmsConvergeWithUnknownDifferentBasesAtVariableRates) {
  for(double hz:{3.,10.,25.}) {
    Scene io(hz);auto s=settings();s.deadline=600;
    const auto initial=io.state;
    aa::VisionAlign controller(s,io);ASSERT_NO_THROW(controller.run())<<"hz="<<hz;
    EXPECT_GT(controller.moves(),12);EXPECT_LT(controller.moves(),s.max_moves);
    for(const auto& side:controller.counts()) for(const auto& kind:side) EXPECT_EQ(kind.learning,3);
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
TEST(VisionController, ConsecutiveProbesUseActualDisplacementAndRetainFreshTargets) {
  Scene io;io.actual_scale=.8;auto s=settings();s.deadline=600;
  const auto initial=io.state;
  aa::VisionAlign controller(s,io);ASSERT_NO_THROW(controller.run());
  for(const auto& side:controller.counts()) for(const auto& kind:side) EXPECT_EQ(kind.learning,3);
  EXPECT_DOUBLE_EQ(controller.targets()[0].x(),initial.pipes[0].endpoint.x());
  EXPECT_DOUBLE_EQ(controller.targets()[0].y(),.006);
}
TEST(VisionCache, NewProcessReusesMappingsWhileTargetsComeFromNewMeasurements) {
  CacheFile file;auto s=settings();s.deadline=600;const int cold_moves=cold_save(s,file.path);
  Scene io;for(auto& p:io.state.pipes) p.endpoint.y()+=.002;
  aa::VisionCache cache(file.path,"test-geometry",s);EXPECT_NE(cache.load().find("loaded=4"),std::string::npos);
  aa::VisionAlign controller(s,io,&cache);ASSERT_NO_THROW(controller.run());
  int corrections=0;
  for(const auto& side:controller.counts()) for(const auto& kind:side) {
    EXPECT_EQ(kind.learning,0);EXPECT_EQ(kind.reused,1);EXPECT_GT(kind.validation,0);
    EXPECT_EQ(kind.validation,kind.correction);corrections+=kind.correction;
  }
  EXPECT_EQ(corrections,controller.moves());
  EXPECT_LT(controller.moves(),cold_moves);
  RecordProperty("cold_moves",cold_moves);RecordProperty("warm_moves",controller.moves());
  EXPECT_DOUBLE_EQ(controller.targets()[0].y(),.008);
}
TEST(VisionCache, WrongResponseRelearnsOnlyAffectedArmAndKind) {
  CacheFile file;auto s=settings();s.deadline=600;cold_save(s,file.path);
  aa::VisionCache edit(file.path,"test-geometry",s);edit.load();auto entries=edit.entries();
  entries[0][0]->jacobian*=-1;edit.save(entries);
  Scene io;aa::VisionCache cache(file.path,"test-geometry",s);cache.load();
  aa::VisionAlign controller(s,io,&cache);ASSERT_NO_THROW(controller.run());
  EXPECT_EQ(controller.counts()[0][0].rejected,1);EXPECT_EQ(controller.counts()[0][0].learning,3);
  EXPECT_EQ(controller.counts()[0][1].learning,0);EXPECT_EQ(controller.counts()[1][0].learning,0);
  EXPECT_EQ(controller.counts()[1][1].learning,0);
}
TEST(VisionCache, LaterResponseIsStillCheckedAfterFirstDirectionWasAccepted) {
  CacheFile file;auto s=settings();s.deadline=600;cold_save(s,file.path);
  Scene io;io.change_after=3;
  aa::VisionCache cache(file.path,"test-geometry",s);cache.load();
  aa::VisionAlign controller(s,io,&cache);ASSERT_NO_THROW(controller.run());
  EXPECT_EQ(controller.counts()[0][1].reused,1);
  EXPECT_EQ(controller.counts()[0][1].rejected,1);EXPECT_EQ(controller.counts()[0][1].learning,3);
  EXPECT_EQ(controller.counts()[1][1].learning,0);
}
TEST(VisionCache, TcpReferenceMismatchRelearnsOnlyThatArm) {
  CacheFile file;auto s=settings();s.deadline=600;cold_save(s,file.path);
  Scene io;io.tcp[0].translation().x()=.04;
  aa::VisionCache cache(file.path,"test-geometry",s);cache.load();
  aa::VisionAlign controller(s,io,&cache);ASSERT_NO_THROW(controller.run());
  EXPECT_EQ(controller.counts()[0][0].learning,3);EXPECT_EQ(controller.counts()[0][1].learning,3);
  EXPECT_EQ(controller.counts()[1][0].learning,0);EXPECT_EQ(controller.counts()[1][1].learning,0);
}
TEST(VisionCache, SmallUnobservableResponseDoesNotValidateSavedMapping) {
  CacheFile file;auto s=settings();s.deadline=600;cold_save(s,file.path);
  Scene io;io.position_error=.002;io.angle_error=.02;
  aa::VisionCache cache(file.path,"test-geometry",s);cache.load();
  aa::VisionAlign controller(s,io,&cache);ASSERT_NO_THROW(controller.run());
  for(const auto& side:controller.counts()) for(const auto& kind:side) {
    EXPECT_EQ(kind.rejected,1);EXPECT_EQ(kind.learning,3);EXPECT_EQ(kind.reused,0);
  }
}
TEST(VisionCache, FailurePreservesLastGoodFileAndSafetyStopsBeforeRelearning) {
  CacheFile file;auto s=settings();s.deadline=600;cold_save(s,file.path);
  std::ifstream input(file.path);const std::string original((std::istreambuf_iterator<char>(input)),{});
  Scene io;io.dropout=true;aa::VisionCache cache(file.path,"test-geometry",s);cache.load();
  aa::VisionAlign controller(s,io,&cache);EXPECT_THROW(controller.run(),std::runtime_error);
  EXPECT_EQ(io.commands,1);EXPECT_EQ(controller.counts()[0][1].learning,0);
  std::ifstream after(file.path);EXPECT_EQ(std::string((std::istreambuf_iterator<char>(after)),{}),original);
  Scene limited;auto tight=s;tight.max_travel=.001;aa::VisionCache empty(file.directory/"never.yaml","test-geometry",tight);
  aa::VisionAlign bounded(tight,limited,&empty);EXPECT_THROW(bounded.run(),std::runtime_error);
  EXPECT_FALSE(std::filesystem::exists(file.directory/"never.yaml"));
}
TEST(VisionCache, CorruptIncompatibleExpiredAndForceRelearnUseColdPath) {
  CacheFile file;auto s=settings();s.deadline=600;cold_save(s,file.path);
  aa::VisionCache different(file.path,"changed-context",s);different.load();
  for(const auto& side:different.entries()) for(const auto& e:side) EXPECT_FALSE(e);
  aa::VisionCache forced(file.path,"test-geometry",s);EXPECT_EQ(forced.load(true),"CACHE: force_relearn");
  for(const auto& side:forced.entries()) for(const auto& e:side) EXPECT_FALSE(e);
  aa::VisionCache edit(file.path,"test-geometry",s);edit.load();auto entries=edit.entries();
  entries[0][0]->saved_at-=s.cache_max_age+1;edit.save(entries);
  aa::VisionCache expired(file.path,"test-geometry",s);expired.load();
  EXPECT_FALSE(expired.entries()[0][0]);EXPECT_TRUE(expired.entries()[0][1]);
  std::ofstream(file.path)<<"{broken yaml";
  aa::VisionCache corrupt(file.path,"test-geometry",s);EXPECT_NO_THROW(corrupt.load());
  for(const auto& side:corrupt.entries()) for(const auto& e:side) EXPECT_FALSE(e);
}
TEST(VisionCache, InvalidCandidateCannotOverwriteGoodFileAndDisabledCacheDoesNotWrite) {
  CacheFile file;auto s=settings();s.deadline=600;cold_save(s,file.path);
  aa::VisionCache cache(file.path,"test-geometry",s);cache.load();auto entries=cache.entries();
  entries[0][0]->jacobian.setZero();EXPECT_THROW(cache.save(entries),std::runtime_error);
  aa::VisionCache good(file.path,"test-geometry",s);good.load();EXPECT_TRUE(good.entries()[0][0]);
  s.cache_enabled=false;aa::VisionCache disabled(file.directory/"disabled.yaml","test-geometry",s);
  Scene io;aa::VisionAlign controller(s,io,&disabled);ASSERT_NO_THROW(controller.run());
  EXPECT_FALSE(std::filesystem::exists(file.directory/"disabled.yaml"));
}
TEST(VisionCache, PartialAndEmptyFilesRetainOnlyAvailableMappings) {
  CacheFile file;auto s=settings();s.deadline=600;cold_save(s,file.path);
  aa::VisionCache edit(file.path,"test-geometry",s);edit.load();auto entries=edit.entries();
  entries[0][1].reset();entries[1][0].reset();entries[1][1].reset();edit.save(entries);
  aa::VisionCache partial(file.path,"test-geometry",s);
  EXPECT_NE(partial.load().find("loaded=1 rejected=0"),std::string::npos);
  EXPECT_TRUE(partial.entries()[0][0]);EXPECT_FALSE(partial.entries()[0][1]);
  EXPECT_FALSE(partial.entries()[1][0]);EXPECT_FALSE(partial.entries()[1][1]);
  edit.save({});aa::VisionCache empty(file.path,"test-geometry",s);
  EXPECT_NE(empty.load().find("loaded=0 rejected=0"),std::string::npos);
}
TEST(VisionCache, AlreadyAlignedRunDoesNotProbeOrRefreshUnusedMappings) {
  CacheFile file;auto s=settings();s.deadline=600;cold_save(s,file.path);
  aa::VisionCache cache(file.path,"test-geometry",s);cache.load();
  const double saved_at=cache.entries()[0][0]->saved_at;
  Scene io;io.state=sample(1);aa::VisionAlign controller(s,io,&cache);ASSERT_NO_THROW(controller.run());
  EXPECT_EQ(io.commands,0);EXPECT_DOUBLE_EQ(cache.entries()[0][0]->saved_at,saved_at);
}
