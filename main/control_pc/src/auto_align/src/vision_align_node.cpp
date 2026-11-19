#include "auto_align/vision_align.hpp"
#include "auto_align/steps/pre_align_ready.hpp"
#include "xarm_motion/xarm_backend.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <poll.h>
#include <unistd.h>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <sstream>
#include <thread>

namespace auto_align {
namespace {
volatile std::sig_atomic_t interrupted=0;
void signal_handler(int) { interrupted=1; }
void require(bool ok,const std::string& text) { if(!ok) throw std::runtime_error(text); }
std::string read_file(const std::filesystem::path& path) {
  std::ifstream file(path,std::ios::binary); require(file.good(),"설정 읽기 실패: "+path.string());
  std::ostringstream out; out<<file.rdbuf(); return out.str();
}
class VisionRuntime final : public rclcpp::Node,public VisionIO {
public:
  VisionRuntime(VisionSettings settings,YAML::Node frames,bool readonly):Node(readonly?"vision_measure":"auto_align"),s_(std::move(settings)),frames_(std::move(frames)) {
    status_=create_publisher<std_msgs::msg::String>(readonly?"/auto_align/vision_status":"/auto_align/status",rclcpp::QoS(1).transient_local());
    diagnostics_=create_publisher<std_msgs::msg::String>("/auto_align/vision_diagnostics",rclcpp::QoS(1).transient_local());
    markers_=create_publisher<visualization_msgs::msg::MarkerArray>("/auto_align/vision_markers",1);
    joints_=create_publisher<sensor_msgs::msg::JointState>("/auto_align/joint_states",rclcpp::SensorDataQoS());
    for(size_t i=0;i<2;++i) {
      subs_[i]=create_subscription<geometry_msgs::msg::PoseStamped>(s_.topics[i],rclcpp::SensorDataQoS().keep_last(1),
        [this,i](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) { receive(i,*m); });
      poses_[i]=create_publisher<geometry_msgs::msg::PoseStamped>(std::string("/auto_align/")+(i==0?"left":"right")+"/base_flange_pose",rclcpp::SensorDataQoS());
    }
  }
  void attach(std::array<xm::XArmBackend*,2> backends,std::array<xm::ArmKinematics*,2> models,CollisionChecker* checker,xm::Limits limits) {
    robots_=backends; models_=models; checker_=checker; limits_=limits;
    limits_.linear_speed=std::min(limits_.linear_speed,s_.linear_speed);
    limits_.angular_speed=std::min(limits_.angular_speed,s_.angular_speed);
    limits_.arrival_position=std::min(limits_.arrival_position,.0005);
    limits_.arrival_orientation=std::min(limits_.arrival_orientation,.003);
    limits_.arrival_joint=std::min(limits_.arrival_joint,.001);
    for(size_t i=0;i<2;++i) initial_tcp_[i]=robots_[i]->read().base_T_tcp;
  }
  bool cancelled() { service(); return interrupted || !rclcpp::ok(); }
  void service() {
    if(servicing_) return;
    servicing_=true;
    try {
      rclcpp::spin_some(get_node_base_interface());
      require(!interrupted && rclcpp::ok(),"사용자가 비전 정렬을 중지했습니다.");
      const double t=xm::steady_seconds();
      if(watching_) {
        require(latest_.has_value() && t-latest_->received<=s_.pair_timeout,"좌우 비전 쌍 수신이 중단됐습니다.");
        require(now().seconds()-latest_->stamp*1e-9<=s_.max_age,"비전 영상이 오래됐습니다. PC 시각 동기화·추론 지연을 확인하세요.");
        require(elapsed()<=s_.deadline,"UVS 전체 시간 상한을 넘었습니다.");
      }
      if(robots_[0] && !executing_ && t-last_feedback_>=.05) {
        publish_feedback({robots_[0]->read(),robots_[1]->read()}); last_feedback_=t;
      }
      servicing_=false;
    } catch(...) { servicing_=false; throw; }
  }
  void report(const std::string& text) override {
    const std::string full=(text.rfind("READY",0)==0 || text.rfind("COMPLETED",0)==0 || text.rfind("FAULT",0)==0 || text.rfind("STOPPED",0)==0)?text:"RUNNING: "+text;
    std_msgs::msg::String m; m.data=full; status_->publish(m); RCLCPP_INFO(get_logger(),"%s",full.c_str());
  }
  double elapsed() const override { return begun_==0?0:xm::steady_seconds()-begun_; }
  void publish_stopped_feedback() {
    // Only called by the start-path validator, before either backend is enabled.
    // SDK read() also consults cancelled(); prevent recursive service/read calls.
    const bool previous=servicing_;servicing_=true;
    try {
      publish_feedback({robots_[0]->read(),robots_[1]->read()});
      last_feedback_=xm::steady_seconds();servicing_=previous;
    } catch(...) { servicing_=previous;throw; }
  }
  void start_control() { begun_=xm::steady_seconds(); cutoff_=now().nanoseconds()+static_cast<int64_t>(s_.settle*1e9); }
  VisionPair measure() override {
    const double started=xm::steady_seconds(); std::vector<VisionPair> samples;
    int64_t consumed=last_measurement_;
    while(true) {
      service();
      require(xm::steady_seconds()-started<=s_.sample_timeout,"새 정지 영상의 좌우 쌍을 충분히 받지 못했습니다.");
      if(latest_ && latest_->stamp>std::max(cutoff_,consumed) && now().seconds()-latest_->stamp*1e-9<=s_.max_age) {
        samples.push_back(*latest_); consumed=latest_->stamp;
        if(samples.size()>=static_cast<size_t>(s_.min_samples) && (samples.back().stamp-samples.front().stamp)*1e-9>=s_.sample_window) {
          auto pair=average_pairs(samples,s_); last_measurement_=pair.stamp;
          if(begun_!=0) watching_=true;
          if(!baseline_ && begun_!=0) baseline_=pair;
          diagnostic(pair); return pair;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
  void ready() {
    report("READY: vision_align step4~7 / 좌우 CAD 끝단·camera 축 표시를 확인한 뒤 Enter / 작은 시험 이동이 시작됩니다.");
    while(true) {
      service(); pollfd fd{STDIN_FILENO,POLLIN,0}; const int n=poll(&fd,1,0);
      require(n>=0 && !(fd.revents&(POLLERR|POLLHUP|POLLNVAL)),"터미널 입력이 종료됐습니다.");
      if(n>0 && (fd.revents&POLLIN)) { std::string line; require(bool(std::getline(std::cin,line)),"입력 종료"); if(line.empty()) return; }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
  Eigen::Vector3d move(size_t selected,const Eigen::Vector3d& delta,bool rotation) override {
    require(robots_[0] && checker_ && delta.allFinite() && delta.norm()<=
      std::max(rotation?s_.probe_rotation:s_.probe_translation,rotation?s_.max_rotation:s_.max_translation)*1.1,"보정 명령 범위가 유효하지 않습니다.");
    service();
    std::array<xm::Feedback,2> before{robots_[0]->read(),robots_[1]->read()};
    std::vector<xm::MotionArm> arms;
    for(size_t i=0;i<2;++i) {
      require(before[i].velocities.cwiseAbs().maxCoeff()<=limits_.stationary_speed,"정지 후에만 새 보정 경로를 계산합니다.");
      const auto tcp=xm::tcp_transform(frames_,i==0?"left":"right");
      xm::Pose target=before[i].base_T_tcp;
      if(i==selected) {
        if(rotation) target.linear()=Eigen::AngleAxisd(delta.norm(),delta.normalized()).toRotationMatrix()*target.linear();
        else target.translation()+=delta;
      }
      require((target.translation()-initial_tcp_[i].translation()).norm()<=s_.max_travel &&
        xm::rotation_error(target,initial_tcp_[i])<=s_.max_turn,"TCP 누적 보정 범위를 넘었습니다.");
      xm::SdkKinematics kinematics(*robots_[i],*models_[i],tcp);
      auto plan=std::make_unique<xm::MotionPlan>(kinematics,before[i].joints,target*tcp.inverse(),tcp,limits_);
      arms.push_back({i==0?"left":"right",robots_[i],models_[i],tcp,before[i].base_T_tcp*tcp.inverse(),target*tcp.inverse(),before[i],std::move(plan)});
    }
    const double duration=std::max(arms[0].plan->duration(),arms[1].plan->duration());
    checker_->check_path([&](double t)->DualJoints {
      return {arms[0].plan->sample(t/duration),arms[1].plan->sample(t/duration)};
    },duration,{arms[0].plan->joint_speed_bound(),arms[1].plan->joint_speed_bound()},[this]{return cancelled();});
    xm::MotionSession session(std::move(arms),limits_,[this]{return cancelled();});
    session.set_start_path_validator([&](const std::vector<xm::MotionArm>& connected,double seconds) {
      report("START_CONNECTION: 실측 자세에서 기존 경로로 연결 / 충돌 경로 재검사");
      double published=0;
      checker_->check_path([&](double t)->DualJoints {
        return {connected[0].plan->sample(t/seconds),connected[1].plan->sample(t/seconds)};
      },seconds,{connected[0].plan->joint_speed_bound(),connected[1].plan->joint_speed_bound()},[&] {
        if(cancelled()) return true;
        if(xm::steady_seconds()-published>=.05) {publish_stopped_feedback();published=xm::steady_seconds();}
        return false;
      });
    });
    session.prepare_planned(xm::steady_seconds()); service();
    executing_=true;
    try {
      session.begin(xm::steady_seconds());
      if(session.start_connection_duration()>0) report("START_CONNECTION: "+session.diagnostics());
      auto next=std::chrono::steady_clock::now();
      while(true) {
        service();
        next+=std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(limits_.period()));
        std::this_thread::sleep_until(next);
        const bool done=session.tick(xm::steady_seconds());
        if(xm::steady_seconds()-last_feedback_>=.05 || done) { publish_feedback(session.last_feedback()); last_feedback_=xm::steady_seconds(); }
        if(done) break;
        next=std::max(next,std::chrono::steady_clock::now());
      }
      session.stop(); executing_=false;
    } catch(...) { executing_=false; throw; }
    const auto after=robots_[selected]->read();
    cutoff_=now().nanoseconds()+static_cast<int64_t>(s_.settle*1e9);
    if(rotation) {
      Eigen::AngleAxisd aa(after.base_T_tcp.linear()*before[selected].base_T_tcp.linear().transpose());
      return aa.angle()*aa.axis();
    }
    return after.base_T_tcp.translation()-before[selected].base_T_tcp.translation();
  }
private:
  void receive(size_t side,const geometry_msgs::msg::PoseStamped& m) {
    require(m.header.frame_id==s_.frame,"비전 frame_id가 설정과 다릅니다.");
    const int64_t stamp=static_cast<int64_t>(m.header.stamp.sec)*1000000000LL+m.header.stamp.nanosec;
    const double age=now().seconds()-stamp*1e-9;
    if(stamp<=0 || m.header.stamp.nanosec>=1000000000U || age<-.05 || age>s_.max_age) return;
    const auto& p=m.pose.position; const auto& q=m.pose.orientation;
    Eigen::Quaterniond quat(q.w,q.x,q.y,q.z);
    require(quat.coeffs().allFinite() && std::abs(quat.norm()-1)<.001,"비전 quaternion이 유효하지 않습니다.");
    xm::Pose pose=xm::Pose::Identity(); pose.linear()=quat.normalized().toRotationMatrix(); pose.translation()=Eigen::Vector3d(p.x,p.y,p.z);
    pairs_.add(side,stamp,xm::steady_seconds(),pipe_observation(pose,s_.endpoint_z[side]));
    if(auto pair=pairs_.take()) {
      if(baseline_) {
        const double old_dx=baseline_->pipes[1].endpoint.x()-baseline_->pipes[0].endpoint.x();
        const double dx=pair->pipes[1].endpoint.x()-pair->pipes[0].endpoint.x();
        require(dx*old_dx>0 && std::abs(dx)>=s_.min_gap && std::abs(dx-old_dx)<=s_.gap_drift,"비전 감시: 끝단 간격이 허용 범위를 벗어났습니다.");
        for(size_t i=0;i<2;++i) require((pair->pipes[i].endpoint-baseline_->pipes[i].endpoint).norm()<=s_.max_travel &&
          pair->pipes[i].axis.dot(baseline_->pipes[i].axis)>=std::cos(s_.max_turn),"비전 감시: 축 부호·누적 보정 범위가 바뀌었습니다.");
      }
      latest_=*pair;
    }
  }
  void diagnostic(const VisionPair& pair) {
    std::ostringstream out; out<<std::setprecision(12)<<"stamp="<<pair.stamp<<" frame_id="<<s_.frame;
    visualization_msgs::msg::MarkerArray markers;
    auto marker=[&](int id,const Eigen::Vector3d& p,const Eigen::Vector3d& end,bool target) {
      visualization_msgs::msg::Marker m; m.header.frame_id=s_.frame; m.header.stamp=now(); m.ns="vision_axes";m.id=id;
      m.type=visualization_msgs::msg::Marker::ARROW; m.action=m.ADD; m.pose.orientation.w=1;
      m.scale.x=.003; m.scale.y=.007; m.scale.z=.012; m.color.a=1;
      m.color.r=target?1:0; m.color.g=id==0 || target?1:0; m.color.b=id==1?1:0;
      geometry_msgs::msg::Point a,b; a.x=p.x();a.y=p.y();a.z=p.z();b.x=end.x();b.y=end.y();b.z=end.z();m.points={a,b};
      markers.markers.push_back(m);
    };
    for(size_t i=0;i<2;++i) {
      const auto& p=pair.pipes[i];
      // Match the controller's initial +/-X convention without changing its state.
      const double sign=(baseline_?baseline_->pipes[i].axis.x():p.axis.x())>=0?1:-1;
      const Eigen::Vector3d a=sign*p.axis;
      out<<" / "<<(i==0?"left":"right")<<" endpoint_m=["<<p.endpoint.transpose()<<"] CAD_z="<<s_.endpoint_z[i]
         <<" axis_X_deg="<<std::acos(std::clamp(std::abs(p.axis.x()),0.,1.))*180/std::acos(-1.)
         <<" axis_camera=["<<p.axis.transpose()<<"] axis_sign="<<sign
         <<" angle_xy_deg="<<std::atan2(a.y(),a.x())*180/std::acos(-1.)
         <<" angle_xz_deg="<<std::atan2(a.z(),a.x())*180/std::acos(-1.);
      marker(i,p.endpoint,p.endpoint+.1*p.axis,false);
    }
    const Eigen::Vector3d center=baseline_?(baseline_->pipes[0].endpoint+baseline_->pipes[1].endpoint)/2:
      (pair.pipes[0].endpoint+pair.pipes[1].endpoint)/2;
    marker(2,center-.2*Eigen::Vector3d::UnitX(),center+.2*Eigen::Vector3d::UnitX(),true);
    out<<" / goal_yz_m="<<center.tail<2>().transpose()
       <<" / relative_angle_deg="<<std::acos(std::clamp(std::abs(pair.pipes[0].axis.dot(pair.pipes[1].axis)),0.,1.))*180/std::acos(-1.)
       <<" / gap_x_mm="<<std::abs(pair.pipes[1].endpoint.x()-pair.pipes[0].endpoint.x())*1000
       <<" / source_age_s="<<now().seconds()-pair.stamp*1e-9
       <<" / delta_yz_mm="<<((pair.pipes[1].endpoint-pair.pipes[0].endpoint).tail<2>()*1000).transpose();
    std_msgs::msg::String m;m.data=out.str();diagnostics_->publish(m);markers_->publish(markers);
    RCLCPP_INFO(get_logger(),"VISION: %s",m.data.c_str());
  }
  void publish_feedback(const std::vector<xm::Feedback>& values) {
    sensor_msgs::msg::JointState msg;msg.header.stamp=now();
    for(size_t i=0;i<2;++i) {
      const std::string side=i==0?"left":"right";
      for(int j=0;j<7;++j) { msg.name.push_back(side+"_joint"+std::to_string(j+1));msg.position.push_back(values[i].joints[j]);msg.velocity.push_back(values[i].velocities[j]); }
      const auto flange=values[i].base_T_tcp*xm::tcp_transform(frames_,side).inverse(); Eigen::Quaterniond q(flange.linear());
      geometry_msgs::msg::PoseStamped pose;pose.header.stamp=msg.header.stamp;pose.header.frame_id=frames_["frames"][side+"_base"].as<std::string>();
      pose.pose.position.x=flange.translation().x();pose.pose.position.y=flange.translation().y();pose.pose.position.z=flange.translation().z();
      pose.pose.orientation.x=q.x();pose.pose.orientation.y=q.y();pose.pose.orientation.z=q.z();pose.pose.orientation.w=q.w();poses_[i]->publish(pose);
    }
    joints_->publish(msg);
  }
  VisionSettings s_;YAML::Node frames_;VisionPairs pairs_;std::optional<VisionPair> latest_,baseline_;
  int64_t last_measurement_=0,cutoff_=0;double begun_=0,last_feedback_=0;bool watching_=false,servicing_=false,executing_=false;
  std::array<xm::XArmBackend*,2> robots_{};std::array<xm::ArmKinematics*,2> models_{};
  std::array<xm::Pose,2> initial_tcp_;CollisionChecker* checker_=nullptr;xm::Limits limits_{};
  std::array<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr,2> subs_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_,diagnostics_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_;
  std::array<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr,2> poses_;
};
}
int vision_align_main(int argc,char** argv) {
  std::shared_ptr<VisionRuntime> node;
  std::array<std::unique_ptr<xm::XArmBackend>,2> backends;
  int result=0;
  try {
    require((argc==3 || argc==5) && (argc==3 || std::string(argv[3])=="--expected-inputs"),"--step vision_measure 또는 vision_align을 사용하세요.");
    const bool readonly=std::string(argv[2])=="vision_measure";
    const auto root=std::filesystem::path(AUTO_ALIGN_CONTROL_PC_DIR);
    const auto control_text=read_file(root/"config/control.yaml");const auto control=YAML::Load(control_text);
    const auto settings=vision_settings(control);
    if(!readonly) { require_execution_enabled(control);require_vision_enabled(control);require(isatty(STDIN_FILENO),"Enter·Ctrl+C 터미널이 필요합니다."); }
    if(argc==5) require(input_fingerprint(control_text,read_file(root/"config/arm_poses.yaml"))==argv[4],"시각화 준비 중 설정이 변경됐습니다.");
    const auto frames=YAML::LoadFile((root/"config/fixed_frames.yaml").string());
    struct sigaction action{};action.sa_handler=signal_handler;sigemptyset(&action.sa_mask);
    sigaction(SIGINT,&action,nullptr);sigaction(SIGTERM,&action,nullptr);
    const auto logs=root/"_log/auto_align";std::filesystem::create_directories(logs);setenv("ROS_LOG_DIR",logs.c_str(),1);
    rclcpp::init(0,nullptr,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
    node=std::make_shared<VisionRuntime>(settings,frames,readonly);
    node->report("STEP4: 좌우 비전 수신·끝단 표시 / 로봇 이동 없음");node->measure();
    if(readonly) { while(!interrupted && rclcpp::ok()) node->measure(); }
    else {
      const auto config=xm::read_config(control);
      const auto share=ament_index_cpp::get_package_share_directory("xarm_motion");
      std::array<std::string,2> urdfs;std::array<std::unique_ptr<xm::ArmKinematics>,2> models;
      for(size_t i=0;i<2;++i) {
        urdfs[i]=share+"/models/xarm7_"+(config.model1300[i]?"true":"false")+".urdf";
        models[i]=std::make_unique<xm::ArmKinematics>(urdfs[i]);
        backends[i]=std::make_unique<xm::XArmBackend>(config.ips[i],xm::tcp_transform(frames,i==0?"left":"right"),
          config.limits.feedback_timeout,[&]{return node->cancelled();},(root/"_colcon/locks/xarm_motion").string(),config.limits.report_timeout);
      }
      const auto environment=YAML::LoadFile(ament_index_cpp::get_package_share_directory("simulation_validation")+"/config/environment_models.yaml");
      require(environment["frame_id"].as<std::string>()==frames["frames"]["base"].as<std::string>(),"환경과 팔의 world 이름이 다릅니다.");
      CollisionChecker checker(urdfs,{mounting_transform(frames,"left"),mounting_transform(frames,"right")},environment_boxes(environment),collision_settings(control));
      checker.check({backends[0]->read().joints,backends[1]->read().joints});
      node->attach({backends[0].get(),backends[1].get()},{models[0].get(),models[1].get()},&checker,config.limits);
      node->ready();node->start_control();VisionAlign controller(settings,*node);controller.run();
      node->report("COMPLETED: vision_align / step7 비전 정렬 유지 확인");
    }
  } catch(const std::exception& e) {
    if(node) node->report(std::string(interrupted?"STOPPED: ":"FAULT: ")+e.what());
    else std::cerr<<"[경고] "<<e.what()<<'\n';
    result=interrupted?130:1;
  }
  // Disconnect before destroying the node captured by backend cancellation hooks.
  for(auto& b:backends) b.reset();
    node.reset();if(rclcpp::ok()) rclcpp::shutdown();return result;
}
} // namespace auto_align
