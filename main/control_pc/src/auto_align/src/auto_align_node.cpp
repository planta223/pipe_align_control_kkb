#include "auto_align/steps/pre_align_ready.hpp"
#include "xarm_motion/xarm_backend.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <poll.h>
#include <unistd.h>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>

namespace aa = auto_align;
namespace xm = xarm_motion;
namespace {
volatile std::sig_atomic_t interrupted = 0;
void handle_signal(int) { interrupted = 1; }
constexpr const char* usage =
  "사용법: ros2 run auto_align auto_align --step pre_align_ready\n"
  "control.yaml의 execution.auto_align_enabled=true가 필요합니다. pre_align_ready.target_pose를 목표로 사용합니다.\n";
void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}
std::string read_file(const std::filesystem::path& path) {
  std::ifstream file(path,std::ios::binary);
  require(file.good(),"설정 파일을 읽을 수 없습니다: "+path.string());
  std::ostringstream text;
  text << file.rdbuf();
  require(!file.bad(),"설정 파일 읽기 실패: "+path.string());
  return text.str();
}
bool enter_pressed() {
  pollfd fd{STDIN_FILENO,POLLIN,0};
  const int result = poll(&fd,1,0);
  if (result < 0 && !interrupted) throw std::runtime_error("터미널 입력 확인 실패");
  if (result > 0 && (fd.revents & (POLLERR|POLLHUP|POLLNVAL)))
    throw std::runtime_error("터미널 입력이 종료됐습니다.");
  if (result > 0 && (fd.revents & POLLIN)) {
    std::string line;
    if (!std::getline(std::cin,line)) throw std::runtime_error("터미널 입력이 종료됐습니다.");
    return line.empty();
  }
  return false;
}
aa::DualJoints joints(const std::vector<xm::Feedback>& feedback) {
  require(feedback.size() == 2,"양팔 피드백이 필요합니다.");
  return {feedback[0].joints,feedback[1].joints};
}
class Monitor : public rclcpp::Node {
public:
  explicit Monitor(const YAML::Node& frames) : Node("auto_align",rclcpp::NodeOptions().use_global_arguments(false)), frames_(frames) {
    joints_ = create_publisher<sensor_msgs::msg::JointState>("/auto_align/joint_states",rclcpp::SensorDataQoS());
    status_ = create_publisher<std_msgs::msg::String>("/auto_align/status",rclcpp::QoS(1).transient_local());
    for (const auto& side : {"left","right"})
      poses_.push_back(create_publisher<geometry_msgs::msg::PoseStamped>(
        std::string("/auto_align/")+side+"/base_flange_pose",rclcpp::SensorDataQoS()));
  }
  void status(const std::string& text) {
    std_msgs::msg::String msg; msg.data = text; status_->publish(msg);
    RCLCPP_INFO(get_logger(),"%s",text.c_str());
  }
  void diagnostics(const std::string& text) { RCLCPP_INFO(get_logger(),"MEASURE: %s",text.c_str()); }
  void feedback(const std::vector<xm::MotionArm>& arms, const std::vector<xm::Feedback>& values) {
    sensor_msgs::msg::JointState msg; msg.header.stamp = now();
    for (size_t side = 0; side < arms.size(); ++side) {
      for (int joint = 0; joint < 7; ++joint) {
        msg.name.push_back(arms[side].side+"_joint"+std::to_string(joint+1));
        msg.position.push_back(values[side].joints[joint]); msg.velocity.push_back(values[side].velocities[joint]);
      }
      const auto flange = values[side].base_T_tcp*arms[side].flange_T_tcp.inverse();
      const Eigen::Quaterniond q(flange.linear());
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = msg.header.stamp;
      pose.header.frame_id = frames_["frames"][arms[side].side+"_base"].as<std::string>();
      pose.pose.position.x = flange.translation().x(); pose.pose.position.y = flange.translation().y();
      pose.pose.position.z = flange.translation().z(); pose.pose.orientation.x = q.x(); pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z(); pose.pose.orientation.w = q.w();
      poses_[side]->publish(pose);
    }
    joints_->publish(msg);
  }
private:
  YAML::Node frames_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> poses_;
};
}  // namespace

int main(int argc, char** argv) {
  if (argc == 2 && std::string(argv[1]) == "--help") { std::cout << usage; return 0; }
  std::shared_ptr<Monitor> monitor;
  // SDK 연결 해제 대기 전에 종료 원인을 발행한다.
  std::vector<std::unique_ptr<xm::XArmBackend>> backends;
  int result = 0;
  try {
    require((argc == 3 || argc == 5) && std::string(argv[1]) == "--step" &&
      std::string(argv[2]) == "pre_align_ready" &&
      (argc == 3 || std::string(argv[3]) == "--expected-inputs"),usage);
    const auto root = std::filesystem::path(AUTO_ALIGN_CONTROL_PC_DIR);
    const auto control_text = read_file(root/"config/control.yaml");
    const auto control = YAML::Load(control_text);
    aa::require_execution_enabled(control); // SDK 생성·접속보다 먼저 파일 허용값을 검사한다.
    const auto coordinates_text = read_file(root/"config/arm_poses.yaml");
    if (argc == 5)
      require(aa::input_fingerprint(control_text,coordinates_text) == argv[4],
        "시각화 준비 중 control.yaml 또는 arm_poses.yaml이 바뀌었습니다. 다시 실행하세요.");
    const auto coordinates = YAML::Load(coordinates_text);
    const auto target = aa::ready_target(control,coordinates);
    const auto config = xm::read_config(control);
    const auto collision_config = aa::collision_settings(control);
    require(isatty(STDIN_FILENO),"Enter·Ctrl+C 입력이 가능한 터미널에서 실행하세요.");
    const auto frames = YAML::LoadFile((root/"config/fixed_frames.yaml").string());
    const std::string visualization_share = ament_index_cpp::get_package_share_directory("simulation_validation");
    const auto environment = YAML::LoadFile(visualization_share+"/config/environment_models.yaml");
    require(environment["frame_id"].as<std::string>() == frames["frames"]["base"].as<std::string>(),
      "환경과 로봇의 world 좌표계 이름이 다릅니다.");
    const std::array<xm::Pose,2> mounts{aa::mounting_transform(frames,"left"),aa::mounting_transform(frames,"right")};
    const std::string model_share = ament_index_cpp::get_package_share_directory("xarm_motion");
    std::array<std::string,2> urdfs;
    for (size_t i = 0; i < 2; ++i)
      urdfs[i] = model_share+"/models/xarm7_"+(config.model1300[i]?"true":"false")+".urdf";
    aa::CollisionChecker checker(urdfs,mounts,aa::environment_boxes(environment),collision_config);
    std::array<std::unique_ptr<xm::ArmKinematics>,2> models;
    for (size_t i = 0; i < 2; ++i) models[i] = std::make_unique<xm::ArmKinematics>(urdfs[i]);
    struct sigaction action{}; action.sa_handler = handle_signal; sigemptyset(&action.sa_mask);
    sigaction(SIGINT,&action,nullptr); sigaction(SIGTERM,&action,nullptr);
    const auto logs = root/"_log/auto_align";
    std::filesystem::create_directories(logs);
    require(setenv("ROS_LOG_DIR",logs.c_str(),1) == 0,"ROS 로그 경로 설정 실패");
    rclcpp::init(0,nullptr,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
    monitor = std::make_shared<Monitor>(frames);
    const auto cancelled = [] { return interrupted != 0 || !rclcpp::ok(); };
    std::vector<xm::MotionArm> arms;
    monitor->status("PREPARING: pre_align_ready / 목표 "+target.name+" / 현재 자세 조회");
    for (size_t i = 0; i < 2; ++i) {
      const std::string side = i == 0 ? "left" : "right";
      const auto tcp = xm::tcp_transform(frames,side);
      backends.push_back(std::make_unique<xm::XArmBackend>(config.ips[i],tcp,config.limits.feedback_timeout,
        cancelled,(root/"_colcon/locks/xarm_motion").string(),config.limits.report_timeout));
      const auto actual = backends.back()->read();
      models[i]->check(actual.joints);
      require(actual.velocities.allFinite() && actual.velocities.cwiseAbs().maxCoeff() <= config.limits.stationary_speed,
        side+": 로봇이 정지 상태가 아닙니다.");
      arms.push_back({side,backends.back().get(),models[i].get(),tcp,actual.base_T_tcp*tcp.inverse(),
        target.flange[i],actual,nullptr});
    }
    checker.check({arms[0].prepared.joints,arms[1].prepared.joints});
    monitor->status("PREPARING: vertical → horizontal → orientation / 전체 SDK 경로 계산");
    aa::PreAlignReady step(arms,mounts,config.limits,cancelled);
    const double duration = arms[0].plan->duration();
    require(std::abs(duration-arms[1].plan->duration()) < 1e-9,"양팔 경로 시간이 다릅니다.");
    monitor->status("PREPARING: 전체 경로와 구간 사이 충돌 여유거리 검사");
    checker.check_path([&](double time) -> aa::DualJoints {
      return {arms[0].plan->sample(time/duration),arms[1].plan->sample(time/duration)};
    },duration,{arms[0].plan->joint_speed_bound(),arms[1].plan->joint_speed_bound()},cancelled);
    xm::MotionSession session(std::move(arms),config.limits,cancelled);
    session.prepare_planned(xm::steady_seconds());
    monitor->status("READY: pre_align_ready / 목표 "+target.name+" / 충돌 여유 "+std::to_string(collision_config.margin*1000)+
      "mm / 예상 "+std::to_string(session.duration())+"초 / Enter로 실행");
    monitor->diagnostics(session.diagnostics());
    while (!cancelled()) {
      monitor->feedback(session.arms(),session.feedback(xm::steady_seconds()));
      rclcpp::spin_some(monitor);
      if (enter_pressed()) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    require(!cancelled(),"실행이 취소됐습니다.");
    checker.check(joints(session.feedback(xm::steady_seconds())));
    session.begin(xm::steady_seconds());
    auto next = std::chrono::steady_clock::now();
    double published_at = 0, diagnostic_at = 0;
    std::string phase;
    bool complete = false;
    while (!cancelled()) {
      next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(config.limits.period()));
      std::this_thread::sleep_until(next);
      const double now = xm::steady_seconds();
      if (session.tick(now)) { complete = true; monitor->diagnostics(session.diagnostics()); break; }
      const auto current_phase = step.phase(session.path_elapsed());
      if (phase != current_phase) { phase = current_phase; monitor->status("RUNNING: pre_align_ready / "+phase); }
      if (now-diagnostic_at >= 1.0) { monitor->diagnostics(session.diagnostics()); diagnostic_at = now; }
      if (now-published_at >= 0.05) { monitor->feedback(session.arms(),session.last_feedback()); published_at = now; }
      rclcpp::spin_some(monitor);
      next = std::max(next,std::chrono::steady_clock::now());
    }
    session.stop();
    if (complete) {
      monitor->feedback(session.arms(),session.last_feedback());
      monitor->status("COMPLETED: pre_align_ready / 실측 도착 확인 후 중지·종료");
    } else { monitor->status("STOPPED: pre_align_ready / 사용자 중지"); result = 130; }
  } catch (const std::exception& error) {
    std::cerr << "[경고] " << error.what() << '\n';
    if (monitor) monitor->status(std::string(interrupted?"STOPPED: ":"FAULT: ")+error.what());
    result = interrupted ? 130 : 1;
  }
  backends.clear();
  if (rclcpp::ok()) rclcpp::shutdown();
  return result;
}
