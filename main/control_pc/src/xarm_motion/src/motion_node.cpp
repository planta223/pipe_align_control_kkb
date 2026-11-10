#include "xarm_motion/motion_core.hpp"
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
#include <iostream>
#include <map>
#include <thread>

namespace xm = xarm_motion;
namespace {
volatile std::sig_atomic_t interrupted = 0;
void handle_signal(int) { interrupted = 1; }
const char* usage =
  "사용법: ros2 run xarm_motion xarm_motion --arm left|right|both "
  "--start-pose NAME --target-pose NAME\n"
  "control.yaml의 standalone_enabled=true가 필요합니다. Enter로 실행, Ctrl+C로 중지합니다.\n";
std::map<std::string,std::string> arguments(int argc, char** argv) {
  std::map<std::string,std::string> args;
  for (int i = 1; i < argc; i += 2) {
    const std::string key = argv[i];
    if ((key != "--arm" && key != "--start-pose" && key != "--target-pose") ||
        i+1 >= argc || args.count(key) || std::string(argv[i+1]).empty())
      throw std::runtime_error("지원하지 않거나 누락·중복된 실행 인자입니다.\n"+std::string(usage));
    args[key] = argv[i+1];
  }
  if (args.size() != 3 || (args["--arm"] != "left" && args["--arm"] != "right" && args["--arm"] != "both"))
    throw std::runtime_error(usage);
  return args;
}
bool enter_pressed() {
  pollfd fd{STDIN_FILENO,POLLIN,0};
  const int result = poll(&fd,1,0);
  if (result < 0 && !interrupted) throw std::runtime_error("터미널 입력 확인 실패");
  if (result > 0 && (fd.revents & (POLLERR|POLLHUP|POLLNVAL)))
    throw std::runtime_error("터미널 입력이 종료되었습니다.");
  if (result > 0 && (fd.revents & POLLIN)) {
    std::string line;
    if (!std::getline(std::cin,line)) throw std::runtime_error("터미널 입력이 종료되었습니다.");
    return line.empty();
  }
  return false;
}
class Monitor : public rclcpp::Node {
public:
  explicit Monitor(const YAML::Node& frames) : Node("xarm_motion",rclcpp::NodeOptions().use_global_arguments(false)), frames_(frames) {
    joints_ = create_publisher<sensor_msgs::msg::JointState>("/xarm_motion/joint_states",rclcpp::SensorDataQoS());
    status_ = create_publisher<std_msgs::msg::String>("/xarm_motion/status",rclcpp::QoS(1).transient_local());
    for (const auto& side : {"left","right"})
      poses_[side] = create_publisher<geometry_msgs::msg::PoseStamped>(
        std::string("/xarm_motion/")+side+"/base_flange_pose",rclcpp::SensorDataQoS());
  }
  void status(const std::string& text) {
    std_msgs::msg::String msg; msg.data = text;
    status_->publish(msg);
    RCLCPP_INFO(get_logger(),"%s",text.c_str());
  }
  void diagnostics(const std::string& text) {
    RCLCPP_INFO(get_logger(),"MEASURE: %s",text.c_str());
  }
  void feedback(const std::vector<xm::MotionArm>& arms, const std::vector<xm::Feedback>& values) {
    sensor_msgs::msg::JointState joints; joints.header.stamp = now();
    for (size_t i = 0; i < arms.size(); ++i) {
      const auto& arm = arms[i];
      for (int j = 0; j < 7; ++j) {
        joints.name.push_back(arm.side+"_joint"+std::to_string(j+1));
        joints.position.push_back(values[i].joints[j]); joints.velocity.push_back(values[i].velocities[j]);
      }
      const xm::Pose flange = values[i].base_T_tcp*arm.flange_T_tcp.inverse();
      const Eigen::Quaterniond q(flange.linear());
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = joints.header.stamp;
      pose.header.frame_id = frames_["frames"][arm.side+"_base"].as<std::string>();
      pose.pose.position.x = flange.translation()[0]; pose.pose.position.y = flange.translation()[1];
      pose.pose.position.z = flange.translation()[2]; pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y(); pose.pose.orientation.z = q.z(); pose.pose.orientation.w = q.w();
      poses_.at(arm.side)->publish(pose);
    }
    joints_->publish(joints);
  }
private:
  YAML::Node frames_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  std::map<std::string,rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> poses_;
};
}  // namespace

int main(int argc, char** argv) {
  if (argc == 2 && std::string(argv[1]) == "--help") { std::cout << usage; return 0; }
  std::shared_ptr<Monitor> monitor;
  int result = 0;
  try {
    const std::filesystem::path config_dir = std::filesystem::path(XARM_CONTROL_PC_DIR)/"config";
    const auto yaml = YAML::LoadFile((config_dir/"control.yaml").string());
    xm::require_standalone_enabled(yaml); // SDK 생성·ROS 초기화·로봇 연결보다 먼저 검사한다.
    const auto config = xm::read_config(yaml);
    const auto args = arguments(argc,argv);
    if (!isatty(STDIN_FILENO)) throw std::runtime_error("Enter·Ctrl+C 입력이 가능한 터미널에서 실행하세요.");
    const auto poses = YAML::LoadFile((config_dir/"arm_poses.yaml").string());
    const auto frames = YAML::LoadFile((config_dir/"fixed_frames.yaml").string());
    std::vector<xm::MotionArm> arms;
    std::vector<std::unique_ptr<xm::ArmKinematics>> models;
    const std::string share = ament_index_cpp::get_package_share_directory("xarm_motion");
    for (int i = 0; i < 2; ++i) {
      const std::string side = i == 0 ? "left" : "right";
      if (args.at("--arm") != "both" && args.at("--arm") != side) continue;
      models.push_back(std::make_unique<xm::ArmKinematics>(share+"/models/xarm7_"+
        (config.model1300[i]?"true":"false")+".urdf"));
      arms.push_back({side,nullptr,models.back().get(),xm::tcp_transform(frames,side),
        xm::named_pose(poses,args.at("--start-pose"),side),
        xm::named_pose(poses,args.at("--target-pose"),side),{},nullptr});
    }
    struct sigaction action{}; action.sa_handler = handle_signal; sigemptyset(&action.sa_mask);
    sigaction(SIGINT,&action,nullptr); sigaction(SIGTERM,&action,nullptr);
    const std::string log_dir = (std::filesystem::path(XARM_CONTROL_PC_DIR)/"_log/xarm_motion").string();
    std::filesystem::create_directories(log_dir);
    if (setenv("ROS_LOG_DIR",log_dir.c_str(),1) != 0) throw std::runtime_error("ROS 로그 경로 설정 실패");
    rclcpp::init(0,nullptr,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
    monitor = std::make_shared<Monitor>(frames);
    const auto cancelled = [] { return interrupted != 0 || !rclcpp::ok(); };
    std::vector<std::unique_ptr<xm::XArmBackend>> backends;
    for (auto& arm : arms) {
      const int i = arm.side == "left" ? 0 : 1;
      backends.push_back(std::make_unique<xm::XArmBackend>(config.ips[i],arm.flange_T_tcp,
        config.limits.feedback_timeout,cancelled,
        (std::filesystem::path(XARM_CONTROL_PC_DIR)/"_colcon/locks/xarm_motion").string(),
        config.limits.report_timeout));
      arm.backend = backends.back().get();
    }
    xm::MotionSession session(std::move(arms),config.limits,cancelled);
    monitor->status("PREPARING: 실제 시작 자세 확인 및 전체 경로 계산");
    session.prepare(xm::steady_seconds());
    monitor->status("READY: "+args.at("--arm")+" / "+args.at("--start-pose")+" -> "+
      args.at("--target-pose")+" / 예상 "+std::to_string(session.duration())+"초");
    monitor->diagnostics(session.diagnostics());
    std::cout << "Enter를 누르면 로봇 제어를 시작합니다. Ctrl+C는 중지·종료입니다.\n" << std::flush;
    while (!cancelled()) {
      monitor->feedback(session.arms(),session.feedback(xm::steady_seconds()));
      rclcpp::spin_some(monitor);
      if (enter_pressed()) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (cancelled()) throw std::runtime_error("실행이 취소되었습니다.");
    session.begin(xm::steady_seconds());
    monitor->status("RUNNING: Ctrl+C로 중지");
    auto next = std::chrono::steady_clock::now();
    double published_at = 0, diagnostic_at = 0;
    bool complete = false;
    while (!cancelled()) {
      next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(config.limits.period()));
      std::this_thread::sleep_until(next);
      const double now = xm::steady_seconds();
      if (session.tick(now)) { monitor->diagnostics(session.diagnostics()); complete = true; break; }
      if (now-diagnostic_at >= 1.0) {
        monitor->diagnostics(session.diagnostics());
        diagnostic_at = now;
      }
      if (now-published_at >= 0.05) {
        monitor->feedback(session.arms(),session.last_feedback());
        published_at = now;
      }
      rclcpp::spin_some(monitor);
      // 늦어진 주기를 빠르게 몰아서 실행하지 않는다.
      next = std::max(next,std::chrono::steady_clock::now());
    }
    session.stop();
    if (complete) monitor->status("COMPLETED: 실측 도착 확인 후 중지·종료");
    else { monitor->status("STOPPED: 사용자 중지"); result = 130; }
  } catch (const std::exception& e) {
    std::cerr << (interrupted ? "[중지] " : "[경고] ") << e.what() << '\n';
    if (monitor) monitor->status(std::string(interrupted?"STOPPED: ":"FAULT: ")+e.what());
    result = interrupted ? 130 : 1;
  }
  if (rclcpp::ok()) rclcpp::shutdown();
  return result;
}
