#include "xarm_motion/xarm_backend.hpp"
#include "xarm/wrapper/xarm_api.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>
#include <chrono>
#include <cerrno>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace xarm_motion {
namespace {
void checked(int code, const std::string& operation) {
  if (code != 0) throw std::runtime_error(operation+" 실패: SDK code="+std::to_string(code));
}
Pose sdk_pose(const fp32* data) {
  Pose pose = Pose::Identity();
  for (int i = 0; i < 6; ++i)
    if (!std::isfinite(data[i])) throw std::runtime_error("SDK 자세에 유한하지 않은 값이 있습니다.");
  pose.translation() = Eigen::Vector3d(data[0],data[1],data[2])*0.001;
  pose.linear() = (Eigen::AngleAxisd(data[5],Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(data[4],Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(data[3],Eigen::Vector3d::UnitX())).toRotationMatrix();
  return pose;
}
}  // namespace
struct XArmBackend::Impl {
  std::unique_ptr<XArmAPI> sdk;
  Pose tcp;
  double timeout, read_at = -1;
  std::function<bool()> cancelled;
  int lock_fd = -1;
  int native_lock_fd = -1;
  bool owned = false;

  ~Impl() {
    if (owned && sdk) {
      const int code = sdk->set_state(4);
      if (code) std::cerr << "[오류] 종료 시 중지 요청 실패: SDK code=" << code << '\n';
    }
    if (sdk) sdk->disconnect();
    if (lock_fd >= 0) close(lock_fd);
    if (native_lock_fd >= 0) close(native_lock_fd);
  }
  void check_cancelled() const {
    if (cancelled()) throw std::runtime_error("실행이 취소되었습니다.");
  }
  void check_connected() const {
    if (!sdk->is_connected() || !sdk->is_reported())
      throw std::runtime_error("SDK 제어 또는 상태 보고 연결이 끊겼습니다.");
  }
  void check_settings() const {
    if (sdk->axis != 7 || sdk->is_simulation_robot != 0)
      throw std::runtime_error("실제 xArm7 연결을 확인하세요. 컨트롤러 시뮬레이션 모드는 허용하지 않습니다.");
    if (!pose_close(sdk_pose(sdk->tcp_offset),tcp,1e-5,1e-4))
      throw std::runtime_error("실제 SDK TCP offset과 fixed_frames.yaml이 다릅니다. 자동 변경하지 않습니다.");
    if (!pose_close(sdk_pose(sdk->world_offset),Pose::Identity(),1e-6,1e-6))
      throw std::runtime_error("SDK world offset이 0이 아닙니다. base 기준 자세를 사용할 수 없습니다.");
  }
};
XArmBackend::XArmBackend(const std::string& ip, const Pose& tcp, double timeout,
                         std::function<bool()> cancelled, const std::string& lock_dir)
  : impl_(std::make_unique<Impl>()) {
  impl_->tcp = tcp; impl_->timeout = timeout; impl_->cancelled = std::move(cancelled);
  impl_->check_cancelled();
  in_addr address{};
  if (inet_pton(AF_INET,ip.c_str(),&address) != 1) throw std::runtime_error("IPv4 주소가 필요합니다: "+ip);
  // 새 잠금 파일은 저장소 내부에만 만든다.
  std::filesystem::create_directories(lock_dir);
  impl_->lock_fd = open((lock_dir+"/"+ip+".lock").c_str(),
    O_CREAT|O_RDWR|O_NOFOLLOW|O_CLOEXEC,0600);
  if (impl_->lock_fd < 0 || flock(impl_->lock_fd,LOCK_EX|LOCK_NB) != 0)
    throw std::runtime_error("다른 xarm_motion이 해당 로봇을 사용 중입니다: "+ip);
  // 기존 native 잠금이 있으면 읽기 전용으로 열어 함께 잠근다. 외부 파일을 만들거나 쓰지 않는다.
  impl_->native_lock_fd = open(("/tmp/omega7_cartesian_"+ip+".lock").c_str(),O_RDONLY|O_NOFOLLOW|O_CLOEXEC);
  if ((impl_->native_lock_fd < 0 && errno != ENOENT) ||
      (impl_->native_lock_fd >= 0 && flock(impl_->native_lock_fd,LOCK_EX|LOCK_NB) != 0))
    throw std::runtime_error("기존 native 제어기 잠금을 확보할 수 없습니다: "+ip);
  impl_->sdk = std::make_unique<XArmAPI>(ip,true,true,true,true,true,false,true,false,0);
  checked(impl_->sdk->connect(),"connect");
  checked(impl_->sdk->set_timeout(static_cast<fp32>(timeout)),"set_timeout");
  const double until = steady_seconds()+2.0;
  while (!impl_->sdk->is_reported() && steady_seconds()<until) {
    impl_->check_cancelled();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  impl_->check_connected();
  impl_->check_settings();
}
XArmBackend::~XArmBackend() = default;
Feedback XArmBackend::read() {
  impl_->check_cancelled(); impl_->check_connected(); impl_->check_settings();
  Feedback value;
  value.received_at = steady_seconds();
  fp32 q[7]{}, velocity[7]{}, effort[7]{}, pose[6]{};
  int errors[2]{}, state = -1;
  checked(impl_->sdk->get_err_warn_code(errors),"get_err_warn_code");
  checked(impl_->sdk->get_state(&state),"get_state");
  if (errors[0] || errors[1]) throw std::runtime_error("로봇 오류·경고가 있습니다. 자동 해제하지 않습니다: "+
    std::to_string(errors[0])+"/"+std::to_string(errors[1]));
  if (impl_->owned) {
    if (impl_->sdk->mode != 1 || (state != 0 && state != 1 && state != 2))
      throw std::runtime_error("로봇이 servo mode/실행 가능 상태가 아닙니다.");
    for (int i = 0; i < 7; ++i)
      if (!impl_->sdk->motor_enable_states[i] || !impl_->sdk->motor_brake_states[i])
        throw std::runtime_error("관절 모터 활성화·브레이크 상태를 확인하세요.");
  } else if (state == 1 || state == 3 || state == 5) {
    throw std::runtime_error("이동 중·일시 정지·초기화 중인 로봇에는 연결할 수 없습니다.");
  }
  checked(impl_->sdk->get_joint_states(q,velocity,effort,2),"get_joint_states");
  checked(impl_->sdk->get_position(pose),"get_position");
  for (int i = 0; i < 7; ++i) { value.joints[i] = q[i]; value.velocities[i] = velocity[i]; }
  value.base_T_tcp = sdk_pose(pose);
  if (steady_seconds()-value.received_at > impl_->timeout)
    throw std::runtime_error("SDK 피드백 조회가 제한 시간을 초과했습니다.");
  impl_->read_at = value.received_at;
  return value;
}
void XArmBackend::enable() {
  impl_->check_cancelled();
  impl_->owned = true;
  checked(impl_->sdk->motion_enable(true),"motion_enable");
  impl_->check_cancelled(); checked(impl_->sdk->set_mode(1),"set_mode(1)");
  impl_->check_cancelled(); checked(impl_->sdk->set_state(0),"set_state(0)");
  const double until = steady_seconds()+2.0;
  bool ready = false;
  while (steady_seconds()<until) {
    impl_->check_cancelled(); impl_->check_connected();
    ready = impl_->sdk->mode == 1;
    for (int i = 0; i < 7; ++i) ready &= impl_->sdk->motor_enable_states[i] && impl_->sdk->motor_brake_states[i];
    if (ready) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (!ready) throw std::runtime_error("로봇 servo mode 전환 확인 시간 초과");
  read();
}
void XArmBackend::send(const Joints& q) {
  impl_->check_cancelled(); impl_->check_connected();
  if (!impl_->owned || !q.allFinite() || steady_seconds()-impl_->read_at > impl_->timeout)
    throw std::runtime_error("명령 전송 조건 또는 피드백 유효 시간을 확인하세요.");
  fp32 command[7];
  for (int i = 0; i < 7; ++i) command[i] = static_cast<fp32>(q[i]);
  checked(impl_->sdk->set_servo_angle_j(command),"set_servo_angle_j");
}
void XArmBackend::stop() {
  if (!impl_->owned) return;
  checked(impl_->sdk->set_state(4),"set_state(4) 중지");
  impl_->owned = false;
}
}  // namespace xarm_motion
