#pragma once
#include "xarm_motion/motion_core.hpp"

namespace xarm_motion {
// SDK 헤더와 연결 생명주기는 계산 라이브러리 밖에 둔다.
class XArmBackend final : public ArmBackend {
public:
  XArmBackend(const std::string& ip, const Pose& flange_T_tcp,
              double timeout, std::function<bool()> cancelled, const std::string& lock_dir,
              double report_timeout);
  ~XArmBackend() override;
  Feedback read() override;
  void enable() override;
  void send(const Joints& joints) override;
  void stop() override;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace xarm_motion
