#pragma once
#include "xarm_motion/motion_core.hpp"
#include <array>
#include <memory>

namespace auto_align {
namespace xm = xarm_motion;
using DualJoints = std::array<xm::Joints,2>;
struct CollisionSettings {
  double margin = 0.02;
  double interval = 0.05;
  double minimum_interval = 0.0001;
  size_t maximum_checks = 200000;
};
struct EnvironmentBox {
  std::string name;
  xm::Pose world_T_box;
  Eigen::Vector3d size;
};
struct CollisionStats {
  size_t samples = 0, fk_evaluations = 0, pair_checks = 0, aabb_rejections = 0;
  size_t collision_queries = 0, distance_queries = 0, subdivisions = 0, max_depth = 0;
  double total_seconds = 0, fk_seconds = 0, narrowphase_seconds = 0;
};
std::vector<EnvironmentBox> environment_boxes(const YAML::Node& environment);
class CollisionChecker {
public:
  CollisionChecker(const std::array<std::string,2>& urdfs,
                   const std::array<xm::Pose,2>& mounts,
                   const std::vector<EnvironmentBox>& environment,
                   CollisionSettings settings);
  ~CollisionChecker();
  // 시작점·대기 중 실측 자세와 실행 전 전체 경로를 검사한다.
  void check(const DualJoints& joints);
  void check_path(const std::function<DualJoints(double)>& sample, double duration,
                  const std::array<double,2>& joint_speed_bounds,
                  const std::function<bool()>& cancelled);
  void check_path_joint_bounds(const std::function<DualJoints(double)>& sample, double duration,
                              const std::array<xm::Joints,2>& joint_speed_bounds,
                              const std::function<bool()>& cancelled);
  const CollisionStats& stats() const;
  std::string diagnostics() const;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace auto_align
