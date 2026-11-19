#pragma once
#include "xarm_motion/motion_core.hpp"
#include <deque>
#include <map>
#include <optional>

namespace auto_align {
namespace xm = xarm_motion;
using Features = Eigen::Matrix<double,5,1>; // camera endpoint xyz [m], axis angles [rad]
using VisualJacobian = Eigen::Matrix<double,5,3>;
struct VisionSettings {
  std::array<std::string,2> topics;
  std::string frame;
  std::array<double,2> endpoint_z;
  double length, max_age, pair_timeout, sample_window, sample_timeout, settle;
  int min_samples, success_samples;
  double angle_tolerance, relative_angle_tolerance, position_tolerance, relative_position_tolerance;
  double hold, max_position_noise, max_angle_noise, probe_translation, probe_rotation;
  double max_translation, max_rotation, linear_speed, angular_speed, gain, damping;
  double max_travel, max_turn, min_gap, gap_drift, deadline;
  int max_iterations, max_moves, max_worsening;
};
VisionSettings vision_settings(const YAML::Node& control);
void require_vision_enabled(const YAML::Node& control);
struct PipeObservation { Eigen::Vector3d endpoint, axis; };
struct VisionPair {
  std::array<PipeObservation,2> pipes;
  int64_t stamp = 0;
  double received = 0;
};
PipeObservation pipe_observation(const xm::Pose& camera_T_pipe, double endpoint_z);
class VisionPairs {
public:
  void add(size_t side, int64_t stamp, double received, const PipeObservation& pipe);
  std::optional<VisionPair> take();
  int64_t last_stamp() const { return last_stamp_; }
private:
  struct Value { double received; PipeObservation pipe; };
  std::array<std::map<int64_t,Value>,2> pending_;
  int64_t last_stamp_ = 0;
};
VisionPair average_pairs(const std::vector<VisionPair>& samples, const VisionSettings& settings);
Eigen::Vector3d visual_solve(const Eigen::MatrixXd& jacobian, const Eigen::VectorXd& error,
                            double gain, double damping, double maximum);

// Hardware-free boundary: the runtime supplies fresh settled measurements and
// checked SDK motion. Tests use the same controller with a synthetic scene.
class VisionIO {
public:
  virtual ~VisionIO() = default;
  virtual VisionPair measure() = 0;
  // Returns the measured base-frame displacement/rotation, not the request.
  virtual Eigen::Vector3d move(size_t side, const Eigen::Vector3d& delta, bool rotation) = 0;
  virtual double elapsed() const = 0;
  virtual void report(const std::string& status) = 0;
};
class VisionAlign {
public:
  VisionAlign(VisionSettings settings, VisionIO& io);
  void run();
  const std::array<Eigen::Vector3d,2>& targets() const { return targets_; }
  int moves() const { return moves_; }
private:
  void initialize();
  Features features(const VisionPair& pair, size_t side) const;
  void check(const VisionPair& pair) const;
  VisionPair measure();
  VisionPair move(size_t side, const Eigen::Vector3d& delta, bool rotation,
                  Eigen::Vector3d* actual = nullptr);
  void learn(size_t side, bool rotation);
  void rotation_stage();
  bool translation_stage();
  bool angles_ok(const VisionPair& pair) const;
  bool positions_ok(const VisionPair& pair) const;
  void correct(size_t side, bool rotation);
  VisionSettings settings_;
  VisionIO& io_;
  VisionPair current_, initial_;
  std::array<Eigen::Vector3d,2> targets_;
  std::array<double,2> signs_{};
  std::array<std::array<VisualJacobian,2>,2> jacobians_{}; // side, translation/rotation
  std::array<std::array<bool,2>,2> learned_{};
  std::array<std::array<int,2>,2> worsening_{};
  int moves_ = 0;
  int64_t last_stamp_ = 0;
};
int vision_align_main(int argc, char** argv);
} // namespace auto_align
