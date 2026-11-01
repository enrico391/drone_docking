#pragma once

#include <Eigen/Core>
#include <optional>
#include <string>
#include <vector>

namespace dock_apriltag_localizer {
using Pose = Eigen::Matrix4d;
Eigen::Matrix3d rpy_to_rotation(const Eigen::Vector3d & rpy);
Pose make_transform(const Eigen::Vector3d & xyz, const Eigen::Vector3d & rpy);
Eigen::Matrix3d correct_pnp_rotation(const Eigen::Matrix3d & rotation);
Eigen::Matrix3d project_rotation(const Eigen::Matrix3d & matrix);

struct Candidate {
  int id = 0;
  bool central = false;
  Pose transform = Pose::Identity();
  double distance = 0.0;
  double weight = 1.0;
  Eigen::Matrix<double, 4, 2> corners = Eigen::Matrix<double, 4, 2>::Zero();
  Eigen::Vector2d center = Eigen::Vector2d::Zero();
};
std::optional<Pose> fuse(const std::vector<Candidate> & candidates, double max_error);
struct Selection { std::optional<Pose> pose; std::string mode; };
struct SwitchingOptions {
  double central_switch_distance_m = 1.8;
  int central_stable_frames = 4;
  int central_dropout_frames = 5;
  double max_candidate_position_error_m = 0.35;
};
class PoseSelector {
public:
  explicit PoseSelector(SwitchingOptions options = {}) : options_(options) {}
  Selection select(const std::vector<Candidate> & candidates);
  void reset();
private:
  SwitchingOptions options_;
  int good_ = 0, missing_ = 0;
  bool active_ = false;
};
struct FilterOptions {
  bool enabled = true;
  double time_constant_s = 0.12;
  double max_translation_speed_m_s = 2.0;
  double max_position_jump_m = 0.20;
  double max_gate_dt_s = 0.20;
  double reset_timeout_s = 1.0;
  int reacquire_frames = 3;
};
struct FilterResult { std::optional<Pose> pose; std::string status; };
class TemporalPoseFilter {
public:
  explicit TemporalPoseFilter(FilterOptions options = {});
  void reset();
  FilterResult update(const Pose & measurement, double stamp_s);
private:
  FilterResult accept_initial(const Pose & pose, double stamp_s, const std::string & reason);
  FilterResult reacquire(const Pose & measurement, double stamp_s);
  static Pose blend(const Pose & previous, const Pose & measurement, double alpha);
  FilterOptions options_;
  std::optional<Pose> filtered_, measurement_, pending_;
  double measurement_stamp_ = 0, last_input_stamp_ = 0, pending_stamp_ = 0;
  int pending_count_ = 0;
};
}  // namespace dock_apriltag_localizer
