#include "dock_apriltag_localizer/pose_processing.hpp"

#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dock_apriltag_localizer {
Eigen::Matrix3d rpy_to_rotation(const Eigen::Vector3d & rpy) {
  return (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
}
Pose make_transform(const Eigen::Vector3d & xyz, const Eigen::Vector3d & rpy) {
  Pose t = Pose::Identity(); t.block<3, 3>(0, 0) = rpy_to_rotation(rpy);
  t.block<3, 1>(0, 3) = xyz; return t;
}
Eigen::Matrix3d correct_pnp_rotation(const Eigen::Matrix3d & rotation) {
  return rotation * Eigen::Vector3d(-1, 1, -1).asDiagonal();
}
Eigen::Matrix3d project_rotation(const Eigen::Matrix3d & matrix) {
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d u = svd.matrixU();
  if ((u * svd.matrixV().transpose()).determinant() < 0) u.col(2) *= -1;
  return u * svd.matrixV().transpose();
}
std::optional<Pose> fuse(const std::vector<Candidate> & candidates, double max_error) {
  if (candidates.empty()) return std::nullopt;
  std::vector<const Candidate *> kept;
  if (candidates.size() >= 3) {
    Eigen::Vector3d median;
    for (int axis = 0; axis < 3; ++axis) {
      std::vector<double> values;
      for (const auto & c : candidates) values.push_back(c.transform(axis, 3));
      std::sort(values.begin(), values.end());
      const auto n = values.size();
      median(axis) = n % 2 ? values[n / 2] : (values[n / 2 - 1] + values[n / 2]) / 2;
    }
    for (const auto & c : candidates) {
      if ((c.transform.block<3, 1>(0, 3) - median).norm() <= max_error) kept.push_back(&c);
    }
  }
  // Match Python: if median rejection would discard everything, keep all.
  if (kept.empty()) for (const auto & c : candidates) kept.push_back(&c);
  double total = 0;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Zero();
  for (const auto * c : kept) {
    total += c->weight;
    position += c->weight * c->transform.block<3, 1>(0, 3);
    rotation += c->weight * c->transform.block<3, 3>(0, 0);
  }
  Pose result = Pose::Identity();
  result.block<3, 1>(0, 3) = position / total;
  result.block<3, 3>(0, 0) = project_rotation(rotation / total);
  return result;
}
void PoseSelector::reset() { good_ = 0; missing_ = 0; active_ = false; }
Selection PoseSelector::select(const std::vector<Candidate> & candidates) {
  std::vector<Candidate> outer, central;
  for (const auto & c : candidates) {
    if (!c.central) outer.push_back(c);
    else if (c.distance <= options_.central_switch_distance_m) central.push_back(c);
  }
  auto central_pose = fuse(central, options_.max_candidate_position_error_m);
  auto outer_pose = fuse(outer, options_.max_candidate_position_error_m);
  if (!central.empty()) { ++good_; missing_ = 0; }
  else { good_ = 0; ++missing_; }
  if (!active_ && good_ >= options_.central_stable_frames) active_ = true;
  else if (active_ && missing_ > options_.central_dropout_frames) active_ = false;
  if (active_ && central_pose) return {central_pose, "CENTRAL"};
  if (outer_pose) return {outer_pose, "OUTER"};
  if (central_pose) return {central_pose, "CENTRAL_ACQUIRING"};
  return {std::nullopt, "NO_TARGET"};
}
TemporalPoseFilter::TemporalPoseFilter(FilterOptions options) : options_(options) {
  if (!std::isfinite(options.time_constant_s) || options.time_constant_s < 0 ||
    !std::isfinite(options.max_translation_speed_m_s) || options.max_translation_speed_m_s < 0 ||
    !std::isfinite(options.max_position_jump_m) || options.max_position_jump_m < 0 ||
    !std::isfinite(options.max_gate_dt_s) || options.max_gate_dt_s <= 0 ||
    !std::isfinite(options.reset_timeout_s) || options.reset_timeout_s <= 0 ||
    options.reacquire_frames < 1) throw std::invalid_argument("Invalid pose filter parameters");
}
void TemporalPoseFilter::reset() {
  filtered_.reset(); measurement_.reset(); pending_.reset(); pending_count_ = 0;
  measurement_stamp_ = last_input_stamp_ = pending_stamp_ = 0;
}
Pose TemporalPoseFilter::blend(const Pose & previous, const Pose & measurement, double alpha) {
  Pose result = Pose::Identity();
  result.block<3, 1>(0, 3) = (1 - alpha) * previous.block<3, 1>(0, 3) +
    alpha * measurement.block<3, 1>(0, 3);
  result.block<3, 3>(0, 0) = project_rotation((1 - alpha) * previous.block<3, 3>(0, 0) +
    alpha * measurement.block<3, 3>(0, 0));
  return result;
}
FilterResult TemporalPoseFilter::accept_initial(
  const Pose & pose, double stamp, const std::string & reason)
{
  filtered_ = pose; measurement_ = pose; measurement_stamp_ = last_input_stamp_ = stamp;
  pending_.reset(); pending_count_ = 0; return {pose, reason};
}
FilterResult TemporalPoseFilter::update(const Pose & value, double stamp) {
  if (!value.allFinite() || !std::isfinite(stamp)) {
    throw std::invalid_argument("Pose and timestamp must be finite");
  }
  if (!options_.enabled) return {value, "disabled"};
  if (!filtered_) return accept_initial(value, stamp, "initialized");
  if (stamp <= last_input_stamp_) return {std::nullopt, "non_monotonic_timestamp"};
  const double gap = stamp - last_input_stamp_; last_input_stamp_ = stamp;
  if (gap > options_.reset_timeout_s) return accept_initial(value, stamp, "reset_after_gap");
  const double dt = stamp - measurement_stamp_;
  const double gate = options_.max_position_jump_m + options_.max_translation_speed_m_s *
    std::clamp(dt, 0.0, options_.max_gate_dt_s);
  if ((value.block<3, 1>(0, 3) - measurement_->block<3, 1>(0, 3)).norm() > gate) {
    return reacquire(value, stamp);
  }
  const double alpha = options_.time_constant_s <= 0 ? 1 :
    1 - std::exp(-dt / options_.time_constant_s);
  filtered_ = blend(*filtered_, value, alpha);
  measurement_ = value; measurement_stamp_ = stamp;
  pending_.reset(); pending_count_ = 0; return {filtered_, "accepted"};
}
FilterResult TemporalPoseFilter::reacquire(const Pose & value, double stamp) {
  if (!pending_) { pending_ = value; pending_count_ = 1; }
  else {
    const double gate = options_.max_position_jump_m + options_.max_translation_speed_m_s *
      std::clamp(stamp - pending_stamp_, 0.0, options_.max_gate_dt_s);
    if ((value.block<3, 1>(0, 3) - pending_->block<3, 1>(0, 3)).norm() > gate) {
      pending_ = value; pending_count_ = 1;
    } else { pending_ = blend(*pending_, value, 0.5); ++pending_count_; }
  }
  pending_stamp_ = stamp;
  if (pending_count_ < options_.reacquire_frames) return {std::nullopt, "outlier_rejected"};
  const Pose result = *pending_;
  return accept_initial(result, stamp, "reacquired");
}
}  // namespace dock_apriltag_localizer
