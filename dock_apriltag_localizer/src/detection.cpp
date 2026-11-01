#include "dock_apriltag_localizer/detection.hpp"

#include <Eigen/Geometry>
#include <opencv2/calib3d.hpp>
#include <yaml-cpp/yaml.h>
#include <apriltag/apriltag.h>
#include <apriltag/tag16h5.h>
#include <apriltag/tag25h9.h>
#include <apriltag/tag36h10.h>
#include <apriltag/tag36h11.h>
#include <apriltag/tagCircle21h7.h>
#include <apriltag/tagCircle49h12.h>
#include <apriltag/tagCustom48h12.h>
#include <apriltag/tagStandard41h12.h>
#include <apriltag/tagStandard52h13.h>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <chrono>
#include <cstdlib>
#include <map>
#include <sstream>
#include <stdexcept>

namespace dock_apriltag_localizer {
namespace {
template<typename T> T value(const YAML::Node & node, const char * key, T fallback) {
  return node && node[key] ? node[key].as<T>() : fallback;
}
Eigen::Vector3d vector3(const YAML::Node & node) {
  if (!node) return Eigen::Vector3d::Zero();
  if (!node.IsSequence() || node.size() != 3) throw std::runtime_error("Expected three coordinates");
  Eigen::Vector3d result(node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
  if (!result.allFinite()) throw std::runtime_error("Tag coordinates must be finite");
  return result;
}
using Family = std::unique_ptr<apriltag_family_t, void (*)(apriltag_family_t *)>;
Family make_family(const std::string & name) {
#define FAMILY(name_) if (name == #name_) return Family(name_##_create(), name_##_destroy)
  FAMILY(tag16h5); FAMILY(tag25h9); FAMILY(tag36h10); FAMILY(tag36h11);
  FAMILY(tagCircle21h7); FAMILY(tagCircle49h12); FAMILY(tagCustom48h12);
  FAMILY(tagStandard41h12); FAMILY(tagStandard52h13);
#undef FAMILY
  throw std::runtime_error("Unsupported AprilTag family: " + name);
}
}  // namespace
struct DetectionEngine::Impl {
  struct Tag { bool central; double size; Pose tag_to_dock; };
  // Destroy the detector before the families it references.
  std::vector<Family> families;
  std::unique_ptr<apriltag_detector_t, decltype(&apriltag_detector_destroy)> detector{
    apriltag_detector_create(), apriltag_detector_destroy};
  std::map<int, Tag> tags;
  SwitchingOptions switching;
  double min_margin = 20.0;
};
DetectionEngine::~DetectionEngine() = default;
DetectionEngine::DetectionEngine(const std::string & config_file) : impl_(std::make_unique<Impl>()) {
  std::string path = config_file;
  if (path.rfind("~/", 0) == 0) {
    const char * home_dir = std::getenv("HOME");
    if (home_dir) path = std::string(home_dir) + path.substr(1);
  }
  const auto config = YAML::LoadFile(path);
  const auto detector = config["detector"];
  auto * native = impl_->detector.get();
  if (!native) throw std::runtime_error("Cannot create AprilTag detector");
  native->nthreads = value(detector, "nthreads", 2);
  native->quad_decimate = value(detector, "quad_decimate", 1.0);
  native->quad_sigma = value(detector, "quad_sigma", 0.0);
  native->refine_edges = value(detector, "refine_edges", 1) != 0;
  native->decode_sharpening = value(detector, "decode_sharpening", 0.25);
  impl_->min_margin = value(detector, "min_decision_margin", 20.0);
  if (native->nthreads < 1 || !std::isfinite(native->quad_decimate) || native->quad_decimate <= 0 ||
    !std::isfinite(native->quad_sigma) || !std::isfinite(native->decode_sharpening) ||
    !std::isfinite(impl_->min_margin) || impl_->min_margin < 0) {
    throw std::runtime_error("Invalid detector configuration");
  }
  std::istringstream family_names(value<std::string>(detector, "family", "tag36h11"));
  std::string name;
  while (family_names >> name) {
    auto family = make_family(name);
    if (!family) throw std::runtime_error("Cannot create AprilTag family");
    apriltag_detector_add_family_bits(native, family.get(), 2);
    impl_->families.push_back(std::move(family));
  }
  if (impl_->families.empty()) throw std::runtime_error("At least one AprilTag family is required");
  const auto switching = config["switching"];
  impl_->switching.central_switch_distance_m = value(switching, "central_switch_distance_m", 1.8);
  impl_->switching.central_stable_frames = value(switching, "central_stable_frames", 4);
  impl_->switching.central_dropout_frames = value(switching, "central_dropout_frames", 5);
  impl_->switching.max_candidate_position_error_m = value(
    config["fusion"], "max_candidate_position_error_m", 0.35);
  const auto & s = impl_->switching;
  if (!std::isfinite(s.central_switch_distance_m) || s.central_switch_distance_m <= 0 ||
    s.central_stable_frames < 1 || s.central_dropout_frames < 0 ||
    !std::isfinite(s.max_candidate_position_error_m) || s.max_candidate_position_error_m < 0) {
    throw std::runtime_error("Invalid switching/fusion configuration");
  }
  if (!config["tags"].IsSequence() || config["tags"].size() == 0) {
    throw std::runtime_error("Configuration requires a nonempty tags sequence");
  }
  for (const auto & item : config["tags"]) {
    const int id = item["id"].as<int>();
    const double size = item["size_m"].as<double>();
    if (id < 0 || !std::isfinite(size) || size <= 0 || impl_->tags.count(id)) {
      throw std::runtime_error("Tag IDs must be unique/nonnegative and sizes positive/finite");
    }
    auto role = value<std::string>(item, "role", "outer");
    std::transform(role.begin(), role.end(), role.begin(), [](unsigned char c) { return std::tolower(c); });
    const auto transform = item["dock_to_tag"];
    const Eigen::Vector3d xyz = vector3(transform ? transform["xyz_m"] : YAML::Node());
    const Eigen::Vector3d rpy = vector3(transform ? transform["rpy_deg"] : YAML::Node()) *
      (std::acos(-1.0) / 180.0);
    impl_->tags.emplace(id, Impl::Tag{role == "central", size, make_transform(xyz, rpy).inverse()});
  }
}
SwitchingOptions DetectionEngine::switching_options() const { return impl_->switching; }
std::size_t DetectionEngine::tag_count() const { return impl_->tags.size(); }
std::vector<Candidate> DetectionEngine::detect(
  const cv::Mat & gray, const cv::Mat & k, const cv::Mat & distortion, double * detection_duration_s)
{
  if (gray.type() != CV_8UC1) throw std::invalid_argument("AprilTag input must be mono8");
  // Native detector reads but does not own the image; respect non-contiguous row stride.
  image_u8_t image{gray.cols, gray.rows, static_cast<int>(gray.step), gray.data};
  const auto started = std::chrono::steady_clock::now();
  std::unique_ptr<zarray_t, decltype(&apriltag_detections_destroy)> detections(
    apriltag_detector_detect(impl_->detector.get(), &image), apriltag_detections_destroy);
  if (detection_duration_s) *detection_duration_s = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - started).count();
  if (!detections) throw std::runtime_error("AprilTag detection failed");
  std::vector<Candidate> candidates;
  for (int i = 0; i < zarray_size(detections.get()); ++i) {
    apriltag_detection_t * detection;
    zarray_get(detections.get(), i, &detection);
    const auto entry = impl_->tags.find(detection->id);
    if (entry == impl_->tags.end() || detection->decision_margin < impl_->min_margin) continue;
    const double half = entry->second.size / 2;
    const std::vector<cv::Point3d> object{{-half, half, 0}, {half, half, 0},
      {half, -half, 0}, {-half, -half, 0}};
    std::vector<cv::Point2d> pixels;
    for (const auto & p : detection->p) pixels.emplace_back(p[0], p[1]);
    cv::Mat rotation_vector, translation, rotation;
    if (!cv::solvePnP(object, pixels, k, distortion, rotation_vector, translation, false,
      cv::SOLVEPNP_IPPE_SQUARE)) continue;
    if (!cv::checkRange(translation) || !cv::checkRange(rotation_vector) ||
      translation.at<double>(2) <= 0) continue;
    cv::Rodrigues(rotation_vector, rotation);
    Pose camera_tag = Pose::Identity();
    for (int r = 0; r < 3; ++r) {
      camera_tag(r, 3) = translation.at<double>(r);
      for (int c = 0; c < 3; ++c) camera_tag(r, c) = rotation.at<double>(r, c);
    }
    camera_tag.block<3, 3>(0, 0) = correct_pnp_rotation(camera_tag.block<3, 3>(0, 0));
    Candidate candidate;
    candidate.id = detection->id; candidate.central = entry->second.central;
    candidate.transform = camera_tag * entry->second.tag_to_dock;
    candidate.distance = camera_tag.block<3, 1>(0, 3).norm();
    candidate.weight = std::max(static_cast<double>(detection->decision_margin), 1.0) /
      std::max(candidate.distance * candidate.distance, 0.04);
    for (int p = 0; p < 4; ++p) {
      candidate.corners(p, 0) = detection->p[p][0]; candidate.corners(p, 1) = detection->p[p][1];
    }
    candidate.center = {detection->c[0], detection->c[1]};
    if (candidate.transform.allFinite()) candidates.push_back(candidate);
  }
  return candidates;
}
}  // namespace dock_apriltag_localizer
