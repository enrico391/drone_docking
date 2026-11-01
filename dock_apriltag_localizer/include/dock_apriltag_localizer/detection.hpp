#pragma once
#include "dock_apriltag_localizer/pose_processing.hpp"
#include <opencv2/core.hpp>
#include <memory>

namespace dock_apriltag_localizer {
// Owns a native AprilTag detector and a validated snapshot of the tag YAML.
class DetectionEngine {
public:
  explicit DetectionEngine(const std::string & config_file);
  ~DetectionEngine();
  std::vector<Candidate> detect(const cv::Mat & gray, const cv::Mat & k, const cv::Mat & distortion,
    double * detection_duration_s = nullptr);
  SwitchingOptions switching_options() const;
  std::size_t tag_count() const;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace dock_apriltag_localizer
