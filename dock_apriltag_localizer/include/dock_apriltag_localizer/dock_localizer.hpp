#pragma once
#include <rclcpp/rclcpp.hpp>
#include <memory>

namespace dock_apriltag_localizer {
class DockLocalizer : public rclcpp::Node {
public:
  explicit DockLocalizer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~DockLocalizer() override;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace dock_apriltag_localizer
