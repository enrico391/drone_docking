#include "dock_apriltag_localizer/dock_localizer.hpp"
#include "dock_apriltag_localizer/detection.hpp"

#include <Eigen/Geometry>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace dock_apriltag_localizer {
namespace {
using SteadyClock = std::chrono::steady_clock;
double seconds(SteadyClock::time_point end, SteadyClock::time_point start) {
  return std::chrono::duration<double>(end - start).count();
}
std::string json_number(std::optional<double> value) {
  if (!value || !std::isfinite(*value)) return "null";
  std::ostringstream stream; stream << std::setprecision(17) << *value; return stream.str();
}
}  // namespace
struct DockLocalizer::Impl {
  DockLocalizer & node;
  std::unique_ptr<DetectionEngine> detector;
  PoseSelector selector;
  TemporalPoseFilter filter;
  cv::Mat k, distortion;
  std::string camera_frame;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub, timing_pub;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr ids_pub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload;

  explicit Impl(DockLocalizer & owner) : node(owner), filter(filter_options(owner)) {
    const auto path = node.declare_parameter<std::string>("config_file", "");
    if (path.empty()) throw std::invalid_argument("Parameter config_file is required.");
    node.declare_parameter<std::string>("dock_frame", "dock_target");
    const auto image_topic = node.declare_parameter<std::string>("image_topic", "/camera/image_raw");
    const auto info_topic = node.declare_parameter<std::string>("camera_info_topic", "/camera/camera_info");
    const bool debug = node.declare_parameter<bool>("publish_debug_image", true);
    const auto debug_topic = node.declare_parameter<std::string>("debug_image_topic", "/dock/debug_image");
    detector = std::make_unique<DetectionEngine>(path);
    selector = PoseSelector(detector->switching_options());
    pose_pub = node.create_publisher<geometry_msgs::msg::PoseStamped>("/dock/pose", 10);
    mode_pub = node.create_publisher<std_msgs::msg::String>("/dock/mode", 10);
    timing_pub = node.create_publisher<std_msgs::msg::String>("/dock/timing", 10);
    ids_pub = node.create_publisher<std_msgs::msg::Int32MultiArray>("/dock/visible_tag_ids", 10);
    if (debug) debug_pub = node.create_publisher<sensor_msgs::msg::Image>(debug_topic, 2);
    tf = std::make_unique<tf2_ros::TransformBroadcaster>(node);
    // All callbacks use the default mutually exclusive group, including reload.
    reload = node.create_service<std_srvs::srv::Trigger>("~/reload_config",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        try {
          auto replacement = std::make_unique<DetectionEngine>(node.get_parameter("config_file").as_string());
          selector = PoseSelector(replacement->switching_options());
          detector = std::move(replacement); filter.reset();
          response->success = true;
          response->message = "Reloaded " + std::to_string(detector->tag_count()) + " tags.";
        } catch (const std::exception & error) {
          response->success = false; response->message = error.what();
        }
      });
    info_sub = node.create_subscription<sensor_msgs::msg::CameraInfo>(info_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        k = cv::Mat(3, 3, CV_64F, const_cast<double *>(msg->k.data())).clone();
        distortion = msg->d.empty() ? cv::Mat::zeros(5, 1, CV_64F) : cv::Mat(msg->d).clone();
        camera_frame = msg->header.frame_id;
      });
    image_sub = node.create_subscription<sensor_msgs::msg::Image>(image_topic,
      rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { image_callback(msg); });
    RCLCPP_INFO(node.get_logger(), "Loaded %zu tags from %s (intra-process: %s)",
      detector->tag_count(), path.c_str(), node.get_node_options().use_intra_process_comms() ? "enabled" : "disabled");
  }
  static FilterOptions filter_options(DockLocalizer & node) {
    FilterOptions options;
    options.enabled = node.declare_parameter<bool>("pose_filter_enabled", true);
    options.time_constant_s = node.declare_parameter<double>("pose_filter_time_constant_s", 0.12);
    options.max_translation_speed_m_s = node.declare_parameter<double>("pose_filter_max_translation_speed_m_s", 2.0);
    options.max_position_jump_m = node.declare_parameter<double>("pose_filter_max_position_jump_m", 0.20);
    options.max_gate_dt_s = node.declare_parameter<double>("pose_filter_max_gate_dt_s", 0.20);
    options.reset_timeout_s = node.declare_parameter<double>("pose_filter_reset_timeout_s", 1.0);
    options.reacquire_frames = node.declare_parameter<int>("pose_filter_reacquire_frames", 3);
    return options;
  }
  std::optional<double> frame_age(const builtin_interfaces::msg::Time & stamp) {
    const auto ns = static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
    if (ns == 0) return std::nullopt;
    return static_cast<double>(node.now().nanoseconds() - ns) * 1e-9;
  }
  void publish_pose(const Pose & transform, const std_msgs::msg::Header & header) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    if (pose.header.frame_id.empty()) pose.header.frame_id = camera_frame.empty() ? "camera_optical_frame" : camera_frame;
    pose.pose.position.x = transform(0, 3); pose.pose.position.y = transform(1, 3); pose.pose.position.z = transform(2, 3);
    Eigen::Quaterniond q(transform.block<3, 3>(0, 0)); q.normalize();
    pose.pose.orientation.x = q.x(); pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z(); pose.pose.orientation.w = q.w();
    pose_pub->publish(pose);
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header = pose.header; tf_msg.child_frame_id = node.get_parameter("dock_frame").as_string();
    tf_msg.transform.translation.x = pose.pose.position.x;
    tf_msg.transform.translation.y = pose.pose.position.y;
    tf_msg.transform.translation.z = pose.pose.position.z;
    tf_msg.transform.rotation = pose.pose.orientation; tf->sendTransform(tf_msg);
  }
  void publish_debug(const sensor_msgs::msg::Image::ConstSharedPtr & msg,
    const std::vector<Candidate> & candidates, const std::string & mode)
  {
    // Always clone before annotation: the input may be shared with other components.
    cv::Mat bgr = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    for (const auto & c : candidates) {
      for (int i = 0; i < 4; ++i) {
        const int j = (i + 1) % 4;
        cv::line(bgr, cv::Point(static_cast<int>(c.corners(i, 0)), static_cast<int>(c.corners(i, 1))),
          cv::Point(static_cast<int>(c.corners(j, 0)), static_cast<int>(c.corners(j, 1))), cv::Scalar(0, 255, 0), 2);
      }
      std::ostringstream label;
      label << "ID " << c.id << (c.central ? " central " : " outer ") << std::fixed << std::setprecision(2) << c.distance << "m";
      cv::putText(bgr, label.str(), cv::Point(static_cast<int>(c.center.x()) + 5, static_cast<int>(c.center.y()) - 5),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    }
    cv::putText(bgr, "MODE: " + mode, cv::Point(15, 30), cv::FONT_HERSHEY_SIMPLEX,
      0.8, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
    debug_pub->publish(*cv_bridge::CvImage(msg->header, "bgr8", bgr).toImageMsg());
  }
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg) {
    const auto started = SteadyClock::now();
    const auto image_age = frame_age(msg->header.stamp);
    if (k.empty()) {
      RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 3000, "Waiting for CameraInfo..."); return;
    }
    try {
      cv::Mat gray;
      // Share mono images directly; RGB/BGR need only the grayscale conversion.
      cv_bridge::CvImageConstPtr shared;
      if (msg->encoding == "mono8") { shared = cv_bridge::toCvShare(msg, "mono8"); gray = shared->image; }
      else if (msg->encoding == "rgb8" || msg->encoding == "bgr8") {
        shared = cv_bridge::toCvShare(msg, msg->encoding);
        cv::cvtColor(shared->image, gray, msg->encoding == "rgb8" ? cv::COLOR_RGB2GRAY : cv::COLOR_BGR2GRAY);
      } else {
        shared = cv_bridge::toCvShare(msg, "bgr8"); cv::cvtColor(shared->image, gray, cv::COLOR_BGR2GRAY);
      }
      const auto detection_started = SteadyClock::now();
      double detection_duration_s = 0;
      const auto candidates = detector->detect(gray, k, distortion, &detection_duration_s);
      // PnP and candidate construction belong to pose_processing_s, as in Python.
      const auto detection_finished = detection_started +
        std::chrono::duration_cast<SteadyClock::duration>(std::chrono::duration<double>(detection_duration_s));
      std_msgs::msg::Int32MultiArray ids;
      for (const auto & c : candidates) ids.data.push_back(c.id);
      ids_pub->publish(ids);
      const auto selected = selector.select(candidates);
      std::optional<double> pose_age;
      std::string filter_status = "no_target";
      if (selected.pose) {
        const double stamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
        const auto filtered = filter.update(*selected.pose, stamp); filter_status = filtered.status;
        if (filtered.pose) { pose_age = frame_age(msg->header.stamp); publish_pose(*filtered.pose, msg->header); }
        else RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 1000,
          "Rejecting dock pose: %s", filter_status.c_str());
      }
      std_msgs::msg::String mode; mode.data = selected.mode; mode_pub->publish(mode);
      const auto debug_started = SteadyClock::now();
      if (debug_pub) publish_debug(msg, candidates, selected.mode);
      const auto finished = SteadyClock::now();
      std::ostringstream json;
      json << std::setprecision(17)
        << "{\"source_stamp_s\":" << msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9
        << ",\"image_age_at_callback_s\":" << json_number(image_age)
        << ",\"pose_age_at_publish_s\":" << json_number(pose_age)
        << ",\"conversion_s\":" << seconds(detection_started, started)
        << ",\"detection_s\":" << seconds(detection_finished, detection_started)
        << ",\"pose_processing_s\":" << seconds(debug_started, detection_finished)
        << ",\"debug_image_s\":" << seconds(finished, debug_started)
        << ",\"callback_processing_s\":" << seconds(finished, started)
        << ",\"mode\":\"" << selected.mode << "\",\"filter_status\":\"" << filter_status << "\"}";
      std_msgs::msg::String timing; timing.data = json.str(); timing_pub->publish(timing);
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(node.get_logger(), *node.get_clock(), 1000, "Image processing failed: %s", error.what());
    }
  }
};
DockLocalizer::DockLocalizer(const rclcpp::NodeOptions & options)
: Node("dock_apriltag_localizer", options), impl_(std::make_unique<Impl>(*this)) {}
DockLocalizer::~DockLocalizer() = default;
}  // namespace dock_apriltag_localizer
RCLCPP_COMPONENTS_REGISTER_NODE(dock_apriltag_localizer::DockLocalizer)
