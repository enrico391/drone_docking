#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <opencv2/imgproc.hpp>
#include <thread>
#include "dock_apriltag_localizer/dock_localizer.hpp"

TEST(Component, IntraProcessImagesProduceStampedPoseTimingAndReload) {
  rclcpp::init(0, nullptr);
  {
    auto options = rclcpp::NodeOptions().use_intra_process_comms(true).parameter_overrides({
      rclcpp::Parameter("config_file", TEST_CONFIG),
      rclcpp::Parameter("publish_debug_image", true)});
    auto localizer = std::make_shared<dock_apriltag_localizer::DockLocalizer>(options);
    auto source = std::make_shared<rclcpp::Node>("localizer_test", rclcpp::NodeOptions().use_intra_process_comms(true));
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(localizer); executor.add_node(source);
    auto image_pub = source->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", rclcpp::SensorDataQoS().keep_last(1));
    auto info_pub = source->create_publisher<sensor_msgs::msg::CameraInfo>("/camera/camera_info", rclcpp::SensorDataQoS());
    geometry_msgs::msg::PoseStamped::ConstSharedPtr pose;
    std::string mode, timing;
    sensor_msgs::msg::Image::ConstSharedPtr debug, observed_input;
    auto observer = source->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw",
      rclcpp::SensorDataQoS().keep_last(1),
      [&](sensor_msgs::msg::Image::ConstSharedPtr msg) { observed_input = msg; });
    auto poses = source->create_subscription<geometry_msgs::msg::PoseStamped>("/dock/pose", 10,
      [&](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { pose = msg; });
    auto modes = source->create_subscription<std_msgs::msg::String>("/dock/mode", 10,
      [&](std_msgs::msg::String::ConstSharedPtr msg) { mode = msg->data; });
    auto timings = source->create_subscription<std_msgs::msg::String>("/dock/timing", 10,
      [&](std_msgs::msg::String::ConstSharedPtr msg) { timing = msg->data; });
    auto images = source->create_subscription<sensor_msgs::msg::Image>("/dock/debug_image", 2,
      [&](sensor_msgs::msg::Image::ConstSharedPtr msg) { debug = msg; });
    auto spin = [&]() {
      for (int i = 0; i < 20; ++i) { executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(5)); }
    };
    sensor_msgs::msg::CameraInfo info;
    info.header.frame_id = "calibration_optical";
    info.k = {600, 0, 320, 0, 600, 240, 0, 0, 1};
    info_pub->publish(info); spin();
    std_msgs::msg::Header header; header.frame_id = "test_optical"; header.stamp.sec = 10;
    cv::Mat gray(480, 640, CV_8UC1, cv::Scalar(255));
    image_pub->publish(*cv_bridge::CvImage(header, "mono8", gray).toImageMsg()); spin();
    EXPECT_EQ(mode, "NO_TARGET"); EXPECT_FALSE(pose);
    EXPECT_NE(timing.find("\"pose_age_at_publish_s\":null"), std::string::npos);
    auto * family = tag36h11_create(); auto * tag = apriltag_to_image(family, 0);
    cv::Mat small(tag->height, tag->width, CV_8UC1, tag->buf, tag->stride), large;
    cv::resize(small, large, cv::Size(), 15, 15, cv::INTER_NEAREST);
    large.copyTo(gray(cv::Rect(250, 180, large.cols, large.rows)));
    image_u8_destroy(tag); tag36h11_destroy(family);
    auto input = cv_bridge::CvImage(header, "mono8", gray).toImageMsg();
    const auto original = input->data;
    image_pub->publish(std::make_unique<sensor_msgs::msg::Image>(*input)); spin();
    ASSERT_TRUE(pose); EXPECT_EQ(pose->header.stamp.sec, 10);
    EXPECT_EQ(pose->header.frame_id, "test_optical"); EXPECT_EQ(mode, "OUTER");
    EXPECT_NEAR(pose->pose.position.z, 1.501, 0.03);
    EXPECT_NE(timing.find("\"filter_status\":\"initialized\""), std::string::npos);
    ASSERT_TRUE(debug); EXPECT_EQ(debug->encoding, "bgr8");
    ASSERT_TRUE(observed_input); EXPECT_EQ(observed_input->data, original);
    pose.reset(); image_pub->publish(*input); spin();
    EXPECT_FALSE(pose);
    EXPECT_NE(timing.find("non_monotonic_timestamp"), std::string::npos);
    auto reload = source->create_client<std_srvs::srv::Trigger>("/dock_apriltag_localizer/reload_config");
    auto response = reload->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    ASSERT_EQ(executor.spin_until_future_complete(response, std::chrono::seconds(2)), rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_TRUE(response.get()->success);
    image_pub->publish(std::make_unique<sensor_msgs::msg::Image>(*input)); spin(); EXPECT_TRUE(pose);
    localizer->set_parameter(rclcpp::Parameter("config_file", "/no/such/config.yaml"));
    auto invalid = reload->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    ASSERT_EQ(executor.spin_until_future_complete(invalid, std::chrono::seconds(2)), rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_FALSE(invalid.get()->success);
    pose.reset(); input->header.stamp.nanosec = 100000000;
    image_pub->publish(std::make_unique<sensor_msgs::msg::Image>(*input)); spin();
    EXPECT_TRUE(pose);  // Invalid reload leaves the previous detector usable.
    localizer->set_parameter(rclcpp::Parameter("config_file", TEST_CONFIG));
    auto reset = reload->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    ASSERT_EQ(executor.spin_until_future_complete(reset, std::chrono::seconds(2)), rclcpp::FutureReturnCode::SUCCESS);
    input->header.stamp.sec = 0; input->header.stamp.nanosec = 0; input->header.frame_id.clear();
    image_pub->publish(std::make_unique<sensor_msgs::msg::Image>(*input)); spin();
    ASSERT_TRUE(pose); EXPECT_EQ(pose->header.frame_id, "calibration_optical");
    EXPECT_NE(timing.find("\"image_age_at_callback_s\":null"), std::string::npos);
    EXPECT_NE(timing.find("\"pose_age_at_publish_s\":null"), std::string::npos);
  }
  rclcpp::shutdown();
}
