#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <opencv2/imgproc.hpp>
#include "dock_apriltag_localizer/detection.hpp"

using namespace dock_apriltag_localizer;
TEST(Detection, EstimatesKnownTagAndRejectsBlankImage) {
  DetectionEngine engine(TEST_CONFIG);
  cv::Mat gray(480, 640, CV_8UC1, cv::Scalar(255));
  cv::Mat k = (cv::Mat_<double>(3, 3) << 600, 0, 320, 0, 600, 240, 0, 0, 1);
  cv::Mat d = cv::Mat::zeros(5, 1, CV_64F);
  EXPECT_TRUE(engine.detect(gray, k, d).empty());
  auto * family = tag36h11_create();
  auto * image = apriltag_to_image(family, 0);
  cv::Mat small(image->height, image->width, CV_8UC1, image->buf, image->stride);
  cv::Mat large; cv::resize(small, large, cv::Size(), 15, 15, cv::INTER_NEAREST);
  large.copyTo(gray(cv::Rect(250, 180, large.cols, large.rows)));
  image_u8_destroy(image); tag36h11_destroy(family);
  auto candidates = engine.detect(gray, k, d);
  ASSERT_EQ(candidates.size(), 1u);
  EXPECT_EQ(candidates[0].id, 0);
  EXPECT_FALSE(candidates[0].central);
  EXPECT_NEAR(candidates[0].distance, 1.5, 0.025);
  EXPECT_NEAR((candidates[0].transform.block<3, 3>(0, 0).determinant()), 1.0, 1e-6);
  Eigen::Matrix3d expected;
  expected << -1, 0, 0, 0, 1, 0, 0, 0, -1;
  EXPECT_TRUE((candidates[0].transform.block<3, 3>(0, 0) - expected).norm() < 0.04);
  EXPECT_NEAR(candidates[0].transform(0, 3), 0.5625, 0.03);
  EXPECT_NEAR(candidates[0].transform(1, 3), -0.5125, 0.03);
  // A quarter-turn must rotate the outer-tag offset with the physical tag.
  cv::rotate(large, large, cv::ROTATE_90_COUNTERCLOCKWISE);
  large.copyTo(gray(cv::Rect(250, 180, large.cols, large.rows)));
  const auto rotated = engine.detect(gray, k, d);
  ASSERT_EQ(rotated.size(), 1u);
  expected << 0, 1, 0, 1, 0, 0, 0, 0, -1;
  EXPECT_TRUE((rotated[0].transform.block<3, 3>(0, 0) - expected).norm() < 0.04);
  EXPECT_NEAR(rotated[0].transform(0, 3), -0.5375, 0.03);
  EXPECT_NEAR(rotated[0].transform(1, 3), -0.5125, 0.03);
  EXPECT_EQ(engine.tag_count(), 5u);
}
TEST(Detection, InvalidReloadCanBeRejectedBeforeReplacingDetector) {
  EXPECT_THROW(DetectionEngine("/no/such/tag/config.yaml"), std::exception);
}
