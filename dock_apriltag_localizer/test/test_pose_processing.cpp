#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include "dock_apriltag_localizer/pose_processing.hpp"

using namespace dock_apriltag_localizer;

static Pose pose(double x = 0.0, double z = 2.0) {
  Pose t = Pose::Identity(); t(0, 3) = x; t(2, 3) = z; return t;
}
static Candidate candidate(int id, bool central, double x, double distance = 1.0) {
  Candidate c; c.id = id; c.central = central; c.transform = pose(x);
  c.distance = distance; c.weight = 1.0; return c;
}
TEST(Geometry, CorrectsPhysicalTagNormalAndOffset) {
  const auto r = correct_pnp_rotation(Eigen::Matrix3d::Identity());
  EXPECT_TRUE(r.isApprox(Eigen::Vector3d(-1, 1, -1).asDiagonal().toDenseMatrix(), 1e-12));
  Pose tag = pose(0.68, 4.901); tag(1, 3) = 1.03;
  tag.block<3, 3>(0, 0) = r;
  const Pose dock = tag * make_transform({0.55, 0.55, 0.001}, {0, 0, 0}).inverse();
  EXPECT_NEAR(dock(0, 3), 1.23, 1e-12);
  EXPECT_NEAR(dock(1, 3), 0.48, 1e-12);
  EXPECT_NEAR(dock(2, 3), 4.902, 1e-12);
}
TEST(Fusion, RejectsMedianOutlierAndWeightsRemainingPoses) {
  auto a = candidate(0, false, 0.0), b = candidate(1, false, 0.1);
  b.weight = 3.0;
  const auto result = fuse({a, b, candidate(2, false, 10.0)}, 0.35);
  ASSERT_TRUE(result); EXPECT_NEAR((*result)(0, 3), 0.075, 1e-12);
  EXPECT_NEAR((result->block<3, 3>(0, 0).determinant()), 1.0, 1e-12);
  EXPECT_FALSE(fuse({}, 0.35));
}
TEST(Switching, CentralRequiresStableFramesAndSurvivesDropouts) {
  PoseSelector s;
  const auto outer = candidate(0, false, 1), central = candidate(4, true, 0);
  for (int i = 0; i < 3; ++i) EXPECT_EQ(s.select({outer, central}).mode, "OUTER");
  EXPECT_EQ(s.select({outer, central}).mode, "CENTRAL");
  for (int i = 0; i < 5; ++i) EXPECT_EQ(s.select({outer}).mode, "OUTER");
  EXPECT_EQ(s.select({outer, central}).mode, "CENTRAL");
  for (int i = 0; i < 6; ++i) s.select({outer});
  EXPECT_EQ(s.select({outer, central}).mode, "OUTER");
  s.reset(); EXPECT_EQ(s.select({central}).mode, "CENTRAL_ACQUIRING");
  EXPECT_EQ(s.select({candidate(4, true, 0, 3.0)}).mode, "NO_TARGET");
}
TEST(Filter, SmoothsWithAcquisitionTimeAndKeepsProperRotation) {
  TemporalPoseFilter f;
  EXPECT_EQ(f.update(pose(), 1).status, "initialized");
  auto t = pose(0.1); t.block<3, 3>(0, 0) = rpy_to_rotation({0, 0, 0.3});
  const auto result = f.update(t, 1.12);
  ASSERT_TRUE(result.pose); EXPECT_EQ(result.status, "accepted");
  EXPECT_NEAR((*result.pose)(0, 3), 0.0632120558828558, 1e-12);
  EXPECT_NEAR((result.pose->block<3, 3>(0, 0).determinant()), 1.0, 1e-12);
}
TEST(Filter, RejectsJumpAndReacquiresThreeConsistentFrames) {
  TemporalPoseFilter f; f.update(pose(), 1.0);
  EXPECT_EQ(f.update(pose(2), 1.1).status, "outlier_rejected");
  EXPECT_FALSE(f.update(pose(2.02), 1.2).pose);
  const auto result = f.update(pose(2.04), 1.3);
  ASSERT_TRUE(result.pose); EXPECT_EQ(result.status, "reacquired");
  EXPECT_NEAR((*result.pose)(0, 3), 2.025, 1e-12);
}
TEST(Filter, ChecksTimestampsAndResetsAfterGap) {
  TemporalPoseFilter f; f.update(pose(), 1);
  EXPECT_EQ(f.update(pose(), 1).status, "non_monotonic_timestamp");
  EXPECT_EQ(f.update(pose(), 0).status, "non_monotonic_timestamp");
  EXPECT_EQ(f.update(pose(5), 2.1).status, "reset_after_gap");
  f.reset(); EXPECT_EQ(f.update(pose(), 0).status, "initialized");
}
TEST(Filter, CapsGateAndRestartsInconsistentReacquisition) {
  TemporalPoseFilter f; f.update(pose(), 1);
  EXPECT_EQ(f.update(pose(1), 1.9).status, "outlier_rejected");
  EXPECT_EQ(f.update(pose(3), 2).status, "outlier_rejected");
  EXPECT_EQ(f.update(pose(3), 2.1).status, "outlier_rejected");
  EXPECT_EQ(f.update(pose(3), 2.2).status, "reacquired");
}
TEST(Filter, DisabledPassesThroughAndRejectsInvalidConfiguration) {
  FilterOptions options; options.enabled = false; TemporalPoseFilter f(options);
  auto result = f.update(pose(3), 0);
  ASSERT_TRUE(result.pose); EXPECT_EQ(result.status, "disabled");
  EXPECT_DOUBLE_EQ((*result.pose)(0, 3), 3);
  options.time_constant_s = -1;
  EXPECT_THROW(TemporalPoseFilter{options}, std::invalid_argument);
}
