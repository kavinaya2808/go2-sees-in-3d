#include <gtest/gtest.h>

#include <cmath>

#include "go2_fusion/projection.hpp"

using go2_fusion::ImageBox;
using go2_fusion::Intrinsics;

namespace
{

// The simulated D455 colour camera: 640x480, fx = fy = 347.3
Intrinsics d455()
{
  Intrinsics k;
  k.fx = 347.306851365299;
  k.fy = 347.306851365299;
  k.cx = 320.0;
  k.cy = 240.0;
  k.width = 640;
  k.height = 480;
  return k;
}

// Sensor frame (x forward, y left, z up) -> camera optical frame (z forward, x right, y down),
// with both frames at the same origin. This is the rotation every LiDAR-camera pair needs.
Eigen::Isometry3d sensorToOptical()
{
  Eigen::Matrix3d r;
  r << 0, -1, 0,   // optical x = -sensor y
    0, 0, -1,      // optical y = -sensor z
    1, 0, 0;       // optical z =  sensor x
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.linear() = r;
  return t;
}

}  // namespace

TEST(Projection, PointOnOpticalAxisHitsPrincipalPoint)
{
  const auto uv = go2_fusion::projectPoint(d455(), Eigen::Vector3d(0, 0, 5));
  ASSERT_TRUE(uv.has_value());
  EXPECT_NEAR(uv->x(), 320.0, 1e-9);
  EXPECT_NEAR(uv->y(), 240.0, 1e-9);
}

TEST(Projection, OffAxisPointFollowsPinholeModel)
{
  // 1 m right and 0.5 m down at 2 m depth
  const auto uv = go2_fusion::projectPoint(d455(), Eigen::Vector3d(1.0, 0.5, 2.0));
  ASSERT_TRUE(uv.has_value());
  EXPECT_NEAR(uv->x(), 320.0 + 347.306851365299 * 0.5, 1e-9);
  EXPECT_NEAR(uv->y(), 240.0 + 347.306851365299 * 0.25, 1e-9);
}

TEST(Projection, PointsBehindOrTooCloseAreRejected)
{
  EXPECT_FALSE(go2_fusion::projectPoint(d455(), Eigen::Vector3d(0, 0, -1)).has_value());
  EXPECT_FALSE(go2_fusion::projectPoint(d455(), Eigen::Vector3d(0, 0, 0.05)).has_value());
}

TEST(Projection, BoxStraightAheadOfLidarIsCentredInImage)
{
  // 0.5 m cube 4 m in front of the sensor
  const auto box = go2_fusion::projectBox(
    d455(), sensorToOptical(), Eigen::Vector3d(4, 0, 0), Eigen::Vector3d(0.5, 0.5, 0.5));
  ASSERT_TRUE(box.has_value());
  EXPECT_NEAR(box->centerX(), 320.0, 1e-6);
  EXPECT_NEAR(0.5 * (box->y_min + box->y_max), 240.0, 1e-6);
  // Near face at 3.75 m: half-width 0.25 m -> 347.3 * 0.25 / 3.75 = 23.15 px
  EXPECT_NEAR(box->width(), 2 * 347.306851365299 * 0.25 / 3.75, 1e-6);
}

TEST(Projection, ObjectToTheLeftAppearsLeftInImage)
{
  const auto box = go2_fusion::projectBox(
    d455(), sensorToOptical(), Eigen::Vector3d(4, 1, 0), Eigen::Vector3d(0.4, 0.4, 0.4));
  ASSERT_TRUE(box.has_value());
  EXPECT_LT(box->centerX(), 320.0);
}

TEST(Projection, BoxBehindTheCameraIsNotVisible)
{
  EXPECT_FALSE(go2_fusion::projectBox(
    d455(), sensorToOptical(), Eigen::Vector3d(-4, 0, 0), Eigen::Vector3d(0.5, 0.5, 0.5)));
}

TEST(Projection, BoxOutsideFieldOfViewIsNotVisible)
{
  // Far to the left: ~80 deg off-axis, outside the ~85 deg horizontal FOV (+-42.6 deg)
  EXPECT_FALSE(go2_fusion::projectBox(
    d455(), sensorToOptical(), Eigen::Vector3d(1, 6, 0), Eigen::Vector3d(0.3, 0.3, 0.3)));
}

TEST(Projection, CameraOffsetFromLidarShiftsTheProjection)
{
  // Camera mounted 0.2 m to the LEFT of the LiDAR: an object straight ahead of the
  // LiDAR is to the camera's right, so it must land right of the image centre.
  Eigen::Isometry3d cam_T_sensor = sensorToOptical();
  // optical x is "right"; camera sits 0.2 m left => sensor points shift +0.2 m in optical x
  cam_T_sensor.translation() = Eigen::Vector3d(0.2, 0, 0);
  const auto box = go2_fusion::projectBox(
    d455(), cam_T_sensor, Eigen::Vector3d(4, 0, 0), Eigen::Vector3d(0.5, 0.5, 0.5));
  ASSERT_TRUE(box.has_value());
  EXPECT_GT(box->centerX(), 320.0);
}

TEST(Iou, KnownValues)
{
  const ImageBox a{0, 0, 10, 10};
  EXPECT_DOUBLE_EQ(go2_fusion::iou(a, a), 1.0);
  EXPECT_DOUBLE_EQ(go2_fusion::iou(a, ImageBox{20, 20, 30, 30}), 0.0);
  // Half overlap: intersection 50, union 150
  EXPECT_NEAR(go2_fusion::iou(a, ImageBox{5, 0, 15, 10}), 1.0 / 3.0, 1e-12);
}

TEST(Matching, PicksBestOverlapOneToOne)
{
  std::vector<std::optional<ImageBox>> clusters{
    ImageBox{0, 0, 100, 100},
    ImageBox{200, 0, 300, 100},
    std::nullopt};  // cluster outside the camera view
  const std::vector<ImageBox> detections{
    ImageBox{210, 0, 300, 100},  // overlaps cluster 1
    ImageBox{10, 10, 100, 100},  // overlaps cluster 0
    ImageBox{500, 300, 600, 400}};  // matches nothing

  const auto matches = go2_fusion::greedyMatch(clusters, detections, 0.2);
  ASSERT_EQ(matches.size(), 2u);
  for (const auto & m : matches) {
    if (m.cluster == 0) {
      EXPECT_EQ(m.detection, 1u);
    } else {
      EXPECT_EQ(m.cluster, 1u);
      EXPECT_EQ(m.detection, 0u);
    }
  }
}

TEST(Matching, RespectsIouThreshold)
{
  std::vector<std::optional<ImageBox>> clusters{ImageBox{0, 0, 100, 100}};
  const std::vector<ImageBox> detections{ImageBox{90, 90, 190, 190}};  // tiny overlap
  EXPECT_TRUE(go2_fusion::greedyMatch(clusters, detections, 0.2).empty());
}

TEST(Fusion, NoisyOrAndBearing)
{
  EXPECT_NEAR(go2_fusion::noisyOr(0.6, 0.9), 0.96, 1e-12);
  EXPECT_NEAR(go2_fusion::noisyOr(0.6, 0.0), 0.6, 1e-12);

  const auto k = d455();
  EXPECT_NEAR(go2_fusion::bearingFromPixel(k, 320.0), 0.0, 1e-12);
  EXPECT_GT(go2_fusion::bearingFromPixel(k, 100.0), 0.0);  // left of centre
  EXPECT_LT(go2_fusion::bearingFromPixel(k, 600.0), 0.0);  // right of centre
  // Image edge = half the horizontal FOV, ~42.66 deg
  EXPECT_NEAR(go2_fusion::bearingFromPixel(k, 0.0) * 180.0 / M_PI, 42.66, 0.05);
}
