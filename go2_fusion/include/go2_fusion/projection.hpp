// Camera projection and 2D/3D association maths. No ROS in here, so it is unit-testable.
//
// Frames: the camera frame is the OPTICAL frame (z forward, x right, y down), which is
// what the pinhole model below assumes. ROS "camera_link" frames (x forward) are not.
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <optional>
#include <vector>

#include <Eigen/Geometry>

namespace go2_fusion
{

// Pinhole intrinsics, taken from the K matrix of sensor_msgs/CameraInfo.
struct Intrinsics
{
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  int width{0};
  int height{0};

  bool valid() const {return fx > 0.0 && fy > 0.0 && width > 0 && height > 0;}
};

// Axis-aligned box in pixel coordinates.
struct ImageBox
{
  double x_min{0.0};
  double y_min{0.0};
  double x_max{0.0};
  double y_max{0.0};

  double width() const {return std::max(0.0, x_max - x_min);}
  double height() const {return std::max(0.0, y_max - y_min);}
  double area() const {return width() * height();}
  double centerX() const {return 0.5 * (x_min + x_max);}
};

// Projects a point given in the camera optical frame. Returns nothing if the point is
// closer than min_depth or behind the camera.
std::optional<Eigen::Vector2d> projectPoint(
  const Intrinsics & k, const Eigen::Vector3d & p_cam, double min_depth = 0.1);

// The 8 corners of an axis-aligned 3D box.
std::array<Eigen::Vector3d, 8> boxCorners(
  const Eigen::Vector3d & center, const Eigen::Vector3d & size);

// Projects a 3D box given in the sensor frame into the image.
// cam_T_sensor maps sensor-frame points into the camera optical frame (this is what
// tf2 lookupTransform(camera_frame, sensor_frame) returns).
// Returns the 2D box around all 8 projected corners, clipped to the image, or nothing
// if any corner is behind the camera or the box falls outside the image.
std::optional<ImageBox> projectBox(
  const Intrinsics & k, const Eigen::Isometry3d & cam_T_sensor,
  const Eigen::Vector3d & center, const Eigen::Vector3d & size, double min_depth = 0.1);

// Intersection over union of two image boxes, in [0, 1].
double iou(const ImageBox & a, const ImageBox & b);

// Fraction of `inner` that lies inside `outer`, in [0, 1]. Used to spot detection
// fragments, e.g. an object split in two by something standing in front of it.
double containment(const ImageBox & inner, const ImageBox & outer);

struct Match
{
  std::size_t cluster;
  std::size_t detection;
  double iou;
};

// Greedy one-to-one association: repeatedly take the highest-IoU (cluster, detection)
// pair whose members are both still free, as long as IoU >= min_iou.
std::vector<Match> greedyMatch(
  const std::vector<std::optional<ImageBox>> & clusters,
  const std::vector<ImageBox> & detections, double min_iou);

// Probability that at least one of two independent sensors is right ("noisy-OR").
double noisyOr(double p_a, double p_b);

// Horizontal bearing of an image column, in radians. Positive = left of the optical
// axis, matching the ROS convention (y left, counter-clockwise positive).
double bearingFromPixel(const Intrinsics & k, double u);

}  // namespace go2_fusion
