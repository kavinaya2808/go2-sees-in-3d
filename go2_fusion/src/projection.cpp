#include "go2_fusion/projection.hpp"

#include <cmath>
#include <limits>
#include <tuple>

namespace go2_fusion
{

std::optional<Eigen::Vector2d> projectPoint(
  const Intrinsics & k, const Eigen::Vector3d & p_cam, double min_depth)
{
  const double z = p_cam.z();
  if (z < min_depth) {
    return std::nullopt;
  }
  // Pinhole model: u = fx * x/z + cx, v = fy * y/z + cy
  return Eigen::Vector2d(k.fx * p_cam.x() / z + k.cx, k.fy * p_cam.y() / z + k.cy);
}

std::array<Eigen::Vector3d, 8> boxCorners(
  const Eigen::Vector3d & center, const Eigen::Vector3d & size)
{
  const Eigen::Vector3d h = 0.5 * size;
  std::array<Eigen::Vector3d, 8> corners;
  std::size_t i = 0;
  for (double sx : {-1.0, 1.0}) {
    for (double sy : {-1.0, 1.0}) {
      for (double sz : {-1.0, 1.0}) {
        corners[i++] = center + Eigen::Vector3d(sx * h.x(), sy * h.y(), sz * h.z());
      }
    }
  }
  return corners;
}

std::optional<ImageBox> projectBox(
  const Intrinsics & k, const Eigen::Isometry3d & cam_T_sensor,
  const Eigen::Vector3d & center, const Eigen::Vector3d & size, double min_depth)
{
  constexpr double inf = std::numeric_limits<double>::infinity();
  ImageBox box{inf, inf, -inf, -inf};

  for (const auto & corner : boxCorners(center, size)) {
    const auto uv = projectPoint(k, cam_T_sensor * corner, min_depth);
    if (!uv) {
      return std::nullopt;  // part of the box is behind the camera: not reliably in view
    }
    box.x_min = std::min(box.x_min, uv->x());
    box.y_min = std::min(box.y_min, uv->y());
    box.x_max = std::max(box.x_max, uv->x());
    box.y_max = std::max(box.y_max, uv->y());
  }

  // Clip to the image; a box that is entirely outside ends up with zero area.
  box.x_min = std::clamp(box.x_min, 0.0, static_cast<double>(k.width));
  box.x_max = std::clamp(box.x_max, 0.0, static_cast<double>(k.width));
  box.y_min = std::clamp(box.y_min, 0.0, static_cast<double>(k.height));
  box.y_max = std::clamp(box.y_max, 0.0, static_cast<double>(k.height));
  if (box.width() < 1.0 || box.height() < 1.0) {
    return std::nullopt;
  }
  return box;
}

double iou(const ImageBox & a, const ImageBox & b)
{
  const ImageBox inter{
    std::max(a.x_min, b.x_min), std::max(a.y_min, b.y_min),
    std::min(a.x_max, b.x_max), std::min(a.y_max, b.y_max)};
  const double inter_area = inter.area();
  const double union_area = a.area() + b.area() - inter_area;
  return union_area > 0.0 ? inter_area / union_area : 0.0;
}

double containment(const ImageBox & inner, const ImageBox & outer)
{
  const ImageBox inter{
    std::max(inner.x_min, outer.x_min), std::max(inner.y_min, outer.y_min),
    std::min(inner.x_max, outer.x_max), std::min(inner.y_max, outer.y_max)};
  return inner.area() > 0.0 ? inter.area() / inner.area() : 0.0;
}

std::vector<Match> greedyMatch(
  const std::vector<std::optional<ImageBox>> & clusters,
  const std::vector<ImageBox> & detections, double min_iou)
{
  std::vector<Match> candidates;
  for (std::size_t c = 0; c < clusters.size(); ++c) {
    if (!clusters[c]) {
      continue;  // cluster not visible in the camera
    }
    for (std::size_t d = 0; d < detections.size(); ++d) {
      const double score = iou(*clusters[c], detections[d]);
      if (score >= min_iou) {
        candidates.push_back({c, d, score});
      }
    }
  }
  // Best overlaps first; ties broken by index so the result is deterministic.
  std::sort(
    candidates.begin(), candidates.end(), [](const Match & a, const Match & b) {
      return std::tie(b.iou, a.cluster, a.detection) < std::tie(a.iou, b.cluster, b.detection);
    });

  std::vector<bool> cluster_used(clusters.size(), false);
  std::vector<bool> detection_used(detections.size(), false);
  std::vector<Match> matches;
  for (const auto & m : candidates) {
    if (!cluster_used[m.cluster] && !detection_used[m.detection]) {
      cluster_used[m.cluster] = true;
      detection_used[m.detection] = true;
      matches.push_back(m);
    }
  }
  return matches;
}

double noisyOr(double p_a, double p_b)
{
  return 1.0 - (1.0 - p_a) * (1.0 - p_b);
}

double bearingFromPixel(const Intrinsics & k, double u)
{
  // Optical x points right, so a pixel left of cx has negative x and positive bearing.
  return -std::atan2(u - k.cx, k.fx);
}

}  // namespace go2_fusion
