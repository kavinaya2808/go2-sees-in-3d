// Pure PCL point-cloud pipeline: no ROS in here, so every step is unit-testable.
//
//   raw cloud -> NaN removal -> ROI crop -> self-body crop -> voxel grid
//             -> RANSAC ground removal -> Euclidean clustering -> 3D boxes
#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace go2_perception
{

using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;

struct PipelineConfig
{
  // Region of interest, in the sensor frame (metres).
  Eigen::Vector3f roi_min{-10.0f, -10.0f, -1.0f};
  Eigen::Vector3f roi_max{10.0f, 10.0f, 1.5f};

  // Box around the robot's own body and legs, removed from the cloud.
  Eigen::Vector3f self_min{-0.6f, -0.35f, -1.0f};
  Eigen::Vector3f self_max{0.6f, 0.35f, 0.3f};

  float voxel_leaf{0.08f};  // <= 0 disables downsampling

  // RANSAC ground plane: a plane roughly perpendicular to the sensor's z axis.
  float ground_distance{0.08f};       // inlier distance to the plane
  float ground_max_angle_deg{15.0f};  // allowed tilt of the plane normal from z
  int ground_max_iterations{100};
  int ground_min_inliers{50};         // fewer inliers -> treat as "no ground found"

  // Euclidean clustering.
  float cluster_tolerance{0.35f};
  int min_cluster_size{8};
  int max_cluster_size{5000};

  // Cluster sanity filters.
  float min_cluster_height{0.10f};  // drops flat leftovers of the ground
  float max_cluster_extent{4.0f};   // drops walls and other very long structures
};

struct Cluster
{
  Eigen::Vector3f min_pt{Eigen::Vector3f::Zero()};
  Eigen::Vector3f max_pt{Eigen::Vector3f::Zero()};
  Eigen::Vector3f centroid{Eigen::Vector3f::Zero()};
  std::size_t num_points{0};

  Eigen::Vector3f center() const {return 0.5f * (min_pt + max_pt);}
  Eigen::Vector3f size() const {return max_pt - min_pt;}
  // Horizontal distance from the sensor to the centroid: the "4.2 m" in a label.
  float range() const {return centroid.head<2>().norm();}
};

struct PipelineResult
{
  Cloud::Ptr obstacles;            // points left after cropping, downsampling and ground removal
  std::vector<Cluster> clusters;   // sorted by range, nearest first
  std::size_t ground_points{0};
};

Cloud::Ptr removeInvalid(const Cloud::ConstPtr & in);

Cloud::Ptr cropBox(
  const Cloud::ConstPtr & in, const Eigen::Vector3f & min_pt,
  const Eigen::Vector3f & max_pt, bool negative);

Cloud::Ptr voxelDownsample(const Cloud::ConstPtr & in, float leaf_size);

// Returns the non-ground points. If ground_count is given, it receives the number of
// points classified as ground.
Cloud::Ptr removeGround(
  const Cloud::ConstPtr & in, const PipelineConfig & cfg,
  std::size_t * ground_count = nullptr);

std::vector<Cluster> extractClusters(const Cloud::ConstPtr & in, const PipelineConfig & cfg);

PipelineResult runPipeline(const Cloud::ConstPtr & in, const PipelineConfig & cfg);

}  // namespace go2_perception
