#include "go2_perception/cloud_pipeline.hpp"

#include <algorithm>
#include <cmath>

#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

namespace go2_perception
{

namespace
{
constexpr float kPi = 3.14159265358979f;

Cloud::Ptr copyOf(const Cloud::ConstPtr & in)
{
  return Cloud::Ptr(new Cloud(*in));
}
}  // namespace

Cloud::Ptr removeInvalid(const Cloud::ConstPtr & in)
{
  // Gazebo marks rays that hit nothing as NaN/inf; PCL filters expect finite points.
  Cloud::Ptr out(new Cloud);
  out->header = in->header;
  out->reserve(in->size());
  for (const auto & p : in->points) {
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
      out->push_back(p);
    }
  }
  out->is_dense = true;
  return out;
}

Cloud::Ptr cropBox(
  const Cloud::ConstPtr & in, const Eigen::Vector3f & min_pt,
  const Eigen::Vector3f & max_pt, bool negative)
{
  Cloud::Ptr out(new Cloud);
  if (in->empty()) {
    return out;
  }
  pcl::CropBox<Point> crop;
  crop.setInputCloud(in);
  // CropBox takes homogeneous coordinates, hence the trailing 1.
  crop.setMin(Eigen::Vector4f(min_pt.x(), min_pt.y(), min_pt.z(), 1.0f));
  crop.setMax(Eigen::Vector4f(max_pt.x(), max_pt.y(), max_pt.z(), 1.0f));
  crop.setNegative(negative);  // true = keep everything OUTSIDE the box
  crop.filter(*out);
  return out;
}

Cloud::Ptr voxelDownsample(const Cloud::ConstPtr & in, float leaf_size)
{
  if (in->empty() || leaf_size <= 0.0f) {
    return copyOf(in);
  }
  Cloud::Ptr out(new Cloud);
  pcl::VoxelGrid<Point> voxel;
  voxel.setInputCloud(in);
  voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel.filter(*out);
  return out;
}

Cloud::Ptr removeGround(
  const Cloud::ConstPtr & in, const PipelineConfig & cfg,
  std::size_t * ground_count)
{
  if (ground_count != nullptr) {
    *ground_count = 0;
  }
  if (in->size() < 3) {
    return copyOf(in);
  }

  // PERPENDICULAR_PLANE = a plane perpendicular to the given axis, i.e. a horizontal
  // plane when the axis is z. This stops RANSAC from picking a box side as "ground".
  pcl::SACSegmentation<Point> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setAxis(Eigen::Vector3f::UnitZ());
  seg.setEpsAngle(cfg.ground_max_angle_deg * kPi / 180.0f);
  seg.setDistanceThreshold(cfg.ground_distance);
  seg.setMaxIterations(cfg.ground_max_iterations);
  seg.setInputCloud(in);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients coefficients;
  seg.segment(*inliers, coefficients);

  if (static_cast<int>(inliers->indices.size()) < cfg.ground_min_inliers) {
    return copyOf(in);  // no convincing ground plane in view
  }
  if (ground_count != nullptr) {
    *ground_count = inliers->indices.size();
  }

  Cloud::Ptr out(new Cloud);
  pcl::ExtractIndices<Point> extract;
  extract.setInputCloud(in);
  extract.setIndices(inliers);
  extract.setNegative(true);  // keep the points that are NOT ground
  extract.filter(*out);
  return out;
}

std::vector<Cluster> extractClusters(const Cloud::ConstPtr & in, const PipelineConfig & cfg)
{
  std::vector<Cluster> clusters;
  if (in->empty()) {
    return clusters;
  }

  // A k-d tree makes the "all neighbours within tolerance" queries fast.
  pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>);
  tree->setInputCloud(in);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<Point> ec;
  ec.setClusterTolerance(cfg.cluster_tolerance);
  ec.setMinClusterSize(cfg.min_cluster_size);
  ec.setMaxClusterSize(cfg.max_cluster_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(in);
  ec.extract(cluster_indices);

  clusters.reserve(cluster_indices.size());
  for (const auto & indices : cluster_indices) {
    Eigen::Vector4f min4, max4, centroid4;
    pcl::getMinMax3D(*in, indices.indices, min4, max4);
    pcl::compute3DCentroid(*in, indices.indices, centroid4);

    Cluster c;
    c.min_pt = min4.head<3>();
    c.max_pt = max4.head<3>();
    c.centroid = centroid4.head<3>();
    c.num_points = indices.indices.size();

    const Eigen::Vector3f size = c.size();
    if (size.z() < cfg.min_cluster_height) {
      continue;
    }
    if (std::max(size.x(), size.y()) > cfg.max_cluster_extent) {
      continue;
    }
    clusters.push_back(c);
  }

  std::sort(
    clusters.begin(), clusters.end(),
    [](const Cluster & a, const Cluster & b) {return a.range() < b.range();});
  return clusters;
}

PipelineResult runPipeline(const Cloud::ConstPtr & in, const PipelineConfig & cfg)
{
  PipelineResult result;
  Cloud::Ptr cloud = removeInvalid(in);
  cloud = cropBox(cloud, cfg.roi_min, cfg.roi_max, false);
  cloud = cropBox(cloud, cfg.self_min, cfg.self_max, true);
  cloud = voxelDownsample(cloud, cfg.voxel_leaf);
  cloud = removeGround(cloud, cfg, &result.ground_points);
  result.clusters = extractClusters(cloud, cfg);
  result.obstacles = cloud;
  return result;
}

}  // namespace go2_perception
