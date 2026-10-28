// ROS 2 wrapper around the PCL pipeline.
//
// Subscribes: sensor_msgs/PointCloud2           (input_topic, default /velodyne_points/points)
// Publishes:  /perception/clusters               vision_msgs/Detection3DArray  -> fusion node
//             /perception/cluster_markers        visualization_msgs/MarkerArray -> RViz
//             /perception/obstacle_points        sensor_msgs/PointCloud2 (debug: cloud after ground removal)
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "go2_perception/cloud_pipeline.hpp"

namespace go2_perception
{

namespace
{

Eigen::Vector3f toVec3(const std::vector<double> & v, const std::string & name)
{
  if (v.size() != 3) {
    throw std::invalid_argument("parameter '" + name + "' needs exactly 3 values");
  }
  return {static_cast<float>(v[0]), static_cast<float>(v[1]), static_cast<float>(v[2])};
}

std::string formatLabel(std::size_t id, float range)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "#%zu  %.1f m", id, range);
  return buf;
}

}  // namespace

class ClusterNode : public rclcpp::Node
{
public:
  explicit ClusterNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("cluster_node", options)
  {
    const auto input_topic =
      declare_parameter<std::string>("input_topic", "/velodyne_points/points");

    cfg_.roi_min = toVec3(declare_parameter("roi_min", std::vector<double>{-10.0, -10.0, -1.0}), "roi_min");
    cfg_.roi_max = toVec3(declare_parameter("roi_max", std::vector<double>{10.0, 10.0, 1.5}), "roi_max");
    cfg_.self_min = toVec3(declare_parameter("self_min", std::vector<double>{-0.6, -0.35, -1.0}), "self_min");
    cfg_.self_max = toVec3(declare_parameter("self_max", std::vector<double>{0.6, 0.35, 0.3}), "self_max");
    cfg_.voxel_leaf = static_cast<float>(declare_parameter("voxel_leaf", 0.08));
    cfg_.ground_distance = static_cast<float>(declare_parameter("ground_distance", 0.08));
    cfg_.ground_max_angle_deg = static_cast<float>(declare_parameter("ground_max_angle_deg", 15.0));
    cfg_.ground_max_iterations = static_cast<int>(declare_parameter("ground_max_iterations", 100));
    cfg_.ground_min_inliers = static_cast<int>(declare_parameter("ground_min_inliers", 50));
    cfg_.cluster_tolerance = static_cast<float>(declare_parameter("cluster_tolerance", 0.35));
    cfg_.min_cluster_size = static_cast<int>(declare_parameter("min_cluster_size", 8));
    cfg_.max_cluster_size = static_cast<int>(declare_parameter("max_cluster_size", 5000));
    cfg_.min_cluster_height = static_cast<float>(declare_parameter("min_cluster_height", 0.10));
    cfg_.max_cluster_extent = static_cast<float>(declare_parameter("max_cluster_extent", 4.0));

    detection_pub_ = create_publisher<vision_msgs::msg::Detection3DArray>("perception/clusters", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("perception/cluster_markers", 1);
    obstacle_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("perception/obstacle_points", 1);

    // SensorDataQoS = best effort, small queue: we always want the newest scan, never a backlog.
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {onCloud(*msg);});

    RCLCPP_INFO(get_logger(), "Clustering point clouds from '%s'", input_topic.c_str());
  }

private:
  void onCloud(const sensor_msgs::msg::PointCloud2 & msg)
  {
    const auto t0 = std::chrono::steady_clock::now();

    Cloud::Ptr cloud(new Cloud);
    pcl::fromROSMsg(msg, *cloud);  // keeps x, y, z; intensity and ring are dropped
    const PipelineResult result = runPipeline(cloud, cfg_);

    const double ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    ++frames_;
    total_ms_ += ms;

    publishDetections(msg.header, result.clusters);
    publishMarkers(msg.header, result.clusters);
    if (obstacle_pub_->get_subscription_count() > 0) {
      sensor_msgs::msg::PointCloud2 out;
      pcl::toROSMsg(*result.obstacles, out);
      out.header = msg.header;
      obstacle_pub_->publish(out);
    }

    // Steady clock, not sim time, so the log rate doesn't depend on the sim's speed.
    RCLCPP_INFO_THROTTLE(
      get_logger(), steady_clock_, 5000,
      "%zu clusters | %zu ground pts | pipeline %.1f ms (avg %.1f ms over %zu scans)",
      result.clusters.size(), result.ground_points, ms, total_ms_ / frames_, frames_);
  }

  void publishDetections(const std_msgs::msg::Header & header, const std::vector<Cluster> & clusters)
  {
    vision_msgs::msg::Detection3DArray array;
    array.header = header;
    for (std::size_t i = 0; i < clusters.size(); ++i) {
      const auto & c = clusters[i];
      const Eigen::Vector3f center = c.center();
      const Eigen::Vector3f size = c.size();

      vision_msgs::msg::Detection3D det;
      det.header = header;
      det.id = std::to_string(i);
      det.bbox.center.position.x = center.x();
      det.bbox.center.position.y = center.y();
      det.bbox.center.position.z = center.z();
      det.bbox.center.orientation.w = 1.0;  // axis-aligned box in the sensor frame
      det.bbox.size.x = size.x();
      det.bbox.size.y = size.y();
      det.bbox.size.z = size.z();
      array.detections.push_back(det);
    }
    detection_pub_->publish(array);
  }

  void publishMarkers(const std_msgs::msg::Header & header, const std::vector<Cluster> & clusters)
  {
    using visualization_msgs::msg::Marker;
    visualization_msgs::msg::MarkerArray array;

    // Clear last scan's markers first, otherwise boxes of vanished clusters linger in RViz.
    Marker clear;
    clear.header = header;
    clear.action = Marker::DELETEALL;
    array.markers.push_back(clear);

    for (std::size_t i = 0; i < clusters.size(); ++i) {
      const auto & c = clusters[i];
      const Eigen::Vector3f center = c.center();
      const Eigen::Vector3f size = c.size();

      Marker box;
      box.header = header;
      box.ns = "cluster_boxes";
      box.id = static_cast<int>(i);
      box.type = Marker::CUBE;
      box.action = Marker::ADD;
      box.pose.position.x = center.x();
      box.pose.position.y = center.y();
      box.pose.position.z = center.z();
      box.pose.orientation.w = 1.0;
      box.scale.x = std::max(size.x(), 0.05f);
      box.scale.y = std::max(size.y(), 0.05f);
      box.scale.z = std::max(size.z(), 0.05f);
      box.color.r = 0.1f;
      box.color.g = 0.8f;
      box.color.b = 1.0f;
      box.color.a = 0.35f;
      array.markers.push_back(box);

      Marker label;
      label.header = header;
      label.ns = "cluster_labels";
      label.id = static_cast<int>(i);
      label.type = Marker::TEXT_VIEW_FACING;
      label.action = Marker::ADD;
      label.pose.position.x = center.x();
      label.pose.position.y = center.y();
      label.pose.position.z = c.max_pt.z() + 0.25;
      label.pose.orientation.w = 1.0;
      label.scale.z = 0.25;  // text height in metres
      label.color.r = label.color.g = label.color.b = label.color.a = 1.0f;
      label.text = formatLabel(i, c.range());
      array.markers.push_back(label);
    }
    marker_pub_->publish(array);
  }

  PipelineConfig cfg_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  std::size_t frames_{0};
  double total_ms_{0.0};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detection_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
};

}  // namespace go2_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<go2_perception::ClusterNode>());
  rclcpp::shutdown();
  return 0;
}
