// LiDAR-camera fusion.
//
// For every LiDAR cluster array:
//   1. project each 3D cluster box into the camera image (TF + intrinsics)
//   2. match projected boxes to 2D detections by IoU (greedy, one-to-one)
//   3. publish fused objects, a JSON assessment per object, RViz markers and a debug image
//
// Subscribes: /perception/clusters        vision_msgs/Detection3DArray (LiDAR frame)
//             /perception/detections_2d   vision_msgs/Detection2DArray (pixels)
//             camera_info_topic, image_topic
// Publishes:  /fusion/objects             vision_msgs/Detection3DArray (with class + score)
//             /fusion/alerts              std_msgs/String (JSON)
//             /fusion/markers             visualization_msgs/MarkerArray
//             /fusion/debug_image         sensor_msgs/Image (only when someone is watching)
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "go2_fusion/projection.hpp"

namespace go2_fusion
{

namespace
{

constexpr double kRadToDeg = 180.0 / M_PI;

enum class Sensors { kLidar, kCamera, kBoth };

const char * toString(Sensors s)
{
  switch (s) {
    case Sensors::kLidar: return "lidar";
    case Sensors::kCamera: return "camera";
    case Sensors::kBoth: return "lidar+camera";
  }
  return "?";
}

const char * shortTag(Sensors s)
{
  switch (s) {
    case Sensors::kLidar: return "L";
    case Sensors::kCamera: return "C";
    case Sensors::kBoth: return "L+C";
  }
  return "?";
}

struct FusedObject
{
  std::string class_id{"unknown"};
  double confidence{0.0};
  Sensors sensors{Sensors::kLidar};
  bool has_position{false};             // false for camera-only objects
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};  // box centre, LiDAR frame
  Eigen::Vector3d size{Eigen::Vector3d::Zero()};
  double distance{0.0};                 // horizontal range from the LiDAR, metres
  double bearing_deg{0.0};              // positive = left
  std::optional<ImageBox> image_box;
  bool alert{false};
};

ImageBox toImageBox(const vision_msgs::msg::BoundingBox2D & b)
{
  const double cx = b.center.position.x;
  const double cy = b.center.position.y;
  return {cx - 0.5 * b.size_x, cy - 0.5 * b.size_y, cx + 0.5 * b.size_x, cy + 0.5 * b.size_y};
}

}  // namespace

class FusionNode : public rclcpp::Node
{
public:
  FusionNode()
  : Node("fusion_node")
  {
    camera_frame_ = declare_parameter<std::string>("camera_frame", "d455_camera_color_optical_frame");
    min_iou_ = declare_parameter("min_iou", 0.15);
    max_detection_age_ = declare_parameter("max_detection_age", 1.0);
    lidar_confidence_ = declare_parameter("lidar_confidence", 0.6);
    min_depth_ = declare_parameter("min_depth", 0.3);
    alert_classes_ = declare_parameter("alert_classes", std::vector<std::string>{"person"});
    alert_radius_ = declare_parameter("alert_radius", 5.0);

    const auto clusters_topic = declare_parameter<std::string>("clusters_topic", "/perception/clusters");
    const auto detections_topic =
      declare_parameter<std::string>("detections_topic", "/perception/detections_2d");
    const auto info_topic = declare_parameter<std::string>("camera_info_topic", "/d455/camera_info");
    const auto image_topic = declare_parameter<std::string>("image_topic", "/d455/image");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    objects_pub_ = create_publisher<vision_msgs::msg::Detection3DArray>("fusion/objects", 10);
    alerts_pub_ = create_publisher<std_msgs::msg::String>("fusion/alerts", 10);
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("fusion/markers", 1);
    debug_pub_ = create_publisher<sensor_msgs::msg::Image>("fusion/debug_image", 1);

    clusters_sub_ = create_subscription<vision_msgs::msg::Detection3DArray>(
      clusters_topic, 10,
      [this](vision_msgs::msg::Detection3DArray::ConstSharedPtr msg) {onClusters(*msg);});
    detections_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      detections_topic, 10,
      [this](vision_msgs::msg::Detection2DArray::ConstSharedPtr msg) {latest_detections_ = msg;});
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      info_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {onCameraInfo(*msg);});
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {latest_image_ = msg;});

    RCLCPP_INFO(get_logger(), "Fusing '%s' with '%s' in camera frame '%s'",
      clusters_topic.c_str(), detections_topic.c_str(), camera_frame_.c_str());
  }

private:
  void onCameraInfo(const sensor_msgs::msg::CameraInfo & msg)
  {
    if (intrinsics_.valid()) {
      return;  // static camera, read once
    }
    // K is row-major 3x3: [fx 0 cx; 0 fy cy; 0 0 1]. We use K, not P: P can carry a
    // stereo baseline term that doesn't apply to a single colour camera.
    intrinsics_.fx = msg.k[0];
    intrinsics_.cx = msg.k[2];
    intrinsics_.fy = msg.k[4];
    intrinsics_.cy = msg.k[5];
    intrinsics_.width = static_cast<int>(msg.width);
    intrinsics_.height = static_cast<int>(msg.height);
    RCLCPP_INFO(get_logger(), "Camera intrinsics: %dx%d fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
      intrinsics_.width, intrinsics_.height, intrinsics_.fx, intrinsics_.fy,
      intrinsics_.cx, intrinsics_.cy);
  }

  void onClusters(const vision_msgs::msg::Detection3DArray & clusters)
  {
    if (!intrinsics_.valid()) {
      RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 5000, "Waiting for camera_info...");
      return;
    }

    Eigen::Isometry3d cam_T_sensor;
    try {
      // Static mount, so the latest transform is the right one.
      cam_T_sensor = tf2::transformToEigen(
        tf_buffer_->lookupTransform(camera_frame_, clusters.header.frame_id, tf2::TimePointZero));
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 5000, "TF %s -> %s failed: %s",
        clusters.header.frame_id.c_str(), camera_frame_.c_str(), e.what());
      return;
    }

    // 1. Project every LiDAR cluster into the image.
    std::vector<std::optional<ImageBox>> projected;
    projected.reserve(clusters.detections.size());
    for (const auto & det : clusters.detections) {
      const auto & p = det.bbox.center.position;
      const auto & s = det.bbox.size;
      projected.push_back(projectBox(
        intrinsics_, cam_T_sensor, Eigen::Vector3d(p.x, p.y, p.z),
        Eigen::Vector3d(s.x, s.y, s.z), min_depth_));
    }

    // 2. Take the camera detections, if they are recent enough to describe the same moment.
    std::vector<ImageBox> det_boxes;
    std::vector<const vision_msgs::msg::Detection2D *> dets;
    if (latest_detections_) {
      const double age = std::abs(
        (rclcpp::Time(clusters.header.stamp) - rclcpp::Time(latest_detections_->header.stamp))
        .seconds());
      if (age <= max_detection_age_) {
        for (const auto & d : latest_detections_->detections) {
          det_boxes.push_back(toImageBox(d.bbox));
          dets.push_back(&d);
        }
      }
    }

    // 3. Associate and build the fused objects.
    const auto matches = greedyMatch(projected, det_boxes, min_iou_);
    std::vector<int> match_of_cluster(clusters.detections.size(), -1);
    std::vector<bool> detection_matched(det_boxes.size(), false);
    for (const auto & m : matches) {
      match_of_cluster[m.cluster] = static_cast<int>(m.detection);
      detection_matched[m.detection] = true;
    }

    std::vector<FusedObject> objects;
    for (std::size_t i = 0; i < clusters.detections.size(); ++i) {
      const auto & bbox = clusters.detections[i].bbox;
      FusedObject o;
      o.has_position = true;
      o.position = {bbox.center.position.x, bbox.center.position.y, bbox.center.position.z};
      o.size = {bbox.size.x, bbox.size.y, bbox.size.z};
      o.distance = o.position.head<2>().norm();
      o.bearing_deg = std::atan2(o.position.y(), o.position.x()) * kRadToDeg;
      o.image_box = projected[i];
      if (match_of_cluster[i] >= 0) {
        const auto & d = *dets[static_cast<std::size_t>(match_of_cluster[i])];
        const auto [cls, score] = classOf(d);
        o.class_id = cls;
        o.sensors = Sensors::kBoth;
        o.confidence = noisyOr(lidar_confidence_, score);
      } else {
        o.sensors = Sensors::kLidar;
        o.confidence = lidar_confidence_;
      }
      objects.push_back(o);
    }
    for (std::size_t j = 0; j < dets.size(); ++j) {
      if (detection_matched[j]) {
        continue;
      }
      // Seen by the camera only: we know the direction, but not the distance.
      const auto [cls, score] = classOf(*dets[j]);
      FusedObject o;
      o.class_id = cls;
      o.sensors = Sensors::kCamera;
      o.confidence = score;
      o.bearing_deg = bearingFromPixel(intrinsics_, det_boxes[j].centerX()) * kRadToDeg;
      o.image_box = det_boxes[j];
      objects.push_back(o);
    }
    for (auto & o : objects) {
      const bool watched =
        std::find(alert_classes_.begin(), alert_classes_.end(), o.class_id) != alert_classes_.end();
      // Alert on a watched class inside the radius, or a watched class we can't range yet.
      o.alert = watched && (!o.has_position || o.distance <= alert_radius_);
    }

    publishObjects(clusters.header, objects);
    publishAlerts(clusters.header, objects);
    publishMarkers(clusters.header, objects);
    publishDebugImage(projected, det_boxes, dets, objects);

    std::size_t n_both = 0, n_lidar = 0, n_camera = 0;
    for (const auto & o : objects) {
      n_both += o.sensors == Sensors::kBoth;
      n_lidar += o.sensors == Sensors::kLidar;
      n_camera += o.sensors == Sensors::kCamera;
    }
    RCLCPP_INFO_THROTTLE(get_logger(), steady_clock_, 5000,
      "%zu clusters + %zu detections -> %zu both, %zu lidar-only, %zu camera-only",
      clusters.detections.size(), dets.size(), n_both, n_lidar, n_camera);
  }

  static std::pair<std::string, double> classOf(const vision_msgs::msg::Detection2D & d)
  {
    if (d.results.empty()) {
      return {"object", 0.5};
    }
    return {d.results[0].hypothesis.class_id, d.results[0].hypothesis.score};
  }

  void publishObjects(const std_msgs::msg::Header & header, const std::vector<FusedObject> & objects)
  {
    vision_msgs::msg::Detection3DArray out;
    out.header = header;
    for (std::size_t i = 0; i < objects.size(); ++i) {
      const auto & o = objects[i];
      if (!o.has_position) {
        continue;
      }
      vision_msgs::msg::Detection3D d;
      d.header = header;
      d.id = std::to_string(i);
      d.bbox.center.position.x = o.position.x();
      d.bbox.center.position.y = o.position.y();
      d.bbox.center.position.z = o.position.z();
      d.bbox.center.orientation.w = 1.0;
      d.bbox.size.x = o.size.x();
      d.bbox.size.y = o.size.y();
      d.bbox.size.z = o.size.z();
      vision_msgs::msg::ObjectHypothesisWithPose h;
      h.hypothesis.class_id = o.class_id;
      h.hypothesis.score = o.confidence;
      d.results.push_back(h);
      out.detections.push_back(d);
    }
    objects_pub_->publish(out);
  }

  void publishAlerts(const std_msgs::msg::Header & header, const std::vector<FusedObject> & objects)
  {
    // Hand-written JSON keeps the dependency list short; all values are numbers or plain ids.
    std::ostringstream js;
    js << std::fixed << std::setprecision(2);
    js << "{\"stamp\":" << rclcpp::Time(header.stamp).seconds()
       << ",\"frame\":\"" << header.frame_id << "\",\"objects\":[";
    for (std::size_t i = 0; i < objects.size(); ++i) {
      const auto & o = objects[i];
      js << (i ? "," : "") << "{\"id\":" << i
         << ",\"class\":\"" << o.class_id << "\""
         << ",\"sensors\":\"" << toString(o.sensors) << "\""
         << ",\"confidence\":" << o.confidence
         << ",\"bearing_deg\":" << o.bearing_deg;
      if (o.has_position) {
        js << ",\"distance_m\":" << o.distance
           << ",\"position\":{\"x\":" << o.position.x() << ",\"y\":" << o.position.y()
           << ",\"z\":" << o.position.z() << "}";
      } else {
        js << ",\"distance_m\":null,\"position\":null";
      }
      js << ",\"level\":\"" << (o.alert ? "alert" : "info") << "\"}";
    }
    js << "]}";

    std_msgs::msg::String msg;
    msg.data = js.str();
    alerts_pub_->publish(msg);
  }

  void publishMarkers(const std_msgs::msg::Header & header, const std::vector<FusedObject> & objects)
  {
    using visualization_msgs::msg::Marker;
    visualization_msgs::msg::MarkerArray array;
    Marker clear;
    clear.header = header;
    clear.action = Marker::DELETEALL;
    array.markers.push_back(clear);

    int id = 0;
    for (const auto & o : objects) {
      if (!o.has_position) {
        continue;
      }
      // green = both sensors agree, orange = LiDAR only, red = alert
      float r = 1.0f, g = 0.6f, b = 0.1f;
      if (o.sensors == Sensors::kBoth) {
        r = 0.2f; g = 0.9f; b = 0.3f;
      }
      if (o.alert) {
        r = 1.0f; g = 0.15f; b = 0.15f;
      }

      Marker box;
      box.header = header;
      box.ns = "fused_boxes";
      box.id = id;
      box.type = Marker::CUBE;
      box.action = Marker::ADD;
      box.pose.position.x = o.position.x();
      box.pose.position.y = o.position.y();
      box.pose.position.z = o.position.z();
      box.pose.orientation.w = 1.0;
      box.scale.x = std::max(o.size.x(), 0.05);
      box.scale.y = std::max(o.size.y(), 0.05);
      box.scale.z = std::max(o.size.z(), 0.05);
      box.color.r = r;
      box.color.g = g;
      box.color.b = b;
      box.color.a = 0.4f;
      array.markers.push_back(box);

      Marker label;
      label.header = header;
      label.ns = "fused_labels";
      label.id = id;
      label.type = Marker::TEXT_VIEW_FACING;
      label.action = Marker::ADD;
      label.pose.position.x = o.position.x();
      label.pose.position.y = o.position.y();
      label.pose.position.z = o.position.z() + 0.5 * o.size.z() + 0.3;
      label.pose.orientation.w = 1.0;
      label.scale.z = 0.28;
      label.color.r = label.color.g = label.color.b = label.color.a = 1.0f;
      char text[96];
      std::snprintf(text, sizeof(text), "%s, %.1f m [%s]",
        o.class_id.c_str(), o.distance, shortTag(o.sensors));
      label.text = text;
      array.markers.push_back(label);
      ++id;
    }
    markers_pub_->publish(array);
  }

  void publishDebugImage(
    const std::vector<std::optional<ImageBox>> & projected,
    const std::vector<ImageBox> & det_boxes,
    const std::vector<const vision_msgs::msg::Detection2D *> & dets,
    const std::vector<FusedObject> & objects)
  {
    if (debug_pub_->get_subscription_count() == 0 || !latest_image_) {
      return;
    }
    cv_bridge::CvImagePtr cv;
    try {
      cv = cv_bridge::toCvCopy(latest_image_, "bgr8");
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), steady_clock_, 5000, "cv_bridge: %s", e.what());
      return;
    }
    auto rect = [](const ImageBox & b) {
        return cv::Rect(cv::Point(static_cast<int>(b.x_min), static_cast<int>(b.y_min)),
                 cv::Point(static_cast<int>(b.x_max), static_cast<int>(b.y_max)));
      };

    // Camera detections: thin white boxes.
    for (std::size_t j = 0; j < det_boxes.size(); ++j) {
      cv::rectangle(cv->image, rect(det_boxes[j]), cv::Scalar(255, 255, 255), 1);
      cv::putText(cv->image, classOf(*dets[j]).first,
        cv::Point(static_cast<int>(det_boxes[j].x_min), static_cast<int>(det_boxes[j].y_max) + 14),
        cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 255), 1);
    }
    // Projected LiDAR clusters: green if matched, orange if LiDAR-only (BGR colours).
    for (std::size_t i = 0; i < projected.size() && i < objects.size(); ++i) {
      if (!projected[i]) {
        continue;
      }
      const auto & o = objects[i];
      const cv::Scalar colour = o.sensors == Sensors::kBoth ?
        cv::Scalar(80, 230, 50) : cv::Scalar(30, 150, 255);
      cv::rectangle(cv->image, rect(*projected[i]), colour, 2);
      char text[96];
      std::snprintf(text, sizeof(text), "%s %.1fm", o.class_id.c_str(), o.distance);
      cv::putText(cv->image, text,
        cv::Point(static_cast<int>(projected[i]->x_min), std::max(12, static_cast<int>(projected[i]->y_min) - 4)),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, colour, 2);
    }
    debug_pub_->publish(*cv->toImageMsg());
  }

  // Parameters
  std::string camera_frame_;
  double min_iou_{0.15};
  double max_detection_age_{1.0};
  double lidar_confidence_{0.6};
  double min_depth_{0.3};
  std::vector<std::string> alert_classes_;
  double alert_radius_{5.0};

  // State
  Intrinsics intrinsics_;
  vision_msgs::msg::Detection2DArray::ConstSharedPtr latest_detections_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_image_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<vision_msgs::msg::Detection3DArray>::SharedPtr clusters_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr objects_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr alerts_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
};

}  // namespace go2_fusion

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<go2_fusion::FusionNode>());
  rclcpp::shutdown();
  return 0;
}
