#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "go2_perception/cloud_pipeline.hpp"

using go2_perception::Cloud;
using go2_perception::PipelineConfig;
using go2_perception::Point;

namespace
{

constexpr float kGroundZ = -0.4f;  // sensor sits ~0.4 m above the floor

void addGround(Cloud & cloud, float half_size, float step)
{
  for (float x = -half_size; x <= half_size; x += step) {
    for (float y = -half_size; y <= half_size; y += step) {
      cloud.push_back(Point(x, y, kGroundZ));
    }
  }
}

// Solid block of points centred at (cx, cy, cz).
void addBlock(Cloud & cloud, float cx, float cy, float cz, float sx, float sy, float sz)
{
  const float step = 0.05f;
  for (float x = -sx / 2; x <= sx / 2 + 1e-4f; x += step) {
    for (float y = -sy / 2; y <= sy / 2 + 1e-4f; y += step) {
      for (float z = -sz / 2; z <= sz / 2 + 1e-4f; z += step) {
        cloud.push_back(Point(cx + x, cy + y, cz + z));
      }
    }
  }
}

Cloud::Ptr sceneWithTwoObjects()
{
  Cloud::Ptr cloud(new Cloud);
  addGround(*cloud, 5.0f, 0.1f);
  addBlock(*cloud, 3.0f, 0.0f, -0.15f, 0.5f, 0.5f, 0.5f);   // box in front
  addBlock(*cloud, -2.0f, 2.5f, 0.1f, 0.4f, 0.4f, 1.0f);    // "person-sized" column
  return cloud;
}

}  // namespace

TEST(CloudPipeline, FindsTwoObjectsAtTheRightPlaces)
{
  const auto result = go2_perception::runPipeline(sceneWithTwoObjects(), PipelineConfig{});

  ASSERT_EQ(result.clusters.size(), 2u);
  // Sorted nearest first: the box at 3.0 m comes before the column at ~3.2 m.
  EXPECT_NEAR(result.clusters[0].centroid.x(), 3.0f, 0.1f);
  EXPECT_NEAR(result.clusters[0].centroid.y(), 0.0f, 0.1f);
  EXPECT_NEAR(result.clusters[0].range(), 3.0f, 0.1f);
  EXPECT_NEAR(result.clusters[1].centroid.x(), -2.0f, 0.1f);
  EXPECT_NEAR(result.clusters[1].centroid.y(), 2.5f, 0.1f);
  EXPECT_NEAR(result.clusters[1].size().z(), 1.0f, 0.15f);
}

TEST(CloudPipeline, RemovesTheGroundPlane)
{
  const auto result = go2_perception::runPipeline(sceneWithTwoObjects(), PipelineConfig{});

  EXPECT_GT(result.ground_points, 1000u);
  for (const auto & p : result.obstacles->points) {
    EXPECT_GT(p.z, kGroundZ + 0.05f) << "ground point survived at z=" << p.z;
  }
}

TEST(CloudPipeline, RemovesTheRobotsOwnBody)
{
  Cloud::Ptr cloud(new Cloud);
  addGround(*cloud, 5.0f, 0.1f);
  addBlock(*cloud, 0.0f, 0.0f, -0.2f, 0.6f, 0.3f, 0.3f);  // the dog's body/legs around the sensor

  const auto result = go2_perception::runPipeline(cloud, PipelineConfig{});
  EXPECT_TRUE(result.clusters.empty());
}

TEST(CloudPipeline, IgnoresNaNsAndHandlesEmptyClouds)
{
  Cloud::Ptr empty(new Cloud);
  EXPECT_TRUE(go2_perception::runPipeline(empty, PipelineConfig{}).clusters.empty());

  Cloud::Ptr cloud = sceneWithTwoObjects();
  const float nan = std::numeric_limits<float>::quiet_NaN();
  for (int i = 0; i < 100; ++i) {
    cloud->push_back(Point(nan, nan, nan));
  }
  EXPECT_EQ(go2_perception::runPipeline(cloud, PipelineConfig{}).clusters.size(), 2u);
}

TEST(CloudPipeline, RejectsClustersThatAreTooSmall)
{
  Cloud::Ptr cloud(new Cloud);
  addGround(*cloud, 5.0f, 0.1f);
  cloud->push_back(Point(2.0f, 2.0f, 0.2f));  // a few stray points, not an object
  cloud->push_back(Point(2.02f, 2.0f, 0.2f));

  const auto result = go2_perception::runPipeline(cloud, PipelineConfig{});
  EXPECT_TRUE(result.clusters.empty());
}
