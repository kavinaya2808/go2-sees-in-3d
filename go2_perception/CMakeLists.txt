cmake_minimum_required(VERSION 3.16)
project(go2_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters segmentation search kdtree sample_consensus)

# ROS-free core, shared by the node and the unit tests
add_library(cloud_pipeline STATIC src/cloud_pipeline.cpp)
target_include_directories(cloud_pipeline PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(cloud_pipeline SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(cloud_pipeline PUBLIC ${PCL_LIBRARIES})

add_executable(cluster_node src/cluster_node.cpp)
target_link_libraries(cluster_node cloud_pipeline)
ament_target_dependencies(cluster_node
  rclcpp sensor_msgs std_msgs vision_msgs visualization_msgs pcl_conversions)

install(TARGETS cluster_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_cloud_pipeline test/test_cloud_pipeline.cpp)
  target_link_libraries(test_cloud_pipeline cloud_pipeline)
endif()

ament_package()
