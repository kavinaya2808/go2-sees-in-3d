cmake_minimum_required(VERSION 3.16)
project(go2_fusion)

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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(Eigen3 REQUIRED)

# ROS-free projection and association maths, shared by the node and the unit tests
add_library(projection STATIC src/projection.cpp)
target_include_directories(projection PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(projection PUBLIC Eigen3::Eigen)

add_executable(fusion_node src/fusion_node.cpp)
target_link_libraries(fusion_node projection ${OpenCV_LIBS})
ament_target_dependencies(fusion_node
  rclcpp sensor_msgs std_msgs vision_msgs visualization_msgs tf2 tf2_ros tf2_eigen cv_bridge)

install(TARGETS fusion_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_projection test/test_projection.cpp)
  target_link_libraries(test_projection projection)
endif()

ament_package()
