FROM osrf/ros:jazzy-desktop

ARG DEBIAN_FRONTEND=noninteractive

# Gazebo Harmonic + bridge, Go2/CHAMP sim deps, perception deps
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-gz \
      ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-gz-ros2-control \
      ros-jazzy-xacro ros-jazzy-robot-localization \
      ros-jazzy-velodyne ros-jazzy-velodyne-description \
      ros-jazzy-teleop-twist-keyboard \
      ros-jazzy-pcl-ros ros-jazzy-pcl-conversions libpcl-dev \
      ros-jazzy-cv-bridge ros-jazzy-image-geometry ros-jazzy-vision-msgs \
      ros-jazzy-tf2-eigen ros-jazzy-tf2-geometry-msgs \
      python3-pip python3-colcon-common-extensions python3-vcstool git \
    && rm -rf /var/lib/apt/lists/*

RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc && \
    echo '[ -f /root/go2_ws/install/setup.bash ] && source /root/go2_ws/install/setup.bash' >> /root/.bashrc

WORKDIR /root/go2_ws
