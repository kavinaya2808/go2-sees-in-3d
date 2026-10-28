FROM osrf/ros:jazzy-desktop

ARG DEBIAN_FRONTEND=noninteractive

# Gazebo Harmonic + bridge, Go2/CHAMP simulation, perception and fusion dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-gz \
      ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-gz-ros2-control \
      ros-jazzy-xacro ros-jazzy-robot-localization \
      ros-jazzy-velodyne ros-jazzy-velodyne-description \
      ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui \
      ros-jazzy-realsense2-description \
      ros-jazzy-teleop-twist-keyboard \
      ros-jazzy-pcl-ros ros-jazzy-pcl-conversions libpcl-dev \
      ros-jazzy-cv-bridge ros-jazzy-image-geometry ros-jazzy-vision-msgs \
      ros-jazzy-tf2-eigen ros-jazzy-tf2-geometry-msgs \
      python3-pip python3-colcon-common-extensions python3-vcstool git \
    && rm -rf /var/lib/apt/lists/*

# YOLO on CPU. numpy<2 keeps ROS Jazzy's Python bindings (built against NumPy 1.x) working.
RUN pip install --break-system-packages --no-cache-dir torch torchvision \
      --index-url https://download.pytorch.org/whl/cpu && \
    pip install --break-system-packages --no-cache-dir "numpy<2" "opencv-python<4.12" ultralytics

RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc && \
    echo '[ -f /root/go2_ws/install/setup.bash ] && source /root/go2_ws/install/setup.bash' >> /root/.bashrc

WORKDIR /root/go2_ws
