# Start from ROS 2 Jazzy base image
FROM ros:jazzy

# Install tools and dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    python3-opencv \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    v4l-utils \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

# Create and set workspace
RUN mkdir -p /workspace/ros2_ws/src
WORKDIR /workspace/ros2_ws

# Source ROS 2 on shell start
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
