FROM ros:jazzy

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    python3-venv \
    libgl1 \
    libglib2.0-0 \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    v4l-utils \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

# Create a virtual environment
RUN python3 -m venv /venv

# Upgrade pip inside the venv and install opencv-contrib-python
RUN /venv/bin/pip install --upgrade pip && \
    /venv/bin/pip install --no-cache-dir opencv-contrib-python

# Set environment variables so Python and pip use the venv by default
ENV PATH="/venv/bin:$PATH"
ENV VIRTUAL_ENV="/venv"

# Create workspace
RUN mkdir -p /workspace/ros2_ws/src
WORKDIR /workspace/ros2_ws

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
