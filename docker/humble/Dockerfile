# Prediction Humble dev image layered on the upstream integration base.
# Base matches System Integrator image family:
#   stereolabs/zed:5.4-devel-cuda12.8-ubuntu22.04

FROM stereolabs/zed:5.4-devel-cuda12.8-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=humble

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    gnupg \
    lsb-release \
    software-properties-common \
    git \
    build-essential \
    python3-pip \
    python3-venv \
    python3-pytest \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 apt repository (required before colcon / ros-humble packages).
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
      > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-ros-base \
    ros-humble-ament-cmake \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init 2>/dev/null || true \
    && rosdep update

RUN pip3 install --no-cache-dir \
    "numpy>=1.24" \
    "shapely>=2.0" \
    "PyYAML>=6.0" \
    "matplotlib>=3.7" \
    "pytest>=7.4"

COPY docker/humble/ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

WORKDIR /workspace/prediction-src

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
