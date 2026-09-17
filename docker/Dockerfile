# Base image: Official ROS 1 Noetic on Ubuntu 20.04 (Focal)
FROM osrf/ros:noetic-desktop-full

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Set default shell to bash
SHELL ["/bin/bash", "-c"]

# Install build tools, navigation packages, planners, and utilities
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    curl \
    wget \
    python3-pip \
    python-is-python3 \
    dos2unix \
    python3-catkin-tools \
    python3-rosdep \
    python3-osrf-pycommon \
    ros-noetic-navigation \
    ros-noetic-teb-local-planner \
    ros-noetic-dwa-local-planner \
    ros-noetic-gmapping \
    ros-noetic-map-server \
    ros-noetic-amcl \
    ros-noetic-move-base \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-rosserial \
    ros-noetic-rosserial-python \
    ros-noetic-rosserial-server \
    ros-noetic-rosserial-client \
    ros-noetic-teleop-twist-keyboard \
    nano \
    net-tools \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# Auto-source ROS in bashrc so every new terminal (docker exec) immediately has catkin_make
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# Set up environment entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN sed -i 's/\r$//' /entrypoint.sh && chmod +x /entrypoint.sh

# Default working directory
WORKDIR /catkin_ws

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
