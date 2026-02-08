# Base image with ROS 2 Humble + desktop tools (LTS version)
FROM osrf/ros:humble-desktop-full

# Prevent installation prompts
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DOMAIN_ID=0
ENV ROS_DISTRO=humble

# Install ROS 2 packages and tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-vision-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    git \
    vim \
    nano \
    tmux \
    wget \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Remove conflicting system package
RUN apt-get purge -y python3-sympy || true

# Upgrade pip first
RUN pip3 install --upgrade pip

# Install PyTorch separately (large download)
RUN pip3 install --no-cache-dir --break-system-packages \
    torch \
    torchvision

# Install other Python packages
RUN pip3 install --no-cache-dir --break-system-packages \
    ultralytics \
    opencv-python-headless \
    numpy \
    matplotlib

# Set workspace directory
WORKDIR /ros2_ws

# Initialize rosdep
RUN rosdep update

# Auto-source ROS on terminal startup
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> ~/.bashrc && \
    echo "echo 'ROS 2 Humble environment loaded'" >> ~/.bashrc

CMD ["bash"]
