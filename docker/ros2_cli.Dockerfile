FROM ros:humble-ros-core

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Base tooling + ROS deps you had in compose
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-pip \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-cv-bridge \
    python3-opencv \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
 && rm -rf /var/lib/apt/lists/*

# Python deps
RUN python3 -m pip install --no-cache-dir \
    pyyaml \
    Flask \
    waitress \
    influxdb_client

# rosdep basic setup (you can comment this out if you prefer to do it manually)
RUN rosdep init || true && \
    rosdep update || true

# Workspace location (will still be mounted from host at runtime)
WORKDIR /root/colcon_ws

# Auto-source in interactive shells
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "[ -f /root/colcon_ws/install/setup.bash ] && source /root/colcon_ws/install/setup.bash" >> /root/.bashrc

# Default command – we will override in compose to run your launch,
# but this makes 'docker run -it image' give you a ROS shell
CMD ["bash"]
