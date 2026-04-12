# ──────────────────────────────────────────────────────────────
# Amiga FarNav – ROS 2 Humble on Ubuntu 22.04 LTS
# ──────────────────────────────────────────────────────────────
FROM ros:humble-ros-base-jammy

ARG DEBIAN_FRONTEND=noninteractive

# ── System dependencies ──────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-cv-bridge \
    ros-humble-rviz2 \
    libgl1-mesa-glx \
    libglib2.0-0 \
    git \
    curl \
    && rm -rf /var/lib/apt/lists/*

# ── Python dependencies ──────────────────────────────────────
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# ── Workspace setup ──────────────────────────────────────────
ENV AMIGA_WS=/ros2_ws
RUN mkdir -p ${AMIGA_WS}/src
WORKDIR ${AMIGA_WS}

# Copy source packages
COPY src/ ${AMIGA_WS}/src/

# ── Build the workspace ──────────────────────────────────────
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# ── Entrypoint ───────────────────────────────────────────────
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
