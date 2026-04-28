# Build context must be the mote-ros-noetic/ directory.
# Build command: docker build -t mote-ros-noetic .

# ── Build the ROS Noetic node ────────────────────────────────────────────────
FROM ros:noetic-ros-base AS ros-builder

# Install bare build tools (rosdep handles all ROS/package deps below).
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    && rm -rf /var/lib/apt/lists/*

# Copy package.xml first so rosdep can install ROS deps as a cached layer —
# this layer only re-runs when package.xml changes, not on every source edit.
WORKDIR /catkin_ws/src/mote_ros_noetic
COPY package.xml .
RUN apt-get update && \
    rosdep update --rosdistro noetic  --include-eol-distros && \
    rosdep install --from-paths /catkin_ws/src --ignore-src -r -y --include-eol-distros

# Copy full source and build the catkin workspace.
# CMakeLists.txt downloads libmote_ffi from GitHub releases at configure time.
WORKDIR /catkin_ws/src
COPY . mote_ros_noetic/

WORKDIR /catkin_ws
RUN /bin/bash -c "\
    source /opt/ros/noetic/setup.bash && \
    catkin_make"

COPY docker-entrypoint.sh /docker-entrypoint.sh
RUN apt-get update && apt-get install -y --no-install-recommends dos2unix \
    && dos2unix /docker-entrypoint.sh \
    && rm -rf /var/lib/apt/lists/*
RUN chmod +x /docker-entrypoint.sh

ENTRYPOINT ["/docker-entrypoint.sh"]
