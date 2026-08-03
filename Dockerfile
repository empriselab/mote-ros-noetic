# docker build -t mote-ros-noetic .
FROM ros:noetic-ros-base

# Install bare build tools (rosdep handles all ROS/package deps below).
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    && rm -rf /var/lib/apt/lists/*

# Copy every package's package.xml first so rosdep can install ROS deps as a cached layer
WORKDIR /catkin_ws/src/mote_ros_noetic
COPY mote_base/package.xml mote_base/package.xml
COPY mote_control/package.xml mote_control/package.xml
COPY mote_description/package.xml mote_description/package.xml
COPY mote_navigation/package.xml mote_navigation/package.xml
COPY mote_viz/package.xml mote_viz/package.xml
COPY mote_demos/package.xml mote_demos/package.xml

WORKDIR /catkin_ws
RUN apt-get update && \
    rosdep update --rosdistro noetic --include-eol-distros && \
    rosdep install --from-paths src --ignore-src -r -y --include-eol-distros

# Copy full source and build the catkin workspace.
COPY . src/mote_ros_noetic/

RUN /bin/bash -c "\
    source /opt/ros/noetic/setup.bash && \
    catkin_make"

COPY docker-entrypoint.sh /docker-entrypoint.sh
RUN apt-get update && apt-get install -y --no-install-recommends dos2unix \
    && dos2unix /docker-entrypoint.sh \
    && rm -rf /var/lib/apt/lists/*
RUN chmod +x /docker-entrypoint.sh

ENTRYPOINT ["/docker-entrypoint.sh"]
