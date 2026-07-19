# Build context must be the mote-ros-noetic/ directory.
# Build command: docker build -t mote-ros-noetic .
#
# This image is meant to be used as a base by downstream repos (e.g. students
# writing their own nodes against the robot), so it keeps the full build
# toolchain (build-essential, cmake, rosdep build deps) rather than stripping
# it down to a slim runtime-only image — anyone building on top needs to be
# able to add a package and run catkin_make again.
FROM ros:noetic-ros-base

# Install bare build tools (rosdep handles all ROS/package deps below).
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    && rm -rf /var/lib/apt/lists/*

# Copy every package's package.xml first so rosdep can install ROS deps as a
# cached layer — this layer only re-runs when a package.xml changes, not on
# every source edit. (Copying just mote_base/package.xml here would leave
# rosdep blind to the other 5 packages' dependencies, since --from-paths only
# sees what's been copied in by this point.)
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
# CMakeLists.txt downloads libmote_ffi from GitHub releases at configure time.
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
