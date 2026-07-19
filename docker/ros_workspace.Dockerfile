# Build context must be the repo root (parent of mote-ros-noetic/).
# Build command: docker build -f mote-ros-noetic/docker/ros_workspace.Dockerfile -t mote-ros-noetic .

# ── Stage 1: Build libmote_ffi.a ────────────────────────────────────────────
FROM rust:bookworm AS rust-builder

WORKDIR /build

# Copy only the crates needed for the C FFI build.
# mote-ffi depends on mote-api via path = "../mote-api", so both must be present
# at the same relative depth.
COPY mote-core/mote-api/ mote-api/
COPY mote-core/mote-ffi/ mote-ffi/

WORKDIR /build/mote-ffi

# Build the C static library. Disables the default wasm_ffi feature to avoid
# pulling in wasm-bindgen on a native target.
RUN cargo build --release --no-default-features --features c_ffi

# ── Stage 2: Build the ROS Noetic node ──────────────────────────────────────
FROM ros:noetic-ros-base AS ros-builder

# Install bare build tools (rosdep handles all ROS/package deps below).
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*

# Install the built C library and header into a well-known location.
RUN mkdir -p /opt/mote_ffi/include
COPY --from=rust-builder /build/mote-ffi/target/release/libmote_ffi.a /opt/mote_ffi/
COPY --from=rust-builder /build/mote-ffi/include/mote_link.h            /opt/mote_ffi/include/

# Copy package.xml first so rosdep can install ROS deps as a cached layer —
# this layer only re-runs when package.xml changes, not on every source edit.
WORKDIR /catkin_ws/src/mote_base
COPY mote-ros-noetic/mote_base/package.xml .
RUN apt-get update && \
    rosdep update --rosdistro noetic  --include-eol-distros && \
    rosdep install --from-paths /catkin_ws/src --ignore-src -r -y --include-eol-distros

# Copy full source and build the catkin workspace.
WORKDIR /catkin_ws/src
COPY mote-ros-noetic/mote_base/ mote_base/

WORKDIR /catkin_ws
RUN /bin/bash -c "\
    source /opt/ros/noetic/setup.bash && \
    catkin_make -DMOTE_FFI_DIR=/opt/mote_ffi"

COPY mote-ros-noetic/docker/docker-entrypoint.sh /docker-entrypoint.sh
# Strip Windows CRLF line endings and make executable
RUN sed -i 's/\r$//' /docker-entrypoint.sh && chmod +x /docker-entrypoint.sh

ENTRYPOINT ["/docker-entrypoint.sh"]
