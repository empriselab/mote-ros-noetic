cmake_minimum_required(VERSION 3.14)
project(mote_ros_noetic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  hardware_interface
  controller_manager
)

include(FetchContent)

FetchContent_Declare(nlohmann_json
  URL "https://github.com/nlohmann/json/archive/refs/tags/v3.11.3.tar.gz"
)
set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(nlohmann_json)

catkin_package()

# --- libmote_ffi ---
# MOTE_FFI_DIR should point to the directory containing:
#   libmote_ffi.a
#   include/mote_link.h
# If not provided, the pre-built release artifact is downloaded from GitHub.
set(MOTE_FFI_VERSION "0.1.0")

if(NOT DEFINED MOTE_FFI_DIR)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(_MOTE_FFI_ARCH "aarch64")
  else()
    set(_MOTE_FFI_ARCH "x86_64")
  endif()

  FetchContent_Declare(mote_ffi
    URL "https://github.com/empriselab/mote-core/releases/download/mote-ffi-v${MOTE_FFI_VERSION}/mote-ffi-c-linux-${_MOTE_FFI_ARCH}-v${MOTE_FFI_VERSION}.tar.gz"
  )
  FetchContent_MakeAvailable(mote_ffi)
  set(MOTE_FFI_DIR "${mote_ffi_SOURCE_DIR}")
endif()

add_library(mote_ffi_static STATIC IMPORTED)
set_target_properties(mote_ffi_static PROPERTIES
  IMPORTED_LOCATION "${MOTE_FFI_DIR}/libmote_ffi.a"
)

# --- mote_node executable ---
add_executable(mote_node src/mote_node.cpp)

target_include_directories(mote_node PRIVATE
  ${catkin_INCLUDE_DIRS}
  "${MOTE_FFI_DIR}/include"
)

target_link_libraries(mote_node
  ${catkin_LIBRARIES}
  mote_ffi_static
  nlohmann_json::nlohmann_json
  pthread
  dl
  rt
  m
)

target_compile_features(mote_node PRIVATE cxx_std_14)

install(TARGETS mote_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
