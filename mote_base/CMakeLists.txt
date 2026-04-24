cmake_minimum_required(VERSION 3.0.2)
project(mote_base)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  hardware_interface
  controller_manager
)

find_package(nlohmann_json 3.2.0 REQUIRED)

catkin_package()

# --- libmote_ffi ---
# MOTE_FFI_DIR should point to the directory containing:
#   libmote_ffi.a
#   include/mote_link.h
# Defaults to /opt/mote_ffi which is where the Dockerfile places it.
if(NOT DEFINED MOTE_FFI_DIR)
  set(MOTE_FFI_DIR "/opt/mote_ffi" CACHE PATH
    "Directory containing libmote_ffi.a and include/mote_link.h")
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
