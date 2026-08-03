cmake_minimum_required(VERSION 3.14)
project(mote_base)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  hardware_interface
  controller_manager
)

find_package(Boost REQUIRED COMPONENTS system)

include(FetchContent)
FetchContent_Declare(nlohmann_json
  URL "https://github.com/nlohmann/json/archive/refs/tags/v3.11.3.tar.gz"
)
set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(nlohmann_json)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs
)

set(MOTE_FFI_VERSION "0.2.1")

if(NOT DEFINED MOTE_FFI_DIR)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(_MOTE_FFI_ARCH "aarch64")
  else()
    set(_MOTE_FFI_ARCH "x86_64")
  endif()

  FetchContent_Declare(mote_ffi
    URL "https://github.com/empriselab/mote/releases/download/mote-cxx-v${MOTE_FFI_VERSION}/mote-ffi-cxx-linux-${_MOTE_FFI_ARCH}-v${MOTE_FFI_VERSION}.tar.gz"
  )
  FetchContent_MakeAvailable(mote_ffi)
  set(MOTE_FFI_DIR "${mote_ffi_SOURCE_DIR}")
endif()

add_library(mote_ffi_static STATIC IMPORTED)
set_target_properties(mote_ffi_static PROPERTIES
  IMPORTED_LOCATION "${MOTE_FFI_DIR}/libmote_ffi.a"
)

add_executable(mote_node src/mote_node.cpp)

#catkin_lint: ignore_once missing_directory external_directory
target_include_directories(mote_node PRIVATE
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  "${MOTE_FFI_DIR}/include"
)

target_link_libraries(mote_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  mote_ffi_static
  nlohmann_json::nlohmann_json
  pthread
  dl
  rt
  m
)

target_compile_features(mote_node PRIVATE cxx_std_17)

install(TARGETS mote_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_scan_rasterizer test/test_scan_rasterizer.cpp)
  target_include_directories(test_scan_rasterizer PRIVATE include)

  catkin_add_gtest(test_messages test/test_messages.cpp)
  target_include_directories(test_messages PRIVATE include)
  target_link_libraries(test_messages nlohmann_json::nlohmann_json)
  target_compile_features(test_messages PRIVATE cxx_std_17)
  target_compile_features(test_scan_rasterizer PRIVATE cxx_std_17)
endif()
