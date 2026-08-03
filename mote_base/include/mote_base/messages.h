#pragma once

#include <nlohmann/json.hpp>
#include <vector>

#include "mote_base/scan_rasterizer.h"

namespace mote_base {

struct DriveBaseState {
  double left_position_rad = 0.0;
  double left_velocity_rad_per_s = 0.0;
  double left_effort_percent = 0.0;
  double right_position_rad = 0.0;
  double right_velocity_rad_per_s = 0.0;
  double right_effort_percent = 0.0;
};

struct ImuMeasurement {
  double accel_x = 0.0;
  double accel_y = 0.0;
  double accel_z = 0.0;
  double gyro_x = 0.0;
  double gyro_y = 0.0;
  double gyro_z = 0.0;
};

inline DriveBaseState parse_drive_base_state(const nlohmann::json& state) {
  DriveBaseState out;
  out.left_position_rad = state["left"]["position_rad"].get<double>();
  out.left_velocity_rad_per_s = state["left"]["velocity_rad_per_s"].get<double>();
  out.left_effort_percent = state["left"]["effort_percent"].get<double>();
  out.right_position_rad = state["right"]["position_rad"].get<double>();
  out.right_velocity_rad_per_s = state["right"]["velocity_rad_per_s"].get<double>();
  out.right_effort_percent = state["right"]["effort_percent"].get<double>();
  return out;
}

inline ImuMeasurement parse_imu_measurement(const nlohmann::json& imu) {
  ImuMeasurement out;
  out.accel_x = imu["accel"]["x"].get<double>();
  out.accel_y = imu["accel"]["y"].get<double>();
  out.accel_z = imu["accel"]["z"].get<double>();
  out.gyro_x = imu["gyro"]["x"].get<double>();
  out.gyro_y = imu["gyro"]["y"].get<double>();
  out.gyro_z = imu["gyro"]["z"].get<double>();
  return out;
}

// Parses a Scan message's point array, filtering out zero-quality (invalid)
// readings, ready to feed into ScanRasterizer::add_point.
inline std::vector<RawScanPoint> parse_scan_points(const nlohmann::json& points) {
  std::vector<RawScanPoint> out;
  out.reserve(points.size());
  for (const auto& pt : points) {
    if (pt["distance_mm"].get<double>() < 50.0) {
      continue;
    }
    out.push_back({pt["angle_rad"].get<double>(), pt["distance_mm"].get<double>(),
                    pt["quality"].get<double>()});
  }
  return out;
}

}  // namespace mote_base
