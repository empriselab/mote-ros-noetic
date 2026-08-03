#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

namespace mote_base {

struct RawScanPoint {
  double angle_rad;
  double distance_mm;
};

// Accumulates raw (angle, distance) lidar samples and rasterizes them onto a
// fixed angular grid once a full rotation completes.
class ScanRasterizer {
 public:
  static constexpr int kScanBins = 360;
  static constexpr float kScanAngleInc = 2.0f * static_cast<float>(M_PI) / kScanBins;
  static constexpr float kRangeMin = 0.05f;  // 5 cm
  static constexpr float kRangeMax = 12.0f;  // 12 m (RPLiDAR C1 max range)

 private:
  std::vector<RawScanPoint> accum_;
  double prev_angle_ = -1.0f;  // -1 = no previous point

  [[nodiscard]] std::vector<float> rasterize() const {
    std::vector<float> ranges(kScanBins, std::numeric_limits<float>::infinity());
    for (const auto& point : accum_) {
      int bin{static_cast<int>(point.angle_rad / kScanAngleInc)};
      bin = std::max(0, std::min(bin, kScanBins - 1));
      const auto dist_m = static_cast<float>(point.distance_mm / 1000.0);
      if (dist_m < ranges[bin]) {
        ranges[kScanBins - 1 - bin] = dist_m;  // reversed: ROS expects ccw scans
      }
    }
    return ranges;
  }

 public:
  std::optional<std::vector<float>> add_point(const RawScanPoint& point) {
    std::optional<std::vector<float>> completed;

    constexpr double two_pi = 2.0f * M_PI;
    double angle = std::fmod(point.angle_rad, two_pi);
    if (angle < 0.0f) {
      angle += two_pi;
    }

    if (prev_angle_ >= 0.0f && angle < prev_angle_ - M_PI) {
      completed = rasterize();
      accum_.clear();
    }

    accum_.push_back({angle, point.distance_mm});
    prev_angle_ = angle;

    return completed;
  }

  // Number of points accumulated for the in-progress rotation.
  [[nodiscard]] std::size_t size() const { return accum_.size(); }
};

}  // namespace mote_base
