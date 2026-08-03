#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

namespace {
constexpr int kScanBins = 360;
constexpr double kScanAngleInc = 2.0f * M_PI / kScanBins;
constexpr double kRangeMin = 0.05f;  // 5 cm
constexpr double kRangeMax = 12.0f;  // 12 m (RPLiDAR C1 max range)
}  // namespace

namespace mote_base {

struct RawScanPoint {
  double angle_rad;
  double distance_mm;
};

// Accumulates raw (angle, distance) lidar samples and rasterizes them onto a
// fixed angular grid once a full rotation completes.
class ScanRasterizer {
  std::vector<RawScanPoint> accum_;
  double prev_angle_ = -1.0f;  // -1 = no previous point

  [[nodiscard]] std::vector<double> rasterize() const {
    std::vector<double> ranges(kScanBins, std::numeric_limits<double>::infinity());
    for (const auto& point : accum_) {
      int bin{static_cast<int>(point.angle_rad / kScanAngleInc)};
      bin = std::max(0, std::min(bin, kScanBins - 1));
      const double dist_m = point.distance_mm / 1000.0f;
      if (dist_m < ranges[bin]) {
        ranges[kScanBins - 1 - bin] = dist_m;  // reversed: ROS expects ccw scans
      }
    }
    return ranges;
  }

 public:
  std::optional<std::vector<double>> add_point(const RawScanPoint& point) {
    std::optional<std::vector<double>> completed;

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
