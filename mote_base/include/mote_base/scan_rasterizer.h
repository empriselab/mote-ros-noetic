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
  double intensity;
};

// Ranges and per-bin intensities produced by rasterizing one full rotation.
struct RasterizedScan {
  std::vector<double> ranges;
  std::vector<double> intensities;
};

// Accumulates raw (angle, distance, intensity) lidar samples and rasterizes
// them onto a fixed angular grid once a full rotation completes.
class ScanRasterizer {
 public:
  static constexpr int kScanBins = 720;
  static constexpr double kScanAngleInc = 2.0 * M_PI / kScanBins;
  static constexpr double kRangeMin = 0.05;  // 5 cm
  static constexpr double kRangeMax = 12.0;  // 12 m (RPLiDAR C1 max range)

 private:
  std::vector<RawScanPoint> accum_;
  double prev_angle_ = -1.0;  // -1 = no previous point

  [[nodiscard]] RasterizedScan rasterize() const {
    RasterizedScan out;
    out.ranges.assign(kScanBins, std::numeric_limits<double>::infinity());
    out.intensities.assign(kScanBins, 0.0);
    for (const auto& point : accum_) {
      int bin{static_cast<int>(point.angle_rad / kScanAngleInc)};
      bin = std::max(0, std::min(bin, kScanBins - 1));
      const double dist_m = point.distance_mm / 1000.0;
      const int idx = kScanBins - 1 - bin;  // reversed: ROS expects ccw scans
      if (dist_m < out.ranges[idx]) {
        out.ranges[idx] = dist_m;
        out.intensities[idx] = point.intensity;
      }
    }
    return out;
  }

 public:
  std::optional<RasterizedScan> add_point(const RawScanPoint& point) {
    std::optional<RasterizedScan> completed;

    constexpr double two_pi = 2.0 * M_PI;
    double angle = std::fmod(point.angle_rad, two_pi);
    if (angle < 0.0) {
      angle += two_pi;
    }

    if (prev_angle_ >= 0.0 && angle < prev_angle_ - M_PI) {
      completed = rasterize();
      accum_.clear();
    }

    accum_.push_back({angle, point.distance_mm, point.intensity});
    prev_angle_ = angle;

    return completed;
  }

  // Number of points accumulated for the in-progress rotation.
  [[nodiscard]] std::size_t size() const { return accum_.size(); }
};

}  // namespace mote_base
