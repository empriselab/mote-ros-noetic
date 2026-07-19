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
  float angle_rad;
  float distance_mm;
};

// Accumulates raw (angle, distance) lidar samples and rasterizes them onto a
// fixed angular grid once a full rotation completes.
//
// A rotation boundary is detected when the incoming angle wraps back near
// zero -- i.e. drops by more than pi from the previous point's angle.
class ScanRasterizer {
 public:
  static constexpr int kScanBins = 360;
  static constexpr float kScanAngleInc = 2.0f * static_cast<float>(M_PI) / kScanBins;
  static constexpr float kRangeMin = 0.05f;  // 5 cm
  static constexpr float kRangeMax = 12.0f;  // 12 m (RPLiDAR C1 max range)

  // Adds a point to the in-progress rotation. If this point crosses a
  // rotation boundary, the just-completed rotation is rasterized onto a
  // kScanBins-sized range grid and returned; the point itself starts the
  // next rotation. Otherwise returns std::nullopt.
  std::optional<std::vector<float>> add_point(const RawScanPoint &pt) {
    std::optional<std::vector<float>> completed;

    constexpr float two_pi = 2.0f * static_cast<float>(M_PI);
    float angle = std::fmod(pt.angle_rad, two_pi);
    if (angle < 0.0f) angle += two_pi;

    if (prev_angle_ >= 0.0f && angle < prev_angle_ - static_cast<float>(M_PI)) {
      completed = rasterize();
      accum_.clear();
    }

    accum_.push_back({angle, pt.distance_mm});
    prev_angle_ = angle;

    return completed;
  }

  // Number of points accumulated for the in-progress rotation.
  std::size_t size() const { return accum_.size(); }

 private:
  std::vector<RawScanPoint> accum_;
  float prev_angle_ = -1.0f;  // -1 = no previous point

  std::vector<float> rasterize() const {
    std::vector<float> ranges(kScanBins, std::numeric_limits<float>::infinity());
    for (const auto &pt : accum_) {
      int bin = static_cast<int>(pt.angle_rad / kScanAngleInc);
      bin = std::max(0, std::min(bin, kScanBins - 1));
      const float dist_m = pt.distance_mm / 1000.0f;
      if (dist_m < ranges[bin])
        ranges[kScanBins - 1 - bin] = dist_m;  // reversed: ROS expects ccw scans
    }
    return ranges;
  }
};

}  // namespace mote_base
