#include <gtest/gtest.h>

#include "mote_base/messages.h"

using mote_base::parse_drive_base_state;
using mote_base::parse_imu_measurement;
using mote_base::parse_scan_points;
using nlohmann::json;

TEST(Messages, ParsesDriveBaseState) {
  const json state = {
      {"left", {{"position_rad", 1.0}, {"velocity_rad_per_s", 2.0}, {"effort_percent", 3.0}}},
      {"right", {{"position_rad", 4.0}, {"velocity_rad_per_s", 5.0}, {"effort_percent", 6.0}}},
  };

  const auto out = parse_drive_base_state(state);
  EXPECT_DOUBLE_EQ(out.left_position_rad, 1.0);
  EXPECT_DOUBLE_EQ(out.left_velocity_rad_per_s, 2.0);
  EXPECT_DOUBLE_EQ(out.left_effort_percent, 3.0);
  EXPECT_DOUBLE_EQ(out.right_position_rad, 4.0);
  EXPECT_DOUBLE_EQ(out.right_velocity_rad_per_s, 5.0);
  EXPECT_DOUBLE_EQ(out.right_effort_percent, 6.0);
}

TEST(Messages, ParsesImuMeasurement) {
  const json imu = {
      {"accel", {{"x", 1.0}, {"y", 2.0}, {"z", 3.0}}},
      {"gyro", {{"x", 4.0}, {"y", 5.0}, {"z", 6.0}}},
  };

  const auto out = parse_imu_measurement(imu);
  EXPECT_DOUBLE_EQ(out.accel_x, 1.0);
  EXPECT_DOUBLE_EQ(out.accel_y, 2.0);
  EXPECT_DOUBLE_EQ(out.accel_z, 3.0);
  EXPECT_DOUBLE_EQ(out.gyro_x, 4.0);
  EXPECT_DOUBLE_EQ(out.gyro_y, 5.0);
  EXPECT_DOUBLE_EQ(out.gyro_z, 6.0);
}

TEST(Messages, ScanPointsFiltersZeroQuality) {
  const json points = {
      {{"quality", 0}, {"angle_rad", 1.0}, {"distance_mm", 100.0}},
      {{"quality", 10}, {"angle_rad", 2.0}, {"distance_mm", 200.0}},
  };

  const auto out = parse_scan_points(points);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_DOUBLE_EQ(out[0].angle_rad, 2.0);
  EXPECT_DOUBLE_EQ(out[0].distance_mm, 200.0);
  EXPECT_DOUBLE_EQ(out[0].intensity, 10.0);
}

TEST(Messages, ScanPointsEmptyWhenAllFiltered) {
  const json points = {
      {{"quality", 0}, {"angle_rad", 1.0}, {"distance_mm", 100.0}},
  };

  const auto out = parse_scan_points(points);
  EXPECT_TRUE(out.empty());
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
