#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "mote_base/scan_rasterizer.h"

using mote_base::RawScanPoint;
using mote_base::ScanRasterizer;

TEST(ScanRasterizer, AccumulatesWithoutCompletingUntilWrap) {
  ScanRasterizer rasterizer;

  EXPECT_FALSE(rasterizer.add_point({0.0, 1000.0, 10.0}).has_value());
  EXPECT_EQ(rasterizer.size(), 1u);

  EXPECT_FALSE(rasterizer.add_point({6.0, 2000.0, 20.0}).has_value());
  EXPECT_EQ(rasterizer.size(), 2u);
}

TEST(ScanRasterizer, WrapCompletesRotationAndStartsNewOne) {
  ScanRasterizer rasterizer;
  rasterizer.add_point({0.0, 1000.0, 10.0});  // bin 0   -> stored at index 359
  rasterizer.add_point({6.0, 2000.0, 20.0});  // bin 343 -> stored at index 16

  // 0.05 rad is well below (6.0 - pi), so this triggers a wrap.
  auto completed = rasterizer.add_point({0.05, 500.0, 30.0});
  ASSERT_TRUE(completed.has_value());
  EXPECT_EQ(completed->ranges.size(), static_cast<std::size_t>(ScanRasterizer::kScanBins));
  EXPECT_EQ(completed->intensities.size(), static_cast<std::size_t>(ScanRasterizer::kScanBins));

  EXPECT_NEAR(completed->ranges[359], 1.0, 1e-4);
  EXPECT_NEAR(completed->intensities[359], 10.0, 1e-4);
  EXPECT_NEAR(completed->ranges[16], 2.0, 1e-4);
  EXPECT_NEAR(completed->intensities[16], 20.0, 1e-4);
  EXPECT_EQ(completed->ranges[100], std::numeric_limits<double>::infinity());
  EXPECT_EQ(completed->intensities[100], 0.0);

  // The wrap-triggering point starts the next rotation.
  EXPECT_EQ(rasterizer.size(), 1u);
}

TEST(ScanRasterizer, KeepsClosestPointPerBin) {
  ScanRasterizer rasterizer;
  rasterizer.add_point({0.0, 2000.0, 5.0});
  rasterizer.add_point({0.0, 500.0, 15.0});  // same bin, closer -- should win
  auto completed = rasterizer.add_point({6.0, 1000.0, 25.0});
  ASSERT_FALSE(completed.has_value());

  // Force a wrap to inspect the rasterized result.
  completed = rasterizer.add_point({0.0, 999.0, 35.0});
  ASSERT_TRUE(completed.has_value());
  EXPECT_NEAR(completed->ranges[359], 0.5, 1e-4);
  EXPECT_NEAR(completed->intensities[359], 15.0, 1e-4);
}

TEST(ScanRasterizer, ScanBinsConstantsAreConsistent) {
  EXPECT_EQ(ScanRasterizer::kScanBins, 360);
  EXPECT_NEAR(ScanRasterizer::kScanAngleInc * ScanRasterizer::kScanBins, 2.0 * M_PI, 1e-4);
  EXPECT_LT(ScanRasterizer::kRangeMin, ScanRasterizer::kRangeMax);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
