#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "mote_base/scan_rasterizer.h"

using mote_base::RawScanPoint;
using mote_base::ScanRasterizer;

TEST(ScanRasterizer, AccumulatesWithoutCompletingUntilWrap) {
  ScanRasterizer rasterizer;

  EXPECT_FALSE(rasterizer.add_point({0.0f, 1000.0f}).has_value());
  EXPECT_EQ(rasterizer.size(), 1u);

  EXPECT_FALSE(rasterizer.add_point({6.0f, 2000.0f}).has_value());
  EXPECT_EQ(rasterizer.size(), 2u);
}

TEST(ScanRasterizer, WrapCompletesRotationAndStartsNewOne) {
  ScanRasterizer rasterizer;
  rasterizer.add_point({0.0f, 1000.0f});  // bin 0   -> stored at index 359
  rasterizer.add_point({6.0f, 2000.0f});  // bin 343 -> stored at index 16

  // 0.05 rad is well below (6.0 - pi), so this triggers a wrap.
  auto completed = rasterizer.add_point({0.05f, 500.0f});
  ASSERT_TRUE(completed.has_value());
  EXPECT_EQ(completed->size(), static_cast<std::size_t>(ScanRasterizer::kScanBins));

  EXPECT_NEAR((*completed)[359], 1.0f, 1e-4f);
  EXPECT_NEAR((*completed)[16], 2.0f, 1e-4f);
  EXPECT_EQ((*completed)[100], std::numeric_limits<float>::infinity());

  // The wrap-triggering point starts the next rotation.
  EXPECT_EQ(rasterizer.size(), 1u);
}

TEST(ScanRasterizer, KeepsClosestPointPerBin) {
  ScanRasterizer rasterizer;
  rasterizer.add_point({0.0f, 2000.0f});
  rasterizer.add_point({0.0f, 500.0f});  // same bin, closer -- should win
  auto completed = rasterizer.add_point({6.0f, 1000.0f});
  ASSERT_FALSE(completed.has_value());

  // Force a wrap to inspect the rasterized result.
  completed = rasterizer.add_point({0.0f, 999.0f});
  ASSERT_TRUE(completed.has_value());
  EXPECT_NEAR((*completed)[359], 0.5f, 1e-4f);
}

TEST(ScanRasterizer, ScanBinsConstantsAreConsistent) {
  EXPECT_EQ(ScanRasterizer::kScanBins, 360);
  EXPECT_NEAR(ScanRasterizer::kScanAngleInc * ScanRasterizer::kScanBins,
              2.0f * static_cast<float>(M_PI), 1e-4f);
  EXPECT_LT(ScanRasterizer::kRangeMin, ScanRasterizer::kRangeMax);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
