// Copyright 2026 JMU NASA Lunabotics Team
// SPDX-License-Identifier: MIT

#include "lb_sensors/terrain_hazard_core.hpp"

#include <cmath>
#include <gtest/gtest.h>
#include <vector>

namespace {

using lb_sensors::TerrainConfig;
using lb_sensors::TerrainPoint;

TerrainConfig TestConfig() {
  TerrainConfig config;
  config.rock_height_m = 0.08;
  config.drop_depth_m = 0.10;
  config.max_slope_deg = 25.0;
  return config;
}

template <typename HeightFunction> std::vector<TerrainPoint> GroundPoints(HeightFunction height) {
  std::vector<TerrainPoint> points;
  for (int row = 0; row < 65; ++row) {
    const float x = 0.3F + (row + 0.5F) * 0.05F;
    for (int column = 0; column < 21; ++column) {
      const float y = -0.525F + (column + 0.5F) * 0.05F;
      points.push_back({x, y, height(x, y)});
    }
  }
  return points;
}

TEST(TerrainHazardCore, FlatGroundAndIsolatedMissingDepthAreClear) {
  const auto flat = GroundPoints([](float, float) { return 0.0F; });
  const auto clear = lb_sensors::DetectTerrain(flat, TestConfig());
  ASSERT_TRUE(clear.ground_valid);
  EXPECT_TRUE(clear.raised_obstacles.empty());
  EXPECT_TRUE(clear.terrain_hazards.empty());
  EXPECT_EQ(clear.unobserved_ground_cells, 0U);

  auto with_gap = GroundPoints(
      [](float x, float y) { return x > 1.3F && x < 1.36F && std::abs(y) < 0.03F ? NAN : 0.0F; });
  const auto gap = lb_sensors::DetectTerrain(with_gap, TestConfig());
  ASSERT_TRUE(gap.ground_valid);
  EXPECT_TRUE(gap.raised_obstacles.empty());
  EXPECT_TRUE(gap.terrain_hazards.empty());
  EXPECT_GT(gap.unobserved_ground_cells, 0U);

  const auto longer_gap = GroundPoints(
      [](float x, float y) { return x > 1.3F && x < 1.55F && std::abs(y) < 0.03F ? NAN : 0.0F; });
  const auto uncertain = lb_sensors::DetectTerrain(longer_gap, TestConfig());
  ASSERT_TRUE(uncertain.ground_valid);
  EXPECT_TRUE(uncertain.terrain_hazards.empty());
  EXPECT_GT(uncertain.unobserved_ground_cells, gap.unobserved_ground_cells);
}

TEST(TerrainHazardCore, MarksRaisedRock) {
  const auto points = GroundPoints(
      [](float x, float y) { return x > 1.7F && x < 2.0F && std::abs(y) < 0.2F ? 0.16F : 0.0F; });
  const auto result = lb_sensors::DetectTerrain(points, TestConfig());
  ASSERT_TRUE(result.ground_valid);
  EXPECT_GT(result.raised_obstacles.size(), 4U);
}

TEST(TerrainHazardCore, MarksMeasuredCraterAndItsRim) {
  const auto points = GroundPoints(
      [](float x, float y) { return x > 1.8F && x < 2.5F && std::abs(y) < 0.3F ? -0.18F : 0.0F; });
  const auto result = lb_sensors::DetectTerrain(points, TestConfig());
  ASSERT_TRUE(result.ground_valid);
  EXPECT_GT(result.terrain_hazards.size(), 10U);
  EXPECT_TRUE(result.raised_obstacles.empty());
}

TEST(TerrainHazardCore, MarksUnsafeSlope) {
  auto config = TestConfig();
  config.max_slope_deg = 12.0;
  config.rock_height_m = 0.5;
  config.drop_depth_m = 0.5;
  const auto points = GroundPoints([](float x, float y) {
    return x > 1.8F && x < 2.5F && std::abs(y) < 0.3F ? 0.4F * (x - 1.8F) : 0.0F;
  });
  const auto result = lb_sensors::DetectTerrain(points, config);
  ASSERT_TRUE(result.ground_valid);
  EXPECT_GT(result.terrain_hazards.size(), 4U);
}

TEST(TerrainHazardCore, InvalidCalibrationAndInsufficientGroundFailClosed) {
  auto config = TestConfig();
  config.rock_height_m = -1.0;
  EXPECT_THROW(lb_sensors::DetectTerrain({}, config), std::invalid_argument);

  config = TestConfig();
  const auto result = lb_sensors::DetectTerrain({{1.0F, 0.0F, 0.0F}}, config);
  EXPECT_FALSE(result.ground_valid);
  EXPECT_TRUE(result.raised_obstacles.empty());
  EXPECT_TRUE(result.terrain_hazards.empty());

  auto contradictory = GroundPoints([](float, float) { return 0.0F; });
  int index = 0;
  for (auto& point : contradictory) {
    if (point.x < 1.5F) {
      point.z = static_cast<float>((index++ % 3) - 1) * 0.2F;
    }
  }
  EXPECT_FALSE(lb_sensors::DetectTerrain(contradictory, config).ground_valid);
}

TEST(TerrainHazardCore, StaleAndInvalidTimestampsAreRejected) {
  EXPECT_TRUE(lb_sensors::IsFreshDepthFrame(0.05, 0.3));
  EXPECT_FALSE(lb_sensors::IsFreshDepthFrame(0.31, 0.3));
  EXPECT_FALSE(lb_sensors::IsFreshDepthFrame(NAN, 0.3));
  EXPECT_FALSE(lb_sensors::IsFreshDepthFrame(-0.2, 0.3));
}

} // namespace
