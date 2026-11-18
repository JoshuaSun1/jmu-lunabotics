// Copyright 2026 JMU NASA Lunabotics Team
// SPDX-License-Identifier: MIT

#include "lb_sensors/terrain_hazard_core.hpp"

#include <cmath>
#include <cstdint>
#include <functional>
#include <gtest/gtest.h>
#include <random>
#include <stdexcept>
#include <vector>

namespace {

using lb_sensors::FusionConfig;
using lb_sensors::TerrainConfig;
using lb_sensors::TerrainFrame;
using lb_sensors::TerrainMapper;
using lb_sensors::TerrainPoint;
using lb_sensors::TerrainResult;
using Height = std::function<float(float, float)>;

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

// World surface sampled on a 2.5 cm grid ahead of a robot at (base_x, 0) facing +x.
std::vector<TerrainPoint> Surface(const Height& height, float base_x = 0.0F,
                                  float half_width = 0.6F) {
  std::vector<TerrainPoint> points;
  for (float forward = 0.3125F; forward < 4.0F; forward += 0.025F) {
    for (float y = -half_width + 0.0125F; y < half_width; y += 0.025F) {
      const float x = base_x + forward;
      points.push_back({x, y, height(x, y)});
    }
  }
  return points;
}

// Keeps only the points whose line of sight from the camera clears the surface.
std::vector<TerrainPoint> Visible(const std::vector<TerrainPoint>& points, const Height& height,
                                  const TerrainPoint& camera) {
  std::vector<TerrainPoint> visible;
  for (const auto& point : points) {
    const double distance = std::hypot(point.x - camera.x, point.y - camera.y);
    bool clear = true;
    for (double along = 0.01; clear && along < distance - 0.002; along += 0.01) {
      const double t = along / distance;
      const auto x = static_cast<float>(camera.x + t * (point.x - camera.x));
      const auto y = static_cast<float>(camera.y + t * (point.y - camera.y));
      clear = camera.z + t * (point.z - camera.z) >= height(x, y) - 1.0e-4;
    }
    if (clear) {
      visible.push_back(point);
    }
  }
  return visible;
}

// Stereo-like noise that grows with range: sigma_at_2m at 2 m forward.
std::vector<TerrainPoint> AddNoise(std::vector<TerrainPoint> points, double sigma_at_2m,
                                   std::uint32_t seed) {
  std::mt19937 generator(seed);
  const auto uniform = [&generator]() { return (generator() + 0.5) / 4294967296.0; };
  for (auto& point : points) {
    const double gaussian =
        std::sqrt(-2.0 * std::log(uniform())) * std::cos(2.0 * 3.14159265358979 * uniform());
    point.z += static_cast<float>(gaussian * sigma_at_2m * point.x / 2.0);
  }
  return points;
}

TerrainFrame Frame(std::vector<TerrainPoint> points, double stamp_s, float base_x = 0.0F) {
  TerrainFrame frame;
  frame.points = std::move(points);
  frame.base.x = base_x;
  frame.stamp_s = stamp_s;
  return frame;
}

TerrainFrame CameraFrame(std::vector<TerrainPoint> points, double stamp_s) {
  TerrainFrame frame = Frame(std::move(points), stamp_s);
  frame.has_camera = true;
  frame.camera = {0.0F, 0.0F, 0.6F};
  return frame;
}

std::size_t CountInBox(const std::vector<TerrainPoint>& points, float min_x, float max_x,
                       float half_width) {
  std::size_t count = 0;
  for (const auto& point : points) {
    count += point.x > min_x && point.x < max_x && std::abs(point.y) < half_width ? 1U : 0U;
  }
  return count;
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

TEST(TerrainHazardFusion, AveragingSuppressesDepthNoise) {
  const auto flat = [](float, float) { return 0.0F; };
  const auto single = lb_sensors::DetectTerrain(AddNoise(Surface(flat), 0.02, 1U), TestConfig());
  ASSERT_TRUE(single.ground_valid);
  EXPECT_GT(single.terrain_hazards.size(), 100U);

  TerrainMapper mapper(TestConfig(), FusionConfig{});
  TerrainResult fused;
  for (int frame = 0; frame < 10; ++frame) {
    fused = mapper.Integrate(Frame(AddNoise(Surface(flat), 0.02, 100U + frame), 0.1 * frame));
    ASSERT_TRUE(fused.ground_valid);
  }
  EXPECT_EQ(fused.confident_cells, fused.map_cells);
  EXPECT_TRUE(fused.terrain_hazards.empty());
  EXPECT_TRUE(fused.raised_obstacles.empty());
}

TEST(TerrainHazardFusion, NoisyRockIsStillMarked) {
  const auto rock = [](float x, float y) {
    return x > 1.7F && x < 2.0F && std::abs(y) < 0.2F ? 0.16F : 0.0F;
  };
  TerrainMapper mapper(TestConfig(), FusionConfig{});
  TerrainResult fused;
  for (int frame = 0; frame < 10; ++frame) {
    fused = mapper.Integrate(Frame(AddNoise(Surface(rock), 0.02, 200U + frame), 0.1 * frame));
  }
  ASSERT_TRUE(fused.ground_valid);
  EXPECT_GT(CountInBox(fused.raised_obstacles, 1.7F, 2.0F, 0.2F), 20U);
  EXPECT_EQ(CountInBox(fused.raised_obstacles, 0.0F, 1.6F, 1.0F), 0U);
}

TEST(TerrainHazardFusion, CellsNeedRepeatedObservations) {
  const auto crater = [](float x, float y) {
    return x > 1.8F && x < 2.5F && std::abs(y) < 0.3F ? -0.18F : 0.0F;
  };
  TerrainMapper mapper(TestConfig(), FusionConfig{});
  EXPECT_TRUE(mapper.Integrate(Frame(Surface(crater), 0.0)).terrain_hazards.empty());
  EXPECT_TRUE(mapper.Integrate(Frame(Surface(crater), 0.1)).terrain_hazards.empty());
  EXPECT_GT(mapper.Integrate(Frame(Surface(crater), 0.2)).terrain_hazards.size(), 10U);
}

TEST(TerrainHazardFusion, RemembersHazardBehindCameraUntilMemoryExpires) {
  const auto crater = [](float x, float y) {
    return x > 1.0F && x < 1.4F && std::abs(y) < 0.3F ? -0.2F : 0.0F;
  };
  TerrainMapper mapper(TestConfig(), FusionConfig{});
  double stamp = 0.0;
  TerrainResult result;
  for (int step = 0; step <= 28; ++step, stamp += 0.1) {
    const float base_x = 0.05F * step;
    result = mapper.Integrate(Frame(Surface(crater, base_x), stamp, base_x));
    ASSERT_TRUE(result.ground_valid);
  }
  // The window now starts at x = 1.7, past the crater.
  EXPECT_GT(CountInBox(result.terrain_hazards, 1.0F, 1.4F, 0.3F), 10U);

  stamp += FusionConfig{}.cell_memory_s;
  result = mapper.Integrate(Frame(Surface(crater, 1.4F), stamp, 1.4F));
  ASSERT_TRUE(result.ground_valid);
  EXPECT_EQ(CountInBox(result.terrain_hazards, 1.0F, 1.4F, 0.3F), 0U);
}

TEST(TerrainHazardFusion, TimestampRewindClearsMap) {
  const auto crater = [](float x, float y) {
    return x > 1.8F && x < 2.5F && std::abs(y) < 0.3F ? -0.18F : 0.0F;
  };
  FusionConfig fusion;
  fusion.min_observations = 1;
  TerrainMapper mapper(TestConfig(), fusion);
  ASSERT_GT(mapper.Integrate(Frame(Surface(crater), 5.0)).terrain_hazards.size(), 10U);

  // The rewound frame never sees the crater; only stale map state could report it.
  std::vector<TerrainPoint> no_crater_view;
  for (const auto& point : Surface([](float, float) { return 0.0F; })) {
    if (point.x < 1.7F) {
      no_crater_view.push_back(point);
    }
  }
  const auto rewound = mapper.Integrate(Frame(no_crater_view, 1.0));
  ASSERT_TRUE(rewound.ground_valid);
  EXPECT_TRUE(rewound.terrain_hazards.empty());
}

TEST(TerrainHazardOcclusion, HiddenCraterFloorIsMarked) {
  // A 0.5 m deep crater with vertical walls. From 0.6 m up the camera sees the
  // top of the far wall but none of the floor.
  const auto crater = [](float x, float y) {
    return x >= 2.0F && x < 2.6F && std::abs(y) < 0.4F ? -0.5F : 0.0F;
  };
  const TerrainPoint camera{0.0F, 0.0F, 0.6F};
  auto candidates = Surface(crater);
  for (float y = -0.3875F; y < 0.4F; y += 0.025F) {
    for (float z = -0.5F; z <= 0.0F; z += 0.02F) {
      candidates.push_back({2.599F, y, z});
    }
  }
  const auto visible = Visible(candidates, crater, camera);
  ASSERT_EQ(CountInBox(visible, 2.0F, 2.55F, 0.4F), 0U);

  const auto blind = lb_sensors::DetectTerrain(visible, TestConfig());
  ASSERT_TRUE(blind.ground_valid);
  EXPECT_EQ(CountInBox(blind.terrain_hazards, 2.0F, 2.55F, 0.35F), 0U);

  TerrainMapper mapper(TestConfig(), FusionConfig{});
  const auto first = mapper.Integrate(CameraFrame(visible, 0.0));
  ASSERT_TRUE(first.ground_valid);
  EXPECT_GT(first.hidden_drop_cells, 20U);
  // One frame of occlusion evidence is not enough on its own.
  EXPECT_EQ(CountInBox(first.terrain_hazards, 2.0F, 2.55F, 0.35F), 0U);

  const auto second = mapper.Integrate(CameraFrame(visible, 0.1));
  EXPECT_GT(CountInBox(second.terrain_hazards, 2.25F, 2.55F, 0.35F), 20U);
  EXPECT_EQ(CountInBox(second.terrain_hazards, 0.0F, 1.95F, 1.0F), 0U);
}

TEST(TerrainHazardOcclusion, StereoDropoutOnVisibleGroundIsNotAHazard) {
  std::vector<TerrainPoint> dropout;
  for (const auto& point : Surface([](float, float) { return 0.0F; })) {
    if (!(point.x >= 2.0F && point.x < 2.6F && std::abs(point.y) < 0.3F)) {
      dropout.push_back(point);
    }
  }
  TerrainMapper mapper(TestConfig(), FusionConfig{});
  TerrainResult result;
  for (int frame = 0; frame < 5; ++frame) {
    result = mapper.Integrate(CameraFrame(dropout, 0.1 * frame));
  }
  ASSERT_TRUE(result.ground_valid);
  EXPECT_GT(result.unobserved_ground_cells, 20U);
  EXPECT_EQ(result.hidden_drop_cells, 0U);
  EXPECT_TRUE(result.terrain_hazards.empty());
}

TEST(TerrainHazardOcclusion, RockShadowIsNotAHazard) {
  const auto rock = [](float x, float y) {
    return x >= 1.7F && x < 2.0F && std::abs(y) < 0.3F ? 0.3F : 0.0F;
  };
  const auto visible = Visible(Surface(rock), rock, {0.0F, 0.0F, 0.6F});
  TerrainMapper mapper(TestConfig(), FusionConfig{});
  TerrainResult result;
  for (int frame = 0; frame < 5; ++frame) {
    result = mapper.Integrate(CameraFrame(visible, 0.1 * frame));
  }
  ASSERT_TRUE(result.ground_valid);
  EXPECT_GT(result.unobserved_ground_cells, 20U);
  EXPECT_EQ(result.hidden_drop_cells, 0U);
  EXPECT_GT(CountInBox(result.raised_obstacles, 1.7F, 2.0F, 0.3F), 20U);
  EXPECT_EQ(CountInBox(result.terrain_hazards, 2.05F, 4.0F, 1.0F), 0U);
}

TEST(TerrainHazardFusion, InvalidFusionSettingsAreRejected) {
  FusionConfig fusion;
  fusion.min_observations = 12;
  EXPECT_THROW(TerrainMapper(TestConfig(), fusion), std::invalid_argument);
  fusion = FusionConfig{};
  fusion.map_radius_m = 3.0;
  EXPECT_THROW(TerrainMapper(TestConfig(), fusion), std::invalid_argument);
  fusion = FusionConfig{};
  fusion.cell_memory_s = 0.0;
  EXPECT_THROW(TerrainMapper(TestConfig(), fusion), std::invalid_argument);
}

} // namespace
