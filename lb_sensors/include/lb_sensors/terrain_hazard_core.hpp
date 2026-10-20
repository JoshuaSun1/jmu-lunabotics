// Copyright 2026 JMU NASA Lunabotics Team
// SPDX-License-Identifier: MIT

#pragma once

#include <cstddef>
#include <vector>

namespace lb_sensors {

struct TerrainPoint {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
};

struct TerrainConfig {
  double min_forward_m = 0.3;
  double max_forward_m = 4.0;
  double lateral_limit_m = 1.5;
  double cell_size_m = 0.05;
  double ground_inlier_tolerance_m = 0.04;
  double rock_height_m = -1.0;
  double drop_depth_m = -1.0;
  double max_slope_deg = -1.0;
  int min_cluster_cells = 3;
  int min_ground_cells = 30;
};

struct TerrainResult {
  bool ground_valid = false;
  std::vector<TerrainPoint> raised_obstacles;
  std::vector<TerrainPoint> terrain_hazards;
  std::size_t observed_cells = 0;
  std::size_t unobserved_ground_cells = 0;
  std::size_t ground_inlier_cells = 0;
};

// Pure geometry. Input points have already been projected into base_link.
TerrainResult DetectTerrain(const std::vector<TerrainPoint>& points, const TerrainConfig& config);
void ValidateTerrainConfig(const TerrainConfig& config);
bool IsFreshDepthFrame(double age_s, double max_age_s);

} // namespace lb_sensors
