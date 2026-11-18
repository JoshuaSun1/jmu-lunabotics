// Copyright 2026 JMU NASA Lunabotics Team
// SPDX-License-Identifier: MIT

#pragma once

#include <cstddef>
#include <cstdint>
#include <unordered_map>
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

// How depth frames are accumulated into the local height map.
struct FusionConfig {
  // Frames that must observe a cell before it is classified.
  int min_observations = 3;
  // Length of the exponential averaging window, in frames.
  int window_frames = 10;
  // Cells not refreshed for this long are forgotten (limits odometry drift).
  double cell_memory_s = 8.0;
  // Cells farther than this from base_link are forgotten.
  double map_radius_m = 5.0;
  // A cell must exceed a threshold by this many standard errors.
  double confidence_sigma = 2.0;
  // Frames of occlusion evidence before an unseen cell is a hidden drop.
  int min_hidden_frames = 2;
  // Allowed height error when checking that a gap is a real line-of-sight shadow.
  double occlusion_tolerance_m = 0.04;
};

struct PlanarPose {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct TerrainFrame {
  // Depth samples already transformed into the fusion frame.
  std::vector<TerrainPoint> points;
  // base_link pose in the fusion frame at the image timestamp.
  PlanarPose base;
  // Camera origin in the fusion frame. Hidden-drop inference needs it.
  bool has_camera = false;
  TerrainPoint camera;
  double stamp_s = 0.0;
};

struct TerrainResult {
  bool ground_valid = false;
  // Points are in the fusion frame.
  std::vector<TerrainPoint> raised_obstacles;
  std::vector<TerrainPoint> terrain_hazards;
  // Per-frame counts.
  std::size_t observed_cells = 0;
  std::size_t unobserved_ground_cells = 0;
  std::size_t ground_inlier_cells = 0;
  std::size_t hidden_drop_cells = 0;
  // Map counts after this frame.
  std::size_t map_cells = 0;
  std::size_t confident_cells = 0;
};

// Accumulates per-frame terrain heights, measured relative to each frame's own
// ground plane, into a world-aligned grid in the fusion frame (normally odom).
class TerrainMapper {
public:
  struct MapCell {
    double mean = 0.0;
    double variance = 0.0;
    int observations = 0;
    int hidden_frames = 0;
    double last_update_s = 0.0;
    bool raised = false;
    bool hazard = false;
  };

  TerrainMapper(const TerrainConfig& terrain, const FusionConfig& fusion);

  // Rejected frames (no valid ground) leave the map unchanged.
  TerrainResult Integrate(const TerrainFrame& frame);
  void Reset();

private:
  TerrainConfig terrain_;
  FusionConfig fusion_;
  std::unordered_map<std::int64_t, MapCell> cells_;
  bool has_previous_ = false;
  double previous_stamp_s_ = 0.0;
  PlanarPose previous_base_;
};

// Single-frame detection. Input points have already been projected into base_link.
TerrainResult DetectTerrain(const std::vector<TerrainPoint>& points, const TerrainConfig& config);
void ValidateTerrainConfig(const TerrainConfig& config);
void ValidateFusionConfig(const FusionConfig& fusion, const TerrainConfig& terrain);
bool IsFreshDepthFrame(double age_s, double max_age_s);

} // namespace lb_sensors
