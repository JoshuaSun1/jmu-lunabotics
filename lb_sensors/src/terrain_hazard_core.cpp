// Copyright 2026 JMU NASA Lunabotics Team
// SPDX-License-Identifier: MIT

#include "lb_sensors/terrain_hazard_core.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_set>
#include <utility>
#include <vector>

namespace lb_sensors {
namespace {

constexpr double kPi = 3.14159265358979323846;
// Ground is fitted to cells this far beyond min_forward_m.
constexpr double kGroundFitDepthM = 1.2;
// Synthetic hazard points sit this far above the estimated ground.
constexpr double kHazardMarkerHeightM = 0.05;
// A larger jump between integrated frames means odometry reset or jumped.
constexpr double kMaxFrameJumpM = 1.0;

struct Plane {
  double a = 0.0;
  double b = 0.0;
  double c = 0.0;

  double Height(double x, double y) const {
    return a * x + b * y + c;
  }

  double Height(const TerrainPoint& point) const {
    return Height(point.x, point.y);
  }
};

bool PlaneThrough(const TerrainPoint& p, const TerrainPoint& q, const TerrainPoint& r,
                  Plane* plane) {
  const double x1 = q.x - p.x;
  const double y1 = q.y - p.y;
  const double z1 = q.z - p.z;
  const double x2 = r.x - p.x;
  const double y2 = r.y - p.y;
  const double z2 = r.z - p.z;
  const double determinant = x1 * y2 - x2 * y1;
  if (std::abs(determinant) < 1.0e-5) {
    return false;
  }
  plane->a = (z1 * y2 - z2 * y1) / determinant;
  plane->b = (x1 * z2 - x2 * z1) / determinant;
  plane->c = p.z - plane->a * p.x - plane->b * p.y;
  return std::hypot(plane->a, plane->b) <= 1.0;
}

// Refit the selected RANSAC inliers so one noisy triplet cannot set the baseline.
bool RefitPlane(const std::vector<TerrainPoint>& points, const Plane& initial, double tolerance,
                Plane* fitted, std::size_t* inlier_count) {
  std::array<std::array<double, 4>, 3> matrix{};
  *inlier_count = 0;
  for (const auto& point : points) {
    if (std::abs(point.z - initial.Height(point)) > tolerance) {
      continue;
    }
    ++*inlier_count;
    const std::array<double, 3> row{point.x, point.y, 1.0};
    for (std::size_t i = 0; i < 3; ++i) {
      for (std::size_t j = 0; j < 3; ++j) {
        matrix[i][j] += row[i] * row[j];
      }
      matrix[i][3] += row[i] * point.z;
    }
  }
  for (std::size_t column = 0; column < 3; ++column) {
    std::size_t pivot = column;
    for (std::size_t row = column + 1; row < 3; ++row) {
      if (std::abs(matrix[row][column]) > std::abs(matrix[pivot][column])) {
        pivot = row;
      }
    }
    if (std::abs(matrix[pivot][column]) < 1.0e-8) {
      return false;
    }
    std::swap(matrix[pivot], matrix[column]);
    const double divisor = matrix[column][column];
    for (std::size_t j = column; j < 4; ++j) {
      matrix[column][j] /= divisor;
    }
    for (std::size_t row = 0; row < 3; ++row) {
      if (row == column) {
        continue;
      }
      const double factor = matrix[row][column];
      for (std::size_t j = column; j < 4; ++j) {
        matrix[row][j] -= factor * matrix[column][j];
      }
    }
  }
  *fitted = {matrix[0][3], matrix[1][3], matrix[2][3]};
  return std::hypot(fitted->a, fitted->b) <= 1.0;
}

bool EstimateGround(const std::vector<TerrainPoint>& near_points, const TerrainConfig& config,
                    Plane* ground, std::size_t* ground_inliers) {
  if (near_points.size() < static_cast<std::size_t>(config.min_ground_cells)) {
    return false;
  }
  std::uint32_t state = 0x5EED1234U;
  std::size_t best_count = 0;
  double best_error = std::numeric_limits<double>::infinity();
  Plane best;
  for (int trial = 0; trial < 120; ++trial) {
    auto draw = [&state, &near_points]() {
      state = state * 1664525U + 1013904223U;
      return static_cast<std::size_t>(state >> 8U) % near_points.size();
    };
    const auto i = draw();
    const auto j = draw();
    const auto k = draw();
    if (i == j || i == k || j == k) {
      continue;
    }
    Plane candidate;
    if (!PlaneThrough(near_points[i], near_points[j], near_points[k], &candidate)) {
      continue;
    }
    std::size_t count = 0;
    double error = 0.0;
    for (const auto& point : near_points) {
      const double residual = std::abs(point.z - candidate.Height(point));
      if (residual <= config.ground_inlier_tolerance_m) {
        ++count;
        error += residual;
      }
    }
    if (count > best_count || (count == best_count && error < best_error)) {
      best_count = count;
      best_error = error;
      best = candidate;
    }
  }
  if (best_count < static_cast<std::size_t>(config.min_ground_cells) ||
      best_count * 2U < near_points.size()) {
    return false;
  }
  return RefitPlane(near_points, best, config.ground_inlier_tolerance_m, ground, ground_inliers);
}

std::int64_t CellKey(int ix, int iy) {
  return static_cast<std::int64_t>(
      (static_cast<std::uint64_t>(static_cast<std::uint32_t>(ix)) << 32U) |
      static_cast<std::uint64_t>(static_cast<std::uint32_t>(iy)));
}

int KeyX(std::int64_t key) {
  return static_cast<std::int32_t>(static_cast<std::uint64_t>(key) >> 32U);
}

int KeyY(std::int64_t key) {
  return static_cast<std::int32_t>(static_cast<std::uint64_t>(key) & 0xFFFFFFFFU);
}

// The detection window is a rectangle in base_link; the grid is aligned with the fusion frame.
class RobotWindow {
public:
  RobotWindow(const PlanarPose& base, const TerrainConfig& config)
      : base_(base), config_(config), cos_yaw_(std::cos(base.yaw)), sin_yaw_(std::sin(base.yaw)) {}

  void ToBase(double x, double y, double* forward, double* left) const {
    const double dx = x - base_.x;
    const double dy = y - base_.y;
    *forward = cos_yaw_ * dx + sin_yaw_ * dy;
    *left = -sin_yaw_ * dx + cos_yaw_ * dy;
  }

  void ToWorld(double forward, double left, double* x, double* y) const {
    *x = base_.x + cos_yaw_ * forward - sin_yaw_ * left;
    *y = base_.y + sin_yaw_ * forward + cos_yaw_ * left;
  }

  bool Contains(double x, double y) const {
    double forward = 0.0;
    double left = 0.0;
    ToBase(x, y, &forward, &left);
    return forward >= config_.min_forward_m && forward < config_.max_forward_m &&
           std::abs(left) < config_.lateral_limit_m;
  }

  std::array<std::array<double, 2>, 4> Corners() const {
    return {{{{config_.min_forward_m, -config_.lateral_limit_m}},
             {{config_.min_forward_m, config_.lateral_limit_m}},
             {{config_.max_forward_m, -config_.lateral_limit_m}},
             {{config_.max_forward_m, config_.lateral_limit_m}}}};
  }

  double yaw() const {
    return base_.yaw;
  }

private:
  PlanarPose base_;
  TerrainConfig config_;
  double cos_yaw_;
  double sin_yaw_;
};

struct FrameCell {
  float height = 0.0F;
  float lowest = 0.0F;
  float highest = 0.0F;
  bool observed = false;
  bool gap = false;
  bool hidden = false;
  // A ray crossed this unseen cell with a clear line of sight to the far side.
  bool contradicted = false;
};

// This frame's cells over the window's bounding box, indexed like the map.
struct FrameGrid {
  int ix0 = 0;
  int iy0 = 0;
  int nx = 0;
  int ny = 0;
  double cell = 0.05;
  std::vector<FrameCell> cells;

  bool Index(double x, double y, std::size_t* index) const {
    const int ix = static_cast<int>(std::floor(x / cell)) - ix0;
    const int iy = static_cast<int>(std::floor(y / cell)) - iy0;
    if (ix < 0 || ix >= nx || iy < 0 || iy >= ny) {
      return false;
    }
    *index = static_cast<std::size_t>(ix) * ny + iy;
    return true;
  }

  int CellX(std::size_t index) const {
    return ix0 + static_cast<int>(index / ny);
  }

  int CellY(std::size_t index) const {
    return iy0 + static_cast<int>(index % ny);
  }

  double CenterX(std::size_t index) const {
    return (CellX(index) + 0.5) * cell;
  }

  double CenterY(std::size_t index) const {
    return (CellY(index) + 0.5) * cell;
  }
};

FrameGrid BuildFrameGrid(const std::vector<TerrainPoint>& points, const RobotWindow& window,
                         double cell_size) {
  FrameGrid grid;
  grid.cell = cell_size;
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = min_x;
  double max_x = -min_x;
  double max_y = -min_x;
  for (const auto& corner : window.Corners()) {
    double x = 0.0;
    double y = 0.0;
    window.ToWorld(corner[0], corner[1], &x, &y);
    min_x = std::min(min_x, x);
    min_y = std::min(min_y, y);
    max_x = std::max(max_x, x);
    max_y = std::max(max_y, y);
  }
  grid.ix0 = static_cast<int>(std::floor(min_x / cell_size));
  grid.iy0 = static_cast<int>(std::floor(min_y / cell_size));
  grid.nx = static_cast<int>(std::floor(max_x / cell_size)) - grid.ix0 + 1;
  grid.ny = static_cast<int>(std::floor(max_y / cell_size)) - grid.iy0 + 1;
  grid.cells.resize(static_cast<std::size_t>(grid.nx) * grid.ny);

  std::vector<std::pair<std::size_t, float>> samples;
  samples.reserve(points.size());
  for (const auto& point : points) {
    std::size_t index = 0;
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
        !window.Contains(point.x, point.y) || !grid.Index(point.x, point.y, &index)) {
      continue;
    }
    samples.emplace_back(index, point.z);
  }
  std::sort(samples.begin(), samples.end());
  for (std::size_t begin = 0; begin < samples.size();) {
    std::size_t end = begin;
    while (end < samples.size() && samples[end].first == samples[begin].first) {
      ++end;
    }
    auto& cell = grid.cells[samples[begin].first];
    cell.observed = true;
    cell.height = samples[begin + (end - begin) / 2].second;
    cell.lowest = samples[begin].second;
    cell.highest = samples[end - 1].second;
    begin = end;
  }
  return grid;
}

// Walk rays outward from the camera across this frame's cells. Runs of missing
// depth bracketed by measurements are unobserved ground. With a known camera
// height, a run cell is a hidden drop when the sight line grazing the nearer
// terrain already passes below -drop_depth_m there, and the far side of the run
// is measured at or below that sight line. The second test separates a real
// shadow (crater interior) from a stereo dropout on visible flat ground, whose
// far side sits well above the sight line; such rays count against the cell.
// Rock shadows never qualify because the sight line over a rock stays above
// the ground until the shadow ends.
void ScanRays(const TerrainConfig& config, const FusionConfig& fusion, const RobotWindow& window,
              double origin_x, double origin_y, const Plane* ground, const TerrainPoint* camera,
              FrameGrid* grid, TerrainResult* result) {
  const double camera_height =
      ground != nullptr && camera != nullptr ? camera->z - ground->Height(*camera) : 0.0;
  const bool infer_hidden = camera_height > config.ground_inlier_tolerance_m;

  double origin_forward = 0.0;
  double origin_left = 0.0;
  window.ToBase(origin_x, origin_y, &origin_forward, &origin_left);
  double min_azimuth = -kPi;
  double max_azimuth = kPi;
  double max_range = 0.0;
  if (origin_forward < config.min_forward_m) {
    min_azimuth = kPi;
    max_azimuth = -kPi;
  }
  for (const auto& corner : window.Corners()) {
    const double forward = corner[0] - origin_forward;
    const double left = corner[1] - origin_left;
    max_range = std::max(max_range, std::hypot(forward, left));
    if (origin_forward < config.min_forward_m) {
      min_azimuth = std::min(min_azimuth, std::atan2(left, forward));
      max_azimuth = std::max(max_azimuth, std::atan2(left, forward));
    }
  }

  const double step = 0.5 * config.cell_size_m;
  const double azimuth_step = step / max_range;
  std::vector<std::pair<std::size_t, double>> gap;
  for (double azimuth = min_azimuth; azimuth <= max_azimuth; azimuth += azimuth_step) {
    const double direction_x = std::cos(window.yaw() + azimuth);
    const double direction_y = std::sin(window.yaw() + azimuth);
    bool entered = false;
    bool seen = false;
    double horizon_slope = -std::numeric_limits<double>::infinity();
    gap.clear();
    for (double range = step; range <= max_range; range += step) {
      const double x = origin_x + range * direction_x;
      const double y = origin_y + range * direction_y;
      std::size_t index = 0;
      if (!window.Contains(x, y) || !grid->Index(x, y, &index)) {
        if (entered) {
          break;
        }
        continue;
      }
      entered = true;
      const auto& cell = grid->cells[index];
      if (!cell.observed) {
        if (seen) {
          gap.emplace_back(index, range);
        }
        continue;
      }
      const double ground_height =
          ground != nullptr ? ground->Height(grid->CenterX(index), grid->CenterY(index)) : 0.0;
      if (!gap.empty()) {
        const bool in_shadow =
            infer_hidden && cell.lowest - ground_height <= camera_height + horizon_slope * range +
                                                               fusion.occlusion_tolerance_m;
        for (const auto& [gap_index, gap_range] : gap) {
          auto& gap_cell = grid->cells[gap_index];
          if (!gap_cell.gap) {
            gap_cell.gap = true;
            ++result->unobserved_ground_cells;
          }
          if (in_shadow && !gap_cell.hidden &&
              camera_height + horizon_slope * gap_range <= -config.drop_depth_m) {
            gap_cell.hidden = true;
            ++result->hidden_drop_cells;
          } else if (infer_hidden && !in_shadow) {
            gap_cell.contradicted = true;
          }
        }
        gap.clear();
      }
      seen = true;
      if (infer_hidden) {
        horizon_slope =
            std::max(horizon_slope, (cell.highest - ground_height - camera_height) / range);
      }
    }
  }
}

void AppendClusters(const std::unordered_map<std::int64_t, TerrainMapper::MapCell>& cells,
                    const std::vector<std::int64_t>& keys, bool raised, const Plane& ground,
                    const TerrainConfig& config, std::vector<TerrainPoint>* output) {
  const auto marked = [raised](const TerrainMapper::MapCell& cell) {
    return raised ? cell.raised : cell.hazard;
  };
  std::unordered_set<std::int64_t> visited;
  for (const auto start : keys) {
    if (!marked(cells.at(start)) || visited.count(start) != 0) {
      continue;
    }
    std::queue<std::int64_t> pending;
    std::vector<std::int64_t> cluster;
    pending.push(start);
    visited.insert(start);
    while (!pending.empty()) {
      const auto key = pending.front();
      pending.pop();
      cluster.push_back(key);
      const int ix = KeyX(key);
      const int iy = KeyY(key);
      for (const auto next :
           {CellKey(ix - 1, iy), CellKey(ix + 1, iy), CellKey(ix, iy - 1), CellKey(ix, iy + 1)}) {
        const auto found = cells.find(next);
        if (found != cells.end() && marked(found->second) && visited.insert(next).second) {
          pending.push(next);
        }
      }
    }
    if (cluster.size() < static_cast<std::size_t>(config.min_cluster_cells)) {
      continue;
    }
    for (const auto key : cluster) {
      TerrainPoint point;
      point.x = static_cast<float>((KeyX(key) + 0.5) * config.cell_size_m);
      point.y = static_cast<float>((KeyY(key) + 0.5) * config.cell_size_m);
      const double offset = raised ? cells.at(key).mean : kHazardMarkerHeightM;
      point.z = static_cast<float>(ground.Height(point) + offset);
      output->push_back(point);
    }
  }
}

} // namespace

void ValidateTerrainConfig(const TerrainConfig& config) {
  if (!(std::isfinite(config.min_forward_m) && config.min_forward_m >= 0.0 &&
        std::isfinite(config.max_forward_m) && config.max_forward_m > config.min_forward_m &&
        config.max_forward_m <= 10.0 && std::isfinite(config.lateral_limit_m) &&
        config.lateral_limit_m > 0.0 && config.lateral_limit_m <= 5.0 &&
        std::isfinite(config.cell_size_m) && config.cell_size_m >= 0.02 &&
        std::isfinite(config.ground_inlier_tolerance_m) && config.ground_inlier_tolerance_m > 0.0 &&
        std::isfinite(config.rock_height_m) && config.rock_height_m > 0.0 &&
        std::isfinite(config.drop_depth_m) && config.drop_depth_m > 0.0 &&
        std::isfinite(config.max_slope_deg) && config.max_slope_deg > 0.0 &&
        config.max_slope_deg < 80.0 && config.min_cluster_cells >= 1 &&
        config.min_ground_cells >= 3)) {
    throw std::invalid_argument("terrain geometry/clearance parameters are missing or invalid");
  }
}

void ValidateFusionConfig(const FusionConfig& fusion, const TerrainConfig& terrain) {
  if (!(fusion.min_observations >= 1 && fusion.window_frames >= fusion.min_observations &&
        std::isfinite(fusion.cell_memory_s) && fusion.cell_memory_s > 0.0 &&
        std::isfinite(fusion.map_radius_m) &&
        fusion.map_radius_m >= std::hypot(terrain.max_forward_m, terrain.lateral_limit_m) &&
        std::isfinite(fusion.confidence_sigma) && fusion.confidence_sigma >= 0.0 &&
        fusion.min_hidden_frames >= 1 && std::isfinite(fusion.occlusion_tolerance_m) &&
        fusion.occlusion_tolerance_m > 0.0)) {
    throw std::invalid_argument("terrain fusion parameters are invalid");
  }
}

bool IsFreshDepthFrame(double age_s, double max_age_s) {
  return std::isfinite(age_s) && std::isfinite(max_age_s) && max_age_s > 0.0 && age_s >= -0.1 &&
         age_s <= max_age_s;
}

TerrainMapper::TerrainMapper(const TerrainConfig& terrain, const FusionConfig& fusion)
    : terrain_(terrain), fusion_(fusion) {
  ValidateTerrainConfig(terrain_);
  ValidateFusionConfig(fusion_, terrain_);
}

void TerrainMapper::Reset() {
  cells_.clear();
  has_previous_ = false;
}

TerrainResult TerrainMapper::Integrate(const TerrainFrame& frame) {
  if (has_previous_ && (frame.stamp_s < previous_stamp_s_ - 1.0e-3 ||
                        std::hypot(frame.base.x - previous_base_.x,
                                   frame.base.y - previous_base_.y) > kMaxFrameJumpM)) {
    Reset();
  }
  has_previous_ = true;
  previous_stamp_s_ = frame.stamp_s;
  previous_base_ = frame.base;

  TerrainResult result;
  const double cell_size = terrain_.cell_size_m;
  const RobotWindow window(frame.base, terrain_);
  FrameGrid grid = BuildFrameGrid(frame.points, window, cell_size);

  std::vector<TerrainPoint> near_points;
  for (std::size_t index = 0; index < grid.cells.size(); ++index) {
    if (!grid.cells[index].observed) {
      continue;
    }
    ++result.observed_cells;
    const double x = grid.CenterX(index);
    const double y = grid.CenterY(index);
    double forward = 0.0;
    double left = 0.0;
    window.ToBase(x, y, &forward, &left);
    if (forward <= terrain_.min_forward_m + kGroundFitDepthM) {
      near_points.push_back(
          {static_cast<float>(x), static_cast<float>(y), grid.cells[index].height});
    }
  }

  Plane ground;
  const bool ground_valid =
      EstimateGround(near_points, terrain_, &ground, &result.ground_inlier_cells);
  ScanRays(terrain_, fusion_, window, frame.has_camera ? frame.camera.x : frame.base.x,
           frame.has_camera ? frame.camera.y : frame.base.y, ground_valid ? &ground : nullptr,
           frame.has_camera ? &frame.camera : nullptr, &grid, &result);
  if (!ground_valid) {
    result.map_cells = cells_.size();
    return result;
  }
  result.ground_valid = true;

  // Each frame contributes one height per cell, relative to its own ground plane,
  // so robot pitch and odometry height drift do not bias the average.
  for (std::size_t index = 0; index < grid.cells.size(); ++index) {
    const auto& frame_cell = grid.cells[index];
    const auto key = CellKey(grid.CellX(index), grid.CellY(index));
    if (!frame_cell.observed && !frame_cell.hidden) {
      const auto found = frame_cell.contradicted ? cells_.find(key) : cells_.end();
      if (found != cells_.end()) {
        found->second.hidden_frames = std::max(found->second.hidden_frames - 1, 0);
      }
      continue;
    }
    auto& cell = cells_[key];
    cell.last_update_s = frame.stamp_s;
    if (frame_cell.hidden) {
      cell.hidden_frames = std::min(cell.hidden_frames + 1, fusion_.min_hidden_frames);
      continue;
    }
    const double residual =
        frame_cell.height - ground.Height(grid.CenterX(index), grid.CenterY(index));
    cell.observations = std::min(cell.observations + 1, fusion_.window_frames);
    const double alpha = 1.0 / cell.observations;
    const double delta = residual - cell.mean;
    cell.mean += alpha * delta;
    cell.variance = (1.0 - alpha) * (cell.variance + alpha * delta * delta);
  }

  std::vector<std::int64_t> keys;
  keys.reserve(cells_.size());
  for (auto it = cells_.begin(); it != cells_.end();) {
    const double x = (KeyX(it->first) + 0.5) * cell_size;
    const double y = (KeyY(it->first) + 0.5) * cell_size;
    if (frame.stamp_s - it->second.last_update_s > fusion_.cell_memory_s ||
        std::hypot(x - frame.base.x, y - frame.base.y) > fusion_.map_radius_m) {
      it = cells_.erase(it);
      continue;
    }
    it->second.raised = false;
    it->second.hazard = false;
    keys.push_back(it->first);
    ++it;
  }
  std::sort(keys.begin(), keys.end());

  const auto confident = [this](const MapCell& cell) {
    return cell.observations >= fusion_.min_observations;
  };
  const auto standard_error = [](const MapCell& cell) {
    return std::sqrt(cell.variance / cell.observations);
  };
  const double max_step = std::tan(terrain_.max_slope_deg * kPi / 180.0) * cell_size;
  for (const auto key : keys) {
    auto& cell = cells_.at(key);
    if (!confident(cell)) {
      // Seen too few times to trust: occlusion evidence alone decides.
      cell.hazard = cell.hidden_frames >= fusion_.min_hidden_frames;
      continue;
    }
    ++result.confident_cells;
    const double error = standard_error(cell);
    const double margin = fusion_.confidence_sigma * error;
    cell.raised = cell.mean - margin >= terrain_.rock_height_m;
    cell.hazard = cell.hazard || cell.mean + margin <= -terrain_.drop_depth_m;
    for (const auto next : {CellKey(KeyX(key) + 1, KeyY(key)), CellKey(KeyX(key), KeyY(key) + 1)}) {
      const auto found = cells_.find(next);
      if (found == cells_.end() || !confident(found->second)) {
        continue;
      }
      const double step_margin =
          fusion_.confidence_sigma * std::hypot(error, standard_error(found->second));
      if (std::abs(cell.mean - found->second.mean) - step_margin > max_step) {
        cell.hazard = true;
        found->second.hazard = true;
      }
    }
  }

  AppendClusters(cells_, keys, true, ground, terrain_, &result.raised_obstacles);
  AppendClusters(cells_, keys, false, ground, terrain_, &result.terrain_hazards);
  result.map_cells = cells_.size();
  return result;
}

TerrainResult DetectTerrain(const std::vector<TerrainPoint>& points, const TerrainConfig& config) {
  FusionConfig single_frame;
  single_frame.min_observations = 1;
  single_frame.window_frames = 1;
  single_frame.min_hidden_frames = 1;
  single_frame.map_radius_m = std::hypot(config.max_forward_m, config.lateral_limit_m);
  TerrainMapper mapper(config, single_frame);
  TerrainFrame frame;
  frame.points = points;
  return mapper.Integrate(frame);
}

} // namespace lb_sensors
