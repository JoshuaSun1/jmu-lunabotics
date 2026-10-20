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
#include <vector>

namespace lb_sensors {
namespace {

struct Cell {
  std::vector<float> heights;
  float height = 0.0F;
  bool observed = false;
};

struct Plane {
  double a = 0.0;
  double b = 0.0;
  double c = 0.0;

  double Height(const TerrainPoint& point) const {
    return a * point.x + b * point.y + c;
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

void AppendClusters(const std::vector<bool>& candidates, const std::vector<Cell>& cells, int rows,
                    int columns, const TerrainConfig& config, bool synthetic, const Plane& ground,
                    std::vector<TerrainPoint>* output) {
  std::vector<bool> visited(candidates.size(), false);
  for (std::size_t start = 0; start < candidates.size(); ++start) {
    if (!candidates[start] || visited[start]) {
      continue;
    }
    std::queue<std::size_t> pending;
    std::vector<std::size_t> cluster;
    pending.push(start);
    visited[start] = true;
    while (!pending.empty()) {
      const auto index = pending.front();
      pending.pop();
      cluster.push_back(index);
      const int row = static_cast<int>(index / columns);
      const int column = static_cast<int>(index % columns);
      const std::array<std::array<int, 2>, 4> neighbors{
          {{{row - 1, column}}, {{row + 1, column}}, {{row, column - 1}}, {{row, column + 1}}}};
      for (const auto& neighbor : neighbors) {
        if (neighbor[0] < 0 || neighbor[0] >= rows || neighbor[1] < 0 || neighbor[1] >= columns) {
          continue;
        }
        const auto next = static_cast<std::size_t>(neighbor[0] * columns + neighbor[1]);
        if (candidates[next] && !visited[next]) {
          visited[next] = true;
          pending.push(next);
        }
      }
    }
    if (cluster.size() < static_cast<std::size_t>(config.min_cluster_cells)) {
      continue;
    }
    for (const auto index : cluster) {
      const int row = static_cast<int>(index / columns);
      const int column = static_cast<int>(index % columns);
      TerrainPoint point;
      point.x = static_cast<float>(config.min_forward_m + (row + 0.5) * config.cell_size_m);
      point.y = static_cast<float>(-config.lateral_limit_m + (column + 0.5) * config.cell_size_m);
      point.z = synthetic ? static_cast<float>(ground.Height(point) + 0.05) : cells[index].height;
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

bool IsFreshDepthFrame(double age_s, double max_age_s) {
  return std::isfinite(age_s) && std::isfinite(max_age_s) && max_age_s > 0.0 && age_s >= -0.1 &&
         age_s <= max_age_s;
}

TerrainResult DetectTerrain(const std::vector<TerrainPoint>& points, const TerrainConfig& config) {
  ValidateTerrainConfig(config);
  TerrainResult result;
  const int rows = static_cast<int>(
      std::ceil((config.max_forward_m - config.min_forward_m) / config.cell_size_m));
  const int columns =
      static_cast<int>(std::ceil(2.0 * config.lateral_limit_m / config.cell_size_m));
  std::vector<Cell> cells(static_cast<std::size_t>(rows * columns));
  for (const auto& point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
        point.x < config.min_forward_m || point.x >= config.max_forward_m ||
        std::abs(point.y) >= config.lateral_limit_m) {
      continue;
    }
    const int row = static_cast<int>((point.x - config.min_forward_m) / config.cell_size_m);
    const int column = static_cast<int>((point.y + config.lateral_limit_m) / config.cell_size_m);
    if (row < 0 || row >= rows || column < 0 || column >= columns) {
      continue;
    }
    cells[static_cast<std::size_t>(row * columns + column)].heights.push_back(point.z);
  }
  std::vector<TerrainPoint> near_points;
  for (int row = 0; row < rows; ++row) {
    for (int column = 0; column < columns; ++column) {
      auto& cell = cells[static_cast<std::size_t>(row * columns + column)];
      if (cell.heights.empty()) {
        continue;
      }
      const auto middle = cell.heights.begin() + cell.heights.size() / 2;
      std::nth_element(cell.heights.begin(), middle, cell.heights.end());
      cell.height = *middle;
      cell.observed = true;
      ++result.observed_cells;
      if (config.min_forward_m + (row + 0.5) * config.cell_size_m <= config.min_forward_m + 1.2) {
        near_points.push_back(
            {static_cast<float>(config.min_forward_m + (row + 0.5) * config.cell_size_m),
             static_cast<float>(-config.lateral_limit_m + (column + 0.5) * config.cell_size_m),
             cell.height});
      }
    }
  }
  // A run of missing cells bracketed by measured returns is uncertain ground,
  // regardless of whether the run is one cell or several cells wide.
  for (int column = 0; column < columns; ++column) {
    int first = -1;
    int last = -1;
    for (int row = 0; row < rows; ++row) {
      if (cells[static_cast<std::size_t>(row * columns + column)].observed) {
        if (first < 0) {
          first = row;
        }
        last = row;
      }
    }
    if (first < 0) {
      continue;
    }
    for (int row = first + 1; row < last; ++row) {
      if (!cells[static_cast<std::size_t>(row * columns + column)].observed) {
        ++result.unobserved_ground_cells;
      }
    }
  }
  Plane ground;
  if (!EstimateGround(near_points, config, &ground, &result.ground_inlier_cells)) {
    return result;
  }
  result.ground_valid = true;
  std::vector<bool> raised(cells.size(), false);
  std::vector<bool> hazards(cells.size(), false);
  constexpr double kPi = 3.14159265358979323846;
  const double max_gradient = std::tan(config.max_slope_deg * kPi / 180.0);
  for (int row = 0; row < rows; ++row) {
    for (int column = 0; column < columns; ++column) {
      const auto index = static_cast<std::size_t>(row * columns + column);
      const auto& cell = cells[index];
      if (!cell.observed) {
        continue;
      }
      TerrainPoint location{
          static_cast<float>(config.min_forward_m + (row + 0.5) * config.cell_size_m),
          static_cast<float>(-config.lateral_limit_m + (column + 0.5) * config.cell_size_m),
          cell.height};
      const double residual = cell.height - ground.Height(location);
      raised[index] = residual >= config.rock_height_m;
      hazards[index] = hazards[index] || residual <= -config.drop_depth_m;
      const std::array<std::array<int, 2>, 2> neighbors{{{{row + 1, column}}, {{row, column + 1}}}};
      for (const auto& neighbor : neighbors) {
        if (neighbor[0] >= rows || neighbor[1] >= columns) {
          continue;
        }
        const auto next = static_cast<std::size_t>(neighbor[0] * columns + neighbor[1]);
        if (cells[next].observed &&
            std::abs(cell.height - cells[next].height) / config.cell_size_m > max_gradient) {
          hazards[index] = true;
          hazards[next] = true;
        }
      }
    }
  }
  AppendClusters(raised, cells, rows, columns, config, false, ground, &result.raised_obstacles);
  AppendClusters(hazards, cells, rows, columns, config, true, ground, &result.terrain_hazards);
  return result;
}

} // namespace lb_sensors
