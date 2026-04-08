#include "mower_logic/terrain/TerrainMemory.h"

#include <geometry_msgs/Point32.h>
#include <nlohmann/json.hpp>

#include <cmath>
#include <deque>
#include <fstream>
#include <limits>
#include <unordered_set>

using json = nlohmann::ordered_json;

namespace mower_logic {
namespace terrain {

namespace {
struct Point {
  double x = 0.0;
  double y = 0.0;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Point, x, y)
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(TerrainCell, observations, roll_abs_deg, pitch_abs_deg, uphill_abs_deg,
                                   cross_abs_deg, slip_score, risk, recovery_count, hard_failure_count)

double clamp01(double value) {
  if (value < 0.0) return 0.0;
  if (value > 1.0) return 1.0;
  return value;
}

bool pointInsidePolygon(const geometry_msgs::Polygon& polygon, double x, double y) {
  if (polygon.points.size() < 3) {
    return false;
  }

  bool inside = false;
  for (size_t i = 0, j = polygon.points.size() - 1; i < polygon.points.size(); j = i++) {
    const auto& pi = polygon.points[i];
    const auto& pj = polygon.points[j];
    const bool intersects = ((pi.y > y) != (pj.y > y)) &&
                            (x < (pj.x - pi.x) * (y - pi.y) / ((pj.y - pi.y) + 1e-9f) + pi.x);
    if (intersects) {
      inside = !inside;
    }
  }

  return inside;
}
}  // namespace

TerrainMemory::TerrainMemory(double cell_size) : cell_size_(cell_size) {}

TerrainMemory::CellIndex TerrainMemory::toIndex(double x, double y) const {
  return CellIndex{static_cast<int>(std::floor(x / cell_size_)), static_cast<int>(std::floor(y / cell_size_))};
}

std::pair<double, double> TerrainMemory::toCenter(const CellIndex& index) const {
  return {((static_cast<double>(index.x) + 0.5) * cell_size_), ((static_cast<double>(index.y) + 0.5) * cell_size_)};
}

bool TerrainMemory::load(const std::string& path) {
  cells_.clear();

  std::ifstream file(path);
  if (!file.is_open()) {
    return false;
  }

  try {
    json j;
    file >> j;
    if (j.contains("cell_size") && j["cell_size"].is_number()) {
      cell_size_ = j["cell_size"].get<double>();
    }
    for (const auto& entry : j.value("cells", json::array())) {
      CellIndex idx;
      idx.x = entry.value("x", 0);
      idx.y = entry.value("y", 0);
      cells_[idx] = entry.value("stats", TerrainCell{});
    }
  } catch (const json::exception&) {
    cells_.clear();
    return false;
  }

  return true;
}

bool TerrainMemory::save(const std::string& path) const {
  json j;
  j["version"] = 1;
  j["cell_size"] = cell_size_;
  j["cells"] = json::array();

  for (const auto& entry : cells_) {
    const CellIndex& index = entry.first;
    const TerrainCell& cell = entry.second;
    j["cells"].push_back({{"x", index.x}, {"y", index.y}, {"stats", cell}});
  }

  std::ofstream file(path);
  if (!file.is_open()) {
    return false;
  }
  file << j.dump(2);
  return true;
}

void TerrainMemory::clear() {
  cells_.clear();
}

void TerrainMemory::observe(double x, double y, const TerrainSample& sample) {
  auto& cell = cells_[toIndex(x, y)];
  cell.observations += 1;

  const double alpha = 0.25;
  const auto update_avg = [alpha](double current, double sample_value) {
    return current == 0.0 ? std::abs(sample_value) : (1.0 - alpha) * current + alpha * std::abs(sample_value);
  };

  cell.roll_abs_deg = update_avg(cell.roll_abs_deg, sample.roll_deg);
  cell.pitch_abs_deg = update_avg(cell.pitch_abs_deg, sample.pitch_deg);
  cell.uphill_abs_deg = update_avg(cell.uphill_abs_deg, sample.uphill_deg);
  cell.cross_abs_deg = update_avg(cell.cross_abs_deg, sample.cross_deg);
  cell.slip_score = cell.slip_score == 0.0 ? sample.slip_score : (1.0 - alpha) * cell.slip_score + alpha * sample.slip_score;

  if (sample.recovery) {
    cell.recovery_count += 1;
  }
  if (sample.hard_failure) {
    cell.hard_failure_count += 1;
  }

  const double severity = clamp01((cell.cross_abs_deg / 18.0) * 0.45 + cell.slip_score * 0.40 +
                                  std::min(1.0, static_cast<double>(cell.recovery_count) / 4.0) * 0.10 +
                                  std::min(1.0, static_cast<double>(cell.hard_failure_count) / 2.0) * 0.25);
  cell.risk = cell.risk == 0.0 ? severity : (0.88 * cell.risk + 0.12 * severity);
}

double TerrainMemory::riskAt(double x, double y) const {
  const auto it = cells_.find(toIndex(x, y));
  if (it == cells_.end()) return 0.0;
  return it->second.risk;
}

double TerrainMemory::riskAlongPath(const nav_msgs::Path& path, double x, double y, size_t lookahead_poses) const {
  if (path.poses.empty() || lookahead_poses == 0) {
    return 0.0;
  }

  size_t nearest_index = 0;
  double nearest_distance = std::numeric_limits<double>::max();
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const double dx = path.poses[i].pose.position.x - x;
    const double dy = path.poses[i].pose.position.y - y;
    const double dist = dx * dx + dy * dy;
    if (dist < nearest_distance) {
      nearest_distance = dist;
      nearest_index = i;
    }
  }

  const size_t end_index = std::min(path.poses.size(), nearest_index + lookahead_poses);
  double total_risk = 0.0;
  size_t samples = 0;
  for (size_t i = nearest_index; i < end_index; ++i) {
    total_risk += riskAt(path.poses[i].pose.position.x, path.poses[i].pose.position.y);
    ++samples;
  }

  return samples == 0 ? 0.0 : total_risk / static_cast<double>(samples);
}

std::vector<geometry_msgs::Polygon> TerrainMemory::buildExclusionPolygons(const geometry_msgs::Polygon& outline,
                                                                          double threshold, double buffer,
                                                                          size_t max_cells) const {
  std::vector<geometry_msgs::Polygon> polygons;
  if (outline.points.empty() || cells_.empty()) {
    return polygons;
  }

  std::vector<CellIndex> risky_cells;
  risky_cells.reserve(std::min(max_cells, cells_.size()));
  for (const auto& entry : cells_) {
    const CellIndex& index = entry.first;
    const TerrainCell& cell = entry.second;
    if (cell.risk < threshold) continue;
    const std::pair<double, double> center = toCenter(index);
    const double cx = center.first;
    const double cy = center.second;
    if (!pointInsidePolygon(outline, cx, cy)) {
      continue;
    }
    risky_cells.push_back(index);
    if (risky_cells.size() >= max_cells) break;
  }

  std::unordered_set<CellIndex, CellIndexHash> remaining(risky_cells.begin(), risky_cells.end());
  while (!remaining.empty()) {
    const CellIndex seed = *remaining.begin();
    remaining.erase(seed);

    int min_ix = seed.x;
    int max_ix = seed.x;
    int min_iy = seed.y;
    int max_iy = seed.y;

    std::deque<CellIndex> queue;
    queue.push_back(seed);

    while (!queue.empty()) {
      const CellIndex current = queue.front();
      queue.pop_front();
      min_ix = std::min(min_ix, current.x);
      max_ix = std::max(max_ix, current.x);
      min_iy = std::min(min_iy, current.y);
      max_iy = std::max(max_iy, current.y);

      const CellIndex neighbors[] = {
          {current.x + 1, current.y}, {current.x - 1, current.y}, {current.x, current.y + 1}, {current.x, current.y - 1}};
      for (const auto& neighbor : neighbors) {
        const auto it = remaining.find(neighbor);
        if (it == remaining.end()) continue;
        queue.push_back(neighbor);
        remaining.erase(it);
      }
    }

    geometry_msgs::Polygon polygon;
    const double left = static_cast<double>(min_ix) * cell_size_ - buffer;
    const double right = static_cast<double>(max_ix + 1) * cell_size_ + buffer;
    const double bottom = static_cast<double>(min_iy) * cell_size_ - buffer;
    const double top = static_cast<double>(max_iy + 1) * cell_size_ + buffer;
    geometry_msgs::Point32 point;

    point.x = left;
    point.y = bottom;
    polygon.points.push_back(point);
    point.x = right;
    point.y = bottom;
    polygon.points.push_back(point);
    point.x = right;
    point.y = top;
    polygon.points.push_back(point);
    point.x = left;
    point.y = top;
    polygon.points.push_back(point);
    point.x = left;
    point.y = bottom;
    polygon.points.push_back(point);
    polygons.push_back(polygon);
  }

  return polygons;
}

}  // namespace terrain
}  // namespace mower_logic
