#pragma once

#include <geometry_msgs/Polygon.h>
#include <nav_msgs/Path.h>

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace mower_logic {
namespace terrain {

struct TerrainSample {
  double roll_deg = 0.0;
  double pitch_deg = 0.0;
  double uphill_deg = 0.0;
  double cross_deg = 0.0;
  double slip_score = 0.0;
  bool recovery = false;
  bool hard_failure = false;
};

struct TerrainCell {
  uint32_t observations = 0;
  double roll_abs_deg = 0.0;
  double pitch_abs_deg = 0.0;
  double uphill_abs_deg = 0.0;
  double cross_abs_deg = 0.0;
  double slip_score = 0.0;
  double risk = 0.0;
  uint32_t recovery_count = 0;
  uint32_t hard_failure_count = 0;
};

class TerrainMemory {
 public:
  explicit TerrainMemory(double cell_size = 0.5);

  bool load(const std::string& path);
  bool save(const std::string& path) const;
  void clear();

  void observe(double x, double y, const TerrainSample& sample);

  double riskAt(double x, double y) const;
  double riskAlongPath(const nav_msgs::Path& path, double x, double y, size_t lookahead_poses) const;
  std::vector<geometry_msgs::Polygon> buildExclusionPolygons(const geometry_msgs::Polygon& outline, double threshold,
                                                             double buffer, size_t max_cells) const;

  double cellSize() const { return cell_size_; }
  size_t cellCount() const { return cells_.size(); }

 private:
  struct CellIndex {
    int x = 0;
    int y = 0;

    bool operator==(const CellIndex& other) const { return x == other.x && y == other.y; }
  };

  struct CellIndexHash {
    size_t operator()(const CellIndex& index) const {
      return std::hash<long long>{}((static_cast<long long>(index.x) << 32) ^ static_cast<unsigned int>(index.y));
    }
  };

  CellIndex toIndex(double x, double y) const;
  std::pair<double, double> toCenter(const CellIndex& index) const;

  double cell_size_;
  std::unordered_map<CellIndex, TerrainCell, CellIndexHash> cells_;
};

}  // namespace terrain
}  // namespace mower_logic
