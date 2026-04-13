#ifndef FTC_LOCAL_PLANNER_TERRAIN_CONTROL_H_
#define FTC_LOCAL_PLANNER_TERRAIN_CONTROL_H_

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace ftc_local_planner {

struct TerrainControlInput {
  bool enabled = false;
  bool fresh = false;
  uint8_t mode = 0;
  double slip_score = 0.0;
  double speed_scale = 1.0;
  double heading_bias = 0.0;
  double risk_ahead = 0.0;
  double cross_track_error = 0.0;
};

struct TerrainControlConfig {
  double speed_scale_min = 0.25;
  double speed_scale_max = 1.20;
  double heading_bias_scale = 1.0;
  double heading_bias_max_rad = 0.20;
  double lat_gain_scale = 1.0;
  double ang_gain_scale = 0.8;
  double lookahead_scale_min = 0.20;
  double risk_speed_penalty = 0.40;
  double progress_freeze_slip_threshold = 0.45;
  double recovery_release_slip_threshold = 0.25;
  double recovery_release_lat_error = 0.10;
};

struct TerrainCommandModifiers {
  double speed_scale = 1.0;
  double lat_gain_scale = 1.0;
  double ang_gain_scale = 1.0;
  double heading_bias = 0.0;
  double lookahead_scale = 1.0;
  bool freeze_progress = false;
  bool hold_position = false;
  bool recovery_requested = false;
};

template <typename T>
inline T clamp_value(T value, T lo, T hi) {
  return std::max(lo, std::min(value, hi));
}

TerrainCommandModifiers computeTerrainCommandModifiers(const TerrainControlInput& input,
                                                       const TerrainControlConfig& config);

bool shouldReleaseTerrainRecovery(const TerrainControlInput& input, const TerrainControlConfig& config);

}  // namespace ftc_local_planner

#endif
