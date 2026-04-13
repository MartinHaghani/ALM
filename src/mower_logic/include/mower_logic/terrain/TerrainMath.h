#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace mower_logic {
namespace terrain {

enum class TerrainMode : uint8_t {
  NORMAL = 0,
  COMPENSATING = 1,
  RECOVERY = 2,
  HOLD = 3,
};

struct TerrainProjection {
  double uphill_deg = 0.0;
  double cross_deg = 0.0;
};

struct SlipInputs {
  double cross_track_error = 0.0;
  double cross_track_error_rate = 0.0;
  double speed_error = 0.0;
  double cross_slope_deg = 0.0;
};

struct ObserverTuning {
  double slip_lat_error_weight = 0.70;
  double slip_error_rate_weight = 0.45;
  double slip_speed_error_weight = 0.35;
  double slip_cross_slope_weight = 0.25;
  double compensation_slip_threshold = 0.28;
  double recovery_slip_threshold = 0.58;
  double hold_slip_threshold = 0.82;
  double compensation_cross_slope_deg = 4.0;
  double speed_scale_min = 0.35;
  double recovery_speed_scale = 0.0;
  double speed_scale_downhill_per_deg = 0.025;
  double speed_scale_cross_per_deg = 0.05;
  double uphill_speed_bonus_per_deg = 0.010;
  double speed_scale_max_uphill = 1.10;
  double heading_bias_gain = 0.04;
  double heading_bias_max_rad = 0.20;
};

template <typename T>
inline T clamp_value(T value, T lo, T hi) {
  return std::max(lo, std::min(value, hi));
}

TerrainProjection projectSlopeToPath(double roll_rad, double pitch_rad, double robot_heading_rad,
                                     double path_heading_rad);

double computeSlipScore(const SlipInputs& inputs, const ObserverTuning& tuning);

double computeSpeedScale(double uphill_deg, double cross_deg, double slip_score, double risk_ahead,
                         const ObserverTuning& tuning);

double computeHeadingBias(double cross_track_error, double cross_track_error_rate, double cross_slope_deg,
                          double slip_score, const ObserverTuning& tuning);

TerrainMode determineMode(bool autonomous_active, double cross_slope_deg, double slip_score, double risk_ahead,
                          const ObserverTuning& tuning);

}  // namespace terrain
}  // namespace mower_logic
