#include "mower_logic/terrain/TerrainMath.h"

namespace mower_logic {
namespace terrain {

namespace {
double wrapAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}
}  // namespace

TerrainProjection projectSlopeToPath(double roll_rad, double pitch_rad, double robot_heading_rad,
                                     double path_heading_rad) {
  const double delta = wrapAngle(path_heading_rad - robot_heading_rad);
  TerrainProjection projection;
  projection.uphill_deg = (pitch_rad * std::cos(delta) + roll_rad * std::sin(delta)) * 180.0 / M_PI;
  projection.cross_deg = (-pitch_rad * std::sin(delta) + roll_rad * std::cos(delta)) * 180.0 / M_PI;
  return projection;
}

double computeSlipScore(const SlipInputs& inputs, const ObserverTuning& tuning) {
  const double score = std::abs(inputs.cross_track_error) * tuning.slip_lat_error_weight +
                       std::max(0.0, inputs.cross_track_error_rate) * tuning.slip_error_rate_weight +
                       std::max(0.0, inputs.speed_error) * tuning.slip_speed_error_weight +
                       std::abs(inputs.cross_slope_deg) * tuning.slip_cross_slope_weight / 10.0;
  return clamp_value(score, 0.0, 1.0);
}

double computeSpeedScale(double uphill_deg, double cross_deg, double slip_score, double risk_ahead,
                         const ObserverTuning& tuning) {
  double scale = 1.0;

  if (uphill_deg < 0.0) {
    scale -= std::abs(uphill_deg) * tuning.speed_scale_downhill_per_deg;
  } else if (uphill_deg > 0.0 && slip_score < tuning.compensation_slip_threshold) {
    scale += uphill_deg * tuning.uphill_speed_bonus_per_deg;
  }

  scale -= std::abs(cross_deg) * tuning.speed_scale_cross_per_deg;
  scale -= clamp_value(risk_ahead, 0.0, 1.0) * 0.35;

  if (slip_score >= tuning.recovery_slip_threshold) {
    scale = std::min(scale, tuning.recovery_speed_scale);
  }

  scale = clamp_value(scale, tuning.speed_scale_min, tuning.speed_scale_max_uphill);
  return scale;
}

double computeHeadingBias(double cross_track_error, double cross_track_error_rate, double cross_slope_deg,
                          double slip_score, const ObserverTuning& tuning) {
  if (slip_score <= 0.0 || std::abs(cross_slope_deg) < 1e-3) {
    return 0.0;
  }

  const double direction = (std::abs(cross_track_error_rate) > 1e-3) ? -std::copysign(1.0, cross_track_error_rate)
                                                                      : -std::copysign(1.0, cross_track_error);
  const double magnitude = std::abs(cross_slope_deg) * tuning.heading_bias_gain * clamp_value(slip_score, 0.0, 1.0);
  return clamp_value(direction * magnitude, -tuning.heading_bias_max_rad, tuning.heading_bias_max_rad);
}

TerrainMode determineMode(bool autonomous_active, double cross_slope_deg, double slip_score, double risk_ahead,
                          const ObserverTuning& tuning) {
  if (!autonomous_active) {
    return TerrainMode::HOLD;
  }
  if (slip_score >= tuning.hold_slip_threshold) {
    return TerrainMode::HOLD;
  }
  if (slip_score >= tuning.recovery_slip_threshold) {
    return TerrainMode::RECOVERY;
  }
  if (std::abs(cross_slope_deg) >= tuning.compensation_cross_slope_deg ||
      slip_score >= tuning.compensation_slip_threshold || risk_ahead >= tuning.compensation_slip_threshold) {
    return TerrainMode::COMPENSATING;
  }
  return TerrainMode::NORMAL;
}

}  // namespace terrain
}  // namespace mower_logic
