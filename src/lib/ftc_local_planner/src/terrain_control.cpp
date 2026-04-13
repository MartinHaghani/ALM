#include "ftc_local_planner/terrain_control.h"

namespace ftc_local_planner {

TerrainCommandModifiers computeTerrainCommandModifiers(const TerrainControlInput& input,
                                                       const TerrainControlConfig& config) {
  TerrainCommandModifiers modifiers;
  if (!input.enabled || !input.fresh) {
    return modifiers;
  }

  modifiers.speed_scale =
      clamp_value(input.speed_scale * (1.0 - config.risk_speed_penalty * clamp_value(input.risk_ahead, 0.0, 1.0)),
                  config.speed_scale_min, config.speed_scale_max);
  modifiers.heading_bias =
      clamp_value(input.heading_bias * config.heading_bias_scale, -config.heading_bias_max_rad, config.heading_bias_max_rad);
  modifiers.lat_gain_scale = 1.0 + config.lat_gain_scale * clamp_value(input.slip_score, 0.0, 1.0);
  modifiers.ang_gain_scale = 1.0 + config.ang_gain_scale * clamp_value(input.slip_score, 0.0, 1.0);
  modifiers.lookahead_scale =
      clamp_value(modifiers.speed_scale, config.lookahead_scale_min, 1.0);

  if (input.mode >= 2 || input.slip_score >= config.progress_freeze_slip_threshold) {
    modifiers.freeze_progress = true;
    modifiers.recovery_requested = true;
  }
  if (input.mode >= 3) {
    modifiers.hold_position = true;
    modifiers.speed_scale = config.speed_scale_min;
  }

  return modifiers;
}

bool shouldReleaseTerrainRecovery(const TerrainControlInput& input, const TerrainControlConfig& config) {
  return input.fresh && input.slip_score <= config.recovery_release_slip_threshold &&
         std::abs(input.cross_track_error) <= config.recovery_release_lat_error;
}

}  // namespace ftc_local_planner
