# A state to display to a human to tell what's going on
float32 battery_percentage
bool emergency
bool is_charging
bool rain_detected
float32 gps_percentage
float32 current_action_progress
string current_state                # Like "Cleaning"
string current_sub_state            # Like "doing the dishes"
int16 current_area
int16 current_path
int16 current_path_index
uint8 terrain_mode
float32 terrain_roll_deg
float32 terrain_pitch_deg
float32 terrain_uphill_slope_deg
float32 terrain_cross_slope_deg
float32 terrain_slip_score
float32 terrain_speed_scale
float32 terrain_heading_bias
float32 terrain_risk_ahead
float32 terrain_memory_risk
uint16 terrain_recovery_count
xbot_msgs/AbsolutePose robot_pose # Pose of the docking station
