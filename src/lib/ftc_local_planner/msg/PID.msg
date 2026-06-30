time stamp

# P set point
float32 kp_lon_set
float32 kp_lat_set
float32 kp_ang_set

# I set point
float32 ki_lon_set
float32 ki_lat_set
float32 ki_ang_set

# D set point
float32 kd_lon_set
float32 kd_lat_set
float32 kd_ang_set

# errors
float32 lon_err
float32 lat_err
float32 ang_err

# speeds
float32 ang_speed
float32 lin_speed

# planner state
uint8 planner_state
string planner_state_name
uint32 current_index
float32 current_progress
float32 movement_speed
bool is_crashed
float32 control_distance
float32 control_yaw_error
string failure_reason
