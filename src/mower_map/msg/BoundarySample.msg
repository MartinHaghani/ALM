uint8 AREA_MOW=1
uint8 AREA_NAV=2
uint8 AREA_OBSTACLE=3

uint8 POINT_BASE=0
uint8 POINT_FRONT_RIGHT=1
uint8 POINT_FRONT_LEFT=2

std_msgs/Header header
uint8 area_type
uint8 point_mode
uint32 point_index
geometry_msgs/Pose fused_pose
geometry_msgs/Point32 gps_point
uint16 gps_flags
float32 gps_accuracy
bool rtk_fixed
bool auto_collected
