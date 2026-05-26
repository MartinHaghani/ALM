export interface RosTime {
  secs: number;
  nsecs: number;
}

export interface RosHeader {
  seq: number;
  stamp: RosTime;
  frame_id: string;
}

export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

export interface Quaternion {
  x: number;
  y: number;
  z: number;
  w: number;
}

export interface NavSatStatus {
  status: number;
  service: number;
}

export interface Pose {
  position: Vector3;
  orientation: Quaternion;
}

export interface PoseWithCovariance {
  pose: Pose;
  covariance: number[];
}

export interface MapMetaData {
  map_load_time: RosTime;
  resolution: number;
  width: number;
  height: number;
  origin: Pose;
}

export interface NavSatFix {
  header: RosHeader;
  status: NavSatStatus;
  latitude: number;
  longitude: number;
  altitude: number;
  position_covariance: number[];
  position_covariance_type: number;
}

export interface AbsolutePose {
  header: RosHeader;
  sensor_stamp: number;
  received_stamp: number;
  source: number;
  flags: number;
  orientation_valid: number;
  motion_vector_valid: number;
  position_accuracy: number;
  orientation_accuracy: number;
  pose: PoseWithCovariance;
  motion_vector: Vector3;
  vehicle_heading: number;
  motion_heading: number;
}

export interface LaserScan {
  header: RosHeader;
  angle_min: number;
  angle_max: number;
  angle_increment: number;
  time_increment: number;
  scan_time: number;
  range_min: number;
  range_max: number;
  ranges: number[];
  intensities: number[];
}

export interface Imu {
  header: RosHeader;
  orientation: Quaternion;
  orientation_covariance: number[];
  angular_velocity: Vector3;
  angular_velocity_covariance: number[];
  linear_acceleration: Vector3;
  linear_acceleration_covariance: number[];
}

export interface OccupancyGrid {
  header: RosHeader;
  info: MapMetaData;
  data: number[];
}

export interface Transform {
  translation: Vector3;
  rotation: Quaternion;
}

export interface TransformStamped {
  header: RosHeader;
  child_frame_id: string;
  transform: Transform;
}

export interface TfMessage {
  transforms: TransformStamped[];
}

export interface StringMessage {
  data: string;
}

export interface SlamManagerStatus {
  base_frame: string;
  last_error: string;
  last_exit_code: number | null;
  last_odom_reset_at?: number | null;
  last_odom_reset_message?: string;
  mapping_enabled: boolean;
  odom_frame: string;
  origin_captured: boolean;
  origin_frame: string;
  publish_origin_transform?: boolean;
  slam_node: string;
  slam_running: boolean;
}

export interface SensorStats {
  hz: number;
  lastMessageAt: number | null;
  messageCount: number;
}

export type ScanStats = SensorStats;
export type ImuStats = SensorStats;
export type GpsFixStats = SensorStats;
export type GpsStatusStats = SensorStats;
export type MapStats = SensorStats;
export type TfStats = SensorStats;
