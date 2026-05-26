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

export interface MapPoint {
  x: number;
  y: number;
}

export interface MowerMapArea {
  id: string;
  outline: MapPoint[];
  properties?: {
    active?: boolean;
    name?: string;
    type?: "draft" | "mow" | "nav" | "obstacle" | string;
  };
}

export interface MowerDockingStation {
  heading: number;
  id: string;
  position: MapPoint;
  properties?: {
    active?: boolean;
    name?: string;
  };
}

export interface MowerMapData {
  areas: MowerMapArea[];
  docking_stations: MowerDockingStation[];
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

export interface SlamAlignmentPose {
  x: number;
  y: number;
  yaw: number;
}

export interface SlamAlignmentPoint {
  x: number;
  y: number;
}

export interface SlamAlignmentBoundaryPair {
  gps: SlamAlignmentPoint;
  lidar: SlamAlignmentPoint;
  residual_m: number;
}

export interface SlamAlignmentStatus {
  aligned: boolean;
  alignment_source?: string;
  base_frame: string;
  boundary_pairs?: SlamAlignmentBoundaryPair[];
  boundary_path_length_m?: number;
  boundary_sample_count?: number;
  drift_warning?: boolean;
  gps_accuracy_m: number | null;
  gps_age: number | null;
  gps_pose: SlamAlignmentPose | null;
  gps_record_point?: SlamAlignmentPoint | null;
  last_error: string;
  lidar_pose: SlamAlignmentPose | null;
  lidar_record_point?: SlamAlignmentPoint | null;
  map_frame: string;
  mapping_enabled: boolean;
  max_residual_m: number;
  min_travel_m: number;
  outlier_count?: number;
  path_length_m: number;
  pose_sync_age?: number | null;
  pose_sync_lag?: number | null;
  residual_m: number | null;
  residual_max_m?: number | null;
  residual_p95_m?: number | null;
  rtk_fixed: boolean;
  sample_count: number;
  scale_diagnostic?: number | null;
  separation_m: number | null;
  slam_base_frame: string;
  slam_footprint_center?: SlamAlignmentPoint;
  slam_lidar_frame?: string;
  slam_map_frame: string;
  slam_record_offset?: SlamAlignmentPoint;
  slam_pose: SlamAlignmentPose | null;
  slam_running: boolean;
  state: string;
  tf_health?: Record<string, string>;
  transform: SlamAlignmentPose | null;
  yaw_offset_rad?: number | null;
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
