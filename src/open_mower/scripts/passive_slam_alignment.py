#!/usr/bin/env python3

import json
import math
import threading
import time
from collections import deque

import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped
from mower_map.msg import BoundarySample
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from xbot_msgs.msg import AbsolutePose


FLAG_GPS_RTK_FIXED = 2
DEFAULT_FOOTPRINT = [[0.0, 0.34], [0.82, 0.34], [0.82, -0.34], [0.0, -0.34]]


def clean_frame(frame):
    return str(frame).lstrip("/")


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(rotation):
    quat = [rotation.x, rotation.y, rotation.z, rotation.w]
    return tf.transformations.euler_from_quaternion(quat)[2]


def quaternion_from_yaw(yaw):
    x, y, z, w = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
    return x, y, z, w


def pose_from_transform(transform):
    return {
        "x": transform.transform.translation.x,
        "y": transform.transform.translation.y,
        "yaw": yaw_from_quaternion(transform.transform.rotation),
    }


def stamp_seconds(stamp):
    return None if stamp == rospy.Time() else stamp.to_sec()


def pose_from_pose_msg(pose):
    return {
        "x": pose.position.x,
        "y": pose.position.y,
        "yaw": yaw_from_quaternion(pose.orientation),
    }


def point_from_point_msg(point):
    return {"x": point.x, "y": point.y}


def distance(a, b):
    return math.hypot(a["x"] - b["x"], a["y"] - b["y"])


def compose_planar(transform, pose):
    cos_yaw = math.cos(transform["yaw"])
    sin_yaw = math.sin(transform["yaw"])
    return {
        "x": transform["x"] + cos_yaw * pose["x"] - sin_yaw * pose["y"],
        "y": transform["y"] + sin_yaw * pose["x"] + cos_yaw * pose["y"],
        "yaw": normalize_angle(transform["yaw"] + pose.get("yaw", 0.0)),
    }


def project_point(pose, offset):
    cos_yaw = math.cos(pose.get("yaw", 0.0))
    sin_yaw = math.sin(pose.get("yaw", 0.0))
    return {
        "x": pose["x"] + cos_yaw * offset["x"] - sin_yaw * offset["y"],
        "y": pose["y"] + sin_yaw * offset["x"] + cos_yaw * offset["y"],
    }


def path_length(samples):
    if len(samples) < 2:
        return 0.0

    total = 0.0
    previous = samples[0]["gps"]
    for sample in list(samples)[1:]:
        current = sample["gps"]
        total += distance(previous, current)
        previous = current
    return total


def percentile(values, percent):
    finite = sorted(value for value in values if math.isfinite(value))
    if not finite:
        return None
    if len(finite) == 1:
        return finite[0]
    rank = (len(finite) - 1) * max(0.0, min(100.0, percent)) / 100.0
    lower = int(math.floor(rank))
    upper = int(math.ceil(rank))
    if lower == upper:
        return finite[lower]
    ratio = rank - lower
    return finite[lower] * (1.0 - ratio) + finite[upper] * ratio


def root_mean_square(values):
    finite = [value for value in values if math.isfinite(value)]
    if not finite:
        return None
    return math.sqrt(sum(value * value for value in finite) / len(finite))


def solve_rigid_transform(gps_points, slam_points):
    if len(gps_points) != len(slam_points) or len(gps_points) < 2:
        raise ValueError("at least two matched GPS/SLAM pose pairs are required")

    gps_center = {
        "x": sum(point["x"] for point in gps_points) / len(gps_points),
        "y": sum(point["y"] for point in gps_points) / len(gps_points),
    }
    slam_center = {
        "x": sum(point["x"] for point in slam_points) / len(slam_points),
        "y": sum(point["y"] for point in slam_points) / len(slam_points),
    }

    cross = 0.0
    dot = 0.0
    slam_norm = 0.0
    for gps, slam in zip(gps_points, slam_points):
        sx = slam["x"] - slam_center["x"]
        sy = slam["y"] - slam_center["y"]
        gx = gps["x"] - gps_center["x"]
        gy = gps["y"] - gps_center["y"]
        cross += sx * gy - sy * gx
        dot += sx * gx + sy * gy
        slam_norm += sx * sx + sy * sy

    yaw = math.atan2(cross, dot)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    x = gps_center["x"] - (cos_yaw * slam_center["x"] - sin_yaw * slam_center["y"])
    y = gps_center["y"] - (sin_yaw * slam_center["x"] + cos_yaw * slam_center["y"])
    transform = {"x": x, "y": y, "yaw": normalize_angle(yaw)}

    residuals = residuals_for_transform(transform, gps_points, slam_points)
    residual = math.sqrt(sum(value * value for value in residuals) / len(residuals))
    scale = math.sqrt(cross * cross + dot * dot) / slam_norm if slam_norm > 1e-9 else None
    return transform, residual, residuals, scale


def residuals_for_transform(transform, gps_points, slam_points):
    residuals = []
    for gps, slam in zip(gps_points, slam_points):
        estimate = compose_planar(transform, slam)
        residuals.append(distance(gps, estimate))
    return residuals


def fit_rigid_transform(gps_points, slam_points, max_residual_m, min_samples):
    transform, residual, residuals, scale = solve_rigid_transform(gps_points, slam_points)
    kept_indices = list(range(len(gps_points)))

    if len(gps_points) >= max(8, min_samples + 2):
        median = percentile(residuals, 50.0) or 0.0
        deviations = [abs(value - median) for value in residuals]
        mad = percentile(deviations, 50.0) or 0.0
        robust_sigma = max(0.05, mad * 1.4826)
        limit = max(max_residual_m * 2.0, median + 3.0 * robust_sigma)
        kept_indices = [index for index, value in enumerate(residuals) if value <= limit]

        if len(kept_indices) >= min_samples and len(kept_indices) < len(gps_points):
            gps_kept = [gps_points[index] for index in kept_indices]
            slam_kept = [slam_points[index] for index in kept_indices]
            transform, residual, _, scale = solve_rigid_transform(gps_kept, slam_kept)
            residuals = residuals_for_transform(transform, gps_points, slam_points)

    return transform, residual, residuals, kept_indices, scale


def footprint_geometry():
    footprint = rospy.get_param("/move_base_flex/global_costmap/footprint", None)
    if footprint is None:
        footprint = rospy.get_param("/global_costmap/footprint", None)
    if footprint is None:
        footprint = rospy.get_param("/footprint", DEFAULT_FOOTPRINT)

    points = []
    for entry in footprint:
        try:
            x = float(entry[0])
            y = float(entry[1])
        except (TypeError, ValueError, IndexError):
            continue
        if math.isfinite(x) and math.isfinite(y):
            points.append({"x": x, "y": y})

    if len(points) < 3:
        points = [{"x": x, "y": y} for x, y in DEFAULT_FOOTPRINT]

    min_x = min(point["x"] for point in points)
    max_x = max(point["x"] for point in points)
    min_y = min(point["y"] for point in points)
    max_y = max(point["y"] for point in points)
    center = {"x": (min_x + max_x) / 2.0, "y": (min_y + max_y) / 2.0}
    front_right = {"x": max_x, "y": min_y}
    return center, front_right


class PassiveSlamAlignment:
    def __init__(self):
        rospy.init_node("passive_slam_alignment")

        self.map_frame = clean_frame(rospy.get_param("~map_frame", "map"))
        self.base_frame = clean_frame(rospy.get_param("~base_frame", "base_link"))
        self.slam_map_frame = clean_frame(rospy.get_param("~slam_map_frame", "slam_map"))
        self.slam_base_frame = clean_frame(rospy.get_param("~slam_base_frame", "slam_base_link"))
        self.slam_lidar_frame = clean_frame(rospy.get_param("~slam_lidar_frame", "slam_lidar"))
        self.gps_topic = rospy.get_param("~gps_topic", "/hw/position/gps")
        self.navsat_topic = rospy.get_param("~navsat_topic", "/hw/position/gps/fix")
        self.boundary_sample_topic = rospy.get_param("~boundary_sample_topic", "/area_recorder/boundary_samples")
        self.manager_status_topic = rospy.get_param("~manager_status_topic", "/slam_toolbox_manager/status")
        self.status_topic = rospy.get_param("~status_topic", "/slam_toolbox_alignment/status")
        self.max_gps_accuracy = float(
            rospy.get_param("~max_gps_accuracy", rospy.get_param("/xbot_positioning/max_gps_accuracy", 0.2))
        )
        self.min_travel_m = float(rospy.get_param("~min_travel_m", 2.0))
        self.max_residual_m = float(rospy.get_param("~max_residual_m", 0.35))
        self.min_sample_spacing_m = float(rospy.get_param("~min_sample_spacing_m", 0.15))
        self.min_samples = int(rospy.get_param("~min_samples", 6))
        legacy_max_samples = int(rospy.get_param("~max_samples", 240))
        self.max_motion_samples = int(rospy.get_param("~max_motion_samples", legacy_max_samples))
        self.max_boundary_samples = int(rospy.get_param("~max_boundary_samples", 5000))
        self.max_status_boundary_pairs = int(rospy.get_param("~max_status_boundary_pairs", 600))
        self.tf_timeout = float(rospy.get_param("~tf_timeout", 0.1))
        self.tf_max_age = float(rospy.get_param("~tf_max_age", 1.0))
        self.tf_buffer_duration = float(rospy.get_param("~tf_buffer_duration", 900.0))
        self.pose_sync_max_lag = float(rospy.get_param("~pose_sync_max_lag", 1.5))
        self.gps_max_age = float(rospy.get_param("~gps_max_age", 2.0))
        self.manager_max_age = float(rospy.get_param("~manager_max_age", 2.0))
        self.status_period = float(rospy.get_param("~status_period", 0.5))
        self.rate_hz = float(rospy.get_param("~rate_hz", 10.0))
        self.drift_p95_factor = float(rospy.get_param("~drift_p95_factor", 1.75))
        self.drift_max_factor = float(rospy.get_param("~drift_max_factor", 2.5))

        self.footprint_center, self.footprint_front_right = footprint_geometry()
        self.slam_record_offset = dict(self.footprint_front_right)

        self.lock = threading.Lock()
        self.samples = deque(maxlen=max(2, self.max_motion_samples))
        self.boundary_samples = deque(maxlen=max(2, self.max_boundary_samples))
        self.last_error = ""
        self.state = "stopped"
        self.transform = None
        self.residual_m = None
        self.residual_p95_m = None
        self.residual_max_m = None
        self.residual_all_p95_m = None
        self.residual_all_max_m = None
        self.outlier_residual_max_m = None
        self.scale_diagnostic = None
        self.drift_warning = False
        self.outlier_warning = False
        self.outlier_count = 0
        self.alignment_hold_reason = ""
        self.boundary_sample_received_count = 0
        self.boundary_sample_skip_counts = {
            "low_gps_quality": 0,
            "not_rtk": 0,
            "spacing": 0,
            "tf_missing": 0,
        }
        self.alignment_source = "none"
        self.current_gps_pose = None
        self.current_slam_pose = None
        self.current_lidar_pose = None
        self.current_gps_record_point = None
        self.current_lidar_record_point = None
        self.pose_sync_age = None
        self.pose_sync_lag = None
        self.mapping_enabled = False
        self.slam_running = False
        self.last_manager_wall_time = None
        self.last_gps_wall_time = None
        self.last_fix_wall_time = None
        self.last_gps = None
        self.last_fix = None
        self.last_reset_at = time.time()
        self.tf_health = {"gps": "missing", "slam": "missing", "boundary": "missing"}

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(self.tf_buffer_duration))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=1, latch=True)

        rospy.Subscriber(self.gps_topic, AbsolutePose, self.on_gps, queue_size=10)
        rospy.Subscriber(self.navsat_topic, NavSatFix, self.on_fix, queue_size=10)
        rospy.Subscriber(self.boundary_sample_topic, BoundarySample, self.on_boundary_sample, queue_size=max(1000, self.max_boundary_samples))
        rospy.Subscriber(self.manager_status_topic, String, self.on_manager_status, queue_size=10)
        rospy.Service("~reset", Trigger, self.reset)

        rospy.loginfo(
            "Passive SLAM alignment ready: boundary topic %s, publishing %s -> %s. Boundary record offset from %s is (%.3f, %.3f).",
            self.boundary_sample_topic,
            self.map_frame,
            self.slam_map_frame,
            self.slam_base_frame,
            self.slam_record_offset["x"],
            self.slam_record_offset["y"],
        )

    def run(self):
        rate = rospy.Rate(max(1.0, self.rate_hz))
        last_status = 0.0
        while not rospy.is_shutdown():
            self.update_alignment()
            self.publish_transform()

            now = time.time()
            if now - last_status >= self.status_period:
                self.publish_status()
                last_status = now

            rate.sleep()

    def on_gps(self, msg):
        with self.lock:
            self.last_gps = msg
            self.last_gps_wall_time = time.time()

    def on_fix(self, msg):
        with self.lock:
            self.last_fix = msg
            self.last_fix_wall_time = time.time()

    def on_manager_status(self, msg):
        try:
            status = json.loads(msg.data)
        except (TypeError, ValueError) as exc:
            with self.lock:
                self.last_error = "Could not parse manager status: {}".format(exc)
            return

        with self.lock:
            was_mapping = self.mapping_enabled
            self.mapping_enabled = bool(status.get("mapping_enabled", False))
            self.slam_running = bool(status.get("slam_running", False))
            self.last_manager_wall_time = time.time()
            if self.mapping_enabled and not was_mapping:
                self.clear_alignment_locked("mapping started")

    def on_boundary_sample(self, msg):
        if msg.area_type != BoundarySample.AREA_MOW or msg.point_mode != BoundarySample.POINT_FRONT_RIGHT:
            return
        with self.lock:
            self.boundary_sample_received_count += 1
        if not msg.rtk_fixed:
            self.set_tf_health("boundary", "not_rtk")
            self.increment_boundary_skip("not_rtk")
            return
        if not math.isfinite(msg.gps_accuracy) or msg.gps_accuracy > self.max_gps_accuracy:
            self.set_tf_health("boundary", "low_gps_quality")
            self.increment_boundary_skip("low_gps_quality")
            return

        try:
            stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time(0)
            slam_base_pose = self.lookup_pose(self.slam_map_frame, self.slam_base_frame, stamp=stamp, allow_historical=True)
            slam_record_point = project_point(slam_base_pose, self.slam_record_offset)
            self.set_tf_health("boundary", "ok")
        except Exception as exc:  # pylint: disable=broad-except
            self.set_tf_health("boundary", "missing")
            self.increment_boundary_skip("tf_missing")
            with self.lock:
                self.last_error = "boundary sample skipped: {}".format(exc)
            return

        gps_record_point = point_from_point_msg(msg.gps_point)
        gps_pose = pose_from_pose_msg(msg.fused_pose)
        boundary_sample = {
            "gps": gps_record_point,
            "gps_pose": gps_pose,
            "point_index": int(msg.point_index),
            "slam": slam_record_point,
            "slam_pose": slam_base_pose,
            "stamp": msg.header.stamp.to_sec() if msg.header.stamp != rospy.Time() else time.time(),
        }

        self.add_boundary_sample(boundary_sample)

    def reset(self, _request):
        with self.lock:
            self.clear_alignment_locked("manual reset")
        self.publish_status()
        return TriggerResponse(success=True, message="Passive SLAM alignment reset")

    def clear_alignment_locked(self, reason):
        self.samples.clear()
        self.boundary_samples.clear()
        self.transform = None
        self.residual_m = None
        self.residual_p95_m = None
        self.residual_max_m = None
        self.residual_all_p95_m = None
        self.residual_all_max_m = None
        self.outlier_residual_max_m = None
        self.scale_diagnostic = None
        self.drift_warning = False
        self.outlier_warning = False
        self.outlier_count = 0
        self.alignment_hold_reason = ""
        self.boundary_sample_received_count = 0
        for key in self.boundary_sample_skip_counts:
            self.boundary_sample_skip_counts[key] = 0
        self.current_lidar_pose = None
        self.current_gps_record_point = None
        self.current_lidar_record_point = None
        self.alignment_source = "none"
        self.state = "collecting"
        self.last_error = ""
        self.last_reset_at = time.time()
        rospy.loginfo("Passive SLAM alignment cleared: %s.", reason)

    def gps_is_rtk_fixed(self):
        with self.lock:
            gps = self.last_gps
            fix = self.last_fix
            gps_age = None if self.last_gps_wall_time is None else time.time() - self.last_gps_wall_time
            fix_age = None if self.last_fix_wall_time is None else time.time() - self.last_fix_wall_time

        if gps is None or gps_age is None or gps_age > self.gps_max_age:
            return False
        if fix is not None and fix_age is not None and fix_age <= self.gps_max_age and fix.status.status < 0:
            return False
        if (gps.flags & FLAG_GPS_RTK_FIXED) == 0:
            return False
        if not math.isfinite(gps.position_accuracy) or gps.position_accuracy > self.max_gps_accuracy:
            return False
        return True

    def lookup_pose(self, parent_frame, child_frame, stamp=None, allow_historical=False):
        transform = self.lookup_transform_msg(parent_frame, child_frame, stamp=stamp, allow_historical=allow_historical)
        return pose_from_transform(transform)

    def lookup_pose_stamped(self, parent_frame, child_frame, stamp=None, allow_historical=False):
        transform = self.lookup_transform_msg(parent_frame, child_frame, stamp=stamp, allow_historical=allow_historical)
        return pose_from_transform(transform), stamp_seconds(transform.header.stamp)

    def lookup_transform_msg(self, parent_frame, child_frame, stamp=None, allow_historical=False):
        query_time = rospy.Time(0) if stamp is None else stamp
        transform = self.tf_buffer.lookup_transform(
            parent_frame,
            child_frame,
            query_time,
            rospy.Duration(self.tf_timeout),
        )
        if not allow_historical and transform.header.stamp != rospy.Time():
            age = (rospy.Time.now() - transform.header.stamp).to_sec()
            if age > self.tf_max_age:
                raise RuntimeError("{} -> {} TF is stale ({:.2f} s)".format(parent_frame, child_frame, age))
        return transform

    def lookup_synced_live_poses(self):
        gps_pose, gps_stamp = self.lookup_pose_stamped(self.map_frame, self.base_frame)
        slam_pose, slam_stamp = self.lookup_pose_stamped(self.slam_map_frame, self.slam_base_frame)

        self.pose_sync_age = None
        self.pose_sync_lag = None
        if gps_stamp is None or slam_stamp is None:
            return gps_pose, slam_pose

        self.pose_sync_lag = abs(gps_stamp - slam_stamp)
        common_stamp = min(gps_stamp, slam_stamp)
        self.pose_sync_age = max(0.0, rospy.Time.now().to_sec() - common_stamp)

        if self.pose_sync_lag > self.pose_sync_max_lag:
            return gps_pose, slam_pose

        stamp = rospy.Time.from_sec(common_stamp)
        synced_gps_pose = self.lookup_pose(self.map_frame, self.base_frame, stamp=stamp, allow_historical=True)
        synced_slam_pose = self.lookup_pose(self.slam_map_frame, self.slam_base_frame, stamp=stamp, allow_historical=True)
        return synced_gps_pose, synced_slam_pose

    def add_sample(self, gps_pose, slam_pose):
        with self.lock:
            if self.samples:
                previous = self.samples[-1]
                if (
                    distance(previous["gps"], gps_pose) < self.min_sample_spacing_m
                    and distance(previous["slam"], slam_pose) < self.min_sample_spacing_m
                ):
                    return

            self.samples.append({
                "gps": gps_pose,
                "slam": slam_pose,
                "stamp": time.time(),
            })

    def add_boundary_sample(self, sample):
        with self.lock:
            if self.boundary_samples:
                previous = self.boundary_samples[-1]
                if (
                    distance(previous["gps"], sample["gps"]) < self.min_sample_spacing_m
                    and distance(previous["slam"], sample["slam"]) < self.min_sample_spacing_m
                ):
                    self.boundary_sample_skip_counts["spacing"] = self.boundary_sample_skip_counts.get("spacing", 0) + 1
                    return
            self.boundary_samples.append(sample)
            self.current_gps_record_point = dict(sample["gps"])
            self.current_lidar_record_point = dict(sample["slam"])

    def update_alignment(self):
        manager_fresh = self.manager_is_fresh()
        with self.lock:
            mapping_enabled = self.mapping_enabled
            last_transform = self.transform
            boundary_snapshot = list(self.boundary_samples)

        if not manager_fresh:
            self.set_state("waiting_for_manager")
            return
        if not mapping_enabled:
            self.set_state("stopped")
            return

        try:
            gps_pose, slam_pose = self.lookup_synced_live_poses()
            self.set_tf_health("gps", "ok")
        except Exception as exc:  # pylint: disable=broad-except
            self.set_tf_health("gps", "missing")
            self.set_error_state("waiting_for_tf", str(exc))
            return

        try:
            if slam_pose is None:
                slam_pose = self.lookup_pose(self.slam_map_frame, self.slam_base_frame)
            self.set_tf_health("slam", "ok")
        except Exception as exc:  # pylint: disable=broad-except
            self.set_tf_health("slam", "missing")
            self.set_error_state("waiting_for_tf", str(exc))
            return

        with self.lock:
            self.current_gps_pose = gps_pose
            self.current_slam_pose = slam_pose

        if self.boundary_ready(boundary_snapshot):
            self.solve_from_samples(boundary_snapshot, "boundary", slam_pose)
            return

        if not self.gps_is_rtk_fixed():
            if last_transform:
                with self.lock:
                    self.current_lidar_pose = compose_planar(last_transform, slam_pose)
                    self.alignment_hold_reason = "gps_not_rtk_fixed"
                    self.state = "aligned_rtk_hold"
                    self.last_error = ""
                return
            self.set_state("waiting_for_rtk")
            with self.lock:
                self.alignment_hold_reason = ""
                self.alignment_source = "boundary_collecting" if boundary_snapshot else "none"
            return

        self.add_sample(gps_pose, slam_pose)
        with self.lock:
            sample_snapshot = list(self.samples)

        if not self.samples_ready(sample_snapshot):
            self.set_state("collecting")
            with self.lock:
                self.alignment_hold_reason = ""
                self.alignment_source = "boundary_collecting" if boundary_snapshot else "motion_collecting"
            return

        self.solve_from_samples(sample_snapshot, "motion", slam_pose)

    def boundary_ready(self, samples):
        return len(samples) >= self.min_samples and path_length(samples) >= self.min_travel_m

    def samples_ready(self, samples):
        return len(samples) >= self.min_samples and path_length(samples) >= self.min_travel_m

    def solve_from_samples(self, samples, source, slam_pose):
        gps_points = [sample["gps"] for sample in samples]
        slam_points = [sample["slam"] for sample in samples]
        try:
            transform, residual, residuals, kept_indices, scale = fit_rigid_transform(
                gps_points,
                slam_points,
                self.max_residual_m,
                self.min_samples,
            )
        except Exception as exc:  # pylint: disable=broad-except
            self.set_error_state("degraded", str(exc))
            return

        kept_index_set = set(kept_indices)
        kept_residuals = [residuals[index] for index in kept_indices if 0 <= index < len(residuals)]
        rejected_residuals = [value for index, value in enumerate(residuals) if index not in kept_index_set]
        residual = root_mean_square(kept_residuals) if kept_residuals else residual
        residual_p95 = percentile(kept_residuals, 95.0)
        residual_max = max(kept_residuals) if kept_residuals else None
        residual_all_p95 = percentile(residuals, 95.0)
        residual_all_max = max(residuals) if residuals else None
        outlier_residual_max = max(rejected_residuals) if rejected_residuals else None
        drift_warning = bool(
            residual_p95 is not None
            and residual_max is not None
            and (
                residual_p95 > self.max_residual_m * self.drift_p95_factor
                or residual_max > self.max_residual_m * self.drift_max_factor
            )
        )
        outlier_warning = bool(
            outlier_residual_max is not None
            and outlier_residual_max > self.max_residual_m * self.drift_max_factor
            and not drift_warning
        )

        with self.lock:
            self.residual_m = residual
            self.residual_p95_m = residual_p95
            self.residual_max_m = residual_max
            self.residual_all_p95_m = residual_all_p95
            self.residual_all_max_m = residual_all_max
            self.outlier_residual_max_m = outlier_residual_max
            self.scale_diagnostic = scale
            self.drift_warning = drift_warning
            self.outlier_warning = outlier_warning
            self.outlier_count = max(0, len(samples) - len(kept_indices))
            self.alignment_source = source
            self.alignment_hold_reason = ""

            if residual <= self.max_residual_m:
                self.transform = transform
                self.current_lidar_pose = compose_planar(transform, slam_pose)
                if source == "boundary" and samples:
                    self.current_gps_record_point = dict(samples[-1]["gps"])
                    self.current_lidar_record_point = compose_planar(transform, samples[-1]["slam"])
                if drift_warning:
                    self.state = "degraded_slam_drift"
                    self.last_error = "boundary residual spread suggests SLAM map drift or deformation"
                else:
                    self.state = "aligned"
                    self.last_error = ""
            else:
                if self.transform:
                    self.current_lidar_pose = compose_planar(self.transform, slam_pose)
                self.state = "degraded_slam_drift" if drift_warning or source == "boundary" else "degraded"
                self.last_error = "alignment residual {:.3f} m exceeds {:.3f} m".format(
                    residual,
                    self.max_residual_m,
                )

    def manager_is_fresh(self):
        with self.lock:
            last_manager_wall_time = self.last_manager_wall_time
        return last_manager_wall_time is not None and time.time() - last_manager_wall_time <= self.manager_max_age

    def set_tf_health(self, key, value):
        with self.lock:
            self.tf_health[key] = value

    def increment_boundary_skip(self, key):
        with self.lock:
            self.boundary_sample_skip_counts[key] = self.boundary_sample_skip_counts.get(key, 0) + 1

    def set_state(self, state):
        with self.lock:
            self.state = state
            if state not in ("degraded", "degraded_slam_drift", "waiting_for_tf"):
                self.last_error = ""

    def set_error_state(self, state, error):
        with self.lock:
            self.state = state
            self.last_error = error

    def publish_transform(self):
        with self.lock:
            transform = None if self.transform is None else dict(self.transform)

        if transform is None:
            return

        msg = TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.slam_map_frame
        msg.transform.translation.x = transform["x"]
        msg.transform.translation.y = transform["y"]
        msg.transform.translation.z = 0.0
        x, y, z, w = quaternion_from_yaw(transform["yaw"])
        msg.transform.rotation.x = x
        msg.transform.rotation.y = y
        msg.transform.rotation.z = z
        msg.transform.rotation.w = w
        self.tf_broadcaster.sendTransform(msg)

    def boundary_pairs_for_status(self, transform):
        if transform is None:
            return []
        with self.lock:
            samples = list(self.boundary_samples)
        pairs = []
        if len(samples) > self.max_status_boundary_pairs:
            stride = int(math.ceil(float(len(samples)) / float(max(1, self.max_status_boundary_pairs))))
            selected = samples[::stride]
            if selected[-1] is not samples[-1]:
                selected.append(samples[-1])
        else:
            selected = samples

        for sample in selected:
            lidar_map_point = compose_planar(transform, sample["slam"])
            pairs.append({
                "gps": {"x": sample["gps"]["x"], "y": sample["gps"]["y"]},
                "lidar": {"x": lidar_map_point["x"], "y": lidar_map_point["y"]},
                "residual_m": distance(sample["gps"], lidar_map_point),
            })
        return pairs

    def status_payload(self):
        with self.lock:
            transform = None if self.transform is None else dict(self.transform)
            gps_pose = None if self.current_gps_pose is None else dict(self.current_gps_pose)
            slam_pose = None if self.current_slam_pose is None else dict(self.current_slam_pose)
            lidar_pose = None if self.current_lidar_pose is None else dict(self.current_lidar_pose)
            samples = list(self.samples)
            boundary_samples = list(self.boundary_samples)
            state = self.state
            residual_m = self.residual_m
            residual_p95_m = self.residual_p95_m
            residual_max_m = self.residual_max_m
            residual_all_p95_m = self.residual_all_p95_m
            residual_all_max_m = self.residual_all_max_m
            outlier_residual_max_m = self.outlier_residual_max_m
            scale_diagnostic = self.scale_diagnostic
            drift_warning = self.drift_warning
            outlier_warning = self.outlier_warning
            outlier_count = self.outlier_count
            alignment_hold_reason = self.alignment_hold_reason
            boundary_sample_received_count = self.boundary_sample_received_count
            boundary_sample_skip_counts = dict(self.boundary_sample_skip_counts)
            alignment_source = self.alignment_source
            gps_record_point = None if self.current_gps_record_point is None else dict(self.current_gps_record_point)
            lidar_record_point = None if self.current_lidar_record_point is None else dict(self.current_lidar_record_point)
            mapping_enabled = self.mapping_enabled
            slam_running = self.slam_running
            last_error = self.last_error
            tf_health = dict(self.tf_health)
            last_gps_wall_time = self.last_gps_wall_time
            last_fix_wall_time = self.last_fix_wall_time
            last_manager_wall_time = self.last_manager_wall_time
            gps_accuracy = None if self.last_gps is None else self.last_gps.position_accuracy
            pose_sync_age = self.pose_sync_age
            pose_sync_lag = self.pose_sync_lag

        if transform and slam_pose:
            lidar_pose = compose_planar(transform, slam_pose)

        separation_m = None
        if gps_pose and lidar_pose:
            separation_m = distance(gps_pose, lidar_pose)

        now = time.time()
        return {
            "aligned": transform is not None and state in ("aligned", "aligned_rtk_hold"),
            "alignment_frozen": bool(alignment_hold_reason),
            "alignment_hold_reason": alignment_hold_reason,
            "alignment_source": alignment_source,
            "base_frame": self.base_frame,
            "boundary_pairs": self.boundary_pairs_for_status(transform),
            "boundary_path_length_m": path_length(boundary_samples),
            "boundary_sample_count": len(boundary_samples),
            "boundary_sample_received_count": boundary_sample_received_count,
            "boundary_sample_skip_counts": boundary_sample_skip_counts,
            "drift_warning": drift_warning,
            "gps_accuracy_m": gps_accuracy,
            "gps_age": None if last_gps_wall_time is None else now - last_gps_wall_time,
            "gps_pose": gps_pose,
            "gps_record_point": gps_record_point,
            "last_error": last_error,
            "last_reset_at": self.last_reset_at,
            "lidar_pose": lidar_pose,
            "lidar_record_point": lidar_record_point,
            "manager_age": None if last_manager_wall_time is None else now - last_manager_wall_time,
            "map_frame": self.map_frame,
            "mapping_enabled": mapping_enabled,
            "max_residual_m": self.max_residual_m,
            "min_travel_m": self.min_travel_m,
            "navsat_age": None if last_fix_wall_time is None else now - last_fix_wall_time,
            "outlier_count": outlier_count,
            "outlier_residual_max_m": outlier_residual_max_m,
            "outlier_warning": outlier_warning,
            "path_length_m": path_length(samples),
            "pose_sync_age": pose_sync_age,
            "pose_sync_lag": pose_sync_lag,
            "residual_m": residual_m,
            "residual_all_max_m": residual_all_max_m,
            "residual_all_p95_m": residual_all_p95_m,
            "residual_max_m": residual_max_m,
            "residual_p95_m": residual_p95_m,
            "rtk_fixed": self.gps_is_rtk_fixed(),
            "sample_count": len(samples),
            "scale_diagnostic": scale_diagnostic,
            "separation_m": separation_m,
            "slam_base_frame": self.slam_base_frame,
            "slam_lidar_frame": self.slam_lidar_frame,
            "slam_map_frame": self.slam_map_frame,
            "slam_footprint_center": dict(self.footprint_center),
            "slam_record_offset": dict(self.slam_record_offset),
            "slam_pose": slam_pose,
            "slam_running": slam_running,
            "state": state,
            "tf_health": tf_health,
            "transform": transform,
            "yaw_offset_rad": None if transform is None else transform["yaw"],
        }

    def publish_status(self):
        msg = String()
        msg.data = json.dumps(self.status_payload(), sort_keys=True)
        self.status_pub.publish(msg)


if __name__ == "__main__":
    PassiveSlamAlignment().run()
