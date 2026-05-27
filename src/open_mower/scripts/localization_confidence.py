#!/usr/bin/env python3

import json
import math
import threading
import time
from collections import deque

import numpy as np
import rospy
import tf
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, NavSatFix
from std_msgs.msg import String
from xbot_msgs.msg import AbsolutePose


FLAG_GPS_RTK_FIXED = 2
FLAG_GPS_RTK_FLOAT = 4
FLAG_GPS_DEAD_RECKONING = 8


def clean_frame(frame):
    return str(frame).lstrip("/")


def clamp(value, low=0.0, high=1.0):
    if value is None or not math.isfinite(value):
        return low
    return max(low, min(high, value))


def finite_float(value, default=None):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def ramp_high(value, poor, good):
    if value is None or not math.isfinite(value):
        return 0.0
    if good <= poor:
        return 1.0 if value >= good else 0.0
    return clamp((value - poor) / (good - poor))


def ramp_low(value, good, poor):
    if value is None or not math.isfinite(value):
        return 0.0
    if poor <= good:
        return 1.0 if value <= good else 0.0
    return clamp((poor - value) / (poor - good))


def weighted_geomean(components, weights):
    total_weight = 0.0
    log_sum = 0.0
    for key, weight in weights.items():
        value = clamp(components.get(key, 0.0), 0.001, 1.0)
        log_sum += weight * math.log(value)
        total_weight += weight
    if total_weight <= 0.0:
        return 0.0
    return clamp(math.exp(log_sum / total_weight))


def percentile(values, percent):
    finite = sorted(value for value in values if math.isfinite(value))
    if not finite:
        return None
    if len(finite) == 1:
        return finite[0]
    rank = (len(finite) - 1) * clamp(percent, 0.0, 100.0) / 100.0
    lower = int(math.floor(rank))
    upper = int(math.ceil(rank))
    if lower == upper:
        return finite[lower]
    ratio = rank - lower
    return finite[lower] * (1.0 - ratio) + finite[upper] * ratio


def yaw_from_quaternion(rotation):
    return tf.transformations.euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])[2]


def pose_from_transform(transform):
    return {
        "x": transform.transform.translation.x,
        "y": transform.transform.translation.y,
        "yaw": yaw_from_quaternion(transform.transform.rotation),
        "stamp": None if transform.header.stamp == rospy.Time() else transform.header.stamp.to_sec(),
    }


def pose_from_absolute_pose(msg):
    return {
        "x": msg.pose.pose.position.x,
        "y": msg.pose.pose.position.y,
        "yaw": yaw_from_quaternion(msg.pose.pose.orientation),
    }


def apply_transform(transform, point):
    yaw = transform["yaw"]
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return {
        "x": transform["x"] + cos_yaw * point["x"] - sin_yaw * point["y"],
        "y": transform["y"] + sin_yaw * point["x"] + cos_yaw * point["y"],
    }


def distance(a, b):
    return math.hypot(a["x"] - b["x"], a["y"] - b["y"])


def vector_length(x, y):
    return math.hypot(finite_float(x, 0.0), finite_float(y, 0.0))


class GridCache:
    def __init__(self, msg):
        self.width = int(msg.info.width)
        self.height = int(msg.info.height)
        self.resolution = float(msg.info.resolution)
        self.origin_x = float(msg.info.origin.position.x)
        self.origin_y = float(msg.info.origin.position.y)
        self.origin_yaw = yaw_from_quaternion(msg.info.origin.orientation)
        self.cos_yaw = math.cos(self.origin_yaw)
        self.sin_yaw = math.sin(self.origin_yaw)
        self.wall_time = time.time()
        self.stamp = None if msg.header.stamp == rospy.Time() else msg.header.stamp.to_sec()

        if self.width <= 0 or self.height <= 0 or self.resolution <= 0.0:
            self.data = np.zeros((0, 0), dtype=np.int16)
            self.occupied = np.zeros((0, 0), dtype=bool)
            self.occupied_count = 0
            return

        try:
            self.data = np.asarray(msg.data, dtype=np.int16).reshape((self.height, self.width))
        except ValueError:
            self.data = np.zeros((0, 0), dtype=np.int16)
            self.occupied = np.zeros((0, 0), dtype=bool)
            self.occupied_count = 0
            return
        self.occupied = self.data >= 50
        self.occupied_count = int(np.count_nonzero(self.occupied))

    def world_to_grid(self, x, y):
        dx = x - self.origin_x
        dy = y - self.origin_y
        gx = (self.cos_yaw * dx + self.sin_yaw * dy) / self.resolution
        gy = (-self.sin_yaw * dx + self.cos_yaw * dy) / self.resolution
        return int(math.floor(gx)), int(math.floor(gy))

    def grid_to_world(self, gx, gy):
        lx = (float(gx) + 0.5) * self.resolution
        ly = (float(gy) + 0.5) * self.resolution
        return {
            "x": self.origin_x + self.cos_yaw * lx - self.sin_yaw * ly,
            "y": self.origin_y + self.sin_yaw * lx + self.cos_yaw * ly,
        }

    def in_bounds(self, gx, gy):
        return 0 <= gx < self.width and 0 <= gy < self.height

    def is_occupied_world(self, x, y):
        gx, gy = self.world_to_grid(x, y)
        return self.in_bounds(gx, gy) and bool(self.occupied[gy, gx])

    def nearest_occupied(self, point, radius_m):
        if self.occupied_count <= 0:
            return None
        gx, gy = self.world_to_grid(point["x"], point["y"])
        radius_cells = max(1, int(math.ceil(radius_m / self.resolution)))
        best = None
        best_dist = None
        for iy in range(max(0, gy - radius_cells), min(self.height, gy + radius_cells + 1)):
            for ix in range(max(0, gx - radius_cells), min(self.width, gx + radius_cells + 1)):
                if not self.occupied[iy, ix]:
                    continue
                cell_point = self.grid_to_world(ix, iy)
                dist_m = distance(point, cell_point)
                if dist_m <= radius_m and (best_dist is None or dist_m < best_dist):
                    best = {"grid": (ix, iy), "point": cell_point, "distance": dist_m}
                    best_dist = dist_m
        return best

    def normal_at_match(self, endpoint, match):
        ix, iy = match["grid"]
        points = []
        for y in range(max(0, iy - 2), min(self.height, iy + 3)):
            for x in range(max(0, ix - 2), min(self.width, ix + 3)):
                if self.occupied[y, x]:
                    point = self.grid_to_world(x, y)
                    points.append([point["x"], point["y"]])

        if len(points) >= 3:
            coords = np.asarray(points, dtype=float)
            centered = coords - coords.mean(axis=0)
            covariance = centered.T @ centered
            try:
                values, vectors = np.linalg.eigh(covariance)
                normal = vectors[:, int(np.argmin(values))]
                norm = float(np.linalg.norm(normal))
                if norm > 1e-6:
                    return {"x": float(normal[0] / norm), "y": float(normal[1] / norm)}
            except np.linalg.LinAlgError:
                pass

        dx = endpoint["x"] - match["point"]["x"]
        dy = endpoint["y"] - match["point"]["y"]
        norm = math.hypot(dx, dy)
        if norm > 1e-6:
            return {"x": dx / norm, "y": dy / norm}
        return {"x": 1.0, "y": 0.0}


class LocalizationConfidence:
    def __init__(self):
        rospy.init_node("localization_confidence")

        self.status_topic = rospy.get_param("~status_topic", "/localization_confidence/status")
        self.gps_topic = rospy.get_param("~gps_topic", "/hw/position/gps")
        self.gps_fix_topic = rospy.get_param("~gps_fix_topic", "/hw/position/gps/fix")
        self.gps_quality_topic = rospy.get_param("~gps_quality_topic", "/hw/position/gps/quality")
        self.fused_pose_topic = rospy.get_param("~fused_pose_topic", "/xbot_positioning/xb_pose")
        self.scan_topic = rospy.get_param("~scan_topic", "/slam_toolbox/scan")
        self.map_topic = rospy.get_param("~map_topic", "/slam_toolbox/map")
        self.passive_odom_status_topic = rospy.get_param("~passive_odom_status_topic", "/passive_slam_odom/status")
        self.alignment_status_topic = rospy.get_param("~alignment_status_topic", "/slam_toolbox_alignment/status")
        self.slam_map_frame = clean_frame(rospy.get_param("~slam_map_frame", "slam_map"))
        self.rate_hz = float(rospy.get_param("~rate_hz", 2.0))
        self.max_topic_age = float(rospy.get_param("~max_topic_age_sec", 2.0))
        self.scan_max_points = int(rospy.get_param("~scan_max_points", 180))
        self.scan_match_near_m = float(rospy.get_param("~scan_match_near_m", 0.15))
        self.scan_match_far_m = float(rospy.get_param("~scan_match_far_m", 0.35))
        self.tf_timeout = float(rospy.get_param("~tf_timeout", 0.1))
        self.tf_max_age = float(rospy.get_param("~tf_max_age", 1.0))
        self.min_motion_heading_speed_mps = float(rospy.get_param("~min_motion_heading_speed_mps", 0.25))

        self.lock = threading.Lock()
        self.gps_pose = None
        self.previous_gps_pose = None
        self.gps_wall_time = None
        self.previous_gps_wall_time = None
        self.gps_fix = None
        self.gps_fix_wall_time = None
        self.gps_quality = None
        self.gps_quality_wall_time = None
        self.gps_consistency_score = 0.8
        self.gps_innovation_m = None
        self.gps_innovation_sigma_m = None
        self.gps_consistency_window_s = None
        self.gps_short_consistency_score = 0.8
        self.gps_short_innovation_m = None
        self.gps_short_innovation_sigma_m = None
        self.gps_short_consistency_window_s = None
        self.gps_long_consistency_score = 0.8
        self.gps_long_innovation_m = None
        self.gps_long_innovation_sigma_m = None
        self.gps_long_consistency_window_s = None
        self.gps_history = deque(maxlen=80)
        self.last_quality_counters = None
        self.last_quality_counter_time = None
        self.parser_rates = {
            "skipped_bytes_per_sec": 0.0,
            "invalid_checksums_per_sec": 0.0,
            "fix_warnings_per_sec": 0.0,
            "timing_warnings_per_sec": 0.0,
        }

        self.fused_pose = None
        self.fused_pose_wall_time = None
        self.scan = None
        self.scan_wall_time = None
        self.scan_arrivals = []
        self.grid = None
        self.passive_odom_status = None
        self.passive_odom_wall_time = None
        self.alignment_status = None
        self.alignment_wall_time = None

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(20.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=1, latch=True)

        rospy.Subscriber(self.gps_topic, AbsolutePose, self.on_gps, queue_size=20)
        rospy.Subscriber(self.gps_fix_topic, NavSatFix, self.on_gps_fix, queue_size=10)
        rospy.Subscriber(self.gps_quality_topic, String, self.on_gps_quality, queue_size=10)
        rospy.Subscriber(self.fused_pose_topic, AbsolutePose, self.on_fused_pose, queue_size=10)
        rospy.Subscriber(self.scan_topic, LaserScan, self.on_scan, queue_size=2)
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.on_map, queue_size=1)
        rospy.Subscriber(self.passive_odom_status_topic, String, self.on_passive_odom_status, queue_size=2)
        rospy.Subscriber(self.alignment_status_topic, String, self.on_alignment_status, queue_size=2)

        rospy.loginfo("Localization confidence ready: publishing %s.", self.status_topic)

    def run(self):
        rate = rospy.Rate(max(0.2, self.rate_hz))
        while not rospy.is_shutdown():
            self.publish_status()
            rate.sleep()

    def on_gps(self, msg):
        with self.lock:
            now = time.time()
            if self.gps_pose is not None and self.gps_wall_time is not None:
                self.previous_gps_pose = self.gps_pose
                self.previous_gps_wall_time = self.gps_wall_time
            self.gps_pose = msg
            self.gps_wall_time = now
            self.update_gps_self_consistency_locked()
            self.gps_history.append(self.gps_sample_from_pose_locked(msg, now))

    def on_gps_fix(self, msg):
        with self.lock:
            self.gps_fix = msg
            self.gps_fix_wall_time = time.time()

    def on_gps_quality(self, msg):
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            return

        with self.lock:
            now = time.time()
            self.update_parser_rates_locked(payload, now)
            self.gps_quality = payload
            self.gps_quality_wall_time = now

    def on_fused_pose(self, msg):
        with self.lock:
            self.fused_pose = msg
            self.fused_pose_wall_time = time.time()

    def on_scan(self, msg):
        with self.lock:
            now = time.time()
            self.scan = msg
            self.scan_wall_time = now
            self.scan_arrivals = [stamp for stamp in self.scan_arrivals if now - stamp <= 5.0]
            self.scan_arrivals.append(now)

    def on_map(self, msg):
        with self.lock:
            self.grid = GridCache(msg)

    def on_passive_odom_status(self, msg):
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            payload = {"parse_error": True}
        with self.lock:
            self.passive_odom_status = payload
            self.passive_odom_wall_time = time.time()

    def on_alignment_status(self, msg):
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            payload = {"parse_error": True}
        with self.lock:
            self.alignment_status = payload
            self.alignment_wall_time = time.time()

    def update_parser_rates_locked(self, payload, now):
        counters = {
            "skipped": float(payload.get("parser_skipped_bytes") or 0.0),
            "checksum": float(payload.get("parser_invalid_checksums") or 0.0),
            "fix": float(payload.get("parser_fix_warnings") or 0.0),
            "timing": float(payload.get("parser_timing_warnings") or 0.0),
        }
        if self.last_quality_counters is not None and self.last_quality_counter_time is not None:
            dt = now - self.last_quality_counter_time
            if dt < 1.0:
                return
            if any(counters[key] < self.last_quality_counters[key] for key in counters):
                self.last_quality_counters = counters
                self.last_quality_counter_time = now
                return
            self.parser_rates = {
                "skipped_bytes_per_sec": max(0.0, counters["skipped"] - self.last_quality_counters["skipped"]) / dt,
                "invalid_checksums_per_sec": max(0.0, counters["checksum"] - self.last_quality_counters["checksum"]) / dt,
                "fix_warnings_per_sec": max(0.0, counters["fix"] - self.last_quality_counters["fix"]) / dt,
                "timing_warnings_per_sec": max(0.0, counters["timing"] - self.last_quality_counters["timing"]) / dt,
            }
        self.last_quality_counters = counters
        self.last_quality_counter_time = now

    def gps_sample_from_pose_locked(self, msg, wall_time):
        pose = pose_from_absolute_pose(msg)
        quality = self.gps_quality or {}
        speed_acc = finite_float(quality.get("speed_acc_mps"))
        time_acc_ns = finite_float(quality.get("time_acc_ns"))
        position_accuracy = finite_float(msg.position_accuracy, 5.0)
        return {
            "accuracy": position_accuracy,
            "stamp": msg.header.stamp.to_sec() if msg.header.stamp != rospy.Time() else wall_time,
            "time_acc_s": (time_acc_ns * 1e-9) if time_acc_ns is not None else 0.0,
            "vx": finite_float(msg.motion_vector.x, 0.0),
            "vy": finite_float(msg.motion_vector.y, 0.0),
            "speed_acc_mps": speed_acc,
            "wall_time": wall_time,
            "x": pose["x"],
            "y": pose["y"],
        }

    def gps_consistency_pair(self, previous_sample, current_sample, absolute_good, absolute_poor, normalized_good, normalized_poor):
        dt = current_sample["stamp"] - previous_sample["stamp"]
        if dt <= 0.05 or dt > 5.0:
            return None

        predicted = {
            "x": previous_sample["x"] + 0.5 * (previous_sample["vx"] + current_sample["vx"]) * dt,
            "y": previous_sample["y"] + 0.5 * (previous_sample["vy"] + current_sample["vy"]) * dt,
        }
        current_point = {"x": current_sample["x"], "y": current_sample["y"]}
        innovation = distance(current_point, predicted)
        speed = 0.5 * (
            vector_length(previous_sample["vx"], previous_sample["vy"]) + vector_length(current_sample["vx"], current_sample["vy"])
        )
        speed_acc_candidates = [
            value
            for value in (previous_sample["speed_acc_mps"], current_sample["speed_acc_mps"])
            if value is not None and math.isfinite(value)
        ]
        speed_acc = max(speed_acc_candidates) if speed_acc_candidates else max(0.05, 0.08 * speed)
        time_uncertainty = max(previous_sample["time_acc_s"], current_sample["time_acc_s"], 0.05)
        sigma = math.sqrt(
            max(0.03, previous_sample["accuracy"]) ** 2
            + max(0.03, current_sample["accuracy"]) ** 2
            + (max(0.05, speed_acc) * dt) ** 2
            + (max(0.04, 0.12 * speed) * dt) ** 2
            + (speed * time_uncertainty) ** 2
        )
        normalized = innovation / max(0.03, sigma)
        score = min(ramp_low(normalized, normalized_good, normalized_poor), ramp_low(innovation, absolute_good, absolute_poor))
        return {
            "score": score,
            "innovation_m": innovation,
            "sigma_m": sigma,
            "window_s": dt,
            "normalized": normalized,
        }

    def reset_gps_consistency_locked(self):
        self.gps_consistency_score = 0.8
        self.gps_innovation_m = None
        self.gps_innovation_sigma_m = None
        self.gps_consistency_window_s = None
        self.gps_short_consistency_score = 0.8
        self.gps_short_innovation_m = None
        self.gps_short_innovation_sigma_m = None
        self.gps_short_consistency_window_s = None
        self.gps_long_consistency_score = 0.8
        self.gps_long_innovation_m = None
        self.gps_long_innovation_sigma_m = None
        self.gps_long_consistency_window_s = None

    def pick_gps_history_sample(self, current_sample, min_dt, max_dt, target_dt):
        candidates = []
        for sample in self.gps_history:
            dt = current_sample["stamp"] - sample["stamp"]
            if min_dt <= dt <= max_dt:
                candidates.append((abs(dt - target_dt), sample))
        if not candidates:
            return None
        return min(candidates, key=lambda entry: entry[0])[1]

    def update_gps_self_consistency_locked(self):
        if self.gps_pose is None:
            self.reset_gps_consistency_locked()
            return

        current = self.gps_pose
        current_sample = self.gps_sample_from_pose_locked(current, self.gps_wall_time or time.time())

        short_sample = self.pick_gps_history_sample(current_sample, 0.08, 0.45, 0.20)
        if short_sample is None and self.previous_gps_pose is not None and self.previous_gps_wall_time is not None:
            previous_sample = self.gps_sample_from_pose_locked(self.previous_gps_pose, self.previous_gps_wall_time)
            dt = current_sample["stamp"] - previous_sample["stamp"]
            if 0.05 <= dt <= 0.60:
                short_sample = previous_sample

        long_sample = self.pick_gps_history_sample(current_sample, 0.70, 2.50, 1.25)
        short_result = (
            self.gps_consistency_pair(short_sample, current_sample, 0.10, 0.45, 2.5, 9.0)
            if short_sample is not None
            else None
        )
        long_result = (
            self.gps_consistency_pair(long_sample, current_sample, 0.18, 0.80, 2.0, 7.0)
            if long_sample is not None
            else None
        )

        if short_result is None and long_result is None:
            self.reset_gps_consistency_locked()
            return

        if short_result is not None:
            self.gps_short_consistency_score = short_result["score"]
            self.gps_short_innovation_m = short_result["innovation_m"]
            self.gps_short_innovation_sigma_m = short_result["sigma_m"]
            self.gps_short_consistency_window_s = short_result["window_s"]
        else:
            self.gps_short_consistency_score = 0.8
            self.gps_short_innovation_m = None
            self.gps_short_innovation_sigma_m = None
            self.gps_short_consistency_window_s = None

        if long_result is not None:
            self.gps_long_consistency_score = long_result["score"]
            self.gps_long_innovation_m = long_result["innovation_m"]
            self.gps_long_innovation_sigma_m = long_result["sigma_m"]
            self.gps_long_consistency_window_s = long_result["window_s"]
        else:
            self.gps_long_consistency_score = 0.8
            self.gps_long_innovation_m = None
            self.gps_long_innovation_sigma_m = None
            self.gps_long_consistency_window_s = None

        available = [result for result in (short_result, long_result) if result is not None]
        limiting = min(available, key=lambda result: result["score"])
        self.gps_consistency_score = limiting["score"]
        self.gps_innovation_m = limiting["innovation_m"]
        self.gps_innovation_sigma_m = limiting["sigma_m"]
        self.gps_consistency_window_s = limiting["window_s"]

    def latest_snapshot(self):
        with self.lock:
            return {
                "gps_pose": self.gps_pose,
                "gps_wall_time": self.gps_wall_time,
                "gps_fix": self.gps_fix,
                "gps_fix_wall_time": self.gps_fix_wall_time,
                "gps_quality": None if self.gps_quality is None else dict(self.gps_quality),
                "gps_quality_wall_time": self.gps_quality_wall_time,
                "gps_consistency_score": self.gps_consistency_score,
                "gps_innovation_m": self.gps_innovation_m,
                "gps_innovation_sigma_m": self.gps_innovation_sigma_m,
                "gps_consistency_window_s": self.gps_consistency_window_s,
                "gps_short_consistency_score": self.gps_short_consistency_score,
                "gps_short_innovation_m": self.gps_short_innovation_m,
                "gps_short_innovation_sigma_m": self.gps_short_innovation_sigma_m,
                "gps_short_consistency_window_s": self.gps_short_consistency_window_s,
                "gps_long_consistency_score": self.gps_long_consistency_score,
                "gps_long_innovation_m": self.gps_long_innovation_m,
                "gps_long_innovation_sigma_m": self.gps_long_innovation_sigma_m,
                "gps_long_consistency_window_s": self.gps_long_consistency_window_s,
                "parser_rates": dict(self.parser_rates),
                "fused_pose": self.fused_pose,
                "fused_pose_wall_time": self.fused_pose_wall_time,
                "scan": self.scan,
                "scan_wall_time": self.scan_wall_time,
                "scan_arrivals": list(self.scan_arrivals),
                "grid": self.grid,
                "passive_odom_status": None if self.passive_odom_status is None else dict(self.passive_odom_status),
                "passive_odom_wall_time": self.passive_odom_wall_time,
                "alignment_status": None if self.alignment_status is None else dict(self.alignment_status),
                "alignment_wall_time": self.alignment_wall_time,
            }

    def gps_confidence(self, snapshot, now):
        pose = snapshot["gps_pose"]
        fix = snapshot["gps_fix"]
        quality = snapshot["gps_quality"]
        pose_age = None if snapshot["gps_wall_time"] is None else now - snapshot["gps_wall_time"]
        fix_age = None if snapshot["gps_fix_wall_time"] is None else now - snapshot["gps_fix_wall_time"]
        quality_age = None if snapshot["gps_quality_wall_time"] is None else now - snapshot["gps_quality_wall_time"]
        reasons = []

        raw = {
            "pose_age_s": pose_age,
            "fix_age_s": fix_age,
            "quality_age_s": quality_age,
            "parser_rates": snapshot["parser_rates"],
            "self_innovation_m": snapshot["gps_innovation_m"],
            "self_innovation_sigma_m": snapshot["gps_innovation_sigma_m"],
            "self_consistency_window_s": snapshot["gps_consistency_window_s"],
            "self_consistency_score": snapshot["gps_consistency_score"],
            "short_self_consistency": {
                "score": snapshot["gps_short_consistency_score"],
                "innovation_m": snapshot["gps_short_innovation_m"],
                "innovation_sigma_m": snapshot["gps_short_innovation_sigma_m"],
                "window_s": snapshot["gps_short_consistency_window_s"],
            },
            "long_self_consistency": {
                "score": snapshot["gps_long_consistency_score"],
                "innovation_m": snapshot["gps_long_innovation_m"],
                "innovation_sigma_m": snapshot["gps_long_innovation_sigma_m"],
                "window_s": snapshot["gps_long_consistency_window_s"],
            },
        }

        if pose is None or pose_age is None or pose_age > self.max_topic_age:
            reasons.append("gps_pose_stale_or_missing")
            return self.empty_gps_result("unavailable", raw, reasons)
        if fix is None or fix_age is None or fix_age > self.max_topic_age or fix.status.status < 0:
            reasons.append("navsat_fix_invalid_or_stale")
            return self.empty_gps_result("no_fix", raw, reasons)
        if quality is None or quality_age is None or quality_age > self.max_topic_age:
            reasons.append("gps_quality_stale_or_missing")
            return self.empty_gps_result("quality_missing", raw, reasons)

        flags = int(pose.flags)
        position_accuracy = float(pose.position_accuracy) if math.isfinite(pose.position_accuracy) else None
        h_acc = quality.get("h_acc_m")
        if h_acc is None:
            h_acc = position_accuracy

        rtk_type = quality.get("rtk_type", "none")
        carrier_phase = quality.get("carrier_phase", "none")
        fix_type = quality.get("fix_type", "no_fix")
        dead_reckoning = bool(flags & FLAG_GPS_DEAD_RECKONING) or fix_type == "dead_reckoning_only"

        if flags & FLAG_GPS_RTK_FIXED:
            cap = 1.0
        elif flags & FLAG_GPS_RTK_FLOAT:
            cap = 0.55
            reasons.append("rtk_float_cap")
        elif quality.get("diff_solution") or rtk_type != "none":
            cap = 0.35
            reasons.append("differential_no_fixed_carrier")
        elif fix_type in ("2d", "3d"):
            cap = 0.20
            reasons.append("plain_gnss_cap")
        elif dead_reckoning:
            cap = 0.10
            reasons.append("gps_dead_reckoning_cap")
        else:
            reasons.append("no_gps_fix")
            return self.empty_gps_result("no_fix", raw, reasons)

        if dead_reckoning:
            cap = min(cap, 0.10)

        telemetry_supported = bool(quality.get("telemetry_supported", False))
        accuracy_score = ramp_low(float(h_acc), 0.03, 0.20) if h_acc is not None else 0.0
        if carrier_phase == "fixed":
            carrier_score = 1.0
        elif carrier_phase == "float":
            carrier_score = 0.55
        elif quality.get("diff_solution"):
            carrier_score = 0.35
        else:
            carrier_score = 0.15

        pdop_value = quality.get("pdop")
        pdop_score = ramp_low(float(pdop_value), 1.5, 4.0) if pdop_value is not None else (0.55 if not telemetry_supported else 0.0)
        num_sv = quality.get("num_sv")
        satellite_score = ramp_high(float(num_sv), 8.0, 18.0) if num_sv is not None and num_sv > 0 else (0.55 if not telemetry_supported else 0.0)

        fresh_score = min(
            ramp_low(pose_age, 0.2, self.max_topic_age),
            ramp_low(fix_age, 0.2, self.max_topic_age),
            ramp_low(quality_age, 0.2, self.max_topic_age),
        )
        rtcm_age = quality.get("rtcm_age_s")
        if cap >= 0.55 and rtcm_age is not None:
            fresh_score = min(fresh_score, ramp_low(float(rtcm_age), 1.5, 10.0))
            if rtcm_age > 10.0:
                reasons.append("rtcm_stale")

        parser_rates = snapshot["parser_rates"]
        parser_penalty = min(
            1.0,
            parser_rates.get("invalid_checksums_per_sec", 0.0) / 2.0
            + parser_rates.get("timing_warnings_per_sec", 0.0) / 50.0
            + parser_rates.get("fix_warnings_per_sec", 0.0) / 2.0
            + parser_rates.get("skipped_bytes_per_sec", 0.0) / 500000.0,
        )
        parser_score = clamp(1.0 - parser_penalty)

        components = {
            "accuracy": accuracy_score,
            "carrier_correction": carrier_score,
            "pdop": pdop_score,
            "satellites": satellite_score,
            "freshness_latency": fresh_score,
            "self_consistency": snapshot["gps_consistency_score"],
            "parser_health": parser_score,
        }
        confidence = cap * weighted_geomean(
            components,
            {
                "accuracy": 0.35,
                "carrier_correction": 0.25,
                "pdop": 0.10,
                "satellites": 0.10,
                "freshness_latency": 0.10,
                "self_consistency": 0.07,
                "parser_health": 0.03,
            },
        )
        consistency_score = snapshot["gps_consistency_score"]
        if snapshot["gps_short_consistency_window_s"] is not None and snapshot["gps_short_consistency_score"] < 0.65:
            reasons.append("gps_short_motion_inconsistency")
        if snapshot["gps_long_consistency_window_s"] is not None and snapshot["gps_long_consistency_score"] < 0.65:
            reasons.append("gps_long_motion_inconsistency")
        if snapshot["gps_consistency_window_s"] is not None and consistency_score < 0.80:
            confidence = min(confidence, 0.35 + 0.65 * consistency_score)
        if not telemetry_supported:
            reasons.append("gps_quality_partial_nmea")
            confidence = min(confidence, 0.45)

        speed = vector_length(pose.motion_vector.x, pose.motion_vector.y)
        head_acc = quality.get("head_acc_rad")
        if bool(pose.orientation_valid) and head_acc is not None and math.isfinite(float(head_acc)):
            heading_confidence = ramp_low(float(head_acc), 0.03, 0.35) * confidence
            yaw_sigma = max(0.02, float(head_acc)) / math.sqrt(max(0.15, heading_confidence))
        elif bool(pose.motion_vector_valid) and speed >= self.min_motion_heading_speed_mps:
            speed_score = ramp_high(speed, self.min_motion_heading_speed_mps, 1.0)
            heading_confidence = 0.65 * speed_score * confidence
            yaw_sigma = max(0.12, 0.4 / max(0.2, speed)) / math.sqrt(max(0.15, heading_confidence))
            reasons.append("using_motion_heading")
        else:
            heading_confidence = 0.05 * confidence
            yaw_sigma = math.pi
            reasons.append("heading_weak_or_unavailable")

        receiver_sigma = max(0.01, finite_float(h_acc, finite_float(position_accuracy, 5.0)))
        consistency_sigma_floor = 0.0
        for prefix in ("short", "long"):
            innovation = snapshot["gps_%s_innovation_m" % prefix]
            innovation_sigma = snapshot["gps_%s_innovation_sigma_m" % prefix]
            score = snapshot["gps_%s_consistency_score" % prefix]
            if innovation is None or innovation_sigma is None:
                continue
            excess = max(0.0, innovation - 2.0 * max(0.03, innovation_sigma))
            score_floor = innovation * max(0.0, 1.0 - score)
            consistency_sigma_floor = max(consistency_sigma_floor, excess, score_floor)
        speed_acc = finite_float(quality.get("speed_acc_mps"), 0.0)
        motion_sigma_floor = max(0.0, 0.04 * speed, 0.25 * speed_acc)
        position_sigma_base = max(receiver_sigma, consistency_sigma_floor, motion_sigma_floor)
        position_sigma = position_sigma_base / math.sqrt(max(0.10, confidence))
        position_confidence = min(confidence, ramp_low(position_sigma, 0.06, 0.45))
        if consistency_sigma_floor > receiver_sigma:
            reasons.append("gps_position_sigma_inflated_by_motion_check")
        if position_sigma > 0.35:
            reasons.append("gps_position_sigma_high")

        state = "good" if position_confidence >= 0.75 else "degraded" if position_confidence >= 0.35 else "poor"
        raw.update(
            {
                "flags": flags,
                "fix_type": fix_type,
                "rtk_type": rtk_type,
                "carrier_phase": carrier_phase,
                "h_acc_m": h_acc,
                "pdop": pdop_value,
                "num_sv": num_sv,
                "rtcm_age_s": rtcm_age,
                "speed_mps": speed,
                "speed_acc_mps": quality.get("speed_acc_mps"),
                "time_acc_ns": quality.get("time_acc_ns"),
                "position_sigma_base_m": position_sigma_base,
                "position_sigma_receiver_m": receiver_sigma,
                "position_sigma_consistency_floor_m": consistency_sigma_floor,
                "position_sigma_motion_floor_m": motion_sigma_floor,
            }
        )
        return {
            "confidence": confidence,
            "position_confidence": position_confidence,
            "heading_confidence": clamp(heading_confidence),
            "position_sigma_m": min(10.0, position_sigma),
            "yaw_sigma_rad": min(math.pi, yaw_sigma),
            "state": state,
            "components": components,
            "raw": raw,
            "reasons": reasons,
        }

    def empty_gps_result(self, state, raw, reasons):
        return {
            "confidence": 0.0,
            "position_confidence": 0.0,
            "heading_confidence": 0.0,
            "position_sigma_m": None,
            "yaw_sigma_rad": None,
            "state": state,
            "components": {},
            "raw": raw,
            "reasons": reasons,
        }

    def lookup_pose(self, parent_frame, child_frame, stamp=None):
        query_time = rospy.Time(0) if stamp is None else stamp
        transform = self.tf_buffer.lookup_transform(parent_frame, child_frame, query_time, rospy.Duration(self.tf_timeout))
        pose = pose_from_transform(transform)
        if transform.header.stamp != rospy.Time():
            pose["age"] = max(0.0, rospy.Time.now().to_sec() - transform.header.stamp.to_sec())
        else:
            pose["age"] = 0.0
        return pose

    def selected_scan_points(self, scan):
        points = []
        if scan is None:
            return points
        ranges = list(scan.ranges)
        if not ranges:
            return points
        stride = max(1, int(math.ceil(float(len(ranges)) / float(max(1, self.scan_max_points)))))
        max_range = min(scan.range_max, 18.0)
        for index in range(0, len(ranges), stride):
            range_m = ranges[index]
            if not math.isfinite(range_m) or range_m < scan.range_min or range_m > max_range:
                continue
            angle = scan.angle_min + index * scan.angle_increment
            points.append({"x": math.cos(angle) * range_m, "y": math.sin(angle) * range_m, "range": range_m})
        return points

    def scan_motion_status(self, scan, passive_status):
        passive_status = passive_status or {}
        scan_duration = float(getattr(scan, "scan_time", 0.0) or 0.0)
        if scan_duration <= 0.0:
            scan_duration = float(getattr(scan, "time_increment", 0.0) or 0.0) * float(len(getattr(scan, "ranges", []) or []))
        if scan_duration <= 0.0:
            scan_duration = 0.10

        try:
            yaw_rate = float(passive_status.get("yaw_rate") or 0.0)
        except (TypeError, ValueError):
            yaw_rate = 0.0
        try:
            vx = float(passive_status.get("vx") or 0.0)
        except (TypeError, ValueError):
            vx = 0.0

        yaw_sweep = abs(yaw_rate) * scan_duration
        translation_sweep = abs(vx) * scan_duration
        yaw_score = ramp_low(yaw_sweep, 0.035, 0.14)
        translation_score = ramp_low(translation_sweep, 0.04, 0.20)
        return {
            "component": min(yaw_score, translation_score),
            "scan_duration_s": scan_duration,
            "translation_sweep_m": translation_sweep,
            "vx_mps": vx,
            "yaw_rate_radps": yaw_rate,
            "yaw_sweep_rad": yaw_sweep,
        }

    def scan_pose_correction(self, jacobians, signed_errors):
        if len(jacobians) < 8 or len(jacobians) != len(signed_errors):
            return None
        try:
            matrix = np.asarray(jacobians, dtype=float)
            errors = np.asarray(signed_errors, dtype=float)
            delta, _, _, _ = np.linalg.lstsq(matrix, -errors, rcond=None)
            dx = float(delta[0])
            dy = float(delta[1])
            yaw = float(delta[2])
            return {
                "translation_m": math.hypot(dx, dy),
                "x_m": dx,
                "y_m": dy,
                "yaw_rad": yaw,
            }
        except Exception:  # pylint: disable=broad-except
            return None

    def lidar_confidence(self, snapshot, alignment, now):
        scan = snapshot["scan"]
        grid = snapshot["grid"]
        passive_status = snapshot["passive_odom_status"] or {}
        scan_age = None if snapshot["scan_wall_time"] is None else now - snapshot["scan_wall_time"]
        passive_odom_age = None if snapshot["passive_odom_wall_time"] is None else now - snapshot["passive_odom_wall_time"]
        reasons = []
        raw = {
            "scan_age_s": scan_age,
            "map_age_s": None if grid is None else now - grid.wall_time,
            "passive_odom_age_s": passive_odom_age,
            "scan_hz": self.estimate_hz(snapshot["scan_arrivals"]),
        }

        if scan is None or scan_age is None or scan_age > self.max_topic_age:
            reasons.append("scan_stale_or_missing")
            return self.empty_lidar_result("unavailable", raw, reasons, alignment)
        if grid is None or grid.occupied_count <= 0:
            reasons.append("slam_map_missing_or_empty")
            return self.empty_lidar_result("unavailable", raw, reasons, alignment)

        scan_frame = clean_frame(scan.header.frame_id)
        try:
            stamp = scan.header.stamp if scan.header.stamp != rospy.Time() else None
            scan_to_map = self.lookup_pose(self.slam_map_frame, scan_frame, stamp=stamp)
        except Exception:
            try:
                scan_to_map = self.lookup_pose(self.slam_map_frame, scan_frame)
            except Exception as exc:  # pylint: disable=broad-except
                reasons.append("scan_tf_missing")
                raw["tf_error"] = str(exc)
                return self.empty_lidar_result("tf_missing", raw, reasons, alignment)

        points = self.selected_scan_points(scan)
        if len(points) < 12:
            reasons.append("too_few_valid_scan_points")

        valid_ratio = float(len(points)) / float(max(1, min(len(scan.ranges), self.scan_max_points)))
        scan_hz = raw["scan_hz"]
        scan_component = min(ramp_low(scan_age, 0.15, self.max_topic_age), ramp_high(scan_hz, 3.0, 8.0), ramp_high(valid_ratio, 0.08, 0.35))

        sensor_origin = apply_transform(scan_to_map, {"x": 0.0, "y": 0.0})
        matched_near = 0
        matched_far = 0
        contradictions = 0
        contradiction_checked = 0
        endpoint_errors = []
        jacobians = []
        signed_errors = []
        matched_points = []

        for point in points:
            endpoint = apply_transform(scan_to_map, point)
            match = grid.nearest_occupied(endpoint, self.scan_match_far_m)
            if match is not None:
                matched_far += 1
                endpoint_errors.append(match["distance"])
                if match["distance"] <= self.scan_match_near_m:
                    matched_near += 1
                normal = grid.normal_at_match(endpoint, match)
                d_yaw = {
                    "x": -math.sin(scan_to_map["yaw"]) * point["x"] - math.cos(scan_to_map["yaw"]) * point["y"],
                    "y": math.cos(scan_to_map["yaw"]) * point["x"] - math.sin(scan_to_map["yaw"]) * point["y"],
                }
                jacobians.append([normal["x"], normal["y"], normal["x"] * d_yaw["x"] + normal["y"] * d_yaw["y"]])
                signed_errors.append(
                    normal["x"] * (endpoint["x"] - match["point"]["x"])
                    + normal["y"] * (endpoint["y"] - match["point"]["y"])
                )
                matched_points.append(endpoint)

            if point["range"] > 0.6:
                contradiction_checked += 1
                if self.ray_has_early_occupied_cell(grid, sensor_origin, endpoint, point["range"]):
                    contradictions += 1

        total = max(1, len(points))
        near_ratio = float(matched_near) / total
        far_ratio = float(matched_far) / total
        contradiction_ratio = float(contradictions) / max(1, contradiction_checked)
        median_error = percentile(endpoint_errors, 50.0)
        p95_error = percentile(endpoint_errors, 95.0)
        error_score = ramp_low(median_error, self.scan_match_near_m, self.scan_match_far_m) if median_error is not None else 0.0
        fit_component = clamp(0.50 * near_ratio + 0.35 * far_ratio + 0.15 * error_score)
        free_space_component = ramp_low(contradiction_ratio, 0.12, 0.70)
        if contradiction_ratio > 0.45:
            reasons.append("free_space_contradictions_high")
        observability_component = self.observability_score(jacobians)
        correction = self.scan_pose_correction(jacobians, signed_errors)
        if correction is not None:
            yaw_correction_abs = abs(correction["yaw_rad"])
            translation_correction = correction["translation_m"]
            pose_correction_component = min(
                ramp_low(yaw_correction_abs, 0.025, 0.12),
                ramp_low(translation_correction, 0.04, 0.30),
            )
            if yaw_correction_abs > 0.05 or translation_correction > 0.12:
                reasons.append("scan_pose_correction_high")
        else:
            yaw_correction_abs = None
            translation_correction = None
            pose_correction_component = 0.45 if observability_component >= 0.50 else 0.65
        motion_status = self.scan_motion_status(scan, passive_status)
        motion_distortion_component = motion_status["component"]
        if motion_status["yaw_sweep_rad"] > 0.08 or motion_status["translation_sweep_m"] > 0.12:
            reasons.append("scan_motion_distortion")
        map_age = raw["map_age_s"]
        map_component = min(ramp_high(grid.occupied_count, 20.0, 150.0), 1.0 if map_age is not None and map_age < 30.0 else 0.75)
        tf_component = ramp_low(scan_to_map.get("age", 0.0), 0.1, self.tf_max_age)

        components = {
            "motion_distortion": motion_distortion_component,
            "pose_correction": pose_correction_component,
            "scan_health": scan_component,
            "scan_to_map_fit": fit_component,
            "free_space": free_space_component,
            "observability": observability_component,
            "map_health": map_component,
            "tf_health": tf_component,
        }
        local_confidence = weighted_geomean(
            components,
            {
                "scan_health": 0.12,
                "scan_to_map_fit": 0.32,
                "pose_correction": 0.22,
                "observability": 0.16,
                "motion_distortion": 0.08,
                "free_space": 0.04,
                "map_health": 0.03,
                "tf_health": 0.03,
            },
        )
        if passive_odom_age is not None and passive_odom_age > self.max_topic_age:
            reasons.append("passive_odom_status_stale")

        global_confidence = local_confidence * alignment["confidence"]
        state = "good" if global_confidence >= 0.70 else "local_only" if local_confidence >= 0.60 and alignment["confidence"] < 0.35 else "degraded" if global_confidence >= 0.30 else "poor"
        raw.update(
            {
                "valid_scan_points": len(points),
                "matched_near": matched_near,
                "matched_far": matched_far,
                "near_ratio": near_ratio,
                "far_ratio": far_ratio,
                "median_endpoint_error_m": median_error,
                "p95_endpoint_error_m": p95_error,
                "free_space_contradictions": contradictions,
                "free_space_checked": contradiction_checked,
                "free_space_contradiction_ratio": contradiction_ratio,
                "occupied_cells": grid.occupied_count,
                "scan_fit_pose_correction": correction,
                "scan_motion": motion_status,
                "tf_age_s": scan_to_map.get("age", 0.0),
            }
        )
        position_base_error = max(
            0.03,
            float(median_error or self.scan_match_far_m),
            0.5 * float(translation_correction or 0.0),
            0.5 * motion_status["translation_sweep_m"],
        )
        position_sigma = position_base_error / math.sqrt(max(0.10, local_confidence))
        yaw_base_error = max(
            0.03,
            float(yaw_correction_abs or 0.0),
            0.5 * motion_status["yaw_sweep_rad"],
        )
        yaw_sigma = min(math.pi, yaw_base_error / math.sqrt(max(0.10, observability_component * local_confidence)))
        return {
            "local_confidence": local_confidence,
            "global_confidence": global_confidence,
            "position_sigma_local_m": min(10.0, position_sigma),
            "yaw_sigma_local_rad": yaw_sigma,
            "state": state,
            "components": components,
            "raw": raw,
            "reasons": reasons,
        }

    def empty_lidar_result(self, state, raw, reasons, alignment):
        return {
            "local_confidence": 0.0,
            "global_confidence": 0.0,
            "position_sigma_local_m": None,
            "yaw_sigma_local_rad": None,
            "state": state,
            "components": {},
            "raw": raw,
            "reasons": reasons + ([] if alignment["confidence"] > 0.0 else ["alignment_unavailable"]),
        }

    def ray_has_early_occupied_cell(self, grid, origin, endpoint, range_m):
        dx = endpoint["x"] - origin["x"]
        dy = endpoint["y"] - origin["y"]
        length = max(0.001, math.hypot(dx, dy))
        stop = max(0.0, length - max(0.55, self.scan_match_far_m * 1.5))
        step = max(grid.resolution * 2.0, 0.10)
        steps = int(stop / step)
        if steps <= 0:
            return False
        for index in range(2, steps + 1):
            ratio = float(index) / float(steps + 1)
            x = origin["x"] + dx * ratio
            y = origin["y"] + dy * ratio
            if grid.is_occupied_world(x, y):
                return True
        return False

    def observability_score(self, jacobians):
        if len(jacobians) < 8:
            return ramp_high(len(jacobians), 3.0, 8.0) * 0.35
        try:
            matrix = np.asarray(jacobians, dtype=float)
            information = (matrix.T @ matrix) / float(len(jacobians))
            values = np.linalg.eigvalsh(information)
            values = np.sort(np.maximum(values, 0.0))
            min_strength = ramp_high(math.sqrt(values[0]), 0.03, 0.18)
            condition = values[0] / max(values[-1], 1e-6)
            condition_score = ramp_high(condition, 0.005, 0.06)

            normals = matrix[:, :2]
            angles = np.arctan2(normals[:, 1], normals[:, 0])
            diversity = 1.0 - float(np.hypot(np.mean(np.cos(2.0 * angles)), np.mean(np.sin(2.0 * angles))))
            diversity_score = clamp(diversity)
            return clamp(0.40 * min_strength + 0.30 * condition_score + 0.30 * diversity_score)
        except Exception:  # pylint: disable=broad-except
            return 0.25

    def alignment_confidence(self, snapshot, now):
        status = snapshot["alignment_status"]
        age = None if snapshot["alignment_wall_time"] is None else now - snapshot["alignment_wall_time"]
        if status is None or age is None or age > self.max_topic_age:
            return {
                "confidence": 0.0,
                "source": "none",
                "residual_m": None,
                "p95_m": None,
                "scale_diagnostic": None,
                "drift_warning": False,
                "outlier_warning": False,
                "components": {},
                "reasons": ["alignment_status_stale_or_missing"],
            }

        transform = status.get("transform")
        source = status.get("alignment_source") or "none"
        state = status.get("state") or "unknown"
        residual = status.get("residual_m")
        p95 = status.get("residual_p95_m")
        max_residual = status.get("max_residual_m") or 0.35
        scale = status.get("scale_diagnostic")
        sample_count = float(status.get("boundary_sample_count") or status.get("sample_count") or 0.0)
        path_length = float(status.get("boundary_path_length_m") or status.get("path_length_m") or 0.0)
        drift_warning = bool(status.get("drift_warning", False))
        outlier_warning = bool(status.get("outlier_warning", False))

        if transform is None:
            return {
                "confidence": 0.0,
                "source": source,
                "residual_m": residual,
                "p95_m": p95,
                "scale_diagnostic": scale,
                "drift_warning": drift_warning,
                "outlier_warning": outlier_warning,
                "components": {},
                "reasons": ["alignment_transform_missing"],
            }

        source_score = 1.0 if source == "boundary" else 0.65 if source == "motion" else 0.25
        residual_score = ramp_low(float(residual), 0.05, max_residual) if residual is not None else 0.35
        p95_score = ramp_low(float(p95), 0.08, max_residual * 1.5) if p95 is not None else residual_score
        sample_score = ramp_high(sample_count, 4.0, 14.0)
        path_score = ramp_high(path_length, 1.0, 4.0)
        scale_score = ramp_low(abs(float(scale) - 1.0), 0.02, 0.08) if scale is not None else 0.7
        freshness_score = ramp_low(age, 0.25, self.max_topic_age)

        components = {
            "source": source_score,
            "residual": residual_score,
            "p95": p95_score,
            "samples": sample_score,
            "path": path_score,
            "scale": scale_score,
            "freshness": freshness_score,
        }
        confidence = weighted_geomean(
            components,
            {
                "source": 0.20,
                "residual": 0.25,
                "p95": 0.15,
                "samples": 0.10,
                "path": 0.10,
                "scale": 0.10,
                "freshness": 0.10,
            },
        )
        reasons = []
        if drift_warning:
            confidence = min(confidence, 0.35)
            reasons.append("slam_drift_warning")
        elif outlier_warning:
            confidence = min(confidence, 0.75)
            reasons.append("alignment_outliers_rejected")
        if state not in ("aligned", "degraded", "degraded_slam_drift"):
            confidence = min(confidence, 0.25)
            reasons.append("alignment_not_ready")
        elif state.startswith("degraded"):
            confidence = min(confidence, 0.50)
            reasons.append("alignment_degraded")

        return {
            "confidence": confidence,
            "source": source,
            "residual_m": residual,
            "p95_m": p95,
            "scale_diagnostic": scale,
            "drift_warning": drift_warning,
            "outlier_warning": outlier_warning,
            "components": components,
            "reasons": reasons,
        }

    def agreement_status(self, snapshot):
        status = snapshot["alignment_status"] or {}
        separation = status.get("separation_m")
        timestamp_lag = status.get("pose_sync_lag")
        consistency = None
        if separation is not None:
            consistency = ramp_low(float(separation), 0.10, 0.60)
        return {
            "separation_m": separation,
            "timestamp_lag_s": timestamp_lag,
            "consistency_score": consistency,
            "note": "Agreement is diagnostic only and is not used to decide which sensor is correct.",
        }

    def estimate_hz(self, arrivals):
        if len(arrivals) < 2:
            return 0.0
        duration = arrivals[-1] - arrivals[0]
        if duration <= 0.0:
            return 0.0
        return float(len(arrivals) - 1) / duration

    def status_payload(self):
        now = time.time()
        snapshot = self.latest_snapshot()
        gps = self.gps_confidence(snapshot, now)
        alignment = self.alignment_confidence(snapshot, now)
        lidar = self.lidar_confidence(snapshot, alignment, now)
        return {
            "stamp": now,
            "version": 1,
            "read_only": True,
            "gps": gps,
            "lidar": lidar,
            "alignment": alignment,
            "agreement": self.agreement_status(snapshot),
        }

    def publish_status(self):
        msg = String()
        msg.data = json.dumps(self.status_payload(), sort_keys=True)
        self.status_pub.publish(msg)


if __name__ == "__main__":
    LocalizationConfidence().run()
