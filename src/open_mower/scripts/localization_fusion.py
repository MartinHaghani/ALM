#!/usr/bin/env python3

import json
import math
import threading
import time

import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from xbot_msgs.msg import AbsolutePose


FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE = 1
FLAG_GPS_RTK_FIXED = 2
FLAG_GPS_RTK_FLOAT = 4
FLAG_SENSOR_FUSION_DEAD_RECKONING = 8


def clamp(value, low=0.0, high=1.0):
    return max(low, min(high, value))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(rotation):
    quat = [rotation.x, rotation.y, rotation.z, rotation.w]
    return tf.transformations.euler_from_quaternion(quat)[2]


def quaternion_from_yaw(yaw):
    x, y, z, w = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
    return x, y, z, w


def ros_stamp_age(stamp, now):
    if stamp is None or stamp == rospy.Time():
        return None
    return max(0.0, (now - stamp).to_sec())


def finite(value):
    return value is not None and math.isfinite(float(value))


def finite_pose(pose):
    return finite(pose.get("x")) and finite(pose.get("y")) and finite(pose.get("yaw"))


def circular_weighted_mean(entries):
    sin_sum = 0.0
    cos_sum = 0.0
    weight_sum = 0.0
    for yaw, weight in entries:
        if not finite(yaw) or not finite(weight) or weight <= 0.0:
            continue
        sin_sum += math.sin(yaw) * weight
        cos_sum += math.cos(yaw) * weight
        weight_sum += weight
    if weight_sum <= 0.0:
        return None
    return math.atan2(sin_sum, cos_sum)


def pose_from_absolute_pose(msg):
    return {
        "x": msg.pose.pose.position.x,
        "y": msg.pose.pose.position.y,
        "yaw": yaw_from_quaternion(msg.pose.pose.orientation),
    }


def pose_from_transform(transform):
    return {
        "x": transform.transform.translation.x,
        "y": transform.transform.translation.y,
        "yaw": yaw_from_quaternion(transform.transform.rotation),
    }


def source_status(accepted, confidence, age_s, sigma_m=None, yaw_sigma_rad=None, reason=None, weight=None):
    return {
        "accepted": bool(accepted),
        "age_s": age_s,
        "confidence": confidence,
        "position_sigma_m": sigma_m,
        "reason": reason,
        "weight": weight,
        "yaw_sigma_rad": yaw_sigma_rad,
    }


def gps_rtk_state(flags):
    if flags & FLAG_GPS_RTK_FIXED:
        return "fixed"
    if flags & FLAG_GPS_RTK_FLOAT:
        return "float"
    return "not_fixed"


class LocalizationFusion:
    def __init__(self):
        rospy.init_node("localization_fusion")

        self.pose_topic = rospy.get_param("~pose_topic", "/localization_fusion/pose")
        self.odom_topic = rospy.get_param("~odom_topic", "/localization_fusion/odom")
        self.status_topic = rospy.get_param("~status_topic", "/localization_fusion/status")
        self.gps_topic = rospy.get_param("~gps_topic", "/hw/position/gps")
        self.confidence_topic = rospy.get_param("~confidence_topic", "/localization_confidence/status")
        self.legacy_pose_topic = rospy.get_param("~legacy_pose_topic", "/xbot_positioning/xb_pose")
        self.imu_topic = rospy.get_param("~imu_topic", "/hw/imu/data_raw")
        self.twist_topic = rospy.get_param("~twist_topic", "/hw/diff_drive/measured_twist")
        self.map_frame = clean_frame(rospy.get_param("~map_frame", "map"))
        self.output_base_frame = clean_frame(rospy.get_param("~output_base_frame", "fused_base_link"))
        self.slam_base_frame = clean_frame(rospy.get_param("~slam_base_frame", "slam_base_link"))
        self.rate_hz = float(rospy.get_param("~rate_hz", 10.0))
        self.max_topic_age = float(rospy.get_param("~max_topic_age_sec", 2.0))
        self.tf_timeout = float(rospy.get_param("~tf_timeout", 0.08))
        self.tf_max_age = float(rospy.get_param("~tf_max_age_sec", 1.0))
        self.min_gps_confidence = float(rospy.get_param("~min_gps_confidence", 0.10))
        self.min_lidar_global_confidence = float(rospy.get_param("~min_lidar_global_confidence", 0.10))
        self.position_sigma_floor = float(rospy.get_param("~position_sigma_floor_m", 0.03))
        self.position_sigma_cap = float(rospy.get_param("~position_sigma_cap_m", 5.0))
        self.yaw_sigma_floor = float(rospy.get_param("~yaw_sigma_floor_rad", 0.03))
        self.yaw_sigma_cap = float(rospy.get_param("~yaw_sigma_cap_rad", math.pi))
        self.conflict_warn_distance = float(rospy.get_param("~conflict_warn_distance_m", 0.45))
        self.conflict_block_distance = float(rospy.get_param("~conflict_block_distance_m", 1.0))
        self.publish_tf = bool(rospy.get_param("~publish_tf", True))

        self.lock = threading.Lock()
        self.gps_pose = None
        self.gps_wall_time = None
        self.confidence_status = None
        self.confidence_wall_time = None
        self.legacy_pose = None
        self.legacy_wall_time = None
        self.imu = None
        self.imu_wall_time = None
        self.twist = None
        self.twist_wall_time = None
        self.last_status = None

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.pose_pub = rospy.Publisher(self.pose_topic, AbsolutePose, queue_size=5)
        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=5)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=1, latch=True)

        rospy.Subscriber(self.gps_topic, AbsolutePose, self.on_gps, queue_size=20)
        rospy.Subscriber(self.confidence_topic, String, self.on_confidence, queue_size=5)
        rospy.Subscriber(self.legacy_pose_topic, AbsolutePose, self.on_legacy_pose, queue_size=10)
        rospy.Subscriber(self.imu_topic, Imu, self.on_imu, queue_size=20)
        rospy.Subscriber(self.twist_topic, TwistStamped, self.on_twist, queue_size=10)

        rospy.loginfo(
            "Localization fusion shadow publisher ready: %s, %s, map -> %s.",
            self.pose_topic,
            self.status_topic,
            self.output_base_frame,
        )

    def on_gps(self, msg):
        with self.lock:
            self.gps_pose = msg
            self.gps_wall_time = time.time()

    def on_confidence(self, msg):
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        with self.lock:
            self.confidence_status = payload
            self.confidence_wall_time = time.time()

    def on_legacy_pose(self, msg):
        with self.lock:
            self.legacy_pose = msg
            self.legacy_wall_time = time.time()

    def on_imu(self, msg):
        with self.lock:
            self.imu = msg
            self.imu_wall_time = time.time()

    def on_twist(self, msg):
        with self.lock:
            self.twist = msg
            self.twist_wall_time = time.time()

    def run(self):
        rate = rospy.Rate(max(0.5, self.rate_hz))
        while not rospy.is_shutdown():
            self.publish_once()
            rate.sleep()

    def snapshot(self):
        with self.lock:
            return {
                "confidence": self.confidence_status,
                "confidence_wall_time": self.confidence_wall_time,
                "gps": self.gps_pose,
                "gps_wall_time": self.gps_wall_time,
                "imu": self.imu,
                "imu_wall_time": self.imu_wall_time,
                "legacy": self.legacy_pose,
                "legacy_wall_time": self.legacy_wall_time,
                "twist": self.twist,
                "twist_wall_time": self.twist_wall_time,
            }

    def publish_once(self):
        now = rospy.Time.now()
        wall_now = time.time()
        snapshot = self.snapshot()

        gps = self.gps_measurement(snapshot, now, wall_now)
        lidar = self.lidar_measurement(snapshot, now, wall_now)
        fused = self.fuse_measurements(gps, lidar)

        if fused is None:
            self.publish_status(now, gps, lidar, None)
            return

        self.publish_pose(now, fused)
        self.publish_odom(now, fused)
        if self.publish_tf:
            self.publish_transform(now, fused)
        self.publish_status(now, gps, lidar, fused)

    def gps_measurement(self, snapshot, now, wall_now):
        raw_msg = snapshot["gps"]
        pose_msg = snapshot["legacy"]
        confidence = snapshot["confidence"] or {}
        gps_confidence = confidence.get("gps", {}) if isinstance(confidence, dict) else {}

        raw_age_s = None if snapshot["gps_wall_time"] is None else max(0.0, wall_now - snapshot["gps_wall_time"])
        pose_age_s = None if snapshot["legacy_wall_time"] is None else max(0.0, wall_now - snapshot["legacy_wall_time"])
        confidence_age_s = (
            None if snapshot["confidence_wall_time"] is None else max(0.0, wall_now - snapshot["confidence_wall_time"])
        )
        position_confidence = gps_confidence.get("position_confidence")
        heading_confidence = gps_confidence.get("heading_confidence")
        sigma_m = gps_confidence.get("position_sigma_m")
        yaw_sigma = gps_confidence.get("yaw_sigma_rad")

        if position_confidence is None and pose_msg is not None:
            position_confidence = clamp(1.0 - min(max(pose_msg.position_accuracy, 0.0), 1.0))
        position_confidence = float(position_confidence) if finite(position_confidence) else 0.0
        heading_confidence = float(heading_confidence) if finite(heading_confidence) else 0.0

        if raw_msg is None:
            return source_status(False, 0.0, raw_age_s, sigma_m, yaw_sigma, "raw_gps_missing")
        if raw_age_s is None or raw_age_s > self.max_topic_age:
            status = source_status(False, position_confidence, raw_age_s, sigma_m, yaw_sigma, "raw_gps_stale")
            status["raw_gps_age_s"] = raw_age_s
            status["gps_pose_age_s"] = pose_age_s
            return status
        rtk_state = gps_rtk_state(int(raw_msg.flags))
        if rtk_state != "fixed":
            reason = "gps_rtk_float" if rtk_state == "float" else "gps_not_rtk_fixed"
            status = source_status(False, position_confidence, pose_age_s, sigma_m, yaw_sigma, reason)
            status["raw_gps_age_s"] = raw_age_s
            status["gps_pose_age_s"] = pose_age_s
            status["rtk_state"] = rtk_state
            return status
        if pose_msg is None:
            return source_status(False, 0.0, pose_age_s, sigma_m, yaw_sigma, "gps_base_pose_missing")
        if pose_age_s is None or pose_age_s > self.max_topic_age:
            status = source_status(False, position_confidence, pose_age_s, sigma_m, yaw_sigma, "gps_base_pose_stale")
            status["raw_gps_age_s"] = raw_age_s
            status["gps_pose_age_s"] = pose_age_s
            status["rtk_state"] = rtk_state
            return status
        if confidence_age_s is not None and confidence_age_s > self.max_topic_age:
            status = source_status(False, position_confidence, pose_age_s, sigma_m, yaw_sigma, "confidence_stale")
            status["raw_gps_age_s"] = raw_age_s
            status["gps_pose_age_s"] = pose_age_s
            status["rtk_state"] = rtk_state
            return status

        pose = pose_from_absolute_pose(pose_msg)
        if not finite_pose(pose):
            status = source_status(False, position_confidence, pose_age_s, sigma_m, yaw_sigma, "gps_base_pose_invalid")
            status["raw_gps_age_s"] = raw_age_s
            status["gps_pose_age_s"] = pose_age_s
            status["rtk_state"] = rtk_state
            return status
        if position_confidence < self.min_gps_confidence:
            status = source_status(False, position_confidence, pose_age_s, sigma_m, yaw_sigma, "gps_low_confidence")
            status["raw_gps_age_s"] = raw_age_s
            status["gps_pose_age_s"] = pose_age_s
            status["rtk_state"] = rtk_state
            return status

        if not finite(sigma_m):
            sigma_m = pose_msg.position_accuracy if finite(pose_msg.position_accuracy) and pose_msg.position_accuracy > 0.0 else 1.0
        sigma_m = max(self.position_sigma_floor, min(self.position_sigma_cap, float(sigma_m)))

        yaw_usable = pose_msg.orientation_valid or heading_confidence >= 0.25
        if not yaw_usable:
            yaw_sigma = None
        elif not finite(yaw_sigma):
            yaw_sigma = max(self.yaw_sigma_floor, min(self.yaw_sigma_cap, pose_msg.orientation_accuracy or 0.7))
        else:
            yaw_sigma = max(self.yaw_sigma_floor, min(self.yaw_sigma_cap, float(yaw_sigma)))

        weight = position_confidence / max(sigma_m * sigma_m, self.position_sigma_floor * self.position_sigma_floor)
        return {
            **source_status(True, position_confidence, pose_age_s, sigma_m, yaw_sigma, None, weight),
            "raw_gps_age_s": raw_age_s,
            "gps_pose_age_s": pose_age_s,
            "pose": pose,
            "rtk_state": rtk_state,
            "yaw_weight": (heading_confidence / max(yaw_sigma * yaw_sigma, self.yaw_sigma_floor * self.yaw_sigma_floor))
            if finite(yaw_sigma)
            else 0.0,
        }

    def lidar_measurement(self, snapshot, now, wall_now):
        confidence = snapshot["confidence"] or {}
        lidar_confidence = confidence.get("lidar", {}) if isinstance(confidence, dict) else {}
        alignment = confidence.get("alignment", {}) if isinstance(confidence, dict) else {}
        confidence_age_s = (
            None if snapshot["confidence_wall_time"] is None else max(0.0, wall_now - snapshot["confidence_wall_time"])
        )

        global_confidence = lidar_confidence.get("global_confidence")
        local_confidence = lidar_confidence.get("local_confidence")
        sigma_m = lidar_confidence.get("position_sigma_local_m")
        yaw_sigma = lidar_confidence.get("yaw_sigma_local_rad")
        global_confidence = float(global_confidence) if finite(global_confidence) else 0.0
        local_confidence = float(local_confidence) if finite(local_confidence) else 0.0

        if confidence_age_s is None or confidence_age_s > self.max_topic_age:
            return source_status(False, global_confidence, confidence_age_s, sigma_m, yaw_sigma, "confidence_stale")
        if global_confidence < self.min_lidar_global_confidence:
            reason = "lidar_global_low_confidence"
            if local_confidence >= self.min_lidar_global_confidence and alignment.get("confidence", 0.0) <= 0.0:
                reason = "alignment_unavailable"
            return source_status(False, global_confidence, confidence_age_s, sigma_m, yaw_sigma, reason)

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.slam_base_frame,
                rospy.Time(0),
                rospy.Duration(self.tf_timeout),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
            return source_status(False, global_confidence, confidence_age_s, sigma_m, yaw_sigma, "lidar_tf_missing: %s" % exc)

        tf_age_s = ros_stamp_age(transform.header.stamp, now)
        if tf_age_s is not None and tf_age_s > self.tf_max_age:
            return source_status(False, global_confidence, tf_age_s, sigma_m, yaw_sigma, "lidar_tf_stale")

        pose = pose_from_transform(transform)
        if not finite_pose(pose):
            return source_status(False, global_confidence, tf_age_s, sigma_m, yaw_sigma, "lidar_invalid_pose")

        if not finite(sigma_m):
            sigma_m = 1.0
        alignment_confidence = alignment.get("confidence")
        if finite(alignment_confidence):
            sigma_m = float(sigma_m) / math.sqrt(max(0.10, float(alignment_confidence)))
        sigma_m = max(self.position_sigma_floor, min(self.position_sigma_cap, float(sigma_m)))

        if not finite(yaw_sigma):
            yaw_sigma = 0.7
        yaw_sigma = max(self.yaw_sigma_floor, min(self.yaw_sigma_cap, float(yaw_sigma)))

        weight = global_confidence / max(sigma_m * sigma_m, self.position_sigma_floor * self.position_sigma_floor)
        return {
            **source_status(True, global_confidence, tf_age_s, sigma_m, yaw_sigma, None, weight),
            "pose": pose,
            "yaw_weight": global_confidence / max(yaw_sigma * yaw_sigma, self.yaw_sigma_floor * self.yaw_sigma_floor),
        }

    def fuse_measurements(self, gps, lidar):
        accepted = [source for source in (gps, lidar) if source.get("accepted") and source.get("pose")]
        if not accepted:
            return None

        total_weight = sum(max(0.0, source.get("weight") or 0.0) for source in accepted)
        if total_weight <= 0.0:
            return None

        x = sum(source["pose"]["x"] * source["weight"] for source in accepted) / total_weight
        y = sum(source["pose"]["y"] * source["weight"] for source in accepted) / total_weight
        yaw = circular_weighted_mean((source["pose"]["yaw"], source.get("yaw_weight", 0.0)) for source in accepted)
        if yaw is None:
            yaw = accepted[0]["pose"]["yaw"]

        position_sigma = math.sqrt(1.0 / total_weight)
        yaw_weight = sum(max(0.0, source.get("yaw_weight") or 0.0) for source in accepted)
        yaw_sigma = math.sqrt(1.0 / yaw_weight) if yaw_weight > 0.0 else None
        confidence = 1.0
        for source in accepted:
            confidence *= 1.0 - clamp(source.get("confidence") or 0.0)
        confidence = 1.0 - confidence

        reasons = []
        ready = confidence >= 0.35 and position_sigma <= 0.65
        recommended_action = "continue" if ready else "diagnose_localization"
        conflict_m = None
        if gps.get("accepted") and lidar.get("accepted"):
            conflict_m = math.hypot(gps["pose"]["x"] - lidar["pose"]["x"], gps["pose"]["y"] - lidar["pose"]["y"])
            if conflict_m >= self.conflict_warn_distance:
                reasons.append("gps_lidar_conflict")
                confidence *= clamp(1.0 - (conflict_m - self.conflict_warn_distance) / max(0.1, self.conflict_block_distance), 0.15, 1.0)
            if conflict_m >= self.conflict_block_distance:
                ready = False
                recommended_action = "diagnose_sensor_conflict"

        source_weights = {
            "gps": (gps.get("weight") or 0.0) / total_weight if gps.get("accepted") else 0.0,
            "lidar": (lidar.get("weight") or 0.0) / total_weight if lidar.get("accepted") else 0.0,
        }
        active_sources = [name for name, source in (("gps", gps), ("lidar", lidar)) if source.get("accepted")]
        state = "ready" if ready else "degraded"
        if active_sources == ["gps"]:
            state = "gps_only" if ready else "gps_degraded"
        elif active_sources == ["lidar"]:
            state = "lidar_only" if ready else "lidar_degraded"

        return {
            "active_sources": active_sources,
            "confidence": clamp(confidence),
            "gps_lidar_separation_m": conflict_m,
            "position_sigma_m": max(self.position_sigma_floor, min(self.position_sigma_cap, position_sigma)),
            "ready_for_navigation": ready,
            "recommended_action": recommended_action,
            "reasons": reasons,
            "source_weights": source_weights,
            "state": state,
            "x": x,
            "y": y,
            "yaw": normalize_angle(yaw),
            "yaw_sigma_rad": yaw_sigma,
        }

    def publish_pose(self, now, fused):
        msg = AbsolutePose()
        msg.header.stamp = now
        msg.header.frame_id = self.map_frame
        msg.source = AbsolutePose.SOURCE_SENSOR_FUSION
        msg.flags = FLAG_SENSOR_FUSION_DEAD_RECKONING
        if fused["ready_for_navigation"]:
            msg.flags |= FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE
        msg.orientation_valid = True
        msg.motion_vector_valid = False
        msg.position_accuracy = float(fused["position_sigma_m"])
        msg.orientation_accuracy = float(fused["yaw_sigma_rad"] or self.yaw_sigma_cap)
        msg.pose.pose.position.x = fused["x"]
        msg.pose.pose.position.y = fused["y"]
        msg.pose.pose.position.z = 0.0
        qx, qy, qz, qw = quaternion_from_yaw(fused["yaw"])
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance = covariance_from_sigmas(fused["position_sigma_m"], fused["yaw_sigma_rad"])
        msg.vehicle_heading = fused["yaw"]
        msg.motion_heading = fused["yaw"]
        self.pose_pub.publish(msg)

    def publish_odom(self, now, fused):
        msg = Odometry()
        msg.header.stamp = now
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.output_base_frame
        msg.pose.pose.position.x = fused["x"]
        msg.pose.pose.position.y = fused["y"]
        qx, qy, qz, qw = quaternion_from_yaw(fused["yaw"])
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance = covariance_from_sigmas(fused["position_sigma_m"], fused["yaw_sigma_rad"])
        self.odom_pub.publish(msg)

    def publish_transform(self, now, fused):
        transform = TransformStamped()
        transform.header.stamp = now
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.output_base_frame
        transform.transform.translation.x = fused["x"]
        transform.transform.translation.y = fused["y"]
        transform.transform.translation.z = 0.0
        qx, qy, qz, qw = quaternion_from_yaw(fused["yaw"])
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

    def publish_status(self, now, gps, lidar, fused):
        status = {
            "accepted_source_weights": (fused or {}).get("source_weights", {}),
            "confidence": (fused or {}).get("confidence"),
            "gps_lidar_separation_m": (fused or {}).get("gps_lidar_separation_m"),
            "position_sigma_m": (fused or {}).get("position_sigma_m"),
            "read_only": True,
            "ready_for_navigation": (fused or {}).get("ready_for_navigation", False),
            "recommended_action": (fused or {}).get("recommended_action", "waiting_for_sources"),
            "reasons": (fused or {}).get("reasons", []),
            "source_weights": (fused or {}).get("source_weights", {}),
            "sources": {
                "gps": strip_pose(gps),
                "lidar": strip_pose(lidar),
            },
            "stamp": now.to_sec(),
            "state": (fused or {}).get("state", "waiting_for_sources"),
            "version": 1,
            "yaw_sigma_rad": (fused or {}).get("yaw_sigma_rad"),
        }
        self.last_status = status
        self.status_pub.publish(String(data=json.dumps(status, sort_keys=True)))


def covariance_from_sigmas(position_sigma_m, yaw_sigma_rad):
    position_variance = max(0.0, float(position_sigma_m or 999.0) ** 2)
    yaw_variance = max(0.0, float(yaw_sigma_rad or math.pi) ** 2)
    covariance = [0.0] * 36
    covariance[0] = position_variance
    covariance[7] = position_variance
    covariance[14] = 999.0
    covariance[21] = 999.0
    covariance[28] = 999.0
    covariance[35] = yaw_variance
    return covariance


def strip_pose(source):
    cleaned = dict(source)
    cleaned.pop("pose", None)
    cleaned.pop("yaw_weight", None)
    return cleaned


def clean_frame(frame):
    return str(frame).lstrip("/")


if __name__ == "__main__":
    try:
        LocalizationFusion().run()
    except rospy.ROSInterruptException:
        pass
