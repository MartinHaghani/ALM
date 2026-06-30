#!/usr/bin/env python3

import json
import math
import threading
import time

import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse


def clean_frame(frame):
    return str(frame).lstrip("/")


def quaternion_from_yaw(yaw):
    x, y, z, w = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
    return x, y, z, w


class PassiveSlamOdom:
    def __init__(self):
        rospy.init_node("passive_slam_odom")

        self.odom_frame = clean_frame(rospy.get_param("~odom_frame", "slam_odom"))
        self.base_frame = clean_frame(rospy.get_param("~base_frame", "slam_base_link"))
        self.scan_frame = clean_frame(rospy.get_param("~scan_frame", "slam_lidar"))
        self.odom_topic = rospy.get_param("~odom_topic", "/slam_toolbox/local_odom")
        self.scan_out_topic = rospy.get_param("~scan_out_topic", "/slam_toolbox/scan")
        self.status_period = float(rospy.get_param("~status_period", 0.5))
        self.twist_timeout = float(rospy.get_param("~twist_timeout", 0.5))
        self.max_dt = float(rospy.get_param("~max_dt", 0.25))
        self.gyro_calibration_seconds = float(rospy.get_param("~gyro_calibration_seconds", 5.0))
        self.gyro_stationary_vx_threshold = float(rospy.get_param("~gyro_stationary_vx_threshold", 0.03))
        self.gyro_stationary_wz_threshold = float(rospy.get_param("~gyro_stationary_wz_threshold", 0.04))
        self.gyro_warning_yaw_rate_threshold = float(rospy.get_param("~gyro_warning_yaw_rate_threshold", 0.025))
        self.gyro_warning_seconds = float(rospy.get_param("~gyro_warning_seconds", 2.0))
        self.gyro_calibration_max_abs_offset = float(rospy.get_param("~gyro_calibration_max_abs_offset", 0.05))
        self.gyro_calibration_max_stddev = float(rospy.get_param("~gyro_calibration_max_stddev", 0.01))
        self.gyro_calibration_max_range = float(rospy.get_param("~gyro_calibration_max_range", 0.03))
        self.publish_tf = bool(rospy.get_param("~publish_tf", True))

        self.lock = threading.Lock()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.twist_yaw_rate = 0.0
        self.raw_yaw_rate = 0.0
        self.yaw_rate = 0.0
        self.gyro_offset = 0.0
        self.previous_gyro_offset = 0.0
        self.gyro_offset_sum = 0.0
        self.gyro_offset_sum_sq = 0.0
        self.gyro_offset_min = 0.0
        self.gyro_offset_max = 0.0
        self.gyro_offset_samples = 0
        self.calibration_start = None
        self.calibration_had_valid_offset = False
        self.last_calibration_rejected = False
        self.last_calibration_message = None
        self.calibrated = self.gyro_calibration_seconds <= 0.0
        self.last_imu_stamp = None
        self.last_imu_wall_time = None
        self.last_twist_wall_time = None
        self.last_reset_wall_time = time.time()
        self.gyro_warning_candidate_since = None
        self.gyro_warning_duration = 0.0
        self.gyro_stationary_warning = False
        self.stationary_detected = False

        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
        self.scan_pub = rospy.Publisher(self.scan_out_topic, LaserScan, queue_size=2)
        self.status_pub = rospy.Publisher("~status", String, queue_size=1, latch=True)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        rospy.Subscriber("~imu_in", Imu, self.on_imu, queue_size=20)
        rospy.Subscriber("~twist_in", TwistStamped, self.on_twist, queue_size=10)
        rospy.Subscriber("~scan_in", LaserScan, self.on_scan, queue_size=2)
        rospy.Service("~reset", Trigger, self.reset)
        rospy.Service("~calibrate_gyro", Trigger, self.calibrate_gyro)

        rospy.loginfo(
            "Passive SLAM odom ready: %s -> %s from twist + IMU gyro, republishing scans as %s on %s.",
            self.odom_frame,
            self.base_frame,
            self.scan_frame,
            self.scan_out_topic,
        )

    def run(self):
        rate = rospy.Rate(max(1.0, 1.0 / max(0.01, self.status_period)))
        while not rospy.is_shutdown():
            self.publish_status()
            rate.sleep()

    def reset(self, _request):
        with self.lock:
            self.x = 0.0
            self.y = 0.0
            self.yaw = 0.0
            self.yaw_rate = 0.0
            self.last_imu_stamp = None
            self.last_reset_wall_time = time.time()

        self.publish_odometry(rospy.Time.now())
        self.publish_status()
        return TriggerResponse(success=True, message="Passive SLAM local odom reset")

    def calibrate_gyro(self, _request):
        if self.gyro_calibration_seconds <= 0.0:
            return TriggerResponse(success=False, message="Passive SLAM gyro calibration is disabled")

        with self.lock:
            if self.twist_is_fresh() and (
                abs(self.current_vx()) > self.gyro_stationary_vx_threshold
                or abs(self.current_twist_yaw_rate()) > self.gyro_stationary_wz_threshold
            ):
                return TriggerResponse(success=False, message="Mower appears to be moving; stop before calibrating gyro")
            self.start_gyro_calibration_locked()

        self.publish_status()
        return TriggerResponse(
            success=True,
            message="Passive SLAM gyro calibration started; keep the mower completely still for {:.1f} s".format(
                self.gyro_calibration_seconds
            ),
        )

    def start_gyro_calibration_locked(self):
        self.previous_gyro_offset = self.gyro_offset
        self.calibration_had_valid_offset = self.calibrated
        self.last_calibration_rejected = False
        self.last_calibration_message = None
        self.calibrated = False
        self.calibration_start = None
        self.gyro_offset_sum = 0.0
        self.gyro_offset_sum_sq = 0.0
        self.gyro_offset_min = 0.0
        self.gyro_offset_max = 0.0
        self.gyro_offset_samples = 0
        self.last_imu_stamp = None
        self.raw_yaw_rate = 0.0
        self.yaw_rate = 0.0
        self.gyro_warning_candidate_since = None
        self.gyro_warning_duration = 0.0
        self.gyro_stationary_warning = False

    def current_vx(self):
        if self.last_twist_wall_time is None:
            return 0.0
        if time.time() - self.last_twist_wall_time > self.twist_timeout:
            return 0.0
        return self.vx

    def current_twist_yaw_rate(self):
        if self.last_twist_wall_time is None:
            return 0.0
        if time.time() - self.last_twist_wall_time > self.twist_timeout:
            return 0.0
        return self.twist_yaw_rate

    def twist_is_fresh(self):
        return self.last_twist_wall_time is not None and time.time() - self.last_twist_wall_time <= self.twist_timeout

    def update_gyro_warning_locked(self):
        now = time.time()
        twist_fresh = self.twist_is_fresh()
        vx = self.current_vx()
        twist_yaw_rate = self.current_twist_yaw_rate()
        self.stationary_detected = bool(
            twist_fresh
            and abs(vx) <= self.gyro_stationary_vx_threshold
            and abs(twist_yaw_rate) <= self.gyro_stationary_wz_threshold
        )

        candidate = bool(
            self.calibrated
            and self.stationary_detected
            and abs(self.yaw_rate) >= self.gyro_warning_yaw_rate_threshold
        )
        if candidate:
            if self.gyro_warning_candidate_since is None:
                self.gyro_warning_candidate_since = now
            self.gyro_warning_duration = now - self.gyro_warning_candidate_since
            self.gyro_stationary_warning = self.gyro_warning_duration >= self.gyro_warning_seconds
        else:
            self.gyro_warning_candidate_since = None
            self.gyro_warning_duration = 0.0
            self.gyro_stationary_warning = False

    def on_twist(self, msg):
        with self.lock:
            self.vx = msg.twist.linear.x
            self.twist_yaw_rate = msg.twist.angular.z
            self.last_twist_wall_time = time.time()
            self.update_gyro_warning_locked()

    def on_scan(self, msg):
        scan = LaserScan()
        scan.header = msg.header
        scan.header.frame_id = self.scan_frame
        scan.angle_min = msg.angle_min
        scan.angle_max = msg.angle_max
        scan.angle_increment = msg.angle_increment
        scan.time_increment = msg.time_increment
        scan.scan_time = msg.scan_time
        scan.range_min = msg.range_min
        scan.range_max = msg.range_max
        scan.ranges = msg.ranges
        scan.intensities = msg.intensities
        self.scan_pub.publish(scan)

    def on_imu(self, msg):
        with self.lock:
            now = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
            self.last_imu_wall_time = time.time()
            self.raw_yaw_rate = msg.angular_velocity.z

            if not self.calibrated:
                if self.calibration_start is None:
                    self.calibration_start = now
                    self.gyro_offset_sum = 0.0
                    self.gyro_offset_sum_sq = 0.0
                    self.gyro_offset_samples = 0
                    self.gyro_offset_min = msg.angular_velocity.z
                    self.gyro_offset_max = msg.angular_velocity.z
                    rospy.loginfo("Passive SLAM odom gyro calibration started.")

                self.gyro_offset_sum += msg.angular_velocity.z
                self.gyro_offset_sum_sq += msg.angular_velocity.z * msg.angular_velocity.z
                self.gyro_offset_min = min(self.gyro_offset_min, msg.angular_velocity.z)
                self.gyro_offset_max = max(self.gyro_offset_max, msg.angular_velocity.z)
                self.gyro_offset_samples += 1
                if (now - self.calibration_start).to_sec() < self.gyro_calibration_seconds:
                    self.last_imu_stamp = now
                    return

                mean_offset = self.gyro_offset_sum / max(1, self.gyro_offset_samples)
                variance = max(0.0, self.gyro_offset_sum_sq / max(1, self.gyro_offset_samples) - mean_offset * mean_offset)
                stddev = math.sqrt(variance)
                sample_range = self.gyro_offset_max - self.gyro_offset_min
                valid_calibration = (
                    abs(mean_offset) <= self.gyro_calibration_max_abs_offset
                    and stddev <= self.gyro_calibration_max_stddev
                    and sample_range <= self.gyro_calibration_max_range
                )

                if not valid_calibration:
                    self.last_calibration_rejected = True
                    self.last_calibration_message = (
                        "rejected gyro calibration: mean={:.6f} stddev={:.6f} range={:.6f}".format(
                            mean_offset,
                            stddev,
                            sample_range,
                        )
                    )
                    rospy.logerr("Passive SLAM odom %s", self.last_calibration_message)
                    self.gyro_offset_sum = 0.0
                    self.gyro_offset_sum_sq = 0.0
                    self.gyro_offset_samples = 0
                    if self.calibration_had_valid_offset:
                        self.gyro_offset = self.previous_gyro_offset
                        self.calibrated = True
                        self.last_imu_stamp = now
                        self.yaw_rate = msg.angular_velocity.z - self.gyro_offset
                        self.update_gyro_warning_locked()
                        return

                    self.calibration_start = None
                    self.last_imu_stamp = now
                    return

                self.gyro_offset = mean_offset
                self.calibrated = True
                self.last_imu_stamp = now
                self.yaw_rate = 0.0
                self.last_calibration_rejected = False
                self.last_calibration_message = None
                self.update_gyro_warning_locked()
                rospy.loginfo("Passive SLAM odom gyro offset: %s", self.gyro_offset)
                return

            if self.last_imu_stamp is None:
                self.last_imu_stamp = now
                return

            dt = (now - self.last_imu_stamp).to_sec()
            self.last_imu_stamp = now
            if dt <= 0.0 or dt > self.max_dt:
                return

            self.yaw_rate = msg.angular_velocity.z - self.gyro_offset
            vx = self.current_vx()
            self.yaw += self.yaw_rate * dt
            self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
            self.x += math.cos(self.yaw) * vx * dt
            self.y += math.sin(self.yaw) * vx * dt
            self.update_gyro_warning_locked()

        self.publish_odometry(now)

    def publish_odometry(self, stamp):
        with self.lock:
            x = self.x
            y = self.y
            yaw = self.yaw
            vx = self.current_vx()
            yaw_rate = self.yaw_rate

        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = yaw_rate
        self.odom_pub.publish(odom)

        if not self.publish_tf:
            return

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

    def publish_status(self):
        with self.lock:
            calibration_elapsed = None
            calibration_progress = 1.0
            if not self.calibrated:
                if self.calibration_start is not None:
                    calibration_elapsed = max(0.0, (rospy.Time.now() - self.calibration_start).to_sec())
                calibration_progress = min(
                    1.0,
                    max(0.0, (calibration_elapsed or 0.0) / max(0.001, self.gyro_calibration_seconds)),
                )
            self.update_gyro_warning_locked()
            status = {
                "base_frame": self.base_frame,
                "calibrated": self.calibrated,
                "gyro_calibrating": not self.calibrated,
                "gyro_calibration_elapsed": calibration_elapsed,
                "gyro_calibration_last_message": self.last_calibration_message,
                "gyro_calibration_last_rejected": self.last_calibration_rejected,
                "gyro_calibration_max_abs_offset": self.gyro_calibration_max_abs_offset,
                "gyro_calibration_max_range": self.gyro_calibration_max_range,
                "gyro_calibration_max_stddev": self.gyro_calibration_max_stddev,
                "gyro_calibration_progress": calibration_progress,
                "gyro_calibration_seconds": self.gyro_calibration_seconds,
                "gyro_offset": self.gyro_offset,
                "gyro_stationary_vx_threshold": self.gyro_stationary_vx_threshold,
                "gyro_stationary_wz_threshold": self.gyro_stationary_wz_threshold,
                "gyro_warning_duration": self.gyro_warning_duration,
                "gyro_warning_seconds": self.gyro_warning_seconds,
                "gyro_warning_yaw_rate_threshold": self.gyro_warning_yaw_rate_threshold,
                "gyro_stationary_warning": self.gyro_stationary_warning,
                "last_imu_age": None if self.last_imu_wall_time is None else time.time() - self.last_imu_wall_time,
                "last_reset_at": self.last_reset_wall_time,
                "last_twist_age": None if self.last_twist_wall_time is None else time.time() - self.last_twist_wall_time,
                "odom_frame": self.odom_frame,
                "raw_yaw_rate": self.raw_yaw_rate,
                "scan_frame": self.scan_frame,
                "stationary_detected": self.stationary_detected,
                "twist_yaw_rate": self.current_twist_yaw_rate(),
                "vx": self.current_vx(),
                "x": self.x,
                "y": self.y,
                "yaw": self.yaw,
                "yaw_rate": self.yaw_rate,
            }
        msg = String()
        msg.data = json.dumps(status, sort_keys=True)
        self.status_pub.publish(msg)


if __name__ == "__main__":
    PassiveSlamOdom().run()
