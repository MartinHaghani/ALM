#!/usr/bin/env python3

import json
import math
import os
import signal
import subprocess
import time

import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse


def clean_frame(frame):
    return str(frame).lstrip("/")


def yaw_from_quaternion(rotation):
    quat = [rotation.x, rotation.y, rotation.z, rotation.w]
    return tf.transformations.euler_from_quaternion(quat)[2]


def quaternion_from_yaw(yaw):
    x, y, z, w = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
    return x, y, z, w


def invert_planar_transform(transform):
    translation = transform.transform.translation
    yaw = yaw_from_quaternion(transform.transform.rotation)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    return {
        "x": -cos_yaw * translation.x - sin_yaw * translation.y,
        "y": sin_yaw * translation.x - cos_yaw * translation.y,
        "z": -translation.z,
        "yaw": -yaw,
    }


class PassiveSlamManager:
    def __init__(self):
        rospy.init_node("passive_slam_manager")

        self.slam_namespace = clean_frame(rospy.get_param("~slam_namespace", "slam_toolbox"))
        self.slam_node_name = clean_frame(rospy.get_param("~slam_node_name", self.slam_namespace))
        self.slam_executable = rospy.get_param("~slam_executable", "async_slam_toolbox_node")
        self.odom_frame = clean_frame(rospy.get_param("~odom_frame", "slam_odom"))
        self.origin_frame = clean_frame(rospy.get_param("~origin_frame", "map"))
        self.base_frame = clean_frame(rospy.get_param("~base_frame", "base_link"))
        self.start_enabled = bool(rospy.get_param("~start_enabled", False))
        self.publish_origin_transform_enabled = bool(rospy.get_param("~publish_origin_transform", False))
        self.reset_odom_on_start = bool(rospy.get_param("~reset_odom_on_start", True))
        self.reset_odom_on_clear = bool(rospy.get_param("~reset_odom_on_clear", True))
        self.reset_odom_service = rospy.get_param("~reset_odom_service", "/passive_slam_odom/reset")
        self.tf_timeout = float(rospy.get_param("~tf_timeout", 2.0))
        self.service_timeout = float(rospy.get_param("~service_timeout", 2.0))
        self.restart_delay = float(rospy.get_param("~restart_delay", 3.0))
        self.status_period = float(rospy.get_param("~status_period", 0.5))
        self.tf_rate_hz = float(rospy.get_param("~tf_rate_hz", 20.0))
        self.cleanup_master_on_stop = bool(rospy.get_param("~cleanup_master_on_stop", True))

        self.mapping_enabled = False
        self.last_error = ""
        self.last_exit_code = None
        self.last_restart_at = 0.0
        self.last_odom_reset_at = None
        self.last_odom_reset_message = ""
        self.origin_transform = None
        self.process = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.status_pub = rospy.Publisher("~status", String, queue_size=1, latch=True)

        rospy.Service("~set_mapping_enabled", SetBool, self.set_mapping_enabled)
        rospy.Service("~clear_map", Trigger, self.clear_map)

        rospy.on_shutdown(self.shutdown)
        rospy.loginfo(
            "Passive SLAM manager ready. Mapping starts %s, slam node /%s, odom frame %s, origin frame %s.",
            "enabled" if self.start_enabled else "disabled",
            self.slam_node_name,
            self.odom_frame,
            self.origin_frame,
        )

    def run(self):
        if self.start_enabled:
            try:
                self.enable_mapping()
            except Exception as exc:  # pylint: disable=broad-except
                self.last_error = str(exc)
                rospy.logerr("Could not auto-start passive SLAM: %s", exc)

        rate = rospy.Rate(max(1.0, self.tf_rate_hz))
        last_status = 0.0
        while not rospy.is_shutdown():
            self.check_process()
            self.publish_origin_transform()

            now = time.time()
            if now - last_status >= self.status_period:
                self.publish_status()
                last_status = now

            rate.sleep()

    def slam_command(self):
        return [
            "rosrun",
            "slam_toolbox",
            self.slam_executable,
            "__name:={}".format(self.slam_node_name),
        ]

    def process_running(self):
        return self.process is not None and self.process.poll() is None

    def lookup_current_pose(self):
        return self.tf_buffer.lookup_transform(
            self.origin_frame,
            self.base_frame,
            rospy.Time(0),
            rospy.Duration(self.tf_timeout),
        )

    def capture_origin(self):
        if not self.publish_origin_transform_enabled:
            self.origin_transform = None
            self.last_error = ""
            return

        if self.odom_frame == self.origin_frame:
            self.origin_transform = None
            return

        current_pose = self.lookup_current_pose()
        self.origin_transform = invert_planar_transform(current_pose)
        self.last_error = ""
        rospy.loginfo(
            "Captured passive SLAM origin: %s -> %s starts at current %s pose.",
            self.odom_frame,
            self.origin_frame,
            self.base_frame,
        )

    def start_slam_process(self):
        if self.process_running():
            return

        command = self.slam_command()
        rospy.loginfo("Starting passive SLAM node: %s", " ".join(command))
        self.process = subprocess.Popen(command, start_new_session=True)
        self.last_exit_code = None

    def stop_slam_process(self):
        process = self.process
        if not process:
            self.cleanup_stale_slam_node()
            return

        if process.poll() is None:
            rospy.loginfo("Stopping passive SLAM node /%s.", self.slam_node_name)
            self.request_slam_node_shutdown()
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                rospy.logwarn("Passive SLAM node did not stop after ROS shutdown request; sending SIGTERM.")
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                    process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    rospy.logwarn("Passive SLAM node did not stop after SIGTERM; killing it.")
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                    process.wait(timeout=2.0)
            except OSError as exc:
                rospy.logwarn("Could not stop passive SLAM process cleanly: %s", exc)

        self.last_exit_code = process.poll()
        if self.process is process:
            self.process = None
        self.cleanup_stale_slam_node()

    def reset_local_odom(self):
        if not self.reset_odom_service:
            return

        try:
            rospy.wait_for_service(self.reset_odom_service, timeout=self.service_timeout)
            reset = rospy.ServiceProxy(self.reset_odom_service, Trigger)
            response = reset()
            self.last_odom_reset_at = time.time()
            self.last_odom_reset_message = response.message
            if not response.success:
                raise RuntimeError(response.message or "passive SLAM odom reset failed")
        except Exception as exc:  # pylint: disable=broad-except
            self.last_error = str(exc)
            rospy.logerr("Passive SLAM odom reset failed: %s", exc)
            raise

    def request_slam_node_shutdown(self):
        node_name = "/{}".format(self.slam_node_name)
        try:
            subprocess.run(
                ["rosnode", "kill", node_name],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=3.0,
                check=False,
            )
        except (OSError, subprocess.SubprocessError) as exc:
            rospy.logdebug("Could not request ROS shutdown for %s: %s", node_name, exc)

    def cleanup_stale_slam_node(self):
        if not self.cleanup_master_on_stop:
            return

        try:
            subprocess.run(
                ["rosnode", "cleanup"],
                input=b"y\n",
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=8.0,
                check=False,
            )
        except (OSError, subprocess.SubprocessError) as exc:
            rospy.logdebug("Could not clean stale passive SLAM ROS master entries: %s", exc)

    def enable_mapping(self):
        if self.mapping_enabled and self.process_running():
            return

        if self.reset_odom_on_start:
            self.reset_local_odom()
        self.capture_origin()
        self.mapping_enabled = True
        self.start_slam_process()
        self.publish_status()

    def disable_mapping(self):
        self.mapping_enabled = False
        self.stop_slam_process()
        self.publish_status()

    def set_mapping_enabled(self, request):
        try:
            if request.data:
                self.enable_mapping()
                return SetBoolResponse(success=True, message="Passive SLAM mapping started")

            self.disable_mapping()
            return SetBoolResponse(success=True, message="Passive SLAM mapping stopped")
        except Exception as exc:  # pylint: disable=broad-except
            self.last_error = str(exc)
            rospy.logerr("Passive SLAM mapping toggle failed: %s", exc)
            return SetBoolResponse(success=False, message=str(exc))

    def clear_map(self, _request):
        try:
            self.mapping_enabled = False
            self.stop_slam_process()
            self.origin_transform = None
            if self.reset_odom_on_clear:
                self.reset_local_odom()
            self.last_error = ""
            self.publish_status()
            return TriggerResponse(success=True, message="Passive SLAM map cleared; mapping is stopped")
        except Exception as exc:  # pylint: disable=broad-except
            self.last_error = str(exc)
            rospy.logerr("Passive SLAM clear failed: %s", exc)
            return TriggerResponse(success=False, message=str(exc))

    def check_process(self):
        if not self.process:
            return

        exit_code = self.process.poll()
        if exit_code is None:
            return

        self.last_exit_code = exit_code
        self.process = None
        if not self.mapping_enabled:
            return

        now = time.time()
        if now - self.last_restart_at < self.restart_delay:
            return

        self.last_restart_at = now
        rospy.logwarn("Passive SLAM node exited with code %s; restarting.", exit_code)
        self.start_slam_process()

    def publish_origin_transform(self):
        if (
            not self.publish_origin_transform_enabled
            or not self.mapping_enabled
            or not self.origin_transform
            or self.odom_frame == self.origin_frame
        ):
            return

        msg = TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.origin_frame
        msg.transform.translation.x = self.origin_transform["x"]
        msg.transform.translation.y = self.origin_transform["y"]
        msg.transform.translation.z = self.origin_transform["z"]
        x, y, z, w = quaternion_from_yaw(self.origin_transform["yaw"])
        msg.transform.rotation.x = x
        msg.transform.rotation.y = y
        msg.transform.rotation.z = z
        msg.transform.rotation.w = w
        self.tf_broadcaster.sendTransform(msg)

    def publish_status(self):
        status = {
            "mapping_enabled": self.mapping_enabled,
            "slam_running": self.process_running(),
            "slam_node": "/{}".format(self.slam_node_name),
            "odom_frame": self.odom_frame,
            "origin_frame": self.origin_frame,
            "base_frame": self.base_frame,
            "origin_captured": self.origin_transform is not None or self.odom_frame == self.origin_frame,
            "last_error": self.last_error,
            "last_exit_code": self.last_exit_code,
            "last_odom_reset_at": self.last_odom_reset_at,
            "last_odom_reset_message": self.last_odom_reset_message,
            "publish_origin_transform": self.publish_origin_transform_enabled,
        }
        msg = String()
        msg.data = json.dumps(status, sort_keys=True)
        self.status_pub.publish(msg)

    def shutdown(self):
        self.mapping_enabled = False
        self.stop_slam_process()


if __name__ == "__main__":
    PassiveSlamManager().run()
