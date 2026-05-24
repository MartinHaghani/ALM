#!/usr/bin/env python3

import csv
import datetime
import os

import rospy
from mower_msgs.msg import HwPower


CSV_HEADER = [
    "wall_time_utc",
    "ros_time",
    "battery_voltage",
    "battery_voltage_valid",
    "battery_percentage_0_to_1",
    "battery_source",
    "left_drive_voltage",
    "left_drive_voltage_valid",
    "right_drive_voltage",
    "right_drive_voltage_valid",
    "mower_esc_voltage",
    "mower_esc_voltage_valid",
    "drive_voltage_mismatch",
    "warning",
]


def default_log_path():
    ros_home = os.environ.get("ROS_HOME") or os.path.join(os.path.expanduser("~"), ".ros")
    return os.path.join(ros_home, "battery_voltage_log.csv")


def clean_log_path(path):
    path = os.path.expandvars(os.path.expanduser(path or ""))
    if path:
        return path
    return default_log_path()


def get_bool_param(name, default):
    value = rospy.get_param(name, default)
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class BatteryVoltageLogger:
    def __init__(self):
        rospy.init_node("battery_voltage_logger")

        self.topic = rospy.get_param("~topic", "/hw/power")
        self.log_path = clean_log_path(rospy.get_param("~log_path", ""))
        self.sample_period = max(0.1, float(rospy.get_param("~sample_period", 1.0)))
        self.fsync_each_row = get_bool_param("~fsync_each_row", True)
        self.max_bytes = max(0, int(rospy.get_param("~max_bytes", 10 * 1024 * 1024)))
        self.max_files = max(1, int(rospy.get_param("~max_files", 5)))

        self.latest_msg = None
        self.last_written_stamp = None
        self.log_file = None
        self.writer = None

        self.open_log()
        self.subscriber = rospy.Subscriber(self.topic, HwPower, self.power_received, queue_size=1, tcp_nodelay=True)
        self.timer = rospy.Timer(rospy.Duration(self.sample_period), self.write_latest)
        rospy.on_shutdown(self.close_log)

        rospy.loginfo(
            "Battery voltage logger writing %s every %.2fs from %s, rotating at %d bytes x %d files",
            self.log_path,
            self.sample_period,
            self.topic,
            self.max_bytes,
            self.max_files,
        )

    def open_log(self):
        log_dir = os.path.dirname(os.path.abspath(self.log_path))
        os.makedirs(log_dir, exist_ok=True)
        needs_header = not os.path.exists(self.log_path) or os.path.getsize(self.log_path) == 0
        self.log_file = open(self.log_path, "a", newline="", buffering=1)
        self.writer = csv.writer(self.log_file)
        if needs_header:
            self.writer.writerow(CSV_HEADER)
            self.flush()

    def close_log(self):
        if self.log_file is None:
            return
        try:
            self.flush()
        finally:
            self.log_file.close()
            self.log_file = None
            self.writer = None

    def flush(self):
        self.log_file.flush()
        if self.fsync_each_row:
            os.fsync(self.log_file.fileno())

    def rotate_if_needed(self):
        if self.log_file is None:
            self.open_log()
            return
        if self.max_bytes <= 0 or self.log_file.tell() < self.max_bytes:
            return

        try:
            self.close_log()

            if self.max_files <= 1:
                if os.path.exists(self.log_path):
                    os.remove(self.log_path)
            else:
                for index in range(self.max_files - 1, 0, -1):
                    source = self.log_path if index == 1 else "{}.{}".format(self.log_path, index - 1)
                    target = "{}.{}".format(self.log_path, index)
                    if not os.path.exists(source):
                        continue
                    if os.path.exists(target):
                        os.remove(target)
                    os.replace(source, target)
        finally:
            if self.log_file is None:
                self.open_log()

    def power_received(self, msg):
        self.latest_msg = msg

    def write_latest(self, _event):
        msg = self.latest_msg
        if msg is None:
            return
        if self.last_written_stamp == msg.stamp:
            return

        row = [
            datetime.datetime.now(datetime.timezone.utc).isoformat().replace("+00:00", "Z"),
            "{:.9f}".format(msg.stamp.to_sec()),
            "{:.3f}".format(msg.v_battery),
            msg.battery_voltage_valid,
            "{:.4f}".format(msg.battery_percentage),
            msg.battery_source,
            "{:.3f}".format(msg.left_drive_voltage),
            msg.left_drive_voltage_valid,
            "{:.3f}".format(msg.right_drive_voltage),
            msg.right_drive_voltage_valid,
            "{:.3f}".format(msg.mower_esc_voltage),
            msg.mower_esc_voltage_valid,
            msg.drive_voltage_mismatch,
            msg.warning,
        ]

        try:
            self.rotate_if_needed()
            self.writer.writerow(row)
            self.flush()
            self.last_written_stamp = msg.stamp
        except OSError as exc:
            rospy.logwarn_throttle(10.0, "Battery voltage log write failed for %s: %s", self.log_path, exc)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    BatteryVoltageLogger().run()
