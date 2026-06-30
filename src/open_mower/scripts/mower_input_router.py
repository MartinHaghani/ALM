#!/usr/bin/env python3
import json

import rospy
from geometry_msgs.msg import Twist
from mower_msgs.srv import SetManualInputSource, SetManualInputSourceResponse
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String


WEB_SOURCE = "web_gamepad"
DIRECT_SOURCE = "direct_bluetooth"
VALID_SOURCES = {WEB_SOURCE, DIRECT_SOURCE}
STOP_MANUAL_MOWING_ACTION = "mower_logic:area_recording/stop_manual_mowing"


def zero_twist():
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = 0.0
    return msg


class MowerInputRouter:
    def __init__(self):
        default_source = rospy.get_param("~default_source", WEB_SOURCE)
        if default_source not in VALID_SOURCES:
            rospy.logwarn("Invalid default manual input source '%s', using %s", default_source, WEB_SOURCE)
            default_source = WEB_SOURCE

        self.active_source = default_source
        self.last_web_stamp = rospy.Time(0)
        self.last_direct_stamp = rospy.Time(0)
        self.last_direct_joy_stamp = rospy.Time(0)
        self.last_switch_stamp = rospy.Time.now()
        self.direct_timeout_sec = float(rospy.get_param("~direct_timeout_sec", 2.0))
        self.direct_was_connected = False
        self.direct_controller_status = {}
        self.direct_mapper_status = {}

        self.joy_vel_pub = rospy.Publisher("/joy_vel", Twist, queue_size=1)
        self.record_polygon_pub = rospy.Publisher("/record_polygon", Bool, queue_size=1)
        self.record_dock_pub = rospy.Publisher("/record_dock", Bool, queue_size=1)
        self.record_mowing_pub = rospy.Publisher("/record_mowing", Bool, queue_size=1)
        self.record_navigation_pub = rospy.Publisher("/record_navigation", Bool, queue_size=1)
        self.action_pub = rospy.Publisher("/xbot/action", String, queue_size=1)
        self.status_pub = rospy.Publisher("/mower_input/status", String, queue_size=1, latch=True)

        rospy.Subscriber("/web_joy_vel", Twist, self.web_joy_vel_received, queue_size=1)
        rospy.Subscriber("/direct_joy_vel", Twist, self.direct_joy_vel_received, queue_size=1)
        rospy.Subscriber("/direct_joy", Joy, self.direct_joy_seen, queue_size=1)
        rospy.Subscriber("/direct_record_polygon", Bool, self.direct_record_polygon_received, queue_size=10)
        rospy.Subscriber("/direct_record_dock", Bool, self.direct_record_dock_received, queue_size=10)
        rospy.Subscriber("/direct_record_mowing", Bool, self.direct_record_mowing_received, queue_size=10)
        rospy.Subscriber("/direct_record_navigation", Bool, self.direct_record_navigation_received, queue_size=10)
        rospy.Subscriber("/direct_action", String, self.direct_action_received, queue_size=10)
        rospy.Subscriber("/bluetooth_gamepad/status", String, self.bluetooth_status_received, queue_size=1)
        rospy.Subscriber("/direct_gamepad_mapper/status", String, self.direct_mapper_status_received, queue_size=1)

        rospy.Service("/mower_input/set_source", SetManualInputSource, self.set_source)
        self.status_timer = rospy.Timer(rospy.Duration(0.25), self.publish_status)
        self.publish_status()

    def set_source(self, req):
        requested = req.source.strip()
        if requested not in VALID_SOURCES:
            return SetManualInputSourceResponse(
                success=False,
                message="source must be web_gamepad or direct_bluetooth",
                active_source=self.active_source,
            )

        if requested != self.active_source:
            self.publish_zero()
            self.publish_blade_safety_release()
            self.active_source = requested
            self.last_switch_stamp = rospy.Time.now()
            rospy.loginfo("Manual input source switched to %s", self.active_source)
            self.publish_status()

        return SetManualInputSourceResponse(
            success=True,
            message="manual input source set",
            active_source=self.active_source,
        )

    def publish_zero(self):
        self.joy_vel_pub.publish(zero_twist())

    def publish_blade_safety_release(self):
        self.action_pub.publish(String(data=STOP_MANUAL_MOWING_ACTION))

    def web_joy_vel_received(self, msg):
        self.last_web_stamp = rospy.Time.now()
        if self.active_source == WEB_SOURCE:
            self.joy_vel_pub.publish(msg)

    def direct_joy_vel_received(self, msg):
        self.last_direct_stamp = rospy.Time.now()
        if self.active_source == DIRECT_SOURCE:
            self.joy_vel_pub.publish(msg)

    def direct_joy_seen(self, _msg):
        self.last_direct_joy_stamp = rospy.Time.now()

    def direct_record_polygon_received(self, msg):
        if self.active_source == DIRECT_SOURCE:
            self.record_polygon_pub.publish(msg)

    def direct_record_dock_received(self, msg):
        if self.active_source == DIRECT_SOURCE:
            self.record_dock_pub.publish(msg)

    def direct_record_mowing_received(self, msg):
        if self.active_source == DIRECT_SOURCE:
            self.record_mowing_pub.publish(msg)

    def direct_record_navigation_received(self, msg):
        if self.active_source == DIRECT_SOURCE:
            self.record_navigation_pub.publish(msg)

    def direct_action_received(self, msg):
        if self.active_source == DIRECT_SOURCE:
            self.action_pub.publish(msg)

    def bluetooth_status_received(self, msg):
        try:
            self.direct_controller_status = json.loads(msg.data)
        except (TypeError, ValueError):
            self.direct_controller_status = {}

    def direct_mapper_status_received(self, msg):
        try:
            self.direct_mapper_status = json.loads(msg.data)
        except (TypeError, ValueError):
            self.direct_mapper_status = {}

    @staticmethod
    def age_ms(stamp, now):
        if stamp == rospy.Time(0):
            return None
        return int(max(0.0, (now - stamp).to_sec() * 1000.0))

    def publish_status(self, _event=None):
        now = rospy.Time.now()
        direct_joy_age_ms = self.age_ms(self.last_direct_joy_stamp, now)
        web_command_age_ms = self.age_ms(self.last_web_stamp, now)
        direct_command_age_ms = self.age_ms(self.last_direct_stamp, now)
        direct_connected = direct_joy_age_ms is not None and direct_joy_age_ms < int(self.direct_timeout_sec * 1000)

        if self.active_source == DIRECT_SOURCE and self.direct_was_connected and not direct_connected:
            rospy.logwarn("Direct Bluetooth controller input timed out; publishing zero manual drive")
            self.publish_zero()
            self.publish_blade_safety_release()

        self.direct_was_connected = direct_connected

        payload = {
            "active_source": self.active_source,
            "selected_source": self.active_source,
            "available_sources": sorted(VALID_SOURCES),
            "web_command_age_ms": web_command_age_ms,
            "direct_command_age_ms": direct_command_age_ms,
            "direct_joy_age_ms": direct_joy_age_ms,
            "direct_connected": direct_connected,
            "last_switch_time": self.last_switch_stamp.to_sec(),
            "direct_profile": self.direct_mapper_status.get("profile"),
            "direct_blade_hold_active": bool(self.direct_mapper_status.get("blade_hold_active", False)),
            "bluetooth": self.direct_controller_status,
        }
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def shutdown(self):
        self.publish_zero()
        self.publish_blade_safety_release()


if __name__ == "__main__":
    rospy.init_node("mower_input_router")
    router = MowerInputRouter()
    rospy.on_shutdown(router.shutdown)
    rospy.spin()
