#!/usr/bin/env python3
import json
import os

import rospy
import yaml
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String


SUPPORTED_PROFILES = {
    "ps3",
    "shield",
    "steam_stick",
    "steam_touch",
    "switch_pro",
    "xbox360",
}

START_MANUAL_MOWING_ACTION = "mower_logic:area_recording/start_manual_mowing"
STOP_MANUAL_MOWING_ACTION = "mower_logic:area_recording/stop_manual_mowing"


def zero_twist():
    msg = Twist()
    msg.linear.x = 0.0
    msg.angular.z = 0.0
    return msg


def button_pressed(joy, index):
    return 0 <= index < len(joy.buttons) and joy.buttons[index] != 0


def axis_value(joy, index):
    if 0 <= index < len(joy.axes):
        return joy.axes[index]
    return 0.0


def all_buttons_pressed(joy, buttons):
    return all(button_pressed(joy, button) for button in buttons)


class DirectGamepadMapper:
    def __init__(self):
        self.params_dir = rospy.get_param("~gamepad_params_dir", "")
        self.profile_param = rospy.get_param("~profile_param", "/direct_gamepad/profile")
        self.default_profile = rospy.get_param("~profile", "xbox360")
        self.linear_scale = rospy.get_param("~linear_scale", 1.0)
        self.angular_scale = rospy.get_param("~angular_scale", 1.0)

        if not rospy.has_param(self.profile_param):
            rospy.set_param(self.profile_param, self.default_profile)

        self.profile = None
        self.profile_config = None
        self.last_buttons = set()
        self.last_event_state = {}
        self.blade_hold_active = False
        self.last_drive_nonzero = False

        self.joy_vel_pub = rospy.Publisher("joy_vel", Twist, queue_size=1)
        self.record_polygon_pub = rospy.Publisher("record_polygon", Bool, queue_size=1)
        self.record_dock_pub = rospy.Publisher("record_dock", Bool, queue_size=1)
        self.record_mowing_pub = rospy.Publisher("record_mowing", Bool, queue_size=1)
        self.record_navigation_pub = rospy.Publisher("record_navigation", Bool, queue_size=1)
        self.direct_action_pub = rospy.Publisher("direct_action", String, queue_size=1)
        self.status_pub = rospy.Publisher("~status", String, queue_size=1, latch=True)

        self.joy_sub = rospy.Subscriber("joy", Joy, self.joy_received, queue_size=10)
        self.profile_timer = rospy.Timer(rospy.Duration(1.0), self.refresh_profile)
        self.status_timer = rospy.Timer(rospy.Duration(1.0), self.publish_status)
        self.refresh_profile(None)

    def load_yaml_profile(self, profile):
        path = os.path.join(self.params_dir, "{}.yaml".format(profile))
        if not path or not os.path.exists(path):
            return None
        with open(path, "r", encoding="utf-8") as profile_file:
            return yaml.safe_load(profile_file) or {}

    def refresh_profile(self, _event):
        requested = rospy.get_param(self.profile_param, self.default_profile)
        if requested not in SUPPORTED_PROFILES:
            rospy.logwarn_throttle(5.0, "Unsupported direct gamepad profile '%s', falling back to xbox360", requested)
            requested = "xbox360"
            rospy.set_param(self.profile_param, requested)

        if requested == self.profile:
            return

        yaml_config = None if requested == "xbox360" else self.load_yaml_profile(requested)
        self.profile = requested
        self.profile_config = self.build_profile(requested, yaml_config)
        self.last_event_state = {}
        self.publish_zero()
        rospy.loginfo("Direct gamepad profile active: %s", self.profile)
        self.publish_status()

    def build_profile(self, profile, yaml_config):
        if profile == "xbox360":
            return {
                "move": {
                    "deadman_buttons": [0],
                    "axis_mappings": [
                        {"axis": 1, "target": "linear.x", "scale": 0.5},
                        {"axis": 0, "target": "angular.z", "scale": 1.5},
                    ],
                    "turbo_button": 4,
                    "turbo_axis_mappings": [
                        {"axis": 1, "target": "linear.x", "scale": 1.0},
                        {"axis": 0, "target": "angular.z", "scale": 3.0},
                    ],
                },
                "events": {
                    "record_polygon": {"buttons": [1]},
                    "record_dock": {"buttons": [2]},
                    "record_navigation": {"buttons": [3], "axis": 7, "axis_min": 0.5},
                    "record_mowing": {"buttons": [3], "axis": 7, "axis_max": -0.5},
                },
                "blade_hold_buttons": [4, 5],
            }

        teleop = (yaml_config or {}).get("teleop", {})
        move = teleop.get("move", {})
        events = {}
        for name in ("record_polygon", "record_dock", "record_mowing", "record_navigation"):
            action = teleop.get(name)
            if not action:
                continue
            events[name] = {"buttons": list(action.get("deadman_buttons", []))}

        blade_hold_buttons = {
            "ps3": [4, 5],
            "shield": [4, 5],
            "steam_stick": [6, 7],
            "steam_touch": [6, 7],
            "switch_pro": [5, 6],
        }.get(profile, [4, 5])

        return {
            "move": {
                "deadman_buttons": list(move.get("deadman_buttons", [])),
                "axis_mappings": list(move.get("axis_mappings", [])),
            },
            "events": events,
            "blade_hold_buttons": blade_hold_buttons,
        }

    def joy_received(self, joy):
        self.refresh_profile(None)
        self.publish_drive(joy)
        self.publish_events(joy)
        self.publish_blade_hold(joy)

    def publish_drive(self, joy):
        move = self.profile_config.get("move", {})
        deadman_buttons = move.get("deadman_buttons", [])
        enabled = not deadman_buttons or all_buttons_pressed(joy, deadman_buttons)
        mappings = move.get("axis_mappings", [])

        if enabled and move.get("turbo_button") is not None and button_pressed(joy, move["turbo_button"]):
            mappings = move.get("turbo_axis_mappings", mappings)

        twist = zero_twist()
        if enabled:
            for mapping in mappings:
                value = axis_value(joy, int(mapping.get("axis", 0))) * float(mapping.get("scale", 1.0))
                target = mapping.get("target")
                if target == "linear.x":
                    twist.linear.x = value * self.linear_scale
                elif target == "angular.z":
                    twist.angular.z = value * self.angular_scale

        nonzero = abs(twist.linear.x) > 0.001 or abs(twist.angular.z) > 0.001
        if nonzero or self.last_drive_nonzero:
            self.joy_vel_pub.publish(twist)
        self.last_drive_nonzero = nonzero

    def publish_events(self, joy):
        publishers = {
            "record_polygon": self.record_polygon_pub,
            "record_dock": self.record_dock_pub,
            "record_mowing": self.record_mowing_pub,
            "record_navigation": self.record_navigation_pub,
        }
        for name, event in self.profile_config.get("events", {}).items():
            active = all_buttons_pressed(joy, event.get("buttons", []))
            if "axis" in event:
                value = axis_value(joy, int(event["axis"]))
                if "axis_min" in event:
                    active = active and value > float(event["axis_min"])
                if "axis_max" in event:
                    active = active and value < float(event["axis_max"])

            if active and not self.last_event_state.get(name, False):
                publishers[name].publish(Bool(data=True))
            self.last_event_state[name] = active

    def publish_blade_hold(self, joy):
        buttons = self.profile_config.get("blade_hold_buttons", [])
        active = bool(buttons) and all_buttons_pressed(joy, buttons)
        if active == self.blade_hold_active:
            return

        self.blade_hold_active = active
        action = START_MANUAL_MOWING_ACTION if active else STOP_MANUAL_MOWING_ACTION
        self.direct_action_pub.publish(String(data=action))

    def publish_zero(self):
        self.joy_vel_pub.publish(zero_twist())
        self.last_drive_nonzero = False
        if self.blade_hold_active:
            self.blade_hold_active = False
            self.direct_action_pub.publish(String(data=STOP_MANUAL_MOWING_ACTION))

    def publish_status(self, _event=None):
        payload = {
            "profile": self.profile,
            "supported_profiles": sorted(SUPPORTED_PROFILES),
            "blade_hold_active": self.blade_hold_active,
            "last_drive_nonzero": self.last_drive_nonzero,
        }
        self.status_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def shutdown(self):
        self.publish_zero()


if __name__ == "__main__":
    rospy.init_node("direct_gamepad_mapper")
    mapper = DirectGamepadMapper()
    rospy.on_shutdown(mapper.shutdown)
    rospy.spin()
