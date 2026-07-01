#!/usr/bin/env python3

import math

import rospy
from sensor_msgs.msg import Temperature


class PiTemperatureNode:
    def __init__(self):
        rospy.init_node("pi_temperature")

        self.path = rospy.get_param("~thermal_zone_path", "/sys/class/thermal/thermal_zone0/temp")
        self.frame_id = rospy.get_param("~frame_id", "raspberry_pi")
        self.rate_hz = float(rospy.get_param("~rate_hz", 1.0))
        self.publisher = rospy.Publisher("temperature", Temperature, queue_size=10)

        rospy.loginfo("Pi temperature publisher reading %s at %.2f Hz", self.path, self.rate_hz)

    def read_temperature(self):
        with open(self.path, "r", encoding="ascii") as temp_file:
            raw = temp_file.read().strip()

        value = float(raw)
        if abs(value) > 1000.0:
            value /= 1000.0
        return value

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            try:
                temperature_c = self.read_temperature()
            except OSError as exc:
                rospy.logwarn_throttle(30.0, "Pi temperature read failed from %s: %s", self.path, exc)
                rate.sleep()
                continue
            except ValueError as exc:
                rospy.logwarn_throttle(30.0, "Pi temperature parse failed from %s: %s", self.path, exc)
                rate.sleep()
                continue

            if not math.isfinite(temperature_c):
                rospy.logwarn_throttle(30.0, "Pi temperature was not finite: %r", temperature_c)
                rate.sleep()
                continue

            msg = Temperature()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.frame_id
            msg.temperature = temperature_c
            msg.variance = 0.0
            self.publisher.publish(msg)

            rate.sleep()


if __name__ == "__main__":
    PiTemperatureNode().run()
