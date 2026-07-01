#!/usr/bin/env python3

import fcntl
import math
import re
import struct

import rospy
from sensor_msgs.msg import Imu, Temperature


I2C_SLAVE = 0x0703

WHO_AM_I = 0x0F
CTRL1_XL = 0x10
CTRL2_G = 0x11
CTRL3_C = 0x12
OUT_TEMP_L = 0x20
OUTX_L_G = 0x22

EXPECTED_WHO_AM_I = 0x6C
STANDARD_GRAVITY = 9.80665

ACCEL_RANGES = {
    2: (0x00, 0.061e-3 * STANDARD_GRAVITY),
    4: (0x08, 0.122e-3 * STANDARD_GRAVITY),
    8: (0x0C, 0.244e-3 * STANDARD_GRAVITY),
    16: (0x04, 0.488e-3 * STANDARD_GRAVITY),
}

GYRO_RANGES = {
    125: (0x02, 4.375e-3 * math.pi / 180.0),
    250: (0x00, 8.75e-3 * math.pi / 180.0),
    500: (0x04, 17.50e-3 * math.pi / 180.0),
    1000: (0x08, 35.00e-3 * math.pi / 180.0),
    2000: (0x0C, 70.00e-3 * math.pi / 180.0),
}


class LSM6DSO:
    def __init__(self, bus, address, accel_range_g, gyro_range_dps):
        self.path = "/dev/i2c-{}".format(bus)
        self.address = address
        self.accel_control, self.accel_scale = ACCEL_RANGES[accel_range_g]
        self.gyro_control, self.gyro_scale = GYRO_RANGES[gyro_range_dps]
        self.device = open(self.path, "r+b", buffering=0)
        fcntl.ioctl(self.device, I2C_SLAVE, self.address)

    def read_registers(self, register, length):
        self.device.write(bytes([register]))
        data = self.device.read(length)
        if len(data) != length:
            raise OSError("short I2C read from register 0x{:02x}".format(register))
        return data

    def read_register(self, register):
        return self.read_registers(register, 1)[0]

    def write_register(self, register, value):
        self.device.write(bytes([register, value & 0xFF]))

    def configure(self):
        who_am_i = self.read_register(WHO_AM_I)
        if who_am_i != EXPECTED_WHO_AM_I:
            raise RuntimeError(
                "LSM6DSO WHO_AM_I mismatch at 0x{:02x}: expected 0x{:02x}, got 0x{:02x}".format(
                    self.address,
                    EXPECTED_WHO_AM_I,
                    who_am_i,
                )
            )

        # BDU keeps high/low output bytes coherent; IF_INC enables burst reads.
        self.write_register(CTRL3_C, 0x44)

        # 0x40 selects 104 Hz ODR. The low bits select the configured full scale.
        self.write_register(CTRL1_XL, 0x40 | self.accel_control)
        self.write_register(CTRL2_G, 0x40 | self.gyro_control)

    def read_sample(self):
        raw = struct.unpack("<hhhhhhh", self.read_registers(OUT_TEMP_L, 14))
        temperature_c = 25.0 + raw[0] / 256.0
        gyro = tuple(value * self.gyro_scale for value in raw[1:4])
        accel = tuple(value * self.accel_scale for value in raw[4:7])
        return accel, gyro, temperature_c


def parse_address(value):
    if isinstance(value, int):
        return value
    return int(str(value), 0)


def parse_axis_config(axis_config):
    tokens = re.findall(r"([+-])([XYZxyz])", axis_config)
    if len(tokens) != 3:
        raise ValueError("axis_config must contain three signed axes, for example '+X+Y+Z'")

    axes = [axis.upper() for _, axis in tokens]
    if sorted(axes) != ["X", "Y", "Z"]:
        raise ValueError("axis_config must use X, Y, and Z exactly once")

    mapping = []
    for sign, axis in tokens:
        mapping.append((1.0 if sign == "+" else -1.0, axis.upper()))
    return mapping


def remap(values, mapping):
    source = {"X": values[0], "Y": values[1], "Z": values[2]}
    return tuple(sign * source[axis] for sign, axis in mapping)


class LSM6DSOImuNode:
    def __init__(self):
        rospy.init_node("lsm6dso_imu")

        bus = int(rospy.get_param("~bus", 1))
        address = parse_address(rospy.get_param("~address", "0x6B"))
        accel_range_g = int(rospy.get_param("~accel_range_g", 4))
        gyro_range_dps = int(rospy.get_param("~gyro_range_dps", 500))

        if accel_range_g not in ACCEL_RANGES:
            raise ValueError("accel_range_g must be one of {}".format(sorted(ACCEL_RANGES)))
        if gyro_range_dps not in GYRO_RANGES:
            raise ValueError("gyro_range_dps must be one of {}".format(sorted(GYRO_RANGES)))

        self.frame_id = rospy.get_param("~frame_id", "base_link")
        self.axis_mapping = parse_axis_config(rospy.get_param("~axis_config", "+X+Y+Z"))
        self.rate_hz = float(rospy.get_param("~rate_hz", 104.0))

        self.imu = LSM6DSO(bus, address, accel_range_g, gyro_range_dps)
        self.imu.configure()

        self.publisher = rospy.Publisher("imu", Imu, queue_size=10)
        self.temperature_publisher = rospy.Publisher("temperature", Temperature, queue_size=10)
        rospy.loginfo(
            "LSM6DSO IMU active on /dev/i2c-%s address 0x%02x, accel +/- %sg, gyro +/- %s dps",
            bus,
            address,
            accel_range_g,
            gyro_range_dps,
        )

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            accel, gyro, temperature_c = self.imu.read_sample()
            accel = remap(accel, self.axis_mapping)
            gyro = remap(gyro, self.axis_mapping)

            stamp = rospy.Time.now()
            msg = Imu()
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            msg.orientation_covariance[0] = -1.0
            msg.linear_acceleration.x = accel[0]
            msg.linear_acceleration.y = accel[1]
            msg.linear_acceleration.z = accel[2]
            msg.angular_velocity.x = gyro[0]
            msg.angular_velocity.y = gyro[1]
            msg.angular_velocity.z = gyro[2]
            self.publisher.publish(msg)

            temperature_msg = Temperature()
            temperature_msg.header.stamp = stamp
            temperature_msg.header.frame_id = self.frame_id
            temperature_msg.temperature = temperature_c
            temperature_msg.variance = 0.0
            self.temperature_publisher.publish(temperature_msg)

            rate.sleep()


if __name__ == "__main__":
    try:
        LSM6DSOImuNode().run()
    except Exception as exc:
        rospy.logerr("LSM6DSO IMU node failed: %s", exc)
        raise
