#!/usr/bin/env python3

import argparse
import fcntl
import math
import struct
import time


I2C_SLAVE = 0x0703

WHO_AM_I = 0x0F
CTRL1_XL = 0x10
CTRL2_G = 0x11
CTRL3_C = 0x12
OUTX_L_G = 0x22

EXPECTED_WHO_AM_I = 0x6C
STANDARD_GRAVITY = 9.80665
ACCEL_SCALE_4G = 0.122e-3 * STANDARD_GRAVITY
GYRO_SCALE_500DPS = 17.50e-3 * math.pi / 180.0


class I2CDevice:
    def __init__(self, bus, address):
        self.path = "/dev/i2c-{}".format(bus)
        self.address = address
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


def parse_address(value):
    return int(str(value), 0)


def read_sample(device):
    raw = struct.unpack("<hhhhhh", device.read_registers(OUTX_L_G, 12))
    gyro = tuple(value * GYRO_SCALE_500DPS for value in raw[0:3])
    accel = tuple(value * ACCEL_SCALE_4G for value in raw[3:6])
    return accel, gyro


def main():
    parser = argparse.ArgumentParser(description="Read a SparkFun LSM6DSO over Raspberry Pi I2C.")
    parser.add_argument("--bus", type=int, default=1, help="I2C bus number; Raspberry Pi header pins 3/5 use bus 1")
    parser.add_argument("--address", default="0x6B", help="7-bit I2C address, usually 0x6B or 0x6A")
    parser.add_argument("--samples", type=int, default=3, help="number of samples to print")
    args = parser.parse_args()

    address = parse_address(args.address)
    device = I2CDevice(args.bus, address)

    who_am_i = device.read_register(WHO_AM_I)
    print("LSM6DSO WHO_AM_I at /dev/i2c-{} 0x{:02x}: 0x{:02x}".format(args.bus, address, who_am_i))
    if who_am_i != EXPECTED_WHO_AM_I:
        raise SystemExit("expected WHO_AM_I 0x{:02x}; check power, SDA/SCL, address jumper, and ground".format(EXPECTED_WHO_AM_I))

    device.write_register(CTRL3_C, 0x44)
    device.write_register(CTRL1_XL, 0x48)
    device.write_register(CTRL2_G, 0x44)

    for _ in range(args.samples):
        accel, gyro = read_sample(device)
        print(
            "accel m/s^2: x={:+.3f} y={:+.3f} z={:+.3f} | gyro rad/s: x={:+.4f} y={:+.4f} z={:+.4f}".format(
                accel[0],
                accel[1],
                accel[2],
                gyro[0],
                gyro[1],
                gyro[2],
            )
        )
        time.sleep(0.2)


if __name__ == "__main__":
    main()
