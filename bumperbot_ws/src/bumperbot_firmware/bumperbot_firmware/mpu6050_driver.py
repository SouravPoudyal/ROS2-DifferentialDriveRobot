#!/usr/bin/env python3
import math
import struct
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from smbus2 import SMBus

G = 9.80665
PWR_MGMT_1, SMPLRT_DIV, CONFIG = 0x6B, 0x19, 0x1A
GYRO_CONFIG, ACCEL_CONFIG, ACCEL_XOUT_H = 0x1B, 0x1C, 0x3B
ACCEL_LSB_PER_G = 16384.0   # +-2 g
GYRO_LSB_PER_DPS = 131.0    # +-250 dps


class Mpu6050Driver(Node):
    def __init__(self):
        super().__init__("mpu6050_driver")
        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("address", 0x68)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("calibration_s", 2.0)
        # Robot axis <- board axis. Default: board X forward, Z down.
        self.declare_parameter("axis_map", ["+x", "-y", "-z"])
        # Applied in BOARD frame: a_corrected = (raw_g - offset) * scale
        self.declare_parameter("accel_offset_g", [0.0, 0.0, 0.0])
        self.declare_parameter("accel_scale", [1.0, 1.0, 1.0])

        p = self.get_parameter
        self.addr = int(p("address").value)
        self.frame_id = p("frame_id").value
        self.axis_map = self._parse_map(list(p("axis_map").value))
        self.a_off = list(p("accel_offset_g").value)
        self.a_scale = list(p("accel_scale").value)

        self.bus = SMBus(int(p("i2c_bus").value))
        self._init_chip()
        self.gyro_bias = self._calibrate_gyro(float(p("calibration_s").value))

        self.pub = self.create_publisher(Imu, "imu", 10)
        self.errors = 0
        self.create_timer(1.0 / float(p("rate_hz").value), self._tick)
        self.get_logger().info(
            f"MPU6050 ready, gyro bias (dps, board frame) = "
            f"{[round(b, 3) for b in self.gyro_bias]}")

    @staticmethod
    def _parse_map(m):
        idx = {"x": 0, "y": 1, "z": 2}
        out = []
        for s in m:
            sign = -1.0 if s.startswith("-") else 1.0
            out.append((idx[s.strip("+-")], sign))
        return out

    def _init_chip(self):
        self.bus.write_byte_data(self.addr, PWR_MGMT_1, 0x01)  # wake, PLL clock
        time.sleep(0.1)
        self.bus.write_byte_data(self.addr, CONFIG, 0x03)       # DLPF ~44 Hz
        self.bus.write_byte_data(self.addr, SMPLRT_DIV, 0x09)   # 100 Hz
        self.bus.write_byte_data(self.addr, GYRO_CONFIG, 0x00)  # +-250 dps
        self.bus.write_byte_data(self.addr, ACCEL_CONFIG, 0x00) # +-2 g

    def _read_raw(self):
        d = bytes(self.bus.read_i2c_block_data(self.addr, ACCEL_XOUT_H, 14))
        ax, ay, az, _t, gx, gy, gz = struct.unpack(">7h", d)
        return ([ax / ACCEL_LSB_PER_G, ay / ACCEL_LSB_PER_G, az / ACCEL_LSB_PER_G],
                [gx / GYRO_LSB_PER_DPS, gy / GYRO_LSB_PER_DPS, gz / GYRO_LSB_PER_DPS])

    def _calibrate_gyro(self, seconds):
        self.get_logger().info(
            f"Calibrating gyro for {seconds:.1f} s - keep the robot still")
        samples, t_end = [], time.time() + seconds
        while time.time() < t_end:
            try:
                samples.append(self._read_raw()[1])
            except OSError:
                pass
            time.sleep(0.01)
        if not samples:
            raise RuntimeError("No IMU samples during calibration")
        n = len(samples)
        mean = [sum(s[i] for s in samples) / n for i in range(3)]
        std = [math.sqrt(sum((s[i] - mean[i]) ** 2 for s in samples) / n)
               for i in range(3)]
        if max(std) > 0.5:
            self.get_logger().warn(
                f"Gyro std {max(std):.2f} dps during calibration: "
                "robot may have moved, bias could be wrong")
        return mean

    def _remap(self, v):
        return [sign * v[i] for i, sign in self.axis_map]

    def _tick(self):
        try:
            acc_g, gyro_dps = self._read_raw()
        except OSError:
            self.errors += 1
            if self.errors % 50 == 1:
                self.get_logger().warn(f"I2C read failed ({self.errors} total)")
            return

        acc_g = [(acc_g[i] - self.a_off[i]) * self.a_scale[i] for i in range(3)]
        gyro_dps = [gyro_dps[i] - self.gyro_bias[i] for i in range(3)]
        a = self._remap(acc_g)
        w = self._remap(gyro_dps)

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.orientation_covariance[0] = -1.0      # no orientation estimate
        msg.angular_velocity.x = math.radians(w[0])
        msg.angular_velocity.y = math.radians(w[1])
        msg.angular_velocity.z = math.radians(w[2])
        msg.linear_acceleration.x = a[0] * G
        msg.linear_acceleration.y = a[1] * G
        msg.linear_acceleration.z = a[2] * G
        for i in (0, 4, 8):
            msg.angular_velocity_covariance[i] = 1e-4
            msg.linear_acceleration_covariance[i] = 4e-2
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = Mpu6050Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.bus.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()