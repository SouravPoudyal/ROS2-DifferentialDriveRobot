#!/usr/bin/env python3
import copy
import math
import random

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from bumperbot_controller.simple_controller import (
    SimpleController, LEFT_JOINT, RIGHT_JOINT)


class NoisyController(SimpleController):
    """SimpleController with a real-robot-like wheel response and noisy, sampled encoders."""

    def __init__(self):
        super().__init__()

        # --- reading noise ---
        self.declare_parameter("position_noise_std", 0.002)    # rad, jitter per sample
        self.declare_parameter("increment_noise_ratio", 0.01)  # relative std of each increment
        self.declare_parameter("quantization_cpr", 1320.0)     # 0 = off
        self.declare_parameter("seed", -1)                     # -1 = random

        # --- telemetry timing ---
        self.declare_parameter("sample_rate_hz", 20.0)         # 0 = update on every message
        self.declare_parameter("sample_jitter_s", 0.005)       # std of the sample period

        # --- velocity estimation ---
        self.declare_parameter("vel_filter_alpha", 0.6)        # per sample

        # --- wheel response (command side) ---
        self.declare_parameter("command_rate_hz", 50.0)
        self.declare_parameter("slew_limit", 50.0)             # rad/s^2, 0 = off
        self.declare_parameter("lag_tau", 0.15)                # s, first-order lag, 0 = off
        self.declare_parameter("min_speed", 1.0)               # rad/s, below this -> 0
        self.declare_parameter("left_gain", 1.0)               # wheel mismatch tests
        self.declare_parameter("right_gain", 1.0)

        p = lambda n: self.get_parameter(n).value
        self.pos_std = p("position_noise_std")
        self.incr_ratio = p("increment_noise_ratio")
        self.cpr = p("quantization_cpr")
        seed = p("seed")
        self.rng = random.Random(None if seed < 0 else seed)

        rate = p("sample_rate_hz")
        self.sample_period = 1.0 / rate if rate > 0.0 else 0.0
        self.sample_jitter = p("sample_jitter_s")
        self.next_sample = None

        self.slew = p("slew_limit")
        self.lag_tau = p("lag_tau")
        self.min_speed = p("min_speed")
        self.gains = [p("left_gain"), p("right_gain")]  # [left, right]

        self.true_prev = {}
        self.noisy_pos = {}
        self.held = {}

        # Filtered velocities and previous sampled values for differentiation.
        self.vel_filt = {LEFT_JOINT: 0.0, RIGHT_JOINT: 0.0}
        self.last_held = {}
        self.last_sample_t = None

        self.noisy_pub = self.create_publisher(
            JointState, "noisy_joint_states", 10)

        self.cmd_target = [0.0, 0.0]
        self.cmd_ramped = [0.0, 0.0]
        self.cmd_filtered = [0.0, 0.0]
        self.last_cmd_time = self.get_clock().now()
        self.create_timer(1.0 / p("command_rate_hz"), self.command_step)

        self.get_logger().info(
            f"NOISY controller: pos_std={self.pos_std} incr={self.incr_ratio} "
            f"cpr={self.cpr} sample={rate} Hz jitter={self.sample_jitter}s | "
            f"slew={self.slew} lag={self.lag_tau}s min={self.min_speed} "
            f"gains={self.gains} seed={seed}")

    # ---------- command side: wheel response ----------
    def vel_callback(self, msg: Twist):
        robot_speed = np.array([[msg.linear.x], [msg.angular.z]])
        ws = self.inv_conversion @ robot_speed          # [[right], [left]]
        target = [float(ws[1, 0]), float(ws[0, 0])]     # [left, right]

        for k in range(2):
            if abs(target[k]) < self.min_speed:
                target[k] = 0.0
            target[k] *= self.gains[k]

        self.cmd_target = target

    def command_step(self):
        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds / 1e9
        self.last_cmd_time = now

        if dt <= 0.0:
            return

        for k in range(2):
            if self.slew > 0.0:
                step = self.slew * dt
                err = self.cmd_target[k] - self.cmd_ramped[k]
                self.cmd_ramped[k] += max(-step, min(step, err))
            else:
                self.cmd_ramped[k] = self.cmd_target[k]

            if self.lag_tau > 0.0:
                alpha = 1.0 - math.exp(-dt / self.lag_tau)
                self.cmd_filtered[k] += alpha * (
                    self.cmd_ramped[k] - self.cmd_filtered[k])
            else:
                self.cmd_filtered[k] = self.cmd_ramped[k]

        out = Float64MultiArray()
        out.data = [float(v) for v in self.cmd_filtered]  # [left, right]
        self.wheel_cmd_pub.publish(out)

    # ---------- reading side: noisy, sampled encoders ----------
    def joint_callback(self, msg):
        try:
            idx = {
                LEFT_JOINT: msg.name.index(LEFT_JOINT),
                RIGHT_JOINT: msg.name.index(RIGHT_JOINT),
            }
        except ValueError:
            return

        stamp = Time.from_msg(msg.header.stamp)

        if (
            self.sample_period <= 0.0
            or self.next_sample is None
            or stamp >= self.next_sample
        ):
            for name, i in idx.items():
                true_pos = msg.position[i]
                prev = self.true_prev.get(name)

                if prev is None:
                    self.noisy_pos[name] = true_pos
                else:
                    dp = true_pos - prev
                    self.noisy_pos[name] += dp * (
                        1.0 + self.rng.gauss(0.0, self.incr_ratio))

                self.true_prev[name] = true_pos

                value = self.noisy_pos[name] + self.rng.gauss(
                    0.0, self.pos_std)

                if self.cpr > 0.0:
                    step = 2.0 * math.pi / self.cpr
                    value = round(value / step) * step

                self.held[name] = value

            # Estimate velocity from the sampled, noisy positions.
            t = stamp.nanoseconds / 1e9
            if self.last_sample_t is not None and t > self.last_sample_t:
                dt_s = t - self.last_sample_t
                alpha = float(p_value(self, "vel_filter_alpha"))

                for name in (LEFT_JOINT, RIGHT_JOINT):
                    raw_velocity = (
                        self.held[name] - self.last_held[name]
                    ) / dt_s
                    self.vel_filt[name] += alpha * (
                        raw_velocity - self.vel_filt[name])

            self.last_held = dict(self.held)
            self.last_sample_t = t

            if self.sample_period > 0.0:
                period = max(
                    0.005,
                    self.sample_period
                    + self.rng.gauss(0.0, self.sample_jitter))
                self.next_sample = stamp + Duration(seconds=period)

        noisy = copy.deepcopy(msg)

        for name, i in idx.items():
            noisy.position[i] = self.held[name]
            noisy.velocity[i] = self.vel_filt[name]

        self.noisy_pub.publish(noisy)
        super().joint_callback(noisy)


def p_value(node, parameter_name):
    """Return a parameter value as a float."""
    return float(node.get_parameter(parameter_name).value)


def main():
    rclpy.init()
    node = NoisyController()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
