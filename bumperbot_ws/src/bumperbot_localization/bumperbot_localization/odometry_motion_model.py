#!/usr/bin/env python3
from math import sin, cos, atan2, sqrt, fabs, pi
import random

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseArray
from tf_transformations import euler_from_quaternion, quaternion_from_euler


def normalize(z):
    return atan2(sin(z), cos(z))


def angle_diff(a, b):
    """Smallest signed difference a - b, wrapped to [-pi, pi]."""
    a = normalize(a)
    b = normalize(b)
    d1 = a - b
    d2 = 2 * pi - fabs(d1)
    if d1 > 0:
        d2 *= -1.0
    if fabs(d1) < fabs(d2):
        return d1
    else:
        return d2


class OdometryMotionModel(Node):

    def __init__(self):
        super().__init__("odometry_motion_model")
        self.last_odom_x = 0.0
        self.last_odom_y = 0.0
        self.last_odom_theta = 0.0
        self.is_first_odom = True

        self.declare_parameter("alpha1", 0.1)
        self.declare_parameter("alpha2", 0.1)
        self.declare_parameter("alpha3", 0.1)
        self.declare_parameter("alpha4", 0.1)
        self.declare_parameter("nr_samples", 300)
        # Below this translation per message, the heading of the motion is
        # considered undefined and delta_rot1 is set to 0.
        self.declare_parameter("min_translation", 0.01)

        self.declare_parameter("odom_topic", "bumperbot_controller/odom")
        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value

        self.alpha1 = self.get_parameter("alpha1").get_parameter_value().double_value
        self.alpha2 = self.get_parameter("alpha2").get_parameter_value().double_value
        self.alpha3 = self.get_parameter("alpha3").get_parameter_value().double_value
        self.alpha4 = self.get_parameter("alpha4").get_parameter_value().double_value
        self.nr_samples = self.get_parameter("nr_samples").get_parameter_value().integer_value
        self.min_translation = (
            self.get_parameter("min_translation").get_parameter_value().double_value)

        if self.nr_samples <= 0:
            self.get_logger().fatal(
                f"Invalid number of samples requested: {self.nr_samples}! Exit.")
            raise ValueError("nr_samples must be positive")

        self.samples = PoseArray()
        self.samples.poses = [Pose() for _ in range(self.nr_samples)]
        for pose in self.samples.poses:
            pose.orientation.w = 1.0  # valid identity quaternion

        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odomCallback, 10)
        self.pose_array_pub = self.create_publisher(
            PoseArray, "odometry_motion/samples", 10)

    def odomCallback(self, odom):
        q = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
             odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
        roll, pitch, yaw = euler_from_quaternion(q)

        if self.is_first_odom:
            self.samples.header.frame_id = odom.header.frame_id
            self.last_odom_x = odom.pose.pose.position.x
            self.last_odom_y = odom.pose.pose.position.y
            self.last_odom_theta = yaw
            self.is_first_odom = False
            return

        # Odometry increment
        odom_x_increment = odom.pose.pose.position.x - self.last_odom_x
        odom_y_increment = odom.pose.pose.position.y - self.last_odom_y
        odom_theta_increment = angle_diff(yaw, self.last_odom_theta)

        # Motion model (Probabilistic Robotics, odometry model)
        delta_trasl = sqrt(odom_x_increment ** 2 + odom_y_increment ** 2)

        if delta_trasl < self.min_translation:
            delta_rot1 = 0.0
        else:
            delta_rot1 = angle_diff(
                atan2(odom_y_increment, odom_x_increment), self.last_odom_theta)

        delta_rot2 = angle_diff(odom_theta_increment, delta_rot1)

        # Standard deviations of the noise (random.gauss expects sigma)
        rot1_std = self.alpha1 * fabs(delta_rot1) + self.alpha2 * delta_trasl
        trasl_std = (self.alpha3 * delta_trasl
                     + self.alpha4 * (fabs(delta_rot1) + fabs(delta_rot2)))
        rot2_std = self.alpha1 * fabs(delta_rot2) + self.alpha2 * delta_trasl

        for sample in self.samples.poses:
            rot1_noise = random.gauss(0.0, rot1_std)
            rot2_noise = random.gauss(0.0, rot2_std)
            trasl_noise = random.gauss(0.0, trasl_std)

            delta_rot1_draw = angle_diff(delta_rot1, rot1_noise)
            delta_trans_draw = delta_trasl - trasl_noise
            delta_rot2_draw = angle_diff(delta_rot2, rot2_noise)

            sample_q = [sample.orientation.x, sample.orientation.y,
                        sample.orientation.z, sample.orientation.w]
            _, _, sample_yaw = euler_from_quaternion(sample_q)

            sample.position.x += delta_trans_draw * cos(sample_yaw + delta_rot1_draw)
            sample.position.y += delta_trans_draw * sin(sample_yaw + delta_rot1_draw)

            new_q = quaternion_from_euler(
                0.0, 0.0, sample_yaw + delta_rot1_draw + delta_rot2_draw)
            (sample.orientation.x, sample.orientation.y,
             sample.orientation.z, sample.orientation.w) = new_q

        # Update for the next iteration (once per message, outside the loop)
        self.last_odom_x = odom.pose.pose.position.x
        self.last_odom_y = odom.pose.pose.position.y
        self.last_odom_theta = yaw

        # Publish samples once per message
        self.samples.header.stamp = odom.header.stamp
        self.pose_array_pub.publish(self.samples)


def main():
    rclpy.init()

    odometry_motion_model = OdometryMotionModel()
    try:
        rclpy.spin(odometry_motion_model)
    finally:
        odometry_motion_model.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()