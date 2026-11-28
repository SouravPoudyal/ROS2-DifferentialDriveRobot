#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from rclpy.time import Time
import math


LEFT_JOINT = "wheel_left_joint"
RIGHT_JOINT = "wheel_right_joint"

class SimpleController(Node):
    # Subclasses can override these (the noisy controller does).
    ODOM_TOPIC = "bumperbot_controller/odom"
    TF_CHILD_FRAME = "base_footprint"

    def __init__(self):
        super().__init__("simple_controller")
        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.1402)
        # False: compute odometry only, do not command the wheels.
        self.declare_parameter("drive_wheels", True)
        self.declare_parameter("odom_topic", self.ODOM_TOPIC)
        self.declare_parameter("tf_child_frame", self.TF_CHILD_FRAME)
        self.declare_parameter("odom_child_frame", "base_footprint")
        self.declare_parameter("enable_odom_tf", True)
        self.declare_parameter(
            "pose_covariance_diagonal",
            [1.0e-3, 1.0e-3, 1.0e6, 1.0e6, 1.0e6, 1.0e-2],
        )
        self.declare_parameter(
            "twist_covariance_diagonal",
            [4.0e-3, 1.0e-3, 1.0e6, 1.0e6, 1.0e6, 4.0e-2],
        )
        self.r = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.L = self.get_parameter("wheel_separation").get_parameter_value().double_value
        self.drive_wheels = self.get_parameter("drive_wheels").value
        self.get_logger().info(f"Wheel radius: {self.r}, Wheel separation: {self.L}")

        # [v, w] = A @ [w_right, w_left]
        conversion = np.array([[self.r / 2, self.r / 2], 
                               [self.r / self.L, -self.r / self.L]])

        self.left_prev = None
        self.right_prev = None
        self.prev_time = None
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.inv_conversion = np.linalg.inv(conversion)

        self.wheel_cmd_pub = None
        if self.drive_wheels:
            self.wheel_cmd_pub = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
            self.vel_sub = self.create_subscription(Twist, "bumperbot_controller/cmd_vel", self.vel_callback, 10)

        self.odom_pub = self.create_publisher(Odometry, self.get_parameter("odom_topic").value, 10)
        self.joint_sub = self.create_subscription(JointState, "joint_states", self.joint_callback, 10)

        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = self.get_parameter("odom_child_frame").value
        self.enable_odom_tf = self.get_parameter("enable_odom_tf").value
        pose_cov = self.get_parameter("pose_covariance_diagonal").value
        twist_cov = self.get_parameter("twist_covariance_diagonal").value

        for i, idx in enumerate((0, 7, 14, 21, 28, 35)):
            self.odom_msg.pose.covariance[idx] = float(pose_cov[i])
            self.odom_msg.twist.covariance[idx] = float(twist_cov[i])

        self.br = TransformBroadcaster(self)
        self.tf_msg = TransformStamped()
        self.tf_msg.header.frame_id = "odom"
        self.tf_msg.child_frame_id = self.get_parameter("tf_child_frame").value

    def vel_callback(self, msg: Twist):
        if not self.drive_wheels:
            return
        robot_speed = np.array([[msg.linear.x], [msg.angular.z]])
        wheel_speed = self.inv_conversion @ robot_speed
        wheel_cmd = Float64MultiArray()
        wheel_cmd.data = [float(wheel_speed[1, 0]), float(wheel_speed[0, 0])]
        self.wheel_cmd_pub.publish(wheel_cmd)

    def joint_callback(self, msg):
        try:
            left = msg.position[msg.name.index(LEFT_JOINT)]
            right = msg.position[msg.name.index(RIGHT_JOINT)]
        except ValueError:
            return

        stamp = Time.from_msg(msg.header.stamp)
        if self.prev_time is None:  # first message: store the reference and publish the start pose
            self.left_prev, self.right_prev, self.prev_time = left, right, stamp
            self.publish_odom(msg.header.stamp, 0.0, 0.0)
            return

        dt = (stamp - self.prev_time).nanoseconds / 1e9
        if dt <= 0.0:
            return

        dp_left = left - self.left_prev
        dp_right = right - self.right_prev

        # Repeated telemetry sample: wait for a longer interval.
        # A stopped robot still gets an update every 0.1 s.
        if dp_left == 0.0 and dp_right == 0.0 and dt < 0.1:
            return

        self.left_prev, self.right_prev, self.prev_time = left, right, stamp

        d_s = self.r * (dp_right + dp_left) / 2
        d_theta = self.r * (dp_right - dp_left) / self.L

        self.x += d_s * math.cos(self.theta + d_theta / 2)
        self.y += d_s * math.sin(self.theta + d_theta / 2)
        self.theta += d_theta

        self.publish_odom(msg.header.stamp, d_s / dt, d_theta / dt)

    def publish_odom(self, stamp, linear, angular):
        qz = math.sin(self.theta / 2)
        qw = math.cos(self.theta / 2)

        self.odom_msg.header.stamp = stamp
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.orientation.z = qz
        self.odom_msg.pose.pose.orientation.w = qw
        self.odom_msg.twist.twist.linear.x = linear
        self.odom_msg.twist.twist.angular.z = angular
        self.odom_pub.publish(self.odom_msg)

        if self.enable_odom_tf:
            self.tf_msg.header.stamp = stamp
            self.tf_msg.transform.translation.x = self.x
            self.tf_msg.transform.translation.y = self.y
            self.tf_msg.transform.rotation.z = qz
            self.tf_msg.transform.rotation.w = qw
            self.br.sendTransform(self.tf_msg)


def main():
    rclpy.init()
    node = SimpleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

