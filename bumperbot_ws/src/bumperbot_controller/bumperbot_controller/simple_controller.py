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
    def __init__(self):
        super().__init__("simple_controller")
        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.1402)
        self.r = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.L = self.get_parameter("wheel_separation").get_parameter_value().double_value
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
        self.wheel_cmd_pub = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
        self.odom_pub = self.create_publisher(Odometry, "bumperbot_controller/odom", 10)
        self.vel_sub = self.create_subscription(Twist, "bumperbot_controller/cmd_vel", self.vel_callback, 10)
        self.joint_sub = self.create_subscription(JointState, "joint_states", self.joint_callback, 10)

        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_footprint"

        self.br = TransformBroadcaster(self)
        self.tf_msg = TransformStamped()
        self.tf_msg.header.frame_id = "odom"
        self.tf_msg.child_frame_id = "base_footprint"

    def vel_callback(self, msg: Twist):
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
        if self.prev_time is None:  # first message: just store the reference
            self.left_prev, self.right_prev, self.prev_time = left, right, stamp
            return

        dt = (stamp - self.prev_time).nanoseconds / 1e9
        if dt <= 0.0:
            return

        dp_left = left - self.left_prev
        dp_right = right - self.right_prev
        self.left_prev, self.right_prev, self.prev_time = left, right, stamp

        d_s = self.r * (dp_right + dp_left) / 2
        d_theta = self.r * (dp_right - dp_left) / self.L
        linear = d_s / dt
        angular = d_theta / dt

        # Midpoint heading is more accurate than the end heading
        self.x += d_s * math.cos(self.theta + d_theta / 2)
        self.y += d_s * math.sin(self.theta + d_theta / 2)
        self.theta += d_theta

        qz = math.sin(self.theta / 2)
        qw = math.cos(self.theta / 2)
        now = self.get_clock().now().to_msg()

        self.odom_msg.header.stamp = now
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.orientation.z = qz
        self.odom_msg.pose.pose.orientation.w = qw
        self.odom_msg.twist.twist.linear.x = linear
        self.odom_msg.twist.twist.angular.z = angular
        self.odom_pub.publish(self.odom_msg)

        self.tf_msg.header.stamp = now
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

