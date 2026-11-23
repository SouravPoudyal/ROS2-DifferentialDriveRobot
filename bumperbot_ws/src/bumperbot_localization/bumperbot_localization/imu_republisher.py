#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuRepublisher(Node):

    def __init__(self):
        super().__init__("imu_republisher_node")
        self.imu_pub = self.create_publisher(Imu, "imu_ekf", 10)
        self.imu_sub = self.create_subscription(Imu, "imu", self.imu_callback, 10)

    def imu_callback(self, imu):
        imu.header.frame_id = "imu_link_ekf"
        for i in (0, 4, 8):                                # xx, yy, zz of each 3x3 matrix
            imu.orientation_covariance[i] = 1e-4
            imu.angular_velocity_covariance[i] = 1e-6
            imu.linear_acceleration_covariance[i] = 1e-3
        self.imu_pub.publish(imu)


def main(args=None):
    rclpy.init(args=args)
    node = ImuRepublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()