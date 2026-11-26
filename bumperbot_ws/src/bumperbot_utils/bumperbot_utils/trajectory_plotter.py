#!/usr/bin/env python3
"""Turn an Odometry topic into a nav_msgs/Path that RViz can draw.

Run one instance per source (see launch/trajectories.launch.py).

By default the path is expressed relative to the first message of its topic, so
trajectories that start at different poses (the Gazebo world frame for ground truth,
odom for the controller and the EKF) all start at the origin heading along +x and can
be compared by shape. Set relative_to_start:=false to draw the raw poses instead.

Clear a path between runs with:  ros2 service call /<node name>/reset std_srvs/srv/Empty
(it also clears itself when the clock jumps back, e.g. after a simulation restart).
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Empty


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TrajectoryPlotter(Node):

    def __init__(self):
        super().__init__("trajectory_plotter")

        self.declare_parameter("odometry_topic", "bumperbot_controller/odom")
        self.declare_parameter("trajectory_topic", "bumperbot_controller/trajectory")
        self.declare_parameter("path_frame", "odom")        # frame_id of the Path message
        self.declare_parameter("relative_to_start", True)
        self.declare_parameter("min_distance", 0.01)        # m, store a pose after this much motion
        self.declare_parameter("min_angle", 0.1)            # rad, or after this much rotation
        self.declare_parameter("max_poses", 5000)           # oldest poses are dropped beyond this
        self.declare_parameter("publish_rate_hz", 5.0)

        odom_topic = self.get_parameter("odometry_topic").value
        traj_topic = self.get_parameter("trajectory_topic").value
        self.path_frame = self.get_parameter("path_frame").value
        self.relative = self.get_parameter("relative_to_start").value
        self.min_distance = self.get_parameter("min_distance").value
        self.min_angle = self.get_parameter("min_angle").value
        self.max_poses = int(self.get_parameter("max_poses").value)
        rate = max(float(self.get_parameter("publish_rate_hz").value), 0.1)

        # best effort accepts both reliable and best-effort publishers
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=50)
        self.odom_subscriber = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, qos)
        self.traj_publisher = self.create_publisher(Path, traj_topic, 10)
        self.create_timer(1.0 / rate, self.publish_path)
        self.create_service(Empty, "~/reset", self.reset_callback)

        self.path = Path()
        self.path.header.frame_id = self.path_frame
        self.origin = None        # (x, y, yaw) of the first message
        self.last_pose = None     # (x, y, yaw) of the last stored pose
        self.last_stamp = None

        self.get_logger().info(
            f"Trajectory: {odom_topic} -> {traj_topic} "
            f"(frame {self.path_frame}, relative_to_start={self.relative})")

    def reset(self):
        self.path.poses.clear()
        self.origin = None
        self.last_pose = None
        self.traj_publisher.publish(self.path)

    def reset_callback(self, request, response):
        self.reset()
        self.get_logger().info("Trajectory cleared")
        return response

    def odom_callback(self, msg: Odometry):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_stamp is not None and stamp < self.last_stamp - 1.0:
            self.get_logger().info("Clock jumped back, clearing the trajectory")
            self.reset()
        self.last_stamp = stamp

        p = msg.pose.pose
        yaw = yaw_of(p.orientation)

        if self.relative:
            if self.origin is None:
                self.origin = (p.position.x, p.position.y, yaw)
            x0, y0, a0 = self.origin
            c, s = math.cos(a0), math.sin(a0)
            dx, dy = p.position.x - x0, p.position.y - y0
            x, y, th = c * dx + s * dy, -s * dx + c * dy, wrap(yaw - a0)
        else:
            x, y, th = p.position.x, p.position.y, yaw

        # store a pose only after enough motion, so the path stays small
        if self.last_pose is not None:
            lx, ly, lth = self.last_pose
            if (math.hypot(x - lx, y - ly) < self.min_distance
                    and abs(wrap(th - lth)) < self.min_angle):
                return
        self.last_pose = (x, y, th)

        pose = PoseStamped()
        pose.header.frame_id = self.path_frame
        pose.header.stamp = msg.header.stamp
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(th / 2.0)
        pose.pose.orientation.w = math.cos(th / 2.0)
        self.path.poses.append(pose)

        extra = len(self.path.poses) - self.max_poses
        if extra > 0:
            del self.path.poses[:extra]

    def publish_path(self):
        if self.path.poses:
            self.traj_publisher.publish(self.path)


def main():
    rclpy.init()
    traj_plotter = TrajectoryPlotter()
    try:
        rclpy.spin(traj_plotter)
    finally:
        traj_plotter.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()