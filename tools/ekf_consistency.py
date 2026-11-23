#!/usr/bin/env python3
"""Check whether the EKF's reported covariance matches its real error.

  python3 ekf_consistency.py --duration 40 --csv run1.csv

Start it with the robot at rest right after the simulation and EKF came up, then
drive. For each /odometry/filtered message it compares the displacement since the
start with Gazebo ground truth (/model/bumperbot/odometry), and prints the EKF's own
1-sigma from pose.covariance next to the real error.

  consistent:     the error stays inside about 2 sigma most of the time (~95 %)
  overconfident:  the error leaves the 2 sigma band (declared noise too small)
  underconfident: the error is always far below sigma (declared noise too large)

Positions are expressed in the initial body frame, yaw is unwrapped, so frames with
different starting offsets are comparable.
"""
import argparse
import csv
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

GT_TOPIC = "/model/bumperbot/odometry"
EKF_TOPIC = "/odometry/filtered"


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def body_disp(p0, p1):
    c, s = math.cos(p0[2]), math.sin(p0[2])
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    return c * dx + s * dy, -s * dx + c * dy


def rot_cov(cov, a0):
    """x and y variance of the 6x6 pose covariance, rotated into the initial body frame."""
    cx, cxy, cy = cov[0], cov[1], cov[7]
    c, s = math.cos(a0), math.sin(a0)
    return (c * c * cx + 2 * c * s * cxy + s * s * cy,
            s * s * cx - 2 * c * s * cxy + c * c * cy)


class Consistency(Node):
    def __init__(self):
        super().__init__("ekf_consistency")
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=100)
        self.gt = None
        self.gt_raw = 0.0
        self.ekf = None
        self.ekf_raw = 0.0
        self.ref = None
        self.t0 = None
        self.last_print = -10.0
        self.rows = []
        self.create_subscription(Odometry, GT_TOPIC, self.on_gt, qos)
        self.create_subscription(Odometry, EKF_TOPIC, self.on_ekf, qos)

    def on_gt(self, m):
        raw = yaw_of(m.pose.pose.orientation)
        yaw = raw if self.gt is None else self.gt[2] + wrap(raw - self.gt_raw)
        self.gt_raw = raw
        self.gt = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)

    def on_ekf(self, m):
        raw = yaw_of(m.pose.pose.orientation)
        yaw = raw if self.ekf is None else self.ekf[2] + wrap(raw - self.ekf_raw)
        self.ekf_raw = raw
        self.ekf = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)
        if self.gt is None:
            return

        t_msg = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.ref is None:
            self.ref = (self.gt, self.ekf)
            self.t0 = t_msg
            return

        gt0, ekf0 = self.ref
        gx, gy = body_disp(gt0, self.gt)
        ex, ey = body_disp(ekf0, self.ekf)
        err_x, err_y = ex - gx, ey - gy
        err_yaw = (self.ekf[2] - ekf0[2]) - (self.gt[2] - gt0[2])

        vx, vy = rot_cov(m.pose.covariance, ekf0[2])
        sig_x, sig_y = math.sqrt(max(vx, 0.0)), math.sqrt(max(vy, 0.0))
        sig_yaw = math.sqrt(max(m.pose.covariance[35], 0.0))

        t = t_msg - self.t0
        self.rows.append((t, err_x, err_y, err_yaw, sig_x, sig_y, sig_yaw))
        if t - self.last_print >= 2.0:
            self.last_print = t
            print(f"t={t:6.1f}s  err x={err_x:+.3f} y={err_y:+.3f} yaw={math.degrees(err_yaw):+6.1f}deg"
                  f" | sigma x={sig_x:.3f} y={sig_y:.3f} yaw={math.degrees(sig_yaw):.1f}deg")

    def summary(self, csv_path):
        if not self.rows:
            print("no data: are /odometry/filtered and the ground truth topic publishing?")
            return
        if csv_path:
            with open(csv_path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["t", "err_x", "err_y", "err_yaw", "sig_x", "sig_y", "sig_yaw"])
                w.writerows(self.rows)
            print(f"wrote {csv_path}")
        n = len(self.rows)
        inside = [0, 0, 0]
        for _, ex, ey, eyaw, sx, sy, syaw in self.rows:
            inside[0] += abs(ex) <= 2 * sx
            inside[1] += abs(ey) <= 2 * sy
            inside[2] += abs(eyaw) <= 2 * syaw
        t, ex, ey, eyaw, sx, sy, syaw = self.rows[-1]
        print(f"\n{n} samples over {t:.1f} s")
        print(f"final error: x={ex:+.3f} m  y={ey:+.3f} m  yaw={math.degrees(eyaw):+.1f} deg")
        print(f"final sigma: x={sx:.3f} m  y={sy:.3f} m  yaw={math.degrees(syaw):.1f} deg")
        print(f"inside 2 sigma: x={100 * inside[0] / n:.0f} %  y={100 * inside[1] / n:.0f} %"
              f"  yaw={100 * inside[2] / n:.0f} %   (about 95 % means consistent)")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration", type=float, default=40.0)
    ap.add_argument("--csv", default="")
    a = ap.parse_args()

    rclpy.init()
    node = Consistency()
    print(f"Recording {a.duration:.0f} s from the first EKF message (Ctrl-C to stop early)")
    end = time.monotonic() + a.duration
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    node.summary(a.csv)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
