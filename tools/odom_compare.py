#!/usr/bin/env python3
"""Compare ground truth, clean, noisy and EKF odometry over one test motion.

  python3 odom_compare.py --duration 14        # start it, then send the motion command

Every topic is expressed relative to its own first message (displacement in the initial
body frame, yaw change unwrapped), so frames that start with different offsets are
comparable. Press Ctrl-C to stop early; the table is still printed.
"""
import argparse
import math
import time

SOURCES = [
    ("ground_truth", "/model/bumperbot/odometry"),
    ("clean_odom", "/bumperbot_controller/odom"),
    ("noisy_odom", "/bumperbot_controller/odom_noisy"),
    ("ekf", "/odometry/filtered"),
]


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def summarise(data):
    """data: name -> list of (x, y, yaw_unwrapped). Returns name -> dict of results."""
    out = {}
    for name, pts in data.items():
        if len(pts) < 2:
            continue
        x0, y0, a0 = pts[0]
        x1, y1, a1 = pts[-1]
        c, s = math.cos(a0), math.sin(a0)
        dx = c * (x1 - x0) + s * (y1 - y0)
        dy = -s * (x1 - x0) + c * (y1 - y0)
        path = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
        out[name] = dict(dx=dx, dy=dy, dyaw=math.degrees(a1 - a0), path=path, n=len(pts))
    gt = out.get("ground_truth")
    for r in out.values():
        if gt:
            r["pos_err"] = math.hypot(r["dx"] - gt["dx"], r["dy"] - gt["dy"])
            r["yaw_err"] = r["dyaw"] - gt["dyaw"]
    return out


def show(res):
    print(f"\n{'source':14s}{'dx [m]':>9s}{'dy [m]':>9s}{'dyaw [deg]':>12s}{'path [m]':>10s}"
          f"{'pos err [m]':>13s}{'yaw err [deg]':>15s}{'msgs':>7s}")
    for name, _ in SOURCES:
        r = res.get(name)
        if not r:
            print(f"{name:14s}  no data")
            continue
        pe = f"{r['pos_err']:.3f}" if "pos_err" in r else "-"
        ye = f"{r['yaw_err']:+.1f}" if "yaw_err" in r else "-"
        print(f"{name:14s}{r['dx']:9.3f}{r['dy']:9.3f}{r['dyaw']:12.1f}{r['path']:10.3f}"
              f"{pe:>13s}{ye:>15s}{r['n']:7d}")
    print("\nerrors are relative to ground_truth; dx = forward, dy = left in the initial body frame")


def record(duration):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

    rclpy.init()
    node = Node("odom_compare")
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     history=HistoryPolicy.KEEP_LAST, depth=100)
    data = {name: [] for name, _ in SOURCES}
    last = {}

    def make_cb(name):
        def cb(m):
            yaw = yaw_of(m.pose.pose.orientation)
            if name in last:
                yaw = data[name][-1][2] + wrap(yaw - last[name])   # unwrap
            last[name] = yaw_of(m.pose.pose.orientation)
            data[name].append((m.pose.pose.position.x, m.pose.pose.position.y, yaw))
        return cb

    for name, topic in SOURCES:
        node.create_subscription(Odometry, topic, make_cb(name), qos)
    print(f"Recording {duration:.0f} s - send the motion command now (Ctrl-C to stop early)")
    end = time.monotonic() + duration
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return data


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration", type=float, default=14.0)
    show(summarise(record(ap.parse_args().duration)))
