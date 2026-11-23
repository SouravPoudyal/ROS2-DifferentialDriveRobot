#!/usr/bin/env python3
"""Drive the robot with a constant Twist for an exact time, then stop it.

  python3 drive_test.py --linear 0.3 --duration 4      # straight
  python3 drive_test.py --angular 0.9 --duration 7     # spin in place

One process does both the motion and the stop, so there is no gap between two
`ros2 topic pub` start-ups during which the controller keeps the last command.
The stop (zeros for 2 s) is also sent if you press Ctrl-C.
"""
import argparse
import time

import rclpy
from geometry_msgs.msg import Twist


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--linear", type=float, default=0.0, help="m/s")
    ap.add_argument("--angular", type=float, default=0.0, help="rad/s")
    ap.add_argument("--duration", type=float, required=True, help="seconds of motion")
    ap.add_argument("--topic", default="/bumperbot_controller/cmd_vel")
    ap.add_argument("--rate", type=float, default=20.0)
    a = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("drive_test")
    pub = node.create_publisher(Twist, a.topic, 10)

    deadline = time.monotonic() + 5.0           # wait until the controller is connected
    while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        print(f"warning: nobody subscribes to {a.topic}")

    def send(v, w):
        m = Twist()
        m.linear.x, m.angular.z = v, w
        pub.publish(m)

    period = 1.0 / a.rate
    try:
        end = time.monotonic() + a.duration
        while time.monotonic() < end:
            send(a.linear, a.angular)
            time.sleep(period)
    finally:
        for _ in range(int(2 * a.rate)):        # always finish with zeros
            send(0.0, 0.0)
            time.sleep(period)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
