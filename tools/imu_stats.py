import sys, math, rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

T = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
rclpy.init()
n = Node("imu_stats")
rows = []
n.create_subscription(Imu, "/imu", lambda m: rows.append((
    m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
    m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z)), 50)
t0 = n.get_clock().now()
while (n.get_clock().now() - t0).nanoseconds < T * 1e9:
    rclpy.spin_once(n, timeout_sec=0.1)
names = ["gyro_x", "gyro_y", "gyro_z", "acc_x", "acc_y", "acc_z"]
print(f"{len(rows)} samples in {T:.0f}s (~{len(rows)/T:.0f} Hz)")
for i, nm in enumerate(names):
    v = [r[i] for r in rows]
    mean = sum(v) / len(v)
    # std of first differences / sqrt(2): ignores slow trends like the turn itself
    d = [b - a for a, b in zip(v, v[1:])]
    dm = sum(d) / len(d)
    std = math.sqrt(sum((x - dm) ** 2 for x in d) / len(d)) / math.sqrt(2)
    print(f"{nm}: mean={mean:+.4f}  noise_std={std:.4f}")
