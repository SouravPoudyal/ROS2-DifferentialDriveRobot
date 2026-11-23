#!/usr/bin/env bash
# Collect IMU + encoder statistics with identical commands on the real robot or in simulation.
#   ./collect.sh real     (terminal that sourced ~/ros_laptop.sh)
#   ./collect.sh sim      (terminal that sourced ~/ros_sim.sh)
#   ./collect.sh real enc (encoder runs only)
# Needs imu_stats.py, encoder_stats.py and drive_test.py in the current folder.
# Do NOT run this with joystick/keyboard teleop active: it would fight the test commands.

MODE=$1
ONLY=$2   # optional: "enc" repeats only the encoder runs
case "$MODE" in
  real) JT=/joint_states;       ODOM=/bumperbot_controller/odom ;;
  sim)  JT=/noisy_joint_states; ODOM=/bumperbot_controller/odom_noisy ;;
  *) echo "usage: $0 real|sim"; exit 1 ;;
esac
CMD=/bumperbot_controller/cmd_vel

# message counts (-t) instead of timeouts: the CLI start-up time must not eat the publishing window
# The controller keeps the LAST command when the publisher dies, so every test ends with zeros.
stop() { ros2 topic pub -r 10 -t 20 "$CMD" geometry_msgs/msg/Twist "{}" > /dev/null 2>&1; }
# drive SECONDS LINEAR ANGULAR: exact-time motion and stop in one process (needs drive_test.py)
drive() { python3 drive_test.py --duration "$1" --linear "$2" --angular "$3"; }
trap stop EXIT INT

echo "== environment check =="
env | grep -E "ROS_DOMAIN_ID|ROS_DISCOVERY_SERVER|ROS_LOCALHOST_ONLY"
ros2 topic list | grep -E "^/clock$|noisy_joint_states|^/imu$|/joint_states$"
read -p "Is this the right graph for '$MODE' and the robot clear to move? [Enter = go, Ctrl-C = abort] "
stop   # make sure the robot is stopped before the first test

if [ "$ONLY" != enc ]; then
echo "== 1/4 IMU, still (do not touch the robot) =="
python3 imu_stats.py 20 | tee "${MODE}_imu_still.txt"

echo "== 2/4 IMU, spinning in place at 0.9 rad/s =="
drive 22 0 0.9 &
sleep 1
python3 imu_stats.py 20 | tee "${MODE}_imu_spin.txt"
wait
fi

sleep 4   # let the wheels stop completely
echo "== 3/4 encoders, straight at 0.3 m/s for 7 s (about 2 m: use blocks or clear floor) =="
python3 encoder_stats.py record --label "${MODE}_straight" --duration 14 --joint-topic "$JT" --odom-topic "$ODOM"  --csv &
sleep 3
drive 7 0.3 0
wait

sleep 4
echo "== 4/4 encoders, turn in place at 0.9 rad/s for 7 s =="
python3 encoder_stats.py record --label "${MODE}_turn" --duration 14 --joint-topic "$JT" --odom-topic "$ODOM"  --csv &
sleep 3
drive 7 0 0.9
wait

echo "Done. Files: ${MODE}_imu_still.txt ${MODE}_imu_spin.txt ${MODE}_straight.json ${MODE}_turn.json (+ .csv)"
