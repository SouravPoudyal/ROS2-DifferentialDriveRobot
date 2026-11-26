from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# name, odometry topic, launch argument that switches it on (None = always on)
SOURCES = [
    ("controller", "bumperbot_controller/odom", None),
    ("noisy", "bumperbot_controller/odom_noisy", "noisy"),
    ("ekf", "odometry/filtered", None),
    ("ground_truth", "/model/bumperbot/odometry", "ground_truth"),
]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    relative = LaunchConfiguration("relative_to_start")

    actions = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("relative_to_start", default_value="true"),
        DeclareLaunchArgument("noisy", default_value="true",
                              description="Draw the noisy-controller odometry"),
        DeclareLaunchArgument("ground_truth", default_value="true",
                              description="Draw the Gazebo ground truth (simulation only)"),
    ]

    for name, topic, switch in SOURCES:
        actions.append(Node(
            package="bumperbot_utils",
            executable="trajectory_plotter.py",
            name=f"trajectory_{name}",
            output="screen",
            parameters=[{
                "odometry_topic": topic,
                "trajectory_topic": f"trajectory/{name}",
                "relative_to_start": relative,
                "use_sim_time": use_sim_time,
            }],
            condition=IfCondition(LaunchConfiguration(switch)) if switch else None,
        ))

    return LaunchDescription(actions)