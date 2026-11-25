import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

NOISY_DEFAULTS = {
    "use_noisy": "false",
    "velocity_noise_ratio": "0.2",
    "noise_position_std": "0.0025", "noise_increment_ratio": "0.01", "noise_cpr": "1880.0",
    "noise_seed": "-1", "sample_rate_hz": "22.0", "sample_jitter_s": "0.0097",
    "slew_limit": "50.0", "lag_tau": "0.19", "min_speed": "1.0",
    "left_gain": "1.0", "right_gain": "0.991",
    "use_simple_controller": "true"
}


def generate_launch_description():
    desc_dir = get_package_share_directory("bumperbot_description")
    ctrl_dir = get_package_share_directory("bumperbot_controller")

    declares = [DeclareLaunchArgument(k, default_value=v) for k, v in NOISY_DEFAULTS.items()]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(desc_dir, "launch", "gazebo.launch.py")))

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ctrl_dir, "launch", "controller.launch.py")),
        launch_arguments={"use_sim_time": "true",
                          **{k: LaunchConfiguration(k) for k in NOISY_DEFAULTS}}.items())

    return LaunchDescription(declares + [gazebo, controller])