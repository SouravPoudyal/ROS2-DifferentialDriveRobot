import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

NOISY_DEFAULTS = {
    "use_noisy": "false",
    "noise_position_std": "0.0025", "noise_increment_ratio": "0.01", "noise_cpr": "1880.0",
    "noise_seed": "-1", "sample_rate_hz": "22.0", "sample_jitter_s": "0.0097",
    "slew_limit": "50.0", "lag_tau": "0.19", "min_speed": "1.0",
    "left_gain": "1.0", "right_gain": "0.991",
    "velocity_noise_ratio": "0.2",
    "use_simple_controller": "true",
}

def generate_launch_description():
    bringup_dir = get_package_share_directory("bumperbot_bringup")
    ctrl_dir = get_package_share_directory("bumperbot_controller")

    use_sim = LaunchConfiguration("use_sim")
    use_joystick = LaunchConfiguration("use_joystick")
    use_rviz = LaunchConfiguration("use_rviz")

    use_sim_arg = DeclareLaunchArgument(
        "use_sim", default_value="true",
        description="true: Gazebo simulation, false: real robot")
       
    use_joystick_arg = DeclareLaunchArgument(
        "use_joystick", default_value="false",
        description="Also start the joystick teleop")
    
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="false",
        description="Also start RViz")
    
    noisy_declares = [DeclareLaunchArgument(k, default_value=v) for k, v in NOISY_DEFAULTS.items()]
    
    simulated = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, "launch", "simulated_robot.launch.py")),
        condition=IfCondition(use_sim))

    real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, "launch", "real_robot.launch.py")),
        condition=UnlessCondition(use_sim))

    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ctrl_dir, "launch", "joystick_teleop.launch.py")),
        condition=IfCondition(use_joystick))

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", os.path.join(bringup_dir, "rviz", "bumperbot.rviz")],
        parameters=[{"use_sim_time": ParameterValue(use_sim, value_type=bool)}],
        output="screen",
        condition=IfCondition(use_rviz))
        
    return LaunchDescription([
        use_sim_arg,
        use_joystick_arg,
        use_rviz_arg,
        *noisy_declares,
        simulated,
        real,
        joystick,
        rviz,

    ])
