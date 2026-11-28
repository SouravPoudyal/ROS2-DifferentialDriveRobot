
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():

    bumperbot_description_dir = get_package_share_directory("bumperbot_controller")
    config_file_path = lambda f: os.path.join(bumperbot_description_dir, "config", f)
    sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    sim_time_arg =   DeclareLaunchArgument("use_sim_time", default_value="false")
    use_simple_controller_arg = DeclareLaunchArgument("use_simple_controller", default_value="true")

    cmd_vel_out = PythonExpression([
        "'bumperbot_controller/cmd_vel' if '",
        LaunchConfiguration("use_simple_controller"),
        "'.lower() == 'true' else 'bumperbot_controller/cmd_vel_unstamped'"
    ])

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[config_file_path("joystick.yaml"), {"use_sim_time": sim_time}],
        output="screen",
    )

    joystick_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[config_file_path("joy_teleop.yaml"), {"use_sim_time": sim_time}],
        output="screen",
    )
    twist_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("twist_mux"), "launch", "twist_mux_launch.py")),
        launch_arguments={
            "cmd_vel_out": cmd_vel_out,
            "config_topics": config_file_path("twist_mux_topics.yaml"),
            "config_locks": config_file_path("twist_mux_locks.yaml"),
            "config_joy": config_file_path("twist_mux_joy.yaml"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items())


    return LaunchDescription(
        [
            sim_time_arg,
            use_simple_controller_arg,
            joy_node,
            joystick_teleop,
            twist_mux
        ]
    )