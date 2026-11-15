import os
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    default_controllers_file = os.path.join(
        get_package_share_directory("bumperbot_controller"),
        "config", "bumperbot_controllers.yaml")
    
    wheel_radius_arg = DeclareLaunchArgument("wheel_radius", default_value="0.033", description="Wheel radius in meters")
    wheel_separation_arg = DeclareLaunchArgument("wheel_separation", default_value="0.1402", description="Wheel separation in meters")
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true")
    controllers_file_arg = DeclareLaunchArgument(
        "controllers_file", default_value=default_controllers_file,
        description="YAML with the controller parameters")

    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    use_sim_time = LaunchConfiguration("use_sim_time")
    controllers_file = LaunchConfiguration("controllers_file")

    def spawner(name):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name,
                       "--controller-manager", "/controller_manager",
                       "--controller-manager-timeout", "60",
                       "--param-file", controllers_file],
            output="screen",
        )

    simple_controller = Node(
        package="bumperbot_controller",
        executable="simple_controller.py",
        name="simple_controller",
        parameters=[{
            "wheel_radius": ParameterValue(wheel_radius, value_type=float),
            "wheel_separation": ParameterValue(wheel_separation, value_type=float),
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        }],
        output="screen",
    )

    return LaunchDescription([
        wheel_radius_arg,
        wheel_separation_arg,
        use_sim_time_arg,
        controllers_file_arg,
        spawner("joint_state_broadcaster"),
        spawner("simple_velocity_controller"),
        simple_controller,

    ])