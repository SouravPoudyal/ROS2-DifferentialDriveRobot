from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    wheel_radius_arg = DeclareLaunchArgument("wheel_radius", default_value="0.033", description="Wheel radius in meters")
    wheel_separation_arg = DeclareLaunchArgument("wheel_separation", default_value="0.1402", description="Wheel separation in meters")

    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", 
                   "--controller-manager", "/controller_manager", 
                   "--controller-manager-timeout", "30"],
        output="screen",
    )
    simple_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["simple_velocity_controller",
                   "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "30"],
        output="screen",
    )
    simple_controller = Node(
        package="bumperbot_controller",
        executable="simple_controller.py",
        name="simple_controller",
        parameters=[{"wheel_radius": ParameterValue(wheel_radius, value_type=float),
                     "wheel_separation":ParameterValue(wheel_separation, value_type=float),
                     "use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription([
        wheel_radius_arg,
        wheel_separation_arg,
        joint_state_broadcaster_spawner,
        simple_velocity_controller_spawner,
        simple_controller,

    ])