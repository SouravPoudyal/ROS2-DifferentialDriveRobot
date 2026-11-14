import os
from pathlib import Path
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    bumperbot_description_dir = get_package_share_directory('bumperbot_description')

    ros_distro = os.environ['ROS_DISTRO']

    is_ignition = ros_distro == 'humble'

    model_arg = DeclareLaunchArgument(
        'model',
        default_value=os.path.join(
            bumperbot_description_dir, 'urdf', 'bumperbot.urdf.xacro'),
        description='Absolute path to robot urdf file'
    )

    resource_path = str(Path(bumperbot_description_dir).parent.resolve())
    resource_env = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH' if is_ignition else 'GAZEBO_RESOURCE_PATH',
        value=resource_path
    )
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, "use_sim_time": True}],
        output='screen',
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': '-v 4 -r empty.sdf'}.items()
        )

    clock_msg = "ignition.msgs.Clock" if is_ignition else "gz.msgs.Clock"
    gz_ros2_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[f'/clock@rosgraph_msgs/msg/Clock[{clock_msg}'],
        output='screen'
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'bumperbot', '-topic', 'robot_description', '-z', '0.05'],
        output='screen'
    )

    return LaunchDescription([
        model_arg,
        resource_env,
        robot_state_publisher_node,
        gazebo,
        gz_ros2_bridge,
        gz_spawn_entity
    ])