import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    desc_dir = get_package_share_directory("bumperbot_description")
    ctrl_dir = get_package_share_directory("bumperbot_controller")

    # Gazebo + robot_state_publisher + spawn + /clock bridge (is_sim defaults to true)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(desc_dir, "launch", "gazebo.launch.py")))

    # Spawners wait for /controller_manager, which appears once Gazebo loads the plugin
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ctrl_dir, "launch", "controller.launch.py")),
        launch_arguments={"use_sim_time": "true"}.items())

    return LaunchDescription([
        gazebo,
        controller,
    ])