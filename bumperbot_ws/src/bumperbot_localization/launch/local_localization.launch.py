import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument




def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    odom_topic_arg = DeclareLaunchArgument("odom_topic",
                                       default_value="bumperbot_controller/odom_noisy")


    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x", "0", "--y", "0","--z", "0.103",
                   "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                   "--frame-id", "base_footprint_ekf",
                   "--child-frame-id", "imu_link_ekf"],
    )

    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("bumperbot_localization"), "config", "ekf.yaml"),
                {"use_sim_time": LaunchConfiguration("use_sim_time"),
                 "odom0": LaunchConfiguration("odom_topic")}],
    )
    imu_republisher_py = Node(
        package="bumperbot_localization",
        executable="imu_republisher.py",
    )

    return LaunchDescription([
        use_sim_time_arg,
        odom_topic_arg,
        static_transform_publisher,
        robot_localization,
        imu_republisher_py,

    ])
