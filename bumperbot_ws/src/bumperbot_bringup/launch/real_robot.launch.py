import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():

    desc_dir = get_package_share_directory("bumperbot_description")
    ctrl_dir = get_package_share_directory("bumperbot_controller")  

    wheel_radius_arg = DeclareLaunchArgument("wheel_radius", default_value="0.033")
    wheel_separation_arg = DeclareLaunchArgument("wheel_separation", default_value="0.1402")

    use_simple_controller_arg = DeclareLaunchArgument("use_simple_controller", default_value="true")

    robot_description = ParameterValue(
        Command(["xacro ", os.path.join(desc_dir, "urdf", "bumperbot.urdf.xacro"),
                 " is_sim:=false"]),
        value_type=str)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": False}],
        output="screen")
    
    # On the real robot the controller manager is its own node
    # (in simulation it lives inside Gazebo). Later entries override earlier ones.
  
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            os.path.join(ctrl_dir, "config", "bumperbot_controllers_real.yaml"),
            {"use_sim_time": False, "update_rate": 50},
        ],
        output="screen")

    mpu6050 = Node(
        package="bumperbot_firmware",
        executable="mpu6050_driver.py",
        parameters=[{"frame_id": "imu_link",
                    "axis_map": ["-y", "-x", "-z"],
                    "accel_scale": [0.857, 0.857, 0.857]}],
        output="screen")

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ctrl_dir, "launch", "controller.launch.py")),
        launch_arguments={
            "use_sim_time": "false",
            "wheel_radius": LaunchConfiguration("wheel_radius"),
            "wheel_separation": LaunchConfiguration("wheel_separation"),
            "controllers_file": os.path.join(ctrl_dir, "config", "bumperbot_controllers_real.yaml"),
            "use_simple_controller": LaunchConfiguration("use_simple_controller"),
            "odom_child_frame": "base_footprint_ekf",
            "vx_cov": "0.004",
            "wz_cov": "0.04",
        }.items())   

    return LaunchDescription([
        wheel_radius_arg,
        wheel_separation_arg,
        use_simple_controller_arg,
        robot_state_publisher,
        controller_manager,
        mpu6050,
        controller,
    ])