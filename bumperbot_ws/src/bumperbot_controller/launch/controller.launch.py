import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NOISY_DEFAULTS = {
    "noise_position_std": "0.0025", "noise_increment_ratio": "0.01", "noise_cpr": "1880.0",
    "noise_seed": "-1", "sample_rate_hz": "22.0", "sample_jitter_s": "0.0097",
    "slew_limit": "50.0", "lag_tau": "0.19", "min_speed": "1.0",
    "left_gain": "1.0", "right_gain": "0.991",
    "velocity_noise_ratio": "0.2",
    "use_simple_controller": "true",
}


def spawner(name, controllers_file):
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=[name,
                   "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "60",
                   "--param-file", controllers_file],
        output="screen",
    )


def launch_setup(context, *args, **kwargs):
    cfg = lambda k: LaunchConfiguration(k).perform(context)
    use_simple = cfg("use_simple_controller").lower() == "true"
    use_noisy = cfg("use_noisy").lower() == "true"
    use_sim_time = cfg("use_sim_time").lower() == "true"
    controllers_file = cfg("controllers_file")

    actions = [spawner("joint_state_broadcaster", controllers_file)]

    if use_simple:
        # ---- custom kinematics/odometry node (optionally noisy) ----
        params = {
            "wheel_radius": float(cfg("wheel_radius")),
            "wheel_separation": float(cfg("wheel_separation")),
            "use_sim_time": use_sim_time,
        }
        if use_noisy:
            params.update({
                "position_noise_std": float(cfg("noise_position_std")),
                "increment_noise_ratio": float(cfg("noise_increment_ratio")),
                "quantization_cpr": float(cfg("noise_cpr")),
                "seed": int(cfg("noise_seed")),
                "sample_rate_hz": float(cfg("sample_rate_hz")),
                "sample_jitter_s": float(cfg("sample_jitter_s")),
                "slew_limit": float(cfg("slew_limit")),
                "lag_tau": float(cfg("lag_tau")),
                "min_speed": float(cfg("min_speed")),
                "left_gain": float(cfg("left_gain")),
                "right_gain": float(cfg("right_gain")),
                "velocity_noise_ratio": float(cfg("velocity_noise_ratio")),
            })
        
        if not use_noisy:
            # Plain controller (real robot): what the EKF needs from the odometry message.
            params.update({
                "odom_child_frame": cfg("odom_child_frame"),
                "vx_cov": float(cfg("vx_cov")),
                "wz_cov": float(cfg("wz_cov")),
            })       

        controller_node = Node(
            package="bumperbot_controller",
            executable="noisy_controller.py" if use_noisy else "simple_controller.py",
            name="noisy_controller" if use_noisy else "simple_controller",
            parameters=[params],
            output="screen",
        )
        actions += [spawner("simple_velocity_controller", controllers_file),
                    controller_node]
        if use_noisy:
            # Clean odometry (odom -> base_footprint) next to the noisy controller.
            # It only observes the joint states; the noisy node drives the wheels.
            actions.append(Node(
                package="bumperbot_controller",
                executable="simple_controller.py",
                name="simple_controller",
                parameters=[{
                    "wheel_radius": params["wheel_radius"],
                    "wheel_separation": params["wheel_separation"],
                    "use_sim_time": use_sim_time,
                    "drive_wheels": False,
                }],
                output="screen",
            ))
    else:
        # ---- standard diff_drive_controller (publishes odom and TF itself) ----
        # It listens on cmd_vel_unstamped (Twist, use_stamped_vel: false), so a relay
        # keeps joystick and keyboard on /bumperbot_controller/cmd_vel in both modes.
        relay = Node(
            package="topic_tools",
            executable="relay",
            name="cmd_vel_relay",
            arguments=["/bumperbot_controller/cmd_vel",
                       "/bumperbot_controller/cmd_vel_unstamped"],
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
        )
        actions += [spawner("bumperbot_controller", controllers_file), relay]

    return actions


def generate_launch_description():
    default_controllers_file = os.path.join(
        get_package_share_directory("bumperbot_controller"),
        "config", "bumperbot_controllers.yaml")
    

    return LaunchDescription([
        DeclareLaunchArgument("wheel_radius", default_value="0.033"),
        DeclareLaunchArgument("wheel_separation", default_value="0.1402"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("controllers_file", default_value=default_controllers_file),
        DeclareLaunchArgument("odom_child_frame", default_value="base_footprint"),
        DeclareLaunchArgument("vx_cov", default_value="0.0"),
        DeclareLaunchArgument("wz_cov", default_value="0.0"),
        DeclareLaunchArgument("use_noisy", default_value="false",
                              description="Run the noise-injecting controller (simple controller only)"),
        *[DeclareLaunchArgument(k, default_value=v) for k, v in NOISY_DEFAULTS.items()],
        OpaqueFunction(function=launch_setup),
    ])