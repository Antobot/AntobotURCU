import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from antobot_com_postgresql.db_config_loader import get_robot_config


def generate_launch_description():
    package_path = get_package_share_directory("antobot_description")
    platform_config_path = os.path.join(package_path, "config", "platform_config.yaml")
    platform_config = get_robot_config("platform_config", platform_config_path)

    default_use_sim_time = "true" if not platform_config.get("robot_hardware", False) else "false"

    use_sim_time = LaunchConfiguration("use_sim_time")
    odom_output_topic = LaunchConfiguration("odom_output_topic")
    map_output_topic = LaunchConfiguration("map_output_topic")

    ekf_odom_config = os.path.join(
        get_package_share_directory("antobot_ekf"),
        "params",
        "ekf_odom_lidar.yaml",
    )
    ekf_map_config = os.path.join(
        get_package_share_directory("antobot_ekf"),
        "params",
        "ekf_map_lidar.yaml",
    )
    navsat_transform_config = os.path.join(
        get_package_share_directory("antobot_ekf"),
        "params",
        "navsat_transform.yaml",
    )

    ekf_odom_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekfOdom_node",
        parameters=[ekf_odom_config, {"use_sim_time": use_sim_time}],
        remappings=[("/odometry/filtered", odom_output_topic)],
        output="screen",
    )

    ekf_map_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekfMap_node",
        parameters=[ekf_map_config, {"use_sim_time": use_sim_time}],
        remappings=[("/odometry/filtered", map_output_topic)],
        output="screen",
    )

    navsat_transform_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform_node",
        parameters=[navsat_transform_config, {"use_sim_time": use_sim_time}],
        remappings=[("/gps/fix", "/antobot_gps"), ("/imu", "/imu/data_corrected")],
        arguments=[
            "--ros-args",
            "--log-level",
            "navsat_transform:=debug",
        ],
        output="both",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value=default_use_sim_time),
            DeclareLaunchArgument("odom_output_topic", default_value="/odometry/ekfOdom"),
            DeclareLaunchArgument("map_output_topic", default_value="/odometry/filtered"),
            ekf_odom_node,
            ekf_map_node,
            navsat_transform_node,
        ]
    )
