from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Launch RViz2 if true"
    )


    pkg_share = get_package_share_directory("fastlio2")



    config_file = os.path.join(pkg_share, "config", "lio.yaml")
    rviz_config = os.path.join(pkg_share, "rviz", "fastlio2.rviz")


    lio_node = Node(
        package="fastlio2",
        executable="antobot_lio_fast",
        name="antobot_lio_fast",
        output="screen",
        parameters=[{"config_path": config_file}]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("rviz"))
    )


    return LaunchDescription([
        rviz_arg,
        lio_node,
        rviz_node
    ])
