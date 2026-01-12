# <!-- 
# The primary purpose of this launch file is to launch the 
# navsat_transform parameters for sensor fusion localiastion.
# -->


import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from antobot_urcu.launchManager import AntobotSWNode, Launchfile

def generate_launch_description():

    # Get the path to the YAML configuration file
    navsat_transform_config = os.path.join(
       get_package_share_directory('antobot_ekf'), 
       'params',
       'navsat_transform_lio.yaml'
    )

    # Path to platform YAML
    platform_config_file = os.path.join(
        get_package_share_directory('antobot_description'),
        'config',
        'platform_config.yaml'
    )

    with open(platform_config_file, 'r') as f:
        platform_config = yaml.safe_load(f)

    robot_hardware = platform_config.get('robot_hardware', True)

    # Define nodes
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        parameters=[navsat_transform_config,
                    {'robot_hardware': robot_hardware}],
        remappings=[('/gps/fix', '/antobot_gps'), ('/imu', '/imu/data_corrected')
                    ],
        arguments=[
            '--ros-args',
            '--log-level', 'navsat_transform:=debug',
            '--log-level', 'robot_localization:=debug',
        ],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(navsat_transform_node)

    return ld