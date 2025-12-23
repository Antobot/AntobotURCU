# <!-- 
# The primary purpose of this launch file is to launch the ekf_map and 
# ekf_odom parameters for sensor fusion localiastion.
# The ekf_odom is for the local parameters like odometry and IMU, 
# ekf_map is the for the global parameters like RTK GNSS, wheel odometry and IMU.
# 
#  -->

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from antobot_urcu.launchManager import AntobotSWNode, Launchfile

def generate_launch_description():

    ld = LaunchDescription()

    # Declare launch arguments
    yaw_offset = DeclareLaunchArgument('yaw_offset', default_value='0.0', description='Initial yaw offset')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false', description='Enable simulation time')
    ld.add_action(yaw_offset)
    ld.add_action(use_sim_time_arg)

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Get the path to the YAML configuration files
    ekf_odom_config = os.path.join(get_package_share_directory('antobot_ekf'),'params','ekf_odom_fastlio.yaml')
    ekf_map_config = os.path.join(get_package_share_directory('antobot_ekf'), 'params','ekf_map_fastlio.yaml')

    # Define EKF odometry node
    # ekf_odom_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekfOdom_node',
    #     parameters=[ekf_odom_config, {'use_sim_time': use_sim_time}],
    #     remappings=[('/odometry/filtered', '/odometry/ekfOdom')],
    #     output='screen'
    # )
    # ld.add_action(ekf_odom_node)

    # Define EKF map node
    ekf_map_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekfMap_node',
        parameters=[ekf_map_config, {'use_sim_time': use_sim_time}],
        output='screen'
    )
    ld.add_action(ekf_map_node)

    # Include NavSat launch
    navSatLaunchObj = Launchfile("navSatTransform", 'antobot_ekf', 'navsat_transform_lio.launch.py')
    ld.add_action(navSatLaunchObj.include_launch())

    return ld
