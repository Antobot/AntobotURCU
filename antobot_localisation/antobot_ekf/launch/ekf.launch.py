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
   use_sim_time = DeclareLaunchArgument(name='use_sim_time', default_value='True', description='Flag to enable use_sim_time'),

   # Get the path to the YAML configuration file
   ekf_odom_config = os.path.join(get_package_share_directory('antobot_ekf'),'params','ekf_odom.yaml')

   # Get the path to the YAML configuration file
   ekf_map_config = os.path.join(get_package_share_directory('antobot_ekf'), 'params','ekf_map.yaml')

   # Define nodes
   ekf_odom_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekfOdom_node',
       parameters=[ekf_odom_config]
   )
   ld.add_action(ekf_odom_node)

   # Define nodes
   ekf_map_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekfMap_node',
       parameters=[ekf_map_config]
   )

   navSatLaunchObj=Launchfile("navSatTransform", 'antobot_ekf', 'navsat_transform.launch.py')
   
   ld.add_action(navSatLaunchObj.include_launch())
   ld.add_action(ekf_map_node)

   return ld
