#! /usr/bin/env python3

# Copyright (c) 2022, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# # # Code Description:     This code is used to launch and monitor scripts which cannot be launched individually due to
#                           configuration issues, or because the code is external. It should also monitor the processes
#                           launched in this manner, and report any issues to antoSupervisor.

# Contacts: daniel.freer@antobot.ai

# # # #  # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #

# TODO
# handle simulation - what to launch when the service is called in the simulation?



import sys
import os
import yaml
import rclpy
from pathlib import Path
from datetime import datetime

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node

from std_srvs.srv import Empty

# ===== imports for SSH execution =====
# These imports are used by the inline SSH execution path when `ssh=True`.
from launch.actions import ExecuteProcess
# ==========================================

class ProcessListener():

    def __init__():
        print("process listener object (dummy)")

#     def process_died(self, name, exit_code):
#         rclpy.logwarn("%s died with code %s", name, exit_code)
#         if ('costmap' in name):
#             rclpy.logerr("SW109" + "1: " + name + " node has died!")
#             self.costmapManagerClient = rclpy.ServiceProxy('/costmap_manager/node_died_costmap',Empty)
#             # req = EmptyRequest()
#             # res = self.costmapManagerClient.call(req)
#         elif ('imu' in name):
#             rclpy.logerr("SW108" + "1: " + name + " node has died!")
#             self.costmapManagerClient = rclpy.ServiceProxy('/costmap_manager/node_died_imu',Empty)
#             # req = EmptyRequest()
#             # res = self.costmapManagerClient.call(req)
#
#         #elif ('opcua_nodes' in name):
#         #    rclpy.logerr("SW2131: " + name + " node has died!")
#         #    netMonitorLaunchClient = rclpy.ServiceProxy("/antobot/netManager/netMonitorLaunch", netMonitorLaunch)
#         #    req = netMonitorLaunch()
#         #    req.command = 1
#         #    req.network = "OPCUA"
#         #    res = netMonitorLaunchClient.call(req.network, req.command)
#
#         elif('webUI_server' in name):
#             rclpy.logerr("SW2131: " + name + " node has died!")
#             netMonitorLaunchClient = rclpy.ServiceProxy("/antobot/netManager/netMonitorLaunch", netMonitorLaunch)
#             req = netMonitorLaunch()
#             req.command = 1
#             req.network = "webUI"
#             res = netMonitorLaunchClient.call(req.network, req.command)
#         else:
#             pass


class RoslaunchWrapperObject():
    def __init__():
        print("roslaunch wrapper object (dummy)")
#     # ref: https://github.com/ros/ros_comm/issues/2238
#     def start(self):
#         super(RoslaunchWrapperObject, self).start()
#         #print(self.server)
#
#     def start_node_name(self,node_name): # for costmap node for now
#         super(RoslaunchWrapperObject, self).start()
#         #print(self.server)
#         if node_name == 'costmap':
#             rclpy.loginfo("SW109" + "0: " + "costmap" + " node has started!")
#         elif node_name == 'imu_euler':
#             rclpy.loginfo("SW108" + "0: " + "imu_euler" + " node has started!")
#         elif node_name == 'opcua_nodes':
#             rclpy.loginfo("SW2130: OPCUA nodes have started!")
#         elif node_name == 'webUI':
#             rclpy.loginfo("SW2130: webUI node has started!")
#
#     def stop(self):
#         #print("Stopping...")
#         #print(self.server)
#         server = self.server.server if self.server is not None and self.server.server is not None else None
#         super(RoslaunchWrapperObject, self).shutdown()
#         if server:
#            server.shutdown()


class AntobotSWNode:
    def __init__(self, name_arg, package_arg, executable_arg, err_code_id, name_space, input_args,node_type, param_files=None, param_dict=None, ssh=[], prefix=[]):
        self._name = name_arg
        self._package = package_arg
        self._executable = executable_arg
        self._err_code_id = err_code_id
        self._namespace = name_space
        self._instance_num = 0
        self._relaunch_count = 0
        self._node_type = node_type
        self._param_files = param_files
        self._param_dict = param_dict
        self._prefix = prefix

        self._relaunch = True # Determines if the node should relaunch
        self._running = False # Tracks if the node should be running - not that it is

        self._input_args = input_args

        self._process = None
        self._node = None

        self.all_params = []
        if self._param_files:
            self.all_params.append(self._param_files)

        if self._param_dict:
            self.all_params.append(self._param_dict)

        # ===== ssh toggle =====
        # If True, this node will be launched remotely over SSH instead of locally.
        self._ssh = ssh
        # =================================

    def define_node(self):
        """
        Method to define a ROS node

        """
        # ===== Inline SSH execution path =====
        # If ssh is enabled, run the target executable on a remote host via SSH.
        # NOTE: we only use the provided ssh list if it contains at least 3 entries:
        #       [user, host, ws]. If the list is shorter, we treat it as "no SSH".
        if len(self._ssh) >= 3:
            # Read remote parameters from the provided list: [user, host, ws]
            user = self._ssh[0]
            host = self._ssh[1]
            ws   = self._ssh[2]

            remote = f"{user}@{host}"

            # Decide between `ros2 run` and `ros2 launch` based on file suffix.
            is_launch = self._executable.endswith('.launch.py') or self._executable.endswith('.xml')

            # Build remote command. We wrap with bash -lc and a quoted string to avoid quoting issues.
            if is_launch:
                remote_cmd = (
                    f'set -e; '
                    f'source /opt/ros/humble/setup.bash; '
                    f'source {ws}/install/setup.bash; '
                    f'exec ros2 launch {self._package} {self._executable}'
                )
            else:
                remote_cmd = (
                    f'set -e; '
                    f'source /opt/ros/humble/setup.bash; '
                    f'source {ws}/install/setup.bash; '
                    f'exec ros2 run {self._package} {self._executable}'
                )

            # Return an ExecuteProcess action that performs the SSH invocation.
            return ExecuteProcess(
                cmd=[
                    'ssh', '-tt', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=no',
                    remote, 'bash', '-lc', f'"{remote_cmd}"'
                ],
                shell=False,
                output='screen'
            )
        # ===========================================

        # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
        self._node = Node(package=self._package, executable=self._executable, name=self._name, parameters=self.all_params,
                          output='log', respawn=True, respawn_delay=3, prefix=self._prefix)
        return self._node

    def launch(self, launcher):
        """
        Method to launch a ROS Node
        """
        # self._launch.start()
        self._process = launcher.launch(self._node)
        self._running = True

    def kill(self):
        """
        Method to kill a node
        """
        self._process.stop()
        self._running = False

    def status(self):
        """
        Method to check activity status of the node
        """
        if self._process.is_alive():
            return "Active"
        else:
            return "Inactive"


class Launchfile:
    def __init__(self, name_arg, package_arg, exec_arg, ssh=[], delay=None, cpu=None):
        self._name = name_arg
        self._package = get_package_share_directory(package_arg)
        self._exec = exec_arg

        # ===== ssh toggle for launch files =====
        # If True, this launch file will be invoked remotely over SSH.
        self._ssh = ssh
        # ============================================
        self._cpu = cpu
        if delay is None:
            self._delay = 0
        else:
            self._delay = delay


    def include_launch(self):

        # ===== Inline SSH execution for launch files =====
        # If ssh is enabled, execute `ros2 launch <package> <launchfile>` on the remote host.
        # NOTE: we only use the provided ssh list if it contains at least 3 entries:
        #       [user, host, ws]. If the list is shorter, we treat it as "no SSH".
        if len(self._ssh) >= 3:
            user   = self._ssh[0]
            host   = self._ssh[1]
            ws     = self._ssh[2]



            remote = f"{user}@{host}"
            remote_cmd = (
                f'set -e; '
                f'source /opt/ros/humble/setup.bash; '
                f'source {ws}/install/setup.bash; '
                f'sleep {self._delay}; '
                f'exec ros2 launch {os.path.basename(self._package)} {self._exec}'
            )
            return ExecuteProcess(
                cmd=[
                    'ssh', '-tt', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=no',
                    remote, 'bash', '-lc', f'"{remote_cmd}"'
                ],
                shell=False,
                output='screen'
            )
        # ========================================================

        if self._cpu is not None:
            # taskset -c <cpu> ros2 launch pkg file.py
            return ExecuteProcess(
                cmd=[
                    'taskset', '-c', self._cpu,
                    'ros2', 'launch',
                    os.path.basename(self._package), self._exec
                ],
                shell=False,
                output='screen'
            )

        extension = self._exec.rsplit('.',1)[-1]
        if extension == "py":
            launch_desc = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(self._package, 'launch', self._exec))
            )
        elif extension == "xml":
            launch_desc = IncludeLaunchDescription(
                XMLLaunchDescriptionSource(os.path.join(self._package, 'launch', self._exec))
            )

        return launch_desc

def main(args):

    rclpy.init_node('launchManager', anonymous=False)
    rclpy.spin()

if __name__ == '__main__':
    main(sys.argv)
