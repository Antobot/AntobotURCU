#!/usr/bin/env python3

# Copyright (c) 2019, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

#Description:   This is a python script to kill the nodes and shutdown xaiver and robot
#Interface:
#Inputs:      soft shutdown request [softShutdown] - soft shutdown client request send from urcuMonitor.py 
#Contact:     zhuang.zhou@antobot.ai

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

import subprocess
from antobot_platform_msgs.srv import SoftShutdown
import os
import rclpy
from rclpy.node import Node
import time


class softShutdownServer(Node):

    def __init__(self):
        super().__init__("softShutdownServer")
        self.logger = self.get_logger()
        self.create_service(SoftShutdown, "/antobot/soft_shutdown_req", self.softShutdownProcess)

        return

    def softShutdownProcess(req, res):

        self.logger.info("SF1410: Soft shutdown service callback entered!")
        time.sleep(4) #leave time to send /antobridge/soft_shutdown_req topic to antobridge, repeat 100 times
        
        self.logger.info("SF1410: killing nodes now")
        os.system("rosnode kill /anto_bridge")
        res.response_bool = True
        res.response_string = "Success!"
        
        self.logger.info("SF1410: will shutdown in 2 sec ")
        time.sleep(2)
        subprocess.run(["systemctl", "poweroff"], check=True)
        return res


def main():
    rclpy.init()
    srv = softShutdownServer()
    rclpy.spin(srv)

        
if __name__ == '__main__': 
    main()
    
