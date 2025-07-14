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
from antobot_platform_msgs.srv import softShutdown, softShutdownResponse
import os
import rclpy
import time


def softShutdownprocess(req):
    return_msg = softShutdownResponse()
    print("soft shutdown service callback entered!")
    time.sleep(4) #leave time to send /antobridge/soft_shutdown_req topic to antobridge, repeat 100 times
     
    print("killing nodes now")
    os.system("rosnode kill /anto_bridge")
    return_msg.responseBool = True
    return_msg.responseString = "Success!"
    
    print("will shutdown in 2 sec ")
    time.sleep(2)
    subprocess.run(["systemctl", "poweroff"], check=True)
    return return_msg

def soft_shutdown_server():
    rclpy.init_node('soft_shutdown_server')
    s = rclpy.Service('soft_shutdown_req', softShutdown,softShutdownprocess)
    rclpy.spin()

        
if __name__ == '__main__': 
    soft_shutdown_server()
    
