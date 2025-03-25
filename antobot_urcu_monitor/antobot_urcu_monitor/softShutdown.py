#!/usr/bin/env python3

# Copyright (c) 2019, ANTOBOT LTD.
# All rights reserved.

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 

#Description:   This is a python script to kill the nodes and shutdown host pc using ssh
#Interface:
#Inputs:      soft shutdown request [softShutdown] - soft shutdown client request send from Anto_supervisor      
#Contact:     zhuang.zhou@antobot.ai

# # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # 
import paramiko
import rclpy
from rclpy.node import Node
from antobot_platform_msgs.srv import SoftShutdown
import os
import time

#host info to access from inside docker, can be removed if no docker in robot platform

class SoftShutdownServer(Node):
    def __init__(self):
        super().__init__('soft_shutdown_server')
        self.srv = self.create_service(SoftShutdown, 'soft_shutdown_req', self.softshutdown_callback)
        self.host = '192.168.1.102' #'antobot-desktop'
        self.user = 'ubuntu' #'antobot'
        self.password = 'Antobot2021-'
        
    def softshutdown_callback(self, req, return_msg):

        print("soft shutdown service callback entered!")
        print("Sending softShutdown request to Aurix")
        time.sleep(4) #leave time to send /antobridge/soft_shutdown_req topic to antobridge, repeat 100 times
        print("killing nodes now")
        os.system("ros2 lifecycle set /anto_bridge shutdown")
        return_msg.response_bool = True
        return_msg.response_string = "Success!"
    
        print("will shutdown in 2 sec ")
        time.sleep(2)
        os.system("sudo mkdir /test")
        #ssh_exec_command("sudo shutdown -h now") #shutdown computer now
        #self.ssh_exec_command("sudo mkdir /test")
        return return_msg
        



    def ssh_exec_command(self, command): 
        #ssh the host, send command passed from softShutdownprocess function
        try: 
            ssh_client = paramiko.SSHClient() 
            ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy()) 
            ssh_client.connect(self.host,22,self.user,self.password)       
            print("command: " + command) #command passed from softShutdownprocess function
            std_in, std_out, std_err = ssh_client.exec_command(command, get_pty=True) 
            std_in.write(self.password + '\n')
            for line in std_out: 
                print(line.strip("\n"))
            for line in std_err: 
                print(line.strip("\n"))         
            ssh_client.close() 
        except Exception as e: 
            print("error: " + str(e)) 

def main():
    #receiving req from antosupervisor 
    rclpy.init()
    softshutdown_server=SoftShutdownServer()
    rclpy.spin(softshutdown_server)
    rclpy.shutdown()
        
if __name__ == '__main__': 
    main()
    
