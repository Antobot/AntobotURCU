#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped

def twist_to_twist_with_covariance(twist_msg):
    # Create a TwistWithCovarianceStamped message
    twist_with_cov_msg = TwistWithCovarianceStamped()
    
    # Set the header (you can modify this as needed)
    twist_with_cov_msg.header.stamp = rclpy.Time.now()
    twist_with_cov_msg.header.frame_id = "base_link"

    # Copy the Twist data
    twist_with_cov_msg.twist.twist = twist_msg

    # Add covariance (example: identity matrix scaled with small values)
    twist_with_cov_msg.twist.covariance = [0.00001] * 36  # Example covariance values

    return twist_with_cov_msg

def twist_callback(twist_msg):
    # Convert Twist to TwistWithCovarianceStamped
    twist_with_cov_msg = twist_to_twist_with_covariance(twist_msg)
    
    # Publish the new message (assuming we have a publisher set up)
    twist_with_cov_pub.publish(twist_with_cov_msg)

if __name__ == "__main__":
    rclpy.init_node("twist_to_twist_with_covariance")

    # Subscribe to the Twist topic
    twist_sub = rclpy.Subscriber("/antobot_robot/cmd_vel", Twist, twist_callback)

    # Publisher for the TwistWithCovarianceStamped topic
    twist_with_cov_pub = rclpy.Publisher("/antobot_robot/cmd_vel_cov", TwistWithCovarianceStamped, queue_size=10)

    rclpy.spin()
