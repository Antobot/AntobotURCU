#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, JointState

class FrameRemapper(Node):
    def __init__(self):
        super().__init__('frame_remapper')

        # Parameters for odom and GPS
        self.declare_parameter('odom_input_topic', '/antobot/robot/odometry_sim')
        self.declare_parameter('odom_output_topic', '/antobot/robot/odometry')
        self.declare_parameter('gps_input_topic', '/antobot_gps_sim')
        self.declare_parameter('gps_output_topic', '/antobot_gps')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_link_frame', 'base_link')
        self.declare_parameter('gps_frame', 'gps_frame')

        # Parameters for joint_states
        self.declare_parameter('joint_input_topic', '/joint_states_sim')
        self.declare_parameter('joint_output_topic', '/joint_states')
        self.declare_parameter('joint_frame_id', 'base_link')

        # Get parameters
        odom_in = self.get_parameter('odom_input_topic').get_parameter_value().string_value
        odom_out = self.get_parameter('odom_output_topic').get_parameter_value().string_value
        gps_in = self.get_parameter('gps_input_topic').get_parameter_value().string_value
        gps_out = self.get_parameter('gps_output_topic').get_parameter_value().string_value

        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_link_frame = self.get_parameter('base_link_frame').get_parameter_value().string_value
        self.gps_frame = self.get_parameter('gps_frame').get_parameter_value().string_value

        joint_in = self.get_parameter('joint_input_topic').get_parameter_value().string_value
        joint_out = self.get_parameter('joint_output_topic').get_parameter_value().string_value
        self.joint_frame_id = self.get_parameter('joint_frame_id').get_parameter_value().string_value

        # Subscriptions and publishers
        self.odom_sub = self.create_subscription(Odometry, odom_in, self.odom_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, odom_out, 10)

        self.gps_sub = self.create_subscription(NavSatFix, gps_in, self.gps_callback, 10)
        self.gps_pub = self.create_publisher(NavSatFix, gps_out, 10)

        self.joint_sub = self.create_subscription(JointState, joint_in, self.joint_callback, 10)
        self.joint_pub = self.create_publisher(JointState, joint_out, 10)

        self.get_logger().info(f"Remapping odometry: {odom_in} → {odom_out} (frame_id={self.odom_frame})")
        self.get_logger().info(f"Remapping GPS: {gps_in} → {gps_out} (frame_id={self.gps_frame})")
        self.get_logger().info(f"Fixing joint_states: {joint_in} → {joint_out} (frame_id={self.joint_frame_id})")

    # Odometry callback
    def odom_callback(self, msg: Odometry):
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_link_frame
        self.odom_pub.publish(msg)

    # GPS callback
    def gps_callback(self, msg: NavSatFix):
        msg.header.frame_id = self.gps_frame
        self.gps_pub.publish(msg)

    # JointStates callback
    def joint_callback(self, msg: JointState):
        msg.header.frame_id = self.joint_frame_id
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FrameRemapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
