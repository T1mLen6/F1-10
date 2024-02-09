#!/usr/bin/env python3
from math import sqrt
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /ego_racecar/odom topic to get the current speed of the vehicle.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0.01
        self.last_min = 0.01
        self.lastlast_min = 0.01
        self.this_min = 0.01
        # TODO: create ROS subscribers and publishers.

        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)

        self.subscription1 = self.create_subscription(
            Odometry,
            'ego_racecar/odom',
            self.odom_callback,
            10)
        
        self.subscription1  # prevent unused variable warning

        self.subscription2 = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        
        self.subscription2  # prevent unused variable warning

        # self.subscription3 = self.create_subscription(
        #     AckermannDriveStamped,
        #     'drive',
        #     self.listener_callback,
        #     10)
        
        # self.subscription3  # prevent unused variable warning



        

    def odom_callback(self, odom_msg):
        # TODO: update current speed
        vx = odom_msg.twist.twist.linear.x
        vy = odom_msg.twist.twist.linear.y
        self.speed = np.sqrt(vx**2 + vy**2) 
        #self.get_logger().info('Received speed: %s' % self.speed)

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        TTC = scan_msg.ranges[540] / self.speed
        # self.this_min = scan_msg.range_min
        # lateral_spd = (self.lastlast_min - self.last_min) / scan_msg.scan_time
        # lateral_TTC = (self.last_min - self.this_min) / lateral_spd
        #scan_msg.scan.ranges 
        #print('yeah', scan_msg.angle_min)
        self.get_logger().info('min%s' % scan_msg.angle_min)
        self.get_logger().info('max: %s' % scan_msg.angle_max )
        self.get_logger().info('increment: %s' % scan_msg.angle_increment )
        #self.get_logger().info('Received time incre: %s' % scan_msg.time_increment)

        threshold = 2.0
        if TTC <= threshold:
            ADS = AckermannDriveStamped()
            ADS.drive.speed = 0.0
            self.publisher_.publish(ADS)


        # self.lastlast_min = self.last_min
        # self.last_min = self.this_min
        
        # TODO: publish command to brake
        #pass

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()