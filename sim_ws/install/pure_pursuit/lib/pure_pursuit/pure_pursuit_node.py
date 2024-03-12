#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from geometry_msgs.msg import Quaternion
import tf2_ros


# TODO CHECK: include needed ROS msg type headers and libraries

class PurePursuit(Node):
    """ 
    Implement Pure Pursuit on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('pure_pursuit_node')
        # TODO: create ROS subscribers and publishers
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)

  

        self.subscription = self.create_subscription(
            Odometry,
            'ego_racecar/odom',
            self.pose_callback,
            10)
        
        
        self.subscription  # prevent unused variable warning

    def quaternion_to_rpy(self, x,y,z,w):


       # Create a numpy array from the quaternion components
        quat_array = np.array([w, x, y, z])

        # Calculate roll (rotation around x-axis)
        roll = np.arctan2(2.0 * (quat_array[0] * quat_array[1] + quat_array[2] * quat_array[3]),
                        1.0 - 2.0 * (quat_array[1]**2 + quat_array[2]**2))

        # Calculate pitch (rotation around y-axis)
        pitch = np.arcsin(2.0 * (quat_array[0] * quat_array[2] - quat_array[3] * quat_array[1]))

        # Calculate yaw (rotation around z-axis)
        yaw = np.arctan2(2.0 * (quat_array[0] * quat_array[3] + quat_array[1] * quat_array[2]),
                        1.0 - 2.0 * (quat_array[2]**2 + quat_array[3]**2))
        
        
        return roll, pitch, yaw

    def pose_callback(self, pose_msg):
        
        # TODO: find the current waypoint to track using methods mentioned in lecture
        # TODO: update current speed
        x = pose_msg.pose.pose.orientation.x
        y = pose_msg.pose.pose.orientation.y
        z = pose_msg.pose.pose.orientation.z
        w = pose_msg.pose.pose.orientation.w
        self.get_logger().info('X %s' % x)

        

        roll,pitch,yaw = self.quaternion_to_rpy(x,y,z,w)
        self.get_logger().info('RPY[0]%s' % yaw)
        #self.speed = np.sqrt(vx**2 + vy**2) 

        # TODO: transform goal point to vehicle frame of reference

        # TODO: calculate curvature/steering angle

        # TODO: publish drive message, don't forget to limit the steering angle.

def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
