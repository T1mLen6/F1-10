#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: create subscribers and publishers
        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)

  

        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        
        self.subscription  # prevent unused variable warning


        # TODO: set PID gains
        self.kp = 0.75
        self.kd = 1.0
        self.ki = 0.0

        # TODO: store history
        self.integral = 0
        self.prev_error = 0
        self.error = 0

        # TODO: store any necessary values you think you'll need
        self.ang_incre = 0.
        self.theta = np.pi/3 #adjustable
        
        self.scan_ini_ang = 2.35 - (0.5*np.pi)



    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """
        half_range_index = angle // self.ang_incre
        half_range_index = int(half_range_index)
        #ange_index = 540*2-half_range_index 

        range = range_data[half_range_index]
        #TODO: implement
        return range

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        """

        #TODO:implement
        a = self.get_range(range_data, 2.35*2 - (self.scan_ini_ang + self.theta))
        b = self.get_range(range_data, 2.35*2 - self.scan_ini_ang )

        
  
        alpha = -np.arctan2(a*np.cos(self.theta) - b, a*np.sin(self.theta))
        error = dist - b*np.cos(alpha)



        #self.prev_error = self.error

        return error

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        angle = 0.0

        gain = self.kp*error + self.kd*(self.error-self.prev_error)
        steering_angle = -gain
        #self.get_logger().info('   gain: '+str(gain))
        
        
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = velocity

        if abs(steering_angle) < 0.1745:
            drive_msg.drive.speed = 0.5

        elif abs(steering_angle) < 0.35:
            drive_msg.drive.speed = 1.0
        
        
        self.publisher_.publish(drive_msg)

        drive_msg = AckermannDriveStamped()
        # TODO: fill in drive message and publish

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        self.ang_incre = msg.angle_increment
        #self.get_logger().info('left%s' % msg.ranges[300])
        #self.get_logger().info('right%s' % msg.ranges[750])
        
        
        self.error = self.get_error(msg.ranges, 0.8) # TODO: replace with error calculated by get_error()


        #velocity = 0.0 # TODO: calculate desired car velocity based on error
        self.pid_control(self.error, 1.5) # TODO: actuate the car with PID

        self.prev_error = self.error


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()