#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker
import tf2_ros
from nav_msgs.msg import OccupancyGrid


# TODO CHECK: include needed ROS msg type headers and libraries

class PurePursuit(Node):
    """ 
    Implement Pure Pursuit on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('pure_pursuit_node')
        # TODO: create ROS subscribers and publishers
        self.publisher1_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)

        self.publisher2_ = self.create_publisher(Marker, 'visualization_marker', 10)

  

        self.subscription = self.create_subscription(
            Odometry,
            'ego_racecar/odom',
            self.pose_callback,
            10)
        
        self.scan_sub_ = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            1)
        self.scan_sub_
        
        self.subscription  # prevent unused variable warning

        self.publisher3_ = self.create_publisher(OccupancyGrid, '/grid', 10)

        data = np.genfromtxt('src/pure_pursuit/scripts/waypoint.csv', delimiter=',')

        # Assuming the first column is the X values and the second column is the Y values
        self.x_values = data[:, 0]  # Selecting all rows from the first column
        self.y_values = data[:, 1]  # Selecting all rows from the second column

        self.i = 0
        self.kp = 1

    def scan_callback(self, scan_msg):
        """
        LaserScan callback, you should update your occupancy grid here

        Args: 
            scan_msg (LaserScan): incoming message from subscribed topic
        Returns:

        """
        range_data = scan_msg
        
        angle = 30 / 180 * np.pi

        indexL = 1080 - int(((-np.pi/2 + angle) - range_data.angle_min) / range_data.angle_increment)
        indexR = 0 - int(((np.pi/2 - angle) - range_data.angle_max) /range_data.angle_increment) 

        self.L_range = range_data.ranges[indexL]
        self.R_range = range_data.ranges[indexR]

        self.occupancy_map = OccupancyGrid()
        self.occupancy_map.header = scan_msg.header
        self.occupancy_map.info.width = 30 #cells
        self.occupancy_map.info.height = 20 #cells
        self.occupancy_map.info.resolution = 0.1 # m/cell
        self.occupancy_map.info.origin.position.x = 0.0
        self.occupancy_map.info.origin.position.y = -1.0
        grid = [0] * (30 * 20)

        for i, range_measurement in enumerate(scan_msg.ranges):
            # if range_measurement < scan_msg.range_max:
            # range_measurement -= 0.3
            grid_x= int((range_measurement * np.cos( scan_msg.angle_min + i * scan_msg.angle_increment)) / self.occupancy_map.info.resolution)
            grid_y= int((1.+ range_measurement * np.sin( scan_msg.angle_min + i * scan_msg.angle_increment)) / self.occupancy_map.info.resolution)
            # if grid_y > 19: 
            #     grid_y -=2
            # else:
            #     grid_y +=2
            if 0.0 <= grid_x < 30 and 0.0 <= grid_y < 20:
                index = grid_x + grid_y * 30
                grid[index] = 75

        self.occupancy_map.data = grid
        self.publisher3_.publish(self.occupancy_map)

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
    
    def euc_distance(self, x_array, y_array, a, b):
        distance = np.sqrt((x_array - a)**2 + (y_array - b)**2)
        return distance

    def angle(self, x_array, y_array, x, y):   
        # Coordinates of the starting point
        a, b = x, y
        dx = x_array - a
        dy = y_array - b
        angle = np.arctan2(dy, dx)
        return angle

    def convert_coordinates(self, x, y, shift_x, shift_y, angle_radians):
        # Convert the angle from degrees to radians
        angle_radians = -angle_radians + 3.1415926/2
        translation_x = x - shift_x
        translation_y = y - shift_y
        # Define the transformation matrix
        transformation_matrix = np.array([
        [np.cos(angle_radians), -np.sin(angle_radians)],
        [np.sin(angle_radians), np.cos(angle_radians)]])
        # Apply the transformation
        rotated_point = np.dot(transformation_matrix, np.array([translation_x, translation_y]))
        transformed_a, transformed_b = rotated_point #+ np.array([shift_x, shift_y])
        # Add the shift
        return transformed_a, transformed_b

    def pose_callback(self, pose_msg):
        
        # TODO: find the current waypoint to track using methods mentioned in lecture
        # TODO: update current speed
        x_pos = pose_msg.pose.pose.position.x
        y_pos = pose_msg.pose.pose.position.y

        x = pose_msg.pose.pose.orientation.x
        y = pose_msg.pose.pose.orientation.y
        z = pose_msg.pose.pose.orientation.z
        w = pose_msg.pose.pose.orientation.w
        #self.get_logger().info('X %s' % x)

        _,_,yaw = self.quaternion_to_rpy(x,y,z,w)
        #self.get_logger().info('RPY[0]%s' % yaw)
        angle_arr = self.angle(self.x_values, self.y_values, x_pos, y_pos)

        angle_diff_arr = np.abs(yaw - angle_arr)

        qualified_angle_index_arr = np.where(angle_diff_arr < 1.75)

        selected_x = self.x_values[qualified_angle_index_arr]
        selected_y = self.y_values[qualified_angle_index_arr]
        #self.speed = np.sqrt(vx**2 + vy**2) 

        # if self.i % 100 == 0:
        #     self.get_logger().info('angle:  %s' % angle_arr)
        #     self.get_logger().info('YAW:  %s' % yaw)
        #     self.get_logger().info('angle_diff_arr:  %s' % angle_diff_arr)
        #     self.get_logger().info('select_angle:  %s' % qualified_angle_index_arr)

        #     self.get_logger().info('selected_x:  %s' % selected_x)
        #     self.get_logger().info('selected_y:  %s' % selected_y)
        #     self.get_logger().info('%s')

        #Distance Calculation, upper and lower thershold
        distance_arr = self.euc_distance(selected_x, selected_y, x_pos, y_pos)

        threshold = 0.75

        above_threshold_indices = np.where(distance_arr > threshold)[0]

        # Get the elements that satisfy the condition
        qualified_elements = distance_arr[above_threshold_indices]

        if qualified_elements.size == 0:
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.steering_angle = 0.
            drive_msg.drive.speed = 0.
            return

        # Find the index of the minimum element among the qualified elements
        min_index = np.argmin(qualified_elements)

        # Get the index of the minimum element in the original array
        qualified_min_index = above_threshold_indices[min_index]

        x_next = selected_x[qualified_min_index]
        y_next = selected_y[qualified_min_index]
        
        self.i += 1

        # TODO: calculate curvature/steering angle
            
        shifted_goal_x, shifted_goal_y = self.convert_coordinates(x_next, y_next, x_pos, y_pos, yaw)
            
        L = self.euc_distance(x_pos, y_pos, x_next, y_next)
        y_diff = shifted_goal_x
        gamma = 2*y_diff/(L**2)

        gain = self.kp*gamma

        steering_angle = np.clip(-gain, -0.6, 0.6)

        if self.i % 100 == 0:
            #self.get_logger().info('gain:  %s' % L)
            self.get_logger().info('--------------------- %s')
            self.get_logger().info('shifted_goal_y:  %s' % shifted_goal_x)
            self.get_logger().info('gain: '+str(gain))
            self.get_logger().info('Steering Angle:  %s' % steering_angle)

        
        
        
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering_angle*1.5
        drive_msg.drive.speed = 0.5

        if abs(steering_angle) < 0.1745:
            drive_msg.drive.speed = 0.25

        elif abs(steering_angle) < 0.35:
            drive_msg.drive.speed = 0.2
       
       
       
        self.publisher1_.publish(drive_msg)





        marker_msg = Marker()
        marker_msg.header.frame_id = "map"
        marker_msg.header.stamp = self.get_clock().now().to_msg()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker_msg.ns = "waypoints"
        marker_msg.id = 0
        marker_msg.type = 2  # Sphere
        marker_msg.action = Marker.ADD
        marker_msg.pose.position.x = x_next
        marker_msg.pose.position.y = y_next
        marker_msg.pose.position.z = 0.0

        marker_msg.pose.orientation.x = 0.0
        marker_msg.pose.orientation.y = 0.0
        marker_msg.pose.orientation.z = 0.0
        marker_msg.pose.orientation.w = 1.0

        marker_msg.scale.x = 0.2  # Size of the marker points
        marker_msg.scale.y = 0.2
        marker_msg.scale.z = 0.2

        marker_msg.color.r = 1.0
        marker_msg.color.g = 0.0
        marker_msg.color.b = 0.0
        marker_msg.color.a = 1.0
        marker_msg.lifetime = rclpy.duration.Duration(seconds=1).to_msg()

        self.publisher2_.publish(marker_msg)

        

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
