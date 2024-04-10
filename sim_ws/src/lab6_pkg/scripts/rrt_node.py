#!/usr/bin/env python3
"""
This file contains the class definition for tree nodes and RRT
Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
"""

import numpy as np
from numpy import linalg as LA
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker
import time

import random
# TODO: import as you need

# class def for tree nodes
# It's up to you if you want to use this
class node(object):
    def __init__(self,x,y):
        self.x = x
        self.y = y
        self.parent = []
        self.cost = None # only used in RRT*
        self.is_root = False

# class def for RRT
class RRT(Node):
    def __init__(self):
        super().__init__('rrt_node')
        # topics, not saved as attributes
        # TODO: grab topics from param file, you'll need to change the yaml file
        pose_topic = "ego_racecar/odom"
        scan_topic = "/scan"

        # you could add your own parameters to the rrt_params.yaml file,
        # and get them here as class attributes as shown above.

        # TODO: create subscribers
        self.pose_sub_ = self.create_subscription(
            #PoseStamped,
            Odometry,
            pose_topic,
            self.pose_callback,
            1)
        self.pose_sub_

        self.scan_sub_ = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            1)
        self.scan_sub_

        # publishers
        # TODO: create a drive message publisher, and other publishers that you might need

        self.publisher1_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)

        self.publisher2_ = self.create_publisher(Marker, 'visualization_marker', 10)

        self.publisher3_ = self.create_publisher(OccupancyGrid, '/grid', 10)

        self.publisher4_ = self.create_publisher(Path, '/rrt_path', 10)

        # class attributes
        # TODO: maybe create your occupancy grid here
        data = np.genfromtxt('src/lab6_pkg/scripts/new_waypoints.csv', delimiter=',')

        # Assuming the first column is the X values and the second column is the Y values
        self.x_values = data[:, 0]  # Selecting all rows from the first column
        self.y_values = data[:, 1]  # Selecting all rows from the second column
        self.waypoints = np.column_stack((self.x_values, self.y_values))
        #self.i = 0
        self.kp = 0.3
        self.lookahead = 0.2
        self.last_idx = 0
        self.offset = -1.
        self.last_path = []
        #self.timer_ = self.create_timer(1.0, self.publish_grid)


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

    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def select_points(self, x,y,z,w,x_pos,y_pos):

        _,_,yaw = self.quaternion_to_rpy(x,y,z,w)
        #self.get_logger().info('RPY[0]%s' % yaw)
        angle_arr = self.angle(self.x_values, self.y_values, x_pos, y_pos)

        angle_diff_arr = np.abs(yaw - angle_arr)

        qualified_angle_index_arr = np.where(angle_diff_arr < 1.5)

        selected_x = self.x_values[qualified_angle_index_arr]
        selected_y = self.y_values[qualified_angle_index_arr]

        #Distance Calculation, upper and lower thershold
        distance_arr = self.euc_distance(selected_x, selected_y, x_pos, y_pos)

        threshold = 1

        above_threshold_indices = np.where(distance_arr > threshold)[0]

        # Get the elements that satisfy the condition
        qualified_elements = distance_arr[above_threshold_indices]

 

        # Find the index of the minimum element among the qualified elements
        min_index = np.argmin(qualified_elements)

        # Get the index of the minimum element in the original array
        qualified_min_index = above_threshold_indices[min_index]

        x_next = selected_x[qualified_min_index]
        y_next = selected_y[qualified_min_index]

        end = (x_next, y_next)

        return end
          
    def pose_callback(self, pose_msg):
        """
        The pose callback when subscribed to particle filter's inferred pose
        Here is where the main RRT loop happens

        Args: 
            pose_msg (PoseStamped): incoming message from subscribed topic
        Returns:

        """
        
        # TODO: find the current waypoint to track using methods mentioned in lecture
        # TODO: update current speed
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = 0.2
        self.current_pos = pose_msg.pose
        start_point = (pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y)
        #self.get_logger().info('start:  %s' % start)
        # print(start)
       # exit()
        x_pos = pose_msg.pose.pose.position.x
        y_pos = pose_msg.pose.pose.position.y

        x_ori = pose_msg.pose.pose.orientation.x
        y_ori = pose_msg.pose.pose.orientation.y
        z_ori = pose_msg.pose.pose.orientation.z
        w_ori = pose_msg.pose.pose.orientation.w

        end_point = self.select_points(x_ori,y_ori, z_ori, w_ori, x_pos, y_pos)

        marker_msg = Marker()
        marker_msg.header.frame_id = "map"
        marker_msg.header.stamp = self.get_clock().now().to_msg()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker_msg.ns = "waypoints"
        marker_msg.id = 0
        marker_msg.type = 2  # Sphere
        marker_msg.action = Marker.ADD
        marker_msg.pose.position.x = end_point[0]
        marker_msg.pose.position.y = end_point[1]
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

        self.get_logger().info('start_0:  %s' % start_point[0])
        self.get_logger().info('start_1:  %s' % start_point[1])
        self.get_logger().info('end_0:  %s' % end_point[0])
        self.get_logger().info('end_1:  %s' % end_point[1])
    
        

        # # print(self.pos_x, self.pos_y)
        tree = [0]
        tree[0] = node(start_point[0], start_point[1])
        #tree[0].parent.append(start_point)

        isgoal = False
        # iscollision = 
        i = 1
        start_time = time.time()
        while (isgoal is False): 
            rand = self.sample()
  
            print('sample', rand)
            quaternion = np.array([
            pose_msg.pose.pose.orientation.x,
            pose_msg.pose.pose.orientation.y,
            pose_msg.pose.pose.orientation.z,
            pose_msg.pose.pose.orientation.w
            ])
            euler = self.euler_from_quaternion(quaternion)
            rand_point = ((rand[1]*(-np.sin(euler[2])) + rand[0]*np.cos(euler[2])), (rand[1]*np.cos(euler[2]) + rand[0]*np.sin(euler[2])))

            sampled_point = (rand_point[0] + start_point[0], rand_point[1] + start_point[1])

            print('sample', sampled_point)

            nearest_ind = self.nearest(tree, sampled_point)

            nearest_node = tree[nearest_ind]
            new_node = self.steer(nearest_node, sampled_point)

            print('new:', new_node)

            iscollision = self.check_collision(nearest_node, new_node)

            distance1 = np.sqrt((new_node[0]-start_point[0])**2 + (new_node[1]-start_point[1])**2)
            distance2 = np.sqrt((end_point[0]-start_point[0])**2 + (end_point[1]-start_point[1])**2)

            if (iscollision is True) & (distance1 < distance2):
                tree.append(i)
                tree[i] = node(new_node[0], new_node[1])
                tree[i].parent.append((nearest_ind, nearest_node.x, nearest_node.y))
                i += 1
                isgoal = self.is_goal(new_node, end_point[0], end_point[1])
            running_time = time.time()-start_time
            if (running_time > 0.5):
                drive_msg.drive.speed = 0.1
                break

  
        if running_time < 0.5:
            #self.get_logger().info('tree:  %s' % tree[-1])
            path = self.find_path(tree, tree[-1])
            path = np.array(path)
            
        else:
            print('last')
            path = self.last_path

        
        self.last_path = path
        path_msg = Path()
        path_msg.header.frame_id = 'map'

        pose = PoseStamped()
        for row in path:
            x, y = float(row[0]), float(row[1] + self.offset)
            
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)

        pose.pose.position.x = end_point[0]
        pose.pose.position.y = end_point[1]
        pose.pose.position.z = 0.0
        path_msg.poses.append(pose)

        self.publisher4_.publish(path_msg)

        


        for i in path:
            delta_x = i[0] - pose_msg.pose.pose.position.x
            delta_y = i[1] - (pose_msg.pose.pose.position.y-self.offset)
        
            desired_yaw = math.atan2(delta_y, delta_x)

            quaternion = np.array([
            pose_msg.pose.pose.orientation.x,
            pose_msg.pose.pose.orientation.y,
            pose_msg.pose.pose.orientation.z,
            pose_msg.pose.pose.orientation.w
            ])
            euler = self.euler_from_quaternion(quaternion)
            yaw_err = desired_yaw - euler[2]
            
            drive_msg.drive.steering_angle = np.clip(yaw_err*1.3, -0.6, 0.6)
  
            
            self.publisher1_.publish(drive_msg)

        

        # TODO: publish drive message, don't forget to limit the steering angle.

  

    def sample(self):
        """
        This method should randomly sample the free space, and returns a viable point

        Args:
        Returns:
            (x, y) (float float): a tuple representing the sampled point

        """
        free_space = []
        i = 0
        #self.occupancy_map = OccupancyGrid()
        for grid_x in range(self.occupancy_map.info.width):
            for grid_y in range(self.occupancy_map.info.height):

                map_index = grid_x + grid_y * self.occupancy_map.info.width

                if i % 500 == 0:
                    self.get_logger().info('map_index%s' % map_index)
                i += 1
                
                if  self.occupancy_map.data[grid_x + grid_y * self.occupancy_map.info.width] == 0:  
                    free_space.append((grid_x, grid_y))

        #self.get_logger().info('free_space:  %s' % free_space)          
        rand_grid_x, rand_grid_y = random.choice(free_space)

        x = rand_grid_x*0.1
        x = np.random.uniform(0, 3)
        y = np.random.uniform((self.L_range-0.3), -(self.R_range-0.3))
 
        return (x, y)

    def nearest(self, tree, sampled_point):
        """
        This method should return the nearest node on the tree to the sampled point

        Args:
            tree ([]): the current RRT tree
            sampled_point (tuple of (float, float)): point sampled in free space
        Returns:
            nearest_node (int): index of neareset node on the tree
        """
        # Convert tree to numpy array for efficient computation
        #self.get_logger().info('Tree:  %s' % tree)
        tree_arr = np.column_stack(([each_node.x for each_node in tree], [each_node.y for each_node in tree]))
    
        
        # Calculate distances between sampled point and all nodes in the tree
        distances = np.linalg.norm(tree_arr - np.array(sampled_point), axis=1)
        
        # Find index of node with minimum distance
        nearest_node_idx = np.argmin(distances)

        return nearest_node_idx
    
    def euc_distance(self, x_array, y_array, a, b):
        distance = np.sqrt((x_array - a)**2 + (y_array - b)**2)
        return distance

    def steer(self, nearest_node, sampled_point):
        """
        This method should return a point in the viable set such that it is closer 
        to the nearest_node than sampled_point is.

        Args:
            nearest_node (Node): nearest node on the tree to the sampled point
            sampled_point (tuple of (float, float)): sampled point
        Returns:
            new_node (Node): new node created from steering
        """

        nearest_node_arr = np.array([nearest_node.x, nearest_node.y])
        sampled_point = np.array(sampled_point)
        
        direction = sampled_point - nearest_node_arr

        angle = np.arctan2(direction[1], direction[0])
        
        # Move the new node closer along the calculated direction
        new_node = (nearest_node.x + 0.1*np.cos(angle), nearest_node.y + 0.1*np.sin(angle))

        return new_node

    def check_collision(self, nearest_node, new_node):
        """
        This method should return whether the path between nearest and new_node is
        collision free.

        Args:
            nearest (Node): nearest node on the tree
            new_node (Node): new node from steering
        Returns:
            collision (bool): whether the path between the two nodes are in collision
                              with the occupancy grid
        """
        x1, y1 = nearest_node.x, nearest_node.y
        # print(x1, y1)
        x2, y2 = new_node[0], new_node[1]
        
        # print(x2, y2)
        num_steps = 5 
        x_step = (x2 - x1) / num_steps
        y_step = (y2 - y1) / num_steps
        for i in range(num_steps+1):
            x = x1 + i*x_step
            y = y1 + i*y_step

            grid_x = np.min((int(x/0.1),19))
            grid_y = np.min((int(y/0.1),19))

            #self.get_logger().info('data:  %s' % len(self.occupancy_map.data))
            if self.occupancy_map.data[grid_x + grid_y * 30] == 100:
                return False
        # exit()
        return True

    def is_goal(self, latest_added_node, goal_x, goal_y):
        """
        This method should return whether the latest added node is close enough
        to the goal.

        Args:
            latest_added_node (Node): latest added node on the tree
            goal_x (double): x coordinate of the current goal
            goal_y (double): y coordinate of the current goal
        Returns:
            close_enough (bool): true if node is close enough to the goal
        """
        if np.sqrt((latest_added_node[0] - goal_x)**2 + (latest_added_node[1] - goal_y)**2) < 0.5:
            return True 
        else:
            return False

    def find_path(self, tree, latest_added_node):
        """
        This method returns a path as a list of Nodes connecting the starting point to
        the goal once the latest added node is close enough to the goal

        Args:
            tree ([]): current tree as a list of Nodes
            latest_added_node (Node): latest added node in the tree
        Returns:
            path ([]): valid path as a list of Nodes
        """
        path = []
        path.append([latest_added_node.x, latest_added_node.y])
        parent = latest_added_node.parent
        b = str(parent[0][2])
        b = float(b)
        
        while parent[0][0] != 0:
            a = float(parent[0][1])
        
            path.append([a, b])
            self.get_logger().info('parent:  %s' % str(parent[0][0]))
            parent = tree[parent[0][0]].parent
        path.append([tree[0].x, tree[0].y])
        path = path[::-1]
        return path





    # # The following methods are needed for RRT* and not RRT
    # def cost(self, tree, node):
    #     """
    #     This method should return the cost of a node

    #     Args:
    #         node (Node): the current node the cost is calculated for
    #     Returns:
    #         cost (float): the cost value of the node
    #     """
    #     return 0

    # def line_cost(self, n1, n2):
    #     """
    #     This method should return the cost of the straight line between n1 and n2

    #     Args:
    #         n1 (Node): node at one end of the straight line
    #         n2 (Node): node at the other end of the straint line
    #     Returns:
    #         cost (float): the cost value of the line
    #     """
    #     return 0

    # def near(self, tree, node):
    #     """
    #     This method should return the neighborhood of nodes around the given node

    #     Args:
    #         tree ([]): current tree as a list of Nodes
    #         node (Node): current node we're finding neighbors for
    #     Returns:
    #         neighborhood ([]): neighborhood of nodes as a list of Nodes
    #     """
    #     neighborhood = []
    #     return neighborhood



def main(args=None):
    rclpy.init(args=args)
    print("RRT Initialized")
    rrt_node = RRT()
    rclpy.spin(rrt_node)

    rrt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
