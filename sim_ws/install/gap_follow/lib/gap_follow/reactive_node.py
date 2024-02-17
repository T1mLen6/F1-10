#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class ReactiveFollowGap(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: Subscribe to LIDAR
        # TODO: Publish to drive

        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)

  

        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            10)
        
        self.subscription  # prevent unused variable warning
        self.abab = 0
        self.threshold = 2.1


    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        
        #thershold = 2.5

        #thershold_lower = 0.2
        

        # Reshape the array into groups of 3 elements
        reshaped_array = np.array(ranges).reshape(-1, 3)

        # Calculate the mean along the second axis (axis=1)
        average_per_group = np.mean(reshaped_array, axis=1)

        averages_repeated = np.repeat(average_per_group, 3)

        proc_ranges = np.minimum(averages_repeated, self.threshold)
        #self.get_logger().info('-------------------------%s' % np.argmax(proc_ranges))
        proc_ranges[0:200] = 0.001
        proc_ranges[-200:0] = 0.001


        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        t = 1.8
        free_space_ranges = np.maximum(free_space_ranges - t, 0)

        gap_index_count = np.zeros(len(free_space_ranges))
        index = 0



        for i in range(len(free_space_ranges)):
            if free_space_ranges[i] != 0:
                index += 1
                gap_index_count[i] = index
            else:
                index = 0
                gap_index_count[i] = index

        
        index_a = np.argmax(gap_index_count).astype(int)
        index_b = index_a - max(gap_index_count).astype(int) + 1

        if index_a < index_b:
            start_index = index_a
            end_index = index_b
        else:
            start_index = index_b
            end_index = index_a

        return start_index, end_index
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """
        subset_array = ranges[start_i:end_i + 1]
        mid_index = (start_i + end_i) // 2

        if abs(np.mean(subset_array)) - self.threshold <= 0.0001:
            return mid_index

        max_index = np.argmax(subset_array) + start_i


        # if max_index < 520:
        #     max_index -= abs(max_index - 520) // 2

        # if max_index > 560:
        #     max_index += abs(max_index - 560) // 2

    

        return max_index
        #return (max_index*5 + mid_index)//6

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)
        
        # TODO:
        #Find closest point to LiDAR
        min_index = 200 + np.argmin(proc_ranges[200:-199])

        self.get_logger().info('Min distance: %s ' % min_index)

        #Eliminate all points inside 'bubble' (set them to zero) 
        bubble_radius = 75
        proc_ranges[min_index-bubble_radius : min_index+bubble_radius] = 0


        #Find max length gap 
        start_index, end_index = self.find_max_gap(proc_ranges)
        #Find the best point in the gap 
        
        max_index = self.find_best_point(start_index, end_index, proc_ranges)
        #self.get_logger().info('start index%s' % start_index)
        #self.get_logger().info('max index %s ' % max_index)

        #Publish Drive message
        drive_msg = AckermannDriveStamped()

        steering_angle =  (0.0043518519*max_index - 2.35)*1.2

        #self.get_logger().info('steering_angle %s ' % steering_angle)
        
        #if self.abab % 50 == 0:
            

        

        #steering_angle = 0.0
        velocity = 1.5

        drive_msg.drive.steering_angle = steering_angle

        if steering_angle > 0.35:
            velocity = 0.8
        # else:
        #     velocity = -2.8498*abs(steering_angle) + 1.497

        drive_msg.drive.speed = velocity

        self.publisher_.publish(drive_msg)

        self.abab += 1


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()