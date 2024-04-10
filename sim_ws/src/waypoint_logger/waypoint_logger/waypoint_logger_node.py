#!/usr/bin/env python
# import rospy
import rclpy
import numpy as np
import atexit
# import tf
from os.path import expanduser
from time import gmtime, strftime
from numpy import linalg as LA
# import tf
# from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
import os


class WaypointsLogger:
    def __init__(self):
        self.home = expanduser('~')
        log_dir = os.path.join(self.home, 'rcws', 'logs')
        os.makedirs(log_dir, exist_ok=True)
        log_file = strftime(log_dir + '/wp-%Y-%m-%d-%H-%M-%S', gmtime()) + '.csv'
        self.file = open(log_file, 'w')
        rclpy.init()
        self.node = rclpy.create_node('waypoints_logger')
        self.subscription = self.node.create_subscription(
            Odometry,
            'ego_racecar/odom',
            self.save_waypoint,
            qos_profile=rclpy.qos.qos_profile_sensor_data
        )

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

    def save_waypoint(self, data):
        quaternion = np.array([
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w
        ])

        euler = self.euler_from_quaternion(quaternion)
        speed = LA.norm(np.array([
            data.twist.twist.linear.x,
            data.twist.twist.linear.y,
            data.twist.twist.linear.z
        ]), 2)
        if data.twist.twist.linear.x > 0.:
            print(data.twist.twist.linear.x)

        self.file.write('%f, %f, %f, %f\n' % (
            data.pose.pose.position.x,
            data.pose.pose.position.y,
            euler[2],
            speed
        ))

    def shutdown(self):
        self.file.close()
        print('Goodbye')
        self.node.destroy_node()
        rclpy.shutdown()

def main():
    waypoints_logger = WaypointsLogger()
    atexit.register(waypoints_logger.shutdown)
    print('Saving waypoints...')
    try:
        rclpy.spin(waypoints_logger.node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()

# def odom_callback(msg):
#     position = msg.pose.pose.position
#     orientation = msg.pose.pose.orientation
#     linear_velocity = msg.twist.twist.linear
#     angular_velocity = msg.twist.twist.angular

#     print(f"Position: x={position.x}, y={position.y}, z={position.z}")
#     print(f"Orientation: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}")
#     print(f"Linear Velocity: x={linear_velocity.x}, y={linear_velocity.y}, z={linear_velocity.z}")
#     print(f"Angular Velocity: x={angular_velocity.x}, y={angular_velocity.y}, z={angular_velocity.z}")

# def main():
#     rclpy.init()
#     node = rclpy.create_node('odom_subscriber')
#     subscription = node.create_subscription(Odometry, 'ego_racecar/odom', odom_callback, 10)

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass

#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
# class waypoints_logger(Node):
#     def __init__(self):
#         super().__init__('waypoint_logger_node')

#         topic = 'pf/pose/odom'

#         # TODO: create subscribers and publishers

#         self.sub = self.create_subscription(Odometry, topic, self.save_waypoint)
    
#     def euler_from_quaternion(quaternion):
#         """
#         Converts quaternion (w in last place) to euler roll, pitch, yaw
#         quaternion = [x, y, z, w]
#         Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
#         """
#         x = quaternion.x
#         y = quaternion.y
#         z = quaternion.z
#         w = quaternion.w

#         sinr_cosp = 2 * (w * x + y * z)
#         cosr_cosp = 1 - 2 * (x * x + y * y)
#         roll = np.arctan2(sinr_cosp, cosr_cosp)

#         sinp = 2 * (w * y - z * x)
#         pitch = np.arcsin(sinp)

#         siny_cosp = 2 * (w * z + x * y)
#         cosy_cosp = 1 - 2 * (y * y + z * z)
#         yaw = np.arctan2(siny_cosp, cosy_cosp)

#         return roll, pitch, yaw

#     def save_waypoint(data):
#         print(data)
#         home = expanduser('~')
#         file = open(strftime(home+'/rcws/logs/wp-%Y-%m-%d-%H-%M-%S',gmtime())+'.csv', 'w')
#         quaternion = np.array([data.pose.pose.orientation.x, 
#                             data.pose.pose.orientation.y, 
#                             data.pose.pose.orientation.z, 
#                             data.pose.pose.orientation.w])

#         euler = self.euler_from_quaternion(quaternion)
#         speed = LA.norm(np.array([data.twist.twist.linear.x, 
#                                 data.twist.twist.linear.y, 
#                                 data.twist.twist.linear.z]),2)
#         if data.twist.twist.linear.x>0.:
#             print(data.twist.twist.linear.x)
#         print(data.pose.pose.position.x, data.pose.pose.position.y)

#         file.write('%f, %f, %f, %f\n' % (data.pose.pose.position.x,
#                                         data.pose.pose.position.y,
#                                         euler[2],
#                                         speed))

#     def shutdown():
#         atexit.register(shutdown)
#         file.close()
#         print('Goodbye')



# def main(args=None):
#     rclpy.init(args=args)
#     print("Saving Waypoint...")
#     waypoint_logger_node = waypoints_logger()
#     rclpy.spin(waypoint_logger_node)

#     # Destroy the node explicitly
#     waypoint_logger_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
# def main():
#     print('Hi from waypoint_logger.')


# if __name__ == '__main__':
#     main()
