#!/usr/bin/env python3
# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

#from std_msgs.msg import String

import numpy as np

from ackermann_msgs.msg import AckermannDriveStamped 


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('talker')


        self.publisher_ = self.create_publisher(AckermannDriveStamped, 'drive', 10)
        timer_period = 0.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.declare_parameter('v', 0)
        self.declare_parameter('d', 0)
        self.v = np.float(self.get_parameter('v').value)
        self.d = np.float(self.get_parameter('d').value)

    def timer_callback(self):
        msg = AckermannDriveStamped()
        #msg.data = 'Frame: %d' % self.i
        msg.drive.steering_angle = self.d
        msg.drive.speed = self.v


        self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
        print(self.i)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
