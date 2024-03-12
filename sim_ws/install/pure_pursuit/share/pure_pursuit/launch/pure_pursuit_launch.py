#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    safety = Node(
        package='pure_pursuit',
        executable='pure_pursuit_node.py',
    )
    

        
    
    ld.add_action(safety)

    
    return ld