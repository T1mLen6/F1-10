#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    safety = Node(
        package='safety_node',
        executable='safety_node.py',
    )
    

        
    
    ld.add_action(safety)

    
    return ld