#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    sim = Node(package='simulation', executable='simlaunch.sh')

    lqr = Node(package='control', executable='controltest')

    return LaunchDescription([sim])
