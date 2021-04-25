#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    diffDrive = Node(package='controller', executable='controller')

    slidingModeControl = Node(package='control', executable='control')

    return LaunchDescription([diffDrive, slidingModeControl])
