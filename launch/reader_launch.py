#!/usr/bin/env python3
"""
Launch file to run the rosbag reader node.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gps_to_kml',       # Replace with your actual package name
            executable='reader',        # Ensure the executable is installed as 'reader'
            name='rosbag_reader',
            output='screen',
            arguments=[
                '--bag_folder', '/home/miguel/multipacha_ws/src/gps_to_kml/db/rosbag2_2025_04_04-22_08_55',  # Update to the actual bag folder path
                '--target_topic', '/swift/navsat_fix'
            ]
        )
    ])
