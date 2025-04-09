#!/usr/bin/env python3
"""
Launch file to run the utm_converter node.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gps_to_kml',  # Replace with your actual package name
            executable='gps2utm',   # Ensure the executable is installed as 'utm_converter'
            name='utm_converter_node',
            output='screen',
            arguments=[
                '--bag_folder', '/home/miguel/multipacha_ws/src/gps_to_kml/db/rosbag2_2025_04_04-22_08_55',  # Update this path as needed
                '--target_topic', '/swift/navsat_fix',
                '--output_file', '/home/miguel/multipacha_ws/src/gps_to_kml/db/utm_data.csv'    # Update this path as needed
            ]
        )
    ])
