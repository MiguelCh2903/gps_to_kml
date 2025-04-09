#!/usr/bin/env python3
"""
Launch file to run the gps_to_kml node.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gps_to_kml',  # Replace with your actual package name if different
            executable='gps_to_kml',  # Ensure the executable is installed as 'gps_to_kml'
            name='gps_to_kml_node',
            output='screen',
            arguments=[
                '--bag_folder', '/home/miguel/multipacha_ws/src/gps_to_kml/db/rosbag2_2025_04_04-22_08_55',  # Update this path as needed
                '--target_topic', '/swift/navsat_fix',
                '--output_kml', '/home/miguel/multipacha_ws/src/gps_to_kml/db/navsat_fix.kml'  # Update this path as needed
            ]
        )
    ])
