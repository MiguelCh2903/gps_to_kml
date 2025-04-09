#!/usr/bin/env python3
import os
import sys
import argparse
import logging

import rosbag2_py
import rclpy.serialization
import simplekml

from sensor_msgs.msg import NavSatFix

def setup_logger() -> logging.Logger:
    """
    Configures and returns a logger for the application.
    """
    logger = logging.getLogger("gps_to_kml")
    logger.setLevel(logging.DEBUG)
    
    # Create a console handler with a debug log level
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.DEBUG)
    
    # Define a formatter and add it to the handler
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    console_handler.setFormatter(formatter)
    
    if not logger.handlers:
        logger.addHandler(console_handler)
        
    return logger

def create_kml(bag_folder: str, target_topic: str, output_kml: str, logger: logging.Logger) -> None:
    """
    Reads NavSatFix messages from a rosbag and writes the data to a KML file.
    
    :param bag_folder: Absolute path to the rosbag2 directory.
    :param target_topic: The topic to filter messages.
    :param output_kml: File path for the output KML file.
    :param logger: Logger instance for logging.
    """
    try:
        # Configure storage and conversion options
        storage_options = rosbag2_py.StorageOptions(uri=bag_folder, storage_id="sqlite3")
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr"
        )
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
    except Exception as e:
        logger.error("Failed to open rosbag: %s", e)
        return

    logger.info("Processing messages from topic: %s", target_topic)
    
    # Initialize KML object
    kml = simplekml.Kml()
    
    try:
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            if topic == target_topic:
                try:
                    # Deserialize the NavSatFix message
                    msg = rclpy.serialization.deserialize_message(data, NavSatFix)
                    
                    # Create a new placemark in KML (coordinates are in lon, lat, alt)
                    placemark = kml.newpoint(
                        name=str(timestamp),
                        coords=[(msg.longitude, msg.latitude, msg.altitude)]
                    )
                    
                    logger.debug(
                        "Processed message: lat=%.6f, lon=%.6f, alt=%.2f",
                        msg.latitude, msg.longitude, msg.altitude
                    )
                except Exception as e:
                    logger.error("Error processing message on topic '%s': %s", topic, e)
    except Exception as e:
        logger.error("Error reading messages from bag: %s", e)
    
    try:
        kml.save(output_kml)
        logger.info("KML file saved to: %s", output_kml)
    except Exception as e:
        logger.error("Error saving KML file: %s", e)

def parse_arguments() -> argparse.Namespace:
    """
    Parses command-line arguments, ignoring extra ROS 2 launch arguments.
    """
    parser = argparse.ArgumentParser(
        description="Convert NavSatFix messages from a rosbag2 file into a KML file."
    )
    parser.add_argument(
        '--bag_folder',
        type=str,
        required=True,
        help="Absolute path to the rosbag2 directory."
    )
    parser.add_argument(
        '--target_topic',
        type=str,
        required=True,
        help="The topic to filter messages (e.g., '/swift/navsat_fix')."
    )
    parser.add_argument(
        '--output_kml',
        type=str,
        required=True,
        help="File path for the output KML file."
    )
    args, unknown = parser.parse_known_args()
    return args

def main() -> None:
    logger = setup_logger()
    args = parse_arguments()

    if not os.path.exists(args.bag_folder):
        logger.error("Bag folder does not exist: %s", args.bag_folder)
        sys.exit(1)
    
    logger.debug("Using bag folder: %s", args.bag_folder)
    create_kml(args.bag_folder, args.target_topic, args.output_kml, logger)

if __name__ == "__main__":
    main()
