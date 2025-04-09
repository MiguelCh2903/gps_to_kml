#!/usr/bin/env python3
import os
import sys
import argparse
import logging

import rosbag2_py
import rclpy.serialization
from sensor_msgs.msg import NavSatFix  # Using NavSatFix for the /swift/navsat_fix topic

def setup_logger() -> logging.Logger:
    """
    Configures and returns a logger for the application.
    """
    logger = logging.getLogger("bag_reader")
    logger.setLevel(logging.DEBUG)
    
    # Create a console handler with debug log level
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.DEBUG)
    
    # Create formatter and add it to the handler
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    console_handler.setFormatter(formatter)
    
    # Avoid duplicate handlers if the logger is already configured
    if not logger.handlers:
        logger.addHandler(console_handler)
    
    return logger

def read_rosbag(bag_folder: str, target_topic: str, logger: logging.Logger) -> None:
    """
    Reads messages from a rosbag2 folder and logs messages from the specified topic.
    
    :param bag_folder: Absolute path to the rosbag2 directory.
    :param target_topic: The topic to filter messages.
    :param logger: Logger instance for logging.
    """
    try:
        storage_options = rosbag2_py.StorageOptions(uri=bag_folder, storage_id="sqlite3")
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr"
        )
    except Exception as e:
        logger.error("Failed to configure storage or converter options: %s", e)
        return

    try:
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
    except Exception as e:
        logger.error("Failed to open rosbag at '%s': %s", bag_folder, e)
        return

    logger.info("Processing messages from topic: %s", target_topic)

    try:
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            if topic == target_topic:
                try:
                    # Deserialize the message using the NavSatFix type
                    msg = rclpy.serialization.deserialize_message(data, NavSatFix)
                    logger.info(
                        "Received NavSatFix message: latitude=%.6f, longitude=%.6f, altitude=%.2f",
                        msg.latitude, msg.longitude, msg.altitude
                    )
                except Exception as e:
                    logger.error(
                        "Error deserializing message on topic '%s' (data length: %d): %s",
                        topic, len(data), e
                    )
    except Exception as e:
        logger.error("Error reading messages from bag: %s", e)

def parse_arguments() -> argparse.Namespace:
    """
    Parses command-line arguments, ignoring any additional ROS 2 launch arguments.
    """
    parser = argparse.ArgumentParser(
        description="Read and process a rosbag2 file."
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
    args, unknown = parser.parse_known_args()
    return args

def main() -> None:
    logger = setup_logger()
    args = parse_arguments()

    if not os.path.exists(args.bag_folder):
        logger.error("Bag folder does not exist: %s", args.bag_folder)
        sys.exit(1)

    logger.debug("Using bag folder: %s", args.bag_folder)
    read_rosbag(args.bag_folder, args.target_topic, logger)

if __name__ == "__main__":
    main()
