#!/usr/bin/env python3
import os
import sys
import logging

import rosbag2_py
import rclpy.serialization
from sensor_msgs.msg import NavSatFix  # Updated message type for the navsat_fix topic

def setup_logger() -> logging.Logger:
    """
    Configures and returns a logger for the application.
    """
    logger = logging.getLogger("bag_reader")
    logger.setLevel(logging.DEBUG)
    
    # Create a console handler with debug log level
    ch = logging.StreamHandler()
    ch.setLevel(logging.DEBUG)
    
    # Create formatter and add it to the handler
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    ch.setFormatter(formatter)
    
    # Avoid adding duplicate handlers in case of multiple calls
    if not logger.handlers:
        logger.addHandler(ch)
    return logger

def read_rosbag(bag_folder: str, target_topic: str, logger: logging.Logger) -> None:
    """
    Reads messages from a rosbag2 folder and logs messages from the specified topic.
    
    :param bag_folder: Absolute path to the rosbag2 directory.
    :param target_topic: The topic to filter messages.
    :param logger: Logger instance for logging.
    """
    # Configure storage and converter options
    try:
        storage_options = rosbag2_py.StorageOptions(uri=bag_folder, storage_id="sqlite3")
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr"
        )
    except Exception as e:
        logger.error("Failed to configure storage or converter options: %s", e)
        return

    # Open the rosbag reader
    try:
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
    except Exception as e:
        logger.error("Failed to open rosbag at '%s': %s", bag_folder, e)
        return

    logger.info("Processing messages from topic: %s", target_topic)

    # Process the messages from the rosbag
    try:
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            if topic == target_topic:
                try:
                    # Deserialize the message using the NavSatFix type
                    msg = rclpy.serialization.deserialize_message(data, NavSatFix)
                    logger.info("Received NavSatFix message: latitude=%.6f, longitude=%.6f, altitude=%.2f",
                                msg.latitude, msg.longitude, msg.altitude)
                except Exception as e:
                    logger.error("Error deserializing message on topic '%s' with data length %d: %s",
                                 topic, len(data), e)
    except Exception as e:
        logger.error("Error reading messages from bag: %s", e)

def main() -> None:
    logger = setup_logger()

    # Determine the absolute path to the package directory
    package_dir = os.path.dirname(os.path.abspath(__file__))
    # Assume the "db" folder is located at the package root
    db_dir = os.path.join(package_dir, '..', 'db')
    # Construct the path to the rosbag2 folder inside the "db" directory
    bag_folder = os.path.join(db_dir, "rosbag2_2025_04_04-22_08_55")

    if not os.path.exists(bag_folder):
        logger.error("Bag folder does not exist: %s", bag_folder)
        sys.exit(1)

    # Define the target topic as provided
    target_topic = "/swift/navsat_fix"
    logger.debug("Resolved bag folder: %s", bag_folder)

    read_rosbag(bag_folder, target_topic, logger)

if __name__ == "__main__":
    main()
