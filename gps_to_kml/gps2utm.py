#!/usr/bin/env python3
import os
import sys
import csv
import argparse
import logging

import rosbag2_py
import rclpy.serialization
import utm

from sensor_msgs.msg import NavSatFix

def setup_logger() -> logging.Logger:
    """
    Configures and returns a logger for the application.
    """
    logger = logging.getLogger("utm_converter")
    logger.setLevel(logging.DEBUG)

    ch = logging.StreamHandler()
    ch.setLevel(logging.DEBUG)

    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    ch.setFormatter(formatter)

    if not logger.handlers:
        logger.addHandler(ch)

    return logger

def convert_and_save_to_utm(bag_folder: str, target_topic: str, output_file: str, logger: logging.Logger) -> None:
    """
    Reads NavSatFix messages from a rosbag, converts the GPS coordinates to UTM, 
    and writes the data to a CSV file.
    
    :param bag_folder: Absolute path to the rosbag2 directory.
    :param target_topic: The topic to filter messages.
    :param output_file: File path for the output CSV file.
    :param logger: Logger instance for logging.
    """
    try:
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

    logger.info("Reading messages from topic: %s", target_topic)

    try:
        with open(output_file, mode="w", newline="") as csvfile:
            fieldnames = [
                "timestamp", "latitude", "longitude", "altitude",
                "easting", "northing", "zone_number", "zone_letter"
            ]
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()

            while reader.has_next():
                topic, data, timestamp = reader.read_next()

                if topic == target_topic:
                    try:
                        msg = rclpy.serialization.deserialize_message(data, NavSatFix)
                        utm_coords = utm.from_latlon(msg.latitude, msg.longitude)
                        writer.writerow({
                            "timestamp": timestamp,
                            "latitude": msg.latitude,
                            "longitude": msg.longitude,
                            "altitude": msg.altitude,
                            "easting": utm_coords[0],
                            "northing": utm_coords[1],
                            "zone_number": utm_coords[2],
                            "zone_letter": utm_coords[3]
                        })
                    except Exception as e:
                        logger.error("Deserialization or conversion error: %s", e)
    except Exception as e:
        logger.error("File write error: %s", e)

    logger.info("UTM data saved to: %s", output_file)

def parse_arguments() -> argparse.Namespace:
    """
    Parses command-line arguments, ignoring extra ROS 2 launch arguments.
    """
    parser = argparse.ArgumentParser(
        description="Convert NavSatFix messages from a rosbag2 file to CSV with UTM coordinates."
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
        '--output_file',
        type=str,
        required=True,
        help="File path for the output CSV file."
    )
    args, unknown = parser.parse_known_args()
    return args

def main():
    logger = setup_logger()
    args = parse_arguments()

    if not os.path.exists(args.bag_folder):
        logger.error("Bag folder does not exist: %s", args.bag_folder)
        sys.exit(1)

    convert_and_save_to_utm(args.bag_folder, args.target_topic, args.output_file, logger)

if __name__ == "__main__":
    main()
