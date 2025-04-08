#!/usr/bin/env python3
import os
import sys
import csv
import logging

import rosbag2_py
import rclpy.serialization
import utm

from sensor_msgs.msg import NavSatFix

def setup_logger() -> logging.Logger:
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
    try:
        storage_options = rosbag2_py.StorageOptions(uri=bag_folder, storage_id="sqlite3")
        converter_options = rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")

        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
    except Exception as e:
        logger.error("Failed to open rosbag: %s", e)
        return

    logger.info("Reading messages from topic: %s", target_topic)

    try:
        with open(output_file, mode="w", newline="") as csvfile:
            fieldnames = ["timestamp", "latitude", "longitude", "altitude", "easting", "northing", "zone_number", "zone_letter"]
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

def main():
    logger = setup_logger()

    # Resolve path to bag and output file
    package_dir = os.path.dirname(os.path.abspath(__file__))
    db_dir = os.path.join(package_dir, '..', 'db')
    bag_folder = os.path.join(db_dir, "rosbag2_2025_04_04-22_08_55")
    output_file = os.path.join(db_dir, "utm_data.csv")

    if not os.path.exists(bag_folder):
        logger.error("Bag folder does not exist: %s", bag_folder)
        sys.exit(1)

    target_topic = "/swift/navsat_fix"

    convert_and_save_to_utm(bag_folder, target_topic, output_file, logger)

if __name__ == "__main__":
    main()
