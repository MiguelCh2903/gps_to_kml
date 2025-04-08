#!/usr/bin/env python3
import os
import sys
import logging

import rosbag2_py
import rclpy.serialization
import utm
import simplekml

from sensor_msgs.msg import NavSatFix

def setup_logger() -> logging.Logger:
    """
    Configures and returns a logger for the application.
    """
    logger = logging.getLogger("gps_to_kml")
    logger.setLevel(logging.DEBUG)
    
    # Create a console handler with a debug log level
    ch = logging.StreamHandler()
    ch.setLevel(logging.DEBUG)
    
    # Define a formatter and add it to the handler
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    ch.setFormatter(formatter)
    
    if not logger.handlers:
        logger.addHandler(ch)
        
    return logger

def create_kml(bag_folder: str, target_topic: str, output_kml: str, logger: logging.Logger) -> None:
    """
    Reads NavSatFix messages from a rosbag, converts the GPS coordinates to UTM, 
    and writes the data to a KML file.
    
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
                    
                    # Convert GPS (lat, lon) to UTM
                    utm_coords = utm.from_latlon(msg.latitude, msg.longitude)
                    easting, northing, zone_number, zone_letter = utm_coords
                    
                    # Create a new placemark in KML. KML requires coordinates in (lon, lat, alt)
                    pnt = kml.newpoint(name=str(timestamp),
                                       coords=[(msg.longitude, msg.latitude, msg.altitude)])
                    
                    # Add extended data with the UTM conversion details
                    pnt.extendeddata.newdata(name="easting", value=str(easting))
                    pnt.extendeddata.newdata(name="northing", value=str(northing))
                    pnt.extendeddata.newdata(name="zone_number", value=str(zone_number))
                    pnt.extendeddata.newdata(name="zone_letter", value=str(zone_letter))
                    
                    logger.debug("Processed message: lat=%.6f, lon=%.6f, alt=%.2f | UTM: %.2f, %.2f, %d%s",
                                 msg.latitude, msg.longitude, msg.altitude,
                                 easting, northing, zone_number, zone_letter)
                except Exception as e:
                    logger.error("Error processing message on topic '%s': %s", topic, e)
    except Exception as e:
        logger.error("Error reading messages from bag: %s", e)
    
    try:
        kml.save(output_kml)
        logger.info("KML file saved to: %s", output_kml)
    except Exception as e:
        logger.error("Error saving KML file: %s", e)

def main() -> None:
    logger = setup_logger()

    # Determine the absolute path to the package directory
    package_dir = os.path.dirname(os.path.abspath(__file__))
    # Assume the "db" folder is at the root of the package
    db_dir = os.path.join(package_dir, '..', 'db')
    # Path to the rosbag2 folder inside the "db" directory
    bag_folder = os.path.join(db_dir, "rosbag2_2025_04_04-22_08_55")
    # Define the output KML file path inside the "db" folder
    output_kml = os.path.join(db_dir, "navsat_fix.kml")
    
    if not os.path.exists(bag_folder):
        logger.error("Bag folder does not exist: %s", bag_folder)
        sys.exit(1)
    
    target_topic = "/swift/navsat_fix"
    logger.debug("Resolved bag folder: %s", bag_folder)
    
    create_kml(bag_folder, target_topic, output_kml, logger)

if __name__ == "__main__":
    main()
