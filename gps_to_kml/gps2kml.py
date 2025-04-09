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
    Lee mensajes NavSatFix de un rosbag y escribe la ruta (línea) en un archivo KML.
    
    :param bag_folder: Ruta absoluta a la carpeta del rosbag2.
    :param target_topic: El tópico a filtrar los mensajes.
    :param output_kml: Ruta del archivo KML de salida.
    :param logger: Instancia del logger para el registro.
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
        logger.error("Error al abrir el rosbag: %s", e)
        return

    logger.info("Procesando mensajes del tópico: %s", target_topic)
    
    # Lista para acumular las coordenadas
    route_coords = []
    
    try:
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            if topic == target_topic:
                try:
                    msg = rclpy.serialization.deserialize_message(data, NavSatFix)
                    # Agrega las coordenadas (lon, lat, alt) a la lista
                    route_coords.append((msg.longitude, msg.latitude, msg.altitude))
                    
                    logger.debug(
                        "Mensaje procesado: lat=%.6f, lon=%.6f, alt=%.2f",
                        msg.latitude, msg.longitude, msg.altitude
                    )
                except Exception as e:
                    logger.error("Error al procesar el mensaje en el tópico '%s': %s", topic, e)
    except Exception as e:
        logger.error("Error al leer mensajes del bag: %s", e)
    
    # Verifica que se hayan obtenido coordenadas antes de crear la ruta
    if route_coords:
        # Crea un objeto Kml
        kml = simplekml.Kml()
        
        # Crea el LineString usando newlinestring, siguiendo el ejemplo de la documentación
        lin = kml.newlinestring(
            name="Ruta",
            description="Ruta generada a partir de mensajes NavSatFix",
            coords=route_coords
        )
        
        # Opcional: personaliza el estilo del LineString
        lin.style.linestyle.color = simplekml.Color.red  # Color rojo
        lin.style.linestyle.width = 3  # Ancho de línea
    else:
        logger.warning("No se encontraron coordenadas para el tópico: %s", target_topic)
        
    try:
        kml.save(output_kml)
        logger.info("Archivo KML guardado en: %s", output_kml)
    except Exception as e:
        logger.error("Error al guardar el archivo KML: %s", e)

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
