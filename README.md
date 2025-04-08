# GPS_TO_KML

**gps_to_kml** es un paquete de ROS2 en Python que provee herramientas para procesar archivos rosbag2 que contienen mensajes de tipo NavSatFix. Este paquete ofrece utilidades para:

- Leer e imprimir datos GPS desde un rosbag.
- Convertir coordenadas GPS a UTM y guardar los resultados en un archivo CSV.
- Convertir datos GPS a UTM y generar un archivo KML para visualización geográfica.

El paquete está diseñado siguiendo buenas prácticas de programación, con manejo robusto de errores y registro detallado (logging) para facilitar la depuración y el desarrollo.

## Tabla de Contenidos

- [Características](#características)
- [Instalación](#instalación)
- [Estructura del Paquete](#estructura-del-paquete)
- [Uso](#uso)
  - [Lectura del Rosbag](#lectura-del-rosbag)
  - [Conversión a UTM (CSV)](#conversión-a-utm-csv)
  - [Conversión a KML](#conversión-a-kml)
- [Visualización Ejemplo de KML](#visualización-ejemplo-de-kml)
- [Dependencias](#dependencias)
- [Licencia](#licencia)

## Características

- **Lector de Rosbag:** Lee mensajes NavSatFix desde un rosbag e imprime los datos GPS.
- **Conversor a UTM:** Convierte las coordenadas GPS a UTM y exporta los resultados a un archivo CSV.
- **Generador de KML:** Convierte los datos GPS a UTM y genera un archivo KML para su visualización en aplicaciones geográficas (por ejemplo, Google Earth).
- **Manejo de Errores y Logging:** Proporciona un registro detallado de la ejecución y manejo de errores para facilitar la depuración.

## Instalación

1. **Clonar el Repositorio:**

   ```bash
   cd ~/ros2_ws/src
   git clone <url-del-repositorio> tu_package
   ```

2. **Instalar Dependencias:**
3. 
