from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gps_to_kml'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='miguel',
    maintainer_email='miguel.chumacero.b@uni.pe',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reader = gps_to_kml.reader:main',
            'gps2utm = gps_to_kml.gps2utm:main',
            "gps_to_kml = gps_to_kml.gps2kml:main"
        ],
    },
)
