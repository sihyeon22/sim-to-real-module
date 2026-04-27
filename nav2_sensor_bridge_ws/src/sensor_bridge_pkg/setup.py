import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'sensor_bridge_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'mkmini'), glob('config/mkmini/*.yaml')),
        (os.path.join('share', package_name, 'config', 'fr09'), glob('config/fr09/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FutureDrive',
    maintainer_email='chyt5355@gmail.com',
    description='Sensor topic bridge supporting multiple vehicle models and LiDAR types',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sensor_bridge = sensor_bridge_pkg.sensor_bridge:main',
        ],
    },
)
