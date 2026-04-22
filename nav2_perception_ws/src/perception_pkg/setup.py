from setuptools import find_packages, setup

package_name = 'perception_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/perception_pkg']),
        ('share/' + package_name, ['package.xml']),

        # Perception
        ('share/' + package_name + '/launch', [
            'launch/perception.launch.py'
        ]),
        ('share/' + package_name + '/config', [
                'config/perception_fr09.yaml',
                'config/sensor_fr09.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FutureDrive',
    maintainer_email='todo@todo.com',
    description='Modular Nav2 launch package for FR-09/MK-Mini autonomous driving',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'speed_imu_odom = perception_pkg.odometry:main',
            'waypoint_sender = perception_pkg.waypoint_sender:main',
        ],
    },
)
