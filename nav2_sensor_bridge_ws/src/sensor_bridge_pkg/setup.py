from setuptools import find_packages, setup

package_name = 'sensor_bridge_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sensor_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FutureDrive',
    maintainer_email='todo@todo.com',
    description='Simulation-only bridge package for CARLA sensor topic normalization',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sensor_bridge = sensor_bridge_pkg.sensor_bridge:main',
        ],
    },
)
