from setuptools import find_packages, setup

package_name = 'control_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/control_pkg']),
        ('share/' + package_name, ['package.xml']),

        # Control
        ('share/' + package_name + '/launch', [
            'launch/control.launch.py'
        ]),
        ('share/' + package_name + '/config', [
            'config/control_2d_fr09.yaml',
            'config/control_3d_fr09.yaml'
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
            'waypoint_sender = control_pkg.waypoint_sender:main',
        ],
    },
)
