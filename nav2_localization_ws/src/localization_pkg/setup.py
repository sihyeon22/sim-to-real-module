from setuptools import find_packages, setup

package_name = 'localization_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/localization_pkg']),
        ('share/' + package_name, ['package.xml']),

        # Localization
        ('share/' + package_name + '/launch',
            ['launch/localization.launch.py']),
        ('share/' + package_name + '/config',
            ['config/localization.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FutureDrive',
    maintainer_email='todo@todo.com',
    description='Modular Nav2 launch package for FR-09/MK-Mini autonomous driving',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'waypoint_sender = nav2_pkg_nodes.waypoint_sender:main',
        ],
    },
)
