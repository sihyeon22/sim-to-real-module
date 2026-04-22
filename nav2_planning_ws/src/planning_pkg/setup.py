from setuptools import find_packages, setup

package_name = 'planning_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/planning_pkg']),
        ('share/' + package_name, ['package.xml']),

        # Planning
        ('share/' + package_name + '/launch', [
            'launch/planning.launch.py'
        ]),
        ('share/' + package_name + '/config', [
            'config/planning_2d_fr09.yaml',
            'config/planning_3d_fr09.yaml'
        ]),

        # Planning BT XML
        ('share/' + package_name + '/behavior', [
            'behavior/navigate_to_pose_no_spin.xml',
            'behavior/navigate_through_poses_no_spin.xml',
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
            'waypoint_sender = planning_pkg.waypoint_sender:main',
        ],
    },
)
