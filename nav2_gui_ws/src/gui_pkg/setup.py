from setuptools import find_packages, setup

package_name = 'gui_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/gui_pkg']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gui.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/carla.rviz', 'rviz/slam.rviz', 'rviz/fixed_path.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FutureDrive',
    maintainer_email='todo@todo.com',
    description='GUI module for launching RViz in the modular Nav2 stack',
    license='Apache-2.0',
    entry_points={'console_scripts': []},
)
