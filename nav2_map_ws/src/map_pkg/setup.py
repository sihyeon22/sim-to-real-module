from setuptools import find_packages, setup

package_name = 'map_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Map
        ('share/' + package_name + '/launch',
            ['launch/map.launch.py']),
        ('share/' + package_name + '/config',
            ['config/map.yaml']),
        ('share/' + package_name + '/maps',
            [
                'maps/parking.yaml',
                'maps/parking.pgm',
                'maps/test_vanjee.yaml',
                'maps/test_vanjee.pgm',
            ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FutureDrive',
    maintainer_email='todo@todo.com',
    description='Modular Nav2 map server package for MK-Mini autonomous driving',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [],
    },
)
