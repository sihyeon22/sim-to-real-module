from setuptools import find_packages, setup

package_name = 'slam_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # SLAM
        ('share/' + package_name + '/launch',
            ['launch/slam.launch.py']),
        ('share/' + package_name + '/config',
            ['config/slam_ouster.yaml', 'config/slam_vanjee.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FutureDrive',
    maintainer_email='chyt5355@gmail.com',
    description='SLAM package supporting multiple vehicle models and LiDAR types',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'teleop_key = slam_pkg.teleop_key:main',
        ],
    },
)
