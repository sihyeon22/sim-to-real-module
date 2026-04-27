from setuptools import find_packages, setup

package_name = 'fixed_planning_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/fixed_planning_pkg']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/fixed_path.launch.py']),
        ('share/' + package_name + '/config', ['config/fixed_planning_2d_fr09.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FutureDrive',
    maintainer_email='todo@todo.com',
    description='Fixed path following with obstacle wait for FR-09',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fixed_path_follower = fixed_planning_pkg.fixed_path_follower:main',
        ],
    },
)
