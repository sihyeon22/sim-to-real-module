# File: sensor_bridge.launch.py
# Description:
#   - sim_mode / robot / lidar 에 따라서 소프트웨어에서 사용하는 공용 토픽으로 처리
#   - 차량 및 센서 모델 추가 시 choices 목록에 추가

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

import os


ARGUMENTS = [
    DeclareLaunchArgument('sim_mode', default_value='true'),
    DeclareLaunchArgument('robot', default_value='mkmini', choices=['mkmini', 'fr09']),
    DeclareLaunchArgument('lidar', default_value='vanjee', choices=['vanjee', 'ouster']),
]

def set_node(context, *args, **kwargs):
    sim_mode = LaunchConfiguration('sim_mode').perform(context)
    robot    = LaunchConfiguration('robot').perform(context)
    lidar    = LaunchConfiguration('lidar').perform(context)

    if sim_mode == 'true':
        params_file = os.path.join(
            os.path.dirname(__file__), '..', 'config', 'sensor_bridge_carla.yaml'
        )
    else:
        params_file = os.path.join(
            os.path.dirname(__file__), '..', 'config', robot, f'sensor_bridge_{lidar}.yaml'
        )
        
    node = Node(
        package='sensor_bridge_pkg',
        executable='sensor_bridge',
        name='sensor_bridge',
        output='screen',
        parameters=[params_file],
    )
    return [node]

def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=set_node))

    return ld
