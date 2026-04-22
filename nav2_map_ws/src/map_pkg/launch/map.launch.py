# File: map.launch.py
# Description:
#     맵 서버 — map_server
#     저장된 지도를 로드하여 /map 토픽으로 발행
#     Publishes : /map
#     Note: localization(AMCL)보다 먼저 실행되어야 함

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_name     = LaunchConfiguration('map_name')

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=PathJoinSubstitution([
                FindPackageShare('map_pkg'), 'config', 'map.yaml'
            ]),
            param_rewrites={
                'use_sim_time': use_sim_time,
                'yaml_filename': PathJoinSubstitution([
                    FindPackageShare('map_pkg'), 'maps',
                    PythonExpression(["'", map_name, ".yaml'"])
                ]),
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'map_name',
            default_value='parking',
            description='Map name to load (file must exist in maps/ as <map_name>.yaml)',
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server'],
            }],
        ),
    ])