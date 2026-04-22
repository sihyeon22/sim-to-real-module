# File: slam.launch.py
# Description:
#     순수 SLAM 모듈
#     Inputs : /scan, /odom, TF(odom→base_link, base_link→os_sensor/os_imu)
#     Outputs: /map, TF(map→odom)
#
#     Note:
#       - perception 모듈이 먼저 실행되어 /scan, /odom, TF를 발행 중이어야 함
#       - sensor_bridge는 시뮬레이션에서만 perception 앞단에 필요
#       - gui는 별도 실행

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params = LaunchConfiguration('slam_params')
    use_teleop = LaunchConfiguration('use_teleop')

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=slam_params,
            param_rewrites={
                'use_sim_time': use_sim_time,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'slam_params',
            default_value=PathJoinSubstitution([
                FindPackageShare('slam_pkg'), 'config', 'slam.yaml'
            ]),
            description='SLAM Toolbox params yaml',
        ),
        DeclareLaunchArgument(
            'use_teleop',
            default_value='true',
            description='Launch slam_pkg teleop_key in a separate terminal window',
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[configured_params],
            remappings=[
                ('scan', '/scan'),
            ],
        ),
        ExecuteProcess(
            condition=IfCondition(use_teleop),
            cmd=[
                'x-terminal-emulator',
                '-e',
                'bash',
                '-lc',
                'ros2 run slam_pkg teleop_key',
            ],
            output='screen',
        ),
    ])
