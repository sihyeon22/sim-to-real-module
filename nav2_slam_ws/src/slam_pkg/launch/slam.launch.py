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
#       - 차량 및 센서 모델 추가 시 ARGUMENTS의 choices 목록에 추가

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


ARGUMENTS = [
    DeclareLaunchArgument('sim_mode', default_value='true'),
    DeclareLaunchArgument('lidar', default_value='ouster', choices=['ouster', 'vanjee']),
]


def launch_setup(context, *args, **kwargs):
    sim_mode = LaunchConfiguration('sim_mode').perform(context)
    lidar    = LaunchConfiguration('lidar').perform(context)

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=PathJoinSubstitution([
                FindPackageShare('slam_pkg'), 'config', f'slam_{lidar}.yaml'
            ]),
            param_rewrites={'use_sim_time': sim_mode},
            convert_types=True,
        ),
        allow_substs=True,
    )

    actions = [
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[configured_params],
            remappings=[('scan', '/scan')],
        ),
    ]

    if sim_mode == 'true':
        actions.append(ExecuteProcess(
            cmd=['x-terminal-emulator', '-e', 'bash', '-lc', 'ros2 run slam_pkg teleop_key'],
            output='screen',
        ))

    return actions


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
