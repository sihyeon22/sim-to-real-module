# File: localization.launch.py
# Description:
#     위치 추정 스택 — map_server + AMCL
#     사전에 저장된 지도(/map)와 실시간 스캔(/scan)을 비교해 지도 위 차량 위치 추정
#     Subscribes : /scan, /odom_wheel
#     Publishes  : TF(map→odom), /amcl_pose, /map
#     Requires   : perception.launch.py가 먼저 실행되어 /scan, TF(odom→base_link) 발행 중이어야 함

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml
from launch_ros.descriptions import ParameterFile


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    nav2_params  = LaunchConfiguration('nav2_params')

    # 파라미터 파일 Runtime Override
    # RewrittenYaml: yaml 파일을 읽어 특정 키를 런타임 값으로 교체한 임시 파일 생성
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=nav2_params,
            param_rewrites={
                'use_sim_time': use_sim_time,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # 파라미터 파일 경로 — 기본값은 패키지 내 localization_params.yaml
        DeclareLaunchArgument(
            'nav2_params',
            default_value=PathJoinSubstitution([
                FindPackageShare('localization_pkg'), 'config', 'localization.yaml'
            ]),
            description='Localization params yaml',
        ),

        # amcl
        # Monte Carlo 파티클 필터로 지도 위 차량 위치 추정
        # /scan과 /odom_wheel을 입력으로 받아 TF(map→odom)와 /amcl_pose 발행
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[configured_params],
        ),

        # lifecycle_manager_localization
        # map_server와 amcl은 ROS2 lifecycle 노드 → 직접 activate하지 않으면 동작 안 함
        # autostart=True로 설정하면 이 매니저가 자동으로 두 노드를 활성화함
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['amcl'],
            }],
        ),
    ])
