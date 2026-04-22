# File: vehicle.launch.py
# Description:
#     차량 인터페이스 — cmd_vel_to_ackermann + carla_ackermann_control
#     Nav2의 /cmd_vel(Twist)을 CARLA 차량이 받을 수 있는 Ackermann 명령으로 변환
#     /cmd_vel → cmd_vel_to_ackermann → /carla/hero/ackermann_cmd → carla_ackermann_control → CARLA
#     Note: 실제 차량으로 교체 시 이 파일만 수정하면 됨

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot        = LaunchConfiguration('robot')

    # vehicle_{robot}.yaml 경로 (설치된 패키지 내부)
    config = PathJoinSubstitution([
        FindPackageShare('vehicle_pkg'), 'config',
        PythonExpression(["'vehicle_", robot, ".yaml'"])
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('robot', default_value='fr09'),

        # carla_ackermann_control
        # /carla/hero/ackermann_cmd(AckermannDrive)를 받아 CARLA 시뮬레이터 차량을 직접 제어
        # role_name: CARLA에서 hero 차량의 이름, control_loop_rate: 제어 루프 주기(s)
        Node(
            package='carla_ackermann_control',
            executable='carla_ackermann_control_node',
            name='carla_ackermann_control',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('carla_ackermann_control'), 'config', 'settings.yaml',
                ]),
                {'role_name': 'hero', 'control_loop_rate': 0.05},
            ],
        ),

        # cmd_vel_to_ackermann
        # /cmd_vel(Twist: linear.x + angular.z) → AckermannDrive(speed + steering_angle)
        Node(
            package='vehicle_pkg',
            executable='cmd_vel_to_ackermann',
            name='cmd_vel_to_ackermann',
            output='screen',
            parameters=[
                config,
                {'use_sim_time': use_sim_time},
            ],
        ),
    ])
