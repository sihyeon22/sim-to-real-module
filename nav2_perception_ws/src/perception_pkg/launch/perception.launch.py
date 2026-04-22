# File: perception.launch.py
# Description:
#     공통 perception 파이프라인 — odometry, static TF, pointcloud→laserscan
#     Inputs : /raw/points, /raw/imu, /carla/hero/speedometer
#     Outputs: /odom, /scan
#     TF     : odom→base_link, base_link→os_sensor, base_link→os_imu
#
#     Note:
#       - LiDAR/IMU는 실차 naming(/raw/*, os_*) 기준으로 처리
#       - CARLA에서는 sim_bridge_pkg의 sensor bridge를 먼저 실행해 /raw/* 를 만들어야 함
#       - speedometer는 아직 CARLA 토픽을 직접 사용하며, 실차 전환 시 CAN odom 발행기로 교체

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 설치/소스 어느 쪽에서 실행해도 패키지 share 경로 기준으로 config를 찾는다.
PKG_SHARE = get_package_share_directory('perception_pkg')
CONFIG = f'{PKG_SHARE}/config/perception_fr09.yaml'
TF_CONFIG = f'{PKG_SHARE}/config/sensor_fr09.yaml'

# sensor_tf.yaml 로드
# static_transform_publisher는 ROS 파라미터가 아닌 CLI 인수로 좌표를 받으므로
# yaml을 직접 파싱하여 문자열로 전달해야 함.
with open(TF_CONFIG, 'r') as f:
    cfg = yaml.safe_load(f)

lidar_tf = cfg['lidar_tf']
imu_tf   = cfg['imu_tf']


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # speed_imu_odom
        # speedometer + /raw/imu → /odom(Odometry) + TF(odom → base_link)
        Node(
            package='perception_pkg',
            executable='speed_imu_odom',
            name='speed_imu_odom_node',
            output='screen',
            parameters=[
                CONFIG,
                {'use_sim_time': use_sim_time},
            ],
        ),

        # static_transform_publisher: base_link → os_sensor
        # LiDAR가 차체 기준 어디에 달려있는지 TF로 발행 (sensor_tf.yaml에서 로드)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_lidar_tf',
            arguments=[
                '--x',        str(lidar_tf['x']),
                '--y',        str(lidar_tf['y']),
                '--z',        str(lidar_tf['z']),
                '--roll',     str(lidar_tf['roll']),
                '--pitch',    str(lidar_tf['pitch']),
                '--yaw',      str(lidar_tf['yaw']),
                '--frame-id', lidar_tf['frame_id'],
                '--child-frame-id', lidar_tf['child_frame_id'],
            ],
            output='screen',
        ),

        # static_transform_publisher: base_link → os_imu
        # IMU가 차체 기준 어디에 달려있는지 TF로 발행 (sensor_tf.yaml에서 로드)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu_tf',
            arguments=[
                '--x',        str(imu_tf['x']),
                '--y',        str(imu_tf['y']),
                '--z',        str(imu_tf['z']),
                '--roll',     str(imu_tf['roll']),
                '--pitch',    str(imu_tf['pitch']),
                '--yaw',      str(imu_tf['yaw']),
                '--frame-id', imu_tf['frame_id'],
                '--child-frame-id', imu_tf['child_frame_id'],
            ],
            output='screen',
        ),

        # pointcloud_to_laserscan
        # /raw/points (PointCloud2) → /scan (LaserScan)
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            parameters=[
                CONFIG,
                {'use_sim_time': use_sim_time},
            ],
            remappings=[
                ('cloud_in', '/raw/lidar'),
                ('scan',     '/scan'),
            ],
        ),
    ])
