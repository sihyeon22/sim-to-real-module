#!/usr/bin/env python3
# File: sensor_bridge.py
# Description:
#     센서 토픽 -> 공용 토픽 전환 브리지 노드
#
#     sim_mode=True  (시뮬레이션)
#       LiDAR: /carla/hero/lidar       -> /raw/lidar  (frame: os_lidar)
#       IMU  : /carla/hero/imu         -> /raw/imu    (frame: os_imu)
#
#     sim_mode=False (실차)
#       LiDAR: /sensing/lidar/top/pointcloud -> /raw/lidar  (frame 유지 — /tf_static 사용)
#       IMU  : /sensing/imu/imu_data         -> /raw/imu    (frame 유지 — /tf_static 사용)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, Imu


class SensorBridge(Node):
    def __init__(self):
        super().__init__('sensor_bridge')

        # sim_mode에 따라 센서 토픽/프레임 기본값 결정
        self.declare_parameter('sim_mode', True)
        sim_mode = self.get_parameter('sim_mode').value

        if sim_mode:
            default_lidar_in    = '/carla/hero/lidar'
            default_lidar_frame = 'os_lidar'
            default_imu_in      = '/carla/hero/imu'
            default_imu_frame   = 'os_imu'
        else:
            default_lidar_in    = '/sensing/lidar/top/pointcloud'
            default_lidar_frame = ''   # 실차는 /tf_static이 TF를 제공하므로 덮어쓰지 않음
            default_imu_in      = '/sensing/imu/imu_data'
            default_imu_frame   = ''

        # 파라미터 선언 (launch 파일에서 개별 덮어쓰기 가능)
        self.declare_parameter('lidar_input',  default_lidar_in)
        self.declare_parameter('lidar_output', '/raw/lidar')
        self.declare_parameter('lidar_frame',  default_lidar_frame)

        self.declare_parameter('imu_input',  default_imu_in)
        self.declare_parameter('imu_output', '/raw/imu')
        self.declare_parameter('imu_frame',  default_imu_frame)

        lidar_in  = self.get_parameter('lidar_input').value
        lidar_out = self.get_parameter('lidar_output').value
        self.lidar_frame = self.get_parameter('lidar_frame').value

        imu_in  = self.get_parameter('imu_input').value
        imu_out = self.get_parameter('imu_output').value
        self.imu_frame = self.get_parameter('imu_frame').value

        # 입력 센서 QoS: 시뮬은 Reliable, 실차 센서는 Best Effort
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE if sim_mode else ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # 출력 QoS: downstream(Nav2 등)이 Reliable을 기대하므로 항상 Reliable 고정
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Publisher / Subscriber
        self.lidar_pub = self.create_publisher(PointCloud2, lidar_out, pub_qos)
        self.imu_pub   = self.create_publisher(Imu,         imu_out,   pub_qos)

        self.create_subscription(PointCloud2, lidar_in, self.lidar_cb, sub_qos)
        self.create_subscription(Imu,         imu_in,   self.imu_cb,   sub_qos)

        mode_str = 'simulation' if sim_mode else 'real'
        self.get_logger().info(f'SensorBridge started [{mode_str} mode]')
        self.get_logger().info(f'  LiDAR: {lidar_in} -> {lidar_out}  ')
        self.get_logger().info(f'  IMU  : {imu_in}   -> {imu_out}  ')

    def lidar_cb(self, msg: PointCloud2):
        # 실차 모드에서는 frame_id를 덮어쓰지 않음 (/tf_static이 TF를 제공)
        if self.lidar_frame:
            msg.header.frame_id = self.lidar_frame
        self.lidar_pub.publish(msg)

    def imu_cb(self, msg: Imu):
        # 실차 모드에서는 frame_id를 덮어쓰지 않음 (/tf_static이 TF를 제공)
        if self.imu_frame:
            msg.header.frame_id = self.imu_frame
        self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SensorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
