#!/usr/bin/env python3
# File: odometry.py
# Description:
#     speedometer(Float32) + IMU를 융합하여 /odom(Odometry)과
#     TF(odom → base_link)를 발행하는 dead reckoning 노드.
#     파라미터로 토픽명과 프레임명을 주입할 수 있으며,
#     실차 naming(/raw/imu, /odom)에 맞춰 launch에서 remap해 사용한다.

import math
from typing import Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster


# 좌표 변환 유틸

def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """쿼터니언 → yaw(rad) 변환"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """yaw(rad) → 쿼터니언 변환 (2D 평면 운동 가정, roll/pitch = 0)"""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def normalize_angle(angle: float) -> float:
    """각도를 [-π, π] 범위로 정규화"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


# 메인 노드

class SpeedImuOdomNode(Node):
    def __init__(self):
        super().__init__('speed_imu_odom_node')

        # 파라미터 선언 및 로드
        # perception_params.yaml의 speed_imu_odom_node 섹션에서 읽어옴
        self.declare_parameter('speed_topic', '/hero/speedometer')
        self.declare_parameter('imu_topic', '/hero/imu')
        self.declare_parameter('clock_topic', '/clock')
        self.declare_parameter('odom_topic', '/hero/wheel_odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('use_imu_angular_velocity', True)
        self.declare_parameter('invert_speed', False)

        self.speed_topic = self.get_parameter('speed_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.clock_topic = self.get_parameter('clock_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.use_imu_angular_velocity = self.get_parameter('use_imu_angular_velocity').value
        self.invert_speed = self.get_parameter('invert_speed').value

        # 내부 상태 변수
        self.speed_mps: float = 0.0          # 현재 속도 (m/s)
        self.current_yaw: Optional[float] = None
        self.initial_yaw: Optional[float] = None
        self.last_yaw: Optional[float] = None
        self.last_time: Optional[float] = None
        self.last_clock_msg = None

        self.x = 0.0   # odom 프레임 기준 x 위치 (m)
        self.y = 0.0   # odom 프레임 기준 y 위치 (m)

        self.has_speed = False
        self.has_imu = False

        self.last_imu_msg: Optional[Imu] = None

        # Publisher / Subscriber
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Float32, self.speed_topic, self.speed_cb, 10)
        self.create_subscription(Imu, self.imu_topic, self.imu_cb, 50)
        # clock을 트리거로 사용하여 시뮬레이션 시간 기준으로 적분
        self.create_subscription(Clock, self.clock_topic, self.clock_cb, 100)

        self.get_logger().info('SpeedImuOdomNode started')
        self.get_logger().info(f'speed_topic: {self.speed_topic}')
        self.get_logger().info(f'imu_topic:   {self.imu_topic}')
        self.get_logger().info(f'clock_topic: {self.clock_topic}')
        self.get_logger().info(f'odom_topic:  {self.odom_topic}')
        self.get_logger().info(f'publish_tf:  {self.publish_tf}')

    # 콜백

    def speed_cb(self, msg: Float32):
        """speedometer 수신 → 속도 저장 (invert_speed=True면 부호 반전)"""
        speed = float(msg.data)
        if self.invert_speed:
            speed = -speed
        self.speed_mps = speed
        self.has_speed = True

    def imu_cb(self, msg: Imu):
        """
        IMU 수신 → yaw 갱신
        - use_imu_angular_velocity=False: IMU orientation에서 직접 yaw 추출 (CARLA 시뮬용)
        - use_imu_angular_velocity=True : angular_velocity.z를 적분하여 yaw 계산 (실차용)
        """
        self.last_imu_msg = msg

        if not self.use_imu_angular_velocity:
            # 절대 yaw 모드: CARLA가 제공하는 ground truth orientation 사용
            q = msg.orientation
            yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
            if self.initial_yaw is None:
                self.initial_yaw = yaw
            self.current_yaw = normalize_angle(yaw - self.initial_yaw)
        else:
            # 상대 yaw 모드: IMU angular_velocity.z를 시간 적분 -> 실차에 적합
            stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            if self.current_yaw is None:
                self.current_yaw = 0.0
                self._last_imu_stamp = stamp
            else:
                dt = stamp - self._last_imu_stamp
                if 0.0 < dt < 0.5:
                    self.current_yaw = normalize_angle(
                        self.current_yaw + msg.angular_velocity.z * dt
                    )
                self._last_imu_stamp = stamp

        self.has_imu = True

    def clock_cb(self, msg: Clock):
        """
        /clock 수신마다 위치 적분 및 odometry 발행
        - dt가 0.5s 초과 시 (일시정지 후 재개 등) 적분 skip
        - speed와 imu 데이터가 모두 준비된 경우에만 적분 실행
        """
        sim_time = float(msg.clock.sec) + float(msg.clock.nanosec) * 1e-9
        self.last_clock_msg = msg

        if self.last_time is None:
            self.last_time = sim_time
            if self.current_yaw is not None:
                self.last_yaw = self.current_yaw
            return

        dt = sim_time - self.last_time
        if dt <= 0.0:
            return

        # 너무 긴 공백 구간은 적분 오차 방지를 위해 skip
        if dt > 0.5:
            self.last_time = sim_time
            if self.current_yaw is not None:
                self.last_yaw = self.current_yaw
            return

        self.last_time = sim_time

        if not self.has_speed or not self.has_imu or self.current_yaw is None:
            return

        yaw = self.current_yaw
        v = self.speed_mps

        # 속도와 yaw로 x, y 위치 적분 (오일러 적분)
        self.x += v * math.cos(yaw) * dt
        self.y += v * math.sin(yaw) * dt

        # 각속도 계산: IMU angular_velocity 직접 사용 또는 yaw 차분으로 추정
        omega = 0.0
        if self.use_imu_angular_velocity and self.last_imu_msg is not None:
            omega = float(self.last_imu_msg.angular_velocity.z)
        elif self.last_yaw is not None:
            dyaw = normalize_angle(yaw - self.last_yaw)
            omega = dyaw / dt

        self.last_yaw = yaw

        self.publish_odom(msg.clock, yaw, v, omega)

    def publish_odom(self, stamp, yaw: float, v: float, omega: float):
        """
        Odometry 메시지 발행 및 TF(odom → base_link) broadcast
        - covariance: x/y/yaw에만 유효한 값, 나머지(z, roll, pitch)는 999로 무효 처리
        """
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = yaw_to_quaternion(yaw)

        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = omega

        # 6x6 covariance (x, y, z, roll, pitch, yaw 순서)
        # z/roll/pitch는 2D 주행에서 의미 없으므로 999(매우 큰 불확실도)로 설정
        odom.pose.covariance = [
            0.05, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.05, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  999.0,0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  999.0,0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  999.0,0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.02
        ]

        odom.twist.covariance = [
            0.02, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.2,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  999.0,0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  999.0,0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  999.0,0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.05
        ]

        self.odom_pub.publish(odom)

        # publish_tf=True일 때 odom → base_link TF도 함께 발행
        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation = yaw_to_quaternion(yaw)
            self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SpeedImuOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
