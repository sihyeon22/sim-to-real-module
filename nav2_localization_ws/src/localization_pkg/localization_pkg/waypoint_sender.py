#!/usr/bin/env python3
# File: waypoint_sender.py
# Description:
#     WAYPOINTS 리스트에 정의된 목표 지점을 순서대로 navigate_to_pose 액션으로 전송하는 유틸 노드.
#     한 목표에 도달(결과 수신)하면 자동으로 다음 목표를 전송하고, 모두 완료되면 종료.
#     waypoint 좌표는 RViz에서 /amcl_pose를 확인하거나 ros2 topic echo /amcl_pose로 기록.

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


# Waypoint 목록
# 형식: (x, y, qz, qw) — map 프레임 기준 위치와 방향 (quaternion의 z, w 성분)
# qx=0, qy=0 고정 (2D 평면 운동)
WAYPOINTS = [
    (46.626,  0.0,    -0.131, 0.991),   # WP1
    (48.180, -16.008, -0.892, 0.452),   # WP2
    (11.741, -17.762,  0.991, 0.131),   # WP3
    (20.609,  -0.391, -0.020, 1.000),   # WP4
]


class WaypointSender(Node):
    def __init__(self):
        super().__init__('waypoint_sender')

        # navigate_to_pose 액션 클라이언트 (bt_navigator가 서버)
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._waypoints = WAYPOINTS
        self._index = 0   # 현재 전송 중인 waypoint 인덱스

    def send_next(self):
        """다음 waypoint 목표 전송. 모두 완료되면 노드 종료."""
        if self._index >= len(self._waypoints):
            self.get_logger().info('모든 waypoint 완료!')
            rclpy.shutdown()
            return

        x, y, qz, qw = self._waypoints[self._index]
        qx, qy = 0.0, 0.0

        # PoseStamped 목표 생성
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(
            f'[{self._index + 1}/{len(self._waypoints)}] 목표 전송: ({x}, {y})'
        )

        # 서버(bt_navigator) 준비 대기 후 goal 전송
        self._client.wait_for_server()
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        """goal 수락 여부 확인 후 result 콜백 등록"""
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Goal 거절됨')
            return
        handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        """목표 도달 결과 수신 → 다음 waypoint 전송"""
        self._index += 1
        self.send_next()


def main():
    rclpy.init()
    node = WaypointSender()
    node.get_logger().info('Waypoint sender 시작')
    node.send_next()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
