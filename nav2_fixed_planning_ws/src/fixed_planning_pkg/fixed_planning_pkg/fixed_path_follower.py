#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav2_msgs.srv import ClearEntireCostmap
from tf2_ros import Buffer, TransformListener


class FixedPathFollower(Node):
    def __init__(self):
        super().__init__('fixed_path_follower')

        cb_group = ReentrantCallbackGroup()

        self._goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self._goal_cb,
            10,
            callback_group=cb_group,
        )

        self._planner_client = ActionClient(
            self, ComputePathToPose, 'compute_path_to_pose',
            callback_group=cb_group,
        )
        self._controller_client = ActionClient(
            self, FollowPath, 'follow_path',
            callback_group=cb_group,
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._clear_local_costmap = self.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap',
            callback_group=cb_group,
        )
        self._clear_global_costmap = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap',
            callback_group=cb_group,
        )

        self._fixed_path = None     # planner가 계산한 고정 경로 (메모리 저장)
        self._last_index = 0        # 마지막으로 추종하던 경로 인덱스 (역주행 방지)
        self._retry_timer = None    # 장애물 대기 타이머
        self._post_clear_timer = None  # costmap clear 후 재시도 타이머


    # 1. Goal 수신
    def _goal_cb(self, goal_pose: PoseStamped):
        self.get_logger().info(
            f'Goal 수신: ({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})'
        )
        self._fixed_path = None         # 이전 경로 초기화
        self._last_index = 0            # 경로 인덱스 초기화
        self._cancel_all_timers()
        self._compute_path(goal_pose)   # 경로 계산 시작 -> planner의 ComputePathPose action


    # 2. 경로 계산 (planner_server, 1회)
    def _compute_path(self, goal_pose: PoseStamped):
        self._planner_client.wait_for_server()
        goal = ComputePathToPose.Goal()     # aciton goal 전송
        goal.goal = goal_pose
        goal.planner_id = 'GridBased'
        future = self._planner_client.send_goal_async(goal)
        future.add_done_callback(self._path_response_cb)

    def _path_response_cb(self, future):    # goal 수락 여부 확인
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('경로 계산 요청 거절됨')
            return
        handle.get_result_async().add_done_callback(self._path_result_cb)

    def _path_result_cb(self, future):  # 경로 결과 수신
        status = future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error('경로 계산 실패')
            return
        self._fixed_path = future.result().result.path  # nav_msgs/Path 저장
        self._last_index = 0
        self.get_logger().info(f'경로 생성 완료: {len(self._fixed_path.poses)}개 포인트')
        self._follow_path(self._fixed_path) # 추종 시작


    # 3. 경로 추종 (controller_server)
    def _follow_path(self, path: Path):
        self._controller_client.wait_for_server()
        goal = FollowPath.Goal()    # action goal 전송
        goal.path = path
        goal.controller_id = 'FollowPath'
        future = self._controller_client.send_goal_async(goal)
        future.add_done_callback(self._follow_response_cb)

    def _follow_response_cb(self, future):  # goal 수락 여부 확인
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Follow path 요청 거절됨')
            return
        handle.get_result_async().add_done_callback(self._follow_result_cb)

    def _follow_result_cb(self, future):    # 결과 수신 (SUCCESS or ABORTED)
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('주행 완료')
            return
        # STATUS_ABORTED: 장애물 감지 → 대기 후 재시도
        self.get_logger().warn('장애물 감지 - 2초 후 재시도')
        self._schedule_retry()


    # 4. 장애물 대기 및 재시도
    def _schedule_retry(self):
        self._cancel_all_timers()
        self._retry_timer = self.create_timer(2.0, self._clear_and_retry)

    def _clear_and_retry(self):
        self._cancel_all_timers()
        req = ClearEntireCostmap.Request()
        if self._clear_local_costmap.service_is_ready():
            self._clear_local_costmap.call_async(req)
            self.get_logger().info('Local Costmap 초기화')
        if self._clear_global_costmap.service_is_ready():
            self._clear_global_costmap.call_async(req)
            self.get_logger().info('Global Costmap 초기화')
        # 0.5초 대기 후 follow_path 실행 (scan으로 costmap 재구성 시간 확보)
        self._post_clear_timer = self.create_timer(0.5, self._retry_follow)

    def _retry_follow(self):
        self._cancel_all_timers()
        remaining = self._get_remaining_path(self._fixed_path)
        self.get_logger().info(f'재시도: {len(remaining.poses)}개 포인트 남음')
        self._follow_path(remaining)

    def _cancel_all_timers(self):
        for attr in ('_retry_timer', '_post_clear_timer'):
            timer = getattr(self, attr, None)
            if timer is not None:
                timer.cancel()
                setattr(self, attr, None)


    # 5. 남은 경로 계산
    def _get_remaining_path(self, full_path: Path) -> Path:
        current_pose = self._get_current_pose()

        if current_pose is None:
            remaining = Path()
            remaining.header = full_path.header
            remaining.poses = full_path.poses[self._last_index:]
            return remaining

        # 마지막 인덱스 기준 앞방향만 탐색 (로컬라이제이션 오차로 인한 역주행 방지)
        search_end = min(len(full_path.poses), self._last_index + 50)
        min_dist = float('inf')
        closest_idx = self._last_index

        for i in range(self._last_index, search_end):
            pose = full_path.poses[i]
            dx = pose.pose.position.x - current_pose.position.x
            dy = pose.pose.position.y - current_pose.position.y
            dist = math.hypot(dx, dy)
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        self._last_index = closest_idx

        remaining = Path()
        remaining.header = full_path.header
        remaining.poses = full_path.poses[closest_idx:]
        return remaining

    def _get_current_pose(self) -> Pose:
        try:
            t = self._tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            pose = Pose()
            pose.position.x = t.transform.translation.x
            pose.position.y = t.transform.translation.y
            return pose
        except Exception as e:
            self.get_logger().warn(f'TF lookup 실패: {e}')
            return None


def main():
    rclpy.init()
    node = FixedPathFollower()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()


if __name__ == '__main__':
    main()
