"""
동적 장애물 시나리오 스크립트
- NPC 차량을 주차 구역에서 스폰하여 중앙 주행로를 가로지르게 제어
- hero 차량의 동적 장애물 회피 성능을 검증하기 위한 용도
"""

import carla
import math
import time
import argparse


# ────────────────────────────────────────────────
# 시나리오 파라미터 (필요시 수정)
# ────────────────────────────────────────────────

# NPC 스폰 좌표 (dynamic_obstacle.json 기준)
NPC_SPAWN = {
    "x": 2.5,
    "y": 28.0,
    "z": 0.3,
    "yaw": 0.0,   # +Y 방향(도로 쪽)으로 전진
}

# 주행 경유점: 주차 구역 → 중앙로 진입 → 반대편 주차 구역
# y값을 맵 구조에 맞게 수정하세요
WAYPOINTS = [
    carla.Location(x=8.5, y=28.0, z=0.3),      # 중앙 주행로 진입 (ego 경로 횡단)
    carla.Location(x=14.5, y=28.0, z=0.3),     # 도로 횡단 완료
]

# 주행 속도 제어
THROTTLE     = 0.11         # 가속 강도 (0.0 ~ 1.0)
TARGET_SPEED = 0.4          # 목표 속도 (m/s)
BRAKE_GAIN   = 0.3          # 목표 속도 초과 시 제동 강도
ARRIVAL_DIST = 0.8          # 경유점 도달 판정 반경 (m)
STEER_GAIN   = 1.0 / 60.0  # 조향 게인 (각도 오차 → steer 값 변환)
STEER_MAX    = 0.5          # 최대 조향 제한

# ────────────────────────────────────────────────


def compute_steer(transform: carla.Transform, target: carla.Location) -> float:
    """현재 transform과 목표 위치 기반으로 조향각 계산"""
    loc = transform.location
    dx = target.x - loc.x
    dy = target.y - loc.y

    # 목표 방향 절대 yaw (CARLA 기준: X+ = 0도, 반시계 방향 +)
    target_yaw = math.degrees(math.atan2(dy, dx))
    current_yaw = transform.rotation.yaw

    # 각도 차이를 -180 ~ +180으로 정규화
    diff = (target_yaw - current_yaw + 180) % 360 - 180

    steer = diff * STEER_GAIN
    return max(-STEER_MAX, min(STEER_MAX, steer))


def distance_2d(loc: carla.Location, target: carla.Location) -> float:
    """XY 평면 2D 거리 계산"""
    return math.sqrt((loc.x - target.x) ** 2 + (loc.y - target.y) ** 2)


def run_scenario(host: str = "localhost", port: int = 2000):
    client = carla.Client(host, port)
    client.set_timeout(10.0)
    world = client.get_world()

    blueprint_library = world.get_blueprint_library()

    # NPC 차량 블루프린트 설정
    bp = blueprint_library.find("vehicle.tesla.model3")
    bp.set_attribute("role_name", "obstacle1")

    spawn_transform = carla.Transform(
        carla.Location(x=NPC_SPAWN["x"], y=NPC_SPAWN["y"], z=NPC_SPAWN["z"]),
        carla.Rotation(yaw=NPC_SPAWN["yaw"]),
    )

    # NPC 스폰
    npc = world.try_spawn_actor(bp, spawn_transform)
    if npc is None:
        print("[ERROR] NPC 스폰 실패 - 좌표 충돌 또는 맵 범위 초과")
        return

    print(f"[INFO] NPC 스폰 완료: id={npc.id} (위치는 tick 후 반영됨)")

    # 물리 활성화 명시 및 위치 반영 대기
    npc.set_simulate_physics(True)
    time.sleep(0.5)
    print(f"[INFO] NPC 실제 위치 확인: {npc.get_location()}")

    try:
        # 경유점 순서대로 이동
        for idx, waypoint in enumerate(WAYPOINTS):
            print(f"[INFO] 경유점 {idx + 1}/{len(WAYPOINTS)} 이동 중: {waypoint}")

            step = 0
            while True:
                time.sleep(0.05)  # 20Hz 루프 (ROS Bridge tick 주기와 맞춤)

                tf = npc.get_transform()
                dist = distance_2d(tf.location, waypoint)

                # 10 step마다 위치 출력 (이동 여부 확인용)
                if step % 10 == 0:
                    print(f"  [DEBUG] loc={tf.location}, dist={dist:.2f}m")
                step += 1

                if dist < ARRIVAL_DIST:
                    print(f"[INFO] 경유점 {idx + 1} 도달 (잔여 거리: {dist:.2f}m)")
                    break

                steer = compute_steer(tf, waypoint)
                vel = npc.get_velocity()
                current_speed = math.sqrt(vel.x**2 + vel.y**2)
                if current_speed < TARGET_SPEED:
                    throttle, brake = THROTTLE, 0.0
                else:
                    throttle, brake = 0.0, BRAKE_GAIN
                control = carla.VehicleControl(
                    throttle=throttle,
                    steer=steer,
                    brake=brake,
                    hand_brake=False,
                )
                npc.apply_control(control)

        # 최종 정지
        npc.apply_control(carla.VehicleControl(brake=1.0, throttle=0.0))
        print("[INFO] 시나리오 완료 - NPC 정지 (Ctrl+C로 종료 시 차량 제거)")

        # NPC를 시뮬레이터에 유지한 채 대기
        while True:
            time.sleep(1.0)

    except KeyboardInterrupt:
        print("[INFO] 사용자 중단 - NPC 제거")

    finally:
        npc.apply_control(carla.VehicleControl(brake=1.0))
        npc.destroy()
        print(f"[INFO] NPC actor 제거: id={npc.id}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="동적 장애물 시나리오 실행")
    parser.add_argument("--host", default="localhost", help="CARLA 서버 IP")
    parser.add_argument("--port", type=int, default=2000, help="CARLA 서버 포트")
    args = parser.parse_args()

    run_scenario(host=args.host, port=args.port)
