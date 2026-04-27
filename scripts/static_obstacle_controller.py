"""
정적 장애물 시나리오 스크립트
- NPC 차량을 자율주행 경로 위에 스폰
- hero 차량의 정적 장애물 회피 성능을 검증하기 위한 용도
"""

import carla
import time
import argparse


# NPC 스폰 좌표 (dynamic_obstacle.json 기준)
NPC_SPAWN = {
    "x": 8.5,
    "y": 28.0,
    # "x": -10.0,
    # "y": 20.0,
    "z": 0.3,
    "yaw": 0.0,   # +Y 방향(도로 쪽)으로 전진
}


def spawn_static_obstacle(host: str = "localhost", port: int = 2000):
    client = carla.Client(host, port)
    client.set_timeout(10.0)
    world = client.get_world()

    blueprint_library = world.get_blueprint_library()

    # NPC 차량 블루프린트 설정
    bp = blueprint_library.find("vehicle.tesla.model3")
    bp.set_attribute("role_name", "obstacle2")

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
        print("[INFO] NPC 유지 중 (Ctrl+C로 종료 시 차량 제거)")
        while True:
            time.sleep(1.0)

    except KeyboardInterrupt:
        print("[INFO] 사용자 중단 - NPC 제거")

    finally:
        npc.destroy()
        print(f"[INFO] NPC actor 제거: id={npc.id}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="정적 장애물 시나리오 실행")
    parser.add_argument("--host", default="localhost", help="CARLA 서버 IP")
    parser.add_argument("--port", type=int, default=2000, help="CARLA 서버 포트")
    args = parser.parse_args()

    spawn_static_obstacle(host=args.host, port=args.port)
