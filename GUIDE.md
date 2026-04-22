# fr09_module Guide

## Overview
`fr09_module`은 기존 단일 launch 구조를 기능별 워크스페이스로 나눈 모듈형 ROS 2 구성이다.

현재 파이프라인은 크게 두 가지로 나뉜다.

- `SLAM` 파이프라인
- `Nav2` 파이프라인

공통 원칙은 다음과 같다.

- 센서 입력은 `sensor_bridge`에서 공통 인터페이스로 정규화
- perception은 `/raw/*`를 받아 `/scan`, `/odom`, TF를 생성
- SLAM과 Nav2는 perception 출력만 사용
- GUI는 공용 패키지 `gui_pkg`를 사용하고 RViz config만 목적에 따라 구분

## Module Summary
### `nav2_sensor_bridge_ws`
- 패키지: `sensor_bridge_pkg`
- 역할: 입력 센서 토픽명을 공통 인터페이스로 정규화
- 기본 출력:
  - `/raw/points`
  - `/raw/imu`
- 시뮬레이션에서는 `/carla/hero/lidar`, `/carla/hero/imu`를 입력으로 사용
- 실차에서는 실차에서 발행하는 토픽을 입력으로 변경해서 사용
- 이후 perception, slam, nav2는 모두 이 공통 인터페이스를 기준으로 동작

### `nav2_perception_ws`
- 패키지: `perception_pkg`
- 역할:
  - `/raw/points` -> `/scan`
  - `/raw/imu` + speedometer -> `/odom`
  - TF 발행:
    - `odom -> base_link`
    - `base_link -> os_sensor`
    - `base_link -> os_imu`
- Nav2/SLAM의 공통 전처리 모듈

### `nav2_slam_ws`
- 패키지: `slam_pkg`
- 역할:
  - `/scan`, `/odom`, TF를 입력으로 받아 `slam_toolbox` 실행
  - `/map`, `map -> odom` 생성
- 옵션:
  - `use_teleop:=true(기본값)`이면 별도 터미널에서 `teleop_key` 자동 실행

### `nav2_map_ws`
- 패키지: `map_pkg`
- 역할: 저장된 정적 map을 로드하여 `/map` 발행
- Nav2 localization용 모듈

### `nav2_localization_ws`
- 패키지: `localization_pkg`
- 역할: `amcl` 기반 위치 추정
- 입력:
  - `/scan`
  - `/odom`
  - `/map`
- 출력:
  - `map -> odom`
  - `/amcl_pose`

### `nav2_planning_ws`
- 패키지: `planning_pkg`
- 역할:
  - `planner_server`
  - `bt_navigator`
  - `smoother_server`
  - `waypoint_follower`
- 현재 FR-09 기준 2D/3D 파라미터 파일을 분리해 관리
- 실행 시 인자로 `level:=3d` 넘겨주면 3D 파일로 실행(기본값: 2d)

### `nav2_control_ws`
- 패키지: `control_pkg`
- 역할:
  - `controller_server`
  - `behavior_server`
  - `velocity_smoother`
- `/cmd_vel_nav` -> `/cmd_vel` 흐름을 담당
- 현재 FR-09 기준 2D/3D 파라미터 파일을 분리해 관리
- 실행 시 인자로 `level:=3d` 넘겨주면 3D 파일로 실행(기본값: 2d)

### `nav2_vehicle_ws`
- 패키지: `vehicle_pkg`
- 역할:
  - `/cmd_vel` -> Ackermann command 변환
  - CARLA vehicle control 연동

### `nav2_gui_ws`
- 패키지: `gui_pkg`
- 역할: RViz 실행
- RViz config:
  - Nav2용: `carla.rviz`
  - SLAM용: `slam.rviz`

## Common Interface
모듈 간 주요 공통 인터페이스는 아래와 같다.

- Sensor bridge output
  - `/raw/points`
  - `/raw/imu`
- Perception output
  - `/scan`
  - `/odom`
  - `odom -> base_link`
  - `base_link -> os_sensor`
  - `base_link -> os_imu`
- SLAM output
  - `/map`
  - `map -> odom`

## Build
각 워크스페이스는 독립적으로 빌드 가능하다.

실행 전에는 해당 워크스페이스의 `install/setup.bash`를 source 해야 한다.
여러 워크스페이스를 함께 사용할 경우 필요한 워크스페이스들을 순서대로 source 한다.

## Pipeline 1: SLAM
SLAM 파이프라인은 정적 map 없이 실시간 map을 생성하는 흐름이다.

### 실행 순서
1. `sensor_bridge`
2. `perception`
3. `slam`
4. `gui`

### 1. Sensor Bridge
```bash
cd ~/nav2_sensor_bridge_ws
colcon build --packages-select sensor_bridge_pkg
source install/setup.bash
ros2 launch sensor_bridge_pkg sensor_bridge.launch.py
```

### 2. Perception
```bash
cd ~/nav2_perception_ws
colcon build --packages-select perception_pkg
source install/setup.bash
ros2 launch perception_pkg perception.launch.py
```

### 3. SLAM
```bash
cd ~/nav2_slam_ws
colcon build --packages-select slam_pkg
source install/setup.bash
ros2 launch slam_pkg slam.launch.py
```
-> 다른 터미널에서 teleop_key.py 실행

### 4. GUI
SLAM용 RViz:

```bash
cd ~/nav2_gui_ws
colcon build --packages-select gui_pkg
source install/setup.bash
ros2 launch gui_pkg gui.launch.py rviz_config:=~/nav2_gui_ws/src/gui_pkg/rviz/slam.rviz
```

## Pipeline 2: Nav2

### 실행 순서
1. `map`
2. `vehicle`
3. `sensor_bridge`
4. `perception`
5. `localization`
6. `control`
7. `planning`
8. `gui`

### 1. Map Server
```bash
cd ~/nav2_map_ws
colcon build --packages-select map_pkg
source install/setup.bash
ros2 launch map_pkg map.launch.py
```

필요 시 다른 map 선택:

```bash
ros2 launch map_pkg map.launch.py map_name:=<맵 이름>
```

### 2. Vehicle Interface
```bash
cd ~/nav2_vehicle_ws
colcon build --packages-select vehicle_pkg
source install/setup.bash
ros2 launch vehicle_pkg vehicle.launch.py
```

### 3. Sensor Bridge
```bash
cd ~/nav2_sensor_bridge_ws
colcon build --packages-select sensor_bridge_pkg
source install/setup.bash
ros2 launch sensor_bridge_pkg sensor_bridge.launch.py
```

### 4. Perception
```bash
cd ~/nav2_perception_ws
colcon build --packages-select perception_pkg
source install/setup.bash
ros2 launch perception_pkg perception.launch.py
```

### 5. Localization
```bash
cd ~/nav2_localization_ws
colcon build --packages-select localization_pkg
source install/setup.bash
ros2 launch localization_pkg localization.launch.py
```

### 6. Control
기본값: fr09, 2d

```bash
cd ~/nav2_control_ws
colcon build --packages-select control_pkg
source install/setup.bash
ros2 launch control_pkg control.launch.py
```

MK-Mini 3D 기준 예시:

```bash
cd ~/nav2_control_ws
colcon build --packages-select control_pkg
source install/setup.bash
ros2 launch control_pkg control.launch.py robot:=mkmini level:=3d
```

### 7. Planning
기본값: fr09, 2d

```bash
cd ~/nav2_planning_ws
colcon build --packages-select planning_pkg
source install/setup.bash
ros2 launch planning_pkg planning.launch.py robot:=fr09 level:=2d
```

MK-Mini 3D 기준 예시:

```bash
cd ~/nav2_planning_ws
colcon build --packages-select planning_pkg
source install/setup.bash
ros2 launch planning_pkg planning.launch.py robot:=mkmini level:=3d
```

### 8. GUI
Nav2용 RViz:

```bash
cd ~/nav2_gui_ws
colcon build --packages-select gui_pkg
source install/setup.bash
ros2 launch gui_pkg gui.launch.py
```

## Recommended Checks
실행 후 아래 항목을 확인하면 된다.

- TF
  - `map -> odom -> base_link -> os_sensor`
  - `map -> odom -> base_link -> os_imu`
  - 명령어: `ros2 run tf2_tools view_frames` -> 명령어 실행한 경로에 pdf 파일로 저장

- 토픽
  - `/raw/points`
  - `/raw/imu`
  - `/scan`
  - `/odom`
  - `/map`
  - 명령어: `ros2 topic list`

## Notes
- `slam`과 `nav2`는 동시에 올리는 구성이 아니라 목적에 따라 하나의 파이프라인을 선택해서 실행한다.
- `slam`은 동적 map 생성용이고, `nav2`는 저장된 정적 map 기반 자율주행용이다.
- teleop는 `slam` 파이프라인에서만 바로 붙여 사용하도록 정리되어 있다.
