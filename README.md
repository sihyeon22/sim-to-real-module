# sim-to-real-module

Modular ROS2/Nav2 stack for sim-to-real autonomous driving on a compact mobility platform, with CARLA integration.

## Overview

This repository contains the FR-09 autonomous driving modules split into small ROS2 workspaces.

The stack is organized around two main pipelines:

- `SLAM`: build a map from live sensor data
- `Nav2`: run autonomous navigation on a saved map

Common data flow:

- `sensor_bridge` normalizes simulator or vehicle sensor topics into `/raw/*`
- `perception` converts `/raw/*` into `/scan`, `/odom`, and TF
- `slam` and `nav2` consume the perception output
- `gui` provides RViz configurations for SLAM and Nav2

## Repository Layout

```text
fr09_module/
├── nav2_sensor_bridge_ws/
├── nav2_perception_ws/
├── nav2_slam_ws/
├── nav2_map_ws/
├── nav2_localization_ws/
├── nav2_planning_ws/
├── nav2_control_ws/
├── nav2_vehicle_ws/
└── nav2_gui_ws/
```

Each directory is an independent colcon workspace.
Build artifacts such as `build/`, `install/`, and `log/` are intentionally excluded from Git.

## Modules

| Workspace | Package | Role |
| --- | --- | --- |
| `nav2_sensor_bridge_ws` | `sensor_bridge_pkg` | Normalize simulator or vehicle sensor topics to `/raw/points` and `/raw/imu` |
| `nav2_perception_ws` | `perception_pkg` | Generate `/scan`, `/odom`, and TF from raw sensor topics |
| `nav2_slam_ws` | `slam_pkg` | Run `slam_toolbox` for live map generation |
| `nav2_map_ws` | `map_pkg` | Load saved maps for Nav2 localization |
| `nav2_localization_ws` | `localization_pkg` | Run AMCL-based localization |
| `nav2_planning_ws` | `planning_pkg` | Run Nav2 planner, BT navigator, smoother, and waypoint follower |
| `nav2_control_ws` | `control_pkg` | Run Nav2 controller, behavior server, and velocity smoother |
| `nav2_vehicle_ws` | `vehicle_pkg` | Convert `/cmd_vel` to Ackermann commands and connect to CARLA vehicle control |
| `nav2_gui_ws` | `gui_pkg` | Launch RViz with SLAM or Nav2 configurations |

## Common Interfaces

Sensor bridge output:

- `/raw/lidar`
- `/raw/imu`

Perception output:

- `/scan`
- `/odom`
- `odom -> base_link`
- `base_link -> os_lidar`
- `base_link -> os_imu`

SLAM output:

- `/map`
- `map -> odom`

Vehicle command flow:

```text
/cmd_vel -> cmd_vel_to_ackermann -> /carla/hero/ackermann_cmd -> carla_ackermann_control -> CARLA
```

## Build

Clone the repository:

```bash
cd ~
git clone https://github.com/sihyeon22/sim-to-real-module.git fr09_module
```

Build each workspace from the repository root:

```bash
cd ~/fr09_module/nav2_sensor_bridge_ws
colcon build

cd ~/fr09_module/nav2_perception_ws
colcon build

cd ~/fr09_module/nav2_slam_ws
colcon build

cd ~/fr09_module/nav2_map_ws
colcon build

cd ~/fr09_module/nav2_localization_ws
colcon build

cd ~/fr09_module/nav2_control_ws
colcon build

cd ~/fr09_module/nav2_planning_ws
colcon build

cd ~/fr09_module/nav2_vehicle_ws
colcon build

cd ~/fr09_module/nav2_gui_ws
colcon build
```

Before launching a module, source its workspace:

```bash
source install/setup.bash
```

When multiple modules are used together, source the required workspaces in the same terminal or launch each module from a terminal where its workspace has been sourced.

## Pipeline 1: SLAM

Use this pipeline to generate a map from live sensor data.

Launch order:

1. `sensor_bridge`
2. `perception`
3. `slam`
4. `gui`

### Sensor Bridge

```bash
cd ~/fr09_module/nav2_sensor_bridge_ws
source install/setup.bash
ros2 launch sensor_bridge_pkg sensor_bridge.launch.py
```

### Perception

```bash
cd ~/fr09_module/nav2_perception_ws
source install/setup.bash
ros2 launch perception_pkg perception.launch.py
```

### SLAM

```bash
cd ~/fr09_module/nav2_slam_ws
source install/setup.bash
ros2 launch slam_pkg slam.launch.py
```

### GUI

```bash
cd ~/fr09_module/nav2_gui_ws
source install/setup.bash
ros2 launch gui_pkg gui.launch.py rviz_config:=~/fr09_module/nav2_gui_ws/src/gui_pkg/rviz/slam.rviz
```

### Notes

- The SLAM launch also starts a keyboard controller. Use `w`, `a`, `s`, `d` to drive the vehicle manually and build the map.
- For better map quality, drive **two full laps** — one along the inner course and one along the outer course.
- Once mapping is complete, save the map with the command below. Replace `<map_name>` with the desired map name.

```bash
ros2 run nav2_map_server map_saver_cli -f ~/fr09_module/nav2_map_ws/src/map_pkg/maps/<map_name>
```

## Pipeline 2: Nav2

Use this pipeline for autonomous navigation on a saved map.

Launch order:

1. `map`
2. `vehicle`
3. `sensor_bridge`
4. `perception`
5. `localization`
6. `control`
7. `planning`
8. `gui`

### Map Server

```bash
cd ~/fr09_module/nav2_map_ws
source install/setup.bash
ros2 launch map_pkg map.launch.py
```

To select another map:

```bash
ros2 launch map_pkg map.launch.py map_name:=<map_name>
```

### Vehicle Interface

```bash
cd ~/fr09_module/nav2_vehicle_ws
source install/setup.bash
ros2 launch vehicle_pkg vehicle.launch.py
```

### Sensor Bridge

```bash
cd ~/fr09_module/nav2_sensor_bridge_ws
source install/setup.bash
ros2 launch sensor_bridge_pkg sensor_bridge.launch.py
```

### Perception

```bash
cd ~/fr09_module/nav2_perception_ws
source install/setup.bash
ros2 launch perception_pkg perception.launch.py
```

### Localization

```bash
cd ~/fr09_module/nav2_localization_ws
source install/setup.bash
ros2 launch localization_pkg localization.launch.py
```

### Control

Default: `robot:=fr09`, `level:=2d`

```bash
cd ~/fr09_module/nav2_control_ws
source install/setup.bash
ros2 launch control_pkg control.launch.py
```

3D example:

```bash
ros2 launch control_pkg control.launch.py robot:=fr09 level:=3d
```

### Planning

Default: `robot:=fr09`, `level:=2d`

```bash
cd ~/fr09_module/nav2_planning_ws
source install/setup.bash
ros2 launch planning_pkg planning.launch.py
```

3D example:

```bash
ros2 launch planning_pkg planning.launch.py robot:=fr09 level:=3d
```

### GUI

```bash
cd ~/fr09_module/nav2_gui_ws
source install/setup.bash
ros2 launch gui_pkg gui.launch.py
```

### Notes

- Once RViz opens, click the **2D Pose Estimate** button in the toolbar. Then click on the grid at the vehicle's current position and drag in the direction the vehicle is facing to set the initial pose.
- To set a navigation goal, click the **Nav2 Goal** button in the toolbar, then click on the target position you want and drag to specify the desired heading.

## Recommended Checks

Check TF:

```bash
ros2 run tf2_tools view_frames
```

Expected TF tree:

```text
map -> odom -> base_link -> os_lidar
map -> odom -> base_link -> os_imu
```

Check topics:

```bash
ros2 topic list
```

Important topics:

- `/raw/lidar`
- `/raw/imu`
- `/scan`
- `/odom`
- `/map`
- `/cmd_vel`
- `/carla/hero/ackermann_cmd`

## Notes

- Run either the SLAM pipeline or the Nav2 pipeline depending on the task.
- The SLAM pipeline is for live map generation.
- The Nav2 pipeline is for autonomous driving with a saved static map.
- CARLA bridge and `carla_ackermann_control` are expected to be available from the external CARLA ROS workspace.