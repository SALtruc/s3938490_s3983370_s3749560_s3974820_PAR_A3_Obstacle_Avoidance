# PAR Final Project - ROSbot Obstacle Avoidance

Programming Autonomous Robots final project for the Husarion ROSbot 3 PRO.
This repository runs a reactive obstacle avoidance behaviour using ROS 2 on the
real robot.

The Project C stack uses:

| Component | Input/Output |
|---|---|
| `obstacle_perception` | Fuses S2 LIDAR, OAK-D depth image, OAK-D PointCloud2, and ToF into `/obstacle_representation` |
| `obstacle_avoidance` | Reads `/obstacle_representation` and publishes velocity commands |
| `obstacle_trial_logger` | Writes trial metrics and sensor fallback diagnostics into one trial folder |
| `nav2_collision_monitor` | Optional final safety filter from `/cmd_vel_raw` to `/cmd_vel` |

## One-Command Demo Run

Run this from the repository root on the ROSbot:

```bash
bash tools/run_project_c_full.sh
```

That command restarts the ROSbot sensor snaps, builds the package, checks the
required topics, and launches the full-fusion controller. By default the robot
uses LIDAR, OAK depth, OAK point cloud, and ToF for obstacle perception.

If the robot snaps are already running and the package is already built, use:

```bash
bash tools/run_project_c_safety.sh
```

## First-Time Setup On The ROSbot

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <repo-url> PAR_A3
cd PAR_A3
bash tools/build_project_c.sh
```

The build script sources ROS 2 Jazzy, builds only
`rosbot_obstacle_avoidance`, and avoids accidentally using another workspace.

## Required Robot Topics

Check these before enabling motion:

```bash
ros2 topic list | sort
ros2 topic hz /scan_filtered
ros2 topic hz /camera/depth/image_rect_raw
ros2 topic hz /oak/points
ros2 topic echo --once /range/fl
ros2 topic echo --once /range/fr
ros2 topic echo --once /rosbot_base_controller/odom
ros2 topic echo --once /imu_broadcaster/imu
```

Default topic mapping:

| Purpose | Default topic |
|---|---|
| LIDAR scan | `/scan_filtered` |
| OAK depth image | `/camera/depth/image_rect_raw` |
| OAK point cloud | `/oak/points` |
| Front ToF | `/range/fl,/range/fr` |
| All ToF | `/range/fl,/range/fr,/range/rl,/range/rr` |
| Odometry | `/rosbot_base_controller/odom` through the run script |
| IMU | `/imu_broadcaster/imu` through the run script |
| Velocity command | `/cmd_vel` |

Override topic names with environment variables:

```bash
DEPTH_TOPIC=/camera/camera/depth/image_rect_raw \
POINTCLOUD_TOPIC=/oak/points \
SCAN_TOPIC=/scan_filtered \
bash tools/run_project_c_safety.sh
```

## OAK Point Cloud

If `/oak/points` is missing, start the DepthAI point cloud driver in one
terminal:

```bash
bash tools/start_oak_pointcloud.sh
```

When it prints the ready message, keep that terminal open and start Project C in
another terminal:

```bash
bash tools/run_project_c_safety.sh
```

## Safe Dry Run

Use a dummy command topic before the first live run:

```bash
CMD_VEL_TOPIC=/dummy_cmd_vel bash tools/run_project_c_safety.sh
```

In another terminal:

```bash
ros2 topic echo /obstacle_representation
ros2 topic echo /obstacle_avoidance_state
ros2 topic echo /obstacle_trial_summary
ros2 topic echo /dummy_cmd_vel
```

When the sensor topics and state transitions look correct, stop the dry run and
launch again with the real `/cmd_vel`.

## Useful Run Variants

Full fusion, labelled for report tables:

```bash
TRIAL_LABEL=full_fusion bash tools/run_project_c_safety.sh
```

LIDAR and ToF control, while still logging OAK diagnostics:

```bash
CONTROL_USE_DEPTH=false \
CONTROL_USE_POINTCLOUD=false \
LOG_DEPTH=true \
LOG_POINTCLOUD=true \
TRIAL_LABEL=lidar_tof_ablation \
bash tools/run_project_c_safety.sh
```

LIDAR-only degraded run:

```bash
CONTROL_USE_DEPTH=false \
CONTROL_USE_POINTCLOUD=false \
USE_TOF=false \
LOG_DEPTH=true \
LOG_POINTCLOUD=true \
TRIAL_LABEL=lidar_only \
bash tools/run_project_c_safety.sh
```

Optional Nav2 Collision Monitor shield:

```bash
USE_NAV2_COLLISION_MONITOR=true bash tools/run_project_c_safety.sh
```

Lower speed for a tight demo area:

```bash
MAX_SPEED=0.10 BACKUP_SPEED=0.04 bash tools/run_project_c_safety.sh
```

## Manual ROS 2 Launch

After `source install/setup.bash`, the equivalent manual launch is:

```bash
ros2 launch rosbot_obstacle_avoidance project_c_safety.launch.py \
  scan_topic:=/scan_filtered \
  depth_topic:=/camera/depth/image_rect_raw \
  pointcloud_topic:=/oak/points \
  odom_topic:=/rosbot_base_controller/odom \
  imu_topic:=/imu_broadcaster/imu \
  use_depth:=true \
  use_pointcloud:=true \
  use_tof:=true \
  cmd_vel_topic:=/cmd_vel
```

## Trial Logs For The Report

Every run creates:

```text
record/project_c_trial_<timestamp>/
```

Files in that folder:

| File | Use in report |
|---|---|
| `events.csv` | FSM transitions, dynamic obstacle events, emergency events, manual collision marks |
| `odom.csv` | Path length, coverage area, linear/angular speed samples |
| `summary.csv` | Collision rate, recovery success rate, dynamic response latency |
| `sensor_fallback.csv` | LIDAR/ToF/OAK status, OAK depth front range, point cloud low-obstacle evidence |
| `obstacles.csv` | Optional obstacle samples when `LOG_OBSTACLE_SAMPLES=true` |

Mark a physical collision during a trial:

```bash
ros2 topic pub --once /collision_event std_msgs/msg/String "{data: collision}"
```

## Troubleshooting

If `ros2 pkg prefix rosbot_obstacle_avoidance` points to the wrong workspace,
run:

```bash
cd ~/ros2_ws/src/PAR_A3
bash tools/build_project_c.sh
```

If OAK topics do not appear:

```bash
sudo snap restart husarion-depthai
bash tools/start_oak_pointcloud.sh
```

If ToF topics are missing and the demo must continue:

```bash
USE_TOF=false PROJECT_C_REQUIRE_FULL_FUSION=false bash tools/run_project_c_full.sh
```

If the robot does not move, check the command type:

```bash
ros2 topic info /cmd_vel
```

The default is `geometry_msgs/msg/TwistStamped`. If the robot expects
`geometry_msgs/msg/Twist`, launch with:

```bash
CMD_VEL_STAMPED=false bash tools/run_project_c_safety.sh
```
