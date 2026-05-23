# Project C Demo Checklist

Run each scenario for 2-4 trials. For the final autonomous roaming trial, run at
least five minutes without joystick control.

## Required Scenarios

| Scenario | Expected result | Evidence |
|---|---|---|
| Static obstacle | Observe, classify, then dodge toward clearer side | `events.csv` contains `STATE,DODGE`, no `COLLISION` |
| Dynamic obstacle | Person crossing front field triggers stop/observe before motion resumes | `events.csv` contains `DYNAMIC_SEEN` and `dynamic_latency_s` |
| Dead end | Back up, rotate/search, then recover to `DRIVE` | `summary.csv` recovery count and recovery success rate |
| Narrow passage | Move slowly without side scrape | `odom.csv` speed trend and optional `obstacles.csv` side ranges |
| ToF emergency | Very close front ToF reading triggers backup | `events.csv` contains `EMERGENCY` then `STATE,BACKUP` |
| Ablation | Compare LIDAR/ToF control against full fusion | `summary.csv` labels `lidar_tof_ablation` and `full_fusion` |

## Run

```bash
bash tools/run_project_c_full.sh
```

For a built workspace:

```bash
bash tools/run_project_c_safety.sh
```

Watch:

```bash
ros2 topic echo /obstacle_representation
ros2 topic echo /obstacle_avoidance_state
ros2 topic echo /obstacle_trial_summary
ros2 topic echo /cmd_vel
```

Mark physical contacts:

```bash
ros2 topic pub --once /collision_event std_msgs/msg/String "{data: collision}"
```

## Trial Records

Each run creates `record/project_c_trial_<timestamp>/` with:

- `events.csv`: FSM transitions, collisions, dynamic/dead-end/emergency events
- `odom.csv`: odometry samples for path length, coverage, and speed
- `summary.csv`: periodic and final evaluation metrics
- `sensor_fallback.csv`: LIDAR, ToF, OAK depth, and point cloud health/evidence
- `obstacles.csv`: optional obstacle samples when `LOG_OBSTACLE_SAMPLES=true`

## Ablation Commands

```bash
CONTROL_USE_DEPTH=false \
CONTROL_USE_POINTCLOUD=false \
LOG_DEPTH=true \
LOG_POINTCLOUD=true \
TRIAL_LABEL=lidar_tof_ablation \
bash tools/run_project_c_safety.sh
```

```bash
TRIAL_LABEL=full_fusion bash tools/run_project_c_safety.sh
```

## Tuning Order

1. Start with wheels lifted or a clear test area.
2. Tune `emergency_distance` and `front_tof_hard_distance`.
3. Tune `clear_distance`, `stop_distance`, and `depth_obstacle_distance`.
4. Tune `side_guard_distance` and `dodge_clearance` in narrow passages.
5. Tune `dynamic_closing_speed` with a person crossing in front of the robot.
6. Tune `backup_sec`, `rotation_step_deg`, and `rear_stop_distance` for dead ends.
