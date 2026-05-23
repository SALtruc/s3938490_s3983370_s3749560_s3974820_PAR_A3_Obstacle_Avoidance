# rosbot_obstacle_avoidance

ROS 2 package for the Project C reactive obstacle avoidance demo on ROSbot 3
PRO.

## Run

From the repository root:

```bash
bash tools/run_project_c_full.sh
```

For an already built workspace:

```bash
bash tools/run_project_c_safety.sh
```

The launch defaults use LIDAR, OAK depth image, OAK PointCloud2, and ToF. The
trial logger writes `events.csv`, `odom.csv`, `summary.csv`, and
`sensor_fallback.csv` into `record/project_c_trial_<timestamp>/`.

See the repository root `README.md` for deployment steps, topic overrides, OAK
point cloud startup, ablation commands, and troubleshooting.
