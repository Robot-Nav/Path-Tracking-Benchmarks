# Gazebo benchmark results

[English README](../README.md) · [中文 README](../README_CN.md) · [中文项目分析](../docs/project-analysis.zh-CN.md)

The two figures use 30 saved Gazebo Classic runs. Each panel overlays the recorded XY odometry and the reference polyline. The reference points were checked for equality across all included runs of each path and condition.

- **Paths** — an 8 m straight and a 10 m long S-curve along the x-axis.
- **No disturbance** — LQR, MPC, open-loop QP, PID, RPP and the geometric tracker on each path.
- **With disturbance** — all nine methods on each path.
- **RMSE** — root mean square of the Euclidean distance from each eligible odometry sample to its nearest point on the reference polyline. The original evaluator excluded samples within its start-progress threshold.
- **Reached** — the evaluator recorded arrival within its goal tolerance and a settled stop. Both disturbed QP trials timed out at about 150 s. Their RMSE includes eligible samples through timeout.
- **Coverage** — one execution per listed combination. APF, DWA and PP have no corresponding no-disturbance entries in this selection. The trajectory chart may thin plotted samples for readability. The RMSE values are the metrics saved by the evaluator.

| Asset | Contents |
| --- | --- |
| [XY tracks](figures/gazebo-xy-tracks.png) | Reference paths, odometry traces and final positions |
| [Cross-track RMSE](figures/gazebo-cross-track-rmse.png) | Per-condition metrics, gaps and timeouts |
| [Metrics CSV](metrics.csv) | Condition, path, controller, RMSE, arrival flag and elapsed time |

These are single executions from historical Gazebo runs. The geometric tracker was recorded in a separate batch on the same reference paths. The [kinematic demo](../examples/kinematic_demo.py) runs a separate synthetic case for checking ROS topics and controller output.
