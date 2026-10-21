# Host RViz review and long-duration localization issue — 2026-10-20

The user ran the C++ numerical simulation with `rviz:=true` in a host terminal and supplied the launch log and an RViz screenshot in the conversation. RViz initialized OpenGL 4.6; the screenshot shows Global Status OK, Robot Status OK, visible robot geometry, and reference/ground-truth/EKF paths. This resolves the earlier inability to open the RViz GUI for this host run. Wheel animation, all expanded display statuses, and the TF display were not independently verified from the static screenshot. The screenshot itself is in the conversation, not stored in this evidence directory.

The teal ground-truth path rotates across successive laps while the orange EKF path remains near the reference. Display initialization is successful, but long-duration true trajectory tracking is not accepted on this evidence.

A separate offline diagnostic used the unchanged numerical model and controller, seed 42, duration 240 s, nominal scenario. The only configuration difference between two runs was IMU yaw-rate bias: the existing default 0.005 rad/s versus zero. This changed no production defaults and no saved 120-trial experiment artifacts. Exact results: [long_drift_diagnostic.json](long_drift_diagnostic.json).

| IMU bias | Final heading estimation error | Last 30 s true tracking RMSE | Last 30 s estimated tracking RMSE |
|---|---:|---:|---:|
| 0.005 rad/s | -1.1423 rad | 2.3157 m | 0.02276 m |
| 0 rad/s | 0.01162 rad | 0.14390 m | 0.02266 m |

The numerical comparison supports unmodeled IMU bias as a major drift mechanism. It does not establish the exact error in the user's live run. Zero-bias configuration is a diagnostic control, not a correction to the robot estimator.

Open release issue: implement and validate a defensible bias handling strategy, such as explicit gyro-bias estimation using wheel-rate/IMU consistency with stated wheel-slip assumptions, or calibration plus an appropriate independent pose/heading reference. Do not fuse simulation ground truth or remove injected bias to manufacture a nominal pass. Preserve the old results; test multi-minute runs and sensor-slip/fault cases after any estimator change. Velocity-only sensing still cannot provide an absolute pose guarantee.
