# Wheel-referenced gyro-bias handling

The host review found rotating true-pose laps while estimated tracking stayed near the reference. The unchanged plant injects a constant **0.005 rad/s** yaw-rate bias. A zero-bias run was a diagnostic control only; it is not used as the correction. Original evidence remains in [the host review](release/host-20261020T182919Z/rviz-review.md) and [diagnostic JSON](release/host-20261020T182919Z/long_drift_diagnostic.json).

## Measurement model and observability

The estimator state is `[x, y, yaw, v, omega, b_g]`. It starts with `b_g=0`, not the simulator's bias. Measurements are:

```text
wheel linear velocity = v         + encoder noise
wheel yaw rate        = omega     + encoder noise
IMU yaw rate          = omega+b_g  + gyro noise
```

The active `[v, omega, b_g]` measurement matrix has rows `[1,0,0]`, `[0,1,0]`, `[0,1,1]` and full rank. Thus a gyro offset is identifiable **relative to an unbiased wheel yaw-rate reference** when both are available. No stationary period is required. A gyro alone observes only `omega+b_g`; it cannot distinguish motion from bias. Absolute initial position and heading remain unobservable. Learning the offset does not relocate a robot after it has accumulated pose error.

Wheel yaw-rate scale, wheel geometry, differential slip, gyro scale/extrinsics and timestamp alignment must be sufficiently correct. If the wheel yaw rate has an unknown offset `b_w`, these sensors identify `b_g-b_w`, not `b_g`. Persistent wheel slip can be absorbed as an apparent gyro offset. Linear wheel slip is also unobservable from these inputs. No reliable slip classification or absolute pose guarantee is claimed. Tests explicitly cover the indistinguishable wheel-offset case rather than claiming it is solved.

Bias follows a zero-mean random walk in the process model; this is a common slowly varying gyro error model ([Kalibr IMU noise model](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model)). Defaults are `process_noise_gyro_bias=1e-8` in `(rad/s)^2/s` and `initial_gyro_bias_variance=0.01` in `(rad/s)^2`. The initial standard deviation is 0.1 rad/s. These are illustrative filter settings, not hardware Allan-variance calibration or a bound on real temperature drift. Existing wheel/IMU measurement covariance settings are unchanged. Process covariance is scaled by elapsed time.

## Conservative pose propagation

The final design learns velocity and bias from rate observations, and propagates pose forward kinematically. Rate-only measurement corrections do not revise the already-integrated `[x,y,yaw]` mean. Their Kalman gain rows are zero, while the full six-state covariance and cross-covariances are retained and updated with the Joseph form using that modified gain. This is a Schmidt/consider-style partial update; [NASA's discussion](https://ntrs.nasa.gov/api/citations/20120010515/downloads/20120010515.pdf) describes retaining consider-state uncertainty while withholding state correction. It is a deliberate conservative, non-optimal update, not proof of nonlinear consistency under slip.

This policy avoids letting wheel/gyro consistency retroactively pull the entire pose through accumulated pose/bias correlations. The first full joint EKF prototype improved nominal bias drift but produced a severe sustained-slip regression; its complete results are retained separately. The conservative policy applies to every bias-enabled trial, not only fault scenarios. It uses no scenario name, simulator pose, known injected bias, or fault window inside the estimator. It cannot undo earlier heading error, and it still fails the non-slip assumptions during slip.

## ROS/API behavior

`core.py` uses `update_imu(raw_yaw_rate)` for the observation `omega+b_g`; direct `update(..., indices, covariance)` remains a direct-state observation. Both ROS sensor callbacks and the offline model use this same core. The ROS node still rejects stale/replayed/nonfinite inputs and stops refreshing output when either required sensor is stale. Controller stop/recovery behavior is retained. During a missing-sensor interval the bias/rate split is not newly identifiable; any estimate changes are conditional on the prior/model. The estimator does not fuse `/ground_truth`, IMU orientation or the reference path.

The 1 Hz `/localization/gyro_bias` JSON exposes estimate, variance, model flags and whether both sensors are fresh. Its covariance is conditional on the model assumptions; the freshness flag is not a calibration or slip-free certificate. `estimate_gyro_bias:=false` is a diagnostic ablation that fixes bias at zero and reproduces the archived five-state filter's state/covariance block. `conservative_pose_update:=false` enables the rejected full-pose-update prototype for investigation. Keep the default enabled for both settings in normal runs.

C++ telemetry additionally includes absolute ROS `stamp_sec`. The long ROS check aligns reference samples against interpolated source-stamped truth and filtered odometry. This timestamp supports evaluation only; no truth is routed back into estimation. The usual telemetry `tracking_error_m` is **estimated-pose** tracking error and must not be presented as true-pose accuracy.

## Reproduction and evidence

Use a new output directory; scripts refuse to replace an existing experiment directory. No physical robot should be connected to the chosen ROS domain.

```bash
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/amr-bias-mpl \
  python3 scripts/validate_gyro_bias.py --output /tmp/amr-bias-paired
OPENBLAS_NUM_THREADS=1 MPLCONFIGDIR=/tmp/amr-bias-mpl \
  python3 scripts/run_experiments.py --output-dir /tmp/amr-bias-comparisons

source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=191 ROS_LOG_DIR=/tmp/amr-bias-long OPENBLAS_NUM_THREADS=1
python3 scripts/long_localization_check.py --seed 42 --duration 240 \
  --output /tmp/amr-bias-live-seed42.json
ros2 topic echo /localization/gyro_bias --once
```

Stop the live demo/check before launching another controller in the same domain. Both offline and live tools report true-pose tracking RMSE separately from estimated-pose tracking RMSE, including the last 30 seconds. Original artifacts are preserved; detailed executed results and remaining release limits are linked from [PREPARATION.md](PREPARATION.md). Script presence or a `passed` stream/stop check is not a declaration of acceptable true-pose tracking in every scenario.
